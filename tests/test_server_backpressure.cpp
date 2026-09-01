#include "wbr_gps/server.hpp"
#include "wbr_gps/json_io.hpp"
#include "server_test_util.h"

// Server suite 2 of 2: guarantees G2 and G3 -- bounded queues, FIX
// coalescing, NMEA drop-oldest, partial writes, wedged peers and line
// integrity on the wire. Protocol and lifecycle live in
// test_server_protocol.cpp.

// T7-A. enqueue() is private, and the newline guard it enforces cannot be
// reached through the public API because json_io's escape() already strips
// control bytes. That is precisely why the guard needs a direct test: it
// exists so this file's line-integrity invariant does not silently depend on
// another file's behaviour.
namespace wbr_gps {
struct ServerTestAccess {
    static size_t enqueue(wbr_gps::Server& s, int fd, const std::string& line)
    {
        return wbr_gps::Server::enqueue(s.clients_.at(fd), line);
    }
    static uint64_t dropped(const wbr_gps::Server& s, int fd)
    {
        return s.clients_.at(fd).dropped;
    }
};
} // namespace wbr_gps
using wbr_gps::ServerTestAccess;

static void test_embedded_newline_is_refused()
{
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));

    const int c = connect_client(path);
    const int s = srv.accept_client();
    ASSERT_TRUE(srv.flush_client(s));            // drain the HELLO
    ASSERT_EQ(srv.pending_bytes(s), (size_t)0);
    read_line(c);

    const uint64_t before = ServerTestAccess::dropped(srv, s);

    // A line carrying its own newline would give drop_oldest() a false
    // message boundary and splice the wire. It must be refused outright.
    const size_t at = ServerTestAccess::enqueue(
        srv, s, "{\"class\":\"NMEA\",\"raw\":\"$GNGGA,1\nEVIL\",\"dropped\":0}");
    ASSERT_EQ(at, std::string::npos);
    ASSERT_EQ(srv.pending_bytes(s), (size_t)0);  // nothing was queued
    ASSERT_EQ(ServerTestAccess::dropped(srv, s), before + 1);

    // A clean line still queues, and the queue is still exactly one line.
    const size_t ok_at = ServerTestAccess::enqueue(srv, s, "{\"class\":\"PING\"}");
    ASSERT_EQ(ok_at, (size_t)0);
    ASSERT_TRUE(srv.flush_client(s));
    const std::string got = read_line(c);
    ASSERT_STREQ(got, "{\"class\":\"PING\"}");

    ::close(c);
    srv.shutdown();
}

static void test_control_bytes_from_the_device_cannot_break_framing()
{
    // The same property end to end, through the only path an attacker-shaped
    // byte can actually take: a raw NMEA sentence off the serial device.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    set_nonblock_fd(c);
    send_line(c, "{\"op\":\"watch\",\"fix\":false,\"nmea\":true}");
    ASSERT_TRUE(srv.on_client_readable(s, st, 1000));

    srv.broadcast_nmea("$GNGGA,1\n{\"class\":\"FIX\",\"lat\":99}\r\n,2*6F", 1000);
    srv.broadcast_nmea("$GNGGA,ordinary,2*6F", 2000);
    ASSERT_TRUE(srv.flush_client(s));

    std::string got;
    while (drain_available(c, got, 65536) > 0) { }
    std::string rest;
    const std::vector<std::string> lines = split_lines(got, rest);

    // HELLO, the immediate snapshot, and the two NMEA lines: four lines, no
    // more. An injected newline would have produced extra ones.
    ASSERT_EQ(lines.size(), (size_t)4);
    size_t bad = 0;
    for (const std::string& line : lines) {
        std::string cls;
        if (!wbr_gps::json_get_string(line, "class", cls)) ++bad;
        if (line.find("{\"class\":", 1) != std::string::npos) ++bad;
    }
    ASSERT_EQ(bad, (size_t)0);

    ::close(c);
    srv.shutdown();
}

static void test_slow_client_coalesces_and_does_not_grow(void)
{
    // S6. The client never reads. FIX messages must coalesce so the queue
    // stops growing, and the daemon must never block.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    send_line(c, "{\"op\":\"watch\",\"fix\":true,\"nmea\":false}");
    srv.on_client_readable(s, st, 1000);

    // Fill the socket buffer and then some, without ever reading from `c`.
    for (int i = 0; i < 20000; ++i) {
        st.set_serial_ok(i % 2 == 0);
        srv.broadcast_fix(st, 1000 + i);
        srv.flush_client(s);
    }

    // The queue must be bounded. One coalesced FIX is under 1 KB.
    ASSERT_TRUE(srv.pending_bytes(s) < 4096);
    // And the client must still be connected: coalescing, not disconnection.
    ASSERT_EQ(srv.client_count(), (size_t)1);

    ::close(c);
    srv.shutdown();
}

static void test_nmea_overflow_drops_and_counts()
{
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    send_line(c, "{\"op\":\"watch\",\"fix\":false,\"nmea\":true}");
    srv.on_client_readable(s, st, 1000);

    for (int i = 0; i < 20000; ++i) {
        srv.broadcast_nmea("$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,,*6F", i);
        srv.flush_client(s);
    }

    // Bounded queue, client retained.
    ASSERT_TRUE(srv.pending_bytes(s) <= wbr_gps::Server::kMaxQueueBytes);
    ASSERT_EQ(srv.client_count(), (size_t)1);

    ::close(c);
    srv.shutdown();
}

static void test_wire_lines_are_never_spliced()
{
    // THE property this daemon exists to provide. Under overflow, with the
    // socket buffer full and partial writes happening constantly, every line
    // the peer receives must still be a whole, well-formed message. A queue
    // edit that touches bytes already handed to the kernel would splice two
    // messages together -- exactly the corruption four concurrent readers of
    // /dev/ttyACM0 produce today, reintroduced one layer up.
    const std::string kRaw =
        "$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F";

    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;
    st.apply_nmea(kRaw, 1000);

    const int c = connect_client(path);
    ASSERT_TRUE(c >= 0);
    const int s = srv.accept_client();
    ASSERT_TRUE(s >= 0);
    set_nonblock_fd(c);
    shrink_buffers(s, c);

    send_line(c, "{\"op\":\"watch\",\"fix\":true,\"nmea\":true}");
    ASSERT_TRUE(srv.on_client_readable(s, st, 1000));

    std::string got;
    bool cap_held = true;
    bool peer_ok  = true;
    for (int i = 0; i < 20000; ++i) {
        srv.broadcast_nmea(kRaw, i);
        srv.broadcast_fix(st, 1000 + i);
        if (srv.pending_bytes(s) > wbr_gps::Server::kMaxQueueBytes) cap_held = false;
        if (!srv.flush_client(s)) peer_ok = false;
        // Read a trickle so the socket keeps making partial progress.
        if (i % 3 == 0) drain_available(c, got, 29);
    }

    // Stop producing and drain both sides cleanly.
    for (int i = 0; i < 200000 && srv.pending_bytes(s) > 0; ++i) {
        srv.flush_client(s);
        drain_available(c, got, 4096);
    }
    srv.drop_client(s);                  // closes the server end -> peer sees EOF
    while (drain_available(c, got, 4096) > 0) { }

    // The daemon must have stayed bounded the whole way through, and
    // never once decided the peer had gone.
    ASSERT_TRUE(cap_held);
    ASSERT_TRUE(peer_ok);

    std::string rest;
    const std::vector<std::string> lines = split_lines(got, rest);
    ASSERT_TRUE(lines.size() > 100);     // the run must actually have transferred data

    size_t bad_class = 0, bad_raw = 0, spliced = 0, nmea_seen = 0;
    uint64_t max_dropped = 0;
    for (const std::string& line : lines) {
        std::string cls;
        if (!wbr_gps::json_get_string(line, "class", cls)) { ++bad_class; continue; }
        // A spliced line carries two message openings.
        if (line.find("{\"class\":", 1) != std::string::npos) ++spliced;
        if (cls != "NMEA") continue;
        ++nmea_seen;
        std::string raw;
        if (!wbr_gps::json_get_string(line, "raw", raw) || raw != kRaw) ++bad_raw;
        int64_t d = 0;
        if (wbr_gps::json_get_int64(line, "dropped", d) && (uint64_t)d > max_dropped)
            max_dropped = (uint64_t)d;
    }
    ASSERT_EQ(bad_class, (size_t)0);
    ASSERT_EQ(spliced, (size_t)0);
    ASSERT_EQ(bad_raw, (size_t)0);
    ASSERT_TRUE(nmea_seen > 0);
    // The overflow path must actually have been exercised, or this test
    // proves nothing about drop-oldest.
    ASSERT_TRUE(max_dropped > 0);
    // The trailing fragment is a legitimately truncated tail only if the
    // stream ended mid-line; after a clean drain there must be none.
    ASSERT_EQ(rest.size(), (size_t)0);

    ::close(c);
    srv.shutdown();
}

static void test_queue_cap_holds_across_partial_writes()
{
    // The bound in guarantee G2 must be a bound, not an approximation:
    // dropping one line and appending one unconditionally lets the queue
    // creep past the cap whenever the head of the queue is a part-sent line.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    set_nonblock_fd(c);
    shrink_buffers(s, c);

    send_line(c, "{\"op\":\"watch\",\"fix\":false,\"nmea\":true}");
    srv.on_client_readable(s, st, 1000);

    size_t worst = 0;
    for (int i = 0; i < 20000; ++i) {
        srv.broadcast_nmea("$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,,*6F", i);
        srv.flush_client(s);
        if (srv.pending_bytes(s) > worst) worst = srv.pending_bytes(s);
    }
    if (worst > wbr_gps::Server::kMaxQueueBytes)
        fprintf(stderr, "  queue peaked at %zu, cap is %zu\n",
                worst, wbr_gps::Server::kMaxQueueBytes);
    ASSERT_TRUE(worst <= wbr_gps::Server::kMaxQueueBytes);

    ::close(c);
    srv.shutdown();
}

static void test_multiple_clients_are_independent()
{
    // G2 stated positively: a wedged client must not affect a healthy one.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;
    st.apply_nmea("$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F", 1000);

    const int slow_c = connect_client(path);
    const int slow_s = srv.accept_client();
    const int fast_c = connect_client(path);
    const int fast_s = srv.accept_client();
    ASSERT_EQ(srv.client_count(), (size_t)2);
    set_nonblock_fd(fast_c);
    shrink_buffers(slow_s, slow_c);

    send_line(slow_c, "{\"op\":\"watch\",\"fix\":true,\"nmea\":true}");
    srv.on_client_readable(slow_s, st, 1000);
    send_line(fast_c, "{\"op\":\"watch\",\"fix\":true,\"nmea\":true}");
    srv.on_client_readable(fast_s, st, 1000);

    std::string fast_got;
    bool slow_ok = true, fast_ok = true;
    for (int i = 0; i < 5000; ++i) {     // slow_c never reads
        srv.broadcast_nmea("$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,,*6F", i);
        srv.broadcast_fix(st, 1000 + i);
        if (!srv.flush_client(slow_s)) slow_ok = false;
        if (!srv.flush_client(fast_s)) fast_ok = false;
        drain_available(fast_c, fast_got, 65536);
    }
    ASSERT_TRUE(slow_ok);                // the wedged peer is still a peer
    ASSERT_TRUE(fast_ok);
    // The fast client kept up: nothing left queued for it, and nothing was
    // dropped from its stream.
    ASSERT_EQ(srv.pending_bytes(fast_s), (size_t)0);
    ASSERT_TRUE(srv.pending_bytes(slow_s) > 0);
    ASSERT_EQ(srv.client_count(), (size_t)2);

    std::string rest;
    const std::vector<std::string> lines = split_lines(fast_got, rest);
    size_t bad = 0, dropped_seen = 0;
    for (const std::string& line : lines) {
        std::string cls;
        if (!wbr_gps::json_get_string(line, "class", cls)) { ++bad; continue; }
        int64_t d = 0;
        if (cls == "NMEA" && wbr_gps::json_get_int64(line, "dropped", d) && d != 0)
            ++dropped_seen;
    }
    ASSERT_EQ(bad, (size_t)0);
    ASSERT_EQ(dropped_seen, (size_t)0);  // the slow peer cost the fast one nothing

    ::close(slow_c);
    ::close(fast_c);
    srv.shutdown();
}

static void test_one_client_cannot_monopolise_the_loop()
{
    // G3. A peer that writes without pause must not keep on_client_readable()
    // spinning until it decides to stop: the call has to yield so the epoll
    // loop can service the serial source and every other client. Reading
    // "until EAGAIN" is unbounded work when the writer is faster than us.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    ASSERT_TRUE(s >= 0);

    // ~40 KB of back-to-back valid requests, well past one event's budget.
    const std::string req = "{\"op\":\"get\",\"pad\":\"" + std::string(1000, 'x') + "\"}";
    for (int i = 0; i < 40; ++i) send_line(c, req);

    ASSERT_TRUE(srv.on_client_readable(s, st, 1000));
    const size_t after_one = srv.pending_bytes(s);
    ASSERT_TRUE(after_one > 0);

    ASSERT_TRUE(srv.on_client_readable(s, st, 1000));
    const size_t after_two = srv.pending_bytes(s);

    // The first call must have handled only part of the backlog, leaving the
    // rest for the next pass of the loop.
    ASSERT_TRUE(after_two > after_one);

    ::close(c);
    srv.shutdown();
}

static void test_write_to_closed_peer_does_not_signal()
{
    // MSG_NOSIGNAL, proved rather than asserted: without it this send() would
    // raise SIGPIPE and kill the whole daemon. If the flag were missing the
    // test binary would die here rather than fail an assertion.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    send_line(c, "{\"op\":\"watch\",\"fix\":true,\"nmea\":false}");
    ASSERT_TRUE(srv.on_client_readable(s, st, 1000));
    ::close(c);                          // peer vanishes with data still queued

    // Keep pushing at a socket whose peer is gone. The first send may still
    // succeed into the orphaned buffer; a later one must report EPIPE.
    bool reported_gone = false;
    for (int i = 0; i < 50 && !reported_gone; ++i) {
        srv.broadcast_fix(st, 1000 + i);
        if (!srv.flush_client(s)) reported_gone = true;
    }
    ASSERT_TRUE(reported_gone);
    srv.drop_client(s);
    ASSERT_EQ(srv.client_count(), (size_t)0);

    srv.shutdown();
}

static void test_oversized_message_is_refused_not_queued()
{
    // A single message larger than the whole queue budget cannot be trimmed
    // into the budget by dropping others. It must be refused outright rather
    // than allowed to blow the bound.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    srv.flush_client(s);
    read_line(c);                        // HELLO
    send_line(c, "{\"op\":\"watch\",\"fix\":false,\"nmea\":true}");
    srv.on_client_readable(s, st, 1000);
    srv.flush_client(s);
    read_line(c);                        // immediate snapshot
    ASSERT_EQ(srv.pending_bytes(s), (size_t)0);

    srv.broadcast_nmea(std::string(wbr_gps::Server::kMaxQueueBytes + 1000, 'A'), 1000);
    ASSERT_EQ(srv.pending_bytes(s), (size_t)0);      // refused, not queued

    // A normal sentence afterwards still flows, and reports the drop.
    srv.broadcast_nmea("$GNGGA,1,2*6F", 2000);
    ASSERT_TRUE(srv.pending_bytes(s) > 0);
    srv.flush_client(s);
    const std::string line = read_line(c);
    int64_t dropped = 0;
    ASSERT_TRUE(wbr_gps::json_get_int64(line, "dropped", dropped));
    ASSERT_EQ(dropped, (int64_t)1);

    ::close(c);
    srv.shutdown();
}

static void run_tests()
{
    test_slow_client_coalesces_and_does_not_grow();
    test_nmea_overflow_drops_and_counts();
    test_wire_lines_are_never_spliced();
    test_queue_cap_holds_across_partial_writes();
    test_multiple_clients_are_independent();
    test_one_client_cannot_monopolise_the_loop();
    test_write_to_closed_peer_does_not_signal();
    test_oversized_message_is_refused_not_queued();
    test_embedded_newline_is_refused();
    test_control_bytes_from_the_device_cannot_break_framing();
    cleanup_tmp_dirs();
}

TEST_MAIN
