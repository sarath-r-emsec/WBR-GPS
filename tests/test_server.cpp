#include "wbr_gps/server.hpp"
#include "wbr_gps/json_io.hpp"
#include "test_util.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

// Every socket lives in its own private directory so tests cannot collide.
// The directories are removed together at the end of the run, matching the
// cleanup convention in test_device_presence.cpp.
static std::vector<std::string> g_tmp_dirs;

static std::string tmp_sock()
{
    char tmpl[] = "/tmp/wbrgps_sock_XXXXXX";
    const char* d = mkdtemp(tmpl);
    if (d == nullptr) { fprintf(stderr, "mkdtemp failed\n"); abort(); }
    g_tmp_dirs.push_back(d);
    return std::string(d) + "/gpsd.sock";
}

static void cleanup_tmp_dirs()
{
    for (const std::string& d : g_tmp_dirs) {
        ::unlink((d + "/gpsd.sock").c_str());
        ::rmdir(d.c_str());
    }
    g_tmp_dirs.clear();
}

static int connect_client(const std::string& path)
{
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof addr.sun_path, "%s", path.c_str());
    if (::connect(fd, (struct sockaddr*)&addr, sizeof addr) != 0) { ::close(fd); return -1; }
    return fd;
}

static std::string read_line(int fd)
{
    std::string out;
    char c;
    while (::read(fd, &c, 1) == 1) {
        if (c == '\n') break;
        out += c;
    }
    return out;
}

static void send_line(int fd, const std::string& s)
{
    const std::string framed = s + "\n";
    ssize_t rc = ::write(fd, framed.data(), framed.size());
    (void)rc;
}


// --- helpers for the backpressure tests ---------------------------------

static void set_nonblock_fd(int fd)
{
    const int fl = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

// Shrink the kernel socket buffers so send() is forced into partial writes
// after only a few hundred bytes. Without this the 200 KB default buffer
// hides every partial-write path in the server.
static void shrink_buffers(int server_fd, int client_fd)
{
    int sz = 2048;
    ::setsockopt(server_fd, SOL_SOCKET, SO_SNDBUF, &sz, sizeof sz);
    ::setsockopt(client_fd, SOL_SOCKET, SO_RCVBUF, &sz, sizeof sz);
}

// Read whatever is available right now. Returns bytes appended; -1 on EOF.
static ssize_t drain_available(int fd, std::string& sink, size_t max_bytes)
{
    char buf[512];
    size_t got = 0;
    while (got < max_bytes) {
        const size_t want = std::min(sizeof buf, max_bytes - got);
        const ssize_t n = ::read(fd, buf, want);
        if (n > 0) { sink.append(buf, (size_t)n); got += (size_t)n; continue; }
        if (n == 0) return -1;                                   // EOF
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        if (errno == EINTR) continue;
        return -1;
    }
    return (ssize_t)got;
}

static void test_hello_on_connect()
{
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));

    const int c = connect_client(path);
    ASSERT_TRUE(c >= 0);
    const int s = srv.accept_client();
    ASSERT_TRUE(s >= 0);
    ASSERT_TRUE(srv.flush_client(s));

    const std::string hello = read_line(c);
    std::string cls;
    ASSERT_TRUE(wbr_gps::json_get_string(hello, "class", cls));
    ASSERT_STREQ(cls, "HELLO");
    ASSERT_TRUE(hello.find("\"proto\":1") != std::string::npos);

    ::close(c);
    srv.shutdown();
}

static void test_get_returns_one_snapshot()
{
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));

    wbr_gps::StateStore st;
    st.apply_nmea("$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F", 1000);

    const int c = connect_client(path);
    const int s = srv.accept_client();
    srv.flush_client(s);
    read_line(c);                       // HELLO

    send_line(c, "{\"op\":\"get\"}");
    ASSERT_TRUE(srv.on_client_readable(s, st, 1480));
    srv.flush_client(s);

    wbr_gps::Snapshot got;
    ASSERT_TRUE(wbr_gps::snapshot_from_json(read_line(c), got));
    ASSERT_TRUE(got.has_fix);
    ASSERT_NEAR(got.lat, 13.0028260, 1e-6);
    ASSERT_EQ(got.fix_age_ms, (int64_t)480);

    ::close(c);
    srv.shutdown();
}

static void test_get_client_receives_no_pushes()
{
    // A client that only sent "get" must not be subscribed. Otherwise the
    // Django badge, which connects and closes, would accumulate pushes.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    srv.flush_client(s);
    read_line(c);                       // HELLO

    send_line(c, "{\"op\":\"get\"}");
    srv.on_client_readable(s, st, 1000);
    srv.flush_client(s);
    read_line(c);                       // the one FIX

    srv.broadcast_fix(st, 2000);
    ASSERT_EQ(srv.pending_bytes(s), (size_t)0);

    ::close(c);
    srv.shutdown();
}

static void test_watch_receives_pushes()
{
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    srv.flush_client(s);
    read_line(c);                       // HELLO

    send_line(c, "{\"op\":\"watch\",\"fix\":true,\"nmea\":false}");
    srv.on_client_readable(s, st, 1000);
    srv.flush_client(s);
    read_line(c);                       // immediate snapshot on subscribe (S5)

    st.apply_nmea("$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F", 2000);
    srv.broadcast_fix(st, 2000);
    srv.flush_client(s);

    wbr_gps::Snapshot got;
    ASSERT_TRUE(wbr_gps::snapshot_from_json(read_line(c), got));
    ASSERT_TRUE(got.has_fix);

    ::close(c);
    srv.shutdown();
}

static void test_nmea_subscription_is_opt_in()
{
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    srv.flush_client(s);
    read_line(c);

    send_line(c, "{\"op\":\"watch\",\"fix\":false,\"nmea\":false}");
    srv.on_client_readable(s, st, 1000);
    srv.flush_client(s);
    read_line(c);                       // immediate snapshot

    srv.broadcast_nmea("$GNGGA,1,2*6F", 1000);
    ASSERT_EQ(srv.pending_bytes(s), (size_t)0);

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

static void test_abrupt_disconnect_is_cleaned_up()
{
    // S7. Killing the peer must free the client slot, not leak it.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    ASSERT_EQ(srv.client_count(), (size_t)1);

    ::close(c);                          // peer vanishes
    // A read now reports EOF; the server must drop the client.
    ASSERT_FALSE(srv.on_client_readable(s, st, 1000));
    srv.drop_client(s);
    ASSERT_EQ(srv.client_count(), (size_t)0);

    srv.shutdown();
}

static void test_malformed_input_gets_error_not_crash()
{
    // S10.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    srv.flush_client(s);
    read_line(c);                        // HELLO

    send_line(c, "this is not json");
    ASSERT_TRUE(srv.on_client_readable(s, st, 1000));
    srv.flush_client(s);
    std::string cls;
    ASSERT_TRUE(wbr_gps::json_get_string(read_line(c), "class", cls));
    ASSERT_STREQ(cls, "ERROR");

    // An oversized line must be refused, and the client dropped rather than
    // allowed to consume unbounded memory.
    send_line(c, std::string(wbr_gps::Server::kMaxRequestBytes + 100, 'A'));
    srv.on_client_readable(s, st, 1000);

    ::close(c);
    srv.shutdown();
}

static void test_singleton_lock()
{
    // S14. Two daemons must not both run.
    char tmpl[] = "/tmp/wbrgps_lock_XXXXXX";
    const char* made = mkdtemp(tmpl);
    if (made == nullptr) { fprintf(stderr, "mkdtemp failed\n"); abort(); }
    const std::string dir = made;
    const std::string lock = dir + "/wbr-gpsd.pid";

    wbr_gps::Server a, b;
    ASSERT_TRUE(a.acquire_singleton(lock));
    ASSERT_FALSE(b.acquire_singleton(lock));   // loser
    a.shutdown();
    // After the winner exits, the lock must be reusable.
    wbr_gps::Server c;
    ASSERT_TRUE(c.acquire_singleton(lock));
    c.shutdown();

    ::unlink(lock.c_str());
    ::rmdir(dir.c_str());
}

static void test_stale_socket_is_replaced()
{
    // S15. A leftover socket file from an unclean kill must not block startup,
    // but a live daemon's socket must be respected.
    const std::string path = tmp_sock();
    {
        wbr_gps::Server first;
        ASSERT_TRUE(first.listen_on(path, nullptr, 0600));
        // While `first` is alive, a second bind must refuse.
        wbr_gps::Server second;
        ASSERT_FALSE(second.listen_on(path, nullptr, 0600));
        first.shutdown();
    }
    // The file may remain; a fresh server must reclaim it.
    wbr_gps::Server third;
    ASSERT_TRUE(third.listen_on(path, nullptr, 0600));
    third.shutdown();
}


// ---------------------------------------------------------------------------
// Adversarial tests added by the implementer. These pin the two properties
// the brief asked to be verified rather than assumed.
// ---------------------------------------------------------------------------

// Split a byte stream into complete newline-terminated lines. A trailing
// fragment (no newline yet) is left in `rest`.
static std::vector<std::string> split_lines(const std::string& s, std::string& rest)
{
    std::vector<std::string> out;
    size_t start = 0;
    for (;;) {
        const size_t nl = s.find('\n', start);
        if (nl == std::string::npos) break;
        out.push_back(s.substr(start, nl - start));
        start = nl + 1;
    }
    rest = s.substr(start);
    return out;
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

static void test_truly_stale_socket_is_reclaimed()
{
    // S15 for real: simulate SIGKILL by leaving a bound socket file behind
    // with nothing listening on it. shutdown() unlinks, so the brief's own
    // version of this test never exercises the stale path at all.
    const std::string path = tmp_sock();
    {
        const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
        ASSERT_TRUE(fd >= 0);
        struct sockaddr_un addr {};
        addr.sun_family = AF_UNIX;
        snprintf(addr.sun_path, sizeof addr.sun_path, "%s", path.c_str());
        ASSERT_EQ(::bind(fd, (struct sockaddr*)&addr, sizeof addr), 0);
        ::listen(fd, 4);
        ::close(fd);                     // file remains, nobody is listening
    }
    ASSERT_EQ(::access(path.c_str(), F_OK), 0);

    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    const int c = connect_client(path);
    ASSERT_TRUE(c >= 0);                 // the new socket really is usable
    ::close(c);
    srv.shutdown();
}

static void test_live_socket_with_full_backlog_is_not_unlinked()
{
    // socket_is_live() probes with connect(). If the live daemon's accept
    // backlog is full, connect() must not be read as "dead" -- unlinking a
    // live daemon's socket is precisely the two-daemon failure this guards.
    // It must also never block the probing process.
    const std::string path = tmp_sock();
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    ASSERT_TRUE(fd >= 0);
    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof addr.sun_path, "%s", path.c_str());
    ASSERT_EQ(::bind(fd, (struct sockaddr*)&addr, sizeof addr), 0);
    ASSERT_EQ(::listen(fd, 1), 0);       // backlog 1, never accept

    std::vector<int> pending;
    for (int i = 0; i < 8; ++i) {        // stuff the backlog
        const int p = ::socket(AF_UNIX, SOCK_STREAM, 0);
        set_nonblock_fd(p);
        ::connect(p, (struct sockaddr*)&addr, sizeof addr);
        pending.push_back(p);
    }

    wbr_gps::Server srv;
    ASSERT_FALSE(srv.listen_on(path, nullptr, 0600));
    // The live listener's socket file must survive the refused start.
    ASSERT_EQ(::access(path.c_str(), F_OK), 0);

    for (int p : pending) ::close(p);
    ::close(fd);
    ::unlink(path.c_str());
}

static void test_oversized_request_reports_peer_gone()
{
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    send_line(c, std::string(wbr_gps::Server::kMaxRequestBytes + 100, 'A'));
    ASSERT_FALSE(srv.on_client_readable(s, st, 1000));
    srv.drop_client(s);
    ASSERT_EQ(srv.client_count(), (size_t)0);

    ::close(c);
    srv.shutdown();
}

static void test_unwatch_stops_pushes()
{
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    srv.flush_client(s);
    read_line(c);                        // HELLO

    send_line(c, "{\"op\":\"watch\",\"fix\":true,\"nmea\":true}");
    srv.on_client_readable(s, st, 1000);
    srv.flush_client(s);
    read_line(c);                        // immediate snapshot
    ASSERT_EQ(srv.pending_bytes(s), (size_t)0);

    srv.broadcast_fix(st, 2000);
    ASSERT_TRUE(srv.pending_bytes(s) > 0);
    srv.flush_client(s);
    read_line(c);

    send_line(c, "{\"op\":\"unwatch\"}");
    ASSERT_TRUE(srv.on_client_readable(s, st, 2500));
    ASSERT_EQ(srv.pending_bytes(s), (size_t)0);   // unwatch is silent

    srv.broadcast_fix(st, 3000);
    srv.broadcast_nmea("$GNGGA,1,2*6F", 3000);
    ASSERT_EQ(srv.pending_bytes(s), (size_t)0);

    ::close(c);
    srv.shutdown();
}

static void test_unknown_op_gets_error()
{
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    srv.flush_client(s);
    read_line(c);                        // HELLO

    send_line(c, "{\"op\":\"launch_missiles\"}");
    ASSERT_TRUE(srv.on_client_readable(s, st, 1000));
    srv.flush_client(s);
    std::string cls;
    ASSERT_TRUE(wbr_gps::json_get_string(read_line(c), "class", cls));
    ASSERT_STREQ(cls, "ERROR");

    ::close(c);
    srv.shutdown();
}

static void test_wants_write_and_unknown_fd()
{
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    // Unknown fds must be inert, not crash: the epoll loop will ask about
    // fds it has already dropped.
    ASSERT_FALSE(srv.wants_write(4242));
    ASSERT_EQ(srv.pending_bytes(4242), (size_t)0);
    ASSERT_FALSE(srv.flush_client(4242));
    ASSERT_FALSE(srv.on_client_readable(4242, st, 1000));
    srv.drop_client(4242);               // must not close an unrelated fd

    const int c = connect_client(path);
    const int s = srv.accept_client();
    ASSERT_TRUE(srv.wants_write(s));     // HELLO is queued
    ASSERT_TRUE(srv.flush_client(s));
    ASSERT_FALSE(srv.wants_write(s));    // and gone once written

    ASSERT_TRUE(srv.listen_fd() >= 0);
    ::close(c);
    srv.shutdown();
}

static void test_overlong_socket_path_is_refused()
{
    // sun_path is 108 bytes. A silently truncated path would make the daemon
    // listen somewhere nobody is connecting to.
    const std::string path = "/tmp/" + std::string(200, 'p') + ".sock";
    wbr_gps::Server srv;
    ASSERT_FALSE(srv.listen_on(path, nullptr, 0600));
    ASSERT_EQ(srv.listen_fd(), -1);
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

static void test_headless_flood_without_newline_is_dropped()
{
    // A client that opens a connection and streams bytes forever without ever
    // terminating a line must not grow the daemon's input buffer without end.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    const std::string junk(6000, 'A');   // no newline anywhere
    ssize_t rc = ::write(c, junk.data(), junk.size());
    (void)rc;

    ASSERT_FALSE(srv.on_client_readable(s, st, 1000));
    srv.drop_client(s);
    ASSERT_EQ(srv.client_count(), (size_t)0);

    ::close(c);
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

static void test_unknown_group_is_reported_not_fatal()
{
    // A misconfigured group must be loud but must not stop the daemon: the
    // socket still exists, just without the intended group ownership.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, "wbr_gps_no_such_group_xyz", 0600));
    const int c = connect_client(path);
    ASSERT_TRUE(c >= 0);
    ::close(c);
    srv.shutdown();
}

static void run_tests()
{
    test_hello_on_connect();
    test_get_returns_one_snapshot();
    test_get_client_receives_no_pushes();
    test_watch_receives_pushes();
    test_nmea_subscription_is_opt_in();
    test_slow_client_coalesces_and_does_not_grow();
    test_nmea_overflow_drops_and_counts();
    test_abrupt_disconnect_is_cleaned_up();
    test_malformed_input_gets_error_not_crash();
    test_singleton_lock();
    test_stale_socket_is_replaced();
    test_wire_lines_are_never_spliced();
    test_queue_cap_holds_across_partial_writes();
    test_truly_stale_socket_is_reclaimed();
    test_live_socket_with_full_backlog_is_not_unlinked();
    test_oversized_request_reports_peer_gone();
    test_unwatch_stops_pushes();
    test_unknown_op_gets_error();
    test_wants_write_and_unknown_fd();
    test_overlong_socket_path_is_refused();
    test_multiple_clients_are_independent();
    test_one_client_cannot_monopolise_the_loop();
    test_write_to_closed_peer_does_not_signal();
    test_headless_flood_without_newline_is_dropped();
    test_oversized_message_is_refused_not_queued();
    test_unknown_group_is_reported_not_fatal();
    cleanup_tmp_dirs();
}

TEST_MAIN
