#include "wbr_gps/server.hpp"
#include "wbr_gps/json_io.hpp"
#include "server_test_util.h"

// Server suite 1 of 2: the request/response protocol and the process
// lifecycle -- HELLO, get/watch/unwatch, malformed input, client teardown,
// the flock singleton and socket binding. Queueing and overflow behaviour
// lives in test_server_backpressure.cpp.

static void test_singleton_rejects_a_second_path()
{
    // T7-C. acquire_singleton() short-circuits when it already holds a lock.
    // That is right for a repeat call on the same path and wrong for a
    // different one: it would report a lock this Server never took.
    char tmpl_a[] = "/tmp/wbrgps_lock_XXXXXX";
    const char* made_a = mkdtemp(tmpl_a);
    if (made_a == nullptr) { fprintf(stderr, "mkdtemp failed\n"); abort(); }
    const std::string dir_a = made_a;
    char tmpl_b[] = "/tmp/wbrgps_lock_XXXXXX";
    const char* made_b = mkdtemp(tmpl_b);
    if (made_b == nullptr) { fprintf(stderr, "mkdtemp failed\n"); abort(); }
    const std::string dir_b = made_b;

    const std::string lock_a = dir_a + "/wbr-gpsd.pid";
    const std::string lock_b = dir_b + "/wbr-gpsd.pid";

    wbr_gps::Server a;
    ASSERT_TRUE(a.acquire_singleton(lock_a));
    ASSERT_TRUE(a.acquire_singleton(lock_a));    // same path: idempotent
    ASSERT_FALSE(a.acquire_singleton(lock_b));   // different path: not ours

    // The decisive check: lock_b must still be free, proving `a` really did
    // not take it rather than merely declining to say so.
    wbr_gps::Server b;
    ASSERT_TRUE(b.acquire_singleton(lock_b));
    b.shutdown();
    a.shutdown();

    ::unlink(lock_a.c_str()); ::rmdir(dir_a.c_str());
    ::unlink(lock_b.c_str()); ::rmdir(dir_b.c_str());
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

static void test_overlong_socket_path_is_refused()
{
    // sun_path is 108 bytes. A silently truncated path would make the daemon
    // listen somewhere nobody is connecting to.
    const std::string path = "/tmp/" + std::string(200, 'p') + ".sock";
    wbr_gps::Server srv;
    ASSERT_FALSE(srv.listen_on(path, nullptr, 0600));
    ASSERT_EQ(srv.listen_fd(), -1);
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
    test_unwatch_stops_pushes();
    test_unknown_op_gets_error();
    test_malformed_input_gets_error_not_crash();
    test_oversized_request_reports_peer_gone();
    test_headless_flood_without_newline_is_dropped();
    test_abrupt_disconnect_is_cleaned_up();
    test_wants_write_and_unknown_fd();
    test_singleton_lock();
    test_stale_socket_is_replaced();
    test_truly_stale_socket_is_reclaimed();
    test_live_socket_with_full_backlog_is_not_unlinked();
    test_overlong_socket_path_is_refused();
    test_unknown_group_is_reported_not_fatal();
    test_singleton_rejects_a_second_path();
    cleanup_tmp_dirs();
}

TEST_MAIN
