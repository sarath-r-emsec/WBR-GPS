// Scenarios S1-S10: many programs talking to one live daemon at once.
//
// This is half the answer to the original request -- "test all possible
// scenarios with multiple programs trying to connect." Fifty simultaneous
// watchers, three request modes at once, a consumer that wedges, one that is
// killed mid-write, two hundred connections in a row. The other half, where
// the device or the daemon disappears underneath the fleet, is in
// test_concurrency_lifecycle.cpp.

#include "concurrency_util.h"

// ---------- scenarios -------------------------------------------------------

static void S1_single_get()
{
    PtyPair& pty = new_pty(); ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);
    pty.emit(kGGA);

    wbr_gps::Snapshot s;
    ASSERT_TRUE(wait_until([&] { s = wbr_gps::get_once(d.sock, 1000); return s.has_fix; }, 3000));
    ASSERT_TRUE(s.service_ok);
    ASSERT_TRUE(s.has_fix);
    ASSERT_NEAR(s.lat, 13.0028260, 1e-6);
    d.stop();
}

static void S2_ten_simultaneous_gets_agree()
{
    PtyPair& pty = new_pty(); ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);
    pty.emit(kGGA);
    ASSERT_TRUE(wait_until([&] { return wbr_gps::get_once(d.sock, 1000).has_fix; }, 3000));

    std::vector<wbr_gps::Snapshot> results(10);
    std::vector<std::thread> ts;
    for (int i = 0; i < 10; ++i) {
        ts.emplace_back([&, i] { results[i] = wbr_gps::get_once(d.sock, 2000); });
    }
    for (auto& t : ts) t.join();

    // Every client must succeed and see the SAME position. Under the old
    // scheme they raced for bytes and disagreed.
    for (const auto& r : results) {
        ASSERT_TRUE(r.service_ok);
        ASSERT_TRUE(r.has_fix);
        ASSERT_NEAR(r.lat, results[0].lat, 1e-12);
        ASSERT_NEAR(r.lon, results[0].lon, 1e-12);
        ASSERT_EQ(r.seq, results[0].seq);
    }
    d.stop();
}

static void S3_fifty_watchers_all_receive()
{
    PtyPair& pty = new_pty(); ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    std::vector<int> fds;
    for (int i = 0; i < 50; ++i) {
        const int fd = raw_connect(d.sock);
        ASSERT_TRUE(fd >= 0);
        send_line(fd, "{\"op\":\"watch\",\"fix\":true,\"nmea\":false}");
        fds.push_back(fd);
    }
    ::usleep(400000);          // let all fifty subscriptions land
    pty.emit(kGGA);
    ::usleep(1000000);         // and the resulting broadcast reach them all

    int with_fix = 0;
    for (const int fd : fds) {
        wbr_gps::Snapshot s;
        // Drain until we see a FIX carrying has_fix. Everyone gets a HELLO,
        // an immediate snapshot with no fix, then the real one.
        for (int i = 0; i < 10; ++i) {
            if (!read_fix(fd, s, 2)) break;
            if (s.has_fix) { ++with_fix; break; }
        }
    }
    ASSERT_EQ(with_fix, 50);

    for (const int fd : fds) ::close(fd);
    d.stop();
}

static void S4_mixed_modes_concurrently()
{
    PtyPair& pty = new_pty(); ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    const int fix_watcher  = raw_connect(d.sock);
    const int nmea_watcher = raw_connect(d.sock);
    ASSERT_TRUE(fix_watcher >= 0 && nmea_watcher >= 0);
    send_line(fix_watcher,  "{\"op\":\"watch\",\"fix\":true,\"nmea\":false}");
    send_line(nmea_watcher, "{\"op\":\"watch\",\"fix\":false,\"nmea\":true}");
    ::usleep(300000);

    std::atomic<bool> get_ok{ false };
    std::thread getter([&] { get_ok = wbr_gps::get_once(d.sock, 2000).service_ok; });

    pty.emit(kGGA);
    ::usleep(800000);
    getter.join();

    // Three consumers, three different modes, at the same instant.
    ASSERT_TRUE(get_ok.load());

    wbr_gps::Snapshot s;
    ASSERT_TRUE(read_fix(fix_watcher, s, 8));

    // The NMEA watcher must get the verbatim sentence, not a re-rendering.
    bool saw_raw = false;
    for (int i = 0; i < 8 && !saw_raw; ++i) {
        const std::string line = read_line(nmea_watcher);
        std::string cls, raw;
        if (wbr_gps::json_get_string(line, "class", cls) && cls == "NMEA" &&
            wbr_gps::json_get_string(line, "raw", raw) && raw == kGGA) {
            saw_raw = true;
        }
    }
    ASSERT_TRUE(saw_raw);

    ::close(fix_watcher); ::close(nmea_watcher);
    d.stop();
}

static void S5_late_joiner_gets_snapshot_immediately()
{
    PtyPair& pty = new_pty(); ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);
    pty.emit(kGGA);
    ASSERT_TRUE(wait_until([&] { return wbr_gps::get_once(d.sock, 1000).has_fix; }, 3000));

    // Join well after the last sentence. We must get state at once, not wait
    // for the next GGA -- a consumer starting up between fixes cannot be made
    // to sit blind until the device happens to speak again.
    const int64_t t0 = wbr_gps::now_mono_ms();
    const int fd = raw_connect(d.sock);
    ASSERT_TRUE(fd >= 0);
    send_line(fd, "{\"op\":\"watch\",\"fix\":true,\"nmea\":false}");
    wbr_gps::Snapshot s;
    ASSERT_TRUE(read_fix(fd, s, 4));
    const int64_t elapsed = wbr_gps::now_mono_ms() - t0;

    ASSERT_TRUE(s.has_fix);
    ASSERT_TRUE(elapsed < 500);
    ::close(fd);
    d.stop();
}

static void S6_slow_client_does_not_starve_others()
{
    PtyPair& pty = new_pty(); ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    // A watcher that subscribes and then never reads a byte. This is the
    // wedged-consumer case: a GUI stopped in a debugger, a process swapped
    // out, a pipe nobody drains.
    const int slow = raw_connect(d.sock);
    ASSERT_TRUE(slow >= 0);
    send_line(slow, "{\"op\":\"watch\",\"fix\":true,\"nmea\":true}");

    // A healthy watcher.
    const int fast = raw_connect(d.sock);
    ASSERT_TRUE(fast >= 0);
    send_line(fast, "{\"op\":\"watch\",\"fix\":true,\"nmea\":false}");
    ::usleep(300000);

    // Hammer the daemon while `slow` reads nothing.
    for (int i = 0; i < 400; ++i) { pty.emit(kGGA); ::usleep(2000); }
    ::usleep(400000);

    // The healthy client must still be served, and get_once must still work.
    //
    // Drain rather than read once: the first FIX still sitting in this
    // socket is the empty snapshot sent when the watch was accepted, long
    // before any sentence arrived. Reading a single message here would
    // assert on that one and fail while the daemon was behaving perfectly.
    wbr_gps::Snapshot s;
    bool fast_saw_fix = false;
    for (int i = 0; i < 40 && !fast_saw_fix; ++i) {
        if (!read_fix(fast, s, 2)) break;
        fast_saw_fix = s.has_fix;
    }
    ASSERT_TRUE(fast_saw_fix);
    ASSERT_TRUE(wbr_gps::get_once(d.sock, 2000).service_ok);

    ::close(slow); ::close(fast);
    d.stop();
}

static void S7_sigkilled_client_is_cleaned_up()
{
    PtyPair& pty = new_pty(); ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    // Baseline only once the daemon has finished opening its own devices:
    // it binds the socket first and reaches the serial port up to 500 ms
    // later, so a count taken the moment the socket appears is short by one.
    ASSERT_TRUE(wait_until([&] { return wbr_gps::get_once(d.sock, 1000).serial_ok; }, 5000));
    const size_t before = settled_daemon_fd_count(d.pid);
    ASSERT_TRUE(before > 0);

    const pid_t child = ::fork();
    if (child == 0) {
        const int fd = raw_connect(d.sock);
        if (fd >= 0) send_line(fd, "{\"op\":\"watch\",\"fix\":true}");
        for (;;) ::pause();
    }
    ASSERT_TRUE(wait_until([&] { return daemon_fd_count(d.pid) > before; }, 2000));
    ::kill(child, SIGKILL);
    int st = 0; ::waitpid(child, &st, 0);

    // The daemon must notice and reclaim the descriptor, not hold it forever.
    ASSERT_TRUE(wait_until([&] { return daemon_fd_count(d.pid) == before; }, 3000));

    // And it must survive and keep serving.
    pty.emit(kGGA);
    ASSERT_TRUE(wait_until([&] { return wbr_gps::get_once(d.sock, 1000).has_fix; }, 3000));
    d.stop();
}

static void S8_disconnect_mid_write_does_not_kill_daemon()
{
    PtyPair& pty = new_pty(); ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    for (int round = 0; round < 20; ++round) {
        const int fd = raw_connect(d.sock);
        if (fd < 0) continue;
        send_line(fd, "{\"op\":\"watch\",\"fix\":true,\"nmea\":true}");
        pty.emit(kGGA);
        ::close(fd);          // vanish while the daemon is writing
    }
    ::usleep(500000);

    // If SIGPIPE were not ignored, the daemon would be dead by now.
    ASSERT_TRUE(wbr_gps::get_once(d.sock, 2000).service_ok);
    d.stop();
}

static void S9_connect_storm_leaks_nothing()
{
    PtyPair& pty = new_pty(); ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    // The interesting baseline is the DAEMON's, not ours: a descriptor the
    // daemon fails to reclaim is the leak that takes it down after a week.
    ASSERT_TRUE(wait_until([&] { return wbr_gps::get_once(d.sock, 1000).serial_ok; }, 5000));
    const size_t daemon_before = settled_daemon_fd_count(d.pid);
    const size_t self_before   = open_fd_count();
    ASSERT_TRUE(daemon_before > 0);

    for (int i = 0; i < 200; ++i) {
        const int fd = raw_connect(d.sock);
        if (fd >= 0) { send_line(fd, "{\"op\":\"get\"}"); ::close(fd); }
    }

    // Both sides must return to where they started.
    //
    // Measured once it stops moving, not the first time it happens to touch
    // the baseline: the daemon is still working through the storm's accept
    // backlog while we are still closing, so the count dips and rises again
    // several times before it settles. Sampling the dip would pass a leak.
    ASSERT_EQ(settled_daemon_fd_count(d.pid, 8000), daemon_before);
    ASSERT_TRUE(open_fd_count() <= self_before + 2);

    // And the daemon must still be healthy.
    ASSERT_TRUE(wbr_gps::get_once(d.sock, 2000).service_ok);
    d.stop();
}

static void S10_malformed_input_is_survivable()
{
    PtyPair& pty = new_pty(); ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    const int fd = raw_connect(d.sock);
    ASSERT_TRUE(fd >= 0);
    read_line(fd);                                   // HELLO
    send_line(fd, "not json at all");
    std::string cls;
    ASSERT_TRUE(wbr_gps::json_get_string(read_line(fd), "class", cls));
    ASSERT_STREQ(cls, "ERROR");
    ::close(fd);

    // Oversized request from a second client: over kMaxRequestBytes the
    // daemon drops the peer deliberately rather than buffering without bound.
    const int fd2 = raw_connect(d.sock);
    ASSERT_TRUE(fd2 >= 0);
    send_line(fd2, std::string(9000, 'A'));
    ::usleep(300000);
    ::close(fd2);

    ASSERT_TRUE(wbr_gps::get_once(d.sock, 2000).service_ok);
    d.stop();
}

static void run_tests()
{
    // This suite writes to sockets whose peers vanish on purpose. Without
    // this the harness itself dies of SIGPIPE partway through S8.
    ::signal(SIGPIPE, SIG_IGN);
    std::atexit(cleanup_everything);

    S1_single_get();
    S2_ten_simultaneous_gets_agree();
    S3_fifty_watchers_all_receive();
    S4_mixed_modes_concurrently();
    S5_late_joiner_gets_snapshot_immediately();
    S6_slow_client_does_not_starve_others();
    S7_sigkilled_client_is_cleaned_up();
    S8_disconnect_mid_write_does_not_kill_daemon();
    S9_connect_storm_leaks_nothing();
    S10_malformed_input_is_survivable();

    cleanup_everything();
}

TEST_MAIN
