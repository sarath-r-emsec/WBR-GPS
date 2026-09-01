#include "wbr_gps/client.hpp"
#include "pty_harness.h"
#include "test_util.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

static const char* kGGA =
    "$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F";

// Path to wbr-gpsd, set by CMake so the test does not guess.
#ifndef WBR_GPSD_PATH
#define WBR_GPSD_PATH "./wbr-gpsd"
#endif

struct Daemon {
    pid_t       pid = -1;
    std::string dir;
    std::string sock;

    void start(const std::string& serial_path)
    {
        char tmpl[] = "/tmp/wbrgps_client_XXXXXX";
        dir  = mkdtemp(tmpl);
        sock = dir + "/gpsd.sock";
        const std::string lock = dir + "/pid";

        pid = fork();
        if (pid == 0) {
            execl(WBR_GPSD_PATH, "wbr-gpsd",
                  "--socket", sock.c_str(),
                  "--lock",   lock.c_str(),
                  "--serial", serial_path.c_str(),
                  "--group",  "",
                  (char*)nullptr);
            _exit(127);
        }
        usleep(400000);   // let it bind
    }

    void stop()
    {
        if (pid > 0) { kill(pid, SIGTERM); int st = 0; waitpid(pid, &st, 0); pid = -1; }
    }

    ~Daemon() { stop(); }
};

static void test_snapshot_tracks_daemon()
{
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    Daemon d;
    d.start(pty.slave_path);

    wbr_gps::Client c;
    ASSERT_TRUE(c.start(d.sock));
    usleep(300000);

    // Before any sentence: connected to the service, but no fix.
    wbr_gps::Snapshot s = c.snapshot();
    ASSERT_TRUE(s.service_ok);
    ASSERT_FALSE(s.has_fix);

    pty.emit(kGGA);
    usleep(800000);

    s = c.snapshot();
    ASSERT_TRUE(s.service_ok);
    ASSERT_TRUE(s.has_fix);
    ASSERT_NEAR(s.lat, 13.0028260, 1e-6);
    ASSERT_NEAR(s.lon, 77.6799202, 1e-6);

    c.stop();
    d.stop();
}

static void test_service_down_is_explicit_not_silent()
{
    // Guarantee G4/G5: when the daemon dies the client must report
    // service_ok=false, NOT quietly open the device itself.
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    Daemon d;
    d.start(pty.slave_path);

    wbr_gps::Client c;
    ASSERT_TRUE(c.start(d.sock));
    pty.emit(kGGA);
    usleep(800000);
    ASSERT_TRUE(c.snapshot().service_ok);

    d.stop();
    usleep(1200000);

    const wbr_gps::Snapshot s = c.snapshot();
    ASSERT_FALSE(s.service_ok);

    c.stop();
}

static void test_disconnect_clears_health_but_retains_position()
{
    // R2 / guarantee G4: service_ok=false alone is not enough. A consumer
    // reading has_fix (or serial_ok, hid_ok, device_present, gpsdo_locked)
    // in isolation must never see a stale "true" once the daemon is gone --
    // that is the exact "confident wrong answer" this project exists to
    // remove. lat/lon/alt_m ARE retained as last-known position, mirroring
    // StateStore::mark_fix_stale_if_older_than.
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    Daemon d;
    d.start(pty.slave_path);

    wbr_gps::Client c;
    ASSERT_TRUE(c.start(d.sock));
    usleep(300000);

    pty.emit(kGGA);
    usleep(800000);

    // Sanity: a real fix landed before we kill anything, so the assertions
    // below are meaningful rather than vacuously true.
    wbr_gps::Snapshot before = c.snapshot();
    ASSERT_TRUE(before.service_ok);
    ASSERT_TRUE(before.has_fix);
    ASSERT_TRUE(before.serial_ok);
    ASSERT_NEAR(before.lat, 13.0028260, 1e-6);
    ASSERT_NEAR(before.lon, 77.6799202, 1e-6);

    d.stop();
    usleep(1000000);

    const wbr_gps::Snapshot after = c.snapshot();
    ASSERT_FALSE(after.service_ok);
    ASSERT_FALSE(after.has_fix);
    ASSERT_FALSE(after.serial_ok);
    ASSERT_FALSE(after.hid_ok);
    ASSERT_FALSE(after.device_present);
    ASSERT_FALSE(after.gpsdo_locked);
    // Position retained, not zeroed.
    ASSERT_NEAR(after.lat, before.lat, 1e-9);
    ASSERT_NEAR(after.lon, before.lon, 1e-9);
    ASSERT_NEAR(after.alt_m, before.alt_m, 1e-9);

    c.stop();
}

static void test_client_reconnects_after_daemon_restart()
{
    // S13.
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    Daemon d;
    d.start(pty.slave_path);

    wbr_gps::Client c;
    ASSERT_TRUE(c.start(d.sock));
    usleep(300000);
    ASSERT_TRUE(c.snapshot().service_ok);

    const std::string sock = d.sock;
    const std::string dir  = d.dir;
    d.stop();
    usleep(600000);
    ASSERT_FALSE(c.snapshot().service_ok);

    // Restart on the same socket path.
    Daemon d2;
    d2.dir  = dir;
    d2.sock = sock;
    d2.pid  = fork();
    if (d2.pid == 0) {
        execl(WBR_GPSD_PATH, "wbr-gpsd",
              "--socket", sock.c_str(), "--lock", (dir + "/pid").c_str(),
              "--serial", pty.slave_path.c_str(), "--group", "", (char*)nullptr);
        _exit(127);
    }
    usleep(2500000);   // client backoff plus daemon startup

    ASSERT_TRUE(c.snapshot().service_ok);

    c.stop();
    d2.stop();
}

static void test_get_once_without_daemon_reports_unavailable()
{
    const wbr_gps::Snapshot s = wbr_gps::get_once("/tmp/definitely-no-socket-here", 200);
    ASSERT_FALSE(s.service_ok);
    ASSERT_FALSE(s.has_fix);
    ASSERT_FALSE(s.device_present);
}

static void test_get_once_returns_a_fix()
{
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    Daemon d;
    d.start(pty.slave_path);
    pty.emit(kGGA);
    usleep(800000);

    const wbr_gps::Snapshot s = wbr_gps::get_once(d.sock, 1000);
    ASSERT_TRUE(s.service_ok);
    ASSERT_TRUE(s.has_fix);
    ASSERT_NEAR(s.lat, 13.0028260, 1e-6);

    d.stop();
}

static void test_nmea_callback_receives_raw_sentences_verbatim()
{
    // T9-A: the raw NMEA path (want_nmea=true + set_nmea_callback) had no
    // coverage at all. Task 16's gps_capture is the first real consumer,
    // and it depends on raw subscribers getting the unfiltered stream --
    // including sentences the parser itself rejects -- so this test emits
    // one accepted sentence and one the parser rejects on checksum, and
    // requires the callback to receive both, byte for byte.
    static const char* kBadChecksum =
        "$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*00";

    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    Daemon d;
    d.start(pty.slave_path);

    std::mutex               cb_mu;
    std::vector<std::string> received;

    wbr_gps::Client c;
    c.set_nmea_callback([&](const std::string& raw) {
        std::lock_guard<std::mutex> lk(cb_mu);
        received.push_back(raw);
    });
    ASSERT_TRUE(c.start(d.sock, /*want_nmea=*/true));
    usleep(300000);

    pty.emit(kGGA);
    pty.emit(kBadChecksum);
    usleep(800000);

    {
        std::lock_guard<std::mutex> lk(cb_mu);
        ASSERT_EQ(received.size(), (size_t)2);
        if (received.size() >= 2) {
            ASSERT_STREQ(received[0], kGGA);
            ASSERT_STREQ(received[1], kBadChecksum);
        }
    }

    // The rejected sentence reached the raw subscriber unfiltered, but it
    // must not have corrupted the parsed FIX snapshot, which still stands
    // on the one accepted sentence.
    ASSERT_TRUE(c.snapshot().has_fix);

    c.stop();

    // No callback after stop() returns.
    size_t count_after_stop;
    {
        std::lock_guard<std::mutex> lk(cb_mu);
        count_after_stop = received.size();
    }
    pty.emit(kGGA);
    usleep(500000);
    {
        std::lock_guard<std::mutex> lk(cb_mu);
        ASSERT_EQ(received.size(), count_after_stop);
    }

    d.stop();
}

static void test_nmea_callback_exception_does_not_crash_reader_thread()
{
    // Investigation item: a subscriber's callback is arbitrary user code.
    // If it throws and src/client.cpp did not contain it, the throw would
    // escape run() -- a std::thread entry function -- which calls
    // std::terminate() and takes down the whole process, not merely "the
    // reader thread". Proven here, not assumed: if the throw were
    // uncaught, the process would already be dead (SIGABRT) before any
    // assertion below could run, and no further test in this binary would
    // execute either.
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    Daemon d;
    d.start(pty.slave_path);

    wbr_gps::Client c;
    c.set_nmea_callback([](const std::string&) {
        throw std::runtime_error("deliberate test exception");
    });
    ASSERT_TRUE(c.start(d.sock, /*want_nmea=*/true));
    usleep(300000);

    pty.emit(kGGA);
    usleep(500000);

    // The reader loop must still be alive and still processing FIX
    // messages after the callback threw.
    pty.emit(kGGA);
    usleep(500000);
    ASSERT_TRUE(c.snapshot().service_ok);
    ASSERT_TRUE(c.snapshot().has_fix);

    c.stop();
    d.stop();
}

static void test_nmea_callback_exception_logging_is_bounded()
{
    // T9-C: a callback that throws on every sentence must not log once
    // per sentence. This codebase already has a one-shot latch for
    // exactly this reason elsewhere (src/main.cpp's serial_fail_logged);
    // src/client.cpp's note_nmea_callback_exception()/note_nmea_callback_ok()
    // mirror it. Redirect this process's own stderr to a temp file for
    // the burst, then assert the property that matters: N sentences
    // through a permanently-broken callback produce fewer than N log
    // lines. Not an exact count -- that would be brittle.
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    Daemon d;
    d.start(pty.slave_path);

    wbr_gps::Client c;
    c.set_nmea_callback([](const std::string&) {
        throw std::runtime_error("consumer callback blew up");
    });
    ASSERT_TRUE(c.start(d.sock, /*want_nmea=*/true));
    usleep(300000);

    char tmpl[] = "/tmp/wbrgps_stderr_capture_XXXXXX";
    const int cap_fd = mkstemp(tmpl);
    ASSERT_TRUE(cap_fd >= 0);
    const int saved_stderr = dup(2);
    ASSERT_TRUE(saved_stderr >= 0);
    fflush(stderr);
    dup2(cap_fd, 2);

    const int kSentences = 40;
    for (int i = 0; i < kSentences; ++i) {
        pty.emit(kGGA);
        usleep(50000);
    }
    usleep(300000);

    fflush(stderr);
    dup2(saved_stderr, 2);   // restore before anything else logs
    close(saved_stderr);

    lseek(cap_fd, 0, SEEK_SET);
    std::string captured;
    char       rbuf[4096];
    ssize_t    n;
    while ((n = read(cap_fd, rbuf, sizeof rbuf)) > 0) captured.append(rbuf, (size_t)n);
    close(cap_fd);
    unlink(tmpl);

    size_t lines = 0;
    for (const char ch : captured) if (ch == '\n') ++lines;

    ASSERT_TRUE(lines < (size_t)kSentences);
    ASSERT_TRUE(captured.find("nmea callback threw") != std::string::npos);

    c.stop();
    d.stop();
}

static void test_nmea_callback_non_std_exception_is_latched_too()
{
    // T9-C's first "thing to get right": a callback throwing a
    // non-std::exception type must be caught by catch(...) and go
    // through the SAME one-shot latch as the std::exception path, not an
    // unbounded fallback.
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    Daemon d;
    d.start(pty.slave_path);

    wbr_gps::Client c;
    c.set_nmea_callback([](const std::string&) {
        throw 42;   // deliberately not a std::exception
    });
    ASSERT_TRUE(c.start(d.sock, /*want_nmea=*/true));
    usleep(300000);

    char tmpl[] = "/tmp/wbrgps_stderr_capture2_XXXXXX";
    const int cap_fd = mkstemp(tmpl);
    ASSERT_TRUE(cap_fd >= 0);
    const int saved_stderr = dup(2);
    ASSERT_TRUE(saved_stderr >= 0);
    fflush(stderr);
    dup2(cap_fd, 2);

    const int kSentences = 20;
    for (int i = 0; i < kSentences; ++i) {
        pty.emit(kGGA);
        usleep(50000);
    }
    usleep(300000);

    fflush(stderr);
    dup2(saved_stderr, 2);
    close(saved_stderr);

    lseek(cap_fd, 0, SEEK_SET);
    std::string captured;
    char       rbuf[4096];
    ssize_t    n;
    while ((n = read(cap_fd, rbuf, sizeof rbuf)) > 0) captured.append(rbuf, (size_t)n);
    close(cap_fd);
    unlink(tmpl);

    size_t lines = 0;
    for (const char ch : captured) if (ch == '\n') ++lines;

    ASSERT_TRUE(lines < (size_t)kSentences);
    ASSERT_TRUE(captured.find("nmea callback threw: non-std::exception") != std::string::npos);
    ASSERT_TRUE(c.snapshot().service_ok);

    c.stop();
    d.stop();
}

static void test_nmea_callback_latch_survives_reconnect()
{
    // T9-C's second "thing to get right": the latch must not be
    // resettable by a reconnect alone. Trip it, force a daemon restart
    // (same mechanism as test_client_reconnects_after_daemon_restart),
    // then emit another sentence through the new connection and confirm
    // no second "threw" line appears -- nmea_cb_broken_/nmea_cb_suppressed_
    // are Client members, not local to run()'s per-connection scope.
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    Daemon d;
    d.start(pty.slave_path);

    wbr_gps::Client c;
    c.set_nmea_callback([](const std::string&) {
        throw std::runtime_error("still broken");
    });
    ASSERT_TRUE(c.start(d.sock, /*want_nmea=*/true));
    usleep(300000);

    char tmpl[] = "/tmp/wbrgps_stderr_capture3_XXXXXX";
    const int cap_fd = mkstemp(tmpl);
    ASSERT_TRUE(cap_fd >= 0);
    const int saved_stderr = dup(2);
    ASSERT_TRUE(saved_stderr >= 0);
    fflush(stderr);
    dup2(cap_fd, 2);

    pty.emit(kGGA);
    usleep(500000);   // latch trips: exactly one "threw" line so far

    const std::string sock = d.sock;
    const std::string dir  = d.dir;
    d.stop();
    usleep(600000);

    Daemon d2;
    d2.dir  = dir;
    d2.sock = sock;
    d2.pid  = fork();
    if (d2.pid == 0) {
        execl(WBR_GPSD_PATH, "wbr-gpsd",
              "--socket", sock.c_str(), "--lock", (dir + "/pid").c_str(),
              "--serial", pty.slave_path.c_str(), "--group", "", (char*)nullptr);
        _exit(127);
    }
    usleep(2500000);   // client backoff plus daemon startup

    pty.emit(kGGA);
    usleep(500000);

    fflush(stderr);
    dup2(saved_stderr, 2);
    close(saved_stderr);

    lseek(cap_fd, 0, SEEK_SET);
    std::string captured;
    char       rbuf[4096];
    ssize_t    n;
    while ((n = read(cap_fd, rbuf, sizeof rbuf)) > 0) captured.append(rbuf, (size_t)n);
    close(cap_fd);
    unlink(tmpl);

    size_t threw_lines = 0;
    size_t pos = 0;
    while ((pos = captured.find("nmea callback threw", pos)) != std::string::npos) {
        ++threw_lines;
        pos += 1;
    }

    // Exactly one occurrence across the whole run, including the
    // reconnect: the latch was never reset by reconnecting.
    ASSERT_EQ(threw_lines, (size_t)1);

    c.stop();
    d2.stop();
}

static void test_stop_returns_promptly_when_peer_backlog_is_full()
{
    // Investigation item: can stop() hang if the background thread is stuck
    // inside connect() itself (as opposed to its own poll()/reconnect-sleep
    // checks, which both bound themselves)? A raw AF_UNIX listener with a
    // backlog of 1, saturated by extra non-blocking connections and never
    // accepted, reproduces a wedged peer with no real daemon involved.
    // Verified empirically (outside this suite) that a plain blocking
    // connect() against exactly this setup does not return -- confirming
    // this is a real hang vector, not a hypothetical one.
    char tmpl[] = "/tmp/wbrgps_wedge_XXXXXX";
    const std::string dir  = mkdtemp(tmpl);
    const std::string sock = dir + "/wedged.sock";

    const int lfd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    ASSERT_TRUE(lfd >= 0);
    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof addr.sun_path, "%s", sock.c_str());
    ASSERT_TRUE(::bind(lfd, (struct sockaddr*)&addr, sizeof addr) == 0);
    ASSERT_TRUE(::listen(lfd, 1) == 0);   // tiny backlog, deliberately

    // Saturate the accept queue well past the nominal backlog so kernel
    // off-by-one accounting cannot leave room. Non-blocking so the test's
    // own setup can never hang regardless of exact backlog semantics.
    std::vector<int> fillers;
    for (int i = 0; i < 8; ++i) {
        const int ffd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
        ::connect(ffd, (struct sockaddr*)&addr, sizeof addr);
        fillers.push_back(ffd);
    }
    // lfd never calls accept(): it stays wedged for the life of the test.

    wbr_gps::Client c;
    ASSERT_TRUE(c.start(sock));
    usleep(300000);   // let the background thread attempt to connect and wedge

    const int64_t t0 = wbr_gps::now_mono_ms();
    c.stop();
    const int64_t elapsed = wbr_gps::now_mono_ms() - t0;

    // Bounded well below "forever": one connect attempt is capped at
    // 500 ms inside connect_unix(). A true hang would never reach here.
    ASSERT_TRUE(elapsed < 5000);

    for (const int ffd : fillers) ::close(ffd);
    ::close(lfd);
    ::unlink(sock.c_str());
}

static void test_client_destroyed_without_stop_is_safe()
{
    // Investigation item: does a Client destroyed without stop() called
    // explicitly behave safely? ~Client() calls stop() itself, so this
    // should be a clean join with no leaked thread and no crash -- proven
    // here by the process continuing normally afterward (a leaked joinable
    // thread destroyed without join()/detach() would instead call
    // std::terminate).
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    Daemon d;
    d.start(pty.slave_path);

    {
        wbr_gps::Client c;
        ASSERT_TRUE(c.start(d.sock));
        usleep(300000);
        ASSERT_TRUE(c.snapshot().service_ok);
        // c goes out of scope here with no explicit stop() call.
    }

    // If the destructor mishandled this, the process would already have
    // aborted above. Confirm the daemon and its socket are still perfectly
    // usable by a fresh client, i.e. nothing was left in a broken state.
    wbr_gps::Client c2;
    ASSERT_TRUE(c2.start(d.sock));
    usleep(300000);
    ASSERT_TRUE(c2.snapshot().service_ok);
    c2.stop();

    d.stop();
}

static void run_tests()
{
    test_snapshot_tracks_daemon();
    test_service_down_is_explicit_not_silent();
    test_disconnect_clears_health_but_retains_position();
    test_client_reconnects_after_daemon_restart();
    test_get_once_without_daemon_reports_unavailable();
    test_get_once_returns_a_fix();
    test_nmea_callback_receives_raw_sentences_verbatim();
    test_nmea_callback_exception_does_not_crash_reader_thread();
    test_nmea_callback_exception_logging_is_bounded();
    test_nmea_callback_non_std_exception_is_latched_too();
    test_nmea_callback_latch_survives_reconnect();
    test_stop_returns_promptly_when_peer_backlog_is_full();
    test_client_destroyed_without_stop_is_safe();
}

TEST_MAIN
