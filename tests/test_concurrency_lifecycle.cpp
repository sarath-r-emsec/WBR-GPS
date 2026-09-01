// Scenarios S11-S17: the device and the daemon coming and going while
// clients are connected.
//
// The companion to test_concurrency.cpp. Where that file asks whether many
// programs can share one daemon, this one asks what they are told when the
// GPS is unplugged, when the daemon is killed outright, when it restarts over
// its own stale socket, and when a second one tries to start alongside it.
// The answers must be consistent across every client and must never be a
// confident wrong one.

#include "concurrency_util.h"

static void S11_unplug_is_seen_consistently_by_all()
{
    PtyPair& pty = new_pty(); ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);
    pty.emit(kGGA);

    std::vector<std::unique_ptr<wbr_gps::Client>> clients;
    for (int i = 0; i < 5; ++i) {
        auto c = std::unique_ptr<wbr_gps::Client>(new wbr_gps::Client());
        ASSERT_TRUE(c->start(d.sock));
        clients.push_back(std::move(c));
    }
    ASSERT_TRUE(wait_until([&] {
        for (auto& c : clients) if (!c->snapshot().serial_ok) return false;
        return true;
    }, 4000));
    for (auto& c : clients) ASSERT_TRUE(c->snapshot().serial_ok);

    pty.hangup();              // "unplug": the device side goes away

    // Every client must agree that the serial port is gone. This only works
    // because the daemon treats EPOLLHUP on the serial fd as a hangup: a tty
    // at VMIN=0/VTIME=0 returns read()==0 for "idle" and for "gone" alike, so
    // without that branch the daemon reports serial_ok=true forever after an
    // unplug -- a confident wrong answer.
    ASSERT_TRUE(wait_until([&] {
        for (auto& c : clients) if (c->snapshot().serial_ok) return false;
        return true;
    }, 5000));
    for (auto& c : clients) ASSERT_FALSE(c->snapshot().serial_ok);

    // And they must still be talking to the daemon: a device failure is not
    // a service failure, and collapsing the two is the original bug.
    for (auto& c : clients) ASSERT_TRUE(c->snapshot().service_ok);

    for (auto& c : clients) c->stop();
    clients.clear();
    d.stop();
}

static void S12_daemon_death_leaves_device_untouched()
{
    // THE guarantee-G5 scenario. When the daemon dies, no client may grab the
    // device. Asserted twice over: behaviourally, that every client says so
    // out loud, and structurally, that no descriptor in this process points
    // at the device afterwards.
    PtyPair& pty = new_pty(); ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);
    pty.emit(kGGA);

    std::vector<std::unique_ptr<wbr_gps::Client>> clients;
    for (int i = 0; i < 5; ++i) {
        auto c = std::unique_ptr<wbr_gps::Client>(new wbr_gps::Client());
        ASSERT_TRUE(c->start(d.sock));
        clients.push_back(std::move(c));
    }
    ASSERT_TRUE(wait_until([&] {
        for (auto& c : clients) if (!c->snapshot().service_ok) return false;
        return true;
    }, 4000));

    // Only the daemon holds the device while it is alive: the pty slave is
    // open here exactly once, by the harness itself.
    ASSERT_EQ(fds_pointing_at(pty.slave_path, pty.slave), (size_t)0);

    d.kill_hard();

    // Every client reports the service down, explicitly.
    ASSERT_TRUE(wait_until([&] {
        for (auto& c : clients) if (c->snapshot().service_ok) return false;
        return true;
    }, 5000));
    for (auto& c : clients) ASSERT_FALSE(c->snapshot().service_ok);
    // And none has invented a fix from somewhere.
    for (auto& c : clients) ASSERT_FALSE(c->snapshot().has_fix);

    // The structural claim, and the reason this scenario exists. Give the
    // clients several reconnect cycles (backoff is 250 ms doubling to 2 s) to
    // do the wrong thing before concluding that they will not. A fallback
    // that opened the port would show up here as a second opener even if it
    // reported every value correctly.
    ::usleep(3000000);
    ASSERT_EQ(fds_pointing_at(pty.slave_path, pty.slave), (size_t)0);

    for (auto& c : clients) c->stop();
    clients.clear();
}

static void S13_clients_reconnect_after_restart()
{
    // The device is addressed through a re-pointable link because a pty
    // cannot be reopened by a restarted daemon -- see SerialLink. Nothing
    // about the client behaviour under test depends on that; the clients
    // never see the device, only the socket.
    SerialLink link; ASSERT_TRUE(link.up());
    Daemon d; d.start(link.path);

    wbr_gps::Client c;
    ASSERT_TRUE(c.start(d.sock));
    ASSERT_TRUE(wait_until([&] { return c.snapshot().service_ok; }, 4000));

    d.kill_hard();
    ASSERT_TRUE(wait_until([&] { return !c.snapshot().service_ok; }, 4000));
    ASSERT_FALSE(c.snapshot().service_ok);

    // The daemon comes back on the same socket path, over the stale socket
    // file its SIGKILL left behind. No client is restarted, and no client is
    // told to reconnect: it is the client library's own job to notice.
    ASSERT_TRUE(link.replace());
    d.spawn(link.path);
    ASSERT_TRUE(wait_until([&] { return c.snapshot().service_ok; }, 10000));
    ASSERT_TRUE(c.snapshot().service_ok);

    // And data flows again through the restarted daemon, end to end.
    ASSERT_TRUE(wait_until([&] { return daemon_holds(d.pid, link.pty->slave_path); }, 8000));
    link.emit(kGGA);
    ASSERT_TRUE(wait_until([&] { return c.snapshot().has_fix; }, 5000));

    c.stop();
    d.stop();
}

static void S14_second_daemon_refuses_to_start()
{
    PtyPair& pty = new_pty(); ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);
    ASSERT_TRUE(wbr_gps::get_once(d.sock, 2000).service_ok);

    const pid_t second = ::fork();
    if (second == 0) {
        ::execl(WBR_GPSD_PATH, "wbr-gpsd", "--socket", d.sock.c_str(),
                "--lock", d.lock.c_str(), "--serial", pty.slave_path.c_str(),
                "--group", "", (char*)nullptr);
        _exit(127);
    }
    int st = 0;
    ::waitpid(second, &st, 0);
    // Must exit non-zero rather than run alongside the first. Two daemons
    // fighting over one tty is the entire problem this program exists to end.
    ASSERT_TRUE(WIFEXITED(st));
    ASSERT_TRUE(WEXITSTATUS(st) != 0);
    ASSERT_TRUE(WEXITSTATUS(st) != 127);   // it ran; it did not fail to exec

    // The original must be unharmed.
    ASSERT_TRUE(wbr_gps::get_once(d.sock, 2000).service_ok);
    d.stop();
}

static void S15_stale_socket_does_not_block_startup()
{
    PtyPair& pty = new_pty(); ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);
    ASSERT_TRUE(wbr_gps::get_once(d.sock, 2000).service_ok);

    d.kill_hard();               // SIGKILL: no cleanup, socket file remains
    struct stat st {};
    ASSERT_EQ(::stat(d.sock.c_str(), &st), 0);   // the stale file really is there

    // The restarted daemon logs one "serial ... Device or resource busy" here
    // and that is expected, not a fault: the SIGKILLed daemon had set
    // TIOCEXCL on the pty, and the harness's own slave descriptor keeps that
    // tty -- and so its exclusivity -- alive past the owner's death. Real
    // hardware has no second opener, so the port frees on exit. This scenario
    // is about the stale SOCKET, so the serial port is beside the point; S13
    // is the one that proves the device comes back, and it uses a replaceable
    // node for exactly this reason.
    d.spawn(pty.slave_path);     // must reclaim it rather than refuse to bind
    ASSERT_TRUE(wait_until([&] { return wbr_gps::get_once(d.sock, 1000).service_ok; }, 5000));
    ASSERT_TRUE(wbr_gps::get_once(d.sock, 3000).service_ok);
    d.stop();
}

// S16 is not in the design doc. It was added because nothing else in this
// project exercises the reconnect path more than once, and a descriptor
// leaked per reconnect is invisible until the daemon has been running for
// hours against a flaky cable -- which is the normal condition for a USB
// device in a vehicle.
static void S16_repeated_reconnect_leaks_no_descriptors()
{
    SerialLink link; ASSERT_TRUE(link.up());
    Daemon d; d.start(link.path);
    ASSERT_TRUE(wait_until([&] { return daemon_holds(d.pid, link.pty->slave_path); }, 6000));

    const size_t baseline = settled_daemon_fd_count(d.pid);
    ASSERT_TRUE(baseline > 0);

    const int kCycles = 50;
    int reconnected = 0;
    for (int i = 0; i < kCycles; ++i) {
        if (!link.replace()) break;
        // Wait for the daemon to be reading the NEW node specifically. A
        // reported serial_ok flag would also go true if it had never let go
        // of the old one, which is exactly the failure this scenario hunts.
        if (!wait_until([&] { return daemon_holds(d.pid, link.pty->slave_path); }, 8000, 25)) break;
        ++reconnected;
    }
    ASSERT_EQ(reconnected, kCycles);

    // The whole point: fifty open/close cycles must cost nothing permanent.
    const size_t after = settled_daemon_fd_count(d.pid);
    ASSERT_EQ(after, baseline);

    // Still serving, and the position still flows through the newest device.
    link.emit(kGGA);
    ASSERT_TRUE(wait_until([&] { return wbr_gps::get_once(d.sock, 1000).has_fix; }, 5000));

    d.stop();
}

// S17 is not a concurrency scenario, and it is here because main.cpp had no
// CI-checked coverage of any kind before this suite existed. Argument
// handling is the part of it a test can reach in milliseconds, and it guards
// a hazard the file calls out in its own comments: a bad --baud must be
// REJECTED, never coerced, because atoi() would turn a typo into baud 0 and
// the daemon would then open the port at the driver default and quietly
// decode nothing.
static void S17_command_line_is_validated_not_coerced()
{
    // --help is a request, so it succeeds; a bad argument is a diagnostic,
    // so it fails with 2. Neither may start a daemon.
    ASSERT_EQ(daemon_exit_status({ "--help" }), 0);
    ASSERT_EQ(daemon_exit_status({ "-h" }), 0);

    ASSERT_EQ(daemon_exit_status({ "--nonsense" }), 2);
    ASSERT_EQ(daemon_exit_status({ "--baud" }), 2);            // missing value
    ASSERT_EQ(daemon_exit_status({ "--socket" }), 2);          // missing value

    // Every one of these would be silently accepted by atoi().
    ASSERT_EQ(daemon_exit_status({ "--baud", "0" }), 2);
    ASSERT_EQ(daemon_exit_status({ "--baud", "-1" }), 2);
    ASSERT_EQ(daemon_exit_status({ "--baud", "abc" }), 2);
    ASSERT_EQ(daemon_exit_status({ "--baud", "115200x" }), 2);
    ASSERT_EQ(daemon_exit_status({ "--baud", "99999999" }), 2);
    ASSERT_EQ(daemon_exit_status({ "--stale-ms", "0" }), 2);
    ASSERT_EQ(daemon_exit_status({ "--stale-ms", "-1" }), 2);
    ASSERT_EQ(daemon_exit_status({ "--stale-ms", "junk" }), 2);

    // A well-formed command line that cannot take the singleton lock must
    // exit 1, not 0: exiting 0 would report success to systemd, and would
    // silently break any later switch to Restart=on-failure.
    ASSERT_EQ(daemon_exit_status({ "--socket", "/nonexistent-dir/gpsd.sock",
                                   "--lock",   "/nonexistent-dir/pid",
                                   "--serial", "/dev/null",
                                   "--group",  "",
                                   "--foreground" }), 1);
}

static void run_tests()
{
    ::signal(SIGPIPE, SIG_IGN);
    std::atexit(cleanup_everything);

    S11_unplug_is_seen_consistently_by_all();
    S12_daemon_death_leaves_device_untouched();
    S13_clients_reconnect_after_restart();
    S14_second_daemon_refuses_to_start();
    S15_stale_socket_does_not_block_startup();
    S16_repeated_reconnect_leaks_no_descriptors();
    S17_command_line_is_validated_not_coerced();

    cleanup_everything();
}

TEST_MAIN
