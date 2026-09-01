// Scenarios S1-S16 from the design doc, run against the real wbr-gpsd binary
// with a pty standing in for the Leo Bodnar.
//
// This is the file that answers the original request: "test all possible
// scenarios with multiple programs trying to connect." Every other suite in
// this project tests a component in isolation. This one starts the actual
// daemon as a child process and then behaves like a hostile fleet of
// consumers -- fifty at once, one that wedges, one that is killed mid-write,
// one that connects two hundred times in a row -- and asserts that the
// answers stay consistent and the daemon stays up.
//
// Two conventions hold throughout:
//
//   * Nothing is left behind. Every Daemon owns its temp directory and kills
//     its child on every exit path, including an assertion failure, because
//     the fd and inode measurements in S9 and S16 are meaningless from a
//     polluted baseline.
//   * Waits are on conditions, not clocks, wherever a condition exists.
//     A fixed usleep() is both slower than it needs to be and flakier than
//     it looks; wait_until() is used instead except where the point of the
//     test IS that nothing happens.

#include "wbr_gps/client.hpp"
#include "wbr_gps/json_io.hpp"
#include "pty_harness.h"
#include "test_util.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <functional>
#include <memory>
#include <signal.h>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#ifndef WBR_GPSD_PATH
#define WBR_GPSD_PATH "./wbr-gpsd"
#endif

static const char* kGGA =
    "$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F";

// ---------- cleanup registry ------------------------------------------------
//
// An assertion failure in this suite does not abort -- test_util.h counts and
// carries on -- so destructors do run. The registry is the belt to that pair
// of braces: it also covers a hard crash mid-suite via atexit().

static std::vector<pid_t>       g_daemon_pids;
static std::vector<std::string> g_temp_dirs;

// Every pty this suite creates lives until the suite ends. Scenarios take a
// reference rather than owning one, so no pts index is ever recycled between
// them -- see PtyPair::hangup() for why a recycled index is a source of
// failures that look like daemon bugs and are not.
static std::vector<std::unique_ptr<PtyPair>> g_ptys;

static PtyPair& new_pty()
{
    g_ptys.push_back(std::unique_ptr<PtyPair>(new PtyPair()));
    return *g_ptys.back();
}

static void reap_temp_dir(const std::string& dir)
{
    if (dir.empty()) return;
    DIR* d = ::opendir(dir.c_str());
    if (d != nullptr) {
        while (const struct dirent* e = ::readdir(d)) {
            const std::string n = e->d_name;
            if (n == "." || n == "..") continue;
            ::unlink((dir + "/" + n).c_str());
        }
        ::closedir(d);
    }
    ::rmdir(dir.c_str());
}

static void cleanup_everything()
{
    for (const pid_t p : g_daemon_pids) {
        if (p > 0) { ::kill(p, SIGKILL); int st = 0; ::waitpid(p, &st, WNOHANG); }
    }
    g_daemon_pids.clear();
    for (const std::string& d : g_temp_dirs) reap_temp_dir(d);
    g_temp_dirs.clear();
    for (auto& p : g_ptys) if (p) p->close_pair();
    g_ptys.clear();
}

// ---------- helpers ---------------------------------------------------------

// Poll a predicate instead of guessing a sleep length.
static bool wait_until(const std::function<bool()>& done, int timeout_ms, int step_ms = 50)
{
    for (int waited = 0; waited < timeout_ms; waited += step_ms) {
        if (done()) return true;
        ::usleep((useconds_t)step_ms * 1000);
    }
    return done();
}

struct Daemon {
    pid_t pid = -1;
    std::string dir, sock, lock;

    void start(const std::string& serial_path)
    {
        char tmpl[] = "/tmp/wbrgps_conc_XXXXXX";
        const char* made = ::mkdtemp(tmpl);
        if (made == nullptr) { fprintf(stderr, "mkdtemp failed\n"); return; }
        dir  = made;
        g_temp_dirs.push_back(dir);
        sock = dir + "/gpsd.sock";
        lock = dir + "/pid";
        spawn(serial_path);
    }

    void spawn(const std::string& serial_path)
    {
        pid = ::fork();
        if (pid == 0) {
            ::execl(WBR_GPSD_PATH, "wbr-gpsd", "--socket", sock.c_str(),
                    "--lock", lock.c_str(), "--serial", serial_path.c_str(),
                    "--group", "", (char*)nullptr);
            _exit(127);
        }
        g_daemon_pids.push_back(pid);
        // The daemon binds before it does anything else, so waiting for the
        // socket to exist is both faster and more reliable than a fixed sleep.
        const std::string s = sock;
        wait_until([s] { struct stat st {}; return ::stat(s.c_str(), &st) == 0; }, 3000, 20);
    }

    void forget(pid_t p)
    {
        for (auto& x : g_daemon_pids) if (x == p) x = -1;
    }

    // SIGKILL: no cleanup, no unlink, no lock release by the program itself.
    void kill_hard()
    {
        if (pid > 0) { ::kill(pid, SIGKILL); int st = 0; ::waitpid(pid, &st, 0); forget(pid); pid = -1; }
    }

    void stop()
    {
        if (pid > 0) { ::kill(pid, SIGTERM); int st = 0; ::waitpid(pid, &st, 0); forget(pid); pid = -1; }
    }

    // Runs on every exit path from a scenario, assertion failures included.
    ~Daemon()
    {
        stop();
        if (!dir.empty()) {
            reap_temp_dir(dir);
            for (auto& d : g_temp_dirs) if (d == dir) d.clear();
        }
    }
};

static int raw_connect(const std::string& path)
{
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un a {};
    a.sun_family = AF_UNIX;
    snprintf(a.sun_path, sizeof a.sun_path, "%s", path.c_str());
    if (::connect(fd, (struct sockaddr*)&a, sizeof a) != 0) { ::close(fd); return -1; }
    return fd;
}

static void send_line(int fd, const std::string& s)
{
    const std::string f = s + "\n";
    ssize_t rc = ::send(fd, f.data(), f.size(), MSG_NOSIGNAL);
    (void)rc;
}

static std::string read_line(int fd, int timeout_ms = 2000)
{
    std::string out;
    struct timeval tv { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    char c;
    while (::recv(fd, &c, 1, 0) == 1) {
        if (c == '\n') break;
        out += c;
    }
    return out;
}

// Read lines until one parses as a FIX, skipping HELLO and NMEA.
static bool read_fix(int fd, wbr_gps::Snapshot& out, int tries = 5)
{
    for (int i = 0; i < tries; ++i) {
        const std::string line = read_line(fd);
        if (line.empty()) return false;
        if (wbr_gps::snapshot_from_json(line, out)) return true;
    }
    return false;
}

static size_t count_fds(const std::string& proc_fd_dir)
{
    size_t n = 0;
    DIR* d = ::opendir(proc_fd_dir.c_str());
    if (d == nullptr) return 0;
    while (const struct dirent* e = ::readdir(d)) {
        const std::string nm = e->d_name;
        if (nm == "." || nm == "..") continue;
        ++n;
    }
    ::closedir(d);
    return n;
}

static size_t open_fd_count()               { return count_fds("/proc/self/fd"); }
static size_t daemon_fd_count(pid_t pid)
{
    return count_fds("/proc/" + std::to_string(pid) + "/fd");
}

// A descriptor baseline is only meaningful once the daemon has finished
// opening what it opens. It binds the socket first and only reaches the
// serial and hidraw devices on its first periodic tick up to 500 ms later, so
// a count sampled the instant the socket appears is short by one or two and
// every later comparison against it fails for the wrong reason. Wait for the
// number to hold still instead of guessing how long that takes.
static size_t settled_daemon_fd_count(pid_t pid, int timeout_ms = 5000)
{
    size_t last   = daemon_fd_count(pid);
    int    stable = 0;
    for (int t = 0; t < timeout_ms; t += 200) {
        ::usleep(200000);
        const size_t now = daemon_fd_count(pid);
        if (now == last) { if (++stable >= 2) return now; }
        else             { stable = 0; last = now; }
    }
    return last;
}

// True when the daemon currently holds `target` open. Used instead of polling
// a reported flag, because it answers the question the reconnect scenarios
// actually ask -- "is the daemon reading THIS node?" -- rather than "does the
// daemon believe some serial port is open?".
static bool daemon_holds(pid_t pid, const std::string& target)
{
    const std::string dir = "/proc/" + std::to_string(pid) + "/fd";
    DIR* d = ::opendir(dir.c_str());
    if (d == nullptr) return false;
    bool found = false;
    while (const struct dirent* e = ::readdir(d)) {
        const std::string nm = e->d_name;
        if (nm == "." || nm == "..") continue;
        char buf[512];
        const ssize_t len = ::readlink((dir + "/" + nm).c_str(), buf, sizeof buf - 1);
        if (len <= 0) continue;
        buf[len] = '\0';
        if (target == buf) { found = true; break; }
    }
    ::closedir(d);
    return found;
}

// How many fds in THIS process resolve to `target`, ignoring `except`.
// The structural half of S12: a client that fell back to the device would
// show up here even if it never changed a single reported value.
static size_t fds_pointing_at(const std::string& target, int except)
{
    size_t n = 0;
    DIR* d = ::opendir("/proc/self/fd");
    if (d == nullptr) return 0;
    while (const struct dirent* e = ::readdir(d)) {
        const std::string nm = e->d_name;
        if (nm == "." || nm == "..") continue;
        const int fd = std::atoi(nm.c_str());
        if (fd == except) continue;
        char buf[512];
        const ssize_t len = ::readlink(("/proc/self/fd/" + nm).c_str(), buf, sizeof buf - 1);
        if (len <= 0) continue;
        buf[len] = '\0';
        if (target == buf) ++n;
    }
    ::closedir(d);
    return n;
}

// A stable path that can be re-pointed at a fresh pty, standing in for the
// udev symlink the daemon is given on real hardware (/dev/gpsdo).
//
// Two scenarios need the device to come BACK, and a bare pty cannot express
// that. Two reasons, both measured rather than assumed:
//
//   * TIOCEXCL is a property of the tty, cleared only when the last
//     descriptor on it closes. The harness holds the slave open, so the
//     master keeps the tty alive and a dead daemon's exclusivity outlives
//     it: the restarted daemon reopens the same node and is told EBUSY.
//     On real hardware the daemon is the only opener, so its death releases
//     the port -- the pty is what differs, not the daemon.
//   * pts indices are recycled immediately. Tearing a pty down and building
//     another usually lands on the same index, and the daemon's reopen then
//     races that teardown. Because a failed open doubles the retry backoff
//     (250 ms up to 5 s) while only the first failure is logged, a single
//     transient loss costs seconds of reconnect delay.
//
// Both disappear if the replacement pty is created and published BEFORE the
// old one is hung up: the new node cannot reuse the live index, and the path
// never points at anything dead.
struct SerialLink {
    std::string dir;
    std::string path;                 // the symlink; this is what --serial gets
    PtyPair*    pty = nullptr;        // owned by g_ptys, not by this object

    bool up()
    {
        char tmpl[] = "/tmp/wbrgps_link_XXXXXX";
        const char* made = ::mkdtemp(tmpl);
        if (made == nullptr) return false;
        dir = made;
        g_temp_dirs.push_back(dir);
        path = dir + "/gpsdo";
        pty = &new_pty();
        if (!pty->open_pair()) return false;
        return ::symlink(pty->slave_path.c_str(), path.c_str()) == 0;
    }

    // Publish a fresh node, THEN hang up the old one. Order is the whole
    // point; see above.
    bool replace()
    {
        PtyPair& next = new_pty();
        if (!next.open_pair()) return false;
        ::unlink(path.c_str());
        if (::symlink(next.slave_path.c_str(), path.c_str()) != 0) return false;
        PtyPair* old = pty;
        pty = &next;
        old->hangup();                // the unplug, once the new node is live
        return true;
    }

    void emit(const std::string& line) { if (pty) pty->emit(line); }

    ~SerialLink()
    {
        if (!path.empty()) ::unlink(path.c_str());
        if (!dir.empty()) {
            reap_temp_dir(dir);
            for (auto& d : g_temp_dirs) if (d == dir) d.clear();
        }
    }
};

// Run the daemon to completion with these arguments and return its exit
// status, or -1 if it did not exit normally. Used only for command lines that
// are meant to be refused -- a valid one would never return.
static int daemon_exit_status(const std::vector<std::string>& args)
{
    const pid_t p = ::fork();
    if (p == 0) {
        std::vector<char*> argv;
        argv.push_back(const_cast<char*>("wbr-gpsd"));
        for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
        argv.push_back(nullptr);
        // The usage text is not what is under test, and printing it for every
        // case would bury the real output of this suite.
        if (std::freopen("/dev/null", "w", stdout) == nullptr) _exit(126);
        if (std::freopen("/dev/null", "w", stderr) == nullptr) _exit(126);
        ::execv(WBR_GPSD_PATH, argv.data());
        _exit(127);
    }
    int st = 0;
    ::waitpid(p, &st, 0);
    return WIFEXITED(st) ? WEXITSTATUS(st) : -1;
}

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
