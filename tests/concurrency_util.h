#pragma once

// Shared fixtures for the two concurrency suites, split the same way the two
// Server suites are: test_concurrency.cpp covers many clients against one
// live daemon, test_concurrency_lifecycle.cpp covers the device and the
// daemon coming and going underneath them. Both are separate executables, so
// this header is included exactly once per binary.
//
// Two conventions hold throughout both:
//
//   * Nothing is left behind. Every Daemon owns its temp directory and kills
//     its child on every exit path, including an assertion failure, because
//     the fd and inode measurements are meaningless from a polluted baseline.
//   * Waits are on conditions, not clocks, wherever a condition exists.
//     A fixed usleep() is both slower than it needs to be and flakier than it
//     looks; wait_until() is used instead, except where the point of the test
//     IS that nothing happens.

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

inline PtyPair& new_pty()
{
    g_ptys.push_back(std::unique_ptr<PtyPair>(new PtyPair()));
    return *g_ptys.back();
}

inline void reap_temp_dir(const std::string& dir)
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

inline void cleanup_everything()
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
inline bool wait_until(const std::function<bool()>& done, int timeout_ms, int step_ms = 50)
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

inline int raw_connect(const std::string& path)
{
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un a {};
    a.sun_family = AF_UNIX;
    snprintf(a.sun_path, sizeof a.sun_path, "%s", path.c_str());
    if (::connect(fd, (struct sockaddr*)&a, sizeof a) != 0) { ::close(fd); return -1; }
    return fd;
}

inline void send_line(int fd, const std::string& s)
{
    const std::string f = s + "\n";
    ssize_t rc = ::send(fd, f.data(), f.size(), MSG_NOSIGNAL);
    (void)rc;
}

inline std::string read_line(int fd, int timeout_ms = 2000)
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
inline bool read_fix(int fd, wbr_gps::Snapshot& out, int tries = 5)
{
    for (int i = 0; i < tries; ++i) {
        const std::string line = read_line(fd);
        if (line.empty()) return false;
        if (wbr_gps::snapshot_from_json(line, out)) return true;
    }
    return false;
}

inline size_t count_fds(const std::string& proc_fd_dir)
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

inline size_t open_fd_count()               { return count_fds("/proc/self/fd"); }
inline size_t daemon_fd_count(pid_t pid)
{
    return count_fds("/proc/" + std::to_string(pid) + "/fd");
}

// A descriptor baseline is only meaningful once the daemon has finished
// opening what it opens. It binds the socket first and only reaches the
// serial and hidraw devices on its first periodic tick up to 500 ms later, so
// a count sampled the instant the socket appears is short by one or two and
// every later comparison against it fails for the wrong reason. Wait for the
// number to hold still instead of guessing how long that takes.
inline size_t settled_daemon_fd_count(pid_t pid, int timeout_ms = 5000)
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
inline bool daemon_holds(pid_t pid, const std::string& target)
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
inline size_t fds_pointing_at(const std::string& target, int except)
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
inline int daemon_exit_status(const std::vector<std::string>& args)
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
