// wbr-gpsd: the single owner of the Leo Bodnar GPSDO.
//
// One process, one thread, one epoll loop. Everything this file does is
// assembly: the parser, the state store, the exclusive serial source, the
// hidraw lock source and the socket server were each built and tested on
// their own. What lives here is the wiring, and the wiring has its own ways
// of being wrong -- a daemon that burns a core, or one that never notices the
// device was unplugged. Both are called out at the point where they are
// prevented.

// ClientTable and the epoll helpers used to live in this file. They were
// lifted into client_table.hpp so the EPOLLOUT arm/disarm invariant can be
// asserted directly by tests/test_client_table.cpp -- see that header for
// why that particular piece, and only that piece, was worth extracting.
#include "client_table.hpp"

#include "wbr_gps/device_presence.hpp"
#include "wbr_gps/gps_types.hpp"
#include "wbr_gps/hid_source.hpp"
#include "wbr_gps/serial_source.hpp"
#include "wbr_gps/server.hpp"
#include "wbr_gps/state_store.hpp"

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <sys/epoll.h>
#include <unistd.h>

using namespace wbr_gps;

namespace {

// --- tunables, all in one place ---------------------------------------------

constexpr int     kMaxEvents     = 64;
// Idle wakeup. Also the upper bound on how long shutdown can take after a
// signal that arrives while we are parked in epoll_wait().
constexpr int     kPollTimeoutMs = 500;
// Presence, reconnect and staleness are all "at most twice a second" work.
constexpr int64_t kPeriodicMs    = 500;
// A push at least this often lets a client detect a dead daemon by silence
// rather than waiting forever for a change that will never come.
constexpr int64_t kHeartbeatMs   = 2000;
constexpr int64_t kDefaultStaleMs = 10000;
constexpr mode_t  kSocketMode    = 0660;

// How long the listener stays deregistered after an accept() failure that
// retrying cannot clear.
constexpr int64_t kAcceptBackoffMs = 1000;

// The timerfd is gone, so the epoll timeout is the only thing that wakes an
// idle loop, and it is what bounds how late the periodic block can run. A
// poll timeout longer than the period would silently slow presence
// detection, fix staleness and reconnect.
static_assert(kPollTimeoutMs <= kPeriodicMs,
              "the epoll timeout is the idle tick; it must not outrun the periodic period");

volatile sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

// Wall clock, for the NMEA passthrough timestamp only. Never for ages: the
// GPSDO can step this clock, and a negative fix age is a classic GPS daemon
// bug. Ages come from now_mono_ms().
int64_t now_unix_ms()
{
    struct timespec ts {};
    ::clock_gettime(CLOCK_REALTIME, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

// --- command line -----------------------------------------------------------

struct Options {
    std::string socket_path = kDefaultSocketPath;
    std::string serial_path;                     // empty: auto-discover
    std::string hid_path;                        // empty: auto-discover
    std::string lock_path   = "/run/wbr-gps/wbr-gpsd.pid";
    std::string group       = "dialout";         // empty: leave ownership alone
    int         baud        = kDefaultBaud;
    int64_t     stale_ms    = kDefaultStaleMs;
};

// --help is a request, so its answer goes to stdout; a usage message
// provoked by a bad argument is a diagnostic and goes to stderr.
void usage(std::FILE* out, const char* argv0)
{
    std::fprintf(out,
        "Usage: %s [--socket PATH] [--serial PATH] [--baud N] [--hid PATH]\n"
        "          [--lock PATH] [--group NAME] [--stale-ms N] [--foreground]\n"
        "\n"
        "The daemon always runs in the foreground; --foreground is accepted\n"
        "for clarity in a systemd Type=simple unit and does nothing.\n"
        "An empty --group leaves socket ownership untouched.\n", argv0);
}

bool parse_i64(const char* s, int64_t& out)
{
    errno = 0;
    char* end = nullptr;
    const long long v = std::strtoll(s, &end, 10);
    if (errno != 0 || end == s || end == nullptr || *end != '\0') return false;
    out = (int64_t)v;
    return true;
}

enum class ParseOutcome { Run, Help, Error };

ParseOutcome parse_args(int argc, char** argv, Options& o)
{
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];

        if (a == "--help" || a == "-h") return ParseOutcome::Help;
        if (a == "--foreground") continue;      // accepted, and already true

        const bool takes_value =
            a == "--socket" || a == "--serial" || a == "--hid" ||
            a == "--lock"   || a == "--group"  || a == "--baud" ||
            a == "--stale-ms";
        if (!takes_value) {
            std::fprintf(stderr, "[wbr-gpsd] unknown argument: %s\n", a.c_str());
            return ParseOutcome::Error;
        }
        if (i + 1 >= argc) {
            std::fprintf(stderr, "[wbr-gpsd] missing value for %s\n", a.c_str());
            return ParseOutcome::Error;
        }
        const char* v = argv[++i];

        if      (a == "--socket") o.socket_path = v;
        else if (a == "--serial") o.serial_path = v;
        else if (a == "--hid")    o.hid_path    = v;
        else if (a == "--lock")   o.lock_path   = v;
        else if (a == "--group")  o.group       = v;
        else if (a == "--baud") {
            int64_t n = 0;
            // Rejected rather than coerced: atoi() would turn a typo into
            // baud 0, and the daemon would then open the port at whatever
            // speed the driver defaults to and quietly decode nothing.
            if (!parse_i64(v, n) || n <= 0 || n > 4000000) {
                std::fprintf(stderr, "[wbr-gpsd] bad --baud: %s\n", v);
                return ParseOutcome::Error;
            }
            o.baud = (int)n;
        }
        else if (a == "--stale-ms") {
            int64_t n = 0;
            if (!parse_i64(v, n) || n <= 0) {
                std::fprintf(stderr, "[wbr-gpsd] bad --stale-ms: %s\n", v);
                return ParseOutcome::Error;
            }
            o.stale_ms = n;
        }
    }
    return ParseOutcome::Run;
}

} // namespace

int main(int argc, char** argv)
{
    Options opt;
    switch (parse_args(argc, argv, opt)) {
        case ParseOutcome::Help:  usage(stdout, argv[0]); return 0;
        case ParseOutcome::Error: usage(stderr, argv[0]); return 2;
        case ParseOutcome::Run:   break;
    }

    // A client that disconnects mid-write must never kill the daemon. The
    // server also passes MSG_NOSIGNAL; this is the belt to that pair of braces.
    std::signal(SIGPIPE, SIG_IGN);
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    Server server;

    // Order matters, and it is this file's job to get right. server.hpp
    // documents that acquire_singleton() must come first and must have
    // succeeded, but Server cannot enforce it internally without breaking the
    // standalone tests that construct it without a lock. The flock is the
    // only real defence against two daemons: the stale-socket probe inside
    // listen_on() is a convenience and is inherently racy between two
    // processes starting at the same instant. Failure here is fatal --
    // continuing would mean a second daemon fighting the first for the tty,
    // which is the entire problem this program exists to end.
    if (!server.acquire_singleton(opt.lock_path)) {
        std::fprintf(stderr,
                     "[wbr-gpsd] could not take the singleton lock at %s, refusing to start\n",
                     opt.lock_path.c_str());
        return 1;
    }

    // An empty --group means "leave ownership alone". Passing "" through
    // would send getgrnam("") off to fail and log a spurious "no such group".
    const char* group = opt.group.empty() ? nullptr : opt.group.c_str();
    if (!server.listen_on(opt.socket_path, group, kSocketMode)) return 1;
    std::fprintf(stderr, "[wbr-gpsd] listening on %s (pid %d)\n",
                 opt.socket_path.c_str(), (int)::getpid());

    const int ep = ::epoll_create1(EPOLL_CLOEXEC);
    if (ep < 0) {
        // Left unchecked this is a spin, not a stumble: every epoll_wait on
        // fd -1 returns EBADF immediately and the loop never sleeps.
        std::fprintf(stderr, "[wbr-gpsd] epoll_create1: %s\n", std::strerror(errno));
        return 1;
    }
    if (!epoll_add(ep, server.listen_fd(), EPOLLIN)) { ::close(ep); return 1; }

    StateStore   store;
    SerialSource serial;
    HidSource    hid;
    serial.set_baud(opt.baud);
    if (!opt.hid_path.empty()) hid.set_path(opt.hid_path);

    ClientTable clients(ep, server);

    // Logged once per outage rather than once per retry: with a 5 s backoff
    // ceiling an absent device would otherwise write a line to the journal
    // every five seconds forever.
    bool serial_fail_logged = false;
    bool hid_fail_logged    = false;

    auto close_serial = [&](const char* why) {
        if (serial.fd() >= 0) {
            // Only when we still hold it. SerialSource::on_readable() closes
            // the fd itself on a hard error, and closing an fd already
            // removes it from the epoll set.
            epoll_drop(ep, serial.fd());
            serial.close_device();
        }
        store.set_serial_ok(false);
        serial.note_retry(now_mono_ms());
        std::fprintf(stderr, "[wbr-gpsd] serial closed: %s\n", why);
    };

    auto close_hid = [&](const char* why) {
        if (hid.fd() >= 0) {
            epoll_drop(ep, hid.fd());
            hid.close_device();
        }
        store.set_hid_ok(false);
        // We no longer know the lock state, so we must stop asserting the
        // last one we saw. A consumer switching an SDR to the GPSDO's 10 MHz
        // reference on a stale gpsdo_locked=true would be steering off a
        // clock that is no longer plugged in.
        store.set_gpsdo_locked(false);
        hid.note_retry(now_mono_ms());
        std::fprintf(stderr, "[wbr-gpsd] hid closed: %s\n", why);
    };

    int64_t  last_periodic_ms  = 0;
    int64_t  last_heartbeat_ms = 0;
    // Non-zero while the listener is deregistered after a hard accept
    // failure: the monotonic time at which to try registering it again.
    int64_t  listener_resume_ms = 0;
    // T8-B. Exiting 0 after a fatal loop error would report success to
    // systemd, which is wrong on its own terms even though the Task 11 unit
    // uses Restart=always and would bring us back regardless. It also
    // silently breaks any later switch to Restart=on-failure, and misleads
    // whoever reads `systemctl status`.
    int      exit_status       = 0;
    // A plain local. As a function-static it would outlive main's own state
    // and quietly survive into any second call or test harness that reran
    // this loop, comparing a fresh store's seq against the previous run's.
    uint64_t last_seq          = 0;

    while (!g_stop) {
        struct epoll_event evs[kMaxEvents];
        const int n = ::epoll_wait(ep, evs, kMaxEvents, kPollTimeoutMs);
        if (n < 0) {
            if (errno == EINTR) continue;        // a signal; g_stop decides
            std::fprintf(stderr, "[wbr-gpsd] epoll_wait: %s\n", std::strerror(errno));
            exit_status = 1;
            break;                               // never spin on a broken epoll
        }
        const int64_t now = now_mono_ms();

        for (int i = 0; i < n; ++i) {
            const int      fd = evs[i].data.fd;
            const uint32_t ev = evs[i].events;

            if (fd == server.listen_fd()) {
                for (;;) {
                    const int c = server.accept_client();
                    if (c >= 0) { clients.add(c); continue; }
                    if (c == Server::kAcceptDrained) break;

                    // A hard accept failure -- EMFILE, ENFILE, ENOMEM.
                    // Breaking out is not enough on its own: the pending
                    // connection stays in the accept queue and the listening
                    // socket is level triggered, so it would report ready on
                    // every pass and the loop would spin at 100% of a core
                    // without ever making progress. Deregister the listener
                    // and try again shortly; existing clients keep being
                    // served throughout, and closing one is what frees the
                    // descriptor that lets accept succeed again.
                    epoll_drop(ep, server.listen_fd());
                    listener_resume_ms = now + kAcceptBackoffMs;
                    std::fprintf(stderr,
                                 "[wbr-gpsd] pausing accept for %lld ms (%zu clients connected)\n",
                                 (long long)kAcceptBackoffMs, server.client_count());
                    break;
                }
                continue;
            }

            if (serial.fd() >= 0 && fd == serial.fd()) {
                // The hangup check belongs here, on the serial fd, and not
                // only on client fds.
                //
                // On a tty, read() returning 0 does NOT mean EOF. Measured on
                // a live pty and again on one whose master had been closed:
                //     idle pty    read()=0 errno=0
                //     master gone read()=0 errno=0  revents=0x19
                // Identical. SerialSource::on_readable() is right to treat 0
                // as "drained", which leaves EPOLLHUP/EPOLLERR as the only
                // thing that separates a device that was unplugged from one
                // that simply has nothing to say. Without this branch the
                // daemon would go on reporting serial_ok=true forever after
                // an unplug -- a confident wrong answer, which is exactly the
                // failure mode this daemon exists to remove.
                //
                // Tested before EPOLLIN because a hangup raises both at once:
                // 0x19 above is POLLHUP|POLLERR|POLLIN.
                if (ev & (EPOLLHUP | EPOLLERR)) {
                    close_serial("hangup");
                } else if (ev & EPOLLIN) {
                    const bool alive = serial.on_readable(
                        store, now,
                        [&](const std::string& line) {
                            server.broadcast_nmea(line, now_unix_ms());
                        });
                    if (!alive) close_serial("read error");
                }
                continue;
            }

            if (hid.fd() >= 0 && fd == hid.fd()) {
                // Same reasoning as the serial fd: a detached hidraw node
                // must be noticed, not waited on.
                if (ev & (EPOLLHUP | EPOLLERR)) {
                    close_hid("hangup");
                } else if (ev & EPOLLIN) {
                    if (!hid.on_readable(store)) close_hid("read error");
                }
                continue;
            }

            if (!clients.has(fd)) {
                // Not the listener, not a source, and not a client we own.
                // Deregister it rather than act on an fd whose identity we
                // cannot account for.
                std::fprintf(stderr, "[wbr-gpsd] event on unowned fd %d, deregistering\n", fd);
                epoll_drop(ep, fd);
                continue;
            }

            if (ev & (EPOLLHUP | EPOLLERR)) { clients.remove(fd); continue; }
            if ((ev & EPOLLIN) && !server.on_client_readable(fd, store, now)) {
                clients.remove(fd);
                continue;
            }
            // EPOLLOUT needs no branch of its own: the sync_all() at the foot
            // of this loop writes every non-empty queue and re-registers.
        }

        // --- periodic work, rate limited ------------------------------------
        //
        // Gated on elapsed monotonic time rather than run once per pass.
        // leo_bodnar_present() opens and scans /dev/serial/by-id, so a
        // per-pass version would put a directory walk behind every client
        // request and every NMEA sentence. The gate also survives a busy
        // loop, which a bare epoll timeout would not: when events keep
        // arriving, epoll_wait never times out.
        if (now - last_periodic_ms >= kPeriodicMs) {
            last_periodic_ms = now;

            // Presence is its own signal, deliberately independent of whether
            // we managed to open the port. "It is plugged in but I cannot
            // read it" and "it is not plugged in" are different faults and
            // must stay separately reportable.
            store.set_device_present(leo_bodnar_present());

            // Bring the listener back after a hard accept failure. If the
            // condition has not cleared, the next accept fails again and
            // pauses it again -- bounded at one attempt per backoff rather
            // than one per loop pass.
            if (listener_resume_ms != 0 && now >= listener_resume_ms) {
                if (epoll_add(ep, server.listen_fd(), EPOLLIN)) {
                    listener_resume_ms = 0;
                    std::fprintf(stderr, "[wbr-gpsd] accepting connections again\n");
                } else {
                    listener_resume_ms = now + kAcceptBackoffMs;
                }
            }

            if (serial.fd() < 0 && serial.should_retry(now)) {
                std::string path = opt.serial_path;
                if (path.empty()) {
                    path = find_leo_bodnar_serial();
                    if (path.empty()) path = kDefaultSerialPath;
                }
                serial.set_path(path);
                // The two failures are handled apart so that each reports its
                // own reason. Sharing one branch meant reading errno after
                // epoll_add() had already called strerror(), which is allowed
                // to clobber it -- so a registration failure could be
                // reported with the wrong cause.
                if (!serial.open_device()) {
                    const int err = errno;      // captured before anything else runs
                    store.set_serial_ok(false);
                    serial.note_retry(now);
                    if (!serial_fail_logged) {
                        std::fprintf(stderr, "[wbr-gpsd] serial %s unavailable: %s\n",
                                     path.c_str(), std::strerror(err));
                        serial_fail_logged = true;
                    }
                } else if (!epoll_add(ep, serial.fd(), EPOLLIN)) {
                    // epoll_add has already logged the reason. Closing is
                    // mandatory: otherwise we hold the port exclusively with
                    // nothing on earth ever reading from it.
                    serial.close_device();
                    store.set_serial_ok(false);
                    serial.note_retry(now);
                } else {
                    store.set_serial_ok(true);
                    serial_fail_logged = false;
                    std::fprintf(stderr, "[wbr-gpsd] serial open: %s @ %d\n",
                                 path.c_str(), opt.baud);
                }
            }

            if (hid.fd() < 0 && hid.should_retry(now)) {
                // Re-run discovery unless the operator pinned a path.
                // HidSource caches the node it found in path_ and skips
                // discover() while that is set, so a device that comes back
                // as a different hidrawN would never be found again.
                if (opt.hid_path.empty()) hid.set_path("");
                if (!hid.open_device()) {
                    const int err = errno;      // captured before anything else runs
                    store.set_hid_ok(false);
                    store.set_gpsdo_locked(false);
                    hid.note_retry(now);
                    if (!hid_fail_logged) {
                        std::fprintf(stderr, "[wbr-gpsd] hid unavailable: %s\n",
                                     std::strerror(err));
                        hid_fail_logged = true;
                    }
                } else if (!epoll_add(ep, hid.fd(), EPOLLIN)) {
                    hid.close_device();
                    store.set_hid_ok(false);
                    store.set_gpsdo_locked(false);
                    hid.note_retry(now);
                } else {
                    store.set_hid_ok(true);
                    hid_fail_logged = false;
                    std::fprintf(stderr, "[wbr-gpsd] hid open: %s\n", hid.path().c_str());
                }
            }

            // A fix that stopped arriving stops being a fix. The position is
            // retained so a consumer can still show a last-known location,
            // but has_fix goes false.
            store.mark_fix_stale_if_older_than(now, opt.stale_ms);
        }

        // --- push -----------------------------------------------------------

        const bool changed   = store.seq() != last_seq;
        const bool heartbeat = now - last_heartbeat_ms >= kHeartbeatMs;
        if (changed || heartbeat) {
            server.broadcast_fix(store, now);
            last_seq          = store.seq();
            last_heartbeat_ms = now;
        }

        // Everything queued this pass -- HELLOs, request replies, NMEA
        // passthrough, the FIX above -- is written here, and EPOLLOUT is
        // armed only for whoever could not take it all.
        clients.sync_all();
    }

    std::fprintf(stderr, "[wbr-gpsd] shutting down\n");
    serial.close_device();
    hid.close_device();
    server.shutdown();          // closes clients and the listener, unlinks the
                                // socket, releases the singleton lock
    ::close(ep);
    return exit_status;
}
