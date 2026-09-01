#include "wbr_gps/client.hpp"
#include "wbr_gps/json_io.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace wbr_gps {

namespace {

constexpr int kReconnectMinMs = 250;
constexpr int kReconnectMaxMs = 2000;
// If no message arrives for this long the daemon is considered gone: it
// heartbeats every 2 s, so 6 s is three missed beats.
constexpr int64_t kSilenceTimeoutMs = 6000;

// Connects with the connect() phase itself bounded by timeout_ms. A plain
// blocking connect() to AF_UNIX can sit forever if the peer's listen backlog
// is already full and nobody is calling accept() -- e.g. a wedged daemon --
// and that would leave the background thread's run() loop stuck outside any
// of its own poll()/sleep() checks, so stop()'s join() would hang with it.
// A non-blocking socket plus poll() on POLLOUT makes the worst case bounded
// no matter what the peer is doing. Once connected, the fd is switched back
// to blocking with the same timeout applied to send/recv.
int connect_unix(const std::string& path, int timeout_ms)
{
    const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) return -1;

    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof addr.sun_path, "%s", path.c_str());

    if (::connect(fd, (struct sockaddr*)&addr, sizeof addr) != 0) {
        if (errno != EINPROGRESS) {
            ::close(fd);
            return -1;
        }
        for (;;) {
            struct pollfd p { fd, POLLOUT, 0 };
            const int pr = ::poll(&p, 1, timeout_ms);
            if (pr < 0 && errno == EINTR) continue;
            if (pr <= 0) { ::close(fd); return -1; }   // timed out: never hangs
            break;
        }
        int         err  = 0;
        socklen_t   elen = sizeof err;
        if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &elen) != 0 || err != 0) {
            ::close(fd);
            return -1;
        }
    }

    const int fl = ::fcntl(fd, F_GETFL, 0);
    if (fl >= 0) ::fcntl(fd, F_SETFL, fl & ~O_NONBLOCK);

    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);

    return fd;
}

bool send_all(int fd, const std::string& s)
{
    size_t off = 0;
    while (off < s.size()) {
        const ssize_t n = ::send(fd, s.data() + off, s.size() - off, MSG_NOSIGNAL);
        if (n > 0) { off += (size_t)n; continue; }
        if (n < 0 && errno == EINTR) continue;
        return false;
    }
    return true;
}

} // namespace

Client::~Client() { stop(); }

void Client::set_nmea_callback(std::function<void(const std::string&)> cb)
{
    std::lock_guard<std::mutex> lk(mu_);
    nmea_cb_ = std::move(cb);
}

bool Client::start(const std::string& sock_path, bool want_nmea)
{
    if (running_.load()) return true;
    sock_path_ = sock_path;
    want_nmea_ = want_nmea;
    stopping_.store(false);
    running_.store(true);
    thread_ = std::thread([this] { run(); });
    return true;
}

void Client::stop()
{
    if (!running_.load()) return;
    stopping_.store(true);
    if (thread_.joinable()) thread_.join();
    running_.store(false);
}

Snapshot Client::snapshot() const
{
    std::lock_guard<std::mutex> lk(mu_);
    return cached_;
}

// Guarantee G4: unavailability must be explicit, never a stale confident
// answer. Clearing service_ok alone is not enough -- has_fix, serial_ok,
// hid_ok, device_present and gpsdo_locked would otherwise keep reading their
// last value while the daemon is gone. lat/lon/alt_m are retained as
// last-known position, mirroring how StateStore handles staleness.
void Client::mark_unavailable()
{
    std::lock_guard<std::mutex> lk(mu_);
    cached_.service_ok     = false;
    cached_.has_fix        = false;
    cached_.serial_ok      = false;
    cached_.hid_ok         = false;
    cached_.device_present = false;
    cached_.gpsdo_locked   = false;
}

// T9-C: bounded logging for a misbehaving NMEA callback. A subscriber
// whose callback throws once will almost always throw on every
// subsequent sentence -- NMEA sentences arrive at roughly ten per
// second, so unbounded per-occurrence logging would fill a collection
// box's disk. Log the first occurrence with the exception text, then
// suppress until the callback succeeds again, and report the suppressed
// count exactly once on recovery so it is not silently lost. Deliberately
// not reset by a reconnect (nmea_cb_broken_/nmea_cb_suppressed_ are
// Client members, not local to run()'s per-connection scope): a
// permanently broken callback must stay quiet across reconnects, not log
// once per reconnect cycle.
void Client::note_nmea_callback_exception(const char* what)
{
    if (!nmea_cb_broken_) {
        std::fprintf(stderr, "[wbr_gps::Client] nmea callback threw: %s\n", what);
        nmea_cb_broken_ = true;
    } else {
        ++nmea_cb_suppressed_;
    }
}

void Client::note_nmea_callback_ok()
{
    if (!nmea_cb_broken_) return;
    std::fprintf(stderr,
        "[wbr_gps::Client] nmea callback recovered after %llu suppressed exception%s\n",
        (unsigned long long)nmea_cb_suppressed_, nmea_cb_suppressed_ == 1 ? "" : "s");
    nmea_cb_broken_     = false;
    nmea_cb_suppressed_ = 0;
}

void Client::run()
{
    int backoff = kReconnectMinMs;

    while (!stopping_.load()) {
        const int fd = connect_unix(sock_path_, 500);
        if (fd < 0) {
            mark_unavailable();   // Explicit. Never a fallback to the device.
            for (int slept = 0; slept < backoff && !stopping_.load(); slept += 50) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            backoff = std::min(backoff * 2, kReconnectMaxMs);
            continue;
        }
        backoff = kReconnectMinMs;

        const std::string sub = std::string("{\"op\":\"watch\",\"fix\":true,\"nmea\":") +
                                (want_nmea_ ? "true" : "false") + "}\n";
        if (!send_all(fd, sub)) { ::close(fd); mark_unavailable(); continue; }

        // Deliberately uncapped: wbr-gpsd is a trusted local daemon on a
        // Unix socket only root/the operator's group can reach, not an
        // untrusted network peer, so bounding this against a malicious
        // sender is not this library's job.
        std::string buf;
        int64_t last_msg_ms = now_mono_ms();

        while (!stopping_.load()) {
            struct pollfd p { fd, POLLIN, 0 };
            const int pr = ::poll(&p, 1, 200);

            if (pr > 0 && (p.revents & POLLIN)) {
                char io[2048];
                const ssize_t n = ::read(fd, io, sizeof io);
                if (n <= 0) break;                 // daemon gone
                buf.append(io, (size_t)n);
                last_msg_ms = now_mono_ms();

                size_t nl;
                while ((nl = buf.find('\n')) != std::string::npos) {
                    const std::string line = buf.substr(0, nl);
                    buf.erase(0, nl + 1);

                    Snapshot s;
                    if (snapshot_from_json(line, s)) {
                        s.service_ok = true;
                        std::lock_guard<std::mutex> lk(mu_);
                        cached_ = s;
                        continue;
                    }
                    std::string cls, raw;
                    if (json_get_string(line, "class", cls) && cls == "NMEA" &&
                        json_get_string(line, "raw", raw)) {
                        std::function<void(const std::string&)> cb;
                        { std::lock_guard<std::mutex> lk(mu_); cb = nmea_cb_; }
                        // A subscriber's callback is arbitrary user code
                        // (Task 16's gps_capture, for one). Run outside
                        // mu_ so a reentrant snapshot() call from inside
                        // the callback cannot deadlock, and contain any
                        // exception here: an uncaught throw would escape
                        // run(), which is a std::thread entry function --
                        // that calls std::terminate() and takes down the
                        // whole process, not just this thread.
                        if (cb) {
                            try {
                                cb(raw);
                                note_nmea_callback_ok();
                            } catch (const std::exception& e) {
                                note_nmea_callback_exception(e.what());
                            } catch (...) {
                                note_nmea_callback_exception("non-std::exception");
                            }
                        }
                    }
                }
                continue;
            }
            if (pr < 0 && errno == EINTR) continue;
            if (pr < 0) break;

            // Silence for longer than three heartbeats: treat as gone.
            if (now_mono_ms() - last_msg_ms > kSilenceTimeoutMs) break;
        }

        ::close(fd);
        mark_unavailable();
    }

    mark_unavailable();
}

Snapshot get_once(const std::string& sock_path, int timeout_ms)
{
    Snapshot out;   // service_ok = false by default: unavailable unless proven
    const int fd = connect_unix(sock_path, timeout_ms);
    if (fd < 0) return out;

    if (!send_all(fd, "{\"op\":\"get\"}\n")) { ::close(fd); return out; }

    std::string buf;
    const int64_t deadline = now_mono_ms() + timeout_ms;
    while (now_mono_ms() < deadline) {
        struct pollfd p { fd, POLLIN, 0 };
        const int pr = ::poll(&p, 1, 50);
        if (pr <= 0) continue;

        char io[2048];
        const ssize_t n = ::read(fd, io, sizeof io);
        if (n <= 0) break;
        buf.append(io, (size_t)n);

        size_t nl;
        while ((nl = buf.find('\n')) != std::string::npos) {
            const std::string line = buf.substr(0, nl);
            buf.erase(0, nl + 1);
            Snapshot s;
            if (snapshot_from_json(line, s)) {   // skips HELLO
                s.service_ok = true;
                ::close(fd);
                return s;
            }
        }
    }
    ::close(fd);
    return out;
}

} // namespace wbr_gps
