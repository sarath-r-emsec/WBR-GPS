#include "wbr_gps/server.hpp"
#include "wbr_gps/json_io.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <grp.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace wbr_gps {

namespace {

constexpr size_t kSunPathMax = sizeof(sockaddr_un::sun_path);

bool set_nonblock(int fd)
{
    const int fl = ::fcntl(fd, F_GETFL, 0);
    return fl >= 0 && ::fcntl(fd, F_SETFL, fl | O_NONBLOCK) == 0;
}

// sun_path is a fixed field with no length prefix, so an over-long path can
// only ever be silently truncated. Refuse instead of truncating.
bool fill_addr(struct sockaddr_un& addr, const std::string& path)
{
    if (path.size() >= kSunPathMax) return false;
    addr.sun_family = AF_UNIX;
    std::memcpy(addr.sun_path, path.c_str(), path.size() + 1);
    return true;
}

// Is something already listening on this socket path? Used to distinguish a
// stale file from a live daemon: unlinking a live daemon's socket would let
// two daemons run, which is exactly what we must prevent.
//
// Two things this must get right, both learned the hard way:
//  * The probe must not block. A blocking connect() to a live daemon whose
//    accept backlog is full parks in unix_wait_for_peer() forever, hanging
//    daemon startup.
//  * Only a definite "nobody is listening" counts as dead. connect() failing
//    with EAGAIN means the backlog is full, which means somebody IS there.
//    Anything we cannot classify is treated as live, because refusing to
//    start is always safer than unlinking a running daemon's socket.
bool socket_is_live(const std::string& path)
{
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return true;                 // cannot prove it is dead
    if (!set_nonblock(fd)) { ::close(fd); return true; }

    struct sockaddr_un addr {};
    if (!fill_addr(addr, path)) { ::close(fd); return true; }

    // Dead means, and only means: no file at all, or a socket inode with no
    // listener behind it. Everything else -- EAGAIN from a full backlog,
    // EACCES, EINPROGRESS -- is treated as live.
    const bool connected = ::connect(fd, (struct sockaddr*)&addr, sizeof addr) == 0;
    const bool dead      = !connected && (errno == ENOENT || errno == ECONNREFUSED);
    ::close(fd);
    return !dead;
}

} // namespace

Server::~Server() { shutdown(); }

bool Server::acquire_singleton(const std::string& lock_path)
{
    if (lock_fd_ >= 0) return true;          // already held by this instance

    const int fd = ::open(lock_path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (fd < 0) {
        std::fprintf(stderr, "[wbr-gpsd] open %s: %s\n",
                     lock_path.c_str(), std::strerror(errno));
        return false;
    }
    // flock(), not fcntl(): fcntl locks are per-process, so a second
    // acquire in the same process would be granted silently. flock locks are
    // per open file description and conflict correctly either way.
    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
        std::fprintf(stderr, "[wbr-gpsd] another daemon holds %s\n",
                     lock_path.c_str());
        ::close(fd);
        return false;
    }
    lock_fd_ = fd;

    char pid[32];
    const int n = std::snprintf(pid, sizeof pid, "%d\n", (int)::getpid());
    if (n > 0 && ::ftruncate(lock_fd_, 0) == 0) {
        const ssize_t rc = ::write(lock_fd_, pid, (size_t)n);
        (void)rc;                            // the pid is advisory only
    }
    return true;
}

bool Server::listen_on(const std::string& sock_path, const char* group, mode_t mode)
{
    if (listen_fd_ >= 0) return false;       // already listening

    // sun_path is a fixed 108-byte field. Truncating silently would leave the
    // daemon listening on a path no client will ever connect to.
    if (sock_path.size() >= kSunPathMax) {
        std::fprintf(stderr, "[wbr-gpsd] socket path is %zu bytes, max is %zu: %s\n",
                     sock_path.size(), kSunPathMax - 1, sock_path.c_str());
        return false;
    }

    if (socket_is_live(sock_path)) {
        std::fprintf(stderr, "[wbr-gpsd] another daemon is listening on %s\n",
                     sock_path.c_str());
        return false;
    }
    ::unlink(sock_path.c_str());     // safe: nothing is listening

    const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        std::fprintf(stderr, "[wbr-gpsd] socket(): %s\n", std::strerror(errno));
        return false;
    }
    if (!set_nonblock(fd)) { ::close(fd); return false; }

    struct sockaddr_un addr {};
    if (!fill_addr(addr, sock_path)) { ::close(fd); return false; }

    if (::bind(fd, (struct sockaddr*)&addr, sizeof addr) != 0) {
        std::fprintf(stderr, "[wbr-gpsd] bind %s: %s\n",
                     sock_path.c_str(), std::strerror(errno));
        ::close(fd);
        return false;
    }

    // Tighten the mode between bind() and listen(). The inode exists from
    // bind() onward but nothing can connect until listen(), so setting
    // permissions here closes the window in which the socket would be
    // reachable at whatever the umask happened to allow.
    if (::chmod(sock_path.c_str(), mode) != 0) {
        std::fprintf(stderr, "[wbr-gpsd] chmod %s: %s\n",
                     sock_path.c_str(), std::strerror(errno));
    }
    if (group != nullptr) {
        const struct group* g = ::getgrnam(group);
        if (g == nullptr) {
            std::fprintf(stderr, "[wbr-gpsd] no such group %s\n", group);
        } else if (::chown(sock_path.c_str(), (uid_t)-1, g->gr_gid) != 0) {
            std::fprintf(stderr, "[wbr-gpsd] chown %s to group %s: %s\n",
                         sock_path.c_str(), group, std::strerror(errno));
        }
    }

    if (::listen(fd, 64) != 0) {
        std::fprintf(stderr, "[wbr-gpsd] listen %s: %s\n",
                     sock_path.c_str(), std::strerror(errno));
        ::close(fd);
        ::unlink(sock_path.c_str());         // do not leave our own debris
        return false;
    }

    listen_fd_ = fd;
    // Set last, and only on success: a Server that failed to bind must never
    // unlink the socket belonging to the daemon that did.
    sock_path_ = sock_path;
    return true;
}

int Server::accept_client()
{
    if (listen_fd_ < 0) return -1;
    const int fd = ::accept4(listen_fd_, nullptr, nullptr,
                             SOCK_NONBLOCK | SOCK_CLOEXEC);
    if (fd < 0) return -1;

    Client& c = clients_[fd];
    c = Client{};                            // fd numbers are reused

    char hello[128];
    std::snprintf(hello, sizeof hello,
                  "{\"class\":\"HELLO\",\"proto\":%d,\"daemon\":\"wbr-gpsd/1.0\"}",
                  kProtoVersion);
    enqueue(c, hello);
    return fd;
}

void Server::drop_client(int fd)
{
    auto it = clients_.find(fd);
    if (it == clients_.end()) return;
    clients_.erase(it);
    ::close(fd);
}

size_t Server::first_editable(const Client& c)
{
    if (!c.head_partial) return 0;
    const size_t nl = c.out.find('\n');
    return (nl == std::string::npos) ? std::string::npos : nl + 1;
}

bool Server::drop_oldest(Client& c)
{
    const size_t start = first_editable(c);
    if (start == std::string::npos || start >= c.out.size()) return false;
    const size_t nl = c.out.find('\n', start);
    if (nl == std::string::npos) return false;      // no whole line to drop

    const size_t end     = nl + 1;
    const size_t removed = end - start;
    c.out.erase(start, removed);

    if (c.fix_at != std::string::npos) {
        if (c.fix_at >= end)        c.fix_at -= removed;
        else if (c.fix_at >= start) c.fix_at = std::string::npos;  // it was the drop
    }
    ++c.dropped;
    return true;
}

size_t Server::enqueue(Client& c, const std::string& line)
{
    const size_t need = line.size() + 1;
    if (need > kMaxQueueBytes) {
        // A single message larger than the entire budget. Refuse it rather
        // than blow the bound; only NMEA passthrough could ever get close.
        ++c.dropped;
        return std::string::npos;
    }
    // Drop as many whole lines as it takes to fit. Dropping exactly one is
    // not enough: the line freed can be shorter than the line appended, and
    // the head of the queue may be a part-sent line that cannot be dropped
    // at all. This is guarantee G2 -- the queue is bounded, not merely
    // trimmed. If the only thing left is an undroppable part-sent line we
    // append anyway; that overshoot is at most one message.
    while (c.out.size() + need > kMaxQueueBytes && drop_oldest(c)) { }

    const size_t at = c.out.size();
    c.out += line;
    c.out += '\n';
    return at;
}

void Server::broadcast_fix(const StateStore& store, int64_t now_mono_ms)
{
    const std::string line = snapshot_to_json(store.current(), now_mono_ms);
    for (auto& kv : clients_) {
        Client& c = kv.second;
        if (!c.want_fix) continue;

        // Coalesce: if a FIX is queued and still entirely unsent, replace it.
        // Only the newest snapshot has meaning, so a slow client's queue
        // cannot grow. A FIX whose leading bytes already reached the kernel
        // is off limits -- rewriting it would splice two messages on the wire.
        const size_t editable = first_editable(c);
        if (c.fix_at != std::string::npos && editable != std::string::npos &&
            c.fix_at >= editable && c.fix_at < c.out.size()) {
            const size_t end = c.out.find('\n', c.fix_at);
            if (end != std::string::npos) {
                c.out.replace(c.fix_at, end - c.fix_at, line);
                // The replacement can be longer than what it replaced.
                while (c.out.size() > kMaxQueueBytes && drop_oldest(c)) { }
                continue;
            }
        }
        c.fix_at = enqueue(c, line);
    }
}

void Server::broadcast_nmea(const std::string& raw, int64_t t_unix_ms)
{
    for (auto& kv : clients_) {
        Client& c = kv.second;
        if (!c.want_nmea) continue;
        // NMEA cannot coalesce: every sentence is distinct data. Oldest is
        // dropped by enqueue() and reported via the dropped counter.
        enqueue(c, nmea_to_json(raw, t_unix_ms, c.dropped));
    }
}

void Server::handle_request(Client& c, const std::string& line,
                            StateStore& store, int64_t now_mono_ms)
{
    std::string op;
    if (!json_get_string(line, "op", op)) {
        enqueue(c, "{\"class\":\"ERROR\",\"msg\":\"missing op\"}");
        return;
    }

    if (op == "get") {
        // Deliberately does NOT subscribe. The Django badge connects, asks
        // once and closes; subscribing it would queue pushes for nobody.
        // fix_at is still recorded so that a push landing before this line is
        // flushed replaces it, rather than arriving in front of it and
        // leaving a stale snapshot behind a fresh one on the wire.
        c.fix_at = enqueue(c, snapshot_to_json(store.current(), now_mono_ms));
        return;
    }
    if (op == "watch") {
        bool want_fix = true, want_nmea = false;
        json_get_bool(line, "fix", want_fix);
        json_get_bool(line, "nmea", want_nmea);
        c.want_fix  = want_fix;
        c.want_nmea = want_nmea;
        // Send the current snapshot immediately (scenario S5), so a client
        // joining mid-stream does not wait up to a second for the next GGA.
        c.fix_at = enqueue(c, snapshot_to_json(store.current(), now_mono_ms));
        return;
    }
    if (op == "unwatch") {
        c.want_fix  = false;
        c.want_nmea = false;
        return;
    }
    enqueue(c, "{\"class\":\"ERROR\",\"msg\":\"unknown op\"}");
}

bool Server::on_client_readable(int fd, StateStore& store, int64_t now_mono_ms)
{
    auto it = clients_.find(fd);
    if (it == clients_.end()) return false;
    Client& c = it->second;

    char io[1024];
    // Bound the work one readiness event may cost. A peer that writes without
    // pause would otherwise keep read() returning data forever and this loop
    // would never hand control back -- one client blocking the daemon, which
    // is precisely what G3 forbids. Whatever is left is picked up on the next
    // pass of the epoll loop, interleaved with every other fd.
    for (int pass = 0; pass < kMaxReadsPerEvent; ++pass) {
        const ssize_t n = ::read(fd, io, sizeof io);
        if (n > 0) {
            c.in.append(io, (size_t)n);
            // The cap is on one request line, checked whether or not the line
            // has arrived complete. Checking buffer occupancy instead would
            // both reject legitimate bursts and let a complete oversized line
            // through on the read that happens to deliver its newline.
            // No error is queued on refusal: the caller drops this client
            // immediately, so nothing queued here could ever be written.
            for (;;) {
                const size_t nl = c.in.find('\n');
                if (nl == std::string::npos) break;
                if (nl > kMaxRequestBytes) {
                    std::fprintf(stderr,
                                 "[wbr-gpsd] client fd %d sent a %zu byte request line, dropping\n",
                                 fd, nl);
                    return false;
                }
                const std::string line = c.in.substr(0, nl);
                c.in.erase(0, nl + 1);
                if (!line.empty()) handle_request(c, line, store, now_mono_ms);
            }
            if (c.in.size() > kMaxRequestBytes) {
                std::fprintf(stderr,
                             "[wbr-gpsd] client fd %d has %zu buffered bytes with no newline, dropping\n",
                             fd, c.in.size());
                return false;
            }
            continue;
        }
        // On a socket, read() == 0 really is EOF, unlike the tty in
        // serial_source where it only means "no bytes right now".
        if (n == 0) return false;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return true;
        if (errno == EINTR) continue;
        return false;    // hard error: peer is gone
    }
    return true;         // budget spent, not an error: come back next pass
}

bool Server::flush_client(int fd)
{
    auto it = clients_.find(fd);
    if (it == clients_.end()) return false;
    Client& c = it->second;

    while (!c.out.empty()) {
        // MSG_NOSIGNAL: a disconnecting client must never kill the daemon.
        const ssize_t n = ::send(fd, c.out.data(), c.out.size(), MSG_NOSIGNAL);
        if (n > 0) {
            const size_t w = (size_t)n;
            // Whether the peer now holds a whole number of lines decides
            // whether out[0] becomes a fragment we may no longer touch.
            const bool ended_on_newline = (c.out[w - 1] == '\n');
            c.out.erase(0, w);
            c.head_partial = !c.out.empty() && !ended_on_newline;
            if (c.fix_at != std::string::npos) {
                // fix_at == w means the FIX starts at the first unsent byte,
                // so it survives intact at offset 0. Anything below w had its
                // first bytes written already and can never be replaced.
                c.fix_at = (c.fix_at >= w) ? c.fix_at - w : std::string::npos;
            }
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return true;
        if (n < 0 && errno == EINTR) continue;
        return false;    // EPIPE or similar: peer is gone
    }
    c.fix_at       = std::string::npos;
    c.head_partial = false;
    return true;
}

bool Server::wants_write(int fd) const
{
    const auto it = clients_.find(fd);
    return it != clients_.end() && !it->second.out.empty();
}

size_t Server::pending_bytes(int fd) const
{
    const auto it = clients_.find(fd);
    return it == clients_.end() ? 0 : it->second.out.size();
}

void Server::shutdown()
{
    for (auto& kv : clients_) ::close(kv.first);
    clients_.clear();
    if (listen_fd_ >= 0) { ::close(listen_fd_); listen_fd_ = -1; }
    if (!sock_path_.empty()) { ::unlink(sock_path_.c_str()); sock_path_.clear(); }
    if (lock_fd_ >= 0) { ::close(lock_fd_); lock_fd_ = -1; }
}

} // namespace wbr_gps
