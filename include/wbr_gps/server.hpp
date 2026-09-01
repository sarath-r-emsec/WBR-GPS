#pragma once

#include "wbr_gps/state_store.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <sys/types.h>

namespace wbr_gps {

// Unix socket server. Single-threaded: every method is called from the one
// epoll loop in main.cpp, which is why there is no locking anywhere.
//
// Two guarantees live in this class:
//   G2  no client can block another -- every output queue is bounded and
//       every write is non-blocking.
//   G3  no client can block the daemon -- no call here ever waits on a peer.
//
// Startup ordering matters: acquire_singleton() MUST be called, and MUST
// have succeeded, before listen_on(). The flock is the real defence against
// two daemons; the stale-socket probe in listen_on() is only a convenience
// and is inherently racy between two processes starting at the same instant.
class Server {
public:
    // A queue larger than this means the client is not keeping up. FIX
    // messages coalesce; NMEA messages drop oldest.
    static constexpr size_t kMaxQueueBytes   = 64 * 1024;
    // Longest request line we will buffer from a client before dropping it.
    static constexpr size_t kMaxRequestBytes = 4096;
    // Reads granted to one client per readiness event. on_client_readable()
    // deliberately returns before EAGAIN once this is spent, so that a peer
    // writing flat out cannot monopolise the loop. The consequence is that
    // client fds MUST be registered level triggered -- the epoll default.
    // With EPOLLET a client would stall holding unread bytes.
    static constexpr int    kMaxReadsPerEvent = 16;

    ~Server();

    // flock() a pidfile so a second daemon cannot start. Returns false if
    // another instance holds it.
    bool acquire_singleton(const std::string& lock_path);

    // Bind and listen. If `group` is non-null, chown the socket to it. A
    // stale socket file is only unlinked when nothing is listening on it.
    bool listen_on(const std::string& sock_path, const char* group, mode_t mode);

    int  listen_fd() const { return listen_fd_; }
    int  accept_client();
    void drop_client(int fd);
    size_t client_count() const { return clients_.size(); }

    // Read and handle requests. Returns false when the peer has gone away.
    bool on_client_readable(int fd, StateStore& store, int64_t now_mono_ms);

    // Enqueue to every subscriber. Never blocks.
    void broadcast_fix(const StateStore& store, int64_t now_mono_ms);
    void broadcast_nmea(const std::string& raw, int64_t t_unix_ms);

    // Try to drain one client's queue. Returns false if the peer is gone.
    bool   flush_client(int fd);
    bool   wants_write(int fd) const;
    size_t pending_bytes(int fd) const;

    void shutdown();

private:
    struct Client {
        std::string in;         // partial request line
        std::string out;        // pending output bytes
        bool        want_fix  = false;
        bool        want_nmea = false;
        // True when out[0] sits in the middle of a line whose leading bytes
        // are already in the kernel's send buffer. Those bytes are gone; the
        // remainder of that line must never be dropped, replaced or reordered,
        // or the peer receives two messages spliced into one -- the exact
        // stream corruption this daemon exists to eliminate.
        bool        head_partial = false;
        // Index into `out` where the last queued FIX begins, so a newer FIX
        // can replace it instead of queueing behind it. npos = none pending.
        size_t      fix_at  = std::string::npos;
        uint64_t    dropped = 0;
    };

    // Offset of the first byte of `out` that has never been handed to the
    // kernel, i.e. the first byte it is safe to edit. npos when the whole
    // buffer is the tail of one part-sent line.
    static size_t first_editable(const Client& c);
    // Drop the oldest whole line that has not been partly sent. Bumps the
    // dropped counter. Returns false when there is nothing droppable.
    static bool   drop_oldest(Client& c);
    // Append one line plus its newline, dropping oldest lines first if the
    // cap would be exceeded. Returns the offset where the line starts, or
    // npos if the line was refused.
    static size_t enqueue(Client& c, const std::string& line);

    void handle_request(Client& c, const std::string& line,
                        StateStore& store, int64_t now_mono_ms);

    int         listen_fd_ = -1;
    int         lock_fd_   = -1;
    std::string sock_path_;
    std::map<int, Client> clients_;
};

} // namespace wbr_gps
