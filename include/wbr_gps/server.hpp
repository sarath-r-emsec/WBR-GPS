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

    Server() = default;
    ~Server();

    // T7-B, rule of three. This object owns the listening fd, the lock fd and
    // every client fd. A copy would double-close descriptors the process may
    // have since reissued to something else, and double-unlink the socket.
    // Declaring the destructor already suppresses the implicit moves, so
    // Server is neither copyable nor movable.
    Server(const Server&)            = delete;
    Server& operator=(const Server&) = delete;

    // flock() a pidfile so a second daemon cannot start. Returns false if
    // another instance holds it. Calling it again with the same path is a
    // no-op that returns true; calling it with a *different* path returns
    // false, because this Server holds no lock on that path and must not
    // report one it does not hold.
    bool acquire_singleton(const std::string& lock_path);

    // Bind and listen. If `group` is non-null, chown the socket to it. A
    // stale socket file is only unlinked when nothing is listening on it.
    bool listen_on(const std::string& sock_path, const char* group, mode_t mode);

    int  listen_fd() const { return listen_fd_; }

    // accept_client() returns a non-negative fd, or one of the two codes
    // below. They are NOT interchangeable. kAcceptDrained means the backlog
    // is empty and the caller should stop looping until the next readiness
    // event. kAcceptFailed means the accept itself failed -- EMFILE, ENFILE,
    // ENOMEM -- and is the case a bare -1 used to hide: under fd exhaustion
    // the pending connection stays in the accept queue AND the listening
    // socket stays level-triggered readable, so a caller that reads the
    // failure as "drained" is handed a socket that reports ready forever and
    // spins at 100% of a core. Callers must stop accepting and back the
    // listener off; retrying immediately cannot clear the condition. T8-A.
    static constexpr int kAcceptDrained = -1;
    static constexpr int kAcceptFailed  = -2;

    int  accept_client();
    void drop_client(int fd);
    size_t client_count() const { return clients_.size(); }

    // Read and handle requests. Returns false when this client must be
    // dropped -- either the peer has gone away, or it broke the protocol
    // (an over-long request line) and we are dropping it deliberately.
    // The caller must call drop_client(fd) on false.
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
    // Lets the test suite drive enqueue() directly. The newline guard it
    // enforces is unreachable through the public API today because json_io's
    // escape() already strips control bytes -- which is exactly why the guard
    // must be proven to fire rather than inferred from another file's
    // behaviour. See T7-A.
    friend struct ServerTestAccess;

    struct Client {
        std::string in;         // partial request line
        // Pending output bytes. INVARIANT: every message in here is exactly
        // one line, because every boundary is located with find('\n').
        // There are exactly two places that write content into `out` --
        // enqueue() and the in-place coalesce in broadcast_fix_line() -- and
        // BOTH must reject a line containing a newline before writing it.
        // Guarding only one relocates the dependency instead of removing it.
        // See is_single_line(). T7-A, T7-E.
        std::string out;
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

    // The queued-message invariant, with one definition rather than a magic
    // find('\n') copied to each writer. See Client::out.
    static bool is_single_line(const std::string& line)
    {
        return line.find('\n') == std::string::npos;
    }

    // broadcast_fix() with the snapshot line already rendered. Separated so
    // the coalesce branch can be driven directly by the test suite: the guard
    // protecting it is otherwise unreachable, because snapshot_to_json()
    // cannot emit a newline today -- which is exactly why it must be tested
    // rather than assumed.
    void broadcast_fix_line(const std::string& line);

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
    std::string lock_path_;    // path lock_fd_ was taken on; see T7-C
    std::map<int, Client> clients_;
};

} // namespace wbr_gps
