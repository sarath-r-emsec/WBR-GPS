#pragma once

// Daemon-internal, NOT part of the client API. Nothing under python/ and
// nothing in client.hpp / client.cpp may include this; it exists so the one
// piece of genuinely novel logic in main.cpp can be unit-tested without
// launching a process.
//
// Extracted verbatim from src/main.cpp (Task 12, recommendation A1). The rest
// of main.cpp is sequencing -- open this, register that, then loop -- and is
// covered end-to-end by tests/test_concurrency.cpp against the real binary.
// What lives here is the EPOLLOUT arm/disarm invariant, which has a failure
// mode no functional test can see: a permanently registered EPOLLOUT still
// answers every request correctly, it just does so while burning 100% of a
// core. Only a direct assertion on the epoll registration catches that, and
// that assertion needs this class reachable from a test.

#include "wbr_gps/server.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <map>
#include <sys/epoll.h>
#include <vector>

namespace wbr_gps {

// Hard ceiling on concurrent clients. Two independent reasons, and the
// tighter of the two is memory, not file descriptors:
//
//   * Memory. Each client can hold up to Server::kMaxQueueBytes = 64 KB of
//     output queue, and the Task 11 systemd unit sets MemoryMax=64M. At
//     64 KB apiece, 1024 clients is 64 MB of queues alone, so the daemon
//     would be OOM-killed by its own unit before it ever ran out of file
//     descriptors. 64 clients caps queue memory at 64 * 64 KB = 4 MB, which
//     is 6.25% of the unit's ceiling and leaves the rest to the program.
//   * Spin. Refusing past the cap keeps draining the accept queue, so the
//     level-triggered listener stops reporting ready. See T8-A.
//
// The real client count is about four (the Django badge, the GUI, the
// monitor, a debug shell), so 64 is generous by an order of magnitude.
// If you change this, check the MemoryMax arithmetic above; if you change
// MemoryMax in the unit, check this.
constexpr size_t  kMaxClients    = 64;

// --- epoll helpers ----------------------------------------------------------

inline bool epoll_change(int ep, int op, int fd, uint32_t events)
{
    struct epoll_event ev {};
    ev.events  = events;
    ev.data.fd = fd;
    if (::epoll_ctl(ep, op, fd, &ev) == 0) return true;
    std::fprintf(stderr, "[wbr-gpsd] epoll_ctl(%s, fd %d): %s\n",
                 op == EPOLL_CTL_ADD ? "add" : "mod", fd, std::strerror(errno));
    return false;
}

inline bool epoll_add(int ep, int fd, uint32_t events)
{
    return epoll_change(ep, EPOLL_CTL_ADD, fd, events);
}

inline bool epoll_mod(int ep, int fd, uint32_t events)
{
    return epoll_change(ep, EPOLL_CTL_MOD, fd, events);
}

inline void epoll_drop(int ep, int fd)
{
    ::epoll_ctl(ep, EPOLL_CTL_DEL, fd, nullptr);   // best effort by design
}

// --- client registration ----------------------------------------------------

// The one thing about a client that Server deliberately does not track: how
// its fd is currently registered with epoll.
//
// EPOLLOUT is armed ONLY while bytes are actually queued for that client.
// Registering it up front looks harmless and is not: a socket with room in
// its send buffer is writable, so a permanently armed EPOLLOUT makes
// epoll_wait() return immediately on every single pass. The daemon would sit
// at 100% of a core while still answering every request correctly, so every
// functional test would pass and only a CPU measurement would catch it.
class ClientTable {
public:
    ClientTable(int ep, Server& server) : ep_(ep), server_(server) {}

    // A freshly accepted client already has a HELLO queued by
    // accept_client(), so it goes straight through sync() and normally
    // leaves with an empty queue and no EPOLLOUT armed.
    void add(int fd)
    {
        // The cap is enforced here rather than by declining to accept,
        // because the connection must still come off the accept queue.
        // Leaving it there would keep the level-triggered listener readable
        // forever. accept_client() has already inserted this client, so the
        // count includes it.
        if (server_.client_count() > kMaxClients) {
            if (!cap_logged_) {
                std::fprintf(stderr,
                             "[wbr-gpsd] at the %zu client cap, refusing further connections\n",
                             kMaxClients);
                cap_logged_ = true;   // cleared only when the count drops back
            }
            server_.drop_client(fd);          // accepted, then closed at once
            return;
        }

        // Level triggered, and EPOLLIN only. Level triggered is required, not
        // stylistic: Server::on_client_readable() yields after
        // kMaxReadsPerEvent reads instead of looping to EAGAIN, so with
        // EPOLLET a client that sent more than that in one burst would sit
        // holding unread bytes with no further event ever arriving.
        if (!epoll_add(ep_, fd, EPOLLIN)) {
            server_.drop_client(fd);
            return;
        }
        armed_out_[fd] = false;
        sync(fd);
    }

    bool has(int fd) const { return armed_out_.count(fd) != 0; }

    void remove(int fd)
    {
        if (!has(fd)) return;
        epoll_drop(ep_, fd);        // must precede the close in drop_client()
        server_.drop_client(fd);
        armed_out_.erase(fd);
        // Re-arm the cap message only once we are genuinely back under it. A
        // sustained flood therefore logs once: refused peers never enter
        // clients_, so the count does not move while the flood lasts.
        if (server_.client_count() < kMaxClients) cap_logged_ = false;
    }

    // Write whatever the kernel will take, then make the EPOLLOUT
    // registration match what is left over. This is the arm/disarm half of
    // the rule above, and it replaces the inert placeholder loop that stood
    // in for it in the task brief.
    void sync(int fd)
    {
        const auto it = armed_out_.find(fd);
        if (it == armed_out_.end()) return;

        if (!server_.flush_client(fd)) { remove(fd); return; }

        const bool want = server_.wants_write(fd);
        if (want == it->second) return;          // registration already correct
        if (!epoll_mod(ep_, fd, want ? (EPOLLIN | EPOLLOUT) : EPOLLIN)) {
            remove(fd);
            return;
        }
        it->second = want;
    }

    // sync() can remove the client it is given, so the fds are snapshotted
    // before the walk rather than iterating the map being mutated.
    void sync_all()
    {
        scratch_.clear();
        scratch_.reserve(armed_out_.size());
        for (const auto& kv : armed_out_) scratch_.push_back(kv.first);
        for (const int fd : scratch_) sync(fd);
    }

private:
    int                  ep_;
    Server&              server_;
    std::map<int, bool>  armed_out_;   // fd -> is EPOLLOUT currently armed
    std::vector<int>     scratch_;
    bool                 cap_logged_ = false;
};

} // namespace wbr_gps
