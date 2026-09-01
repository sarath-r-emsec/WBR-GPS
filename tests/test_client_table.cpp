// ClientTable: the arm/disarm invariant, asserted directly.
//
// Why this file exists. Everything else in src/main.cpp is sequencing, and
// tests/test_concurrency.cpp drives all of it end-to-end against the real
// binary. ClientTable is the exception: it holds a rule whose violation is
// invisible to every functional test ever written for this daemon.
//
// The rule is that EPOLLOUT is armed ONLY while bytes are actually queued for
// a client. Arm it permanently and the daemon still answers every request
// correctly -- S1 through S15 all stay green -- while epoll_wait() returns
// instantly on every pass and the process sits at 100% of a core. Task 8
// fixed exactly that defect. Nothing has protected it since.
//
// So the assertions below are about the epoll REGISTRATION, not about
// answers. There is no epoll "read my registration" call, so the
// registration is observed the way the kernel exposes it: an idle client
// socket with EPOLLOUT armed is immediately ready, and one without it is not.
// epoll_wait() with a zero timeout is therefore a direct readout of the bug.

#include "client_table.hpp"
#include "server_test_util.h"

#include <string>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

using wbr_gps::ClientTable;
using wbr_gps::Server;
using wbr_gps::StateStore;
using wbr_gps::kMaxClients;

namespace {

// A Server on a private socket plus its own epoll set. No singleton lock:
// this suite never starts a second one, and Server does not require it.
struct Fixture {
    Server      server;
    int         ep = -1;
    std::string sock;
    std::vector<int> peers;      // our end of every connection we opened

    bool up()
    {
        sock = tmp_sock();
        if (!server.listen_on(sock, nullptr, 0660)) return false;
        ep = ::epoll_create1(EPOLL_CLOEXEC);
        return ep >= 0;
    }

    // Connect, accept, register. Returns {peer_fd, server_side_fd}.
    // server_side_fd is -1 if the accept failed.
    int connect_and_add(ClientTable& clients, bool shrink = false)
    {
        const int peer = connect_client(sock);
        if (peer < 0) return -1;
        peers.push_back(peer);
        const int fd = server.accept_client();
        if (fd < 0) return -1;
        if (shrink) shrink_buffers(fd, peer);
        clients.add(fd);
        return fd;
    }

    // Every exit path closes everything: this suite runs alongside the
    // concurrency soak tests, which measure fd and inode pressure and are
    // worthless from a polluted baseline.
    ~Fixture()
    {
        for (const int p : peers) if (p >= 0) ::close(p);
        if (ep >= 0) ::close(ep);
        server.shutdown();
    }
};

// The readiness mask epoll currently reports for `fd`, right now.
uint32_t ready_mask(int ep, int fd)
{
    struct epoll_event evs[16];
    const int n = ::epoll_wait(ep, evs, 16, 0);
    uint32_t mask = 0;
    for (int i = 0; i < n; ++i) {
        if (evs[i].data.fd == fd) mask |= evs[i].events;
    }
    return mask;
}

// How many fds epoll reports ready at this instant, over the whole set.
int ready_count(int ep)
{
    struct epoll_event evs[128];
    return ::epoll_wait(ep, evs, 128, 0);
}

// ---------------------------------------------------------------------------

// THE regression test for the Task 8 spin defect.
void idle_client_is_not_registered_for_write()
{
    Fixture f;
    ASSERT_TRUE(f.up());
    ClientTable clients(f.ep, f.server);

    const int fd = f.connect_and_add(clients);
    ASSERT_TRUE(fd >= 0);
    ASSERT_TRUE(clients.has(fd));

    // add() queued a HELLO and sync() wrote it, so the queue is empty and the
    // peer is not sending anything. A correctly registered client is now
    // reported ready by nothing at all.
    //
    // Ten consecutive passes rather than one: a spin is defined by recurring
    // on every pass, and a single sample could be a one-off wakeup.
    for (int pass = 0; pass < 10; ++pass) {
        ASSERT_EQ(ready_count(f.ep), 0);
    }

    // The queue really is empty -- otherwise the assertion above would be
    // passing for the wrong reason.
    ASSERT_EQ(f.server.pending_bytes(fd), (size_t)0);
    ASSERT_FALSE(f.server.wants_write(fd));
}

// The other half of the rule: when bytes ARE queued, EPOLLOUT must be armed,
// or the leftovers sit in the queue until the next unrelated wakeup.
void epollout_is_armed_while_bytes_are_pending()
{
    Fixture f;
    ASSERT_TRUE(f.up());
    ClientTable clients(f.ep, f.server);
    StateStore  store;

    // 2 KB socket buffers, so a few hundred bytes already will not fit.
    const int fd = f.connect_and_add(clients, /*shrink=*/true);
    ASSERT_TRUE(fd >= 0);

    // Subscribe, through the real request path. Note that a watch is answered
    // with an immediate snapshot -- roughly 250 bytes -- so the queue is
    // already non-empty on return. sync() it out before measuring anything,
    // or the fill loop below never runs and the assertions pass vacuously.
    send_line(f.peers.back(), "{\"op\":\"watch\",\"fix\":false,\"nmea\":true}");
    ASSERT_TRUE(f.server.on_client_readable(fd, store, wbr_gps::now_mono_ms()));
    clients.sync(fd);

    // Fill it while the peer reads nothing. The loop is written so that a
    // sync() is always the last thing to happen before the check: it is
    // sync() that owns the registration, so testing wants_write() without one
    // in between would be asking the wrong object.
    for (int i = 0; i < 400; ++i) {
        f.server.broadcast_nmea("$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F", 0);
        clients.sync(fd);
        if (f.server.wants_write(fd)) break;
    }
    ASSERT_TRUE(f.server.wants_write(fd));
    ASSERT_TRUE(f.server.pending_bytes(fd) > 0);

    // Now, and only now, the fd must be registered for write.
    //
    // "Registered" and "ready" are not the same thing, and the difference
    // matters here: the send buffer is full, which is precisely why bytes are
    // still queued, and a full socket is not writable no matter how it is
    // registered. So make room first by reading from the peer WITHOUT
    // syncing. That leaves the daemon-side queue non-empty -- still wanting
    // EPOLLOUT -- while the socket becomes writable, and epoll can finally
    // answer the question being asked.
    std::string warm;
    set_nonblock_fd(f.peers.back());
    ASSERT_TRUE(drain_available(f.peers.back(), warm, 4096) > 0);
    ASSERT_TRUE(f.server.wants_write(fd));
    ASSERT_TRUE((ready_mask(f.ep, fd) & EPOLLOUT) != 0);

    // Drain from the peer until the daemon-side queue is empty...
    std::string sink;
    for (int i = 0; i < 500 && f.server.pending_bytes(fd) > 0; ++i) {
        set_nonblock_fd(f.peers.back());
        drain_available(f.peers.back(), sink, 8192);
        clients.sync(fd);
    }
    ASSERT_EQ(f.server.pending_bytes(fd), (size_t)0);

    // ...and the registration must have gone back with it. This is the
    // disarm half; without it the daemon spins from the first busy client
    // onwards and never recovers.
    ASSERT_FALSE(f.server.wants_write(fd));
    ASSERT_EQ(ready_count(f.ep), 0);
}

void cap_is_enforced_at_registration()
{
    Fixture f;
    ASSERT_TRUE(f.up());
    ClientTable clients(f.ep, f.server);

    std::vector<int> fds;
    const size_t over = kMaxClients + 5;
    for (size_t i = 0; i < over; ++i) fds.push_back(f.connect_and_add(clients));

    // The connections past the cap were accepted -- they must come off the
    // accept queue, or the level-triggered listener reports ready forever --
    // and then closed at once. So the count settles exactly at the cap.
    ASSERT_EQ(f.server.client_count(), kMaxClients);

    size_t registered = 0, refused = 0;
    for (const int fd : fds) {
        if (fd < 0) continue;
        if (clients.has(fd)) ++registered; else ++refused;
    }
    ASSERT_EQ(registered, kMaxClients);
    ASSERT_EQ(refused, over - kMaxClients);
}

void remove_frees_a_slot_and_deregisters()
{
    Fixture f;
    ASSERT_TRUE(f.up());
    ClientTable clients(f.ep, f.server);

    const int a = f.connect_and_add(clients);
    const int b = f.connect_and_add(clients);
    ASSERT_TRUE(a >= 0 && b >= 0);
    ASSERT_EQ(f.server.client_count(), (size_t)2);

    clients.remove(a);
    ASSERT_FALSE(clients.has(a));
    ASSERT_TRUE(clients.has(b));
    ASSERT_EQ(f.server.client_count(), (size_t)1);

    // Idempotent: a second remove of the same fd must not touch the Server
    // again. The fd number may already have been reissued by the kernel to
    // something else entirely by then.
    clients.remove(a);
    ASSERT_EQ(f.server.client_count(), (size_t)1);
}

void unknown_fd_is_ignored_rather_than_acted_on()
{
    Fixture f;
    ASSERT_TRUE(f.up());
    ClientTable clients(f.ep, f.server);

    const int fd = f.connect_and_add(clients);
    ASSERT_TRUE(fd >= 0);

    // An fd this table never registered must be inert here, not dropped from
    // the Server behind the loop's back.
    ASSERT_FALSE(clients.has(9999));
    clients.sync(9999);
    clients.remove(9999);
    ASSERT_EQ(f.server.client_count(), (size_t)1);
    ASSERT_TRUE(clients.has(fd));
}

void sync_all_reaps_peers_that_vanished()
{
    Fixture f;
    ASSERT_TRUE(f.up());
    ClientTable clients(f.ep, f.server);
    StateStore  store;

    std::vector<int> fds;
    for (int i = 0; i < 3; ++i) fds.push_back(f.connect_and_add(clients));
    for (size_t i = 0; i < fds.size(); ++i) {
        ASSERT_TRUE(fds[i] >= 0);
        send_line(f.peers[i], "{\"op\":\"watch\",\"fix\":true,\"nmea\":false}");
        ASSERT_TRUE(f.server.on_client_readable(fds[i], store, wbr_gps::now_mono_ms()));
    }
    ASSERT_EQ(f.server.client_count(), (size_t)3);

    // Two peers disappear without a word.
    ::close(f.peers[0]); f.peers[0] = -1;
    ::close(f.peers[1]); f.peers[1] = -1;

    // sync_all() must notice on a write and reap them. A first write into a
    // closed socket can be accepted by the kernel before the RST comes back,
    // so this is allowed a few passes -- what matters is that it converges
    // and that the survivor is untouched.
    for (int i = 0; i < 20 && f.server.client_count() > 1; ++i) {
        f.server.broadcast_fix(store, wbr_gps::now_mono_ms());
        clients.sync_all();
    }
    ASSERT_EQ(f.server.client_count(), (size_t)1);
    ASSERT_TRUE(clients.has(fds[2]));
    ASSERT_FALSE(clients.has(fds[0]));
    ASSERT_FALSE(clients.has(fds[1]));
}

} // namespace

static void run_tests()
{
    idle_client_is_not_registered_for_write();
    epollout_is_armed_while_bytes_are_pending();
    cap_is_enforced_at_registration();
    remove_frees_a_slot_and_deregisters();
    unknown_fd_is_ignored_rather_than_acted_on();
    sync_all_reaps_peers_that_vanished();
    cleanup_tmp_dirs();
}

TEST_MAIN
