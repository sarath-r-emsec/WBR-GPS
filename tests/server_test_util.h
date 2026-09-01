#pragma once

// Shared fixtures for the two Server suites: test_server_protocol.cpp covers
// the request/response protocol and process lifecycle, test_server_backpressure
// .cpp covers queueing, overflow and adversarial peers. Both are separate
// executables, so this header is included exactly once per binary.

#include "wbr_gps/server.hpp"
#include "test_util.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>

// Every socket lives in its own private directory so tests cannot collide.
// The directories are removed together at the end of the run, matching the
// cleanup convention in test_device_presence.cpp.
inline std::vector<std::string> g_tmp_dirs;

inline std::string tmp_sock()
{
    char tmpl[] = "/tmp/wbrgps_sock_XXXXXX";
    const char* d = mkdtemp(tmpl);
    if (d == nullptr) { fprintf(stderr, "mkdtemp failed\n"); abort(); }
    g_tmp_dirs.push_back(d);
    return std::string(d) + "/gpsd.sock";
}

inline void cleanup_tmp_dirs()
{
    for (const std::string& d : g_tmp_dirs) {
        ::unlink((d + "/gpsd.sock").c_str());
        ::rmdir(d.c_str());
    }
    g_tmp_dirs.clear();
}

inline int connect_client(const std::string& path)
{
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof addr.sun_path, "%s", path.c_str());
    if (::connect(fd, (struct sockaddr*)&addr, sizeof addr) != 0) { ::close(fd); return -1; }
    return fd;
}

inline std::string read_line(int fd)
{
    std::string out;
    char c;
    while (::read(fd, &c, 1) == 1) {
        if (c == '\n') break;
        out += c;
    }
    return out;
}

inline void send_line(int fd, const std::string& s)
{
    const std::string framed = s + "\n";
    ssize_t rc = ::write(fd, framed.data(), framed.size());
    (void)rc;
}

inline void set_nonblock_fd(int fd)
{
    const int fl = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

// Shrink the kernel socket buffers so send() is forced into partial writes
// after only a few hundred bytes. Without this the 200 KB default buffer
// hides every partial-write path in the server.
inline void shrink_buffers(int server_fd, int client_fd)
{
    int sz = 2048;
    ::setsockopt(server_fd, SOL_SOCKET, SO_SNDBUF, &sz, sizeof sz);
    ::setsockopt(client_fd, SOL_SOCKET, SO_RCVBUF, &sz, sizeof sz);
}

// Read whatever is available right now. Returns bytes appended; -1 on EOF.
inline ssize_t drain_available(int fd, std::string& sink, size_t max_bytes)
{
    char buf[512];
    size_t got = 0;
    while (got < max_bytes) {
        const size_t want = std::min(sizeof buf, max_bytes - got);
        const ssize_t n = ::read(fd, buf, want);
        if (n > 0) { sink.append(buf, (size_t)n); got += (size_t)n; continue; }
        if (n == 0) return -1;                                   // EOF
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        if (errno == EINTR) continue;
        return -1;
    }
    return (ssize_t)got;
}

// Split a byte stream into complete newline-terminated lines. A trailing
// fragment (no newline yet) is left in `rest`.
inline std::vector<std::string> split_lines(const std::string& s, std::string& rest)
{
    std::vector<std::string> out;
    size_t start = 0;
    for (;;) {
        const size_t nl = s.find('\n', start);
        if (nl == std::string::npos) break;
        out.push_back(s.substr(start, nl - start));
        start = nl + 1;
    }
    rest = s.substr(start);
    return out;
}
