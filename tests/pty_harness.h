#pragma once

// A pseudo-terminal standing in for the Leo Bodnar serial port, so tests run
// in CI with no hardware and without root.
//
// NOTE: a pty is not a USB CDC-ACM device. Unplug/replug behaviour, TIOCEXCL
// semantics and ModemManager interaction can only be verified on real
// hardware — see the spec, section 7.6.

#include <fcntl.h>
#include <pty.h>
#include <string>
#include <unistd.h>

struct PtyPair {
    int         master = -1;
    int         slave  = -1;
    std::string slave_path;

    bool open_pair()
    {
        char name[256];
        if (openpty(&master, &slave, name, nullptr, nullptr) != 0) return false;
        slave_path = name;
        return true;
    }

    // Write as if the GPS had emitted this sentence.
    void emit(const std::string& line)
    {
        const std::string framed = line + "\r\n";
        ssize_t rc = ::write(master, framed.data(), framed.size());
        (void)rc;
    }

    // Emit raw bytes with no framing, for torn-read simulation.
    void emit_raw(const std::string& bytes)
    {
        ssize_t rc = ::write(master, bytes.data(), bytes.size());
        (void)rc;
    }

    void close_pair()
    {
        if (master >= 0) { ::close(master); master = -1; }
        if (slave  >= 0) { ::close(slave);  slave  = -1; }
    }

    ~PtyPair() { close_pair(); }
};
