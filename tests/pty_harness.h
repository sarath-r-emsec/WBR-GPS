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

        // Close-on-exec, and this is load-bearing rather than tidy.
        //
        // Tests in this project fork and exec the real wbr-gpsd. openpty()
        // returns descriptors WITHOUT FD_CLOEXEC, so without these two calls
        // the daemon inherits the master -- and a pty hangs up only when the
        // last master descriptor closes. The test would then close its own
        // master, believe it had unplugged the device, and watch the daemon
        // go on reading happily from a port the test thought was gone. Every
        // unplug and reconnect scenario silently tests nothing.
        //
        // The daemon opens the slave by path, which is unaffected.
        ::fcntl(master, F_SETFD, FD_CLOEXEC);
        ::fcntl(slave,  F_SETFD, FD_CLOEXEC);
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

    // Hang up WITHOUT releasing the pts index: this is what an unplug
    // actually is -- the device side goes away -- and the slave descriptor
    // the harness holds is a pty artifact with no counterpart in hardware.
    //
    // Keeping it open matters. A pts index is recycled the instant it is
    // free, and a tty that a previous exclusive owner (TIOCEXCL) has touched
    // can hand the next opener of the same index an EBUSY that has nothing
    // to do with the code under test. Holding the slave pins the index so no
    // later pty can inherit that confusion.
    //
    // Measured: with the master closed and this descriptor still open, the
    // slave reports revents=0x19 (POLLIN|POLLERR|POLLHUP) -- the daemon sees
    // the hangup exactly as it would from a yanked cable.
    void hangup()
    {
        if (master >= 0) { ::close(master); master = -1; }
    }

    void close_pair()
    {
        if (master >= 0) { ::close(master); master = -1; }
        if (slave  >= 0) { ::close(slave);  slave  = -1; }
    }

    ~PtyPair() { close_pair(); }
};
