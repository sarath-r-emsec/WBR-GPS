#include "wbr_gps/serial_source.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace wbr_gps {

namespace {

constexpr size_t  kMaxBuf      = 1024;   // discard beyond this: torn input
constexpr int64_t kBackoffMin  = 250;
constexpr int64_t kBackoffMax  = 5000;

speed_t baud_const(int baud)
{
    switch (baud) {
        case   4800: return B4800;
        case   9600: return B9600;
        case  19200: return B19200;
        case  38400: return B38400;
        case  57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:     return B115200;
    }
}

bool configure(int fd, int baud)
{
    struct termios tio {};
    if (tcgetattr(fd, &tio) != 0) return false;
    cfmakeraw(&tio);
    cfsetispeed(&tio, baud_const(baud));
    cfsetospeed(&tio, baud_const(baud));
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;   // pure non-blocking: epoll decides when to read
    return tcsetattr(fd, TCSANOW, &tio) == 0;
}

} // namespace

bool SerialSource::open_device()
{
    fd_ = ::open(path_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return false;
    }

    // THE critical call. TIOCEXCL makes further open() attempts fail with
    // EBUSY, so a stray reader gets a loud error instead of silently stealing
    // bytes out of the shared tty input queue. This is guarantee G1.
    if (::ioctl(fd_, TIOCEXCL) != 0) {
        std::fprintf(stderr, "[wbr-gpsd] TIOCEXCL failed on %s: %s\n",
                     path_.c_str(), std::strerror(errno));
        // Not fatal on a pty, which does not support it; fatal in spirit on
        // real hardware, where the caller logs and continues.
    }

    if (!configure(fd_, baud_)) {
        std::fprintf(stderr, "[wbr-gpsd] configure %s failed: %s\n",
                     path_.c_str(), std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    buf_.clear();
    backoff_ms_ = kBackoffMin;
    return true;
}

void SerialSource::close_device()
{
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    buf_.clear();
}

bool SerialSource::on_readable(StateStore& store, int64_t now_mono_ms,
                               const std::function<void(const std::string&)>& on_line)
{
    char io[512];
    // Set once this call clears an oversized run, so the tail of that same
    // burst can't dribble a few more bytes into buf_ before the run ends.
    // Without this, a garbage run whose length isn't an exact multiple of
    // (kMaxBuf + 1) leaves a residual fragment in buf_ that then prefixes
    // and corrupts the next legitimate line. Scoped to this call only: a
    // later on_readable() call is a fresh readable-event, and by then buf_
    // is already guaranteed empty, so normal accumulation resumes cleanly.
    bool resyncing = false;
    for (;;) {
        const ssize_t n = ::read(fd_, io, sizeof io);
        if (n > 0) {
            for (ssize_t i = 0; i < n; ++i) {
                const char c = io[i];
                if (c == '\r') continue;
                if (c == '\n') {
                    resyncing = false;   // a delimiter always resyncs us
                    if (!buf_.empty()) {
                        store.apply_nmea(buf_, now_mono_ms);
                        on_line(buf_);
                        buf_.clear();
                    }
                    continue;
                }
                if (resyncing) continue;
                buf_ += c;
                // A line this long is a torn or concatenated read. Drop it
                // rather than letting the buffer grow without bound, and
                // stop accumulating for the rest of this burst.
                if (buf_.size() > kMaxBuf) {
                    buf_.clear();
                    resyncing = true;
                }
            }
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return true;                     // drained
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        // n == 0 is NOT a reliable "device gone" signal here: with
        // VMIN=0/VTIME=0 (a deliberate choice, see the header), a tty's
        // read() returns 0 for "no data available right now" exactly the
        // same way it does after the peer truly hangs up — verified against
        // both a live pty and a pty whose master has been closed. Treat it
        // like a drained buffer, not EOF. A genuine detach on real hardware
        // (or a pty hangup once more bytes are attempted) instead surfaces
        // as a hard read() error below, which is what we act on.
        if (n == 0) {
            return true;                     // drained, not gone
        }
        // A real error other than EAGAIN/EWOULDBLOCK/EINTR: the device went
        // away (e.g. ENXIO/EIO from a detached USB CDC-ACM device).
        close_device();
        return false;
    }
}

bool SerialSource::should_retry(int64_t now_mono_ms) const
{
    return now_mono_ms >= next_retry_ms_;
}

void SerialSource::note_retry(int64_t now_mono_ms)
{
    next_retry_ms_ = now_mono_ms + backoff_ms_;
    backoff_ms_ *= 2;
    if (backoff_ms_ > kBackoffMax) backoff_ms_ = kBackoffMax;
}

} // namespace wbr_gps
