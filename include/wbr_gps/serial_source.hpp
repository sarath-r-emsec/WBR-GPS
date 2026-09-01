#pragma once

#include "wbr_gps/state_store.hpp"

#include <cstdint>
#include <functional>
#include <string>

namespace wbr_gps {

// Owns the GPS serial port. Exclusively: the fd is opened with TIOCEXCL so a
// stray reader gets EBUSY rather than silently stealing bytes from our queue.
class SerialSource {
public:
    void        set_path(const std::string& p) { path_ = p; }
    std::string path() const { return path_; }
    void        set_baud(int b) { baud_ = b; }

    // Open and configure. Returns false on any failure; the caller then uses
    // should_retry()/note_retry() to back off.
    bool open_device();
    void close_device();
    int  fd() const { return fd_; }

    // Read whatever is available, assemble lines, apply them to `store`, and
    // invoke `on_line` for each complete line (for raw NMEA subscribers).
    // Returns false if the device went away and was closed.
    bool on_readable(StateStore& store, int64_t now_mono_ms,
                     const std::function<void(const std::string&)>& on_line);

    bool should_retry(int64_t now_mono_ms) const;
    void note_retry(int64_t now_mono_ms);

private:
    std::string path_ = kDefaultSerialPath;
    int         baud_ = kDefaultBaud;
    int         fd_   = -1;
    std::string buf_;

    int64_t next_retry_ms_ = 0;
    int64_t backoff_ms_    = 250;   // 250 ms, doubling, capped at 5 s
};

} // namespace wbr_gps
