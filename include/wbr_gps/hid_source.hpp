#pragma once

#include "wbr_gps/state_store.hpp"

#include <cstdint>
#include <string>

namespace wbr_gps {

// Owns the Leo Bodnar hidraw device. The 2-byte status report's byte 1 bit 0
// is the GPSDO lock bit (0 means locked). This is a genuinely different
// signal from NMEA fix: the clock can be locked with no current GGA.
class HidSource {
public:
    void        set_path(const std::string& p) { path_ = p; }
    std::string path() const { return path_; }

    // Locate a Leo Bodnar hidraw node by walking /sys/class/hidraw/*/device/uevent
    // for vendor 1DD2. Returns "" if none. Avoids shelling out to udevadm, which
    // is what makes the current GUI probe slow enough to need a cache.
    static std::string discover();

    bool open_device();
    void close_device();
    int  fd() const { return fd_; }

    // Read one report and update the lock bit. Returns false if the device
    // went away and was closed.
    bool on_readable(StateStore& store);

    bool should_retry(int64_t now_mono_ms) const;
    void note_retry(int64_t now_mono_ms);

private:
    std::string path_;
    int         fd_ = -1;
    int64_t     next_retry_ms_ = 0;
    int64_t     backoff_ms_    = 250;
};

} // namespace wbr_gps
