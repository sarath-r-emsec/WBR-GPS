#pragma once

#include "wbr_gps/gps_types.hpp"
#include "wbr_gps/nmea_parser.hpp"

#include <cstdint>
#include <string>

namespace wbr_gps {

// The one authoritative snapshot. No locking: the daemon is single-threaded
// on an epoll loop, which is precisely why it needs none.
class StateStore {
public:
    ParseResult apply_nmea(const std::string& line, int64_t now_mono_ms);

    void set_serial_ok(bool v);
    void set_hid_ok(bool v);
    void set_device_present(bool v);
    void set_gpsdo_locked(bool v);

    // Drop has_fix when the last fix is older than max_age_ms. The position
    // is retained so consumers can still report a last-known location.
    void mark_fix_stale_if_older_than(int64_t now_mono_ms, int64_t max_age_ms);

    const Snapshot& current() const { return snap_; }
    uint64_t        seq() const { return snap_.seq; }
    uint64_t        rejected_count() const { return rejected_; }

private:
    void bump() { ++snap_.seq; }

    Snapshot snap_;
    uint64_t rejected_ = 0;
};

} // namespace wbr_gps
