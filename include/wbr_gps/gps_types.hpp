#pragma once

#include <cstdint>
#include <string>

namespace wbr_gps {

inline constexpr int         kProtoVersion      = 1;
inline constexpr const char* kDefaultSocketPath = "/run/wbr-gps/gpsd.sock";
inline constexpr const char* kDefaultSerialPath = "/dev/gpsdo";
inline constexpr int         kDefaultBaud       = 115200;

// Milliseconds on CLOCK_MONOTONIC. Never wall time: the GPSDO can step the
// system clock, and a negative fix age is a classic bug in GPS daemons.
int64_t now_mono_ms();

// Resolves the socket path to use when the caller does not pass one
// explicitly: $WBR_GPS_SOCKET_DIR/gpsd.sock if that env var is set, else the
// compiled-in kDefaultSocketPath (/run/wbr-gps/gpsd.sock). The env var is how
// an unprivileged/prefix install with no /etc/tmpfiles.d entry (e.g. a
// $XDG_RUNTIME_DIR/wbr-gps directory, no root needed) moves the daemon AND
// every client to the same non-default location by setting one variable
// once, instead of a --socket flag that has to be repeated identically on
// every consumer. See CMakeLists.txt's WBR_GPS_SYSTEM_SERVICE option and
// Server::listen_on()'s ensure_parent_dir() (server.cpp), which creates that
// directory on first bind so this path needs no separate install step at
// all.
std::string default_socket_path();

// Same env-var-first resolution as default_socket_path(), for the daemon's
// own pid lockfile (main.cpp's Options::lock_path default) -- so
// WBR_GPS_SOCKET_DIR moves both files together, not just the socket.
std::string default_lock_path();

// One complete view of GPS state. Every health field is independent on
// purpose: "I could not read the device" must never be confusable with "the
// device is not locked". That conflation is the original bug.
struct Snapshot {
    uint64_t seq = 0;

    // --- health, four independent signals ---
    bool device_present = false;  // a Leo Bodnar is attached to USB at all
    bool serial_ok      = false;  // the daemon currently holds the serial port
    bool hid_ok         = false;  // the daemon currently holds the hidraw device
    bool service_ok     = false;  // client-side only: a live daemon connection

    // --- GPSDO lock, distinct from NMEA fix ---
    bool gpsdo_locked = false;

    // --- NMEA fix ---
    bool        has_fix     = false;
    int         fix_quality = 0;     // GGA field 6: 0 none, 1 GPS, 2 DGPS
    int         satellites  = 0;
    double      hdop        = 0.0;
    double      lat         = 0.0;   // signed degrees, north positive
    double      lon         = 0.0;   // signed degrees, east positive
    double      alt_m       = 0.0;
    double      speed_kph   = 0.0;
    std::string time_utc;            // "HHMMSS.SS" as sent

    // CLOCK_MONOTONIC ms when lat/lon were last set. 0 means never.
    int64_t fix_mono_ms = 0;

    // Age of the fix in ms, computed when the snapshot is serialized.
    int64_t fix_age_ms = 0;
};

} // namespace wbr_gps
