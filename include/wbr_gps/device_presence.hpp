#pragma once

#include <string>

namespace wbr_gps {

// Case-insensitive scan of `dir` for an entry containing `needle`. Returns a
// full path, or "" if nothing matched or the directory is unreadable.
std::string scan_by_id_for(const char* dir, const char* needle);

// Full path to the Leo Bodnar serial device via /dev/serial/by-id, or "".
std::string find_leo_bodnar_serial();

// Is a Leo Bodnar attached to USB at all? This is the ref-present signal that
// replaces leo_bodnar_usb_present() in WBR-GSM/gps_source.h, used by
// gsm_monitor.cpp:772 to auto-select the SDR's 10 MHz external reference.
bool leo_bodnar_present();

} // namespace wbr_gps
