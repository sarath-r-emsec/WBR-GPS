#pragma once

#include "wbr_gps/gps_types.hpp"

#include <cstdint>
#include <string>

namespace wbr_gps {

// Emit one FIX message as a single line, with no trailing newline. fix_age_ms
// is computed here from now_mono_ms so that every consumer sees a consistent
// age regardless of when it reads the line.
std::string snapshot_to_json(const Snapshot& s, int64_t now_mono_ms);

// Parse a FIX message. Returns false unless "class" is exactly "FIX".
bool snapshot_from_json(const std::string& line, Snapshot& out);

// Emit one NMEA passthrough message as a single line.
std::string nmea_to_json(const std::string& raw, int64_t t_unix_ms, uint64_t dropped);

// Minimal scanners for the small fixed message set. They look for the literal
// "<key>": at the top level; the protocol has no nested objects, so this is
// sufficient and avoids a JSON dependency in a header two other projects include.
bool json_get_string(const std::string& js, const char* key, std::string& out);
bool json_get_bool(const std::string& js, const char* key, bool& out);
bool json_get_double(const std::string& js, const char* key, double& out);
bool json_get_int64(const std::string& js, const char* key, int64_t& out);

} // namespace wbr_gps
