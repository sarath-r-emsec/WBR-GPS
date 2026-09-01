#pragma once

#include "wbr_gps/gps_types.hpp"

#include <string>
#include <vector>

namespace wbr_gps {

enum class ParseResult {
    Rejected,      // malformed: no '$', too short/long, bad or missing checksum
    Ignored,       // well-formed, but not a sentence type we consume
    UpdatedFix,    // GGA applied
    UpdatedSpeed,  // RMC applied
};

// XOR of every byte between '$' and '*' must equal the two hex digits after
// '*'. A line with no '*' fails: an unchecksummed sentence is exactly what a
// torn read looks like, so we refuse to trust it.
bool nmea_checksum_ok(const std::string& line);

// Decode DDMM.MMMM / DDDMM.MMMM plus a hemisphere character into signed
// decimal degrees. Returns false if the field is empty or malformed, rather
// than yielding 0 and passing as a valid fix on the equator.
bool nmea_coord(const std::string& field, char hemi, double& out);

// Split on ',' after truncating at '*'.
std::vector<std::string> nmea_split(const std::string& line);

// Apply one sentence to `snap`. `snap` is modified only when accepted.
ParseResult nmea_apply(const std::string& line, Snapshot& snap, int64_t now_mono_ms);

} // namespace wbr_gps
