#include "wbr_gps/nmea_parser.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>

namespace wbr_gps {

namespace {
// The longest legal NMEA sentence is 82 bytes. Anything materially longer is
// a torn or concatenated read, not a sentence.
constexpr size_t kMaxLine = 128;
} // namespace

bool nmea_checksum_ok(const std::string& line)
{
    if (line.size() < 7 || line.size() > kMaxLine) return false;
    if (line[0] != '$') return false;

    const size_t star = line.find('*');
    if (star == std::string::npos || star + 3 > line.size()) return false;

    unsigned cs = 0;
    for (size_t i = 1; i < star; ++i) {
        const unsigned char c = (unsigned char)line[i];
        // A NUL or any control/high byte inside a sentence means a torn read.
        if (c < 0x20 || c > 0x7E) return false;
        cs ^= c;
    }

    const std::string hex = line.substr(star + 1, 2);
    for (char c : hex) {
        if (!std::isxdigit((unsigned char)c)) return false;
    }
    return cs == (unsigned)std::strtoul(hex.c_str(), nullptr, 16);
}

bool nmea_coord(const std::string& field, char hemi, double& out)
{
    // DDMM.MMMM needs at least 4 characters. Reject short fields instead of
    // letting strtod return 0 and pass as a valid fix on the equator.
    if (field.size() < 4) return false;
    if (hemi != 'N' && hemi != 'S' && hemi != 'E' && hemi != 'W') return false;

    char* end = nullptr;
    const double raw = std::strtod(field.c_str(), &end);
    if (end == field.c_str()) return false;
    if (!std::isfinite(raw)) return false;

    const double deg = std::floor(raw / 100.0);
    const double min = raw - deg * 100.0;
    if (min >= 60.0) return false;

    double v = deg + min / 60.0;
    if (hemi == 'S' || hemi == 'W') v = -v;
    if (std::fabs(v) > 180.0) return false;

    out = v;
    return true;
}

std::vector<std::string> nmea_split(const std::string& line)
{
    std::vector<std::string> out;
    std::string cur;
    const size_t star = line.find('*');
    const size_t end  = (star == std::string::npos) ? line.size() : star;
    for (size_t i = 0; i < end; ++i) {
        if (line[i] == ',') {
            out.push_back(cur);
            cur.clear();
        } else {
            cur += line[i];
        }
    }
    out.push_back(cur);
    return out;
}

ParseResult nmea_apply(const std::string& line, Snapshot& snap, int64_t now_mono_ms)
{
    if (!nmea_checksum_ok(line)) return ParseResult::Rejected;

    // line[0]=='$', [1..2]=talker ("GP"/"GN"), [3..5]=sentence type.
    const std::string tag = line.substr(3, 3);
    const std::vector<std::string> f = nmea_split(line);

    if (tag == "GGA") {
        // 1=time 2=lat 3=N/S 4=lon 5=E/W 6=quality 7=sats 8=hdop 9=alt
        if (f.size() < 10) return ParseResult::Rejected;

        snap.time_utc    = f[1];
        snap.fix_quality = std::atoi(f[6].c_str());
        snap.has_fix     = snap.fix_quality > 0;
        snap.satellites  = std::atoi(f[7].c_str());
        const double hdop = std::strtod(f[8].c_str(), nullptr);
        snap.hdop        = std::isfinite(hdop) ? hdop : 0.0;

        double lat = 0.0, lon = 0.0;
        if (snap.has_fix && !f[3].empty() && !f[5].empty() &&
            nmea_coord(f[2], f[3][0], lat) &&
            nmea_coord(f[4], f[5][0], lon)) {
            snap.lat         = lat;
            snap.lon         = lon;
            const double alt_m = std::strtod(f[9].c_str(), nullptr);
            snap.alt_m       = std::isfinite(alt_m) ? alt_m : 0.0;
            snap.fix_mono_ms = now_mono_ms;
        }
        // With no fix we keep the last known position but do NOT advance
        // fix_mono_ms, so age reporting correctly shows the data as stale.
        return ParseResult::UpdatedFix;
    }

    if (tag == "RMC") {
        // 1=time 2=status 3=lat 4=N/S 5=lon 6=E/W 7=speed(knots) 8=track 9=date
        if (f.size() < 8) return ParseResult::Rejected;
        if (f[2] == "A") {
            const double speed_knots = std::strtod(f[7].c_str(), nullptr);
            const double speed = speed_knots * 1.852;
            snap.speed_kph = std::isfinite(speed) ? speed : 0.0;
        } else {
            snap.speed_kph = 0.0;
        }
        return ParseResult::UpdatedSpeed;
    }

    return ParseResult::Ignored;
}

} // namespace wbr_gps
