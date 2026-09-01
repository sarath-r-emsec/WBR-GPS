#include "wbr_gps/json_io.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace wbr_gps {

namespace {

std::string escape(const std::string& s)
{
    std::string o;
    o.reserve(s.size() + 8);
    for (char c : s) {
        unsigned char uc = (unsigned char)c;
        if (c == '"' || c == '\\')  { o += '\\'; o += c; }
        else if (uc < 0x20 || uc >= 0x7F)  { /* drop control and high bytes */ }
        else                                { o += c; }
    }
    return o;
}

// Find the value position for "key": at the top level.
size_t find_value(const std::string& js, const char* key)
{
    const std::string pat = std::string("\"") + key + "\":";
    const size_t at = js.find(pat);
    if (at == std::string::npos) return std::string::npos;
    return at + pat.size();
}

} // namespace

std::string snapshot_to_json(const Snapshot& s, int64_t now_mono_ms)
{
    // Age is 0 when we have never had a fix, rather than a huge number, so
    // clients distinguish "never" via has_fix/fix_mono_ms, not via magnitude.
    const int64_t age = (s.fix_mono_ms == 0) ? 0 : (now_mono_ms - s.fix_mono_ms);

    // Guard doubles against NaN/Inf which produce invalid JSON.
    const double hdop = std::isfinite(s.hdop) ? s.hdop : 0.0;
    const double lat = std::isfinite(s.lat) ? s.lat : 0.0;
    const double lon = std::isfinite(s.lon) ? s.lon : 0.0;
    const double alt_m = std::isfinite(s.alt_m) ? s.alt_m : 0.0;
    const double speed_kph = std::isfinite(s.speed_kph) ? s.speed_kph : 0.0;

    char buf[768];
    const int n = snprintf(buf, sizeof buf,
        "{\"class\":\"FIX\",\"seq\":%llu,"
        "\"device_present\":%s,\"serial_ok\":%s,\"hid_ok\":%s,"
        "\"gpsdo_locked\":%s,"
        "\"has_fix\":%s,\"fix_quality\":%d,\"satellites\":%d,\"hdop\":%.2f,"
        "\"lat\":%.7f,\"lon\":%.7f,\"alt_m\":%.2f,\"speed_kph\":%.3f,"
        "\"time_utc\":\"%s\",\"fix_age_ms\":%lld}",
        (unsigned long long)s.seq,
        s.device_present ? "true" : "false",
        s.serial_ok      ? "true" : "false",
        s.hid_ok         ? "true" : "false",
        s.gpsdo_locked   ? "true" : "false",
        s.has_fix        ? "true" : "false",
        s.fix_quality, s.satellites, hdop,
        lat, lon, alt_m, speed_kph,
        escape(s.time_utc).c_str(),
        (long long)age);
    if (n < 0 || n >= (int)sizeof buf) return "{\"class\":\"ERROR\",\"msg\":\"encode failed\"}";
    return std::string(buf, (size_t)n);
}

std::string nmea_to_json(const std::string& raw, int64_t t_unix_ms, uint64_t dropped)
{
    return "{\"class\":\"NMEA\",\"raw\":\"" + escape(raw) +
           "\",\"t_unix_ms\":" + std::to_string(t_unix_ms) +
           ",\"dropped\":" + std::to_string(dropped) + "}";
}

bool json_get_string(const std::string& js, const char* key, std::string& out)
{
    size_t p = find_value(js, key);
    if (p == std::string::npos) return false;
    while (p < js.size() && (js[p] == ' ')) ++p;
    if (p >= js.size() || js[p] != '"') return false;
    ++p;
    std::string v;
    while (p < js.size() && js[p] != '"') {
        if (js[p] == '\\' && p + 1 < js.size()) ++p;
        v += js[p++];
    }
    if (p >= js.size()) return false;   // unterminated string
    out = v;
    return true;
}

bool json_get_bool(const std::string& js, const char* key, bool& out)
{
    size_t p = find_value(js, key);
    if (p == std::string::npos) return false;
    while (p < js.size() && js[p] == ' ') ++p;
    if (js.compare(p, 4, "true") == 0)  { out = true;  return true; }
    if (js.compare(p, 5, "false") == 0) { out = false; return true; }
    return false;
}

bool json_get_double(const std::string& js, const char* key, double& out)
{
    const size_t p = find_value(js, key);
    if (p == std::string::npos) return false;
    char* end = nullptr;
    const double v = std::strtod(js.c_str() + p, &end);
    if (end == js.c_str() + p) return false;
    out = v;
    return true;
}

bool json_get_int64(const std::string& js, const char* key, int64_t& out)
{
    const size_t p = find_value(js, key);
    if (p == std::string::npos) return false;
    char* end = nullptr;
    const long long v = std::strtoll(js.c_str() + p, &end, 10);
    if (end == js.c_str() + p) return false;
    out = (int64_t)v;
    return true;
}

bool snapshot_from_json(const std::string& line, Snapshot& out)
{
    std::string cls;
    if (!json_get_string(line, "class", cls) || cls != "FIX") return false;

    Snapshot s;
    int64_t i = 0;
    double  d = 0.0;

    if (json_get_int64(line, "seq", i)) s.seq = (uint64_t)i;
    json_get_bool(line, "device_present", s.device_present);
    json_get_bool(line, "serial_ok",      s.serial_ok);
    json_get_bool(line, "hid_ok",         s.hid_ok);
    json_get_bool(line, "gpsdo_locked",   s.gpsdo_locked);
    json_get_bool(line, "has_fix",        s.has_fix);
    if (json_get_int64(line, "fix_quality", i)) s.fix_quality = (int)i;
    if (json_get_int64(line, "satellites",  i)) s.satellites  = (int)i;
    if (json_get_double(line, "hdop",      d)) s.hdop      = d;
    if (json_get_double(line, "lat",       d)) s.lat       = d;
    if (json_get_double(line, "lon",       d)) s.lon       = d;
    if (json_get_double(line, "alt_m",     d)) s.alt_m     = d;
    if (json_get_double(line, "speed_kph", d)) s.speed_kph = d;
    json_get_string(line, "time_utc", s.time_utc);
    if (json_get_int64(line, "fix_age_ms", i)) s.fix_age_ms = i;

    out = s;
    return true;
}

} // namespace wbr_gps
