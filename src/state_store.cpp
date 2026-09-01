#include "wbr_gps/state_store.hpp"

namespace wbr_gps {

ParseResult StateStore::apply_nmea(const std::string& line, int64_t now_mono_ms)
{
    const ParseResult r = nmea_apply(line, snap_, now_mono_ms);
    if (r == ParseResult::Rejected) {
        ++rejected_;
    } else if (r != ParseResult::Ignored) {
        bump();
    }
    return r;
}

// Each setter is a no-op when the value is unchanged, so clients are never
// woken by non-events.
void StateStore::set_serial_ok(bool v)
{
    if (snap_.serial_ok == v) return;
    snap_.serial_ok = v;
    bump();
}

void StateStore::set_hid_ok(bool v)
{
    if (snap_.hid_ok == v) return;
    snap_.hid_ok = v;
    bump();
}

void StateStore::set_device_present(bool v)
{
    if (snap_.device_present == v) return;
    snap_.device_present = v;
    bump();
}

void StateStore::set_gpsdo_locked(bool v)
{
    if (snap_.gpsdo_locked == v) return;
    snap_.gpsdo_locked = v;
    bump();
}

void StateStore::mark_fix_stale_if_older_than(int64_t now_mono_ms, int64_t max_age_ms)
{
    if (!snap_.has_fix) return;                 // already stale: idempotent
    if (now_mono_ms - snap_.fix_mono_ms <= max_age_ms) return;
    snap_.has_fix = false;
    bump();
}

} // namespace wbr_gps
