#include "wbr_gps/hid_source.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

namespace wbr_gps {

namespace {
constexpr int64_t kBackoffMin = 250;
constexpr int64_t kBackoffMax = 5000;
constexpr const char* kHidrawClassDir = "/sys/class/hidraw";
constexpr const char* kHidIdPrefix = "HID_ID=";
constexpr const char* kLeoVendorSeg = ":00001DD2:";
// Byte 0 of the LBE-1421 status report while the unit is locked.
// LED-confirmed; see hid_decode_lock() for the full evidence.
constexpr unsigned char kLockedStatus = 0x7f;
} // namespace

bool hid_decode_lock(const unsigned char* rep, ssize_t n)
{
    if (n < 2) return false;
    // The whole of byte 0 is the lock indicator: 0x7f means locked.
    //
    // This is the third decode tried here, and the first with ground truth on
    // BOTH sides of the transition. The LBE-1421 sends a 64-byte report about
    // 8 times a second, and across every observation only byte 0 ever varies
    // -- bytes 1..63 are constant (0x00 then 0xff). Byte 0 takes exactly three
    // values, and which value appears is decided by the front-panel LED:
    //
    //     LED SOLID  (locked)    0x7f, and only 0x7f     4 sessions, incl. 20h continuous
    //     LED BLINK  (unlocked)  0x6e or 0x76, never 0x7f    157 samples
    //
    // Both states were confirmed by a human watching the LED, not inferred.
    // The unplug/replug transition was captured live in both directions with
    // no intermediate values (0x77..0x7e never appear), which rules out byte 0
    // being a quality or DAC level climbing toward 127.
    //
    // Why compare the whole byte instead of testing bit 0, which also fits?
    // Because the two hypotheses are indistinguishable from this data -- 0x7f
    // has every low bit set -- and they fail in opposite directions. If byte 0
    // is really a status bitmask whose bits can be set independently, then
    // `rep[0] & 0x01` would call 0x7d "locked" while some other status bit is
    // clear. `rep[0] == 0x7f` cannot do that: anything it has not actually
    // observed while locked reads as unlocked. A consumer steering an SDR onto
    // the GPSDO's 10 MHz reference should get a false negative, never a false
    // positive.
    //
    // The two earlier decodes, both wrong, both for the same reason -- a
    // format concluded from samples taken in only ONE device state:
    //
    //   (rep[1] & 0x01) == 0   "bit 0 clear means locked". Byte 1 is 0x00 in
    //                          every report ever captured, locked or not, so
    //                          this was a constant TRUE. It is why every UI
    //                          kept reporting "Locked" through a replug while
    //                          the unit was visibly still acquiring.
    //
    //   rep[0] == 0x7f as a    right predicate, wrong reason. 0x7f was called
    //   "report marker"        a report-ID and the check was placed before the
    //                          byte-1 test; when byte 0 turned out to vary it
    //                          was withdrawn as broken. It was not broken --
    //                          it was reading unlocked during an unlocked
    //                          period. Reinstated here on evidence.
    //
    // If a future unit reports a different locked value, this returns false
    // (safe) rather than mis-reporting, and the fix is to widen this set with
    // a new LED-confirmed observation -- not to loosen it to a bit test.
    return rep[0] == kLockedStatus;
}

std::string HidSource::discover()
{
    DIR* d = ::opendir(kHidrawClassDir);
    if (!d) return "";

    std::string result;
    struct dirent* ent;
    while ((ent = ::readdir(d)) != nullptr) {
        const std::string name = ent->d_name;
        if (name.rfind("hidraw", 0) != 0) continue;

        const std::string uevent =
            std::string(kHidrawClassDir) + "/" + name + "/device/uevent";
        FILE* f = std::fopen(uevent.c_str(), "r");
        if (!f) continue;

        char line[256];
        bool match = false;
        while (std::fgets(line, sizeof line, f)) {
            // HID_ID=0003:00001DD2:00002444
            // Anchor on HID_ID= and check for the vendor segment with colons.
            // This prevents false positives from other fields containing these digits.
            if (std::strstr(line, kHidIdPrefix) && std::strstr(line, kLeoVendorSeg)) {
                match = true;
                break;
            }
        }
        std::fclose(f);

        if (match) { result = "/dev/" + name; break; }
    }
    ::closedir(d);
    return result;
}

bool HidSource::open_device()
{
    if (path_.empty()) path_ = discover();
    if (path_.empty()) return false;

    fd_ = ::open(path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) return false;
    backoff_ms_ = kBackoffMin;
    return true;
}

void HidSource::close_device()
{
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool HidSource::on_readable(StateStore& store)
{
    unsigned char rep[8] = { 0 };
    for (;;) {
        const ssize_t n = ::read(fd_, rep, sizeof rep);
        if (n >= 2) {
            store.set_gpsdo_locked(hid_decode_lock(rep, n));
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return true;
        if (n < 0 && errno == EINTR) continue;
        if (n >= 0 && n < 2) return true;   // short report: ignore, keep fd
        close_device();
        return false;
    }
}

bool HidSource::should_retry(int64_t now_mono_ms) const
{
    return now_mono_ms >= next_retry_ms_;
}

void HidSource::note_retry(int64_t now_mono_ms)
{
    next_retry_ms_ = now_mono_ms + backoff_ms_;
    backoff_ms_ *= 2;
    if (backoff_ms_ > kBackoffMax) backoff_ms_ = kBackoffMax;
}

} // namespace wbr_gps
