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
} // namespace

bool hid_decode_lock(const unsigned char* rep, ssize_t n)
{
    if (n < 2) return false;
    // Bit 0 clear means locked, matching the existing WBR-SA reader.
    return (rep[1] & 0x01) == 0;
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
