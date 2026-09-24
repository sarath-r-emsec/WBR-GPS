#include "wbr_gps/gps_types.hpp"

#include <cstdlib>
#include <ctime>

namespace wbr_gps {

int64_t now_mono_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

std::string default_socket_path()
{
    if (const char* dir = std::getenv("WBR_GPS_SOCKET_DIR")) {
        return std::string(dir) + "/gpsd.sock";
    }
    return kDefaultSocketPath;
}

std::string default_lock_path()
{
    if (const char* dir = std::getenv("WBR_GPS_SOCKET_DIR")) {
        return std::string(dir) + "/wbr-gpsd.pid";
    }
    return "/run/wbr-gps/wbr-gpsd.pid";
}

} // namespace wbr_gps
