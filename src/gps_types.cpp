#include "wbr_gps/gps_types.hpp"

#include <ctime>

namespace wbr_gps {

int64_t now_mono_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

} // namespace wbr_gps
