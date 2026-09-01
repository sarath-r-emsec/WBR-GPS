#include "wbr_gps/gps_types.hpp"
#include "test_util.h"

#include <unistd.h>

static void run_tests()
{
    // A default Snapshot must mean "nothing known", not "everything fine".
    wbr_gps::Snapshot s;
    ASSERT_FALSE(s.device_present);
    ASSERT_FALSE(s.serial_ok);
    ASSERT_FALSE(s.hid_ok);
    ASSERT_FALSE(s.service_ok);
    ASSERT_FALSE(s.gpsdo_locked);
    ASSERT_FALSE(s.has_fix);
    ASSERT_EQ(s.seq, (uint64_t)0);
    ASSERT_EQ(s.fix_mono_ms, (int64_t)0);

    // now_mono_ms must advance and must never be zero or negative.
    const int64_t t0 = wbr_gps::now_mono_ms();
    ASSERT_TRUE(t0 > 0);
    usleep(20000);
    const int64_t t1 = wbr_gps::now_mono_ms();
    ASSERT_TRUE(t1 > t0);
    ASSERT_TRUE(t1 - t0 >= 15);
}

TEST_MAIN
