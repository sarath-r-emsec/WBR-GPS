#include "wbr_gps/state_store.hpp"
#include "test_util.h"

#include <cstdio>
#include <string>

static const char* kGGA =
    "$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F";
static const char* kTorn =
    ",,3,,5$G32,,,9151,,2,96*G3,,1,,,8,,6379$M055A3.5N7.500,*";

static void test_seq_advances_only_on_change()
{
    wbr_gps::StateStore st;
    ASSERT_EQ(st.seq(), (uint64_t)0);

    st.apply_nmea(kGGA, 1000);
    const uint64_t after_fix = st.seq();
    ASSERT_TRUE(after_fix > 0);

    // A rejected line must not advance seq and must be counted.
    st.apply_nmea(kTorn, 1100);
    ASSERT_EQ(st.seq(), after_fix);
    ASSERT_EQ(st.rejected_count(), (uint64_t)1);

    // An ignored sentence must not advance seq either.
    st.apply_nmea("$GNGSA,A,3,32,31,25,18,,,,,,,,,2.39,1.33,1.99*19", 1200);
    ASSERT_EQ(st.seq(), after_fix);
}

static void test_health_flags_independent()
{
    wbr_gps::StateStore st;
    st.apply_nmea(kGGA, 1000);
    ASSERT_TRUE(st.current().has_fix);

    // Losing the serial port must NOT clear has_fix or the position. The two
    // are different facts; conflating them is the original bug.
    st.set_serial_ok(false);
    ASSERT_FALSE(st.current().serial_ok);
    ASSERT_TRUE(st.current().has_fix);
    ASSERT_NEAR(st.current().lat, 13.0028260, 1e-6);

    // Losing hidraw must not affect the NMEA side at all.
    st.set_hid_ok(false);
    st.set_gpsdo_locked(false);
    ASSERT_FALSE(st.current().hid_ok);
    ASSERT_TRUE(st.current().has_fix);

    // Device presence is independent of both.
    st.set_device_present(true);
    ASSERT_TRUE(st.current().device_present);
    ASSERT_FALSE(st.current().serial_ok);
}

static void test_health_change_advances_seq()
{
    wbr_gps::StateStore st;
    const uint64_t s0 = st.seq();

    // Test set_serial_ok: change advances seq, repeat is no-op
    st.set_serial_ok(true);
    ASSERT_TRUE(st.seq() > s0);
    const uint64_t s1 = st.seq();
    st.set_serial_ok(true);
    ASSERT_EQ(st.seq(), s1);

    // Test set_hid_ok: change advances seq, repeat is no-op
    st.set_hid_ok(true);
    ASSERT_TRUE(st.seq() > s1);
    const uint64_t s2 = st.seq();
    st.set_hid_ok(true);
    ASSERT_EQ(st.seq(), s2);

    // Test set_device_present: change advances seq, repeat is no-op
    st.set_device_present(true);
    ASSERT_TRUE(st.seq() > s2);
    const uint64_t s3 = st.seq();
    st.set_device_present(true);
    ASSERT_EQ(st.seq(), s3);

    // Test set_gpsdo_locked: change advances seq, repeat is no-op
    st.set_gpsdo_locked(true);
    ASSERT_TRUE(st.seq() > s3);
    const uint64_t s4 = st.seq();
    st.set_gpsdo_locked(true);
    ASSERT_EQ(st.seq(), s4);
}

static void test_staleness()
{
    wbr_gps::StateStore st;
    st.apply_nmea(kGGA, 1000);
    ASSERT_TRUE(st.current().has_fix);

    // Not yet stale at 5 s with a 10 s limit.
    st.mark_fix_stale_if_older_than(6000, 10000);
    ASSERT_TRUE(st.current().has_fix);

    // Stale at 12 s. has_fix drops, but the last position is retained so a
    // consumer can still report "last known".
    st.mark_fix_stale_if_older_than(13000, 10000);
    ASSERT_FALSE(st.current().has_fix);
    ASSERT_NEAR(st.current().lat, 13.0028260, 1e-6);

    // Idempotent: repeating must not advance seq again.
    const uint64_t s = st.seq();
    st.mark_fix_stale_if_older_than(14000, 10000);
    ASSERT_EQ(st.seq(), s);
}

static void test_bad_coords_rejected_at_parser()
{
    // Regression for T4-A: GGA claiming fix_quality=1 but with undecodable
    // coordinates (e.g., minutes >= 60) must not set has_fix. Layer 1 fix:
    // nmea_parser ensures has_fix = true only when both coords decode.
    wbr_gps::StateStore st;

    // Build a checksum-valid GGA with fix_quality=1 but bad latitude (60+ minutes)
    unsigned cs = 0;
    const std::string body = "$GNGGA,045519.50,1360.00000,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,";
    for (size_t i = 1; i < body.size(); ++i) cs ^= (unsigned char)body[i];
    char buf[8];
    snprintf(buf, sizeof buf, "*%02X", cs);
    const std::string bad_gga = body + buf;

    st.apply_nmea(bad_gga, 1000);
    // Quality field claims a fix, but coords are garbage.
    ASSERT_EQ(st.current().fix_quality, 1);
    // has_fix must be false: we do not hold a trustworthy position.
    ASSERT_FALSE(st.current().has_fix);
}

static void run_tests()
{
    test_seq_advances_only_on_change();
    test_health_flags_independent();
    test_health_change_advances_seq();
    test_staleness();
    test_bad_coords_rejected_at_parser();
}

TEST_MAIN
