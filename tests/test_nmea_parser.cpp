#include "wbr_gps/nmea_parser.hpp"
#include "test_util.h"

#include <cstdio>
#include <string>

using wbr_gps::ParseResult;
using wbr_gps::Snapshot;

static const char* kGGA =
    "$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F";
static const char* kRMC =
    "$GNRMC,045519.50,A,1300.16956,N,07740.79521,E,0.000,,010926,,,A*68";
// Verbatim from the three-reader contention reproduction.
static const char* kTorn =
    ",,3,,5$G32,,,9151,,2,96*G3,,1,,,8,,6379$M055A3.5N7.500,*";

// Append a correct checksum, so synthetic test lines cannot rot.
static std::string with_checksum(const std::string& body)
{
    unsigned cs = 0;
    for (size_t i = 1; i < body.size(); ++i) cs ^= (unsigned char)body[i];
    char buf[8];
    snprintf(buf, sizeof buf, "*%02X", cs);
    return body + buf;
}

static void test_checksum()
{
    ASSERT_TRUE(wbr_gps::nmea_checksum_ok(kGGA));
    ASSERT_TRUE(wbr_gps::nmea_checksum_ok(kRMC));
    // The torn line must be rejected. This is the regression test for the bug.
    ASSERT_FALSE(wbr_gps::nmea_checksum_ok(kTorn));
    ASSERT_FALSE(wbr_gps::nmea_checksum_ok(
        "$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*00"));
    ASSERT_FALSE(wbr_gps::nmea_checksum_ok(
        "$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,"));
    ASSERT_FALSE(wbr_gps::nmea_checksum_ok("$GN"));
    ASSERT_FALSE(wbr_gps::nmea_checksum_ok("$"));
    ASSERT_FALSE(wbr_gps::nmea_checksum_ok(""));
    // Non-hex checksum digits.
    ASSERT_FALSE(wbr_gps::nmea_checksum_ok("$GNGGA,1,2,3*ZZ"));
}

static void test_coord()
{
    double v = 0.0;
    ASSERT_TRUE(wbr_gps::nmea_coord("1300.16956", 'N', v));
    ASSERT_NEAR(v, 13.0028260, 1e-6);
    ASSERT_TRUE(wbr_gps::nmea_coord("07740.79521", 'E', v));
    ASSERT_NEAR(v, 77.6799202, 1e-6);
    // Southern and western hemispheres must come back negative.
    ASSERT_TRUE(wbr_gps::nmea_coord("1300.16956", 'S', v));
    ASSERT_NEAR(v, -13.0028260, 1e-6);
    ASSERT_TRUE(wbr_gps::nmea_coord("07740.79521", 'W', v));
    ASSERT_NEAR(v, -77.6799202, 1e-6);
    // Empty, short and bad-hemisphere fields must fail, not return 0.
    ASSERT_FALSE(wbr_gps::nmea_coord("", 'N', v));
    ASSERT_FALSE(wbr_gps::nmea_coord("12", 'N', v));
    ASSERT_FALSE(wbr_gps::nmea_coord("1300.16956", 'X', v));
    // Minutes >= 60 are impossible.
    ASSERT_FALSE(wbr_gps::nmea_coord("1360.00000", 'N', v));
}

static void test_gga()
{
    Snapshot s;
    ASSERT_TRUE(wbr_gps::nmea_apply(kGGA, s, 1000) == ParseResult::UpdatedFix);
    ASSERT_TRUE(s.has_fix);
    ASSERT_EQ(s.fix_quality, 1);
    ASSERT_EQ(s.satellites, 4);
    ASSERT_NEAR(s.hdop, 1.33, 1e-9);
    ASSERT_NEAR(s.lat, 13.0028260, 1e-6);
    ASSERT_NEAR(s.lon, 77.6799202, 1e-6);
    ASSERT_NEAR(s.alt_m, 921.8, 1e-9);
    ASSERT_STREQ(s.time_utc, "045519.50");
    ASSERT_EQ(s.fix_mono_ms, (int64_t)1000);
}

static void test_gga_no_fix_keeps_position_but_not_time()
{
    Snapshot s;
    wbr_gps::nmea_apply(kGGA, s, 1000);
    const double last_lat = s.lat;

    const std::string line = with_checksum(
        "$GNGGA,045530.50,1300.16956,N,07740.79521,E,0,00,99.99,921.8,M,-86.3,M,,");

    ASSERT_TRUE(wbr_gps::nmea_apply(line, s, 5000) == ParseResult::UpdatedFix);
    ASSERT_FALSE(s.has_fix);
    ASSERT_EQ(s.fix_quality, 0);
    ASSERT_NEAR(s.lat, last_lat, 1e-9);      // position retained
    ASSERT_EQ(s.fix_mono_ms, (int64_t)1000); // NOT advanced: the fix is stale
}

static void test_rmc_speed()
{
    Snapshot s;
    ASSERT_TRUE(wbr_gps::nmea_apply(kRMC, s, 1000) == ParseResult::UpdatedSpeed);
    ASSERT_NEAR(s.speed_kph, 0.0, 1e-9);

    // 10 knots is 18.52 km/h.
    const std::string moving = with_checksum(
        "$GNRMC,045519.50,A,1300.16956,N,07740.79521,E,10.000,,010926,,,A");
    ASSERT_TRUE(wbr_gps::nmea_apply(moving, s, 1000) == ParseResult::UpdatedSpeed);
    ASSERT_NEAR(s.speed_kph, 18.52, 1e-6);

    // Status 'V' (void) must zero the speed rather than keep a stale value.
    const std::string void_fix = with_checksum(
        "$GNRMC,045519.50,V,1300.16956,N,07740.79521,E,10.000,,010926,,,N");
    ASSERT_TRUE(wbr_gps::nmea_apply(void_fix, s, 1000) == ParseResult::UpdatedSpeed);
    ASSERT_NEAR(s.speed_kph, 0.0, 1e-9);
}

static void test_rejects_garbage()
{
    Snapshot s;
    const Snapshot before = s;
    ASSERT_TRUE(wbr_gps::nmea_apply(kTorn, s, 1000) == ParseResult::Rejected);
    // A rejected line must leave the snapshot completely untouched.
    ASSERT_EQ(s.has_fix, before.has_fix);
    ASSERT_NEAR(s.lat, before.lat, 1e-12);
    ASSERT_EQ(s.fix_mono_ms, before.fix_mono_ms);

    ASSERT_TRUE(wbr_gps::nmea_apply("", s, 1000) == ParseResult::Rejected);
    ASSERT_TRUE(wbr_gps::nmea_apply("$", s, 1000) == ParseResult::Rejected);
    ASSERT_TRUE(wbr_gps::nmea_apply("$GN", s, 1000) == ParseResult::Rejected);
    ASSERT_TRUE(wbr_gps::nmea_apply(std::string(4096, 'A'), s, 1000) == ParseResult::Rejected);

    // Embedded NUL must not be parsed into acceptance.
    std::string with_nul = kGGA;
    with_nul[10] = '\0';
    ASSERT_TRUE(wbr_gps::nmea_apply(with_nul, s, 1000) == ParseResult::Rejected);

    // Correct checksum but too few fields: still rejected.
    ASSERT_TRUE(wbr_gps::nmea_apply(with_checksum("$GNGGA,045519.50,1300.1"), s, 1000)
                == ParseResult::Rejected);
}

static void test_ignores_unused_sentences()
{
    Snapshot s;
    ASSERT_TRUE(wbr_gps::nmea_apply(
        "$GNGSA,A,3,32,31,25,18,,,,,,,,,2.39,1.33,1.99*19", s, 1000)
        == ParseResult::Ignored);
    ASSERT_TRUE(wbr_gps::nmea_apply(
        "$GPGSV,3,1,11,08,09,296,,10,36,007,,18,42,135,25,23,33,048,*76", s, 1000)
        == ParseResult::Ignored);
}

static void test_gp_and_gn_talkers()
{
    // Both $GP (GPS only) and $GN (multi-constellation) must be accepted.
    Snapshot s;
    const std::string gp = with_checksum(
        "$GPGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,");
    ASSERT_TRUE(wbr_gps::nmea_apply(gp, s, 1000) == ParseResult::UpdatedFix);
    ASSERT_TRUE(s.has_fix);
}

static void test_non_finite_hdop_and_altitude()
{
    // Parser must guard non-finite values that strtod can produce.
    // Feed hdop and altitude fields with 1e400 (parses as infinity).
    Snapshot s;
    const std::string gga_inf_hdop = with_checksum(
        "$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1e400,921.8,M,-86.3,M,,");
    ASSERT_TRUE(wbr_gps::nmea_apply(gga_inf_hdop, s, 1000) == ParseResult::UpdatedFix);
    // Parser must have replaced non-finite hdop with 0.0
    ASSERT_NEAR(s.hdop, 0.0, 1e-9);

    const std::string gga_inf_alt = with_checksum(
        "$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,1e400,M,-86.3,M,,");
    ASSERT_TRUE(wbr_gps::nmea_apply(gga_inf_alt, s, 1000) == ParseResult::UpdatedFix);
    // Parser must have replaced non-finite alt_m with 0.0
    ASSERT_NEAR(s.alt_m, 0.0, 1e-9);
}

static void test_gga_claims_fix_but_bad_coordinates()
{
    // Regression test for T4-A: fix_quality=1 claims a fix, but minutes >= 60
    // is impossible. has_fix must be false because coordinates are malformed,
    // not trusted based on a quality field alone.
    Snapshot s;
    const std::string bad_coord_gga = with_checksum(
        "$GNGGA,045519.50,1360.00000,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,");
    ASSERT_TRUE(wbr_gps::nmea_apply(bad_coord_gga, s, 1000) == ParseResult::UpdatedFix);
    // Quality field claims a fix, but coordinate decode failed.
    ASSERT_EQ(s.fix_quality, 1);
    // has_fix must be false: we do not hold a trustworthy position.
    ASSERT_FALSE(s.has_fix);
}

static void run_tests()
{
    test_checksum();
    test_coord();
    test_gga();
    test_gga_no_fix_keeps_position_but_not_time();
    test_rmc_speed();
    test_rejects_garbage();
    test_ignores_unused_sentences();
    test_gp_and_gn_talkers();
    test_non_finite_hdop_and_altitude();
    test_gga_claims_fix_but_bad_coordinates();
}

TEST_MAIN
