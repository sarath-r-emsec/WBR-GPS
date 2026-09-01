#include "wbr_gps/json_io.hpp"
#include "test_util.h"

#include <limits>

static void test_round_trip()
{
    wbr_gps::Snapshot a;
    a.seq            = 42;
    a.device_present = true;
    a.serial_ok      = true;
    a.hid_ok         = true;
    a.gpsdo_locked   = true;
    a.has_fix        = true;
    a.fix_quality    = 1;
    a.satellites     = 4;
    a.hdop           = 1.336;  // non-round value to catch precision regressions
    a.lat            = 13.0028260;
    a.lon            = 77.6799202;
    a.alt_m          = 921.856;  // non-round value to catch precision regressions
    a.speed_kph      = 18.527;   // non-round value to catch precision regressions
    a.time_utc       = "045519.50";
    a.fix_mono_ms    = 1000;

    const std::string js = wbr_gps::snapshot_to_json(a, 1480);

    wbr_gps::Snapshot b;
    ASSERT_TRUE(wbr_gps::snapshot_from_json(js, b));
    ASSERT_EQ(b.seq, (uint64_t)42);
    ASSERT_TRUE(b.device_present);
    ASSERT_TRUE(b.serial_ok);
    ASSERT_TRUE(b.hid_ok);
    ASSERT_TRUE(b.gpsdo_locked);
    ASSERT_TRUE(b.has_fix);
    ASSERT_EQ(b.fix_quality, 1);
    ASSERT_EQ(b.satellites, 4);
    // hdop: %.2f format allows ~±0.005 round-trip tolerance
    ASSERT_NEAR(b.hdop, 1.336, 5e-3);
    // Position must survive to at least 1e-7 degrees (~1 cm).
    ASSERT_NEAR(b.lat, 13.0028260, 1e-7);
    ASSERT_NEAR(b.lon, 77.6799202, 1e-7);
    // alt_m: %.2f format allows ~±0.005 round-trip tolerance
    ASSERT_NEAR(b.alt_m, 921.856, 5e-3);
    // speed_kph: %.3f format allows ~±0.0005 round-trip tolerance
    ASSERT_NEAR(b.speed_kph, 18.527, 5e-4);
    ASSERT_STREQ(b.time_utc, "045519.50");
    // fix_age_ms is computed at serialization: 1480 - 1000 = 480.
    ASSERT_EQ(b.fix_age_ms, (int64_t)480);
}

static void test_all_false_round_trip()
{
    // The default "nothing known" snapshot must survive intact. If booleans
    // silently default to true on parse, a dead daemon looks healthy.
    wbr_gps::Snapshot a;
    const std::string js = wbr_gps::snapshot_to_json(a, 0);
    wbr_gps::Snapshot b;
    b.has_fix = true;   // poison, to prove the parser overwrites it
    b.serial_ok = true;
    ASSERT_TRUE(wbr_gps::snapshot_from_json(js, b));
    ASSERT_FALSE(b.has_fix);
    ASSERT_FALSE(b.serial_ok);
    ASSERT_FALSE(b.device_present);
    ASSERT_EQ(b.fix_age_ms, (int64_t)0);   // never fixed: age reported as 0
}

static void test_class_field()
{
    wbr_gps::Snapshot a;
    const std::string js = wbr_gps::snapshot_to_json(a, 0);
    std::string cls;
    ASSERT_TRUE(wbr_gps::json_get_string(js, "class", cls));
    ASSERT_STREQ(cls, "FIX");
    // Output must be exactly one line: the framing is newline-delimited.
    ASSERT_TRUE(js.find('\n') == std::string::npos);
}

static void test_nmea_message()
{
    const std::string js = wbr_gps::nmea_to_json("$GNGGA,1,2*6F", 1756701234567LL, 3);
    std::string cls, raw;
    ASSERT_TRUE(wbr_gps::json_get_string(js, "class", cls));
    ASSERT_STREQ(cls, "NMEA");
    ASSERT_TRUE(wbr_gps::json_get_string(js, "raw", raw));
    ASSERT_STREQ(raw, "$GNGGA,1,2*6F");
    ASSERT_TRUE(js.find("\"dropped\":3") != std::string::npos);
    ASSERT_TRUE(js.find('\n') == std::string::npos);
}

static void test_string_escaping()
{
    // A raw sentence containing a quote or backslash must not break framing.
    const std::string js = wbr_gps::nmea_to_json("$GN\"A\\B", 0, 0);
    std::string raw;
    ASSERT_TRUE(wbr_gps::json_get_string(js, "raw", raw));
    ASSERT_STREQ(raw, "$GN\"A\\B");
}

static void test_high_byte_filtering()
{
    // High bytes (>= 0x7F) must be dropped to ensure valid UTF-8.
    // NMEA 0183 is ASCII-only; any byte >= 0x7F is corruption.
    // Concatenate byte values as chars to create test string.
    std::string input = "$GNGGA";
    input += (char)0x81;  // high byte
    input += "data";
    input += (char)0xFF;  // high byte

    const std::string js = wbr_gps::nmea_to_json(input, 0, 0);
    std::string raw;
    ASSERT_TRUE(wbr_gps::json_get_string(js, "raw", raw));

    // Verify all bytes in output are < 0x80 (valid ASCII/UTF-8).
    for (char c : raw) {
        unsigned char uc = (unsigned char)c;
        ASSERT_TRUE(uc < 0x80);
    }

    // Verify the ASCII parts survived intact.
    ASSERT_TRUE(raw.find("$GNGGA") == 0);
    ASSERT_TRUE(raw.find("data") != std::string::npos);
}

static void test_rejects_malformed()
{
    wbr_gps::Snapshot b;
    ASSERT_FALSE(wbr_gps::snapshot_from_json("", b));
    ASSERT_FALSE(wbr_gps::snapshot_from_json("not json", b));
    ASSERT_FALSE(wbr_gps::snapshot_from_json("{\"class\":\"ERROR\"}", b));
}

static void test_scanner_helpers()
{
    const std::string js = "{\"a\":\"x\",\"b\":true,\"c\":false}";
    std::string s;
    bool v = false;
    ASSERT_TRUE(wbr_gps::json_get_string(js, "a", s));
    ASSERT_STREQ(s, "x");
    ASSERT_TRUE(wbr_gps::json_get_bool(js, "b", v));
    ASSERT_TRUE(v);
    ASSERT_TRUE(wbr_gps::json_get_bool(js, "c", v));
    ASSERT_FALSE(v);
    ASSERT_FALSE(wbr_gps::json_get_string(js, "missing", s));
}

static void test_non_finite_values()
{
    // Non-finite doubles should be converted to 0.0 in JSON output.
    wbr_gps::Snapshot a;
    a.has_fix = true;
    a.hdop = std::numeric_limits<double>::quiet_NaN();
    a.alt_m = std::numeric_limits<double>::infinity();
    a.speed_kph = -std::numeric_limits<double>::infinity();

    const std::string js = wbr_gps::snapshot_to_json(a, 0);

    // JSON must be parseable (no bare nan/inf which would fail json.loads).
    wbr_gps::Snapshot b;
    ASSERT_TRUE(wbr_gps::snapshot_from_json(js, b));
    // Non-finite values replaced with 0.0
    ASSERT_NEAR(b.hdop, 0.0, 1e-9);
    ASSERT_NEAR(b.alt_m, 0.0, 1e-9);
    ASSERT_NEAR(b.speed_kph, 0.0, 1e-9);
}

static void test_oversized_truncation()
{
    // Buffer overflow on very long time_utc field should produce ERROR class.
    wbr_gps::Snapshot a;
    a.has_fix = true;
    // Create a time_utc that's very long (>700 bytes when formatted).
    a.time_utc = std::string(750, 'x');

    const std::string js = wbr_gps::snapshot_to_json(a, 0);
    std::string cls;
    ASSERT_TRUE(wbr_gps::json_get_string(js, "class", cls));
    ASSERT_STREQ(cls, "ERROR");
    // Parser must reject the ERROR class
    wbr_gps::Snapshot b;
    ASSERT_FALSE(wbr_gps::snapshot_from_json(js, b));
}

static void test_precision_non_round_values()
{
    // Use non-round values to catch precision regressions.
    // hdop: %.2f should preserve to ~±0.005
    // alt_m: %.2f should preserve to ~±0.005
    // speed_kph: %.3f should preserve to ~±0.0005
    wbr_gps::Snapshot a;
    a.has_fix = true;
    a.hdop = 1.234;
    a.alt_m = 567.891;
    a.speed_kph = 12.345;

    const std::string js = wbr_gps::snapshot_to_json(a, 0);
    wbr_gps::Snapshot b;
    ASSERT_TRUE(wbr_gps::snapshot_from_json(js, b));

    // Verify precision within expected tolerances.
    ASSERT_NEAR(b.hdop, 1.234, 5e-3);
    ASSERT_NEAR(b.alt_m, 567.891, 5e-3);
    ASSERT_NEAR(b.speed_kph, 12.345, 5e-4);
}

static void run_tests()
{
    test_round_trip();
    test_all_false_round_trip();
    test_class_field();
    test_nmea_message();
    test_string_escaping();
    test_high_byte_filtering();
    test_rejects_malformed();
    test_scanner_helpers();
    test_non_finite_values();
    test_oversized_truncation();
    test_precision_non_round_values();
}

TEST_MAIN
