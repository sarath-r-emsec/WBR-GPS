#include "wbr_gps/json_io.hpp"
#include "test_util.h"

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
    a.hdop           = 1.33;
    a.lat            = 13.0028260;
    a.lon            = 77.6799202;
    a.alt_m          = 921.8;
    a.speed_kph      = 18.52;
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
    ASSERT_NEAR(b.hdop, 1.33, 1e-9);
    // Position must survive to at least 1e-7 degrees (~1 cm).
    ASSERT_NEAR(b.lat, 13.0028260, 1e-7);
    ASSERT_NEAR(b.lon, 77.6799202, 1e-7);
    ASSERT_NEAR(b.alt_m, 921.8, 1e-6);
    ASSERT_NEAR(b.speed_kph, 18.52, 1e-6);
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

static void run_tests()
{
    test_round_trip();
    test_all_false_round_trip();
    test_class_field();
    test_nmea_message();
    test_string_escaping();
    test_rejects_malformed();
    test_scanner_helpers();
}

TEST_MAIN
