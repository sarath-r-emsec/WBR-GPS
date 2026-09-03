#include "wbr_gps/hid_source.hpp"
#include "test_util.h"

#include <cstdint>

// Test the pure bit-decode function
// Byte layout captured live from an LBE-1421 on 2026-09-03, locked:
//     7f 00 ff ff ff ff ff ff ...   (64 bytes, ~1 per second)
static void test_hid_decode_lock_real_locked_report()
{
    unsigned char rep[8] = { 0x7f, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    ASSERT_TRUE(wbr_gps::hid_decode_lock(rep, 8));
}

static void test_hid_decode_lock_bit_set_is_unlocked()
{
    unsigned char rep[8] = { 0x7f, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    ASSERT_FALSE(wbr_gps::hid_decode_lock(rep, 8));
}

// THE REGRESSION. on_readable() zero-initialises its report buffer, so an
// all-zero report is exactly what "we learned nothing" looks like. The old
// decode was `(rep[1] & 0x01) == 0` alone, which called that LOCKED --
// asserting a disciplined 10 MHz reference that nothing had confirmed.
// Absence of evidence must read as unlocked.
static void test_hid_decode_lock_all_zero_report_is_not_locked()
{
    unsigned char rep[8] = { 0 };
    ASSERT_FALSE(wbr_gps::hid_decode_lock(rep, 8));
    ASSERT_FALSE(wbr_gps::hid_decode_lock(rep, 2));
}

// Any report that is not the status report yields no lock opinion at all,
// rather than a confident wrong one.
static void test_hid_decode_lock_wrong_marker_is_not_locked()
{
    unsigned char rep[8] = { 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    ASSERT_FALSE(wbr_gps::hid_decode_lock(rep, 8));
}

static void test_hid_decode_lock_short_read_zero()
{
    unsigned char rep[8] = { 0 };
    // n=0 should return false; caller should not use the result
    ASSERT_FALSE(wbr_gps::hid_decode_lock(rep, 0));
}

static void test_hid_decode_lock_short_read_one()
{
    unsigned char rep[8] = { 0 };
    // n=1 should return false; caller should not use the result
    ASSERT_FALSE(wbr_gps::hid_decode_lock(rep, 1));
}

// Test fd-level behavior: discover() finds the correct hidraw device
static void test_hid_source_discover()
{
    // This test runs on real hardware where a Leo Bodnar is attached.
    // discover() should return /dev/hidraw2 on this machine.
    const std::string result = wbr_gps::HidSource::discover();
    ASSERT_TRUE(!result.empty());
    ASSERT_TRUE(result.find("hidraw") != std::string::npos);
}

static void run_tests()
{
    test_hid_decode_lock_real_locked_report();
    test_hid_decode_lock_bit_set_is_unlocked();
    test_hid_decode_lock_all_zero_report_is_not_locked();
    test_hid_decode_lock_wrong_marker_is_not_locked();
    test_hid_decode_lock_short_read_zero();
    test_hid_decode_lock_short_read_one();
    test_hid_source_discover();
}

TEST_MAIN
