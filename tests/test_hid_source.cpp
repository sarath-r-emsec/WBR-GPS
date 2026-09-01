#include "wbr_gps/hid_source.hpp"
#include "test_util.h"

#include <cstdint>

// Test the pure bit-decode function
static void test_hid_decode_lock_bit_clear_is_locked()
{
    unsigned char rep[8] = { 0 };
    rep[1] = 0x00;  // bit 0 clear = locked
    ASSERT_TRUE(wbr_gps::hid_decode_lock(rep, 2));
}

static void test_hid_decode_lock_bit_set_is_unlocked()
{
    unsigned char rep[8] = { 0 };
    rep[1] = 0x01;  // bit 0 set = unlocked
    ASSERT_FALSE(wbr_gps::hid_decode_lock(rep, 2));
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
    test_hid_decode_lock_bit_clear_is_locked();
    test_hid_decode_lock_bit_set_is_unlocked();
    test_hid_decode_lock_short_read_zero();
    test_hid_decode_lock_short_read_one();
    test_hid_source_discover();
}

TEST_MAIN
