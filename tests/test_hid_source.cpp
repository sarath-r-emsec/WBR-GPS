#include "wbr_gps/hid_source.hpp"
#include "test_util.h"

#include <cstdint>

// Test the pure bit-decode function
// hid_decode_lock() reports "not confirmed locked" for every input, because
// no byte of this report has been shown to carry the lock state. Byte 1 was
// 0x00 in all 157 reports of a 20s capture spanning locked and unlocked
// periods, and byte 0 alternates 0x6e/0x76. These pin that contract so a
// future decode has to arrive with evidence rather than a guess.
static void test_hid_decode_lock_never_claims_locked()
{
    unsigned char locked_looking[8] = { 0x7f, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    unsigned char unlocked_bit[8]   = { 0x7f, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    unsigned char observed_a[8]     = { 0x6e, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    unsigned char observed_b[8]     = { 0x76, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    unsigned char all_zero[8]       = { 0 };
    ASSERT_FALSE(wbr_gps::hid_decode_lock(locked_looking, 8));
    ASSERT_FALSE(wbr_gps::hid_decode_lock(unlocked_bit, 8));
    ASSERT_FALSE(wbr_gps::hid_decode_lock(observed_a, 8));
    ASSERT_FALSE(wbr_gps::hid_decode_lock(observed_b, 8));
    ASSERT_FALSE(wbr_gps::hid_decode_lock(all_zero, 8));
    ASSERT_FALSE(wbr_gps::hid_decode_lock(all_zero, 0));
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
    test_hid_decode_lock_never_claims_locked();
    test_hid_source_discover();
}

TEST_MAIN
