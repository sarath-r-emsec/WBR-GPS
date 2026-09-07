#include "wbr_gps/hid_source.hpp"
#include "test_util.h"

#include <cstdint>

// Test the pure bit-decode function
// Every byte value below was observed on a real LBE-1421 with a human
// watching the front-panel LED. Byte 0 is the only byte that ever varies;
// bytes 1..63 are constant (0x00 then 0xff) in locked and unlocked states
// alike, which is why the two earlier decodes -- both keyed on byte 1 --
// were constants rather than signals.
static void test_hid_decode_lock_led_solid_is_locked()
{
    // LED SOLID. Seen across four sessions including one 20h continuous run.
    unsigned char rep[8] = { 0x7f, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    ASSERT_TRUE(wbr_gps::hid_decode_lock(rep, 8));
}

static void test_hid_decode_lock_led_blinking_is_unlocked()
{
    // LED BLINKING, captured during a post-replug reacquisition. Byte 0
    // alternates between exactly these two values and never reads 0x7f.
    unsigned char a[8] = { 0x6e, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    unsigned char b[8] = { 0x76, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    ASSERT_FALSE(wbr_gps::hid_decode_lock(a, 8));
    ASSERT_FALSE(wbr_gps::hid_decode_lock(b, 8));
}

// THE REGRESSION that started all of this. The original decode was
// `(rep[1] & 0x01) == 0`, and byte 1 is 0x00 in every report ever captured --
// so it answered "locked" unconditionally. A field replug exposed it: every
// UI kept saying "Locked" while the unit was visibly still acquiring.
static void test_hid_decode_lock_byte1_alone_cannot_decide()
{
    unsigned char unlocked_but_byte1_clear[8] =
        { 0x6e, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    ASSERT_FALSE(wbr_gps::hid_decode_lock(unlocked_but_byte1_clear, 8));
}

// A bit test (rep[0] & 0x01) fits the observed data just as well, and is
// rejected because it fails in the dangerous direction: it would call a
// status byte with other bits clear "locked". Whole-byte equality cannot.
static void test_hid_decode_lock_unobserved_values_are_not_locked()
{
    unsigned char bit0_set_but_never_seen[8] =
        { 0x7d, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    ASSERT_FALSE(wbr_gps::hid_decode_lock(bit0_set_but_never_seen, 8));
}

// An all-zero buffer is what "we learned nothing" looks like: on_readable()
// zero-initialises its report before reading. It must never read as locked.
static void test_hid_decode_lock_no_information_is_not_locked()
{
    unsigned char all_zero[8] = { 0 };
    ASSERT_FALSE(wbr_gps::hid_decode_lock(all_zero, 8));
    ASSERT_FALSE(wbr_gps::hid_decode_lock(all_zero, 0));
    ASSERT_FALSE(wbr_gps::hid_decode_lock(all_zero, 1));
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
    test_hid_decode_lock_led_solid_is_locked();
    test_hid_decode_lock_led_blinking_is_unlocked();
    test_hid_decode_lock_byte1_alone_cannot_decide();
    test_hid_decode_lock_unobserved_values_are_not_locked();
    test_hid_decode_lock_no_information_is_not_locked();
    test_hid_source_discover();
}

TEST_MAIN
