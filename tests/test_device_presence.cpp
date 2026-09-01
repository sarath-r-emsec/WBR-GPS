#include "wbr_gps/device_presence.hpp"
#include "test_util.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

static std::string make_tmpdir()
{
    char tmpl[] = "/tmp/wbrgps_test_XXXXXX";
    const char* d = mkdtemp(tmpl);
    return d ? std::string(d) : std::string();
}

static void touch(const std::string& path)
{
    FILE* f = fopen(path.c_str(), "w");
    if (f) fclose(f);
}

static void test_scan_finds_leo_bodnar()
{
    const std::string dir = make_tmpdir();
    ASSERT_TRUE(!dir.empty());

    touch(dir + "/usb-Some_Other_Vendor_Widget-if00");
    touch(dir + "/usb-Leo_Bodnar_Electronics_LBE-1421_GPSDO_Locked_Clock_Source_0C7BB81010E5-if00");

    const std::string found = wbr_gps::scan_by_id_for(dir.c_str(), "Leo_Bodnar");
    ASSERT_TRUE(found.find("Leo_Bodnar") != std::string::npos);
    // The result must be a full path, not a bare filename.
    ASSERT_TRUE(found.rfind(dir, 0) == 0);

    // A needle that is not present must yield an empty string, not a guess.
    ASSERT_STREQ(wbr_gps::scan_by_id_for(dir.c_str(), "u-blox"), "");

    // A directory that does not exist must be handled, not crash.
    ASSERT_STREQ(wbr_gps::scan_by_id_for("/nonexistent-dir-xyz", "Leo_Bodnar"), "");

    unlink((dir + "/usb-Some_Other_Vendor_Widget-if00").c_str());
    unlink((dir + "/usb-Leo_Bodnar_Electronics_LBE-1421_GPSDO_Locked_Clock_Source_0C7BB81010E5-if00").c_str());
    rmdir(dir.c_str());
}

static void test_scan_is_case_insensitive()
{
    const std::string dir = make_tmpdir();
    ASSERT_TRUE(!dir.empty());
    touch(dir + "/usb-leo_bodnar_lowercase-if00");
    const std::string found = wbr_gps::scan_by_id_for(dir.c_str(), "Leo_Bodnar");
    ASSERT_TRUE(!found.empty());
    unlink((dir + "/usb-leo_bodnar_lowercase-if00").c_str());
    rmdir(dir.c_str());
}

static void run_tests()
{
    test_scan_finds_leo_bodnar();
    test_scan_is_case_insensitive();
}

TEST_MAIN
