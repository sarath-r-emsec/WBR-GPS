// Stands in for gsm_monitor in tests/regression_all_consumers.sh.
//
// gsm_monitor.cpp needs a live SDR to do anything meaningful past startup,
// and driving one for real in a regression script would mean tuning and
// decoding actual GSM traffic -- out of scope and not something to do
// casually. What gsm_monitor itself calls at startup (before it even opens
// the SDR: see the wbr_gps::get_once().device_present check in
// WBR-GSM/gsm_monitor.cpp) is exactly wbr_gps::get_once(), the same
// synchronous one-shot request its long-lived wbr_gps::Client wraps
// internally. This probe calls that real function, compiled against the
// same wbr_gps_core static library the daemon and every other consumer
// link -- an honest stand-in, not a pretend one.
//
// Usage: gps_get_once_probe SOCKET_PATH [TIMEOUT_MS]
// Prints one line of key=value pairs and exits 0 iff service_ok.
#include "wbr_gps/client.hpp"

#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv)
{
    const char* sock       = argc > 1 ? argv[1] : wbr_gps::kDefaultSocketPath;
    const int   timeout_ms = argc > 2 ? std::atoi(argv[2]) : 500;

    const wbr_gps::Snapshot s = wbr_gps::get_once(sock, timeout_ms);

    std::printf("service_ok=%d device_present=%d serial_ok=%d hid_ok=%d "
                "gpsdo_locked=%d has_fix=%d lat=%.7f lon=%.7f seq=%llu\n",
                (int)s.service_ok, (int)s.device_present, (int)s.serial_ok,
                (int)s.hid_ok, (int)s.gpsdo_locked, (int)s.has_fix,
                s.lat, s.lon, (unsigned long long)s.seq);

    return s.service_ok ? 0 : 1;
}
