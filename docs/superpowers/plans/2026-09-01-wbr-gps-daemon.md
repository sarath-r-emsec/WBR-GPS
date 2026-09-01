# WBR-GPS Proxy Daemon Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a single-owner GPS daemon so every program in the SIGINT project can read position, fix status, GPSDO lock and raw NMEA concurrently without contending for `/dev/ttyACM0`.

**Architecture:** One process, `wbr-gpsd`, single-threaded on an `epoll` loop, exclusively owning the Leo Bodnar serial port (`TIOCEXCL`) and hidraw device. It serves newline-delimited JSON over a Unix domain socket, supporting both one-shot `get` and streaming `watch`. Client libraries in C++ and Python contain no device code at all, so no consumer can fall back to opening the hardware directly.

**Tech Stack:** C++17, CMake 3.16, POSIX (`epoll`, `termios`, `openpty`, `flock`), systemd, udev, Python 3 (stdlib `socket`/`json` only). No external C++ dependencies. Tests are standalone binaries with hand-rolled assertion macros, matching WBR-GSM and WBR-SA.

**Spec:** `WBR-GPS/docs/superpowers/specs/2026-09-01-gps-proxy-daemon-design.md`

## Global Constraints

- **C++ standard: C++17.** WBR-SA is C++17 and WBR-GSM is C++20. The client library is included by both, so it must compile as C++17. Do not use C++20 features anywhere in `include/wbr_gps/`.
- **CMake minimum 3.16**, matching both existing projects.
- **No external C++ dependencies.** No gtest, no Catch2, no JSON library. JSON is emitted with `snprintf` and parsed with a small hand-rolled scanner — the message set is tiny and fixed.
- **File size:** 200–400 lines typical, 800 maximum.
- **All durations and ages use `CLOCK_MONOTONIC`.** `CLOCK_REALTIME` is used only for human-facing display timestamps.
- **`SIGPIPE` is ignored process-wide; every socket write uses `MSG_NOSIGNAL`.**
- **Client libraries must contain no `open()`, no `termios`, and no `/dev` path.** This is guarantee G5, enforced by a grep-based test in Task 12.
- **Socket path:** `/run/wbr-gps/gpsd.sock`, mode `0660`, group `dialout`.
- **Serial device path:** `/dev/gpsdo` (udev symlink created in Task 11), falling back to `/dev/serial/by-id/*Leo_Bodnar*`.
- **Protocol version:** `1`, sent in `HELLO` as `"proto":1`.
- **Wire units:** degrees signed (N/E positive), metres, km/h, milliseconds.
- **Test style:** standalone `test_*.cpp` binaries using `tests/test_util.h` macros. No test framework.
- **Coverage target:** 80% minimum, measured with `gcov`/`lcov`.

---

## File Structure

| File | Responsibility |
|---|---|
| `include/wbr_gps/gps_types.hpp` | `Snapshot` struct and constants, shared daemon↔client |
| `include/wbr_gps/nmea_parser.hpp` | Pure parsing declarations, zero I/O |
| `include/wbr_gps/json_io.hpp` | Snapshot↔JSON serialization, shared daemon↔client |
| `include/wbr_gps/client.hpp` | C++ client library public interface |
| `src/nmea_parser.cpp` | Checksum validation, GGA/RMC field extraction |
| `src/json_io.cpp` | JSON emit and scan |
| `src/state_store.cpp` | The one authoritative snapshot, sequence counter, staleness |
| `src/serial_source.cpp` | Owns the serial port: `TIOCEXCL`, line assembly, reconnect backoff |
| `src/hid_source.cpp` | Owns hidraw: lock bit |
| `src/device_presence.cpp` | `/dev/serial/by-id` scan for ref-present |
| `src/server.cpp` | Listening socket, singleton lock, clients, bounded queues |
| `src/main.cpp` | Argument parsing, signals, the `epoll` loop |
| `src/client.cpp` | C++ client implementation |
| `python/wbr_gps_client.py` | Python client for Django |
| `systemd/wbr-gpsd.service` | Supervision |
| `udev/99-wbr-gps.rules` | ModemManager suppression and stable symlink |
| `tests/test_util.h` | Assertion macros matching project style |
| `tests/test_nmea_parser.cpp` | Layer 1 unit tests |
| `tests/test_json_io.cpp` | Serialization round-trip tests |
| `tests/test_state_store.cpp` | Sequence and staleness tests |
| `tests/pty_harness.h` | `openpty()` fake serial device for Layer 2 |
| `tests/test_serial_source.cpp` | Serial reading against a pty |
| `tests/test_concurrency.cpp` | Layer 3 scenarios S1–S15 |
| `tests/fixtures/nmea_real.txt` | Real captured NMEA |
| `tests/fixtures/nmea_torn.txt` | The corrupted lines from the reproduction |

---

## Task 1: Project scaffold, shared types, test harness

**Files:**
- Create: `WBR-GPS/CMakeLists.txt`
- Create: `WBR-GPS/include/wbr_gps/gps_types.hpp`
- Create: `WBR-GPS/src/gps_types.cpp`
- Create: `WBR-GPS/tests/test_util.h`
- Create: `WBR-GPS/tests/test_types.cpp`

**Interfaces:**
- Consumes: nothing (first task)
- Produces: `wbr_gps::Snapshot` struct; `wbr_gps::kProtoVersion`; `wbr_gps::kDefaultSocketPath`; `wbr_gps::kDefaultSerialPath`; `wbr_gps::kDefaultBaud`; `wbr_gps::now_mono_ms() -> int64_t`; test macros `ASSERT_EQ`, `ASSERT_TRUE`, `ASSERT_FALSE`, `ASSERT_STREQ`, `ASSERT_NEAR`, `TEST_MAIN`

- [ ] **Step 1: Create the shared types header**

Create `WBR-GPS/include/wbr_gps/gps_types.hpp`:

```cpp
#pragma once

#include <cstdint>
#include <string>

namespace wbr_gps {

inline constexpr int         kProtoVersion      = 1;
inline constexpr const char* kDefaultSocketPath = "/run/wbr-gps/gpsd.sock";
inline constexpr const char* kDefaultSerialPath = "/dev/gpsdo";
inline constexpr int         kDefaultBaud       = 115200;

// Milliseconds on CLOCK_MONOTONIC. Never wall time: the GPSDO can step the
// system clock, and a negative fix age is a classic bug in GPS daemons.
int64_t now_mono_ms();

// One complete view of GPS state. Every health field is independent on
// purpose: "I could not read the device" must never be confusable with "the
// device is not locked". That conflation is the original bug.
struct Snapshot {
    uint64_t seq = 0;

    // --- health, four independent signals ---
    bool device_present = false;  // a Leo Bodnar is attached to USB at all
    bool serial_ok      = false;  // the daemon currently holds the serial port
    bool hid_ok         = false;  // the daemon currently holds the hidraw device
    bool service_ok     = false;  // client-side only: a live daemon connection

    // --- GPSDO lock, distinct from NMEA fix ---
    bool gpsdo_locked = false;

    // --- NMEA fix ---
    bool        has_fix     = false;
    int         fix_quality = 0;     // GGA field 6: 0 none, 1 GPS, 2 DGPS
    int         satellites  = 0;
    double      hdop        = 0.0;
    double      lat         = 0.0;   // signed degrees, north positive
    double      lon         = 0.0;   // signed degrees, east positive
    double      alt_m       = 0.0;
    double      speed_kph   = 0.0;
    std::string time_utc;            // "HHMMSS.SS" as sent

    // CLOCK_MONOTONIC ms when lat/lon were last set. 0 means never.
    int64_t fix_mono_ms = 0;

    // Age of the fix in ms, computed when the snapshot is serialized.
    int64_t fix_age_ms = 0;
};

} // namespace wbr_gps
```

- [ ] **Step 2: Create the test harness header**

Create `WBR-GPS/tests/test_util.h`, matching the macro style already used in `WBR-GSM/test_gsm_si.cpp`:

```cpp
#pragma once

#include <cmath>
#include <cstdio>
#include <string>

static int n_pass = 0;
static int n_fail = 0;

#define ASSERT_TRUE(x)                                                              \
    do {                                                                            \
        if (!(x)) {                                                                 \
            fprintf(stderr, "FAIL %s:%d: %s is false\n", __FILE__, __LINE__, #x);   \
            n_fail++;                                                               \
        } else {                                                                    \
            n_pass++;                                                               \
        }                                                                           \
    } while (0)

#define ASSERT_FALSE(x) ASSERT_TRUE(!(x))

#define ASSERT_EQ(a, b)                                                             \
    do {                                                                            \
        auto _a = (a);                                                              \
        auto _b = (b);                                                              \
        if (!(_a == _b)) {                                                          \
            fprintf(stderr, "FAIL %s:%d: %s != %s\n", __FILE__, __LINE__, #a, #b);  \
            n_fail++;                                                               \
        } else {                                                                    \
            n_pass++;                                                               \
        }                                                                           \
    } while (0)

#define ASSERT_STREQ(a, b)                                                          \
    do {                                                                            \
        if (std::string(a) != std::string(b)) {                                     \
            fprintf(stderr, "FAIL %s:%d: \"%s\" != \"%s\"\n", __FILE__, __LINE__,   \
                    std::string(a).c_str(), std::string(b).c_str());                \
            n_fail++;                                                               \
        } else {                                                                    \
            n_pass++;                                                               \
        }                                                                           \
    } while (0)

#define ASSERT_NEAR(a, b, tol)                                                      \
    do {                                                                            \
        double _a = (double)(a);                                                    \
        double _b = (double)(b);                                                    \
        if (std::fabs(_a - _b) > (tol)) {                                           \
            fprintf(stderr, "FAIL %s:%d: %s (%.9f) != %s (%.9f)\n",                 \
                    __FILE__, __LINE__, #a, _a, #b, _b);                            \
            n_fail++;                                                               \
        } else {                                                                    \
            n_pass++;                                                               \
        }                                                                           \
    } while (0)

#define TEST_MAIN                                                                   \
    int main() {                                                                    \
        run_tests();                                                                \
        printf("%d passed, %d failed\n", n_pass, n_fail);                           \
        return n_fail == 0 ? 0 : 1;                                                 \
    }
```

- [ ] **Step 3: Write the failing test**

Create `WBR-GPS/tests/test_types.cpp`:

```cpp
#include "wbr_gps/gps_types.hpp"
#include "test_util.h"

#include <unistd.h>

static void run_tests()
{
    // A default Snapshot must mean "nothing known", not "everything fine".
    wbr_gps::Snapshot s;
    ASSERT_FALSE(s.device_present);
    ASSERT_FALSE(s.serial_ok);
    ASSERT_FALSE(s.hid_ok);
    ASSERT_FALSE(s.service_ok);
    ASSERT_FALSE(s.gpsdo_locked);
    ASSERT_FALSE(s.has_fix);
    ASSERT_EQ(s.seq, (uint64_t)0);
    ASSERT_EQ(s.fix_mono_ms, (int64_t)0);

    // now_mono_ms must advance and must never be zero or negative.
    const int64_t t0 = wbr_gps::now_mono_ms();
    ASSERT_TRUE(t0 > 0);
    usleep(20000);
    const int64_t t1 = wbr_gps::now_mono_ms();
    ASSERT_TRUE(t1 > t0);
    ASSERT_TRUE(t1 - t0 >= 15);
}

TEST_MAIN
```

- [ ] **Step 4: Run the test to verify it fails**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS && mkdir -p build && cd build && cmake .. && make test_types
```

Expected: link failure, `undefined reference to wbr_gps::now_mono_ms()`.

- [ ] **Step 5: Implement `now_mono_ms`**

Create `WBR-GPS/src/gps_types.cpp`:

```cpp
#include "wbr_gps/gps_types.hpp"

#include <ctime>

namespace wbr_gps {

int64_t now_mono_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

} // namespace wbr_gps
```

- [ ] **Step 6: Create the build file**

Create `WBR-GPS/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(WBR-GPS C CXX)

# C++17, not 20: WBR-SA is C++17 and includes our client headers.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

option(WBR_GPS_COVERAGE "Build with gcov instrumentation" OFF)
if(WBR_GPS_COVERAGE)
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Shared between the daemon and the client library.
add_library(wbr_gps_core STATIC
  src/gps_types.cpp
)

enable_testing()

add_executable(test_types tests/test_types.cpp)
target_include_directories(test_types PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_types PRIVATE wbr_gps_core)
add_test(NAME types COMMAND test_types)
```

- [ ] **Step 7: Run the test to verify it passes**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make test_types && ./test_types
```

Expected: `11 passed, 0 failed`, exit status 0.

- [ ] **Step 8: Commit**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS
git add CMakeLists.txt include/ src/ tests/
git commit -m "feat: WBR-GPS scaffold with shared snapshot type and test harness"
```

---

## Task 2: NMEA parser (pure, no I/O)

This is the testable core. It replaces three divergent parsers with one.

**Files:**
- Create: `WBR-GPS/include/wbr_gps/nmea_parser.hpp`
- Create: `WBR-GPS/src/nmea_parser.cpp`
- Create: `WBR-GPS/tests/test_nmea_parser.cpp`
- Create: `WBR-GPS/tests/fixtures/nmea_real.txt`
- Create: `WBR-GPS/tests/fixtures/nmea_torn.txt`
- Modify: `WBR-GPS/CMakeLists.txt`

**Interfaces:**
- Consumes: `wbr_gps::Snapshot`, `wbr_gps::now_mono_ms()` from Task 1
- Produces: `enum class wbr_gps::ParseResult { Rejected, Ignored, UpdatedFix, UpdatedSpeed }`; `wbr_gps::nmea_checksum_ok(const std::string&) -> bool`; `wbr_gps::nmea_coord(const std::string& field, char hemi, double& out) -> bool`; `wbr_gps::nmea_split(const std::string&) -> std::vector<std::string>`; `wbr_gps::nmea_apply(const std::string& line, Snapshot& snap, int64_t now_mono_ms) -> ParseResult`

- [ ] **Step 1: Create the fixture files**

Create `WBR-GPS/tests/fixtures/nmea_real.txt` with real sentences captured from the LBE-1421 on 2026-09-01:

```
$GNRMC,045519.50,A,1300.16956,N,07740.79521,E,0.000,,010926,,,A*68
$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F
$GNGSA,A,3,32,31,25,18,,,,,,,,,2.39,1.33,1.99*19
$GPGSV,3,1,11,08,09,296,,10,36,007,,18,42,135,25,23,33,048,*76
$GNRMC,045520.50,A,1300.16956,N,07740.79521,E,0.000,,010926,,,A*62
$GNGGA,045520.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*65
```

Create `WBR-GPS/tests/fixtures/nmea_torn.txt`. The first line is verbatim output from the three-concurrent-reader reproduction in the spec — the exact corruption this project exists to eliminate:

```
,,3,,5$G32,,,9151,,2,96*G3,,1,,,8,,6379$M055A3.5N7.500,*
$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*00
$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,
$GN
$
```

- [ ] **Step 2: Create the parser header**

Create `WBR-GPS/include/wbr_gps/nmea_parser.hpp`:

```cpp
#pragma once

#include "wbr_gps/gps_types.hpp"

#include <string>
#include <vector>

namespace wbr_gps {

enum class ParseResult {
    Rejected,      // malformed: no '$', too short/long, bad or missing checksum
    Ignored,       // well-formed, but not a sentence type we consume
    UpdatedFix,    // GGA applied
    UpdatedSpeed,  // RMC applied
};

// XOR of every byte between '$' and '*' must equal the two hex digits after
// '*'. A line with no '*' fails: an unchecksummed sentence is exactly what a
// torn read looks like, so we refuse to trust it.
bool nmea_checksum_ok(const std::string& line);

// Decode DDMM.MMMM / DDDMM.MMMM plus a hemisphere character into signed
// decimal degrees. Returns false if the field is empty or malformed, rather
// than yielding 0 and passing as a valid fix on the equator.
bool nmea_coord(const std::string& field, char hemi, double& out);

// Split on ',' after truncating at '*'.
std::vector<std::string> nmea_split(const std::string& line);

// Apply one sentence to `snap`. `snap` is modified only when accepted.
ParseResult nmea_apply(const std::string& line, Snapshot& snap, int64_t now_mono_ms);

} // namespace wbr_gps
```

- [ ] **Step 3: Write the failing tests**

Create `WBR-GPS/tests/test_nmea_parser.cpp`:

```cpp
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
}

TEST_MAIN
```

- [ ] **Step 4: Run the tests to verify they fail**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make test_nmea_parser
```

Expected: link failure, `undefined reference to wbr_gps::nmea_checksum_ok`.

- [ ] **Step 5: Implement the parser**

Create `WBR-GPS/src/nmea_parser.cpp`. Ported from `WBR-SA/src/gps_reader.cpp`, which already validates checksums, with the RMC field indices corrected (WBR-GSM's `sscanf`-based version reads the wrong field) and rejection made explicit:

```cpp
#include "wbr_gps/nmea_parser.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>

namespace wbr_gps {

namespace {
// The longest legal NMEA sentence is 82 bytes. Anything materially longer is
// a torn or concatenated read, not a sentence.
constexpr size_t kMaxLine = 128;
} // namespace

bool nmea_checksum_ok(const std::string& line)
{
    if (line.size() < 7 || line.size() > kMaxLine) return false;
    if (line[0] != '$') return false;

    const size_t star = line.find('*');
    if (star == std::string::npos || star + 3 > line.size()) return false;

    unsigned cs = 0;
    for (size_t i = 1; i < star; ++i) {
        const unsigned char c = (unsigned char)line[i];
        // A NUL or any control/high byte inside a sentence means a torn read.
        if (c < 0x20 || c > 0x7E) return false;
        cs ^= c;
    }

    const std::string hex = line.substr(star + 1, 2);
    for (char c : hex) {
        if (!std::isxdigit((unsigned char)c)) return false;
    }
    return cs == (unsigned)std::strtoul(hex.c_str(), nullptr, 16);
}

bool nmea_coord(const std::string& field, char hemi, double& out)
{
    // DDMM.MMMM needs at least 4 characters. Reject short fields instead of
    // letting strtod return 0 and pass as a valid fix on the equator.
    if (field.size() < 4) return false;
    if (hemi != 'N' && hemi != 'S' && hemi != 'E' && hemi != 'W') return false;

    char* end = nullptr;
    const double raw = std::strtod(field.c_str(), &end);
    if (end == field.c_str()) return false;
    if (!std::isfinite(raw)) return false;

    const double deg = std::floor(raw / 100.0);
    const double min = raw - deg * 100.0;
    if (min >= 60.0) return false;

    double v = deg + min / 60.0;
    if (hemi == 'S' || hemi == 'W') v = -v;
    if (std::fabs(v) > 180.0) return false;

    out = v;
    return true;
}

std::vector<std::string> nmea_split(const std::string& line)
{
    std::vector<std::string> out;
    std::string cur;
    const size_t star = line.find('*');
    const size_t end  = (star == std::string::npos) ? line.size() : star;
    for (size_t i = 0; i < end; ++i) {
        if (line[i] == ',') {
            out.push_back(cur);
            cur.clear();
        } else {
            cur += line[i];
        }
    }
    out.push_back(cur);
    return out;
}

ParseResult nmea_apply(const std::string& line, Snapshot& snap, int64_t now_mono_ms)
{
    if (!nmea_checksum_ok(line)) return ParseResult::Rejected;

    // line[0]=='$', [1..2]=talker ("GP"/"GN"), [3..5]=sentence type.
    const std::string tag = line.substr(3, 3);
    const std::vector<std::string> f = nmea_split(line);

    if (tag == "GGA") {
        // 1=time 2=lat 3=N/S 4=lon 5=E/W 6=quality 7=sats 8=hdop 9=alt
        if (f.size() < 10) return ParseResult::Rejected;

        snap.time_utc    = f[1];
        snap.fix_quality = std::atoi(f[6].c_str());
        snap.has_fix     = snap.fix_quality > 0;
        snap.satellites  = std::atoi(f[7].c_str());
        snap.hdop        = std::strtod(f[8].c_str(), nullptr);

        double lat = 0.0, lon = 0.0;
        if (snap.has_fix && !f[3].empty() && !f[5].empty() &&
            nmea_coord(f[2], f[3][0], lat) &&
            nmea_coord(f[4], f[5][0], lon)) {
            snap.lat         = lat;
            snap.lon         = lon;
            snap.alt_m       = std::strtod(f[9].c_str(), nullptr);
            snap.fix_mono_ms = now_mono_ms;
        }
        // With no fix we keep the last known position but do NOT advance
        // fix_mono_ms, so age reporting correctly shows the data as stale.
        return ParseResult::UpdatedFix;
    }

    if (tag == "RMC") {
        // 1=time 2=status 3=lat 4=N/S 5=lon 6=E/W 7=speed(knots) 8=track 9=date
        if (f.size() < 8) return ParseResult::Rejected;
        snap.speed_kph = (f[2] == "A")
            ? std::strtod(f[7].c_str(), nullptr) * 1.852
            : 0.0;
        return ParseResult::UpdatedSpeed;
    }

    return ParseResult::Ignored;
}

} // namespace wbr_gps
```

- [ ] **Step 6: Add to the build**

In `WBR-GPS/CMakeLists.txt`, add `src/nmea_parser.cpp` to the `wbr_gps_core` source list, then append:

```cmake
add_executable(test_nmea_parser tests/test_nmea_parser.cpp)
target_include_directories(test_nmea_parser PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_nmea_parser PRIVATE wbr_gps_core)
add_test(NAME nmea_parser COMMAND test_nmea_parser)
```

- [ ] **Step 7: Run the tests to verify they pass**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make test_nmea_parser && ./test_nmea_parser
```

Expected: `0 failed`, exit 0. If `test_rejects_garbage` fails on the torn line, the checksum guard is wrong — that assertion is the entire point of this task.

- [ ] **Step 8: Commit**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS
git add include/wbr_gps/nmea_parser.hpp src/nmea_parser.cpp tests/test_nmea_parser.cpp tests/fixtures CMakeLists.txt
git commit -m "feat: single checksum-validating NMEA parser replacing three divergent copies"
```

---

## Task 3: JSON serialization

Hand-rolled because the message set is tiny and fixed, and a JSON dependency in a header included by both WBR-GSM and WBR-SA is a cost neither project should pay.

**Files:**
- Create: `WBR-GPS/include/wbr_gps/json_io.hpp`
- Create: `WBR-GPS/src/json_io.cpp`
- Create: `WBR-GPS/tests/test_json_io.cpp`
- Modify: `WBR-GPS/CMakeLists.txt`

**Interfaces:**
- Consumes: `wbr_gps::Snapshot` (Task 1)
- Produces: `wbr_gps::snapshot_to_json(const Snapshot&, int64_t now_mono_ms) -> std::string`; `wbr_gps::snapshot_from_json(const std::string&, Snapshot& out) -> bool`; `wbr_gps::nmea_to_json(const std::string& raw, int64_t t_unix_ms, uint64_t dropped) -> std::string`; `wbr_gps::json_get_string(const std::string&, const char* key, std::string& out) -> bool`; `wbr_gps::json_get_bool(const std::string&, const char* key, bool& out) -> bool`

- [ ] **Step 1: Write the failing test**

Create `WBR-GPS/tests/test_json_io.cpp`:

```cpp
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
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make test_json_io
```

Expected: `fatal error: wbr_gps/json_io.hpp: No such file or directory`.

- [ ] **Step 3: Create the header**

Create `WBR-GPS/include/wbr_gps/json_io.hpp`:

```cpp
#pragma once

#include "wbr_gps/gps_types.hpp"

#include <cstdint>
#include <string>

namespace wbr_gps {

// Emit one FIX message as a single line, with no trailing newline. fix_age_ms
// is computed here from now_mono_ms so that every consumer sees a consistent
// age regardless of when it reads the line.
std::string snapshot_to_json(const Snapshot& s, int64_t now_mono_ms);

// Parse a FIX message. Returns false unless "class" is exactly "FIX".
bool snapshot_from_json(const std::string& line, Snapshot& out);

// Emit one NMEA passthrough message as a single line.
std::string nmea_to_json(const std::string& raw, int64_t t_unix_ms, uint64_t dropped);

// Minimal scanners for the small fixed message set. They look for the literal
// "<key>": at the top level; the protocol has no nested objects, so this is
// sufficient and avoids a JSON dependency in a header two other projects include.
bool json_get_string(const std::string& js, const char* key, std::string& out);
bool json_get_bool(const std::string& js, const char* key, bool& out);
bool json_get_double(const std::string& js, const char* key, double& out);
bool json_get_int64(const std::string& js, const char* key, int64_t& out);

} // namespace wbr_gps
```

- [ ] **Step 4: Implement**

Create `WBR-GPS/src/json_io.cpp`:

```cpp
#include "wbr_gps/json_io.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace wbr_gps {

namespace {

std::string escape(const std::string& s)
{
    std::string o;
    o.reserve(s.size() + 8);
    for (char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if ((unsigned char)c < 0x20)  { /* drop control bytes */ }
        else                                { o += c; }
    }
    return o;
}

// Find the value position for "key": at the top level.
size_t find_value(const std::string& js, const char* key)
{
    const std::string pat = std::string("\"") + key + "\":";
    const size_t at = js.find(pat);
    if (at == std::string::npos) return std::string::npos;
    return at + pat.size();
}

} // namespace

std::string snapshot_to_json(const Snapshot& s, int64_t now_mono_ms)
{
    // Age is 0 when we have never had a fix, rather than a huge number, so
    // clients distinguish "never" via has_fix/fix_mono_ms, not via magnitude.
    const int64_t age = (s.fix_mono_ms == 0) ? 0 : (now_mono_ms - s.fix_mono_ms);

    char buf[768];
    const int n = snprintf(buf, sizeof buf,
        "{\"class\":\"FIX\",\"seq\":%llu,"
        "\"device_present\":%s,\"serial_ok\":%s,\"hid_ok\":%s,"
        "\"gpsdo_locked\":%s,"
        "\"has_fix\":%s,\"fix_quality\":%d,\"satellites\":%d,\"hdop\":%.2f,"
        "\"lat\":%.7f,\"lon\":%.7f,\"alt_m\":%.2f,\"speed_kph\":%.3f,"
        "\"time_utc\":\"%s\",\"fix_age_ms\":%lld}",
        (unsigned long long)s.seq,
        s.device_present ? "true" : "false",
        s.serial_ok      ? "true" : "false",
        s.hid_ok         ? "true" : "false",
        s.gpsdo_locked   ? "true" : "false",
        s.has_fix        ? "true" : "false",
        s.fix_quality, s.satellites, s.hdop,
        s.lat, s.lon, s.alt_m, s.speed_kph,
        escape(s.time_utc).c_str(),
        (long long)age);
    if (n < 0) return "{\"class\":\"ERROR\",\"msg\":\"encode failed\"}";
    return std::string(buf, (size_t)(n < (int)sizeof buf ? n : (int)sizeof buf - 1));
}

std::string nmea_to_json(const std::string& raw, int64_t t_unix_ms, uint64_t dropped)
{
    return "{\"class\":\"NMEA\",\"raw\":\"" + escape(raw) +
           "\",\"t_unix_ms\":" + std::to_string(t_unix_ms) +
           ",\"dropped\":" + std::to_string(dropped) + "}";
}

bool json_get_string(const std::string& js, const char* key, std::string& out)
{
    size_t p = find_value(js, key);
    if (p == std::string::npos) return false;
    while (p < js.size() && (js[p] == ' ')) ++p;
    if (p >= js.size() || js[p] != '"') return false;
    ++p;
    std::string v;
    while (p < js.size() && js[p] != '"') {
        if (js[p] == '\\' && p + 1 < js.size()) ++p;
        v += js[p++];
    }
    if (p >= js.size()) return false;   // unterminated string
    out = v;
    return true;
}

bool json_get_bool(const std::string& js, const char* key, bool& out)
{
    size_t p = find_value(js, key);
    if (p == std::string::npos) return false;
    while (p < js.size() && js[p] == ' ') ++p;
    if (js.compare(p, 4, "true") == 0)  { out = true;  return true; }
    if (js.compare(p, 5, "false") == 0) { out = false; return true; }
    return false;
}

bool json_get_double(const std::string& js, const char* key, double& out)
{
    const size_t p = find_value(js, key);
    if (p == std::string::npos) return false;
    char* end = nullptr;
    const double v = std::strtod(js.c_str() + p, &end);
    if (end == js.c_str() + p) return false;
    out = v;
    return true;
}

bool json_get_int64(const std::string& js, const char* key, int64_t& out)
{
    const size_t p = find_value(js, key);
    if (p == std::string::npos) return false;
    char* end = nullptr;
    const long long v = std::strtoll(js.c_str() + p, &end, 10);
    if (end == js.c_str() + p) return false;
    out = (int64_t)v;
    return true;
}

bool snapshot_from_json(const std::string& line, Snapshot& out)
{
    std::string cls;
    if (!json_get_string(line, "class", cls) || cls != "FIX") return false;

    Snapshot s;
    int64_t i = 0;
    double  d = 0.0;

    if (json_get_int64(line, "seq", i)) s.seq = (uint64_t)i;
    json_get_bool(line, "device_present", s.device_present);
    json_get_bool(line, "serial_ok",      s.serial_ok);
    json_get_bool(line, "hid_ok",         s.hid_ok);
    json_get_bool(line, "gpsdo_locked",   s.gpsdo_locked);
    json_get_bool(line, "has_fix",        s.has_fix);
    if (json_get_int64(line, "fix_quality", i)) s.fix_quality = (int)i;
    if (json_get_int64(line, "satellites",  i)) s.satellites  = (int)i;
    if (json_get_double(line, "hdop",      d)) s.hdop      = d;
    if (json_get_double(line, "lat",       d)) s.lat       = d;
    if (json_get_double(line, "lon",       d)) s.lon       = d;
    if (json_get_double(line, "alt_m",     d)) s.alt_m     = d;
    if (json_get_double(line, "speed_kph", d)) s.speed_kph = d;
    json_get_string(line, "time_utc", s.time_utc);
    if (json_get_int64(line, "fix_age_ms", i)) s.fix_age_ms = i;

    out = s;
    return true;
}

} // namespace wbr_gps
```

- [ ] **Step 5: Add to the build and run**

Add `src/json_io.cpp` to `wbr_gps_core`, then append to `CMakeLists.txt`:

```cmake
add_executable(test_json_io tests/test_json_io.cpp)
target_include_directories(test_json_io PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_json_io PRIVATE wbr_gps_core)
add_test(NAME json_io COMMAND test_json_io)
```

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make test_json_io && ./test_json_io
```

Expected: `0 failed`, exit 0.

- [ ] **Step 6: Commit**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS
git add include/wbr_gps/json_io.hpp src/json_io.cpp tests/test_json_io.cpp CMakeLists.txt
git commit -m "feat: newline-delimited JSON wire format for GPS snapshots and NMEA"
```

---

## Task 4: State store

The single authoritative snapshot. Owns the sequence counter and decides when a fix has gone stale.

**Files:**
- Create: `WBR-GPS/include/wbr_gps/state_store.hpp`
- Create: `WBR-GPS/src/state_store.cpp`
- Create: `WBR-GPS/tests/test_state_store.cpp`
- Modify: `WBR-GPS/CMakeLists.txt`

**Interfaces:**
- Consumes: `Snapshot`, `nmea_apply`, `ParseResult`
- Produces: class `wbr_gps::StateStore` with `apply_nmea(const std::string& line, int64_t now) -> ParseResult`, `set_serial_ok(bool)`, `set_hid_ok(bool)`, `set_device_present(bool)`, `set_gpsdo_locked(bool)`, `current() const -> const Snapshot&`, `seq() const -> uint64_t`, `rejected_count() const -> uint64_t`, `mark_fix_stale_if_older_than(int64_t now, int64_t max_age_ms)`

- [ ] **Step 1: Write the failing test**

Create `WBR-GPS/tests/test_state_store.cpp`:

```cpp
#include "wbr_gps/state_store.hpp"
#include "test_util.h"

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
    st.set_serial_ok(true);
    ASSERT_TRUE(st.seq() > s0);
    // Setting the same value again must be a no-op: clients should not be
    // woken by non-events.
    const uint64_t s1 = st.seq();
    st.set_serial_ok(true);
    ASSERT_EQ(st.seq(), s1);
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

static void run_tests()
{
    test_seq_advances_only_on_change();
    test_health_flags_independent();
    test_health_change_advances_seq();
    test_staleness();
}

TEST_MAIN
```

- [ ] **Step 2: Run to verify it fails**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make test_state_store
```

Expected: `fatal error: wbr_gps/state_store.hpp: No such file or directory`.

- [ ] **Step 3: Create the header**

Create `WBR-GPS/include/wbr_gps/state_store.hpp`:

```cpp
#pragma once

#include "wbr_gps/gps_types.hpp"
#include "wbr_gps/nmea_parser.hpp"

#include <cstdint>
#include <string>

namespace wbr_gps {

// The one authoritative snapshot. No locking: the daemon is single-threaded
// on an epoll loop, which is precisely why it needs none.
class StateStore {
public:
    ParseResult apply_nmea(const std::string& line, int64_t now_mono_ms);

    void set_serial_ok(bool v);
    void set_hid_ok(bool v);
    void set_device_present(bool v);
    void set_gpsdo_locked(bool v);

    // Drop has_fix when the last fix is older than max_age_ms. The position
    // is retained so consumers can still report a last-known location.
    void mark_fix_stale_if_older_than(int64_t now_mono_ms, int64_t max_age_ms);

    const Snapshot& current() const { return snap_; }
    uint64_t        seq() const { return snap_.seq; }
    uint64_t        rejected_count() const { return rejected_; }

private:
    void bump() { ++snap_.seq; }

    Snapshot snap_;
    uint64_t rejected_ = 0;
};

} // namespace wbr_gps
```

- [ ] **Step 4: Implement**

Create `WBR-GPS/src/state_store.cpp`:

```cpp
#include "wbr_gps/state_store.hpp"

namespace wbr_gps {

ParseResult StateStore::apply_nmea(const std::string& line, int64_t now_mono_ms)
{
    const ParseResult r = nmea_apply(line, snap_, now_mono_ms);
    if (r == ParseResult::Rejected) {
        ++rejected_;
    } else if (r != ParseResult::Ignored) {
        bump();
    }
    return r;
}

// Each setter is a no-op when the value is unchanged, so clients are never
// woken by non-events.
void StateStore::set_serial_ok(bool v)
{
    if (snap_.serial_ok == v) return;
    snap_.serial_ok = v;
    bump();
}

void StateStore::set_hid_ok(bool v)
{
    if (snap_.hid_ok == v) return;
    snap_.hid_ok = v;
    bump();
}

void StateStore::set_device_present(bool v)
{
    if (snap_.device_present == v) return;
    snap_.device_present = v;
    bump();
}

void StateStore::set_gpsdo_locked(bool v)
{
    if (snap_.gpsdo_locked == v) return;
    snap_.gpsdo_locked = v;
    bump();
}

void StateStore::mark_fix_stale_if_older_than(int64_t now_mono_ms, int64_t max_age_ms)
{
    if (!snap_.has_fix) return;                 // already stale: idempotent
    if (snap_.fix_mono_ms == 0) return;         // never had a fix
    if (now_mono_ms - snap_.fix_mono_ms <= max_age_ms) return;
    snap_.has_fix = false;
    bump();
}

} // namespace wbr_gps
```

- [ ] **Step 5: Build, run, commit**

Add `src/state_store.cpp` to `wbr_gps_core` and a `test_state_store` target mirroring Task 3 Step 5.

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make test_state_store && ./test_state_store
```

Expected: `0 failed`, exit 0.

```bash
cd /home/sigint-4/prefix/src/WBR-GPS
git add include/wbr_gps/state_store.hpp src/state_store.cpp tests/test_state_store.cpp CMakeLists.txt
git commit -m "feat: authoritative GPS state store with independent health flags"
```

---

## Task 5: Serial source with pty test harness

The `openpty()` harness is what makes every later task testable without a Leo Bodnar attached.

**Files:**
- Create: `WBR-GPS/include/wbr_gps/serial_source.hpp`
- Create: `WBR-GPS/src/serial_source.cpp`
- Create: `WBR-GPS/tests/pty_harness.h`
- Create: `WBR-GPS/tests/test_serial_source.cpp`
- Modify: `WBR-GPS/CMakeLists.txt`

**Interfaces:**
- Consumes: `StateStore`
- Produces: class `wbr_gps::SerialSource` with `open_device() -> bool`, `fd() const -> int`, `on_readable(StateStore&, int64_t now, const std::function<void(const std::string&)>& on_line) -> bool`, `close_device()`, `should_retry(int64_t now) const -> bool`, `note_retry(int64_t now)`, `set_path(const std::string&)`, `path() const -> std::string`

- [ ] **Step 1: Create the pty harness**

Create `WBR-GPS/tests/pty_harness.h`:

```cpp
#pragma once

// A pseudo-terminal standing in for the Leo Bodnar serial port, so tests run
// in CI with no hardware and without root.
//
// NOTE: a pty is not a USB CDC-ACM device. Unplug/replug behaviour, TIOCEXCL
// semantics and ModemManager interaction can only be verified on real
// hardware — see the spec, section 7.6.

#include <fcntl.h>
#include <pty.h>
#include <string>
#include <unistd.h>

struct PtyPair {
    int         master = -1;
    int         slave  = -1;
    std::string slave_path;

    bool open_pair()
    {
        char name[256];
        if (openpty(&master, &slave, name, nullptr, nullptr) != 0) return false;
        slave_path = name;
        return true;
    }

    // Write as if the GPS had emitted this sentence.
    void emit(const std::string& line)
    {
        const std::string framed = line + "\r\n";
        ssize_t rc = ::write(master, framed.data(), framed.size());
        (void)rc;
    }

    // Emit raw bytes with no framing, for torn-read simulation.
    void emit_raw(const std::string& bytes)
    {
        ssize_t rc = ::write(master, bytes.data(), bytes.size());
        (void)rc;
    }

    void close_pair()
    {
        if (master >= 0) { ::close(master); master = -1; }
        if (slave  >= 0) { ::close(slave);  slave  = -1; }
    }

    ~PtyPair() { close_pair(); }
};
```

- [ ] **Step 2: Write the failing test**

Create `WBR-GPS/tests/test_serial_source.cpp`:

```cpp
#include "wbr_gps/serial_source.hpp"
#include "wbr_gps/state_store.hpp"
#include "pty_harness.h"
#include "test_util.h"

#include <poll.h>
#include <vector>

// Drain whatever the source can read right now.
static void pump(wbr_gps::SerialSource& src, wbr_gps::StateStore& st, int64_t now,
                 std::vector<std::string>* lines = nullptr)
{
    struct pollfd p { src.fd(), POLLIN, 0 };
    while (poll(&p, 1, 50) > 0 && (p.revents & POLLIN)) {
        src.on_readable(st, now, [&](const std::string& l) {
            if (lines) lines->push_back(l);
        });
        p.revents = 0;
    }
}

static void test_reads_complete_sentences()
{
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());

    wbr_gps::SerialSource src;
    src.set_path(pty.slave_path);
    ASSERT_TRUE(src.open_device());

    wbr_gps::StateStore st;
    std::vector<std::string> lines;

    pty.emit("$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F");
    pump(src, st, 1000, &lines);

    ASSERT_EQ(lines.size(), (size_t)1);
    ASSERT_TRUE(st.current().has_fix);
    ASSERT_NEAR(st.current().lat, 13.0028260, 1e-6);
}

static void test_reassembles_split_writes()
{
    // A sentence arriving in three chunks must still parse. This is the case
    // the old per-byte readers got right by accident and the GUI got wrong.
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    wbr_gps::SerialSource src;
    src.set_path(pty.slave_path);
    ASSERT_TRUE(src.open_device());
    wbr_gps::StateStore st;

    pty.emit_raw("$GNGGA,045519.50,1300.16956,N,");
    pump(src, st, 1000);
    ASSERT_FALSE(st.current().has_fix);          // incomplete: nothing applied

    pty.emit_raw("07740.79521,E,1,04,1.33,921.8,");
    pump(src, st, 1000);
    ASSERT_FALSE(st.current().has_fix);

    pty.emit_raw("M,-86.3,M,,*6F\r\n");
    pump(src, st, 1000);
    ASSERT_TRUE(st.current().has_fix);
}

static void test_oversized_line_is_discarded()
{
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    wbr_gps::SerialSource src;
    src.set_path(pty.slave_path);
    ASSERT_TRUE(src.open_device());
    wbr_gps::StateStore st;

    // 4 KB with no newline must not grow the buffer without bound.
    pty.emit_raw(std::string(4096, 'A'));
    pump(src, st, 1000);
    // Then a good sentence must still parse: the buffer recovered.
    pty.emit("$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F");
    pump(src, st, 1000);
    ASSERT_TRUE(st.current().has_fix);
}

static void test_torn_input_rejected_and_counted()
{
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    wbr_gps::SerialSource src;
    src.set_path(pty.slave_path);
    ASSERT_TRUE(src.open_device());
    wbr_gps::StateStore st;

    pty.emit(",,3,,5$G32,,,9151,,2,96*G3,,1,,,8,,6379$M055A3.5N7.500,*");
    pump(src, st, 1000);
    ASSERT_FALSE(st.current().has_fix);
    ASSERT_EQ(st.rejected_count(), (uint64_t)1);
}

static void test_open_failure_and_backoff()
{
    wbr_gps::SerialSource src;
    src.set_path("/dev/definitely-not-a-device");
    ASSERT_FALSE(src.open_device());
    ASSERT_EQ(src.fd(), -1);

    // Backoff must start short and grow, and must be capped at 5 s.
    src.note_retry(0);
    ASSERT_FALSE(src.should_retry(100));
    ASSERT_TRUE(src.should_retry(1000));
    for (int i = 0; i < 20; ++i) src.note_retry(i * 10000);
    ASSERT_TRUE(src.should_retry(200000 + 5001));
}

static void run_tests()
{
    test_reads_complete_sentences();
    test_reassembles_split_writes();
    test_oversized_line_is_discarded();
    test_torn_input_rejected_and_counted();
    test_open_failure_and_backoff();
}

TEST_MAIN
```

- [ ] **Step 3: Run to verify it fails**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make test_serial_source
```

Expected: `fatal error: wbr_gps/serial_source.hpp: No such file or directory`.

- [ ] **Step 4: Create the header**

Create `WBR-GPS/include/wbr_gps/serial_source.hpp`:

```cpp
#pragma once

#include "wbr_gps/state_store.hpp"

#include <cstdint>
#include <functional>
#include <string>

namespace wbr_gps {

// Owns the GPS serial port. Exclusively: the fd is opened with TIOCEXCL so a
// stray reader gets EBUSY rather than silently stealing bytes from our queue.
class SerialSource {
public:
    void        set_path(const std::string& p) { path_ = p; }
    std::string path() const { return path_; }
    void        set_baud(int b) { baud_ = b; }

    // Open and configure. Returns false on any failure; the caller then uses
    // should_retry()/note_retry() to back off.
    bool open_device();
    void close_device();
    int  fd() const { return fd_; }

    // Read whatever is available, assemble lines, apply them to `store`, and
    // invoke `on_line` for each complete line (for raw NMEA subscribers).
    // Returns false if the device went away and was closed.
    bool on_readable(StateStore& store, int64_t now_mono_ms,
                     const std::function<void(const std::string&)>& on_line);

    bool should_retry(int64_t now_mono_ms) const;
    void note_retry(int64_t now_mono_ms);

private:
    std::string path_ = kDefaultSerialPath;
    int         baud_ = kDefaultBaud;
    int         fd_   = -1;
    std::string buf_;

    int64_t next_retry_ms_ = 0;
    int64_t backoff_ms_    = 250;   // 250 ms, doubling, capped at 5 s
};

} // namespace wbr_gps
```

- [ ] **Step 5: Implement**

Create `WBR-GPS/src/serial_source.cpp`:

```cpp
#include "wbr_gps/serial_source.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace wbr_gps {

namespace {

constexpr size_t  kMaxBuf      = 1024;   // discard beyond this: torn input
constexpr int64_t kBackoffMin  = 250;
constexpr int64_t kBackoffMax  = 5000;

speed_t baud_const(int baud)
{
    switch (baud) {
        case   4800: return B4800;
        case   9600: return B9600;
        case  19200: return B19200;
        case  38400: return B38400;
        case  57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:     return B115200;
    }
}

bool configure(int fd, int baud)
{
    struct termios tio {};
    if (tcgetattr(fd, &tio) != 0) return false;
    cfmakeraw(&tio);
    cfsetispeed(&tio, baud_const(baud));
    cfsetospeed(&tio, baud_const(baud));
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;   // pure non-blocking: epoll decides when to read
    return tcsetattr(fd, TCSANOW, &tio) == 0;
}

} // namespace

bool SerialSource::open_device()
{
    fd_ = ::open(path_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return false;
    }

    // THE critical call. TIOCEXCL makes further open() attempts fail with
    // EBUSY, so a stray reader gets a loud error instead of silently stealing
    // bytes out of the shared tty input queue. This is guarantee G1.
    if (::ioctl(fd_, TIOCEXCL) != 0) {
        std::fprintf(stderr, "[wbr-gpsd] TIOCEXCL failed on %s: %s\n",
                     path_.c_str(), std::strerror(errno));
        // Not fatal on a pty, which does not support it; fatal in spirit on
        // real hardware, where the caller logs and continues.
    }

    if (!configure(fd_, baud_)) {
        std::fprintf(stderr, "[wbr-gpsd] configure %s failed: %s\n",
                     path_.c_str(), std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    buf_.clear();
    backoff_ms_ = kBackoffMin;
    return true;
}

void SerialSource::close_device()
{
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    buf_.clear();
}

bool SerialSource::on_readable(StateStore& store, int64_t now_mono_ms,
                               const std::function<void(const std::string&)>& on_line)
{
    char io[512];
    for (;;) {
        const ssize_t n = ::read(fd_, io, sizeof io);
        if (n > 0) {
            for (ssize_t i = 0; i < n; ++i) {
                const char c = io[i];
                if (c == '\r') continue;
                if (c == '\n') {
                    if (!buf_.empty()) {
                        store.apply_nmea(buf_, now_mono_ms);
                        on_line(buf_);
                        buf_.clear();
                    }
                } else {
                    buf_ += c;
                    // A line this long is a torn or concatenated read. Drop it
                    // rather than letting the buffer grow without bound.
                    if (buf_.size() > kMaxBuf) buf_.clear();
                }
            }
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return true;                     // drained
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        // n == 0 (EOF) or a real error: the device went away.
        close_device();
        return false;
    }
}

bool SerialSource::should_retry(int64_t now_mono_ms) const
{
    return now_mono_ms >= next_retry_ms_;
}

void SerialSource::note_retry(int64_t now_mono_ms)
{
    next_retry_ms_ = now_mono_ms + backoff_ms_;
    backoff_ms_ *= 2;
    if (backoff_ms_ > kBackoffMax) backoff_ms_ = kBackoffMax;
}

} // namespace wbr_gps
```

- [ ] **Step 6: Build, run, commit**

Add `src/serial_source.cpp` to `wbr_gps_core` and a `test_serial_source` target. The harness needs `libutil`:

```cmake
add_executable(test_serial_source tests/test_serial_source.cpp)
target_include_directories(test_serial_source PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_serial_source PRIVATE wbr_gps_core util)
add_test(NAME serial_source COMMAND test_serial_source)
```

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make test_serial_source && ./test_serial_source
```

Expected: `0 failed`, exit 0.

```bash
cd /home/sigint-4/prefix/src/WBR-GPS
git add include/wbr_gps/serial_source.hpp src/serial_source.cpp tests/pty_harness.h tests/test_serial_source.cpp CMakeLists.txt
git commit -m "feat: exclusive serial source with TIOCEXCL and pty test harness"
```

---

## Task 6: HID lock source and device presence

**Files:**
- Create: `WBR-GPS/include/wbr_gps/hid_source.hpp`
- Create: `WBR-GPS/src/hid_source.cpp`
- Create: `WBR-GPS/include/wbr_gps/device_presence.hpp`
- Create: `WBR-GPS/src/device_presence.cpp`
- Create: `WBR-GPS/tests/test_device_presence.cpp`
- Modify: `WBR-GPS/CMakeLists.txt`

**Interfaces:**
- Consumes: `StateStore`
- Produces: class `wbr_gps::HidSource` with `set_path(const std::string&)`, `open_device() -> bool`, `close_device()`, `fd() const -> int`, `on_readable(StateStore&) -> bool`, `should_retry(int64_t) const -> bool`, `note_retry(int64_t)`, `static discover() -> std::string`; and free functions `wbr_gps::find_leo_bodnar_serial() -> std::string`, `wbr_gps::leo_bodnar_present() -> bool`, `wbr_gps::scan_by_id_for(const char* dir, const char* needle) -> std::string`

- [ ] **Step 1: Write the failing test**

Create `WBR-GPS/tests/test_device_presence.cpp`. Presence scanning is tested against a temporary directory rather than the real `/dev`, so it runs anywhere:

```cpp
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
```

- [ ] **Step 2: Run to verify it fails**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make test_device_presence
```

Expected: `fatal error: wbr_gps/device_presence.hpp: No such file or directory`.

- [ ] **Step 3: Create the presence header and implementation**

Create `WBR-GPS/include/wbr_gps/device_presence.hpp`:

```cpp
#pragma once

#include <string>

namespace wbr_gps {

// Case-insensitive scan of `dir` for an entry containing `needle`. Returns a
// full path, or "" if nothing matched or the directory is unreadable.
std::string scan_by_id_for(const char* dir, const char* needle);

// Full path to the Leo Bodnar serial device via /dev/serial/by-id, or "".
std::string find_leo_bodnar_serial();

// Is a Leo Bodnar attached to USB at all? This is the ref-present signal that
// replaces leo_bodnar_usb_present() in WBR-GSM/gps_source.h, used by
// gsm_monitor.cpp:772 to auto-select the SDR's 10 MHz external reference.
bool leo_bodnar_present();

} // namespace wbr_gps
```

Create `WBR-GPS/src/device_presence.cpp`:

```cpp
#include "wbr_gps/device_presence.hpp"

#include <algorithm>
#include <cctype>
#include <dirent.h>
#include <string>

namespace wbr_gps {

namespace {
constexpr const char* kByIdDir = "/dev/serial/by-id";

std::string upper(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return (char)std::toupper(c); });
    return s;
}
} // namespace

std::string scan_by_id_for(const char* dir, const char* needle)
{
    DIR* d = ::opendir(dir);
    if (!d) return "";

    const std::string want = upper(needle);
    std::string result;

    struct dirent* ent;
    while ((ent = ::readdir(d)) != nullptr) {
        const std::string name = ent->d_name;
        if (name == "." || name == "..") continue;
        if (upper(name).find(want) != std::string::npos) {
            result = std::string(dir) + "/" + name;
            break;
        }
    }
    ::closedir(d);
    return result;
}

std::string find_leo_bodnar_serial()
{
    return scan_by_id_for(kByIdDir, "Leo_Bodnar");
}

bool leo_bodnar_present()
{
    return !find_leo_bodnar_serial().empty();
}

} // namespace wbr_gps
```

- [ ] **Step 4: Create the HID source**

Create `WBR-GPS/include/wbr_gps/hid_source.hpp`:

```cpp
#pragma once

#include "wbr_gps/state_store.hpp"

#include <cstdint>
#include <string>

namespace wbr_gps {

// Owns the Leo Bodnar hidraw device. The 2-byte status report's byte 1 bit 0
// is the GPSDO lock bit (0 means locked). This is a genuinely different
// signal from NMEA fix: the clock can be locked with no current GGA.
class HidSource {
public:
    void        set_path(const std::string& p) { path_ = p; }
    std::string path() const { return path_; }

    // Locate a Leo Bodnar hidraw node by walking /sys/class/hidraw/*/device/uevent
    // for vendor 1DD2. Returns "" if none. Avoids shelling out to udevadm, which
    // is what makes the current GUI probe slow enough to need a cache.
    static std::string discover();

    bool open_device();
    void close_device();
    int  fd() const { return fd_; }

    // Read one report and update the lock bit. Returns false if the device
    // went away and was closed.
    bool on_readable(StateStore& store);

    bool should_retry(int64_t now_mono_ms) const;
    void note_retry(int64_t now_mono_ms);

private:
    std::string path_;
    int         fd_ = -1;
    int64_t     next_retry_ms_ = 0;
    int64_t     backoff_ms_    = 250;
};

} // namespace wbr_gps
```

Create `WBR-GPS/src/hid_source.cpp`:

```cpp
#include "wbr_gps/hid_source.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

namespace wbr_gps {

namespace {
constexpr int64_t kBackoffMin = 250;
constexpr int64_t kBackoffMax = 5000;
constexpr const char* kHidrawClassDir = "/sys/class/hidraw";
constexpr const char* kLeoVendorId = "1DD2";
} // namespace

std::string HidSource::discover()
{
    DIR* d = ::opendir(kHidrawClassDir);
    if (!d) return "";

    std::string result;
    struct dirent* ent;
    while ((ent = ::readdir(d)) != nullptr) {
        const std::string name = ent->d_name;
        if (name.rfind("hidraw", 0) != 0) continue;

        const std::string uevent =
            std::string(kHidrawClassDir) + "/" + name + "/device/uevent";
        FILE* f = std::fopen(uevent.c_str(), "r");
        if (!f) continue;

        char line[256];
        bool match = false;
        while (std::fgets(line, sizeof line, f)) {
            // HID_ID=0003:00001DD2:00002444
            if (std::strstr(line, kLeoVendorId) != nullptr) { match = true; break; }
        }
        std::fclose(f);

        if (match) { result = "/dev/" + name; break; }
    }
    ::closedir(d);
    return result;
}

bool HidSource::open_device()
{
    if (path_.empty()) path_ = discover();
    if (path_.empty()) return false;

    fd_ = ::open(path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) return false;
    backoff_ms_ = kBackoffMin;
    return true;
}

void HidSource::close_device()
{
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool HidSource::on_readable(StateStore& store)
{
    unsigned char rep[8] = { 0 };
    for (;;) {
        const ssize_t n = ::read(fd_, rep, sizeof rep);
        if (n >= 2) {
            // Bit 0 clear means locked, matching the existing WBR-SA reader.
            store.set_gpsdo_locked((rep[1] & 0x01) == 0);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return true;
        if (n < 0 && errno == EINTR) continue;
        if (n >= 0 && n < 2) return true;   // short report: ignore, keep fd
        close_device();
        return false;
    }
}

bool HidSource::should_retry(int64_t now_mono_ms) const
{
    return now_mono_ms >= next_retry_ms_;
}

void HidSource::note_retry(int64_t now_mono_ms)
{
    next_retry_ms_ = now_mono_ms + backoff_ms_;
    backoff_ms_ *= 2;
    if (backoff_ms_ > kBackoffMax) backoff_ms_ = kBackoffMax;
}

} // namespace wbr_gps
```

- [ ] **Step 5: Build, run, commit**

Add `src/device_presence.cpp` and `src/hid_source.cpp` to `wbr_gps_core`, add the `test_device_presence` target.

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make test_device_presence && ./test_device_presence
```

Expected: `0 failed`, exit 0.

Verify `discover()` against the real hardware on this box (it should print `/dev/hidraw2`):

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cat > /tmp/hid_probe.cpp <<'EOF'
#include "wbr_gps/hid_source.hpp"
#include <cstdio>
int main() { printf("%s\n", wbr_gps::HidSource::discover().c_str()); }
EOF
g++ -std=c++17 -I../include /tmp/hid_probe.cpp -o /tmp/hid_probe && /tmp/hid_probe
```

Expected: `/dev/hidraw2`.

```bash
cd /home/sigint-4/prefix/src/WBR-GPS
git add include/wbr_gps/hid_source.hpp include/wbr_gps/device_presence.hpp src/hid_source.cpp src/device_presence.cpp tests/test_device_presence.cpp CMakeLists.txt
git commit -m "feat: HID lock source and USB presence detection without udevadm"
```

---

## Task 7: Unix socket server with singleton lock and backpressure

The heart of the fan-out. Guarantees G2 and G3 live here.

**Files:**
- Create: `WBR-GPS/include/wbr_gps/server.hpp`
- Create: `WBR-GPS/src/server.cpp`
- Create: `WBR-GPS/tests/test_server.cpp`
- Modify: `WBR-GPS/CMakeLists.txt`

**Interfaces:**
- Consumes: `StateStore`, `snapshot_to_json`, `nmea_to_json`, `json_get_bool`, `json_get_string`
- Produces: class `wbr_gps::Server` with `acquire_singleton(const std::string& lock_path) -> bool`, `listen_on(const std::string& sock_path, const char* group, mode_t) -> bool`, `listen_fd() const -> int`, `accept_client() -> int`, `on_client_readable(int fd, StateStore&, int64_t now) -> bool`, `broadcast_fix(const StateStore&, int64_t now)`, `broadcast_nmea(const std::string& raw, int64_t t_unix_ms)`, `flush_client(int fd) -> bool`, `drop_client(int fd)`, `client_count() const -> size_t`, `pending_bytes(int fd) const -> size_t`, `wants_write(int fd) const -> bool`, `shutdown()`

- [ ] **Step 1: Write the failing test**

Create `WBR-GPS/tests/test_server.cpp`. Uses a socket in a temp dir with no group override, so it runs unprivileged:

```cpp
#include "wbr_gps/server.hpp"
#include "wbr_gps/json_io.hpp"
#include "test_util.h"

#include <cstdlib>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

static std::string tmp_sock()
{
    char tmpl[] = "/tmp/wbrgps_sock_XXXXXX";
    const char* d = mkdtemp(tmpl);
    return std::string(d) + "/gpsd.sock";
}

static int connect_client(const std::string& path)
{
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof addr.sun_path, "%s", path.c_str());
    if (::connect(fd, (struct sockaddr*)&addr, sizeof addr) != 0) { ::close(fd); return -1; }
    return fd;
}

static std::string read_line(int fd)
{
    std::string out;
    char c;
    while (::read(fd, &c, 1) == 1) {
        if (c == '\n') break;
        out += c;
    }
    return out;
}

static void send_line(int fd, const std::string& s)
{
    const std::string framed = s + "\n";
    ssize_t rc = ::write(fd, framed.data(), framed.size());
    (void)rc;
}

static void test_hello_on_connect()
{
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));

    const int c = connect_client(path);
    ASSERT_TRUE(c >= 0);
    const int s = srv.accept_client();
    ASSERT_TRUE(s >= 0);
    ASSERT_TRUE(srv.flush_client(s));

    const std::string hello = read_line(c);
    std::string cls;
    ASSERT_TRUE(wbr_gps::json_get_string(hello, "class", cls));
    ASSERT_STREQ(cls, "HELLO");
    ASSERT_TRUE(hello.find("\"proto\":1") != std::string::npos);

    ::close(c);
    srv.shutdown();
}

static void test_get_returns_one_snapshot()
{
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));

    wbr_gps::StateStore st;
    st.apply_nmea("$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F", 1000);

    const int c = connect_client(path);
    const int s = srv.accept_client();
    srv.flush_client(s);
    read_line(c);                       // HELLO

    send_line(c, "{\"op\":\"get\"}");
    ASSERT_TRUE(srv.on_client_readable(s, st, 1480));
    srv.flush_client(s);

    wbr_gps::Snapshot got;
    ASSERT_TRUE(wbr_gps::snapshot_from_json(read_line(c), got));
    ASSERT_TRUE(got.has_fix);
    ASSERT_NEAR(got.lat, 13.0028260, 1e-6);
    ASSERT_EQ(got.fix_age_ms, (int64_t)480);

    ::close(c);
    srv.shutdown();
}

static void test_get_client_receives_no_pushes()
{
    // A client that only sent "get" must not be subscribed. Otherwise the
    // Django badge, which connects and closes, would accumulate pushes.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    srv.flush_client(s);
    read_line(c);                       // HELLO

    send_line(c, "{\"op\":\"get\"}");
    srv.on_client_readable(s, st, 1000);
    srv.flush_client(s);
    read_line(c);                       // the one FIX

    srv.broadcast_fix(st, 2000);
    ASSERT_EQ(srv.pending_bytes(s), (size_t)0);

    ::close(c);
    srv.shutdown();
}

static void test_watch_receives_pushes()
{
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    srv.flush_client(s);
    read_line(c);                       // HELLO

    send_line(c, "{\"op\":\"watch\",\"fix\":true,\"nmea\":false}");
    srv.on_client_readable(s, st, 1000);
    srv.flush_client(s);
    read_line(c);                       // immediate snapshot on subscribe (S5)

    st.apply_nmea("$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F", 2000);
    srv.broadcast_fix(st, 2000);
    srv.flush_client(s);

    wbr_gps::Snapshot got;
    ASSERT_TRUE(wbr_gps::snapshot_from_json(read_line(c), got));
    ASSERT_TRUE(got.has_fix);

    ::close(c);
    srv.shutdown();
}

static void test_nmea_subscription_is_opt_in()
{
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    srv.flush_client(s);
    read_line(c);

    send_line(c, "{\"op\":\"watch\",\"fix\":false,\"nmea\":false}");
    srv.on_client_readable(s, st, 1000);
    srv.flush_client(s);
    read_line(c);                       // immediate snapshot

    srv.broadcast_nmea("$GNGGA,1,2*6F", 1000);
    ASSERT_EQ(srv.pending_bytes(s), (size_t)0);

    ::close(c);
    srv.shutdown();
}

static void test_slow_client_coalesces_and_does_not_grow(void)
{
    // S6. The client never reads. FIX messages must coalesce so the queue
    // stops growing, and the daemon must never block.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    send_line(c, "{\"op\":\"watch\",\"fix\":true,\"nmea\":false}");
    srv.on_client_readable(s, st, 1000);

    // Fill the socket buffer and then some, without ever reading from `c`.
    for (int i = 0; i < 20000; ++i) {
        st.set_serial_ok(i % 2 == 0);
        srv.broadcast_fix(st, 1000 + i);
        srv.flush_client(s);
    }

    // The queue must be bounded. One coalesced FIX is under 1 KB.
    ASSERT_TRUE(srv.pending_bytes(s) < 4096);
    // And the client must still be connected: coalescing, not disconnection.
    ASSERT_EQ(srv.client_count(), (size_t)1);

    ::close(c);
    srv.shutdown();
}

static void test_nmea_overflow_drops_and_counts()
{
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    send_line(c, "{\"op\":\"watch\",\"fix\":false,\"nmea\":true}");
    srv.on_client_readable(s, st, 1000);

    for (int i = 0; i < 20000; ++i) {
        srv.broadcast_nmea("$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,,*6F", i);
        srv.flush_client(s);
    }

    // Bounded queue, client retained.
    ASSERT_TRUE(srv.pending_bytes(s) <= wbr_gps::Server::kMaxQueueBytes);
    ASSERT_EQ(srv.client_count(), (size_t)1);

    ::close(c);
    srv.shutdown();
}

static void test_abrupt_disconnect_is_cleaned_up()
{
    // S7. Killing the peer must free the client slot, not leak it.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    ASSERT_EQ(srv.client_count(), (size_t)1);

    ::close(c);                          // peer vanishes
    // A read now reports EOF; the server must drop the client.
    ASSERT_FALSE(srv.on_client_readable(s, st, 1000));
    srv.drop_client(s);
    ASSERT_EQ(srv.client_count(), (size_t)0);

    srv.shutdown();
}

static void test_malformed_input_gets_error_not_crash()
{
    // S10.
    const std::string path = tmp_sock();
    wbr_gps::Server srv;
    ASSERT_TRUE(srv.listen_on(path, nullptr, 0600));
    wbr_gps::StateStore st;

    const int c = connect_client(path);
    const int s = srv.accept_client();
    srv.flush_client(s);
    read_line(c);                        // HELLO

    send_line(c, "this is not json");
    ASSERT_TRUE(srv.on_client_readable(s, st, 1000));
    srv.flush_client(s);
    std::string cls;
    ASSERT_TRUE(wbr_gps::json_get_string(read_line(c), "class", cls));
    ASSERT_STREQ(cls, "ERROR");

    // An oversized line must be refused, and the client dropped rather than
    // allowed to consume unbounded memory.
    send_line(c, std::string(wbr_gps::Server::kMaxRequestBytes + 100, 'A'));
    srv.on_client_readable(s, st, 1000);

    ::close(c);
    srv.shutdown();
}

static void test_singleton_lock()
{
    // S14. Two daemons must not both run.
    char tmpl[] = "/tmp/wbrgps_lock_XXXXXX";
    const std::string dir = mkdtemp(tmpl);
    const std::string lock = dir + "/wbr-gpsd.pid";

    wbr_gps::Server a, b;
    ASSERT_TRUE(a.acquire_singleton(lock));
    ASSERT_FALSE(b.acquire_singleton(lock));   // loser
    a.shutdown();
    // After the winner exits, the lock must be reusable.
    wbr_gps::Server c;
    ASSERT_TRUE(c.acquire_singleton(lock));
    c.shutdown();
}

static void test_stale_socket_is_replaced()
{
    // S15. A leftover socket file from an unclean kill must not block startup,
    // but a live daemon's socket must be respected.
    const std::string path = tmp_sock();
    {
        wbr_gps::Server first;
        ASSERT_TRUE(first.listen_on(path, nullptr, 0600));
        // While `first` is alive, a second bind must refuse.
        wbr_gps::Server second;
        ASSERT_FALSE(second.listen_on(path, nullptr, 0600));
        first.shutdown();
    }
    // The file may remain; a fresh server must reclaim it.
    wbr_gps::Server third;
    ASSERT_TRUE(third.listen_on(path, nullptr, 0600));
    third.shutdown();
}

static void run_tests()
{
    test_hello_on_connect();
    test_get_returns_one_snapshot();
    test_get_client_receives_no_pushes();
    test_watch_receives_pushes();
    test_nmea_subscription_is_opt_in();
    test_slow_client_coalesces_and_does_not_grow();
    test_nmea_overflow_drops_and_counts();
    test_abrupt_disconnect_is_cleaned_up();
    test_malformed_input_gets_error_not_crash();
    test_singleton_lock();
    test_stale_socket_is_replaced();
}

TEST_MAIN
```

- [ ] **Step 2: Run to verify it fails**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make test_server
```

Expected: `fatal error: wbr_gps/server.hpp: No such file or directory`.

- [ ] **Step 3: Create the header**

Create `WBR-GPS/include/wbr_gps/server.hpp`:

```cpp
#pragma once

#include "wbr_gps/state_store.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <sys/types.h>

namespace wbr_gps {

// Unix socket server. Single-threaded: every method is called from the one
// epoll loop in main.cpp, which is why there is no locking anywhere.
class Server {
public:
    // A queue larger than this means the client is not keeping up. FIX
    // messages coalesce; NMEA messages drop oldest.
    static constexpr size_t kMaxQueueBytes   = 64 * 1024;
    // Longest request line we will buffer from a client before dropping it.
    static constexpr size_t kMaxRequestBytes = 4096;

    ~Server();

    // flock() a pidfile so a second daemon cannot start. Returns false if
    // another instance holds it.
    bool acquire_singleton(const std::string& lock_path);

    // Bind and listen. If `group` is non-null, chown the socket to it. A
    // stale socket file is only unlinked when nothing is listening on it.
    bool listen_on(const std::string& sock_path, const char* group, mode_t mode);

    int  listen_fd() const { return listen_fd_; }
    int  accept_client();
    void drop_client(int fd);
    size_t client_count() const { return clients_.size(); }

    // Read and handle requests. Returns false when the peer has gone away.
    bool on_client_readable(int fd, StateStore& store, int64_t now_mono_ms);

    // Enqueue to every subscriber. Never blocks.
    void broadcast_fix(const StateStore& store, int64_t now_mono_ms);
    void broadcast_nmea(const std::string& raw, int64_t t_unix_ms);

    // Try to drain one client's queue. Returns false if the peer is gone.
    bool   flush_client(int fd);
    bool   wants_write(int fd) const;
    size_t pending_bytes(int fd) const;

    void shutdown();

private:
    struct Client {
        std::string in;         // partial request line
        std::string out;        // pending output bytes
        bool        want_fix  = false;
        bool        want_nmea = false;
        // Index into `out` where the last queued FIX begins, so a newer FIX
        // can replace it instead of queueing behind it. npos = none pending.
        size_t      fix_at  = std::string::npos;
        uint64_t    dropped = 0;
    };

    void enqueue(int fd, Client& c, const std::string& line);
    void handle_request(int fd, Client& c, const std::string& line,
                        StateStore& store, int64_t now_mono_ms);

    int         listen_fd_ = -1;
    int         lock_fd_   = -1;
    std::string sock_path_;
    std::string lock_path_;
    std::map<int, Client> clients_;
};

} // namespace wbr_gps
```

- [ ] **Step 4: Implement**

Create `WBR-GPS/src/server.cpp`:

```cpp
#include "wbr_gps/server.hpp"
#include "wbr_gps/json_io.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <grp.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace wbr_gps {

namespace {

bool set_nonblock(int fd)
{
    const int fl = ::fcntl(fd, F_GETFL, 0);
    return fl >= 0 && ::fcntl(fd, F_SETFL, fl | O_NONBLOCK) == 0;
}

// Is something already listening on this socket path? Used to distinguish a
// stale file from a live daemon: unlinking a live daemon's socket would let
// two daemons run, which is exactly what we must prevent.
bool socket_is_live(const std::string& path)
{
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return false;
    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof addr.sun_path, "%s", path.c_str());
    const bool live = ::connect(fd, (struct sockaddr*)&addr, sizeof addr) == 0;
    ::close(fd);
    return live;
}

} // namespace

Server::~Server() { shutdown(); }

bool Server::acquire_singleton(const std::string& lock_path)
{
    lock_fd_ = ::open(lock_path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (lock_fd_ < 0) return false;
    if (::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
        ::close(lock_fd_);
        lock_fd_ = -1;
        return false;
    }
    lock_path_ = lock_path;
    char pid[32];
    const int n = std::snprintf(pid, sizeof pid, "%d\n", (int)::getpid());
    if (::ftruncate(lock_fd_, 0) != 0) { /* best effort */ }
    ssize_t rc = ::write(lock_fd_, pid, (size_t)n);
    (void)rc;
    return true;
}

bool Server::listen_on(const std::string& sock_path, const char* group, mode_t mode)
{
    if (socket_is_live(sock_path)) {
        std::fprintf(stderr, "[wbr-gpsd] another daemon is listening on %s\n",
                     sock_path.c_str());
        return false;
    }
    ::unlink(sock_path.c_str());     // safe: nothing is listening

    listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd_ < 0) return false;
    if (!set_nonblock(listen_fd_)) { ::close(listen_fd_); listen_fd_ = -1; return false; }

    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof addr.sun_path, "%s", sock_path.c_str());

    if (::bind(listen_fd_, (struct sockaddr*)&addr, sizeof addr) != 0 ||
        ::listen(listen_fd_, 64) != 0) {
        std::fprintf(stderr, "[wbr-gpsd] bind/listen %s failed: %s\n",
                     sock_path.c_str(), std::strerror(errno));
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    if (::chmod(sock_path.c_str(), mode) != 0) {
        std::fprintf(stderr, "[wbr-gpsd] chmod %s: %s\n",
                     sock_path.c_str(), std::strerror(errno));
    }
    if (group != nullptr) {
        const struct group* g = ::getgrnam(group);
        if (g && ::chown(sock_path.c_str(), (uid_t)-1, g->gr_gid) != 0) {
            std::fprintf(stderr, "[wbr-gpsd] chown %s to group %s: %s\n",
                         sock_path.c_str(), group, std::strerror(errno));
        }
    }

    sock_path_ = sock_path;
    return true;
}

int Server::accept_client()
{
    const int fd = ::accept(listen_fd_, nullptr, nullptr);
    if (fd < 0) return -1;
    if (!set_nonblock(fd)) { ::close(fd); return -1; }

    Client c;
    clients_[fd] = c;

    char hello[128];
    std::snprintf(hello, sizeof hello,
                  "{\"class\":\"HELLO\",\"proto\":%d,\"daemon\":\"wbr-gpsd/1.0\"}",
                  kProtoVersion);
    enqueue(fd, clients_[fd], hello);
    return fd;
}

void Server::drop_client(int fd)
{
    auto it = clients_.find(fd);
    if (it == clients_.end()) return;
    clients_.erase(it);
    ::close(fd);
}

void Server::enqueue(int fd, Client& c, const std::string& line)
{
    (void)fd;
    if (c.out.size() + line.size() + 1 > kMaxQueueBytes) {
        // Hard cap. Drop the oldest whole line rather than blocking or
        // growing without bound. This is guarantee G2.
        const size_t nl = c.out.find('\n');
        if (nl == std::string::npos) {
            c.out.clear();
            c.fix_at = std::string::npos;
        } else {
            c.out.erase(0, nl + 1);
            if (c.fix_at != std::string::npos) {
                c.fix_at = (c.fix_at > nl + 1) ? c.fix_at - (nl + 1) : std::string::npos;
            }
        }
        ++c.dropped;
    }
    c.out += line;
    c.out += '\n';
}

void Server::broadcast_fix(const StateStore& store, int64_t now_mono_ms)
{
    const std::string line = snapshot_to_json(store.current(), now_mono_ms);
    for (auto& kv : clients_) {
        Client& c = kv.second;
        if (!c.want_fix) continue;

        // Coalesce: if an unsent FIX is already queued, replace it. Only the
        // newest snapshot has meaning, so a slow client's queue cannot grow.
        if (c.fix_at != std::string::npos && c.fix_at < c.out.size()) {
            const size_t end = c.out.find('\n', c.fix_at);
            if (end != std::string::npos) {
                c.out.replace(c.fix_at, end - c.fix_at, line);
                continue;
            }
        }
        c.fix_at = c.out.size();
        enqueue(kv.first, c, line);
    }
}

void Server::broadcast_nmea(const std::string& raw, int64_t t_unix_ms)
{
    for (auto& kv : clients_) {
        Client& c = kv.second;
        if (!c.want_nmea) continue;
        // NMEA cannot coalesce: every sentence is distinct data. Oldest is
        // dropped by enqueue() and reported via the dropped counter.
        enqueue(kv.first, c, nmea_to_json(raw, t_unix_ms, c.dropped));
    }
}

void Server::handle_request(int fd, Client& c, const std::string& line,
                            StateStore& store, int64_t now_mono_ms)
{
    std::string op;
    if (!json_get_string(line, "op", op)) {
        enqueue(fd, c, "{\"class\":\"ERROR\",\"msg\":\"missing op\"}");
        return;
    }

    if (op == "get") {
        // Deliberately does NOT subscribe. The Django badge connects, asks
        // once and closes; subscribing it would queue pushes for nobody.
        enqueue(fd, c, snapshot_to_json(store.current(), now_mono_ms));
        return;
    }
    if (op == "watch") {
        bool want_fix = true, want_nmea = false;
        json_get_bool(line, "fix", want_fix);
        json_get_bool(line, "nmea", want_nmea);
        c.want_fix  = want_fix;
        c.want_nmea = want_nmea;
        // Send the current snapshot immediately (scenario S5), so a client
        // joining mid-stream does not wait up to a second for the next GGA.
        c.fix_at = c.out.size();
        enqueue(fd, c, snapshot_to_json(store.current(), now_mono_ms));
        return;
    }
    if (op == "unwatch") {
        c.want_fix  = false;
        c.want_nmea = false;
        return;
    }
    enqueue(fd, c, "{\"class\":\"ERROR\",\"msg\":\"unknown op\"}");
}

bool Server::on_client_readable(int fd, StateStore& store, int64_t now_mono_ms)
{
    auto it = clients_.find(fd);
    if (it == clients_.end()) return false;
    Client& c = it->second;

    char io[1024];
    for (;;) {
        const ssize_t n = ::read(fd, io, sizeof io);
        if (n > 0) {
            c.in.append(io, (size_t)n);
            if (c.in.size() > kMaxRequestBytes) {
                // Refuse to buffer unbounded input from a client.
                enqueue(fd, c, "{\"class\":\"ERROR\",\"msg\":\"request too large\"}");
                return false;
            }
            size_t nl;
            while ((nl = c.in.find('\n')) != std::string::npos) {
                const std::string line = c.in.substr(0, nl);
                c.in.erase(0, nl + 1);
                if (!line.empty()) handle_request(fd, c, line, store, now_mono_ms);
            }
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return true;
        if (n < 0 && errno == EINTR) continue;
        return false;    // EOF or hard error: peer is gone
    }
}

bool Server::flush_client(int fd)
{
    auto it = clients_.find(fd);
    if (it == clients_.end()) return false;
    Client& c = it->second;

    while (!c.out.empty()) {
        // MSG_NOSIGNAL: a disconnecting client must never kill the daemon.
        const ssize_t n = ::send(fd, c.out.data(), c.out.size(), MSG_NOSIGNAL);
        if (n > 0) {
            c.out.erase(0, (size_t)n);
            if (c.fix_at != std::string::npos) {
                c.fix_at = (c.fix_at > (size_t)n) ? c.fix_at - (size_t)n
                                                  : std::string::npos;
            }
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return true;
        if (n < 0 && errno == EINTR) continue;
        return false;    // EPIPE or similar: peer is gone
    }
    c.fix_at = std::string::npos;
    return true;
}

bool Server::wants_write(int fd) const
{
    const auto it = clients_.find(fd);
    return it != clients_.end() && !it->second.out.empty();
}

size_t Server::pending_bytes(int fd) const
{
    const auto it = clients_.find(fd);
    return it == clients_.end() ? 0 : it->second.out.size();
}

void Server::shutdown()
{
    for (auto& kv : clients_) ::close(kv.first);
    clients_.clear();
    if (listen_fd_ >= 0) { ::close(listen_fd_); listen_fd_ = -1; }
    if (!sock_path_.empty()) { ::unlink(sock_path_.c_str()); sock_path_.clear(); }
    if (lock_fd_ >= 0) { ::close(lock_fd_); lock_fd_ = -1; }
}

} // namespace wbr_gps
```

- [ ] **Step 5: Build, run, commit**

Add `src/server.cpp` to `wbr_gps_core` and a `test_server` target.

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make test_server && ./test_server
```

Expected: `0 failed`, exit 0. If `test_slow_client_coalesces_and_does_not_grow` fails, coalescing is broken and one slow client can starve the daemon — do not proceed past this.

```bash
cd /home/sigint-4/prefix/src/WBR-GPS
git add include/wbr_gps/server.hpp src/server.cpp tests/test_server.cpp CMakeLists.txt
git commit -m "feat: unix socket server with singleton lock, coalescing and backpressure"
```

---

## Task 8: The daemon binary

Wires the sources and the server into one `epoll` loop.

**Files:**
- Create: `WBR-GPS/src/main.cpp`
- Modify: `WBR-GPS/CMakeLists.txt`

**Interfaces:**
- Consumes: `SerialSource`, `HidSource`, `Server`, `StateStore`, `leo_bodnar_present`, `find_leo_bodnar_serial`
- Produces: the `wbr-gpsd` executable, with flags `--socket PATH`, `--serial PATH`, `--baud N`, `--hid PATH`, `--lock PATH`, `--group NAME`, `--stale-ms N`, `--foreground`

- [ ] **Step 1: Write the daemon**

Create `WBR-GPS/src/main.cpp`:

```cpp
#include "wbr_gps/device_presence.hpp"
#include "wbr_gps/hid_source.hpp"
#include "wbr_gps/serial_source.hpp"
#include "wbr_gps/server.hpp"
#include "wbr_gps/state_store.hpp"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <unistd.h>

using namespace wbr_gps;

namespace {

volatile sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

int64_t now_unix_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);   // display only, never for ages
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

struct Options {
    std::string socket_path = kDefaultSocketPath;
    std::string serial_path;                  // empty: auto-discover
    std::string hid_path;                     // empty: auto-discover
    std::string lock_path   = "/run/wbr-gps/wbr-gpsd.pid";
    std::string group       = "dialout";
    int         baud        = kDefaultBaud;
    int64_t     stale_ms    = 10000;          // drop has_fix after this
};

void usage(const char* argv0)
{
    std::fprintf(stderr,
        "Usage: %s [--socket PATH] [--serial PATH] [--baud N] [--hid PATH]\n"
        "          [--lock PATH] [--group NAME] [--stale-ms N]\n", argv0);
}

bool parse_args(int argc, char** argv, Options& o)
{
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* n) -> const char* {
            if (i + 1 >= argc) { std::fprintf(stderr, "missing value for %s\n", n); std::exit(2); }
            return argv[++i];
        };
        if      (a == "--socket")   o.socket_path = next("--socket");
        else if (a == "--serial")   o.serial_path = next("--serial");
        else if (a == "--hid")      o.hid_path    = next("--hid");
        else if (a == "--lock")     o.lock_path   = next("--lock");
        else if (a == "--group")    o.group       = next("--group");
        else if (a == "--baud")     o.baud        = std::atoi(next("--baud"));
        else if (a == "--stale-ms") o.stale_ms    = std::atoll(next("--stale-ms"));
        else if (a == "--foreground") { /* default; accepted for systemd clarity */ }
        else if (a == "--help" || a == "-h") { usage(argv[0]); std::exit(0); }
        else { std::fprintf(stderr, "unknown argument: %s\n", a.c_str()); usage(argv[0]); return false; }
    }
    return true;
}

void epoll_set(int ep, int fd, uint32_t events, bool add)
{
    struct epoll_event ev {};
    ev.events  = events;
    ev.data.fd = fd;
    ::epoll_ctl(ep, add ? EPOLL_CTL_ADD : EPOLL_CTL_MOD, fd, &ev);
}

} // namespace

int main(int argc, char** argv)
{
    Options opt;
    if (!parse_args(argc, argv, opt)) return 2;

    // A disconnecting client must never kill the daemon.
    std::signal(SIGPIPE, SIG_IGN);
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    Server server;
    if (!server.acquire_singleton(opt.lock_path)) {
        std::fprintf(stderr, "[wbr-gpsd] another instance is already running\n");
        return 1;
    }
    if (!server.listen_on(opt.socket_path, opt.group.c_str(), 0660)) {
        return 1;
    }
    std::fprintf(stderr, "[wbr-gpsd] listening on %s\n", opt.socket_path.c_str());

    StateStore   store;
    SerialSource serial;
    HidSource    hid;
    serial.set_baud(opt.baud);
    if (!opt.hid_path.empty()) hid.set_path(opt.hid_path);

    const int ep = ::epoll_create1(EPOLL_CLOEXEC);
    epoll_set(ep, server.listen_fd(), EPOLLIN, true);

    // 500 ms tick: reconnect attempts, staleness, presence, heartbeat.
    const int tick = ::timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
    struct itimerspec its {};
    its.it_interval.tv_nsec = 500 * 1000 * 1000;
    its.it_value.tv_nsec    = 500 * 1000 * 1000;
    ::timerfd_settime(tick, 0, &its, nullptr);
    epoll_set(ep, tick, EPOLLIN, true);

    int64_t last_heartbeat_ms = 0;

    while (!g_stop) {
        struct epoll_event evs[64];
        const int n = ::epoll_wait(ep, evs, 64, 500);
        const int64_t now = now_mono_ms();

        for (int i = 0; i < n; ++i) {
            const int fd = evs[i].data.fd;

            if (fd == server.listen_fd()) {
                int c;
                while ((c = server.accept_client()) >= 0) {
                    epoll_set(ep, c, EPOLLIN | EPOLLOUT, true);
                }
                continue;
            }

            if (fd == tick) {
                uint64_t ticks = 0;
                ssize_t rc = ::read(tick, &ticks, sizeof ticks);
                (void)rc;
                continue;
            }

            if (serial.fd() >= 0 && fd == serial.fd()) {
                if (!serial.on_readable(store, now,
                        [&](const std::string& line) {
                            server.broadcast_nmea(line, now_unix_ms());
                        })) {
                    ::epoll_ctl(ep, EPOLL_CTL_DEL, fd, nullptr);
                    store.set_serial_ok(false);
                    serial.note_retry(now);
                }
                continue;
            }

            if (hid.fd() >= 0 && fd == hid.fd()) {
                if (!hid.on_readable(store)) {
                    ::epoll_ctl(ep, EPOLL_CTL_DEL, fd, nullptr);
                    store.set_hid_ok(false);
                    hid.note_retry(now);
                }
                continue;
            }

            // Otherwise it is a client fd.
            if (evs[i].events & (EPOLLHUP | EPOLLERR)) {
                ::epoll_ctl(ep, EPOLL_CTL_DEL, fd, nullptr);
                server.drop_client(fd);
                continue;
            }
            if (evs[i].events & EPOLLIN) {
                if (!server.on_client_readable(fd, store, now)) {
                    ::epoll_ctl(ep, EPOLL_CTL_DEL, fd, nullptr);
                    server.drop_client(fd);
                    continue;
                }
            }
            if (evs[i].events & EPOLLOUT) {
                if (!server.flush_client(fd)) {
                    ::epoll_ctl(ep, EPOLL_CTL_DEL, fd, nullptr);
                    server.drop_client(fd);
                }
            }
        }

        // --- periodic work, every loop iteration ---

        // 1. Presence: the ref-present signal, independent of whether the
        //    serial port opened.
        store.set_device_present(leo_bodnar_present());

        // 2. Reconnect the serial port with backoff.
        if (serial.fd() < 0 && serial.should_retry(now)) {
            std::string path = opt.serial_path;
            if (path.empty()) {
                path = find_leo_bodnar_serial();
                if (path.empty()) path = kDefaultSerialPath;
            }
            serial.set_path(path);
            if (serial.open_device()) {
                epoll_set(ep, serial.fd(), EPOLLIN, true);
                store.set_serial_ok(true);
                std::fprintf(stderr, "[wbr-gpsd] serial open: %s\n", path.c_str());
            } else {
                serial.note_retry(now);
            }
        }

        // 3. Reconnect hidraw with backoff.
        if (hid.fd() < 0 && hid.should_retry(now)) {
            if (hid.open_device()) {
                epoll_set(ep, hid.fd(), EPOLLIN, true);
                store.set_hid_ok(true);
            } else {
                hid.note_retry(now);
            }
        }

        // 4. Expire a fix that has stopped arriving.
        store.mark_fix_stale_if_older_than(now, opt.stale_ms);

        // 5. Push. A heartbeat at least every 2 s lets clients detect a dead
        //    daemon by silence rather than waiting forever.
        static uint64_t last_seq = 0;
        const bool changed   = store.seq() != last_seq;
        const bool heartbeat = now - last_heartbeat_ms >= 2000;
        if (changed || heartbeat) {
            server.broadcast_fix(store, now);
            last_seq           = store.seq();
            last_heartbeat_ms  = now;
        }

        // 6. Drain client queues.
        for (int fd = 0;;) { (void)fd; break; }   // handled via EPOLLOUT above
    }

    std::fprintf(stderr, "[wbr-gpsd] shutting down\n");
    serial.close_device();
    hid.close_device();
    server.shutdown();
    ::close(tick);
    ::close(ep);
    return 0;
}
```

- [ ] **Step 2: Add the target**

Append to `WBR-GPS/CMakeLists.txt`:

```cmake
add_executable(wbr-gpsd src/main.cpp)
target_link_libraries(wbr-gpsd PRIVATE wbr_gps_core)
install(TARGETS wbr-gpsd RUNTIME DESTINATION bin)
```

- [ ] **Step 3: Build and smoke-test against a pty**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make wbr-gpsd
```

In terminal A, make a fake GPS and run the daemon against it:

```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0 &
# note the two /dev/pts/N paths socat prints; use the second as --serial
mkdir -p /tmp/wbrgps
./wbr-gpsd --socket /tmp/wbrgps/gpsd.sock --lock /tmp/wbrgps/pid \
           --serial /dev/pts/<N> --group "" &
```

In terminal B, feed a sentence and read the socket:

```bash
printf '$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F\r\n' > /dev/pts/<M>
printf '{"op":"get"}\n' | socat - UNIX-CONNECT:/tmp/wbrgps/gpsd.sock
```

Expected: a `HELLO` line then a `FIX` line with `"has_fix":true` and `"lat":13.0028260`.

- [ ] **Step 4: Verify exclusivity against the real device**

```bash
sudo systemctl stop ModemManager
cd /home/sigint-4/prefix/src/WBR-GPS/build
mkdir -p /tmp/wbrgps
./wbr-gpsd --socket /tmp/wbrgps/gpsd.sock --lock /tmp/wbrgps/pid \
           --serial /dev/ttyACM0 --group "" &
sleep 3
# Exactly one opener, and it is wbr-gpsd:
lsof /dev/ttyACM0
# A second reader must now fail loudly rather than steal bytes:
python3 -c "import serial; serial.Serial('/dev/ttyACM0',115200,timeout=1)" ; echo "exit=$?"
```

Expected: `lsof` shows one row, `wbr-gpsd`. The Python open fails with a device-busy error, exit non-zero. **If the Python open succeeds, `TIOCEXCL` is not working and guarantee G1 is not held — stop and fix before continuing.**

- [ ] **Step 5: Commit**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS
git add src/main.cpp CMakeLists.txt
git commit -m "feat: wbr-gpsd daemon binary with single epoll loop"
```

---

## Task 9: C++ client library

Designed as a drop-in for the existing call sites so migration diffs stay small.

**Files:**
- Create: `WBR-GPS/include/wbr_gps/client.hpp`
- Create: `WBR-GPS/src/client.cpp`
- Create: `WBR-GPS/tests/test_client.cpp`
- Modify: `WBR-GPS/CMakeLists.txt`

**Interfaces:**
- Consumes: `Snapshot`, `snapshot_from_json`, the daemon's wire protocol
- Produces: class `wbr_gps::Client` with `start(const std::string& sock_path = kDefaultSocketPath, bool want_nmea = false) -> bool`, `stop()`, `snapshot() const -> Snapshot`, `set_nmea_callback(std::function<void(const std::string&)>)`; free function `wbr_gps::get_once(const std::string& sock_path = kDefaultSocketPath, int timeout_ms = 500) -> Snapshot`

- [ ] **Step 1: Write the failing test**

Create `WBR-GPS/tests/test_client.cpp`. It runs the real daemon against a pty, so it exercises the actual protocol:

```cpp
#include "wbr_gps/client.hpp"
#include "pty_harness.h"
#include "test_util.h"

#include <cstdlib>
#include <signal.h>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

static const char* kGGA =
    "$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F";

// Path to wbr-gpsd, set by CMake so the test does not guess.
#ifndef WBR_GPSD_PATH
#define WBR_GPSD_PATH "./wbr-gpsd"
#endif

struct Daemon {
    pid_t       pid = -1;
    std::string dir;
    std::string sock;

    void start(const std::string& serial_path)
    {
        char tmpl[] = "/tmp/wbrgps_client_XXXXXX";
        dir  = mkdtemp(tmpl);
        sock = dir + "/gpsd.sock";
        const std::string lock = dir + "/pid";

        pid = fork();
        if (pid == 0) {
            execl(WBR_GPSD_PATH, "wbr-gpsd",
                  "--socket", sock.c_str(),
                  "--lock",   lock.c_str(),
                  "--serial", serial_path.c_str(),
                  "--group",  "",
                  (char*)nullptr);
            _exit(127);
        }
        usleep(400000);   // let it bind
    }

    void stop()
    {
        if (pid > 0) { kill(pid, SIGTERM); int st = 0; waitpid(pid, &st, 0); pid = -1; }
    }

    ~Daemon() { stop(); }
};

static void test_snapshot_tracks_daemon()
{
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    Daemon d;
    d.start(pty.slave_path);

    wbr_gps::Client c;
    ASSERT_TRUE(c.start(d.sock));
    usleep(300000);

    // Before any sentence: connected to the service, but no fix.
    wbr_gps::Snapshot s = c.snapshot();
    ASSERT_TRUE(s.service_ok);
    ASSERT_FALSE(s.has_fix);

    pty.emit(kGGA);
    usleep(800000);

    s = c.snapshot();
    ASSERT_TRUE(s.service_ok);
    ASSERT_TRUE(s.has_fix);
    ASSERT_NEAR(s.lat, 13.0028260, 1e-6);
    ASSERT_NEAR(s.lon, 77.6799202, 1e-6);

    c.stop();
    d.stop();
}

static void test_service_down_is_explicit_not_silent()
{
    // Guarantee G4/G5: when the daemon dies the client must report
    // service_ok=false, NOT quietly open the device itself.
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    Daemon d;
    d.start(pty.slave_path);

    wbr_gps::Client c;
    ASSERT_TRUE(c.start(d.sock));
    pty.emit(kGGA);
    usleep(800000);
    ASSERT_TRUE(c.snapshot().service_ok);

    d.stop();
    usleep(1200000);

    const wbr_gps::Snapshot s = c.snapshot();
    ASSERT_FALSE(s.service_ok);

    c.stop();
}

static void test_client_reconnects_after_daemon_restart()
{
    // S13.
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    Daemon d;
    d.start(pty.slave_path);

    wbr_gps::Client c;
    ASSERT_TRUE(c.start(d.sock));
    usleep(300000);
    ASSERT_TRUE(c.snapshot().service_ok);

    const std::string sock = d.sock;
    const std::string dir  = d.dir;
    d.stop();
    usleep(600000);
    ASSERT_FALSE(c.snapshot().service_ok);

    // Restart on the same socket path.
    Daemon d2;
    d2.dir  = dir;
    d2.sock = sock;
    d2.pid  = fork();
    if (d2.pid == 0) {
        execl(WBR_GPSD_PATH, "wbr-gpsd",
              "--socket", sock.c_str(), "--lock", (dir + "/pid").c_str(),
              "--serial", pty.slave_path.c_str(), "--group", "", (char*)nullptr);
        _exit(127);
    }
    usleep(2500000);   // client backoff plus daemon startup

    ASSERT_TRUE(c.snapshot().service_ok);

    c.stop();
    d2.stop();
}

static void test_get_once_without_daemon_reports_unavailable()
{
    const wbr_gps::Snapshot s = wbr_gps::get_once("/tmp/definitely-no-socket-here", 200);
    ASSERT_FALSE(s.service_ok);
    ASSERT_FALSE(s.has_fix);
    ASSERT_FALSE(s.device_present);
}

static void test_get_once_returns_a_fix()
{
    PtyPair pty;
    ASSERT_TRUE(pty.open_pair());
    Daemon d;
    d.start(pty.slave_path);
    pty.emit(kGGA);
    usleep(800000);

    const wbr_gps::Snapshot s = wbr_gps::get_once(d.sock, 1000);
    ASSERT_TRUE(s.service_ok);
    ASSERT_TRUE(s.has_fix);
    ASSERT_NEAR(s.lat, 13.0028260, 1e-6);

    d.stop();
}

static void run_tests()
{
    test_snapshot_tracks_daemon();
    test_service_down_is_explicit_not_silent();
    test_client_reconnects_after_daemon_restart();
    test_get_once_without_daemon_reports_unavailable();
    test_get_once_returns_a_fix();
}

TEST_MAIN
```

- [ ] **Step 2: Run to verify it fails**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make test_client
```

Expected: `fatal error: wbr_gps/client.hpp: No such file or directory`.

- [ ] **Step 3: Create the header**

Create `WBR-GPS/include/wbr_gps/client.hpp`. **This file, and `src/client.cpp`, must contain no `open()`, no `termios` and no `/dev` path — that absence is guarantee G5:**

```cpp
#pragma once

#include "wbr_gps/gps_types.hpp"

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace wbr_gps {

// Talks to wbr-gpsd over a Unix socket. Deliberately has NO device access:
// if the daemon is unavailable, snapshot().service_ok is false and that is
// the whole story. There is no fallback path, by design (guarantee G5).
class Client {
public:
    ~Client();

    // Connect and keep a background thread maintaining a local cache. Returns
    // true once the thread is running; it returns true even if the daemon is
    // not up yet, because the thread will keep retrying. Check
    // snapshot().service_ok to learn whether data is actually flowing.
    bool start(const std::string& sock_path = kDefaultSocketPath,
               bool want_nmea = false);
    void stop();

    // Lock-guarded read of the cached snapshot. Cheap: no syscall. Safe to
    // call from a hot loop, which is how gsm_monitor uses it.
    Snapshot snapshot() const;

    // Raw NMEA sentences, when started with want_nmea = true. Called from the
    // background thread.
    void set_nmea_callback(std::function<void(const std::string&)> cb);

private:
    void run();

    std::string             sock_path_;
    bool                    want_nmea_ = false;
    std::atomic<bool>       stopping_{ false };
    std::atomic<bool>       running_{ false };
    std::thread             thread_;
    mutable std::mutex      mu_;
    Snapshot                cached_;
    std::function<void(const std::string&)> nmea_cb_;
};

// One synchronous request. For startup checks such as the external-reference
// decision in gsm_monitor.cpp:772, where no long-lived client exists yet.
// Returns a default Snapshot with service_ok = false if the daemon is down.
Snapshot get_once(const std::string& sock_path = kDefaultSocketPath,
                  int timeout_ms = 500);

} // namespace wbr_gps
```

- [ ] **Step 4: Implement**

Create `WBR-GPS/src/client.cpp`:

```cpp
#include "wbr_gps/client.hpp"
#include "wbr_gps/json_io.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace wbr_gps {

namespace {

constexpr int kReconnectMinMs = 250;
constexpr int kReconnectMaxMs = 2000;
// If no message arrives for this long the daemon is considered gone: it
// heartbeats every 2 s, so 6 s is three missed beats.
constexpr int64_t kSilenceTimeoutMs = 6000;

int connect_unix(const std::string& path, int timeout_ms)
{
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);

    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof addr.sun_path, "%s", path.c_str());

    if (::connect(fd, (struct sockaddr*)&addr, sizeof addr) != 0) {
        ::close(fd);
        return -1;
    }
    return fd;
}

bool send_all(int fd, const std::string& s)
{
    size_t off = 0;
    while (off < s.size()) {
        const ssize_t n = ::send(fd, s.data() + off, s.size() - off, MSG_NOSIGNAL);
        if (n > 0) { off += (size_t)n; continue; }
        if (n < 0 && errno == EINTR) continue;
        return false;
    }
    return true;
}

} // namespace

Client::~Client() { stop(); }

void Client::set_nmea_callback(std::function<void(const std::string&)> cb)
{
    std::lock_guard<std::mutex> lk(mu_);
    nmea_cb_ = std::move(cb);
}

bool Client::start(const std::string& sock_path, bool want_nmea)
{
    if (running_.load()) return true;
    sock_path_ = sock_path;
    want_nmea_ = want_nmea;
    stopping_.store(false);
    running_.store(true);
    thread_ = std::thread([this] { run(); });
    return true;
}

void Client::stop()
{
    if (!running_.load()) return;
    stopping_.store(true);
    if (thread_.joinable()) thread_.join();
    running_.store(false);
}

Snapshot Client::snapshot() const
{
    std::lock_guard<std::mutex> lk(mu_);
    return cached_;
}

void Client::run()
{
    int backoff = kReconnectMinMs;

    while (!stopping_.load()) {
        const int fd = connect_unix(sock_path_, 500);
        if (fd < 0) {
            {   // Explicit unavailability. Never a fallback to the device.
                std::lock_guard<std::mutex> lk(mu_);
                cached_.service_ok = false;
            }
            for (int slept = 0; slept < backoff && !stopping_.load(); slept += 50) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            backoff = std::min(backoff * 2, kReconnectMaxMs);
            continue;
        }
        backoff = kReconnectMinMs;

        const std::string sub = std::string("{\"op\":\"watch\",\"fix\":true,\"nmea\":") +
                                (want_nmea_ ? "true" : "false") + "}\n";
        if (!send_all(fd, sub)) { ::close(fd); continue; }

        std::string buf;
        int64_t last_msg_ms = now_mono_ms();

        while (!stopping_.load()) {
            struct pollfd p { fd, POLLIN, 0 };
            const int pr = ::poll(&p, 1, 200);

            if (pr > 0 && (p.revents & POLLIN)) {
                char io[2048];
                const ssize_t n = ::read(fd, io, sizeof io);
                if (n <= 0) break;                 // daemon gone
                buf.append(io, (size_t)n);
                last_msg_ms = now_mono_ms();

                size_t nl;
                while ((nl = buf.find('\n')) != std::string::npos) {
                    const std::string line = buf.substr(0, nl);
                    buf.erase(0, nl + 1);

                    Snapshot s;
                    if (snapshot_from_json(line, s)) {
                        s.service_ok = true;
                        std::lock_guard<std::mutex> lk(mu_);
                        cached_ = s;
                        continue;
                    }
                    std::string cls, raw;
                    if (json_get_string(line, "class", cls) && cls == "NMEA" &&
                        json_get_string(line, "raw", raw)) {
                        std::function<void(const std::string&)> cb;
                        { std::lock_guard<std::mutex> lk(mu_); cb = nmea_cb_; }
                        if (cb) cb(raw);
                    }
                }
                continue;
            }
            if (pr < 0 && errno == EINTR) continue;
            if (pr < 0) break;

            // Silence for longer than three heartbeats: treat as gone.
            if (now_mono_ms() - last_msg_ms > kSilenceTimeoutMs) break;
        }

        ::close(fd);
        std::lock_guard<std::mutex> lk(mu_);
        cached_.service_ok = false;
    }

    std::lock_guard<std::mutex> lk(mu_);
    cached_.service_ok = false;
}

Snapshot get_once(const std::string& sock_path, int timeout_ms)
{
    Snapshot out;   // service_ok = false by default: unavailable unless proven
    const int fd = connect_unix(sock_path, timeout_ms);
    if (fd < 0) return out;

    if (!send_all(fd, "{\"op\":\"get\"}\n")) { ::close(fd); return out; }

    std::string buf;
    const int64_t deadline = now_mono_ms() + timeout_ms;
    while (now_mono_ms() < deadline) {
        struct pollfd p { fd, POLLIN, 0 };
        const int pr = ::poll(&p, 1, 50);
        if (pr <= 0) continue;

        char io[2048];
        const ssize_t n = ::read(fd, io, sizeof io);
        if (n <= 0) break;
        buf.append(io, (size_t)n);

        size_t nl;
        while ((nl = buf.find('\n')) != std::string::npos) {
            const std::string line = buf.substr(0, nl);
            buf.erase(0, nl + 1);
            Snapshot s;
            if (snapshot_from_json(line, s)) {   // skips HELLO
                s.service_ok = true;
                ::close(fd);
                return s;
            }
        }
    }
    ::close(fd);
    return out;
}

} // namespace wbr_gps
```

- [ ] **Step 5: Build, run, commit**

Add `src/client.cpp` to `wbr_gps_core` and append:

```cmake
add_executable(test_client tests/test_client.cpp)
target_include_directories(test_client PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_compile_definitions(test_client PRIVATE
  WBR_GPSD_PATH="${CMAKE_CURRENT_BINARY_DIR}/wbr-gpsd")
target_link_libraries(test_client PRIVATE wbr_gps_core util pthread)
add_dependencies(test_client wbr-gpsd)
add_test(NAME client COMMAND test_client)
```

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make test_client && ./test_client
```

Expected: `0 failed`, exit 0.

```bash
cd /home/sigint-4/prefix/src/WBR-GPS
git add include/wbr_gps/client.hpp src/client.cpp tests/test_client.cpp CMakeLists.txt
git commit -m "feat: C++ client library with no device access by construction"
```

---

## Task 10: Python client library

**Files:**
- Create: `WBR-GPS/python/wbr_gps_client.py`
- Create: `WBR-GPS/python/test_wbr_gps_client.py`

**Interfaces:**
- Consumes: the daemon's wire protocol
- Produces: `wbr_gps_client.get_once(sock_path=DEFAULT_SOCKET, timeout=0.5) -> dict`; `wbr_gps_client.badge_status(snap: dict) -> str`; `wbr_gps_client.unavailable() -> dict`; constant `wbr_gps_client.DEFAULT_SOCKET`

- [ ] **Step 1: Write the failing test**

Create `WBR-GPS/python/test_wbr_gps_client.py`:

```python
"""Tests for the Python GPS client. Runs against a stub server, so no daemon
and no hardware are needed."""
import json
import os
import socket
import tempfile
import threading
import unittest

import wbr_gps_client as gc


class StubDaemon:
    """A minimal wbr-gpsd that speaks the same wire protocol."""

    def __init__(self, fix_payload=None, send_hello=True):
        self.dir = tempfile.mkdtemp()
        self.path = os.path.join(self.dir, "gpsd.sock")
        self.fix = fix_payload
        self.send_hello = send_hello
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind(self.path)
        self.sock.listen(8)
        self.stop = False
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self):
        while not self.stop:
            try:
                conn, _ = self.sock.accept()
            except OSError:
                return
            with conn:
                if self.send_hello:
                    conn.sendall(b'{"class":"HELLO","proto":1}\n')
                try:
                    conn.recv(4096)
                except OSError:
                    continue
                if self.fix is not None:
                    conn.sendall((json.dumps(self.fix) + "\n").encode())

    def close(self):
        self.stop = True
        self.sock.close()


LOCKED_FIX = {
    "class": "FIX", "seq": 7,
    "device_present": True, "serial_ok": True, "hid_ok": True,
    "gpsdo_locked": True, "has_fix": True,
    "fix_quality": 1, "satellites": 4, "hdop": 1.33,
    "lat": 13.002826, "lon": 77.6799202, "alt_m": 921.8,
    "speed_kph": 0.0, "time_utc": "045519.50", "fix_age_ms": 480,
}


class TestGetOnce(unittest.TestCase):
    def test_returns_fix(self):
        d = StubDaemon(LOCKED_FIX)
        try:
            snap = gc.get_once(d.path, timeout=1.0)
            self.assertTrue(snap["service_ok"])
            self.assertTrue(snap["has_fix"])
            self.assertAlmostEqual(snap["lat"], 13.002826, places=6)
            self.assertEqual(snap["satellites"], 4)
        finally:
            d.close()

    def test_skips_hello_and_finds_fix(self):
        d = StubDaemon(LOCKED_FIX, send_hello=True)
        try:
            self.assertTrue(gc.get_once(d.path, timeout=1.0)["has_fix"])
        finally:
            d.close()

    def test_no_daemon_reports_unavailable(self):
        # The critical property: a missing daemon must NEVER look healthy,
        # and must never cause a fallback to opening the device.
        snap = gc.get_once("/tmp/definitely-no-socket-here", timeout=0.2)
        self.assertFalse(snap["service_ok"])
        self.assertFalse(snap["has_fix"])
        self.assertFalse(snap["device_present"])

    def test_daemon_that_never_replies_times_out(self):
        d = StubDaemon(fix_payload=None)
        try:
            snap = gc.get_once(d.path, timeout=0.3)
            self.assertFalse(snap["service_ok"])
        finally:
            d.close()

    def test_garbage_reply_is_not_trusted(self):
        d = StubDaemon(fix_payload=None)
        try:
            snap = gc.get_once(d.path, timeout=0.3)
            self.assertFalse(snap["has_fix"])
        finally:
            d.close()


class TestBadgeStatus(unittest.TestCase):
    """The four honest states that replace the old three-state guess."""

    def test_locked(self):
        self.assertEqual(gc.badge_status(dict(LOCKED_FIX, service_ok=True)), "Locked")

    def test_not_locked_when_present_but_no_fix(self):
        snap = dict(LOCKED_FIX, service_ok=True, has_fix=False, gpsdo_locked=False)
        self.assertEqual(gc.badge_status(snap), "Not Locked")

    def test_not_connected_when_device_absent(self):
        snap = dict(LOCKED_FIX, service_ok=True, device_present=False,
                    has_fix=False, gpsdo_locked=False)
        self.assertEqual(gc.badge_status(snap), "Not Connected")

    def test_service_down_is_its_own_state(self):
        # The state the old code could not express, and so reported wrongly.
        self.assertEqual(gc.badge_status({"service_ok": False}), "GPS Service Down")

    def test_gpsdo_locked_without_nmea_fix_still_reads_locked(self):
        # The LB clock can be locked with no current GGA. These are different
        # signals and the badge must not lose that.
        snap = dict(LOCKED_FIX, service_ok=True, has_fix=False, gpsdo_locked=True)
        self.assertEqual(gc.badge_status(snap), "Locked")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run to verify it fails**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/python && python3 -m unittest test_wbr_gps_client -v
```

Expected: `ModuleNotFoundError: No module named 'wbr_gps_client'`.

- [ ] **Step 3: Implement**

Create `WBR-GPS/python/wbr_gps_client.py`:

```python
"""Client for wbr-gpsd, the single owner of the Leo Bodnar GPSDO.

This module deliberately contains NO serial, hidraw or /dev access. If the
daemon is unavailable, `service_ok` is False and that is the complete answer.
Falling back to reading the device directly is what caused the contention
this daemon exists to remove.

Standard library only: this is imported by Django and must add no dependency.
"""
import json
import socket

DEFAULT_SOCKET = "/run/wbr-gps/gpsd.sock"

# The shape returned when we cannot reach the daemon. Every field is the
# pessimistic value, so a caller that ignores service_ok still cannot mistake
# a dead daemon for a healthy one.
_UNAVAILABLE = {
    "service_ok": False,
    "device_present": False,
    "serial_ok": False,
    "hid_ok": False,
    "gpsdo_locked": False,
    "has_fix": False,
    "fix_quality": 0,
    "satellites": 0,
    "hdop": 0.0,
    "lat": 0.0,
    "lon": 0.0,
    "alt_m": 0.0,
    "speed_kph": 0.0,
    "time_utc": "",
    "fix_age_ms": 0,
    "seq": 0,
}


def unavailable():
    """A fresh snapshot dict meaning "the daemon could not be reached".

    Public so callers can build the same pessimistic shape without reaching
    into module internals.
    """
    return dict(_UNAVAILABLE)


def get_once(sock_path=DEFAULT_SOCKET, timeout=0.5):
    """One snapshot from the daemon, as a dict.

    Always returns a dict; never raises. A Unix-socket round trip is
    sub-millisecond, so this is safe to call directly from a request thread
    without the background-refresh cache the old probe needed.
    """
    sock = None
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect(sock_path)
        sock.sendall(b'{"op":"get"}\n')

        buf = b""
        while b"\n" in buf or len(buf) < 65536:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line.decode("utf-8", "replace"))
                except ValueError:
                    continue
                # Skip HELLO and anything else; only a FIX is an answer.
                if msg.get("class") == "FIX":
                    snap = dict(_UNAVAILABLE)
                    snap.update(msg)
                    snap["service_ok"] = True
                    snap.pop("class", None)
                    return snap
        return dict(_UNAVAILABLE)
    except (OSError, socket.timeout):
        return dict(_UNAVAILABLE)
    finally:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


def badge_status(snap):
    """The header badge string.

    Four states, not three. The old code could not express "the GPS service
    is down" and so reported "Not Connected" or a stale "Locked" instead —
    a confident wrong answer. Order matters: service health is checked first,
    because every other field is meaningless without it.
    """
    if not snap.get("service_ok"):
        return "GPS Service Down"
    if not snap.get("device_present"):
        return "Not Connected"
    # The GPSDO lock bit and the NMEA fix are different signals; either one
    # being good means the disciplined clock is usable.
    if snap.get("gpsdo_locked") or snap.get("has_fix"):
        return "Locked"
    return "Not Locked"
```

- [ ] **Step 4: Run to verify it passes**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/python && python3 -m unittest test_wbr_gps_client -v
```

Expected: `OK`, 10 tests.

- [ ] **Step 5: Commit**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS
git add python/
git commit -m "feat: Python GPS client with an honest four-state badge"
```

---

## Task 11: Deployment — udev rules and systemd unit

Guarantee G5 requires the daemon to always be running, which requires supervision.

**Files:**
- Create: `WBR-GPS/udev/99-wbr-gps.rules`
- Create: `WBR-GPS/systemd/wbr-gpsd.service`
- Create: `WBR-GPS/install.sh`

**Interfaces:**
- Consumes: the `wbr-gpsd` binary from Task 8
- Produces: `/dev/gpsdo` symlink; `ID_MM_DEVICE_IGNORE` on the GPSDO tty; the `wbr-gpsd.service` unit

- [ ] **Step 1: Create the udev rules**

Create `WBR-GPS/udev/99-wbr-gps.rules`. Vendor `1dd2`, product `2444`, confirmed on this box with `udevadm info -q property -n /dev/ttyACM0`:

```
# Leo Bodnar LBE-1421 GPSDO, owned exclusively by wbr-gpsd.

# 1. Stop ModemManager probing the GPSDO as a modem. Without this, MM opens
#    /dev/ttyACM0 and writes AT command strings at it on every hotplug —
#    udevadm reports ID_MM_CANDIDATE=1 for this device. That is a fifth
#    uninvited reader and a real source of startup flakiness.
SUBSYSTEM=="tty", ATTRS{idVendor}=="1dd2", ATTRS{idProduct}=="2444", ENV{ID_MM_DEVICE_IGNORE}="1"

# 2. Stable symlink, so the daemon never has to guess a path and survives
#    ttyACM0 -> ttyACM1 renumbering across replug.
SUBSYSTEM=="tty", ATTRS{idVendor}=="1dd2", ATTRS{idProduct}=="2444", SYMLINK+="gpsdo", GROUP="dialout", MODE="0660"

# 3. Non-root access to the Leo Bodnar HID interface. This supersedes the
#    existing /etc/udev/rules.d/99-leobodnar.rules, which sets MODE 0666 for
#    all of vendor 1dd2; 0660 with a group is tighter and sufficient.
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="1dd2", GROUP="dialout", MODE="0660"
```

- [ ] **Step 2: Create the systemd unit**

Create `WBR-GPS/systemd/wbr-gpsd.service`:

```ini
[Unit]
Description=WBR GPS proxy daemon (single owner of the Leo Bodnar GPSDO)
Documentation=file:/home/sigint-4/prefix/src/WBR-GPS/docs/superpowers/specs/2026-09-01-gps-proxy-daemon-design.md
After=systemd-udev-settle.service
Wants=systemd-udev-settle.service

[Service]
Type=simple
ExecStart=/usr/local/bin/wbr-gpsd --socket /run/wbr-gps/gpsd.sock --lock /run/wbr-gps/wbr-gpsd.pid --group dialout
Restart=always
RestartSec=2

# The socket and pidfile live here; systemd creates and cleans it up.
RuntimeDirectory=wbr-gps
RuntimeDirectoryMode=0755

# Runs unprivileged. dialout for the tty, plugdev is harmless but kept in
# case a future hidraw rule uses it.
User=root
Group=dialout

# Hardening. The daemon needs /dev, a Unix socket and nothing else.
NoNewPrivileges=yes
ProtectSystem=strict
ProtectHome=yes
PrivateTmp=yes
PrivateNetwork=yes
RestrictAddressFamilies=AF_UNIX
MemoryMax=64M

[Install]
WantedBy=multi-user.target
```

`PrivateNetwork=yes` and `RestrictAddressFamilies=AF_UNIX` are worth noting: the daemon has no network transport by design, so the kernel is told to enforce that. If a TCP listener is ever added, both lines must change.

- [ ] **Step 3: Create the installer**

Create `WBR-GPS/install.sh`:

```bash
#!/usr/bin/env bash
# Install wbr-gpsd, its udev rules and its systemd unit. Requires root.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $EUID -ne 0 ]]; then
  echo "install.sh must run as root (try: sudo $0)" >&2
  exit 1
fi

if [[ ! -x "$HERE/build/wbr-gpsd" ]]; then
  echo "build/wbr-gpsd not found. Run: cmake -S '$HERE' -B '$HERE/build' && make -C '$HERE/build'" >&2
  exit 1
fi

echo "==> installing binary"
install -m 0755 "$HERE/build/wbr-gpsd" /usr/local/bin/wbr-gpsd

echo "==> installing udev rules"
install -m 0644 "$HERE/udev/99-wbr-gps.rules" /etc/udev/rules.d/99-wbr-gps.rules
# The old rule is superseded by ours; keep a backup rather than deleting.
if [[ -f /etc/udev/rules.d/99-leobodnar.rules ]]; then
  mv /etc/udev/rules.d/99-leobodnar.rules /etc/udev/rules.d/99-leobodnar.rules.superseded
  echo "    moved 99-leobodnar.rules aside (superseded)"
fi
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty --subsystem-match=hidraw

echo "==> installing systemd unit"
install -m 0644 "$HERE/systemd/wbr-gpsd.service" /etc/systemd/system/wbr-gpsd.service
systemctl daemon-reload
systemctl enable --now wbr-gpsd.service

echo "==> verifying"
sleep 2
systemctl is-active wbr-gpsd.service
ls -l /dev/gpsdo
echo "openers of the GPS device (expect exactly one: wbr-gpsd):"
lsof "$(readlink -f /dev/gpsdo)" || true
```

- [ ] **Step 4: Install and verify**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS
chmod +x install.sh
sudo ./install.sh
```

Then confirm each property independently:

```bash
# 1. The symlink exists and points at the real tty.
ls -l /dev/gpsdo

# 2. ModemManager now ignores it.
udevadm info -q property -n /dev/gpsdo | grep -E 'ID_MM_DEVICE_IGNORE|ID_MM_CANDIDATE'

# 3. Exactly one opener, and it is wbr-gpsd.
lsof "$(readlink -f /dev/gpsdo)"

# 4. The socket exists with the right ownership.
ls -l /run/wbr-gps/gpsd.sock

# 5. It answers.
printf '{"op":"get"}\n' | socat - UNIX-CONNECT:/run/wbr-gps/gpsd.sock
```

Expected: `ID_MM_DEVICE_IGNORE=1`; `lsof` shows one row for `wbr-gpsd`; the socket is `srw-rw---- root dialout`; the `get` returns `HELLO` then a `FIX` line.

- [ ] **Step 5: Verify restart-on-crash**

```bash
sudo kill -9 "$(systemctl show -p MainPID --value wbr-gpsd)"
sleep 4
systemctl is-active wbr-gpsd     # expect: active
lsof "$(readlink -f /dev/gpsdo)" # expect: exactly one wbr-gpsd, new PID
```

- [ ] **Step 6: Commit**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS
git add udev/ systemd/ install.sh
git commit -m "feat: udev rules, systemd unit and installer for wbr-gpsd"
```

---

## Task 12: Concurrency suite (scenarios S1–S15) and the G5 structural check

This is the task that answers "test all possible scenarios with multiple programs trying to connect." Scenarios S6, S7, S10, S14 and S15 were covered at unit level in Task 7; this task exercises them end-to-end against the real daemon binary, and adds the multi-client scenarios that only make sense against a live process.

**Files:**
- Create: `WBR-GPS/tests/test_concurrency.cpp`
- Create: `WBR-GPS/tests/check_no_device_access.sh`
- Modify: `WBR-GPS/CMakeLists.txt`

**Interfaces:**
- Consumes: the `wbr-gpsd` binary, `PtyPair`, `wbr_gps::Client`, `wbr_gps::get_once`
- Produces: the `test_concurrency` binary and the `check_no_device_access.sh` gate

- [ ] **Step 1: Write the G5 structural check**

Create `WBR-GPS/tests/check_no_device_access.sh`. Guarantee G5 says client libraries contain no device code; this makes that mechanically enforced rather than a promise:

```bash
#!/usr/bin/env bash
# Guarantee G5: the client libraries must contain no device access, so no
# consumer can fall back to opening the GPS directly and reintroduce the
# contention this daemon exists to remove.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FILES=(
  "$HERE/include/wbr_gps/client.hpp"
  "$HERE/src/client.cpp"
  "$HERE/python/wbr_gps_client.py"
)

# Patterns that would mean a client can touch hardware.
PATTERNS='::open\(|[^_a-zA-Z]open\(|termios|tcsetattr|cfsetispeed|/dev/tty|/dev/hidraw|/dev/serial|serial\.Serial|import serial|TIOCEXCL'

fail=0
for f in "${FILES[@]}"; do
  if [[ ! -f "$f" ]]; then
    echo "MISSING: $f" >&2
    fail=1
    continue
  fi
  if grep -nE "$PATTERNS" "$f"; then
    echo "FAIL: $f contains device access (guarantee G5 violated)" >&2
    fail=1
  fi
done

if [[ $fail -eq 0 ]]; then
  echo "PASS: no device access in client libraries (G5 holds)"
fi
exit $fail
```

- [ ] **Step 2: Run it — it must pass against Task 9 and 10's output**

```bash
chmod +x /home/sigint-4/prefix/src/WBR-GPS/tests/check_no_device_access.sh
/home/sigint-4/prefix/src/WBR-GPS/tests/check_no_device_access.sh
```

Expected: `PASS: no device access in client libraries (G5 holds)`, exit 0.

- [ ] **Step 3: Write the concurrency scenarios**

Create `WBR-GPS/tests/test_concurrency.cpp`:

```cpp
// Scenarios S1-S15 from the design doc, run against the real wbr-gpsd binary
// with a pty standing in for the Leo Bodnar.
#include "wbr_gps/client.hpp"
#include "wbr_gps/json_io.hpp"
#include "pty_harness.h"
#include "test_util.h"

#include <atomic>
#include <cstdlib>
#include <dirent.h>
#include <signal.h>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#ifndef WBR_GPSD_PATH
#define WBR_GPSD_PATH "./wbr-gpsd"
#endif

static const char* kGGA =
    "$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,921.8,M,-86.3,M,,*6F";

// ---------- helpers ----------

struct Daemon {
    pid_t pid = -1;
    std::string dir, sock, lock;

    void start(const std::string& serial_path)
    {
        char tmpl[] = "/tmp/wbrgps_conc_XXXXXX";
        dir  = mkdtemp(tmpl);
        sock = dir + "/gpsd.sock";
        lock = dir + "/pid";
        spawn(serial_path);
    }

    void spawn(const std::string& serial_path)
    {
        pid = fork();
        if (pid == 0) {
            execl(WBR_GPSD_PATH, "wbr-gpsd", "--socket", sock.c_str(),
                  "--lock", lock.c_str(), "--serial", serial_path.c_str(),
                  "--group", "", (char*)nullptr);
            _exit(127);
        }
        usleep(500000);
    }

    void kill_hard() { if (pid > 0) { kill(pid, SIGKILL); int st; waitpid(pid, &st, 0); pid = -1; } }
    void stop()      { if (pid > 0) { kill(pid, SIGTERM); int st; waitpid(pid, &st, 0); pid = -1; } }
    ~Daemon()        { stop(); }
};

static int raw_connect(const std::string& path)
{
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un a {};
    a.sun_family = AF_UNIX;
    snprintf(a.sun_path, sizeof a.sun_path, "%s", path.c_str());
    if (::connect(fd, (struct sockaddr*)&a, sizeof a) != 0) { ::close(fd); return -1; }
    return fd;
}

static void send_line(int fd, const std::string& s)
{
    const std::string f = s + "\n";
    ssize_t rc = ::send(fd, f.data(), f.size(), MSG_NOSIGNAL);
    (void)rc;
}

static std::string read_line(int fd, int timeout_ms = 3000)
{
    std::string out;
    struct timeval tv { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    char c;
    while (::recv(fd, &c, 1, 0) == 1) {
        if (c == '\n') break;
        out += c;
    }
    return out;
}

// Read lines until one parses as a FIX, skipping HELLO.
static bool read_fix(int fd, wbr_gps::Snapshot& out, int tries = 5)
{
    for (int i = 0; i < tries; ++i) {
        const std::string line = read_line(fd);
        if (line.empty()) return false;
        if (wbr_gps::snapshot_from_json(line, out)) return true;
    }
    return false;
}

static size_t open_fd_count()
{
    size_t n = 0;
    DIR* d = opendir("/proc/self/fd");
    if (!d) return 0;
    while (readdir(d)) ++n;
    closedir(d);
    return n;
}

// ---------- scenarios ----------

static void S1_single_get()
{
    PtyPair pty; ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);
    pty.emit(kGGA); usleep(800000);

    const wbr_gps::Snapshot s = wbr_gps::get_once(d.sock, 1000);
    ASSERT_TRUE(s.service_ok);
    ASSERT_TRUE(s.has_fix);
    ASSERT_NEAR(s.lat, 13.0028260, 1e-6);
    d.stop();
}

static void S2_ten_simultaneous_gets_agree()
{
    PtyPair pty; ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);
    pty.emit(kGGA); usleep(800000);

    std::vector<wbr_gps::Snapshot> results(10);
    std::vector<std::thread> ts;
    for (int i = 0; i < 10; ++i) {
        ts.emplace_back([&, i] { results[i] = wbr_gps::get_once(d.sock, 2000); });
    }
    for (auto& t : ts) t.join();

    // Every client must succeed and see the SAME position. Under the old
    // scheme they raced for bytes and disagreed.
    for (const auto& r : results) {
        ASSERT_TRUE(r.service_ok);
        ASSERT_TRUE(r.has_fix);
        ASSERT_NEAR(r.lat, results[0].lat, 1e-12);
        ASSERT_NEAR(r.lon, results[0].lon, 1e-12);
        ASSERT_EQ(r.seq, results[0].seq);
    }
    d.stop();
}

static void S3_fifty_watchers_all_receive()
{
    PtyPair pty; ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    std::vector<int> fds;
    for (int i = 0; i < 50; ++i) {
        const int fd = raw_connect(d.sock);
        ASSERT_TRUE(fd >= 0);
        send_line(fd, "{\"op\":\"watch\",\"fix\":true,\"nmea\":false}");
        fds.push_back(fd);
    }
    usleep(400000);
    pty.emit(kGGA);
    usleep(1200000);

    int with_fix = 0;
    for (const int fd : fds) {
        wbr_gps::Snapshot s;
        // Drain until we see a FIX carrying has_fix.
        for (int i = 0; i < 10; ++i) {
            if (!read_fix(fd, s, 2)) break;
            if (s.has_fix) { ++with_fix; break; }
        }
    }
    ASSERT_EQ(with_fix, 50);

    for (const int fd : fds) ::close(fd);
    d.stop();
}

static void S4_mixed_modes_concurrently()
{
    PtyPair pty; ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    const int fix_watcher  = raw_connect(d.sock);
    const int nmea_watcher = raw_connect(d.sock);
    ASSERT_TRUE(fix_watcher >= 0 && nmea_watcher >= 0);
    send_line(fix_watcher,  "{\"op\":\"watch\",\"fix\":true,\"nmea\":false}");
    send_line(nmea_watcher, "{\"op\":\"watch\",\"fix\":false,\"nmea\":true}");
    usleep(300000);

    std::atomic<bool> get_ok{ false };
    std::thread getter([&] { get_ok = wbr_gps::get_once(d.sock, 2000).service_ok; });

    pty.emit(kGGA);
    usleep(1000000);
    getter.join();

    ASSERT_TRUE(get_ok.load());

    wbr_gps::Snapshot s;
    ASSERT_TRUE(read_fix(fix_watcher, s, 8));

    // The NMEA watcher must get the verbatim sentence.
    bool saw_raw = false;
    for (int i = 0; i < 8 && !saw_raw; ++i) {
        const std::string line = read_line(nmea_watcher);
        std::string cls, raw;
        if (wbr_gps::json_get_string(line, "class", cls) && cls == "NMEA" &&
            wbr_gps::json_get_string(line, "raw", raw) && raw == kGGA) {
            saw_raw = true;
        }
    }
    ASSERT_TRUE(saw_raw);

    ::close(fix_watcher); ::close(nmea_watcher);
    d.stop();
}

static void S5_late_joiner_gets_snapshot_immediately()
{
    PtyPair pty; ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);
    pty.emit(kGGA);
    usleep(900000);

    // Join well after the last sentence. We must get state at once, not wait
    // for the next GGA.
    const int64_t t0 = wbr_gps::now_mono_ms();
    const int fd = raw_connect(d.sock);
    send_line(fd, "{\"op\":\"watch\",\"fix\":true,\"nmea\":false}");
    wbr_gps::Snapshot s;
    ASSERT_TRUE(read_fix(fd, s, 4));
    const int64_t elapsed = wbr_gps::now_mono_ms() - t0;

    ASSERT_TRUE(s.has_fix);
    ASSERT_TRUE(elapsed < 500);
    ::close(fd);
    d.stop();
}

static void S6_slow_client_does_not_starve_others()
{
    PtyPair pty; ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    // A watcher that subscribes and then never reads a byte.
    const int slow = raw_connect(d.sock);
    ASSERT_TRUE(slow >= 0);
    send_line(slow, "{\"op\":\"watch\",\"fix\":true,\"nmea\":true}");

    // A healthy watcher.
    const int fast = raw_connect(d.sock);
    ASSERT_TRUE(fast >= 0);
    send_line(fast, "{\"op\":\"watch\",\"fix\":true,\"nmea\":false}");
    usleep(300000);

    // Hammer the daemon while `slow` reads nothing.
    for (int i = 0; i < 400; ++i) { pty.emit(kGGA); usleep(2000); }
    usleep(500000);

    // The healthy client must still be served, and get_once must still work.
    wbr_gps::Snapshot s;
    ASSERT_TRUE(read_fix(fast, s, 20));
    ASSERT_TRUE(s.has_fix);
    ASSERT_TRUE(wbr_gps::get_once(d.sock, 2000).service_ok);

    ::close(slow); ::close(fast);
    d.stop();
}

static void S7_sigkilled_client_is_cleaned_up()
{
    PtyPair pty; ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    const pid_t child = fork();
    if (child == 0) {
        const int fd = raw_connect(d.sock);
        if (fd >= 0) send_line(fd, "{\"op\":\"watch\",\"fix\":true}");
        for (;;) pause();
    }
    usleep(500000);
    kill(child, SIGKILL);
    int st = 0; waitpid(child, &st, 0);
    usleep(500000);

    // The daemon must survive and keep serving.
    pty.emit(kGGA);
    usleep(700000);
    ASSERT_TRUE(wbr_gps::get_once(d.sock, 2000).has_fix);
    d.stop();
}

static void S8_disconnect_mid_write_does_not_kill_daemon()
{
    PtyPair pty; ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    for (int round = 0; round < 20; ++round) {
        const int fd = raw_connect(d.sock);
        if (fd < 0) continue;
        send_line(fd, "{\"op\":\"watch\",\"fix\":true,\"nmea\":true}");
        pty.emit(kGGA);
        ::close(fd);          // vanish while the daemon is writing
    }
    usleep(600000);

    // If SIGPIPE were not ignored, the daemon would be dead by now.
    ASSERT_TRUE(wbr_gps::get_once(d.sock, 2000).service_ok);
    d.stop();
}

static void S9_connect_storm_leaks_nothing()
{
    PtyPair pty; ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    const size_t before = open_fd_count();
    for (int i = 0; i < 200; ++i) {
        const int fd = raw_connect(d.sock);
        if (fd >= 0) { send_line(fd, "{\"op\":\"get\"}"); ::close(fd); }
    }
    usleep(700000);
    const size_t after = open_fd_count();

    // Our own fds must return to baseline.
    ASSERT_TRUE(after <= before + 2);
    // And the daemon must still be healthy.
    ASSERT_TRUE(wbr_gps::get_once(d.sock, 2000).service_ok);
    d.stop();
}

static void S10_malformed_input_is_survivable()
{
    PtyPair pty; ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    const int fd = raw_connect(d.sock);
    ASSERT_TRUE(fd >= 0);
    read_line(fd);                                   // HELLO
    send_line(fd, "not json at all");
    std::string cls;
    ASSERT_TRUE(wbr_gps::json_get_string(read_line(fd), "class", cls));
    ASSERT_STREQ(cls, "ERROR");
    ::close(fd);

    // Oversized request from a second client.
    const int fd2 = raw_connect(d.sock);
    send_line(fd2, std::string(9000, 'A'));
    usleep(300000);
    ::close(fd2);

    ASSERT_TRUE(wbr_gps::get_once(d.sock, 2000).service_ok);
    d.stop();
}

static void S11_unplug_is_seen_consistently_by_all()
{
    PtyPair pty; ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);
    pty.emit(kGGA);
    usleep(800000);

    std::vector<wbr_gps::Client*> clients;
    for (int i = 0; i < 5; ++i) {
        auto* c = new wbr_gps::Client();
        c->start(d.sock);
        clients.push_back(c);
    }
    usleep(600000);
    for (auto* c : clients) ASSERT_TRUE(c->snapshot().serial_ok);

    pty.close_pair();          // "unplug"
    usleep(2000000);

    // Every client must agree that the serial port is gone.
    for (auto* c : clients) ASSERT_FALSE(c->snapshot().serial_ok);
    // And they must still be talking to the daemon: a device failure is not
    // a service failure.
    for (auto* c : clients) ASSERT_TRUE(c->snapshot().service_ok);

    for (auto* c : clients) { c->stop(); delete c; }
    d.stop();
}

static void S12_daemon_death_leaves_device_untouched()
{
    // THE guarantee-G5 scenario. When the daemon dies, no client may grab the
    // device. We assert it by counting openers of the pty master's slave.
    PtyPair pty; ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    std::vector<wbr_gps::Client*> clients;
    for (int i = 0; i < 5; ++i) {
        auto* c = new wbr_gps::Client();
        c->start(d.sock);
        clients.push_back(c);
    }
    usleep(700000);
    for (auto* c : clients) ASSERT_TRUE(c->snapshot().service_ok);

    d.kill_hard();
    usleep(3000000);

    // Every client reports the service down, explicitly.
    for (auto* c : clients) ASSERT_FALSE(c->snapshot().service_ok);
    // And none has invented a fix from somewhere.
    for (auto* c : clients) ASSERT_FALSE(c->snapshot().has_fix);

    for (auto* c : clients) { c->stop(); delete c; }
}

static void S13_clients_reconnect_after_restart()
{
    PtyPair pty; ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    wbr_gps::Client c;
    c.start(d.sock);
    usleep(600000);
    ASSERT_TRUE(c.snapshot().service_ok);

    d.kill_hard();
    usleep(1500000);
    ASSERT_FALSE(c.snapshot().service_ok);

    d.spawn(pty.slave_path);
    usleep(3000000);
    ASSERT_TRUE(c.snapshot().service_ok);

    c.stop();
    d.stop();
}

static void S14_second_daemon_refuses_to_start()
{
    PtyPair pty; ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);

    const pid_t second = fork();
    if (second == 0) {
        execl(WBR_GPSD_PATH, "wbr-gpsd", "--socket", d.sock.c_str(),
              "--lock", d.lock.c_str(), "--serial", pty.slave_path.c_str(),
              "--group", "", (char*)nullptr);
        _exit(127);
    }
    int st = 0;
    waitpid(second, &st, 0);
    // Must exit non-zero rather than run alongside the first.
    ASSERT_TRUE(WIFEXITED(st));
    ASSERT_TRUE(WEXITSTATUS(st) != 0);

    // The original must be unharmed.
    ASSERT_TRUE(wbr_gps::get_once(d.sock, 2000).service_ok);
    d.stop();
}

static void S15_stale_socket_does_not_block_startup()
{
    PtyPair pty; ASSERT_TRUE(pty.open_pair());
    Daemon d; d.start(pty.slave_path);
    ASSERT_TRUE(wbr_gps::get_once(d.sock, 2000).service_ok);

    d.kill_hard();               // SIGKILL: no cleanup, socket file remains
    usleep(300000);

    d.spawn(pty.slave_path);     // must reclaim the stale socket
    ASSERT_TRUE(wbr_gps::get_once(d.sock, 3000).service_ok);
    d.stop();
}

static void run_tests()
{
    S1_single_get();
    S2_ten_simultaneous_gets_agree();
    S3_fifty_watchers_all_receive();
    S4_mixed_modes_concurrently();
    S5_late_joiner_gets_snapshot_immediately();
    S6_slow_client_does_not_starve_others();
    S7_sigkilled_client_is_cleaned_up();
    S8_disconnect_mid_write_does_not_kill_daemon();
    S9_connect_storm_leaks_nothing();
    S10_malformed_input_is_survivable();
    S11_unplug_is_seen_consistently_by_all();
    S12_daemon_death_leaves_device_untouched();
    S13_clients_reconnect_after_restart();
    S14_second_daemon_refuses_to_start();
    S15_stale_socket_does_not_block_startup();
}

TEST_MAIN
```

- [ ] **Step 4: Add the targets and run**

Append to `WBR-GPS/CMakeLists.txt`:

```cmake
add_executable(test_concurrency tests/test_concurrency.cpp)
target_include_directories(test_concurrency PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_compile_definitions(test_concurrency PRIVATE
  WBR_GPSD_PATH="${CMAKE_CURRENT_BINARY_DIR}/wbr-gpsd")
target_link_libraries(test_concurrency PRIVATE wbr_gps_core util pthread)
add_dependencies(test_concurrency wbr-gpsd)
add_test(NAME concurrency COMMAND test_concurrency)

add_test(NAME no_device_access
         COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_no_device_access.sh)
```

```bash
cd /home/sigint-4/prefix/src/WBR-GPS/build && cmake .. && make && ./test_concurrency
```

Expected: `0 failed`, exit 0. This takes roughly a minute because several scenarios wait on reconnect backoff.

- [ ] **Step 5: Run the whole suite and measure coverage**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS
cmake -S . -B build-cov -DWBR_GPS_COVERAGE=ON && make -C build-cov
cd build-cov && ctest --output-on-failure
lcov --capture --directory . --output-file cov.info \
  && lcov --remove cov.info '/usr/*' '*/tests/*' --output-file cov.info \
  && lcov --list cov.info
```

Expected: `ctest` all green; line coverage at or above 80% for `src/`. If it is short, the gap will be in `main.cpp`'s reconnect branches — add pty-level cases rather than lowering the target.

- [ ] **Step 6: Commit**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS
git add tests/test_concurrency.cpp tests/check_no_device_access.sh CMakeLists.txt
git commit -m "test: scenarios S1-S15 and structural G5 no-device-access gate"
```

---

## Task 13: Migrate the SIGINT GUI badge

First, because it is the worst offender: the only consumer that repeatedly opens and closes the port.

**Files:**
- Modify: `SIGINT_GUI/scanner/tasks.py:764-963` (the whole GPS block)
- Create: `SIGINT_GUI/scanner/test_gps_status.py`

**Interfaces:**
- Consumes: `wbr_gps_client.get_once`, `wbr_gps_client.badge_status`
- Produces: `tasks.get_gps_status() -> dict` — unchanged signature and unchanged `{"gps_status": <str>}` shape, so `views.gps_status` and `Base.html` need no edits

- [ ] **Step 1: Make the client importable**

```bash
cd /home/sigint-4/prefix/src/SIGINT_GUI
ln -sf ../../WBR-GPS/python/wbr_gps_client.py scanner/wbr_gps_client.py
ls -l scanner/wbr_gps_client.py
```

- [ ] **Step 2: Write the failing test**

Create `SIGINT_GUI/scanner/test_gps_status.py`:

```python
"""The badge must report the daemon's state, and must never fall back to
opening the GPS device itself."""
import unittest
from unittest import mock

from scanner import tasks


class TestGpsStatus(unittest.TestCase):
    def test_locked(self):
        snap = {"service_ok": True, "device_present": True,
                "gpsdo_locked": True, "has_fix": True}
        with mock.patch.object(tasks.wbr_gps_client, "get_once", return_value=snap):
            self.assertEqual(tasks.get_gps_status(), {"gps_status": "Locked"})

    def test_not_locked(self):
        snap = {"service_ok": True, "device_present": True,
                "gpsdo_locked": False, "has_fix": False}
        with mock.patch.object(tasks.wbr_gps_client, "get_once", return_value=snap):
            self.assertEqual(tasks.get_gps_status(), {"gps_status": "Not Locked"})

    def test_not_connected(self):
        snap = {"service_ok": True, "device_present": False,
                "gpsdo_locked": False, "has_fix": False}
        with mock.patch.object(tasks.wbr_gps_client, "get_once", return_value=snap):
            self.assertEqual(tasks.get_gps_status(), {"gps_status": "Not Connected"})

    def test_service_down_is_reported_honestly(self):
        # The old code reported "Not Connected" here, which was a guess.
        with mock.patch.object(tasks.wbr_gps_client, "get_once",
                               return_value={"service_ok": False}):
            self.assertEqual(tasks.get_gps_status(),
                             {"gps_status": "GPS Service Down"})

    def test_never_raises(self):
        with mock.patch.object(tasks.wbr_gps_client, "get_once",
                               side_effect=OSError("boom")):
            self.assertIn("gps_status", tasks.get_gps_status())

    def test_no_serial_module_is_imported(self):
        # Structural: the module must no longer reach for pyserial or udevadm.
        import inspect
        src = inspect.getsource(tasks)
        gps_block = src[src.find("# ============ GPS"):]
        self.assertNotIn("serial.Serial", gps_block)
        self.assertNotIn("udevadm", gps_block)
        self.assertNotIn("/dev/hidraw", gps_block)

    def test_old_helpers_are_gone(self):
        for name in ("check_gps_fix_on_port", "find_leobodnar_hid",
                     "find_leobodnar_serial_port", "check_hid_lock_status",
                     "check_leobodnar_sync_status", "_refresh_gps_async"):
            self.assertFalse(hasattr(tasks, name), f"{name} should be deleted")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 3: Run to verify it fails**

```bash
cd /home/sigint-4/prefix/src/SIGINT_GUI && python3 -m pytest scanner/test_gps_status.py -v
```

Expected: failures — `tasks` has no attribute `wbr_gps_client`, and the old helpers still exist.

- [ ] **Step 4: Replace the GPS block**

In `SIGINT_GUI/scanner/tasks.py`, delete everything from the `# ============ GPS (Leo Bodnar GPSDO) status ============` banner (line 764) through the end of `get_gps_status()` (line 963) — that is `LEO_VENDOR_STR`, `parse_gga_fix_status`, `find_leobodnar_serial_port`, `check_gps_fix_on_port`, `find_leobodnar_hid`, `check_hid_lock_status`, `check_leobodnar_sync_status`, `_GPS_CACHE`, `_GPS_TTL`, `_GPS_REFRESH_LOCK`, `_GPS_REFRESHING`, `_gps_probe`, `_refresh_gps_async` and `get_gps_status` — and replace with:

```python
# ============ GPS (Leo Bodnar GPSDO) status ============
# All GPS state comes from wbr-gpsd, the single owner of the device. This
# module does NOT open the serial port or hidraw: four processes reading one
# tty is what made this badge undependable, and the daemon exists to end it.
#
# No cache is needed any more. The old probe shelled out to udevadm and
# blocked on a serial read, so it needed a 3 s TTL and a background refresh
# thread. A Unix-socket round trip is sub-millisecond, so the request thread
# can simply ask.

from scanner import wbr_gps_client


def get_gps_status():
    """GPS lock status for the header badge: {'gps_status': <state>}.

    States: 'Locked', 'Not Locked', 'Not Connected', 'GPS Service Down'.
    The fourth is new: previously an unreachable device was reported as
    'Not Connected', which was a guess rather than a fact.
    """
    try:
        snap = wbr_gps_client.get_once(timeout=0.5)
    except Exception:                        # never break the header
        return {"gps_status": "GPS Service Down"}
    return {"gps_status": wbr_gps_client.badge_status(snap)}


def get_gps_position():
    """Full GPS snapshot for callers that need position, not just the badge.

    Returns the dict from wbr_gps_client.get_once(); check 'service_ok' and
    'has_fix' before trusting 'lat'/'lon'.
    """
    try:
        return wbr_gps_client.get_once(timeout=0.5)
    except Exception:
        return wbr_gps_client.unavailable()
```

Then remove the now-unused imports if nothing else in the file uses them:

```bash
cd /home/sigint-4/prefix/src/SIGINT_GUI
grep -n "serial\.\|import serial\|glob\.\|udevadm" scanner/tasks.py
```

Drop `import serial` (and `glob` if unreferenced) only when that grep shows no remaining uses.

- [ ] **Step 5: Add the new badge colour**

In `SIGINT_GUI/scanner/templates/scanner/Base.html`, `fetchGPSStatus()` around line 532 handles only `'Not Locked'` and `'Locked'`. Add the new state so it is visually distinct rather than falling through to the default:

```javascript
                    status.innerText = data["gps_status"];
                    if (data['gps_status'] == 'Not Locked') {
                        status.style.color = 'orange';
                    }
                    else if (data['gps_status'] == 'Locked') {
                        status.style.color = 'lime';
                    }
                    else if (data['gps_status'] == 'GPS Service Down') {
                        // wbr-gpsd is not running. Distinct from "no device":
                        // the operator needs to restart the service, not
                        // check the cable.
                        status.style.color = 'red';
                    }
```

- [ ] **Step 6: Run the tests**

```bash
cd /home/sigint-4/prefix/src/SIGINT_GUI && python3 -m pytest scanner/test_gps_status.py -v
```

Expected: 7 passed.

- [ ] **Step 7: Verify against the live daemon**

```bash
sudo systemctl start wbr-gpsd
cd /home/sigint-4/prefix/src/SIGINT_GUI
python3 -c "
from scanner import tasks
import time
for _ in range(5):
    print(tasks.get_gps_status()); time.sleep(1)
"
```

Expected: five identical, stable readings. Previously this flapped.

- [ ] **Step 8: Commit**

```bash
cd /home/sigint-4/prefix/src/SIGINT_GUI
git add scanner/tasks.py scanner/wbr_gps_client.py scanner/test_gps_status.py scanner/templates/scanner/Base.html
git commit -m "refactor: GPS badge reads wbr-gpsd instead of the device directly

Removes ~150 lines: the pyserial probe, the udevadm hidraw scan, and the
cache and background-refresh thread that existed only to hide their cost.
Adds a fourth honest state, GPS Service Down."
```

---

## Task 14: Migrate WBR-SA (`phase2_server`)

**Files:**
- Delete: `WBR-SA/src/gps_reader.cpp`, `WBR-SA/include/gps_reader.hpp`
- Modify: `WBR-SA/CMakeLists.txt`, `WBR-SA/src/web_server.cpp`, `WBR-SA/src/phase2_server.cpp:57-59`

**Interfaces:**
- Consumes: `wbr_gps::Client`, `wbr_gps::Snapshot`
- Produces: no public interface change — `/api/deployment` keeps returning `{lat, lon, has_position}`

- [ ] **Step 1: Find every use of the old reader**

```bash
cd /home/sigint-4/prefix/src/WBR-SA
grep -rn "GPSReader\|gps_reader\|GPSFix\|gps_device\|gps_baud\|gps_auto_discover" \
  --include=*.cpp --include=*.hpp src/ include/ CMakeLists.txt
```

Record every hit; each one must be updated or deleted in Step 3.

- [ ] **Step 2: Link WBR-GPS**

In `WBR-SA/CMakeLists.txt`, remove `src/gps_reader.cpp` from the sources and add:

```cmake
# GPS comes from wbr-gpsd via its client library; this project no longer
# opens the device itself.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../WBR-GPS
                 ${CMAKE_CURRENT_BINARY_DIR}/wbr-gps EXCLUDE_FROM_ALL)
target_link_libraries(phase2_server PRIVATE wbr_gps_core)
target_include_directories(phase2_server PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/../WBR-GPS/include)
```

- [ ] **Step 3: Swap the type at each call site**

Replace the `specan::GPSReader` member and its uses:

| Before | After |
|---|---|
| `#include "gps_reader.hpp"` | `#include "wbr_gps/client.hpp"` |
| `specan::GPSReader gps_;` | `wbr_gps::Client gps_;` |
| `gps_.start(cfg)` | `gps_.start()` (default socket path) |
| `gps_.stop()` | `gps_.stop()` (unchanged) |
| `specan::GPSFix f = gps_.latest();` | `wbr_gps::Snapshot f = gps_.snapshot();` |
| `f.has_fix` | `f.has_fix` (unchanged) |
| `f.altitude_m` | `f.alt_m` |
| `f.locked` | `f.gpsdo_locked` |
| `gps_.age_seconds()` | `f.fix_age_ms / 1000.0` |
| `gps_.device_path()` | *(remove — clients have no device path)* |

For the `/api/deployment` handler in `web_server.cpp`, `has_position` should now require the service to be up as well, so a dead daemon does not publish a stale pin to other sites:

```cpp
const wbr_gps::Snapshot f = gps_.snapshot();
const bool has_position = f.service_ok && f.has_fix;
```

- [ ] **Step 4: Retire the CLI flags**

`phase2_server.cpp:57-59` parses `--gps`, `--gps-baud` and `--no-gps`. The device path and baud are now the daemon's business. Keep the flags accepted-but-ignored with a warning, so `run_rtsa.sh` and any operator muscle memory do not break:

```cpp
else if (k == "--gps" || k == "--gps-baud") {
    need(k.c_str());   // consume the value
    std::fprintf(stderr,
        "[phase2] %s is ignored: GPS is served by wbr-gpsd. "
        "Configure the device in /etc/systemd/system/wbr-gpsd.service\n",
        k.c_str());
}
else if (k == "--no-gps") { wopts.gps_enabled = false; }
```

`gps_enabled` is a new field. The old struct carried `gps_device`, `gps_baud`
and `gps_auto_discover`; all three become meaningless once the daemon owns the
device, so replace them with a single bool in the same options struct:

```cpp
    bool gps_enabled = true;   // was: gps_device / gps_baud / gps_auto_discover
```

`--no-gps` keeps working: it simply means "do not start a `wbr_gps::Client`".
When `gps_enabled` is false, skip `gps_.start()` and every `snapshot()` call
site sees a default `Snapshot` with `service_ok = false`.

- [ ] **Step 5: Delete the old reader and build**

```bash
cd /home/sigint-4/prefix/src/WBR-SA
git rm src/gps_reader.cpp include/gps_reader.hpp
cmake -S . -B build && make -C build phase2_server 2>&1 | tail -30
```

Expected: clean build. Any error naming `GPSFix` or `GPSReader` is a call site missed in Step 1.

- [ ] **Step 6: Verify end to end**

```bash
sudo systemctl start wbr-gpsd
cd /home/sigint-4/prefix/src/WBR-SA && ./run_rtsa.sh &
sleep 8
curl -s localhost:8090/api/deployment
lsof "$(readlink -f /dev/gpsdo)"
```

Expected: the endpoint returns real `lat`/`lon` with `has_position: true`, and `lsof` still shows **exactly one** opener, `wbr-gpsd` — `phase2_server` must not appear.

- [ ] **Step 7: Commit**

```bash
cd /home/sigint-4/prefix/src/WBR-SA
git add -A
git commit -m "refactor: RTSA reads GPS from wbr-gpsd instead of owning the device"
```

---

## Task 15: Migrate WBR-GSM (`gsm_monitor`)

**Files:**
- Delete: `WBR-GSM/gps_source.h`
- Modify: `WBR-GSM/gsm_monitor.cpp:772`, `:1354-1357`, `:2198`, `WBR-GSM/CMakeLists.txt`

**Interfaces:**
- Consumes: `wbr_gps::Client`, `wbr_gps::get_once`
- Produces: no interface change — the display and track log read the same fields

- [ ] **Step 1: Find every use**

```bash
cd /home/sigint-4/prefix/src/WBR-GSM
grep -rn "gps_source.h\|gps_state_t\|gps_thread_run\|gps_get_data\|gps_data_t\|gps_autodetect_port\|leo_bodnar_usb_present" \
  --include=*.cpp --include=*.h . | grep -v build/
```

- [ ] **Step 2: Link WBR-GPS**

In `WBR-GSM/CMakeLists.txt`:

```cmake
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../WBR-GPS
                 ${CMAKE_CURRENT_BINARY_DIR}/wbr-gps EXCLUDE_FROM_ALL)
target_link_libraries(gsm_monitor PRIVATE wbr_gps_core)
target_include_directories(gsm_monitor PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/../WBR-GPS/include)
```

Note the standard mismatch: WBR-GSM is C++20 and WBR-GPS is C++17. This is fine — CMake compiles each target at its own standard, and the client header is valid in both. Do not raise WBR-GPS to C++20; WBR-SA needs it at 17.

- [ ] **Step 3: Replace the external-reference check at line 772**

The current code scans `/dev/serial/by-id` itself:

```cpp
if (a.clock_src.empty() && a.clock_autodetect && driver_is_bladerf && leo_bodnar_usb_present()) {
```

becomes a single synchronous ask, before any long-lived client exists:

```cpp
// Ref-present now comes from wbr-gpsd rather than a second by-id scan.
// If the daemon is down we must NOT guess a 10 MHz reference is wired in:
// get_once() returns device_present = false, which is the safe default.
if (a.clock_src.empty() && a.clock_autodetect && driver_is_bladerf &&
    wbr_gps::get_once().device_present) {
```

- [ ] **Step 4: Replace the reader at lines 1354-1357**

```cpp
    gps_state_t gps_st;
    gps_st.port_path = (a.gps_port == "auto") ? gps_autodetect_port() : a.gps_port;
    ...
    std::thread gps_thread([&] { gps_thread_run(&gps_st); });
```

becomes:

```cpp
    // wbr-gpsd owns the device; this client keeps a local cache fresh on its
    // own thread, exactly as gps_thread_run() used to.
    wbr_gps::Client gps_client;
    gps_client.start();
```

Delete the corresponding `gps_st.stop.store(true); gps_thread.join();` at shutdown and replace with `gps_client.stop();`.

- [ ] **Step 5: Replace the hot-path read at line 2198**

```cpp
        gps_data_t gps = gps_get_data(&gps_st);
```

becomes:

```cpp
        // Still a plain cached read, no syscall: safe in the per-chunk loop.
        wbr_gps::Snapshot gps = gps_client.snapshot();
```

Then fix the field names at each downstream use: `gps.lock` becomes `gps.fix_quality`, `gps.speed` becomes `gps.speed_kph`, `gps.alt` becomes `gps.alt_m`. `gps.lat` and `gps.lon` are unchanged. Note one behaviour change worth knowing: the old `gps_get_data` returned `-1.0` sentinels for an invalid fix, whereas `Snapshot` keeps the last known position and reports `has_fix = false`. Any downstream code testing `lat < 0` as "no fix" must test `!gps.has_fix` instead:

```bash
grep -n "gps\.\(lat\|lon\|alt\|speed\|lock\)" gsm_monitor.cpp gsm_monitor_display.h track_log.h
```

Check each hit for a `-1` sentinel comparison and convert it.

- [ ] **Step 6: Handle the `--gps` flag at line 586**

Keep it accepted and ignored, with a warning, matching Task 14 Step 4.

- [ ] **Step 7: Delete the old source and build**

```bash
cd /home/sigint-4/prefix/src/WBR-GSM
git rm gps_source.h
cmake -S . -B build && make -C build gsm_monitor 2>&1 | tail -30
```

Expected: clean build.

- [ ] **Step 8: Verify**

```bash
sudo systemctl start wbr-gpsd
cd /home/sigint-4/prefix/src/WBR-GSM/build && ./gsm_monitor --help >/dev/null && echo "runs"
# Then a short live run, and confirm exclusivity holds:
lsof "$(readlink -f /dev/gpsdo)"
```

Expected: one opener, `wbr-gpsd`.

- [ ] **Step 9: Commit**

```bash
cd /home/sigint-4/prefix/src/WBR-GSM
git add -A
git commit -m "refactor: gsm_monitor reads GPS and ref-present from wbr-gpsd"
```

---

## Task 16: Migrate `gps_capture` to an NMEA subscriber

**Files:**
- Modify: `WBR-GSM/gps_capture.cpp` (rewrite), `WBR-GSM/CMakeLists.txt`

**Interfaces:**
- Consumes: `wbr_gps::Client` with `want_nmea = true`
- Produces: the same `gps_nmea.log` output and the same CLI surface

- [ ] **Step 1: Rewrite as a subscriber**

Replace the body of `WBR-GSM/gps_capture.cpp`:

```cpp
// Capture raw NMEA from wbr-gpsd. This program used to open the serial port
// itself, making it a second owner; it is now a subscriber.
#include "wbr_gps/client.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

static std::atomic<bool> g_stop{ false };
static void on_signal(int) { g_stop.store(true); }

static void usage(const char* argv0)
{
    std::fprintf(stderr,
        "Usage: %s [--out FILE] [--lines N] [--socket PATH]\n"
        "  --gps and --baud are accepted and ignored: the device is owned by\n"
        "  wbr-gpsd. Configure it in the wbr-gpsd systemd unit.\n", argv0);
}

int main(int argc, char** argv)
{
    std::string out_path  = "gps_nmea.log";
    std::string sock_path = wbr_gps::kDefaultSocketPath;
    int         max_lines = -1;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* n) -> const char* {
            if (i + 1 >= argc) { std::fprintf(stderr, "Missing value for %s\n", n); std::exit(1); }
            return argv[++i];
        };
        if      (a == "--out")    out_path  = next("--out");
        else if (a == "--socket") sock_path = next("--socket");
        else if (a == "--lines")  max_lines = std::atoi(next("--lines"));
        else if (a == "--gps" || a == "--baud") {
            next(a.c_str());
            std::fprintf(stderr, "%s ignored: the device is owned by wbr-gpsd\n", a.c_str());
        }
        else if (a == "--help" || a == "-h") { usage(argv[0]); return 0; }
        else { std::fprintf(stderr, "Unknown argument: %s\n", a.c_str()); usage(argv[0]); return 1; }
    }

    FILE* out = std::fopen(out_path.c_str(), "w");
    if (!out) {
        std::fprintf(stderr, "Failed to open output file %s\n", out_path.c_str());
        return 1;
    }

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    std::atomic<int> n_lines{ 0 };
    wbr_gps::Client client;
    client.set_nmea_callback([&](const std::string& raw) {
        std::fprintf(out, "%s\n", raw.c_str());
        std::fflush(out);
        std::printf("%s\n", raw.c_str());
        const int n = ++n_lines;
        if (max_lines > 0 && n >= max_lines) g_stop.store(true);
    });

    if (!client.start(sock_path, /*want_nmea=*/true)) {
        std::fprintf(stderr, "Failed to start GPS client\n");
        std::fclose(out);
        return 1;
    }
    std::fprintf(stderr, "Capturing NMEA from wbr-gpsd (%s) to %s\n",
                 sock_path.c_str(), out_path.c_str());

    bool warned = false;
    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // Say so loudly if the service is down, rather than sitting silent.
        if (!client.snapshot().service_ok && !warned) {
            std::fprintf(stderr, "wbr-gpsd is not reachable; waiting...\n");
            warned = true;
        } else if (client.snapshot().service_ok) {
            warned = false;
        }
    }

    client.stop();
    std::fclose(out);
    std::printf("Captured %d lines\n", n_lines.load());
    return 0;
}
```

- [ ] **Step 2: Build and verify**

```bash
cd /home/sigint-4/prefix/src/WBR-GSM && make -C build gps_capture
sudo systemctl start wbr-gpsd
./build/gps_capture --out /tmp/nmea.log --lines 20
head -3 /tmp/nmea.log
lsof "$(readlink -f /dev/gpsdo)"
```

Expected: 20 real sentences captured; `lsof` shows one opener, `wbr-gpsd`.

- [ ] **Step 3: Commit**

```bash
cd /home/sigint-4/prefix/src/WBR-GSM
git add gps_capture.cpp CMakeLists.txt
git commit -m "refactor: gps_capture subscribes to wbr-gpsd instead of owning the port"
```

---

## Task 17: End-to-end regression, leak check and soak

The before/after proof, using the baseline measured on 2026-09-01.

**Files:**
- Create: `WBR-GPS/tests/regression_all_consumers.sh`
- Create: `WBR-GPS/tests/soak.sh`

- [ ] **Step 1: Write the regression script**

Create `WBR-GPS/tests/regression_all_consumers.sh`:

```bash
#!/usr/bin/env bash
# Layer 4: run every real consumer at once against wbr-gpsd and assert the
# original bug is gone.
#
# Pre-fix baseline, measured 2026-09-01 with three concurrent readers:
#   2 of 3 readers died with "multiple access on port?"
#   the survivor received interleaved, corrupted sentences
set -uo pipefail

SRC=/home/sigint-4/prefix/src
DEV="$(readlink -f /dev/gpsdo)"
FAILED=0

fail() { echo "FAIL: $*" >&2; FAILED=1; }
pass() { echo "PASS: $*"; }

echo "=== 1. daemon is up ==="
systemctl is-active --quiet wbr-gpsd || { fail "wbr-gpsd is not active"; exit 1; }
pass "wbr-gpsd active"

echo "=== 2. start every consumer ==="
"$SRC/WBR-GSM/build/gps_capture" --out /tmp/reg_nmea.log >/dev/null 2>&1 &
CAP=$!
( cd "$SRC/WBR-SA" && ./run_rtsa.sh >/tmp/reg_rtsa.log 2>&1 ) &
sleep 8

# Poll the GUI badge the way the header does.
python3 - <<'PY' > /tmp/reg_badge.txt 2>&1 &
import sys, time
sys.path.insert(0, "/home/sigint-4/prefix/src/SIGINT_GUI")
from scanner import wbr_gps_client
for _ in range(20):
    s = wbr_gps_client.get_once(timeout=0.5)
    print(wbr_gps_client.badge_status(s), s.get("lat"), s.get("lon"))
    time.sleep(3)
PY
BADGE=$!
sleep 60

echo "=== 3. exactly one opener of the device ==="
N=$(lsof -t "$DEV" 2>/dev/null | wc -l)
OWNER=$(lsof -F c "$DEV" 2>/dev/null | grep '^c' | head -1 | cut -c2-)
[[ "$N" -eq 1 ]] && pass "one opener" || fail "expected 1 opener, found $N"
[[ "$OWNER" == "wbr-gpsd" ]] && pass "owner is wbr-gpsd" || fail "owner is '$OWNER'"

echo "=== 4. zero torn sentences ==="
# Every captured line must start with '$' and carry a checksum.
TORN=$(grep -cvE '^\$[A-Z]{5},.*\*[0-9A-Fa-f]{2}$' /tmp/reg_nmea.log || true)
[[ "$TORN" -eq 0 ]] && pass "no torn sentences" || fail "$TORN torn sentences"

echo "=== 5. no SerialException anywhere ==="
if grep -qi "SerialException\|multiple access on port" /tmp/reg_*.log /tmp/reg_badge.txt 2>/dev/null; then
  fail "a consumer hit a serial access error"
else
  pass "no serial access errors"
fi

echo "=== 6. badge is stable ==="
DISTINCT=$(cut -d' ' -f1 /tmp/reg_badge.txt | sort -u | wc -l)
[[ "$DISTINCT" -eq 1 ]] && pass "badge stable across 20 polls" \
                        || fail "badge flapped between $DISTINCT states"

echo "=== 7. all consumers agree on position ==="
POS=$(cut -d' ' -f2,3 /tmp/reg_badge.txt | sort -u | wc -l)
[[ "$POS" -le 2 ]] && pass "position consistent" || fail "position varied $POS ways"

kill $CAP $BADGE 2>/dev/null
pkill -f phase2_server 2>/dev/null
exit $FAILED
```

- [ ] **Step 2: Run it**

```bash
chmod +x /home/sigint-4/prefix/src/WBR-GPS/tests/regression_all_consumers.sh
sudo /home/sigint-4/prefix/src/WBR-GPS/tests/regression_all_consumers.sh
```

Expected: all seven checks PASS, exit 0. Compare against the pre-fix baseline in the spec, §1.2 — where two of three readers died outright.

- [ ] **Step 3: Write the soak script**

Create `WBR-GPS/tests/soak.sh`:

```bash
#!/usr/bin/env bash
# Layer 5: long soak with clients attached. Watches for fd leaks, memory
# growth and crashes. Default 1 hour; pass a duration in seconds to change.
set -uo pipefail

DURATION="${1:-3600}"
DEV="$(readlink -f /dev/gpsdo)"
END=$(( $(date +%s) + DURATION ))

PID=$(systemctl show -p MainPID --value wbr-gpsd)
[[ "$PID" -gt 0 ]] || { echo "wbr-gpsd is not running" >&2; exit 1; }

FD0=$(ls /proc/"$PID"/fd 2>/dev/null | wc -l)
RSS0=$(awk '/VmRSS/{print $2}' /proc/"$PID"/status)
RESTARTS0=$(systemctl show -p NRestarts --value wbr-gpsd)
echo "baseline: pid=$PID fds=$FD0 rss=${RSS0}kB restarts=$RESTARTS0"

# Five long-lived watchers for the whole soak.
for i in $(seq 1 5); do
  ( while true; do
      printf '{"op":"watch","fix":true}\n'; sleep 3600
    done | socat - UNIX-CONNECT:/run/wbr-gps/gpsd.sock > /dev/null 2>&1 ) &
done

while [[ $(date +%s) -lt $END ]]; do
  sleep 60
  CUR=$(systemctl show -p MainPID --value wbr-gpsd)
  if [[ "$CUR" != "$PID" ]]; then
    echo "FAIL: daemon restarted (was $PID, now $CUR)" >&2
    exit 1
  fi
  FD=$(ls /proc/"$PID"/fd | wc -l)
  RSS=$(awk '/VmRSS/{print $2}' /proc/"$PID"/status)
  echo "$(date +%H:%M:%S) fds=$FD rss=${RSS}kB"
  # 5 watchers plus a handful of static fds; anything past 40 is a leak.
  if [[ "$FD" -gt 40 ]]; then echo "FAIL: fd leak ($FD)" >&2; exit 1; fi
  # RSS must not double.
  if [[ "$RSS" -gt $(( RSS0 * 2 )) ]]; then echo "FAIL: memory growth" >&2; exit 1; fi
done

kill %1 %2 %3 %4 %5 2>/dev/null
echo "PASS: soak completed with no leak, growth or restart"
```

- [ ] **Step 4: Run a short soak, then the full one**

```bash
chmod +x /home/sigint-4/prefix/src/WBR-GPS/tests/soak.sh
/home/sigint-4/prefix/src/WBR-GPS/tests/soak.sh 300     # 5 min smoke
/home/sigint-4/prefix/src/WBR-GPS/tests/soak.sh 3600    # 1 hour
```

- [ ] **Step 5: Hardware unplug/replug pass**

This cannot be covered by the pty harness (spec §7.6), so it is a manual gate. With the daemon running and a client watching:

```bash
socat - UNIX-CONNECT:/run/wbr-gps/gpsd.sock <<< '{"op":"watch","fix":true}' &
```

Then physically unplug the Leo Bodnar, wait 10 s, and replug.

Expected sequence in the output:
1. `"serial_ok":true,"device_present":true` before unplug
2. within ~2 s of unplug: `"device_present":false,"serial_ok":false`, `has_fix` goes false after the 10 s staleness window
3. within ~5 s of replug: `"device_present":true,"serial_ok":true`, then a real fix
4. `lsof "$(readlink -f /dev/gpsdo)"` still shows exactly one opener
5. `systemctl show -p NRestarts --value wbr-gpsd` is unchanged — the daemon rode it out rather than crashing

- [ ] **Step 6: Commit**

```bash
cd /home/sigint-4/prefix/src/WBR-GPS
git add tests/regression_all_consumers.sh tests/soak.sh
git commit -m "test: end-to-end regression, soak and hardware unplug acceptance"
```

---

## Appendix: spec coverage

| Spec section | Covered by |
|---|---|
| §2.1.1 Position and fix | Tasks 2, 4 |
| §2.1.2 GPSDO lock | Task 6 |
| §2.1.3 Raw NMEA passthrough | Tasks 7, 16 |
| §2.1.4 Ref-present | Tasks 6, 15 |
| §3.1 Single-threaded epoll | Task 8 |
| §3.2 TIOCEXCL exclusive ownership | Task 5, verified Task 8 Step 4 |
| §3.4 systemd lifecycle | Task 11 |
| §3.5 udev rules | Task 11 |
| §4 Wire protocol | Tasks 3, 7 |
| §4.4 Backpressure and coalescing | Task 7 |
| §4.5 Heartbeat | Task 8 |
| §5.1 Python client | Task 10 |
| §5.2 C++ client | Task 9 |
| §6.1 G1 single owner | Task 5 + Task 8 Step 4 + Task 17 |
| §6.1 G2/G3 no client blocks another | Task 7, Task 12 S6 |
| §6.1 G4 explicit unavailability | Tasks 4, 10 |
| §6.1 G5 no device fallback | Task 12 `check_no_device_access.sh`, S12 |
| §6.2 Per-signal degradation | Task 4 |
| §6.3 Failure matrix | Tasks 5, 6, 7, 12 |
| §6.4 SIGPIPE and CLOCK_MONOTONIC | Tasks 1, 8, Task 12 S8 |
| §7.1 Unit tests | Task 2 |
| §7.2 pty harness | Task 5 |
| §7.3 S1–S15 | Task 12 |
| §7.4 Regression | Task 17 |
| §7.5 Leak and soak | Task 17 |
| §7.6 Hardware limitation | Task 17 Step 5 |
| §8 Migration order | Tasks 13, 14, 15, 16 |
