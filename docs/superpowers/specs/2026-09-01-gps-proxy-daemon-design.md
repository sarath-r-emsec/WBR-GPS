# WBR-GPS: GPS Proxy Daemon — Design

**Date:** 2026-09-01
**Status:** Approved design, pending implementation plan
**Component:** `WBR-GPS/` (new, sibling of `WBR-GSM` and `WBR-SA`)

---

## 1. Problem

A single Leo Bodnar LBE-1421 GPSDO is shared by every program in the SIGINT
project. Four independent consumers each open the device themselves:

| Consumer | Code | Access pattern |
|---|---|---|
| `gsm_monitor` | `WBR-GSM/gps_source.h` → `gps_thread_run()` | Holds `ttyACM0` open for the whole run |
| `gps_capture` | `WBR-GSM/gps_capture.cpp` | Holds it open, logs raw NMEA |
| `phase2_server` (RTSA) | `WBR-SA/src/gps_reader.cpp` → `GPSReader` | Holds serial open + optional hidraw thread |
| SIGINT GUI badge | `SIGINT_GUI/scanner/tasks.py:886` | Opens/closes every 3 s, reads ≤30 lines, falls back to hidraw + `udevadm` |

A fifth, uninvited reader exists: `ModemManager` is `active` and `enabled`, and
`udevadm info -n /dev/ttyACM0` reports `ID_MM_CANDIDATE=1`, so ModemManager
opens the port and writes AT probe strings at it on every hotplug.

### 1.1 Root cause

A tty has **one shared input queue**. Every `read()` *consumes* bytes; readers
do not each receive a copy, they race for fragments. This is not a locking bug
that can be fixed in place — it is four processes reading one stream.

### 1.2 Reproduction (measured 2026-09-01)

Baseline, one reader for 6 s: `lines=37 gga=6` — GGA at 1 Hz, ~6 sentences/s.

Three concurrent readers for 6 s:

```
A: SerialException: device reports readiness to read but returned no data
   (device disconnected or multiple access on port?)          <- crashed
C: SerialException: ... multiple access on port?              <- crashed
B: lines=78 gga=13                                            <- survived, corrupted
```

Reader B's first sentence arrived as two sentences shredded together:

```
,,3,,5$G32,,,9151,,2,96*G3,,1,,,8,,6379$M055A3.5N7.500,*
```

This explains the undependable GPS badge precisely. `check_gps_fix_on_port`
either raises (returning `None`, silently falling through to the HID path) or
parses a torn line whose `fields[6]` is garbage.

### 1.3 Secondary problems

- **Three divergent NMEA parsers.** WBR-SA validates checksums; WBR-GSM does
  not. Each extracts a different subset of fields, with different bugs.
- **Failure is conflated with state.** "I could not read the device" and "the
  device is not locked" produce the same badge value, so a confident wrong
  answer is displayed.

---

## 2. Requirements

Any program must be able to obtain the GPS information it needs, at any time,
without contending with any other program.

### 2.1 Scope — all four signals

1. **Position and fix status** — lat/lon/alt/speed, fix quality, satellites, HDOP, UTC time
2. **GPSDO lock status** — the `/dev/hidraw2` lock bit, a signal genuinely distinct
   from NMEA fix (the LB clock can be locked without a current GGA)
3. **Raw NMEA passthrough** — verbatim sentence stream, so `gps_capture` becomes
   a client rather than a second port owner
4. **Reference-present** — "is a Leo Bodnar attached", replacing the
   `/dev/serial/by-id` scan at `gsm_monitor.cpp:772` that auto-selects the SDR's
   10 MHz external clock reference

### 2.2 Decisions taken

| Decision | Choice | Rationale |
|---|---|---|
| Transport | **Unix domain socket only** | All clients are on this host. Filesystem permissions, no port management, no network exposure of position. Remote access remains `phase2_server`'s `/api/deployment` over Tailscale, which already works (see `WBR-SA/docs/MULTISITE_ARCHITECTURE.md`). |
| Language | **C++ daemon; C++ and Python clients** | Matches WBR-GSM and WBR-SA. Reuses WBR-SA's checksum-validating parser instead of creating a fourth one. |
| Fallback | **None. Report unavailable; never touch the device** | Makes contention structurally impossible. Requires systemd supervision. |

### 2.3 Alternatives considered

**`gpsd`** — the canonical daemon for this problem, with mature hotplug,
reconnection, `libgps` and a Python module. Rejected because it models a *GPS
receiver*, not a *GPSDO*: it covers signals 1 and 3 but not 2 or 4. Adopting it
would mean running gpsd **plus** a second daemon for the HID half — two daemons,
one of them not under project control — and gpsd speaks TCP on 2947 rather than
the chosen Unix socket, and contends with ModemManager over `ttyACM*` exactly as
the current code does.

Revisit if non-Bodnar receivers or NTP/PPS time service are added; gpsd's
maturity would then outweigh the gap. Its `?WATCH={...}` subscribe model is
borrowed below regardless.

**Shared status file / shared memory** — simple and non-blocking, but cannot
stream, so requirement 3 is unsatisfiable and `gps_capture` would remain a
second device owner.

---

## 3. Architecture

### 3.1 Process model

One process, `wbr-gpsd`, **single-threaded on an `epoll` loop**. Everything it
watches is a file descriptor: the serial port, the hidraw device, the listening
socket, each connected client, and a `timerfd` for staleness and reconnect
ticks.

A single thread means **no mutexes and no shared mutable state inside the
daemon** — the class of race conditions being fixed cannot reappear in the fix.
The load justifies it: 13 NMEA sentences per second and a handful of clients.

### 3.2 Ownership

| Signal | Source | Mechanism |
|---|---|---|
| Position + fix | `/dev/gpsdo` (NMEA, 115200) | Opened with `TIOCEXCL` so a stray reader gets `EBUSY` rather than silently stealing bytes |
| GPSDO lock | `/dev/hidraw2` | Bit 0 of byte 1 |
| Raw NMEA | same serial stream | Fanned out verbatim to subscribers |
| Ref-present | `/dev/serial/by-id` scan | Replaces `gsm_monitor.cpp:772` |

### 3.3 File layout

Files kept to the project's 200–400 line guidance.

```
WBR-GPS/
├── CMakeLists.txt
├── include/wbr_gps/
│   ├── gps_types.hpp      # snapshot struct, shared daemon<->client
│   ├── nmea_parser.hpp    # pure functions, zero I/O — the testable core
│   └── client.hpp         # C++ client library
├── src/
│   ├── main.cpp           # args, signals, epoll loop
│   ├── serial_source.cpp  # owns the serial port, TIOCEXCL, reconnect backoff
│   ├── hid_source.cpp     # owns hidraw
│   ├── nmea_parser.cpp    # ported from WBR-SA's checksum-validated parser
│   ├── state_store.cpp    # the one authoritative snapshot + sequence counter
│   ├── server.cpp         # accept loop, per-client bounded output queues
│   └── client.cpp
├── python/wbr_gps_client.py
├── systemd/wbr-gpsd.service
└── tests/
```

The parser becomes **one implementation, not four**. `nmea_parser.cpp` is pure —
string in, struct out, no file descriptors — which makes every NMEA edge case
testable without hardware.

### 3.4 Lifecycle

systemd unit with `Restart=always`, `RuntimeDirectory=wbr-gps`, socket at
`/run/wbr-gps/gpsd.sock`, mode `0660`, group `dialout`. Every project program
already runs as `sigint-4`, who is in `dialout`. This supervision is what the
"never touch the device directly" decision requires.

### 3.5 udev changes

Extending the existing `/etc/udev/rules.d/99-leobodnar.rules`
(vendor `1dd2`, product `2444`):

```
# Stop ModemManager probing the GPSDO as a modem
SUBSYSTEM=="tty", ATTRS{idVendor}=="1dd2", ATTRS{idProduct}=="2444", ENV{ID_MM_DEVICE_IGNORE}="1"
# Stable symlink so the daemon never guesses a device path
SUBSYSTEM=="tty", ATTRS{idVendor}=="1dd2", SYMLINK+="gpsdo"
```

The first removes the fifth reader. The second survives `ttyACM0` → `ttyACM1`
renumbering across replug.

---

## 4. Wire protocol

Newline-delimited JSON, one object per line, both directions. Debuggable with
`socat - UNIX-CONNECT:/run/wbr-gps/gpsd.sock`. Traffic is 1 Hz, so encoding cost
is irrelevant.

### 4.1 Client to daemon

```json
{"op":"get"}                              // one snapshot, then the client may close
{"op":"watch","fix":true,"nmea":false}    // subscribe to pushes
{"op":"unwatch"}
```

### 4.2 Daemon to client

```json
{"class":"HELLO","proto":1,"daemon":"wbr-gpsd/1.0"}
{"class":"FIX", ...}
{"class":"NMEA","raw":"$GNGGA,045519.50,...*6F","t_unix_ms":1756701234567}
{"class":"ERROR","msg":"..."}
```

### 4.3 The FIX object

```json
{"class":"FIX","seq":12345,
 "device_present":true,     // Leo Bodnar on USB at all      -> replaces gsm_monitor.cpp:772
 "serial_ok":true,          // daemon actually holds the serial port
 "hid_ok":true,             // daemon actually holds hidraw
 "gpsdo_locked":true,       // HID lock bit — separate signal from fix
 "has_fix":true, "fix_quality":1, "satellites":4, "hdop":1.33,
 "lat":13.00282, "lon":77.67992, "alt_m":921.8, "speed_kph":0.0,
 "time_utc":"045519.50", "fix_age_ms":480}
```

Splitting `device_present` / `serial_ok` / `gpsdo_locked` / `has_fix` into four
independent booleans is what makes the status *dependable* rather than merely
*unlocked*. The GUI gains an honest fourth state — "GPS service down" — instead
of silently reporting a wrong one.

`seq` lets clients detect gaps. `fix_age_ms` lets each client choose its own
staleness tolerance.

### 4.4 Backpressure

If a client stops reading, the daemon must never block on `write()`. Each client
has a bounded output queue. On overflow:

- **FIX** messages **coalesce** — discard all but the newest, since only the
  latest snapshot has meaning
- **NMEA** messages drop oldest and increment a `"dropped":N` field so the
  client knows it missed sentences

One slow client must never degrade the others. Without this rule the contention
problem is simply rebuilt one layer up.

### 4.5 Heartbeat

The daemon emits a FIX at least every 2 s even when nothing has changed, so a
client detects a dead daemon by silence rather than by waiting indefinitely.

---

## 5. Client libraries

### 5.1 Python

```python
from wbr_gps_client import get_once
snap = get_once(timeout=0.5)      # connect, one line, close
```

This removes roughly 150 lines from `SIGINT_GUI/scanner/tasks.py`:
`find_leobodnar_hid`, `find_leobodnar_serial_port`, `check_gps_fix_on_port`,
`check_hid_lock_status`, `check_leobodnar_sync_status`, **and** the entire
`_GPS_CACHE` / `_GPS_TTL` / `_refresh_gps_async` machinery. That cache exists
only because the probe shells out to `udevadm` and blocks on a serial read; a
Unix-socket round trip is sub-millisecond, so the request thread can simply ask.

### 5.2 C++

```cpp
wbr_gps::Client c;
c.start();                              // background thread keeps a local cache fresh
wbr_gps::Snapshot s = c.snapshot();     // lock-free read of the cache, no syscall
wbr_gps::Snapshot s = wbr_gps::get_once();   // synchronous, for startup checks
```

Designed as a drop-in so migration diffs stay small. `gsm_monitor` already spawns
`gps_thread` and calls `gps_get_data(&gps_st)` at line 2198, so migration is
mechanical:

| Before | After |
|---|---|
| `gps_state_t gps_st` + `gps_thread_run` | `wbr_gps::Client c; c.start();` |
| `gps_get_data(&gps_st)` | `c.snapshot()` |
| `leo_bodnar_usb_present()` (line 772) | `wbr_gps::get_once().device_present` |

The hot-path call remains a plain memory read, exactly as today.

---

## 6. Failure handling

### 6.1 Guarantees

| | Guarantee | Enforced by |
|---|---|---|
| **G1** | Exactly one process ever opens the serial port | `TIOCEXCL` + `flock()` singleton pidfile |
| **G2** | No client can block another | Bounded per-client queues, non-blocking writes |
| **G3** | No client can block the daemon | Single epoll loop that never blocks on a client fd |
| **G4** | Unavailability is explicit, never silently wrong | Separate `serial_ok` / `hid_ok` / `device_present` flags |
| **G5** | No program ever falls back to the raw device | Client libraries contain no device code at all |

G5 is a **structural** property: the client libraries will contain no `open()`,
no `termios`, and no `/dev` path. A fallback cannot be added later by accident
because the code to do it does not exist in those files.

### 6.2 Degradation is per-signal

If `hidraw` is missing or permission-denied, `hid_ok` goes false and everything
else keeps working. If the serial port dies while the device remains on USB,
`serial_ok` goes false while `device_present` stays true. Today, one failure
poisons the whole answer.

### 6.3 Failure matrix

| Failure | Daemon response | Client observes |
|---|---|---|
| GPS unplugged | Close fd, retry open, 250 ms → 5 s backoff | `device_present:false`, stays connected |
| GPS replugged as new `ttyACMn` | Reopens via the stable `/dev/gpsdo` symlink | `serial_ok:true` again |
| No satellite fix (indoors) | Normal operation | `has_fix:false`; `gpsdo_locked` may still be true |
| Client killed with SIGKILL | `EPOLLHUP` → close fd, free queue | Others unaffected |
| Client stops reading | Queue caps; FIX coalesces, NMEA drops oldest and counts | `"dropped":N` |
| Daemon crashes | systemd `Restart=always` | EOF → "unavailable" → auto-reconnect |
| Two daemons launched at once | `flock` loser exits with a clear message | One wins deterministically |
| Stale socket after unclean kill | `connect()` probe first; unlink only if refused | Clean start |
| Client sends garbage / oversized line | `ERROR` reply, line length capped, disconnect on abuse | — |

### 6.4 Two details that cause real outages

- **`SIGPIPE` must be ignored** and all writes use `MSG_NOSIGNAL`, or one
  disconnecting client kills the daemon.
- **All ages use `CLOCK_MONOTONIC`**, never wall time. `CLOCK_REALTIME` is used
  only for display timestamps. A GPS daemon that steps the system clock and then
  computes a negative fix age is a classic failure.

---

## 7. Test plan

Test style matches the existing projects: standalone `test_*.cpp` binaries with
hand-rolled `ASSERT_EQ` macros. No gtest, no Catch2, no new dependency.

### 7.1 Layer 1 — Unit tests, no hardware

`nmea_parser` is pure functions, so every edge case is testable in-process.
Fixtures come from real captured NMEA, including the corrupted line produced by
the reproduction in §1.2:

```
,,3,,5$G32,,,9151,,2,96*G3,,1,,,8,,6379$M055A3.5N7.500,*
```

That exact string is a regression test — the parser must reject it.

Cases: valid GGA/RMC; `$GP` vs `$GN` talkers; bad checksum; missing checksum;
empty fields; `fix_quality` 0/1/2; southern and western hemispheres (sign
handling); oversized lines; embedded NULs.

### 7.2 Layer 2 — Daemon integration on a pseudo-terminal

`openpty()` provides a fake serial device, so the full daemon runs in CI with no
Leo Bodnar attached and without root. Recorded NMEA is replayed at 1 Hz or
fast-forwarded. This is what makes the rest of the plan practical.

### 7.3 Layer 3 — Concurrency scenarios

| # | Scenario | Assertion |
|---|---|---|
| S1 | Single `get` | Baseline snapshot correct |
| S2 | 10 simultaneous `get` | All return identical `seq` and position |
| S3 | 50 simultaneous `watch` | Every client receives every FIX, zero drops |
| S4 | Mixed `get` + `watch fix` + `watch nmea` | All satisfied concurrently |
| S5 | Client connects mid-stream | Current snapshot delivered immediately, not after the next GGA |
| S6 | One client stops reading | Others unaffected; slow client's FIXes coalesce; daemon never blocks |
| S7 | Client SIGKILLed | Daemon cleans up; no fd leak; others unaffected |
| S8 | Client disconnects mid-write | No SIGPIPE death |
| S9 | 200 rapid connect/disconnect | No fd leak, no degradation |
| S10 | Garbage / oversized / partial JSON | `ERROR`, no crash |
| S11 | Device unplugged with N clients | All flip `serial_ok:false` consistently |
| S12 | Daemon killed with N clients | All report unavailable; `lsof /dev/ttyACM0` shows **zero** openers |
| S13 | Daemon restarts | All clients reconnect automatically |
| S14 | Two daemons launched together | Exactly one wins |
| S15 | Stale socket file | Clean start |

**S12 is the scenario that proves G5** — that no client quietly grabbed the
device when the daemon went away.

### 7.4 Layer 4 — Regression test for the original bug

Run the real trio (`gsm_monitor`, `phase2_server`, GUI poll loop) against the
daemon for 60 s and assert:

- zero torn sentences
- zero `SerialException`
- all three report the same position
- `lsof /dev/ttyACM0` shows exactly one opener, `wbr-gpsd`

The pre-fix baseline from §1.2 is captured, making this a genuine before/after.

### 7.5 Layer 5 — Resource leak and soak

`/proc/self/fd` count before and after 1000 connect/disconnect cycles. Then a
long soak with periodic unplug/replug, asserting no leak, no crash, and correct
state transitions throughout.

Coverage target: the project standard of 80%, measured with `gcov`/`lcov`. The
pure-parser split is what makes that reachable — the hard-to-reach code is I/O,
and Layer 2's pty harness covers most of it.

### 7.6 Known limitation

The pty harness tests the daemon's logic faithfully, but **a pty is not a USB
CDC-ACM device**. Unplug/replug behaviour, `TIOCEXCL` semantics and ModemManager
interaction can only be verified on the real LBE-1421. Layers 1–3 run in CI, but
S11, S12 and the soak require a hardware pass on the actual box before this is
trusted in the field.

---

## 8. Migration

Each consumer moves independently; the daemon can run alongside the old code
during transition.

| Order | Change | Removes |
|---|---|---|
| 1 | Deploy daemon + udev rules + systemd unit | ModemManager as a reader |
| 2 | `SIGINT_GUI/scanner/tasks.py` → `wbr_gps_client` | ~150 lines, the worst offender (open/close every 3 s) |
| 3 | `WBR-SA` `GPSReader` → `wbr_gps::Client` | `src/gps_reader.cpp`, `include/gps_reader.hpp` |
| 4 | `WBR-GSM` `gps_source.h` → `wbr_gps::Client` | `gps_source.h` parser and thread |
| 5 | `WBR-GSM/gps_capture.cpp` → `watch nmea` client | Its direct port ownership |

Step 2 first: it is the largest single reduction in contention, since it is the
only consumer that repeatedly opens and closes the port.

---

## 9. Open items

None. All decisions in §2.2 are settled.
