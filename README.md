# WBR-GPS

`wbr-gpsd` — a proxy daemon that owns the Leo Bodnar GPSDO and serves GPS to
every program in the SIGINT project at once.

## The problem it solves

One GPSDO, four programs that need it: the SIGINT GUI's status badge, RTSA
(`phase2_server`), `gsm_monitor`, and `gps_capture`.

A serial port is not a file. When two programs read a *file*, each gets its own
copy. `/dev/ttyACM0` has **one shared byte queue** — several readers race for
the same bytes and each ends up with a fragment. Measured on the real hardware
with three concurrent readers:

```
A: SerialException: ... multiple access on port?   crashed
C: SerialException: ... multiple access on port?   crashed
B: survived, but its first sentence was two GPS lines shredded together:
   ,,3,,5$G32,,,9151,,2,96*G3,,1,,,8,,6379$M055A3.5N7.500,*
```

There was a fifth reader nobody had accounted for: ModemManager, which probes
new serial devices by *writing* AT commands at them.

This cannot be fixed with a lock inside any one program. It needs a single
owner — which is what this daemon is.

## Install

If you are also building the consumers, they expect a sibling checkout layout:

```
~/prefix/src/WBR-GPS      <- this repo
~/prefix/src/WBR-GSM         (its CMake looks for ../WBR-GPS)
~/prefix/src/WBR-SA
```

### 1. Build and test

```bash
cmake -S . -B build && cmake --build build -j$(nproc)
ctest --test-dir build            # expect 15/15
```

### 2. Try it without installing anything

Fully reversible — nothing system-wide is touched.

```bash
D=$(mktemp -d)
./build/wbr-gpsd --socket $D/s.sock --lock $D/pid --group "" &
./tools/gpsq --socket $D/s.sock    # expect HELLO then a FIX line
kill %1
```

Do **not** pass `--serial`. With no path given the daemon finds the receiver
itself by scanning `/dev/serial/by-id` for a Leo Bodnar, so the port number
does not matter. Passing `--serial` disables that.

### 3. Install as a service

```bash
sudo ./install.sh
```

That is the whole install. It puts four files in place:

| path | purpose |
|------|---------|
| `/usr/local/bin/wbr-gpsd` | the daemon |
| `/etc/udev/rules.d/99-wbr-gps.rules` | creates `/dev/gpsdo`, sets `ID_MM_DEVICE_IGNORE=1` so ModemManager stops probing the port |
| `/etc/tmpfiles.d/wbr-gps.conf` | recreates `/run/wbr-gps` at every boot, group-writable by `dialout`, so a launcher can create the socket without root |
| `/etc/systemd/system/wbr-gpsd.service` | installed but **not enabled** — see below |

**The daemon does not run as a service.** It is started by whichever launcher
needs it (`bash SIGINT`, `./run_rtsa.sh`) and dies with it, including on
`kill -9`. So the GPSDO is free whenever nothing is using it — which matters
while some programs still open the device directly and would get `EBUSY` from
an idle daemon holding the port.

The unit file is still installed, so an always-on deployment is one command
away on a headless box with no GUI:

```bash
sudo systemctl enable --now wbr-gpsd
```

Do that only once every consumer on that machine reads GPS from the daemon.

**Whoever runs the programs must be in the `dialout` group** (`id -nG`). The
udev rule sets the device `0660 root:dialout`; an account outside that group
cannot open GPS at all.

> **From this moment the daemon owns the serial port exclusively.** Any program
> still opening `/dev/ttyACM0` directly gets `EBUSY`. That is the guarantee
> working, not a fault — but it means the daemon and every migrated consumer
> must go live together. Merging a consumer change *early* is harmless; it just
> starts using the daemon once one exists.

To also run the destructive failure proofs (crash-restart, and clean
stop/start against the live daemon), use `sudo ./tools/handoff-root.sh`
instead. It prints a warning banner and waits — answer `y`; pressing Enter
defaults to No and does nothing.

### 4. Verify

```bash
systemctl is-active wbr-gpsd            # active
tools/gpsq                              # HELLO, then a FIX with real lat/lon
ls -l /dev/gpsdo                        # -> ttyACM0
udevadm info -q property -n /dev/gpsdo | grep ID_MM_DEVICE_IGNORE
```

To confirm the daemon is the sole owner, **do not use `lsof` as a normal
user** — the daemon runs as root, so `lsof` reports zero openers, which looks
identical to "nobody has it". Ask the kernel instead:

```bash
python3 -c "import os; os.open('/dev/gpsdo', os.O_RDWR|os.O_NOCTTY)"
# expect: OSError [Errno 16] Device or resource busy
```

`EBUSY` proves something holds it exclusively.

### 5. The full end-to-end check

```bash
./tests/regression_all_consumers.sh     # expect ALL CHECKS PASSED
```

Runs every real consumer concurrently for 60 s and asserts one device owner,
zero torn NMEA sentences, zero `SerialException`, a stable badge, and a
consistent position across consumers.

## Using it from your program

**C++**

```cmake
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../WBR-GPS
                 ${CMAKE_CURRENT_BINARY_DIR}/wbr-gps EXCLUDE_FROM_ALL)
target_link_libraries(your_prog PRIVATE wbr_gps_core)
target_include_directories(your_prog PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/../WBR-GPS/include)
```

```cpp
#include "wbr_gps/client.hpp"

wbr_gps::Client gps;
gps.start();                       // background thread keeps a cache fresh
auto s = gps.snapshot();           // lock-guarded copy, no syscall — hot-loop safe

if (s.service_ok && s.has_fix) {
    use(s.lat, s.lon);
}

wbr_gps::get_once();               // one-shot, for a startup check
```

**Python** — standard library only, nothing to install.

```python
import wbr_gps_client as gc

snap = gc.get_once()               # never raises, on any path
if snap["service_ok"] and snap["has_fix"]:
    use(snap["lat"], snap["lon"])

gc.badge_status(snap)              # 'Locked' | 'Clock locked, no fix'
                                   # 'Not Locked' | 'Not Connected' | 'GPS Service Down'
```

**One rule: check `service_ok` before trusting anything else.** It means "I am
actually connected to the daemon right now". Without it a program will happily
report a position from a daemon that died ten minutes ago.

**And never add a fallback that opens the device.** That is the bug this
removes. `tests/check_no_device_access.sh` fails the build if anyone does.

### If your project builds with `-ffast-math`

Scope it off the GPS library:

```cmake
target_compile_options(wbr_gps_core PRIVATE -fno-fast-math -fno-finite-math-only)
```

`-ffast-math` implies `-ffinite-math-only`, under which the compiler folds away
the `std::isfinite()` guards that keep `nan` off the wire — where Python's
`json.loads` rejects it outright. Reproduced under real flags. Scoping it this
way leaves your own targets' optimisation untouched.

## The five guarantees

| | |
|---|---|
| **G1** | Exactly one process owns the device (`TIOCEXCL`, kernel-enforced) |
| **G2** | No client can block another — each has its own bounded queue |
| **G3** | No client can block the daemon — it never blocks on a write |
| **G4** | Unavailability is reported explicitly, never guessed |
| **G5** | No client has any fallback that opens the device; enforced at build time |

## Wire protocol

One JSON object per line over a Unix socket at `/run/wbr-gps/gpsd.sock`.
Readable with any tool.

```
{"class":"HELLO","proto":1,"daemon":"wbr-gpsd/1.0"}
{"class":"FIX","seq":548,"device_present":true,"serial_ok":true,"hid_ok":true,
 "gpsdo_locked":true,"has_fix":true,"fix_quality":1,"satellites":5,"hdop":1.39,
 "lat":13.0028535,"lon":77.6798595,"alt_m":914.90,"speed_kph":0.000,
 "time_utc":"090859.50","fix_age_ms":772}
{"class":"NMEA","raw":"$GNRMC,...*6A","t_unix_ms":1788340141598,"dropped":0}
```

Requests are `{"op":"get"}` or `{"op":"watch","nmea":true}`. The `dropped`
counter is how a client learns it fell behind — the daemon tells it rather than
hiding it.

## Layout

| file | responsibility |
|------|----------------|
| `src/serial_source.cpp` | **The only file in the whole tree that opens the GPS device.** |
| `src/hid_source.cpp` | GPSDO lock state over HID |
| `src/nmea_parser.cpp` | GGA/RMC, XOR checksum — a torn sentence is rejected here |
| `src/state_store.cpp` | the current snapshot, four independent health flags |
| `src/server.cpp` | accept, per-client queues, backpressure |
| `src/json_io.cpp` | wire format, NaN/Inf guards |
| `src/main.cpp` | epoll loop, single-instance lock, signals |
| `src/client.cpp` | the client library — contains no device code by construction |

## Known issues

**`gpsdo_locked` is decoded without a vendor spec.** Byte 0 of the HID status
report reads `0x7f` when the front-panel LED is solid, and `0x6e`/`0x76` when
it is blinking — confirmed by a human watching the LED across several sessions
including a 20-hour continuous run. The decode compares the whole byte rather
than testing a bit: both fit the data, and only whole-byte equality fails
toward "not locked". If a unit ever reports a different locked value it will
read as unlocked — safe, but wrong. Prefer `has_fix`. The comment in
`src/hid_source.cpp` carries the full evidence and the two earlier decodes that
were wrong.

**The udev rule tightens device permissions.** It replaces a prior `MODE=0666`
with the default `0660 root:dialout`. Any account that needs GPS -- including
whatever user a demo or service runs as -- must be in the `dialout` group or it
cannot open the port at all. Check with `id -nG` before deploying to a machine
where programs run as someone other than the installing user.

**No CI.** The test suite is real, but nothing runs it automatically.

**`systemd-analyze security` scores 7.5/10.** No `CapabilityBoundingSet`,
`SystemCallFilter`, or `PrivateUsers`. Running as root is justified in the unit
file's comments, but the hardening could go further — carefully, since the
daemon needs `ioctl(TIOCEXCL)` and raw HID reads.

## Reference

- Design spec: `docs/superpowers/specs/2026-09-01-gps-proxy-daemon-design.md`
- Every ruling, deferred item and reversal made during the build:
  `.superpowers/sdd/2026-09-01-wbr-gps-daemon/progress.md`
