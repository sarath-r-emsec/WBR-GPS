#!/usr/bin/env bash
# Guarantee G5: the client libraries must contain no device access, so no
# consumer can fall back to opening the GPS directly and reintroduce the
# contention this daemon exists to remove.
#
# This is a STRUCTURAL check, not a behavioural one. Every other test in the
# suite asks "did the client do the right thing this time?"; this one asks
# "could it ever do the wrong thing at all?" A fallback path that only fires
# when the daemon is down is exactly the code a behavioural test is least
# likely to reach, and exactly the code that would put two readers back on
# the tty at the worst possible moment.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# The three files a consumer links against or imports. Anything else in the
# tree may touch the device; these three may not.
FILES=(
  "$HERE/include/wbr_gps/client.hpp"
  "$HERE/src/client.cpp"
  "$HERE/python/wbr_gps_client.py"
)

# Patterns that would mean a client can touch hardware.
#
# Deliberately NOT the bare word "serial": the client legitimately reports
# Snapshot::serial_ok, which is the daemon's opinion about the port relayed
# over the socket, not an act of opening one. Matching it would make the gate
# fire on correct code, and a gate that cries wolf gets deleted. The Python
# device-access forms are pinned precisely instead: "serial.Serial",
# "import serial", "from serial import", "pyserial".
#
# For the same reason the bare word "hidraw" is not a pattern, only the path
# "/dev/hidraw": the Python client's own docstring states that it contains no
# hidraw access, and a gate that fails on a comment saying the right thing is
# a gate somebody deletes. Patterns are matched against the whole file,
# comments included, so an API named in prose still trips it -- that is
# intended for the names below, which have no reason to appear in a client at
# all.
PATTERNS='::open\(|[^_a-zA-Z]open\(|termios|tcsetattr|tcgetattr|cfsetispeed|cfsetospeed|cfmakeraw|ioctl|TIOCEXCL|O_NOCTTY|/dev/tty|/dev/hidraw|/dev/serial|/dev/gpsdo|serial\.Serial|import serial|from serial import|pyserial'

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
