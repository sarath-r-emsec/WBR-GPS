#!/usr/bin/env bash
# Everything Task 17 needs that requires root, collected into one script for
# a human to run. `sudo` in the environment this was written in prompts for
# a password and would hang an unattended agent session -- so this script
# is WRITTEN here but deliberately never EXECUTED by the agent that wrote
# it. A human with a real terminal runs it.
#
# It does two things, in order:
#
#   1. Runs install.sh. That script builds nothing itself -- it installs
#      the already-built binary, udev rules and systemd unit, enables and
#      RESTARTS the service, and prints its own detailed warning about the
#      visible behaviour change the moment it runs (the GPSDO becomes
#      single-owned; any not-yet-migrated program that still opens it
#      directly starts failing with EBUSY; /dev/hidraw* for vendor 1dd2
#      retightens from 0666 to 0660 group dialout). Read that warning when
#      it prints -- it is not repeated here.
#
#   2. Runs every verification command install.sh's own final output block
#      tells a human to run by hand, so this script actually PERFORMS the
#      post-install check instead of leaving it as a to-do. socat is not
#      installed in this environment; tools/gpsq stands in for it, exactly
#      the same substitution tests/regression_all_consumers.sh and
#      tests/soak.sh make and for the same reason.
#
# Usage: sudo tools/handoff-root.sh
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEV_SYMLINK=/dev/gpsdo
SOCK=/run/wbr-gps/gpsd.sock

if [[ $EUID -ne 0 ]]; then
    echo "handoff-root.sh must run as root (try: sudo $0)" >&2
    exit 1
fi

cat <<EOF
================================================================================
 This script is about to, IN ORDER:

  1. Run $HERE/install.sh
       - install build/wbr-gpsd to /usr/local/bin
       - install udev/99-wbr-gps.rules to /etc/udev/rules.d (retightens
         /dev/hidraw* for vendor 1dd2 from 0666 to 0660 group dialout)
       - install systemd/wbr-gpsd.service, enable it, and RESTART it
         (bounces any wbr-gpsd already running and drops its clients)
       - run install.sh's own automated smoke checks

  2. Run every verification command install.sh's final output tells a
     human to run by hand -- so this script performs them instead of
     leaving them as a suggestion:
       2.1  ls -l $DEV_SYMLINK
       2.2  udevadm info -q property -n $DEV_SYMLINK | grep ID_MM_...
       2.3  lsof \$(readlink -f $DEV_SYMLINK)          -- expect ONE opener: wbr-gpsd
       2.4  ls -l $SOCK                                -- expect srw-rw---- root dialout
       2.5  tools/gpsq $SOCK '{"op":"get"}' 2           -- expect a HELLO line, then a FIX line
                                                            (socat is not installed here)
       2.6  kill -9 the daemon's MainPID, then confirm systemd restarted it
            and it re-acquired the device sole ownership. This step is
            DESTRUCTIVE BY DESIGN: it is the actual proof of guarantee G5
            (the daemon survives a hard crash), not a simulation of it.

 install.sh prints its own warning about the visible behaviour change
 (EBUSY for any unmigrated direct opener) when step 1 runs below -- read it.
================================================================================
EOF

read -r -p "Continue? [y/N] " REPLY
if [[ ! "$REPLY" =~ ^[Yy]$ ]]; then
    echo "aborted -- nothing was done" >&2
    exit 1
fi

echo
echo "=== step 1: install.sh ==="
"$HERE/install.sh"

echo
echo "=== step 2.1: the symlink exists and points at the real tty ==="
ls -l "$DEV_SYMLINK"

echo
echo "=== step 2.2: ModemManager ignores it (expect ID_MM_DEVICE_IGNORE=1) ==="
udevadm info -q property -n "$DEV_SYMLINK" | grep -E 'ID_MM_DEVICE_IGNORE|ID_MM_CANDIDATE'

echo
echo "=== step 2.3: exactly one opener, and it is wbr-gpsd ==="
lsof "$(readlink -f "$DEV_SYMLINK")"

echo
echo "=== step 2.4: socket exists with the right ownership ==="
ls -l "$SOCK"

echo
echo "=== step 2.5: the daemon answers (HELLO, then a FIX) ==="
"$HERE/tools/gpsq" "$SOCK" '{"op":"get"}' 2

echo
echo "=== step 2.6: restart-on-crash (G5) -- WILL kill the daemon; systemd must bring it back ==="
BEFORE_PID="$(systemctl show -p MainPID --value wbr-gpsd)"
BEFORE_RESTARTS="$(systemctl show -p NRestarts --value wbr-gpsd)"
echo "killing MainPID $BEFORE_PID ..."
kill -9 "$BEFORE_PID"
sleep 4
if systemctl is-active --quiet wbr-gpsd; then
    AFTER_PID="$(systemctl show -p MainPID --value wbr-gpsd)"
    AFTER_RESTARTS="$(systemctl show -p NRestarts --value wbr-gpsd)"
    echo "wbr-gpsd is active again: pid $BEFORE_PID -> $AFTER_PID, NRestarts $BEFORE_RESTARTS -> $AFTER_RESTARTS"
    echo "re-checking the device is single-owned after the crash+restart:"
    lsof "$(readlink -f "$DEV_SYMLINK")"
else
    echo "FAIL: wbr-gpsd did not come back within 4s of being killed." >&2
    echo "      Check: journalctl -u wbr-gpsd -n 50" >&2
fi

cat <<EOF

================================================================================
 Root-requiring work is done. One thing is left that no script -- root or
 not -- can do:

   The physical unplug/replug acceptance test (spec §7.6). With the daemon
   running and a client watching, e.g.:

     $HERE/tools/gpsq $SOCK '{"op":"watch","fix":true}' 30

   physically unplug the Leo Bodnar, wait 10s, then replug it, and confirm
   the state transitions documented in the Task 17 report
   (.superpowers/sdd/2026-09-01-wbr-gps-daemon/task-17-report.md, section
   "Hardware unplug/replug acceptance test"). That is a human-hands step
   by its nature and cannot be scripted, root or not.
================================================================================
EOF
