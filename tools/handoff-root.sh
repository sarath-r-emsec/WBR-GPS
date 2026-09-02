#!/usr/bin/env bash
# Everything Task 17 needs that requires root, collected into one script for
# a human to run. `sudo` in the environment this was written in prompts for
# a password and would hang an unattended agent session -- so this script
# is WRITTEN here but deliberately never EXECUTED by the agent that wrote
# it. A human with a real terminal runs it.
#
# It does these things, in order:
#
#   1. Runs install.sh. That script builds nothing itself -- it installs
#      the already-built binary, udev rules and systemd unit, enables and
#      RESTARTS the service, and prints its own detailed warning about the
#      visible behaviour change the moment it runs (the GPSDO becomes
#      single-owned; any not-yet-migrated program that still opens it
#      directly starts failing with EBUSY; /dev/hidraw* for vendor 1dd2
#      retightens from 0666 to 0660 group dialout). Read that warning when
#      it prints -- it is not repeated here. If it fails, this script stops
#      immediately: nothing below is safe to run against a failed install.
#
#   2. Runs and ASSERTS every verification step install.sh's own final
#      output block tells a human to run and eyeball by hand (steps 2.1-2.5
#      below), plus a real crash-restart proof (2.6) and a real proof of G5
#      (2.7, spec scenario S12) -- see the step comments for why 2.6 is NOT
#      G5, a mislabelling this script used to carry. socat is not installed
#      in this environment; tools/gpsq stands in for it, exactly the same
#      substitution tests/regression_all_consumers.sh and tests/soak.sh
#      make and for the same reason.
#
# Every step accumulates into FAILED and the closing banner is gated on it
# -- this script does not claim the handoff is done if a check above it
# actually failed.
#
# Usage: sudo tools/handoff-root.sh
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEV_SYMLINK=/dev/gpsdo
SOCK=/run/wbr-gps/gpsd.sock

FAILED=0
fail() { echo "FAIL: $*" >&2; FAILED=1; }
pass() { echo "PASS: $*"; }

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
     If this fails, the script stops here -- nothing below runs.

  2. Run and ASSERT every verification step install.sh's final output
     tells a human to run by hand, plus two destructive proofs:
       2.1  the /dev/gpsdo symlink exists and resolves to a real device
       2.2  ModemManager ignores it (ID_MM_DEVICE_IGNORE=1)
       2.3  exactly one opener of the device, and it is wbr-gpsd
       2.4  socket mode/ownership is exactly srw-rw---- root dialout
       2.5  the daemon answers a request (HELLO, then a FIX)
       2.6  crash-restart supervision + G1 re-established: kill -9 the
            daemon's MainPID, confirm systemd restarts it under a new pid
            with NRestarts incremented, and confirm it re-acquires sole
            ownership of the device. This is NOT a proof of G5 (a common
            mislabelling this script used to carry) -- it proves the daemon
            survives a hard crash and G1 holds again afterward.
       2.7  the actual proof of G5 (spec scenario S12): cleanly STOP the
            daemon, confirm the device shows ZERO openers and a client
            gets no reply, then START it again and confirm recovery.
     Both 2.6 and 2.7 are DESTRUCTIVE BY DESIGN against a live production
     daemon and its connected clients.

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
"$HERE/install.sh" || {
    echo "install.sh failed -- stopping before any verification. Nothing" >&2
    echo "past this point is safe to run against a failed/partial install." >&2
    exit 1
}

echo
echo "=== step 2.1: the symlink exists and points at the real tty ==="
ls -l "$DEV_SYMLINK" 2>&1 || true
if [[ -L "$DEV_SYMLINK" ]] && DEV="$(readlink -f "$DEV_SYMLINK" 2>/dev/null)" && [[ -e "$DEV" ]]; then
    pass "$DEV_SYMLINK -> $DEV"
else
    fail "$DEV_SYMLINK is not a symlink to an existing device"
fi

echo
echo "=== step 2.2: ModemManager ignores it (expect ID_MM_DEVICE_IGNORE=1) ==="
MM_PROPS="$(udevadm info -q property -n "$DEV_SYMLINK" 2>/dev/null | grep -E 'ID_MM_DEVICE_IGNORE|ID_MM_CANDIDATE' || true)"
echo "$MM_PROPS"
if [[ "$MM_PROPS" == *'ID_MM_DEVICE_IGNORE=1'* ]]; then
    pass "ModemManager ignores the device"
else
    fail "ID_MM_DEVICE_IGNORE=1 not found -- ModemManager may still probe this device (may need: systemctl restart ModemManager, or a physical replug)"
fi

echo
echo "=== step 2.3: exactly one opener, and it is wbr-gpsd ==="
DEV="$(readlink -f "$DEV_SYMLINK" 2>/dev/null)"
lsof "$DEV" 2>&1 || true
LSOF_PARSED="$(lsof -F pc "$DEV" 2>/dev/null || true)"
N_OPENERS="$(grep -c '^p' <<<"$LSOF_PARSED" || true)"
OWNER="$(grep '^c' <<<"$LSOF_PARSED" | head -1 | cut -c2-)"
if [[ "$N_OPENERS" -eq 1 && "$OWNER" == "wbr-gpsd" ]]; then
    pass "exactly one opener, and it is wbr-gpsd"
else
    fail "expected exactly one opener named wbr-gpsd; found $N_OPENERS opener(s), first named '${OWNER:-none}'"
fi

echo
echo "=== step 2.4: socket exists with the right ownership (expect srw-rw---- root dialout) ==="
ls -l "$SOCK" 2>&1 || true
if SOCK_STAT="$(stat -c '%A %U %G' "$SOCK" 2>/dev/null)"; then
    if [[ "$SOCK_STAT" == "srw-rw---- root dialout" ]]; then
        pass "socket mode/ownership is srw-rw---- root dialout"
    else
        fail "expected 'srw-rw---- root dialout', got '$SOCK_STAT'"
    fi
else
    fail "$SOCK does not exist"
fi

echo
echo "=== step 2.5: the daemon answers (HELLO, then a FIX) ==="
GPSQ_OUT="$("$HERE/tools/gpsq" "$SOCK" '{"op":"get"}' 2 2>&1)"
echo "$GPSQ_OUT"
if [[ "$GPSQ_OUT" == *'"class":"HELLO"'* && "$GPSQ_OUT" == *'"class":"FIX"'* ]]; then
    pass "daemon replied with HELLO then FIX"
else
    fail "expected a HELLO line and a FIX line from the daemon, did not see both"
fi

echo
echo "=== step 2.6: crash-restart supervision + G1 re-established (NOT G5 -- see 2.7) ==="
echo "WILL kill -9 the daemon; systemd must bring it back."
BEFORE_PID="$(systemctl show -p MainPID --value wbr-gpsd)"
BEFORE_RESTARTS="$(systemctl show -p NRestarts --value wbr-gpsd)"
# MainPID is "0" when the unit is not active. kill -9 0 sends SIGKILL to
# THIS SHELL'S ENTIRE PROCESS GROUP -- refusing to proceed without a
# genuine positive pid is not optional caution, it is the difference
# between killing the daemon and killing the operator's own session.
if [[ ! "$BEFORE_PID" =~ ^[1-9][0-9]*$ ]]; then
    fail "no live MainPID for wbr-gpsd ('$BEFORE_PID') -- refusing to run kill -9 with it"
    echo "This should not happen right after a successful install.sh (which" >&2
    echo "already waits for the unit to become active). Stopping here rather" >&2
    echo "than risk 'kill -9 0' -- investigate with: systemctl status wbr-gpsd" >&2
    exit 1
fi
echo "killing MainPID $BEFORE_PID ..."
kill -9 "$BEFORE_PID"
sleep 4
if systemctl is-active --quiet wbr-gpsd; then
    AFTER_PID="$(systemctl show -p MainPID --value wbr-gpsd)"
    AFTER_RESTARTS="$(systemctl show -p NRestarts --value wbr-gpsd)"
    if [[ "$AFTER_PID" =~ ^[1-9][0-9]*$ && "$AFTER_PID" != "$BEFORE_PID" && "$AFTER_RESTARTS" -gt "$BEFORE_RESTARTS" ]]; then
        pass "systemd restarted wbr-gpsd: pid $BEFORE_PID -> $AFTER_PID, NRestarts $BEFORE_RESTARTS -> $AFTER_RESTARTS"
    else
        fail "wbr-gpsd is active but the restart evidence looks wrong: pid $BEFORE_PID -> $AFTER_PID, NRestarts $BEFORE_RESTARTS -> $AFTER_RESTARTS"
    fi
    DEV="$(readlink -f "$DEV_SYMLINK" 2>/dev/null)"
    lsof "$DEV" 2>&1 || true
    LSOF_PARSED="$(lsof -F pc "$DEV" 2>/dev/null || true)"
    N_OPENERS="$(grep -c '^p' <<<"$LSOF_PARSED" || true)"
    OWNER="$(grep '^c' <<<"$LSOF_PARSED" | head -1 | cut -c2-)"
    if [[ "$N_OPENERS" -eq 1 && "$OWNER" == "wbr-gpsd" ]]; then
        pass "G1 re-established: single owner (wbr-gpsd) after the crash+restart"
    else
        fail "G1 NOT re-established after crash+restart: $N_OPENERS opener(s), first named '${OWNER:-none}'"
    fi
else
    fail "wbr-gpsd did not come back within 4s of being killed -- check: journalctl -u wbr-gpsd -n 50"
fi

echo
echo "=== step 2.7: G5 proof (spec S12) -- cleanly STOP the daemon, confirm ZERO openers and a client reporting unavailable, then START it again ==="
systemctl stop wbr-gpsd
sleep 1
DEV="$(readlink -f "$DEV_SYMLINK" 2>/dev/null)"
lsof "$DEV" 2>&1 || true
LSOF_PARSED="$(lsof -F pc "$DEV" 2>/dev/null || true)"
S12_OPENERS="$(grep -c '^p' <<<"$LSOF_PARSED" || true)"
if [[ "$S12_OPENERS" -eq 0 ]]; then
    pass "zero openers of the device while the daemon is stopped"
else
    fail "expected zero openers while stopped, found $S12_OPENERS"
fi
S12_REPLY="$("$HERE/tools/gpsq" "$SOCK" '{"op":"get"}' 2 2>&1 || true)"
echo "$S12_REPLY"
if [[ "$S12_REPLY" != *'"class":'* ]]; then
    pass "a client gets no reply from the stopped daemon (reports unavailable)"
else
    fail "expected no reply from a stopped daemon, but got one"
fi
echo "restarting wbr-gpsd..."
systemctl start wbr-gpsd
S12_UP=0
for _ in $(seq 1 10); do
    systemctl is-active --quiet wbr-gpsd && { S12_UP=1; break; }
    sleep 1
done
if [[ "$S12_UP" -eq 1 ]]; then
    pass "wbr-gpsd active again after the clean stop/start"
    DEV="$(readlink -f "$DEV_SYMLINK" 2>/dev/null)"
    lsof "$DEV" 2>&1 || true
else
    fail "wbr-gpsd did not come back after 'systemctl start' -- check: journalctl -u wbr-gpsd -n 50"
fi

echo
echo "=== result ==="
if [[ "$FAILED" -eq 0 ]]; then
    cat <<EOF
================================================================================
 Every verification step above PASSED. Root-requiring work is done. One
 thing is left that no script -- root or not -- can do:

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
else
    cat <<EOF
================================================================================
 At least one verification step above FAILED (see the FAIL lines) -- do
 NOT consider this handoff complete. Investigate and re-run before trusting
 the installed daemon. Once every check above passes, the physical
 unplug/replug acceptance test (spec §7.6, documented in the Task 17
 report) still needs a human hand on the actual USB connector -- no script
 can do that part either.
================================================================================
EOF
fi
exit "$FAILED"
