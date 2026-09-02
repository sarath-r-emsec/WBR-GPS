#!/usr/bin/env bash
# Layer 5 (spec §7.5): long soak with clients attached. Watches for fd
# leaks, memory growth and unexpected restarts. Default 1 hour; pass a
# duration in seconds to change (a short smoke run, e.g. 300, is enough to
# prove the mechanics -- a real soak needs the full default or longer).
#
# --- two modes, chosen automatically, same as tests/regression_all_consumers.sh ---
#
#   PROD         wbr-gpsd is already installed and active via systemd (i.e.
#                tools/handoff-root.sh has been run). Watches the real
#                production daemon and never touches its lifecycle.
#   SELF-HOSTED  no root has run install.sh yet. This script must still be
#                runnable unprivileged, so it starts its own wbr-gpsd
#                against the real device, in a temp dir, and tears it down
#                on every exit path.
#
# In PROD mode the daemon runs as root (systemd unit: User=root -- see
# systemd/wbr-gpsd.service). Reading another UID's /proc/PID/fd directory
# needs to be that UID or have CAP_SYS_PTRACE; an unprivileged soak run
# against PROD cannot list its fds and says so plainly rather than reporting
# a silently-wrong count. Run this script as root (or as whatever account
# ends up owning the daemon, if one is ever provisioned -- see the comment
# on User=root in the unit) to soak-test the installed PROD daemon.
#
# socat is not installed here; tools/gpsq stands in for it exactly as
# gpsq's own docstring intends: `gpsq SOCKET REQUEST SECONDS` holds the
# connection open, printing whatever the daemon pushes, for up to SECONDS.
# Five of those, backgrounded for the whole soak with SECONDS=$DURATION,
# ARE the long-lived watchers socat would have been.
set -uo pipefail

SRC="${SRC:-/home/sigint-4/prefix/src}"
WBR_GPS_DIR="${WBR_GPS_DIR:-$SRC/WBR-GPS}"

DURATION="${1:-3600}"
if ! [[ "$DURATION" =~ ^[0-9]+$ ]] || [[ "$DURATION" -le 0 ]]; then
    echo "usage: $0 [DURATION_SECONDS]  (got: '$DURATION')" >&2
    exit 2
fi
N_WATCHERS="${SOAK_WATCHERS:-5}"
POLL_INTERVAL="${SOAK_POLL_INTERVAL:-60}"
# RSS is allowed to double over the run before this calls it a leak -- same
# heuristic as the original brief. FD growth is judged against a threshold
# DERIVED from the measured baseline (below), not a hardcoded constant: it
# must track N_WATCHERS if that ever changes.
RSS_GROWTH_FACTOR=2

FAILED=0
fail() { echo "FAIL: $*" >&2; FAILED=1; }
pass() { echo "PASS: $*"; }
note() { echo "NOTE: $*"; }

# See tests/regression_all_consumers.sh for why an unconditional `kill` +
# `wait` is not safe: at least one real consumer in this codebase (WBR-SA's
# phase2_server) has been observed to survive plain SIGTERM indefinitely.
# soak.sh only ever spawns gpsq (plain Python, no custom signal handling)
# and, in SELF-HOSTED mode, wbr-gpsd itself (which does exit promptly on
# SIGTERM -- src/main.cpp's on_signal), so this bound is mostly insurance,
# not an expected code path.
kill_wait_all() {
    local bound_ds="$1"; shift
    local pids=("$@") p i any_alive
    for p in "${pids[@]}"; do
        [[ -n "$p" ]] && kill "$p" 2>/dev/null
    done
    i=0
    while (( i < bound_ds )); do
        any_alive=0
        for p in "${pids[@]}"; do
            [[ -n "$p" ]] && kill -0 "$p" 2>/dev/null && any_alive=1
        done
        (( any_alive == 0 )) && break
        sleep 0.1
        i=$(( i + 1 ))
    done
    for p in "${pids[@]}"; do
        [[ -n "$p" ]] && kill -0 "$p" 2>/dev/null && kill -9 "$p" 2>/dev/null
    done
    for p in "${pids[@]}"; do
        [[ -n "$p" ]] && wait "$p" 2>/dev/null
    done
}

TMPDIR=""
DAEMON_PID=""            # only set (and only ours to kill) in SELF-HOSTED mode
WATCHER_PIDS=()

cleanup() {
    kill_wait_all 50 "${WATCHER_PIDS[@]}"
    [[ -n "$DAEMON_PID" ]] && kill_wait_all 50 "$DAEMON_PID"
    [[ -n "$TMPDIR" ]] && rm -rf "$TMPDIR"
}
trap cleanup EXIT INT TERM

TMPDIR="$(mktemp -d /tmp/wbrgps_soak.XXXXXX)"

echo "=== 0. locate the device ==="
if [[ -e /dev/gpsdo ]]; then
    DEV="$(readlink -f /dev/gpsdo)"
    note "using udev symlink /dev/gpsdo -> $DEV"
else
    DEV="/dev/ttyACM0"
    note "/dev/gpsdo not present (udev rules not installed yet) -- using $DEV directly"
fi

echo "=== 1. choose PROD or SELF-HOSTED mode ==="
MODE="SELF-HOSTED"
SOCK=""
if systemctl is-active --quiet wbr-gpsd.service 2>/dev/null && [[ -S /run/wbr-gps/gpsd.sock ]]; then
    MODE="PROD"
    SOCK="/run/wbr-gps/gpsd.sock"
    PID="$(systemctl show -p MainPID --value wbr-gpsd.service)"
    note "wbr-gpsd.service is active (pid $PID) -- soaking the real production daemon"
else
    note "wbr-gpsd.service is not active (or not installed) -- this script has"
    note "no root, so it starts its own instance against the real device."
    if [[ ! -e "$DEV" ]]; then
        fail "device $DEV does not exist -- is the Leo Bodnar plugged in?"
        exit 1
    fi
    SOCK="$TMPDIR/gpsd.sock"
    DAEMON_BIN="$WBR_GPS_DIR/build/wbr-gpsd"
    if [[ ! -x "$DAEMON_BIN" ]]; then
        fail "$DAEMON_BIN not built -- cmake --build $WBR_GPS_DIR/build first"
        exit 1
    fi
    "$DAEMON_BIN" --socket "$SOCK" --serial "$DEV" \
        --lock "$TMPDIR/wbr-gpsd.pid" --group "" \
        >"$TMPDIR/daemon.log" 2>&1 &
    DAEMON_PID=$!
    for _ in $(seq 1 50); do
        [[ -S "$SOCK" ]] && break
        sleep 0.1
    done
    if [[ ! -S "$SOCK" ]]; then
        fail "self-hosted wbr-gpsd never created $SOCK"
        cat "$TMPDIR/daemon.log" >&2
        exit 1
    fi

    # Wait for a genuine serial_ok=true reply before taking any baseline
    # measurement below -- not a fixed sleep. Measured taking the fd
    # baseline right after the socket merely EXISTS (before serial/hid
    # finish their own open()s a few hundred ms later, on the daemon's own
    # kPeriodicMs tick): the baseline undercounts by exactly those two fds,
    # which a fixed slack margin can paper over but a correct baseline
    # should not need to.
    SERIAL_OK=0
    for _ in $(seq 1 50); do
        REPLY="$("$WBR_GPS_DIR/tools/gpsq" "$SOCK" '{"op":"get"}' 1 2>/dev/null | tail -1)"
        if [[ "$REPLY" == *'"serial_ok":true'* ]]; then
            SERIAL_OK=1
            break
        fi
        sleep 0.2
    done
    if [[ "$SERIAL_OK" -ne 1 ]]; then
        fail "self-hosted wbr-gpsd never reported serial_ok=true within 10s"
        cat "$TMPDIR/daemon.log" >&2
        exit 1
    fi
    PID="$DAEMON_PID"
    note "self-hosted wbr-gpsd up, pid=$PID, socket=$SOCK, serial_ok=true"
fi

echo "=== 2. baseline fd/rss, taken before any watcher connects ==="
if [[ ! -r "/proc/$PID/status" ]]; then
    fail "cannot read /proc/$PID/status for pid $PID (wrong UID?)"
    exit 1
fi
if ! FD0="$(ls "/proc/$PID/fd" 2>/dev/null | wc -l)" || [[ "$FD0" -eq 0 ]]; then
    fail "cannot list /proc/$PID/fd for pid $PID -- likely a permission problem:"
    fail "the daemon runs as a different UID (root, in the installed PROD unit)"
    fail "and reading another UID's fd table needs to BE that UID or root."
    fail "re-run this script as root to soak-test the installed daemon."
    exit 1
fi
RSS0="$(awk '/VmRSS/{print $2}' "/proc/$PID/status")"
RESTARTS0=""
[[ "$MODE" == "PROD" ]] && RESTARTS0="$(systemctl show -p NRestarts --value wbr-gpsd.service)"
FD_LIMIT=$(( FD0 + N_WATCHERS + 5 ))   # +5: transient accept/negotiation slack
echo "baseline: pid=$PID fds=$FD0 rss=${RSS0}kB restarts=${RESTARTS0:-n/a} fd_limit=$FD_LIMIT"

echo "=== 3. start $N_WATCHERS long-lived watchers for the whole soak ==="
for i in $(seq 1 "$N_WATCHERS"); do
    "$WBR_GPS_DIR/tools/gpsq" "$SOCK" '{"op":"watch","fix":true}' "$DURATION" \
        >"$TMPDIR/watcher_$i.log" 2>&1 &
    WATCHER_PIDS+=("$!")
done
note "watcher pids: ${WATCHER_PIDS[*]}"

echo "=== 4. poll every ${POLL_INTERVAL}s for ${DURATION}s ==="
END=$(( $(date +%s) + DURATION ))
N_CHECKS=0
while [[ $(date +%s) -lt "$END" ]]; do
    SLEEP_LEFT=$(( END - $(date +%s) ))
    (( SLEEP_LEFT > POLL_INTERVAL )) && SLEEP_LEFT="$POLL_INTERVAL"
    (( SLEEP_LEFT > 0 )) && sleep "$SLEEP_LEFT"
    N_CHECKS=$(( N_CHECKS + 1 ))

    if [[ "$MODE" == "PROD" ]]; then
        CUR="$(systemctl show -p MainPID --value wbr-gpsd.service)"
        if [[ "$CUR" != "$PID" ]]; then
            fail "daemon restarted (was pid $PID, now $CUR)"
            break
        fi
    else
        if ! kill -0 "$PID" 2>/dev/null; then
            fail "self-hosted daemon (pid $PID) is no longer running"
            break
        fi
    fi

    FD="$(ls "/proc/$PID/fd" 2>/dev/null | wc -l)"
    RSS="$(awk '/VmRSS/{print $2}' "/proc/$PID/status" 2>/dev/null)"
    echo "$(date '+%H:%M:%S') check=$N_CHECKS fds=$FD rss=${RSS:-?}kB"

    if [[ "$FD" -gt "$FD_LIMIT" ]]; then
        fail "fd leak: $FD open fds exceeds baseline-derived limit $FD_LIMIT (fds0=$FD0 + watchers=$N_WATCHERS + slack=5)"
        break
    fi
    if [[ -n "$RSS" ]] && [[ "$RSS" -gt $(( RSS0 * RSS_GROWTH_FACTOR )) ]]; then
        fail "memory growth: ${RSS}kB exceeds ${RSS_GROWTH_FACTOR}x baseline (${RSS0}kB)"
        break
    fi
done

echo "=== 5. final restart check ==="
if [[ "$MODE" == "PROD" ]]; then
    RESTARTS1="$(systemctl show -p NRestarts --value wbr-gpsd.service)"
    if [[ "$RESTARTS1" == "$RESTARTS0" ]]; then
        pass "NRestarts unchanged ($RESTARTS0 -> $RESTARTS1)"
    else
        fail "NRestarts changed: $RESTARTS0 -> $RESTARTS1"
    fi
else
    if kill -0 "$PID" 2>/dev/null; then
        pass "self-hosted daemon still running under its original pid ($PID) -- no restart possible without a supervisor, so this is the closest equivalent check"
    else
        fail "self-hosted daemon (pid $PID) died during the soak"
    fi
fi

echo "=== result ==="
if [[ "$FAILED" -eq 0 ]]; then
    echo "PASS: soak completed ($N_CHECKS checks over ${DURATION}s) with no leak, growth or restart"
else
    echo "FAIL: soak found a problem -- see above" >&2
fi
exit "$FAILED"
