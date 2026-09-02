#!/usr/bin/env bash
# Layer 4 (spec §7.4): run every real migrated consumer at once against
# wbr-gpsd and assert the original bug is gone.
#
# Pre-fix baseline, measured 2026-09-01 with three concurrent readers of
# /dev/ttyACM0 (no daemon -- each reader opened the tty itself):
#   A: SerialException: device reports readiness to read but returned no
#      data (device disconnected or multiple access on port?)   <- crashed
#   C: SerialException: ... multiple access on port?             <- crashed
#   B: lines=78 gga=13                                           <- survived,
#      but corrupted. Its first sentence arrived as two GGA lines shredded
#      together:  ",,3,,5$G32,,,9151,,2,96*G3,,1,,,8,,6379$M055A3.5N7.500,*"
#
# This script proves that no longer happens: exactly one opener of the
# device (and it is wbr-gpsd), zero torn/interleaved sentences, zero
# SerialException anywhere, a consistent position across every consumer,
# and a badge that does not flap.
#
# --- two modes, chosen automatically -----------------------------------
#
#   PROD         wbr-gpsd is already installed and active via systemd --
#                i.e. tools/handoff-root.sh has already been run. Talks to
#                the real production socket (/run/wbr-gps/gpsd.sock) and
#                does not touch the daemon at all.
#   SELF-HOSTED  no root has run install.sh yet (true of the sandbox this
#                was developed in: sudo prompts for a password there and
#                would hang the session). This script must still be
#                runnable unprivileged, so in this mode it starts its own
#                wbr-gpsd -- pointed at the real serial device, in a temp
#                dir -- and tears it down on every exit path. G1 (single
#                owner) is exactly as meaningfully proven this way: lsof
#                does not care how the daemon was started, only that
#                there is exactly one opener and it is named wbr-gpsd.
#
# Every consumer is pointed at whichever socket that mode picked, via each
# consumer's own --socket / --gps-socket flag -- except the SIGINT GUI's
# tasks.get_gps_status(), which has no such flag: scanner/tasks.py always
# calls wbr_gps_client.get_once(timeout=0.5), which resolves against
# wbr_gps_client.DEFAULT_SOCKET ("/run/wbr-gps/gpsd.sock") with no
# override, and writing to /run/wbr-gps/ needs root. In SELF-HOSTED mode
# this script substitutes the transport by replacing the *function object*
# tasks.wbr_gps_client.get_once with functools.partial(get_once,
# sock_path=OUR_SOCKET) before calling tasks.get_gps_status() -- the exact
# real function, actually invoked, only its target changed. In PROD mode
# nothing is patched: the GUI entry point runs completely unmodified.
#
# A one-off framing note, confirmed empirically while building this script:
# the very first NMEA line read from a FRESHLY opened tty can be a genuine
# fragment (a live GPS receiver transmits continuously; opening the port
# does not wait for a sentence boundary). That is physics, not the
# multi-reader corruption bug this daemon exists to remove -- it happens
# once per device open, is orthogonal to how many readers exist, and a
# production daemon only ever opens the device once at boot, long before
# any client attaches. So this script waits for the daemon to report
# serial_ok=true BEFORE starting any consumer, and the "zero torn
# sentences" assertion below is a real, meaningful check of the
# concurrent-access property, not a coin flip on device-open timing.
set -uo pipefail

SRC="${SRC:-/home/sigint-4/prefix/src}"
WBR_GPS_DIR="${WBR_GPS_DIR:-$SRC/WBR-GPS}"
WBR_GSM_DIR="${WBR_GSM_DIR:-$SRC/WBR-GSM}"
WBR_SA_DIR="${WBR_SA_DIR:-$SRC/WBR-SA}"
SIGINT_GUI_DIR="${SIGINT_GUI_DIR:-$SRC/SIGINT_GUI}"
GUI_PYTHON="${GUI_PYTHON:-$SIGINT_GUI_DIR/scan_test/bin/python}"

# spec §7.4 asks for a 60s run against the real trio; four consumers now
# (Task 16 migrated gps_capture too), still 60s by default.
RUN_SECONDS="${1:-60}"
PHASE2_PORT="${REG_PHASE2_PORT:-18099}"
POS_TOLERANCE_DEG="${REG_POS_TOLERANCE_DEG:-0.0005}"   # ~55 m at the equator

FAILED=0
fail() { echo "FAIL: $*" >&2; FAILED=1; }
pass() { echo "PASS: $*"; }
note() { echo "NOTE: $*"; }

# SIGTERM, wait up to $1 deciseconds for every listed pid to exit, then
# SIGKILL any stragglers and reap them all. Never blocks forever: WBR-SA's
# phase2_server was observed during development to survive plain SIGTERM
# indefinitely (it does install a handler -- see phase2_server.cpp's
# on_signal -- but its blocking event loop does not act on it promptly), and
# an unconditional `kill` followed by an unconditional `wait` hung this
# script's own cleanup for minutes the first time it ran against the real
# daemon. Nothing in this file trusts a spawned process to die on request.
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

# --- cleanup, on every exit path -------------------------------------------
#
# Mirrors tests/concurrency_util.h's rule: nothing is left behind, even on
# an assertion failure or a Ctrl-C. TMPDIR is only ever set once mktemp
# below has actually succeeded, and DAEMON_PID stays empty in PROD mode so
# cleanup never touches a systemd-managed daemon it does not own.
TMPDIR=""
DAEMON_PID=""
CAP_PID=""
PHASE2_PID=""
GUI_PID=""
PROBE_LOOP_PID=""
PHASE2_POLL_PID=""
LSOF_POLL_PID=""

cleanup() {
    # 50 deciseconds = 5s bound: comfortably past phase2_server's observed
    # slow-SIGTERM behaviour before this escalates to SIGKILL.
    kill_wait_all 50 "$PROBE_LOOP_PID" "$PHASE2_POLL_PID" "$GUI_PID" "$CAP_PID" "$PHASE2_PID" "$LSOF_POLL_PID"
    # Only kill a daemon THIS script started (SELF-HOSTED mode). A PROD-mode
    # daemon is systemd's to manage; this script must never bounce it.
    [[ -n "$DAEMON_PID" ]] && kill_wait_all 50 "$DAEMON_PID"
    [[ -n "$TMPDIR" ]] && rm -rf "$TMPDIR"
}
trap cleanup EXIT INT TERM

TMPDIR="$(mktemp -d /tmp/wbrgps_regression.XXXXXX)"

echo "=== 0. locate the device ==="
if [[ -e /dev/gpsdo ]]; then
    DEV="$(readlink -f /dev/gpsdo)"
    note "using udev symlink /dev/gpsdo -> $DEV"
else
    DEV="/dev/ttyACM0"
    note "/dev/gpsdo not present (udev rules not installed yet) -- using $DEV directly"
fi
if [[ ! -e "$DEV" ]]; then
    fail "device $DEV does not exist -- is the Leo Bodnar plugged in?"
    exit 1
fi

echo "=== 1. choose PROD or SELF-HOSTED mode ==="
MODE="SELF-HOSTED"
SOCK=""
if systemctl is-active --quiet wbr-gpsd.service 2>/dev/null && [[ -S /run/wbr-gps/gpsd.sock ]]; then
    MODE="PROD"
    SOCK="/run/wbr-gps/gpsd.sock"
    note "wbr-gpsd.service is active -- driving the real production socket"
else
    note "wbr-gpsd.service is not active (or not installed) -- this script has"
    note "no root, so it starts its own instance against the real device."
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

    # Wait for a genuine serial_ok=true reply, not a fixed sleep -- and this
    # is also what keeps the "zero torn sentences" check meaningful (see the
    # framing note at the top of this file): no consumer starts until the
    # daemon's own first, possibly-fragmentary read is already behind it.
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
    note "self-hosted wbr-gpsd up, pid=$DAEMON_PID, socket=$SOCK, serial_ok=true"
fi

# --- continuous lsof sampling for the whole run ----------------------------
#
# A single post-run lsof check (the original design) only sees whoever
# still holds the device AFTER step 4 has already killed three of the four
# consumers -- a consumer that held the device for the whole run and then
# exited cleanly leaves no trace in a one-shot check. Sampling throughout
# and requiring every sample to agree is the only way this assertion
# actually covers the run, not just its last instant.
: >"$TMPDIR/lsof_samples.log"
( while true; do
      OUT="$(lsof -F pc "$DEV" 2>/dev/null || true)"
      N="$(grep -c '^p' <<<"$OUT" || true)"
      NAMES="$(grep '^c' <<<"$OUT" | cut -c2- | sort -u | paste -sd, -)"
      echo "openers=$N names=${NAMES:-none}" >>"$TMPDIR/lsof_samples.log"
      sleep 2
  done ) &
LSOF_POLL_PID=$!

echo "=== 2. start every real consumer against $SOCK ==="

# --- gps_capture (WBR-GSM) --------------------------------------------------
GPS_CAPTURE_BIN="$WBR_GSM_DIR/build/gps_capture"
if [[ ! -x "$GPS_CAPTURE_BIN" ]]; then
    fail "$GPS_CAPTURE_BIN not built"
else
    "$GPS_CAPTURE_BIN" --out "$TMPDIR/nmea.log" --socket "$SOCK" \
        >"$TMPDIR/capture.log" 2>&1 &
    CAP_PID=$!
    note "gps_capture started, pid=$CAP_PID"
fi

# --- phase2_server (WBR-SA), --mock: no bladeRF needed for this check ------
PHASE2_BIN="$WBR_SA_DIR/build/phase2_server"
if [[ ! -x "$PHASE2_BIN" ]]; then
    fail "$PHASE2_BIN not built"
else
    if (exec 3<>"/dev/tcp/127.0.0.1/$PHASE2_PORT") 2>/dev/null; then
        exec 3>&- 3<&-
        fail "port $PHASE2_PORT is already in use -- set REG_PHASE2_PORT to another port"
    else
        mkdir -p "$TMPDIR/phase2_state" "$TMPDIR/phase2_recordings"
        ( cd "$WBR_SA_DIR" && exec "$PHASE2_BIN" --port "$PHASE2_PORT" --mock \
            --gps-socket "$SOCK" \
            --state "$TMPDIR/phase2_state" --recordings "$TMPDIR/phase2_recordings" \
            >"$TMPDIR/phase2.log" 2>&1 ) &
        PHASE2_PID=$!
        UP=0
        for _ in $(seq 1 80); do
            if (exec 3<>"/dev/tcp/127.0.0.1/$PHASE2_PORT") 2>/dev/null; then
                exec 3>&- 3<&-
                UP=1
                break
            fi
            sleep 0.25
        done
        if [[ "$UP" -ne 1 ]]; then
            fail "phase2_server did not come up on :$PHASE2_PORT"
            cat "$TMPDIR/phase2.log" >&2
        else
            note "phase2_server started, pid=$PHASE2_PID, port=$PHASE2_PORT"
            # Poll /api/gps for the whole run so it is a real concurrent
            # reader too, not a single point query.
            ( while true; do
                  curl -s "http://127.0.0.1:$PHASE2_PORT/api/gps" >>"$TMPDIR/phase2_poll.jsonl"
                  echo >>"$TMPDIR/phase2_poll.jsonl"
                  sleep 2
              done ) &
            PHASE2_POLL_PID=$!
        fi
    fi
fi

# --- gsm_monitor's GPS path (WBR-GSM) --------------------------------------
# gsm_monitor.cpp needs a live SDR (bladeRF) to do anything meaningful past
# startup -- driving it for real here would mean tuning and decoding actual
# GSM traffic, which this regression has no business doing. What it uses at
# startup, and what its long-lived Client wraps internally, is exactly
# wbr_gps::get_once() (see WBR-GSM/gsm_monitor.cpp, the
# wbr_gps::get_once().device_present check before the SDR is even opened).
# This probe exercises that same real entry point, compiled against the
# same wbr_gps_core the daemon and every other consumer use, so it stands
# in for gsm_monitor honestly rather than pretending gsm_monitor ran.
PROBE_SRC="$WBR_GPS_DIR/tests/fixtures/gps_get_once_probe.cpp"
PROBE_BIN="$TMPDIR/gps_get_once_probe"
if [[ ! -f "$PROBE_SRC" ]]; then
    fail "$PROBE_SRC missing"
elif ! g++ -std=c++17 -O2 -I "$WBR_GPS_DIR/include" "$PROBE_SRC" \
        "$WBR_GPS_DIR/build/libwbr_gps_core.a" -lpthread -o "$PROBE_BIN" \
        2>"$TMPDIR/probe_build.log"; then
    fail "gps_get_once_probe failed to build"
    cat "$TMPDIR/probe_build.log" >&2
else
    ( while true; do
          "$PROBE_BIN" "$SOCK" 500 >>"$TMPDIR/probe_poll.log" 2>>"$TMPDIR/probe_poll.log"
          sleep 2
      done ) &
    PROBE_LOOP_PID=$!
    note "gsm_monitor stand-in (wbr_gps::get_once probe) started, pid=$PROBE_LOOP_PID"
fi

# --- SIGINT GUI: tasks.get_gps_status(), the real production entry point ---
if [[ ! -x "$GUI_PYTHON" ]]; then
    fail "$GUI_PYTHON not found"
else
    SOCK_ARG="$SOCK"
    [[ "$MODE" == "PROD" ]] && SOCK_ARG="PROD"
    "$GUI_PYTHON" "$WBR_GPS_DIR/tests/fixtures/gui_gps_poll.py" \
        "$SIGINT_GUI_DIR" "$SOCK_ARG" "$RUN_SECONDS" 2 \
        >"$TMPDIR/gui_poll.jsonl" 2>"$TMPDIR/gui_poll.err" &
    GUI_PID=$!
    note "SIGINT GUI poller (tasks.get_gps_status) started, pid=$GUI_PID"
fi

echo "=== 3. let all four consumers run concurrently for ${RUN_SECONDS}s ==="
sleep "$RUN_SECONDS"

echo "=== 4. stop the consumers (daemon and phase2_server stay up until exit) ==="
# 30 deciseconds = 3s: these five are our own light background loops (a
# bash while-loop and gps_capture/the Python poller, both of which install
# ordinary SIGINT/SIGTERM handlers), so they are expected to die quickly.
kill_wait_all 30 "$PROBE_LOOP_PID" "$PHASE2_POLL_PID" "$GUI_PID" "$CAP_PID" "$LSOF_POLL_PID"
CAP_PID=""; GUI_PID=""; PROBE_LOOP_PID=""; PHASE2_POLL_PID=""; LSOF_POLL_PID=""

echo "=== 5. every lsof sample across the whole run showed at most one opener, and it was always wbr-gpsd ==="
# A one-shot check here would only see whoever holds the device AFTER the
# kill above -- three of the four consumers are already dead by this line.
# The real evidence is the sample log collected throughout step 3 (started
# right before step 2, so it also covers consumer startup).
if [[ ! -s "$TMPDIR/lsof_samples.log" ]]; then
    fail "no lsof samples were collected during the run"
else
    N_SAMPLES="$(wc -l <"$TMPDIR/lsof_samples.log")"
    MAX_OPENERS="$(sed -n 's/^openers=\([0-9]*\).*/\1/p' "$TMPDIR/lsof_samples.log" | sort -n | tail -1)"
    BAD_LINES="$(grep -vE '^openers=(0 names=none|1 names=wbr-gpsd)$' "$TMPDIR/lsof_samples.log" || true)"
    if [[ -z "$BAD_LINES" && "$MAX_OPENERS" -eq 1 ]]; then
        pass "all $N_SAMPLES lsof samples over the run showed at most one opener, and every single-opener sample was wbr-gpsd (max seen: $MAX_OPENERS)"
    else
        fail "lsof sampling found a problem across $N_SAMPLES samples (max openers seen: ${MAX_OPENERS:-0})"
        if [[ -n "$BAD_LINES" ]]; then
            echo "bad samples:" >&2
            echo "$BAD_LINES" >&2
        fi
    fi
fi

echo "=== 6. every captured NMEA line is a single, well-formed, checksum-valid sentence ==="
# A shape-only regex (leading $, five letters, a comma, ANYTHING, a star,
# two hex digits) passes a two-sentence splice on one line as long as the
# tail looks like a valid trailer -- exactly the shape of the original
# bug's corruption -- and it passes a shape-valid-but-checksum-invalid
# sentence outright, which matters here specifically because the daemon's
# passthrough deliberately forwards parser-rejected sentences verbatim
# (see gps_capture.cpp). tests/fixtures/check_nmea_integrity.py checks the
# real NMEA-0183 checksum and an exactly-one-'$'-per-line splice guard.
if [[ -s "$TMPDIR/nmea.log" ]]; then
    N_LINES="$(wc -l <"$TMPDIR/nmea.log")"
    if python3 "$WBR_GPS_DIR/tests/fixtures/check_nmea_integrity.py" "$TMPDIR/nmea.log"; then
        pass "all captured NMEA lines are single, well-formed, checksum-valid sentences"
    else
        fail "at least one captured NMEA line failed integrity checks (see above)"
    fi
    if [[ "$N_LINES" -lt 10 ]]; then
        note "only $N_LINES lines captured -- this check has little statistical weight this run"
    fi
else
    fail "no NMEA captured at all (nmea.log is empty or missing)"
fi

echo "=== 7. no SerialException / multiple-access anywhere ==="
# *.err is in this glob because the GUI poller's own errors go to
# gui_poll.err, not gui_poll.jsonl or any *.log file -- an earlier version
# of this check globbed only *.log/*.jsonl and so could never see a
# SerialException from the one consumer that historically produced them.
if grep -qiE "SerialException|multiple access on port" \
        "$TMPDIR"/*.log "$TMPDIR"/*.jsonl "$TMPDIR"/*.err 2>/dev/null; then
    fail "a consumer hit a serial access error"
    grep -inE "SerialException|multiple access on port" "$TMPDIR"/*.log "$TMPDIR"/*.jsonl "$TMPDIR"/*.err 2>/dev/null >&2
else
    pass "no serial access errors in any consumer's output"
fi

echo "=== 8. all consumers agree on position, and the GUI badge is stable ==="
python3 "$WBR_GPS_DIR/tests/fixtures/regression_summary.py" \
    --gui "$TMPDIR/gui_poll.jsonl" \
    --phase2 "$TMPDIR/phase2_poll.jsonl" \
    --probe "$TMPDIR/probe_poll.log" \
    --tolerance-deg "$POS_TOLERANCE_DEG"
SUMMARY_RC=$?
[[ "$SUMMARY_RC" -ne 0 ]] && FAILED=1

echo "=== result ==="
if [[ "$FAILED" -eq 0 ]]; then
    echo "ALL CHECKS PASSED"
else
    echo "AT LEAST ONE CHECK FAILED" >&2
fi
exit "$FAILED"
