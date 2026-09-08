# shellcheck shell=bash
#
# gpsd-session.sh -- wbr-gpsd lives exactly as long as something needs it.
#
# Source this from a launcher (SIGINT, run_rtsa.sh, run_gsm_monitor), call
# wbr_gpsd_session_start early, and wbr_gpsd_session_stop from the exit trap.
#
# WHY NOT ALWAYS-ON
#
# The systemd unit holds the GPSDO whether anything needs it or not. That is
# wrong while the fleet is half-migrated: an unmigrated gsm_monitor still opens
# /dev/ttyACM0 directly, and a daemon idling with no consumers would refuse it
# with EBUSY for no reason. The device must be free whenever nothing is using
# it.
#
# WHY REFERENCE COUNTING RATHER THAN AN OWNER
#
# "First launcher to start it owns it, and kills it on exit" is simpler, and it
# is wrong. Start run_rtsa.sh, then start the GUI: the GUI reuses the daemon
# without owning it, and killing RTSA yanks the device out from under a GUI
# that is still running. Any rule based on who started it breaks as soon as two
# consumers overlap.
#
# So every launcher registers a holder and the daemon outlives all of them:
#
#     /run/wbr-gps/holders/<pid>     one file per live launcher
#
# The daemon shuts down when the last holder is gone -- whether that holder
# exited cleanly, was Ctrl-C'd, or was killed with -9.
#
# HOW kill -9 IS SURVIVED
#
# A trap cannot catch SIGKILL, so holder files cannot be relied on to be
# removed. Instead a watchdog owns the daemon:
#
#     launcher --> (setsid) watchdog --> wbr-gpsd     [setpriv --pdeathsig TERM]
#
# The watchdog polls the holders directory once a second and PRUNES entries
# whose pid is no longer alive. When none remain it exits, and pdeathsig makes
# the kernel kill the daemon -- so a launcher dying by any means eventually
# releases the device without anything having to run in that launcher.
#
# The watchdog is setsid'd so it is not in any launcher's process group and
# does not die with the first one. If the watchdog itself is killed, the daemon
# dies with it via pdeathsig: the failure direction is always "device
# released", never "device held by nothing".

WBR_GPSD_SOCK="${WBR_GPSD_SOCK:-/run/wbr-gps/gpsd.sock}"
WBR_GPSD_DIR="$(dirname "$WBR_GPSD_SOCK")"
WBR_GPSD_HOLDERS="$WBR_GPSD_DIR/holders"
# Binaries this launcher is about to run that might want GPS. Set by the
# launcher before calling wbr_gpsd_session_start; missing entries are ignored.
WBR_GPSD_CONSUMERS=("${WBR_GPSD_CONSUMERS[@]}")
WBR_GPSD_HELD=0

_wbr_gpsd_binary() {
    if [[ -x /usr/local/bin/wbr-gpsd ]]; then
        echo /usr/local/bin/wbr-gpsd
        return 0
    fi
    local here c
    here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    for c in "$here/build/wbr-gpsd" "$here/../WBR-GPS/build/wbr-gpsd"; do
        [[ -x "$c" ]] && { echo "$c"; return 0; }
    done
    return 1
}

# Is a daemon ANSWERING? A stale socket file left by a hard kill is not a
# running daemon, so connect rather than trusting -S.
_wbr_gpsd_answering() {
    [[ -S "$WBR_GPSD_SOCK" ]] || return 1
    timeout 3 python3 - "$WBR_GPSD_SOCK" <<'PY' 2>/dev/null
import socket, sys
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.settimeout(1.5)
try:
    s.connect(sys.argv[1])
    s.sendall(b'{"op":"get"}\n')
    sys.exit(0 if s.recv(64) else 1)
except OSError:
    sys.exit(1)
PY
}

# Would starting the daemon BREAK one of the programs about to run?
#
# The daemon takes the GPSDO exclusively, so a consumer that still opens
# /dev/ttyACM0 itself gets EBUSY and reports no GPS -- worse than not running
# the daemon at all. And this cannot be settled per-repo: `bash SIGINT` launches
# gsm_monitor and bwi_server from OTHER repos, which may not have migrated, so
# the GUI merging first would silently break them.
#
# Rather than demanding the repos merge in lockstep, look at the binaries.
# Classification by what each references:
#
#   references wbr-gpsd ............... migrated, safe
#   references /dev/ttyACM or by-id
#     but NOT wbr-gpsd ............... UNMIGRATED -> do not start the daemon
#   references neither ............... does not use GPS -> safe
#   not installed .................... cannot be broken -> safe
#
# Self-correcting: as each program migrates, this starts passing on its own,
# with no coordination between repos.
_wbr_gpsd_unmigrated_consumers() {
    local b out=""
    for b in "$@"; do
        [[ -n "$b" && -x "$b" ]] || continue
        strings "$b" 2>/dev/null | grep -q "wbr-gpsd" && continue
        if strings "$b" 2>/dev/null | grep -qE "/dev/ttyACM|serial/by-id"; then
            out+="${out:+, }$(basename "$b")"
        fi
    done
    [[ -n "$out" ]] && echo "$out"
    return 0
}

wbr_gpsd_session_start() {
    local bin
    if ! bin="$(_wbr_gpsd_binary)"; then
        echo "[gps] wbr-gpsd not found -- GPS will be unavailable" >&2
        return 0                      # never block a launcher over GPS
    fi
    if [[ ! -d "$WBR_GPSD_DIR" ]]; then
        echo "[gps] $WBR_GPSD_DIR missing -- install tmpfiles/wbr-gps.conf into" >&2
        echo "[gps] /etc/tmpfiles.d and run 'sudo systemd-tmpfiles --create'." >&2
        return 0
    fi
    if [[ ! -w "$WBR_GPSD_DIR" ]]; then
        echo "[gps] $WBR_GPSD_DIR not writable by $(id -un) -- GPS unavailable" >&2
        return 0
    fi

    # Register BEFORE starting anything, so a watchdog that is already running
    # is unlikely to observe an empty holders directory and shut down during a
    # handover. This NARROWS that race, it does not close it: the watchdog's
    # prune is a point-in-time glob, so a departing one can still decide the
    # directory is empty in the instant before this file appears and take the
    # daemon with it. Closing it properly needs an atomic handshake (a lock the
    # watchdog holds while deciding, or a rendezvous fd), which is more
    # machinery than the failure deserves -- the loser is a launcher that finds
    # no daemon and starts its own, so the outcome is a brief GPS gap, never a
    # held-by-nothing device. Raised by an independent audit after an earlier
    # version of this comment claimed the race was eliminated.
    mkdir -p "$WBR_GPSD_HOLDERS" 2>/dev/null || true
    : >"$WBR_GPSD_HOLDERS/$$" 2>/dev/null || true
    WBR_GPSD_HELD=1

    # Refuse to claim the device if it would starve a consumer that still opens
    # it directly. Checked BEFORE the already-running test: if another launcher
    # started a daemon the damage is already done, and stealing it back helps
    # nobody.
    local stale
    stale="$(_wbr_gpsd_unmigrated_consumers "${WBR_GPSD_CONSUMERS[@]}")"
    if [[ -n "$stale" ]]; then
        echo "[gps] NOT starting wbr-gpsd: $stale still open the GPS device" >&2
        echo "[gps] directly. Running the daemon would take the port and leave" >&2
        echo "[gps] them with no GPS. They keep working as before; migrate them" >&2
        echo "[gps] to wbr_gps::Client and this starts automatically." >&2
        return 0
    fi

    if _wbr_gpsd_answering; then
        echo "[gps] wbr-gpsd already running -- joined as holder $$" >&2
        return 0
    fi

    # Not running: start watchdog + daemon, detached from this launcher's
    # process group so it survives us and is governed only by the holder count.
    setsid bash -c '
        SOCK="$1"; HOLDERS="$2"; BIN="$3"; DIR="$4"
        setpriv --pdeathsig TERM "$BIN" \
            --socket "$SOCK" --lock "$DIR/wbr-gpsd.pid" --group "" \
            >/dev/null 2>&1 &
        DPID=$!
        while :; do
            sleep 1
            kill -0 "$DPID" 2>/dev/null || exit 0     # daemon gone; nothing to guard
            alive=0
            for f in "$HOLDERS"/*; do
                [ -e "$f" ] || continue
                pid=${f##*/}
                # A holder killed with -9 never removed its file. Prune it.
                if kill -0 "$pid" 2>/dev/null; then alive=1; else rm -f "$f"; fi
            done
            if [ "$alive" -eq 0 ]; then
                kill -TERM "$DPID" 2>/dev/null
                exit 0                                 # pdeathsig backstops this
            fi
        done
    ' _ "$WBR_GPSD_SOCK" "$WBR_GPSD_HOLDERS" "$bin" "$WBR_GPSD_DIR" \
      >/dev/null 2>&1 &
    disown 2>/dev/null || true

    local i
    for i in $(seq 1 20); do
        _wbr_gpsd_answering && { echo "[gps] wbr-gpsd started, holder $$" >&2; return 0; }
        sleep 0.25
    done
    echo "[gps] wbr-gpsd did not come up -- GPS will be unavailable" >&2
    return 0
}

# Deregister. The daemon goes away when the LAST holder does, so this is not
# "stop the daemon" -- it is "I no longer need it". The watchdog decides.
wbr_gpsd_session_stop() {
    [[ "$WBR_GPSD_HELD" == "1" ]] || return 0
    rm -f "$WBR_GPSD_HOLDERS/$$" 2>/dev/null || true
    WBR_GPSD_HELD=0
}
