#!/usr/bin/env bash
# Install wbr-gpsd. File installation, udev/tmpfiles/systemd wiring, and
# their reload hooks all go through `cmake --install` now (CMakeLists.txt's
# WBR_GPS_SYSTEM_SERVICE option and its runtime/service COMPONENTs) -- this
# script is a thin wrapper around it, keeping only what a CMake install()
# rule cannot reasonably express: root/EUID checks, pre-flight verification
# BEFORE touching /etc (or ~/.config), the one-time 99-leobodnar.rules
# migration, and a live smoke test afterwards.
#
# Two modes:
#   sudo ./install.sh            system-wide (default): udev rule, tmpfiles.d,
#                                 a system systemd unit. Needs root. Socket at
#                                 /run/wbr-gps/gpsd.sock.
#   ./install.sh --user [--prefix DIR]
#                                 per-user, no root ANYWHERE (configure,
#                                 build, or install): a `systemctl --user`
#                                 unit, socket under $XDG_RUNTIME_DIR/wbr-gps.
#                                 DIR defaults to $HOME/.local. No udev rule --
#                                 see CMakeLists.txt's WBR_GPS_SYSTEM_SERVICE
#                                 message for the (non-fatal) trade-offs.
#
# Re-running either mode is not destructive -- install() overwrites, the
# reload hooks are stateless, `systemctl daemon-reload` on unchanged unit
# files is a no-op -- but it does NOT restart an already-running daemon.
# Deploying a rebuilt binary to a live daemon needs an explicit
# `sudo systemctl restart wbr-gpsd` (or `systemctl --user restart wbr-gpsd`)
# afterwards; this script will not bounce a healthy daemon out from under its
# connected clients on your behalf.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="$HERE/build"
BINARY="$BUILD/wbr-gpsd"

MODE=system
PREFIX=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --user) MODE=user ;;
    --prefix)
      [[ $# -ge 2 ]] || { echo "--prefix needs an argument" >&2; exit 1; }
      PREFIX="$2"
      shift
      ;;
    -h|--help)
      echo "usage: $0 [--user [--prefix DIR]]" >&2
      exit 0
      ;;
    *)
      echo "unknown argument: $1 (usage: $0 [--user [--prefix DIR]])" >&2
      exit 1
      ;;
  esac
  shift
done

if [[ -n "$PREFIX" && "$MODE" != user ]]; then
  echo "--prefix only applies to --user mode (system mode always installs the" >&2
  echo "binary to the standard /usr/local/bin; the system unit's ExecStart is" >&2
  echo "not templated)." >&2
  exit 1
fi

if [[ "$MODE" == system ]]; then
  if [[ $EUID -ne 0 ]]; then
    echo "install.sh (system mode) must run as root (try: sudo $0), or use" >&2
    echo "'$0 --user' for a no-root per-user install." >&2
    exit 1
  fi
else
  if [[ $EUID -eq 0 ]]; then
    echo "$0 --user must NOT run as root -- it installs into YOUR ~/.config" >&2
    echo "and \$HOME/.local, not root's. Run it as your normal user." >&2
    exit 1
  fi
fi

if [[ "$MODE" == system ]]; then
cat <<'EOF' >&2
================================================================================
 System-wide install. This installs files and reloads udev/tmpfiles/systemd
 config -- it does NOT start or enable anything: the daemon is launched by
 whichever program needs it (bash SIGINT, ./run_rtsa.sh) and dies with it, so
 the GPSDO stays free the rest of the time. For an always-on deployment
 instead, run this once, then: sudo systemctl enable --now wbr-gpsd
 (do that only once every consumer on the machine reads GPS from the daemon).

 WHILE the daemon runs it is the SOLE owner of the GPSDO, and any program
 still opening /dev/ttyACM0 or /dev/gpsdo directly gets EBUSY. The launchers
 check for that and refuse to start the daemon while any consumer they run
 is still unmigrated -- but if you start the daemon by hand, that protection
 is not there.

 SEPARATELY: the udev rule retightens /dev/hidraw* for vendor 1dd2 from mode
 0666 (the old, permissive 99-leobodnar.rules) to 0660 group dialout. That
 lands the moment the udevadm trigger step below runs, BEFORE the daemon or
 its unit are even installed, and regardless of whether the daemon ever
 starts successfully. Nothing here replaces the old access model on hidraw
 the way the tty symlink does for the serial port: the daemon reads it but
 does not become its sole owner. Any non-dialout-group program reading
 hidraw loses access as soon as this runs, whether or not the rest of the
 install succeeds.

 This does NOT restart an already-running daemon (see this file's header
 comment) -- rerun with `sudo systemctl restart wbr-gpsd` afterwards if one
 is live and you rebuilt.
================================================================================
EOF
else
cat <<'EOF' >&2
================================================================================
 Per-user install, no root anywhere. Installs a `systemctl --user` unit; the
 socket/lock file live under $XDG_RUNTIME_DIR/wbr-gps, created by the daemon
 itself on first bind. No udev rule in this mode -- see the
 WBR_GPS_SYSTEM_SERVICE=OFF message this script's `cmake` step prints for
 exactly what that costs (all non-fatal: the daemon degrades, it does not
 fail to start).

 This does NOT enable or start the unit for you: `systemctl --user enable
 --now wbr-gpsd` once this finishes.
================================================================================
EOF
fi

echo "==> configuring + building"
CMAKE_ARGS=()
if [[ "$MODE" == user ]]; then
  CMAKE_ARGS+=(-DWBR_GPS_SYSTEM_SERVICE=OFF)
  CMAKE_ARGS+=(-DCMAKE_INSTALL_PREFIX="${PREFIX:-$HOME/.local}")
fi
cmake -S "$HERE" -B "$BUILD" "${CMAKE_ARGS[@]}"
cmake --build "$BUILD"

if [[ "$MODE" == system ]]; then
  echo "==> pre-flight: validating the udev rules file"
  udevadm verify "$HERE/udev/99-wbr-gps.rules" || {
    echo "udevadm verify failed -- not touching /etc at all." >&2
    exit 1
  }
fi

echo "==> installing binary"
cmake --install "$BUILD" --component runtime

echo "==> pre-flight: validating the systemd unit (binary now in place)"
if [[ "$MODE" == system ]]; then
  systemd-analyze verify "$HERE/systemd/wbr-gpsd.service" || {
    echo "systemd-analyze verify failed -- not touching udev rules or the unit." >&2
    exit 1
  }
else
  # Generated by configure_file() into the build dir at the `cmake -S/-B`
  # step above, with ExecStart already resolved to CMAKE_INSTALL_FULL_BINDIR
  # -- see CMakeLists.txt's WBR_GPS_SYSTEM_SERVICE=OFF branch.
  systemd-analyze --user verify "$BUILD/wbr-gpsd.service" || {
    echo "systemd-analyze verify failed -- not installing the unit." >&2
    exit 1
  }
fi

if [[ "$MODE" == system ]]; then
  # The old rule is superseded by ours; keep a backup rather than deleting.
  # Guarded so a second run of this script is a no-op here, not a failure.
  if [[ -f /etc/udev/rules.d/99-leobodnar.rules ]]; then
    mv /etc/udev/rules.d/99-leobodnar.rules /etc/udev/rules.d/99-leobodnar.rules.superseded
    echo "    moved 99-leobodnar.rules aside (superseded by 99-wbr-gps.rules)"
  fi
fi

echo "==> installing udev rules / tmpfiles / systemd unit (and reloading them)"
cmake --install "$BUILD" --component service

if [[ "$MODE" == system ]]; then
  echo "    /run/wbr-gps: $(stat -c '%A %U:%G' /run/wbr-gps 2>/dev/null || echo MISSING)"
  # Deliberately NOT enabled -- see the banner above.
  systemctl disable wbr-gpsd.service >/dev/null 2>&1 || true
  systemctl stop    wbr-gpsd.service >/dev/null 2>&1 || true
  echo "    installed but not enabled (per-launcher model)"
fi

echo "==> automated smoke checks"
# Poll rather than a flat sleep: a fixed `sleep 2` reports a false failure
# for a daemon that is merely slow to come up. Bounded at 10s so a
# genuinely dead binary still fails promptly. Nothing runs by design at this
# point, so start a throwaway daemon exactly as a launcher would, on its own
# socket/lock so it cannot collide with a real instance, prove it works, and
# stop it -- verifying the install must not leave the device held.
SMOKE_DIR="$(mktemp -d)"
"$BINARY" --socket "$SMOKE_DIR/s.sock" --lock "$SMOKE_DIR/pid" --group "" \
    >/dev/null 2>&1 &
SMOKE_PID=$!
active=0
for _ in $(seq 1 10); do
  if [[ -S "$SMOKE_DIR/s.sock" ]] && kill -0 "$SMOKE_PID" 2>/dev/null; then
    active=1
    break
  fi
  sleep 1
done
kill -TERM "$SMOKE_PID" 2>/dev/null || true
sleep 1
kill -9 "$SMOKE_PID" 2>/dev/null || true
rm -rf "$SMOKE_DIR"
if [[ "$active" -ne 1 ]]; then
  echo "wbr-gpsd did not come up 10s after starting -- installation left the" >&2
  echo "system CONFIGURED (files installed) but the daemon itself would not" >&2
  echo "start. Run it by hand to see why:" >&2
  echo "  $BINARY --socket /tmp/t.sock --lock /tmp/t.pid --group ''" >&2
  if [[ "$MODE" == system ]]; then
    echo "Note: /dev/hidraw* for vendor 1dd2 is ALREADY retightened to 0660" >&2
    echo "group dialout (was 0666 under the old 99-leobodnar.rules) -- that took" >&2
    echo "effect regardless of this failure, and nothing has taken over hidraw" >&2
    echo "access to compensate." >&2
  fi
  exit 1
fi
echo "    daemon starts and answers (test instance started and stopped)"

if [[ "$MODE" == system ]]; then
  if [[ -L /dev/gpsdo ]]; then
    echo "    /dev/gpsdo symlink present -> $(readlink -f /dev/gpsdo 2>/dev/null || echo '?')"
  else
    echo "    WARNING: /dev/gpsdo symlink not present yet (device unplugged, or" >&2
    echo "    udev hasn't settled). This is not necessarily fatal -- the daemon" >&2
    echo "    retries with backoff -- but check it before trusting the fix." >&2
  fi
fi

if [[ "$MODE" == system ]]; then
cat <<EOF

==> install.sh cannot verify everything for you -- it ran only the smoke
    checks above. Run these yourself and read the output; do not assume
    success from this script's exit code alone:

  # 1. The symlink exists and points at the real tty.
  ls -l /dev/gpsdo

  # 2. ModemManager now ignores it (expect ID_MM_DEVICE_IGNORE=1). If
  #    ID_MM_CANDIDATE is still 1 after this script's udevadm trigger, the
  #    running ModemManager instance may need \`systemctl restart ModemManager\`
  #    or a physical replug to pick up the new rule.
  udevadm info -q property -n /dev/gpsdo | grep -E 'ID_MM_DEVICE_IGNORE|ID_MM_CANDIDATE'

  # 3. NOTHING should hold the device right now. Nothing is running.
  #    Do not use lsof for this: the daemon runs as another user when
  #    started by systemd, and lsof then reports zero openers whether the
  #    device is held or not. Ask the kernel instead -- expect it to SUCCEED
  #    here, because nothing should own the port yet.
  python3 -c "import os; os.close(os.open('/dev/gpsdo', os.O_RDWR|os.O_NOCTTY)); print('free')"

  # 4. Start something that needs GPS, and watch the daemon appear.
  cd /path/to/SIGINT_GUI && bash SIGINT
  #    expect: "[gps] wbr-gpsd started, holder <pid>"

  # 5. While it runs, the daemon answers (expect HELLO then FIX):
  tools/gpsq

  # 6. And the device is now held -- the same probe must now FAIL with EBUSY:
  python3 -c "import os; os.open('/dev/gpsdo', os.O_RDWR|os.O_NOCTTY)"

  # 7. Stop the launcher. Within ~2s the daemon exits and step 3 succeeds
  #    again -- proving the GPSDO is released for anything else to use.

Reminder: the daemon is NOT a service here. If you want the always-on model
instead -- a headless box where everything reads GPS from the daemon -- the
unit is installed and one command away:

  sudo systemctl enable --now wbr-gpsd

Do that only once every consumer on the machine is migrated; an idle daemon
holding the port starves anything that still opens the device directly.
EOF
else
cat <<EOF

==> per-user install done. This did not enable or start anything:

  systemctl --user enable --now wbr-gpsd
  systemctl --user status wbr-gpsd
  WBR_GPS_SOCKET_DIR="\${XDG_RUNTIME_DIR}/wbr-gps" tools/gpsq

Trade-offs vs the system install (no udev rule in this mode) are printed
above by the cmake configure step; none of them are fatal.
EOF
fi
