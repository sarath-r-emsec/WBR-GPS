#!/usr/bin/env bash
# Install wbr-gpsd, its udev rules and its systemd unit. Requires root.
#
# Safe to re-run: every install step below is either naturally idempotent
# (install(1) overwrites, udevadm reload/trigger are stateless, `systemctl
# enable` on an already-enabled unit is a no-op) or explicitly guarded (the
# 99-leobodnar.rules supersession only fires if that file still exists).
# The one thing a naive re-run would get wrong -- deploying a rebuilt binary
# to a daemon that is already running the old one in memory -- is handled
# below by an explicit `restart`, not just `enable --now`.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BINARY="$HERE/build/wbr-gpsd"

if [[ $EUID -ne 0 ]]; then
  echo "install.sh must run as root (try: sudo $0)" >&2
  exit 1
fi

cat <<'EOF' >&2
================================================================================
 WARNING: after this daemon starts, it becomes the SOLE owner of the GPSDO.
 Any other program still opening /dev/ttyACM0 or /dev/gpsdo directly --
 phase2_se and anything else not yet migrated to the wbr-gps client library
 (Tasks 13-16) -- will start failing LOUDLY with EBUSY instead of silently
 stealing bytes from the daemon. That is the fix working as designed, not a
 regression, but it is a visible behaviour change starting the moment this
 script enables the service. Confirm those programs are expected to fail,
 or that this is an acceptable maintenance window, before continuing.
================================================================================
EOF

echo "==> checking build/wbr-gpsd"
if [[ ! -x "$BINARY" ]]; then
  echo "build/wbr-gpsd not found. Run: cmake -S '$HERE' -B '$HERE/build' && cmake --build '$HERE/build'" >&2
  exit 1
fi

# A binary that predates its own sources is a silent-wrong-answer risk: the
# installer would happily deploy code that does not match what is on disk.
# `find -newer` is a coarse mtime check, not a hash, but it is enough to
# catch "I edited a .cpp and forgot to rebuild."
stale_src="$(find "$HERE/src" "$HERE/include" "$HERE/CMakeLists.txt" -type f \
  \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name 'CMakeLists.txt' \) \
  -newer "$BINARY" 2>/dev/null | head -n1)"
if [[ -n "$stale_src" ]]; then
  echo "build/wbr-gpsd is OLDER than $stale_src -- rebuild before installing:" >&2
  echo "  cmake --build '$HERE/build'" >&2
  exit 1
fi

echo "==> pre-flight: validating the udev rules file"
udevadm verify "$HERE/udev/99-wbr-gps.rules"

echo "==> installing binary"
install -m 0755 "$BINARY" /usr/local/bin/wbr-gpsd

echo "==> pre-flight: validating the systemd unit (binary now in place)"
systemd-analyze verify "$HERE/systemd/wbr-gpsd.service" || {
  echo "systemd-analyze verify failed -- not touching udev rules or the unit." >&2
  exit 1
}

echo "==> installing udev rules"
install -m 0644 "$HERE/udev/99-wbr-gps.rules" /etc/udev/rules.d/99-wbr-gps.rules
# The old rule is superseded by ours; keep a backup rather than deleting.
# Guarded so a second run of this script is a no-op here, not a failure.
if [[ -f /etc/udev/rules.d/99-leobodnar.rules ]]; then
  mv /etc/udev/rules.d/99-leobodnar.rules /etc/udev/rules.d/99-leobodnar.rules.superseded
  echo "    moved 99-leobodnar.rules aside (superseded by 99-wbr-gps.rules)"
fi
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty --subsystem-match=hidraw

echo "==> installing systemd unit"
install -m 0644 "$HERE/systemd/wbr-gpsd.service" /etc/systemd/system/wbr-gpsd.service
systemctl daemon-reload
systemctl enable wbr-gpsd.service
# restart, not `enable --now`: on a first install this starts the daemon
# same as --now would; on a re-run after a rebuild, --now is a no-op against
# an already-running unit and would leave the OLD binary resident in memory
# even though a new one just landed at /usr/local/bin/wbr-gpsd. restart
# always picks up whatever is on disk right now.
systemctl restart wbr-gpsd.service

echo "==> automated smoke checks"
sleep 2
if ! systemctl is-active --quiet wbr-gpsd.service; then
  echo "wbr-gpsd.service is not active after restart -- installation left the" >&2
  echo "system CONFIGURED (unit installed, udev rules active, enabled) but the" >&2
  echo "daemon itself is not running. Check: journalctl -u wbr-gpsd -n 50" >&2
  exit 1
fi
echo "    wbr-gpsd.service is active"

if [[ -L /dev/gpsdo ]]; then
  echo "    /dev/gpsdo symlink present -> $(readlink -f /dev/gpsdo 2>/dev/null || echo '?')"
else
  echo "    WARNING: /dev/gpsdo symlink not present yet (device unplugged, or" >&2
  echo "    udev hasn't settled). This is not necessarily fatal -- the daemon" >&2
  echo "    retries with backoff -- but check it before trusting the fix." >&2
fi

cat <<'EOF'

==> install.sh cannot verify everything for you -- it ran only the smoke
    checks above. Run these yourself and read the output; do not assume
    success from this script's exit code alone:

  # 1. The symlink exists and points at the real tty.
  ls -l /dev/gpsdo

  # 2. ModemManager now ignores it (expect ID_MM_DEVICE_IGNORE=1). If
  #    ID_MM_CANDIDATE is still 1 after this script's udevadm trigger, the
  #    running ModemManager instance may need `systemctl restart ModemManager`
  #    or a physical replug to pick up the new rule.
  udevadm info -q property -n /dev/gpsdo | grep -E 'ID_MM_DEVICE_IGNORE|ID_MM_CANDIDATE'

  # 3. Exactly one opener, and it is wbr-gpsd.
  lsof "$(readlink -f /dev/gpsdo)"

  # 4. The socket exists with the right ownership (expect srw-rw---- root dialout).
  ls -l /run/wbr-gps/gpsd.sock

  # 5. It answers (expect a HELLO line, then a FIX line).
  printf '{"op":"get"}\n' | socat - UNIX-CONNECT:/run/wbr-gps/gpsd.sock

  # 6. Restart-on-crash (optional but recommended before trusting G5):
  sudo kill -9 "$(systemctl show -p MainPID --value wbr-gpsd)"
  sleep 4 && systemctl is-active wbr-gpsd && lsof "$(readlink -f /dev/gpsdo)"

Reminder: any program still opening the GPSDO directly (not yet migrated to
the wbr-gps client) will now see EBUSY. That is expected until Tasks 13-16
land.
EOF
