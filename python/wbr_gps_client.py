"""Client for wbr-gpsd, the single owner of the Leo Bodnar GPSDO.

This module deliberately contains NO serial, hidraw or /dev access. If the
daemon is unavailable, `service_ok` is False and that is the complete answer.
Falling back to reading the device directly is what caused the contention
this daemon exists to remove.

Standard library only: this is imported by Django and must add no dependency.
"""
import json
import socket

DEFAULT_SOCKET = "/run/wbr-gps/gpsd.sock"

# The shape returned when we cannot reach the daemon. Every field is the
# pessimistic value, so a caller that ignores service_ok still cannot mistake
# a dead daemon for a healthy one.
_UNAVAILABLE = {
    "service_ok": False,
    "device_present": False,
    "serial_ok": False,
    "hid_ok": False,
    "gpsdo_locked": False,
    "has_fix": False,
    "fix_quality": 0,
    "satellites": 0,
    "hdop": 0.0,
    "lat": 0.0,
    "lon": 0.0,
    "alt_m": 0.0,
    "speed_kph": 0.0,
    "time_utc": "",
    "fix_age_ms": 0,
    "seq": 0,
}


def unavailable():
    """A fresh snapshot dict meaning "the daemon could not be reached".

    Public so callers can build the same pessimistic shape without reaching
    into module internals.
    """
    return dict(_UNAVAILABLE)


def get_once(sock_path=DEFAULT_SOCKET, timeout=0.5):
    """One snapshot from the daemon, as a dict.

    Always returns a dict; never raises. A Unix-socket round trip is
    sub-millisecond, so this is safe to call directly from a request thread
    without the background-refresh cache the old probe needed.

    Note: timeout bounds each individual recv() call, not the total wall-clock
    time, so a slow-trickling peer could block longer than the stated value.
    This is acceptable against a local cooperative daemon, but callers should
    be aware of this property.
    """
    sock = None
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect(sock_path)
        sock.sendall(b'{"op":"get"}\n')

        buf = b""
        while b"\n" in buf or len(buf) < 65536:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line.decode("utf-8", "replace"))
                except ValueError:
                    continue
                # Guard against non-dict JSON: bare number, string, null, array, bool.
                # Skip HELLO and anything else; only a FIX is an answer.
                if isinstance(msg, dict) and msg.get("class") == "FIX":
                    snap = dict(_UNAVAILABLE)
                    snap.update(msg)
                    snap["service_ok"] = True
                    snap.pop("class", None)
                    return snap
        return dict(_UNAVAILABLE)
    except (OSError, socket.timeout, AttributeError, TypeError):
        return dict(_UNAVAILABLE)
    finally:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


def badge_status(snap):
    """The header badge string.

    Four states, not three. The old code could not express "the GPS service
    is down" and so reported "Not Connected" or a stale "Locked" instead —
    a confident wrong answer. Order matters: service health is checked first,
    because every other field is meaningless without it.
    """
    if not snap.get("service_ok"):
        return "GPS Service Down"
    if not snap.get("device_present"):
        return "Not Connected"
    # The GPSDO lock bit and the NMEA fix are different signals; either one
    # being good means the disciplined clock is usable.
    if snap.get("gpsdo_locked") or snap.get("has_fix"):
        return "Locked"
    return "Not Locked"
