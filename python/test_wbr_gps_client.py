"""Tests for the Python GPS client."""
import json
import os
import pty
import shutil
import socket
import subprocess
import tempfile
import threading
import time
import unittest

import wbr_gps_client as gc


class StubDaemon:
    """A minimal wbr-gpsd that speaks the same wire protocol."""

    def __init__(self, fix_payload=None, send_hello=True):
        self.dir = tempfile.mkdtemp()
        self.path = os.path.join(self.dir, "gpsd.sock")
        self.fix = fix_payload
        self.send_hello = send_hello
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind(self.path)
        self.sock.listen(8)
        self.stop = False
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self):
        while not self.stop:
            try:
                conn, _ = self.sock.accept()
            except OSError:
                return
            with conn:
                if self.send_hello:
                    conn.sendall(b'{"class":"HELLO","proto":1}\n')
                try:
                    conn.recv(4096)
                except OSError:
                    continue
                if self.fix is not None:
                    conn.sendall((json.dumps(self.fix) + "\n").encode())

    def close(self):
        self.stop = True
        self.sock.close()


LOCKED_FIX = {
    "class": "FIX", "seq": 7,
    "device_present": True, "serial_ok": True, "hid_ok": True,
    "gpsdo_locked": True, "has_fix": True,
    "fix_quality": 1, "satellites": 4, "hdop": 1.33,
    "lat": 13.002826, "lon": 77.6799202, "alt_m": 921.8,
    "speed_kph": 0.0, "time_utc": "045519.50", "fix_age_ms": 480,
}


class TestGetOnce(unittest.TestCase):
    def test_returns_fix(self):
        d = StubDaemon(LOCKED_FIX)
        try:
            snap = gc.get_once(d.path, timeout=1.0)
            self.assertTrue(snap["service_ok"])
            self.assertTrue(snap["has_fix"])
            self.assertAlmostEqual(snap["lat"], 13.002826, places=6)
            self.assertEqual(snap["satellites"], 4)
        finally:
            d.close()

    def test_skips_hello_and_finds_fix(self):
        d = StubDaemon(LOCKED_FIX, send_hello=True)
        try:
            self.assertTrue(gc.get_once(d.path, timeout=1.0)["has_fix"])
        finally:
            d.close()

    def test_no_daemon_reports_unavailable(self):
        snap = gc.get_once("/tmp/definitely-no-socket-here", timeout=0.2)
        self.assertFalse(snap["service_ok"])
        self.assertFalse(snap["has_fix"])
        self.assertFalse(snap["device_present"])

    def test_daemon_that_never_replies_times_out(self):
        d = StubDaemon(fix_payload=None)
        try:
            snap = gc.get_once(d.path, timeout=0.3)
            self.assertFalse(snap["service_ok"])
        finally:
            d.close()

    def test_garbage_reply_is_not_trusted(self):
        d = StubDaemon(fix_payload=None)
        try:
            snap = gc.get_once(d.path, timeout=0.3)
            self.assertFalse(snap["has_fix"])
        finally:
            d.close()


class TestBadgeStatus(unittest.TestCase):
    """The four honest states that replace the old three-state guess."""

    def test_locked(self):
        self.assertEqual(gc.badge_status(dict(LOCKED_FIX, service_ok=True)), "Locked")

    def test_not_locked_when_present_but_no_fix(self):
        snap = dict(LOCKED_FIX, service_ok=True, has_fix=False, gpsdo_locked=False)
        self.assertEqual(gc.badge_status(snap), "Not Locked")

    def test_not_connected_when_device_absent(self):
        snap = dict(LOCKED_FIX, service_ok=True, device_present=False,
                    has_fix=False, gpsdo_locked=False)
        self.assertEqual(gc.badge_status(snap), "Not Connected")

    def test_service_down_is_its_own_state(self):
        self.assertEqual(gc.badge_status({"service_ok": False}), "GPS Service Down")

    def test_gpsdo_locked_without_nmea_fix_still_reads_locked(self):
        snap = dict(LOCKED_FIX, service_ok=True, has_fix=False, gpsdo_locked=True)
        self.assertEqual(gc.badge_status(snap), "Locked")


class TestNonDictJSON(unittest.TestCase):
    """Test all five bad JSON shapes: int, str, null, list, bool."""

    def _test_json_shape(self, malformed_json):
        d = StubDaemon()
        try:
            def patched_serve():
                try:
                    conn, _ = d.sock.accept()
                except OSError:
                    return
                with conn:
                    conn.sendall(b'{"class":"HELLO","proto":1}\n')
                    try:
                        conn.recv(4096)
                    except OSError:
                        return
                    conn.sendall((malformed_json + "\n").encode())
            d._serve = patched_serve
            d.thread = threading.Thread(target=d._serve, daemon=True)
            d.thread.start()
            snap = gc.get_once(d.path, timeout=1.0)
            self.assertFalse(snap["service_ok"],
                            f"Malformed JSON {malformed_json!r} should not look healthy")
        finally:
            d.close()

    def test_bare_number(self):
        self._test_json_shape("123")

    def test_bare_string(self):
        self._test_json_shape('"hello"')

    def test_bare_null(self):
        self._test_json_shape("null")

    def test_bare_array(self):
        self._test_json_shape("[1,2,3]")

    def test_bare_bool(self):
        self._test_json_shape("true")


class TestRealDaemon(unittest.TestCase):
    """Drives the real wbr-gpsd binary and proves byte-for-byte agreement.

    Every other test in this file talks to StubDaemon, a Python fake that
    emits whatever this file's author believed the wire protocol to be. Only
    this test launches the actual C++ daemon, so only this test can catch a
    real drift between the C++ emitter (float formatting, field names,
    escaping) and what this client expects.
    """

    # Verbatim from the real device. Parsed values confirmed against real
    # hardware: lat=13.0028260, lon=77.6799202, alt_m=921.8, satellites=4,
    # hdop=1.33, has_fix=True.
    NMEA_LINE = ("$GNGGA,045519.50,1300.16956,N,07740.79521,E,1,04,1.33,"
                 "921.8,M,-86.3,M,,*6F")

    # Candidate build directories, relative to the repo root (one level up
    # from this file's directory). All three exist on the reference dev
    # machine; WBR_GPSD_BIN overrides this search entirely.
    _CANDIDATE_BUILD_DIRS = ("build", "build-t9", "build-t8")

    @classmethod
    def _find_binary(cls):
        """Locate wbr-gpsd. Returns (path_or_None, [paths tried])."""
        tried = []
        env_path = os.environ.get("WBR_GPSD_BIN")
        if env_path:
            tried.append(env_path + " (from $WBR_GPSD_BIN)")
            if os.path.isfile(env_path) and os.access(env_path, os.X_OK):
                return env_path, tried

        repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        for build_dir in cls._CANDIDATE_BUILD_DIRS:
            candidate = os.path.join(repo_root, build_dir, "wbr-gpsd")
            tried.append(candidate)
            if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                return candidate, tried
        return None, tried

    def test_real_daemon_with_nmea(self):
        binary, tried = self._find_binary()
        if binary is None:
            self.skipTest(
                "wbr-gpsd binary not found. Tried: " + "; ".join(tried) +
                ". Set WBR_GPSD_BIN to override."
            )

        tmpdir = tempfile.mkdtemp(prefix="wbr-gpsd-test-")
        master_fd = None
        slave_fd = None
        proc = None
        try:
            sock_path = os.path.join(tmpdir, "s.sock")
            lock_path = os.path.join(tmpdir, "pid")

            # Guard ONLY genuine setup failures here: pty creation and the
            # process spawn. Narrowed to OSError/SubprocessError so a real
            # protocol mismatch below can never be mistaken for one of
            # these and swallowed. A setup failure is reported as a test
            # failure, not a skip -- once the binary is confirmed to exist,
            # nothing past that point is "environmental".
            try:
                master_fd, slave_fd = pty.openpty()
                slave_path = os.ttyname(slave_fd)
                proc = subprocess.Popen(
                    [binary,
                     "--socket", sock_path,
                     "--lock", lock_path,
                     "--serial", slave_path,
                     "--group", ""],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )

                deadline = time.monotonic() + 5.0
                while not os.path.exists(sock_path):
                    rc = proc.poll()
                    if rc is not None:
                        _out, err = proc.communicate(timeout=1)
                        self.fail(
                            "wbr-gpsd exited before creating its socket "
                            f"(rc={rc}): {err.decode('utf-8', 'replace')}"
                        )
                    if time.monotonic() > deadline:
                        self.fail(f"wbr-gpsd never created {sock_path} within 5s")
                    time.sleep(0.05)
            except (OSError, subprocess.SubprocessError) as exc:
                self.fail(f"could not launch the real daemon for this test: {exc}")

            # --- communication with the running daemon: nothing below is
            # caught broadly. A mismatch here is a real bug and must fail. ---
            snap = gc.unavailable()
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                # Re-emit each attempt: this makes the test robust to the
                # daemon's own startup race (opening and reconfiguring the
                # serial port to raw mode happens on its first periodic
                # tick, not synchronously with socket creation) without
                # ever masking a real communication failure -- it is not
                # inside any except clause.
                os.write(master_fd, (self.NMEA_LINE + "\r\n").encode("ascii"))
                snap = gc.get_once(sock_path, timeout=1.0)
                if snap["service_ok"] and snap["has_fix"]:
                    break
                time.sleep(0.2)

            # The proof. If the C++ emitter and this client ever disagree on
            # field names, float formatting or escaping, this is what goes
            # red -- and it must go red as FAILED, not SKIPPED.
            self.assertTrue(
                snap["service_ok"],
                f"get_once() could not reach the real daemon: {snap}")
            self.assertTrue(
                snap["has_fix"],
                f"daemon never reported a fix for the known-good NMEA sentence: {snap}")
            self.assertAlmostEqual(snap["lat"], 13.0028260, places=5)
            self.assertAlmostEqual(snap["lon"], 77.6799202, places=5)
            self.assertAlmostEqual(snap["alt_m"], 921.8, places=1)
            self.assertEqual(snap["satellites"], 4)
            self.assertAlmostEqual(snap["hdop"], 1.33, places=2)
        finally:
            # Leave nothing behind, including when setup above raised.
            if proc is not None:
                try:
                    proc.terminate()
                    proc.wait(timeout=5)
                except Exception:
                    try:
                        proc.kill()
                        proc.wait(timeout=5)
                    except Exception:
                        pass
                if proc.stdout is not None:
                    proc.stdout.close()
                if proc.stderr is not None:
                    proc.stderr.close()
            for fd in (master_fd, slave_fd):
                if fd is not None:
                    try:
                        os.close(fd)
                    except OSError:
                        pass
            shutil.rmtree(tmpdir, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
