#!/usr/bin/env python3
"""Argument parsing for tools/gpsq.

tools/gpsq grew a --socket flag because every other binary in the project
takes --socket PATH, and reaching for it here produced a baffling error:
the flag was read as the socket path, so the failure printed
"connect --socket: No such file or directory" while a healthy daemon sat
on the socket the user had just been told about.

The positional form is load-bearing -- tests/soak.sh,
tests/regression_all_consumers.sh and tools/handoff-root.sh all call it
that way -- so the first block below pins the exact shapes those callers
use. If they ever break, they break here first and loudly, rather than
mid-soak at minute 40.

Run standalone (`python3 tests/test_gpsq_args.py`) or via `ctest -R gpsq_args`.
"""

import importlib.machinery
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GPSQ = os.path.join(HERE, os.pardir, "tools", "gpsq")


def load_gpsq():
    # tools/gpsq has no .py extension, so a plain import will not find it.
    loader = importlib.machinery.SourceFileLoader("gpsq_under_test", GPSQ)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    mod = importlib.util.module_from_spec(spec)
    loader.exec_module(mod)
    return mod


class Checker(object):
    def __init__(self, mod):
        self.mod = mod
        self.failures = []
        self.checked = 0

    def case(self, argv, expect):
        self.checked += 1
        try:
            got = self.mod.parse_args(["gpsq"] + argv)
        except ValueError as exc:
            got = ("ERR", exc.args[0])
        if got != expect:
            self.failures.append((argv, got, expect))


def main():
    mod = load_gpsq()
    c = Checker(mod)
    DS = mod.DEFAULT_SOCKET
    DR = mod.DEFAULT_REQUEST
    DT = mod.DEFAULT_SECONDS

    # The shapes existing callers use. These must never change meaning.
    c.case(["/run/wbr-gps/gpsd.sock", '{"op":"get"}', "1"],
           ("/run/wbr-gps/gpsd.sock", '{"op":"get"}', 1.0))
    c.case(["/tmp/x/s.sock", '{"op":"watch","fix":true}', "600"],
           ("/tmp/x/s.sock", '{"op":"watch","fix":true}', 600.0))
    c.case(["/tmp/x/s.sock", '{"op":"get"}', "2"],
           ("/tmp/x/s.sock", '{"op":"get"}', 2.0))
    c.case(["/tmp/x/s.sock", '{"op":"get"}'],
           ("/tmp/x/s.sock", '{"op":"get"}', DT))

    # The --socket spelling, with and without a request and a duration.
    c.case(["--socket", "/tmp/x/s.sock"], ("/tmp/x/s.sock", DR, DT))
    c.case(["--socket=/tmp/x/s.sock"], ("/tmp/x/s.sock", DR, DT))
    c.case(["--socket", "/tmp/x/s.sock", '{"op":"watch","nmea":true}'],
           ("/tmp/x/s.sock", '{"op":"watch","nmea":true}', DT))
    c.case(["--socket", "/tmp/x/s.sock", '{"op":"get"}', "3"],
           ("/tmp/x/s.sock", '{"op":"get"}', 3.0))

    # Defaults, so a bare `gpsq` does the obvious thing.
    c.case([], (DS, DR, DT))
    c.case(["/tmp/x/s.sock"], ("/tmp/x/s.sock", DR, DT))

    # Rejections. A path given twice is the mistake worth naming precisely:
    # taking it as a request would send the daemon garbage and report a
    # JSON error, pointing away from the actual problem.
    c.case(["--socket"], ("ERR", "--socket needs a path"))
    c.case(["--socket", "/a", "--socket", "/b"],
           ("ERR", "--socket given more than once"))
    c.case(["--socket", "/a", "/run/wbr-gps/gpsd.sock"],
           ("ERR", "path given twice: --socket /a and positional "
                   "'/run/wbr-gps/gpsd.sock'; use one or the other"))
    c.case(["--nope"], ("ERR", "unknown option: --nope"))
    c.case(["a", "b", "c", "d"], ("ERR", "too many arguments"))
    c.case(["--socket", "/a", '{"op":"get"}', "1", "extra"],
           ("ERR", "too many arguments after --socket"))
    c.case(["/s.sock", '{"op":"get"}', "abc"],
           ("ERR", "SECONDS must be a number, got 'abc'"))
    c.case(["/s.sock", '{"op":"get"}', "0"],
           ("ERR", "SECONDS must be positive, got '0'"))

    # --help is not an error; ValueError(None) is the sentinel main() reads
    # to print usage and exit 0.
    c.case(["-h"], ("ERR", None))
    c.case(["--help"], ("ERR", None))

    if c.failures:
        for argv, got, expect in c.failures:
            sys.stderr.write("FAIL %r\n  got    %r\n  expect %r\n"
                             % (argv, got, expect))
        sys.stderr.write("%d of %d cases failed\n"
                         % (len(c.failures), c.checked))
        return 1

    sys.stdout.write("gpsq args: %d cases passed\n" % c.checked)
    return 0


if __name__ == "__main__":
    sys.exit(main())
