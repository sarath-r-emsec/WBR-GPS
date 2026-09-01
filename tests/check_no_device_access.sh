#!/usr/bin/env bash
# Guarantee G5: the client libraries must contain no device access, so no
# consumer can fall back to opening the GPS directly and reintroduce the
# contention this daemon exists to remove.
#
# This is a STRUCTURAL check, not a behavioural one. Every other test in the
# suite asks "did the client do the right thing this time?"; this one asks
# "could it ever do the wrong thing at all?" A fallback path that only fires
# when the daemon is down is exactly the code a behavioural test is least
# likely to reach, and exactly the code that would put two readers back on
# the tty at the worst possible moment.
#
# Two properties this gate learned the hard way (T12-A, T12-C):
#
#   * It must cover REUSE, not just raw syscalls. Nobody in this codebase
#     reaches for termios; they reach for SerialSource, which is written,
#     tested and already linked into the same library target. A gate that
#     only knows libc and pyserial spellings is blind to the single most
#     likely way G5 gets broken.
#   * It must not match prose. Matching whole files means rewording a comment
#     to "contains no termios or ioctl access" turns the build red on correct
#     code, and a gate that cries wolf is deleted by the third person it
#     annoys -- after which G5 has no gate at all.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

exec python3 - "$HERE" <<'PY'
import re
import sys

root = sys.argv[1]

# The files a consumer links against or imports. Anything else in the tree may
# touch the device; these three may not.
FILES = [
    "include/wbr_gps/client.hpp",
    "src/client.cpp",
    "python/wbr_gps_client.py",
]

# --- what counts as device access -------------------------------------------
#
# Deliberately NOT the bare word "serial": the client legitimately reports
# Snapshot::serial_ok, which is the daemon's opinion about the port relayed
# over the socket, not an act of opening one.
RAW_SURFACES = [
    r"::open\(", r"[^_a-zA-Z]open\(", r"\bopen\(",
    r"termios", r"tcsetattr", r"tcgetattr",
    r"cfsetispeed", r"cfsetospeed", r"cfmakeraw",
    r"\bioctl\b", r"TIOCEXCL", r"O_NOCTTY", r"O_NDELAY",
    r"/dev/tty", r"/dev/hidraw", r"/dev/serial", r"/dev/gpsdo",
    r"serial\.Serial", r"^\s*import serial\b", r"from serial import",
    r"pyserial",
]

# The reuse surfaces. These are what a developer in THIS codebase would
# actually reach for, and none of them contains a libc spelling.
INTERNAL_SURFACES = [
    r"#\s*include\s*[<\"][^\">]*(serial_source|hid_source|device_presence)\.hpp",
    r"\bSerialSource\b", r"\bHidSource\b",
    r"\bopen_device\b", r"\bclose_device\b",
    r"\bkDefaultSerialPath\b",
    r"\bleo_bodnar_present\b", r"\bfind_leo_bodnar_serial\b",
]

PATTERNS = [(p, re.compile(p)) for p in RAW_SURFACES + INTERNAL_SURFACES]


def blank_cpp_comments(text):
    """Replace // and /* */ comments with spaces, preserving line numbers.

    String literals are preserved: a device path is a string, and blanking
    those would blind the gate to the very thing it looks for. Raw string
    literals (R"(...)") are not handled; this codebase uses none, and the
    consequence would be a false positive, which is the safe direction.
    """
    out, i, n = [], 0, len(text)
    state = "code"
    while i < n:
        c = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if state == "code":
            if c == "/" and nxt == "/":
                state = "line"; out.append("  "); i += 2; continue
            if c == "/" and nxt == "*":
                state = "block"; out.append("  "); i += 2; continue
            if c in "\"'":
                state = "str"; quote = c
            out.append(c); i += 1; continue
        if state == "str":
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(text[i + 1]); i += 2; continue
            if c == quote:
                state = "code"
            i += 1; continue
        if state == "line":
            if c == "\n":
                state = "code"; out.append("\n")
            else:
                out.append(" ")
            i += 1; continue
        # block
        if c == "*" and nxt == "/":
            state = "code"; out.append("  "); i += 2; continue
        out.append("\n" if c == "\n" else " ")
        i += 1
    return "".join(out)


def blank_py_comments(text):
    """Replace # comments and triple-quoted strings, preserving line numbers.

    Triple-quoted strings are treated as comments because in this codebase
    they are docstrings -- prose, not executable device access. Ordinary
    string literals are preserved, so a path in a string still trips the gate.
    """
    out, i, n = [], 0, len(text)
    state = "code"
    while i < n:
        c = text[i]
        three = text[i:i + 3]
        if state == "code":
            if three in ('"""', "'''"):
                state = "triple"; delim = three; out.append("   "); i += 3; continue
            if c == "#":
                state = "comment"; out.append(" "); i += 1; continue
            if c in "\"'":
                state = "str"; quote = c
            out.append(c); i += 1; continue
        if state == "str":
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(text[i + 1]); i += 2; continue
            if c == quote or c == "\n":
                state = "code"
            i += 1; continue
        if state == "comment":
            if c == "\n":
                state = "code"; out.append("\n")
            else:
                out.append(" ")
            i += 1; continue
        # triple
        if text[i:i + 3] == delim:
            state = "code"; out.append("   "); i += 3; continue
        out.append("\n" if c == "\n" else " ")
        i += 1
    return "".join(out)


fail = 0
for rel in FILES:
    path = root + "/" + rel
    try:
        with open(path, "r", errors="replace") as fh:
            text = fh.read()
    except OSError:
        print("MISSING: %s" % path, file=sys.stderr)
        fail = 1
        continue

    stripped = (blank_py_comments(text) if path.endswith(".py")
                else blank_cpp_comments(text))

    hits = []
    for lineno, line in enumerate(stripped.split("\n"), 1):
        for src, rx in PATTERNS:
            if rx.search(line):
                hits.append((lineno, src, text.split("\n")[lineno - 1].strip()))
                break
    if hits:
        for lineno, src, original in hits:
            print("%s:%d: matches /%s/: %s" % (rel, lineno, src, original))
        print("FAIL: %s contains device access (guarantee G5 violated)" % path,
              file=sys.stderr)
        fail = 1

if fail == 0:
    print("PASS: no device access in client libraries (G5 holds)")
sys.exit(fail)
PY
