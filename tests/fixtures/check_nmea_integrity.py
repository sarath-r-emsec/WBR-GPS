#!/usr/bin/env python3
"""Real NMEA integrity check for tests/regression_all_consumers.sh.

The original shape-only check -- a regex requiring a leading dollar sign,
five uppercase letters, a comma, then anything, then a star and two hex
digits at the end of the line -- verifies that a line LOOKS like one NMEA
sentence, but its greedy middle section happily matches a two-sentence
splice on one line (sentence-one's payload, a star, sentence-two's whole
body, then a final valid-looking checksum trailer). That is exactly the
shape of the original bug's corruption (see regression_all_consumers.sh's
header: reader B's shredded line was two GGA sentences run together), so a
check that cannot distinguish "one sentence" from "two sentences spliced
into one line" is not actually testing for the thing this daemon exists to
prevent.

This checks, per captured line:
  1. Exactly one '$' and exactly one '*' -- catches a splice directly: two
     sentences concatenated onto one line carries two of each.
  2. The five characters after '$' are the talker+sentence-type field
     (A-Z), followed by a comma.
  3. The two characters after '*' are hex digits, and there is nothing
     after them (matches the original shape check).
  4. The NMEA-0183 checksum -- XOR of every byte between '$' and '*' --
     equals that hex value. This is the check the original regex never
     did: shape-valid-but-corrupted data (which the daemon's passthrough
     deliberately forwards verbatim -- see gps_capture.cpp's own comment
     about not filtering parser-rejected sentences) passes shape but fails
     this.

Usage: check_nmea_integrity.py NMEA_LOG_FILE
Prints one PASS/FAIL summary line and, on failure, up to 20 offending
lines with their reason. Exit 0 iff every line in the file is valid.
"""
import sys


def check_line(line):
    """Return None if valid, else a short reason string."""
    if line.count("$") != 1:
        return f"expected exactly one '$', found {line.count('$')}"
    if line.count("*") != 1:
        return f"expected exactly one '*', found {line.count('*')}"
    if not line.startswith("$"):
        return "'$' is not the first character"
    star = line.index("*")
    payload = line[1:star]
    checksum_field = line[star + 1:]
    if len(payload) < 6 or not payload[:5].isalpha() or not payload[:5].isupper():
        return "no 5 uppercase letters after '$'"
    if payload[5:6] != ",":
        return "no comma after the 5-letter talker+sentence-type field"
    if len(checksum_field) != 2:
        return f"checksum field is not exactly 2 characters ('{checksum_field}')"
    if not all(c in "0123456789ABCDEFabcdef" for c in checksum_field):
        return f"checksum field '{checksum_field}' is not hex"
    computed = 0
    for ch in payload:
        computed ^= ord(ch)
    claimed = int(checksum_field, 16)
    if computed != claimed:
        return f"checksum mismatch: computed {computed:02X}, sentence says {checksum_field.upper()}"
    return None


def main(argv):
    if len(argv) != 2:
        print("usage: check_nmea_integrity.py NMEA_LOG_FILE", file=sys.stderr)
        return 2
    path = argv[1]
    try:
        with open(path, "r", errors="replace") as fh:
            lines = [ln.rstrip("\n") for ln in fh if ln.strip()]
    except OSError as exc:
        print(f"FAIL: cannot read {path}: {exc}", file=sys.stderr)
        return 1

    if not lines:
        print(f"FAIL: {path} has no captured lines", file=sys.stderr)
        return 1

    bad = []
    for i, line in enumerate(lines, 1):
        reason = check_line(line)
        if reason is not None:
            bad.append((i, reason, line))

    if not bad:
        print(f"PASS: all {len(lines)} captured lines are single, "
              f"well-formed, checksum-valid NMEA sentences")
        return 0

    print(f"FAIL: {len(bad)} of {len(lines)} captured lines failed integrity "
          f"checks (shape, splice, or checksum)", file=sys.stderr)
    for i, reason, line in bad[:20]:
        print(f"  line {i}: {reason}: {line}", file=sys.stderr)
    if len(bad) > 20:
        print(f"  ... and {len(bad) - 20} more", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
