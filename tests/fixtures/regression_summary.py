#!/usr/bin/env python3
"""Assertions 4 and 5 for tests/regression_all_consumers.sh: every consumer
agrees on position, and the GUI badge does not flap.

Reads the logs the three background pollers wrote (the GUI poller's JSON
lines, phase2_server's /api/gps JSON lines, and the gsm_monitor stand-in's
key=value lines) and does the float comparison and stability check in
Python rather than bash, because comparing floating-point lat/lon with
awk/bc is exactly the kind of thing that quietly gets it wrong.

Prints PASS:/FAIL:/NOTE: lines matching the calling script's convention.
Exit code 0 iff both assertions in this file passed.
"""
import argparse
import json
import sys


def read_gui(path):
    """[{"t":..,"badge":..,"has_fix":..,"lat":..,"lon":..,"service_ok":..}, ...]"""
    records = []
    try:
        with open(path) as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    records.append(json.loads(line))
                except ValueError:
                    continue
    except OSError:
        pass
    return records


def read_phase2(path):
    """Same shape as read_gui, translated from /api/gps's field names."""
    records = []
    try:
        with open(path) as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    d = json.loads(line)
                except ValueError:
                    continue
                records.append({
                    "has_fix": bool(d.get("has_position")),
                    "lat": d.get("lat"),
                    "lon": d.get("lon"),
                })
    except OSError:
        pass
    return records


def read_probe(path):
    """key=value per line -> same shape."""
    records = []
    try:
        with open(path) as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                fields = {}
                for tok in line.split():
                    if "=" not in tok:
                        continue
                    k, v = tok.split("=", 1)
                    fields[k] = v
                if "has_fix" not in fields:
                    continue
                try:
                    records.append({
                        "has_fix": fields.get("has_fix") == "1",
                        "lat": float(fields["lat"]),
                        "lon": float(fields["lon"]),
                    })
                except (KeyError, ValueError):
                    continue
    except OSError:
        pass
    return records


def last_fix(records):
    """The most recent record with has_fix true and numeric lat/lon, else None."""
    for r in reversed(records):
        if r.get("has_fix") and isinstance(r.get("lat"), (int, float)) \
                and isinstance(r.get("lon"), (int, float)):
            return r
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", required=True)
    ap.add_argument("--phase2", required=True)
    ap.add_argument("--probe", required=True)
    ap.add_argument("--tolerance-deg", type=float, required=True)
    args = ap.parse_args()

    failed = False

    # --- assertion: GUI badge is stable, no flapping ------------------------
    gui = read_gui(args.gui)
    if not gui:
        print("FAIL: no GUI polls captured at all", file=sys.stderr)
        failed = True
    else:
        badges = [r.get("badge") for r in gui if "badge" in r]
        distinct = sorted(set(badges))
        if len(distinct) == 1:
            print(f"PASS: badge stable across {len(badges)} polls "
                  f"('{distinct[0]}')")
        else:
            print(f"FAIL: badge flapped across {len(badges)} polls: {distinct}",
                  file=sys.stderr)
            failed = True
        if any(not r.get("service_ok") for r in gui):
            n_down = sum(1 for r in gui if not r.get("service_ok"))
            print(f"NOTE: {n_down}/{len(gui)} GUI polls saw service_ok=false")

    # --- assertion: every consumer agrees on position -----------------------
    sources = {
        "gui": last_fix(gui),
        "phase2_server": last_fix(read_phase2(args.phase2)),
        "gsm_monitor(get_once)": last_fix(read_probe(args.probe)),
    }
    fixed = {name: rec for name, rec in sources.items() if rec is not None}

    if len(fixed) < 2:
        missing = [name for name, rec in sources.items() if rec is None]
        print(f"FAIL: fewer than 2 consumers reported a fix (no fix from: "
              f"{missing}) -- cannot assert position consistency", file=sys.stderr)
        failed = True
    else:
        names = list(fixed)
        worst_lat = worst_lon = 0.0
        worst_pair = None
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = fixed[names[i]], fixed[names[j]]
                dlat = abs(a["lat"] - b["lat"])
                dlon = abs(a["lon"] - b["lon"])
                if dlat > worst_lat or dlon > worst_lon:
                    worst_lat, worst_lon = max(worst_lat, dlat), max(worst_lon, dlon)
                    worst_pair = (names[i], names[j])
        if worst_lat <= args.tolerance_deg and worst_lon <= args.tolerance_deg:
            detail = ", ".join(f"{n}=({r['lat']:.7f},{r['lon']:.7f})"
                                for n, r in fixed.items())
            print(f"PASS: position consistent within {args.tolerance_deg} deg "
                  f"across {len(fixed)} consumers: {detail}")
        else:
            print(f"FAIL: position disagreement {worst_pair}: "
                  f"dlat={worst_lat:.7f} dlon={worst_lon:.7f} "
                  f"exceeds tolerance {args.tolerance_deg}", file=sys.stderr)
            for n, r in fixed.items():
                print(f"       {n}: lat={r['lat']:.7f} lon={r['lon']:.7f}", file=sys.stderr)
            failed = True
        if len(fixed) < len(sources):
            missing = [name for name, rec in sources.items() if rec is None]
            print(f"NOTE: no fix observed from: {missing} (compared the "
                  f"{len(fixed)} that did report one)")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
