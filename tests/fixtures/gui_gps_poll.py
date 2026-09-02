#!/usr/bin/env python3
"""Poll the SIGINT GUI's real GPS badge path for tests/regression_all_consumers.sh.

Calls scanner.tasks.get_gps_status() -- the exact function views.py's
/api/gps-status/ endpoint calls (see SIGINT_GUI/scanner/views.py) -- and
scanner.tasks.get_gps_position() for the full snapshot, repeatedly, while
the other three consumers are also hammering wbr-gpsd.

tasks.py always asks wbr_gps_client.get_once(timeout=0.5), which resolves
against wbr_gps_client.DEFAULT_SOCKET ("/run/wbr-gps/gpsd.sock") with no
override, and creating /run/wbr-gps/ needs root. When the caller is not
talking to that production socket (the regression's SELF-HOSTED mode), this
substitutes the *function object* tasks.wbr_gps_client.get_once with
functools.partial(get_once, sock_path=SOCK), which is the same real
function, actually called every poll -- only its target socket changes.
Pass sock_path "PROD" to skip the patch entirely and run tasks.py exactly
as it ships.

Usage: gui_gps_poll.py SIGINT_GUI_DIR SOCK_PATH_OR_PROD RUN_SECONDS INTERVAL_SECONDS

Prints one compact JSON object per line to stdout, flushed immediately:
  {"t": <monotonic seconds since start>, "badge": <str>, ...snapshot fields}
"""
import functools
import json
import os
import sys
import time


def main(argv):
    if len(argv) != 5:
        print("usage: gui_gps_poll.py SIGINT_GUI_DIR SOCK_PATH_OR_PROD "
              "RUN_SECONDS INTERVAL_SECONDS", file=sys.stderr)
        return 2

    gui_dir, sock_arg, run_seconds, interval = (
        argv[1], argv[2], float(argv[3]), float(argv[4]))

    sys.path.insert(0, gui_dir)
    os.environ.setdefault("DJANGO_SETTINGS_MODULE", "auto_scan_SIGINT.settings")
    import django
    django.setup()

    from scanner import tasks

    if tasks.wbr_gps_client is None:
        print("scanner.tasks.wbr_gps_client is None -- WBR-GPS not checked "
              "out next to SIGINT_GUI?", file=sys.stderr)
        return 1

    if sock_arg != "PROD":
        real_get_once = tasks.wbr_gps_client.get_once
        tasks.wbr_gps_client.get_once = functools.partial(
            real_get_once, sock_path=sock_arg)

    n_polls = max(1, int(run_seconds / interval))
    start = time.monotonic()
    for _ in range(n_polls):
        status = tasks.get_gps_status()          # {"gps_status": ...}
        position = tasks.get_gps_position()       # full snapshot dict
        record = {"t": round(time.monotonic() - start, 3),
                   "badge": status["gps_status"]}
        record.update(position)
        print(json.dumps(record), flush=True)
        time.sleep(interval)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
