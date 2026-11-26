#!/usr/bin/env python3
"""Go / no-go for the flight recorder before a flight. Run ON the Pi:

    .venv-pi/bin/python scripts/field/recorder_check.py            # read the live status
    .venv-pi/bin/python scripts/field/recorder_check.py --camera   # also record 3 s of test video

or from the laptop:  ssh admin@192.168.0.61 'cd interceptor-sim && .venv-pi/bin/python scripts/field/recorder_check.py'

It reads the status the flight-recorder service rewrites every second
(/dev/shm/flight_recorder_status.json) and prints one line per check. Exit 0 = GO,
1 = NO-GO. A missing or stale status file is a NO-GO: no answer is not a pass.

The limits: 30 KB/s is three quarters of the 40 KB/s the log runs at over USB
(measured 2026-11-13 to 2026-11-22, disarmed); 0.47 GB per minute of video was
measured outdoors on 2026-11-15; 10 GB free is one long flight plus the 5 GB below
which the recorder refuses to start a video.
"""
import argparse
import json
import os
import subprocess
import sys
import time

STATUS_FILE = "/dev/shm/flight_recorder_status.json"
STALE_S = 5.0
MIN_KB_PER_S = 30.0
QUIET_AFTER_GAP_S = 30.0
MIN_FREE_GB = 10.0
VIDEO_GB_PER_MIN = 0.47
VIDEO_FLOOR_GB = 5.0


def judge(st: dict | None, now: float) -> list[tuple[bool, str]]:
    """(ok, plain line) per check, from one status reading."""
    if st is None:
        return [(False, "no status file: the flight-recorder service is not running "
                        "(sudo systemctl start flight-recorder)")]
    age = now - float(st.get("updated_unix", 0))
    if age > STALE_S:
        return [(False, f"status is {age:.0f} s old: the recorder has stopped or hung "
                        "(journalctl -u flight-recorder -n 30)")]
    out = []
    state = st.get("state")
    if state != "recording":
        out.append((False, f"not recording: {state}" + (f" ({st['detail']})" if st.get("detail") else "")))
    else:
        out.append((True, f"recording the Pixhawk's log to {st.get('file')}"))
        usb = st.get("link") == "usb"
        out.append((usb, "log cable: USB" if usb else
                    f"log cable: {st.get('port')}, the SERIAL cable, which loses about a third "
                    "of the log. Plug the Pixhawk's USB into the Pi"))
        rate = float(st.get("recent_kb_per_s") or 0.0)
        out.append((rate >= MIN_KB_PER_S, f"log rate {rate:.0f} KB/s (needs {MIN_KB_PER_S:.0f})"))
        gap = st.get("last_gap_s_ago")
        quiet = gap is None or gap > QUIET_AFTER_GAP_S
        out.append((quiet, "no packets lost in the last 30 s" if quiet else
                    f"packets were lost {gap:.0f} s ago (QGroundControl still loading? wait, then check again)"))
    video = st.get("video")
    out.append((video in ("idle", "recording"),
                {"idle": "video: ready, starts at arming", "recording": "video: recording now",
                 "failed": "video: the camera recording FAILED this flight (journalctl -u flight-recorder)",
                 "off": "video: no on-arm command set, nothing will be recorded"}.get(video, f"video: {video}")))
    free = float(st.get("free_gb") or 0.0)
    minutes = max(0.0, (free - VIDEO_FLOOR_GB) / VIDEO_GB_PER_MIN)
    out.append((free >= MIN_FREE_GB, f"card: {free:.0f} GB free, about {minutes:.0f} min of video"))
    return out


def camera_test(repo: str) -> tuple[bool, str]:
    """Record 3 s with the same program the service runs at arming."""
    cmd = [sys.executable, "scripts/field/record_video.py", "--seconds", "3", "--flip",
           "--note", "recorder_check camera test", "--out-dir", "/tmp/recorder_check"]
    try:
        r = subprocess.run(cmd, cwd=repo, capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        return False, "camera test: no result in 60 s"
    if r.returncode == 0:
        return True, "camera test: 3 s recorded with no dropped frames"
    tail = (r.stderr or r.stdout).strip().splitlines()[-1:] or ["no output"]
    return False, f"camera test FAILED (exit {r.returncode}): {tail[0][:160]}"


# flight/deploy/board_overrides.py saves the Pixhawk's own settings here while a
# program-flown flight has its low-battery action and GPS quality checks
# switched off. The file is removed when they are restored. If it is still
# there, the last run did not put them back.
BOARD_STATE_FILE = os.path.expanduser("~/.interceptor/board_overrides.json")


def board_settings_line(path: str) -> tuple[bool, str]:
    if not os.path.exists(path):
        return True, "board settings: the Pixhawk holds its own values"
    return False, ("board settings: the last program flight did NOT restore the "
                   "Pixhawk's low-battery action and GPS checks. Run: "
                   ".venv-pi/bin/python -m flight.deploy.board_overrides --restore")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--status-file", default=STATUS_FILE)
    ap.add_argument("--board-state-file", default=BOARD_STATE_FILE)
    ap.add_argument("--camera", action="store_true",
                    help="also record 3 s of test video (only while disarmed and not recording)")
    a = ap.parse_args()
    try:
        with open(a.status_file) as f:
            st = json.load(f)
    except (OSError, ValueError):
        st = None
    lines = judge(st, time.time())
    lines.append(board_settings_line(a.board_state_file))
    if a.camera:
        if st is not None and (st.get("armed") or st.get("video") == "recording"):
            lines.append((False, "camera test skipped: armed, or the video is already recording"))
        else:
            lines.append(camera_test(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
    for ok, line in lines:
        print(("  ok    " if ok else "  NO-GO ") + line)
    go = all(ok for ok, _ in lines)
    print("RECORDER: GO" if go else "RECORDER: NO-GO")
    return 0 if go else 1


if __name__ == "__main__":
    sys.exit(main())
