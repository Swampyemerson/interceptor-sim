#!/usr/bin/env python3
"""Is the FLYING seeker frame-rate capped? Measure it on the real class.

WHY THIS EXISTS (2026-07-26)
----------------------------
The skr-07 soak found the Pi 5 decoding AprilTags at ~117 fps while the camera
delivered exactly 30.0 — the seeker was CAMERA-CAPPED by an inherited picamera2
frame-duration default, not compute-bound. Lifting the cap gave 111.8 fps.

That matters far beyond the bench. The handoff streak burn goes as 1/fps:

    R_streak_burn = (E[T] / stream_fps) x V_closing        (ADR-0079)

so a 3.7x frame-rate difference is a ~3.7x difference in the range consumed
forming the 5-in-a-row handoff — the exact quantity that decides whether the
camera earns its place in the terminal phase.

`flight/deploy/seeker_loop.py:PicameraSource` sets exposure and gain but NO
`FrameDurationLimits`, exactly like the bench did before the fix. So the flying
seeker is probably capped too. "Probably" is not good enough for a number the
gate turns on, hence this probe.

IT TESTS THE REAL CLASS, NOT A COPY
-----------------------------------
It imports and instantiates `PicameraSource` itself. A reimplementation of the
camera setup here would be a hand-typed fixture of the thing under test — the
precise pattern that hid this project's schema breaks (both sides passed their
own self-tests; nothing tested the pair). If the deployed source is capped, this
sees it; if someone later fixes the deployed source, this stops reporting a cap
without anyone editing this file.

The comparison arm re-opens the SAME class with a FrameDurationLimits control
patched into its control dict, so the only difference between arms is the cap.

USAGE (on the Pi, inside .venv-pi)
    scripts/seeker/probe_flight_frame_rate.py --frames 300
    scripts/seeker/probe_flight_frame_rate.py --frames 300 --json out.json
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time

# This file is scripts/seeker/<name>.py, so the repo root is THREE levels up.
# Two levels lands on scripts/ and `import flight` then fails on the Pi.
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if REPO not in sys.path:
    sys.path.insert(0, REPO)


def _time_source(src, n_frames, warmup):
    """Pull n_frames off a frame source, return the delivered-cadence stats."""
    dts, prev, seen = [], None, 0
    for _ in src.frames():
        now = time.monotonic()
        if prev is not None:
            dts.append(now - prev)
        prev = now
        seen += 1
        if seen >= n_frames:
            break
    dts = dts[warmup:]                      # drop the pipeline's spin-up
    if not dts:
        return None
    med = statistics.median(dts)
    slow = sorted(dts)[int(0.9 * (len(dts) - 1))]
    return {
        "n_timed": len(dts),
        "fps_median": round(1.0 / med, 3),
        "fps_p10_slow_tail": round(1.0 / slow, 3),
        "frame_dt_ms_median": round(med * 1000.0, 3),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--warmup", type=int, default=30)
    ap.add_argument("--target-fps", type=float, default=143.0,
                    help="the uncapped comparison arm")
    ap.add_argument("--exposure-us", type=int, default=1000)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    from flight.deploy.seeker_loop import PicameraSource

    # ---- arm A: the deployed source, exactly as it flies -------------------
    src = PicameraSource(exposure_us=args.exposure_us)
    as_flown = _time_source(src, args.frames, args.warmup)
    capped_controls = dict(src.controls)
    applied_exp = src.applied_exposure_us
    try:
        src.cam.stop(); src.cam.close()
    except Exception:
        pass
    time.sleep(2.0)

    # ---- arm B: same class, cap lifted -------------------------------------
    # Patch FrameDurationLimits into the control dict the class builds, then let
    # the class configure itself with it. Only the cap differs between arms.
    dur = int(round(1e6 / args.target_fps))
    src2 = PicameraSource.__new__(PicameraSource)
    from picamera2 import Picamera2
    src2.cam = Picamera2()
    src2.exposure_us = args.exposure_us
    src2.applied_exposure_us = None
    src2.controls = dict(capped_controls)
    src2.controls["FrameDurationLimits"] = (dur, dur)
    cfg = src2.cam.create_video_configuration(
        main={"size": (1280, 800), "format": "RGB888"}, controls=src2.controls)
    src2.cam.configure(cfg)
    src2.cam.start()
    src2.cam.set_controls(src2.controls)
    uncapped = _time_source(src2, args.frames, args.warmup)
    try:
        src2.cam.stop(); src2.cam.close()
    except Exception:
        pass

    ratio = (uncapped["fps_median"] / as_flown["fps_median"]
             if as_flown and uncapped and as_flown["fps_median"] else None)
    rec = {
        "tool": "probe_flight_frame_rate.py",
        "subject": "flight/deploy/seeker_loop.py:PicameraSource (the DEPLOYED class)",
        "exposure_us_applied": applied_exp,
        "format": "RGB888 (what the flight source requests, on a MONO sensor)",
        "as_flown": as_flown,
        "uncapped": uncapped,
        "target_fps_requested": args.target_fps,
        "speedup_x": round(ratio, 2) if ratio else None,
        "capped": bool(ratio and ratio > 1.3),
        "so_what": (
            "R_streak_burn goes as 1/fps (ADR-0079), so this ratio is the factor "
            "by which the deployed seeker over-spends range forming the 5-frame "
            "handoff streak. It is NOT a bench artifact: this is the class that "
            "flies." if ratio and ratio > 1.3 else
            "The deployed source is not frame-duration capped."),
    }
    print(json.dumps(rec, indent=2))
    if args.json:
        with open(args.json, "w") as fh:
            json.dump(rec, fh, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
