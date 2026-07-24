#!/usr/bin/env python3
"""camera_session_summary.py -- grade one pi_capture.py capture session.

Used by scripts/field/01_camera_live_check.sh. It does NOT talk to a camera:
scripts/seeker/pi_capture.py is the ONE code path that touches hardware (its
picamera2 / v4l2 / dir-replay backends), and this tool reads the session
directory that script writes (frames/ + index.csv + meta.json [+ tags.csv];
layout documented in pi_capture.py's header) and turns it into a PASS/FAIL.

WHAT IT GRADES, AND WHERE EACH NUMBER COMES FROM
------------------------------------------------
  frames arrived   meta.json n_frames >= --min-frames.
  resolution       meta.json resolution vs --expect-res. The BOM camera is the
                   innomaker OV9281, 1280x800 mono global shutter
                   (docs/camera_paper_check.md; pi_capture.py DEF_WIDTH/HEIGHT).
  exposure         meta.json applied_exposure_us vs the <=1 ms SPEC
                   (pi_capture.py EXPOSURE_SPEC_US = 1000; the spec exists
                   because real motion blur at the >=9 m/s engagement regime is
                   what the field day measures -- docs/tripod_test_protocol.md
                   3.3/4.4). Reported as UNKNOWN, not FAIL, when the backend
                   cannot report it (dir replay has no real exposure).
  frame rate       median dt between index.csv t_mono_s values -> fps. ADVISORY
                   only: it is a capture-loop rate, not a certified sensor rate,
                   and the dir-replay backend synthesises its timestamps.

Exit: 0 PASS, 1 FAIL, 2 usage/parse error.

Usage:
  camera_session_summary.py SESSION_DIR [--expect-res 1280x800|any]
      [--min-frames 1] [--sample-out PATH] [--json-out PATH]
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import sys

# The exposure spec target, kept in sync with scripts/seeker/pi_capture.py
# (EXPOSURE_SPEC_US). Duplicated as a constant rather than imported so this
# tool stays runnable from anywhere; the value traces to the same doc.
EXPOSURE_SPEC_US = 1000


def _median(xs):
    s = sorted(xs)
    if not s:
        return None
    m = len(s) // 2
    return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="pi_capture.py session dir (frames/ + meta.json)")
    ap.add_argument("--expect-res", default="1280x800",
                    help="WxH the sensor must report, or 'any' (default 1280x800 = OV9281)")
    ap.add_argument("--min-frames", type=int, default=1)
    ap.add_argument("--exposure-spec-us", type=int, default=EXPOSURE_SPEC_US)
    ap.add_argument("--sample-out", default=None,
                    help="copy one mid-session frame here (a picture you can eyeball)")
    ap.add_argument("--json-out", default=None, help="write the machine-readable verdict here")
    args = ap.parse_args()

    sess = args.session
    meta_path = os.path.join(sess, "meta.json")
    if not os.path.isfile(meta_path):
        print(f"[camera-summary] FAIL: no meta.json in {sess}", file=sys.stderr)
        print("[camera-summary]   -> that directory is not a pi_capture.py session")
        return 2
    with open(meta_path) as fh:
        meta = json.load(fh)

    fails, warns = [], []

    # --- frames ------------------------------------------------------------
    n = int(meta.get("n_frames") or 0)
    print(f"[camera-summary] session   : {sess}")
    print(f"[camera-summary] backend   : {meta.get('source')} {meta.get('backend_detail')}")
    print(f"[camera-summary] frames    : {n}")
    if n < args.min_frames:
        fails.append(f"only {n} frame(s), expected >= {args.min_frames}")

    # --- resolution --------------------------------------------------------
    res = meta.get("resolution") or {}
    w, h = int(res.get("width") or 0), int(res.get("height") or 0)
    print(f"[camera-summary] resolution: {w}x{h}")
    if args.expect_res.lower() != "any":
        try:
            ew, eh = (int(v) for v in args.expect_res.lower().split("x"))
        except ValueError:
            print(f"[camera-summary] FAIL: bad --expect-res {args.expect_res!r}", file=sys.stderr)
            return 2
        if (w, h) != (ew, eh):
            fails.append(f"resolution {w}x{h} != expected {ew}x{eh}")
        else:
            print(f"[camera-summary]   OK matches the expected {ew}x{eh}")

    # --- exposure vs the <=1 ms spec ---------------------------------------
    applied = meta.get("applied_exposure_us")
    req = meta.get("requested_exposure_us")
    if applied is not None and meta.get("source") == "v4l2":
        # pi_capture.py's v4l2 backend reads CAP_PROP_EXPOSURE back, whose units
        # are DEVICE-SPECIFIC (often log2 steps, not microseconds) -- its own
        # header and backend_detail say so. Comparing that number to a
        # microsecond spec would be a fake verdict, so we refuse to grade it.
        print(f"[camera-summary] exposure  : read-back {applied} in DEVICE-SPECIFIC "
              f"V4L2 units (requested {req} us) -- NOT graded")
        warns.append("v4l2 exposure units are device-specific: the <=1 ms spec is "
                     "UNVERIFIED here. The spec is verified on the Pi's picamera2 "
                     "backend (scripts/pi_setup/README.md step 3).")
        applied = None
    elif applied is None:
        print(f"[camera-summary] exposure  : UNKNOWN "
              f"(source={meta.get('exposure_source')}, requested {req} us)")
        warns.append("this backend cannot report the applied exposure -- "
                     "the <=1 ms spec is UNVERIFIED for this session")
    else:
        verdict = "OK" if applied <= args.exposure_spec_us else "OVER SPEC"
        print(f"[camera-summary] exposure  : applied ~{applied:.0f} us "
              f"(requested {req}, spec <= {args.exposure_spec_us}) -> {verdict}")
        print(f"[camera-summary] gain      : ~{meta.get('applied_gain')}")
        if applied > args.exposure_spec_us:
            fails.append(f"applied exposure {applied:.0f} us exceeds the "
                         f"{args.exposure_spec_us} us spec")

    # --- frame rate (advisory) --------------------------------------------
    idx = os.path.join(sess, "index.csv")
    fps = None
    if os.path.isfile(idx):
        ts = []
        with open(idx) as fh:
            for row in csv.DictReader(fh):
                try:
                    ts.append(float(row["t_mono_s"]))
                except (KeyError, TypeError, ValueError):
                    pass
        dts = [b - a for a, b in zip(ts, ts[1:]) if b > a]
        med = _median(dts)
        if med:
            fps = 1.0 / med
            print(f"[camera-summary] rate      : ~{fps:.1f} fps (median dt {med * 1e3:.1f} ms) "
                  f"[ADVISORY -- capture-loop rate]")
    else:
        warns.append("no index.csv in the session")

    # --- a picture you can actually look at --------------------------------
    sample_src = None
    frames_dir = os.path.join(sess, meta.get("frames_dir") or "frames")
    if os.path.isdir(frames_dir):
        names = sorted(f for f in os.listdir(frames_dir)
                       if f.lower().endswith((".png", ".jpg", ".jpeg")))
        if names:
            sample_src = os.path.join(frames_dir, names[len(names) // 2])
    if args.sample_out and sample_src:
        os.makedirs(os.path.dirname(os.path.abspath(args.sample_out)), exist_ok=True)
        shutil.copyfile(sample_src, args.sample_out)
        print(f"[camera-summary] sample    : {args.sample_out}  <- OPEN THIS and check "
              f"focus/aim/brightness")
    elif args.sample_out:
        warns.append("no frame files found to save a sample from")

    for wmsg in warns:
        print(f"[camera-summary]   WARN {wmsg}")

    verdict = {
        "session": os.path.abspath(sess), "n_frames": n,
        "resolution": [w, h], "expect_res": args.expect_res,
        "applied_exposure_us": applied, "exposure_spec_us": args.exposure_spec_us,
        "fps_estimate": fps, "sample_frame": args.sample_out if sample_src else None,
        "warnings": warns, "failures": fails, "pass": not fails,
    }
    if args.json_out:
        os.makedirs(os.path.dirname(os.path.abspath(args.json_out)), exist_ok=True)
        with open(args.json_out, "w") as fh:
            json.dump(verdict, fh, indent=2)

    if fails:
        for f in fails:
            print(f"[camera-summary]   BAD  {f}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
