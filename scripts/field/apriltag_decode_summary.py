#!/usr/bin/env python3
"""apriltag_decode_summary.py -- grade the AprilTag decode in one capture session.

Used by scripts/field/02_apriltag_desk_check.sh. The decoding itself is done by
scripts/seeker/pi_capture.py (--decode-tags), which writes tags.csv; this tool
only reads that CSV and turns it into a PASS/FAIL plus the numbers you want on
a desk check: how often the tag decoded, how strong the decode was, how big the
tag was in pixels, where it sat in the frame, and what range the tag pose
implies.

WHY THE TAG MATTERS (docs/hardware_order_list.md 0c; project_state constraint
`pi5-compute`): the AprilTag is not just a sim stand-in -- the FIRST REAL KILLS
FLY THE TAG, because the tag decoder runs real-time on the Pi 5 CPU while the
markerless NN needs the deferred Hailo HAT. So "does the tag decode" is the
baseline seeker's sanity check, and the tripod day's decode-vs-range curve is
the money gate on the Tier-2 interceptor order.

RANGE HONESTY: range comes from the tag's pose, which is only as good as the
camera intrinsics. Intrinsics from the SIM camera applied to real pixels give a
silently wrong range (scripts/calibrate_camera.py header spells this out), so
this tool refuses to GRADE range unless it is told the calibration is trusted
(--calib-trusted); otherwise range is printed as INDICATIVE only.

This is a sanity check, NOT the money-gate scorer. The decode-envelope curve
that gates the interceptor order is scripts/seeker/tripod_score.py.

Exit: 0 PASS, 1 FAIL, 2 usage/parse error.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys


def _median(xs):
    s = sorted(xs)
    if not s:
        return None
    m = len(s) // 2
    return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])


def _corner_edge_px(corners_px: str):
    """Mean edge length in pixels of the decoded tag quad.

    corners_px format is pi_capture.py's: 'x0:y0;x1:y1;x2:y2;x3:y3'.
    The apparent edge is the single most useful desk number: it is what decode
    range is really made of (pixels on the tag), independent of any calibration.
    """
    try:
        pts = [tuple(float(v) for v in p.split(":")) for p in corners_px.split(";")]
    except (ValueError, AttributeError):
        return None
    if len(pts) != 4:
        return None
    edges = [math.dist(pts[i], pts[(i + 1) % 4]) for i in range(4)]
    return sum(edges) / 4.0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="pi_capture.py session dir with tags.csv")
    ap.add_argument("--min-decoded", type=int, default=1,
                    help="PASS needs at least this many frames with a decode (default 1)")
    ap.add_argument("--expect-range", type=float, default=None,
                    help="tape-measured camera-to-tag distance in metres (desk check)")
    ap.add_argument("--range-tol", type=float, default=0.15,
                    help="allowed fractional error vs --expect-range (default 0.15 = 15%%)")
    ap.add_argument("--calib-trusted", action="store_true",
                    help="the intrinsics came from THIS camera (calibrate_camera.py); "
                         "only then is range graded")
    ap.add_argument("--json-out", default=None)
    args = ap.parse_args()

    tags_csv = os.path.join(args.session, "tags.csv")
    meta_path = os.path.join(args.session, "meta.json")
    if not os.path.isfile(tags_csv):
        print(f"[tag-summary] FAIL: no tags.csv in {args.session}", file=sys.stderr)
        print("[tag-summary]   -> re-run the capture with --decode-tags")
        return 2
    meta = {}
    if os.path.isfile(meta_path):
        with open(meta_path) as fh:
            meta = json.load(fh)

    frames_seen, decoded_frames = set(), set()
    margins, ranges, edges, ids = [], [], [], {}
    centers = []
    with open(tags_csv) as fh:
        for row in csv.DictReader(fh):
            idx = row.get("frame_idx")
            frames_seen.add(idx)
            if not row.get("tag_id"):
                continue
            decoded_frames.add(idx)
            ids[row["tag_id"]] = ids.get(row["tag_id"], 0) + 1
            for key, sink in (("decision_margin", margins), ("range_m", ranges)):
                try:
                    sink.append(float(row[key]))
                except (KeyError, TypeError, ValueError):
                    pass
            e = _corner_edge_px(row.get("corners_px", ""))
            if e:
                edges.append(e)
            try:
                centers.append((float(row["center_u"]), float(row["center_v"])))
            except (KeyError, TypeError, ValueError):
                pass

    n_frames = int(meta.get("n_frames") or len(frames_seen))
    n_dec = len(decoded_frames)
    rate = (n_dec / n_frames) if n_frames else 0.0

    res = meta.get("resolution") or {}
    print(f"[tag-summary] session      : {args.session}")
    print(f"[tag-summary] family       : {meta.get('family', 'tag36h11')}   "
          f"tag edge assumed: {meta.get('tag_size_m')} m")
    print(f"[tag-summary] frames       : {n_frames}")
    print(f"[tag-summary] decoded      : {n_dec}  ({rate * 100:.1f}% of frames)")
    print(f"[tag-summary] tag ids seen : {', '.join(f'{k} x{v}' for k, v in sorted(ids.items())) or '(none)'}")

    med_margin = _median(margins)
    if med_margin is not None:
        # decision_margin is the detector's confidence in the decoded bits;
        # bigger = further from a misread. Reported, not thresholded (no
        # project-measured threshold exists for the real camera yet).
        print(f"[tag-summary] margin (med) : {med_margin:.1f}  [higher = stronger decode]")
    if edges:
        print(f"[tag-summary] tag in frame : ~{_median(edges):.0f} px per edge "
              f"(min {min(edges):.0f}, max {max(edges):.0f})  [calibration-free]")
    if centers and res.get("width"):
        cu = _median([c[0] for c in centers]); cv = _median([c[1] for c in centers])
        du = cu - res["width"] / 2.0
        dv = cv - (res.get("height") or 0) / 2.0
        print(f"[tag-summary] aim offset   : {du:+.0f} px right, {dv:+.0f} px down "
              f"from frame centre  [aim the camera with this]")

    fails = []
    med_range = _median(ranges)
    if med_range is not None:
        trust = "CALIBRATED" if args.calib_trusted else "INDICATIVE ONLY (intrinsics not from this camera)"
        print(f"[tag-summary] range (med)  : {med_range:.2f} m  "
              f"(min {min(ranges):.2f}, max {max(ranges):.2f})  [{trust}]")
        if args.expect_range is not None:
            err = abs(med_range - args.expect_range)
            frac = err / args.expect_range if args.expect_range else float("inf")
            print(f"[tag-summary] vs tape      : expected {args.expect_range:.2f} m -> "
                  f"error {err:+.2f} m ({frac * 100:.1f}%)")
            if args.calib_trusted:
                if frac > args.range_tol:
                    fails.append(f"range error {frac * 100:.1f}% exceeds the "
                                 f"{args.range_tol * 100:.0f}% tolerance")
            else:
                print("[tag-summary]   NOTE range not graded: pass --calib-trusted only "
                      "with intrinsics from calibrate_camera.py on THIS camera")
    elif args.expect_range is not None:
        print("[tag-summary] range        : not computed (needs --calib AND --tag-size at capture)")

    if n_dec < args.min_decoded:
        fails.append(f"only {n_dec} frame(s) decoded, need >= {args.min_decoded}")

    out = {
        "session": os.path.abspath(args.session), "n_frames": n_frames,
        "n_decoded": n_dec, "decode_rate": rate, "tag_ids": ids,
        "median_decision_margin": med_margin,
        "median_tag_edge_px": _median(edges), "median_range_m": med_range,
        "expect_range_m": args.expect_range, "calib_trusted": bool(args.calib_trusted),
        "failures": fails, "pass": not fails,
    }
    if args.json_out:
        os.makedirs(os.path.dirname(os.path.abspath(args.json_out)), exist_ok=True)
        with open(args.json_out, "w") as fh:
            json.dump(out, fh, indent=2)

    if fails:
        for f in fails:
            print(f"[tag-summary]   BAD  {f}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
