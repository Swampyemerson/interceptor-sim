#!/usr/bin/env python3
"""What does a tag IN FRAME cost the detector? (the soak's missing quantity)

WHY (2026-07-26)
----------------
`pi_fps_soak.py` refuses to call a tagless run gate-eligible, because a scene
with no tag gives the detector no candidate quads to refine: decode cost is
understated, so fps is OVERSTATED — the flattering direction for a ~$740 gate.
That refusal is right, but it leaves a hole: with no placard printed and nobody
at the bench, how wrong is the tagless number?

At the capped 30 fps this did not matter — decode throughput (~117 fps) was so
far above delivered cadence that min() was set by the camera, and a slower decode
could not move the published number. Uncapped, that headroom fell to ~1.21x, so
the tag's marginal cost now lands directly on the published fps. This measures it.

METHOD — MATCHED PAIRS, ONE VARIABLE
------------------------------------
Comparing tag frames against unrelated blank frames would confound the tag with
the whole background. Instead each frame is used TWICE:

  A. as-is (tag present)
  B. the same frame with the detected tag's quad filled in with the local median
     (tag absent, background otherwise identical)

so the only thing that differs is the presence of the fiducial. Frames where no
tag is found are skipped and COUNTED (a pair that cannot be formed is not
silently dropped).

WHAT THIS IS NOT
----------------
NOT a frame rate, and NOT gate-eligible. Frames come off disk, so there is no
delivered cadence to bound a throughput against — the same reason `pi_capture.py`
publishes no `stream_fps` for a replay source. The output is a decode COST and a
RATIO. Feed the ratio back into the soak's published fps to get a corrected
estimate; do not quote it as a rate.

Also: the only tag imagery this repo has is SIM-RENDERED (the 2026-07-25 field
"apriltag" logs are dry runs that fell back to those same renders). Sim
backgrounds are cleaner than an outdoor scene, and real clutter raises the
candidate-quad count for BOTH arms. So the absolute milliseconds are indicative;
the ratio is the useful product, and it is still measured on the real Pi 5 CPU.

USAGE (on the Pi, inside .venv-pi)
    scripts/seeker/tag_decode_cost.py --dir scripts/seeker/data/<set>/images \
        --quad-decimate 2.0 --size 1280x800 --json cost_qd2.json
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import statistics
import sys
import time


def _stats(xs):
    if not xs:
        return None
    s = sorted(xs)
    med = statistics.median(s)
    p90 = s[int(0.9 * (len(s) - 1))]
    return {"n": len(s), "ms_median": round(med * 1e3, 3),
            "ms_p90": round(p90 * 1e3, 3),
            "implied_throughput_fps": round(1.0 / med, 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True, help="directory of tag images")
    ap.add_argument("--quad-decimate", type=float, required=True)
    ap.add_argument("--size", default="1280x800",
                    help="resize to the FLIGHT grid; decode cost scales with it")
    ap.add_argument("--limit", type=int, default=120)
    ap.add_argument("--repeats", type=int, default=3,
                    help="passes per image; the median across all is reported")
    ap.add_argument("--nthreads", type=int, default=4)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    import cv2
    import numpy as np
    from pyapriltags import Detector

    w, h = (int(v) for v in args.size.lower().split("x"))
    det = Detector(families="tag36h11", nthreads=args.nthreads,
                   quad_decimate=args.quad_decimate)

    files = sorted(glob.glob(os.path.join(args.dir, "*.png")) +
                   glob.glob(os.path.join(args.dir, "*.jpg")))[:args.limit]
    if not files:
        raise SystemExit(f"[tag_decode_cost] no images in {args.dir}")

    with_ms, without_ms = [], []
    n_pairs, n_no_tag = 0, 0

    for f in files:
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        img = cv2.resize(img, (w, h))

        dets = det.detect(img, estimate_tag_pose=False)
        if not dets:
            n_no_tag += 1          # counted, never silently dropped
            continue

        # Build the matched tag-free twin: fill each detected quad with the
        # median of the surrounding image so the fill is not itself an edge.
        blank = img.copy()
        for d in dets:
            poly = np.array(d.corners, dtype=np.int32).reshape(-1, 1, 2)
            mask = np.zeros(img.shape, dtype=np.uint8)
            cv2.fillPoly(mask, [poly], 255)
            cv2.dilate(mask, np.ones((9, 9), np.uint8), dst=mask)
            fill = int(np.median(img[mask == 0])) if (mask == 0).any() else 128
            blank[mask > 0] = fill

        # Confirm the twin really is tag-free; if the fill failed, the pair is
        # invalid and must not be counted as a "no tag" sample.
        if det.detect(blank, estimate_tag_pose=False):
            n_no_tag += 1
            continue

        for _ in range(args.repeats):
            t0 = time.monotonic(); det.detect(img, estimate_tag_pose=False)
            with_ms.append(time.monotonic() - t0)
            t0 = time.monotonic(); det.detect(blank, estimate_tag_pose=False)
            without_ms.append(time.monotonic() - t0)
        n_pairs += 1

    if not n_pairs:
        print(json.dumps({"error": "no matched pairs could be formed",
                          "n_images": len(files), "n_no_tag": n_no_tag},
                         indent=2))
        return 1

    a, b = _stats(with_ms), _stats(without_ms)
    ratio = a["ms_median"] / b["ms_median"] if b["ms_median"] else None
    rec = {
        "tool": "tag_decode_cost.py",
        "gate_eligible": False,
        "not_a_rate": ("frames come off disk: there is no delivered cadence to "
                       "bound a throughput against, so these are decode COSTS"),
        "quad_decimate": args.quad_decimate,
        "size": f"{w}x{h}",
        "source_dir": args.dir,
        "imagery": "SIM-RENDERED tags (no real tag capture exists in the repo)",
        "n_pairs": n_pairs, "n_images": len(files),
        "n_unpairable_counted": n_no_tag,
        "with_tag": a, "without_tag": b,
        "tag_cost_ratio": round(ratio, 3) if ratio else None,
        "tag_cost_ms": round(a["ms_median"] - b["ms_median"], 3),
        "how_to_use": ("multiply the soak's TAGLESS published fps by "
                       "1/tag_cost_ratio to estimate the with-tag rate, then "
                       "re-clamp by the delivered cadence (fps cannot exceed "
                       "frame arrival)"),
    }
    print(json.dumps(rec, indent=2))
    if args.json:
        with open(args.json, "w") as fh:
            json.dump(rec, fh, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
