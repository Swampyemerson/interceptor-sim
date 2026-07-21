#!/usr/bin/env python3
"""Re-score the EXISTING frame-top sweep + #18j (quad_approach) datasets with
the OLD vs the FIXED box_hits_gt gate, and run a clean-negative inflation
control -- the validation for constraint `box-hits-gt-scoring-debt`.

No sim: reads frames/labels already on disk, runs the v2 detector (arm-a, the
deployable full-frame metric) once per frame, scores each detection two ways:

  OLD : gt_scale=1.0, offaxis_aware=False   (the pre-fix gate, fixed 3x)
  NEW : gt_scale=TARGET_EXTENT_M/capture_extent, offaxis_aware=True
        (extent unified to the measured fpv_quad_enemy silhouette + sec^2 gt)

Prints per-range OLD->NEW recall for each dataset and writes per-frame CSVs +
a summary CSV under logs/box_hits_gt_rescore/. The clean-negative control puts
a synthetic frame-top gt on the quad_approach NEGATIVE frames (target NOT in
view) and reports the false-accept rate OLD vs NEW: the fix must recover
frame-top positives WITHOUT lifting this.

Run under .venv-seeker (onnxruntime + cv2).
Usage: scripts/seeker/rescore_box_hits_gt.py [--conf 0.25] [--limit N]
"""
import argparse
import csv
import glob
import json
import os
import re
import sys
from collections import defaultdict

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from finetuned_seeker import FinetunedNNSeeker  # noqa: E402
from box_scoring import box_hits_gt, gt_scale_for, TARGET_EXTENT_M  # noqa: E402

FX = FY = 539.936
CX, CY = 640.0, 480.0
W_FULL, H_FULL = 1280, 960
V2 = os.path.join(HERE, "weights", "drone_finetuned_quad_v2.onnx")
DATA = os.path.join(HERE, "data")
OUT = os.path.join(HERE, "..", "..", "logs", "box_hits_gt_rescore")
_RANGE_RE = re.compile(r"_r(\d+\.?\d*)")

# dataset -> (relative dir, capture --extent-m it was labelled at)
DATASETS = [
    ("frametop_centered", "frametop_sweep/centered", 0.35),
    ("frametop", "frametop_sweep/frametop", 0.35),
    ("frametop_banked", "frametop_sweep/frametop_banked", 0.35),
    ("quad_approach", "quad_approach", None),   # None -> read manifest (0.9)
]


def gt_range_of(name):
    m = _RANGE_RE.search(name)
    return float(m.group(1)) if m else None


def load_gt_box(lp):
    try:
        parts = open(lp).readline().split()
    except OSError:
        return None
    if len(parts) < 5:
        return None
    _, cx, cy, w, h = (float(v) for v in parts[:5])
    return cx * W_FULL, cy * H_FULL, w * W_FULL, h * H_FULL


def capture_extent(ddir, override):
    if override is not None:
        return override
    mp = os.path.join(ddir, "manifest.json")
    if os.path.exists(mp):
        try:
            return float(json.load(open(mp)).get("extent_m", 0.9))
        except (OSError, ValueError, TypeError):
            pass
    return 0.9


def old_hit(boxes, gcx, gcy, gw, gh):
    return box_hits_gt(boxes, gcx, gcy, gw, gh, gt_scale=1.0, offaxis_aware=False)


def new_hit(boxes, gcx, gcy, gw, gh, gscale):
    return box_hits_gt(boxes, gcx, gcy, gw, gh, gt_scale=gscale, offaxis_aware=True)


def rescore_dataset(v2, name, ddir, cap_ext, limit, wr):
    gscale = gt_scale_for(cap_ext)
    imgs = sorted(glob.glob(os.path.join(ddir, "images", "*.png")))
    lbl_dir = os.path.join(ddir, "labels")
    # per 2 m bin -> [old_hits, new_hits, total]
    bins = defaultdict(lambda: [0, 0, 0])
    flips = 0
    n = 0
    for ip in imgs:
        base = os.path.splitext(os.path.basename(ip))[0]
        gt = load_gt_box(os.path.join(lbl_dir, base + ".txt"))
        if gt is None:
            continue  # negative frame -> handled by the control, not here
        gr = gt_range_of(base)
        if gr is None:
            continue
        frame = cv2.imread(ip)
        if frame is None:
            continue
        n += 1
        gcx, gcy, gw, gh = gt
        boxes = v2._infer_boxes(frame)
        oh = old_hit(boxes, gcx, gcy, gw, gh)
        nh = new_hit(boxes, gcx, gcy, gw, gh, gscale)
        b = int(gr // 2) * 2
        row = bins[b]
        row[0] += int(oh)
        row[1] += int(nh)
        row[2] += 1
        if nh and not oh:
            flips += 1
        wr.writerow([name, base, f"{gr:.1f}", f"{gcx:.1f}", f"{gcy:.1f}",
                     f"{gw:.1f}", f"{gh:.1f}", len(boxes), int(oh), int(nh)])
        if limit and n >= limit:
            break
    return bins, flips, n, gscale


def negative_control(v2, ddir, limit):
    """Clean-negative inflation control: quad_approach frames with NO target in
    view. Drop a synthetic frame-top gt (CX, 58 px, 20 px @0.35 -- the r12
    profile) and count how often OLD vs NEW falsely calls it a hit."""
    imgs = sorted(glob.glob(os.path.join(ddir, "images", "*.png")))
    lbl_dir = os.path.join(ddir, "labels")
    gscale = gt_scale_for(0.35)
    gcx, gcy, gw, gh = CX, 58.0, 20.0, 20.0
    old_fa = new_fa = tot = 0
    for ip in imgs:
        base = os.path.splitext(os.path.basename(ip))[0]
        if load_gt_box(os.path.join(lbl_dir, base + ".txt")) is not None:
            continue  # a positive -> not part of the negative control
        frame = cv2.imread(ip)
        if frame is None:
            continue
        tot += 1
        boxes = v2._infer_boxes(frame)
        old_fa += int(old_hit(boxes, gcx, gcy, gw, gh))
        new_fa += int(new_hit(boxes, gcx, gcy, gw, gh, gscale))
        if limit and tot >= limit:
            break
    return old_fa, new_fa, tot


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--limit", type=int, default=0, help="cap frames/dataset (0=all)")
    args = ap.parse_args()
    if not os.path.exists(V2):
        print(f"detector weights missing at {V2} -- cannot re-score.")
        return 1
    os.makedirs(OUT, exist_ok=True)
    v2 = FinetunedNNSeeker(FX, FY, CX, CY, V2, conf_thres=args.conf)
    print(f"box_hits_gt RE-SCORE  conf={args.conf}  unified extent={TARGET_EXTENT_M} m")
    print(f"OLD = fixed 3x, no off-axis, no extent-unify   NEW = the fix\n")

    per_frame = open(os.path.join(OUT, "per_frame.csv"), "w", newline="")
    wr = csv.writer(per_frame)
    wr.writerow(["dataset", "frame", "gt_range", "gcx", "gcy", "gw", "gh",
                 "n_boxes", "old_hit", "new_hit"])
    summary = []
    for name, rel, ext in DATASETS:
        ddir = os.path.join(DATA, rel)
        if not os.path.isdir(os.path.join(ddir, "images")):
            print(f"[skip] {name}: {ddir} not found")
            continue
        cap_ext = capture_extent(ddir, ext)
        bins, flips, n, gscale = rescore_dataset(v2, name, ddir, cap_ext, args.limit, wr)
        print(f"=== {name}  (n={n}, labelled {cap_ext} m -> gt_scale {gscale:.3f}) ===")
        print(f"  {'range':<9}{'OLD':>12}{'NEW':>12}   flip->hit")
        o_all = nw_all = t_all = 0
        for b in sorted(bins):
            oh, nh, t = bins[b]
            o_all += oh; nw_all += nh; t_all += t
            print(f"  {f'{b}-{b+2} m':<9}{f'{100*oh/t:.0f}% ({oh}/{t})':>12}"
                  f"{f'{100*nh/t:.0f}% ({nh}/{t})':>12}   +{nh-oh}")
        if t_all:
            print(f"  {'ALL':<9}{f'{100*o_all/t_all:.0f}% ({o_all}/{t_all})':>12}"
                  f"{f'{100*nw_all/t_all:.0f}% ({nw_all}/{t_all})':>12}   "
                  f"flips={flips}")
        summary.append((name, cap_ext, n, o_all, nw_all, t_all, flips))
        print()
    per_frame.close()

    # clean-negative inflation control
    qa = os.path.join(DATA, "quad_approach")
    if os.path.isdir(os.path.join(qa, "images")):
        ofa, nfa, tot = negative_control(v2, qa, args.limit)
        print("=== clean-negative inflation control (quad_approach negatives) ===")
        print(f"  synthetic frame-top gt on {tot} target-absent frames:")
        print(f"  OLD false-accept {ofa}/{tot} ({100*ofa/max(1,tot):.1f}%)   "
              f"NEW false-accept {nfa}/{tot} ({100*nfa/max(1,tot):.1f}%)")
        print("  (NEW must NOT be materially higher than OLD -> fix recovers "
              "positives without inflating misses)")
    else:
        ofa = nfa = tot = 0

    with open(os.path.join(OUT, "summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["dataset", "capture_extent_m", "unified_extent_m", "n",
                    "old_hits", "new_hits", "total", "flips_to_hit"])
        for name, ce, n, o, nw, t, fl in summary:
            w.writerow([name, ce, TARGET_EXTENT_M, n, o, nw, t, fl])
        w.writerow(["neg_control", 0.35, TARGET_EXTENT_M, tot, ofa, nfa, tot, ""])
    print(f"\nwrote {OUT}/per_frame.csv + summary.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
