#!/usr/bin/env python3
"""Offline RESOLUTION-vs-ASPECT probe (ADR-0076 add #18i / Fable round-2): does
the markerless detector fire on the APPROACHING quad at 8-25 m when given FULL-RES
pixels, or not at any resolution? Forks the roadmap:
  YES on native crop  -> RESOLUTION wall  -> foveated crop / higher-res inference (sim-provable)
  NO even on crop      -> ASPECT/appearance wall -> only REAL DATA fixes it (shapes R5/R6 capture)

Scores 4 arms on the SAME captured approach frames (scripts/seeker/capture_quad_approach.sh),
using ALL boxes >= conf (not the seeker's top-1 -> closes the conf-NULL's top-1-masking hole):
  (a) v2 @ 640 downscale (full 1280 frame letterboxed to 640)  -- must reproduce the wall
  (b) v2 @ gt-centred 640 NATIVE crop                          -- resolution alone
  (c) quad_crop model @ gt-centred native crop                 -- in-domain crop model (b confounds
                                                                  resolution with scale-domain)
  (d) v2 @ AIM-centred (image-centre) native crop              -- the DEPLOYABLE variant (no gt track
                                                                  at acquisition; the dash aim ~= boresight)
A "hit" = a box whose centre lands within the gt box (arm a, full frame) or within the gt's
mapped position in the crop (b/c/d). Reports hit-rate vs gt-range. gt boxes + gt_range come from
capture_flight_frames.py's saved labels + filenames. Uses gt for SCORING only.

Usage: scripts/seeker/resolution_probe.py scripts/seeker/data/quad_approach [--conf 0.05]
Run under .venv-seeker (onnxruntime + cv2).
"""
import argparse
import glob
import os
import re
import sys
from collections import defaultdict

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from finetuned_seeker import FinetunedNNSeeker  # noqa: E402
from box_scoring import (  # noqa: E402  -- single source of truth for the gate
    box_hits_gt, gt_scale_for, centre_lag_tol_px, TARGET_EXTENT_M,
)

FX = FY = 539.9363327026367   # sim gz_x500_mono_cam intrinsics @1280x960
CX, CY = 640.0, 480.0
W_FULL, H_FULL = 1280, 960
CROP = 640
V2 = os.path.join(HERE, "weights", "drone_finetuned_quad_v2.onnx")
CROP_W = os.path.join(HERE, "weights", "drone_finetuned_quad_crop.onnx")

_RANGE_RE = re.compile(r"_r(\d+\.?\d*)_")


def gt_range_of(name):
    m = _RANGE_RE.search(name)
    return float(m.group(1)) if m else None


def load_gt_box(label_path):
    """YOLO 'class cx cy w h' (normalized) -> (cx_px, cy_px, w_px, h_px) or None."""
    try:
        parts = open(label_path).readline().split()
    except OSError:
        return None
    if len(parts) < 5:
        return None
    _, cx, cy, w, h = (float(v) for v in parts[:5])
    return cx * W_FULL, cy * H_FULL, w * W_FULL, h * H_FULL


def crop_at(frame, cx, cy, size):
    x0 = int(max(0, min(W_FULL - size, cx - size / 2)))
    y0 = int(max(0, min(H_FULL - size, cy - size / 2)))
    return frame[y0:y0 + size, x0:x0 + size], x0, y0


# box_hits_gt now lives in box_scoring.py (single, unit-tested, pure source of
# truth) -- imported above. The frame-top-sweep scoring debt (off-axis-blind gt,
# un-unified --extent-m, no moving-capture centre-lag) is fixed there; see
# constraint `box-hits-gt-scoring-debt` + tests/test_box_hits_gt.py.


def _capture_extent(data_dir, override):
    """The --extent-m this dataset was LABELLED at (so the scorer can re-scale
    its gt boxes to the one true extent). Honour an explicit override, else read
    the capture manifest, else fall back to classify()'s 0.9 default."""
    if override is not None:
        return override
    mpath = os.path.join(data_dir, "manifest.json")
    if os.path.exists(mpath):
        try:
            import json
            return float(json.load(open(mpath)).get("extent_m", 0.9))
        except (OSError, ValueError, TypeError):
            pass
    return 0.9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("data_dir")
    ap.add_argument("--conf", type=float, default=0.05)
    ap.add_argument("--rmin", type=float, default=8.0)
    ap.add_argument("--rmax", type=float, default=25.0)
    ap.add_argument("--capture-extent-m", type=float, default=None,
                    help="the --extent-m this dataset was labelled at (default: "
                         "read manifest.json, else 0.9). gt boxes are re-scaled "
                         f"to the one true extent {TARGET_EXTENT_M} m (box_scoring).")
    ap.add_argument("--no-offaxis", action="store_true",
                    help="disable the sec^2(theta) off-axis gt widening (debug/A-B)")
    ap.add_argument("--centre-lag-px", type=float, default=0.0,
                    help="extra centre tolerance for MOVING captures (px); "
                         "0 for static set-pose sweeps")
    args = ap.parse_args()
    cap_ext = _capture_extent(args.data_dir, args.capture_extent_m)
    gscale = gt_scale_for(cap_ext)
    offaxis = not args.no_offaxis
    print(f"[probe] gt extent: labelled {cap_ext} m -> unified {TARGET_EXTENT_M} m "
          f"(gt_scale {gscale:.3f}); off-axis-aware={offaxis}; "
          f"centre_lag_px={args.centre_lag_px}")
    v2 = FinetunedNNSeeker(FX, FY, CX, CY, V2, conf_thres=args.conf)
    cm = FinetunedNNSeeker(FX, FY, CX, CY, CROP_W, conf_thres=args.conf) if os.path.exists(CROP_W) else None
    imgs = sorted(glob.glob(os.path.join(args.data_dir, "images", "*.png")))
    lbl_dir = os.path.join(args.data_dir, "labels")
    # per 2 m bin: {arm: [hits, total]}
    bins = defaultdict(lambda: defaultdict(lambda: [0, 0]))
    used = 0
    for ip in imgs:
        base = os.path.splitext(os.path.basename(ip))[0]
        gr = gt_range_of(base)
        if gr is None or not (args.rmin <= gr <= args.rmax):
            continue
        gt = load_gt_box(os.path.join(lbl_dir, base + ".txt"))
        if gt is None:  # negative frame (target not in view) -> skip
            continue
        frame = cv2.imread(ip)
        if frame is None:
            continue
        used += 1
        gcx, gcy, gw, gh = gt
        b = int(gr // 2) * 2
        # shared scoring knobs; the off-axis angle is read from the gt centre
        # relative to the principal point, so a CROP shifts cx/cy by (x0,y0)
        # (the x0 cancels: (gcx-x0)-(CX-x0) = gcx-CX -- true off-axis preserved).
        sk = dict(gt_scale=gscale, offaxis_aware=offaxis, centre_lag_px=args.centre_lag_px)
        # (a) full frame @640 downscale
        hit_a = box_hits_gt(v2._infer_boxes(frame), gcx, gcy, gw, gh, **sk)
        # (b) v2 native gt-centred crop
        crop, x0, y0 = crop_at(frame, gcx, gcy, CROP)
        hit_b = box_hits_gt(v2._infer_boxes(crop), gcx - x0, gcy - y0, gw, gh,
                            cx=CX - x0, cy=CY - y0, **sk)
        # (c) crop model, same crop
        hit_c = box_hits_gt(cm._infer_boxes(crop), gcx - x0, gcy - y0, gw, gh,
                            cx=CX - x0, cy=CY - y0, **sk) if cm else False
        # (d) aim/centre-crop (no gt): crop around image centre; hit if gt is inside + a box lands there
        cropd, xd, yd = crop_at(frame, CX, CY, CROP)
        inside = (xd <= gcx <= xd + CROP) and (yd <= gcy <= yd + CROP)
        hit_d = inside and box_hits_gt(v2._infer_boxes(cropd), gcx - xd, gcy - yd, gw, gh,
                                       cx=CX - xd, cy=CY - yd, **sk)
        for arm, hit in (("a_full640", hit_a), ("b_gtcrop", hit_b),
                         ("c_cropmodel", hit_c), ("d_aimcrop", hit_d)):
            bins[b][arm][0] += int(hit)
            bins[b][arm][1] += 1
    print(f"=== resolution-vs-aspect probe: {used} approach frames ({args.rmin}-{args.rmax} m), conf {args.conf} ===")
    print(f"  {'range':<8}{'a_full640':>12}{'b_gtcrop':>12}{'c_cropmodel':>13}{'d_aimcrop':>12}")
    arms = ["a_full640", "b_gtcrop", "c_cropmodel", "d_aimcrop"]
    tot = defaultdict(lambda: [0, 0])
    for b in sorted(bins):
        cells = []
        for a in arms:
            h, t = bins[b][a]
            tot[a][0] += h
            tot[a][1] += t
            cells.append(f"{100*h/t:.0f}% ({h}/{t})" if t else "-")
        print(f"  {f'{b}-{b+2}':<8}" + "".join(f"{c:>12}" for c in cells[:1]) +
              f"{cells[1]:>12}{cells[2]:>13}{cells[3]:>12}")
    print("  " + "-" * 55)
    tcells = [f"{100*tot[a][0]/tot[a][1]:.0f}%" if tot[a][1] else "-" for a in arms]
    print(f"  {'ALL':<8}{tcells[0]:>12}{tcells[1]:>12}{tcells[2]:>13}{tcells[3]:>12}")
    print("\n  READ: a≈0 reproduces the wall. b/c >> a -> RESOLUTION wall (foveated crop is the lever).")
    print("        b/c ≈ a ≈ 0 -> ASPECT/appearance wall (only real data fixes it). d = deployable ceiling.")


if __name__ == "__main__":
    main()
