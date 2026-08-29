#!/usr/bin/env python3
"""Synthetic motion-blur replay harness — design-review Action 1(a), gap G1.

WHY: every in-sim detection number so far comes from instantaneous, perfectly
sharp Gazebo renders. The real terminal phase runs 32-58 deg/s line-of-sight
rates; a rolling/global-shutter camera integrating over a finite exposure
smears the target across pixels exactly where the guidance budget has least
slack. This harness replays ALREADY-CAPTURED frames (no sim boot) through the
DEPLOYED markerless detector with synthetic linear motion blur applied, and
measures detection rate + bearing error vs blur length — the first
quantitative curve for that gap.

BLUR <-> PHYSICS MAPPING (documented assumption; flagged for a bench check):
  The sim camera is 1280x960 with fx = fy = 539.936 px (configs/camera_intrinsics.json,
  the same constants markerless_loop.py hardcodes). Horizontal FOV:
      HFOV = 2*atan((1280/2)/539.936) = 2*atan(1.18533) = 99.72 deg
  Linear-average pixels per degree (task convention, frame width over HFOV):
      PX_PER_DEG = 1280 / 99.72 = 12.84 px/deg
  (Boresight-exact value is fx*pi/180 = 9.42 px/deg; the linear average is
  ~36% larger, i.e. it errs toward LONGER blur for a given LOS rate =
  worst-credible-leaning. Real-camera check needed: actual exposure time and
  shutter type of the flight camera are UNMEASURED — {2, 5, 10} ms brackets a
  plausible auto-exposure range outdoors-to-dim.)
  Blur length in pixels for a target smearing at the LOS rate:
      blur_px = LOS_rate(deg/s) * exposure(s) * PX_PER_DEG
  Design band 32-58 deg/s x {2,5,10} ms:
      32 deg/s * 2 ms * 12.84 = 0.8 px     58 * 2 ms  = 1.5 px
      32 * 5 ms = 2.1 px                   58 * 5 ms  = 3.7 px
      45 * 5 ms = 2.9 px                   32 * 10 ms = 4.1 px
      45 * 10 ms = 5.8 px                  58 * 10 ms = 7.4 px
  Default sweep levels {0, 2, 3, 4, 6, 8, 12} px therefore cover the whole
  design band (0 = paired clean control; 12 px is a deliberate beyond-design
  stress point, ~90 deg/s @ 10 ms).
  MODEL NOTE: blur is applied to the WHOLE frame — correct for the dominant
  rotational (body-rate/LOS-rate) smear, which moves the entire scene across
  the sensor; target-only translational smear is a second-order refinement.

FRAME SOURCE: scripts/seeker/data/v3_grid (READ-ONLY — owned by the v3
pipeline; this script only reads images/labels/index.csv). Each positive frame
has a gt YOLO label (class cx cy w h, normalized) projected from sim ground
truth. Frames are pre-filtered so the CLEAN gt box passes the deployed
self-mask geometry (|bearing| <= 26 deg, box >= 44 px from the L/R edges;
mask is 30 deg / 40 px — the margin keeps mask-edge rejection from
confounding the blur curve). The filter count is reported.

DETECTOR: FinetunedNNSeeker (scripts/seeker/finetuned_seeker.py) imported and
called as-is — the exact deployed operating point markerless_loop.py uses:
weights drone_finetuned_v2.onnx, conf 0.25, self-mask ON (max bearing 30 deg,
edge margin 40 px), calib-sidecar span. No geometry is reimplemented here.

MATCH RULE: identical to eval_seeker_v3.py — IoU >= 0.25 OR box-center error
< 25 px vs gt (iou/center_err imported from v3_onnx_infer.py).

STRETCH (CSRT): per frame and per blur level, a FRESH cv2 CSRT tracker
(detect_track.py's default kind per validate_track_kind) is seeded on the
CLEAN frame at the gt box, then updated ONCE on the blurred variant of the
same frame. The target has not moved, so a healthy tracker must stay on it;
this isolates pure blur sensitivity of the correlation tracker, paired
frame-for-frame with the NN. It is a single-step probe, not a full-sequence
track — sequence-level blur drift is a TODO for a flight-log replay.

HONESTY: gt labels are used for SCORING ONLY (as in every seeker eval); the
detector sees pixels + fixed intrinsics, nothing else.

Usage (small validation sample, default ~42 frames x 7 levels):
  .venv-seeker/bin/python scripts/experiments/blur_replay.py

Full sweep (printed again at the end of every run):
  .venv-seeker/bin/python scripts/experiments/blur_replay.py \
      --ranges all --per-range 0 \
      --out scripts/experiments/logs/blur_replay_full.csv
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
from typing import List, Optional, Tuple

import numpy as np
import cv2

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SEEKER_DIR = os.path.join(REPO, "scripts", "seeker")
sys.path.insert(0, SEEKER_DIR)

from finetuned_seeker import FinetunedNNSeeker          # noqa: E402 (deployed detector)
from v3_onnx_infer import iou, center_err               # noqa: E402 (eval-v3 match helpers)

# ---- deployed camera constants (configs/camera_intrinsics.json / markerless_loop.py) ----
FX = FY = 539.936
CX, CY = 640.0, 480.0
FRAME_W, FRAME_H = 1280, 960
HFOV_DEG = 2.0 * math.degrees(math.atan((FRAME_W / 2.0) / FX))     # 99.72 deg
PX_PER_DEG = FRAME_W / HFOV_DEG                                    # 12.84 px/deg

# ---- eval_seeker_v3 match rule (kept numerically identical) ----
MATCH_IOU_THRESH = 0.25
MATCH_CENTER_PX = 25.0

# ---- clean-frame mask-geometry pre-filter (self-mask is 30 deg / 40 px) ----
MAX_GT_BEARING_DEG = 26.0
GT_EDGE_MARGIN_PX = 44.0

DESIGN_LOS_DEG_S = (32.0, 58.0)
EXPOSURES_MS = (2.0, 5.0, 10.0)

DEFAULT_WEIGHTS = os.path.join(SEEKER_DIR, "weights", "drone_finetuned_v2.onnx")
DEFAULT_DATA = os.path.join(SEEKER_DIR, "data", "v3_grid")
DEFAULT_OUT = os.path.join(REPO, "scripts", "experiments", "logs",
                           "blur_replay_sample.csv")
DEFAULT_RANGES = "1.5,2,3,4,6,8,12"     # terminal-relevant rungs of the grid
DEFAULT_BLUR_PX = "0,2,3,4,6,8,12"


def bearing_deg_of_u(u: float) -> float:
    """Horizontal bearing (deg, + = right) of image column u — the seeker's own
    geometry (finetuned_seeker.detect)."""
    return math.degrees(math.atan2((u - CX) / FX, 1.0))


def blur_annotation(blur_px: float) -> str:
    """Which (LOS rate @ exposure) combos a blur length represents."""
    if blur_px <= 0:
        return "clean control"
    parts = []
    for exp_ms in EXPOSURES_MS:
        los = blur_px / (PX_PER_DEG * exp_ms * 1e-3)
        tag = "" if DESIGN_LOS_DEG_S[0] <= los <= DESIGN_LOS_DEG_S[1] else \
            ("<band" if los < DESIGN_LOS_DEG_S[0] else ">band")
        parts.append(f"{los:.0f}d/s@{exp_ms:.0f}ms{tag}")
    return " | ".join(parts)


def motion_kernel(length_px: int, angle_deg: float) -> Optional[np.ndarray]:
    """Normalized linear motion-blur kernel: a 1-px-wide line of `length_px`
    through the kernel center at `angle_deg` (0 = horizontal). None = identity.
    Even lengths center within 0.5 px of the true centroid (<< the 25 px match
    tolerance), so gt centers stay valid for scoring."""
    if length_px <= 1:
        return None
    size = length_px if length_px % 2 == 1 else length_px + 1
    k = np.zeros((size, size), np.float32)
    k[size // 2, (size - length_px + 1) // 2:(size - length_px + 1) // 2 + length_px] = 1.0
    if angle_deg % 360.0 != 0.0:
        m = cv2.getRotationMatrix2D(((size - 1) / 2.0, (size - 1) / 2.0),
                                    angle_deg, 1.0)
        k = cv2.warpAffine(k, m, (size, size))
    s = k.sum()
    assert s > 0
    return k / s


def apply_blur(frame_bgr: np.ndarray, kernel: Optional[np.ndarray]) -> np.ndarray:
    if kernel is None:
        return frame_bgr
    return cv2.filter2D(frame_bgr, -1, kernel)


def load_frames(data_dir: str, ranges: Optional[List[float]], per_range: int,
                seed: int) -> Tuple[List[dict], int, int]:
    """Positive frames whose CLEAN gt box passes the deployed-mask geometry,
    stratified-sampled per range ring. Returns (sample, n_positive, n_pass)."""
    idx_path = os.path.join(data_dir, "index.csv")
    with open(idx_path) as fh:
        rows = list(csv.DictReader(fh))
    n_pos, candidates = 0, []
    for r in rows:
        if r["kind"] != "positive":
            continue
        n_pos += 1
        lp = os.path.join(data_dir, "labels", r["stem"] + ".txt")
        if not os.path.exists(lp):
            continue
        txt = open(lp).read().split()
        if len(txt) < 5:
            continue
        # YOLO normalized (class cx cy w h) -> pixel center box
        gcx, gcy = float(txt[1]) * FRAME_W, float(txt[2]) * FRAME_H
        gw, gh = float(txt[3]) * FRAME_W, float(txt[4]) * FRAME_H
        b = bearing_deg_of_u(gcx)
        if abs(b) > MAX_GT_BEARING_DEG:
            continue
        if (gcx - gw / 2) < GT_EDGE_MARGIN_PX or \
           (gcx + gw / 2) > (FRAME_W - GT_EDGE_MARGIN_PX):
            continue
        candidates.append({
            "stem": r["stem"], "range_m": float(r["range_m"]),
            "gt_box": (gcx, gcy, gw, gh), "gt_bearing_deg": b,
            "img": os.path.join(data_dir, "images", r["stem"] + ".png"),
        })
    n_pass = len(candidates)
    if ranges is not None:
        candidates = [c for c in candidates
                      if any(abs(c["range_m"] - rr) < 1e-6 for rr in ranges)]
    rng = np.random.default_rng(seed)
    sample: List[dict] = []
    for rr in sorted({c["range_m"] for c in candidates}):
        ring = sorted((c for c in candidates if c["range_m"] == rr),
                      key=lambda c: c["stem"])
        if per_range > 0 and len(ring) > per_range:
            ring = [ring[i] for i in sorted(
                rng.choice(len(ring), size=per_range, replace=False))]
        sample.extend(ring)
    return sample, n_pos, n_pass


def run_csrt_probe(clean_bgr, blurred_bgr, gt_box) -> Tuple[bool, Optional[float]]:
    """Seed CSRT (detect_track.py's default tracker kind) on the clean frame at
    the gt box, update once on the blurred same-frame. Returns (held_on_target,
    center_err_px or None)."""
    gcx, gcy, gw, gh = gt_box
    seed = (int(round(gcx - gw / 2)), int(round(gcy - gh / 2)),
            max(int(round(gw)), 4), max(int(round(gh)), 4))
    trk = cv2.TrackerCSRT_create()
    try:
        trk.init(clean_bgr, seed)
        ok, box = trk.update(blurred_bgr)
    except cv2.error:
        return False, None
    if not ok:
        return False, None
    tb = (box[0] + box[2] / 2.0, box[1] + box[3] / 2.0, box[2], box[3])
    ce = center_err(tb, gt_box)
    held = iou(tb, gt_box) >= MATCH_IOU_THRESH or ce < MATCH_CENTER_PX
    return held, ce


def median_or_nan(vals: List[float]) -> float:
    return statistics.median(vals) if vals else float("nan")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--weights", default=DEFAULT_WEIGHTS)
    ap.add_argument("--conf", type=float, default=0.25,
                    help="deployed operating point (MARKERLESS_NN_CONF default)")
    ap.add_argument("--data", default=DEFAULT_DATA)
    ap.add_argument("--ranges", default=DEFAULT_RANGES,
                    help="comma list of grid range rings, or 'all'")
    ap.add_argument("--per-range", type=int, default=6,
                    help="frames sampled per range ring (0 = all)")
    ap.add_argument("--blur-px", default=DEFAULT_BLUR_PX,
                    help="comma list of blur lengths in px (0 = clean control)")
    ap.add_argument("--angle", type=float, default=0.0,
                    help="blur direction deg (0 = horizontal — worst case for "
                         "the guidance-primary horizontal bearing)")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--no-csrt", action="store_true")
    ap.add_argument("--out", default=DEFAULT_OUT)
    a = ap.parse_args()

    cv2.setNumThreads(2)   # stay polite next to the v3 capture/train pipeline

    ranges = None if a.ranges.strip().lower() == "all" else \
        [float(x) for x in a.ranges.split(",") if x.strip()]
    blur_levels = [int(float(x)) for x in a.blur_px.split(",") if x.strip()]

    sample, n_pos, n_pass = load_frames(a.data, ranges, a.per_range, a.seed)
    if not sample:
        print("ERROR: no suitable (frame, gt) pairs found — check --data/--ranges")
        return 1
    print(f"[blur_replay] data={a.data}")
    print(f"[blur_replay] positives={n_pos}, pass mask-geometry filter "
          f"(|bearing|<={MAX_GT_BEARING_DEG} deg, {GT_EDGE_MARGIN_PX:.0f} px "
          f"edge)={n_pass}, sampled={len(sample)} "
          f"(per-range={a.per_range or 'all'}, seed={a.seed})")
    print(f"[blur_replay] HFOV={HFOV_DEG:.2f} deg  PX_PER_DEG={PX_PER_DEG:.2f} "
          f"(boresight-exact {FX * math.pi / 180.0:.2f})  angle={a.angle:.0f} deg")

    seeker = FinetunedNNSeeker(FX, FY, CX, CY, a.weights, conf_thres=a.conf)

    fields = ["stem", "range_m", "gt_bearing_deg", "blur_px", "angle_deg",
              "nn_fired", "nn_matched", "nn_center_err_px", "nn_dbearing_deg",
              "nn_conf", "csrt_ok", "csrt_held", "csrt_center_err_px"]
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    rows_out: List[dict] = []

    for fr in sample:
        clean = cv2.imread(fr["img"])
        if clean is None:
            print(f"WARN: unreadable {fr['img']}, skipping")
            continue
        gt_box = fr["gt_box"]
        for L in blur_levels:
            blurred = apply_blur(clean, motion_kernel(L, a.angle))
            det = seeker.detect(blurred)
            fired = det.box_xywh is not None
            matched, ce, dbear, conf = False, None, None, None
            if fired:
                x0, y0, bw, bh = det.box_xywh
                db = (x0 + bw / 2.0, y0 + bh / 2.0, bw, bh)
                ce = center_err(db, gt_box)
                matched = iou(db, gt_box) >= MATCH_IOU_THRESH or ce < MATCH_CENTER_PX
                dbear = bearing_deg_of_u(db[0]) - bearing_deg_of_u(gt_box[0])
                conf = det.decision_margin
            row = {"stem": fr["stem"], "range_m": fr["range_m"],
                   "gt_bearing_deg": round(fr["gt_bearing_deg"], 2),
                   "blur_px": L, "angle_deg": a.angle,
                   "nn_fired": int(fired), "nn_matched": int(matched),
                   "nn_center_err_px": None if ce is None else round(ce, 2),
                   "nn_dbearing_deg": None if dbear is None else round(dbear, 3),
                   "nn_conf": None if conf is None else round(conf, 3),
                   "csrt_ok": "", "csrt_held": "", "csrt_center_err_px": ""}
            if not a.no_csrt:
                held, tce = run_csrt_probe(clean, blurred, gt_box)
                row["csrt_ok"] = int(tce is not None)
                row["csrt_held"] = int(held)
                row["csrt_center_err_px"] = "" if tce is None else round(tce, 2)
            rows_out.append(row)

    with open(a.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        w.writerows(rows_out)
    print(f"[blur_replay] wrote {len(rows_out)} rows -> {a.out}")

    # ---------------- summary table ----------------
    print("\nblur_px | represents (LOS@exposure)            | n  | NN det | NN "
          "med ctr px | med |dbrg| deg | med conf | CSRT held | CSRT med px")
    print("-" * 132)
    for L in blur_levels:
        rl = [r for r in rows_out if r["blur_px"] == L]
        n = len(rl)
        det_rate = sum(r["nn_matched"] for r in rl) / n if n else float("nan")
        ces = [r["nn_center_err_px"] for r in rl
               if r["nn_matched"] and r["nn_center_err_px"] is not None]
        dbs = [abs(r["nn_dbearing_deg"]) for r in rl
               if r["nn_matched"] and r["nn_dbearing_deg"] is not None]
        cfs = [r["nn_conf"] for r in rl if r["nn_matched"] and r["nn_conf"] is not None]
        if a.no_csrt:
            csrt_s, csrt_e = "  --", "  --"
        else:
            ch = sum(1 for r in rl if r["csrt_held"] == 1)
            tes = [r["csrt_center_err_px"] for r in rl
                   if r["csrt_center_err_px"] != ""]
            csrt_s = f"{ch / n:.2f}" if n else "nan"
            csrt_e = f"{median_or_nan(tes):.1f}"
        print(f"{L:7d} | {blur_annotation(L):37s} | {n:2d} | {det_rate:6.2f} | "
              f"{median_or_nan(ces):10.1f} | {median_or_nan(dbs):13.3f} | "
              f"{median_or_nan(cfs):8.3f} | {csrt_s:>9s} | {csrt_e:>11s}")

    print("\nNN det = fraction of gt-present frames the deployed detector fires "
          f"within eval-v3 tolerance (IoU>={MATCH_IOU_THRESH} or <{MATCH_CENTER_PX:.0f} px). "
          "CSRT held = same-tolerance single-step hold, seeded clean.")
    print("\nFull sweep command (run at idle, NOT alongside a training run):\n"
          f"  {REPO}/.venv-seeker/bin/python {os.path.abspath(__file__)} "
          "--ranges all --per-range 0 "
          f"--out {os.path.join(REPO, 'scripts', 'experiments', 'logs', 'blur_replay_full.csv')}\n"
          "  (optionally repeat with --angle 45 and --angle 90 for direction "
          "sensitivity)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
