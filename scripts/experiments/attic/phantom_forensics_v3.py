#!/usr/bin/env python3
"""Phase-0 phantom forensics (docs/seeker_v3_dataset_plan.md §3 Phase 0 / §4.2
cross-check / assumption [A5]).

WHY: the T21 flight CSVs logged meas_x/y/z but never the detection BOX, so
there is no on-disk pixel evidence of WHAT the ~18 m phantom locks onto
(plan §3 Phase 0). This script re-runs the DEPLOYED v2 detector
(drone_finetuned_v2.onnx) OFFLINE over the Phase-0 forensics flight frames
(evalv3_01/02 by default), clusters every >=0.25-conf detection by
(u, v, w, h, IoU-vs-gt), and pins the phantom texture(s). Output shapes the
hard-negative harvest (which frame regions to mine, §4) and freezes v2's
baseline phantom/recall numbers on these exact frames (the [A3] floor for
gate G2). These frames belong to the EVAL pool — never training.

Phantom classification reuses ADR-0057's operational definition on the
DEPLOYED (post-self-mask) stream that feeds handoff:
  * GOOD    — matches the true target: IoU>=0.1 vs gt AND range error <=3 m
  * PHANTOM — gross false lock: IoU<0.1 vs gt OR range error >8 m (incl. ANY
              detection on a no-target frame)
  * MID     — in between (3 m < range err <= 8 m, IoU<0.1)
range error = |known-size implied range (calibrated span) − true gt range|.

HONESTY (CLAUDE.md/ADR-0010): gt_* is read here only to SCORE/diagnose
offline; the live detector reads pixels only. This never runs in a control
loop.

RUN (offline, .venv-seeker for onnxruntime):
    .venv-seeker/bin/python scripts/seeker/phantom_forensics_v3.py \
      --eval-dir scripts/seeker/data/v3_flight_eval \
      --run-tags evalv3_01,evalv3_02 --out logs/seeker_v3_phase0
"""
from __future__ import annotations

import argparse
import csv
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cv2  # noqa: E402
import v3_onnx_infer as vi  # noqa: E402


def parse_name(fname: str):
    """capture_flight_frames stem: {run_tag}_{seq:06d}_r{gt_range:05.1f}_{pos|neg}
    -> (run_tag, gt_range_m, positive)."""
    stem = os.path.splitext(fname)[0]
    parts = stem.split("_")
    positive = parts[-1] == "pos"
    gt_range = None
    for p in parts:
        if p.startswith("r") and p[1:].replace(".", "", 1).isdigit():
            try:
                gt_range = float(p[1:])
            except ValueError:
                pass
    # run_tag = everything before the 6-digit zero-padded seq field (the
    # first all-digit, 6-char part). capture_flight_frames stem is
    # {run_tag}_{seq:06d}_r{range}_{pos|neg}; run_tag itself contains an
    # underscore (e.g. evalv3_01), so we cannot just take parts[0].
    seq_idx = next((i for i, p in enumerate(parts) if len(p) == 6 and p.isdigit()),
                   1)
    run_tag = "_".join(parts[:seq_idx]) if seq_idx > 0 else parts[0]
    return run_tag, gt_range, positive


def grid_cell(u, v, W=vi.IMG_W, H=vi.IMG_H, nx=4, ny=4):
    cx = min(nx - 1, int(u / W * nx))
    cy = min(ny - 1, int(v / H * ny))
    return cx, cy


CELL_NAME = {  # 4x4 human labels (col, row) -> description
    (0, 0): "upper-left", (1, 0): "upper-center-L", (2, 0): "upper-center-R", (3, 0): "upper-right",
    (0, 1): "mid-left", (1, 1): "center-L", (2, 1): "center-R", (3, 1): "mid-right",
    (0, 2): "lower-mid-left", (1, 2): "lower-center-L", (2, 2): "lower-center-R", (3, 2): "lower-mid-right",
    (0, 3): "bottom-left", (1, 3): "bottom-center-L", (2, 3): "bottom-center-R", (3, 3): "bottom-right",
}


def classify(iou_gt, range_err, gt_present):
    if not gt_present:
        return "PHANTOM"  # any detection with no target present
    if iou_gt >= 0.1 and range_err is not None and range_err <= 3.0:
        return "GOOD"
    if iou_gt < 0.1 or (range_err is not None and range_err > 8.0):
        return "PHANTOM"
    return "MID"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eval-dir", default=os.path.join(
        os.path.dirname(HERE), "seeker", "data", "v3_flight_eval"))
    ap.add_argument("--run-tags", default="evalv3_01,evalv3_02",
                    help="comma list of flight run_tags to analyze (default the 2 forensics flights)")
    ap.add_argument("--weights", default=os.path.join(HERE, "weights", "drone_finetuned_v2.onnx"))
    ap.add_argument("--out", default=os.path.join(os.path.dirname(HERE), "..", "logs", "seeker_v3_phase0"))
    args = ap.parse_args()

    want = set(t.strip() for t in args.run_tags.split(",") if t.strip())
    img_dir = os.path.join(args.eval_dir, "images")
    lbl_dir = os.path.join(args.eval_dir, "labels")
    if not os.path.isdir(img_dir):
        print(f"[phantom] FAILED: no images dir at {img_dir}")
        return 2
    os.makedirs(args.out, exist_ok=True)

    sess, iname, imgsz = vi.load_session(args.weights)
    span, span_src = vi.load_span(args.weights)
    print(f"[phantom] v2={os.path.basename(args.weights)} imgsz={imgsz} span={span:.4f} ({span_src})")
    print(f"[phantom] eval-dir={args.eval_dir} run-tags={sorted(want)}")

    rows = []
    n_frames = 0
    n_frames_gt = 0
    frames_with_masked_det = 0
    frames_with_masked_phantom = 0
    frames_gt_with_good = 0
    good_confs, phantom_confs = [], []
    phantom_wh = []
    cell_counts = {}
    for fname in sorted(os.listdir(img_dir)):
        if not fname.lower().endswith(".png"):
            continue
        run_tag, gt_range, positive = parse_name(fname)
        if want and run_tag not in want:
            continue
        img = cv2.imread(os.path.join(img_dir, fname))
        if img is None:
            continue
        gt_box = vi.load_gt_box(os.path.join(lbl_dir, os.path.splitext(fname)[0] + ".txt"))
        gt_present = gt_box is not None and min(gt_box[2], gt_box[3]) >= 4.0
        n_frames += 1
        if gt_present:
            n_frames_gt += 1

        raw = vi.infer_raw(sess, iname, imgsz, img, conf=vi.DEPLOY_CONF)
        masked = vi.apply_self_mask(raw)
        if masked:
            frames_with_masked_det += 1
        frame_has_phantom = False
        frame_has_good = False
        for d in masked:
            db = vi.det_box(d)
            iou_gt = vi.iou(db, gt_box) if gt_present else 0.0
            cerr = vi.center_err(db, gt_box) if gt_present else None
            implied = vi.range_from_box(d["w"], span)
            range_err = abs(implied - gt_range) if (gt_range is not None) else None
            klass = classify(iou_gt, range_err, gt_present)
            if klass == "PHANTOM":
                frame_has_phantom = True
                phantom_confs.append(d["score"])
                phantom_wh.append((d["w"], d["h"]))
                cell = grid_cell(d["cx"], d["cy"])
                cell_counts[cell] = cell_counts.get(cell, 0) + 1
            elif klass == "GOOD":
                frame_has_good = True
                good_confs.append(d["score"])
            rows.append({
                "image": fname, "run_tag": run_tag, "gt_range_m": gt_range,
                "gt_present": int(gt_present),
                "u": round(d["cx"], 1), "v": round(d["cy"], 1),
                "w": round(d["w"], 1), "h": round(d["h"], 1),
                "score": round(d["score"], 4),
                "iou_gt": round(iou_gt, 4),
                "center_err_px": (round(cerr, 1) if cerr is not None else ""),
                "implied_range_m": round(implied, 2),
                "range_err_m": (round(range_err, 2) if range_err is not None else ""),
                "klass": klass, "masked": 1,
            })
        if frame_has_phantom:
            frames_with_masked_phantom += 1
        if gt_present and frame_has_good:
            frames_gt_with_good += 1

    if n_frames == 0:
        print("[phantom] FAILED: 0 frames matched the requested run-tags "
              "(has the Phase-0 capture finished writing?)")
        return 2

    csv_path = os.path.join(args.out, "phantom_forensics.csv")
    with open(csv_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()) if rows else
                           ["image", "run_tag", "gt_range_m", "gt_present", "u", "v",
                            "w", "h", "score", "iou_gt", "center_err_px",
                            "implied_range_m", "range_err_m", "klass", "masked"])
        w.writeheader()
        w.writerows(rows)

    def med(xs):
        return statistics.median(xs) if xs else float("nan")

    n_det = len(rows)
    n_phantom = sum(1 for r in rows if r["klass"] == "PHANTOM")
    n_good = sum(1 for r in rows if r["klass"] == "GOOD")
    n_mid = sum(1 for r in rows if r["klass"] == "MID")
    print("\n" + "=" * 70)
    print("PHASE-0 PHANTOM FORENSICS (deployed v2, post-self-mask)")
    print("=" * 70)
    print(f"frames                : {n_frames}  (with gt target >=4px: {n_frames_gt})")
    print(f"post-mask detections  : {n_det}  (GOOD {n_good} / MID {n_mid} / PHANTOM {n_phantom})")
    print(f"frames w/ any det     : {frames_with_masked_det}/{n_frames}")
    print(f"frames w/ PHANTOM det : {frames_with_masked_phantom}/{n_frames}  "
          f"(v2 phantom-frame rate {frames_with_masked_phantom/n_frames:.3f})")
    print(f"gt-frames recovered   : {frames_gt_with_good}/{n_frames_gt}  "
          f"(v2 recall proxy {frames_gt_with_good/max(1,n_frames_gt):.3f})")
    print(f"conf separation       : GOOD median {med(good_confs):.3f}  vs  "
          f"PHANTOM median {med(phantom_confs):.3f}  "
          f"({'INVERTED' if med(phantom_confs) > med(good_confs) else 'ok'} — ADR-0057 saw 0.17 vs 0.34)")
    if phantom_wh:
        pw = med([w for w, _ in phantom_wh]); ph = med([h for _, h in phantom_wh])
        print(f"phantom box size      : median {pw:.0f}x{ph:.0f} px  "
              f"(plan predicted ~293 px wide at 1.7 m implied range)")
    print("\nphantom spatial cluster (4x4 grid of box CENTERS):")
    if cell_counts:
        total = sum(cell_counts.values())
        for cell in sorted(cell_counts, key=lambda c: -cell_counts[c])[:6]:
            c = cell_counts[cell]
            print(f"   {CELL_NAME.get(cell, str(cell)):>16}  {c:4d}  ({c/total:.2f})")
        top_cell = max(cell_counts, key=lambda c: cell_counts[c])
        print(f"\n[A5] leading phantom locus: {CELL_NAME.get(top_cell, str(top_cell))} "
              f"({cell_counts[top_cell]/total:.0%} of phantom dets). Interpret vs the "
              "plan's hypothesis (shadow/prop-arm/horizon interior at banked attitudes) "
              "and shape the §4 mining emphasis accordingly.")
    else:
        print("   (no phantom detections found in these frames)")
    print(f"\n[phantom] wrote {csv_path}")
    print("[phantom] NOTE: these frames are EVAL-pool only; the v2 numbers above "
          "are the [A3] baseline to freeze before v3 training.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
