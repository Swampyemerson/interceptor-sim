#!/usr/bin/env python3
"""Range calibration for FinetunedNNSeeker (ADR-0040 addendum, SECONDARY lever).

WHY: the fine-tuned seeker's known-size range formula (range = FX * span /
box_width_px, span hardcoded to 1.0 m) underestimates badly in flight
(meas/gt range ratio median ~0.06, n=506 ticks, ADR-0040 addendum) because the
model's boxes run systematically large relative to the assumed 1.0 m body
span. This tool fits an EFFECTIVE span from the static-render val set so that
range = FX * span_m_eff / bw_px lands near 1.0 meas/gt in the median -- and,
as a side effect, is itself a bias DIAGNOSTIC: it isolates how much of the
flight bias is pure box-size scaling (fixable by this calibration) vs
detection pollution / own-prop false-locks (NOT fixable by a range formula --
per the addendum, "a range fix cannot help when the detections themselves are
wrong, only the formula would be"; the PRIMARY v2 lever is hard negatives).

POSITIVES-ONLY, IoU-GATED: only images whose label file has >=1 gt box are
used, and only detections whose box overlaps the labeled box (IoU >=
--min-iou) are kept in the fit -- so a false-lock elsewhere in frame (e.g. an
own-airframe artifact) can't pollute the span estimate.

Fit: span_m_eff = median(gt_range * bw_px / FX) over kept detections. This is
exactly the span value that would make the CURRENT range formula
(range = FX * span / bw_px) read gt_range in the median, given the detector's
actual box widths.

Writes a JSON sidecar next to --weights (<weights>.calib.json) that
FinetunedNNSeeker resolves automatically at load time (see the calibration
resolution order added to finetuned_seeker.py.__init__).

HONESTY: this reads ONLY the rendered image pixels + the training-time label
file (auto-labeled from gt OFFLINE, same scoring-only boundary as gt_* --
CLAUDE.md / render_sim_dataset.py docstring) to fit a lens-geometry constant.
No gt is read at guidance inference time; nothing here touches a live flight.

Usage:
  .venv-seeker/bin/python scripts/seeker/calibrate_range.py \\
      --weights scripts/seeker/weights/drone_finetuned.onnx \\
      --images scripts/seeker/data/sim_dataset/images/val

  # diagnostic only, don't write a sidecar (keeps an arm uncalibrated for A/B):
  .venv-seeker/bin/python scripts/seeker/calibrate_range.py --dry-run
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import sys

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from finetuned_seeker import FinetunedNNSeeker, TARGET_SPAN_M  # noqa: E402

FX = FY = 539.936
CX, CY = 640.0, 480.0

# Matches "..._r04.5_..." (render_sim_dataset.py stems) and the flight-capture
# stem format "flt_000123_r012.3_pos" -- both put gt range right after "_r".
RANGE_RE = re.compile(r"_r(\d+(?:\.\d+)?)")


def parse_gt_range(stem: str):
    m = RANGE_RE.search(stem)
    if not m:
        return None
    return float(m.group(1))


def load_gt_boxes(label_path, w, h):
    """Return a list of (x0,y0,x1,y1) pixel boxes from a YOLO label file."""
    boxes = []
    if not os.path.exists(label_path):
        return boxes
    with open(label_path) as fh:
        for line in fh:
            parts = line.split()
            if len(parts) < 5:
                continue
            cx, cy, bw, bh = (float(v) for v in parts[1:5])
            x0 = (cx - bw / 2.0) * w
            y0 = (cy - bh / 2.0) * h
            x1 = (cx + bw / 2.0) * w
            y1 = (cy + bh / 2.0) * h
            boxes.append((x0, y0, x1, y1))
    return boxes


def iou(box_a, box_b):
    ax0, ay0, ax1, ay1 = box_a
    bx0, by0, bx1, by1 = box_b
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    iw, ih = max(0.0, ix1 - ix0), max(0.0, iy1 - iy0)
    inter = iw * ih
    area_a = max(0.0, ax1 - ax0) * max(0.0, ay1 - ay0)
    area_b = max(0.0, bx1 - bx0) * max(0.0, by1 - by0)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def find_images(images_dir):
    paths = []
    for ext in ("*.png", "*.jpg", "*.jpeg"):
        paths.extend(glob.glob(os.path.join(images_dir, ext)))
    return sorted(paths)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--weights",
                     default=os.path.join(HERE, "weights", "drone_finetuned.onnx"))
    ap.add_argument("--images",
                     default=os.path.join(HERE, "data", "sim_dataset", "images", "val"))
    ap.add_argument("--labels", default=None,
                     help="default: infer by replacing /images/ with /labels/ "
                          "in --images")
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--min-iou", type=float, default=0.3)
    ap.add_argument("--out", default=None,
                     help="default: <weights> + '.calib.json'")
    ap.add_argument("--dry-run", action="store_true",
                     help="compute + print but do not write the sidecar JSON")
    args = ap.parse_args()

    labels_dir = args.labels
    if labels_dir is None:
        images_norm = args.images.rstrip(os.sep)
        labels_dir = images_norm.replace(
            os.sep + "images" + os.sep, os.sep + "labels" + os.sep)
        if os.sep + "images" not in images_norm:
            # fall back: sibling "labels" dir next to whatever dir this is
            labels_dir = os.path.join(os.path.dirname(images_norm), "labels",
                                       os.path.basename(images_norm))
    out_path = args.out or (args.weights + ".calib.json")

    seeker = FinetunedNNSeeker(FX, FY, CX, CY, args.weights, conf_thres=args.conf)

    img_paths = find_images(args.images)
    if not img_paths:
        print(f"[calib] FAILED: no images found under {args.images}")
        return 2

    n_images = 0     # positives considered (label has >=1 box)
    n_detected = 0   # positives where the seeker fired at all
    n_kept = 0       # detections passing the IoU gate w/ a parseable gt_range
    n_no_range = 0   # gt_range unparsable from the filename stem
    span_samples = []
    ratio_before_samples = []  # meas_range(hardcoded 1.0 m span) / gt_range

    for img_path in img_paths:
        stem = os.path.splitext(os.path.basename(img_path))[0]
        label_path = os.path.join(labels_dir, stem + ".txt")
        img = cv2.imread(img_path)
        if img is None:
            continue
        h, w = img.shape[:2]
        gt_boxes = load_gt_boxes(label_path, w, h)
        if not gt_boxes:
            continue  # positives only
        n_images += 1

        gt_range = parse_gt_range(stem)
        if gt_range is None:
            n_no_range += 1
            continue

        det = seeker.detect(img)
        if det.range_m is None or det.box_xywh is None:
            continue
        n_detected += 1

        x, y, bw, bh = det.box_xywh
        det_box = (x, y, x + bw, y + bh)
        best_iou = max(iou(det_box, gt) for gt in gt_boxes)
        if best_iou < args.min_iou:
            continue

        n_kept += 1
        span_samples.append(gt_range * bw / FX)
        # "before" ratio uses the CURRENT hardcoded default span (TARGET_SPAN_M,
        # 1.0 m) explicitly -- independent of whatever span this seeker instance
        # resolved (env / an existing sidecar), so re-running calibration never
        # contaminates the "before" baseline with a prior calibration.
        range_before = FX * TARGET_SPAN_M / max(bw, 1.0)
        ratio_before_samples.append(range_before / gt_range)

    print(f"[calib] weights={os.path.basename(args.weights)}")
    print(f"[calib] images={args.images}")
    print(f"[calib] labels={labels_dir}")
    print(f"[calib] n_images(positives)={n_images} n_detected={n_detected} "
          f"n_kept={n_kept} n_no_range={n_no_range}")

    if n_kept == 0:
        print("[calib] FAILED: 0 kept detections -- nothing to fit")
        return 2

    span_arr = np.array(span_samples)
    span_m_eff = float(np.median(span_arr))
    q1, q3 = float(np.percentile(span_arr, 25)), float(np.percentile(span_arr, 75))

    ratio_before = np.array(ratio_before_samples)
    median_ratio_before = float(np.median(ratio_before))
    # After calibration the median range/gt ratio is 1.0 BY CONSTRUCTION: that
    # is exactly the equation span_m_eff solves.
    median_ratio_after = 1.0

    print(f"[calib] span_m_eff = median(gt_range * bw_px / FX) = "
          f"{span_m_eff:.4f} m  (IQR [{q1:.4f}, {q3:.4f}], n={n_kept})")
    print(f"[calib] median meas/gt range ratio BEFORE calibration "
          f"(hardcoded span={TARGET_SPAN_M:.3f} m) = {median_ratio_before:.4f}")
    print(f"[calib] median meas/gt range ratio AFTER calibration  "
          f"(span_m_eff={span_m_eff:.4f} m, by construction) = "
          f"{median_ratio_after:.4f}")

    result = {
        "span_m_eff": span_m_eff,
        "n_kept": n_kept,
        "iqr": [q1, q3],
        "fit": "median(range_gt*bw_px/FX)",
        "images": args.images,
        "weights": os.path.basename(args.weights),
    }

    if args.dry_run:
        print(f"[calib] --dry-run: NOT writing {out_path}")
        print(f"[calib] would write: {json.dumps(result)}")
    else:
        with open(out_path, "w") as fh:
            json.dump(result, fh, indent=2)
        print(f"[calib] wrote {out_path}")

    return 0 if n_kept >= 20 else 2


if __name__ == "__main__":
    raise SystemExit(main())
