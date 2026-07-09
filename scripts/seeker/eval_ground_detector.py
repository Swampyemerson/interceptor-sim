#!/usr/bin/env python3
"""T17 gate: offline evaluation of the ground single-class drone detector
on its held-out (val) split. NO SIM -- pure filesystem + ultralytics
inference. Invoked by scripts/check_t17.sh.

MATCHING RULE: a predicted box counts as a match to an image's GT box if
IoU >= --iou-thresh (default 0.3 -- deliberately looser than the classic
COCO 0.5: these targets are 10-35 px wide, where a 2-3 px localization
wobble already erodes IoU well below 0.5 even for a visually "correct"
box; there is no established IoU convention elsewhere in this repo's
seeker evals to match, so this is a new, documented choice). Per positive
image: the highest-confidence matching prediction (if any) is the TP; every
OTHER predicted box on that image (matching or not) counts as an extra FP.
Per negative image: any predicted box is a FP; zero predictions is "clean".

RANGE BANDS: split VAL POSITIVES at the median of their gt-derived range_m
(from split_meta.csv, itself from label_rig_captures.py's projection). See
the T17 eval report / ground_dataset_v1/README.md for why this sweep's
achieved range only spans ~160.1-162.5 m (NOT a wide acquisition-range
test) -- the near/far split is real but narrow.

RUN:
    .venv-seeker-train/bin/python scripts/seeker/eval_ground_detector.py \\
        --weights scripts/seeker/weights/ground_v1.pt \\
        --dataset scripts/seeker/data/ground_dataset_v1
"""
from __future__ import annotations

import argparse
import csv
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))


def load_split_meta(dataset_dir):
    path = os.path.join(dataset_dir, "split_meta.csv")
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    return {r["stem"]: r for r in rows}


def yolo_label_to_xyxy(label_path, W, H):
    """Read a YOLO label file (single line expected for these positives) and
    return (x0,y0,x1,y1) in pixel coords, or None if the file is empty."""
    with open(label_path) as fh:
        lines = [ln for ln in fh.read().splitlines() if ln.strip()]
    if not lines:
        return None
    _, xc, yc, w, h = lines[0].split()
    xc, yc, w, h = float(xc) * W, float(yc) * H, float(w) * W, float(h) * H
    return (xc - w / 2, yc - h / 2, xc + w / 2, yc + h / 2)


def iou(a, b):
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    iw, ih = max(0.0, ix1 - ix0), max(0.0, iy1 - iy0)
    inter = iw * ih
    area_a = max(0.0, ax1 - ax0) * max(0.0, ay1 - ay0)
    area_b = max(0.0, bx1 - bx0) * max(0.0, by1 - by0)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def band_stats(records):
    """records: list of dicts {tp, fn, fp}. Returns (recall, precision, tp, fn, fp)."""
    tp = sum(r["tp"] for r in records)
    fn = sum(r["fn"] for r in records)
    fp = sum(r["fp"] for r in records)
    recall = tp / (tp + fn) if (tp + fn) > 0 else float("nan")
    precision = tp / (tp + fp) if (tp + fp) > 0 else float("nan")
    return recall, precision, tp, fn, fp


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--dataset", default=os.path.join(HERE, "data", "ground_dataset_v1"))
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--iou-thresh", type=float, default=0.3)
    ap.add_argument("--out-json", default=None)
    args = ap.parse_args()

    from ultralytics import YOLO

    meta = load_split_meta(args.dataset)
    val_img_dir = os.path.join(args.dataset, "images", "val")
    val_lbl_dir = os.path.join(args.dataset, "labels", "val")

    val_stems = sorted(os.path.splitext(f)[0] for f in os.listdir(val_img_dir) if f.endswith(".png"))
    img_paths = [os.path.join(val_img_dir, s + ".png") for s in val_stems]

    print(f"[eval] weights={args.weights}")
    print(f"[eval] dataset={args.dataset} val_n={len(val_stems)}")
    print(f"[eval] conf={args.conf} imgsz={args.imgsz} iou_thresh={args.iou_thresh}")

    model = YOLO(args.weights)
    results = model.predict(source=img_paths, conf=args.conf, imgsz=args.imgsz, verbose=False)

    per_image = []
    for stem, res in zip(val_stems, results):
        row = meta[stem]
        is_positive = row["positive"] == "1"
        W, H = res.orig_shape[1], res.orig_shape[0]
        pred_boxes = []
        if res.boxes is not None and len(res.boxes) > 0:
            xyxy = res.boxes.xyxy.cpu().numpy()
            confs = res.boxes.conf.cpu().numpy()
            order = confs.argsort()[::-1]  # highest confidence first
            for i in order:
                pred_boxes.append((float(confs[i]), tuple(float(v) for v in xyxy[i])))

        gt_box = None
        if is_positive:
            lbl_path = os.path.join(val_lbl_dir, stem + ".txt")
            gt_box = yolo_label_to_xyxy(lbl_path, W, H)

        tp = fn = fp = 0
        matched = False
        if is_positive and gt_box is not None:
            for conf, box in pred_boxes:
                if not matched and iou(box, gt_box) >= args.iou_thresh:
                    matched = True
                else:
                    fp += 1
            if matched:
                tp = 1
            else:
                fn = 1
        else:
            fp = len(pred_boxes)  # negative image: every box is a FP

        per_image.append({
            "stem": stem, "positive": is_positive, "n_pred": len(pred_boxes),
            "tp": tp, "fn": fn, "fp": fp,
            "range_m": float(row["range_m"]) if row.get("range_m") else None,
            "source": row["source"],
        })

    # --- range bands (positives only) ---
    pos_records = [r for r in per_image if r["positive"]]
    ranges = sorted(r["range_m"] for r in pos_records)
    median_range = ranges[len(ranges) // 2] if ranges else None
    near = [r for r in pos_records if r["range_m"] <= median_range]
    far = [r for r in pos_records if r["range_m"] > median_range]

    near_recall, near_prec, near_tp, near_fn, near_fp = band_stats(near)
    far_recall, far_prec, far_tp, far_fn, far_fp = band_stats(far)

    neg_records = [r for r in per_image if not r["positive"]]
    n_neg = len(neg_records)
    n_neg_clean = sum(1 for r in neg_records if r["n_pred"] == 0)
    neg_clean_rate = n_neg_clean / n_neg if n_neg > 0 else float("nan")

    if ranges:
        print(f"[eval] val positive range_m: min={ranges[0]:.2f} max={ranges[-1]:.2f} "
              f"median={median_range:.2f}")
    print(f"[eval] NEAR band: n={len(near)} range=[{min((r['range_m'] for r in near), default=float('nan')):.2f}, "
          f"{max((r['range_m'] for r in near), default=float('nan')):.2f}] "
          f"recall={near_recall:.4f} precision={near_prec:.4f} tp={near_tp} fn={near_fn} fp={near_fp}")
    print(f"[eval] FAR band:  n={len(far)} range=[{min((r['range_m'] for r in far), default=float('nan')):.2f}, "
          f"{max((r['range_m'] for r in far), default=float('nan')):.2f}] "
          f"recall={far_recall:.4f} precision={far_prec:.4f} tp={far_tp} fn={far_fn} fp={far_fp}")
    print(f"[eval] NEGATIVES: n={n_neg} clean={n_neg_clean} clean_rate={neg_clean_rate:.4f}")

    # machine-parseable summary line for check_t17.sh
    print(f"SUMMARY near_recall={near_recall:.4f} near_precision={near_prec:.4f} "
          f"far_recall={far_recall:.4f} far_precision={far_prec:.4f} "
          f"neg_clean_rate={neg_clean_rate:.4f} n_near={len(near)} n_far={len(far)} n_neg={n_neg}")

    if args.out_json:
        with open(args.out_json, "w") as fh:
            json.dump({
                "weights": args.weights, "dataset": args.dataset,
                "conf": args.conf, "imgsz": args.imgsz, "iou_thresh": args.iou_thresh,
                "near": {"recall": near_recall, "precision": near_prec, "tp": near_tp,
                         "fn": near_fn, "fp": near_fp, "n": len(near),
                         "range_min": min((r['range_m'] for r in near), default=None),
                         "range_max": max((r['range_m'] for r in near), default=None)},
                "far": {"recall": far_recall, "precision": far_prec, "tp": far_tp,
                        "fn": far_fn, "fp": far_fp, "n": len(far),
                        "range_min": min((r['range_m'] for r in far), default=None),
                        "range_max": max((r['range_m'] for r in far), default=None)},
                "negatives": {"n": n_neg, "n_clean": n_neg_clean, "clean_rate": neg_clean_rate},
                "median_range_m": median_range,
                "per_image": per_image,
            }, fh, indent=2)
        print(f"[eval] wrote {args.out_json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
