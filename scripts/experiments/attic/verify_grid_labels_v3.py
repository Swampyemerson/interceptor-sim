#!/usr/bin/env python3
"""Phase-1 auto-label verification for the v3 static grid
(docs/seeker_v3_dataset_plan.md §5 label-verification step / [A7]).

Two independent checks on scripts/seeker/data/v3_grid BEFORE the batch is
trusted for training:

  (1) GEOMETRY SELF-CONSISTENCY (all positives): the label box width must
      equal the gt projection fx*extent/range within tolerance, i.e. the
      auto-label really is the gt projected through the intrinsics (not a
      stale/mislabeled frame). Uses index.csv's commanded range_m and the
      label .txt box. Flags any cell whose |w_px - fx*0.9/range| is large.

  (2) PIXEL CROSS-CHECK (close-range boresight cells): run the DEPLOYED v2
      detector (pixels only) and require its best masked box to overlap the
      gt-projected label (median IoU). Two INDEPENDENT chains — gt-projection
      vs a pixels-only detector — agreeing at close range is strong evidence
      the labels are correct. (v2 is reliable <=12 m boresight, ADR-0042 G1.)

Also renders --n-draw sample frames with their label boxes drawn, for a human
eyeball (saved under --out), matching the ADR-0042 "labels visually verified"
precedent.

HONESTY: gt_* used offline only, to verify manufactured labels; v2 reads
pixels only. Offline (onnxruntime + cv2), no sim.

RUN (after grid_capture_v3.sh finishes):
    .venv-seeker/bin/python scripts/seeker/verify_grid_labels_v3.py
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cv2  # noqa: E402
import v3_onnx_infer as vi  # noqa: E402

EXTENT_M = 0.9  # render_sim_dataset / seeker_v3_capture default silhouette extent


def load_label_box(path, W=vi.IMG_W, H=vi.IMG_H):
    return vi.load_gt_box(path)  # (cx,cy,w,h) px or None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grid", default=os.path.join(HERE, "data", "v3_grid"))
    ap.add_argument("--weights", default=os.path.join(HERE, "weights", "drone_finetuned_v2.onnx"))
    ap.add_argument("--out", default=os.path.join(os.path.dirname(HERE), "..",
                                                  "logs", "seeker_v3_phase0", "grid_label_check"))
    ap.add_argument("--n-draw", type=int, default=15)
    ap.add_argument("--geom-tol-frac", type=float, default=0.25,
                    help="max fractional deviation of label width from fx*extent/range")
    ap.add_argument("--iou-pass", type=float, default=0.4,
                    help="min median close-range v2-vs-label IoU to PASS check (2)")
    args = ap.parse_args()

    idx_path = os.path.join(args.grid, "index.csv")
    img_dir = os.path.join(args.grid, "images")
    lbl_dir = os.path.join(args.grid, "labels")
    if not os.path.isfile(idx_path):
        print(f"[verify] FAILED: no index.csv at {idx_path} (grid capture not finished?)")
        return 2
    os.makedirs(args.out, exist_ok=True)

    rows = []
    with open(idx_path) as fh:
        for r in csv.DictReader(fh):
            if r.get("kind") == "positive":
                rows.append(r)
    if not rows:
        print("[verify] FAILED: no positive rows in index.csv")
        return 2
    print(f"[verify] {len(rows)} grid positives in {args.grid}")

    # ---- (1) geometry self-consistency over ALL positives ----
    geom_bad = []
    geom_dev = []
    for r in rows:
        stem = r["stem"]
        box = load_label_box(os.path.join(lbl_dir, stem + ".txt"))
        if box is None:
            geom_bad.append((stem, "no/empty label"))
            continue
        rng = float(r["range_m"])
        exp_w = vi.FX * EXTENT_M / rng
        dev = abs(box[2] - exp_w) / max(exp_w, 1e-6)
        geom_dev.append(dev)
        if dev > args.geom_tol_frac:
            geom_bad.append((stem, f"w={box[2]:.1f}px exp={exp_w:.1f}px dev={dev:.2f}"))
    med_dev = statistics.median(geom_dev) if geom_dev else float("nan")
    print(f"[verify] (1) geometry: median |w-fx*ext/range|/exp = {med_dev:.3f}; "
          f"{len(geom_bad)} cells out of tol ({args.geom_tol_frac})")
    for s, why in geom_bad[:8]:
        print(f"         BAD {s}: {why}")

    # ---- (2) pixel cross-check on close-range boresight cells ----
    sess, iname, imgsz = vi.load_session(args.weights)
    span, _ = vi.load_span(args.weights)
    close = [r for r in rows
             if float(r["range_m"]) <= 8.0 and abs(float(r["bearing_deg"])) < 1.0
             and abs(float(r["yaw_deg"])) < 1.0 and abs(float(r["height_m"]) - 0.5) < 0.01]
    ious, fired = [], 0
    for r in close:
        stem = r["stem"]
        img = cv2.imread(os.path.join(img_dir, stem + ".png"))
        box = load_label_box(os.path.join(lbl_dir, stem + ".txt"))
        if img is None or box is None:
            continue
        det = vi.best_by_score(vi.apply_self_mask(vi.infer_raw(sess, iname, imgsz, img)))
        if det is None:
            continue
        fired += 1
        ious.append(vi.iou(vi.det_box(det), box))
    med_iou = statistics.median(ious) if ious else float("nan")
    print(f"[verify] (2) pixel cross-check: v2 fired on {fired}/{len(close)} close boresight "
          f"cells; median IoU(v2, label) = {med_iou:.3f}")

    # ---- sample renders for eyeball ----
    step = max(1, len(rows) // args.n_draw)
    drawn = 0
    for r in rows[::step][:args.n_draw]:
        stem = r["stem"]
        img = cv2.imread(os.path.join(img_dir, stem + ".png"))
        box = load_label_box(os.path.join(lbl_dir, stem + ".txt"))
        if img is None or box is None:
            continue
        cx, cy, w, h = box
        cv2.rectangle(img, (int(cx - w / 2), int(cy - h / 2)),
                      (int(cx + w / 2), int(cy + h / 2)), (0, 255, 0), 2)
        cv2.putText(img, f"r{r['range_m']}m b{r['bearing_deg']} yaw{r['yaw_deg']}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imwrite(os.path.join(args.out, stem + "_lbl.png"), img)
        drawn += 1
    print(f"[verify] drew {drawn} annotated samples -> {args.out}")

    geom_ok = len(geom_bad) <= max(2, int(0.01 * len(rows)))  # allow <=1% or 2 cells
    pixel_ok = (not math.isnan(med_iou)) and med_iou >= args.iou_pass and fired >= max(1, len(close) // 2)
    ok = geom_ok and pixel_ok
    print(f"[verify] geometry_ok={geom_ok} pixel_ok={pixel_ok} => "
          f"{'LABELS VALIDATED' if ok else 'LABELS SUSPECT — inspect before training'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
