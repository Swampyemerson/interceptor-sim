#!/usr/bin/env python3
r"""T17-v2: assemble the multi-range ground-detector YOLO dataset from a
scripts/multirange_capture.py campaign directory (logs/rig_captures/
multirange_v2_<stamp>/standoff_*m/ + negatives/), split HELD-OUT BY
STANDOFF (not per-seq Bernoulli, ADR-0049's v1 weakness -- see below) so the
val set genuinely tests RANGE GENERALIZATION, not interpolation between
near-duplicate frames.

WHY HELD-OUT-BY-STANDOFF (the whole point of T17-v2, ADR-0053): v1's
build_ground_dataset.py drew one random Bernoulli split per POSE SEQUENCE
within a SINGLE ~160 m capture -- train and val ended up ~0.3 deg apart,
near-pixel-identical (ADR-0049 finding #4). Here, EVERY image from the
--held-out-standoffs (default 70 m, 130 m) goes to val and NEVER appears in
train, regardless of pose (grid or jitter); every image from the other
standoffs goes to train. A detector that only memorizes the specific
apparent sizes it trained on cannot pass a held-out-standoff eval by
interpolating between near-identical neighbors -- 70 m and 130 m sit
between the trained standoffs (50/90 and 110/150) but are never seen
themselves.

LABELING: reuses scripts/seeker/label_rig_captures.py's exact projection
math (project_box/rot_z, imported not re-derived) applied per standoff
directory (that script assumes ONE rig pose per invocation; this loops it
over every standoff_*m/ subdirectory in-memory, without writing an
intermediate positives/ directory per standoff).

NEGATIVES: multirange_capture.py's T17-v2 negative-capture phase tags each
negative frame with the rig standoff it was captured at (`rig_standoff_m`
in negatives/index.csv); this script applies the SAME held-out-standoff
split rule to negatives (a negative captured while the rig was at a
held-out standoff goes to val, others to train) -- see the module docstring
of multirange_capture.py's capture_negatives() for why NEG_STANDOFFS_M
deliberately includes both held-out and train standoffs. HONEST CAVEAT
(same as ADR-0049's v1 finding, disclosed again here, not re-litigated):
this project's flat, textureless world was already shown to produce
near-identical negative frames regardless of camera position, so a
standoff-based negative split is unlikely to buy real scene diversity --
this script still performs it (some volume/regularization value, and to
give the gate SOME held-out negatives to compute a real, non-nan clean
rate) but does not claim it as genuine background diversity.

RUN:
    .venv-seeker/bin/python scripts/seeker/build_ground_dataset_v2.py \
        --capture-root logs/rig_captures/multirange_v2_<stamp> \
        --held-out-standoffs 70 130 \
        --out scripts/seeker/data/ground_dataset_v2
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import shutil
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)

from label_rig_captures import project_box, rot_z, TARGET_EXTENT_M, PADDING_FACTOR  # noqa: E402


def discover_standoff_dirs(capture_root):
    return sorted(glob.glob(os.path.join(capture_root, "standoff_*m")))


def standoff_m_from_dir(d):
    tag = os.path.basename(d)  # "standoff_070m"
    digits = tag.replace("standoff_", "").replace("m", "")
    return float(digits)


def label_standoff_dir(rig_dir, extent_m=TARGET_EXTENT_M, padding=PADDING_FACTOR):
    """Projects gt_target_{x,y,z} into both rig cameras for every row of
    `rig_dir`'s index.csv, EXACTLY reproducing label_rig_captures.py's
    main() loop body (same camera-geometry chain, same project_box call) --
    returns (meta, rows) where each row is a dict with the label result
    in-memory (no intermediate labels_left/labels_right/ directory)."""
    with open(os.path.join(rig_dir, "capture_meta.json")) as fh:
        meta = json.load(fh)
    with open(os.path.join(rig_dir, "index.csv"), newline="") as fh:
        rows = list(csv.DictReader(fh))

    W, H = meta["resolution"]
    hfov = meta["rig_hfov_rad"]
    fx = fy = (W / 2.0) / math.tan(hfov / 2.0)
    cx, cy = W / 2.0, H / 2.0
    rig_x, rig_y, rig_z, yaw = meta["rig_pose_xyz_yaw"]
    rig_pos = np.array([rig_x, rig_y, rig_z])
    R = rot_z(yaw)
    baseline_half = meta["rig_baseline_m"] / 2.0
    cam_offsets = {"left": np.array([0.0, +baseline_half, 0.0]),
                   "right": np.array([0.0, -baseline_half, 0.0])}
    cam_world = {side: rig_pos + R @ off for side, off in cam_offsets.items()}

    out_rows = []
    for row in rows:
        seq = int(row["seq"])
        target_world = np.array([float(row["gt_target_x"]), float(row["gt_target_y"]),
                                  float(row["gt_target_z"])])
        for side in ("left", "right"):
            rel_world = target_world - cam_world[side]
            rel_link = R.T @ rel_world
            rel_optical = np.array([-rel_link[1], -rel_link[2], rel_link[0]])
            range_m = float(np.linalg.norm(rel_world))
            img_path = os.path.join(rig_dir, side, f"{seq:05d}.png")
            result, reason = project_box(rel_optical, extent_m, W, H, fx, fy, cx, cy, padding=padding)
            box_norm = result[0] if result else None
            out_rows.append({
                "seq": seq, "camera": side, "image_path": img_path,
                "range_m": range_m, "pose_kind": row.get("dash_direction", ""),
                "written": box_norm is not None, "reason": reason, "box_norm": box_norm,
            })
    return meta, out_rows


def load_negatives(capture_root):
    """Returns list of dicts {seq, rig_standoff_m, camera, image_path} for
    every left/<seq>.png + right/<seq>.png pair under
    capture_root/negatives/, or [] if that directory does not exist."""
    neg_dir = os.path.join(capture_root, "negatives")
    index_path = os.path.join(neg_dir, "index.csv")
    if not os.path.exists(index_path):
        return []
    with open(index_path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    out = []
    for r in rows:
        seq = int(r["seq"])
        standoff_m = float(r["rig_standoff_m"])
        for side in ("left", "right"):
            img_path = os.path.join(neg_dir, side, f"{seq:05d}.png")
            if not os.path.exists(img_path):
                continue
            out.append({"seq": seq, "rig_standoff_m": standoff_m, "camera": side,
                        "image_path": img_path, "pos_name": r.get("pos_name", "")})
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture-root", required=True,
                    help="scripts/multirange_capture.py output dir "
                         "(contains standoff_*m/ and negatives/)")
    ap.add_argument("--held-out-standoffs", type=float, nargs="+", default=[70.0, 130.0],
                    help="standoffs (m) held out ENTIRELY for val -- never appear in train")
    ap.add_argument("--out", default=os.path.join(HERE, "data", "ground_dataset_v2"))
    ap.add_argument("--extent-m", type=float, default=TARGET_EXTENT_M)
    ap.add_argument("--padding", type=float, default=PADDING_FACTOR)
    args = ap.parse_args()

    held_out = set(args.held_out_standoffs)
    standoff_dirs = discover_standoff_dirs(args.capture_root)
    if not standoff_dirs:
        print(f"[build-v2] FAIL: no standoff_*m/ directories under {args.capture_root}")
        return 2
    print(f"[build-v2] capture_root={args.capture_root}")
    print(f"[build-v2] standoffs found: {[standoff_m_from_dir(d) for d in standoff_dirs]}")
    print(f"[build-v2] held-out (val-only) standoffs: {sorted(held_out)}")
    missing_held_out = held_out - set(standoff_m_from_dir(d) for d in standoff_dirs)
    if missing_held_out:
        print(f"[build-v2] FAIL: held-out standoffs {missing_held_out} not found in capture_root")
        return 2

    img_tr = os.path.join(args.out, "images", "train")
    img_va = os.path.join(args.out, "images", "val")
    lbl_tr = os.path.join(args.out, "labels", "train")
    lbl_va = os.path.join(args.out, "labels", "val")
    for d in (img_tr, img_va, lbl_tr, lbl_va):
        os.makedirs(d, exist_ok=True)

    counts = {"train": {"pos": 0, "neg": 0}, "val": {"pos": 0, "neg": 0}}
    split_meta_rows = []

    def write_sample(split, img_dst_dir, lbl_dst_dir, src_image_path, label_lines, stem, extra):
        shutil.copy2(src_image_path, os.path.join(img_dst_dir, stem + ".png"))
        with open(os.path.join(lbl_dst_dir, stem + ".txt"), "w") as fh:
            for line in label_lines:
                fh.write(line + "\n")
        row = {"split": split, "stem": stem, "positive": 1 if label_lines else 0}
        row.update(extra)
        split_meta_rows.append(row)

    # --- positives, held out ENTIRELY by standoff ---
    n_written_total = n_skipped_total = 0
    per_standoff_summary = []
    for d in standoff_dirs:
        standoff_m = standoff_m_from_dir(d)
        is_val = standoff_m in held_out
        split = "val" if is_val else "train"
        img_dir = img_va if is_val else img_tr
        lbl_dir = lbl_va if is_val else lbl_tr

        meta, rows = label_standoff_dir(d, extent_m=args.extent_m, padding=args.padding)
        n_written = n_skipped = 0
        for r in rows:
            if not r["written"]:
                n_skipped += 1
                continue
            xc, yc, bw, bh = r["box_norm"]
            line = f"0 {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}"
            stem = f"pos_standoff{int(round(standoff_m)):03d}m_seq{r['seq']:05d}_{r['camera']}"
            write_sample(split, img_dir, lbl_dir, r["image_path"], [line], stem, {
                "seq": r["seq"], "camera": r["camera"], "range_m": f"{r['range_m']:.4f}",
                "standoff_m": standoff_m, "pose_kind": r["pose_kind"], "source": "positive",
            })
            counts[split]["pos"] += 1
            n_written += 1
        n_written_total += n_written
        n_skipped_total += n_skipped
        per_standoff_summary.append({"standoff_m": standoff_m, "split": split,
                                      "n_written": n_written, "n_skipped": n_skipped})
        print(f"[build-v2] {os.path.basename(d):>16} (standoff={standoff_m:6.1f} m, "
              f"split={split:>5}): wrote {n_written}, skipped {n_skipped}")

    # --- negatives, split by the SAME held-out-standoff rule ---
    neg_rows = load_negatives(args.capture_root)
    print(f"[build-v2] negatives found: {len(neg_rows)} images "
          f"({len(set((r['rig_standoff_m'], r['seq']) for r in neg_rows))} pose-sequences)")
    for r in neg_rows:
        is_val = r["rig_standoff_m"] in held_out
        split = "val" if is_val else "train"
        img_dir = img_va if is_val else img_tr
        lbl_dir = lbl_va if is_val else lbl_tr
        stem = f"neg_standoff{int(round(r['rig_standoff_m'])):03d}m_seq{r['seq']:05d}_{r['camera']}"
        write_sample(split, img_dir, lbl_dir, r["image_path"], [], stem, {
            "seq": r["seq"], "camera": r["camera"], "range_m": "",
            "standoff_m": r["rig_standoff_m"], "pose_kind": r["pos_name"], "source": "negative",
        })
        counts[split]["neg"] += 1

    split_meta_path = os.path.join(args.out, "split_meta.csv")
    with open(split_meta_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["split", "stem", "positive", "seq", "camera",
                                            "range_m", "standoff_m", "pose_kind", "source"])
        w.writeheader()
        w.writerows(split_meta_rows)

    data_yaml = os.path.join(args.out, "data.yaml")
    with open(data_yaml, "w") as fh:
        fh.write(
            "# T17-v2 ground single-class drone detector (ADR-0047: acquisition-only,\n"
            "# no interceptor class, no type discrimination). ADR-0053 fix: multi-range\n"
            "# training, held out BY STANDOFF (see build_ground_dataset_v2.py) so the\n"
            "# val split tests range GENERALIZATION, not interpolation.\n"
            f"path: {os.path.abspath(args.out)}\n"
            "train: images/train\n"
            "val: images/val\n"
            "nc: 1\n"
            "names: ['drone']\n"
        )

    n_tr = counts["train"]["pos"] + counts["train"]["neg"]
    n_va = counts["val"]["pos"] + counts["val"]["neg"]
    print(f"\n[build-v2] train: pos={counts['train']['pos']} neg={counts['train']['neg']} (total {n_tr})")
    print(f"[build-v2] val:   pos={counts['val']['pos']} neg={counts['val']['neg']} (total {n_va})")
    print(f"[build-v2] dataset: {data_yaml}")
    print(f"[build-v2] split_meta: {split_meta_path}")

    readme_path = os.path.join(args.out, "README.md")
    with open(readme_path, "w") as fh:
        standoffs_all = sorted(standoff_m_from_dir(d) for d in standoff_dirs)
        train_standoffs = sorted(s for s in standoffs_all if s not in held_out)
        fh.write(f"""# ground_dataset_v2 (T17-v2, ADR-0053 fix)

Single-class "drone" detector dataset for the ground stereo rig (ADR-0047:
acquisition-only, no interceptor class, no type discrimination). Fixes the
out-of-domain bias ADR-0053 measured in ground_v1 (trained on a single
~160 m band, -6.3 m bias at 50 m, non-monotonic box-size head).

## Provenance

- **Capture**: `{args.capture_root}` (scripts/multirange_capture.py, T17-v2
  extension: 7 standoffs, wedge-sized Y-grid per standoff, X/Z jitter for
  apparent-size variety, integrated multi-standoff negative capture).
- **Standoffs captured**: {standoffs_all} m.
- **HELD OUT (val only, NEVER in train)**: {sorted(held_out)} m -- this is
  the whole point of v2 vs v1: v1's held-out split was ~0.3 deg of
  near-pixel-identical interpolation (ADR-0049); here the val standoffs sit
  BETWEEN trained standoffs and are never seen in any pose during training.
- **Train standoffs**: {train_standoffs} m.
- **Positives**: {counts['train']['pos'] + counts['val']['pos']} images,
  projected from gt_target_{{x,y,z}} exactly as
  scripts/seeker/label_rig_captures.py does (CLAUDE.md gt_* scoring/label-
  only honesty boundary -- the trained detector reads pixels only).
- **Negatives**: {counts['train']['neg'] + counts['val']['neg']} images,
  split by the rig standoff they were captured at (same held-out rule as
  positives). HONEST CAVEAT (ADR-0049, re-confirmed not re-litigated): this
  world's flat, textureless ground + uniform sky gives near-identical
  negative frames regardless of camera position -- see the eval report for
  whether that held here too.

## Split

By STANDOFF, not by pose or per-image Bernoulli draw. Every image (grid
AND jitter poses, both cameras) from a held-out standoff goes to val;
every image from every other standoff goes to train. See split_meta.csv
for the per-sample assignment (standoff_m / pose_kind columns).
""")
    print(f"[build-v2] README: {readme_path}")
    print(f"[build-v2] per-standoff summary: {json.dumps(per_standoff_summary, indent=2)}")
    return 0 if n_written_total > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
