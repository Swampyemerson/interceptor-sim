#!/usr/bin/env python3
"""T17: assemble the final ground-detector YOLO dataset from
label_rig_captures.py's positives (scripts/seeker/data/ground_v1_positives)
and a negatives capture directory (logs/rig_captures/negatives_<runstamp>),
split by POSE SEQUENCE (not per-image) so left/right frames of the same
capture never straddle train/val -- they are near-duplicate, highly
correlated views (same target pose, ~2 m apart), so a per-image random
split would leak geometry between the two and inflate held-out recall.

WHY SEQ-LEVEL SPLIT: scripts/seeker/merge_dataset_v2.py (the onboard v2
recipe this reuses the shape of) splits per-IMAGE, which is fine there
because its capture sources are either single-camera renders or in-flight
frames with no L/R sibling. The ground rig always captures a stereo PAIR
per pose -- splitting siblings apart is a leakage bug this script avoids by
construction (one Bernoulli draw per seq/group, both cameras follow it).

NEGATIVES CAVEAT (report this, do not bury it): the empty-wedge negative
burst (capture_rig_negatives.py, run against a flat, textureless ground
plane + uniform sky with the rig yawed away from the corridor) produced
frames that are BYTE-IDENTICAL within each camera (confirmed by hash --
0 real-world diversity, since nothing in the scene differs between shots).
Splitting 30 identical copies 80/20 is a split in name only: train and val
literally share pixel-identical content for the negative class. This script
still performs the mechanical split (for volume / regularization value, and
to give the gate SOME negative frames in both splits) but the dataset
README written here says so explicitly, and the eval report must not
overstate what a "held-out negative" means here.

RUN:
    .venv-seeker/bin/python scripts/seeker/build_ground_dataset.py \\
        --positives scripts/seeker/data/ground_v1_positives \\
        --negatives logs/rig_captures/negatives_20260709T021857Z \\
        --out scripts/seeker/data/ground_dataset_v1 --val-frac 0.2 --seed 0
"""
from __future__ import annotations

import argparse
import csv
import os
import shutil

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))


def load_positives(positives_dir):
    """Returns dict seq -> list of dicts {camera, image_path, label_path,
    range_m, off_boresight_deg, dash_direction}, only rows with written==1."""
    meta_path = os.path.join(positives_dir, "meta.csv")
    with open(meta_path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    groups = {}
    for r in rows:
        if r["written"] != "1":
            continue
        seq = int(r["seq"])
        groups.setdefault(seq, []).append(r)
    return groups


def load_negatives(negatives_dir):
    """Returns dict seq -> list of dicts {camera, image_path} for every
    left/<seq>.png + right/<seq>.png pair under negatives_dir."""
    left_dir = os.path.join(negatives_dir, "left")
    right_dir = os.path.join(negatives_dir, "right")
    groups = {}
    for side, d in (("left", left_dir), ("right", right_dir)):
        if not os.path.isdir(d):
            continue
        for fname in sorted(os.listdir(d)):
            if not fname.endswith(".png"):
                continue
            seq = int(os.path.splitext(fname)[0])
            groups.setdefault(seq, []).append({
                "camera": side,
                "image_path": os.path.relpath(os.path.join(d, fname), REPO),
            })
    return groups


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--positives", default=os.path.join(HERE, "data", "ground_v1_positives"))
    ap.add_argument("--negatives", required=True,
                    help="negatives capture dir, e.g. logs/rig_captures/negatives_<runstamp>")
    ap.add_argument("--out", default=os.path.join(HERE, "data", "ground_dataset_v1"))
    ap.add_argument("--val-frac", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    pos_groups = load_positives(args.positives)
    neg_groups = load_negatives(args.negatives)

    print(f"[build] positives: {len(pos_groups)} pose-sequences "
          f"({sum(len(v) for v in pos_groups.values())} images)")
    print(f"[build] negatives: {len(neg_groups)} pose-sequences "
          f"({sum(len(v) for v in neg_groups.values())} images)")

    img_tr = os.path.join(args.out, "images", "train")
    img_va = os.path.join(args.out, "images", "val")
    lbl_tr = os.path.join(args.out, "labels", "train")
    lbl_va = os.path.join(args.out, "labels", "val")
    for d in (img_tr, img_va, lbl_tr, lbl_va):
        os.makedirs(d, exist_ok=True)

    rng = np.random.RandomState(args.seed)
    counts = {"train": {"pos": 0, "neg": 0}, "val": {"pos": 0, "neg": 0}}
    split_meta_rows = []

    def write_sample(split, img_dst_dir, lbl_dst_dir, src_image_path, label_lines,
                      stem, extra):
        shutil.copy2(os.path.join(REPO, src_image_path), os.path.join(img_dst_dir, stem + ".png"))
        with open(os.path.join(lbl_dst_dir, stem + ".txt"), "w") as fh:
            for line in label_lines:
                fh.write(line + "\n")
        row = {"split": split, "stem": stem, "positive": 1 if label_lines else 0}
        row.update(extra)
        split_meta_rows.append(row)

    # --- positives: one Bernoulli draw per SEQ, both cameras follow it ---
    for seq in sorted(pos_groups.keys()):
        is_val = rng.random_sample() < args.val_frac
        split = "val" if is_val else "train"
        img_dir = img_va if is_val else img_tr
        lbl_dir = lbl_va if is_val else lbl_tr
        for r in pos_groups[seq]:
            with open(os.path.join(REPO, r["label_path"])) as fh:
                lines = [ln for ln in fh.read().splitlines() if ln.strip()]
            stem = f"pos_seq{seq:05d}_{r['camera']}"
            write_sample(split, img_dir, lbl_dir, r["image_path"], lines, stem, {
                "seq": seq, "camera": r["camera"], "range_m": r["range_m"],
                "off_boresight_deg": r["off_boresight_deg"], "dash_direction": r["dash_direction"],
                "source": "positive",
            })
            counts[split]["pos"] += 1

    # --- negatives: one Bernoulli draw per SEQ (both cameras follow it) ---
    for seq in sorted(neg_groups.keys()):
        is_val = rng.random_sample() < args.val_frac
        split = "val" if is_val else "train"
        img_dir = img_va if is_val else img_tr
        lbl_dir = lbl_va if is_val else lbl_tr
        for r in neg_groups[seq]:
            stem = f"neg_seq{seq:05d}_{r['camera']}"
            write_sample(split, img_dir, lbl_dir, r["image_path"], [], stem, {
                "seq": seq, "camera": r["camera"], "range_m": "", "off_boresight_deg": "",
                "dash_direction": "", "source": "negative",
            })
            counts[split]["neg"] += 1

    split_meta_path = os.path.join(args.out, "split_meta.csv")
    with open(split_meta_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["split", "stem", "positive", "seq", "camera",
                                            "range_m", "off_boresight_deg", "dash_direction",
                                            "source"])
        w.writeheader()
        w.writerows(split_meta_rows)

    data_yaml = os.path.join(args.out, "data.yaml")
    with open(data_yaml, "w") as fh:
        fh.write(
            "# T17 ground single-class drone detector (ADR-0047: acquisition-only,\n"
            "# no interceptor class, no type discrimination).\n"
            "# Positives: gt-projected boxes from logs/rig_captures/full_sweep_...\n"
            "# (label_rig_captures.py). Negatives: empty-wedge captures with the rig\n"
            "# yawed away from the parked interceptor (capture_rig_negatives.py).\n"
            "# Split BY POSE SEQUENCE (see build_ground_dataset.py) -- L/R of the same\n"
            "# seq always share a split, avoiding stereo-pair leakage.\n"
            f"path: {os.path.abspath(args.out)}\n"
            "train: images/train\n"
            "val: images/val\n"
            "nc: 1\n"
            "names: ['drone']\n"
        )

    n_tr = counts["train"]["pos"] + counts["train"]["neg"]
    n_va = counts["val"]["pos"] + counts["val"]["neg"]
    print(f"[build] train: pos={counts['train']['pos']} neg={counts['train']['neg']} (total {n_tr})")
    print(f"[build] val:   pos={counts['val']['pos']} neg={counts['val']['neg']} (total {n_va})")
    print(f"[build] dataset: {data_yaml}")
    print(f"[build] split_meta: {split_meta_path}")

    readme_path = os.path.join(args.out, "README.md")
    with open(readme_path, "w") as fh:
        fh.write(f"""# ground_dataset_v1 (T17)

Single-class "drone" detector dataset for the ground stereo rig (ADR-0047:
acquisition-only, no interceptor class, no type discrimination).

## Provenance

- **Positives** ({counts['train']['pos'] + counts['val']['pos']} images): projected from
  `logs/rig_captures/full_sweep_20260709T015530Z` (T16's 56 L/R pose-sequence
  sweep) via `scripts/seeker/label_rig_captures.py`. Ground truth (`gt_target_*`
  world position) is used ONLY to manufacture the YOLO box label offline --
  the same honesty boundary as `render_sim_dataset.py` (CLAUDE.md's `gt_*`
  scoring-only rule). The trained detector reads pixels only at inference.
- **Negatives** ({counts['train']['neg'] + counts['val']['neg']} images): empty-wedge
  captures via `capture_rig_negatives.py`, with the rig yawed 90 deg away
  from the target corridor. **Finding, not swept under the rug**: the world
  has a flat, textureless ground plane and a uniform sky background, so all
  30 frames per camera are BYTE-IDENTICAL (confirmed by hash) -- there is
  effectively 1 unique negative scene per camera, not 30 independent ones.
  A first negative-capture attempt (target teleported far away, rig NOT
  moved) accidentally captured the PARKED INTERCEPTOR (spawned near world
  origin by `make px4_sitl gz_x500_mono_cam`, independent of the world SDF)
  sitting dead-center in the wedge -- discarded per ADR-0047 (interceptor
  must not be in the training set at all, positive or negative).

## Target extent / bbox derivation

`fpv_target_markerless`'s real silhouette was measured by hand from
`models/fpv_target_markerless/model.sdf`: prop-disc tips are the outermost
points (radial center 0.353553 m + prop radius 0.095 m = 0.4486 m from
centerline, both local Y and Z) -> 0.897 m span, independently confirming
`render_sim_dataset.py`'s established `--extent-m 0.9` default (not a new
number). A 15% multiplicative padding is applied for AA/prop-blur fuzz at
these very small (~10-35 px) sizes. Verified against 6 hand-drawn sanity
crops (`scripts/seeker/data/ground_v1_positives/sanity/`) -- 5/6 tight
clean fits, 1/6 correctly shows a genuinely edge-of-frame-clipped box (far
extreme of the sweep, ~10.4 deg off-boresight, just past the rig's 10 deg
half-angle).

## Split

By POSE SEQUENCE (not per-image): one random draw per `seq`, both cameras
(left/right) of that seq follow the same draw. This is deliberately
DIFFERENT from `merge_dataset_v2.py`'s per-image split -- the ground rig
always captures a stereo PAIR per pose (near-duplicate, ~2 m apart), so a
per-image split would leak geometry between train and val. See
`split_meta.csv` for the per-sample split assignment.

## Honest range-band note

This sweep holds target world-X fixed at 6.5 m (160 m downrange of the rig
at all times) and only sweeps world-Y (lateral) -- so the true camera-to-
target RANGE only varies ~160.1-162.5 m (about 1.6%), not a wide
acquisition-range sweep. See the T17 eval report for the near/far band
definition and why it is NOT a true long-range detection-floor test in this
dataset.
""")
    print(f"[build] README: {readme_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
