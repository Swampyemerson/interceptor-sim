#!/usr/bin/env python3
"""Merge the LEVEL quad dataset (quad_dataset, yaw-only) with the BANKED
augmentation (quad_dataset_banked, roll+pitch+yaw) into quad_dataset_v2.

WHY: `drone_finetuned_quad` (level+yaw only) detected `fpv_quad_enemy` at 23 m
but lost lock during the terminal dash once the target banks (up to 55 deg)
and pitches nose-down (up to 25 deg) via `--orient-to-velocity` -- aspects the
level-only training set never showed. This script unions both GT-auto-labeled
render sets (each already train/val split by render_sim_dataset[_banked].py)
into one train/val structure for the v2 retrain, PRESERVING each source's own
split assignment (task instruction: "keep the val split -- put the banked val
frames in val") rather than re-shuffling.

HONESTY: pure filesystem/labeling merge -- does not touch gz-transport or
gt_*; the honesty boundary (gt used ONLY to auto-label offline) was already
enforced by the two capture scripts. No orientation or ground truth is ever
read at inference; the merged dataset just teaches the detector what pixels
look like at more aspects.

Files are COPIED (not symlinked) with a source-tag prefix (`lvl_` / `bnk_`) so
provenance stays visible in the merged filenames and any accidental stem
collision across sources cannot silently overwrite a frame.

RUN:
    .venv-seeker/bin/python scripts/seeker/merge_quad_v2.py \\
      --level scripts/seeker/data/quad_dataset \\
      --banked scripts/seeker/data/quad_dataset_banked \\
      --out scripts/seeker/data/quad_dataset_v2
"""
from __future__ import annotations

import argparse
import os
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))

IMG_EXTS = (".png", ".jpg", ".jpeg")


def _copy_split(src_dir: str, dst_dir: str, split: str, tag: str) -> int:
    """Copy every image+label pair from src_dir/{images,labels}/{split} into
    dst_dir/{images,labels}/{split}, prefixed with `tag` to guarantee
    cross-source uniqueness. Returns the count copied. A missing label for an
    image is an ERROR (skipped + reported), never silently dropped/assumed
    negative -- these are all-positive render grids, not flight capture."""
    img_src = os.path.join(src_dir, "images", split)
    lbl_src = os.path.join(src_dir, "labels", split)
    img_dst = os.path.join(dst_dir, "images", split)
    lbl_dst = os.path.join(dst_dir, "labels", split)
    os.makedirs(img_dst, exist_ok=True)
    os.makedirs(lbl_dst, exist_ok=True)

    if not os.path.isdir(img_src):
        print(f"[merge] WARNING: missing {img_src}, skipping")
        return 0

    n = 0
    n_missing_label = 0
    for fn in sorted(os.listdir(img_src)):
        stem, ext = os.path.splitext(fn)
        if ext.lower() not in IMG_EXTS:
            continue
        lbl_fn = stem + ".txt"
        lbl_path = os.path.join(lbl_src, lbl_fn)
        if not os.path.isfile(lbl_path):
            n_missing_label += 1
            print(f"[merge] ERROR: {lbl_path} missing for image {fn}, skipping")
            continue
        new_stem = f"{tag}{stem}"
        shutil.copy2(os.path.join(img_src, fn), os.path.join(img_dst, new_stem + ext))
        shutil.copy2(lbl_path, os.path.join(lbl_dst, new_stem + ".txt"))
        n += 1
    if n_missing_label:
        print(f"[merge] {src_dir}/{split}: {n_missing_label} images had NO label (skipped)")
    return n


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--level", default=os.path.join(HERE, "data", "quad_dataset"))
    ap.add_argument("--banked", default=os.path.join(HERE, "data", "quad_dataset_banked"))
    ap.add_argument("--out", default=os.path.join(HERE, "data", "quad_dataset_v2"))
    args = ap.parse_args()

    if os.path.exists(args.out):
        print(f"[merge] FAILED: {args.out} already exists -- remove it first "
              f"(refusing to silently mix stale + fresh content)")
        return 2

    counts = {}
    for split in ("train", "val"):
        n_lvl = _copy_split(args.level, args.out, split, "lvl_")
        n_bnk = _copy_split(args.banked, args.out, split, "bnk_")
        counts[split] = (n_lvl, n_bnk)

    data_yaml = os.path.join(args.out, "data.yaml")
    with open(data_yaml, "w") as fh:
        fh.write(
            "# quad_dataset_v2: LEVEL (quad_dataset, yaw-only) UNION BANKED\n"
            "# (quad_dataset_banked, roll+pitch+yaw) fpv_quad_enemy renders.\n"
            "# Auto-labeled from ground truth (SCORING-only boundary, ADR-0033 B);\n"
            "# ground truth position (not orientation) + fixed body extent -> the\n"
            "# box is orientation-invariant, so banked/pitched frames auto-label\n"
            "# exactly as correctly as level ones. No orientation reaches inference.\n"
            f"path: {os.path.abspath(args.out)}\n"
            "train: images/train\n"
            "val: images/val\n"
            "nc: 1\n"
            "names: ['drone']\n"
        )

    n_tr_lvl, n_tr_bnk = counts["train"]
    n_va_lvl, n_va_bnk = counts["val"]
    n_tr, n_va = n_tr_lvl + n_tr_bnk, n_va_lvl + n_va_bnk
    print(f"[merge] train: {n_tr} ({n_tr_lvl} level + {n_tr_bnk} banked)")
    print(f"[merge] val:   {n_va} ({n_va_lvl} level + {n_va_bnk} banked)")
    print(f"[merge] total: {n_tr + n_va}")
    print(f"[merge] dataset: {data_yaml}")
    return 0 if (n_tr + n_va) > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
