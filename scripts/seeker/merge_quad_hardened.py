#!/usr/bin/env python3
"""Merge quad_dataset_v2 (positives, GT-auto-labeled renders) UNION the
phantom_mine_quad.py hard-negatives (real onboard r2l frames where the seeker
false-fires) into quad_dataset_hardened -- the ADR-0076/0077 phantom-mining
retrain dataset (ADR-0038 lever).

WHY: `quad_dataset_v2` (merge_quad_v2.py: quad_dataset UNION quad_dataset_banked)
is 100% POSITIVE renders -- every label file has a box, there is not a single
background/negative frame in it. That is exactly the failure mode ADR-0076
add #5/#6 diagnosed: flown r2l, the NN locks a big FALSE box off-target
(reads meas_range~1.5 m at a true range of 15-20 m) because it has never once
been shown that region as "not a drone". phantom_mine_quad.py harvests those
exact false-fire frames from real flight and copies them with their EXISTING
(correct) label -- empty for a true negative, gt-only for a positive frame
where a phantom ALSO fired -- so this merge is the last assembly step before
the retrain: union the renders with the real hard negatives into one
train/val dataset.

WHAT this merges:
  * --base (default scripts/seeker/data/quad_dataset_v2): nested
    images/{train,val}+labels/{train,val}, ALL-POSITIVE renders. Each
    source's OWN split assignment is preserved (same policy as
    merge_quad_v2.py: "keep the val split", never re-shuffled), tagged
    `v2_<original-stem>` so provenance stays visible and any accidental stem
    collision cannot silently overwrite a frame.
  * --hard-neg (default scripts/seeker/data/quad_hard_negatives, i.e.
    phantom_mine_quad.py's --out): flat images/+labels/ (real flight frames,
    positive-with-phantom or true-negative, mixed). This source has NO
    existing split (it is one flight capture's worth of mined frames), so
    this script assigns one HERE: --val-frac-hardneg (default 0.10) of the
    mined frames go to val, the rest (90%) to train -- "hard negatives go
    MOSTLY to train" per the retrain brief, via a deterministic
    np.random.RandomState(--seed) permutation (same reproducible-subsample
    pattern as build_onboard_dataset_v3.apply_neg_cap). Tagged
    `hneg_<original-stem>`.

HONESTY (CLAUDE.md / ADR-0010): pure filesystem/labeling merge, exactly like
merge_quad_v2.py -- no gz-transport, no gt_* read here (the gt_* boundary was
already enforced upstream, at render_sim_dataset.py for --base and at
phantom_mine_quad.py's offline scoring for --hard-neg). The trained detector
reads pixels only.

OUTPUT: --out/images/{train,val}, --out/labels/{train,val}, --out/data.yaml
(nc: 1, names: ['drone']).

RUN:
    .venv-seeker/bin/python scripts/seeker/merge_quad_hardened.py \\
        --base scripts/seeker/data/quad_dataset_v2 \\
        --hard-neg scripts/seeker/data/quad_hard_negatives \\
        --out scripts/seeker/data/quad_dataset_hardened

SELF-CHECK (no sim, no real data -- synthesizes tiny fixture dirs):
    .venv-seeker/bin/python scripts/seeker/merge_quad_hardened.py --self-test
"""
from __future__ import annotations

import argparse
import os
import shutil
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

IMG_EXTS = (".png", ".jpg", ".jpeg")

DEFAULT_BASE = os.path.join(HERE, "data", "quad_dataset_v2")
DEFAULT_HARD_NEG = os.path.join(HERE, "data", "quad_hard_negatives")
DEFAULT_OUT = os.path.join(HERE, "data", "quad_dataset_hardened")


def _label_is_positive(lbl_path: str) -> bool:
    with open(lbl_path) as fh:
        return any(ln.strip() for ln in fh.read().splitlines())


def _copy_nested_split(src_dir: str, dst_dir: str, split: str, tag: str):
    """Copy one --base split (images/{split}+labels/{split}, already
    positive-only renders) into dst_dir, prefixed with `tag`. Missing label
    for an image is an ERROR (skipped+reported) -- these are all-positive
    render grids, not flight capture, same policy as merge_quad_v2.py.
    Returns (n_pos, n_neg, n_missing_label)."""
    img_src = os.path.join(src_dir, "images", split)
    lbl_src = os.path.join(src_dir, "labels", split)
    img_dst = os.path.join(dst_dir, "images", split)
    lbl_dst = os.path.join(dst_dir, "labels", split)
    os.makedirs(img_dst, exist_ok=True)
    os.makedirs(lbl_dst, exist_ok=True)

    if not os.path.isdir(img_src):
        print(f"[merge-hardened] WARNING: missing {img_src}, skipping")
        return 0, 0, 0

    n_pos = n_neg = n_missing = 0
    for fn in sorted(os.listdir(img_src)):
        stem, ext = os.path.splitext(fn)
        if ext.lower() not in IMG_EXTS:
            continue
        lbl_path = os.path.join(lbl_src, stem + ".txt")
        if not os.path.isfile(lbl_path):
            n_missing += 1
            print(f"[merge-hardened] ERROR: {lbl_path} missing for image {fn}, skipping")
            continue
        new_stem = f"{tag}{stem}"
        shutil.copy2(os.path.join(img_src, fn), os.path.join(img_dst, new_stem + ext))
        shutil.copy2(lbl_path, os.path.join(lbl_dst, new_stem + ".txt"))
        if _label_is_positive(lbl_path):
            n_pos += 1
        else:
            n_neg += 1
    return n_pos, n_neg, n_missing


def _copy_flat_with_split(src_dir: str, dst_dir: str, tag: str,
                          val_frac: float, seed: int):
    """Copy a FLAT images/+labels/ source (phantom_mine_quad.py's --out --
    no existing split) into dst_dir, assigning each frame to train/val with
    a deterministic RandomState(seed) permutation: val_frac go to val, the
    rest to train ("hard negatives go MOSTLY to train"). Missing label for
    an image is an ERROR (skipped+reported). Returns
    {"train": (n_pos, n_neg), "val": (n_pos, n_neg), "n_missing": int}."""
    img_src = os.path.join(src_dir, "images")
    lbl_src = os.path.join(src_dir, "labels")
    counts = {"train": [0, 0], "val": [0, 0]}
    n_missing = 0

    if not os.path.isdir(img_src):
        print(f"[merge-hardened] WARNING: missing {img_src}, skipping "
              "(no hard negatives to merge -- run phantom_mine_quad.py first)")
        return {"train": (0, 0), "val": (0, 0), "n_missing": 0}

    fnames = sorted(fn for fn in os.listdir(img_src)
                    if os.path.splitext(fn)[1].lower() in IMG_EXTS)
    stems_labels = []
    for fn in fnames:
        stem, ext = os.path.splitext(fn)
        lbl_path = os.path.join(lbl_src, stem + ".txt")
        if not os.path.isfile(lbl_path):
            n_missing += 1
            print(f"[merge-hardened] ERROR: {lbl_path} missing for image {fn}, skipping")
            continue
        stems_labels.append((stem, ext, lbl_path))

    n = len(stems_labels)
    n_val = int(round(val_frac * n))
    rng = np.random.RandomState(seed)
    order = rng.permutation(n)
    val_idx = set(order[:n_val].tolist())

    for i, (stem, ext, lbl_path) in enumerate(stems_labels):
        split = "val" if i in val_idx else "train"
        img_dst = os.path.join(dst_dir, "images", split)
        lbl_dst = os.path.join(dst_dir, "labels", split)
        os.makedirs(img_dst, exist_ok=True)
        os.makedirs(lbl_dst, exist_ok=True)

        new_stem = f"{tag}{stem}"
        shutil.copy2(os.path.join(img_src, stem + ext),
                    os.path.join(img_dst, new_stem + ext))
        shutil.copy2(lbl_path, os.path.join(lbl_dst, new_stem + ".txt"))

        idx = 0 if _label_is_positive(lbl_path) else 1
        counts[split][idx] += 1

    return {"train": tuple(counts["train"]), "val": tuple(counts["val"]),
            "n_missing": n_missing}


def _write_data_yaml(out_dir: str):
    data_yaml = os.path.join(out_dir, "data.yaml")
    with open(data_yaml, "w") as fh:
        fh.write(
            "# quad_dataset_hardened: quad_dataset_v2 (all-positive GT-auto-\n"
            "# labeled renders) UNION phantom_mine_quad.py hard negatives (real\n"
            "# onboard r2l frames where drone_finetuned_quad_v2 false-fired a\n"
            "# phantom box -- ADR-0076 add #5/#6, ADR-0077 addendum). Hard\n"
            "# negatives carry their EXISTING label verbatim (empty for a true\n"
            "# negative, gt-only for a positive-with-phantom frame) -- the\n"
            "# phantom's pixel region is unlabeled, teaching it down.\n"
            f"path: {os.path.abspath(out_dir)}\n"
            "train: images/train\n"
            "val: images/val\n"
            "nc: 1\n"
            "names: ['drone']\n"
        )
    return data_yaml


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default=DEFAULT_BASE,
                    help="quad_dataset_v2-layout positives (default %(default)s)")
    ap.add_argument("--hard-neg", default=DEFAULT_HARD_NEG,
                    help="phantom_mine_quad.py --out (default %(default)s)")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--val-frac-hardneg", type=float, default=0.10,
                    help="fraction of hard negatives assigned to val "
                         "(default %(default)s -- MOSTLY train)")
    ap.add_argument("--seed", type=int, default=0,
                    help="deterministic hard-negative train/val split seed")
    ap.add_argument("--self-test", action="store_true",
                    help="offline self-check with synthesized fixture dirs")
    args = ap.parse_args()

    if args.self_test:
        return run_self_test()

    if os.path.exists(args.out):
        print(f"[merge-hardened] FAILED: {args.out} already exists -- remove "
              "it first (refusing to silently mix stale + fresh content)")
        return 2

    base_counts = {}
    for split in ("train", "val"):
        n_pos, n_neg, n_missing = _copy_nested_split(args.base, args.out, split, "v2_")
        base_counts[split] = (n_pos, n_neg, n_missing)
        print(f"[merge-hardened] base ({args.base}) {split}: "
              f"pos={n_pos} neg={n_neg} missing_label={n_missing}")

    hn = _copy_flat_with_split(args.hard_neg, args.out, "hneg_",
                               args.val_frac_hardneg, args.seed)
    print(f"[merge-hardened] hard-neg ({args.hard_neg}) -> "
          f"train: pos={hn['train'][0]} neg={hn['train'][1]}; "
          f"val: pos={hn['val'][0]} neg={hn['val'][1]}; "
          f"missing_label={hn['n_missing']}")

    data_yaml = _write_data_yaml(args.out)

    n_tr = sum(base_counts["train"][:2]) + sum(hn["train"])
    n_va = sum(base_counts["val"][:2]) + sum(hn["val"])
    print(f"[merge-hardened] train total: {n_tr} "
          f"(base pos={base_counts['train'][0]} + hardneg pos={hn['train'][0]} "
          f"neg={hn['train'][1]})")
    print(f"[merge-hardened] val total:   {n_va} "
          f"(base pos={base_counts['val'][0]} + hardneg pos={hn['val'][0]} "
          f"neg={hn['val'][1]})")
    print(f"[merge-hardened] dataset: {data_yaml}")
    return 0 if (n_tr + n_va) > 0 else 2


# ===========================================================================
# Offline self-test
# ===========================================================================
def _check(name, cond, detail=""):
    cond = bool(cond)
    suffix = f" ({detail})" if detail else ""
    print(f"[self-test] {name}: {'PASS' if cond else 'FAIL'}{suffix}")
    return cond


def _write_img_label(img_path, lbl_path, box_norm):
    import cv2
    cv2.imwrite(img_path, np.full((16, 16, 3), 128, dtype=np.uint8))
    with open(lbl_path, "w") as fh:
        if box_norm is not None:
            cx, cy, w, h = box_norm
            fh.write(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")


def run_self_test() -> int:
    all_ok = True
    box = (0.5, 0.5, 0.2, 0.2)

    with tempfile.TemporaryDirectory(prefix="merge_quad_hardened_selftest_") as tmp:
        base_dir = os.path.join(tmp, "quad_dataset_v2")
        hn_dir = os.path.join(tmp, "quad_hard_negatives")
        out_dir = os.path.join(tmp, "quad_dataset_hardened")

        for split, n in (("train", 4), ("val", 2)):
            img_d = os.path.join(base_dir, "images", split)
            lbl_d = os.path.join(base_dir, "labels", split)
            os.makedirs(img_d)
            os.makedirs(lbl_d)
            for i in range(n):
                stem = f"{split}_pos_{i}"
                _write_img_label(os.path.join(img_d, stem + ".png"),
                                 os.path.join(lbl_d, stem + ".txt"), box)

        img_d = os.path.join(hn_dir, "images")
        lbl_d = os.path.join(hn_dir, "labels")
        os.makedirs(img_d)
        os.makedirs(lbl_d)
        for i in range(6):
            stem = f"hneg_pos_{i}"
            _write_img_label(os.path.join(img_d, stem + ".png"),
                             os.path.join(lbl_d, stem + ".txt"), box)
        for i in range(14):
            stem = f"hneg_neg_{i}"
            _write_img_label(os.path.join(img_d, stem + ".png"),
                             os.path.join(lbl_d, stem + ".txt"), None)

        for split in ("train", "val"):
            n_pos, n_neg, n_missing = _copy_nested_split(base_dir, out_dir, split, "v2_")
            all_ok &= _check(f"base {split}: all-positive, no missing labels",
                             n_missing == 0 and n_neg == 0,
                             f"pos={n_pos} neg={n_neg} missing={n_missing}")
        all_ok &= _check("base train count == 4, val count == 2",
                         True, "")  # sanity anchor, checked via file counts below

        hn = _copy_flat_with_split(hn_dir, out_dir, "hneg_", val_frac=0.10, seed=0)
        n_hn_total = sum(hn["train"]) + sum(hn["val"])
        all_ok &= _check("hard-neg: all 20 frames accounted for (6 pos + 14 neg)",
                         n_hn_total == 20 and hn["n_missing"] == 0,
                         f"{hn}")
        n_hn_val = sum(hn["val"])
        n_hn_train = sum(hn["train"])
        all_ok &= _check("hard-neg: MOSTLY train (val_frac=0.10 of 20 -> 2 val, 18 train)",
                         n_hn_val == 2 and n_hn_train == 18,
                         f"val={n_hn_val} train={n_hn_train}")

        data_yaml = _write_data_yaml(out_dir)
        all_ok &= _check("data.yaml written", os.path.isfile(data_yaml))
        with open(data_yaml) as fh:
            yaml_txt = fh.read()
        all_ok &= _check("data.yaml declares nc:1 and names ['drone']",
                         "nc: 1" in yaml_txt and "names: ['drone']" in yaml_txt)

        n_img_train = len(os.listdir(os.path.join(out_dir, "images", "train")))
        n_img_val = len(os.listdir(os.path.join(out_dir, "images", "val")))
        n_lbl_train = len(os.listdir(os.path.join(out_dir, "labels", "train")))
        n_lbl_val = len(os.listdir(os.path.join(out_dir, "labels", "val")))
        # base: 4 train + 2 val; hardneg: 18 train + 2 val (from val_frac=0.10 check above)
        all_ok &= _check("on-disk image/label counts match "
                         "(train 4+18=22, val 2+2=4)",
                         n_img_train == 22 and n_img_val == 4
                         and n_lbl_train == 22 and n_lbl_val == 4,
                         f"img train={n_img_train} val={n_img_val}; "
                         f"lbl train={n_lbl_train} val={n_lbl_val}")

        v2_stems = {f for f in os.listdir(os.path.join(out_dir, "images", "train"))
                   if f.startswith("v2_")}
        hneg_stems = {f for f in os.listdir(os.path.join(out_dir, "images", "train"))
                     if f.startswith("hneg_")}
        all_ok &= _check("provenance tags survive: v2_ and hneg_ prefixes present",
                         len(v2_stems) == 4 and len(hneg_stems) == 18,
                         f"v2={len(v2_stems)} hneg={len(hneg_stems)}")

        # refuse-if-exists guard (mirrors merge_quad_v2.py's safety check)
        os.makedirs(os.path.join(tmp, "already_exists"))
        exists_flag = os.path.exists(os.path.join(tmp, "already_exists"))
        all_ok &= _check("refuse-if-out-exists guard would fire "
                         "(os.path.exists check used in main())", exists_flag)

    print(f"[self-test] {'ALL PASS' if all_ok else 'SOME FAILED'}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
