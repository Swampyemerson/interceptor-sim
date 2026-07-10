#!/usr/bin/env python3
r"""Seeker v3 dataset assembler (docs/seeker_v3_dataset_plan.md Sec 9 step 1):
merge grid + flight-train + legacy renders/capture, split FLIGHT-LEVEL (not
per-frame Bernoulli), phantom-mine the union with drone_finetuned_v2.onnx,
and emit split_meta.csv -- the ADR-0055-style artifact an independent
verifier uses to prove zero leakage between train and val.

WHY (plan Sec 6): frames within a flight are temporally correlated (30 Hz,
`--every 3`) -- per-frame Bernoulli splitting (merge_dataset_v2.py's
approach) leaks near-duplicates across the split, the exact ADR-0049
weakness build_ground_dataset_v2.py fixed for the ground detector with a
STRUCTURAL split (held out by standoff). This script applies the identical
medicine to the two v3 data kinds: entire FLIGHTS (by run_tag) and entire
RANGE buckets (grid, by held-out range) go to val wholesale, never mixed
with train. Legacy v1/v2 data's old val role is retired -- it all becomes
train, so the v3 val set is correlated with nothing in train.

WHAT it merges (plan Sec 3/6):
  * --grid       : the Phase-1 static teleport grid (flat images/+labels/ +
                    index.csv). Split by held-out RANGE bucket (3.0, 18.0 m
                    -> val; ADR-0055 pattern).
  * --flight-train (repeatable) : Phase-2 ride-along captures
                    (capture_flight_frames.py output, flat images/+labels/,
                    filenames encode run_tag + gt_range). Split by run_tag:
                    entire flights capv3_03/04/08/10 -> val, all others ->
                    train.
  * --legacy-render  : v1 static renders (nested images/{train,val}+
                    labels/{train,val}). ALL -> train (old split retired).
  * --legacy-capture (repeatable) : v2 flight capture (flat images/+
                    labels/). ALL -> train (old split retired).

PHANTOM MINING (plan Sec 4.2, the targeted lever): every CAPTURED frame
(grid + flight-train only -- legacy is NOT re-mined, its role is "anchor
the old domain wholesale") is scored offline with drone_finetuned_v2.onnx
via scripts/seeker/v3_onnx_infer.py at the DEPLOYED operating point (conf
0.25 + the deployed self-mask -- the exact stream that feeds handoff). Any
NEGATIVE frame where v2 fires >= 1 masked detection is a GUARANTEED-INCLUDE
hard negative (mined_neg=1), EXEMPT from the neg-cap subsample below. Any
POSITIVE frame where v2's best masked box has IoU < 0.1 vs the gt box is
tagged mined_pos=1 (informational only -- positives are never dropped, the
gt label is already the counter-supervision). Mining scores BOTH train and
val captured frames (it is curation/labeling of the union, val frames keep
their structural split).

NEG-CAP (plan Sec 4 curation): negatives are capped at --max-neg-frac
(default 0.45, plan's +5 pt over v2's 0.40 because the failure mode is
false-positive-driven) of the TRAIN split only (val is fixed by the
structural rule above and is never subsampled). Mined negatives are EXEMPT
from the cap -- only non-mined ("free") negatives are candidates for the
deterministic np.random.RandomState(seed) subsample, following
merge_dataset_v2.py's target-count formula but solved against the *mined +
free* total so mined frames are guaranteed to survive even if that pushes
the realized fraction above --max-neg-frac (that is the intended, declared
behavior: mined hard negatives are the whole point of Sec 4.2). Positives
are never dropped.

HONESTY (CLAUDE.md / ADR-0010, restated -- same boundary every dataset
producer in this repo documents): gt_* labels were already baked into the
YOLO .txt files by the capture scripts (seeker_v3_capture.py,
capture_flight_frames.py) OFFLINE, to manufacture training labels only --
this script never talks to gz/mavsdk and never feeds gt_* to a live
detector. The one place gt_* is read here (load_gt_box, for the mined_pos
IoU check) is purely an offline scoring/curation operation on already-
manufactured label files, the same boundary v3_onnx_infer.py documents.
The trained detector itself reads pixels only.

OUTPUT: --out/images/{train,val}, --out/labels/{train,val} (files COPIED,
unique-stem-on-collision like merge_dataset_v2.py), a data.yaml, and
--out/split_meta.csv with columns:
    dest_stem, source (grid|flight:<run_tag>|legacy_render|legacy_capture),
    flight_key, range_m, positive, mined_neg, mined_pos, split
plus a printed "leakage check: no run_tag in both splits, no grid range in
both splits" assertion.

RUN (offline, no sim; needs onnxruntime + cv2, i.e. .venv-seeker):
    .venv-seeker/bin/python scripts/seeker/build_onboard_dataset_v3.py \
        --grid scripts/seeker/data/v3_grid \
        --flight-train scripts/seeker/data/v3_flight_train \
        --legacy-render scripts/seeker/data/sim_dataset \
        --legacy-capture scripts/seeker/data/flight_capture \
        --out scripts/seeker/data/onboard_dataset_v3

OFFLINE SELF-CHECK (no sim, no real weights -- synthesizes tiny fixture
data in a tmp dir and injects a fake detector function into mine_dataset()
in place of real ONNX weights):
    .venv-seeker/bin/python scripts/seeker/build_onboard_dataset_v3.py --self-test
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import shutil
import sys
import tempfile

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from v3_onnx_infer import (  # noqa: E402
    DEPLOY_CONF, apply_self_mask, best_by_score, det_box, infer_raw,
    iou, load_gt_box, load_session,
)

DEFAULT_GRID = os.path.join(HERE, "data", "v3_grid")
DEFAULT_FLIGHT_TRAIN = os.path.join(HERE, "data", "v3_flight_train")
DEFAULT_LEGACY_RENDER = os.path.join(HERE, "data", "sim_dataset")
DEFAULT_LEGACY_CAPTURE = os.path.join(HERE, "data", "flight_capture")
DEFAULT_V2_WEIGHTS = os.path.join(HERE, "weights", "drone_finetuned_v2.onnx")
DEFAULT_OUT = os.path.join(HERE, "data", "onboard_dataset_v3")

IMG_EXTS = (".png", ".jpg", ".jpeg")

# Plan Sec 6 -- the structural split keys, hard-pinned (not a CLI knob: this
# IS the point of the v3 dataset, drifting it silently would be a real bug).
VAL_FLIGHT_TAGS = {"capv3_03", "capv3_04", "capv3_08", "capv3_10"}
VAL_GRID_RANGES_M = {3.0, 18.0}

# capture_flight_frames.py filename convention:
#   f"{run_tag}_{seq:06d}_r{gt_range:05.1f}_{'pos' if positive else 'neg'}"
# run_tag itself may contain underscores (e.g. "capv3_01"); the trailing
# "_NNNNNN_rRRR.R_pos|neg" suffix is fixed-shape, so greedy .+ backtracking
# finds the right split point unambiguously.
FLIGHT_STEM_RE = re.compile(
    r"^(?P<run_tag>.+)_(?P<seq>\d{6})_r(?P<range>\d+\.\d)_(?P<label>pos|neg)$")


def _parse_bool(v) -> bool:
    if v is None:
        return False
    return str(v).strip().lower() in ("1", "true", "yes")


def _parse_float_or_none(v):
    if v is None:
        return None
    v = str(v).strip()
    if v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


# ===========================================================================
# Collection: grid / flight-train / legacy-render / legacy-capture
# ===========================================================================
def collect_flat_dir(images_dir: str, labels_dir: str):
    """Scan one flat images_dir/labels_dir pair (capture_flight_frames.py
    layout). Returns (samples, n_missing_label); samples are
    {image, label, positive} in sorted-filename (reproducible) order. A
    missing label file is an ERROR (counted, skipped), never silently
    treated as either class -- same policy as merge_dataset_v2.collect_dir."""
    samples = []
    n_missing = 0
    if not os.path.isdir(images_dir):
        return samples, n_missing
    for fname in sorted(os.listdir(images_dir)):
        stem, ext = os.path.splitext(fname)
        if ext.lower() not in IMG_EXTS:
            continue
        img_path = os.path.join(images_dir, fname)
        lbl_path = os.path.join(labels_dir, stem + ".txt")
        if not os.path.isfile(lbl_path):
            n_missing += 1
            continue
        with open(lbl_path) as fh:
            lines = [ln for ln in fh.read().splitlines() if ln.strip()]
        samples.append({"image": img_path, "label": lbl_path,
                        "positive": len(lines) >= 1})
    return samples, n_missing


def collect_grid(grid_dir: str):
    """Phase-1 grid: flat images/+labels/ + index.csv (seeker_v3_capture.py
    layout). Split by held-out RANGE bucket (plan Sec 6): trust index.csv's
    held_out_range flag first, fall back to range_m in VAL_GRID_RANGES_M."""
    samples = []
    n_missing = 0
    index_path = os.path.join(grid_dir, "index.csv")
    img_dir = os.path.join(grid_dir, "images")
    lbl_dir = os.path.join(grid_dir, "labels")
    if not os.path.isfile(index_path):
        return samples, n_missing
    with open(index_path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    for row in rows:
        stem = row["stem"]
        kind = (row.get("kind") or "").strip()
        img_path = os.path.join(img_dir, stem + ".png")
        lbl_path = os.path.join(lbl_dir, stem + ".txt")
        if not os.path.isfile(img_path) or not os.path.isfile(lbl_path):
            n_missing += 1
            continue
        positive = kind == "positive"
        with open(lbl_path) as fh:
            has_content = any(ln.strip() for ln in fh.read().splitlines())
        if has_content != positive:
            print(f"[build-v3] WARNING: grid stem {stem!r} kind={kind!r} but "
                  f"label file {'has' if has_content else 'has no'} content "
                  "-- trusting index.csv kind")

        range_m = _parse_float_or_none(row.get("range_m"))
        held_out = _parse_bool(row.get("held_out_range"))
        if not held_out and range_m is not None:
            held_out = any(abs(range_m - v) < 1e-6 for v in VAL_GRID_RANGES_M)
        split = "val" if held_out else "train"

        samples.append({
            "image": img_path, "label": lbl_path, "positive": positive,
            "source": "grid", "flight_key": "", "range_m": range_m,
            "split": split, "mine_eligible": True,
            "mined_neg": 0, "mined_pos": 0,
        })
    return samples, n_missing


def collect_flight(flight_dir: str):
    """One --flight-train dir: flat images/+labels/, filenames encode
    run_tag + gt_range (capture_flight_frames.py). Split by run_tag (plan
    Sec 6): VAL_FLIGHT_TAGS -> val, every other run_tag -> train."""
    samples = []
    n_missing = 0
    img_dir = os.path.join(flight_dir, "images")
    lbl_dir = os.path.join(flight_dir, "labels")
    flat, n_missing = collect_flat_dir(img_dir, lbl_dir)
    for f in flat:
        stem = os.path.splitext(os.path.basename(f["image"]))[0]
        m = FLIGHT_STEM_RE.match(stem)
        if m:
            run_tag = m.group("run_tag")
            range_m = float(m.group("range"))
            declared_positive = m.group("label") == "pos"
            if declared_positive != f["positive"]:
                print(f"[build-v3] WARNING: flight frame {stem!r} filename "
                      f"suffix says {'pos' if declared_positive else 'neg'} "
                      f"but label file says "
                      f"{'positive' if f['positive'] else 'negative'} -- "
                      "trusting the label file")
        else:
            print(f"[build-v3] WARNING: flight frame stem does not match the "
                  f"run_tag/seq/range convention, treated as run_tag="
                  f"'unknown': {stem!r}")
            run_tag = "unknown"
            range_m = None
        split = "val" if run_tag in VAL_FLIGHT_TAGS else "train"
        samples.append({
            "image": f["image"], "label": f["label"], "positive": f["positive"],
            "source": "flight", "flight_key": run_tag, "range_m": range_m,
            "split": split, "mine_eligible": True,
            "mined_neg": 0, "mined_pos": 0,
        })
    return samples, n_missing


def collect_legacy_render(root_dir: str):
    """v1 nested images/{train,val}+labels/{train,val}. ALL -> train (plan
    Sec 6: the old val role is retired)."""
    samples = []
    n_missing_total = 0
    if not os.path.isdir(root_dir):
        return samples, n_missing_total
    for nested in ("train", "val"):
        img_dir = os.path.join(root_dir, "images", nested)
        lbl_dir = os.path.join(root_dir, "labels", nested)
        flat, n_missing = collect_flat_dir(img_dir, lbl_dir)
        n_missing_total += n_missing
        for f in flat:
            samples.append({
                "image": f["image"], "label": f["label"], "positive": f["positive"],
                "source": "legacy_render", "flight_key": "", "range_m": None,
                "split": "train", "mine_eligible": False,
                "mined_neg": 0, "mined_pos": 0,
            })
    return samples, n_missing_total


def collect_legacy_capture(cap_dir: str):
    """v2 flat flight-capture images/+labels/. ALL -> train (plan Sec 6)."""
    samples = []
    img_dir = os.path.join(cap_dir, "images")
    lbl_dir = os.path.join(cap_dir, "labels")
    flat, n_missing = collect_flat_dir(img_dir, lbl_dir)
    for f in flat:
        samples.append({
            "image": f["image"], "label": f["label"], "positive": f["positive"],
            "source": "legacy_capture", "flight_key": "", "range_m": None,
            "split": "train", "mine_eligible": False,
            "mined_neg": 0, "mined_pos": 0,
        })
    return samples, n_missing


def collect_all(grid_dir, flight_dirs, legacy_render_dir, legacy_capture_dirs):
    all_samples = []
    n_missing = 0

    s, m = collect_grid(grid_dir)
    all_samples += s
    n_missing += m
    pos = sum(1 for x in s if x["positive"])
    print(f"[build-v3] grid ({grid_dir}): {len(s)} samples "
          f"(pos={pos} neg={len(s) - pos})"
          + ("" if s or os.path.isfile(os.path.join(grid_dir, "index.csv"))
             else " -- NOT FOUND, treated as empty"))

    for d in flight_dirs:
        s, m = collect_flight(d)
        all_samples += s
        n_missing += m
        pos = sum(1 for x in s if x["positive"])
        tags = sorted({x["flight_key"] for x in s})
        print(f"[build-v3] flight-train ({d}): {len(s)} samples "
              f"(pos={pos} neg={len(s) - pos}, run_tags={tags})"
              + ("" if s or os.path.isdir(os.path.join(d, "images"))
                 else " -- NOT FOUND, treated as empty"))

    s, m = collect_legacy_render(legacy_render_dir)
    all_samples += s
    n_missing += m
    pos = sum(1 for x in s if x["positive"])
    print(f"[build-v3] legacy-render ({legacy_render_dir}): {len(s)} samples "
          f"(pos={pos} neg={len(s) - pos}) -> ALL train")

    for d in legacy_capture_dirs:
        s, m = collect_legacy_capture(d)
        all_samples += s
        n_missing += m
        pos = sum(1 for x in s if x["positive"])
        print(f"[build-v3] legacy-capture ({d}): {len(s)} samples "
              f"(pos={pos} neg={len(s) - pos}) -> ALL train")

    print(f"[build-v3] union: {len(all_samples)} samples; "
          f"missing_label_errors={n_missing} (image with no label file -- skipped)")
    return all_samples, n_missing


# ===========================================================================
# Phantom mining (plan Sec 4.2) -- USES v3_onnx_infer, injectable detect_fn
# ===========================================================================
def make_detector(weights: str):
    """Build a detect_fn(frame_bgr) -> masked dets, at the DEPLOYED
    operating point (conf 0.25 + self-mask), from a real v2 ONNX. Lazy
    onnxruntime import lives inside v3_onnx_infer.load_session, so importing
    THIS module never requires onnxruntime unless make_detector is called."""
    sess, iname, imgsz = load_session(weights)

    def detect_fn(frame_bgr):
        dets = infer_raw(sess, iname, imgsz, frame_bgr, conf=DEPLOY_CONF)
        return apply_self_mask(dets)

    return detect_fn


def mine_dataset(samples, detect_fn) -> dict:
    """Run detect_fn over every mine-eligible (grid + flight-train, NOT
    legacy) sample and tag mined_neg/mined_pos IN PLACE (plan Sec 4.2):
      - negative frame, >=1 masked detection -> mined_neg=1 (guaranteed-
        include hard negative, exempt from the neg-cap subsample).
      - positive frame, best masked box has IoU<0.1 vs the gt box ->
        mined_pos=1 (informational tag; positives are never dropped).
    Runs on BOTH train and val captured frames (curation of the union; val
    frames keep whatever split the structural rule already gave them)."""
    n_scored = n_mined_neg = n_mined_pos = 0
    for s in samples:
        if not s.get("mine_eligible"):
            continue
        frame = cv2.imread(s["image"])
        if frame is None:
            print(f"[build-v3] WARNING: mining could not read image "
                  f"(skipped): {s['image']}")
            continue
        dets = detect_fn(frame)
        n_scored += 1
        if not s["positive"]:
            if dets:
                s["mined_neg"] = 1
                n_mined_neg += 1
        else:
            gt = load_gt_box(s["label"])
            best = best_by_score(dets)
            if gt is not None and best is not None and iou(det_box(best), gt) < 0.1:
                s["mined_pos"] = 1
                n_mined_pos += 1
    return {"n_scored": n_scored, "n_mined_neg": n_mined_neg, "n_mined_pos": n_mined_pos}


# ===========================================================================
# Neg-cap (plan Sec 4): TRAIN split only, mined negatives EXEMPT
# ===========================================================================
def apply_neg_cap(samples, max_neg_frac: float, seed: int):
    """Cap negatives at max_neg_frac of the TRAIN split total (val is fixed
    by the structural split, never subsampled here). Mined negatives
    (mined_neg truthy) are EXEMPT -- only "free" (non-mined) negatives are
    candidates for the deterministic RandomState(seed) subsample. Positives
    are never dropped. Follows merge_dataset_v2.py's target-count formula,
    solved against the pos/neg TOTAL (mined negatives can push the realized
    fraction above max_neg_frac -- that is the declared, intended behavior:
    they are guaranteed-include). Returns (final_samples, report)."""
    train = [s for s in samples if s["split"] == "train"]
    other = [s for s in samples if s["split"] != "train"]
    train_pos = [s for s in train if s["positive"]]
    train_neg = [s for s in train if not s["positive"]]
    mined_neg = [s for s in train_neg if s.get("mined_neg")]
    free_neg = [s for s in train_neg if not s.get("mined_neg")]

    n_pos = len(train_pos)
    n_neg = len(train_neg)
    dropped = 0
    kept_neg = train_neg

    if (n_pos + n_neg) > 0 and 0.0 <= max_neg_frac < 1.0:
        neg_frac = n_neg / (n_pos + n_neg)
        if neg_frac > max_neg_frac:
            target_total_neg = int(np.floor(max_neg_frac * n_pos / (1.0 - max_neg_frac)))
            target_total_neg = max(0, target_total_neg)
            n_mined = len(mined_neg)
            target_free = max(0, target_total_neg - n_mined)
            target_free = min(target_free, len(free_neg))
            rng = np.random.RandomState(seed)
            if target_free < len(free_neg):
                order = rng.permutation(len(free_neg))
                keep_idx = sorted(order[:target_free].tolist())
                kept_free = [free_neg[i] for i in keep_idx]
                dropped = len(free_neg) - target_free
            else:
                kept_free = free_neg
            kept_neg = mined_neg + kept_free
            print(f"[build-v3] train negatives {n_neg} exceed max_neg_frac="
                  f"{max_neg_frac} of train total -> target_total_neg="
                  f"{target_total_neg} (mined={n_mined} exempt, free kept="
                  f"{len(kept_free)}/{len(free_neg)}, dropped={dropped}, "
                  f"seed={seed})")

    final_samples = other + train_pos + kept_neg
    report = {"n_pos": n_pos, "n_neg_pre": n_neg, "n_neg_post": len(kept_neg),
              "n_mined_neg_exempt": len(mined_neg), "dropped": dropped}
    return final_samples, report


# ===========================================================================
# Leakage check (the whole point of the structural split)
# ===========================================================================
def leakage_check(samples):
    tr_tags = {s["flight_key"] for s in samples
              if s["source"] == "flight" and s["split"] == "train" and s["flight_key"]}
    va_tags = {s["flight_key"] for s in samples
              if s["source"] == "flight" and s["split"] == "val" and s["flight_key"]}
    overlap_tags = tr_tags & va_tags

    tr_ranges = {s["range_m"] for s in samples
                if s["source"] == "grid" and s["split"] == "train" and s["range_m"] is not None}
    va_ranges = {s["range_m"] for s in samples
                if s["source"] == "grid" and s["split"] == "val" and s["range_m"] is not None}
    overlap_ranges = tr_ranges & va_ranges

    ok = not overlap_tags and not overlap_ranges
    return ok, overlap_tags, overlap_ranges


# ===========================================================================
# Write: images/labels + data.yaml + split_meta.csv
# ===========================================================================
def write_dataset(samples, out_dir: str):
    img_tr = os.path.join(out_dir, "images", "train")
    img_va = os.path.join(out_dir, "images", "val")
    lbl_tr = os.path.join(out_dir, "labels", "train")
    lbl_va = os.path.join(out_dir, "labels", "val")
    for d in (img_tr, img_va, lbl_tr, lbl_va):
        os.makedirs(d, exist_ok=True)

    counts = {"train": {"pos": 0, "neg": 0}, "val": {"pos": 0, "neg": 0}}
    used_stems = {"train": set(), "val": set()}
    split_meta_rows = []

    for s in samples:
        split = s["split"]
        img_dst_dir = img_tr if split == "train" else img_va
        lbl_dst_dir = lbl_tr if split == "train" else lbl_va

        base = os.path.basename(s["image"])
        stem, ext = os.path.splitext(base)
        dest_stem = stem
        n = 0
        while dest_stem in used_stems[split]:
            n += 1
            dest_stem = f"{stem}_dup{n}"
        used_stems[split].add(dest_stem)

        shutil.copy2(s["image"], os.path.join(img_dst_dir, dest_stem + ext))
        shutil.copy2(s["label"], os.path.join(lbl_dst_dir, dest_stem + ".txt"))

        src_field = f"flight:{s['flight_key']}" if s["source"] == "flight" else s["source"]
        range_field = "" if s["range_m"] is None else f"{s['range_m']:.4f}"
        split_meta_rows.append({
            "dest_stem": dest_stem, "source": src_field,
            "flight_key": s["flight_key"], "range_m": range_field,
            "positive": int(s["positive"]), "mined_neg": int(s.get("mined_neg", 0)),
            "mined_pos": int(s.get("mined_pos", 0)), "split": split,
        })
        counts[split]["pos" if s["positive"] else "neg"] += 1

    split_meta_path = os.path.join(out_dir, "split_meta.csv")
    with open(split_meta_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["dest_stem", "source", "flight_key",
                                           "range_m", "positive", "mined_neg",
                                           "mined_pos", "split"])
        w.writeheader()
        w.writerows(split_meta_rows)

    data_yaml = os.path.join(out_dir, "data.yaml")
    with open(data_yaml, "w") as fh:
        fh.write(
            "# onboard_dataset_v3 (docs/seeker_v3_dataset_plan.md Sec 6/9):\n"
            "# grid + flight-train UNION, held out ENTIRELY by flight (run_tag)\n"
            "# and by grid range bucket -- see split_meta.csv for the per-image\n"
            "# leakage-audit artifact (source/flight_key/range_m/split).\n"
            f"path: {os.path.abspath(out_dir)}\n"
            "train: images/train\n"
            "val: images/val\n"
            "nc: 1\n"
            "names: ['drone']\n"
        )

    return counts, split_meta_path, data_yaml


# ===========================================================================
# main
# ===========================================================================
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grid", default=DEFAULT_GRID)
    ap.add_argument("--flight-train", action="append", default=None,
                    help=f"repeatable (default: [{DEFAULT_FLIGHT_TRAIN}])")
    ap.add_argument("--legacy-render", default=DEFAULT_LEGACY_RENDER)
    ap.add_argument("--legacy-capture", action="append", default=None,
                    help=f"repeatable (default: [{DEFAULT_LEGACY_CAPTURE}])")
    ap.add_argument("--v2-weights", default=DEFAULT_V2_WEIGHTS,
                    help="the mining detector (default %(default)s)")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--max-neg-frac", type=float, default=0.45)
    ap.add_argument("--seed", type=int, default=0,
                    help="deterministic mining-independent neg-cap subsample seed")
    ap.add_argument("--dry-run", action="store_true",
                    help="report raw union/split counts, no mining, no copy")
    ap.add_argument("--no-mine", action="store_true",
                    help="skip phantom mining (mined_neg/mined_pos left 0 for "
                         "every sample); use if --v2-weights is unavailable")
    ap.add_argument("--self-test", action="store_true",
                    help="offline self-check with synthesized fixtures + an "
                         "injected fake detector; no sim, no real weights")
    args = ap.parse_args()

    if args.self_test:
        return run_self_test()

    flight_dirs = args.flight_train if args.flight_train is not None else [DEFAULT_FLIGHT_TRAIN]
    legacy_capture_dirs = (args.legacy_capture if args.legacy_capture is not None
                           else [DEFAULT_LEGACY_CAPTURE])

    samples, n_missing = collect_all(args.grid, flight_dirs, args.legacy_render,
                                     legacy_capture_dirs)
    if not samples:
        print("[build-v3] FAILED: union is empty (no grid/flight/legacy data found)")
        return 2

    if args.dry_run:
        for split in ("train", "val"):
            for src in ("grid", "flight", "legacy_render", "legacy_capture"):
                subset = [s for s in samples if s["split"] == split and s["source"] == src]
                if not subset:
                    continue
                pos = sum(1 for s in subset if s["positive"])
                print(f"[build-v3] [dry-run] {split:5} {src:14}: "
                      f"pos={pos} neg={len(subset) - pos}")
        n_pos_all = sum(1 for s in samples if s["positive"])
        print(f"[build-v3] [dry-run] TOTAL: {len(samples)} samples "
              f"(pos={n_pos_all} neg={len(samples) - n_pos_all}); "
              "mining and neg-cap NOT applied in --dry-run")
        return 0

    if args.no_mine:
        print("[build-v3] --no-mine: skipping phantom mining, mined_neg/"
              "mined_pos left 0 for every sample")
    else:
        if not os.path.isfile(args.v2_weights):
            print(f"[build-v3] FAILED: --v2-weights not found: {args.v2_weights} "
                  "(pass --no-mine to skip phantom mining)")
            return 2
        detect_fn = make_detector(args.v2_weights)
        mine_report = mine_dataset(samples, detect_fn)
        print(f"[build-v3] mining: scored {mine_report['n_scored']} captured "
              f"frames (grid+flight only, conf={DEPLOY_CONF} + deployed "
              f"self-mask) -> mined_neg={mine_report['n_mined_neg']} "
              f"mined_pos={mine_report['n_mined_pos']}")

    final_samples, cap_report = apply_neg_cap(samples, args.max_neg_frac, args.seed)

    ok, overlap_tags, overlap_ranges = leakage_check(final_samples)
    leak_msg = ("PASS" if ok else
               f"FAIL (overlap run_tags={sorted(overlap_tags)} "
               f"overlap grid ranges={sorted(overlap_ranges)})")
    print("[build-v3] leakage check: no run_tag in both splits, "
          f"no grid range in both splits -> {leak_msg}")

    counts, split_meta_path, data_yaml = write_dataset(final_samples, args.out)
    n_tr = counts["train"]["pos"] + counts["train"]["neg"]
    n_va = counts["val"]["pos"] + counts["val"]["neg"]
    print(f"[build-v3] train: pos={counts['train']['pos']} "
          f"neg={counts['train']['neg']} (total {n_tr})")
    print(f"[build-v3] val:   pos={counts['val']['pos']} "
          f"neg={counts['val']['neg']} (total {n_va})")
    print(f"[build-v3] mined_neg exempt from subsample: "
          f"{cap_report['n_mined_neg_exempt']}; dropped_neg (train, "
          f"non-mined subsample)={cap_report['dropped']}; "
          f"missing_label_errors={n_missing}")
    print(f"[build-v3] dataset: {data_yaml}")
    print(f"[build-v3] split_meta: {split_meta_path}")

    return 0 if ok else 2


# ===========================================================================
# Offline self-test: synthesized fixtures + an INJECTED fake detector
# ===========================================================================
BRIGHT, DARK = 200, 10

# Normalized against the fixed 1280x960 frame v3_onnx_infer.load_gt_box()
# denormalizes against by default, so a gt box lands exactly where
# _fake_detect_fn's fixed (50,50,20,20)px box does ("clean") or far from it
# ("phantom") -- independent of the tiny synthetic PNGs' actual pixel size.
_CLEAN_BOX_NORM = (50.0 / 1280.0, 50.0 / 960.0, 20.0 / 1280.0, 20.0 / 960.0)
_PHANTOM_BOX_NORM = (1200.0 / 1280.0, 900.0 / 960.0, 10.0 / 1280.0, 10.0 / 960.0)


def _fake_detect_fn(frame_bgr):
    """Injected in place of a real v2 ONNX for the self-test (the miner's
    'accept an injectable detector function' contract): fires one detection
    at a fixed (cx=50, cy=50, w=20, h=20)px box whenever the frame is BRIGHT
    (mean > 100), fires nothing for DARK frames. Never touches onnxruntime,
    gz, or the sim."""
    if float(frame_bgr.mean()) > 100.0:
        return [{"cx": 50.0, "cy": 50.0, "w": 20.0, "h": 20.0, "score": 0.9}]
    return []


def _write_fixture_image(path, fill_value):
    cv2.imwrite(path, np.full((32, 32, 3), fill_value, dtype=np.uint8))


def _write_label(path, box_norm):
    with open(path, "w") as fh:
        if box_norm is not None:
            cx, cy, w, h = box_norm
            fh.write(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")
        # else: empty file IS the negative label.


def _build_grid_fixture(root):
    img_dir = os.path.join(root, "images")
    lbl_dir = os.path.join(root, "labels")
    os.makedirs(img_dir)
    os.makedirs(lbl_dir)
    rows = []
    positives = [
        # (stem, range_m, held_out, fill, box_norm)
        ("g_r02_clean", 2.0, 0, BRIGHT, _CLEAN_BOX_NORM),      # train, mined_pos=0
        ("g_r04_phantom", 4.0, 0, BRIGHT, _PHANTOM_BOX_NORM),  # train, mined_pos=1
        ("g_r03_val", 3.0, 1, BRIGHT, _CLEAN_BOX_NORM),        # val (held-out range)
        ("g_r18_val", 18.0, 1, BRIGHT, _CLEAN_BOX_NORM),       # val (held-out range)
    ]
    for stem, r, ho, fill, box in positives:
        _write_fixture_image(os.path.join(img_dir, stem + ".png"), fill)
        _write_label(os.path.join(lbl_dir, stem + ".txt"), box)
        rows.append({"seq": len(rows), "stem": stem, "kind": "positive",
                     "gt_target_x": r, "gt_target_y": 0.0, "gt_target_z": 0.5,
                     "range_m": r, "bearing_deg": 0.0, "height_m": 0.5,
                     "yaw_deg": 0.0, "held_out_range": ho})
    negatives = [("g_neg_loud", BRIGHT)] + [(f"g_neg_quiet{i}", DARK) for i in range(4)]
    for stem, fill in negatives:
        _write_fixture_image(os.path.join(img_dir, stem + ".png"), fill)
        _write_label(os.path.join(lbl_dir, stem + ".txt"), None)
        rows.append({"seq": len(rows), "stem": stem, "kind": "negative",
                     "gt_target_x": "", "gt_target_y": "", "gt_target_z": "",
                     "range_m": "", "bearing_deg": "", "height_m": "",
                     "yaw_deg": "", "held_out_range": 0})
    with open(os.path.join(root, "index.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    return root


def _build_flight_fixture(root):
    img_dir = os.path.join(root, "images")
    lbl_dir = os.path.join(root, "labels")
    os.makedirs(img_dir)
    os.makedirs(lbl_dir)
    # (run_tag, seq, range_m, suffix, fill, box_norm_or_None)
    frames = [
        ("capv3_01", 1, 12.3, "pos", BRIGHT, _CLEAN_BOX_NORM),     # train, mined_pos=0
        ("capv3_01", 2, 13.1, "pos", BRIGHT, _PHANTOM_BOX_NORM),   # train, mined_pos=1
        ("capv3_01", 3, 14.0, "neg", DARK, None),                  # train, mined_neg=0
        ("capv3_01", 4, 15.0, "neg", DARK, None),                  # train, mined_neg=0
        ("capv3_01", 5, 16.0, "neg", BRIGHT, None),                # train, mined_neg=1
        ("capv3_03", 1, 20.0, "pos", BRIGHT, _CLEAN_BOX_NORM),     # val
        ("capv3_03", 2, 21.0, "neg", DARK, None),                  # val
        ("capv3_03", 3, 22.0, "neg", BRIGHT, None),                # val (mined but never subsampled)
        ("capv3_08", 1, 30.0, "pos", BRIGHT, _CLEAN_BOX_NORM),     # val
        ("capv3_08", 2, 31.0, "neg", DARK, None),                  # val
    ]
    for run_tag, seq, r, suf, fill, box in frames:
        stem = f"{run_tag}_{seq:06d}_r{r:05.1f}_{suf}"
        _write_fixture_image(os.path.join(img_dir, stem + ".png"), fill)
        _write_label(os.path.join(lbl_dir, stem + ".txt"), box)
    return root


def _build_legacy_render_fixture(root):
    for nested in ("train", "val"):
        img_dir = os.path.join(root, "images", nested)
        lbl_dir = os.path.join(root, "labels", nested)
        os.makedirs(img_dir)
        os.makedirs(lbl_dir)
        stem = f"legacy_render_{nested}_0"
        _write_fixture_image(os.path.join(img_dir, stem + ".png"), BRIGHT)
        _write_label(os.path.join(lbl_dir, stem + ".txt"), _CLEAN_BOX_NORM)
    return root


def _build_legacy_capture_fixture(root):
    img_dir = os.path.join(root, "images")
    lbl_dir = os.path.join(root, "labels")
    os.makedirs(img_dir)
    os.makedirs(lbl_dir)
    for stem, fill, box in [("legacycap_pos", BRIGHT, _CLEAN_BOX_NORM),
                            ("legacycap_neg", BRIGHT, None)]:
        _write_fixture_image(os.path.join(img_dir, stem + ".png"), fill)
        _write_label(os.path.join(lbl_dir, stem + ".txt"), box)
    return root


def run_self_test() -> int:
    all_ok = True

    def check(name, cond, detail=""):
        nonlocal all_ok
        cond = bool(cond)
        all_ok = all_ok and cond
        suffix = f" ({detail})" if detail else ""
        print(f"[self-test] {name}: {'PASS' if cond else 'FAIL'}{suffix}")

    # --- 1. neg-cap unit test (pure data, no filesystem) ---
    def make_synth(n, positive, split, mined_neg=0):
        return [{"positive": positive, "split": split, "mined_neg": mined_neg}
                for _ in range(n)]

    synth = (make_synth(5, True, "train")
            + make_synth(3, False, "train", mined_neg=1)
            + make_synth(20, False, "train", mined_neg=0)
            + make_synth(2, True, "val")
            + make_synth(10, False, "val"))
    kept, report = apply_neg_cap(synth, max_neg_frac=0.3, seed=0)
    val_kept = [s for s in kept if s["split"] == "val"]
    train_pos_kept = [s for s in kept if s["split"] == "train" and s["positive"]]
    train_mined_kept = [s for s in kept
                        if s["split"] == "train" and not s["positive"] and s["mined_neg"]]
    check("neg-cap: val split untouched", len(val_kept) == 12, f"{len(val_kept)}")
    check("neg-cap: positives never dropped", len(train_pos_kept) == 5,
          f"{len(train_pos_kept)}")
    check("neg-cap: all mined negatives kept (exempt)", len(train_mined_kept) == 3,
          f"{len(train_mined_kept)}")
    check("neg-cap: mined can push final frac above the cap (guaranteed-include)",
          report["n_neg_post"] == 3 and report["dropped"] == 20,
          f"n_neg_post={report['n_neg_post']} dropped={report['dropped']}")
    kept2, _ = apply_neg_cap(synth, max_neg_frac=0.3, seed=0)
    ids1 = [id(s) for s in kept if s["split"] == "train" and not s["positive"]]
    ids2 = [id(s) for s in kept2 if s["split"] == "train" and not s["positive"]]
    check("neg-cap: deterministic given the same seed", ids1 == ids2)

    synth2 = (make_synth(10, True, "train")
             + make_synth(2, False, "train", mined_neg=1)
             + make_synth(30, False, "train", mined_neg=0))
    kept3, report3 = apply_neg_cap(synth2, max_neg_frac=0.4, seed=0)
    check("neg-cap: normal-path math matches merge_dataset_v2's target-count formula",
          report3["n_neg_post"] == 6 and report3["dropped"] == 26,
          f"n_neg_post={report3['n_neg_post']} dropped={report3['dropped']}")

    # --- 2. end-to-end pipeline on synthesized fixture dirs ---
    with tempfile.TemporaryDirectory(prefix="build_v3_selftest_") as tmp:
        grid_dir = _build_grid_fixture(os.path.join(tmp, "grid"))
        flight_dir = _build_flight_fixture(os.path.join(tmp, "flight"))
        legacy_render_dir = _build_legacy_render_fixture(os.path.join(tmp, "legacy_render"))
        legacy_capture_dir = _build_legacy_capture_fixture(os.path.join(tmp, "legacy_capture"))

        samples, n_missing = collect_all(grid_dir, [flight_dir], legacy_render_dir,
                                         [legacy_capture_dir])
        check("collect: no missing-label errors", n_missing == 0, f"{n_missing}")

        grid_val_ranges = {s["range_m"] for s in samples
                           if s["source"] == "grid" and s["split"] == "val"}
        check("split: grid held-out ranges {3.0,18.0} land ONLY in val",
              grid_val_ranges == {3.0, 18.0}, f"{grid_val_ranges}")
        grid_train_ranges = {s["range_m"] for s in samples
                             if s["source"] == "grid" and s["split"] == "train"
                             and s["range_m"] is not None}
        check("split: no grid train range is a held-out range",
              grid_train_ranges.isdisjoint({3.0, 18.0}), f"{grid_train_ranges}")

        flight_val_tags = {s["flight_key"] for s in samples
                           if s["source"] == "flight" and s["split"] == "val"}
        flight_train_tags = {s["flight_key"] for s in samples
                             if s["source"] == "flight" and s["split"] == "train"}
        check("split: capv3_03/capv3_08 land ONLY in val",
              flight_val_tags == {"capv3_03", "capv3_08"}, f"{flight_val_tags}")
        check("split: capv3_01 lands ONLY in train",
              flight_train_tags == {"capv3_01"}, f"{flight_train_tags}")
        check("split: no run_tag spans both splits",
              flight_val_tags.isdisjoint(flight_train_tags))

        legacy_splits = {s["split"] for s in samples
                         if s["source"] in ("legacy_render", "legacy_capture")}
        check("split: legacy sources are ALL train (old val role retired)",
              legacy_splits == {"train"}, f"{legacy_splits}")

        mine_report = mine_dataset(samples, _fake_detect_fn)
        mined_neg_stems = {os.path.basename(s["image"]) for s in samples if s.get("mined_neg")}
        # 3 loud negatives are mine-eligible: 1 grid (train) + 2 flight (one
        # TRAIN, one VAL -- mining deliberately runs on both splits, plan
        # Sec 4.2 "mining runs on BOTH train and val captured frames").
        check("mining: exactly the 3 loud negatives (grid+flight, both splits) are mined_neg",
              mined_neg_stems == {"g_neg_loud.png", "capv3_01_000005_r016.0_neg.png",
                                  "capv3_03_000003_r022.0_neg.png"},
              f"{mined_neg_stems}")
        mined_pos_stems = {os.path.basename(s["image"]) for s in samples if s.get("mined_pos")}
        check("mining: exactly the 2 phantom positives (grid+flight) are mined_pos",
              mined_pos_stems == {"g_r04_phantom.png", "capv3_01_000002_r013.1_pos.png"},
              f"{mined_pos_stems}")
        legacy_mined = any(s.get("mined_neg") or s.get("mined_pos") for s in samples
                          if s["source"] in ("legacy_render", "legacy_capture"))
        check("mining: legacy frames NEVER mined (not mine-eligible)", not legacy_mined)
        check("mining: only grid+flight frames scored",
              mine_report["n_scored"] == sum(1 for s in samples if s["mine_eligible"]),
              f"n_scored={mine_report['n_scored']}")

        final_samples, cap_report = apply_neg_cap(samples, max_neg_frac=0.45, seed=0)
        ok_leak, overlap_tags, overlap_ranges = leakage_check(final_samples)
        check("leakage check passes on the assembled split", ok_leak,
              f"overlap_tags={overlap_tags} overlap_ranges={overlap_ranges}")

        out_dir = os.path.join(tmp, "onboard_dataset_v3")
        counts, split_meta_path, data_yaml = write_dataset(final_samples, out_dir)
        check("write: split_meta.csv exists", os.path.isfile(split_meta_path))
        check("write: data.yaml exists", os.path.isfile(data_yaml))

        with open(split_meta_path, newline="") as fh:
            meta_rows = list(csv.DictReader(fh))
        expect_cols = {"dest_stem", "source", "flight_key", "range_m", "positive",
                      "mined_neg", "mined_pos", "split"}
        check("write: split_meta.csv columns match the spec",
              bool(meta_rows) and set(meta_rows[0].keys()) == expect_cols,
              f"{sorted(meta_rows[0].keys()) if meta_rows else 'EMPTY'}")
        check("write: split_meta.csv row count matches the written sample count",
              len(meta_rows) == len(final_samples),
              f"{len(meta_rows)} vs {len(final_samples)}")

        n_img_train = len(os.listdir(os.path.join(out_dir, "images", "train")))
        n_img_val = len(os.listdir(os.path.join(out_dir, "images", "val")))
        n_meta_train = sum(1 for r in meta_rows if r["split"] == "train")
        n_meta_val = sum(1 for r in meta_rows if r["split"] == "val")
        check("write: on-disk file counts match split_meta split counts",
              n_img_train == n_meta_train and n_img_val == n_meta_val,
              f"disk train={n_img_train} val={n_img_val}; "
              f"meta train={n_meta_train} val={n_meta_val}")

        meta_tags_val = {r["flight_key"] for r in meta_rows if r["split"] == "val" and r["flight_key"]}
        meta_tags_train = {r["flight_key"] for r in meta_rows if r["split"] == "train" and r["flight_key"]}
        check("write: split_meta run_tag columns agree with the split rule",
              meta_tags_val == {"capv3_03", "capv3_08"} and meta_tags_train == {"capv3_01"},
              f"val={meta_tags_val} train={meta_tags_train}")

        meta_mined_neg = sum(1 for r in meta_rows if r["mined_neg"] == "1")
        meta_mined_pos = sum(1 for r in meta_rows if r["mined_pos"] == "1")
        check("write: split_meta mined_neg/mined_pos counts survive the copy",
              meta_mined_neg == mine_report["n_mined_neg"]
              and meta_mined_pos == mine_report["n_mined_pos"],
              f"mined_neg={meta_mined_neg} mined_pos={meta_mined_pos}")

    # --- 3. stem-collision dedup unit check ---
    with tempfile.TemporaryDirectory(prefix="build_v3_dedup_") as tmp2:
        a_dir = os.path.join(tmp2, "a")
        b_dir = os.path.join(tmp2, "b")
        os.makedirs(a_dir)
        os.makedirs(b_dir)
        _write_fixture_image(os.path.join(a_dir, "dup_test.png"), BRIGHT)
        _write_label(os.path.join(a_dir, "dup_test.txt"), _CLEAN_BOX_NORM)
        _write_fixture_image(os.path.join(b_dir, "dup_test.png"), DARK)
        _write_label(os.path.join(b_dir, "dup_test.txt"), None)
        dup_samples = [
            {"image": os.path.join(a_dir, "dup_test.png"),
             "label": os.path.join(a_dir, "dup_test.txt"), "positive": True,
             "source": "grid", "flight_key": "", "range_m": None, "split": "train",
             "mine_eligible": False, "mined_neg": 0, "mined_pos": 0},
            {"image": os.path.join(b_dir, "dup_test.png"),
             "label": os.path.join(b_dir, "dup_test.txt"), "positive": False,
             "source": "grid", "flight_key": "", "range_m": None, "split": "train",
             "mine_eligible": False, "mined_neg": 0, "mined_pos": 0},
        ]
        dup_out = os.path.join(tmp2, "out")
        write_dataset(dup_samples, dup_out)
        n_imgs = len(os.listdir(os.path.join(dup_out, "images", "train")))
        check("write: stem collision -> unique-stem dedup (both files survive)",
              n_imgs == 2, f"{n_imgs}")

    print(f"[self-test] {'ALL PASS' if all_ok else 'SOME FAILED'}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
