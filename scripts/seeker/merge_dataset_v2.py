#!/usr/bin/env python3
"""Build sim_dataset_v2: v1 in-domain renders UNION in-flight capture
(ADR-0040 addendum).

WHY: ADR-0040's addendum traced the v1 fine-tune's flight regressions to a
POSITIVES-ONLY training set -- render_sim_dataset.py's static ground renders
never show the interceptor's own body-fixed prop arms at the FOV periphery,
so the model was never taught what "not the target" looks like and
false-locks on them in flight. capture_flight_frames.py harvests real
in-flight hard NEGATIVES (plus flight-realistic positives) to fix that. This
script is the union+re-split step: it merges v1's renders with one or more
flight-capture runs into a single sim_dataset_v2, deterministically.

WHAT: unions every image+label pair from v1 (both its existing train AND val
splits -- the v1 split is thrown away here and everything is re-split fresh)
with every image+label pair from each --capture directory (flat
images/+labels/, as written by capture_flight_frames.py). A label file with
>= 1 non-blank line is a POSITIVE; an empty label file is a NEGATIVE (a
missing label file for an image is an ERROR, counted and skipped -- it is
never silently treated as either). If negatives end up above --max-neg-frac
of the total, they are deterministically SUBSAMPLED DOWN (positives are never
dropped) and the drop count is reported -- no silent caps. The surviving set
is re-split into train/val via `--val-frac`, using
`np.random.RandomState(--seed)` for every random decision (no `random`
module, no wall-clock seeding -- CLAUDE.md: numbers must be reproducible).
Files are COPIED (not symlinked), so sim_dataset_v2 is a standalone dataset.

HONESTY: this is a pure filesystem/labeling tool -- it does not touch
gz-transport, gt_*, or any live sim; the honesty boundary was already
enforced by the two dataset producers (render_sim_dataset.py,
capture_flight_frames.py) at capture time.

RUN:
    .venv-seeker/bin/python scripts/seeker/merge_dataset_v2.py \\
      --v1 scripts/seeker/data/sim_dataset \\
      --capture scripts/seeker/data/flight_capture \\
      --out scripts/seeker/data/sim_dataset_v2
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)

DEFAULT_V1 = os.path.join(HERE, "data", "sim_dataset")
DEFAULT_CAPTURE = os.path.join(HERE, "data", "flight_capture")
DEFAULT_OUT = os.path.join(HERE, "data", "sim_dataset_v2")

IMG_EXTS = (".png", ".jpg", ".jpeg")


def collect_dir(images_dir: str, labels_dir: str, source_tag: str):
    """Scan one flat images_dir/labels_dir pair.

    Returns (samples, n_missing_label) where samples is a list of dicts
    {image, label, positive, source}, in a FIXED (sorted-filename) order for
    reproducibility -- os.listdir() order is not guaranteed, so we never rely
    on it directly.
    """
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
        positive = len(lines) >= 1
        samples.append({
            "image": img_path, "label": lbl_path,
            "positive": positive, "source": source_tag,
        })
    return samples, n_missing


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--v1", default=DEFAULT_V1,
                    help="v1 render dataset root (has images/{train,val}, "
                         "labels/{train,val}; default %(default)s)")
    ap.add_argument("--capture", action="append", default=None,
                    help="flight-capture directory (flat images/+labels/); "
                         f"repeatable (default: [{DEFAULT_CAPTURE}])")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--val-frac", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0, help="deterministic split/subsample seed")
    ap.add_argument("--max-neg-frac", type=float, default=0.4,
                    help="cap negatives at this fraction of the total (default 0.4); "
                         "excess negatives are deterministically subsampled, "
                         "positives are never dropped")
    args = ap.parse_args()

    capture_dirs = args.capture if args.capture is not None else [DEFAULT_CAPTURE]

    all_samples = []
    n_missing_total = 0

    for split in ("train", "val"):
        img_dir = os.path.join(args.v1, "images", split)
        lbl_dir = os.path.join(args.v1, "labels", split)
        s, m = collect_dir(img_dir, lbl_dir, f"v1:{split}")
        if not os.path.isdir(img_dir):
            print(f"[merge] note: v1 split has no images/ dir, treated as empty: {img_dir}")
        all_samples.extend(s)
        n_missing_total += m

    for cap_dir in capture_dirs:
        img_dir = os.path.join(cap_dir, "images")
        lbl_dir = os.path.join(cap_dir, "labels")
        if not os.path.isdir(img_dir):
            print(f"[merge] note: capture dir has no images/ (empty or missing), "
                  f"treated as empty: {cap_dir}")
        s, m = collect_dir(img_dir, lbl_dir, f"capture:{os.path.basename(os.path.normpath(cap_dir))}")
        all_samples.extend(s)
        n_missing_total += m

    positives = [s for s in all_samples if s["positive"]]
    negatives = [s for s in all_samples if not s["positive"]]
    n_pos, n_neg_raw = len(positives), len(negatives)
    total_raw = n_pos + n_neg_raw

    print(f"[merge] union: {len(all_samples)} labeled samples "
          f"(pos={n_pos} neg={n_neg_raw}); missing_label_errors={n_missing_total} "
          "(image with no label file -- skipped)")

    if total_raw == 0:
        print("[merge] FAILED: union is empty (no v1 renders and no capture data found)")
        return 2

    rng = np.random.RandomState(args.seed)

    dropped_neg = 0
    if n_neg_raw > 0 and args.max_neg_frac < 1.0:
        neg_frac = n_neg_raw / total_raw
        if neg_frac > args.max_neg_frac:
            target_neg = int(np.floor(args.max_neg_frac * n_pos / (1.0 - args.max_neg_frac)))
            target_neg = max(0, min(target_neg, n_neg_raw))
            if target_neg < n_neg_raw:
                order = rng.permutation(n_neg_raw)
                keep_idx = sorted(order[:target_neg].tolist())
                negatives = [negatives[i] for i in keep_idx]
                dropped_neg = n_neg_raw - target_neg
                print(f"[merge] negatives {n_neg_raw} exceed max_neg_frac="
                      f"{args.max_neg_frac} of total -> subsampled to {target_neg} "
                      f"(dropped {dropped_neg}, seed={args.seed})")

    final_samples = positives + negatives
    total_final = len(final_samples)
    if total_final == 0:
        print("[merge] FAILED: union is empty after max-neg-frac subsampling "
              "(zero positives means zero negatives can be kept under the cap)")
        return 2

    img_tr = os.path.join(args.out, "images", "train")
    img_va = os.path.join(args.out, "images", "val")
    lbl_tr = os.path.join(args.out, "labels", "train")
    lbl_va = os.path.join(args.out, "labels", "val")
    for d in (img_tr, img_va, lbl_tr, lbl_va):
        os.makedirs(d, exist_ok=True)

    counts = {"train": {"pos": 0, "neg": 0}, "val": {"pos": 0, "neg": 0}}
    used_stems = {"train": set(), "val": set()}

    # Deterministic re-split: one Bernoulli draw per sample, in the fixed
    # (sorted, source-ordered) order final_samples was built in -- same
    # per-item rng.random_sample() < val_frac pattern as render_sim_dataset.py.
    for s in final_samples:
        is_val = rng.random_sample() < args.val_frac
        split = "val" if is_val else "train"
        img_dst_dir = img_va if is_val else img_tr
        lbl_dst_dir = lbl_va if is_val else lbl_tr

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

        key = "pos" if s["positive"] else "neg"
        counts[split][key] += 1

    data_yaml = os.path.join(args.out, "data.yaml")
    with open(data_yaml, "w") as fh:
        fh.write(
            "# sim_dataset_v2 = v1 in-domain renders (render_sim_dataset.py) UNION\n"
            "# in-flight hard-negative/positive capture (capture_flight_frames.py).\n"
            "# ADR-0040 addendum: hard NEGATIVE flight frames (the interceptor's own\n"
            "# body-fixed prop arms at the FOV periphery) are the primary v2 lever\n"
            "# for the v1 false-lock regression.\n"
            f"path: {os.path.abspath(args.out)}\n"
            "train: images/train\n"
            "val: images/val\n"
            "nc: 1\n"
            "names: ['drone']\n"
        )

    n_tr = counts["train"]["pos"] + counts["train"]["neg"]
    n_va = counts["val"]["pos"] + counts["val"]["neg"]
    print(f"[merge] train: pos={counts['train']['pos']} neg={counts['train']['neg']} "
          f"(total {n_tr})")
    print(f"[merge] val:   pos={counts['val']['pos']} neg={counts['val']['neg']} "
          f"(total {n_va})")
    print(f"[merge] wrote {total_final} samples ({n_tr} train, {n_va} val); "
          f"dropped_neg={dropped_neg} missing_label_errors={n_missing_total}")
    print(f"[merge] dataset: {data_yaml}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
