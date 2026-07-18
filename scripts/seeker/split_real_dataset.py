#!/usr/bin/env python3
"""Flight-level train/val assembler for autolabel_from_apriltag.py outputs
(docs/real_data_pipeline.md), used by real_data_pipeline_run.sh.

WHY flight-level, not per-frame: train_real_data.py's recipe note and the
v3/rebal anti-mirage gate both say the same thing -- frames within one flight
are temporally correlated (30 Hz-ish captures of one pass), so a per-frame
Bernoulli split (merge_dataset_v2.py's approach, fine for the sim renders it
was built for) leaks near-duplicates across train/val on REAL flight footage
and inflates val metrics. Here an entire --flight directory (one
autolabel_from_apriltag.py output = one recorded session) is assigned WHOLE
to train or val -- never split.

WHAT it merges:
  --flight DIR   (repeatable) autolabel_from_apriltag.py output: DIR/images/*.png
                 + DIR/labels/*.txt (flat, may be positives+negatives mixed --
                 target-present sessions with --negatives-from-untagged off
                 have only positives, since tag-miss frames were dropped there).
  --val-flight NAME (repeatable) the basename of a --flight DIR to send WHOLE
                 to val; every other --flight DIR goes WHOLE to train.
  --neg DIR      (repeatable) a target-FREE autolabel_from_apriltag.py output
                 (run with --negatives-from-untagged over pre-launch/empty
                 footage). ALL its frames are TRAIN-only candidates, subsampled
                 to --max-neg-frac of the train split (recipe: ~15-20%
                 negatives -- the rebal lesson, ADR add #15) using the same
                 target-count formula as merge_dataset_v2.py. Never placed in
                 val: a coin-flip could otherwise put an all-negative session on
                 either side and understate the real val floor.

OUTPUT: --out/images/{train,val} + --out/labels/{train,val} + --out/data.yaml
(standard ultralytics YOLO layout, same convention as merge_dataset_v2.py /
build_onboard_dataset_v3.py).

Usage:
    scripts/seeker/split_real_dataset.py --flight datasets/raw/session1 \
        --flight datasets/raw/session2 --val-flight session2 \
        --neg datasets/raw/prelaunch_empty --out datasets/real_flight_01
    scripts/seeker/split_real_dataset.py --self-test    # tiny synthetic fixture, no data needed

No cv2/numpy/torch dependency -- pure file I/O, runs under any venv.
"""
import argparse
import glob
import os
import shutil
import sys
import tempfile


def _load_dir(d):
    """Return [(image_path, label_path, positive_bool), ...] for one flat
    images/+labels/ dir. positive = label file has >=1 non-blank line."""
    items = []
    img_paths = sorted(glob.glob(os.path.join(d, "images", "*.png")) +
                       glob.glob(os.path.join(d, "images", "*.jpg")))
    for ip in img_paths:
        stem = os.path.splitext(os.path.basename(ip))[0]
        lp = os.path.join(d, "labels", stem + ".txt")
        if not os.path.isfile(lp):
            continue
        positive = any(line.strip() for line in open(lp))
        items.append((ip, lp, positive))
    return items


def _copy_into(items, split, img_dir, lbl_dir, used_stems):
    n = 0
    for ip, lp, _pos in items:
        base = os.path.basename(ip)
        stem, ext = os.path.splitext(base)
        dest_stem = stem
        k = 0
        while dest_stem in used_stems[split]:
            k += 1
            dest_stem = f"{stem}_dup{k}"
        used_stems[split].add(dest_stem)
        shutil.copy2(ip, os.path.join(img_dir, dest_stem + ext))
        shutil.copy2(lp, os.path.join(lbl_dir, dest_stem + ".txt"))
        n += 1
    return n


def assemble(flight_dirs, val_flight_names, neg_dirs, out_dir, max_neg_frac, seed):
    import random
    val_set = set(val_flight_names)
    train_items, val_items = [], []
    per_flight = {}
    for d in flight_dirs:
        items = _load_dir(d)
        per_flight[os.path.basename(d)] = len(items)
        (val_items if os.path.basename(d) in val_set else train_items).extend(items)

    unmatched = val_set - {os.path.basename(d) for d in flight_dirs}
    if unmatched:
        raise SystemExit(f"--val-flight name(s) {sorted(unmatched)} match no --flight "
                         f"dir basename (have: {sorted(per_flight)})")

    neg_items = []
    for d in neg_dirs:
        neg_items.extend(_load_dir(d))
    # Only truly-negative frames from --neg dirs are eligible (guards against
    # pointing --neg at a mixed dir by mistake).
    neg_items = [it for it in neg_items if not it[2]]

    n_train_pos = sum(1 for it in train_items if it[2])
    n_train_neg_inline = sum(1 for it in train_items if not it[2])
    target_total_neg = int(max_neg_frac * n_train_pos / max(1e-9, (1.0 - max_neg_frac))) \
        if n_train_pos > 0 else len(neg_items)
    target_from_pool = max(0, target_total_neg - n_train_neg_inline)
    rng = random.Random(seed)
    if target_from_pool < len(neg_items):
        neg_items = sorted(neg_items, key=lambda it: it[0])  # deterministic order
        rng.shuffle(neg_items)
        dropped = len(neg_items) - target_from_pool
        neg_items = neg_items[:target_from_pool]
    else:
        dropped = 0
    train_items = train_items + neg_items

    img_tr = os.path.join(out_dir, "images", "train")
    img_va = os.path.join(out_dir, "images", "val")
    lbl_tr = os.path.join(out_dir, "labels", "train")
    lbl_va = os.path.join(out_dir, "labels", "val")
    for p in (img_tr, img_va, lbl_tr, lbl_va):
        os.makedirs(p, exist_ok=True)
    used = {"train": set(), "val": set()}
    n_tr = _copy_into(train_items, "train", img_tr, lbl_tr, used)
    n_va = _copy_into(val_items, "val", img_va, lbl_va, used)

    data_yaml = os.path.join(out_dir, "data.yaml")
    with open(data_yaml, "w") as fh:
        fh.write(
            "# split_real_dataset.py -- FLIGHT-LEVEL split (whole sessions to\n"
            "# train/val, never per-frame) of autolabel_from_apriltag.py output.\n"
            f"path: {os.path.abspath(out_dir)}\n"
            "train: images/train\n"
            "val: images/val\n"
            "nc: 1\n"
            "names: ['drone']\n"
        )
    return dict(per_flight=per_flight, val_flights=sorted(val_set),
               n_train=n_tr, n_val=n_va, n_train_pos=n_train_pos,
               neg_pool=len(neg_items) + dropped, neg_used=len(neg_items),
               neg_dropped=dropped, data_yaml=data_yaml)


def run(args):
    if not args.flight:
        raise SystemExit("need >=1 --flight DIR (or --self-test)")
    if not args.val_flight:
        raise SystemExit("need >=1 --val-flight NAME (matches a --flight dir's "
                         "basename) -- an empty val split breaks ultralytics "
                         "training and hides the real val floor")
    stats = assemble(args.flight, args.val_flight, args.neg, args.out,
                     args.max_neg_frac, args.seed)
    print(f"[split] flights: " +
          ", ".join(f"{k}={v}" for k, v in stats["per_flight"].items()))
    print(f"[split] val flights: {stats['val_flights']}")
    print(f"[split] train: {stats['n_train']} frames ({stats['n_train_pos']} positive "
          f"+ {stats['n_train'] - stats['n_train_pos']} negative)")
    print(f"[split] val:   {stats['n_val']} frames")
    print(f"[split] negative pool: {stats['neg_pool']} available, "
          f"{stats['neg_used']} used (cap {args.max_neg_frac}), "
          f"{stats['neg_dropped']} dropped")
    print(f"[split] dataset: {stats['data_yaml']}")
    if stats["n_val"] == 0:
        print("[split] WARNING: val split is empty (val-flight matched but had 0 frames)")
    return 0


def self_test():
    """Build a tiny synthetic fixture (2 flights + 1 neg dir, plain files --
    no cv2/image decoding needed here, split only copies bytes) and verify:
    (a) the val-flight's frames land ONLY in val, (b) neg frames land ONLY in
    train and respect the cap, (c) data.yaml is well-formed."""
    ok = True
    with tempfile.TemporaryDirectory() as tmp:
        def make_flight(name, n_pos, n_neg_inline=0):
            d = os.path.join(tmp, name)
            os.makedirs(os.path.join(d, "images"))
            os.makedirs(os.path.join(d, "labels"))
            for i in range(n_pos):
                open(os.path.join(d, "images", f"{name}_{i:03d}.png"), "w").close()
                open(os.path.join(d, "labels", f"{name}_{i:03d}.txt"), "w").write(
                    "0 0.5 0.5 0.05 0.05\n")
            for i in range(n_neg_inline):
                j = n_pos + i
                open(os.path.join(d, "images", f"{name}_{j:03d}.png"), "w").close()
                open(os.path.join(d, "labels", f"{name}_{j:03d}.txt"), "w").close()
            return d

        flightA = make_flight("flightA", n_pos=8)
        flightB = make_flight("flightB", n_pos=4)
        negdir = make_flight("negs", n_pos=0, n_neg_inline=40)

        out = os.path.join(tmp, "out")
        stats = assemble([flightA, flightB], ["flightB"], [negdir], out,
                         max_neg_frac=0.2, seed=0)

        check_a = stats["n_val"] == 4
        print(f"[self-test] val = flightB frames only (4): got {stats['n_val']} : "
              f"{'OK' if check_a else 'FAIL'}")
        ok = ok and check_a

        # train should be 8 positives + capped negatives ~= 0.2 * 8/(1-0.2) = 2
        expect_neg = int(0.2 * 8 / 0.8)
        check_b = stats["neg_used"] == expect_neg
        print(f"[self-test] neg cap: expect {expect_neg} used, got {stats['neg_used']} : "
              f"{'OK' if check_b else 'FAIL'}")
        ok = ok and check_b

        val_img_dir = os.path.join(out, "images", "val")
        val_names = set(os.listdir(val_img_dir))
        check_c = all(n.startswith("flightB_") for n in val_names) and len(val_names) == 4
        print(f"[self-test] val dir contains ONLY flightB frames: {'OK' if check_c else 'FAIL'}")
        ok = ok and check_c

        check_d = os.path.isfile(stats["data_yaml"]) and "names: ['drone']" in \
            open(stats["data_yaml"]).read()
        print(f"[self-test] data.yaml well-formed: {'OK' if check_d else 'FAIL'}")
        ok = ok and check_d

        # unmatched --val-flight name must raise
        raised = False
        try:
            assemble([flightA], ["nope"], [], os.path.join(tmp, "out2"), 0.2, 0)
        except SystemExit:
            raised = True
        print(f"[self-test] unmatched --val-flight raises: {'OK' if raised else 'FAIL'}")
        ok = ok and raised

    print(f"[self-test] {'PASS' if ok else 'FAIL'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flight", action="append", default=[],
                    help="autolabel_from_apriltag.py output dir (repeatable)")
    ap.add_argument("--val-flight", action="append", default=[],
                    help="basename of a --flight dir to hold out WHOLE for val (repeatable)")
    ap.add_argument("--neg", action="append", default=[],
                    help="target-free autolabel output dir, train-only (repeatable)")
    ap.add_argument("--max-neg-frac", type=float, default=0.18,
                    help="cap negatives at this fraction of the TRAIN split (default 0.18)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", help="output dataset dir")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return 0 if self_test() else 1
    if not args.out:
        ap.error("--out is required (or --self-test)")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
