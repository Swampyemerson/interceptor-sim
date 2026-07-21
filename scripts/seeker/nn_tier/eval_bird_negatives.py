#!/usr/bin/env python3
"""Bird false-fire eval — the ONE open cell of the mono-vs-color camera decision.

WHAT THIS IS (mono-vs-color bird A/B, 2026-07-21)
--------------------------------------------------
The camera decision (`docs/nn_tier/PLAN.md`, `baseline_scoreboard.md` §5) is
already KEEP MONO. What was never isolated is the RESIDUAL bird-false-fire
risk for a model that is trained mono-native (not the OOD color-net-fed-gray
artifact the 0.667->0.967 regression measured). This script scores any
nn_tier ONNX model against the Mendeley "Drone-vs-Bird" corpus's real bird
photos (`scripts/seeker/nn_tier/data/dvb_extract/Dataset/*/images/B*`) at the
deployed operating point (conf 0.25) and reports:
  * false-fire rate (fraction of bird images with >=1 drone-class detection)
  * confidence distribution of the top score on images that DID fire
    (mirrors the "median top-score 0.32->0.43" mono-probe style number)

THE LEAKAGE TRAP THIS SCRIPT CLOSES
------------------------------------
`prepare_nn_tier_dataset.py:index_dvb()` POOLS all three of DVB's own splits
(train/valid/test — "their random split is ignored") and samples a fixed
count of bird images from the pool as `dvb_bird` TRAIN-only negatives (whole
source pinned train, no scene ids). That means some of the DVB *test*-split
bird images (filename prefix `BT`) already trained n-mono/n-color as
empty-label negatives — scoring "held-out" bird false-fire on the naive full
BT-test folder would silently include exam questions the model has seen the
answer key to, and would bias the false-fire rate down.

`leaked_dvb_bird_test_basenames()` below re-derives the exact leaked filename
set from `manifest_all.csv` (uid pattern `dvb_bird_all_BT__<N>_` — the `BT`
prefix, one underscore-run, distinguishes it from `BTR` [DVB-train] and `BV`
[DVB-valid] prefixed uids, which sanitize to `BTR__<N>_` / `BV__<N>_`) and
excludes them from the eval set by default. Verified 2026-07-21: 68/361 BT
test images leaked into nn_tier TRAIN this way. The headline number in this
script's output is always the DISJOINT (leak-excluded) set; the
leak-INCLUDED number is reported alongside only for transparency, clearly
labeled, never as the verdict number.

HONESTY (CLAUDE.md, ADR-0061): labels are not even needed here — the whole
DVB Bird partition is scored as drone-FREE (the corpus's own audit found its
Bird-class boxes largely mislabeled onto birds, `baseline_scoreboard.md` §3
finding 1; policy: treat every `B*`-prefixed image as a fires-are-false-fires
negative, ground truth = zero drone boxes). Nothing here trains a model or
feeds a live seeker — read-only scoring.

Run (read-only seeker venv, CPU ORT):
  .venv-seeker/bin/python scripts/seeker/nn_tier/eval_bird_negatives.py \\
      --models n_mono --modes gray --tag mono_v_color
Self-test (no full dataset scoring; exits 0/1):
  .venv-seeker/bin/python scripts/seeker/nn_tier/eval_bird_negatives.py --self-test
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import glob
import json
import os
import re
import sys
import time

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))          # scripts/seeker/nn_tier
SEEKER = os.path.abspath(os.path.join(HERE, ".."))          # scripts/seeker
REPO = os.path.abspath(os.path.join(SEEKER, "..", ".."))    # repo root
sys.path.insert(0, SEEKER)
sys.path.insert(0, HERE)

from v3_onnx_infer import load_session, DEPLOY_CONF          # noqa: E402
from baseline_eval import run_model_on_image, to_gray3        # noqa: E402

WEIGHTS = os.path.join(SEEKER, "weights")
DVB_ROOT = os.path.join(HERE, "data", "dvb_extract", "Dataset")
MANIFEST_ALL = os.path.join(SEEKER, "data", "nn_tier", "manifest_all.csv")
OUT_DIR = os.path.join(REPO, "logs", "nn_tier")

# name -> (onnx path, drone class id, n classes) — same convention as
# eval_n_mono.py / baseline_eval.py MODELS dicts (kept in sync by hand).
MODELS = {
    "n_mono":        (os.path.join(WEIGHTS, "nn_tier", "n-mono.onnx"), 0, 1),
    "n_mono_aug":    (os.path.join(WEIGHTS, "nn_tier", "n-mono-aug.onnx"), 0, 1),
    "s_mono":        (os.path.join(WEIGHTS, "nn_tier", "s-mono.onnx"), 0, 1),
    "n_color":       (os.path.join(WEIGHTS, "nn_tier", "n-color.onnx"), 0, 1),
    "v2_deployed":   (os.path.join(WEIGHTS, "drone_finetuned_quad_v2.onnx"), 0, 1),
}

CONF_BINS = [(0.25, 0.4), (0.4, 0.6), (0.6, 0.8), (0.8, 1.01)]
CONF_BIN_NAMES = ["0.25-0.40", "0.40-0.60", "0.60-0.80", "0.80-1.00"]

_LEAK_UID_RE = re.compile(r"^dvb_bird_all_BT__(\d+)_$")


# ---------------------------------------------------------------- leak check

def leaked_dvb_bird_test_basenames(manifest_all_path=MANIFEST_ALL):
    """Return the set of DVB *test*-split bird filenames ('BT (N).jpg') that
    prepare_nn_tier_dataset.py's index_dvb() pooled into the nn_tier TRAIN
    split as `dvb_bird` negatives (leakage vs. an eval on the DVB test BT
    folder). Derived from manifest_all.csv's uid column — see module
    docstring for the sanitize-regex reasoning. Returns empty set if the
    manifest is missing (caller should then eval unfiltered, degraded mode)."""
    if not os.path.isfile(manifest_all_path):
        return set()
    leaked = set()
    with open(manifest_all_path) as fh:
        for r in csv.DictReader(fh):
            if r.get("source") != "dvb_bird":
                continue
            m = _LEAK_UID_RE.match(r["uid"])
            if m:
                leaked.add(f"BT ({m.group(1)}).jpg")
    return leaked


# ---------------------------------------------------------------- dataset

def collect_bird_images(prefix="BT", split="test", exclude_leaked=True):
    """Sorted list of DVB Bird-partition image paths for `prefix` under
    `split` (default: the held-out DVB test folder, filename prefix BT).
    Every one of these is treated as a drone-FREE negative regardless of its
    on-disk label file (DVB's own Bird-class boxes are largely mislabeled
    onto birds, baseline_scoreboard.md finding 1 — labels are not read here).
    """
    img_dir = os.path.join(DVB_ROOT, split, "images")
    paths = sorted(glob.glob(os.path.join(img_dir, prefix + "*")))
    if not exclude_leaked:
        return paths, set()
    leaked = leaked_dvb_bird_test_basenames()
    kept = [p for p in paths if os.path.basename(p) not in leaked]
    excluded = [p for p in paths if os.path.basename(p) in leaked]
    return kept, {os.path.basename(p) for p in excluded}


# ---------------------------------------------------------------- scoring

def _conf_bin(score):
    for name, (lo, hi) in zip(CONF_BIN_NAMES, CONF_BINS):
        if lo <= score < hi:
            return name
    return CONF_BIN_NAMES[-1]


def eval_bird_false_fire(model_key, mode, img_paths, progress_fh=None, tag=""):
    """Run one model, one color mode, over img_paths (all drone-FREE bird
    photos). Returns (summary dict, perimage rows, top_scores list)."""
    path, drone_cls, n_classes = MODELS[model_key]
    sess, iname, imgsz = load_session(path, intra_threads=8)
    n_ok = n_fired = n_fires_total = 0
    top_scores = []          # top det score per FIRED image
    all_fire_scores = []     # every det score >= DEPLOY_CONF, any image
    perimage = []
    lat = []
    for k, ip in enumerate(img_paths):
        frame = cv2.imread(ip)
        if frame is None:
            continue
        if mode == "gray":
            frame = to_gray3(frame)
        n_ok += 1
        t0 = time.time()
        dets = run_model_on_image(sess, iname, imgsz, frame, n_classes, drone_cls)
        lat.append(time.time() - t0)
        d25 = [d for d in dets if d["score"] >= DEPLOY_CONF]
        fired = len(d25) > 0
        n_fired += 1 if fired else 0
        n_fires_total += len(d25)
        top = max((d["score"] for d in d25), default=None)
        if top is not None:
            top_scores.append(top)
        all_fire_scores.extend(d["score"] for d in d25)
        perimage.append({
            "model": model_key, "mode": mode, "image": os.path.basename(ip),
            "n_det25": len(d25), "top_score": round(top, 4) if top is not None else "",
        })
        if progress_fh and k % 50 == 0:
            progress_fh.write(f"{_dt.datetime.utcnow().isoformat()} {tag} "
                              f"{model_key}/{mode} {k}/{len(img_paths)}\n")
            progress_fh.flush()

    hist = {name: sum(1 for s in top_scores if _conf_bin(s) == name)
            for name in CONF_BIN_NAMES}
    ts = np.array(top_scores, dtype=float) if top_scores else np.array([])
    summary = {
        "model": model_key, "mode": mode, "n_images": n_ok,
        "false_fire_rate@25": round(n_fired / max(n_ok, 1), 4),
        "n_fired": n_fired,
        "fires_per_image": round(n_fires_total / max(n_ok, 1), 3),
        "top_score_mean": round(float(ts.mean()), 4) if ts.size else None,
        "top_score_median": round(float(np.median(ts)), 4) if ts.size else None,
        "top_score_p25": round(float(np.percentile(ts, 25)), 4) if ts.size else None,
        "top_score_p75": round(float(np.percentile(ts, 75)), 4) if ts.size else None,
        "top_score_min": round(float(ts.min()), 4) if ts.size else None,
        "top_score_max": round(float(ts.max()), 4) if ts.size else None,
        "latency_ms_cpu": round(1000 * float(np.mean(lat)), 1) if lat else -1,
    }
    for name in CONF_BIN_NAMES:
        summary[f"hist[{name}]"] = hist[name]
    return summary, perimage, top_scores


# ---------------------------------------------------------------- self-test

def self_test():
    """1) leak-derivation regex on a tiny synthetic manifest with known
          answers (BT/BTR/BV must be told apart).
       2) collect_bird_images() runs and returns disjoint kept/excluded sets
          that partition the real DVB test BT folder (if present).
       3) tiny end-to-end eval (6 images) through whichever model exists.
    Exits 0 iff all pass."""
    ok = True
    import tempfile

    # -- (1) leak regex on a synthetic manifest_all.csv
    rows = [
        {"uid": "dvb_bird_all_BT__3_", "source": "dvb_bird"},      # DVB-test origin -> LEAK
        {"uid": "dvb_bird_all_BT__21_", "source": "dvb_bird"},     # DVB-test origin -> LEAK
        {"uid": "dvb_bird_all_BTR__1002_", "source": "dvb_bird"},  # DVB-train origin -> NOT a leak
        {"uid": "dvb_bird_all_BV__50_", "source": "dvb_bird"},     # DVB-valid origin -> NOT a leak
        {"uid": "dut_all_00001", "source": "dut"},                 # different source -> ignored
    ]
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["uid", "source"])
        w.writeheader()
        w.writerows(rows)
        mpath = fh.name
    leaked = leaked_dvb_bird_test_basenames(mpath)
    os.unlink(mpath)
    expect = {"BT (3).jpg", "BT (21).jpg"}
    if leaked != expect:
        print(f"SELF-TEST FAIL: leak regex {leaked} != {expect}")
        ok = False

    # -- (2) collect_bird_images on the real corpus (if present): kept + excluded partition all
    if os.path.isdir(os.path.join(DVB_ROOT, "test", "images")):
        kept, excluded = collect_bird_images(exclude_leaked=True)
        all_bt, _ = collect_bird_images(exclude_leaked=False)
        if len(kept) + len(excluded) != len(all_bt):
            print(f"SELF-TEST FAIL: kept({len(kept)})+excluded({len(excluded)}) "
                  f"!= all({len(all_bt)})")
            ok = False
        if any(os.path.basename(p) in excluded for p in kept):
            print("SELF-TEST FAIL: kept/excluded overlap")
            ok = False
        print(f"collect_bird_images: {len(all_bt)} total BT-test images, "
              f"{len(excluded)} excluded as train-leaked, {len(kept)} clean held-out")
    else:
        print("(DVB corpus not present locally -- skipping collect_bird_images check)")

    # -- (3) tiny end-to-end eval through the first available model
    model = next((m for m in ("n_mono", "n_color", "v2_deployed")
                  if os.path.exists(MODELS[m][0])), None)
    if model is None:
        print("SELF-TEST FAIL: no model weights found")
        ok = False
    else:
        kept, _ = collect_bird_images(exclude_leaked=True)
        sample = kept[:6] if kept else []
        if not sample:
            print("SELF-TEST FAIL: no bird images available for tiny eval")
            ok = False
        else:
            s, pi, _ = eval_bird_false_fire(model, "gray", sample)
            if s["n_images"] != len(sample) or len(pi) != len(sample) \
                    or not (0 <= s["false_fire_rate@25"] <= 1):
                print(f"SELF-TEST FAIL: tiny eval {s}")
                ok = False
            else:
                print(f"tiny eval OK on {model}/gray (n={len(sample)}): {json.dumps(s)}")
    print("SELF-TEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--models", nargs="+", default=["n_mono"])
    ap.add_argument("--modes", nargs="+", default=["gray"],
                    help="color modes to feed the model: gray and/or color")
    ap.add_argument("--include-leaked", action="store_true",
                    help="ALSO score the leak-included full BT-test folder "
                         "(reported for transparency, never as the verdict)")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        sys.exit(self_test())

    tag = args.tag or _dt.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    os.makedirs(OUT_DIR, exist_ok=True)
    prog = open(os.path.join(OUT_DIR, f"eval_bird_progress_{tag}.log"), "a")

    kept, excluded = collect_bird_images(exclude_leaked=True)
    print(f"DVB test BT bird images: {len(kept) + len(excluded)} total, "
          f"{len(excluded)} excluded (train-leaked via dvb_bird pooling), "
          f"{len(kept)} clean held-out -> this is the headline eval set")
    prog.write(f"clean={len(kept)} excluded_leaked={len(excluded)}\n")

    summaries, perimage = [], []
    for m in args.models:
        if not os.path.exists(MODELS[m][0]):
            print(f"SKIP {m}: weights not found at {MODELS[m][0]}")
            continue
        for mode in args.modes:
            t0 = time.time()
            s, pi, _ = eval_bird_false_fire(m, mode, kept, prog, tag)
            s["subset"] = "clean_holdout"
            s["wall_s"] = round(time.time() - t0, 1)
            summaries.append(s)
            perimage.extend(pi)
            print(json.dumps(s))
            prog.write(json.dumps(s) + "\n")
            prog.flush()
            if args.include_leaked:
                all_bt = kept + [p for p in glob.glob(
                    os.path.join(DVB_ROOT, "test", "images", "BT*"))
                    if os.path.basename(p) in excluded]
                t0 = time.time()
                s2, pi2, _ = eval_bird_false_fire(m, mode, all_bt, prog, tag + "_incl_leak")
                s2["subset"] = "leak_included_DO_NOT_USE_AS_VERDICT"
                s2["wall_s"] = round(time.time() - t0, 1)
                summaries.append(s2)
                print(json.dumps(s2))
                prog.write(json.dumps(s2) + "\n")
                prog.flush()

    if summaries:
        sp = os.path.join(OUT_DIR, f"eval_bird_negatives_{tag}.csv")
        with open(sp, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(summaries[0].keys()))
            w.writeheader()
            w.writerows(summaries)
        pp = os.path.join(OUT_DIR, f"eval_bird_negatives_perimage_{tag}.csv")
        with open(pp, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(perimage[0].keys()))
            w.writeheader()
            w.writerows(perimage)
        print(f"wrote {sp}\nwrote {pp}")


if __name__ == "__main__":
    main()
