#!/usr/bin/env python3
"""EVALUATE phase — honest held-out-source eval of the nn_tier `n-color` A/B
arm, in COLOR, on the SAME source-disjoint split as eval_n_mono.py.

WHAT THIS IS (mono-vs-color camera decision, residual bird risk, 2026-07-21)
-----------------------------------------------------------------------------
Copy of eval_n_mono.py's eval machinery (same scorer, same bins, same
gate — box_scoring.box_hits_gt, read-only import), with ONE change: images
are read from the manifest's `color_img` column (true RGB) instead of
`gray_img`, so a COLOR-native model (n-color) is scored in ITS OWN training
modality, exactly as n-mono is scored in gray in eval_n_mono.py. Labels are
IDENTICAL between the color/ and gray/ trees (verified: same box coords,
only pixel content differs — prepare_nn_tier_dataset.py emits one label set
per uid, reused for both variants), so nothing about the GT side changes.

Eval set = the Dataset phase's SOURCE-DISJOINT TEST split (n=4175 images,
4315 GT boxes, 469 drone-free negatives) — same split eval_n_mono.py scores,
byte-identical membership (dataset.yaml / dataset_color.yaml differ only in
which image tree they point at). dut = a whole NEVER-SEEN source; nps = held-
out FLIGHT clips; plates = held-out photographers' desert negatives.

THIS SCRIPT DOES NOT EDIT eval_n_mono.py (project rule) — it is a full copy
with the color swap, kept as its own file so the two arms' eval code paths
are provably identical except for the pixel-source column.

HONESTY / anti-mirage (CLAUDE.md, ADR-0061): frame metrics do NOT certify a
seeker; this ranks the color-native fine-tune against the gray-native one on
held-out real sources, for the mono-vs-color camera decision only.

Run (CPU, read-only seeker venv):
  .venv-seeker/bin/python scripts/seeker/nn_tier/eval_n_color.py \\
      --models n_color --tag <tag>
Self-test (tiny 6-image set through the first available model; exits 0/1):
  .venv-seeker/bin/python scripts/seeker/nn_tier/eval_n_color.py --self-test
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import glob
import json
import os
import random
import sys
import time

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))          # scripts/seeker/nn_tier
SEEKER = os.path.abspath(os.path.join(HERE, ".."))         # scripts/seeker
REPO = os.path.abspath(os.path.join(SEEKER, "..", ".."))   # repo root
sys.path.insert(0, SEEKER)
sys.path.insert(0, HERE)

from v3_onnx_infer import load_session, DEPLOY_CONF, iou            # noqa: E402
from baseline_eval import (run_model_on_image, ap50)                # noqa: E402
import box_scoring                                                  # noqa: E402

WEIGHTS = os.path.join(SEEKER, "weights")
DATA_ROOT = os.path.join(SEEKER, "data", "nn_tier")
OUT_DIR = os.path.join(REPO, "logs", "nn_tier")

# name -> (onnx path, drone class id, n classes)
MODELS = {
    "n_color":       (os.path.join(WEIGHTS, "nn_tier", "n-color.onnx"), 0, 1),
    "n_mono":        (os.path.join(WEIGHTS, "nn_tier", "n-mono.onnx"), 0, 1),   # cross-ref, color-fed (OOD for n_mono)
    "v2_deployed":   (os.path.join(WEIGHTS, "drone_finetuned_quad_v2.onnx"), 0, 1),
}

SCALE_BINS = [(0, 12), (12, 24), (24, 40), (40, 80), (80, 10_000)]
SCALE_NAMES = ["0-12px", "12-24px", "24-40px", "40-80px", "80+px"]
POS_BINS = ["center<0.33", "mid0.33-0.66", "edge>0.66"]


# ---------------------------------------------------------------- dataset

def load_manifest(split="test"):
    """manifest_<split>.csv rows with per-image source/group/paths."""
    rows = []
    with open(os.path.join(DATA_ROOT, f"manifest_{split}.csv")) as fh:
        for r in csv.DictReader(fh):
            rows.append(r)
    return rows


def load_gt_boxes(label_path, W, H):
    """YOLO bbox label file -> [(cx,cy,w,h)] px, class 0 only."""
    out = []
    if not label_path or not os.path.isfile(label_path):
        return out
    with open(label_path) as fh:
        for ln in fh.read().splitlines():
            p = ln.split()
            if len(p) < 5 or int(float(p[0])) != 0:
                continue
            ncx, ncy, nw, nh = (float(v) for v in p[1:5])
            out.append((ncx * W, ncy * H, nw * W, nh * H))
    return out


def stratified_subsample(rows, caps, seed=0):
    """Deterministic per-source cap. caps: {source: n or None(=all)}."""
    by_src = {}
    for r in rows:
        by_src.setdefault(r["source"], []).append(r)
    keep = []
    rng = random.Random(seed)
    for src, lst in sorted(by_src.items()):
        lst = sorted(lst, key=lambda r: r["uid"])
        cap = caps.get(src)
        if cap is not None and len(lst) > cap:
            lst = rng.sample(lst, cap)
        keep.extend(lst)
    return sorted(keep, key=lambda r: r["uid"])


# ---------------------------------------------------------------- scoring

def match_boxes(dets, gts, iou_thr=0.5):
    """Greedy score-order IoU matching.
    Returns (scored [(score,is_tp)...], gt_matched_conf [best matched det
    score per gt, or None])."""
    taken = [None] * len(gts)
    scored = []
    for d in sorted(dets, key=lambda x: -x["score"]):
        db = (d["cx"], d["cy"], d["w"], d["h"])
        best_i, best_v = -1, iou_thr
        for i, g in enumerate(gts):
            if taken[i] is not None:
                continue
            v = iou(db, g)
            if v >= best_v:
                best_i, best_v = i, v
        if best_i >= 0:
            taken[best_i] = d["score"]
            scored.append((d["score"], True))
        else:
            scored.append((d["score"], False))
    return scored, taken


def pos_bin_of(gcx, gcy, W, H):
    r = max(abs(gcx - W / 2) / (W / 2), abs(gcy - H / 2) / (H / 2))
    return POS_BINS[0] if r < 0.33 else (POS_BINS[1] if r < 0.66 else POS_BINS[2])


def scale_bin_of(w1280):
    for name, (lo, hi) in zip(SCALE_NAMES, SCALE_BINS):
        if lo <= w1280 < hi:
            return name
    return SCALE_NAMES[-1]


def gate_hit(dets25, gcx, gcy, gw, gh, W, H):
    """The fixed box_scoring.box_hits_gt gate with declared proxy intrinsics
    for real imagery (see eval_n_mono.py module docstring POSITION GATE NOTE
    -- identical proxy used here)."""
    s = W / 1280.0
    return box_scoring.box_hits_gt(
        [(d["score"], d["cx"], d["cy"], d["w"], d["h"]) for d in dets25],
        gcx, gcy, gw, gh,
        tol=15.0 * s,
        fx=box_scoring.FX * s, fy=box_scoring.FX * s,
        cx=W / 2.0, cy=H / 2.0,
        gt_scale=1.0, offaxis_aware=True, centre_lag_px=0.0)


# ---------------------------------------------------------------- eval core

def eval_model_on_rows(model_key, rows, progress_fh=None, tag=""):
    path, drone_cls, n_classes = MODELS[model_key]
    sess, iname, imgsz = load_session(path, intra_threads=8)
    all_scored, n_gt = [], 0
    tp25 = fp25 = fn25 = 0
    neg_stats = {}           # source -> [n_neg, n_fired, n_fires]
    scale_stats = {n: [0, 0] for n in SCALE_NAMES}     # matched@25 / total
    pos_stats = {n: [0, 0, 0] for n in POS_BINS}       # iou-hit / gate-hit / total
    lat, perimage = [], []
    for k, r in enumerate(rows):
        ip = os.path.join(DATA_ROOT, r["color_img"])                # COLOR, not gray
        frame = cv2.imread(ip)
        if frame is None:
            continue
        H, W = frame.shape[:2]
        lp = os.path.join(DATA_ROOT, r["color_img"]
                          .replace("images/", "labels/")).rsplit(".", 1)[0] + ".txt"
        gts = load_gt_boxes(lp, W, H)
        t0 = time.time()
        dets = run_model_on_image(sess, iname, imgsz, frame, n_classes, drone_cls)
        lat.append(time.time() - t0)
        scored, gt_match = match_boxes(dets, gts)
        all_scored.extend(scored)
        n_gt += len(gts)
        dets25 = [d for d in dets if d["score"] >= DEPLOY_CONF]
        s25 = [(s, tp) for s, tp in scored if s >= DEPLOY_CONF]
        tp_i = sum(1 for _, tp in s25 if tp)
        fp_i = sum(1 for _, tp in s25 if not tp)
        tp25 += tp_i
        fp25 += fp_i
        fn25 += max(0, len(gts) - tp_i)
        if not gts:
            st = neg_stats.setdefault(r["source"], [0, 0, 0])
            st[0] += 1
            st[1] += 1 if dets25 else 0
            st[2] += len(dets25)
        for (gcx, gcy, gw, gh), mconf in zip(gts, gt_match):
            hit25 = mconf is not None and mconf >= DEPLOY_CONF
            sb = scale_bin_of(gw * 1280.0 / W)
            scale_stats[sb][0] += 1 if hit25 else 0
            scale_stats[sb][1] += 1
            pb = pos_bin_of(gcx, gcy, W, H)
            pos_stats[pb][0] += 1 if hit25 else 0
            pos_stats[pb][1] += 1 if gate_hit(dets25, gcx, gcy, gw, gh, W, H) else 0
            pos_stats[pb][2] += 1
        perimage.append({"model": model_key, "uid": r["uid"], "source": r["source"],
                         "group": r["group"], "n_gt": len(gts),
                         "n_det25": len(dets25), "tp25": tp_i, "fp25": fp_i})
        if progress_fh and k % 100 == 0:
            progress_fh.write(f"{_dt.datetime.utcnow().isoformat()} {tag} "
                              f"{model_key} {k}/{len(rows)}\n")
            progress_fh.flush()
    prec = tp25 / max(tp25 + fp25, 1)
    rec = tp25 / max(tp25 + fn25, 1)
    n_neg = sum(v[0] for v in neg_stats.values())
    n_neg_fired = sum(v[1] for v in neg_stats.values())
    summary = {
        "model": model_key, "subset_n": len(rows), "n_gt_boxes": n_gt,
        "ap50": round(ap50(all_scored, n_gt), 4),
        "precision@25": round(prec, 4), "recall@25": round(rec, 4),
        "f1@25": round(2 * prec * rec / max(prec + rec, 1e-9), 4),
        "neg_images": n_neg,
        "false_fire_rate@25": round(n_neg_fired / max(n_neg, 1), 4),
        "false_fires_per_neg_img": round(
            sum(v[2] for v in neg_stats.values()) / max(n_neg, 1), 3),
        "latency_ms_cpu": round(1000 * float(np.mean(lat)), 1) if lat else -1,
    }
    breakdown = {
        "neg_by_source": {s: {"n": v[0], "fired": v[1], "fires": v[2]}
                          for s, v in sorted(neg_stats.items())},
        "recall25_by_scale": {n: {"matched": v[0], "total": v[1],
                                  "recall": round(v[0] / v[1], 4) if v[1] else None}
                              for n, v in scale_stats.items()},
        "by_position": {n: {"iou_hit25": v[0], "gate_hit25": v[1], "total": v[2],
                            "recall_iou": round(v[0] / v[2], 4) if v[2] else None,
                            "recall_gate": round(v[1] / v[2], 4) if v[2] else None}
                        for n, v in pos_stats.items()},
    }
    return summary, breakdown, perimage


def eval_per_source(model_key, rows, progress_fh=None, tag=""):
    """Split rows by source and score each independently (dut / nps / plates)."""
    out = {}
    for src in sorted({r["source"] for r in rows}):
        sub = [r for r in rows if r["source"] == src]
        s, b, _ = eval_model_on_rows(model_key, sub, progress_fh, f"{tag}:{src}")
        out[src] = {"summary": s, "breakdown": b}
    return out


# ---------------------------------------------------------------- self-test

def self_test():
    """Scorer math with known answers + a 6-image tiny eval through the first
    available model (n_color if exported, else the deployed v2). Exits 0/1."""
    ok = True
    gts = [(50, 50, 20, 20)]
    dets = [{"cx": 51, "cy": 50, "w": 20, "h": 20, "score": 0.9},
            {"cx": 200, "cy": 200, "w": 20, "h": 20, "score": 0.3}]
    scored, taken = match_boxes(dets, gts)
    if scored != [(0.9, True), (0.3, False)] or taken != [0.9]:
        print(f"SELF-TEST FAIL: match_boxes {scored} {taken}")
        ok = False
    if abs(ap50(scored, 2) - 0.5) > 1e-6:
        print("SELF-TEST FAIL: ap50")
        ok = False
    if scale_bin_of(11.9) != "0-12px" or scale_bin_of(40) != "40-80px":
        print("SELF-TEST FAIL: scale bins")
        ok = False
    if (pos_bin_of(640, 480, 1280, 960) != POS_BINS[0]
            or pos_bin_of(40, 480, 1280, 960) != POS_BINS[2]):
        print("SELF-TEST FAIL: pos bins")
        ok = False
    if not gate_hit([{"cx": 640, "cy": 480, "w": 30, "h": 30, "score": .9}],
                    640, 480, 28, 28, 1280, 960):
        print("SELF-TEST FAIL: gate_hit centred")
        ok = False
    if gate_hit([{"cx": 640, "cy": 480, "w": 376, "h": 368, "score": .9}],
                640, 480, 60, 60, 1280, 960):
        print("SELF-TEST FAIL: gate_hit blob guard")
        ok = False
    # color_img manifest column must exist and point at real files
    rows = load_manifest("test")[:6]
    if not all("color_img" in r for r in rows):
        print("SELF-TEST FAIL: manifest missing color_img column")
        ok = False
    model = next((m for m in ("n_color", "n_mono", "v2_deployed")
                  if os.path.exists(MODELS[m][0])), None)
    if model is None:
        print("SELF-TEST FAIL: no model weights found")
        ok = False
    else:
        s, b, pi = eval_model_on_rows(model, rows)
        if s["subset_n"] != 6 or len(pi) != 6 or not (0 <= s["recall@25"] <= 1):
            print(f"SELF-TEST FAIL: tiny eval {s}")
            ok = False
        else:
            print(f"tiny eval OK on {model} (color): {json.dumps(s)}")
    print("SELF-TEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--models", nargs="+", default=["n_color"])
    ap.add_argument("--subsample", action="store_true",
                    help="stratified per-source caps (dut 300 / nps 300 / "
                         "plates 200, seed 0) for slow models")
    ap.add_argument("--per-source", action="store_true",
                    help="also score each source (dut/nps/plates) separately")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        sys.exit(self_test())

    tag = args.tag or _dt.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    os.makedirs(OUT_DIR, exist_ok=True)
    prog = open(os.path.join(OUT_DIR, f"eval_n-color_progress_{tag}.log"), "a")

    rows = load_manifest("test")
    subset = "full"
    if args.subsample:
        rows = stratified_subsample(
            rows, {"dut": 300, "nps": 300, "plates": 200}, seed=0)
        subset = "subsample-seed0"
    print(f"test rows: {len(rows)} ({subset})")

    summaries, perimage, extras = [], [], {"subset": subset, "tag": tag}
    for m in args.models:
        if not os.path.exists(MODELS[m][0]):
            print(f"SKIP {m}: weights not found at {MODELS[m][0]}")
            continue
        t0 = time.time()
        s, b, pi = eval_model_on_rows(m, rows, prog, tag)
        s["subset"] = subset
        s["wall_s"] = round(time.time() - t0, 1)
        summaries.append(s)
        perimage.extend(pi)
        extras[m] = {"breakdown": b}
        if args.per_source:
            extras[m]["per_source"] = eval_per_source(m, rows, prog, tag)
        print(json.dumps(s))
        prog.write(json.dumps(s) + "\n")
        prog.flush()

    if summaries:
        sp = os.path.join(OUT_DIR, f"eval_n-color_{tag}.csv")
        with open(sp, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(summaries[0].keys()))
            w.writeheader()
            w.writerows(summaries)
        pp = os.path.join(OUT_DIR, f"eval_n-color_perimage_{tag}.csv")
        with open(pp, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(perimage[0].keys()))
            w.writeheader()
            w.writerows(perimage)
        jp = os.path.join(OUT_DIR, f"eval_n-color_breakdown_{tag}.json")
        with open(jp, "w") as fh:
            json.dump(extras, fh, indent=1)
        print(f"wrote {sp}\nwrote {pp}\nwrote {jp}")


if __name__ == "__main__":
    main()
