#!/usr/bin/env python3
"""EVALUATE phase — honest held-out-source scoring of `n-mono-aug` (the heavy-
augmentation yolo11n fine-tune) + the comparison bar, on the nn_tier TEST split.

WHAT THIS MEASURES (and the anti-mirage rules it obeys)
-------------------------------------------------------
* Eval corpus = the Dataset phase's SOURCE-DISJOINT test split
  (`scripts/seeker/data/nn_tier/manifest_test.csv`): `dut` is a whole
  never-seen-in-training SOURCE pinned to test, `nps` test rows are whole
  held-out CLIPS, `plates` test rows are held-out photographers. NEVER random
  frames (ADR-0061 — the v3/rebal NULLs both aced random-frame eval and
  failed in flight). Nothing here was trained on by any nn_tier arm.
* GRAYSCALE only — the deployment OV9281 is monochrome; the gray/ corpus is
  the deployment track (camera_paper_check.md item 4).
* Metrics: AP50 (IoU 0.5 greedy match, all-point interpolation), recall /
  precision @ the deployed operating point conf 0.25 (v3_onnx_infer.
  DEPLOY_CONF), false-fire density on drone-FREE images, per-frame CPU
  latency (this box, NOT a Pi number).
* SCALE + POSITION recall breakdowns use the ONE fixed offline hit gate
  `scripts/seeker/box_scoring.box_hits_gt` (ADR-0076 add #18k; READ-ONLY
  here). Intrinsics for its off-axis widening are the sim mono_cam values
  scaled to each image's width (real-media intrinsics are unknown —
  approximate, disclosed); gt_scale=1.0 because these are real labeled boxes,
  not extent-derived sim gt.
* Scale bins are @1280-equivalent px (box w * 1280 / img_w), matching the
  dataset report + the viewpoint spec's 6–20 m terminal band proxy
  (~7–36 px @1280 spans the 0-12/12-24/24-40 bins).
* CLUTTER mode: the transfer-bet footage segments
  (`scripts/seeker/data/transfer_bet_footage/`, docs/transfer_bet_kill_test.md)
  run in grayscale for a real false-fire density read on desert-ish clutter
  (fields/sky/birds; NOT true Central-Oregon high desert — disclosed).
* Big 1280-input baselines (yolo11x_mit, flyingobj_mit) are evaluated on a
  DETERMINISTIC seed-0 subsample (500 pos + 200 neg) for CPU-time reasons;
  n is reported next to every number. 640-input models run the FULL split.
* Honesty: labels are used for SCORING ONLY; frame metrics rank models, they
  certify no seeker (ADR-0061); numbers trace to the CSVs this writes.

Run (inference venv):
  .venv-seeker/bin/python scripts/seeker/nn_tier/eval_n_mono_aug.py \
      --models n-mono-aug v2_deployed --clutter --tag aug1
Self-test (tiny set, <1 min, exits 0/1):
  .venv-seeker/bin/python scripts/seeker/nn_tier/eval_n_mono_aug.py --self-test
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

from v3_onnx_infer import load_session, DEPLOY_CONF                    # noqa: E402
from box_scoring import box_hits_gt, FX as SIM_FX                      # noqa: E402
from baseline_eval import (run_model_on_image, to_gray3, load_labels,  # noqa: E402
                           ap50, match_image)

WEIGHTS = os.path.join(SEEKER, "weights")
NNT_WEIGHTS = os.path.join(WEIGHTS, "nn_tier")
DATA = os.path.join(SEEKER, "data", "nn_tier")
GRAY = os.path.join(DATA, "gray")
CLUTTER_DIR = os.path.join(SEEKER, "data", "transfer_bet_footage")
OUT_DIR = os.path.join(REPO, "logs", "nn_tier")

# name -> (onnx path, drone class id in the model head, n classes, big-input?)
MODELS = {
    "n-mono-aug":    (os.path.join(NNT_WEIGHTS, "n-mono-aug.onnx"), 0, 1, False),
    "n-mono":        (os.path.join(NNT_WEIGHTS, "n-mono.onnx"), 0, 1, False),
    "s-mono":        (os.path.join(NNT_WEIGHTS, "s-mono.onnx"), 0, 1, False),
    "v2_deployed":   (os.path.join(WEIGHTS, "drone_finetuned_quad_v2.onnx"), 0, 1, False),
    "yolo11x_mit":   (os.path.join(WEIGHTS, "drone_yolo11x_1280.onnx"), 0, 1, True),
    "flyingobj_mit": (os.path.join(WEIGHTS, "flying_objects_yolov8m.onnx"), 0, 5, True),
}

SCALE_BINS = [(0, 12), (12, 24), (24, 40), (40, 80), (80, 10 ** 9)]
POS_BINS = [("center", 0.0, 1 / 3), ("mid", 1 / 3, 2 / 3), ("edge", 2 / 3, 1.01)]
CMP_POS, CMP_NEG, CMP_SEED = 500, 200, 0


def scale_bin(w1280):
    for lo, hi in SCALE_BINS:
        if lo <= w1280 < hi:
            return f"{lo}-{hi if hi < 10**9 else '+'}px"
    return "?"


def pos_bin(ncx, ncy):
    off = max(abs(ncx - 0.5), abs(ncy - 0.5)) * 2.0   # 0 center .. 1 edge
    for name, lo, hi in POS_BINS:
        if lo <= off < hi:
            return name
    return "edge"


def read_manifest(path):
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def sample_rows(rows, full):
    """full -> all rows; else deterministic seed-0 subsample (CMP_POS+CMP_NEG)."""
    if full:
        return rows, "full"
    pos = [r for r in rows if int(r["n_boxes"]) > 0]
    neg = [r for r in rows if int(r["n_boxes"]) == 0]
    rng = random.Random(CMP_SEED)
    rng.shuffle(pos)
    rng.shuffle(neg)
    return pos[:CMP_POS] + neg[:CMP_NEG], f"cmp{CMP_POS}+{CMP_NEG}"


def eval_model(name, rows, progress=None, conf=DEPLOY_CONF, threads=6):
    """One model over manifest rows (gray corpus). Returns (summary, perimage,
    perbox). perbox rows carry the box_hits_gt verdict + scale/position bins."""
    path, dcls, ncls, _big = MODELS[name]
    sess, iname, imgsz = load_session(path, intra_threads=threads)
    all_scored, n_gt = [], 0
    tp = fp = fn = 0
    neg_imgs = neg_fired = 0
    neg_fp_count = 0
    lat, perimage, perbox = [], [], []
    hits_gate = boxes_gate = 0
    for k, r in enumerate(rows):
        ip = os.path.join(DATA, r["gray_img"])
        frame = cv2.imread(ip)
        if frame is None:
            continue
        frame = to_gray3(frame)                     # idempotent on gray corpus
        H, W = frame.shape[:2]
        lp = os.path.join(GRAY, "labels", "test", r["uid"] + ".txt")
        gts = load_labels(lp, W, H, {0})
        t0 = time.time()
        dets = run_model_on_image(sess, iname, imgsz, frame, ncls, dcls)
        lat.append(time.time() - t0)
        scored = match_image(dets, gts)
        all_scored.extend(scored)
        n_gt += len(gts)
        s25 = [(s, is_tp) for s, is_tp in scored if s >= conf]
        tp_i = sum(1 for _, t in s25 if t)
        fp_i = len(s25) - tp_i
        tp += tp_i
        fp += fp_i
        fn += max(0, len(gts) - tp_i)
        if not gts:
            neg_imgs += 1
            neg_fp_count += len(s25)
            if s25:
                neg_fired += 1
        # ---- fixed-gate (box_scoring.box_hits_gt) per-GT-box verdicts ----
        det25 = [(d["score"], d["cx"], d["cy"], d["w"], d["h"])
                 for d in dets if d["score"] >= conf]
        f_eff = SIM_FX * (W / 1280.0)   # sim intrinsics scaled to this width
        for (gcx, gcy, gw, gh) in gts:
            hit = box_hits_gt(det25, gcx, gcy, gw, gh, tol=15,
                              fx=f_eff, fy=f_eff, cx=W / 2.0, cy=H / 2.0,
                              gt_scale=1.0, offaxis_aware=True)
            boxes_gate += 1
            hits_gate += int(hit)
            perbox.append({
                "model": name, "uid": r["uid"], "source": r["source"],
                "w1280": round(gw * 1280.0 / W, 1),
                "scale_bin": scale_bin(gw * 1280.0 / W),
                "pos_bin": pos_bin(gcx / W, gcy / H),
                "hit": int(hit),
            })
        perimage.append({"model": name, "uid": r["uid"], "source": r["source"],
                         "n_gt": len(gts), "n_det25": len(s25),
                         "tp25": tp_i, "fp25": fp_i})
        if progress and k % 200 == 0:
            progress.write(f"{_dt.datetime.now().isoformat()} {name} "
                           f"{k}/{len(rows)}\n")
            progress.flush()
    prec = tp / max(tp + fp, 1)
    rec = tp / max(tp + fn, 1)
    summary = {
        "model": name, "n_images": len(rows), "n_gt_boxes": n_gt,
        "neg_images": neg_imgs,
        "ap50": round(ap50(all_scored, n_gt), 4),
        "recall@25": round(rec, 4), "precision@25": round(prec, 4),
        "f1@25": round(2 * prec * rec / max(prec + rec, 1e-9), 4),
        "gate_recall@25": round(hits_gate / max(boxes_gate, 1), 4),
        "false_fire_rate@25": round(neg_fired / max(neg_imgs, 1), 4),
        "neg_fp_per_image": round(neg_fp_count / max(neg_imgs, 1), 3),
        "latency_ms_cpu": round(1000 * float(np.mean(lat)), 1) if lat else -1,
        "model_size_mb": round(os.path.getsize(MODELS[name][0]) / 2 ** 20, 1),
    }
    return summary, perimage, perbox


def breakdown(perbox):
    """perbox rows -> [(model, dim, bin, n, hit, recall)] using the fixed gate."""
    out = []
    for dim, key in (("scale", "scale_bin"), ("position", "pos_bin"),
                     ("source", "source")):
        agg = {}
        for r in perbox:
            k = (r["model"], r[key])
            n, h = agg.get(k, (0, 0))
            agg[k] = (n + 1, h + r["hit"])
        for (m, b), (n, h) in sorted(agg.items()):
            out.append({"model": m, "dim": dim, "bin": b, "n_boxes": n,
                        "n_hit": h, "gate_recall": round(h / max(n, 1), 4)})
    return out


def eval_clutter(name, conf=DEPLOY_CONF, threads=6):
    """Transfer-bet footage segments, GRAYSCALE: per-segment fire density."""
    path, dcls, ncls, _big = MODELS[name]
    sess, iname, imgsz = load_session(path, intra_threads=threads)
    rows = []
    for seg in sorted(os.listdir(CLUTTER_DIR)):
        seg_dir = os.path.join(CLUTTER_DIR, seg)
        if not os.path.isdir(seg_dir):
            continue
        frames = sorted(f for f in glob.glob(os.path.join(seg_dir, "*.png"))
                        if not os.path.basename(f).startswith("_raw"))
        if not frames:
            continue
        fired = 0
        n_fires = 0
        for fp_ in frames:
            frame = cv2.imread(fp_)
            if frame is None:
                continue
            dets = run_model_on_image(sess, iname, imgsz, to_gray3(frame),
                                      ncls, dcls)
            d25 = [d for d in dets if d["score"] >= conf]
            fired += bool(d25)
            n_fires += len(d25)
        rows.append({"model": name, "segment": seg,
                     "label": "drone" if seg.startswith("pos") else "none",
                     "n_frames": len(frames),
                     "fire_density@25": round(fired / max(len(frames), 1), 3),
                     "fires_per_frame": round(n_fires / max(len(frames), 1), 2)})
    return rows


def write_csv(path, rows):
    if not rows:
        return
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {path}")


# ---------------------------------------------------------------- self-test

def self_test():
    ok = True
    # 1) bin logic
    if scale_bin(11.9) != "0-12px" or scale_bin(24) != "24-40px" \
            or scale_bin(500) != "80-+px":
        print("SELF-TEST FAIL: scale_bin")
        ok = False
    if pos_bin(0.5, 0.5) != "center" or pos_bin(0.05, 0.5) != "edge" \
            or pos_bin(0.5, 0.72) != "mid":
        print(f"SELF-TEST FAIL: pos_bin")
        ok = False
    # 2) fixed gate sanity (box_scoring.box_hits_gt, read-only import):
    #    centred det on centred gt -> hit; far det -> miss; giant blob -> miss.
    gt = (320.0, 240.0, 30.0, 30.0)
    hit = box_hits_gt([(0.9, 322.0, 241.0, 40.0, 40.0)], *gt, tol=15,
                      fx=SIM_FX * 0.5, fy=SIM_FX * 0.5, cx=320, cy=240)
    miss = box_hits_gt([(0.9, 100.0, 100.0, 40.0, 40.0)], *gt, tol=15,
                       fx=SIM_FX * 0.5, fy=SIM_FX * 0.5, cx=320, cy=240)
    blob = box_hits_gt([(0.9, 322.0, 241.0, 300.0, 300.0)], *gt, tol=15,
                       fx=SIM_FX * 0.5, fy=SIM_FX * 0.5, cx=320, cy=240)
    if not hit or miss or blob:
        print(f"SELF-TEST FAIL: box_hits_gt gate {hit} {miss} {blob}")
        ok = False
    # 3) deterministic subsample
    rows = [{"n_boxes": str(i % 2), "uid": str(i)} for i in range(1000)]
    s1, t1 = sample_rows(rows, full=False)
    s2, _ = sample_rows(rows, full=False)
    if [r["uid"] for r in s1] != [r["uid"] for r in s2]:
        print("SELF-TEST FAIL: subsample not deterministic")
        ok = False
    if t1 != f"cmp{CMP_POS}+{CMP_NEG}":
        print("SELF-TEST FAIL: subsample tag")
        ok = False
    # 4) tiny end-to-end run on real test-split images through a real ONNX net
    #    (v2_deployed always exists; n-mono-aug used instead when present).
    man = os.path.join(DATA, "manifest_test.csv")
    net = "n-mono-aug" if os.path.exists(MODELS["n-mono-aug"][0]) else "v2_deployed"
    if os.path.exists(man) and os.path.exists(MODELS[net][0]):
        rows = read_manifest(man)
        pos = [r for r in rows if int(r["n_boxes"]) > 0][:4]
        neg = [r for r in rows if int(r["n_boxes"]) == 0][:2]
        s, pi, pb = eval_model(net, pos + neg, threads=2)
        if s["n_images"] != 6 or len(pi) != 6 or s["n_gt_boxes"] < 4:
            print(f"SELF-TEST FAIL: tiny eval {s}")
            ok = False
        bd = breakdown(pb)
        if pb and not bd:
            print("SELF-TEST FAIL: breakdown empty")
            ok = False
        print(f"tiny-eval [{net}] on 6 real held-out images: {json.dumps(s)}")
    else:
        print(f"SELF-TEST FAIL: missing manifest or weights for {net}")
        ok = False
    print("SELF-TEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--models", nargs="+", default=["n-mono-aug"])
    ap.add_argument("--clutter", action="store_true",
                    help="also run the transfer-bet clutter segments (gray)")
    ap.add_argument("--conf", type=float, default=DEPLOY_CONF)
    ap.add_argument("--threads", type=int, default=6)
    ap.add_argument("--limit", type=int, default=0, help="debug: cap images")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        sys.exit(self_test())

    tag = args.tag or _dt.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    os.makedirs(OUT_DIR, exist_ok=True)
    prog = open(os.path.join(OUT_DIR, f"eval_n-mono-aug_progress_{tag}.log"), "a")
    rows_all = read_manifest(os.path.join(DATA, "manifest_test.csv"))

    summaries, perimage, perbox, clutter = [], [], [], []
    for m in args.models:
        if not os.path.exists(MODELS[m][0]):
            print(f"SKIP {m}: weights not found at {MODELS[m][0]}")
            continue
        rows, samp = sample_rows(rows_all, full=not MODELS[m][3])
        if args.limit:
            rows = rows[:args.limit]
        t0 = time.time()
        s, pi, pb = eval_model(m, rows, progress=prog, conf=args.conf,
                               threads=args.threads)
        s["sample"] = samp
        s["wall_s"] = round(time.time() - t0, 1)
        summaries.append(s)
        perimage.extend(pi)
        perbox.extend(pb)
        print(json.dumps(s))
        prog.write(json.dumps(s) + "\n")
        prog.flush()
        if args.clutter:
            c = eval_clutter(m, conf=args.conf, threads=args.threads)
            clutter.extend(c)
            for r in c:
                print(json.dumps(r))

    write_csv(os.path.join(OUT_DIR, f"eval_n-mono-aug_summary_{tag}.csv"), summaries)
    write_csv(os.path.join(OUT_DIR, f"eval_n-mono-aug_breakdown_{tag}.csv"),
              breakdown(perbox))
    write_csv(os.path.join(OUT_DIR, f"eval_n-mono-aug_perimage_{tag}.csv"), perimage)
    if clutter:
        write_csv(os.path.join(OUT_DIR, f"eval_n-mono-aug_clutter_{tag}.csv"), clutter)


if __name__ == "__main__":
    main()
