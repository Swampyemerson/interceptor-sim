#!/usr/bin/env python3
"""HELD-OUT-SOURCE eval for the nn_tier fine-tunes (EVALUATE phase, s-mono).

WHAT THIS IS (2026-07-21)
-------------------------
Scores a model the HONEST way on the nn_tier test split, which is
SOURCE/VIDEO/SCENE-disjoint by construction (dataset_report.md: `dut` is a
whole never-seen source pinned to test; `nps` test images are whole held-out
clips; `plates` test negatives are held-out photographers). NEVER random
frames — the v3/rebal NULLs (ADR-0061) both aced random-frame eval and failed
in flight, so nothing here evaluates on frames whose source was trained on.

Reports, per model, in GRAYSCALE (the innomaker OV9281 deployment camera is
monochrome — camera_paper_check.md item 4):
  * AP50 (all-point interpolation, greedy IoU-0.5 match)
  * box-level recall / precision / F1 at the deployed operating point
    conf = 0.25 (v3_onnx_infer.DEPLOY_CONF)
  * gate-recall@25: per-GT-box hits via the FIXED scoring gate
    scripts/seeker/box_scoring.box_hits_gt (ADR-0076 add #18k). Called with
    gt_scale=1.0 and offaxis_aware=False because these are REAL images whose
    labels were drawn on the actual pixels (they already contain any off-axis
    apparent-size growth; the sec^2 widening exists to fix PARAXIAL sim-derived
    gt, not human labels). tol=15 px, size_ratio=3.0 (defaults).
  * recall broken down by TARGET SCALE (max-box-width px @1280-equivalent —
    the 6-20 m terminal-band proxy: per viewpoint_and_deploy_spec.md A3 the
    0.35 m target spans ~7-36 px @1280-eq across 6-20 m) and by FRAME POSITION
    (3x3 grid of the GT centre).
  * false-fire density on drone-free negatives (fraction of neg frames with
    >=1 fire @0.25, and mean fires per neg frame), broken down by source.
  * mean CPU latency per frame on THIS box (NOT a Pi/Hailo number).

Comparison bar: the deployed sim-trained `drone_finetuned_quad_v2` and the
MIT/candidate baselines (yolo11x_mit, flyingobj_mit), on the SAME split.
Heavy @1280 models run a deterministic seed-0 subset (all negatives + capped
positives) for CPU-time reasons; every model's summary is ALSO computed
restricted to that exact subset so heavy-vs-light rows are apples-to-apples
(subset column: `full` vs `heavy`).

--clutter additionally runs the transfer-bet outdoor footage's drone-FREE
segments (docs/transfer_bet_kill_test.md; scripts/seeker/data/
transfer_bet_footage/neg_*) for real-clutter false-fire density, plus the
pos_photo_dji frames as an UNLABELED anecdote (density, never recall).

HONESTY
  * Labels score, never steer: nothing here feeds a live seeker (gt_* rule).
  * Frame metrics do not certify a seeker (ADR-0061); this ranks fine-tunes
    and feeds the Hailo scale-down decision (viewpoint_and_deploy_spec.md B5).
  * Every number lands in logs/nn_tier/eval_s-mono_*.csv with n and policy.

Run (read-only inference venv):
  .venv-seeker/bin/python scripts/seeker/nn_tier/eval_heldout_smono.py \
      --models v2_deployed --mode gray --tag <tag>
Self-test (fast, no network; needs the deployed weights + a few test images):
  .venv-seeker/bin/python scripts/seeker/nn_tier/eval_heldout_smono.py --self-test
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import json
import os
import random
import sys
import time

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))            # scripts/seeker/nn_tier
SEEKER = os.path.abspath(os.path.join(HERE, ".."))           # scripts/seeker
REPO = os.path.abspath(os.path.join(SEEKER, "..", ".."))     # repo root
sys.path.insert(0, SEEKER)
sys.path.insert(0, HERE)

from v3_onnx_infer import load_session, iou, DEPLOY_CONF            # noqa: E402
from box_scoring import box_hits_gt                                  # noqa: E402
from baseline_eval import (run_model_on_image, to_gray3, load_labels,  # noqa: E402
                           ap50)

DATA_ROOT = os.path.join(SEEKER, "data", "nn_tier")
MANIFEST = os.path.join(DATA_ROOT, "manifest_test.csv")
WEIGHTS = os.path.join(SEEKER, "weights")
NN_W = os.path.join(WEIGHTS, "nn_tier")
CLUTTER_DIR = os.path.join(SEEKER, "data", "transfer_bet_footage")
OUT_DIR = os.path.join(REPO, "logs", "nn_tier")

# name -> (onnx path, drone class id in the MODEL's head, n classes)
MODELS = {
    "s_mono":        (os.path.join(NN_W, "s-mono.onnx"), 0, 1),
    "n_mono":        (os.path.join(NN_W, "n-mono.onnx"), 0, 1),
    "v2_deployed":   (os.path.join(WEIGHTS, "drone_finetuned_quad_v2.onnx"), 0, 1),
    "yolo11x_mit":   (os.path.join(WEIGHTS, "drone_yolo11x_1280.onnx"), 0, 1),
    "flyingobj_mit": (os.path.join(WEIGHTS, "flying_objects_yolov8m.onnx"), 0, 5),
}
HEAVY = {"yolo11x_mit", "flyingobj_mit"}     # @1280 CPU-slow -> subset policy
HEAVY_MAX_POS = 700
SUBSET_SEED = 0

# scale bins in px @1280-equivalent (dataset_report.md convention).
# 6-20 m terminal-band proxy (0.35 m target, viewpoint_and_deploy_spec.md A3):
# 20 m ~ 6.7-10.9 px, 6 m ~ 22.4-36.3 px -> the band lives in 0-40 px.
SCALE_BINS = [(0, 12, "00-12px"), (12, 24, "12-24px"), (24, 40, "24-40px"),
              (40, 80, "40-80px"), (80, 10 ** 9, "80+px")]
POS_CELLS = ["top", "mid", "bot"]  # x thirds: left/cen/right


def scale_bin(w1280: float) -> str:
    for lo, hi, name in SCALE_BINS:
        if lo <= w1280 < hi:
            return name
    return SCALE_BINS[-1][2]


def pos_cell(gcx: float, gcy: float, W: int, H: int) -> str:
    col = ["left", "cen", "right"][min(2, int(3 * gcx / max(W, 1)))]
    row = POS_CELLS[min(2, int(3 * gcy / max(H, 1)))]
    return f"{row}-{col}"


def match_image_full(dets, gts, iou_thr=0.5):
    """Greedy score-order match. Returns ([(score,is_tp)], gt_matched_bools,
    gt_match_scores). Mirrors baseline_eval.match_image but also reports which
    GT each match consumed (needed for per-box recall breakdowns)."""
    taken = [False] * len(gts)
    gt_score = [None] * len(gts)
    scored = []
    for d in sorted(dets, key=lambda x: -x["score"]):
        db = (d["cx"], d["cy"], d["w"], d["h"])
        best_i, best_iou = -1, iou_thr
        for i, g in enumerate(gts):
            if taken[i]:
                continue
            v = iou(db, g)
            if v >= best_iou:
                best_i, best_iou = i, v
        if best_i >= 0:
            taken[best_i] = True
            gt_score[best_i] = d["score"]
            scored.append((d["score"], True))
        else:
            scored.append((d["score"], False))
    return scored, taken, gt_score


# ---------------------------------------------------------------- manifest

def read_manifest(path=MANIFEST):
    rows = []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            r["n_boxes"] = int(r["n_boxes"])
            rows.append(r)
    return rows


def heavy_subset_uids(rows):
    """Deterministic seed-0 subset for @1280 models: ALL negatives + the first
    HEAVY_MAX_POS of a seeded shuffle of positives."""
    pos = sorted(r["uid"] for r in rows if r["n_boxes"] > 0)
    neg = sorted(r["uid"] for r in rows if r["n_boxes"] == 0)
    rng = random.Random(SUBSET_SEED)
    rng.shuffle(pos)
    return set(pos[:HEAVY_MAX_POS]) | set(neg)


# ---------------------------------------------------------------- evaluation

def eval_model(model_key, rows, mode, threads, progress_path, limit=None):
    """Run one model over manifest rows. Returns (per_image, per_box, scored_by_uid).
    per_image/per_box are lists of dicts; scored_by_uid maps uid -> ([(score,tp)], n_gt)."""
    path, drone_cls, n_classes = MODELS[model_key]
    sess, iname, imgsz = load_session(path, intra_threads=threads)
    per_image, per_box = [], []
    scored_by_uid = {}
    lat = []
    prog = open(progress_path, "a")
    use = rows if limit is None else rows[:limit]
    for k, r in enumerate(use):
        img_rel = r["gray_img"] if mode == "gray" else r["color_img"]
        ip = os.path.join(DATA_ROOT, img_rel)
        frame = cv2.imread(ip)
        if frame is None:
            continue
        if mode == "gray":
            frame = to_gray3(frame)      # idempotent on the gray tree; exact OV9281 path
        H, W = frame.shape[:2]
        lp = os.path.join(DATA_ROOT, ("gray" if mode == "gray" else "color"),
                          "labels", r["split"], r["uid"] + ".txt")
        gts = load_labels(lp, W, H, drone_class_ids={0})
        t0 = time.time()
        dets = run_model_on_image(sess, iname, imgsz, frame, n_classes, drone_cls)
        lat.append(time.time() - t0)
        scored, gt_taken, _ = match_image_full(dets, gts)
        scored_by_uid[r["uid"]] = (scored, len(gts))
        dets25 = [d for d in dets if d["score"] >= DEPLOY_CONF]
        s25 = [(s, tp) for s, tp in scored if s >= DEPLOY_CONF]
        tp_i = sum(1 for _, tp in s25 if tp)
        fp_i = sum(1 for _, tp in s25 if not tp)
        # per-GT-box rows: IoU-match flag + the FIXED box_hits_gt gate flag
        boxes25 = [(d["score"], d["cx"], d["cy"], d["w"], d["h"]) for d in dets25]
        # which GTs were IoU-matched at >= DEPLOY_CONF specifically:
        _, gt_taken25, _ = match_image_full(dets25, gts)
        for gi, g in enumerate(gts):
            gcx, gcy, gw, gh = g
            w1280 = gw * 1280.0 / max(W, 1)
            hit = box_hits_gt(boxes25, gcx, gcy, gw, gh, tol=15,
                              gt_scale=1.0, offaxis_aware=False)
            per_box.append({
                "model": model_key, "mode": mode, "uid": r["uid"],
                "source": r["source"], "W": W, "H": H,
                "gcx": round(gcx, 1), "gcy": round(gcy, 1),
                "gw": round(gw, 1), "gh": round(gh, 1),
                "w1280": round(w1280, 1), "scale_bin": scale_bin(w1280),
                "pos_cell": pos_cell(gcx, gcy, W, H),
                "iou_tp25": int(bool(gt_taken25[gi])),
                "gate_hit25": int(hit),
            })
        per_image.append({
            "model": model_key, "mode": mode, "uid": r["uid"],
            "source": r["source"], "n_gt": len(gts), "n_det25": len(dets25),
            "tp25": tp_i, "fp25": fp_i, "fired": int(len(dets25) > 0),
            "ms": round(1000 * lat[-1], 1),
        })
        if k % 25 == 0:
            prog.write(f"{_dt.datetime.utcnow().isoformat()} {model_key} {mode} "
                       f"{k}/{len(use)}\n")
            prog.flush()
    prog.write(f"{_dt.datetime.utcnow().isoformat()} {model_key} {mode} DONE "
               f"n={len(per_image)} mean_ms={1000 * float(np.mean(lat)):.1f}\n")
    prog.close()
    return per_image, per_box, scored_by_uid


def summarize(model_key, mode, subset_name, uid_filter, per_image, per_box,
              scored_by_uid):
    """Compute one summary row over the given uid set (None = all)."""
    pi = [r for r in per_image if uid_filter is None or r["uid"] in uid_filter]
    pb = [r for r in per_box if uid_filter is None or r["uid"] in uid_filter]
    all_scored, n_gt = [], 0
    for uid, (scored, ng) in scored_by_uid.items():
        if uid_filter is None or uid in uid_filter:
            all_scored.extend(scored)
            n_gt += ng
    tp = sum(r["tp25"] for r in pi)
    fp = sum(r["fp25"] for r in pi)
    fn = n_gt - tp
    prec = tp / max(tp + fp, 1)
    rec = tp / max(n_gt, 1)
    f1 = 2 * prec * rec / max(prec + rec, 1e-9)
    negs = [r for r in pi if r["n_gt"] == 0]
    fired = sum(r["fired"] for r in negs)
    fires = sum(r["n_det25"] for r in negs)
    return {
        "model": model_key, "mode": mode, "subset": subset_name,
        "n_images": len(pi), "n_pos_images": len(pi) - len(negs),
        "n_gt_boxes": n_gt,
        "ap50": round(ap50(all_scored, n_gt), 4),
        "recall@25": round(rec, 4), "precision@25": round(prec, 4),
        "f1@25": round(f1, 4),
        "gate_recall@25": round(sum(r["gate_hit25"] for r in pb) / max(len(pb), 1), 4),
        "n_neg_images": len(negs),
        "false_fire_rate@25": round(fired / max(len(negs), 1), 4),
        "fires_per_neg_frame": round(fires / max(len(negs), 1), 3),
        "latency_ms_mean": round(float(np.mean([r["ms"] for r in pi])), 1) if pi else -1,
    }


def breakdown_rows(model_key, mode, per_box, per_image):
    """Recall by scale bin, by position cell, and false-fire by source."""
    out = []
    for _, _, bname in SCALE_BINS:
        sel = [r for r in per_box if r["scale_bin"] == bname]
        if not sel:
            continue
        out.append({"model": model_key, "mode": mode, "kind": "scale",
                    "key": bname, "n_gt": len(sel),
                    "recall_iou@25": round(sum(r["iou_tp25"] for r in sel) / len(sel), 4),
                    "recall_gate@25": round(sum(r["gate_hit25"] for r in sel) / len(sel), 4)})
    cells = sorted({r["pos_cell"] for r in per_box})
    for c in cells:
        sel = [r for r in per_box if r["pos_cell"] == c]
        out.append({"model": model_key, "mode": mode, "kind": "position",
                    "key": c, "n_gt": len(sel),
                    "recall_iou@25": round(sum(r["iou_tp25"] for r in sel) / len(sel), 4),
                    "recall_gate@25": round(sum(r["gate_hit25"] for r in sel) / len(sel), 4)})
    for src in sorted({r["source"] for r in per_image}):
        negs = [r for r in per_image if r["source"] == src and r["n_gt"] == 0]
        if not negs:
            continue
        out.append({"model": model_key, "mode": mode, "kind": "falsefire_src",
                    "key": src, "n_gt": len(negs),
                    "recall_iou@25": round(sum(r["fired"] for r in negs) / len(negs), 4),
                    "recall_gate@25": round(sum(r["n_det25"] for r in negs) / len(negs), 3)})
        # for falsefire_src rows: recall_iou@25 column holds FIRE RATE and
        # recall_gate@25 holds MEAN FIRES/FRAME (documented in the header note)
    return out


# ---------------------------------------------------------------- clutter

def eval_clutter(model_key, mode, threads):
    """Transfer-bet footage smoke: drone-FREE segments -> false-fire density;
    pos_photo_dji -> unlabeled anecdote. Density, never recall (unlabeled)."""
    path, drone_cls, n_classes = MODELS[model_key]
    sess, iname, imgsz = load_session(path, intra_threads=threads)
    rows = []
    segs = ["neg_bird", "neg_seagull", "neg_sky", "neg_ground", "pos_photo_dji"]
    for seg in segs:
        d = os.path.join(CLUTTER_DIR, seg)
        if not os.path.isdir(d):
            continue
        frames = sorted(f for f in os.listdir(d)
                        if f.lower().endswith((".png", ".jpg"))
                        and not f.startswith("_raw"))
        fired = 0
        n_det_total = 0
        per_frame = []
        for f in frames:
            frame = cv2.imread(os.path.join(d, f))
            if frame is None:
                continue
            if mode == "gray":
                frame = to_gray3(frame)
            dets = run_model_on_image(sess, iname, imgsz, frame, n_classes, drone_cls)
            n25 = sum(1 for x in dets if x["score"] >= DEPLOY_CONF)
            fired += int(n25 > 0)
            n_det_total += n25
            per_frame.append(f"{f}:{n25}")
        n = len(frames)
        rows.append({"model": model_key, "mode": mode, "segment": seg, "n_frames": n,
                     "frames_fired@25": fired,
                     "fire_density": round(fired / max(n, 1), 3),
                     "fires_per_frame": round(n_det_total / max(n, 1), 2),
                     "per_frame": ";".join(per_frame)})
    return rows


# ---------------------------------------------------------------- self-test

def self_test():
    ok = True
    # 1) box_hits_gt integration sanity: centred hit passes, far box fails,
    #    6x blob guard rejects.
    if not box_hits_gt([(0.9, 100, 100, 30, 30)], 100, 100, 28, 28,
                       gt_scale=1.0, offaxis_aware=False):
        print("SELF-TEST FAIL: box_hits_gt centred hit rejected")
        ok = False
    if box_hits_gt([(0.9, 400, 400, 30, 30)], 100, 100, 28, 28,
                   gt_scale=1.0, offaxis_aware=False):
        print("SELF-TEST FAIL: box_hits_gt far box accepted")
        ok = False
    if box_hits_gt([(0.9, 100, 100, 200, 200)], 100, 100, 30, 30,
                   gt_scale=1.0, offaxis_aware=False):
        print("SELF-TEST FAIL: box_hits_gt blob guard failed")
        ok = False
    # 2) match_image_full parity with known case
    gts = [(50, 50, 20, 20)]
    dets = [{"cx": 51, "cy": 50, "w": 20, "h": 20, "score": 0.9},
            {"cx": 200, "cy": 200, "w": 20, "h": 20, "score": 0.3}]
    scored, taken, _ = match_image_full(dets, gts)
    if scored != [(0.9, True), (0.3, False)] or taken != [True]:
        print(f"SELF-TEST FAIL: match_image_full {scored} {taken}")
        ok = False
    if abs(ap50(scored, 2) - 0.5) > 1e-6:
        print("SELF-TEST FAIL: ap50 chain")
        ok = False
    # 3) scale/pos binning
    if scale_bin(11.9) != "00-12px" or scale_bin(35.0) != "24-40px":
        print("SELF-TEST FAIL: scale_bin")
        ok = False
    if pos_cell(10, 10, 300, 300) != "top-left" or pos_cell(290, 290, 300, 300) != "bot-right":
        print("SELF-TEST FAIL: pos_cell")
        ok = False
    # 4) tiny REAL run: 3 positives + 2 negatives from the manifest through the
    #    deployed net (always present), gray mode, end-to-end through the same
    #    code path the full eval uses. Uses s_mono instead when it exists.
    rows = read_manifest()
    pos = [r for r in rows if r["n_boxes"] > 0][:3]
    neg = [r for r in rows if r["n_boxes"] == 0][:2]
    key = "s_mono" if os.path.exists(MODELS["s_mono"][0]) else "v2_deployed"
    if not os.path.exists(MODELS[key][0]):
        print(f"SELF-TEST FAIL: no weights at {MODELS[key][0]}")
        return 1
    prog = os.path.join(OUT_DIR, "eval_smono_selftest_progress.log")
    pi, pb, sb = eval_model(key, pos + neg, "gray", 2, prog)
    if len(pi) != 5:
        print(f"SELF-TEST FAIL: tiny run produced {len(pi)} image rows != 5")
        ok = False
    s = summarize(key, "gray", "tiny", None, pi, pb, sb)
    for col in ("ap50", "recall@25", "false_fire_rate@25", "gate_recall@25"):
        if not (0.0 <= float(s[col]) <= 1.0):
            print(f"SELF-TEST FAIL: {col}={s[col]} out of [0,1]")
            ok = False
    print(f"tiny run [{key}]: {json.dumps(s)}")
    print("SELF-TEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--models", nargs="+", default=["s_mono"],
                    choices=list(MODELS.keys()))
    ap.add_argument("--mode", default="gray", choices=["gray", "color"])
    ap.add_argument("--threads", type=int, default=6)
    ap.add_argument("--tag", default=None)
    ap.add_argument("--clutter", action="store_true",
                    help="also run the transfer-bet clutter segments")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        sys.exit(self_test())

    os.makedirs(OUT_DIR, exist_ok=True)
    tag = args.tag or _dt.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    rows = read_manifest()
    heavy_uids = heavy_subset_uids(rows)
    prog_path = os.path.join(OUT_DIR, f"eval_smono_progress_{tag}.log")

    sum_rows, bd_rows, clutter_rows = [], [], []
    pi_path = os.path.join(OUT_DIR, f"eval_s-mono_perimage_{tag}.csv")
    pb_path = os.path.join(OUT_DIR, f"eval_s-mono_perbox_{tag}.csv")
    pi_all, pb_all = [], []

    for m in args.models:
        if not os.path.exists(MODELS[m][0]):
            print(f"SKIP {m}: no weights at {MODELS[m][0]}")
            continue
        use = [r for r in rows if r["uid"] in heavy_uids] if m in HEAVY else rows
        t0 = time.time()
        pi, pb, sb = eval_model(m, use, args.mode, args.threads, prog_path)
        wall = round(time.time() - t0, 1)
        if m in HEAVY:
            s = summarize(m, args.mode, "heavy", None, pi, pb, sb)
            s["wall_s"] = wall
            sum_rows.append(s)
        else:
            s = summarize(m, args.mode, "full", None, pi, pb, sb)
            s["wall_s"] = wall
            sum_rows.append(s)
            s2 = summarize(m, args.mode, "heavy", heavy_uids, pi, pb, sb)
            s2["wall_s"] = wall
            sum_rows.append(s2)
        bd_rows.extend(breakdown_rows(m, args.mode, pb, pi))
        pi_all.extend(pi)
        pb_all.extend(pb)
        print(json.dumps(s))
        if args.clutter:
            clutter_rows.extend(eval_clutter(m, args.mode, args.threads))
        # incremental CSV flush after each model (crash safety)
        for path, data in ((pi_path, pi_all), (pb_path, pb_all)):
            if data:
                with open(path, "w", newline="") as fh:
                    w = csv.DictWriter(fh, fieldnames=list(data[0].keys()))
                    w.writeheader()
                    w.writerows(data)
        if sum_rows:
            with open(os.path.join(OUT_DIR, f"eval_s-mono_summary_{tag}.csv"),
                      "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(sum_rows[0].keys()))
                w.writeheader()
                w.writerows(sum_rows)
        if bd_rows:
            with open(os.path.join(OUT_DIR, f"eval_s-mono_breakdown_{tag}.csv"),
                      "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(bd_rows[0].keys()))
                w.writeheader()
                w.writerows(bd_rows)
        if clutter_rows:
            with open(os.path.join(OUT_DIR, f"eval_s-mono_clutter_{tag}.csv"),
                      "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(clutter_rows[0].keys()))
                w.writeheader()
                w.writerows(clutter_rows)
    print(f"wrote eval_s-mono_*_{tag}.csv in {OUT_DIR}")


if __name__ == "__main__":
    main()
