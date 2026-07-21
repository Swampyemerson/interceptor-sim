#!/usr/bin/env python3
"""NN-tier RECON baseline — proper labeled eval of the on-hand drone detectors
on REAL imagery, in COLOR and GRAYSCALE (the OV9281 deployment camera is mono).

WHAT THIS IS (nn_tier recon phase, 2026-07-21)
----------------------------------------------
The transfer-bet smoke read (docs/transfer_bet_kill_test.md) was UNLABELED —
detection density only, no recall/precision. This script is the labeled
follow-on: it evaluates each candidate ONNX detector against a REAL, labeled,
permissively-licensed dataset (default: the Mendeley "YOLOv7 Segmented
Drone-vs-Bird" set, CC BY 4.0, DOI 10.17632/6ghdz52pd7.5 — real photos/frames,
drone + bird classes) and reports, per model, per color-mode:

  * AP50 for the drone class (all-point interpolated, IoU 0.5 greedy match)
  * precision / recall / F1 at the DEPLOYED operating point conf=0.25
    (v3_onnx_infer.DEPLOY_CONF — the same threshold the live seeker runs)
  * false-fire rate on drone-FREE images (here: bird-only images — the classic
    confuser class) = fraction of such images with >=1 drone fire @0.25
  * mean inference latency per frame (CPU, this box — NOT the Pi/Hailo number)

GRAYSCALE mode replicates the deployment camera: the innomaker OV9281 is
MONOCHROME, so frames are converted BGR->GRAY->3ch before the (color-trained)
nets see them. The color-vs-gray delta on the SAME frames is the measured mono
penalty (docs/camera_paper_check.md item 4 calls this UNVERIFIED-ON-PAPER).

HONESTY / anti-mirage notes (CLAUDE.md, ADR-0061, candidate_nn_shortlist.md §5)
  * Labels are used for SCORING ONLY — nothing here feeds a live seeker.
  * The eval corpus is TRAIN-DISJOINT from every evaluated model by
    construction (v2_* trained on Gazebo sim renders only; drone_yolo11x on
    the Kaggle amateur-drone set; flying_objects on its own 5-class corpus).
    Possible incidental web-image overlap with the two real-photo candidates
    cannot be ruled out and is disclosed in the scoreboard doc.
  * Frame-level metrics do NOT predict in-flight recall — this scoreboard
    ranks starting points for the fine-tune, it does not certify a seeker.
  * Sampling is deterministic (--seed) and reported with n; no cherry-picks.

Run (read-only seeker venv, CPU ORT):
  .venv-seeker/bin/python scripts/seeker/nn_tier/baseline_eval.py \
      --data scripts/seeker/nn_tier/data/dvb_yolo --split test \
      --max-pos 200 --max-neg 100
Self-test (no dataset/network needed; synthetic scorer + ONNX smoke):
  .venv-seeker/bin/python scripts/seeker/nn_tier/baseline_eval.py --self-test
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

HERE = os.path.dirname(os.path.abspath(__file__))
SEEKER = os.path.abspath(os.path.join(HERE, ".."))
REPO = os.path.abspath(os.path.join(SEEKER, "..", ".."))
sys.path.insert(0, SEEKER)

from v3_onnx_infer import _letterbox, load_session, iou, DEPLOY_CONF  # noqa: E402

WEIGHTS = os.path.join(SEEKER, "weights")

# name -> (onnx path, drone class id in the model's own head, n classes)
MODELS = {
    "v2_deployed":   (os.path.join(WEIGHTS, "drone_finetuned_quad_v2.onnx"), 0, 1),
    "quad_hardened": (os.path.join(WEIGHTS, "drone_finetuned_quad_hardened.onnx"), 0, 1),
    "quad_rebal":    (os.path.join(WEIGHTS, "drone_finetuned_quad_rebal.onnx"), 0, 1),
    "yolo11x_mit":   (os.path.join(WEIGHTS, "drone_yolo11x_1280.onnx"), 0, 1),
    "flyingobj_mit": (os.path.join(WEIGHTS, "flying_objects_yolov8m.onnx"), 0, 5),
}

AP_CONF_FLOOR = 0.01     # collect down to this conf for the PR curve
NMS_IOU = 0.45
MAX_DETS_PER_IMG = 100
MATCH_IOU = 0.5


# ---------------------------------------------------------------- inference

def decode_yolo(out, r, pads, n_classes, drone_cls, conf_floor):
    """Decode a YOLOv8/11-style ONNX output (1, 4+nc, N) into drone-class
    detections [{cx,cy,w,h,score}] in original-frame pixels (letterbox undone).
    Single-class heads are (1,5,N). Multi-class: take the drone channel."""
    pad_l, pad_t = pads
    preds = out[0]
    if preds.shape[0] < preds.shape[1]:
        preds = preds.T                       # (N, 4+nc)
    dets = []
    for row in preds:
        score = float(row[4 + drone_cls]) if n_classes > 1 else float(row[4])
        if score < conf_floor:
            continue
        cx_, cy_, w_, h_ = (float(row[0]), float(row[1]),
                            float(row[2]), float(row[3]))
        dets.append({"cx": (cx_ - pad_l) / r, "cy": (cy_ - pad_t) / r,
                     "w": w_ / r, "h": h_ / r, "score": score})
    return dets


def nms(dets, iou_thr=NMS_IOU, cap=MAX_DETS_PER_IMG):
    dets = sorted(dets, key=lambda d: -d["score"])
    kept = []
    for d in dets:
        db = (d["cx"], d["cy"], d["w"], d["h"])
        if all(iou(db, (k["cx"], k["cy"], k["w"], k["h"])) < iou_thr
               for k in kept):
            kept.append(d)
            if len(kept) >= cap:
                break
    return kept


def run_model_on_image(sess, iname, imgsz, frame_bgr, n_classes, drone_cls):
    img, r, pads = _letterbox(frame_bgr, imgsz)
    blob = img[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    out = sess.run(None, {iname: np.ascontiguousarray(blob)})[0]
    return nms(decode_yolo(out, r, pads, n_classes, drone_cls, AP_CONF_FLOOR))


def to_gray3(frame_bgr):
    g = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    return cv2.merge([g, g, g])


# ---------------------------------------------------------------- dataset

def load_labels(label_path, W, H, drone_class_ids):
    """YOLO label file -> list of drone-class GT boxes (cx,cy,w,h) px.
    Handles bbox lines (5 floats) AND segmentation-polygon lines
    (class x1 y1 x2 y2 ... normalized) by taking the polygon's bbox."""
    boxes = []
    if not os.path.isfile(label_path):
        return boxes
    with open(label_path) as fh:
        for ln in fh.read().splitlines():
            parts = ln.split()
            if len(parts) < 5:
                continue
            cls = int(float(parts[0]))
            if cls not in drone_class_ids:
                continue
            vals = [float(v) for v in parts[1:]]
            if len(vals) == 4:                      # bbox line
                ncx, ncy, nw, nh = vals
            else:                                   # polygon line
                xs, ys = vals[0::2], vals[1::2]
                x0, x1 = max(0.0, min(xs)), min(1.0, max(xs))
                y0, y1 = max(0.0, min(ys)), min(1.0, max(ys))
                ncx, ncy, nw, nh = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
            boxes.append((ncx * W, ncy * H, nw * W, nh * H))
    return boxes


def collect_split(data_dir, split, drone_class_ids, max_pos, max_neg, seed,
                  pos_prefix=None, neg_prefix=None):
    """Return (pos, neg) lists of (img_path, label_path_or_None) sampled
    deterministically.

    Default policy: positives = images with >=1 drone-class label; negatives =
    images with no drone-class label.

    PREFIX policy (--pos-prefix/--neg-prefix): the Mendeley Drone-vs-Bird
    corpus was found (2026-07-21 eyeball audit, 15 samples) to systematically
    MISLABEL birds as class 0 'Drone' inside its Bird partition (test: 271/361
    BT files are class-0-only boxes drawn on birds). Filename prefixes (DT/BT
    etc.) encode the true partition, so when prefixes are given: positives are
    drawn ONLY from pos_prefix files that carry a drone label (their class-0
    boxes were verified correct on samples), negatives ONLY from neg_prefix
    files with their label files DISCARDED (label_path=None -> zero GT), since
    their class bytes cannot be trusted. Disclosed in the scoreboard doc."""
    img_dir = os.path.join(data_dir, split, "images")
    lab_dir = os.path.join(data_dir, split, "labels")
    if not os.path.isdir(img_dir):                  # flat layout fallback
        img_dir = os.path.join(data_dir, "images")
        lab_dir = os.path.join(data_dir, "labels")
    imgs = sorted(glob.glob(os.path.join(img_dir, "*")))
    pos, neg = [], []
    for ip in imgs:
        base = os.path.splitext(os.path.basename(ip))[0]
        lp = os.path.join(lab_dir, base + ".txt")
        # cheap has-drone check without decoding the image yet
        has = False
        if os.path.isfile(lp):
            with open(lp) as fh:
                for ln in fh.read().splitlines():
                    p = ln.split()
                    if p and int(float(p[0])) in drone_class_ids:
                        has = True
                        break
        if pos_prefix or neg_prefix:
            if pos_prefix and base.startswith(pos_prefix) and has:
                pos.append((ip, lp))
            elif neg_prefix and base.startswith(neg_prefix):
                neg.append((ip, None))              # labels untrusted -> no GT
        else:
            (pos if has else neg).append((ip, lp))
    rng = random.Random(seed)
    rng.shuffle(pos)
    rng.shuffle(neg)
    return pos[:max_pos], neg[:max_neg]


# ---------------------------------------------------------------- scoring

def ap50(all_dets, n_gt):
    """All-point interpolated AP at IoU .5. all_dets = [(score, is_tp)],
    already greedily matched per image in score order."""
    if n_gt == 0 or not all_dets:
        return 0.0
    all_dets = sorted(all_dets, key=lambda t: -t[0])
    tps = np.cumsum([1 if t else 0 for _, t in all_dets])
    fps = np.cumsum([0 if t else 1 for _, t in all_dets])
    recall = tps / n_gt
    precision = tps / np.maximum(tps + fps, 1e-9)
    # precision envelope
    mpre = np.concatenate([[0.0], precision, [0.0]])
    mrec = np.concatenate([[0.0], recall, [1.0]])
    for i in range(len(mpre) - 2, -1, -1):
        mpre[i] = max(mpre[i], mpre[i + 1])
    idx = np.where(mrec[1:] != mrec[:-1])[0]
    return float(np.sum((mrec[idx + 1] - mrec[idx]) * mpre[idx + 1]))


def match_image(dets, gts, iou_thr=MATCH_IOU):
    """Greedy score-order matching. Returns [(score, is_tp)] for AP and the
    per-threshold TP bookkeeping ((score of det matched to each gt) list)."""
    taken = [False] * len(gts)
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
            scored.append((d["score"], True))
        else:
            scored.append((d["score"], False))
    return scored


def evaluate(model_key, samples, mode, progress_fh=None):
    """Run one model over the sample list in one color mode. Returns summary
    dict + per-image rows."""
    path, drone_cls, n_classes = MODELS[model_key]
    sess, iname, imgsz = load_session(path, intra_threads=8)
    all_scored, n_gt = [], 0
    tp25 = fp25 = fn25 = 0
    neg_imgs = neg_fired = 0
    rows = []
    lat = []
    for k, (ip, lp) in enumerate(samples):
        frame = cv2.imread(ip)
        if frame is None:
            continue
        if mode == "gray":
            frame = to_gray3(frame)
        H, W = frame.shape[:2]
        gts = load_labels(lp, W, H, drone_class_ids=DRONE_CLASS_IDS) if lp else []
        t0 = time.time()
        dets = run_model_on_image(sess, iname, imgsz, frame, n_classes, drone_cls)
        lat.append(time.time() - t0)
        scored = match_image(dets, gts)
        all_scored.extend(scored)
        n_gt += len(gts)
        # operating point conf 0.25
        s25 = [(s, tp) for s, tp in scored if s >= DEPLOY_CONF]
        tp_i = sum(1 for _, tp in s25 if tp)
        fp_i = sum(1 for _, tp in s25 if not tp)
        tp25 += tp_i
        fp25 += fp_i
        fn25 += max(0, len(gts) - tp_i)
        if not gts:
            neg_imgs += 1
            if len(s25) > 0:
                neg_fired += 1
        rows.append({"model": model_key, "mode": mode,
                     "image": os.path.basename(ip), "n_gt": len(gts),
                     "n_det25": len(s25), "tp25": tp_i, "fp25": fp_i})
        if progress_fh and (k % 20 == 0):
            progress_fh.write(f"{_dt.datetime.now().isoformat()} {model_key} "
                              f"{mode} {k}/{len(samples)}\n")
            progress_fh.flush()
    prec = tp25 / max(tp25 + fp25, 1)
    rec = tp25 / max(tp25 + fn25, 1)
    f1 = 2 * prec * rec / max(prec + rec, 1e-9)
    return {
        "model": model_key, "mode": mode, "n_images": len(samples),
        "n_gt_boxes": n_gt, "ap50": round(ap50(all_scored, n_gt), 4),
        "precision@25": round(prec, 4), "recall@25": round(rec, 4),
        "f1@25": round(f1, 4),
        "neg_images": neg_imgs,
        "false_fire_rate@25": round(neg_fired / max(neg_imgs, 1), 4),
        "latency_ms_mean": round(1000 * float(np.mean(lat)), 1) if lat else -1,
    }, rows


DRONE_CLASS_IDS = {0}          # overwritten from --drone-class


# ---------------------------------------------------------------- self-test

def self_test():
    """1) Scorer math on a fabricated case with known answers.
       2) ONNX pipeline smoke on 2 synthetic frames through the deployed net.
    Exits 0 iff both pass."""
    ok = True
    # -- scorer: 2 images. img A: 1 gt, det hits (score .9) + 1 fp (.3).
    #            img B: 1 gt, missed. => P@25 = 1/2, R@25 = 1/2.
    gts_a = [(50, 50, 20, 20)]
    dets_a = [{"cx": 51, "cy": 50, "w": 20, "h": 20, "score": 0.9},
              {"cx": 200, "cy": 200, "w": 20, "h": 20, "score": 0.3}]
    scored = match_image(dets_a, gts_a)
    exp = [(0.9, True), (0.3, False)]
    if scored != exp:
        print(f"SELF-TEST FAIL: match_image {scored} != {exp}")
        ok = False
    a = ap50(scored, n_gt=2)   # PR: (r=.5,p=1), then fp -> envelope AP = .5
    if abs(a - 0.5) > 1e-6:
        print(f"SELF-TEST FAIL: ap50 {a} != 0.5")
        ok = False
    # degenerate guards
    if ap50([], 0) != 0.0 or ap50([(0.5, True)], 0) != 0.0:
        print("SELF-TEST FAIL: ap50 degenerate")
        ok = False
    # polygon->bbox label parsing
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as fh:
        fh.write("0 0.1 0.1 0.3 0.1 0.3 0.5 0.1 0.5\n1 0.5 0.5 0.1 0.1\n")
        lp = fh.name
    b = load_labels(lp, 100, 100, {0})
    os.unlink(lp)
    if len(b) != 1 or abs(b[0][0] - 20) > 1e-6 or abs(b[0][2] - 20) > 1e-6:
        print(f"SELF-TEST FAIL: polygon parse {b}")
        ok = False
    # -- ONNX smoke: deployed net on 2 synthetic frames (sky + dark blob).
    path, dcls, ncls = MODELS["v2_deployed"]
    if os.path.exists(path):
        sess, iname, imgsz = load_session(path, intra_threads=2)
        for i in range(2):
            f = np.full((480, 640, 3), 200, np.uint8)
            if i:
                cv2.rectangle(f, (300, 220), (340, 250), (30, 30, 30), -1)
            dets = run_model_on_image(sess, iname, imgsz, f, ncls, dcls)
            _ = run_model_on_image(sess, iname, imgsz, to_gray3(f), ncls, dcls)
            if not isinstance(dets, list):
                print("SELF-TEST FAIL: onnx smoke returned non-list")
                ok = False
        print("onnx smoke: deployed net ran on 2 synthetic frames (color+gray)")
    else:
        print(f"SELF-TEST FAIL: deployed weights missing at {path}")
        ok = False
    print("SELF-TEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ---------------------------------------------------------------- main

def main():
    global DRONE_CLASS_IDS
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", help="YOLO dataset root (train/valid/test dirs)")
    ap.add_argument("--split", default="test")
    ap.add_argument("--drone-class", type=int, nargs="+", default=[0],
                    help="class id(s) in the DATASET labels meaning 'drone'")
    ap.add_argument("--models", nargs="+", default=list(MODELS.keys()))
    ap.add_argument("--modes", nargs="+", default=["color", "gray"])
    ap.add_argument("--max-pos", type=int, default=200)
    ap.add_argument("--max-neg", type=int, default=100)
    ap.add_argument("--pos-prefix", default=None,
                    help="restrict positives to basenames with this prefix")
    ap.add_argument("--neg-prefix", default=None,
                    help="negatives = basenames with this prefix, labels IGNORED")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--tag", default=None, help="output filename tag")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        sys.exit(self_test())
    if not args.data:
        ap.error("--data required (or --self-test)")

    DRONE_CLASS_IDS = set(args.drone_class)
    tag = args.tag or _dt.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    out_dir = os.path.join(REPO, "logs", "nn_tier")
    os.makedirs(out_dir, exist_ok=True)
    prog = open(os.path.join(out_dir, f"baseline_progress_{tag}.log"), "a")

    pos, neg = collect_split(args.data, args.split, DRONE_CLASS_IDS,
                             args.max_pos, args.max_neg, args.seed,
                             pos_prefix=args.pos_prefix,
                             neg_prefix=args.neg_prefix)
    samples = pos + neg
    print(f"eval set: {len(pos)} drone-positive + {len(neg)} drone-free images "
          f"(split={args.split}, seed={args.seed}, drone_cls={DRONE_CLASS_IDS})")
    prog.write(f"eval set pos={len(pos)} neg={len(neg)} split={args.split}\n")

    summaries, perimage = [], []
    for m in args.models:
        if not os.path.exists(MODELS[m][0]):
            print(f"SKIP {m}: weights not found")
            continue
        for mode in args.modes:
            t0 = time.time()
            s, rows = evaluate(m, samples, mode, progress_fh=prog)
            s["wall_s"] = round(time.time() - t0, 1)
            summaries.append(s)
            perimage.extend(rows)
            print(json.dumps(s))
            prog.write(json.dumps(s) + "\n")
            prog.flush()

    sum_path = os.path.join(out_dir, f"baseline_scoreboard_{tag}.csv")
    with open(sum_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(summaries[0].keys()))
        w.writeheader()
        w.writerows(summaries)
    pi_path = os.path.join(out_dir, f"baseline_perimage_{tag}.csv")
    with open(pi_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(perimage[0].keys()))
        w.writeheader()
        w.writerows(perimage)
    print(f"wrote {sum_path}\nwrote {pi_path}")


if __name__ == "__main__":
    main()
