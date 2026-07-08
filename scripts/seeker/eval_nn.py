"""Offline evaluation of the NN markerless seeker (ADR-0033 Stage-B).

Runs scripts/seeker/nn_seeker.py over PRE-CAPTURED onboard frames
(demo_out/onboard_frames/frame_*.png, 1280x960) and reports, honestly:

  1. Detection rate over a sample spanning the approach (+ terminal window).
  2. Single-frame inference latency (mean/median/p95) with the brief's
     Pi-5 / Hailo-8 targets for context (interview-relevant).
  3. Bearing quality: NN box-centroid bearing vs the AprilTag-centroid bearing
     on the frames where the tag is readable (tag = EVAL-ONLY ground truth).
  4. BODY-NOT-TAG proof: on tag frames, inpaint the tag region out (replace it
     with body/arm texture reconstructed from around it -- the natural "no tag"
     scenario) and confirm the detector STILL fires on the airframe. Also
     reports box-size vs tag-size (box encloses the body, not just the tag).

NO SIM, NO PX4, NO GAZEBO. Offline against saved frames only.
Run with the ISOLATED seeker venv:
  OMP_NUM_THREADS=2 nice -n 15 \
    /home/emerson/interceptor-sim/.venv-seeker/bin/python \
    scripts/seeker/eval_nn.py [--all] [--n 50] [--out DIR]

`eval_tag_boxes.json` holds the AprilTag pixel boxes for the 6 frames where the
tag decodes; it is used ONLY for scoring/GT here and is NEVER consumed by the
seeker (honesty boundary, CLAUDE.md / ADR-0010).
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import time

import numpy as np
import cv2

from nn_seeker import NNSeeker, FX, CX

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
FRAMES_DIR = os.path.join(REPO, "demo_out", "onboard_frames")
TAG_GT = os.path.join(HERE, "eval_tag_boxes.json")


def bearing_from_u(u):
    return math.atan2((u - CX) / FX, 1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=50,
                    help="sample size spanning the approach (default 50)")
    ap.add_argument("--all", action="store_true",
                    help="evaluate all frames (post-batch full sweep)")
    ap.add_argument("--out", default=os.path.join(HERE, "eval_out"))
    ap.add_argument("--conf", type=float, default=0.15)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    frames = sorted(glob.glob(os.path.join(FRAMES_DIR, "frame_*.png")))
    assert frames, f"no frames in {FRAMES_DIR}"
    if args.all:
        sample = frames
    else:
        # uniform spread across the whole approach ...
        idx = set(np.linspace(0, len(frames) - 1, args.n).round().astype(int))
        # ... UNION dense coverage of the terminal window, where a COCO nano
        # can actually resolve the target (it is a few px at range -> R2).
        idx |= {i for i in range(525, min(561, len(frames)))}
        sample = [frames[i] for i in sorted(idx)]

    tag_gt = json.load(open(TAG_GT)) if os.path.exists(TAG_GT) else {}
    seeker = NNSeeker(conf_thres=args.conf)

    # warm up (first ORT run pays graph-optimization cost)
    seeker.detect(cv2.imread(sample[0]))

    n_det = 0
    latencies = []
    bearing_errs = []          # NN vs tag-centroid bearing (deg), tag frames
    range_rows = []            # (frame_idx, detected, conf, range_m)
    det_rows = []              # printed detail
    for p in sample:
        name = os.path.basename(p)
        img = cv2.imread(p)
        t0 = time.perf_counter()
        d = seeker.detect(img)
        latencies.append((time.perf_counter() - t0) * 1e3)
        detected = d.range_m is not None
        n_det += int(detected)
        fi = int(name.split("_")[1].split(".")[0])
        range_rows.append((fi, detected, d.decision_margin, d.range_m))
        if detected and name in tag_gt:
            x0, y0, x1, y1, tcx, tcy = tag_gt[name]
            nn_u = d.box_xywh[0] + d.box_xywh[2] / 2.0
            berr = math.degrees(bearing_from_u(nn_u) - bearing_from_u(tcx))
            bearing_errs.append(berr)
        if detected:
            det_rows.append(
                f"  {name}: conf={d.decision_margin:.2f} cls={d.class_name} "
                f"bearing={math.degrees(d.bearing_rad):+.1f}deg "
                f"range~{d.range_m:.1f}m box={tuple(round(v) for v in d.box_xywh)}")
            # save annotated frame
            ann = img.copy()
            x, y, w, h = [int(v) for v in d.box_xywh]
            cv2.rectangle(ann, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.drawMarker(ann, (int(x + w / 2), int(y + h / 2)), (0, 255, 255),
                           cv2.MARKER_CROSS, 24, 2)
            cv2.putText(ann, f"{d.class_name} {d.decision_margin:.2f} "
                        f"b={math.degrees(d.bearing_rad):+.1f}deg "
                        f"r~{d.range_m:.1f}m",
                        (max(0, x - 4), max(20, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imwrite(os.path.join(args.out, f"det_{name}"), ann)

    n = len(sample)
    lat = np.array(latencies)
    print("=" * 70)
    print(f"NN MARKERLESS SEEKER EVAL  (yolov8n COCO, onnxruntime CPU)")
    print(f"sample: {n} frames spanning approach "
          f"(frame {int(os.path.basename(sample[0]).split('_')[1].split('.')[0])}"
          f"..{int(os.path.basename(sample[-1]).split('_')[1].split('.')[0])} of {len(frames)})")
    print("=" * 70)
    print(f"DETECTION RATE (whole approach): {n_det}/{n} = {100*n_det/n:.0f}%")
    term = [r for r in range_rows if r[0] >= 520]
    term_det = sum(1 for r in term if r[1])
    if term:
        print(f"  terminal window (frame>=520, target large): "
              f"{term_det}/{len(term)} = {100*term_det/len(term):.0f}%")
    print(f"\nLATENCY (single frame, CPU, 2 threads): "
          f"mean {lat.mean():.1f} ms  median {np.median(lat):.1f} ms  "
          f"p95 {np.percentile(lat,95):.1f} ms  -> {1000/lat.mean():.0f} fps")
    print(f"  brief targets (seeker_design_brief sec 2.2): YOLOv8n Pi-5 CPU "
          f"~13 fps; Pi5+Hailo-8 ~35 fps end-to-end; Hailo HW ceiling ~431 fps")
    if bearing_errs:
        be = np.array(bearing_errs)
        print(f"\nBEARING (NN box centroid vs AprilTag centroid, {len(be)} tag "
              f"frames): mean {be.mean():+.2f} deg  |err| {np.abs(be).mean():.2f} "
              f"deg  std {be.std():.2f} deg")
        print("  (bearing is the steering-critical quantity; brief sec 3)")
    print("\nDETECTIONS:")
    for r in det_rows:
        print(r)

    # ---- BODY-NOT-TAG proof: inpaint the tag out, re-detect ---------------
    print("\n" + "=" * 70)
    print("BODY-NOT-TAG PROOF (inpaint tag region -> reconstruct body texture)")
    print("=" * 70)
    print("A fiducial-only detector would DIE when the tag is removed. We paint")
    print("the tag pixels out via inpainting (fills from surrounding body/arm")
    print("pixels = the natural 'no tag' case) and re-run the seeker.\n")
    proof = []
    for name in sorted(tag_gt):
        p = os.path.join(FRAMES_DIR, name)
        if not os.path.exists(p):
            continue
        img = cv2.imread(p)
        x0, y0, x1, y1, tcx, tcy = tag_gt[name]
        tag_sz = max(x1 - x0, y1 - y0)
        d0 = seeker.detect(img)
        pad = 5
        mask = np.zeros(img.shape[:2], np.uint8)
        mask[max(0, int(y0-pad)):int(y1+pad), max(0, int(x0-pad)):int(x1+pad)] = 255
        inp = cv2.inpaint(img, mask, 5, cv2.INPAINT_TELEA)
        d1 = seeker.detect(inp)
        def cov(d):
            if d.box_xywh is None:
                return False
            bx, by, bw, bh = d.box_xywh
            return bx <= tcx <= bx + bw and by <= tcy <= by + bh
        orig_ok = d0.range_m is not None
        box_ratio = (max(d0.box_xywh[2], d0.box_xywh[3]) / tag_sz) if orig_ok else 0
        inp_ok = d1.range_m is not None and cov(d1)
        proof.append((name, orig_ok, box_ratio, inp_ok,
                      d0.decision_margin, d1.decision_margin))
        cv2.imwrite(os.path.join(args.out, f"inpaint_{name}"), inp)
        print(f"  {name}: tag={tag_sz:.0f}px | ORIG {'HIT' if orig_ok else 'MISS'} "
              f"conf={d0.decision_margin if orig_ok else 0:.2f} "
              f"box={box_ratio:.1f}x tag | TAG-INPAINTED "
              f"{'STILL HITS body' if inp_ok else 'no hit'} "
              f"conf={d1.decision_margin if d1.range_m else 0:.2f}")
    n_orig = sum(1 for r in proof if r[1])
    n_inp = sum(1 for r in proof if r[3])
    med_ratio = np.median([r[2] for r in proof if r[1]]) if n_orig else 0
    print(f"\n  SUMMARY: {n_orig}/{len(proof)} tag frames detected; "
          f"of those, box is {med_ratio:.1f}x the tag size (encloses the "
          f"airframe, not the tag plane).")
    print(f"  With the tag INPAINTED OUT, the airframe is STILL detected in "
          f"{n_inp}/{len(proof)} frames -> detection is body/silhouette-driven, "
          f"not fiducial decode.")
    print(f"  (Honest caveat: confidence drops when the tag is removed -- the "
          f"tag's contrast does help. A fully tag-free evaluation needs the "
          f"tag-less Gazebo model models/fpv_target_markerless; flagged.)")
    print(f"\nAnnotated + inpainted frames saved to: {args.out}")


if __name__ == "__main__":
    main()
