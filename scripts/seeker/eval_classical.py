"""Offline eval for the classical markerless seeker (ADR-0033 Stage A).

Runs ENTIRELY on pre-captured onboard frames -- NO simulator, NO PX4, NO flight.
Evaluates, on a ~50-frame sample spanning the approach:
  1. Detection rate (per-frame stateless detect()).
  2. BODY-NOT-TAG evidence (the point of the milestone):
     a. tag-white fraction inside the detected box (a real tag is ~50% white),
     b. the REAL AprilTag detector (pupil_apriltags) on the same frames -- if it
        finds no tag yet our seeker fires, the seeker cannot be riding the tag,
     c. masked-tag robustness: blank the brightest sub-region of each detection
        (a hypothetical emissive tag plane) and confirm detection persists.
  3. Bearing continuity across a contiguous run (detect-then-track).
Saves annotated evidence frames to scripts/seeker/out/ and a per-frame CSV.

Run:  OMP_NUM_THREADS=2 nice -n 15 ./.venv-seeker/bin/python scripts/seeker/eval_classical.py
"""

from __future__ import annotations

import csv
import glob
import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from classical_seeker import ClassicalSeeker, load_intrinsics  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
FRAMES_DIR = ROOT / "demo_out" / "onboard_frames"
OUT = Path(__file__).resolve().parent / "out"
OUT.mkdir(exist_ok=True)

N_SAMPLE = 50  # sample size for the offline gate (full 866-frame sweep = post-batch)


def sample_indices(n_files: int, n: int):
    return list(np.linspace(0, n_files - 1, n).round().astype(int))


def brightest_region_mask(gray, box, frac=0.4):
    """uint8 mask blanking the brightest `frac` of the detection box -- stands in
    for masking an emissive AprilTag plane."""
    x, y, w, h = box
    m = np.zeros(gray.shape, np.uint8)
    roi = gray[y:y + h, x:x + w]
    if roi.size == 0:
        return m
    thr = np.quantile(roi, 1.0 - frac)
    sub = (roi >= thr).astype(np.uint8) * 255
    m[y:y + h, x:x + w] = sub
    return m


def main():
    files = sorted(glob.glob(str(FRAMES_DIR / "frame_*.png")))
    fx, fy, cx, cy = load_intrinsics(str(ROOT / "configs/camera_intrinsics.json"))
    seeker = ClassicalSeeker(fx, fy, cx, cy)
    print(f"intrinsics fx={fx:.1f} cx={cx} cy={cy}; {len(files)} frames; "
          f"sampling {N_SAMPLE}")

    # real AprilTag detector for the cross-check
    from pupil_apriltags import Detector
    tag_det = Detector(families="tag36h11")

    idxs = sample_indices(len(files), N_SAMPLE)
    rows = []
    n_det = 0
    n_tag_frames = 0
    masked_still_fire = 0
    masked_eligible = 0
    save_every = max(1, len(idxs) // 6)

    for k, i in enumerate(idxs):
        im = cv2.imread(files[i])
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        t = i / 30.0  # synthetic mono time (~30 Hz capture), offline only
        det = seeker.detect(im, t)

        # real-tag cross-check on the same frame
        tags = tag_det.detect(gray)
        if tags:
            n_tag_frames += 1

        twf = mask_fire = None
        if det.detected():
            n_det += 1
            twf = det.body_bright_frac  # tag-white ON the silhouette (~0 => no tag)
            # masked-tag robustness: blank brightest 40% of the box, re-detect
            ig = brightest_region_mask(gray, det.box, frac=0.4)
            det2 = seeker.detect(im, t, ignore_mask=ig)
            masked_eligible += 1
            mask_fire = det2.detected()
            if mask_fire:
                masked_still_fire += 1
            bdeg = math.degrees(det.bearing_rad)
            print(f"[{i:4d}] DET area={det.area_px:6d} box={det.box} "
                  f"bearing={bdeg:+6.2f}deg range~{det.range_m:5.1f}m "
                  f"conf={det.decision_margin:.2f} body_white={twf:.3f} "
                  f"tags_found={len(tags)} mask_ok={mask_fire}")
        else:
            print(f"[{i:4d}] no-detect                                   "
                  f"           tags_found={len(tags)}")

        rows.append(dict(idx=i, detected=int(det.detected()),
                         bearing_deg=(math.degrees(det.bearing_rad)
                                      if det.detected() else ""),
                         bearing_vert_deg=(math.degrees(det.bearing_vert_rad)
                                           if det.detected() else ""),
                         range_m=(round(det.range_m, 2) if det.detected() else ""),
                         area_px=det.area_px, conf=(round(det.decision_margin, 3)
                                                    if det.detected() else ""),
                         box=(str(det.box) if det.detected() else ""),
                         body_bright_frac=(round(twf, 4) if twf is not None else ""),
                         real_tags_found=len(tags),
                         masked_still_fires=("" if mask_fire is None
                                             else int(mask_fire))))

        # save a few annotated evidence frames
        if k % save_every == 0 and det.detected():
            vis = im.copy()
            x, y, w, h = det.box
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 3)
            u, v = det.centroid
            cv2.drawMarker(vis, (int(u), int(v)), (0, 0, 255),
                           cv2.MARKER_CROSS, 30, 2)
            cv2.line(vis, (int(cx), 0), (int(cx), im.shape[0]), (255, 200, 0), 1)
            cv2.putText(vis, f"frame {i}  bearing {math.degrees(det.bearing_rad):+.1f}deg"
                        f"  range~{det.range_m:.1f}m  conf {det.decision_margin:.2f}",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.putText(vis, f"tag-white ON body: {det.body_bright_frac:.3f}"
                        f"   real AprilTags found: {len(tags)}",
                        (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imwrite(str(OUT / f"annot_{i:04d}.png"), vis)

    # ---- detect-then-track continuity on a contiguous run ----
    run_start = 640  # a contiguous target-present stretch (see trajectory map)
    seeker_t = ClassicalSeeker(fx, fy, cx, cy, tracker="KCF")
    prev_b = None
    cont_deltas = []
    track_hits = 0
    run = range(run_start, min(run_start + 60, len(files)))
    for i in run:
        im = cv2.imread(files[i])
        d = seeker_t.update(im, i / 30.0)
        if d.detected():
            track_hits += 1
            b = math.degrees(d.bearing_rad)
            if prev_b is not None:
                cont_deltas.append(abs(b - prev_b))
            prev_b = b
        else:
            prev_b = None

    # ---- write CSV ----
    csv_path = OUT / "eval_classical.csv"
    with open(csv_path, "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)

    span_det = [r["idx"] for r in rows if r["detected"]]
    print("\n===================== SUMMARY =====================")
    print(f"sampled frames                 : {len(idxs)}")
    print(f"detection rate                 : {n_det}/{len(idxs)} = "
          f"{n_det/len(idxs):.1%}")
    if span_det:
        print(f"detections span frame idx      : {min(span_det)}..{max(span_det)}")
    print(f"--- BODY-NOT-TAG evidence ---")
    print(f"real AprilTags found (any frame): {n_tag_frames}/{len(idxs)} frames "
          f"(0 = tag never decodable here)")
    twfs = [r["body_bright_frac"] for r in rows if r["body_bright_frac"] != ""]
    if twfs:
        print(f"tag-white ON silhouette        : mean {np.mean(twfs):.3f} "
              f"max {np.max(twfs):.3f}  (a tag lock would be ~0.5)")
    print(f"masked-tag test (blank bright  : {masked_still_fire}/{masked_eligible} "
          f"detections still fire on body")
    print(f"--- detect-then-track continuity (frames {run_start}..{run_start+60}) ---")
    print(f"tracked detection rate         : {track_hits}/{len(list(run))}")
    if cont_deltas:
        print(f"frame-to-frame |dbearing|      : median {np.median(cont_deltas):.2f}deg "
              f"mean {np.mean(cont_deltas):.2f}deg max {np.max(cont_deltas):.2f}deg")
    print(f"annotated frames + CSV written : {OUT}")
    print("===================================================")


if __name__ == "__main__":
    main()
