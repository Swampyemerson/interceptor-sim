#!/usr/bin/env python3
"""T17 (ADR-0047): label the T16 full-sweep rig capture with YOLO boxes,
by PROJECTING the capture's gt target world position into each rig camera.

WHAT: reads logs/rig_captures/<run>/{index.csv,capture_meta.json} (produced
offline by scripts/rig_snapshot_capture.py -- no sim needed here, this is a
pure geometry + filesystem tool) and, for each captured pose and each of the
rig's two cameras (left offset +1.0 m, right offset -1.0 m along the rig's
local Y, per models/ground_stereo_rig/model.sdf), computes the camera-optical
-frame relative vector to the target and projects the target's physical
extent through the rig's pinhole intrinsics into a clipped YOLO box. Mirrors
scripts/seeker/render_sim_dataset.py's project_box() (same pinhole model,
same clipping/reject-if-too-small rule) -- see that file for the original.

CAMERA GEOMETRY (generalizes render_sim_dataset.py's onboard-camera chain,
scripts/m2_detect.py's PoseTracker.ground_truth_rel_optical, to the STATIC
ground rig -- no live pose topic needed since the rig's pose is fixed and
known from capture_meta.json):
  rig_pose_xyz_yaw = (rx, ry, rz, yaw) -- models/ground_stereo_rig/model.sdf
  places rig_left/rig_right at link-local (0, +/-1.0, 0), boresight along
  link-local +X, no toe-in. World camera position:
      cam_world = rig_xyz + R_z(yaw) @ (0, +/-1.0, 0)
  World-to-link rotation is R_z(yaw) (roll=pitch=0 for this static model).
  Then, exactly as scripts/m2_detect.py's docstring derives for the onboard
  camera (gz sensor frame x-fwd/y-left/z-up -> OpenCV optical z-fwd/x-right/
  y-down):
      rel_world = target_world - cam_world
      rel_link  = R_z(yaw)^T @ rel_world
      x_opt = -rel_link[1]; y_opt = -rel_link[2]; z_opt = rel_link[0]

TARGET EXTENT: fpv_target_markerless's real geometric silhouette, measured
BY HAND from models/fpv_target_markerless/model.sdf (prop-disc tips are the
outermost points: radial center 0.353553 m + prop radius 0.095 m = 0.4486 m
from the body centerline in BOTH local Y and Z) -- span = 2 * 0.4486 =
0.897 m, matching scripts/seeker/render_sim_dataset.py's ALREADY-ESTABLISHED
--extent-m default of 0.9 (independently confirms that convention, not a new
number). A +/-15% multiplicative margin is applied on top (PADDING_FACTOR)
to cover anti-aliasing / prop-blur transparency fuzz at these very small
(~10-30 px) sizes -- the task's explicit "sensible margin" ask. This is
DELIBERATELY the real measured silhouette (0.9 m), not stereo_design.md's
worst-case PLANNING assumption of 0.3 m used to derive the n_90=10px
detection-floor number -- see the T17 eval report for the reconciliation
(0.9 m is what is actually RENDERED and must be pixel-matched for training
labels; 0.3 m was a conservative range-budget assumption, not this
specific Gazebo model's true size).

HONESTY (same boundary as render_sim_dataset.py / rig_snapshot_capture.py,
CLAUDE.md's gt_* rule): gt_target_{x,y,z} here is capture-time-only, used
ONLY to manufacture offline training LABELS -- exactly as a human labeler's
boxes would be. The trained detector reads pixels only at inference. This
tool touches no gz-transport, no live sim -- pure filesystem + numpy.

RUN (no sim needed):
    .venv-seeker/bin/python scripts/seeker/label_rig_captures.py \\
        --rig-dir logs/rig_captures/full_sweep_20260709T015530Z \\
        --out scripts/seeker/data/ground_v1_positives
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))

# Measured from models/fpv_target_markerless/model.sdf (see module docstring):
# prop-disc-tip span in local Y and Z, both directions.
TARGET_EXTENT_M = 0.9  # matches render_sim_dataset.py's --extent-m default
PADDING_FACTOR = 1.15  # "sensible margin" for AA / prop-blur fuzz at small sizes

MIN_BOX_PX = 3.0  # same floor as render_sim_dataset.py's project_box


def rot_z(yaw_rad: float) -> np.ndarray:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def project_box(rel_optical, extent_m, W, H, fx, fy, cx, cy, padding=1.0):
    """Same pinhole model + clip/reject rule as render_sim_dataset.py's
    project_box, parameterized for the rig's own intrinsics (fx=fy derived
    from horizontal_fov, square-pixel gz/ogre camera assumption -- matches
    the onboard camera's FX==FY convention in render_sim_dataset.py)."""
    x, y, z = float(rel_optical[0]), float(rel_optical[1]), float(rel_optical[2])
    if z <= 0.05:
        return None, "behind camera"
    u = cx + fx * (x / z)
    v = cy + fy * (y / z)
    half_w = padding * fx * (extent_m / 2.0) / z
    half_h = padding * fy * (extent_m / 2.0) / z
    x0, x1 = u - half_w, u + half_w
    y0, y1 = v - half_h, v + half_h
    if x1 < 0 or x0 > W or y1 < 0 or y0 > H:
        return None, "fully off-frame"
    x0c, x1c = max(0.0, x0), min(float(W), x1)
    y0c, y1c = max(0.0, y0), min(float(H), y1)
    bw, bh = x1c - x0c, y1c - y0c
    if bw < MIN_BOX_PX or bh < MIN_BOX_PX:
        return None, "clipped too small"
    cxn, cyn = (x0c + x1c) / 2.0, (y0c + y1c) / 2.0
    box_px = (x0c, y0c, x1c, y1c)
    box_norm = (cxn / W, cyn / H, bw / W, bh / H)
    return (box_norm, box_px), ""


def load_capture(rig_dir):
    with open(os.path.join(rig_dir, "capture_meta.json")) as fh:
        meta = json.load(fh)
    with open(os.path.join(rig_dir, "index.csv"), newline="") as fh:
        rows = list(csv.DictReader(fh))
    return meta, rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rig-dir", default=os.path.join(
        REPO, "logs", "rig_captures", "full_sweep_20260709T015530Z"))
    ap.add_argument("--out", default=os.path.join(HERE, "data", "ground_v1_positives"))
    ap.add_argument("--extent-m", type=float, default=TARGET_EXTENT_M)
    ap.add_argument("--padding", type=float, default=PADDING_FACTOR)
    ap.add_argument("--n-sanity", type=int, default=6)
    args = ap.parse_args()

    meta, rows = load_capture(args.rig_dir)
    W, H = meta["resolution"]
    hfov = meta["rig_hfov_rad"]
    fx = fy = (W / 2.0) / math.tan(hfov / 2.0)
    cx, cy = W / 2.0, H / 2.0
    rig_x, rig_y, rig_z, yaw = meta["rig_pose_xyz_yaw"]
    rig_pos = np.array([rig_x, rig_y, rig_z])
    R = rot_z(yaw)
    baseline_half = meta["rig_baseline_m"] / 2.0
    cam_offsets = {"left": np.array([0.0, +baseline_half, 0.0]),
                   "right": np.array([0.0, -baseline_half, 0.0])}
    cam_world = {side: rig_pos + R @ off for side, off in cam_offsets.items()}

    print(f"[label] rig_dir={args.rig_dir}")
    print(f"[label] {W}x{H} hfov={hfov} -> fx=fy={fx:.2f}px cx={cx} cy={cy}")
    print(f"[label] rig_pos={rig_pos.tolist()} yaw={yaw}")
    print(f"[label] cam_world left={cam_world['left'].tolist()} right={cam_world['right'].tolist()}")
    print(f"[label] extent_m={args.extent_m} padding={args.padding}")

    labels_dir = {s: os.path.join(args.out, f"labels_{s}") for s in ("left", "right")}
    for d in labels_dir.values():
        os.makedirs(d, exist_ok=True)
    sanity_dir = os.path.join(args.out, "sanity")
    os.makedirs(sanity_dir, exist_ok=True)

    meta_rows = []
    n_written = n_skipped = 0
    for row in rows:
        seq = int(row["seq"])
        target_world = np.array([float(row["gt_target_x"]), float(row["gt_target_y"]),
                                  float(row["gt_target_z"])])
        for side in ("left", "right"):
            rel_world = target_world - cam_world[side]
            rel_link = R.T @ rel_world
            rel_optical = np.array([-rel_link[1], -rel_link[2], rel_link[0]])
            range_m = float(np.linalg.norm(rel_world))
            off_boresight_deg = math.degrees(math.atan2(
                math.hypot(rel_link[1], rel_link[2]), rel_link[0]))

            img_path = os.path.join(args.rig_dir, "left" if side == "left" else "right",
                                     f"{seq:05d}.png")
            lbl_path = os.path.join(labels_dir[side], f"{seq:05d}.txt")
            result, reason = project_box(rel_optical, args.extent_m, W, H, fx, fy, cx, cy,
                                          padding=args.padding)
            if result is None:
                n_skipped += 1
                meta_rows.append({
                    "seq": seq, "camera": side, "dash_direction": row["dash_direction"],
                    "range_m": f"{range_m:.4f}", "off_boresight_deg": f"{off_boresight_deg:.4f}",
                    "written": 0, "reason": reason,
                    "image_path": os.path.relpath(img_path, REPO),
                    "label_path": "", "bbox_w_px": "", "bbox_h_px": "",
                })
                continue
            box_norm, box_px = result
            with open(lbl_path, "w") as fh:
                fh.write(f"0 {box_norm[0]:.6f} {box_norm[1]:.6f} {box_norm[2]:.6f} {box_norm[3]:.6f}\n")
            bw_px = box_px[2] - box_px[0]
            bh_px = box_px[3] - box_px[1]
            n_written += 1
            meta_rows.append({
                "seq": seq, "camera": side, "dash_direction": row["dash_direction"],
                "range_m": f"{range_m:.4f}", "off_boresight_deg": f"{off_boresight_deg:.4f}",
                "written": 1, "reason": "",
                "image_path": os.path.relpath(img_path, REPO),
                "label_path": os.path.relpath(lbl_path, REPO),
                "bbox_w_px": f"{bw_px:.2f}", "bbox_h_px": f"{bh_px:.2f}",
            })

    meta_csv = os.path.join(args.out, "meta.csv")
    with open(meta_csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(meta_rows[0].keys()))
        w.writeheader()
        w.writerows(meta_rows)

    written = [r for r in meta_rows if r["written"] == 1]
    ranges = [float(r["range_m"]) for r in written]
    bws = [float(r["bbox_w_px"]) for r in written]
    bhs = [float(r["bbox_h_px"]) for r in written]
    print(f"[label] wrote {n_written} labels, skipped {n_skipped} "
          f"(of {len(rows) * 2} row*camera combos)")
    if ranges:
        print(f"[label] range_m: min={min(ranges):.2f} max={max(ranges):.2f} "
              f"mean={sum(ranges)/len(ranges):.2f}")
        print(f"[label] bbox width px: min={min(bws):.2f} max={max(bws):.2f} "
              f"mean={sum(bws)/len(bws):.2f}")
        print(f"[label] bbox height px: min={min(bhs):.2f} max={max(bhs):.2f} "
              f"mean={sum(bhs)/len(bhs):.2f}")
    print(f"[label] meta: {meta_csv}")

    # --- 6 sanity crops: draw the box on the full frame + save a margin-padded
    # crop, spanning near/far range and left/right camera for visual review.
    written_sorted_by_range = sorted(written, key=lambda r: float(r["range_m"]))
    picks = []
    if written_sorted_by_range:
        n = len(written_sorted_by_range)
        k = args.n_sanity
        idxs = sorted(set(min(n - 1, round(i * (n - 1) / (k - 1))) for i in range(k)))
        # pad out to k distinct indices if rounding collapsed any
        cand = 0
        while len(idxs) < k and cand < n:
            if cand not in idxs:
                idxs.append(cand)
                idxs = sorted(set(idxs))
            cand += 1
        idxs = sorted(idxs)[:k]
        picks = [written_sorted_by_range[i] for i in idxs]
        # ensure at least one right-camera sample is included
        if not any(p["camera"] == "right" for p in picks):
            for r in written_sorted_by_range:
                if r["camera"] == "right":
                    # swap in place of the last pick to keep the count at k
                    picks[-1] = r
                    break

    print(f"[label] writing {len(picks)} sanity crops -> {sanity_dir}")
    for i, r in enumerate(picks):
        img_path = os.path.join(REPO, r["image_path"])
        img = cv2.imread(img_path)
        if img is None:
            print(f"[label] sanity WARNING: could not read {img_path}")
            continue
        with open(os.path.join(REPO, r["label_path"])) as fh:
            _, xc, yc, bw, bh = fh.read().split()
        xc, yc, bw, bh = float(xc) * W, float(yc) * H, float(bw) * W, float(bh) * H
        x0, y0, x1, y1 = int(xc - bw / 2), int(yc - bh / 2), int(xc + bw / 2), int(yc + bh / 2)
        vis = img.copy()
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 0, 255), 2)
        stem = f"{i}_{r['camera']}_seq{r['seq']}_r{float(r['range_m']):.0f}m"
        cv2.imwrite(os.path.join(sanity_dir, stem + "_full.png"), vis)
        # margin-padded crop around the box for a close-up look
        m = 60
        cx0, cy0 = max(0, x0 - m), max(0, y0 - m)
        cx1, cy1 = min(W, x1 + m), min(H, y1 + m)
        crop = vis[cy0:cy1, cx0:cx1]
        # upscale crop for visibility (target is tiny)
        crop_big = cv2.resize(crop, None, fx=6.0, fy=6.0, interpolation=cv2.INTER_NEAREST)
        cv2.imwrite(os.path.join(sanity_dir, stem + "_crop6x.png"), crop_big)
        print(f"[label] sanity[{i}]: seq={r['seq']} cam={r['camera']} "
              f"range={r['range_m']}m off_boresight={r['off_boresight_deg']}deg "
              f"bbox={r['bbox_w_px']}x{r['bbox_h_px']}px -> {stem}")

    return 0 if n_written > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
