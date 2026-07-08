#!/usr/bin/env python3
"""Render an in-domain YOLO dataset of the tag-less target (ADR-0033 item 2, B).

WHY: off-the-shelf detectors (COCO YOLOv8n, the MIT drone YOLO11x) do NOT transfer
to the synthetic sim body -- the markerless range probe fired only ~2 m
(docs/seeker_prototype_results.md, logs/markerless_probe). To make the markerless
seeker actually WORK in sim (Option B, a sim/demo aid -- real hardware uses the MIT
model), we fine-tune a single-class 'drone' nano on IN-DOMAIN renders: the exact
`models/fpv_target_markerless` body, in the exact deployment world, from the exact
gz_x500_mono_cam. This tool produces that dataset with PIXEL-EXACT labels.

HOW: with PX4 SITL + gz_x500_mono_cam already running on the MARKERLESS world (see
probe_markerless_range.py for the boot line), it teleports the target across a grid
of (range x lateral x height [x yaw]) poses, and for each grabs one onboard frame
AND the ground-truth camera-optical-frame target vector (via m2_detect.PoseTracker,
the same transform chain M2 validated). It projects that vector + the known body
extent through the fixed intrinsics into a 2-D box, and writes YOLO-format labels.

LABEL HONESTY: ground truth is used ONLY to auto-label TRAINING data offline --
never fed to the detector at inference. This is the same scoring-vs-guidance
boundary as gt_* (CLAUDE.md): a fine-tune's labels come from gt exactly as a human
labeler's boxes would; the trained detector then reads pixels only. The live
no-cheat audit (which governs the guidance path) is unaffected.

SET-POSE QUIRK: this process holds gz subscriptions (camera + pose), so it can't
receive gz service RESPONSES -- teleport via a `gz service` CLI SUBPROCESS, exactly
as probe_markerless_range.py / m4_intercept.preplace_target do.

RUN (sim already up on the markerless world, see probe docstring):
  INTERCEPTOR_WORLD_NAME=markerless INTERCEPTOR_TARGET_MODEL=fpv_target_markerless \
    .venv-seeker/bin/python scripts/seeker/render_sim_dataset.py \
    --out scripts/seeker/data/sim_dataset --val-frac 0.2
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
REPO = os.path.dirname(SCRIPTS)
os.environ.setdefault("INTERCEPTOR_WORLD_NAME", "markerless")
os.environ.setdefault("INTERCEPTOR_TARGET_MODEL", "fpv_target_markerless")
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, HERE)

import cv2  # noqa: E402

from m2_detect import (  # noqa: E402
    IMAGE_TOPIC, POSE_TOPIC, WORLD_NAME, TAG_MODEL_NAME, PoseTracker,
)
from gz.transport13 import Node  # noqa: E402
from gz.msgs10.image_pb2 import Image  # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402

FX = FY = 539.936
CX, CY = 640.0, 480.0
SET_POSE_SERVICE = f"/world/{WORLD_NAME}/set_pose"


class FrameHolder:
    def __init__(self) -> None:
        self.count = 0
        self.bgr = None
        self.w = 0
        self.h = 0

    def on_image(self, msg: Image) -> None:
        try:
            w, h = msg.width, msg.height
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            if arr.size < w * h * 3:
                return
            self.bgr = cv2.cvtColor(arr[: w * h * 3].reshape(h, w, 3),
                                    cv2.COLOR_RGB2BGR)
            self.w, self.h = w, h
            self.count += 1
        except Exception as exc:  # pragma: no cover
            print(f"[render] frame decode error: {exc}")


def teleport(x, y, z, yaw=0.0, timeout_s=4.0) -> bool:
    # quaternion for a yaw about world Z (identity when yaw=0 -> same as the world
    # file's baked facing)
    qz, qw = np.sin(yaw / 2.0), np.cos(yaw / 2.0)
    req = (f'name: "{TAG_MODEL_NAME}" position {{ x: {x} y: {y} z: {z} }} '
           f'orientation {{ x: 0 y: 0 z: {qz} w: {qw} }}')
    cmd = ["gz", "service", "-s", SET_POSE_SERVICE,
           "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
           "--timeout", "2000", "--req", req]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        print(f"[render] teleport failed ({exc})")
        return False
    return r.returncode == 0


def wait_new_frames(holder: FrameHolder, n: int, timeout_s=8.0) -> bool:
    start = holder.count
    t0 = time.time()
    while holder.count < start + n:
        if time.time() - t0 > timeout_s:
            return False
        time.sleep(0.03)
    return True


def project_box(rel_optical, extent_m, W, H):
    """Project the optical-frame target vector + physical extent to a clipped
    YOLO box (cx,cy,w,h normalized) or None if behind/entirely off-frame."""
    x, y, z = float(rel_optical[0]), float(rel_optical[1]), float(rel_optical[2])
    if z <= 0.05:
        return None
    u = CX + FX * (x / z)
    v = CY + FY * (y / z)
    half_w = FX * (extent_m / 2.0) / z
    half_h = FY * (extent_m / 2.0) / z
    x0, x1 = u - half_w, u + half_w
    y0, y1 = v - half_h, v + half_h
    # reject if fully outside
    if x1 < 0 or x0 > W or y1 < 0 or y0 > H:
        return None
    # clip to frame
    x0, x1 = max(0.0, x0), min(float(W), x1)
    y0, y1 = max(0.0, y0), min(float(H), y1)
    bw, bh = x1 - x0, y1 - y0
    if bw < 3 or bh < 3:  # too small to be a useful label
        return None
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    return (cx / W, cy / H, bw / W, bh / H)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ranges", default="2,2.5,3,3.5,4,5,6,7,8,10,12",
                    help="forward ranges (m)")
    ap.add_argument("--laterals", default="-1.5,-0.75,0,0.75,1.5",
                    help="lateral offsets (m, world +Y)")
    ap.add_argument("--heights", default="0.0,0.25,0.5",
                    help="target world Z heights (m)")
    ap.add_argument("--yaws-deg", default="0,25,-25",
                    help="target yaw variations (deg) for silhouette diversity")
    ap.add_argument("--extent-m", type=float, default=0.9,
                    help="target silhouette extent (m) for the bbox")
    ap.add_argument("--out", default=os.path.join(HERE, "data", "sim_dataset"))
    ap.add_argument("--val-frac", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0, help="deterministic split seed")
    ap.add_argument("--settle-frames", type=int, default=6)
    args = ap.parse_args()

    ranges = [float(v) for v in args.ranges.split(",") if v.strip()]
    laterals = [float(v) for v in args.laterals.split(",") if v.strip()]
    heights = [float(v) for v in args.heights.split(",") if v.strip()]
    yaws = [float(v) * np.pi / 180.0 for v in args.yaws_deg.split(",") if v.strip()]

    img_tr = os.path.join(args.out, "images", "train")
    img_va = os.path.join(args.out, "images", "val")
    lbl_tr = os.path.join(args.out, "labels", "train")
    lbl_va = os.path.join(args.out, "labels", "val")
    for d in (img_tr, img_va, lbl_tr, lbl_va):
        os.makedirs(d, exist_ok=True)

    print(f"[render] world={WORLD_NAME} target={TAG_MODEL_NAME}")
    print(f"[render] camera={IMAGE_TOPIC}")

    node = Node()
    holder = FrameHolder()
    tracker = PoseTracker()
    if not node.subscribe(Image, IMAGE_TOPIC, holder.on_image):
        print(f"[render] FAILED: subscribe {IMAGE_TOPIC}"); return 2
    if not node.subscribe(Pose_V, POSE_TOPIC, tracker.on_pose_v):
        print(f"[render] FAILED: subscribe {POSE_TOPIC}"); return 2

    print("[render] waiting for first frame + pose...")
    t0 = time.time()
    while holder.count == 0:
        if time.time() - t0 > 20:
            print("[render] FAILED: no frames (is the markerless sim up?)"); return 2
        time.sleep(0.1)

    # deterministic split via a hashed index (no Math.random)
    rng = np.random.RandomState(args.seed)
    n_written = n_skipped = 0
    idx = 0
    for r in ranges:
        for lat in laterals:
            for h in heights:
                for yaw in yaws:
                    idx += 1
                    if not teleport(r, lat, h, yaw):
                        n_skipped += 1
                        continue
                    if not wait_new_frames(holder, args.settle_frames):
                        n_skipped += 1
                        continue
                    rel, ok, reason = tracker.ground_truth_rel_optical()
                    if not ok:
                        print(f"[render] gt miss @r={r},lat={lat}: {reason}")
                        n_skipped += 1
                        continue
                    box = project_box(rel, args.extent_m, holder.w, holder.h)
                    if box is None:
                        n_skipped += 1
                        continue
                    is_val = rng.random_sample() < args.val_frac
                    img_dir = img_va if is_val else img_tr
                    lbl_dir = lbl_va if is_val else lbl_tr
                    stem = f"m{idx:05d}_r{r:04.1f}_y{lat:+.2f}_h{h:.2f}_a{int(yaw*180/np.pi):+03d}"
                    cv2.imwrite(os.path.join(img_dir, stem + ".png"), holder.bgr)
                    with open(os.path.join(lbl_dir, stem + ".txt"), "w") as fh:
                        fh.write(f"0 {box[0]:.6f} {box[1]:.6f} {box[2]:.6f} {box[3]:.6f}\n")
                    n_written += 1

    node.unsubscribe(IMAGE_TOPIC)
    node.unsubscribe(POSE_TOPIC)

    data_yaml = os.path.join(args.out, "data.yaml")
    with open(data_yaml, "w") as fh:
        fh.write(
            f"# In-domain sim dataset ({TAG_MODEL_NAME} in world {WORLD_NAME}).\n"
            f"# Auto-labeled from ground truth (SCORING-only boundary, ADR-0033 B).\n"
            f"path: {os.path.abspath(args.out)}\n"
            "train: images/train\n"
            "val: images/val\n"
            "nc: 1\n"
            "names: ['drone']\n"
        )
    n_tr = len(os.listdir(img_tr))
    n_va = len(os.listdir(img_va))
    print(f"[render] wrote {n_written} labeled frames "
          f"(train {n_tr}, val {n_va}); skipped {n_skipped}.")
    print(f"[render] dataset: {data_yaml}")
    print("[render] next: fine-tune with train_drone_finetune.py --data " + data_yaml)
    return 0 if n_written > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
