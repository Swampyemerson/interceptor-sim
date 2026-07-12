#!/usr/bin/env python3
"""Render an in-domain YOLO dataset of NATIVE-RESOLUTION CROPS around the
tag-less target (ADR-0074, AUTO-CROP high-res seeker mode).

WHY (ADR-0074 / ADR-0076): the camera is 1280x960; the NN runs at imgsz 640,
so the plain full-frame path DOWNSCALES 1280->640 before every inference,
shrinking an already-small far target (~12-46 native px) even further -- and
a 640-trained model fed that mismatched scale detects noisier (ADR-0076 add
#3). This script captures the OTHER half of the fix: instead of the full
frame, save a 640x640 NATIVE-resolution crop centered on the gt-projected
target pixel, so a model trained on these never sees the target downscaled --
it's fed the same native pixel density AUTO-CROP inference (auto_crop_seeker.
AutoCropSeeker) will feed it at flight time.

IDENTICAL to render_sim_dataset_banked.py in every other respect: same banked
grid (range x lateral x height x yaw x bank x pitch), same
`_euler_zyx_to_quat` teleport, same gt projection via
`m2_detect.PoseTracker.ground_truth_rel_optical()`. Only the OUTPUT changes:
a 640x640 crop (not the full 1280x960 frame) + its box remapped to crop-local
coordinates.

CROP CONVENTION (ADR-0074, shared with auto_crop_seeker.py via crop_geom.py
so the two sides of the training/inference boundary cannot silently drift
apart): crop_size x crop_size (default 640) NATIVE px, CLAMPED so it sits
fully inside the 1280x960 frame (shifted near an edge, never padded).
TRAINING (here) centers the crop on the gt-projected target pixel -- this is
the offline-label-legal use of ground truth (CLAUDE.md gt_* boundary: the
same auto-labeling boundary render_sim_dataset.py/_banked.py already use,
never fed to a live detector). A pose is SKIPPED (no image/label written) if
the target's gt box projects outside the frame entirely, exactly like the
full-frame variant.

HONESTY: ground truth is used ONLY to auto-label TRAINING data offline --
never fed to the detector at inference (CLAUDE.md / ADR-0033 B). The live
no-cheat audit governing the guidance path is unaffected; this is a training
data generator, not a guidance path (test_honesty_seekers.py does not scan
this file for exactly that reason -- it isn't one of the live seeker modules).

SET-POSE QUIRK: this process holds gz subscriptions (camera + pose), so it
can't receive gz service RESPONSES -- teleport via a `gz service` CLI
SUBPROCESS, exactly as render_sim_dataset_banked.py / probe_markerless_range.py.

RUN (sim already up on the markerless world, see probe_markerless_range.py
docstring for the boot line):
  INTERCEPTOR_WORLD_NAME=markerless INTERCEPTOR_TARGET_MODEL=fpv_target_markerless \\
    .venv-seeker/bin/python scripts/seeker/render_sim_dataset_crop.py \\
    --out scripts/seeker/data/quad_dataset_crop --val-frac 0.2
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

from crop_geom import DEFAULT_CROP_SIZE, crop_frame, box_full_to_crop  # noqa: E402

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
            print(f"[render-crop] frame decode error: {exc}")


def _euler_zyx_to_quat(roll, pitch, yaw):
    """Aircraft ZYX (yaw about world +Z-up, then pitch about +Y, then roll
    about +X-forward) -> world orientation quaternion (w, x, y, z). Copied
    verbatim from scripts/m4_target_mover.py / render_sim_dataset_banked.py so
    captured poses match what --orient-to-velocity actually commands live."""
    cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
    cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
    cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
    return (cr * cp * cy + sr * sp * sy,   # w
            sr * cp * cy - cr * sp * sy,   # x
            cr * sp * cy + sr * cp * sy,   # y
            cr * cp * sy - sr * sp * cy)   # z


def teleport(x, y, z, yaw=0.0, roll=0.0, pitch=0.0, timeout_s=4.0) -> bool:
    qw, qx, qy, qz = _euler_zyx_to_quat(roll, pitch, yaw)
    req = (f'name: "{TAG_MODEL_NAME}" position {{ x: {x} y: {y} z: {z} }} '
           f'orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }}')
    cmd = ["gz", "service", "-s", SET_POSE_SERVICE,
           "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
           "--timeout", "2000", "--req", req]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        print(f"[render-crop] teleport failed ({exc})")
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


def project_box_px(rel_optical, extent_m, W, H):
    """Project the optical-frame target vector + physical extent to a
    CLIPPED pixel-space box (x0, y0, x1, y1), or None if behind the camera /
    entirely off-frame. Same projection as render_sim_dataset_banked.
    project_box, but returns un-normalized pixel coords (the crop remap needs
    pixels, not frame-normalized fractions) -- normalization happens AFTER
    the crop offset is known, in main()."""
    x, y, z = float(rel_optical[0]), float(rel_optical[1]), float(rel_optical[2])
    if z <= 0.05:
        return None
    u = CX + FX * (x / z)
    v = CY + FY * (y / z)
    half_w = FX * (extent_m / 2.0) / z
    half_h = FY * (extent_m / 2.0) / z
    x0, x1 = u - half_w, u + half_w
    y0, y1 = v - half_h, v + half_h
    # reject if fully outside the frame
    if x1 < 0 or x0 > W or y1 < 0 or y0 > H:
        return None
    # clip to frame
    x0, x1 = max(0.0, x0), min(float(W), x1)
    y0, y1 = max(0.0, y0), min(float(H), y1)
    bw, bh = x1 - x0, y1 - y0
    if bw < 3 or bh < 3:  # too small to be a useful label
        return None
    return (x0, y0, x1, y1)


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
    ap.add_argument("--banks-deg", default="0",
                    help="target ROLL/bank variations (deg) for aspect diversity")
    ap.add_argument("--pitches-deg", default="0",
                    help="target PITCH variations (deg, +=nose-down) for "
                         "aspect diversity")
    ap.add_argument("--extent-m", type=float, default=0.9,
                    help="target silhouette extent (m) for the bbox")
    ap.add_argument("--crop-size", type=int, default=DEFAULT_CROP_SIZE,
                    help="native px crop size (ADR-0074; MUST match the "
                         "AUTO-CROP inference crop size -- keep at the "
                         "default 640 unless auto_crop_seeker.py's default "
                         "changes too)")
    ap.add_argument("--out", default=os.path.join(HERE, "data", "quad_dataset_crop"))
    ap.add_argument("--val-frac", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0, help="deterministic split seed")
    ap.add_argument("--settle-frames", type=int, default=6)
    args = ap.parse_args()

    ranges = [float(v) for v in args.ranges.split(",") if v.strip()]
    laterals = [float(v) for v in args.laterals.split(",") if v.strip()]
    heights = [float(v) for v in args.heights.split(",") if v.strip()]
    yaws = [float(v) * np.pi / 180.0 for v in args.yaws_deg.split(",") if v.strip()]
    banks = [float(v) * np.pi / 180.0 for v in args.banks_deg.split(",") if v.strip()]
    pitches = [float(v) * np.pi / 180.0 for v in args.pitches_deg.split(",") if v.strip()]
    crop_size = args.crop_size

    img_tr = os.path.join(args.out, "images", "train")
    img_va = os.path.join(args.out, "images", "val")
    lbl_tr = os.path.join(args.out, "labels", "train")
    lbl_va = os.path.join(args.out, "labels", "val")
    for d in (img_tr, img_va, lbl_tr, lbl_va):
        os.makedirs(d, exist_ok=True)

    print(f"[render-crop] world={WORLD_NAME} target={TAG_MODEL_NAME}")
    print(f"[render-crop] camera={IMAGE_TOPIC} crop_size={crop_size}x{crop_size} native px")

    node = Node()
    holder = FrameHolder()
    tracker = PoseTracker()
    if not node.subscribe(Image, IMAGE_TOPIC, holder.on_image):
        print(f"[render-crop] FAILED: subscribe {IMAGE_TOPIC}"); return 2
    if not node.subscribe(Pose_V, POSE_TOPIC, tracker.on_pose_v):
        print(f"[render-crop] FAILED: subscribe {POSE_TOPIC}"); return 2

    print("[render-crop] waiting for first frame + pose...")
    t0 = time.time()
    while holder.count == 0:
        if time.time() - t0 > 20:
            print("[render-crop] FAILED: no frames (is the markerless sim up?)"); return 2
        time.sleep(0.1)

    # deterministic split via a hashed index (no Math.random)
    rng = np.random.RandomState(args.seed)
    n_written = n_skipped = 0
    idx = 0
    for r in ranges:
        for lat in laterals:
            for h in heights:
                for yaw in yaws:
                    for bank in banks:
                        for pitch in pitches:
                            idx += 1
                            if not teleport(r, lat, h, yaw, bank, pitch):
                                n_skipped += 1
                                continue
                            if not wait_new_frames(holder, args.settle_frames):
                                n_skipped += 1
                                continue
                            rel, ok, reason = tracker.ground_truth_rel_optical()
                            if not ok:
                                print(f"[render-crop] gt miss @r={r},lat={lat}: {reason}")
                                n_skipped += 1
                                continue
                            box_px = project_box_px(rel, args.extent_m, holder.w, holder.h)
                            if box_px is None:
                                n_skipped += 1
                                continue
                            x0f, y0f, x1f, y1f = box_px
                            bw_full, bh_full = x1f - x0f, y1f - y0f
                            ctr_u, ctr_v = (x0f + x1f) / 2.0, (y0f + y1f) / 2.0

                            crop, cx0, cy0 = crop_frame(holder.bgr, ctr_u, ctr_v, crop_size)
                            crop_box = box_full_to_crop(
                                (x0f, y0f, bw_full, bh_full), cx0, cy0, crop_size)
                            if crop_box is None:
                                # Should only happen if extent-m/crop-size are
                                # changed such that the box no longer fits the
                                # crop at all -- skip rather than write a bad label.
                                n_skipped += 1
                                continue
                            bx, by, bw, bh = crop_box
                            if bw < 3 or bh < 3:
                                n_skipped += 1
                                continue
                            box_norm = ((bx + bw / 2.0) / crop_size,
                                       (by + bh / 2.0) / crop_size,
                                       bw / crop_size, bh / crop_size)

                            is_val = rng.random_sample() < args.val_frac
                            img_dir = img_va if is_val else img_tr
                            lbl_dir = lbl_va if is_val else lbl_tr
                            stem = (f"m{idx:05d}_r{r:04.1f}_y{lat:+.2f}_h{h:.2f}"
                                    f"_a{int(yaw*180/np.pi):+03d}"
                                    f"_b{int(round(bank*180/np.pi)):+03d}"
                                    f"_p{int(round(pitch*180/np.pi)):+03d}")
                            cv2.imwrite(os.path.join(img_dir, stem + ".png"), crop)
                            with open(os.path.join(lbl_dir, stem + ".txt"), "w") as fh:
                                fh.write(f"0 {box_norm[0]:.6f} {box_norm[1]:.6f} "
                                        f"{box_norm[2]:.6f} {box_norm[3]:.6f}\n")
                            n_written += 1

    node.unsubscribe(IMAGE_TOPIC)
    node.unsubscribe(POSE_TOPIC)

    data_yaml = os.path.join(args.out, "data.yaml")
    with open(data_yaml, "w") as fh:
        fh.write(
            f"# In-domain NATIVE-RES CROP dataset ({TAG_MODEL_NAME} in world "
            f"{WORLD_NAME}), ADR-0074.\n"
            f"# Auto-labeled from ground truth (SCORING-only boundary, ADR-0033 B).\n"
            f"# crop_size={crop_size}x{crop_size} native px, centered on the "
            "gt-projected target pixel, clamped fully inside the 1280x960 frame.\n"
            f"# MUST match auto_crop_seeker.py's inference crop convention "
            "(crop_geom.py is shared by both).\n"
            f"path: {os.path.abspath(args.out)}\n"
            "train: images/train\n"
            "val: images/val\n"
            "nc: 1\n"
            "names: ['drone']\n"
        )
    n_tr = len(os.listdir(img_tr))
    n_va = len(os.listdir(img_va))
    print(f"[render-crop] wrote {n_written} labeled {crop_size}x{crop_size} crops "
          f"(train {n_tr}, val {n_va}); skipped {n_skipped}.")
    print(f"[render-crop] dataset: {data_yaml}")
    print("[render-crop] next: fine-tune with train_drone_finetune.py --data " + data_yaml)
    return 0 if n_written > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
