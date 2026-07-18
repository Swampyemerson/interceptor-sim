#!/usr/bin/env python3
"""Synthetic AprilTag-in-frame generator -- a PLUMBING FIXTURE for
real_data_pipeline_run.sh --dry-run, so the real-hardware pipeline can be
proven end-to-end WITHOUT needing tripod footage yet (builder directive
2026-07-17).

WHAT: composites the repo's actual tag36h11 texture
(models/apriltag_target/tag36h11_00000.png -- the same asset the Gazebo
apriltag_target model renders) onto background crops pulled from EXISTING
sim capture frames (scripts/seeker/data/**), at a pixel size that matches a
chosen physical tag size + range under the SIM camera's own intrinsics
(same FX/FY/CX/CY/W/H constants as resolution_probe.py). pupil-apriltags
then decodes REAL PIXELS (not a math mock), so this exercises the same
image-read -> detect -> pose -> project -> YOLO-label path
autolabel_from_apriltag.py uses on true tripod footage.

HONESTY / SCOPE: this is a plumbing fixture, not a vision benchmark. A
pasted flat tag carries none of a real lens's distortion, motion blur, or
exposure realism, and the "drone" a downstream detector would need to learn
is never rendered here (only the tag). Any accuracy number produced further
down the dry-run chain (e.g. resolution_probe.py's table) is NOT a claim --
only "did every stage run and produce well-formed artifacts" is being
tested. Real tripod footage is still required before any accuracy claim
(ADR-0076 add #18h).

Usage:
    scripts/seeker/synth_tag_frames.py --out /tmp/x/flightA --run-tag flightA --seed 1
    scripts/seeker/synth_tag_frames.py --self-test     # composite+decode roundtrip, no --out needed

Requires cv2 + numpy + pupil-apriltags (self-test only). Run under .venv-seeker.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
TAG_PNG = os.path.join(REPO, "models", "apriltag_target", "tag36h11_00000.png")
BG_GLOB = os.path.join(HERE, "data", "quad_dataset_v2", "images", "val", "*.png")

# Same constants as resolution_probe.py: gz_x500_mono_cam @ 1280x960.
FX = FY = 539.9363327026367
CX, CY = 640.0, 480.0
W, H = 1280, 960

DEFAULT_RANGES = (3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0)
DEFAULT_OFFSETS = (-1.5, 0.0, 1.5)

# tag36h11_00000.png is the FULL textured plane (models/apriltag_target/model.sdf:
# 0.625 m/side), but pupil-apriltags' `tag_size` measures the inner BLACK-SQUARE
# marker, which the model.sdf comment fixes at 0.8x the plane (0.5 m of 0.625 m).
# So a pasted image of PHYSICAL width `tag_size_m` must be rendered at the pixel
# size for `tag_size_m / PNG_BORDER_FRAC` m, or pose recovery reads a systematically
# LARGER range than requested (confirmed empirically: 8 m -> decoded ~10 m, a
# constant ~1.25x = 1/0.8 bias, before this correction).
PNG_BORDER_FRAC = 0.8


def _load_background(rng, cv2):
    paths = sorted(glob.glob(BG_GLOB))
    if paths:
        p = paths[int(rng.integers(0, len(paths)))]
        img = cv2.imread(p)
        if img is not None:
            return cv2.resize(img, (W, H))
    # Fallback if the sim dataset isn't present: flat gray + noise canvas.
    base = np.full((H, W, 3), 90, dtype=np.int16)
    base += rng.integers(-8, 9, (H, W, 3))
    return np.clip(base, 0, 255).astype(np.uint8)


def composite_tag(bg, tag_img, cv2, range_m, tag_size_m, off_x_m=0.0, off_y_m=0.0):
    """Paste tag_img at world offset (off_x_m right, off_y_m down of boresight)
    and `range_m` forward, sized by pinhole projection under FX/FY so a
    pose-recovery on the result should decode back to ~range_m. Returns
    (frame, u_px, v_px, tag_px_size) or None if it would land off-frame or
    too small to matter."""
    if range_m <= 1e-3:
        return None
    px = FX * (tag_size_m / PNG_BORDER_FRAC) / range_m
    size = int(round(px))
    if size < 8:
        return None  # too small to decode -- not a useful fixture frame
    u = CX + FX * off_x_m / range_m
    v = CY + FY * off_y_m / range_m
    x0, y0 = int(round(u - size / 2)), int(round(v - size / 2))
    x1, y1 = x0 + size, y0 + size
    dx0, dy0 = max(0, x0), max(0, y0)
    dx1, dy1 = min(W, x1), min(H, y1)
    if dx1 <= dx0 or dy1 <= dy0:
        return None
    tag_resized = cv2.resize(tag_img, (size, size), interpolation=cv2.INTER_AREA)
    sx0, sy0 = dx0 - x0, dy0 - y0
    sx1, sy1 = sx0 + (dx1 - dx0), sy0 + (dy1 - dy0)
    frame = bg.copy()
    frame[dy0:dy1, dx0:dx1] = tag_resized[sy0:sy1, sx0:sx1]
    return frame, u, v, size


def self_test():
    """Composite a tag at three ranges and confirm pupil-apriltags decodes
    each one back to roughly the requested range (loose tolerance -- this is
    a plumbing check, not a precision claim). No --out needed."""
    import cv2
    from pupil_apriltags import Detector

    tag_img = cv2.imread(TAG_PNG)
    if tag_img is None:
        print(f"[self-test] FAIL: cannot read tag texture {TAG_PNG}")
        return False
    rng = np.random.default_rng(0)
    det = Detector(families="tag36h11")
    ok_all = True
    for test_range in (4.0, 7.0, 10.0):
        bg = _load_background(rng, cv2)
        result = composite_tag(bg, tag_img, cv2, test_range, 0.5)
        if result is None:
            print(f"[self-test] range {test_range} m: composite returned None : FAIL")
            ok_all = False
            continue
        frame, u, v, size = result
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dets = det.detect(gray, estimate_tag_pose=True,
                          camera_params=(FX, FY, CX, CY), tag_size=0.5)
        if not dets:
            print(f"[self-test] range {test_range} m: composited {size}px -> NOT decoded : FAIL")
            ok_all = False
            continue
        decoded_range = float(np.linalg.norm(dets[0].pose_t.reshape(3)))
        err = abs(decoded_range - test_range)
        ok = err < 0.5 * test_range
        print(f"[self-test] range {test_range} m: composited {size}px -> decoded "
              f"{decoded_range:.2f} m (err {err:.2f}) : {'OK' if ok else 'FAIL'}")
        ok_all = ok_all and ok
    print(f"[self-test] {'PASS' if ok_all else 'FAIL'}")
    return ok_all


def run(args):
    import cv2

    tag_img = cv2.imread(TAG_PNG)
    if tag_img is None:
        raise SystemExit(f"cannot read tag texture: {TAG_PNG}")
    rng = np.random.default_rng(args.seed)
    frames_dir = os.path.join(args.out, "frames")
    neg_dir = os.path.join(args.out, "frames_negfree")
    os.makedirs(frames_dir, exist_ok=True)
    os.makedirs(neg_dir, exist_ok=True)

    ranges = args.ranges or DEFAULT_RANGES
    n_written = n_skipped = 0
    i = 0
    for r in ranges:
        for off_x in DEFAULT_OFFSETS:
            i += 1
            bg = _load_background(rng, cv2)
            result = composite_tag(bg, tag_img, cv2, float(r), args.tag_size, off_x_m=off_x)
            if result is None:
                n_skipped += 1
                continue
            frame, _, _, _ = result
            fn = f"{args.run_tag}_{i:04d}_r{r:05.1f}_x{off_x:+.1f}.png"
            cv2.imwrite(os.path.join(frames_dir, fn), frame)
            n_written += 1

    for j in range(args.n_negatives):
        bg = _load_background(rng, cv2)
        cv2.imwrite(os.path.join(neg_dir, f"{args.run_tag}_neg{j:04d}.png"), bg)

    calib = dict(fx=FX, fy=FY, cx=CX, cy=CY,
                resolution=dict(width=W, height=H), dist_coeffs=[0.0] * 5)
    with open(os.path.join(args.out, "calib.json"), "w") as f:
        json.dump(calib, f, indent=2)

    print(f"[synth] wrote {n_written} tag frames -> {frames_dir} "
          f"({n_skipped} skipped: off-frame/too-small)")
    print(f"[synth] wrote {args.n_negatives} target-free frames -> {neg_dir}")
    print(f"[synth] calib -> {os.path.join(args.out, 'calib.json')}")
    return 0 if n_written > 0 else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", help="output dir (writes frames/, frames_negfree/, calib.json)")
    ap.add_argument("--run-tag", default="synth", help="filename prefix, disambiguates flights")
    ap.add_argument("--tag-size", type=float, default=0.5,
                    help="physical tag size m (0.5 matches models/apriltag_target)")
    ap.add_argument("--n-negatives", type=int, default=6, help="target-free frames to write")
    ap.add_argument("--ranges", type=float, nargs="+", default=None,
                    help=f"ranges m to composite at (default {DEFAULT_RANGES})")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--self-test", action="store_true", help="composite+decode roundtrip; exit 0/1")
    args = ap.parse_args()
    if args.self_test:
        return 0 if self_test() else 1
    if not args.out:
        ap.error("--out is required (or --self-test)")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
