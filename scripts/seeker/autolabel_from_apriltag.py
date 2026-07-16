#!/usr/bin/env python3
"""AprilTag-SUPERVISED auto-labeler for REAL-hardware seeker data (the minimal-
manual-effort path, builder directive 2026-07-16: "do as little feeding in
specific data as possible").

THE IDEA: the AprilTag is allowed for CALIBRATION. So fly the TARGET DRONE
carrying a tag36h11 placard, record the interceptor's camera frames, and this
tool AUTO-GENERATES the YOLO 'drone' bounding-box labels from the tag's detected
pose -- NO hand-labeling. You then fine-tune a MARKERLESS detector on those
auto-labels; at inference the tag is off (the deployed seeker never needs it).
This is the real-hardware twin of gen_sim_dataset.py (which auto-labels sim
frames from GROUND TRUTH); here the tag pose IS the real-world ground truth.

    fly tagged target (APPROACHING passes)  ->  record frames
        -> autolabel_from_apriltag.py  ->  YOLO dataset  ->  fine-tune markerless
        -> shadow-eval on HELD-OUT real flights (approach-recall, ADR-0076 add #18h)

HONESTY: the tag is a TRAINING-TIME label source only (same class as gt in
gen_sim_dataset). The DEPLOYED detector is markerless and re-earns the no-cheat
audit. Do NOT leave the tag on the operational target.

Pipeline math: pupil-apriltags returns the tag translation `pose_t` in the
camera optical frame (x right, y down, z forward); we project the DRONE's
physical extent at that range into a pixel box via gen_sim_dataset.project_to_bbox
(reused verbatim). An optional --tag-offset-m shifts the tag position to the
drone body centre if the placard isn't centred.

Usage:
    scripts/seeker/autolabel_from_apriltag.py --frames FLIGHT_DIR --calib calib.json \
        --tag-size 0.10 --drone-size 0.35 --out datasets/real_flight_01
    scripts/seeker/autolabel_from_apriltag.py --self-test     # projection roundtrip, no camera

Requires cv2 + pupil-apriltags + numpy (run under .venv-seeker).
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_sim_dataset import project_to_bbox, make_label_line  # noqa: E402


def _load_calib(path):
    """Load fx,fy,cx,cy + resolution + distortion from a calib JSON — accepts the
    flight.camera / calibrate_camera.py format (fx,fy,cx,cy, dist_coeffs/dist,
    resolution) and the plain intrinsics format."""
    d = json.loads(open(path).read())
    fx, fy, cx, cy = float(d["fx"]), float(d["fy"]), float(d["cx"]), float(d["cy"])
    res = d.get("resolution", {})
    w = int(res.get("width", d.get("width", 0)))
    h = int(res.get("height", d.get("height", 0)))
    dist = d.get("dist_coeffs", d.get("dist", [0.0] * 5))
    return fx, fy, cx, cy, w, h, np.asarray(dist, dtype=float).ravel()


def autolabel_frame(gray, detector, cam_params, tag_size, drone_size,
                    tag_offset_m, w, h):
    """Detect the tag in one grayscale frame and return a YOLO box, or None if
    NO TAG WAS DECODED (the caller must NOT treat that as a negative — see run()).
    cam_params = (fx,fy,cx,cy)."""
    dets = detector.detect(gray, estimate_tag_pose=True,
                           camera_params=cam_params, tag_size=tag_size)
    if not dets:
        return None
    # Nearest tag (smallest z) if several placards are visible. pose_t is (3,1),
    # so reshape before indexing (float() on a 1-element array errors in numpy 2).
    det = min(dets, key=lambda t: float(t.pose_t.reshape(3)[2]))
    t = det.pose_t.reshape(3)
    # Placard -> drone body centre: the offset is fixed in the TAG's frame, so
    # rotate it by the tag orientation (pose_R) before adding to the translation
    # -- adding it raw in the camera frame is only right when the tag faces the
    # camera dead-on and systematically mislabels as the aspect changes (Fable
    # round-1 check, issue 3).
    off = np.asarray(tag_offset_m, dtype=float)
    if off.any():
        t = t + det.pose_R.reshape(3, 3) @ off
    x, y, z = float(t[0]), float(t[1]), float(t[2])
    rng = math.sqrt(x * x + y * y + z * z)
    fx, fy, cx, cy = cam_params
    return project_to_bbox(x, y, z, rng, fx, fy, cx, cy, drone_size, w, h)


def run(args):
    import cv2
    from pupil_apriltags import Detector
    fx, fy, cx, cy, w, h, dist = _load_calib(args.calib)
    if not (w and h):
        raise SystemExit("calib JSON needs a resolution (width/height)")
    # Undistort before detect+project (Fable round-1 check, issue 4): pupil-
    # apriltags + project_to_bbox assume an ideal pinhole, so a real lens shifts
    # boxes at the frame edges -- exactly where a crossing target lives. We
    # rectify to the SAME K (getOptimalNewCameraMatrix alpha=0 would change K).
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)
    undistort = bool(dist.any())
    map1 = map2 = None
    if undistort:
        map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, K, (w, h), cv2.CV_16SC2)
    det = Detector(families="tag36h11")
    off = tuple(float(v) for v in args.tag_offset_m.split(","))
    img_dir = os.path.join(args.out, "images")
    lbl_dir = os.path.join(args.out, "labels")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(lbl_dir, exist_ok=True)
    paths = sorted(glob.glob(os.path.join(args.frames, "*.png")) +
                   glob.glob(os.path.join(args.frames, "*.jpg")))
    if not paths:
        raise SystemExit(f"no frames in {args.frames}")
    labeled = untagged = 0
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            continue
        if undistort:
            img = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        box = autolabel_frame(gray, det, (fx, fy, cx, cy), args.tag_size,
                              args.drone_size, off, w, h)
        base = os.path.splitext(os.path.basename(p))[0]
        if box is not None:
            cv2.imwrite(os.path.join(img_dir, base + ".png"), img)
            with open(os.path.join(lbl_dir, base + ".txt"), "w") as f:
                f.write(make_label_line(box, w, h, 0) + "\n")
            labeled += 1
        elif args.negatives_from_untagged:
            # ONLY use tag-miss frames as negatives when the caller asserts the
            # footage is target-FREE (pre-launch / empty sky/ground). Otherwise
            # DROP them (below): on target-PRESENT approach footage the tag
            # decodes only to ~2-4 m while the drone is visible from ~30 m, so
            # labeling those frames 'background' would train the detector to NOT
            # see the approaching target = baking in the exact add-#18h wall this
            # pipeline exists to fix, invisibly (Fable round-1 check, issue 1).
            cv2.imwrite(os.path.join(img_dir, base + ".png"), img)
            open(os.path.join(lbl_dir, base + ".txt"), "w").close()
            labeled += 1
        else:
            untagged += 1  # DROPPED, not labeled -- never poison with a false negative
    mode = "target-FREE (tag-miss=negative)" if args.negatives_from_untagged else \
           "target-PRESENT (tag-miss DROPPED)"
    print(f"[autolabel] {labeled} labeled, {untagged} tag-miss dropped, "
          f"of {len(paths)} frames [{mode}] -> {args.out}")
    if not args.negatives_from_untagged and untagged:
        print(f"[autolabel] NOTE: {untagged} tag-miss frames DROPPED (not labeled "
              f"background) -- the tag's decode ceiling is below the drone's "
              f"visibility range, so far-band labels need a bigger placard / "
              f"tag-seeded tracking / telemetry boxes (docs/real_data_pipeline.md).")
    print(f"[autolabel] tag-label rate {100*labeled/max(1,len(paths)):.0f}% "
          f"(the tag's decode ceiling — measure the deployed markerless recall on "
          f"the SAME flights with approach_recall.py, ADR-0076 add #18h).")
    return 0


def self_test():
    """Projection roundtrip with a known tag pose (no camera/tag image needed):
    a tag/drone 4 m dead ahead, 0.35 m span, fx=fy=500 -> box centred at the
    principal point with half-width fx*0.175/4 ≈ 21.9 px."""
    fx = fy = 500.0
    cx, cy, w, h = 640.0, 360.0, 1280, 720
    box = project_to_bbox(0.0, 0.0, 4.0, 4.0, fx, fy, cx, cy, 0.35, w, h)
    ok = box is not None
    if ok:
        bcx, bcy, bw, bh = box
        exp_w = 2 * fx * 0.175 / 4.0
        ok = (abs(bcx - cx) < 1e-6 and abs(bcy - cy) < 1e-6
              and abs(bw - exp_w) < 1e-3)
        print(f"[self-test] tag 4 m ahead, 0.35 m -> box center=({bcx:.1f},{bcy:.1f}) "
              f"w={bw:.2f}px (expect {exp_w:.2f}) : {'OK' if ok else 'FAIL'}")
    # off-axis + range scaling
    box2 = project_to_bbox(1.0, 0.0, 4.0, math.hypot(1.0, 4.0), fx, fy, cx, cy, 0.35, w, h)
    ok2 = box2 is not None and box2[0] > cx  # target to the right -> box right of center
    print(f"[self-test] off-axis right -> box center u={box2[0]:.1f} > cx : {'OK' if ok2 else 'FAIL'}")
    ok = ok and ok2
    print(f"[self-test] {'PASS' if ok else 'FAIL'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", help="dir of recorded flight frames (png/jpg)")
    ap.add_argument("--calib", help="camera calib JSON (calibrate_camera.py / flight.camera)")
    ap.add_argument("--tag-size", type=float, default=0.10, help="AprilTag black-square edge (m)")
    ap.add_argument("--drone-size", type=float, default=0.35, help="target drone extent (m); 5\" ~0.35")
    ap.add_argument("--tag-offset-m", default="0,0,0",
                    help="tag->drone-centre offset x,y,z in the TAG frame (m): x right, y down, "
                         "z INTO the tag away from the camera. E.g. tag mounted 0.05 m below the "
                         "drone body centre -> '0,-0.05,0'. Rotated by pose_R before use.")
    ap.add_argument("--negatives-from-untagged", action="store_true",
                    help="treat tag-miss frames as YOLO negatives -- ONLY for TARGET-FREE "
                         "footage (empty sky/ground/pre-launch). Default DROPS tag-miss "
                         "frames so target-present approach footage can't poison the set "
                         "with false 'background' labels beyond the tag decode range.")
    ap.add_argument("--out", help="output YOLO dataset dir")
    ap.add_argument("--self-test", action="store_true", help="projection roundtrip; exit 0/1")
    args = ap.parse_args()
    if args.self_test:
        return 0 if self_test() else 1
    if not (args.frames and args.calib and args.out):
        ap.error("--frames, --calib and --out are required (or --self-test)")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
