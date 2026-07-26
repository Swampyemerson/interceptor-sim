#!/usr/bin/env python3
"""Calibrate a REAL camera's pinhole intrinsics from chessboard views, and
write them in the same JSON format as camera_intrinsics.json.

WHY THIS EXISTS (the #1 sim-to-hardware gotcha): the whole perception->range
chain depends on fx/fy/cx/cy. The sim recorded fx=fy=539.9 for its 1280x960
camera; those numbers are meaningless for any real lens+sensor. AprilTag pose
(and therefore every range/bearing the guidance acts on) is only correct if
these intrinsics match the ACTUAL camera at the ACTUAL capture resolution.
Fly with the sim's intrinsics on real hardware and every range is wrong by
the ratio of the true focal length to 539.9 -- a silent, total failure of the
intercept. So: print a chessboard, run this, capture ~15-20 views, and the
resulting JSON feeds OpenCVFrameSource (see frame_source.py).

Two modes:
  --live <device>   capture from a camera interactively (SPACE grabs a view
                    when the board is found; 'c' calibrates; 'q' quits).
  --images <glob>   calibrate from a directory of already-captured stills.

Chessboard: give the INNER-corner count (e.g. a 10x7-square board has 9x6
inner corners: --cols 9 --rows 6) and the square size in metres (--square).

Output JSON (matches camera_intrinsics.json so frame_source.Intrinsics.from_json
reads it directly):
  { "resolution": {"width", "height"}, "fx","fy","cx","cy",
    "dist_coeffs": [...], "rms_reproj_error_px": ..., "n_views": ...,
    "recorded_utc": ... }
"""

import argparse
import glob
import json
import os
import sys
from datetime import datetime, timezone

import numpy as np

try:
    import cv2
except ImportError:
    print("[calib] FAILED: opencv (cv2) not installed in this environment")
    sys.exit(2)


def _object_points(cols, rows, square):
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return objp * square


def _find_corners(gray, cols, rows):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not ok:
        return None
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
    return corners


def _calibrate(objpoints, imgpoints, image_size):
    rms, K, dist, _, _ = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None
    )
    return rms, K, dist


def self_test():
    """Validate the calibration math end-to-end WITHOUT a camera or board:
    synthesize checkerboard corners from KNOWN intrinsics + Brown-Conrady
    distortion (cv2.projectPoints bakes the distortion in), re-solve with
    cv2.calibrateCamera, and assert the recovered parameters match. Exits 0/1 so
    CI can gate the tool before the hardware exists. Also confirms the result
    loads into flight.camera.CameraModel (the downstream consumer)."""
    K_true = np.array([[500.0, 0.0, 640.0],
                       [0.0, 500.0, 360.0],
                       [0.0, 0.0, 1.0]])
    dist_true = np.array([-0.30, 0.10, 0.001, -0.001, 0.0])  # k1,k2,p1,p2,k3 (barrel)
    cols, rows, sq = 9, 6, 0.025
    image_size = (1280, 720)
    objp = _object_points(cols, rows, sq)
    # Deterministic spread of board poses (tilts + shifts) so corners land across
    # the frame, where distortion is largest (no Math.random -> reproducible).
    tilts = [(-0.3, -0.2), (-0.3, 0.2), (0.0, 0.0), (0.3, -0.2), (0.3, 0.2),
             (-0.4, 0.0), (0.4, 0.0), (0.0, -0.35), (0.0, 0.35), (0.2, 0.2),
             (-0.2, -0.2), (0.15, -0.3)]
    shifts = [(-0.05, -0.03, 0.45), (0.05, -0.03, 0.5), (0.0, 0.0, 0.4),
              (-0.06, 0.02, 0.55), (0.06, 0.03, 0.5), (-0.08, 0.0, 0.6),
              (0.08, 0.0, 0.6), (0.0, -0.05, 0.45), (0.0, 0.05, 0.5),
              (0.04, 0.04, 0.48), (-0.04, -0.04, 0.52), (0.03, -0.05, 0.47)]
    objpoints, imgpoints = [], []
    for (rx, ry), (tx, ty, tz) in zip(tilts, shifts):
        rvec = np.array([rx, ry, 0.0])
        tvec = np.array([tx - (cols - 1) * sq / 2, ty - (rows - 1) * sq / 2, tz])
        img_pts, _ = cv2.projectPoints(objp, rvec, tvec, K_true, dist_true)
        objpoints.append(objp)
        imgpoints.append(img_pts.astype(np.float32))
    rms, K, dist = _calibrate(objpoints, imgpoints, image_size)
    d = dist.ravel()
    ok = True
    for name, got, want, tol in [
        ("fx", K[0, 0], 500.0, 2.0), ("fy", K[1, 1], 500.0, 2.0),
        ("cx", K[0, 2], 640.0, 2.0), ("cy", K[1, 2], 360.0, 2.0),
        ("k1", d[0], -0.30, 0.02), ("k2", d[1], 0.10, 0.05),
    ]:
        hit = abs(got - want) <= tol
        ok = ok and hit
        print(f"[self-test] {name:3} recovered {got:+.4f} (true {want:+.4f}, tol {tol})"
              f"  {'OK' if hit else 'FAIL'}")
    print(f"[self-test] rms reproj = {rms:.4f} px  ({'OK' if rms < 0.5 else 'FAIL'})")
    ok = ok and rms < 0.5
    # Downstream consumer loads it (flight.camera lives at the repo root).
    try:
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from flight.camera import CameraModel
        m = CameraModel.from_dict({
            "fx": float(K[0, 0]), "fy": float(K[1, 1]),
            "cx": float(K[0, 2]), "cy": float(K[1, 2]),
            "dist_coeffs": [float(x) for x in d[:5]],
        })
        loads = abs(m.k1 - d[0]) < 1e-9 and m.has_distortion
        print(f"[self-test] flight.camera.CameraModel loads the result: "
              f"{'OK' if loads else 'FAIL'}")
        ok = ok and loads
    except Exception as e:  # pragma: no cover
        print(f"[self-test] flight.camera load FAILED: {e}")
        ok = False
    print(f"[self-test] {'PASS' if ok else 'FAIL'}")
    return ok


def _write_json(path, K, dist, image_size, n_views, rms):
    w, h = image_size
    out = {
        # POSITIVE provenance stamp (2026-07-26): consumers on a live camera must
        # be able to tell a MEASURED checkerboard calibration from the Gazebo
        # camera_info dump (camera_intrinsics.json), which carries the same keys
        # and the same 1280x960 shape. Testing for the ABSENCE of a sim marker is
        # fragile; testing for the presence of this one is not.
        "source": "checkerboard",
        "resolution": {"width": int(w), "height": int(h)},
        "fx": float(K[0, 0]), "fy": float(K[1, 1]),
        "cx": float(K[0, 2]), "cy": float(K[1, 2]),
        "dist_coeffs": [float(x) for x in dist.flatten()],
        "rms_reproj_error_px": float(rms),
        "n_views": int(n_views),
        "recorded_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    }
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"[calib] wrote {path}")
    print(
        f"[calib] fx={out['fx']:.2f} fy={out['fy']:.2f} "
        f"cx={out['cx']:.2f} cy={out['cy']:.2f} "
        f"rms={out['rms_reproj_error_px']:.3f}px over {n_views} views"
    )
    if out["rms_reproj_error_px"] > 1.0:
        print(
            "[calib] WARNING: RMS reprojection error > 1.0 px -- calibration "
            "is marginal. Capture more views at varied angles/distances and "
            "re-run before trusting ranges."
        )


def calibrate_from_images(paths, cols, rows, square, out_path):
    objp = _object_points(cols, rows, square)
    objpoints, imgpoints = [], []
    image_size = None
    used = 0
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print(f"[calib] skip unreadable {p}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])
        corners = _find_corners(gray, cols, rows)
        if corners is None:
            print(f"[calib] no board found in {p}")
            continue
        objpoints.append(objp)
        imgpoints.append(corners)
        used += 1
        print(f"[calib] found board in {os.path.basename(p)} ({used})")
    if used < 5:
        print(f"[calib] FAILED: only {used} usable views (need >= 5, ~15+ ideal)")
        return 1
    rms, K, dist = _calibrate(objpoints, imgpoints, image_size)
    _write_json(out_path, K, dist, image_size, used, rms)
    return 0


def calibrate_live(device, cols, rows, square, out_path):
    objp = _object_points(cols, rows, square)
    objpoints, imgpoints = [], []
    cap = cv2.VideoCapture(int(device) if str(device).isdigit() else device)
    if not cap.isOpened():
        print(f"[calib] FAILED: cannot open camera {device!r}")
        return 2
    image_size = None
    print("[calib] LIVE: SPACE = grab a view when the board is highlighted, "
          "'c' = calibrate, 'q' = quit")
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])
        corners = _find_corners(gray, cols, rows)
        disp = frame.copy()
        if corners is not None:
            cv2.drawChessboardCorners(disp, (cols, rows), corners, True)
        cv2.putText(disp, f"views: {len(objpoints)}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.imshow("calibrate (SPACE grab / c calibrate / q quit)", disp)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(" ") and corners is not None:
            objpoints.append(objp)
            imgpoints.append(corners)
            print(f"[calib] grabbed view {len(objpoints)}")
        elif key == ord("c"):
            break
        elif key == ord("q"):
            cap.release()
            cv2.destroyAllWindows()
            return 1
    cap.release()
    cv2.destroyAllWindows()
    if len(objpoints) < 5:
        print(f"[calib] FAILED: only {len(objpoints)} views (need >= 5)")
        return 1
    rms, K, dist = _calibrate(objpoints, imgpoints, image_size)
    _write_json(out_path, K, dist, image_size, len(objpoints), rms)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=False)
    src.add_argument("--live", metavar="DEVICE",
                     help="camera device (index like 0, /dev/videoN, or a "
                          "GStreamer pipeline) for interactive capture")
    src.add_argument("--images", metavar="GLOB",
                     help="glob of chessboard stills to calibrate from")
    ap.add_argument("--self-test", action="store_true",
                    help="validate the calibration math on synthetic data (no "
                         "camera/board needed); exit 0/1")
    ap.add_argument("--cols", type=int, help="inner corners per row (e.g. 9 for a 10-square row)")
    ap.add_argument("--rows", type=int, help="inner corners per column")
    ap.add_argument("--square", type=float, help="chessboard square size in METRES")
    ap.add_argument("--out", default="camera_intrinsics_real.json",
                    help="output intrinsics JSON path")
    args = ap.parse_args()

    if args.self_test:
        return 0 if self_test() else 1
    if not (args.live or args.images):
        ap.error("one of --live / --images / --self-test is required")
    if args.cols is None or args.rows is None or args.square is None:
        ap.error("--cols, --rows and --square are required for a real calibration")

    if args.live:
        return calibrate_live(args.live, args.cols, args.rows, args.square, args.out)
    paths = sorted(glob.glob(args.images))
    if not paths:
        print(f"[calib] FAILED: no images matched {args.images!r}")
        return 2
    return calibrate_from_images(paths, args.cols, args.rows, args.square, args.out)


if __name__ == "__main__":
    sys.exit(main())
