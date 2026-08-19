#!/usr/bin/env python3
"""arm_stack_check.py -- the payload that actually exercises the seeker stack.

Run BY qemu_arm_check.sh inside an aarch64 emulation target (Docker
`--platform linux/arm64`, or a chrooted aarch64 rootfs under
`qemu-aarch64-static`). Can also be run directly on the host for development
(it will simply report the host's real arch instead of aarch64 -- see
`platform.machine()` in the output -- and the wrapper script refuses to call
that "ARM validated").

WHAT THIS PROVES: the three pieces of the on-drone seeker stack actually
IMPORT and RUN on the interpreter's reported CPU architecture:
  1. onnxruntime loads `drone_finetuned_quad_v2.onnx` and completes one
     inference, through the SAME decode path the deployed detector uses
     (scripts/seeker/v3_onnx_infer.py -- byte-identical to
     finetuned_seeker.FinetunedNNSeeker, per that module's own docstring).
  2. `flight/` (the pure-Python guidance/geometry/estimator/camera core)
     imports and its math is self-consistent (a few cheap identity checks,
     not a full pytest run -- flight/tests/ already covers correctness on
     x86_64; this only asks "does it still behave the same on this CPU").
  3. `scripts/frame_source.OpenCVFrameSource` -- the REAL ingestion class
     `fly_intercept`-style code would construct on hardware -- opens and
     reads a frame via cv2.VideoCapture. We point it at a static captured
     PNG instead of a live device (see WHAT THIS DOES NOT PROVE below); this
     is deliberately the "non-Picamera path": OpenCVFrameSource is a
     generic V4L2/USB/GStreamer ingestion path (works via cv2 alone), NOT
     the Pi-specific `picamera2`/libcamera Python bindings, which need the
     real Broadcom/RP1 ISP driver stack and cannot be meaningfully exercised
     under user-mode CPU emulation anyway.

WHAT THIS DOES NOT PROVE (read before citing this as "the Pi 5 works"):
  - FPS / real-time performance. QEMU user-mode emulation interprets/JITs
    aarch64 instructions on the host CPU; it is NOT a Cortex-A76 speed
    proxy in either direction. The one onnxruntime inference here is timed
    and printed for curiosity ONLY -- do not use it to size the seeker loop
    rate. Real fps needs the physical Pi 5 (CPU path, target ~AprilTag
    30 fps per docs/hardware_order_list.md) or the Hailo vendor bench (NN
    path).
  - The real camera driver (picamera2/libcamera + the Arducam kernel
    driver). That is untestable without the physical Pi + camera and is
    explicitly out of scope here.
  - MAVSDK / PX4 offboard control on ARM (not imported by this script;
    mavsdk-python ships aarch64 wheels but the actual offboard link needs a
    flight controller to talk to, which is also out of scope for a
    pre-purchase compute check).

Exit code: 0 iff every check passes. Prints a human summary AND writes a
JSON result file (--out) for logging.
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import sys
import time
import traceback


def _repo_root() -> str:
    # scripts/deploy/arm_stack_check.py -> repo root is two levels up.
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, "..", ".."))


def check_platform_info() -> dict:
    return {
        "machine": platform.machine(),
        "system": platform.system(),
        "python_version": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "processor": platform.processor(),
    }


def check_flight_core(repo_root: str) -> dict:
    """Import flight/ (pure-Python guidance/geometry/estimator/camera) and
    run a handful of cheap, deterministic identity checks -- NOT a
    correctness re-validation (flight/tests/ already owns that on x86_64);
    this only asks whether the same math still behaves identically on the
    interpreter's actual CPU (float/libm parity)."""
    sys.path.insert(0, repo_root)
    from flight.camera import CameraModel
    from flight.geometry import euler_to_quat_body_to_ned, quat_rotate
    from flight.guidance import pronav_lateral_accel
    from flight.estimator import kalata_alpha_beta

    intr_path = os.path.join(repo_root, "configs/camera_intrinsics.json")
    cam = CameraModel.from_json(intr_path)
    bearing_rad = cam.pixel_to_bearing(700.0, 480.0)

    q = euler_to_quat_body_to_ned(0.0, 0.0, 0.0)
    identity_vec = quat_rotate(q, (1.0, 0.0, 0.0))

    accel = pronav_lateral_accel(3.0, 50.0, 0.1)
    alpha, beta = kalata_alpha_beta(1.0, 1.0, 0.05)

    problems = []
    if not (0.05 < bearing_rad < 0.20):
        problems.append(f"pixel_to_bearing out of expected range: {bearing_rad}")
    if any(abs(a - b) > 1e-9 for a, b in zip(identity_vec, (1.0, 0.0, 0.0))):
        problems.append(f"identity quat_rotate not identity: {identity_vec}")
    if abs(accel - 15.0) > 1e-9:
        problems.append(f"pronav_lateral_accel expected 15.0, got {accel}")
    if not (0.0 < alpha < 1.0 and 0.0 < beta < 1.0):
        problems.append(f"kalata_alpha_beta out of (0,1): {alpha}, {beta}")

    return {
        "ok": not problems,
        "problems": problems,
        "bearing_rad": bearing_rad,
        "identity_vec": list(identity_vec),
        "pronav_accel": accel,
        "kalata_alpha_beta": [alpha, beta],
    }


def check_onnx_inference(repo_root: str, model_path: str) -> dict:
    """Load the deployed quad-v2 ONNX weights and run ONE inference through
    the same decode path scripts/seeker/finetuned_seeker.py uses at deploy
    time (via v3_onnx_infer, which the codebase documents as byte-identical
    to it)."""
    import cv2

    seeker_dir = os.path.join(repo_root, "scripts", "seeker")
    sys.path.insert(0, seeker_dir)
    import v3_onnx_infer as vi  # noqa: E402

    if not os.path.isabs(model_path):
        model_path = os.path.join(repo_root, model_path)
    if not os.path.exists(model_path):
        return {"ok": False, "problems": [f"model not found: {model_path}"]}

    default_image = os.path.join(
        seeker_dir, "data", "flight_capture", "images",
        "cap3_000155_r026.3_pos.png",
    )

    t0 = time.time()
    sess, iname, imgsz = vi.load_session(model_path)
    t1 = time.time()

    problems = []
    return {
        "ok": True,
        "problems": problems,
        "model_path": model_path,
        "onnx_input_name": iname,
        "onnx_imgsz": imgsz,
        "load_seconds": t1 - t0,
        "_sess": sess,
        "_vi": vi,
        "_default_image": default_image,
    }


def check_frame_source_dry_run(repo_root: str, image_path: str) -> dict:
    """Dry-run the NON-Picamera ingestion path: construct the real
    scripts/frame_source.OpenCVFrameSource against a static captured PNG
    (standing in for a live device -- see module docstring) and confirm it
    yields a decoded grayscale frame with the expected shape. This is the
    generic OpenCV/V4L2 path a Pi 5 running a USB/UVC camera (or the
    Arducam through libcamera's V4L2 compat layer) would use -- NOT the
    picamera2-specific bindings, which need real hardware."""
    sys.path.insert(0, os.path.join(repo_root, "scripts"))
    import frame_source  # noqa: E402

    intr_path = os.path.join(repo_root, "configs/camera_intrinsics.json")
    if not os.path.exists(image_path):
        return {"ok": False, "problems": [f"test image not found: {image_path}"]}

    problems = []
    src = None
    try:
        src = frame_source.OpenCVFrameSource(image_path, intr_path)
        # Give the background grabber thread a moment to pull the one frame
        # a static-image "video" yields.
        gray, t = None, None
        for _ in range(50):
            gray, t = src.read()
            if gray is not None:
                break
            time.sleep(0.05)
        if gray is None:
            problems.append("OpenCVFrameSource.read() never returned a frame")
        else:
            intr = src.intrinsics
            if gray.ndim != 2:
                problems.append(f"expected grayscale (2D) frame, got shape {gray.shape}")
            if (intr.width, intr.height) != (gray.shape[1], gray.shape[0]):
                problems.append(
                    f"frame shape {gray.shape[::-1]} != calibrated intrinsics "
                    f"({intr.width}x{intr.height}) -- resolution mismatch would "
                    "silently corrupt range"
                )
        return {
            "ok": not problems,
            "problems": problems,
            "image_path": image_path,
            "frame_shape": None if gray is None else list(gray.shape),
        }
    finally:
        if src is not None:
            src.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="scripts/seeker/weights/drone_finetuned_quad_v2.onnx",
                     help="ONNX weights to load (repo-relative or absolute).")
    ap.add_argument("--image", default=None,
                     help="Test image (repo-relative or absolute). Default: a real "
                          "captured frame from scripts/seeker/data/flight_capture/.")
    ap.add_argument("--repo-root", default=None,
                     help="Override repo root (default: two dirs up from this file).")
    ap.add_argument("--out", default=None, help="Write JSON results here.")
    args = ap.parse_args()

    repo_root = args.repo_root or _repo_root()
    results = {"repo_root": repo_root}

    print("=" * 72)
    print("arm_stack_check.py -- seeker software-stack architecture check")
    print("=" * 72)

    plat = check_platform_info()
    results["platform"] = plat
    print(f"[platform] machine={plat['machine']} system={plat['system']} "
          f"python={plat['python_version']} ({plat['python_implementation']})")
    is_aarch64 = plat["machine"] in ("aarch64", "arm64")
    if not is_aarch64:
        print(f"[platform] *** NOT aarch64 (got {plat['machine']!r}) *** -- this run "
              "is a HOST SELF-TEST of the check logic, NOT an ARM validation. "
              "Run this under qemu_arm_check.sh's emulation backend to get a real answer.")

    overall_ok = True

    # 1. flight/ core.
    try:
        fc = check_flight_core(repo_root)
    except Exception as exc:
        fc = {"ok": False, "problems": [f"exception: {exc}"], "traceback": traceback.format_exc()}
    results["flight_core"] = fc
    status = "PASS" if fc.get("ok") else "FAIL"
    print(f"[1/3] flight/ core import + smoke math ... {status}")
    for p in fc.get("problems", []):
        print(f"      ! {p}")
    overall_ok &= bool(fc.get("ok"))

    # 2. onnxruntime inference.
    try:
        oi = check_onnx_inference(repo_root, args.model)
    except Exception as exc:
        oi = {"ok": False, "problems": [f"exception: {exc}"], "traceback": traceback.format_exc()}
    if oi.get("ok"):
        sess, vi, default_image = oi.pop("_sess"), oi.pop("_vi"), oi.pop("_default_image")
        image_path = args.image or default_image
        if not os.path.isabs(image_path):
            image_path = os.path.join(repo_root, image_path)
        try:
            import cv2
            img = cv2.imread(image_path)
            if img is None:
                oi["ok"] = False
                oi.setdefault("problems", []).append(f"cv2.imread failed: {image_path}")
            else:
                t0 = time.time()
                dets = vi.infer_raw(sess, oi["onnx_input_name"], oi["onnx_imgsz"], img, conf=0.25)
                t1 = time.time()
                oi["image_path"] = image_path
                oi["infer_seconds"] = t1 - t0
                oi["n_detections"] = len(dets)
                oi["providers"] = sess.get_providers()
        except Exception as exc:
            oi["ok"] = False
            oi.setdefault("problems", []).append(f"inference exception: {exc}")
            oi["traceback"] = traceback.format_exc()
    results["onnx_inference"] = oi
    status = "PASS" if oi.get("ok") else "FAIL"
    print(f"[2/3] onnxruntime load + 1 inference ({os.path.basename(args.model)}) ... {status}")
    if oi.get("ok"):
        print(f"      providers={oi.get('providers')} imgsz={oi.get('onnx_imgsz')} "
              f"n_detections={oi.get('n_detections')} "
              f"load={oi.get('load_seconds', 0):.3f}s infer={oi.get('infer_seconds', 0):.3f}s "
              "(emulated timing -- NOT an fps proxy, see module docstring)")
    for p in oi.get("problems", []):
        print(f"      ! {p}")
    overall_ok &= bool(oi.get("ok"))

    # 3. frame_source dry run (non-Picamera path).
    default_image = os.path.join(repo_root, "scripts", "seeker", "data",
                                  "flight_capture", "images", "cap3_000155_r026.3_pos.png")
    fs_image = args.image or default_image
    if not os.path.isabs(fs_image):
        fs_image = os.path.join(repo_root, fs_image)
    try:
        fs = check_frame_source_dry_run(repo_root, fs_image)
    except Exception as exc:
        fs = {"ok": False, "problems": [f"exception: {exc}"], "traceback": traceback.format_exc()}
    results["frame_source_dry_run"] = fs
    status = "PASS" if fs.get("ok") else "FAIL"
    print(f"[3/3] frame_source.OpenCVFrameSource dry-run (non-Picamera path) ... {status}")
    if fs.get("ok"):
        print(f"      frame_shape={fs.get('frame_shape')}")
    for p in fs.get("problems", []):
        print(f"      ! {p}")
    overall_ok &= bool(fs.get("ok"))

    results["overall_ok"] = bool(overall_ok)
    results["is_aarch64"] = is_aarch64
    results["validated_arm"] = bool(overall_ok and is_aarch64)

    print("=" * 72)
    if results["validated_arm"]:
        print("RESULT: PASS -- stack imports and runs one inference on REAL aarch64 "
              "(emulated). This validates the stack RUNS on ARM. It does NOT "
              "validate fps -- see module docstring.")
    elif overall_ok and not is_aarch64:
        print("RESULT: checks passed, but machine() != aarch64 -- this was a HOST "
              "SELF-TEST, not an ARM validation. Nothing is proven about the Pi 5.")
    else:
        print("RESULT: FAIL -- see problems above.")
    print("=" * 72)

    if args.out:
        out_path = args.out
        os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
        with open(out_path, "w") as fh:
            json.dump(results, fh, indent=2, default=str)
        print(f"[out] wrote {out_path}")

    return 0 if overall_ok else 1


if __name__ == "__main__":
    sys.exit(main())
