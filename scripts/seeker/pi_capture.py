#!/usr/bin/env python3
"""Pi-side capture recorder for the tripod field-test — the "Pi-side frame-dump
script" flagged NOT BUILT in docs/tripod_test_protocol.md §1 and NEXT.md's
REMAINING SIM PREP (P0). It records a decision-grade capture session on the
Pi 5 + Arducam/innomaker OV9281 so the ONE tripod afternoon (docs/
tripod_test_protocol.md) produces data BOTH money gates hang on.

WHY THIS EXISTS: the tripod session scores TWO curves off ONE flight set
(protocol §0) — (a) the AprilTag decode envelope (gates the ~$740 Tier-2
interceptor order) and (b) the markerless NN approach recall (gates only the
Hailo HAT). Both are OFFLINE scoring passes (protocol §7) that read raw frames.
This script is the on-Pi capture half: full-res frames to disk + a per-frame
index so autolabel_from_apriltag.py (curve a) and the two-curve tripod scorer
(curve b, P0.3) can point straight at a session directory and consume it. It is
built and bench-tested BEFORE the field day (protocol §1) so the first time it
runs isn't during the one flight session.

The OV9281 is a MONO GLOBAL-SHUTTER sensor (docs/project_state.json hardware
node: innomaker OV9281, ~118 deg HFoV, 1280x800). Global shutter + a SHORT
exposure is the whole point of this sensor choice: the field day's primary read
is whether real motion blur at >=9 m/s eats the tag decode range / NN recall
(protocol §4.4). The spec target is <=1 ms exposure (protocol §3.3 / §4.4;
docs/project_state.json build_plan P0 camera paper-check). So this script
REQUESTS a short exposure and logs the ACTUAL applied value into meta.json —
you verify the sensor really hit <=1 ms, you don't assume it.

HONESTY BOUNDARY (CLAUDE.md): no gt_* is read anywhere here — there is no
ground truth on a Pi capturing real frames. The AprilTag decode this records is
SANCTIONED (calibration / auto-labels / the staged baseline seeker); it is a
capture-time QC record, not a guidance input. The deployed markerless seeker
never reads it.

============================================================================
ON-DISK SESSION LAYOUT  (dead simple — the tripod scorer agent reads the SAME
layout, so keep it stable):

    SESSION_DIR/
      frames/                 full-res frames, zero-padded, capture order
        000000.png
        000001.png
        ...
      index.csv               ONE row per frame (header below)
      meta.json               session-level metadata (source, exposure, res, ...)
      tags.csv                OPTIONAL — one row per frame when --decode-tags is
                              on (a frame with no tag still gets a row, n_tags=0),
                              plus one extra row per additional tag in a frame

    index.csv columns:
      frame_idx     integer, 0-based, capture order
      frame_path    path RELATIVE to SESSION_DIR (e.g. frames/000000.png)
      t_mono_s      monotonic capture time, seconds (time.monotonic; for the dir
                    replay backend this is synthetic = frame_idx / --replay-fps)
      t_wall_unix   wall-clock UNIX time, seconds (float) — the axis that lines
                    up with the target's ULog for §6.1 time-sync; blank for the
                    dir replay backend
      exposure_us   exposure ACTUALLY APPLIED, microseconds (blank if the backend
                    can't report it, e.g. dir replay)
      gain          analogue gain ACTUALLY APPLIED (blank if unavailable)

    tags.csv columns (only written when --decode-tags):
      frame_idx, frame_path, n_tags, tag_id, decision_margin, hamming,
      center_u, center_v, corners_px, range_m
      (range_m is filled only when BOTH --calib and --tag-size were given;
       corners_px is "x0:y0;x1:y1;x2:y2;x3:y3")

autolabel_from_apriltag.py consumes it directly:
    scripts/seeker/autolabel_from_apriltag.py --frames SESSION_DIR/frames \
        --calib calib.json --tag-size 0.30 --drone-size 0.35 --out DATASET
============================================================================

BACKENDS (--source):
  picamera2       real Pi + libcamera stack (OV9281). Imported LAZILY; off-Pi it
                  fails with a clear message. Requests a short exposure (<=1 ms
                  target) with AE off and logs the sensor's ACTUAL ExposureTime /
                  AnalogueGain per frame + in meta.json.
  v4l2            OpenCV VideoCapture(device) fallback (USB/UVC path, or a Pi
                  where libcamera isn't wired up). Exposure control is
                  best-effort (V4L2 units are device-specific — the requested and
                  read-back values are both logged, units flagged in meta.json).
  dir=PATH        REPLAY existing frames from PATH (png/jpg) — the dev/self-test
                  path on this WSL box (no camera). No real exposure exists, so
                  exposure/gain are blank and meta flags exposure_source=replay.

Usage:
    # real field capture on the Pi (300 frames at <=1 ms exposure, tag decode on)
    scripts/seeker/pi_capture.py --source picamera2 --out sessions/pass01 \
        --n-frames 300 --exposure-us 1000 --calib calib.json --tag-size 0.30
    # replay dev frames through the exact same pipeline (no hardware)
    scripts/seeker/pi_capture.py --source dir=/path/to/frames --out /tmp/sess
    scripts/seeker/pi_capture.py --self-test   # offline, exits 0/1, no hardware

Requires cv2 + numpy (+ pupil-apriltags for tag decode). Runs under .venv here;
on the Pi run under whatever venv carries picamera2 + pupil-apriltags.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone

# OV9281 native capture geometry (docs/project_state.json hardware node:
# innomaker OV9281, 1280x800 mono global-shutter). Overridable via --width/--height.
DEF_WIDTH, DEF_HEIGHT = 1280, 800

# The exposure SPEC TARGET, not a tuning knob: <=1 ms so real motion blur at the
# >=9 m/s engagement regime doesn't eat the tag decode range / NN recall
# (docs/tripod_test_protocol.md §3.3 / §4.4; docs/project_state.json build_plan
# P0 camera paper-check). We REQUEST this and log what the sensor actually applied.
EXPOSURE_SPEC_US = 1000
DEF_EXPOSURE_US = EXPOSURE_SPEC_US

# tag36h11 + default detector params — the SAME family/threshold convention as
# scripts/seeker/autolabel_from_apriltag.py and synth_tag_frames.py (Detector(
# families="tag36h11"), no custom thresholds). Do not diverge: the offline
# autolabel re-decodes with these defaults, so a capture-time record must match.
TAG_FAMILY = "tag36h11"

INDEX_HEADER = ["frame_idx", "frame_path", "t_mono_s", "t_wall_unix",
                "exposure_us", "gain"]
TAGS_HEADER = ["frame_idx", "frame_path", "n_tags", "tag_id", "decision_margin",
               "hamming", "center_u", "center_v", "corners_px", "range_m"]


# --------------------------------------------------------------------------
# AprilTag decode (optional, capture-time QC) — reused across every backend
# --------------------------------------------------------------------------

def _make_detector():
    try:
        from pupil_apriltags import Detector  # lazy: only when --decode-tags
    except ImportError:
        # aarch64/Pi: pupil-apriltags ships no ARM wheel; pyapriltags is the
        # API-identical drop-in but does NOT alias the module name (proven on
        # real pixels under qemu — docs/pi_emulation_check.md, 2026-07-20).
        from pyapriltags import Detector
    return Detector(families=TAG_FAMILY)


def _decode(detector, gray, cam_params, tag_size):
    """Decode tag36h11 in a grayscale frame. If cam_params (fx,fy,cx,cy) AND
    tag_size are both given, also recover pose (so range_m can be logged);
    otherwise decode-only (presence + geometry), which is all curve (a) scoring
    strictly needs from a capture-time record."""
    if cam_params is not None and tag_size:
        return detector.detect(gray, estimate_tag_pose=True,
                               camera_params=cam_params, tag_size=tag_size)
    return detector.detect(gray)


def _tag_rows(frame_idx, rel_path, dets):
    """Turn a frame's detections into tags.csv rows. A frame with NO tag still
    gets one row (n_tags=0) so a scorer reads a clean per-frame decode boolean;
    each extra tag adds a row."""
    import numpy as np
    if not dets:
        return [[frame_idx, rel_path, 0, "", "", "", "", "", "", ""]]
    rows = []
    n = len(dets)
    for det in dets:
        cu, cv = float(det.center[0]), float(det.center[1])
        corners = ";".join(f"{float(x):.2f}:{float(y):.2f}"
                           for x, y in np.asarray(det.corners))
        rng = ""
        if getattr(det, "pose_t", None) is not None:
            rng = f"{float(np.linalg.norm(det.pose_t.reshape(3))):.4f}"
        rows.append([frame_idx, rel_path, n, int(det.tag_id),
                     f"{float(det.decision_margin):.3f}", int(det.hamming),
                     f"{cu:.2f}", f"{cv:.2f}", corners, rng])
    return rows


# --------------------------------------------------------------------------
# Backends — each yields Frame(gray_img, t_mono, t_wall, exposure_us, gain)
# where exposure_us / gain are None when the backend cannot report them.
# --------------------------------------------------------------------------

class Frame:
    __slots__ = ("gray", "t_mono", "t_wall", "exposure_us", "gain")

    def __init__(self, gray, t_mono, t_wall, exposure_us, gain):
        self.gray = gray
        self.t_mono = t_mono
        self.t_wall = t_wall
        self.exposure_us = exposure_us
        self.gain = gain


def _to_gray(cv2, img):
    """OV9281 is mono; normalize any backend's frame to a single 8-bit channel so
    the whole pipeline (and the saved PNGs) is consistently grayscale."""
    if img is None:
        return None
    if img.ndim == 3 and img.shape[2] == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if img.ndim == 3 and img.shape[2] == 4:
        return cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
    return img


def iter_dir(cv2, path, n_frames, replay_fps):
    """Replay existing frames from a directory (dev/self-test). Synthetic
    monotonic timestamps (frame_idx / replay_fps) keep the self-test
    reproducible; no real exposure/gain exist so both are None."""
    paths = sorted(glob.glob(os.path.join(path, "*.png")) +
                   glob.glob(os.path.join(path, "*.jpg")) +
                   glob.glob(os.path.join(path, "*.jpeg")))
    if not paths:
        raise SystemExit(f"[pi_capture] no frames (*.png/*.jpg) in {path!r}")
    if n_frames:
        paths = paths[:n_frames]
    dt = 1.0 / float(replay_fps) if replay_fps else 1.0 / 30.0
    for i, p in enumerate(paths):
        img = cv2.imread(p)
        gray = _to_gray(cv2, img)
        if gray is None:
            print(f"[pi_capture] skip unreadable {p}", file=sys.stderr)
            continue
        yield Frame(gray, i * dt, None, None, None)


def iter_v4l2(cv2, device, w, h, exposure_us, gain, n_frames, duration):
    """OpenCV VideoCapture fallback. Exposure control is best-effort — V4L2
    exposure units are device-specific (often log2 steps, not microseconds), so
    we set what we can and log the READ-BACK value; meta.json flags the unit
    ambiguity so nobody mistakes it for a verified microsecond figure."""
    dev = int(device) if str(device).isdigit() else device
    cap = cv2.VideoCapture(dev)
    if not cap.isOpened():
        raise SystemExit(f"[pi_capture] v4l2: cannot open camera {device!r}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    # Manual exposure where the driver honors it (0.25 == manual on many UVC
    # drivers). Value units are device-specific; we read the applied value back.
    try:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure_us))
        cap.set(cv2.CAP_PROP_GAIN, float(gain))
    except Exception as e:  # pragma: no cover - driver-dependent
        print(f"[pi_capture] v4l2: exposure/gain set failed ({e}); "
              f"auto-exposure may be active", file=sys.stderr)
    applied_exp = cap.get(cv2.CAP_PROP_EXPOSURE)
    applied_gain = cap.get(cv2.CAP_PROP_GAIN)
    t0 = time.monotonic()
    i = 0
    try:
        while True:
            ok, img = cap.read()
            if not ok:
                print("[pi_capture] v4l2: frame grab failed", file=sys.stderr)
                break
            now_m, now_w = time.monotonic(), time.time()
            gray = _to_gray(cv2, img)
            yield Frame(gray, now_m, now_w,
                        applied_exp if applied_exp else None,
                        applied_gain if applied_gain else None)
            i += 1
            if n_frames and i >= n_frames:
                break
            if duration and (now_m - t0) >= duration:
                break
    finally:
        cap.release()


def iter_picamera2(cv2, w, h, exposure_us, gain, n_frames, duration):
    """Real Pi + OV9281 via libcamera/picamera2 (imported LAZILY — a clear
    failure off-Pi). AE off + a requested short exposure; the ACTUAL applied
    ExposureTime / AnalogueGain are read from each request's metadata and
    logged, so the <=1 ms spec (protocol §3.3/§4.4) is VERIFIED not assumed."""
    try:
        from picamera2 import Picamera2  # pragma: no cover - Pi hardware only
    except ImportError as e:  # pragma: no cover
        raise SystemExit(
            "[pi_capture] --source picamera2 needs the picamera2 library, which "
            "is only present on the Pi (libcamera stack). This is NOT installed "
            "on this dev box. Use --source dir=PATH for dev/self-test, or "
            "--source v4l2 for a UVC camera. Original import error: " + str(e))
    picam2 = Picamera2()  # pragma: no cover
    if exposure_us > EXPOSURE_SPEC_US:  # pragma: no cover
        print(f"[pi_capture] WARNING: requested exposure {exposure_us} us > "
              f"{EXPOSURE_SPEC_US} us spec target (protocol §3.3) — motion blur "
              f"read may be optimistic", file=sys.stderr)
    controls = {"AeEnable": False, "ExposureTime": int(exposure_us),
                "AnalogueGain": float(gain)}
    # Request a mono/greyscale main stream at native size; keep it robust to
    # driver format quirks (OV9281 may present raw mono or an ISP-upsampled
    # frame) — whatever comes back is normalized to grey by _to_gray().
    cfg = picam2.create_video_configuration(  # pragma: no cover
        main={"size": (w, h)}, controls=controls)
    picam2.configure(cfg)  # pragma: no cover
    picam2.start()  # pragma: no cover
    picam2.set_controls(controls)  # pragma: no cover
    t0 = time.monotonic()  # pragma: no cover
    i = 0  # pragma: no cover
    try:  # pragma: no cover
        while True:
            request = picam2.capture_request()
            try:
                img = request.make_array("main")
                md = request.get_metadata()
            finally:
                request.release()
            now_m, now_w = time.monotonic(), time.time()
            gray = _to_gray(cv2, img)
            exp = md.get("ExposureTime")           # microseconds (libcamera)
            g = md.get("AnalogueGain")
            yield Frame(gray, now_m, now_w, exp, g)
            i += 1
            if n_frames and i >= n_frames:
                break
            if duration and (now_m - t0) >= duration:
                break
    finally:  # pragma: no cover
        picam2.stop()


# --------------------------------------------------------------------------
# Session recorder — consumes a backend's Frame stream, writes the layout
# --------------------------------------------------------------------------

def _git_rev():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return None


def record_session(frames_iter, out_dir, source, backend_detail, w, h,
                   requested_exposure_us, requested_gain, decode_tags,
                   cam_params, tag_size, run_tag):
    """Drive a backend's Frame stream to disk in the documented layout. Returns
    the meta dict."""
    import cv2  # for imwrite; already imported by the backend, cheap re-import

    frames_dir = os.path.join(out_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)
    index_path = os.path.join(out_dir, "index.csv")
    tags_path = os.path.join(out_dir, "tags.csv") if decode_tags else None

    detector = _make_detector() if decode_tags else None
    applied_exps, applied_gains = [], []
    n = 0
    n_tag_frames = 0
    actual_w, actual_h = w, h  # overwritten with the real saved-frame shape below

    idx_f = open(index_path, "w", newline="")
    idx_w = csv.writer(idx_f)
    idx_w.writerow(INDEX_HEADER)
    tag_f = tag_w = None
    if decode_tags:
        tag_f = open(tags_path, "w", newline="")
        tag_w = csv.writer(tag_f)
        tag_w.writerow(TAGS_HEADER)

    try:
        for fr in frames_iter:
            if fr.gray is None:
                continue
            rel = os.path.join("frames", f"{n:06d}.png")
            if not cv2.imwrite(os.path.join(out_dir, rel), fr.gray):
                raise SystemExit(f"[pi_capture] failed to write {rel}")
            actual_h, actual_w = fr.gray.shape[:2]  # honest: report what was saved
            idx_w.writerow([
                n, rel,
                f"{fr.t_mono:.6f}" if fr.t_mono is not None else "",
                f"{fr.t_wall:.6f}" if fr.t_wall is not None else "",
                f"{fr.exposure_us:.1f}" if fr.exposure_us is not None else "",
                f"{fr.gain:.4f}" if fr.gain is not None else "",
            ])
            if fr.exposure_us is not None:
                applied_exps.append(float(fr.exposure_us))
            if fr.gain is not None:
                applied_gains.append(float(fr.gain))
            if decode_tags:
                dets = _decode(detector, fr.gray, cam_params, tag_size)
                if dets:
                    n_tag_frames += 1
                for row in _tag_rows(n, rel, dets):
                    tag_w.writerow(row)
            n += 1
    finally:
        idx_f.close()
        if tag_f is not None:
            tag_f.close()

    def _median(xs):
        if not xs:
            return None
        s = sorted(xs)
        m = len(s) // 2
        return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])

    applied_exp_med = _median(applied_exps)
    meta = {
        "session": run_tag,
        "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "source": source,
        "backend_detail": backend_detail,
        "resolution": {"width": int(actual_w), "height": int(actual_h)},
        "requested_resolution": {"width": int(w), "height": int(h)},
        "n_frames": n,
        "family": TAG_FAMILY,
        "tag_decode": bool(decode_tags),
        "n_tag_frames": n_tag_frames if decode_tags else None,
        "requested_exposure_us": int(requested_exposure_us),
        "applied_exposure_us": applied_exp_med,
        "exposure_source": ("sensor-metadata" if applied_exps else
                            ("replay-unknown" if source == "dir" else "unavailable")),
        "requested_gain": float(requested_gain),
        "applied_gain": _median(applied_gains),
        "exposure_spec_us": EXPOSURE_SPEC_US,
        "exposure_meets_spec": (None if applied_exp_med is None
                                else bool(applied_exp_med <= EXPOSURE_SPEC_US)),
        "calib": cam_params is not None,
        "tag_size_m": tag_size,
        "git_rev": _git_rev(),
        "frames_dir": "frames",
        "index_csv": "index.csv",
        "tags_csv": "tags.csv" if decode_tags else None,
        "note": ("gt-free real capture; AprilTag decode is capture-time QC "
                 "(sanctioned: calibration/auto-labels/baseline seeker), not a "
                 "guidance input (CLAUDE.md honesty boundary)."),
    }
    with open(os.path.join(out_dir, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    return meta


def _load_cam_params(calib_path):
    """(fx,fy,cx,cy) from a calibrate_camera.py / synth calib JSON, or None."""
    if not calib_path:
        return None
    d = json.loads(open(calib_path).read())
    return (float(d["fx"]), float(d["fy"]), float(d["cx"]), float(d["cy"]))


def capture(args):
    import cv2

    if args.source.startswith("dir="):
        path = args.source[len("dir="):]
        backend = iter_dir(cv2, path, args.n_frames, args.replay_fps)
        source, detail = "dir", {"path": path, "replay_fps": args.replay_fps}
    elif args.source == "v4l2":
        backend = iter_v4l2(cv2, args.device, args.width, args.height,
                            args.exposure_us, args.gain, args.n_frames, args.duration)
        source, detail = "v4l2", {"device": args.device,
                                  "exposure_units": "device-specific (V4L2)"}
    elif args.source == "picamera2":
        backend = iter_picamera2(cv2, args.width, args.height, args.exposure_us,
                                 args.gain, args.n_frames, args.duration)
        source, detail = "picamera2", {"sensor": "OV9281 (mono global-shutter)"}
    else:
        raise SystemExit(f"[pi_capture] unknown --source {args.source!r} "
                         f"(use picamera2 | v4l2 | dir=PATH)")

    cam_params = _load_cam_params(args.calib)
    if args.tag_size and cam_params is None and args.decode_tags:
        print("[pi_capture] NOTE: --tag-size given without --calib; decode is "
              "presence-only (no range_m in tags.csv)", file=sys.stderr)
    os.makedirs(args.out, exist_ok=True)
    meta = record_session(
        backend, args.out, source, detail, args.width, args.height,
        args.exposure_us, args.gain, args.decode_tags,
        cam_params if args.tag_size else None, args.tag_size, args.run_tag)

    print(f"[pi_capture] wrote {meta['n_frames']} frames -> {args.out}")
    print(f"[pi_capture]   index.csv + meta.json"
          + (f" + tags.csv ({meta['n_tag_frames']} frames with a tag)"
             if args.decode_tags else " (tag decode OFF)"))
    if meta["applied_exposure_us"] is not None:
        spec = "OK <=1ms" if meta["exposure_meets_spec"] else "OVER 1ms spec!"
        print(f"[pi_capture]   applied exposure ~{meta['applied_exposure_us']:.0f} us "
              f"({spec}), gain ~{meta['applied_gain']}")
    else:
        print(f"[pi_capture]   exposure: {meta['exposure_source']} "
              f"(requested {meta['requested_exposure_us']} us)")
    return 0 if meta["n_frames"] > 0 else 1


# --------------------------------------------------------------------------
# Self-test: build synthetic tag frames, run the dir backend end-to-end, and
# assert the whole session layout + CSV/meta contents. No hardware, exits 0/1.
# --------------------------------------------------------------------------

def self_test():
    import shutil
    import tempfile
    import cv2

    HERE = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, HERE)
    import synth_tag_frames as stf  # reuse the real composite+decode fixture

    tag_img = cv2.imread(stf.TAG_PNG)
    if tag_img is None:
        print(f"[self-test] FAIL: cannot read tag texture {stf.TAG_PNG}")
        return False

    import numpy as np
    rng = np.random.default_rng(0)
    work = tempfile.mkdtemp(prefix="pi_capture_selftest_")
    raw_dir = os.path.join(work, "raw")
    os.makedirs(raw_dir)
    ranges = (4.0, 5.0, 6.0, 7.0)  # all decode at a 0.5 m tag under the sim K
    n_expected = 0
    for i, r in enumerate(ranges):
        bg = stf._load_background(rng, cv2)
        result = stf.composite_tag(bg, tag_img, cv2, float(r), 0.5)
        if result is None:
            continue
        frame = result[0]
        cv2.imwrite(os.path.join(raw_dir, f"raw_{i:03d}_r{r:04.1f}.png"), frame)
        n_expected += 1

    # write a calib so the pose/range path is exercised too (synth's own K)
    calib_path = os.path.join(work, "calib.json")
    with open(calib_path, "w") as f:
        json.dump(dict(fx=stf.FX, fy=stf.FY, cx=stf.CX, cy=stf.CY,
                       resolution=dict(width=stf.W, height=stf.H),
                       dist_coeffs=[0.0] * 5), f)

    session = os.path.join(work, "session")
    ns = argparse.Namespace(
        source=f"dir={raw_dir}", out=session, n_frames=None, replay_fps=30.0,
        device="0", width=stf.W, height=stf.H, exposure_us=DEF_EXPOSURE_US,
        gain=1.0, duration=None, decode_tags=True, calib=calib_path,
        tag_size=0.5, run_tag="selftest")
    rc = capture(ns)

    ok = True

    def check(cond, msg):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[self-test] {'OK  ' if cond else 'FAIL'} {msg}")

    check(rc == 0, "capture() returned 0")

    # --- layout ---
    frames_dir = os.path.join(session, "frames")
    saved = sorted(glob.glob(os.path.join(frames_dir, "*.png")))
    check(len(saved) == n_expected,
          f"frames/ holds {len(saved)} png (expected {n_expected})")
    check(saved and os.path.basename(saved[0]) == "000000.png",
          "frames zero-padded from 000000.png")

    # --- index.csv ---
    with open(os.path.join(session, "index.csv")) as f:
        idx_rows = list(csv.DictReader(f))
    check(list(idx_rows[0].keys()) == INDEX_HEADER if idx_rows else False,
          f"index.csv header == {INDEX_HEADER}")
    check(len(idx_rows) == n_expected,
          f"index.csv has {len(idx_rows)} rows (expected {n_expected})")
    all_exist = all(os.path.exists(os.path.join(session, r["frame_path"]))
                    for r in idx_rows)
    check(all_exist, "every index frame_path points at a real file")
    tvals = [float(r["t_mono_s"]) for r in idx_rows]
    check(tvals == sorted(tvals) and len(set(tvals)) == len(tvals),
          "t_mono_s strictly increasing")
    # dir backend has no real exposure -> blank, per contract
    check(all(r["exposure_us"] == "" for r in idx_rows),
          "dir backend leaves exposure_us blank (no real exposure)")

    # --- meta.json ---
    with open(os.path.join(session, "meta.json")) as f:
        meta = json.load(f)
    required = ["source", "resolution", "n_frames", "family",
                "requested_exposure_us", "applied_exposure_us", "exposure_source",
                "exposure_spec_us", "tag_decode", "git_rev", "note"]
    check(all(k in meta for k in required),
          f"meta.json has all required keys {required}")
    check(meta["source"] == "dir" and meta["n_frames"] == n_expected,
          "meta.source == dir and n_frames matches")
    check(meta["family"] == "tag36h11", "meta.family == tag36h11")
    check(meta["requested_exposure_us"] == DEF_EXPOSURE_US
          and meta["exposure_spec_us"] == EXPOSURE_SPEC_US,
          f"meta records requested exposure {DEF_EXPOSURE_US} us + spec "
          f"{EXPOSURE_SPEC_US} us")
    check(meta["applied_exposure_us"] is None
          and meta["exposure_source"] == "replay-unknown",
          "dir backend: applied exposure null, source=replay-unknown")

    # --- tags.csv (the decode actually ran on real pixels) ---
    with open(os.path.join(session, "tags.csv")) as f:
        tag_rows = list(csv.DictReader(f))
    check(list(tag_rows[0].keys()) == TAGS_HEADER if tag_rows else False,
          f"tags.csv header == {TAGS_HEADER}")
    frame_ids = {int(r["frame_idx"]) for r in tag_rows}
    check(len(frame_ids) == n_expected,
          f"tags.csv covers every frame ({len(frame_ids)}/{n_expected})")
    decoded = [r for r in tag_rows if r["n_tags"] not in ("", "0")]
    check(len(decoded) >= 1,
          f"pupil-apriltags decoded >=1 composited tag ({len(decoded)} frames)")
    ranged = [float(r["range_m"]) for r in decoded if r["range_m"]]
    check(ranged and all(1.0 < v < 12.0 for v in ranged),
          f"decoded tag range_m populated + sane ({['%.2f' % v for v in ranged]})")

    shutil.rmtree(work, ignore_errors=True)
    print(f"[self-test] {'PASS' if ok else 'FAIL'}")
    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", help="picamera2 | v4l2 | dir=PATH")
    ap.add_argument("--out", help="session output directory")
    ap.add_argument("--n-frames", type=int, default=None,
                    help="stop after N frames (default: all, for dir; run until "
                         "--duration/Ctrl-C for live)")
    ap.add_argument("--duration", type=float, default=None,
                    help="live backends: stop after this many seconds")
    ap.add_argument("--exposure-us", type=int, default=DEF_EXPOSURE_US,
                    help=f"requested exposure microseconds (default {DEF_EXPOSURE_US} "
                         f"= the <=1 ms spec target, protocol §3.3/§4.4)")
    ap.add_argument("--gain", type=float, default=1.0, help="analogue gain")
    ap.add_argument("--width", type=int, default=DEF_WIDTH,
                    help=f"frame width (default {DEF_WIDTH}, OV9281 native)")
    ap.add_argument("--height", type=int, default=DEF_HEIGHT,
                    help=f"frame height (default {DEF_HEIGHT}, OV9281 native)")
    ap.add_argument("--device", default="0",
                    help="v4l2 device (index like 0 or /dev/videoN)")
    ap.add_argument("--replay-fps", type=float, default=30.0,
                    help="dir backend: synthetic timestamp rate (default 30)")
    ap.add_argument("--decode-tags", dest="decode_tags", action="store_true",
                    default=True, help="live tag36h11 decode -> tags.csv (default ON)")
    ap.add_argument("--no-decode-tags", dest="decode_tags", action="store_false",
                    help="skip tag decode (frames + index only)")
    ap.add_argument("--calib", default=None,
                    help="camera calib JSON (calibrate_camera.py format) — enables "
                         "tag pose -> range_m in tags.csv (needs --tag-size)")
    ap.add_argument("--tag-size", type=float, default=None,
                    help="AprilTag black-square edge (m) for pose/range; without "
                         "it decode is presence-only")
    ap.add_argument("--run-tag", default="session",
                    help="session label recorded in meta.json")
    ap.add_argument("--self-test", action="store_true",
                    help="offline end-to-end check on synthetic tag frames; exit 0/1")
    args = ap.parse_args()

    if args.self_test:
        return 0 if self_test() else 1
    if not (args.source and args.out):
        ap.error("--source and --out are required (or --self-test)")
    return capture(args)


if __name__ == "__main__":
    sys.exit(main())
