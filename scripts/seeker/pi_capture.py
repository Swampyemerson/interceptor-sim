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
                    up with the target's ULog/.BIN for §6.1 time-sync; blank for
                    the dir replay backend. scripts/seeker/range_truth_join.py
                    consumes exactly this column (+ a surveyed tripod position +
                    the target's flight log) to write the per-frame
                    `true_range_m` the tripod scorer bins on; without it a
                    session CANNOT be scored, so a field capture must use the
                    picamera2/v4l2 backend, not dir replay.
      exposure_us   exposure ACTUALLY APPLIED, microseconds (blank if the backend
                    can't report it, e.g. dir replay)
      gain          analogue gain ACTUALLY APPLIED (blank if unavailable)

    meta.json also records the MEASURED capture rate — `stream_fps` (median
    inter-frame dt), `stream_fps_p10_slow_tail` (the worst-decile rate, the
    honest input for a purchase gate on a throttling Pi) and
    `stream_fps_source`. The tripod money gate REFUSES to invent a frame rate
    (its streak burn goes as 1/fps and the ~$740 verdict flips across ~24 fps at
    the predicted decode range), so this measurement is load-bearing. A
    dir-replay session publishes NO stream_fps — its timestamps are synthetic.

    TWO RATES, NOT ONE (2026-07-25). `stream_fps` is the RECORDER LOOP rate: it
    includes a per-frame PNG write the flight loop never pays, and with tag
    decode off it pays no decode cost at all. The gate's burn model wants the
    ONBOARD DECODE cadence, so `decode_loop_fps` is recorded separately as the
    WRITE-EXCLUSIVE grab+decode rate whenever --decode-tags is on. Neither is the
    flying Pi 5 at the flight quad_decimate — that is the protocol §7.3 bench,
    passed to the scorer explicitly with --stream-fps.

    CRASH-SAFE: meta.json is written from a `finally`, so a Ctrl-C'd (or
    SIGTERM'd) pass is still scorable and self-labels `terminated_early` /
    `capture_truncated`.

    tags.csv columns (only written when --decode-tags):
      frame_idx, frame_path, n_tags, tag_id, decision_margin, hamming,
      center_u, center_v, corners_px, range_m, incidence_deg
      (range_m and incidence_deg are filled only when BOTH --calib and
       --tag-size were given -- both come from the recovered POSE;
       corners_px is "x0:y0;x1:y1;x2:y2;x3:y3". incidence_deg is the angle
       between the tag normal and the line of sight, the `theta` in
       R_decode(theta)=R_decode(0)*cos theta -- protocol §4.2b.)

    meta.json records `quad_decimate` (+ `quad_decimate_source`): the detector
    setting that PRODUCED the session. ADR-0082 makes this a decision variable
    (qd=1.0 is the planned tripod-day setting), and an unrecorded value cannot be
    recovered from the PNGs afterwards -- so it is stamped on every session.

autolabel_from_apriltag.py consumes it directly:
    scripts/seeker/autolabel_from_apriltag.py --frames SESSION_DIR/frames \
        --calib calib.json --tag-size 0.35 --drone-size 0.35 --out DATASET
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
        --n-frames 300 --exposure-us 1000 --calib calib.json --tag-size 0.35
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
import math
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

# quad_decimate — the pupil-apriltags/pyapriltags LIBRARY DEFAULT, made EXPLICIT
# here (2026-07-25). It downsamples the frame before quad detection: 2.0 halves
# the working resolution (fast, ~half the decode range), 1.0 decodes at full res
# (~1.5x range and a +-48-56 deg incidence cone instead of +-32 deg, at a real
# Pi-5 fps cost -- docs/placard_mount.md §3.3, docs/placard_sizing.md §CORRECTION).
#
# WHY IT IS A FIRST-CLASS, RECORDED PARAMETER: ADR-0082 promoted quad_decimate=1.0
# from "reclaim lever" to the PLANNED tripod-day capture setting, because at the
# adopted 0.35 m placard qd=2.0 FAILS the t_go money gate at both 20 Hz (0.442 s)
# and 14 Hz (0.294 s). Every decode site in this repo used to construct
# Detector(families=...) with no quad_decimate and no CLI flag, so all of them ran
# 2.0 silently AND meta.json did not record which value produced a session -- the
# reclaim comparison of protocol §4.2b ("re-decode the same frames at qd in
# {2.0, 1.0}") was therefore unrunnable, and after the field day it would have been
# unrecoverable, because the value that produced the frames was nowhere on record.
DEFAULT_QUAD_DECIMATE = 2.0

INDEX_HEADER = ["frame_idx", "frame_path", "t_mono_s", "t_wall_unix",
                "exposure_us", "gain"]
# incidence_deg (2026-07-25): the angle between the tag's surface NORMAL and the
# camera->tag line of sight, recovered from the SAME pose the range comes from.
# Protocol §4.2b makes incidence a scored variable, not a nuisance -- decode range
# falls as R(0)*cos(theta) (docs/placard_mount.md §3.2) and the adopted BEAM-facing
# placard makes crossing the tag's primary aspect. It costs nothing to record here
# and cannot be reconstructed later from a PNG alone.
TAGS_HEADER = ["frame_idx", "frame_path", "n_tags", "tag_id", "decision_margin",
               "hamming", "center_u", "center_v", "corners_px", "range_m",
               "incidence_deg"]


# --------------------------------------------------------------------------
# AprilTag decode (optional, capture-time QC) — reused across every backend
# --------------------------------------------------------------------------

# Detector objects are CACHED per quad_decimate and never destroyed for the
# life of the process. pupil_apriltags' C Detector.__del__ frees native state
# that SIGSEGVs when construction/destruction churns in one process (measured:
# building+dropping a Detector in a loop dies at ~cycle 3, reproduced with zero
# repo code). The field builds one Detector per capture process, so a cache is
# a no-op there; it only matters when a test (or a batch tool) makes many in one
# process. Keyed by float(quad_decimate) so the flag still selects the pyramid.
_DETECTOR_CACHE = {}


def _make_detector(quad_decimate=DEFAULT_QUAD_DECIMATE):
    key = float(quad_decimate)
    det = _DETECTOR_CACHE.get(key)
    if det is not None:
        return det
    try:
        from pupil_apriltags import Detector  # lazy: only when --decode-tags
    except ImportError:
        # aarch64/Pi: pupil-apriltags ships no ARM wheel; pyapriltags is the
        # API-identical drop-in but does NOT alias the module name (proven on
        # real pixels under qemu — docs/pi_emulation_check.md, 2026-07-20).
        from pyapriltags import Detector
    det = Detector(families=TAG_FAMILY, quad_decimate=key)
    _DETECTOR_CACHE[key] = det
    return det


def tag_incidence_deg(pose_R, pose_t):
    """Angle (deg, 0-90) between the tag's surface NORMAL and the camera->tag
    LINE OF SIGHT — the `theta` in `R_decode(theta) = R_decode(0)*cos theta`
    (docs/placard_mount.md §3.2, DERIVED; measured in sim to 32.6 deg, a
    HYPOTHESIS above 33 deg).

    pupil-apriltags returns the tag pose in the camera optical frame with the tag
    lying in its own x-y plane, so the tag's outward normal is the third column of
    `pose_R`. The LOS is `pose_t / |pose_t|`. `|cos|` is used because the recovered
    normal's SIGN flips with the classic near-frontal planar-pose ambiguity and the
    foreshortening cost is symmetric: what shortens the decode range is the
    obliquity, not which way the normal points.

    Returns None when the pose is degenerate (zero translation/rotation), never a
    fabricated 0.0 -- an unknown incidence must stay unknown (error policy §5)."""
    import numpy as np
    R = np.asarray(pose_R, dtype=float).reshape(3, 3)
    t = np.asarray(pose_t, dtype=float).reshape(3)
    n = R[:, 2]
    nt = float(np.linalg.norm(t))
    nn = float(np.linalg.norm(n))
    if not (nt > 0.0 and nn > 0.0) or not math.isfinite(nt) or not math.isfinite(nn):
        return None
    c = abs(float(np.dot(n, t)) / (nt * nn))
    return math.degrees(math.acos(min(1.0, max(0.0, c))))


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
        return [[frame_idx, rel_path, 0, "", "", "", "", "", "", "", ""]]
    rows = []
    n = len(dets)
    for det in dets:
        cu, cv = float(det.center[0]), float(det.center[1])
        corners = ";".join(f"{float(x):.2f}:{float(y):.2f}"
                           for x, y in np.asarray(det.corners))
        rng = ""
        inc = ""
        if getattr(det, "pose_t", None) is not None:
            rng = f"{float(np.linalg.norm(det.pose_t.reshape(3))):.4f}"
            if getattr(det, "pose_R", None) is not None:
                th = tag_incidence_deg(det.pose_R, det.pose_t)
                inc = "" if th is None else f"{th:.2f}"
        rows.append([frame_idx, rel_path, n, int(det.tag_id),
                     f"{float(det.decision_margin):.3f}", int(det.hamming),
                     f"{cu:.2f}", f"{cv:.2f}", corners, rng, inc])
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

def _fps_stats(t_monos):
    """(median fps, 10th-percentile-SLOW fps, median frame dt) from the capture
    timestamps, or (None, None, None) with <3 frames.

    WHY THIS IS RECORDED (2026-07-24): the tripod money gate's streak burn is
    `(E[T] / stream_fps) x V_closing`, so the capture rate moves the ~$740
    verdict directly -- at the predicted R_decode90 (7.10 m) the gate PASSES at
    30 fps and FAILS at 20 Hz. tripod_score.py therefore refuses to assume a
    rate; this is where the real number comes from. The p10 SLOW-tail rate is
    reported too because a thermally-throttled Pi's worst decile is the honest
    input for a purchase gate (constraint `pi5-emulation-gap`)."""
    if not t_monos or len(t_monos) < 3:
        return None, None, None
    ts = sorted(float(t) for t in t_monos)
    dts = [b - a for a, b in zip(ts, ts[1:]) if b > a]
    if not dts:
        return None, None, None
    dts.sort()
    n = len(dts)
    med = dts[n // 2] if n % 2 else 0.5 * (dts[n // 2 - 1] + dts[n // 2])
    slow_dt = dts[min(n - 1, int(round(0.9 * (n - 1))))]   # 90th-pct dt = p10 fps
    return (round(1.0 / med, 3) if med > 0 else None,
            round(1.0 / slow_dt, 3) if slow_dt > 0 else None,
            round(med, 6))


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
                   cam_params, tag_size, run_tag, requested_n_frames=None,
                   quad_decimate=DEFAULT_QUAD_DECIMATE):
    """Drive a backend's Frame stream to disk in the documented layout. Returns
    the meta dict.

    CRASH-SAFE FINALIZATION (2026-07-25, review 2 BLOCKER). meta.json used to be
    written only AFTER the frame loop completed, and nothing caught
    KeyboardInterrupt -- yet Ctrl-C is the DOCUMENTED way to end a live capture
    (see main()'s --n-frames help). A Ctrl-C'd pass therefore left frames/ and
    index.csv but NO meta.json, which destroys the measured stream_fps (the money
    gate's divisor), tag_size_m (every tag-recovered range scales linearly in
    it), the exposure verification and n_frames -- for a capture whose pixels are
    sitting right there. camera_session_summary then exits 2 ("not a pi_capture
    session") and tripod_score falls back to defaults. meta.json is now written
    from a `finally`, so SIGINT, SIGTERM, SystemExit and normal exit all produce
    a scorable session, self-labelled with `terminated_early`."""
    import cv2  # for imwrite; already imported by the backend, cheap re-import

    frames_dir = os.path.join(out_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)
    index_path = os.path.join(out_dir, "index.csv")
    tags_path = os.path.join(out_dir, "tags.csv") if decode_tags else None

    detector = _make_detector(quad_decimate) if decode_tags else None
    applied_exps, applied_gains = [], []
    t_monos = []   # for the MEASURED stream_fps (see _fps_stats)
    decode_loop_dts = []   # grab->decode-done, PNG write EXCLUDED (see below)
    n = 0
    n_tag_frames = 0
    actual_w, actual_h = w, h  # overwritten with the real saved-frame shape below
    terminated_early = False
    early_reason = None

    idx_f = open(index_path, "w", newline="")
    idx_w = csv.writer(idx_f)
    idx_w.writerow(INDEX_HEADER)
    tag_f = tag_w = None
    if decode_tags:
        tag_f = open(tags_path, "w", newline="")
        tag_w = csv.writer(tag_f)
        tag_w.writerow(TAGS_HEADER)

    def _median(xs):
        if not xs:
            return None
        s = sorted(xs)
        m = len(s) // 2
        return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])

    def _build_meta():
        applied_exp_med = _median(applied_exps)
        fps_med, fps_slow, dt_med = _fps_stats(t_monos)
        live = source in ("picamera2", "v4l2")
        if live and fps_med:
            fps_out, fps_src = fps_med, (
                "measured: median inter-frame dt (t_mono_s) -- RECORDER LOOP, "
                "INCLUDES the per-frame PNG write the flight loop never pays")
        elif fps_med:
            # The dir-REPLAY backend stamps SYNTHETIC timestamps (frame_idx /
            # --replay-fps), so its cadence is not a capture-rate measurement.
            # Report it, but never as `stream_fps` -- the tripod gate refuses a
            # `replay` source outright (tripod_score.resolve_stream_fps).
            fps_out, fps_src = None, ("replay-synthetic (NOT a capture-rate "
                                      f"measurement; synthetic {fps_med:.2f} fps)")
        else:
            fps_out, fps_src = None, "unavailable (too few timestamped frames)"
        # decode_loop_fps: the WRITE-EXCLUSIVE grab+decode cadence -- the quantity
        # the money gate's burn model actually needs (review 2). stream_fps is a
        # DIFFERENT experiment: it is disk-bound. Only meaningful when the decode
        # really ran in the loop.
        dec_dt_med = _median(decode_loop_dts) if decode_tags else None
        dec_fps = (round(1.0 / dec_dt_med, 3)
                   if dec_dt_med and dec_dt_med > 0 else None)
        meta = _meta_dict(fps_out, fps_src, fps_slow, dt_med, dec_fps, dec_dt_med,
                          applied_exp_med, _median(applied_gains))
        return meta

    def _meta_dict(fps_out, fps_src, fps_slow, dt_med, dec_fps, dec_dt_med,
                   applied_exp_med, applied_gain_med):
        return _make_meta_dict(
            run_tag=run_tag, source=source, backend_detail=backend_detail,
            actual_w=actual_w, actual_h=actual_h, w=w, h=h, n=n,
            decode_tags=decode_tags, n_tag_frames=n_tag_frames,
            requested_exposure_us=requested_exposure_us,
            applied_exp_med=applied_exp_med, applied_exps=applied_exps,
            requested_gain=requested_gain, applied_gain_med=applied_gain_med,
            fps_out=fps_out, fps_src=fps_src, fps_slow=fps_slow, dt_med=dt_med,
            dec_fps=dec_fps, dec_dt_med=dec_dt_med, cam_params=cam_params,
            tag_size=tag_size, requested_n_frames=requested_n_frames,
            terminated_early=terminated_early, early_reason=early_reason,
            quad_decimate=quad_decimate)

    meta = None
    try:
        for fr in frames_iter:
            if fr.gray is None:
                continue
            rel = os.path.join("frames", f"{n:06d}.png")
            t_loop0 = time.perf_counter()
            if decode_tags:
                dets = _decode(detector, fr.gray, cam_params, tag_size)
                decode_loop_dts.append(time.perf_counter() - t_loop0)
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
            if fr.t_mono is not None:
                t_monos.append(float(fr.t_mono))
            if fr.exposure_us is not None:
                applied_exps.append(float(fr.exposure_us))
            if fr.gain is not None:
                applied_gains.append(float(fr.gain))
            if decode_tags:
                if dets:
                    n_tag_frames += 1
                for row in _tag_rows(n, rel, dets):
                    tag_w.writerow(row)
            n += 1
            # Flush every 30 rows: a hard kill then costs at most one row of
            # index.csv/tags.csv instead of a whole buffer (measured 25).
            if n % 30 == 0:
                idx_f.flush()
                if tag_f is not None:
                    tag_f.flush()
    except KeyboardInterrupt:
        terminated_early, early_reason = True, "KeyboardInterrupt (Ctrl-C/SIGINT)"
        print(f"\n[pi_capture] interrupted after {n} frames -- finalizing the "
              f"session (meta.json IS written)", file=sys.stderr)
    except BaseException as e:   # SystemExit, backend death, anything
        terminated_early = True
        early_reason = f"{type(e).__name__}: {e}"
        raise
    finally:
        idx_f.close()
        if tag_f is not None:
            tag_f.close()
        # THE FIX: meta.json is written on EVERY exit path (normal, Ctrl-C,
        # SIGTERM-turned-KeyboardInterrupt, exception) so an interrupted pass is
        # still scorable. Never let a meta-write failure mask the real error.
        try:
            meta = _build_meta()
            with open(os.path.join(out_dir, "meta.json"), "w") as f:
                json.dump(meta, f, indent=2)
        except Exception as e:   # pragma: no cover - last-ditch
            print(f"[pi_capture] WARNING: could not write meta.json: {e}",
                  file=sys.stderr)
    return meta


def _make_meta_dict(*, run_tag, source, backend_detail, actual_w, actual_h, w, h,
                    n, decode_tags, n_tag_frames, requested_exposure_us,
                    applied_exp_med, applied_exps, requested_gain,
                    applied_gain_med, fps_out, fps_src, fps_slow, dt_med,
                    dec_fps, dec_dt_med, cam_params, tag_size,
                    requested_n_frames, terminated_early, early_reason,
                    quad_decimate=DEFAULT_QUAD_DECIMATE):
    meta = {
        "session": run_tag,
        "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "source": source,
        "backend_detail": backend_detail,
        "resolution": {"width": int(actual_w), "height": int(actual_h)},
        "requested_resolution": {"width": int(w), "height": int(h)},
        "n_frames": n,
        "family": TAG_FAMILY,
        # THE DETECTOR SETTING THAT PRODUCED THIS SESSION (2026-07-25, ADR-0082).
        # Stamped ALWAYS -- including when tag decode was off, because the value
        # a later OFFLINE re-decode must match/deliberately-diverge-from is a
        # property of the session, and tripod_score REFUSES to score a session
        # at a decimate other than the one on record (--allow-decimate-mismatch
        # is the explicit §4.2b reclaim-lever escape hatch).
        "quad_decimate": float(quad_decimate),
        "quad_decimate_source": (
            "--quad-decimate (explicit)"
            if float(quad_decimate) != DEFAULT_QUAD_DECIMATE else
            f"DEFAULT {DEFAULT_QUAD_DECIMATE} (the pupil-apriltags library "
            f"default, made explicit; ADR-0082 PLANS 1.0 for the tripod day)"),
        "tag_decode": bool(decode_tags),
        "n_tag_frames": n_tag_frames if decode_tags else None,
        "requested_exposure_us": int(requested_exposure_us),
        "applied_exposure_us": applied_exp_med,
        "exposure_source": ("sensor-metadata" if applied_exps else
                            ("replay-unknown" if source == "dir" else "unavailable")),
        "requested_gain": float(requested_gain),
        "applied_gain": applied_gain_med,
        # MEASURED RECORDER-LOOP rate -- the tripod money gate consumes this
        # (tripod_score.resolve_stream_fps) and REFUSES to invent one, because
        # R_streak_burn = (E[T]/stream_fps) x V_closing flips the ~$740 verdict
        # across ~24 fps at the predicted R_decode90 (tripod_score docstring).
        # HONEST SCOPE (2026-07-25): this is the DISK-BOUND recorder throughput
        # -- it pays a per-frame PNG write the flight loop never pays. It is NOT
        # the onboard decode cadence. Use decode_loop_fps below, or better, the
        # protocol §7.3 Pi 5 bench at the flying quad_decimate.
        "stream_fps": fps_out,
        "stream_fps_source": fps_src,
        "stream_fps_p10_slow_tail": fps_slow,   # conservative rate for the gate
        "frame_dt_median_s": dt_med,
        # WRITE-EXCLUSIVE grab+decode cadence: the quantity the burn model
        # actually models. Only present when the decode really ran in the loop.
        "decode_loop_fps": dec_fps,
        "decode_loop_dt_median_s": (round(dec_dt_med, 6)
                                    if dec_dt_med is not None else None),
        "decode_loop_fps_source": (
            "measured: median AprilTag decode wall-time per frame, PNG WRITE "
            "EXCLUDED (the modelled handoff cadence; still this rig's CPU, not "
            "the flying Pi 5 -- protocol §7.3)" if dec_fps else
            "not measured (tag decode was OFF in this capture)"),
        "exposure_spec_us": EXPOSURE_SPEC_US,
        "exposure_meets_spec": (None if applied_exp_med is None
                                else bool(applied_exp_med <= EXPOSURE_SPEC_US)),
        "calib": cam_params is not None,
        "tag_size_m": tag_size,
        # Request-vs-delivered for the FRAME COUNT (meta already does this for
        # resolution/exposure/gain). A pass that died 12 frames into a 300-frame
        # request must be self-labelling AFTER the field window closes.
        "requested_n_frames": requested_n_frames,
        "capture_truncated": bool(
            (requested_n_frames is not None and n < requested_n_frames)
            or terminated_early),
        "terminated_early": bool(terminated_early),
        "terminated_early_reason": early_reason,
        "git_rev": _git_rev(),
        "frames_dir": "frames",
        "index_csv": "index.csv",
        "tags_csv": "tags.csv" if decode_tags else None,
        "note": ("gt-free real capture; AprilTag decode is capture-time QC "
                 "(sanctioned: calibration/auto-labels/baseline seeker), not a "
                 "guidance input (CLAUDE.md honesty boundary)."),
    }
    return meta


def _install_sigterm_handler():
    """Turn SIGTERM into KeyboardInterrupt so `systemctl stop` / `kill` unwinds
    through record_session's finally and STILL writes meta.json, instead of hard-
    killing the process and destroying the session's metadata (review 2). SIGINT
    already raises KeyboardInterrupt. Best-effort: a non-main thread cannot
    install handlers, and that is not a reason to fail a capture."""
    import signal

    def _raise(_signum, _frame):
        raise KeyboardInterrupt("SIGTERM")

    try:
        signal.signal(signal.SIGTERM, _raise)
    except (ValueError, AttributeError, OSError):   # pragma: no cover
        pass


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
    # Only for a LIVE capture: a dir-replay desk rehearsal has no placard, so
    # warning there would be noise the operator learns to ignore.
    if args.decode_tags and not args.tag_size and source in ("picamera2", "v4l2"):
        print(f"[pi_capture] WARNING: live capture with tag decode but no "
              f"--tag-size. tags.csv will carry NO range_m and meta.json's "
              f"tag_size_m will be null, so the scorer falls back to a default "
              f"and every tag-recovered range scales by the wrong edge. The "
              f"ADOPTED placard is 0.350 m (docs/placard_mount.md §2) -- pass "
              f"--tag-size 0.35.", file=sys.stderr)
    os.makedirs(args.out, exist_ok=True)
    _install_sigterm_handler()
    qd = float(getattr(args, "quad_decimate", DEFAULT_QUAD_DECIMATE))
    if args.decode_tags and qd != DEFAULT_QUAD_DECIMATE:
        print(f"[pi_capture] quad_decimate = {qd} (NOT the library default "
              f"{DEFAULT_QUAD_DECIMATE}) -- stamped into meta.json; the scorer "
              f"must be run at the same value (ADR-0082 / protocol §4.2b)",
              file=sys.stderr)
    meta = record_session(
        backend, args.out, source, detail, args.width, args.height,
        args.exposure_us, args.gain, args.decode_tags,
        cam_params if args.tag_size else None, args.tag_size, args.run_tag,
        requested_n_frames=args.n_frames, quad_decimate=qd)

    print(f"[pi_capture] wrote {meta['n_frames']} frames -> {args.out}")
    print(f"[pi_capture]   quad_decimate={meta['quad_decimate']} "
          f"[{meta['quad_decimate_source']}]")
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
    if meta.get("decode_loop_fps"):
        print(f"[pi_capture]   rates: stream_fps={meta['stream_fps']} "
              f"(recorder loop, PNG write INCLUDED) | "
              f"decode_loop_fps={meta['decode_loop_fps']} (grab+decode, write "
              f"EXCLUDED -- the quantity the money gate models)")
    if meta["capture_truncated"]:
        # A short capture is an ERROR, not a success with fewer frames: the
        # camera wedging 12 frames into a 300-frame pass used to print "wrote 12
        # frames" and exit 0, and the field check graded --min-frames 1.
        print(f"[pi_capture] TRUNCATED: {meta['n_frames']} frames of "
              f"{meta['requested_n_frames']} requested"
              + (f" ({meta['terminated_early_reason']})"
                 if meta["terminated_early_reason"] else "")
              + f" -- backend {source} ended early", file=sys.stderr)
        return 1
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
        tag_size=0.5, run_tag="selftest", quad_decimate=DEFAULT_QUAD_DECIMATE)
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
                "exposure_spec_us", "tag_decode", "git_rev", "note",
                # 2026-07-25: tag_size_m is what every tag-recovered range scales
                # by; decode_loop_fps is the quantity the money gate models;
                # capture_truncated / terminated_early make a short or
                # interrupted pass self-labelling instead of merely incomplete.
                "tag_size_m", "decode_loop_fps", "requested_n_frames",
                "capture_truncated", "terminated_early",
                # 2026-07-25 (ADR-0082): the detector setting that produced the
                # session. Unrecorded, the §4.2b qd=1.0-vs-2.0 reclaim lever is
                # lost WITH the session -- the value cannot be read off a PNG.
                "quad_decimate", "quad_decimate_source"]
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
    # stream_fps is a MEASUREMENT the tripod money gate consumes. The dir-replay
    # backend's timestamps are synthetic, so it must NOT publish one (the gate
    # refuses a `replay` source outright -- tripod_score.resolve_stream_fps).
    check(meta["stream_fps"] is None
          and "replay" in str(meta["stream_fps_source"]).lower(),
          f"dir backend: stream_fps null, source={meta['stream_fps_source']!r} "
          f"(synthetic cadence is never a capture-rate measurement)")
    check(_fps_stats([0.0, 0.05, 0.10, 0.15, 0.20])[0] == 20.0
          and _fps_stats([0.0, 0.05])[0] is None,
          "_fps_stats: 20 Hz timestamps -> 20.0 fps; <3 frames -> None")

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
    # INCIDENCE (2026-07-25, protocol §4.2b): recovered from the same pose as the
    # range. The synth composite places the tag dead-centre and FRONTO-PARALLEL,
    # so a correct computation must come back near 0 deg -- a bug that returned
    # the off-boresight BEARING, or 90-theta, would show up here immediately.
    incs = [float(r["incidence_deg"]) for r in decoded if r["incidence_deg"]]
    check(len(incs) == len(decoded) and all(0.0 <= v < 25.0 for v in incs),
          f"decoded tag incidence_deg populated + near-frontal "
          f"({['%.1f' % v for v in incs]}) for a fronto-parallel composite")
    # The pure function, on SYNTHETIC poses (no pixels): identity pose = 0 deg;
    # a 30 deg rotation about the camera y-axis = 30 deg; a degenerate pose is
    # None, never a fabricated 0.
    import numpy as _np
    _c30, _s30 = math.cos(math.radians(30.0)), math.sin(math.radians(30.0))
    _R30 = _np.array([[_c30, 0.0, _s30], [0.0, 1.0, 0.0], [-_s30, 0.0, _c30]])
    _i0 = tag_incidence_deg(_np.eye(3), _np.array([0.0, 0.0, 5.0]))
    _i30 = tag_incidence_deg(_R30, _np.array([0.0, 0.0, 5.0]))
    _ideg = tag_incidence_deg(_np.eye(3), _np.zeros(3))
    check(abs(_i0) < 1e-9 and abs(_i30 - 30.0) < 1e-9 and _ideg is None,
          f"tag_incidence_deg on synthetic poses: identity={_i0:.3f} deg, "
          f"R_y(30)={_i30:.3f} deg, degenerate={_ideg}")

    # --- quad_decimate IS STAMPED (2026-07-25, ADR-0082) ---------------------
    check(meta["quad_decimate"] == DEFAULT_QUAD_DECIMATE
          and "DEFAULT" in meta["quad_decimate_source"],
          f"meta.json stamps quad_decimate={meta['quad_decimate']} "
          f"[{meta['quad_decimate_source']}]")
    ns_qd = argparse.Namespace(**{**vars(ns), "out": os.path.join(work, "session_qd1"),
                                  "quad_decimate": 1.0, "run_tag": "selftest-qd1"})
    rc_qd = capture(ns_qd)
    with open(os.path.join(work, "session_qd1", "meta.json")) as f:
        meta_qd = json.load(f)
    check(rc_qd == 0 and meta_qd["quad_decimate"] == 1.0
          and "explicit" in meta_qd["quad_decimate_source"],
          f"a --quad-decimate 1.0 capture records 1.0 "
          f"[{meta_qd['quad_decimate_source']}] -- the §4.2b reclaim lever is "
          f"now recoverable after the field day")
    # decode_loop_fps is the WRITE-EXCLUSIVE grab+decode cadence -- the quantity
    # the money gate models. It must exist (decode ran) and be a plausible rate.
    check(isinstance(meta["decode_loop_fps"], (int, float))
          and meta["decode_loop_fps"] > 0
          and meta["decode_loop_fps"] != meta["stream_fps"],
          f"decode_loop_fps measured separately from stream_fps "
          f"({meta['decode_loop_fps']} vs {meta['stream_fps']})")
    check(meta["capture_truncated"] is False and meta["terminated_early"] is False,
          "a complete capture is NOT flagged truncated")

    # --- CRASH-SAFE FINALIZATION (2026-07-25, review 2 BLOCKER) --------------
    # Ctrl-C is the DOCUMENTED way to end a live capture, and it used to kill the
    # session before meta.json existed -- destroying stream_fps (the money gate's
    # divisor), tag_size_m, and the exposure verification for a capture whose
    # pixels were already on disk. A KeyboardInterrupt mid-stream must still
    # produce a scorable, self-labelling session.
    def _interrupting_frames(k):
        t = 0.0
        for i in range(k):
            gray = np.zeros((16, 16), dtype=np.uint8)
            yield Frame(gray, t, 1_752_700_000.0 + t, None, None)
            t += 0.05
        raise KeyboardInterrupt("simulated Ctrl-C")

    sess_ki = os.path.join(work, "session_ctrlc")
    os.makedirs(sess_ki, exist_ok=True)
    meta_ki = record_session(_interrupting_frames(4), sess_ki, "v4l2", {}, 16, 16,
                             DEF_EXPOSURE_US, 1.0, False, None, None, "ctrlc",
                             requested_n_frames=300)
    ki_path = os.path.join(sess_ki, "meta.json")
    check(os.path.exists(ki_path),
          "Ctrl-C mid-capture STILL writes meta.json (the money gate's inputs "
          "survive an interrupted pass)")
    if os.path.exists(ki_path):
        with open(ki_path) as f:
            m2 = json.load(f)
        check(m2["n_frames"] == 4 and m2["terminated_early"] is True
              and m2["capture_truncated"] is True
              and m2["requested_n_frames"] == 300
              and "KeyboardInterrupt" in str(m2["terminated_early_reason"]),
              f"interrupted session self-labels (n={m2['n_frames']}/300, "
              f"terminated_early={m2['terminated_early']})")
        check(m2["stream_fps"] == 20.0,
              f"interrupted session still carries the MEASURED stream_fps "
              f"({m2['stream_fps']} from 0.05 s frame dt)")
        n_idx = len(open(os.path.join(sess_ki, "index.csv")).readlines()) - 1
        check(n_idx == 4, f"index.csv holds every captured frame ({n_idx}/4)")
    check(meta_ki is not None and meta_ki["terminated_early"] is True,
          "record_session RETURNS the meta dict on the interrupted path")

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
    ap.add_argument("--quad-decimate", type=float, default=DEFAULT_QUAD_DECIMATE,
                    help=f"AprilTag detector quad_decimate (default "
                         f"{DEFAULT_QUAD_DECIMATE} = the pupil-apriltags LIBRARY "
                         f"default, stated explicitly). 1.0 decodes at full "
                         f"resolution: ~1.5x range and a +-48-56 deg incidence "
                         f"cone instead of +-32 deg, at a real Pi-5 fps cost -- "
                         f"ADR-0082 PLANS 1.0 for the tripod day because 2.0 does "
                         f"not clear the t_go money gate at 20/14 Hz. The value is "
                         f"STAMPED into meta.json so the session records what it ran.")
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
