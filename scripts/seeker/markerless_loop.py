"""Markerless two-stage seeker -> guidance detection loop (ADR-0033 item 2).

Drop-in replacement for `m3_static_intercept.detection_loop` when
`m4_intercept.py` is launched with `--seeker markerless`. It replaces ONLY the
detection SOURCE: each live camera frame (the SAME gz Image the AprilTag
detector consumes) is decoded to BGR and run through the two-stage markerless
seeker -- classical dark-blob PROPOSAL -> self-mask/horizon -> crop+upsample ->
NN verify (scripts/seeker/two_stage_seeker.py). Its `SeekerDetection` is
converted to the EXACT same `Measurement` the guidance / alpha-beta tracker /
pro-nav already consume: bearing primary, coarse known-size range, confidence in
the `decision_margin` slot, None-fill on no-detect. NOTHING downstream changes --
the guidance, tracker and pro-nav math are untouched; only where the Measurement
comes from changes (seeker_design_brief.md s1.2 "only the inner detect() call
changes"). Latest-wins publish; a slower/blinkier seeker just publishes fewer
Measurements and the alpha-beta LOS filter COASTS through the gaps (safe by
design, brief s1.3) -- a missing detection is coasted, never a confident-wrong
bearing (the two-stage self-mask + NN gate is what removes the standalone
classical self-lock, seeker_prototype_results.md s3.4).

HONESTY BOUNDARY (CLAUDE.md / ADR-0010): the seeker reads ONLY camera pixels +
the fixed camera_intrinsics.json intrinsics (fx=fy=539.936, cx=640, cy=480) --
never any gt_* field. The gt_* scoring/logging path in m4_intercept.py is
untouched, and the numeric no-cheat audit that applies to the AprilTag path
applies IDENTICALLY here. Because this is a NEW guidance path (brief s4 honesty
note), `--seeker markerless` RE-EARNS the numeric no-cheat audit at the live
Gazebo A/B (the NEXT gated step, docs/audit_targets.md pattern) -- offline wiring
alone does not discharge it.

DEPENDENCIES (the thing the live A/B must resolve): the two-stage seeker needs
onnxruntime + opencv (opencv-contrib for the classical detect-then-track path)
which live in `.venv-seeker`, NOT the main `.venv` that m4_intercept.py otherwise
runs under (main .venv has opencv but NO onnxruntime). A live `--seeker
markerless` run therefore needs ONE venv carrying BOTH gz-transport/mavsdk (main
.venv) AND onnxruntime/opencv (.venv-seeker): install onnxruntime (+
opencv-contrib) into the main .venv, or add gz-transport/mavsdk to .venv-seeker.
This module imports gz/mavsdk NOWHERE, so it stays importable under .venv-seeker
for the offline smoke test.
"""

from __future__ import annotations

import os
import sys
import time
from typing import Optional

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from two_stage_seeker import TwoStageSeeker  # noqa: E402

# Match m2_detect.EXPECTED_WIDTH/HEIGHT (gz_x500_mono_cam). Hardcoded rather than
# imported so this module stays importable under .venv-seeker (no gz / no m2).
EXPECTED_WIDTH = 1280
EXPECTED_HEIGHT = 960


def image_msg_to_bgr(msg) -> np.ndarray:
    """Decode a gz Image (RGB_INT8, no row padding -- the SAME assumption as
    m3_static_intercept.image_msg_to_gray) to a BGR numpy array for the seeker.

    The gz RGB_INT8 check is a LAZY import so this stays usable under
    .venv-seeker (which has no gz) for the offline smoke test; under the real
    sim venv the format guard fires exactly like image_msg_to_gray's."""
    try:
        from gz.msgs10.image_pb2 import RGB_INT8
        if msg.pixel_format_type != RGB_INT8:
            raise ValueError(
                f"unexpected pixel_format_type {msg.pixel_format_type} "
                f"(expected RGB_INT8={RGB_INT8})"
            )
    except ImportError:
        pass  # seeker venv without gz (smoke test) -> trust the RGB_INT8 layout
    rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        (msg.height, msg.width, 3))
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def markerless_detection_loop(frame_holder, meas_holder, fx, fy, cx, cy,
                              stop_event, detector_kwargs=None):
    """Same signature / threading contract as m3.detection_loop, so
    m4_intercept.py only swaps the thread `target` and nothing else.

    Publishes a fresh `Measurement` (latest-wins, atomic attribute assign) per
    new frame seq; None-fills every target field on no-detect so the alpha-beta
    LOS filter coasts. `detector_kwargs` is accepted for signature-compat with
    detection_loop but is unused (the seeker has no quad_decimate knob)."""
    # SAME Measurement class the guidance publishes/consumes -- taken off the
    # holder by type() so we need not import m3 (which pulls gz/pupil-apriltags
    # absent from .venv-seeker). meas_holder.latest is a Measurement at start.
    Measurement = type(meas_holder.latest)
    seeker = TwoStageSeeker(fx, fy, cx, cy)
    last_seq = -1
    warned_resolution = False
    while not stop_event.is_set():
        seq, msg = frame_holder.latest
        if msg is None or seq == last_seq:
            time.sleep(0.005)
            continue
        last_seq = seq
        t_mono = time.monotonic()

        if msg.width != EXPECTED_WIDTH or msg.height != EXPECTED_HEIGHT:
            if not warned_resolution:
                print(
                    f"[markerless] WARNING: frame resolution "
                    f"{msg.width}x{msg.height} != expected "
                    f"{EXPECTED_WIDTH}x{EXPECTED_HEIGHT}; treating as no detection"
                )
                warned_resolution = True
            meas_holder.latest = Measurement(t_mono, None, None, None, None, 0)
            continue

        try:
            frame_bgr = image_msg_to_bgr(msg)
        except ValueError as exc:
            print(f"[markerless] WARNING: frame decode failed: {exc}")
            meas_holder.latest = Measurement(t_mono, None, None, None, None, 0)
            continue

        det = seeker.detect(frame_bgr, t_mono=t_mono)
        # SeekerDetection.as_measurement_tuple() == the exact positional args of
        # Measurement(t_mono, range_m, bearing_rad, meas_xyz, decision_margin,
        # n_detections); None-filled when nothing was confirmed (filter coasts).
        meas_holder.latest = Measurement(*det.as_measurement_tuple())
