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
the fixed configs/camera_intrinsics.json intrinsics (fx=fy=539.936, cx=640, cy=480) --
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
                              stop_event, detector_kwargs=None,
                              track=False, seed_ctx=None):
    """Same signature / threading contract as m3.detection_loop, so
    m4_intercept.py only swaps the thread `target` and nothing else.

    Publishes a fresh `Measurement` (latest-wins, atomic attribute assign) per
    new frame seq; None-fills every target field on no-detect so the alpha-beta
    LOS filter coasts. `detector_kwargs` is accepted for signature-compat with
    detection_loop but is unused (the seeker has no quad_decimate knob).

    DETECT-THEN-TRACK (ADR-0058, --track): when `track=True`, the per-frame NN
    detect() is wrapped in a DetectThenTracker (detect_track.py) -- the NN
    ACQUIRES a validated target, a cv2 CSRT tracker FOLLOWS it densely
    frame-to-frame (ignoring phantom NN boxes), and the NN periodically
    re-validates. `seed_ctx` (SeekerSeedContext) carries own-state + the
    pre-handoff cue so the seed is cue-validated (real target, not a phantom);
    post-handoff it runs camera-only (honesty). `track=False` is byte-identical
    to the plain markerless path -- the tracker is never constructed."""
    # SAME Measurement class the guidance publishes/consumes -- taken off the
    # holder by type() so we need not import m3 (which pulls gz/pupil-apriltags
    # absent from .venv-seeker). meas_holder.latest is a Measurement at start.
    Measurement = type(meas_holder.latest)
    # Option B (ADR-0033 item 2): if a fine-tuned single-class ONNX is provided via
    # MARKERLESS_NN_WEIGHTS, run it DIRECTLY on the full frame (denser than the
    # two-stage's COCO-verify). Otherwise the default off-the-shelf two-stage path.
    _nn_weights = os.environ.get("MARKERLESS_NN_WEIGHTS")
    if _nn_weights:
        from finetuned_seeker import FinetunedNNSeeker
        _conf = float(os.environ.get("MARKERLESS_NN_CONF", "0.25"))
        seeker = FinetunedNNSeeker(fx, fy, cx, cy, _nn_weights, conf_thres=_conf)
        print(f"[markerless] fine-tuned full-frame seeker: {_nn_weights}")
    else:
        seeker = TwoStageSeeker(fx, fy, cx, cy)
    # AUTO-CROP high-res mode (ADR-0074, default OFF via MARKERLESS_AUTOCROP=0):
    # wraps whichever seeker was just built. When disabled (the default),
    # AutoCropSeeker.detect() is a pure passthrough to seeker.detect() -- the
    # exact same call this loop made before the wrapper existed -- so
    # constructing it unconditionally is byte-identical to not wrapping at
    # all. When enabled, it crops a 640x640 NATIVE-res window around the last
    # detection's pixel and runs the NN on that instead of the downscaled
    # full frame (raises coverage on a small/edge-on/banking target, ADR-0076);
    # it falls back to the ordinary full-frame path itself when there is no
    # recent detection to center on (see auto_crop_seeker.py for the full
    # convention). This wrap happens BEFORE the detect-then-track wrapper
    # below, so `--track`'s own ACQUIRE/re-validate calls to seeker.detect()
    # transparently go through the crop logic too.
    from auto_crop_seeker import AutoCropSeeker
    seeker = AutoCropSeeker(seeker)
    # ADR-0058 detect-then-track wrapper (default OFF -> `frame_source` is the
    # bare seeker, byte-identical to the plain markerless path).
    tracker = None
    if track:
        from detect_track import DetectThenTracker
        tracker = DetectThenTracker(seeker, fx, fy, cx, cy, seed_ctx=seed_ctx)
        print("[markerless] DETECT-THEN-TRACK ON (ADR-0058): NN acquires a "
              f"cue-validated target, {tracker.tracker_kind} tracker follows it "
              "densely; NN re-validates every "
              f"{tracker.revalidate_every} frames.", flush=True)
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

        if tracker is not None:
            det = tracker.process(frame_bgr, t_mono)
        else:
            det = seeker.detect(frame_bgr, t_mono=t_mono)
        # SeekerDetection.as_measurement_tuple() == the exact positional args of
        # Measurement(t_mono, range_m, bearing_rad, meas_xyz, decision_margin,
        # n_detections); None-filled when nothing was confirmed (filter coasts).
        # The optional 7th arg (Measurement.source, default "") tags provenance
        # for m4's meas_source CSV column (ADR-0058 review minor): "track" = a
        # CSRT tracker box, an NN class name (e.g. "drone") = a fresh NN
        # detection, "" = no detection. Plain (non---track) markerless also
        # gains the NN tag -- additive column only, guidance never reads it.
        meas_holder.latest = Measurement(
            *det.as_measurement_tuple(),
            (det.class_name or "") if det.range_m is not None else "")

    if tracker is not None:
        print(tracker.stats_line(), flush=True)
    if getattr(seeker, "enabled", False):
        print(seeker.stats_line(), flush=True)
