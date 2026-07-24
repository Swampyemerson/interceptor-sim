#!/usr/bin/env python3
"""flight/deploy/seeker_loop.py -- the REAL-HARDWARE camera-only pro-nav terminal.

This is the code the physical interceptor's Pi 5 runs. It mirrors the Gazebo
sim's terminal seeker (scripts/m4_intercept.py ENGAGE phase) but on REAL I/O, and
it drives the SAME portable `flight/` core the sim uses -- so a bug in the frame
math, the LOS derotation, the alpha-beta tracker, or the pro-nav law surfaces
HERE on the desk, before it ever costs a real flight.

PIPELINE (each stage is a thin wrapper over an already-tested piece):

  FRAME SOURCE                Picamera2 (real cam) | video file | image dir | stub
    -> ONNX NN DETECT         scripts/seeker/finetuned_seeker.FinetunedNNSeeker
                              DEFAULT WEIGHTS = the REAL-DATA n-mono model
                              (scripts/seeker/weights/nn_tier/n-mono.onnx, YOLO11n
                              COCO-init, grayscale-native). The sim-trained
                              drone_finetuned_quad_v2 is BLIND on real imagery
                              (AP50 0.0003 / recall 1.1% / false-fire 88.5% on a
                              source-disjoint held-out set, n=4175) and must never
                              be the default on hardware -- it stays selectable via
                              --weights as the historical bar. Modality is resolved
                              per-model (--gray-input auto).
    -> BEARING + RANGE        box centre + intrinsics via flight.camera.CameraModel
                              (UNDISTORTS -- honesty-critical for the real wide lens);
                              range = fx*span/box_width_px (span from calib sidecar)
    -> LOS AZIMUTH            flight.geometry.derotate_bearing_lambda
                              (full-attitude LOS, +mount up-tilt)
    -> LEVER-ARM             flight.geometry.camera_to_cg_los (camera->CG re-anchor)
    -> ESTIMATOR             flight.estimator.AlphaBetaFilter (lambda + range channels)
    -> PRO-NAV (N=5)         flight.guidance.closing_speed + pronav_lateral_accel
    -> SETPOINT              NED velocity + absolute yaw -> MAVSDK offboard
                              (--dry-run PRINTS the setpoint instead of sending)

HONESTY / NO-CHEAT AUDIT (CLAUDE.md #5, ADR-0010):
  * The ONLY inputs are (a) camera pixels and (b) the flight controller's OWN-state
    EKF -- the body->NED attitude quaternion, yaw, and altitude that MAVSDK reports
    over MAVLink. That is the same own-state basis the sim's guidance is allowed to
    read. There is NO gt_* anywhere in this file (there is no ground truth on real
    hardware to read); grep confirms it. The `flight/` core it calls is itself
    gt-free and test-pinned. The ONNX detector's weights were fine-tuned OFFLINE on
    gt-projected labels (an offline human-labeler analogue) -- the LIVE detector at
    inference is gt-free.
  * On the DESK (no flight controller), own-state falls back to a LEVEL hover
    (identity attitude, yaw 0, a fixed altitude). That is a benign stand-in, not
    ground truth: it says "assume the camera is level," which is exactly what a
    bench test with the camera on a table is.

DEPS: numpy + opencv + onnxruntime (all in .venv-seeker). Picamera2 and mavsdk are
GUARDED imports -- absent on this x86 desk, so the desk/dry-run path runs without
them. Run with:  .venv-seeker/bin/python -m flight.deploy.seeker_loop ...
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

# The portable, honesty-audited core (pure Python + math; no gz/gt/mavsdk).
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
# The ONNX seeker lives under scripts/seeker and imports its siblings flat.
_SEEKER_DIR = os.path.join(_REPO_ROOT, "scripts", "seeker")
if _SEEKER_DIR not in sys.path:
    sys.path.insert(0, _SEEKER_DIR)

from flight.camera import CameraModel
from flight.estimator import AlphaBetaFilter
from flight.geometry import camera_to_cg_los, derotate_bearing_lambda
from flight.guidance import closing_speed, pronav_lateral_accel

# ------------------------------------------------------------------ configuration


@dataclass
class GuidanceConfig:
    """Every guidance/geometry constant the terminal needs. Defaults mirror the
    sim's FPV profile (scripts/m4_intercept.py FPV{}, ADR-0010/0011) so the desk
    loop behaves like the validated fast-interceptor regime."""

    # Pro-nav + alpha-beta (flight.guidance / flight.estimator).
    n_pronav: float = 5.0          # navigation gain (lab trade study winner, ADR-0011)
    alpha: float = 0.5             # alpha-beta position gain (both channels)
    beta_lambda: float = 0.30      # alpha-beta rate gain, LOS azimuth channel
    beta_range: float = 0.45       # alpha-beta rate gain, range channel (FPV)
    lambda_rate_cap: Optional[float] = math.radians(60.0)  # clamp |lambda_dot|

    # Closing-speed law (compute_v_close, two-speed).
    vc_floor: float = 4.5          # range-rate floor feeding a_cmd = N*Vc*lambda_dot
    v_close_runin: float = 9.0     # closing speed while far
    v_close_terminal: float = 5.5  # throttled closing speed in the terminal band
    throttle_range_m: float = 5.0  # r_hat at/below which closing throttles down

    # Velocity clamps.
    v_perp_max: float = 8.0        # lateral (pro-nav) velocity clamp
    v_total_max: float = 13.0      # horizontal command safety clamp
    v_vert_max: float = 2.0        # vertical command clamp
    kp_alt: float = 1.0            # altitude-hold P gain
    alt_ref_m: float = 5.0         # desk/hold altitude reference

    # Terminal-coast: as R->0 lambda_dot is singular -> freeze the velocity vector.
    terminal_freeze_range_m: float = 3.5

    # Target known size (fed to range = fx*span/box_width). Overridden by the
    # detector's own calib sidecar if present; this is the fallback.
    target_span_m: float = 1.0

    # Camera mount (docs/hardware_order_list.md): forward/prop-clearance offset and
    # a fixed up-tilt. All in metres / radians in the vehicle BODY (FRD) frame.
    mount_fwd_m: float = 0.10      # camera ahead of the CG (prop clearance)
    mount_left_m: float = 0.0
    mount_up_m: float = 0.05       # camera above the CG
    mount_up_rad: float = 0.0      # fixed up-tilt of the boresight (#40)

    @property
    def cam_offset_body(self) -> Tuple[float, float, float]:
        return (self.mount_fwd_m, self.mount_left_m, self.mount_up_m)


def compute_v_close(r_hat: Optional[float], cfg: GuidanceConfig) -> float:
    """Two-speed closing law (mirror of m4_intercept.compute_v_close): full run-in
    speed while far, linearly throttled to the terminal speed inside throttle_range.
    r_hat None (range filter cold) -> run-in speed (get moving)."""
    hi = cfg.throttle_range_m
    lo = cfg.terminal_freeze_range_m
    if r_hat is None or r_hat >= hi:
        return cfg.v_close_runin
    if r_hat <= lo:
        return cfg.v_close_terminal
    frac = (r_hat - lo) / (hi - lo)  # 1 at hi -> 0 at lo, EXACTLY matching m4_intercept.compute_v_close
    return cfg.v_close_terminal + frac * (cfg.v_close_runin - cfg.v_close_terminal)


# ------------------------------------------------------------------ data records


@dataclass
class OwnState:
    """The vehicle's OWN-state EKF outputs (from MAVSDK on hardware; a level
    stand-in on the desk). NO ground truth."""

    quat: Optional[Tuple[float, float, float, float]] = None  # (w,x,y,z) body->NED
    psi_rad: Optional[float] = None                           # yaw
    alt_m: Optional[float] = None                             # altitude AMSL/rel

    @staticmethod
    def level(alt_m: float) -> "OwnState":
        """Desk stand-in: camera level, yaw 0. (identity quat = no rotation)."""
        return OwnState(quat=(1.0, 0.0, 0.0, 0.0), psi_rad=0.0, alt_m=alt_m)


@dataclass
class Setpoint:
    """A PX4 OFFBOARD velocity+yaw command (NED). Straight into MAVSDK's
    VelocityNedYaw; also what --dry-run prints."""

    v_north: float
    v_east: float
    v_down: float
    yaw_deg: float

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return (self.v_north, self.v_east, self.v_down, self.yaw_deg)


@dataclass
class StepTelemetry:
    """Per-tick record (for the CSV log / --dry-run printout / tests)."""

    t: float
    detected: bool
    bearing_deg: Optional[float] = None
    range_meas_m: Optional[float] = None
    lambda_deg: Optional[float] = None
    lambda_dot_deg_s: Optional[float] = None
    r_hat_m: Optional[float] = None
    rdot_hat_m_s: Optional[float] = None
    vc: Optional[float] = None
    a_cmd: Optional[float] = None
    v_perp: Optional[float] = None
    v_close: Optional[float] = None
    terminal_coast: bool = False
    setpoint: Optional[Setpoint] = None


# ------------------------------------------------------------------ measurement


def measurement_from_box(box_xywh, cfg: GuidanceConfig, cam: CameraModel,
                         span_m: float):
    """Turn a detector box (x0, y0, w, h in FULL-FRAME pixels) into the camera
    measurement the LOS math consumes:
      * bearing_h  -- horizontal bearing, UNDISTORTED via CameraModel (honesty-
                      critical for the real wide M12 lens; a no-op for the sim
                      pinhole, so byte-identical there).
      * range_m    -- known-size range = fx*span/box_width_px.
      * meas_xyz   -- target vector in the OPTICAL frame (x right, y down,
                      z forward), = range * the UNDISTORTED unit ray, so it
                      carries the elevation term derotate_bearing_lambda wants.
    """
    x0, y0, bw, bh = box_xywh
    u = x0 + bw / 2.0
    v = y0 + bh / 2.0
    bearing_h = cam.pixel_to_bearing(u, v)          # undistorted horizontal bearing
    ray = cam.pixel_to_ray(u, v)                    # undistorted UNIT optical ray
    range_m = cam.fx * span_m / max(bw, 1.0)
    meas_xyz = np.array([ray[0], ray[1], ray[2]]) * range_m
    return bearing_h, range_m, meas_xyz


# ------------------------------------------------------------------ guidance core


class SeekerGuidance:
    """The camera-only pro-nav terminal, as a pure step function so it is
    MAVSDK-free and unit-testable. `step(det_box, own, t)` folds one frame's
    detection (or a miss) into the alpha-beta trackers and returns the NED
    velocity+yaw setpoint (or None before first lock). This is the exact ENGAGE-
    phase math from scripts/m4_intercept.py, calling the SAME flight/ core."""

    def __init__(self, cfg: GuidanceConfig, cam: CameraModel, span_m: float):
        self.cfg = cfg
        self.cam = cam
        self.span_m = span_m
        self.lam = AlphaBetaFilter(cfg.alpha, cfg.beta_lambda, angular=True,
                                   rate_cap=cfg.lambda_rate_cap, label="lambda")
        self.rng = AlphaBetaFilter(cfg.alpha, cfg.beta_range, angular=False,
                                   label="range")
        self.v_perp = 0.0
        self._last_t: Optional[float] = None
        self._frozen_vworld: Optional[Tuple[float, float]] = None
        self._last_bearing_rad: float = 0.0

    def step(self, det_box, own: OwnState, t: float) -> Tuple[Optional[Setpoint], StepTelemetry]:
        cfg = self.cfg
        dt = 0.05 if self._last_t is None else max(1e-3, t - self._last_t)
        self._last_t = t

        # PREDICT every tick (coast the estimate forward on the current rate).
        self.lam.predict(dt)
        self.rng.predict(dt)

        tel = StepTelemetry(t=t, detected=det_box is not None)

        # CORRECT on a fresh detection.
        bearing_h = None
        if det_box is not None:
            bearing_h, range_meas, meas_xyz = measurement_from_box(
                det_box, cfg, self.cam, self.span_m)
            self._last_bearing_rad = bearing_h
            tel.bearing_deg = math.degrees(bearing_h)
            tel.range_meas_m = range_meas
            # Camera-frame LOS azimuth -> inertial, derotating the FULL own
            # attitude (+ fixed mount up-tilt). Own EKF only, no gt_*.
            lambda_meas = derotate_bearing_lambda(
                meas_xyz, bearing_h, own.quat, own.psi_rad or 0.0,
                mount_up_rad=cfg.mount_up_rad)
            # Re-anchor camera range/LOS from the CAMERA to the CG (lever arm).
            range_cg, lambda_cg = camera_to_cg_los(
                range_meas, lambda_meas, own.quat, cfg.cam_offset_body)
            self.lam.correct(lambda_cg, t)
            self.rng.correct(range_cg, t)

        lambda_hat = self.lam.x_hat
        lambda_dot = self.lam.xdot_hat if self.lam.initialized else None
        r_hat = self.rng.x_hat
        rdot_hat = self.rng.xdot_hat if self.rng.initialized else None
        tel.lambda_deg = None if lambda_hat is None else math.degrees(lambda_hat)
        tel.lambda_dot_deg_s = None if lambda_dot is None else math.degrees(lambda_dot)
        tel.r_hat_m = r_hat
        tel.rdot_hat_m_s = rdot_hat

        if lambda_hat is None:
            # No lock yet -- nothing to steer on.
            return None, tel

        # PRO-NAV: a_cmd = N * Vc * lambda_dot  (flight.guidance).
        vc = closing_speed(rdot_hat, cfg.vc_floor)
        a_cmd = pronav_lateral_accel(cfg.n_pronav, vc, lambda_dot)
        tel.vc = vc
        tel.a_cmd = a_cmd

        v_close = compute_v_close(r_hat, cfg)
        tel.v_close = v_close

        # TERMINAL COAST: inside the freeze range lambda_dot is singular -> lock
        # the collision-course velocity vector and fly it straight through CPA.
        in_coast = (self._frozen_vworld is not None
                    or (r_hat is not None and r_hat < cfg.terminal_freeze_range_m))
        if not in_coast:
            self.v_perp = _clamp(self.v_perp + a_cmd * dt,
                                 -cfg.v_perp_max, cfg.v_perp_max)
            u = (math.cos(lambda_hat), math.sin(lambda_hat))       # LOS dir (N,E)
            p = (-math.sin(lambda_hat), math.cos(lambda_hat))      # perpendicular
            vh_n = v_close * u[0] + self.v_perp * p[0]
            vh_e = v_close * u[1] + self.v_perp * p[1]
            norm = math.hypot(vh_n, vh_e)
            if norm > cfg.v_total_max:
                s = cfg.v_total_max / norm
                vh_n *= s
                vh_e *= s
        else:
            if self._frozen_vworld is None:
                u = (math.cos(lambda_hat), math.sin(lambda_hat))
                p = (-math.sin(lambda_hat), math.cos(lambda_hat))
                self._frozen_vworld = (v_close * u[0] + self.v_perp * p[0],
                                       v_close * u[1] + self.v_perp * p[1])
            vh_n, vh_e = self._frozen_vworld
        tel.v_perp = self.v_perp
        tel.terminal_coast = in_coast

        # Altitude hold (own-state alt only; None on the desk -> level).
        if own.alt_m is not None:
            v_down = _clamp(cfg.kp_alt * (own.alt_m - cfg.alt_ref_m),
                            -cfg.v_vert_max, cfg.v_vert_max)
        else:
            v_down = 0.0

        # Absolute yaw at the target's azimuth: psi + bearing (nose on the LOS).
        psi_deg = 0.0 if own.psi_rad is None else math.degrees(own.psi_rad)
        yaw_deg = psi_deg + math.degrees(self._last_bearing_rad)

        sp = Setpoint(vh_n, vh_e, v_down, yaw_deg)
        tel.setpoint = sp
        return sp, tel


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ------------------------------------------------------------------ frame sources


class ImageDirSource:
    """Iterate PNG/JPG frames from a directory in sorted order -- the DESK test
    mode over existing captured frames (scripts/seeker/data/*/images)."""

    def __init__(self, path: str, fps: float = 20.0, loop: bool = False):
        pats = ("*.png", "*.jpg", "*.jpeg")
        files: List[str] = []
        if os.path.isdir(path):
            # Common layouts: <dir>/*.png or <dir>/images/*.png
            for base in (path, os.path.join(path, "images")):
                for p in pats:
                    files += glob.glob(os.path.join(base, p))
        else:
            files = [path]
        self.files = sorted(set(files))
        if not self.files:
            raise FileNotFoundError(f"no images under {path}")
        self.dt = 1.0 / fps
        self.loop = loop

    def frames(self):
        import cv2
        i = 0
        while True:
            f = self.files[i]
            img = cv2.imread(f)
            if img is not None:
                yield img, os.path.basename(f)
            i += 1
            if i >= len(self.files):
                if not self.loop:
                    return
                i = 0


class VideoSource:
    """Iterate frames from a video file (cv2.VideoCapture) -- desk test over a
    recorded flight video."""

    def __init__(self, path: str):
        self.path = path

    def frames(self):
        import cv2
        cap = cv2.VideoCapture(self.path)
        if not cap.isOpened():
            raise FileNotFoundError(f"cannot open video {self.path}")
        n = 0
        try:
            while True:
                ok, img = cap.read()
                if not ok:
                    return
                n += 1
                yield img, f"frame{n:06d}"
        finally:
            cap.release()


class PicameraSource:
    """Live frames from the real Pi 5 camera (Picamera2). GUARDED import -- only
    constructed on the vehicle; absent on this x86 desk."""

    def __init__(self, size=(1280, 960)):
        try:
            from picamera2 import Picamera2  # type: ignore
        except ImportError as exc:  # pragma: no cover -- hardware only
            raise RuntimeError(
                "Picamera2 not available -- run on the Pi, or use "
                "--source <image-dir|video> on the desk.") from exc
        self.cam = Picamera2()
        config = self.cam.create_video_configuration(
            main={"size": size, "format": "RGB888"})
        self.cam.configure(config)
        self.cam.start()

    def frames(self):  # pragma: no cover -- hardware only
        import cv2
        while True:
            rgb = self.cam.capture_array()
            yield cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), "picam"


class StubSeeker:
    """A synthetic 'detector' for --self-test: returns a box drifting right each
    call (non-zero LOS rate -> exercises pro-nav lambda_dot) whose implied range
    STARTS beyond the terminal-freeze band and SHRINKS (so the non-coast pro-nav
    v_perp integration actually reaches the setpoint before terminal coast
    latches). NO camera, NO weights -- exercises the flight/ core deterministically.
    The returned box width is derived from the target range so range = fx*span/bw
    reproduces `range_m` self-consistently."""

    def __init__(self, u0=660.0, v0=480.0, drift_px=8.0, r0=8.0, r_shrink=0.5,
                 fx=539.936, span=1.0):
        self.u0, self.v0, self.drift = u0, v0, drift_px
        self.r0, self.r_shrink, self.fx, self.span = r0, r_shrink, fx, span
        self.n = 0

    def detect(self, frame_bgr, t_mono=None):
        from types import SimpleNamespace
        u = self.u0 + self.drift * self.n
        rng = max(1.0, self.r0 - self.r_shrink * self.n)
        bw = self.fx * self.span / rng      # box width consistent with the range
        self.n += 1
        return SimpleNamespace(
            range_m=rng, box_xywh=(u - bw / 2.0, self.v0 - bw / 2.0, bw, bw))


class SmokeSeeker:
    """Synthetic detector for the --sitl-smoke MAVSDK check: a target box that
    oscillates gently about frame centre at a CONSTANT range, so the deploy loop
    stays in the GUIDED pro-nav regime for the whole run (bounded bearing + a
    realistic, bounded LOS rate) and continuously exercises the
    guidance -> setpoint -> MAVSDK path -- with NO camera / weights / onnxruntime.

    Distinct from StubSeeker on purpose: StubSeeker SHRINKS range into terminal
    coast (freezing the velocity vector), which is right for the offline self-test
    but would make the live setpoint stream a single frozen vector plus a drifting
    yaw. Holding a constant range > terminal_freeze keeps every tick a live
    pro-nav command, which is what we want to push through the MAVLink plumbing.
    Range is keyed to t_mono, so the box motion is deterministic and frame-content
    independent (the paired SyntheticSource yields blank frames)."""

    def __init__(self, fx, span, cx=640.0, cy=480.0, range_m=8.0,
                 amp_px=60.0, period_s=6.0):
        self.fx, self.span = fx, span
        self.cx, self.cy = cx, cy
        self.range_m = range_m
        self.amp = amp_px
        self.w = 2.0 * math.pi / period_s

    def detect(self, frame_bgr, t_mono=0.0):
        from types import SimpleNamespace
        t = 0.0 if t_mono is None else t_mono
        u = self.cx + self.amp * math.sin(self.w * t)
        bw = self.fx * self.span / self.range_m   # box width consistent with range
        return SimpleNamespace(
            range_m=self.range_m,
            box_xywh=(u - bw / 2.0, self.cy - bw / 2.0, bw, bw))


class SyntheticSource:
    """A bounded stream of blank frames for the --sitl-smoke MAVSDK check. The
    smoke validates the MAVLink plumbing, NOT perception, so the frame content is
    irrelevant (the paired SmokeSeeker fabricates the box). Yielding a FINITE
    number of frames is also what bounds the offboard setpoint stream, so the
    check terminates and lands cleanly on its own."""

    def __init__(self, n_frames: int, size=(960, 1280)):
        self.n_frames = int(n_frames)
        self.size = size

    def frames(self):
        blank = np.zeros((self.size[0], self.size[1], 3), dtype=np.uint8)
        for i in range(self.n_frames):
            yield blank, f"synth{i:05d}"


# ------------------------------------------------------------------ config load


def load_camera(intrinsics_path: str) -> CameraModel:
    """Build the CameraModel from a calibrate_camera.py / camera_intrinsics.json
    file (fx,fy,cx,cy + optional dist_coeffs). Zero distortion (the sim case) is
    a pinhole special case, so the same code serves sim and hardware."""
    return CameraModel.from_json(intrinsics_path)


def resolve_span(weights: str, cfg: GuidanceConfig) -> Tuple[float, str]:
    """Effective known-size span, matching FinetunedNNSeeker: MARKERLESS_SPAN_M
    env, then a <weights>.calib.json sidecar, then the config default."""
    env = os.environ.get("MARKERLESS_SPAN_M")
    if env:
        try:
            return float(env), f"env {env}"
        except ValueError:
            pass
    sidecar = str(weights) + ".calib.json"
    if os.path.exists(sidecar):
        try:
            with open(sidecar) as fh:
                return float(json.load(fh)["span_m_eff"]), \
                    f"calib {os.path.basename(sidecar)}"
        except Exception:
            pass
    return cfg.target_span_m, "config default"


# ------------------------------------------------------------------ desk driver


def run_over_source(source, detector, guidance: SeekerGuidance,
                    dry_run: bool = True, max_frames: Optional[int] = None,
                    fps: float = 20.0, verbose: bool = True) -> List[StepTelemetry]:
    """Drive the terminal over a frame source with a LEVEL own-state (desk). On
    real hardware the MAVSDK driver replaces this and supplies live own-state +
    sends the setpoints; here we PRINT them (dry-run). Returns the telemetry log."""
    log: List[StepTelemetry] = []
    dt = 1.0 / fps
    t = 0.0
    n_det = 0
    for i, (frame, name) in enumerate(source.frames()):
        if max_frames is not None and i >= max_frames:
            break
        det = detector.detect(frame, t)
        box = getattr(det, "box_xywh", None)
        # A detection with no range/box is a miss (finetuned_seeker returns a
        # SeekerDetection with box_xywh=None on no-detect).
        if getattr(det, "range_m", None) is None:
            box = None
        own = OwnState.level(guidance.cfg.alt_ref_m)
        sp, tel = guidance.step(box, own, t)
        log.append(tel)
        if tel.detected:
            n_det += 1
        if verbose and (sp is not None) and (i % 5 == 0 or box is not None):
            if sp is not None:
                print(f"[{name}] t={t:5.2f} det={tel.detected} "
                      f"bear={_f(tel.bearing_deg)}deg r_meas={_f(tel.range_meas_m)}m "
                      f"lam={_f(tel.lambda_deg)}deg lamdot={_f(tel.lambda_dot_deg_s)}deg/s "
                      f"r_hat={_f(tel.r_hat_m)}m -> "
                      f"SETPOINT vN={sp.v_north:+.2f} vE={sp.v_east:+.2f} "
                      f"vD={sp.v_down:+.2f} yaw={sp.yaw_deg:+.1f}"
                      + ("  [COAST]" if tel.terminal_coast else ""))
        t += dt
    if verbose:
        n = len(log)
        n_sp = sum(1 for r in log if r.setpoint is not None)
        print(f"\n[summary] {n} frames, {n_det} detections, {n_sp} setpoints "
              f"produced ({'DRY-RUN, nothing sent' if dry_run else 'sent'})")
    return log


def _f(x, nd=2):
    return "  None" if x is None else f"{x:.{nd}f}"


# ------------------------------------------------------------------ self-test


def self_test() -> int:
    """Synthetic-frame end-to-end check: a stub detection -> a finite velocity
    setpoint out of the flight/ core. Exercises geometry + estimator + guidance
    with NO sim, NO weights, NO camera. Exits 0 on pass, 1 on fail."""
    cfg = GuidanceConfig(mount_up_rad=math.radians(10.0))  # non-trivial tilt
    cam = CameraModel(539.936, 539.936, 640.0, 480.0)      # sim pinhole
    guid = SeekerGuidance(cfg, cam, span_m=1.0)
    stub = StubSeeker()
    frame = np.zeros((960, 1280, 3), dtype=np.uint8)       # synthetic frame

    last_sp = None
    saw_lambda_dot = False
    saw_pronav_vperp = False   # pro-nav lateral term actually reached a setpoint
    for k in range(15):
        det = stub.detect(frame, 0.05 * k)
        own = OwnState.level(cfg.alt_ref_m)
        sp, tel = guid.step(det.box_xywh, own, 0.05 * k)
        if sp is not None:
            last_sp = sp
        if tel.lambda_dot_deg_s is not None and abs(tel.lambda_dot_deg_s) > 1e-6:
            saw_lambda_dot = True
        if (sp is not None and not tel.terminal_coast
                and tel.v_perp is not None and abs(tel.v_perp) > 1e-3):
            saw_pronav_vperp = True

    ok = True

    def check(cond, msg):
        nonlocal ok
        print(f"  [{'PASS' if cond else 'FAIL'}] {msg}")
        ok = ok and cond

    check(last_sp is not None, "a setpoint is produced from a synthetic detection")
    if last_sp is not None:
        vals = last_sp.as_tuple()
        check(all(math.isfinite(v) for v in vals),
              f"setpoint is finite: {tuple(round(v, 3) for v in vals)}")
        speed = math.hypot(last_sp.v_north, last_sp.v_east)
        check(speed > 0.1, f"horizontal closing speed is non-zero ({speed:.2f} m/s)")
        check(abs(last_sp.v_north) <= cfg.v_total_max + 1e-6
              and abs(last_sp.v_east) <= cfg.v_total_max + 1e-6,
              "setpoint respects the total-velocity clamp")
    check(saw_lambda_dot, "the LOS-rate (lambda_dot) channel produced a non-zero rate")
    check(saw_pronav_vperp,
          "the pro-nav lateral term (v_perp) integrated into a live setpoint")

    # Undistort sanity: a distorted lens shifts an off-centre bearing.
    cam_d = CameraModel(539.936, 539.936, 640.0, 480.0,
                        dist=(-0.28, 0.08, 0.0, 0.0, 0.0))
    b_pin = cam.pixel_to_bearing(1100.0, 480.0)
    b_dis = cam_d.pixel_to_bearing(1100.0, 480.0)
    check(abs(b_pin - b_dis) > 1e-4,
          f"CameraModel undistort changes an edge bearing (pin={math.degrees(b_pin):.2f} "
          f"vs lens={math.degrees(b_dis):.2f} deg)")

    print(f"\n[self-test] {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


# ------------------------------------------------------------------ MAVSDK driver


# Timeouts / thresholds for the MAVSDK driver. Mirror the sim gate constants in
# scripts/m0_takeoff.py / scripts/m3_static_intercept.py (same PX4 SITL boot);
# no unsourced magic. All in wall seconds unless noted.
_CONNECT_TIMEOUT_S = 30.0    # m0_takeoff.CONNECT_TIMEOUT_S regime
_HEALTH_TIMEOUT_S = 60.0     # EKF global+home position ok after boot (m0/m3)
_TAKEOFF_TIMEOUT_S = 40.0    # climb to the takeoff altitude
_TAKEOFF_ALT_FRAC = 0.8      # "airborne" once at 80% of the target altitude (m0)
_LAND_TIMEOUT_S = 60.0       # land + disarm after the stream


async def run_mavsdk(args, cam, detector, guidance, source,
                     smoke: bool = False):  # pragma: no cover
    """Real-vehicle driver: connect over MAVLink, stream own-state EKF, and send
    the terminal's NED velocity+yaw setpoints via PX4 OFFBOARD. GUARDED mavsdk
    import so the desk/dry-run path never needs it.

    Returns 0 on success, 1 on a failure worth failing a gate over.

    Two modes, ONE shared setpoint-streaming body (this is the code the real Pi
    runs, so the SITL check exercises the real path -- not a copy):
      * TERMINAL (`smoke=False`, the vehicle path): the coded open-loop dash has
        already delivered an airborne, armed interceptor; this loop just connects,
        reads own-state, enters OFFBOARD, and streams pro-nav setpoints until the
        (infinite, live-camera) source ends -- unchanged behaviour, no arm/takeoff/
        land here (props-spinning autonomy is the dash's job, not the seeker's).
      * SITL SMOKE (`smoke=True`, the desk-gate path): there is no dash in SITL, so
        we SYNTHESISE the airborne precondition -- health-gate, arm, takeoff -- then
        run the SAME OFFBOARD stream, then land + disarm cleanly. Every flight
        bookend is gated behind `smoke`, so the vehicle path is untouched.

    Honesty: the only inputs remain camera pixels + own-state EKF (attitude quat,
    yaw, relative altitude). arm/takeoff/land are actuation, not guidance inputs;
    health/armed/landed are own status. No gt_* anywhere (grep-clean)."""
    import asyncio

    from mavsdk import System
    from mavsdk.offboard import OffboardError, VelocityNedYaw
    from mavsdk.telemetry import LandedState

    drone = System()

    def _kill_server():
        # MAVSDK-Python spawns a `mavsdk_server` child per System(); if we bail out
        # on a timeout it would otherwise linger. Kill it so a failed connect never
        # leaks a server (mirrors scripts/field/fc_link_check.py, measured 2026-07-24).
        proc = getattr(drone, "_server_process", None)
        if proc is not None and proc.poll() is None:
            proc.kill()

    print(f"[mavsdk] connecting {args.mavsdk_url} ...")
    # THE TRAP (measured 2026-07-24): mavsdk_server does not open its gRPC port until
    # it discovers a vehicle, and connect() awaits channel_ready() -- so on a DEAD
    # link connect() ITSELF blocks forever, before the connection_state() timeout
    # below can ever fire. The timeout must wrap connect(), not just _wait_connected.
    try:
        await asyncio.wait_for(
            drone.connect(system_address=args.mavsdk_url),
            timeout=_CONNECT_TIMEOUT_S)
    except asyncio.TimeoutError:
        print(f"[mavsdk] FAIL: connect() did not return within {_CONNECT_TIMEOUT_S}s "
              f"(no mavsdk_server/vehicle on {args.mavsdk_url})")
        _kill_server()
        return 1

    async def _wait_connected():
        async for st in drone.core.connection_state():
            if st.is_connected:
                return

    try:
        await asyncio.wait_for(_wait_connected(), timeout=_CONNECT_TIMEOUT_S)
    except asyncio.TimeoutError:
        print(f"[mavsdk] FAIL: no connection within {_CONNECT_TIMEOUT_S}s")
        _kill_server()
        return 1
    print("[mavsdk] connected")

    # OWN-STATE EKF streams (attitude quaternion + euler yaw + altitude) plus the
    # armed/landed STATUS used only to shut down cleanly. These are the vehicle's
    # own estimate/status -- the only non-camera inputs, no gt_*.
    state = {"quat": None, "psi": None, "alt": None, "armed": None, "landed": None}

    async def _att_q():
        async for q in drone.telemetry.attitude_quaternion():
            state["quat"] = (q.w, q.x, q.y, q.z)

    async def _att_e():
        async for e in drone.telemetry.attitude_euler():
            state["psi"] = math.radians(e.yaw_deg)

    async def _pos():
        async for p in drone.telemetry.position():
            state["alt"] = p.relative_altitude_m

    async def _armed():
        async for a in drone.telemetry.armed():
            state["armed"] = a

    async def _landed():
        async for ls in drone.telemetry.landed_state():
            state["landed"] = ls

    tasks = [asyncio.ensure_future(c())
             for c in (_att_q, _att_e, _pos, _armed, _landed)]

    rc = 0
    offboard_ok = False
    n_frame = n_sp = 0
    try:
        if smoke:
            # SITL only: synthesise the "already airborne" precondition the real
            # dash provides on the vehicle. Gated behind `smoke` so the vehicle
            # terminal path (which must NOT arm/takeoff) is untouched.
            print(f"[mavsdk] waiting for health "
                  f"(global+home position ok), timeout {_HEALTH_TIMEOUT_S}s ...")

            async def _wait_health():
                async for h in drone.telemetry.health():
                    if h.is_global_position_ok and h.is_home_position_ok:
                        return

            try:
                await asyncio.wait_for(_wait_health(), timeout=_HEALTH_TIMEOUT_S)
            except asyncio.TimeoutError:
                print(f"[mavsdk] FAIL: health not OK within {_HEALTH_TIMEOUT_S}s")
                return 1
            print("[mavsdk] health OK")

            alt = guidance.cfg.alt_ref_m
            print(f"[mavsdk] set takeoff altitude {alt:.1f} m; arming; takeoff ...")
            await drone.action.set_takeoff_altitude(alt)
            await drone.action.arm()
            await drone.action.takeoff()
            deadline = time.monotonic() + _TAKEOFF_TIMEOUT_S
            while time.monotonic() < deadline:
                if state["alt"] is not None and state["alt"] >= _TAKEOFF_ALT_FRAC * alt:
                    break
                await asyncio.sleep(0.2)
            alt_now = state["alt"]
            alt_str = "None" if alt_now is None else f"{alt_now:.2f}"
            print(f"[mavsdk] airborne at rel_alt={alt_str} m")

        # OFFBOARD: PX4 needs a setpoint STREAM established before it will accept
        # the mode switch, so prime one zero setpoint, then start (m3 pattern).
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        try:
            await drone.offboard.start()
            offboard_ok = True
            print("[mavsdk] OFFBOARD active; streaming setpoints ...")
        except OffboardError as e:
            print(f"[mavsdk] FAIL: offboard start failed: {e}")
            rc = 1

        if offboard_ok:
            t0 = time.monotonic()
            t_first = t_last = None
            for frame, name in source.frames():
                t = time.monotonic() - t0
                det = detector.detect(frame, t)
                box = getattr(det, "box_xywh", None)
                if getattr(det, "range_m", None) is None:
                    box = None
                own = OwnState(quat=state["quat"], psi_rad=state["psi"],
                               alt_m=state["alt"])
                sp, tel = guidance.step(box, own, t)
                n_frame += 1
                if sp is not None:
                    if args.dry_run:
                        print(f"[dry-run] {name} SETPOINT {sp.as_tuple()}")
                    else:
                        await drone.offboard.set_velocity_ned(VelocityNedYaw(
                            sp.v_north, sp.v_east, sp.v_down, sp.yaw_deg))
                    n_sp += 1
                    t_last = t
                    if t_first is None:
                        t_first = t
                    if n_sp % 20 == 0:
                        print(f"[mavsdk] t={t:5.1f}s frames={n_frame} "
                              f"setpoints={n_sp} last vN={sp.v_north:+.2f} "
                              f"vE={sp.v_east:+.2f} vD={sp.v_down:+.2f} "
                              f"yaw={sp.yaw_deg:+.1f}"
                              + ("  [COAST]" if tel.terminal_coast else ""))
                await asyncio.sleep(1.0 / args.fps)

            span = (t_last - t_first) if (t_first is not None
                                          and t_last is not None) else 0.0
            cadence = (n_sp - 1) / span if (span > 0 and n_sp > 1) else 0.0
            print(f"[mavsdk] stream complete: {n_frame} frames, {n_sp} setpoints "
                  f"over {span:.1f}s wall, mean cadence {cadence:.1f} Hz "
                  f"({'DRY-RUN, nothing sent' if args.dry_run else 'sent over MAVLink'})")
            if n_sp == 0:
                print("[mavsdk] FAIL: no setpoints were produced")
                rc = 1
    finally:
        # Clean shutdown regardless of how we got here: stop offboard, (smoke)
        # land + wait for disarm, and cancel the own-state stream tasks so the
        # event loop can exit without orphaned generators.
        if offboard_ok:
            try:
                await drone.offboard.stop()
                print("[mavsdk] offboard stopped")
            except Exception as e:  # noqa: BLE001 -- best-effort teardown
                print(f"[mavsdk] offboard.stop skipped: {e}")
        if smoke:
            try:
                await drone.action.land()
                print(f"[mavsdk] landing; waiting up to {_LAND_TIMEOUT_S}s "
                      f"for touchdown + disarm ...")
                deadline = time.monotonic() + _LAND_TIMEOUT_S
                touch_t = None
                forced = False
                while time.monotonic() < deadline:
                    if state["armed"] is False:     # fully disarmed -> clean stop
                        break
                    if state["landed"] == LandedState.ON_GROUND and touch_t is None:
                        touch_t = time.monotonic()
                    # PX4 auto-disarms ~2 s after touchdown (COM_DISARM_LAND); if
                    # it lags, force a disarm so the shutdown is unambiguously clean.
                    if touch_t is not None and not forced \
                            and time.monotonic() - touch_t > 5.0:
                        forced = True
                        try:
                            await drone.action.disarm()
                            print("[mavsdk] forced disarm (auto-disarm lagged)")
                        except Exception as e:  # noqa: BLE001
                            print(f"[mavsdk] disarm request skipped: {e}")
                    await asyncio.sleep(0.5)
                print(f"[mavsdk] landed_state={state['landed']} "
                      f"armed={state['armed']}")
            except Exception as e:  # noqa: BLE001 -- best-effort teardown
                print(f"[mavsdk] land skipped: {e}")
        for tk in tasks:
            tk.cancel()

    print(f"[sitl-smoke] {'PASS' if rc == 0 else 'FAIL'}" if smoke else
          f"[mavsdk] done rc={rc}")
    return rc


# ------------------------------------------------------------------ CLI


def build_detector(args, cam: CameraModel, span_m: float):
    """The ONNX fine-tuned seeker (scripts/seeker/finetuned_seeker.py), fed the
    SAME intrinsics as the LOS math so bearing/range are self-consistent.

    `--gray-input auto` (the default) resolves the input modality PER MODEL:
    gray for the gray-native real-data nn_tier weights, color for the older
    color-trained sim weights (finetuned_seeker.resolve_input_modality). Real
    OV9281 frames are already mono, so the gray step is a bit-exact no-op there."""
    from finetuned_seeker import FinetunedNNSeeker
    gray = {"auto": "auto", "on": True, "off": False}[getattr(args, "gray_input", "auto")]
    return FinetunedNNSeeker(cam.fx, cam.fy, cam.cx, cam.cy, args.weights,
                             conf_thres=args.conf, target_span_m=span_m,
                             gray_input=gray)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Real-hardware camera-only pro-nav terminal seeker loop.")
    ap.add_argument("--self-test", action="store_true",
                    help="synthetic end-to-end check (no sim/weights/camera); exits 0/1")
    ap.add_argument("--sitl-smoke", action="store_true",
                    help="drive the LIVE MAVSDK OFFBOARD path against a local PX4 "
                         "SITL with a synthetic detector (no camera/weights/onnx): "
                         "health-gate, arm, takeoff, stream setpoints, land+disarm. "
                         "Needs --mavsdk-url. Exits 0/1. (Do NOT pass --dry-run: PX4 "
                         "fail-safes out of OFFBOARD without a live setpoint stream.)")
    ap.add_argument("--smoke-duration", type=float, default=45.0,
                    help="--sitl-smoke: wall seconds of setpoint streaming. Default "
                         "45 s clears the >=30 s sim-time bar even if RTF sags to "
                         "~0.67 (GPU render holds RTF ~0.95, ADR-0075).")
    ap.add_argument("--source",
                    help="frame source: an image dir, a video file, or 'picamera'")
    ap.add_argument("--weights",
                    default=os.path.join(_SEEKER_DIR, "weights", "nn_tier",
                                         "n-mono.onnx"),
                    help="ONNX detector weights. DEFAULT = the REAL-DATA model "
                         "n-mono (YOLO11n, COCO-init, grayscale-native): on "
                         "source-disjoint real held-out imagery it scores AP50 "
                         "0.442 / recall 44.2%% / false-fire 4.9%% vs the "
                         "sim-trained quad_v2's 0.0003 / 1.1%% / 88.5%% "
                         "(logs/nn_tier/eval_n-mono_heldout.csv, n=4175). The sim "
                         "model stays selectable: --weights "
                         "scripts/seeker/weights/drone_finetuned_quad_v2.onnx")
    ap.add_argument("--gray-input", choices=("auto", "on", "off"), default="auto",
                    help="feed the detector 3-channel GRAYSCALE. 'auto' (default) "
                         "picks per-model: gray for the gray-native nn_tier "
                         "weights, color for the color-trained sim weights. On the "
                         "real OV9281 (mono sensor) the conversion is a bit-exact "
                         "no-op; it only bites on COLOR bench/sim replay.")
    ap.add_argument("--intrinsics",
                    default=os.path.join(_REPO_ROOT, "camera_intrinsics.json"),
                    help="camera_intrinsics.json / calibrate_camera.py output")
    ap.add_argument("--conf", type=float, default=0.25, help="detector confidence")
    ap.add_argument("--fps", type=float, default=20.0, help="control loop rate")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--dry-run", action="store_true",
                    help="PRINT setpoints instead of sending over MAVSDK")
    ap.add_argument("--mavsdk-url", default=None,
                    help="MAVSDK connect URL (e.g. udpin://0.0.0.0:14540); "
                         "omit for desk/dry-run over --source")
    # Mount config (real hardware; docs/hardware_order_list.md).
    ap.add_argument("--mount-fwd-m", type=float, default=0.10)
    ap.add_argument("--mount-left-m", type=float, default=0.0)
    ap.add_argument("--mount-up-m", type=float, default=0.05)
    ap.add_argument("--mount-tilt-deg", type=float, default=0.0,
                    help="fixed camera up-tilt (degrees)")
    ap.add_argument("--n-pronav", type=float, default=5.0)
    args = ap.parse_args(argv)

    if args.self_test:
        return self_test()

    if args.sitl_smoke:
        # Validate the LIVE MAVSDK OFFBOARD path (flight/deploy/README.md item 4)
        # against a local PX4 SITL, with a synthetic detector so NO camera /
        # weights / onnxruntime are needed -- the point is the MAVLink plumbing
        # (connect, mode switch, setpoint cadence, clean land), not perception.
        if not args.mavsdk_url:
            ap.error("--sitl-smoke requires --mavsdk-url "
                     "(e.g. udpin://0.0.0.0:14540)")
        cfg = GuidanceConfig(
            n_pronav=args.n_pronav,
            mount_fwd_m=args.mount_fwd_m, mount_left_m=args.mount_left_m,
            mount_up_m=args.mount_up_m,
            mount_up_rad=math.radians(args.mount_tilt_deg))
        cam = (load_camera(args.intrinsics) if os.path.exists(args.intrinsics)
               else CameraModel(539.936, 539.936, 640.0, 480.0))  # sim pinhole
        span_m = cfg.target_span_m
        n_frames = max(1, int(args.smoke_duration * args.fps))
        print(f"[sitl-smoke] LIVE MAVSDK OFFBOARD check | url={args.mavsdk_url} | "
              f"fx={cam.fx:.1f} cx={cam.cx:.1f} span={span_m:.2f}m N={cfg.n_pronav} "
              f"| fps={args.fps} duration~{args.smoke_duration:.0f}s "
              f"({n_frames} frames)")
        guidance = SeekerGuidance(cfg, cam, span_m)
        detector = SmokeSeeker(cam.fx, span_m, cx=cam.cx, cy=cam.cy)
        source = SyntheticSource(n_frames)
        import asyncio
        return asyncio.run(
            run_mavsdk(args, cam, detector, guidance, source, smoke=True))

    if not args.source:
        ap.error("need --source <image-dir|video|picamera> (or --self-test)")

    cfg = GuidanceConfig(
        n_pronav=args.n_pronav,
        mount_fwd_m=args.mount_fwd_m, mount_left_m=args.mount_left_m,
        mount_up_m=args.mount_up_m, mount_up_rad=math.radians(args.mount_tilt_deg))
    cam = load_camera(args.intrinsics)
    span_m, span_src = resolve_span(args.weights, cfg)
    print(f"[config] intrinsics fx={cam.fx:.1f} cx={cam.cx:.1f} "
          f"dist={'yes' if cam.has_distortion else 'none (pinhole)'} | "
          f"span={span_m:.3f} m ({span_src}) | N={cfg.n_pronav} "
          f"| mount fwd={cfg.mount_fwd_m} up={cfg.mount_up_m} "
          f"tilt={math.degrees(cfg.mount_up_rad):.1f}deg")

    detector = build_detector(args, cam, span_m)
    guidance = SeekerGuidance(cfg, cam, span_m)

    # Frame source.
    if args.source == "picamera":
        source = PicameraSource()
    elif os.path.isdir(args.source):
        source = ImageDirSource(args.source, fps=args.fps)
    elif args.source.lower().endswith((".mp4", ".avi", ".mov", ".mkv")):
        source = VideoSource(args.source)
    else:
        source = ImageDirSource(args.source, fps=args.fps)  # single image path

    # Real MAVLink vs desk dry-run.
    if args.mavsdk_url and not args.dry_run:
        import asyncio
        return asyncio.run(run_mavsdk(args, cam, detector, guidance, source))
    run_over_source(source, detector, guidance, dry_run=True,
                    max_frames=args.max_frames, fps=args.fps)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
