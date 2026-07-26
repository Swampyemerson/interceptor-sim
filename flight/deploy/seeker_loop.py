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
    read. There is NO ground-truth symbol anywhere in this file (there is no ground
    truth on real hardware to read). That is MACHINE-CHECKED, not grepped: this file
    is in real_flight.py's `_AUDITED_MODULES`, so `python -m flight.deploy.real_flight
    --audit` walks its syntax tree (pinned by flight/tests/test_real_flight.py's
    injection-calibration test). The `flight/` core it calls is audited the same way.
    The ONNX detector's weights were fine-tuned OFFLINE on gt-projected labels (an
    offline human-labeler analogue) -- the LIVE detector at inference is gt-free.
  * On the DESK (no flight controller), own-state falls back to a LEVEL hover
    (identity attitude, yaw 0, a fixed altitude) via `OwnState.level()`. That is a
    benign BENCH stand-in, not ground truth: it says "assume the camera is level,"
    which is exactly what a bench test with the camera on a table is. It is NOT
    allowed to become a flight input: `GuidanceConfig.require_own_attitude`
    (default True) makes `SeekerGuidance.step` REFUSE to steer when the live
    own-state is missing/stale, instead of silently substituting the stand-in.

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

    # --- TERMINAL-COAST LATCH HYGIENE (review2 BLOCKER, 2026-07-25) ------------
    # The freeze is a LATCH: once armed it flies a constant NED vector through
    # CPA, and un-freezing v_perp mid-terminal is GRAVEYARDED (ADR-0023 measured
    # split-freeze WORSE, +0.084 m, 2/12 better). The defect was not the latch,
    # it was that a SINGLE oversized/merged detector box could arm it -- the
    # range channel carries no rate_cap (deliberate: m4 parity), so one 3x box at
    # 12 m injects rdot_hat ~ -72 m/s, predict() coasts r_hat through 3.5 m and
    # the camera is disconnected for the rest of the engagement, silently.
    # Three guards, none of which REJECTS a measurement (that family is
    # graveyarded -- "phantom range-plausibility gates", ADR-0077); they only
    # decide when the LATCH may arm and when a demonstrably spurious one is
    # released:
    coast_arm_ticks: int = 2        # consecutive FRESHLY-CORRECTED ticks with
                                    # r_hat < terminal_freeze_range_m required to
                                    # arm. m4 parity is "only on a detected tick"
                                    # (m4_intercept.py:3589 `if detected:`); the
                                    # 2nd tick is the corroboration that kills a
                                    # lone gross outlier. 1 = m4-parity only.
    # RELEASE (a legitimate latch can NEVER hit either bar -- pinned by test):
    # at v_close_terminal=5.5 m/s a real 3.5 m freeze reaches CPA in ~0.6 s, so
    # it can neither see fresh detections back out at 2x the freeze range nor
    # outlive coast_max_s. Firing either is PROOF the arming was spurious.
    coast_release_range_m: Optional[float] = 7.0   # = 2 x terminal_freeze_range_m
    coast_release_ticks: int = 3    # consecutive fresh corrections above it
    coast_max_s: Optional[float] = 3.0            # ~5x the physical coast time
    # BROKEN TRACK: a negative range or a non-physical range rate is proof the
    # range channel has diverged. It must never silently select the coast branch;
    # it raises a health flag the caller (real_flight FAILSAFE 8) breaks off on.
    rdot_sane_max_ms: Optional[float] = None      # None -> 2 x v_total_max

    # --- MEASUREMENT STALENESS (review2 BLOCKER #2) ----------------------------
    # Past this age the LOS rate is an extrapolation, not a measurement: stop
    # integrating a_cmd into v_perp (measured +1.8 m/s of lateral velocity
    # accumulated on a stale rate in 0.5 s) and flag the tick. 0.25 s is >= 3
    # detector periods at the measured ~14 Hz onboard cadence, so the nominal
    # 20 Hz-loop/14 Hz-detector path (max gap ~0.07-0.14 s) is UNAFFECTED.
    v_perp_stale_s: float = 0.25

    # --- OWN-STATE PRECONDITION (review2 HIGH) ---------------------------------
    # With quat None, derotate_bearing_lambda returns the PRE-FIX yaw-only
    # lambda = psi + beta, and psi None additionally assumes the nose points
    # NORTH -- i.e. guidance steers on the DESK stand-in (level, yaw 0). That is
    # a benign stand-in on a bench and a fabricated control input in the air, and
    # it is indistinguishable in the log. Refuse to steer instead: the caller
    # holds (real_flight coasts the last dash velocity) and the stream never gaps.
    require_own_attitude: bool = True
    # Age gate. Deliberately None (OFF): no measured own-state warm-up/stall
    # latency exists yet -- measure it on bench brn-05 from the logged
    # own_age_s column BEFORE sizing this. The flag is recorded either way.
    own_state_max_age_s: Optional[float] = None

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
    # Seconds since the newest own-state sample arrived (None = not instrumented).
    # A telemetry generator that dies leaves this dict FROZEN at its last value
    # with no other trace, so the age is the only thing that can see it.
    age_s: Optional[float] = None

    @staticmethod
    def level(alt_m: float) -> "OwnState":
        """Desk stand-in: camera level, yaw 0. (identity quat = no rotation).

        This is a BENCH stand-in, never a flight input: `GuidanceConfig.
        require_own_attitude` makes the terminal REFUSE to steer when the live
        own-state is absent, rather than silently substituting this."""
        return OwnState(quat=(1.0, 0.0, 0.0, 0.0), psi_rad=0.0, alt_m=alt_m,
                        age_s=0.0)


def own_state_status(own: OwnState, cfg: GuidanceConfig) -> Tuple[bool, str]:
    """Is this own-state fit to derive a STEERING command from? Returns
    (ok, reason). Pure, so the guard is unit-testable without a vehicle.

    `require_own_attitude=False` restores the old permissive behaviour (the
    yaw-only / level fallback), for a deliberate bench experiment only."""
    if not cfg.require_own_attitude:
        return True, "ok"
    if own.quat is None:
        return False, "no_own_attitude_quat"
    if own.psi_rad is None:
        return False, "no_own_yaw"
    if cfg.own_state_max_age_s is not None and own.age_s is not None \
            and own.age_s > cfg.own_state_max_age_s:
        return False, "own_state_stale"
    return True, "ok"


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
    # --- HEALTH (review2): a tick that LOOKS nominal must be distinguishable
    #     from one that is coasting, stale, or running on a stand-in. `health`
    #     is the machine-readable fault list the caller/log consumes; empty =
    #     clean. Nothing here changes the command; it makes the command legible.
    own_state_ok: bool = True
    own_age_s: Optional[float] = None
    meas_age_s: Optional[float] = None   # seconds since the last fresh detection
    stale: bool = False                  # meas_age_s past v_perp_stale_s
    yaw_hold: bool = False               # yaw cmd coasted, not re-derived
    track_broken: bool = False           # r_hat <= 0 or |rdot_hat| non-physical
    coast_age_s: Optional[float] = None  # seconds since the freeze armed
    health: List[str] = field(default_factory=list)


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
    velocity+yaw setpoint (or None before first lock). It is the ENGAGE-phase
    math from scripts/m4_intercept.py, calling the SAME flight/ core.

    THREE DELIBERATE DIVERGENCES from m4 (review2 silent-failure audit,
    2026-07-25 -- each fixes a path that produced a confident, plausible, WRONG
    command with nothing raised; all three are NO-OPS on a nominal tick and are
    pinned byte-identical by flight/tests/test_seeker_loop_coast.py):

      1. The terminal-coast freeze may only ARM off freshly-corrected ticks and
         needs `coast_arm_ticks` corroboration, and a demonstrably spurious latch
         RELEASES (range or timeout). Pre-fix, ONE oversized box + a short
         dropout latched it permanently -- the camera was disconnected for the
         rest of the engagement while every logged value stayed plausible.
      2. The absolute yaw command is LATCHED and coasted on the estimated LOS
         rate through a dropout. Pre-fix it was re-derived from the LIVE yaw plus
         a stale bearing every dark tick, so the command chased the vehicle it
         was steering (measured 90-108 deg of uncommanded yaw in 2 s).
      3. It REFUSES to steer when the own-state EKF is absent/stale rather than
         falling back to the level/yaw-0 desk stand-in.

    `step` NEVER raises: a fault is reported on `StepTelemetry.health` (and
    printed once), and the caller decides. Returning None means "I have no
    trustworthy command" -- real_flight holds the last dash velocity, so the
    OFFBOARD setpoint stream never gaps."""

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
        # --- review2 fix state ------------------------------------------------
        self._last_det_t: Optional[float] = None    # last FRESH correction time
        self._coast_arm_count = 0                   # consecutive corroborations
        self._coast_release_count = 0
        self._coast_t0: Optional[float] = None      # when the freeze armed
        self._yaw_cmd_deg: Optional[float] = None   # LATCHED absolute yaw command
        self._warned_own_state = False
        self._faults_seen: set = set()
        self.n_coast_releases = 0                   # health counters (log/tests)
        self.n_track_broken = 0

    def _emit_fault(self, tel: "StepTelemetry", flag: str, msg: str,
                    once: bool = False) -> None:
        """Record a fault on the tick AND say it out loud (once, if asked). A
        fault that only exists as a struct field is the failure class this whole
        pass is about -- the flag goes on EVERY affected tick so the log shows
        the extent; the print is rate-limited so it stays readable."""
        tel.health.append(flag)
        if not once or flag not in self._faults_seen:
            self._faults_seen.add(flag)
            print(f"[seeker] FAULT {flag}: {msg}")

    def step(self, det_box, own: OwnState, t: float) -> Tuple[Optional[Setpoint], StepTelemetry]:
        cfg = self.cfg
        dt = 0.05 if self._last_t is None else max(1e-3, t - self._last_t)
        self._last_t = t

        # PREDICT every tick (coast the estimate forward on the current rate).
        self.lam.predict(dt)
        self.rng.predict(dt)

        tel = StepTelemetry(t=t, detected=det_box is not None)
        tel.own_age_s = own.age_s

        # ---- OWN-STATE PRECONDITION (review2 HIGH) ---------------------------
        # Without a live attitude the LOS math silently reverts to the pre-fix
        # yaw-only formula on an assumed-level, assumed-north vehicle -- a
        # fabricated steering input that is finite, plausible and invisible.
        # REFUSE to steer (and, critically, refuse to INITIALISE the tracker off
        # a stand-in-derived lambda); the caller holds and the stream never gaps.
        own_ok, own_why = own_state_status(own, cfg)
        tel.own_state_ok = own_ok
        if not own_ok:
            tel.stale = True
            self._emit_fault(
                tel, own_why,
                "own-state EKF unavailable/stale -- NOT steering on the level/"
                "yaw-0 stand-in (guidance holds; see GuidanceConfig."
                "require_own_attitude)", once=True)
            return None, tel

        # CORRECT on a fresh detection.
        bearing_h = None
        fresh = det_box is not None
        if det_box is not None:
            bearing_h, range_meas, meas_xyz = measurement_from_box(
                det_box, cfg, self.cam, self.span_m)
            self._last_bearing_rad = bearing_h
            self._last_det_t = t
            tel.bearing_deg = math.degrees(bearing_h)
            tel.range_meas_m = range_meas
            # Camera-frame LOS azimuth -> inertial, derotating the FULL own
            # attitude (+ fixed mount up-tilt). Own EKF only, no gt_*.
            lambda_meas = derotate_bearing_lambda(
                meas_xyz, bearing_h, own.quat,
                (0.0 if own.psi_rad is None else own.psi_rad),
                mount_up_rad=cfg.mount_up_rad)
            # Re-anchor camera range/LOS from the CAMERA to the CG (lever arm).
            range_cg, lambda_cg = camera_to_cg_los(
                range_meas, lambda_meas, own.quat, cfg.cam_offset_body)
            self.lam.correct(lambda_cg, t)
            self.rng.correct(range_cg, t)

        # MEASUREMENT AGE: past v_perp_stale_s the LOS rate is an extrapolation.
        meas_age = None if self._last_det_t is None else (t - self._last_det_t)
        tel.meas_age_s = meas_age
        tel.stale = meas_age is not None and meas_age > cfg.v_perp_stale_s

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

        # ---- BROKEN RANGE TRACK ---------------------------------------------
        # r_hat <= 0 is geometrically impossible and |rdot_hat| above ~2x the
        # vehicle's own speed clamp cannot be a real closure -- both mean the
        # (deliberately rate-cap-free) range channel has diverged. This must not
        # silently select the coast branch; it raises a flag the caller aborts on.
        rdot_bound = (cfg.rdot_sane_max_ms if cfg.rdot_sane_max_ms is not None
                      else 2.0 * cfg.v_total_max)
        broken = ((r_hat is not None and r_hat <= 0.0)
                  or (rdot_hat is not None and abs(rdot_hat) > rdot_bound))
        if broken:
            tel.track_broken = True
            self.n_track_broken += 1
            self._emit_fault(
                tel, "range_track_broken",
                f"range channel diverged (r_hat={r_hat}, rdot_hat={rdot_hat}, "
                f"|rdot| bound {rdot_bound:.1f} m/s) -- coast latch INHIBITED",
                once=True)

        # ---- TERMINAL COAST: arm / release ----------------------------------
        # Inside the freeze range lambda_dot is singular -> lock the collision-
        # course velocity vector and fly it straight through CPA. Once armed the
        # latch HOLDS by design (ADR-0023: split-freeze measured worse); the only
        # exits are the two spurious-latch proofs below.
        if self._frozen_vworld is not None:
            coast_age = 0.0 if self._coast_t0 is None else (t - self._coast_t0)
            tel.coast_age_s = coast_age
            if fresh and r_hat is not None and cfg.coast_release_range_m is not None \
                    and r_hat > cfg.coast_release_range_m:
                self._coast_release_count += 1
            elif fresh:
                self._coast_release_count = 0
            release_why = None
            if cfg.coast_release_ticks and cfg.coast_release_range_m is not None \
                    and self._coast_release_count >= cfg.coast_release_ticks:
                release_why = ("coast_released_range",
                               f"{self._coast_release_count} consecutive fresh "
                               f"detections at r_hat > "
                               f"{cfg.coast_release_range_m:.1f} m while frozen")
            elif cfg.coast_max_s is not None and coast_age > cfg.coast_max_s:
                release_why = ("coast_released_timeout",
                               f"the freeze has been latched {coast_age:.2f} s "
                               f"(> {cfg.coast_max_s:.1f} s); a real 3.5 m coast "
                               f"reaches CPA in well under 1 s")
            if release_why is not None:
                self._frozen_vworld = None
                self._coast_t0 = None
                self._coast_arm_count = 0
                self._coast_release_count = 0
                self.n_coast_releases += 1
                self._emit_fault(tel, release_why[0],
                                 release_why[1] + " -- the latch was SPURIOUS; "
                                 "releasing back to closed-loop pro-nav")

        if self._frozen_vworld is None:
            # ARM only off FRESHLY-CORRECTED ticks (m4 parity, m4_intercept.py:3589
            # `if detected:`) and only with corroboration -- a no-detection tick
            # HOLDS the count, a fresh tick back outside the range RESETS it.
            below = (r_hat is not None and r_hat < cfg.terminal_freeze_range_m)
            if fresh and not broken:
                self._coast_arm_count = self._coast_arm_count + 1 if below else 0
            if self._coast_arm_count >= max(1, int(cfg.coast_arm_ticks)):
                u = (math.cos(lambda_hat), math.sin(lambda_hat))
                p = (-math.sin(lambda_hat), math.cos(lambda_hat))
                self._frozen_vworld = (v_close * u[0] + self.v_perp * p[0],
                                       v_close * u[1] + self.v_perp * p[1])
                self._coast_t0 = t
                self._coast_release_count = 0
                tel.coast_age_s = 0.0

        in_coast = self._frozen_vworld is not None
        if not in_coast:
            # STALE-RATE GUARD: only integrate the pro-nav acceleration while the
            # LOS rate is backed by a recent measurement. On a dropout the rate is
            # an extrapolation and integrating it walks v_perp to its clamp.
            if not tel.stale:
                self.v_perp = _clamp(self.v_perp + a_cmd * dt,
                                     -cfg.v_perp_max, cfg.v_perp_max)
            else:
                self._emit_fault(
                    tel, "v_perp_hold_stale",
                    f"no fresh detection for {meas_age:.2f} s "
                    f"(> {cfg.v_perp_stale_s:.2f} s) -- HOLDING v_perp instead of "
                    "integrating a_cmd off an extrapolated LOS rate", once=True)
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
            vh_n, vh_e = self._frozen_vworld
        tel.v_perp = self.v_perp
        tel.terminal_coast = in_coast

        # Altitude hold (own-state alt only; None on the desk -> level).
        if own.alt_m is not None:
            v_down = _clamp(cfg.kp_alt * (own.alt_m - cfg.alt_ref_m),
                            -cfg.v_vert_max, cfg.v_vert_max)
        else:
            v_down = 0.0

        # ---- ABSOLUTE YAW COMMAND -------------------------------------------
        # On a DETECTION tick: nose on the LOS, yaw = psi + bearing (unchanged --
        # byte-identical to the pre-fix expression on every detected tick).
        # On a DARK tick the pre-fix code re-evaluated psi + <stale bearing> with
        # the LIVE psi, so as the vehicle slewed toward the command the command
        # ran away with it -- a pure positive-feedback integrator (measured ~108
        # deg of uncommanded yaw in 2 s). Instead LATCH the absolute command and
        # coast it on the ESTIMATED LOS rate, which is the constant-LOS-rate
        # doctrine flight/terminal_coast.py already implements, and is bounded by
        # the estimator's lambda_rate_cap.
        psi_deg = 0.0 if own.psi_rad is None else math.degrees(own.psi_rad)
        if bearing_h is not None or self._yaw_cmd_deg is None:
            self._yaw_cmd_deg = psi_deg + math.degrees(self._last_bearing_rad)
        else:
            self._yaw_cmd_deg += math.degrees(lambda_dot or 0.0) * dt
            tel.yaw_hold = True
        yaw_deg = self._yaw_cmd_deg

        sp = Setpoint(vh_n, vh_e, v_down, yaw_deg)
        tel.setpoint = sp
        return sp, tel


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ------------------------------------------------------------------ frame sources


class ImageDirSource:
    """Iterate PNG/JPG frames from a directory in sorted order -- the DESK test
    mode over existing captured frames (scripts/seeker/data/*/images)."""

    _IMG_EXT = (".png", ".jpg", ".jpeg")

    def __init__(self, path: str, fps: float = 20.0, loop: bool = False):
        pats = ("*.png", "*.jpg", "*.jpeg")
        files: List[str] = []
        if os.path.isdir(path):
            # Common layouts: <dir>/*.png or <dir>/images/*.png
            for base in (path, os.path.join(path, "images")):
                for p in pats:
                    files += glob.glob(os.path.join(base, p))
        else:
            # FAIL CLOSED on a bad --source (2026-07-26). This branch used to
            # accept ANY string: a typo'd path became files=[<typo>], cv2.imread
            # returned None on every frame, and the run reported "0 frames, 0
            # setpoints" -- which reads as a dead FC or a dead detector and sent
            # the bench operator to re-check TX/RX on a link that was working.
            # os.path.exists alone is not enough: an EXISTING non-image file
            # (a .txt) reproduces the same silent zero-frame run.
            if not os.path.isfile(path):
                raise FileNotFoundError(f"--source path does not exist: {path}")
            if not path.lower().endswith(self._IMG_EXT):
                raise ValueError(
                    f"--source {path} is not an image "
                    f"({'/'.join(self._IMG_EXT)}), a directory, a video, or "
                    f"'picamera'")
            files = [path]
        self.files = sorted(set(files))
        if not self.files:
            raise FileNotFoundError(f"no images under {path}")
        self.dt = 1.0 / fps
        self.loop = loop
        # SHRINKING DENOMINATOR, COUNTED not absorbed: a dir where most images
        # are corrupt used to yield a partial n_frame and a clean PASS.
        self.n_unreadable = 0

    def frames(self):
        import cv2
        i = 0
        while True:
            f = self.files[i]
            img = cv2.imread(f)
            if img is not None:
                yield img, os.path.basename(f)
            else:
                self.n_unreadable += 1
                if self.n_unreadable <= 3:
                    print(f"[source] UNREADABLE, skipped: {f}")
            i += 1
            if i >= len(self.files):
                if self.n_unreadable:
                    print(f"[source] {self.n_unreadable}/{len(self.files)} "
                          f"file(s) under the --source path were unreadable and "
                          f"were SKIPPED (the frame count below is the survivors)")
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


# The <=1 ms exposure SPEC TARGET, mirrored from the instrument that measures
# against it (scripts/seeker/pi_capture.EXPOSURE_SPEC_US). It is duplicated, not
# imported, because `flight/` cannot import `scripts/` (no package there) -- so
# the equality is PINNED by tests/test_exposure_spec_transfers.py, which loads
# pi_capture directly and fails if the two ever drift. The tripod day measures
# the decode-range / recall envelope at this exposure and this loop flies against
# that verdict, so they must be the same number
# (docs/tripod_test_protocol.md §3.3/§4.4; docs/camera_paper_check.md item 3).
EXPOSURE_SPEC_US = 1000


def picam_exposure_controls(camera_controls, exposure_us, gain=1.0):
    """The picamera2 control dict that pins EXPOSURE manual and leaves GAIN auto.

    Pure + off-Pi testable on purpose: it is the only part of the camera setup
    that can be exercised without hardware, and it is the part that carries the
    decision. `camera_controls` is the live camera's advertised control map (or
    None on an older stack)."""
    avail = camera_controls or {}
    if "ExposureTimeMode" in avail:
        # 1 == Manual, 0 == Auto for libcamera's split mode controls; they take
        # precedence over AeEnable in the same request.
        return {"ExposureTimeMode": 1, "ExposureTime": int(exposure_us),
                "AnalogueGainMode": 0}
    return {"AeEnable": False, "ExposureTime": int(exposure_us),
            "AnalogueGain": float(gain)}


def picam_exposure_verdict(applied_us, spec_us=EXPOSURE_SPEC_US):
    """One line stating whether the FLOWN exposure matches the condition the
    tripod curve was measured under. `None` (no ExposureTime in the metadata) is
    UNVERIFIED, never 'fine' -- fail-closed on a measured quantity."""
    if applied_us is None:
        return ("[picam] WARNING: the sensor reported no ExposureTime -- the "
                f"<={spec_us} us condition the tripod decode-range / recall "
                "curve was measured under is UNVERIFIED on this flight.")
    if applied_us > spec_us:
        return (f"[picam] WARNING: applied exposure {applied_us:.0f} us > "
                f"{spec_us} us spec -- motion blur at 9 m/s is "
                f"~{applied_us / float(spec_us):.1f}x what the tripod curve was "
                f"measured at, so its ranges do not transfer to this flight.")
    return f"[picam] exposure {applied_us:.0f} us (<= {spec_us} us spec) -- OK"


class PicameraSource:
    """Live frames from the real Pi 5 camera (Picamera2). GUARDED import -- only
    constructed on the vehicle; absent on this x86 desk.

    `size` MUST be the resolution the intrinsics were CALIBRATED at -- main()
    reads it out of the calibration file and refuses to construct this source
    without one. The old hard-coded (1280, 960) was the SIM camera's shape while
    the adopted sensor is the innomaker OV9281 at 1280x800 native
    (scripts/seeker/pi_capture.py DEF_WIDTH/DEF_HEIGHT), so the ISP rescaled
    every frame away from the grid fx/fy/cx/cy describe. The default here is the
    OV9281 native shape, but nothing on the live path relies on it.

    Requesting a size is NOT the same as getting it (libcamera may substitute a
    size the sensor supports). We deliberately do NOT try to read the negotiated
    size back out of picamera2 -- that would pin this file to picamera2 internals
    we cannot exercise off-Pi. The universal first-frame shape assertion in the
    drivers (frame_shape_fault) catches an adjusted size on the frame itself.

    EXPOSURE IS PINNED HERE, NOT LEFT TO AUTO-EXPOSURE (2026-07-26). The tripod
    instrument (scripts/seeker/pi_capture.py) captures with AE off at the <=1 ms
    spec and VERIFIES the applied value -- that is the optical condition under
    which R_decode90 and the NN recall curve are measured, and it is the whole
    reason a global-shutter sensor was bought. This source used to set NO camera
    controls at all, so the FLYING camera ran picamera2's auto-exposure: 8-20 ms
    in overcast light is 8-20x the measured motion blur at 9 m/s closing, and
    detection would die at ranges the tripod curve certified. The instrument and
    the operational system disagreeing on the one knob the sensor was chosen for
    is invisible to every offline test AND to the tripod data itself -- neither
    arm can see it.

    EXPOSURE MANUAL, GAIN AUTO. libcamera's control_ids_core.yaml documents
    `AeEnable` as setting BOTH ExposureTimeMode and AnalogueGainMode, so pinning
    gain too would mean a flight in different light than the bench session gets a
    black or blown frame with no adaptation -- a NEW failure the auto-everything
    code did not have. Where libcamera's split mode controls exist we use them
    (manual exposure, auto gain); otherwise we fall back to AeEnable=False with an
    explicit gain. Either way the APPLIED value is read back off the first
    request's metadata and logged, because a requested-but-unverified exposure is
    the same class of claim the instrument refuses to make."""

    def __init__(self, size=(1280, 800), exposure_us=EXPOSURE_SPEC_US, gain=1.0):
        try:
            from picamera2 import Picamera2  # type: ignore
        except ImportError as exc:  # pragma: no cover -- hardware only
            raise RuntimeError(
                "Picamera2 not available -- run on the Pi, or use "
                "--source <image-dir|video> on the desk.") from exc
        self.cam = Picamera2()
        self.exposure_us = int(exposure_us)
        self.applied_exposure_us = None      # read back off the FIRST request
        self.controls = picam_exposure_controls(
            getattr(self.cam, "camera_controls", None), self.exposure_us, gain)
        config = self.cam.create_video_configuration(
            main={"size": size, "format": "RGB888"}, controls=self.controls)
        self.cam.configure(config)
        self.cam.start()
        self.cam.set_controls(self.controls)

    def frames(self):  # pragma: no cover -- hardware only
        import cv2
        while True:
            # capture_request, not capture_array: it is the only way to see the
            # per-frame metadata, i.e. what the sensor ACTUALLY applied.
            request = self.cam.capture_request()
            try:
                rgb = request.make_array("main")
                md = request.get_metadata()
            finally:
                request.release()
            if self.applied_exposure_us is None:
                self.applied_exposure_us = md.get("ExposureTime")
                print(picam_exposure_verdict(self.applied_exposure_us),
                      file=sys.stderr)
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


def resolve_span(weights: str, cfg: GuidanceConfig,
                 explicit_m: Optional[float] = None):
    """Effective known-size span. Returns (span_m, source, calibrated).

    Order (matching FinetunedNNSeeker): an EXPLICIT --target-span-m, then the
    MARKERLESS_SPAN_M env, then a <weights>.calib.json sidecar, then the config
    default. `calibrated` is False ONLY for that last case.

    WHY THE THIRD RETURN VALUE (review2 HIGH): NO nn_tier model has a calib
    sidecar -- only the retired SIM models do -- so on real hardware the span
    silently became cfg.target_span_m = 1.0 m, a constant fitted to the SIM
    quad, and the run printed a reassuring "span=1.000 m (config default)".
    That constant is not a display value: range = fx*span/box_width_px feeds
    EVERY range-keyed threshold (closing-speed throttle 5.0 m, terminal freeze
    3.5 m, breakoff arm 4.0 m, hard floor 0.5 m) and multiplies Vc into the
    pro-nav command. Callers must be able to see it was never measured."""
    if explicit_m is not None:
        return float(explicit_m), f"--target-span-m {explicit_m}", True
    env = os.environ.get("MARKERLESS_SPAN_M")
    if env:
        try:
            return float(env), f"env MARKERLESS_SPAN_M={env}", True
        except ValueError:
            pass
    sidecar = str(weights) + ".calib.json"
    if os.path.exists(sidecar):
        try:
            with open(sidecar) as fh:
                return (float(json.load(fh)["span_m_eff"]),
                        f"calib {os.path.basename(sidecar)}", True)
        except Exception:
            pass
    return cfg.target_span_m, "config default (NOT CALIBRATED)", False


def warn_uncalibrated_span(span_m: float, source: str, weights: str,
                           cfg: GuidanceConfig, live: bool,
                           require: bool = False) -> int:
    """Print the consequence of an uncalibrated span; return 1 if the caller
    should REFUSE. Silent when the span is calibrated.

    Default is a LOUD warning rather than a refusal because a live PERCEPTION
    run over the uncalibrated n-mono weights (scripts/check_deploy_bench.sh with
    BENCH_SYNTHETIC=0) has no way to pass a span; `--require-span-calib` (or a
    fitted sidecar) is the switch that makes it fail-closed. The bench LINK gate
    itself no longer touches this path at all -- it runs --synthetic-detector,
    whose span is a self-consistent synthetic constant, not a range claim."""
    if not live or "NOT CALIBRATED" not in source:
        return 0
    lvl = "REFUSING" if require else "WARNING"
    print(f"\n[span] {lvl}: NO RANGE CALIBRATION for {os.path.basename(weights)}")
    print(f"[span]   using span = {span_m:.3f} m from the config default -- a "
          f"constant fitted to the SIM target, not this detector on this target.")
    print(f"[span]   range = fx*span/box_width_px, so EVERY range-keyed threshold "
          f"fires at the wrong true distance:")
    print(f"[span]     closing-speed throttle {cfg.throttle_range_m:.1f} m, "
          f"terminal freeze {cfg.terminal_freeze_range_m:.1f} m "
          f"(+ real_flight breakoff arm / hard floor)")
    print(f"[span]   and span multiplies Vc into a_cmd = N*Vc*lambda_dot, so an "
          f"uncalibrated run flies an effective N far from the ADR-0011 N=5.")
    print(f"[span]   FIX: fit scripts/seeker/calibrate_range.py -> "
          f"{os.path.basename(weights)}.calib.json, or pass --target-span-m, "
          f"or set MARKERLESS_SPAN_M.\n")
    return 1 if require else 0


# --------------------------------------------------- intrinsics/frame contract


class FrameShapeMismatch(RuntimeError):
    """The delivered frames are not the pixel grid the intrinsics describe."""


def frame_shape_fault(frame, cam: CameraModel) -> Optional[str]:
    """Return None if this frame matches the resolution the intrinsics were
    CALIBRATED at, else the message to refuse with.

    WHY THIS EXISTS (2026-07-26): fx/fy/cx/cy only describe ONE pixel grid. Feed
    them frames of a different size and every bearing shifts, box_width_px
    rescales, and range = fx*span/box_width_px is wrong -- which then moves EVERY
    range-keyed threshold (closing throttle 5.0 m, terminal freeze 3.5 m,
    real_flight's breakoff arm 4.0 m) and multiplies Vc into the pro-nav command.
    Nothing anywhere compared the two, because CameraModel.from_dict used to
    DISCARD the calibration's `resolution` key.

    A calibration that declares no resolution returns None (older sidecars must
    keep loading); the LIVE-camera path refuses that case up front instead."""
    if cam.width is None or cam.height is None:
        return None
    h, w = frame.shape[:2]
    if (w, h) == (cam.width, cam.height):
        return None
    return (f"frame is {w}x{h} but the intrinsics were calibrated at "
            f"{cam.width}x{cam.height} -- fx/fy/cx/cy do not describe these "
            f"pixels, so every bearing and every range = fx*span/box_width_px "
            f"is silently rescaled/shifted")


# ------------------------------------------------------------------ desk driver


def run_over_source(source, detector, guidance: SeekerGuidance,
                    dry_run: bool = True, max_frames: Optional[int] = None,
                    fps: float = 20.0, verbose: bool = True,
                    cam: Optional[CameraModel] = None) -> List[StepTelemetry]:
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
        if i == 0 and cam is not None:
            fault = frame_shape_fault(frame, cam)
            if fault:
                print(f"[frame] FAULT intrinsics/frame mismatch: {fault}")
                raise FrameShapeMismatch(fault)
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
_ARMED_WAIT_S = 10.0         # bounded wait for the ARMED status (--require-disarmed)

# ---- STREAM ACCEPTANCE BAR (what a rc=0 actually asserts) --------------------
# The only stream criterion used to be `n_sp == 0`, so ONE setpoint PASSED -- and
# the line printed directly above the PASS read "mean cadence 0.0 Hz", a verdict
# computed over one unit. These are named so scripts/check_deploy_bench.sh's
# header and this code cannot drift apart.
_STREAM_MIN_SETPOINTS = 2    # cadence is UNDEFINED on <2 samples -> UNVERIFIABLE
_STREAM_MIN_SPAN_S = 1.0     # a shorter stream measures nothing
_STREAM_MIN_HZ = 2.0         # the offboard streaming rate of thumb in the PX4 docs
# THE BINDING NUMBER IS THE GAP, NOT THE MEAN. PX4 treats offboard availability
# as a RECENCY test -- `hrt_absolute_time() < offboard_control_mode.timestamp +
# COM_OF_LOSS_T` (v1.16.0 src/modules/commander/HealthAndArmingChecks/checks/
# offboardCheck.cpp) with COM_OF_LOSS_T defaulting to 1.0 s (v1.16.0
# src/modules/commander/commander_params.c). A 20 Hz mean with a 3 s hole in the
# middle is a stream PX4 would have dropped, so the GAP is what we fail on.
_STREAM_MAX_GAP_S = 1.0      # = COM_OF_LOSS_T default
_STREAM_WARN_GAP_S = 0.5     # half of it: loud, but not a failure
# BENCH LIVENESS BOUND on the own-state round trip. This is NOT
# GuidanceConfig.own_state_max_age_s -- that constant is deliberately unset
# because brn-05 exists to MEASURE the own-state latency that sizes it, and
# picking it here would be inventing the number the run is supposed to produce.
# This is a blunt "the link stopped" bound, orders above any plausible warm-up.
_OWN_AGE_ABORT_S = 2.0


def _stream_verdict(*, n_frame, n_det, n_sp, span, cadence, max_gap_s,
                    max_frames, stop_reason) -> int:
    """Does this setpoint stream meet the bar the bench gate CLAIMS to verify?

    Returns 0 (pass) or 1 (fail) and prints the reason with the measured value.
    Pure + argument-only so it is exercisable without MAVSDK or hardware.

    NO VACUOUS VERDICTS: a stream of <2 setpoints has no defined cadence, so it
    is UNVERIFIABLE, never a pass. A truncated stream that never saw a detection
    is named as a BOUND/config problem, not reported the same as a detector that
    saw nothing over the whole source -- they send the operator to different
    places."""
    if n_sp == 0:
        if max_frames is not None and "max-frames" in stop_reason:
            print(f"[mavsdk] FAIL: no setpoints -- the stream was TRUNCATED by "
                  f"--max-frames ({max_frames}) after {n_det} detection(s) in "
                  f"{n_frame} frames. That is a BOUND/source problem, not "
                  f"necessarily a link or detector one: raise --max-frames, or "
                  f"point --source at frames that contain the target.")
        else:
            print(f"[mavsdk] FAIL: no setpoints over the WHOLE source "
                  f"({n_frame} frames, {n_det} detections)")
        return 1
    if n_sp < _STREAM_MIN_SETPOINTS or span <= 0.0:
        print(f"[mavsdk] FAIL: stream UNVERIFIABLE -- {n_sp} setpoint(s) over "
              f"{span:.2f}s; cadence is undefined on <{_STREAM_MIN_SETPOINTS} "
              f"samples and must not be reported as 0.0 Hz")
        return 1
    if span < _STREAM_MIN_SPAN_S:
        print(f"[mavsdk] FAIL: stream lasted {span:.2f}s "
              f"(< {_STREAM_MIN_SPAN_S:.1f}s required)")
        return 1
    if cadence is not None and cadence < _STREAM_MIN_HZ:
        print(f"[mavsdk] FAIL: mean cadence {cadence:.2f} Hz "
              f"(< {_STREAM_MIN_HZ:.1f} Hz required)")
        return 1
    if max_gap_s >= _STREAM_MAX_GAP_S:
        print(f"[mavsdk] FAIL: max inter-setpoint gap {max_gap_s:.2f}s >= "
              f"COM_OF_LOSS_T ({_STREAM_MAX_GAP_S:.1f}s) -- PX4 would have "
              f"dropped OFFBOARD in that hole, whatever the mean says")
        return 1
    if max_gap_s > _STREAM_WARN_GAP_S:
        print(f"[mavsdk] WARNING: max inter-setpoint gap {max_gap_s:.2f}s is "
              f"over half of COM_OF_LOSS_T ({_STREAM_MAX_GAP_S:.1f}s)")
    return 0


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
    # `t_own` is the monotonic stamp of the NEWEST own-state sample. Without it a
    # telemetry generator that dies silently freezes the dict at its last value
    # and guidance keeps steering on it with nothing raised (review2).
    state = {"quat": None, "psi": None, "alt": None, "armed": None,
             "landed": None, "t_own": None}

    async def _att_q():
        async for q in drone.telemetry.attitude_quaternion():
            state["quat"] = (q.w, q.x, q.y, q.z)
            state["t_own"] = time.monotonic()

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

    # LINK LIVENESS (2026-07-26). Nothing watched the link after connect: a
    # Dupont jumper working loose mid-run left MAVSDK's own resend timer queueing
    # setpoints into a dead port, n_sp kept climbing, and the gate printed PASS
    # on a link that had been dead for half the run. `n_sp` counts messages the
    # Pi QUEUED, so it can never be the liveness evidence -- these two flags and
    # the own-state round trip below are.
    fault = {"link_lost": False, "task_died": None}

    async def _conn():
        async for st in drone.core.connection_state():
            if not st.is_connected:
                fault["link_lost"] = True

    tasks = [asyncio.ensure_future(c())
             for c in (_att_q, _att_e, _pos, _armed, _landed, _conn)]

    def _task_died(tk):
        # A telemetry generator that dies silently FREEZES the state dict at its
        # last value and guidance keeps steering on it (real_flight.py's
        # `_task_died` pattern). Make it loud and fatal instead.
        if tk.cancelled():
            return
        exc = tk.exception()
        if exc is not None:
            fault["task_died"] = repr(exc)
            print(f"[mavsdk] FAULT own-state telemetry task died: {exc!r}")

    for tk in tasks:
        tk.add_done_callback(_task_died)

    require_disarmed = bool(getattr(args, "require_disarmed", False))
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

        # FIRST-SETPOINT PRECONDITION (review2 HIGH): do not start the guidance
        # loop until the own-state EKF is actually streaming. Otherwise the first
        # detections initialise the LOS tracker off the level/yaw-0 stand-in
        # (guidance.step now REFUSES those ticks, so without this wait the loop
        # would simply produce no setpoints and look like a dead seeker).
        # Bounded + LOUD: never a silent fallback.
        # ATTITUDE (quat + yaw) is HARD: it is the LOS input, it comes from the
        # IMU/EKF and needs no GPS, and without it guidance is a stand-in.
        # ALTITUDE is a LOUD WARNING, not a gate: it only feeds the altitude-hold
        # v_down term, and a props-off bench FC indoors legitimately has no
        # position solution -- failing there would block bring-up for no honesty
        # gain. Its absence is now visible (v_down 0) instead of unremarked.
        _own_deadline = time.monotonic() + _HEALTH_TIMEOUT_S
        while time.monotonic() < _own_deadline:
            if state["quat"] is not None and state["psi"] is not None:
                break
            await asyncio.sleep(0.1)
        if state["quat"] is None or state["psi"] is None:
            print(f"[mavsdk] FAIL: own-state EKF ATTITUDE did not stream within "
                  f"{_HEALTH_TIMEOUT_S}s (quat={state['quat'] is not None} "
                  f"psi={state['psi'] is not None}) -- refusing to guide on the "
                  f"level/yaw-0 stand-in (GuidanceConfig.require_own_attitude)")
            return 1
        if state["alt"] is None:
            print("[mavsdk] WARNING: no own-state ALTITUDE yet -- altitude hold "
                  "is inert (v_down = 0) until position streams.")
        print("[mavsdk] own-state EKF attitude streaming (quat + yaw); "
              f"alt={'yes' if state['alt'] is not None else 'NOT YET'}")

        # --- BENCH-ONLY DISARM INTERLOCK (--require-disarmed, default OFF) -----
        # scripts/check_deploy_bench.sh's headline safety claim is "the FC stays
        # DISARMED the whole time -- nothing can spin", and it was enforced only
        # by the ABSENCE of an arm call: `armed` was subscribed and never read.
        # An FC left armed from an RC bind or a QGC params check would take a
        # 13 m/s pro-nav velocity stream. This makes the documented property an
        # ENFORCED one -- but strictly opt-in, because the REAL terminal runs
        # ARMED and AIRBORNE by design (the coded dash delivers it that way), so
        # a blanket armed-refusal would break the mission.
        # FAIL CLOSED on the telemetry too: state["armed"] starts None, and a
        # bare `if state["armed"]:` reads None as "not armed" -- a vacuous pass
        # of exactly the class this guard exists for.
        if require_disarmed:
            _armed_deadline = time.monotonic() + _ARMED_WAIT_S
            while state["armed"] is None and time.monotonic() < _armed_deadline:
                await asyncio.sleep(0.1)
            if state["armed"] is None:
                print(f"[mavsdk] UNCERTAIN: --require-disarmed but the ARMED "
                      f"status never streamed within {_ARMED_WAIT_S:.0f}s -- "
                      f"refusing to certify a 'disarmed' run we cannot observe")
                return 1
            if state["armed"]:
                print("[mavsdk] FAIL: --require-disarmed and the FC is ARMED. "
                      "Refusing to enter OFFBOARD and stream velocity setpoints "
                      "into an armed vehicle. Disarm it, then re-run.")
                return 1
            print("[mavsdk] armed=False confirmed (--require-disarmed)")

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
            t_own_start = state["t_own"]
            t0 = time.monotonic()
            t_first = t_last = t_prev_sp = None
            max_gap_s = 0.0
            max_own_age_s = 0.0
            n_det = 0
            first_lock = None            # (frame index, frame name) of first lock
            stop_reason = "source exhausted"
            abort = None                 # not None -> rc 1, with this reason
            max_frames = getattr(args, "max_frames", None)
            max_seconds = getattr(args, "max_seconds", None)
            for i, (frame, name) in enumerate(source.frames()):
                # STREAM BOUNDS. --max-frames used to be parsed, passed by
                # check_deploy_bench.sh, and IGNORED here (honoured only in the
                # desk driver), so the documented bound was inert -- and the
                # documented `--source picamera` path, whose generator is
                # `while True`, could only ever end on the outer 180 s timeout.
                if max_frames is not None and i >= max_frames:
                    stop_reason = f"--max-frames bound reached ({max_frames})"
                    break
                t = time.monotonic() - t0
                if max_seconds is not None and t >= max_seconds:
                    stop_reason = f"--max-seconds bound reached ({max_seconds:.0f}s)"
                    break
                if fault["link_lost"]:
                    abort = ("the MAVLink link DROPPED mid-run "
                             "(connection_state went not-connected)")
                    break
                if fault["task_died"] is not None:
                    abort = (f"an own-state telemetry stream died mid-run "
                             f"({fault['task_died']}) -- own-state would have "
                             f"frozen at its last value")
                    break
                if require_disarmed and state["armed"]:
                    abort = ("the FC ARMED mid-run (--require-disarmed) -- "
                             "setpoint stream stopped immediately")
                    break
                if i == 0:
                    _shape = frame_shape_fault(frame, cam)
                    if _shape:
                        abort = f"intrinsics/frame mismatch: {_shape}"
                        break
                det = detector.detect(frame, t)
                box = getattr(det, "box_xywh", None)
                if getattr(det, "range_m", None) is None:
                    box = None
                _t_own = state["t_own"]
                own = OwnState(quat=state["quat"], psi_rad=state["psi"],
                               alt_m=state["alt"],
                               age_s=(None if _t_own is None
                                      else time.monotonic() - _t_own))
                if own.age_s is not None:
                    max_own_age_s = max(max_own_age_s, own.age_s)
                    if own.age_s > _OWN_AGE_ABORT_S:
                        abort = (f"own-state went STALE ({own.age_s:.1f}s > the "
                                 f"{_OWN_AGE_ABORT_S:.0f}s bench liveness bound) "
                                 f"-- the FC stopped answering; every setpoint "
                                 f"after this would be steered off a frozen "
                                 f"attitude")
                        break
                sp, tel = guidance.step(box, own, t)
                n_frame += 1
                if tel.detected:
                    n_det += 1
                    if first_lock is None:
                        first_lock = (i, name)
                if sp is not None:
                    if args.dry_run:
                        print(f"[dry-run] {name} SETPOINT {sp.as_tuple()}")
                    else:
                        await drone.offboard.set_velocity_ned(VelocityNedYaw(
                            sp.v_north, sp.v_east, sp.v_down, sp.yaw_deg))
                    n_sp += 1
                    if t_prev_sp is not None:
                        max_gap_s = max(max_gap_s, t - t_prev_sp)
                    t_prev_sp = t_last = t
                    if t_first is None:
                        t_first = t
                    if n_sp % 20 == 0:
                        _age = ("None" if own.age_s is None
                                else f"{own.age_s:.2f}s")
                        print(f"[mavsdk] t={t:5.1f}s frames={n_frame} "
                              f"det={n_det} setpoints={n_sp} last vN="
                              f"{sp.v_north:+.2f} "
                              f"vE={sp.v_east:+.2f} vD={sp.v_down:+.2f} "
                              f"yaw={sp.yaw_deg:+.1f} own_age={_age}"
                              + ("  [COAST]" if tel.terminal_coast else "")
                              + (f"  [{','.join(tel.health)}]"
                                 if tel.health else ""))
                await asyncio.sleep(1.0 / args.fps)

            if abort is not None:
                stop_reason = "ABORTED"
            span = (t_last - t_first) if (t_first is not None
                                          and t_last is not None) else 0.0
            cadence = ((n_sp - 1) / span) if (span > 0 and n_sp > 1) else None
            cad_s = ("n/a (<2 setpoints -- UNDEFINED, not 0.0)" if cadence is None
                     else f"{cadence:.1f} Hz")
            lock_s = ("none" if first_lock is None
                      else f"frame #{first_lock[0]} {first_lock[1]}")
            # EVERY DENOMINATOR, ALWAYS PRINTED. `n_sp` is the number of setpoints
            # the Pi QUEUED to MAVSDK, not the number that crossed the wire (the
            # mavsdk_server re-sends the last one on its own timer), so it is
            # labelled as such -- the wire evidence is the own-state round trip.
            print(f"[mavsdk] stream complete ({stop_reason}): {n_frame} frames, "
                  f"{n_det} detections, {n_sp} setpoints over {span:.1f}s wall, "
                  f"mean cadence {cad_s}, max inter-setpoint gap "
                  f"{max_gap_s:.2f}s, max own_age {max_own_age_s:.2f}s, "
                  f"first lock {lock_s} "
                  f"({'DRY-RUN, nothing queued' if args.dry_run else 'queued to MAVSDK'})")

            if abort is not None:
                print(f"[mavsdk] FAIL: {abort}")
                rc = 1
            else:
                rc = max(rc, _stream_verdict(
                    n_frame=n_frame, n_det=n_det, n_sp=n_sp, span=span,
                    cadence=cadence, max_gap_s=max_gap_s,
                    max_frames=max_frames, stop_reason=stop_reason))
                # OWN-STATE ROUND TRIP: `t_own` advancing is the only proof that
                # bytes came BACK over the wire for the whole stream, and it
                # needs no tuned constant -- it either advanced or it did not.
                # (This is also the run that MEASURES the own_age distribution
                # GuidanceConfig.own_state_max_age_s is waiting on: max own_age
                # is printed above.)
                if (state["t_own"] is None or t_own_start is None
                        or state["t_own"] <= t_own_start):
                    print("[mavsdk] FAIL: the own-state stamp did NOT advance "
                          "across the stream -- no telemetry came back over the "
                          "link while we were queueing setpoints into it")
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
                # NOT "skipped": a successful stop needs a command ACK from the
                # FC, so a failed one is positive evidence the link was not
                # delivering. Swallowing it let a dead-link run still print PASS.
                print(f"[mavsdk] FAIL: offboard.stop() did not succeed ({e}) -- "
                      f"the FC never ACKed the stop, so the link was not "
                      f"round-tripping at teardown")
                rc = 1
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
    ap.add_argument("--max-frames", type=int, default=None,
                    help="stop after N frames. Honoured on BOTH the desk and "
                         "the MAVSDK path (it was silently ignored on the "
                         "MAVSDK one until 2026-07-26). Required in practice "
                         "for --source picamera, whose stream never ends.")
    ap.add_argument("--max-seconds", type=float, default=None,
                    help="stop after N wall seconds of streaming. A bound that "
                         "does not depend on the frame RATE, so a slow detector "
                         "cannot push the run past a caller's outer timeout.")
    ap.add_argument("--synthetic-detector", action="store_true",
                    help="drive the MAVSDK path with the DETERMINISTIC "
                         "SmokeSeeker + blank frames instead of the camera/ONNX "
                         "detector, WITHOUT arming anything (unlike "
                         "--sitl-smoke). This is what makes a link test a link "
                         "test: the setpoint count stops being a perception "
                         "outcome. Needs --max-frames or --max-seconds to bound "
                         "the stream.")
    ap.add_argument("--require-disarmed", action="store_true",
                    help="BENCH interlock: refuse to enter OFFBOARD (exit 1) "
                         "unless the FC reports armed=False, and abort if it "
                         "arms mid-run. Default OFF because the real terminal "
                         "runs armed and airborne by design.")
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
    ap.add_argument("--target-span-m", type=float, default=None,
                    help="EXPLICIT known-size span (m) for range = fx*span/"
                         "box_width_px. Highest precedence, ahead of "
                         "MARKERLESS_SPAN_M and the <weights>.calib.json "
                         "sidecar. No nn_tier model ships a sidecar yet, so on "
                         "real hardware this is currently the ONLY calibrated "
                         "source -- without it the span falls back to a constant "
                         "fitted to the SIM target and every range-keyed "
                         "threshold fires at the wrong true distance.")
    ap.add_argument("--require-span-calib", action="store_true",
                    help="on the LIVE path, REFUSE to run (exit 1) when the span "
                         "is the uncalibrated config default, instead of warning. "
                         "Make this the default once a fitted sidecar exists.")
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

    if not args.source and not args.synthetic_detector:
        ap.error("need --source <image-dir|video|picamera> (or --self-test, or "
                 "--synthetic-detector for a perception-free link test)")

    cfg = GuidanceConfig(
        n_pronav=args.n_pronav,
        mount_fwd_m=args.mount_fwd_m, mount_left_m=args.mount_left_m,
        mount_up_m=args.mount_up_m, mount_up_rad=math.radians(args.mount_tilt_deg))
    cam = load_camera(args.intrinsics)
    span_m, span_src, span_ok = resolve_span(args.weights, cfg,
                                             args.target_span_m)
    _res = ("unstated" if cam.width is None
            else f"{cam.width}x{cam.height}")
    print(f"[config] intrinsics fx={cam.fx:.1f} cx={cam.cx:.1f} "
          f"res={_res} src={cam.source or 'unstamped'} | "
          f"dist={'yes' if cam.has_distortion else 'none (pinhole)'} | "
          f"span={span_m:.3f} m ({span_src}) | N={cfg.n_pronav} "
          f"| mount fwd={cfg.mount_fwd_m} up={cfg.mount_up_m} "
          f"tilt={math.degrees(cfg.mount_up_rad):.1f}deg")

    # ---- PERCEPTION-FREE LINK TEST (--synthetic-detector) --------------------
    # The bench gate's job is the UART/OFFBOARD link, and it used to earn its
    # PASS from whatever the ONNX detector happened to fire on -- measured, the
    # first setpoint of a default bench run came from a hallucinated detection
    # on a frame with no target in it, and on a night without that false fire
    # the same correctly-wired hardware produces n_sp=0 and FAILs. A link test
    # whose verdict is controlled by detector noise is not a link test.
    # SmokeSeeker + SyntheticSource make the setpoint count DETERMINISTIC, and
    # `smoke` stays False so nothing arms / takes off / lands.
    if args.synthetic_detector:
        if args.max_frames is None and args.max_seconds is None:
            ap.error("--synthetic-detector needs --max-frames or --max-seconds "
                     "to bound the stream")
        n_frames = (args.max_frames if args.max_frames is not None
                    else max(1, int(args.max_seconds * args.fps)))
        detector = SmokeSeeker(cam.fx, span_m, cx=cam.cx, cy=cam.cy)
        source = SyntheticSource(
            n_frames, size=((cam.height or 960), (cam.width or 1280)))
        guidance = SeekerGuidance(cfg, cam, span_m)
        print(f"[config] SYNTHETIC DETECTOR: {n_frames} blank frames + "
              f"SmokeSeeker at a constant {span_m:.2f} m span -- setpoints are "
              f"deterministic, so a PASS is a LINK result, not a perception one")
        if args.mavsdk_url and not args.dry_run:
            import asyncio
            return asyncio.run(run_mavsdk(args, cam, detector, guidance, source))
        run_over_source(source, detector, guidance, dry_run=True,
                        max_frames=args.max_frames, fps=args.fps, cam=cam)
        return 0

    # LIVE = a real vehicle link with a real camera; the desk replay/dry-run keeps
    # the 1.0 fallback so nothing offline breaks.
    _live = bool(args.mavsdk_url) and not args.dry_run
    if warn_uncalibrated_span(span_m, span_src, args.weights, cfg, live=_live,
                              require=args.require_span_calib):
        return 1

    detector = build_detector(args, cam, span_m)
    guidance = SeekerGuidance(cfg, cam, span_m)

    # Frame source.
    if args.source == "picamera":
        # FAIL CLOSED: the camera must be streamed at the resolution the lens was
        # CALIBRATED at. This used to be a hard-coded (1280, 960) -- the SIM
        # camera's shape -- while the real OV9281 is 1280x800 native, so the ISP
        # rescaled every frame away from the grid fx/cx describe and nothing
        # compared the two. A resolution is a MEASURED quantity; no default.
        if cam.width is None or cam.height is None:
            print(f"[config] FAIL: --source picamera needs the CALIBRATED "
                  f"resolution, and {args.intrinsics} declares none. Re-run "
                  f"scripts/calibrate_camera.py on this lens (its output "
                  f"carries a `resolution` block) -- do not guess a size.")
            return 1
        source = PicameraSource(size=(cam.width, cam.height))
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
    try:
        run_over_source(source, detector, guidance, dry_run=True,
                        max_frames=args.max_frames, fps=args.fps, cam=cam)
    except FrameShapeMismatch:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
