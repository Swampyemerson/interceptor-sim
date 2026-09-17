"""flight.tag_terminal -- a 3-D predicted-intercept-point (PIP) camera terminal
that USES THE KNOWN TAG SIZE for range, instead of the bearing-only pro-nav law
in `flight.deploy.seeker_loop.SeekerGuidance`.

WHY THIS EXISTS. `SeekerGuidance` only closes at 5.5-9 m/s against a 9 m/s
target, holds altitude with a P-loop instead of steering it, and never uses the
range an AprilTag of known size actually gives. This module estimates the full
3-D relative position AND the target's own velocity, then solves the
constant-velocity intercept triangle every tick (the same triangle
`flight.guidance.collision_lead_heading` solves ONCE, pre-flight, generalized to
3-D and re-solved closed-loop) -- a predicted-intercept-point (PIP) law.

SAME CONTRACT AS SeekerGuidance (drops into an UNMODIFIED `RealFlightSM` by
duck-typing -- no `isinstance` check anywhere on `RealFlightSM.guidance`):

    step(det_box_xywh, own: OwnState, t: float) -> (Optional[Setpoint], StepTelemetry)

HONESTY (CLAUDE.md; ADR-0008/0010). Inputs are (a) a detector box (camera
pixels) and (b) `own: OwnState` -- own-state EKF attitude/altitude/velocity.
No `gt_*` anywhere (this module is in `real_flight._AUDITED_MODULES`, same
AST audit as `seeker_loop.py`). `own.vel_ned` is own-state, not ground truth.

ROUND 2 (this pass) -- per head review of round 1 (measured: miss 17.8 m,
hovering target, camera lost the tag at ~40 deg nose-down dash pitch; the law
never got to use its measurements):

  1. LOOK-ANGLE-AWARE ACCEL LIMIT. The only lever is HOW FAST the velocity
     setpoint is slewed (the autopilot picks the actual tilt). Reuses
     `flight.fov_guidance.pitch_headroom_accel_cap` for the nose-down
     (target->top-edge) bound and the SAME function MIRRORED (negate eps_v
     AND pitch) for the nose-up/braking (target->bottom-edge) bound -- see
     `_look_angle_accel_bounds` for the algebra. Applied to the along-LOS
     component of the horizontal command only, floored at `accel_floor_ms2`
     on the forward side so a tight geometry throttles rather than stalls.
  2. SEPARATE horizontal/vertical slew budgets (round 1's single combined-
     norm cap let a big horizontal correction starve a small vertical one --
     measured: raising it 6->60 only moved a 2 m vertical miss to 1.8 m).
  3. ESTIMATOR REPLACED: `_RelStateKF`, a 6-state (r, v_t) constant-velocity
     Kalman filter with a KNOWN control input (own velocity) and ANISOTROPIC
     per-correction measurement noise (`range_measurement_noise`: along-LOS
     grows as range^2, cross-LOS as range). Removes round 1's dilemma -- one
     fixed gain that tracked a 9 m/s target fast enough also amplified
     sparse pixel noise into wild swings (measured v_t_hat=(-20,-2.5,1.0)
     against a true (-9,-1,0)); a KF's gain adapts on its own.
  4. SPEED SCHEDULE: `v_engage_ms` is a CEILING throttled by what the look-
     angle cap can deliver from the current along-LOS speed over
     `speed_schedule_horizon_s`, so a standing start doesn't demand the full
     ceiling (and its pitch) on tick one.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from flight.camera import CameraModel
from flight.fov_guidance import FovHoldConfig, half_fov_from_intrinsics, pitch_headroom_accel_cap
from flight.geometry import quat_rotate
from flight.guidance import GRAVITY_MS2, floor_v_down
from flight.deploy.seeker_loop import (
    GuidanceConfig,
    OwnState,
    Setpoint,
    StepTelemetry,
    measurement_from_box,
    own_state_status,
)

__all__ = ["TagTerminalConfig", "TagInterceptGuidance", "solve_intercept_t_go"]


@dataclass
class TagTerminalConfig:
    """Every constant the PIP terminal needs. Camera mount tilt/offset and the
    own-state precondition (`require_own_attitude`/`own_state_max_age_s`) are
    NOT duplicated here -- they live on the `GuidanceConfig` (`gcfg`) passed to
    `TagInterceptGuidance`, the SAME instance `SeekerGuidance` reads."""

    v_engage_ms: float = 14.0          # commanded closing speed CEILING, m/s
    fallback_lead_s: float = 1.0       # no-solution heading guess horizon, s
    v_vert_max_ms: float = 4.0         # |v_down| clamp, m/s (NED, + = down)
    yaw_rate_max_deg_s: float = 180.0  # slew limit on the commanded yaw
    coast_max_s: float = 1.0           # keep predicting+commanding this long dark
    freeze_range_m: float = 1.5        # inside this |r_hat|, freeze v_cmd (fly through)
    meas_latency_s: float = 0.045      # capture -> available-to-guidance latency
    v_t_prior_ned: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # initial v_t_hat
    min_agl_m: Optional[float] = None
    alt_floor_kp: float = 1.0

    # --- item 2: separate horizontal/vertical command-slew budgets ---------
    cmd_accel_horiz_max_ms2: float = 12.0
    cmd_accel_vert_max_ms2: float = 6.0

    # --- item 1: look-angle-aware acceleration limit ------------------------
    # Usable half-FOV = half_vfov (derived from cam.fy/cam.height) x this
    # fraction -- the safety margin inside the geometric edge (detector needs
    # the whole tag in frame, not just its centre).
    fov_margin_frac: float = 0.7
    # Never let the look-angle cap throttle FORWARD progress below this, so a
    # tight-but-not-impossible geometry still closes, just slower.
    accel_floor_ms2: float = 1.5

    # --- item 4: speed schedule ---------------------------------------------
    speed_schedule_horizon_s: float = 1.0

    # --- item 3: 6-state KF process/measurement noise -----------------------
    q_target_accel_ms2: float = 3.0    # target-maneuver process noise density
    pixel_noise_px: float = 0.3        # centroid (u,v) 1-sigma -> cross-LOS
    side_noise_px: float = 0.3 * math.sqrt(2.0)  # box-width 1-sigma -> along-LOS


def solve_intercept_t_go(r: np.ndarray, v_t: np.ndarray, v: float) -> Optional[float]:
    """Smallest t_go > 0 solving |r + v_t*t_go| = v*t_go (the constant-velocity
    collision triangle, generalized from `flight.guidance.collision_lead_heading`'s
    2-D compass form to a 3-D NED vector). Returns None if no positive real root
    exists (the target cannot be caught at this commanded speed on this heading --
    e.g. a target receding at >= v)."""
    a = float(np.dot(v_t, v_t) - v * v)
    b = float(2.0 * np.dot(r, v_t))
    c = float(np.dot(r, r))
    if abs(a) < 1e-9:
        if abs(b) < 1e-9:
            return None
        t = -c / b
        return t if t > 1e-3 else None
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    roots = [(-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)]
    positive = [t for t in roots if t > 1e-3]
    return min(positive) if positive else None


def _wrap_deg(a: float) -> float:
    return (float(a) + 180.0) % 360.0 - 180.0


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _optical_vec_to_ned(meas_xyz, quat, mount_up_rad: float) -> np.ndarray:
    """A camera-OPTICAL-frame vector (x right, y down, z forward; magnitude
    carries the known-size range) -> the SAME physical vector expressed in NED
    -- the exact transform chain `flight.geometry.derotate_bearing_lambda` uses
    internally, just returning the full 3-vector instead of one atan2. Pinned
    against that function's azimuth in `flight/tests/test_tag_terminal.py`."""
    rx, ry, rz = float(meas_xyz[0]), float(meas_xyz[1]), float(meas_xyz[2])
    body = (rz, rx, ry)   # optical -> body FRD
    if mount_up_rad:
        ct, st = math.cos(mount_up_rad), math.sin(mount_up_rad)
        bx, by, bz = body
        body = (ct * bx + st * bz, by, -st * bx + ct * bz)
    return np.array(quat_rotate(quat, body), dtype=float)


def _ned_vec_to_optical(ned_vec, quat, mount_up_rad: float):
    """INVERSE of `_optical_vec_to_ned`: a NED vector -> the camera-OPTICAL
    vector it would appear as, given the current attitude. Used to estimate
    the tag's look angle on ticks with no fresh detection (from the filter's
    own `r_hat`). Round-trip-pinned against `_optical_vec_to_ned` in
    `flight/tests/test_tag_terminal.py`. None if `quat` is unavailable."""
    if quat is None:
        return None
    w, x, y, z = quat
    body = quat_rotate((w, -x, -y, -z), tuple(float(c) for c in ned_vec))
    if mount_up_rad:
        ct, st = math.cos(mount_up_rad), math.sin(mount_up_rad)
        bx, by, bz = body
        body = (ct * bx - st * bz, by, st * bx + ct * bz)
    bx, by, bz = body
    return (by, bz, bx)   # body FRD -> optical (x=right, y=down, z=fwd)


def _cam_offset_ned(quat, cam_offset_body: Tuple[float, float, float]) -> np.ndarray:
    """Camera position relative to the CG, in NED -- the SAME lever-arm vector
    `flight.geometry.camera_to_cg_los` adds into its (range, lambda) re-anchor."""
    fwd, left, up = cam_offset_body
    if not (fwd or left or up) or quat is None:
        return np.zeros(3)
    return np.array(quat_rotate(quat, (fwd, -left, -up)), dtype=float)


def _pitch_rad_from_quat(quat) -> Optional[float]:
    """Own-state pitch (rad), NOSE-DOWN NEGATIVE -- the exact convention
    `flight.fov_guidance.pitch_headroom_accel_cap` expects (its docstring:
    theta = -atan(a_fwd/g), nose down for positive forward accel). Standard
    ZYX/aerospace extraction, pinned in tests against
    `flight.geometry.euler_to_quat_body_to_ned`'s own convention."""
    if quat is None:
        return None
    w, x, y, z = (float(c) for c in quat)
    return math.asin(_clamp(2.0 * (w * y - z * x), -1.0, 1.0))


def _look_angle_accel_bounds(eps_v: Optional[float], pitch_rad: Optional[float],
                             fov_cfg: FovHoldConfig
                             ) -> Tuple[Optional[float], Optional[float]]:
    """(a_min, a_max): admissible along-LOS forward-accel range whose steady
    pitch keeps the tag inside the usable vertical FOV. a_max (>=0) reuses
    `pitch_headroom_accel_cap` UNCHANGED: accelerating TOWARD the target
    pitches nose-down, pushing the tag toward the TOP edge. a_min (<=0) is
    the algebraic MIRROR (negate BOTH eps_v and pitch_rad) for the opposite
    failure -- braking pitches nose-UP, pushing the tag toward the BOTTOM
    edge; that function's internal bound `theta_min=pitch-(eps_v+limit)`
    becomes, under the negation, exactly `-(limit-eps_v+pitch)`, the mirror
    image of the SAME derivation for the nose-up side. Pinned symmetrically
    in `flight/tests/test_tag_terminal.py`."""
    if eps_v is None or pitch_rad is None:
        return None, None
    a_max = pitch_headroom_accel_cap(eps_v, pitch_rad, fov_cfg, GRAVITY_MS2)
    a_min = -pitch_headroom_accel_cap(-eps_v, -pitch_rad, fov_cfg, GRAVITY_MS2)
    return a_min, a_max


def range_measurement_noise(range_m: float, direction_ned: np.ndarray,
                            cfg: TagTerminalConfig, cam: CameraModel,
                            tag_side_m: float) -> np.ndarray:
    """3x3 measurement-noise covariance for one fix, in NED -- diagonal in the
    LOS frame (along-LOS range noise ~ range^2, cross-LOS bearing noise ~
    range), rotated into NED for the KF's `correct`. `direction_ned` need
    not be a unit vector."""
    sigma_cross = max(range_m, 0.0) * cfg.pixel_noise_px / cam.fx
    sigma_along = max(range_m, 0.0) ** 2 * cfg.side_noise_px / (cam.fx * tag_side_m)
    n = np.asarray(direction_ned, dtype=float)
    norm = float(np.linalg.norm(n))
    if norm < 1e-9:
        return np.eye(3) * max(sigma_cross, 1e-3) ** 2
    e_los = n / norm
    up = np.array([0.0, 0.0, -1.0])
    e1 = np.cross(up, e_los)
    if float(np.linalg.norm(e1)) < 1e-6:      # e_los nearly vertical -> use north
        e1 = np.cross(np.array([1.0, 0.0, 0.0]), e_los)
    e1 = e1 / max(float(np.linalg.norm(e1)), 1e-9)
    e2 = np.cross(e_los, e1)
    rot = np.stack([e_los, e1, e2])           # world -> LOS-frame rows
    diag_local = np.diag([sigma_along ** 2, sigma_cross ** 2, sigma_cross ** 2])
    return rot.T @ diag_local @ rot


class _RelStateKF:
    """6-state constant-velocity KF on (r, v_t), NED, with a KNOWN control
    input: predict subtracts `v_own*dt` from r. Anisotropic measurement
    noise comes from the caller (`range_measurement_noise`) -- see the
    module docstring item 3 for why this replaced the fixed-gain filter."""

    _POS_VAR0 = 0.05 ** 2
    _VEL_VAR0 = 15.0 ** 2

    def __init__(self, q_accel_ms2: float, v_t_prior=(0.0, 0.0, 0.0)):
        self.q = float(q_accel_ms2)
        self._prior = np.array(v_t_prior, dtype=float)
        self.x: Optional[np.ndarray] = None
        self.P: Optional[np.ndarray] = None

    @property
    def initialized(self) -> bool:
        return self.x is not None

    @property
    def r_hat(self) -> Optional[np.ndarray]:
        return None if self.x is None else self.x[0:3]

    @property
    def v_t_hat(self) -> Optional[np.ndarray]:
        return None if self.x is None else self.x[3:6]

    def predict(self, dt: float, v_own: np.ndarray) -> None:
        if self.x is None:
            return
        f = np.eye(6)
        f[0:3, 3:6] = dt * np.eye(3)
        x = f @ self.x
        x[0:3] -= dt * np.asarray(v_own, dtype=float)
        q = np.zeros((6, 6))
        for i in range(3):
            q[i, i] = self.q * dt ** 3 / 3.0
            q[i, i + 3] = q[i + 3, i] = self.q * dt ** 2 / 2.0
            q[i + 3, i + 3] = self.q * dt
        self.x = x
        self.P = f @ self.P @ f.T + q

    def correct(self, r_meas: np.ndarray, r_noise: np.ndarray) -> None:
        r_meas = np.asarray(r_meas, dtype=float)
        if self.x is None:
            self.x = np.concatenate([r_meas, self._prior])
            self.P = np.zeros((6, 6))
            self.P[0:3, 0:3] = np.eye(3) * self._POS_VAR0
            self.P[3:6, 3:6] = np.eye(3) * self._VEL_VAR0
            return
        y = r_meas - self.x[0:3]
        s = self.P[0:3, 0:3] + r_noise
        k = self.P[:, 0:3] @ np.linalg.inv(s)
        self.x = self.x + k @ y
        self.P = self.P - k @ self.P[0:3, :]


class TagInterceptGuidance:
    """The 3-D predicted-intercept-point camera terminal. `step()` never
    raises: a fault is reported on `StepTelemetry.health`; returning
    `(None, tel)` means "no trustworthy command" -- the caller holds the
    last dash velocity, exactly as for SeekerGuidance."""

    def __init__(self, cfg: TagTerminalConfig, cam: CameraModel, tag_side_m: float,
                 gcfg: GuidanceConfig):
        self.cfg = cfg
        self.cam = cam
        self.tag_side_m = tag_side_m
        self.gcfg = gcfg
        self.kf = _RelStateKF(cfg.q_target_accel_ms2, cfg.v_t_prior_ned)
        self._half_vfov = (None if cam.height is None
                           else half_fov_from_intrinsics(cam.fy, cam.height))
        self._last_t: Optional[float] = None
        self._last_det_t: Optional[float] = None
        self._v_cmd_arr = np.zeros(3)          # last COMMANDED velocity (NED)
        self._yaw_cmd_deg: Optional[float] = None
        self._frozen: Optional[Tuple[float, float, float]] = None
        self._frozen_yaw: Optional[float] = None
        self._warned: set = set()
        self.n_track_broken = 0

    # Back-compat / convenience views onto the KF state (round-1 callers and
    # tests read `.r_hat`/`.v_t_hat` directly).
    @property
    def r_hat(self) -> Optional[np.ndarray]:
        return self.kf.r_hat

    @property
    def v_t_hat(self) -> Optional[np.ndarray]:
        return self.kf.v_t_hat

    def _emit_fault(self, tel: StepTelemetry, flag: str, msg: str, once: bool = False) -> None:
        tel.health.append(flag)
        if not once or flag not in self._warned:
            self._warned.add(flag)
            print(f"[tag_terminal] FAULT {flag}: {msg}")

    def step(self, det_box, own: OwnState, t: float) -> Tuple[Optional[Setpoint], StepTelemetry]:
        cfg, gcfg = self.cfg, self.gcfg
        dt = 0.05 if self._last_t is None else max(1e-3, t - self._last_t)
        self._last_t = t

        tel = StepTelemetry(t=t, detected=det_box is not None)
        tel.own_age_s = own.age_s

        own_ok, own_why = own_state_status(own, gcfg)
        tel.own_state_ok = own_ok
        if not own_ok:
            tel.stale = True
            self._emit_fault(
                tel, own_why,
                "own-state EKF unavailable/stale -- tag_terminal refusing to "
                "steer (see GuidanceConfig.require_own_attitude)", once=True)
            return None, tel

        if own.vel_ned is not None:
            v_own = np.array(own.vel_ned, dtype=float)
        else:
            v_own = self._v_cmd_arr.copy()
            self._emit_fault(
                tel, "own_vel_fallback",
                "OwnState.vel_ned is None -- falling back to the last "
                "COMMANDED velocity for the relative-velocity bookkeeping",
                once=True)

        self.kf.predict(dt, v_own)

        fresh = det_box is not None
        meas_xyz = None
        if fresh:
            bearing_h, range_meas, meas_xyz = measurement_from_box(
                det_box, gcfg, self.cam, self.tag_side_m)
            tel.bearing_deg = math.degrees(bearing_h)
            tel.range_meas_m = range_meas
            cam_to_tgt_ned = _optical_vec_to_ned(meas_xyz, own.quat, gcfg.mount_up_rad)
            cam_ofs_ned = _cam_offset_ned(own.quat, gcfg.cam_offset_body)
            r_meas = cam_to_tgt_ned + cam_ofs_ned    # target relative to OWN (CG), NED

            if self.kf.initialized and cfg.meas_latency_s:
                vrel_hat = self.kf.v_t_hat - v_own
                r_meas = r_meas + vrel_hat * cfg.meas_latency_s

            r_noise = range_measurement_noise(range_meas, r_meas, cfg, self.cam,
                                              self.tag_side_m)
            self.kf.correct(r_meas, r_noise)
            self._last_det_t = t

        if self.kf.r_hat is None:
            return None, tel     # no lock yet -- nothing to steer on

        r_hat, v_t_hat = self.kf.r_hat, self.kf.v_t_hat
        norm_r = float(np.linalg.norm(r_hat))
        tel.r_hat_m = norm_r
        vrel_hat = v_t_hat - v_own
        tel.lambda_deg = math.degrees(math.atan2(float(r_hat[1]), float(r_hat[0])))
        r_h = math.hypot(float(r_hat[0]), float(r_hat[1]))
        if r_h > 1e-6:
            tel.lambda_dot_deg_s = math.degrees(
                (r_hat[0] * vrel_hat[1] - r_hat[1] * vrel_hat[0]) / (r_h * r_h))
        tel.rdot_hat_m_s = (float(np.dot(r_hat, vrel_hat)) / norm_r
                           if norm_r > 1e-6 else None)
        tel.vc = cfg.v_engage_ms

        meas_age = None if self._last_det_t is None else (t - self._last_det_t)
        tel.meas_age_s = meas_age
        tel.stale = (not fresh) and self._last_det_t is not None

        broken = not (np.all(np.isfinite(r_hat)) and np.all(np.isfinite(v_t_hat)))
        if broken:
            tel.track_broken = True
            self.n_track_broken += 1
            self._emit_fault(tel, "track_broken",
                             "relative-position/velocity estimate is non-finite",
                             once=True)

        # eps_v (item 1 input): the FRESH ray this tick, else the estimate's
        # own r_hat rotated into the camera frame (round 2 spec).
        optical_for_eps = meas_xyz if fresh else _ned_vec_to_optical(
            r_hat, own.quat, gcfg.mount_up_rad)
        eps_v = (None if optical_for_eps is None
                else math.atan2(float(optical_for_eps[1]), float(optical_for_eps[2])))
        pitch_rad = _pitch_rad_from_quat(own.quat)

        if self._frozen is not None:
            tel.terminal_coast = True
            v_cmd, yaw_cmd = self._frozen, self._frozen_yaw
        elif norm_r < cfg.freeze_range_m and not broken:
            v_cmd, yaw_cmd = self._compute_command(own, v_own, dt, eps_v, pitch_rad)
            self._frozen, self._frozen_yaw = v_cmd, yaw_cmd
            tel.terminal_coast = True
        elif meas_age is not None and meas_age > cfg.coast_max_s:
            self._emit_fault(
                tel, "track_lost",
                f"no fresh detection for {meas_age:.2f} s (> coast_max_s "
                f"{cfg.coast_max_s:.2f} s) -- tag_terminal declining to command",
                once=True)
            return None, tel
        else:
            v_cmd, yaw_cmd = self._compute_command(own, v_own, dt, eps_v, pitch_rad)

        tel.v_perp = 0.0   # no separate lateral channel in this law (PIP, not pro-nav)
        sp = Setpoint(v_cmd[0], v_cmd[1], v_cmd[2], yaw_cmd)
        tel.setpoint = sp
        return sp, tel

    def _look_angle_bounds(self, eps_v, pitch_rad) -> Tuple[Optional[float], Optional[float]]:
        if eps_v is None or pitch_rad is None or self._half_vfov is None:
            return None, None
        cfg = self.cfg
        fov_cfg = FovHoldConfig(
            half_vfov_rad=self._half_vfov,
            edge_margin_rad=(1.0 - cfg.fov_margin_frac) * self._half_vfov)
        a_min, a_max = _look_angle_accel_bounds(eps_v, pitch_rad, fov_cfg)
        if a_max is not None:
            a_max = max(a_max, cfg.accel_floor_ms2)
        return a_min, a_max

    def _compute_command(self, own: OwnState, v_own: np.ndarray, dt: float,
                         eps_v: Optional[float], pitch_rad: Optional[float]
                         ) -> Tuple[Tuple[float, float, float], float]:
        cfg = self.cfg
        r, v_t = self.kf.r_hat, self.kf.v_t_hat
        a_min, a_max = self._look_angle_bounds(eps_v, pitch_rad)

        r_h = math.hypot(float(r[0]), float(r[1]))
        u_los_h = ((float(r[0]) / r_h, float(r[1]) / r_h) if r_h > 1e-6 else None)

        # --- item 4: speed schedule -- v_engage is a CEILING ----------------
        v_engage_eff = cfg.v_engage_ms
        if a_max is not None and u_los_h is not None:
            v_along_now = (self._v_cmd_arr[0] * u_los_h[0]
                          + self._v_cmd_arr[1] * u_los_h[1])
            v_engage_eff = min(cfg.v_engage_ms, max(
                0.0, v_along_now + a_max * cfg.speed_schedule_horizon_s))

        t_go = solve_intercept_t_go(r, v_t, v_engage_eff)
        if t_go is not None:
            v_raw = (r + v_t * t_go) / t_go
        else:
            direction = r + v_t * cfg.fallback_lead_s
            n = float(np.linalg.norm(direction))
            direction = direction / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])
            v_raw = v_engage_eff * direction

        v_down = _clamp(float(v_raw[2]), -cfg.v_vert_max_ms, cfg.v_vert_max_ms)
        if cfg.min_agl_m is not None and own.alt_m is not None:
            v_down, _floored = floor_v_down(
                v_down, own.alt_m, min_agl_m=cfg.min_agl_m,
                kp=cfg.alt_floor_kp, v_vert_max=cfg.v_vert_max_ms)

        # --- item 1+2: horizontal slew -- look-angle cap on the along-LOS
        #     component, then the general horizontal budget -----------------
        v_raw_h = np.array([float(v_raw[0]), float(v_raw[1])])
        prev_h = self._v_cmd_arr[0:2]
        dv_h = v_raw_h - prev_h
        if u_los_h is not None:
            u = np.array(u_los_h)
            dv_along = float(np.dot(dv_h, u))
            dv_perp = dv_h - dv_along * u
            step_max = (a_max if a_max is not None
                       else cfg.cmd_accel_horiz_max_ms2) * dt
            step_min = (a_min if a_min is not None
                       else -cfg.cmd_accel_horiz_max_ms2) * dt
            dv_along = _clamp(dv_along, step_min, step_max)
            dv_h = dv_along * u + dv_perp
        n = float(np.linalg.norm(dv_h))
        max_h_step = cfg.cmd_accel_horiz_max_ms2 * dt
        if n > max_h_step and n > 1e-12:
            dv_h = dv_h * (max_h_step / n)
        v_cmd_h = prev_h + dv_h

        # --- item 2: vertical slew, its OWN budget --------------------------
        max_v_step = cfg.cmd_accel_vert_max_ms2 * dt
        dv_v = _clamp(v_down - self._v_cmd_arr[2], -max_v_step, max_v_step)
        v_cmd_v = self._v_cmd_arr[2] + dv_v

        v_cmd = np.array([v_cmd_h[0], v_cmd_h[1], v_cmd_v])
        self._v_cmd_arr = v_cmd

        yaw_target = math.degrees(math.atan2(float(r[1]), float(r[0])))
        if self._yaw_cmd_deg is None:
            yaw_cmd = yaw_target
        else:
            delta = _wrap_deg(yaw_target - self._yaw_cmd_deg)
            max_yaw_step = cfg.yaw_rate_max_deg_s * dt
            delta = _clamp(delta, -max_yaw_step, max_yaw_step)
            yaw_cmd = _wrap_deg(self._yaw_cmd_deg + delta)
        self._yaw_cmd_deg = yaw_cmd
        return (float(v_cmd[0]), float(v_cmd[1]), float(v_cmd[2])), yaw_cmd
