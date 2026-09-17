"""Own-state noise as seen by GUIDANCE only (isim/specs/pursuit_hardening_v5.md
#1-4). `OwnStateNoise` wraps a `Guidance`, perturbing the `VehicleState` and
`Detection.t_capture` handed to `step()` with attitude/velocity/position
error and frame-timestamp error a real vehicle's own-state EKF and camera
driver would carry. The ENGINE always keeps the TRUE `VehicleState` for
physics and for `EngagementResult`'s trace (scoring) -- this module only
changes what is handed FORWARD to the wrapped guidance; nothing it does is
visible to `isim.engine.run_engagement` or to the seeker.

HONESTY. This is not a new violation of the honesty boundary; it is what the
boundary was always describing. Guidance was never meant to see ground
truth for its own state either -- prior rounds simply handed it the exact
truth as a stand-in for "the own-state EKF" because no noise model existed
yet. This module is that model: a plausible EKF/IMU/camera-driver estimate
of the vehicle's own state, in place of the truth `isim.engine` still keeps
for scoring. See `isim.scenario.build()` for where this gets wired in
(concept="pursuit" only -- "flyby"'s `RealFlightGuidance` has its own,
separate `Scatter`-driven pre-flight-error story and is untouched)."""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from isim.types import Detection, Guidance, VehicleState, VelCmd

_EPS = 1e-9


@dataclass
class OwnStateNoiseConfig:
    """Every sigma here is a bench-PLAUSIBLE ESTIMATE (no logged bench run
    backs any of them yet -- same honesty class as `isim.scenario.Scatter`,
    whose fields this is built from; see `isim.scenario.build()`), and
    every one is independently zeroable (0.0 = that error source off)."""

    # --- #1: own-attitude error, guidance-visible only ---
    attitude_bias_rp_sigma_deg: float = 1.0     # deg, slowly-varying roll/pitch bias (OU)
    attitude_bias_yaw_sigma_deg: float = 3.0    # deg, slowly-varying yaw bias (OU)
    attitude_white_sigma_deg: float = 0.3       # deg, white noise, per axis, per tick
    attitude_bias_tau_s: float = 20.0           # s, OU correlation time (both biases above)

    # --- #2: own-velocity error, guidance-visible only ---
    vel_bias_sigma_ms: float = 0.15             # m/s per axis, ONE draw, held for the run
    vel_white_sigma_ms: float = 0.1             # m/s per axis, white noise, per tick

    # --- #3: own-position error, guidance-visible only ---
    pos_randomwalk_sigma_ms_sqrt_s: float = 0.05  # m/sqrt(s) per axis, random-walk RATE

    # --- #4: frame timestamp error, guidance-visible only (true t_capture unchanged) ---
    frame_ts_bias_max_s: float = 0.020          # s, ONE draw uniform +/- this, held for the run
    frame_ts_jitter_sigma_s: float = 0.005       # s, 1-sigma, per decoded frame


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product, (w, x, y, z) convention -- same formula as
    `isim.vehicle._quat_mul`, duplicated locally (a tiny pure function; not
    worth importing a private helper across modules for)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def _small_angle_quat(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    """First-order (small-angle) quaternion for a body-frame XYZ rotation --
    exact to O(angle^2), which is negligible at the few-degree angles this
    module ever perturbs by. `(w,x,y,z) = normalize(1, roll/2, pitch/2, yaw/2)`."""
    q = np.array([1.0, roll_rad * 0.5, pitch_rad * 0.5, yaw_rad * 0.5])
    n = float(np.linalg.norm(q))
    return q / n if n > _EPS else np.array([1.0, 0.0, 0.0, 0.0])


class OwnStateNoise:
    """isim `Guidance`: wraps `inner`, perturbing what `inner.step()` is
    handed. Forwards every OTHER attribute access to `inner` via
    `__getattr__` (so `isim.mc`'s `state_log`/`last_decision`/`debug`
    reads -- and a further wrapper like `isim.mc._DebugRecorder` -- see
    straight through to the real guidance, unaffected by this layer)."""

    def __init__(self, inner: Guidance, cfg: OwnStateNoiseConfig,
                 rng: np.random.Generator) -> None:
        self.inner = inner
        self.cfg = cfg
        self._rng = rng
        self.reset()

    def reset(self) -> None:
        cfg = self.cfg
        self._att_bias_rp = np.zeros(2)      # (roll, pitch) OU state, deg
        self._att_bias_yaw = 0.0             # OU state, deg
        self._vel_bias = self._rng.normal(0.0, cfg.vel_bias_sigma_ms, size=3)
        self._pos_walk = np.zeros(3)
        self._ts_bias_s = float(self._rng.uniform(-cfg.frame_ts_bias_max_s,
                                                   cfg.frame_ts_bias_max_s))
        self._last_t: Optional[float] = None
        self.inner.reset()

    def step(self, t: float, own: VehicleState, det: Optional[Detection]) -> VelCmd:
        cfg = self.cfg
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t

        # #1: attitude -- OU (slowly-varying) bias + white noise, roll/pitch/yaw.
        if dt > 0.0 and cfg.attitude_bias_tau_s > 0.0:
            decay = math.exp(-dt / cfg.attitude_bias_tau_s)
            diffuse = math.sqrt(max(0.0, 1.0 - decay * decay))
            self._att_bias_rp = (self._att_bias_rp * decay
                                + self._rng.normal(0.0, 1.0, size=2)
                                * cfg.attitude_bias_rp_sigma_deg * diffuse)
            self._att_bias_yaw = (self._att_bias_yaw * decay
                                  + float(self._rng.normal(0.0, 1.0))
                                  * cfg.attitude_bias_yaw_sigma_deg * diffuse)
        white = self._rng.normal(0.0, cfg.attitude_white_sigma_deg, size=3)
        roll_err = math.radians(self._att_bias_rp[0] + white[0])
        pitch_err = math.radians(self._att_bias_rp[1] + white[1])
        yaw_err = math.radians(self._att_bias_yaw + white[2])

        q_err = _small_angle_quat(roll_err, pitch_err, yaw_err)
        q_pert = _quat_mul(np.asarray(own.quat_wxyz, dtype=np.float64), q_err)
        q_pert = q_pert / max(float(np.linalg.norm(q_pert)), _EPS)

        # #2: velocity -- per-run constant bias + white noise.
        vel_white = self._rng.normal(0.0, cfg.vel_white_sigma_ms, size=3)
        vel_pert = np.asarray(own.vel_ned, dtype=np.float64) + self._vel_bias + vel_white

        # #3: position -- random walk (rate sigma m/sqrt(s)); "should mostly
        # cancel because estimate and control share the frame" -- see the
        # task's final report for the measured check of that claim.
        if dt > 0.0:
            self._pos_walk = self._pos_walk + self._rng.normal(
                0.0, cfg.pos_randomwalk_sigma_ms_sqrt_s * math.sqrt(dt), size=3)
        pos_pert = np.asarray(own.pos_ned, dtype=np.float64) + self._pos_walk

        own_pert = VehicleState(
            t=own.t, pos_ned=pos_pert, vel_ned=vel_pert, quat_wxyz=tuple(q_pert),
            yaw_rad=own.yaw_rad + yaw_err, saturated=own.saturated)

        # #4: frame timestamp -- per-run constant bias + per-frame jitter,
        # on the COPY handed to `inner` only; `det.t_capture` itself (and
        # everything else about `det`) is the TRUE value from the seeker.
        det_pert = det
        if det is not None:
            jitter = float(self._rng.normal(0.0, cfg.frame_ts_jitter_sigma_s))
            det_pert = dataclasses.replace(det, t_capture=det.t_capture + self._ts_bias_s + jitter)

        return self.inner.step(t, own_pert, det_pert)

    def __getattr__(self, name: str):
        return getattr(self.inner, name)
