"""Target attitude derived from the trajectory (isim/specs/tag_realism_v1.md A2).

A real multirotor banks to turn and pitches nose-down to hold a cruise speed,
and the AprilTag is bolted to it. `AttitudeTarget` wraps any TargetModel and
adds the attitude a quad would need to fly that trajectory -- DERIVED, not
integrated, so the wrapped target stays a pure function of `t` (the property
isim's reproducibility rests on): every finite difference below re-evaluates
`inner.state()`, which is itself pure, and nothing is cached between calls.

Physics, at time t:
  a(t)   = (vel(t+h) - vel(t-h)) / 2h         h = accel_stencil_s (also the low-pass)
  d(v)   = g*tan(drag_tilt_at_9ms_deg) * (v/9)^2, opposite the horizontal velocity
  f      = a - g_ned - d_vec                   specific force the rotors must make
  z_body = -f/|f|                              (FRD: body z is DOWN, thrust is -z)
  yaw    = heading of the horizontal velocity when > 0.5 m/s, else north
  R      = R_tilt(z_body) @ Rz(yaw)            tilt about the horizontal axis _|_ f
  tilt capped at max_tilt_deg; ang_vel_body from a central difference of the
  derived quaternion (isim.seeker._omega_body).

SIGN NOTE: the spec text writes `f = a - g_ned + a_drag_vec` with `a_drag_vec`
"directed opposite the horizontal velocity". Taken literally that tilts the
thrust BACKWARD (nose-up) at cruise, which contradicts the spec's own physics
anchor ("constant-velocity 9 m/s target -> nose-down pitch ~= drag_tilt").
The rotors must CANCEL drag, so thrust leans INTO the velocity: f = a - g - d
with d the drag acceleration (opposite v). The anchor is what is implemented
and pinned in the tests.

HONESTY: every field of TargetAttitudeParams is an `estimate` (see UNMEASURED
below) -- physics-plausible, not fitted to a flown target.

Frames: world NED (+z down, gravity +9.81 on z), body FRD, quaternions
(w, x, y, z) body -> NED.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

from isim.seeker import _omega_body
from isim.types import TargetModel, TargetState

G_MS2 = 9.81
_G_NED = np.array([0.0, 0.0, G_MS2])
_EPS = 1e-12
# Horizontal speed (m/s) below which the heading is undefined and yaw falls
# back to north (a hovering target: tag facing dominates, yaw is cosmetic).
_YAW_MIN_SPEED_MS = 0.5
# The speed the drag-tilt knob is quoted at (m/s) -- the project's >= 9 m/s
# target spec.
_DRAG_REF_SPEED_MS = 9.0

# Every TargetAttitudeParams field is unmeasured (no flown target logged its
# attitude); parallel to isim.seeker.UNMEASURED.
UNMEASURED = (
    "accel_stencil_s", "max_tilt_deg", "drag_tilt_at_9ms_deg", "rate_stencil_s",
)


@dataclass
class TargetAttitudeParams:
    """Knobs of the derived attitude. All `estimate`."""
    accel_stencil_s: float = 0.10       # half-width h of the accel central difference;
                                        # doubles as the low-pass (a real quad cannot
                                        # tilt instantly). estimate.
    max_tilt_deg: float = 35.0          # tilt cap -- SpeedChangeTarget has a true
                                        # velocity step -> accel pulse. estimate.
    drag_tilt_at_9ms_deg: float = 12.0  # nose-down pitch needed to hold 9 m/s cruise
                                        # (quadratic drag through this point). estimate.
    rate_stencil_s: float = 0.02        # half-width of the angular-rate difference. estimate.


# ------------------------------------------------------------ quaternion bits

def _qmul(a, b) -> Tuple[float, float, float, float]:
    """Hamilton product a*b, (w,x,y,z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def euler_rpy_deg(q) -> Tuple[float, float, float]:
    """(roll, pitch, yaw) in degrees, ZYX aerospace convention, from a
    (w,x,y,z) body->NED quaternion. Positive pitch = nose UP, positive roll =
    right wing DOWN. Diagnostics/tests only."""
    w, x, y, z = (float(c) for c in q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def tilt_deg(q) -> float:
    """Angle between body z (down) and world down, degrees."""
    w, x, y, z = (float(c) for c in q)
    r33 = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, r33))))


# --------------------------------------------------------------- the wrapper

@dataclass
class AttitudeTarget:
    """TargetModel wrapper: `inner`'s position/velocity unchanged, plus the
    derived body->NED quaternion and body angular rate (see module docstring).
    Stateless: `state(t)` is a pure function of `t`."""

    inner: TargetModel
    prm: TargetAttitudeParams = field(default_factory=TargetAttitudeParams)

    def state(self, t: float) -> TargetState:
        s = self.inner.state(t)
        q = self._quat_at(t, s.vel_ned)
        r = self.prm.rate_stencil_s
        q_m = self._quat_at(t - r, self.inner.state(t - r).vel_ned)
        q_p = self._quat_at(t + r, self.inner.state(t + r).vel_ned)
        w_b = _omega_body(q_m, q_p, 2.0 * r) if r > 0.0 else np.zeros(3)
        return TargetState(t=s.t, pos_ned=s.pos_ned, vel_ned=s.vel_ned,
                           quat_wxyz=q, ang_vel_body=w_b)

    # ---- derivation ------------------------------------------------------
    def _quat_at(self, t: float, vel: np.ndarray) -> Tuple[float, float, float, float]:
        prm = self.prm
        h = prm.accel_stencil_s
        vel = np.asarray(vel, float)
        if h > 0.0:
            a = (np.asarray(self.inner.state(t + h).vel_ned, float)
                 - np.asarray(self.inner.state(t - h).vel_ned, float)) / (2.0 * h)
        else:
            a = np.zeros(3)

        v_h = np.array([vel[0], vel[1], 0.0])
        speed_h = float(np.linalg.norm(v_h))
        f = a - _G_NED
        if speed_h > _EPS:
            k = G_MS2 * math.tan(math.radians(prm.drag_tilt_at_9ms_deg))
            d_mag = k * (speed_h / _DRAG_REF_SPEED_MS) ** 2
            # Drag accel points opposite v_h; the rotors cancel it, so the
            # required specific force leans INTO the velocity (module note).
            f = f + d_mag * (v_h / speed_h)

        yaw = math.atan2(vel[1], vel[0]) if speed_h > _YAW_MIN_SPEED_MS else 0.0
        q_yaw = (math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw))

        f_norm = float(np.linalg.norm(f))
        if f_norm < _EPS:
            return q_yaw                  # free fall: no thrust direction -- hold level
        z_b = -f / f_norm
        # Minimal rotation taking world down (0,0,1) onto z_b: axis = down x z_b
        # (horizontal, _|_ f), angle = the tilt.
        axis = np.array([-z_b[1], z_b[0], 0.0])
        s_ang = float(np.linalg.norm(axis))
        tilt = math.atan2(s_ang, float(z_b[2]))
        if s_ang < 1e-12:
            if z_b[2] > 0.0:
                return q_yaw              # exactly level
            axis, s_ang = np.array([1.0, 0.0, 0.0]), 1.0   # inverted: pick an axis
        tilt = min(tilt, math.radians(prm.max_tilt_deg))
        axis = axis / s_ang
        sh = math.sin(0.5 * tilt)
        q_tilt = (math.cos(0.5 * tilt), float(axis[0]) * sh, float(axis[1]) * sh, 0.0)
        return _qmul(q_tilt, q_yaw)
