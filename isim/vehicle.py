"""Point-mass quadcopter model under velocity-setpoint control (mimics PX4
OFFBOARD velocity / MPC behaviour). Fitted later against logged x500 flights;
this module only needs the right STRUCTURE, not tuned numbers.

Pipeline: setpoint slew -> velocity-loop P(+I) -> desired specific-thrust
vector (accel demand + gravity cancel) -> desired tilt + thrust magnitude
(both clamped) -> actual tilt/thrust LAG the desired ones (rate limit +
first-order lag) -> acceleration = thrust + gravity - drag(v - wind) ->
Euler-integrate velocity/position. Yaw is independent: rate-limited
first-order tracking of the commanded yaw.

Why hard horizontal accel bleeds into altitude: tilting thrust over to
accelerate horizontally shrinks its vertical component; if thrust magnitude
is saturated or still lagging up to its ceiling there is not enough vertical
thrust to hold altitude -- the vehicle sags or climbs depending on the
relative timing of the tilt and thrust lags. That is a *consequence* of the
physics below, not a special-cased rule.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from isim.types import VehicleState, VelCmd

G = 9.80665  # m/s^2, magnitude, NED so gravity accel is +G in the down axis


def _clip_norm(v: np.ndarray, max_norm: float) -> Tuple[np.ndarray, bool]:
    """Scale v down to max_norm if it exceeds it; report whether it did."""
    n = float(np.linalg.norm(v))
    if max_norm > 0.0 and n > max_norm:
        return v * (max_norm / n), True
    return v, False


def _dir_to_tilt(unit_dir: np.ndarray) -> np.ndarray:
    """Encode a unit thrust direction (NED) as a 2-vector (tilt_n, tilt_e) =
    (horizontal unit direction) * sin(angle off vertical). This sine-scaled
    representation has norm in [0, 1] and is what gets rate-limited/lagged
    below, so the lag state composes linearly instead of needing its own
    trig each step."""
    sin_mag = math.sqrt(max(0.0, 1.0 - unit_dir[2] ** 2))
    h_norm = float(np.linalg.norm(unit_dir[:2]))
    horiz_unit = unit_dir[:2] / h_norm if h_norm > 1e-9 else np.zeros(2)
    return horiz_unit * sin_mag


def _tilt_to_dir(tilt: np.ndarray) -> np.ndarray:
    """Inverse of _dir_to_tilt: decode a sine-tilt vector back to a unit
    thrust direction (NED down-component negative == pointing up)."""
    sin_mag = float(np.linalg.norm(tilt))
    cos_mag = math.sqrt(max(0.0, 1.0 - sin_mag**2))
    return np.array([tilt[0], tilt[1], -cos_mag])


@dataclass
class VehicleParams:
    """Every tunable of the model. Units are in each field's comment. These
    are plausible x500/PX4-default starting points -- fitting happens in
    fit_vehicle.py, not here. All thrust/accel quantities are SPECIFIC
    thrust (force / mass, m/s^2) so vehicle mass never enters the equations
    directly (~2 kg x500-with-battery is what these defaults assume)."""

    # --- velocity loop (PX4 MPC_XY_VEL_P / MPC_Z_VEL_P analogue) ---
    kp_vel_horiz: float = 1.8  # (m/s^2) per (m/s) horizontal velocity error
    ki_vel_horiz: float = 0.4  # (m/s^2) per (m*s) integrated horizontal error
    ki_vel_horiz_limit: float = 3.0  # m/s^2, horizontal integrator clamp
    kp_vel_vert: float = 4.0  # (m/s^2) per (m/s) vertical (down) velocity error
    ki_vel_vert: float = 1.5  # (m/s^2) per (m*s) integrated vertical error
    ki_vel_vert_limit: float = 3.0  # m/s^2, vertical integrator clamp

    # --- setpoint slew, like PX4 MPC_ACC_HOR / MPC_ACC_UP_MAX limiting the
    # *setpoint* itself before it ever reaches the velocity loop ---
    max_setpoint_accel_horiz: float = 6.0  # m/s^2, slew limit on cmd v_n/v_e
    max_setpoint_accel_vert: float = 4.0  # m/s^2, slew limit on cmd v_d

    # --- horizontal accel demand clamp (post velocity-loop, pre tilt-solve) ---
    max_accel_horiz: float = 8.0  # m/s^2, commanded horizontal accel ceiling

    # --- attitude (tilt) response: desired tilt vector lags the commanded one
    # as a rate-limited first-order lag (a cheap stand-in for the attitude
    # controller's closed loop) ---
    max_tilt_deg: float = 35.0  # deg, hard tilt-magnitude ceiling
    max_tilt_rate_deg_s: float = 220.0  # deg/s, physical tilt-rate ceiling
    tilt_time_constant_s: float = 0.12  # s, first-order lag time constant

    # --- thrust (collective) response ---
    thrust_max: float = 19.6  # m/s^2, specific thrust ceiling (~2 g)
    thrust_min: float = 1.0  # m/s^2, specific thrust floor (near-zero throttle)
    thrust_time_constant_s: float = 0.08  # s, first-order lag on thrust magnitude

    # --- drag (opposes velocity relative to wind) ---
    drag_linear_horiz: float = 0.25  # 1/s, horizontal linear drag coefficient
    drag_linear_vert: float = 0.35  # 1/s, vertical linear drag coefficient
    drag_quad_horiz: float = 0.0  # (1/m), horizontal quadratic drag coefficient
    drag_quad_vert: float = 0.0  # (1/m), vertical quadratic drag coefficient

    # --- thrust-azimuth error (reference-stack artefact; see step()) ---
    thrust_az_bias_deg: float = 0.0  # deg, + = thrust rotated clockwise seen from above

    # --- yaw ---
    yaw_rate_limit_deg_s: float = 120.0  # deg/s
    yaw_time_constant_s: float = 0.25  # s, first-order lag towards yaw rate cmd

    # --- command path ---
    latency_s: float = 0.02  # s, pure delay from cmd issue to cmd applied
    # implemented as an integer-step delay line at whatever dt step() is called with

    # --- wind ---
    wind_ned: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # m/s, constant wind
    gust_std: float = 0.0  # m/s, per-axis gust std (Gaussian, iid per step)


class _DelayLine:
    """Fixed-length FIFO implementing pure latency as an integer number of
    simulation steps (sized from latency_s / dt on first use). Applying a
    command N steps after it was issued: seed the queue with N copies of the
    first-ever command (so the vehicle holds that command, unchanged, for the
    first N steps -- there is no earlier command to have been "in transit"),
    then each step returns the oldest queued command and pushes the new one."""

    def __init__(self, n_delay: int, first_cmd):
        self.n_delay = n_delay
        if n_delay <= 0:
            self.q = None
        else:
            self.q = deque([first_cmd] * n_delay, maxlen=n_delay)

    def push_pop(self, cmd):
        if self.q is None:
            return cmd
        applied = self.q[0]
        self.q.append(cmd)  # deque with maxlen auto-drops the oldest
        return applied


class QuadVelocityModel:
    """Point-mass quad tracking NED velocity + yaw setpoints, PX4-offboard
    style. Implements the isim.types.VehicleModel protocol."""

    def __init__(self, params: Optional[VehicleParams] = None):
        self.p = params if params is not None else VehicleParams()
        self._rng: Optional[np.random.Generator] = None
        self._state: Optional[VehicleState] = None
        self._reset_internal()

    def _reset_internal(self) -> None:
        p = self.p
        self._int_horiz = np.zeros(2)  # (n, e) velocity-error integrator
        self._int_vert = 0.0  # d-axis velocity-error integrator
        self._tilt = np.zeros(2)  # actual sine-tilt vector (tilt_n, tilt_e),
        # dimensionless, see _dir_to_tilt/_tilt_to_dir for the encoding
        self._thrust_mag = G  # actual specific thrust magnitude, starts at hover
        self._yaw = 0.0  # actual yaw (rad)
        self._delay: Optional[_DelayLine] = None
        if hasattr(self, "_slewed_v"):
            del self._slewed_v

    def reset(self, state: VehicleState, rng: np.random.Generator) -> None:
        self._rng = rng
        self._state = VehicleState(
            t=state.t,
            pos_ned=np.array(state.pos_ned, dtype=float).copy(),
            vel_ned=np.array(state.vel_ned, dtype=float).copy(),
            quat_wxyz=tuple(state.quat_wxyz),
            yaw_rad=state.yaw_rad,
            saturated=False,
        )
        self._reset_internal()
        self._yaw = state.yaw_rad
        self._delay = None  # sized lazily on first step() once dt is known

    def step(self, cmd: VelCmd, dt: float) -> VehicleState:
        p = self.p
        assert self._state is not None, "call reset() before step()"

        # --- command latency: integer-step delay line, sized from dt ---
        n_delay = max(0, round(p.latency_s / dt))
        if self._delay is None:
            self._delay = _DelayLine(n_delay, cmd)
        applied_cmd = self._delay.push_pop(cmd)

        cmd_v = np.array([applied_cmd.v_north, applied_cmd.v_east, applied_cmd.v_down])
        cmd_yaw = math.radians(applied_cmd.yaw_deg)

        saturated = False

        # --- setpoint slew (rate-limit the setpoint itself, PX4 MPC_ACC_*) ---
        if not hasattr(self, "_slewed_v"):
            self._slewed_v = self._state.vel_ned.copy()
        d = cmd_v - self._slewed_v
        d[:2], sat_h = _clip_norm(d[:2], p.max_setpoint_accel_horiz * dt)
        max_step_v = p.max_setpoint_accel_vert * dt
        sat_v = abs(d[2]) > max_step_v
        if sat_v:
            d[2] = math.copysign(max_step_v, d[2])
        saturated = saturated or sat_h or sat_v
        self._slewed_v = self._slewed_v + d
        vel_sp = self._slewed_v

        vel = self._state.vel_ned
        err = vel_sp - vel

        # --- velocity loop: P + clamped I, separate horizontal/vertical ---
        horiz_int_bound = p.ki_vel_horiz_limit / max(p.ki_vel_horiz, 1e-9)
        self._int_horiz = np.clip(
            self._int_horiz + err[:2] * dt, -horiz_int_bound, horiz_int_bound
        )
        accel_cmd_horiz = p.kp_vel_horiz * err[:2] + p.ki_vel_horiz * self._int_horiz

        vert_int_bound = p.ki_vel_vert_limit / max(p.ki_vel_vert, 1e-9)
        self._int_vert = float(
            np.clip(self._int_vert + err[2] * dt, -vert_int_bound, vert_int_bound)
        )
        accel_cmd_vert = p.kp_vel_vert * err[2] + p.ki_vel_vert * self._int_vert

        accel_cmd_horiz, sat = _clip_norm(accel_cmd_horiz, p.max_accel_horiz)
        saturated = saturated or sat

        # --- desired thrust-acceleration vector: net_accel = thrust_accel + g,
        # so thrust_accel = accel_cmd - gravity_vec. NED gravity_vec = [0,0,+G],
        # so hover (accel_cmd=0) needs thrust_accel = [0,0,-G]: magnitude G
        # pointing up (negative-down), exactly cancelling gravity. ---
        desired_thrust_ned = np.array(
            [accel_cmd_horiz[0], accel_cmd_horiz[1], accel_cmd_vert - G]
        )
        desired_mag = float(np.linalg.norm(desired_thrust_ned))
        thrust_dir_desired = (
            desired_thrust_ned / desired_mag if desired_mag > 1e-9 else np.array([0.0, 0.0, -1.0])
        )

        max_sin_tilt = math.sin(math.radians(p.max_tilt_deg))
        desired_tilt, sat = _clip_norm(_dir_to_tilt(thrust_dir_desired), max_sin_tilt)
        saturated = saturated or sat

        desired_thrust_mag = min(p.thrust_max, max(p.thrust_min, desired_mag))
        saturated = saturated or not (p.thrust_min <= desired_mag <= p.thrust_max)

        # --- actual tilt lags desired: rate-limited first-order lag, in the
        # sine-tilt representation (see _dir_to_tilt) ---
        tau_t = max(p.tilt_time_constant_s, 1e-6)
        tilt_rate = (desired_tilt - self._tilt) / tau_t
        tilt_rate, sat_rate = _clip_norm(tilt_rate, math.radians(p.max_tilt_rate_deg_s))
        self._tilt = self._tilt + tilt_rate * dt
        self._tilt, sat_mag = _clip_norm(self._tilt, max_sin_tilt)
        saturated = saturated or sat_rate or sat_mag

        # --- actual thrust magnitude lags desired: first-order lag ---
        tau_th = max(p.thrust_time_constant_s, 1e-6)
        self._thrust_mag += (desired_thrust_mag - self._thrust_mag) * (dt / tau_th)
        self._thrust_mag = min(p.thrust_max, max(p.thrust_min, self._thrust_mag))

        thrust_dir = _tilt_to_dir(self._tilt)  # actual thrust direction, NED
        # --- thrust-azimuth error (default 0 = off). The PX4/Gazebo reference
        # slides 0.4-0.8 m sideways during a hard acceleration and then holds the
        # offset (measured 2026-09-17, 16 flights, same side both crossing
        # directions, larger while yaw is still slewing). Modelled as the tilt
        # being realised rotated about vertical by a constant angle; the velocity loop
        # then fights it, which is what makes the offset saturate. ---
        if p.thrust_az_bias_deg != 0.0:
            eps = math.radians(p.thrust_az_bias_deg)
            ce, se = math.cos(eps), math.sin(eps)
            thrust_dir = np.array([ce * thrust_dir[0] - se * thrust_dir[1],
                                   se * thrust_dir[0] + ce * thrust_dir[1], thrust_dir[2]])
        thrust_accel_ned = thrust_dir * self._thrust_mag

        # --- yaw: rate-limited first-order tracking ---
        yaw_err = _wrap_pi(cmd_yaw - self._yaw)
        max_yaw_rate = math.radians(p.yaw_rate_limit_deg_s)
        tau_y = max(p.yaw_time_constant_s, 1e-6)
        yaw_rate = yaw_err / tau_y
        if abs(yaw_rate) > max_yaw_rate:
            yaw_rate = math.copysign(max_yaw_rate, yaw_rate)
            saturated = True
        self._yaw = _wrap_pi(self._yaw + yaw_rate * dt)

        # --- wind + drag ---
        wind = np.array(p.wind_ned, dtype=float)
        if p.gust_std > 0.0 and self._rng is not None:
            wind = wind + self._rng.normal(0.0, p.gust_std, size=3)
        rel_v = vel - wind
        lin_coef = np.array([p.drag_linear_horiz, p.drag_linear_horiz, p.drag_linear_vert])
        drag = lin_coef * rel_v
        if p.drag_quad_horiz > 0.0 or p.drag_quad_vert > 0.0:
            quad_coef = np.array([p.drag_quad_horiz, p.drag_quad_horiz, p.drag_quad_vert])
            drag = drag + quad_coef * rel_v * np.abs(rel_v)

        accel = thrust_accel_ned + np.array([0.0, 0.0, G]) - drag

        # --- semi-implicit Euler integration ---
        new_vel = vel + accel * dt
        new_pos = self._state.pos_ned + new_vel * dt

        quat = _quat_from_thrust_dir_yaw(thrust_dir, self._yaw)

        new_state = VehicleState(
            t=self._state.t + dt,
            pos_ned=new_pos,
            vel_ned=new_vel,
            quat_wxyz=quat,
            yaw_rad=self._yaw,
            saturated=saturated,
        )
        self._state = new_state
        return new_state


def _wrap_pi(a: float) -> float:
    """Wrap an angle (rad) into (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _quat_from_thrust_dir_yaw(
    thrust_dir_ned: np.ndarray, yaw_rad: float
) -> Tuple[float, float, float, float]:
    """Body->NED quaternion whose body -z axis equals thrust_dir_ned and whose
    heading about that axis matches yaw_rad: compose a BODY-frame yaw
    rotation (about body z, which leaves body -z itself unmoved) with a
    tilt-only rotation (shortest rotation taking level "-z" = NED [0,0,-1]
    to thrust_dir_ned) applied after -- so yaw only sets heading, never
    perturbs the thrust direction the tilt already solved for."""
    level_up = np.array([0.0, 0.0, -1.0])
    dot = float(np.clip(np.dot(level_up, thrust_dir_ned), -1.0, 1.0))
    angle = math.acos(dot)
    if angle < 1e-9:
        tilt_quat = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        axis = np.cross(level_up, thrust_dir_ned)
        axis_norm = float(np.linalg.norm(axis))
        # antiparallel (angle==pi) has no unique axis; any perpendicular works
        axis = axis / axis_norm if axis_norm > 1e-9 else np.array([1.0, 0.0, 0.0])
        half = angle / 2.0
        s = math.sin(half)
        tilt_quat = np.array([math.cos(half), axis[0] * s, axis[1] * s, axis[2] * s])

    half_y = yaw_rad / 2.0
    yaw_quat = np.array([math.cos(half_y), 0.0, 0.0, math.sin(half_y)])
    w = _quat_mul(tilt_quat, yaw_quat)  # yaw is a body-frame rotation, applied first
    w = w / math.sqrt(float(np.dot(w, w)))
    return (float(w[0]), float(w[1]), float(w[2]), float(w[3]))


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product, (w, x, y, z) convention."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )
