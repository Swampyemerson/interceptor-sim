"""TargetModel implementations -- pure functions of time (ADR-0102 isim).

Each target is stateless once constructed: `state(t)` derives position and
velocity analytically from `t`, so results are exactly reproducible and there
is nothing to `reset()`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from isim.types import TargetState, Vec3


def _as_vec3(v) -> Vec3:
    return np.asarray(v, dtype=np.float64).reshape(3)


@dataclass
class ConstantVelocityTarget:
    """Straight-line target: pos(t) = pos0 + vel*t. A vertical component in
    `vel_ned` makes this a climbing/descending target too -- no separate
    class needed."""
    pos0_ned: Vec3
    vel_ned: Vec3

    def __post_init__(self) -> None:
        self.pos0_ned = _as_vec3(self.pos0_ned)
        self.vel_ned = _as_vec3(self.vel_ned)

    def state(self, t: float) -> TargetState:
        pos = self.pos0_ned + self.vel_ned * t
        return TargetState(t=t, pos_ned=pos, vel_ned=self.vel_ned.copy())


@dataclass
class HoverTarget:
    """Stationary target."""
    pos_ned: Vec3

    def __post_init__(self) -> None:
        self.pos_ned = _as_vec3(self.pos_ned)
        self._zero = np.zeros(3)

    def state(self, t: float) -> TargetState:
        return TargetState(t=t, pos_ned=self.pos_ned.copy(), vel_ned=self._zero.copy())


@dataclass
class WeaveTarget:
    """Cruise velocity `vel_ned` plus a sinusoidal lateral (cross-track)
    offset of amplitude `amp_m` and period `period_s`, perpendicular to the
    horizontal component of `vel_ned`. Velocity is the exact analytic
    derivative of position, so it is consistent with the trajectory to
    machine precision (no finite-difference drift)."""
    pos0_ned: Vec3
    vel_ned: Vec3
    amp_m: float
    period_s: float

    def __post_init__(self) -> None:
        self.pos0_ned = _as_vec3(self.pos0_ned)
        self.vel_ned = _as_vec3(self.vel_ned)
        self._omega = 2.0 * math.pi / self.period_s
        horiz = self.vel_ned[:2]
        horiz_speed = float(np.linalg.norm(horiz))
        if horiz_speed > 1e-9:
            # rotate (vN, vE) by +90 deg in the horizontal plane
            perp_horiz = np.array([-horiz[1], horiz[0]]) / horiz_speed
        else:
            perp_horiz = np.array([0.0, 1.0])  # fallback: east
        self._perp = np.array([perp_horiz[0], perp_horiz[1], 0.0])

    def state(self, t: float) -> TargetState:
        wt = self._omega * t
        pos = self.pos0_ned + self.vel_ned * t + (self.amp_m * math.sin(wt)) * self._perp
        vel = self.vel_ned + (self.amp_m * self._omega * math.cos(wt)) * self._perp
        return TargetState(t=t, pos_ned=pos, vel_ned=vel)
