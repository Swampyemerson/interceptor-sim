"""Shared value types and the four plug-in protocols. OWNED BY THE HEAD SESSION:
every other isim module imports from here and none of them edits it.

Frames: world NED, body FRD, camera OpenCV. Quaternions are (w, x, y, z),
body->NED, matching flight/deploy/real_flight.VehicleObs.quat.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol, Tuple

import numpy as np

Vec3 = np.ndarray  # shape (3,), float64


@dataclass
class VehicleState:
    t: float
    pos_ned: Vec3
    vel_ned: Vec3
    quat_wxyz: Tuple[float, float, float, float]   # body -> NED
    yaw_rad: float
    saturated: bool = False     # any tilt / thrust / rate limit active this step


@dataclass
class TargetState:
    t: float
    pos_ned: Vec3
    vel_ned: Vec3
    # tag_realism_v1 A1 (additive): the target's attitude, when a target
    # model derives one (isim.target_attitude.AttitudeTarget). None = "no
    # attitude model" -- every consumer keeps its legacy behaviour (the tag
    # hangs upright in world axes). Same convention as VehicleState.quat_wxyz.
    quat_wxyz: Optional[Tuple[float, float, float, float]] = None  # body -> NED
    ang_vel_body: Optional[Vec3] = None      # rad/s, target body FRD; None = zero


@dataclass
class VelCmd:
    """Same meaning as flight.guidance.Setpoint: NED velocity + absolute yaw."""
    v_north: float
    v_east: float
    v_down: float
    yaw_deg: float


@dataclass
class Detection:
    """One decoded AprilTag, as the Pi would report it."""
    t_capture: float            # sim time the frame was exposed
    t_available: float          # sim time the result reaches guidance (latency)
    u_px: float                 # tag centre, image column
    v_px: float                 # tag centre, image row
    side_px: float              # apparent tag side length in pixels
    range_m: float              # range from tag pose (noisy)
    bearing_deg: float          # horizontal, camera-relative, + = right
    elevation_deg: float        # vertical, camera-relative, + = up


@dataclass
class FrameReport:
    """What happened to one camera frame -- for the miss budget. Ground truth,
    SCORING ONLY: guidance never sees this."""
    t_capture: float
    in_fov: bool
    side_px: float
    incidence_deg: float
    blur_px: float
    p_decode: float
    decoded: bool
    # tag_realism_v1 diagnostics (additive, defaulted so every existing
    # constructor still works). All are TRUTH, scoring only.
    blur_rot_px: float = 0.0            # smear from target rotation + wobble (A4/B1)
    blur_vib_px: float = 0.0            # smear from own-camera vibration (B2)
    tgt_shake_deg: float = 0.0          # |wobble angle| applied to the tag this frame (B1)
    glare_specular_mult: float = 1.0    # specular-lobe decode multiplier (C)
    glare_backlight_mult: float = 1.0   # into-the-sun decode multiplier (C)


class VehicleModel(Protocol):
    def reset(self, state: VehicleState, rng: np.random.Generator) -> None: ...
    def step(self, cmd: VelCmd, dt: float) -> VehicleState: ...


class TargetModel(Protocol):
    def state(self, t: float) -> TargetState: ...


class SeekerModel(Protocol):
    def reset(self, rng: np.random.Generator) -> None: ...
    def observe(self, t: float, own: VehicleState, tgt: TargetState
                ) -> Tuple[Optional[Detection], Optional[FrameReport]]:
        """Call every sim step. Exposes a frame when one is due (FrameReport is
        returned for THAT step) and returns a Detection on the step its latency
        elapses; otherwise (None, None)."""
        ...


class Guidance(Protocol):
    def reset(self) -> None: ...
    def step(self, t: float, own: VehicleState, det: Optional[Detection]) -> VelCmd:
        """`own` stands in for the own-state EKF. Guidance must never be handed
        a TargetState or a FrameReport."""
        ...
