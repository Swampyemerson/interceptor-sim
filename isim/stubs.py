"""Trivial reference implementations of the four protocols, so the engine is
testable stand-alone (no PX4/vehicle model, no real seeker needed). These are
NOT the deployed models -- just fixtures.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np

from isim.types import (
    Detection,
    FrameReport,
    TargetState,
    VehicleState,
    VelCmd,
)


def yaw_to_quat_wxyz(yaw_rad: float) -> Tuple[float, float, float, float]:
    """Quaternion (body->NED) for a pure yaw rotation about the down axis."""
    half = 0.5 * yaw_rad
    return (math.cos(half), 0.0, 0.0, math.sin(half))


class FirstOrderVehicle:
    """Velocity follows the commanded NED velocity with a first-order lag;
    yaw follows the commanded yaw instantly; attitude is pure yaw (no
    roll/pitch modelled). Saturates commanded speed at `vmax`."""

    def __init__(self, tau_s: float = 0.4, vmax: float = 15.0) -> None:
        self.tau_s = tau_s
        self.vmax = vmax
        self._pos = np.zeros(3)
        self._vel = np.zeros(3)
        self._yaw = 0.0
        self._t = 0.0

    def reset(self, state: VehicleState, rng: np.random.Generator) -> None:
        self._rng = rng
        self._pos = np.array(state.pos_ned, dtype=np.float64).copy()
        self._vel = np.array(state.vel_ned, dtype=np.float64).copy()
        self._yaw = float(state.yaw_rad)
        self._t = float(state.t)

    def step(self, cmd: VelCmd, dt: float) -> VehicleState:
        v_cmd = np.array([cmd.v_north, cmd.v_east, cmd.v_down])
        speed_cmd = float(np.linalg.norm(v_cmd))
        saturated = speed_cmd > self.vmax
        if saturated and speed_cmd > 1e-12:
            v_cmd = v_cmd * (self.vmax / speed_cmd)
        self._vel = self._vel + (v_cmd - self._vel) * (dt / self.tau_s)
        self._pos = self._pos + self._vel * dt
        self._yaw = math.radians(cmd.yaw_deg)
        self._t += dt
        quat = yaw_to_quat_wxyz(self._yaw)
        return VehicleState(
            t=self._t,
            pos_ned=self._pos.copy(),
            vel_ned=self._vel.copy(),
            quat_wxyz=quat,
            yaw_rad=self._yaw,
            saturated=saturated,
        )


class PerfectSeeker:
    """Noiseless camera: reports exact bearing/elevation/range whenever the
    target is within the (level, yaw-only) FOV, at `fps`, with optional fixed
    `latency_s`. Pinhole intrinsics fx=fy=540, cx=640, cy=400; tag side
    0.2 m."""

    _FX = 540.0
    _FY = 540.0
    _CX = 640.0
    _CY = 400.0
    _TAG_SIDE_M = 0.2

    def __init__(
        self,
        fps: float = 30.0,
        latency_s: float = 0.0,
        hfov_deg: float = 100.0,
        vfov_deg: float = 80.0,
    ) -> None:
        self.fps = fps
        self.latency_s = latency_s
        self.hfov_deg = hfov_deg
        self.vfov_deg = vfov_deg
        self._next_frame_idx = 0
        self._pending: Deque[Detection] = deque()

    def reset(self, rng: np.random.Generator) -> None:
        self._rng = rng
        self._next_frame_idx = 0
        self._pending = deque()

    def observe(
        self, t: float, own: VehicleState, tgt: TargetState
    ) -> Tuple[Optional[Detection], Optional[FrameReport]]:
        rep: Optional[FrameReport] = None
        frame_due_t = self._next_frame_idx / self.fps
        if t + 1e-9 >= frame_due_t:
            self._next_frame_idx += 1
            rep, det_new = self._make_report_and_detection(t, own, tgt)
            if det_new is not None:
                self._pending.append(det_new)

        det: Optional[Detection] = None
        if self._pending and self._pending[0].t_available <= t + 1e-9:
            det = self._pending.popleft()
        return det, rep

    def _make_report_and_detection(
        self, t: float, own: VehicleState, tgt: TargetState
    ) -> Tuple[FrameReport, Optional[Detection]]:
        psi = own.yaw_rad
        fwd = np.array([math.cos(psi), math.sin(psi), 0.0])
        right = np.array([-math.sin(psi), math.cos(psi), 0.0])
        r = tgt.pos_ned - own.pos_ned
        rx = float(np.dot(r, fwd))
        ry = float(np.dot(r, right))
        rz = float(r[2])
        range_m = float(np.linalg.norm(r))
        fwd_horiz = math.hypot(rx, ry)
        bearing_deg = math.degrees(math.atan2(ry, rx))
        elevation_deg = (
            math.degrees(math.atan2(-rz, fwd_horiz)) if fwd_horiz > 1e-9 else 0.0
        )
        in_fov = (
            rx > 0.0
            and abs(bearing_deg) <= self.hfov_deg / 2.0
            and abs(elevation_deg) <= self.vfov_deg / 2.0
        )
        decoded = in_fov
        p_decode = 1.0 if in_fov else 0.0
        side_px = self._FX * self._TAG_SIDE_M / range_m if range_m > 1e-6 else 0.0
        rep = FrameReport(
            t_capture=t,
            in_fov=in_fov,
            side_px=side_px,
            incidence_deg=0.0,
            blur_px=0.0,
            p_decode=p_decode,
            decoded=decoded,
        )
        det: Optional[Detection] = None
        if decoded:
            bearing_rad = math.radians(bearing_deg)
            elevation_rad = math.radians(elevation_deg)
            u_px = self._CX + self._FX * math.tan(bearing_rad)
            v_px = self._CY - self._FY * math.tan(elevation_rad)
            det = Detection(
                t_capture=t,
                t_available=t + self.latency_s,
                u_px=u_px,
                v_px=v_px,
                side_px=side_px,
                range_m=range_m,
                bearing_deg=bearing_deg,
                elevation_deg=elevation_deg,
            )
        return rep, det


class PursuitGuidance:
    """Flies at a constant `speed` along the line of sight of the latest
    detection (reconstructed from own yaw + bearing + elevation). Before the
    first detection ever arrives, flies along the initial yaw."""

    def __init__(self, speed: float) -> None:
        self.speed = speed
        self._los_dir: Optional[np.ndarray] = None
        self._initial_yaw: Optional[float] = None

    def reset(self) -> None:
        self._los_dir = None
        self._initial_yaw = None

    def step(self, t: float, own: VehicleState, det: Optional[Detection]) -> VelCmd:
        if self._initial_yaw is None:
            self._initial_yaw = own.yaw_rad
        if det is not None:
            self._los_dir = self._reconstruct_dir(own.yaw_rad, det)

        if self._los_dir is not None:
            direction = self._los_dir
        else:
            direction = np.array(
                [math.cos(self._initial_yaw), math.sin(self._initial_yaw), 0.0]
            )

        vel = direction * self.speed
        yaw_deg = math.degrees(math.atan2(direction[1], direction[0]))
        return VelCmd(v_north=float(vel[0]), v_east=float(vel[1]), v_down=float(vel[2]), yaw_deg=yaw_deg)

    @staticmethod
    def _reconstruct_dir(yaw_rad: float, det: Detection) -> np.ndarray:
        fwd = np.array([math.cos(yaw_rad), math.sin(yaw_rad), 0.0])
        right = np.array([-math.sin(yaw_rad), math.cos(yaw_rad), 0.0])
        down = np.array([0.0, 0.0, 1.0])
        b = math.radians(det.bearing_deg)
        e = math.radians(det.elevation_deg)
        dir_body = np.array([math.cos(e) * math.cos(b), math.cos(e) * math.sin(b), -math.sin(e)])
        dir_ned = dir_body[0] * fwd + dir_body[1] * right + dir_body[2] * down
        norm = np.linalg.norm(dir_ned)
        if norm > 1e-12:
            dir_ned = dir_ned / norm
        return dir_ned
