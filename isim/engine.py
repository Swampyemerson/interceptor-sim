"""The engagement loop (ADR-0102 isim). Deterministic, headless, sim-time
only: no wall clock anywhere in this module.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from isim.types import (
    Guidance,
    SeekerModel,
    TargetModel,
    VehicleModel,
    VehicleState,
    VelCmd,
)

_TRACE_VECTOR_FIELDS = {
    "own_pos": 3,
    "own_vel": 3,
    "quat": 4,
    "cmd": 4,
    "tgt_pos": 3,
    "tgt_vel": 3,
}
_TRACE_SCALAR_FIELDS = ("t", "yaw", "range", "saturated", "det_new", "det_u", "det_v", "det_range")


@dataclass
class EngagementConfig:
    dt: float = 0.005
    guidance_dt: float = 0.02
    max_t: float = 30.0
    stop_after_cpa_s: float = 1.0
    # The stop rule sleeps until this sim time: a vehicle holding station before
    # its GO has a flat or growing range, which is not "past closest approach".
    stop_not_before_s: float = 0.0
    seed: int = 0


@dataclass
class EngagementResult:
    miss_m: float
    t_cpa: float
    miss_vec_ned: np.ndarray          # target - vehicle, at CPA
    miss_horiz_m: float
    miss_vert_m: float                # signed, + = vehicle below target
    closing_speed_ms: float
    n_frames: int
    n_in_fov: int
    n_decoded: int
    frac_saturated: float
    trace: Optional[Dict[str, np.ndarray]] = None
    frame_reports: List = field(default_factory=list)


def _alloc_trace(n_max: int) -> Dict[str, np.ndarray]:
    trace: Dict[str, np.ndarray] = {}
    for name, width in _TRACE_VECTOR_FIELDS.items():
        trace[name] = np.zeros((n_max, width), dtype=np.float64)
    for name in _TRACE_SCALAR_FIELDS:
        trace[name] = np.zeros(n_max, dtype=np.float64)
    return trace


def run_engagement(
    cfg: EngagementConfig,
    vehicle: VehicleModel,
    target: TargetModel,
    seeker: SeekerModel,
    guidance: Guidance,
    init_state: VehicleState,
    record_trace: bool = False,
) -> EngagementResult:
    rng = np.random.default_rng(cfg.seed)
    vehicle_rng, seeker_rng = rng.spawn(2)
    vehicle.reset(init_state, vehicle_rng)
    seeker.reset(seeker_rng)
    guidance.reset()

    dt = cfg.dt
    guidance_steps = max(1, round(cfg.guidance_dt / dt))
    n_steps = max(0, int(round(cfg.max_t / dt)))

    own = init_state
    tgt = target.state(0.0)
    pending_det = None
    cmd = VelCmd(v_north=0.0, v_east=0.0, v_down=0.0, yaw_deg=math.degrees(init_state.yaw_rad))

    best_range = math.inf
    best_t = 0.0
    best_miss_vec = np.zeros(3)
    best_vrel = np.zeros(3)

    min_range_running = math.inf
    t_at_min_running = 0.0

    n_frames = 0
    n_in_fov = 0
    n_decoded = 0
    n_saturated = 0
    frame_reports: List = []

    trace = _alloc_trace(n_steps) if record_trace else None
    actual_steps = 0

    for k in range(n_steps):
        t = k * dt
        tgt = target.state(t)
        det, rep = seeker.observe(t, own, tgt)

        if rep is not None:
            n_frames += 1
            if rep.in_fov:
                n_in_fov += 1
            if rep.decoded:
                n_decoded += 1
            frame_reports.append(rep)

        if det is not None:
            pending_det = det

        if k % guidance_steps == 0:
            cmd = guidance.step(t, own, pending_det)
            pending_det = None

        if trace is not None:
            trace["t"][k] = t
            trace["own_pos"][k] = own.pos_ned
            trace["own_vel"][k] = own.vel_ned
            trace["quat"][k] = own.quat_wxyz
            trace["yaw"][k] = own.yaw_rad
            trace["cmd"][k] = (cmd.v_north, cmd.v_east, cmd.v_down, cmd.yaw_deg)
            trace["tgt_pos"][k] = tgt.pos_ned
            trace["tgt_vel"][k] = tgt.vel_ned
            trace["range"][k] = np.linalg.norm(tgt.pos_ned - own.pos_ned)
            trace["saturated"][k] = 1.0 if own.saturated else 0.0
            if det is not None:
                trace["det_new"][k] = 1.0
                trace["det_u"][k] = det.u_px
                trace["det_v"][k] = det.v_px
                trace["det_range"][k] = det.range_m
            else:
                trace["det_new"][k] = 0.0
                trace["det_u"][k] = math.nan
                trace["det_v"][k] = math.nan
                trace["det_range"][k] = math.nan

        own_next = vehicle.step(cmd, dt)
        t_next = t + dt
        tgt_next = target.state(t_next)

        r0 = tgt.pos_ned - own.pos_ned
        r1 = tgt_next.pos_ned - own_next.pos_ned
        vrel = (r1 - r0) / dt
        denom = float(np.dot(vrel, vrel))
        if denom > 1e-12:
            tau = -float(np.dot(r0, vrel)) / denom
            tau = min(max(tau, 0.0), dt)
        else:
            tau = 0.0
        r_tau = r0 + vrel * tau
        range_tau = float(np.linalg.norm(r_tau))
        t_tau = t + tau

        if range_tau < best_range:
            best_range = range_tau
            best_t = t_tau
            best_miss_vec = r_tau
            best_vrel = vrel

        if range_tau < min_range_running:
            min_range_running = range_tau
            t_at_min_running = t_tau

        if own_next.saturated:
            n_saturated += 1

        actual_steps += 1
        own, tgt = own_next, tgt_next

        if (t_next >= cfg.stop_not_before_s
                and t_next - max(t_at_min_running, cfg.stop_not_before_s)
                >= cfg.stop_after_cpa_s):
            break

    if trace is not None:
        for name in trace:
            trace[name] = trace[name][:actual_steps]

    miss_vec_ned = best_miss_vec
    miss_horiz_m = float(math.hypot(miss_vec_ned[0], miss_vec_ned[1]))
    miss_vert_m = float(-miss_vec_ned[2])
    closing_speed_ms = float(np.linalg.norm(best_vrel))
    frac_saturated = (n_saturated / actual_steps) if actual_steps > 0 else 0.0

    return EngagementResult(
        miss_m=float(best_range),
        t_cpa=float(best_t),
        miss_vec_ned=miss_vec_ned,
        miss_horiz_m=miss_horiz_m,
        miss_vert_m=miss_vert_m,
        closing_speed_ms=closing_speed_ms,
        n_frames=n_frames,
        n_in_fov=n_in_fov,
        n_decoded=n_decoded,
        frac_saturated=frac_saturated,
        trace=trace,
        frame_reports=frame_reports,
    )
