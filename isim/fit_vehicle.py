"""Fit isim.vehicle.QuadVelocityModel parameters against logged PX4/Gazebo
flight segments. Replays each segment's commands through the model (holding
each command constant between log samples -- zero-order hold, matching how
the real flight controller received it) and least-squares-fits the free
parameters to the logged velocity, altitude, and tilt-angle response.

Prefers scipy.optimize.least_squares (bounded trust-region) when scipy is
importable; otherwise falls back to a bounded Nelder-Mead written here (no
new dependency). Either way the model is a black box to the optimizer: it is
only ever called through simulate_segment.
"""
from __future__ import annotations

import math
from dataclasses import replace
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from isim.types import VehicleState, VelCmd
from isim.vehicle import QuadVelocityModel, VehicleParams

Segment = Dict[str, np.ndarray]

# Fit bounds for every scalar VehicleParams field that is sensible to fit.
# (wind_ned is a 3-tuple, not a scalar, and is deliberately excluded --
# fit that from a per-segment constant offset if ever needed.)
BOUNDS: Dict[str, Tuple[float, float]] = {
    "kp_vel_horiz": (0.1, 10.0), "ki_vel_horiz": (0.0, 5.0),
    "ki_vel_horiz_limit": (0.1, 10.0), "kp_vel_vert": (0.1, 15.0),
    "ki_vel_vert": (0.0, 5.0), "ki_vel_vert_limit": (0.1, 10.0),
    "max_setpoint_accel_horiz": (0.5, 20.0), "max_setpoint_accel_vert": (0.5, 20.0),
    "max_accel_horiz": (0.5, 20.0), "max_tilt_deg": (5.0, 60.0),
    "max_tilt_rate_deg_s": (30.0, 720.0), "tilt_time_constant_s": (0.01, 1.0),
    "thrust_max": (10.0, 40.0), "thrust_min": (0.0, 5.0),
    "thrust_time_constant_s": (0.01, 1.0), "drag_linear_horiz": (0.0, 3.0),
    "drag_linear_vert": (0.0, 3.0), "drag_quad_horiz": (0.0, 1.0),
    "drag_quad_vert": (0.0, 1.0), "yaw_rate_limit_deg_s": (10.0, 720.0),
    "yaw_time_constant_s": (0.01, 2.0), "latency_s": (0.0, 0.5),
    "gust_std": (0.0, 5.0),
}

# Residual channel weights: velocity in m/s and altitude in m are already
# comparable; tilt (degrees) is scaled down so a few degrees of tilt error
# does not dominate a fit that is mostly about translational tracking.
DEFAULT_WEIGHTS = {"vel": 1.0, "pos_d": 2.0, "tilt_deg": 0.05}


def _tilt_deg_from_quat(qw, qx, qy, qz) -> np.ndarray:
    """Angle (deg) between the body -z axis (rotated into NED by the
    body->NED quaternion) and vertical. Depends only on qx, qy: roll/pitch
    tilt is decoupled from yaw for a (w,x,y,z) quaternion."""
    cos_tilt = np.clip(1.0 - 2.0 * (qx**2 + qy**2), -1.0, 1.0)
    return np.degrees(np.arccos(cos_tilt))


def simulate_segment(
    params: VehicleParams, segment: Segment, dt_model: float = 0.005
) -> Segment:
    """Replay one logged segment's commands through the model, starting from
    the segment's own initial position/velocity, and return the model's
    trajectory RESAMPLED BACK onto the segment's own timestamps -- so it can
    be compared to the log sample-for-sample. Commands are held constant
    (zero-order hold) between log samples while the model substeps at
    dt_model for integration accuracy."""
    t = np.asarray(segment["t"], dtype=float)
    n = len(t)
    log_dt = float(np.median(np.diff(t))) if n > 1 else dt_model
    n_sub = max(1, round(log_dt / dt_model))
    dt_actual = log_dt / n_sub

    pos0 = np.array([segment["pos_n"][0], segment["pos_e"][0], segment["pos_d"][0]])
    vel0 = np.array([segment["vel_n"][0], segment["vel_e"][0], segment["vel_d"][0]])
    if "quat_w" in segment:
        quat0 = tuple(float(segment[k][0]) for k in ("quat_w", "quat_x", "quat_y", "quat_z"))
    else:
        quat0 = (1.0, 0.0, 0.0, 0.0)
    yaw0 = math.radians(float(segment["cmd_yaw_deg"][0]))

    model = QuadVelocityModel(params)
    rng = np.random.default_rng(0)  # deterministic: only matters if gust_std > 0
    init = VehicleState(t=t[0], pos_ned=pos0, vel_ned=vel0, quat_wxyz=quat0, yaw_rad=yaw0)
    model.reset(init, rng)

    out_pos, out_vel, out_quat = np.zeros((n, 3)), np.zeros((n, 3)), np.zeros((n, 4))
    out_pos[0], out_vel[0], out_quat[0] = pos0, vel0, quat0
    for i in range(n - 1):
        cmd = VelCmd(
            v_north=float(segment["cmd_vn"][i]),
            v_east=float(segment["cmd_ve"][i]),
            v_down=float(segment["cmd_vd"][i]),
            yaw_deg=float(segment["cmd_yaw_deg"][i]),
        )
        st = None
        for _ in range(n_sub):
            st = model.step(cmd, dt_actual)
        out_pos[i + 1] = st.pos_ned
        out_vel[i + 1] = st.vel_ned
        out_quat[i + 1] = st.quat_wxyz

    return {
        "t": t,
        "pos_n": out_pos[:, 0], "pos_e": out_pos[:, 1], "pos_d": out_pos[:, 2],
        "vel_n": out_vel[:, 0], "vel_e": out_vel[:, 1], "vel_d": out_vel[:, 2],
        "quat_w": out_quat[:, 0], "quat_x": out_quat[:, 1],
        "quat_y": out_quat[:, 2], "quat_z": out_quat[:, 3],
    }


def _channel_errors(params: VehicleParams, segment: Segment) -> Dict[str, np.ndarray]:
    """Per-channel error arrays (sim - logged) for one segment."""
    sim = simulate_segment(params, segment)
    errs = {
        "vel_n": sim["vel_n"] - segment["vel_n"],
        "vel_e": sim["vel_e"] - segment["vel_e"],
        "vel_d": sim["vel_d"] - segment["vel_d"],
        "pos_d": sim["pos_d"] - segment["pos_d"],
    }
    if "quat_w" in segment:
        tilt_sim = _tilt_deg_from_quat(sim["quat_w"], sim["quat_x"], sim["quat_y"], sim["quat_z"])
        tilt_log = _tilt_deg_from_quat(
            segment["quat_w"], segment["quat_x"], segment["quat_y"], segment["quat_z"]
        )
        errs["tilt_deg"] = tilt_sim - tilt_log
    return errs


def _rms_report(params: VehicleParams, segments: Sequence[Segment]) -> Dict[str, float]:
    """RMS of each error channel, pooled across all given segments."""
    pooled: Dict[str, List[np.ndarray]] = {}
    for seg in segments:
        for k, v in _channel_errors(params, seg).items():
            pooled.setdefault(k, []).append(v)
    return {k: float(np.sqrt(np.mean(np.concatenate(v) ** 2))) for k, v in pooled.items()}


def _residual_vector(
    params: VehicleParams, segments: Sequence[Segment], weights: Dict[str, float]
) -> np.ndarray:
    parts = []
    for seg in segments:
        errs = _channel_errors(params, seg)
        parts.append(weights["vel"] * errs["vel_n"])
        parts.append(weights["vel"] * errs["vel_e"])
        parts.append(weights["vel"] * errs["vel_d"])
        parts.append(weights["pos_d"] * errs["pos_d"])
        if "tilt_deg" in errs:
            parts.append(weights["tilt_deg"] * errs["tilt_deg"])
    return np.concatenate(parts)


def _bounded_nelder_mead(f, x0, lb, ub, max_iter=300, tol=1e-6):
    """Minimal bounded Nelder-Mead: standard reflect/expand/contract/shrink,
    with every candidate point clipped into [lb, ub] before evaluation.
    Used only when scipy is unavailable."""
    n = len(x0)
    if n == 0:
        return x0
    clip = lambda x: np.clip(x, lb, ub)
    step = np.maximum(0.1 * (ub - lb), 1e-3)
    simplex = [clip(x0)]
    for i in range(n):
        p = x0.copy()
        p[i] = p[i] + step[i] if p[i] + step[i] <= ub[i] else p[i] - step[i]
        simplex.append(clip(p))
    simplex = np.array(simplex)
    fvals = np.array([f(x) for x in simplex])

    for _ in range(max_iter):
        order = np.argsort(fvals)
        simplex, fvals = simplex[order], fvals[order]
        if abs(fvals[-1] - fvals[0]) < tol * (1.0 + abs(fvals[0])):
            break
        centroid = simplex[:-1].mean(axis=0)
        worst = simplex[-1]

        xr = clip(centroid + (centroid - worst))
        fr = f(xr)
        if fr < fvals[0]:
            xe = clip(centroid + 2.0 * (centroid - worst))
            fe = f(xe)
            simplex[-1], fvals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < fvals[-2]:
            simplex[-1], fvals[-1] = xr, fr
        else:
            xc = clip(centroid + 0.5 * (worst - centroid))
            fc = f(xc)
            if fc < fvals[-1]:
                simplex[-1], fvals[-1] = xc, fc
            else:
                for i in range(1, n + 1):
                    simplex[i] = clip(simplex[0] + 0.5 * (simplex[i] - simplex[0]))
                    fvals[i] = f(simplex[i])
    best = np.argmin(fvals)
    return simplex[best]


def fit_vehicle(
    segments: Sequence[Segment],
    p0: VehicleParams,
    free: List[str],
    holdout_segments: Optional[Sequence[Segment]] = None,
    weights: Optional[Dict[str, float]] = None,
) -> Tuple[VehicleParams, dict]:
    """Fit the named `free` VehicleParams fields to `segments`. Returns the
    fitted params and a report with per-channel RMS before/after, the fitted
    values, and (if holdout_segments given) hold-out RMS on unseen segments."""
    weights = weights or DEFAULT_WEIGHTS
    for name in free:
        if name not in BOUNDS:
            raise ValueError(f"no fit bound registered for VehicleParams.{name}")

    lb = np.array([BOUNDS[name][0] for name in free])
    ub = np.array([BOUNDS[name][1] for name in free])
    x0 = np.clip(np.array([getattr(p0, name) for name in free]), lb, ub)

    def unpack(x: np.ndarray) -> VehicleParams:
        return replace(p0, **{name: float(v) for name, v in zip(free, x)})

    def residuals(x: np.ndarray) -> np.ndarray:
        return _residual_vector(unpack(x), segments, weights)

    rms_before = _rms_report(p0, segments)

    if len(free) == 0:
        x_fit = x0
    else:
        try:
            from scipy.optimize import least_squares  # type: ignore

            result = least_squares(residuals, x0, bounds=(lb, ub))
            x_fit = result.x
            optimizer = "scipy.optimize.least_squares"
        except ImportError:
            x_fit = _bounded_nelder_mead(
                lambda x: float(np.sum(residuals(x) ** 2)), x0, lb, ub
            )
            optimizer = "bounded_nelder_mead (scipy unavailable)"

    fitted = unpack(x_fit)
    rms_after = _rms_report(fitted, segments)

    report = {
        "free": list(free),
        "optimizer": optimizer if len(free) else "no-op (free=[])",
        "fitted_values": {name: float(v) for name, v in zip(free, x_fit)},
        "rms_before": rms_before,
        "rms_after": rms_after,
    }
    if holdout_segments is not None:
        report["rms_holdout"] = _rms_report(fitted, holdout_segments)
    return fitted, report


def _make_synthetic_segment(
    truth: VehicleParams, cmd_profile, duration_s: float, dt_log: float, noise_rng, noise_std
) -> Segment:
    """Build one 'logged' segment by running the TRUE model at dt_log and
    adding measurement noise -- stands in for a real 50 Hz PX4 CSV."""
    n = int(round(duration_s / dt_log)) + 1
    t = np.arange(n) * dt_log
    model = QuadVelocityModel(truth)
    state = VehicleState(
        t=0.0, pos_ned=np.zeros(3), vel_ned=np.zeros(3),
        quat_wxyz=(1.0, 0.0, 0.0, 0.0), yaw_rad=0.0,
    )
    model.reset(state, np.random.default_rng(1))
    cmd_vn, cmd_ve, cmd_vd, cmd_yaw = cmd_profile(t)
    pos, vel, quat = np.zeros((n, 3)), np.zeros((n, 3)), np.zeros((n, 4))
    quat[0] = state.quat_wxyz
    for i in range(n - 1):
        cmd = VelCmd(v_north=cmd_vn[i], v_east=cmd_ve[i], v_down=cmd_vd[i], yaw_deg=cmd_yaw[i])
        st = model.step(cmd, dt_log)
        pos[i + 1], vel[i + 1], quat[i + 1] = st.pos_ned, st.vel_ned, st.quat_wxyz

    def noisy(arr):
        return arr + noise_rng.normal(0.0, noise_std, size=arr.shape)

    return {
        "t": t, "cmd_vn": cmd_vn, "cmd_ve": cmd_ve, "cmd_vd": cmd_vd, "cmd_yaw_deg": cmd_yaw,
        "pos_n": noisy(pos[:, 0]), "pos_e": noisy(pos[:, 1]), "pos_d": noisy(pos[:, 2]),
        "vel_n": noisy(vel[:, 0]), "vel_e": noisy(vel[:, 1]), "vel_d": noisy(vel[:, 2]),
        "quat_w": quat[:, 0], "quat_x": quat[:, 1], "quat_y": quat[:, 2], "quat_z": quat[:, 3],
    }


def _demo() -> None:
    """Self-demo: fit 3 perturbed parameters back to their truth values on
    synthetic (noisy) logged-style segments."""
    # thrust_max is deliberately NOT in this trio: with the default
    # max_accel_horiz=8 these command profiles never demand more than
    # ~12.7 m/s^2 of thrust, so any thrust_max above that is unidentifiable
    # (truth and p0 would look identical) -- a real identifiability limit,
    # not a fitter bug. Pick 3 params these profiles actually constrain.
    truth = replace(
        VehicleParams(), tilt_time_constant_s=0.18, drag_linear_horiz=0.6, kp_vel_horiz=2.4
    )
    p0 = replace(
        VehicleParams(), tilt_time_constant_s=0.30, drag_linear_horiz=0.10, kp_vel_horiz=1.0
    )
    free = ["tilt_time_constant_s", "drag_linear_horiz", "kp_vel_horiz"]

    def step_profile(t, vn=10.0, ve=0.0, t0=1.0):
        on = np.where(t < t0, 0.0, 1.0)
        return vn * on, ve * on, np.zeros_like(t), np.zeros_like(t)

    def diag_profile(t):
        return step_profile(t, vn=6.0, ve=-6.0, t0=0.5)

    noise_rng = np.random.default_rng(42)
    fit_segs = [
        _make_synthetic_segment(truth, step_profile, 4.0, 0.02, noise_rng, noise_std=0.03),
        _make_synthetic_segment(truth, diag_profile, 4.0, 0.02, noise_rng, noise_std=0.03),
    ]
    holdout_segs = [
        _make_synthetic_segment(truth, step_profile, 3.0, 0.02, noise_rng, noise_std=0.03)
    ]

    fitted, report = fit_vehicle(fit_segs, p0, free, holdout_segments=holdout_segs)

    print(f"optimizer: {report['optimizer']}")
    print(f"{'param':<22}{'truth':>10}{'p0':>10}{'fitted':>10}{'% err':>10}")
    for name in free:
        tv, fv = getattr(truth, name), report["fitted_values"][name]
        pct = 100.0 * abs(fv - tv) / abs(tv)
        print(f"{name:<22}{tv:>10.4f}{getattr(p0, name):>10.4f}{fv:>10.4f}{pct:>9.2f}%")
    print("\nRMS before fit:", {k: round(v, 4) for k, v in report["rms_before"].items()})
    print("RMS after fit: ", {k: round(v, 4) for k, v in report["rms_after"].items()})
    print("RMS holdout:   ", {k: round(v, 4) for k, v in report["rms_holdout"].items()})


if __name__ == "__main__":
    _demo()
