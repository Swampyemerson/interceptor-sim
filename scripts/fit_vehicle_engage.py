#!/usr/bin/env python3
"""ENGAGE-regime vehicle re-fit (tick-trace S4 fix -- isim/specs/
xcheck_tick_trace_2026-09-23.md).

The deployed isim vehicle fit (isim/fits/vehicle_gazebo_x500.json) was shaped
on DASH segments; the 2026-09-23 Gazebo pursuit cross-check's tick trace
measured the real vehicle ~2x slower horizontally (tau 0.51/0.60 s vs the
model's 0.30/0.36) with ~0.2 s more dead time in the braking/lateral ENGAGE
regime -- the regime rule: a fit validated at one operating point is not
validated at another. This script re-fits the SAME model class
(isim.vehicle.QuadVelocityModel, via isim.fit_vehicle.fit_vehicle) on the
EIGHT cross-check flights' ENGAGE segments and writes a NEW fit file
(isim/fits/vehicle_gazebo_x500_engage.json). The dash-regime fit is NOT
overwritten -- each fit is named for its regime.

Data: logs/xcheck_gz_20260923_fixed/f{1..8}.csv (driver per-tick: sim_t,
ENGAGE state, gz-truth own position -- scoring-only ground truth, used here to
fit the PLANT, never guidance) + f{1..8}_rf.csv (the commanded setpoints the
flight code actually emitted). Velocity channel = 5-point-boxcar-smoothed
gradient of the gz-truth position (the tick-trace instrument's own
convention); altitude channel = gz-truth down position directly.

Usage: .venv/bin/python scripts/fit_vehicle_engage.py
       [--log-dir .../logs/xcheck_gz_20260923_fixed] [--out isim/fits/...]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _REPO)

from isim.fit_vehicle import fit_vehicle  # noqa: E402
from isim.replay_a0 import load_params    # noqa: E402

DEFAULT_LOG_DIR = "/home/emerson/interceptor-sim/logs/xcheck_gz_20260923_fixed"

# Free parameters: the horizontal response chain the tick trace indicted
# (gain, accel/tilt limits, tilt lag, thrust, drag), the command-path dead
# time (latency_s -- measured 0.18-0.24 s vs the dash fit's 0.14), and the
# vertical loop the dash fit made too PESSIMISTIC (real 0.08/0.30 s vs model
# 0.29/0.99). No quat channel in these logs, so tilt params are constrained
# only through the velocity response -- documented, not hidden.
FREE = ["kp_vel_horiz", "max_setpoint_accel_horiz", "max_accel_horiz",
        "max_tilt_deg", "tilt_time_constant_s", "thrust_max",
        "drag_linear_horiz", "kp_vel_vert", "ki_vel_vert", "latency_s"]
# fit_logs.py's weights, minus the tilt channel (no quat logged here).
WEIGHTS = {"vel": 1.0, "pos_d": 4.0, "tilt_deg": 0.05}


def _f(x):
    return math.nan if x in ("", None) else float(x)


def _smooth(x, n=5):
    if len(x) < n:
        return x
    return np.convolve(x, np.ones(n) / n, mode="same")


def load_engage_segment(log_dir: str, k: int):
    with open(os.path.join(log_dir, f"f{k}.csv")) as fh:
        drv = list(csv.DictReader(fh))
    with open(os.path.join(log_dir, f"f{k}_rf.csv")) as fh:
        rf = list(csv.DictReader(fh))
    assert len(drv) == len(rf), f"f{k}: {len(drv)} vs {len(rf)} rows"
    idx = [i for i, r in enumerate(drv) if r["sm_state_pre_step"] == "ENGAGE"]
    i0, i1 = idx[0], idx[-1]
    t = np.array([_f(drv[i]["sim_t"]) for i in range(i0, i1 + 1)])
    keep = np.concatenate([[True], np.diff(t) > 1e-6])   # strictly increasing
    sl = np.arange(i0, i1 + 1)[keep]
    t = t[keep]
    pos = np.stack([np.array([_f(drv[i][c]) for i in sl])
                    for c in ("own_n_gz", "own_e_gz", "own_d_gz")])
    vel = np.stack([_smooth(np.gradient(pos[a], t), 5) for a in range(3)])
    cmd = np.stack([np.array([_f(rf[i][c]) for i in sl])
                    for c in ("v_north", "v_east", "v_down")])
    yaw_cmd = np.array([_f(rf[i]["yaw_cmd_deg"]) for i in sl])
    assert not np.isnan(pos).any() and not np.isnan(cmd).any(), f"f{k}: NaNs"
    return {
        "t": t,
        "pos_n": pos[0], "pos_e": pos[1], "pos_d": pos[2],
        "vel_n": vel[0], "vel_e": vel[1], "vel_d": vel[2],
        "cmd_vn": cmd[0], "cmd_ve": cmd[1], "cmd_vd": cmd[2],
        "cmd_yaw_deg": yaw_cmd,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log-dir", default=DEFAULT_LOG_DIR)
    ap.add_argument("--out", default=os.path.join(
        _REPO, "isim", "fits", "vehicle_gazebo_x500_engage.json"))
    args = ap.parse_args()

    segs = [load_engage_segment(args.log_dir, k) for k in range(1, 9)]
    for k, s in enumerate(segs, 1):
        print(f"f{k}: {len(s['t'])} rows, {s['t'][-1] - s['t'][0]:.1f} s ENGAGE")

    p0 = load_params()   # start from the deployed dash-regime fit
    t0 = time.time()
    fitted, report = fit_vehicle(segs, p0, FREE, weights=WEIGHTS)
    wall = time.time() - t0
    print(f"fit wall {wall:.1f} s  optimizer {report['optimizer']}")
    for name in FREE:
        print(f"  {name:<26} {getattr(p0, name):>10.4f} -> "
              f"{report['fitted_values'][name]:>10.4f}")
    print("RMS before:", {k: round(v, 3) for k, v in report["rms_before"].items()})
    print("RMS after: ", {k: round(v, 3) for k, v in report["rms_after"].items()})

    params = {f: getattr(fitted, f) for f in type(fitted).__dataclass_fields__}
    params["wind_ned"] = list(params["wind_ned"])
    out = {
        "regime": "ENGAGE (braking / lateral-nulling terminal), NOT dash",
        "provenance": (
            "Fit on the 8 Gazebo pursuit cross-check flights' full ENGAGE "
            "segments, logs/xcheck_gz_20260923_fixed/f{1..8}{,_rf}.csv "
            "(docs/xcheck_gazebo_pursuit_prereg.md n=8 run, 2026-09-23), by "
            "scripts/fit_vehicle_engage.py. Motivation + measured mismatch: "
            "isim/specs/xcheck_tick_trace_2026-09-23.md S4 (real horiz "
            "delay/tau 0.24/0.51 and 0.18/0.60 s vs the dash fit's 0.03/0.30 "
            "and 0.02/0.36; vertical the dash fit was the pessimist). "
            "p0 = isim/fits/vehicle_gazebo_x500.json (the dash-regime fit, "
            "kept as-is for dash work -- the regime rule cuts both ways)."),
        "flights": [f"f{k} ENGAGE, {len(s['t'])} rows,"
                    f" {s['t'][-1] - s['t'][0]:.1f} s" for k, s in
                    enumerate(segs, 1)],
        "channels": ("vel = 5-pt boxcar smoothed gradient of gz-truth "
                     "position (tick-trace convention); pos_d = gz-truth "
                     "down; NO tilt channel (rf.csv logs no quaternion)"),
        "free": FREE,
        "weights": WEIGHTS,
        "fit_wall_s": round(wall, 1),
        "params": params,
        "report": report,
    }
    with open(args.out, "w") as fh:
        json.dump(out, fh, indent=1)
    print("wrote", args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
