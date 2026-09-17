"""WP6: fit the vehicle model to the logged PX4/Gazebo flights and write the
result (params + per-channel RMS + hold-out RMS + the flight denominator) to
isim/fits/. Usage: python -m isim.fit_logs [--n-fit 30] [--n-hold 30] [--max-s 6]
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import time

import numpy as np

from isim.fit_vehicle import fit_vehicle
from isim.fitdata import flights_from_batches, load_all
from isim.vehicle import VehicleParams

FREE = ["kp_vel_horiz", "max_setpoint_accel_horiz", "max_accel_horiz",
        "max_tilt_deg", "tilt_time_constant_s", "thrust_max",
        "thrust_time_constant_s", "drag_linear_horiz", "drag_quad_horiz",
        "kp_vel_vert", "ki_vel_vert"]
# Differenced-position velocity is noisy (pose-sample jitter): weight it down,
# lean on altitude and tilt, which are logged directly.
WEIGHTS = {"vel": 0.3, "pos_d": 4.0, "tilt_deg": 0.1}


def _trim(seg, max_s):
    keep = seg["t"] <= max_s
    return {k: v[keep] for k, v in seg.items()}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-fit", type=int, default=30)
    ap.add_argument("--n-hold", type=int, default=30)
    ap.add_argument("--max-s", type=float, default=6.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="isim/fits/vehicle_gazebo_x500.json")
    a = ap.parse_args()
    named = len(flights_from_batches())
    segs = load_all()
    paths = sorted(segs)
    rng = np.random.default_rng(a.seed)
    rng.shuffle(paths)
    if len(paths) < a.n_fit + a.n_hold:
        print(f"FAIL: only {len(paths)} usable flights")
        return 1
    fit = [_trim(segs[p], a.max_s) for p in paths[:a.n_fit]]
    hold = [_trim(segs[p], a.max_s) for p in paths[a.n_fit:a.n_fit + a.n_hold]]
    t0 = time.time()
    params, rep = fit_vehicle(fit, VehicleParams(), FREE, holdout_segments=hold,
                              weights=WEIGHTS)
    out = {"flights_named": named, "flights_usable": len(paths),
           "n_fit": a.n_fit, "n_hold": a.n_hold, "max_s": a.max_s,
           "free": FREE, "weights": WEIGHTS, "fit_wall_s": round(time.time() - t0, 1),
           "params": dataclasses.asdict(params),
           "report": json.loads(json.dumps(rep, default=lambda o: (
               o.tolist() if hasattr(o, "tolist") else
               dataclasses.asdict(o) if dataclasses.is_dataclass(o) else str(o))))}
    with open(a.out, "w") as fh:
        json.dump(out, fh, indent=2)
    print(json.dumps({k: out["report"].get(k) for k in out["report"]
                      if "rms" in k.lower()}, indent=1))
    print("FIT_DONE", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
