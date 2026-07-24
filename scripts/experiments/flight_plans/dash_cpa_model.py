#!/usr/bin/env python3
"""dash_cpa_model.py -- BALLISTIC CPA model for the coded open-loop dash.

STATUS: DESIGN-TIME ANALYSIS TOOL (pre-registration support for
docs/flight_plan_candidates.md). It runs NO sim. "Lab ranks, Gazebo decides"
(CLAUDE.md): this model RANKS candidate dash aims; only a paired-seed Gazebo
arm turns a ranking into a conclusion.

WHAT IT IS
----------
A point-mass model of the CODED_DASH phase, which is the only phase the
interceptor flies open-loop:

    interceptor: starts at the origin AT REST, accelerates at a constant
                 a (m/s^2) along a FIXED heading, clamped at v_max
                 s(t) = 0.5*a*t^2            for t <= v_max/a
                      = v_max*t - v_max^2/(2a) after that
    target:      constant velocity from its launch position (the mc_batch
                 line-9 crossing geometry)
    CPA:         min over t of |p_interceptor(t) - p_target(t)|

WHY IT MATTERS (the finding it was written to quantify)
------------------------------------------------------
`flight.guidance.collision_lead_heading` solves a CONSTANT-SPEED intercept
triangle at `--dash-speed` (16 m/s). The real quad starts from REST and needs
~1.3 s just to reach 16 m/s at PX4's MPC_ACC_HOR_MAX=12 -- and the whole
engagement is only ~1.5-2 s. So the coded dash flies a lead solution for a
vehicle that does not exist, and the residual is absorbed by the empirically
tuned `--dash-crossing-bias-deg`. That makes the bias an ACCELERATION-dependent
constant: change the dash accel (e.g. add `--dash-accel-cap`) WITHOUT re-sizing
the bias and you inject a systematic aim error.

HONESTY BOUNDARY
----------------
Nothing here runs in the guidance loop. The inputs are pre-flight constants
(target launch position/velocity = what the operator programs, own accel
capability = a vehicle spec). Logged `gt`/`miss_m` values are read ONLY to
SCORE/validate the model offline, never fed to guidance.

USAGE
  .venv/bin/python scripts/experiments/flight_plans/dash_cpa_model.py --validate
  .venv/bin/python scripts/experiments/flight_plans/dash_cpa_model.py --sweep
  .venv/bin/python scripts/experiments/flight_plans/dash_cpa_model.py --sensitivity
  .venv/bin/python scripts/experiments/flight_plans/dash_cpa_model.py --all
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

from flight.guidance import collision_lead_heading  # noqa: E402

# --- canonical line-9 geometry (scripts/experiments/loft_dive/run_arm.sh) ---
X0 = 6.5                 # target start east, m (mc_batch --x0)
DASH_SPEED = 16.0        # --dash-speed: what the lead solve ASSUMES
FLOWN_BIAS = 30.0        # --dash-crossing-bias-deg on the flown arms
# PX4 horizontal accel limit under --fpv (m4_intercept FPV["PX4_PARAMS"]).
A_UNCAPPED = 12.0

ARM_A = "logs/mc_loftdive_armA_line9_s123.csv"
ARM_ADASH = "logs/mc_loftdive_armAdash_line9_s123.csv"
ARM_BDASH = "logs/mc_loftdive_armBdash_line9_s123.csv"


def dash_distance(t, a, v_max=DASH_SPEED):
    """Distance travelled from rest under constant accel `a`, clamped v_max."""
    t_b = v_max / a
    return 0.5 * a * t * t if t <= t_b else v_max * t - v_max * v_max / (2.0 * a)


def ballistic_cpa(heading_deg, a, start, vel, v_max=DASH_SPEED,
                  t_max=6.0, dt=0.002):
    """Closest approach (m) of the fixed-heading accelerating dash to a
    constant-velocity target. start/vel are (east, north)."""
    hx, hy = math.sin(math.radians(heading_deg)), math.cos(math.radians(heading_deg))
    best, t = float("inf"), 0.0
    while t <= t_max:
        s = dash_distance(t, a, v_max)
        d = math.hypot(s * hx - (start[0] + vel[0] * t),
                       s * hy - (start[1] + vel[1] * t))
        if d < best:
            best = d
        t += dt
    return best


def flown_heading(start, vel, bias_deg, dash_speed=DASH_SPEED):
    """The heading m4_intercept.py actually flies: the constant-speed lead
    solve, then the sign-keyed crossing bias (m4_intercept.py:2296-2306)."""
    h, _ = collision_lead_heading(start, vel, dash_speed)
    cross = math.sin(math.radians(h)) * vel[1] - math.cos(math.radians(h)) * vel[0]
    if cross != 0.0:
        h -= math.copysign(bias_deg, cross)
    return h


def load_geometry(arm_csv=ARM_A):
    """The 8 per-flight canonical geometries (y-jitter included) from a flown
    mc_batch arm CSV -- so predictions are per-flight PAIRED with the logs."""
    out = []
    for r in csv.DictReader(open(os.path.join(_REPO, arm_csv))):
        out.append({
            "run": r["run_idx"], "dir": r["direction"],
            "start": (X0, float(r["target_start_y"])),
            "vel": (0.0, float(r["target_vy"])),
            "miss": float(r["miss_m"]) if r.get("miss_m") else None,
        })
    return out


def mean_cpa(bias_deg, a, geo):
    return sum(ballistic_cpa(flown_heading(g["start"], g["vel"], bias_deg),
                             a, g["start"], g["vel"]) for g in geo) / len(geo)


def optimal_bias(a, geo, lo=0.0, hi=90.0, step=1.0):
    best = None
    b = lo
    while b <= hi:
        m = mean_cpa(b, a, geo)
        if best is None or m < best[1]:
            best = (b, m)
        b += step
    return best


# ----------------------------------------------------------------- validate
def validate():
    """Fit the model's effective accel to the two DASH-ONLY arms (camera
    disabled -> pure ballistics, the only clean test of a ballistic model)."""
    print("=== VALIDATION: model vs the flown dash-only arms (bias 30) ===")
    print("(dash-only = --coded-dash-acquire-range-min 999: handoff never "
          "fires, 0 ENGAGE ticks -> the logged miss IS the dash ballistic CPA)")
    for arm, cands in ((ARM_ADASH, (8.0, 9.0, 10.0, 12.0, 14.0, 16.0)),
                       (ARM_BDASH, (3.0, 3.57, 4.0, 4.5, 4.78, 5.5))):
        p = os.path.join(_REPO, arm)
        if not os.path.exists(p):
            print(f"  (missing {arm} -- skipped)")
            continue
        geo = load_geometry(arm)
        print(f"\n  -- {arm}")
        best = None
        for a in cands:
            errs = [ballistic_cpa(flown_heading(g["start"], g["vel"], FLOWN_BIAS),
                                  a, g["start"], g["vel"]) - g["miss"] for g in geo]
            mae = sum(abs(e) for e in errs) / len(errs)
            bias = sum(errs) / len(errs)
            print(f"     a={a:5.2f} m/s^2  MAE={mae:4.2f} m  mean_signed={bias:+5.2f} m")
            if best is None or mae < best[1]:
                best = (a, mae)
        print(f"     -> best-fit effective accel {best[0]:.2f} m/s^2 (MAE {best[1]:.2f} m)")
        geo = load_geometry(arm)
        a = best[0]
        for g in geo:
            pred = ballistic_cpa(flown_heading(g["start"], g["vel"], FLOWN_BIAS),
                                 a, g["start"], g["vel"])
            print(f"       run {g['run']} {g['dir']:>3}  pred {pred:5.2f}  "
                  f"meas {g['miss']:5.2f}  err {pred - g['miss']:+5.2f}")


# -------------------------------------------------------------------- sweep
def sweep():
    """The headline table: the ballistically-optimal crossing bias is a
    FUNCTION OF THE DASH ACCEL."""
    geo = load_geometry()
    print("=== SWEEP: model-optimal --dash-crossing-bias-deg vs dash accel ===")
    print(f"{'a (m/s^2)':>10} {'theta=atan(a/g)':>16} {'opt bias':>9} "
          f"{'CPA@opt':>8} {'CPA@30 (flown)':>15}")
    for a in (3.0, 3.57, 4.5, 5.66, 7.0, 9.0, 12.0, 16.0):
        b, m = optimal_bias(a, geo)
        print(f"{a:10.2f} {math.degrees(math.atan(a / 9.80665)):15.1f}d "
              f"{b:9.1f} {m:8.2f} {mean_cpa(FLOWN_BIAS, a, geo):15.2f}")
    print("\n  Read: --dash-accel-cap 3.57 (the flown ARM B) needs bias ~64, not 30.")
    print("  Read: the UNCAPPED dash (a~12) is itself over-biased at 30 (model opt ~16).")


# -------------------------------------------------------------- sensitivity
def sensitivity():
    """Aim-error budget -- the number the LAUNCH mechanism must meet
    (docs/launch_mechanism_plan.md)."""
    geo = load_geometry()
    print("=== SENSITIVITY: CPA vs launch-aim error, about the model optimum ===")
    for a in (A_UNCAPPED, 3.57):
        b_opt, _ = optimal_bias(a, geo)
        print(f"\n  -- dash accel {a} m/s^2 (model-optimal bias {b_opt:.0f} deg)")
        for d in (0, 1, 2, 3, 5, 8, 10, 15, 20):
            lo, hi = mean_cpa(b_opt - d, a, geo), mean_cpa(b_opt + d, a, geo)
            print(f"     aim error +-{d:2d} deg -> CPA {lo:5.2f} / {hi:5.2f} m")
        for bar in (0.5, 1.0, 2.5):
            d = 0.0
            while d < 60 and max(mean_cpa(b_opt - d, a, geo),
                                 mean_cpa(b_opt + d, a, geo)) < bar:
                d += 0.5
            print(f"     aim tolerance for CPA <= {bar} m:  +-{d - 0.5:.1f} deg")


# ------------------------------------------------------------------ predict
def predict(bias_list, accel_list):
    """Per-flight pre-registered predictions for candidate arms."""
    geo = load_geometry()
    print("=== PER-FLIGHT PREDICTIONS (pre-registration) ===")
    for a in accel_list:
        for b in bias_list:
            preds = [ballistic_cpa(flown_heading(g["start"], g["vel"], b), a,
                                   g["start"], g["vel"]) for g in geo]
            srt = sorted(preds)
            print(f"  a={a:5.2f} bias={b:5.1f} -> per-flight "
                  f"[{', '.join(f'{p:.2f}' for p in preds)}]  "
                  f"median {0.5 * (srt[3] + srt[4]):.2f} m")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--validate", action="store_true")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--sensitivity", action="store_true")
    ap.add_argument("--predict", action="store_true")
    ap.add_argument("--all", action="store_true")
    a = ap.parse_args()
    if a.all or a.validate:
        validate()
        print()
    if a.all or a.sweep:
        sweep()
        print()
    if a.all or a.sensitivity:
        sensitivity()
        print()
    if a.all or a.predict:
        predict([16.0, 23.0, 30.0, 45.0, 64.0], [12.0, 3.57])
    if not any((a.validate, a.sweep, a.sensitivity, a.predict, a.all)):
        ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
