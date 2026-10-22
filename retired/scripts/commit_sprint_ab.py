#!/usr/bin/env python3
"""Commit-sprint-in-the-coast A/B (2026-09-24, builder idea). PRE-REGISTERED:
isim/specs/commit_sprint_prereg_2026-09-24.md -- read it before interpreting
output.

Port arm (real flight code, terminal="pursuit"), rear tag, all v5 errors on
(scatter=Scatter()), tag-realism rung R2 (target attitude + default wobble),
honest ENGAGE vehicle fit, paired seeds 0..49. Four arms: the sprint OFF/ON
at two base configs -- the plain default and the brake package (the
--pursuit-brake constants: a=3 m/s^2 + vertical arrival-sync) -- x four
cells (nominal, aim20, alt+3 pair, weave = the mechanism cell).

TWO PASSES, both printed:
  pose   the registered recipe ("ENGAGE fit + pose"): the pose-range channel
         supplied (RealFlightGuidance.supply_pose_range=True, the fixed
         Gazebo driver's PnP range). `mc._run_one` cannot set that, so this
         pass runs its own worker (build -> pose on -> run_engagement), the
         rehearsal_margin_sweep.py recipe.
  box    the same flights through `mc._run_one` unchanged (box-width range
         channel), for continuity with brake_shaping_ab.py's numbers.
`mc.run_many` hardcodes the default (dash) fit, so both passes load the
ENGAGE fit themselves and fan out under their own spawn-context pool.

Columns: med / p90 miss (m), % inside 0.35 m (contact) and 1.0 m, median
decodes, `med_vcl` = median CLOSING SPEED AT CPA (`closing_speed`; the P3
mechanism check), and `n_chg` = ON flights whose miss differs from the paired
OFF twin (how often the sprint acted at all -- it is inert unless the coast
latch fires).
"""
import contextlib
import dataclasses
import io
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from isim.replay_a0 import load_params
from isim.scenario import Scatter, Scenario
from isim import mc

N_SEEDS = 50
WORKERS = 10
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIT = os.path.join(_ROOT, "isim/fits/vehicle_gazebo_x500_engage.json")

BRAKE = {"brake_shaping": True, "brake_accel_ms2": 3.0, "brake_vert_sync": True}
ARMS = {
    "base_off": None,
    "base_on": {"commit_sprint": True},
    "brake_off": dict(BRAKE),
    "brake_on": dict(BRAKE, commit_sprint=True),
}
# ON arm -> its paired OFF twin (for n_chg).
TWIN = {"base_on": "base_off", "brake_on": "brake_off"}

# Rung R2 (tag_realism_v1 §F): target attitude + default wobble.
RUNG = dict(target_attitude=True)

CELLS = [
    ("nominal", dict()),
    ("aim20", dict(aim_error_deg=20.0)),
    ("alt+3", dict(target_alt_offset_m=3.0, cam_tilt_up_deg=10.0,
                   tag_mount_pitch_deg=12.0)),
    ("weave", dict(target_motion="weave")),
]


def _scenario(cell_name, arm_name, seed):
    return Scenario(concept="flyby", terminal="pursuit", tag_facing="rear",
                    scatter=Scatter(), port_pursuit_overrides=ARMS[arm_name],
                    seed=seed, **dict(CELLS)[cell_name], **RUNG)


def run_pose(job):
    """Registered recipe: pose-range channel on. Module-scope for spawn."""
    cell_name, arm_name, seed = job
    from isim.engine import run_engagement
    from isim.scenario import build
    scn = _scenario(cell_name, arm_name, seed)
    ecfg, vehicle, target, seeker, guidance, init = build(scn, load_params(FIT))
    guidance.supply_pose_range = True
    with contextlib.redirect_stdout(io.StringIO()):
        res = run_engagement(ecfg, vehicle, target, seeker, guidance, init)
    return dict(miss_m=float(res.miss_m), n_decoded=float(res.n_decoded),
                closing_speed=float(res.closing_speed_ms))


def run_box(job):
    """Continuity recipe: `mc._run_one` unchanged (box-width range)."""
    cell_name, arm_name, seed = job
    return mc._run_one(_scenario(cell_name, arm_name, seed), load_params(FIT))


def _table(ex, fn, label):
    jobs = [(c, a, s) for c, _ in CELLS for a in ARMS for s in range(N_SEEDS)]
    rows = list(ex.map(fn, jobs, chunksize=2))
    by = {j: r for j, r in zip(jobs, rows)}
    print(f"\n=== pass: {label} -- R2, ENGAGE fit, n={N_SEEDS} paired seeds/cell ===",
          flush=True)
    print(f"{'cell':>8} {'arm':>9} {'med':>7} {'p90':>7} {'<=0.35':>7} "
          f"{'<=1.0':>6} {'med_dec':>8} {'med_vcl':>8} {'n_chg':>6}", flush=True)
    for cell_name, _kw in CELLS:
        miss_by_arm = {}
        for arm_name in ARMS:
            arm = [by[(cell_name, arm_name, s)] for s in range(N_SEEDS)
                   if (cell_name, arm_name, s) in by]
            if len(arm) != N_SEEDS:
                print(f"{cell_name:>8} {arm_name:>9}  UNCERTAIN -- {len(arm)} rows",
                      flush=True)
                continue
            miss = np.array([float(r["miss_m"]) for r in arm])
            miss_by_arm[arm_name] = miss
            dec = np.median([float(r["n_decoded"]) for r in arm])
            vcl = np.median([float(r["closing_speed"]) for r in arm])
            twin = TWIN.get(arm_name)
            n_chg = (f"{int(np.sum(miss != miss_by_arm[twin])):6d}"
                     if twin in miss_by_arm else f"{'-':>6}")
            print(f"{cell_name:>8} {arm_name:>9} {np.median(miss):7.3f} "
                  f"{np.percentile(miss, 90):7.3f} "
                  f"{100 * np.mean(miss <= 0.35):6.0f}% "
                  f"{100 * np.mean(miss <= 1.0):5.0f}% {dec:8.0f} {vcl:8.2f} {n_chg}",
                  flush=True)
        print(flush=True)


def main():
    ctx = get_context("spawn")
    with ProcessPoolExecutor(max_workers=WORKERS, mp_context=ctx) as ex:
        _table(ex, run_pose, "pose (registered recipe: pose-range channel on)")
        _table(ex, run_box, "box (mc._run_one, box-width range; continuity)")


if __name__ == "__main__":
    main()
