"""Monte-Carlo runner over `isim.scenario.Scenario` (ADR-0102 isim).

`run_many` fans a list of scenarios out across a `multiprocessing` pool (or
runs them in-process at `workers<=1`); `sweep` builds one axis of scenarios at
several values x several seeds and prints a summary table per value;
`requirement` runs four such sweeps (aim error, altitude offset, target
speed, sprint scale) and writes every row to one CSV.

Lab-vs-Gazebo honesty note: this is a fast KINEMATIC/vehicle-model surrogate
(`isim`), not Gazebo -- its numbers rank scenarios and locate trouble, they do
not stand in for a Gazebo gate.
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import itertools
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from isim.engine import run_engagement
from isim.replay_a0 import load_params
from isim.scenario import Scenario, build
from isim.vehicle import VehicleParams

# Scenario fields carried straight through into every result dict.
_SCENARIO_FIELDS = tuple(f.name for f in dataclasses.fields(Scenario))


def _run_one(scn: Scenario, vehicle_params: VehicleParams) -> Dict[str, Any]:
    """Run one engagement and flatten it to a plain, picklable dict. Top-level
    (module-scope) so a `spawn`-context worker process can import and call it
    -- a closure or bound method cannot be pickled for `spawn`."""
    ecfg, vehicle, target, seeker, guidance, init_state = build(scn, vehicle_params)
    result = run_engagement(ecfg, vehicle, target, seeker, guidance, init_state,
                            record_trace=False)

    state_log = list(guidance.state_log)
    reached_engage = any(s == "ENGAGE" for _, s in state_log)
    final_state = guidance.last_decision.state if guidance.last_decision is not None else None
    transitions = "|".join(f"{s}@{t:.2f}" for t, s in state_log)

    row: Dict[str, Any] = {f: getattr(scn, f) for f in _SCENARIO_FIELDS}
    row.update(
        miss_m=result.miss_m,
        miss_horiz_m=result.miss_horiz_m,
        miss_vert_m=result.miss_vert_m,
        t_cpa=result.t_cpa,
        closing_speed=result.closing_speed_ms,
        n_frames=result.n_frames,
        n_in_fov=result.n_in_fov,
        n_decoded=result.n_decoded,
        frac_saturated=result.frac_saturated,
        final_state=final_state,
        reached_engage=reached_engage,
        transitions=transitions,
    )
    return row


def run_many(scenarios: Sequence[Scenario],
             workers: Optional[int] = None) -> List[Dict[str, Any]]:
    """Run every scenario, deterministic per `scn.seed` and independent of
    `workers`: each engagement seeds its own `np.random.default_rng`, so the
    result for a given seed never depends on which worker (or none) ran it."""
    if workers is None:
        workers = os.cpu_count() or 1
    vehicle_params = load_params()
    scenarios = list(scenarios)
    if not scenarios:
        return []
    if workers <= 1:
        return [_run_one(s, vehicle_params) for s in scenarios]
    ctx = get_context("spawn")
    with ProcessPoolExecutor(max_workers=workers, mp_context=ctx) as ex:
        return list(ex.map(_run_one, scenarios, itertools.repeat(vehicle_params)))


def sweep(axis_name: str, values: Sequence[float], n_seeds: int,
          base: Optional[Scenario] = None,
          workers: Optional[int] = None) -> List[Dict[str, Any]]:
    """One value of `axis_name` at a time, `n_seeds` seeds each (0..n_seeds-1),
    all overridden on `base` (default `Scenario()`). Returns a flat list of
    per-engagement dicts across every value -- `_summarize` groups them back
    by value for the table."""
    if base is None:
        base = Scenario()
    scenarios = [
        dataclasses.replace(base, **{axis_name: value, "seed": seed})
        for value in values
        for seed in range(n_seeds)
    ]
    rows = run_many(scenarios, workers=workers)
    for row in rows:
        row["_axis_value"] = row[axis_name]
    return rows


# --------------------------------------------------------------- summarizing

_MISS_HIT_M = 0.35   # the binary-kill proximity radius this project targets

_TABLE_COLS = ("value", "n", "med_miss_m", "p90_miss_m", "pct_hit", "med_horiz_m",
              "med_vert_m", "pct_engage", "med_decoded")


def _summarize(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    """One table row's stats from a list of per-engagement result dicts for a
    SINGLE axis value. `n == 0` is UNCERTAIN, never a silent pass -- there is
    no unit to compute a median or a percentage over (no-vacuous-verdicts)."""
    n = len(rows)
    if n == 0:
        return {"n": 0, "verdict": "UNCERTAIN"}
    miss = np.array([r["miss_m"] for r in rows], dtype=np.float64)
    horiz = np.array([r["miss_horiz_m"] for r in rows], dtype=np.float64)
    vert = np.array([r["miss_vert_m"] for r in rows], dtype=np.float64)
    decoded = np.array([r["n_decoded"] for r in rows], dtype=np.float64)
    engage = np.array([bool(r["reached_engage"]) for r in rows], dtype=np.float64)
    return {
        "n": n,
        "verdict": "ok",
        "med_miss_m": float(np.median(miss)),
        "p90_miss_m": float(np.percentile(miss, 90)),
        "pct_hit": float(np.mean(miss <= _MISS_HIT_M) * 100.0),
        "med_horiz_m": float(np.median(horiz)),
        "med_vert_m": float(np.median(vert)),
        "pct_engage": float(np.mean(engage) * 100.0),
        "med_decoded": float(np.median(decoded)),
    }


def _print_table(axis_name: str, rows: List[Dict[str, Any]], values: Sequence[float]) -> None:
    hit_col = f"%hit<={_MISS_HIT_M:.2f}m"
    header = (f"{'value':>10s} {'n':>4s} {'med_miss':>9s} {'p90_miss':>9s} "
             f"{hit_col:>11s} {'med_|h|':>8s} {'med_v':>7s} "
             f"{'%engage':>8s} {'med_dec':>8s}")
    print(f"axis={axis_name}")
    print(header)
    for v in values:
        group = [r for r in rows if r["_axis_value"] == v]
        s = _summarize(group)
        if s["verdict"] == "UNCERTAIN":
            print(f"{v!s:>10s} {0:>4d}  UNCERTAIN -- zero engagements, no verdict")
            continue
        print(f"{v!s:>10s} {s['n']:>4d} {s['med_miss_m']:>9.3f} {s['p90_miss_m']:>9.3f} "
              f"{s['pct_hit']:>10.1f}% {s['med_horiz_m']:>8.3f} {s['med_vert_m']:>7.3f} "
              f"{s['pct_engage']:>7.1f}% {s['med_decoded']:>8.1f}")


# --------------------------------------------------------------------- CLI

def _parse_values(spec: str) -> List[float]:
    return [float(v) for v in spec.split(",") if v.strip() != ""]


def _write_csv(path: str, rows: List[Dict[str, Any]]) -> None:
    if not rows:
        # FAIL-CLOSED: an empty CSV with no header would silently look like
        # "ran and found nothing" to a downstream reader. Say so instead.
        with open(path, "w") as f:
            f.write("# UNCERTAIN: zero rows, no engagements were run\n")
        return
    fields = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _cmd_sweep(args: argparse.Namespace) -> int:
    values = _parse_values(args.values)
    rows = sweep(args.axis, values, args.n, workers=args.workers)
    _print_table(args.axis, rows, values)
    if args.out:
        _write_csv(args.out, rows)
    return 0 if rows else 1


# The four axes `requirement` sweeps -- deliberately not configurable beyond
# `--n`/`--out`, so a run is always comparable to the last one.
_REQUIREMENT_AXES = (
    ("aim_error_deg", [0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0]),
    ("target_alt_offset_m", [-2.0, -1.0, 0.0, 1.0, 2.0, 3.0]),
    ("target_speed_ms", [0.0, 2.0, 4.0, 6.0, 9.0]),
    ("sprint_scale", [1.0, 0.75, 0.5, 0.25, 0.0]),
)


def _cmd_requirement(args: argparse.Namespace) -> int:
    all_rows: List[Dict[str, Any]] = []
    any_engagement = False
    for axis, values in _REQUIREMENT_AXES:
        rows = sweep(axis, values, args.n, workers=args.workers)
        for r in rows:
            r["_axis_name"] = axis
        _print_table(axis, rows, values)
        print()
        all_rows.extend(rows)
        any_engagement = any_engagement or any(r["reached_engage"] for r in rows)
    _write_csv(args.out, all_rows)
    print(f"wrote {len(all_rows)} rows to {args.out}")
    if not any_engagement:
        print("UNCERTAIN: zero engagements reached ENGAGE across all four axes")
        return 1
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="isim.mc", description="isim Monte-Carlo runner")
    sub = p.add_subparsers(dest="command", required=True)

    p_sweep = sub.add_parser("sweep", help="sweep one Scenario axis over several values/seeds")
    p_sweep.add_argument("--axis", required=True, help="Scenario field name to sweep")
    p_sweep.add_argument("--values", required=True, help="comma-separated float values")
    p_sweep.add_argument("--n", type=int, default=50, help="seeds per value")
    p_sweep.add_argument("--workers", type=int, default=None)
    p_sweep.add_argument("--out", type=str, default=None, help="optional CSV output path")
    p_sweep.set_defaults(func=_cmd_sweep)

    p_req = sub.add_parser("requirement", help="run the four requirement axes, write one CSV")
    p_req.add_argument("--out", required=True, help="CSV output path")
    p_req.add_argument("--n", type=int, default=50, help="seeds per value")
    p_req.add_argument("--workers", type=int, default=None)
    p_req.set_defaults(func=_cmd_requirement)

    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
