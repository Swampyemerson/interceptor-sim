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
import contextlib
import csv
import dataclasses
import io
import itertools
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from isim.engine import run_engagement
from isim.replay_a0 import load_params
from isim.scenario import Scatter, Scenario, build
from isim.vehicle import VehicleParams

# Scenario fields carried straight through into every result dict.
_SCENARIO_FIELDS = tuple(f.name for f in dataclasses.fields(Scenario))

# The flight code's own "[seeker] FAULT ..." (and similar) stdout chatter --
# see `flight.deploy.seeker_loop._emit_fault` -- fires on ~every engagement
# with any scatter on it and would otherwise flood a batch's console. It is
# COUNTED, never silently dropped (no-silent-failure): `n_fault_lines` on the
# result row.
_FAULT_LINE_MARKER = "FAULT"


class _DebugRecorder:
    """Wraps a `concept="pursuit"` Guidance and snapshots its `.debug`
    (isim.concepts.PursuitDebug) after every `step()` call -- installed only
    inside `_run_one`, in the SAME process that calls `run_engagement`, so
    the recording never crosses a `multiprocessing` boundary (only the
    plain-dict return value of `_run_one` does). Read-only pass-through: it
    never alters the command `step()` returns, and every OTHER attribute
    (`state_log`, `last_decision`, ...) resolves straight to the wrapped
    guidance via `__getattr__` -- pursuit_concept_v2.md #5's diagnostics."""

    def __init__(self, inner: Any) -> None:
        self.inner = inner
        self.log: List[Any] = []   # (t, phase, r_est copy, v_t_est copy)

    def reset(self) -> None:
        self.log = []
        self.inner.reset()

    def step(self, t: float, own: Any, det: Any) -> Any:
        cmd = self.inner.step(t, own, det)
        dbg = self.inner.debug
        self.log.append((t, dbg.phase, np.array(dbg.r_est, dtype=np.float64),
                         np.array(dbg.v_t_est, dtype=np.float64),
                         float(getattr(dbg, "last_decode_t", -np.inf))))
        return cmd

    def __getattr__(self, name: str) -> Any:
        return getattr(self.inner, name)


_ATTRIBUTION_CATEGORIES = ("never_engaged", "no_tag_last_second", "saturated",
                          "estimate", "control")
# Diagnostic dict fields carried into the row twice (at CPA, and at
# CPA-1s -- suffix "_m1s"); "attribution" itself is computed at CPA only.
# (diag-dict key, row-column base name) -- "phase" -> "phase_at_cpa"/
# "phase_at_cpa_m1s" to read unambiguously in a CSV alongside "attribution".
# v3 #"Also": `frac_saturated_last_2s` (the vehicle's any-limit-active flag,
# true almost always -- see the report) is REPLACED by
# `accel_exceed_frac_last_1s`, a real test on commanded vs delivered accel.
_DIAG_ROW_FIELDS = (
    ("estimator_pos_err_m", "estimator_pos_err_m"),
    ("estimator_vel_err_m", "estimator_vel_err_m"),
    ("control_err_m", "control_err_m"),
    ("tag_in_frame", "tag_in_frame"),
    ("decodes_last_1s", "decodes_last_1s"),
    ("accel_exceed_frac_last_1s", "accel_exceed_frac_last_1s"),
    ("phase", "phase_at_cpa"),
)
# v3 #1: own-state-history window a real Pi/autopilot pairing would use to
# convert a Detection's t_capture -> own state; mirrors
# isim.concepts.PursuitConfig.own_state_history_s so the accel-series stride
# lines up with the engine's own guidance_dt (passed in via `ecfg`, not
# hardcoded, so a non-default EngagementConfig still gets a correct stride).


def _nearest_idx(sorted_arr: np.ndarray, value: float) -> int:
    """Index of the entry in a SORTED array nearest `value` (ties -> lower
    index); clamps to the array's bounds rather than raising on an
    out-of-range `value`."""
    if len(sorted_arr) == 0:
        return 0
    idx = int(np.searchsorted(sorted_arr, value))
    if idx <= 0:
        return 0
    if idx >= len(sorted_arr):
        return len(sorted_arr) - 1
    return idx if (sorted_arr[idx] - value) < (value - sorted_arr[idx - 1]) else idx - 1


def _accel_series(result: Any, ecfg: Any) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """(t, commanded_accel_mag, delivered_accel_mag) at the trace's native
    grid, per pursuit_concept_v2.md's "Also" (v3): a real saturation test
    needs commanded vs DELIVERED acceleration, not the vehicle's any-limit
    `saturated` flag (which is true whenever ANY slew limit is active on
    ANY axis, which this concept's Phase-B slew limits make true almost
    always -- see the report). Both are the discrete derivative of the
    `cmd`/`own_vel` trace columns over one `guidance_dt` (the cmd only
    CHANGES every guidance tick, so differencing at the engine's finer `dt`
    would mostly measure zero, then spike -- not a real rate)."""
    trace = result.trace
    if trace is None:
        return None
    dt = ecfg.dt
    stride = max(1, round(ecfg.guidance_dt / dt))
    t = trace["t"]
    if len(t) <= stride:
        return t, np.zeros(len(t)), np.zeros(len(t))
    cmd_v = trace["cmd"][:, :3]
    own_v = trace["own_vel"]
    dt_stride = stride * dt
    cmd_accel = np.zeros(len(t))
    act_accel = np.zeros(len(t))
    cmd_accel[stride:] = np.linalg.norm(cmd_v[stride:] - cmd_v[:-stride], axis=1) / dt_stride
    act_accel[stride:] = np.linalg.norm(own_v[stride:] - own_v[:-stride], axis=1) / dt_stride
    cmd_accel[:stride] = cmd_accel[stride] if len(t) > stride else 0.0
    act_accel[:stride] = act_accel[stride] if len(t) > stride else 0.0
    return t, cmd_accel, act_accel


def _diagnose(result: Any, debug_log: List[Any], t_query: float,
              accel_series: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]) -> Dict[str, Any]:
    """One diagnostic snapshot near `t_query` (typically `t_cpa` or
    `t_cpa - 1.0`), per pursuit_concept_v2.md #5 (extended by v3). Truth
    (`own_pos`/`tgt_pos`/`tgt_vel`) comes ONLY from `result.trace`/
    `result.frame_reports` -- data the ENGINE already collects for scoring,
    read here AFTER the run ends; `debug_log` never contains anything
    guidance was not itself handed."""
    empty = {"estimator_pos_err_m": None, "estimator_vel_err_m": None,
            "control_err_m": None, "tag_in_frame": None, "decodes_last_1s": 0,
            "accel_exceed_frac_last_1s": None, "phase": None, "last_decode_t": None}
    if not debug_log or result.trace is None:
        return empty
    debug_ts = np.array([d[0] for d in debug_log])
    d_idx = _nearest_idx(debug_ts, t_query)
    t_d, phase, r_est, v_t_est, last_decode_t = debug_log[d_idx]

    trace = result.trace
    t_idx = _nearest_idx(trace["t"], t_d)
    r_true = trace["tgt_pos"][t_idx] - trace["own_pos"][t_idx]
    v_true = trace["tgt_vel"][t_idx]

    decodes_last_1s = sum(1 for rep in result.frame_reports
                          if rep.decoded and (t_d - 1.0) <= rep.t_capture <= t_d)
    last_rep = None
    for rep in result.frame_reports:
        if rep.t_capture > t_d:
            break
        last_rep = rep
    tag_in_frame = bool(last_rep.in_fov) if last_rep is not None else False

    accel_exceed_frac_last_1s = None
    if accel_series is not None:
        t_arr, cmd_accel, act_accel = accel_series
        window = (t_arr >= t_d - 1.0) & (t_arr <= t_d)
        if np.any(window):
            accel_exceed_frac_last_1s = float(np.mean(cmd_accel[window] > act_accel[window]))

    return {
        "estimator_pos_err_m": float(np.linalg.norm(r_est - r_true)),
        "estimator_vel_err_m": float(np.linalg.norm(v_t_est - v_true)),
        "control_err_m": float(np.linalg.norm(r_est)),
        "tag_in_frame": tag_in_frame,
        "decodes_last_1s": decodes_last_1s,
        "accel_exceed_frac_last_1s": accel_exceed_frac_last_1s,
        "phase": phase,
        "last_decode_t": (None if not math.isfinite(last_decode_t) else last_decode_t),
    }


def _attribute_miss(diag: Dict[str, Any], miss_m: float) -> str:
    """Attribute a run's miss to the largest of five contributors, per
    pursuit_concept_v2.md #5, REVISED by v3's "Also": `never_engaged` is
    still checked first (phase != "B" is the cleanest possible story), then
    `no_tag_last_second` fires ONLY when decodes in the last second is
    exactly zero (a dead sensor, not a judgment call) -- otherwise
    `estimate`/`control`/`saturated` all compete on magnitude together.
    `estimate` = `|r_est - r_true|` (the ESTIMATE's own error); `control` =
    `|r_est|` (the residual the CONTROL law itself still believed existed --
    reading the original spec's parenthetical "control error (|r_true| that
    the estimate says should be zero)" as `|r_est|`, a judgment call kept
    from v2); `saturated` scores `miss_m` when the REAL accel-exceedance
    test (`accel_exceed_frac_last_1s >= 0.5`) fires, else 0 -- so it only
    wins the argmax when it fires AND the true miss is at least as large as
    the estimate/control residuals."""
    if diag["phase"] is None or diag["phase"] != "B":
        return "never_engaged"
    if diag["decodes_last_1s"] == 0:
        return "no_tag_last_second"
    saturated_score = 0.0
    if (diag["accel_exceed_frac_last_1s"] is not None
            and diag["accel_exceed_frac_last_1s"] >= 0.5):
        saturated_score = miss_m
    scores = {"estimate": diag["estimator_pos_err_m"] or 0.0,
             "control": diag["control_err_m"] or 0.0,
             "saturated": saturated_score}
    return max(scores, key=scores.get)


def attribution_shares(rows: List[Dict[str, Any]]) -> Dict[str, float]:
    """Percentage share of each attribution category, over the rows that
    HAVE one (i.e. `concept="pursuit"` rows) -- pursuit_concept_v2.md #5:
    "print the attribution shares per sweep cell next to the miss numbers".
    `{}` (no-vacuous-verdicts) when no row in the group carries one."""
    labelled = [r["attribution"] for r in rows if r.get("attribution") is not None]
    if not labelled:
        return {}
    n = len(labelled)
    return {cat: 100.0 * labelled.count(cat) / n for cat in _ATTRIBUTION_CATEGORIES}


def _run_one(scn: Scenario, vehicle_params: VehicleParams) -> Dict[str, Any]:
    """Run one engagement and flatten it to a plain, picklable dict. Top-level
    (module-scope) so a `spawn`-context worker process can import and call it
    -- a closure or bound method cannot be pickled for `spawn`."""
    ecfg, vehicle, target, seeker, guidance, init_state = build(scn, vehicle_params)

    is_pursuit = scn.concept == "pursuit"
    recorder = _DebugRecorder(guidance) if is_pursuit else None
    run_guidance = recorder if recorder is not None else guidance

    captured = io.StringIO()
    with contextlib.redirect_stdout(captured):
        # record_trace=True only for pursuit: the diagnostics (#5) need the
        # truth trace; flyby doesn't use it, so it stays off there (cost).
        result = run_engagement(ecfg, vehicle, target, seeker, run_guidance, init_state,
                                record_trace=is_pursuit)
    n_fault_lines = sum(1 for line in captured.getvalue().splitlines()
                        if _FAULT_LINE_MARKER in line)

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
        n_fault_lines=n_fault_lines,
    )

    if is_pursuit:
        accel_series = _accel_series(result, ecfg)
        diag_cpa = _diagnose(result, recorder.log, result.t_cpa, accel_series)
        diag_m1s = _diagnose(result, recorder.log, result.t_cpa - 1.0, accel_series)
        row["attribution"] = _attribute_miss(diag_cpa, result.miss_m)
        for diag_key, col in _DIAG_ROW_FIELDS:
            row[col] = diag_cpa[diag_key]
            row[col + "_m1s"] = diag_m1s[diag_key]
        # v4 #4: ALSO query at the last decode before CPA (not just at CPA
        # itself) -- resolves the v3 oddity (the worst estimator errors AT
        # CPA landed on the SMALLEST true misses): "error at CPA" mixes in
        # whatever the estimate drifted to during a post-decode COAST, which
        # is a different, less meaningful number than "how good was the
        # estimate the last time it was actually fed a detection". None when
        # the run never decoded at all (no_tag_last_second/never_engaged).
        if diag_cpa["last_decode_t"] is not None:
            diag_last_decode = _diagnose(result, recorder.log, diag_cpa["last_decode_t"],
                                         accel_series)
            row["estimator_pos_err_m_at_last_decode"] = diag_last_decode["estimator_pos_err_m"]
            row["estimator_vel_err_m_at_last_decode"] = diag_last_decode["estimator_vel_err_m"]
        else:
            row["estimator_pos_err_m_at_last_decode"] = None
            row["estimator_vel_err_m_at_last_decode"] = None
    else:
        row["attribution"] = None
        for _diag_key, col in _DIAG_ROW_FIELDS:
            row[col] = None
            row[col + "_m1s"] = None
        row["estimator_pos_err_m_at_last_decode"] = None
        row["estimator_vel_err_m_at_last_decode"] = None
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

_MISS_HIT_M = 0.35        # the binary-kill proximity radius this project targets
_MISS_HIT_LOOSE_M = 1.0   # a looser reference radius, for scatter's wider tails

_TABLE_COLS = ("value", "n", "med_miss_m", "p10_miss_m", "p90_miss_m", "pct_hit",
              "pct_hit_loose", "med_horiz_m", "med_vert_m", "pct_engage", "med_decoded")


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
        "p10_miss_m": float(np.percentile(miss, 10)),
        "p90_miss_m": float(np.percentile(miss, 90)),
        "pct_hit": float(np.mean(miss <= _MISS_HIT_M) * 100.0),
        "pct_hit_loose": float(np.mean(miss <= _MISS_HIT_LOOSE_M) * 100.0),
        "med_horiz_m": float(np.median(horiz)),
        "med_vert_m": float(np.median(vert)),
        "pct_engage": float(np.mean(engage) * 100.0),
        "med_decoded": float(np.median(decoded)),
    }


def _print_table(axis_name: str, rows: List[Dict[str, Any]], values: Sequence[float]) -> None:
    hit_col = f"%<={_MISS_HIT_M:.2f}m"
    hit_loose_col = f"%<={_MISS_HIT_LOOSE_M:.2f}m"
    header = (f"{'value':>10s} {'n':>4s} {'med_miss':>9s} {'p10_miss':>9s} {'p90_miss':>9s} "
             f"{hit_col:>9s} {hit_loose_col:>9s} {'med_|h|':>8s} {'med_v':>7s} "
             f"{'%engage':>8s} {'med_dec':>8s}")
    print(f"axis={axis_name}")
    print(header)
    for v in values:
        group = [r for r in rows if r["_axis_value"] == v]
        s = _summarize(group)
        if s["verdict"] == "UNCERTAIN":
            print(f"{v!s:>10s} {0:>4d}  UNCERTAIN -- zero engagements, no verdict")
            continue
        print(f"{v!s:>10s} {s['n']:>4d} {s['med_miss_m']:>9.3f} {s['p10_miss_m']:>9.3f} "
              f"{s['p90_miss_m']:>9.3f} {s['pct_hit']:>8.1f}% {s['pct_hit_loose']:>8.1f}% "
              f"{s['med_horiz_m']:>8.3f} {s['med_vert_m']:>7.3f} "
              f"{s['pct_engage']:>7.1f}% {s['med_decoded']:>8.1f}")


def _print_attribution(axis_name: str, rows: List[Dict[str, Any]],
                       values: Sequence[float]) -> None:
    """Attribution shares per axis value, next to the miss numbers --
    pursuit_concept_v2.md #5. A no-op (prints nothing) when no row in `rows`
    carries an attribution (e.g. `concept="flyby"`, no-vacuous-verdicts)."""
    if not any(r.get("attribution") is not None for r in rows):
        return
    header = f"{'value':>10s} " + " ".join(f"{c:>11s}" for c in _ATTRIBUTION_CATEGORIES)
    print(f"attribution shares, axis={axis_name}")
    print(header)
    for v in values:
        group = [r for r in rows if r["_axis_value"] == v]
        shares = attribution_shares(group)
        if not shares:
            print(f"{v!s:>10s}  UNCERTAIN -- no attributed rows")
            continue
        print(f"{v!s:>10s} " + " ".join(f"{shares[c]:>10.1f}%" for c in _ATTRIBUTION_CATEGORIES))


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


def _base_scenario_from_args(args: argparse.Namespace) -> Scenario:
    """The `--scatter`/`--terminal`/`--cam-tilt` pass-throughs, common to both
    `sweep` and `requirement`. `--scatter` uses `Scatter()`'s own defaults --
    there is no per-field CLI override; edit `Scatter` (and its source notes)
    if a different magnitude is wanted."""
    return Scenario(
        scatter=Scatter() if args.scatter else None,
        terminal=args.terminal,
        cam_tilt_up_deg=args.cam_tilt,
        concept=args.concept,
        tag_facing=args.tag_facing,
    )


def _add_scatter_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--scatter", action="store_true",
                   help="apply Scatter() defaults' run-to-run noise to every scenario")
    p.add_argument("--terminal", type=str, default="stock",
                   help="passed to RealFlightGuidance(terminal=...) when != 'stock'")
    p.add_argument("--cam-tilt", type=float, default=0.0, dest="cam_tilt",
                   help="deg, nominal camera mount tilt (Scenario.cam_tilt_up_deg)")
    p.add_argument("--concept", type=str, default="flyby",
                   help="Scenario.concept: 'flyby' (default) or 'pursuit'")
    p.add_argument("--tag-facing", type=str, default="camera", dest="tag_facing",
                   help="Scenario.tag_facing: 'camera' (default), 'rear', or 'side'")


def _cmd_sweep(args: argparse.Namespace) -> int:
    values = _parse_values(args.values)
    base = _base_scenario_from_args(args)
    rows = sweep(args.axis, values, args.n, base=base, workers=args.workers)
    _print_table(args.axis, rows, values)
    _print_attribution(args.axis, rows, values)
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
    base = _base_scenario_from_args(args)
    all_rows: List[Dict[str, Any]] = []
    any_engagement = False
    for axis, values in _REQUIREMENT_AXES:
        rows = sweep(axis, values, args.n, base=base, workers=args.workers)
        for r in rows:
            r["_axis_name"] = axis
        _print_table(axis, rows, values)
        _print_attribution(axis, rows, values)
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
    _add_scatter_args(p_sweep)
    p_sweep.set_defaults(func=_cmd_sweep)

    p_req = sub.add_parser("requirement", help="run the four requirement axes, write one CSV")
    p_req.add_argument("--out", required=True, help="CSV output path")
    p_req.add_argument("--n", type=int, default=50, help="seeds per value")
    p_req.add_argument("--workers", type=int, default=None)
    _add_scatter_args(p_req)
    p_req.set_defaults(func=_cmd_requirement)

    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
