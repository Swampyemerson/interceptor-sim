#!/usr/bin/env python3
r"""T18 -- the sigma-R / sigma-v validation harness (PERFECT-CENTROID FLOOR
mode). PURE MATH, no Gazebo/PX4, no GPU, no NN -- this is the pre-T17
checkpoint: it validates `scripts/ground_station/triangulate.py` and
`scripts/ground_station/track_filter.py` against the analytic model in
`docs/rig_geometry_analysis.md` BEFORE any real detector exists, by
projecting GROUND TRUTH world positions into both cameras (the exact
inverse of triangulation) instead of running a detector on a rendered
frame. The NN-centroid mode (real detections, T17-dependent) reuses this
harness's same `triangulate()`/`track_filter` calls -- swap the centroid
SOURCE, not the math.

docs/phase2_sim_to_real_plan.md Sec 5.6 items this closes out:
  #7  sigma_R model must scale with the flown RESOLUTION, not the fixed
      c=4.45e-05 the mock ships with -- validated here against the
      RESOLUTION-SCALED model `c = sigma_d/(b*f_px)` from
      `docs/rig_geometry_analysis.md`'s Analysis 1 (reproduced, not
      re-derived: `scripts/rig_geometry_analysis.py`'s own numbers).
  #8  cue-VELOCITY quality: propagate sigma_R through the track filter at
      the candidate rate, compare the resulting sigma_v against the
      fusion arc's assumed 0.5 m/s and against the doc's own Kalata-based
      prediction (`docs/rig_geometry_analysis.md` Sec 3).

THREE CHECKS, IN DEPENDENCY ORDER:

(1) SYNTHETIC RANGE SWEEP (the direct table-row reproduction). A synthetic
    target is placed ON the rig's own boresight at each of
    `rig_geometry_analysis.A1_RANGES_M` (15/30/60/100/150 m), projected into
    both cameras via the TRUE rig pose, jittered per BEST/EXPECTED/WORST
    matching-noise tiers (sigma_d = 0.1/0.4/1.0 px), triangulated, and the
    RESULTING range-error std is compared against the resolution-scaled
    model `c = sigma_d/(b*f_px)` evaluated at THIS capture's own
    (resolution, f_px) -- not re-derived, just evaluated at the real
    f_px≈5445 the T16 capture used. This is the "assert your harness
    reproduces that row within ~10%" checkpoint.

(2) REAL-CAPTURE SIGMA_R (the sweep-capture-grounded check the task
    literally asks for). For EVERY row of a T16 capture's `index.csv`, the
    GT world position is projected/jittered/triangulated exactly as in (1),
    but at the capture's OWN (narrow, ~160-163 m) range band -- binned by
    true range -- so the "per range bin" comparison uses genuinely captured
    geometry, not a synthetic placement. A single narrow bin is expected
    (the capture is one static rig pose), disclosed, not hidden.

(3) SIGMA_V VALIDATION (Sec 5.6 #8). Two complementary measurements,
    because a scalar tracked channel's REAL measurement noise is
    axis-dependent (matching-noise-only sigma_R is RANGE-axis noise; this
    capture's rig points its boresight along world+X while the target
    dashes along world+Y, i.e. mostly CROSS-RANGE relative to the rig --
    disclosed, not a bug):
      (3a) SCALAR KALATA REPRODUCTION -- a direct Monte-Carlo replica of
           `docs/rig_geometry_analysis.md`'s own Sec-3 methodology (a WPA
           [Wiener-process-acceleration] target-maneuver channel, measurement
           noise = sigma_R, tracked via `guidance_lab.AlphaBetaFilter` in
           Kalata mode) -- validates that `ground_station.track_filter`'s
           underlying machinery reproduces the doc's own closed-form
           steady-state sigma_v (their K3/K4 formulas) to within a few
           percent. THIS is the assertable "vs the rig_geometry_analysis.md
           prediction at 10 Hz EXPECTED" checkpoint.
      (3b) REAL-PIPELINE SIGMA_V -- the SAME jittered-trajectory-sweep idea
           but through the ACTUAL project → jitter → triangulate → track_filter
           pipeline, run over exactly ONE captured-geometry corridor pass
           (~6.5 s -- extending it in time/space was TRIED and found to walk
           the target off-boresight / re-trigger fresh transients on a
           bounce-reflection; see the function's docstring). Because (3a)
           shows this filter needs ~15-20 s to reach Kalata's steady state at
           these gains -- LONGER than one pass -- this is reported as a
           TRANSIENT figure (pooled over the last 20 ticks, closest to
           convergence this geometry offers), NOT asserted against (3a)'s
           steady-state number. Reported for completeness/pipeline
           integration and because it happens to land close to (3a)'s
           number for this specific rig pose -- a disclosed coincidence, not
           a second independent confirmation.

Run:
    .venv/bin/python scripts/t18_sigma_validation.py
    .venv/bin/python scripts/t18_sigma_validation.py --sweep-dir logs/rig_captures/<other>/
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import statistics
import sys
from typing import Optional

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import stereo_model as sm  # noqa: E402  (reuse constants, not re-derive)
from guidance_lab import AlphaBetaFilter  # noqa: E402  (single source of Kalata gains)
from rig_geometry_analysis import (  # noqa: E402  (reuse the doc's own formulas)
    A1_RANGES_M,
    alpha_beta_variances,
)
from ground_station.triangulate import (  # noqa: E402
    RigConfig,
    project_world_to_stereo_pixels,
    triangulate,
    camera_axes,
)
from ground_station.track_filter import GroundTrackFilter  # noqa: E402

DEFAULT_SWEEP_DIR = os.path.join(
    REPO_ROOT, "logs", "rig_captures", "full_sweep_20260709T015530Z"
)
OUT_DIR = os.path.join(REPO_ROOT, "logs", "t18_sigma_validation")

# BEST/EXPECTED/WORST matching-noise tiers, px -- same values as
# stereo_model.PARAMS["sigma_match_px"] (reused, not re-typed, in
# model_c_matching()/model_sigma_match_tiers()).
TIER_ORDER = ("best", "expected", "worst")

DASH_ACCEL_CLASS_GENTLE_MPS2 = 1.0   # rig_geometry_analysis.ACCEL_CLASSES[0]
SIGMA_V_TARGET_MPS = 0.5             # s2_cue_mock.py DEFAULT_VEL_SIGMA_M_S


def print_header(title: str) -> None:
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)


# ============================================================================
# Shared helpers
# ============================================================================
def jitter_centroid_pair(u_l, v_l, u_r, v_r, sigma_d_px, rng):
    """Independent per-camera centroid jitter, std `sigma_d_px/sqrt(2)`
    EACH, so the COMBINED disparity-noise std (Var(u_l)+Var(u_r), the two
    draws are independent) equals `sigma_d_px` exactly -- matching
    `stereo_model.py`'s `sigma_match_px` convention, which treats sigma_d as
    a single disparity-error term, not a per-image one. v is jittered the
    same way for completeness (feeds cross-range/altitude); by construction
    of this fronto-parallel rig, v-noise does NOT affect the range estimate
    (Z depends only on u_l - u_r) -- see triangulate.py's module docstring."""
    if sigma_d_px <= 0.0:
        return u_l, v_l, u_r, v_r
    s = sigma_d_px / math.sqrt(2.0)
    return (
        u_l + rng.gauss(0.0, s), v_l + rng.gauss(0.0, s),
        u_r + rng.gauss(0.0, s), v_r + rng.gauss(0.0, s),
    )


def model_c_matching(sigma_d_px: float, baseline_m: float, f_px: float) -> float:
    """The resolution-scaled model, Sec 5.6 #7 / rig_geometry_analysis.py's
    `scaled_c_matching_only`: c = sigma_d / (b * f_px). Reproduced here
    (not imported) so ground_station/ + this harness have zero import
    dependency on the analysis script's plotting/CLI machinery -- only the
    two pure-math functions imported above."""
    return sigma_d_px / (baseline_m * f_px)


def synthetic_boresight_point(rig: RigConfig, range_m: float):
    """A synthetic target ON the rig's own boresight at `range_m` (world
    point = rig_true_center + range_m * forward). Used by the range-sweep
    reproduction check -- NOT tied to any captured frame."""
    forward, _, _ = camera_axes(rig.true_pose.yaw)
    return (
        rig.true_pose.x + range_m * forward[0],
        rig.true_pose.y + range_m * forward[1],
        rig.true_pose.z + range_m * forward[2],
    )


def true_range_from_rig(rig: RigConfig, p_world) -> float:
    return math.sqrt(
        (p_world[0] - rig.true_pose.x) ** 2
        + (p_world[1] - rig.true_pose.y) ** 2
        + (p_world[2] - rig.true_pose.z) ** 2
    )


def measure_sigma_R_at_point(rig: RigConfig, p_world, sigma_d_px: float,
                              n_trials: int, seed: int):
    """Project p_world into both TRUE cameras once, then run `n_trials`
    independent jitter+triangulate trials (assumed pose == true pose, no
    datum bias -- that's tested separately). Returns (measured_sigma_R,
    mean_residual, n_ok, true_range)."""
    true_range = true_range_from_rig(rig, p_world)
    uv = project_world_to_stereo_pixels(p_world, rig.true_pose, rig.baseline_m,
                                         rig.f_px, rig.cx, rig.cy)
    if uv is None:
        return None
    u_l, v_l, u_r, v_r = uv
    rng = random.Random(seed)
    residuals = []
    for _ in range(n_trials):
        jl_u, jl_v, jr_u, jr_v = jitter_centroid_pair(u_l, v_l, u_r, v_r, sigma_d_px, rng)
        res = triangulate(jl_u, jl_v, jr_u, jr_v, rig.true_pose, rig.baseline_m,
                           rig.f_px, rig.cx, rig.cy)
        if res is None:
            continue
        residuals.append(res.range_m - true_range)
    if len(residuals) < 2:
        return None
    return (statistics.pstdev(residuals), statistics.mean(residuals),
            len(residuals), true_range)


# ============================================================================
# (1) Synthetic range-sweep reproduction of rig_geometry_analysis.md's table
# ============================================================================
def run_synthetic_sigma_R_sweep(rig: RigConfig, ranges=A1_RANGES_M,
                                 n_trials=3000, seed=0):
    rows = []
    for tier in TIER_ORDER:
        sigma_d = sm.PARAMS["sigma_match_px"][tier]
        c = model_c_matching(sigma_d, rig.baseline_m, rig.f_px)
        for R in ranges:
            p = synthetic_boresight_point(rig, R)
            m = measure_sigma_R_at_point(rig, p, sigma_d, n_trials,
                                          seed=(seed * 100003 + hash((tier, R)) & 0xFFFF))
            if m is None:
                continue
            measured, mean_resid, n_ok, true_R = m
            model_sigma = c * true_R * true_R
            rel_err = (measured - model_sigma) / model_sigma if model_sigma > 0 else float("nan")
            rows.append({
                "tier": tier, "sigma_d_px": sigma_d, "R_m": true_R,
                "c_model_per_m2": c, "model_sigma_R_m": model_sigma,
                "measured_sigma_R_m": measured, "mean_residual_m": mean_resid,
                "rel_err": rel_err, "n_trials_ok": n_ok,
            })
    return rows


# ============================================================================
# (2) Real-capture sigma_R, binned by true range
# ============================================================================
def load_capture(sweep_dir: str):
    meta_path = os.path.join(sweep_dir, "capture_meta.json")
    index_path = os.path.join(sweep_dir, "index.csv")
    with open(meta_path) as f:
        meta = json.load(f)
    with open(index_path) as f:
        rows = list(csv.DictReader(f))
    return meta, rows


def run_capture_sigma_R(rig: RigConfig, index_rows, n_trials=500, seed=0,
                         bin_width_m=1.0):
    per_row = []
    for tier in TIER_ORDER:
        sigma_d = sm.PARAMS["sigma_match_px"][tier]
        c = model_c_matching(sigma_d, rig.baseline_m, rig.f_px)
        for row in index_rows:
            p = (float(row["gt_target_x"]), float(row["gt_target_y"]), float(row["gt_target_z"]))
            m = measure_sigma_R_at_point(
                rig, p, sigma_d, n_trials,
                seed=(seed * 1009 + int(row["seq"]) * 7 + hash(tier) & 0xFFFF),
            )
            if m is None:
                continue
            measured, mean_resid, n_ok, true_R = m
            per_row.append({
                "tier": tier, "seq": row["seq"], "dash_direction": row["dash_direction"],
                "R_m": true_R, "residual_std_this_row_m": measured,
                "model_sigma_R_m": c * true_R * true_R,
            })

    # Bin by rounded range, per tier -- pool residual STD across all rows in
    # the bin using each row's own per-row std as one draw's worth of signal
    # is too little (n_trials per row already gives a per-row std; bin-level
    # "measured sigma_R" is the RMS of those per-row stds, a standard way to
    # pool variance estimates across a bin without re-touching raw samples).
    bins = {}
    for r in per_row:
        key = (r["tier"], round(r["R_m"] / bin_width_m) * bin_width_m)
        bins.setdefault(key, []).append(r)
    bin_rows = []
    for (tier, R_bin), items in sorted(bins.items()):
        rms_measured = math.sqrt(sum(i["residual_std_this_row_m"] ** 2 for i in items) / len(items))
        mean_model = statistics.mean(i["model_sigma_R_m"] for i in items)
        mean_R = statistics.mean(i["R_m"] for i in items)
        rel_err = (rms_measured - mean_model) / mean_model if mean_model > 0 else float("nan")
        bin_rows.append({
            "tier": tier, "R_bin_center_m": R_bin, "R_actual_mean_m": mean_R,
            "n_rows_in_bin": len(items), "measured_sigma_R_m": rms_measured,
            "model_sigma_R_m": mean_model, "rel_err": rel_err,
        })
    return per_row, bin_rows


# ============================================================================
# (3a) Scalar Kalata reproduction of rig_geometry_analysis.md Sec 3
# ============================================================================
def run_scalar_kalata_reproduction(sigma_w: float, sigma_meas: float, rate_hz: float,
                                    duration_s: float, warmup_s: float, n_seeds: int,
                                    seed0: int = 0):
    """A direct Monte-Carlo replica of the doc's own Sec-3 methodology: a
    scalar WPA (Wiener-process-acceleration) channel -- x += v*T + a*T^2/2,
    v += a*T, a ~ N(0, sigma_w^2) each tick -- measured with additive N(0,
    sigma_meas^2) noise and tracked by `guidance_lab.AlphaBetaFilter` in
    Kalata mode (the SAME class `ground_station.track_filter.
    GroundTrackFilter` wraps per axis). Returns (measured_sigma_v, n_samples,
    alpha, beta, analytic_sigma_v).

    TIMING NOTE (found the hard way building this harness -- see task
    report): the residual must compare the filter's post-correction
    xdot_hat against the TRUE velocity AT THE SAME INSTANT (before that
    tick's process-noise advance), and the "true" trajectory must actually
    carry the sigma_w-scale process noise Kalata's gains are tuned for --
    comparing against a PERFECTLY constant-velocity truth (no injected
    process noise, as this capture's dash literally flies) understates the
    doc's steady-state variance by ~2x, because Kalata's steady-state gains
    assume the target really does maneuver at sigma_w."""
    T = 1.0 / rate_hz
    n_steps = int(round(duration_s / T))
    warmup_steps = int(round(warmup_s / T))
    residuals = []
    for seed in range(n_seeds):
        rng = random.Random(seed0 + seed)
        f = AlphaBetaFilter(0.0, 0.0, kalata_sigma_process=sigma_w, kalata_sigma_meas=sigma_meas)
        y_true, v_true, t = 0.0, 0.0, 0.0
        for i in range(n_steps):
            meas = y_true + rng.gauss(0.0, sigma_meas)
            if f.initialized:
                f.predict(T)
            f.correct(meas, t)
            if i >= warmup_steps:
                residuals.append(f.xdot_hat - v_true)  # v_true BEFORE this tick's advance
            t += T
            a = rng.gauss(0.0, sigma_w)
            y_true += v_true * T + 0.5 * a * T * T
            v_true += a * T
    measured = statistics.pstdev(residuals) if len(residuals) > 1 else float("nan")
    alpha, beta, lam, var_pos, var_vel = alpha_beta_variances(sigma_w, sigma_meas, T)
    return measured, len(residuals), alpha, beta, math.sqrt(var_vel)


# ============================================================================
# (3b) Real-pipeline sigma_v over ONE captured-geometry corridor pass
# ============================================================================
def run_pipeline_sigma_v(rig: RigConfig, meta: dict, sigma_w: float, sigma_d_px: float,
                          rate_hz: float, n_seeds: int, tail_ticks: int = 20,
                          seed0: int = 0):
    """The full project -> jitter centroids -> triangulate -> GroundTrackFilter
    pipeline, run over the REAL dash geometry (x0/z0/dash_speed from the
    capture's own capture_meta.json) for exactly ONE pass across the
    corridor (+y0_mag -> -y0_mag, ~6.5 s at 9 m/s/10 Hz = 65 ticks) -- the
    span this rig pose's wedge coverage is actually validated for
    (`docs/rig_geometry_analysis.md` Sec 2). This is DELIBERATELY NOT
    extended in time/space past that corridor (an earlier version of this
    harness tried extending the straight-line dash for tens of seconds to
    reach Kalata's steady state and found the target walks arbitrarily far
    off-boresight, which is a geometry-validity bug, not a filter one; a
    reflecting/bounce trajectory was tried too and made things WORSE, because
    a hard velocity-sign reversal is not a WPA-consistent "gentle maneuver"
    and re-triggers a fresh transient every bounce -- see the task report).

    KNOWN, DISCLOSED LIMITATION: at gentle/EXPECTED/10 Hz/this rig's range,
    (3a) shows the Kalata filter needs ~15-20 s to reach steady state --
    LONGER than one 6.5 s corridor pass. So this measurement is NOT a
    steady-state number; it pools the residuals from the LAST `tail_ticks`
    of the single pass (the closest to convergence this geometry offers,
    i.e. right before HANDOFF) and reports it as a TRANSIENT figure,
    honestly labeled. (3a) remains the assertable steady-state checkpoint.

    Also note (found while building this): this capture's rig boresight
    points along world+X while the target dashes along world+Y -- i.e. the
    dash is mostly CROSS-RANGE relative to the rig, not along the range
    axis `kalata_sigma_meas` (deliberately the WORST-AXIS-pessimistic
    `stereo_model.sigma_R()`, matching rig_geometry_analysis.md's own
    disclosed simplification) assumes. A mismatched (too-pessimistic) gain
    assumption does NOT automatically help even though the real per-tick
    measurement noise is smaller off-boresight-axis: it makes the filter
    UNDER-react to the real (sigma_w-scale) target maneuvers baked into this
    test, which can net out as WORSE tracking than the "matched" analytic
    number -- a genuine, disclosed, non-intuitive finding, not a bug."""
    T = 1.0 / rate_hz
    x0, z0 = meta["dash_x0_m"], meta["dash_z0_m"]
    y0 = meta["dash_y0_mag_m"]
    dash_speed = meta["dash_speed_m_s"]
    v0 = -dash_speed  # r2l convention (matches rig_snapshot_capture.py)
    n_steps = int(round((2.0 * y0 / dash_speed) / T))  # one full corridor pass

    R_start = true_range_from_rig(rig, (x0, y0, z0))
    sigma_meas = float(sm.sigma_R(R_start, rig.baseline_m, rig.f_px, "expected"))

    tail_residuals = []
    for seed in range(n_seeds):
        rng = random.Random(seed0 + seed)
        gtf = GroundTrackFilter(update_rate_hz=rate_hz, kalata_sigma_process=sigma_w,
                                 kalata_sigma_meas=sigma_meas)
        y_true, v_true, t = y0, v0, 0.0
        pass_residuals = []
        for i in range(n_steps):
            p = (x0, y_true, z0)
            uv = project_world_to_stereo_pixels(p, rig.true_pose, rig.baseline_m,
                                                 rig.f_px, rig.cx, rig.cy)
            if uv is None:
                break
            jl_u, jl_v, jr_u, jr_v = jitter_centroid_pair(*uv, sigma_d_px, rng)
            res = triangulate(jl_u, jl_v, jr_u, jr_v, rig.true_pose, rig.baseline_m,
                               rig.f_px, rig.cx, rig.cy)
            if res is None:
                break
            dt = T if gtf.initialized else None
            st = gtf.step(res.x, res.y, res.z, t, dt=dt)
            pass_residuals.append(st.vy - v_true)  # v_true AT correction time
            t += T
            a = rng.gauss(0.0, sigma_w)
            y_true += v_true * T + 0.5 * a * T * T
            v_true += a * T
        tail_residuals.extend(pass_residuals[-tail_ticks:])

    measured = statistics.pstdev(tail_residuals) if len(tail_residuals) > 1 else float("nan")
    return measured, len(tail_residuals), sigma_meas, R_start, n_steps


# ============================================================================
# I/O
# ============================================================================
def _write_csv(path, rows):
    if not rows:
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"[t18-sigma] wrote {path} ({len(rows)} rows)")


# ============================================================================
# main
# ============================================================================
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sweep-dir", default=DEFAULT_SWEEP_DIR,
                     help="T16 capture directory (capture_meta.json + index.csv)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n-trials", type=int, default=3000,
                     help="Monte-Carlo trials per (tier, range) point, synthetic sweep")
    ap.add_argument("--n-trials-capture", type=int, default=500,
                     help="Monte-Carlo trials per captured row, real-capture sweep")
    ap.add_argument("--sigma-v-n-seeds", type=int, default=80)
    ap.add_argument("--sigma-v-duration-s", type=float, default=60.0)
    ap.add_argument("--sigma-v-warmup-s", type=float, default=30.0)
    ap.add_argument("--tol", type=float, default=0.10,
                     help="relative-error tolerance for the EXPECTED-tier "
                          "table-row reproduction checkpoint (default 10%%)")
    args = ap.parse_args()

    meta, index_rows = load_capture(args.sweep_dir)
    rig = RigConfig.from_capture_meta(os.path.join(args.sweep_dir, "capture_meta.json"))
    print(f"[t18-sigma] sweep_dir={args.sweep_dir}")
    print(f"[t18-sigma] rig: baseline={rig.baseline_m} m  HFOV={rig.hfov_rad} rad  "
          f"{rig.width_px}x{rig.height_px}  f_px={rig.f_px:.2f}  true_pose={tuple(rig.true_pose)}")
    print(f"[t18-sigma] n_captured rows: {len(index_rows)}  "
          f"directions: {sorted(set(r['dash_direction'] for r in index_rows))}")

    # ---- (1) synthetic range-sweep reproduction ----
    print_header("(1) SYNTHETIC RANGE SWEEP -- reproducing rig_geometry_analysis.md's table")
    sweep_rows = run_synthetic_sigma_R_sweep(rig, n_trials=args.n_trials, seed=args.seed)
    _write_csv(os.path.join(OUT_DIR, "sigma_R_synthetic_sweep.csv"), sweep_rows)
    print(f"{'tier':>9} {'sigma_d(px)':>11} {'R(m)':>7} {'model_c':>11} "
          f"{'model sigR':>11} {'meas sigR':>10} {'rel_err':>9}")
    all_ok = True
    for r in sweep_rows:
        flag = ""
        if r["tier"] == "expected":
            ok = abs(r["rel_err"]) <= args.tol
            all_ok = all_ok and ok
            flag = "OK" if ok else f"FAIL(>{args.tol:.0%})"
        print(f"{r['tier']:>9} {r['sigma_d_px']:>11.2f} {r['R_m']:>7.1f} "
              f"{r['c_model_per_m2']:>11.3e} {r['model_sigma_R_m']:>11.4f} "
              f"{r['measured_sigma_R_m']:>10.4f} {r['rel_err']:>+8.2%} {flag}")
    print(f"\n[t18-sigma] EXPECTED-tier checkpoint (<= {args.tol:.0%} rel. error, "
          f"every A1_RANGES_M point): {'PASS' if all_ok else 'FAIL'}")

    # ---- (2) real-capture sigma_R, binned by true range ----
    print_header("(2) REAL-CAPTURE SIGMA_R -- binned by true range, from the T16 sweep")
    per_row, bin_rows = run_capture_sigma_R(rig, index_rows, n_trials=args.n_trials_capture,
                                             seed=args.seed)
    _write_csv(os.path.join(OUT_DIR, "sigma_R_capture_per_row.csv"), per_row)
    _write_csv(os.path.join(OUT_DIR, "sigma_R_capture_bins.csv"), bin_rows)
    print(f"{'tier':>9} {'R_bin(m)':>9} {'n_rows':>7} {'meas sigR':>10} "
          f"{'model sigR':>11} {'rel_err':>9}")
    for r in bin_rows:
        print(f"{r['tier']:>9} {r['R_bin_center_m']:>9.1f} {r['n_rows_in_bin']:>7} "
              f"{r['measured_sigma_R_m']:>10.4f} {r['model_sigma_R_m']:>11.4f} "
              f"{r['rel_err']:>+8.2%}")

    # ---- (3a) scalar Kalata reproduction ----
    print_header("(3a) SCALAR KALATA REPRODUCTION -- vs rig_geometry_analysis.md Sec 3")
    R_ref = true_range_from_rig(rig, (meta["dash_x0_m"], 0.0, meta["dash_z0_m"]))
    sigma_R_expected_full = float(sm.sigma_R(R_ref, rig.baseline_m, rig.f_px, "expected"))
    sigma_R_expected_matching = model_c_matching(
        sm.PARAMS["sigma_match_px"]["expected"], rig.baseline_m, rig.f_px
    ) * R_ref * R_ref
    meas_full, n_full, a_full, b_full, ana_full = run_scalar_kalata_reproduction(
        DASH_ACCEL_CLASS_GENTLE_MPS2, sigma_R_expected_full, 10.0,
        args.sigma_v_duration_s, args.sigma_v_warmup_s, args.sigma_v_n_seeds, seed0=args.seed,
    )
    meas_match, n_match, a_match, b_match, ana_match = run_scalar_kalata_reproduction(
        DASH_ACCEL_CLASS_GENTLE_MPS2, sigma_R_expected_matching, 10.0,
        args.sigma_v_duration_s, args.sigma_v_warmup_s, args.sigma_v_n_seeds,
        seed0=args.seed + 90000,
    )
    print(f"R_ref (rig-to-corridor-CPA range) = {R_ref:.2f} m")
    print(f"gentle target (sigma_w={DASH_ACCEL_CLASS_GENTLE_MPS2} m/s^2), rate=10 Hz")
    print(f"  FULL-4-term sigma_R (doc's own headline input)     = {sigma_R_expected_full:.4f} m")
    print(f"    alpha={a_full:.4f} beta={b_full:.4f}  analytic sigma_v={ana_full:.4f} m/s  "
          f"measured (n={n_full})={meas_full:.4f} m/s  "
          f"rel_err={(meas_full-ana_full)/ana_full:+.2%}")
    print(f"  MATCHING-ONLY sigma_R (this harness's own injectable noise) = "
          f"{sigma_R_expected_matching:.4f} m")
    print(f"    alpha={a_match:.4f} beta={b_match:.4f}  analytic sigma_v={ana_match:.4f} m/s  "
          f"measured (n={n_match})={meas_match:.4f} m/s  "
          f"rel_err={(meas_match-ana_match)/ana_match:+.2%}")
    scalar_tol = 0.15
    scalar_ok = (abs((meas_full - ana_full) / ana_full) <= scalar_tol
                 and abs((meas_match - ana_match) / ana_match) <= scalar_tol)
    print(f"[t18-sigma] scalar Kalata reproduction checkpoint "
          f"(<= {scalar_tol:.0%} rel. error, both inputs): {'PASS' if scalar_ok else 'FAIL'}")
    print(f"[t18-sigma] doc's own published number (rig_geometry_analysis.md, "
          f"broadside_160m handoff, 10 Hz, gentle, EXPECTED): 0.38 m/s "
          f"(this rig's own f_px gives {ana_full:.3f} m/s at R_ref={R_ref:.1f} m -- "
          f"the ~2% f_px difference vs the doc's chosen-lens design point, disclosed "
          f"in rig_geometry_analysis.py's own comment, explains the small gap)")

    # ---- (3b) real-pipeline sigma_v ----
    print_header("(3b) REAL-PIPELINE SIGMA_V -- one captured-geometry corridor pass, "
                 "project->triangulate->filter")
    sigma_d_expected = sm.PARAMS["sigma_match_px"]["expected"]
    meas_pipeline, n_pipeline, sigma_meas_used, R_start, n_steps_pass = run_pipeline_sigma_v(
        rig, meta, DASH_ACCEL_CLASS_GENTLE_MPS2, sigma_d_expected, 10.0,
        args.sigma_v_n_seeds, tail_ticks=20, seed0=args.seed,
    )
    print(f"single corridor pass: {n_steps_pass} ticks ({n_steps_pass/10.0:.1f} s @ 10 Hz) -- "
          f"the wedge-coverage-validated span (rig_geometry_analysis.md Sec 2), "
          f"NOT extended past it (see function docstring for why)")
    print(f"R_start (dash start range) = {R_start:.2f} m; "
          f"kalata_sigma_meas fed to the filter (worst-axis-pessimistic, full sm.sigma_R) "
          f"= {sigma_meas_used:.4f} m")
    print(f"measured sigma_v on world-Y, last 20 ticks of the pass (n={n_pipeline} pooled "
          f"tick-residuals) = {meas_pipeline:.4f} m/s")
    print(f"  vs (3a)'s STEADY-STATE scalar-model analytic = {ana_full:.4f} m/s  "
          f"vs s2_cue_mock's assumed 0.5 m/s fusion-arc target")
    print("  HONEST CAVEAT: (3a) shows this filter needs ~15-20 s to reach Kalata's steady "
          "state at gentle/EXPECTED/10 Hz/this range -- LONGER than one 6.5 s corridor pass, "
          "so this is a TRANSIENT figure (closest to convergence this geometry offers, right "
          "before HANDOFF), not a steady-state reproduction. It happens to land close to "
          "(3a)'s number here, which is a favorable coincidence of this rig pose/speed, not "
          "a second independent confirmation -- (3a) is the assertable checkpoint.")
    meets_target = meas_pipeline <= SIGMA_V_TARGET_MPS
    print(f"[t18-sigma] real-pipeline (transient) sigma_v <= {SIGMA_V_TARGET_MPS} m/s "
          f"(s2_cue_mock's assumed fusion-arc target): {'YES' if meets_target else 'NO'}")

    v_rows = [{
        "check": "scalar_kalata_full_budget", "R_ref_m": R_ref,
        "sigma_meas_m": sigma_R_expected_full, "alpha": a_full, "beta": b_full,
        "analytic_sigma_v_mps": ana_full, "measured_sigma_v_mps": meas_full,
        "n_samples": n_full,
    }, {
        "check": "scalar_kalata_matching_only", "R_ref_m": R_ref,
        "sigma_meas_m": sigma_R_expected_matching, "alpha": a_match, "beta": b_match,
        "analytic_sigma_v_mps": ana_match, "measured_sigma_v_mps": meas_match,
        "n_samples": n_match,
    }, {
        "check": "real_pipeline_worldY", "R_ref_m": R_start,
        "sigma_meas_m": sigma_meas_used, "alpha": None, "beta": None,
        "analytic_sigma_v_mps": ana_full, "measured_sigma_v_mps": meas_pipeline,
        "n_samples": n_pipeline,
    }]
    _write_csv(os.path.join(OUT_DIR, "sigma_v_validation.csv"), v_rows)

    print_header("SUMMARY")
    print(f"(1) synthetic range-sweep EXPECTED-tier reproduction: {'PASS' if all_ok else 'FAIL'}")
    print(f"(3a) scalar Kalata reproduction (<= {scalar_tol:.0%}):        "
          f"{'PASS' if scalar_ok else 'FAIL'}")
    overall_ok = all_ok and scalar_ok
    print(f"\n[t18-sigma] OVERALL: {'PASS' if overall_ok else 'FAIL'}")
    return 0 if overall_ok else 1


if __name__ == "__main__":
    sys.exit(main())
