#!/usr/bin/env python3
"""P0 of the fusion capstone (docs/fusion_capstone_design.md D1 / ADR-0041):
OFFLINE Q-tuning replay harness for the EKF tracker. Boots no sim, reads no
live gz/mavsdk -- it replays the CAMERA MEASUREMENT STREAMS already logged by
ADR-0037's EKF flight A/B (`logs/mc_ekfab_{alphabeta,ekf}.csv`, 8 flights per
arm, per-tick CSVs at `flight_csv_path`) through the REAL `EKFTracker`
(scripts/ekf_tracker.py) at a grid of candidate `Q_ACCEL_PSD` values, to
SCREEN which Q is worth confirming in Gazebo (the L0 confirm arm, run
separately -- not by this script).

WHY THIS EXISTS (context, ADR-0037 / ADR-0041). The shipped default
Q_ACCEL_PSD=64 is WORST-credible and UNTUNED; the flight A/B showed alpha-beta
clean 8/8 vs EKF clean 2/8 at that Q, with the 6 EKF failures aborting "lost
tag >5.0s" -- a terminal-track-loss symptom consistent with an over-wide P
(Q too high) that stops trusting good measurements near CPA. This harness
replays the SAME logged camera streams at smaller candidate Q to see which
value's innovation statistics look like a well-tuned filter, BEFORE spending
another Gazebo arm.

DESIGN DECISIONS THIS SCRIPT IMPLEMENTS (D1a/b/c, fusion_capstone_design.md
section 3):

  D1a -- FIXED dt=0.05 cadence, not sim-time dt. Production
  (`m4_intercept.py`: CONTROL_RATE_HZ=20 at line 282, `dt = 1.0 /
  CONTROL_RATE_HZ` at line 1472) calls `lambda_filter.predict(dt)` /
  `range_filter.predict(dt)` ONCE PER CONTROL TICK with dt FIXED at 0.05 s,
  regardless of how long that tick actually took in sim time -- RTF sag
  (measured ~0.62 previously; this script re-measures and reports it) means
  each control tick's SIM-time duration varies, but the tuned Q is a
  CONTROL-LOOP-RELATIVE number (Q sets how much uncertainty accrues per
  PREDICT CALL, not per second of physics). So: one predict(DT) per replayed
  CSV row, DT=0.05, never the logged t_sim delta. We independently compute
  the ACHIEVED RTF (median sim-dt / DT) per flight purely for REPORTING
  context, matching the previously-measured ~0.62 figure.

  D1b -- exclude the post-breakoff tail. `POST_BREAKOFF_LOG_S=2.0`
  (m4_intercept.py:313) means CLEAN flights keep logging ~2 s into the
  BREAKOFF phase (climb-off) after the intercept regime ends -- mixing a
  different flight regime into the measurement stream. We replay
  [start_idx, end_idx] where end_idx is the LAST row with phase=="ENGAGE".
  Verified against the logs: ABORTED flights (clean=0, "lost tag >5.0s")
  never reach BREAKOFF at all -- their last logged row IS already ENGAGE, so
  the cut is a no-op for them and only trims the tail for clean flights.

  Replay START: the first row where phase=="ENGAGE" OR (phase=="DASH" AND
  detected=="1") -- i.e. the earliest tick that could possibly drive a real
  camera correction (pre-handoff DASH-phase acquisitions do happen and DO
  correct the shared lambda/range channels in production, m4_intercept.py
  ~1630-1655, which runs unconditionally every tick regardless of phase).
  This is provably safe even if we started earlier: EKFTracker.predict() is
  a no-op while self.x is None (cold, pre-first-detection), so any
  predict-only ticks before the first real joint update never touch the
  filter state either way.

  D1c -- BOTH arms' streams, reported separately + pooled, with a caveat.
  We replay all 16 flights (8 alphabeta-arm + 8 ekf-arm). The ekf-arm's own
  streams are what the untuned Q=64 filter actually saw and, for 6/8
  flights, aborted on -- directly relevant, but SELECTION-BIASED: aborted
  flights are truncated at the moment of failure, so their measurement
  cadence/duration sample is drawn from flights that already went wrong
  (whatever cadence happened to precede a Q=64 track-loss). The alphabeta
  arm flew the SAME cue seeds (paired, ADR-0037) with a tracker that has no
  Q to mistune, so all 8 reached a clean terminal hold -- a fuller, less
  failure-biased measurement-cadence sample of "what a healthy flight's
  camera stream looks like." Neither is strictly better: the ekf-arm stream
  is more IN-CONTEXT (literally what Q=64 failed on), the alphabeta-arm
  stream is less SURVIVORSHIP-BIASED. We report both tables plus a POOLED
  (all 16) table and base the primary recommendation on the pooled table,
  cross-checking it does not flip sign against either arm-only table.

CAVEAT (verbatim from the design, D1c/D1(c)): "replay is a SCREEN, not
proof -- open-loop replay of a closed-loop failure cannot show a better Q
would have prevented the aborts; the confirm arm does the epistemic work."
This script's recommendation is a ranking input to ONE Gazebo L0 confirm
arm (run separately by the orchestrating session), not a verdict.

METRICS (per flight, per candidate PSD) -- see docstring of
`replay_flight_at_psd` for the exact per-tick mechanics:
  1. NIS consistency: fraction of camera-update innovations (NIS =
     y^T S^-1 y, dof=2 always -- the loop always corrects [lambda, R]
     jointly) landing inside the EXACT chi-square(2) 95% two-sided band
     ([-2 ln(0.975), -2 ln(0.025)] -- dof=2 has a closed-form CDF, no
     approximation needed), plus the sample mean NIS vs its expected value
     (2.0). Computed over ALL attempted joint updates, gated or not (the
     gate decision is itself diagnostic of Q, not something to condition
     out of the consistency check).
  2. Coast behavior: the LONGEST real inter-detection gap in the replay
     window; |lambda_hat error| (vs gt_cam/gt_tag -- SCORING ONLY, exactly
     the honesty boundary EKFTracker.nees() already uses; never fed back
     into any correction) at the tick right after the gap-starting
     correction, at the tick right before the gap-ending correction (i.e.
     "the error vs the next measurement when it arrives" -- the literal
     coast divergence a real re-acquire would see), and the PEAK |error|
     anywhere in between.
  3. Numerical health: Cholesky(P) succeeds and x/P stay finite at every
     tick; the EFFECTIVE gain spectral radius rho(K@H) (the same
     unit-free "fraction of state corrected" check as
     tests/test_ekf_tracker.py::test_long_gap_gain_stays_sane) at every
     ACCEPTED correction, must stay < 1.
  4. Gate-rejection rate: count of joint updates REJECTED by EKFTracker's
     own chi-square gate (GATE_CHI2_DOF2_P99=9.210, the production
     threshold, untouched) over total attempted updates (excluding the
     one cold-init "correction" per flight, which bypasses the gate
     entirely -- see ekf_tracker.py:_camera_update).

Camera updates are driven through the REAL production surface --
`ekf.lambda_filter.correct(lambda_meas, t)` then `ekf.range_filter.correct
(range_m, t)` (the `_ChannelView` pair, ekf_tracker.py:227-287) -- buffering
the bearing then flushing the joint 2-D update, EXACTLY the call sequence
m4_intercept.py:1637-1639 uses. `lambda_meas` is reconstructed the same way
production computes it (m4_intercept.py:1637): `lambda_meas = radians(psi_deg)
+ bearing_rad`, using the LOGGED psi_deg/bearing_rad columns -- NOT the
logged `lambda_deg` column, which is the FILTERED estimate (lambda_hat),
not the raw measurement (verified against m4_intercept.py:2288's write_row_m4
call site).

FRESHNESS RECONSTRUCTION (a necessary approximation, disclosed). Production
gates camera corrections on `fresh = detected and new_meas`, where
`new_meas` compares the camera thread's frame timestamp (`meas.t_mono`) to
the last-seen one -- NOT logged to CSV. We approximate freshness by
comparing a detected=1 row's (meas_range, bearing_rad) STRING values (as
logged, 4-5 dp) against the immediately preceding detected=1 row's values:
unchanged => the same held camera frame sampled by a faster control tick
(NOT fresh, matches production's cadence mismatch -- camera runs ~14 Hz vs
the 20 Hz control loop per m4_intercept.py's own comment); changed => a
genuinely new detection (coincidental collision at this precision is
negligible given continuous target motion + camera noise). Measured on
these 16 flights: 17/89 (~19%) of detected=1 rows are such duplicates --
NOT negligible, so this correction matters and is applied.

Run:  .venv/bin/python3 scripts/ekf_q_replay.py
Output: printed ranked tables (ekf-arm / alphabeta-arm / pooled) +
        logs/ekf_q_replay_results.json
"""

import argparse
import csv
import glob
import json
import math
import os
import sys
from datetime import datetime, timezone

import numpy as np

SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPTS_DIR)
LOGS_DIR = os.path.join(REPO_ROOT, "logs")
sys.path.insert(0, SCRIPTS_DIR)

from ekf_tracker import EKFTracker, wrap_pi  # noqa: E402

DT = 0.05  # production fixed control-loop dt (m4_intercept.py:282 CONTROL_RATE_HZ=20; :1472)

DEFAULT_CANDIDATE_PSDS = [64.0, 32.0, 16.0, 8.0, 4.0, 2.0, 1.0, 0.5]

ARM_CSV = {
    "ekf": os.path.join(LOGS_DIR, "mc_ekfab_ekf.csv"),
    "alphabeta": os.path.join(LOGS_DIR, "mc_ekfab_alphabeta.csv"),
}

# Exact chi-square(dof=2) two-sided 95% band (dof=2 is the exponential
# distribution scaled by 2 -> closed-form CDF(x) = 1 - exp(-x/2), so this is
# EXACT, not the Wilson-Hilferty approximation ekf_tracker.chi2_ppf uses for
# large-dof bands). GATE_CHI2_DOF2_P99=9.210 (ekf_tracker.py) is the
# production ONE-SIDED 99% gate threshold -- a different number for a
# different purpose (reject gross outliers vs. score consistency).
NIS_DOF2_LO = -2.0 * math.log(0.975)   # chi2.ppf(0.025, 2) = 0.0506
NIS_DOF2_HI = -2.0 * math.log(0.025)   # chi2.ppf(0.975, 2) = 7.3778
NIS_DOF2_EXPECTED_MEAN = 2.0

GATE_REJECT_BAND = (0.01, 0.05)  # design D1: "well-tuned gates ~1-5%"

CAVEAT = (
    "replay is a SCREEN, not proof -- open-loop replay of a closed-loop "
    "failure cannot show a better Q would have prevented the aborts; the "
    "L0 confirm arm (one Gazebo arm, run separately) does the epistemic work."
)


# =====================================================================
# Log loading + replay-window selection (D1a/D1b)
# =====================================================================
def load_arm_flights(arm: str):
    """Return [(run_idx, flight_csv_path, clean)] for one ADR-0037 arm."""
    with open(ARM_CSV[arm], newline="") as f:
        rows = list(csv.DictReader(f))
    return [(int(r["run_idx"]), r["flight_csv_path"], r["clean"] == "1") for r in rows]


def load_flight_rows(path: str):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def determine_replay_window(rows):
    """(start_idx, end_idx), both inclusive, per D1a/D1b. Returns None if the
    flight never reaches a usable window (should not happen for handoff=1
    flights -- all 16 do, verified against the logs)."""
    start_idx = None
    for i, r in enumerate(rows):
        if r["phase"] == "ENGAGE" or (r["phase"] == "DASH" and r["detected"] == "1"):
            start_idx = i
            break
    if start_idx is None:
        return None

    end_idx = None
    for i, r in enumerate(rows):
        if r["phase"] == "ENGAGE":
            end_idx = i
    if end_idx is None or end_idx < start_idx:
        end_idx = len(rows) - 1  # fallback: no ENGAGE row at all (unseen in these logs)

    return start_idx, end_idx


def achieved_rtf(rows, start_idx, end_idx):
    """Median(sim-dt)/DT over the replay window -- reporting only, per D1a
    ("log the achieved RTF beside it"). Returns (rtf, n_dt_samples)."""
    tsims = []
    for i in range(start_idx, end_idx + 1):
        v = rows[i]["t_sim"]
        if v:
            tsims.append(float(v))
    if len(tsims) < 2:
        return None, 0
    dts = [tsims[i + 1] - tsims[i] for i in range(len(tsims) - 1)]
    dts = [d for d in dts if d > 0.0]
    if not dts:
        return None, 0
    return float(np.median(dts)) / DT, len(dts)


def gt_lambda_true(row):
    """TRUE relative LOS azimuth from gt_cam/gt_tag -- SCORING ONLY (same
    honesty boundary as EKFTracker.nees(): reconstructed from gt_* offline,
    never fed into a correction). Matches the convention lambda_meas uses
    (north=x, east=y, atan2(de, dn))."""
    try:
        dn = float(row["gt_tag_x"]) - float(row["gt_cam_x"])
        de = float(row["gt_tag_y"]) - float(row["gt_cam_y"])
    except (ValueError, KeyError):
        return None
    return math.atan2(de, dn)


# =====================================================================
# Core replay (drives the REAL EKFTracker through its production surface)
# =====================================================================
def replay_flight_at_psd(rows, start_idx, end_idx, q_psd, gating=True):
    """Replay one flight's [start_idx, end_idx] window through a freshly
    constructed EKFTracker(q_accel_psd=q_psd) (all other params default,
    matching m4_intercept.py's `EKFTracker()` construction exactly). Returns
    a per-flight metrics dict. See module docstring for the metric
    definitions and the freshness-reconstruction caveat."""
    ekf = EKFTracker(q_accel_psd=q_psd, gating=gating)

    prev_detected_key = None
    n = end_idx - start_idx + 1

    fresh_flags = [False] * n
    lambda_hat_pre = [None] * n   # post-predict, pre-correct this tick
    lambda_hat_post = [None] * n  # post all processing this tick
    lambda_true = [None] * n

    nis_samples = []
    n_attempts = 0        # joint updates attempted AFTER cold-init (gate-eligible)
    n_gated = 0
    cholesky_fail = 0
    nonfinite = 0
    max_gain_rho = 0.0
    n_gain_checks = 0
    skipped_malformed = 0

    for local_i in range(n):
        row = rows[start_idx + local_i]
        lambda_true[local_i] = gt_lambda_true(row)

        ekf.lambda_filter.predict(DT)
        ekf.range_filter.predict(DT)

        if ekf.initialized:
            lambda_hat_pre[local_i] = ekf.lambda_hat

        fresh = False
        if row["detected"] == "1" and row["meas_range"] and row["bearing_rad"]:
            key = (row["meas_range"], row["bearing_rad"])
            fresh = key != prev_detected_key
            prev_detected_key = key

        if fresh:
            try:
                psi_rad = math.radians(float(row["psi_deg"]))
                bearing_rad = float(row["bearing_rad"])
                range_m = float(row["meas_range"])
            except (ValueError, TypeError):
                skipped_malformed += 1
                fresh = False

        if fresh:
            was_initialized = ekf.initialized
            lam_meas = psi_rad + bearing_rad
            t = float(row["t_sim"]) if row["t_sim"] else float(start_idx + local_i) * DT

            # Effective-gain check (pre-update state), mirrors
            # tests/test_ekf_tracker.py::test_long_gap_gain_stays_sane --
            # recomputed from the SAME formulas ekf_tracker._camera_update
            # uses, purely for scoring (does not alter the tracker).
            gain_rho = None
            if was_initialized:
                dn, de = ekf.x[0], ekf.x[1]
                R2 = max(dn * dn + de * de, 1e-9)
                R = math.sqrt(R2)
                H = np.array([[-de / R2, dn / R2, 0.0, 0.0], [dn / R, de / R, 0.0, 0.0]])
                sigma_R = max(0.05, ekf.cam_range_frac * R)
                Rmat = np.diag([ekf.cam_bearing_sigma ** 2, sigma_R ** 2])
                S = H @ ekf.P @ H.T + Rmat
                try:
                    K = ekf.P @ H.T @ np.linalg.inv(S)
                    gain_rho = float(max(abs(np.linalg.eigvals(K @ H))))
                except np.linalg.LinAlgError:
                    gain_rho = None

            n_gated_before = ekf.n_gated
            ekf.lambda_filter.correct(lam_meas, t)
            ekf.range_filter.correct(range_m, t)

            if was_initialized:
                n_attempts += 1
                gated = ekf.n_gated > n_gated_before
                if gated:
                    n_gated += 1
                if ekf.last_nis is not None:
                    nis_samples.append(ekf.last_nis)
                if not gated and gain_rho is not None:
                    max_gain_rho = max(max_gain_rho, gain_rho)
                    n_gain_checks += 1

            fresh_flags[local_i] = True

        if ekf.P is not None:
            if not np.all(np.isfinite(ekf.P)) or not np.all(np.isfinite(ekf.x)):
                nonfinite += 1
            else:
                try:
                    np.linalg.cholesky(ekf.P)
                except np.linalg.LinAlgError:
                    cholesky_fail += 1

        lambda_hat_post[local_i] = ekf.lambda_hat if ekf.initialized else None

    # --- coast-gap metric: longest inter-correction gap, error growth ---
    fresh_idx = [i for i, f in enumerate(fresh_flags) if f]
    coast = {
        "gap_ticks": None, "gap_s": None,
        "err_start_deg": None, "err_end_deg": None,
        "err_peak_deg": None, "growth_deg": None,
    }
    if len(fresh_idx) >= 2:
        gaps = [(fresh_idx[k + 1] - fresh_idx[k], k) for k in range(len(fresh_idx) - 1)]
        gap_ticks, k = max(gaps, key=lambda g: g[0])
        idx_a, idx_b = fresh_idx[k], fresh_idx[k + 1]
        lt_a, lt_b = lambda_true[idx_a], lambda_true[idx_b]
        lh_a, lh_b = lambda_hat_post[idx_a], lambda_hat_pre[idx_b]
        if lt_a is not None and lh_a is not None and lt_b is not None and lh_b is not None:
            err_start = abs(wrap_pi(lh_a - lt_a))
            err_end = abs(wrap_pi(lh_b - lt_b))
            peak = err_start
            for i in range(idx_a, idx_b + 1):
                if lambda_hat_pre[i] is not None and lambda_true[i] is not None:
                    peak = max(peak, abs(wrap_pi(lambda_hat_pre[i] - lambda_true[i])))
            coast = {
                "gap_ticks": gap_ticks, "gap_s": gap_ticks * DT,
                "err_start_deg": math.degrees(err_start),
                "err_end_deg": math.degrees(err_end),
                "err_peak_deg": math.degrees(peak),
                "growth_deg": math.degrees(err_end - err_start),
            }

    nis_arr = np.asarray(nis_samples, dtype=float)
    nis_inband_frac = (
        float(np.mean((nis_arr >= NIS_DOF2_LO) & (nis_arr <= NIS_DOF2_HI)))
        if nis_arr.size else None
    )

    return {
        "n_ticks": n,
        "n_attempts": n_attempts,
        "n_gated": n_gated,
        "gate_reject_frac": (n_gated / n_attempts) if n_attempts else None,
        "n_nis_samples": int(nis_arr.size),
        "nis_mean": float(np.mean(nis_arr)) if nis_arr.size else None,
        "nis_inband_frac": nis_inband_frac,
        "max_gain_spectral_radius": max_gain_rho if n_gain_checks else None,
        "cholesky_fail_count": cholesky_fail,
        "nonfinite_count": nonfinite,
        "skipped_malformed_rows": skipped_malformed,
        **{f"coast_{k}": v for k, v in coast.items()},
        # Internal-only (stripped before JSON dump, used by aggregate() to
        # build POOLED-RAW statistics -- see aggregate()'s docstring for why
        # this matters: per-flight n_nis_samples is tiny, median 4-5,
        # sometimes 1, so a "median of per-flight fractions" is dominated by
        # whichever flights happen to have 0/1 samples).
        "_nis_samples": nis_samples,
    }


# =====================================================================
# Aggregation + ranking
# =====================================================================
NUMERIC_FIELDS = [
    "gate_reject_frac", "nis_mean", "nis_inband_frac",
    "max_gain_spectral_radius", "coast_growth_deg", "coast_err_end_deg",
    "coast_err_peak_deg", "coast_gap_s",
]


def aggregate(flight_metrics_list):
    """Median + IQR (P25/P75) of each numeric field across flights, PLUS
    pooled-raw statistics, and a pass/fail numerical-health rollup.

    HONEST CAVEAT (found by running this, not anticipated by the design
    doc): per-flight n_nis_samples is TINY -- median ~4-5 joint-update
    attempts per flight in the replay window (detection coverage is
    sparse, ~1-5%), and several flights have only 1. A "median of
    per-flight fractions" is then dominated by whichever flights happen to
    land on 0/1 vs 1/1 -- e.g. at Q=64 pooled the MEDIAN-of-fractions
    gate-reject rate reads 0.0% (most flights' few attempts happened not to
    gate) while the POOLED-RAW rate (7 rejects / 56 total attempts across
    all 16 flights) is 12.5%, a materially different number. Both are
    computed and reported; the POOLED-RAW numbers (more sample mass, ~56
    vs a median of 8 near-single-digit-n fractions) are what `rank_psds`
    actually uses."""
    out = {"n_flights": len(flight_metrics_list)}
    for field in NUMERIC_FIELDS:
        vals = [m[field] for m in flight_metrics_list if m.get(field) is not None]
        if vals:
            arr = np.asarray(vals, dtype=float)
            out[field] = {
                "median": float(np.median(arr)),
                "p25": float(np.percentile(arr, 25)),
                "p75": float(np.percentile(arr, 75)),
                "n": len(vals),
            }
        else:
            out[field] = {"median": None, "p25": None, "p75": None, "n": 0}
    out["numerically_healthy"] = all(
        m["cholesky_fail_count"] == 0 and m["nonfinite_count"] == 0
        for m in flight_metrics_list
    )
    out["total_cholesky_failures"] = sum(m["cholesky_fail_count"] for m in flight_metrics_list)
    out["total_nonfinite"] = sum(m["nonfinite_count"] for m in flight_metrics_list)
    out["total_attempts"] = sum(m["n_attempts"] for m in flight_metrics_list)
    out["total_gated"] = sum(m["n_gated"] for m in flight_metrics_list)
    out["pooled_gate_reject_frac"] = (
        out["total_gated"] / out["total_attempts"] if out["total_attempts"] else None
    )
    pooled_nis = np.asarray(
        [v for m in flight_metrics_list for v in m.get("_nis_samples", [])], dtype=float
    )
    out["pooled_n_nis_samples"] = int(pooled_nis.size)
    out["pooled_nis_mean"] = float(np.mean(pooled_nis)) if pooled_nis.size else None
    out["pooled_nis_inband_frac"] = (
        float(np.mean((pooled_nis >= NIS_DOF2_LO) & (pooled_nis <= NIS_DOF2_HI)))
        if pooled_nis.size else None
    )
    return out


def gate_band_penalty(frac):
    """0 if inside [1%,5%]; else distance to the nearest band edge."""
    if frac is None:
        return float("inf")
    lo, hi = GATE_REJECT_BAND
    if lo <= frac <= hi:
        return 0.0
    return min(abs(frac - lo), abs(frac - hi))


def rank_psds(agg_by_psd):
    """Sort candidate PSDs by (|NIS in-band - 0.95|, gate-band penalty,
    |median coast growth|), per the design's stated recommendation order.
    Numerically-unhealthy PSDs are pushed to the bottom regardless.

    Uses the POOLED-RAW nis-in-band-fraction and gate-reject-fraction (NOT
    the median-of-per-flight-fractions) -- see aggregate()'s docstring for
    why: per-flight n is too thin (median 4-5 samples, several flights=1)
    for a median-of-fractions to be trustworthy. Coast growth stays
    median-of-flights (it is a per-flight structural quantity -- "the
    longest gap in THIS flight" -- pooling it across flights would not be
    meaningful the way pooling independent NIS samples is)."""
    def key(psd):
        a = agg_by_psd[psd]
        nis_ib = a["pooled_nis_inband_frac"]
        nis_score = abs(nis_ib - 0.95) if nis_ib is not None else float("inf")
        gate_frac = a["pooled_gate_reject_frac"]
        gate_score = gate_band_penalty(gate_frac)
        growth = a["coast_growth_deg"]["median"]
        growth_score = abs(growth) if growth is not None else float("inf")
        health_score = 0 if a["numerically_healthy"] else 1
        return (health_score, round(nis_score, 4), round(gate_score, 4), round(growth_score, 4))

    return sorted(agg_by_psd.keys(), key=key)


def fmt_pct(x):
    return "n/a" if x is None else f"{100.0 * x:5.1f}%"


def fmt_num(x, spec="{:6.3f}"):
    return "n/a" if x is None else spec.format(x)


def print_table(title, agg_by_psd, ranked_order):
    """Prints POOLED-RAW NIS-in-band / gate-reject (the numbers rank_psds
    actually uses, sample mass ~ total_attempts) alongside the
    median-of-per-flight-fractions (in parens) for transparency -- see
    aggregate()'s docstring for why these two can diverge at this n."""
    print(f"\n=== {title} ===")
    header = (
        f"{'PSD':>7} | {'NIS mean':>9} | {'NIS in-band (pooled/med-of-flt)':>32} | "
        f"{'gate-reject (pooled/med-of-flt)':>32} | {'n':>4} | {'coast growth deg':>17} | "
        f"{'gain rho max':>12} | {'healthy':>7}"
    )
    print(header)
    print("-" * len(header))
    for rank, psd in enumerate(ranked_order, 1):
        a = agg_by_psd[psd]
        nis_mean = fmt_num(a["pooled_nis_mean"])
        nis_ib = f"{fmt_pct(a['pooled_nis_inband_frac'])} / {fmt_pct(a['nis_inband_frac']['median'])}"
        gate = f"{fmt_pct(a['pooled_gate_reject_frac'])} / {fmt_pct(a['gate_reject_frac']['median'])}"
        n_pooled = a["pooled_n_nis_samples"]
        growth = fmt_num(a["coast_growth_deg"]["median"], "{:6.2f}")
        rho = fmt_num(a["max_gain_spectral_radius"]["median"])
        healthy = "yes" if a["numerically_healthy"] else "NO"
        tag = " <-- rank 1" if rank == 1 else ""
        print(
            f"{psd:7.2f} | {nis_mean:>9} | {nis_ib:>32} | {gate:>32} | "
            f"{n_pooled:4d} | {growth:>17} | {rho:>12} | {healthy:>7}{tag}"
        )


# =====================================================================
# Main
# =====================================================================
def build_flight_index():
    """Load both arms' flight lists + windows + achieved RTF once."""
    flights = {}  # arm -> [dict]
    for arm in ("ekf", "alphabeta"):
        flights[arm] = []
        for run_idx, path, clean in load_arm_flights(arm):
            rows = load_flight_rows(path)
            window = determine_replay_window(rows)
            if window is None:
                print(f"[warn] {arm} run {run_idx}: no usable replay window, skipping", file=sys.stderr)
                continue
            start_idx, end_idx = window
            rtf, n_dt = achieved_rtf(rows, start_idx, end_idx)
            flights[arm].append({
                "run_idx": run_idx, "path": path, "clean": clean,
                "rows": rows, "start_idx": start_idx, "end_idx": end_idx,
                "rtf": rtf, "n_dt_samples": n_dt,
            })
    return flights


def sweep(flights_by_arm, candidate_psds):
    """Returns metrics[arm][run_idx][psd] = per-flight metrics dict."""
    metrics = {"ekf": {}, "alphabeta": {}}
    for arm, flist in flights_by_arm.items():
        for fl in flist:
            metrics[arm][fl["run_idx"]] = {}
            for psd in candidate_psds:
                metrics[arm][fl["run_idx"]][psd] = replay_flight_at_psd(
                    fl["rows"], fl["start_idx"], fl["end_idx"], psd
                )
    return metrics


def aggregates_for(metrics, arm_filter, psds):
    """arm_filter in {'ekf', 'alphabeta', 'pooled'} -> {psd: aggregate}."""
    out = {}
    for psd in psds:
        if arm_filter == "pooled":
            flist = [metrics["ekf"][r][psd] for r in metrics["ekf"]] + \
                    [metrics["alphabeta"][r][psd] for r in metrics["alphabeta"]]
        else:
            flist = [metrics[arm_filter][r][psd] for r in metrics[arm_filter]]
        out[psd] = aggregate(flist)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--psds", default=",".join(str(p) for p in DEFAULT_CANDIDATE_PSDS))
    ap.add_argument("--output", default=os.path.join(LOGS_DIR, "ekf_q_replay_results.json"))
    ap.add_argument("--max-grid-extensions", type=int, default=4)
    args = ap.parse_args()

    candidate_psds = [float(x) for x in args.psds.split(",")]

    print(f"[ekf_q_replay] loading flights from {ARM_CSV['ekf']} + {ARM_CSV['alphabeta']}")
    flights_by_arm = build_flight_index()
    for arm in ("ekf", "alphabeta"):
        n = len(flights_by_arm[arm])
        rtfs = [f["rtf"] for f in flights_by_arm[arm] if f["rtf"] is not None]
        print(f"[ekf_q_replay] arm={arm}: {n} flights loaded, "
              f"windows=[{','.join(str(f['end_idx']-f['start_idx']+1) for f in flights_by_arm[arm])}] ticks")

    print(f"\n[ekf_q_replay] candidate PSDs: {candidate_psds}")
    metrics = sweep(flights_by_arm, candidate_psds)

    extensions_done = 0
    while True:
        agg_pooled = aggregates_for(metrics, "pooled", candidate_psds)
        ranked_pooled = rank_psds(agg_pooled)
        best = ranked_pooled[0]
        lo, hi = min(candidate_psds), max(candidate_psds)
        if best not in (lo, hi) or extensions_done >= args.max_grid_extensions:
            break
        new_psd = lo / 2.0 if best == lo else hi * 2.0
        print(f"[ekf_q_replay] recommendation landed on grid edge ({best}); "
              f"extending grid by one octave -> {new_psd}")
        for arm, flist in flights_by_arm.items():
            for fl in flist:
                metrics[arm][fl["run_idx"]][new_psd] = replay_flight_at_psd(
                    fl["rows"], fl["start_idx"], fl["end_idx"], new_psd
                )
        candidate_psds.append(new_psd)
        candidate_psds.sort()
        extensions_done += 1

    agg_ekf = aggregates_for(metrics, "ekf", candidate_psds)
    agg_ab = aggregates_for(metrics, "alphabeta", candidate_psds)
    agg_pooled = aggregates_for(metrics, "pooled", candidate_psds)
    ranked_ekf = rank_psds(agg_ekf)
    ranked_ab = rank_psds(agg_ab)
    ranked_pooled = rank_psds(agg_pooled)

    print_table("ekf-arm streams (8 flights; Q=64's OWN measurement history -- "
                "note: 6/8 truncated by the abort itself, D1c caveat)", agg_ekf, ranked_ekf)
    print_table("alphabeta-arm streams (8 flights; same seeds, all 8 reached a "
                "clean terminal -- 'closer to healthy' cadence sample, D1c)", agg_ab, ranked_ab)
    print_table("POOLED (all 16 flights) -- primary recommendation basis", agg_pooled, ranked_pooled)

    recommended = ranked_pooled[0]
    shortlist = ranked_pooled[:2]
    agrees_ekf = ranked_ekf[0]
    agrees_ab = ranked_ab[0]

    print("\n=== Recommendation ===")
    print(f"Primary PSD (pooled 16-flight ranking): {recommended}")
    print(f"Top-2 shortlist (pooled): {shortlist}")
    print(f"ekf-arm-only top pick: {agrees_ekf}   alphabeta-arm-only top pick: {agrees_ab}")
    if recommended not in (min(candidate_psds), max(candidate_psds)):
        print("Grid check: recommendation is NOT at a grid edge. OK.")
    else:
        print("Grid check: recommendation is STILL at a grid edge after "
              f"{extensions_done} extension(s) (cap={args.max_grid_extensions}).")
    print(f"\nCAVEAT (verbatim, design D1c): {CAVEAT}")

    # --- achieved RTF summary ---
    all_rtfs = [f["rtf"] for arm in flights_by_arm.values() for f in arm if f["rtf"] is not None]
    rtf_median = float(np.median(all_rtfs)) if all_rtfs else None
    rtf_p25 = float(np.percentile(all_rtfs, 25)) if all_rtfs else None
    rtf_p75 = float(np.percentile(all_rtfs, 75)) if all_rtfs else None
    print(f"\n=== Achieved RTF (median sim-dt / {DT}) across {len(all_rtfs)} flights ===")
    print(f"median={rtf_median:.3f}  IQR=[{rtf_p25:.3f}, {rtf_p75:.3f}]  "
          f"(previously measured ~0.62 under load, ADR-0009 context)")

    # --- sanity check vs design's predicted Q=64 signature ---
    q64_pooled = agg_pooled.get(64.0)
    if q64_pooled is not None:
        gate64 = q64_pooled["pooled_gate_reject_frac"]
        nis64 = q64_pooled["pooled_nis_inband_frac"]
        n64 = q64_pooled["pooled_n_nis_samples"]
        print(f"\n=== Sanity check: Q=64 signature (pooled, n={n64} joint-update attempts) ===")
        print(f"gate-reject (pooled)={fmt_pct(gate64)}  NIS-in-band (pooled)={fmt_pct(nis64)}  "
              f"(median-of-flights was {fmt_pct(q64_pooled['gate_reject_frac']['median'])} / "
              f"{fmt_pct(q64_pooled['nis_inband_frac']['median'])} -- diverges, see caveat below)")
        if gate64 is not None and gate64 < 0.01:
            print("-> near-0% gate rejections: consistent with the predicted over-wide-P signature.")
        else:
            print("-> does NOT show near-0% gate rejections at Q=64 (pooled-raw is the honest "
                  "number, ~12-13%, itself ABOVE the 1-5% well-tuned band) -- reporting honestly, "
                  "this does NOT match the design's pre-registered 'near-0%' expectation. The "
                  "median-of-per-flight-fractions DID read ~0% and would have been misleading here "
                  "-- most flights have only 1-6 attempts, so a flight's fraction is coarse (0/1, "
                  "1/1, ...) and the median across 16 such coarse fractions is dominated by mode, "
                  "not mass. Pooling the raw attempts (n=56) is the trustworthy number.")

    # --- JSON output ---
    result = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "dt_replay_s": DT,
        "candidate_psds": candidate_psds,
        "grid_extensions_applied": extensions_done,
        "arms": {
            "ekf": {psd: agg_ekf[psd] for psd in candidate_psds},
            "alphabeta": {psd: agg_ab[psd] for psd in candidate_psds},
            "pooled": {psd: agg_pooled[psd] for psd in candidate_psds},
        },
        "ranked_order": {
            "ekf": ranked_ekf, "alphabeta": ranked_ab, "pooled": ranked_pooled,
        },
        "recommendation": {
            "primary_psd": recommended,
            "shortlist": shortlist,
            "ekf_arm_only_pick": agrees_ekf,
            "alphabeta_arm_only_pick": agrees_ab,
            "basis": "pooled-RAW-sample (not median-of-per-flight-fraction, see aggregate() "
                     "docstring) NIS-in-band closest to 0.95, then pooled gate-reject fraction "
                     "in [1%,5%], then bounded |median coast growth|",
            "caveat": CAVEAT,
        },
        "achieved_rtf": {
            "median": rtf_median, "p25": rtf_p25, "p75": rtf_p75,
            "n_flights": len(all_rtfs),
            "per_flight": {
                arm: {f["run_idx"]: f["rtf"] for f in flist}
                for arm, flist in flights_by_arm.items()
            },
        },
        "per_flight_metrics": {
            # "_nis_samples" (raw per-tick NIS list, used internally by
            # aggregate() for pooled-raw stats) is stripped here to keep
            # this JSON small, per the task spec -- the pooled stats it
            # feeds are already in "arms" above.
            arm: {
                run_idx: {
                    psd: {k: v for k, v in metrics[arm][run_idx][psd].items() if k != "_nis_samples"}
                    for psd in candidate_psds
                }
                for run_idx in metrics[arm]
            }
            for arm in ("ekf", "alphabeta")
        },
    }
    with open(args.output, "w") as f:
        json.dump(result, f, indent=2, default=str)
    print(f"\n[ekf_q_replay] wrote {args.output}")


if __name__ == "__main__":
    main()
