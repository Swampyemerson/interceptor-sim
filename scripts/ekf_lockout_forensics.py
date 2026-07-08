#!/usr/bin/env python3
"""Focused forensic (delegated task, 2026-07-08): test the LOCKOUT-CASCADE
hypothesis for ADR-0037's EKF clean-rate regression (docs/decisions.md
ADR-0037 / ADR-0041).

CONTEXT. `scripts/ekf_q_replay.py` (P0 of the fusion capstone, ADR-0041)
replayed the ADR-0037 A/B's logged camera streams through the REAL
`EKFTracker` at Q=64 (production) and found a TIGHT-P signature -- 12.5%
pooled gate-rejection rate, NIS mean 4.19 vs the expected 2.0 -- which
CONTRADICTS the ADR-0037 narrative ("Q too high / over-wide P loses lock
near CPA"). That leaves an open question: if it is not an over-wide-P
problem, is it a GATE problem at all? This script asks a narrower,
mechanistic question.

HYPOTHESIS UNDER TEST (LOCKOUT-CASCADE). In the 6 aborted ekf-arm flights
(`logs/mc_ekfab_ekf.csv`, clean=0, "lost tag >5.0s"), the causal chain is:
chi-square gate rejects a burst of CONSECUTIVE valid camera innovations near
CPA (tight P + noisy near-CPA measurements) -> lambda_dot_hat goes
stale/wrong -> guidance commands point the vehicle away -> tag leaves FOV ->
seeker dropout -> abort. If true: fix is D3.2's adaptive
consecutive-rejection gate-recovery (`docs/fusion_capstone_design.md` D3,
item 2 -- built for the POST-LATCH/post-cue case, tested here in the
general form), NOT a Q change.

WHAT WOULD CONFIRM IT: the 6 aborted flights show a cluster of consecutive
gate rejections immediately (within ~10 ticks) before the fatal >5s dropout
begins, and the 10 clean/control flights do not.

METHOD. Reuses `scripts/ekf_q_replay.py`'s flight-CSV parsing, D1a/D1b
replay-window selection, and D1's freshness reconstruction UNCHANGED
(imported, never copied/modified -- see that module's docstring for the
derivation of each piece). This script adds: (1) a full per-tick TIMELINE
(detected / fresh-attempted / gated / NIS / lambda_hat error / P-trace)
instead of just the aggregated metrics ekf_q_replay computes, (2) explicit
consecutive-gate-rejection streak detection anchored to the FATAL DROPOUT
(the tail all-`detected=0` run that trips `m4_intercept.py`'s
LOST_TAG_ABORT_S=5.0 abort) for the 6 aborted ekf-arm flights, with the 2
clean-ekf + 8 alphabeta-arm flights as controls (same seeds, no Q to
mistune, all reached a normal terminal), and (3) an offline SCREEN of the
D3.2 adaptive gate-recovery mechanism: after N consecutive rejections,
reinflate P's position+velocity diagonal to the cold-init floor ("restore
to seed sigmas", state x untouched -- the same "re-init or provably decay"
discipline fusion_capstone_design.md D3 item 1 uses at the cue latch) and
keep replaying, checking whether the very next attempted correction(s) then
clear the gate.

CAVEAT (same one ekf_q_replay.py states for its own screen, reproduced
verbatim here because it applies identically): "replay is a SCREEN, not
proof -- open-loop replay of a closed-loop failure cannot show a better
[fix] would have prevented the aborts" -- there is no guidance loop, no
re-flown trajectory, and no way for a different gate decision here to change
where the vehicle actually was or what it actually saw next; only a Gazebo
arm does that epistemic work.

FATAL-GAP TIMING NOTE (do not misapply the "sim time never wall time"
gotcha here). `m4_intercept.py`'s LOST_TAG_ABORT_S=5.0 abort timer is
measured against `tick_start = time.monotonic()` -- i.e. WALL time, not sim
time -- because it is an operator real-time patience budget (how long a
human/system should keep hovering blind), not a physics quantity. So
reconstructing "was this gap actually >5s when it aborted" correctly means
reading the CSV's `t` (wall) column, not `t_sim`, for THIS ONE quantity;
every kinematic/filter quantity in this script still uses the fixed
DT=0.05 replay cadence per ekf_q_replay's D1a, exactly as production's
`predict(dt)` calls do.

Run:  .venv/bin/python3 scripts/ekf_lockout_forensics.py
Output: printed verdict table + cascade/D3.2 sections +
        logs/ekf_lockout_forensics.json
"""

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

from ekf_tracker import (  # noqa: E402
    EKFTracker, wrap_pi, INIT_POS_SIGMA_M, INIT_VEL_SIGMA_M_S,
)
from ekf_q_replay import (  # noqa: E402
    ARM_CSV, DT, load_arm_flights, load_flight_rows,
    determine_replay_window, gt_lambda_true,
)

Q_PROD = 64.0                  # production Q_ACCEL_PSD (ADR-0037 shipped default; untouched here)
N_CONSEC_PRIMARY = 3           # D3.2's own trigger, as specified in fusion_capstone_design.md D3
N_CONSEC_SUPPLEMENTARY = 1     # exploratory: does ANY single rejection's reinflation unblock?
LAST_N_TICKS = 10              # "rejections in the last 10 detected ticks" per the task brief
CASCADE_PROXIMITY_TICKS = 10   # streak counts as "immediately before" the fatal gap within this

CAVEAT = (
    "replay is a SCREEN, not proof -- open-loop replay of a closed-loop "
    "failure cannot show a better gate policy would have prevented the "
    "aborts (no closed guidance loop, no re-flown trajectory, no way for a "
    "different gate decision here to change what the vehicle actually saw "
    "next); a Gazebo confirm arm would do the epistemic work, same caveat "
    "ekf_q_replay.py states verbatim for its own Q screen (D1c)."
)


# =====================================================================
# Timeline replay (drives the REAL EKFTracker, same production surface
# ekf_q_replay.replay_flight_at_psd uses -- extended to keep every tick,
# not just aggregates, and to optionally run the D3.2 recovery rule).
# =====================================================================
def ang_diff_deg(a_deg, b_deg):
    if a_deg is None or b_deg is None:
        return None
    return math.degrees(wrap_pi(math.radians(a_deg - b_deg)))


def replay_timeline(rows, start_idx, end_idx, q_psd=Q_PROD,
                     recover=False, n_consec=N_CONSEC_PRIMARY):
    """Returns (timeline, recovery_events).

    timeline: list of per-tick dicts, one per replayed row --
      detected_raw, fresh, attempted, gated, nis, lambda_hat_pre/post_deg,
      lambda_true_deg, p_trace_pos (P[0,0]+P[1,1], the relative-position
      variance trace).

    recovery_events: only populated when recover=True. Each entry records a
    reinflation trigger (`n_consec` consecutive gate rejections) and,
    once known, whether the NEXT attempted correction after it was gated.
    """
    ekf = EKFTracker(q_accel_psd=q_psd, gating=True)
    n = end_idx - start_idx + 1

    timeline = []
    prev_detected_key = None
    consec_gated = 0
    recovery_events = []

    for local_i in range(n):
        row = rows[start_idx + local_i]
        lam_true = gt_lambda_true(row)

        ekf.lambda_filter.predict(DT)
        ekf.range_filter.predict(DT)

        tick = {
            "local_i": local_i,
            "row_idx": start_idx + local_i,
            "t": float(row["t"]) if row["t"] else None,
            "t_sim": float(row["t_sim"]) if row["t_sim"] else None,
            "phase": row["phase"],
            "detected_raw": row["detected"] == "1",
            "fresh": False,
            "attempted": False,
            "gated": None,
            "nis": None,
            "reinflated_after": False,
            "lambda_hat_pre_deg": math.degrees(ekf.lambda_hat) if ekf.initialized else None,
            "lambda_true_deg": math.degrees(lam_true) if lam_true is not None else None,
        }

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
                fresh = False

        if fresh:
            was_initialized = ekf.initialized
            lam_meas = psi_rad + bearing_rad
            t = float(row["t_sim"]) if row["t_sim"] else float(start_idx + local_i) * DT

            n_gated_before = ekf.n_gated
            ekf.lambda_filter.correct(lam_meas, t)
            ekf.range_filter.correct(range_m, t)
            tick["fresh"] = True

            if was_initialized:
                tick["attempted"] = True
                gated = ekf.n_gated > n_gated_before
                tick["gated"] = gated
                tick["nis"] = ekf.last_nis

                if gated:
                    consec_gated += 1
                else:
                    consec_gated = 0

                if recover and consec_gated >= n_consec:
                    ekf.P = np.diag([
                        INIT_POS_SIGMA_M ** 2, INIT_POS_SIGMA_M ** 2,
                        INIT_VEL_SIGMA_M_S ** 2, INIT_VEL_SIGMA_M_S ** 2,
                    ])
                    tick["reinflated_after"] = True
                    recovery_events.append({
                        "at_local_i": local_i,
                        "streak_len": consec_gated,
                        "next_attempt_local_i": None,
                        "next_attempt_gated": None,
                        "next_attempt_nis": None,
                    })
                    consec_gated = 0
            # else: cold-init "correction" -- bypasses the gate entirely,
            # not eligible for streak counting (matches ekf_q_replay).

        tick["lambda_hat_post_deg"] = math.degrees(ekf.lambda_hat) if ekf.initialized else None
        tick["p_trace_pos"] = float(ekf.P[0, 0] + ekf.P[1, 1]) if ekf.P is not None else None
        timeline.append(tick)

    attempts = [t for t in timeline if t["attempted"]]
    for ev in recovery_events:
        nxt = [t for t in attempts if t["local_i"] > ev["at_local_i"]]
        if nxt:
            ev["next_attempt_local_i"] = nxt[0]["local_i"]
            ev["next_attempt_gated"] = nxt[0]["gated"]
            ev["next_attempt_nis"] = nxt[0]["nis"]

    return timeline, recovery_events


# =====================================================================
# Cascade analysis (per flight)
# =====================================================================
def analyze_flight(timeline, category):
    n = len(timeline)
    detected_idx = [t["local_i"] for t in timeline if t["detected_raw"]]
    attempts = [t for t in timeline if t["attempted"]]

    # (a) max consecutive-rejection streak over ATTEMPTED corrections, in
    # temporal order (gaps with no attempt do not break a streak; only an
    # ACCEPTED attempt resets it -- exactly the counter replay_timeline
    # itself uses for the D3.2 trigger).
    max_streak = 0
    max_streak_range = None
    cur = 0
    cur_start = None
    for t in attempts:
        if t["gated"]:
            cur += 1
            if cur == 1:
                cur_start = t["local_i"]
            if cur > max_streak:
                max_streak = cur
                max_streak_range = (cur_start, t["local_i"])
        else:
            cur = 0

    # Fatal-gap / trailing dropout: the tail run of detected_raw==False
    # through the end of the replay window. For the 6 aborted flights this
    # IS the >5s gap that triggers LOST_TAG_ABORT_S (D1b/ekf_q_replay note:
    # aborted flights never reach BREAKOFF, so end_idx is already their last
    # row). For controls this is the much shorter DESIGNED terminal-dropout
    # tail -- computed identically, for direct comparison.
    i = n - 1
    while i >= 0 and not timeline[i]["detected_raw"]:
        i -= 1
    gap_start = i + 1
    gap_ticks = n - gap_start
    t_gap_start = timeline[gap_start]["t"] if gap_start < n else None
    t_end = timeline[-1]["t"]
    gap_wall_s = (t_end - t_gap_start) if (t_gap_start is not None and t_end is not None) else None

    # (b) rejections in the last LAST_N_TICKS *detected* ticks before the gap
    last_detected_before_gap = [i for i in detected_idx if i < gap_start][-LAST_N_TICKS:]
    last_window_attempts = [t for t in attempts if t["local_i"] in last_detected_before_gap]
    last_window_gated = sum(1 for t in last_window_attempts if t["gated"])

    # (c) lambda_hat error at last accepted vs last attempted (whether or
    # not it was accepted -- "last available measurement") vs at the start
    # and end of the fatal/trailing gap (drift during the coast).
    accepted = [t for t in attempts if not t["gated"]]
    last_accepted = accepted[-1] if accepted else None
    last_attempt = attempts[-1] if attempts else None

    err_last_accepted = ang_diff_deg(
        last_accepted["lambda_hat_post_deg"], last_accepted["lambda_true_deg"]
    ) if last_accepted else None
    err_last_attempt = ang_diff_deg(
        last_attempt["lambda_hat_post_deg"], last_attempt["lambda_true_deg"]
    ) if last_attempt else None
    err_gap_start = ang_diff_deg(
        timeline[gap_start]["lambda_hat_post_deg"], timeline[gap_start]["lambda_true_deg"]
    ) if gap_start < n else None
    err_window_end = ang_diff_deg(
        timeline[-1]["lambda_hat_post_deg"], timeline[-1]["lambda_true_deg"]
    )
    drift = (
        (err_window_end - err_gap_start)
        if (err_window_end is not None and err_gap_start is not None) else None
    )

    # Cascade signature: a real (>=2) consecutive-reject streak whose LAST
    # rejection lands within CASCADE_PROXIMITY_TICKS ticks of the fatal gap
    # start (i.e. "immediately before" the dropout, not scattered earlier).
    cascade_signature = (
        max_streak >= 2
        and max_streak_range is not None
        and 0 <= (gap_start - max_streak_range[1]) <= CASCADE_PROXIMITY_TICKS
    )

    return {
        "n_ticks": n,
        "n_detected_raw": len(detected_idx),
        "n_attempts": len(attempts),
        "n_gated": sum(1 for t in attempts if t["gated"]),
        "max_consec_reject_streak": max_streak,
        "max_streak_local_i_range": max_streak_range,
        "gap_start_local_i": gap_start,
        "gap_ticks": gap_ticks,
        "gap_wall_s": gap_wall_s,
        "last10_detected_considered": len(last_detected_before_gap),
        "last10_gated_count": last_window_gated,
        "lambda_hat_err_last_accepted_deg": err_last_accepted,
        "lambda_hat_err_last_attempt_deg": err_last_attempt,
        "lambda_hat_err_gap_start_deg": err_gap_start,
        "lambda_hat_err_window_end_deg": err_window_end,
        "lambda_hat_drift_over_gap_deg": drift,
        "cascade_signature": cascade_signature,
    }


# =====================================================================
# Main
# =====================================================================
def build_flight_records():
    """[(arm, run_idx, category, rows, start_idx, end_idx)] for all 16."""
    ekf_flights = load_arm_flights("ekf")
    ab_flights = load_arm_flights("alphabeta")
    aborted_run_idxs = sorted(r for r, _, clean in ekf_flights if not clean)
    clean_ekf_run_idxs = sorted(r for r, _, clean in ekf_flights if clean)
    print(f"[ekf_lockout_forensics] ekf-arm aborted (clean=0) run_idx: {aborted_run_idxs} "
          f"(expect 6, ADR-0037: 6/8 aborted 'lost tag >5.0s')")
    print(f"[ekf_lockout_forensics] ekf-arm clean (clean=1) run_idx: {clean_ekf_run_idxs} "
          f"(expect 2)")
    assert len(aborted_run_idxs) == 6, "expected exactly 6 aborted ekf-arm flights per ADR-0037"
    assert len(clean_ekf_run_idxs) == 2, "expected exactly 2 clean ekf-arm flights per ADR-0037"

    records = []
    for run_idx, path, clean in ekf_flights:
        rows = load_flight_rows(path)
        window = determine_replay_window(rows)
        assert window is not None, f"ekf run {run_idx}: no usable replay window"
        start_idx, end_idx = window
        category = "aborted_ekf" if not clean else "clean_ekf"
        records.append(("ekf", run_idx, category, path, rows, start_idx, end_idx))
    for run_idx, path, clean in ab_flights:
        rows = load_flight_rows(path)
        window = determine_replay_window(rows)
        assert window is not None, f"alphabeta run {run_idx}: no usable replay window"
        start_idx, end_idx = window
        assert clean, "expected all 8 alphabeta-arm flights clean per ADR-0037"
        records.append(("alphabeta", run_idx, "alphabeta_control", path, rows, start_idx, end_idx))
    return records


def fmt(x, spec="{:.2f}"):
    return "n/a" if x is None else spec.format(x)


def print_verdict_table(rows_out):
    header = (
        f"{'arm':>10} | {'run':>3} | {'category':>12} | {'abort?':>6} | "
        f"{'n_att':>5} | {'n_gate':>6} | {'max_streak':>10} | {'streak_end->gap':>15} | "
        f"{'last10_gated':>12} | {'cascade?':>8} | {'err_last_acc_deg':>16} | "
        f"{'err_gap_start_deg':>17} | {'err_end_deg':>11} | {'drift_deg':>9} | {'gap_s':>6}"
    )
    print(header)
    print("-" * len(header))
    for r in rows_out:
        m = r["metrics"]
        streak_gap = (
            m["gap_start_local_i"] - m["max_streak_local_i_range"][1]
            if m["max_streak_local_i_range"] is not None else None
        )
        print(
            f"{r['arm']:>10} | {r['run_idx']:>3} | {r['category']:>12} | "
            f"{'YES' if r['category']=='aborted_ekf' else 'no':>6} | "
            f"{m['n_attempts']:>5} | {m['n_gated']:>6} | {m['max_consec_reject_streak']:>10} | "
            f"{fmt(streak_gap, '{:.0f}') if streak_gap is not None else 'n/a':>15} | "
            f"{m['last10_gated_count']:>12} | "
            f"{'YES' if m['cascade_signature'] else 'no':>8} | "
            f"{fmt(m['lambda_hat_err_last_accepted_deg']):>16} | "
            f"{fmt(m['lambda_hat_err_gap_start_deg']):>17} | "
            f"{fmt(m['lambda_hat_err_window_end_deg']):>11} | "
            f"{fmt(m['lambda_hat_drift_over_gap_deg']):>9} | "
            f"{fmt(m['gap_wall_s'], '{:.1f}'):>6}"
        )


TERMINAL_RANGE_M = 3.0    # m4_intercept.py:294 -- inside this, a dropout gets the lenient
                          # TERMINAL_HOLD_MAX_S=1.0s hold; outside it, the harsh
                          # LOST_TAG_ABORT_S=5.0s "far from target" clock applies instead.


def terminal_range_branch_check(rows):
    """SURPRISE FINDING, added after the cascade numbers came back essentially at
    chance (see main()). Not part of the original hypothesis -- a follow-up
    once the gate/cascade signal turned out not to discriminate at all.

    Reads the PRODUCTION-LOGGED `t` (wall) and `r_hat_m` columns directly
    (NOT this script's own replay -- the actual number the live guidance
    loop's branch check (`m4_intercept.py`'s dropout `if r_hat <
    TERMINAL_RANGE_M`) saw) and asks a much narrower question than the
    cascade hypothesis: after the LAST detected tick, does the filter's own
    coasted r_hat estimate stay below TERMINAL_RANGE_M=3.0m (the lenient
    1.0s-hold branch) or blow through it and STAY above (the harsh 5.0s
    'far from target' branch) within the first second of losing the tag?
    That branch choice, not any gate rejection, is what LOST_TAG_ABORT_S is
    gated on. Reports r_hat at the last-detected tick and at +1.0s wall
    time after it (the TERMINAL_HOLD_MAX_S boundary)."""
    det_idx = [i for i, r in enumerate(rows) if r["detected"] == "1"]
    if not det_idx:
        return None
    last_det = max(det_idx)
    t_last = float(rows[last_det]["t"])
    r_hat_last = float(rows[last_det]["r_hat_m"]) if rows[last_det]["r_hat_m"] else None

    r_hat_at_1s = None
    for j in range(last_det, len(rows)):
        if float(rows[j]["t"]) - t_last >= 1.0:
            v = rows[j]["r_hat_m"]
            r_hat_at_1s = float(v) if v else None
            break

    # "sustained" cross: r_hat_m stays >= TERMINAL_RANGE_M for every tick from
    # first crossing through +1.0s (a real transition, not a 1-tick noise blip
    # that the branch logic's un-reset dropout_start_mono absorbs harmlessly).
    sustained_far = r_hat_at_1s is not None and r_hat_at_1s >= TERMINAL_RANGE_M

    return {
        "last_detected_row_idx": last_det,
        "t_last_detected": t_last,
        "r_hat_at_last_detected": r_hat_last,
        "r_hat_at_plus_1s": r_hat_at_1s,
        "sustained_far_branch_at_1s": sustained_far,
    }


def main():
    print("[ekf_lockout_forensics] loading flights + replay windows (reusing ekf_q_replay's "
          "loader/window logic, unmodified)")
    records = build_flight_records()

    rows_out = []
    all_timelines = {}
    for arm, run_idx, category, path, rows, start_idx, end_idx in records:
        timeline, _ = replay_timeline(rows, start_idx, end_idx, q_psd=Q_PROD, recover=False)
        metrics = analyze_flight(timeline, category)
        rows_out.append({
            "arm": arm, "run_idx": run_idx, "category": category,
            "flight_csv_path": path, "metrics": metrics,
        })
        all_timelines[f"{arm}_{run_idx}"] = timeline

    print(f"\n=== Per-flight cascade forensics (Q={Q_PROD}, production) ===")
    print_verdict_table(rows_out)

    aborted_rows = [r for r in rows_out if r["category"] == "aborted_ekf"]
    control_rows = [r for r in rows_out if r["category"] != "aborted_ekf"]
    n_aborted_cascade = sum(1 for r in aborted_rows if r["metrics"]["cascade_signature"])
    n_control_cascade = sum(1 for r in control_rows if r["metrics"]["cascade_signature"])
    aborted_zero_gate = sum(1 for r in aborted_rows if r["metrics"]["n_gated"] == 0)
    aborted_max_streak = max(r["metrics"]["max_consec_reject_streak"] for r in aborted_rows)
    control_max_streak = max(r["metrics"]["max_consec_reject_streak"] for r in control_rows)
    global_max_streak = max(r["metrics"]["max_consec_reject_streak"] for r in rows_out)

    print(f"\n=== Discrimination check ===")
    print(f"Aborted-ekf flights (n=6) with cascade signature: {n_aborted_cascade}/6")
    print(f"Control flights (clean-ekf + alphabeta, n=10) with cascade signature: {n_control_cascade}/10")
    print(f"Aborted-ekf flights with ZERO gate rejections anywhere in the replay window: "
          f"{aborted_zero_gate}/6")
    print(f"Max consecutive-reject streak observed: aborted-ekf={aborted_max_streak}, "
          f"controls={control_max_streak}, GLOBAL (all 16)={global_max_streak}")

    if n_aborted_cascade >= 4 and n_control_cascade <= 2:
        verdict = ("HYPOTHESIS SUPPORTED: cascade signature clearly separates aborted from "
                   "clean flights.")
    elif n_aborted_cascade == 0:
        verdict = ("HYPOTHESIS DEAD: zero aborted flights show a cascade signature -- the "
                   "gate is not clustering rejections before the fatal dropout in this data.")
    else:
        verdict = ("HYPOTHESIS NOT SUPPORTED / MIXED: cascade signature does not cleanly "
                   "separate aborted from clean flights at this n.")
    print(f"\nVERDICT: {verdict}")

    # --- Surprise follow-up: terminal-range branch check (production-logged
    # r_hat_m, not this script's replay) -- see terminal_range_branch_check()
    # docstring for why this was added after the cascade signal came back
    # near chance. ---
    print(f"\n=== SURPRISE FOLLOW-UP: terminal-range branch check (production r_hat_m/t; "
          f"TERMINAL_RANGE_M={TERMINAL_RANGE_M} m, TERMINAL_HOLD_MAX_S=1.0s vs "
          f"LOST_TAG_ABORT_S=5.0s) ===")
    trb_header = (
        f"{'arm':>10} | {'run':>3} | {'category':>12} | {'abort?':>6} | "
        f"{'r_hat@last_det':>14} | {'r_hat@+1.0s':>11} | {'sustained_far?':>14}"
    )
    print(trb_header)
    print("-" * len(trb_header))
    trb_by_flight = {}
    for arm, run_idx, category, path, rows, start_idx, end_idx in records:
        trb = terminal_range_branch_check(rows)
        trb_by_flight[f"{arm}_{run_idx}"] = trb
        is_abort = category == "aborted_ekf"
        print(
            f"{arm:>10} | {run_idx:>3} | {category:>12} | {'YES' if is_abort else 'no':>6} | "
            f"{fmt(trb['r_hat_at_last_detected']):>14} | {fmt(trb['r_hat_at_plus_1s']):>11} | "
            f"{'YES' if trb['sustained_far_branch_at_1s'] else 'no':>14}"
        )
    n_aborted_sustained = sum(
        1 for arm, run_idx, category, *_ in records
        if category == "aborted_ekf" and trb_by_flight[f"{arm}_{run_idx}"]["sustained_far_branch_at_1s"]
    )
    n_control_sustained = sum(
        1 for arm, run_idx, category, *_ in records
        if category != "aborted_ekf" and trb_by_flight[f"{arm}_{run_idx}"]["sustained_far_branch_at_1s"]
    )
    print(f"\nSustained-far-at-+1.0s: {n_aborted_sustained}/6 aborted-ekf vs "
          f"{n_control_sustained}/10 controls.")
    if n_aborted_sustained == 6 and n_control_sustained == 0:
        trb_verdict = (
            "PERFECT DISCRIMINATOR (6/6 vs 0/10), COMPETING MECHANISM, NOT the gate: whether "
            "the coasted r_hat estimate is above or below TERMINAL_RANGE_M=3.0m ~1s after "
            "losing the tag decides which abort-timer branch the guidance falls into (lenient "
            "1.0s hold vs harsh 5.0s abort) -- driven by each filter's own frozen/extrapolated "
            "range-rate at the moment tracking stopped, independent of any gate rejection."
        )
    else:
        trb_verdict = "Partial/no separation on this check."
    print(f"TERMINAL-RANGE-BRANCH VERDICT: {trb_verdict}")

    # --- D3.2 offline screen ---
    print(f"\n=== D3.2 adaptive gate-recovery mechanism screen (N={N_CONSEC_PRIMARY}, as specified "
          f"in fusion_capstone_design.md D3 item 2) ===")
    d32_events_by_flight = {}
    any_primary_trigger = False
    for arm, run_idx, category, path, rows, start_idx, end_idx in records:
        _, events = replay_timeline(
            rows, start_idx, end_idx, q_psd=Q_PROD, recover=True, n_consec=N_CONSEC_PRIMARY
        )
        key = f"{arm}_{run_idx}"
        d32_events_by_flight[key] = events
        if events:
            any_primary_trigger = True
            print(f"  {key} ({category}): {len(events)} reinflation trigger(s) -> "
                  f"{events}")
    if not any_primary_trigger:
        print(f"  N={N_CONSEC_PRIMARY} NEVER TRIGGERS on any of the 16 flights. Global max "
              f"consecutive-reject streak at Q={Q_PROD} is {global_max_streak} "
              f"(< {N_CONSEC_PRIMARY}), so the D3.2 trigger condition as specified is "
              f"structurally unreachable on this dataset -- the mechanism cannot be screened "
              f"as literally written here.")

    print(f"\n=== Supplementary (exploratory, N={N_CONSEC_SUPPLEMENTARY}): does reinflation "
          f"after ANY single gate rejection unblock the very next attempt? ===")
    d31_events_by_flight = {}
    for arm, run_idx, category, path, rows, start_idx, end_idx in records:
        _, events = replay_timeline(
            rows, start_idx, end_idx, q_psd=Q_PROD, recover=True, n_consec=N_CONSEC_SUPPLEMENTARY
        )
        key = f"{arm}_{run_idx}"
        d31_events_by_flight[key] = events
        if events:
            n_unblocked = sum(1 for e in events if e["next_attempt_gated"] is False)
            n_still_gated = sum(1 for e in events if e["next_attempt_gated"] is True)
            n_no_next = sum(1 for e in events if e["next_attempt_gated"] is None)
            print(f"  {key} ({category}): {len(events)} trigger(s), next-attempt outcome: "
                  f"{n_unblocked} unblocked / {n_still_gated} still gated / "
                  f"{n_no_next} no further attempt in window")

    print(f"\nCAVEAT (verbatim, ekf_q_replay.py's own D1c caveat, applies identically here): "
          f"{CAVEAT}")

    # --- JSON output ---
    result = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "q_psd": Q_PROD,
        "dt_replay_s": DT,
        "n_consec_primary": N_CONSEC_PRIMARY,
        "n_consec_supplementary": N_CONSEC_SUPPLEMENTARY,
        "cascade_proximity_ticks": CASCADE_PROXIMITY_TICKS,
        "last_n_ticks": LAST_N_TICKS,
        "per_flight": [
            {"arm": r["arm"], "run_idx": r["run_idx"], "category": r["category"],
             "flight_csv_path": r["flight_csv_path"], "metrics": r["metrics"]}
            for r in rows_out
        ],
        "discrimination": {
            "n_aborted_with_cascade_signature": n_aborted_cascade,
            "n_aborted_total": 6,
            "n_control_with_cascade_signature": n_control_cascade,
            "n_control_total": 10,
            "n_aborted_with_zero_gate_rejections": aborted_zero_gate,
            "max_consec_streak_aborted": aborted_max_streak,
            "max_consec_streak_control": control_max_streak,
            "max_consec_streak_global": global_max_streak,
            "verdict": verdict,
        },
        "d32_primary_screen": {
            "n_consec": N_CONSEC_PRIMARY,
            "any_trigger": any_primary_trigger,
            "events_by_flight": d32_events_by_flight,
        },
        "d31_supplementary_screen": {
            "n_consec": N_CONSEC_SUPPLEMENTARY,
            "events_by_flight": d31_events_by_flight,
        },
        "terminal_range_branch_check": {
            "note": "SURPRISE follow-up, not part of the original cascade hypothesis -- added "
                    "after the cascade signal came back near chance. Uses PRODUCTION-LOGGED "
                    "r_hat_m/t columns directly, not this script's replay.",
            "terminal_range_m": TERMINAL_RANGE_M,
            "n_aborted_sustained_far": n_aborted_sustained,
            "n_control_sustained_far": n_control_sustained,
            "verdict": trb_verdict,
            "by_flight": trb_by_flight,
        },
        "caveat": CAVEAT,
        "timelines": all_timelines,
    }
    out_path = os.path.join(LOGS_DIR, "ekf_lockout_forensics.json")
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2, default=str)
    print(f"\n[ekf_lockout_forensics] wrote {out_path}")


if __name__ == "__main__":
    main()
