#!/usr/bin/env python3
"""Post-CPA abort reclassification lens (fusion-P0, ADR-0041 2nd addendum).

WHAT THIS IS: an ANALYZER-ONLY lens over existing mc_batch arm CSVs. It never
boots a sim, never edits a flight CSV, and never touches flight code -- it is
exactly the "P0 (i) analyzer-level post-CPA abort reclassification" build item
named in `docs/fusion_capstone_design.md` D1/D4/build-plan, and it operationalizes
the finding written up in `docs/decisions.md`:

  * ADR-0037 addendum (2026-07-08): the ADR-0037 EKF arm's 6 "aborted" flights
    (clean 8/8 -> 2/8) all intercepted NORMALLY (miss 0.675-1.339 m, 3/6 beating
    their paired alpha-beta twin) -- the `LOST_TAG_ABORT_S=5.0` abort fired
    ~5s AFTER the closest-approach pass because the EKF's physically-correct
    post-CPA coasted range estimate grew past the branch that starts the abort
    clock, while alpha-beta's physically-impossible negative coasted range
    never crosses that branch and rides the terminal-hold branch forever
    instead. That is bookkeeping, not a failed intercept.
  * ADR-0041 addendum + `docs/fusion_capstone_design.md` D1: P0 is (i) this
    reclassification lens, applied to ALL arms including a markerless-abort
    re-examination, before (ii) any clean-rate-parity verdict is drawn.

HOW AN "ABORT" ACTUALLY SHOWS UP IN THE PER-TICK CSV (there is no `abort`
or `abort_reason` *column* -- `scripts/m4_intercept.py`'s per-tick writer
never logs one; abort_reason/breakoff_reason are stdout-only / arm-CSV-only
fields). Reverse-engineered against `scripts/m4_intercept.py`'s ENGAGE-phase
state machine (2026-07-08):

  * A `breakoff_reason` branch (hard floor, range-increase-past-CPA, dropout
    *inside* terminal range for > TERMINAL_HOLD_MAX_S, or the ENGAGE timeout)
    explicitly sets `phase = "BREAKOFF"` and the flight logs a BREAKOFF-phase
    tail (`POST_BREAKOFF_LOG_S`) before ending. This is the NORMAL ending path
    -- clean AND many non-clean flights alike end this way (e.g. the apriltag
    control arm is 8/8 clean via exactly this branch).
  * The `LOST_TAG_ABORT_S=5.0` abort (dropout OUTSIDE terminal range, i.e.
    `r_hat >= TERMINAL_RANGE_M` with no detection) sets `aborted = True` and
    `break`s the loop THAT SAME TICK -- phase is NEVER set to "BREAKOFF". An
    `set_velocity_ned` offboard failure does the same. So: **a flight that
    reaches ENGAGE but never logs a single BREAKOFF-phase row is the
    LOST_TAG_ABORT_S/offboard-failure signature; a flight that logs a
    BREAKOFF-phase row ended via the ordinary breakoff branch, not this
    abort.** Verified 2026-07-08 against all 8 `mc_ekfab_ekf.csv` flights:
    the 6 clean=0 rows (whose arm-CSV `breakoff_reason` is literally "n/a",
    i.e. `result["breakoff_reason"]` was never set) are exactly the 6 with
    no BREAKOFF-phase row in their flight CSV; the 2 clean=1 rows both have
    a BREAKOFF-phase tail.

CLEAN_CORRECTED SEMANTICS (per the P0 brief, deliberately conservative):

    clean_corrected = raw_clean OR (aborted AND post_cpa AND min_gt_range <= PK_RADIUS_M)

`PK_RADIUS_M = 2.5` is NOT invented here -- it is the project's own ratified
Pk-vs-lethal-radius sanity bound (ADR-0025's proximity-Pk metric; ADR-0036's
M5 headline reports "Pk@2.5 m 100%" as the outer/net radius the whole FPV
band clears). A post-CPA abort with a large min_gt_range (a real flyby, not a
close pass followed by abort-clock bookkeeping) must NOT be rescued by this
lens -- it is still a failed intercept, full stop. This bound is exactly what
keeps the lens honest.

HONESTY: this script is scoring-only, like every other gt_*-consuming
analyzer in this repo (ADR-0008 lineage). It never feeds anything back into a
guidance path, never edits an existing CSV, and never overwrites flight code.
It reads `gt_range` (ground truth) purely to find the closest-approach tick
and the miss-vs-radius sanity check -- exactly how `ekf_ab_analyze.py` and
`audit_per_tick.py` already use gt_* for scoring.

USAGE:
    python3 scripts/abort_lens.py logs/mc_ekfab_alphabeta.csv logs/mc_ekfab_ekf.csv
    python3 scripts/abort_lens.py logs/mc_ab_apriltag.csv logs/mc_ab_markerless.csv \\
        logs/mc_ab_markerless_tuned.csv --json /tmp/abort_lens_markerless.json

Each positional arg is treated as one "arm" (its own table + arm totals).
Multiple arms can be passed in one invocation; the arm label is the CSV's
basename. Never overwrites any CSV -- output is a printed table plus an
optional --json dump (opt-in, a NEW file only).

Standard library only -- no numpy/scipy (matches the project's other
analysis scripts).
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys

SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPTS_DIR)
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

# Reuse ekf_ab_analyze's arm-CSV loader and float parser verbatim -- do NOT
# reimplement or edit that module (task boundary: integration happens later
# in the main session).
import ekf_ab_analyze as _ekf_ab_analyze  # noqa: E402

load_arm = _ekf_ab_analyze.load_arm
fnum = _ekf_ab_analyze.fnum

# The project's ratified Pk-vs-lethal-radius sanity bound (ADR-0025 proposal,
# ADR-0036 "Pk@2.5 m 100%" headline) -- used here ONLY as the ceiling past
# which a post-CPA abort can no longer be waved off as bookkeeping; it is not
# a claim about kill mechanism, just the honesty bound this lens must respect.
PK_RADIUS_M = 2.5


# --- path resolution (same defensive pattern as audit_per_tick.py) ---------

def _resolve_path(pth):
    if not pth:
        return pth
    return pth if os.path.isabs(pth) else os.path.join(REPO_ROOT, pth)


# --- the lens itself ---------------------------------------------------------

def classify_abort(flight_csv_path):
    """Classify one m4_intercept per-tick flight CSV through the post-CPA
    abort lens.

    Returns a dict:
        flight_csv_path : str (as given)
        ok               : bool -- False if the CSV could not be read/found
        note             : str  -- human-readable classification note
        n_rows           : int | None
        engage_idx       : int | None -- row index of first ENGAGE row, or
                            None if ENGAGE was never reached
        breakoff_seen    : bool | None -- any BREAKOFF-phase row at/after
                            engage_idx (None if engage never reached)
        aborted          : bool | None -- True = ENGAGE reached with NO
                            BREAKOFF-phase row ever logged (the
                            LOST_TAG_ABORT_S / offboard-failure signature,
                            ADR-0037 addendum). False = a BREAKOFF-phase row
                            was logged (ordinary breakoff-branch ending,
                            clean or not). None = ENGAGE never reached (a
                            pre-engage failure; the lens does not apply --
                            there is no post-CPA-bookkeeping story to tell
                            about a flight that never got that far).
        t_cpa            : float | None -- sim time (t_sim) of the minimum
                            gt_range row (closest-approach tick), over ALL
                            rows regardless of phase (mirrors
                            ekf_ab_analyze.flight_metrics's CPA definition).
        min_gt_range      : float | None -- that minimum gt_range (m).
        t_abort_or_end    : float | None -- t_sim of the LAST row in the CSV
                            (the tick the flight ended, aborted or not).
        post_cpa          : bool -- did logging persist strictly past t_cpa
                            before the flight ended (t_abort_or_end > t_cpa)?
                            Computed for every flight (useful context even
                            when aborted is False/None); only load-bearing
                            for clean_corrected when aborted is True.
        r_hat_at_abort    : float | None -- r_hat_m at the LAST logged row --
                            the tracker's own coasted-range readout at the
                            moment the flight ended. This is the "branch
                            evidence": for a LOST_TAG_ABORT_S abort this is
                            expected to sit well past TERMINAL_RANGE_M
                            (the branch whose crossing starts the abort
                            clock), confirming a physically-diverging COAST
                            rather than a live measurement.
    """
    result = {
        "flight_csv_path": flight_csv_path,
        "ok": True,
        "note": "",
        "n_rows": None,
        "engage_idx": None,
        "breakoff_seen": None,
        "aborted": None,
        "t_cpa": None,
        "min_gt_range": None,
        "t_abort_or_end": None,
        "post_cpa": False,
        "r_hat_at_abort": None,
    }

    if not flight_csv_path or not os.path.exists(flight_csv_path):
        result["ok"] = False
        result["note"] = f"flight CSV not found: {flight_csv_path!r}"
        return result

    try:
        with open(flight_csv_path, newline="") as f:
            reader = csv.DictReader(f)
            rows = list(reader)
    except OSError as e:
        result["ok"] = False
        result["note"] = f"could not read {flight_csv_path}: {e}"
        return result

    if not rows:
        result["ok"] = False
        result["note"] = "flight CSV has no data rows"
        return result

    result["n_rows"] = len(rows)

    # ---- CPA / min_gt_range: min(gt_range) over every row, any phase ------
    min_gt, t_cpa = None, None
    for r in rows:
        g = fnum(r.get("gt_range"))
        ts = fnum(r.get("t_sim"))
        if g is None or ts is None:
            continue
        if min_gt is None or g < min_gt:
            min_gt, t_cpa = g, ts
    result["min_gt_range"] = min_gt
    result["t_cpa"] = t_cpa

    # ---- end-of-log evidence -----------------------------------------------
    last = rows[-1]
    t_end = fnum(last.get("t_sim"))
    result["t_abort_or_end"] = t_end
    result["r_hat_at_abort"] = fnum(last.get("r_hat_m"))

    # ---- ENGAGE / BREAKOFF phase evidence -----------------------------------
    engage_idx = None
    for i, r in enumerate(rows):
        if r.get("phase") == "ENGAGE":
            engage_idx = i
            break
    result["engage_idx"] = engage_idx

    if engage_idx is None:
        result["aborted"] = None
        result["breakoff_seen"] = None
        result["note"] = (
            "no ENGAGE-phase row in this flight CSV -- a pre-engage failure "
            "(e.g. acquire timeout during DASH); the post-CPA abort lens "
            "does not apply (there is no engage/breakoff branch to have "
            "mis-fired)."
        )
        result["post_cpa"] = False
        return result

    breakoff_seen = any(r.get("phase") == "BREAKOFF" for r in rows[engage_idx:])
    result["breakoff_seen"] = breakoff_seen
    aborted = not breakoff_seen
    result["aborted"] = aborted

    if aborted:
        result["note"] = (
            "ENGAGE reached, ZERO BREAKOFF-phase rows logged -- the "
            "LOST_TAG_ABORT_S=5.0s (or offboard-failure) abort signature "
            "(ADR-0037 addendum); phase never transitions through a "
            "breakoff branch before the loop breaks."
        )
    else:
        result["note"] = (
            "BREAKOFF-phase row(s) logged after ENGAGE -- ordinary "
            "breakoff-branch ending (hard floor / range-increase / "
            "terminal-dropout / engage-timeout), NOT the LOST_TAG_ABORT_S "
            "abort signature, regardless of clean/not-clean."
        )

    result["post_cpa"] = bool(
        t_cpa is not None and t_end is not None and t_end > t_cpa
    )
    return result


def clean_corrected_verdict(raw_clean, classification):
    """Apply the pre-registered, deliberately conservative rescue rule:

        clean_corrected = raw_clean OR (aborted AND post_cpa AND min_gt_range <= PK_RADIUS_M)

    `raw_clean` is a bool (already resolved from the arm CSV's `clean`
    column). `classification` is a classify_abort() result dict. Returns
    (clean_corrected: bool, eligible_rescue: bool, rescue_reason: str).
    """
    aborted = bool(classification.get("aborted"))
    post_cpa = bool(classification.get("post_cpa"))
    min_gt = classification.get("min_gt_range")

    if raw_clean:
        return True, False, "already raw-clean (lens not needed)"

    if not aborted:
        return False, False, "not the LOST_TAG_ABORT_S signature (breakoff-branch ending, or pre-engage)"

    if not post_cpa:
        return False, False, "aborted but PRE-CPA (real failure, lens does not apply)"

    if min_gt is None:
        return False, False, "aborted + post-CPA but min_gt_range unknown (no gt_range data) -- NOT rescued (conservative)"

    if min_gt > PK_RADIUS_M:
        return False, False, (
            f"aborted + post-CPA but min_gt_range={min_gt:.3f} m > "
            f"PK_RADIUS_M={PK_RADIUS_M} m -- a real flyby/miss, NOT rescued"
        )

    return True, True, (
        f"aborted + post-CPA + min_gt_range={min_gt:.3f} m <= "
        f"PK_RADIUS_M={PK_RADIUS_M} m -- RESCUED (post-CPA bookkeeping, "
        "not a failed intercept)"
    )


# --- arm-level driver --------------------------------------------------------

def analyze_arm(arm_csv_path, law=None):
    """Load one mc_batch arm CSV and run the lens over every flight.

    Returns a dict: arm_label, arm_csv_path, rows (list of per-flight
    result dicts), totals (raw_clean_n, corrected_n, n).
    """
    arm_label = os.path.basename(arm_csv_path)
    arm_rows, warns = load_arm(arm_csv_path, law)

    out_rows = []
    for row in arm_rows:
        fc_path = _resolve_path(row["flight_csv_path"])
        cls = classify_abort(fc_path)
        raw_clean = bool(row["clean"] is not None and row["clean"] >= 0.5)
        cc, eligible, reason = clean_corrected_verdict(raw_clean, cls)
        raw_breakoff_reason = (row["raw"].get("breakoff_reason") or "").strip()
        out_rows.append({
            "run_idx": row["run_idx"],
            "cue_seed": row["cue_seed"],
            "raw_clean": raw_clean,
            "miss_m": row["miss"],
            "flight_csv_path": row["flight_csv_path"],
            "raw_breakoff_reason": raw_breakoff_reason,
            "classification": cls,
            "clean_corrected": cc,
            "eligible_rescue": eligible,
            "rescue_reason": reason,
            "lens_changed_verdict": (cc != raw_clean),
        })

    n = len(out_rows)
    raw_clean_n = sum(1 for r in out_rows if r["raw_clean"])
    corrected_n = sum(1 for r in out_rows if r["clean_corrected"])

    return {
        "arm_label": arm_label,
        "arm_csv_path": arm_csv_path,
        "warnings": warns,
        "rows": out_rows,
        "totals": {"raw_clean_n": raw_clean_n, "corrected_n": corrected_n, "n": n},
    }


# --- printing -----------------------------------------------------------------

def _fmt(v, spec="{:.3f}"):
    return spec.format(v) if isinstance(v, (int, float)) else "-"


def print_arm_table(arm_result):
    rows = arm_result["rows"]
    headers = ["SEED", "RAW_CLEAN", "MISS_M", "ABORTED", "PRE/POST-CPA",
               "MIN_GT_M", "R_HAT_END", "CLEAN_CORR", "CHANGED?"]
    table = []
    for r in rows:
        cls = r["classification"]
        aborted = cls.get("aborted")
        aborted_s = "-" if aborted is None else ("YES" if aborted else "no")
        if aborted is None:
            cpa_s = "n/a (no ENGAGE)"
        elif not aborted:
            cpa_s = "n/a (not aborted)"
        else:
            cpa_s = "POST-CPA" if cls.get("post_cpa") else "PRE-CPA"
        table.append([
            r["cue_seed"] or r["run_idx"],
            "1" if r["raw_clean"] else "0",
            _fmt(r["miss_m"]),
            aborted_s,
            cpa_s,
            _fmt(cls.get("min_gt_range")),
            _fmt(cls.get("r_hat_at_abort")),
            "1" if r["clean_corrected"] else "0",
            "*** CHANGED ***" if r["lens_changed_verdict"] else "",
        ])
    widths = [len(h) for h in headers]
    for row in table:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(str(cell)))
    fmt = "  ".join("{:<" + str(w) + "}" for w in widths)
    print(fmt.format(*headers))
    print(fmt.format(*["-" * w for w in widths]))
    for row in table:
        print(fmt.format(*row))


def print_arm_report(arm_result):
    print("=" * 88)
    print(f"ARM: {arm_result['arm_label']}  ({arm_result['arm_csv_path']})")
    print("=" * 88)
    for w in arm_result["warnings"]:
        print("  [warn]", w)
    print_arm_table(arm_result)

    t = arm_result["totals"]
    print()
    print(f"  clean raw {t['raw_clean_n']}/{t['n']} -> corrected {t['corrected_n']}/{t['n']}")

    changed = [r for r in arm_result["rows"] if r["lens_changed_verdict"]]
    if changed:
        print(f"  {len(changed)} flight(s) where the lens CHANGED the verdict:")
        for r in changed:
            cls = r["classification"]
            print(
                f"    seed={r['cue_seed']} run_idx={r['run_idx']} "
                f"miss={_fmt(r['miss_m'])} m  t_cpa={_fmt(cls.get('t_cpa'))}s "
                f"t_end={_fmt(cls.get('t_abort_or_end'))}s "
                f"min_gt={_fmt(cls.get('min_gt_range'))} m  "
                f"r_hat_at_abort={_fmt(cls.get('r_hat_at_abort'))} m  "
                f"-> {r['rescue_reason']}"
            )
    else:
        print("  0 flights where the lens changed the verdict.")

    # Also surface non-clean flights the lens explicitly declined to rescue,
    # so "is it pre-CPA (real failure) or post-CPA bookkeeping?" is answered
    # for EVERY non-clean flight, not just the ones that flipped.
    not_rescued = [
        r for r in arm_result["rows"]
        if not r["raw_clean"] and not r["clean_corrected"]
    ]
    if not_rescued:
        print(f"  {len(not_rescued)} non-clean flight(s) NOT rescued by the lens "
              "(real failures, per the lens):")
        for r in not_rescued:
            cls = r["classification"]
            print(
                f"    seed={r['cue_seed']} run_idx={r['run_idx']} "
                f"miss={_fmt(r['miss_m'])} m  aborted={cls.get('aborted')}  "
                f"min_gt={_fmt(cls.get('min_gt_range'))} m  "
                f"breakoff_reason={r['raw_breakoff_reason'] or 'n/a'!r}  "
                f"-> {r['rescue_reason']}"
            )
    print()


def to_jsonable(arm_results):
    # classification dicts are already JSON-safe (str/float/bool/None); the
    # only nested structure is arm_result["rows"][i]["classification"].
    return arm_results


# --- CLI ----------------------------------------------------------------------

def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("arm_csvs", nargs="+",
                    help="one or more mc_batch arm CSVs, e.g. "
                         "logs/mc_ekfab_alphabeta.csv logs/mc_ekfab_ekf.csv")
    p.add_argument("--law", default=None,
                    help="filter each arm to this guidance law (default: no "
                         "filter -- every row in the arm CSV; the arms this "
                         "tool is built for are pronav-only anyway).")
    p.add_argument("--json", dest="json_out", default=None,
                    help="optional path to write the full per-flight "
                         "classification as JSON (NEW file only; never "
                         "overwrites an existing CSV).")
    args = p.parse_args(argv)

    print("=" * 88)
    print("POST-CPA ABORT RECLASSIFICATION LENS  (fusion-P0, ADR-0041 2nd "
          "addendum / ADR-0037 addendum)")
    print(f"PK_RADIUS_M = {PK_RADIUS_M}  (ratified Pk-vs-lethal-radius sanity "
          "bound, ADR-0025/ADR-0036 -- a post-CPA abort past this range is "
          "NOT rescued, it is a real miss)")
    print("clean_corrected = raw_clean OR (aborted AND post_cpa AND "
          "min_gt_range <= PK_RADIUS_M)")
    print("Analyzer-only: reads existing arm/flight CSVs, writes nothing, "
          "boots no sim, changes no flight code.")
    print("=" * 88)
    print()

    arm_results = []
    any_missing = False
    for arm_path_arg in args.arm_csvs:
        arm_path = os.path.abspath(arm_path_arg)
        if not os.path.exists(arm_path):
            print(f"[abort_lens] ERROR: arm CSV not found: {arm_path_arg}")
            any_missing = True
            continue
        arm_result = analyze_arm(arm_path, args.law)
        arm_results.append(arm_result)
        print_arm_report(arm_result)

    print("=" * 88)
    print("SUMMARY (all arms)")
    print("=" * 88)
    for ar in arm_results:
        t = ar["totals"]
        print(f"  {ar['arm_label']:32s}  clean raw {t['raw_clean_n']}/{t['n']} "
              f"-> corrected {t['corrected_n']}/{t['n']}")

    if args.json_out:
        out_path = os.path.abspath(args.json_out)
        if os.path.exists(out_path):
            print(f"\n[abort_lens] ERROR: --json path already exists, refusing "
                  f"to overwrite: {out_path}")
            return 2
        with open(out_path, "w") as f:
            json.dump(to_jsonable(arm_results), f, indent=2, default=str)
        print(f"\n[abort_lens] wrote JSON: {out_path}")

    return 2 if any_missing else 0


if __name__ == "__main__":
    sys.exit(main())
