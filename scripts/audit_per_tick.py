#!/usr/bin/env python3
"""Standalone per-tick honesty audit -- extracted from scripts/check_s2.sh,
generalized to sweep EVERY flight in an mc_batch arm CSV.

WHY THIS EXISTS: docs/next.md records the debt "Full per-tick check owed at A/B
close" -- scripts/check_s2.sh's audit_csv() audits exactly the one flight it
just flew (it boots a fresh sim, flies, then audits), and scripts/mc_batch.sh
NEVER calls it: batch arms only ever get the S2_RESULT summary line
(miss/clean/handoff), never a per-tick honesty check. This tool pays that
debt offline: point it at one or more mc_batch arm CSVs (e.g.
logs/mc_ab_markerless_tuned.csv) and it resolves + audits every flight's
per-tick CSV the same way check_s2.sh would have, one at a time, after the
fact. It NEVER boots a sim and NEVER edits scripts/check_s2.sh (that script
is a gated milestone script and is left untouched -- see repo CLAUDE.md).

THE AUDIT LOGIC ITSELF IS A FAITHFUL PORT of check_s2.sh's `audit_csv` bash
function's embedded python heredoc (scripts/check_s2.sh, the `audit_csv()`
function, as of 2026-07-08) -- see `audit_flight_csv()` below. Same checks,
same thresholds, same pass/fail semantics:

  (a) zero non-empty ext_* cells at/after the first ENGAGE row (the cue
      channel must stay latched closed forever once HANDOFF triggers), AND
      every non-detected ENGAGE row exactly holds the previous cmd_v* triple
      or hovers (0,0,0) -- FAIL-able.
  (b) >= 1 DASH row precedes the first ENGAGE row (proves the running-start
      phase actually ran) -- FAIL-able.
  (c) commanded-velocity-azimuth vs camera lambda_deg correlation, on ENGAGE
      rows carrying a camera detection, >= a law-aware bound (pip 0.55, else
      0.7 -- ADR-0013) -- FAIL-able. Rows inside the LATCHED terminal coast
      (identified by the trailing run of byte-identical nonzero cmd_vn/cmd_ve
      -- the frozen vector re-issued verbatim) are EXCLUDED and counted:
      during the ADR-0023 one-way freeze the command is frozen by design
      while lambda keeps evolving, so correlating it tests the freeze, not
      the camera coupling (found 2026-07-25 on the first re-fly arm through
      the fixed zero-command latch; check (e) still gates that the latched
      vector is nonzero).
  (e) THE TERMINAL ACTUALLY ACTUATED: of the detected, pre-CPA (vc > 0)
      ENGAGE ticks, at most half may have commanded ZERO horizontal velocity
      -- FAIL-able. NOT a port from check_s2.sh; added 2026-07-25 because
      this tool was ANTI-CORRELATED with the defect it should have caught.
      scripts/m4_intercept.py's terminal freeze latched an unset (0.0, 0.0)
      whenever ENGAGE began inside the freeze range (the normal markerless
      case), so the whole ENGAGE phase commanded zero horizontal velocity --
      braking, not steering -- across 29/44 engaged flights of five arms.
      Check (c) MUST drop zero-command rows (a zero vector has no azimuth),
      so on exactly those flights it dropped nearly every row, fell under its
      "n < 10, underpowered" branch, stopped gating, and this tool printed
      "PASS: N/N audited flights passed (a)/(b)/(c)". Check (e) gates on the
      discarded denominator, and every report now prints it (ZEROCMD column).
  (d) residual-leak correlation corr(d_cmd, d_gt) -- ADVISORY ONLY, never
      gates pass/fail. check_s2.sh's own calibration (offline over every
      historical S2 flight) found honest flights span roughly -0.99..+0.92
      on this metric, so no numeric bound separates "honest" from "leaking"
      at this sample size; see the comment block above (d) in
      audit_flight_csv() for the full reasoning, ported verbatim.

DEVIATION FROM check_s2.sh (documented, deliberate -- the ONLY behavioral
deviation): check_s2.sh always audits a flight it JUST flew itself, so "no
ENGAGE rows found in this CSV" is an unconditional hard FAIL there -- that
can never legitimately happen in its use case (m4_intercept.py --handoff
either reaches ENGAGE or the run is broken). A batch flight is different: an
mc_batch arm can legitimately BREAKOFF/abort before ever reaching ENGAGE
(e.g. lost the target during DASH) -- that is a valid, honest batch OUTCOME,
not a bug in the flight or in the audit. So when this tool finds a flight
CSV with no ENGAGE row, it reports SKIPPED(no-engage), not FAIL, and that
flight does not count against the arm-level verdict either way (excluded
from both the pass and fail tallies, per the task brief). Every other check
-- thresholds, tolerances, correlation bounds, the (a)-extended
non-detected-hold check, advisory-only (d) -- is the same logic as
check_s2.sh, just translated from the embedded bash heredoc (exec'd once per
sim boot) into a plain importable Python function (called in a loop, no sim
involved).

SECOND, SMALLER EXTENSION (defensive, beyond what check_s2.sh needs): because
this tool may see flight CSVs from schemas check_s2.sh never has to worry
about (it only ever audits its own freshly-written CSV), it preflights the
required columns for checks (a)/(b) and reports ERROR (not a false PASS) if
they are entirely absent, and independently WARNs (does not fail) if the
columns needed for (c) or (d) specifically are absent. In THIS repo, as
inspected 2026-07-08, the apriltag/markerless/markerless_tuned arms all
write flight CSVs from the same scripts/m4_intercept.py writer and have an
IDENTICAL column schema (verified against one flight from each arm) -- so
this preflight is a no-op guard for today's data, not a workaround for an
observed mismatch.

RESOLUTION OF flight_csv_path FROM AN ARM CSV: reuses
scripts/ekf_ab_analyze.py's `load_arm()` (imported, not reimplemented) to
read run_idx/law/flight_csv_path/clean/handoff/miss_m out of an mc_batch arm
CSV row. Unlike ekf_ab_analyze.py's own callers (which filter to one law,
e.g. --law pronav, because the EKF A/B arms are pronav-only), this tool
calls `load_arm(path, law=None)` to keep EVERY row regardless of law, since
the point here is to sweep every flight in the arm. If importing
ekf_ab_analyze.py fails for any reason, an inline replica of the same
column-reading logic is used instead (see `_load_arm_rows_inline()`); the
tool prints which resolution path it took at the top of its output.

USAGE:
    # Sweep every flight in one or more mc_batch arm CSVs:
    python3 scripts/audit_per_tick.py logs/mc_ab_apriltag.csv \\
        logs/mc_ab_markerless.csv logs/mc_ab_markerless_tuned.csv

    # Audit a single flight CSV directly (repeatable flag); law is inferred
    # from the CSV's own `law` column unless --law overrides it:
    python3 scripts/audit_per_tick.py --flight-csv logs/m4_intercept_pip_20260708T005935Z.csv

FLIGHT STATUS has THREE non-error outcomes, not two (2026-07-25): PASS (every
gating check actually ran and passed), PASS_PARTIAL (failed nothing, but at
least one gating check WARN-skipped and therefore verified nothing -- the run
banner then refuses to claim it), and FAIL. The banner is built by COUNTING
what each check returned, so "N/N passed (a)/(b)/(c)" can no longer be printed
for a run in which (c) gated on zero flights.

Exit code: 0 if every audited flight passes the gating checks it could run
((a)/(b)/(c)/(e); SKIPPED flights don't count either way, (d) never gates) --
a PASS_PARTIAL run still exits 0 but says so in the banner; 1 if any audited
flight FAILS (a)/(b)/(c)/(e) (only in --strict mode, the default); 2 on a usage error or a
missing/unreadable arm CSV / flight CSV (a file-existence problem, as
distinct from an audit finding a dishonest flight); 3 if any arm's verdict
is VACUOUS -- zero flights were actually scored (every flight SKIPPED, or
the arm resolved none), so NOTHING WAS VERIFIED and that must never read
as PASS (docs/error_handling_policy.md "no vacuous verdicts"; strict mode
only -- --no-strict still prints the VACUOUS banner but exits 0).

Standard library only -- no numpy/scipy/pandas (matches the project's other
analysis scripts, e.g. ekf_ab_analyze.py).
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys

# --- import (or replicate) ekf_ab_analyze.py's arm-CSV resolution logic ----

SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPTS_DIR)
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

try:
    import ekf_ab_analyze as _ekf_ab_analyze  # noqa: E402
    _RESOLVER_SOURCE = "imported scripts/ekf_ab_analyze.py:load_arm()"
except Exception as _import_exc:  # pragma: no cover -- defensive fallback only
    _ekf_ab_analyze = None
    _RESOLVER_SOURCE = (
        "replicated inline in audit_per_tick.py "
        f"(import of ekf_ab_analyze.py failed: {_import_exc})"
    )


def _load_arm_rows_inline(path):
    """Fallback replica of ekf_ab_analyze.load_arm()'s column reading, used
    only if importing ekf_ab_analyze.py fails. Deliberately minimal -- just
    the columns this tool needs (run_idx, law, flight_csv_path, clean,
    handoff, miss_m)."""
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for raw in reader:
            rows.append({
                "run_idx": (raw.get("run_idx") or "").strip(),
                "law": (raw.get("law") or "").strip(),
                "clean": (raw.get("clean") or "").strip(),
                "handoff": (raw.get("handoff") or "").strip(),
                "miss_m": (raw.get("miss_m") or "").strip(),
                "flight_csv_path": (raw.get("flight_csv_path") or "").strip(),
            })
    return rows


def load_arm_flights(arm_csv_path):
    """Every flight row in an mc_batch arm CSV, LAW-UNFILTERED (we sweep
    every flight regardless of law; audit_flight_csv() picks the correct
    per-row correlation bound from each row's own `law` field, mirroring how
    check_s2.sh picks its bound from the law it just flew)."""
    if _ekf_ab_analyze is not None:
        raw_rows, _warns = _ekf_ab_analyze.load_arm(arm_csv_path, law=None)
        out = []
        for r in raw_rows:
            out.append({
                "run_idx": r["run_idx"],
                "law": (r["raw"].get("law") or "").strip(),
                "clean": r["raw"].get("clean", ""),
                "handoff": r["raw"].get("handoff", ""),
                "miss_m": r["raw"].get("miss_m", ""),
                "flight_csv_path": r["flight_csv_path"],
            })
        return out
    return _load_arm_rows_inline(arm_csv_path)


def _resolve_path(pth):
    """Defensive extra beyond ekf_ab_analyze.py (which assumes flight_csv_path
    is already absolute -- true in every arm CSV inspected 2026-07-08, since
    m4_intercept.py's LOGS_DIR is built from its own file's absolute
    location): if a path ever shows up relative, resolve it against the repo
    root rather than crash."""
    if not pth:
        return pth
    return pth if os.path.isabs(pth) else os.path.join(REPO_ROOT, pth)


def infer_law_from_flight_csv(csv_path):
    """Single-flight mode has no arm row to read `law` from -- pull it from
    the flight CSV's own per-row `law` column instead (first non-empty
    value)."""
    try:
        with open(csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for r in reader:
                law = (r.get("law") or "").strip()
                if law:
                    return law
    except OSError:
        pass
    return ""


# --- the audit itself: faithful port of check_s2.sh's audit_csv() ----------

HOLD_TOL = 1e-6

# --- check (e) "the terminal actually ACTUATED" ----------------------------
# A commanded horizontal velocity whose magnitude is below this is ZERO: not a
# small steering command, an absent one. Same 1e-6 the (c)/(d) loops already
# used to drop such rows -- named here because it is now a MEASURED quantity,
# not a silent filter.
ZERO_CMD_TOL = 1e-6
# FAIL when MORE THAN this fraction of the (e) population commanded zero.
# Rationale for 0.5: the frozen_vworld defect (m4_intercept.py, found
# 2026-07-25) put 29/44 engaged flights above 75% zero-command and several at
# 99%, while an honest terminal steers on essentially every detected tick --
# the two populations are nowhere near this boundary, so the threshold is not
# load-bearing to a tenth.
ZERO_CMD_FAIL_FRAC = 0.5
# ... and only when the population is big enough to mean anything. Below this
# the check REPORTS and refuses to gate (WARN) -- it never silently passes.
ZERO_CMD_MIN_N = 10

# Columns required to run checks (a)/(b) at all. If entirely absent from the
# CSV's header, this is a schema mismatch this tool cannot audit -- reported
# as ERROR rather than a false PASS (check_s2.sh never hits this path since
# it only ever audits the CSV it just wrote with a known-current schema).
REQUIRED_COLUMNS_AB = [
    "phase", "detected", "ext_x", "ext_y", "ext_z", "ext_fresh",
    "cmd_vn", "cmd_ve", "cmd_vd",
]
# Additional columns needed for (c); missing -> WARN-skip (c), not ERROR.
REQUIRED_COLUMNS_C = ["lambda_deg"]
# Additional columns needed for (d); missing -> WARN-skip (d) (already
# advisory-only, so this just means "no advisory number this flight").
REQUIRED_COLUMNS_D = ["gt_cam_x", "gt_cam_y", "gt_tag_x", "gt_tag_y"]

# Phase labels that count as the running-start "dash" for checks (a)/(b).
# check_s2.sh's S2/--handoff pipeline emits "DASH"; the coded-dash flight path
# (scripts/m4_intercept.py --coded-dash, the real-build architecture) emits
# "CODED_DASH" for the same running-start phase. Both must satisfy (b) "a dash
# ran before ENGAGE" and grant the (a) one-tick DASH->ENGAGE transition grace,
# else every coded-dash flight FAILs (b) spuriously on a phase-name mismatch,
# not a real honesty violation (tool-scope fix, 2026-07-22).
DASH_PHASES = ("DASH", "CODED_DASH")

# The checks that may gate a flight's pass/fail. (d) is advisory BY DESIGN
# (no bound separates honest from leaking at this sample size) and is
# deliberately absent. Anything listed here that did not return PASS/FAIL on a
# given flight makes that flight PASS_PARTIAL, never PASS.
GATING_CHECKS = ("a", "b", "c", "e")


def audit_flight_csv(csv_path, law):
    """Faithful port of check_s2.sh's audit_csv() embedded python heredoc.
    Does NOT print -- returns a result dict (schema below) for the caller to
    format. `law` selects the (c) correlation bound: 0.55 for "pip", 0.7 for
    anything else (including "pronav" or unknown) -- exactly check_s2.sh's
    `bound = 0.55 if law == "pip" else 0.7`.

    Returns:
        {
          "csv_path": str, "law": str,
          "status": "PASS" | "PASS_PARTIAL" | "FAIL" | "SKIPPED" | "ERROR",
          "engage_idx": int | None, "n_rows": int | None,
          "checks": {"a": {...}, "b": {...}, "c": {...}, "d": {...},
                     "e": {...}},
          "fail_reasons": [str, ...],   # non-advisory failure detail lines
          "notes": [str, ...],          # PASS_PARTIAL: what was NOT enforced
          "counts": {...},              # (e) denominators (n_e_pop, n_zerocmd, ...)
          "gated_checks": [...], "ungated_checks": [...],
        }

    PASS_PARTIAL means "did not fail anything, but at least one gating check
    never ran" -- see the status block at the end of this function.
    """
    result = {
        "csv_path": csv_path,
        "law": law,
        "status": None,
        "engage_idx": None,
        "n_rows": None,
        "checks": {},
        "fail_reasons": [],
        "notes": [],            # non-failing things the reader MUST still see
        "counts": {},           # (e) denominators -- never absorbed silently
        "gated_checks": [],     # which of GATING_CHECKS actually returned PASS/FAIL
        "ungated_checks": [],
    }

    if not csv_path or not os.path.exists(csv_path):
        result["status"] = "ERROR"
        result["fail_reasons"].append(f"flight CSV not found: {csv_path!r}")
        return result

    try:
        with open(csv_path, newline="") as f:
            reader = csv.DictReader(f)
            cols = set(reader.fieldnames or [])
            rows = list(reader)
    except OSError as e:
        result["status"] = "ERROR"
        result["fail_reasons"].append(f"could not read {csv_path}: {e}")
        return result

    result["n_rows"] = len(rows)

    missing_ab = [c for c in REQUIRED_COLUMNS_AB if c not in cols]
    if missing_ab:
        result["status"] = "ERROR"
        result["fail_reasons"].append(
            "CSV schema missing required column(s) for checks (a)/(b): "
            + ", ".join(missing_ab)
        )
        return result

    engage_idx = None
    for i, r in enumerate(rows):
        if r.get("phase") == "ENGAGE":
            engage_idx = i
            break

    if engage_idx is None:
        # DEVIATION from check_s2.sh -- see module docstring "DEVIATION"
        # section. check_s2.sh treats this as an unconditional hard FAIL
        # because it always just flew the flight itself; a batch flight can
        # legitimately abort/BREAKOFF before ENGAGE, which is a valid
        # outcome, not a bug -- so this is SKIPPED, not FAIL.
        result["status"] = "SKIPPED"
        result["fail_reasons"].append(
            "no ENGAGE phase rows found (abort before engage -- legitimate "
            "batch outcome; not audited, does not count against the arm "
            "verdict either way)"
        )
        return result

    result["engage_idx"] = engage_idx
    ok = True

    # ---- (a) ext_* silence + non-detected hold/hover, at/after ENGAGE -----
    bad_rows = [
        i for i, r in enumerate(rows[engage_idx:], start=engage_idx)
        if r.get("ext_x") or r.get("ext_y") or r.get("ext_z")
        or r.get("ext_fresh") not in ("", "0")
    ]

    # (a) EXTENDED (ported from check_s2.sh's 2026-07-06 addition): a
    # non-detected ENGAGE tick must EXACTLY hold the prior row's cmd_v* or
    # hover at (0,0,0) -- literal value reuse / hardcoded constants, never
    # recomputed from any source. NOTE (inherited quirk, preserved
    # faithfully): if engage_idx == 0 (ENGAGE is literally the first CSV
    # row), `rows[i - 1]` wraps to the LAST row of the file, same as the
    # original bash heredoc's `prev = rows[i - 1]` -- this can't happen for
    # a real S2/M4 flight (TAKEOFF/CUE_WAIT/DASH always precede ENGAGE), and
    # is not "fixed" here so the port stays byte-for-byte faithful.
    # DEPLOYMENT-PROFILE CALIBRATION (adjudicated 2026-07-08, main session):
    # the FIRST ENGAGE row after a DASH row may carry the dash controller's
    # final steering command (its per-tick evolution is ~0.03-0.14 m/s; the
    # one observed "violation" across 24 batch flights was exactly this row,
    # then held EXACTLY from the next tick on). That command is computed
    # pre-latch from legal cue+tracker state -- a one-tick phase-boundary
    # carry, not a post-handoff read -- so it gets a one-row grace. Possible
    # only under --early-handoff (latch without a same-tick detection).
    bad_nondet_rows = []
    n_transition_grace = 0
    for i, r in enumerate(rows[engage_idx:], start=engage_idx):
        if r.get("phase") != "ENGAGE" or r.get("detected") == "1":
            continue
        prev = rows[i - 1]
        if prev.get("phase") in DASH_PHASES:
            n_transition_grace += 1
            continue
        try:
            cur = tuple(float(r[k]) for k in ("cmd_vn", "cmd_ve", "cmd_vd"))
            prv = tuple(float(prev[k]) for k in ("cmd_vn", "cmd_ve", "cmd_vd"))
        except (KeyError, ValueError):
            bad_nondet_rows.append((i, "missing/unparseable cmd_v* cell"))
            continue
        is_hold = all(abs(cur[k] - prv[k]) < HOLD_TOL for k in range(3))
        is_hover = all(abs(v) < HOLD_TOL for v in cur)
        if not (is_hold or is_hover):
            bad_nondet_rows.append((
                i,
                f"cmd=({cur[0]:.4f},{cur[1]:.4f},{cur[2]:.4f}) is neither "
                f"the held previous row's ({prv[0]:.4f},{prv[1]:.4f},"
                f"{prv[2]:.4f}) nor the hover fallback (0,0,0)",
            ))

    if bad_rows or bad_nondet_rows:
        ok = False
        details = []
        if bad_rows:
            details.append(
                f"{len(bad_rows)} rows at/after first ENGAGE (row "
                f"{engage_idx}) have non-empty ext_* -- cue used "
                f"post-handoff (first offender: row {bad_rows[0]})"
            )
        if bad_nondet_rows:
            first_i, first_reason = bad_nondet_rows[0]
            details.append(
                f"{len(bad_nondet_rows)} non-detected ENGAGE rows commanded "
                "a velocity that is neither held nor hovered (first "
                f"offender: row {first_i}, {first_reason})"
            )
        detail = "; ".join(details)
        result["checks"]["a"] = {"result": "FAIL", "detail": detail}
        result["fail_reasons"].append("(a) " + detail)
    else:
        n_nondet_checked = sum(
            1 for r in rows[engage_idx:]
            if r.get("phase") == "ENGAGE" and r.get("detected") != "1"
        )
        result["checks"]["a"] = {
            "result": "PASS",
            "detail": (
                f"zero non-empty ext_* cells across {len(rows) - engage_idx} "
                f"rows at/after first ENGAGE (row {engage_idx}), and all "
                f"{n_nondet_checked} non-detected ENGAGE rows held/hovered "
                "exactly"
                + (f" ({n_transition_grace} DASH->ENGAGE transition rows "
                   "granted the one-tick grace)" if n_transition_grace else "")
            ),
        }

    # ---- (b) >= 1 DASH row precedes the first ENGAGE row -------------------
    n_dash_before = sum(1 for r in rows[:engage_idx] if r.get("phase") in DASH_PHASES)
    if n_dash_before < 1:
        ok = False
        detail = (
            f"{n_dash_before} DASH rows precede the first ENGAGE row (row "
            f"{engage_idx}); expected >= 1"
        )
        result["checks"]["b"] = {"result": "FAIL", "detail": detail}
        result["fail_reasons"].append("(b) " + detail)
    else:
        result["checks"]["b"] = {
            "result": "PASS",
            "detail": (
                f"{n_dash_before} DASH rows precede the first ENGAGE row "
                f"(row {engage_idx})"
            ),
        }

    # ---- (e) POPULATION: did the terminal actually ACTUATE? ---------------
    # Computed BEFORE (c) because (c) reports the same denominator, and
    # computed INDEPENDENTLY of lambda_deg so it still runs when (c)
    # WARN-skips on a missing column (the two blind spots were independent).
    #
    # vc_m_s absent entirely -> fall back to every detected ENGAGE row rather
    # than scoring zero rows (a check that silently evaluates an empty
    # population is the vacuous verdict this tool exists to prevent).
    have_vc = "vc_m_s" in cols
    n_det_engage = 0        # detected ENGAGE rows (any closing geometry)
    n_e_pop = 0             # (e) population: detected + pre-CPA (vc > 0)
    n_zerocmd = 0           # ... of which commanded ZERO horizontal velocity
    n_zerocmd_all = 0       # ... over every detected ENGAGE row
    for r in rows[engage_idx:]:
        if r.get("phase") != "ENGAGE":
            continue
        if (r.get("detected") != "1" or not r.get("cmd_vn")
                or not r.get("cmd_ve")):
            continue
        try:
            vn = float(r["cmd_vn"])
            ve = float(r["cmd_ve"])
        except ValueError:
            continue
        is_zero = abs(vn) < ZERO_CMD_TOL and abs(ve) < ZERO_CMD_TOL
        n_det_engage += 1
        if is_zero:
            n_zerocmd_all += 1
        pre_cpa = True
        if have_vc:
            try:
                pre_cpa = float(r.get("vc_m_s") or 0.0) > 0.0
            except ValueError:
                pre_cpa = False
        if pre_cpa:
            n_e_pop += 1
            if is_zero:
                n_zerocmd += 1
    zerocmd_frac = (n_zerocmd / n_e_pop) if n_e_pop else None

    # ---- (c) cmd-velocity-azimuth vs camera lambda_deg correlation --------
    missing_c = [c for c in REQUIRED_COLUMNS_C if c not in cols]
    if missing_c:
        result["checks"]["c"] = {
            "result": "WARN",
            "detail": "missing column(s) for (c): " + ", ".join(missing_c),
        }
    else:
        # DEPLOYMENT-PROFILE CALIBRATION (adjudicated 2026-07-08, main
        # session; evidence in the session log + ADR): the 0.7 bound was
        # calibrated on S2 from-hover engagements where the commanded-velocity
        # azimuth swings ~20 deg (std). On the M5 running-start profile the
        # azimuth is PINNED by the ~16 m/s forward dash -- its physical swing
        # is atan(v_perp/16) (a few deg; measured std 0.0-1.7 deg on the clean
        # apriltag control arm) -- and post-CPA flyby rows reverse lambda
        # faster than an accel-limited (honest) controller can follow. A
        # correlation over a variance-starved or post-CPA window has no power
        # either way, so (c) GATES only when powered: pre-CPA rows (vc > 0),
        # n >= 10, command-azimuth std >= 2 deg; otherwise WARN with full
        # diagnostics. PASS still reported whenever corr >= bound.
        # TERMINAL-COAST EXCLUSION (2026-07-25, found on the first re-fly arm
        # through the FIXED zero-command latch). Once the one-way terminal
        # freeze latches (ADR-0023 -- the sanctioned near-CPA behaviour, kept
        # because split-freeze measured worse), every remaining ENGAGE tick
        # re-issues the SAME frozen (cmd_vn, cmd_ve) verbatim while lambda
        # keeps evolving off fresh detections through the near-CPA singularity
        # -- so a correlation over those rows tests a vector that is FROZEN BY
        # DESIGN, not whether guidance was camera-driven. Pre-fix this was
        # invisible: the latch held (0,0), and the zero-command skip below
        # dropped those rows. Post-fix the latch holds a REAL vector, so
        # without this exclusion (c) FAILs exactly the flights the fix
        # repaired (measured: 13 of 14 correlated rows az==71.00 deg while
        # lambda swept -10..+107 deg, corr 0.614). The latch is identified by
        # its own signature -- the maximal TRAILING run of ENGAGE rows whose
        # cmd_vn/cmd_ve strings are byte-identical to the final row's (a live
        # law never repeats a command to full float precision for >=2
        # consecutive ticks) -- and the excluded rows are COUNTED AND
        # REPORTED, never silently dropped (docs/error_handling_policy.md).
        # Check (e) still gates that the latched vector is not zero.
        # The latch's offline signature, precisely: the frozen vector is the
        # LAST NONZERO command of the flight (dropout ticks interleave (0,0)
        # holds, so the coast is NOT one contiguous trailing run), it first
        # appears at the latch tick (the latch freezes the vector computed on
        # that very tick), and every later detected tick re-issues it
        # byte-identically. So: take the last nonzero (cmd_vn, cmd_ve); if it
        # occurs >= 2 times, every occurrence from its first onward is coast.
        engage_rows = [r for r in rows[engage_idx:] if r.get("phase") == "ENGAGE"]
        coast_ids = set()
        fk = None
        for r in reversed(engage_rows):
            k = (r.get("cmd_vn"), r.get("cmd_ve"))
            if not k[0] or not k[1]:
                continue
            try:
                if (abs(float(k[0])) < ZERO_CMD_TOL
                        and abs(float(k[1])) < ZERO_CMD_TOL):
                    continue
            except ValueError:
                continue
            fk = k
            break
        if fk is not None:
            matches = [r for r in engage_rows
                       if (r.get("cmd_vn"), r.get("cmd_ve")) == fk]
            if len(matches) >= 2:
                coast_ids = {id(r) for r in matches}
        n_coast = 0
        xs, ys = [], []
        for r in rows[engage_idx:]:
            if r.get("phase") != "ENGAGE":
                continue
            if (r.get("detected") != "1" or not r.get("lambda_deg")
                    or not r.get("cmd_vn") or not r.get("cmd_ve")):
                continue
            if id(r) in coast_ids:
                n_coast += 1
                continue
            try:
                if float(r.get("vc_m_s") or 0.0) <= 0.0:
                    continue  # post-CPA / opening geometry: no (c) power
            except ValueError:
                continue
            lam = float(r["lambda_deg"])
            vn = float(r["cmd_vn"])
            ve = float(r["cmd_ve"])
            if abs(vn) < ZERO_CMD_TOL and abs(ve) < ZERO_CMD_TOL:
                # A ZERO horizontal command has NO azimuth (atan2(0,0) is a
                # meaningless 0.0), so it cannot enter the correlation -- but
                # it is NOT nothing, and dropping it silently is exactly how
                # this check went blind. THE DENOMINATOR IS COUNTED AND
                # REPORTED (docs/error_handling_policy.md), and check (e)
                # below gates on it. See the (e) comment for the defect.
                continue
            az = math.degrees(math.atan2(ve, vn))
            # Lift az to the coterminal angle nearest lambda_deg. lambda_deg
            # is the filter's UNWRAPPED state -- through the near-CPA
            # singularity it accumulates full windings (measured 874 deg on a
            # re-fly flight, 2026-07-25) -- while atan2 is bounded to
            # (-180, 180], so the single branch-cut shift this used to apply
            # cannot reach past one winding and the correlation compared
            # incompatible representations (measured: corr 0.607 -> 0.998 on
            # the same 10 rows once lifted; the 0.607 read as a gating FAIL).
            # A mod-360 lift is pure representation, never a real angular
            # difference, and reduces to the old branch-cut shift whenever
            # |az - lam| <= 540.
            az += 360.0 * round((lam - az) / 360.0)
            xs.append(lam)
            ys.append(az)

        n = len(xs)
        if n_e_pop and zerocmd_frac > ZERO_CMD_FAIL_FRAC:
            # THE SHRINKING DENOMINATOR, MADE VISIBLE. (c) can only be
            # computed over ticks that carried a command; when most ticks
            # carried none, a high correlation over the surviving few is not
            # evidence that the terminal was camera-driven -- it is evidence
            # about the handful of ticks that happened to steer. Refuse to
            # gate and point at (e), which does gate on this.
            result["checks"]["c"] = {
                "result": "WARN",
                "detail": (
                    f"computed over only {n} of {n_e_pop} detected pre-CPA "
                    f"ENGAGE ticks -- the other {n_zerocmd} "
                    f"({100.0 * zerocmd_frac:.0f}%) commanded ZERO horizontal "
                    "velocity and have no azimuth. A correlation over the "
                    "few ticks that DID steer cannot certify a terminal that "
                    "mostly did not: NOT gating, see (e)"
                ),
            }
        elif n < 10:
            result["checks"]["c"] = {
                "result": "WARN",
                "detail": (
                    f"only {n} pre-CPA ENGAGE rows with a camera detection, "
                    f"a nonzero command, and a LIVE (pre-latch) law (< 10; "
                    f"{n_zerocmd} of the {n_e_pop} detected pre-CPA ticks "
                    f"were zero-command, {n_coast} were the latched terminal "
                    "coast re-issuing its frozen vector) -- correlation "
                    "underpowered, not gating "
                    "(deployment-profile calibration 2026-07-08)"
                ),
            }
        else:
            mx = sum(xs) / n
            my = sum(ys) / n
            cov = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))
            vx = sum((v - mx) ** 2 for v in xs)
            vy = sum((v - my) ** 2 for v in ys)
            bound = 0.55 if law == "pip" else 0.7
            std_azi = math.sqrt(vy / n)
            if vx < 1e-6 or vy < 1e-6:
                result["checks"]["c"] = {
                    "result": "WARN",
                    "detail": (
                        f"degenerate variance (vx={vx:.4f}, vy={vy:.4f}) "
                        f"over {n} rows -- skipping correlation check"
                    ),
                }
            elif std_azi < 2.0:
                corr = cov / math.sqrt(vx * vy)
                result["checks"]["c"] = {
                    "result": "WARN",
                    "detail": (
                        f"command-azimuth variance-starved (std={std_azi:.2f} "
                        f"deg < 2.0; forward-speed-pinned profile) over {n} "
                        f"pre-CPA rows -- corr={corr:.3f} reported, not "
                        "gating (deployment-profile calibration 2026-07-08)"
                    ),
                }
            else:
                corr = cov / math.sqrt(vx * vy)
                if corr < bound:
                    ok = False
                    detail = (
                        f"cmd-velocity-azimuth vs camera lambda_deg "
                        f"correlation {corr:.3f} < {bound} over {n} live "
                        f"pre-latch rows ({n_coast} latched-coast rows "
                        "excluded)"
                    )
                    result["checks"]["c"] = {"result": "FAIL", "detail": detail}
                    result["fail_reasons"].append("(c) " + detail)
                else:
                    result["checks"]["c"] = {
                        "result": "PASS",
                        "detail": (
                            f"cmd-velocity-azimuth vs camera lambda_deg "
                            f"correlation {corr:.3f} >= {bound} over {n} live "
                            f"pre-latch rows ({n_coast} latched-coast rows "
                            "excluded)"
                        ),
                    }

    # ---- (e) the terminal actually ACTUATED -- FAIL-able -------------------
    # WHY THIS CHECK EXISTS (2026-07-25). scripts/m4_intercept.py's terminal
    # freeze latched `frozen_vworld = (vh0, vh1)` off the module-scope
    # initialisers `vh0 = vh1 = 0.0` whenever ENGAGE began already inside
    # TERMINAL_FREEZE_RANGE_M -- which the markerless seeker's short r_hat at
    # handoff makes the NORMAL case. The whole ENGAGE phase then re-issued
    # cmd=(0, 0, v_down, yaw): the terminal was BRAKING, not steering, and
    # every camera-arm conclusion measured through it was measuring a
    # non-actuating terminal.
    #
    # THIS AUDIT WAS ANTI-CORRELATED WITH THAT DEFECT. Check (c) drops
    # zero-command rows (it must -- they have no azimuth), so on exactly the
    # broken flights it dropped nearly every row, fell under its n < 10
    # underpowered branch, stopped gating, and the run still printed
    # "PASS: N/N audited flights passed (a)/(b)/(c)". The instrument could not
    # see the failure it was pointed at -- the silent-failure rule in
    # CLAUDE.md: a bug in the SCORER invalidates every run that passed
    # through it, and a paired control cannot catch it because both arms
    # share the instrument.
    #
    # A detected, pre-CPA ENGAGE tick is a tick where the seeker HAS the
    # target and the geometry is still closing. A camera-only terminal that
    # commands exactly (0, 0) there is not guiding, whatever else it logs.
    if n_e_pop == 0:
        result["checks"]["e"] = {
            "result": "WARN",
            "detail": (
                f"no detected pre-CPA ENGAGE ticks ({n_det_engage} detected "
                f"ENGAGE rows total"
                + ("" if have_vc else "; no vc_m_s column, so 'pre-CPA' could "
                                      "not be applied")
                + ") -- terminal actuation UNVERIFIED, not gating"
            ),
        }
    elif zerocmd_frac > ZERO_CMD_FAIL_FRAC and n_e_pop >= ZERO_CMD_MIN_N:
        ok = False
        detail = (
            f"terminal commanded ZERO horizontal velocity on {n_zerocmd}/"
            f"{n_e_pop} ({100.0 * zerocmd_frac:.0f}%) detected pre-CPA ENGAGE "
            f"ticks (|cmd_vn|,|cmd_ve| < {ZERO_CMD_TOL:g}) -- the terminal was "
            f"not steering. Known cause: the m4_intercept.py frozen_vworld "
            f"latch capturing an unset (0,0); check the run's 'velocity vector "
            f"frozen at ... ((0.00, 0.00) world)' line"
        )
        result["checks"]["e"] = {"result": "FAIL", "detail": detail}
        result["fail_reasons"].append("(e) " + detail)
    elif n_e_pop < ZERO_CMD_MIN_N:
        result["checks"]["e"] = {
            "result": "WARN",
            "detail": (
                f"only {n_e_pop} detected pre-CPA ENGAGE ticks "
                f"(< {ZERO_CMD_MIN_N}) -- {n_zerocmd} zero-command; "
                "underpowered, reported but not gating"
            ),
        }
    else:
        result["checks"]["e"] = {
            "result": "PASS",
            "detail": (
                f"terminal actuated on {n_e_pop - n_zerocmd}/{n_e_pop} "
                f"detected pre-CPA ENGAGE ticks "
                f"({100.0 * zerocmd_frac:.0f}% zero-command <= "
                f"{100.0 * ZERO_CMD_FAIL_FRAC:.0f}%)"
            ),
        }
    result["counts"] = {
        "n_det_engage": n_det_engage,
        "n_e_pop": n_e_pop,
        "n_zerocmd": n_zerocmd,
        "n_zerocmd_all": n_zerocmd_all,
    }

    # ---- (d) residual-leak, ADVISORY ONLY -- never gates pass/fail --------
    # See check_s2.sh's own comment block above its (d) for the full
    # calibration reasoning (ported verbatim in spirit): offline over every
    # historical S2 (--handoff) flight under logs/ with a DASH phase, all
    # independently (a)/(b)/(c)-clean, corr(d_cmd, d_gt) spans roughly
    # pip [-0.986, +0.293] mean -0.527 (n=10 flights) and pronav
    # [-0.973, +0.920] mean -0.389 (n=124 flights) -- it does not converge
    # toward zero with more data (so it's tracking a real, law-dependent
    # mechanization coupling, not a leak/no-leak split), and known-honest
    # flights already span almost the entire [-1, +1] range in both signs.
    # No numeric bound separates "honest" from "leaking" at this sample
    # size -- hence advisory only, printed for a human to eyeball, never
    # gating the exit code.
    missing_d = [c for c in REQUIRED_COLUMNS_D if c not in cols] + \
        (["lambda_deg"] if "lambda_deg" not in cols else [])
    if missing_d:
        result["checks"]["d"] = {
            "result": "WARN",
            "detail": "missing column(s) for (d): " + ", ".join(sorted(set(missing_d))),
        }
    else:
        xs_res, ys_res = [], []
        for r in rows[engage_idx:]:
            if r.get("phase") != "ENGAGE":
                continue
            if (r.get("detected") != "1" or not r.get("lambda_deg")
                    or not r.get("cmd_vn") or not r.get("cmd_ve")):
                continue
            if (not r.get("gt_cam_x") or not r.get("gt_cam_y")
                    or not r.get("gt_tag_x") or not r.get("gt_tag_y")):
                continue
            lam = float(r["lambda_deg"])
            vn = float(r["cmd_vn"])
            ve = float(r["cmd_ve"])
            if abs(vn) < 1e-6 and abs(ve) < 1e-6:
                continue
            az = math.degrees(math.atan2(ve, vn))
            diff = az - lam
            if diff > 180.0:
                az -= 360.0
            elif diff < -180.0:
                az += 360.0
            d_cmd = az - lam

            # world -> NED mapping (ADR-0013): north = world_y, east = world_x.
            north = float(r["gt_tag_y"]) - float(r["gt_cam_y"])
            east = float(r["gt_tag_x"]) - float(r["gt_cam_x"])
            gt_az = math.degrees(math.atan2(east, north))
            diff2 = gt_az - lam
            if diff2 > 180.0:
                gt_az -= 360.0
            elif diff2 < -180.0:
                gt_az += 360.0
            d_gt = gt_az - lam

            xs_res.append(d_cmd)
            ys_res.append(d_gt)

        n_res = len(xs_res)
        if n_res < 5:
            result["checks"]["d"] = {
                "result": "WARN",
                "detail": f"only {n_res} usable rows -- skipping residual-leak advisory",
            }
        else:
            mxr = sum(xs_res) / n_res
            myr = sum(ys_res) / n_res
            covr = sum((xs_res[i] - mxr) * (ys_res[i] - myr) for i in range(n_res))
            vxr = sum((v - mxr) ** 2 for v in xs_res)
            vyr = sum((v - myr) ** 2 for v in ys_res)
            if vxr < 1e-6 or vyr < 1e-6:
                result["checks"]["d"] = {
                    "result": "WARN",
                    "detail": f"degenerate variance over {n_res} rows -- skipping residual-leak advisory",
                }
            else:
                corr_d = covr / math.sqrt(vxr * vyr)
                result["checks"]["d"] = {
                    "result": "INFO",
                    "detail": (
                        f"residual-leak corr(d_cmd, d_gt) = {corr_d:.3f} over "
                        f"{n_res} rows (advisory, non-gating; historical "
                        "honest-flight range roughly -0.99..+0.92)"
                    ),
                }

    # ---- flight STATUS: name what was actually ENFORCED --------------------
    # THE THIRD STATUS (2026-07-25). Before this, any flight that did not FAIL
    # was "PASS", and the run-level banner then claimed "passed (a)/(b)/(c)"
    # -- even when (c) had WARN-skipped (missing lambda_deg column, or an
    # underpowered / variance-starved / zero-command-dominated window) and had
    # gated NOTHING. That is a vacuous verdict: a claim about a check that
    # never ran. PASS_PARTIAL says exactly which checks were enforced; the
    # summary is built from these counts, so the printed claim cannot drift
    # from the enforcement again.
    gated = [k for k in GATING_CHECKS
             if result["checks"].get(k, {}).get("result") in ("PASS", "FAIL")]
    ungated = [k for k in GATING_CHECKS if k not in gated]
    result["gated_checks"] = gated
    result["ungated_checks"] = ungated
    if not ok:
        result["status"] = "FAIL"
    elif ungated:
        result["status"] = "PASS_PARTIAL"
        result["notes"].append(
            "enforced (" + ")/(".join(gated) + ") only; NOT enforced: "
            + ", ".join(
                f"({k}) {result['checks'].get(k, {}).get('detail', 'not run')}"
                for k in ungated)
        )
    else:
        result["status"] = "PASS"
    return result


# --- table / report formatting ---------------------------------------------

def _check_symbol(checks, key):
    c = checks.get(key)
    if c is None:
        return "-"
    return c["result"]


def print_flight_table(records):
    """records: list of dicts with keys arm, run_idx, law, miss_m, clean,
    handoff, flight_csv, result (audit_flight_csv() output)."""
    headers = ["ARM", "RUN", "LAW", "STATUS", "A", "B", "C", "E", "D",
               "ZEROCMD", "MISS_M", "CLEAN", "HANDOFF", "FLIGHT_CSV"]
    rows_out = []
    for rec in records:
        res = rec["result"]
        cnt = res.get("counts") or {}
        # ZEROCMD is printed for EVERY flight, passing or not: the denominator
        # that check (c) silently discarded is now always on the face of the
        # report (zero-command / detected-pre-CPA-ENGAGE ticks).
        zc = ("-" if not cnt.get("n_e_pop")
              else f"{cnt['n_zerocmd']}/{cnt['n_e_pop']}")
        rows_out.append([
            rec["arm"], str(rec["run_idx"]), rec["law"] or "-", res["status"],
            _check_symbol(res["checks"], "a"), _check_symbol(res["checks"], "b"),
            _check_symbol(res["checks"], "c"), _check_symbol(res["checks"], "e"),
            _check_symbol(res["checks"], "d"), zc,
            rec.get("miss_m", "") or "-", rec.get("clean", "") or "-",
            rec.get("handoff", "") or "-",
            os.path.basename(res["csv_path"]) if res["csv_path"] else "-",
        ])
    widths = [len(h) for h in headers]
    for row in rows_out:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(str(cell)))
    fmt = "  ".join("{:<" + str(w) + "}" for w in widths)
    print(fmt.format(*headers))
    print(fmt.format(*["-" * w for w in widths]))
    for row in rows_out:
        print(fmt.format(*row))

    # Full reasons for anything not a clean PASS, so nothing is lost to
    # column truncation.
    for rec in records:
        res = rec["result"]
        if res["status"] in ("FAIL", "ERROR", "SKIPPED", "PASS_PARTIAL"):
            print(f"\n[audit_per_tick] {rec['arm']} run_idx={rec['run_idx']} "
                  f"law={rec['law'] or '-'} -> {res['status']} "
                  f"({res['csv_path']})")
            for reason in res["fail_reasons"]:
                print(f"    {reason}")
            for note in res.get("notes", []):
                print(f"    {note}")
        # Always surface (d), since it never shows in the table's own detail.
        d = res["checks"].get("d")
        if d and d["result"] == "INFO":
            print(f"[audit_per_tick] {rec['arm']} run_idx={rec['run_idx']} "
                  f"(d) advisory: {d['detail']}")


def arm_verdict(records_for_arm):
    """PASS only if every non-SKIPPED, non-ERROR flight in the arm PASSed
    EVERY gating check -- (a)/(b)/(c)/(e) -- AND at least one flight was
    actually scored. A flight that failed nothing but had a gating check
    WARN-skip is PASS_PARTIAL and makes the ARM PASS_PARTIAL: the arm did not
    fail, and it also was not fully checked, and those are different facts.
    Returns (verdict, n_pass, n_fail, n_skip, n_error, n_partial). SKIPPED flights
    (legitimate aborts) are excluded from both the pass and fail tallies.
    ERROR flights (couldn't be read/audited at all) count as failures -- a
    flight this tool cannot verify does not get the benefit of the doubt.

    NO VACUOUS VERDICTS (review-2 finding, 2026-07-24; policy:
    docs/error_handling_policy.md): when ZERO flights were actually scored
    (everything SKIPPED, or the arm resolved no flights at all), the verdict
    is VACUOUS, never PASS. Proven live before this fix: an all-skip arm
    printed "PASS (pass=0 fail=0 skipped=8)" and exited 0 -- a green audit
    that never audited anything. Same class as the CODED_DASH phase-name bug
    (the anti-mirage audit silently ran over zero ticks)."""
    n_pass = n_fail = n_skip = n_error = n_partial = 0
    for rec in records_for_arm:
        status = rec["result"]["status"]
        if status == "PASS":
            n_pass += 1
        elif status == "PASS_PARTIAL":
            n_partial += 1
        elif status == "FAIL":
            n_fail += 1
        elif status == "SKIPPED":
            n_skip += 1
        elif status == "ERROR":
            n_error += 1
    if n_fail or n_error:
        verdict = "FAIL"
    elif n_pass + n_partial == 0:
        verdict = "VACUOUS"  # zero units scored -> not PASS, ever
    elif n_partial:
        # Not a FAIL (nothing was caught) and not a PASS (not everything was
        # checked). Naming it is the whole point: an arm whose (c) never gated
        # on any flight used to print the same "PASS" as a fully-checked arm.
        verdict = "PASS_PARTIAL"
    else:
        verdict = "PASS"
    return verdict, n_pass, n_fail, n_skip, n_error, n_partial


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "arm_csvs", nargs="*",
        help="one or more mc_batch arm CSVs to sweep, e.g. "
             "logs/mc_ab_markerless_tuned.csv",
    )
    parser.add_argument(
        "--flight-csv", action="append", default=[], dest="flight_csvs",
        help="audit a single per-tick flight CSV directly (repeatable). "
             "Law is inferred from the CSV's own `law` column unless --law "
             "is given.",
    )
    parser.add_argument(
        "--law", default=None,
        help="override the law used for the (c)/(d) correlation bound on "
             "EVERY flight processed this run (both arm-swept and "
             "--flight-csv). Default: arm-swept flights use each row's own "
             "`law` field; --flight-csv flights infer it from the CSV's "
             "own `law` column.",
    )
    parser.add_argument(
        "--strict", dest="strict", action="store_true", default=True,
        help="(default) checks (a)-(c) are FAIL-able and gate the exit "
             "code, matching check_s2.sh semantics; (d) is always "
             "advisory-only regardless of this flag.",
    )
    parser.add_argument(
        "--no-strict", dest="strict", action="store_false",
        help="report-only mode: still prints every (a)-(c) PASS/FAIL/WARN "
             "verdict per flight, but never gates the exit code (exits 0 "
             "unless a usage/file error occurs). Useful for an exploratory "
             "sweep you don't want to fail a CI/gate step.",
    )
    args = parser.parse_args(argv)

    if not args.arm_csvs and not args.flight_csvs:
        parser.error("give at least one arm CSV or --flight-csv PATH")
        return 2  # pragma: no cover -- parser.error() already exits

    print(f"[audit_per_tick] per-tick audit logic: faithful port of "
          f"scripts/check_s2.sh's audit_csv() (checks a/b/c FAIL-able, d "
          f"advisory) -- scripts/check_s2.sh itself is untouched.")
    print(f"[audit_per_tick] arm-CSV resolution: {_RESOLVER_SOURCE}")
    print(f"[audit_per_tick] strict mode: "
          f"{'ON (a-c gate the exit code)' if args.strict else 'OFF (report-only, exit 0 unless usage/file error)'}")

    missing_file_errors = []
    records = []          # every flight actually audited (any source)
    arm_names_in_order = []
    per_arm_records = {}  # arm label -> list of records

    # --- resolve --flight-csv (single-flight mode) --------------------------
    for fc in args.flight_csvs:
        fc_abs = os.path.abspath(fc)
        if not os.path.exists(fc_abs):
            missing_file_errors.append(f"--flight-csv not found: {fc}")
            print(f"[audit_per_tick] ERROR: --flight-csv not found: {fc}")
            continue
        law = args.law if args.law else infer_law_from_flight_csv(fc_abs)
        res = audit_flight_csv(fc_abs, law)
        rec = {
            "arm": "(single)", "run_idx": "-", "law": law,
            "miss_m": "", "clean": "", "handoff": "", "result": res,
        }
        records.append(rec)
        per_arm_records.setdefault("(single)", []).append(rec)
        if "(single)" not in arm_names_in_order:
            arm_names_in_order.append("(single)")

    # --- resolve arm CSVs (batch-sweep mode) --------------------------------
    for arm_path_arg in args.arm_csvs:
        arm_path = os.path.abspath(arm_path_arg)
        arm_label = os.path.basename(arm_path_arg)
        if not os.path.exists(arm_path):
            missing_file_errors.append(f"arm CSV not found: {arm_path_arg}")
            print(f"[audit_per_tick] ERROR: arm CSV not found: {arm_path_arg}")
            continue
        try:
            flights = load_arm_flights(arm_path)
        except OSError as e:
            missing_file_errors.append(f"could not read arm CSV {arm_path_arg}: {e}")
            print(f"[audit_per_tick] ERROR: could not read arm CSV {arm_path_arg}: {e}")
            continue

        arm_names_in_order.append(arm_label)
        per_arm_records.setdefault(arm_label, [])
        for flight in flights:
            fc_path = _resolve_path(flight["flight_csv_path"])
            law = args.law if args.law else (flight["law"] or "")
            if not fc_path or not os.path.exists(fc_path):
                missing_file_errors.append(
                    f"{arm_label} run_idx={flight['run_idx']}: flight CSV "
                    f"not found ({flight['flight_csv_path']!r})"
                )
            res = audit_flight_csv(fc_path, law)
            rec = {
                "arm": arm_label, "run_idx": flight["run_idx"], "law": law,
                "miss_m": flight.get("miss_m", ""),
                "clean": flight.get("clean", ""),
                "handoff": flight.get("handoff", ""),
                "result": res,
            }
            records.append(rec)
            per_arm_records[arm_label].append(rec)

    if not records:
        print("[audit_per_tick] no flights were resolved -- nothing to audit.")
        return 2

    print()
    print_flight_table(records)

    print()
    print("[audit_per_tick] ================ Arm verdicts ================")
    any_fail = False
    any_vacuous = False
    n_pass_total = n_skip_total = n_partial_total = 0
    for arm_label in arm_names_in_order:
        recs = per_arm_records[arm_label]
        verdict, n_pass, n_fail, n_skip, n_error, n_partial = arm_verdict(recs)
        if verdict == "FAIL":
            any_fail = True
        elif verdict == "VACUOUS":
            any_vacuous = True
        n_pass_total += n_pass
        n_skip_total += n_skip
        n_partial_total += n_partial
        print(
            f"[audit_per_tick] {arm_label}: {verdict}  "
            f"(pass={n_pass} partial={n_partial} fail={n_fail} "
            f"skipped={n_skip} error={n_error} total={len(recs)})"
        )

    # PER-CHECK ENFORCEMENT TALLY -- the claim the banner is allowed to make.
    # Built by COUNTING what each check actually returned, so "N/N passed
    # (a)/(b)/(c)" can never again be printed for a run where (c) gated on
    # nothing.
    # (loop variable is `rec`, never `r`: these are RECORD dicts, not CSV rows
    # -- tests/test_flight_csv_contract.py's source sweep reads any `r[...]`
    # as a producer-column read, and it is right to.)
    scored = [rec for rec in records
              if rec["result"]["status"] in ("PASS", "PASS_PARTIAL", "FAIL")]
    enforced = {}
    for k in GATING_CHECKS:
        enforced[k] = sum(
            1 for rec in scored
            if rec["result"]["checks"].get(k, {}).get("result") in ("PASS", "FAIL"))
    n_zerocmd_total = sum((rec["result"].get("counts") or {}).get("n_zerocmd", 0)
                          for rec in scored)
    n_epop_total = sum((rec["result"].get("counts") or {}).get("n_e_pop", 0)
                       for rec in scored)
    if scored:
        print("[audit_per_tick] checks ENFORCED over the "
              f"{len(scored)} scored flights: "
              + ", ".join(f"({k}) on {enforced[k]}/{len(scored)}"
                          for k in GATING_CHECKS))
        print(f"[audit_per_tick] terminal actuation (e): {n_zerocmd_total}/"
              f"{n_epop_total} detected pre-CPA ENGAGE ticks commanded ZERO "
              f"horizontal velocity"
              + (" -- see the (e) FAIL detail above"
                 if any_fail else ""))

    if missing_file_errors:
        print()
        print("[audit_per_tick] FILE ERRORS (usage-or-missing-files):")
        for msg in missing_file_errors:
            print(f"    {msg}")
        print("[audit_per_tick] exit 2 (usage-or-missing-files)")
        return 2

    if any_vacuous:
        # No vacuous verdicts (docs/error_handling_policy.md): printed in BOTH
        # strict and --no-strict modes so an all-skip arm can never scroll by
        # as a quiet green line.
        print("[audit_per_tick] VACUOUS: at least one arm audited 0 flights "
              "(all SKIPPED or none resolved) -- nothing was verified there. "
              "A verdict computed on zero units is UNCERTAIN, never PASS.")

    if not args.strict:
        print("[audit_per_tick] --no-strict: report-only, exit 0 "
              "(a-c findings above are NOT gating this run).")
        return 0

    if any_fail:
        print("[audit_per_tick] FAIL: at least one arm has a flight that "
              "failed a gating check (a)/(b)/(c)/(e).")
        return 1

    if any_vacuous:
        print("[audit_per_tick] exit 3 (VACUOUS -- audited 0 flights in at "
              "least one arm; nothing was verified, which must never read "
              "as PASS).")
        return 3

    # THE BANNER STATES ONLY WHAT WAS ENFORCED (2026-07-25). It used to read
    # "PASS: N/N audited flights passed (a)/(b)/(c)" unconditionally -- printed
    # verbatim on runs where (c) WARN-skipped on every flight and gated
    # nothing, including the frozen_vworld zero-command flights (c) is blind
    # to by construction. The counts above are the enforcement record.
    n_scored = n_pass_total + n_partial_total
    if n_partial_total:
        print(f"[audit_per_tick] PASS_PARTIAL: {n_scored}/{n_scored} audited "
              f"flights failed NO gating check, but {n_partial_total} of them "
              f"had at least one check that never gated (see the PASS_PARTIAL "
              f"detail above) -- this run did NOT verify (a)/(b)/(c)/(e) on "
              f"every flight ({n_skip_total} SKIPPED aborts excluded, "
              f"(d) advisory-only).")
        return 0
    print(f"[audit_per_tick] PASS: {n_scored}/{n_scored} audited "
          f"flights passed every gating check (a)/(b)/(c)/(e) "
          f"({n_skip_total} SKIPPED aborts excluded, (d) advisory-only).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
