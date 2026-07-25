#!/usr/bin/env python3
"""Standalone per-tick honesty audit -- extracted from scripts/check_s2.sh,
generalized to sweep EVERY flight in an mc_batch arm CSV.

WHY THIS EXISTS: NEXT.md records the debt "Full per-tick check owed at A/B
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
      0.7 -- ADR-0013) -- FAIL-able.
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

Exit code: 0 if every audited flight passes checks (a)-(c) (SKIPPED flights
don't count either way, (d) never gates); 1 if any audited flight fails
(a)/(b)/(c) (only in --strict mode, the default); 2 on a usage error or a
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


def audit_flight_csv(csv_path, law):
    """Faithful port of check_s2.sh's audit_csv() embedded python heredoc.
    Does NOT print -- returns a result dict (schema below) for the caller to
    format. `law` selects the (c) correlation bound: 0.55 for "pip", 0.7 for
    anything else (including "pronav" or unknown) -- exactly check_s2.sh's
    `bound = 0.55 if law == "pip" else 0.7`.

    Returns:
        {
          "csv_path": str, "law": str,
          "status": "PASS" | "FAIL" | "SKIPPED" | "ERROR",
          "engage_idx": int | None, "n_rows": int | None,
          "checks": {"a": {...}, "b": {...}, "c": {...}, "d": {...}},
          "fail_reasons": [str, ...],   # non-advisory failure detail lines
        }
    """
    result = {
        "csv_path": csv_path,
        "law": law,
        "status": None,
        "engage_idx": None,
        "n_rows": None,
        "checks": {},
        "fail_reasons": [],
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
        xs, ys = [], []
        for r in rows[engage_idx:]:
            if r.get("phase") != "ENGAGE":
                continue
            if (r.get("detected") != "1" or not r.get("lambda_deg")
                    or not r.get("cmd_vn") or not r.get("cmd_ve")):
                continue
            try:
                if float(r.get("vc_m_s") or 0.0) <= 0.0:
                    continue  # post-CPA / opening geometry: no (c) power
            except ValueError:
                continue
            lam = float(r["lambda_deg"])
            vn = float(r["cmd_vn"])
            ve = float(r["cmd_ve"])
            if abs(vn) < 1e-6 and abs(ve) < 1e-6:
                continue
            az = math.degrees(math.atan2(ve, vn))
            # Normalize away atan2's +-180 branch cut relative to lambda_deg
            # (a pure +-360 artifact, not a real angular difference).
            diff = az - lam
            if diff > 180.0:
                az -= 360.0
            elif diff < -180.0:
                az += 360.0
            xs.append(lam)
            ys.append(az)

        n = len(xs)
        if n < 10:
            result["checks"]["c"] = {
                "result": "WARN",
                "detail": (
                    f"only {n} pre-CPA ENGAGE rows with a camera detection "
                    "(< 10) -- correlation underpowered, not gating "
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
                        f"correlation {corr:.3f} < {bound} over {n} rows"
                    )
                    result["checks"]["c"] = {"result": "FAIL", "detail": detail}
                    result["fail_reasons"].append("(c) " + detail)
                else:
                    result["checks"]["c"] = {
                        "result": "PASS",
                        "detail": (
                            f"cmd-velocity-azimuth vs camera lambda_deg "
                            f"correlation {corr:.3f} >= {bound} over {n} rows"
                        ),
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

    result["status"] = "PASS" if ok else "FAIL"
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
    headers = ["ARM", "RUN", "LAW", "STATUS", "A", "B", "C", "D", "MISS_M",
               "CLEAN", "HANDOFF", "FLIGHT_CSV"]
    rows_out = []
    for rec in records:
        res = rec["result"]
        rows_out.append([
            rec["arm"], str(rec["run_idx"]), rec["law"] or "-", res["status"],
            _check_symbol(res["checks"], "a"), _check_symbol(res["checks"], "b"),
            _check_symbol(res["checks"], "c"), _check_symbol(res["checks"], "d"),
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
        if res["status"] in ("FAIL", "ERROR", "SKIPPED"):
            print(f"\n[audit_per_tick] {rec['arm']} run_idx={rec['run_idx']} "
                  f"law={rec['law'] or '-'} -> {res['status']} "
                  f"({res['csv_path']})")
            for reason in res["fail_reasons"]:
                print(f"    {reason}")
        # Always surface (d), since it never shows in the table's own detail.
        d = res["checks"].get("d")
        if d and d["result"] == "INFO":
            print(f"[audit_per_tick] {rec['arm']} run_idx={rec['run_idx']} "
                  f"(d) advisory: {d['detail']}")


def arm_verdict(records_for_arm):
    """PASS only if every non-SKIPPED, non-ERROR flight in the arm PASSed
    (a)/(b)/(c) AND at least one flight was actually scored. SKIPPED flights
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
    n_pass = n_fail = n_skip = n_error = 0
    for rec in records_for_arm:
        status = rec["result"]["status"]
        if status == "PASS":
            n_pass += 1
        elif status == "FAIL":
            n_fail += 1
        elif status == "SKIPPED":
            n_skip += 1
        elif status == "ERROR":
            n_error += 1
    if n_fail or n_error:
        verdict = "FAIL"
    elif n_pass == 0:
        verdict = "VACUOUS"  # zero units scored -> not PASS, ever
    else:
        verdict = "PASS"
    return verdict, n_pass, n_fail, n_skip, n_error


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
    n_pass_total = n_skip_total = 0
    for arm_label in arm_names_in_order:
        recs = per_arm_records[arm_label]
        verdict, n_pass, n_fail, n_skip, n_error = arm_verdict(recs)
        if verdict == "FAIL":
            any_fail = True
        elif verdict == "VACUOUS":
            any_vacuous = True
        n_pass_total += n_pass
        n_skip_total += n_skip
        print(
            f"[audit_per_tick] {arm_label}: {verdict}  "
            f"(pass={n_pass} fail={n_fail} skipped={n_skip} error={n_error} "
            f"total={len(recs)})"
        )

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
              "failed (a)/(b)/(c).")
        return 1

    if any_vacuous:
        print("[audit_per_tick] exit 3 (VACUOUS -- audited 0 flights in at "
              "least one arm; nothing was verified, which must never read "
              "as PASS).")
        return 3

    print(f"[audit_per_tick] PASS: {n_pass_total}/{n_pass_total} audited "
          f"flights passed (a)/(b)/(c) "
          f"({n_skip_total} SKIPPED aborts excluded, (d) advisory-only).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
