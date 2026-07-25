"""REGRESSION PIN: the honesty audit must not be ANTI-CORRELATED with the defect
it is pointed at.

THE DEFECT (review-2, 2026-07-25). scripts/audit_per_tick.py's check (c) -- the
only check that tests whether the command follows the CAMERA -- dropped every row
where the commanded horizontal velocity was ~zero (it must: atan2(0,0) has no
azimuth). On the frozen_vworld flights (scripts/m4_intercept.py's terminal
latching an unset (0,0), see tests/test_terminal_coast_latch.py) that dropped
nearly every row, so (c) fell under its n<10 "underpowered" branch, STOPPED
GATING, and the tool printed:

    (single): PASS  (pass=1 fail=0 skipped=0 error=0 total=1)
    PASS: 1/1 audited flights passed (a)/(b)/(c)

The worse the guidance failure, the fewer rows survived the filter, the more
certainly (c) went silent. That is the silent-failure rule in CLAUDE.md: a bug in
the SCORER invalidates every run that passed through it, and a paired control
cannot see it because both arms share the instrument.

THREE THINGS ARE PINNED HERE:
  1. check (e) "the terminal actually ACTUATED" GATES on the zero-command
     fraction -- the fixed tool FAILs the flight the old tool PASSed.
  2. the excluded rows are COUNTED and NAMED (no absorbed denominator).
  3. the summary line states what was ENFORCED -- it can no longer claim
     "passed (a)/(b)/(c)" for a flight where (c) never gated (PASS_PARTIAL).

The fixture is written through the PRODUCER's own CSV_HEADER (tests/
contract_helpers.py) -- a hand-typed fixture is exactly what hid the 2026-07-24
schema breaks.

Offline, no sim. Run:
  .venv/bin/python -m pytest tests/test_audit_zero_command_terminal.py -v
"""
import csv
import math
import os
import subprocess
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
for p in (SCRIPTS, os.path.dirname(os.path.abspath(__file__))):
    if p not in sys.path:
        sys.path.insert(0, p)

import audit_per_tick                                            # noqa: E402
from contract_helpers import (m4_csv_header, mc_arm_header,       # noqa: E402
                              write_flight_csv, write_arm_csv)

CSV_HEADER, _ = m4_csv_header()
ARM_HEADER = mc_arm_header()
AUDIT = os.path.join(SCRIPTS, "audit_per_tick.py")
PY = os.path.join(REPO_ROOT, ".venv", "bin", "python")
if not os.path.exists(PY):
    PY = sys.executable


def write_zerocmd_flight(path, n_zero=18, n_real=2):
    """The frozen_vworld signature, through the producer's own header: an ENGAGE
    phase whose cmd_vn/cmd_ve are IDENTICALLY zero on all but `n_real` ticks.

    Geometry mirrors contract_helpers.write_flight_csv (3 TAKEOFF + 5
    CODED_DASH + detected, closing, pre-CPA ENGAGE ticks) so the ONLY difference
    from the honest fixture is the commanded velocity -- exactly the difference
    the real defect makes."""
    rows, t = [], 0.0
    for _ in range(3):
        rows.append({"t": f"{t:.2f}", "phase": "TAKEOFF", "law": "pronav",
                     "detected": "0", "cmd_vn": "0.0", "cmd_ve": "0.0",
                     "cmd_vd": "0.0", "alt_m": "2.0"})
        t += 0.2
    for i in range(5):
        rows.append({"t": f"{t:.2f}", "phase": "CODED_DASH", "law": "pronav",
                     "detected": "0", "cmd_vn": "8.0", "cmd_ve": "0.0",
                     "cmd_vd": "0.0", "alt_m": "12.0",
                     "gt_range": f"{18.0 - 0.4 * i:.2f}"})
        t += 0.2
    for i in range(n_zero + n_real):
        gt = 16.0 - 0.6 * i
        lam = -10.0 + i
        zero = i >= n_real          # the coast latches after n_real honest ticks
        rows.append({
            "t": f"{t:.2f}", "phase": "ENGAGE", "law": "pronav", "detected": "1",
            "meas_range": f"{gt * 1.05:.3f}", "gt_range": f"{gt:.3f}",
            "lambda_deg": f"{lam:.3f}",
            "cmd_vn": "0.000000" if zero else f"{8.0 * math.cos(math.radians(lam)):.6f}",
            "cmd_ve": "0.000000" if zero else f"{8.0 * math.sin(math.radians(lam)):.6f}",
            "cmd_vd": "0.0", "vc_m_s": "6.0", "alt_m": "12.0",
            "coast_zero": "1" if zero else "",
            "gt_cam_x": "0.0", "gt_cam_y": "0.0", "gt_cam_z": "12.0",
            "gt_tag_x": f"{gt * math.sin(math.radians(lam)):.3f}",
            "gt_tag_y": f"{gt * math.cos(math.radians(lam)):.3f}",
            "gt_tag_z": "12.0",
        })
        t += 0.2
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_HEADER, restval="")
        w.writeheader()
        for r in rows:
            unknown = set(r) - set(CSV_HEADER)
            assert not unknown, (
                f"fixture writes column(s) the PRODUCER does not: "
                f"{sorted(unknown)} -- update it FROM CSV_HEADER")
            w.writerow(r)
    return path


# --- 1. the gating check the defect needed ---------------------------------

def test_zero_command_terminal_now_FAILS(tmp_path):
    """OLD BEHAVIOUR: STATUS=PASS, exit 0. NEW: check (e) FAILs the flight."""
    fp = write_zerocmd_flight(str(tmp_path / "zerocmd.csv"))
    res = audit_per_tick.audit_flight_csv(fp, "pronav")
    assert res["status"] == "FAIL", res["checks"]
    assert res["checks"]["e"]["result"] == "FAIL"
    assert any(r.startswith("(e)") for r in res["fail_reasons"]), res["fail_reasons"]
    detail = res["checks"]["e"]["detail"]
    assert "18/20" in detail and "ZERO horizontal velocity" in detail
    # (a) and (b) still PASS -- the flight is HONEST, it just is not guiding.
    # (e) is deliberately a separate, honestly-named assertion, NOT a mutation
    # of the camera-coupling correlation (the review's category-error warning).
    assert res["checks"]["a"]["result"] == "PASS"
    assert res["checks"]["b"]["result"] == "PASS"


def test_the_shrinking_denominator_is_counted_not_absorbed(tmp_path):
    fp = write_zerocmd_flight(str(tmp_path / "zerocmd.csv"))
    res = audit_per_tick.audit_flight_csv(fp, "pronav")
    counts = res["counts"]
    assert counts["n_e_pop"] == 20
    assert counts["n_zerocmd"] == 18
    assert counts["n_det_engage"] == 20
    # (c) must REFUSE to gate and SAY WHY, naming the excluded rows.
    c = res["checks"]["c"]
    assert c["result"] == "WARN"
    assert "18" in c["detail"] and "ZERO horizontal velocity" in c["detail"]
    assert "(e)" in c["detail"]


def test_an_honest_flight_is_untouched(tmp_path):
    """CALIBRATION PRESERVED: the honest producer-schema fixture must still be a
    clean full PASS, with (e) affirming actuation."""
    fp = write_flight_csv(str(tmp_path / "honest.csv"), CSV_HEADER)
    res = audit_per_tick.audit_flight_csv(fp, "pronav")
    assert res["status"] == "PASS", res["fail_reasons"]
    assert res["checks"]["e"]["result"] == "PASS"
    assert res["counts"]["n_zerocmd"] == 0
    assert res["gated_checks"] == ["a", "b", "c", "e"]
    assert res["ungated_checks"] == []


# --- 2. the third status + honest summary ----------------------------------

def test_a_check_that_did_not_gate_is_not_reported_as_passed(tmp_path):
    """PASS_PARTIAL: no failure, but (c) never ran. Before this, both cases
    printed the identical 'PASS ... passed (a)/(b)/(c)'."""
    # Drop lambda_deg -> (c) WARN-skips on a missing column; everything else
    # still gates (this is reader-2's half of the defect, independent of the
    # zero-command half).
    src = write_flight_csv(str(tmp_path / "honest.csv"), CSV_HEADER)
    hdr = [c for c in CSV_HEADER if c != "lambda_deg"]
    dst = str(tmp_path / "no_lambda.csv")
    with open(src, newline="") as fin, open(dst, "w", newline="") as fout:
        r = csv.DictReader(fin)
        w = csv.DictWriter(fout, fieldnames=hdr, restval="")
        w.writeheader()
        for row in r:
            row.pop("lambda_deg", None)
            w.writerow(row)
    res = audit_per_tick.audit_flight_csv(dst, "pronav")
    assert res["status"] == "PASS_PARTIAL", res["checks"]
    assert "c" in res["ungated_checks"]
    assert res["checks"]["e"]["result"] == "PASS"   # (e) needs no lambda_deg


def test_arm_verdict_names_partial_and_never_calls_it_pass():
    def rec(status):
        return {"result": {"status": status}}
    assert audit_per_tick.arm_verdict([rec("PASS")])[0] == "PASS"
    assert audit_per_tick.arm_verdict([rec("PASS_PARTIAL")])[0] == "PASS_PARTIAL"
    assert audit_per_tick.arm_verdict(
        [rec("PASS"), rec("PASS_PARTIAL")])[0] == "PASS_PARTIAL"
    assert audit_per_tick.arm_verdict([rec("PASS_PARTIAL"), rec("FAIL")])[0] == "FAIL"
    # the pre-existing no-vacuous-verdicts contract is intact
    assert audit_per_tick.arm_verdict([])[0] == "VACUOUS"
    assert audit_per_tick.arm_verdict([rec("SKIPPED")] * 8)[0] == "VACUOUS"


# --- 3. end to end through the CLI (exit code + banner) --------------------

def _cli(*args):
    return subprocess.run([PY, AUDIT, *args], capture_output=True, text=True,
                          timeout=120, cwd=REPO_ROOT)


def test_cli_exits_nonzero_and_says_which_checks_gated(tmp_path):
    fp = write_zerocmd_flight(str(tmp_path / "zerocmd.csv"))
    r = _cli("--flight-csv", fp, "--law", "pronav")
    assert r.returncode == 1, r.stdout
    out = r.stdout
    # The banner must never again assert an unenforced check.
    assert "audited flights passed (a)/(b)/(c)" not in out
    assert "checks ENFORCED over" in out
    assert "(c) on 0/1" in out          # (c) gated on NO flight ...
    assert "(e) on 1/1" in out          # ... and (e) did the gating
    assert "ZERO horizontal velocity" in out


def test_cli_on_an_honest_arm_still_exits_zero(tmp_path):
    """The gate this tool feeds (check_t19.sh assertion 5, check_seeker_v2.sh)
    must stay green on honest flights."""
    flights = [write_flight_csv(str(tmp_path / f"f{k}.csv"), CSV_HEADER)
               for k in range(2)]
    arm = write_arm_csv(str(tmp_path / "arm.csv"), ARM_HEADER, [
        {"run_idx": k, "law": "pronav", "clean": "1", "handoff": "1",
         "miss_m": "1.10", "flight_csv_path": fp}
        for k, fp in enumerate(flights)])
    r = _cli(arm)
    assert r.returncode == 0, r.stdout
    assert "passed every gating check (a)/(b)/(c)/(e)" in r.stdout


# --- 3. the latched-coast exclusion (2026-07-25, first re-fly arm) ----------

def write_coast_flight(path, n_live=12, n_coast=8, n_dropout=3,
                       coast_tracks_lambda=False):
    """A flight through the FIXED terminal: n_live honest steering ticks whose
    azimuth tracks lambda exactly, then the one-way latch -- every later
    detected tick re-issues ONE frozen nonzero vector VERBATIM while lambda
    keeps evolving, interleaved with (0,0,0) dropout holds (detected=0), the
    way the real 20260725T220504Z flight looked (48+3+2+5... runs split by
    hover rows). coast_tracks_lambda=False is the REAL latch; True writes the
    coast ticks as live tracking instead (the no-latch control)."""
    rows, t = [], 0.0
    for _ in range(3):
        rows.append({"t": f"{t:.2f}", "phase": "TAKEOFF", "law": "pronav",
                     "detected": "0", "cmd_vn": "0.0", "cmd_ve": "0.0",
                     "cmd_vd": "0.0", "alt_m": "2.0"})
        t += 0.2
    for i in range(5):
        rows.append({"t": f"{t:.2f}", "phase": "CODED_DASH", "law": "pronav",
                     "detected": "0", "cmd_vn": "8.0", "cmd_ve": "0.0",
                     "cmd_vd": "0.0", "alt_m": "12.0",
                     "gt_range": f"{18.0 - 0.4 * i:.2f}"})
        t += 0.2

    def engage_row(i, lam, vn, ve, detected="1"):
        gt = 16.0 - 0.5 * i
        return {"t": f"{t:.2f}", "phase": "ENGAGE", "law": "pronav",
                "detected": detected,
                "meas_range": f"{gt * 1.05:.3f}", "gt_range": f"{gt:.3f}",
                "lambda_deg": f"{lam:.3f}", "cmd_vn": vn, "cmd_ve": ve,
                "cmd_vd": "0.0", "vc_m_s": "6.0", "alt_m": "12.0",
                "gt_cam_x": "0.0", "gt_cam_y": "0.0", "gt_cam_z": "12.0",
                "gt_tag_x": f"{gt * math.sin(math.radians(lam)):.3f}",
                "gt_tag_y": f"{gt * math.cos(math.radians(lam)):.3f}",
                "gt_tag_z": "12.0"}

    i = 0
    for k in range(n_live):
        lam = -10.0 + 1.6 * k
        rows.append(engage_row(i, lam,
                    f"{8.0 * math.cos(math.radians(lam)):.6f}",
                    f"{8.0 * math.sin(math.radians(lam)):.6f}"))
        t += 0.2; i += 1
    frozen = ("-0.340200", "12.036800")
    for k in range(n_coast + n_dropout):
        lam = 15.0 + 9.0 * k                 # lambda keeps sweeping post-latch
        dropout = k in (2, 5, 7)[:n_dropout]
        if dropout:
            rows.append(engage_row(i, lam, "0.0", "0.0", detected="0"))
        elif coast_tracks_lambda:
            rows.append(engage_row(i, lam,
                        f"{8.0 * math.cos(math.radians(lam)):.6f}",
                        f"{8.0 * math.sin(math.radians(lam)):.6f}"))
        else:
            rows.append(engage_row(i, lam, frozen[0], frozen[1]))
        t += 0.2; i += 1
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_HEADER, restval="")
        w.writeheader()
        for r in rows:
            unknown = set(r) - set(CSV_HEADER)
            assert not unknown, sorted(unknown)
            w.writerow(r)
    return path


def test_latched_coast_rows_are_excluded_from_c_and_counted(tmp_path):
    """THE 2026-07-25 RE-FLY LESSON: the frozen vector is re-issued verbatim
    while lambda evolves; correlating it tests the ADR-0023 freeze, not the
    camera coupling. (c) must gate over the LIVE pre-latch rows only, PASS
    there, and COUNT the excluded coast rows in its detail."""
    fp = write_coast_flight(str(tmp_path / "coast.csv"))
    res = audit_per_tick.audit_flight_csv(fp, "pronav")
    assert res["status"] == "PASS", (res["fail_reasons"], res["checks"])
    c = res["checks"]["c"]
    assert c["result"] == "PASS", c
    assert "12 live pre-latch rows" in c["detail"], c["detail"]
    assert "8 latched-coast rows excluded" in c["detail"], c["detail"]
    # (e): the latched vector is NONZERO -- actuation affirmed, not skipped.
    assert res["checks"]["e"]["result"] == "PASS"
    assert res["counts"]["n_zerocmd"] == 0


def test_exclusion_does_not_blind_c_to_a_genuinely_decorrelated_terminal(tmp_path):
    """CONTROL: with NO latch (every tick a distinct live command) a terminal
    whose azimuth anti-tracks lambda must still FAIL (c) -- the exclusion
    keys on the repeated-verbatim signature, so it cannot absolve live rows."""
    fp = write_coast_flight(str(tmp_path / "anti.csv"), n_coast=8,
                            coast_tracks_lambda=True)
    # rewrite the LIVE prefix anti-correlated: az = -lam (distinct every tick)
    rows = list(csv.DictReader(open(fp)))
    for r in rows:
        if r["phase"] == "ENGAGE" and r["detected"] == "1" and r["lambda_deg"]:
            lam = float(r["lambda_deg"])
            r["cmd_vn"] = f"{8.0 * math.cos(math.radians(-lam)):.6f}"
            r["cmd_ve"] = f"{8.0 * math.sin(math.radians(-lam)):.6f}"
    with open(fp, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_HEADER, restval="")
        w.writeheader()
        w.writerows(rows)
    res = audit_per_tick.audit_flight_csv(fp, "pronav")
    assert res["checks"]["c"]["result"] == "FAIL", res["checks"]["c"]
    assert res["status"] == "FAIL"


def test_c_lifts_azimuth_across_multiple_lambda_windings(tmp_path):
    """The filter's lambda_deg is UNWRAPPED (874 deg measured through the
    near-CPA singularity on a 2026-07-25 re-fly flight) while atan2 is bounded
    to (-180, 180]. A perfectly-tracking terminal must PASS (c) even when
    lambda has wound past +-540, where the old single branch-cut shift could
    not reach (measured: corr 0.607 -> 0.998 on the same rows once lifted)."""
    fp = write_coast_flight(str(tmp_path / "wind.csv"), n_live=0, n_coast=0,
                            n_dropout=0)
    rows = list(csv.DictReader(open(fp)))
    t = float(rows[-1]["t"]) if rows and rows[-1]["t"] else 2.0
    import math as _m
    for k in range(12):
        lam = 100.0 + 70.0 * k          # unwrapped: 100 .. 870 deg
        t += 0.2
        gt = 16.0 - 0.5 * k
        rows.append({"t": f"{t:.2f}", "phase": "ENGAGE", "law": "pronav",
                     "detected": "1", "meas_range": f"{gt * 1.05:.3f}",
                     "gt_range": f"{gt:.3f}", "lambda_deg": f"{lam:.3f}",
                     "cmd_vn": f"{8.0 * _m.cos(_m.radians(lam)):.6f}",
                     "cmd_ve": f"{8.0 * _m.sin(_m.radians(lam)):.6f}",
                     "cmd_vd": "0.0", "vc_m_s": "6.0", "alt_m": "12.0",
                     "gt_cam_x": "0.0", "gt_cam_y": "0.0", "gt_cam_z": "12.0",
                     "gt_tag_x": f"{gt * _m.sin(_m.radians(lam)):.3f}",
                     "gt_tag_y": f"{gt * _m.cos(_m.radians(lam)):.3f}",
                     "gt_tag_z": "12.0"})
    with open(fp, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_HEADER, restval="")
        w.writeheader()
        w.writerows(rows)
    res = audit_per_tick.audit_flight_csv(fp, "pronav")
    c = res["checks"]["c"]
    assert c["result"] == "PASS", c
    assert "0.99" in c["detail"] or "1.000" in c["detail"], c["detail"]
