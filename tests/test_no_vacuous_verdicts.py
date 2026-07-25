"""Pins the standing rule: A VERDICT COMPUTED ON ZERO UNITS IS NEVER PASS.

(docs/error_handling_policy.md; review-2 finding 2026-07-24 — proven live
before the fix: an all-skip arm made scripts/audit_per_tick.py print
"PASS (pass=0 fail=0 skipped=8)" and exit 0, a green audit that never audited
anything. The same shape has bitten at least four times: the tripod zero-row
join, the CODED_DASH phase-filter no-op, the rebal-arm mirage, the tags.csv
schema mismatch.)

Covers the verdict-bearing emitters this rule currently binds and that this
change may touch:
  * scripts/audit_per_tick.py — arm_verdict VACUOUS + exit 3 (fixed 2026-07-25)
  * scripts/seeker/tripod_score.py gate_verdict — n_decoded=0 => UNCERTAIN
    (fixed 129ed45; pinned here so it cannot silently regress)

Offline, no sim. Run: .venv/bin/python -m pytest tests/test_no_vacuous_verdicts.py -v
"""
import os
import re
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (os.path.join(REPO_ROOT, "scripts"),
          os.path.join(REPO_ROOT, "scripts", "seeker"),
          os.path.dirname(os.path.abspath(__file__))):
    if p not in sys.path:
        sys.path.insert(0, p)

import audit_per_tick                                             # noqa: E402
import tripod_score                                               # noqa: E402
from contract_helpers import (m4_csv_header, mc_arm_header,       # noqa: E402
                              write_flight_csv, write_arm_csv)

CSV_HEADER, _ = m4_csv_header()
ARM_HEADER = mc_arm_header()
AUDIT = os.path.join(REPO_ROOT, "scripts", "audit_per_tick.py")


def _rec(status):
    return {"result": {"status": status}}


def test_arm_verdict_zero_scored_is_vacuous_never_pass():
    assert audit_per_tick.arm_verdict([])[0] == "VACUOUS"
    assert audit_per_tick.arm_verdict([_rec("SKIPPED")] * 8)[0] == "VACUOUS"
    # Scored arms keep their existing semantics.
    assert audit_per_tick.arm_verdict([_rec("PASS"), _rec("SKIPPED")])[0] == "PASS"
    assert audit_per_tick.arm_verdict([_rec("PASS"), _rec("FAIL")])[0] == "FAIL"
    assert audit_per_tick.arm_verdict([_rec("ERROR")])[0] == "FAIL"


def _make_arm(tmp_path, engage):
    flights = []
    for k in range(2):
        flights.append(write_flight_csv(
            str(tmp_path / f"flight{k}.csv"), CSV_HEADER, engage=engage))
    return write_arm_csv(str(tmp_path / "arm.csv"), ARM_HEADER, [
        {"run_idx": k, "law": "pronav", "clean": "0", "handoff": "0",
         "miss_m": "" if not engage else "1.10", "direction": "l2r",
         "flight_csv_path": fp, "config": "pronav_9.000",
         "breakoff_reason": "" if engage else "lost during dash"}
        for k, fp in enumerate(flights)])


def _run_audit(args):
    return subprocess.run([sys.executable, AUDIT, *args],
                          capture_output=True, text=True, cwd=REPO_ROOT)


def test_all_skipped_arm_exits_3_and_prints_vacuous(tmp_path):
    """THE regression (proven live pre-fix: 'PASS', exit 0). An arm whose every
    flight legitimately aborted before ENGAGE verified NOTHING, and must say so
    in the verdict, the banner, and the exit code."""
    arm = _make_arm(tmp_path, engage=False)
    proc = _run_audit([arm])
    assert proc.returncode == 3, proc.stdout + proc.stderr
    assert "VACUOUS" in proc.stdout
    assert "nothing was verified" in proc.stdout
    assert not re.search(r"arm\.csv: PASS", proc.stdout), (
        "an all-skip arm may never print a PASS verdict line")


def test_scored_arm_still_passes_and_names_its_n(tmp_path):
    """Positive control: the fix must not break a genuinely audited arm, and
    the final banner must name the n it was computed from."""
    arm = _make_arm(tmp_path, engage=True)
    proc = _run_audit([arm])
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "PASS: 2/2 audited flights passed" in proc.stdout


def test_no_strict_mode_still_prints_the_vacuous_banner(tmp_path):
    """--no-strict keeps its documented exit-0 contract but may not hide the
    vacuousness: the banner still prints."""
    arm = _make_arm(tmp_path, engage=False)
    proc = _run_audit(["--no-strict", arm])
    assert proc.returncode == 0
    assert "VACUOUS" in proc.stdout


def test_tripod_gate_zero_decoded_frames_is_uncertain():
    """Pin tripod_score's existing floor (DEFAULT_MIN_DECODED): a would-be PASS
    configuration with n_decoded=0 must return UNCERTAIN, not a verdict."""
    passing = tripod_score.gate_verdict(7.10, 7.10, 0.9, 30.0, 5, 9.0, 0.5,
                                        n_decoded=40)
    assert passing["verdict"] == "PASS", (
        "test premise broke: this configuration should PASS at n=40 "
        f"(got {passing['verdict']}: {passing['reason']})")
    g = tripod_score.gate_verdict(7.10, 7.10, 0.9, 30.0, 5, 9.0, 0.5,
                                  n_decoded=0)
    assert g["verdict"] == "UNCERTAIN", g
    assert "data floor" in g["reason"] or "decoded" in g["reason"]
