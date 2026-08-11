"""GOLDEN CONTRACT: m4_intercept per-flight CSV -> its verdict-bearing consumers.

THE DEFECT CLASS (review 2, 2026-07-24): a producer writes one schema, a
consumer reads another, both sides pass their own tests, and the pair silently
returns a confident wrong answer (empty join -> vacuous verdict, or a mis-keyed
filter -> the wrong quantity scored). It has bitten at least four times
(tripod join, CODED_DASH phase filter, rebal-arm mirage, tags.csv schema).
tests/test_tripod_pipeline_join.py is the template check-KIND; this file covers
the highest-stakes uncovered pair:

  PRODUCER  scripts/m4_intercept.py per-flight CSV (CSV_HEADER — the module's
            own list; ast fallback, never hand-typed, never skipped)
  PRODUCER  scripts/mc_batch.sh arm CSV (header extracted from its own heredoc)
  CONSUMERS scripts/coded_dash_summary.py         (camera-guided vs dash-only)
            scripts/experiments/loft_dive/parse_ab.py  (Phase A recall/verdicts)
            scripts/audit_per_tick.py             (the per-tick honesty audit)

Two layers of protection:
  1. SCHEMA: every column a consumer reads by name exists in the producer's
     header (enumerated with line refs + a mechanical source sweep so the
     enumeration itself cannot go stale silently).
  2. BEHAVIOR: a synthetic flight written THROUGH the producer's schema with
     known real-target ENGAGE ticks classifies CAMERA-GUIDED, and a
     phantom-only one classifies dash-only — end to end through the real
     consumer functions, arm CSV included.

Everything offline; no sim. Run: .venv/bin/python -m pytest tests/test_flight_csv_contract.py -v
"""
import csv
import os
import re
import sys
import types

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
LOFT_DIVE = os.path.join(SCRIPTS, "experiments", "loft_dive")
for p in (SCRIPTS, LOFT_DIVE, os.path.dirname(os.path.abspath(__file__))):
    if p not in sys.path:
        sys.path.insert(0, p)

from contract_helpers import (m4_csv_header, mc_arm_header,      # noqa: E402
                              write_flight_csv, write_arm_csv)

CSV_HEADER, CSV_HEADER_SOURCE = m4_csv_header()
ARM_HEADER = mc_arm_header()

import coded_dash_summary                                        # noqa: E402
import audit_per_tick                                            # noqa: E402

# parse_ab imports pyulog at module scope but only touches it in the ULog
# branch of flight_metrics(), which our fixtures never reach (their filenames
# carry no UTC stamp -> find_ulog is never consulted). Stub the import if the
# env lacks pyulog so this contract can NEVER skip (green must mean ran).
try:
    import pyulog  # noqa: F401
except ImportError:  # pragma: no cover -- env-dependent
    _stub = types.ModuleType("pyulog")
    _stub.ULog = type("ULog", (), {})
    sys.modules["pyulog"] = _stub
import parse_ab                                                   # noqa: E402


# --------------------------------------------------------------- layer 1: schema
# Flight-CSV columns each consumer reads BY NAME (line refs as of 2026-07-25).
CODED_DASH_FLIGHT_READS = {
    "phase", "detected",            # coded_dash_summary.engagement():50-52
    "meas_range", "gt_range",       # coded_dash_summary._is_real_target():32
}
PARSE_AB_FLIGHT_READS = {
    "phase", "detected",            # parse_ab.flight_metrics/is_det
    "meas_range", "gt_range",       # parse_ab.is_real
    "t", "alt_m",                   # parse_ab ulog alignment ticks:184-186
    "gt_cam_x", "gt_cam_y", "gt_cam_z",
    "gt_tag_x", "gt_tag_y", "gt_tag_z",   # parse_ab.vert_cam():202-204
}
AUDIT_FLIGHT_READS = (
    set(audit_per_tick.REQUIRED_COLUMNS_AB)      # introspected, not re-typed
    | set(audit_per_tick.REQUIRED_COLUMNS_C)
    | set(audit_per_tick.REQUIRED_COLUMNS_D)
    | {"law",                       # infer_law_from_flight_csv():182
       "vc_m_s"}                    # check (c) pre-CPA power gate:425
)

# Arm-CSV columns read by name.
CODED_DASH_ARM_READS = {"direction", "miss_m", "clean", "flight_csv_path"}
PARSE_AB_ARM_READS = {"run_idx", "direction", "miss_m", "clean",
                      "breakoff_reason", "flight_csv_path"}
AUDIT_ARM_READS = {"run_idx", "law", "clean", "handoff", "miss_m",
                   "flight_csv_path"}          # _load_arm_rows_inline():131-137


def test_flight_columns_every_consumer_reads_exist_in_producer_header():
    """The core contract: a rename in CSV_HEADER (or a consumer reading a
    column the producer never wrote) fails HERE, mechanically, instead of
    silently classifying every flight the same way in the field."""
    hdr = set(CSV_HEADER)
    for who, reads in [("coded_dash_summary", CODED_DASH_FLIGHT_READS),
                       ("loft_dive/parse_ab", PARSE_AB_FLIGHT_READS),
                       ("audit_per_tick", AUDIT_FLIGHT_READS)]:
        missing = sorted(reads - hdr)
        assert not missing, (
            f"{who} reads flight-CSV column(s) the producer does not write "
            f"(schema drift): {missing}  [producer schema via {CSV_HEADER_SOURCE}]")


def test_arm_columns_every_consumer_reads_exist_in_arm_header():
    hdr = set(ARM_HEADER)
    for who, reads in [("coded_dash_summary.summarize", CODED_DASH_ARM_READS),
                       ("loft_dive/parse_ab.arm_report", PARSE_AB_ARM_READS),
                       ("audit_per_tick.load_arm_flights", AUDIT_ARM_READS)]:
        missing = sorted(reads - hdr)
        assert not missing, (
            f"{who} reads arm-CSV column(s) mc_batch.sh does not write "
            f"(schema drift): {missing}")


def test_enumerated_reads_cannot_go_stale_source_sweep():
    """Mechanical completeness guard on the enumerations above: sweep each
    consumer's source for row reads (r/prev/raw/flight .get('col') / ['col'] /
    fnum(r,'col')) and require every name to exist in a producer header. If a
    future edit adds a read of a NEW column, this fails until the contract —
    header and enumeration both — is updated consciously."""
    row_vars = "r|prev|raw|flight"
    pats = [re.compile(rf"\b(?:{row_vars})\.get\(\s*[\"']([A-Za-z_][A-Za-z0-9_]*)[\"']"),
            re.compile(rf"\b(?:{row_vars})\[\s*[\"']([A-Za-z_][A-Za-z0-9_]*)[\"']\s*\]"),
            re.compile(r"\bfnum\(\s*r\s*,\s*[\"']([A-Za-z_][A-Za-z0-9_]*)[\"']")]
    # Non-CSV dict keys reached through a row-named variable (each justified):
    allowed_extra = {
        "raw",   # audit_per_tick.load_arm_flights: ekf_ab_analyze's wrapper dict key
    }
    known = set(CSV_HEADER) | set(ARM_HEADER) | allowed_extra
    # SWEEP FLOORS (added 2026-08-10). Without these the guard is VACUOUS: the
    # regexes bind specific row-variable names, so an ordinary refactor renaming
    # `r` to `rec` drops the swept count to ZERO and `assert not unknown` still
    # passes -- having checked nothing. Mutant-proven that night: 8 -> 0 reads,
    # test still green. A verdict computed on zero units is never a PASS
    # (CLAUDE.md; docs/error_handling_policy.md), and the sibling guard
    # tests/test_ci_gz_deselect_list.py already carries the same protection under
    # the name test_the_measurement_is_not_vacuous.
    #
    # The floors are deliberately BELOW the counts measured on 2026-08-10
    # (8 / 18 / 21) so ordinary edits do not trip them, while a sweep that
    # collapses to near-zero does. Raise them if a consumer grows substantially.
    floors = {"coded_dash_summary.py": 5, "parse_ab.py": 12,
              "audit_per_tick.py": 14}
    for fname in [os.path.join(SCRIPTS, "coded_dash_summary.py"),
                  os.path.join(LOFT_DIVE, "parse_ab.py"),
                  os.path.join(SCRIPTS, "audit_per_tick.py")]:
        with open(fname) as f:
            src = f.read()
        reads = set()
        for pat in pats:
            reads |= set(pat.findall(src))
        base = os.path.basename(fname)
        floor = floors[base]
        assert len(reads) >= floor, (
            f"{base}: the completeness sweep found only {len(reads)} row "
            f"read(s), below the floor of {floor}. The guard is not finding the "
            f"reads it is supposed to check, so its PASS would be meaningless. "
            f"Most likely the row variable was renamed away from "
            f"'{row_vars}' -- update the pattern, do not lower the floor.")
        unknown = sorted(reads - known)
        assert not unknown, (
            f"{base} reads name(s) not in any producer "
            f"header (new column, rename, or a new non-CSV key needing the "
            f"allowlist): {unknown}")


# ------------------------------------------------------------- layer 2: behavior
def test_real_target_engage_classifies_camera_guided(tmp_path):
    """A flight whose ENGAGE detections agree with gt range (real target) must
    count as CAMERA-GUIDED by the real consumer function."""
    fp = write_flight_csv(str(tmp_path / "real_flight.csv"), CSV_HEADER)
    got = coded_dash_summary.engagement(fp)
    assert got is not None, "producer-schema fixture unreadable by the consumer"
    total, eng, eng_det, eng_real = got
    assert (total, eng, eng_det) == (28, 20, 20)
    assert eng_real == 20, (
        "known-real ENGAGE ticks did not classify as REAL -- the "
        "camera-guided/dash-only split is broken")


def test_phantom_only_engage_classifies_dash_only(tmp_path):
    """Same geometry, phantom meas_range (1.6 m vs gt 16->4.6 m): ZERO ticks may
    classify as real — a phantom-triggered ENGAGE flying dash ballistics must
    read dash-only (the rebal-mirage, one level up)."""
    fp = write_flight_csv(str(tmp_path / "phantom_flight.csv"), CSV_HEADER,
                          phantom=True)
    total, eng, eng_det, eng_real = coded_dash_summary.engagement(fp)
    assert (eng, eng_det) == (20, 20)
    assert eng_real == 0, "phantom detections classified as the real target"


def test_arm_csv_end_to_end_summarize_splits_camera_vs_dash(tmp_path):
    """Full pair: mc_batch-schema arm CSV -> summarize() -> per-flight
    engagement audit of producer-schema flight CSVs."""
    real = write_flight_csv(str(tmp_path / "real.csv"), CSV_HEADER)
    ph = write_flight_csv(str(tmp_path / "phantom.csv"), CSV_HEADER, phantom=True)
    arm = write_arm_csv(str(tmp_path / "arm.csv"), ARM_HEADER, [
        {"run_idx": 0, "law": "pronav", "miss_m": "1.10", "clean": "1",
         "handoff": "1", "direction": "l2r", "flight_csv_path": real,
         "config": "pronav_9.000"},
        {"run_idx": 1, "law": "pronav", "miss_m": "0.90", "clean": "1",
         "handoff": "1", "direction": "l2r", "flight_csv_path": ph,
         "config": "pronav_9.000"},
    ])
    s = coded_dash_summary.summarize(arm, 2.5)
    assert s["l2r"]["n"] == 2 and s["l2r"]["eng"] == 2
    assert s["l2r"]["cam"] == 1, "the real-target flight must count camera-guided"
    assert s["l2r"]["dash_only"] == 1, "the phantom flight must count dash-only"
    # And the audit's arm loader resolves the same pair (arm boundary, consumer 3).
    flights = audit_per_tick.load_arm_flights(arm)
    assert [f["flight_csv_path"] for f in flights] == [real, ph]
    assert all(f["law"] == "pronav" for f in flights)


def test_audit_per_tick_passes_an_honest_producer_schema_flight(tmp_path):
    """The honesty audit must fully RUN (not ERROR/SKIP) on a flight written
    through the producer's own header, and its powered check (c) must engage."""
    fp = write_flight_csv(str(tmp_path / "real.csv"), CSV_HEADER)
    res = audit_per_tick.audit_flight_csv(fp, "pronav")
    assert res["status"] == "PASS", res["fail_reasons"]
    assert res["checks"]["a"]["result"] == "PASS"
    assert res["checks"]["b"]["result"] == "PASS"   # CODED_DASH counts as dash
    assert res["checks"]["c"]["result"] == "PASS", res["checks"]["c"]
    assert "20 live pre-latch rows" in res["checks"]["c"]["detail"]
    assert "0 latched-coast rows excluded" in res["checks"]["c"]["detail"]


def test_audit_reports_error_not_pass_when_a_required_column_vanishes(tmp_path):
    """Mutant drill: drop `detected` from the schema -- the audit must say
    ERROR (schema mismatch), never PASS. This is the loud path the 2026-07-24
    breaks lacked."""
    hdr = [c for c in CSV_HEADER if c != "detected"]
    fp = str(tmp_path / "mutant.csv")
    with open(fp, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(hdr)
        w.writerow([""] * len(hdr))
    res = audit_per_tick.audit_flight_csv(fp, "pronav")
    assert res["status"] == "ERROR"
    assert any("detected" in r for r in res["fail_reasons"])


def test_parse_ab_band_recall_real_vs_phantom(tmp_path):
    """Phase A's five decisive numbers come from these counters: pin the 8-12 m
    band recall and the near-band trap against the analytic fixture (7 in-band
    ENGAGE ticks at gt 11.8..8.2, one at 4.6). Filenames carry no UTC stamp, so
    the ulog branch is (by construction) never entered."""
    real = write_flight_csv(str(tmp_path / "real.csv"), CSV_HEADER)
    m = parse_ab.flight_metrics(real, 0.0)
    assert m["band_ticks"] == 7 and m["band_det"] == 7 and m["band_real"] == 7
    assert m["near_ticks"] == 1 and m["near_real"] == 1
    assert m["eng_ticks"] == 20 and m["eng_real"] == 20

    ph = write_flight_csv(str(tmp_path / "phantom.csv"), CSV_HEADER, phantom=True)
    m2 = parse_ab.flight_metrics(ph, 0.0)
    assert m2["band_ticks"] == 7 and m2["band_det"] == 7
    assert m2["band_real"] == 0, "phantom ticks leaked into REAL recall"
    assert m2["eng_real"] == 0


def test_is_real_criterion_identical_across_consumers(tmp_path):
    """parse_ab.is_real documents itself as coded_dash_summary._is_real_target;
    mirror drift between the two is exactly the class that bit at
    project_state.json:575. Pin them row-for-row on both fixtures."""
    for phantom in (False, True):
        fp = write_flight_csv(str(tmp_path / f"f_{phantom}.csv"), CSV_HEADER,
                              phantom=phantom)
        with open(fp, newline="") as f:
            rows = list(csv.DictReader(f))
        for r in rows:
            assert parse_ab.is_real(r) == coded_dash_summary._is_real_target(r), (
                f"is_real mirror drift on row {r.get('phase')}/{r.get('t')} "
                f"(meas={r.get('meas_range')}, gt={r.get('gt_range')})")
            assert parse_ab.is_det(r) == (r.get("detected") in ("1", "True", "true"))
