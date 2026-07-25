#!/usr/bin/env python3
"""PRODUCER -> CONSUMER contract test for the onboard decision log.

  PRODUCER: flight/deploy/real_flight.py  (--log-csv, 36 columns, `_CSV_FIELDS`)
  CONSUMER: scripts/field/parse_flight_log.py

THE RULE THIS OBEYS (CLAUDE.md, "instruments are evidence"): when a tool crosses
a file boundary it earns a contract test WHOSE FIXTURE COMES FROM THE PRODUCER'S
OWN WRITER. A hand-typed fixture is exactly what hid this project's schema breaks
-- both sides passed their own self-tests and nothing tested the PAIR. So every
CSV here is produced by CALLING `real_flight.run_offline(..., csv_path=...)`,
which walks the real state machine with the real `_csv_row`/`_CSV_HEADER` writer
and writes a real file. Nothing below types a row.

The audit's own critic filed the weaker sibling of this test
(tests/test_tripod_pipeline_join.py imports the producer's HEADER but hand-writes
the row VALUES, so the writer path is never exercised); this file avoids that
shape deliberately.

WHAT IT PROVES
  * the consumer parses a real produced log, with the header matching column-for-column
  * a REAL flight's state machine, failsafes and ENGAGE stats survive the round trip
  * ADDING or DROPPING a producer column is caught (mutation test on both sides)
  * the zero-command mirror agrees with scripts/audit_per_tick.py's constants
  * the no-vacuous-verdict rule holds on a real-but-empty log

Run: `.venv/bin/python -m pytest tests/test_parse_flight_log_contract.py -v`
"""
import csv
import importlib.util
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

from flight.deploy.real_flight import (  # noqa: E402
    _CSV_FIELDS,
    _CSV_HEADER,
    GateReadyTrigger,
    MissionConfig,
    ScriptedTrigger,
    State,
    run_offline,
)

_PARSER_PATH = os.path.join(REPO_ROOT, "scripts", "field", "parse_flight_log.py")
_spec = importlib.util.spec_from_file_location("parse_flight_log", _PARSER_PATH)
pfl = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pfl)


# ============================================================ producer fixtures
def _cfg(**kw):
    """The same arm-gate-satisfying config flight/tests/test_real_flight.py uses."""
    base = dict(preflight_heading_deg=90.0, standby_alt_m=7.0,
                standby_settle_s=0.0, dash_speed_ms=10.0, dash_accel_ms2=10.0)
    base.update(kw)
    return MissionConfig(**base)


def _guidance():
    """The REAL terminal the driver composes in --dry-run (`SeekerGuidance` over
    the same `flight/` core), so the produced CSV carries the guidance half of
    the schema (sp_source/bearing/lambda/r_hat/...) instead of 20 blank columns.
    With `guidance=None` the whole guidance block is empty and the contract test
    would silently only cover 16 of 36 columns."""
    from flight.camera import CameraModel
    from flight.deploy.seeker_loop import GuidanceConfig, SeekerGuidance
    gcfg = GuidanceConfig(alt_ref_m=7.0)
    cam = CameraModel(539.936, 539.936, 640.0, 480.0)
    return SeekerGuidance(gcfg, cam, gcfg.target_span_m)


@pytest.fixture(scope="module")
def full_flight(tmp_path_factory):
    """A COMPLETE flight written by the producer's own writer: STANDBY ->
    CODED_DASH -> ENGAGE -> BREAKOFF -> SAFE."""
    path = str(tmp_path_factory.mktemp("prod") / "real_flight_full.csv")
    sm, rows = run_offline(_cfg(engage_max_s=3.0, breakoff_s=0.5, safe_hold_s=0.5),
                           GateReadyTrigger(), guidance=_guidance(), fps=20.0,
                           max_s=60.0, verbose=False, csv_path=path)
    assert os.path.isfile(path), "the producer did not write the CSV at all"
    return path, sm, rows


def test_the_fixture_exercises_the_GUIDANCE_half_of_the_schema(full_flight):
    """Guards the fixture itself: if a future refactor drops the guidance object,
    20 of the 36 columns go blank and every assertion below would still pass
    while covering nothing."""
    path, _sm, _rows = full_flight
    _hdr, rows = pfl.load_rows(path)
    for col in ("sp_source", "bearing_deg", "lambda_deg", "r_hat_m",
                "own_state_ok", "v_north", "state", "streak"):
        assert any((r.get(col) or "") != "" for r in rows), (
            f"column {col!r} is empty on EVERY produced row -- the fixture is "
            f"not exercising the writer path that fills it")


@pytest.fixture(scope="module")
def link_denied_flight(tmp_path_factory):
    """A flight the LINK failsafe sends straight to SAFE -- exercises the
    safe_reason column, which is the only record of WHY a real flight aborted."""
    path = str(tmp_path_factory.mktemp("prod") / "real_flight_linkdead.csv")
    trig = ScriptedTrigger(go_at_s=None, link_dies_at_s=0.5)
    sm, rows = run_offline(_cfg(link_acquire_grace_s=0.0, safe_hold_s=0.5),
                           trig, guidance=None, fps=20.0, max_s=20.0,
                           verbose=False, csv_path=path)
    return path, sm, rows


# ============================================================ the seam
def test_the_producer_writes_a_file_the_consumer_parses(full_flight):
    path, sm, rows = full_flight
    header, parsed = pfl.load_rows(path)
    assert header == list(_CSV_FIELDS), (
        "the consumer's DictReader header must be the producer's field tuple")
    assert len(parsed) == len(rows), (
        f"consumer read {len(parsed)} ticks, producer wrote {len(rows)}")


def test_the_consumer_validates_the_header_against_the_producer(full_flight):
    path, _sm, _rows = full_flight
    s = pfl.summarize(*pfl.load_rows(path))
    assert s["header_matches_producer"] is True
    assert s["missing_columns"] == [] and s["extra_columns"] == []
    code, notes = pfl.verdicts(s)
    assert code == pfl.EXIT_OK, notes


def test_the_state_machine_survives_the_round_trip(full_flight):
    """What the producer's own `sm.visited` says the flight did must be what the
    consumer recovers from the FILE -- the whole point of the log."""
    path, sm, _rows = full_flight
    s = pfl.summarize(*pfl.load_rows(path))
    assert s["states_visited"] == sm.visited, (s["states_visited"], sm.visited)
    assert s["terminal_state"] == sm.visited[-1] == State.SAFE
    assert s["terminated_in_safe"] is True
    assert State.CODED_DASH in s["state_ticks"] and State.ENGAGE in s["state_ticks"]
    assert sum(s["state_ticks"].values()) == s["n_ticks"]
    # transitions must be one per state entry, in order, with rising timestamps
    assert [t["to"] for t in s["transitions"]] == sm.visited
    stamps = [t["t_s"] for t in s["transitions"]]
    assert stamps == sorted(stamps)


def test_the_failsafe_reason_reaches_the_consumer(link_denied_flight):
    path, sm, _rows = link_denied_flight
    s = pfl.summarize(*pfl.load_rows(path))
    reasons = [r["reason"] for r in s["safe_reasons"]]
    assert reasons, "a flight that aborted recorded NO safe_reason in its log"
    assert sm.safe_reason in reasons, (sm.safe_reason, reasons)
    assert any(r.startswith("link_denied") for r in reasons), reasons
    assert s["terminal_state"] == State.SAFE


def test_engage_stats_are_recovered_from_a_real_flight(full_flight):
    path, _sm, _rows = full_flight
    s = pfl.summarize(*pfl.load_rows(path))
    e = s["engage"]
    assert e["n_ticks"] > 0, "the fixture flight must actually reach ENGAGE"
    assert e["n_detected_ticks"] > 0, "the stub seeker must produce detections"
    assert sum(e["sp_source"].values()) == e["n_ticks"]
    # the stub seeker shrinks the range, so a real span must be recovered
    assert e["n_r_hat_present"] > 0
    assert e["r_hat_max_m"] >= e["r_hat_min_m"]


def test_a_real_flight_is_not_flagged_as_a_frozen_terminal(full_flight):
    """The zero-command gate must not fire on a healthy flight (no false alarm),
    and its population must be counted, not empty."""
    path, _sm, _rows = full_flight
    s = pfl.summarize(*pfl.load_rows(path))
    z = s["zero_cmd"]
    assert z["population"] > 0, "the gate scored ZERO ticks -- a vacuous verdict"
    assert z["frac"] <= z["fail_frac"], z


# ============================================================ MUTATION TESTS
# A contract test that cannot FAIL on a break is decoration. These mutate the
# produced file the way a schema change would and assert the consumer notices.
def test_dropping_a_producer_column_is_caught(full_flight, tmp_path):
    path, _sm, _rows = full_flight
    mutant = tmp_path / "dropped.csv"
    with open(path) as fin, open(mutant, "w", newline="") as fout:
        r = csv.reader(fin)
        w = csv.writer(fout)
        for row in r:
            w.writerow(row[:-1])          # drop the last column (safe_reason)
    s = pfl.summarize(*pfl.load_rows(str(mutant)))
    assert s["header_matches_producer"] is False
    assert s["missing_columns"] == [_CSV_FIELDS[-1]]
    code, notes = pfl.verdicts(s)
    assert code == pfl.EXIT_FAIL, notes


def test_adding_a_producer_column_is_caught(full_flight, tmp_path):
    path, _sm, _rows = full_flight
    mutant = tmp_path / "added.csv"
    with open(path) as fin, open(mutant, "w", newline="") as fout:
        r = csv.reader(fin)
        w = csv.writer(fout)
        head = next(r)
        w.writerow(head + ["brand_new_column"])
        for row in r:
            w.writerow(row + ["0"])
    s = pfl.summarize(*pfl.load_rows(str(mutant)))
    assert s["header_matches_producer"] is False
    assert s["extra_columns"] == ["brand_new_column"]
    assert pfl.verdicts(s)[0] == pfl.EXIT_FAIL


def test_reordering_producer_columns_is_caught(full_flight, tmp_path):
    """A reorder keeps every NAME present -- only an ordered comparison sees it,
    and getting it wrong silently mis-assigns every value."""
    path, _sm, _rows = full_flight
    mutant = tmp_path / "reordered.csv"
    with open(path) as fin, open(mutant, "w", newline="") as fout:
        r = csv.reader(fin)
        w = csv.writer(fout)
        for row in r:
            w.writerow([row[1], row[0]] + row[2:])
    s = pfl.summarize(*pfl.load_rows(str(mutant)))
    assert s["header_matches_producer"] is False
    assert s["missing_columns"] == [] and s["extra_columns"] == []
    assert pfl.verdicts(s)[0] == pfl.EXIT_FAIL


def test_a_frozen_terminal_is_DETECTED_in_a_real_produced_log(full_flight, tmp_path):
    """THE DEFECT THE GATE EXISTS FOR, injected into a REAL produced file: zero
    every ENGAGE row's horizontal command (the frozen-vworld shape that flew 29/44
    engaged sim flights while the auditor printed PASS) and require a FAIL."""
    path, _sm, _rows = full_flight
    i_state = _CSV_FIELDS.index("state")
    i_vn, i_ve = _CSV_FIELDS.index("v_north"), _CSV_FIELDS.index("v_east")
    mutant = tmp_path / "frozen.csv"
    n_zeroed = 0
    with open(path) as fin, open(mutant, "w", newline="") as fout:
        r = csv.reader(fin)
        w = csv.writer(fout)
        w.writerow(next(r))
        for row in r:
            if row[i_state] == State.ENGAGE:
                row[i_vn] = row[i_ve] = "0.0000"
                n_zeroed += 1
            w.writerow(row)
    assert n_zeroed > 0, "the mutation touched nothing -- the test is vacuous"

    s = pfl.summarize(*pfl.load_rows(str(mutant)))
    code, notes = pfl.verdicts(s)
    assert code == pfl.EXIT_FAIL, notes
    assert any("DID NOT ACTUATE" in t for _l, t in notes), notes
    assert s["zero_cmd"]["frac"] == 1.0


def test_an_empty_but_wellformed_log_is_UNCERTAIN_not_a_pass(tmp_path):
    """Header from the producer, zero ticks -- the no-vacuous-verdict rule."""
    p = tmp_path / "empty.csv"
    p.write_text(_CSV_HEADER)
    s = pfl.summarize(*pfl.load_rows(str(p)))
    code, notes = pfl.verdicts(s)
    assert s["n_ticks"] == 0
    assert code == pfl.EXIT_UNCERTAIN, notes
    assert code != pfl.EXIT_OK


def test_a_flight_that_never_engaged_is_not_silently_OK(link_denied_flight):
    """A link-denied abort never reaches ENGAGE. Default: UNCERTAIN (honest --
    there is no terminal to verdict). With --require-engage: FAIL."""
    path, _sm, _rows = link_denied_flight
    s = pfl.summarize(*pfl.load_rows(path))
    assert s["engage"].get("n_ticks", 0) == 0
    assert pfl.verdicts(s, require_engage=False)[0] == pfl.EXIT_UNCERTAIN
    assert pfl.verdicts(s, require_engage=True)[0] == pfl.EXIT_FAIL


def test_fail_beats_uncertain_when_both_fire(full_flight, tmp_path):
    """EXIT_UNCERTAIN is 3 and EXIT_FAIL is 1, so a naive max() would let an
    UNCERTAIN mask a FAIL. Pin the severity ordering."""
    assert pfl.worse(pfl.EXIT_UNCERTAIN, pfl.EXIT_FAIL) == pfl.EXIT_FAIL
    assert pfl.worse(pfl.EXIT_OK, pfl.EXIT_UNCERTAIN) == pfl.EXIT_UNCERTAIN
    assert pfl.worse(pfl.EXIT_FAIL, pfl.EXIT_UNCERTAIN) == pfl.EXIT_FAIL


# ============================================================ mirror pin
def test_the_zero_command_constants_MIRROR_the_sim_auditor():
    """The field parser and scripts/audit_per_tick.py must score the same defect
    with the SAME numbers, or a sim finding and a field finding are not
    comparable. Pinned in both directions."""
    import audit_per_tick as apt
    pfl._sync_zero_cmd_constants()
    assert pfl.ZERO_CMD_TOL == apt.ZERO_CMD_TOL == 1e-6
    assert pfl.ZERO_CMD_FAIL_FRAC == apt.ZERO_CMD_FAIL_FRAC == 0.5
    assert pfl.ZERO_CMD_MIN_N == apt.ZERO_CMD_MIN_N == 10


def test_the_parser_self_test_passes():
    """Stage-4 wiring guard: the entry point run_tests.sh invokes must exit 0."""
    assert pfl.self_test() == 0


def test_the_cli_returns_the_verdict_exit_code(full_flight, tmp_path):
    """End-to-end through main(), including the --json sidecar, so the exit code
    a field operator sees is the one the verdict computed."""
    path, _sm, _rows = full_flight
    out = tmp_path / "summary.json"
    rc = pfl.main([path, "--json", str(out)])
    assert rc == pfl.EXIT_OK
    assert out.is_file()
    import json
    blob = json.loads(out.read_text())
    assert blob["verdict"]["exit"] == pfl.EXIT_OK
    assert blob["header_matches_producer"] is True
    assert blob["zero_cmd"]["population"] > 0
