#!/usr/bin/env python3
"""PRODUCER -> CONSUMER contract test for the anemometer reading that feeds the
pre-flight wind trim.

    scripts/wind_anemometer_sim.py   PRODUCER (sim harness; may read the sim's
                                     configured TRUE wind, adds instrument error)
                 |  reading JSON
                 v
    scripts/wind_trim.py             CONSUMER (flight-side correction; stdlib
                                     only; refuses any gt_-shaped key)

CLAUDE.md's rule, learned the hard way twice: "when a tool crosses a file
boundary, it earns a producer->consumer contract test WHOSE FIXTURE COMES FROM
THE PRODUCER'S OWN WRITER -- a hand-typed fixture is exactly what hid the
schema breaks (both scripts passed their own self-tests; nothing tested the
pair)". So every fixture below is produced by
`wind_anemometer_sim.write_reading_json`, never typed here.

What is pinned:
  * the round trip producer-writes -> consumer-loads is exact;
  * the emitted file contains NO truth field, checked BOTH by key name AND by
    scanning every number reachable in the document (free-text provenance
    included) for the true wind values that made it -- so a leak is caught
    even if someone invents an innocuous key name or buries the truth in a
    log string. That scanner is itself proven to fire
    (test_the_by_value_scanner_actually_fires);
  * the instrument model is deterministic in its seed (arms pair) and is not
    a no-op (a "simulated" reading identical to the truth would be a
    given-perfect input wearing a measured costume);
  * the whole chain runs end to end: simulate -> write -> load -> apply trim.

Run: `.venv/bin/python -m pytest tests/test_wind_trim_contract.py -v`
"""
import json
import re
import os
import statistics
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

from wind_anemometer_sim import simulate_reading, write_reading_json  # noqa: E402
from wind_trim import (  # noqa: E402
    PLACEHOLDER_ANEMOMETER_SPEC,
    READING_SCHEMA,
    GroundTruthLeakError,
    SagTable,
    WindTrim,
    apply_wind_trim,
    load_reading_json,
)

TRUE_WIND_MPS = 4.5          # sim CONFIG (the EXPECTED tier), gt side
TRUE_DIR_FROM_DEG = 270.0    # met convention: blowing FROM the west

SYNTH_TABLE = SagTable(points=((0.0, 0.0), (4.5, 0.6), (8.0, 1.8)),
                       provenance="SYNTHETIC contract-test table -- NOT MEASURED")


def _produce(tmp_path, seed=7, name="reading.json"):
    """FIXTURE VIA THE PRODUCER'S OWN WRITER -- never hand-typed JSON."""
    reading = simulate_reading(TRUE_WIND_MPS, TRUE_DIR_FROM_DEG, seed=seed,
                               taken_at_sim_t=0.0)
    return write_reading_json(str(tmp_path / name), reading), reading


def test_producer_output_loads_in_the_consumer_unchanged(tmp_path):
    path, produced = _produce(tmp_path)
    loaded = load_reading_json(path)
    assert loaded == produced, (
        "the reading changed crossing the file boundary -- writer and reader have "
        "drifted (this is the exact schema-break class the contract-test rule exists "
        "for).")


def test_wire_form_carries_the_declared_schema(tmp_path):
    path, _ = _produce(tmp_path)
    with open(path, encoding="utf-8") as fh:
        data = json.load(fh)
    assert data["schema"] == READING_SCHEMA
    assert set(data) == {"schema", "speed_mps", "dir_from_deg", "sigma_speed_mps",
                         "sigma_dir_deg", "taken_at_sim_t", "provenance"}


def test_wire_form_contains_no_truth_field_by_key(tmp_path):
    path, _ = _produce(tmp_path)
    with open(path, encoding="utf-8") as fh:
        data = json.load(fh)
    for key in data:
        low = key.lower()
        assert not any(p in low for p in ("gt_", "truth", "true_", "sim_wind",
                                          "world_wind", "actual")), (
            f"the producer emitted {key!r}: the simulator's true wind must never cross "
            "into the correction path's input file.")


def _all_numbers_in(obj):
    """Every number reachable in a parsed JSON document, INCLUDING numbers
    embedded in free text (a leak hidden in a provenance string is still a
    leak). Numeric, not substring, matching -- a substring scan for "4.5"
    would false-fire on a noisy reading of 4.51 m/s, which is a ~13% flake at
    the instrument's 0.3 m/s sigma."""
    out = []
    if isinstance(obj, bool) or obj is None:
        return out
    if isinstance(obj, (int, float)):
        out.append(float(obj))
    elif isinstance(obj, str):
        out.extend(float(m) for m in re.findall(r"-?\d+(?:\.\d+)?", obj))
    elif isinstance(obj, dict):
        for k, v in obj.items():
            out.extend(_all_numbers_in(k))
            out.extend(_all_numbers_in(v))
    elif isinstance(obj, (list, tuple)):
        for v in obj:
            out.extend(_all_numbers_in(v))
    return out


def _truth_leaks(doc):
    nums = _all_numbers_in(doc)
    return [n for n in nums
            if abs(n - TRUE_WIND_MPS) < 1e-9 or abs(n - TRUE_DIR_FROM_DEG) < 1e-9]


def test_wire_form_contains_no_truth_field_by_value(tmp_path):
    """Key-name scanning can be defeated by an innocuous-looking key (or by a
    truth value helpfully logged inside a free-text provenance string), so
    also scan every number reachable in the document -- text included -- for
    the TRUE wind that made the reading."""
    path, reading = _produce(tmp_path)
    doc = json.load(open(path, encoding="utf-8"))
    assert reading.speed_mps != TRUE_WIND_MPS, "instrument model produced a noiseless read"
    leaks = _truth_leaks(doc)
    assert leaks == [], (
        f"the produced reading file contains the TRUE wind value(s) {leaks} -- the sim's "
        "configured wind leaked into the operator-reading file (check the provenance "
        "string as well as the numeric fields).")


def test_the_by_value_scanner_actually_fires(tmp_path):
    """MUTANT / instrument check: a producer that helpfully writes the true
    wind into the PROVENANCE TEXT must be caught. An audit that has never been
    seen to fail is not evidence -- and the first version of this scanner
    (raw-substring based) missed exactly this case."""
    from wind_trim import AnemometerReading
    leaky = AnemometerReading(
        4.4232, 277.67, 0.3, 15.0, 0.0,
        f"SIMULATED; true wind was {TRUE_WIND_MPS} m/s from {TRUE_DIR_FROM_DEG}")
    path = write_reading_json(str(tmp_path / "leaky.json"), leaky)
    doc = json.load(open(path, encoding="utf-8"))
    assert _truth_leaks(doc), "the by-value scanner failed to catch a provenance leak"


def test_consumer_refuses_a_producer_that_starts_leaking(tmp_path):
    """The failure MODE, exercised: if the producer were changed to append the
    truth 'just for logging', the consumer fails closed instead of using it."""
    path, _ = _produce(tmp_path)
    data = json.load(open(path, encoding="utf-8"))
    data["gt_wind_mps"] = TRUE_WIND_MPS
    json.dump(data, open(path, "w", encoding="utf-8"))
    with pytest.raises(GroundTruthLeakError):
        load_reading_json(path)


def test_instrument_model_is_deterministic_in_its_seed(tmp_path):
    a = simulate_reading(TRUE_WIND_MPS, TRUE_DIR_FROM_DEG, seed=11, taken_at_sim_t=0.0)
    b = simulate_reading(TRUE_WIND_MPS, TRUE_DIR_FROM_DEG, seed=11, taken_at_sim_t=0.0)
    c = simulate_reading(TRUE_WIND_MPS, TRUE_DIR_FROM_DEG, seed=12, taken_at_sim_t=0.0)
    assert a == b, "same seed must give the same reading (A/B arms pair on the seed)"
    assert a != c, "different seeds must give different instrument realizations"


def test_instrument_model_does_not_touch_the_global_rng():
    """A local random.Random(seed) -- otherwise this call would perturb every
    other seeded stream in a flight and quietly break pairing."""
    import random
    random.seed(1234)
    first = [random.random() for _ in range(3)]
    random.seed(1234)
    simulate_reading(TRUE_WIND_MPS, TRUE_DIR_FROM_DEG, seed=3)
    second = [random.random() for _ in range(3)]
    assert first == second


def test_instrument_error_matches_the_declared_spec():
    """The reading is NOT the truth: over 400 seeds the sample sigma sits
    inside a generous band around the declared spec sigma, and the mean sits
    near the truth plus the declared bias. This is what stops a 'simulated'
    instrument silently becoming a given-perfect input."""
    spec = PLACEHOLDER_ANEMOMETER_SPEC
    sigma = spec.sigma_speed_at(TRUE_WIND_MPS)          # max(0.3, 5% of 4.5) = 0.3
    reads = [simulate_reading(TRUE_WIND_MPS, TRUE_DIR_FROM_DEG, seed=s).speed_mps
             for s in range(400)]
    assert 0.6 * sigma < statistics.pstdev(reads) < 1.6 * sigma
    assert abs(statistics.fmean(reads) - (TRUE_WIND_MPS + spec.bias_speed_mps)) < 0.15
    assert all(r >= 0.0 for r in reads), "a cup anemometer cannot read negative"


def test_end_to_end_sim_to_trim(tmp_path):
    """simulate -> write -> load -> trim. The dash flies WEST (heading 270)
    into a wind FROM the west, so the operator's noisy reading resolves to a
    ~4.5 m/s headwind and the lead solve's assumed speed drops by the MEASURED
    (here synthetic) sag."""
    path, _ = _produce(tmp_path, seed=7)
    reading = load_reading_json(path)
    trim = WindTrim(enabled=True, reading=reading, table=SYNTH_TABLE,
                    max_reading_age_s=600.0)
    res = apply_wind_trim(16.0, 270.0, trim, now_sim_t=30.0)
    assert res.enabled and not res.is_noop
    assert res.headwind_mps == pytest.approx(4.4, abs=0.6)   # noisy read of a 4.5 truth
    assert 15.0 < res.corrected_speed_mps < 16.0
    assert "SIMULATED handheld anemometer" in res.log_line
    assert "no gt" in res.log_line
