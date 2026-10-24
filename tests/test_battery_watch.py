#!/usr/bin/env python3
"""Pure-logic tests for scripts/bench/battery_watch.py (no hardware, no
pymavlink import -- the module imports it lazily inside the burst function).

Covers: the OK/WARN/CRITICAL classifier at and around both thresholds, the
trailing-window slope fit, the time-to-threshold projection (discharging,
charging/flat -> None, already-below -> 0), and the no-vacuous-verdict rule
(zero samples -> UNKNOWN, stale sample -> UNKNOWN(stale)) via write_status.
"""
import importlib.util
import json
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_SPEC = importlib.util.spec_from_file_location(
    "battery_watch",
    os.path.join(_HERE, "..", "scripts", "bench", "battery_watch.py"))
BW = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(BW)


# ---- classifier -----------------------------------------------------------

def test_classify_thresholds():
    # 6S: WARN 22.8 V (3.80/cell), CRITICAL 22.2 V (3.70/cell)
    assert BW.classify(24.20) == "OK"
    assert BW.classify(22.81) == "OK"
    assert BW.classify(22.80) == "WARN"          # boundary is inclusive
    assert BW.classify(22.21) == "WARN"
    assert BW.classify(22.20) == "CRITICAL"      # boundary is inclusive
    assert BW.classify(20.00) == "CRITICAL"


def test_classify_custom_thresholds():
    assert BW.classify(15.2, warn_v=15.2, crit_v=14.8) == "WARN"   # 4S numbers
    assert BW.classify(14.8, warn_v=15.2, crit_v=14.8) == "CRITICAL"


# ---- slope fit ------------------------------------------------------------

def test_slope_needs_two_samples():
    assert BW.fit_slope_v_per_s([]) is None
    assert BW.fit_slope_v_per_s([(0.0, 24.0)]) is None


def test_slope_exact_on_linear_data():
    # 0.01 V lost per 60 s -> slope = -1/6000 V/s
    rows = [(t, 24.0 - 0.01 * (t / 60.0)) for t in (0.0, 60.0, 120.0, 180.0)]
    slope = BW.fit_slope_v_per_s(rows)
    assert abs(slope - (-0.01 / 60.0)) < 1e-12


def test_slope_window_excludes_old_samples():
    # Ancient steep decline, then a flat recent window: the fit must only see
    # the recent window (two equal samples inside it -> slope 0).
    rows = [(0.0, 25.0), (10_000.0, 23.0), (10_060.0, 23.0)]
    slope = BW.fit_slope_v_per_s(rows, window_s=900.0)
    assert slope == 0.0


# ---- projection -----------------------------------------------------------

def test_projection_linear_discharge():
    # 24.0 V falling 0.01 V/min: 22.8 V is 120 min = 7200 s out from t=180's
    # value of 23.97 V -> (23.97-22.8)/0.01 min = 117 min = 7020 s.
    rows = [(t, 24.0 - 0.01 * (t / 60.0)) for t in (0.0, 60.0, 120.0, 180.0)]
    proj = BW.project_time_to_s(rows, 22.8)
    assert abs(proj - 7020.0) < 1.0


def test_projection_none_when_charging_or_flat():
    rising = [(0.0, 23.0), (60.0, 23.1)]
    flat = [(0.0, 23.0), (60.0, 23.0)]
    assert BW.project_time_to_s(rising, 22.8) is None
    assert BW.project_time_to_s(flat, 22.8) is None


def test_projection_zero_when_already_below():
    rows = [(0.0, 23.0), (60.0, 22.5)]
    assert BW.project_time_to_s(rows, 22.8) == 0.0


def test_projection_none_without_samples():
    assert BW.project_time_to_s([], 22.8) is None


# ---- status verdicts (no vacuous OK) --------------------------------------

def test_status_unknown_with_zero_samples(tmp_path):
    path = str(tmp_path / "status.json")
    status = BW.write_status(path, rows=[], latest=None, n_miss=3,
                             cells=6, warn_v=22.8, crit_v=22.2,
                             interval_s=60.0)
    assert status["verdict"] == "UNKNOWN"
    assert status["n_misses"] == 3
    with open(path) as f:                      # file round-trips as JSON
        assert json.load(f)["verdict"] == "UNKNOWN"


def test_status_stale_sample_is_unknown(tmp_path):
    import time
    path = str(tmp_path / "status.json")
    old = time.time() - 1000.0                 # > 3 * 60 s interval
    latest = {"t_unix": old, "pack_v": 24.0, "current_a": None,
              "remaining_pct": 97.0}
    status = BW.write_status(path, rows=[(old, 24.0)], latest=latest,
                             n_miss=0, cells=6, warn_v=22.8, crit_v=22.2,
                             interval_s=60.0)
    assert status["verdict"] == "UNKNOWN(stale)"


def test_status_fresh_sample_classifies(tmp_path):
    import time
    path = str(tmp_path / "status.json")
    now = time.time()
    latest = {"t_unix": now, "pack_v": 22.5, "current_a": 0.4,
              "remaining_pct": 40.0}
    status = BW.write_status(path, rows=[(now - 60, 22.52), (now, 22.5)],
                             latest=latest, n_miss=0, cells=6,
                             warn_v=22.8, crit_v=22.2, interval_s=60.0)
    assert status["verdict"] == "WARN"
    assert status["cell_v"] == 3.75
