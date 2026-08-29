"""Tests for the WIND GATE-0 (W3) probe's measurement and verdict layer.

WHY THIS FILE EXISTS: the probe is an INSTRUMENT, and this project's dominant
defect class is a bug in a scorer/auditor that a paired control cannot see
(CLAUDE.md, "instruments are evidence"). The probe decides whether Gazebo
applies the wind wrench -- if its geometry or its verdict logic is wrong, it
either blesses an inert wind arm or condemns a working one, and in both cases
the CSV underneath looks fine.

The Gazebo half cannot run per-commit (it needs a sim boot, ~3 minutes), so the
PURE half is pinned here: quaternion -> tilt/bearing, the settled-window
summariser, and every branch of the pre-registered verdict.
"""

import math
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from wind_gate0_probe import (  # noqa: E402
    angle_diff_deg,
    build_verdict,
    circular_mean_deg,
    summarise,
    tilt_from_quaternion,
)

G = 9.80665
PREDICTED_TILT = math.degrees(math.atan((0.15 * 5.0 + 1.225 * 25.0 / 200.0) / G))
CFG = {"wind_mps": 5.0, "dir_from_deg": 270.0}


# ------------------------------------------------------------------ geometry
def quat_from_axis_angle(ax, ay, az, angle_deg):
    a = math.radians(angle_deg) / 2.0
    s = math.sin(a)
    n = math.sqrt(ax * ax + ay * ay + az * az)
    return (math.cos(a), ax / n * s, ay / n * s, az / n * s)


def test_level_attitude_is_zero_tilt():
    tilt, _bearing = tilt_from_quaternion(1.0, 0.0, 0.0, 0.0)
    assert tilt == pytest.approx(0.0, abs=1e-9)


def test_lean_west_reads_as_bearing_270():
    """A wind FROM the west (270) must produce a lean bearing of 270.

    Gazebo world is ENU (x=east, y=north, z=up). Leaning the thrust axis west
    means rotating the body about the +north (+y) axis by a NEGATIVE angle:
    that carries body +z toward -east. This is the sign convention the whole
    direction check rests on, so it is pinned rather than trusted.
    """
    q = quat_from_axis_angle(0, 1, 0, -PREDICTED_TILT)
    tilt, bearing = tilt_from_quaternion(*q)
    assert tilt == pytest.approx(PREDICTED_TILT, abs=1e-6)
    assert angle_diff_deg(bearing, 270.0) == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("dir_from_deg,axis,sign", [
    (270.0, (0, 1, 0), -1),   # wind from west  -> lean west
    (90.0, (0, 1, 0), +1),    # wind from east  -> lean east
    (180.0, (1, 0, 0), +1),   # wind from south -> lean south
    (0.0, (1, 0, 0), -1),     # wind from north -> lean north
])
def test_lean_bearing_matches_the_wind_it_opposes(dir_from_deg, axis, sign):
    """The aircraft leans TOWARD the direction the wind comes FROM.

    That identity is what makes the direction check readable: expected lean
    bearing == --dir-from-deg, with no extra 180 deg to get wrong.
    """
    q = quat_from_axis_angle(*axis, sign * 6.0)
    tilt, bearing = tilt_from_quaternion(*q)
    assert tilt == pytest.approx(6.0, abs=1e-6)
    assert abs(angle_diff_deg(bearing, dir_from_deg)) < 1e-6


def test_tilt_is_sign_agnostic_but_bearing_is_not():
    """Leaning east and leaning west give the SAME tilt magnitude.

    So magnitude alone cannot catch a sign error in the driver's ENU
    conversion -- which is exactly why the pre-registration makes direction a
    separate, independently-failing check instead of folding it into the band.
    """
    east = tilt_from_quaternion(*quat_from_axis_angle(0, 1, 0, +5.0))
    west = tilt_from_quaternion(*quat_from_axis_angle(0, 1, 0, -5.0))
    assert east[0] == pytest.approx(west[0])
    assert abs(angle_diff_deg(east[1], west[1])) == pytest.approx(180.0, abs=1e-6)


def test_degenerate_quaternion_does_not_fabricate_an_attitude():
    tilt, bearing = tilt_from_quaternion(0.0, 0.0, 0.0, 0.0)
    assert math.isnan(tilt) and math.isnan(bearing)


def test_circular_mean_wraps_across_north():
    assert circular_mean_deg([359.0, 1.0]) == pytest.approx(0.0, abs=1e-6)
    assert math.isnan(circular_mean_deg([]))


# ---------------------------------------------------------------- summariser
def _rows(tilt, bearing, n=200, dt=0.1, drift=0.0):
    return [{"t_sim": i * dt, "t_phase": i * dt, "east_m": i * dt * drift,
             "north_m": 0.0, "up_m": 6.0, "tilt_deg": tilt,
             "lean_bearing_deg": bearing} for i in range(n)]


def test_summarise_scores_only_the_tail_window():
    """The settled read must ignore the transient at the START of the phase."""
    rows = _rows(0.0, 0.0, n=100)          # first 10 s: level (transient)
    rows += [{"t_sim": 10 + i * 0.1, "t_phase": 10 + i * 0.1, "east_m": 0.0,
              "north_m": 0.0, "up_m": 6.0, "tilt_deg": 5.0,
              "lean_bearing_deg": 270.0} for i in range(100)]  # then settled
    s = summarise(rows, tail_sim_s=5.0)
    assert s["mean_tilt_deg"] == pytest.approx(5.0, abs=1e-6)
    assert s["n_samples_phase"] == 200


def test_summarise_ignores_bearings_from_level_samples():
    """A bearing computed off numerical noise at ~0 tilt is a fabricated
    direction; the summariser must not average it in."""
    rows = _rows(0.01, 123.0, n=100)
    s = summarise(rows, tail_sim_s=5.0)
    assert s["n_bearing_samples"] == 0
    assert math.isnan(s["mean_lean_bearing_deg"])


def test_summarise_reports_drift():
    s = summarise(_rows(5.0, 270.0, n=100, drift=1.0), tail_sim_s=5.0)
    assert s["drift_within_window_m"] > 4.0


# ------------------------------------------------------------------- verdict
def _phase(tilt, bearing=270.0, n=200, drift=0.1):
    return {"n_samples_phase": n, "n_samples_window": 100, "window_sim_s": 10.0,
            "mean_tilt_deg": tilt, "max_tilt_deg": tilt + 0.2,
            "min_tilt_deg": max(tilt - 0.2, 0.0),
            "mean_lean_bearing_deg": bearing, "n_bearing_samples": 100,
            "drift_within_window_m": drift, "mean_up_m": 6.0}


GOOD_DRIVER = {"rows": 600, "published": 600, "publish_failed": 0,
               "mean_force_n": 1.909, "max_force_n": 1.95}
RESULT_LINE = ("WIND_DRIVER_RESULT label=gate0 ticks=600 published=600 "
               "publish_failed=0 pose_updates=600 mean_force_n=1.9090")


def _verdict(a, b, c, driver=None, line=RESULT_LINE):
    return build_verdict(CFG, {"A": a, "B": b, "C": c},
                         driver if driver is not None else dict(GOOD_DRIVER),
                         line)


def test_pass_when_all_four_criteria_hold():
    v = _verdict(_phase(0.2), _phase(PREDICTED_TILT), _phase(0.2))
    assert v["verdict"] == "PASS", v
    assert v["predicted_tilt_deg"] == pytest.approx(PREDICTED_TILT, abs=1e-6)
    assert all(c["pass"] for c in v["checks"])


def test_null_when_no_tilt_appears():
    """The outcome the probe exists to catch: the wrench is inert."""
    v = _verdict(_phase(0.2), _phase(0.25), _phase(0.2))
    assert v["verdict"] == "NULL", v


def test_partial_when_direction_is_right_but_magnitude_is_off_band():
    v = _verdict(_phase(0.2), _phase(PREDICTED_TILT * 2.0), _phase(0.2))
    assert v["verdict"] == "PARTIAL", v


def test_wrong_direction_is_a_hard_fail_even_with_a_perfect_magnitude():
    """A sign error in the driver's ENU conversion gives a textbook-correct
    tilt magnitude pointing the wrong way. It must not pass."""
    v = _verdict(_phase(0.2), _phase(PREDICTED_TILT, bearing=90.0), _phase(0.2))
    assert v["verdict"] == "FAIL", v
    assert [c["pass"] for c in v["checks"]] == [True, True, False, True]


def test_a_wrench_that_cannot_be_removed_fails():
    """Phase C still tilted = the wrench persisted past the driver's exit,
    which would contaminate every later flight in the same boot."""
    v = _verdict(_phase(0.2), _phase(PREDICTED_TILT), _phase(PREDICTED_TILT))
    assert v["verdict"] == "FAIL", v


def test_a_pre_tilted_baseline_fails():
    v = _verdict(_phase(3.0), _phase(PREDICTED_TILT), _phase(0.2))
    assert v["verdict"] == "FAIL", v


# --- VOID: no vacuous verdicts ---------------------------------------------
def test_void_when_the_driver_published_nothing():
    """Zero publishes is a VOID run, never a NULL finding: 'the driver ran and
    nothing happened' must not be reportable as evidence about Gazebo."""
    d = dict(GOOD_DRIVER, published=0)
    v = _verdict(_phase(0.2), _phase(0.2), _phase(0.2), driver=d)
    assert v["verdict"] == "VOID"
    assert any("ZERO wrenches" in r for r in v["reasons"])


def test_void_on_publish_failures():
    d = dict(GOOD_DRIVER, publish_failed=3)
    v = _verdict(_phase(0.2), _phase(PREDICTED_TILT), _phase(0.2), driver=d)
    assert v["verdict"] == "VOID"


def test_void_when_the_driver_never_saw_the_airframe():
    line = RESULT_LINE.replace("pose_updates=600", "pose_updates=0")
    v = _verdict(_phase(0.2), _phase(0.2), _phase(0.2), line=line)
    assert v["verdict"] == "VOID"
    assert any("pose_updates=0" in r for r in v["reasons"])


def test_void_when_the_driver_never_summarised():
    v = _verdict(_phase(0.2), _phase(PREDICTED_TILT), _phase(0.2), line=None)
    assert v["verdict"] == "VOID"


def test_void_on_a_thin_phase():
    v = _verdict(_phase(0.2, n=10), _phase(PREDICTED_TILT), _phase(0.2))
    assert v["verdict"] == "VOID"


def test_void_when_position_hold_was_lost():
    """Past 3 m of drift the tilt reflects a translation transient, not a force
    balance, so the whole derivation stops applying."""
    v = _verdict(_phase(0.2), _phase(PREDICTED_TILT, drift=9.0), _phase(0.2))
    assert v["verdict"] == "VOID"
    assert any("position hold was lost" in r for r in v["reasons"])


def test_void_when_the_commanded_force_is_not_the_assumed_field():
    """Guards the probe against silently scoring a DIFFERENT wind than the one
    the prediction was derived for."""
    d = dict(GOOD_DRIVER, mean_force_n=1.909 * 1.5)
    v = _verdict(_phase(0.2), _phase(PREDICTED_TILT), _phase(0.2), driver=d)
    assert v["verdict"] == "VOID"
    assert any("commanded force" in r for r in v["reasons"])


def test_void_beats_pass():
    """An otherwise-perfect run with a broken instrument is VOID, not PASS."""
    d = dict(GOOD_DRIVER, published=0)
    v = _verdict(_phase(0.2), _phase(PREDICTED_TILT), _phase(0.2), driver=d)
    assert v["verdict"] == "VOID"
    assert v["checks"] == []


def test_the_band_is_the_pre_registered_one():
    v = _verdict(_phase(0.2), _phase(PREDICTED_TILT), _phase(0.2))
    lo, hi = v["band_deg"]
    assert lo == pytest.approx(3.683, abs=0.01)
    assert hi == pytest.approx(6.840, abs=0.01)
