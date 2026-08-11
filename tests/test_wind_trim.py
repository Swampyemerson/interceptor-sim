#!/usr/bin/env python3
"""Behaviour tests for the PRE-FLIGHT ANEMOMETER WIND TRIM
(scripts/wind_trim.py) -- the only wind error-correction the open-loop dash
can honestly have.

Sim-free, millisecond-fast, no PX4/Gazebo. Three groups:

  GEOMETRY     the met-convention reading -> dash-frame headwind/crosswind
               resolution is a pure trig identity; pinned at the four cardinal
               relative directions plus an oblique case with a hand-derivation.
  FAIL-CLOSED  every place a MEASURED number could be silently replaced by a
               default or an extrapolation raises instead (no baked-in sag
               table; no extrapolation past the flown tiers; no enabled-but-
               empty trim; no silent staleness skip; no non-positive assumed
               speed).
  LIMITS       the things this unit deliberately does NOT do are pinned by
               test, not left in prose: heading trim is ALWAYS 0.0, a disabled
               trim is an exact identity that calls the lead solver exactly
               once with the untouched nominal speed, and the fixed-point
               solver refuses to return a half-converged aim.

The honesty-boundary tests live next door in tests/test_wind_trim_honesty.py;
the producer->consumer wire contract lives in tests/test_wind_trim_contract.py.

Run: `.venv/bin/python -m pytest tests/test_wind_trim.py -v`
"""
import math
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

from wind_trim import (  # noqa: E402
    PLACEHOLDER_ANEMOMETER_SPEC,
    AnemometerReading,
    AnemometerSpec,
    GroundTruthLeakError,
    SagTable,
    StaleReadingError,
    UnmeasuredSagError,
    WindTrim,
    WindTrimConvergenceError,
    WindTrimError,
    apply_wind_trim,
    solve_wind_trimmed_lead,
    wind_components,
)

# A SYNTHETIC table used only to exercise the arithmetic. It is NOT a
# measurement and must never be flown -- the real one comes from the Gate-2
# wind arms, which have not flown. Every test that uses it is testing the
# MECHANISM, never a wind number.
SYNTH_TABLE = SagTable(
    points=((0.0, 0.0), (4.5, 0.6), (8.0, 1.8)),
    provenance="SYNTHETIC unit-test table -- NOT MEASURED, never flown",
)


def _reading(speed=4.5, dir_from=0.0, t=0.0):
    return AnemometerReading(
        speed_mps=speed, dir_from_deg=dir_from,
        sigma_speed_mps=PLACEHOLDER_ANEMOMETER_SPEC.sigma_speed_at(speed),
        sigma_dir_deg=PLACEHOLDER_ANEMOMETER_SPEC.sigma_dir_deg,
        taken_at_sim_t=t, provenance="unit test")


def _trim(speed=4.5, dir_from=0.0, table=SYNTH_TABLE, t=0.0, max_age=None):
    return WindTrim(enabled=True, reading=_reading(speed, dir_from, t),
                    table=table, max_reading_age_s=max_age)


# =====================================================================
# GEOMETRY
# =====================================================================
@pytest.mark.parametrize("heading,dir_from,exp_head,exp_cross,what", [
    (0.0, 0.0, +4.5, 0.0, "dash N, wind from N -> pure headwind"),
    (0.0, 180.0, -4.5, 0.0, "dash N, wind from S -> pure tailwind"),
    (0.0, 90.0, 0.0, -4.5, "dash N, wind from E -> crosswind pushing LEFT (west)"),
    (0.0, 270.0, 0.0, +4.5, "dash N, wind from W -> crosswind pushing RIGHT (east)"),
    (90.0, 0.0, 0.0, +4.5, "dash E, wind from N -> crosswind pushing RIGHT (south)"),
    (90.0, 90.0, +4.5, 0.0, "dash E, wind from E -> pure headwind"),
    (270.0, 90.0, -4.5, 0.0, "dash W, wind from E -> pure tailwind"),
])
def test_wind_components_cardinal(heading, dir_from, exp_head, exp_cross, what):
    """The four cardinal relative geometries, in the project's compass frame
    (heading = atan2(east, north), 0 = north; wind direction = met convention,
    the bearing the wind blows FROM). Headwind > 0 opposes the dash; crosswind
    > 0 pushes to the RIGHT of the heading."""
    head, tail, cross = wind_components(heading, _reading(4.5, dir_from))
    assert head == pytest.approx(exp_head, abs=1e-9), what
    assert cross == pytest.approx(exp_cross, abs=1e-9), what
    assert tail == pytest.approx(-head, abs=1e-12), "tailwind is exactly -headwind"


def test_wind_components_oblique_matches_hand_derivation():
    """45 deg off the nose: headwind = V cos(d - h), crosswind = -V sin(d - h).
    V = 6.0, d = 45, h = 0  ->  head = 6 cos45 = 4.2426, cross = -6 sin45 =
    -4.2426 (a wind from the NE pushes you to the LEFT/west when you fly
    north). Hand-derived, not fitted."""
    head, _tail, cross = wind_components(0.0, _reading(6.0, 45.0))
    assert head == pytest.approx(6.0 * math.cos(math.radians(45.0)), abs=1e-12)
    assert cross == pytest.approx(-6.0 * math.sin(math.radians(45.0)), abs=1e-12)


def test_wind_components_magnitude_is_conserved():
    """head^2 + cross^2 == V^2 for every geometry -- the resolution is a
    rotation, so it cannot invent or lose wind energy."""
    for heading in range(0, 360, 17):
        for dir_from in range(0, 360, 23):
            head, _t, cross = wind_components(float(heading), _reading(7.3, float(dir_from)))
            assert math.hypot(head, cross) == pytest.approx(7.3, abs=1e-9)


# =====================================================================
# SAG TABLE -- measured-only, no extrapolation
# =====================================================================
def test_sag_table_interpolates_between_measured_points():
    # midway between the (0,0) and (4.5,0.6) measured points
    assert SYNTH_TABLE.sag_mps(2.25) == pytest.approx(0.30, abs=1e-12)
    # exactly on a measured point
    assert SYNTH_TABLE.sag_mps(4.5) == pytest.approx(0.60, abs=1e-12)
    # midway on the upper segment: 0.6 + 0.5*(1.8-0.6) = 1.2
    assert SYNTH_TABLE.sag_mps(6.25) == pytest.approx(1.20, abs=1e-12)


def test_sag_table_refuses_to_extrapolate():
    """A headwind past the last FLOWN tier has no measured sag. Guessing it is
    the 'threshold validated at one operating point' trap; fail closed."""
    with pytest.raises(UnmeasuredSagError) as e:
        SYNTH_TABLE.sag_mps(9.0)
    assert "outside the MEASURED sag domain" in str(e.value)
    with pytest.raises(UnmeasuredSagError):
        SYNTH_TABLE.sag_mps(-0.1)          # a tailwind was never measured either
    with pytest.raises(UnmeasuredSagError):
        SYNTH_TABLE.sag_mps(float("nan"))


def test_sag_table_dead_band_absorbs_float_dust_only():
    """A near-90 deg geometry gives headwind ~1e-16 from cos() rounding --
    physically zero, numerically just outside a domain starting at 0.0. That
    dust is snapped to the boundary; anything a real instrument could ever
    resolve (>= 1e-9 m/s past the edge, i.e. 9 orders below its 0.3 m/s sigma)
    still raises."""
    assert SYNTH_TABLE.sag_mps(-1e-16) == 0.0
    assert SYNTH_TABLE.sag_mps(8.0 + 1e-16) == pytest.approx(1.8)
    with pytest.raises(UnmeasuredSagError):
        SYNTH_TABLE.sag_mps(-1e-6)
    with pytest.raises(UnmeasuredSagError):
        SYNTH_TABLE.sag_mps(8.0 + 1e-6)


def test_sag_table_requires_the_zero_headwind_reference():
    """The Gate-1 drag-only control arm IS the zero-headwind reference, so the
    table must contain (0.0, 0.0). Without it the sag is measured against an
    unstated baseline -- the unrecorded-assumption failure mode."""
    with pytest.raises(WindTrimError) as e:
        SagTable(points=((2.0, 0.1), (8.0, 1.8)), provenance="x")
    assert "headwind 0.0" in str(e.value)
    with pytest.raises(WindTrimError) as e:
        SagTable(points=((0.0, 0.25), (8.0, 1.8)), provenance="x")
    assert "must have sag 0.0" in str(e.value)


def test_sag_table_rejects_degenerate_or_unsourced_tables():
    with pytest.raises(WindTrimError):                       # one point is not a curve
        SagTable(points=((0.0, 0.0),), provenance="x")
    with pytest.raises(WindTrimError):                       # unsorted / duplicated
        SagTable(points=((0.0, 0.0), (4.5, 0.6), (4.5, 0.9)), provenance="x")
    with pytest.raises(WindTrimError):                       # numbers trace to a run
        SagTable(points=((0.0, 0.0), (8.0, 1.8)), provenance="  ")


def test_no_measured_sag_table_ships_with_the_repo():
    """*** FLIP-LATER (pins an ABSENCE, deliberately) ***

    There is no measured sag table today: the Gate-2 wind arms have not flown.
    So (a) the module must expose no default/constant table, and (b) enabling
    the trim without one must RAISE. When a real table lands (from flown arms,
    with a provenance naming them), this test is updated to assert the table
    file exists AND carries a run-traceable provenance -- it is not deleted.
    """
    import wind_trim as wt
    defaults = [n for n in dir(wt)
                if isinstance(getattr(wt, n), SagTable)]
    assert defaults == [], (
        f"scripts/wind_trim.py exposes a module-level SagTable {defaults} -- there is "
        "no measured sag data yet, so any built-in table is an invented default for a "
        "MEASURED quantity (CLAUDE.md fail-closed rule).")
    with pytest.raises(UnmeasuredSagError) as e:
        apply_wind_trim(16.0, 0.0, WindTrim(enabled=True, reading=_reading(), table=None))
    assert "no default sag" in str(e.value)


# =====================================================================
# THE CORRECTION ITSELF
# =====================================================================
def test_disabled_trim_is_an_exact_identity():
    """The m4 flags default to 0.0 -> WindTrim.disabled() -> the lead solve
    sees the nominal speed unchanged, so a run that does not ask for wind trim
    is byte-identical to today's flights."""
    r = apply_wind_trim(16.0, 37.0, WindTrim.disabled())
    assert r.enabled is False and r.is_noop
    assert r.corrected_speed_mps == 16.0        # exact, not approx
    assert (r.headwind_mps, r.crosswind_mps, r.sag_mps) == (0.0, 0.0, 0.0)
    assert "OFF" in r.log_line


def test_from_cli_values_zero_is_the_disabled_sentinel():
    assert WindTrim.from_cli_values(0.0, 123.0, SYNTH_TABLE).enabled is False
    t = WindTrim.from_cli_values(4.5, 370.0, SYNTH_TABLE)
    assert t.enabled is True
    assert t.reading.speed_mps == 4.5
    assert t.reading.dir_from_deg == pytest.approx(10.0)      # wrapped into [0,360)
    # the instrument spec's sigma rides along with the reading so a log can
    # never lose it: max(0.3, 5% of 4.5) = 0.3
    assert t.reading.sigma_speed_mps == pytest.approx(0.3)
    assert "PLACEHOLDER" in t.reading.provenance


def test_headwind_slows_the_assumed_dash_speed():
    """Dash north into a 4.5 m/s wind from the north: sag 0.6 -> the lead solve
    should assume 15.4 m/s, not 16."""
    r = apply_wind_trim(16.0, 0.0, _trim(4.5, 0.0))
    assert r.headwind_mps == pytest.approx(4.5, abs=1e-9)
    assert r.sag_mps == pytest.approx(0.6, abs=1e-12)
    assert r.corrected_speed_mps == pytest.approx(15.4, abs=1e-12)
    assert not r.is_noop
    assert "PRE-FLIGHT" in r.log_line and "no gt" in r.log_line


def test_pure_crosswind_makes_no_speed_correction():
    """The 1-D table is keyed on the HEADWIND component, so a pure crosswind
    leaves the assumed speed alone -- and the crosswind is still reported, so
    the arms can test whether that 1-D approximation holds."""
    r = apply_wind_trim(16.0, 90.0, _trim(4.5, 0.0))
    assert r.headwind_mps == pytest.approx(0.0, abs=1e-9)
    assert r.corrected_speed_mps == pytest.approx(16.0, abs=1e-12)
    assert r.crosswind_mps == pytest.approx(+4.5, abs=1e-9)


def test_heading_trim_is_always_zero():
    """PINS A DELIBERATE LIMITATION. Crosswind track-holding is PX4's velocity
    loop (correction layer 1). A pre-flight heading bias from a noisy
    anemometer would fight the controller already doing it, so this unit emits
    NO heading correction -- ever, in any geometry. If a future finding says
    otherwise it arrives as a new, separately pre-registered parameter, and
    this test is the thing that must be consciously changed."""
    for heading in range(0, 360, 15):
        for dir_from in range(0, 360, 15):
            spd = 4.5
            head = spd * math.cos(math.radians(dir_from - heading))
            if head < 0.0 or head > 8.0:
                continue                     # outside the synthetic table's domain
            r = apply_wind_trim(16.0, float(heading), _trim(spd, float(dir_from)))
            assert r.heading_trim_deg == 0.0


def test_correction_refuses_a_non_positive_assumed_speed():
    """A sag that eats the whole dash speed is not a correction, it is a
    no-launch condition."""
    brutal = SagTable(points=((0.0, 0.0), (8.0, 12.0)),
                      provenance="SYNTHETIC saturation case -- NOT MEASURED")
    with pytest.raises(WindTrimError) as e:
        apply_wind_trim(10.0, 0.0, _trim(8.0, 0.0, table=brutal))
    assert "not an intercept solution" in str(e.value)


def test_enabled_but_empty_trim_raises_instead_of_logging_a_lie():
    """The inert-flag class of bug: 'wind trim ON' in the log while nothing
    changed. Refuse it."""
    with pytest.raises(WindTrimError) as e:
        apply_wind_trim(16.0, 0.0, WindTrim(enabled=True, reading=None, table=SYNTH_TABLE))
    assert "nothing to correct FROM" in str(e.value)


def test_nominal_speed_must_be_positive():
    with pytest.raises(WindTrimError):
        apply_wind_trim(0.0, 0.0, _trim())


# =====================================================================
# STALENESS -- sim seconds, and never silently skipped
# =====================================================================
def test_staleness_check_uses_sim_time_and_fails_closed():
    trim = _trim(t=100.0, max_age=60.0)
    ok = apply_wind_trim(16.0, 0.0, trim, now_sim_t=140.0)      # 40 s old
    assert ok.enabled and ok.corrected_speed_mps == pytest.approx(15.4)
    with pytest.raises(StaleReadingError) as e:
        apply_wind_trim(16.0, 0.0, trim, now_sim_t=200.0)       # 100 s old
    assert "older than the configured" in str(e.value)


def test_staleness_check_without_a_clock_raises_rather_than_skipping():
    """Asking for a check and silently not performing it is the silent-failure
    class this project keeps root-causing to measurement-layer code."""
    with pytest.raises(StaleReadingError) as e:
        apply_wind_trim(16.0, 0.0, _trim(t=0.0, max_age=60.0), now_sim_t=None)
    assert "refusing to silently skip" in str(e.value)


def test_undated_reading_cannot_pass_a_staleness_check():
    trim = WindTrim(enabled=True, reading=_reading(t=None), table=SYNTH_TABLE,
                    max_reading_age_s=60.0)
    with pytest.raises(StaleReadingError):
        apply_wind_trim(16.0, 0.0, trim, now_sim_t=10.0)


def test_reading_from_the_future_is_a_clock_basis_error():
    with pytest.raises(StaleReadingError) as e:
        apply_wind_trim(16.0, 0.0, _trim(t=500.0, max_age=60.0), now_sim_t=100.0)
    assert "FUTURE" in str(e.value)


# =====================================================================
# THE AIM/SPEED FIXED POINT
# =====================================================================
class _CountingSolver:
    """Stand-in for flight.guidance.collision_lead_heading: a slower assumed
    dash speed means a longer time of flight, so the aim leads FURTHER ahead
    of a crossing target. Injected (not imported) so this unit stays
    stdlib-only -- see the honesty test."""

    def __init__(self, k=2.0, base=0.0):
        self.calls = []
        self.k = k
        self.base = base

    def __call__(self, target_pos, target_vel, dash_speed):
        self.calls.append((tuple(target_pos), tuple(target_vel), dash_speed))
        # heading drifts with 1/speed: more lead when we expect to be slower
        return (self.base + self.k * (16.0 / dash_speed - 1.0) * 10.0,
                100.0 / dash_speed)


def test_disabled_trim_calls_the_lead_solver_exactly_once_unchanged():
    solver = _CountingSolver()
    h, t_lead, res = solve_wind_trimmed_lead(
        solver, (30.0, 40.0), (0.0, -9.0), 16.0, WindTrim.disabled())
    assert len(solver.calls) == 1
    assert solver.calls[0][2] == 16.0        # nominal speed, untouched
    assert h == pytest.approx(0.0) and t_lead == pytest.approx(6.25)
    assert res.is_noop


def test_fixed_point_converges_and_is_deterministic():
    solver_a = _CountingSolver()
    h_a, t_a, res_a = solve_wind_trimmed_lead(
        solver_a, (30.0, 40.0), (0.0, -9.0), 16.0, _trim(4.5, 0.0))
    solver_b = _CountingSolver()
    h_b, t_b, res_b = solve_wind_trimmed_lead(
        solver_b, (30.0, 40.0), (0.0, -9.0), 16.0, _trim(4.5, 0.0))
    assert (h_a, t_a) == (h_b, t_b), "same launch inputs must give the same aim"
    assert res_a.corrected_speed_mps == res_b.corrected_speed_mps
    assert len(solver_a.calls) == len(solver_b.calls)
    assert res_a.corrected_speed_mps < 16.0, "a headwind must reduce the assumed speed"
    assert h_a != 0.0, "the trimmed aim must actually differ from the untrimmed one"


def test_fixed_point_works_against_the_REAL_lead_solver():
    """Signature-drift guard + a physical sanity check, using the actual
    pre-flight aim function the flight code uses
    (flight.guidance.collision_lead_heading). The module under test never
    IMPORTS it -- the solver is injected -- so this test is what pins the two
    to the same calling convention (target_pos, target_vel, dash_speed) ->
    (heading_deg, t_lead).

    Physics: a headwind means the interceptor will be SLOWER than nominal, so
    the honest lead solve must aim FURTHER ahead of a crossing target and
    expect a LONGER flight. Target crosses west-to-east at 9 m/s, 40 m north.
    """
    sys.path.insert(0, REPO_ROOT)
    from flight.guidance import collision_lead_heading

    target_pos, target_vel = (0.0, 40.0), (9.0, 0.0)     # east-bound crosser
    h_plain, t_plain = collision_lead_heading(target_pos, target_vel, 16.0)
    h_trim, t_trim, res = solve_wind_trimmed_lead(
        collision_lead_heading, target_pos, target_vel, 16.0, _trim(4.5, 0.0))

    # The fixed point lands on a heading EAST of north (it is leading a
    # crossing target), so the 4.5 m/s wind from the north is only partly a
    # headwind -- the result must be self-consistent at the FINAL heading, not
    # at the initial one. That self-consistency IS what the fixed point buys.
    assert res.headwind_mps == pytest.approx(
        4.5 * math.cos(math.radians(h_trim)), abs=1e-9)
    assert res.corrected_speed_mps == pytest.approx(
        16.0 - SYNTH_TABLE.sag_mps(res.headwind_mps), abs=1e-12)
    assert 15.0 < res.corrected_speed_mps < 16.0
    assert h_trim > h_plain, "a slower expected dash must lead FURTHER east"
    assert t_trim > t_plain, "a slower expected dash must expect a longer flight"
    # and the shift is small and physical, not a sign error: single-digit deg
    assert 0.0 < (h_trim - h_plain) < 5.0


def test_fixed_point_refuses_to_return_a_half_converged_aim():
    """A pathological solver that oscillates must produce an EXCEPTION, not a
    plausible-looking heading. Half a converged aim is a silent wrong answer."""
    class _Oscillator:
        def __init__(self):
            self.n = 0

        def __call__(self, target_pos, target_vel, dash_speed):
            self.n += 1
            return (0.0 if self.n % 2 else 90.0, 5.0)

    with pytest.raises(WindTrimConvergenceError) as e:
        solve_wind_trimmed_lead(_Oscillator(), (30.0, 40.0), (0.0, -9.0), 16.0,
                                _trim(4.5, 0.0))
    assert "did not converge" in str(e.value)


# =====================================================================
# VALUE-OBJECT VALIDATION
# =====================================================================
def test_spec_and_reading_require_provenance():
    with pytest.raises(WindTrimError):
        AnemometerSpec(0.3, 0.05, 15.0, 0.0, 0.0, "   ")
    with pytest.raises(WindTrimError):
        AnemometerReading(1.0, 0.0, 0.3, 15.0, 0.0, "")


def test_spec_sigma_is_the_whichever_is_greater_form():
    s = PLACEHOLDER_ANEMOMETER_SPEC
    assert s.sigma_speed_at(2.0) == pytest.approx(0.3)      # floor dominates
    assert s.sigma_speed_at(12.0) == pytest.approx(0.6)     # 5% dominates


def test_negative_speed_reading_is_rejected():
    with pytest.raises(WindTrimError):
        AnemometerReading(-1.0, 0.0, 0.3, 15.0, 0.0, "unit test")


def test_reading_mapping_round_trip_is_exact():
    r = _reading(4.5, 271.3, 12.0)
    back = AnemometerReading.from_mapping(r.to_mapping())
    assert back == r


def test_reading_rejects_unknown_keys():
    """An unreviewed input is how a truth value gets in."""
    data = _reading().to_mapping()
    data["gust_peak_mps"] = 7.0
    with pytest.raises(GroundTruthLeakError) as e:
        AnemometerReading.from_mapping(data)
    assert "unknown key" in str(e.value)


def test_reading_rejects_a_schema_mismatch():
    data = _reading().to_mapping()
    data["schema"] = "anemometer_reading/999"
    with pytest.raises(WindTrimError) as e:
        AnemometerReading.from_mapping(data)
    assert "out of step" in str(e.value)


def test_reading_rejects_a_missing_sigma():
    data = _reading().to_mapping()
    del data["sigma_speed_mps"]
    with pytest.raises(WindTrimError) as e:
        AnemometerReading.from_mapping(data)
    assert "no default for a MEASURED quantity" in str(e.value)
