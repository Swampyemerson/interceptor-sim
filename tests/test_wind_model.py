#!/usr/bin/env python3
"""OFFLINE unit tests for scripts/wind_model.py -- the wind + gust disturbance
model built as the sim-side mitigation for the project-state assumption
`zero-wind-environment`. NO sim, NO Gazebo, NO PX4: pure math and seeded RNG,
runs in ~2 s, collected by scripts/run_tests.sh stage 1 on every commit.

The module is an INSTRUMENT for a measured A/B (the wind arms), so these tests
are written to the "instruments are evidence" rule: a defect in here would not
be visible to a paired control, because BOTH arms would share it. The four
things most able to hide a defect are pinned hardest:

  (a) THE SIGN CONVENTION. Meteorological direction ("blows FROM") vs a flow
      vector ("blows TOWARD") is a 180 deg error that would look like a
      perfectly plausible wind arm. All four cardinals are pinned.
  (b) THE DERIVED NUMBERS. sigma, L_u, tau, a_drag and the hover tilt are
      pinned to independently-computed literals with the derivation in the
      comment, so a formula typo fails rather than producing a believable
      wrong number.
  (c) PAIRING. Paired seeds are this project's evidence standard; if the gust
      realisation depended on the caller's cadence (or on RTF), the "paired"
      arms would not be paired at all. Includes a regression pin for a REAL
      bug this suite caught: floor(2.15/0.05) == 42, not 43.
  (d) FAIL-CLOSED. Every measured input must raise rather than default, and
      the error must say how to measure the thing (builder mandate: every
      realism knob maps to a bench-measurable quantity).

Plus a static honesty check: the module must not import Gazebo/PX4/MAVSDK and
must not read any gt_* value. It generates world physics; it is on the
scoring/logging side of the boundary and must stay there.
"""

import ast
import math
import os
import statistics
import sys

import pytest

SCRIPTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import wind_model as wm  # noqa: E402
from wind_model import (  # noqa: E402
    DragParams,
    MissingMeasurementError,
    WindConditions,
    WindField,
    dryden_length_scale_m,
    dryden_sigma_mps,
    dryden_tau_s,
    sigma_from_gust_factor,
    tier,
)

H = 6.0          # the design brief's assumed engagement height, m AGL
STEP = 0.05      # 20 Hz driver tick, SIM seconds
PROV = "unit test (not a field measurement)"


def _cond(**kw):
    base = dict(mean_mps=4.5, dir_from_deg=270.0, height_m=H, sigma_mps=0.8686,
                tau_s=9.588, label="t", provenance=PROV)
    base.update(kw)
    return WindConditions(**base)


# ============================================================ (a) sign convention
@pytest.mark.parametrize("dir_from_deg,expect_ne", [
    (0.0, (-1.0, 0.0)),      # northerly: air moves SOUTH
    (90.0, (0.0, -1.0)),     # easterly:  air moves WEST
    (180.0, (1.0, 0.0)),     # southerly: air moves NORTH
    (270.0, (0.0, 1.0)),     # westerly:  air moves EAST
])
def test_meteorological_direction_is_where_it_comes_from(dir_from_deg, expect_ne):
    """`dir_from_deg` is what a wind sock + compass give: the direction the
    wind blows FROM. The stored flow vector must point the OTHER way. A 180 deg
    error here would produce a completely plausible-looking wind arm that
    pushed the dash the wrong way, so all four cardinals are pinned."""
    c = _cond(mean_mps=5.0, dir_from_deg=dir_from_deg, sigma_mps=0.0, tau_s=1.0)
    n, e = c.steady_ne
    assert n == pytest.approx(5.0 * expect_ne[0], abs=1e-9)
    assert e == pytest.approx(5.0 * expect_ne[1], abs=1e-9)
    assert c.flow_heading_deg == pytest.approx((dir_from_deg + 180.0) % 360.0)


def test_direction_wraps_and_is_normalised():
    assert _cond(dir_from_deg=-90.0).dir_from_deg == pytest.approx(270.0)
    assert _cond(dir_from_deg=450.0).dir_from_deg == pytest.approx(90.0)


@pytest.mark.parametrize("dir_from_deg,along_ne,cross_ne", [
    # (flow heading) along-flow unit,        90 deg RIGHT of it
    (180.0, (1.0, 0.0), (0.0, 1.0)),                    # flow N   -> right is E
    (0.0, (-1.0, 0.0), (0.0, -1.0)),                    # flow S   -> right is W
    (270.0, (0.0, 1.0), (-1.0, 0.0)),                   # flow E   -> right is S
    (225.0, (0.70711, 0.70711), (-0.70711, 0.70711)),   # flow NE  -> right is SE
])
def test_gust_v_is_ninety_degrees_right_of_the_flow(dir_from_deg, along_ne, cross_ne):
    """The cross-gust axis must be perpendicular to the mean flow and to the
    RIGHT of it. A left/right swap is invisible at some cardinal headings
    (mutation testing caught exactly that gap: at a due-north flow, sin(psi)=0
    hides the sign), so a NON-CARDINAL heading is included on purpose."""
    c = _cond(mean_mps=0.0, dir_from_deg=dir_from_deg, sigma_mps=0.0, tau_s=1.0)
    wf = WindField(c, seed=1, sim_step_s=STEP)
    wf._gu, wf._gv = 2.0, 0.0                       # pure along-flow gust
    s = wf.sample(0.0)
    assert s.wind_n == pytest.approx(2.0 * along_ne[0], abs=1e-4)
    assert s.wind_e == pytest.approx(2.0 * along_ne[1], abs=1e-4)
    wf._gu, wf._gv = 0.0, 3.0                       # pure cross-flow gust
    s = wf.sample(0.0)
    assert s.wind_n == pytest.approx(3.0 * cross_ne[0], abs=1e-4)
    assert s.wind_e == pytest.approx(3.0 * cross_ne[1], abs=1e-4)


def test_vertical_wind_is_declared_zero_not_absent():
    """w_z = 0 is a DECLARED scope limit. The field exists and is 0 so a
    consumer cannot mistake 'not modelled' for 'not there'."""
    s = WindField(_cond(), seed=1, sim_step_s=STEP).sample(1.0)
    assert s.wind_d == 0.0 and "wind_d" in s.to_dict()
    f = WindField(_cond(), seed=1, sim_step_s=STEP).force(s, 0.0, 0.0,
                                                          DragParams.px4_x500_mono_cam_sitl())
    assert f.f_d == 0.0 and "f_d" in f.to_dict()


# ========================================================= (b) derived numbers
def test_dryden_sigma_and_length_trace_to_the_mil_form():
    """MIL-F-8785C low altitude, h = 6 m = 19.68504 ft:
         denom = 0.177 + 0.000823*19.68504 = 0.19320079
         sigma = 0.1*V/denom^0.4 = 0.193017 * V
         L_u   = 19.68504/denom^1.2 ft = 141.556 ft = 43.146 m
    Literals computed independently of the module (see the docstring's
    derivation), so a formula typo fails instead of looking plausible."""
    assert dryden_sigma_mps(1.0, H) == pytest.approx(0.193017, abs=1e-5)
    assert dryden_sigma_mps(4.5, H) == pytest.approx(0.86858, abs=1e-4)
    assert dryden_length_scale_m(H) == pytest.approx(43.1460, abs=1e-3)
    assert dryden_tau_s(4.5, H) == pytest.approx(43.1460 / 4.5, rel=1e-6)


def test_gust_factor_and_dryden_sigma_agree_with_the_open_terrain_reference():
    """Cross-check between the two routes to sigma. The Dryden coefficient
    0.193 at h=6 m implies a gust factor of 1 + 2*0.193 = 1.386 at 2-sigma
    peaks, which matches the WMO/Wieringa open-terrain reference GF ~ 1.4.
    Two independent derivations landing on the same number is the check."""
    c = tier("EXPECTED", height_m=H, dir_from_deg=270.0)
    assert c.gust_factor(n_sigma=2.0) == pytest.approx(1.386, abs=0.005)
    # ...and the inverse conversion round-trips.
    assert sigma_from_gust_factor(4.5, c.gust_factor(2.0), 2.0) == \
        pytest.approx(c.sigma_mps, rel=1e-9)


def test_height_changes_tau_by_an_order_of_magnitude():
    """A LOAD-BEARING sensitivity, pinned so nobody quietly changes the height.
    The sim's own altitude reference is ALT_REF_M = 0.5 m (m4_intercept.py),
    not the 6 m the design brief assumed. L_u falls 43.15 m -> 3.96 m, so at
    EXPECTED (4.5 m/s) the gust correlation time falls 9.59 s -> 0.88 s. That
    is the difference between a gust acting like a constant bias across a ~2 s
    engagement and a gust that fully decorrelates inside it."""
    assert dryden_length_scale_m(0.5, allow_below_10ft=True) == \
        pytest.approx(3.9577, abs=1e-3)
    assert dryden_tau_s(4.5, H) == pytest.approx(9.588, abs=1e-2)
    assert dryden_tau_s(4.5, 0.5, allow_below_10ft=True) == \
        pytest.approx(0.8795, abs=1e-3)


@pytest.mark.parametrize("v_rel,a_expect,tilt_expect", [
    # a = 0.15*v + 1.225*v^2/200 ; tilt = atan(a/9.80665), all hand-computed:
    (1.5, 0.238781, 1.3948),    # BEST tier hover
    (4.5, 0.799031, 4.6581),    # EXPECTED tier hover
    (8.0, 1.592000, 9.2209),    # WORST tier hover
    (24.0, 7.128000, 36.0117),  # WORST headwind against the 16 m/s dash
])
def test_px4_drag_law_and_hover_tilt(v_rel, a_expect, tilt_expect):
    """The drag law and the ROTATION half of the assumption. Note the design
    brief quoted '~4.0 deg at EXPECTED 4.5 m/s'; the law gives 4.66 deg -- the
    code is the derivation of record and the brief's figure is the typo."""
    d = DragParams.px4_x500_mono_cam_sitl()
    assert d.drag_accel_m_s2(v_rel) == pytest.approx(a_expect, abs=1e-6)
    assert d.hover_tilt_deg(v_rel) == pytest.approx(tilt_expect, abs=1e-3)
    assert d.drag_force_n(v_rel) == pytest.approx(2.114308 * a_expect, abs=1e-5)


def test_worst_headwind_eats_half_the_dash_authority():
    """The prediction the WORST-tier arm exists to test: at 16 m/s dash into an
    8 m/s headwind, |dv| = 24 m/s and a_drag = 7.13 m/s^2 -- more than half of
    PX4's MPC_ACC_HOR_MAX = 12 m/s^2 under the FPV profile. This is a
    DERIVATION, not a result; the sag itself has to come from a flown arm."""
    d = DragParams.px4_x500_mono_cam_sitl()
    assert d.drag_accel_m_s2(24.0) / 12.0 > 0.5


def test_sitl_drag_params_trace_to_px4_and_the_sdf():
    """Numbers trace to a run or a derivation: EKF2_MCOEF/EKF2_BCOEF defaults
    from src/modules/ekf2/params_drag.yaml, mass from the model SDFs
    (2.0 + 4*0.016076923 + 0.050)."""
    d = DragParams.px4_x500_mono_cam_sitl()
    assert d.mcoef_per_s == 0.15 and d.bcoef_kg_m2 == 100.0
    assert d.mass_kg == pytest.approx(2.114308, abs=1e-6)
    assert d.rho_kg_m3 == 1.225
    # ...and the provenance says out loud that it is NOT the real airframe.
    assert "NOT measured on the real airframe" in d.provenance


def test_bcoef_zero_is_px4s_bluff_drag_off_sentinel():
    d = DragParams(mass_kg=2.0, mcoef_per_s=0.15, bcoef_kg_m2=0.0,
                   rho_kg_m3=1.225, provenance=PROV)
    assert d.drag_accel_m_s2(10.0) == pytest.approx(1.5)   # linear term only


# ================================================================ gust process
def test_gust_is_stationary_with_the_right_std_and_no_warmup():
    """The OU process must be stationary from the FIRST sample: a warm-up
    transient would sit entirely inside a ~2 s engagement and every flight
    would start in artificially calm air. Checked two ways -- an ensemble of
    first samples across seeds, and a long single realisation."""
    c = tier("EXPECTED", height_m=H, dir_from_deg=270.0)
    first = [WindField(c, seed=s, sim_step_s=STEP).sample(0.0).gust_u
             for s in range(4000)]
    assert statistics.pstdev(first) == pytest.approx(c.sigma_mps, rel=0.05)
    assert abs(statistics.fmean(first)) < 0.1 * c.sigma_mps

    wf = WindField(c, seed=99, sim_step_s=STEP)
    g = [wf.sample(i * STEP).gust_u for i in range(100000)]
    assert statistics.pstdev(g) == pytest.approx(c.sigma_mps, rel=0.06)
    assert abs(statistics.fmean(g)) < 0.15 * c.sigma_mps


def test_lag1_autocorrelation_matches_exp_minus_dt_over_tau():
    """THE gust-timescale property: a first-order Gauss-Markov process has
    lag-1 autocorrelation exp(-dt/tau). This is what makes a gust a GUST
    (correlated) rather than white noise, and it is what the anemometer's
    peak-to-peak period would measure in the field."""
    c = _cond(sigma_mps=1.0, tau_s=0.5)
    wf = WindField(c, seed=5, sim_step_s=0.1)
    g = [wf.sample(i * 0.1).gust_u for i in range(200000)]
    m = statistics.fmean(g)
    num = sum((g[i] - m) * (g[i + 1] - m) for i in range(len(g) - 1))
    den = sum((x - m) ** 2 for x in g)
    assert num / den == pytest.approx(math.exp(-0.1 / 0.5), abs=0.01)


def test_gust_components_are_independent():
    """u and v are drawn from separate noise; a shared draw would make every
    gust a 45 deg diagonal."""
    c = _cond(sigma_mps=1.0, tau_s=0.5)
    wf = WindField(c, seed=13, sim_step_s=0.1)
    s = [wf.sample(i * 0.1) for i in range(50000)]
    u = [x.gust_u for x in s]
    v = [x.gust_v for x in s]
    mu, mv = statistics.fmean(u), statistics.fmean(v)
    cov = sum((a - mu) * (b - mv) for a, b in zip(u, v)) / len(u)
    assert abs(cov / (statistics.pstdev(u) * statistics.pstdev(v))) < 0.02


def test_zero_sigma_means_no_gust_at_all():
    c = _cond(sigma_mps=0.0, tau_s=1.0)
    wf = WindField(c, seed=1, sim_step_s=STEP)
    for i in range(200):
        s = wf.sample(i * STEP)
        assert s.gust_u == 0.0 and s.gust_v == 0.0
        assert s.speed_mps == pytest.approx(c.mean_mps)


# ==================================================================== (c) pairing
def test_same_seed_gives_an_identical_realisation():
    c = tier("WORST", height_m=H, dir_from_deg=15.0)
    a = [WindField(c, seed=42, sim_step_s=STEP).sample(t * STEP).wind_n
         for t in range(200)]
    b = [WindField(c, seed=42, sim_step_s=STEP).sample(t * STEP).wind_n
         for t in range(200)]
    assert a == b


def test_different_seeds_give_different_realisations():
    c = tier("WORST", height_m=H, dir_from_deg=15.0)
    a = [WindField(c, seed=1, sim_step_s=STEP).sample(t * STEP).wind_n
         for t in range(200)]
    b = [WindField(c, seed=2, sim_step_s=STEP).sample(t * STEP).wind_n
         for t in range(200)]
    assert a != b


def test_realisation_is_invariant_to_caller_cadence():
    """THE PAIRING GUARANTEE. Two arms tick at whatever rate the RTF allows;
    the gust must be a function of (seed, sim_step, grid index) ONLY. Sampling
    the same grid points at jittered sim times, and sampling extra times in
    between, must not change what the grid holds."""
    c = tier("EXPECTED", height_m=H, dir_from_deg=270.0)
    ref = [WindField(c, seed=8, sim_step_s=STEP).sample(t * STEP).wind_n
           for t in range(400)]
    jit = WindField(c, seed=8, sim_step_s=STEP)
    assert [jit.sample(t * STEP + 0.049).wind_n for t in range(400)] == ref
    dense = WindField(c, seed=8, sim_step_s=STEP)
    got = []
    for t in range(400):                     # 5 reads per grid step
        for k in range(5):
            v = dense.sample(t * STEP + k * 0.009).wind_n
        got.append(v)
    assert got == ref


def test_grid_index_is_not_fooled_by_binary_float_error():
    """REGRESSION PIN (bug found by this suite, 2026-08-10): a plain
    floor(t/step) puts t = 2.15 s at grid 42, because 2.15/0.05 evaluates to
    42.99999999999999. The same sim instant would then land on a different
    gust depending on how the caller phrased the time -- a measurement-layer
    defect invisible to a paired control, because both arms share it."""
    c = _cond()
    wf = WindField(c, seed=3, sim_step_s=0.05)
    assert wf._grid_index(2.15) == 43
    assert wf._grid_index(0.05) == 1
    assert wf._grid_index(1.0) == 20
    assert wf._grid_index(0.0) == 0
    assert wf._grid_index(0.049) == 0        # genuinely below the boundary
    assert wf._grid_index(43 * 0.05 - 1e-3) == 42


def test_reset_replays_the_same_realisation():
    c = tier("EXPECTED", height_m=H, dir_from_deg=270.0)
    wf = WindField(c, seed=4, sim_step_s=STEP)
    a = [wf.sample(t * STEP).wind_e for t in range(100)]
    wf.reset()
    assert [wf.sample(t * STEP).wind_e for t in range(100)] == a


# ======================================================================= force
def test_force_is_zero_when_the_air_and_the_vehicle_move_together():
    """No RELATIVE air velocity -> no force. A model that used the wind vector
    instead of the relative one would push a vehicle that is already drifting
    with the air."""
    c = _cond(sigma_mps=0.0, tau_s=1.0)
    wf = WindField(c, seed=1, sim_step_s=STEP)
    s = wf.sample(1.0)
    f = wf.force(s, s.wind_n, s.wind_e, DragParams.px4_x500_mono_cam_sitl())
    assert f.v_rel_mps == pytest.approx(0.0, abs=1e-12)
    assert f.f_n == 0.0 and f.f_e == 0.0 and f.magnitude_n == 0.0


def test_force_opposes_motion_in_still_air():
    """The Gate-1 drag-only control's whole mechanism: with wind identically
    zero, a moving vehicle still feels drag, opposite its velocity. Comparing a
    wind arm against a NO-DRIVER baseline would conflate this with wind."""
    c = WindConditions.still_air(height_m=H, provenance=PROV)
    wf = WindField(c, seed=1, sim_step_s=STEP)
    d = DragParams.px4_x500_mono_cam_sitl()
    s = wf.sample(2.0)
    assert (s.wind_n, s.wind_e, s.speed_mps) == (0.0, 0.0, 0.0)
    f = wf.force(s, 16.0, 0.0, d)            # dashing due north
    assert f.f_n == pytest.approx(-d.mass_kg * d.drag_accel_m_s2(16.0), abs=1e-9)
    assert f.f_e == pytest.approx(0.0, abs=1e-12)
    assert f.v_rel_mps == pytest.approx(16.0)


def test_force_is_parallel_to_the_relative_air_velocity():
    c = _cond(mean_mps=8.0, dir_from_deg=270.0, sigma_mps=0.0, tau_s=1.0)
    wf = WindField(c, seed=1, sim_step_s=STEP)
    d = DragParams.px4_x500_mono_cam_sitl()
    s = wf.sample(0.0)
    f = wf.force(s, 3.0, -2.0, d)
    cross = f.f_n * f.v_rel_e - f.f_e * f.v_rel_n
    assert abs(cross) < 1e-9                                  # parallel
    assert f.f_n * f.v_rel_n + f.f_e * f.v_rel_e > 0.0        # same direction, not opposed
    assert f.magnitude_n == pytest.approx(d.mass_kg * d.drag_accel_m_s2(f.v_rel_mps),
                                          rel=1e-9)


def test_crosswind_force_is_purely_lateral_to_a_northbound_dash():
    """The mechanism-attribution arm: a pure westerly (blowing east) against a
    due-north dash puts the force east + a drag component south."""
    c = _cond(mean_mps=8.0, dir_from_deg=270.0, sigma_mps=0.0, tau_s=1.0)
    wf = WindField(c, seed=1, sim_step_s=STEP)
    f = wf.force(wf.sample(0.0), 16.0, 0.0, DragParams.px4_x500_mono_cam_sitl())
    assert f.v_rel_n == pytest.approx(-16.0) and f.v_rel_e == pytest.approx(8.0)
    assert f.f_e > 0.0 and f.f_n < 0.0


# ======================================================================= tiers
@pytest.mark.parametrize("name,mean,peak", [
    ("BEST", 1.5, 2.079),        # Beaufort 1 top; peak = mean + 2 sigma
    ("EXPECTED", 4.5, 6.237),    # Beaufort 3 mid
    ("WORST", 8.0, 11.088),      # Beaufort 5 bottom
])
def test_all_three_tiers_are_sourced_and_ordered(name, mean, peak):
    """CLAUDE.md: BEST / EXPECTED / WORST-credible, and decisions must survive
    WORST. The means are Beaufort/WMO anchors, not tuned numbers, and the tier
    provenance string carries the source into every log."""
    c = tier(name, height_m=H, dir_from_deg=270.0)
    assert c.mean_mps == mean
    assert c.peak_mps(2.0) == pytest.approx(peak, abs=1e-3)
    assert c.label == name
    assert "Beaufort" in c.provenance and str(mean) in c.provenance


def test_tiers_are_monotone_in_every_consequence():
    d = DragParams.px4_x500_mono_cam_sitl()
    cs = [tier(n, height_m=H, dir_from_deg=270.0)
          for n in ("BEST", "EXPECTED", "WORST")]
    assert [c.mean_mps for c in cs] == sorted(c.mean_mps for c in cs)
    assert [c.sigma_mps for c in cs] == sorted(c.sigma_mps for c in cs)
    # ...and tau SHRINKS with wind speed (tau = L/V): faster wind, faster gusts.
    assert [c.tau_s for c in cs] == sorted((c.tau_s for c in cs), reverse=True)
    tilts = [d.hover_tilt_deg(c.mean_mps) for c in cs]
    assert tilts == sorted(tilts)
    assert tilts[0] == pytest.approx(1.3948, abs=1e-3)
    assert tilts[-1] == pytest.approx(9.2209, abs=1e-3)


def test_worst_tier_exceeds_the_targets_own_no_fly_limit():
    """CROSS-DOC CONSISTENCY, flagged rather than silently reconciled:
    docs/placard_mount.md sets the TARGET's hard no-fly at '> 6 m/s steady or
    > 8 m/s gusting'. The WORST-credible tier here (8 m/s mean, ~11 m/s gusts)
    is beyond that -- deliberately, because it is the interceptor's worst
    credible day -- but it means a WORST-tier sim result has NO matching field
    session while the placard target is in use."""
    assert tier("WORST", height_m=H, dir_from_deg=0.0).mean_mps > 6.0


# ================================================================ (d) fail-closed
@pytest.mark.parametrize("kwargs", [
    dict(mean_mps=None), dict(dir_from_deg=None), dict(height_m=None),
    dict(sigma_mps=None), dict(tau_s=None),
    dict(mean_mps=float("nan")), dict(sigma_mps=float("nan")),
    dict(mean_mps=float("inf")),
    dict(mean_mps=-1.0), dict(height_m=0.0), dict(sigma_mps=-0.1),
    dict(tau_s=0.0), dict(label=""), dict(provenance=""), dict(provenance=None),
])
def test_conditions_fail_closed_on_a_missing_or_impossible_input(kwargs):
    with pytest.raises(MissingMeasurementError):
        _cond(**kwargs)


@pytest.mark.parametrize("kwargs", [
    dict(mass_kg=None), dict(mcoef_per_s=None), dict(bcoef_kg_m2=None),
    dict(rho_kg_m3=None), dict(provenance=""), dict(provenance="   "),
    dict(mass_kg=0.0), dict(mass_kg=-1.0), dict(mcoef_per_s=-0.1),
    dict(bcoef_kg_m2=-1.0), dict(rho_kg_m3=0.0), dict(mass_kg=float("nan")),
])
def test_drag_params_fail_closed(kwargs):
    base = dict(mass_kg=2.0, mcoef_per_s=0.15, bcoef_kg_m2=100.0,
                rho_kg_m3=1.225, provenance=PROV)
    base.update(kwargs)
    with pytest.raises(MissingMeasurementError):
        DragParams(**base)


def test_error_messages_say_how_to_measure_the_thing():
    """The builder mandate is that every realism knob maps to a
    BENCH-MEASURABLE quantity. The failure message is where that mapping
    actually reaches whoever hit the error."""
    with pytest.raises(MissingMeasurementError) as e:
        _cond(mean_mps=None)
    assert "anemometer" in str(e.value) and "m/s" in str(e.value)
    with pytest.raises(MissingMeasurementError) as e:
        DragParams(mass_kg=2.0, mcoef_per_s=None, bcoef_kg_m2=100.0,
                   rho_kg_m3=1.225, provenance=PROV)
    assert "ULog" in str(e.value) and "1/s" in str(e.value)


@pytest.mark.parametrize("kwargs", [
    dict(seed=None), dict(seed="7"), dict(seed=1.5), dict(seed=True),
    dict(sim_step_s=None), dict(sim_step_s=0.0), dict(sim_step_s=-0.05),
    dict(sim_step_s=float("nan")), dict(max_catchup_steps=0),
])
def test_windfield_fails_closed_on_config(kwargs):
    base = dict(conditions=_cond(), seed=1, sim_step_s=STEP)
    base.update(kwargs)
    with pytest.raises(MissingMeasurementError):
        WindField(**base)


def test_seed_is_required_because_pairing_is_the_evidence_standard():
    with pytest.raises(MissingMeasurementError) as e:
        WindField(_cond(), seed=None, sim_step_s=STEP)
    assert "paired" in str(e.value)


def test_sim_time_may_not_run_backwards():
    """ADR-0009: sim time, never wall time. A backwards step means the clock
    source is wrong; silently re-serving the old sample would hide it."""
    wf = WindField(_cond(), seed=1, sim_step_s=STEP)
    wf.sample(5.0)
    wf.sample(5.0)                       # same grid point is fine
    wf.sample(5.02)                      # within the same step is fine
    with pytest.raises(MissingMeasurementError) as e:
        wf.sample(1.0)
    assert "BACKWARDS" in str(e.value)
    with pytest.raises(MissingMeasurementError):
        WindField(_cond(), seed=1, sim_step_s=STEP).sample(-0.1)
    with pytest.raises(MissingMeasurementError):
        WindField(_cond(), seed=1, sim_step_s=STEP).sample(None)


def test_absurd_clock_jump_fails_closed_instead_of_grinding():
    wf = WindField(_cond(), seed=1, sim_step_s=STEP, max_catchup_steps=100)
    with pytest.raises(MissingMeasurementError) as e:
        wf.sample(1e6)
    assert "clock fault" in str(e.value)


def test_dryden_below_ten_feet_requires_an_explicit_acknowledgement():
    """The MIL-F-8785C low-altitude form is specified at/above 10 ft. Below it
    the formula still evaluates -- which is exactly how an out-of-validity
    number gets quoted -- so the caller must say they mean it."""
    with pytest.raises(MissingMeasurementError) as e:
        dryden_sigma_mps(4.5, 0.5)
    assert "10 ft" in str(e.value)
    assert dryden_sigma_mps(4.5, 0.5, allow_below_10ft=True) > 0.0
    with pytest.raises(MissingMeasurementError):
        tier("EXPECTED", height_m=1.0, dir_from_deg=0.0)
    assert tier("EXPECTED", height_m=1.0, dir_from_deg=0.0,
                allow_below_10ft=True).sigma_mps > 0.0


def test_still_air_has_no_dryden_timescale_and_says_so():
    """tau = L/V is undefined at V = 0. Rather than defaulting, the still-air
    control arm has its own explicit constructor."""
    with pytest.raises(MissingMeasurementError) as e:
        dryden_tau_s(0.0, H)
    assert "still_air" in str(e.value)
    c = WindConditions.still_air(height_m=H, provenance=PROV)
    assert c.mean_mps == 0.0 and c.sigma_mps == 0.0 and c.gust_factor() is None


def test_unknown_tier_name_fails_closed():
    with pytest.raises(MissingMeasurementError):
        tier("BREEZY", height_m=H, dir_from_deg=0.0)


def test_tier_will_not_invent_a_height_or_a_direction():
    """Height and direction are properties of the SITE and the SESSION, not of
    a tier -- so they have no defaults, at either the API or the CLI."""
    with pytest.raises(TypeError):
        tier("EXPECTED")
    with pytest.raises(TypeError):
        tier("EXPECTED", height_m=H)
    assert wm.main(["--tiers"]) == 2          # CLI refuses --tiers with no --height


def test_gust_factor_below_one_is_rejected():
    with pytest.raises(MissingMeasurementError):
        sigma_from_gust_factor(4.5, 0.8)


# ============================================================== anemometer cue
# NOT TESTED HERE, BECAUSE NOT IMPLEMENTED HERE. The noisy pre-flight
# anemometer reading is scripts/wind_anemometer_sim.py (producer) ->
# scripts/wind_trim.py (consumer), with its own honesty + contract tests.
# A second implementation in this module would be a second ruler for the
# same quantity; test_module_exposes_no_second_anemometer below pins that.


def test_module_exposes_no_second_anemometer():
    """The wind model generates TRUTH (world physics). It must not also offer
    a vehicle-facing reading of that truth -- one repo, one anemometer model."""
    assert not [n for n in dir(wm) if "anemometer" in n.lower()]


# ================================================================ logging shape
def test_samples_and_conditions_serialise_with_provenance():
    """The driver writes these to logs/wind_<run>.csv; the provenance has to
    travel with the numbers or the log is unsourced."""
    c = tier("WORST", height_m=H, dir_from_deg=123.0)
    d = c.to_dict()
    assert set(d) == {"mean_mps", "dir_from_deg", "height_m", "sigma_mps",
                      "tau_s", "label", "provenance"}
    assert d["provenance"]
    s = WindField(c, seed=1, sim_step_s=STEP).sample(0.3)
    sd = s.to_dict()
    assert set(sd) == {"t_sim_s", "grid_index", "grid_t_sim_s", "steady_n",
                       "steady_e", "gust_u", "gust_v", "wind_n", "wind_e",
                       "wind_d", "speed_mps"}
    assert sd["grid_index"] == 6 and sd["grid_t_sim_s"] == pytest.approx(0.30)


# ============================================================== honesty (static)
def test_module_imports_nothing_from_the_sim_or_the_vehicle():
    """This module generates WORLD PHYSICS. It must not reach into Gazebo,
    PX4, MAVSDK or the flight core -- the driver that does the gz plumbing is
    a separate file, and keeping the model importable-anywhere is what makes
    it testable off-sim at all."""
    src = open(os.path.join(SCRIPTS, "wind_model.py")).read()
    tree = ast.parse(src)
    mods = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            mods.update(a.name.split(".")[0] for a in n.names)
        elif isinstance(n, ast.ImportFrom) and n.module:
            mods.add(n.module.split(".")[0])
    assert mods <= {"argparse", "math", "random", "sys", "dataclasses",
                    "typing", "__future__"}, f"unexpected imports: {mods}"


def test_module_reads_no_ground_truth():
    """Honesty boundary: gt_* is scoring/logging only. Nothing named gt_ may
    appear as a value this module consumes. (The word appears in the docstring
    prose explaining the rule; the check is on the CODE, via the AST.)"""
    tree = ast.parse(open(os.path.join(SCRIPTS, "wind_model.py")).read())
    names = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Name):
            names.add(n.id)
        elif isinstance(n, ast.Attribute):
            names.add(n.attr)
    assert not [x for x in names if x.startswith("gt_")]


def test_self_test_entry_point_exits_zero():
    assert wm.main(["--self-test"]) == 0
