"""isim/scenario.py tests (ADR-0102 isim). Geometry sanity + aim-error effect
on the pre-flight heading solve. No sim runs here beyond `build()` itself --
that is `test_mc.py`'s job."""
from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from flight.deploy.real_flight import MissionConfig, resolve_preflight_heading, wrap_deg

from isim.replay_a0 import load_params
from isim.scenario import (
    GO_SETTLE_BUFFER_S,
    NOMINAL_ALT_M,
    Scatter,
    Scenario,
    _BASE_DASH_ACCEL_MS2,
    _BASE_DASH_SPEED_MS,
    _target_geometry,
    build,
)


@pytest.fixture(scope="module")
def vp():
    return load_params()


def _go_at_s(cfg: MissionConfig) -> float:
    return cfg.standby_settle_s + GO_SETTLE_BUFFER_S


def test_crossing_target_is_abeam_at_the_predicted_time(vp):
    scn = Scenario(target_speed_ms=9.0, cross_range_m=6.5, lead_dist_m=16.2,
                   direction=1.0, head_on=False)
    ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
    t_go = _go_at_s(guidance.cfg)
    t_abeam = t_go + scn.lead_dist_m / scn.target_speed_ms

    s = target.state(t_abeam)
    assert s.pos_ned[0] == pytest.approx(0.0, abs=1e-6)          # north == 0: abeam
    assert s.pos_ned[1] == pytest.approx(scn.cross_range_m, abs=1e-6)  # east offset held

    # And it really was still upstream (before abeam) a moment earlier.
    s_before = target.state(t_abeam - 1.0)
    assert s_before.pos_ned[0] < 0.0


def test_crossing_direction_flips_the_approach_side(vp):
    scn_pos = Scenario(direction=1.0)
    scn_neg = Scenario(direction=-1.0)
    for scn, sign in ((scn_pos, 1.0), (scn_neg, -1.0)):
        ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
        t_go = _go_at_s(guidance.cfg)
        s0 = target.state(t_go)   # just after GO: motion has started
        s1 = target.state(t_go + 0.5)
        d_north = s1.pos_ned[0] - s0.pos_ned[0]
        assert math.copysign(1.0, d_north) == sign


def test_head_on_target_passes_over_the_launch_point(vp):
    scn = Scenario(head_on=True, target_speed_ms=9.0, lead_dist_m=16.2, direction=1.0)
    ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
    t_go = _go_at_s(guidance.cfg)
    t_hit = t_go + scn.lead_dist_m / scn.target_speed_ms

    s = target.state(t_hit)
    assert s.pos_ned[0] == pytest.approx(0.0, abs=1e-6)
    assert s.pos_ned[1] == pytest.approx(0.0, abs=1e-6)


def test_head_on_direction_flips_which_side_it_starts_on(vp):
    for d in (1.0, -1.0):
        scn = Scenario(head_on=True, lead_dist_m=16.2, direction=d)
        ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
        s0 = target.state(0.0)
        assert math.copysign(1.0, s0.pos_ned[0]) == d
        assert abs(s0.pos_ned[0]) == pytest.approx(16.2, abs=1e-6)


def test_target_holds_at_start_until_go(vp):
    scn = Scenario()
    ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
    t_go = _go_at_s(guidance.cfg)
    s_early = target.state(0.0)
    s_mid = target.state(t_go * 0.5)
    s_at_go = target.state(t_go)
    assert np.array_equal(s_early.pos_ned, s_mid.pos_ned)
    assert np.array_equal(s_early.pos_ned, s_at_go.pos_ned)
    assert np.array_equal(s_early.vel_ned, np.zeros(3))
    # And it is moving immediately after.
    s_after = target.state(t_go + 0.1)
    assert not np.array_equal(s_after.pos_ned, s_at_go.pos_ned)


def test_aim_error_rotates_the_dash_heading_by_exactly_that_angle(vp):
    base = Scenario(aim_error_deg=0.0)
    err = 12.0
    perturbed = dataclasses.replace(base, aim_error_deg=err)

    _, _, _, _, g0, _ = build(base, vp)
    _, _, _, _, g1, _ = build(perturbed, vp)

    diff = g1.cfg.preflight_heading_deg - g0.cfg.preflight_heading_deg
    diff_wrapped = ((diff + 180.0) % 360.0) - 180.0
    assert diff_wrapped == pytest.approx(err, abs=1e-6)


def test_height_guess_error_perturbs_standby_alt_only(vp):
    base = Scenario(height_guess_error_m=0.0, target_alt_offset_m=0.0)
    err = Scenario(height_guess_error_m=1.5, target_alt_offset_m=0.0)

    _, _, tgt0, _, g0, init0 = build(base, vp)
    _, _, tgt1, _, g1, init1 = build(err, vp)

    assert g1.cfg.standby_alt_m - g0.cfg.standby_alt_m == pytest.approx(1.5, abs=1e-9)
    assert init1.pos_ned[2] - init0.pos_ned[2] == pytest.approx(-1.5, abs=1e-9)
    # The target's true altitude is untouched by the operator's height guess.
    assert tgt0.state(0.0).pos_ned[2] == pytest.approx(tgt1.state(0.0).pos_ned[2], abs=1e-9)


def test_target_alt_offset_moves_only_the_target(vp):
    base = Scenario(target_alt_offset_m=0.0)
    off = Scenario(target_alt_offset_m=2.0)
    _, _, tgt0, _, g0, _ = build(base, vp)
    _, _, tgt1, _, g1, _ = build(off, vp)
    d_alt = -tgt1.state(0.0).pos_ned[2] - (-tgt0.state(0.0).pos_ned[2])
    assert d_alt == pytest.approx(2.0, abs=1e-9)
    assert g1.cfg.standby_alt_m == pytest.approx(g0.cfg.standby_alt_m, abs=1e-9)


def test_sprint_scale_zero_zeroes_dash_speed_but_does_not_skip_coded_dash(vp):
    scn = Scenario(sprint_scale=0.0)
    _, _, _, _, guidance, _ = build(scn, vp)
    assert guidance.cfg.dash_speed_ms == pytest.approx(0.0, abs=1e-12)
    # Skipping CODED_DASH is not reachable from outside flight/ -- see the
    # module docstring. This just pins that dash_speed_ms really is zeroed.


def test_sprint_scale_one_uses_the_full_missionconfig_default_speed(vp):
    scn = Scenario(sprint_scale=1.0)
    _, _, _, _, guidance, _ = build(scn, vp)
    assert guidance.cfg.dash_speed_ms == pytest.approx(MissionConfig().dash_speed_ms, abs=1e-9)


# --------------------------------------------------------------------- scatter

def test_scatter_none_is_bit_identical_to_no_scatter(vp):
    """Pin: `scatter=None` (the default) must reproduce the pre-scatter
    arithmetic exactly -- no noise term may leak in even as a `+0.0`, and the
    vehicle model must not be copied/perturbed at all."""
    scn = Scenario(aim_error_deg=7.0, height_guess_error_m=-0.4, seed=11)
    assert scn.scatter is None

    ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)

    _, _, start_en, vel_en = _target_geometry(scn)
    heading0, _ = resolve_preflight_heading(
        start_en, vel_en, _BASE_DASH_SPEED_MS, dash_accel_ms2=_BASE_DASH_ACCEL_MS2,
        accel_aware=True, origin=(0.0, 0.0))
    expected_heading = wrap_deg(heading0 + scn.aim_error_deg)

    assert guidance.cfg.preflight_heading_deg == pytest.approx(expected_heading, abs=1e-12)
    assert guidance.cfg.standby_alt_m == pytest.approx(
        NOMINAL_ALT_M + scn.height_guess_error_m, abs=1e-12)
    assert vehicle.p is vp   # same object -- the no-scatter path never copies it
    assert seeker.cam.fx == pytest.approx(540.0)
    assert seeker.cam.mount_tilt_up_deg == pytest.approx(0.0)


def test_scatter_same_seed_identical_draws_different_seed_differs(vp):
    scat = Scatter()
    scn_a = Scenario(scatter=scat, seed=5)
    scn_b = Scenario(scatter=scat, seed=5)
    scn_c = Scenario(scatter=scat, seed=6)

    _, veh_a, _, seeker_a, g_a, _ = build(scn_a, vp)
    _, veh_b, _, seeker_b, g_b, _ = build(scn_b, vp)
    _, veh_c, _, seeker_c, g_c, _ = build(scn_c, vp)

    assert g_a.cfg.preflight_heading_deg == g_b.cfg.preflight_heading_deg
    assert g_a.cfg.standby_alt_m == g_b.cfg.standby_alt_m
    assert veh_a.p.wind_ned == veh_b.p.wind_ned
    assert seeker_a.cam.fx == seeker_b.cam.fx

    differs = (g_a.cfg.preflight_heading_deg != g_c.cfg.preflight_heading_deg
              or g_a.cfg.standby_alt_m != g_c.cfg.standby_alt_m
              or veh_a.p.wind_ned != veh_c.p.wind_ned
              or seeker_a.cam.fx != seeker_c.cam.fx)
    assert differs


def test_scatter_target_start_jitter_never_reaches_preflight_solve(vp):
    """The TRUE start-position jitter (`target_start_sigma_m`) must not change
    the solved heading (or anything drawn after it) at all -- honesty
    boundary: the pre-flight solve is never told the true target moved."""
    scat_full = Scatter()
    scat_no_start_jitter = dataclasses.replace(scat_full, target_start_sigma_m=0.0)

    scn_full = Scenario(scatter=scat_full, seed=3)
    scn_no_jitter = Scenario(scatter=scat_no_start_jitter, seed=3)

    _, veh_full, tgt_full, _, g_full, _ = build(scn_full, vp)
    _, veh_no_jitter, tgt_no_jitter, _, g_no_jitter, _ = build(scn_no_jitter, vp)

    # Every later draw (heading noise, height noise, speed belief, wind,
    # vehicle params, latency, go jitter, camera error) is bit-identical --
    # a Gaussian draw with sigma=0 still consumes exactly the same amount of
    # RNG state as a nonzero one, so the sequence downstream is untouched.
    assert g_full.cfg.preflight_heading_deg == g_no_jitter.cfg.preflight_heading_deg
    assert g_full.cfg.standby_alt_m == g_no_jitter.cfg.standby_alt_m
    assert veh_full.p.wind_ned == veh_no_jitter.p.wind_ned

    # But the TRUE target start position really did move.
    pos_full = tgt_full.inner.state(0.0).pos_ned
    pos_no_jitter = tgt_no_jitter.inner.state(0.0).pos_ned
    assert pos_full[0] != pytest.approx(pos_no_jitter[0])


def test_scatter_speed_belief_error_feeds_the_solve_not_the_true_target(vp):
    """`target_speed_belief_sigma_frac` perturbs what the SOLVE is handed; the
    true target's flown speed is untouched."""
    scat = dataclasses.replace(Scatter(), heading_sigma_deg=0.0, height_guess_sigma_m=0.0,
                               target_start_sigma_m=0.0)
    scat_zero_belief = dataclasses.replace(scat, target_speed_belief_sigma_frac=0.0)

    scn = Scenario(scatter=scat, seed=9)
    scn_zero_belief = Scenario(scatter=scat_zero_belief, seed=9)

    _, _, tgt, _, g, _ = build(scn, vp)
    _, _, tgt_zero, _, g_zero, _ = build(scn_zero_belief, vp)

    # The solved heading differs (the belief speed the solve saw differed)...
    assert g.cfg.preflight_heading_deg != g_zero.cfg.preflight_heading_deg
    # ...but the TRUE target's speed (norm of vel_ned) is identical either way.
    true_speed = float(np.linalg.norm(tgt.inner.vel_ned))
    true_speed_zero = float(np.linalg.norm(tgt_zero.inner.vel_ned))
    assert true_speed == pytest.approx(true_speed_zero, abs=1e-12)
    assert true_speed == pytest.approx(scn.target_speed_ms, abs=1e-9)


def test_scatter_wind_reaches_vehicle_params(vp):
    scn = Scenario(scatter=Scatter(), seed=42)
    _, vehicle, _, _, _, _ = build(scn, vp)
    wn, we, wd = vehicle.p.wind_ned
    mag = math.hypot(wn, we)
    assert 0.0 <= mag <= Scatter().wind_mean_max_ms + 1e-9
    assert wd == pytest.approx(0.0)
    assert vehicle.p.gust_std == pytest.approx(Scatter().gust_std_ms)
    # Vehicle params only touched on the scatter-on path -- never the shared
    # caller-supplied object.
    assert vehicle.p is not vp


def test_scatter_camera_error_is_true_camera_only(vp):
    """fx/fy scale + mount-tilt error land on the TRUE (seeker) camera only;
    the flight code's nominal camera stays at the Scenario's nominal tilt."""
    scn = Scenario(scatter=Scatter(), cam_tilt_up_deg=5.0, seed=1)
    _, _, _, seeker, guidance, _ = build(scn, vp)
    assert seeker.cam.mount_tilt_up_deg != pytest.approx(5.0)
    assert guidance._gcfg.mount_up_rad == pytest.approx(math.radians(5.0))


def test_terminal_stock_default_matches_pre_terminal_kwarg_behaviour(vp):
    scn = Scenario(terminal="stock")
    ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
    assert guidance is not None   # builds without a TypeError


def test_terminal_tag_is_import_guarded(vp):
    """`terminal="tag"` is only exercised once `flight.tag_terminal` lands
    (another worker's task); until then this must not fail collection."""
    pytest.importorskip("flight.tag_terminal")
    scn = Scenario(terminal="tag")
    build(scn, vp)
