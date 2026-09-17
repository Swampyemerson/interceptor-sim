"""isim/scenario.py tests (ADR-0102 isim). Geometry sanity + aim-error effect
on the pre-flight heading solve. No sim runs here beyond `build()` itself --
that is `test_mc.py`'s job."""
from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from flight.deploy.real_flight import MissionConfig

from isim.replay_a0 import load_params
from isim.scenario import GO_SETTLE_BUFFER_S, Scenario, build


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
