"""The two stern_approach_2026-10-21 geometry knobs (isim/specs/
stern_approach_2026-10-21.md): `pursuit_d_behind_m` (Phase-A trail distance)
and `dash_aim_behind_m` (the dash heading solve aims behind the believed
track). Both are PRE-ACQUISITION levers only -- the post-lock chase law is
untouched by construction (they feed PursuitConfig.d_behind_m and the
pre-flight solve; neither reaches Phase B/T code paths).
"""
from __future__ import annotations

import math

from dataclasses import replace

import pytest

from isim.replay_a0 import load_params
from isim.scenario import Scenario, build, resolve_preflight_heading


@pytest.fixture()
def vp():
    return load_params()


def _scn(**kw) -> Scenario:
    kw.setdefault("concept", "pursuit")
    return Scenario(target_speed_ms=9.0, cross_range_m=6.5,
                    lead_dist_m=16.2, cam_fx_px=933.0, tag_facing="rear",
                    seed=3, **kw)


def test_defaults_are_byte_identical(vp):
    """Fields at their defaults reproduce the pre-field build exactly
    (same solved heading, same PursuitConfig)."""
    _, _, _, _, g_a, init_a = build(_scn(), vp)
    _, _, _, _, g_b, init_b = build(
        _scn(pursuit_d_behind_m=None, dash_aim_behind_m=0.0), vp)
    assert g_a.initial_yaw_deg == g_b.initial_yaw_deg
    assert g_a.cfg == g_b.cfg
    assert g_a.cfg.d_behind_m == 8.0


def test_pursuit_d_behind_reaches_config_pursuit_and_hybrid(vp):
    _, _, _, _, g_p, _ = build(_scn(pursuit_d_behind_m=16.0), vp)
    assert g_p.cfg.d_behind_m == 16.0
    _, _, _, _, g_h, _ = build(
        _scn(pursuit_d_behind_m=24.0, concept="hybrid"), vp)
    assert g_h.pcfg.d_behind_m == 24.0


def test_dash_aim_behind_changes_heading_and_matches_direct_solve(vp):
    """The knob must (a) move the solved heading, (b) agree exactly with
    calling the solver on a hand-shifted belief start, and (c) leave the
    guidance's own belief untouched (the dash aims differently; the system
    does not believe differently)."""
    d = 12.0
    _, _, _, _, g0, _ = build(_scn(), vp)
    _, _, _, _, gd, _ = build(_scn(dash_aim_behind_m=d), vp)
    h0 = g0.initial_yaw_deg
    hd = gd.initial_yaw_deg
    assert h0 != hd

    # Reproduce the shifted solve by hand on the same geometry the module
    # docstring defines (no scatter, so belief == truth here).
    from isim.scenario import _target_geometry, _BASE_DASH_SPEED_MS, _BASE_DASH_ACCEL_MS2
    _, _, start_en, vel_en = _target_geometry(_scn())
    speed = math.hypot(*vel_en)
    shifted = (start_en[0] - d * vel_en[0] / speed,
               start_en[1] - d * vel_en[1] / speed)
    h_hand, _ = resolve_preflight_heading(
        shifted, vel_en, _BASE_DASH_SPEED_MS, dash_accel_ms2=_BASE_DASH_ACCEL_MS2,
        accel_aware=True, origin=(0.0, 0.0))
    assert hd == pytest.approx(h_hand, abs=1e-9)

    # Guidance belief unshifted: identical belief track in both builds.
    assert (g0._belief_pos0 == gd._belief_pos0).all()
    assert (g0._belief_vel0 == gd._belief_vel0).all()


def test_knobs_are_sweepable_scenario_fields():
    """isim.mc sweeps via dataclasses.replace(base, **{axis: value}) --
    both knobs must survive that machinery as plain float fields."""
    base = _scn()
    assert replace(base, pursuit_d_behind_m=4.0).pursuit_d_behind_m == 4.0
    assert replace(base, dash_aim_behind_m=24.0).dash_aim_behind_m == 24.0
