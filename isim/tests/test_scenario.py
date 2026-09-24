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
    assert seeker.cam.fx == pytest.approx(scn.cam_fx_px)   # v7: nominal is 385, not a magic 540
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


# ------------------------------------------------------------- v5 hardening

def test_flyby_ignores_the_new_ownstate_fields_whatever_their_value(vp):
    """pursuit_hardening_v5.md's "Also check": concept="flyby" must stay
    bit-identical however loud the new OWN-STATE fields are -- flyby never
    wraps its guidance in `OwnStateNoise` at all (that path is
    concept="pursuit" only). Deliberately excludes the decode-realism
    fields here: those DO reach flyby too (the TRUE seeker/camera is
    shared infrastructure, not pursuit-specific) -- see the next test for
    THEIR "zero -> old behaviour" claim instead."""
    scat_default = Scatter()
    scat_loud = dataclasses.replace(
        scat_default, ownstate_attitude_bias_rp_sigma_deg=50.0,
        ownstate_attitude_bias_yaw_sigma_deg=50.0, ownstate_vel_bias_sigma_ms=50.0,
        ownstate_pos_randomwalk_sigma_ms_sqrt_s=50.0, ownstate_frame_ts_bias_max_s=50.0)
    scn_a = Scenario(concept="flyby", scatter=scat_default, seed=3)
    scn_b = Scenario(concept="flyby", scatter=scat_loud, seed=3)
    _, veh_a, _, seeker_a, g_a, init_a = build(scn_a, vp)
    _, veh_b, _, seeker_b, g_b, init_b = build(scn_b, vp)
    assert g_a.cfg.preflight_heading_deg == pytest.approx(g_b.cfg.preflight_heading_deg, abs=1e-12)
    assert g_a.cfg.standby_alt_m == pytest.approx(g_b.cfg.standby_alt_m, abs=1e-12)
    assert seeker_a.dec.p_max == pytest.approx(seeker_b.dec.p_max)
    assert veh_a.p.wind_ned == veh_b.p.wind_ned
    np.testing.assert_array_equal(init_a.pos_ned, init_b.pos_ned)


def test_decode_realism_fields_at_zero_reproduce_old_fixed_decodeparams(vp):
    """The OTHER half of "bit-identical when the new scatter terms are
    zero": with `decode_p_max_min=0.98`/`decode_pixel_noise_min_px=
    decode_pixel_noise_max_px=0.3` (their "off" values -- the draw range
    collapses to a point), EITHER concept gets exactly today's fixed
    `DecodeParams()` defaults, regardless of `scatter` otherwise being on."""
    scat_off = dataclasses.replace(Scatter(), decode_p_max_min=0.98,
                                   decode_pixel_noise_min_px=0.3, decode_pixel_noise_max_px=0.3)
    for concept in ("flyby", "pursuit"):
        scn = Scenario(concept=concept, scatter=scat_off, seed=5)
        _, _, _, seeker, _, _ = build(scn, vp)
        assert seeker.dec.p_max == pytest.approx(0.98)
        assert seeker.dec.pixel_noise_px == pytest.approx(0.3)


def test_pursuit_ownstate_noise_only_wraps_when_scatter_is_set(vp):
    """`scatter=None` -> no `OwnStateNoise` wrapper at all (bit-identical to
    pre-v5 pursuit); `scatter=Scatter()` -> wrapped."""
    from isim.concepts import PursuitRendezvousGuidance
    from isim.ownstate import OwnStateNoise

    scn_none = Scenario(concept="pursuit", seed=0)
    _, _, _, _, g_none, _ = build(scn_none, vp)
    assert isinstance(g_none, PursuitRendezvousGuidance)

    scn_scat = Scenario(concept="pursuit", scatter=Scatter(), seed=0)
    _, _, _, _, g_scat, _ = build(scn_scat, vp)
    assert isinstance(g_scat, OwnStateNoise)
    assert isinstance(g_scat.inner, PursuitRendezvousGuidance)


def test_decode_realism_draws_p_max_and_pixel_noise_within_range(vp):
    scat = Scatter()
    seen_p_max, seen_px = set(), set()
    for seed in range(20):
        scn = Scenario(scatter=scat, seed=seed)
        _, _, _, seeker, _, _ = build(scn, vp)
        assert scat.decode_p_max_min <= seeker.dec.p_max <= 0.98 + 1e-9
        assert scat.decode_pixel_noise_min_px <= seeker.dec.pixel_noise_px \
            <= scat.decode_pixel_noise_max_px + 1e-9
        seen_p_max.add(seeker.dec.p_max)
        seen_px.add(seeker.dec.pixel_noise_px)
    assert len(seen_p_max) > 1 and len(seen_px) > 1   # really varies run to run


def test_decode_realism_off_when_scatter_is_none(vp):
    scn = Scenario(scatter=None, seed=0)
    _, _, _, seeker, _, _ = build(scn, vp)
    assert seeker.dec.p_max == pytest.approx(0.98)
    assert seeker.dec.pixel_noise_px == pytest.approx(0.3)


def test_target_motion_straight_is_the_default_and_unaffected_by_scatter(vp):
    from isim.targets import ConstantVelocityTarget
    scn = Scenario(scatter=Scatter(), seed=0)
    assert scn.target_motion == "straight"
    _, _, target, _, _, _ = build(scn, vp)
    assert isinstance(target.inner, ConstantVelocityTarget)


def test_target_motion_weave_builds_a_weave_target_with_drawn_params(vp):
    from isim.targets import WeaveTarget
    scat = Scatter()
    scn = Scenario(target_motion="weave", scatter=scat, seed=0)
    _, _, target, _, _, _ = build(scn, vp)
    assert isinstance(target.inner, WeaveTarget)
    assert scat.weave_amp_min_m <= target.inner.amp_m <= scat.weave_amp_max_m
    assert scat.weave_period_min_s <= target.inner.period_s <= scat.weave_period_max_s


def test_target_motion_speed_change_builds_with_drawn_params(vp):
    from isim.targets import SpeedChangeTarget
    scat = Scatter()
    scn = Scenario(target_motion="speed_change", scatter=scat, seed=0)
    _, _, target, _, _, _ = build(scn, vp)
    assert isinstance(target.inner, SpeedChangeTarget)
    assert abs(target.inner.delta_ms) <= scat.speed_change_delta_max_ms + 1e-9
    assert 2.0 <= target.inner.change_t <= scat.speed_change_time_max_s


def test_target_motion_non_straight_requires_scatter(vp):
    scn = Scenario(target_motion="weave", scatter=None, seed=0)
    with pytest.raises(ValueError):
        build(scn, vp)


# ----------------------------------------------------- v5 honesty (the spy)

class _OwnStateSpyGuidance:
    """Wraps the (already `OwnStateNoise`-wrapped) guidance `build()`
    returns and records the exact `own`/`det` the ENGINE handed to IT --
    i.e. one layer OUTSIDE `OwnStateNoise`, so this sees the TRUE state (the
    engine never perturbs anything); `OwnStateNoise` sits between this spy
    and the real `PursuitRendezvousGuidance` and does the perturbing."""

    def __init__(self, inner) -> None:
        self.inner = inner
        self.seen_true = []

    def reset(self) -> None:
        self.inner.reset()

    def step(self, t, own, det):
        self.seen_true.append((t, own, det))
        return self.inner.step(t, own, det)


def test_guidance_sees_the_perturbed_state_not_the_true_state(vp):
    """pursuit_hardening_v5.md "Also check": guidance receives only the
    (perturbed) own state and detections, and scoring uses the true state.
    Proven two ways in one run: (1) the innermost `PursuitRendezvousGuidance`
    is fed a DIFFERENT own-position than the engine's true trace at the
    same tick (perturbation reached guidance); (2) `EngagementResult.trace`
    -- what scoring reads -- matches the TRUE vehicle-model output exactly,
    never the perturbed one."""
    from isim.engine import run_engagement
    from isim.concepts import PursuitRendezvousGuidance

    scat = dataclasses.replace(Scatter(), ownstate_pos_randomwalk_sigma_ms_sqrt_s=5.0,
                               ownstate_vel_bias_sigma_ms=0.0, ownstate_vel_white_sigma_ms=0.0,
                               ownstate_attitude_white_sigma_deg=0.0,
                               ownstate_attitude_bias_rp_sigma_deg=0.0,
                               ownstate_attitude_bias_yaw_sigma_deg=0.0,
                               ownstate_frame_ts_bias_max_s=0.0,
                               ownstate_frame_ts_jitter_sigma_s=0.0)
    scn = Scenario(concept="pursuit", target_speed_ms=9.0, cross_range_m=6.5,
                   lead_dist_m=16.2, scatter=scat, seed=0)
    ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)

    # guidance == OwnStateNoise(PursuitRendezvousGuidance(...)); tap the
    # innermost real guidance's OWN record of what it was handed by
    # wrapping the pursuit guidance a SECOND layer down.
    assert hasattr(guidance, "inner")
    real_guidance = guidance.inner
    assert isinstance(real_guidance, PursuitRendezvousGuidance)
    inner_spy = _OwnStateSpyGuidance(real_guidance)
    guidance.inner = inner_spy   # swap: OwnStateNoise now forwards to the spy

    outer_spy = _OwnStateSpyGuidance(guidance)   # sees the TRUE state (engine side)
    result = run_engagement(ecfg, vehicle, target, seeker, outer_spy, init, record_trace=True)

    assert len(outer_spy.seen_true) > 5
    assert len(inner_spy.seen_true) > 5
    # Find a tick recorded on BOTH sides (same t) well after GO, and compare.
    true_by_t = {round(t, 6): own for t, own, _ in outer_spy.seen_true}
    diffs = []
    for t, pert_own, _ in inner_spy.seen_true:
        true_own = true_by_t.get(round(t, 6))
        if true_own is not None and t > scn_go_estimate(vp):
            diffs.append(float(np.linalg.norm(pert_own.pos_ned - true_own.pos_ned)))
    assert diffs, "no matching ticks found -- test setup problem"
    assert max(diffs) > 0.0, "guidance's own state never differed from truth -- wrapper is inert"

    # Scoring (the trace) must be the TRUE vehicle output -- untouched. The
    # trace is recorded every fine engine tick, while guidance.step() (and
    # hence outer_spy) only fires every guidance_dt -- compare at a
    # matching t, not just the trace's last row (which can be a few fine
    # ticks past the last guidance call).
    checked_any = False
    for i, t_trace in enumerate(result.trace["t"]):
        true_own = true_by_t.get(round(float(t_trace), 6))
        if true_own is not None:
            np.testing.assert_array_equal(result.trace["own_pos"][i], true_own.pos_ned)
            checked_any = True
    assert checked_any, "no matching (trace, outer_spy) ticks found -- test setup problem"


def scn_go_estimate(vp) -> float:
    """A cheap lower bound on go_at_s for the default MissionConfig, so the
    honesty test above only compares post-GO ticks (before GO, own_pos is
    identically the standby point and a 0.0 diff would prove nothing)."""
    return MissionConfig().standby_settle_s


# ------------------------------------------- tag_realism_v1 (spec section D/E7)

def _trace_digest(scn: Scenario, vp) -> str:
    """sha256 over a full engagement: the whole engine trace plus every
    FrameReport's LEGACY fields (the fields that existed before
    tag_realism_v1, so a digest taken before the change is comparable)."""
    import contextlib
    import hashlib
    import io

    from isim.engine import run_engagement

    ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
    with contextlib.redirect_stdout(io.StringIO()):
        r = run_engagement(ecfg, vehicle, target, seeker, guidance, init, record_trace=True)
    m = hashlib.sha256()
    for k in sorted(r.trace):
        m.update(k.encode())
        m.update(np.ascontiguousarray(r.trace[k]).tobytes())
    for f in r.frame_reports:
        m.update(np.array([f.t_capture, f.side_px, f.incidence_deg, f.blur_px, f.p_decode,
                           float(f.in_fov), float(f.decoded)]).tobytes())
    m.update(np.array([r.miss_m, r.t_cpa]).tobytes())
    return m.hexdigest()


# Digests measured on the PRE-tag_realism_v1 code (2026-09-23, this dev
# machine, numpy 2.5.1). A floating-point hash is platform-sensitive; if this
# fails on a DIFFERENT machine with every realism test otherwise green, re-pin
# from a checkout of the parent commit rather than from the new code.
_PRE_REALISM_DIGESTS = {
    "flyby_default": (
        dict(seed=3),
        "66b7935a94c78abc6e35929cf23f2211313529f235b768baa231ad0731028697"),
    "pursuit_rear_scatter_weave": (
        dict(concept="pursuit", tag_facing="rear", scatter=Scatter(),
             target_motion="weave", seed=5),
        "c4b08a4ebca007a16c5451535e7d72308dbb3525c5ba2725a4f1eaf6acb60caa"),
    "flyby_rear_dual35_scatter": (
        dict(tag_facing="rear_dual35", scatter=Scatter(), seed=2),
        "3eeda4df23e345bd80becc8b0fbc6c13d482cb37d0a1cb4d9a736153dc089d55"),
    "pursuit_camera_scatter": (
        dict(concept="pursuit", scatter=Scatter(), seed=4),
        "a61af2b15b25302b8fd1fc7fad72bc5e9f53efd7e2e41f315ba158fe5d370a59"),
    "pursuit_side_speedchange": (
        dict(concept="pursuit", tag_facing="side", scatter=Scatter(),
             target_motion="speed_change", seed=1),
        "db32bf834944d96da595bc19226f553a30808f10981eb852fd3c45a87425e75b"),
}


@pytest.mark.parametrize("name", sorted(_PRE_REALISM_DIGESTS))
def test_realism_defaults_are_byte_identical_to_pre_realism_runs(vp, name):
    """E7 pin: with target_attitude=False / glare=False (defaults) and every
    new Scatter field at its default, a full engagement is byte-identical to
    the same run on the code BEFORE tag_realism_v1 existed."""
    kwargs, digest = _PRE_REALISM_DIGESTS[name]
    assert _trace_digest(Scenario(**kwargs), vp) == digest


def test_realism_knobs_are_inert_while_their_flags_are_off(vp):
    """The gated Scatter fields (drag tilt, wobble, sun/glare) draw nothing and
    change nothing unless their Scenario flag is on -- however loud."""
    loud = dataclasses.replace(
        Scatter(), tgt_drag_tilt_sigma_deg=50.0, tgt_shake_rms_min_deg=5.0,
        tgt_shake_rms_max_deg=9.0, sun_elevation_min_deg=1.0, specular_strength_min=0.9,
        specular_strength_max=0.99, backlight_kill_max_deg=80.0)
    base = Scenario(concept="pursuit", tag_facing="rear", scatter=Scatter(), seed=7)
    assert _trace_digest(base, vp) == _trace_digest(dataclasses.replace(base, scatter=loud), vp)


def test_realism_flags_do_not_reshuffle_the_older_draws(vp):
    """New draws sit at the END of the rng stream: turning the flags on keeps
    every pre-existing per-run draw identical (paired seeds stay paired)."""
    scat = dataclasses.replace(Scatter(), own_vib_rate_rms_max_dps=40.0)
    off = Scenario(tag_facing="rear", target_motion="weave", scatter=scat, seed=12)
    on = dataclasses.replace(off, target_attitude=True, glare=True)
    _, veh0, tgt0, sk0, g0, _ = build(off, vp)
    _, veh1, tgt1, sk1, g1, _ = build(on, vp)
    assert veh0.p.wind_ned == veh1.p.wind_ned
    assert g0.cfg.preflight_heading_deg == g1.cfg.preflight_heading_deg
    assert sk0.dec.p_max == sk1.dec.p_max and sk0.cam.fx == sk1.cam.fx
    assert sk0.cam.vib_rate_rms_dps == sk1.cam.vib_rate_rms_dps
    assert tgt0.inner.amp_m == tgt1.inner.inner.amp_m
    assert tgt0.inner.period_s == tgt1.inner.inner.period_s
    # ...and each realism GROUP is independent of the other groups' flags.
    glare_only = dataclasses.replace(off, glare=True)
    att_only = dataclasses.replace(off, target_attitude=True)
    _, _, _, sk_g, _, _ = build(glare_only, vp)
    _, _, tgt_a, sk_a, _, _ = build(att_only, vp)
    assert sk_g.glare == sk1.glare
    assert sk_a.dec.tgt_shake_rms_deg == sk1.dec.tgt_shake_rms_deg
    assert tgt_a.inner.prm == tgt1.inner.prm


def test_target_attitude_wraps_the_motion_and_bolts_rear_side_tags(vp):
    from isim.target_attitude import AttitudeTarget, euler_rpy_deg
    from isim.targets import ConstantVelocityTarget

    scn = Scenario(tag_facing="rear", target_attitude=True)
    _, _, target, seeker, guidance, _ = build(scn, vp)
    assert isinstance(target.inner, AttitudeTarget)
    assert isinstance(target.inner.inner, ConstantVelocityTarget)
    assert seeker.tag.body_normal_frd == (-1.0, 0.0, 0.0)
    assert seeker.tag.normal_ned == (-1.0, -0.0, -0.0)     # legacy fallback kept
    # No scatter -> no draws: nominal drag tilt, wobble off.
    assert target.inner.prm.drag_tilt_at_9ms_deg == 12.0
    assert seeker.dec.tgt_shake_rms_deg == 0.0
    t_go = _go_at_s(guidance.cfg)
    s_pre = target.state(0.0)          # frozen before GO: t=0 attitude, no rate
    assert s_pre.quat_wxyz == target.inner.state(0.0).quat_wxyz
    np.testing.assert_array_equal(s_pre.ang_vel_body, np.zeros(3))
    _, pitch, _ = euler_rpy_deg(target.state(t_go + 1.0).quat_wxyz)
    assert pitch == pytest.approx(-12.0, abs=1e-9)
    # "side" is sign-matched to today's world-frame normal (faces the launch
    # side, west): flying north, west is body-LEFT; flying south, body-RIGHT.
    for direction, want_y in ((1.0, -1.0), (-1.0, 1.0)):
        _, _, _, sk, _, _ = build(Scenario(tag_facing="side", target_attitude=True,
                                           direction=direction), vp)
        assert sk.tag.body_normal_frd == (0.0, want_y, 0.0)
        assert sk.tag.normal_ned[1] == -1.0
    # "camera" stays the idealized cheat.
    _, _, _, sk_cam, _, _ = build(Scenario(tag_facing="camera", target_attitude=True), vp)
    assert sk_cam.tag.faces_camera and sk_cam.tag.body_normal_frd is None


def test_rear_dual35_body_mount_matches_the_world_normals_at_level(vp):
    from isim.seeker import quat_to_rot

    _, _, target, seeker, guidance, _ = build(
        Scenario(tag_facing="rear_dual35", target_attitude=True, direction=-1.0), vp)
    q_level_yaw = (0.0, 0.0, 0.0, 1.0)          # flying south: yaw 180, level
    r = quat_to_rot(q_level_yaw)
    for sk in (seeker.first, seeker.second):
        np.testing.assert_allclose(r @ np.array(sk.tag.body_normal_frd),
                                   np.array(sk.tag.normal_ned), atol=1e-12)


def test_target_attitude_with_scatter_draws_tilt_and_wobble_in_range(vp):
    scat = Scatter()
    tilts = set()
    for seed in range(12):
        _, _, target, seeker, _, _ = build(
            Scenario(tag_facing="rear", target_attitude=True, scatter=scat, seed=seed), vp)
        tilt = target.inner.prm.drag_tilt_at_9ms_deg
        assert tilt >= 0.0
        tilts.add(tilt)
        assert scat.tgt_shake_rms_min_deg <= seeker.dec.tgt_shake_rms_deg \
            <= scat.tgt_shake_rms_max_deg
        assert seeker.cam.vib_rate_rms_dps == 0.0      # own vib default OFF
        assert not seeker.glare.active()               # glare flag off
    assert len(tilts) > 1


def test_glare_needs_scatter_and_draws_its_geometry(vp):
    with pytest.raises(ValueError):
        build(Scenario(glare=True), vp)
    scat = Scatter()
    _, _, _, seeker, _, _ = build(Scenario(glare=True, scatter=scat, seed=3), vp)
    g = seeker.glare
    assert 0.0 <= g.sun_azimuth_deg < 360.0
    assert scat.sun_elevation_min_deg <= g.sun_elevation_deg <= scat.sun_elevation_max_deg
    assert scat.specular_strength_min <= g.specular_strength <= scat.specular_strength_max
    assert 0.0 <= g.backlight_kill_deg <= scat.backlight_kill_max_deg
    pinned = dataclasses.replace(scat, sun_azimuth_uniform=False)
    _, _, _, sk2, _, _ = build(Scenario(glare=True, scatter=pinned, sun_azimuth_deg=77.0,
                                        sun_elevation_deg=8.0, seed=3), vp)
    assert (sk2.glare.sun_azimuth_deg, sk2.glare.sun_elevation_deg) == (77.0, 8.0)


def test_own_vibration_scatter_draws_only_when_nonzero(vp):
    scat = dataclasses.replace(Scatter(), own_vib_rate_rms_max_dps=40.0)
    _, _, _, seeker, _, _ = build(Scenario(scatter=scat, seed=2), vp)
    assert 0.0 < seeker.cam.vib_rate_rms_dps <= 40.0


def test_realism_scenario_is_picklable_and_runs(vp):
    import pickle

    from isim.engine import run_engagement

    scn = Scenario(concept="pursuit", tag_facing="rear", target_motion="weave",
                   target_attitude=True, glare=True,
                   scatter=dataclasses.replace(Scatter(), own_vib_rate_rms_max_dps=40.0),
                   seed=5)
    assert pickle.loads(pickle.dumps(scn)) == scn
    ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
    r = run_engagement(ecfg, vehicle, target, seeker, guidance, init)
    assert math.isfinite(r.miss_m) and r.n_frames > 0
    assert any(f.tgt_shake_deg > 0.0 for f in r.frame_reports)
