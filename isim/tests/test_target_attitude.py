"""tag_realism_v1 tests (isim/specs/tag_realism_v1.md section E): the derived
target attitude (A2), the body-mounted tag (A3), rotational blur (A4), target
wobble + own vibration (B), sun glare (C), and the additive-type defaults (E8).

Frames: world NED (+z down), body FRD, camera OpenCV. An identity-quaternion
interceptor at the origin looks NORTH, so a target at (10, 0, 0) is dead ahead
on the boresight and the line of sight is the north axis.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from isim.seeker import (AprilTagSeeker, CameraParams, DecodeParams, GLARE_UNMEASURED,
                         GlareParams, TagParams, UNMEASURED, glare_multipliers,
                         tag_corners_ned)
from isim.target_attitude import (UNMEASURED as ATT_UNMEASURED, AttitudeTarget,
                                  TargetAttitudeParams, euler_rpy_deg, tilt_deg)
from isim.targets import ConstantVelocityTarget, HoverTarget, SpeedChangeTarget, WeaveTarget
from isim.types import FrameReport, TargetState, VehicleState


def vs(pos=(0.0, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0)) -> VehicleState:
    return VehicleState(t=0.0, pos_ned=np.array(pos, float), vel_ned=np.zeros(3),
                        quat_wxyz=q, yaw_rad=0.0)


def ts(pos, vel=(0.0, 0.0, 0.0), q=None, w=None) -> TargetState:
    return TargetState(t=0.0, pos_ned=np.array(pos, float), vel_ned=np.array(vel, float),
                       quat_wxyz=q, ang_vel_body=None if w is None else np.array(w, float))


def q_roll(deg: float):
    h = math.radians(deg) / 2.0
    return (math.cos(h), math.sin(h), 0.0, 0.0)


def certain_dec(**kw) -> DecodeParams:
    base = dict(p_max=1.0, size50_px=0.0, size_k_px=0.5, min_side_px=0.0,
                inc50_deg=1.0e6, inc_k_deg=1.0, blur50_cells=1.0e6, pixel_noise_px=0.0)
    base.update(kw)
    return DecodeParams(**base)


def certain_cam(**kw) -> CameraParams:
    base = dict(latency_s=0.0, latency_jitter_s=0.0)
    base.update(kw)
    return CameraParams(**base)


class CountingRng:
    """Wraps a Generator and counts every draw by method -- the "no new rng
    draw on the default path" guard."""

    def __init__(self, seed: int) -> None:
        self._g = np.random.default_rng(seed)
        self.calls: dict = {}

    def __getattr__(self, name):
        attr = getattr(self._g, name)
        if not callable(attr):
            return attr

        def wrapped(*a, **k):
            self.calls[name] = self.calls.get(name, 0) + 1
            return attr(*a, **k)
        return wrapped


# ------------------------------------------------------ A2: attitude physics

def test_level_cruise_pitches_nose_down_by_the_drag_tilt_with_zero_rate():
    tgt = AttitudeTarget(ConstantVelocityTarget([0.0, 0.0, -10.0], [9.0, 0.0, 0.0]))
    s = tgt.state(2.0)
    roll, pitch, yaw = euler_rpy_deg(s.quat_wxyz)
    assert roll == pytest.approx(0.0, abs=1e-9)
    assert pitch == pytest.approx(-12.0, abs=1e-9)      # measured: -12.000 deg exactly
    assert yaw == pytest.approx(0.0, abs=1e-9)
    np.testing.assert_allclose(s.ang_vel_body, np.zeros(3), atol=1e-9)
    # Quadratic drag: half the speed -> tan(tilt) quartered.
    slow = AttitudeTarget(ConstantVelocityTarget([0.0, 0.0, -10.0], [4.5, 0.0, 0.0]))
    _, p_slow, _ = euler_rpy_deg(slow.state(2.0).quat_wxyz)
    assert math.tan(math.radians(-p_slow)) == pytest.approx(
        math.tan(math.radians(12.0)) / 4.0, rel=1e-9)
    # Heading follows the velocity (flying east -> yaw +90).
    east = AttitudeTarget(ConstantVelocityTarget([0.0, 0.0, -10.0], [0.0, 9.0, 0.0]))
    r_e, p_e, y_e = euler_rpy_deg(east.state(1.0).quat_wxyz)
    assert (r_e, p_e, y_e) == pytest.approx((0.0, -12.0, 90.0), abs=1e-9)


def test_weave_banks_about_13_deg_at_the_weave_period():
    amp, period = 2.0, 6.0
    tgt = AttitudeTarget(WeaveTarget([0.0, 0.0, -10.0], [9.0, 0.0, 0.0], amp, period))
    omega = 2.0 * math.pi / period
    predicted = math.degrees(math.atan(amp * omega * omega / 9.81))   # 12.60 deg
    ts_ = np.arange(0.0, 2.0 * period, 0.005)
    rolls = np.array([euler_rpy_deg(tgt.state(t).quat_wxyz)[0] for t in ts_])
    assert rolls.max() == pytest.approx(predicted, abs=0.3)    # measured 12.58 deg
    assert rolls.min() == pytest.approx(-predicted, abs=0.3)
    # Peak lateral accel (max offset east, accel west) at t = period/4 -> left bank.
    r_q, _, _ = euler_rpy_deg(tgt.state(period / 4.0).quat_wxyz)
    assert r_q == pytest.approx(-rolls.max(), abs=1e-6)
    # Periodic at the weave period, sign-flipped at half of it.
    for t in (0.7, 1.9, 2.6):
        r0 = euler_rpy_deg(tgt.state(t).quat_wxyz)[0]
        assert euler_rpy_deg(tgt.state(t + period).quat_wxyz)[0] == pytest.approx(r0, abs=1e-6)
        assert euler_rpy_deg(tgt.state(t + period / 2.0).quat_wxyz)[0] == pytest.approx(
            -r0, abs=1e-6)
    # A banking target has a nonzero roll rate, peaking at the zero crossing.
    w = tgt.state(0.0).ang_vel_body
    assert abs(float(w[0])) > 0.1


def test_hover_is_identity_with_yaw_north():
    s = AttitudeTarget(HoverTarget([3.0, 4.0, -5.0])).state(7.0)
    assert s.quat_wxyz == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-12)
    np.testing.assert_allclose(s.ang_vel_body, np.zeros(3), atol=1e-12)


def test_tilt_cap_and_speed_change_transient_stay_finite_and_capped():
    prm = TargetAttitudeParams()
    tgt = AttitudeTarget(SpeedChangeTarget([0.0, 0.0, -10.0], [9.0, 0.0, 0.0],
                                           delta_ms=2.0, change_t=5.0), prm)
    tilts = []
    for t in np.arange(4.7, 5.3, 0.001):
        s = tgt.state(float(t))
        assert all(math.isfinite(c) for c in s.quat_wxyz)
        assert np.all(np.isfinite(s.ang_vel_body))
        tilts.append(tilt_deg(s.quat_wxyz))
    assert max(tilts) <= prm.max_tilt_deg + 1e-9
    assert max(tilts) == pytest.approx(prm.max_tilt_deg, abs=1e-9)   # the pulse DOES hit the cap
    # Away from the step it relaxes back to steady cruise (11 m/s after).
    _, p_after, _ = euler_rpy_deg(tgt.state(6.0).quat_wxyz)
    expected = math.degrees(math.atan(math.tan(math.radians(12.0)) * (11.0 / 9.0) ** 2))
    assert -p_after == pytest.approx(expected, abs=1e-9)
    # A tighter cap binds on plain cruise too.
    capped = AttitudeTarget(ConstantVelocityTarget([0, 0, -10], [9.0, 0, 0]),
                            TargetAttitudeParams(max_tilt_deg=5.0))
    assert tilt_deg(capped.state(1.0).quat_wxyz) == pytest.approx(5.0, abs=1e-9)


def test_attitude_is_a_pure_function_of_t():
    tgt = AttitudeTarget(WeaveTarget([0.0, 0.0, -10.0], [9.0, 0.0, 0.0], 2.5, 5.0))
    a = tgt.state(3.3)
    tgt.state(0.1)          # an unrelated call in between must change nothing
    b = tgt.state(3.3)
    assert a.quat_wxyz == b.quat_wxyz
    assert np.array_equal(a.ang_vel_body, b.ang_vel_body)
    assert np.array_equal(a.pos_ned, b.pos_ned) and np.array_equal(a.vel_ned, b.vel_ned)


# ----------------------------------------------------- A3: body-mounted tag

def test_level_body_mount_reproduces_the_legacy_corners():
    tag = TagParams(normal_ned=(-1.0, 0.0, 0.0), body_normal_frd=(-1.0, 0.0, 0.0))
    tgt = ts((10.0, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0))
    n, e1 = tag.frame(tgt, np.zeros(3))
    np.testing.assert_allclose(tag_corners_ned(tgt.pos_ned, n, 0.3, e1),
                               tag_corners_ned(tgt.pos_ned, (-1.0, 0.0, 0.0), 0.3), atol=1e-12)


def test_banked_target_rotates_the_corner_set_by_the_bank_about_the_los():
    bank = 25.0
    tag = TagParams(normal_ned=(-1.0, 0.0, 0.0), body_normal_frd=(-1.0, 0.0, 0.0))
    centre = np.array([10.0, 0.0, 0.0])
    tgt = ts(centre, q=q_roll(bank))
    n, e1 = tag.frame(tgt, np.zeros(3))
    np.testing.assert_allclose(n, [-1.0, 0.0, 0.0], atol=1e-12)   # roll about the LOS
    got = tag_corners_ned(centre, n, 0.3, e1)
    # Hand-rotate the level corners by +bank about north (the LOS).
    c, s = math.cos(math.radians(bank)), math.sin(math.radians(bank))
    rx = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    want = (rx @ (tag_corners_ned(centre, (-1.0, 0.0, 0.0), 0.3) - centre).T).T + centre
    np.testing.assert_allclose(got, want, atol=1e-12)
    # No quat -> legacy path, unrotated.
    n0, e10 = tag.frame(ts(centre), np.zeros(3))
    assert e10 is None


def test_faces_camera_ignores_attitude():
    tag = TagParams(faces_camera=True, body_normal_frd=(-1.0, 0.0, 0.0))
    tgt = ts((10.0, 0.0, 0.0), q=q_roll(40.0), w=(3.0, 0.0, 0.0))
    skr = AprilTagSeeker(certain_cam(), tag, certain_dec())
    skr.reset(np.random.default_rng(0))
    _, rep = skr.observe(0.0, vs(), tgt)
    assert rep.incidence_deg == pytest.approx(0.0, abs=1e-6)
    assert rep.blur_rot_px == 0.0


def test_rear_tag_on_a_pitched_cruiser_adds_the_pitch_to_astern_incidence():
    """E3 regression: chaser dead astern at the target's altitude. Upright
    world-frame rear tag -> incidence 0; the same tag bolted to a 9 m/s
    cruiser (nose-down 12 deg) faces up-and-back -> incidence 12 deg."""
    motion = ConstantVelocityTarget([10.0, 0.0, 0.0], [9.0, 0.0, 0.0])
    legacy = TagParams(normal_ned=(-1.0, 0.0, 0.0))
    body = TagParams(normal_ned=(-1.0, 0.0, 0.0), body_normal_frd=(-1.0, 0.0, 0.0))
    for tag, tgt_state, want in ((legacy, motion.state(0.0), 0.0),
                                 (body, AttitudeTarget(motion).state(0.0), 12.0)):
        skr = AprilTagSeeker(certain_cam(), tag, certain_dec())
        skr.reset(np.random.default_rng(0))
        _, rep = skr.observe(0.0, vs(), tgt_state)
        assert rep.incidence_deg == pytest.approx(want, abs=1e-6)   # measured 12.000
    # The shift tracks the pitch, not a constant.
    s20 = AttitudeTarget(motion, TargetAttitudeParams(drag_tilt_at_9ms_deg=20.0)).state(0.0)
    skr = AprilTagSeeker(certain_cam(), body, certain_dec())
    skr.reset(np.random.default_rng(0))
    _, rep = skr.observe(0.0, vs(), s20)
    assert rep.incidence_deg == pytest.approx(20.0, abs=1e-6)
    # The normal points BACK and UP (NED z < 0).
    n = body.normal(s20, np.zeros(3))
    assert n[0] < 0.0 and n[2] < 0.0


# ---------------------------------------------------- A4: rotational blur

def test_spinning_target_adds_the_predicted_rotational_blur():
    cam = certain_cam()
    omega, z = 6.0, 10.0
    tag = TagParams(normal_ned=(-1.0, 0.0, 0.0), body_normal_frd=(-1.0, 0.0, 0.0))
    # Both vehicles static; first frame -> own body rate 0: translational smear 0.
    tgt = ts((z, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0), w=(omega, 0.0, 0.0))
    skr = AprilTagSeeker(cam, tag, certain_dec())
    skr.reset(np.random.default_rng(0))
    _, rep = skr.observe(0.0, vs(), tgt)
    predicted = cam.fx * omega * (0.5 * tag.side_m) / z * cam.exposure_s
    assert rep.blur_rot_px == pytest.approx(predicted, rel=1e-12)
    assert rep.blur_px == pytest.approx(predicted, rel=1e-12)
    # Absent ang_vel -> exactly zero.
    skr.reset(np.random.default_rng(0))
    _, rep0 = skr.observe(0.0, vs(), ts((z, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0)))
    assert rep0.blur_rot_px == 0.0 and rep0.blur_px == 0.0


def test_rotational_blur_is_root_sum_square_with_translation():
    cam = certain_cam()
    tag = TagParams(normal_ned=(-1.0, 0.0, 0.0), body_normal_frd=(-1.0, 0.0, 0.0))
    vel = (0.0, 5.0, 0.0)      # crossing: pure translational smear

    def blur(w):
        skr = AprilTagSeeker(cam, tag, certain_dec())
        skr.reset(np.random.default_rng(0))
        return skr.observe(0.0, vs(), ts((10.0, 0.0, 0.0), vel=vel,
                                         q=(1.0, 0.0, 0.0, 0.0), w=w))[1]
    trans = blur(None).blur_px
    r = blur((4.0, 0.0, 0.0))
    assert trans > 0.0
    assert r.blur_px == pytest.approx(math.hypot(trans, r.blur_rot_px), rel=1e-12)


# ------------------------------------------------------------- B: shake

def _run_frames(skr, n, tgt, fps=38.2):
    reps = []
    for i in range(n):
        _, rep = skr.observe(i / fps, vs(), tgt)
        reps.append(rep)
    return reps


def test_shake_rms_converges_to_the_knob():
    rms = 2.0
    skr = AprilTagSeeker(certain_cam(), TagParams(normal_ned=(-1.0, 0.0, 0.0)),
                         DecodeParams(tgt_shake_rms_deg=rms, tgt_shake_bw_hz=3.0))
    skr.reset(np.random.default_rng(42))
    reps = _run_frames(skr, 6000, ts((10.0, 0.0, 0.0)))
    mag = np.array([r.tgt_shake_deg for r in reps])
    per_axis_rms = math.sqrt(float(np.mean(mag ** 2)) / 2.0)
    assert per_axis_rms == pytest.approx(rms, rel=0.05)
    # The wobble genuinely moves the incidence off 0 and adds rotational blur.
    assert np.mean([r.incidence_deg for r in reps]) > 1.0
    assert np.mean([r.blur_rot_px for r in reps[1:]]) > 0.0


def test_shake_off_draws_nothing_and_reset_clears_state():
    tgt = ts((6.0, 0.0, 0.0))

    def run(dec):
        rng = CountingRng(3)
        skr = AprilTagSeeker(CameraParams(), TagParams(normal_ned=(-1.0, 0.0, 0.0)), dec)
        skr.reset(rng)
        reps = _run_frames(skr, 300, tgt)
        return rng.calls, reps, skr

    calls_off, reps_off, _ = run(DecodeParams())
    assert "standard_normal" not in calls_off
    # Only the legacy draws: one uniform per p>0 frame, 3 normals (+1 latency
    # jitter) per decode.
    n_try = sum(1 for r in reps_off if r.p_decode > 0.0)
    n_dec = sum(1 for r in reps_off if r.decoded)
    assert calls_off.get("random", 0) == n_try
    assert calls_off.get("normal", 0) == 4 * n_dec
    assert all(r.tgt_shake_deg == 0.0 and r.blur_rot_px == 0.0 for r in reps_off)

    calls_on, _, skr = run(DecodeParams(tgt_shake_rms_deg=1.5))
    assert calls_on["standard_normal"] == 300          # one 2-axis draw per frame
    assert skr._shake_t is not None and np.any(skr._shake != 0.0)
    skr.reset(np.random.default_rng(0))
    assert skr._shake_t is None and np.array_equal(skr._shake, np.zeros(2))


def test_own_vibration_blur_is_off_by_default_and_scales_when_on():
    tgt = ts((10.0, 0.0, 0.0))
    rng = CountingRng(1)
    skr = AprilTagSeeker(CameraParams(), TagParams(faces_camera=True), DecodeParams())
    skr.reset(rng)
    reps = _run_frames(skr, 50, tgt)
    assert all(r.blur_vib_px == 0.0 for r in reps)
    n_dec = sum(1 for r in reps if r.decoded)
    assert rng.calls.get("normal", 0) == 4 * n_dec      # no vibration draws

    cam = CameraParams(vib_rate_rms_dps=40.0)
    skr = AprilTagSeeker(cam, TagParams(faces_camera=True), DecodeParams())
    skr.reset(np.random.default_rng(1))
    vib = np.array([r.blur_vib_px for r in _run_frames(skr, 4000, tgt)])
    # |N(0, s)| has mean s*sqrt(2/pi).
    want = cam.fx * math.radians(40.0) * math.sqrt(2.0 / math.pi) * cam.exposure_s
    assert float(np.mean(vib)) == pytest.approx(want, rel=0.05)


# -------------------------------------------------------------- C: glare

def test_specular_geometry_exact_hit_and_sun_behind_the_tag():
    n = np.array([-1.0, 0.0, 0.0])            # tag faces south, camera due south
    to_cam = np.array([-10.0, 0.0, 0.0])
    bore = np.array([1.0, 0.0, 0.0])
    # Sun due south on the horizon: the mirror ray returns straight to the camera.
    gp = GlareParams(sun_azimuth_deg=180.0, sun_elevation_deg=0.0, specular_strength=0.6)
    spec, back = glare_multipliers(gp, n, to_cam, bore)
    assert spec == pytest.approx(1.0 - 0.6, abs=1e-12)
    assert back == 1.0
    # Sun 12 deg above the normal -> mirror ray 12 deg below it; the camera
    # sits ON the normal, so theta = 12 deg = one lobe half-width -> exp(-1).
    gp_w = GlareParams(sun_azimuth_deg=180.0, sun_elevation_deg=12.0,
                       specular_strength=0.6, specular_width_deg=12.0)
    spec_w, _ = glare_multipliers(gp_w, n, to_cam, bore)
    assert spec_w == pytest.approx(1.0 - 0.6 * math.exp(-1.0), abs=1e-9)
    # Sun behind the tag plane -> no specular at all.
    gp_b = GlareParams(sun_azimuth_deg=0.0, sun_elevation_deg=10.0, specular_strength=0.9)
    assert glare_multipliers(gp_b, n, to_cam, bore)[0] == 1.0


def test_backlight_on_axis_kills_by_strength_and_ramps_linearly():
    n = np.array([-1.0, 0.0, 0.0])
    to_cam = np.array([-10.0, 0.0, 0.0])
    bore = np.array([1.0, 0.0, 0.0])           # looking north, into the sun
    on_axis = GlareParams(sun_azimuth_deg=0.0, sun_elevation_deg=0.0,
                          backlight_kill_deg=20.0, backlight_strength=0.9)
    assert glare_multipliers(on_axis, n, to_cam, bore)[1] == pytest.approx(0.1, abs=1e-12)
    half = GlareParams(sun_azimuth_deg=0.0, sun_elevation_deg=10.0,
                       backlight_kill_deg=20.0, backlight_strength=0.9)
    assert glare_multipliers(half, n, to_cam, bore)[1] == pytest.approx(0.55, abs=1e-9)
    outside = GlareParams(sun_azimuth_deg=0.0, sun_elevation_deg=30.0,
                          backlight_kill_deg=20.0, backlight_strength=0.9)
    assert glare_multipliers(outside, n, to_cam, bore)[1] == 1.0


def test_glare_multiplies_p_in_the_seeker_and_off_changes_nothing():
    tgt = ts((10.0, 0.0, 0.0))
    tag = TagParams(normal_ned=(-1.0, 0.0, 0.0))
    dec = DecodeParams()

    def one(glare, rng):
        skr = AprilTagSeeker(certain_cam(), tag, dec, glare)
        skr.reset(rng)
        return skr.observe(0.0, vs(), tgt)[1]

    base = one(None, np.random.default_rng(0))
    rng_off = CountingRng(0)
    off = one(GlareParams(), rng_off)          # default GlareParams = both effects off
    assert off.p_decode == base.p_decode
    assert (off.glare_specular_mult, off.glare_backlight_mult) == (1.0, 1.0)
    assert rng_off.calls == {"random": 1}       # just the legacy decode draw
    hit = one(GlareParams(sun_azimuth_deg=180.0, sun_elevation_deg=0.0,
                          specular_strength=0.5), np.random.default_rng(0))
    assert hit.glare_specular_mult == pytest.approx(0.5, abs=1e-12)
    assert hit.p_decode == pytest.approx(0.5 * base.p_decode, rel=1e-12)


# --------------------------------------------------- E8 / honesty bookkeeping

def test_old_constructors_still_work_with_neutral_defaults():
    rep = FrameReport(t_capture=0.0, in_fov=True, side_px=10.0, incidence_deg=0.0,
                      blur_px=0.0, p_decode=0.5, decoded=False)
    assert (rep.blur_rot_px, rep.blur_vib_px, rep.tgt_shake_deg) == (0.0, 0.0, 0.0)
    assert (rep.glare_specular_mult, rep.glare_backlight_mult) == (1.0, 1.0)
    st = TargetState(t=0.0, pos_ned=np.zeros(3), vel_ned=np.zeros(3))
    assert st.quat_wxyz is None and st.ang_vel_body is None
    assert TagParams().body_normal_frd is None
    assert DecodeParams().tgt_shake_rms_deg == 0.0
    assert CameraParams().vib_rate_rms_dps == 0.0
    assert not GlareParams().active()


def test_new_unmeasured_names_are_real_fields():
    seeker_fields = set(CameraParams.__dataclass_fields__) | set(
        TagParams.__dataclass_fields__) | set(DecodeParams.__dataclass_fields__)
    for name in ("body_normal_frd", "tgt_shake_rms_deg", "tgt_shake_bw_hz",
                 "vib_rate_rms_dps"):
        assert name in UNMEASURED and name in seeker_fields
    assert set(GLARE_UNMEASURED) == set(GlareParams.__dataclass_fields__)
    assert set(ATT_UNMEASURED) == set(TargetAttitudeParams.__dataclass_fields__)
