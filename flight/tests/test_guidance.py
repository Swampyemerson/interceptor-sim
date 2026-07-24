"""Pin flight.guidance.collision_lead_heading: the coded-dash aim. Covers the
measured flight-0 case, stationary/head-on/uncatchable branches, and the
l2r<->r2l mirror symmetry (ADR-0076: the guidance is provably symmetric). Run
standalone (exit 0/1) or under pytest.
"""

import math

from flight.guidance import (
    collision_lead_heading,
    closing_speed,
    compensate_terminal_los,
    crossing_sign,
    pronav_lateral_accel,
    dash_forward_speed,
    dash_loft_alt_ref,
    resolve_terminal_bearing_bias_deg,
    GRAVITY_MS2,
)


def test_flight0_lead():
    """The measured flight-0 geometry (ADR-0076 add #18b): target crossing at
    12 m/s, dash 16 m/s -> lead heading 140.0 deg (vs the broken 157 deg that
    aimed at the stale initial position and aborted 'no acquire')."""
    h, t = collision_lead_heading((6.5, -15.343), (0.0, 12.0), 16.0)
    assert abs(h - 140.0) < 0.2, h
    assert t is not None and abs(t - 0.632) < 0.01, t


def test_stationary_aims_at_target():
    """Zero target velocity -> aim straight at it; t_lead = range / dash_speed."""
    h, t = collision_lead_heading((0.0, 10.0), (0.0, 0.0), 10.0)
    assert abs(h - 0.0) < 1e-6, h          # due north
    assert abs(t - 1.0) < 1e-6, t


def test_head_on_aims_north():
    """Target approaching down the north axis -> aim due north, catchable."""
    h, t = collision_lead_heading((0.0, 20.0), (0.0, -5.0), 10.0)
    assert abs(h - 0.0) < 1e-6, h
    assert t is not None and abs(t - 4.0 / 3.0) < 1e-3, t


def test_uncatchable_falls_back_to_initial():
    """Target receding faster than the dash speed -> no positive root -> fall
    back to aiming at the initial position, t_lead None."""
    h, t = collision_lead_heading((0.0, 10.0), (0.0, 20.0), 10.0)
    assert t is None
    assert abs(h - 0.0) < 1e-6, h          # initial position azimuth


def test_lr_rl_mirror_symmetry():
    """Mirroring the geometry across the north axis (flip east sign of pos AND
    vel) flips the heading sign exactly -- the l2r and r2l aims are mirror
    images, so any r2l vs l2r asymmetry is NOT in the guidance (ADR-0076)."""
    hl, tl = collision_lead_heading((6.5, -15.343), (0.0, 12.0), 16.0)
    hr, tr = collision_lead_heading((-6.5, -15.343), (0.0, 12.0), 16.0)
    assert abs(hl + hr) < 1e-6, (hl, hr)   # equal and opposite
    assert abs(tl - tr) < 1e-9, (tl, tr)   # same intercept time


def test_explicit_origin_offset():
    """A non-origin launch point shifts the aim consistently."""
    h, t = collision_lead_heading((5.0, 5.0), (0.0, 0.0), 10.0, origin=(5.0, 0.0))
    assert abs(h - 0.0) < 1e-6, h          # target due north of the offset origin
    assert abs(t - 0.5) < 1e-6, t


def test_closing_speed_floor_and_measured():
    """Vc = floor until -Rdot exceeds it, then -Rdot; None -> floor."""
    assert closing_speed(None, 1.5) == 1.5           # filter not up
    assert closing_speed(-0.5, 1.5) == 1.5           # -Rdot=0.5 < floor
    assert closing_speed(-4.0, 1.5) == 4.0           # -Rdot=4.0 > floor
    assert closing_speed(3.0, 1.5) == 1.5            # positive Rdot (opening) -> floor


def test_pronav_law():
    """a = N * Vc * lambda_dot; None LOS rate -> 0."""
    assert pronav_lateral_accel(4.0, 10.0, 0.05) == 4.0 * 10.0 * 0.05
    assert pronav_lateral_accel(4.0, 10.0, 0.0) == 0.0
    assert pronav_lateral_accel(4.0, 10.0, None) == 0.0
    # sign follows lambda_dot (steer to null the LOS rotation)
    assert pronav_lateral_accel(4.0, 10.0, -0.05) < 0.0


def test_dash_forward_speed_default_byte_identical():
    """accel_cap None or <=0 -> the plain dash speed at every t (byte-identical)."""
    for t in (0.0, 0.5, 2.0, 10.0):
        assert dash_forward_speed(16.0, None, t) == 16.0
        assert dash_forward_speed(16.0, 0.0, t) == 16.0
        assert dash_forward_speed(16.0, -3.0, t) == 16.0


def test_dash_forward_speed_ramps_and_clamps():
    """Capped: speed ramps at accel_cap then clamps to dash_speed. theta20 =>
    a_cap = g*tan(20) ~ 3.57 m/s^2, so 16 m/s is reached at ~4.48 s (NOT within a
    ~2 s engagement -- the documented closing-speed cost)."""
    a = GRAVITY_MS2 * math.tan(math.radians(20))     # ~3.569
    assert dash_forward_speed(16.0, a, 0.0) == 0.0
    assert abs(dash_forward_speed(16.0, a, 1.0) - a) < 1e-9
    assert abs(dash_forward_speed(16.0, a, 2.0) - 2 * a) < 1e-9   # ~7.1 m/s at 2 s
    assert dash_forward_speed(16.0, a, 10.0) == 16.0             # clamped
    assert dash_forward_speed(16.0, a, -1.0) == 0.0             # pre-start guard


def test_dash_loft_alt_ref_default_byte_identical():
    """loft 0 (or dive_dur 0) -> the base altitude at every t (byte-identical)."""
    for t in (0.0, 1.0, 5.0):
        assert dash_loft_alt_ref(0.5, 0.0, t, 2.5) == 0.5
        assert dash_loft_alt_ref(0.5, 3.0, t, 0.0) == 0.5


def test_dash_loft_alt_ref_dives_from_loft_to_base():
    """Raised-cosine DIVE: +loft at entry, base at dive_dur, monotone between, and
    holds base after (clamped frac)."""
    base, loft, dur = 0.5, 3.0, 2.0
    assert abs(dash_loft_alt_ref(base, loft, 0.0, dur) - (base + loft)) < 1e-9
    mid = dash_loft_alt_ref(base, loft, dur / 2, dur)
    assert abs(mid - (base + loft / 2)) < 1e-9                  # half-way down at mid
    assert abs(dash_loft_alt_ref(base, loft, dur, dur) - base) < 1e-9
    assert abs(dash_loft_alt_ref(base, loft, 5.0, dur) - base) < 1e-9   # past dive: base
    # strictly descending across the dive
    a = dash_loft_alt_ref(base, loft, 0.4, dur)
    b = dash_loft_alt_ref(base, loft, 0.8, dur)
    assert a > b > base


# --- TERMINAL LOS bearing-bias compensation (docs/flight_plan_candidates.md
#     Sec 1.4 + arm H). ------------------------------------------------------

def test_crossing_sign_matches_the_flown_dash_bias_key():
    """crossing_sign must return the SIGN of the quantity scripts/m4_intercept.py
    computes inline for --dash-crossing-bias-deg (`sin(h)*vy - cos(h)*vx`), on the
    canonical line-9 geometries -- so the terminal bias and the dash bias key off
    ONE convention. (m4's flown inline block is deliberately not refactored; this
    test is what pins the two together.)"""
    for (tx, ty), (vx, vy) in (((6.5, -16.686), (0.0, 9.0)),      # canonical l2r
                               ((6.5, 16.150), (0.0, -9.0)),      # canonical r2l
                               ((6.5, -15.343), (0.0, 12.0)),     # flight-0 l2r
                               ((-6.5, -15.343), (0.0, 12.0))):   # mirrored
        h, _ = collision_lead_heading((tx, ty), (vx, vy), 16.0)
        inline = math.sin(math.radians(h)) * vy - math.cos(math.radians(h)) * vx
        expect = 1 if inline > 0 else (-1 if inline < 0 else 0)
        assert crossing_sign(h, (vx, vy)) == expect, (h, vx, vy, inline)
    # And the canonical directions map the way the mc_batch labels do:
    # l2r = target_vy > 0 -> +1, r2l = target_vy < 0 -> -1.
    hl, _ = collision_lead_heading((6.5, -16.686), (0.0, 9.0), 16.0)
    hr, _ = collision_lead_heading((6.5, 16.150), (0.0, -9.0), 16.0)
    assert crossing_sign(hl, (0.0, 9.0)) == 1
    assert crossing_sign(hr, (0.0, -9.0)) == -1


def test_crossing_sign_degenerate_is_zero():
    """A target flying straight down the dash line (or standing still) has no
    crossing sense -> 0, so the caller applies NO correction instead of a
    coin-flip sign."""
    assert crossing_sign(0.0, (0.0, 0.0)) == 0
    assert crossing_sign(0.0, (0.0, 5.0)) == 0        # receding along the aim
    assert crossing_sign(90.0, (5.0, 0.0)) == 0       # aim east, target east


def test_resolve_bias_symmetric_sign_keying():
    """One MAGNITUDE, sign auto-keyed: l2r -> +B, r2l -> -B (the measured sign
    pattern), degenerate -> 0."""
    assert resolve_terminal_bearing_bias_deg(1, 15.0) == 15.0
    assert resolve_terminal_bearing_bias_deg(-1, 15.0) == -15.0
    assert resolve_terminal_bearing_bias_deg(0, 15.0) == 0.0


def test_resolve_bias_per_direction_overrides_and_asymmetry():
    """Signed per-direction constants win where given, and can express the
    measured ASYMMETRY (+11.0 l2r vs -17.8 r2l on ARM B seed 123) that a single
    magnitude cannot."""
    assert resolve_terminal_bearing_bias_deg(1, 0.0, 11.0, -17.8) == 11.0
    assert resolve_terminal_bearing_bias_deg(-1, 0.0, 11.0, -17.8) == -17.8
    # a per-direction value given for ONE direction only: the other falls back
    # to the symmetric magnitude
    assert resolve_terminal_bearing_bias_deg(1, 15.0, 11.0, None) == 11.0
    assert resolve_terminal_bearing_bias_deg(-1, 15.0, 11.0, None) == -15.0
    # an explicit 0.0 is a real value, not "unset" -- it must NOT fall back
    assert resolve_terminal_bearing_bias_deg(1, 15.0, 0.0, None) == 0.0


def test_compensate_default_is_exact_identity():
    """Both knobs off -> the SAME object back: no wrap, no float round-trip, so
    an unflagged run is byte-identical (the standing requirement)."""
    for lam in (0.0, 0.3, -2.9, math.pi, 1e-9):
        assert compensate_terminal_los(lam) is lam
        assert compensate_terminal_los(lam, 0.0, 1.234, 0.0) is lam
        assert compensate_terminal_los(lam, 0.0, None, 0.0) is lam
    assert compensate_terminal_los(None, 11.0, 1.0, 0.2) is None


def test_compensate_known_angle_and_sign():
    """A known case: lambda_hat = 100 deg with a +11 deg measured bias must be
    commanded as 89 deg (the bias is SUBTRACTED, because it was measured as
    estimate-minus-truth); a -17.8 deg bias must ADD."""
    lam = math.radians(100.0)
    assert abs(math.degrees(compensate_terminal_los(lam, 11.0)) - 89.0) < 1e-9
    assert abs(math.degrees(compensate_terminal_los(lam, -17.8)) - 117.8) < 1e-9
    # end-to-end through the resolver, both crossing directions:
    for csign, want in ((1, 100.0 - 11.0), (-1, 100.0 + 17.8)):
        b = resolve_terminal_bearing_bias_deg(csign, 0.0, 11.0, -17.8)
        assert abs(math.degrees(compensate_terminal_los(lam, b)) - want) < 1e-9


def test_compensate_wraps_across_pi():
    """Near +-180 deg the correction must WRAP, not run off the branch cut --
    the canonical l2r terminal sits at lambda ~ +150..+175 deg."""
    lam = math.radians(175.0)
    out = math.degrees(compensate_terminal_los(lam, -17.8))   # 175 + 17.8 = 192.8
    assert -180.0 < out <= 180.0, out
    assert abs(out - (-167.2)) < 1e-9, out


def test_compensate_lag_lead_self_signs():
    """The lag term LEADS the estimate by lag_s * lambda_dot and needs no
    direction key: an l2r terminal (lambda_dot < 0) is pushed negative, an r2l
    one (lambda_dot > 0) positive -- automatically the opposite signs the raw
    bias shows. 190 ms x -75 deg/s ~ -14.3 deg (the measured l2r case)."""
    lam = math.radians(100.0)
    l2r = math.degrees(compensate_terminal_los(lam, 0.0, math.radians(-75.0), 0.190))
    r2l = math.degrees(compensate_terminal_los(lam, 0.0, math.radians(+42.0), 0.190))
    assert abs(l2r - (100.0 - 0.190 * 75.0)) < 1e-9, l2r
    assert abs(r2l - (100.0 + 0.190 * 42.0)) < 1e-9, r2l
    # a missing rate estimate must be inert, not a crash
    assert compensate_terminal_los(lam, 0.0, None, 0.190) is lam


def test_compensate_bias_and_lag_compose():
    """Both knobs together: lambda - bias + lag*lambda_dot, one wrap."""
    lam = math.radians(100.0)
    got = math.degrees(compensate_terminal_los(lam, 11.0, math.radians(-75.0), 0.190))
    assert abs(got - (100.0 - 11.0 - 0.190 * 75.0)) < 1e-9, got


def test_compensate_rotates_the_command_frame_by_the_bias():
    """The POINT of the correction: the terminal builds v = v_close*u + v_perp*p
    from (cos lambda, sin lambda), so removing a +11 deg LOS bias rotates the
    commanded velocity vector by exactly 11 deg -- and at 4 m/s over 1 s that is
    the ~0.8 m of induced miss the flown arms measured."""
    lam = math.radians(100.0)
    v_close, v_perp, dt = 4.0, 1.0, 1.0
    def cmd(l):
        u = (math.cos(l), math.sin(l))
        p = (-math.sin(l), math.cos(l))
        return (v_close * u[0] + v_perp * p[0], v_close * u[1] + v_perp * p[1])
    a = cmd(lam)
    b = cmd(compensate_terminal_los(lam, 11.0))
    ang = math.degrees(math.atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1]))
    assert abs(ang - (-11.0)) < 1e-9, ang                       # rotated by -11 deg
    assert abs(math.hypot(*a) - math.hypot(*b)) < 1e-12         # speed preserved
    disp = math.hypot(a[0] - b[0], a[1] - b[1]) * dt
    assert 0.7 < disp < 0.9, disp                               # ~0.79 m at 4.12 m/s


ALL = [test_flight0_lead, test_stationary_aims_at_target, test_head_on_aims_north,
       test_uncatchable_falls_back_to_initial, test_lr_rl_mirror_symmetry,
       test_explicit_origin_offset, test_closing_speed_floor_and_measured,
       test_pronav_law, test_dash_forward_speed_default_byte_identical,
       test_dash_forward_speed_ramps_and_clamps,
       test_dash_loft_alt_ref_default_byte_identical,
       test_dash_loft_alt_ref_dives_from_loft_to_base,
       test_crossing_sign_matches_the_flown_dash_bias_key,
       test_crossing_sign_degenerate_is_zero,
       test_resolve_bias_symmetric_sign_keying,
       test_resolve_bias_per_direction_overrides_and_asymmetry,
       test_compensate_default_is_exact_identity,
       test_compensate_known_angle_and_sign,
       test_compensate_wraps_across_pi,
       test_compensate_lag_lead_self_signs,
       test_compensate_bias_and_lag_compose,
       test_compensate_rotates_the_command_frame_by_the_bias]

if __name__ == "__main__":
    import sys
    failed = 0
    for t in ALL:
        try:
            t()
            print(f"PASS {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {t.__name__}: {e}")
    print(f"{len(ALL) - failed}/{len(ALL)} guidance tests passed")
    sys.exit(1 if failed else 0)
