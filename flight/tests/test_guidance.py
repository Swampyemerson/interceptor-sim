"""Pin flight.guidance.collision_lead_heading: the coded-dash aim. Covers the
measured flight-0 case, stationary/head-on/uncatchable branches, and the
l2r<->r2l mirror symmetry (ADR-0076: the guidance is provably symmetric). Run
standalone (exit 0/1) or under pytest.
"""

import math

import os

from flight.guidance import (
    collision_lead_heading,
    collision_lead_heading_accel,
    dash_ramp_distance,
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


# --- ACCEL-AWARE COLLISION LEAD (docs/flight_plan_candidates.md Sec 1.3 +
#     the G20dash RESULT). ---------------------------------------------------

# The canonical line-9 seed-123 geometries the flown arms use: target start
# (east 6.5, north start_y) and velocity (0, vy).  Extracted from the flown
# batch CSVs (logs/mc_fp_armG20dash_line9_s123.csv == the same 8 in
# logs/mc_loftdive_armAdash_line9_s123.csv), so tests are PAIRED with the runs.
CANON_X0 = 6.5
CANON_DASH_SPEED = 16.0
CANON_GEOM = ((-16.686, 9.0), (16.150, -9.0), (-16.520, 9.0), (16.467, -9.0),
              (-15.234, 9.0), (14.865, -9.0), (-16.364, 9.0), (15.526, -9.0))
# Fitted effective accel of the UNCAPPED dash (scripts/experiments/flight_plans/
# dash_cpa_model.py --validate against the dash-only ARM A control).
CANON_FIT_ACCEL = 10.0


def _bias_equivalent_deg(start_y, vy, accel, dash_speed=CANON_DASH_SPEED):
    """How much crossing bias the accel-aware lead supplies by itself, in the
    EXACT sign convention scripts/m4_intercept.py's --dash-crossing-bias-deg
    uses (h -= copysign(bias, dash x Vt))."""
    pos, vel = (CANON_X0, start_y), (0.0, vy)
    h_const, _ = collision_lead_heading(pos, vel, dash_speed)
    h_acc, _ = collision_lead_heading_accel(pos, vel, dash_speed, accel)
    return (h_const - h_acc) * crossing_sign(h_const, vel)


def _ballistic_cpa(heading_deg, accel, start, vel, dash_speed=CANON_DASH_SPEED,
                   t_max=8.0, dt=0.0005):
    """SLOW REFERENCE (independent of the solver): closest approach of a
    fixed-heading ramped dash from the origin to a constant-velocity target,
    by brute-force time sampling.  Mirrors scripts/experiments/flight_plans/
    dash_cpa_model.py's model."""
    hx, hy = math.sin(math.radians(heading_deg)), math.cos(math.radians(heading_deg))
    best, t = float("inf"), 0.0
    while t <= t_max:
        s = dash_ramp_distance(dash_speed, accel, t)
        best = min(best, math.hypot(s * hx - (start[0] + vel[0] * t),
                                    s * hy - (start[1] + vel[1] * t)))
        t += dt
    return best


def test_dash_ramp_distance_is_the_integral_of_dash_forward_speed():
    """The closed form must BE the integral of the flown ramp function -- that
    is what ties the aim to the actual commanded speed profile."""
    for v, a, t in ((16.0, 10.0, 0.7), (16.0, 3.57, 2.0), (16.0, 10.0, 3.0),
                    (16.0, 4.0, 4.0), (16.0, 12.0, 1.0)):
        n = 20000
        num = sum(dash_forward_speed(v, a, (i + 0.5) * t / n) for i in range(n)) * (t / n)
        assert abs(dash_ramp_distance(v, a, t) - num) < 1e-6, (v, a, t)
    # no ramp (accel None/<=0) -> the constant-speed distance; t<=0 -> 0
    assert dash_ramp_distance(16.0, None, 2.0) == 32.0
    assert dash_ramp_distance(16.0, 0.0, 2.0) == 32.0
    assert dash_ramp_distance(16.0, 10.0, 0.0) == 0.0
    assert dash_ramp_distance(16.0, 10.0, -1.0) == 0.0
    # clamp: a=10 reaches 16 m/s at 1.6 s; after that the speed is constant
    assert abs(dash_ramp_distance(16.0, 10.0, 1.6) - 0.5 * 10 * 1.6 ** 2) < 1e-12
    assert abs(dash_ramp_distance(16.0, 10.0, 2.6)
               - (dash_ramp_distance(16.0, 10.0, 1.6) + 16.0)) < 1e-12


def test_accel_lead_infinite_accel_limit():
    """The reduction that makes this a strict generalisation: an INSTANTANEOUS
    ramp is the constant-speed triangle.  accel None/<=0 delegates (exact, bit
    for bit); a large finite accel converges to it."""
    for pos, vel in (((6.5, -16.686), (0.0, 9.0)), ((6.5, 16.150), (0.0, -9.0)),
                     ((0.0, 20.0), (0.0, -5.0))):
        h_c, t_c = collision_lead_heading(pos, vel, 16.0)
        for a in (None, 0.0, -3.0):
            h_a, t_a = collision_lead_heading_accel(pos, vel, 16.0, a)
            assert h_a == h_c and t_a == t_c, (a, h_a, h_c)      # EXACT
        for a, tol in ((1e4, 1e-1), (1e6, 1e-3), (1e8, 1e-5)):
            h_a, t_a = collision_lead_heading_accel(pos, vel, 16.0, a)
            assert abs(h_a - h_c) < tol, (a, h_a, h_c)
            assert abs(t_a - t_c) < tol, (a, t_a, t_c)


def test_accel_lead_hand_solved_cases():
    """Two cases solved BY HAND (closed form), head-on and crossing.

    (1) a=4, v_max huge, target (0,20) closing south at 6 m/s.  At t=2 s the
        ramp has covered 0.5*4*4 = 8 m and the target is at (0,8): an exact
        intercept due north at t=2 -- and it is the FIRST root (f<0 before).
    (2) a=8, v_max huge, target (0,10) crossing east at 3 m/s.  |R(t)| = s(t)
        gives 16t^4 - 9t^2 - 100 = 0 -> t^2 = (9 + sqrt(81+6400))/32, so the
        lead point is (3t, 10) and the heading is atan2(3t, 10)."""
    h, t = collision_lead_heading_accel((0.0, 20.0), (0.0, -6.0), 1e3, 4.0)
    assert abs(t - 2.0) < 1e-9, t
    assert abs(h - 0.0) < 1e-9, h
    t2 = math.sqrt((9.0 + math.sqrt(81.0 + 6400.0)) / 32.0)
    h2, t2_got = collision_lead_heading_accel((0.0, 10.0), (3.0, 0.0), 1e3, 8.0)
    assert abs(t2_got - t2) < 1e-9, (t2_got, t2)
    assert abs(h2 - math.degrees(math.atan2(3.0 * t2, 10.0))) < 1e-9, h2
    # and the hand cases are NOT what the constant-speed solve returns
    assert abs(collision_lead_heading((0.0, 10.0), (3.0, 0.0), 1e3)[0] - h2) > 1.0


def test_accel_lead_hits_where_the_constant_speed_lead_misses():
    """MECHANISM: fly each solved heading through the independent brute-force
    ballistic model at the SAME accel.  The accel-aware aim intercepts (CPA ~ 0);
    the constant-speed aim -- the one the flown arms used -- misses by metres,
    which is the aim error the hand-tuned crossing bias was absorbing."""
    worst_acc, best_const = 0.0, float("inf")
    for start_y, vy in CANON_GEOM:
        start, vel = (CANON_X0, start_y), (0.0, vy)
        h_a, _ = collision_lead_heading_accel(start, vel, CANON_DASH_SPEED,
                                              CANON_FIT_ACCEL)
        h_c, _ = collision_lead_heading(start, vel, CANON_DASH_SPEED)
        worst_acc = max(worst_acc, _ballistic_cpa(h_a, CANON_FIT_ACCEL, start, vel))
        best_const = min(best_const, _ballistic_cpa(h_c, CANON_FIT_ACCEL, start, vel))
    assert worst_acc < 0.02, worst_acc            # exact intercept by construction
    assert best_const > 2.0, best_const           # every constant-speed aim misses


def test_accel_lead_matches_the_slow_reference_minimiser():
    """The solver's heading must also be the CPA-minimising heading found by a
    dense independent scan (0.02 deg grid) -- not just a root of its own solve."""
    for start_y, vy in ((-16.686, 9.0), (16.150, -9.0)):
        start, vel = (CANON_X0, start_y), (0.0, vy)
        h_a, _ = collision_lead_heading_accel(start, vel, CANON_DASH_SPEED,
                                              CANON_FIT_ACCEL)
        best_h, best_d = None, float("inf")
        for i in range(-250, 251):                # +-5 deg at 0.02 deg
            hh = h_a + 0.02 * i
            d = _ballistic_cpa(hh, CANON_FIT_ACCEL, start, vel, dt=0.002)
            if d < best_d:
                best_h, best_d = hh, d
        assert abs(best_h - h_a) < 0.15, (best_h, h_a)


def test_accel_lead_monotone_toward_the_constant_speed_answer():
    """MONOTONICITY: the more accel, the less extra lead is needed, and in the
    limit the accel-aware answer IS the constant-speed one.  (This is why the
    hand-tuned --dash-crossing-bias-deg is an ACCELERATION-dependent constant.)"""
    for start_y, vy in ((-16.686, 9.0), (16.150, -9.0)):
        biases = [_bias_equivalent_deg(start_y, vy, a)
                  for a in (3.57, 4.5, 5.66, 7.0, 9.0, 10.0, 12.0, 16.0, 20.0, 1e4)]
        assert all(b > 0.0 for b in biases[:-1]), biases      # always MORE lead
        assert all(x > y for x, y in zip(biases, biases[1:])), biases
        assert biases[-1] < 0.05, biases[-1]                  # -> 0 at huge accel


def test_accel_lead_reproduces_the_confirmed_20deg_bias():
    """THE OFFLINE SANITY NUMBER.  On the 8 canonical seed-123 geometries at the
    fitted baseline accel (10 m/s^2), the accel-aware lead supplies +20.3 deg of
    crossing bias all by itself -- i.e. it DERIVES the +20 deg that the G20dash
    arm CONFIRMED empirically (combined median miss 1.37 m at the flown 30 deg
    -> 0.75 m at 20 deg, 7/8 paired better), and NOT the mis-sized +30.

    It also reproduces the two other rows of dash_cpa_model.py --sweep: ~16 deg
    at PX4's commanded 12 m/s^2 ceiling, ~68 deg at the ARM B cap of 3.57."""
    b10 = [_bias_equivalent_deg(sy, vy, 10.0) for sy, vy in CANON_GEOM]
    mean10 = sum(b10) / len(b10)
    assert 19.0 < mean10 < 22.0, mean10
    assert abs(mean10 - 20.32) < 0.1, mean10          # pinned value
    assert all(18.0 < b < 23.5 for b in b10), b10     # per-flight spread
    assert abs(sum(_bias_equivalent_deg(sy, vy, 12.0)
                   for sy, vy in CANON_GEOM) / 8.0 - 16.35) < 0.1
    assert abs(sum(_bias_equivalent_deg(sy, vy, 3.57)
                   for sy, vy in CANON_GEOM) / 8.0 - 68.32) < 0.1
    # the flown +30 deg is NOT what the baseline accel asks for
    assert mean10 < 25.0, mean10


def test_accel_lead_mirror_symmetry_and_fallback():
    """Same invariants the constant-speed lead is pinned to: mirroring across
    the north axis flips the heading exactly, and an uncatchable target falls
    back to aiming at its initial position with t_lead None."""
    hl, tl = collision_lead_heading_accel((6.5, -15.343), (0.0, 12.0), 16.0, 10.0)
    hr, tr = collision_lead_heading_accel((-6.5, -15.343), (0.0, 12.0), 16.0, 10.0)
    assert abs(hl + hr) < 1e-9, (hl, hr)
    assert abs(tl - tr) < 1e-12, (tl, tr)
    h, t = collision_lead_heading_accel((0.0, 10.0), (0.0, 20.0), 10.0, 10.0)
    assert t is None and abs(h - 0.0) < 1e-9, (h, t)
    # stationary target: pure ramp range solve, 0.5*a*t^2 = 10 -> t = sqrt(2)
    h, t = collision_lead_heading_accel((0.0, 10.0), (0.0, 0.0), 100.0, 10.0)
    assert abs(h) < 1e-9 and abs(t - math.sqrt(2.0)) < 1e-9, (h, t)


def test_m4_no_flag_aim_is_byte_identical():
    """The flag is OPT-IN: with --dash-accel-aware-lead absent, m4's aim block
    must still make the SAME constant-speed call and get the SAME headings.

    Pinned two ways: (a) the exact float headings the flag-off path produces on
    the 8 canonical seed-123 geometries (regression pins captured from the
    pre-change code), and (b) the flag-off branch of scripts/m4_intercept.py
    still containing the verbatim original call."""
    pins = (146.93541781404898, 34.04722024241256, 146.63687802249333,
            33.45951532978071, 144.13999640537585, 36.6423369189113,
            146.35164073083027, 35.26277678623344)
    for (start_y, vy), want in zip(CANON_GEOM, pins):
        h, _ = collision_lead_heading((CANON_X0, start_y), (0.0, vy),
                                      CANON_DASH_SPEED)
        assert h == want, (start_y, vy, h, want)
        # the accel-aware path is a REAL change when it IS enabled
        h_a, _ = collision_lead_heading_accel((CANON_X0, start_y), (0.0, vy),
                                              CANON_DASH_SPEED, CANON_FIT_ACCEL)
        assert abs(h_a - h) > 15.0, (h_a, h)
    src = open(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))), "scripts", "m4_intercept.py")).read()
    flat = " ".join(src.split())
    # UPDATED 2026-08-10 (wind arm wiring). The constant-speed call is now made
    # THROUGH wind_trim.solve_wind_trimmed_lead, whose DISABLED path is an exact
    # identity: it calls the injected solver exactly once, with the nominal
    # speed, and returns its heading untouched. So the flag-off aim is still
    # byte-identical -- the float pins above are unchanged and still pass -- but
    # the verbatim text moved. The identity itself is pinned behaviourally in
    # tests/test_wind_wiring.py (test_disabled_trim_calls_the_lead_solver_exactly_once
    # and test_explicit_zero_trim_is_identical_to_the_default); this stays a
    # source pin so that swapping in a DIFFERENT solver wrapper still trips here.
    assert ("else: coded_dash_heading_deg, _, _wind_trim_result = "
            "solve_wind_trimmed_lead( collision_lead_heading, "
            "(_tx + args.dash_target_err_e, _ty + args.dash_target_err_n), "
            "(_tvx, _tvy), _vi, _wind_trim)") in flat, "m4 flag-off aim call changed"
    # ...and the wrapper must be the one whose disabled path is the identity.
    assert "_wind_trim = build_wind_trim(args)" in flat


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
       test_dash_ramp_distance_is_the_integral_of_dash_forward_speed,
       test_accel_lead_infinite_accel_limit,
       test_accel_lead_hand_solved_cases,
       test_accel_lead_hits_where_the_constant_speed_lead_misses,
       test_accel_lead_matches_the_slow_reference_minimiser,
       test_accel_lead_monotone_toward_the_constant_speed_answer,
       test_accel_lead_reproduces_the_confirmed_20deg_bias,
       test_accel_lead_mirror_symmetry_and_fallback,
       test_m4_no_flag_aim_is_byte_identical,
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
