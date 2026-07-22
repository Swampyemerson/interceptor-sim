"""Pin flight.guidance.collision_lead_heading: the coded-dash aim. Covers the
measured flight-0 case, stationary/head-on/uncatchable branches, and the
l2r<->r2l mirror symmetry (ADR-0076: the guidance is provably symmetric). Run
standalone (exit 0/1) or under pytest.
"""

import math

from flight.guidance import (
    collision_lead_heading,
    closing_speed,
    pronav_lateral_accel,
    dash_forward_speed,
    dash_loft_alt_ref,
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


ALL = [test_flight0_lead, test_stationary_aims_at_target, test_head_on_aims_north,
       test_uncatchable_falls_back_to_initial, test_lr_rl_mirror_symmetry,
       test_explicit_origin_offset, test_closing_speed_floor_and_measured,
       test_pronav_law, test_dash_forward_speed_default_byte_identical,
       test_dash_forward_speed_ramps_and_clamps,
       test_dash_loft_alt_ref_default_byte_identical,
       test_dash_loft_alt_ref_dives_from_loft_to_base]

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
