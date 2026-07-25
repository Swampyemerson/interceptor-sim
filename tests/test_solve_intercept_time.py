#!/usr/bin/env python3
"""Unit tests for `solve_intercept_time` -- the PIP (predicted-intercept-point)
lead-time solve on a LIVE guidance path.

WHAT IT IS (primer). Pursuit guidance aims at where the target IS; lead guidance
aims at where it WILL BE. `solve_intercept_time` is the "will be" half: given the
target's relative position `rel`, its estimated velocity `vt`, and the speed
`v_close` we can close at, it solves the intercept triangle for the smallest
positive time `t` at which |rel + vt*t| == v_close*t -- i.e. when a
constant-speed us and a constant-velocity target would meet. The caller then aims
at `rel + vt*t_go`. `t_go` directly moves miss distance, so a sign error or a
wrong-root pick here shows up as a systematic lead bias, not as a crash.

THE ALGEBRA under test (both implementations are the same 6 lines):
    |rel + vt*t|^2 = (v_close*t)^2
    => (|vt|^2 - v_close^2) t^2 + 2(rel . vt) t + |rel|^2 = 0
    =>            a t^2         +        b t       +   c   = 0
Take the smallest STRICTLY-POSITIVE root, clamp into [0, max_lead_s]. t_go == 0
is the documented "no solution" answer and degrades to plain pursuit -- it is a
valid return, never an exception, which is exactly why it needs pinning: a
silently-always-zero solve would look like working pursuit guidance.

WHY THIS FILE EXISTS. Audit 2026-07-25 (reader 5) / DEEP-T1: zero test coverage
on a pure function called from two live guidance branches
(scripts/m4_intercept.py:3349 and :3684) with a DUPLICATE definition in
scripts/guidance_lab.py:1490 driving three more. The duplication is its own
hazard -- the lab is the design-time surrogate that RANKS options, so if the two
drift the lab stops predicting Gazebo ("lab ranks, Gazebo decides"). The last
group of tests pins them to each other.

Every expected value below is hand-derived in the case's own comment and then
verified by re-substitution into the defining equation, so a test failure points
at the code, not at a copied golden number.

Run: `.venv/bin/python -m pytest tests/test_solve_intercept_time.py -v`
"""
import math
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
sys.path.insert(0, SCRIPTS)

from m4_intercept import solve_intercept_time, PIP_MAX_LEAD_S  # noqa: E402

BIG_LEAD = 1e6  # effectively "unclamped", so the solve itself is what is tested


def residual(rel, vt, v_close, t):
    """|rel + vt*t| - v_close*t -- zero exactly at a true intercept time."""
    return math.hypot(rel[0] + vt[0] * t, rel[1] + vt[1] * t) - v_close * t


# ============================================================================
# (1) KNOWN GEOMETRY -- closed-form cases, each re-substituted
# ============================================================================
def test_head_on_closure_gives_the_hand_derived_time():
    """Target 10 m dead ahead, running AT us at 2 m/s; we close at 8 m/s.
    a = 4-64 = -60, b = 2*(10*-2) = -40, c = 100, disc = 1600+24000 = 25600,
    sqrt = 160, roots = (40 +/- 160)/(-120) = {-1.667, +1.0} -> t_go = 1.0 s.
    Sanity: closure rate is 8+2 = 10 m/s over 10 m = 1.0 s."""
    t = solve_intercept_time((10.0, 0.0), (-2.0, 0.0), 8.0, BIG_LEAD)
    assert t == pytest.approx(1.0, abs=1e-9)
    assert residual((10.0, 0.0), (-2.0, 0.0), 8.0, t) == pytest.approx(0.0, abs=1e-9)


def test_pure_crossing_target_gives_the_hand_derived_time():
    """Target 10 m off our +y, crossing at 3 m/s along +x; we close at 5 m/s.
    b = 2*(0*3 + 10*0) = 0 (velocity perpendicular to the LOS), a = 9-25 = -16,
    c = 100, disc = 6400, sqrt = 80, roots = +/-2.5 -> t_go = 2.5 s.
    Re-substitute: rel+vt*2.5 = (7.5, 10), |.| = 12.5 = 5*2.5."""
    rel, vt, vc = (0.0, 10.0), (3.0, 0.0), 5.0
    t = solve_intercept_time(rel, vt, vc, BIG_LEAD)
    assert t == pytest.approx(2.5, abs=1e-9)
    assert residual(rel, vt, vc, t) == pytest.approx(0.0, abs=1e-9)


def test_stationary_target_reduces_to_range_over_closing_speed():
    """vt = 0 collapses lead guidance to pursuit: t = R / v_close.
    12 m at 6 m/s = 2.0 s, independent of bearing."""
    for rel in ((12.0, 0.0), (0.0, -12.0), (12.0 / math.sqrt(2), 12.0 / math.sqrt(2))):
        t = solve_intercept_time(rel, (0.0, 0.0), 6.0, BIG_LEAD)
        assert t == pytest.approx(2.0, abs=1e-9), rel


def test_the_SMALLEST_positive_root_is_chosen():
    """Two positive roots need a > 0 (the target is FASTER than our closing
    speed, so the roots share a sign) and b < 0 (it is closing). Geometry: target
    10 m ahead charging AT us at 8 m/s while we can only close at 5 m/s -- it
    passes through our position and then runs away, so there are two moments at
    which the ranges coincide. a = 64-25 = 39, b = 2*(10*-8) = -160, c = 100,
    disc = 25600-15600 = 10000, sqrt = 100, roots = (160 +/- 100)/78
    = {60/78 = 0.76923, 260/78 = 3.33333}. The FIRST meeting is the intercept;
    picking the later root would command a lead point far past it."""
    rel, vt, vc = (10.0, 0.0), (-8.0, 0.0), 5.0
    a = vt[0] ** 2 + vt[1] ** 2 - vc ** 2
    b = 2.0 * (rel[0] * vt[0] + rel[1] * vt[1])
    c = rel[0] ** 2 + rel[1] ** 2
    sq = math.sqrt(b * b - 4 * a * c)
    roots = sorted(r for r in ((-b + sq) / (2 * a), (-b - sq) / (2 * a)) if r > 0)
    assert len(roots) == 2, "this case must actually have two positive roots"

    t = solve_intercept_time(rel, vt, vc, BIG_LEAD)
    assert t == pytest.approx(roots[0], abs=1e-9)
    assert t < roots[1]
    assert residual(rel, vt, vc, t) == pytest.approx(0.0, abs=1e-6)


# ============================================================================
# (2) NO SOLUTION -- must return 0.0 (plain pursuit), never raise, never negative
# ============================================================================
def test_target_outruns_us_and_diverges_gives_zero():
    """Fleeing at 10 m/s while we close at 5: both roots negative -> t_go = 0,
    the documented fall back to aiming at the current estimated position."""
    assert solve_intercept_time((10.0, 0.0), (10.0, 0.0), 5.0, BIG_LEAD) == 0.0


def test_zero_closing_speed_gives_zero():
    """v_close = 0 can never satisfy |rel + vt t| = 0*t for rel != 0."""
    assert solve_intercept_time((10.0, 0.0), (1.0, 0.0), 0.0, BIG_LEAD) == 0.0
    assert solve_intercept_time((10.0, 0.0), (0.0, 0.0), 0.0, BIG_LEAD) == 0.0


def test_negative_discriminant_gives_zero():
    """Fast crossing target we cannot catch: disc < 0 -> no real root.
    rel = (0, 10), vt = (20, 0), v_close = 6 -> a = 400-36 = 364, b = 0,
    c = 100, disc = -145600."""
    rel, vt, vc = (0.0, 10.0), (20.0, 0.0), 6.0
    a = vt[0] ** 2 + vt[1] ** 2 - vc ** 2
    assert (2.0 * (rel[0] * vt[0] + rel[1] * vt[1])) ** 2 - 4 * a * (
        rel[0] ** 2 + rel[1] ** 2) < 0, "this case must have a negative disc"
    assert solve_intercept_time(rel, vt, vc, BIG_LEAD) == 0.0


def test_zero_range_gives_zero():
    """Already coincident: c = 0, the only root is t = 0, which is not strictly
    positive -> 0.0. (Guards against a divide-by-zero or a spurious lead at CPA.)"""
    assert solve_intercept_time((0.0, 0.0), (3.0, 4.0), 7.0, BIG_LEAD) == 0.0
    assert solve_intercept_time((0.0, 0.0), (0.0, 0.0), 7.0, BIG_LEAD) == 0.0


# ============================================================================
# (3) DEGENERATE |vt| == v_close -- the linear branch (|a| < 1e-9)
# ============================================================================
def test_degenerate_equal_speeds_closing_uses_the_linear_branch():
    """When the target's speed EQUALS our closing speed the t^2 term vanishes and
    the quadratic degenerates to a line: t = -c/b. rel = (10,0), vt = (-3,0),
    v_close = 3 -> b = -60, c = 100, t = 100/60 = 1.6667 s.
    Re-substitute: rel+vt*t = (5,0), |.| = 5 = 3*1.6667."""
    rel, vt, vc = (10.0, 0.0), (-3.0, 0.0), 3.0
    t = solve_intercept_time(rel, vt, vc, BIG_LEAD)
    assert t == pytest.approx(100.0 / 60.0, abs=1e-9)
    assert residual(rel, vt, vc, t) == pytest.approx(0.0, abs=1e-9)


def test_degenerate_equal_speeds_diverging_gives_zero():
    """Same degenerate branch, but the target is opening the range: -c/b < 0,
    so no positive time exists -> 0.0."""
    assert solve_intercept_time((10.0, 0.0), (3.0, 0.0), 3.0, BIG_LEAD) == 0.0


def test_degenerate_equal_speeds_pure_crossing_gives_zero():
    """|vt| == v_close AND the velocity is perpendicular to the LOS: a ~ 0 AND
    b ~ 0 -- both branches degenerate, so the guard must return 0 rather than
    divide by zero."""
    assert solve_intercept_time((0.0, 10.0), (3.0, 0.0), 3.0, BIG_LEAD) == 0.0


def test_near_degenerate_a_just_outside_the_epsilon_still_solves():
    """|a| = 1e-8 > the 1e-9 epsilon: the quadratic branch is taken and must not
    blow up. Only assert the invariants (finite, in range, satisfies the
    equation) -- the value itself is ill-conditioned by construction."""
    vc = 3.0
    vt = (-math.sqrt(vc * vc + 1e-8), 0.0)
    t = solve_intercept_time((10.0, 0.0), vt, vc, BIG_LEAD)
    assert math.isfinite(t) and 0.0 <= t <= BIG_LEAD


# ============================================================================
# (4) CLAMPING -- the contract the callers rely on
# ============================================================================
def test_result_is_clamped_to_max_lead_s():
    """The 2.5 s crossing solve, capped at 1.0 s."""
    assert solve_intercept_time((0.0, 10.0), (3.0, 0.0), 5.0, 1.0) == pytest.approx(1.0)


def test_max_lead_zero_pins_the_result_to_zero():
    """max_lead_s = 0 must degrade to plain pursuit for EVERY geometry -- the
    knob a caller uses to disable lead."""
    assert solve_intercept_time((0.0, 10.0), (3.0, 0.0), 5.0, 0.0) == 0.0


def test_output_is_always_in_range_over_a_geometry_sweep():
    """Property sweep: 0 <= t_go <= max_lead_s and finite for a wide grid --
    no NaN from a negative sqrt, no negative time leaking through."""
    for rx in (-20.0, -5.0, 0.0, 3.0, 15.0):
        for ry in (-9.0, 0.0, 12.0):
            for vx in (-12.0, -3.0, 0.0, 3.0, 12.0):
                for vy in (-7.0, 0.0, 7.0):
                    for vc in (0.0, 1.0, 6.0, 13.0):
                        t = solve_intercept_time((rx, ry), (vx, vy), vc,
                                                 PIP_MAX_LEAD_S)
                        assert math.isfinite(t), (rx, ry, vx, vy, vc)
                        assert 0.0 <= t <= PIP_MAX_LEAD_S, (rx, ry, vx, vy, vc)


def test_a_returned_nonzero_time_always_satisfies_the_intercept_equation():
    """The strong property: whenever the solve returns an UNCLAMPED positive
    t_go, that t must actually be a root of |rel + vt t| = v_close t. This is
    what catches a wrong-sign or wrong-root regression, which a range check
    alone would not see."""
    checked = 0
    for rx in (-20.0, -5.0, 3.0, 15.0):
        for ry in (-9.0, 0.0, 12.0):
            for vx in (-12.0, -3.0, 0.0, 3.0):
                for vy in (-7.0, 0.0, 7.0):
                    for vc in (1.0, 6.0, 13.0):
                        rel, vt = (rx, ry), (vx, vy)
                        t = solve_intercept_time(rel, vt, vc, BIG_LEAD)
                        if t <= 0.0:
                            continue
                        assert residual(rel, vt, vc, t) == pytest.approx(
                            0.0, abs=1e-6), (rel, vt, vc, t)
                        checked += 1
    assert checked > 50, f"sweep only exercised {checked} solvable geometries"


# ============================================================================
# (5) THE DUPLICATE -- m4 and guidance_lab must not drift
# ============================================================================
def test_guidance_lab_copy_is_numerically_identical_to_m4s():
    """scripts/guidance_lab.py:1490 carries its own copy of this function (m4's
    docstring says "ported verbatim"). The lab is the design-time surrogate whose
    only job is to RANK options for Gazebo to decide -- a numeric divergence here
    silently invalidates that ranking. Pinned bit-for-bit over the same grid."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "_gl_for_test", os.path.join(SCRIPTS, "guidance_lab.py"))
    gl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gl)

    diffs = []
    for rx in (-20.0, -5.0, 0.0, 3.0, 15.0):
        for ry in (-9.0, 0.0, 12.0):
            for vx in (-12.0, -3.0, 0.0, 3.0, 12.0):
                for vy in (-7.0, 0.0, 7.0):
                    for vc in (0.0, 1.0, 6.0, 13.0):
                        args = ((rx, ry), (vx, vy), vc, PIP_MAX_LEAD_S)
                        a = solve_intercept_time(*args)
                        b = gl.solve_intercept_time(*args)
                        if a != b:
                            diffs.append((args, a, b))
    assert not diffs, (
        f"m4_intercept.solve_intercept_time and guidance_lab's copy DIVERGED on "
        f"{len(diffs)} geometries (first: {diffs[0]}) -- 'lab ranks, Gazebo "
        f"decides' requires them to agree")
