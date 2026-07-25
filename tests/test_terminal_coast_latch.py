"""REGRESSION PIN: the coded-dash camera terminal must COMMAND SOMETHING.

THE DEFECT (review-2 BLOCKER, found 2026-07-25, fixed the same day).
scripts/m4_intercept.py initialises `vh0 = vh1 = 0.0` at module... loop scope and
latched `frozen_vworld = (vh0, vh1)` inside the terminal-coast branch. The
guidance-law block that assigns vh0/vh1 lived under `if not in_terminal_coast:`,
so when ENGAGE STARTED already inside TERMINAL_FREEZE_RANGE_M -- the NORMAL
`--coded-dash --fpv` case, because the markerless NN reports r_hat ~1.5-3.5 m
against a true ~10-12 m -- the law block never ran once and the latch captured
the never-assigned zeros. The whole ENGAGE phase then re-issued
cmd = (0, 0, v_down, yaw): the interceptor BRAKED instead of steering, on 29 of
44 engaged flights of the camera-arm sweep whose verdicts the project published.

WHY A MUTANT DRILL AND NOT JUST AN ASSERTION. CLAUDE.md's "a fix is not done
until its EFFECT is observed" rule: this file rebuilds the PRE-FIX source by
textually reverting the one-line guard in a COPY of the real
scripts/m4_intercept.py, imports both, and drives the SAME terminal-tick replay
through each. The mutant must latch (0.00, 0.00); the real module must latch a
real closing vector. If someone reverts the guard, `test_mutant_reproduces_the_
zero_command_terminal` keeps passing (that is its job) and every other test in
this file fails.

Offline, no sim. Run:
  .venv/bin/python -m pytest tests/test_terminal_coast_latch.py -v
"""
import importlib.util
import math
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
M4_PATH = os.path.join(SCRIPTS, "m4_intercept.py")
for p in (SCRIPTS, REPO_ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

import m4_intercept as m4                                        # noqa: E402

# The two halves of the fix, as they appear in the live source.
FIXED_RETURN = "return (not in_terminal_coast) or (not vh_cmd_valid)"
PREFIX_RETURN = "return not in_terminal_coast"
# The CALL SITE. Pinned separately because a guard that exists but is not wired
# in is an INERT fix -- the exact error CLAUDE.md's "observe the effect
# end-to-end" rule was written for (2026-07-25, a band declared but never used).
FIXED_CALL = "if terminal_must_build_command(in_terminal_coast, vh_cmd_valid):"


def test_the_guard_is_actually_wired_into_the_flight_loop():
    """NOT INERT: the ENGAGE tick loop must CALL the guard, not merely define it."""
    src = open(M4_PATH).read()
    assert src.count(FIXED_CALL) == 1, (
        "scripts/m4_intercept.py's ENGAGE block no longer calls "
        "terminal_must_build_command() -- the zero-command guard is INERT")
    assert "frozen_vworld, _cz, _msgs = latch_coast_vector(" in src


@pytest.fixture(scope="module")
def mutant(tmp_path_factory):
    """The PRE-FIX module: the real source with the guard's RETURN reverted.

    Built from the live file, never hand-typed -- a hand-typed 'old version' is
    exactly the stale fixture that hid two schema breaks on 2026-07-24."""
    src = open(M4_PATH).read()
    assert src.count(FIXED_RETURN) == 1, (
        "the terminal-coast guard moved or was removed from "
        "scripts/m4_intercept.py -- update this drill WITH it")
    mutated = src.replace(FIXED_RETURN, PREFIX_RETURN)
    path = tmp_path_factory.mktemp("mutant") / "m4_prefix.py"
    path.write_text(mutated)
    spec = importlib.util.spec_from_file_location("m4_prefix", str(path))
    mod = importlib.util.module_from_spec(spec)
    sys.modules["m4_prefix"] = mod
    spec.loader.exec_module(mod)
    return mod


def replay_engage(mod, r_hats, lambda_deg=12.0, freeze_range_m=3.5):
    """Replay the ENGAGE terminal tick loop's COMMAND path using the module's
    OWN decision functions and its OWN closing-speed law.

    Mirrors scripts/m4_intercept.py's ENGAGE block for the pronav/pursuit basis
    with v_perp = 0 (first-tick state): vh = v_close * (cos lambda, sin lambda),
    then the coast latch. Everything that DIFFERS between the two modules comes
    from the module under test, not from this driver.

    Returns (frozen_vworld, commands_per_tick, coast_zero).
    """
    lam = math.radians(lambda_deg)
    vh0 = vh1 = 0.0                 # the module's own initialisers
    vh_cmd_valid = False
    frozen = None
    coast_zero = False
    cmds = []
    for r_hat in r_hats:
        in_coast = mod.terminal_coast_active(frozen, r_hat, freeze_range_m)
        must_build = (mod.terminal_must_build_command(in_coast, vh_cmd_valid)
                      if hasattr(mod, "terminal_must_build_command")
                      else not in_coast)
        if must_build:
            v_close = mod.compute_v_close(r_hat, True)   # --fpv two-speed law
            vh0 = v_close * math.cos(lam)
            vh1 = v_close * math.sin(lam)
            vh_cmd_valid = True
        if in_coast:
            if frozen is None:
                frozen, cz, _ = mod.latch_coast_vector(vh0, vh1, r_hat)
                coast_zero = coast_zero or cz
            vh0, vh1 = frozen
        cmds.append((vh0, vh1))
    return frozen, cmds, coast_zero


# --- the geometry that broke: ENGAGE begins INSIDE the freeze range ---------
# r_hat 1.54 m is the value logged in mc_fp_armG20_line9_s123's own
# "Terminal coast: velocity vector frozen at r_hat=1.54 m ((0.00, 0.00) world)".
HANDOFF_INSIDE_FREEZE = [1.54, 1.40, 1.21, 1.05, 0.90, 0.74]


def test_mutant_reproduces_the_zero_command_terminal(mutant):
    """OLD CODE FAILS: the pre-fix module latches (0, 0) and commands nothing
    for the entire ENGAGE phase. This is the bug, reproduced from real source."""
    frozen, cmds, _ = replay_engage(mutant, HANDOFF_INSIDE_FREEZE)
    assert frozen == (0.0, 0.0)
    assert all(abs(vn) < 1e-9 and abs(ve) < 1e-9 for vn, ve in cmds), cmds


def test_fixed_terminal_commands_real_steering(mutant):
    """NEW CODE PASSES: the same geometry now latches a real closing vector,
    and every ENGAGE tick carries it."""
    frozen, cmds, coast_zero = replay_engage(m4, HANDOFF_INSIDE_FREEZE)
    speed = math.hypot(*frozen)
    assert speed > m4.COAST_ZERO_WARN_MS, frozen
    # The FPV terminal closing speed inside the throttle band (compute_v_close).
    assert speed == pytest.approx(m4.FPV["V_CLOSE_TERMINAL"], rel=1e-6)
    # ... and it points up the measured LOS, i.e. it is CAMERA-driven.
    assert math.degrees(math.atan2(frozen[1], frozen[0])) == pytest.approx(12.0)
    assert all(math.hypot(vn, ve) > m4.COAST_ZERO_WARN_MS for vn, ve in cmds)
    assert coast_zero is False

    # And the mutant, on the identical replay, does the opposite. Both ways,
    # one assertion -- this is the old-fails/new-passes pair.
    mfrozen, mcmds, mzero = replay_engage(mutant, HANDOFF_INSIDE_FREEZE)
    assert math.hypot(*mfrozen) < m4.COAST_ZERO_WARN_MS
    assert math.hypot(*frozen) > 10 * math.hypot(*mfrozen) + 1.0


def test_fix_is_a_noop_when_engage_starts_outside_the_freeze_range(mutant):
    """SCOPE PIN: every AprilTag/S2 arm engaged from ~12 m out, well outside the
    3.5 m freeze range, so vh_cmd_valid was already True at the latch. The fixed
    and pre-fix modules must agree EXACTLY there -- no historical result moves."""
    r_hats = [12.0, 9.5, 7.0, 5.0, 3.9, 3.2, 2.4, 1.6]
    f_new, c_new, z_new = replay_engage(m4, r_hats)
    f_old, c_old, z_old = replay_engage(mutant, r_hats)
    assert f_new == f_old, (f_new, f_old)
    assert c_new == c_old
    assert z_new == z_old is False


def test_coast_active_predicate_truth_table():
    """terminal_coast_active() over the full decision surface. One deliberate
    divergence from the original inline expression (2026-07-25 follow-up):
    a NEGATIVE r_hat must NOT arm -- the alpha-beta range channel coasting
    through zero is a BROKEN track (ADR-0041 measured -12..-21 m post-CPA),
    and flight/deploy/seeker_loop.py never selects the coast branch on it;
    m4 armed off it on 8/104 ENGAGE ticks of the fix-verification flight."""
    for frozen in (None, (1.0, 2.0)):
        for r_hat in (None, -21.0, -1.0, -1e-9, 0.0, 1.6,
                      3.4999, 3.5, 3.5001, 12.0):
            for rng in (2.0, 3.5):
                expected = (frozen is not None
                            or (r_hat is not None and 0.0 <= r_hat < rng))
                assert m4.terminal_coast_active(frozen, r_hat, rng) is bool(expected)


def test_negative_r_hat_never_arms_but_a_set_latch_survives_it():
    """The sim==hw broken-track parity fix, pinned both ways:
    (a) a diverged negative r_hat alone can never start the coast;
    (b) the one-way latch, once legitimately set, is NOT released by a
        subsequent negative r_hat (release semantics unchanged -- only the
        ARM trigger gained the physicality guard)."""
    assert m4.terminal_coast_active(None, -0.5, 3.5) is False
    assert m4.terminal_coast_active(None, -21.0, 3.5) is False
    # replay: track diverges negative BEFORE ever dipping into the freeze
    # range -> no latch, the law keeps commanding live ticks throughout.
    frozen, cmds, coast_zero = replay_engage(m4, [12.0, 6.0, -2.0, -8.0])
    assert frozen is None
    assert len(cmds) == 4 and all(math.hypot(*c) > 0 for c in cmds)
    # a latch set at a REAL in-range tick survives later negative dust.
    assert m4.terminal_coast_active((1.0, 2.0), -3.0, 3.5) is True


def test_must_build_command_truth_table():
    # not coasting -> always build (the pre-fix behaviour, preserved)
    assert m4.terminal_must_build_command(False, False) is True
    assert m4.terminal_must_build_command(False, True) is True
    # coasting with a real command already computed -> hold the latch
    assert m4.terminal_must_build_command(True, True) is False
    # coasting with NO command ever computed -> THE FIX: build one first
    assert m4.terminal_must_build_command(True, False) is True


def test_latch_flags_and_narrates_a_zero_command_coast():
    frozen, coast_zero, msgs = m4.latch_coast_vector(0.0, 0.0, 1.54)
    assert frozen == (0.0, 0.0) and coast_zero is True
    joined = " ".join(msgs)
    assert "COAST-ZERO WARNING" in joined
    assert "BRAKING, NOT STEERING" in joined
    assert "do NOT pool this flight" in joined

    frozen, coast_zero, msgs = m4.latch_coast_vector(5.38, 1.14, 1.54)
    assert coast_zero is False
    assert len(msgs) == 1 and "COAST-ZERO" not in msgs[0]

    # Boundary: exactly at the threshold is NOT flagged (strict <).
    _, cz_at, _ = m4.latch_coast_vector(m4.COAST_ZERO_WARN_MS, 0.0, 1.0)
    assert cz_at is False
    _, cz_below, _ = m4.latch_coast_vector(m4.COAST_ZERO_WARN_MS - 1e-6, 0.0, 1.0)
    assert cz_below is True


def test_the_defect_is_observable_in_the_flight_csv_schema():
    """The fix is only half a fix if the log cannot say it happened: coast_zero
    must be a real producer column, and write_row_m4 must emit it."""
    assert "coast_zero" in m4.CSV_HEADER
    import inspect
    sig = inspect.signature(m4.write_row_m4)
    assert "coast_zero" in sig.parameters
