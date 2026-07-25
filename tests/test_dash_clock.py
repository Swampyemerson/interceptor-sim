"""REGRESSION PIN: the coded dash schedules on SIM time, never wall time.

THE RULE (CLAUDE.md's oldest standing methodology rule, ADR-0009): Gazebo's RTF
sags to ~0.3-0.85 under load, so anything SCHEDULED or MEASURED in wall time is
silently distorted. It cost M4 five dev runs and two failed gates in 2026-06.

THE DEFECT (review-2, verified 2026-07-25). `sim_clock` was constructed only
under `if args.handoff:`, and parse_args() makes --coded-dash and --handoff
MUTUALLY EXCLUSIVE. So on every coded-dash flight ever flown sim_clock was None,
`coded_dash_start_sim` never populated, the already-written sim branch was
UNREACHABLE, and all three dash schedules ran on `time.monotonic()`:
  * --dash-accel-cap   (the speed ramp that sets the body pitch the wedge is
                        sized to)
  * --dash-loft-dive-s (the raised-cosine dive)
  * --coded-dash-max-s (the acquisition / abort window)
Measured on the flown ARM B logs (RTF 0.80-0.84): a commanded 3.57 m/s^2 cap
actually flew ~4.35 m/s^2 in sim seconds -- 23.9 deg of body pitch, not the
designed 20.0 -- and a 2.5 s dive completed in ~2.05 sim s. Nothing warned, and
`t_sim` was blank on all 167 rows checked, so the logs could not self-identify
which clock produced them.

Offline, no sim. Run: .venv/bin/python -m pytest tests/test_dash_clock.py -v
"""
import inspect
import os
import re
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
M4_PATH = os.path.join(SCRIPTS, "m4_intercept.py")
for p in (SCRIPTS, REPO_ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

import m4_intercept as m4                                        # noqa: E402
from flight.guidance import dash_forward_speed, dash_loft_alt_ref  # noqa: E402


# --- the pure clock selector ------------------------------------------------

def test_sim_clock_is_preferred_and_named():
    elapsed, basis = m4.dash_elapsed_s(sim_now=14.25, sim_start=10.00,
                                       wall_now=1000.0, wall_start=990.0)
    assert basis == m4.DASH_CLOCK_SIM
    assert elapsed == pytest.approx(4.25)
    # The WALL delta here is 10.0 s -- i.e. RTF 0.425. The sim answer must NOT
    # be contaminated by it.
    assert elapsed != pytest.approx(10.0)


def test_wall_is_used_only_as_a_named_fallback():
    for sim_now, sim_start in ((None, None), (14.25, None), (None, 10.0)):
        elapsed, basis = m4.dash_elapsed_s(sim_now, sim_start, 1000.0, 990.0)
        assert basis == m4.DASH_CLOCK_WALL, (sim_now, sim_start)
        assert elapsed == pytest.approx(10.0)


def test_no_clock_at_all_returns_zero_not_a_crash_and_still_says_wall():
    assert m4.dash_elapsed_s(None, None, None, None) == (0.0, m4.DASH_CLOCK_WALL)


def test_the_rtf_distortion_this_pins_is_real():
    """The quantity the bug corrupted, in the units the wedge is sized in.
    RTF 0.82 (measured on logs/mc_loftdive_armB_line9_s123.csv): 2.5 wall
    seconds of ramp is only 2.05 sim seconds, so the commanded 3.57 m/s^2 cap
    delivers the speed a 4.35 m/s^2 cap should have."""
    rtf = 0.82
    wall_elapsed = 2.5
    sim_elapsed = wall_elapsed * rtf
    cap = 3.57
    v_wall = dash_forward_speed(16.0, cap, wall_elapsed)   # what the bug did
    v_sim = dash_forward_speed(16.0, cap, sim_elapsed)     # what was designed
    assert v_wall > v_sim
    assert v_wall / max(sim_elapsed, 1e-9) == pytest.approx(cap / rtf, rel=1e-6)
    # The dive, same story: a 2.5 s wall dive is complete when only 2.05 sim
    # seconds of the raised cosine have elapsed.
    assert dash_loft_alt_ref(12.0, 2.0, wall_elapsed, 2.5) == pytest.approx(12.0)
    assert dash_loft_alt_ref(12.0, 2.0, sim_elapsed, 2.5) > 12.0


# --- the loud warning -------------------------------------------------------

def test_wall_basis_warns_loudly_exactly_once():
    out = []
    warned = m4.warn_dash_clock_basis(m4.DASH_CLOCK_WALL, False, True,
                                      printer=lambda *a, **k: out.append(a[0]))
    assert warned is True and len(out) == 1
    msg = out[0]
    assert "WALL-CLOCK WARNING" in msg and "ADR-0009" in msg
    assert "dash_clock=wall" in msg
    # One-shot: a second call with the returned flag must stay silent.
    warned2 = m4.warn_dash_clock_basis(m4.DASH_CLOCK_WALL, warned, True,
                                       printer=lambda *a, **k: out.append(a[0]))
    assert warned2 is True and len(out) == 1


def test_sim_basis_never_warns():
    out = []
    assert m4.warn_dash_clock_basis(m4.DASH_CLOCK_SIM, False, True,
                                    printer=lambda *a, **k: out.append(a[0])) is False
    assert out == []


def test_warning_fires_even_with_no_pointing_levers():
    """--coded-dash-max-s always schedules on this basis, so the warning is not
    conditional on the cap/loft flags -- only its wording is."""
    out = []
    m4.warn_dash_clock_basis(m4.DASH_CLOCK_WALL, False, False,
                             printer=lambda *a, **k: out.append(a[0]))
    assert len(out) == 1 and "coded-dash-max-s" in out[0]


# --- the wiring: the fix must not be inert ----------------------------------

def test_the_clock_is_actually_subscribed_under_coded_dash():
    """THE ROOT CAUSE. `if args.handoff:` alone made the sim branch unreachable
    for the one mode that needed it."""
    src = open(M4_PATH).read()
    assert "if args.handoff or args.coded_dash:" in src, (
        "scripts/m4_intercept.py no longer subscribes to /clock under "
        "--coded-dash: every dash schedule is back on wall time (ADR-0009)")
    assert not re.search(r"^    if args\.handoff:\n        sim_clock = SimClockHolder",
                         src, re.M)


def test_all_three_dash_schedules_share_one_basis():
    """The levers AND the abort window must read the same elapsed value; the
    pre-fix code computed the abort window separately, off wall monotonic."""
    src = open(M4_PATH).read()
    assert "coded_dash_elapsed = _dash_elapsed" in src
    assert "coded_dash_elapsed = tick_start - coded_dash_start_mono" not in src
    # ... and the levers are fed from the same variable.
    assert "dash_forward_speed(coded_dash_speed, args.dash_accel_cap,\n" \
           "                                         _dash_elapsed)" in src


def test_every_log_self_identifies_its_clock():
    """The generalizable half: a future log must not need a human to notice a
    blank t_sim column."""
    assert "dash_clock" in m4.CSV_HEADER
    assert "coast_zero" in m4.CSV_HEADER
    assert "dash_clock" in inspect.signature(m4.write_row_m4).parameters
    src = open(M4_PATH).read()
    assert "dash_clock=dash_clock_basis if phase == \"CODED_DASH\" else None" in src
    # ... and the RESULT line carries it too, so batch stdout is enough.
    assert "dash_clock={result.get('dash_clock') or 'n/a'}" in src


def test_warn_helper_is_called_from_the_dash_block():
    src = open(M4_PATH).read()
    assert "dash_clock_warned = warn_dash_clock_basis(" in src, (
        "the wall-clock warning is defined but never called -- an INERT fix")
