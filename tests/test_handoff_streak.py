#!/usr/bin/env python3
"""DEEP-H2 -- interview-proof unit tests for the coded-dash -> camera-terminal
HANDOFF state machine (the last honesty-hardening remainder, NEXT.md THE PLAN #5).

WHAT THE HANDOFF IS. In the real-build architecture the interceptor flies an
OPEN-LOOP coded dash (fixed heading + speed, no cue, no datalink) and hands the
flight to the CAMERA-ONLY pro-nav terminal the instant the onboard detector
holds a solid streak. THE STREAK *IS* THE HANDOFF: 5 consecutive FRESH in-window
camera detections (CODED_DASH_ACQUIRE_STREAK, scripts/m4_intercept.py:312) flip
the phase CODED_DASH -> ENGAGE (m4_intercept.py:2867-2871). There is no cue to
validate against -- so the streak's exact contract is the whole safety story.

WHERE THE LOGIC LIVES. The advance/reset/hold rule was inline in main()'s 20 Hz
guidance loop; DEEP-H2 extracted its pure core to
`m4_intercept.update_coded_dash_streak()` (smallest byte-identical diff -- see
test_byte_identical_to_prefix_inline_logic below and the fuzz that pins it). The
per-tick FRESHNESS gate that drives it stays a one-liner in the loop
(new_meas = last_meas_t_mono is None or meas.t_mono != last_meas_t_mono;
m4_intercept.py:2407) -- reproduced verbatim in _drive() below and cited, not
extracted. The deploy mirror flight/deploy/seeker_loop.py is the POST-handoff
ENGAGE-phase terminal (its own docstring line 5) and does NOT re-implement the
pre-handoff streak, so there is nothing to test there.

'FRESH' -- EXACTLY AS THE CODE DEFINES IT (cited):
  * new_meas (m4:2407): a NEW detector RESULT arrived this tick, i.e. its
    Measurement.t_mono differs from the last one consumed. The 20 Hz control
    loop outpaces the ~14 Hz detector, so most ticks re-read the SAME result
    (new_meas False) -- those HOLD the streak (m4:2774-2778 comment).
  * in the CODED_DASH advance (m4:2854-2865 -> update_coded_dash_streak): that
    new result must have meas.range_m is not None (the frame actually detected a
    target; a no-target frame carries range_m=None, m3_static_intercept.py
    detection_loop) AND be inside the optional plausibility window
    [--coded-dash-acquire-range-min/max]. Note the CODED_DASH advance reads
    meas.range_m directly, NOT `detected`/`fresh` (which fold in the MEAS_STALE_S
    gate) -- so the streak counts consecutive new DETECTOR RESULTS, not
    consecutive sim-time (documented + pinned in
    test_streak_counts_new_results_not_sim_time).

HONESTY BOUNDARY (the point of DEEP-H2). A self-consistent own-prop PHANTOM that
reports a plausible range every frame advances this streak exactly like a real
target and DOES fire handoff -- the streak logic cannot tell them apart, and the
range/appearance gates that tried were all flown NULL (range-plausibility window
add #18f; ADR-0077 range gate; appearance retrain add #18d). The false-handoff
hazard is designed OUT in HARDWARE (the forward-camera mount puts the prop out of
FoV, ADR-0076 add #18h + docs/project_state.json handoff stage note), NOT in
software. test_self_consistent_phantom_streak_fires_handoff pins this residual
HONESTLY rather than pretending it is rejected.

Run: `.venv/bin/python -m pytest tests/test_handoff_streak.py -v`
"""
import os
import random
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, os.path.join(SCRIPTS, "seeker"))

from m4_intercept import (                                    # noqa: E402
    update_coded_dash_streak, CODED_DASH_ACQUIRE_STREAK,
)

N = CODED_DASH_ACQUIRE_STREAK  # 5 -- pinned literal below so a silent change trips


def _drive(ticks, streak0=0, lo=None, hi=None, handoff_at=N):
    """Replay a stream of detector frames through the CODED_DASH handoff state
    machine EXACTLY as m4_intercept.py's guidance loop does, and report when
    handoff fires.

    Each tick is (t_mono, range_m):
      * t_mono  -- the Measurement.t_mono of the detector result the loop reads
        this control tick. A tick whose t_mono equals the previous one is the
        20 Hz loop RE-READING the same ~14 Hz detector result (not new); a
        distinct t_mono is a genuinely NEW detector result.
      * range_m -- that frame's implied range in metres, or None if the frame
        detected no target (the "gap").

    Faithfully mirrors the loop (line refs into scripts/m4_intercept.py):
        new_meas = last_meas_t_mono is None or meas.t_mono != last_meas_t_mono  # :2407
        consecutive_fresh = update_coded_dash_streak(                            # :2854
            consecutive_fresh, new_meas, meas.range_m, lo, hi)
        if consecutive_fresh >= CODED_DASH_ACQUIRE_STREAK:                       # :2867
            phase = "ENGAGE"   # one-way; the CODED_DASH branch never runs again # :2870
    Once handoff fires we STOP feeding the CODED_DASH branch (phase == ENGAGE),
    reproducing that one-way transition -- so n_handoffs records re-handoff churn.

    Returns (fire_tick_index_or_None, final_streak, n_handoffs, streak_trace).
    """
    streak = streak0
    last_t = None
    fire = None
    n_handoffs = 0
    trace = []
    for i, (t_mono, range_m) in enumerate(ticks):
        if fire is not None:
            # phase == ENGAGE: the `elif phase == "CODED_DASH"` branch is dead,
            # so the streak logic is not touched again (m4:2821 guard).
            trace.append(streak)
            continue
        new_meas = last_t is None or t_mono != last_t
        if new_meas:
            last_t = t_mono
        streak = update_coded_dash_streak(streak, new_meas, range_m, lo, hi)
        trace.append(streak)
        if streak >= handoff_at:
            fire = i
            n_handoffs += 1
    return fire, streak, n_handoffs, trace


def _fresh_stream(n, range_m=8.0, t0=1000.0, dt=0.07):
    """n distinct-timestamp in-window detections (a clean fresh streak). dt=0.07
    ~= the measured ~14 Hz detector cadence; only the DISTINCTNESS of t_mono
    matters to new_meas, not its spacing."""
    return [(t0 + k * dt, range_m) for k in range(n)]


# --------------------------------------------------------- the 4-vs-5 boundary
def test_constant_is_five():
    """Pin the documented threshold. HANDOFF = 5 consecutive fresh detections
    (m4_intercept.py:312). If this changes, the tests + docstrings below (and
    docs/project_state.json handoff stage) must be re-read."""
    assert CODED_DASH_ACQUIRE_STREAK == 5


def test_five_consecutive_fresh_fires_handoff():
    fire, streak, n_handoffs, _ = _drive(_fresh_stream(N))
    assert fire == N - 1          # fires on the 5th fresh detection (0-indexed)
    assert streak == N
    assert n_handoffs == 1


def test_four_consecutive_fresh_does_not_fire():
    fire, streak, n_handoffs, _ = _drive(_fresh_stream(N - 1))
    assert fire is None           # 4 is not enough
    assert streak == N - 1
    assert n_handoffs == 0


# ------------------------------------------------- a gap resets; a stall holds
def test_gap_no_target_frame_resets_streak():
    """A NEW frame that detected no target (range_m=None) is the 'gap': it RESETS
    the streak to 0 (update_coded_dash_streak: new_meas True, range None -> 0).
    So 4 fresh + a gap + 4 more fresh must NOT fire; it needs a fresh run of 5
    AFTER the gap."""
    ticks = _fresh_stream(N - 1, t0=1000.0)                       # 4 fresh
    ticks += [(1100.0, None)]                                     # gap: new, no target
    ticks += _fresh_stream(N - 1, t0=1200.0)                      # 4 more fresh
    fire, streak, _, trace = _drive(ticks)
    assert fire is None
    assert trace[N - 2] == N - 1          # 4 built up ...
    assert trace[N - 1] == 0              # ... then the gap frame zeroed it
    assert streak == N - 1                # rebuilt to 4, still short

    # one more fresh detection after the rebuild DOES fire (streak completes 5).
    fire2, _, n2, _ = _drive(ticks + [(1300.0, 8.0)])
    assert fire2 == len(ticks)            # the very next fresh detection
    assert n2 == 1


def test_out_of_window_detection_resets_streak():
    """With a plausibility window [lo, hi] set (add #18f, default None/off), a new
    detection OUTSIDE the window resets the streak just like a no-target gap --
    it advances only in-window."""
    lo, hi = 4.0, 12.0
    ticks = _fresh_stream(N - 1, range_m=8.0)                     # 4 in-window
    ticks += [(1500.0, 1.5)]                                      # phantom-range: out of window
    ticks += _fresh_stream(N - 1, range_m=8.0, t0=1600.0)        # 4 more in-window
    fire, streak, _, trace = _drive(ticks, lo=lo, hi=hi)
    assert trace[N - 2] == N - 1
    assert trace[N - 1] == 0             # the 1.5 m reading (below lo) reset it
    assert fire is None
    assert streak == N - 1


def test_streak_counts_new_results_not_sim_time():
    """HONEST CONTRACT NUANCE (interview-relevant): the streak counts consecutive
    new DETECTOR RESULTS, NOT consecutive sim-time. A stalled detector re-reads
    the SAME t_mono every control tick -> new_meas False -> the streak is HELD,
    not reset; the CODED_DASH advance reads meas.range_m directly and never
    consults the MEAS_STALE_S `detected` gate (m4:2854-2865 vs
    m3_static_intercept.sample_measurement:300). So 4 fresh, a long same-timestamp
    stall, then ONE more new detection completes the 5 and fires -- even though
    the detections were not contiguous in time. (The dash instead times out via
    --coded-dash-max-s, m4:2872, not via a streak reset.)"""
    ticks = _fresh_stream(N - 1, t0=2000.0)                       # 4 fresh, distinct t_mono
    ticks += [(2000.0 + (N - 2) * 0.07, 8.0)] * 40               # 40 ticks re-reading the LAST t_mono (stall)
    ticks += [(2500.0, 8.0)]                                      # one genuinely new detection
    fire, _, n_handoffs, trace = _drive(ticks)
    # the streak sat at 4 through the whole stall (held, never reset) ...
    assert all(s == N - 1 for s in trace[N - 2:len(ticks) - 1])
    # ... and the single new detection after the stall fires the handoff.
    assert fire == len(ticks) - 1
    assert n_handoffs == 1


# ------------------------------------- duplicate timestamp cannot advance
def test_duplicate_timestamp_cannot_advance_streak():
    """new_meas = meas.t_mono != last_meas_t_mono (m4:2407): a repeated/duplicate
    frame timestamp is new_meas False, so a spammed duplicate frame -- however
    many times it is re-read -- CANNOT advance the streak past its first count."""
    # the SAME (t_mono, range) 20 times: exactly one new_meas (the first read is
    # new because last_meas_t_mono starts None, m4:2407) -- duplicates can turn
    # ONE real detection into at most +1, never a full streak.
    ticks = [(3000.0, 8.0)] * 20
    fire, streak, _, trace = _drive(ticks)
    assert streak == 1            # only the first read counted
    assert all(s == 1 for s in trace)
    assert fire is None

    # build 4 from DISTINCT frames, then spam duplicates of the 4th's timestamp:
    # the streak is pinned at 4 and never completes, because re-reading a frame
    # already counted is new_meas False (holds, never advances).
    ticks2 = _fresh_stream(N - 1, t0=3100.0)                     # 4 distinct -> streak 4
    ticks2 += [(3100.0 + (N - 2) * 0.07, 8.0)] * 20             # 20 dups of the 4th t_mono
    fire2, streak2, _, _ = _drive(ticks2)
    assert streak2 == N - 1       # held at 4 -- the duplicate never advances it
    assert fire2 is None


# ------------------------------------------- post-handoff: no re-handoff churn
def test_post_handoff_disengages_no_churn():
    """The handoff is a ONE-WAY phase transition (phase = "ENGAGE", m4:2870);
    the `elif phase == "CODED_DASH"` branch (m4:2821) never runs again, so a long
    tail of further fresh detections triggers NO second handoff."""
    ticks = _fresh_stream(N) + _fresh_stream(20, t0=4000.0)      # fire, then 20 more fresh
    fire, _, n_handoffs, _ = _drive(ticks)
    assert fire == N - 1          # fired on the 5th, not later
    assert n_handoffs == 1        # exactly once -- no churn


# ----------------------------------- the documented residual: phantom fires
def test_self_consistent_phantom_streak_fires_handoff():
    """HONEST RESIDUAL RISK -- pinned, not hidden (ADR-0076 add #18g/#18h;
    docs/project_state.json handoff stage note). In the DEPLOYED config the
    plausibility window is OFF (lo=hi=None: ADR-0077 range gate + add #18f window
    both flown NULL because they also killed the real detections). So an own-prop
    PHANTOM that self-consistently reports ~1.5 m every frame is, to this state
    machine, INDISTINGUISHABLE from a real target: 5 such frames FIRE the handoff.
    The streak logic CANNOT reject it -- the phantom is designed out in HARDWARE
    (forward-camera mount, prop out of FoV), not here. If this ever starts
    asserting no-fire, the honesty scope has silently changed -- re-derive."""
    phantom = _fresh_stream(N, range_m=1.5)   # deployed config: window open
    fire, streak, n_handoffs, _ = _drive(phantom, lo=None, hi=None)
    assert fire == N - 1, "the self-consistent phantom DOES fire handoff (residual risk)"
    assert n_handoffs == 1

    # And confirm this is a REAL indistinguishability, not a quirk: a genuine
    # target at a plausible range fires under the identical open-window config.
    real = _fresh_stream(N, range_m=8.0)
    fire_real, *_ = _drive(real, lo=None, hi=None)
    assert fire_real == N - 1

    # The RETIRED window (add #18f, ADR-0077 NULL) WOULD have rejected the 1.5 m
    # phantom -- documenting WHY it was retired (it rejected real detections too,
    # r2l 8/8->0), i.e. why the residual stands in the deployed config.
    fire_gated, *_ = _drive(_fresh_stream(N, range_m=1.5), lo=4.0, hi=12.0)
    assert fire_gated is None      # the (NULLed) gate would have stopped it ...
    fire_gated_real, *_ = _drive(_fresh_stream(N, range_m=8.0), lo=4.0, hi=12.0)
    assert fire_gated_real == N - 1   # ... but only at the window it also needs open


# --------------------------------------------------- byte-identical extraction
def test_byte_identical_to_prefix_inline_logic():
    """The DEEP-H2 extraction must reproduce the pre-extraction inline block
    (m4_intercept.py CODED_DASH) tick-for-tick. Fuzz update_coded_dash_streak()
    against a verbatim copy of the old inline logic so any future divergence
    (a silent behavior change hiding behind the 'byte-identical' claim) fails."""
    def _inline_ref(streak, new_meas, r, lo, hi):
        # verbatim pre-extraction logic (old m4_intercept.py CODED_DASH branch):
        if new_meas:
            r_ok = r is not None and (
                lo is None or r >= lo) and (hi is None or r <= hi)
            streak = streak + 1 if r_ok else 0
        return streak

    rng = random.Random(20260720)
    for _ in range(50000):
        streak = rng.randint(0, 8)
        new_meas = bool(rng.randint(0, 1))
        r = None if rng.random() < 0.3 else round(rng.uniform(0.0, 20.0), 3)
        lo = None if rng.random() < 0.5 else rng.uniform(1.0, 6.0)
        hi = None if rng.random() < 0.5 else rng.uniform(6.0, 12.0)
        got = update_coded_dash_streak(streak, new_meas, r, lo, hi)
        ref = _inline_ref(streak, new_meas, r, lo, hi)
        assert got == ref, (streak, new_meas, r, lo, hi, got, ref)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
