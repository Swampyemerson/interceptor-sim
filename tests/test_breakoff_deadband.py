#!/usr/bin/env python3
"""FIX B (audit #37): noise deadband + trend/range gate on the past-CPA
range-increase breakoff, and the CSRT frame-border coast.

THE BUG. BREAKOFF_RANGE_INCREASES fires on a STRICT monotone-3 rise of the raw
measured range with NO deadband. Known-size range (fx*span/px) is pixel-
quantized (~2-3 cm/px near 2 m), so pure jitter -- or a phantom near-lock, or a
frame-edge-clipped tracker box -- can manufacture 3 rises and break off WHILE
STILL CLOSING. Forensics over the logged runs flags 7/33 breakoffs as EARLY
(ground-truth range still shrinking at breakoff).

THE FIX (all default-off -> byte-identical):
  * --breakoff-deadband-m : a step counts as a rise only if it exceeds the last
    significant range by > deadband; within-deadband creep HOLDS the streak.
  * --breakoff-min-rise-m : total rise from the streak base must clear this.
  * --breakoff-max-range-m: only fire when the MEASURED range is small (a
    large-range 'increase' is a lost-target flyby, not an overshoot). Gates on
    the camera's measured range, NOT r_hat (a phantom corrupts r_hat UPWARD even
    on a genuine intercept -- see L_B/L_C below, r_hat@bo ~51-54 m at a real
    2.3 m CPA).
  * MARKERLESS_TRACK_BORDER_MARGIN_PX (detect_track): coast a CSRT box clipped
    by a frame edge (truncated width under-reads range).

HONESTY. All four use only camera measurements + own-state; no ground truth.

WHAT THIS PROVES (against the REAL logged tails, meas-range sequences pulled from
the flight CSVs by scratchpad/breakoff_forensics.py):
  * DEFAULT byte-identical to the pre-fix inline logic.
  * At deployment values the SEPARABLE early breakoffs do NOT fire, and the
    genuine post-CPA breakoffs DO fire (<= 1 detection later).
  * The two phantom-CPA-SPOOF early tails are camera-INDISTINGUISHABLE from a
    genuine CPA crossing (their spoofed measured range walks 1.1 -> 1.6 m just
    like a real fly-through) -- this pinned test records that the range gate
    alone canNOT reject them; they are covered UPSTREAM by the
    handoff-cue-gate / terminal-reject-gate / detect-then-track phantom rejection
    in the deployment config, which stops the phantom before it ever reaches the
    breakoff channel. This is an honest, deliberate scope boundary, not a miss.

Run: `.venv/bin/python -m pytest tests/test_breakoff_deadband.py -v`
"""
import math
import os
import random
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, os.path.join(SCRIPTS, "seeker"))

from m4_intercept import (                                    # noqa: E402
    update_range_increase_streak, BREAKOFF_RANGE_INCREASES,
)

N = BREAKOFF_RANGE_INCREASES  # 3

# Deployment values (audit #37 recommended; wired but default-off in argparse).
DEADBAND = 0.05
MIN_RISE = 0.15
MAX_RANGE = 5.0   # == TERMINAL_RANGE_M under --fpv

# --- REAL logged meas-range tails (last fresh detections before breakoff),
# extracted from the flight CSVs via scratchpad/breakoff_forensics.py.
# EARLY = fired while ground-truth range still shrinking (7/33 flagged).
EARLY_SEPARABLE = {
    # (name, tail) -- rejected by max-range and/or deadband at deployment values
    "073207_gt4.19": [1.204, 2.175, 1.392, 2.551, 4.614, 16.543],
    "090121_gt5.92": [1.568, 1.46, 1.318, 1.478, 4.956, 6.364],
    "200718_gt5.51": [6.042, 5.446, 4.894, 5.318, 5.479, 5.544],
    "204800_gt7.41": [8.335, 8.046, 7.214, 7.416, 7.442, 7.846],
    "214456_gt6.74": [8.544, 8.857, 8.402, 8.747, 8.762, 8.85],
}
# The two phantom-CPA-SPOOF early tails: a phantom near-lock whose measured range
# walks up 1.1 -> 1.6 m exactly like a genuine post-CPA recession. NO camera-only
# range rule separates these from a real fly-through (compare LEGIT below).
EARLY_PHANTOM_SPOOF = {
    "090833_gt4.95": [1.696, 1.664, 1.139, 1.338, 1.394, 1.611],
    "074632_gt5.43": [1.532, 1.462, 1.103, 1.293, 1.403, 1.665],
}
# LEGIT = genuine post-CPA breakoffs near a real intercept (must still fire).
LEGIT = {
    "011557_gt2.42_apriltag": [1.786, 1.662, 1.632, 1.781, 1.892, 2.048],
    "204440_gt2.27": [9.932, 8.91, 1.528, 1.64, 1.683, 1.74],
    "213513_gt2.75": [9.626, 9.127, 1.696, 1.821, 1.986, 2.064],
    "205341_gt4.06": [3.032, 2.13, 2.262, 2.332, 2.558, 3.227],
}


def _replay(seq, deadband, min_rise, max_range, armed=True):
    """Replay a fresh-detection meas-range sequence through the streak helper +
    trigger gate. Returns (fire_index_or_None, final_streak, final_start,
    final_meas)."""
    streak, last, start = 0, None, None
    fire = None
    for i, mr in enumerate(seq):
        streak, last, start = update_range_increase_streak(
            streak, last, start, mr, deadband)
        rise_ok = start is None or (mr - start) >= min_rise
        range_ok = max_range is None or mr <= max_range
        if fire is None and armed and streak >= N and range_ok and rise_ok:
            fire = i
    return fire, streak, start, seq[-1]


def _fires_within_one_of_baseline(seq):
    """A genuine breakoff must still fire at deployment values no later than one
    detection after the pre-fix (deadband=0) trigger. The forensic tails end at
    the pre-fix breakoff tick, so a legit tail that deployment delays by exactly
    one detection reaches streak N-1 (with the range/rise gates already
    satisfied at the final sample) -- the very next rising detection fires."""
    base, _, _, _ = _replay(seq, 0.0, 0.0, None)
    dep, streak, start, last = _replay(seq, DEADBAND, MIN_RISE, MAX_RANGE)
    assert base is not None, "baseline (pre-fix) did not fire on a legit tail?"
    if dep is not None:
        return dep <= base + 1
    # not fired in-tail: accept iff it is exactly one detection short AND the
    # gates already pass, so the next rise fires (== base + 1).
    rise_ok = start is not None and (last - start) >= MIN_RISE
    range_ok = last <= MAX_RANGE
    return streak == N - 1 and rise_ok and range_ok


# --------------------------------------------------------------- byte-identical
def test_default_is_byte_identical_to_prefix_inline_logic():
    """deadband=0, min_rise=0, max_range=None must reproduce the OLD inline
    `streak++ if meas>last else streak=0` + threshold, tick for tick."""
    rng = random.Random(20260710)
    for _ in range(400):
        seq = [round(rng.uniform(0.3, 12.0), 3) for _ in range(rng.randint(1, 25))]
        # run the OLD inline logic and the NEW helper in LOCKSTEP, tick for tick
        old_streak, old_last, old_fire = 0, None, None
        new_streak, new_last, new_start, new_fire = 0, None, None, None
        for i, mr in enumerate(seq):
            # old inline reference (verbatim pre-fix logic)
            if old_last is not None and mr > old_last:
                old_streak += 1
            else:
                old_streak = 0
            old_last = mr
            if old_fire is None and old_streak >= N:
                old_fire = i
            # new helper with defaults (deadband 0, min_rise 0, max_range None)
            new_streak, new_last, new_start = update_range_increase_streak(
                new_streak, new_last, new_start, mr, 0.0)
            rise_ok = new_start is None or (mr - new_start) >= 0.0
            if new_fire is None and new_streak >= N and rise_ok:  # max_range None
                new_fire = i
            assert new_streak == old_streak, f"streak diverged at {i}: {seq}"
            assert new_last == old_last, f"last diverged at {i}: {seq}"
        assert new_fire == old_fire


def test_helper_deadband_semantics():
    """Rise > deadband increments; within-deadband creep holds without advancing
    the reference; a drop resets."""
    s, last, start = update_range_increase_streak(0, None, None, 2.0, 0.05)
    assert (s, last, start) == (0, 2.0, None)          # first fresh: no compare
    s, last, start = update_range_increase_streak(s, last, start, 2.20, 0.05)
    assert s == 1 and start == 2.0 and last == 2.20    # rise, base recorded
    s, last, start = update_range_increase_streak(s, last, start, 2.23, 0.05)
    assert s == 1 and last == 2.20                     # creep (+0.03): hold, no advance
    s, last, start = update_range_increase_streak(s, last, start, 2.40, 0.05)
    assert s == 2 and last == 2.40                     # rise vs the held 2.20
    s, last, start = update_range_increase_streak(s, last, start, 1.9, 0.05)
    assert s == 0 and start is None and last == 1.9    # drop resets


# ------------------------------------------------------------- early must not fire
def test_separable_early_breakoffs_do_not_fire_at_deployment():
    for name, seq in EARLY_SEPARABLE.items():
        base, *_ = _replay(seq, 0.0, 0.0, None)
        dep, *_ = _replay(seq, DEADBAND, MIN_RISE, MAX_RANGE)
        assert base is not None, f"{name}: pre-fix should have fired (it did in flight)"
        assert dep is None, f"{name}: still fires at deployment values (should not)"


def test_phantom_spoof_early_tails_are_camera_indistinguishable():
    """HONEST SCOPE BOUNDARY: these two early breakoffs are phantom near-locks
    whose measured range walks up like a real CPA crossing. The range gate alone
    canNOT reject them without also killing the genuine tails (they have the same
    shape as LEGIT '204440'/'011557'). They ARE rejected upstream in the
    deployment config (handoff-cue-gate / terminal-reject-gate / detect-then-
    track) before the phantom ever reaches this channel. Pinned so the limitation
    is explicit, not silently assumed fixed here."""
    for name, seq in EARLY_PHANTOM_SPOOF.items():
        dep, *_ = _replay(seq, DEADBAND, MIN_RISE, MAX_RANGE)
        assert dep is not None, (
            f"{name}: unexpectedly rejected by the range gate -- if this starts "
            "passing, re-derive; it means the tails now differ from a legit CPA")
        # and confirm it is genuinely indistinguishable: a legit tail with the
        # same small-range shape ALSO fires under the identical gate.
    legit_shape, *_ = _replay(LEGIT["011557_gt2.42_apriltag"], DEADBAND, MIN_RISE, MAX_RANGE)
    assert legit_shape is not None


# --------------------------------------------------------------- legit must fire
def test_legit_breakoffs_still_fire_within_one_detection():
    for name, seq in LEGIT.items():
        assert _fires_within_one_of_baseline(seq), (
            f"{name}: genuine post-CPA breakoff no longer fires within one "
            "detection at deployment values")


def test_legit_apriltag_fires_at_same_tick():
    """The clean AprilTag post-CPA breakoff (subpixel range, no phantom) should
    be unaffected -- fires at the same tick as the pre-fix code."""
    seq = LEGIT["011557_gt2.42_apriltag"]
    base, *_ = _replay(seq, 0.0, 0.0, None)
    dep, *_ = _replay(seq, DEADBAND, MIN_RISE, MAX_RANGE)
    assert dep == base


# --------------------------------------------------------------- border coast
def test_border_margin_default_off_is_noop():
    cv2 = pytest.importorskip("cv2")  # noqa: F841
    from detect_track import DetectThenTracker
    t = DetectThenTracker.__new__(DetectThenTracker)
    t.border_margin_px = 0.0
    # every box, including one hard against a corner, passes when disabled
    assert not t._touches_border((0, 0, 10, 10), 640, 480)
    assert not t._touches_border((300, 200, 40, 40), 640, 480)


def test_border_margin_flags_edge_clipped_boxes():
    cv2 = pytest.importorskip("cv2")  # noqa: F841
    from detect_track import DetectThenTracker
    t = DetectThenTracker.__new__(DetectThenTracker)
    t.border_margin_px = 8.0
    W, H = 640, 480
    # interior box: safe
    assert not t._touches_border((300, 200, 40, 40), W, H)
    # left edge within margin
    assert t._touches_border((5, 200, 40, 40), W, H)
    # right edge within margin (x+w >= W-8)
    assert t._touches_border((W - 40, 200, 38, 40), W, H)
    # top edge
    assert t._touches_border((300, 3, 40, 40), W, H)
    # bottom edge (y+h >= H-8)
    assert t._touches_border((300, H - 40, 40, 38), W, H)
    # just inside the margin on all sides: safe
    assert not t._touches_border((9, 9, W - 18, H - 18), W, H)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
