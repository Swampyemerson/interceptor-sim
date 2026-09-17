# Spec v2: make the pursuit concept converge, and make every miss explain itself

Head review of v1 (2026-09-17). Your report was good and honest. One thing you could not
know: the engine was ending pursuit runs 1 s after the target's FIRST fly-past. That is now
fixed in `isim/scenario.py` (`pursuit_window_s = 25`, scored on the closest approach over the
whole window). Re-measured with that fix, Scatter on, fx 933, tilt 12, n = 60:

| tag facing | case | median miss | inside 0.35 m | inside 1 m | t_cpa | closing |
|---|---|---|---|---|---|---|
| rear | nominal | 1.44 m | 5% | 37% | 7.7 s | 6.1 m/s |
| rear | target +2 m | 1.79 | 5% | 28% | 8.2 | 6.0 |
| rear | target -2 m | 1.09 | 5% | 42% | 7.7 | 6.1 |
| rear | aim error 15 deg | 2.23 | 5% | 20% | 8.8 | 6.4 |
| rear | aim error 30 deg | 5.50 | 0% | 3% | 3.6 | 12.6 (never engaged: 7%) |
| rear | target 4 m/s | 0.45 | 38% | 78% | 7.7 | 7.3 |
| rear | v_max 12 | 0.50 | 35% | 70% | 9.4 | 2.9 |
| camera | nominal | 4.39 | 0% | 7% | 3.8 | 13.0 |

So the concept shows life (rear tag, slower arrival = better), but: (1) with the tag always
facing the camera the closest approach is still the first fly-past at 3.8 s and the vehicle
never gets closer in the following 21 s -- that should be the EASIEST case, so something is
wrong after the pass (180-degree yaw? estimator reset? fallback loop? look at traces);
(2) arrival is too fast and too loose in the last metres.

## Build
1. Find and fix why `tag_facing="camera"` does not converge after the first pass. Report the
   cause with a trace excerpt.
2. Closing-speed schedule instead of a constant: `v_close = clamp(k_close * range_est,
   v_close_min, v_close_max)` with defaults k_close 0.6 1/s, min 1.5, max 6 m/s. The final
   metres should be flown slowly and deliberately; this vehicle only needs to TOUCH the
   target (0.35 m), not to hit it hard.
3. Judgment call #1 from your report: keep your interpretation (cross-track relative to the
   target's estimated track) but ALSO steer the full 3-D position error: the command should
   drive r_est (the whole relative position vector) to zero with the target's velocity as
   feed-forward, i.e. `v_cmd = v_target_est + v_close * unit(r_est) + kp_lat * r_perp` where
   r_perp is the part of r_est perpendicular to the current relative-velocity direction
   (that is the component that becomes the miss). Guard the degenerate cases.
4. Last-metre behaviour: the tag fills or leaves the frame very close in. Keep the estimate
   coasting (own-velocity feed-forward makes this accurate over 0.3-0.5 s) and do not
   fall back to Phase A inside 3 m.
5. DIAGNOSTICS (the builder's main complaint about the old simulator was that a miss never
   explained itself). Guidance exposes, per tick, its estimate (`r_est`, `v_t_est`, phase) via
   a `debug` attribute; the SCORING side (`isim/mc.py::_run_one`, which legitimately has truth
   through the trace) records per run, at the time of closest approach and at 1 s before it:
   estimator position error (|r_est - r_true|), estimator velocity error, control error
   (|r_true| that the estimate says should be zero), whether the tag was in frame, decodes in
   the last 1 s, vehicle saturation fraction in the last 2 s, and the phase. Then each miss is
   attributed to the largest contributor: `estimate`, `control`, `no_tag_last_second`,
   `never_engaged`, `saturated`. Print the attribution shares per sweep cell next to the miss
   numbers. Truth must flow ONLY into the scoring record, never back into guidance -- keep
   the spy test, and add a test that guidance has no reference to the trace or target.
6. Re-measure the v1 grid (rear and camera facing, tilt 12, plus `pursuit_v_max_ms` 16/12/10
   and target speeds 0/4/9) with attribution shares, n = 100, Scatter on.

Same rules as before: loose test bounds only, print measured values, no tuning to flatter,
say what you doubt. Reasonable tuning of the pursuit gains to make the concept WORK is part
of this task (it is design, not flattery) -- but report every value you changed and the
before/after.

Verify with `.venv/bin/python -m pytest isim/tests -q`.
