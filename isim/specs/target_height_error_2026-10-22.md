# Pre-registration: target-height cue error tolerance (2026-10-22 overnight)

**Assumption under test:** `target-height-known` (given-perfect): the cue's
believed target altitude is exactly right in every flown arm; the vertical
analogue of the launch-aim cue error. The register's own deliverable line
says a target-height-error sweep replaces the given with a tolerance.

**Mechanism already in the tree:** `Scenario.target_alt_offset_m` moves the
TRUE target off the nominal altitude while every belief (dash altitude,
Phase-A track z) stays nominal — exactly "the cue is wrong about height."
Never swept under the hybrid/chase architecture.

**Design (isim, canon hybrid config, the stern study's exact base):**
- Axis: `target_alt_offset_m` ∈ {0, ±1, ±2, ±3} m (7 cells), Scatter on,
  n=16 paired seeds (0..15), concept="hybrid", 9 m/s, fx 933, rear tag,
  aim_error 0 (isolate the vertical axis).
- Metrics: pct_engage (acquisition), median/p90 miss, median vertical miss
  component (`med_vert_m`), decodes.

**Predictions (before flying):**
1. Small offsets (±1 m) are mostly recovered: the camera measures 3-D
   bearing, so once acquired, the KF pulls the vertical error out —
   engagement ~baseline, median miss inflates <2x.
2. Large offsets (±3 m) break ACQUISITION, not the terminal: the target
   sits outside the camera's vertical wedge during approach (the ADR-0060
   pitch-gap family), so engagement drops and misses go unbounded on the
   never-acquired flights.
3. Sign asymmetry: target BELOW belief (negative offset) hurts more than
   above, because the camera's mount tilt and the dash's nose-down pitch
   both bias the wedge upward relative to the flight path. (Low confidence.)

**Adopt/report rule:** no config change rides on this — the deliverable is
the TOLERANCE NUMBER: the largest |offset| whose engagement stays within
one paired-seed step (2/16) of baseline AND median miss ≤2x. That number
replaces the `target-height-known` given in the assumptions register
(grade moves given-perfect → given-noisy with a stated budget) and becomes
the height-accuracy requirement on the real cue (GPS/baro of the target
feed). A NULL (flat everywhere) means the vertical axis is cheap and the
given was never load-bearing — also worth knowing, register updated either
way.

## Amendment (measure-only probe, registered 2026-10-22 ~02:05 before flying)

**Question:** WHY does +3 m (target above belief) break acquisition while
−3 m does not? Hypothesis H: during approach the camera's vertical wedge
(dash pitch + mount) covers BELOW-belief space better than above — at +3
the target sits above the FOV's top edge for most pre-acquisition frames.

**Design:** rerun offsets {0, +3, −3} (n=8 seeds, same canon config) with a
MEASURE-ONLY seeker wrapper recording every FrameReport's `in_fov` and
`v_px` before first acquisition. Guidance sees exactly what it saw —
scoring/logging only (honesty boundary: the wrapper is outside guidance).

**Predictions:** +3 shows pre-acquisition in-FOV fraction << the 0/−3 cells
(H confirmed → the cue interface gains the sign rule "bias the height cue
LOW when uncertain"); if in-FOV fractions are similar, H is refuted and the
asymmetry lives elsewhere (decode vs incidence, KF pull-in) — then NO sign
rule may be quoted until the real mechanism is found.
