# Spec v4: from 55-71% to the 90% bar

Head review of v3: strong round, numbers reproduced, committed. The bar is >= 90% inside
0.35 m across the requirement axes. What your diagnostics say is left, in priority order:

1. LAST SECOND (52% of camera-facing misses are `no_tag_last_second`). A 0.30 m tag leaves a
   fx-933 frame inside roughly 0.5-1 m and the vehicle then coasts on a velocity estimate that
   is ~0.9 m/s wrong. Work on both sides:
   a. Velocity estimate quality: check the process noise (target accel density) against a
      constant-velocity target; try 3 -> 1 -> 0.5 m/s^2; report velocity error at CPA-1 s.
   b. Arrive on a line that keeps the tag centred: in the last 3 m prefer zero lateral
      relative velocity (null r_perp early, then close straight).
   c. Slower final closing (v_close_min 1.5 -> 1.0) and see whether it helps or hurts.
   d. A hardware option to MEASURE, not adopt: a small inner tag (0.08 m) co-located with the
      0.30 m one, decodable from ~0.25 m to ~3.5 m. Add `TagParams`-level support only if it
      is simple (second AprilTagSeeker on the same camera is fine); report the gain.
2. HEIGHT OFFSETS (target 2 m lower: 16%; 3 m higher: low). Look at traces first. Suspects:
   Phase A flies at the BELIEVED altitude and the tag is outside the vertical field of view
   on arrival (vertical half-FOV 23 deg at fx 933; 2 m offset at 8 m behind = 14 deg, plus
   pitch); the vertical slew limit; mount tilt 12 deg biasing the view upward (helps +, hurts
   -). Try: arrive further behind when nothing is decoded (d_behind 8 -> 12 m), a slow
   vertical search (+/- 2.5 m sweep at the aim point) if no decode within 2 s of arrival.
3. REAR-TAG AIM CLIFF (never engages beyond ~20 deg). If the vehicle reaches its aim point and
   sees nothing, it should not sit there: yaw search (+/- 60 deg) and the same vertical sweep.
   Also measure `tag_facing="side"` and a two-tag target (rear + side) if cheap.
4. The unexplained oddity (largest estimator error at CPA on the smallest misses): my guess
   is simply that at < 0.3 m true range the estimate's error is dominated by the coast after
   the tag left the frame, and "error at CPA" is the wrong statistic there. Check, and if so
   report error at the LAST DECODE instead.

## Measure
Full requirement grid, both facings, scatter on, n = 100: aim error 0-30 deg, altitude offset
-2..+3 m, target speed 0/4/9, v_max 16/12/10. One change at a time where affordable, with
before/after. Headline table at the end: % inside 0.35 m per cell, v3 vs v4.

Same rules: loose test bounds, print measured values, report every tuned value, say what you
doubt, never let truth reach guidance. Verify with `.venv/bin/python -m pytest isim/tests -q`.
Remember your own gotcha: monkeypatched parameters do not reach spawned workers; pass them
through `Scenario`/`PursuitConfig` fields instead.
