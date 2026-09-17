# Spec: "pursuit" engagement concept prototype (isim-native)

## Why
The current concept is a high-speed fly-by: a 16 m/s open-loop sprint across the path of a
9 m/s target, closing at about 19 m/s. A 0.30 m AprilTag decodes only inside about 13 m
(fx = 933 px), so the camera gets about 0.6 s and about 15 decodes, and the quad's nose-down
pitch while accelerating hides the tag. No terminal law can remove 2 m of height error or
15 degrees of aim error in 0.6 s. Measured in isim 2026-09-17.

## Concept: arrive slowly
Phase A (no tag seen yet). Use only the PRE-FLIGHT belief about the target: its start
position and velocity, the same information the sprint's heading solve is given, with the
same `Scatter` belief errors (speed belief error, unknown start jitter, heading/compass
error applied as a rotation of the believed geometry about the launch point, height guess
error). Fly to a moving aim point `d_behind_m` (default 8 m) BEHIND the believed target
position along its track:

    aim(t)  = p_target_belief(t) - d_behind_m * unit(v_target_belief)
    v_cmd   = v_target_belief + kp_pos * (aim - own_pos)        (kp_pos default 0.8 1/s)

clipped to `v_max_ms` (default 16) and slew-limited to `accel_max_ms2` (default 6, gentle on
purpose: less pitch). This is position tracking of a moving point, so the vehicle arrives
velocity-matched, a few metres behind the target, looking at its rear. Yaw command: toward
the believed target position. Vertical: same law, all three axes; believed target altitude.

Phase B (tag decoded, at least `acquire_n` = 2 detections within 0.3 s). Estimate the target's
relative position and velocity from detections: range, bearing, elevation, own attitude
quaternion, camera mount tilt; own velocity is available from `VehicleState`. Use a small
constant-velocity Kalman filter; measurement noise cross-range = range * sigma_px / fx,
along-range = range^2 * sigma_side_px / (fx * tag_side). Command:

    v_cmd = v_target_est + v_close * unit(r_est) + kp_cross * (r_est minus its along-LOS part)

with `v_close_ms` default 4 and `kp_cross` default 1.5, all three axes, slew-limited
(horizontal 8, vertical 6 m/s^2), yaw toward the target. If no decode for `coast_s` (1.0):
keep flying on the estimate; after 3 s without a decode fall back to Phase A using the last
estimate as the new belief. Inside 1.0 m estimated range: hold the current command (fly
through).

Guidance may use ONLY: own `VehicleState`, `Detection`s, and the pre-flight belief passed to
its constructor. Never a `TargetState` or `FrameReport`.

## Files
- NEW `isim/concepts.py`: `PursuitConfig` dataclass + `PursuitRendezvousGuidance`
  (isim `Guidance` protocol). About 250 lines.
- EDIT `isim/scenario.py`: add `concept: str = "flyby"` ("flyby" = today's behaviour,
  bit-identical; "pursuit" = build the new guidance instead of `RealFlightGuidance`, vehicle
  starts hovering at the same standby point, GO at the same time, target motion starts at GO)
  and `tag_facing: str = "camera"` ("camera" = today's best case; "rear" = tag normal is
  minus the target's velocity direction; "side" = horizontal normal perpendicular to the
  track, facing the launch side). Pass `--concept` and `--tag-facing` through `isim/mc.py`.
- NEW `isim/tests/test_concepts.py`.

## Tests (loose bounds only; never assert a tuned number; print the measured values)
- Phase A alone (seeker that never decodes) against an exactly-known target: ends within
  1 m of the aim point and within 1 m/s of target velocity.
- Honesty: a spy shows guidance only ever receives `VehicleState` and `Detection`/None.
- Determinism per seed; `concept="flyby"` bit-identical to before this change.
- Closed loop with `AprilTagSeeker`, fx 933, 0.30 m tag, `tag_facing="rear"`: prints miss for
  the nominal crossing case.

## Measure and report (no tuning to flatter)
With `Scatter()` on, n = 100 per cell, fx 933, camera tilt 0 and 12 deg, `tag_facing` rear and
camera, run the four requirement axes (aim error 0-30 deg, target altitude offset -2..+3 m,
target speed 0-9 m/s, and for sprint quality use `v_max_ms` 16, 12, 10) for `concept="pursuit"`.
Report the tables (median, p90, % inside 0.35 m, % inside 1.0 m, median time to intercept,
median decodes), the failure modes you see in traces (tag leaves the frame? estimator? tag
angle?), and your own doubts. If a number looks too good, look for a leak of truth into
guidance before reporting it.

Verify with `.venv/bin/python -m pytest isim/tests -q`.
