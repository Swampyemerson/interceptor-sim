# Spec v3: the estimate is now the limit -- fix the estimator properly

Head review of v2 (2026-09-17). Excellent round: both bugs were real. Your diagnostics now
say where the remaining miss comes from. Nominal case (camera facing, n = 100, scatter on):

    miss                     median 0.57 m   (p10 0.19, p90 1.38)
    estimator position error median 0.88 m at closest approach, 0.51 m one second earlier
    estimator velocity error median 0.90 m/s
    decodes in the last second: median 12;  closing speed at CPA: 2.5 m/s

So with a dozen decodes per second at 1-4 m range -- where a 0.30 m tag measured to 0.3 px
is geometrically good to about a CENTIMETRE -- the filter is nearly a metre wrong. The
estimator, not control and not "saturated", is what stands between 29% and 90%.
(`saturated` wins the attribution only because the vehicle's flag is true whenever any slew
limit is active, which is most of the time. Fix the attribution too, below.)

## Suspected causes, in the order I would check them
1. **Attitude/position used at the wrong time.** A Detection arrives ~45 ms after the frame
   was exposed (`t_capture` vs `t_available`). The measurement must be converted to NED with
   the vehicle's quaternion and position AT `t_capture`, not at arrival. At 100 deg/s body
   rate, 45 ms is 4.5 deg, which is 0.24 m at 3 m. Keep a short ring buffer (0.3 s) of own
   (t, pos, vel, quat) samples inside the guidance and look up (interpolate) the sample at `t_capture`. This is honest:
   a real Pi timestamps its frames and the autopilot timestamps its state.
2. **Relative-state filter driven by own acceleration.** Estimating RELATIVE position with a
   constant-velocity model makes every own-vehicle acceleration look like target motion
   unless the feed-forward is exact. Re-formulate: estimate the TARGET's absolute NED position
   and velocity (6 states, constant velocity + acceleration process noise). Measurement =
   own position at t_capture + rotated camera measurement. Own position comes from
   `VehicleState.pos_ned` (the own-state EKF; legitimately available). A slowly drifting own
   position error cancels, because the same own position is used for control.
3. **Measurement covariance.** Build R in the camera line-of-sight frame (along, cross, cross)
   and rotate it into NED properly (R_ned = T R_los T^T) every update. With 1 and 2 fixed, the
   0.20 m cross floor should be able
   to come down a lot (try 0.03-0.05 m). Keep a floor; report the sweep.
4. **Calibration errors are part of the scatter** (focal length 1 %, mount tilt 1 degree):
   check how much of the remaining error they explain by running once with those two zeroed.

## Also
- Attribution: replace the vehicle's any-limit `saturated` flag in the attribution with a
  real test: was the COMMANDED acceleration (change of v_cmd per second) above what the
  vehicle delivered for more than half of the last second. Then let `estimate` vs `control`
  compete on magnitude first; `no_tag_last_second` only when decodes in the last second = 0.
- Report, per cell: median estimator position error at CPA and 1 s before, next to the miss.

## Measure
Same grid as v2 (camera and rear facing, tilt 12, fx 933, scatter on, n = 100), plus a lens
comparison at fx 933 vs 1400 for the nominal and the +/-2 m altitude and 30 degree aim cases.
Before/after for every change, one change at a time if affordable, so we learn which fix
bought what.

Same rules: loose test bounds, print measured values, report every tuned value with
before/after, say what you doubt. Verify with `.venv/bin/python -m pytest isim/tests -q`.
