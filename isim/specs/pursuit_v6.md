# Spec v6: make pursuit tolerate timestamp and attitude error

v5 (your adversarial round, committed) is the new honest baseline: nominal 49% (tag facing
camera) / 68% (rear) inside 0.35 m with all errors on; timestamp (+/-20 ms) and own-attitude
(1-3 deg) are co-dominant; velocity error is free. The goal now is design, measured against
that baseline with ALL v5 errors on. Tuning to make the design work is allowed; report every
value changed and its before/after. Never touch the error model to get there.

## A. Close-in steering in the CAMERA frame (expected biggest win)
Why: an attitude bias of 2 deg rotates every NED-converted measurement, so the estimated
target position is wrong by range x 2 deg and, worse, that error CHANGES as range closes,
which the filter reads as target velocity. But the tag's position in the camera's own frame
(range, bearing, elevation -> x right, y down, z forward) needs no attitude at all.
Build: inside `cam_frame_range_m` (default 5 m) compute the lateral and vertical relative
position in the BODY frame directly from the detection, and command body-frame lateral and
vertical velocity = kp_cam x that offset (+ the filter's target-velocity feed-forward rotated
into body), with forward speed from the existing closing schedule. Convert to the NED command
with the (perturbed) attitude: an attitude error now only rotates a command by 2 deg, which
is second order. Between decodes, propagate the body-frame offset with own body-frame
velocity and the feed-forward. Keep the absolute-frame filter for far range and for the
target-velocity feed-forward.

## B. Timestamp bias
A constant timestamp bias looks like the target sitting ahead of / behind its measured
position ALONG ITS OWN VELOCITY by speed x bias. Options, measure each:
1. Do nothing extra but rely on A (camera-frame steering does not care where the target is
   in NED, only where it is in the image -- check how much of the timestamp cost A removes).
2. Add the bias as a 7th filter state (observable when own attitude/position changes quickly
   relative to the target; may be weakly observable -- report honestly).
3. Report the cost as a function of bias size: 0, 5, 10, 20, 40 ms -- this becomes a bench
   requirement for the builder (how well must the Pi timestamp frames?).

## C. Attitude
Report cost vs attitude bias size (0, 0.5, 1, 2, 3 deg roll/pitch; yaw 0, 2, 5 deg) with A in
place -- also a bench requirement.

## D. Weave vs rear tag
With A in place re-measure weave (amp 1-3 m) for camera, rear and a rear tag with the
incidence limit relaxed by mounting TWO rear tags angled +/-35 deg (use `DualTagSeeker`; carry
which tag produced a detection if the noise model needs it).

## Measure
v5 grid cells (nominal, aim 30, alt -2/+2, speed 4) x (camera, rear), all errors on, n = 100:
v5 baseline vs A vs A+B. Then the B3 and C curves. Headline table at the end.

Rules as before. Parameters travel through `Scenario`/`Scatter`/`pursuit_overrides`. Verify
with `.venv/bin/python -m pytest isim/tests -q`.
