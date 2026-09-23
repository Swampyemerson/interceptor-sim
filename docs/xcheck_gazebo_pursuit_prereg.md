# Pre-registration: Gazebo cross-check of the chase-only pursuit terminal

Registered 2026-09-23, BEFORE any Gazebo flight of this arm. This is the last
open gate from `docs/next.md` item 0(c) before the pursuit terminal may become
the flying default (ADR-0103/0105; parity vs the isim prototype CLOSED
2026-09-22, `isim/specs/parity_trace_2026-09-22.md`).

## Question

Does the ported pursuit terminal (`flight/pursuit_terminal.py`, driven through
the UNMODIFIED `RealFlightSM` and live MAVSDK OFFBOARD) transfer from isim to
an independent simulator with real physics, a real EKF, and a real rendered
camera — or do its vehicle-response and measurement assumptions break?

## Configuration

- Gazebo Harmonic + PX4 SITL, `gz_x500_mono_cam`, world `apriltag` (headless,
  GPU render, idle machine, one sim at a time).
- Harness: `scripts/gazebo_pursuit_crosscheck.py` — drives the unmodified
  flight code with a real detector fed by the gz camera topic (AprilTag
  decode, tag_size 0.5 m, intrinsics from CameraInfo, fx ~= 540.3).
- Geometry: the canonical isim crossing — target speed 9 m/s, cross-range
  6.5 m, lead 16.2 m, tag pre-yawed so its face is anti-parallel to the
  target velocity (rear-facing to the chaser; constant for a straight leg).
  Belief seed from the same `--target-start/--target-vel` the mover flies.
- n = 8 flights, identical geometry (Gazebo's own run-to-run variation is the
  noise source; no injected scatter). Engagement window 25 s.
- Scoring: centre-to-centre CPA from the sim-time-stamped own track vs the
  mover's commanded path, interpolated. Ground truth is scoring-only.

## Prediction (generated BEFORE the flights)

`scripts/xcheck_isim_prediction.py` — the isim port arm re-run with the
GAZEBO camera and tag substituted (fx 540.3, tag 0.5 m), same canonical
geometry, seeds 0..49 (`logs/xcheck_isim_prediction_20260923.csv`):

    median 0.112 m · p10 0.034 · p90 0.525 · 84% <= 0.35 m · 96% <= 1.0 m

The isim vehicle model is itself fitted to Gazebo x500 flights, so this IS
the prediction of the same flight code in Gazebo if the port transfers.

## Adopt / reject criterion (registered before flying)

The cross-check PASSES iff, over the 8 flights:
1. >= 6/8 reach CPA <= 1.0 m, AND
2. the median CPA <= 0.5 m, AND
3. >= 1 flight reaches CPA <= 0.35 m, AND
4. 0 flights end in a failsafe abort (target_lost/system-health BREAKOFF
   before the pass counts as an abort; a post-pass ending does not), AND
5. every flight consumes camera detections through ENGAGE (the KF is driving;
   a ballistic pass with zero consumed detections is a FAIL of this clause
   regardless of CPA — the anti-mirage rule).

Deliberately looser than the isim numbers: Gazebo adds EKF/control noise that
isim's own-state model does not carry, and this is a BEHAVIOR-TRANSFER check.
Per the arm-asymmetric-instrument rule, a PASS licenses the DIRECTION claim
("the port controls a real-physics vehicle to a camera-driven chase intercept
in an independent simulator"), never a quantitative margin quote between the
two simulators.

## What a PASS means / does not mean

- PASS => the `docs/next.md` 0(c) gate clears; recommending the default-
  terminal swap becomes a builder decision with evidence behind it.
- PASS does NOT mean a real-world claim: the Gazebo camera is not the OV9281,
  there is no wind, the launch cue is still error-free here, and the target
  is a teleported billboard tag.

## What a NULL/FAIL means

If flights abort or CPA lands far off the prediction, the port's assumptions
do not transfer; the default swap stays BLOCKED. Next step is the tick-trace
diagnosis on the Gazebo CSVs (same instrument as the parity trace), NOT
tuning. Specific known-risk suspects, written down now: the fixed 45 ms
`meas_latency_s` (Gazebo's camera latency differs from the Pi's), the tag's
0.5 m size vs the 0.30 m the noise model's floor was shaped on, and
`RealFlightSM`'s no-re-approach after a missed first pass (isim showed that
exact failure at aim20/alt+2).
