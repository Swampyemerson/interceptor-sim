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
  6.5 m, lead 16.2 m — ROTATED 90 deg so the target flies world +X: the
  mover's position-only set_pose resets the board to identity each tick, and
  the identity board faces -X, so +X is the only rear-facing track the
  unmodified mover can fly (harness refuses --tag-yaw-deg for this reason).
  The isim prediction is rotation-invariant, so it stands unchanged. Belief
  seed from the same `--target-start/--target-vel` the mover flies.
- `--engage-max-s 25` passed explicitly (the flight default 12 s is sized for
  the fly-by; ADR-0105 already flags chase configs must raise it via the
  banner, never silently).
- One fresh sim boot per flight (the check_m4 clean-state discipline).
- Amendment note (2026-09-23, still before any flight): this geometry/config
  block was updated after the harness build disclosed the two items above;
  prediction, criteria, and null-meaning are untouched.
- n = 8 flights, identical geometry (Gazebo's own run-to-run variation is the
  noise source; no injected scatter). Engagement window 25 s.
- Scoring: centre-to-centre CPA from the sim-time-stamped own track vs the
  mover's commanded path, interpolated. Ground truth is scoring-only.

## Prediction (generated BEFORE the flights)

`scripts/xcheck_isim_prediction.py` — the isim port arm re-run with the
GAZEBO camera and tag substituted (fx 540.3, tag 0.5 m), same canonical
geometry, seeds 0..49 (`logs/xcheck_isim_prediction_20260923.csv`):

    PRE-FIX code:  median 0.112 m · p10 0.034 · p90 0.525 · 84% <= 0.35 m
    POST-FIX code: median 0.122 m · p10 0.064 · p90 0.195 · 100% <= 0.35 m

(Regenerated after the flight-1 yaw defect fix, BEFORE any scored flight's
result was read — the prediction must describe the code actually flying.
The fix also closed the registered grid's aim20 residual in isim proper:
70% -> 100% <= 0.35 m, median 0.063 m; alt+2 68-76% unchanged within n=50
noise, its mechanism being the separate no-re-approach design question.)

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

## RESULT (2026-09-23, n=8 flown as registered)

CPAs, sorted (m): 0.549 · 0.839 · 1.007 · 1.240 · 1.780 · 1.862 · 2.327 ·
2.334. Median **1.510 m**. Detections consumed 46-67 per flight; 0 aborts;
every flight ended SAFE via breakoff_complete. Logs:
`logs/xcheck_gz_20260923_fixed/` (per-tick driver CSV, gt-pose scoring CSV,
run+sim logs per flight; the discarded shakedown flight and its defect
diagnosis: `logs/xcheck_gz_20260923/`).

Scored against the registered criteria:
1. >= 6/8 inside 1.0 m — **FAIL** (2/8)
2. median <= 0.5 m — **FAIL** (1.510 m)
3. >= 1 flight <= 0.35 m — **FAIL** (0/8)
4. zero failsafe aborts — PASS
5. camera driving on every flight — PASS

**Registered verdict: FAIL (the NULL branch).** What survives, in the
pre-registered direction-only language: the ported terminal DOES control a
real-physics vehicle through a camera-driven chase in an independent
simulator — acquisition, tracking, and a close to ~0.5-2.3 m, with the state
machine ending the engagement cleanly. What does not survive: the CPA does
not transfer (Gazebo median 12x the matched-optics isim prediction), so
**the default-terminal swap stays BLOCKED** and no chase-only capability
claim may cite isim numbers as if they were vehicle-level.

Per the registered null path, the next step is the tick-trace diagnosis on
the Gazebo CSVs (the parity-trace instrument, pointed at the registered
suspects: the fixed 45 ms latency vs Gazebo's true render-to-consume delay,
the 0.5 m tag vs the noise floor shaped on 0.30 m, detect-in-loop tick
stretching, and the real EKF/controller dynamics isim's own-state model
does not carry) — NOT tuning.

**Bonus finding, already fixed in-contract:** the shakedown flight exposed
parity defect #5 (Phase A yawed at the rendezvous point, freezing the camera
off-target when station-keeping; the prototype yaws at the believed target).
Fixed to match the prototype; the fix also closed the isim grid's aim20
residual (70% -> 100% <= 0.35 m) and is pinned by a regression test that
fails against the pre-fix code.
