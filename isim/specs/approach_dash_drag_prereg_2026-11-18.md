# Approach dash with air drag: robustness screen (registered before flying)

Builder question 2026-11-18: should the sim have air drag, or does the guidance
correct for it anyway? Gazebo's airframe has no body drag (`docs/sim_to_real_gaps.md`
A1), and the 36 m/s (80 mph) approach-dash takes were flown there. This screen puts
body drag into the fast sim's TRUE world only (guidance is never told) and asks
whether the approach dash still intercepts. **isim ranks, Gazebo and the airframe decide.**

Script: `scripts/approach_dash_drag_sweep.py`. Rows: `runs/approach_dash_drag_20261118/`.

## Instrument check (done before registering; not a result)

`--stage check` reproduces four seed-0 rows of the October table
(`runs/cinema_longrange_20261021/dash_sweep.json`) to six decimals: miss, time of
closest approach, peak speed. So this runner is the October setup: real flight code
(port arm), rear 0.5 m tag, fx 540.3, ENGAGE fit, pose-range on, decode thinning
0.096 s, brake shaping with vert-sync, target 12 m/s, 120 s window. The October
16 m/s row turned out to be the legacy control (approach dash off).

Not reproduced: the October "far crossing" table (`dash_sweep_far.json`). Its
geometry is not written in the spec and its script was not committed. The one
failure it holds (36 m/s with brake a=3, 0/8) therefore cannot be re-examined
directly; a=3 at 36 m/s is kept as a profile here for that reason.

## Config

- **Geometries (cross / lead, m):** near 6.5 / 500 (October's registered one),
  demo 150 / 477 (the Gazebo take's), tail 500 / 500 (longest transit).
- **Profiles (dash cap m/s, brake m/s²):** 20 / 3 (October's pick), 36 / 4 (the
  80 mph take), 36 / 3 (October's failure profile).
- **Drag, quadratic, horizontal, added to the fitted linear 0.168 1/s:** D0 none
  (as flown), D1 0.0086 1/m (EXPECTED: Cd·A 0.014 m², 0.853 kg, air 1.042 kg/m³),
  D2 0.0171 1/m (WORST: twice that; the pod and a steep lean present more area).
  The Cd·A figure is an assumption (`docs/airframe_numbers.md` §8), so D1 is a
  guess with a doubled guess beside it. Bench-measurable: lean angle at a steady
  known speed.
- **Plants:** X = the ENGAGE fit as fitted (lean limit 37.7°, thrust 1.86 g): the
  Gazebo x500. R = the same fit with lean limit 60° and thrust 4 g: the demo
  envelope's lean limit and a thrust ceiling below what the real airframe is
  estimated to have. R extrapolates the fit; every other constant is the x500's.
- **Stage 1:** no scatter, 16 paired seeds, 3 geometries x 3 profiles x 3 drag x
  2 plants = 54 cells, 864 runs.
- **Stage 2:** all errors on (`Scatter()`), 50 paired seeds, plant R, profiles
  20 / 3 and 36 / 4, geometries near and demo, target straight and weaving,
  3 drag tiers = 24 cells, 1200 runs.
- **Kill:** closest approach 0.35 m or less (the ram radius).

## Predictions (written before any drag run)

1. **Top speed.** By hand, drag caps peak speed below the 36 m/s cap everywhere:
   plant X about 22 m/s at D1 and 17 at D2; plant R about 31 at D1 and 24 at D2
   (R's horizontal acceleration ceiling is 13.5 m/s²). D0 reaches the cap on both.
2. **Intercepts hold (the main prediction).** The dash is flown as speed commands
   against own position and speed, and the brake reads real speed. So in Stage 1
   every cell with drag keeps 16 / 16 kills wherever its D0 twin has them, with
   median miss within twice the twin's. Speed at first decode stays near the
   D0 value because the brake schedule sets it, not the dash.
3. **Time.** Time to closest approach grows with drag by seconds, not tens of
   seconds: the 36 / 4 cells lose most, ending near the 20 m/s cells' times.
4. **Stage 2.** Kill rate with drag is within 10 percentage points of the D0 twin
   in every cell (paired seeds). Weaving targets: I expect the slower transit to
   cost a few points at D2 and nothing resolvable at D1.

## What counts as a finding, and what a null means

- **Finding:** in Stage 1, drag turns 2 or more of 16 paired seeds from kill to
  miss in any cell; in Stage 2, the paired kill/miss discordance between a drag
  cell and its D0 twin is 6 or more net out of 50. Either sends me to the traces
  for the mechanism before anything is concluded.
- **Null (prediction 2 and 4 hold):** the fast sim cannot see a drag effect on
  intercept reliability. Then drag's exposure is (a) transit time and top speed,
  which prediction 1 already says are not the sim's 80 mph, and (b) what isim does
  not model: props making less thrust in fast air, drag that changes with lean,
  and height hold when the lean limit is saturated. The 80 mph figure then stands
  as a no-drag sim number, and a real fast leg measures the rest.
- **Not a lever test.** Nothing is adopted or rejected on this screen. It decides
  whether drag belongs on the list of things that can make the approach dash miss.

## Known limits, declared up front

- Arm asymmetry: none by construction. Every drag cell and its D0 twin share
  seeds, plant, profile and geometry; only the truth-side drag differs.
- The ENGAGE fit was earned at 16 to 18 m/s and below. Every cell above that
  extrapolates the plant, as the October spec already said.
- isim's vertical channel is soft and unfitted. A height excursion under a
  saturated lean is logged (`alt_drop_max`) but is not evidence either way.
- Quadratic drag here acts per axis and does not depend on lean.

## AMENDMENT 1 (2026-11-18, written after the first Stage 1 run, before the rerun)

**The first Stage 1 run is void for plant X with drag, and suspect for plant X
without it.** Every 36 m/s cell on plant X with drag came back 0 / 16 with no
detections. The rows show why: the vehicle climbed about 2,500 m. That is the
fast sim's vehicle model, not the guidance. `isim/vehicle.py` sized thrust from
the unclipped demand and applied it along the lean-clipped direction, so a
vehicle held on its lean limit pushed the surplus upward and climbed without
bound. Drag plus a cap it cannot reach holds it on the limit for the whole
transit. Even the no-drag October-style rows rose 9 m while accelerating.

- **Instrument repair:** `VehicleParams.tilt_sat_vertical_priority` (default
  off, byte-identical; the instrument check still matches October with it off).
  On, thrust under a clipped lean is sized from the vertical demand, which is
  what PX4's position controller does. Shown working before the rerun: near,
  36 / 4, D1, plant X, seed 0 went from a 118.7 m miss with a 2,556 m climb to
  a 0.073 m miss with a 0.1 m climb, peak speed 21.3 m/s (prediction 1 said
  about 22).
- **What changes in the design:** plants X and R both run with the repair on.
  Nothing else changes: same cells, seeds, predictions and finding thresholds.
  The first run's rows are kept as `stage1_void_asfitted.json`.
- **A consequence for the October spec.** Its "one real cliff" (36 m/s with
  brake a=3 at the far crossing: 0 / 8, no decodes, closest about 150 m at 17 s)
  has the same signature as these void rows (no decodes, closest 104 to 193 m
  at 16 to 26 s). I cannot rerun that geometry, so this is a probable cause, not
  a shown one: that cliff was likely the sim climbing away, not the brake
  schedule. "Do NOT fly 36 / 3" should be read as unproven until someone
  reproduces it with the repair on.
- **Plant R cells of the first run were not on the lean limit** (peak lean 54°
  against a 60° limit) and their misses stand as observed; they are rerun anyway
  so every cell shares one model.

## STAGE 1 RESULT (2026-11-18, rerun with the repair on; 864 runs, 54 cells)

Rows: `runs/approach_dash_drag_20261118/stage1.json`.

- **Prediction 1 (top speed): held.** Peak speed with the 36 m/s cap: plant X
  21 to 23 m/s at D1 and 17 to 18.5 at D2; plant R 30 to 33.5 at D1 and 24 to
  27 at D2. Nothing with drag reached 36. D0 reached 32 to 35.5.
- **Prediction 2 (intercepts hold): held at D1, failed at D2.**
  At D1 (the expected drag): 287 of 288 kills; the one miss is 0.37 m. No cell
  loses 2 seeds, so no finding at D1.
  At D2 (doubled drag): 270 of 288. Five cells lose 2 or more of 16, which is a
  finding by the registered threshold: demo 20/3 R (12/16), demo 36/3 R
  (12/16), demo 20/3 X (13/16), near 36/3 X (13/16), near 36/4 R (15/16, one
  miss at 0.36 m; listed for completeness, below threshold).
  At D0: 288 of 288.
- **Median miss grows with drag in every cell** even where every run is a
  kill: about 0.02 to 0.05 m at D0, 0.05 to 0.10 at D1, 0.08 to 0.18 at D2.
- **Prediction 3 (time): held.** Time to closest approach grows 1 to 3 s at D1
  and 3 to 6 s at D2 on plant R; up to 16 s on plant X in the tail geometry.
- **Mechanism of the D2 misses, from the state machine's own reasons.** 13 of
  the 19 misses end `target_lost(2.0s)` then BREAKOFF then SAFE. In all 13 the
  closest approach (0.75 to 3.5 m, about 7 m/s closing) comes at or before the
  first decode: the target passes the vehicle at the merge, the vehicle then
  has to chase a target that is pulling away, loses sight of it, and 2.0 s
  later the wrapper gives up. The other 6 end `pursuit_miss` between 0.36 and
  0.80 m with about 1 m/s closing: a slow final close that ends just outside
  the ram radius.
- **The October "cliff" profile (36 / 3)** shows no cliff here at D0 or D1 in
  any of the three geometries.

## AMENDMENT 2 (2026-11-18, registered before flying): the lost-target clock

The 13 lost-target misses point at one constant. `MissionConfig.engage_lost_target_s`
is 2.0 s; the pursuit terminal's own return-to-belief-chase fallback
(`PursuitTerminalConfig.fallback_s`) is 3.0 s. If the wrapper's clock fires
first, the terminal's fallback can never run. An earlier registered A/B
(`isim/specs/parity_flightcode_2026-09-21.md`, clocks of 2 / 4 / 10 / 30 s) was
a byte-identical null, but at short range with a 9 m/s chase, where the loss
happens inside 3 m and is permanent. This regime loses the target farther out,
so that null does not carry over (the regime rule).

- **Stage 3 config:** plant R and X, the four cells that lost 2 or more seeds
  plus near 36 / 4 as a clean control, drag D0 / D1 / D2, clock 2.0 (flying) /
  3.5 / 6.0 / 10.0 s, the same 16 seeds. Truth-side only apart from the clock.
- **Prediction:** at 3.5 s and above, at least 10 of the 13 lost-target misses
  become kills, because the terminal's fallback re-flies the belief chase and
  the camera re-acquires. Cells with no lost-target miss do not get worse (their
  kill counts are unchanged; the miss is the minimum over the window, so a longer
  window cannot raise it).
- **A null** (fewer than 4 of 13 convert) means the loss is as permanent here as
  it was at short range, and the fix has to come from how the vehicle arrives
  (speed and geometry at the merge), not from waiting longer.
- **Adopt / reject:** nothing is adopted on this arm alone. A positive result
  earns a Stage 2 style run with all errors on, and the safety cost has to be
  stated beside it: a longer clock is more seconds of blind flight near the
  target after a real miss.

## STAGE 2 RESULT (2026-11-18; all errors on, 1200 runs, 24 cells of 50)

Rows: `runs/approach_dash_drag_20261118/stage2.json`.

- **Prediction 4 held for the expected drag and failed for doubled drag on a
  weaving target.** Paired against the D0 twin, net kills lost: D1 between -2
  and +3 of 50 in every cell (inside the 6-of-50 threshold). D2 straight target
  +1 to +3 (inside). D2 weaving target +7 in both near cells (a finding), +2 and
  +4 in the demo cells.
- **The larger result is not about drag.** With the standing error package on,
  the 500 m approach dash kills in only 13 to 17 of 50 runs against a straight
  target at D0, and 6 to 8 of 50 against a weaving one. In 730 of the 1200 runs
  the camera never decodes the tag at all, and the run ends still in ENGAGE at
  the 120 s window. Of the 468 runs that did decode, 243 are kills.
  In the no-decode runs the closest approach is 12.5 m median (4.5 to 35 m,
  10th to 90th percentile), nearly all of it horizontal, and the tag is inside
  the picture for hundreds of frames without decoding: the vehicle is near the
  target but not behind it where the rear-facing tag can be read.
- **So October's "every cell intercepts" was a statement about a 2.5 m class
  cue.** Its sweep had no scatter. This is the first time the 500 m dash has met
  the error package every other A/B in this repo runs under, and it mostly does
  not acquire. Stage 2b below asks which error does it.

## AMENDMENT 3 (2026-11-18, registered before flying): the real board's envelope

Read off the interceptor's flight controller today, read-only
(`runs/fc_readonly_20261118/interceptor_params.txt`): speed limit 12 m/s,
horizontal acceleration 5 m/s², lean limit 45°. The guidance commands up to
36 m/s and is not told about the limit. `VehicleParams.max_setpoint_speed_horiz`
(default off) now models that clamp on the truth side.

- **Stage 4 config:** plant X with the repair on, guidance profile 36 / 4
  unchanged, three envelopes: stock (12 m/s, 5 m/s², 45°), mid (20, 8, 55°,
  thrust 4 g), demo (36, 15, 60°, 4 g); target 9 and 12 m/s; three geometries;
  drag D0 and D1; no scatter; 16 seeds. 72 cells, 1152 runs.
- **Prediction:** on the stock envelope a 12 m/s target is not caught in any
  geometry that ends as a tail chase, because the vehicle cannot go faster than
  the target (0 of 16 in tail, and demo and near decided by whether the merge
  itself puts the vehicle inside decode range behind the target; I expect under
  half). A 9 m/s target is caught on stock (16 of 16) but late. Mid and demo
  catch both speeds. D1 changes nothing on stock (drag at 12 m/s is small).
- **A null** (stock catches the 12 m/s target as often as demo) would mean the
  merge geometry, not top speed, does the work and the envelope question is
  less urgent than it looks.
- This informs the builder's open envelope decision; it adopts nothing.

## AMENDMENT 4 (2026-11-18, registered before flying): which error loses the target

- **Stage 2b config:** straight target, plant R, D0, profile 36 / 4, near and
  demo geometries, 50 paired seeds, and seven error packages: full; full
  without the target-speed belief error (10 % one sigma); without the heading
  error (2° one sigma); without wind and gusts; without own-state errors;
  without vehicle-parameter scatter and extra latency; and full with the speed
  belief error cut to 4 % (about 0.45 m/s at 12 m/s, the cue plan's own
  velocity-error figure). Rows gain the miss split along and across the
  target's track at closest approach.
- **Prediction:** the speed-belief error is the dominant cause. Without it the
  no-decode share falls from about 61 % to under 15 %; at 4 % it falls to under
  30 %. Heading error is second. The other three packages change the kill count
  by 3 of 50 or less. In the no-decode runs of the full package the miss lies
  mostly ALONG the track (the vehicle ahead of or behind the target), not across.
- **A null** (no single removal recovers half of the lost decodes) means the
  loss is spread across errors and the dash needs a search on arrival whatever
  the cue quality.

## STAGE 3 RESULT (2026-11-18; the lost-target clock, 960 runs, 60 cells of 16)

Rows: `runs/approach_dash_drag_20261118/stage3.json`.

- **The earlier null does not carry over to this regime.** Of the 13 runs that
  end `target_lost` at the flying 2.0 s clock, 9 become kills at 3.5 s and 10 at
  6 s and at 10 s. I predicted at least 10 at 3.5 s and above: one short at
  3.5 s, met at 6 s. Of the 2 `pursuit_miss` runs in these cells, 1 becomes a kill.
- **Nothing got worse.** Across all 60 cells no run that was a kill at 2.0 s is
  a miss at a longer clock, and the D0 and D1 cells stay 16 of 16.
- **What is left:** 4 misses at 6 s, between 0.42 and 0.97 m. Three end
  `pursuit_miss` (the vehicle re-closed to inside 3 m and missed narrowly); one
  is lost again.
- **Reading.** The wrapper's 2.0 s clock does pre-empt the terminal's own 3.0 s
  return to the belief chase, and when the target is lost while still ahead and
  catchable that costs kills. Evidence so far is doubled drag with no other
  errors, 13 runs. Stage 5 asks the same question under the full error package,
  where 117 of 1200 Stage 2 runs ended the same way.

## AMENDMENT 5 (2026-11-18, registered before flying): the clock under all errors

- **Stage 5 config:** the Stage 2 cells at D0 (straight and weaving target, near
  and demo, 20 / 3 and 36 / 4, plant R, all errors on, 50 paired seeds), clock
  3.5 s and 6.0 s. The 2.0 s arm is Stage 2 itself (same seeds, same config).
  800 new runs.
- **Prediction:** of the Stage 2 D0 runs that ended `target_lost` then BREAKOFF,
  at least a third are kills at 6.0 s. Kill counts rise by 2 to 6 of 50 per
  cell at 6.0 s, weaving cells gaining most. No cell loses more than 1 kill.
  The no-decode runs are untouched (the clock only starts after a decode), so
  this cannot fix the larger Stage 2 problem.
- **A null** (fewer than 1 in 6 converted, or any cell down by 3 or more) means
  the 13-run result was specific to drag and the clock stays at 2.0 s.
- **Adopt / reject:** a positive result makes "3.5 to 6 s lost-target clock when
  the target was last seen beyond the 3 m miss range" a candidate for a Gazebo
  rung and a builder ruling. It changes how long the aircraft flies blind near
  the target after losing it, which is his call, so nothing is switched here.

## STAGE 2b RESULT (2026-11-18; which error loses the target, 700 runs)

Rows: `runs/approach_dash_drag_20261118/stage2b.json` (profile 36 / 4, plant R,
D0, straight target, 50 paired seeds per package).

| Error package | near: kills, no decode | demo: kills, no decode |
|---|---|---|
| full | 16 / 50, 31 | 17 / 50, 30 |
| without the target-speed belief error | 50 / 50, 0 | 45 / 50, 6 |
| speed belief error cut to 4 % | 27 / 50, 21 | 31 / 50, 18 |
| without heading error | 18 / 50, 29 | 20 / 50, 27 |
| without wind | 16 / 50, 31 | 17 / 50, 30 |
| without own-state errors | 16 / 50, 31 | 17 / 50, 30 |
| without vehicle scatter | 16 / 50, 31 | 17 / 50, 30 |

- **Prediction held on the cause, missed on the 4 % level.** The target-speed
  belief error is the whole effect: without it the no-decode share is 0 % (near)
  and 12 % (demo), against my "under 15 %". At 4 % (about 0.48 m/s, the class
  the cue plan hopes for) it is still 42 % and 36 %, against my "under 30 %".
  Heading is a distant second; wind, own-state and vehicle scatter change nothing.
- **Where the target is when it is never decoded.** At closest approach the
  target is BEHIND the vehicle in 28 of 31 no-decode runs (near) and 25 of 30
  (demo): median 7.6 and 9.6 m behind, out to about 30 m, with 6 to 7 m of
  cross-track offset that the heading error supplies. The vehicle parks 8 m
  behind where it BELIEVES the target is, facing forward. If the belief runs
  fast, the real target is behind the vehicle, and a rear-facing tag cannot be
  read from in front. The runs where the belief runs slow leave the target
  ahead and in view, and those are the runs that decode.
- **Arithmetic check.** 10 % of 12 m/s is 1.2 m/s; over a 22 to 27 s transit
  that is 26 to 32 m of along-track error (one sigma) at arrival. The same 10 %
  at the 16 m canon geometry is a few metres, which is why the short-range
  studies never saw this.

## STAGE 4 RESULT (2026-11-18; the flight controller's envelope, 1152 runs)

Rows: `runs/approach_dash_drag_20261118/stage4.json` (guidance profile 36 / 4
unchanged, no scatter, 16 seeds per cell; kills of 16, then time to closest
approach).

| Target | Geometry | stock (12 m/s, 5 m/s², 45°) | mid (20, 8, 55°) | demo (36, 15, 60°) |
|---|---|---|---|---|
| 9 m/s | near | 16, 34 s | 16, 28 s | 16, 24 s |
| 9 m/s | demo | 16, 34 s | 16, 27 s | 16, 23 s |
| 9 m/s | tail | 15, 69 s | 16, 38 s | 16, 28 s |
| 12 m/s | near | 0 (0.76 m) | 16, 27 s | 16, 24 s |
| 12 m/s | demo | 0 (6.0 m) | 16, 26 s | 16, 23 s |
| 12 m/s | tail | 0 (never decodes) | 16, 38 s | 16, 28 s |

(D0 shown; D1 differs by at most one kill and 1 to 2 s.)

- **Prediction held.** On the stock envelope the vehicle cannot fly faster
  than a 12 m/s target and never catches it: it sits 0.76 m off the target's
  tail in the near geometry with the tag decoding the whole time, 6 m off in
  demo, and never gets within sight in the tail chase. A 9 m/s target is caught
  on stock, but the tail chase takes 69 s, which is past the flight program's
  default 30 s engage window.
- **The mid envelope catches everything the demo envelope does,** 2 to 10 s
  later. Under the expected drag the demo envelope does not reach 36 m/s
  anyway (Stage 1).
- **Reading for the open envelope decision.** The stock limits rule out the
  12 m/s target and make the 9 m/s goal marginal in a chase. Something like
  20 m/s, 8 m/s², 55° buys nearly all of the benefit. No scatter here, so this
  says nothing about the cue-error problem of Stage 2.

## INSTRUMENT CAVEATS found by the guidance review (2026-11-18, apply to every stage)

`docs/guidance_review_2026-11-18.md`. The sweep camera is the October
Gazebo-matched one, not the flying one: fx 540 (about 100° wide), a 0.5 m tag
and ZERO camera up-tilt. The review reran four doubled-drag cells with 10° of
up-tilt: 50 of 64 kills became 64 of 64, none broke. With its reading of the
flying lens and tag (fx 933, 0.30 m) the EXPECTED drag gives 7 of 16 at zero
tilt and 16 of 16 at 10°. So the drag losses here are a lean-against-field-of-
view effect, they depend on optics this screen did not match to the aircraft,
and "expected drag changes nothing" holds for the October optics only. The
scorer also counts a pass inside 0.35 m with no decode as a kill (11 of 245 in
Stage 2).
