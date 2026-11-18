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
