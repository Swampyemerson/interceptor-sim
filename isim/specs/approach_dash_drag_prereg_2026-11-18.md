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
