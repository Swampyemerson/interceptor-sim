# Approach-dash velocity curves — isim screen (registered before flying)

Builder directive 2026-10-21: long-range engagement — cue at ~500 m, dash the
transit "absolutely ripping" (~80 mph asked), decelerate into predicted tag
capture. Feature built config-gated the same night (`approach_dash` in
flight/pursuit_terminal.py: Phase-A cap widens to `dash_v_max_ms`, Phase-A
stopping-distance brake cap applied unconditionally). This screen picks the
velocity curve for the Gazebo demo; **isim RANKS, Gazebo decides**.

## Setup

Matched optics (fx 540.3, 0.5 m rear tag), ENGAGE fit (`vehicle_gazebo_x500_engage`),
pose-range on, decode thinning 0.096 s, flying brake package (a=3, vert-sync)
unless the cell overrides `brake_accel_ms2`. Geometry: crossing, lead 500 m,
cross 6.5 m, target 12 m/s, window 120 s, cue error 2.5 m-class (aim_error 0.3°
equivalent at that range — injected in the Gazebo arm instead). n=8 seeds/cell.

Sweep: `dash_v_max_ms` ∈ {16 (≈legacy control), 20, 25, 30, 36} ×
`brake_accel_ms2` ∈ {2, 3, 4} (fitted plant braking authority is 5.94 m/s²,
so a ≤ 4 leaves margin; a > authority would under-deliver the schedule).

## HONEST LIMITS, declared up front

- The vehicle fit was earned at ≤ ~16-18 m/s; every cell above that
  **extrapolates the plant**. The isim numbers rank profiles; the Gazebo arm
  (and ultimately the airframe) decides.
- The GAZEBO plant ceiling is PX4's velocity-mode MPC_XY_VEL_MAX = 20 m/s
  (param max). 36 m/s (80 mph) is NOT flyable in the sim demo; on the real
  5-inch it is an unmeasured airframe question (assumptions register:
  airframe-thrust-and-mass-unmeasured, dash-accel-profile).

## Registered prediction

Profiles whose schedule sheds the dash speed within the fitted braking
authority (a ≤ 4) arrive at the acquisition basket at normal chase speed and
match the short-range baseline's intercept quality (isim ~0.05-0.1 m median,
100% ≤ 0.35 m); higher dash speed buys transit TIME roughly linearly and
costs ~nothing in CPA. A null (fast cells losing lock / missing) would mean
the Phase-A brake cap mis-handles the long-range belief geometry and the
feature needs rework before any Gazebo flight.

**Pick rule:** fastest `dash_v_max ≤ 20` cell (the Gazebo-flyable ceiling)
with 8/8 lock and median miss ≤ 2× the 16 m/s control's, with `a` chosen as
the largest value ≤ 4 that doesn't degrade miss vs a=3 (shortest transit).
The >20 cells are reported as extrapolated-plant reference curves only.

## RESULT (2026-10-21, run as registered + a geometry screen)

Registered sweep (two geometries, n=8 seeds/cell; tables in
runs/cinema_longrange_20261021/dash_sweep*.json, curves in curve_*.csv):

* **Prediction CONFIRMED in the main:** every a<=4 cell at both geometries
  locks 8/8 and intercepts (median miss 0.014-0.067 m, 100% <= 0.35 m), and
  dash speed buys transit time (~55 s -> ~27 s at the far crossing) at ~no
  CPA cost. Arrival speed at first decode is set by the brake schedule, not
  the dash ceiling.
* **One real cliff, outside the pick range:** vmax=36 with a=3 at the far
  crossing fails 0/8 (closest ~150 m) while a=4 at 36 is clean -- at that
  speed the softer schedule's braking starts inside the window where the
  crossing-merge geometry has already folded; do NOT fly 36/3. (Extrapolated
  plant; reference only.)
* **Geometry pick for the demo (visible deceleration + long own transit):**
  cross 150 m / lead 477 m (exactly 500 m slant cue). At vmax 20 / a=3 the
  interceptor flies ~345 m of its own path, peaks 19.9 m/s, and arrives at
  first decode at ~6.6 m/s -- a 20 -> 7 braking profile the video can show.
  6/6 lock, median miss 0.045 m, t_cpa ~27 s.
* **PICK (per the registered rule): dash_v_max 20 m/s (the Gazebo-flyable
  ceiling), brake a=3 (the flying package constant), geometry 150/477.**
  Gazebo decides -- the demo takes are the next arm.

## GAZEBO ARM (2026-10-21 — addendum, flown after the pick)

Serial takes, one sim boot each, 500 m slant cue (cross 150 / lead 477),
injected cue error per take, full camera-in-the-loop (logs/cinema_lr/):

| take | envelope | CPA (m) | n_det | note |
|---|---|---|---|---|
| lr1 | 20 m/s, a=3 | 0.272 | 39 | per the registered pick |
| lr2 | 20 m/s, a=3 | 0.199 | 41 | per the registered pick |
| lr3 | 20 m/s, a=3 | 7.821 | 7 | cue-error draw opened range past the decode cliff; SAFE abort path exercised |
| hr1 | 36 m/s, a=4 | 0.380 | 42 | builder asked for the 80 mph class |
| hr2 | 36 m/s, a=4 | 0.697 | 45 | |
| hr3 | 36 m/s, a=4 | 0.453 | 40 | |
| hr4 | 36 m/s, a=4 | 0.298 | 63 | |
| hr5 | 36 m/s, a=4 | **0.087** | 64 | THE VIDEO TAKE; plant peak 35.8 m/s; first detection go+17.5 after a 17 s belief-only dash |

Gazebo verdict: the isim ranking TRANSFERRED. 20 m/s takes land 0.2-0.3 m;
36 m/s with a=4 (the cliff-aware brake) lands 0.09-0.70 m across five takes
(2/5 inside the 0.35 m ram radius — n too small for a rate claim, reported
as takes, not a Pk). The 36 m/s envelope needed a raw PX4 param set
(MPC_XY_VEL_MAX read back 36.0; the declared max 20 is GCS metadata only —
scripts/cinema/set_px4_params.py). All numbers are SIM; the real FC keeps
stock limits (read-only TELEM2 read 2026-10-21: 12/5/45/8) until the
airframe's thrust, mass and brake authority are measured
(builder_queue: dash-envelope-on-real-fc).
