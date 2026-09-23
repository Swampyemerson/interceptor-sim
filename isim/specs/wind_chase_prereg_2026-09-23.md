# Wind vs. the closed-loop chase — pre-registration (2026-09-23)

Builder question being answered: *"As far as not knowing the wind coefficients
of the airframe — why would this matter? If the drone measures its position in
space and relative to the target, it should correct sudden gusts back onto its
desired path, right?"*

This doc is written BEFORE the sweep flies (standing pre-registration rule).
Sections up to and including "What a null / violation would mean" are frozen
before the first result; results and the written answer are appended after.

## Configuration

- Arm: the ported real-flight-code chase — `Scenario(concept="flyby",
  terminal="pursuit", tag_facing="rear")`, same cell mechanics as
  `scripts/adaptive_speed_ab.py`, all realistic errors ON (`scatter=Scatter()`:
  heading/height/speed-belief noise, vehicle-param scatter, camera calibration
  error, decode realism). Target speed 9 m/s, chase `v_max_ms` 16 (defaults).
- Wind model (NEW, this round): steady wind vector + Ornstein–Uhlenbeck
  ("colored"/smooth random) gusts at 40% of the steady magnitude, tau = 2 s
  (Dryden-ish low-frequency turbulence stand-in). Wind enters the vehicle
  model through its existing drag law: `a = ... - C*(v_vehicle - v_wind)`,
  i.e. drag opposes velocity RELATIVE TO THE AIR — a drag-proportional wind
  push, exactly the `a_dist = -MCOEF*(v - v_wind)` form.
  - **Coefficient disclosure:** the coupling coefficient is the model's
    fitted `drag_linear_horiz = 0.1586 s^-1` (`isim/fits/vehicle_gazebo_x500.json`)
    — the same order as the PX4 default `MCOEF = 0.15 s^-1` the open-loop
    wind->sag table assumed. It was fitted against ZERO-WIND Gazebo flights,
    so as a WIND coupling it is a PLACEHOLDER, not a measured airframe
    coefficient. This sweep therefore demonstrates the MECHANISM, not the
    magnitude, of wind response.
  - The TARGET is deliberately NOT wind-affected (it is a scripted
    constant-velocity track, matching the project's existing sim assumption).
    Honest caveat: a real target drone also drifts in wind — see caveats.
- **Gusts are HORIZONTAL-ONLY (amendment made during the pre-sweep shakeout,
  BEFORE the sweep flew).** A single-seed shakeout with 3-axis gusts (1.8 m/s
  sigma, seed 3) collapsed to 0/139 decodes — root-caused to the model's
  VERTICAL channel, which is unfitted placeholder-class twice over
  (`drag_linear_vert = 0.35` is an unfitted default; `kp_vel_vert` fitted to
  0.81, far softer than PX4's real Z-velocity loop): vertical gusts walked
  altitude ~1.5 m and dropped the tag out of the camera's vertical FOV. A
  "wind breaks the chase" verdict via an unfitted channel would be an
  instrument artifact, so vertical gust sigma is 0 in this sweep and the
  vertical-turbulence coupling is declared UNTESTED (see caveats).
- Cells: wind speed {0, 1.5, 4.5, 8} m/s x direction {headwind, crosswind,
  tailwind} **relative to the target's track** (track = north; headwind =
  wind blowing FROM the north, i.e. against the chaser's overtake direction).
  The 0 m/s cell is direction-free (one cell). Gust sigma = 0.4 x steady in
  every non-zero cell. n = 50 paired seeds per cell (seeds 0..49, same seeds
  every cell, so cell-to-cell deltas are paired).
- Config gating: the wind fields default OFF and the off-state is
  byte-identical to the pre-change code (verified by re-running two existing
  seeds and comparing the full position+velocity trace hashes before/after
  the edit — hashes recorded in the results section).

## Metrics (per cell)

1. % of runs closing inside 0.35 m (the project's contact radius).
2. Median CPA (closest-point-of-approach distance), m.
3. Median achieved overtake speed in the last 3 s before CPA: the chaser's
   ground-speed component along the target's track (north), minus the target's
   9 m/s — to expose the authority-margin mechanism if it appears.

## Predictions (frozen before flying)

1. Steady wind and gusts INSIDE the authority margin barely move contact %:
   the chase is closed-loop (Phase A re-converges on own-velocity feedback;
   Phase B's KF measures the relative state every decode), and the vehicle's
   velocity controller holds a GROUND-velocity setpoint by leaning into the
   wind, so a steady displacement never accumulates.
2. The 8 m/s HEADWIND cell degrades most, via eroded overtake margin: the
   command cap is 16 m/s ground speed against a 9 m/s target (7 m/s margin);
   a headwind spends tilt/thrust authority and shifts the achievable-airspeed
   ceiling down by roughly the wind component, eating that margin.
3. Crosswind and tailwind cells stay near the 0-wind cell (tailwind may even
   help the overtake slightly).

## Criterion for "wind-robust" (frozen)

Every cell within 10 percentage points of the 0-wind cell's contact %, except
the 8 m/s headwind cell (allowed to exceed 10 points, per prediction 2).

## What a null / violation would mean (frozen)

- If even the 8 m/s headwind cell stays within 10 points: at THIS placeholder
  drag coefficient (0.1586 s^-1, linear), 8 m/s of wind costs only ~1.3 m/s^2
  of the model's ~10 m/s^2 horizontal authority, so the closed loop absorbs
  it. That would MEAN: the mechanism (feedback absorbs wind inside authority)
  is confirmed across the whole swept range, and the remaining wind risk is
  entirely in the MAGNITUDE question — a real airframe with a larger true
  drag coefficient, or quadratic drag at speed, could still hit the wall this
  sweep failed to reach. It would NOT mean "wind never matters".
- If cells INSIDE the expected authority margin (1.5/4.5 m/s) degrade by >10
  points: the builder's intuition (and prediction 1) is WRONG in a way the
  mechanism story doesn't explain — suspect the gust model shaking the
  camera/attitude enough to break decodes, or an instrument/wiring bug in the
  new wind path; investigate before believing either direction.
- If TAILWIND degrades more than headwind: sign error in the wind wiring
  (instrument bug) until proven otherwise.

---

## Results (appended after the sweep, 2026-09-23)

Off-state identity: verified byte-identical — two pre-edit engagements
(seed 3 no-scatter, seed 7 scatter; full own_pos+own_vel trace SHA-256
`f4619ab3…` / `0fdbe6c8…`) reproduce exactly under the new code with the wind
fields at their defaults. Effect observed when on: a 4.5 m/s steady headwind
moves the seed-3 trajectory up to 2.3 m mid-chase and the closed loop still
makes contact (0.071 -> 0.093 m).

`scripts/wind_chase_ab.py --workers 10`, n=50 paired seeds/cell, isim
(fast surrogate — ranks and mechanisms, not a Gazebo/real-world gate):

| cell        | % <= 0.35 m | med CPA (m) | p90 CPA (m) | med overtake last-3s (m/s) | med decodes |
|-------------|------------|-------------|-------------|---------------------------|-------------|
| 0 (no wind) | 94%        | 0.117       | 0.245       | 2.31                      | 98          |
| 1.5 head    | 96%        | 0.136       | 0.289       | 2.32                      | 94          |
| 1.5 cross   | 90%        | 0.118       | 0.329       | 2.27                      | 96          |
| 1.5 tail    | 94%        | 0.111       | 0.216       | 2.36                      | 95          |
| 4.5 head    | 88%        | 0.164       | 0.375       | 2.54                      | 89          |
| 4.5 cross   | 90%        | 0.117       | 0.287       | 2.38                      | 95          |
| 4.5 tail    | 96%        | 0.103       | 0.318       | 2.38                      | 94          |
| 8 head      | 80%        | 0.175       | 0.518      | 2.35                      | 88          |
| 8 cross     | 92%        | 0.138       | 0.265       | 2.36                      | 90          |
| 8 tail      | 96%        | 0.097       | 0.266       | 2.37                      | 92          |

**Pre-registered criterion: PASS.** Every cell except 8-head is within 10
points of the 0-wind cell's 94% (worst: 4.5-head at −6, 1.5-cross at −4 —
at n=50 a ±4-point wobble is ~2 seeds, i.e. within run-to-run noise); 8-head
degrades most (−14 points), as predicted, and tailwind never beats headwind
(no sign-error signature).

**Mechanism, from the paired failures in the 8 m/s headwind cell** (8 newly
failing seeds vs the 0-wind arm): 7 of 8 are NEAR-misses (0.37–0.86 m) whose
decode counts dropped 25–40% (93→60, 65→32, 92→56, …) — the chase still
catches the target on schedule (median t_cpa 8.66 -> 8.78 s, +0.14 s paired),
but the terminal is working from fewer/worse camera fixes. The plausible path
is attitude: holding ground speed into an 8 m/s headwind at this drag
coefficient costs ~1.3 m/s² of horizontal thrust, i.e. ~7° of extra forward
lean, which shifts camera pointing and worsens the rear-tag decode geometry
(this coupling IS modelled — the sim's attitude comes from the thrust
vector — but pointing-loss was not instrumented separately here, so this
attribution is supported, not proven). 1 of 8 is a true acquisition failure
(74→4 decodes, 3.4 m miss): wind pushed that engagement out of the
acquisition envelope entirely. Overtake-margin erosion — the predicted
mechanism — is present but MILD at this coefficient (p10 overtake in the
last 3 s: 1.59 -> 1.30 m/s; medians unchanged), consistent with the frozen
null reading: 8 m/s of wind at 0.1586 s⁻¹ linear drag spends only ~13% of
the model's horizontal authority, so the margin wall stays out of reach.
A larger true coefficient (or quadratic drag at airspeed) moves that wall
closer — magnitude unresolved by design.

## Why wind coefficients matter, and when they don't (the builder's answer)

**(i) Where you're right.** For the CHASE, your intuition is correct, and the
code confirms it end to end. Phase A re-aims every tick using the measured own
velocity (`flight/pursuit_terminal.py` `step()`: `self._r_track = self._r_track
+ (self._v_track - own_vel) * dt` — a gust that shoves the vehicle shows up in
`own_vel`, so the believed target-relative position stays right and the
proportional command `v_track + kp_pos * r_aim` steers back). Phase B's Kalman
filter measures the target relative to wherever the vehicle actually IS
(camera pixels), and its predict step subtracts the measured own velocity
(`_ConstVelKF.predict(dt, v_own)`), so a wind-perturbed own velocity is
compensated, never assumed. Underneath both, the velocity commands go to a
controller that closes on GROUND velocity (PX4's velocity loop; modelled in
`isim/vehicle.py` as P+I on velocity error): against a steady wind it simply
leans the airframe into the wind and holds the commanded track — that loop,
not the guidance, is what rejects steady wind, and it needs NO knowledge of
the drag coefficient to do it (feedback measures the error the wind causes
and cancels it, whatever the coefficient was). That is exactly why the
unmeasured MCOEF/BCOEF sag table only ever mattered to the OPEN-LOOP sprint:
there, nothing measures and nothing corrects, so the aim solve must PREDICT
the wind's effect from a coefficient — and an unmeasured coefficient is an
unmeasured aim error. The chase retires that failure mode. Measured here:
0 -> 4.5 m/s wind in any direction moves contact rate by at most ~6 points
(~noise at n=50); gusts at 40% of steady included.

**(ii) What wind still costs even closed-loop.** Feedback cancels the ERROR,
but it spends AUTHORITY doing it — the tilt/thrust budget. A headwind (a)
shifts the achievable ground-speed ceiling down by roughly the wind
component, eroding the overtake margin against a fast target, and (b) forces
a constant forward lean plus gust-driven attitude motion, which moves the
CAMERA — and the camera is the terminal sensor. In the 8 m/s headwind cell
(50% of the 16 m/s cap, 89% of the 9 m/s target speed) contact dropped
94% -> 80%, and the paired failures show the cost arriving mostly through
decode counts (−25–40% camera fixes in the failing runs) with mild overtake
erosion; pointing jitter itself was not separately instrumented here. So
"the wind coefficient doesn't matter" is true for steady-state tracking, but
the coefficient still SIZES the authority tax: it decides at what wind speed
the margins run out.

**(iii) Two honest caveats.** First, the target here is scripted and
wind-immune (the project's existing sim assumption): a real target drone also
drifts in wind, which correlates the two aircraft's disturbances (a shared
steady wind partially cancels in the relative state a chaser cares about; an
evading or waypoint-holding target reacts differently). Untested either way.
Second, this wind model couples through the vehicle fit's
`drag_linear_horiz = 0.1586 s⁻¹` — fitted on ZERO-WIND Gazebo flights, the
same order as PX4's default MCOEF 0.15 s⁻¹, and never measured on this
airframe (and the vertical channel was excluded outright because it is
unfitted twice over — see the amendment above). Everything here therefore
demonstrates the MECHANISM (feedback absorbs wind inside authority; the cost
is margin, not tracking) and NOT the magnitude (the wind speed at which the
real airframe's margins run out). Getting the magnitude honestly means
measuring the real airframe's wind response — which is precisely why "we
don't know the wind coefficients" remains on the assumptions register even
though the chase design has made them stop being an AIM input.
