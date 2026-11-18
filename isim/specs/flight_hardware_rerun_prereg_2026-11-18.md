# Approach dash on the flying hardware: rerun (registered before flying)

Builder, 2026-11-18: "why not use the flight hardware instead of the retired hardware
in these sims", "that 30-33 m/s is acceptable for the dash", "see if soft brake should be
implemented with that sim fix", "lost target clock?".

Today's screens (`isim/specs/approach_dash_drag_prereg_2026-11-18.md`) ran on October's
Gazebo-matched optics: fx 540.3 (about 100 degrees wide), a 0.5 m tag, no camera up-tilt,
and a vehicle envelope the real board does not have. This rerun uses the aircraft's own
numbers. **Fast sim only: it ranks and locates; Gazebo and the airframe decide.**

Script: `scripts/flight_hardware_rerun.py` (imports the drag sweep's builder and per-run
function unchanged). Rows: `runs/flight_hardware_rerun_20261118/`.

## Instrument check (done before registering; not a result)

`--stage check`: my wrapper, fed October's optics and plant R, reproduces all 16 rows of
the existing cell `near|v36a4|D1|R` (`runs/approach_dash_drag_20261118/stage1.json`) to
nine decimals. So every difference below comes from the configuration, not the runner.

## The flying configuration

| Item | Value used | Source | Measured or assumed |
|---|---|---|---|
| Camera and lens | OV9281 with the delivered lens, 1280 x 800 | ADR-0126, ADR-0131 ("the OV9281 + 70 degree lens remains the flying seeker") | ruling |
| Focal length | fx 1104 px | `configs/ov9281_intrinsics.json`: checkerboard, 81 views, 0.73 px rms, 2026-10-04 | MEASURED |
| Field of view in the sim | 60.2 x 39.8 degrees (pinhole at fx 1104) | derived | The real lens has strong barrel distortion (k1 -0.43): about 70 x 42 degrees. The sim's camera is a pinhole, so it is exact on pixels-on-target, 2 degrees short vertically and 10 degrees short horizontally. Bracket: fx 933, the repo's earlier 70 degree stand-in (69 x 46 degrees, 15 % too few pixels) |
| Principal point | image centre | sim limit | The calibration puts it 84 px right and 48 px low; not modelled (the real frame sees about 2.5 degrees more upward than the sim's) |
| Tag | tag36h11, 0.35 m black square | `docs/placard_sizing.md` (print-ready PDF, dimension checked) | design of record; ADR-0132 and older sims say 0.30 m. Not confirmed printed |
| Tag facing | rear | ADR-0103 ("the tag must face the arrival direction") and every pursuit study | **UNSETTLED.** `docs/placard_mount.md` decision 1 draws the placard beam-facing (edge-on to flight, printed both sides). Bracket: side-facing |
| Camera up-tilt | 10 degrees; also 15, 20, 25 | mount adjusts 10 to 45 (`hardware/avionics_pod/README.md`); ADR-0112 recommended +10 for the chase, at the wide lens | the plate is not set yet; this rerun is meant to inform it |
| Decode cadence | one attempt per 0.05 s | flight loop `--fps` 20 (`real_flight.py`); the Pi decodes at 96.6 fps (measured, decimate 2) | loop rate from code; decode rate measured |
| Decode model | isim default (50 % at 22 px) | `isim/seeker.py`; the Gazebo cliff at decimate 2 was about 22 px (`docs/placard_sizing.md`) | literature curve, one Gazebo anchor; no outdoor decode curve exists |
| Board speed limit | 33 m/s | written and read back today (`runs/fc_readonly_20261118/README.md`) | MEASURED (read-back) |
| Board lean limit | 45 degrees ("now"); 60 ("lean60", the open option) | same file | MEASURED / option |
| Board acceleration | 5 m/s² on the setpoint | same file (`MPC_ACC_HOR_MAX`) | read off the board; whether PX4 applies it to offboard velocity setpoints is NOT known |
| Thrust | 4 g | target drone measured about 5 g at full throttle (same motors and pack) | assumed, conservative |
| Everything else in the vehicle | the Gazebo x500 ENGAGE fit, repaired lean rule on | `isim/fits/vehicle_gazebo_x500_engage.json` | the x500, not this airframe |
| Drag | D0 none, D1 0.0086 1/m (expected), D2 0.0171 (doubled) | `docs/airframe_numbers.md` section 8 | assumed |
| Guidance | approach dash on, cap 33 m/s, brake shaping with vert-sync, brake 2 / 3 / 4 m/s², pose range on, target 12 m/s, 120 s window | as today's screens | |

## Config

- **Stage a (no random errors, 864 runs):** near / demo / tail x D0 / D1 / D2 x envelope
  now / lean60 x brake 2 / 3 / 4, tilt 10, 16 paired seeds.
- **Stage c (tilt, 576 runs):** tilt 15 / 20 / 25 at brake 3, D1 and D2, both envelopes,
  three geometries, 16 seeds. Tilt 10 is stage a.
- **Brackets (192 runs):** at D1, brake 3, tilt 10: fx 933 instead of 1104; side-facing tag
  instead of rear.
- **Stage b (all errors on, 2000 runs):** envelope now, D1, brake 3, tilt 10. Speed-belief
  error 10 / 4 / 2 / 1 / 0 % x straight / weaving x near / demo x lost-target clock 2.0 / 6.0 s,
  50 paired seeds. **btilt (200 runs):** the 4 % and 1 % straight cells at tilt 20, clock 2.0.
- Kill: closest approach 0.35 m or less. Reported twice: all kills, and camera kills (first
  decode before closest approach). The scorer counts a no-decode pass inside 0.35 m as a
  kill; those are listed as blind.
- Also counted: kills later than 30 s after GO, because the flight program's default engage
  window is 30 s and the sim's is 120 s.

## Predictions (written before any run with these optics)

1. **Stage a, kills.** At D0 and D1 with brake 3 or 4, every cell has at least 15 of 16
   kills on both envelopes. At D2 at least three cells lose 2 or more of 16: the narrower
   vertical view (20 degrees each side of the axis against October's 36) leaves less room
   for the lean of a catch-up.
2. **Brake.** No brake value has a cliff at D0 or D1: 2, 3 and 4 all intercept. Softer
   costs time: about 1.5 s per step at brake 3 and about 4 s at brake 2 against brake 4,
   and a lower peak speed. Speed at first decode stays within 1.5 m/s across brakes.
   **Rule fixed now:** fly the SOFTEST brake whose kills equal the best brake's in every
   D0 and D1 cell and whose median time to closest approach is within 2.0 s of the
   fastest. Softer is preferred because the airframe's real braking is unmeasured.
3. **Envelope.** now and lean60 give the same kills at D0 and D1. Peak speed under D1 is
   about 25 m/s on now and about 30 on lean60 (hand calculation), a time difference under
   2 s, because both are limited to 5 m/s² on the way up.
4. **Tilt.** 15 degrees is at least as good as 10 at D1 and D2 and better at D2. 25 degrees
   loses 2 or more of 16 in at least one D1 cell, because a level or braking vehicle then
   has a co-altitude target below the bottom of the picture. So I expect to recommend 15.
5. **Brackets.** fx 933 gives kills no lower than fx 1104 in every cell. The side-facing
   tag gives 4 of 16 or fewer in every cell: a chase from astern sees it edge-on.
6. **Stage b, straight target, clock 2.0, kills of 50:** 10 %: 14 to 22. 4 %: 28 to 38.
   2 %: 36 to 45. 1 %: 38 to 47. 0 %: 40 to 48. The larger tag-in-pixels (386 px·m against
   October's 270) lengthens the decode window by about 1.4 times; the slower climb to
   speed (5 m/s²) lengthens the transit and so the error. Weaving: 10 to 20 fewer.
7. **Lost-target clock.** 6.0 s gains 0 to 3 kills of 50 per cell and no cell loses more
   than 1; summed over the 20 cells the gain is positive.
8. **btilt.** Tilt 20 changes the 4 % and 1 % straight cells by 3 of 50 or less.

## What counts as a finding, and what a null means

- **Finding:** in stage a / c / brackets, a change that moves 2 or more of 16 paired seeds
  between kill and miss in a cell; in stage b, net paired discordance of 6 or more of 50.
- **A null on prediction 1** (D2 loses nothing) would mean the lean-against-view effect
  the guidance review found does not appear at 10 degrees of tilt with this lens.
- **A null on the brake** (all three identical in kills and within 2 s) means brake 2 by
  the rule, and October's "do not fly 36 with brake 3" does not survive in any form here.
- **If stage b's kills are no better than October's optics gave** (13 to 17 at 10 %, 27
  to 31 at 4 %), the flying optics do not rescue the long-range dash and everything in the
  arrival-geometry note stands as written.
- Nothing is adopted here. The brake value, the tilt and the clock are recommendations
  for the builder and for one Gazebo rung.

## Known limits, declared up front

- Arm asymmetry: each comparison changes one thing on shared seeds. The side-tag and tilt
  arms change what the camera can see, not what the vehicle does before the first decode.
- The vehicle is the Gazebo x500 fit with this board's limits laid over it. Its velocity
  integrator is weak and its height loop soft; both make drag look worse than PX4 would.
- The board's 5 m/s² acceleration limit is applied to the setpoint here. If PX4 does not
  apply it to offboard setpoints the real transit is shorter than these runs.
- The target is level and steady (no target lean or wobble), as in today's other screens.
- Engage window 120 s in the sim; the flight default is 30 s. Late kills are counted.

## AMENDMENT 1 (2026-11-18, written before any of the runs it adds)

Three builder rulings arrived after registration, and main gained the arrival search.
Stages a, c, brackets, b and btilt above run as registered; none had reported when
this was written. The stages below are added. Script: `scripts/flight_hardware_rerun_ext.py`.

**The rulings.**
1. Lean limit: "I don't care about lean limit as long as it doesn't affect the camera
   more than we can correct for reasonably." So the lean limit is free up to about
   60 degrees and the question becomes the camera: which lean limit and mount tilt
   keep the tag in the picture and give the most kills.
2. Engage time: "no limit at least for now". Nothing is cut at 30 s. Every stage
   reports how many kills come later than 30, 45 and 60 s after GO.
3. Lost-target rule: "whatever gets highest intercept rate, I'll be supervising so
   I can disengage."

**Added stages** (flying optics, D1, brake 3, board speed limit 33 m/s, 5 m/s²):

- **grid (no errors, 1008 runs):** lean limit 45 / 52 / 60 x mount tilt 10 / 15 / 20 /
  25 / 30 / 35 / 45 x near / demo / tail, 16 seeds. Each run also records, from truth
  and for scoring only: the share of camera frames with the tag outside the picture
  (a) in the chase, from first decode to closest approach, with the longest such gap,
  and (b) on arrival, while the target is inside 48 m (8 px) and not yet decoded;
  and the vehicle's pitch in those frames (nose down means the tag left by the top).
- **gridb (all errors, 32 seeds, 512 runs):** the three (lean, tilt) pairs with the most
  grid kills (ties: smaller chase out-of-picture share, then smaller tilt, then smaller
  lean) plus the (45, 10) baseline; speed error 4 %; straight and weaving; near and
  demo; lost-target clock 120 s (no limit, ruling 2 and 3). **Recommendation rule:**
  the pair with the most gridb kills; a tie goes to the grid order above.
- **lost (all errors, 32 seeds, 768 runs):** speed error 4 % and 1 %, straight and
  weaving, near and demo, envelope now, tilt 10: clock 20 s; clock 120 s; clock 120 s
  with the terminal's close-range rule off (`no_fallback_range_m` 0, so it re-chases
  after losing a target inside 3 m instead of coasting). Compared, on the same seeds,
  with stage b's 2.0 s and 6.0 s rows. The wrapper's close-miss rule (inside 3 m, go
  SAFE when the clock expires) is inert at 120 s, so the 20 s against 120 s rows show
  what it and the clock cost together.
- **search (all errors, 32 seeds, 1728 runs; needs main at 4044c45 merged first):**
  arrival search off / `stern` / `believed`, its declared speed sigma left at 10 %,
  clock 120 s in all three so the off arm is a fair twin. Lean 45: speed error
  10 / 4 / 1 % x straight / weaving x near / demo. Lean 60: the same, straight only.
  Tilt 10.

**Predictions for the added stages.**

9. **grid.** At tilt 10 the chase out-of-picture share is larger at lean 60 than at 45.
   Raising the tilt cuts it and raises the arrival out-of-picture share. Kills peak at
   tilt 15 to 20 at every lean limit; at tilt 35 and 45 every cell is below half. At its
   best tilt, lean 60 is within 1 kill of 16 per cell of lean 45 and 1 to 2 s faster.
10. **lost.** 20 s gains 0 to 2 of 32 per cell over 6 s; 120 s gains 0 to 1 over 20 s;
    no cell loses more than 1. Turning the close-range rule off changes any cell by 2 or
    fewer either way.
11. **search.** `stern` raises the 10 % straight cells by 10 or more of 32 over the off
    twin and the weaving cells by 5 or more; at 1 % it costs 2 or fewer. `believed`
    gains less than `stern` at 10 %. Lean 60 against 45 differs by 3 or fewer of 32.

**Nulls.** If no pair in the grid beats (45, 10) by 2 of 16 in any cell, the plate goes
to 10 degrees and the lean limit is a matter of time, not kills. If the search arms do
not beat their off twins with these optics, October's 17 to 46 result does not carry
over to the flying lens.

**What this cannot settle for the switches being built elsewhere:** "do not start the
clock before a track exists" and "hold the clock off during a search" are code in
`flight/deploy/real_flight.py`, which this rerun does not touch. A 120 s clock stands in
for both here; it also removes the clock where those switches would keep it, so it is
an upper bound on what they can give, not a test of them.

## AMENDMENT 2 (2026-11-18, written before the runs it changes or adds)

Written after stages a, c, brackets, b, btilt, grid, gridb and lost had reported, and
before the search stage or anything below ran. What I knew when writing it: tilt 20 gives
192 of 192 with no errors where tilt 10 gives 105; the grid's rule picked lean 45 with
tilt 20; at tilt 10 no brake value satisfies the brake rule because the losses there are
the camera's, not the brake's. Main is merged (arrival search, and three lost-target
switches with a flight preset). The instrument check passes again on the merged code.

- **The search stage moves to tilt 20**, the tilt the grid's rule chose, because that is
  the configuration a recommendation would fly. Lean 45: speed error 10 / 4 / 1 % x
  straight / weaving x near / demo x four arms: search off; `stern`; `believed`; and
  `stern` with the flight preset's lost-target switches (`FLY_PRESET`: clock not started
  before a track, held off during the search, 6 s when lost beyond 3 m) instead of the
  120 s clock. Lean 60: straight only, off and `stern`. 60 cells of 32 = 1920 runs.
- **A tilt-10 subset is kept** so the search can be compared at both tilts: lean 45,
  straight, near and demo, 10 % and 4 %, off / `stern` / `believed`, clock 120 s. 12 cells
  of 32 = 384 runs. Its off cells at 4 % repeat the lost stage's 120 s cells on the merged
  code and must match them run for run.
- **brake20 (no errors, 288 runs):** brake 2 / 3 / 4 at lean 45 and tilt 20, D1 and D2,
  three geometries, 16 seeds, clock 2.0 s as in stage a. The brake 3 cells repeat stage c
  and must match. The brake rule registered above is applied to these rows, because at
  tilt 10 it compares camera losses.

**Predictions.**
12. With the flight preset the `stern` arm is within 2 kills of 32 per cell of the same
    arm under the 120 s clock, and never more than 1 below it.
13. At tilt 20 `stern` raises the 10 % straight cells by 10 or more of 32 and the 4 %
    cells by 5 or more; at 1 % it costs 2 or fewer. Weaving cells gain 4 or more at 10 %.
14. brake20: all three brakes give 16 of 16 at D1; at D2 none loses more than 1. Brake 2
    costs about 4 s and brake 3 about 1.5 s against brake 4. By the rule that would make
    brake 3 the choice (softest within 2.0 s).

**Null.** If `stern` does not beat off at tilt 20 by the finding threshold (scaled: net 4
of 32), the search does not carry to the flying lens.

## RESULT (2026-11-18, run as registered and amended; 9,400 runs)

Rows and logs: `runs/flight_hardware_rerun_20261118/` (one JSON per stage). Tables:
`scripts/flight_hardware_rerun_report.py` and `scripts/flight_hardware_rerun_ext.py --report`.
Fast sim on the Gazebo x500 fit with this board's limits laid over it; it ranks and
locates. Two identity checks across the merge of main passed: 64 of 64 and 96 of 96 rows
identical. No cell was short. The scorer's blind kills (inside 0.35 m with no decode
first) are counted separately and never exceed 5 in a cell of 50.

### Camera tilt is the largest lever with the flying lens

No random errors, brake 3, kills of 16 per cell; totals over 12 cells (three geometries,
two envelopes, D1 and D2):

| Tilt | 10 | 15 | 20 | 25 |
|---|---|---|---|---|
| Kills of 192 | 105 | 165 | 192 | 191 |

The grid (D1 only, kills of 48 over three geometries; share of frames with the tag out
of the picture during the chase):

| Lean limit | tilt 10 | 15 | 20 | 25 | 30 | 35 | 45 |
|---|---|---|---|---|---|---|---|
| 45 | 40, 28 % | 48, 15 % | 48, 1.6 % | 48, 4 % | 34, 19 % | 28, 36 % | 22, 48 % |
| 52 | 38, 31 % | 48, 14 % | 48, 2.3 % | 47, 5 % | 32, 20 % | 24, 36 % | 19, 49 % |
| 60 | 38, 32 % | 48, 14 % | 48, 1.7 % | 47, 5 % | 38, 18 % | 27, 36 % | 17, 49 % |

- At 10 degrees the tag leaves by the TOP of the picture while the vehicle leans into
  the catch-up (nose 28 to 32 degrees down in those frames), for about 1.1 s at a time,
  and the 2.0 s clock ends the engagement. At 20 degrees it stays in. From 30 degrees
  the picture looks over the target and kills fall.
- On arrival (target inside 48 m, not yet decoded, vehicle nose-up from braking) the
  tag is out of the picture in 58 % of frames at tilt 10, 79 to 85 % at 20 and 95 to
  98 % at 25 and above. So 20 is the top of the usable range, not the middle of it.
- **The lean limit does not change kills.** 45, 52 and 60 are within 1 kill at every
  tilt up to 25, and 0.3 s apart in time, because the board's 5 m/s² acceleration limit
  sets the climb to speed. Peak speed under expected drag: 26.5 m/s at 45 degrees, 30.0
  at 60.
- All errors on (speed error 4 %, 120 s clock, 32 seeds, kills of 128): lean 45 tilt 20:
  58. Lean 60 tilt 20: 56. Lean 52 tilt 20: 55. Lean 45 tilt 10: 42. By the registered
  rule the pair is **lean 45, tilt 20**; the three lean limits are not distinguishable.
- Tilt 20 against 10 with all errors on, 2.0 s clock, kills of 50: 24 to 28 and 24 to 33
  at 4 %; 42 to 45 and 33 to 40 at 1 %.

### Kill rates by cue speed error (all errors on, straight target, kills of 50)

Envelope now, D1, brake 3, tilt 10, as registered (near / demo):

| Speed error | 10 % | 4 % | 2 % | 1 % | 0 % |
|---|---|---|---|---|---|
| 2.0 s clock | 16 / 16 | 24 / 24 | 38 / 27 | 42 / 33 | 44 / 40 |
| 6.0 s clock | 17 / 19 | 24 / 29 | 38 / 34 | 42 / 38 | 45 / 44 |
| Never decoded (of 100) | 61 | 40 | 28 | 13 | 6 |

- The flying optics do not rescue the long dash: at 10 % and 4 % the kills are what
  October's optics gave (13 to 17 and 27 to 31), and the never-decoded runs again have
  the target behind the vehicle (57 of 61 at 10 %).
- **A weaving target is hardly ever killed, whatever the cue:** 2 to 11 of 50 in every
  cell, including 6 and 11 at zero speed error. The misses are close passes (17 to 27
  per cell) and lost targets (13 to 18), not failures to find it. This is a terminal
  problem, separate from the cue.

### Arrival search carries over to the flying lens

Tilt 20, lean 45, all errors on, 32 seeds, kills of 32 (near / demo):

| Speed error | Search off | `stern` | `believed` | `stern` + flight preset |
|---|---|---|---|---|
| 10 % | 10 / 11 | 30 / 29 | 31 / 31 | 28 / 28 |
| 4 % | 18 / 21 | 32 / 32 | 32 / 32 | 32 / 32 |
| 1 % | 30 / 27 | 32 / 32 | 32 / 32 | 31 / 31 |
| Straight total of 192 | 117 | 187 | 190 | 182 |
| Weaving total of 192 | 43 | 57 | 70 | 54 |

- At tilt 10 the same search gives 101 of 128 where tilt 20 gives 123 (10 % and 4 %
  straight cells), so the two changes add.
- Lean 60 against 45: 184 against 187 with the search, 116 against 117 without.
- Kills with the search come about 40 s after GO: of 428 `stern` kills at tilt 20, 424
  are later than 30 s, 86 later than 45 s and 46 later than 60 s. The old 30 s engage
  window would have cut nearly all of them.
- The flight preset's switches stand in well for "no clock": 182 against 187 straight,
  54 against 57 weaving; 10 seeds lost and 2 gained over 12 cells, at most 2 in a cell.
  Its non-kills end as close misses (129 of 148), i.e. the kept inside-3 m rule.

### Lost-target rule (no search, tilt 10, first 32 seeds, kills of 256)

| 2.0 s | 6.0 s | 20 s | 120 s | 120 s, close-range rule off |
|---|---|---|---|---|
| 88 | 98 | 106 | 107 | 107 |

No seed that was a kill at 2.0 s is lost at 120 s. Letting the terminal re-chase after
losing a target inside 3 m changes no total and moves single cells by 2 or fewer, with
kills as late as 116 s. So the longest clock gives the most kills, nearly all of the
gain is in by 20 s, and the close-miss rule costs nothing measurable without the search
and about 5 of 192 with it.

### Brake

At tilt 10 the registered rule picks nothing: the differences are camera losses (demo
D1: 7 to 8 of 16 at brake 2 and 3, 13 to 14 at brake 4). At tilt 20 (amendment 2) every
brake gives 16 of 16 in all six cells; time to closest approach is 0.8 s later at brake 3
(1.2 s at most) and 3.1 s later at brake 2 (3.6 s at most) than at brake 4; lean at first
decode is 19 to 25 degrees at brake 2, 21 to 32 at brake 3, 23 to 35 at brake 4. **By the
rule: brake 3.** October's "do not fly 36 m/s with brake 3" does not survive in any form:
brake 3 intercepts in every geometry here once the camera can see.

### Brackets

- fx 933 instead of 1104: never worse, and 16 and 14 of 16 where fx 1104 gives 8 and 7
  (demo, D1, tilt 10). The difference is vertical field of view, so the true lens (about
  42 degrees tall against this sim's 39.8) should sit between the two.
- **Side-facing tag, the placard as `docs/placard_mount.md` draws it: 0 of 16 in the
  near and demo geometries** (never decoded at all in near), 9 and 15 of 16 in the tail
  geometry, where the vehicle arrives from the side. A chase from astern needs the tag
  to face rear.

### Predictions, graded

- **Failed:** 1 (D1 already loses kills at tilt 10: demo 7 to 8 of 16). 4 (tilt 20, not
  15, and 25 loses only 1). 5's side-tag bound in the tail geometry. 6 at 4 % (24, not
  28 to 38), at 2 % and 1 % in demo, and on the weaving target (30 or more fewer, not 10
  to 20). 7 (the 6 s clock gains up to 7 in a cell, and one cell loses 2). 8 (tilt 20
  moves the cells by 3 to 9). 9's "tilt 35 and 45 below half everywhere" (demo holds) and
  "lean 60 is 1 to 2 s faster" (0.3 s). 10's 0-to-2 bound (one cell gains 4). 11's
  "`believed` gains less than `stern`" (it gains as much or more). 12's "never more than
  1 below" (2 below in two cells).
- **Held:** 2 and 14 on the brake at tilt 20. 3 (envelopes agree; peak 26.5 and 30.0
  against 25 and 30). 5 on fx 933. 6 at 10 % and 0 %. 7 and 10 in sign (longer clock,
  more kills, summed). 9 on where kills peak and on lean. 11 and 13 on the size of the
  search's gain. 12 within 2 per cell.

### What I would set, and what is unsure

- **Camera plate: 20 degrees up.** Not above 25. This is the one setting here that
  changes kills by a lot, and it holds across drag, lean limit and error level.
- **Lean limit: leave 45.** Raising it buys 3.5 m/s of top speed and no kills in these
  runs. That conclusion depends on the 5 m/s² acceleration limit being applied to
  offboard setpoints, which is not known.
- **Brake 3.**
- **Lost-target rule: the flight preset as it stands** (no clock before a track, none
  during the search, 6 s beyond 3 m). A plain long clock is 5 of 192 better and keeps
  the aircraft chasing after close misses for up to two minutes.
- **Arrival search on.** `believed` did at least as well as `stern` here and is the
  milder change; I would not choose between them on 32 seeds.
- **Tag: rear-facing.** The beam-facing placard cannot be read from astern.
- **Unsure:** the decode curve is a literature guess with one Gazebo anchor; the pinhole
  camera is 2 degrees short vertically; the vehicle is the x500 fit; drag is assumed;
  the target neither leans nor wobbles here. The tilt optimum sits on a 40-degree-tall
  picture, so a real lens that is 2 degrees taller and a principal point 2.5 degrees
  off-centre could move it by a few degrees. One Gazebo rung at tilt 15, 20 and 25 with
  the flying lens model would settle that.
