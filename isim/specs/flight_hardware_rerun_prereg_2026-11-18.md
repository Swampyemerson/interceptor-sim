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
