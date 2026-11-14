# Interceptor tuning ladder: what is left, in order

Written 2026-11-09 after the three flights that removed the hover wobble (ADR-0134).
Starting point: `configs/px4_6cmini/airframe.params`, the settings on the board tonight.

**How to use it.** One rung per flight. Each rung states the change, the prediction, how to
decide, and what "no difference" would mean, all written before the flight (the project's
pre-registration rule). After a rung is adopted, change `airframe.params` and add a fresh
board dump in the same commit, or `tests/test_airframe_params.py` fails. Writing to the
board is done with `scripts/bench/px4_param_set.py` (it reads every value back).

**What judges a rung today.** The flight controller's card slot is dead, and no flight has
yet carried the Pi. Every verdict so far is by eye, ear and a finger on the motor bells
within about 30 s of landing. That is good enough for rungs 1 to 3 and weak for the filter
rungs. The upgrade is the Pi's flight recorder (set up and bench-tested 2026-11-13, not yet
flown): it saves PX4's own flight log, with hover thrust, altitude and setpoints, whenever
the Pi rides along. Its default log holds the gyro at 50 Hz, so for the filter rungs use
`scripts/bench/hover_recorder.py` (100 Hz) instead. See `docs/fc_logging_without_sd.md`.

**Where the numbers come from.** `[PX4]` = read from PX4 v1.16.0 source or its tuning guide
(links at the bottom; I fetched each one). `[judgment]` = my choice, not a PX4 number.

| Rung | Change | Flight needed |
|---|---|---|
| 1 | Fly the height change already on the board | yes |
| 2 | Thrust curve `THR_MDL_FAC` 0 → 0.3 | yes |
| 3 | Gyro filter 40 → 60 Hz | yes |
| 4 | Gyro filter 60 → 90 Hz, then D-term filter 30 → 50 Hz | two flights |
| 5 | Attitude gain `MC_ROLL_P` / `MC_PITCH_P` 6.5 → 8 | yes |
| 6 | Airmode on, then thrust floor down | yes |
| 7 | Yaw gains | only if yaw feels soft |
| B | Speed-controller range calibration (bench job, props off) | then one check flight |
| H | Housekeeping with no flight: autotune switch, battery calibration | no |

---

## Rung 1. Fly the height change (already written, not yet flown)

- **On the board:** `MPC_THR_HOVER` 0.25 (was 0.5), `MPC_THR_MIN` 0.08 (was 0.12).
  *Hover thrust* is the fraction of full thrust that just holds the aircraft up; the
  *thrust floor* is the least thrust the height controller may command.
- **Registered prediction (ADR-0134 addendum 4):** Position-mode height holds without
  throttle nudging; descents are smoother.
- **What the source check adds (addendum 5, written before the flight):** the improvement may
  be smaller than that. PX4 learns hover thrust in flight and only searches ± 0.2 around the
  setting. At 0.5 it could search 0.30 to 0.70, and the derived hover value is about 0.365
  (`docs/airframe_numbers.md` §5.1), inside that band. So the old setting was probably not
  what made the height soft. The new band, 0.05 to 0.45, is safer either way.
- **Watch three things:**
  1. *Lift-off stick position in Stabilized.* Derived: the aircraft gets light near 58 %
     stick. At or below mid-stick means hover needs 0.25 or less and the thrust estimate is
     low. This is the only by-eye reading of hover thrust available; after a few seconds
     in the air PX4 moves mid-stick to its own estimate.
  2. *Position-mode height with your thumb OFF the throttle correction.* The throttle stick
     only holds height inside the middle 10 % of its travel (`MPC_HOLD_DZ` 0.1). A throttle
     stick that does not spring to centre will sit just outside it and command a slow climb
     or sink that feels exactly like "soft height hold".
  3. *Wobble during descents.* The floor went down; less thrust means less steering authority.
- **Decide:** height holds hands-off → adopt. Wobble in descents that was not there before →
  put `MPC_THR_MIN` back to 0.12 `[PX4 default]`.
- **A null means:** not the thrust numbers. Next suspects, in order: the throttle stick's
  centre (item 2 above), then the height estimate (GPS and barometer; needs the recorder).
- **About the slow descents:** the cap is `MPC_Z_VEL_MAX_DN` = 1.5 m/s `[PX4 default,
  allowed 0.5 to 4]`, and the throttle stick has a soft centre (`MPC_Z_MAN_EXPO` 0.6). Slow is
  the stock behaviour, not a fault. Raising the cap is optional and comes after rung 6
  `[judgment]`: a small quad descending fast into its own downwash wobbles, and the cure for
  that is airmode.

## Rung 2. Thrust curve: `THR_MDL_FAC` 0 → 0.3

- **What it is:** motors give thrust roughly as the *square* of the signal, so the same
  correction is weak at low throttle and strong at high throttle. This setting bends PX4's
  output to even that out. 0.3 is what both PX4 small-racer airframes ship `[PX4: 4050, 4052]`.
- **Why now:** the remaining oscillation shows up "returning to hover from a climb", a
  throttle transient. With 0 the control loop is about 1.6 times stronger at 60 % signal than
  at hover; with 0.3 about 1.4 times (derived from the formula in `FunctionMotors.hpp`).
- **Side effect to expect:** hover thrust in PX4's units drops from about 0.365 to about
  0.30, and the stick feels slightly stronger low down. PX4's estimator follows on its own.
- **Prediction:** the settle-after-climb oscillation is reduced, not necessarily gone (the
  change in loop strength is modest). Hover is unchanged or slightly crisper.
- **Decide:** oscillation smaller and nothing worse → adopt. New wobble at low throttle
  (descents) → revert to 0.
- **A null means:** the oscillation is not a too-strong loop at high throttle. The other
  reading is the opposite end: after a climb the controller cuts thrust to the floor and the
  motors have little authority left. That points at rung 6 (airmode), and it would also be
  made slightly worse by rung 1's lower floor.

## Rung 3. Gyro filter: `IMU_GYRO_CUTOFF` 40 → 60 Hz

- **What it is:** a smoothing filter on the rotation sensor. Lower = smoother signal but
  slower reactions: about 8 ms of delay at 30 Hz, 3.8 ms at 60 Hz, 1.9 ms at 120 Hz
  `[PX4 filter guide]`. The racer airframe uses 90 `[PX4: 4050]`; the board has 40.
- **Ordering risk (this is the rung that can cook motors):** a higher cutoff lets more
  vibration noise through to the motors. PX4's guide raises it 10 Hz at a time *with logs*.
  There are no logs, so the steps here are larger but the check is strict `[judgment]`.
  Do not start this rung until the motors come down merely warm after rungs 1 and 2.
- **Prediction:** tighter, more "locked" feel; the faint oscillation shrinks because delay
  falls. Motors no hotter.
- **Decide:** after a 60 s hover, a motor bell you cannot keep a finger on, or a new harsh
  or gritty motor sound → revert to 40 and stop climbing this ladder until the recorder
  exists. Otherwise adopt.
- **A null means:** delay from this filter was not limiting; still worth keeping if motors
  are cool, because the later rungs assume it.

## Rung 4. Finish the filters: gyro 60 → 90 Hz, then D-term 30 → 50 Hz

- Two flights, one change each, same motor-temperature check as rung 3.
- `IMU_DGYRO_CUTOFF` filters only the D term (the part of the controller that reacts to how
  fast the rotation is changing, and the noisiest part). PX4's guide says to leave it at 30
  while raising the main filter, then raise it last `[PX4 filter guide]`. Target 50
  `[PX4: 4050]`.
- **Prediction:** small improvements only. **A null is acceptable here**; stop at whichever
  value keeps the motors cool.
- **Not on this ladder:** a notch filter aimed at the motor frequency (derived hover values:
  213 Hz shaft, 638 Hz blade pass, `airframe_numbers.md` §7). It should be placed from a
  recording, not from a derived number, and the tracking version needs DShot, which this
  wiring cannot do.

## Rung 5. Attitude gain: `MC_ROLL_P` and `MC_PITCH_P` 6.5 → 8

- **What it is:** how hard the aircraft chases a commanded tilt angle. 6.5 is stock; both
  racer airframes use 8 `[PX4: 4050, 4052]`.
- **Why after the filters:** a stronger outer loop on top of a delayed inner loop overshoots.
- **Prediction:** stick inputs and position corrections feel snappier, with no bounce when
  the stick is released.
- **Decide:** one clean stop after a sharp stick release → adopt. A visible bounce or
  overshoot → back to 6.5.
- **A null means:** fine; the guidance work later will care more than a hover does. The rate
  limits (`MC_ROLLRATE_MAX` / `MC_PITCHRATE_MAX` 220 °/s stock; racers use 1200 to 1600
  `[PX4]`) cap how fast it may rotate. Raise those only when the intercept flight plan asks
  for it `[judgment]`.

## Rung 6. Airmode: `MC_AIRMODE` 0 → 1, then the thrust floor

- **What it is:** with airmode on, PX4 may *raise* total thrust to keep steering authority
  when throttle is very low. Value 1 = roll and pitch only.
- **Why last:** PX4's own note says to keep it off during tuning, because it helps an
  unstable aircraft diverge `[PX4: MC_AIRMODE description]`. Use value 1, not 2: value 2
  (yaw as well) requires an arming switch `[PX4]`, which this build does not have mapped.
- **Risk to brief yourself on:** on the ground with motors running, airmode can add thrust
  to "correct" a tilt from uneven ground. Land, throttle fully down, and use the kill switch
  rather than waiting for auto-disarm `[judgment, standard practice]`.
- **Prediction:** descents and the moment after a climb are steadier.
- **Then, as its own flight:** `MPC_THR_MIN` 0.08 → 0.05 `[PX4: 4052; the source says the floor
  can go as low as 0 with airmode on]`, for brisker descents. Only then consider
  `MPC_Z_VEL_MAX_DN` above 1.5.
- **A null means:** low-throttle authority was not a problem at these descent rates. Keep
  airmode off (simpler on the ground) and leave the floor at 0.08.

## Rung 7. Yaw (only if it feels soft)

- Board: `MC_YAW_P` 2.8, `MC_YAWRATE_P` 0.2 (both stock). Racers: 4 and 0.2 to 0.25
  `[PX4: 4050, 4052]`.
- Nobody has reported a yaw problem. Yaw matters little for the intercept. **Do not change
  it without a symptom** `[judgment]`. If the nose wanders or is slow to stop: `MC_YAW_P` → 4
  first.

## Bench job B. Speed-controller range calibration (props OFF)

- **The gap:** the board sends 1100 to 1900 µs and no calibration of the speed controller
  to that range is on record. If the controller still listens for a wider range, the
  aircraft never reaches full thrust and "zero" is not zero. Hover does not care much;
  top-end thrust, and every thrust number in `airframe_numbers.md`, does.
- **When:** any time, but **before** thrust-rig measurements and before dash work. It
  changes how signal maps to thrust, so it re-opens rung 1 (hover thrust) and slightly
  shifts effective gains. If the aircraft is flying well mid-ladder, finish the rung in
  hand first `[judgment]`.
- **How:** PX4's procedure is in QGC under Power → ESC Calibration `[PX4 ESC calibration
  guide]`. **Not verified for this hardware:** whether the AM32 firmware on the KO60II
  accepts that calibration or wants its endpoints set in its own configurator. Check that
  before trusting the result, and confirm with a props-off motor test afterwards.
- **Check flight prediction:** hover feels the same; lift-off stick position (rung 1's
  reading) moves a little. A big change in hover feel means the old range was far off, which
  is itself the finding.

## Housekeeping H (no flight needed; each is your call)

- **`MC_AT_EN` = 1 on the board.** That arms PX4's automatic tuning feature. It was not
  started (`MC_AT_START` 0), but with `MC_AT_APPLY` 1 a started run would overwrite the gains
  that now work. Recommend 0 unless an autotune is planned `[judgment]`.
- **Battery sensing is on defaults** (`BAT1_V_DIV` 18.1, `BAT1_A_PER_V` 36.4, capacity not
  set). The low-battery failsafe acts on these. Calibrate against a multimeter before
  trusting the percentage.
- **`MC_BAT_SCALE_EN` = 0:** thrust is not compensated as the pack sags, so the aircraft
  feels softer late in a flight. Worth turning on only after battery sensing is calibrated.
- **`CBRK_IO_SAFETY` = 22027** (safety switch bypassed) stays as it is: your decision, noted.

---

## What I could not verify

- Hover thrust itself. 0.365 is derived from an estimated 1.5 kg per motor and an
  estimated 800 g; nothing has been weighed, thrust-tested or logged.
- Whether the oscillation after a climb is a high-throttle or a low-throttle effect. Rung 2
  and rung 6 are the two tests.
- The speed controller's real input range, and the AM32 calibration method.
- Motor temperature in numbers. A finger is the instrument.

## Sources (PX4 v1.16.0, fetched 2026-11-09)

- Racer airframes: [4050_generic_250](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/ROMFS/px4fmu_common/init.d/airframes/4050_generic_250),
  [4052_holybro_qav250](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/ROMFS/px4fmu_common/init.d/airframes/4052_holybro_qav250)
- Thrust curve formula: [FunctionMotors.hpp](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/lib/mixer_module/functions/FunctionMotors.hpp)
- Throttle stick mapping: [mc_att_control_main.cpp](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/mc_att_control/mc_att_control_main.cpp)
- Hover-thrust estimator and its ± range: [MulticopterHoverThrustEstimator.cpp](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/mc_hover_thrust_estimator/MulticopterHoverThrustEstimator.cpp)
- Thrust floor, descent cap: [multicopter_position_control_limits_params.c](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/mc_pos_control/multicopter_position_control_limits_params.c)
- Airmode note: [mixer_module/params.c](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/lib/mixer_module/params.c)
- Guides: [filter tuning](https://docs.px4.io/v1.16/en/config_mc/filter_tuning),
  [multicopter PID tuning](https://docs.px4.io/v1.16/en/config_mc/pid_tuning_guide_multicopter),
  [ESC calibration](https://docs.px4.io/v1.16/en/advanced_config/esc_calibration)
