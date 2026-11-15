# Target drone bench session, 2026-11-14

Kakute H7, ArduCopter 4.7.0, on USB, props off, never flown. Write-up: ADR-0138.

| File | What |
|---|---|
| `accelcal_result.json` | Six-position accelerometer calibration run with `scripts/bench/ap_accel_cal.py` after QGroundControl's own page would not advance past the first position. Offsets 0.067 / -0.016 / 0.031 m/s/s, scale factors within 0.2% of 1, identical before and after the reboot. The board's "3D Accel calibration needed" complaint is gone. |

Setting dumps from the same session: `configs/target_kakute/dumps/20261114_before.tsv`, `_after.tsv`
(taken before the calibration).
| `gps_wire_poll.json` | Outdoors, clear sky, `scripts/bench/ap_gps_wire_check.py`. On the board's GPS port: nothing heard in 3 s of listening, a 30-byte nonsense test string came back byte for byte at both 230400 and 115200 baud, and the module answered none of four questions (the only replies were the questions themselves). The board's GPS transmit and receive lines are connected to each other and the module's output does not reach the board. |
| `gps_wire_poll_after_fix.json` | Same check after the builder removed a stray wire strand that was crossing the two GPS signal lines: no echo, 2,062 bytes heard in 3 s, the module identifies itself as a u-blox SAM-M10Q, 3D fix with 14 satellites. The raw bytes are removed from this copy because they contain a position. After a reboot the board's own driver reported a 3D fix with 15 to 16 satellites. |
| `crash_flight_timeline.txt` | The crash flight (board log 7 of 7), four rows a second from arming: throttle stick, the height controller's throttle output (`ThO`), desired and actual climb rate, the four motor outputs, vibration, attitude, battery. Made with `scripts/bench/ap_flight_summary.py`. The `.BIN` logs themselves stay off the repo because they contain positions (kept on the dev PC under `C:\Temp\bench\target_crash_20261114\`). |

## What the crash log shows (2026-11-14, read the same day)

Loiter mode, 3.8 s in the air, 23 m at the top. From lift-off to the motor cut:

| Quantity | Value | Meaning |
|---|---|---|
| Throttle stick | at minimum (987) from 1.3 s after lift-off | the pilot was asking for down |
| Height controller's throttle output | 0.00 throughout; wanted -2.5 m/s | it was asking for down too |
| Actual climb rate | rose to +7.6 m/s | |
| Roll / pitch rate error | 27 / 24 deg/s standard deviation, controller output flipping sign every sample at up to 0.38 | the roll and pitch rate loops were oscillating hard |
| Yaw rate error | 5 deg/s | yaw was calm |
| Motor mixer's average thrust (`ThrOut`) | 0.10, pinned at its cap `ThrAvMx` 0.10 | the mixer raised thrust to make room for the oscillation |
| Motor outputs | mean 1302 us; one motor at the 1150 floor in 30 of 30 samples | |
| Lift-off | at a mean output of roughly 1200 to 1250 us | hover thrust is about 0.06 in ArduPilot's units, not the 0.20 set |

The cap is half of `MOT_THST_HOVER` in Loiter and AltHold (`ATC_THR_MIX_MAX` 0.5). With the hover
guess at 0.20 the cap was 0.10, well above this aircraft's real hover thrust, so an oscillating
attitude controller held the aircraft in a climb the height controller could not cancel.

## Thrust-to-weight and speed estimates from the crash log (derived 2026-11-14)

Inputs are all from board log 7: accelerometer (25 Hz), motor RPM from ESC telemetry, climb
rate, and the fall after the motors were cut. Weight is not needed for thrust-to-weight: an
accelerometer reads (thrust minus drag) divided by mass directly.

| Step | Value | From |
|---|---|---|
| Thrust/weight just after lift-off, climbing under 2.5 m/s | 1.45 | accelerometer, 12 samples |
| Motor speed at that moment | 13,080 rpm mean, at 30% throttle command (1298 us) | ESC telemetry |
| Thrust/weight per unit of summed rpm squared | 0.213 per 1e8 | the two rows above |
| Hover | about 10,800 rpm per motor, about 1235 us, 0.06 in ArduPilot thrust units | thrust scales with rpm squared |
| Check: on the ground at 8,940 rpm | predicts 0.68 of weight; it had not lifted | same constant |
| Full-throttle rpm | NOT measured. 28,000 to 32,000 assumed (1900 KV on 6S is 45,600 unloaded; these props load it down to roughly two thirds) | assumption |
| **Thrust-to-weight at full throttle** | **6.7 to 8.7, about 8** | rpm squared scaling |
| Thrust lost in the climb | 31% at 7.5 m/s (13,500 rpm), so thrust reaches zero near 24 m/s at that rpm, 0.107 m per turn | accelerometer plus drag, against the static constant |
| Drag, falling with motors off | deceleration = 0.017 x speed squared (quartiles 0.015 to 0.018), terminal fall 24 m/s | accelerometer and speed, 44 samples above 8 m/s |

Speed, from those:

| Case | Speed | Limited by |
|---|---|---|
| Auto, as configured | 10 m/s | `WP_SPD` |
| Loiter, as configured | 12.5 m/s | `LOIT_SPEED_MS` |
| Held at the 35 degree lean limit | about 20 m/s (17 to 24) | drag: sqrt(g tan 35 / k) |
| Flat out, no lean limit | about 38 m/s (33 to 44) | thrust falling with speed against drag |

Weak points. The full-throttle rpm is an assumption, and thrust-to-weight moves with its square.
The drag figure is for a tumbling airframe with stopped props, used here for forward flight. The
flat-out figure needs about 80 degrees of lean, which no configured mode allows. The ESC
telemetry's current reading is not usable (it reads about 450 A) and the board's own current
sensor is uncalibrated, so no power figures are given.

## Impact (derived 2026-11-14, same log)

| Quantity | Value | From |
|---|---|---|
| Fall | from 23.0 m, 3.1 s | height and time, motor cut to first clipping |
| Speed just before the ground | 18.4 m/s | logged vertical speed at the last record before the hit; the drag fit above predicts 17.9 m/s for a 24 m fall |
| Rebound | about 3.2 m/s, 0.5 m | second burst of clipping 0.65 s after the first |
| What the accelerometer recorded | saturated: 38 raw readings above ArduPilot's 15.5 g clipping flag at the first hit, 20 more at the second | `VIBE.Clip`; the sensor's range is about 16 g, so the peak itself was not recorded |
| Mean deceleration, from stopping distance | 580 g at 3 cm, 345 g at 5 cm, 215 g at 8 cm, 175 g at 10 cm | v squared / (2 d); the distance the turf and battery gave was not measured |
| Mean deceleration, from clipping time | 58 to 116 g | 21.6 m/s change over 19 to 38 ms (38 readings at 2,000 or 1,000 per second; the raw rate is not confirmed). Implies 17 to 35 cm of travel, more than a lawn gives, so this is too low: the clipping count includes the frame ringing after the hit |

The logged 25 Hz accelerometer value reads a steady 2.7 g for 0.6 s after the hit, which is not
physical for an airframe lying on grass; it is the sensor recovering from overload and is not
used. Energy at impact is 103 J for the 610.5 g bare weight in `docs/airframe_numbers.md` (a
derived weight, not a weighed one).

## The climb fix (on the board 2026-11-14, NOT yet proven)

ADR-0138 addenda 7 and 8. The motor floor came down and the top stayed (builder's call: keep
the top-end power). Settings changed from the crash: `MOT_SPIN_MIN` 0.15 -> 0.10,
`MOT_THST_HOVER` 0.2 -> 0.125, roll/pitch rate P/I/D halved.

- Crash settings: the highest average thrust the board could hold against a bottom stick was
  1285 us computed (1302 us logged), about 1.4 times the weight (1.45 measured).
- Now: 1194 us computed, about 0.7 times the weight. Hover is near 1235 us.
- Why 0.10: 60 logged readings at the armed idle (1100 us), motors at 4,700 to 5,600 rpm,
  none stalled.
- Read-back 76 of 76, persisted across a restart.

**Check to run, props off, before any flight** (criterion written before running it):
`python.exe ap_thrust_ceiling_check.py COM4` from `C:\Temp\bench`. PASS = motor average
below 1235 us while the board fights a bottom stick; predicted 1190 to 1215. NOT EXERCISED is
not a pass. Result file goes here as `thrust_ceiling_result.json`.
