# Target drone, fourth flight (2026-11-14 night)

Log 19, 102 s armed, Loiter throughout, ended by the low-battery failsafe. First flight with
400 Hz logging. Raw `.BIN` stays off the repo (`C:\Temp\bench\target_flight4_20261114\`);
files here hold no positions. Settings flown: ADR-0138 addendum 10.

Builder's report: drifted enough to need a correction for a bit; got more stable as it flew;
near the end he bumped a stick and it then landed itself.

## The two predictions written before this flight

1. *Roll rate error in a still hover, sampled at 10 Hz, below 5 deg/s.* **Held:** 1.0 to 1.9.
2. *Height error under 0.2 m after the first 40 s of a still hover.* **Held:** +0.35 to +0.45 m
   for the first 20 s, under 0.2 m from 26 s, and within 0.09 m from 34 s to 82 s. The
   estimator's vertical accelerometer bias climbed from 0 to +0.08 m/s2 over the same time, as
   in flight 3.

## Why it landed

`Battery 1 is low 21.57V` then `Battery Failsafe` at 82.2 s; mode changed to RTL (reason 4,
battery failsafe) and it landed at home, disarming at 101.8 s. `BATT_LOW_VOLT` is 21.6 V
(3.6 V per cell) held for 10 s. The pack began this flight at 22.5 V resting, after three
earlier flights on the same charge. Working as set.

The current sensor reads about 25 A in a hover, which is not believable for this aircraft, so
the "used 565 mAh" figure in that message is not either. The voltage is what triggered.

## The oscillation, now with a frequency

One peak at 23.4 Hz in both roll and pitch. In a still hover (10 to 40 s):

| | roll (gains 0.047) | pitch (gains 0.0675) |
|---|---|---|
| gyro rate, all frequencies | 1.56 deg/s | 3.56 deg/s |
| of which 18 to 30 Hz | 1.26 | 3.41 |
| of which 150 to 200 Hz (motor speed) | 0.71 | 0.87 |
| D term at 18 to 30 Hz | 0.0030 | 0.0121 |
| P term at 18 to 30 Hz | 0.0010 | 0.0040 |

The D term carries about three times what the P term does at that frequency. Cutting roll
30% took its 23 Hz motion from the larger of the two axes to the smaller. In roll, motor-speed
noise near 180 Hz is now as large in the output as the 23 Hz oscillation.

## The drift

The controller held the drone within 0.03 to 0.23 m of its own position estimate throughout,
and the estimate stayed within 0.27 m of GPS (GPS's own stated accuracy 0.31 to 0.38 m). So
whatever moved, moved with the estimate. For the first 20 s the compass heading disagreed
with the GPS-derived heading by 8 to 10 degrees, shrinking to under 2 degrees by 70 s; the
board logged an in-flight yaw realignment at 85 s. The compass was calibrated beside a laptop
and has not been redone.

## Level reference

36 s of still hover with centred sticks and under 0.05 m/s: roll -3.15 (sd 0.18), pitch -4.05
(sd 0.32) degrees. The same figures as flight 3. ArduPilot's Save Trim adds the attitude held
in a steady hover to the trim (`RC_Channels_Copter::save_trim`, `AP_AHRS::add_trim`), so:
`AHRS_TRIM_X` 0.0640 -> 0.0090 and `AHRS_TRIM_Y` 0.0620 -> -0.0086 rad. Both come out near
zero, i.e. the board sits flat in the frame and the calibration's level step was taken with
the drone tilted about 3.6 degrees each way.

## Changed afterwards (not yet flown)

Pitch rate P/I/D to match roll (0.047 / 0.047 / 0.00126). The two trims above. Fast logging
left on. Read back 78 of 78; trims read back twice.

## Predictions for the next flight

1. Pitch gyro rate in a still hover falls from 3.6 to under 2 deg/s; the 23 Hz peak shrinks
   in pitch as it did in roll.
2. Logged hover attitude with centred sticks within 1 degree of zero in roll and pitch. If it
   reads about twice the old values instead, the trim sign is wrong: restore 0.0640 / 0.0620.
3. Height again soft for the first 20 to 40 s after power-up, then within 0.1 m.
