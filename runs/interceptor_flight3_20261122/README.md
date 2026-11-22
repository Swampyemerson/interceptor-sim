# Interceptor, third flight of 2026-11-22 (after the trim, the raised GPS limit and a compass recalibration)

Hand flight in Position mode, 49 s airborne, 1 to 3 m high by the barometer. The builder:
"even less stable", "super sluggish". Log `fc_20261122T234132Z_flight.ulg` (recorder verdict
CLEAN, 426 s) and 58 s of video on the Pi (`sessions/video_20261122T234739Z`, 3,463 frames,
no gaps). Raw log not in git (GPS positions). Scripts: `flight3_read.py`,
`flight3_followup.py` (compares all three flights of the day).

## What happened

| Log time | Event |
|---|---|
| 365.6 s | Armed in Position mode at 10 satellites. The raised speed-accuracy limit is what let it arm (the unit was reporting 0.4 to 0.9 m/s). |
| 369 to 393 s | Position mode works for 24 s: velocity followed to 0.35 m/s RMS. |
| 393 s | Satellites fall to 6 to 7; the estimator stops using GPS (PDOP check; it failed 29 % of the airborne time). |
| 397.2 s | Position invalid: "blind land", Altitude mode for 5 s. |
| 401.1 s | GPS comes back. The estimator realigns its heading from GPS and inertial data and **declares the compass faulty**, although the compass agreed with GPS-derived heading to 7 degrees all flight. |
| 413 to 414 s | With no compass, the heading estimate **jumps 48 degrees** in one step (the aircraft did not turn: gyro near zero). The estimated position is by then 6 m from the GPS position. "Compass needs calibration - Land now!" |
| 417.6 s | Kill switch (pilot). |

## Why it felt worse than the earlier flights

1. **Height hold followed the GPS's altitude, and that wandered.** The board uses GPS as its
   height reference (`EKF2_HGT_REF` 1, the PX4 default). Over this flight the estimator's
   height went from 0 to -6.6 m while the barometer stayed between -0.8 and +3.1 m; GPS
   altitude drifted 8 m. Holding a height estimate that sinks means the real aircraft
   climbs, and the pilot spent most of the flight holding the throttle stick below centre.
   Flight 2 shows the same in its second half (estimator 5 m from the barometer).
2. **Position mode is slower to answer the sticks, by design.** Stick-to-tilt delay:
   0.14 to 0.16 s in Altitude mode (flights 1 and 2), **0.36 s in pitch and 0.60 s in roll**
   in Position mode on this flight. PX4 shapes the command with an acceleration limit of
   3 m/s2 and a jerk limit of 8 m/s3 (`MPC_ACC_HOR`, `MPC_JERK_MAX`, both defaults). Full
   stick for two seconds only got the speed command to 3.9 of a possible 10 m/s.
3. **The estimator lost its compass and then its heading**, as in the table.

The aircraft's own control was still sound: attitude error 2.6 and 2.3 degrees RMS under
much larger stick inputs than before, motors balanced, hover at 24 to 25 % command, 8.9 A.

## GPS across the day (in flight, camera recording)

| Flight | Satellites used (median, range) | Speed accuracy the unit reports | Position mode |
|---|---|---|---|
| 1, bare ribbon | 5 to 6 | about 1.0 m/s | lost 3.5 s after takeoff |
| 2, foil on ribbon | 11 (9 to 11) | 0.51 m/s (limit then 0.5) | in and out |
| 3, foil, limit 1.0 | 9 (6 to 11), under 8 for 20 % of the time | 0.73 m/s | held 24 s, then lost |

The foil took the GPS from unusable to marginal. Marginal is not enough: every failure
above starts with the estimator dropping GPS and picking it up again.

## Compass

- Compass 0 (in the GPS unit) was recalibrated before this flight. **The result is worse.**
  The estimator's field strength reads 0.39 G on the ground and 0.42 G in the air against
  0.513 G from its Earth-field model; in flight 2, on the old calibration, it read 0.49.
  Where it was calibrated (near metal, on something steel) is the likely cause; not known.
- Compass 0 picks up battery current at 0.0036 G per amp: 0.03 G in hover, 0.28 G at the
  77 A of a full-throttle punch, half the Earth's field. Compass 1 (in the Pixhawk, not
  used for navigation) sees six times more.

## Vibration (all three flights)

- Lateral (y) accelerometer vibration grew flight to flight: 0.86, 1.28, 1.79 m/s2 standard
  deviation; 80 to 90 % of it sits between 20 and 60 Hz with a peak at 22 to 29 Hz. Roll-rate
  content above 6 Hz grew with it (3.5, 5.7, 7.1 deg/s).
- The log samples the accelerometer at about 200 Hz and the gyro at 48, so this could be a
  real 25 Hz rocking (the pod on its mount is the first suspect) or motor-speed vibration
  folded down by the sampling. The log cannot tell which. No clipping; PX4's own vibration
  figures are 1.0 and 1.7.

## Trim, power

- Trim (applied before this flight): mean roll +0.97, pitch -1.30 degrees while manoeuvring
  in Position mode. Not a stationary hover, so it neither confirms nor refutes the 2 degree
  roll correction; the sign is at least not backwards (that would show as about -4).
- 24.18 V before, 8.9 A airborne, lowest 21.33 V, 116 mAh counted, 23.27 V resting after.
