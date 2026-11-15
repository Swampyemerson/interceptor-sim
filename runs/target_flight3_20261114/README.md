# Target drone, third flight (2026-11-14 night)

Log 15, 114 s armed: Loiter, 5 s of Stabilize, Loiter. Raw `.BIN` files stay off the repo
(`C:\Temp\bench\target_flight3_20261114\`); files here hold no positions.
Settings flown: ADR-0138 addendum 9 (height gains 0.008 / 0.016, roll and pitch gains halved).

Builder's report: "much better, still not perfect"; drifts up and down; holds height better
moving side to side than left alone; oscillation still audible. He held the airframe from
below and tugged it (100 to 104 s and 108 to 113.5 s); those stretches are kept out of the
hover figures.

## Height

- The fast motor pulsing of flight 2 is gone: throttle output sd 0.001 in the still hovers.
- **First still hover (6 to 25 s):** the estimator reported a descent of 0.34 to 0.46 m/s
  while the height hardly changed. Barometer and GPS both disagreed with it in the same
  direction (height innovation +0.5 to +1.3 m, vertical speed innovation +0.4 to +0.5 m/s).
  The height controller follows the estimator, so it sat 0.4 to 0.5 m off its target.
- The estimator's vertical accelerometer bias climbed from +0.01 to +0.11 m/s2 over the
  first 45 s. As it did, the error shrank: 0.15 m at 30 to 40 s, 0.04 m at 44 to 55 s.
- **Last still hover (93 to 100 s):** 0.13 m off target, steady to 0.02 m.
- At rest before take-off the accelerometer reads 9.831 m/s2, so the bias appears only with
  the motors running. Vibration is low (2 to 2.5 m/s2).
- So "better when moving" and "better later in the flight" coincide in this log and cannot
  be told apart from it. The barometer and GPS agreeing against the estimator points at the
  accelerometer path, not at prop wash on the barometer (my first guess).

## Roll and pitch

- Still hover: roll rate error 6 to 17 deg/s, pitch 1.4 to 3.8, on equal gains. Successive
  10 Hz readings of roll error alternate (lag-1 correlation -0.24 to -0.44).
- Under the hand tug roll rang at 55 deg/s with lag-1 correlation -0.92; pitch 19 deg/s, -0.70.
  Throttle was up to 0.36 there, where the motors' gain is higher.
- At 10 Hz the oscillation's real frequency cannot be read.
- Hover attitude with centred sticks and near-zero speed: roll -3.0, pitch -4.4 degrees in
  every still segment. `AHRS_TRIM_X/Y` are +3.7 and +3.6 degrees. Either wind or a level
  reference that is off by about that much. Not changed; question put to the builder.

## Changed afterwards (not yet flown)

Roll rate P/I/D x0.7 (0.047 / 0.047 / 0.00126); pitch unchanged. `LOG_BITMASK` 180223: rates
and PID terms at 400 Hz. Read back 78 of 78.

## Predictions for the next flight (written before it)

1. Roll rate error in a still hover, sampled at 10 Hz as here, below 5 deg/s. If it is not,
   the 400 Hz log gives the frequency and the term to cut.
2. Height: in a still hover of 60 s or more with the stick centred, error under 0.2 m after
   the first 40 s. If it still wanders by 0.5 m after 60 s, the bias-learning account is
   wrong and the barometer in prop wash is the next suspect.
