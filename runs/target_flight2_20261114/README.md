# Target drone, second flight session (2026-11-14 evening)

Six logs pulled from the Kakute over USB (logs 8 to 13). The raw `.BIN` files hold positions
and stay off the repo (`C:\Temp\bench\target_flight2_20261114\` on the dev PC). Files here hold
no positions. Read with `scripts/bench/ap_flight_summary.py` plus two one-off readers.

Only log 13 is a flight (39 s armed: Loiter, Stabilize, Loiter, Stabilize, Loiter). Logs 8 to
12 are arm attempts of a few seconds.

## What the log shows

- **The runaway climb did not recur.** Highest point 3.4 m, in Stabilize, on a stick push.
- **Roll and pitch are calm.** In the airborne Loiter segment the rate error is 3.4 and 2.9
  deg/s (it was 27 and 24 in the crash flight). Halved gains held.
- **The height loop oscillates in Loiter.** The controller's throttle demand reverses on every
  10 Hz log sample (-0.4 to +0.4), the throttle output alternates between 0.00 and about 0.18,
  and the motors pulse between 1100 and about 1400 us. Vertical acceleration swings 0.3 to
  1.9 g. Apparent rate 5.6 Hz at the 25 Hz IMU log rate, so the true rate is 5.6 Hz or an
  alias of something faster. `height_loop_analysis.txt`.
- **Much of that Loiter time was on or at the ground.** The height target sat about 0.5 m
  below the estimated height while the estimate did not move: the stick had been below the
  hold band (1272 against a band of 1400 to 1600), which commands descent, and the target ran
  on below the lawn.
- **Hover thrust measured: 0.08.** 96 near-level Stabilize readings of vertical acceleration
  against thrust; 1.04 g in the 0.08 bin; straight-line fit 0.079. `hover_thrust_fit.txt`.
  That is 1216 us, lower than the 1235 us estimated from the crash flight.
- **Ending.** Accelerometer clipping (22 readings) at 35.0 s just after a switch to Stabilize
  at low stick, then the height estimate read 2 m below the barometer. At 38.5 s, back in
  Loiter, three motors went to about 1630 us with one at 1100, roll reached 40 degrees, 17
  more clipped readings, and it came to rest at 17 degrees. Disarmed at 40.3 s.
- A failed switch to Auto is logged (no route loaded).

## Cause of the bounce

`PSC_D_ACC_P` 0.05 and `PSC_D_ACC_I` 0.1 were firmware defaults, sized for a drone that hovers
at half throttle. ArduPilot's first-flight page gives P = 0.1 x hover thrust and I = 0.2 x.
For hover 0.08 that is 0.008 and 0.016: the board was running about 6 times that. This step
was missed in the 2026-11-14 morning setup (ADR-0138 addendum 9).

## Changed on the board afterwards

`PSC_D_ACC_P` 0.008, `PSC_D_ACC_I` 0.016; `FLTMODE3` and `FLTMODE5` to Stabilize to match the
`FLTMODE4` the builder had set. Read back 78 of 78. Not yet flown.

## What this does to the thrust-ceiling margin

With hover at 0.08 the ceiling ArduPilot can hold against a bottom stick (0.0625 thrust,
1194 us) is closer to hover than computed: this flight's readings give 0.86 g at 0.06 thrust,
so about 0.9 times the weight, not 0.7. Still a descent, with a 22 us margin.
