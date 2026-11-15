# Target drone, sixth flight: first automatic course (2026-11-15)

Log 2 on the reformatted card, 60.7 MB, 322 s logged, 318 s airborne. Copy:
`C:\Temp\bench\target_flight6_20261115\log2.bin` (holds positions, stays out of the repo).
Decision record: ADR-0138 addendum 16. Readers: `scripts/bench/ap_flight_overview.py`,
`ap_flight_phases.py`, `ap_flight_series.py`, `ap_hover_window.py`. No `ERR` records in the log.

## Timeline (log seconds)

| Time | Mode | What happened |
|---|---|---|
| 350 to 403 | Loiter | Armed, took off, hovered 51 s |
| 403 to 561 | Auto | Course flown 11 times (first pass plus ten jumps), then held at the last waypoint |
| 561 to 605 | Loiter | Still hover, slow descent to 1.5 m |
| 605 to 611 | Stabilize | Full-throttle climb by the pilot (stick 1583 to 2002 in 0.75 s), then throttle cut |
| 611 to 635 | Loiter | Caught at 17 m, flown about |
| 635.5 to 636.2 | Auto | Switch overshot to Auto for 0.76 s; mission restarted at take-off |
| 636.2 to 637.6 | Stabilize | Second climb (stick about 1850), throttle cut |
| 637.6 to 655 | Loiter | Caught while still rising, peaked 24.7 m, steady 2.8 m/s descent, soft landing |
| 654.9 | | Disarmed on the ground with the arm switch (ch5 2000 to 999) |

## Automatic course

| Quantity | Value |
|---|---|
| Laps | 11, each 12.7 to 13.0 s |
| Horizontal error against the controller's moving target | 0.26 m rms, 0.37 m 95th percentile, 0.46 m worst |
| Height error | 0.06 m rms, 0.20 m worst |
| Ground speed | 0.95 m/s mean, 1.75 m/s peak (commanded peak 1.81) |
| Lean | 8.0 degrees roll, 5.9 pitch (95th percentile) |
| Satellites, estimator checks | 23 or more; worst ratio 0.33 of the limit (compass) |
| Vibration | 7.4 / 5.9 / 2.1 m/s2, no clipping |

The legs are about 3 m, so the aircraft never reached the 3 m/s board speed or the mission's
5 m/s items; the speed setting was not exercised. The error is against the controller's own
target, not against the drawn polygon: with a 2 m waypoint radius the path is rounded.

## Predictions from flight four (`runs/target_flight4_20261114/README.md`), unchecked until now

Still hover 566 to 596 s, sticks centred, 0.055 m/s.

| Prediction | Result | Verdict |
|---|---|---|
| Pitch rate under 2 deg/s; the 23 Hz peak shrinks in pitch | 1.43 deg/s rms; biggest peak 17.4 Hz at 0.13 deg/s (was 3.4 at 23.4 Hz) | held |
| Hover attitude within 1 degree of zero | roll -0.75, pitch +1.09 | roll held; pitch missed by 0.09 degrees. Nowhere near the wrong-sign signature (about -6 / -8), so the trim sign is right |
| Height soft at first, then within 0.1 m | 25 to 50 s after arming: 0.027 m rms, 0.084 worst | held |

Roll rate error in the same hover: 1.91 deg/s rms.

## The two full-throttle climbs (pilot's, in Stabilize)

| Quantity | First (605 s) | Second (636 s) |
|---|---|---|
| Peak climb rate | 18.0 m/s | 16.3 m/s |
| Peak height | 23.9 m | 24.7 m |
| Peak vertical acceleration (IMU, 25 Hz samples) | 42.2 m/s2 | 49.8 m/s2, about 5 g |
| Lowest battery voltage | 18.3 V (3.05 V per cell) | 18.8 V |
| Peak current as logged (sensor uncalibrated) | 86.5 A | 80.5 A |
| Highest motor output | 1950 us | 1883 us |

- Thrust is at least 5 times weight on a pack that was more than half used.
- Switching to Stabilize with the stick where Loiter hovers (1583) raised throttle from 0.078
  to 0.198 at once, before the stick moved: mid-stick maps to `MOT_THST_HOVER` 0.125 (the
  firmware minimum), and this airframe hovers at 0.077.
- With the throttle cut, motors sat at the floor (1100 us) and the aircraft rolled to 50
  degrees in about 0.75 s while falling, until throttle returned.
- The compass consistency ratio reached 4.51 (limit 1.0) during the first climb; 0.33 at most
  otherwise. Motor current disturbs the compass at high throttle.

## Battery and the current sensor

Start 24.29 V, end 22.42 V resting (3.74 V per cell). The board counted 1682 mAh from a
1500 mAh pack that still rested at 3.74 V per cell, so the current sensor over-reads, by
something like a factor of two. Not calibrated: needs the mAh the charger puts back.
