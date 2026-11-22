# Interceptor, second flight of 2026-11-22 (foil on the camera ribbon)

Hand flight, 131 s airborne inside a box of about 8 m by 6 m, up to 3.1 m high, with two
full-throttle punches and some back-and-forth. Log `fc_20261122T230116Z_flight.ulg`
(recorder verdict CLEAN) and 145 s of video on the Pi (`sessions/video_20261122T230121Z`,
8,634 frames, 1 gap). The raw log is not in git (GPS positions); copies are in this folder
on the laptop and in `~/fc_logs` on the Pi. Scripts here reproduce the numbers:
`flight2_read.py`, `heading_and_trim.py`, plus the three in `runs/interceptor_flight_20261122/`.

## GPS: the receiver was fine this time, PX4 still dropped it

- In flight, camera recording, foil on: **11 satellites (9 to 11), 3D fix throughout, HDOP
  1.0**. The first flight had 5 to 6. The ground test earlier showed 7. So the foil works,
  and reception in the air was better than on the ground.
- The position estimate still went invalid at 29 s, 36 s and from 50 s to 121 s ("invalid
  setpoints", "Failsafe: blind land", fall back to Altitude mode).
- Cause, as far as the log shows: the GPS reports its own speed accuracy at a median of
  0.51 m/s (0.22 to 0.74) and PX4's gate is 0.5 (`EKF2_REQ_SACC`). The estimator's GPS
  checks first passed at 38 s and its dead-reckoning flag flickers on and off through the
  flight. While it was using GPS, the GPS agreed well with the inertial sensors (velocity
  test ratio median 0.14, never above 0.82, where 1.0 is the rejection line).
- Heading was right: tilt-implied acceleration matches GPS acceleration with a correlation
  of 0.95 once the estimated heading is turned 7 degrees.
- **Changed after the flight, on the builder's yes: `EKF2_REQ_SACC` 0.5 to 1.0 m/s**, read
  back. It would very likely have kept position through this flight. It loosens a guard,
  so it goes back to 0.5 once the GPS is moved or shielded properly. Not yet flown.

## Compasses

- Compass 1 (inside the Pixhawk) follows battery current: 0.024 G per amp, 1.8 G at 77 A,
  against an Earth field of about 0.5 G. It is switched off for navigation already
  (`CAL_MAG1_PRIO` 0), which is right.
- Compass 0 (in the GPS unit) ignores battery current (0.0002 G per amp) but reads 0.31 to
  0.51 G through the flight and 0.36 G on the ground, and PX4 declared it faulty at 124 s.
  It wants recalibrating now that the foil and the layout have changed.

## Full throttle (two punches in Stabilized mode, 109 to 111 s)

| | Punch 1 | Punch 2 |
|---|---|---|
| Motor command, mean of four | 0.85 | 0.97 (three at 1.00) |
| Peak upward force | 3.64 g | 4.23 g |
| Thrust, total / per motor (930 g aircraft) | 3.38 kg / 0.85 kg | 3.93 kg / 0.98 kg |
| Battery current, peak | 62 A | 77 A |
| Battery voltage, lowest | 18.1 V | 18.0 V (3.00 V per cell) |

- **Measured thrust-to-weight at full throttle: 4.2 to 1**, on a pack that sagged to 3.0 V
  per cell. The plan assumed 1.5 kg per motor (6.5 to 1) and the rule is 5 to 1.
- Sag: voltage against current gives 61 milliohms over the punches (55 early in the
  flight). The pack went from 24.5 V resting to 18.0 V at 77 A. Thrust goes roughly as
  voltage squared, so the same motors on a pack that held 22 V would make about 1.45 kg
  each; the pack is what limits thrust. Pack temperature and age not recorded.
- 77 A is above the power module's 60 A continuous rating and inside its 120 A short-burst
  rating. Motor log rate is 10 Hz and the force is smoothed over 0.1 s, so true peaks may
  be a little higher.
- Hover: 24 to 25 % motor command, 8.9 A mean over the flight.

## Battery gauge

- 24.49 V resting before (4.08 V per cell). The sensor counted 324 mAh. Resting some
  minutes after: 22.94 V (3.82 V per cell), gauge 49 %.
- `BAT1_CAPACITY` had been set to 1350 before this flight. That switched on PX4's
  remaining-flight-time check, which fired twice (88.7 s, 92.9 s: "Failsafe activated",
  Descend for a tenth of a second each time) because with a broken position estimate the
  time it thinks it needs to get home is nonsense. **Reverted to -1 after the flight** and
  read back. It can go back on once the position estimate is dependable.
- "Low battery" came up at 98.8 s. The gauge read 0.22 to 0.25 under load at that point and
  0.49 at rest afterwards: the board's voltage-to-percent line is pessimistic under load.
- What settles the pack's real capacity: the mAh the charger puts back after a flight,
  compared with the mAh the sensor counted. Not yet recorded.

## Level trim (applied)

- Over 129 s the aircraft stayed within a few metres (net GPS displacement 4.2 m, mean GPS
  velocity 0.03 and -0.02 m/s) while holding a mean roll of **-2.07 degrees** and a mean
  pitch of -0.38 degrees; the roll stick averaged 6 % left. Heading 23 to 51 degrees.
- With no steady wind that is a level-reference error of 2 degrees in roll. A steady
  crosswind of about 6 m/s would produce the same lean (assumed drag area), and one flight
  at one heading cannot tell the two apart. The first flight is no help: it was drifting,
  not holding position.
- **Applied 2026-11-22: `SENS_BOARD_X_OFF` 1.459 to -0.61 degrees** (old value plus the
  mean roll; sign checked against PX4 v1.16 `level_calibration.cpp`), read back. Pitch left
  alone (`SENS_BOARD_Y_OFF` -0.26): 0.4 degrees is inside what wind could do. To undo:
  set `SENS_BOARD_X_OFF` back to 1.459. Power-cycle before flying.

## Lean angle, speed and the camera angle

- Largest lean: 34 degrees (the stick limit is 35). Fastest GPS ground speed: 3.4 m/s.
- **Lean angle against speed cannot be read from this flight**: it never held a steady
  speed above 2 m/s. That needs straight runs of 5 s or more at a few steady speeds.
- Camera tilt as mounted, from the video frame at log time 90 s (kept on the laptop as
  `frame_level_attitude_log_t90s.jpg`, not in git: it shows the surroundings; aircraft level to
  within 0.1 degree at that moment): the far horizon sits about 180 pixels below the
  centre of the 960 by 600 frame; with the 3.6 mm lens (600 pixels focal length at that
  size) that is **about 17 degrees up**, give or take 2 (horizon picked by eye, lens not
  calibrated). The ruled angle is 10 and the 2026-11-18 sim rerun preferred 20.
