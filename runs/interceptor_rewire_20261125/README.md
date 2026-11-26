# Interceptor after the wiring tidy: bench checks (2026-11-25)

The builder tidied the wiring (battery leads kept together, 5 V runs moved lower, the
camera ribbon's fold moved from under the lid) and powered the aircraft up indoors,
props on. Everything here is read-only on the flight controller apart from three
settings the builder approved on 2026-11-22. Nothing armed or drove a motor.

## Settings written and read back

| Setting | Before | Now |
|---|---|---|
| `EKF2_HGT_REF` (height reference) | 1 (GPS) | 0 (barometer) |
| `MPC_ACC_HOR` | 3.0 | 5.0 |
| `MPC_JERK_MAX` | 8.0 | 20.0 |

Read back with a fresh `param show` after the write. Not yet re-read after a restart.
Unchanged: `EKF2_REQ_SACC` 1.0, `SENS_BOARD_X_OFF` -0.61, `BAT1_CAPACITY` -1,
`GPS_SAT_INFO` 0.

## Camera on and off, indoors (`cam_test.sh`, `cam_compass.py`, `cam_compass_indoor.out`)

25 s camera off, 30 s recording, 25 s off. The log is the recorder's own, copied from
the Pi (`indoor_camtest.ulg`, not in git).

| | Camera off, before | Recording | Camera off, after |
|---|---|---|---|
| Compass in the GPS unit, shift from "before" | 0 | 0.0060 G | 0.0024 G |
| Compass in the Pixhawk, shift | 0 | 0.0078 G | 0.0024 G |
| GPS satellites | 0 | 0 | 0 |
| GPS jamming figure (median) | 25 | 25 | 18 |

- **The Pi and camera working move the GPS unit's compass by 0.006 G,** about 1 % of
  Earth's field, mostly in the vertical direction. Each reading is noisy by about 0.004 G
  and each window averages 21 to 27 of them, so the shift is real but small. There is no
  measurement from before the tidy to compare it with.
- **The GPS part is not a result.** Indoors there are no satellites, and the jamming
  figure without sky is not comparable with the outdoor figures of 2026-11-22. The
  comparison needs the aircraft outdoors.
- The AI board was idle throughout; only the camera was exercised.
