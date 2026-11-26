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

Read back with a fresh `param show` after the write, and again after a restart.
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

## Outdoors, same evening (`cam_test_outdoor*.out`, `gps_camera_outdoor*.out`, `cam_compass_outdoor*.out`)

Aircraft restarted and set down outdoors, disarmed, props on, read-only. The spot is
close to the house so the Pi could reach the Wi-Fi; it is not the exact spot of
2026-11-22. The three settings read back unchanged after the restart. The camera test
was run twice in one power-up (`outdoor_camtest.ulg`, `outdoor_camtest2.ulg`, not in git).

| Condition | Satellites used (median, range) | Fix | Speed accuracy |
|---|---|---|---|
| Run 1, camera off, before | 18 (16 to 19) | 3D | 0.15 m/s |
| Run 1, camera recording 1920x1200 | 15 (13 to 16) | 3D throughout | 0.16 m/s |
| Run 1, camera off, after | 16 (13 to 17) | 3D | 0.18 m/s |
| Run 2, camera off, before | 16 (14 to 17) | 3D | 0.16 m/s |
| Run 2, camera recording | 14 (13 to 16) | 3D throughout | 0.18 m/s |
| Run 2, camera off, after | 17 (15 to 18) | 3D | 0.13 m/s |
| For comparison, 2026-11-22 after foil: off / recording / off | 7 / 7 (5 to 7) / 7 | 3D | 0.57 / 0.85 / 0.67 m/s |

- **Reception roughly doubled and the speed accuracy is four to five times better**, with
  the camera on or off. What changed between the two days: the wiring tidy (ribbon fold
  moved from under the lid, 5 V runs moved lower, leads tightened), the spot, and the
  time of day. The test cannot say how much each contributed.
- **The camera still costs 2 to 3 satellites** (3 in run 1, 2 in run 2) and nothing
  measurable in speed accuracy.
- **Against the pass mark fixed in `docs/gps_reception_plan.md`:** speed accuracy under
  0.35 m/s with the camera on: met. Camera start costs no more than one satellite: not
  met. Within 2 satellites of a reference with the unit away from the aircraft: not
  measured.
- **The estimator still dropped the position twice in five minutes,** for 4 s and 7 s
  (3 % of the log), both times on a ground-only check with the camera off when the
  warning began: the GPS position drifting over 0.1 m/s sideways (69 to 88 s), then over
  0.2 m/s vertically (278 to 287 s), plus one "PDOP too high" blip (2.7 against a 2.5
  limit). GPS altitude wandered 7 m in 85 s while the aircraft sat still. Preflight
  checks passed at the end of the log. These checks only run on the ground; they would
  refuse arming for those seconds. Standing close to the house may be part of it
  (signals reflecting off the wall); not tested.
- **Compass:** the camera recording moves the GPS unit's compass by 0.007 and 0.009 G
  (0.006 indoors), back to within 0.001 G when it stops. Calibrated field strength reads
  0.461 G against 0.513 expected; the wiring has moved, so a recalibration is owed.
- The GPS "jamming figure" sat at 26 to 31 throughout and no longer moves with the
  camera. On 2026-11-22 it read about 100 with the camera off. It is not used as a
  measure here.

## The Pi did not start on one power-up

After the aircraft was carried outside and the battery plugged back in, the Pi was
unreachable for several minutes. The builder found its light solid red (powered, not
running), unplugged and replugged, and it came up green. Read afterwards on the running
Pi: supply 5.30 V, no under-voltage flag, and no record of the failed start, because the
Pi's system log does not survive a restart. Cause not established. Candidates: a quick
unplug and replug, or a 5 V connection disturbed by the same day's wiring tidy. With the
Pi not running there is no flight log and no video, and nothing stops the aircraft
arming, so the green light and the recorder check come before every flight.
