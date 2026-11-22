# Interceptor, first flight with the pod and the Pi recorder, 2026-11-22

Hand flight outdoors, 81 s airborne, never above 1.6 m. The builder: "very short not stable
flight that ended itself." Log `fc_20261122T223057Z_flight.ulg` (saved by the Pi's flight
recorder, verdict CLEAN, 263 s, 6 packets lost at start-up) and 88 s of video on the Pi
(`sessions/video_20261122T223350Z`, 5,236 frames at 59.5 per second, 4 gaps over 1.5 frames,
worst 50 ms; 672 MB, left on the Pi). This was the recorder's first real arming: the video
started 1.6 s after arming and stopped after disarm.

## What happened (`flight_overview.py`, `flight_stability.py`)

| Log time | Event |
|---|---|
| 172.2 s | Armed by stick in Position mode, 8 satellites. The recorder starts the camera about 1.6 s later. |
| 175.8 s | Takeoff. |
| 176 to 180 s | GPS drops from 8 satellites to 6, then 5; its speed accuracy goes from 0.11 to about 1 m/s. |
| 179.3 s | PX4: "invalid setpoints", "Failsafe: blind land". Position is declared invalid and the aircraft falls back to Altitude mode (height hold only, no position hold). |
| 179 to 247 s | Flown by stick in Altitude mode. With no GPS the estimator's position ran away: it believed 12 m/s and 470 m from the start. The GPS's own track shows at most 33 m and 4.6 m/s. |
| 247.3 s | The runaway position crosses the 457 m geofence: "Geofence: exceeding maximum distance to Home". PX4 switches to Descend. This is the flight "ending itself". |
| 252.0 s | Kill switch (pilot). Disarmed 5 s later. |

## The aircraft itself flew well (`flight_window.py`, 182 to 246 s)

- Attitude followed its commands: error 1.4 degrees RMS in roll, 1.6 in pitch.
- Rates followed their setpoints with a correlation of 0.92 (roll) and 0.97 (pitch), 21 ms behind.
  The log holds rates at 48 Hz, so anything faster than about 24 Hz cannot be seen in it.
- Motors balanced (commands 0.24 to 0.26, never at a limit). Hover needs 24 to 25 % command,
  matching the 0.25 on the board.
- Height wandered (0.77 m standard deviation) and the aircraft drifted, which is what Altitude
  mode without position hold does in wind.

## Cause: the camera link jams the GPS (`gps_camera_test.py`, `gps_windows.py`)

Ground tests straight after the flight, aircraft disarmed, GPS figures read from the log:

| Condition | Satellites used (median, range) | Fix | Speed accuracy |
|---|---|---|---|
| Camera off | 6 to 7 | 3D | 0.2 to 0.7 m/s |
| Camera recording 1920x1200 (the arming video) | 0 (0 to 6) | lost | 3.1 m/s |
| Camera off again | 6 (5 to 7), back within 3 s | 3D | 0.65 m/s |
| Processor fully loaded, camera off | 6 (5 to 6) | 3D | 0.69 m/s |
| Camera streaming 1920x1200, no encoding | 4 (0 to 5) | lost at times | 1.65 m/s |
| Camera streaming 1280x720, no encoding | 0 (0 to 6) | lost | 3.5 m/s |

The GPS's jamming figure drops from about 100 to about 23 within a second of the camera
starting, the same step seen in the flight log two seconds after arming. So the camera
sensor and its ribbon cable are the source, not the Pi's processing and not the idle AI
board. Even with the camera off, 6 to 7 satellites is weak for open sky (the unit had 8 on
the bench rig indoors before it moved onto the pod lid), so the Pi underneath costs some
reception as well.

**Consequence for the project:** the chase needs the camera running and the GPS working at
the same moment. As built, it cannot have both.

## Power, measured for the first time

- Hover: 8.8 A at 22.4 V, 197 W, at the weighed 930 g. The prediction band was 7.3 to 9.8 A.
  With 80 % of a 1500 mAh pack usable that is about 8 minutes of hover. One flight, one pack,
  wind not recorded.
- The current sensor works (it read 0.00 A with only the avionics running, so it does not
  count them).
- Pack: 23.9 V resting before, 22.4 V average in hover, 20.7 V lowest. Voltage against
  current gives 32 milliohms.
- PX4's "battery remaining" fell from 0.85 to a lowest 0.21 in 80 s of hover while the
  sensor counted 178 mAh, 12 % of the pack. **Corrected reading (same afternoon):** I first
  called that a pessimistic gauge. Half an hour later the pack rested at 22.7 V, 3.79 V per
  cell, which really is about 43 %. The avionics draw current the power module cannot see,
  and the aircraft had been powered for about an hour, so the pack was emptier than the
  counted 178 mAh says. The 0.21 was a momentary sag; the board was already correcting for
  load with its own live estimate of the pack's resistance (0.021 read back).
- Changed on the board 2026-11-22, builder's go-ahead: `BAT1_CAPACITY` -1 to 1350 (90 % of
  the label), read back. `BAT1_R_INTERNAL` left at -1 (live estimate). The gauge now also
  counts charge used, which steadies it under a dash; it still cannot see the avionics'
  draw on the ground.

## After the builder wrapped the camera ribbon in foil (`gps_cam_test3_after_foil.out`)

| Condition | Satellites used (median, range) | Fix | Speed accuracy |
|---|---|---|---|
| Camera off, before | 7 (7 to 7) | 3D | 0.57 m/s |
| Camera recording 1920x1200 | 7 (5 to 7) | 3D throughout | 0.85 m/s |
| Camera off, after | 7 (7 to 7) | 3D | 0.67 m/s |

The same test without the foil gave 0 satellites and a lost fix. So the shielding removes
most of the camera's effect. What is left: one dip to 5 satellites (PX4 wants 6) and a
speed accuracy above PX4's 0.5 m/s arming limit, with or without the camera, so reception
as a whole is still weak (7 satellites under open sky; a healthy unit shows 15 or more).
One 30 s test, on the ground, motors stopped.

## Also seen, not followed up

- `navigator low on stack! (268 bytes left)` every 32 s.
- "heading estimate not stable" before arming; compass 1 reads 0.62 G on the ground and
  0.70 G in hover (the Earth's field is about 0.5 G), so it is picking up the aircraft.
- Height reference is set to GPS (`EKF2_HGT_REF` 1) on an aircraft whose GPS is unreliable.

Scripts in this folder reproduce every number above from the log. The raw `.ulg` files
are not in git (they hold GPS positions); copies are on the laptop in this folder and on
the Pi in `~/fc_logs`. The 2026-11-22 arming
refusal earlier the same afternoon is in `runs/interceptor_arming_gps_20261122/`.
