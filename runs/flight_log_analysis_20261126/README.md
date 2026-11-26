# What today's flight logs measure, against what the project assumes (26 November 2026)

Read from the logs of both aircraft after the field session. Analysis only: nothing
was changed on either aircraft, and no setting, plan or status file was edited.

**Where the logs are.** Raw logs are not in git (they carry GPS positions).
On the laptop:

- Interceptor (PX4): `runs/interceptor_flight_20261126/` (five logs, 19:01 to 19:21 UTC)
  and `runs/interceptor_flight_late_20261126/` (flights at 19:31 and 20:02, two ground
  logs covering the rehearsal runs; copied off the Pi today).
- Target (ArduPilot): `runs/target_flight_20261126/` (logs 1, 2, 4, 5, 7 of the seven on
  its card, downloaded read-only over USB; logs 3 and 6 are a 0.1 MB stub and 40 MB of
  sitting powered during the rehearsal, not fetched).
- The target's position reports as the laptop received them during the rehearsal:
  `C:\Users\Emers\crsf_raw_run3.bin` (17 minutes, 2 a second).

**How to rerun.** Each `.txt` and `.json` here is the output of the script of the same
name; the command is in the script's first lines.

## The short version

1. **The GPS wall of 22 November did not appear today.** In six flights the position
   estimate was never lost in the air (22 November: lost in all three). But three flights
   in a row, 19:19 to 19:34, flew on visibly worse GPS (13 to 15 satellites, speed
   accuracy 0.62 to 0.64 m/s) and in those the hover wandered 1.6 m and the heading
   swung 11 to 29 degrees with hands off. Nothing in the logs says what differed.
2. **The battery gauge ends flights early.** The low-battery failsafe fired in three of
   six flights. On a freshly charged pack it called "critical" after 142 seconds in the
   air. Either the packs deliver about 600 to 700 mAh or the current sensor reads low by
   more than half; the plan's "8 minutes" assumes neither.
3. **Camera angle: the camera already sits at about 17 to 21 degrees up, not the ruled
   10.** The pitch the aircraft needs at chase speed is still not measured: nobody has
   flown faster than 5.3 m/s. The logs support 20 over 10 by extrapolation only.
4. **The target's backyard course is smaller than GPS error.** It flies about 1 m/s
   (2 mph) round a course roughly 5 by 6 m, turning about 20 degrees a second, while its
   own GPS position wanders 1.5 to 1.9 m and its GPS height 4 to 9 m. A straight-line
   estimate frozen at GO cannot describe that course for more than a second or two.
5. **Height from the two GPS receivers disagrees by more than the chase tolerates.**
   Sitting on the ground the target read 4.8 m higher than the interceptor (the builder
   puts the real difference at 1.5 to 2 m), with swings of several metres either way.

## Comparison table

"Assumed" is what the contract (`docs/project_state.json`), the plans or the fast sim use
today. "Measured" names its source file in this folder.

### GPS and the estimator (interceptor)

| Quantity | Assumed / before | Measured today | Verdict |
|---|---|---|---|
| Position estimate lost in flight | lost in every flight on 22 Nov: 78 of 82 s, 73 of 136 s, 4 of 50 s (`flights_20261122.json`) | 0.0 s in all six flights, 380 s in the air (`flights_20261126.json`) | **Changed.** "GPS is the wall" (`docs/next.md`) no longer describes the aircraft as it flew today |
| Satellites in flight, camera recording | 5 to 11 on 22 Nov | 15 to 24 in three flights; 12 to 18 (median 13 to 15) in the three flights 19:19 to 19:34 | Better, but two populations in one afternoon |
| GPS's own speed accuracy | board limit was 0.5, now 1.0 m/s; 22 Nov median 0.51 to 1.02 | median 0.19 to 0.22 in the good flights; 0.62 to 0.64 (max 0.80) in the 19:19 to 19:34 flights | Those three flights would have lost position under the old 0.5 limit. The 1.0 limit is what kept them |
| Position estimate jumps (resets to GPS) | not tracked | 0, 0, 6, 6, 15, 2 per flight; the 15 is the 19:21 flight | Open: jumps move the aircraft's idea of where it is, mid-flight |
| Compass field strength | Earth's field 0.51 G | 0.42 to 0.48 G in the good flights, 0.61 to 0.63 G in the 19:16 to 19:34 flights | Still not right; the outdoor recalibration is owed. A calibration attempt at 19:01 failed ("calibration failed: mag", `flights_20261126.json`) |
| Pixhawk's reported height against the GPS receiver's | assumed the same kind of height until today | GPS higher by 97 to 107 m (median per flight), moving between 92 and 114 m through the day | **Confirms** the fix made today in the flight program. The offset is not a constant: it moved 20 m in an afternoon |

### Hover and slow flight (interceptor), relevant to a slow cue-only flight

| Quantity | Assumed | Measured today | Verdict |
|---|---|---|---|
| Hands-off hover wander, Position mode | not stated | 0.06 to 0.12 m RMS (0.14 to 0.32 m span) in good-GPS flights; 0.43 to 0.53 m RMS (1.6 m span) in the poor ones (`flights_20261126.json`, holds of 6 s or more) | Measured. Good enough for a slow flight only when the GPS is in its good state |
| Heading held in a hands-off hover | 1 to 2 degrees (contract assumption `standby-yaw-hold`, "asserted, never measured") | 0.4 to 1.6 degrees RMS, 2.7 to 4.7 degrees span in the good flights; 11 and 29 degrees span in two poor-GPS flights (`hover_yaw_and_modes.txt`) | **Measured for the first time.** Holds in the good state, fails in the poor one |
| Velocity following its command, Position mode | fast sim: 0.12 s lean lag plus 0.02 s delay | lag 0.12 to 0.16 s, 84 to 88 % of the commanded size, in three flights; 0.40 to 0.56 s and 61 to 66 % in the 19:21 poor-GPS flight | Agrees with the sim in the good state. This is the same controller OFFBOARD uses |
| GPS receiver's position, aircraft sitting still | search declares 3 m of cue position error, one sigma | interceptor: 0.8 to 0.9 m RMS, up to 2.7 m of movement inside one minute (`ground.txt`). Target, from its reports: 1.5 to 1.9 m RMS, about 3 m inside one minute (`target_reports_still.txt`, first 5 and 11 minutes) | The declared 3 m is not optimistic for position. Both errors add when one aircraft is placed against the other |
| Speed a 5 s fit reports for a target that is not moving | declared cue speed error 10 % | median 0.04 to 0.05 m/s, 95th percentile 0.09 to 0.13 m/s (`target_reports_still.txt`) | At 9 m/s that floor is about 1 %. At 2 mph (0.9 m/s) it is 5 to 15 %. A moving target's error is still not measured |
| Height difference from the two GPS receivers | chase tolerates the target up to 2 m off, 3 m above breaks acquisition (contract `target-height-known`) | target read 4.8 m higher by medians, against a real 1.5 to 2 m (builder); target's reported height ranged 1640 to 1649 m sitting still; interceptor's GPS height spanned 4 to 8 m | **Exceeds the budget.** Two GPS heights cannot supply the height difference the camera chase needs |

### Speed, lean, power (interceptor)

| Quantity | Assumed | Measured today | Verdict |
|---|---|---|---|
| Top speed flown | board limit 33 m/s; fast sim peaks near 26 | 5.3 m/s (20:02 flight) | **Not measured.** No flight has gone faster than 5.3 m/s |
| Largest lean | limit 45 degrees | 44.5 degrees, in Position mode, at 5 m/s (`hover_yaw_and_modes.txt`) | The lean limit is reached in ordinary stick flying, long before any speed limit |
| Horizontal acceleration | board limit 5 m/s2 on commanded speed; coded dash assumes 10 | 95th percentile 5.2, peak 12.3 m/s2 (20:02 flight) | The board does produce more than 5 on the sticks; whether the 5 limit binds OFFBOARD commands is still open |
| Drag (how much lean a speed costs) | fast sim 0.25 per second, never fitted to this aircraft | 0.26 to 0.29 per second (`drag.txt`), from 15 s of flight above 1 m/s, none above 4.2 m/s, correlation 0.68 | Consistent with the sim, weakly. Says nothing above 4 m/s, where drag stops being linear |
| Hover power | 8.8 A; motor command 24 to 25 % | 8.5 to 8.9 A; 24 to 27 % | **Confirmed** |
| Full-throttle thrust | 4.2 to 1 on a sagging pack (22 Nov) | no full-throttle today: highest motor command 0.59, 1.3 g | Unchanged, not re-measured |
| Pack resistance | 55 to 61 milliohm (22 Nov) | 24 to 60 milliohm from fits at currents under 21 A | Fits at low current are loose; no change claimed |
| Flight time | "about 8 minutes" on a 1500 mAh pack at 8.8 A (contract key number) | pack 1: full to a resting 3.67 V per cell after 544 mAh counted. Pack 2: "critical battery" failsafe 142 s after take-off, 306 mAh counted, resting 3.75 V per cell afterwards (`battery.txt`) | **Contradicted.** The gauge is voltage-only and the failsafe is live: today's flights were limited to 2.5 to 4 minutes |

Battery, in more words: the gauge takes charge from voltage alone (`BAT1_CAPACITY` -1).
By the current sensor, about 550 mAh took pack 1 from 4.06 to 3.67 V per cell at rest,
which is most of a pack. So either these packs give about 600 to 700 mAh, or the current
sensor reads less than half the true current. The logs cannot tell which: the pack's
label and a charger's put-back figure can.

### The target

From `target_20261126.json`. Two automatic flights today.

| Quantity | Assumed | Measured today | Verdict |
|---|---|---|---|
| Speed on its backyard course | "2 mph" (0.9 m/s) for the slow course; 9 to 12 m/s for the project's goal | median 0.87 m/s (log 2, 205 s in Auto) and 1.04 m/s (log 5, 98 s); between 0.6 and 1.4 m/s most of the time, peak 1.8 | About 2 mph, as intended |
| Size of the course | not stated | about 5 to 7 m by 5 to 6 m by its own GPS | Smaller than the sum of the two GPS errors |
| Turn rate | search and chase assume a straight target; a weaving target is "rarely caught" | median 19 degrees a second, a tenth of the time above 77; only 17 to 24 s of each flight within 5 degrees a second of straight | **This course is all turn.** A straight-line estimate is stale within a second or two |
| Reported course on its straight stretches | route sends speed and course, used as a velocity | scatter 11.5 to 12 degrees RMS at this speed; speed scatter 0.11 to 0.15 m/s | At 1 m/s the reported course is poor |
| Height hold | 2 m band tolerated | 3.6 m and 5.0 m flown; error 0.10 and 0.37 m RMS, worst 1.0 m | Holds height well. Its GPS height meanwhile spanned 4.5 to 4.9 m |
| Battery | low-battery return at 21.6 V | log 5 fell to 19.85 V at 24.9 A | Below its return threshold momentarily; not looked into further |

## Camera angle: what the logs give, and what a ruling still needs

**What is measured**

- **As built, the camera looks up about 17 to 21 degrees.** A frame from today's 20:02
  flight (aircraft pitched 1.3 degrees nose down, 3.9 m up) puts the horizon about 210 to
  230 pixels below centre of the half-size frame: 19 to 21 degrees of camera elevation,
  so 20 to 22 on the body, give or take 3 because trees hide the horizon. The 22 November
  reading by the same method was 17, give or take 2. Both assume a 3.6 mm lens; no
  calibration exists. The frame is not kept in git (it shows the surroundings).
- **In a hover the nose sits 1 to 2 degrees down** (median, six flights).
- **Stick flying reaches the 45 degree lean limit at 5 m/s.** Speeding up and braking, not
  cruising, produce the largest nose-down angles: 33 to 44 degrees.

**What is not measured**

- **Pitch at chase speed.** The chase flies 1.5 to 6 m/s faster than a 9 to 12 m/s
  target, so 10 to 18 m/s. The fastest flight so far is 5.3 m/s. With the drag fitted
  above and assuming it stays linear, steady flight would need about 14 to 15 degrees
  nose down at 9 m/s, 18 to 20 at 12 and 23 to 26 at 16 (`drag.txt`). Real drag grows
  faster than linear, so these are low if anything.
- **The picture's height in degrees.** No calibration of the nose camera.

**What it says about 10 against 20.** To keep a target that is level with the aircraft in
the middle of the picture, the camera's up-angle has to equal the nose-down pitch. On the
extrapolation above, 10 degrees puts such a target 4 to 16 degrees above centre across
the chase speeds and 20 degrees puts it within 6 degrees of centre. That agrees with the
18 November sim rerun (10 loses the tag off the top, 20 holds it) and with the camera as
it is already mounted. It is an extrapolation from 15 seconds of flight under 4.2 m/s.

**What a ruling needs**

1. One straight pass at 10 m/s or more, held for 5 seconds, logged. That gives the pitch
   at chase speed directly.
2. The nose camera's calibration, for the picture's height in degrees.
3. The as-built angle read with a level on the plate, to replace the two video readings.
4. A decision on how hard the chase may accelerate, because the hard nose-down moments
   are accelerations: at 35 to 44 degrees nose down no plate angle under 25 keeps a
   level target in the picture.

## Other things these logs can still give

- The 19:19 to 19:34 flights: what made the GPS and compass worse for 15 minutes. The
  video for those flights is on the Pi; where the aircraft was and what was powered
  nearby (the target, the Starlink dish) is not in the log.
- The target's reports against its own log: the laptop file and the target's log 2 or 5
  overlap in time if the handset was cabled then, which would measure the control-link
  route's delay and loss on a moving target. Not attempted.
- The target's log 7 (Stabilize, after the session, battery sensor reading 0.01 V) and
  log 6 (40 MB, sitting powered) were not examined.
- A paired look at the two aircraft flying at once: none of today's logs has both in the
  air together.

## Limits of this analysis

- "Wander" in a hover is measured against the aircraft's own position estimate, which
  itself follows the GPS. True movement over the ground was not measured.
- The still-target figures use the first 5 and 11 minutes of the laptop file; the
  aircraft may have been handled later in the file.
- The velocity-following lag is from command to response inside PX4's Position mode,
  where the command is already smoothed. OFFBOARD commands are not smoothed the same way.
- Good-state and poor-state GPS are my grouping of six flights by their numbers; six
  flights is a small sample.
