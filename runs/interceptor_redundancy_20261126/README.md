# Flight code hardening, 26 November 2026

The builder asked for the vehicle code to be upgraded for flight: redundancy
wherever possible, a review of all flight code, a second link that takes over if
the TELEM2 cable fails, no low-battery trigger once GO is given, and no abort on
a GPS that has only got worse (only on one that is gone).

Nothing here has flown. Everything below is code, offline tests, and read-only
measurements on the aircraft sitting indoors with no flight battery connected.

## What can take the aircraft away from the program after GO

The flight program's own state machine had no battery or GPS trigger. The
Pixhawk does. When one of its failsafes acts, it leaves OFFBOARD (the mode in
which it flies the Pi's commands), the program sees that and goes SAFE. So the
list that matters is the board's. Read from the board today, read-only
(`board_failsafes.txt` holds the firmware source references):

| Trigger | Board setting today | After GO it would | Now |
|---|---|---|---|
| Low battery | `COM_LOW_BAT_ACT` 3; critical 7 %, emergency 5 %; charge judged from voltage alone (`BAT1_CAPACITY` -1) | return home at 7 %, land at 5 %. A full-power dash sags the voltage, which a voltage-only gauge reads as empty | **Suspended for a program flight**, see below |
| Not enough battery to fly home | `COM_FLTT_LOW_ACT` 3 | return home. Inert today (needs a capacity), live once `BAT1_CAPACITY` is set again | **Suspended for a program flight** |
| GPS quality drops | `EKF2_GPS_CHECK` 1023 (every check on); needs 6 satellites, 3.0 m horizontal, 1.0 m/s speed accuracy, PDOP 2.5 | stop using GPS after 5 s of failing checks; velocity then goes invalid and PX4 leaves OFFBOARD | **Suspended for a program flight** |
| GPS gone (no 3D fix) | fixed in firmware; 5 s ride-through (`EKF2_NOAID_TOUT`) | velocity invalid, PX4 leaves OFFBOARD | unchanged: this is the "literally zero" case |
| Radio signal lost | `COM_RC_LOSS_T` 0.5 s, `NAV_RCL_ACT` 2 (return), `COM_RCL_EXCEPT` 0 | return home after 0.5 s without the transmitter | **unchanged, builder's call** (see "Decisions for the builder") |
| Geofence | `GF_ACTION` 2 (hold), 457.2 m out, 30.48 m up | hold at the fence | unchanged; the program already refuses a rendezvous it believes is outside |
| Command stream stops | `COM_OF_LOSS_T` 1.0 s, then Position mode | pilot's aircraft | now takes BOTH links failing |
| Ground-station link lost | `NAV_DLL_ACT` 0 | nothing | unchanged |
| Wind, flight-time limits | `COM_WIND_MAX` -1, `COM_FLT_TIME_MAX` -1 | nothing | unchanged |
| Pilot's mode switch, kill switch | | always work | unchanged |

Noticed, not touched: the board reads `CBRK_IO_SAFETY` 22027 (safety button not
required), while `configs/px4_6cmini/bench.params` lists 0. That setting is the
builder's alone.

## What was built

### 1. A second link that needs no switch-over (`flight/deploy/link_redundancy.py`)

The flight program used one cable: the Pi's serial port to TELEM2. The
Pixhawk's USB port was already plugged into the Pi, held by the flight recorder,
which passes MAVLink to and from UDP port 14550. With
`--mavsdk-backup-url udpout://127.0.0.1:14550` the program now runs a second
MAVSDK server on that path.

- Velocity commands go out on both links every tick. There is no switch-over
  and no gap, and it also covers a break in the Pi-to-Pixhawk wire alone, which
  no check on the Pi could see.
- Telemetry is read from TELEM2. A sample from the USB path is used only when
  TELEM2 has been silent on that stream for 0.12 s (attitude, position,
  velocity), 0.5 s (radio channels) or 1.5 s (once-a-second status).
- Arm, takeoff, land, return and parameter calls go to TELEM2 and are tried on
  USB if that fails.
- A link that will not take a setpoint is ended (its server is killed), because
  a server left alive re-sends its last setpoint on its own timer.
- The watchdog now kills both servers.

What it does not cover: the Pixhawk, the Pi, the Pi's power, the program
itself. The USB path runs through the recorder, so it is only as alive as the
recorder is.

**Measured on the aircraft today, read-only** (`link_pair_probe_indoor.json.txt`,
20 s, both servers connected at once, recorder logging throughout):

| Stream | TELEM2 | USB through the recorder |
|---|---|---|
| attitude (quaternion) | 50.0 Hz, longest gap 0.026 s | 50.0 Hz, longest gap 0.028 s |
| attitude (angles) | 100.0 Hz, 0.014 s | 100.0 Hz, 0.020 s |
| flight mode, armed | 1.0 Hz | 1.0 Hz |
| landed state | 5.0 Hz | 5.0 Hz |
| GPS quality | 10.0 Hz | 10.0 Hz |
| battery | 0.9 Hz, longest gap 2.0 s | 0.8 Hz, 2.0 s |
| position, velocity | none | none |
| radio channels | none | none |

Position and velocity were absent because the aircraft was indoors with 4
satellites and no valid position, and the transmitter was off. So the two
streams the chase steers on most, and the GO switch, are **not yet measured on
the USB path**. The recorder check said GO afterwards with no packets lost.

Not yet done on the aircraft: sending commands on both links at once. That is
the props-off rehearsal (`--bench-props-off` with the backup URL), which needs
the builder to confirm the props are off.

### 2. Battery: before GO and after the intercept, never between

PX4 has no "only before GO" setting. So for a program-flown flight
(`--board-failsafes suspend`, the `--fly` default) the program changes three
settings while the aircraft is disarmed, and restores them after landing
(`flight/deploy/board_overrides.py`):

| Setting | Board | During the flight |
|---|---|---|
| `COM_LOW_BAT_ACT` | 3 | 0 (warning only) |
| `COM_FLTT_LOW_ACT` | 3 | 0 (no action) |
| `EKF2_GPS_CHECK` | 1023 | 0 (only loss of the 3D fix stops GPS use) |

What stands in for them:

- **Before takeoff** the program applies the board's own thresholds itself: no
  arming below the board's "low" battery level (15 %) or without a battery
  reading, and no arming unless every GPS reading of the last 3 s has a 3D fix,
  6 satellites, 3.0 m and 1.0 m/s.
- **In the standby hover**, a pack below the board's "critical" level (7 %) for
  3 s lands the aircraft. No GO on a low pack.
- **From GO to the end of the engagement** the battery is ignored. The log says
  so once.
- **In the hover after a miss**, a pack below 7 % for 3 s lands it.

Safety of the writes: disarmed only; the originals are saved to
`~/.interceptor/board_overrides.json` on the Pi before the first write; every
write is read back (over the other link when there are two); if a run dies the
next run restores first, and `scripts/field/recorder_check.py` says NO-GO while
that file exists. If the pilot takes the aircraft mid-flight the program does
not restore in the air; the settings stay until `python -m
flight.deploy.board_overrides --restore` is run on the ground.

`--board-failsafes keep` changes nothing on the board.

**Not measured:** the battery figure's scale. With no pack connected MAVSDK
reports -1.0, MAVLink's "unknown" for a 0 to 100 field, so the code takes the
figure as 0 to 100. The first run with a pack prints "battery NN %" before the
arm switch; check it against the pack. A wrong scale would refuse to arm.

### 3. Things that used to end a flight and no longer do

| Before | Now |
|---|---|
| One detector error (a decode or pose fault) escaped the control loop: the aircraft landed where it was and the tick log was lost | counted as a missed detection; the flight goes on |
| An exception in the state machine stopped the command stream | the last command is held; six in a row stop the loop and the normal landing runs |
| One failed command call was an exception | six in a row end the flight; fewer cost a tick each |
| The tick log reached the card only after landing | written while flying (`<log>.live`), short by at most a second after a crash |
| A NaN velocity from the Pixhawk (it sends NaN for an invalid one) made the chase's belief NaN for the rest of the flight | non-finite telemetry is dropped at the door; the stream ages instead |
| No check anywhere that a command is a finite number | a non-finite command cannot leave the state machine; it goes SAFE |
| A NaN in a target-position record could stall or kill the receiver before GO | refused and counted |

### 4. Two rules added because the others were removed

- **Dead camera.** With no time limit on the engagement and the battery ignored
  after GO, an aircraft whose camera had died would search blind until the pilot
  or the geofence stopped it. In ENGAGE, no frame at all for 3 s now goes SAFE.
  Frames with no target in them do not count.
- **Upside-down camera.** The reader can now rotate the image 180 degrees at
  the sensor (`--camera-mount upside-down`), `--fly` refuses to start until the
  mount is stated, and a calibration that was not captured with the same
  rotation is refused. Read unrotated, every bearing is mirrored and the
  aircraft steers away from the target.

## The review

Four reviewers were started. The two on the main flight program
(`real_flight.py`) were stopped by the model's safety filter and their part was
not redone, so **that file has had no independent line-by-line review today**;
I read the parts I changed. The two that finished covered the camera chain and
the guidance and target-position code.

Fixed today: detector exception, lost tick log, dead camera, non-finite
velocity and commands, non-finite target records, image rotation.

**Still open, in order of how much they matter:**

1. **No calibration of the AR0234 nose camera exists.** `configs/` holds only
   the simulator file and the old OV9281 one. Without it there is no camera
   intercept. It must be captured with the image rotated, and the capture tools
   have no rotation option yet.
2. **A brief own-state gap stops the chase.** If the oldest of four telemetry
   streams is over 0.25 s old the chase code returns no command, the wrapper
   then commands zero horizontal speed, and the belief is not advanced for the
   gap (measured by the reviewer: 0.9 m of movement where 11.7 m was due, for a
   0.6 s gap at 30 m/s). The second link makes such gaps much rarer; the logic
   is unchanged.
3. **The target's reported speed is never checked against its positions.** The
   belief takes the mean of the reported velocities; a wrong or lagging
   velocity field becomes the belief, and speed error is the limiter of the
   long dash.
4. **The two aircraft's clocks are assumed to agree.** PX4 sets its clock from
   GPS only when it is 5 s or more off. An offset shifts the measured link
   delay (0.1 s is 1.2 m at 12 m/s), and a large one stops GO from ever firing.
   Never measured on the hardware.
5. **The height of the belief is the difference of two separate GPS heights.**
   Yesterday's ground log had the interceptor's GPS height wander 7 m in 85 s.
6. **One bad target record moves the latch** (a 20 m outlier on the last of 28
   records moved it 2.7 m in the reviewer's probe).
7. **The camera and the video recorder.** The flight program opens the camera
   before arming; the recorder service starts its own video at arming. Two
   programs cannot both hold the camera, so on a program flight the recorder's
   video will fail. Not examined on the hardware.
8. The frame recorder reports a clean close after its writer thread has died
   (full card). The capture stamp is arrival time, not exposure time. No
   decode-quality gate on tags. Red and blue are swapped in recorded frames.

## Decisions for the builder

- **Radio loss.** Today, 0.5 s without the transmitter's signal ends the
  intercept with a return home. That is a trigger of the kind asked about, and
  I left it alone: with it removed, an aircraft that has lost its radio keeps
  chasing with nobody able to stop it. Recommended for the first flights: keep.
- **Flying a manual flight after a program flight died:** run the recorder
  check first; it now says NO-GO if the board's settings were left changed.

## Tests

`flight/tests/test_flight_redundancy.py`, 43 tests, including a paired control
for the second link: with the primary's telemetry frozen 2 s after GO, one link
ends in SAFE (stale own-state) and two links fly to the normal end. Dev laptop:
`flight/tests` 601 passed, 1 expected failure.
