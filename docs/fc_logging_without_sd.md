# Flight logs without the Pixhawk's SD card

The interceptor's Pixhawk 6C Mini cannot mount a microSD card, so it writes no flight log
(ULog, PX4's flight-recorder file). This page is the substitute: PX4 sends the same log
over a cable to the Raspberry Pi, and the Pi writes it to its own card. The same Pi service
starts a camera recording when the Pixhawk reports "armed".

## Before a flight: one command says GO or NO-GO (added 2026-11-22)

```bash
ssh admin@192.168.0.61 'cd interceptor-sim && .venv-pi/bin/python scripts/field/recorder_check.py'
```

It prints one line per check and ends `RECORDER: GO` or `RECORDER: NO-GO`: the service is
alive, the log is arriving over USB at full rate, no packets were lost in the last 30 s,
the video is ready to start at arming, and the card has room. Add `--camera` to also record
3 s of test video with the same program the service runs at arming. No answer (service
stopped, status older than 5 s) is a NO-GO.

With QGroundControl connected through the Pi (next section), the recorder also puts its
own lines in QGroundControl's message list, each starting `Pi:`, for example
`Pi: video recording started`, `Pi: Pixhawk cable LOST, armed. Video still on`,
`Pi: log on SERIAL cable, lossy. Plug USB in Pi`, `Pi: NO VIDEO: card nearly full`.
These go to the network only, never to the Pixhawk. Checked with a script standing in
for QGroundControl; not yet looked at on QGroundControl's own screen.

## What it does when something goes wrong (added 2026-11-22)

| Fault | What the recorder does | Bench result, 2026-11-22 |
|---|---|---|
| Pixhawk USB cable pulled or loose, disarmed (the compass calibration case below) | Closes the log file as it stands (verdict `LINK_LOST`), moves to the serial cable, and moves back to USB by itself when the cable returns. The service does not restart. | Cable out 30 s, then back: 0 service restarts, on USB again 1 s after replug, check read GO 38 s after replug |
| Pixhawk USB cable lost while armed | The same, and **the video keeps recording through it**. It stays on the serial cable until disarm. | One video ran unbroken across the loss (35.8 s, 2,131 frames, none dropped) |
| USB lost while the flight program is using the serial cable | Does **not** take the serial cable (two readers would each get half the bytes and the guidance link matters more). No log until USB is back; video carries on. | Refused the busy cable, said so, resumed on USB when it returned |
| Camera recording dies while armed | Started again, every 5 s, 12 times a flight at most. Each start is a new `video_*` folder. | Recording killed by hand: a new one was running within 1 s |
| Camera stops delivering frames | `record_video.py` exits when its file has not grown for 3 s, and is then restarted as above. | Logic tested offline only; no way to stall the camera on the bench |
| PX4 accepts "start log" and sends nothing | Asks again after 3 s (was 10 s). | Seen twice on the bench right after a cable change; recovered both times |
| Recorder program crashes | systemd restarts it after 1 s, for ever (before, it gave up after 5 quick failures). A file cut off mid-write is kept as `_recovered.ulg`. | Restart and recovery seen 2026-11-13 and 2026-11-22 |
| Card nearly full | Under 5 GB free: no video (the log still records). Under 2 GB: logging stops. The check warns under 10 GB. | Offline test |
| Battery pulled | Log and video are pushed to the card every second; about the last second is lost. | 2026-11-13 |

Before this change a pulled USB cable crashed the recorder; systemd restarted it within
a few seconds, but a video running at that moment would have been stopped and begun again.

**What these tests did not cover:** the cable was "pulled" in software (the Pi's USB
driver was told to let go), not by hand, and "armed" was the recorder's own pretend
flag. A real pull also removes USB power from the Pixhawk's port, which the software
pull does not. Evidence: `runs/recorder_fault_test_20261122/`.

### Compass calibration with the USB cable moved to the PC

1. Unplug the Pixhawk's USB from the Pi and plug it into the PC. The recorder moves to
   the serial cable within about 5 s and keeps logging (lossy there, which does not
   matter on the ground).
2. Calibrate in QGroundControl.
3. Plug the USB back into the Pi. The recorder moves back within a few seconds.
4. Run the check above and wait for `RECORDER: GO` before arming. It reads NO-GO for the
   first 30 s after the cable returns, because the changeover costs a few packets.

If the Pixhawk asks for a reboot after calibration, power-cycle the whole aircraft; the
Pi restarts with it and the service starts by itself (about half a minute).

Do not arm while the recorder is on the serial cable: about a third of the log is lost
there, and the flight program cannot open that cable while the recorder holds it.

## What is set up (2026-11-13)

A service called `flight-recorder` runs on the Pi from boot. For every flight it writes:

| What | Where on the Pi | Notes |
|---|---|---|
| The Pixhawk's flight log | `~/fc_logs/fc_<time>_flight.ulg` | One file per flight, from before arming to 5 s after disarm. Opens in Flight Review, PlotJuggler or `pyulog`. |
| A report on that log | `~/fc_logs/fc_<time>_flight.stats.json` | Verdict (`CLEAN` or `GAPS`), packets lost, which cable was used, whether the video ran. |
| Camera video | `~/interceptor-sim/sessions/video_<time>/` | 1920x1200 at 60 frames/s, starts about 1.6 s after arming, stops 5 s after disarm. `meta.json` reports dropped frames. |

Time on the ground is recorded too, in rolling 10-minute `_ground` files; only the newest
three are kept. Both the log and the video are pushed to the card every second, so pulling
the battery costs about the last second. A file that was cut off is trimmed to its last
whole record and kept as `_recovered.ulg` the next time the Pi starts.

Of its own accord the service sends the Pixhawk three things only: "start log stream",
"stop log stream" and packet acknowledgements. It sets no parameters and does not arm.
It also carries a ground station's traffic both ways (next section), and a ground station
can do anything it could do over a USB cable.

### QGroundControl over Wi-Fi, through the Pi (added 2026-11-14)

The Pixhawk's USB cable can stay in the Pi. The recorder passes everything the Pixhawk
says on to the PC over the network and passes QGroundControl's messages back, so
QGroundControl (the ground-station program: settings, calibration, live status) works
without moving the cable. The log packets are held back from the ground station, so
logging carries on underneath.

- **To connect:** have the PC on the same Wi-Fi as the Pi and open QGroundControl. It
  finds the Pixhawk by itself (8 s after launch on 2026-11-14): the Pi announces it on UDP
  port 14550, the port QGroundControl listens on. If it does not, add the link by hand:
  Application Settings, Comm Links, Add, type UDP, port 14550, server address
  `192.168.0.61:14550`.
- **Connect before you arm, and wait for QGroundControl to finish loading.** The first
  minute after it connects cost the log about 45 s of data on 2026-11-14 (7,332 packets
  in two holes, beginning 11 s and 56 s after it connected), while QGroundControl fetched the
  settings and their descriptions. After that the log ran with no further gaps for as
  long as QGroundControl stayed connected. A flight's file starts fresh at arming, so a
  ground station that is already loaded costs it nothing.
- **Speed depends on the Pixhawk-to-Pi cable.** Over USB a full settings download took
  0.7 s in the bench test. Over the 3-pin serial cable it took 5.3 s, because that cable
  is already full with the log.
- **Anyone on the same Wi-Fi can reach the Pixhawk through this port**, with no password,
  the same as any Wi-Fi telemetry link. On the home network that is your own devices. To
  turn the bridge off, set `GCS_PORT=0` in `/etc/default/flight-recorder` and restart
  the service.
- **Losing the link in flight does nothing.** The Pixhawk's "ground station lost" action
  (`NAV_DLL_ACT`) reads 0, disabled, on this board (read 2026-11-14).

Only one program can receive the log stream at a time. A second recorder (for example
one run by hand on the PC's USB port while the Pi's service is running) takes the stream
away, and the two then steal it back and forth every 10 s. Stop the service first.

### The one thing it needs from you: a USB cable

| Pixhawk-to-Pi cable used for the log | Log completeness | Effect on the guidance link |
|---|---|---|
| **USB** (Pixhawk USB-C to a Pi USB port) | Complete: 40 KB/s, one 4-packet gap in the first 5 s, none after | None. Serial link stayed at 100 Hz attitude; the offboard bench gate passed alongside it. |
| **Serial only** (the existing 3-pin cable) | About a third of the packets lost | Attitude stream on that cable halves, 100 to 50 Hz, and nothing else can use the cable while the recorder holds it. |

The service picks USB by itself when the Pixhawk's USB is plugged into the Pi and uses the
serial cable otherwise. Since 2026-11-14 the Pixhawk's USB is in the Pi and the logs are
complete. The airframe needs the same: a short USB-C to USB-A cable between the two boards.

While the recorder is on the serial cable, stop it before running anything else that
talks to the Pixhawk over that cable (the offboard bench gate, the hover recorder, the
seeker): `sudo systemctl stop flight-recorder`, and `start` it again afterwards. A second
program opening the port gets "Device or resource busy" instead of half the bytes.

### Commands

```bash
journalctl -u flight-recorder -f              # watch it (on the Pi)
sudo systemctl stop flight-recorder           # free the serial cable and the camera
sudo systemctl start flight-recorder
journalctl -u flight-recorder | grep "ground station"   # when QGroundControl connected and left
scripts/pi_setup/install_flight_recorder.sh   # install or update after a git pull
scp -r admin@192.168.0.61:fc_logs .           # copy the logs to the laptop
```

Settings are in `/etc/default/flight-recorder` on the Pi. The one worth changing is the
video command's exposure. The default is auto exposure, which chose about 150 µs in full
sun on 2026-11-15; a fixed 1000 µs clipped 56% of a sky frame in that light. So outdoors
leave it on auto or fix it near 150 (`--exposure-us 150`), not 1000 as this page used to say.
The video command carries `--flip` because the nose camera is mounted ribbon connector up:
on 2026-11-22, with the aircraft level, the picture was upside down without it and upright
with it (`runs/recorder_fault_test_20261122/camera_frame_*.jpg`). **The flip is in the
video recorder only. The flight program's camera reader has no flip yet**, so the seeker
would see the target on the wrong side; that has to be written before any camera-guided
flight (`docs/next.md`).

## What has and has not been tested

Tested on the bench, Pixhawk disarmed:

- Log over USB: three captures from the PC's USB port, the last one 45 s, verdict `CLEAN`,
  while the Pi ran the offboard bench gate over the serial cable (300 of 300 setpoints,
  largest gap 0.05 s, oldest own-state reading 0.02 s; the same figures as the 2026-10-18
  run with no logging).
- Log over the serial cable: works, lossy, as the table says.
- Arm trigger: with the recorder told to pretend "armed" for 12 s, the camera started
  1.6 s later, recorded 918 frames at 59.5 frames/s with none dropped, and stopped 5 s
  after the pretend disarm.
- Hard kill of the recorder mid-file: the file was recovered on the next start.
- USB plugged into the Pi (2026-11-14): the service moved from the serial cable to
  `/dev/pixhawk-usb` by itself when the port appeared, and recorded at 41 KB/s
  with one start-up gap and none after (four minutes, until QGroundControl connected).
- QGroundControl itself through the bridge (2026-11-14): found the Pixhawk unaided and
  stayed connected; cost described under "QGroundControl over Wi-Fi" above.
- Ground-station bridge, with a script standing in for QGroundControl (it listens on port
  14550, answers the Pixhawk's announcement and downloads every setting):
  over USB from the PC, all 1,122 settings in 0.7 s while a 60 s log recorded `CLEAN`
  (one start-up gap, none after) and attitude stayed at 100 Hz; over Wi-Fi through the
  Pi's service on the serial cable, found by itself in 0.25 s, all 1,122 settings in 5.3 s.
  The Pixhawk's console was also read from the laptop through the Pi.

- Live log read back on the assembled aircraft (2026-11-22, on battery, disarmed): a 108 s
  file opened in `pyulog` with 93 topics; 97 packets were lost in 5 gaps, about 0.54 s of
  data in all, every one in the first 47 s while QGroundControl was connecting.
- Fault handling (2026-11-22): the table under "What it does when something goes wrong".

- **A real arming and a real flight (2026-11-22):** 263 s log, verdict `CLEAN` (6 packets
  lost at start-up, none after, 43 KB/s armed), and 88 s of video that started 1.6 s after
  arming: 5,236 frames at 59.5 per second, 4 gaps over 1.5 frames, worst 50 ms.

**The video the service starts at arming jams the GPS (found 2026-11-22).** With the
camera running the GPS loses its fix within a second; the flight above lost its position
3.5 s after takeoff for that reason (`runs/interceptor_flight_20261122/README.md`). The
builder then wrapped the camera ribbon in foil and the fix held at 7 satellites through a
30 s recording on the ground; not yet confirmed in flight. If it fails again, to fly with
GPS and no video set `VIDEO_CMD=` (empty) in `/etc/default/flight-recorder` and restart
the service; the log still records.

Not tested yet:
- **A cable pulled by hand**, and a cable lost with the motors running.
- **The avionics' share of the battery current.** With the motors stopped the Pixhawk's
  log shows 0.00 A, so the avionics either draw less than the sensor can see or are wired
  ahead of it. The sensor itself works: it read 8.8 A in hover on 2026-11-22.
- **File dates away from Wi-Fi.** The Pi has no clock battery. At power-up its clock
  resumes from the last time it was running (2026-11-22: it woke believing it was the day
  before until the network corrected it), so in the field the file names carry an old
  date. The true time is inside the log once the GPS has a fix.
- **An armed, flying log.** The 40 KB/s figure is for a disarmed aircraft.
- **Connecting QGroundControl while armed.** Its connection burst cost the log about 45 s
  on the ground (see above); nobody has measured it in flight, and the plan is not to do it.
- **Lining the video up with the log in time.** The stats file stores pairs of Pi clock and
  Pixhawk clock, and the video stores its start time; nobody has checked how well they agree.

Known limits:

- The log is PX4's default set: gyro at 50 Hz, attitude at 20 Hz. PX4's high-rate set for
  filter tuning has not been tried and may not fit: PX4 gives the log at most 70 KB/s over
  USB and the default set already uses 40.
- With USB connected PX4 does not play the critical-battery warning tune (read from the
  PX4 source, not tested). The battery failsafe itself is separate. This firmware allows
  arming with USB connected by default.
- On a flight where the seeker owns the camera, the video command fails to open it and
  the log's stats file says so. The log itself is unaffected.

## Why log streaming was "accepted once, then refused" on 2026-10-25

PX4's logger refuses a start request while it believes a stream is already running, and
only a stop request clears that. The first client was killed without sending stop, so
every later start was refused until reboot. The recorder now sends stop before every
start. Reproduced on the bench: session ended without stop, next bare start refused
(result code 1), stop-then-start accepted.

## Tools

| Tool | Use |
|---|---|
| `scripts/bench/ulog_stream_recorder.py` | The recorder. `--port COM5 --seconds 60` on the PC or `--port /dev/ttyACM0` on the Pi for a one-off capture. |
| `scripts/bench/px4_shell_readonly.py` | Status commands in the Pixhawk's console over any MAVLink cable (`logger status`, `mavlink status`, `dmesg`). Refuses anything not on its read-only list. |
| `scripts/bench/hover_recorder.py` | Older, lighter fallback: attitude and body rates at 100 Hz over the serial cable, as CSV. Needs the flight recorder stopped if that is on the serial cable. |
| `scripts/field/record_video.py` | The camera recorder the service starts. |
| `scripts/field/recorder_check.py` | The go / no-go above. Reads `/dev/shm/flight_recorder_status.json`, which the service rewrites every second. |

Evidence: `runs/fc_log_stream_20261113/` (stats for every capture, the Pixhawk console
output, the bench gate log, and `gcs_bridge/` for the ground-station tests);
`runs/recorder_fault_test_20261122/` (the fault tests: scripts, outputs, stats, the
ground-station messages received, and a camera frame with and without the flip).
Decision record: ADR-0137 and its addenda.

## Predictions, written before the first measurement (2026-11-13)

Set-up: Pixhawk on the bench, disarmed, PX4 v1.16.0, talking to the Pi over the 3-pin serial
cable at 921600 baud. PX4's shell reports the logger running with 166 topics, "not logging",
no SD mounted, and the serial link capped at 46,080 bytes/s with 24,042 already used by
ordinary telemetry.

1. **Why streaming was "accepted once, then refused" on 2026-10-25.** PX4's logger refuses a
   start request while it believes a stream is already running, and only a stop request
   clears that. A client that dies without sending stop leaves it stuck until reboot.
   Prediction: a start sent after a session that was killed without a stop is refused
   (result code 1, "temporarily rejected"); the same start sent after a stop is accepted.
   Three sessions in a row start cleanly when each begins with stop-then-start.
   *If this is wrong* (start refused even after a stop), the refusal has another cause and
   the serial path needs a reboot between sessions, which makes it unusable in the field.
2. **The full log does not fit down the serial cable at today's settings.** PX4 gives the
   log stream at most 70% of the link cap (about 32 KB/s), and scales ordinary telemetry
   down to make room. Prediction: gaps in the log's packet numbering, and the attitude
   stream the guidance code reads falls below its normal 100 Hz while the log is streaming.
   *If this is wrong* (no gaps, attitude stays at 100 Hz), the existing cable is enough and
   nothing else needs changing.

A path counts as a usable flight recorder when a 60 s capture opens in `pyulog` (the
standard ULog reader), has no packet gaps, and leaves the attitude stream at 100 Hz.

## Results (same day)

Both predictions held.

1. A session ended without a stop command; the next bare start came back "temporarily
   rejected"; stop-then-start was accepted every time, in more than ten sessions that day.
2. Over the serial cable, 20 s: 2,294 packets received, 966 lost in 163 gaps, and the
   attitude stream on that cable read 50.0 Hz against 100.0 Hz before. The file still
   opens in `pyulog` (100 topics, 1,122 parameters) with 163 dropouts marked.

Over USB the 60 s test met the bar except for one gap at the very start: 4 packets lost
as PX4 switched from the acknowledged header to free-running data, none in the 55 s after,
every topic at its normal spacing, and the serial cable undisturbed at 100.1 Hz. A first
USB run lost 74 packets at that same moment because the recorder acknowledged the header
slowly (22 s for the header against 4.9 s after the fix). The recorder reports a single
gap in the first second of data as a start-up gap and anything else as `GAPS`.
