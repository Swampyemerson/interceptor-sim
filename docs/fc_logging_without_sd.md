# Flight logs without the Pixhawk's SD card

The interceptor's Pixhawk 6C Mini cannot mount a microSD card, so it writes no flight log
(ULog, PX4's flight-recorder file). This page is the substitute: PX4 sends the same log
over a cable to the Raspberry Pi, and the Pi writes it to its own card. The same Pi service
starts a camera recording when the Pixhawk reports "armed".

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
video command: outdoors add `--exposure-us 1000` so a fixed short exposure freezes motion
(the default is auto exposure).

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

Not tested yet:

- **A real arming.** The trigger has only seen a pretend signal. Check: props off, arm for
  10 s, disarm, then look for a new `sessions/video_*` folder and a `_flight.ulg`.
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

Evidence: `runs/fc_log_stream_20261113/` (stats for every capture, the Pixhawk console
output, the bench gate log, and `gcs_bridge/` for the ground-station tests). Decision
record: ADR-0137 and its addendum.

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
