# Target drone would not arm: microSD card refused new logs (2026-11-15)

Aircraft: the target (Kakute H7, ArduCopter 4.7.0, COM4). Decision record: ADR-0138 addendum 15.

## What was reported

QGroundControl showed "Ready to Fly"; arming in the yard did nothing; arming while connected
"spit out errors".

## What was measured

| Step | Result | Source |
|---|---|---|
| Pre-arm checks, board idle, 60 s at 2 Hz | pass in 122 of 122 samples | `prearm_poll.py` (C:\Temp\bench) |
| Builder arms by holding yaw right, three times | "Arm: Logging not started", "PreArm: Logging failed", "Arm: Logging failed" x2; never armed | `arm_watch_20261115.txt` |
| Same moments: battery, GPS, radio, mode | 24.36 V, fix 4 with 19 to 23 satellites, rssi 229 to 254, Loiter | same file |
| Start a log without arming (`LOG_DISARMED` 1 for 25 s), BEFORE format | "PreArm: Logging failed"; logging unhealthy in 49 of 51 samples; no new log; log 21 still 0 bytes | `scripts/bench/ap_log_start_test.py` |
| Card format over MAVLink | "Format: OK" after 32 s | `scripts/bench/ap_sd_format.py` |
| Same test AFTER format and a reboot | no failure message; unhealthy in 2 of 51 samples (the first second); one new log of 8,183,808 bytes | `ap_log_start_test.py`, log list |
| Pre-arm checks after, 20 s | pass in 40 of 40 samples | `prearm_poll.py` |

The check only fails at arming because, with `LOG_DISARMED` 0, the board opens its log file at
the moment of arming (ArduPilot `AP_Arming.cpp`, Copter-4.7.0: `PrepForArming()` then
"Logging not started"). That is why the ground station showed ready.

## What is and is not established

- Established: the card accepted no new log file before the format and accepts one after.
- Not established: why. A file system damaged by power being removed while a log was open fits
  (flight five's log was 0 bytes), and a format would cure that. A failing card also fits and a
  format would only hide it for a while. If "Logging failed" returns, replace the card.
- Not done: an actual arm. The board was never armed by these tools.

## Logs

All 20 logs that were on the card are on the PC under `C:\Temp\bench\target_*_20261114\`
(logs 4 to 15, 19, 20, pulled earlier) and `C:\Temp\bench\target_card_backup_20261115\`
(logs 1, 2, 3, 16, 17, 18, pulled before the format; each parsed end to end, 8,050 to 192,150
messages). They hold positions and stay out of the repo. Log numbering on the card restarts
at 1; log 1 is the 25 s disarmed test.

## Also found

- `WP_SPD` on the board read 10 m/s, not the 3 m/s set in addendum 12. Who or what restored it
  is not established. Set back to 3 and read back.
- The uploaded mission (13 items: take-off to 3.04 m, five waypoints, jump to item 2 ten times)
  carries a 5 m/s speed change after every waypoint, which overrides `WP_SPD`.
- The arm switch channel (ch5) read 2000 for the whole session; he arms with the yaw stick.
