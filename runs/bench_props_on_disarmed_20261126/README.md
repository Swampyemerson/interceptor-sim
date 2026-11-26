# Target-to-interceptor loop, first runs on the real hardware, 26 November 2026

Two runs of the flight program's rehearsal mode (`--bench-props-off`) on the real
Pi and Pixhawk at the flying site, with the target powered and sending its
position. The interceptor had its flight battery in and its props ON, disarmed
(the mode cannot arm; the builder was at the aircraft). The session that started
the runs was stopped by the model's safety filter before it reported. This note
was written afterwards from the files in this folder, pulled from the Pi.

Nothing here flew under program control. GO never fired in runs 1 and 2. Runs 3 and 4,
made afterwards with the builder at the switches (kill switch on), are at the end.

## What was running

```
target autopilot -> control link (ExpressLRS) -> handset USB (COM3)
   -> laptop: scripts/field/crsf_cue_source.py --port COM3 --to 192.168.1.249
   -> Wi-Fi (Starlink router) -> Pi UDP 47810
   -> flight.deploy.real_flight --bench-props-off --trigger cue --terminal pursuit
        --mavsdk-url serial:///dev/ttyAMA10:921600
        --mavsdk-backup-url udpout://127.0.0.1:14550
        --rc-channel 8 --cue-allow-unsynced --board-failsafes keep
        --engage-max-s 15   (no --source: the detector was the synthetic one)
```

## What the two runs show

| | Run 1 (`run1.log`, 20:00:04 UTC) | Run 2 (`run2.log`, 20:02:38 UTC) |
|---|---|---|
| Length | 122 s, 2358 control ticks (19.3 a second) | 235 s, 4524 ticks |
| Target position reports received | 246, none malformed (2.0 a second) | 543, none malformed |
| OFFBOARD entered while disarmed | yes, 0.8 s after start | yes, 0.5 s after start |
| Worst gap between control ticks | 0.06 s (watchdog limit 0.5 s) | 0.06 s |
| Board limits read | all | all (two parameter reads timed out on the cable and were retried) |
| Enable switch, RC channel 8 | LOW throughout (993) | HIGH at start (1994), which the program refuses; lowered at 190 s; never raised again |
| GO | never: "the enable switch is LOW" | never: same |
| How it ended | the 120 s time limit of that run | at 232 s the pilot's mode switch took the aircraft to Position mode, about 3 s later it was ARMED from the transmitter, and the program stopped its commands at once ("vehicle reports armed=True") |

Laptop to Pi over the Starlink router during the session
(`runs/two_aircraft_link_test_20261126/wifi_ping_laptop_to_pi.txt`): 2651 of
2688 pings answered (1.4 % lost), 7.2 ms average, 131 ms worst, over 22.5 minutes.

## First time shown on the hardware

- The control-link route for the target's position works end to end: handset USB
  to laptop to Wi-Fi to the Pi, at the 2 reports a second the route allows.
- The Pi reads position, velocity, attitude, radio channels and UTC from the
  Pixhawk on both links outdoors (the indoor probe that morning had no position
  or radio).
- The pilot's mode switch takes the aircraft out of OFFBOARD (run 2, 232 s).
- The rehearsal's "armed" interlock works: it stopped the command stream.

## Not shown

- **GO and everything after it.** No estimate was ever latched, so the fit on
  real reports, the placing of the target against the interceptor's own GPS, the
  measured link delay and the ENGAGE commands have still never run on the
  hardware. The program prints only the first thing holding GO back, and that
  was the switch, so whether the fit and clock checks would have passed is not
  known.
- The camera. No `--source`, so no real frame went through the loop.

## Faults found

1. **Run 1 never exited.** It printed its report at 122 s and then sat in
   parameter retries, keeping the serial cable open. Run 2 was started on the
   same cable two minutes later, which is why `run2.log` is full of "primary
   silent" lines: two programs were reading one port. The run-1 processes
   (Pi PIDs 1603, 1605, 1632) were still alive at 20:10 UTC. Cause not looked
   into.
2. **The armed interlock reads a once-a-second flag.** With props on, an arming
   by hand while the program is in OFFBOARD could leave up to about a second of
   program commands on an armed aircraft. In run 2 the aircraft was already in
   Position mode when it was armed, so nothing was commanded. This mode was
   written for props off.

## Offline check made afterwards: a 2 mph target

`slow_cue_check.py` (this folder) pushes a made-up 0.9 m/s track through the
control-link frame format and the Pi's receiver, 200 trials each, with an
ASSUMED position error (0.3 m, slowly wandering) and timing jitter. The receiver
accepted every one: fitted velocity error 0.13 m/s median, 0.21 m/s at the 90th
percentile; a hovering target is also accepted (fitted speed 0.02 m/s). The
noise is a guess, not a measurement of this GPS.

## Runs 3 and 4: GO seen, and a 101 m height error found and fixed

Same command, the builder at the transmitter, kill switch on, Stabilized mode.
On his handset the GO switch reads LOW (no-go) in its raised position.

| | Run 3 (`run3.log`, 20:15:39 UTC) | Run 4 (`run4.log`, 20:22:53 UTC, with the fix) |
|---|---|---|
| GO | fired at 24.6 s, from 12 reports over 5.5 s | fired at 63.4 s, from 11 reports over 5.0 s |
| Target latched at | 7.1 m away (north +6.8, east +2.0), **100.9 m above**, 0.03 m/s | 4.8 m away (north +4.5, east +1.7), 2.4 m above, 0.05 m/s |
| ENGAGE | 15 s, then break-off, then SAFE | 15 s (283 ticks, every one in OFFBOARD), break-off, SAFE |
| Commands in ENGAGE | first seconds a 9 m/s climb | climb or descent within 2.2 m/s; horizontal up to the 16 m/s cap |
| Program's own report | not printed: stopped by hand while holding SAFE | **PASS**, all seven checks |

**The height error.** Read from the aircraft between the runs (read-only): the GPS
receiver gave 1641.0 m above sea level, the Pixhawk's position estimate 1543.9 m,
the target's reports 1640 to 1643 m. The board's height reference has been the
barometer since 25 November, so the Pixhawk reports pressure height, 97 m low
that day. The program subtracted that from the target's GPS height. In flight it
would have climbed until the 30 m geofence held it.

**The fix** (`flight/deploy/real_flight.py`, `cue_own_lla`): the target is placed
against the GPS receiver's height; with none under 2 s old, GO waits.
`flight/tests/test_cue_own_height.py`, 4 tests; `flight/tests` 605 passed on the
dev laptop. Run 4 is the fix seen on the hardware.

**What runs 3 and 4 also show**

- Both aircraft sat still, yet the latched horizontal position moved 2.6 m
  between the runs and the height difference read 2.4 m. That is the size of the
  disagreement between two GPS receivers, and it is the error a flight on the
  belief alone starts with. The true separation was not measured.
- With the aircraft unable to move, the chase command grew to its 16 m/s cap
  against a target 5 m away. In flight the gap would close first, but nothing
  limits the chase to a slow speed for a slow course; there is no such option.
- The control loop paused 0.23 s at the GO tick (the fit), against 0.06 s
  otherwise and a 0.5 s watchdog limit.
- The program again did not exit after its report (as run 1) and was stopped by
  hand. Cause still not looked into.
- Not exercised: the camera (synthetic detector), a measured link delay (this
  route has no timestamps), arming and takeoff.
