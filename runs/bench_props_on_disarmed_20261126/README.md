# Target-to-interceptor loop, first runs on the real hardware, 26 November 2026

Two runs of the flight program's rehearsal mode (`--bench-props-off`) on the real
Pi and Pixhawk at the flying site, with the target powered and sending its
position. The interceptor had its flight battery in and its props ON, disarmed
(the mode cannot arm; the builder was at the aircraft). The session that started
the runs was stopped by the model's safety filter before it reported. This note
was written afterwards from the files in this folder, pulled from the Pi.

Nothing here flew under program control. GO never fired.

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
