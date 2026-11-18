# Flight program safety fixes, checked against real PX4 (2026-11-18)

**What this is.** The live driver in `flight/deploy/real_flight.py` was changed on
2026-11-18 to close six gaps from that day's interface audit. Its unit tests run on a
fake vehicle (`flight/tests/test_real_flight_driver.py`). This folder is the second
check: the same program against a **real PX4 flight stack and a real MAVSDK server**,
on the dev laptop, with PX4's built-in physics (SIH) instead of Gazebo.

**What this is not.** No hardware was involved. The Pixhawk, the Pi, the radio and the
camera were not touched. PX4 here is the simulator build with no transmitter, so its
offboard-loss action is Return; the real board is set to Position mode
(`runs/fc_readonly_20261118/`).

## Results (all on the dev laptop, PX4 SIH instance 1, UDP 14541)

| Check | Result | How |
|---|---|---|
| SITL smoke gate, its own arguments, unchanged | PASS, exit 0: 283 setpoints, 148 of 149 engage ticks camera-guided, oldest own-state sample 0.04 s, watchdog clean (worst tick gap 0.069 s) | `sitl_smoke_on_sih.log` |
| Program frozen mid-engagement (SIGSTOP), watchdog 0.5 s | MAVSDK server dead after 0.47 s; PX4 left OFFBOARD at +2.32 s | `sih_hang.py stop 0.5` |
| Same, watchdog off (the behaviour before the fix) | server still alive and PX4 still in OFFBOARD, armed, 8 s later | `sih_hang.py stop 0` |
| Program killed mid-engagement (SIGKILL), watchdog 0.5 s | server dead after 0.02 s; PX4 left OFFBOARD at +1.14 s | `sih_hang.py kill 0.5` |
| Same, watchdog off | server still alive and PX4 still in OFFBOARD, armed, 8 s later | `sih_hang.py kill 0` |
| Aircraft taken out of OFFBOARD by a second client mid-engagement | program printed "sends NOTHING", sent no offboard stop and no land; PX4 was still in the mode the other client chose 3 s after the program exited | `sih_hang.py pilot 0.5` |
| `--bench-props-off`, default battery limit | REFUSED: battery reads 16.2 V (the simulator's battery) | command below |
| `--bench-props-off --bench-max-batt-v 30` | PASS: OFFBOARD entered while disarmed, 110 of 110 engage ticks in OFFBOARD, all nine limits read, never armed | command below |
| `--fly` at the stack's stock 12 m/s limit | REFUSED, did not arm | `sih_fly.py 20 45` |
| `--fly --override-platform-limits`, transmitter played by a second client | waited 10 s without arming; armed in the same 0.1 s as the permit switch; GO read 0.1 s after the GO switch; engaged, landed, disarmed | `sih_fly.py 12 40 --override-platform-limits --alt-tol-m 1.5`, `fly_on_sih.log` |

One observation that is not about these fixes: in the first `--fly` run the standby
height wandered between 6.2 and 8.05 m, and a GO flipped at a moment it read 6.30 m was
refused by the arm gate (tolerance 0.3 m) and ended the flight in SAFE. That is the
gate working as written. A hand-timed GO on an aircraft with soft height hold can hit it.

## Run it again

```
python runs/real_flight_driver_sih_20261118/sih_ctl.py restart      # start PX4 SIH, no Gazebo
python runs/real_flight_driver_sih_20261118/sih_hang.py stop 0.5    # or: kill 0.5 | stop 0 | pilot 0.5
python runs/real_flight_driver_sih_20261118/sih_ctl.py restart
python -m flight.deploy.real_flight --bench-props-off --mavsdk-url udpin://0.0.0.0:14541 \
    --trigger timer --go-after-s 5 --engage-max-s 6 --smoke-duration 40 --bench-max-batt-v 30
python runs/real_flight_driver_sih_20261118/sih_fly.py 12 40 --override-platform-limits --alt-tol-m 1.5
python runs/real_flight_driver_sih_20261118/sih_ctl.py stop
python runs/real_flight_driver_sih_20261118/mutation_check.py       # 25 fixes switched off in turn
```

Restart PX4 between scenarios: several leave the simulated aircraft flying home.
`sih_ctl.py` expects PX4 at `~/PX4-Autopilot/build/px4_sitl_default` (set `PX4_BUILD`
otherwise). Use the project's Python (`.venv/bin/python`).

## What still needs the real Pi and Pixhawk, props off

1. `--bench-props-off` over the real serial link, GPS with a fix (OFFBOARD needs a
   position estimate), no flight battery, transmitter on, `--trigger rc`: flip GO and
   read the report. It prints the RC_CHANNELS rate on TELEM2, which has never been measured.
2. During that run, in ENGAGE: `kill -STOP <pid of the python program>` and watch the
   flight mode in QGroundControl. It must leave OFFBOARD within about 1.5 s (0.5 s
   watchdog plus the board's 1.0 s `COM_OF_LOSS_T`) and go to Position mode.
3. The same with `kill -9`.
4. Unplug the Pixhawk's USB cable during a run and watch whether the flight recorder
   takes the serial port (audit gap G8, not addressed by this change).
