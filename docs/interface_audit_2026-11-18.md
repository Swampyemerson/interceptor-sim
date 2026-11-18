# Guidance / Pi / Pixhawk / cue-link interface audit (2026-11-18, read-only, from the code)

Scope: how `flight/deploy/real_flight.py` talks to the Pixhawk in PX4 OFFBOARD mode, and how
the ~3 s GPS cue reaches it in the field. Nothing was run against hardware. "Verified" below
means "read in the code or in a recorded bench/SITL result"; "assumed" means nobody has shown it.

Short answer to the builder's two questions:

1. **The automatic-mode interface is not completely fleshed out.** The state machine and its
   failsafes are well built and tested offline and in SITL, and the Pi-to-Pixhawk serial link is
   bench-proven with the older `seeker_loop`. But `real_flight.py` has no real-flight mode (the only
   live path is the SITL smoke path, with SITL defaults), it has never been run against the real
   Pixhawk, and several PX4 settings on the real board contradict what the guidance assumes.
2. **The cue link does not work in the field as coded.** The cue file is read once when the
   program starts; the aircraft then arms, takes off and waits for GO, and the belief is used as
   if it were the target's position at GO. Nothing advances it to the GO instant and no program
   exists to get the file from the target to the Pi.

---

## Status of each gap after the same day's fixes (added 2026-11-18, evening)

The text below the table is the audit as written that morning and is left as found. "In
code" means merged and tested on the laptop against a fake vehicle and a simulated PX4
(`runs/real_flight_driver_sih_20261118/README.md`). **Nothing has been run on the real Pi
and Pixhawk yet.** Decision record: `docs/decisions.md` ADR-0139.

| Gap | Now |
|---|---|
| G1 no real-flight mode | **In code.** `--fly` with an RC arm permit; `--sitl-smoke` refuses a serial link. |
| G2 hung program keeps flying | **In code.** Out-of-process watchdog (`flight/deploy/loop_supervisor.py`). Simulated PX4: frozen program, server dead at 0.47 s, out of OFFBOARD 2.3 s later. Bench proof owed. |
| G3 cue never advanced to GO | **In code for `--trigger cue`** (live link, latched at GO, delay removed). The `--cue-json` file path is unchanged and still stale for a moving target. |
| G4 board limits unchecked | **In code** (limits read before arming; mismatches refuse). The board's speed limit is now 33 m/s, not 12; the geofence is still 457 m. |
| G5 radio loss aborts the intercept | **Open.** Builder's call, after the watchdog is bench-proven. |
| G6 landing commanded under a pilot | **In code.** Nothing is sent while the aircraft is in a pilot mode. |
| G7 frozen own-state unnoticed | **In code.** Each stream is stamped; older than 1.0 s goes SAFE (FAILSAFE 9). |
| G8 recorder can grab the guidance port | **Open.** |
| G9 cue link not built end to end | **Partly.** Pi side built and bench-tested over Wi-Fi; own position now comes from the vehicle's GPS on the cue path. The leg off the target is researched (`docs/cue_over_control_link.md`), its reader written, untried. |
| G10 GO switch never read from the real board | **Open.** Also true of the new arm-permit channel. |
| G11 no props-off rehearsal | **In code** (`--bench-props-off`, cannot arm). **Not yet run**: the builder said the props are on. |
| G12 smaller items | The approach dash now has one preset under `--fly` (brake 3 with vertical arrival-sync); the engage window is unlimited under `--fly`; the pre-arm summary names the one parameter write. Unchanged: standby is a zero-velocity hold, the battery failsafe can pull the aircraft out of a hard dash. |

---

## Ranked gaps (most likely to cause a miss or unsafe behaviour first)

### G1. The only way to fly the real aircraft is `--sitl-smoke`, with simulator defaults and no guards
- **Code.** `main()` accepts only `--self-test | --audit | --dry-run | --sitl-smoke | --desk`
  (`real_flight.py:3830`). The live driver is `run_mavsdk_mission(..., smoke=True)` (`:3946`).
  Its defaults are SITL defaults:
  - `--trigger` defaults to `gate-ready` (`:3600`; class at `:1903`), which fires GO **by
    itself** once the arm gate has been good for 4 ticks. A real run that forgets `--trigger rc`
    takes off and dashes with no human GO.
  - With no `--source`, the detector is the synthetic `SmokeSeeker` (`:3936-3938`,
    `seeker_loop.py:921`): a fake target 8 m ahead that the terminal will steer on.
  - Belief defaults `--target-start 0,16.7 --target-vel 9,0` (`:3466-3468`).
  - The program arms and takes off on its own as soon as GPS health is good
    (`:2615-2617`): launching it over SSH *is* the arm command. No confirmation, no RC permit.
  - `--smoke-duration` 90 s is an outer bound on the real path (`:2664`) and is shorter than
    `mission_max_s` 120 s; `--smoke-acquire-after-s` 2.0 blinds the detector for 2 s after GO (`:2675-2676`).
- **Field needs.** A dedicated flight mode that cannot be started with a synthetic trigger or
  detector, and a deliberate human step before props spin.
- **Status.** Verified in code. No test covers "real link + SITL defaults".
- **Smallest fix.** Add `--fly`: require `--trigger rc`, `--source`, an explicit belief (cue or
  typed), and refuse `gate-ready`/`timer`/synthetic detector whenever `--mavsdk-url` is `serial://`.
  Gate arming on an RC permit (aux switch seen LOW then HIGH) or a typed confirmation.
- **Proof.** Unit test on argument validation (serial URL + default trigger must exit non-zero);
  SITL gate unchanged.

### G2. If the Pi program hangs or is killed, the Pixhawk keeps flying the last velocity command
- **Code.** MAVSDK's server process re-sends the last setpoint on its own timer; the repo says so
  itself (`scripts/check_deploy_bench.sh:41-44`, `seeker_loop.py:1434`, `:1674`). The server is a
  child started by `mavsdk/system.py:_start_mavsdk_server` and is only killed by an `atexit`
  hook, which does not run on SIGKILL, an out-of-memory kill or a crash. The control loop has no
  watchdog on its own tick interval (`real_flight.py:2671-2779`); a blocked `detector.detect()`,
  `print()` or gRPC call stalls it silently.
- **Consequence.** PX4's offboard-loss failsafe (`COM_OF_LOSS_T` 1.0 s) never sees a stalled or
  dead Python: the stream is still healthy. During an approach dash the aircraft continues at
  dash speed in a straight line until the geofence (457 m, Hold), the pilot, or the battery
  stops it. On a clean exception the `finally` block does stop offboard and land (`:2780-2811`).
- **Status.** Verified by reading; never tested.
- **Smallest fix.** A watchdog thread outside the asyncio loop: if no control tick for 0.5 s after
  OFFBOARD starts, kill the server process (`drone._server_process.kill()`), so the stream stops
  and PX4 falls to Position mode (`COM_OBL_RC_ACT=0`) within 1 s. Start the program from a wrapper
  that kills `mavsdk_server` when the Python process dies, and run it detached (tmux or
  `systemd-run`), with output to a file, so a dropped Wi-Fi SSH session cannot touch it.
- **Proof (props off, disarmed, same setup as the offboard bench gate).** Enter OFFBOARD, send
  `SIGSTOP` to the Python process, watch the flight mode in QGroundControl: today it should stay
  OFFBOARD indefinitely; with the watchdog it must leave within about 1.5 s. Repeat with `kill -9`
  and check `pgrep mavsdk_server`.

### G3. The cue is read once at program start and never advanced to GO; the sim assumes it is exact at GO
- **Code.** `build_config` calls `load_cue_for_real_flight(path, own_lat, own_lon, own_alt)`
  with no `now_t` and no `trigger_t` (`real_flight.py:3158-3159`), before any link is opened.
  `fit_cue` returns the target state at the **last record's time** (`cue_relay.py:268-300`).
  `build_terminal` hands that position to the pursuit terminal as `belief_r0_ned`
  (`:3317-3324`), which the terminal defines as "the target's position relative to the vehicle
  **at the instant ENGAGE begins**" (`pursuit_terminal.py:41-44`) and only starts propagating at
  the first post-GO tick (`:757-758`). Nothing in STANDBY re-reads or extrapolates
  (`real_flight.py:1335-1404`). `CueSolution.extrapolate_to` exists (`cue_relay.py:196`) and is
  never called by the flight program.
- **Clocks.** Cue timestamps are the ground laptop's `time.monotonic()` at receive
  (`cue_relay.py:650-682`); the Pi uses its own `time.monotonic()`; PX4 has its boot time. There
  is no shared time base, which is why the staleness check was deliberately left off
  (`real_flight.py:3151-3156`). That comment says the ground relay enforces staleness instead;
  no ground relay exists (see G9).
- **What the sims assume.** isim freezes the target at its start point until GO and releases it
  at GO (`isim/scenario.py:607-630`, `:698-701`); belief error is 0.5 m of start jitter, up to
  0.05 s of GO jitter, 10 % speed and 2° heading. The Gazebo 80 mph takes injected 3 m and 1.5°
  **at GO** (`logs/cinema_lr/takes.txt`). Neither models seconds of delay or a turn before GO.
- **The real timeline (my estimate, not measured).** After the cue's last GPS sample: write and
  copy the file, start Python on the Pi, open the camera, MAVSDK connect, health wait, arm,
  PX4 take-off (3 s ramp + 1.5 m/s climb), reach 80 % of standby height, climb the rest, 2 s
  settle, yaw to aim, then the operator's GO. About 25 s if scripted, 40 to 90 s by hand.
- **Belief error at GO.**

  | Case | 2 m/s target | 9 m/s | 12 m/s |
  |---|---|---|---|
  | As coded (no extrapolation), T = 25 s | 50 m | 225 m | 300 m |
  | As coded, T = 40 s | 80 m | 360 m | 480 m |
  | If extrapolated, straight target, best measured velocity error 0.13 m/s | 3 to 5 m | 3 to 5 m | 3 to 5 m |
  | If extrapolated, plan's worst velocity error 1.6 m/s | 40 to 64 m | 40 to 64 m | 40 to 64 m |
  | If extrapolated, target turned 90° 5 s before GO | 14 m | 64 m | 85 m |

  Tolerance for comparison: the pursuit A/B says keep the cue inside about 10° at the 16.7 m
  geometry, about 3 m; the plan itself says keep the gap under about 2 s
  (`docs/cue_relay_plan.md` §6-§8). A target on 40 to 100 m legs at 9 m/s turns every 4 to 11 s.
- **Conclusion.** As coded the cue is usable only for a hovering target (ladder rung L3). For a
  moving target it is wrong by hundreds of metres.
- **Smallest fix that flies soon (no new link).** Make the field procedure match what the sim
  assumes: the target waits at the start of a straight leg, the belief is that start point plus
  the programmed leg velocity (cue for position, mission for velocity), and the target is
  released on the same call as GO. Write that into `docs/kill_day_protocol.md`; it is not there now.
- **Proper fix.** Keep the cue stream running into the Pi through STANDBY, stamp records with GPS
  UTC (both autopilots publish `SYSTEM_TIME`), keep the freshest fit, and at the GO edge call
  `extrapolate_to(GO)` and latch once; refuse GO if the freshest record is older than about 2 s.
  Extend the latch-once audit to the belief.
- **Proof.** Unit test: a constant-velocity track with a 30 s gap must give belief = truth at GO.
  isim arm: belief stale by T (0, 2, 10, 30 s), with and without a turn, with and without
  extrapolation. Bench: the open `docs/next.md` item 4, with the target's GPS feeding the Pi on
  the desk and the latched belief printed at a switch flip.

### G4. The real board's limits are below what the guidance commands, and nothing checks them
- **Real board (dump `configs/px4_6cmini/dumps/20261113_bench_new_pi.tsv`).**
  `MPC_XY_VEL_MAX` 12 m/s, `MPC_TILTMAX_AIR` 45°, `MPC_Z_VEL_MAX_DN` 1.5, `GF_ACTION` 2 (Hold)
  with `GF_MAX_HOR_DIST` 457 m and `GF_MAX_VER_DIST` 30.5 m.
- **Guidance.** Pursuit's own cap is 16 m/s (`pursuit_terminal.py:126`); the approach dash
  defaults to 20 and has flown 36 in Gazebo. PX4 clips an offboard horizontal velocity setpoint
  to `MPC_XY_VEL_MAX` (the repo's own Gazebo notes treat it as the ceiling; consistent with the
  PX4 position controller). `real_flight.py` never reads a PX4 parameter.
- **Consequences.**
  - Against a 9 m/s target the closing speed is 3 m/s instead of the 7 the chase was tuned
    with; a 12 m/s target cannot be caught at all.
  - The launch-time window check uses the commanded cap, not the platform's
    (`real_flight.py:3366-3378`): a 500 m cue at `--pursuit-dash-vmax 36` computes 26 s and prints
    nothing, while the real aircraft at 12 m/s needs over 42 s and times out at
    `engage_max_s` 30 s mid-transit.
  - A 500 m engagement crosses the 457 m geofence: PX4 goes to Hold, the program sees OFFBOARD
    lost and goes SAFE.
  - The head's isim result (the guidance tolerates a vehicle that cannot reach the commanded
    speed) covers the mis-aim risk, not these three.
- **Smallest fix.** Read the limits once before arming (read-only) and print them next to the
  guidance caps; refuse to fly if the commanded cap exceeds the platform cap unless overridden;
  use `min(commanded, platform)` in the transit check; check the believed rendezvous against the
  geofence. The envelope itself stays the builder's decision (`dash-envelope-on-real-fc`).
- **Proof.** Unit test with a fake parameter reader. isim cell with the caps set to 12 m/s
  against 9 and 12 m/s targets.

### G5. Killing the transmitter after GO will abort the intercept on this board
- **Real board.** `COM_RCL_EXCEPT` 0, `NAV_RCL_ACT` 2 (Return), `COM_RC_LOSS_T` 0.5 s. RC loss in
  OFFBOARD triggers PX4's RC-loss failsafe; the program then sees OFFBOARD lost and goes SAFE.
- **Field needs.** The link-denied arm (`docs/kill_day_protocol.md` §5.4) expects the intercept to
  finish with the transmitter off.
- **Status.** Verified from the dump; `configs/px4_6cmini/bench.params:310-317` already lists it
  as "decide later". Not decided.
- **Fix.** `COM_RCL_EXCEPT` bit 2 (offboard) for that test only. It removes PX4's RC-loss
  protection while the kill switch is also gone, so it is the builder's call, and G2 and the
  geofence must be right first.

### G6. When the pilot takes over, the program commands LAND about 2.5 s later
- **Code.** Pilot flips the mode switch: PX4 leaves OFFBOARD, FAILSAFE 7 goes SAFE after 0.5 s
  (`:1245-1258`), SAFE terminates after `safe_hold_s` 2 s, and the `finally` block
  unconditionally sends `action.land()` or return-to-launch (`:2781-2811`). Same in STANDBY at
  the 60 s standby timeout.
- **Field needs.** The protocol tells the pilot "it is safe to take the aircraft back manually"
  (`docs/kill_day_protocol.md` §5.3). An automatic mode change under the pilot's hands is not that.
- **Status.** Verified in code; no test.
- **Fix.** In teardown, send land/RTL only if the vehicle is still in OFFBOARD or Hold; if the
  flight mode is a pilot mode, send nothing.
- **Proof.** SITL: switch to Position mode from a second MAVLink client mid-ENGAGE and assert
  the mode is still Position 10 s later.

### G7. The flight program does not notice frozen attitude, velocity or height data
- **Code.** `VehicleObs.own_state()` builds `OwnState` without `age_s` (`:577-581`), so the
  0.25 s staleness check in `own_state_status` (`seeker_loop.py:264`) is skipped. `t_quat` is
  stamped (`:2523`) and never used; position and velocity are not stamped at all. The older
  `seeker_loop` does pass the age (`seeker_loop.py:1621`), and the bench gate's "oldest own-state
  0.02 s" figure comes from it, not from `real_flight`.
- **Consequence.** A single dead stream leaves the guidance flying on old numbers. A frozen
  height makes the STANDBY height loop command a constant climb or descent. A total loss of the
  link is caught, but only by the RC age (1 s, STANDBY) or the flight-mode age (3 s, after GO);
  3 s at dash speed is up to 100 m.
- **Fix.** Stamp every stream, pass the oldest age into `OwnState`, and go SAFE when it exceeds
  the bound. **Proof.** Unit test with one stream frozen; rerun the bench gate through `real_flight`.

### G8. If the Pixhawk's USB cable drops in flight, the recorder may grab the guidance serial port
- **Code.** The recorder picks USB when `/dev/pixhawk-usb` exists, otherwise `/dev/ttyAMA10`
  (`scripts/bench/ulog_stream_recorder.py:256-262`, `:752-756`); a serial error is not caught, so
  the service exits and systemd restarts it 2 s later, when it re-picks. `/dev/ttyAMA10` is the
  port the flight program uses. The recorder sets an exclusive flag only on a port it opened
  first (`:448-455`).
- **Could not determine.** Whether `mavsdk_server` locks the port. If it does not, both programs
  read the same UART and the guidance loses telemetry mid-flight.
- **Fix.** For flight, set `FALLBACK_PORT=` (empty) in `/etc/default/flight-recorder`, or make the
  recorder refuse to change ports while armed.
- **Proof (props off).** Offboard bench gate running on `/dev/ttyAMA10`, unplug the USB cable,
  watch `journalctl -u flight-recorder` and the gate's own-state age.
- **Solid part.** With USB in, the recorder leaves the serial link alone (100 Hz attitude, gate
  passed alongside, `docs/fc_logging_without_sd.md`). With no USB, a second program gets
  "busy" and the flight program fails closed at connect.

### G9. The cue link itself is not built end to end
- Leg 1 (target GPS to the ground): only a wired USB control run exists
  (`runs/cue_leg1/summary_20260922T192018Z.json`, transport pass, no fix). The ELRS-backpack and
  dedicated-radio paths are untested.
- Leg 2 (ground to Pi): no script. `scripts/bench/cue_leg1_gate.py` writes a cue file as a
  by-product; nothing ships it, checks its age, or starts the flight program.
- `--own-lat/--own-lon/--own-alt-m-msl` must be typed by hand (`:3147-3150`).
- Vertical: the plan's own open risk stands. One sigma of GPS height error (2.84 m measured) sits
  at the +3 m cliff of the altitude sweep, and the standby height is now derived from the cue.

### G10. The GO switch has not been read by the Pi from the real Pixhawk
- `RcChannelTrigger` reads `RC_CHANNELS` through MAVSDK's `mavlink_direct`
  (`:2559-2571`). Channel 7 and the 1700/1300 thresholds are marked TODO-BUILDER
  (`:1761-1771`), and the docstring still asks for a bench check that PX4 streams the message
  on TELEM2 at a useful rate (`:1806-1809`). `link_ok` is message age only; it ignores RSSI and
  the receiver's failsafe flag. `configs/px4_6cmini/rc_check.py` notes the stream stops when the
  transmitter is off, over USB.

### G11. There is no props-off rehearsal of the real program on the real Pixhawk
- The protocol's L0 rung ("start the state machine, watch STANDBY, flip GO, kill the TX") cannot
  be run: the live path always arms and takes off, and the arm gate needs the standby height.
  `--desk` has no vehicle link or RC; `check_deploy_bench.sh` runs `seeker_loop`, not `real_flight`.
- **Fix.** A `--bench-props-off` mode: require disarmed, skip arm and take-off, stub the height
  gate, and exercise RC GO edge, ENGAGE setpoints, transmitter kill and the failsafes against
  the real board. It is also the test bed for G2, G6, G7, G8 and G10.

### G12. Smaller items
- STANDBY and the "hover" miss behaviour are zero-velocity holds, not position holds
  (`:1127-1134`, `:1737-1741`): the aircraft drifts with wind for up to 60 s while the belief
  stays tied to the launch point.
- The Gazebo 80 mph takes flew `--pursuit-brake` **and** `--pursuit-approach-dash`
  (`scripts/cinema/cinema_run.py:335`). `--pursuit-approach-dash` alone leaves vertical arrival
  sync off and the brake at the 4.0 default (`:3349-3355`). There is no single preset.
- `COM_LOW_BAT_ACT` 3: a battery sagging under a hard dash can trigger PX4's return, which the
  program sees as OFFBOARD lost. Battery calibration on this board is still pending.
- `action.set_takeoff_altitude` writes `MIS_TAKEOFF_ALT` on the board every flight (`:2615`).
- A take-off that never reaches 80 % height just falls through after 60 s (`:2618-2625`); the
  arm gate then refuses GO. Fail-closed, but silent until the standby timeout.

---

## What is solid
- **State machine and failsafes.** STANDBY, ENGAGE, BREAKOFF, SAFE, the arm gate, the rising-edge
  GO rules (never a switch found high at boot), link-denied only before GO, OFFBOARD loss after
  GO, both clock-base checks: unit-tested (`flight/tests/test_real_flight.py`, `--self-test`,
  `--audit`) and passing in SITL end to end on the pursuit default (`check_real_flight_sitl.sh`,
  ADR-0133).
- **OFFBOARD entry and keep-alive.** One primed setpoint, `offboard.start()`, then a setpoint
  every tick at about 20 Hz; the "in OFFBOARD" fact comes from the flight-mode telemetry, not
  from the start call (`:2072-2087`).
- **Serial link.** Pi `/dev/ttyAMA10` to TELEM2 at 921600 (`SER_TEL2_BAUD` and `MAV_1_MODE`
  onboard confirmed in the dump); OFFBOARD accepted while disarmed; 300 of 300 setpoints, largest
  gap 0.05 s, own-state age 0.02 s, with the recorder logging over USB (bench, `seeker_loop`).
- **Clean exits.** Any exception lands the aircraft; SAFE is absorbing.
- **Cue fitting.** Fail-closed quality gate, straight and turning models, height and climb rate
  consumed, tested against a real GPS log; `--cue-json` reaches the pursuit belief and works
  together with `--pursuit-approach-dash` (both exercised in `--dry-run` tests).
- **Under-delivery.** The approach dash steers on measured own velocity, so a vehicle slower than
  commanded arrives late rather than mis-aimed (head's isim drag screen).
- **PX4 side.** Kill switch on channel 5, mode switch on 6, RC loss returns home, geofence
  holds, offboard loss falls to Position mode, sticks do not override OFFBOARD.

## Test coverage by behaviour
| Behaviour | Covered by | Not covered |
|---|---|---|
| State machine, failsafes 1-8, latch-once | unit tests, self-test, audit | |
| OFFBOARD entry, setpoint stream, landing | `check_real_flight_sitl.sh` | on the real Pixhawk with `real_flight` |
| Serial link, OFFBOARD while disarmed | `check_deploy_bench.sh` (`seeker_loop`) | `real_flight`, armed |
| RC GO edge | unit tests with fake frames | real `RC_CHANNELS` over TELEM2 |
| Cue fit and `--cue-json` parsing | `flight/tests/test_cue_relay.py` | cue age at GO, a moving target, transport, end to end |
| Approach dash logic and CLI mapping | `flight/tests/test_pursuit_terminal.py`, isim, Gazebo takes | with a real cue; at the real board's limits |
| Program hang or kill | | everything (G2) |
| Pilot takeover then teardown | | everything (G6) |
| Stale own-state in `real_flight` | | everything (G7) |
| Recorder and guidance sharing the port | bench, USB present | USB dropout (G8) |
| Real-link run with SITL defaults | | everything (G1) |

## Could not determine
- Whether `mavsdk_server` locks its serial port (G8).
- The real time from program start to GO (G3 uses my estimate).
- `RC_CHANNELS` rate on TELEM2 and what the stream does on this receiver when the transmitter
  is off (G10).
- Whether PX4 on this firmware applies `MPC_ACC_HOR_MAX` or jerk limits to offboard velocity
  setpoints; the clip at `MPC_XY_VEL_MAX` is from the repo's Gazebo notes and my reading of PX4,
  not a test on the real board.
- Whether a dropped SSH session kills `mavsdk_server` along with Python.
- The Pi's real control-loop rate with camera decode in the loop.
- Whether the ELRS backpack can carry the target's MAVLink (leg 1).
