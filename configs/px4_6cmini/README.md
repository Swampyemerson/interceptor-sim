# 6C Mini bench pack — props-off Pi ↔ Pixhawk OFFBOARD bring-up

Everything needed to bring up the interceptor's **flight-computer pair** on the
desk — Raspberry Pi 5 talking MAVLink/OFFBOARD to a **Pixhawk 6C Mini** — with
**no airframe, no ESC, no motors** (nothing can spin). This retires the deploy
loop's one link the sim never exercised: `flight/deploy/seeker_loop.py` driving
**MAVSDK OFFBOARD over a real serial UART**.

- Canonical build steps: `docs/project_state.json` → `build_tab` → subsystem
  **`brain`** (steps `brn-01`…`brn-06`; `brn-05` is the gate this pack closes).
- Wiring/param sources: `docs/hardware_order_list.md` **§3** (compatibility) and
  **§B** (RC path). Constraints: `no-datalink`, `honesty-boundary`.

Files here:

| File | What it is |
|---|---|
| `bench.params` | QGC-loadable PX4 parameter file (heavily annotated; every non-default has a WHY + doc source). |
| `README.md` | This procedure. |
| `../../scripts/check_deploy_bench.sh` | The props-off, **no-arm** OFFBOARD check (and its `--check-only` self-test). |

---

## What this bench DOES and does NOT prove

**Proves (props-off, disarmed):** a live heartbeat over the real serial port,
own-state EKF streaming to the Pi, a ≥2 Hz setpoint stream, the **OFFBOARD mode
switch accepted while disarmed**, and pro-nav setpoints streaming over the wire —
the whole `flight/deploy/seeker_loop.py` chain end-to-end on real I/O.

**Deferred to the airframe (NOT here):** arming, motors actually responding to
setpoints, takeoff/land, ESC/DShot, battery monitoring, control-allocation
geometry, MC PID tuning. See the *WAITS FOR THE AIRFRAME* block in `bench.params`.

> **CORRECTED 2026-07-26 — you do NOT need a GPS fix for this bench.** This
> README used to say PX4 would refuse OFFBOARD without an EKF horizontal
> position, so run it outdoors. That is wrong for the **disarmed** case on PX4
> v1.16: a mode change is *always* allowed while disarmed
> ([`UserModeIntention.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/commander/UserModeIntention.cpp)
> — "Always allow mode change while disarmed"), and the vehicle code path
> (`seeker_loop.run_mavsdk(smoke=False)`) hard-requires only the EKF **attitude**
> stream, which needs no GPS (missing altitude is a loud warning, not a failure;
> the health/global-position wait lives in the SITL-only `smoke` branch).
> `bench.params` still sets `COM_ARM_WO_GPS=0` — that is an **arming** guard for
> later, not a precondition of this test. **Run brn-05 on the desk.**

---

## Procedure

### 0. Two-rail power (safety first)
Pi 5 on its **own** 27 W USB-C PSU; FC on **USB-C** (or the PM02 pack later).
**Never power the Pi from the FC rail** (two-rail rule, `brn-06`). The 3-wire
TELEM2 link below carries **no +5 V**.

### 1. Flash PX4 on the 6C Mini  (`brn-01`)
Plug the FC into the PC over **USB-C** (data cable). In **QGroundControl** →
*Vehicle Setup → Firmware* → select **PX4 Flight Stack (stable)**. QGC
auto-detects the board as **FMUv6C** and flashes the **`px4_fmu-v6c_default`**
target. (CLI equivalent: `make px4_fmu-v6c_default`.) Insert a **microSD**
(ULogs are the metrology) and confirm QGC shows it.

### 2. First connect + airframe  (over USB-C)
QGC connects automatically over USB. Then *Vehicle Setup → Airframe* → select any
generic multicopter → **Apply & reboot**. Either **Generic Quadcopter**
(`SYS_AUTOSTART = 4001`, *Quadrotor x*) or **Generic Quad + geometry**
(`SYS_AUTOSTART = 5001`, *Quadrotor +*) is fine — with no ESC and no motors the
only difference between them is motor geometry, and both light up the
multicopter control + OFFBOARD stack the plumbing test needs. The **real**
airframe ID / geometry / motors are set when the frame is built.
> **Do the airframe BEFORE loading `bench.params` — this ordering is real, and
> here is the actual mechanism** (verified 2026-07-26, stronger than the vague
> claim this README used to make). QGC's Airframe *Apply* writes `SYS_AUTOSTART`
> **and `SYS_AUTOCONFIG=1`**
> ([`AirframeComponentController::changeAutostart`](https://github.com/mavlink/qgroundcontrol/blob/master/src/AutoPilotPlugins/PX4/AirframeComponentController.cc)),
> and on the next boot PX4 runs
> `param reset_all SYS_AUTOSTART SYS_PARAM_VER RC* CAL_* COM_FLTMODE* LND_FLIGHT* TC_* COM_FLIGHT*`
> ([`rcS`](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/ROMFS/px4fmu_common/init.d/rcS)).
> Everything in `bench.params` except the `RC_*` line would be **wiped**. That is
> why `SYS_AUTOSTART` is **not** in `bench.params` — and why, **if you ever change
> the airframe later, you must re-load `bench.params` afterwards.**

### 3. Load `bench.params`
*Vehicle Setup → Parameters → Tools → Load from file* → pick
`configs/px4_6cmini/bench.params` → **reboot**. The file carries **9** lines;
all of them were re-verified against **PX4 v1.16** on 2026-07-26 (see the header
block inside the file).

**What QGC should show you.** Modern QGC previews a **diff** before applying and
*skips lines whose value already matches the vehicle*. Four of the nine already
equal the v1.16 defaults, so expect the diff to list roughly these **five**:

| Param | New value | Means |
|---|---|---|
| `MAV_1_CONFIG` | 102 | MAVLink instance 1 → **TELEM 2** |
| `MAV_1_FLOW_CTRL` | 0 | flow control **forced off** (3-wire link) |
| `MAV_1_FORWARD` | 1 | forward Pi↔GCS traffic (bench convenience) |
| `COM_ARM_WO_GPS` | 0 | deny arming when the GPS preflight check fails |
| `SDLOG_MODE` | 2 | log from boot until shutdown |

Silently skipped because they are already the v1.16 default: `MAV_1_MODE`=2,
`SER_TEL2_BAUD`=921600, `COM_KILL_DISARM`=5.0, `RC_MAP_KILL_SW`=0. If QGC
instead reports a param as **not present on the vehicle**, stop and tell the
session — that means a name is wrong for this firmware.

After the reboot, re-open Parameters and confirm the five above. Then
**calibrate sensors** (accel / gyro / mag / level) — sensors-only, bench-safe now.

### 4. Bind ELRS + Radio calibration (SBUS RC in)
Bind the RadioMaster **Pocket** TX to the interceptor's ELRS RX (SBUS output),
wire the RX to the 6C Mini's **dedicated RC IN** pad (not a UART — stock PX4 has
no CRSF driver, §B). QGC *Radio* calibration → then map the **kill switch** and
set `RC_MAP_KILL_SW` to the channel your switch actually uses. Flip it and
confirm QGC reads *Kill switch* active. ELRS is **kill/arm only** — it carries no
guidance (`no-datalink`).

> **`RC_MAP_KILL_SW` is now `0` (unassigned), changed 2026-07-26.** It used to
> carry a guessed `5`. A guessed mapping is not protection, it is the *illusion*
> of protection — and `5` was a bad guess specifically: on ELRS/SBUS, channel 5
> = AUX1 is the conventional **arm** channel, and PX4 reads kill as engaged
> whenever that channel sits above `RC_KILLSWITCH_TH` (v1.16 default **0.75**,
> polarity "true when channel > threshold"). That would have made your first arm
> gesture after binding look like *Kill switch engaged*. **Until you do this
> step, the vehicle has no mapped kill switch** — honest, and harmless while
> nothing can spin.

### 5. Wire the three Dupont jumpers: TELEM2 ↔ Pi  (`brn-03`)
Both sides are **3.3 V logic — no level shifter**. **TX/RX crossed.** Connect
**only these three** (leave TELEM2 pin-1 +5 V **unconnected**):

| 6C Mini **TELEM2** (JST-GH 6-pin) | → | Raspberry Pi 5 (40-pin header) |
|---|---|---|
| pin 2 **TX** | → | **GPIO15 / RXD**, header **pin 10** |
| pin 3 **RX** | → | **GPIO14 / TXD**, header **pin 8** |
| pin 6 **GND** | → | **GND**, header **pin 6** |
| pin 1 **+5 V** | ✗ | **do NOT connect** (two-rail rule; would fry the GPIO) |

TELEM2 baud = **921600** (`SER_TEL2_BAUD`, set by `bench.params`).

**Free the Pi's serial port** (so it is a data UART, not a login console): on the
Pi, `sudo raspi-config` → *Interface Options → Serial Port* → login shell over
serial **No**, serial hardware **Yes**; ensure `enable_uart=1` in
`/boot/firmware/config.txt`; reboot. The port is **`/dev/ttyAMA0`**.

### 6. Heartbeat from the Pi  (`brn-04`)
On the Pi, quick check that MAVLink is flowing (any of):
```bash
# raw bytes moving on the link:
stty -F /dev/ttyAMA0 921600 && timeout 3 cat /dev/ttyAMA0 | xxd | head
# or a MAVSDK heartbeat via the seeker venv (has mavsdk):
.venv-seeker/bin/python -c "import asyncio; from mavsdk import System
async def m():
    d=System(); await d.connect(system_address='serial:///dev/ttyAMA0:921600')
    async for s in d.core.connection_state():
        print('connected', s.is_connected); break
asyncio.run(m())"
```
`connected True` = the companion link is up.

### 7. Props-off OFFBOARD bring-up — the gate  (`brn-05`)
From the repo root **on the Pi** (no GPS fix needed — see the corrected note above):
```bash
scripts/check_deploy_bench.sh
```
It runs the **vehicle** OFFBOARD path (`seeker_loop.run_mavsdk(smoke=False)`) —
connect → own-state → prime setpoint → `offboard.start()` → stream pro-nav
setpoints from replayed frames → clean `offboard.stop()`. **No arm, no takeoff,
no land.** Override the link/source with env vars if needed, e.g.:
```bash
# USB fallback from the laptop (FC on USB-C shows as /dev/ttyACM0):
BENCH_DEVICE=/dev/ttyACM0 BENCH_BAUD=57600 scripts/check_deploy_bench.sh
# live Pi camera pointed at the AprilTag placard instead of replayed frames:
BENCH_SOURCE=picamera scripts/check_deploy_bench.sh
```

**Exercise the kill switch by hand** during the run and watch QGC / the ULog —
it must be honored and auto-disarm after `COM_KILL_DISARM` (5 s).

### Expected output (PASS)
```
[check_deploy_bench] REAL bench run (props-off, NO arm) against serial:///dev/ttyAMA0:921600
[mavsdk] connecting serial:///dev/ttyAMA0:921600 ...
[mavsdk] connected
[mavsdk] OFFBOARD active; streaming setpoints ...
[mavsdk] stream complete: <N> frames, <M> setpoints over <t>s wall, mean cadence ~20 Hz (sent over MAVLink)
[mavsdk] offboard stopped
[mavsdk] done rc=0
[check_deploy_bench] PASS (connect + own-state + OFFBOARD + setpoints, disarmed; see logs/deploy_seeker_bench_*.log)
```
Evidence: the run log under `logs/deploy_seeker_bench_*.log` **and** the FC's ULog
on the microSD (logged from boot, `SDLOG_MODE=2`).

**If it fails, in likelihood order (v1.16-corrected 2026-07-26):**

1. **Nothing on the wire at all** (`connected` never prints) → wrong device node,
   TX/RX not crossed, Pi login console still holding the port, or baud mismatch.
   Check the node first: on a Pi 5 the GPIO14/15 UART is `/dev/ttyAMA0`, and
   `ls -l /dev/serial0` should point at it — use whichever the symlink resolves to.
2. **Connects but nothing streams / stalls ~0.5 s at a time** → hardware flow
   control. This is why `bench.params` now sets `MAV_1_FLOW_CTRL=0`: the default
   (`2` = auto) opens TELEM2 **with** CTS/RTS enabled and only drops it after the
   TX buffer backs up with no successful write for >500 ms, and our 3-wire link
   leaves CTS floating. If you skipped that line, set it by hand and reboot.
3. **`own-state EKF ATTITUDE did not stream`** → the companion link is one-way
   (RX not landing on the FC) or the airframe was never applied, so no estimator
   is running.
4. `offboard.start()` rejected → **not** a GPS problem on this bench (see the
   corrected note at the top); look for the FC rejecting the mode in QGC's
   messages panel.

At the *end* of the run, PX4 will notice the setpoint stream stopped
(`COM_OF_LOSS_T` = 1 s) and fall back per `COM_OBL_RC_ACT` (Position mode).
While disarmed that is a no-op mode change — a benign QGC message, **not** a
failure of the gate.

---

## Desk self-test (no hardware)
Validates this pack's own plumbing + that every referenced file exists + that
`bench.params` parses — exits 0/1, touches no hardware:
```bash
scripts/check_deploy_bench.sh --check-only
```

---

## Sources

**Pinned to v1.16** — the firmware actually on the board. `main` docs can and do
differ; every semantic claim above was re-checked at v1.16 on 2026-07-26.

- [PX4 v1.16 Parameter Reference](https://docs.px4.io/v1.16/en/advanced_config/parameter_reference.html) — the enum values for **every** line in `bench.params` (`MAV_1_CONFIG` 102 = TELEM 2, `MAV_1_MODE` 2 = Onboard, `MAV_1_FLOW_CTRL` 0 = Force off, `SER_TEL2_BAUD` 921600, `RC_MAP_KILL_SW` 0 = Unassigned, `RC_KILLSWITCH_TH` 0.75, `COM_KILL_DISARM` 5.0 s default, `COM_ARM_WO_GPS` 0 = Deny arming, `SDLOG_MODE` 2 = from boot until shutdown).
- [PX4 v1.16 Serial Port Configuration](https://docs.px4.io/v1.16/en/peripherals/serial_configuration) — `MAV_1_CONFIG`, `SER_TEL2_BAUD`.
- [PX4 v1.16 MAVLink Peripherals (companion)](https://docs.px4.io/v1.16/en/peripherals/mavlink_peripherals) — recommended companion set: `MAV_1_CONFIG`=TELEM2, `MAV_1_MODE`=Onboard, `MAV_1_RATE`=0, `MAV_1_FORWARD`=Disabled, `SER_TEL2_BAUD`=921600.
- [PX4 v1.16 Raspberry Pi Companion](https://docs.px4.io/v1.16/en/companion_computer/pixhawk_rpi) — TELEM2↔Pi GPIO14/15 wiring, TX/RX crossed, `/dev/ttyAMA0`.
- [PX4 v1.16 Arm/Disarm/Prearm](https://docs.px4.io/v1.16/en/advanced_config/prearm_arm_disarm) — kill switch, safety switch / `CBRK_IO_SAFETY`.
- [Holybro Pixhawk 6C Mini (PX4 v1.16)](https://docs.px4.io/v1.16/en/flight_controller/pixhawk6c_mini) — `px4_fmu-v6c_default`, **UART5 = /dev/ttyS3 = TELEM2**, dedicated SBUS/DSM/CPPM RC IN.
- v1.16.0 source, for the four behaviours the docs do not spell out:
  [`UserModeIntention.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/commander/UserModeIntention.cpp) (mode change always allowed while disarmed),
  [`failsafe/framework.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/commander/failsafe/framework.cpp) (no failsafe action while disarmed),
  [`rc_update.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/rc_update/rc_update.cpp) (switch states published only while RC is present + updating),
  [`mavlink_main.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/mavlink/mavlink_main.cpp) (Onboard-mode stream set; flow-control auto starts ON and falls back after >500 ms).
- Project: `docs/hardware_order_list.md` §3/§B, `docs/project_state.json` (`build_tab.brain`).
