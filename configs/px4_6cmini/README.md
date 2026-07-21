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

> **OFFBOARD needs a position estimate.** PX4 will not accept OFFBOARD
> *velocity-NED* without a valid EKF horizontal position, so the M10 GPS needs a
> fix — run the bench **by a window / outdoors**. This is why `bench.params` sets
> `COM_ARM_WO_GPS=0`: a fix is a real precondition, not an inconvenience.

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
QGC connects automatically over USB. Then *Vehicle Setup → Airframe* → select
**Generic Quadcopter** (this sets `SYS_AUTOSTART = 4001`) → **Apply & reboot**.
This lights up the multicopter control + OFFBOARD stack for the plumbing test;
the **real** airframe ID / geometry / motors are set when the frame is built.
> Do the airframe **before** loading `bench.params` — loading `SYS_AUTOSTART`
> from a file triggers an airframe-default reset on reboot that would clobber the
> tweaks. That is why `SYS_AUTOSTART` is **not** in `bench.params`.

### 3. Load `bench.params`
*Vehicle Setup → Parameters → Tools → Load from file* → pick
`configs/px4_6cmini/bench.params` → **reboot**. Re-open Parameters and confirm
the 8 values (TELEM2 companion link, kill switch, GPS-required arming, boot
logging). Then **calibrate sensors** (accel / gyro / mag / level) — sensors-only,
bench-safe now.

### 4. Bind ELRS + Radio calibration (SBUS RC in)
Bind the RadioMaster **Pocket** TX to the interceptor's ELRS RX (SBUS output),
wire the RX to the 6C Mini's **dedicated RC IN** pad (not a UART — stock PX4 has
no CRSF driver, §B). QGC *Radio* calibration → then map the **kill switch** and
set `RC_MAP_KILL_SW` to the channel your switch actually uses (the `5` in
`bench.params` is a placeholder). Flip it and confirm QGC reads *Kill switch*
active. ELRS is **kill/arm only** — it carries no guidance (`no-datalink`).

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
From the repo root **on the Pi** (GPS fix acquired):
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
on the microSD (logged from boot, `SDLOG_MODE=2`). If `offboard.start()` is
rejected, the most common cause is **no position estimate** — get the GPS fix
(step 0's window) before retrying.

---

## Desk self-test (no hardware)
Validates this pack's own plumbing + that every referenced file exists + that
`bench.params` parses — exits 0/1, touches no hardware:
```bash
scripts/check_deploy_bench.sh --check-only
```

---

## Sources
- [PX4 Serial Port Configuration](https://docs.px4.io/main/en/peripherals/serial_configuration) — `MAV_1_CONFIG`, `SER_TEL2_BAUD`.
- [PX4 MAVLink Peripherals (companion)](https://docs.px4.io/main/en/peripherals/mavlink_peripherals) — `MAV_1_MODE` = Onboard, `MAV_1_FORWARD`.
- [PX4 Raspberry Pi Companion](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi) — TELEM2↔Pi GPIO14/15 wiring, TX/RX crossed, `/dev/ttyAMA0`.
- [PX4 Arm/Disarm/Prearm](https://docs.px4.io/main/en/advanced_config/prearm_arm_disarm) — `RC_MAP_KILL_SW`, `COM_KILL_DISARM`, `COM_ARM_WO_GPS`, safety switch / `CBRK_IO_SAFETY`.
- [PX4 Offboard Mode](https://docs.px4.io/main/en/flight_modes/offboard) — ≥2 Hz setpoint stream + valid position needed to enter OFFBOARD.
- [Holybro Pixhawk 6C Mini (PX4)](https://docs.px4.io/main/en/flight_controller/pixhawk6c_mini) — `px4_fmu-v6c_default` target, TELEM2 = UART5, dedicated SBUS/DSM/CPPM RC IN.
- Project: `docs/hardware_order_list.md` §3/§B, `docs/project_state.json` (`build_tab.brain`).
