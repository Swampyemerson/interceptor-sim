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
own-state EKF streaming to the Pi **and the own-state stamp advancing for the
whole run** (the only evidence bytes came *back* over the wire), the **OFFBOARD
mode switch accepted while disarmed**, and a **measured** pro-nav setpoint stream
— gated on span ≥ 1 s, mean ≥ 2 Hz and **no inter-setpoint gap ≥ `COM_OF_LOSS_T`
(1 s)** — through the whole `flight/deploy/seeker_loop.py` chain on real I/O.

> **The detector is SYNTHETIC here, on purpose (changed 2026-07-26).** This gate
> certifies a *link*, so its verdict must not be decided by perception. It used
> to run the real ONNX detector over replayed frames, and — measured — the first
> setpoint of a default run came from a **hallucinated detection on a frame with
> no target in it**; on a run without that false fire the same correctly-wired
> hardware produces zero setpoints and prints FAIL. The run now passes
> `--synthetic-detector` (deterministic `SmokeSeeker` + blank frames, nothing
> arms). `BENCH_SYNTHETIC=0` runs the real detector — that is a **perception**
> run and its result does not belong to `brn-05`.

**Not verified here:** the MAVLink **wire** rate. The cadence in the run log is
the rate the Pi *computed* setpoints at; `mavsdk_server` re-sends the last
setpoint on its own timer, so the two are different numbers.

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
`configs/px4_6cmini/bench.params` → **reboot**. The file carries **10** lines;
all of them were re-verified against **PX4 v1.16** on 2026-07-26 (see the header
block inside the file).

**What QGC should show you.** Modern QGC previews a **diff** before applying and
*skips lines whose value already matches the vehicle*. Four of the ten already
equal the v1.16 defaults, so expect the diff to list roughly these **six**:

| Param | New value | Means |
|---|---|---|
| `MAV_1_CONFIG` | 102 | MAVLink instance 1 → **TELEM 2** |
| `MAV_1_FLOW_CTRL` | 0 | flow control **forced off** (3-wire link) |
| `MAV_1_FORWARD` | 1 | forward Pi↔GCS traffic (bench convenience) |
| `COM_ARM_WO_GPS` | 0 | deny arming when the GPS preflight check fails |
| `CBRK_IO_SAFETY` | 0 | safety button **required** to arm (see below) |
| `SDLOG_MODE` | 2 | log from boot until shutdown |

Silently skipped because they are already the v1.16 default: `MAV_1_MODE`=2,
`SER_TEL2_BAUD`=921600, `COM_KILL_DISARM`=5.0, `RC_MAP_KILL_SW`=0. If QGC
instead reports a param as **not present on the vehicle**, stop and tell the
session — that means a name is wrong for this firmware.

> ⛔ **`CBRK_IO_SAFETY` is new, and it changes what the board will let you do.**
> The v1.16 **compiled default is 22027**, which means the safety-button arming
> gate is **already bypassed** on a stock board — this pack used to claim the
> opposite ("we keep the safety switch active by not setting it"). Setting **0**
> restores the gate, which is the right fail-closed posture for a props-off
> bench. The cost is real: PX4IO advertises the `safety_button` topic even with
> no button wired, so QGC will show a standing *"Preflight Fail: Press safety
> button first"* and **the board will not arm until the M10 (with its safety
> switch) is on GPS1 and pressed.** Nothing about the disarmed OFFBOARD gate
> changes. If you need a deliberate props-off arm test before the M10 arrives,
> set it back to 22027 *knowingly*, then restore it. Verify on the live board:
> *Parameters → `CBRK_IO_SAFETY`*.

After the reboot, re-open Parameters and confirm the six above. Then
**calibrate sensors** (accel / gyro / mag / level) — sensors-only, bench-safe now.

### 4. Bind ELRS + Radio calibration (SBUS RC in)
Bind the RadioMaster **Pocket** TX to the interceptor's ELRS RX (SBUS output),
wire the RX to the 6C Mini's **dedicated RC IN** pad (not a UART — stock PX4 has
no CRSF driver, §B). QGC *Radio* calibration → then **set `RC_MAP_KILL_SW` by
hand on QGC's *Parameters* page** to the channel your switch actually uses —
Radio calibration does **not** assign it. Flip it and confirm QGC logs *Kill
engaged*, then flip it back and confirm *Kill disengaged*. ELRS is **kill/arm
only** — it carries no guidance (`no-datalink`).

There is no `RC_INPUT_PROTO` to set: the 6C Mini decodes RC in the **PX4IO
co-processor**, which scans SBUS/PPM/DSM in parallel, and that parameter is not
even built into `px4_fmu-v6c`. (`bench.params` §2 has the source citations, and
why writing `0` there would silently *disable* RC on a board that does have it.)

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
BENCH_DEVICE=/dev/ttyACM0 scripts/check_deploy_bench.sh
# the REAL detector over replayed frames (a perception run, not the link gate):
BENCH_SYNTHETIC=0 scripts/check_deploy_bench.sh
# ...or over the live Pi camera:
BENCH_SYNTHETIC=0 BENCH_SOURCE=picamera scripts/check_deploy_bench.sh
```

> ⚠️ **The USB fallback does NOT close `brn-05`.** `/dev/ttyACM0` is PX4's **USB
> CDC-ACM** MAVLink instance, started by `rcS`/`cdcacm_autostart` with its own
> arguments: `MAV_1_CONFIG`, `MAV_1_FLOW_CTRL` and `SER_TEL2_BAUD` are **not on
> that code path**, `BENCH_BAUD` is meaningless on CDC-ACM (which is why the
> old `BENCH_BAUD=57600` in this example has been dropped — it never applied),
> and the 3-wire **floating-CTS** failure mode `MAV_1_FLOW_CTRL=0` exists to
> prevent is unreachable. The script now says so itself: it classifies the
> endpoint and prints **`PARTIAL … brn-05 REMAINS OPEN` and exits 3** for
> anything that is not the TELEM2 UART, so a clean USB run can never be absorbed
> as a green gate.

**Exercise the kill switch by hand** during the run — but note it is **half
deferred**:

- **The switch is UNMAPPED until you do step 4.** QGC *Radio* calibration does
  not assign `RC_MAP_KILL_SW`; you set it on QGC's *Parameters* page.
- **What you CAN observe here (disarmed):** QGC's messages panel shows `Kill
  engaged` / `Kill disengaged`, and `actuator_armed.manual_lockdown` toggles in
  the ULog (that field is named `actuator_armed.kill` on PX4 v1.17+). That is the
  whole switch→FC path, and it is what this step certifies.
- **What you CANNOT observe here:** the `COM_KILL_DISARM` auto-disarm. PX4
  v1.16.0 `Commander::handleAutoDisarm()` runs that block only inside
  `if (isArmed())`, so on a never-armed bench it is **structurally unreachable**.
  It is deferred to the first props-off **arm** test on the real frame. Expected
  evidence for *this* run is therefore a ULog with `manual_lockdown` transitions
  and **no** disarm record — the absence is the PASS, not a gap.
- **Flip the switch back** and confirm `Kill disengaged` before moving on:
  `manual_lockdown` latches, and a latched lockdown carried forward will make the
  later arm test refuse for no visible reason.

### Expected output (PASS)
```
[check_deploy_bench] REAL bench run (props-off, NO arm) against serial:///dev/ttyAMA0:921600
[mavsdk] connecting serial:///dev/ttyAMA0:921600 ...
[mavsdk] connected
[mavsdk] armed=False confirmed (--require-disarmed)
[mavsdk] OFFBOARD active; streaming setpoints ...
[mavsdk] stream complete (--max-frames bound reached (300)): 300 frames, 300 detections, 300 setpoints over 15.0s wall, mean cadence ~20 Hz, max inter-setpoint gap 0.05s, max own_age 0.02s, first lock frame #0 synth00000 (queued to MAVSDK)
[mavsdk] offboard stopped
[mavsdk] done rc=0
[check_deploy_bench] PASS (TELEM2 UART -- connect + own-state round trip + OFFBOARD
[check_deploy_bench]   accepted + a measured setpoint stream, disarmed. brn-05 satisfied;
[check_deploy_bench]   perception NOT exercised by design; see logs/deploy_seeker_bench_*.log)
```
`max own_age` is the number `brn-05` exists to measure — it is what will size
`GuidanceConfig.own_state_max_age_s`, which is deliberately unset until then.
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
5. Connects and OFFBOARD goes active, then **`0 frames, 0 setpoints`** → the
   frame **source**, not the link. This can only happen with `BENCH_SYNTHETIC=0`;
   check the `detector:` line in the run-log header and any OpenCV
   `imread_(...): can't open/read file` warning just above. A missing
   `BENCH_SOURCE` is now caught *before* the serial port is opened (exit 2, "This
   is a CONFIG problem, not a link problem").
6. **`--require-disarmed and the FC is ARMED`** → something armed the vehicle
   (an RC bind, a leftover QGC arm). Disarm it; the gate refuses to stream
   velocity setpoints into an armed FC rather than trusting the props-off note.

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
- [PX4 v1.16 Arm/Disarm/Prearm](https://docs.px4.io/v1.16/en/advanced_config/prearm_arm_disarm) — kill switch, safety switch / `CBRK_IO_SAFETY`. **The polarity claim in this pack comes from the source, not this page:** `circuit_breaker_params.c` (default **22027** = bypassed), `Safety.cpp` (`_safety_off = true` when the breaker is set), `systemCheck.cpp` ("Press safety button first"), `ButtonPublisher.cpp` (advertises `safety_button` unconditionally), `Commander.cpp` `handleAutoDisarm()` (kill auto-disarm is **armed-only**), all at v1.16.0.
- [Holybro Pixhawk 6C Mini (PX4 v1.16)](https://docs.px4.io/v1.16/en/flight_controller/pixhawk6c_mini) — `px4_fmu-v6c_default`, **UART5 = /dev/ttyS3 = TELEM2**, dedicated SBUS/DSM/CPPM RC IN.
- v1.16.0 source, for the four behaviours the docs do not spell out:
  [`UserModeIntention.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/commander/UserModeIntention.cpp) (mode change always allowed while disarmed),
  [`failsafe/framework.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/commander/failsafe/framework.cpp) (no failsafe action while disarmed),
  [`rc_update.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/rc_update/rc_update.cpp) (switch states published only while RC is present + updating),
  [`mavlink_main.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/mavlink/mavlink_main.cpp) (Onboard-mode stream set; flow-control auto starts ON and falls back after >500 ms).
- Project: `docs/hardware_order_list.md` §3/§B, `docs/project_state.json` (`build_tab.brain`).
