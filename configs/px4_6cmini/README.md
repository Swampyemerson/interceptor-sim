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
> button first"* and **the board will not arm until a safety button is wired and
> pressed.** Nothing about the disarmed OFFBOARD gate changes.
>
> ⚠️ **CORRECTED 2026-07-27 (ADR-0091): this pack used to say the M10 would
> supply that button. The M10 actually in hand has a 6-pin cable — it fits GPS2
> only, and the SAFETY_SWITCH / LED pins are GPS1 pins 6/7, which this module
> does not carry.** So there is no safety button anywhere in the current
> hardware. Harmless here (nothing arms in `brn-05`); at the first props-off ARM
> test either wire a button or set `CBRK_IO_SAFETY` back to 22027 *knowingly*.
> **GPS on GPS2 also needs a param:** set `GPS_1_CONFIG` = **"GPS 2"** (not
> `GPS_2_CONFIG`, which would leave the primary pointed at an empty port) and
> `SER_GPS2_BAUD` = Auto. And the M10's **compass will not appear** — v1.16
> `fmu-v6c` probes the external ist8310 on I2C buses 1 and 4 only, and GPS2's
> I2C is bus 2. See ADR-0091 for the three fixes. If you need a deliberate props-off arm test before the M10 arrives,
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

### 5. Wire the three leads: TELEM2 ↔ Pi  (`brn-03`)

> **⚠️ CORRECTED 2026-07-27 — this heading used to say "three Dupont jumpers",
> which is only true of the PI end and sent the builder looking for somewhere to
> push a jumper into.** **TELEM2 is a JST-GH 1.25 mm 6-pin socket** (the same
> latching type as the GPS plug); a Dupont pin physically cannot enter it. You
> need a **JST-GH 6-pin → Dupont-female cable**, or a spare Holybro 6-pin
> telemetry cable with Dupont females crimped onto three of its wires. The
> **`MAIN` / `AUX` 3-pin `S`/`+`/`−` headers do accept jumpers and must NOT be
> used** — they are the PWM motor outputs (servo signal + a 5 V rail), not a
> UART. And note that **POWER1, TELEM1, TELEM2 and GPS2 are all 6-pin JST-GH**
> and differ only by silkscreen — read the label before it clicks in. Wiring card
> (phone-friendly): https://claude.ai/code/artifact/359e5669-d1f9-4a3d-8327-80f747e7f77d

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

**Confirm the header orientation on the actual board before pushing anything on** —
run **`pinout`** on the Pi (ships with Pi OS): it prints your board with the 40-pin
header mapped, so which end is pin 1 is observed, not inferred from a drawing.

#### 5a. Making the lead from a spare TELEM↔TELEM cable (2026-07-29)

> **CHECK THE MAIL FIRST.** A ready-made **JST-GH 6-pin → Dupont pigtail is already on
> order** (`docs/hardware_order_list.md` (D); `project_state.json` → `build_tab.brain` →
> "Pi-to-TELEM2 UART jumper leads"). Holybro sell exactly this part. Only hand-make one if
> you want the link closed before it lands — and note that cutting pins 4/5 off a TELEM
> cable is permanent, so this donor can never carry a SiK radio afterwards.
>
> **You need:** iron + solder, ~2 mm heatshrink (adhesive-lined if you have it — do **not**
> substitute tape on the +5 V stub, it unwraps), fine strippers, and a multimeter with DC
> volts + continuity. JST-GH wire is ~28 AWG, hair-fine: strip ~3 mm on the finest notch or
> nick-and-pull with a blade; ordinary cutters sever it.

**This procedure is ordered so that every identification is MEASURED before anything is
cut, and the wire the Pi drives goes on last.** Do not reorder it — an earlier draft of
this section capped the +5 V wire *before* the step that probes it, which made the
verification impossible to perform.

1. **Do not cut yet.**

2. **Find GND by continuity — no power, no colour.** FC **unpowered, USB unplugged**.
   Plug the intact cable into TELEM2. Meter in **continuity**. One probe on the FC's
   **USB connector shell** (chassis ground); touch the other to each wire *at the far
   plug's back face*. Exactly one beeps: that is **pin 6, GND**. Flag it with tape
   **within 10 mm of the plug boot** — pin order is only guaranteed at the boot, and a
   long cable can cross wires mid-run.

   > Probe the **wires**, never inside the 1.25 mm socket. A probe tip in the socket
   > bridges adjacent pins, which is the exact damage this whole section is avoiding.

3. **Find +5 V.** Now power the FC on **USB only**, Pi nowhere near it. Meter in **DC
   volts**, black probe on your flagged GND. One wire reads **≈5 V** — that is **pin 1**.
   Flag it too. You now have both **ends** of the ribbon measured.

   > **PRE-DECLARED NULL — read this before you measure.** If **nothing** reads ~5 V,
   > that is a legitimate outcome and **not** evidence you did anything wrong: the
   > peripheral 5 V is a firmware-enabled, current-limited, over-current-latching switched
   > rail, and whether it is driven on USB-only is **undocumented for this board**. Do not
   > re-cut, do not re-count. Fall back to: pin 1 is the wire at the **opposite ribbon edge
   > from your measured GND** (cross-check: on most Holybro 6-pin cables pin 1 is the lone
   > **red** wire — but all-black and rainbow variants exist, and one distributor warns in
   > writing that red may not be pin 1, so colour is only ever a cross-check, never the
   > anchor).

4. **Count inward from BOTH measured ends and flag the rest.** From pin 1: next is **2 =
   FC TX**, then 3 = FC RX. From pin 6: next in is 5 = RTS, then 4 = CTS. Counting from a
   ribbon **edge** is flip-invariant — turning the cable over swaps which side the edge is
   on but never reverses the sequence, so there is nothing to get wrong here.

   > Both TX (pin 2) and RTS (pin 5) are FC **outputs** and can both read a clean logic
   > level, which is why they are anchored from **opposite measured ends** rather than by
   > voltage. A valid 3.3 V reading is not a TX fingerprint.

5. **Now cut**, keeping the flags on the plug side. Cut near the middle; keep the finished
   lead under ~25 cm (921600 baud on unshielded Dupont prefers short runs).

6. **Terminate the three you keep** (pins 2, 3, 6). Cut three female-to-female Dupont
   jumpers in half — three different colours, because the Pi end is where a mix-up
   happens. **Slide a 20 mm piece of heatshrink onto each JST wire BEFORE you solder: it
   will not go on afterwards, the Dupont housing is too fat to pass through.** Tin both
   ends, lap the strands side by side (not butted end-to-end), solder, cool, slide the
   sleeve over and shrink. **One joint at a time**, so no two bare joints ever sit
   adjacent.

7. **Kill the three you don't.** Cut the **+5 V (pin 1)**, **CTS (4)** and **RTS (5)**
   wires back to ~10 mm from the boot, fold the bare ends back on themselves and seal each
   in its own heatshrink. Short is safer than long: there is then nothing to flop onto the
   Pi header. (Pin 4 is an FC *input* — 5 V onto it is a plausible board-killer, and TELEM
   pins are **not** documented as 5 V tolerant.)

8. **Continuity sweep, unplugged and unpowered.** Unplug the lead from the FC **and**
   unplug the FC's USB — continuity mode injects its own current and means nothing while
   the plug sits in a powered board; even unpowered, the FC's internals give false paths
   between TX/RX/GND. Then sweep **all 15 pairs — every wire against every other,
   including the three capped stubs.** No beep anywhere. The pair that matters most is
   **+5 V against TX, RX or GND**: that one takes out a Pi GPIO or crowbars the FC's 5 V
   rail. A short here is found free with a meter and expensively with smoke.

9. **Strain-relieve, tug-test, label.** Zip-tie or tape the three pigtails together ~20 mm
   behind the joints so any pull lands on the bundle. Then pull each joint firmly. A joint
   that moves now would have failed mid-run and looked like a firmware problem. Flag the
   ends `TX→Pi 10`, `RX→Pi 8`, `GND→Pi 6`, and write `TELEM2 3-wire, no +5 V` on the plug —
   six months from now the flags are the only pinout that exists.

10. **Wire it to the Pi with everything dead.** FC USB out, Pi PSU out, no lights. Run
    `pinout` on the Pi beforehand so you know which corner is pin 1, and note that **the
    GND target (header pin 6) is one pin away from +5 V (header pin 4)** — a one-pin slip
    dumps the Pi's 5 V rail into the FC's ground. Connect **GND first**, then RX, then TX;
    when dismantling later, remove **GND last**. Until GND is joined, the two independently
    powered grounds float relative to each other, so a signal wire landing first puts that
    offset across a 3.3 V input. Never push or pull a Dupont end on a powered header, and
    unplug the JST end **by the housing, never the wires**.

11. **Prove it in two stages — the wire the Pi drives goes last.** With everything off,
    connect **only GND (Pi pin 6) and FC TX (Pi pin 10)**. Two wires, no Pi output
    involved, nothing that can fight. Power up and run §6's byte check — bytes containing
    `fd`/`fe` prove **GND and TX are both correct**. Power down, add the third wire (FC RX
    ← Pi pin 8), and run §6's MAVSDK check: `connected True` proves RX. Each stage has its
    own pass criterion, and a wrong guess at stage one costs nothing.

    > **Precondition:** TELEM2 is only open once `bench.params` is loaded (§3) —
    > `MAV_1_CONFIG=102` is what opens it, and it needs a **reboot**. If the byte check is
    > silent, suspect a skipped §3 or flow control before you suspect your counting.
    > Recovery: shifting the **FC TX** wire to a neighbouring Pi pin is harmless (a wrong
    > 3.3 V pin gives no bytes, not damage). Do **not** brute-force the wire the **Pi**
    > drives — landing Pi pin 8 on the FC's RTS is output fighting output.

12. **Bag the spare half.** It still has a good plug with a **bare +5 V wire** on it. Tape
    its cut ends together, mark it `BARE +5 V`, and put it away — a loose plug with a live
    pin-1 wire is exactly the thing that gets absent-mindedly pushed into TELEM1.

> **Why this is written down.** The pinout tables say "pin 2, pin 3, pin 6" as if the wires
> were labelled. They are not — five of six are typically the same colour, the colour
> convention is vendor discretion rather than standard, and the cable is the one link in
> this chain with no silkscreen, no self-test and no error message. Steps 2–4 exist to turn
> a counted assumption into a measured fact, and step 11 exists because the only test that
> actually proves a serial lead is bytes arriving.
>
> **One forward trap:** if you ever re-apply an airframe in QGC, `MAV_1_FLOW_CTRL` is wiped
> back to auto (§2's mechanism), and this 3-wire lead will stall until `bench.params` is
> re-loaded.

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

> **Card source now lives in the repo:** `configs/px4_6cmini/wiring_card.html` (published to the
> URL above). Edit and republish them together — a bench surface that drifts from this pack is
> the surface the builder actually reads with an iron in his hand.
