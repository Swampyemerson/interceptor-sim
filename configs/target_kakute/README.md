# Target-drone config pack — Kakute H7 (ArduCopter mover)

Engineer's notebook for the **target / practice quad** — the drone that flies the
scripted AUTO box/line for the tripod field test (`docs/tripod_test_protocol.md`).
Everything here is prep done **before** the hardware arrives so build day is
assembly + copy-paste, not research.

- **Hardware (ORDERED 2026-07-20, `docs/project_state.json` `bom_tiers[0]`):**
  Holybro **Kakute H7 V1.5** FC · Tekko32 F4 Metal 65A 4-in-1 ESC · EMAX ECOII
  2207 1900KV ×4 · TBS Source One V6 5" frame · Matek M10Q-5883 GPS+compass ·
  RadioMaster **RP1 V2** ELRS nano RX (wired **CRSF**) · RadioMaster **Pocket M2**
  ELRS TX · CNHL 6S 1500 mAh.
- **Files in this pack:** `target.param` (annotated ArduCopter params),
  `gen_tripod_mission.py` (emits the `.waypoints` AUTO mission), `lint_param.py`
  (param-file linter), `selftest.sh` (the offline gate — run it, it must exit 0).
- **Role:** MOVER only. It is not the interceptor and carries no seeker.

> ⚠️ Board name is **`KakuteH7`** (the V1 target), **not** `KakuteH7v2`, and not
> the `-bdshot`/`-Wing` variants. The ordered board is the V1.5 which runs the
> `KakuteH7` firmware. Source: [Holybro Kakute H7 — Copter docs](https://ardupilot.org/copter/docs/common-holybro-kakuteh7.html).

> ⚠️ **DECISION TO MAKE FIRST — "two aircraft, one TX"** (`docs/hardware_order_list.md`
> §0b). One ELRS TX + both receivers on the same bind phrase needs Model Match,
> and then the *unselected* aircraft is in RC failsafe (no independent kill).
> `target.param` ships **OPTION A** (continue-in-AUTO + tight geofence); switch to
> **OPTION B** (plain RTL) if you fly the target as the *selected*, kill-live
> aircraft. See the FAILSAFE block in `target.param`. This changes how you fly, so
> settle it before the field.

---

## 0. Bench safety (every step below is PROPS OFF)

Do **all** of steps 1–8 with **propellers removed**. Props go on only at step 9,
after the kill switch and motor directions are verified. Wear ANSI Z87 glasses any
time a pack is plugged in. Source: [Arming the motors — safety](https://ardupilot.org/copter/docs/arming_the_motors.html).

---

## 1. Betaflight OUT → ArduPilot IN (flash the Kakute)

The Kakute H7 ships with Betaflight. ArduPilot is loaded onto these FPV-style
("ChibiOS-only") boards over **DFU**, once — after that, updates use the normal
Mission Planner path.

1. Install **Mission Planner** (Windows) and, for the first DFU flash,
   **STM32CubeProgrammer** (installs the DFU USB driver). Source:
   [Loading Firmware onto boards without existing ArduPilot firmware](https://ardupilot.org/copter/docs/common-loading-firmware-onto-chibios-only-boards.html).
2. Download `arducopter_with_bl.hex` for board **`KakuteH7`** from
   [firmware.ardupilot.org](https://firmware.ardupilot.org/Copter/stable/KakuteH7/)
   (pick the latest **stable** Copter). The `_with_bl` build includes the
   bootloader — needed for the first DFU flash. Source: same page.
3. Put the board in **DFU**: hold the FC's BOOT button (or bridge the BOOT pads),
   plug USB, release. Source: same page.
4. In **STM32CubeProgrammer** select the **USB / DFU** interface → **Open file** →
   the `arducopter_with_bl.hex` → **Download**. Source: same page.
5. Reboot. From now on Mission Planner connects normally and future updates use
   *Setup → Install Firmware* like a Pixhawk. Source: same page.

> If Mission Planner already sees the board over USB, you can instead use
> *Setup → Install Firmware → Load custom firmware* and pick the `KakuteH7`
> `arducopter.apj`. DFU above is the guaranteed first-time path.

---

## 2. First connect + microSD (REQUIRED for the log)

1. Connect the board to Mission Planner over USB (it enumerates as a COM port).
2. **Insert a microSD card into the Kakute** *before* flying. ArduCopter writes the
   DataFlash `.BIN` flight log to this card — and that log is the **scored evidence**
   the tripod test needs (`scripts/field_score.py` reads its `POS`/`GPS` messages).
   **No card = no target track = no scorable pass.** The Kakute H7 has an onboard
   microSD slot for logging. Source:
   [Kakute H7 — Copter docs (logging)](https://ardupilot.org/copter/docs/common-holybro-kakuteh7.html)
   · [Logging / LOG_BITMASK](https://ardupilot.org/copter/docs/common-logs.html).

---

## 3. Bind the RP1 V2 receiver to the Pocket M2 (as **model 1**)

The RP1 V2 talks **CRSF** to the FC (full-duplex UART — this is why `target.param`
sets `BRD_ALT_CONFIG=1` to move RC onto USART6; see §5).

**Wiring (CRSF, cross TX↔RX):**
| RP1 V2 pad | → Kakute H7 pad | Note |
|---|---|---|
| RX **TX** (CRSF out) | **R6** (USART6 RX) | receiver transmits to the FC |
| RX **RX** (CRSF in)  | **T6** (USART6 TX) | FC transmits to the receiver |
| 5V | 5V | |
| GND | GND | |

Source: CRSF is full-duplex and needs both wires + a UART —
[Crossfire & ELRS RC Systems](https://ardupilot.org/copter/docs/common-tbs-rc.html)
· [Kakute H7 CRSF note](https://ardupilot.org/copter/docs/common-holybro-kakuteh7.html).

**Bind:**
1. Flash the **RP1 V2** with an ExpressLRS version whose **major** matches the
   Pocket M2's internal ELRS, and set the **same binding phrase** on both. Source:
   [Binding ExpressLRS](https://www.expresslrs.org/quick-start/binding/).
2. On the Pocket M2 (non-touch radio): **SYS → Tools → ExpressLRS → [Bind]**.
   Receiver LED goes solid when bound. Source:
   [RadioMaster — How to bind an ExpressLRS](https://radiomasterrc.freshdesk.com/support/solutions/articles/64000308598-how-to-bind-an-expresslrs).
3. Create/name this as **Model 1 = "TARGET"** in the Pocket's model list. When you
   later add the interceptor as Model 2, use **Model Match** (unique CRSF receiver
   number per model) so one aircraft can never answer the other's sticks. Source:
   [ExpressLRS Model Matching](https://www.expresslrs.org/software/model-config-match/).

---

## 4. Radio (stick) calibration

*Setup → Mandatory Hardware → Radio Calibration.* Move all sticks + switches to
extremes so ArduPilot learns the endpoints. Set channel reversals here if any axis
is backwards. Source:
[Radio Control Calibration](https://ardupilot.org/copter/docs/common-radio-control-calibration.html).

---

## 5. Upload `target.param`

1. *Config → Full Parameter List* (or *Full Parameter Tree*).
2. **Load from file** → pick `configs/target_kakute/target.param` → **Write Params**.
3. **Reboot the FC.** `BATT_MONITOR`, `BRD_ALT_CONFIG`, and the SERIAL protocol
   changes only take effect after a reboot. Source:
   [Kakute H7 — Battery Monitor / RC note](https://ardupilot.org/copter/docs/common-holybro-kakuteh7.html).

What the file sets (each param is annotated inline with its ArduPilot doc source):
- **CRSF RC** on SERIAL6 via `BRD_ALT_CONFIG=1`, `SERIAL6_PROTOCOL=23`, `RSSI_TYPE=3`.
- **Tekko32 DShot600** (`MOT_PWM_TYPE=6`, `SERVO_BLH_AUTO=1`).
- **Battery monitor** for the onboard V + ESC current sense (`BATT_MONITOR=4`, pins/scales).
- **DataFlash logging** with `POS`+`GPS` for `field_score.py` (`LOG_BITMASK=176126`,
  `LOG_FILE_DSRMROT=1` → one `.BIN` per flight).
- **`WPNAV_SPEED=1000`** (10 m/s) so the AUTO legs clear the protocol's ≥9 m/s.
- **Sane failsafes** (battery low/critical, RC failsafe, geofence) + the §0b
  two-aircraft OPTION A/B blocks.
- **`ARMING_CHECK=1`** — all pre-arm checks left ON.
- **Flight-mode + kill-switch mapping** (see §6).

> After writing, run the linter locally to confirm the file you loaded is the one
> that passes the gate: `python3 lint_param.py target.param`.

---

## 6. Flight-mode switch mapping (incl. the **AUTO** position)

`target.param` sets the mode switch on **RC channel 5** (`FLTMODE_CH=5`) with:

| Switch pos | `FLTMODEn` | Mode | Use |
|---|---|---|---|
| 1 | `FLTMODE1=0` | **Stabilize** | manual fallback |
| 2 | `FLTMODE2=2` | **AltHold** | manual, height held |
| 3 | `FLTMODE3=5` | **Loiter** | GPS hold — **take off / land here** |
| 4 | `FLTMODE4=5` | **Loiter** | (same; spare detent) |
| 5 | `FLTMODE5=6` | **RTL** | manual bail-out |
| 6 | `FLTMODE6=3` | **Auto** | **runs the tripod mission** |

To fly a pass: take off and stabilize in **Loiter** (pos 3), confirm GPS 3D fix +
good HDOP, then flip the switch to **Auto** (pos 6) to run the loaded mission.
Assign the 3-pos + 2-pos switches on the Pocket M2 to produce these six channel-5
values (EdgeTX mixer). Source:
[Flight Modes / FLTMODE_CH](https://ardupilot.org/copter/docs/flight-modes.html).

**Kill:** `RC6_OPTION=31` maps **channel 6** to **Motor Emergency Stop** — the
target's independent RC kill. Assign a channel-6 switch on the Pocket. Source:
[Auxiliary function switches (RCx_OPTION 31)](https://ardupilot.org/copter/docs/channel-7-and-8-options.html).

---

## 7. Sensor calibration

1. **Accelerometer** — *Setup → Mandatory Hardware → Accel Calibration.* Place the
   board level, on each side, nose up/down, inverted as prompted. Source:
   [Accelerometer Calibration](https://ardupilot.org/copter/docs/common-accelerometer-calibration.html).
2. **Compass** (Matek M10Q-5883 external mag) — *Setup → Mandatory Hardware →
   Compass.* Do the onboard/large-vehicle dance until it passes; set the mag's
   mounting orientation if the GPS mast faces off-nose. Fly away from rebar/vehicles.
   Source: [Compass Calibration](https://ardupilot.org/copter/docs/common-compass-calibration-in-mission-planner.html).
3. **Level horizon** — the level-cal button in Accel Calibration, board mounted in
   its flight attitude.

---

## 8. Props-OFF pre-flight checks

1. **ESC / motor config** — with `SERVO_BLH_AUTO=1` you can open *Setup → Optional
   Hardware → BLHeli ESC* to talk to the Tekko32. Source:
   [DShot ESCs](https://ardupilot.org/copter/docs/common-dshot-escs.html).
2. **Motor order + direction** — *Setup → Optional Hardware → Motor Test.* Confirm
   motors A/B/C/D spin in the ArduPilot order and the correct directions (Betaflight
   numbering differs — do not assume). Fix order by reassigning `SERVOn_FUNCTION` or
   re-plugging; fix a wrong direction via BLHeli or `SERVO_BLH_RVMASK`. Source:
   [Motor order / connect ESCs and motors](https://ardupilot.org/copter/docs/connect-escs-and-motors.html).
3. **Kill switch** — flip the channel-6 switch and confirm *Motor Emergency Stop*
   engages (motors will not spin). Source:
   [Motor Emergency Stop](https://ardupilot.org/copter/docs/channel-7-and-8-options.html).
4. **Arming** — confirm all pre-arm checks pass (they are ON). Fix the flagged
   cause; never disable a check to arm. Source:
   [Pre-arm safety checks](https://ardupilot.org/copter/docs/prearm_safety_check.html).

---

## 9. Generate + load the tripod mission

Generate the AUTO `.waypoints` file for the pass matrix
(`docs/tripod_test_protocol.md` §4). Set `--home-lat/--home-lon` to the **surveyed
tripod GPS** and `--bearing-deg` to the compass direction the **camera faces**:

```bash
# both aspects, boresight + ±5 m offset approaches, 9 m/s full passes:
python3 gen_tripod_mission.py \
    --home-lat 37.8199 --home-lon -122.4786 --home-alt 12 \
    --bearing-deg 90 --alt 10 --speed 9 --slow-speed 3 \
    --lateral-offset=-5,0,5 --passes approach,crossing \
    --out tripod_mission.waypoints
```

Useful flags (`--help` for all): `--leg-length` (far station, default 30 m),
`--near-range` (near station, 8 m), `--standoff` (crossing standoff, 17 m),
`--cross-halfwidth` (20 m). **Negative offsets need the `=` form**
(`--lateral-offset=-5,0,5`) or argparse reads the `-` as a flag.

The emitted file is **QGC WPL 110** (tab-separated: TAKEOFF → per-pass
`DO_CHANGE_SPEED` + waypoints → RTL). Load it in **Mission Planner** (*Plan → Load
WP File*) or **QGC** (*Plan → Open*). Source:
[Planning a mission with waypoints](https://ardupilot.org/copter/docs/common-planning-a-mission-with-waypoints-and-events.html)
· format: [MAVLink file formats — QGC WPL 110](https://mavlink.io/en/file_formats/).

> The mission encodes range stations as GPS positions relative to the tripod, so
> every pass repeats the same geometry (protocol §4.1). Re-run per field with the
> real tripod coordinates — do **not** reuse a placeholder home.

---

## 10. Fly order (protocol-driven)

1. **Maiden as a plain quad** — Stabilize/AltHold hover, confirm clean flight,
   verify T/W, land. (Do not fly AUTO on an untuned airframe.)
2. **AUTOTUNE** — one AUTOTUNE flight to set the PIDs (`target.param` ships firmware
   defaults on purpose). Source:
   [AutoTune](https://ardupilot.org/copter/docs/autotune.html).
3. **AUTO passes** — Loiter takeoff → confirm fix → switch to **Auto** to fly the
   loaded mission. Log the pass in the protocol §11 session sheet (pass #, aspect,
   speed, tilt, battery).

---

## 11. After each flight — pull the log

Power off, pull the microSD, copy the newest `.BIN` (one per flight thanks to
`LOG_FILE_DSRMROT=1`). This is the target track for scoring:

```bash
# curve-(a)/(b) + CPA scoring consume the DataFlash POS/GPS track:
python3 scripts/field_score.py --bin-b path/to/target_flight.BIN ...
```

Source: `scripts/field_score.py` (DataFlash `.BIN` via pymavlink DFReader; prefers
`POS`, falls back to `GPS`). Archive the `.BIN` with the session per protocol §12.

---

## Self-test (offline, no hardware)

```bash
configs/target_kakute/selftest.sh        # runs both checks, exits 0/1
# or individually:
python3 configs/target_kakute/gen_tripod_mission.py --self-test
python3 configs/target_kakute/lint_param.py configs/target_kakute/target.param
```

`selftest.sh` passes only if the mission generator's structural + round-trip +
geometry checks pass **and** `target.param` parses cleanly (valid `NAME,VALUE`
lines, no duplicate params, all task-mandated params present + sane).
