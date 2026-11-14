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

> **Status 2026-11-14.** The board is flashed (ArduCopter 4.7.0), motor order and
> direction are verified, the radio is calibrated, and `target.param` is loaded and
> read back (66 of 66). It has never flown. Still to do before it can arm:
> **accelerometer calibration and compass calibration** (§7). Before/after dumps of
> every setting: `dumps/20261114_before.tsv` and `_after.tsv`.
>
> **The flight controller is mounted backwards** (arrow at the tail, found
> 2026-11-14), so `AHRS_ORIENTATION=4`. Calibrate after that is set, never before.
> **Re-run the motor test, props off, before the first arm:** motor A must be the
> front-right of the real nose. The 2026-10-13 record does not say which end was
> called "front"; if A turns out to be back-left, the motor order needs the
> 180° swap first or the first take-off flips.
>
> **RC loss returns the target home, in every mode** (`FS_OPTIONS=0`, decided
> 2026-11-14). The earlier "continue the mission on RC loss" option was written for
> one shared transmitter and a mission that ended by itself. The target now has its
> own transmitter and a mission that loops forever, so continuing would mean a
> target with no radio link flying laps until the battery failsafe.

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
   [firmware.ardupilot.org/Copter/**stable-4.7.0**/KakuteH7/](https://firmware.ardupilot.org/Copter/stable-4.7.0/KakuteH7/).
   The `_with_bl` build includes the bootloader — needed for the first DFU flash.
   Source: same page.

   > ⛔ **Flash 4.7.0 specifically — `target.param` is pinned to it.** "Latest
   > stable" is not good enough here: ArduPilot **renamed five** of the params
   > this pack sets (`GPS_TYPE`→`GPS1_TYPE` in 4.6; `WPNAV_SPEED`→`WP_SPD`,
   > `WPNAV_ACCEL`→`WP_ACC`, `ANGLE_MAX`→`ATC_ANGLE_MAX`,
   > `ARMING_CHECK`→`ARMING_SKIPCHK` in 4.7), **three of them also changed
   > units** (cm/s → m/s, centidegrees → degrees) and one **inverted its sense**.
   > Mission Planner *skips* an unknown name with a warning instead of failing,
   > so loading this file on 4.5/4.6 silently drops those lines. `lint_param.py`
   > enforces the 4.7 spellings; if you must use another release, fix both the
   > file and the linter's `REQUIRED` list first. (Verified 2026-07-26 against
   > `firmware-version.txt` = `4.7.0` and the ArduPilot source at tag
   > `Copter-4.7.0`.)
   >
   > **After loading, read the params back off the FC and diff them against the
   > file.** A dropped write is invisible in the write direction.
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
- **Failsafes**: battery low/critical, RC loss = return home, geofence (80 m
  around the arming point, 50 m ceiling).
- **First-flight setup for a 5-inch quad** (gyro filter 75 Hz, thrust curve 0.49,
  hover-throttle guess 0.2 and the matching filter and acceleration limits), from
  Mission Planner's Initial Parameter Setup formulas. The rate PIDs stay at
  firmware defaults.
- **`ARMING_CHECK=1`** — all pre-arm checks left ON.
- **Flight-mode + arm-switch mapping** (see §6).

> After writing, run the linter locally to confirm the file you loaded is the one
> that passes the gate: `python3 lint_param.py target.param`.

---

## 6. Switches: flight modes and arm/disarm (builder call 2026-11-14)

Sticks, the 3-position mode switch on **channel 6** and the 2-position switch on
**channel 5** are the same channels as on the interceptor. Two things differ on
purpose: the mode in the switch-down position, and what channel 5 does.

| Mode switch (ch6) | Reads | Bands (`FLTMODEn`) | Target (ArduPilot) | Interceptor (PX4) |
|---|---|---|---|---|
| up | 2011 | 6 (`FLTMODE6=5`) | **Loiter**: GPS position hold | Position |
| middle | 1499 | 3–5 (`FLTMODE3..5=2`) | **AltHold**: holds height, no GPS needed | Altitude |
| down | 987 | 1–2 (`FLTMODE1..2=3`) | **Auto**: flies the loaded mission | Stabilized |

- Auto is at an end of the travel, so moving between Loiter and AltHold never
  passes through it. ArduPilot will not arm in Auto with this parameter set, so
  a mission can only be started in the air.
- The switch centre reads 1499, 9 µs above the edge between bands 3 and 4, so
  bands 3, 4 and 5 all carry AltHold. `lint_param.py` enforces that.
- There is no Stabilize position. ArduPilot's first-flight page says to fly first
  in Stabilize and not to take off in AltHold until the height controller has been
  tested in flight ([Initial tuning flight](https://ardupilot.org/copter/docs/initial-tuning-flight.html)).
  To have Stabilize for the first hops, set `FLTMODE3..5` to 0 and put them back
  to 2 afterwards.

**Arm switch (ch5, `RC5_OPTION=153`):** down = disarmed, up = arm. Arming checks
still run. Moving it down **in flight disarms at once and the motors stop**, so it
is also the target's RC kill. Unlike the interceptor's kill switch it cannot be
undone by flipping back quickly: a bumped switch at height is a dropped aircraft.
Powering up with the switch up does not arm. Stick arming (throttle down, yaw
right) still works as well. `RC6_OPTION` must stay 0 (lint-enforced): ch6 is the
mode channel. Source: `RC_Channel.cpp` and `AP_Arming_Copter.cpp` at
`Copter-4.7.0`; [Auxiliary functions](https://ardupilot.org/copter/docs/common-auxiliary-functions.html).

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
3. **Arm switch / kill** — props off, mode switch in the middle (AltHold): arm
   switch up, the motors start at idle; arm switch down, they stop at once.
4. **Arming** — confirm all pre-arm checks pass (they are ON). Fix the flagged
   cause; never disable a check to arm. Source:
   [Pre-arm safety checks](https://ardupilot.org/copter/docs/prearm_safety_check.html).

---

## 9. Missions

**A route that repeats.** Draw it in QGroundControl's Plan view and upload it, or
generate a straight out-and-back leg with `gen_loop_mission.py` (upload with
read-back; `--help` for the options, `--from-fc` uses the drone's own GPS
position as the start of the leg). To make a QGroundControl route repeat, end it
with a "Jump to item" command pointing at the first waypoint, repeat count -1.
Keep every point within the fence: 80 m of where you arm, below 50 m. Auto flies at
`WP_SPD` = 10 m/s unless the plan sets a speed; set about 4 m/s for the first tries.
Fly it by taking off in Loiter or AltHold and moving the mode switch down.

**The tripod mission.** Generate the AUTO `.waypoints` file for the pass matrix
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

Useful flags (`--help` for all): `--leg-length` (far station, default **20 m**),
`--near-range` (near station, **4 m**), `--standoff` (crossing standoff,
**5.5 m**), `--cross-halfwidth` (20 m), `--placard-index-deg` (the mount index
for this block — checked against the HARD ≤4 m/s nose-on cap),
`--off-protocol`. **Negative offsets need the `=` form**
(`--lateral-offset=-5,0,5`) or argparse reads the `-` as a flag.

> ⛔ **DEFAULTS SWEPT 2026-07-25 — they used to manufacture a FALSE NO-GO.** This
> generator shipped `30 / 8 / 17 m`, the geometry `docs/tripod_test_protocol.md`
> §4.1 **superseded on 2026-07-24**: the whole 8–30 m ladder sits *outside* the
> 7.10 m predicted `R_decode90`, so curve (a) would read all-zero and the money
> gate would return **FAIL → do not buy the ~$740 interceptor** for a purely
> geometric reason. Defaults are now the corrected contiguous **4–20 m** grid
> with a **5–6 m tag-envelope crossing standoff**, and the tool now **REFUSES
> (exit 2)** to emit a mission that cannot answer curve (a).
> The one legitimate exception — the curve-(b)/NN crossing block at a 15–20 m
> standoff, which is not envelope-limited the same way — needs `--off-protocol`:
>
> ```bash
> python3 gen_tripod_mission.py --home-lat .. --home-lon .. \
>     --passes crossing --standoff 17 --off-protocol --out nn_crossing.waypoints
> ```
>
> `--self-test` now asserts the **emitted** waypoints against the protocol's
> stations (it previously checked structure + round-trip only, which is why the
> stale defaults passed green for weeks).

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
3. **AUTO passes** — Loiter takeoff → confirm fix → mode switch down to **Auto** to
   fly the loaded mission. Log the pass in the protocol §11 session sheet (pass #, aspect,
   speed, tilt, battery).

> ⚠️ **WHERE YOU ARM MATTERS — the fence and the mission have different centres.**
> ArduPilot sets **HOME at the arming point** (row 0 of the mission file is
> ignored on upload), so the `FENCE_RADIUS` tin-can is centred on **where you
> armed** while every waypoint is placed relative to the **surveyed tripod**. Arm
> too far out and `FENCE_ACTION=1` will **RTL the target mid-pass**, which looks exactly like
> an RC failsafe and is not one. Nothing pre-arm-checks the mission against the fence; with
> `FS_GCS_ENABLE=0` there is no on-site message either, so an unexplained
> mid-pass RTL means *pull the `.BIN` and look for the FENCE/ERR record*.
>
> `gen_tripod_mission.py` **prints the budget for the geometry it just emitted**
> — e.g. `farthest waypoint 20.7 m from the TRIPOD -> … ARM WITHIN 59.3 m OF THE
> TRIPOD` (the off-protocol NN block at `--standoff 17` reaches 26.2 m, so the
> budget drops to 53.8 m). Use that printed number, not one memorised from here.
>
> Same root cause, second consequence: `--alt` is **AGL above the arming point**
> (`MAV_FRAME_GLOBAL_RELATIVE_ALT`), not above the tripod. Arm on ground level
> with the tripod base or protocol §3.1's co-altitude framing is off by the
> ground-height difference. **RTL returns to the arming point**, not the tripod.

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
