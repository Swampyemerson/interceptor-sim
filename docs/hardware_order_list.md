<!-- Generated 2026-07-12 by a research workflow (interceptor-hardware-bom). Parent-project hardware BOM for the counter-UAS interceptor + a programmable test target, anchored to the sim-validated design (PX4/MAVSDK offboard, ~100 deg global-shutter cam, up15 tilt, Pi-class compute, AprilTag target). Prices are estimates — verify at purchase. -->

# Interceptor Hardware — Purchase-Ready Order List

*All prices are ballpark USD, "verify at purchase" (FPV / Pi pricing is moving in 2026). † = **skip if you already own it.** Recommended tier shown in the tables; budget/splurge swaps are in section 2.*

---

## (A) Interceptor airframe & power

| Item | Qty | ~Unit | ~Line | Where | Why + key compatibility |
|---|---|---|---|---|---|
| GEPRC Mark5 Pro 5" freestyle frame | 1 | $60 | $60 | GetFPV / Pyrodrone / RDQ | Roomy squashed-X with a real top deck for the Pi/camera tray. 30.5×30.5 **and** 20×20 mounts, 16×16 M3 motor mount. |
| iFlight XING2 2207 1855KV motor (6S) | 4 | $21 | $84 | iFlight / RDQ / Pyrodrone | ~1.4–1.6 kg/motor → T/W ≈ 6:1 at 950 g AUW. 6S, 16×16 M3, spins 5.1" tri-blade. |
| Holybro Tekko32 F4 Metal 65A 4-in-1 ESC | 1 | $88 | $88 | Holybro / GetFPV | 65A headroom for a loaded agile quad; DShot600. 4–6S, 30.5×30.5 stack, driven from the Pixhawk PWM header. |
| Holybro PM02 V3 power module (**analog**) | 1 | $19 | $19 | Holybro / GetFPV | Feeds 5.2V + V/I telemetry to the FC. **Must be analog PM02 (not digital PM02D)** to match the 6C Mini. XT60 pass-through. |
| HQProp 5.1×4.6×3 tri-blade props (4-pack) | 4 | $4 | $16 | GetFPV / RDQ | 1 flight set + 3 spare sets (you WILL break props). 5.1" tri, M5 T-mount, buy CW+CCW. |
| CNHL Black 6S 1500mAh 100C LiPo (XT60) | 3 | $26 | $78 | ChinaHobbyLine / GetFPV | Sized to the 950 g AUW; 3 packs = a full test day. ~245 g each (the mass in the AUW budget). |
| Battery straps + XT60 pack + 16 AWG wire | 1 | $10 | $10 | GetFPV / Amazon | Secures the LiPo; spare XT60 pigtails + thick wire for the power taps. |

**Subtotal (A): ~$355**

---

## (B) Autopilot, GPS & radio link

| Item | Qty | ~Unit | ~Line | Where | Why + key compatibility |
|---|---|---|---|---|---|
| Holybro Pixhawk 6C Mini (STM32H743, FMUv6C) | 1 | $131 | $131 | Holybro / GetFPV | **The exact stack the sim validated** — native PX4 + OFFBOARD/MAVLink → params & guidance transfer directly. Buy the kit with the cable set. Vibration-mount it (not a stack board). |
| Holybro M10 GPS + IST8310 compass | 1 | $45 | $45 | Holybro / GetFPV | GPS + mag for outdoor position hold + the target's box mission. JST-GH to GPS1; mount on a short mast (compass EMI). |
| RadioMaster RP3 ELRS 2.4GHz diversity nano RX | 1 | $22 | $22 | RadioMaster / RDQ | 2-antenna link holds through hard terminal maneuvers. CRSF to a Pixhawk UART. **Must match the ELRS TX (F) — same firmware + bind phrase.** |
| SanDisk High Endurance 64GB microSD (FC logs) | 1 | $12 | $12 | Amazon / Best Buy | PX4 writes ULog flight logs here — your portfolio proof. High-Endurance survives sustained sequential writes. |

**Subtotal (B): ~$210**

---

## (C) Onboard compute, camera & fixed-tilt mount

| Item | Qty | ~Unit | ~Line | Where | Why + key compatibility |
|---|---|---|---|---|---|
| Raspberry Pi 5, 8 GB | 1 | $120 | $120 | PiShop / SparkFun / CanaKit | Runs the whole sim stack natively: MAVSDK-Python + OpenCV + AprilTag on CPU while the NPU does YOLO. 5V/5A USB-C. ~46 g. |
| Raspberry Pi AI HAT+ 13 TOPS (Hailo-8L) | 1 | $70 | $70 | PiShop / SparkFun | YOLO11n @640px at 30+ FPS on the NPU → CPU stays free for MAVLink + guidance. Pi-5-only (single PCIe lane). Stacks over the cooler. |
| Official Pi 5 Active Cooler | 1 | $5 | $5 | Adafruit / PiShop | **Mandatory** under sustained YOLO load — prop wash alone isn't enough. ~25 g. |
| Arducam AR0234 2.3MP **color global-shutter** CSI (B0353) | 1 | $120 | $120 | Arducam / UCTRONICS | Global shutter = **no rolling-shutter smear** (your hard requirement). 2.3MP overspecs the sim's 1.2MP; color aids YOLO + demo. Needs Arducam's Pivariety driver (extra setup). |
| Arducam M12 lens set (10 lenses, 20–180°) | 1 | $30 | $30 | Arducam / Amazon | Fit the ~2.5–2.8mm element to hit the sim's **~100° HFOV** (stock B0353 lens is only ~90°). Wide M12 has real distortion → **checkerboard calibration mandatory** (`scripts/calibrate_camera.py`). |
| 3D-printed adjustable up-tilt bracket (10–30°, detent ~15°) | 1 | $0–15 | $0–15 | Self-print, or JLCPCB/Craftcloud print service | Sets the sim's **up15 tilt (ADR-0067)** and lets you bench-tune. PETG/ABS for outdoor heat. $12 buy-bracket if no printer. |
| Dedicated compute BEC — Pololu D36V50F5 (6S→5V/5.5A) | 1 | $25 | $25 | Pololu | **Two-rail rule:** powers ONLY the Pi + camera, never off the FC BEC. Pi 5 wants 5.1V → short 16 AWG wire + 1000µF cap + `usb_max_current_enable=1`; if undervolt flags appear, swap to a 5.2V-adjustable UBEC (Matek BEC12S-PRO). |
| SanDisk Extreme 128GB A2/U3 microSD (Pi OS + logs) | 1 | $18 | $18 | Amazon / Best Buy | Pi boot + vision stack + telemetry logging. **A2/U3** for OS random-IO (do NOT use an A1 High-Endurance card here). |
| † Official Pi 27W USB-C PD supply (bench) | 1 | $12 | $12 | PiShop / Adafruit | Bench power for headless dev/flashing before the drone BEC is wired; guarantees full 5A. |

**Subtotal (C): ~$400** (≈$388 if you own a USB-C PD supply)

---

## (D) Cables & PC connection

| Item | Qty | ~Unit | ~Line | Where | Why + key compatibility |
|---|---|---|---|---|---|
| Pi 5 camera FPC cable, 22-pin→15-pin (~200mm) | 1 | $6 | $6 | Adafruit / Arducam | **Pi 5 uses the narrow 22-pin CSI** — the camera's included cable will NOT fit. This bridges it. |
| FC↔Pi UART pigtail (JST-GH 6-pin → Dupont) | 1 | $6 | $6 | Holybro / GetFPV | The flight MAVLink link: **TELEM2 ↔ Pi GPIO14/15, both 3.3V, no level shifter, 921600 baud, TX/RX crossed.** |
| † USB-A → USB-C data cable | 2 | $8 | $16 | Amazon / Anker | One flashes PX4/QGC over the FC's USB-C; one is the bench MAVLink fallback. **Must be data, not charge-only.** |
| † USB-to-UART FTDI adapter, **3.3V** (CP2102 / SparkFun FTDI Basic) | 1 | $12 | $12 | SparkFun / Adafruit | Serial console into the Pi + FC debug. **Must be 3.3V** (5V damages the UART). |
| † USB-C/A microSD card reader (UHS-I) | 1 | $9 | $9 | Amazon | Flash Pi OS + pull ULogs off the FC card on your PC. |
| Holybro SiK Telemetry Radio V3 915MHz pair | 1 | $75 | $75 | Holybro / GetFPV | **Primary way to watch + command autonomous runs** — live MAVLink to QGC at the field (HUD, params, RTL/kill). FC TELEM1 ↔ PC USB. **915MHz = US; use 868MHz in EU.** *(Bought once — dedupes the two subsystem lists.)* |

**Subtotal (D): ~$124** (≈$87 if you own the USB cables, FTDI, and card reader)

> **Interceptor build so far (A+B+C+D): ~$1,089**

---

## (E) Target / practice drone + AprilTag  *(separable purchase — buy this FIRST, see phasing)*

Recommended path: a cheap **outdoor GPS ArduPilot 5" quad** flying an AUTO waypoint box. It matches the interceptor's regime (outdoor, GPS, ≥2 m/s straight legs) — the exact conditions the guidance was validated against.

| Item | Qty | ~Unit | ~Line | Where | Why + key compatibility |
|---|---|---|---|---|---|
| SpeedyBee F405 V5 55A 30×30 FC+ESC stack (flash ArduCopter) | 1 | $75 | $75 | SpeedyBee / GetFPV | Cheapest well-supported GPS autopilot for a box mission. Runs **ArduPilot, not PX4 — fine, it's just a mover.** 3–6S, ELRS via CRSF, GPS via UART. |
| Emax ECO II 2207 1900KV motors | 4 | $16 | $64 | GetFPV / Amazon | Budget 6S 5" motors; same class as the interceptor so **spares interchange.** |
| Budget 5" carbon frame (GEPRC Mark4 or equiv ~220mm) | 1 | $40 | $40 | GetFPV / Pyrodrone | Flat top plate for the tag placard standoff. Shares props/motors with the interceptor. |
| Matek M10Q-5883 GNSS + compass | 1 | $28 | $28 | GetFPV / ReadyMadeRC | Makes the outdoor box repeatable + GPS-aligned to the interceptor. UART to the F405. |
| RadioMaster RP1 V2 ELRS 2.4GHz nano RX | 1 | $20 | $20 | RadioMaster / GetFPV | Arming + manual override + failsafe safety. **Binds to the SAME ELRS TX (add a 2nd model).** |
| 5" tri-blade props + spares (multi-pack) | 1 | $14 | $14 | GetFPV / RDQ | Consumables — the target takes the hard landings. Shares the interceptor's 5" class. |
| Printed **AprilTag tag36h11 (id 0)** on rigid foamboard | 1 | $10 | $10 | Copy shop + DIY spray-mount | **This IS the seeker's target-lock** (`TAG_FAMILY='tag36h11'`, `scripts/m2_detect.py`). Print AS LARGE as the quad can carry (~0.25–0.35 m) — detection range scales ~linearly with tag size; re-measure the M2 envelope for the real tag. |
| Placard mount + misc (foamboard, adhesive, standoffs, zip ties, XT60 pigtail, strap, lost-model buzzer) | 1 | $22 | $22 | Amazon / GetFPV | Rigidly mounts the tag facing the approach; buzzer recovers a target that lands out in a field. |
| † Target battery — **share the interceptor's 6S packs** | — | — | $0 | — | Run 6S so it shares packs + charger. Add 2× dedicated 6S ~$50 only if you want to fly both at once. |

**Subtotal (E): ~$273** (batteries shared)

---

## (F) Ground gear & safety

| Item | Qty | ~Unit | ~Line | Where | Why + key compatibility |
|---|---|---|---|---|---|
| † RadioMaster TX16S MKII (internal ELRS 2.4GHz, EdgeTX) | 1 | $205 | $205 | GetFPV / RadioMaster | Binds both airframes' RX + is your **hardware kill switch**. ELRS must match both receivers (firmware + bind phrase). |
| † HOTA D6 Pro AC/DC balance charger (1–6S) | 1 | $70 | $70 | Pyrodrone / RDQ / Amazon | Balance-charges the 6S packs safely; built-in AC = no extra PSU. Serves both aircraft. |
| Fireproof LiPo charging/storage bag | 1 | $20 | $20 | Amazon | **Never charge a 6S pack unattended without one.** |
| LiPo cell checker + low-voltage alarm (1–8S) | 1 | $8 | $8 | GetFPV / Amazon | Per-cell check + over-discharge alarm; plugs into the JST-XH balance lead. |
| Safety glasses (ANSI Z87) | 1 | $12 | $12 | Amazon | 6S tri-blades draw blood — wear any time props are on + the pack is in. |
| Fire blanket (fiberglass ~1×1 m) | 1 | $16 | $16 | Amazon | A LiPo fire is a chemical fire — smother it. |
| VIFLY ShortSaver 2 smoke stopper (XT60, 1–6S) | 1 | $25 | $25 | GetFPV / Pyrodrone | Inline fuse for **every first power-up after soldering** — a wiring short beeps instead of frying the ESC/FC/Pi. |
| † Pinecil V2 smart soldering iron | 1 | $36 | $36 | Pine64 / Amazon | Solders XT60/ESC/motor/BEC joints. Needs a 60W+ USB-C PD brick or 12–24V DC. |
| 63/37 leaded rosin-core solder 0.8mm | 1 | $12 | $12 | Amazon / GetFPV | Easiest to flow for a beginner on power joints. |
| No-clean flux pen | 1 | $8 | $8 | Amazon / GetFPV | The #1 beginner cold-joint fix on big XT60/battery pads. |
| † FPV tool kit (1.5/2.0/2.5mm hex + prop-nut wrench) | 1 | $26 | $26 | GetFPV / iFlight | M2–M3 hex + 2207 prop nuts; a generic set strips them. |
| † Helping-hands / PCB holder w/ magnifier | 1 | $15 | $15 | Amazon | Third hand for tiny FC/ESC pads. |
| † Digital multimeter (continuity + DC >25V) | 1 | $30 | $30 | Amazon | Continuity-check for solder-bridge shorts before power-up; verify both 5V rails. |

**Subtotal (F): ~$483** (≈$101 for a new buyer once you subtract the ~$382 of skip-if-owned big items: TX, charger, iron, tool kit, helping hands, multimeter)

*Optional LOS monitoring (skip — QGC over the SiK radio is the default):* Eachine LCD5802D 7" 5.8GHz monitor **~$75** — but it needs a **separate analog VTX + FPV cam (~$30–40) added to the drone**; it does **not** tap the CSI seeker camera.

---

## (G) Spares & consumables

| Item | Qty | ~Unit | ~Line | Where | Why + key compatibility |
|---|---|---|---|---|---|
| Build consumables kit (16–22 AWG silicone wire, heat-shrink, spare XT60 pairs, zip ties, blue threadlocker, straps) | 1 | $35 | $35 | GetFPV / Amazon | The glue of the build: BEC/UART wiring, strain relief, locked motor/frame screws. |
| Spare frame arms + screw/standoff hardware | 1 | $20 | $20 | Frame vendor | Carbon arms are the first crash casualty. **Order the exact frame model's arms.** |
| Spare ELRS 2.4GHz RX (match the RP3 family) | 1 | $18 | $18 | GetFPV / RDQ | Backup so a damaged RX doesn't ground the build. Same ELRS family/firmware as the airframe RX. |
| Spare motor ×1 (match the airframe 2207) | 1 | $18 | $18 | Motor vendor | A bent bell after a crash otherwise idles the build. Match KV/mount/shaft. |

**Subtotal (G): ~$91** *(interceptor flight + 3 spare prop sets are already in category A)*

---

## 1. Recommended-build totals

| Category | Subtotal |
|---|---|
| (A) Interceptor airframe & power | ~$355 |
| (B) Autopilot, GPS & radio link | ~$210 |
| (C) Onboard compute, camera & tilt mount | ~$400 |
| (D) Cables & PC connection | ~$124 |
| **Interceptor (A+B+C+D)** | **~$1,089** |
| (E) Target / practice drone + AprilTag *(separable)* | ~$273 |
| (F) Ground gear & safety | ~$483 |
| (G) Spares & consumables | ~$91 |
| **GRAND TOTAL — everything new, own nothing** | **~$1,936** |
| **If you own TX + charger + soldering/tools + USB cables** | **~$1,505** |

**The target drone (E) and ground gear (F) are separable purchases** — E is a standalone cheap quad you can (and should) buy first; most of F's cost is one-time gear you keep forever. The pure *interceptor bill of materials* is ~$1,089.

---

## 2. Budget vs Splurge swaps (big-ticket only)

| Group | Budget | **Recommended** | Splurge |
|---|---|---|---|
| **Frame** | TBS Source One V5 — $33 | **GEPRC Mark5 Pro — $60** | iFlight Chimera7 Pro 7" — $95 *(needs 2807 motors + 7" props — a different power set)* |
| **Motors** | EMAX ECO II 2207 1900KV — $56/4 | **XING2 2207 1855KV — $84/4** | T-Motor F60 Pro V 2207 — $108/4 |
| **ESC** | Aikon AK32 45A — $42 | **Tekko32 F4 65A — $88** | (Tekko is already premium) |
| **Battery** | CNHL 6S 1300 ×3 — $69 | **CNHL 6S 1500 ×3 — $78** | Tattu R-Line 6S 1400 130C ×3 — $120 |
| **Compute** | Pi 5 4GB, no HAT (CPU-only ~5–10 FPS) — $90, **saves ~$180** | **Pi 5 8GB + AI HAT+ — $190** | Jetson Orin Nano Super — $249 *(needs Jetson AR0234 SKU + driver, heavier → 7" frame)* |
| **Camera** | Arducam OV9281 **mono** GS wide — $36 *(native libcamera, simpler; still zero smear; AprilTag fine)* | **AR0234 color GS + M12 set — $150** | AR0234 stays (already overspecced); higher-res is **honesty-gated** — must disclose + re-earn M1/M2 |
| **FC** | (Pixhawk-class is the floor for PX4) | **Pixhawk 6C Mini — $131** | Full-size Pixhawk 6C — $166 *(more UARTs/IO)* |
| **RC TX** | RadioMaster Pocket ELRS — $65 | **TX16S MKII — $205** | TX16S MKII MAX — $260 |
| **Target** | Crazyflie 2.1+ + Flow deck + Crazyradio — ~$320 *(indoor, software-first box logic, no GPS/tag)* | **Outdoor ArduPilot 5" — ~$273** | Holybro X500 V2 + Pixhawk running **PX4** — ~$260+FC *(one identical MAVSDK toolchain for both aircraft)* |

---

## 3. Compatibility checklist (confirm before you order)

- **Battery S-rating — 6S EVERYWHERE.** Motors, ESC, both BEC inputs (rated ≥26V; 6S full = 25.2V), FC via PM02. Target runs 6S too so it shares packs + charger.
- **Connectors:** XT60 main power (battery ↔ PM02 ↔ ESC); JST-GH for FC GPS/TELEM; JST-SH 8-pin ESC signal; JST-XH balance leads; **CSI = Pi 5 22-pin ↔ 15-pin camera adapter cable**.
- **Mount patterns:** frame **30.5×30.5** (ESC/stack) + **20×20**; motors **16×16 M3**; props **M5 T-mount** (buy CW+CCW).
- **AUW vs thrust:** design AUW ~950 g, static thrust ~6.0 kg → **T/W ≈ 6.3:1** (worst case ~1.1 kg → ≈5.5:1). Meets the **≥5:1** hard target — but **re-verify on a bench thrust stand** at the real loaded weight; if motors/ESC run hot, drop to 1755KV and/or 5×4.3×3 props.
- **Camera interface:** MIPI CSI-2, global shutter, ~100° HFOV M12 lens, **checkerboard calibration mandatory** before any range number transfers (real lens has distortion; the sim is a zero-distortion pinhole).
- **Compute power + cooling:** **dedicated ≥5A / 5.1V BEC, physically separate from the FC BEC (two-rail rule)**; active cooler is mandatory; `usb_max_current_enable=1`.
- **FC↔Pi UART:** TELEM2 ↔ Pi GPIO14/15, **both 3.3V (no level shifter)**, 921600, TX/RX crossed; set `MAV_1_CONFIG=TELEM2` and free the Pi's serial console.
- **Power module = analog PM02** (NOT the digital PM02D — that's for the 6X).
- **FC = PX4-reference (Pixhawk-class), never an FPV AIO** (AIO boards are Betaflight/ArduPilot-first, spotty PX4).
- **ELRS:** TX + BOTH receivers on 2.4GHz ELRS, **same firmware major version + same bind phrase**.

---

## 4. Suggested purchase phasing (de-risks a first-time build)

1. **Phase 0 — Learn to fly + solder, cheap.** Buy the **target drone (E)** + **RC TX** + **charger** + **safety/tools (F)**. Fly the box mission, crash it, learn ArduPilot/QGC + soldering on the drone you don't mind breaking. *Lowest-risk money first.*
2. **Phase 1 — Brains on the bench.** FC + GPS + RC RX (B). Flash **PX4**, bind the RX, arm on the bench **with props OFF**, verify motor directions + the **kill switch** in QGC.
3. **Phase 2 — Build the airframe.** Frame + motors + ESC + power (A). Maiden it as a **plain quad, no payload** — confirm T/W and clean flight first.
4. **Phase 3 — Add the seeker.** Pi + AR0234 + tilt mount + BEC + cables (C, D). **Calibrate the camera**, run AprilTag/YOLO on the bench, wire the UART MAVLink link, confirm the Pi commands OFFBOARD.
5. **Phase 4 — Full intercept.** Fly the interceptor (pro-nav) against the target flying its box. Compare pursuit vs pro-nav miss distance, log everything — the resume line.

---

## 5. Sim → hardware mapping (why this transfers)

| Sim (validated) | Hardware |
|---|---|
| Camera fixed **up15** tilt (ADR-0067) | Adjustable bracket, detent ~15° (tune 10–30°) |
| **AprilTag tag36h11** target-lock stand-in | Printed tag36h11 placard on the target |
| PX4 SITL + **MAVSDK offboard over UDP** | Pixhawk PX4 + MAVSDK over UART/USB, **same code** |
| Onboard seeker (AprilTag + YOLO11n) | Pi 5 + AI HAT+ running the same CV stack |
| Camera **HFOV 1.74 rad ≈ 100°, fx≈540 @1280** | AR0234 + ~2.5–2.8mm M12 lens, run CV at ~1280 wide to reuse `fx` |
| **Pro-nav N=5** guidance | PX4 offboard guidance params/code transfer directly |

---

## 6. Assumptions — tell me these to sharpen the list

- **Honest 2.5"-vs-bigger call:** the parent project's aspirational **2.5" quad is rejected as the airframe.** The onboard stack (Pi + HAT + GS camera + GPS + dedicated BEC ≈ 200–280 g) equals or exceeds a whole 2.5" airframe; loaded T/W collapses below the ≥5:1 agility target. 4" is borderline. **5"/6S is the smallest that carries the full stack and still "makes hard adjustments";** 7" is the payload-margin path only if compute grows to a Jetson.
- **Essential vs optional:** *Essential* = everything in A/B/C plus the UART pigtail, camera cable, smoke stopper, LiPo bag, and a way to see telemetry. *Optional* = the AI HAT+ (budget runs CPU-only), color-vs-mono camera, the **SiK radio is strongly recommended** but you *can* bring up over USB first, and **FPV goggles/monitor are fully optional** (monitor via QGC over SiK).

**Tell me to refine if any of these are wrong:**
1. **Budget ceiling?** Right now it's ~$1.9k new / ~$1.5k if you own a TX, charger, and soldering/tools.
2. **What do you already own?** RC transmitter, FPV goggles, LiPo charger, soldering iron/tools, USB cables — each flagged † trims the total.
3. **Indoor or outdoor testing first?** Outdoor GPS is assumed. Indoor-first → swap the target to a Crazyflie (~$320) and defer the GPS regime.
4. **Comfort with soldering + PX4?** Assumed willing-to-learn. If not, we lean on more pre-soldered/turnkey options (and Phase 0 matters even more).
5. **Region + printer?** Assumed **US** (915MHz SiK, FCC ELRS — EU = 868MHz + LBT) and **3D-printer access** for the tilt mount (else the $12 bracket / $15 print service).