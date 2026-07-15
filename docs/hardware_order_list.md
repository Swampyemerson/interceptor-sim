<!-- Generated 2026-07-12 by a research workflow (interceptor-hardware-bom). Parent-project hardware BOM for the counter-UAS interceptor + a programmable test target, anchored to the sim-validated design (PX4/MAVSDK offboard, ~100 deg global-shutter cam, up15 tilt, Pi-class compute, AprilTag target). Prices are estimates — verify at purchase. -->

# Interceptor Hardware — Purchase-Ready Order List

*All prices are ballpark USD, "verify at purchase" (FPV / Pi pricing is moving in 2026). † = **skip if you already own it.** Recommended tier shown in the tables; budget/splurge swaps are in section 2.*

---

## 0. Coded-dash pivot — what the new objective changes (2026-07-15)

This list was drawn up for the older ground-cued design. The objective is now the
**coded open-loop DASH → CAMERA-ONLY terminal** (`docs/real_build_coded_dash.md`,
ADR-0076 add #18, Fable review 2026-07-15): no ground-sensor cue, no datalink
mid-course — a pre-programmed dash, then the onboard camera+NN flies the intercept.
**Most of the BOM is unchanged** (airframe A, autopilot B, compute+camera C, power,
safety F all still needed). The deltas that affect **what to buy**:

- **① Wide FoV is now a HARD REQUIREMENT, not a tunable — do NOT buy/fit a narrow or
  long lens.** The coded-dash acquires a crossing target from a *roughly*-aimed dash;
  that robustness (validated at ±30° aim error, 48/48 flights still acquired) *depends
  on* the ~100° FoV. A narrow lens re-creates the "fast crosser walks out of frame,
  0/12" failure (ADR-0024). The M12 set is fine — fit **only** the ~2.5–2.8 mm (~100°)
  element and keep the rest as spares. (This supersedes the old "narrow-lens-first"
  acquisition-range advice.)
- **② The camera mount must attempt PROP CLEARANCE — the $0 fix for the #1 seeker
  failure.** In sim, the interceptor's own prop blades read as a false target
  ("phantom") the camera-only terminal can lock with no cue to veto it. Mounting the
  camera **forward of / above the prop disc** so the blades sit outside the ~100° FoV
  designs the phantom out for free. **Geometry check (audited on the Mark5 Pro):** a
  mid-deck camera CANNOT clear (prop-disc points ~35–40° off-axis, inside ±50°); only a
  **nose-cantilever** mount (flush at the front-plate edge + ~20 mm forward) gets the blades
  outside ~100°, and even then the margin is thin (~11° flush) and real barrel distortion
  captures *wider* than pinhole math — so **design for the forward overhang, print a spare
  (it's first into the ground on a bad day), and treat the bench corner-spin check (§3) as a
  HARD gate.** ⚠️ **There is NO software fallback:** the phantom-free retrained seeker was
  measured FAILING in flight (0/16 acquisitions, ADR-0076 add #18d — `quad_v2` stays deployed).
  Clearance-by-geometry + the no-cue handoff hardening are the ONLY mitigations.
- **③ Add a checkerboard calibration target** (below) — the wide M12 distorts, and every
  range/bearing is wrong until `scripts/calibrate_camera.py` runs. This is the first
  bench gate; it's ~free to print.
- **④ The Hailo-8L (single NPU stream) is confirmed correct.** The pivot makes the NN
  the *sole* single-stream terminal seeker, so the pricier Hailo-8's second stream is
  not needed — keep the 8L (already the recommended part).
- **⑤ No guidance datalink is needed** (this is the jam-resistance story). The ELRS link
  (B) is **safety/kill + arming only**; the SiK radio (D) is **telemetry monitoring
  only**. Neither carries guidance — the interceptor flies the dash open-loop and the
  camera finishes it. No cue radio, no ground sensor rig, no second (cue) camera.
- **⑥ Acquisition range** ("interceptor distance") is bought in **software first** —
  foveated auto-crop on the AR0234's *native* resolution (we currently downscale
  1280→640 and throw pixels away) + a real-data fine-tune. **No new hardware** buys more
  range here; a higher-res sensor is **honesty-gated** (§2). So the AR0234 as-specced is
  right; spend the range effort on the crop/retrain, not a lens or sensor swap.
- **AprilTag stays the flight BASELINE seeker** (not just a sim stand-in): first real
  intercepts fly on the tag; YOLO is validated in shadow mode alongside it before it
  ever steers. So the target's AprilTag placard (E) is load-bearing, not optional.

---

## 0b. Pre-purchase audit (2026-07-15, Fable-reviewed + web-verified) — fixes applied + open items

Verdict: **good to order after the items below.** The core interceptor stack (A–D) verified sound
(6S closes; two-rail power correct; Pi 5 + AI HAT+ + AR0234 coherent, single NPU stream; mount
patterns consistent; PM02-analog↔6C-Mini right; SiK/Pi UART ports + params correct; AUW/T-W
arithmetic plausible). **Applied inline** (2 blockers + fixes):

- **[BLOCKER→fixed] Target FC** SpeedyBee F405 **V5 → V4** (V5 has no official ArduPilot target). (E)
- **[BLOCKER→fixed] Interceptor RC path** ELRS **CRSF-to-UART → SBUS-to-RC-IN**: the 6C Mini has
  only 4 UARTs (GPS/SiK/Pi spoken for) and stock PX4 has no CRSF driver; SBUS→RC IN needs neither. (B)
- **[fixed] Compute BEC** Pololu 5.0 V → **Matek BEC12S-PRO 5.2 V** (avoids Pi-5 undervolt throttle). (C)
- **[fixed] Prop-clearance honesty** §0② now states there is NO software phantom fallback (the
  retrained seeker fails in flight, add #18d) → nose-cantilever mount + a hard bench gate.

**MISSING — add to the order:**
| Item | ~Cost | Why / unblocks |
|---|---|---|
| ~~RTK GNSS pair (u-blox ZED-F9P ×2)~~ **CUT** — see §0c | ~~$220–300~~ **$0** | Was "non-negotiable" ONLY to *measure* a <1 m CPA. The builder re-scoped the criterion to a **BINARY KILL** ("takes the drone out or not", 2026-07-15), which is confirmed by **video (seeker recording + phone slow-mo) + both FCs' ULogs**, not sub-meter RTK. Cut it. (Buy later only if you want a citable sub-meter *number* for the portfolio.) |
| **Broadcast Remote ID module ×2** | ~$35–60 ea | **LEGAL:** both aircraft >250 g outdoors (US) need FAA reg + broadcast Remote ID; neither FC provides it. (Alt: a FRIA field.) |
| **Thrust test rig** (DIY inverted on a scale) | ~$20 | §3 demands a bench T/W re-verify (and roadmap R4e: characterize real max lateral-accel — the one variable that can move the kinematic floor); nothing was listed. |
| **Interceptor lost-model buzzer** | ~$5 | The 6C Mini has no onboard buzzer; the (E) buzzer is on the TARGET. The interceptor lands out after BREAKOFF. |
| **M2.5 nylon standoff/screw kit** | ~$8 | Mounts the Pi 5 + AI HAT+ to the printed tray; not in the consumables kit. |
| **IR-cut filter check on the flight M12 element** | ~$0–8 | Bare M12 kit lenses often lack IR-cut → magenta cast outdoors → hurts a color-trained YOLO. Verify or add. |

*Camera-exposure note (roadmap R4d):* the motion-blur bench gate must be run at **TERMINAL LOS rates
(≥300°/s, exposure ≤1 ms)**, not the 60°/s mid-course spec — terminal rates are 485–1870°/s and a
5 ms exposure smears ~23 px. Confirm the AR0234 can hit ≤1 ms exposure outdoors (it's global-shutter,
so it can — but verify the driver exposes the control).

**⚠️ OPEN DECISION (yours — audit S2): two aircraft, one TX.** One TX + both RX on the same bind
phrase REQUIRES **Model Match** (else both aircraft answer the same sticks — dangerous). But then the
*unselected* aircraft is in RC failsafe — so while you hold the interceptor's kill, the **target has no
independent RC kill.** Choose: **(a)** target ArduPilot continue-in-AUTO on RC-failsafe + tight geofence
(`FS_OPTIONS`/`FENCE_ACTION`), accept no target kill during runs; or **(b)** add a 2nd cheap TX
(RadioMaster Pocket ~$65) + a 2nd operator. Pick one before ordering.

**Notes:** the (D) 22↔15-pin CSI adapter is **redundant** (the AR0234/B0353 ships a 150 mm 15→22-pin
Pi-5 cable — keep the $6 line as a spare/length option). Order the **TX16S MKII internal-ELRS** variant.
Lens: ~2.5 mm ≈ 98° / ~2.7 mm ≈ 94° on the AR0234 — fit 2.5 mm, calibrate, pick nearest fx≈540@1280.
Pin the flight Pi's kernel (Arducam Pivariety driver is kernel-fragile).

**Net price delta:** roughly **+$250–400** over the prior ~$1,936 (RTK pair dominates; V4/BEC swaps save
a little) — the RTK is what makes the <1 m goal *measurable*, so it's load-bearing, not optional.

---

## 0c. Binary-kill COST-CUT re-scope (2026-07-15, Fable-reviewed) — the lean build

Builder re-scoped the success criterion to a **BINARY KILL** ("ideally it either takes the drone
out or doesn't") and asked to cut cost while keeping the project possible. That moots the precision
metrology and lets budget-tier parts in (a *ram vehicle* SHOULD use cheap parts — expect to break
some on both aircraft per successful attempt, so have spares before the first attempt).

**NEW TOTALS:** interceptor **~$740** (was ~$1,300+ with 0b); **everything all-new ~$1,390**
(was ~$2,200–2,300); **~$1,080 if you own TX/charger/tools/USB.** **Saved ~$850–900.** Evidence
under binary-kill = the **seeker camera's own recording + phone slow-mo + both ULogs** (target
attitude/altitude collapse, interceptor accel spike) — $0, and keeps a defensible "camera-only
intercept, kill confirmed on video + logs" claim (gives up only a citable sub-meter *number*).

**THE CUTS (ranked by $ saved):**
| Item | From → To | Saved | Note |
|---|---|---|---|
| RTK pair (§0b) | $220–300 → **CUT** | **$220–300** | binary-kill needs no sub-meter measurement (above) |
| RC TX | TX16S $205 → **RadioMaster Pocket** $65 | **$140** | same ELRS/EdgeTX. **Better: 2× Pocket ($130, save $75) — also fixes the §0b S2 two-aircraft-kill hole** (independent target kill) |
| Camera | AR0234 color + M12 set $150 → **Arducam OV9281 mono GS wide** $36 | **$114** | still global-shutter (the real need) + native libcamera (no Pivariety kernel pain); mono fine for AprilTag. **Flags:** verify HFOV ≥100° on calibration; 1 MP = shorter R_acq + no foveated-crop headroom (re-run the M2 range gate); YOLO later needs a mono retrain (folds into the mandated real-data fine-tune). AR0234 = the upgrade path if range measures short. |
| Telemetry | SiK pair $75 → **DEFER** | **$75** now | monitoring-only (§0⑤); bench/first-hovers over USB + Pi-5 WiFi→QGC. Buy before long-range autonomy. |
| **AI HAT+ (Hailo)** | $70 → **DEFER** | **$70** now | first kills use the **AprilTag baseline (30+ FPS on Pi 5 CPU)** — no NPU needed. CPU YOLO (~5–10 FPS) is NOT viable at terminal LOS rates → buy the HAT when you graduate to the *markerless* kill (an upgrade, not the gate). |
| ESC | Tekko32 65A $88 → 45–55A budget 4-in-1 $42 | **$46** | ample for 2207/6S; watch bench temps |
| Motors | XING2 $84 → EMAX ECO II 2207 $56 | **$28** | T/W still ≥5:1; **shares a spare pool with the target** |
| Frame | Mark5 Pro $60 → TBS Source One V5 $33 | **$27** | re-run the §0② nose-cantilever prop-clearance on the new frame (mandatory anyway) |
| Spares (G) / 3rd LiPo | partial → **DEFER** | **$82** now | keep consumables; buy spare arms/motor/RX + 3rd pack *before* kill attempts |

**DON'T cut:** Pi 5 8GB → 4GB saves only ~$10–30 now (2026 DRAM crisis collapsed the gap — buy 8GB from an official reseller, not Amazon scalp prices).

**KEEP — non-negotiable (project dies without these):** Pixhawk 6C Mini + M10 GPS; a **global-shutter
camera at ≥100° FoV** + checkerboard + nose-cantilever prop-clearance mount (no software phantom
fallback, add #18d); Pi 5 8GB + 5.2 V BEC + active cooler; ELRS RC **kill** + the **whole safety
bundle** (smoke stopper, LiPo bag, fire blanket, glasses, cell checker — never cut safety on a
vehicle built to hit things); **both microSD cards** (ULogs + onboard video ARE the metrology now);
the **target drone + AprilTag placard**; the $20 DIY thrust rig + $5 buzzer + $8 standoffs.

**DEFER (buy when the phase needs it):** AI HAT+ $70 (markerless phase) · SiK $40–75 (long-range) ·
3rd battery $26 · spare arms/motor/RX $56 (before kill attempts) · Remote ID $0–180 (**legal, not
metrology — $0 at a FRIA field, else 2 modules; decide by field before outdoor flight; FAA reg $5/
aircraft either way**) · AR0234 upgrade $150 (only if OV9281 range measures short).

> **THE LEAN "camera-only kill on video" BUILD — ~$1,390 all-new / ~$1,080 if-owned:** interceptor
> **$740** + target **$273** + ground/safety **$343** (Pocket TX, charger, safety, solder, tools) +
> consumables **$35**. Kill confirmation $0 (video + ULogs). Phasing unchanged (§4): buy target + TX +
> charger + safety/solder first (~$580), spread the ~$740 interceptor across Phases 1–3.

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
| RadioMaster RP3 ELRS 2.4GHz diversity nano RX | 1 | $22 | $22 | RadioMaster / RDQ | 2-antenna link holds through hard terminal maneuvers. **Flash it to SBUS output (ELRS ≥3.3.0) and wire to the 6C Mini's dedicated RC IN pin — NOT a UART.** Two verified reasons (Holybro 6C Mini + PX4 docs): the Mini has only 4 UARTs (GPS1=GPS, TELEM1=SiK, TELEM2=Pi → only GPS2 free), and **stock PX4 has no CRSF driver** (CRSF needs a custom firmware build). SBUS → RC IN uses stock PX4, consumes zero UARTs, keeps GPS2 as a spare. Loses CRSF link-stats *in the PX4 logs* only (fine — this link is kill/arm-only, §0⑤). **Must match the ELRS TX (F) — same firmware + bind phrase.** |
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
| Arducam M12 lens set (10 lenses, 20–180°) | 1 | $30 | $30 | Arducam / Amazon | Fit the ~2.5–2.8mm element for the sim's **~100° HFOV** (stock B0353 lens is only ~90°). **Fit ONLY the wide element — the ~100° FoV is a coded-dash REQUIREMENT (§0①), not a tunable; keep the narrow/long lenses as spares, do NOT fly them.** Wide M12 distorts → **checkerboard calibration mandatory** (`scripts/calibrate_camera.py`; `flight/camera.py` undistorts). |
| 3D-printed up-tilt + **prop-clearance** camera mount (up-tilt 10–30°, detent ~15°; camera fwd/up of the prop disc) | 1 | $0–15 | $0–15 | Self-print, or JLCPCB/Craftcloud print service | Sets the sim's **up15 tilt (ADR-0067)** AND pushes the camera **forward/above the prop disc so the blades sit outside the ~100° FoV** — the $0 fix for the own-prop phantom (§0②). **Run the prop-clearance geometry check before printing final.** PETG/ABS for outdoor heat. $12 buy-bracket if no printer (verify it clears props at 100°). |
| Checkerboard calibration target (printed 9×6 inner-corner board on rigid backing) | 1 | $0–8 | $0–8 | Copy shop + foamboard, or Amazon calibration board | **Required before any range/bearing transfers** (§0③): feeds `scripts/calibrate_camera.py` → intrinsics + distortion for `flight.camera`. Print flat, mount rigid, known square size. A pre-printed board (~$8) is flatter than home print. |
| Dedicated compute BEC — **Matek BEC12S-PRO (9–55V→5.2V, 5A cont/9A peak)** | 1 | $16 | $16 | Matek / GetFPV | **Two-rail rule:** powers ONLY the Pi + camera, never off the FC BEC. **Buy the 5.2V Matek up front, NOT the 5.0V Pololu** — measured Pi 5 + Hailo load is only ~2.7A (5.5A was never the risk), but the Pi 5 undervolts at ~4.75V and a flat 5.0V rail loses that margin across wiring/connector resistance → mid-terminal CPU throttle → YOLO FPS collapse. 5.2V default fixes it at the source. Short 16 AWG + 1000µF cap + `usb_max_current_enable=1`. |
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
| SpeedyBee F405 **V4** 55A 30×30 FC+ESC stack (flash ArduCopter) | 1 | $65 | $65 | SpeedyBee / GetFPV | Cheapest **officially ArduPilot-supported** GPS autopilot for a box mission (ardupilot.org "SpeedyBee F4 V3/V4" — **the V5 has NO official ArduPilot target as of early 2026, only an unofficial custom build; don't gamble the target's whole job on it**; re-check the V5 hwdef at order time). Runs ArduPilot, not PX4 — fine, it's just a mover. 3–6S, RC via SBUS, GPS via UART. |
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
- **Camera interface:** MIPI CSI-2, global shutter, **~100° HFOV M12 lens (REQUIRED — do not fit a narrow lens, §0①)**, **checkerboard calibration mandatory** before any range number transfers (real lens distorts; the sim is a zero-distortion pinhole — `flight/camera.py` undistorts).
- **Prop clearance (§0②) — HARD GATE:** with the camera on its final **nose-cantilever** mount at ~100° FoV, **confirm the prop blades fall OUTSIDE the frame** (bench: point at a plain wall, spin props at idle, check the corners). Blades in-frame = the phantom failure mode with **no software fallback** (the phantom-free seeker fails in flight, add #18d) → move the camera further forward/up before flying.
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
4. **Phase 3 — Add the seeker (bench, per the Fable Stage-A ladder).** Pi + AR0234 + prop-clearance tilt mount + BEC + cables (C, D). **Calibrate the camera** (checkerboard → `flight.camera`, RMS ≤ 1 px) → **prop-clearance check** (props outside the frame, §0②) → AprilTag detection/Hz/**motion-blur** gate → **compile YOLO11n onto the Hailo, measure real FPS** → grab hard-negative footage from the real mount. Wire the UART MAVLink link; confirm the Pi commands OFFBOARD (`flight/` core + MAVSDK over TELEM2 serial). Safety module: RC kill live, geofence, dash timeout.
5. **Phase 4 — Coded-dash intercepts, laddered (Stage B/C).** Fly the **coded dash → camera-only terminal** against the target flying its AUTO box (straight legs ≥2 m/s). Order: **(a) AprilTag seeker first** — proves the whole chain on hardware with the strongest perception; **(b) YOLO in SHADOW MODE** (log-only) during those flights → measure real bearing-σ, phantom rate, and acquisition range at zero risk (needs the Stage-B real-data fine-tune, held-out-**flight** validated); **(c) YOLO-guided terminal** only once shadow-mode clears it. Score CPA from dual-GPS logs (`field_score.py`); log everything — the resume line.

---

## 5. Sim → hardware mapping (why this transfers)

| Sim (validated) | Hardware |
|---|---|
| **Coded dash → camera-only terminal** (ADR-0076 add #18, the objective) | Pre-programmed dash heading/speed + onboard camera; **no cue, no guidance datalink** |
| Portable **`flight/` core** (geometry/estimator/aim/pro-nav law/lens undistort, 26 tests, no gz/gt/cue deps) | Runs **byte-identical** on the Pi 5 — the part that literally transfers |
| Camera fixed **up15** tilt (ADR-0067) | Up-tilt + **prop-clearance** bracket, detent ~15° (tune 10–30°) |
| **AprilTag tag36h11** = the flight BASELINE seeker (Stage C) | Printed tag36h11 placard on the target; YOLO validated in shadow mode alongside it |
| PX4 SITL + **MAVSDK offboard over UDP** | Pixhawk PX4 + MAVSDK over **TELEM2 UART** (921600), **same code** |
| Onboard seeker (AprilTag + YOLO11n) | Pi 5 + AI HAT+ (Hailo-8L, single stream) running the same CV stack |
| Camera **HFOV 1.74 rad ≈ 100°, fx≈540 @1280** | AR0234 + ~2.5–2.8mm M12 (**wide REQUIRED**), run CV at ~1280 wide to reuse `fx`; `flight/camera.py` undistorts the real lens |
| **Pro-nav N=5** terminal | PX4 offboard + `flight/` guidance; the coded-dash front-end replaces the sim's cue/handoff stack |

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