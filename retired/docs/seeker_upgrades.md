# Onboard seeker upgrades — cost-first ranking against the CPA wall

**Question (builder):** the terminal seeker loses the target at closest approach (CPA).
"Where is that limitation emerging from other than the camera, and what can we add?" —
with **cost** as the heavy focus. This doc costs every hardware/sensor knob beyond the
single fixed-forward camera (ADR-0012) and ranks them by **dollars-per-Pk-point**.

> **Read `docs/terminal_diagnosis.md` first.** A 41-flight Gazebo log forensic (main-session
> re-checked) landed *while this study was in flight* and inverts its conclusion. This
> revision is written to that finding.

---

## TL;DR (the conclusion the data forces)

- **The miss is KINEMATIC, not a terminal-perception problem.** 96% of the final-miss
  variance is already fixed at the handoff latch (r²(ZEM@handoff, miss) = 0.957; r² = 0.990
  at the freeze latch). The camera holds the tag to **1.69 m range / 0.074 s before CPA at
  219 px**; the famous "dropout at CPA" is the *outbound flythrough*, and it adds **−0.031 m**
  (i.e. nothing) to the miss. A **perfect** terminal camera removes 100% of the dropout but
  only **25%** of the miss, because terminal correction capacity ½·a·t_go² = **0.72 m** is far
  below the **1.69 m** ZEM delivered into the 0.41 s window (`terminal_diagnosis.md` items 5-7).
- **Therefore every "hold the target through CPA" upgrade is chasing a 2%-of-miss channel:**
  wider-FOV 2nd camera, physical/digital gimbal, higher frame rate, motion-deblur, event/DVS
  camera, terminal range sensing. **All rejected** below with the ZEM math, not on vibes.
> **⛔ SUPERSEDED for the onboard interceptor by the 2026-07-15 coded-dash pivot
> (ADR-0076 add #18, `docs/real_build_coded_dash.md`, Fable review 2026-07-15).**
> The "add a narrower/longer lens first" recommendation below assumed a GROUND CUE
> placed the target inside a small handoff basket, so trading FoV for focal length
> was free. The real interceptor has **no cue**: it flies a coded open-loop dash and
> the camera-only terminal must ACQUIRE a crossing target from a *roughly*-aimed
> dash. That robustness — validated at **±30° aim error, 48/48 flights still
> acquired** *[correction — ADR-0076 add #18g/#18h: the 48/48 is the coded DASH's
> robustness to aim error (it ENGAGED regardless of aim), but the pre-CPA ENGAGE
> streaks were largely the own-prop PHANTOM, not real-target camera acquisition —
> no camera-guided 3D-quad kill exists; "acquired" ≠ "camera-guided". The
> FoV-as-guardrail conclusion still holds.]* — *depends on the wide (~100°) FoV*; a narrow lens re-creates the
> ADR-0024 "fast crosser walks out of frame, 0/12" failure. So on the real build
> the FoV is a GUARD RAIL, not a lever. The pixels-on-target range problem is
> instead attacked by ~~foveated auto-crop on native resolution~~ + a real-data
> fine-tune. *[SUPERSEDED — ADR-0076 add #18j-fix/#18k: foveated auto-crop is NOT a
> lever (crop = full-frame at the 8–12 m wall band); the wall is FLIGHT-DYNAMIC (the
> ~40° nose-down dash pitch parks the target at the frame-top edge), not resolution/
> aspect — the real lever is ADAPTIVE camera POINTING (#46/ADR-0065); real-data
> detector demoted to the outdoor-appearance gap. Source: `docs/project_state.json`.]*
> The f_px-vs-HFOV *math* below is still correct; only the RECOMMENDATION is stale.

- **The ONE seeker lever that moves Pk is longer-range ACQUISITION** — detect the tag *earlier*
  so t_go is bigger (capacity scales t_go²: acquiring at 12 m instead of 6.5 m raises capacity
  0.72 → ~4.3 m, above the worst delivered ZEM of 4.0 m). For a real seeker this is **detect
  range**, which favors a **narrower/longer lens or more resolution-on-target at range — the
  OPPOSITE of the wider-FOV recommendation** floated in ADR-0015.
- **Cheapest-effective stack:** keep the ADR-0012 fixed-forward global-shutter mono camera,
  **swap to a narrower/longer acquisition lens (~$15-35)**, add nothing else. Free software
  (detect-then-track, IMU-aid coast) rides along for robustness, not for the miss. **Total
  seeker delta over ADR-0012: ~$15-35** — and the diagnosis actually *un-justifies* the
  Hailo-8-over-8L premium (the 2nd-stream that the 8 was bought for is now rejected → −$40 possible).
- **Beats the free IMU-aid baseline on $/Pk-point?** Under the corrected diagnosis the free
  IMU-aid buys ≈0 terminal-miss Pk (−0.03 m), so the bar it sets is ~0. The **acquisition lens
  is the only seeker upgrade that buys material Pk at all** — every CPA-hold upgrade buys ≈0 Pk
  at positive cost (infinitely bad $/Pk). **Add the lens first.**

---

## Why the diagnosis reassigns the bottleneck (the numbers this ranking rests on)

From `docs/terminal_diagnosis.md` (41/41 realistic-cue 6 m/s crossers; miss mean 1.404 m,
median 1.126 m; Pk@1 = 41%, Pk@2 = 76%):

| Channel | Share of the ~1.4 m mean miss | What it means for a hardware knob |
|---|---|---|
| **Kinematic: delivered ZEM > terminal correction capacity** (0.41 s × 8.7 m/s² = 0.72 m vs 1.69 m delivered) | **~70%** | Only a **bigger t_go** (earlier acquisition) or a **smaller delivered ZEM** (mid-course track) helps. |
| **Guidance mechanization losses in the window** (freeze discards last 0.20 s; α-β settle) | **~20-25%** (recoverable ~0.3-0.45 m) | Software (ADR-0014 split-freeze). Not a sensor. |
| **FOV-escape at CPA** (LOS rate 485-1870°/s vs yaw ≤124°/s) | **~2%** (−0.03 m) | This is what every FOV / gimbal / fps upgrade targets. |
| **Blur / scale / decimation** | **~0%** | 219 px tag, 19.5° incidence, no blur model exists in sim. |
| **Detection cadence** | **~0%** | ~14-20 Hz wall; doubling the blind gap at RTF=1 is worth ≤0.14 m. |

The pointing deficit at CPA is real (LOS rate ~10:1 over achievable yaw) but it is the
*geometry of any nonzero miss* — it is downstream of the ZEM, so no camera property fixes it.

### The acquisition-range lever, quantified (pixels-on-target)

Detect range for a target of physical size **S** needing **N_min** pixels across it:

```
R_detect = f_px · S / N_min ,   with  f_px = W_px / (2·tan(HFOV/2))
```

Sim today: W_px = 1280, HFOV = 1.74 rad = **99.7°** → f_px ≈ **540** (matches the sim's fx=539.9,
half-cone ±49.85°). Reliable AprilTag lock needs ~30 px, not the 8 px quad_decimate floor
(ADR-0013: first detection at 8.8-9.3 m with S=0.5 m → 30 px at 9 m checks out).

**f_px vs HFOV — the whole trade in one column (W_px = 1280 fixed):**

| HFOV | f_px | R_detect scaling vs 100° | px on a 0.25 m real drone @ 12 m |
|---|---|---|---|
| 160° (very wide) | 113 | **0.21×** (acquire ~5× later) | 2.4 px (undetectable) |
| 120° (wider) | 370 | 0.68× | 7.7 px |
| **100° (today)** | **540** | **1.00×** | 11.2 px (marginal) |
| ~60° (narrower) | 1109 | **2.05×** | 23 px |
| ~48° (narrow) | 1440 | **2.67×** | **30 px (locks at 12 m)** |
| ~24° (16 mm-class tele) | 3009 | **5.6×** | 63 px |

**So the cheap fix is a lens, not a sensor.** A ~48° HFOV lens roughly triples detect range at
**$0 in silicon** (screw in a longer M12/CS lens). Reaching the same gain by *resolution* needs
~2.67× linear pixels ≈ 7 MP — which does not exist cheaply in a global-shutter mono sensor
(the affordable Pi-native GS ceiling is 2.3 MP AR0234 = only ~1.5× — see camera research).

**The floor on "how narrow" is the handoff basket, not the terminal.** The ground cue must put
the target inside the seeker cone at handoff. At ~12 m: RTK cue (~0.5 m cross-track) needs only
±2.4° (≈5° HFOV); standard-GPS datum (~2.5 m, ADR-0017) needs ±11.8° (≈24° HFOV). So a **~40-60°
HFOV** acquisition lens comfortably contains the basket while buying ~1.7-2.5× detect range. Go
tighter only with an RTK-shared datum. This is the seeker side of ADR-0015's coupling — now
resolved *the other way* (see reconciliation).

**Honesty flag (ADR-0014 boundary):** narrowing the sim FOV is a sensor-geometry change and
must **re-earn M1/M2 detection at the new FOV and be disclosed**. Unlike *widening* (which ADR-0014
correctly called functional tag-inflation), *narrowing* makes the wide-angle margin **harder**, so
it is an honest trade (terminal FOV margin — now shown worthless — for acquisition reach), not
laundering "made it easier" as "improved the algorithm." All Pk deltas below are **estimates from
the channel-share model, pending a Gazebo A/B** (change the SDF `<horizontal_fov>` + raise
`HANDOFF_RANGE`, re-run `mc_batch.sh`). The sim has zero blur/vibration, so any sim Pk is an
**upper bound** on a real seeker's.

---

## Master cost/benefit table (ranked by $/Pk-point)

Cost tiers: **BEST / EXPECTED / WORST-CREDIBLE**. "Fixes what at CPA" maps to a diagnosis channel.

| Upgrade | Fixes what at CPA (channel, share) | Cost (BEST/EXP/WORST) | Weight / power | $/Pk-point verdict | Recommend? |
|---|---|---|---|---|---|
| **Narrower/longer acquisition lens** | Nothing *at* CPA — raises t_go by detecting earlier → attacks the **~70% kinematic** channel | $0 (refocus owned lens) / **~$20-35** M12-CS long lens / ~$50 quality C-mount | +a few g, 0 W | **The only seeker lever with material Pk. Finite, low $/Pk.** | **YES — first** |
| **Higher resolution-on-target sensor** (AR0234 2.3 MP) | Same channel as the lens, but only ~1.5× vs the lens's ~2-3× | ~$60 (bare) / **~$110** (B0353 Pi-native, color) / ~$180 (USB3) | ~50 g, ≤1.45 W | Positive Pk but **~3× the $ of the lens for less gain**; use only if the basket floor blocks the lens | Only if lens insufficient |
| **IMU-aided terminal tracking** | Coast through blind window = **~2% (−0.03 m)**; real value is mid-course dropout + reacquire robustness | **$0** (existing PX4 IMU/EKF) | 0 g, 0 W | ≈0 terminal Pk; free → keep for robustness, not the miss | Keep (free), not headline |
| **Wider-FOV 2nd terminal camera** (ADR-0015's proposal) | FOV-retention = **~2%**, AND moves the *wrong way* on the 70% lever (wider = later acquisition) | $26-60 module + $20 lens + Hailo stream | +40-50 g, ~1 W | **Buys ≈0 Pk and hurts the real lever → fails $/Pk.** | **NO (reversed)** |
| **Higher frame rate / short exposure** | Cadence **~0%** + blur **~0%** | ~$0 (same OV9281 runs 120 fps sensor) | 0 g, +compute | Near-free but ≈0 Pk; USB2 caps raw at 10 fps anyway | No |
| **Physical micro-gimbal** (1-axis) | FOV/boresight-retention = **~2%**; LOS rate 485-1870°/s >> gimbal ~15-300°/s | $13 motor+board / **$48** turn-key / $100+ built | **+10-165 g**, 3-5 W, 30 Hz cmd loop | ≈0 Pk, real mass + a failure point on a disposable | No |
| **Digital gimbal** (ROI crop-track) | Same ~2% channel; can't see outside sensor FOV | **$0** (software) | 0 g, +compute | ≈0 Pk on the miss; loses pixels when it crops | No (for the miss) |
| **Event / DVS camera** | Blur/latency at CPA = **~0-2%** | $400 (non-Pi module) / **~€2,900** (DVXplorer Micro) / ~$3-5.5k (EVK4) | 16 g / <0.7 W (sensor) | Costly + TRL 3-4, no Pi/Hailo drone detector → **R&D luxury** | No |
| **Terminal range sensor** (LiDAR/ToF/radar) | Range/closing-vel estimate — but miss is **capacity-limited, not estimate-limited** (r²=0.99 ZEM@freeze) | $22 (Acconeer) / $40-186 / **$220-392** (TF03 / radar) | 5-90 g, 0.35-3 W | Doesn't add correction capacity; poor on a small drone at 10 m | No (fuller-build cue only) |

---

## Upgrade-by-upgrade (the research questions, answered under the corrected diagnosis)

### Q1 — Wider-FOV 2nd terminal camera on the Hailo → **REJECTED (and reversed)**
- **What it was meant to fix at CPA:** keep the target in-frame through the LOS-rate spike
  (ADR-0015 resolution-order (b)). The diagnosis shows FOV-escape is a **~2% / −0.03 m** channel:
  the tag is lost at bearing −18.7° to −42.2°, *inside* the ±49.85° cone, at 219 px, only 0.037 s
  before CPA. Widening the FOV holds a target the vehicle can no longer correct toward anyway.
- **The core tradeoff it ignores:** wider FOV = fewer px on target = **shorter acquisition range**
  (the table above: 100°→120° costs 32% of detect range; 100°→140° costs 57%). It spends money to
  move *backwards* on the one lever (t_go via acquisition) that actually sets the miss.
- **Cost, if one bought it anyway (cited):** a 2nd global-shutter mono module —
  Arducam OV9281 1 MP MIPI **$25.90** ([DFRobot](https://www.dfrobot.com/product-2892.html)) to
  **~$60** USB ([welectron €56.90](https://www.welectron.com/Arducam-B0332-120fps-Global-Shutter-USB-Camera-Board_1)) —
  plus a wide M12 lens (LN064 3.25 mm **€22.90**,
  [welectron](https://www.welectron.com/Arducam-LN064-120-Degree-Wide-Angle-1-23inch-M12-Lens-with-Lens-Adapter-for-Raspberry-Pi-High-Quality-Camera_1);
  note "120°" is only ~75-90° H on the small IMX296 sensor). Weight ~40-50 g, ~1 W. The Hailo-8
  runs the 2nd stream fine (**~38 fps/stream** on two streams,
  [Seeed benchmark](https://wiki.seeedstudio.com/benchmark_of_multistream_inference_on_raspberrypi5_with_hailo8/)).
- **$/Pk verdict:** ≈0 Pk *and* hurts acquisition → **fails**. Do **not** add. This explicitly
  reverses ADR-0015's step (b).

### Q2 — Higher frame rate / shorter exposure → **REJECTED for the miss**
- **What it fixes:** short exposure (all these sensors reach ~30-50 µs) freezes a fast crosser;
  higher fps buys more shots/lower latency near CPA. But cadence and blur are **~0%** channels
  (there is no blur model in the sim at all; terminal inter-detection gap median 0.026 s).
- **Cost delta:** ~$0 — the ADR-0012 OV9281 already does 120 fps at the sensor
  ([OmniVision](https://www.ovt.com/products/ov9281/)). Real limits: ~60 fps on stock libcamera,
  and **USB2 collapses uncompressed frames to ~10 fps** (use CSI/USB3), min exposure ~40 µs.
- **$/Pk verdict:** near-free but ≈0 Pk on the miss → not worth the pipeline complexity. (Second-order:
  a slightly higher rate marginally sharpens the mid-course velocity track — channel #2 — but the
  cue-velocity-emission lever already dominates that, ADR-0015.) **No.**

### Q3 — Event / DVS camera → **REJECTED (R&D luxury, double miss)**
- **What it would fix:** blur-immunity + µs latency at CPA — i.e. the **~0-2%** channels. Even for
  *acquisition* (the lever that matters) an event sensor is weak on a small, slow-angular-motion
  target at long range (few pixels, low event rate when far/slow-crossing).
- **Cost + maturity (cited):** the only flyable part, **Prophesee GenX320** (320×320, <50 mW, ~16 g),
  has **no public bare-sensor price** (quote-only via
  [Framos](https://framos.com/products/sensors/event-based-sensors/genx320-qty5-27810/)); the only
  cheap ready module is **OpenMV GENX320 $400** which locks to a non-Pi MCU
  ([OpenMV](https://openmv.io/products/genx320-camera-module)); the Pi-5 starter kit is quote-only with
  a **CPU-only, no-Hailo** path ([Prophesee](https://www.prophesee.ai/event-based-starter-kit-genx320-raspberry-pi-5/)).
  Lab-grade: DVXplorer Micro **€2,900** ([iniVation](https://shop.inivation.com/products/dvxplorer-micro-academic-rate)),
  EVK4 ~$3-5.5k. **TRL ~3-4**: no standardized benchmark, best embedded results run on Jetson-class
  <15 W not Pi5+Hailo ([survey](https://arxiv.org/abs/2508.04564),
  [I2MTC 2024](https://arxiv.org/abs/2403.11875)); **no off-the-shelf event→drone detector exists for
  Hailo** — from-scratch R&D + your own event→tensor conversion.
- **$/Pk verdict:** high cost, unproven, wrong channel → **hard reject** as a cheap add; keep only
  as the ADR-0015 R&D hedge if the story ever needs night/blur immunity, not for this miss.

### Q4 — Gimbal (physical) vs digital gimbal → **BOTH REJECTED**
- **What they fix:** keep the seeker on-target as LOS rate spikes = the **~2%** FOV-retention channel.
- **Physical, cheapest usable (cited):** a bare brushless motor **GBM2804 $10.94 / 41 g**
  ([RCTimer](https://rctimer.com/rctimer-gbm2804-hollow-shaft-brushless-gimbal-motor-p0445.html)) still
  needs a controller+IMU+frame (~70-100 g built); a turn-key 2-axis brushless is **$47.99 but 165 g**
  ([SpeedyFPV](https://speedyfpv.com/products/2-axis-brushless-gimbal-for-fpv-camera-drones-lightweight-cnc-aluminum)) —
  a non-starter on a 2.5" (usable payload ~30-50 g), heavy on a 7". A servo tilt (SG90 **$2-4 / 9 g**,
  [PiShop](https://www.pishop.us/product/sg90-180-degrees-9g-micro-servo-motor-tower-pro/)) is light but
  jittery. **All are hopeless at the endgame anyway:** LOS rate 485-1870°/s vs a soft-tuned follow rate
  ~15°/s (mechanical ceiling ~300°/s) and a 30 Hz serial command loop
  ([BaseCam](http://forum.basecamelectronics.com/index.php?p=%2Fdiscussion%2F580%2Ffollow-mode-i-am-getting-crazy%2Fp1),
  [ArduPilot](https://ardupilot.org/copter/docs/common-simplebgc-gimbal.html)). Adds mass + a failure
  point to a disposable airframe that already body-points faster than a gimbal follows.
- **Digital gimbal (ROI crop-and-track):** $0, no moving parts, "slew" = move a crop box (instant) —
  but it only tracks *inside* the fixed sensor FOV and *loses pixels when it crops*
  ([EIS overview](https://dronesdeli.com/blogs/blog-posts/gimbal-vs-electronic-image-stabilization-eis-whats-the-difference)).
  It targets the same ~2% channel. (The one strong-looking embedded UAV crop-track result,
  [arXiv 2512.04883](https://arxiv.org/abs/2512.04883), has been **withdrawn** — do not cite its numbers.)
- **$/Pk verdict:** physical = mass + failure for ≈0 Pk → **no**; digital = free but ≈0 Pk on the miss.
  Both wrong-channel. Keep the fixed cam + body-pointing the design already uses.

### Q5 — IMU-aided terminal tracking → **KEEP (free), but DEMOTED from headline**
- **What it fixes:** predict in-frame position between detections and coast through blinks. The
  diagnosis prices the whole blind window at **−0.031 m** → on the *terminal miss* this is ≈0.
- **Where it still earns its (zero) cost:** coasting through **mid-course cue dropouts** (channel #2,
  track quality) and **dead-reckon reacquisition after a jammer link-cut** (the ADR-0015 handoff-continuity
  gap). Both are robustness, not miss-reduction.
- **Cost ≈ $0** (existing PX4 IMU + EKF2; own-state is legal under the honesty boundary).
- **$/Pk verdict:** the free baseline — but the diagnosis reassigns the bottleneck, so its terminal-Pk
  bar is now ~0. **Keep it; it is not the thing that beats the wall.** The acquisition lens does.

### Q6 — Range sensing at terminal (tiny LiDAR/ToF/radar) → **REJECTED for this failure**
- **The premise is defeated by the diagnosis.** Monocular range ambiguity would starve the
  closing-velocity *estimate* — but the miss is **capacity-limited, not estimate-limited**
  (r²(ZEM@freeze, miss) = 0.990). A sharper Vc does not add the ½·a·t_go² capacity the flight lacks.
- **And the sensors are poor on the target anyway (cited):** LiDAR/ToF "max range" is a big flat board;
  on a 0.2-0.4 m drone at 10 m the cheap/light units are range-dead or beam-diluted — TF-Luna 2.5 m@10%
  ([DFRobot $24.90](https://www.dfrobot.com/product-1995.html)), TFmini-S ~7 m@10% / 5 g
  ([DFRobot $39.90](https://www.dfrobot.com/product-1702.html)), Garmin v4 4.77° beam = 83 cm@10 m
  ([Adafruit $59.95](https://www.adafruit.com/product/4441)), VL53L8CX a sub-2 m proximity part
  ([Pololu $24.95](https://www.pololu.com/product/3419)). The only capable single-point units are heavy or
  need camera-slaved aim — TF03 ~90 g **$219.90** ([DFRobot](https://www.dfrobot.com/product-1963.html)),
  LightWare SF20/C 0.3° beam **$279** ([DigiKey](https://www.digikey.com/en/products/detail/lightware-lidar/SF20-C/15848650)).
  Only **radar gives true Doppler closing-velocity** — TI IWR6843AOPEVM **$186.25**
  ([DigiKey](https://www.digikey.com/en/products/detail/texas-instruments/IWR6843AOPEVM/12165115)),
  honest ~4-10 m on a small drone + custom carrier + Pi DSP burden; Acconeer XM125 **$22.20 / sub-5 g**
  ([DigiKey](https://www.digikey.com/en/products/detail/acconeer-ab/XM125/17883267)) but only a few meters
  on a drone.
- **$/Pk verdict:** no correction-capacity gain → ≈0 Pk on this miss. Radar's Doppler could sharpen the
  *mid-course* ZEM slightly, but the free cue-velocity-emission lever already owns that channel. Park
  radar as a **fuller-build all-weather cue layer** (matches ADR-0015), not a terminal-miss fix. **No.**

### Q7 — Extras: lens/aperture/DoF, multi-cam vs single-wide, GS-vs-RS revisited
- **Aperture / depth-of-field:** a 0% channel here (219 px, sharp, no blur). A distant-focused
  (hyperfocal) fixed lens keeps both the far acquisition target and the near CPA target acceptably sharp;
  cheap M12 board lenses are f/2.0-f/2.8 ([representative: LN051 f/2.0 €33.90](https://www.welectron.com/Arducam-LN051-CS-Lens-for-Raspberry-Pi-HQ-Camera_1)),
  ample. Faster glass would only matter if short exposure were a lever (Q2 — it isn't). **No upgrade.**
- **Multi-camera vs single-wide:** the corrected answer is **neither** — a *single, narrower*
  acquisition-optimized camera. The 2nd wide cam (Q1) is rejected; a single wide lens sacrifices the
  acquisition lever.
- **Global- vs rolling-shutter, revisited:** **keep global shutter** (ADR-0012) — but the *rationale
  shifts*. The original GS mandate leaned on terminal-yaw skew corrupting tag corners; the diagnosis
  demotes terminal pose quality (~0% channel). GS now earns its keep on **airframe-vibration + acquisition-phase
  pose integrity** (clean corners feed the handoff ZEM, channel #2) — still worth it, because a mono GS
  module (OV9281 **$25.90-60**) costs about the same as a rolling-shutter one (Pi Cam 3 Wide **$38.50**,
  [PiShop](https://www.pishop.us/product/raspberry-pi-camera-module-3/)) and rolling-shutter jello under
  vibration is a real bias source. Cheap insurance; keep.

---

## Recommended cheapest-effective terminal-seeker stack

| Item | Choice | Cost | Rationale |
|---|---|---|---|
| Camera body | **Keep ADR-0012 global-shutter mono** (Arducam OV9281 1 MP MIPI, or RPi GS IMX296) | $26-55 (already in the BOM) | GS for vibration/acquisition pose integrity |
| **Lens** | **Swap to a narrower/longer M12/CS lens, ~40-60° HFOV** (RTK datum → tighter) | **~$15-35** | **The one seeker lever with material Pk** — 1.7-2.5× detect range → bigger t_go → capacity² |
| Detector | Detect-then-track, lower effective N_min at range (software) | $0 | Squeezes more acquisition range for free |
| IMU-aid | Coast/predict from PX4 EKF (software) | $0 | Mid-course dropout + link-cut reacquire robustness (not the miss) |
| Compute | Single stream now fits **Hailo-8L (13 TOPS, $70)** — the 2nd-stream reason (Q1) is gone | **−$40 vs Hailo-8** | See ADR-0016 reconciliation |

**Total seeker hardware delta over ADR-0012: ~$15-35** (the lens), with a possible **−$40**
compute saving. **The one upgrade to add first: the narrower/longer acquisition lens** — and it is
directly Gazebo-testable (narrow the SDF `<horizontal_fov>`, raise `HANDOFF_RANGE`, re-run
`mc_batch.sh`; re-earn M2 detection at the new FOV and disclose per the honesty boundary).

**Where the *real* Pk lives (out of seeker scope, flagged for the caller):** the ~70% kinematic
channel is mostly a **mid-course-track / delivered-ZEM** problem (cue **velocity emission** is the
proven #1 lever, ADR-0015 2nd addendum) and the ~20-25% **mechanization reclaim** (ADR-0014
split-freeze). The acquisition lens is the seeker's contribution to the kinematic fix (it buys t_go);
it is complementary to — not a substitute for — the mid-course track quality that sets the ZEM it inherits.

---

## Reconciliation with prior ADRs

- **ADR-0012 (hardware stack):** unchanged except the lens. Keep Pi 5, PX4/Pixhawk, fixed-forward
  mono **global-shutter** camera, X500/7" airframe, no ROS 2, ~$800. The change is a **lens choice**
  (narrower/longer for acquisition), which sits inside the existing ~$75 camera line.
- **ADR-0015 (perception architecture) — the coupling is resolved the OTHER way.** ADR-0015 fork
  ((integration seat, #287)) framed acquisition-focal-length vs terminal-FOV as a genuine one-camera
  conflict and proposed a **2nd wide-FOV terminal camera** as fix (b). The terminal diagnosis
  **dissolves the conflict from the terminal side**: terminal FOV-hold is a ~2% channel, so there is
  nothing to protect — pick the **long/narrow acquisition lens** and accept the (worthless) terminal
  FOV loss. **Fix (b) is withdrawn; the resolution order becomes (a) mid-course track + split-freeze,
  then (b') a narrower acquisition lens, bounded below only by the handoff-basket / cue accuracy.**
- **ADR-0016 (compute):** the Hailo-8 (26 TOPS, $110) was chosen over the Hailo-8L (13 TOPS, $70)
  explicitly for **"headroom for a possible second wide-FOV terminal cam."** That second camera is now
  rejected, so a **single stream fits the 8L** (~38 fps single-stream, above the ~35 fps need) — the
  diagnosis **un-justifies the $40 premium**. Keep the Hailo-8 only if a *different* second stream
  (e.g. an IR cue) is planned; otherwise the 8L is the cost-correct part. Prices:
  [Raspberry Pi AI HAT+](https://www.raspberrypi.com/news/raspberry-pi-ai-hat/).
- **ADR-0017 (stereo rig):** its σ_R / datum-bias budget sets the **handoff-basket size** that floors
  how narrow the acquisition lens can go (RTK 0.5 m → ±2.4° @12 m; standard GPS 2.5 m → ±11.8°). A
  tighter ground cue directly unlocks a longer seeker lens — the two levers compound.
- **ADR-0014 (yaw-rate lever):** the terminal diagnosis supersedes its "terminal blind window" reading
  (as the diagnosis itself states). Yaw-rate authority is now a **~2% channel** — keep it only as the
  near-free robustness tweak it always was, not as a Pk lever.

---

## Honesty boundary

- **All Pk deltas here are estimates from the diagnosis's channel-share model, not measured** — no
  Gazebo A/B of these hardware options exists (Pk-dialing is parked). The acquisition-lens claim is
  falsifiable in-sim and must be run before it is trusted.
- **Narrowing the FOV reopens a previously-validated sensor parameter** → re-earn M1/M2 detection at
  the new FOV and state it plainly next to any Pk headline (ADR-0014 boundary (a)). It is an honest
  trade because it makes the wide-margin problem *harder*, not easier.
- **The sim has zero motion blur / rolling-shutter / vibration / lighting variation** — so it structurally
  *cannot* show a benefit for blur/fps/event upgrades, and any sim Pk is an **upper bound** on a real seeker's.
  The rejection of those upgrades rests on the ZEM math (they target a ~0-2% channel *regardless* of
  whether the sim can see them), not on the sim's blindness to them.
- **Vendor-claim flags carried forward:** "120° / 120 fps" camera figures are marketing (real ~75-90° H
  and USB2-throttled); Hailo "simultaneous multi-stream" is one time-shared core; LiDAR "max range" is a
  flat board not a drone; the withdrawn crop-track paper's numbers are not cited.

---

## ADR-lite

- **Context:** builder asked where the CPA seeker limit comes from "other than the camera" + what to add,
  cost-first by $/Pk-point. A 41-flight Gazebo forensic (`terminal_diagnosis.md`) landed mid-study, inverting it.
- **Finding:** the miss is **kinematic, not perceptual** — 96% locked at handoff (r²=0.957); camera holds to
  1.69 m / 0.074 s pre-CPA; blind window −0.03 m; a perfect terminal camera cuts miss only 25% (capacity 0.72 m
  << delivered ZEM 1.69 m). Every hold-through-CPA knob (wider-FOV 2nd cam, gimbal, fps, deblur, event, range) = ~0-2% channel.
- **Decision:** the one seeker lever is longer-range **ACQUISITION** (t_go²) → a **narrower/longer lens (~$15-35)**
  is the single seeker dollar worth spending; keep the ADR-0012 GS-mono body; IMU-aid + detect-then-track ride free
  (robustness, not the miss). Reverses ADR-0015's 2nd wide-FOV cam (fix b); un-justifies the Hailo-8-over-8L premium
  (ADR-0016, −$40). Add the lens first; validate in Gazebo (SDF FOV + `HANDOFF_RANGE`), re-earn M2, disclose.
- **Beats the free IMU-aid baseline?** Diagnosis makes IMU-aid ≈0 terminal Pk, so the lens is the **only** seeker
  upgrade buying material Pk — wins by default; all else buys ≈0 Pk at positive cost. Pk deltas are pre-Gazebo estimates.
- **Date:** 2026-07-06. Evidence: `terminal_diagnosis.md`; ADR-0012/0014/0015/0016/0017; cited 2024-2026 parts (inline URLs).
