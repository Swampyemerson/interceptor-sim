# Ground stereo rig — design study (ADR-0015 P-2)

*Companion to `scripts/stereo_model.py` (the physics) and ADR-0015 (the decision).
Plain-language design for the "smarter sensor" that watches the sky, ranges the
threat, and cues the cheap interceptor. Written for a builder new to stereo
vision — terms are defined on first use.*

## What this rig is, in one sentence

Two cameras a fixed distance apart, both looking at the same drone. Because they
see it from slightly different viewpoints, the **shift** between the two images
(the *disparity*) tells us how **far away** the drone is — the one thing a single
camera cannot measure. That range, plus the drone's own onboard camera bearing,
is what ADR-0015 fuses into the mid-course track.

New terms, once:
- **Baseline (b):** how far apart the two cameras are (metres). Wider = better range accuracy.
- **Focal length (f):** how "zoomed in" the lens is. Given in millimetres (mm) on the lens, and converted to **pixels (f_px = f_mm / pixel_size)** for the math. Longer = more zoom, more pixels on the target, narrower field of view.
- **Disparity (d):** the pixel shift of the target between the left and right image. `d = b·f_px / R`, where R is range.
- **Field of view (FOV):** the angular width the camera sees. Wide lens = big FOV (good for searching), long lens = small FOV (good for reach).

## The one formula that runs everything

Stereo range error grows with the **square of range**:

```
    sigma_R  =  R² / (b · f_px) · sigma_d
```

`sigma_R` is the range uncertainty (metres), `sigma_d` is the disparity error
(pixels). Double the range → 4× worse. Double the baseline or the focal length →
2× better. (Standard result — Teledyne "Stereo Accuracy and Error Modeling";
derived in full in the `stereo_model.py` docstring.)

## Methodology — we model *worse than ideal*, on purpose

This project has **four documented cases** of a clean lab model over-promising
versus what Gazebo actually did (ADR-0011/0014/0015). So every number here comes
in **three tiers**, and the **design conclusions use EXPECTED, never BEST**:

| Tier | Means | Example (sub-pixel matching) |
|---|---|---|
| **BEST** | lab bench, datasheet, ideal | 0.1 px |
| **EXPECTED** | realistic outdoor field rig | 0.4 px |
| **WORST-CREDIBLE** | sun-heated, wind-shaken, cheap lens, stale calibration | 1.0 px |

Every model input maps to a **physically measurable** quantity, so a real bench
test can replace it one-for-one later (see "Bench tests" below).

## The chosen rig (the design point)

| Piece | Choice | Why |
|---|---|---|
| **Sensor** | onsemi **AR0234** global-shutter, 1920×1200, 3.0 µm pixels (Arducam Jetson-native module) | Global shutter = the whole frame is captured at one instant (no rolling-shutter skew on a fast crosser). Native Jetson driver. |
| **Lens** | **16 mm C-mount**, F1.4–2.0 (→ f_px = 5333, HFOV ≈ 20°) | Balances reach (σ_R ≤ 1 m to ~150 m EXPECTED) against a **wide-enough 20° field of view** to search a sky sector and keep a fast crosser in frame. |
| **Baseline** | **2.0 m** | The cost/accuracy knee (below). |
| **Sync** | **hardware GPIO trigger** — one wire fires both shutters together | Removes the sync error term almost entirely (see budget). Without it, a fast crosser injects a range error. |
| **Mount** | rigid 2 m aluminium (or carbon-fibre) bar, **shaded**, on a tripod or pan-tilt head; recalibrated daily | Keeps the two cameras' relative pointing stable between calibrations — the term that a long lens *cannot* fix. |
| **Compute** | Jetson Orin (Nano Super for the EO proof rig; Orin NX 16 GB if adding thermal fusion) | Runs detect-then-track + triangulation on the ground, where power/weight are free (ADR-0015). |
| **Time/space datum** | shared **RTK GNSS base** + **PPS** time pulse | Gets the ground rig and the drone into the same map to ~0.5 m and the same clock to sub-µs (ADR-0015). This, not the cameras, dominates the cue error at handoff range. |

**Global shutter:** every pixel exposes at the same instant, unlike a rolling
shutter that scans top-to-bottom and smears fast motion. Essential for a small,
fast target and for clean stereo matching.

### What it achieves (from `stereo_model.py`, EXPECTED tier)

- **Range accuracy σ_R ≤ 1 m out to ~150 m**; ≤ 0.5 m out to ~106 m; **σ_R ≈ 0.45 m at 100 m** — meets ADR-0015's "sub-metre range to ~100 m" hope.
- **Detection floor ~160 m** (a 0.3 m drone still spans ≥ 10 px — enough for a confident, bird-rejected track). Detection *just* covers the ranging envelope — an honest, balanced design (neither over-lensed nor range-starved).
- **Cross-range (bearing) error ≈ 1 cm at 100 m** — two orders of magnitude better than range error. Confirms ADR-0015: cameras are **angle-strong, range-weak**.
- **WORST-CREDIBLE:** σ_R ≤ 1 m only to ~66 m, and **detection becomes the binding limit at ~59 m** — against a small drone in bad conditions you honestly get ~60 m, not 150 m.

## Why 2 m — the cost/accuracy knee

Range error scales as **1/b**, so wider is always more accurate *in theory*. Two
things stop you going to 6 m:

1. **A longer bar is harder to hold still.** The killer error term is
   *calibration/pointing drift*: if the two cameras' relative aim shifts by a
   tiny angle (thermal expansion in the sun, wind flex), the range error it
   causes is `R²·δθ/b` — and crucially the **focal length cancels out**, so no
   lens fixes it. A longer bar flexes more, so δθ grows with length and eats the
   baseline benefit. In the WORST tier the model finds an **actual accuracy
   optimum at b ≈ 2.2 m** — beyond it a floppier long bar gets *worse*.
2. **Diminishing returns + deployment cost.** Going 1 → 2 m nearly halves σ_R at
   100 m (0.85 → 0.45 m, EXPECTED). Going 2 → 4 m only buys 0.45 → 0.26 m, while
   the bar, the recalibration burden, and the trailer/mount cost all climb. The
   knee is clearly around 2 m.

`sigma_R(100 m)` vs baseline (EXPECTED / WORST), from the model:

| b (m) | EXPECTED σ_R@100 | WORST σ_R@100 |
|---|---|---|
| 1 | 0.85 m | 3.28 m |
| **2** | **0.45 m** | **1.80 m** |
| 3 | 0.32 m | 1.57 m |
| 4 | 0.26 m | 1.71 m ← *rising* |
| 6 | 0.21 m | 2.32 m ← *worse than 2 m* |

(See `plots/stereo_baseline_knee.png`.)

## The three-tier error budget

At R = 100 m, chosen rig, EXPECTED tier — which error source dominates
(share of the total variance):

| Term | What it is | Measurable as | σ_R@100 (EXPECTED) | Share |
|---|---|---|---|---|
| **Matching** | sub-pixel correlator noise (0.1 / 0.4 / 1.0 px) | std of disparity residuals vs a surveyed target | 0.375 m | 71% |
| **Distortion** | residual lens distortion after calibration (0.05 / 0.2 / 0.5 px) | reprojection-error map | 0.188 m | 18% |
| **Calibration** | relative-pointing drift of the 2 m bar (5 / 30 / 150 µrad) | recalibrate over a day, watch extrinsic drift | 0.150 m | 11% |
| **Sync** | shutter mis-timing × target speed (2 µs / 20 µs / 1 ms) | strobe both cameras, compare timestamps | 0.015 m | 0.1% |
| **TOTAL** (RSS) | | | **0.446 m** | 100% |

Two honest takeaways: **matching noise dominates** (so a good correlator and
sharp focus matter most), and **sync is negligible *because* we hardware-trigger**
— with software-only sync (1 ms) it would blow up to ~0.75 m at 100 m against a
fast crosser. That is *why* the hardware trigger is mandatory, not optional.

## Parts and cost (2026)

Prices are real 2026 retail; we take the **higher end** where sources vary.
"Proof rig" = electro-optical (daytime) only.

| Item | Qty | Unit | Subtotal | Source |
|---|---|---|---|---|
| Global-shutter camera (Arducam AR0234 / IMX296, USB3 w/ external-trigger) | 2 | $130 | $260 | Arducam B0429 €119; [welectron](https://www.welectron.com/Arducam-B0429-23MP-AR0234-Color-Global-Shutter-Camera-for-NVIDIA-Jetson-AGX-Orin-Orin-Nano-Orin-NX_1); [Arducam USB3](https://www.amazon.com/Arducam-IMX296-Shutter-High-Speed-Windows/dp/B0DBV4CBDQ) |
| C-mount lens, 16 mm F1.4 (≥ 1/2" format) | 2 | $60 | $120 | [Arducam C-mount](https://www.arducam.com/camera-component/lenses/c-mount-lens-arducam.html); Computar |
| Compute — Jetson **Orin Nano Super** 8 GB dev kit (proof rig) | 1 | $249 | $249 | [NVIDIA](https://developer.nvidia.com/embedded/buy-jetson) |
| RTK GNSS base (u-blox ZED-F9P, e.g. simpleRTK2B) + antenna | 1 | $230 | $230 | [ArduSimple](https://www.ardusimple.com/product/simplertk2b/) €172 |
| Hardware-sync wiring + USB3 active cables (~5 m) | 1 | $80 | $80 | generic |
| Rigid 2 m aluminium/CFRP bar + tripod or pan-tilt head | 1 | $350 | $350 | generic |
| Enclosure, power, cabling, misc | 1 | $150 | $150 | generic |
| **EO-only day-proof ground rig** | | | **≈ $1,440** | |
| *Upgrade:* Jetson **Orin NX 16 GB** module (for thermal fusion / heavier ML) | +1 | +$650 | +$650 | [Arrow SOM ~$989](https://www.arrow.com/en/products/900-13767-0000-000/nvidia.html) vs Nano |
| *Upgrade:* thermal camera (FLIR Boson 640) for night + bird rejection | +1 | +$2,500 | +$2,500 | ADR-0015 |
| *Upgrade:* MANET / telemetry radio (ground side) | +1 | +$150 | +$150 | ADR-0015 |

**Honest note on cost:** ADR-0015 estimated ~$0.8–1.3k for the EO-only rig. Our
itemised, pessimistic total is **~$1.4k** — a bit higher once you actually pay
for a rigid 2 m mount, an RTK base, and long-reach camera cabling. A full
thermal-equipped fielded node lands around **$4.5–5k** (dominated by the FLIR),
matching ADR-0015's upper figure.

**A real gotcha:** a flat MIPI-CSI ribbon cannot span a 2 m baseline. Use **USB3
cameras with an external-trigger line** (as priced) for the proof rig, or
**GMSL2** (coax to ~15 m, +~$200/camera for serialiser/deserialiser) for the
fielded build. Do **not** assume the short ribbon that ships with the module.

## What feeds the sim

`scripts/s2_cue_mock.py` models the ground cue's range noise as
`sigma_R = a + c·R²` and carries the GPS offset in a **separate** `--datum-bias-m`
knob. The physics-derived constants (from `stereo_model.py`, chosen rig):

| Tier | a (m) | c (m/m²) | datum bias (separate knob) |
|---|---|---|---|
| BEST | 0.000 | 1.08e-05 | 0.3 m |
| **EXPECTED** | **0.000** | **4.45e-05** | **0.5 m (shared RTK+PPS)** |
| WORST | 0.214 | 1.94e-04 | 2.5 m (standard GPS) |

Plus, unchanged from the ADR-0015 lab study: update **rate 10 Hz**, **latency
0.12 s mean + 0.05 s jitter**, **and — the #1 lever — the cue must EMIT a filtered
velocity** (σ_v ≈ 0.5 m/s), not just position.

**Important correction the model surfaces.** The mock currently ships
`sigma_R = 0.4 + 0.008·R²`. That `c = 0.008` was hand-set for the *short* handoff
range (it gives a sensible ~2.3 m at 15 m) but is **~180× too steep** for real
stereo geometry — extrapolated to 100 m it predicts **80 m** of error, which is
absurd for a 2 m rig. The physics says a real 2 m rig's *stereo noise* is ~1 cm
at 15 m and ~0.45 m at 100 m. The 2–4 m cue error the mock models at handoff is
**real**, but it is the **GPS datum/clock offset** (ADR-0015), which belongs in
`--datum-bias-m`, **not** in `c`. Recommended fix: set `sigma_R(R) ≈ 4.45e-05·R²`
(stereo noise) and carry the 0.5 m (RTK) / 2.5 m (GPS) offset in `--datum-bias-m`.

## Bench tests — how to replace every number with a measurement

Each model input is deliberately something you can measure on a bench (folds
into ADR-0015 build-plan step 4). Do these before buying the full rig:

1. **Matching noise σ_match** — point the two cameras at a **surveyed** target at
   50 / 100 / 150 m, log disparity for a minute, take the standard deviation.
   Replaces the 0.4 px EXPECTED assumption directly.
2. **Calibration drift δθ** — calibrate at dawn, re-check the epipolar/extrinsic
   error every hour through a hot afternoon (bar in the sun, then shaded).
   Replaces the 30 µrad EXPECTED assumption; tells you the recal interval.
3. **Sync error Δt_sync** — strobe a fast blinking LED, compare the two cameras'
   captured phase. Confirms the hardware trigger really is < 20 µs.
4. **Detection floor** — fly (or hang) the real target drone at increasing range,
   record the pixels-across and whether detect-then-track holds a classified,
   bird-rejected lock. Replaces the n_90 = 10 px assumption.
5. **Range truth** — the whole thing, stereo range vs a laser rangefinder /
   surveyed marks at 50–150 m. This is the one number the resume claim rests on.

## Honest limits

- **Day-only (EO).** This proof rig is electro-optical: daylight, decent contrast
  against sky. Night, dawn/dusk, and low-contrast sky need the staged **thermal**
  add-on (ADR-0015) — never claim all-conditions without it.
- **Calibration is a maintenance item, not a one-time step.** Outdoor thermal and
  wind drift move the extrinsics; the design assumes **daily recalibration** (or
  an online self-calibration refinement). Skip it and σ_R degrades toward the
  WORST tier.
- **Wind and mount rigidity set the floor.** The calibration term `R²·δθ/b` is
  the one error no lens can buy down. A cheap tripod in gusts is the fastest way
  to turn the EXPECTED curve into the WORST curve.
- **Small/partly-hidden targets shrink the envelope.** A 0.2 m or side-on drone
  drops the detection floor to ~60 m even in good light (WORST tier) — the rig
  hands off later, leaving a longer onboard-only terminal phase.
- **The rig's precision is not the cue's precision.** At handoff range the cue
  error is dominated by the **GPS datum + clock** offset between ground and drone
  (0.5–2.5 m), not by stereo geometry (~cm). Spending on a wider baseline past
  the knee buys nothing if the RTK/PPS datum isn't tight.

## Sources

- Teledyne Vision Solutions — *Stereo Accuracy and Error Modeling* (range & cross-range formulas, sub-pixel & calibration figures): https://www.teledynevisionsolutions.com/support/support-center/application-note/iis/stereo-accuracy-and-error-modeling/
- *Know Your Limits: Accuracy of Long Range Stereoscopic Object Measurements in Practice* (Springer, 2014): https://link.springer.com/chapter/10.1007/978-3-319-10605-2_7
- Stereo extrinsic thermal drift — *Modeling of systematic errors in stereo-DIC due to camera self-heating* (Sci. Reports 2019): https://www.nature.com/articles/s41598-019-43019-7 ; *Effect of camera temperature variations on stereo-DIC*: https://www.researchgate.net/publication/284515151
- Johnson's criteria (detection/recognition/identification pixel thresholds): https://en.wikipedia.org/wiki/Johnson%27s_criteria
- Small-object detection floor (YOLO sub-32 px limit; detect-then-track recall 0.405→0.861): `docs/perception_design.md`, ADR-0015 #2, and 2025–2026 tiny-object-detection literature (VisDrone-class).
- Hardware trigger / global-shutter sync — Arducam external-trigger docs: https://docs.arducam.com/Nvidia-Jetson-Camera/Global-Shutter-Camera/external-trigger/
- Parts: Arducam AR0234 Jetson module (B0429): https://www.welectron.com/Arducam-B0429-23MP-AR0234-Color-Global-Shutter-Camera-for-NVIDIA-Jetson-AGX-Orin-Orin-Nano-Orin-NX_1 · Arducam IMX296 USB3: https://www.amazon.com/Arducam-IMX296-Shutter-High-Speed-Windows/dp/B0DBV4CBDQ · Jetson pricing: https://developer.nvidia.com/embedded/buy-jetson and https://www.arrow.com/en/products/900-13767-0000-000/nvidia.html · RTK: https://www.ardusimple.com/product/simplertk2b/ · C-mount lens: https://www.arducam.com/camera-component/lenses/c-mount-lens-arducam.html

*Full derivation and all constants are in `scripts/stereo_model.py`; run it to
regenerate every table and the three plots in `plots/`.*
