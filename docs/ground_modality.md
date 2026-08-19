# Ground sensor modality (P-1) — what the "smarter sensor" should actually be

*Research record for docs/next.md item P-1, the last un-researched perception question.
Companion to ADR-0015 (perception architecture, which STAGED thermal and asserted RF
is defeated by fiber-optic FPV) and ADR-0017 (the EO stereo rig's ~59–160 m detection
floor). This doc pressure-tests those two assertions against 2025–2026 sources and
picks the ground-sensing modality mix. Written for a builder new to the field — terms
are defined on first use. Every number carries a source URL; vendor/marketing numbers
are flagged as such.*

## Methodology (the standing rule — pessimistic-first)

Every range comes in three tiers, and design conclusions use **EXPECTED**, never BEST:

| Tier | Means |
|---|---|
| **BEST** | vendor datasheet / ideal lab / big-target, clear-air |
| **EXPECTED** | published field trial / independent test, realistic |
| **WORST-CREDIBLE** | small (0.3 m) target, low contrast, bad weather, cheap kit |

Where sources conflict, the **pessimistic** number is taken and the conflict is named.
The target we care about is a **~0.25–0.4 m FPV drone** (a first-person-view racing/
attack quad — small, fast, no fiducial, often RF-silent). New terms are defined inline.

**One number frames the whole doc.** A U.S. government C-UAS test of a fielded EO/IR
system measured **40% detection probability at 95% confidence — against a vendor claim
of >90% — with over 700 false alarms per day** ([drone-warfare EO/IR](https://drone-warfare.com/counter-uas/eo-ir-detection/)).
Read every vendor range below through that 90→40 haircut.

---

## 1. EO (visible-light) detection of a small FPV drone

**EO = electro-optical = an ordinary visible-light camera.** It's cheap, passive
(emits nothing), high-resolution, and gives you a picture a human can verify. Its
weakness is that it needs *light* and *contrast*: it is blind at night and collapses at
dawn/dusk and against low-contrast (overcast/haze) sky.

**What actually limits EO range: pixels-on-target, then contrast, then clutter.** The
governing rule is **Johnson's criteria** — a decades-old standard that says how many
pixels must span a target to do a given job ([drone-warfare EO/IR](https://drone-warfare.com/counter-uas/eo-ir-detection/)):

- **Detect** ("something is there"): ~2 pixels across.
- **Recognize** ("it's a drone, not a bird") — *this is the bar that matters for us*: ~8 pixels.
- **Identify** (which drone): ~12–16 pixels.
- Sandia guidance: **minimum 8 pixels on target to judge a UAV threat level**; modern
  neural-net detectors "degrade severely below ~10×10 pixels"
  ([FLIR C-UAS](https://oem.flir.com/learn/discover/thermal-infrared-sensor-design-considerations-for-counter-uas-defense/),
  [airsight](https://www.airsight.com/blog/drone-detection-technology-sensor-modalities)).

So EO range is set by **angular resolution** (how many pixels per degree), which trades
directly against **field of view** (FOV — the angular width the camera sees). A long
lens sees further but searches a narrow slice of sky; a wide lens searches fast but
sees nothing far. This is the same acquisition-vs-terminal-FOV tension ADR-0015 flagged.

**Published EO ranges for a small (DJI-class ~0.3 m) drone:**

| Tier | Range | Source |
|---|---|---|
| BEST (vendor, long lens, clear air) | DroneShield DroneOpt "~380 m for drones"; Spynel-class "multi-km" (marketing) | [drone-warfare EO/IR](https://drone-warfare.com/counter-uas/eo-ir-detection/) |
| EXPECTED (published field trial, DJI-class) | contrast-based detector saw a Mavic ~350 m (dark drone) / Phantom ~440 m (bright drone); a modern DETR net didn't see them until ~240 m | drone-detection literature, search-surfaced ([airsight](https://www.airsight.com/blog/drone-detection-technology-sensor-modalities); provenance: WebSearch synthesis, exact figures not independently re-fetched) |
| WORST (operational, real false alarms) | **40% Pdetect, 700+ false alarms/day** on a fielded system | [drone-warfare EO/IR](https://drone-warfare.com/counter-uas/eo-ir-detection/) |

**Contrast collapse is the real day-to-day killer, not darkness.** EO works in strong
daylight. But dawn/dusk backlight, overcast/white-sky, and haze flatten the drone into
the background; a detector trained on "clean daytime, strong contrast" imagery "may fail
in low-light, haze/humidity, and background variation," and "blur and camera motion are
major constraints… at long range or on a moving platform"
([airsight](https://www.airsight.com/blog/drone-detection-technology-sensor-modalities)).
Domain gap is measurable: a detector scoring 93.55% on known scenes dropped to **66% on
unknown scenes** ([drone-warfare EO/IR](https://drone-warfare.com/counter-uas/eo-ir-detection/)).

**Cross-check against our rig (ADR-0017): consistent, and honestly conservative.** Our
stereo rig is 2× AR0234 (1920×1200) + 16 mm lens, **HFOV ≈ 20°**. That gives ~0.18 mrad/
pixel, so a 0.3 m drone spans **10 px at ~160 m** and 2 px at ~800 m. ADR-0017's
"detection floor ~160 m EXPECTED, ~59 m WORST" is the **recognition/bird-rejected**
floor (≥8–10 px) — *not* the "bare speck" floor. That lines up with fielded EO: the
DroneOpt 380 m and the 350–440 m field trial are **detection** (fewer px) with a
*narrower* lens than our 20° search FOV. Our 160 m is the price of keeping a **wide
enough FOV to search and hold a fast crosser** — a deliberate, honest trade, not an
error. Verdict: **ADR-0017's floor is consistent with fielded EO experience, and if
anything conservative.**

---

## 2. LWIR thermal (FLIR Boson 640-class) — is "thermal solves night" oversold?

**LWIR = long-wave infrared = a thermal camera** (8–14 µm). It doesn't need daylight;
it sees *heat difference* (**delta-T, ΔT**: target temperature minus background
temperature). **No ΔT, no image** — this is the whole story of thermal.

**The drone's hot spots (concrete):** electric motors run **40–80 °C**, the LiPo
battery reaches **~60 °C under load** ([lightpath](https://www.lightpath.com/blog/what-to-look-for-in-a-drone-thermal-imaging-camera)).
Against sky, the *background* is what changes everything:

| Condition | ΔT (drone vs background) | Thermal detectability | Source |
|---|---|---|---|
| Clear cold night sky (~ −40 °C effective) | **80–100 °C** | "Easy" | [drone-warfare EO/IR](https://drone-warfare.com/counter-uas/eo-ir-detection/) |
| Overcast | 30–40 °C | "Moderate" | same |
| Hot summer day | 5–15 °C | "Very challenging" | same |
| **Thermal crossover (dawn & dusk)** | **~0 °C** | **"Near impossible" — camera is effectively blind** | [drone-warfare EO/IR](https://drone-warfare.com/counter-uas/eo-ir-detection/), [airsight](https://www.airsight.com/blog/drone-detection-technology-sensor-modalities) |

**So "thermal solves night" is TRUE but "thermal solves all conditions" is OVERSOLD.**
Thermal is genuinely **best at night** (cold sky = huge ΔT), which is exactly why it is
the right night sensor. But it has two-a-day blind windows (crossover), a hot-day ΔT
collapse, and it is degraded by weather more than salespeople admit:
- **Fog (light): only 50–70% of clear-air range retained.** Coastal/tropical **humidity
  cuts range 20–40%** ([drone-warfare EO/IR](https://drone-warfare.com/counter-uas/eo-ir-detection/)).
- One system "advertised multi-kilometre detection but achieved reliable assessment only
  at 1.6 km" ([drone-warfare EO/IR](https://drone-warfare.com/counter-uas/eo-ir-detection/)) — the same ~2× vendor haircut.

**Pixels-on-target still rules thermal too**, but thermal wins a key trick: against a
cold sky it can flag motion in a **2×2-pixel cluster** — far below EO's needs — because
the background is so uniform ([FLIR C-UAS](https://oem.flir.com/learn/discover/thermal-infrared-sensor-design-considerations-for-counter-uas-defense/)).
Still, robust *classification* wants ≥10×10 px, and a 640×512 core has fewer pixels than
our 1920-wide EO — so thermal's classification range is shorter unless you lens up.

**Real thermal detection range for a small multirotor:** low-cost uncooled systems
"<1000 m"; high-performance ">1000 m"; the multi-km figures are cooled MWIR big-optic
systems (Spynel-S 5 km is maritime marketing) ([drone-warfare EO/IR](https://drone-warfare.com/counter-uas/eo-ir-detection/),
[FLIR C-UAS](https://oem.flir.com/learn/discover/thermal-infrared-sensor-design-considerations-for-counter-uas-defense/)).
EXPECTED for a Boson-640 uncooled core on a small FPV: **a few hundred metres, day or
night, in good ΔT** — comparable to or a bit better than our EO rig at night, worse by
day.

**False targets — thermal is NOT a free bird-rejector (see §3).** A sun-warmed bird, a
sun-heated rooftop/rock, and a drone all emit; thermal gives you a hot blob, not an ID.
The sensitivity spec (Boson+ **≤20 mK NETD** — it resolves 0.02 °C differences,
[dronelife](https://dronelife.com/2025/03/11/teledyne-flir-introduces-radiometric-thermal-camera-modules-for-defense-and-industrial-applications/))
buys sensitivity, not discrimination.

**Cost, 2026 (this moves the budget):** a **FLIR Boson 640 module lists at $3,558**
([GroupGets](https://groupgets.com/products/flir-boson-640)) — *before* a lens and
integration. That is **at/above the top of ADR-0015's "$1,500–3,000 thermal" estimate**
and roughly *doubles* the ~$1,440 EO-only ground rig (ADR-0017). A 320×256 core (e.g.
Boson 320, ~$1,000–1,500) or a Lepton-class micro-core (~$200) is cheaper but trades
resolution → range → bird-rejection.

---

## 3. Bird rejection — the #1 fielded false-alarm source

Birds are the C-UAS false-alarm problem: comparable radar cross-section, similar speed
and altitude, everywhere. The **700+ false alarms/day** on the fielded EO system above
is largely this. What actually tells a drone from a bird at a few hundred metres?

| Discriminator | How it separates drone from bird | Strength | Needs |
|---|---|---|---|
| **Micro-Doppler (radar)** | drone propellers spin at **50–100 Hz** (blade rate) → ~13 kHz Doppler modulation at 24 GHz; bird wing-beat is only **4–10 Hz** — a totally different fingerprint | **Strongest.** Research shows **99.9–100% drone-vs-bird classification** | a coherent radar (§4); not available to EO/thermal |
| **Motion signature** | drones fly straight lines / hover / dash; birds flap-and-glide, wander, thermal-soar | Good, cheap (works from EO or thermal tracks) | a stable track over time |
| **Thermal signature** | drone = compact hot motor cluster; bird = larger, cooler, distributed warm body | **Weak alone** — sun-heated birds/rooftops confound it | shape + context, not just ΔT |
| **ML classifier (EO/IR)** | learned drone-vs-bird appearance | Moderate, but domain-fragile | ≥8–10 px, in-domain training |

Micro-Doppler numbers: [drone-warfare radar](https://drone-warfare.com/counter-uas/radar-detection/),
[Robin Radar](https://www.robinradar.com/blog/how-micro-doppler-radar-works),
[Nature K/W-band drones vs birds](https://www.nature.com/articles/s41598-018-35880-9).

**ML error rates are real and honest:** YOLOv9 hits 95.7% mAP on a clean benchmark, but
the *known→unknown scene* drop (93.55% → **66% AP**) shows how much a fielded classifier
degrades ([drone-warfare EO/IR](https://drone-warfare.com/counter-uas/eo-ir-detection/)).
Sensor-fusion research pushed false alarms "under 10%," but "operational rates vary
significantly by environment" ([drone-warfare radar](https://drone-warfare.com/counter-uas/radar-detection/)).

**The load-bearing correction to ADR-0015:** the council attributed bird-rejection to
**thermal**. The evidence says the real bird-discriminator at these ranges is
**micro-Doppler (radar) + motion-signature/ML**, with thermal a *supporting* cue (hot
compact blob), not the workhorse — because **birds are warm too**. Thermal earns its
place for *night*, not for *bird-rejection*.

---

## 4. Cheap cue layers (radar / acoustic / RF) — be skeptical

A **cue layer** is a cheap, wide-area sensor that says "look over there" so the accurate
(and narrow) EO/thermal tracker doesn't have to search the whole sky. Three candidates:

**Radar (mmWave / FMCW).** *FMCW = frequency-modulated continuous-wave, the cheap
automotive-radar style.* This is the best bird-discriminator (micro-Doppler, §3) and the
only all-weather, day/night option — but **fielded C-UAS radar is not cheap**:
- Entry-level commercial C-UAS radar: **$50,000–200,000**; military-grade $1M–5M
  ([drone-warfare radar](https://drone-warfare.com/counter-uas/radar-detection/)).
- Range on a DJI-class drone: **2–5 km** (X/Ku-band coherent). A small FPV's RCS is
  even lower, so shorter.
- **Cheap DIY path exists but is short-range:** a TI mmWave eval board —
  **IWR6843ISK ~$283, IWR6843AOPEVM ~$196, AWR1642BOOST/IWR1642 ~$392**
  ([DigiKey AWR1642BOOST](https://www.digikey.com/en/products/detail/texas-instruments/AWR1642BOOST/9487449),
  [TI IWR6843ISK](https://www.ti.com/tool/IWR6843ISK)) — collects micro-Doppler on a
  −20 dBsm target only out to **~150–200 m** ([Nature](https://www.nature.com/articles/s41598-018-35880-9)).
  So a ~$300 module can *demonstrate* the bird-discriminating micro-Doppler physics for a
  portfolio, but not field a 2 km cue.

**Acoustic arrays — oversold, confirmed.** Passive microphones listening for prop noise.
- Declared: ZVOOK NW0 **150–450 m** for FPV; **field-confirmed only 200–300 m**
  ([militarnyi](https://militarnyi.com/en/news/ukraine-develops-acoustic-detector-for-fpv-drones/)).
- **Urban / noisy: collapses to 50–200 m**, degrades badly in wind, and "quieter motors/
  props reduce the signature" ([drone-warfare acoustic](https://drone-warfare.com/counter-uas/acoustic-detection/)).
- Verdict: **a cheap close-in last-ditch cue only** — useful passive night alert at ≤200 m,
  never a primary tracker for an intercept that needs metres of accuracy.

**RF / SDR scanners — cheap, long, and DEFEATED by the hard threat.** *RF detection =
listen for the drone's control/video radio and direction-find it.* Passive, cheap, long
range on any *RF-emitting* drone. But:
- **Fiber-optic and fully-autonomous FPV drones emit nothing** — a hair-thin optical
  fibre carries control/video, so "no known EW system can jam a signal inside a fibre,"
  and they are "undetectable by RF sensors" ([ts2](https://ts2.tech/en/fiber-optic-drones-in-ukraine-evolution-applications-and-impact/),
  [JED / NATO](https://www.jedonline.com/2025/05/15/nato-seeks-solutions-for-fiber-optic-fpv-drones/)).
- **NATO (2025): "EW counter-UAS systems are ineffective against this type of drone,"**
  and issued a formal request for new solutions ([JED](https://www.jedonline.com/2025/05/15/nato-seeks-solutions-for-fiber-optic-fpv-drones/)).
- **Prevalence (2025–2026):** still **supply-limited — one Ukrainian unit reported <5%
  fibre** — but ramping hard: **~15 companies, thousands/month**, Russian fibre models
  reaching **20–30 km tethers (80% success at 20 km)** ([ts2](https://ts2.tech/en/fiber-optic-drones-in-ukraine-evolution-applications-and-impact/)).
  The countermeasure everyone converges on is exactly ours: **"spot them via visual/
  acoustic means or radar"** — not RF ([ts2](https://ts2.tech/en/fiber-optic-drones-in-ukraine-evolution-applications-and-impact/)).

**This strongly CONFIRMS ADR-0015's claim #4.** RF is a cheap cue layer only; against the
threat class we most need to beat (RF-silent FPV), it is blind. The whole EO/IR-plus-
onboard-seeker thesis is *motivated* by this fact.

---

## 5. Verdict table (each modality vs OUR 0.3 m FPV target)

| Modality | Range vs 0.3 m FPV (EXPECTED) | Day | Night | Weather | Bird discrim. | Cost (2026) | Emissions/jam | ROLE |
|---|---|---|---|---|---|---|---|---|
| **EO (visible)** stereo pair | recog. **~160 m** (~59 m WORST), detect further w/ long lens | strong | blind | haze/overcast collapse contrast | via motion+ML (fragile) | ~$1.4k rig (ADR-0017) | passive, silent | **PRIMARY tracker (day), our stereo rig** |
| **LWIR thermal** 640 core | few-hundred m; **best at night**, worse hot day | ok | **strong** | fog −30–50%, humidity −20–40%; crossover blind 2×/day | weak alone (birds warm too) | **$3,558 module** + lens ([GroupGets](https://groupgets.com/products/flir-boson-640)) | passive, silent | **CUE LAYER / night add-on (STAGED)** |
| **Radar (coherent, micro-Doppler)** | DJI-class 2–5 km; FPV shorter | yes | yes | **all-weather** | **best (99.9%)** | **$50k–200k** fielded; ~$300 DIY @150 m | active (emits) | **all-weather cue + bird-truth (budget-gated luxury)** |
| **Acoustic** array | **200–300 m** (50–200 m urban) | yes | yes | wind/noise kills it | crude | cheap ($100s–low-$k) | passive | **close-in last-ditch night cue only** |
| **RF / SDR** scanner | long — *if it emits* | yes | yes | fine | n/a | cheap | passive | **cheap cue; DEFEATED by fibre-optic FPV → never primary** |

---

## Implications for OUR design

**1. Does the ADR-0015 staged-thermal decision hold? YES — with two sourced corrections.**
- The **stage-it structure survives intact**: EO-only for the daytime proof/bring-up rig
  (limitation disclosed), thermal added for any night/all-conditions claim. Nothing in
  2025–2026 sources overturns this; it matches the fielded reality that EO is day-only and
  thermal is the only passive night sensor.
- **Correction A (re-attribute bird-rejection):** ADR-0015 justified thermal partly for
  "bird-rejection — the #1 fielded C-UAS failure." The evidence says the real bird
  discriminator at these ranges is **micro-Doppler radar + motion-signature/ML**, not
  thermal — birds are warm, so thermal alone gives a blob, not an ID. Keep thermal for
  **night**; credit **radar/motion/ML** for **birds**.
- **Correction B (don't oversell "all-conditions"):** thermal is a **night enabler, not an
  all-weather solver** — blind at dawn/dusk crossover (twice daily), ΔT-starved on hot
  days, and down 20–70% in fog/humidity. Any "all-conditions" claim needs radar too, and
  even then must disclose crossover.
- **Cost flag:** the Boson 640 module alone is **$3,558**, at/above ADR-0015's upper
  thermal estimate — a night channel roughly **doubles** the EO ground-rig budget. Budget it
  as the real number, not the $1,500 low end.

**2. At what engagement range does the EO-only proof rig honestly work?** Reconciling with
ADR-0017's 59–160 m envelope and the fielded EO data:
- **EXPECTED (good daylight, decent contrast): ~120–160 m** for a *classified, bird-
  rejected* track on a 0.3 m FPV — the range at which it spans ~8–10 px in our 20° FOV.
  This is the number to design engagements around and to put on the resume.
- **BEST (bright target, clear air): out to ~250–440 m detection** — but that is "a speck,"
  not a bird-rejected classification, and needs a narrower lens (less search FOV).
- **WORST (small/side-on target, overcast/dusk, cheap mount): ~59 m** — and remember the
  operational 40%-Pdetect / 700-false-alarms-a-day reality. Below ~60 m the onboard seeker
  is doing almost all the work.
- **Bottom line:** the EO-only rig is an **honest daytime, ~60–160 m demonstration**. It is
  blind at night and unreliable at crossover — say so plainly in the README.

**3. Cheapest credible night path.** In order of cost/credibility:
- **Cheapest that keeps bird-rejection: don't buy stereo thermal — buy ONE mono uncooled
  LWIR core** (Boson-class 320 ≈ $1–1.5k, or 640 ≈ $3.6k for range) co-boresighted with the
  existing EO stereo. Use EO stereo for *day ranging*, thermal-mono for *night
  detection/cueing*; at night the cue's own range model is degraded anyway, so paying to
  duplicate the whole stereo pair in thermal buys little. This adds night for **~$1–3.6k**,
  not the ~$3–7k a thermal *stereo* pair implies.
- **Proof-of-concept only (portfolio, short range): a Lepton-class micro-core (~$200)**
  demonstrates the night *modality* at tens of metres — enough to show the pipeline works,
  not enough to field.
- **The all-weather + true bird-rejection upgrade is radar**, but fielded coherent radar is
  **$50k+**; a **~$300 TI mmWave module** can only *demonstrate* micro-Doppler bird
  discrimination out to ~150 m. Treat radar as a later, budget-gated luxury; keep acoustic
  (~$100s) as a cheap ≤200 m close-in night alert if wanted.
- **Never lean on RF for the hard threat** — fibre-optic FPV defeats it (NATO-confirmed).

---

## Proposed ADR-lite block (for the main session to lift into docs/decisions.md)

> **ADR-00XX — Ground sensor modality (P-1): staged-thermal HOLDS; bird-rejection re-attributed to radar/motion, not thermal**
> - **Context:** P-1 pressure-tested ADR-0015's "thermal mandatory for night/bird-rejection" and "RF defeated by fibre-optic FPV" against 2025–2026 C-UAS sources.
> - **Decision:** Keep the staged plan — **EO-only stereo (ADR-0017) for the daytime proof rig; add one MONO uncooled LWIR core (not thermal-stereo) for a night claim.** Radar (micro-Doppler) is the all-weather + bird-truth upgrade, budget-gated. Acoustic ≤200 m close-in cue only. RF stays a cheap cue, never primary.
> - **Corrections to ADR-0015:** (A) bird-rejection is owned by **micro-Doppler radar + motion/ML**, not thermal (birds are warm — thermal gives a blob); (B) thermal is a **night enabler, not all-conditions** — blind at dawn/dusk crossover, ΔT-starved hot days, −20–70% in fog/humidity; (C) Boson 640 module = **$3,558** (GroupGets), at/above ADR-0015's thermal top end — a night channel ~doubles the ground-rig cost.
> - **Confirmed:** RF-silent fibre-optic FPV defeats RF/EW (NATO 2025: "EW C-UAS ineffective"); the "detect visually/radar, not RF" consensus *is* our thesis.
> - **Honest envelope:** EO-only rig = a **daytime ~60–160 m bird-rejected** demo (WORST 59 m / EXPECTED 160 m, ADR-0017), blind at night/crossover — disclose in README.
> - **Date:** 2026-07-05. (Research: WebSearch/WebFetch, pessimistic tier taken on conflict; sources in docs/ground_modality.md.)
