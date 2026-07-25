# AprilTag placard mount — the target drone's tag carrier (mechanical design)

> **What this is.** `docs/hardware_order_list.md` §E buys a *parts bag* ("Placard mount +
> misc… Rigidly mounts the tag facing the approach", $22) but no *design*. `docs/placard_sizing.md`
> settled **how big** the tag is (0.35 m black-square edge, the carry limit). Nobody has yet
> settled **which way it points, what it does to the aircraft, or how it bolts on.** This doc
> is that design.
>
> **Scope.** Mechanical + aerodynamic + optical-aspect design of the placard and its mount on
> the ORDERED target drone (TBS Source One V6 5" / Kakute H7 stack, `project_state.json`
> `bom_tiers` Tier 1). It does **not** re-open the tag size (settled, `placard_sizing.md`),
> the camera choice (settled, ADR-0078), or the graveyard entry on the tag as a crossing-dash
> terminal seeker.
>
> **Honesty labelling.** Every quantity below is tagged **[MEASURED]** (a logged run or a
> vendor spec), **[DERIVED]** (arithmetic from measured inputs, shown), **[ESTIMATE]**
> (a component mass or coefficient from a catalogue/standard table), or **[ASSUMPTION]** /
> **[HYPOTHESIS]** (must be confirmed — every one of these is repeated in §13 and §10).
> Nothing here is a flight measurement: **no placard has ever flown.** Date: 2026-07-24.

---

## TL;DR — the three decisions

| # | Question | Decision | The number that decides it |
|---|---|---|---|
| **1** | Orientation / how many faces? | **ONE flat panel, vertical, in the target's fore-aft centreline plane — i.e. EDGE-ON to flight, tag facing sideways (beam).** Printed **right-reading on BOTH faces** (covers left and right beams for +~41 g). Mounted on a **15°-indexable disc** so it can be re-set on the ground to 90° (nose-on) for slow passes. | A nose-on panel makes **11.8 N** of drag at 9 m/s — **1.5× the whole aircraft's weight**. Edge-on it makes **0.28 N**. Ratio **42:1** [DERIVED, §4.3]. And **pitch does not foreshorten a beam-facing tag at all** [DERIVED, §3.4]. |
| **2** | Sail — substrate, mass, CG, aero? | **5 mm paper-faced foamboard**, 450×450 mm, two carbon-tube edge spars; **battery moved to the BOTTOM plate** to buy back CG; panel bottom edge only 10 mm above the top plate. | Placard assembly **≈193 g** on a **≈610 g** bare aircraft (**+32 %**); CG rises **+52 mm** and crosses from ~30 mm *below* the rotor plane to ~22 mm *above* it; trim pitch at 9 m/s beam-on **7.0°** (vs 6.7° bare) [DERIVED, §4]. |
| **3** | Attachment? | **Printed PETG index disc + sacrificial "shoe"** bolted to the four existing top-plate standoff holes (no drilling of carbon); the panel's bottom **carbon spar** drops into a 6 mm slot and is held by two rubber bands; the shoe has a designed **1.5 mm frangible web**. All-nylon/aluminium hardware — **no steel, no magnets** (compass). Carry 3 spare shoes. | Frangible release target **60–80 N**: above the worst flight load (**19 N**, §5.4) and far below the frame's failure load [DERIVED + ASSUMPTION]. |

**The one thing that most needs to change elsewhere:** `docs/tripod_test_protocol.md` §4.1/§4.2
puts the crossing standoff at **15–20 m** and the nearest range station at **8 m**, while the
predicted tag decode envelope is **6.0–9.0 m**. As written, **curve (a) would come back ~all
zeros for reasons of geometry, not physics — a FALSE NO-GO on the $740 order.** Full diff-level
recommendation in §11.

---

## 0. Terms, once (the builder is new to this)

- **Incidence angle (θ)** — the angle between the placard's *surface normal* (the direction
  it "faces") and the line of sight to the camera. θ = 0° is looking dead-on at the tag;
  θ = 90° is edge-on (the tag is a line, invisible). This is the single most important
  number in this doc.
- **Aspect angle** — the angle between the *target's nose direction* and the line of sight
  from the camera. Nose-on = 0°, beam (side-on) = 90°, tail-on = 180°.
- **CG (centre of gravity)** — the point the aircraft's mass balances about. On a multirotor
  what matters is where the CG sits relative to the **rotor plane** (the disc the props sweep):
  a CG *below* the rotor plane is mildly self-righting, a CG *above* it makes the aircraft
  more twitchy and lowers the control loop's stability margin.
- **Frangible** — deliberately designed to break first, and cheaply. A frangible mount means
  a crash sacrifices a 30-cent printed part instead of a $33 carbon frame.
- **Dynamic pressure (q)** — `q = ½ρV²`, the "push" of moving air, in pascals. Every
  aerodynamic force in this doc is `q × C × A` for some coefficient C and area A.
- **Trim pitch** — the steady nose-down angle a multirotor must hold to push itself forward
  against drag. `tan(pitch) ≈ drag / weight`.
- **`quad_decimate`** — an AprilTag detector setting that halves the image before looking for
  tags (faster, shorter range). The whole project runs the library default **2.0**;
  **1.0** = full resolution, ~1.5× the range, more CPU (`placard_sizing.md` §3b).

---

## 1. Fixed inputs (do not re-litigate — every one traced)

| Input | Value | Source | Label |
|---|---|---|---|
| AprilTag family / id | `tag36h11`, id 0 | `scripts/m2_detect.py`, BOM §E | MEASURED |
| Tag **black-square** edge | **0.350 m** | `docs/placard_sizing.md` §4 (the carry limit; no room to size down) | MEASURED (decision) |
| Sim decode envelope, 0.5 m tag, `qd=2.0` | `R_decode90 = 12 m` (hard cliff, ~22 px) | `logs/autolabel_sim_20260721T032626Z.csv`; `placard_sizing.md` §3 | MEASURED (sim) |
| Same, `qd=1.0` | `18 m` (~15 px) | same | MEASURED (sim) |
| Camera scaling sim → OV9281 | 0.712 (conservative) / 0.845 (middle) | `docs/camera_paper_check.md` §2 | DERIVED |
| Predicted real `R_decode90` @0.35 m, `qd=2.0` | **5.98 m** (cons.) / **7.10 m** (middle) | `placard_sizing.md` §4 table | DERIVED |
| Same, `qd=1.0` | **8.97 m** (cons.) / **10.65 m** (middle) | same scaling, `18 × scale × 0.7` | DERIVED |
| Money gate | `t_go = (R90 − 1.5)/9 ≥ 0.5 s` ⇒ **`R90 ≥ 6.0 m`** | `scripts/seeker/tripod_score.py`; `placard_sizing.md` §4 | MEASURED (code constants) |
| Target frame (ORDERED) | **TBS Source One V6 5"** — wheelbase **240 mm**, top plate 2 mm, arms 6 mm, stack 30.5/20 mm, standoffs 30/22 mm, frame mass **136.5 g** | vendor spec ([TBS](https://www.team-blacksheep.com/products/product:8547), [Pyrodrone](https://pyrodrone.com/products/tbs-source-one-v5-5inch)) | MEASURED (vendor) |
| Target stack (ORDERED) | Kakute H7 V1.5 + Tekko32 F4 Metal 65A + ECOII 2207 1900KV ×4 + M10Q-5883 + RP1 V2 + CNHL 6S 1500 + 5" tri props | `project_state.json` `bom_tiers[0]` | MEASURED (order) |
| Target autopilot | **ArduPilot**, AUTO waypoint legs, `WPNAV_SPEED ≥ 900` cm/s | `bom_tiers[0]`, constraint `target-is-ardupilot` | MEASURED |
| Interceptor loft (active guidance) | loft **2–4 m**, dive onto a co-altitude target | `docs/intercept_accuracy_levers.md` §"Loft-then-dive" | MEASURED (sim) |
| Stage-0 regime scope | straight-leg / low-speed only; tag is **dead** as a crossing-dash terminal seeker | graveyard (ADR-0076 add #18e), `detection_tracking_methods.md` §B.4 | MEASURED (sim) |

---

## 2. FIRST CORRECTION — the *sheet* is 450 mm, not 350 mm

Everything downstream (mass, area, drag, clearance, the size of the print you order) keys off
the **physical sheet**, and the project has been quoting the **tag** size.

A `tag36h11` image is a **10 × 10 cell grid**: 6×6 data bits + a 1-cell black border + a
1-cell white border. `tag_size` — the number `pupil-apriltags` wants and the number
`placard_sizing.md` settled — measures the **black square**, which is 8/10 of the printed
image. This is stated verbatim in `models/apriltag_target/model.sdf` and is why the sim's
0.625 m plane carries a 0.5 m tag.

```
cell size            = 0.350 m / 8      = 43.75 mm         [DERIVED]
minimum printed sheet = 0.350 m × 10/8  = 437.5 mm         [DERIVED]
recommended sheet     = 450 mm  (50 mm white all round = 1.14 quiet-zone cells)
frontal area A        = 0.450² = 0.2025 m²                 [DERIVED]
```

That white ring is the **quiet zone** — AprilTag's quad detector needs a light margin around
the black border to find the tag's outer edge. 50 mm is a round number that keeps the sim's
own proportions (the sim model is exactly 1.25 × tag size) so the measured sim envelope
transfers without an extra caveat.

**Consequences:**
- The panel (450 mm) is **1.9 × the airframe's wheelbase** (240 mm). It is not an accessory
  bolted to a drone; it is the largest single aerodynamic surface on the aircraft.
- Print ordering: 450 mm does not fit A2 (420 mm short side). Order **A1 (594 × 841 mm)** or a
  US **24"×36"** engineering print, ×2 (one per face), and trim to 450 × 450 mm.
- **The print must be MATTE.** A gloss or laminated finish will specular-reflect the sun
  straight into the camera and wipe out the black/white contrast the decode depends on.
  Gazebo has no glare model, so no sim result covers this. Toner/laser on plain bond is the
  cheap right answer (also more humidity-stable than inkjet). [ASSUMPTION → §10 item 9]

**Contradiction raised:** `project_state.json` `bom_tiers[0]` still titles the line
*"AprilTag tag36h11 placard (~0.3 m print, rigid)"* while its own `why` field says
*"EDGE = 0.35 m"*, and `hardware_order_list.md` §E says *"(~0.25–0.35 m)"*. All three
under-state the physical sheet. **The thing you order and carry is 450 × 450 mm.**

---

## 3. DECISION 1 — orientation and number of faces

### 3.1 Context

A flat fiducial is **directional**: it works over a cone about its normal and is worth nothing
outside it. This project has already paid for that lesson — in sim the AprilTag scored
**0 detections through a 16 m/s crossing dash** (ADR-0076 add #18e), which retired it as a
crossing-dash terminal seeker (graveyard) and scoped it to the straight-leg first-kill
baseline (`detection_tracking_methods.md` §B.4). So the mount's facing is a first-class
design variable, not a detail.

**An important, previously-unrecorded nuance about that sim result** [MEASURED, code-read]:
the sim's tag is placed with **orientation left at identity** — `scripts/m4_intercept.py`
line 1581-1587: *"orientation left at identity (the target model bakes its facing into
geometry)"*. Combined with `models/apriltag_target/model.sdf`'s fixed `0 0 0 0 -1.5708 0`
pose, the sim tag **always faces world −X no matter which way the target flies**. It is
therefore not a faithful model of *any* physical mount — neither a body-fixed one nor a
gimballed one. The 0-detection result is real and the graveyard entry stands (the tag also
had only a ~6 m envelope there), but it does **not** establish what a *body-fixed, beam-facing*
tag would do. That stays a **[HYPOTHESIS]** for the tripod day (§10 item 6), and this doc does
not claim otherwise.

### 3.2 The decode-vs-incidence law (this is the whole argument)

**Physics.** A tag decodes when its black-square span in pixels clears the decoder's threshold
`P_min`. At incidence θ, the tag foreshortens by `cos θ` along one axis, so the limiting span is

```
P = f_px · edge · cos θ / R        and decode needs  P ≥ P_min
⇒  R_decode(θ) = R_decode(0°) · cos θ                                    [DERIVED]
```

**Incidence acts as a pure range multiplier of `cos θ`.** Equivalently, viewing a tag at
incidence θ costs you a factor `1/cos θ` of range.

**Measured support** [MEASURED, sim — `logs/autolabel_sim_20260721T032626Z.csv`]. The
placard-sizing sweep moved the camera sideways *without yawing it*, so for that geometry the
off-boresight bearing **equals** the tag incidence angle. Re-binned:

| incidence θ | gt range R | decode rate (n=12/cell) | effective range `R/cos θ` | cliff at 12 m predicts |
|---:|---:|---:|---:|:--|
| 0.5–8.2° | 1.9–11.9 m | **100 %** (≤11.9 m) | ≈ R | decode ✓ (matches) |
| 0.5–1.1° | 13.9–29.9 m | **0 %** | ≈ R | fail ✓ (matches) |
| 18.5°, 18.8° | 12.5 m | **100 %** | 13.2 m | marginal — decoded (margin 92–100 vs 101 on-axis) |
| 20.7°, 21.1° | 8.4 m | **100 %** | 9.0 m | decode ✓ |
| **32.6°** | **9.4 m** | **100 %** (margin 115) | **11.2 m** | decode ✓ — the strongest point |
| 17.6° | 16.7 m | **0 %** | 17.5 m | fail ✓ (a *range* miss, not an aspect miss) |

All six off-axis placements are consistent with the `cos θ` law. **Limit of the evidence:**
nothing above **32.6°** was ever tested, and all points are static software-render frames.
Beyond 33° the law is **[HYPOTHESIS]** — the AprilTag literature reports detection surviving
to roughly 65–80° incidence when the tag is *large in the image*, but this design operates at
`P ≈ P_min` where any `cos θ` loss immediately breaks decode, so obliquity tolerance from the
literature does **not** transfer. Treat 33° as the edge of knowledge until the tripod day.

*(Second-order, [HYPOTHESIS], not used in any decision: a small deliberate incidence
**improves** planar-pose conditioning — the classic near-frontal AprilTag "flip" ambiguity is
worst dead-on. Decode range is what the money gate scores, and there `cos θ` is a pure cost.)*

### 3.3 The incidence budget — and it is brutally tight

Fold the `cos θ` law into the money gate (`R90_real ≥ 6.0 m`, §1):

```
cos θ_max = 6.0 / R_decode90(0°)                                          [DERIVED]
```

| decoder | camera scaling | `R90(0°)` @0.35 m | **usable incidence cone θ_max** |
|---|---|---:|:--|
| `qd = 2.0` (deployed) | conservative | 5.98 m | **0°** — already fails the gate dead-on |
| `qd = 2.0` (deployed) | middle (realistic) | 7.10 m | **±32.3°** |
| `qd = 1.0` (full-res) | conservative | 8.97 m | **±48.0°** |
| `qd = 1.0` (full-res) | middle | 10.65 m | **±55.7°** |

**➡️ The mount is designed to the ±32° cone** (deployed decoder, realistic scaling). That is
the honest working budget, and it is *nearly consumed by ordinary flight*: target bank in a
gust (±10°), interceptor elevation above the target from the 2–4 m loft (+12–17° at 8–12 m
range), and crab angle in a crosswind (18° at a 3 m/s crosswind on a 9 m/s leg) already sum
past it if they stack.

**Therefore two mitigations are load-bearing, not optional:**
1. **Every degree of incidence the mount can avoid is worth real money.** That drives §3.4.
2. **`quad_decimate = 1.0` is the cheapest way to buy cone**, widening ±32° → ±48–56°.
   The tripod day can score this for **free** — same frames, re-decoded offline (§11 item 9).

### 3.4 The decisive geometric fact: pitch does not foreshorten a beam-facing tag

Put the panel in the target's fore-aft vertical plane, so its normal points out the side
(along body **y**). Then:

- **Pitch** is a rotation about the body y-axis. A rotation about y leaves ŷ *unchanged*:
  `R_y(θ)·ŷ = ŷ`. **The panel's normal does not move at all when the target pitches.** The tag
  merely appears *rotated in the image*, and AprilTag is invariant to in-plane rotation by
  construction. **Incidence cost of pitch: exactly zero.** [DERIVED]
- **Roll (bank) φ** tilts the normal out of horizontal by φ → incidence φ. A 20° bank costs
  `1 − cos20° = 6 %` of range. Cheap.
- **Yaw / crab** rotates the normal in azimuth 1:1 → the expensive axis. Manage with the
  briefed leg heading and the crosswind limit (§8).

Compare a **nose-on** panel (normal along body x): pitch rotates it 1:1, so the target's own
trim pitch is charged straight against the incidence budget. This is the mirror image of the
project's known *interceptor*-side pointing wall (dash pitch swinging ~+40° to −35°,
ADR-0076 add #18j-fix/#18k) — and here it is avoidable for free by choosing the axis.

### 3.5 Options considered

| Option | Frontal area at 0° yaw | Panel mass | Verdict |
|---|---:|---:|---|
| **(A)** one face, **nose-on** (vertical, facing forward) | 0.2025 m² | 152 g | **REJECT** — 11.8 N drag = 1.5× aircraft weight; ~48° trim at 9 m/s; caps the target at 5.8 m/s; and the 48° trim itself throws the tag *outside* the ±32° cone. It defeats itself (§4.3). |
| **(B)** one face, **beam** (edge-on), single-sided | ~0.0023 m² | 111 g | Good aero, but only one side of the aircraft is usable → the leg direction and the tripod side must match every time. |
| **(C)** **one panel, beam, tag printed on BOTH faces** | ~0.0023 m² | 152 g | **✅ CHOSEN.** +41 g and +$5 buys left *and* right beam coverage with **zero** extra area. |
| **(D)** two panels back-to-back, fore + aft | 0.2025 m² | 304 g | **REJECT** — a flat plate's drag does not care which face is painted; you get option (A)'s aero disaster at double the mass, and still nothing at the beam. |
| **(E)** angled **V / dihedral** (two panels canted ±ψ about vertical) | `2A·sinψ` — 0.20 m² at ψ=30° | 304 g | **REJECT** — any cant big enough to matter (≥30°) recreates the full drag of (A). Aerodynamically, "cover more azimuth" and "keep the frontal area small" are the same trade. |
| **(F)** triangular **prism**, 3 faces, 360° azimuth | ≥0.20 m² always | 456 g | **REJECT** — 456 g of panel on a 610 g aircraft (75 %), and *some* face is always normal to the flow. Physically absurd on a 240 mm quad. |
| **(G)** **horizontal** panel facing up or down | ~0.0023 m² (edge-on) | 152 g | **REJECT as primary** — (i) it sits in the rotor inflow/downwash and would cost real thrust; (ii) with the interceptor's 2–4 m loft the elevation to the target at 8–12 m is only **12–17°**, so an up-facing tag would be viewed at **73–78° incidence** = dead; (iii) it cannot clear the props without a huge standoff. Revisit only if loft ever grows past ~45° dive angles. |
| **(H)** beam panel with a build-time **yaw cant** of 10–15° forward | `A·sinψ` — 0.035 m² at 10° | 152 g | **KEEP AS AN OPTION**, not the default: a 10° cant costs **18.4°** of trim pitch at 9 m/s (vs 7.0° at 0°) to buy 10° of aspect shift; 15° costs 23.8°; `ANGLE_MAX` is reached at a **22.3°** cant [DERIVED]. The index disc (§5) makes it a 2-minute ground change if the tripod day says the aspect band needs shifting — but note the trim penalty is charged in *pitch*, which the beam mount otherwise gets for free. |

### 3.6 Decision + why

> **DECISION 1.** A **single flat 450 × 450 mm panel, held vertically in the target's fore-aft
> centreline plane** (edge-on to flight; tag normal points out the left and right beams), with
> **`tag36h11` id 0 printed right-reading on both faces**. It mounts on a **15°-indexable
> disc** so its yaw can be re-set on the ground to 0° (beam, default), ±15/30/45/60/75°, or
> 90° (nose-on) — the 90° setting is **restricted to ≤4 m/s airspeed** by §8.

**Why:**
1. **Aero.** Edge-on the panel adds **0.28 N** of drag at 9 m/s; nose-on it adds **11.8 N**.
   Only the edge-on orientation lets the target fly its briefed **≥9 m/s** leg at all (§4.3).
2. **Incidence.** Pitch — the target's largest and least controllable attitude excursion —
   costs the beam mount **nothing** (§3.4). The ±32° budget is too tight to spend on trim.
3. **Both faces for free.** Two prints on one panel = ±beam coverage at zero aerodynamic cost;
   the tripod does not have to be on a pre-agreed side and the leg can be flown either way.
4. **It is reversible.** The index disc keeps every other aspect a 2-minute ground change,
   so no field-day question is foreclosed by the build.

**Build note that will bite you if missed:** print the tag **right-reading on each face
independently** — do *not* photocopy one sheet and flip it. A mirrored `tag36h11` is not a
valid member of the family and will simply fail to decode.

**What this decision costs:** with the placard at 0° (beam), a target flying **straight at the
tripod** presents the tag at 90° incidence and yields **zero decodes**. That is a real
limitation and it drives the protocol changes in §11 — including that the auto-labeller
(`autolabel_from_apriltag.py`), which produces the NN training labels, can only label frames
where the tag decodes.

*(Rejected sub-idea, for the record: a small second tag on the nose to auto-label approach
frames. A 100 mm tag decodes to `7.10 × 0.10/0.35 = 2.0 m`; even 150 mm only reaches 3.0 m
[DERIVED]. Useless. Approach-aspect labels have to come from the 90° index at low speed.)*

---

## 4. DECISION 2 — it is a sail: substrate, mass, CG, aerodynamics

### 4.1 Substrate options and the mass arithmetic

Areal densities are catalogue/vendor typicals — **[ESTIMATE]**, and §9 makes weighing the
finished panel a bench gate, because these vary ±30 % by brand.

| substrate | areal density | panel mass (0.2025 m²) | notes |
|---|---:|---:|---|
| **5 mm paper-faced foamboard** | 340–485 g/m² ([Flite Test](https://www.flitetest.com/articles/comparing-foam-board), [printmoz](https://www.printmoz.com/blog/foam-core-board-sizes)) → design **450** | **91 g** | ✅ the BOM's own item; stiff sandwich; crushes before the airframe does (frangible); any craft/office store |
| 3 mm XPS / Depron + paper facing | ~100 g/m² | 20 g | lightest by far, but floppy over a 450 mm span — needs full edge + diagonal spars. **The weight-optimised fallback if §9's hover check is unhappy.** |
| 4 mm coroplast (corrugated PP) | 600–1200 g/m² ([US Plastic](https://www.usplastic.com/catalog/item.aspx?itemid=66017)) → 700 | 142 g | toughest/weatherproof but +56 % mass and it *survives* crashes, meaning the frame absorbs the energy instead. Anti-frangible. **Reject.** |
| 3 mm foam PVC (Sintra) | 1500–1800 g/m² | 300–365 g | 60 % of the airframe mass in one part. **Reject.** |
| printed fabric / vinyl banner | 110–450 g/m² | 22–91 g | **Reject on optics, not mass:** a flexible face flaps, and a non-planar tag breaks the homography the decoder and the pose solve both assume. Only viable on a rigid tensioned frame, which costs back the mass saved. |
| any **metallised / foil-faced** board | — | — | **HARD REJECT** — see §5.5 (GPS / RX). |

**Chosen: 5 mm paper-faced foamboard.**

**Placard assembly mass** [ESTIMATE, itemised]:

```
foamboard 450×450×5, @450 g/m²                            91 g
2 × printed sheets, 100 g/m² bond (one per face)          41 g
spray adhesive (2 faces)                                  12 g
2 × carbon tube spar Ø4 × 0.5 wall × 450 mm (top+bottom)   8 g
                                                   panel = 152 g
printed index disc (PETG, Ø75 × 4)                         8 g
printed shoe (PETG, ~110×40×24)                           22 g
4 × M3×10 nylon screws + 2 × M3×10 nylon                   4 g
2 × #64 rubber bands + hook-and-loop strap                 3 g
misc (spar end caps, adhesive)                             4 g
                                                   mount = 41 g
                                       ►  PLACARD TOTAL = 193 g
```

### 4.2 Aircraft mass and CG

Component masses are vendor/catalogue typicals — **[ESTIMATE]**; §9 makes weighing the
finished aircraft a bench gate. Heights `z` are measured **from the top plate's upper
surface**, positive up, and are **[ASSUMPTION]** pending a tape measure on the built airframe.

| item | mass | z (mm) | note |
|---|---:|---:|---|
| Frame (Source One V6, incl. hardware) | 136.5 g | −20 | vendor spec [MEASURED] |
| 4 × ECOII 2207 1900KV + 4 × 5" tri props | 146 g | −20 | |
| Kakute H7 + Tekko32 F4 Metal | 36 g | −18 | |
| M10Q-5883 GNSS/compass | 12 g | +45 | short rear mast, offset beside the panel (§5.5) |
| RP1 V2 RX, wiring, XT60, strap, buzzer, microSD | 30 g | −15 | |
| CNHL 6S 1500 mAh | 250 g | **−48** | **moved to the BOTTOM plate** — see below |
| **Bare AUW** | **610.5 g** | **−29.8 mm** | W = **5.99 N** |
| Placard panel + spars | 152 g | +235 | area centre, bottom edge at +10 |
| Placard mount (disc, shoe, hardware) | 41 g | +10 | |
| **AUW with placard** | **803.5 g** | **+22.3 mm** | W = **7.88 N** (7.90 used below) |

```
CG_bare    = Σmz/Σm = −18193 / 610.5  = −29.8 mm
CG_placard = (610.5·(−29.8) + 193·187) / 803.5 = +22.3 mm
                                     ►  CG RISE = +52.1 mm                  [DERIVED]
```

**The battery moves to the bottom plate.** On a stock Source One the LiPo straps to the *top*
plate — exactly where the placard shoe must go. Moving it underneath (a standard Source One
configuration) does three jobs at once: it frees the top plate, it drops ~250 g by ~50 mm
which buys back most of the placard's CG rise, and it keeps the strap accessible.
Cost: less ground clearance — fit a 3 mm TPU or foam pad under the pack.

**Why the CG rise matters.** The rotor plane on this airframe sits at roughly `z ≈ 0 ± 5 mm`
(2207 motors on 6 mm arms under a 30 mm-standoff top plate) — **[ASSUMPTION], measure it**.
So the placard flips the CG from **~30 mm below** the rotor plane to **~22 mm above** it. A
CG above the rotor plane is flyable (plenty of camera-on-top quads do it) but it reduces the
attitude loop's phase margin. **Hard rule: fit the placard, then run ArduPilot Autotune in
calm air, and let `MOT_THST_HOVER` re-learn in a Loiter hover. Never fly an AUTO mission on a
tune taken bare.**

**Longitudinal CG is unchanged** if the panel is centred fore-aft — and it must be, see §4.5.

### 4.3 Drag: the number that decides everything

```
ρ = 1.225 kg/m³ (ISA sea level, 15 °C)             [ASSUMPTION — adjust for field elevation]
q(V) = ½ρV²  →  q(4) = 9.8 Pa   q(9) = 49.6 Pa
C_d, flat square plate normal to flow = 1.17       [ESTIMATE — standard table, Hoerner; AR=1]
A = 0.2025 m²
```

**Nose-on (panel normal to the flow), at the briefed 9 m/s:**

```
D = q·C_d·A = 49.6 × 1.17 × 0.2025 = 11.75 N = 1.20 kgf                   [DERIVED]
```

That is **1.5 × the whole loaded aircraft's weight (7.90 N)**. Sanity check by feel: holding
an 18-inch square board out of a car window at 20 mph is ~2.6 lbf — which is exactly 1.20 kgf.
The number is right.

**Edge-on (beam), at 9 m/s:**

```
edge bluff drag:   q · C_d,edge · (0.005 × 0.450) = 49.6 × 1.8 × 0.00225 = 0.20 N
skin friction:     C_f · q · S_wet = 0.004 × 49.6 × 0.405              = 0.08 N
                                                    added drag ≈ 0.28 N    [DERIVED]
```

**Ratio 11.75 / 0.28 = 42 : 1.** That single number is Decision 1.

**Bare-aircraft drag reference** [ASSUMPTION, calibrated]: take `(C_d·A)_quad = 0.014 m²`,
back-solved from the common observation that a 5" quad trims ~30° nose-down at 20 m/s
(`D = W·tan30° = 3.4 N` at `q = 245 Pa`). At 9 m/s that gives `D_quad = 0.69 N` and a bare
trim pitch of **6.7°**, which matches how a 5" quad actually cruises — a good sanity check.
Note the placard term dominates it **17 : 1** nose-on, so none of the conclusions below are
sensitive to this assumption.

### 4.4 Trim, and the speed cap the mount imposes

The panel is rigid to the airframe, so pitching the nose down *also* rotates the panel out of
the flow. Using a flat-plate normal-force model (`F_N = C_d·q·A·cos²θ` acting along the plate
normal, which resolves to `F_N·cosθ` of drag and `F_N·sinθ` of lift), steady level flight at
nose-down pitch θ satisfies:

```
horizontal:  T·sinθ = D_quad + F_N·cosθ
vertical:    T·cosθ + F_N·sinθ = W = 7.90 N
```

Solving numerically [DERIVED]:

| configuration | airspeed | trim pitch θ | tag incidence charged to the budget |
|---|---:|---:|---|
| **beam (0° index)** | 9 m/s | **7.0°** | **0°** — pitch is free (§3.4) |
| beam (0° index) | 4 m/s | 1.4° | 0° |
| nose-on (90° index) | 4 m/s | **16.6°** | 16.6° ⇒ `R90` = 6.80 m — **inside** the gate |
| nose-on (90° index) | 5.83 m/s | **30.0° = `ANGLE_MAX`** | 30.0° ⇒ `R90` = 6.15 m — at the gate, zero margin |
| nose-on (90° index) | 6 m/s | **31.2°** | 31.2° ⇒ `R90` = 6.08 m — outside `ANGLE_MAX`, on the gate |
| nose-on (90° index) | 9 m/s | **47.5°** | 47.5° ⇒ `R90` falls to **4.79 m** ⇒ **money gate FAILS** |

**Maximum level speed, nose-on, at ArduPilot's default `ANGLE_MAX = 3000` (30°)**: solving the
same pair with θ fixed at 30° gives `q = 20.8 Pa` ⇒ **V_max = 5.8 m/s** [DERIVED]. **The
briefed ≥9 m/s AUTO leg is simply unreachable with the panel nose-on.** ArduPilot will not
fail — it will quietly cap the leg speed at the lean-angle limit and the pass will not be the
full-speed pass you logged it as.

**And you cannot buy your way out with `ANGLE_MAX`.** At `ANGLE_MAX = 45°` the nose-on config
reaches 8.4 m/s — but at 45° pitch the incidence is 45°, `R90` drops to **5.02 m**, and the
money gate fails anyway. **Raising the lean limit trades speed for incidence one-for-one.**
[DERIVED]

Notice the two failure modes converge at almost the same speed: the nose-on config hits
`ANGLE_MAX` at **5.83 m/s** and hits the decode gate (`R90` = 6.0 m) at **~6.0 m/s**. That
coincidence is not luck — both are the same `cos θ` of the same trim angle.

> **Operating rule that falls straight out:** the **90° (nose-on) index is a ≤4 m/s
> configuration** (16.6° trim, `R90` = 6.80 m, real margin), usable only for the protocol's
> slow control passes and for gathering approach-aspect auto-label frames. Every full-speed
> pass flies at the **0° (beam)** index.

### 4.5 Crosswind, side force, and the yaw/roll moments

Edge-on to the *flight* direction still means broadside to a **crosswind**. Only the wind
component normal to the panel matters:

```
S = q_cross · C_d · A = 0.2369 · q_cross          [DERIVED]
bank demand to hold track:  φ = atan(S / W)
```

| crosswind component | side force S | bank demand φ | roll moment about CG (arm 212 mm) |
|---:|---:|---:|---:|
| 2 m/s | 0.58 N | 4.2° | 0.12 N·m |
| **3 m/s (design limit)** | **1.31 N** | **9.4°** | **0.28 N·m** |
| 5 m/s | 3.63 N | 24.7° | 0.77 N·m |
| **5.6 m/s** | 4.56 N | **30° = `ANGLE_MAX` — track hold is lost** | 0.97 N·m |

Roll authority for reference [ESTIMATE]: 4 × 2207 motors at ~78 mm roll arm with ~13 N of
available differential thrust ⇒ **~2.0 N·m**. So the 3 m/s case uses ~14 % of roll authority —
comfortable; the binding constraint is the **bank-angle/track-hold limit at ~5.6 m/s**, not
roll power.

**Yaw is the tighter one.** A 0.2 m² vertical surface is a *fin*. Its side force acts at the
panel's area centre; if that sits **forward** of the CG the aircraft is directionally
divergent and the (weak) yaw controller fights it continuously.

```
yaw moment N = S × (x_offset)
at 3 m/s crosswind, x_offset = 20 mm  →  N = 0.026 N·m
estimated quad yaw authority (2207/6S, prop torque ≈ 0.012 m × thrust) ≈ 0.10–0.15 N·m
                                                   →  N is ~20 % of authority [ESTIMATE]
```

> **Design rule: keep the panel's area centre within ±20 mm of the CG in x, and bias AFT
> rather than forward.** Aft is weathervane-stable (it turns the nose into the relative wind,
> which is also the direction that zeroes the side force). Forward is divergent. At a 60 mm
> forward offset and a 5 m/s crosswind the yaw moment is 0.22 N·m — past the estimated
> authority. This is a real "get it wrong and it won't fly straight" number.

**Crab and the incidence budget.** In AUTO, ArduPilot points the nose along the *ground track*
(`WP_YAW_BEHAVIOR` default), not into the relative wind — so a 3 m/s crosswind on a 9 m/s leg
leaves the aircraft flying with ~18° of sideslip, which rotates the panel normal by 18° and
spends more than half the ±32° cone. This is why the crosswind limit in §8 is an *optical*
limit as much as a handling one.

### 4.6 Prop wash, flutter, and endurance

- **Rotor interaction** [ESTIMATE]. A *vertical* panel on the centreline sits parallel to the
  rotors' inflow, so blockage is small — this is the second reason the horizontal option (G)
  was rejected (a horizontal plate under the discs would intercept the full downwash;
  momentum theory gives an induced velocity of ~7.5 m/s / ~138 Pa disc loading for this
  aircraft, i.e. a download that could eat a large fraction of thrust). Expect a modest hover
  power penalty from the vertical panel, but **measure it** (§9).
- **Flutter is a real risk.** A 450 mm panel gripped only by a 110 mm shoe is a 170 mm
  cantilever each end. A flapping panel (i) is a non-planar tag — it breaks the decoder's
  homography — and (ii) pumps vibration into the FC, degrading the EKF and the DataFlash log
  the whole test depends on. **Mitigation: bond a Ø4 mm carbon tube spar into a slit along
  the full bottom edge AND the full top edge.** The bottom spar is also the load path into
  the shoe, so the foam never carries a mount load. Add a third vertical centreline spar if
  §9's shake test shows motion.
- **Endurance** [DERIVED]. Thrust ratio 803.5/610.5 = 1.316; induced power scales as `T^1.5`
  ⇒ **+51 % induced power**, perhaps +35–40 % total electrical. **Expect 25–35 % shorter
  flights.** `tripod_test_protocol.md` §4.6 budgets ~28 passes across 3 packs — with the
  placard that is optimistic. **Plan 4–5 packs, or drop the "best-effort" rows.**

### 4.7 Decision + why

> **DECISION 2.** **5 mm paper-faced foamboard**, 450 × 450 mm, two Ø4 mm carbon edge spars,
> prints spray-mounted both faces (**152 g**). **Battery relocated to the bottom plate.**
> Panel bottom edge **+10 mm** above the top plate (as low as prop clearance allows), area
> centre on the CG in x (±20 mm, bias aft). Total added **193 g / +32 % AUW**, CG **+52 mm**.

**Why:** foamboard is the BOM's own part, the stiffest option per gram at this span, and
*correctly frangible* — it crushes and tears in a crash instead of transmitting the load into
$33 of carbon. Coroplast survives crashes, which is exactly wrong. Mounting the panel as low
as possible and the battery as low as possible are the two levers that keep the CG rise to
+52 mm instead of +80 mm, and they cost nothing. If §9's hover/vibration check comes back
unhappy, the pre-planned fallback is the **3 mm XPS + full spar set** panel (78 g): placard
assembly **119 g**, AUW **729.5 g**, CG **+0.7 mm** — a **+30.6 mm** rise instead of +52.1 mm,
which lands the CG essentially *on* the rotor plane rather than above it [DERIVED]. Keep the
XPS sheet and a spare spar set in the field box; it is the cheapest single fix if the aircraft
feels twitchy.

---

## 5. DECISION 3 — the attachment scheme

### 5.1 Requirements

1. No drilling or cutting of the carbon frame (it is the ordered part and the spare is $20).
2. Yaw-indexable on the ground in ≤2 minutes without tools beyond a 2 mm hex driver.
3. **Frangible** — a hard landing sacrifices a printed part, never the frame, motors, or arms.
4. Non-magnetic and non-ferrous throughout (the M10Q-5883 carries the compass).
5. Load path through the panel's spar, never through the foam.
6. Leaves the battery strap, the USB/microSD ports, the buzzer, and the arming/bind buttons
   reachable.

### 5.2 The parts

| # | part | material | mass | source |
|---|---|---|---:|---|
| P1 | **Index disc**, Ø75 × 4 mm, 4 × M3 clearance on the frame's top-plate standoff pattern, 24 × M3 holes on a Ø56 mm circle (15° steps), keyed centre boss | PETG (print) | 8 g | print — `placard_index_disc.stl` |
| P2 | **Shoe**, ~110 × 40 × 24 mm: 6.0 mm × 110 mm slot 18 mm deep, two spar saddles, two rubber-band hooks, **1.5 mm frangible web** at the slot base | PETG (print) | 22 g | print — `placard_shoe.stl` (**×4: 1 + 3 spares**) |
| P3 | **Panel**, 450 × 450 × 5 mm foamboard, tag both faces | foamboard | 152 g | $22 misc bag (foamboard, spray adhesive) + 2 copy-shop prints (~$10 line) |
| P4 | 2 × Ø4 × 0.5 mm wall carbon tube, 480 mm (protrudes 15 mm each side of the bottom panel edge) | carbon | 8 g | **NOT in the misc bag — ~$4 gap, add it** |
| P5 | 4 × M3 × 10 **nylon** screws (disc → frame standoffs) | nylon | 2 g | $22 misc bag ("standoffs") |
| P6 | 2 × M3 × 10 nylon screw + nut (shoe → disc, sets the index angle) | nylon | 2 g | same |
| P7 | 2 × #64 rubber bands + 1 hook-and-loop strap (retention) | — | 3 g | $22 misc bag ("zip ties, strap") |
| P8 | 2 × spar end cap | PETG (print) | 2 g | print — `placard_spar_cap.stl` |
| P9 | 3 mm TPU/foam pad under the relocated battery | TPU/foam | 4 g | $22 misc bag |

**Mapping to the $22 "Placard mount + misc" line** (`hardware_order_list.md` §E): foamboard ✓,
adhesive ✓, standoffs/nylon hardware ✓, zip ties ✓, strap ✓. **Gap: the two carbon spars
(~$4) and the PETG filament are not in the bag** — the XT60 pigtail and the lost-model buzzer
in that line are unrelated to the mount. Call the bag ~$26 and it covers the design.

### 5.3 Assembly

1. Move the LiPo to the bottom plate on the frame's existing strap slots, over the TPU pad.
2. **P1** bolts to the four top-plate standoff positions with **P5** (longer nylon screws into
   the frame's existing 30 mm standoffs — no new holes).
3. **P2** bolts to **P1** through any diametrically-opposed pair of the 15° holes. That bolt
   pair *is* the index setting. Mark 0° on the disc with a paint pen so a field re-index
   cannot be got wrong.
4. Slit the panel's bottom edge, epoxy **P4** in flush; repeat along the top edge; cap with
   **P8**.
5. Drop the bottom spar into the shoe's saddles; the panel's lower 18 mm sits in the slot.
   Loop **P7**'s two rubber bands over the spar into the shoe hooks; add the hook-and-loop
   strap as a secondary.

### 5.4 Frangibility — how the break load was chosen

```
worst steady flight load on the panel   = max side force S(5 m/s crosswind)   ≈  3.6 N
worst inertial load (193 g at 10 g)                                            ≈ 19 N
                    ⇒ release must be well ABOVE ~19 N   ... and well BELOW the frame's limit
                    ⇒ TARGET RELEASE 60–80 N                        [DERIVED + ASSUMPTION]
```

Two independent frangible paths, both tuned to that band: the **rubber bands** stretch off the
spar (release load set by band count — a bench pull test with a luggage scale sets it,
§9 item 7), and the shoe's **1.5 mm web** snaps. Either way the panel departs and the frame is
unloaded. Shoes are ~$0.30 of filament; print four.

**Explicitly rejected: magnetic breakaway.** Neodymium magnets 100 mm from an M10Q-5883
magnetometer would corrupt the compass and therefore the AUTO mission's heading. Not a
close call.

*(A recovery tether was considered and rejected for flight: a released panel on a leash
becomes a flailing 150 g pendulum. Instead, write the aircraft's phone number on the panel
and rely on the lost-model buzzer.)*

### 5.5 Clearances and obstruction — the checks nobody has done

**Props.** For a 240 mm true-X with 5" (127 mm) props the motors sit at `±120·cos45° = ±84.9 mm`
in x and y, so each prop's inner edge is at `84.9 − 63.5 = 21.4 mm` from the centreline:

```
clear corridor at y = 0:  ±21.4 mm      panel half-thickness: 2.5 mm
                       ⇒  clearance ≈ 18.9 mm per side                        [DERIVED]
```

So a centreline panel is clear of the prop discs **laterally** by ~19 mm even at prop height —
which is why the low +10 mm mounting is possible at all. **[ASSUMPTION — the Source One V6 is
a "wide-stance X", so the true x/y motor offsets may not be the symmetric 45° values. MEASURE
prop-tip-to-centreline on the built aircraft, and MEASURE the rotor plane's height relative
to the top plate. If the rotor plane turns out to sit ABOVE the top plate, raise the panel's
bottom edge to 25 mm above the disc and re-run the CG in §4.2.]** This is the target-side
analogue of the interceptor's `prop-clearance` hard gate and deserves the same bench
corner-spin treatment at loaded throttle.

**GPS / compass (Matek M10Q-5883).** A 450 mm sail on the centreline will shadow a top-mounted
GNSS antenna from part of the sky *if the panel is opaque at L-band*. The physics says it is
not: paper + polystyrene foam are thin low-loss dielectrics, and fused toner is carbon
dispersed in an insulating polymer at ~10 µm — expected loss at 1.575 GHz well under 1 dB.
**[HYPOTHESIS — and it is the single highest-value, zero-cost bench check in this document
(§9 item 3).]** If it turns out opaque, the geometric shadow is severe: with the GPS at
+45 mm and 45 mm off-centre, the panel blocks the band from **~38° up to ~84° elevation**
across **±79° of azimuth** on one side — call it a fifth to a quarter of the visible sky
[DERIVED]. Fallbacks in order: (a) a 150 mm rear mast canted
outboard; (b) accept degraded HDOP and re-verify the AUTO leg repeats. **Hard rule either
way: no foil, no metallised board, no carbon sheet in the placard.**

**Compass, separately:** all placard hardware is nylon/PETG/carbon, so there is no hard-iron
contribution — but **calibrate the compass with the placard fitted, in its flight index
position**, and re-calibrate if you change the index. (Carbon tube is conductive; keep the
spars ≥50 mm from the magnetometer, which the +10 mm bottom-edge geometry does naturally if
the GPS is on a rear mast.)

**ELRS RX antennas (RadioMaster RP1 V2, 2.4 GHz).** Same dielectric argument, same
**[HYPOTHESIS]** status, bench-checked the same way (§9 item 4). Rules: keep both dipoles at
the rear, below the panel's bottom edge, at 90° to each other, pointing down-and-out; **never
zip-tie an antenna to the panel or route it along the spar.**

**Ground handling / tip-over — this is the tightest wind limit in the whole design.** With the
CG at ~+22 mm the aircraft's tip-over angle drops from ~76° to ~55°, and a sitting aircraft has
no way to bank into a gust. Wind on the 0.2 m² sail acts ~0.30 m above the ground against a
restoring moment of only `W × ~100 mm footprint = 0.79 N·m`:

```
tips when  1.17 · q · A · 0.30 > 0.79 N·m  ⇒  q > 11.1 Pa  ⇒  V_wind > 4.3 m/s   [DERIVED]
(4 m/s: 0.70 vs 0.79 N·m — marginal.  5 m/s: 1.09 vs 0.79 N·m — over.)
```

> **Do not leave the target standing on the ground with the placard fitted in >3 m/s wind**
> (4.3 m/s is the calculated tip point; 3 m/s is that with margin). Fit the panel **last**,
> immediately before arming; between passes lay the aircraft on its side or hold it.
> A tipped-over target on a hard field costs a GPS mast, an antenna, or a prop.

### 5.6 Decision + why

> **DECISION 3.** **Printed PETG index disc + sacrificial shoe**, bolted to the four existing
> top-plate standoff holes; the panel's bottom carbon spar drops into a 6 mm slot and is
> retained by two rubber bands over a 1.5 mm frangible web; all-nylon/aluminium/carbon
> hardware, **no steel and no magnets**; battery relocated underneath; 3 spare shoes carried.

**Why:** it adds no holes to the ordered frame, it makes the aspect question (the one thing
this design is *least* certain about, §3.3) a two-minute field change instead of a rebuild,
and it puts a 30-cent part in the crash load path of an aircraft the BOM itself describes as
*"the target takes the hard landings."* The all-non-magnetic rule protects the compass the
whole AUTO-repeatability story depends on.

---

## 6. Dimensioned sketch

**SIDE VIEW** — looking at the target's left side, nose to the right. Datum `z = 0` is the top
plate's upper surface.

```
                          |<------------- 450 mm (fore/aft) ------------->|
              z = +460 mm  ______________________________________________
                          |   top spar: Ø4 x 0.5 carbon, bonded in slit   |
                          |  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  |   ← 50 mm white
                          |  ░░ +---------------------------------+ ░░░░  |     quiet zone
                          |  ░░ |  # ##  ###   tag36h11  id 0     | ░░░░  |     (1.14 cells)
                          |  ░░ |  ## #   ##   350 mm BLACK-      | ░░░░  |
                          |  ░░ |   # ###  #   SQUARE EDGE        | ░░░░  |   450 mm tall
                          |  ░░ |  ###  ## #   (43.75 mm/cell,    | ░░░░  |
                          |  ░░ |   ## #  ##    10 x 10 cells)    | ░░░░  |
                          |  ░░ +---------------------------------+ ░░░░  |
                          |  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  |
              z = +10 mm  |________________|___|_________________________|
                                           | |  panel: 5 mm foamboard,
                                           | |  TAG PRINTED ON BOTH FACES
                                           | |  bottom spar Ø4 carbon (480 mm,
                                           | |  15 mm proud each side) in saddles
                            ===============|===|===============  SHOE (P2), PETG
                            ~~ 1.5 mm FRANGIBLE WEB ~~  + 2x rubber-band hooks
   z = 0  TOP PLATE  ======#####################################======  2 mm carbon
   (2 mm CF)                     INDEX DISC (P1) Ø75 x 4, 24 holes @15°
                            ||                                   ||
   30 mm frame standoffs    ||                                   ||
                            ||        GPS mast, rear, 45 mm       ||
   arms / rotor plane  =====##====== off-centre, z = +45 =========##=====   z ≈ 0 ± 5 mm
   (MEASURE THIS)                                                          [ASSUMPTION]
   bottom plate        =============================================
                          [######## CNHL 6S 1500, UNDERSLUNG ########]   z ≈ −48 mm
                                    on 3 mm TPU pad (P9)
```

**FRONT VIEW** — looking at the nose. The panel lives in the clear corridor between the left
and right prop pairs.

```
                              panel, 5 mm thick, edge-on
                                        ||
        prop disc (5", Ø127)            ||            prop disc
    (((((((((((((((((()                 ||                 ())))))))))))))))))
                      |<-- 21.4 mm -->| || |<-- 21.4 mm -->|
                       clear corridor   ||   clear corridor
                       (18.9 mm to the panel face each side)      [DERIVED]
    ==========================#####################==========================
                                  SHOE + INDEX DISC            top plate
                        [############ battery ############]    bottom plate
```

**TOP VIEW** — the index positions.

```
                              NOSE  (0° = flight direction)
                                     ^
                                     |
                                     |
          0° INDEX (DEFAULT):   ======|======   panel along the centreline,
          tag faces LEFT and RIGHT beams        edge-on to flight, 450 mm long
                                     |
                                     |
          90° INDEX:            -----+-----     panel ACROSS the airframe,
          tag faces NOSE-ON                     ***≤ 4 m/s ONLY*** (§8)
                                     |

          available index steps: 0, ±15, ±30, ±45, ±60, ±75, 90°
          area centre must stay within ±20 mm of the CG in x, biased AFT  (§4.5)
```

---

## 7. Build-it checklist

**Order / gather**
- [ ] 2 × copy-shop prints, **A1 or 24"×36", MATTE toner**, of `tag36h11` id 0 rendered so the
      **black square measures exactly 350.0 mm** (the full 10-cell image is 437.5 mm; centre it
      on the sheet). Verify with a ruler on the print before you cut anything.
- [ ] 1 sheet 5 mm paper-faced foamboard ≥500 × 500 mm (+1 spare) — $22 misc bag
- [ ] Spray mount adhesive — $22 misc bag
- [ ] 2 × Ø4 × 0.5 mm carbon tube, 500 mm (**BOM gap, ~$4**)
- [ ] M3 × 10 nylon screws ×8, M3 nylon nuts ×4 — $22 misc bag
- [ ] #64 rubber bands, hook-and-loop strap — $22 misc bag
- [ ] 5-minute epoxy, paint pen, 3 mm TPU/foam offcut

**Print (PETG preferred; PLA acceptable and *more* frangible)**
- [ ] `placard_index_disc.stl` × 1
- [ ] `placard_shoe.stl` × **4** (1 flight + 3 spares — this is the crash part)
- [ ] `placard_spar_cap.stl` × 4

**Build**
- [ ] Cut the panel to **450 × 450 mm**, square to ≤1 mm.
- [ ] Slit both the top and bottom edges 3 mm deep; epoxy in the spars; the **bottom** spar
      protrudes 15 mm each side; cap the ends.
- [ ] Spray-mount one print per face, **both right-reading** (do NOT flip one print).
      Roll flat; no bubbles inside the black square.
- [ ] Weigh the finished panel. **Record it** — §4's 152 g is an estimate.
- [ ] Relocate the LiPo to the bottom plate on the TPU pad; re-route the XT60 lead.
- [ ] Bolt the index disc to the top-plate standoffs; mark 0° with the paint pen.
- [ ] Bolt the shoe at the 0° index; fit the panel; two rubber bands + strap.
- [ ] Weigh the whole aircraft. **Record it.** Balance it on two fingers under the arms and
      note where the CG actually is vs §4.2's +22.3 mm prediction.

**Before it flies** — §9.

---

## 8. Operating limits (a real constraint the protocol currently lacks)

| limit | value | basis |
|---|---|---|
| Target airspeed, **0° index (beam)** | **no aerodynamic cap** — 9 m/s and beyond is fine | added drag 0.28 N, trim 7.0° [DERIVED §4.4] |
| Target airspeed, **90° index (nose-on)** | **≤ 4 m/s. HARD.** | 16.6° trim at 4 m/s (`R90` 6.80 m, in the gate); `ANGLE_MAX` at 5.83 m/s and the decode gate at ~6.0 m/s — both fail together [DERIVED §4.4] |
| **Steady crosswind component** | **≤ 3 m/s** for a *valid* pass | 9.4° bank; 18° crab already spends half the incidence cone [DERIVED §4.5] |
| Gust / peak crosswind | **≤ 5 m/s** | 24.7° bank; track hold is lost at 5.6 m/s |
| Total wind, hard no-fly | **> 6 m/s steady or > 8 m/s gusting** | compounding of the above; judgment call, flag as [ASSUMPTION] |
| Aircraft **on the ground**, placard fitted | **≤ 3 m/s** wind, or lay it down / hold it. Fit the panel last. | calculated tip-over at **4.3 m/s** [DERIVED §5.5] — the tightest wind limit here |
| Briefed bank angle in a turn | **≤ 20°** | costs 6 % of decode range; above 30° it eats the cone |
| Leg heading | brief **within ±30° of the wind line** where the field allows | minimises the crosswind component, which is the expensive one optically |
| Packs per session | **4–5**, not 3 | 25–35 % endurance loss [DERIVED §4.6] |
| PID tune | **Autotune with the placard fitted**, calm air, before any AUTO mission | CG crosses the rotor plane [§4.2] |

**You need a way to measure wind.** A handheld anemometer is ~$20 and is not in any BOM tier.
Without it these limits are unenforceable and every pass carries an unknown crosswind — which
is also an uncontrolled variable in curve (a). **Add it to the field layer.**

---

## 9. Bench gates before the field day (all $0, all at home)

Run these in order; each is pass/fail and each has a cheap fallback.

1. **Panel + aircraft mass on a kitchen scale.** Compare to §4.2's 152 g / 803.5 g.
   Fail (>15 % over) ⇒ switch to the 3 mm XPS panel.
2. **CG by hand-balance**, placard fitted, compared to +22.3 mm. Fail (much higher) ⇒ lower
   the panel / confirm the battery relocation.
3. **★ GPS transparency — the highest-value check in this doc.** Aircraft on the ground,
   same spot, powered, 2 min with the placard fitted and 2 min without. Log satellite count
   and HDOP from the DataFlash log. **Pass: ≥10 sats, HDOP ≤1.2, and no meaningful delta
   between the two.** Fail ⇒ rear GPS mast, re-test, and if still bad the AUTO-leg
   repeatability claim needs re-examining before the field day.
4. **ELRS link.** RSSI / LQ at the far end of the intended flight line, with and without the
   placard. Fail ⇒ re-route antennas, re-test.
5. **Compass calibration with the placard fitted**, in the flight index. Then a Loiter hover
   and check for heading drift / toilet-bowling.
6. **Hover check.** Compare hover throttle %, ArduPilot `VIBE` X/Y/Z, and any new gyro noise
   peak, placard vs bare. **Pass: `VIBE` < 30 m/s², clipping = 0, no visible panel flutter.**
   Fail ⇒ add the vertical centreline spar / go to the XPS panel.
7. **Frangible pull test.** Luggage scale on the panel's top edge, aircraft held down; pull
   until release. **Pass: 60–80 N**, and the frame/shoe-mount is undamaged afterwards.
   Adjust rubber-band count to land in band.
8. **Prop clearance, loaded throttle.** Props on, aircraft restrained, spin up to a realistic
   loaded throttle and confirm no blade-to-panel contact and no contact under panel deflection.
   Also **measure** the rotor plane's height vs the top plate and the true prop-tip-to-
   centreline gap, and correct §4.2/§5.5 if they differ. *(This is the target-side twin of the
   interceptor's `prop-clearance` HARD gate.)*
9. **Autotune with the placard fitted**, calm air. Then a scripted AUTO leg at 9 m/s at the
   0° index and confirm the aircraft actually reaches commanded speed, and log the trim pitch
   (predicted **7.0°**).

---

## 10. What the TRIPOD DAY must confirm

Everything in §3–§5 that is not marked [MEASURED] lands here. In rough priority:

1. **`R_decode90(0°)` itself** — the money-gate number. Prediction: **5.98 m** (pessimistic
   corner) to **7.10 m** (realistic), gate needs **≥6.0 m**. This is the whole point of
   curve (a).
2. **The `cos θ` law out to ≥60° incidence.** Predicted `R_decode(θ) = R_decode(0°)·cos θ`.
   Sim evidence exists only to 32.6°. A crossing pass at a 5–6 m standoff sweeps range and
   incidence together and measures this in one pass (see §11 item 4).
3. **The usable incidence cone.** Predicted **±32°** at the deployed `quad_decimate = 2.0`.
   If it comes back narrower, the beam mount's aspect brief has to tighten or the placard
   grows past the carry limit (which means a bigger target airframe).
4. **`quad_decimate = 1.0` vs `2.0` on the same frames.** Predicted ~1.5× range and a cone
   widening to ±48–56°. Free — offline re-decode, no extra field time.
5. **Motion blur at 9 m/s** with the ≤1 ms exposure — the one thing Gazebo cannot model
   (`tripod_test_protocol.md` §4.4). Does it move `P_min`, i.e. shrink `R_decode90`?
6. **[HYPOTHESIS from §3.1] Does a body-fixed, beam-facing tag decode through a real crossing
   pass?** The sim's 0-detection crossing result used a *world-fixed* tag orientation
   (`m4_intercept.py` line 1581) and cannot answer this. **Report it as a new measurement;
   do not use it to reopen the graveyard entry**, which also rests on the tag's short envelope
   and on the Stage-0 scoping in `detection_tracking_methods.md` §B.4.
7. **Sun glare.** Fly one pass block with the sun behind the camera and one with the sun
   behind the target. Does specular glare off a matte print kill decode at any aspect?
   Nothing in sim covers this.
8. **Measured trim pitch on the 9 m/s beam-index leg** (predicted 7.0°) and **measured max
   speed at the 90° nose-on index** (predicted 5.8 m/s). These two numbers validate or kill
   the whole drag model in §4.3–§4.4 in a single afternoon.
9. **Did the frangible shoe do its job** on the day's inevitable hard landings — panel
   released, frame undamaged? Count shoes consumed.
10. **Real mass, real CG, real flight time per pack** vs §4's estimates.
11. **GPS/ELRS in flight** with the placard fitted, at the far end of the line (the bench
    check in §9 is static and near).

---

## 11. Recommended changes to `docs/tripod_test_protocol.md` (diff-level — DO NOT EDIT HERE)

I am not the writer of that file. These are the specific edits I recommend; item 4 is the
money-relevant one.

1. **§1 pre-session checklist — item 2 is STALE.** It still says the auto-labeler validation is
   *"NOT yet run"* and to use *"the BOM default placard (~0.3 m)"*. Superseded by
   `docs/placard_sizing.md` (2026-07-21): edge = **0.35 m**, print UNBLOCKED. Replace with a
   pointer to `placard_sizing.md` + this doc's §9 bench gates ("placard bench gates GREEN").
2. **§2 What to bring — add:** 3 spare printed shoes, 1 spare pre-printed panel, M3 nylon
   hardware + rubber bands, a **handheld anemometer** (~$20, currently in no BOM tier —
   §8's limits are unenforceable without it), and a paint-pen-marked index disc.
3. **§3.1 Position — add:** for tag-decode (curve (a)) passes the tripod must be **abeam** the
   target's leg, not on its extension, because the placard is beam-facing (§3.6). State the
   intended standoff explicitly.
4. **★ §4.1 Range stations — the stations do not intersect the decode envelope.** Predicted
   `R_decode90` is **6.0–9.0 m**; the current ladder is **8, 12, 16, 20, 25, 30 m** and the
   crossing standoff in §4.2 is **15–20 m**. As written, curve (a) returns ~all zeros for
   *geometric* reasons — **and an all-zero curve (a) is a NO-GO on the $740 order.** The
   protocol can currently manufacture a **false NO-GO**.
   **Recommend:** add near stations **4, 6, 10 m**; bin curve (a) in **2 m bins from 4 to
   14 m**; and add a dedicated **"tag-envelope" crossing block at a 5–6 m standoff**
   (keep 15–20 m for curve (b), where it is correct).
   *Worked example at a 6 m standoff:* along-track offset `s` gives range `√(36+s²)` and
   incidence `atan(s/6)`; the predicted decode window (`R/cos θ ≤ 7.10 m`) closes at
   `s ≈ ±2.6 m` = **0.58 s at 9 m/s ≈ 17 frames** — usable but tight, so fly several passes
   and include slow (3–4 m/s) repeats for dwell.
5. **§4.2 Aspects — re-brief which aspect the tag lives in.** With the beam mount, **CROSSING
   is the tag's primary aspect** and **APPROACH is its null aspect** (90° incidence, zero
   decodes). Say this explicitly so a zero on an approach pass is not misread as a tag failure.
6. **§4.6 Capture matrix — add a "nose-on block":** placard indexed to **90°**, approach
   passes, **≤4 m/s hard limit** (§8), 2–3 passes. This is the only source of approach-aspect
   frames for `autolabel_from_apriltag.py`, which is the NN training-label pipeline.
7. **§4.6 pass budget — 3 packs is optimistic.** 25–35 % endurance loss with the placard
   fitted (§4.6). Plan **4–5 packs** or drop the "best-effort" rows first.
8. **NEW §4.7 "Placard configuration and wind limits"** — import §8 of this doc wholesale:
   the index setting per block, the ≤4 m/s nose-on cap, the ≤3 m/s crosswind / ≤5 m/s gust
   validity limits, the ground-handling wind limit, and "brief the leg within ±30° of the
   wind line".
9. **★ §7.1 Curve (a) scoring — score decode vs `(range × INCIDENCE)`, not range alone**, and
   **re-decode the same frames offline at `quad_decimate ∈ {2.0, 1.0}`.** Incidence is
   computable per frame from the target's ULog attitude + the surveyed tripod position + the
   known mount index angle — **zero extra field time, and it is the single largest information
   gain available from the session.** Report `R_decode90(θ)` and test it against the predicted
   `R_decode90(0°)·cos θ`. This mirrors §7.2's existing "do not report range-only numbers"
   rule (ADR-0076 add #18k), for the same reason.
10. **§8.1 GO/NO-GO — state the gate in incidence-aware form:**
    `t_go = (R_decode90(0°)·cos θ_eng − R_streak_burn) / V_closing ≥ 0.5 s`, where `θ_eng` is
    the worst-case incidence the *real* engagement will present. A GO measured at θ = 0° and
    applied to a θ = 30° engagement is not a GO.
11. **§6 What to record — add per pass:** the **placard index angle** and the **measured wind
    speed + direction**. Without the index angle, no frame's incidence can be reconstructed
    and item 9 is impossible after the fact.
12. **§9 FAA — the mass figure is off.** §9 says *"~700–900 g class"*. The component roll-up
    here is **~610 g bare / ~805 g with the placard** (§4.2). The `>250 g ⇒ register`
    conclusion is unaffected; the number should be corrected so it is not cited elsewhere.

---

## 12. Contradictions and gaps this doc surfaces

| # | Finding | Where it bites |
|---|---|---|
| 1 | **The placard sheet is 450 mm, not 350 mm.** All three places that mention it (`bom_tiers[0]` item title "~0.3 m print", `hardware_order_list.md` §E "~0.25–0.35 m", the common shorthand "the 0.35 m placard") name the *tag*, not the *sheet*. Every mass/area/clearance number keys off 450 mm. | §2. Affects the print order, the misc-bag budget, and every number in §4. |
| 2 | **Ordered frame ≠ BOM text.** `hardware_order_list.md` §E line still reads *"GEPRC Mark4 or equiv ~220 mm"*; the ORDERED frame is the **TBS Source One V6, wheelbase 240 mm**, 136.5 g, 2 mm top plate, 30/22 mm standoffs. The 20 mm changes the prop-clearance corridor (21.4 mm, not ~11 mm). | §5.5. |
| 3 | **Target AUW over-stated.** `tripod_test_protocol.md` §9 says "~700–900 g class"; roll-up gives **~610 g bare / ~805 g with placard**. FAA conclusion unchanged. | §4.2, §11 item 12. |
| 4 | **The tripod protocol's ranges/standoffs sit entirely outside the predicted tag decode envelope** (stations 8–30 m and a 15–20 m crossing standoff vs a 6.0–9.0 m envelope). **The protocol as written can produce a false NO-GO on the $740 order.** | §11 item 4. Highest-value finding in this doc. |
| 5 | **The sim's AprilTag orientation is world-fixed, not body-fixed** (`m4_intercept.py` L1581-87, `apriltag_target/model.sdf`). The graveyard's "0 detections in a 16 m/s crossing dash" is therefore not a model of any physical mount. **The graveyard entry stands** (short envelope + Stage-0 scoping, `detection_tracking_methods.md` §B.4) but the mechanism is partly a mount artifact — recorded here as a hypothesis for the tripod day, **not** as a reopening. | §3.1, §10 item 6. |
| 6 | **The money gate has no incidence budget at the pessimistic corner** (`qd=2.0` + conservative camera scaling ⇒ θ_max = 0°). Under realistic scaling it is ±32°, and ordinary flight attitudes plus engagement-geometry error can consume it. `quad_decimate = 1.0` is the cheapest widening lever and is currently unexercised. | §3.3, §10 item 4, §11 item 9. |
| 7 | **The auto-label pipeline and the money-gate pass block want opposite tag aspects.** `autolabel_from_apriltag.py` only labels frames where the tag decodes; a beam-facing tag cannot label approach frames. Hence the 90°/≤4 m/s nose-on block. | §3.6, §11 item 6. |
| 8 | **Target-side prop clearance was never a named check.** The `prop-clearance` constraint covers only the interceptor's camera boom. A 450 mm panel on the target passes ~19 mm from two prop discs. | §5.5, §9 item 8. |
| 9 | **BOM gaps:** the two carbon spars (~$4) and PETG filament are not in the $22 misc bag; a **handheld anemometer** (~$20) is in no tier and §8's operating limits are unenforceable without one. | §5.2, §8, §11 item 2. |
| 10 | **Specular glare is unmodelled anywhere.** Gazebo has no glare model, so "matte print" is an untested-but-necessary requirement rather than a validated one. | §2, §10 item 7. |

---

## 13. Assumption register — every number that is not measured

| Quantity | Value used | Status | Retired by |
|---|---|---|---|
| Foamboard areal density | 450 g/m² (range 340–485) | ESTIMATE (vendor typicals) | §9 item 1 — weigh it |
| Component masses (motors, stack, GPS, RX, pack) | see §4.2 | ESTIMATE (catalogue) | §9 item 1 |
| Component z-heights | see §4.2 | ASSUMPTION | §9 item 2 — hand-balance |
| Rotor-plane height vs top plate | `z ≈ 0 ± 5 mm` | ASSUMPTION | §9 item 8 — measure |
| Prop-tip-to-centreline gap | 21.4 mm (symmetric-45° X) | DERIVED from a symmetric-X ASSUMPTION; V6 is "wide-stance" | §9 item 8 — measure |
| `C_d` flat plate normal to flow | 1.17 | ESTIMATE (standard table) | §10 item 8 — measured trim pitch |
| `(C_d·A)` bare quad | 0.014 m² | ASSUMPTION, back-calibrated from "5" quad trims ~30° at 20 m/s"; placard term dominates 17:1 | §10 item 8 |
| Air density | 1.225 kg/m³ | ASSUMPTION (ISA SL) | field elevation/temperature |
| Quad yaw authority | 0.10–0.15 N·m | ESTIMATE (prop torque ≈ 0.012 m × thrust) | §9 item 9 |
| Quad roll authority | ~2.0 N·m | ESTIMATE | §9 item 9 |
| Frangible release band | 60–80 N | DERIVED floor (19 N flight load) + ASSUMPTION ceiling | §9 item 7 — pull test |
| `cos θ` decode law above 33° | holds | HYPOTHESIS (validated to 32.6° in sim only) | §10 item 2 |
| Foamboard + toner transparent at L-band / 2.4 GHz | yes, <1 dB | HYPOTHESIS | §9 items 3–4 |
| Matte print avoids decode-killing glare | yes | ASSUMPTION (no sim glare model) | §10 item 7 |
| Endurance loss | 25–35 % | DERIVED from `T^1.5` induced-power scaling | §10 item 10 |
| Hard no-fly wind (6 m/s / 8 m/s gust) | — | ASSUMPTION (judgment) | field experience |

---

## 14. Reproduce the arithmetic

Every number in §3–§5 is closed-form and comes from the inputs in §1. The two non-obvious
solves:

```python
# Trim pitch of a rigidly-mounted flat plate (nose-on) in steady level flight.
#   F_N = Cd * q * A * cos^2(th)      (normal-force model, force along the plate normal)
#   T sin(th) = D_quad + F_N cos(th)          (horizontal)
#   T cos(th) + F_N sin(th) = W               (vertical)
# NOTE: solve this by BISECTION on the vertical residual, NOT by fixed-point iteration on
# tan(th) = (D_quad + F_N cos th)/(W - F_N sin th) -- that form oscillates and converges to a
# spurious root above ~40 deg (it reported 14 deg for the 9 m/s case, which is wrong).
import math
rho, Cd, A, W, CdA_quad = 1.225, 1.17, 0.450**2, 7.90, 0.014

def resid(th, q):
    F = Cd*q*A*math.cos(th)**2
    T = (CdA_quad*q + F*math.cos(th))/math.sin(th)     # from the horizontal equation
    return T*math.cos(th) + F*math.sin(th) - W         # vertical residual, monotone in th

def trim(V):
    q = 0.5*rho*V*V
    lo, hi = math.radians(0.05), math.radians(89.9)
    for _ in range(200):
        mid = 0.5*(lo+hi)
        lo, hi = (mid, hi) if resid(mid, q) > 0 else (lo, mid)
    return math.degrees(0.5*(lo+hi))

# trim(4) = 16.6 deg ; trim(5.83) = 30.0 deg ; trim(6) = 31.2 deg ; trim(9) = 47.5 deg

# Max level speed at a fixed lean limit (ANGLE_MAX), nose-on -- same two equations with th
# fixed and q unknown:   k = Cd*A*cos^2(th)
#   q = W / ( (CdA_quad + k*cos th)*cos th / sin th  +  k*sin th )
# ANGLE_MAX 30 deg -> q = 20.81 Pa -> V = sqrt(2q/rho) = 5.83 m/s   (consistent with trim())
# ANGLE_MAX 45 deg -> q = 43.52 Pa -> V = 8.43 m/s, but incidence 45 deg -> R90 = 5.02 m: FAIL

# Beam (edge-on) drag, no plate normal force:
#   D = CdA_quad*q + q*1.8*(0.005*0.45) + 0.004*q*(2*A)   -> 0.976 N at 9 m/s, trim 7.0 deg
```

Incidence cone: `theta_max = acos(6.0 / R90_0)` with `R90_0` from §1's table.
Crosswind: `S = 0.5*rho*Vc^2 * 1.17 * 0.2025`, `phi = atan(S/7.90)`.
