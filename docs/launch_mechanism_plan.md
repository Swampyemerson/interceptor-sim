# Launch mechanism — IMPLEMENTATION plan on the ordered hardware (B-core)

> **Status: IMPLEMENTATION DESIGN (2026-07-24), NOT RATIFIED.** The builder PARKED
> the launch mechanism on 2026-07-21 ("until real flights inform it"); this note
> does not un-park it. It converts the prior 4-lens **options** evaluation
> (`docs/launch_mechanism_options.md` → recommend **B-core**: manual trigger +
> coded dash + an honesty-clean pre-flight aim stack) into a **buildable plan on
> the parts that are already ordered**, so that when it is un-parked there is
> nothing left to decide except "go". Nothing here changes
> `docs/project_state.json` (the head owns the contract).
>
> **What changed since the options eval:** we now have a quantitative
> **aim-error budget** (§3) derived from a validated ballistic model
> (`scripts/experiments/flight_plans/dash_cpa_model.py`,
> `docs/flight_plan_candidates.md` §1.3). The old note could only say "a human
> point may not be accurate enough". We can now say **how accurate it has to be**:
> **±3° for a 0.5 m ram**, and **±0.1 s of trigger timing** — numbers that decide
> the mechanism rather than leaving it to taste.

---

## 0. Hardware reality check (read before planning anything)

| subsystem | status | what that means for launch |
|---|---|---|
| **Target aircraft** (Kakute H7 V1.5 + Tekko32 + ECOII 2207 + TBS Source One **V6** + M10Q + RP1 RX + Pocket M2 TX + 3× CNHL 6S + charger + props) | **ORDERED 2026-07-20** | The thing being intercepted exists. It flies **ArduPilot AUTO** legs, so the target's track is a **pre-programmed constant** — which is what makes the launch aim honestly computable pre-flight. |
| **Seeker kit** (innomaker OV9281 mono GS ~118° HFOV, Pi 5 8 GB, 22→15 CSI cable, microSD, PSU) | **ORDERED** | The launch-time **bearing latch** (§2) runs entirely on this. Buildable and benchable **today**, no interceptor airframe needed. |
| **Interceptor BRAIN** (Pixhawk 6C Mini + PM02 V3 analog + Holybro M10) | **ORDERED** (front-loaded past the money gate, logged) | The **props-off bench bring-up** of the Pi↔TELEM2 MAVLink/OFFBOARD path — the exact path the launch sequence needs — can be done **now**. |
| **Interceptor AIRFRAME** (frame, motors, ESC, props, RP3 ELRS RX, BEC12S-PRO, camera boom/wedge print) | **NOT ordered** — Tier 2, gated on the tripod AprilTag decode-envelope curve | Anything that needs a flying interceptor is **later**. Note the brief's "GEPRC Mark5" is the *original* BOM frame; the §0c lean swap is a **TBS Source One V5** and it is **unbought**. Plan accordingly: nothing in §7's "now" column assumes it. |
| Consumables/tools (solder, XT60, wiring, UART jumpers, USB-C **data** cable, 18650s, smoke stopper, LiPo checker, +2 6S packs) | ORDERED | The bench work in §7 is unblocked except for the still-missing items listed there. |

**Ordered-hardware rule for this plan:** every "do it now" item uses only ordered
parts; every item that needs Tier 2 is marked **[T2]**.

---

## 1. The physical launch: **ground takeoff → standby hover → trigger**

### The options, and the call

| option | what it is | verdict |
|---|---|---|
| **Hand-launch (throw)** | Operator holds the armed quad and throws it toward the target as the dash starts. | **REJECT.** A 5″/6S quad at ~950 g with 5.1″ tri-blades is a hand injury waiting to happen, the release attitude/velocity is different every time (so the dash's initial condition — which the whole §3 budget depends on — is un-repeatable), and PX4 has no validated throw-launch path in this project. It also destroys the metrology: the sim model (and the ballistic model) assumes the dash starts **from rest at a known point**. |
| **Rail / perch / catapult** | Aircraft sits on a rail or perch; a mechanical release fires it. | **REJECT (for now).** Needs custom PX4 launch handling + a shock-hardened camera boom, none of which is ordered or budgeted, and it buys nothing the standby hover does not: the interceptor's problem is **aim and framing**, not muzzle velocity. Revisit only if the standby hover's ~1 s takeoff transient is ever measured to matter. |
| **Ground takeoff → standby hover → trigger** ✅ | Aircraft sits on a printed launch plate; operator arms via ELRS; the Pi commands takeoff to a standby altitude and **holds position + a commanded yaw**; on the trigger it starts the coded dash. | **ADOPT.** Reasons below. |

### Why the standby hover wins (four reasons, each traced)

1. **It maps 1:1 onto the code that is already validated.** The sim's phase machine
   is `TAKEOFF → CODED_DASH → ENGAGE → BREAKOFF → LAND`
   (`scripts/m4_intercept.py --coded-dash`), and the live MAVSDK OFFBOARD path
   (connect → health-gate → arm → takeoff → stream setpoints → land/disarm) is
   already exercised against PX4 SITL by `scripts/check_deploy_sitl.sh` (it drives
   `flight/deploy/seeker_loop.py --sitl-smoke`; three real MAVSDK defects were found
   and fixed there). Ground takeoff needs **no new flight mode**.
2. **It turns the aim from a human-motor problem into a number.** With a hand-launch
   the aim is where the operator's arm points (±5–10°, and unmeasurable). With a
   standby hover the aim is a **commanded heading held by PX4's yaw controller**
   (~1–2°), and it is logged. Given the §3 budget (±3° for a ram), that difference
   *is* the difference between a hit and a fly-through.
3. **It is the physical form of the Phase-A loft.** The best current pointing lever
   is "climb, then dive on the co-altitude target so the nose-down dash pitch aims
   the camera *at* it" (`docs/intercept_accuracy_levers.md`; measured +7° of centring
   from 2 m of loft). A standby hover *is* the loft — for free, before the clock
   starts, with no dash time spent climbing. **Set the standby altitude = target
   altitude + the loft height** and the dash begins already lofted.
4. **It preserves the metrology.** Same start point every flight → paired,
   comparable ULogs → `scripts/field_score.py` can compute CPA between the two
   aircraft the way the sim does. A throw randomises the initial condition and
   every field number becomes noise.

**Cost, stated honestly:** the interceptor burns battery on standby (endurance on a
6S 1500 mAh pack at ~950 g AUW is **unmeasured — bench it**; plan a **30–60 s**
standby budget and treat anything longer as a measurement you have not made), and a
hovering quad is audible/visible — irrelevant
for a test range, relevant for the "deployment" narrative. Also, PX4 will not enter
OFFBOARD without an already-streaming setpoint, so the standby hover **must** be
flown as a stream of hold setpoints from the Pi, not as a PX4 LOITER the Pi later
takes over (see §5).

**The launch plate [T2-adjacent, printable now]:** a flat printed plate with a
recessed skid pocket and a scribed north line + a printed protractor arc, staked to
the ground. Its job is not to point the aircraft (the yaw controller does that) —
it is to (a) give a repeatable, level, debris-free takeoff surface, (b) hold a known
survey position so the operator can measure the launch→target-leg geometry with a
tape, and (c) keep the prop wash off loose grass (a real source of takeoff yaw
kicks). ~$0.

---

## 2. The honesty-clean launch-time AIM stack

Everything below is latched **at or before the trigger** and is then a **frozen
constant**. Nothing enters the guidance loop after launch — that is the
jam-resistance claim, and it is enforceable by audit (constraint `no-datalink`,
`honesty-boundary`).

### The three constants

| # | constant | source | honesty class |
|---|---|---|---|
| **C1** | **dash heading** (NED deg) | *Primary:* the trigger-instant **camera bearing latch** (below). *Fallback:* `collision_lead_heading(target_start, target_vel, dash_speed)` from the operator-entered AUTO-leg geometry. | Pre-flight constant. The AUTO leg is *programmed by us*, so target start/velocity are known inputs, not a live sensor read (identical status to the sim's `--target-start/--target-vel`). |
| **C2** | **crossing bias / lead correction** (deg, sign keyed on the known crossing direction) | Sized to the aircraft's **own measured dash acceleration** (`dash_cpa_model.py --sweep`), *not* copied from the sim's 30°. | Pre-flight constant, vehicle spec. |
| **C3** | **standby altitude** = target altitude + loft | Chosen with the wedge angle (`docs/flight_plan_candidates.md` §2, arm C) | Pre-flight constant. |

### C1 primary — the **static-camera bearing latch** (Option A's only salvageable half)

The seeker detector reads ~100 % reliably when the camera is **static** (8–22 m,
set-pose sweep) and ~0.8 % in flight — so the *one* moment the camera is
trustworthy is **before launch**. At the trigger the Pi takes the current
detection's box centre, undistorts it through the calibrated intrinsics
(`flight.camera.CameraModel.pixel_to_ray` — mandatory, the ~118° M12 barrel-distorts
>1° at the edge), converts to a bearing, adds the FC's own-state yaw, applies the
lead solve, and **freezes the result as C1**. Then the seeker's output is
*structurally disconnected* from the dash until the 5-detection handoff streak
fires. Precision: 1280 px over ~118° ≈ **0.092°/px**, so even a sloppy ±5 px box
centre is ±0.5° — an order of magnitude inside the §3 budget.

**Two things the latch buys that a pre-computed heading does not:**

- **It absorbs trigger-timing jitter (§3.2).** A pre-computed heading is only valid
  at one instant; the latch is computed *at the instant you fire*.
- **It cancels compass/EKF yaw error to first order.** A map-referenced heading is
  flown against the EKF's yaw estimate, so a 3–5° magnetometer error (entirely
  normal on a small quad near metal) is a 3–5° aim error. A camera-relative aim uses
  the *same* yaw estimate to compute and to fly, so the error largely cancels.

### C1 fallback and the optional one-shot ground spotter

If no static lock is held at the trigger, fall back to the operator-entered
geometry solve (C1 fallback) — always available, since we programmed the AUTO leg.
The **optional no-radio one-shot ground spotter** (2nd OV9281 + narrow lens +
tripod + laptop) stays exactly as the options eval scoped it: it solves **one**
collision-lead heading before launch, the number is typed/loaded as C1, **and the
link dies at launch**. Buy it only if the field aim-error rig (§7) shows the
onboard latch cannot be trusted. It is **not** in the ordered kit.

### The `--dash-speed` trap (new, and it applies to the real aircraft)

`collision_lead_heading` solves a **constant-speed** intercept triangle. The real
interceptor, like the sim one, starts **at rest**. On the canonical sim geometry
that mismatch is worth ~1 m of CPA at the baseline and ~1.5–2 m under an accel cap
(`docs/flight_plan_candidates.md` §1.3). **Do not fly the hardware with a
constant-speed lead solve.** Either size C2 to the measured accel (zero code) or
adopt the proposed accel-aware lead (`accel_lead_heading`, ADR-lite #1 in the flight
-plan doc). The number to measure on the first dash: **the ULog's achieved
horizontal acceleration profile** — that single curve sets C2.

---

## 3. The aim-error budget — the number that decides the mechanism

*DERIVED* (`scripts/experiments/flight_plans/dash_cpa_model.py --sensitivity`,
canonical line-9 geometry @9 m/s; the model is validated against the flown dash-only
arms to 0.29–0.58 m MAE — it **ranks**, it does not conclude).

### 3.1 Heading error → miss

| aim error | CPA, **fast** dash (a≈10–12 m/s²) | CPA, **slow/capped** dash (a≈3.6 m/s²) |
|---|---|---|
| ±1° | 0.18 m | 0.30 m |
| ±3° | 0.42 m | 0.35 m |
| ±5° | 0.69 m | 0.43 m |
| ±10° | 1.38 m | 0.63 m |
| ±20° | 2.22 m | 1.17 m |
| **tolerance for a 0.5 m RAM** | **±3.0°** | **±4.5°** |
| tolerance for 1.0 m | ±6.5° | ±13.0° |
| tolerance for 2.5 m (Pk proxy) | ±18.0° | ±36.5° |

Read against the aiming methods:

| method | typical error | 0.5 m ram? | 2.5 m Pk? |
|---|---|---|---|
| Human eyeball / hand-launch | 5–10° | **no** | yes |
| Boresighted red-dot or phone gunsight | 1–2° | marginal | yes |
| **Static camera bearing latch** | **~0.1–0.5°** | **yes** | yes |
| Magnetometer/EKF yaw (map-referenced aim) | 2–5° | **no** on its own | yes |

**Conclusion:** the ±30° "acquisition tolerance" (48/48 flights engaged) is a
*dash-robustness* number and has nothing to do with the kill bar. **For a ram, the
aim stack is not a nicety — it is load-bearing**, and the camera bearing latch is
the only listed method that clears the budget by a comfortable margin.

### 3.2 Trigger-timing error → miss (the under-appreciated half)

At launch the target sits ~16.7 m away crossing at 9 m/s, so the line-of-sight
sweeps at **v/R ≈ 0.54 rad/s ≈ 31°/s**. With a **pre-computed** heading, every
**100 ms** of trigger latency/jitter is therefore worth **~3° of aim error — the
entire 0.5 m ram budget.** Human press-latency jitter alone is ±100–200 ms.

*Mitigation (and the reason the latch is the primary path):* compute C1 **at the
trigger instant from the latched bearing**. Then trigger jitter no longer rotates
the aim — it only changes the engagement range slightly (~0.9 m per 100 ms), which
the lead solve absorbs. **This is the single strongest argument in the whole
document for putting the bearing latch on the critical path rather than treating it
as a nice-to-have.**

### 3.3 The design fork this exposes

A **slower run-in is dramatically more forgiving of aim error** (±13° vs ±6.5° for a
1 m CPA) because a slower interceptor meets the target closer to its own launch
point, where a given angular error maps to less cross-range. It costs closing speed
and (per the Phase-A Gazebo A/B) terminal authority. **Recommendation for the first
real kill attempts: fly the SLOWER, more forgiving profile**, take the kill, and
only then climb the speed ladder — the goal is a binary contact, not a fast one.
This is a hardware-side reason to keep `--dash-accel-cap` in the toolbox even if
`docs/flight_plan_candidates.md` arm E shows it is not the sim's best miss.

---

## 4. How the pilot points it and fires it

**Pointing** — the operator never physically aims the aircraft. Sequence: place it
on the plate (any orientation), arm, and the Pi flies it to the standby hover and
**yaws it to C1**. The human's aiming job is reduced to *placing the launch plate*
somewhere with a clear line to the target leg — a tape-measure job, not a marksman
job.

**Firing** — a dedicated **ELRS aux-channel switch** on the ordered RadioMaster
Pocket M2 TX. The path: switch → ELRS → interceptor RX (RP3, **[T2]**) → SBUS →
6C Mini RC-IN → PX4 → **MAVLink `RC_CHANNELS` over TELEM2 → the Pi**, which watches
for the rising edge and moves its own state machine `STANDBY → CODED_DASH`. No new
hardware, no new radio, no second link.

**Why this does not violate the no-datalink constraint** (constraint `no-datalink`,
BOM §0⑤: "ELRS is kill/arm only"): the GO bit is a **command**, not a **cue** — it
carries zero target information, it is a single edge, and after it fires the RF link
may be switched off, jammed, or walked out of range with no effect on the intercept.
This must be *demonstrated*, not asserted: **a mandatory link-denied test arm**
(kill the TX immediately after the GO edge; the intercept must complete unaided) is
part of the flight card in §7. That test *is* the jam-resistance deliverable.

**Safety interlocks (all on the same link, all sanctioned):**
- **Deadman/kill:** a second ELRS switch is the hardware kill (PX4 kill switch). It
  stays live for the whole flight — killing is not guiding.
- **Arm gate:** the Pi refuses to enter `CODED_DASH` unless armed, in OFFBOARD, at
  standby altitude ±0.3 m, and holding C1 ±2° (else it aborts to LAND).
- **Dash timeout:** hard `--coded-dash-max-s` equivalent (sim default 8 s) →
  BREAKOFF → LAND.
- **Geofence:** PX4 geofence sized to the range box; breach → RTL/LAND.

**Rejected alternative:** a physical button on the Pi's GPIO. It puts a human within
arm's reach of a spinning-prop aircraft at the moment of launch. No.

---

## 5. Mapping onto the validated PX4/MAVSDK offboard start

**What already exists and is validated:**
- `scripts/check_deploy_sitl.sh` → boots headless PX4 SITL and drives
  `flight/deploy/seeker_loop.py --sitl-smoke`: connect → health-gate → arm →
  takeoff → **enter OFFBOARD** → stream NED velocity+yaw setpoints ~45 s → land →
  disarm, cleanly. Exit 0/1 gate.
- `flight/deploy/seeker_loop.py` runs the **real** terminal: Picamera2/frame source →
  ONNX detector → undistorted bearing + size range → `flight.geometry`
  (full-attitude LOS derotation + `camera_to_cg_los` lever-arm) → `flight.estimator`
  α-β → `flight.guidance` pro-nav N=5 → MAVSDK NED setpoint. Audited gt-free.

**The gap (the one real build item):** `seeker_loop.py` implements the **terminal
only** — on the vehicle path it deliberately does *not* arm, take off, or land
("props-spinning autonomy is the dash's job"). **There is no `CODED_DASH` executor
on hardware.** So the launch mechanism's software deliverable is
`flight/deploy/real_flight.py`:

```
STANDBY   arm-gated; stream hold setpoints (velocity 0, yaw = C1) at >=2 Hz so
          PX4 stays in OFFBOARD; run the detector; maintain the latch candidate.
   |  GO edge (RC_CHANNELS aux high) -> latch C1 (bearing latch or fallback), C2, t0
   v
CODED_DASH stream NED velocity = dash_speed(t) * [sin C1, cos C1] with the
          accel ramp and the loft-dive altitude reference from flight.guidance
          (dash_forward_speed / dash_loft_alt_ref -- the SAME portable functions
          the sim flies). Own-state only. Hand off on 5 consecutive fresh
          detections.
   |  handoff
   v
ENGAGE    unchanged: seeker_loop's pro-nav terminal to contact.
   |
   v
BREAKOFF -> LAND (or, for a ram, contact -> tumble -> the range's recovery drill)
```

Requirements on that module, each traced to a rule already in force:
- **Wall-clock, not sim-clock.** The `flight/` ramp/dive functions take
  "seconds since dash start"; on hardware that is a monotonic wall clock. Add the
  audit that no sim-clock assumption leaks in (`real_build_coded_dash.md` build
  path item 2).
- **Safety module** (RC kill live, geofence, dash timeout) before any props-on test.
- **Honesty audit re-earned:** the bearing latch is a *new* camera→command path
  (camera pixel → dash heading). It is honest because it is latched **once, before
  launch**, but that has to be *checked*: assert that the dash heading variable is
  written exactly once, at the GO edge, and never again — the machine-checkable form
  of "no post-latch cue read".
- **SITL first.** `real_flight.py` is fully testable against PX4 SITL with a
  synthetic detector, exactly as `--sitl-smoke` does today. **Zero hardware
  required** — this is the highest-value item that can be built right now.

---

## 6. REJECTED — do not re-litigate (graveyard)

Both are in `docs/project_state.json` → `graveyard`; they are recorded here so the
next reader does not "discover" them again.

- **Autonomous launch-on-detect (auto-fire on an NN lock).** Zero added acquisition
  range; reaction-time-starved (~1.8 s of onboard visibility vs a 9 m/s inbound,
  before takeoff and dash even happen); does not touch the pointing wall; and
  auto-firing a spinning-prop **ram** on an *unmeasured* outdoor false-positive rate
  (birds/clutter, ADR-0078) is the least-defensible ROE available. **Only the
  bearing latch survives — with the human keeping the fire decision** (§2).
- **Smart launcher with a LIVE datalink cue.** Violates constraint `no-datalink`
  (it *is* the link an adversary jams — the whole headline), and was **measured to
  hurt**: cue-guided r2l 0/8 @Pk2.5 vs coded-dash 5/8, from an aspect-biased bearing
  (ADR-0076 add #18). **Survives only** as the zero-radio, one-shot pre-launch
  spotter of §2, whose link dies at launch.

---

## 7. Build / bench checklist

### Now — with parts already in hand (no interceptor airframe needed)

| # | item | parts used | done-when |
|---|---|---|---|
| L1 | **Build `flight/deploy/real_flight.py`** (§5 state machine) and gate it with a SITL check modelled on `check_deploy_sitl.sh` (synthetic detector, GO edge injected). | none (desk) | headless SITL run: STANDBY hold → GO → dash on a commanded heading → synthetic handoff → ENGAGE → land, exit 0. |
| L2 | **Latch-once honesty assert** + no-cheat audit of the new camera→heading path. | none | the audit script proves the dash-heading variable is assigned exactly once, at the GO edge. |
| L3 | **Props-off bench bring-up:** Pi 5 ↔ 6C Mini over TELEM2 @921600 using the ordered UART jumpers; confirm MAVSDK connect, `RC_CHANNELS` readable, OFFBOARD accepted/rejected as expected. | Pi 5, 6C Mini, PM02, jumpers, USB-C **data** cable | the Pi reads the Pocket M2's aux switch edge and PX4 accepts an OFFBOARD stream, props off. |
| L4 | **Camera calibration** (checkerboard → intrinsics + Brown-Conrady) on the OV9281; store the JSON the latch will load. | OV9281 + Pi 5 + printed checkerboard | `calibrate_camera.py` rms reported; `flight.camera.CameraModel.from_json` loads it. |
| L5 | **Bearing-latch accuracy bench:** tripod the camera, place the target quad (or the AprilTag placard) at surveyed bearings, and measure latched-bearing vs tape-measured truth. | OV9281, Pi 5, placard, tape, **tripod (not yet bought)** | measured 1σ bearing error, compared against the §3 budget (need ≪3°). |
| L6 | **Trigger-latency measurement:** log the GO-edge timestamp on the Pi vs the operator's switch throw (video timestamp). | Pocket M2, 6C Mini, Pi | measured latency + jitter; feeds §3.2 (and confirms whether the latch is mandatory — expect yes). |
| L7 | **Print the launch plate** (skid pocket, scribed north line, protractor arc, stake holes). | 3D printer | a plate that survives being staked and does not sit in the prop wash. |

**Still missing to complete the "now" column:** soldering **iron**, a microSD **for
the Kakute** (no card = no target log = no scored CPA), tripod + ball head (L5),
LiPo bag/glasses/blanket, Pi 5 active cooler, USB-C PD power bank, lost-model
buzzer, zip ties, multimeter (if unowned).

### Later — needs the Tier-2 interceptor **[T2]**

| # | item | done-when |
|---|---|---|
| L8 | **Prop-clearance corner-spin at LOADED throttle across the full wedge range** — a HARD gate (constraint `forward-camera-boom`); the bench-verified angle **clamps** the printable wedge. | no blade strike, no FoV intrusion, at the chosen tilt under load. |
| L9 | **First dash ULog** → measure (a) the steady dash **pitch** (that number *is* the wedge angle — never sim-sized) and (b) the achieved **acceleration profile** (that curve *is* C2, §2). | both numbers logged from a real dash. |
| L10 | **Standby-hover repeatability:** 10× takeoff-to-standby, log yaw-hold error and position error at the hold. | yaw 1σ ≤2°, position 1σ ≤0.5 m — else the §3 budget is not met and the launch method needs re-thinking. |
| L11 | **Link-denied arm (the jam-resistance deliverable):** kill the TX immediately after the GO edge on ≥3 flights. | the dash + terminal complete unaided; ULog shows no RC input after the edge. |
| L12 | **Aim-error field rig:** N launches at a flown target; read achieved heading error from the ULog vs the intended C1. | measured aim 1σ; decides whether the optional ground spotter is ever bought. |

---

## 8. ADR-lite decisions recorded here

1. **Launch = ground takeoff to a standby hover, then a triggered coded dash.**
   *Options:* hand-launch / rail-perch / ground-standby. *Decision:* ground-standby.
   *Why:* it is the only one that (a) reuses the already-validated PX4/MAVSDK
   OFFBOARD path, (b) turns aim into a controller-held number inside the ±3° ram
   budget, (c) *is* the Phase-A loft for free, and (d) keeps launch conditions
   repeatable so field CPAs are comparable. *Reversible:* yes — a rail can be added
   later without touching the software.
2. **The trigger-time camera bearing latch is on the CRITICAL path, not optional.**
   *Why:* §3.2 — with a pre-computed heading, 100 ms of trigger jitter is ~3° of aim
   error, i.e. the whole ram budget; the latch converts timing error into a benign
   range error and additionally cancels compass/EKF yaw error. *Fallback:* the
   operator-entered geometry solve, always available.
3. **The GO bit rides the existing ELRS link and is classed as arm/kill, not cue.**
   *Why:* it carries no target information and the link may die immediately after.
   *Condition:* the link-denied test arm (L11) is **mandatory** before the claim is
   made anywhere.
4. **First kill attempts fly the slower, more aim-forgiving dash.** *Why:* §3.3 —
   ±13° vs ±6.5° of aim tolerance for a 1 m CPA. The success criterion is a **binary
   contact**, not a fast one; climb the speed ladder afterwards.
5. **Nothing in the launch stack is allowed to read a sensor after the GO edge.**
   Enforced by the latch-once assert (L2), not by convention.

---

## 9. Open risks / honest flags

- **The camera terminal is still not proven to *help*.** The newest paired evidence
  (`docs/flight_plan_candidates.md` §1.2) is that the camera arm is **worse than its
  own dash-only control on 8/8 flights** in the framed configuration, and only
  direction-selectively better in the baseline. A launch mechanism cannot fix that;
  it is the flight-plan doc's problem. **Do not spend launcher money before it
  moves.**
- **Wind/gust is unmodelled everywhere.** It pushes the open-loop dash off track *and*
  rotates the airframe (moving where the fixed wedge points) — it degrades the two
  things this entire plan depends on. First outdoor sessions must log wind and
  correlate it with aim error.
- **The ram bar (0.3–0.5 m) is 3–5× tighter than the Pk@2.5 m sim proxy.** Every
  "kill" number quoted from the sim must be re-read against the ram bar before it is
  used to justify launcher hardware.
- **Compass/EKF yaw** is a first-order aim error source for the map-referenced
  fallback (§2) and must be calibrated on-site, away from vehicles/rebar.
- **Legal:** FAA registration + broadcast Remote ID on both >250 g aircraft (or a
  FRIA field) before any outdoor flight. Unresolved at the time of writing.
- **ROE:** a human owns weapons release. That is a decision, not a default, and it is
  the reason auto-fire stays in the graveyard.

---

*Provenance: `docs/launch_mechanism_options.md` (4-lens eval + adversarial critic,
2026-07-21) for the option ranking; `docs/project_state.json` (`launch_mech`
decision, `bom_tiers`, constraints `no-datalink` / `forward-camera-boom` /
`honesty-boundary`, graveyard) for status and hard limits;
`docs/hardware_order_list.md` §0/§0c/§0d + `bom_tiers` for what is ordered;
`scripts/check_deploy_sitl.sh` + `flight/deploy/seeker_loop.py` for the validated
offboard path; `scripts/experiments/flight_plans/dash_cpa_model.py` for the §3
aim-error budget (validated against `logs/mc_loftdive_arm{Adash,Bdash}_line9_s123.csv`
to 0.29–0.58 m MAE); `docs/kill_mechanism.md` §1 for the ram bar; ADR-0024 (wide-FoV
requirement), ADR-0076 add #18/#18e/#18g/#18h (dash aim vs camera claims), ADR-0078
(outdoor false-positive rate unmeasured).*
