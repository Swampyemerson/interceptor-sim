# FPV fidelity — a `--fpv-fast` design study (Phase 2, T22)

*Design doc only — no simulator was run to produce this, and nothing here is
committed as code. Companion to `docs/phase2_sim_to_real_plan.md` §"Track C"
and ADR-0010/ADR-0028/ADR-0028-addendum/ADR-0042/ADR-0043 (`docs/decisions.md`).
Written for a builder new to flight-control tuning — terms are defined once,
on first use.*

## What this doc is, in one sentence

The sim's interceptor currently flies at a **dash speed capped at 16 m/s**
(the fast mid-course "running start," ADR-0028) on the stock PX4 `x500` quad;
the real parent-project airframe is a **2.5"–7" FPV-class** ("first-person
view" — a small, agile racing/freestyle-style quad) interceptor carrying a
**seeker payload** (a Raspberry Pi 5 + global-shutter camera, ~150–250 g,
`NEXT.md` "Hardware Stage 0"). This doc designs — but does not build or fly —
a `--fpv-fast` fidelity profile: a bundle of PX4 flight-controller parameters
**and** guidance command ceilings that would make the sim's flight envelope
honestly match a real FPV build's speed/acceleration, instead of an arbitrary
round number.

**The one finding to hold onto while reading:** ADR-0028's addendum already
proved, in real Gazebo, that raising the PX4-side envelope *alone* did
nothing (miss 1.39→1.52 m, within noise) because the *guidance law itself*
never asked for that much speed — the binding constraint was the guidance's
own command ceiling (`V_PERP_MAX`/`V_TOTAL_MAX` in `scripts/m4_intercept.py`),
not the airframe. Any `--fpv-fast` profile that raises PX4 params without also
raising those two guidance constants is very likely a no-op — and the reverse
is also true (see §4).

## New terms, once

- **PX4 parameter:** a named, floating-point tuning knob PX4's flight-control
  firmware reads at runtime (e.g. `MPC_XY_VEL_MAX`). This project pushes them
  over **MAVSDK's runtime parameter API** — no firmware rebuild, no airframe
  file edit (ADR-0010 decision #3; confirmed in `scripts/m4_intercept.py`
  around line 2854: `drone.param.set_param_float(...)` then
  `drone.param.get_param_float(...)` to read the value back and abort the run
  if it didn't take). Every FPV-profile run logs the params it actually flew.
- **Thrust-to-weight ratio (T/W):** how many times the vehicle's own weight
  its motors can lift at full throttle. T/W = 1.0 is exactly enough to hover
  at 100% throttle (no margin for maneuvering); higher T/W buys tilt/accel
  headroom.
- **Hover thrust fraction (`MPC_THR_HOVER`):** the throttle fraction (0–1)
  needed to hover. It's the reciprocal of T/W: `T/W ≈ 1 / MPC_THR_HOVER`.
- **Guidance command ceiling:** a clamp *inside this project's own Python
  guidance code* (not a PX4 parameter) on how much lateral/total velocity the
  pro-nav law is allowed to command — `V_PERP_MAX`, `V_TOTAL_MAX` in
  `scripts/m4_intercept.py`. This is the layer ADR-0028 found actually binds.
- **Sustained lateral acceleration:** how hard the vehicle can keep turning
  (m/s²) while still holding altitude — different from a one-off "punch,"
  which trades a temporary altitude sag for more turn.

## 1. Two independent ceilings — don't conflate them

There are **two separate limits** stacked on top of each other, and a real
FPV build (or a --fpv-fast profile) has to respect both:

1. **The physical/airframe ceiling** — fixed by the vehicle's mass and motor
   thrust curve. In Gazebo this lives in the model's `.sdf` file (mass,
   `motorConstant`, `maxRotVelocity`) and cannot be changed by any PX4
   parameter.
2. **The software/control ceiling** — PX4's `MPC_*` parameters (§2) and, one
   layer further out, this project's own guidance clamps (§4). These set what
   the *controller is allowed to ask for*; they can be raised past what the
   airframe can physically deliver, in which case the vehicle just plateaus
   below the commanded value (exactly what ADR-0028's addendum measured for
   `--accel-boost`).

Confusing these two is the single easiest way to get an honest-looking
`--fpv-fast` flag that quietly does nothing, or worse, silently asks PX4 for
something the airframe can't deliver and calls the shortfall a "bug."

## 2. PX4 v1.17 parameters that set the speed/accel/tilt envelope

Read directly from this machine's `~/PX4-Autopilot` checkout (`git describe`
confirms **`v1.17.0`**, tagged 2026-04-24) — the authoritative source, since
the public docs site's per-version parameter-reference pages did not render
useful detail for this exact tag (they're generated tables that need
JavaScript; see Sources). `docs.px4.io/main` is cited alongside for prose
context, with the caveat that "main" tracks PX4's ongoing dev branch and may
already differ slightly from the pinned v1.17.0 tag — **the source-code
comment quoted below is what actually ships and flies in this project.**

| Param | What it does (PX4 v1.17.0 source comment, paraphrased) | Firmware default | @min/@max (metadata) | Currently pushed by `FPV["PX4_PARAMS"]`? | Binds today? |
|---|---|---|---|---|---|
| `MPC_XY_VEL_MAX` | Absolute max horizontal velocity for **all velocity-controlled modes** (incl. MAVSDK Offboard) — "any higher value is truncated." | 12 m/s | 0 / 20 | **Yes — 20** | Yes, hard ceiling on Offboard velocity setpoints. |
| `MPC_ACC_HOR_MAX` | Max horizontal acceleration the position-mode trajectory generator will ramp to. | 5 m/s² | 2 / 15 | **Yes — 12** (20 under `--accel-boost`) | *Soft* ceiling only — ADR-0028-addendum: raising it 12→20 bought nothing because the guidance never asked for more than ~6.7 m/s² actual lateral accel. |
| `MPC_TILTMAX_AIR` | Max vehicle tilt angle for all velocity/accel modes. | 45 deg | 20 / 89 | **Yes — 60** (70 under `--accel-boost`) | Same as above — soft; see §3, the x500's own thrust-affordable tilt is already below the FPV profile's committed 60°. |
| `MPC_JERK_MAX` | Rate limit on how fast `MPC_ACC_HOR_MAX` itself ramps in (smoothing, not a hard ceiling). | 8 m/s³ | 0.5 / 500 | **Yes — 30** | Minor; shapes the ramp, not the ceiling. |
| `MPC_THR_MAX` | Max collective thrust fraction (0–1) in climb-rate-controlled modes. | 1.0 | 0 / 1 | **No — never pushed, stays at firmware default 1.0 (already maxed)** | Can't be raised (already at ceiling). Can be **lowered** to crudely emulate an underpowered/overladen craft (§6, WORST tier). |
| `MPC_THR_HOVER` | Thrust fraction needed to hover (not a ceiling — a hover-thrust *estimate seed*). Generic firmware default 0.5; the x500 airframe file overrides it to **0.60** (`ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500`). | 0.5 (generic) / 0.60 (x500 override) | 0.1 / 0.8 | No (airframe default, not part of the FPV push loop) | Doesn't bind speed directly, but its committed 0.60 is the airframe's own admission of its T/W ratio — used in §3's derivation. |
| `MPC_XY_CRUISE` | Default horizontal speed for **autonomous Mission/RTL/Orbit/AutoFollowTarget** waypoints that don't specify their own speed. | 5 m/s | 3 / 20 | No | **Ruled out** — grepped PX4 source (`FlightTaskAuto`, `FlightTaskOrbit`, `FlightTaskAutoFollowTarget`); it never appears in the Offboard velocity-setpoint path this project's MAVSDK code uses. Not part of this profile. |

**Mechanism confirmation (ADR-0010 #3):** the push happens once, before
arming, in `scripts/m4_intercept.py`'s `main()` (~line 2849–2862): only when
`--fpv` is set, it iterates `FPV["PX4_PARAMS"]` — today exactly the first
four rows above — calling `param.set_param_float(name, value)` then
`param.get_param_float(name)` and raising `RuntimeError` if the read-back
doesn't match within `1e-3`. **Adding `MPC_THR_MAX` as a fifth key is a
one-line dict addition; no new plumbing is needed.** This is the same
mechanism `--accel-boost` and `--dash-unclamp` already use to patch the
`FPV` dict *before* `apply_fpv_profile()` reads it into module globals
(`m4_intercept.py` lines ~2614–2639) — `--fpv-fast` should follow the
identical pattern (see §6).

## 3. The x500's own physical ceiling (a from-scratch derivation)

Two independent numbers agree, which is worth trusting:

**Mass** (`~/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf`):
`base_link` mass = 2.0 kg + 4× `rotor_N` links at 0.016077 kg = **2.0643 kg**
total → weight = 2.0643 × 9.81 = **20.25 N**.

**Max thrust** (`~/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf`,
the `gz-sim-multicopter-motor-model-system` plugin, 4 identical motors):
`motorConstant = 8.54858e-06`, `maxRotVelocity = 1000` rad/s →
thrust = `motorConstant × maxRotVelocity²` = **8.549 N/motor** × 4 =
**34.19 N total**.

→ **T/W = 34.19 / 20.25 ≈ 1.69 : 1.**

Cross-check: the x500 airframe file commits `MPC_THR_HOVER = 0.60`
(§2 table), and `T/W ≈ 1/MPC_THR_HOVER = 1/0.60 = 1.67 : 1`. **Two
independent derivations agree to ~1%** — good confidence in both.

**Steady, altitude-holding tilt ceiling.** To hold altitude at tilt angle θ,
the vehicle needs thrust `T = g/cos θ`. Setting that equal to the max thrust
budget (`T/W_max × g`) and solving: `cos θ_min = 1/(T/W) = 1/1.67 → θ_max ≈
53°`. **The FPV profile's own committed `MPC_TILTMAX_AIR = 60°` is already
past this steady-state physical ceiling** — not a new problem this doc
introduces, just an existing fact worth naming (PX4's attitude loop degrades
gracefully here: horizontal saturates or altitude sags slightly in a hard
transient turn; it doesn't crash).

**Max sustained lateral accel from T/W alone:** `g·tan(53°) ≈ 13.0–13.5 m/s²`.
This is **notably higher** than the ~6.7 m/s² ADR-0028-addendum actually
measured the guidance commanding — an independent, physics-derived
confirmation (not just a flight-log observation) that **the x500 has real
unused thrust margin today; the guidance ceiling, not the airframe, is what's
binding** (§4).

**No drag is modeled, anywhere.** Grepped every `.sdf` in the x500 model
tree for a plugin: only `gz-sim-multicopter-motor-model-system` appears (4×,
one per rotor). There is **no** `LiftDrag`/aerodynamics plugin on this
airframe. A real payload's parasitic drag (frontal area of a Pi 5 + camera
pod punching through the air at 25+ m/s) is **not represented at all today**
— no PX4 parameter can add it; only a new Gazebo drag plugin could, and
that's out of scope for a param-only `--fpv-fast` flag. Logged as an honest
gap, carried into T23.

**Caveat on this whole derivation:** it's a back-of-envelope, steady-state
(non-transient) analysis — real PX4 attitude/rate-loop dynamics, momentary
thrust-vectoring during a turn, and gz-sim's actual physics integration are
all more complex than "hover-thrust-budget algebra." Treat these numbers as
an order-of-magnitude sanity check, not a validated flight envelope — the
same status ADR-0010's own probe run gave its numbers before trusting them.

## 4. THE constraint to carry through everything below (ADR-0028)

ADR-0028's addendum ran exactly this experiment in real Gazebo: it doubled
`MPC_ACC_HOR_MAX` (12→20 m/s²) and bumped `MPC_TILTMAX_AIR` (60°→70°) via the
already-shipped `--accel-boost` flag, on top of the running-start geometry.
**Result: miss went 1.39→1.52 m — within noise, if anything worse.** The
root cause, confirmed from the flight's own telemetry: *the interceptor only
ever achieved ~6.7 m/s² lateral acceleration — well under even the
**default** 12 m/s² cap.* Doubling a ceiling nothing was hitting cannot help.
**The binding constraint was the guidance's own command ceiling:**
`V_PERP_MAX` (lateral/pro-nav velocity clamp) and `V_TOTAL_MAX` (combined
horizontal command safety clamp) — module-level Python constants in
`scripts/m4_intercept.py`, not PX4 parameters at all:

| Constant | Base M4 (no `--fpv`) | FPV profile (`--fpv`) | + `--dash-unclamp` |
|---|---|---|---|
| `V_PERP_MAX` (m/s, terminal pro-nav lateral clamp, ENGAGE only) | 3.0 | **8.0** | 8.0 (untouched — `--dash-unclamp` deliberately does not touch it) |
| `V_TOTAL_MAX` (m/s, combined-command safety clamp, both DASH and ENGAGE) | 4.0 | **13.0** | **18.0** |

(`scripts/m4_intercept.py` lines 268–269, 333–334, 478.)

**The rule for `--fpv-fast`:** it must raise **both** the PX4 param bundle
**and** these two guidance constants, or the PX4-side change is very likely
a no-op — this is not a hypothesis, it's a repeated, Gazebo-measured result
on this exact code path.

**The reverse caveat (this doc's own addition, flagged as such — not yet
tested):** raising `V_PERP_MAX`/`V_TOTAL_MAX` *without* a target fast/agile
enough to actually try to saturate them is **also** likely a no-op — nothing
in the ADR-0028/0029 straight-line 9–12 m/s geometry ever asked for more than
today's ceilings allow. `V_PERP_MAX` only matters once the guidance actually
tries to push past it. This is why the phase-2 plan lists T21 (12 m/s +
weave/jink arms) and T22 (this doc) in parallel: **the guidance-ceiling half
of `--fpv-fast` is best validated *together* with a T21-class faster or
maneuvering target, not against the existing 9–12 m/s straight-line suite
alone** — that suite already showed the ceiling doesn't bind at those speeds.

**The risk to A/B, explicitly (per the task brief).** Raising `V_PERP_MAX`
moves in the *opposite direction* from ADR-0043's `--terminal-gain-scale
0.35` lever, which *lowered* the terminal correction gain specifically to
roll off the markerless seeker's box-center bearing-noise throughput
(measured p90 2.8°/tick vs. the AprilTag's near-zero subpixel noise,
ADR-0042). ADR-0043 found that lever mechanically worked (achieved
`|a_cmd|` dropped 29.1→20.8 / 16.9→11.7 / 19.8→10.4 m/s²) but **didn't**
shrink the miss — the residual noise still passed through, and the ~0.4 s
terminal window is kinematically capped (ADR-0023) regardless. A **higher**
`V_PERP_MAX` ceiling could let that *same* noisy bearing signal command
*larger* excursions — plausibly widening terminal dispersion instead of
tightening it, the exact failure mode ADR-0043's lever was built to prevent.
**Any `--fpv-fast` validation must re-run the markerless clean-rate /
median-gap-to-tag gate (ADR-0042 G2/G3-style) alongside the "does it catch a
faster target" question** — a catch-rate win that quietly regresses
markerless terminal precision is not an honest win. See §7.

## 5. Real FPV airframe research — grounding the target envelope

Per the project's simulate-worse-than-ideal mandate, every number below is
given in three tiers, and **that exact 150–250 g payload build doesn't exist
yet** — nobody has published "5–7" quad carrying a Pi 5 + global-shutter
camera pod" numbers. So this section is an honest **engineering synthesis**
from adjacent real data (stripped racing quads with no comparable payload,
cinelifters carrying a much *heavier* 0.5–1.5 kg payload, and one real
integrated-camera airframe), not a single citable measurement — exactly the
gap NEXT.md's "Hardware Stage 0" bench work exists to eventually close with a
real number.

| Class | Config | Top/dash speed | Sustained accel | Source |
|---|---|---|---|---|
| **2.5" ("toothpick")** | own all-up-weight (AUW) **160–250 g including battery** | — | — | Oscar Liang toothpick guide; UAVMODEL sub-250g guide |
| 5" racing/freestyle, **stripped, no comparable payload** | reference only, not payload-realistic | 80–130 mph (36–58 m/s) | sub-1s 0-to-60 mph → **>~27 m/s²** | FPV Know-It-All 5" shopping list; general racing-drone press |
| **5"-class, real payload-carrying analog** — DJI FPV, ~795 g, fully integrated camera | closest real "5in + onboard payload" measurement that exists | **140 km/h (38.9 m/s)**, Sport mode | **0–100 km/h in 2 s ≈ 13.9 m/s² avg** | CineD, DJI FPV announcement |
| 7" cinelifter, carries **0.5–1.5 kg** camera payload (much heavier than our 150–250 g) | our payload is much lighter, so a 7" would retain *more* margin than these specs | up to 150 km/h / 41.7 m/s carrying 1.4 kg (Shendrones Thicc); 86 mph+/38 m/s (Lumenier, 9") | not published | Shendrones Thicc Cinelifter spec; Lumenier QAV-PRO Lifter spec |
| 7" long-range, **efficiency-tuned cruise** (a different regime — do not confuse with dash speed) | most-efficient loiter, not max punch | 40–60 km/h (11–17 m/s) | — | UAVMODEL long-range build guide |

**2.5" is EXCLUDED as a `--fpv-fast` candidate**, carried forward from
ADR-0012's ruling: "2.5" is a fantasy — avionics alone ~200-320 g > a whole
micro's AUW." A 150–250 g seeker payload roughly **doubles-to-triples** a
2.5"'s own weight; the real Stage-1 airframe is the 7" class (ADR-0012 #4),
with the stock x500 as the sim's bring-up-rig-parity check.

**Synthesized target tiers** (this doc's derivation, not a direct citation):

| Tier | Dash/top speed | Sustained lateral accel | Basis |
|---|---|---|---|
| **BEST** | ~35–38 m/s | ~14–16 m/s² | 7" cinelifter ceiling (41.7 m/s @ 1.4 kg) scaled up for our much lighter payload; stripped-5"-racing accel scaled down for a fixed avionics mass. |
| **EXPECTED** | ~26–28 m/s | ~9–11 m/s² | DJI FPV's real, measured, integrated-payload numbers (38.9 m/s / 13.9 m/s² avg), derated ~25–30% for a heavier non-integrated Pi5+GS-camera stack and non-race tuning. |
| **WORST** | ~16–18 m/s | ~5–7 m/s² | payload drag/mass penalty + battery sag under repeated hard maneuvers + conservative CG-shifted tuning. |

**The honest surprise:** WORST tier (~16–18 m/s, ~5–7 m/s²) sits almost
exactly on top of the sim's **current** operating point — a 16 m/s dash and
an ADR-0028-addendum-*measured* ~6.7 m/s² achieved lateral accel. **The
sim's current envelope may already approximate a real WORST-tier FPV build**,
even though its PX4 parameters are set far above that (`MPC_ACC_HOR_MAX =
12` — nearly double the achieved value). This reframes the whole exercise:
the interesting new work is chasing **EXPECTED** and **BEST**, not WORST.

## 6. Payload mass modeling

**Faithfully modelable via PX4 params alone (no SDF edit) — approximate:**
lowering `MPC_THR_MAX` below 1.0 and/or `MPC_ACC_HOR_MAX`/`MPC_TILTMAX_AIR`
below the FPV bundle's values proxies "less power margin," the same way a
real underpowered, overladen craft would be limited by its ESC/battery.
**This does NOT change the vehicle's physical mass, inertia, center of
gravity, or drag** — it only limits what the *controller* is allowed to ask
for. It under-represents the true penalty: a heavier craft is also harder to
*stop* (more momentum into the terminal phase), which a thrust derate alone
never captures.

**NOT faithfully modelable without an SDF edit:**
- **True mass increase** — a PX4 parameter cannot add kilograms; must edit
  `<mass>` in the `.sdf`, or (more physically honest) add a **new small child
  link** at the camera-mount pose with its own `<inertial><mass>` block, so
  the added rotational inertia and CG shift (a nose-forward payload changes
  pitch/yaw inertia asymmetrically — the sim's symmetric `MPC_TILTMAX_AIR`
  cannot represent that) are represented, not just total weight.
- **Aerodynamic drag** — no drag plugin exists anywhere in the x500 model
  tree (§3). Cannot be added via any PX4 parameter; needs a new Gazebo
  aerodynamics/`LiftDrag`-style plugin. Out of scope for a param-only flag.

**A fork to escalate, not decide here (per CLAUDE.md — this touches the
shared airframe every gated M0–M5/S1–S4 number depends on):**

- **(A) A new shadow model variant** (e.g. `models/x500_base_fpv_fast/`),
  following the *exact* precedent already set for the FPV portfolio-demo
  reskin — `models/x500_base` is already a repo-local resource-path shadow
  of PX4's real `x500_base` (ADR-0005 mechanism; material-only edits so far,
  ADR-0032). `--fpv-fast`'s mass work would add a payload link/mass edit to
  a **second** shadow, keeping the plain `x500_base` (used by every gated
  test today) byte-identical. Stronger repo precedent.
- **(B) A separate small static-mass model merged in via a world-level
  `<include merge='true'>`** — the same mechanism `Tools/simulation/gz/
  models/x500/model.sdf` itself uses to merge `x500_base` + add the motor
  plugins. One extra file, less duplication of the whole `x500_base` tree,
  but a less-trodden path in this repo.

**Recommendation (non-binding — flagged for the T22 build session or a
council if it proves contentious):** evaluate both at build time; do not
edit `x500_base` in place under any circumstances — every gated M0–M5/S1–S4
number depends on its mass being untouched.

## 7. Proposed `--fpv-fast` profile

Follows the exact established pattern (`--accel-boost`/`--dash-unclamp`):
requires `--fpv`, patches the `FPV` dict **before** `apply_fpv_profile()`
reads it into module globals, prints a before→after line, byte-identical
when unset. Proposed interface: `--fpv-fast {worst,expected,best}` (a new
`FPV_FAST = {"worst": {...}, "expected": {...}, "best": {...}}` dict).
`--dash-speed` stays a **separate**, already-existing flag the caller sets
independently within the new ceiling — `--fpv-fast` raises the ceiling; it
does not itself choose a flown speed. This deliberately preserves ADR-0028's
hard-won distinction between the **geometry lever** (running-start distance +
`--dash-speed`, `mc_batch.sh --y0-mag/--x0`) and the **airframe/guidance
ceiling lever** (this doc) — conflating them was exactly the mistake
ADR-0028's addendum existed to correct.

| Parameter | Current (`--fpv` + `--dash-unclamp`) | **WORST** | **EXPECTED** | **BEST** | Justification / source |
|---|---|---|---|---|---|
| `MPC_XY_VEL_MAX` (m/s) | 20 | 20 *(unchanged)* | 30 ⚠ | 40 ⚠ | Target dash + margin, §5. ⚠ = **past PX4's documented `@max=20`** — accepted by MAVSDK/firmware today (metadata bounds are UI hints, not enforced — verified by grep, no `math::constrain` to 20 found in the position-control path), but an **unverified excursion**; needs an ADR-0010-style probe before trusting in a batch. |
| `MPC_ACC_HOR_MAX` (m/s²) | 12 | 8 | 16 | 25 ⚠ | §5 target accel + margin. EXPECTED (16) sits just above the x500's own derived physical ceiling (~13–13.5 m/s², §3) — plausibly the stock airframe can *almost* deliver this with no mass edit; cheapest first experiment. BEST (25) is well past that physical ceiling on the stock x500 — expect a plateau (report ACHIEVED accel from telemetry, not the setpoint, mirroring ADR-0028's own rule) unless the §6 payload-mass/thrust work also raises the *physical* ceiling. |
| `MPC_TILTMAX_AIR` (deg) | 60 | 60 *(unchanged)* | 65 | 75 | Modest bump for EXPECTED (already past the ~53° physical steady-state ceiling either way, §3); BEST pushes further into aspirational territory, same plateau caveat. |
| `MPC_JERK_MAX` (m/s³) | 30 | 30 *(unchanged)* | 35 | 40 | Minor smoothing-ramp adjustment only. |
| `MPC_THR_MAX` (norm, 0–1) | 1.0 (never pushed) | **0.80** *(new)* | 1.0 *(unchanged)* | 1.0 *(unchanged)* | WORST-only: crude proxy for reduced power margin under an overladen payload (§6) — approximate, doesn't touch mass/inertia. Not included by default until §6's mass work exists (open question, §9). |
| `FPV["V_PERP_MAX"]` (m/s, guidance) | 8.0 | 8.0 *(unchanged)* | 10.0 | 14.0 | §4's rule: PX4 raises are a no-op without this. Sized to roughly track each tier's target sustained accel over the ~20 Hz control loop, not a first-principles derivation — needs its own probe. |
| `FPV["V_TOTAL_MAX"]` (m/s, guidance) | 18.0 (post `--dash-unclamp`) | 18.0 *(unchanged)* | 22.0 | 30.0 | Combined-command safety clamp; kept above the tier's target dash speed with margin, mirroring how 18.0 was sized to clear a 16 m/s dash today. |
| `--dash-speed` (CLI, unchanged flag) | 16 (current default) | 16–17 | ~27 (operator choice, within new ceiling) | ~36 (operator choice, within new ceiling) | Separate, existing flag — §5 target dash speed per tier. Not silently set by `--fpv-fast`. |

**WORST tier's honest headline:** almost nothing changes from today's
already-flown FPV+`--dash-unclamp` config (§5's surprise) — the one new
lever is the `MPC_THR_MAX` derate, itself flagged as approximate and
possibly premature (§9).

## 8. Validation plan sketch (nothing run yet)

A future Gazebo A/B for this profile should, when it's actually flown:

1. **Sim time only, never wall time** (ADR-0009) — every duration/rate/
   latency claim converted from the CSV's sim-time column, never `t`'s wall
   clock, which sags under load.
2. **Paired seeds, n ≥ 8 per cell, master-seed 42** for continuity with the
   existing ADR-0028/0029/0044-era batches — idle machine, one sim at a time,
   batch arms sequential (standing rules).
3. **Achieved-vs-commanded telemetry check first**, per tier, before any
   miss-distance conclusion — mirrors ADR-0028's own rule ("report ACHIEVED
   accel from the flight's own velocity trace against the commanded ceiling,
   not a bug if it plateaus"). Confirms whether each tier's PX4 params are
   actually being reached or just requested.
4. **Two separate catch-rate arms, not one:**
   - (a) Re-run the *existing* ADR-0028-confirm 9/12 m/s running-start
     straight-line config under the new ceilings — expected to be a **null**
     per §4's reverse caveat (nothing there ever tried to saturate today's
     ceiling either).
   - (b) A T21-class faster/maneuvering target (12 m/s+, weave/jink) — the
     scenario where the raised ceiling is actually likely to bind.
5. **Re-earn the markerless honesty/precision gate on every arm** — per-tick
   audit (`scripts/audit_per_tick.py`) + abort lens (`scripts/abort_lens.py`)
   as standing bars, **plus** an ADR-0042/0043-style clean-rate and
   median-gap-to-tag re-check under any `V_PERP_MAX` raise — the §4 risk,
   not assumed neutral.
6. **Report all three tiers separately; decisions must survive WORST**
   (project mandate) — don't headline a BEST-tier win if WORST regresses or
   is unreachable.
7. **Statistics honesty:** sign-test/Wilson-CI language at n=8/16, ~1 m
   single-flight terminal-dropout noise floor acknowledged, "not significant
   at this n" where it applies.

## 9. Open questions for the F1/T22 review

1. **Payload-mass SDF fork (§6):** shadow-model variant (A) vs.
   merged-include extra link (B) — needs a decision (council if contentious)
   before any true-mass work begins; this doc does not decide it.
2. **Should `--fpv-fast` raise `MPC_XY_VEL_MAX`/`MPC_ACC_HOR_MAX` past PX4's
   documented `@max` (20/15)?** Grep found no firmware-side clamp to those
   values in the position-control path, but that's not a guarantee across
   every code path PX4 touches. Needs an ADR-0010-style probe run before the
   EXPECTED/BEST tiers are trusted; if the excursion misbehaves, cap the
   tiers at the documented ranges instead.
3. **Does raising `V_PERP_MAX` (EXPECTED 10, BEST 14) regress the markerless
   clean-rate / median-gap-to-tag (ADR-0042/0043)?** Unknown until flown —
   the single biggest validation risk flagged in this doc (§4, §8 item 5).
4. **Should the WORST-tier `MPC_THR_MAX` derate (proposed 0.80) ship by
   default**, or wait until §6's true payload-mass modeling exists, so a
   crude throttle-derate isn't left standing in as the *only* WORST-tier
   payload representation indefinitely?
5. **Sequencing vs. T21:** validate `--fpv-fast`'s guidance-ceiling raise
   stand-alone first, or bundle it with T21's 12 m/s+weave/jink arms (where
   §4's reverse caveat says the ceiling is actually more likely to bind)?
   This doc recommends bundling but doesn't own the schedule call.
6. **This doc's tier numbers (§5, §7) are a design proposal, not yet
   dev-run-validated** the way ADR-0010's original FPV envelope was (a real
   probe: 16 m/s clean flight, pitch −12°, before it entered any gate).
   Before `--fpv-fast` enters `mc_batch.sh` defaults, each tier needs its own
   short probe flight, mirroring that precedent — this design doc is step
   one, not the last step.

## Honest limits

- **Two independent ceilings** (§1): raising PX4 params without raising the
  guidance ceiling is very likely a no-op (ADR-0028, measured); raising the
  guidance ceiling without a fast/agile-enough target to saturate it is also
  likely a no-op (this doc's own reverse caveat, untested).
- **No aerodynamic drag is modeled anywhere** in the x500 model tree — a real
  payload's wind resistance is structurally invisible to this sim no matter
  what parameter is tuned. Logged for the T23 sim-to-real gap audit.
- **True payload mass/inertia/CG-shift needs an SDF edit** this doc
  deliberately does not build or decide the shape of (§6, a flagged fork).
  Until that exists, `--fpv-fast` only changes *control* ceilings, not the
  vehicle's true physical response to a heavier, nose-loaded airframe.
- **PX4 parameter `@min`/`@max` metadata are QGC/MAVSDK UI hints, not
  firmware-enforced values** — this project's own MAVSDK push loop will
  happily set (and PX4 will use) values outside them. Every EXPECTED/BEST
  value above the documented max in §7 is an **unverified excursion**
  pending its own probe, not a validated flight envelope.
- **2.5" is excluded outright** (ADR-0012) — this doc's real-world numbers
  target the 7" deployable class, cross-checked against the 5" bring-up-rig
  class and one directly-measured payload-carrying analog (DJI FPV).
- **None of §5's real-world sources measure the actual target build** — a
  5–7" quad carrying exactly a 150–250 g Pi5+global-shutter-camera pod
  doesn't exist yet. The tiers are an honest synthesis, not a citation of a
  single number; Stage-0 bench hardware (`NEXT.md`) is what eventually
  replaces this with a real measurement, the same relationship
  `docs/stereo_design.md`'s bench-test section has to its own lab numbers.
- **The §3 physical-ceiling derivation is back-of-envelope**, steady-state
  algebra — a sanity check, not a substitute for a Gazebo probe.

## Sources

**Repo / this project:**
- `docs/phase2_sim_to_real_plan.md` — Track C, §2 technical grounding.
- `docs/decisions.md` — ADR-0010 (FPV realism upgrade + MAVSDK param
  mechanism, decision #3), ADR-0028 + addendum ("running start beats the
  airframe" / guidance-ceiling-not-airframe finding), ADR-0042 (seeker v2,
  terminal bearing-noise mechanism), ADR-0043 (terminal-gain-scale/freeze
  levers, both NULL), ADR-0012 (hardware stack, 2.5" ruled out), ADR-0005/
  ADR-0032 (`x500_base` shadow-model precedent + material-only reskin).
- `scripts/m4_intercept.py` — `FPV`/`S2` dicts (lines ~326–384),
  `apply_fpv_profile()` (355–368), `--accel-boost`/`--dash-unclamp` patch
  pattern (453–478, 2614–2639), MAVSDK param push+read-back loop
  (2849–2862).
- `NEXT.md` — "Hardware Stage 0" seeker payload parts (Pi 5 8GB + global-
  shutter camera).

**PX4 source (authoritative, this machine's `~/PX4-Autopilot`, tag
`v1.17.0`):**
- `src/modules/mc_pos_control/multicopter_position_control_limits_params.c`
  — `MPC_XY_VEL_MAX`, `MPC_TILTMAX_AIR`, `MPC_TILTMAX_LND`, `MPC_THR_MIN`,
  `MPC_THR_MAX`.
- `src/modules/mc_pos_control/multicopter_position_mode_params.c` —
  `MPC_ACC_HOR_MAX`, `MPC_JERK_MAX`, `MPC_VEL_MANUAL`.
- `src/modules/mc_pos_control/multicopter_position_control_params.c` —
  `MPC_THR_HOVER`.
- `src/modules/mc_pos_control/multicopter_autonomous_params.c` —
  `MPC_XY_CRUISE`.
- `ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500` — x500 airframe
  defaults, including `MPC_THR_HOVER 0.60`.
- `Tools/simulation/gz/models/x500_base/model.sdf` — link masses.
- `Tools/simulation/gz/models/x500/model.sdf` — `motorConstant`,
  `maxRotVelocity`, motor-model plugin (no drag plugin present).

**PX4 docs (prose context, may drift slightly from the pinned v1.17.0 tag):**
- Parameter Reference: https://docs.px4.io/main/en/advanced_config/parameter_reference
- Position Mode (Multicopter): https://docs.px4.io/main/en/flight_modes_mc/position
- Multicopter Setpoint Tuning (Trajectory Generator): https://docs.px4.io/main/en/config_mc/mc_trajectory_tuning
- Multicopter PID Tuning Guide: https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter

**Real FPV airframe research:**
- Oscar Liang, "Introduction to 2.5\" Ultra-light Micro Quad (Toothpicks)":
  https://oscarliang.com/ultralight-micro-quad-toothpick/
- UAVMODEL, "Toothpick and Ultralight FPV Build" (2026):
  https://blog.uavmodel.com/toothpick-and-ultralight-fpv-build-sub-250g-components-aio-boards-and-flight-performance-2026-guide/
- FPV Know-It-All (Joshua Bardwell), "5 Inch Freestyle FPV Drones & Parts":
  https://www.fpvknowitall.com/fpv-shopping-list-five-inch-freestyle/
- CineD, "DJI FPV Drone Announced — 0-100 KPH in 2 Seconds":
  https://www.cined.com/dji-fpv-drone-announced-0-100-kph-in-2-seconds-and-new-motion-controller/
- Shendrones "Thicc" Cinelifter product listing (150 kph / 1.4 kg payload):
  https://southwalesdrones.org/product/shendrones-thicc-cinelifter/
- Lumenier QAV-PRO Lifter 9" (86 mph+ cinema payload):
  https://www.getfpv.com/lumenier-qav-pro-lifter-rtf-ultimate-fpv-cinema-drone-bundle.html
- UAVMODEL, "FPV Long-Range Drone Build Guide" (2026, efficient-cruise
  40–60 km/h figure):
  https://blog.uavmodel.com/fpv-long-range-drone-build-guide-components-configuration-and-flight-strategy-2026/
- DRL RacerX Guinness World Record (163.5–179.6 mph, 800 g) — cited only as
  an extreme-outlier ceiling, **not** used in any tier (not payload- or
  build-comparable): https://dronelife.com/2017/07/14/drone-racing-league-drl-builds-fastest-racing-drone/
