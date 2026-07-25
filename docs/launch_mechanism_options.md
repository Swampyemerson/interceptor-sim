# Launch / release-mechanism options — design exploration

> **Status: DESIGN EXPLORATION, not a ratified decision.** Builder brainstorm
> (2026-07-21): "release mechanisms for the drone" — a hand-pointed launcher that
> auto-fires on NN detect; a manual launch + coded dash; a smart launcher with a
> higher-res/zoom camera that cues an intercept point over a link; or a hybrid.
> Evaluated with a 4-lens workflow (guidance/acquisition physics · jam-resistance/
> honesty · cost/hardware/human-factors · sim-testability) → adversarial completeness
> critic → synthesis. Nothing here changes a stage status in `project_state.json`;
> the canonical contract still governs. Ratify into `docs/decisions.md` only if the
> builder picks a direction. Companion: `docs/real_build_coded_dash.md` (the committed
> Option-B baseline), `docs/seeker_acquisition_range_note.md` (the reach physics),
> `docs/deployment_phases_design_brief.md` (the launch-on-detect / ground-cue parent
> concept), `docs/kill_mechanism.md` (the ram bar).

## The three options as posed

- **A — Autonomous launch-on-detect from a hand-held/loosely-mounted perch.** The
  interceptor sits on the launcher with its camera live and the NN running; you point
  it *roughly* at the incoming drone; it **auto-launches** the instant the NN holds a
  lock and dashes at the detected bearing.
- **B — Manual human-triggered launch + coded open-loop dash → onboard camera-only
  pro-nav terminal.** You see the drone, pull the trigger, it dashes on the pre-set
  heading until the onboard camera acquires and pro-nav takes over. **This is the
  current committed real design** (`real_build_coded_dash.md`).
- **C — Smart launcher with a higher-res / zoomed camera + datalink cue.** The
  launcher's better camera acquires first (farther than the drone's own wide camera),
  computes an estimated intercept point / lead, and **transmits it** to the interceptor
  before its own camera can see the target.

## Bottom line

**Build B; steal the one honesty-clean, jam-safe ingredient from A and one from C;
discard A's auto-fire and C's live datalink.** The launch choice is *largely
orthogonal to the actual hard problem*, for two reasons the options framing hides:

1. **The binding wall is POINTING, not launching or detecting.** The onboard detector
   works ~100% when the camera is static (8–22 m), but only ~0.8% in flight (2 real vs
   1187 phantom detections across 63 flights). Root cause: the ~35–40° nose-down dash
   pitch parks the target at the top edge / out of the frame — 75% of ticks in the
   8–12 m band have the target **out of frame** (`inview_probe_results.md`, settled
   2026-07-21). The committed fix is a **FIXED up-tilt wedge** sized to the measured
   dash pitch (builder FINAL 2026-07-17; adaptive tilt REJECTED for simplicity;
   `project_state.json` `pointing`), and it is **not yet validated**. No launch
   mechanism changes airframe pitch or camera elevation — A, B, and C are all
   downstream of that wedge.
2. **"Kill" means a ~0.35 m RAM contact** (ADR-0084, the ordered 5-inch pair; hit-to-kill, $0 payload; `kill_mechanism.md`
   §1) — the least-forgiving mechanism. Every miss/Pk number the options were first
   ranked on used a **Pk@2.5 m** proximity gate, a bar ~3–5× looser than a ram. A
   0.78 m "Pk@2.5 win" is a ram **miss**. The camera-guided 3D-quad kill is **not yet
   demonstrated in sim at any bar** — today's apparent kills are open-loop dash
   ballistics (a dash-only control with zero real ENGAGE detections scored the same,
   `real_build_coded_dash.md` #18g/#18h).

So the honest sequencing is: **prove the fixed-tilt wedge lets a camera terminal exist
at all (one cheap sim experiment) before spending a dollar on launcher hardware.**

## Per-option verdict

| Option | Verdict | Best use of its idea |
|---|---|---|
| **B** (committed) | The correct near-term build: cheapest (~$0), the only fully-instrumented option, jam-proof, human-in-the-loop. Its only real gap is hand-aiming to the tight kill tolerance — and, like every option, it still owes the unvalidated tilt fix. | The platform every onboard-perception fix (the tilt re-fly, a pitch-programmed dash) is built and validated on; the audited jam-resistance reference. |
| **A** (auto-fire) | **Dominated as a standalone.** Adds *zero* acquisition range; reaction-time-starved (the onboard cam only sees ~16 m ≈ ~1.8 s vs a 9 m/s inbound, before takeoff+dash even happen); does **not** touch the elevation wall; and auto-firing a spinning-prop ram on an unmeasured outdoor false-positive rate is the least-defensible ROE. | Keep only its real payload: at trigger, latch the ~100%-reliable **static camera bearing** into the dash azimuth as a pre-flight constant (machine-accurate aim, ~0.1°/px). Human keeps the fire decision. |
| **C** (live cue) | **Worst as stated** — a live mid-course datalink is self-refuting for the jam-resistance headline (it *is* the link an adversary jams), was already measured to **hurt** aim (r2l 0/8 with the cue vs 5/8 with it dropped, from an aspect-biased bearing), and re-imports the ground cue the sim deliberately mocked away. Its reach physics is real but a pinhole **upper bound** (unbenched). | Demote to a **zero-radio, one-shot pre-launch spotter** (or a human gunsight): solve a lead ONCE at trigger, latch it as a pre-flight constant, let the link die at launch. Adopt only if a field test shows humans can't hand-aim to the ram bar. |

## Recommended hybrid — "B-core + honesty-clean launch-time aim stack"

Keep the committed B pipeline unchanged (human trigger → coded open-loop dash on a
pre-flight-constant heading → AprilTag-first seeker, then camera-only pro-nav → ram
kill → breakoff → land). Resolve the three tensions by pulling **only** the jam-proof,
honesty-clean pieces off A and C:

- **Human trigger + deadman interlock** — a person owns weapons release (most
  defensible ROE; removes A's auto-fire hazard).
- **Coded open-loop dash** on a pre-flight-constant heading (`collision_lead_heading`
  + crossing bias — all pre-flight constants, honesty-clean, validated to ≥30°
  acquisition tolerance).
- **Fixed up-tilt wedge** (10–30° adjustable, sized to the measured dash pitch, on the
  prop-clearance boom — tilt and mount are one mechanical decision; >~15° exceeds the
  flush mount's ~11° clearance). **The real wall fix — validation-gated, not assumed.**
- **AprilTag first-kill seeker** on the Pi 5 CPU (9–12 m decode, 30+ FPS); YOLO
  markerless runs in shadow mode until it earns the flight.
- **Lock-then-launch static bearing latch** (Option A, software only, ~$0): if the
  static detector holds its 5-fresh-detection streak at trigger, take the dash azimuth
  from the camera bearing; else fall back to the sighted/hand heading.
- **Aim aid**: seeker-video-on-phone "gunsight" ($0, removes sight-to-camera boresight
  error by construction) and/or a boresighted red-dot ($15–40).
- **OPTIONAL, field-test-gated**: a **ground zoom spotter** (2nd OV9281 + narrow lens
  + tripod + laptop, **no radio in the guidance loop**) that solves a one-shot
  collision-lead heading from a few seconds of pre-launch tracking (two-operator split:
  spotter tracks, trigger-holder fires), latched as a pre-flight constant. If jammed or
  absent, fall back to pure B — with a **mandatory link-denied test arm**.

**Why jam-resistance survives:** nothing enters the guidance loop after launch-latch.
Every aim input (static camera bearing, optional ground-spotter lead) is a *single
pre-flight constant* baked in at trigger; every RF link dies at launch and is allowed
to be jammed. Guidance reads only camera + own-state EKF. A **mandatory link-denied
Monte-Carlo arm** *demonstrates* the camera terminal completing with no cue — so the
headline is earned, not asserted.

## Cost / constraint comparison

| Option | Hardware added | Approx Δ$ | Weight / complexity | Jam-resistance | Acquisition gain |
|---|---|---|---|---|---|
| **B** (baseline) | printed launch plate; optional red-dot or $0 phone gunsight | +$15–55 | 0 g added; LOW (no new mode/radio/sensor) | **Strongest** — nothing in the loop to jam | none added; human eyeball is the longest-range cue; aim rests on a human point |
| **A** (auto perch) | perch/tripod + deadman (min); rail + hook if boosted | +$30–65 min; +$40–90 boosted | 0 g un-boosted; HIGH boosted (custom PX4 launch mode + shock-hardened boom) | Strong (no radio) but weak ROE, unmeasured outdoor false-fire rate | **zero** added range; reaction-time-starved; only converts cold search → lock-maintenance |
| **C** (live cue) | 2nd camera + narrow lens, ground compute, RF datalink, calibrated mount | +$150–260 | ~0–20 g airborne; MED–HIGH (2nd calibration chain, boresight, mono ranging, link protocol) | **Destroyed** as a live cue; OK only in the one-shot latched recast | real 2–4× range (30–65 m) but a pinhole **upper bound**; only shrinks required ZEM, grows nothing onboard |
| **Hybrid** (recommended) | plate + gunsight; optional ground spotter (no radio); validation-gated fixed-tilt wedge | +$15–55 base; +$70–130 with spotter | 0 g airborne; LOW–MED (spotter is ground-side) | **Fully preserved** — single pre-flight constant, link dies at launch, mandatory link-denied arm | azimuth aim measured to ~0.1°/px from the reliable static regime; optional ground reach as a one-shot lead; elevation wall owned by the (unvalidated) fixed tilt |

## Alternatives worth more than any of the three (surfaced by the critic)

- **2-axis seeker gimbal.** The *only* mechanism that fully decouples the seeker from
  airframe pitch across the whole dash (the fixed wedge admits it **cannot** track the
  +40°→−35° accel/brake pitch swing) — and the only thing that could revive Option A's
  "lock-carry through the dash." Cost/weight/complexity on a sacrificial ~950 g vehicle
  is the real question; est. +$40–120, unpriced in the repo. **A hypothesis, not a
  proven fix.**
- **Dual ONBOARD cameras (wide cue + narrow terminal).** Put the two-stage handoff
  (ADR-0013) entirely onboard: a wide ~100° cam keeps the crosser in frame (dodging the
  ADR-0024 narrow-FOV backfire) and cues a second narrow onboard cam that extends
  terminal reach. Captures **C's acquisition-range benefit while staying 100% jam-proof
  and datalink-free** — arguably dominates C on the headline. ~+$36 camera + concurrent
  Pi compute + weight. Deserves a first-class test.
- **Forgiving kill (net) instead of a ram.** A net capture has a ~1.5–2 m radius
  (`kill_mechanism.md` §2), which **largely dissolves** the pointing/acquisition wall —
  reframes the project from "earn 0.5 m onboard" to "earn ~2 m." Cost: ~370 g net
  payload forcing a 7-inch airframe re-scope. **Decide this before optimizing the launch
  mechanism** — it sets how hard the whole problem is.
- **Re-rank for the AprilTag first-kill regime.** Every lens ranked against the hardest
  seeker (markerless NN, 0.8% in flight) — but first flights fly the **tag** (9–12 m
  decode). The tag's longer in-frame range may make the wall far less binding and change
  which option is even worth building. Cheap to re-check in the existing harness.

## Honesty flags (must be disclosed)

- The camera-only terminal is **currently unproven**: today's "kills" are open-loop
  dash ballistics. The headline ("datalink denied → the camera finishes it") is an
  honest *architecture* claim, not yet a *demonstrated* result.
- The kill is a **ram (0.35 m, ADR-0084)**. Re-score existing batches against the ram radius,
  not Pk@2.5 m, before claiming progress toward a kill.
- The **fixed up-tilt wedge is committed-in-direction but NOT validated**
  (`project_state.json` `pointing` = half-done, active_version None); adaptive tilt is
  rejected. Higher-res / foveated crop are likewise hypotheses with a test attached, not
  proven. (Doc drift: `real_build_coded_dash.md` and the BOM header still cite adaptive
  tilt as "the dominant lever" — reconcile to the contract.)
- Any launcher cue is a **new sensor path**: disclose it, latch it as a single
  pre-flight constant, and re-earn the machine-checked "no post-latch cue read" audit. A
  persistent live datalink is forbidden in the guidance loop.
- Acquisition-range numbers (onboard ~16 m; launcher 30–65 m) are **pinhole upper
  bounds** — real `w_floor`, motion blur (terminal LOS 485–1870°/s → ~23 px smear at
  5 ms), MTF, and low-light SNR are Stage-0-bench-only. "Static ~100% / 0 phantoms" is a
  clean-gray-sim result; the outdoor bird/clutter false-positive rate (active, ADR-0078)
  is unmeasured and load-bearing for any auto-fire.
- **Wind/gust is unmodeled by every lens** — it pushes the open-loop dash off track and
  rotates the airframe (moving where the fixed-tilt camera points), eating the two things
  every option depends on. A universal first-order degrader, untested.

## De-risking experiments (sim first, cheapest-decisive first)

1. **The one decisive batch — fixed up-tilt × coded-dash × dash-only-ballistics
   control** (sim). Decides whether a camera terminal exists at all. Pre-registered bar:
   gt-consistent ENGAGE detections > 0 AND camera-tracked CPA beats the dash-only control
   on paired seeds (n≥8/dir), scored with the tilt-aware gt labeler. Also logs A's
   lock-survival-through-the-transient as a free column. **Highest evidence per token.**
2. **Ram-bar re-scoring of existing batches** (sim analysis, $0). Re-tally every batch
   against the ratified 0.35 m (ADR-0084) to establish the true accuracy gap and whether
   onboard t_go² capacity can ever net < 0.35 m — i.e. whether a forgiving net kill is warranted.
3. **Lock-then-launch AIM A/B** (sim). Dash heading from a scripted "perch detection"
   bearing vs the human-error sweep arms — quantifies how much a machine-accurate launch
   azimuth tightens the miss, no hardware.
4. **One-shot vs persistent cue A/B incl. a link-DENIED arm** (sim). 3 arms (baseline /
   one-shot launch-time cue / persistent datalink) across cue-noise + latency tiers. The
   link-denied arm *is* the jam-resistance deliverable. Likely collapses C into the
   one-shot spotter.
5. **Pitch-program / lofted-dash sweep** (sim). The software control any tilt hardware
   must beat: cap dash accel/speed or loft the profile to bound nose-down pitch inside
   the camera's upper FOV; sweep 6/9/12 m/s vs gt-consistent in-flight recall.
6. **Field aim-error rig** (bench, ~$0). Can an operator hand-aim the plate to the ram
   tolerance? Point at a flown target N times, read achieved heading error from the ULog.
   Decides whether a gunsight/ground spotter is mandatory and sets its spec.
7. **Stage-0 seeker bench + boom-boresight-stability bench** (physics-limited). Real
   `w_floor` / P_detect(R), motion blur at short exposure, SNR, Pi 5 Hz/latency, the
   AprilTag decode-envelope curve — and whether the sacrificial nose boom holds boresight
   under dash load (the calibration the whole camera-only LOS assumes).

## Open decisions — genuinely the builder's

- **ROE / weapons-release autonomy** — human-triggered (recommended) vs autonomous
  launch-on-detect. The one-way ethical/safety door: an auto-spun-up spinning-prop ram
  firing on an NN lock outdoors with an unmeasured false-positive rate.
- **Kill mechanism / accuracy bar** — keep the RAM (0.35 m, ADR-0084; forgives nothing) vs a
  forgiving NET (~1.5–2 m, but ~370 g → a 7-inch airframe + one-shot payload). Sets how
  hard the whole acquisition problem is. **Decide before optimizing the launcher.**
- **Buy the ground zoom spotter at all?** — only if the field aim-error test says humans
  can't hit the ram bar; keep it strictly one-shot / no-radio if adopted.
- **Invest in a 2-axis seeker gimbal?** — the only full elevation-decoupler (revives
  lock-carry), vs living with the fixed wedge's admitted limit. Cost/weight on a
  sacrificial vehicle is unpriced.
- **Legal go/no-go** — FAA registration + broadcast Remote ID on both >250 g aircraft
  (or fly at a FRIA field) before any outdoor flight.
- **Sequencing** — how much sim effort to spend re-ranking A/B/C against the AprilTag
  first-kill regime (what actually flies first) rather than the markerless NN the current
  ranking assumes.

---

*Provenance: 4-lens evaluation workflow (`launch-mechanism-eval`, 6 agents) →
adversarial completeness critic → synthesis, 2026-07-21. All numbers trace to
`real_build_coded_dash.md`, `seeker_acquisition_range_note.md`, `kill_mechanism.md`,
`inview_probe_results.md`, `deployment_phases_design_brief.md`, and
`project_state.json` (`pointing` / `terminal` stages). This note ranks options; only a
gated sim/bench run turns a ranking into a decision.*
