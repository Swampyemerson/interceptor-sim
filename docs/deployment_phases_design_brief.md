# Deployment-profile phases M-1..M-4 — design brief

> **Status:** DESIGN ONLY. No code changes, no sim boots, in the writing of this
> document. This is the "design-as-ADR when picked up" work docs/next.md has parked
> since 2026-07-05 (originally logged in commit `1877ca9`, folded into the
> post-M5 roadmap's Parked list — docs/next.md; referenced by ADR-0028's "the
> deployment roadmap (docs/next.md M-1..M-4)" strategic-implication line). It follows
> the same brief pattern as `docs/seeker_design_brief.md` and
> `docs/ekf_design_brief.md`: recommendation-first design questions, an
> evidence-grounded evaluation plan, and a ratifiable ADR skeleton at the end.
> **Nothing here is built. Every "RECOMMENDATION" is a proposal for the main
> session/builder to ratify, not a decision already taken.**

New terms, one line each (defined once, used freely after):
- **Ground standby** — the interceptor sits armed-or-disarmed on the ground,
  motors off, before any threat is detected. The state the sim currently
  **skips**: every flight arms and climbs unconditionally at script start.
- **Launch-on-detect** — the ground sensor's *own* detection (not a script
  flag) is what triggers the arm/takeoff command. Today nothing gates the
  existing arm call on any sensor state.
- **Climb-out / boost** — the vertical or oblique acceleration phase from pad
  to cruise altitude/speed, before the horizontal pursuit (DASH) begins. docs/goals.md's
  aspirational phrase; not modeled today (see §1.5 — the whole engagement,
  interceptor and target both, currently flies at a fixed 0.5 m altitude).
- **Reaction latency** — the time from "the ground sensor could see the
  threat" to "the interceptor commits to launching." Distinct from the
  **cue-delivery latency** ADR-0016 already measures (how stale each
  individual position/velocity *sample* is once the interceptor is already
  flying and consuming the cue stream).
- **T_total / end-to-end timeline** — the M-4 headline: sim-time-stamped
  duration from first ground-rig detection to CPA (closest point of
  approach), decomposed into named phase durations.

---

## 0. Executive summary (recommendation-first)

M-1..M-4 is **not a new capability** — it is a **relabeling and honest
completion of plumbing that already exists and already runs in every single
logged flight this project has produced.** `m4_intercept.py` already arms,
commands PX4's native takeoff, and climbs to `ALT_REF_M` before the tracked
phase machine (`CUE_WAIT → DASH → ENGAGE → BREAKOFF`) begins (`m4_intercept.py:2864–2899`)
— it is logged as an unlabeled `"TAKEOFF"` phase in every CSV, and it
already costs a real, measured **~21.2–21.7 s of sim time** (six recent
`pronav` flights, see §1.2). ADR-0028/0028-addendum's "running start" is
already a Gazebo-confirmed approximation of launch-on-detect + long dash
(geometry-only: longer standoff + faster dash, no cold-start timing). What is
genuinely missing is **(a)** gating that arm command on an actual detection
signal instead of firing it unconditionally at script start, **(b)** naming
and reporting the phases that already exist as a first-class timeline metric,
and **(c)** deciding whether "climb-out" needs real vertical motion or stays
the sim's existing flat-altitude approximation.

**Recommendations, in one line each (full reasoning in §2):**
1. **No physical launcher/catapult model.** The x500 is a quadrotor — it
   already takes off vertically under its own power; a launcher is a
   fixed-wing concept this project does not need. Parameterize the *time
   cost* of cold-start (arm + motor spin-up + climb), don't build a launch
   rail.
2. **A thin orchestrator wraps `m4_intercept.py`, not a rewrite of its
   phase machine.** The phase machine already carries every gate (S2/M4/M5)
   and audit tool this project has; the CLAUDE.md byte-identical-default-path
   rule argues for composing in front of it, not editing inside it.
3. **Reuse ADR-0016's latency tiers + the existing ACQUIRE-phase
   confirm-streak pattern for the launch decision — do not build a new
   ground-detection-probability-vs-range model.** That model is explicitly
   Stage-0/bird-classifier scope (ADR-0035) and reopening it here is scope
   creep this brief should not invite.
4. **Keep the flat 0.5 m altitude convention for M-3's first cut; treat
   "climb-out" as a labeled *speed* ramp inside the existing DASH mechanism,
   not a new vertical-motion capability.** Flagged as genuinely open — see
   Fork 4 and open question #1.
5. **T_total is a phase-stamped sim-time sum, reported conditioned on clean
   intercepts, never blended with misses** — the same discipline ADR-0031
   already established for handoff-rate under a degraded cue.

---

## 1. What M-1..M-4 mean, and what the sim already proves adjacent to them

### 1.1 Origin

M-1..M-4 were defined by the builder on 2026-07-05 (commit `1877ca9`,
"Real-world deployment roadmap"): *"ground-standby → launch-on-detect →
max-speed intercept"*, motivated by the parent hardware project's actual
question ("X s from cue to airborne", "launches up and is max speed until
interception"). The four items as originally written:

- **M-1 Ground standby (cold start).** Model the interceptor armed on a pad,
  motors off — not the sim's hover-start. Adds an arm→spin-up→liftoff
  sequence and a reaction-latency budget.
- **M-2 Launch-on-detect trigger.** The ground cue firing a threat detection
  is what *launches* the interceptor — today the cue only steers an
  already-flying drone.
- **M-3 Max-speed climb-out + dash.** Vertical/oblique boost to altitude,
  then full-speed run-in to terminal.
- **M-4 Standby→intercept end-to-end timeline.** One logged run: cue-detect →
  launch → climb → dash → handoff → terminal → CPA, sim-time-stamped per
  phase — "the headline reaction + intercept time figure."

Since that commit, three years' (twelve days', sim-time-wise) worth of
guidance/perception work happened *around* these items without building them:
ADR-0028's running start, ADR-0030's dash-track fix, ADR-0031's
perception-availability envelope, and ADR-0036's final M5 batch all fly a
profile that is a **partial, geometry-only realization** of M-2/M-3 (see
§1.4/1.5). M-1..M-4 as a design item is therefore narrower today than in
2026-07-05 — most of the *guidance* substance is already built and gated; what
remains is the *timeline/launch-decision* framing.

### 1.2 The load-bearing fact: TAKEOFF already exists, unconditionally, unlabeled

`m4_intercept.py` — the single script every gate (S2/M4/M5) and every batch
arm flies — does this at the start of **every** flight, before entering
`CUE_WAIT`/`ACQUIRE` (`m4_intercept.py:2864–2899`):

```python
await drone.action.set_takeoff_altitude(ALT_REF_M)   # ALT_REF_M = 0.5 (line 247)
await drone.action.arm()
await drone.action.takeoff()
# ...then poll until relative_altitude_m >= 0.8 * ALT_REF_M, writing
# write_row_m4(..., "TAKEOFF", ...) rows the whole time (line 2888)
```

This is a real PX4 arm + native takeoff sequence, and it is **already logged**
as a `"TAKEOFF"`-phase row in every CSV this project has ever produced — it
has just never been reported as its own metric. Pulled directly from six
recent `pronav` flights (`logs/m4_intercept_pronav_20260708T2140{37,42,46,49}Z.csv`,
`...214600Z.csv`, `...214656Z.csv` — read via `awk` on the `phase`/`t_sim`
columns, no sim boot needed for this measurement):

| flight | last `TAKEOFF` row `t_sim` |
|---|---|
| `...214037Z` | 21.616 s |
| `...214142Z` | 21.528 s |
| `...214246Z` | 21.684 s |
| `...214349Z` | 21.156 s |
| `...214456Z` | 21.200 s |
| `...214600Z` | 21.716 s |

**~21.2–21.7 s of sim time, every flight, already spent arming and climbing
to 0.5 m before the tracked phase machine starts.** This is the honest
starting point for M-1/M-2: the "cold start" isn't a hypothetical to build
from scratch, it's an existing, measured cost that is currently (a)
unconditional — nothing gates it on a detection — and (b) unlabeled in any
reported metric.

### 1.3 M-1 — Ground standby (cold start)

**What exists:** the arm/takeoff sequence above IS a cold-start sequence —
the vehicle spawns disarmed-equivalent (motors not spinning) and the log
already timestamps arm→hover. **What's genuinely new:** (a) making the arm
command *conditional* rather than immediate (§2, Fork 3) — right now
"standby" has zero duration because nothing is waited on; (b) a disclosed
reaction-latency budget layered before the arm call, sourced from ADR-0016's
already-measured cue-delivery latency tiers (BEST ~20 / EXPECTED ~90 / WORST
~210 ms) plus a confirmation requirement (not a single noisy sample); (c)
reporting the standby+liftoff duration as its own named metric instead of an
unlabeled `TAKEOFF` phase buried in the CSV.

### 1.4 M-2 — Launch-on-detect trigger

**What exists:** ADR-0028's running start (`--y0-mag 29.3 --x0 6.5`) and
ADR-0030's `--emit-velocity --cue-velocity --dash-unclamp` fix already prove
the *consequence* of a good launch-on-detect decision — a longer standoff +
faster dash cuts the 9 m/s miss ~47–59% (ADR-0028 addendum, ADR-0030) — but
they prove it by **starting the geometry further away, not by modeling the
detection-to-launch decision itself.** The arm/takeoff at script start (§1.2)
happens regardless of any cue state; `CUE_WAIT` (the *first* phase that reads
the cue at all) begins only after the vehicle is already airborne
(`m4_intercept.py:1591,1912`). **What's genuinely new:** wiring the cue's own
existence/confirmation into the arm decision, i.e. moving the trigger from
"script start" to "cue confirms a track" — see Fork 3 for exactly what
"confirms" should mean without inventing a new ground-detection model.

### 1.5 M-3 — Max-speed climb-out + dash

**What exists — most of the substance.** The `DASH` phase (`m4_intercept.py:1967–2230`)
already is a "boost to max speed then run-in" mechanization: lead-pursuit
(PIP-style intercept-triangle solve) at a fixed `DASH_SPEED` (default 10
m/s, raised to 16 m/s on the adopted deployment profile via `--dash-speed 16
--dash-unclamp`, ADR-0030), continuing until `HANDOFF_RANGE_M` triggers the
one-way latch into `ENGAGE`. ADR-0028-addendum found the airframe achieves
only ~6.7 m/s² lateral against a 12 m/s² cap — **not airframe-limited** —
which is directly relevant here: an M-3 "boost" does not need a more
powerful motor/prop, the existing x500 has headroom the guidance isn't using.

**What's genuinely new — and this is the one honest gap worth naming
plainly:** the *entire* engagement — `TAKEOFF`, `CUE_WAIT`, `DASH`, `ENGAGE`
— flies at a **constant 0.5 m altitude** (`ALT_REF_M = 0.5`,
`m4_intercept.py:247`; every `mc_batch.sh` target spawn is also
`z: 0.5`, e.g. `mc_batch.sh:621`). There is **no vertical climb-out at
all today** — docs/goals.md's "launches up" is aspirational, not modeled. This is
disclosed (ADR-0010's altitude choice was made to keep the camera-to-tag
vertical offset out of the 3D miss budget, not to model real launch
geometry), but M-3 as literally specified ("vertical/oblique boost to
altitude") would be the first item in this whole project to touch that
convention. See Fork 4 — flagged as genuinely uncertain, not a clean
recommend-and-move-on.

### 1.6 M-4 — Standby→intercept end-to-end timeline

**What exists:** every phase transition is already sim-time-stamped in the
per-flight CSV (`t_sim` column, present on every row since ADR-0009's RTF
discovery mandated sim-time scheduling) — `TAKEOFF`, `CUE_WAIT`, `DASH`,
`ENGAGE`, `BREAKOFF` are all distinguishable `phase` values in every flight
this project has flown. **What's genuinely new:** (a) aggregating those
per-phase durations into a reported timeline metric across a batch (nobody
has ever pulled the numbers in §1.2's table before this brief); (b) defining
what "first ground-rig detection" means when the sim has no
detection-probability-vs-range model for the *ground* sensor (only
per-sample noise/dropout, `s2_cue_mock.py`) — see Fork 3; (c) deciding how
T_total composes with the ratified Pk-vs-radius headline (ADR-0025) without
producing a misleading blended number — see Fork 5, and the ADR-0031 lesson
it must not repeat (raw miss averaged across broken/dash-aborted flights
*flattered* the broken arms; the same trap exists for a timeline average
blended across clean and missed flights).

---

## 2. Design forks

Each fork: options considered, **RECOMMENDATION**, why, and risk/uncertainty
carried forward.

### Fork 1 — Model the launcher/boost phase physically, or teleport/approximate it?

- **Option A — physical launcher/catapult.** Build an SDF joint/impulse
  mechanism that imparts an initial velocity, model PX4's recovery from a
  ballistic launch.
- **Option B — teleport/approximate.** Keep the existing MAVSDK
  `arm()`/`takeoff()` (already proven, §1.2), parameterize its *time cost*
  (spin-up delay, climb rate) as a tunable knob rather than new physics.

**RECOMMENDATION: B.** The x500 is a **quadrotor** — it already takes off
vertically under its own power (that's the entire point of a multirotor);
catapults/launch rails are a fixed-wing or tube-launched-munition concept
this project's airframe choice never needed. ADR-0028-addendum already
measured the x500 has *un-used* lateral-accel headroom (6.7 of 12 m/s²
achieved) — there is no physics gap a launcher would close. Building SDF
launch-rail physics would (a) touch airframe modeling docs/goals.md scopes as
"no hardware" territory, (b) risk PX4 attitude-recovery instability after an
external impulse (untested, unbounded engineering risk for a cosmetic gain),
and (c) buy nothing the existing arm/takeoff sequence doesn't already prove.
**Risk:** none identified beyond "the resume phrase 'launches up' reads less
dramatically" — a naming/framing cost, not an engineering one.

### Fork 2 — Extend `m4_intercept.py`'s phase machine, or a new orchestrator?

- **Option A — extend the phase machine.** Add `STANDBY`/`ARM_WAIT` states
  before `CUE_WAIT` directly inside `m4_intercept.py`.
- **Option B — thin orchestrator wrapper.** A new, small script that handles
  standby + the launch-decision gate, then either `exec`s or subprocess-calls
  the existing `m4_intercept.py` (which keeps flying from its own
  `CUE_WAIT`/`DASH` start exactly as today).

**RECOMMENDATION: B.** `m4_intercept.py`'s phase machine is the single most
heavily gated, audited surface in the repo — S2/M4/M5 numeric gates,
`audit_per_tick.py`'s (a)-(d) checks, `abort_lens.py`'s post-CPA
reclassification, and the whole honesty-boundary test suite all key off its
exact phase-transition semantics (ADR-0041's "one-tick DASH→ENGAGE grace"
calibration is a concrete example of how sensitive that machinery already is
to phase-boundary timing). CLAUDE.md's byte-identical-default-path rule
(re-verified at every prior port — seeker, EKF, fusion) argues for composing
new behavior **in front of** that machine rather than inserting new states
**inside** it, so every existing gate stays untouched by construction rather
than by re-verification. A wrapper also cleanly isolates the one genuinely
new piece of logic (the launch-decision gate, Fork 3) from code that already
carries five ADRs' worth of tuning history.
**Uncertainty flagged:** whether the wrapper should `import` `m4_intercept`'s
module and call its `main()` in-process, or `subprocess.run` it as today's
scripts do (mc_batch.sh's own pattern) is a low-stakes implementation detail,
decidable at build time — not escalated here.

### Fork 3 — Where does the ground-rig detection/launch-decision latency model come from?

- **Option A — build a new `P_detect(R)` model for the ground rig's
  *initial* acquisition** (how far out does the ground sensor first flag a
  hostile), analogous to the camera's own detection envelope (ADR-0024/0042).
- **Option B — reuse what's already measured: ADR-0016's cue-delivery
  latency tiers (BEST/EXPECTED/WORST) as a fixed reaction-time budget, plus
  the existing confirm-streak pattern** (`ACQUIRE_MIN_DETECTIONS = 10`
  consecutive fresh detections, `ACQUIRE_CENTERED_STREAK = 6`,
  `m4_intercept.py:284–291` — the camera side's own "don't trigger on one
  noisy sample" logic) applied to the cue's first N valid ticks before
  arming.

**RECOMMENDATION: B.** Option A duplicates scope that is **explicitly and
deliberately gated behind hardware** — ADR-0035's bird-vs-hostile
discrimination doctrine is exactly a ground-detection-confidence model, and
it is *pinned* to VETO-ALL until the Stage-0 bench measures real
`P_correct(R)` numbers (ADR-0035 §0). Building a parallel, ungrounded
detection-range model here to serve a timeline metric would either (a)
silently duplicate that pending work with made-up numbers, which fails the
"numbers trace to a run or derivation" rule, or (b) quietly imply a launch
decision more sophisticated than the sim can honestly claim. Option B reuses
two already-sourced, already-tested patterns — ADR-0016's latency budget
(sourced to a real radio-link/compute-chain arithmetic) and the ACQUIRE
phase's confirm-streak idiom (already gated, already tuned against exactly
this "don't commit on one noisy sample" problem, ADR-0024-3rd-addendum) —
and keeps M-1..M-4 honestly scoped as an *engagement-timeline* exercise, not
a re-opening of the ground-perception design.
**Risk:** Option B's "reaction latency" is therefore a *link/processing*
latency, not a true acquisition-range latency — the brief should say so
explicitly wherever T_total is reported (see Fork 5, open question #2).

### Fork 4 — Does M-3's "climb-out" need real vertical motion?

- **Option A — model real altitude change:** the interceptor climbs from
  ground level to a cruise altitude before DASH; the target's spawn altitude
  and the whole terminal geometry would need re-deriving.
- **Option B — keep the flat 0.5 m convention:** treat "climb-out" as a
  *speed* ramp inside the existing DASH phase (accelerate to `DASH_SPEED`
  over a modeled spin-up interval) with altitude unchanged.

**RECOMMENDATION (light, genuinely uncertain — see open question #1): B for
a first cut, A flagged as real future work, not dismissed.** Real altitude
change touches **two convention loads simultaneously**: (i) `ALT_REF_M =
0.5` is shared by the interceptor's own altitude P-loop *and* every
target-spawn `z` in `mc_batch.sh` — changing one without the other reopens
the M4 council's careful ALT_REF tuning (ADR-0009: lowered 1.0→0.5 m
specifically so the camera-to-tag vertical offset doesn't consume the <1 m
3D miss budget); (ii) the tag board's rigid world −X face convention
(ADR-0010 #6) was validated for a level, constant-altitude approach — an
oblique climbing approach angle is untested aspect geometry, on top of the
already-flagged oblique-incidence pose-jitter risk (ADR-0003). This is
**not** a one-way door in the strict sense (nothing about the guidance law
changes), but it *is* the kind of "genuinely uncertain, costly-if-wrong"
question CLAUDE.md's council trigger describes, and unlike Forks 1-3 this
brief does not have a confident evidence-backed answer — it's presented as
an open question, not a recommendation dressed up as one.

### Fork 5 — What is the end-to-end metric, and how does it compose with Pk@radius?

- **Option A — T_total blended across all flights** (clean and missed
  alike), a single mean/median "time to intercept."
- **Option B — T_total reported only for clean/latched flights, alongside
  (never blended with) the unconditional clean-rate / handoff-rate.**

**RECOMMENDATION: B.** ADR-0031 already found and named this exact trap:
under a degraded cue, `miss_m` is logged even for flights that never reach
`ENGAGE`, and it is "deceptively SMALL (a ballistic closest-approach)" —
*averaging raw miss flattered the broken arms.* A blended T_total has the
identical failure mode in the opposite direction: a flight that fails fast
(dash-aborts at 20 s, ADR-0031's `DASH: failed to reach handoff range within
20 s` signature) would *shrink* a blended average and make a bad arm look
fast rather than broken. **Composition with Pk (ADR-0025):** T_total is a
**companion timeline metric, not a replacement** for the Pk-vs-radius
headline — report the pair the same way ADR-0036 already reports "M4 pro-nav
win + M5 regime map" as characterizing a system: *clean-rate/handoff-rate
first* (the honest headline under any degraded condition, ADR-0031), *then*
T_total's distribution conditioned on those clean flights, *then*
Pk-vs-radius on the resulting misses. Never report a T_total number without
its accompanying clean-rate in the same breath.

---

## 3. Experiment plan

### 3.1 Reused machinery (nothing new to build for orchestration)

- **`scripts/mc_batch.sh`'s existing `MC_*` env knobs** (`MC_WORLD`,
  `MC_TARGET_MODEL`, `MC_SEEKER`, `MC_VENV_PYTHON`) already generalize the
  batch runner across world/seeker/venv without touching its default path —
  the same pattern extends naturally to a new `MC_LAUNCH_MODE` (or
  equivalent) knob once the orchestrator (Fork 2) exists, defaulting to
  today's unconditional-arm behavior so every existing gate stays untouched.
- **`scripts/audit_per_tick.py`** — the standing per-tick honesty auditor
  (checks (a)-(d): no post-latch cue reads, DASH-before-ENGAGE, command↔LOS
  correlation, residual-leak advisory). Runs unchanged against any new
  arm's CSV; **needs one new check** for this work specifically (§3.4).
- **`scripts/abort_lens.py`** — the post-CPA abort reclassification lens.
  Directly relevant: a slow/gated launch decision squeezing the available
  dash distance is a plausible **new** failure mode with the same *symptom*
  as ADR-0031's degraded-cue dash-aborts (`DASH: failed to reach handoff
  range within 20 s`) but a different *cause* (time budget, not cue
  quality) — `abort_lens.py`'s existing reclassification logic should be run
  on any M-1..M-4 batch before trusting a raw clean-rate number.
- **`scripts/mc_analyze.py`** — paired-seed significance testing (95% CI
  must clear zero before claiming "favours"; Wilson CIs for Pk). Unchanged.

### 3.2 New knobs needed (sketch only, no code in this brief)

- A **reaction-latency tier** flag (`--reaction-latency {best,expected,worst}`
  or similar), sourcing ADR-0016's BEST ~20 / EXPECTED ~90 / WORST ~210 ms
  numbers as a fixed pre-arm delay.
- A **confirm-streak** parameter for the launch gate, mirroring
  `ACQUIRE_MIN_DETECTIONS`/`ACQUIRE_CENTERED_STREAK`'s existing shape
  (Fork 3) — default value TBD at build time, proposed starting point:
  same order of magnitude as the camera's own streak (not pre-registered as
  a number here; this brief is design-only).
- A **timeline-reporting mode**: at minimum, a post-hoc analysis script
  (like `audit_per_tick.py`/`abort_lens.py`'s pattern — analyzer-only, never
  boots a sim) that pulls `t_sim` at each phase's first/last row from an
  `mc_batch` arm's per-flight CSVs and reports the §1.2-style table across a
  batch, conditioned on clean flights (Fork 5).

### 3.3 Gates

1. **Plumbing/regression gate (small n, exit 0/1 script, `check_deploy_phases.sh`
   pattern — analogous to `check_m4.sh`).** Confirms: (a) the new
   orchestrator's default path reproduces byte-identical numeric output vs
   calling `m4_intercept.py` directly today (the CLAUDE.md iron rule); (b)
   the gated launch decision, when exercised, arms only after the confirm
   condition is met (a positive-control check with a delayed cue vs a
   negative-control check with an immediate one); (c) `audit_per_tick.py`'s
   existing (a)-(d) checks still pass unchanged.
2. **Timeline batch (verdict-worthy n).** Per ADR-0041's own power finding
   (n=8 binomial metrics — here, clean-rate/handoff-rate — cannot carry a
   verdict; McNemar needs 6/8 discordant pairs), run **n=16 per cell.**
   Speeds **6/9/12 m/s** (ADR-0036/0029/0028/0030 comparability). Cue tiers
   **EXPECTED and WORST** (CLAUDE.md's worst-credible mandate: "decisions
   must survive WORST"; BEST is not the interesting case here). Paired
   seeds against the current ADR-0036 M5 baseline (today's unconditional,
   zero-latency "launch") to isolate exactly what the gated launch decision
   costs.

### 3.4 Honesty audit extension

`audit_per_tick.py`'s (a)-(d) checks all police the *terminal/camera*
honesty boundary (no post-latch cue reads, command↔LOS correlation). None of
them touch the *launch decision* this work introduces. **New check (e),
proposed:** the arm/takeoff command must be issued only after a
cue-confirmed condition (Fork 3's confirm-streak), never before, and never
derived from any `gt_*` column — the same "assert the decision derives from
sensor reads, not ground truth" pattern `docs/audit_targets.md` already uses
for the camera and cue channels, extended one level earlier in the timeline
to the arm decision itself.

### 3.5 Pre-registered expected outcomes (write these BEFORE flying)

- **Terminal miss / Pk-vs-radius: expect NULL vs the ADR-0036 baseline.**
  Once `DASH` begins, the flight is geometrically identical to today's
  profile — the kinematic ceiling (ADR-0023, `r²(ZEM,miss)=0.990`) is set at
  handoff, independent of how the vehicle got airborne. A gated launch
  decision should not move the terminal numbers *if* the standoff geometry
  (`--y0-mag`/`--x0`) stays generous enough to absorb the added reaction
  time.
- **Clean-rate / handoff-rate: expect a possible cost at the WORST reaction
  tier, mirroring ADR-0031's mechanism but from a different cause.**
  ADR-0031 found degraded *cue quality* causes dash-aborts by starving the
  detection streak before `ENGAGE`. A slow reaction-latency + confirm-streak
  gate is a **time-budget** squeeze on the same finite standoff distance —
  eating seconds off the front of `DASH` before it even starts is
  kinematically similar to shortening the dash itself. **Pre-registered
  hypothesis:** at WORST reaction latency, expect the same qualitative
  failure signature ADR-0031 catalogued (`DASH: failed to reach handoff
  range within 20 s`) to reappear, this time attributable to launch-decision
  latency rather than cue degradation — i.e., the honest limits ADR-0031
  already found (perception availability, not terminal accuracy, is the
  binding constraint) should reappear here under a different name.
- **T_total: expect it to be dominated by whatever standoff/dash-speed
  parameters are chosen, not by the launch-decision latency itself** —
  ADR-0016's 20–210 ms budget is small against the ~21 s already-measured
  liftoff-to-hover cost (§1.2) and the tens-of-seconds `DASH` phase; report
  the decomposition so this either confirms or is falsified by data, not
  asserted.

---

## 4. Cost estimate

Two different measured numbers, not to be conflated (CLAUDE.md: sim time vs
wall time):

- **~65–75 s/flight, wall-clock, sim-boot to sim-boot** (ADR-0041 finding F5
  / `docs/fusion_capstone_design.md`, measured across 5 recent batches) — the
  orchestration cost of one flight in a batch: fresh PX4/Gazebo boot, fly,
  teardown. This is the number that sets **batch wall-clock time.**
- **~21.2–21.7 s, sim-time, already inside every flight's `TAKEOFF` phase**
  (§1.2, six recent logs) — this is *sim* time, part of the ~120-180 s of
  in-sim flight duration each `~65-75 s` wall-clock flight represents
  (`PX4_SIM_SPEED_FACTOR`/RTF-dependent; not separately re-measured here).

**Batch sizing (§3.3):** 3 speeds × 2 cue tiers × n=16 = **96 flights** for
the timeline batch — directly comparable in scale to ADR-0036's own 96-flight
M5 final batch. At ~65–75 s/flight wall-clock: **~1.7–2.0 h** of idle-machine
sim time for the timeline batch alone. Add a small plumbing/regression gate
(n≈8-16, a few minutes) and the total cost is **well under an afternoon**,
one sim at a time, at idle load — consistent with every prior batch this
project has run.

---

## 5. Risks

- **R1 — scope creep into the ground-perception/bird-classifier design**
  (Fork 3). The single biggest risk: "how does the ground rig decide to
  launch" sounds adjacent to "how does the ground rig decide a target is
  hostile" (ADR-0035), which is explicitly gated behind hardware. Keep this
  work to *timing*, not *classification*.
- **R2 — altitude-convention creep** (Fork 4). Touching `ALT_REF_M` or the
  target spawn `z` for real climb-out risks silently invalidating every
  prior gated number (M3/M4/M5/S2) that assumed the flat-altitude geometry.
  If Option A is ever chosen, it must be flagged and re-gated exactly like
  every prior sensor-parameter change (ADR-0024's FOV-narrowing precedent:
  "reopens a validated sensor parameter... must re-earn M1/M2 and disclose").
- **R3 — a launch-decision gate could interact with the existing
  `--coast-search` / dash-abort machinery in an untested way** (Fork 3 +
  §3.5). The confirm-streak logic is proven on the *camera* side; porting
  the idiom to the *cue* side pre-`DASH` is new composition, not a re-use of
  tested code — budget dev iterations for it, the way ADR-0009/M4 needed
  five.
- **R4 — the resume-line framing risk.** "Launch-on-detect" and "climb-out"
  are evocative phrases; if Fork 1/Fork 4's recommendations are adopted
  (no launcher, no real altitude change), the honest writeup must say
  plainly that these are *timing and decision-gating* additions to an
  already-vertical-takeoff quadrotor, not a new physical launch system —
  same disclosure discipline as every other "sim optimism" flag in this
  project (ADR-0023/0024's no-blur-model disclosure is the precedent).

---

## 6. Ratifiable ADR skeleton

> **ADR-00XX — Deployment-profile phases M-1..M-4: gated launch decision +
> timeline reporting, no physical launcher, flat-altitude climb-out approximation.**
>
> **Context.** docs/next.md's Parked deployment-profile item (originally
> commit `1877ca9`, 2026-07-05); design-as-ADR-first per that entry. The
> arm/takeoff sequence already exists unconditionally in every flight
> (`m4_intercept.py:2864–2899`, measured ~21.2–21.7 s sim-time, §1.2); the
> guidance *consequence* of a good launch-on-detect + dash is already
> Gazebo-confirmed (ADR-0028/0028-addendum/0030/0036). What remains is the
> launch-decision gate and the timeline metric.
>
> **Options considered (per fork, §2).** Fork 1: physical launcher vs
> teleport/approximate. Fork 2: extend the phase machine vs a thin
> orchestrator. Fork 3: new ground-detection-range model vs reuse
> ADR-0016 latency + the ACQUIRE confirm-streak idiom. Fork 4: real
> altitude climb-out vs flat-altitude speed-ramp approximation. Fork 5:
> blended vs clean-conditioned T_total.
>
> **Decision.** No physical launcher (Fork 1: B). Thin orchestrator wrapping
> `m4_intercept.py`, byte-identical default path (Fork 2: B). Launch-decision
> gate reuses ADR-0016's latency tiers + a confirm-streak, no new
> ground-detection model (Fork 3: B). Altitude convention: **OPEN — builder
> call, see open question #1** (Fork 4 not pre-decided). T_total reported
> conditioned on clean flights, alongside (never blended with) clean-rate
> (Fork 5: B).
>
> **Why.** The x500 is a vertical-takeoff quadrotor with measured unused
> lateral-accel headroom (ADR-0028-addendum) — a launcher adds risk for no
> physics gain. The byte-identical-default-path rule and the existing
> audit/gate density on `m4_intercept.py`'s phase machine favor composing in
> front of it. The ground-detection-range model is explicitly Stage-0/bird-
> classifier scope (ADR-0035) and must not be duplicated here with
> unsourced numbers. ADR-0031 already demonstrated the blended-average trap
> this design avoids by construction.
>
> **Honesty boundary.** New check (e) on `audit_per_tick.py`: the
> arm/launch decision derives only from cue reads (never `gt_*`) — extends
> `docs/audit_targets.md`'s existing pattern one phase earlier.

---

## 7. Open questions for the builder

1. **Does M-3's "climb-out" need real vertical motion, or is a flat-altitude
   speed-ramp an acceptable, disclosed approximation for a sim whose thesis
   is guidance, not launch-vehicle dynamics?** (Fork 4.) This brief does not
   have a confident recommendation — it's the one fork where the two options
   are genuinely close and the cost (reopening `ALT_REF_M`/target-spawn-z/
   tag-aspect conventions across every prior gate) is real either way.
2. **Is "reaction latency" sourced from ADR-0016's cue-delivery-latency
   tiers an honest enough stand-in for "ground-rig detection latency," given
   the two are conceptually different things** (link/processing latency of
   an existing track vs. time-to-first-detect a threat that just entered the
   sensor's field of regard)? Fork 3's recommendation treats them as
   interchangeable for the purposes of a launch-decision *time budget* — is
   that a defensible simplification for the portfolio writeup, or does it
   need its own disclosed caveat every time T_total is quoted?
3. **Does this item still earn its build slot given the roadmap has closed
   the fusion capstone (ADR-0044) and the honest remaining gaps are
   hardware-gated (Stage-0 bench, bird MC gate #8)?** M-1..M-4 is pure
   software and does not depend on hardware, but it also does not move any
   Pk/miss number (§3.5's pre-registered null) — its value is narrative
   (a timeline resume figure) and completeness, not a new capability. Worth
   confirming this is still wanted before scheduling it.
4. **What confirm-streak length for the launch-decision gate (§3.2)?** This
   brief deliberately did not pre-register a number (design-only) — picking
   one is a small, reversible dev-iteration decision at build time, but
   flagged here so it isn't silently defaulted without a rationale line in
   whatever ADR eventually ratifies the build.

---

### Sources

Repo/ADR: `docs/next.md` (Parked section, Current section); `docs/decisions.md`
ADR-0009 (RTF/sim-time discovery, ALT_REF_M rationale), ADR-0010 (#2 dash/
terminal speed decouple, #6 rigid tag face), ADR-0016 (latency budget),
ADR-0023 (kinematic diagnosis), ADR-0024 3rd addendum (confirm-streak /
early-handoff), ADR-0025 (Pk-vs-radius headline metric), ADR-0028 + addendum
(running start, airframe-agility null), ADR-0030 (dash-track fix, velocity-
emission), ADR-0031 (perception-availability envelope, handoff-rate-not-
raw-miss discipline), ADR-0035 (bird discrimination, VETO-ALL pre-bench
gating), ADR-0036 (M5 final batch), ADR-0041 (fusion capstone power/cost
finding F5); `docs/fusion_capstone_design.md`; `docs/stage0_bench_plan.md`;
`docs/seeker_design_brief.md` / `docs/ekf_design_brief.md` (brief pattern);
`docs/audit_targets.md` (honesty-audit pattern); git commit `1877ca9`
(original M-1..M-4 definitions).

Code: `scripts/m4_intercept.py` (arm/takeoff `:2864–2899`, `ALT_REF_M`
`:247`, phase machine `:1591–2444`, `ACQUIRE_MIN_DETECTIONS`/
`ACQUIRE_CENTERED_STREAK` `:284–291`, `DASH` `:1967–2230`, `FPV` dict
`:326–352`, `S2` dict `:382–412`); `scripts/mc_batch.sh` (`MC_*` knobs `:132–142`,
target-spawn `z: 0.5` `:621`); `scripts/audit_per_tick.py`,
`scripts/abort_lens.py`, `scripts/mc_analyze.py` (reused machinery).

Measurements taken for this brief (no sim boot; read from existing logs):
`TAKEOFF`-phase `t_sim` across `logs/m4_intercept_pronav_20260708T{214037,
214142,214246,214349,214456,214600}Z.csv`.
