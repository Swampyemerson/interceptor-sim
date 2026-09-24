# Rehearsal break-off — design + registration (2026-09-24, builder directive)

Builder (2026-09-24, verbatim intent): *"an option to toggle where instead at the very
last possible chance (plus a little margin) when a collision is as certain as possible
the drone breaks off to preserve hardware and get several runs in before we do a real
test."* Practice passes without contact: many scored runs per battery/airframe, then a
real contact test only when chosen. Registered BEFORE the build.

## Design (config-gated, default OFF, byte-identity pinned)

`PursuitTerminalConfig`:
- `rehearsal_breakoff: bool = False`
- `rehearsal_range_m: float = 2.5` — trigger range (sim-derived below; TODO-BUILDER
  until the sweep sets it)
- `rehearsal_min_updates: int = 3` and `rehearsal_fresh_s: float = 0.5` — the
  "as certain as possible" gate: at least N consumed updates within the last X s
  (a converged, currently-fed track; a coasting or phantom track must NOT trigger)
- `rehearsal_evade_s: float = 1.5` — evade duration before ending via the normal
  miss/SAFE path with its own reason `rehearsal_breakoff` (distinct in logs/scoring)

Trigger: Phase B, gate satisfied, closing > 0, `r_hat ≤ rehearsal_range_m`.
Evade command: full-authority CLIMB plus lateral push AWAY from the target's predicted
path side (KF velocity cross own course decides the side; fall back to right), yaw
held; existing accel/slew budgets apply. After `rehearsal_evade_s`, end the engagement
through the existing SAFE machinery (reason `rehearsal_breakoff`) → hover per the
ADR-0107 miss ruling, ready for the next pass.

Honesty rule for the claim "it would have hit": scored two ways, both reported —
(a) sim/gt (scoring-only) WOULD-HAVE CPA = the CPA of the PAIRED same-seed flight with
rehearsal OFF; (b) the onboard estimate = extrapolated ZEM from the KF state at
trigger. (a) is the sim truth; (b) is what the field will have (plus both ULogs). The
mode is only trustworthy if (b) tracks (a).

## Measurement plan (isim, port arm, ENGAGE fit + pose, tag-realism rung R2,
n = 50 paired seeds/cell; cells: nominal · aim20 · alt+3 pair)

1. **Margin sweep:** `rehearsal_range_m` ∈ {1.5, 2.0, 2.5, 3.0} — per arm: trigger
   rate, minimum TRUE separation after trigger (worst case and p05), and the
   would-have-estimate error (b)−(a).
2. **Identity:** flag OFF byte-identical (pin test); flag ON but never-triggering
   flights identical to their OFF twins up to the trigger tick (test).

## Registered predictions & adopt bar

- **P1:** some trigger range ≤ 3.0 m gives worst-case post-trigger true separation
  ≥ 0.7 m on ALL cells while triggering on ≥ 80% of flights that would have passed
  inside 0.35 m (the practice-value condition).
- **P2:** the onboard would-have estimate (b) tracks the paired truth (a) with median
  |error| ≤ 0.15 m on triggered flights.
- **Adopt** the smallest range meeting P1 as the default `rehearsal_range_m`; if P2
  fails, the mode still ships (the sim/gt pairing still scores sim practice) but the
  FIELD would-have claim is downgraded to "ULogs + video only" and says so.
- **Null branch:** if no range ≤ 3.0 m clears P1, report the achievable
  (range, separation, trigger-rate) frontier and put the trade to the builder.

## Safety note (field-facing, recorded now)

The evade is an ADDITIONAL maneuver near the target: the field procedure must treat
`rehearsal_range_m` + closing speed + both airframes' envelopes as the separation
budget, and the first field uses of the mode belong at REDUCED closing speeds. The
sim-derived margin is an isim number until a Gazebo spot-check confirms it — queued
as the mode's validation rung, not flown in this round.

## RESULT (built + swept 2026-09-24; suite 531 green + honesty audit PASS,
head-verified; sweep verbatim in logs/rehearsal_20260924/sweep.txt — key rows below)

The mode is BUILT as specified (default OFF, byte-identity pinned, own SAFE reason
`rehearsal_breakoff` hovering per ADR-0107, 9 mutation-checked terminal tests + CLI
test, `--pursuit-rehearsal` on the real CLI) — **and the registered sweep lands on the
NULL BRANCH: no trigger range ≤ 3.0 m clears P1.** Worst-case post-trigger TRUE
separation is 0.034–0.252 m against the 0.7 m bar (recall ≥ 91% everywhere — it
triggers fine; it just cannot get away), and 8–52% of triggered passes still came
inside 0.35 m. P2 also fails (the onboard would-have estimate reads high, +0.06 to
+0.43 m, worst at alt+3) — per the registered consequence the FIELD "would-have" claim
is ULogs + video only. aim20 additionally shows a late-gate hole: 2 untriggered real
contacts per arm (the freshness gate can open as late as r_hat 0.5 m).

**Traced mechanisms (why, not just that):** (1) arrival is too hot for the honest
plant — median true closing at trigger 5.3–5.5 m/s; the slew-limited evade barely
bends the trajectory before the pass (seed-33 trace: climb command −3.4 m/s, achieved
−0.3 m/s by the pass). (2) The certainty gate is range-blind to reaction time.

**Exploratory (UNREGISTERED, direction only, logs/rehearsal_20260924/
exploratory_unregistered.txt):** with the brake package on, closing at trigger drops
to ~3 m/s and a 4.0 m trigger reaches worst-case 0.45 m (nominal, 0 contacts) — still
short of 0.7, and the aim20 late-gate hole persists at every range.

**Per the registered null branch, the frontier goes to the builder** with the
candidate design changes (none built): (a) trigger on TIME-TO-GO vs the evade's
reaction time (plant lag + authority), not on range; (b) a "too late — do not
trigger, log it" floor closing the aim20 hole; (c) pair rehearsal with the brake
package + reduced practice closing speeds (the safety note anticipated this — and a
practice pass has no need for full intercept speed). Any adopted margin remains an
isim number until the mode's Gazebo spot-check rung.
