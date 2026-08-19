# Results notes — the full scope/caveat text behind the README table

The README's results table keeps every load-bearing scope inline but shortened;
this file holds the complete original caveat prose per row, verbatim, so no
disclosure is ever more than one click from its number. The binding rules for
public wording are in `docs/publish_runbook.md` (claims scrub).

## M0–M2 foundations {#m0m2}

Wide (99.7°) lens; sim lighting. Detection rate and pose error measured against
sim ground truth by the M2 gate (`scripts/check_m2.sh`), 2026-07-04.

## M4 pro-nav vs pursuit {#m4}

Official gate-config runs. **Disclosure (ADR-0009 addendum):** three earlier
dev-phase pro-nav flights that same night read 1.04–1.12 m — over the gate —
before the final configuration; the gate numbers come from the official
gate-config runs and the dev-phase selection is disclosed rather than hidden.

## Two-stage handoff (S2) {#s2}

Proves the *architecture* (running start + structural handoff), not sub-meter
precision.

## 41-flight forensics (why fast crossers miss ~1.4 m) {#zem}

Miss tracks zero-effort-miss at handoff with r² = 0.96 — *variance explained*,
never "96% of any one miss" (the runbook's DEEP-H5 rule). The forensics set is
the 6 m/s cue-era campaign (n=41, ADR-0023); the r² sharpens toward 0.99 at
higher speeds (ADR-0069), where correction capacity shrinks against delivered
ZEM.

## M5 final Monte-Carlo, n=96 {#m5}

Flew the **clean AprilTag sensor** — the disclosed perception upper bound — on
a flat-board target; the guidance laws tied within the ~1 m run-to-run
terminal-dropout noise at these speeds. 96.9% clean means ran to completion and
engaged; failures stay in every Pk denominator.

## Markerless seeker v2 {#v2}

In-sim markerless; the guidance-side recovery levers were pre-registered and
came back NULL (ADR-0043).

## Detect-then-track maneuvering terminal {#t21}

One empty Gazebo world — the gate radii never saw clutter (disclosed). The
paired n=16 baseline reads 3/15 → 14/15 for jink. Zero gross false terminal
detections means **>8 m** gross errors, in the headline arm, per the FWD-A4 /
DEEP-P7 scope. **The CSRT tracker was later dropped for the 3D quad target** —
every tracker tested slips on a banking quad — so the deployed config is
NN-only on every frame (ADR-0076 add #2); this row is a billboard-era result.

## Pk statistics hardening, n=72 {#pk72}

**Flat-billboard target**, weave path + 12 m/s only, never pooled across paths
or speeds, radius always stated (ADR-0064/0025). The realistic 3D-quad target
later exposed the flight-dynamic perception wall that this target shape masked.

## Perception wall {#wall}

The wall is flight-dynamic (pointing + background + phantom competition), not
range/resolution/aspect — each of those was tested and eliminated (ADR-0076
add #18i/#18k).

## Open-loop dash robustness {#dash}

Dash-robustness only — *not* perception proof: the ENGAGE streaks can be
phantom (ADR-0076 add #18g). The ±30° tolerance says the open-loop dash keeps
engaging under aim error; it says nothing about the camera.
