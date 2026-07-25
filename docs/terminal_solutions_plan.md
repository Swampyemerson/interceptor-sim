# How to raise the hit rate — the consolidated plan

*Synthesis of the 5-lane terminal-solutions research (ADR-0020 through ADR-0024),
2026-07-06. This is the "what actually pans out" answer: where the miss really comes
from, and the cheapest ordered set of changes that moves it — each one testable in
the sim before a dollar is spent.*

## The one finding everything hangs on

We instrumented every fast-target flight and found the miss is **not** a camera
problem. It is decided *before* the camera-only endgame begins:

- The final miss tracks the geometry the interceptor is already on at handoff almost
  perfectly (r² = 0.957 at the handoff latch, 0.990 at the freeze latch — n=41).
  **~96% of the miss variance is locked in before the terminal phase starts**
  (ADR-0023, re-verified independently). *(Audit correction 2026-07-06: an earlier
  draft cited the freeze-latch r²=0.99 as the handoff number.)*
- The camera keeps seeing the tag down to ~1.7 m, ~0.07 s before closest approach.
  The "lost detection at CPA" is the drone flying *past* the target, not losing it
  early.
- Worst flights held the camera locked to 3.5 m range and **still missed by 3.4 m.**
- Physics bound: in the last ~0.4 s the drone can only bend its path ~0.7 m, but it
  arrives with ~1.7 m of error. The window is too short. A *perfect* terminal camera
  cuts the miss only 25%.

**So the lever is time-to-go and the geometry delivered at handoff — not the seeker.**
This corrects our earlier "perception-gated" reading (ADR-0014) and reorders every
solution we'd been considering.

## What real interceptors do (and what it means for us)

Cheap fielded kinetic counter-UAS — the FPV-on-FPV interceptors in Ukraine, Coyote,
APKWS, Fortem's net drone — **never** try for a zero-miss hit. They kill within a
few-meter lethal radius (ram, net, or small proximity charge) (ADR-0021/0022). Two
consequences:

1. **At 2-3 m/s we already win.** Our pro-nav miss (~0.37 m) is at the edge of a bare
   kinetic ram's 0.35 m radius (ADR-0084) → that intercept is a kill/near-kill. Part of the
   "problem" was grading against a hit-to-kill bar only a warhead-less ram requires.
2. **You cannot buy out the perception gap with a bigger warhead.** The one mechanism
   that would forgive a 2 m miss at speed (a proximity charge) has to *detect* the
   target at that 2 m to trigger — the same detection we lose. A bigger radius just
   moves the gap to the fuze.

## The plan — cheapest first, each sim-testable

### Tier 1 — FREE software, no honesty cost, do first
These attack the ~20-25% of the miss that is *recoverable mechanization loss*, plus
the time-to-go lever, with zero hardware and zero re-baselining.

1. **Hand off / start terminal correction EARLIER.** We first detect the tag at
   ~7.6 m but don't start correcting until the latch at ~6.2 m. Starting at first solid
   detection buys free t_go (capacity scales with t_go²). Directly attacks the ZEM
   window.
2. **Reclaim the freeze/filter loss (ADR-0014 split-freeze).** The guidance freezes
   its course ~0.2 s before CPA and the α-β filters settle slowly — together they
   realize only 0.28 m of an available 0.72 m of correction. Later freeze + warm-settled
   filters recover an estimated ~0.3-0.45 m. This is the single most concrete free win.
3. **Keep emitting cue velocity (ADR-0015, already validated).** It lowers the ZEM
   delivered at handoff — the exact quantity that determines the miss.

**Test:** lab A/B in `guidance_lab.py` (earlier-handoff + split-freeze variants), then a
paired Gazebo `mc_batch` on an idle machine, primary metric ZEM@handoff and miss, with
a near-R=0 regression so we don't break the M4 gate. "Lab ranks, Gazebo decides."

### Tier 2 — ~$15-35 hardware, but reopens a validated parameter
4. **Narrower/longer acquisition lens (ADR-0024).** A longer lens sees the tag at
   longer range → earlier lock → bigger t_go. Acquiring at 12 m instead of 6.5 m raises
   correction capacity 0.72 → ~4.3 m, above even our worst delivered geometry. This
   *reverses* the earlier wide-FOV plan (terminal FOV-hold is worthless). **Cost caveat:**
   narrowing the sim FOV reopens the M1/M2 detection envelope (the anti-tag-inflation
   honesty boundary, ADR-0010) — it must be re-baselined and disclosed, not slipped in.

**Test:** narrow the SDF `<horizontal_fov>`, raise `HANDOFF_RANGE`, re-run M1/M2 gates +
`mc_batch`.

### Tier 3 — the metric decision (RATIFIED by the builder, 2026-07-06, ADR-0025)
5. **Proximity/lethal-radius Pk is now the headline metric.** Report the full Pk-vs-radius
   curve (the ADR-0014 plan) and headline the radius a chosen cheap kill mechanism actually
   delivers — ram 0.35 m (ADR-0084, 5-inch pair; the slow regime's ~0.37 m pro-nav miss is
   now at the edge of this bar, not comfortably inside it — re-check against 0.35 m, don't
   assume the old 0.5 m margin), net ~1.5 m (lifts the fast regime, salvo-stacked). Radius set by the mechanism's physics,
   never reverse-engineered to a threshold. M0-M4 gates stand; this reframes M5's headline.

## What we are explicitly NOT doing (and why)
The data says these barely touch the miss — rejected with reasons in the ADRs:
yaw-rate authority (already 124°/s, not the cap), wider FOV / 2nd terminal camera,
higher frame rate, motion-deblur, event/DVS camera, nested tags, gimbal, IR seeker,
terminal range sensor, APN, and the look-angle guidance law (its FOV-escape premise is
~2% of the miss). Mid-course fusion stays default-OFF (small handoff-geometry gain only).

## The jam-resistance envelope (ADR-0020, for completeness)
The comms-denied headline is bounded by **target maneuver**, not link-cutoff range: a
straight-line target can be coasted through a jam for tens of meters (velocity-emitting
cue enables dead-reckoning), but a jinking target collapses the margin to ~2 m — so the
onboard seeker must acquire essentially *at* the jam range unless straight flight is
assumed.

## Sources
ADR-0020 (jam envelope), ADR-0021 (kill mechanism), ADR-0022 (real-world guidance,
PROPOSED), ADR-0023 (root-cause diagnosis — the linchpin), ADR-0024 (seeker upgrades),
and the docs each cites: `terminal_diagnosis.md`, `kill_mechanism.md`,
`terminal_guidance_realworld.md`, `seeker_upgrades.md`.
