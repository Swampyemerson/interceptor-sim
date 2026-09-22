# Flight-code pursuit port vs native prototype -- numeric parity (2026-09-21)

The `flight/pursuit_terminal.py` port (ADR-0103 "chase only", run through the
UNMODIFIED `RealFlightSM` via `isim.flight_adapter.RealFlightGuidance`) is now
wired through the full scenario harness: `Scenario(concept="flyby",
terminal="pursuit")` builds the belief seed (`belief_r0_ned` /
`belief_vel0_ned`) from the SAME pre-flight belief geometry the native
concept uses, sets `MissionConfig.pursuit_mode=True`, and runs under the same
25-s engagement window. This file is the first side-by-side measurement of
the port against the native prototype (`isim.concepts.
PursuitRendezvousGuidance`), whose numbers are the reference.

**Headline: the port is much worse than the prototype, and the gap is real,
attributable, and dominated by the STATE-MACHINE WRAPPER, not the guidance
law.** That is the finding this run was for -- the port carries two disclosed
simplifications (a fixed-latency measurement model and the `RealFlightSM`
failsafe wrapper), and their cost now has a number.

## Result table

Fraction of runs inside 0.35 m (the binary-kill ram radius), n=50 seeds per
cell (seeds 0..49, identical in both arms), median miss alongside:

| cell                      | native `--concept pursuit` |          | port `--concept flyby --terminal pursuit` |          |
|---------------------------|---------------------------:|---------:|------------------------------------------:|---------:|
|                           |                  %<=0.35 m | med miss |                                 %<=0.35 m | med miss |
| nominal (aim 0)           |                     100.0% |  0.120 m |                                     24.0% |  0.520 m |
| aim_error_deg = 10        |                     100.0% |  0.129 m |                                     16.0% |  0.554 m |
| aim_error_deg = 20        |                     100.0% |  0.098 m |                                      0.0% |  3.804 m |
| target_alt_offset_m = +2  |                     100.0% |  0.149 m |                                     10.0% |  0.849 m |

How each PORT engagement actually ended (the native prototype has no
terminating state machine -- it chases for the whole 25-s window):

| cell     | target_lost(2.0s) BREAKOFF | hard_floor(<=0.5 m) BREAKOFF (contact declared) |
|----------|---------------------------:|------------------------------------------------:|
| nominal  |                      42/50 |                                             8/50 |
| aim 10   |                      47/50 |                                             3/50 |
| aim 20   |                      50/50 |                                             0/50 |
| alt +2   |                      47/50 |                                             3/50 |

## Attribution of the gap (mechanism, from the transition logs)

1. **`engage_lost_target_s = 2.0 s` is the dominant killer, every cell.**
   The native prototype treats a decode dropout as a mode change: after its
   own `fallback_s = 3.0 s` it re-seeds Phase A from the KF state and keeps
   chasing -- its transition logs show repeated ENGAGE <-> APPROACH bounces
   across the 25-s window, and it re-acquires (median 86-112 decodes/run).
   The `RealFlightSM` wrapper instead reads ANY 2-s post-first-detection
   dropout as FAILSAFE 5 (`target_lost`) and BREAKOFFs permanently -- the
   port's own 3.0-s fallback can never fire because the SM aborts a second
   earlier, and there is no re-approach path out of BREAKOFF. Close to the
   target the tag routinely stops decoding for BOTH arms (FoV exit / too
   close); only the port dies of it (median 47-60 decodes/run). This is an
   arm-asymmetric failure mode in the SYSTEM UNDER TEST (a real property of
   the flight code, not of the instrument), so the DIRECTION is trustworthy
   and the specific margins are properties of this failsafe setting.
2. **`breakoff_hard_floor_m = 0.5` caps the port's measured CPA.** When a
   detection reads <=0.5 m the SM declares contact and flies the breakoff
   climb; the trajectory's scored CPA lands ~0.38-0.5 m. The native concept
   flies through `hold_range_m = 1.0` on a coast and is scored at true
   closest approach (median ~0.12 m). So `%<=0.35 m` structurally
   under-credits port runs that DID reach declared contact -- the 8/50
   nominal `hard_floor` runs are, in the real ram's terms, kills. Even
   crediting those as hits, the nominal port cell is ~24-40%, still far
   under the native 100%.
3. **aim 20 is a qualitative failure, not degradation.** The port's median
   miss (3.804 m, identical across most seeds, t_cpa ~3.6 s) is the CPA of
   the DETERMINISTIC Phase-A leg -- with only ~5 decodes/run, the first
   short decode burst starts the 2-s lost-target clock, the SM aborts at
   ~10 s, and the KF-driven close never happens. The native prototype at the
   same cell re-acquires its way in and still hits 100%.
4. **The fixed-latency simplification (`meas_latency_s = 0.045` constant
   extrapolation vs the prototype's own-state ring buffer at `t_capture`) is
   NOT separable from this data** -- its cost is masked by (1) and (2). A
   clean measurement of it needs the wrapper failsafes matched first.

## Method

- Harness: `isim.mc sweep`, both arms with `--tag-facing rear` (the chase
  concept's own geometry), default `cam_fx_px = 385`, default crossing
  scenario (9 m/s target, 6.5 m abeam, 16.2 m lead), seeds 0..49 per cell,
  workers = default. Commands:
  - native: `.venv/bin/python -m isim.mc sweep --axis aim_error_deg --values 0,10,20 --n 50 --concept pursuit --tag-facing rear`
  - port: same with `--concept flyby --terminal pursuit`
  - height cell: `--axis target_alt_offset_m --values 2 --n 50`, both arms.
- **Scatter is OFF in both arms, deliberately.** `scenario.build()` wraps
  ONLY `concept="pursuit"`/`"hybrid"` guidance in `OwnStateNoise`; the flyby
  path never gets it, so a `--scatter` comparison would hand the port a
  noise-free own-state the native arm doesn't have (arm-asymmetric
  instrument). With scatter off, both arms see identical world/decode-noise
  draws per seed. Consequence: the native 100% cells here are a CLEANER
  condition than ADR-0103's validated 82%-nominal (which was all-errors-on);
  this table is internally consistent but not comparable to that number.
- Terminator counts come from re-running the port cells in-process and
  reading `RealFlightGuidance.events` (the CSV rows don't carry BREAKOFF
  reasons).
- Bounds changed for this path ONLY (`terminal="pursuit"` under
  `concept="flyby"`; everything else byte-identical, pinned by the existing
  `test_scenario.py` identity tests): `MissionConfig.engage_max_s` 12 ->
  `pursuit_window_s` (25 s; the chase takes 6-13 s to contact and ENGAGE now
  starts at the GO edge, so the fly-by-tuned 12 s would truncate it), and
  the flyby+pursuit `EngagementConfig` uses the native concept's window
  (`max_t = stop_after_cpa_s = pursuit_window_s`) instead of the fly-by
  default (`max_t=30, stop_after_cpa_s=1.0`) so both arms are scored on
  closest approach over the same whole-window chase.

## What this does NOT say

- It does not say the port's guidance math is wrong -- nominal runs close to
  ~0.4-0.5 m before the wrapper ends them, and 8/50 reach declared contact.
- It does not price the fixed-latency simplification (masked; see above).
- It is isim, not Gazebo, and not a bench measurement: numbers rank and
  attribute, they don't certify (lab-ranks-Gazebo-decides).

## Obvious next lever (decision, not made here)

The port's cost is concentrated in two `MissionConfig` constants tuned for
the fast fly-by (`engage_lost_target_s=2.0` vs the concept's own
`fallback_s=3.0` + re-approach, and the `hard_floor` climb-away). Whether
`pursuit_mode` should also relax those is a flight-code design decision
(ADR territory), not a sim-side patch -- deliberately not changed in this
round, whose brief was to measure the port as it stands.

## PRE-REGISTERED FOLLOW-UP A/B (written 2026-09-21, BEFORE the runs): the lost-target window

**Config.** Same four cells, same seeds 0..49, port arm only. Control =
`engage_lost_target_s = 2.0` (as measured above). Arm =
`engage_lost_target_s = 4.0` (pursuit's own `fallback_s = 3.0` + 1 s margin,
so the guidance's Phase-B->A recovery gets to fire before the state machine
aborts), applied by mutating the built guidance's MissionConfig, pursuit path
only — no flight-code default changes. Two scores per cell: raw CPA <= 0.35 m,
and "ram-credited" (a `hard_floor` BREAKOFF counted as contact regardless of
scored CPA, per attribution point 2).

**Prediction.** The nominal / aim10 / alt+2 cells improve substantially
(attribution point 1 says the 2 s clock is the dominant killer: 42-47 of 50
runs per cell die of it while the native re-acquires). aim20 improves less or
not at all — its ~5 decodes happen early on the deterministic Phase-A leg,
and a 4 s clock may still expire before the KF close.

**Adopt/recommend criterion.** Recommend `pursuit_mode` relax the window (a
flight-code change, builder-ruled) iff the ram-credited nominal cell at least
doubles AND reaches >= 50%. Below that, the window is NOT the dominant lever
and the recommendation instead becomes "attribute the remainder first".

**What a NULL means.** If lengthening the window does not move the cells, the
dominant cost is elsewhere (the hard-floor scoring cap, or the KF close
itself under the fixed-latency simplification) and attribution point 1 is
WRONG as stated — the spec must then be corrected, not just extended.

## A/B RESULT (same day): NULL — and the null clause fires

Ran exactly as registered (n=50/cell, seeds identical, window 2.0 vs 4.0 s):
**every cell byte-identical between arms** — raw and ram-credited fractions,
median misses, and even the terminator counts (nominal 12/50 raw & 14/50
ram-credited & 42 target_lost & 8 hard_floor in BOTH arms; aim10 8/9/47/3;
aim20 0/0/50/0; alt+2 5/5/47/3).

Instrument checks before believing it (silent-failure rule):
- The override provably reaches the state machine (`sm.cfg` reads 4.0/10.0/
  25.0), and the abort TIME tracks the window exactly (seed 2: target_lost at
  10.34 / 12.34 / 18.34 s for 2/4/10 s windows). The manipulation is live.
- The miss does not move at ANY window — including a probe with the failsafe
  effectively OFF (30 s > max_t, never fires): seed 2 stays 0.436 m and
  seed 4 stays 0.671 m while the engagement runs the full 25 s window.

**Correction to attribution point 1, per the registered null clause.** The
2.0 s lost-target clock correctly describes when port runs END, but ending
early costs NOTHING: the scored CPA is already made on the first pass before
the terminal dropout, the dropout is PERMANENT at close range, and the port
never re-acquires or tightens no matter how long it flies — pursuit's own
Phase-B→A fallback does not produce a second converging pass through the
RealFlightSM wrapper the way the native prototype's ENGAGE↔APPROACH bouncing
does. So: do NOT spend a flight-code ruling on relaxing
`engage_lost_target_s` — it is exonerated as the miss driver in this
condition. (Attribution point 2, the hard-floor CPA cap, stands — the
ram-credited column already prices it.)

**Where the gap actually lives: the FIRST-PASS CLOSE.** Native median
0.120 m vs port first-pass 0.4–0.9 m. Checked and ruled out as the crude
version: the harness DOES model pipeline latency (0.045 s ± 5 ms jitter,
`isim/seeker.py`) and the port's fixed `meas_latency_s = 0.045` matches it
in the mean. The remaining candidates need a per-tick estimator trace, the
diagnostic isim was built for: (a) the native interpolates OWN ATTITUDE at
each detection's t_capture while the port converts the box with the
CURRENT-tick attitude (a bearing error whenever attitude moved during the
latency); (b) latency JITTER around the fixed constant; (c) any residual
wrapper difference in the command path. NEXT: trace one seed side-by-side.
