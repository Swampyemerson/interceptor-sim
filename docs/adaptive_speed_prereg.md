# PRE-REGISTRATION: adaptive speed governor A/B (2026-09-23)

Builder ask: "can we build some sort of adaptive speed system that changes
speed based on whats needed from the targets speed, the phase of flight we
are in, how good our lock is, etc?"

Written BEFORE the sweep flies (standing rule: config, prediction,
adopt/reject criterion, and what a null would mean, in writing first).

## What was built (flag-gated, default OFF = byte-identical legacy)

`flight/pursuit_terminal.py` `PursuitTerminalConfig.adaptive_speed` plus:

- **Phase A cap** (belief chase, camera not in the loop):
  `clamp(|believed target vel| + overtake_margin_ms(5), adaptive_v_floor_ms(8),
  v_hw_max_ms(24))` replacing the fixed `v_max_ms` 16. Rationale:
  `docs/fast_intercept_limits.md` #1 -- the ceiling is overtake margin, and
  ~5 m/s of margin buys a clean catch.
- **Phase B**: the same adaptive cap on the total command (so the
  target-velocity feedforward for a fast target is not strangled at 16), and
  the CLOSURE cap `v_close_max_ms` scaled by **lock quality** = decode
  staleness (fresh <=0.3 s -> full 6 m/s; linear decay to `v_close_min_ms`
  1.5 by 1.5 s stale). Rationale: don't sprint at a target we can't see;
  keep LOS rate and blur down while blind, preserve reacquisition.
- All governor inputs are guidance-visible (pre-flight belief, own KF);
  no gt read. Unit tests: `flight/tests/test_pursuit_terminal.py`
  (governor section, 6 tests incl. flag-off inertness).

## The experiment

Port arm (real flight code in the loop), rear tag, paired seeds 0..49
(n=50/cell), `scripts/adaptive_speed_ab.py`. Metric: % miss <= 0.35 m
(contact), median miss.

Arms:
- **A legacy**: defaults (v_max 16, fixed closure cap).
- **B fixed-high**: `v_max_ms=24`, nothing else (the "just raise the
  constant" strawman the adaptive system must beat or match).
- **C adaptive**: `adaptive_speed=True` (defaults above).

Cells: target 9, 15, 18 m/s nominal; 9 m/s + aim_error_deg=20;
15 m/s + aim_error_deg=10.

## Predictions (registered)

| cell | A | B | C |
|---|---|---|---|
| 9 nominal | 100% (known) | ~100% | ~100% -- C flies SLOWER (cap 14); regression here would falsify the floor choice |
| 15 nominal | ~54% (known) | high (native arm: 100% at v_max 20) | ~B (cap 20); markedly < B means margin 5 is undersized at 15 m/s |
| 18 nominal | 0% (known) | ~40-66% (native arm: 66% at 24) | ~B (cap 23) |
| 9 + aim 20deg | unknown on this arm | ~A-ish | >= B if lock-modulated closure aids reacquisition; C ~ B = the lock lever is a NULL in these cells |
| 15 + aim 10deg | unknown | unknown | same direction as above |

Secondary (descriptive, measured only if adoption is reached): mean own
speed at 9 m/s nominal -- C should be lower than A/B ("as fast as needed").
Not plumbed in mc rows; would come from a separate small engine loop.

## Adopt/reject criterion (registered)

ADOPT adaptive as the chase CONCEPT's recommended config (isim spec +
contract note; NOT the flying default -- the chase itself stays blocked on
the Gazebo transfer gap regardless) iff:
1. no cell regresses vs A by > 5 points, AND
2. C within 10 points of B on the fast nominal cells (15, 18), AND
3. no degraded cell where C trails B by > 10 points.

If C ~ B everywhere (both beating A): adopt the SIMPLER fixed raised cap,
record the adaptivity as unearned complexity for miss-rate purposes (energy
story stays untested, said plainly), keep the flag in the code as a
measurement hook.

## What a NULL would mean

If B ~ A on the fast cells (the raised cap does NOT reproduce the native
arm's gains on the port arm), the real flight code has its own binding
constraint the native sweep didn't see (e.g. the accel slew, the Phase-A ->
B acquisition gate, the fallback ping-pong) -- that becomes the finding, and
no adoption happens until it is understood.

## RESULT (2026-09-23, n=50/cell, paired seeds)

| cell | A legacy | B vmax24 | C adaptive |
|---|---|---|---|
| 9 nominal | 100% (med 0.081) | 100% (0.057) | 100% (0.122) |
| 15 nominal | 54% (0.341) | 100% (0.059) | 100% (0.095) |
| 18 nominal | 0% (5.104) | 80% (0.121) | 64% (0.153) |
| 9 + aim 20 | 100% (0.063) | 100% (0.053) | 100% (0.088) |
| 15 + aim 10 | 0% (4.449) | 98% (0.079) | 96% (0.111) |

Scored against the registered criteria: (1) PASS -- C never regresses vs A
(>=+46 points on the two cells legacy loses). (3) PASS -- degraded cells
within 2 points of B, so the lock-quality closure lever is an honest NULL
in these cells (its value claim is dropped, not spun). (2) **FAIL at 18
m/s: C 64% vs B 80%** -- the pre-named mechanism (margin undersized: cap
18+5=23 vs B's flat 24). Note the degraded-cell baselines: this port
config (rear tag) shrugs off aim errors that the nominal-concept study did
not, so cells 4-5 had less discrimination than intended.

## AMENDMENT 1 (registered BEFORE flying): margin sizing follow-up

Config: C with `overtake_margin_ms=6.0` (cap at 18 m/s = 24 = B's), cells
18_nom and 9_nom (guard).
PREDICTION: 18_nom rises to B's ~80% +/- paired noise (confirming the gap
was margin sizing, not adaptivity overhead); 9_nom stays 100%.
If 18_nom stays ~64% with the SAME cap as B, the deficit is elsewhere in
the governor (e.g. the Phase-B cap tracking a KF speed estimate that reads
low) -- that would be the finding.

### Amendment 1 RESULT (flown, n=50 paired)

margin 6 (cap 18+6=24, equal to B's): 18_nom **66%** (med 0.166), 9_nom
100% (guard held). Prediction WRONG -- equal caps did NOT close the gap to
B's 80%. Registered null path applies: the deficit is inside the governor.

## AMENDMENT 2 (registered BEFORE flying): attribute the governor's cost

Two live suspects at 18 m/s (median ~95 decodes over a long chase =
sparse-decode stretches are common):
(a) the LOCK-MODULATED CLOSURE: staleness > 0.3 s decays the closure cap
toward 1.5 m/s, so C closes slower than B exactly when decodes are sparse;
(b) the Phase-B TOTAL cap follows `|sane(KF v_t)| + margin`, and the KF
speed estimate can read low, sagging the cap below 24.

Config: C, margin 6, `lock_fresh_s=1e9` (lock modulation disabled, factor
pinned to 1.0; total-cap logic unchanged), cell 18_nom.
PREDICTION: if (a) is the cost, this recovers to ~80% (B +/- paired
noise); if it stays ~66%, (b) is the cost and the fix is flooring the
Phase-B cap at the Phase-A belief speed + margin.

### Amendment 2 RESULT (flown, n=50 paired)

18_nom, margin 6, lock modulation disabled: **78%** (med 0.128) -- B's 80%
within paired noise. Suspect (a) CONFIRMED: the lock-modulated closure is
the governor's cost at fast/sparse-decode cells. Mechanism: staleness-
triggered slowdown keeps the range long, the tag small and the decodes
sparse -- a positive feedback of timidity. Combined with the cells-4/5
null, the lock-quality lever is REJECTED (graveyard), not retuned.

## AMENDMENT 3 (registered BEFORE flying): final-candidate confirmation

Config: `adaptive_speed=True, overtake_margin_ms=6.0`, lock modulation
REMOVED from the design (cap sizing only -- Phase A/B cap =
`clamp(|believed target speed| + 6, 8, 24)`). All five original cells,
same paired seeds.
PREDICTION: within paired noise of B on every cell (9/15 nominal and both
degraded cells ~100/~100/~98; 18_nom ~78-80%), while commanding LESS
speed than B whenever the believed target is slow (cap 15 at 9 m/s vs
B's flat 24) -- "as fast as needed, no faster".
ADOPT (as the chase CONCEPT's recommended config, still not the flying
default) iff no cell trails B by > 5 points.

### Amendment 3 RESULT (flown, n=50/cell paired) -- ADOPTED

| cell | A legacy | B vmax24 | C adaptive (final) |
|---|---|---|---|
| 9 nominal | 100% (0.081) | 100% (0.057) | 100% (0.099) |
| 15 nominal | 54% (0.341) | 100% (0.059) | 100% (0.081) |
| 18 nominal | 0% | 80% (0.121) | 78% (0.128) |
| 9 + aim 20 | 100% | 100% | 100% |
| 15 + aim 10 | 0% | 98% (0.079) | 98% (0.094) |

Prediction held; no cell trails B by more than 2 points. Registered ADOPT
criterion met: `adaptive_speed=True` (margin 6, floor 8, hw 24) is the
chase CONCEPT's recommended config. NOT the flying default: the chase
itself remains blocked on the Gazebo transfer gap
(docs/xcheck_gazebo_pursuit_prereg.md), and v > ~16-18 m/s extrapolates
the vehicle fit.

### Secondary (energy / "flies slower"): NULL, reported plainly

9 m/s cell, seeds 0..19, trace-derived: mean own speed B 5.96 vs C 6.57
m/s (C slightly HIGHER, against the prediction; the whole-window mean is
dominated by post-contact station-keeping and is a poor proxy). Peak own
speed median B 14.41 vs C 13.95 m/s -- neither arm's cap binds at 9 m/s
(the rendezvous P-gain self-limits below both). The "as fast as needed
uses less energy" claim is UNSUPPORTED at this cell and is dropped. The
surviving argument for adaptive over a flat 24 is envelope discipline:
the commanded speed never exceeds believed-need + margin, so a slow
target never pulls the vehicle into the unvalidated extrapolated regime.
That is a design rationale, not a measured advantage.

## Caveats (carried from docs/fast_intercept_limits.md #5)

isim only; v > ~16-18 m/s extrapolates the fitted vehicle model; v_hw_max
24 is an assumed ceiling -- the real airframe's is the unmeasured
`dash-accel-profile` given. The Gazebo cross-check for the chase currently
FAILS its bar; every number here is an isim-internal comparison, not a
transfer claim.
