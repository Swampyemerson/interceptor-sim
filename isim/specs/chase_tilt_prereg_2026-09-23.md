# Chase-regime camera up-tilt A/B — pre-registration (2026-09-23)

Registered BEFORE any run was launched (standing rule). Sibling to
`keepframe_prereg_2026-09-23.md` — third attack on the same +3 m-above
altitude-error cliff, after the keepframe assist NULL'd and the Phase-A
vertical sweep was stopped at the history gate (measured negative,
`isim/concepts.py` `vsearch_*` block).

## Why THIS lever

The +3 m acquisition attribution (keepframe prereg doc, follow-up section)
put 41% of blind approach frames in "out of frame — TOP" — a vertical-FoV
CEILING problem. A fixed camera up-tilt raises that ceiling with ZERO
trajectory perturbation, so the recorded sweep failure mechanism
("perturbing a Phase-A trajectory that was about to succeed anyway") cannot
apply. It is un-graveyarded for the chase regime: the historical mount-angle
spec ("size to the measured DASH pitch") is a sprint-era rationale, and the
chase flies near-level — the chase-regime bracket angle is genuinely open.
This A/B is the evidence the printed-bracket angle needs. CONFIG-ONLY: no
flight-code changes; `Scenario.cam_tilt_up_deg` feeds BOTH the true seeker
camera (`CameraParams.mount_tilt_up_deg`, plus scatter's true-only tilt
error on top) and the flight code's own `GuidanceConfig.mount_up_rad` via
`isim.flight_adapter` — the SAME capture-instant attitude/mount path every
measurement uses, never a scoring-only tilt.

## Config (exact)

- Script: `scripts/chase_tilt_ab.py` (cell mechanics as the keepframe A/B).
- Base: `Scenario(concept="flyby", terminal="pursuit", tag_facing="rear",
  scatter=Scatter(), aim_error_deg=0.0, cam_tilt_up_deg=<arm>,
  target_alt_offset_m=<cell>)`.
- Arms: tilt ∈ {0 (current), +10, +20} deg.
- Cells: `target_alt_offset_m` ∈ {−2, 0, +1, +2, +3, +4} m; aim 0 only
  (affordability); n = 50 paired seeds (0..49) per cell-arm.
- Headline metric: %≤0.35 m; also median/p90 miss.
- Mechanism metrics (traced subset, 20 seeds, cells {−2, 0, +3} × all
  tilts): mean decoded box-center-v (px; frame is 800 tall, center 400 —
  larger = lower in frame), median approach-window length (ENGAGE → first
  decode), per-frame attribution shares (same instrument as the keepframe
  follow-up) to confirm the TOP-exit bucket shrinks; decode counts come
  from the main grid (`n_decoded`).

## Predictions (registered)

1. **+3 and +4 m cells improve materially at +10 or +20 deg** — register
   "materially" as ≥15 points on %≤0.35 m — via the raised ceiling; the
   mechanism table must show the +3 m TOP-exit share shrinking and the mean
   box-center-v moving DOWN-frame (larger v) at the winning tilt.
2. **0 and −2 m cells stay within ±5 points at +10 deg.** At +20 deg,
   register the expected SIGN of trouble: the co-altitude/below targets move
   toward the BOTTOM edge (fy·tan(20°) ≈ 140 px of down-frame shift at
   fx=385, plus ~100 px more for −2 m at close range), so **−2 m at +20 deg
   is expected to DEGRADE** (bottom-edge exit / corner clip at close range).
3. Geometry sanity: mean box-center-v increases with tilt in every cell.

## Adopt / reject criterion (registered)

**Recommend a printed-bracket angle T** iff, at tilt T: BOTH +3 m and +4 m
improve by ≥15 points AND NEITHER 0 m nor −2 m degrades by >5 points.
If +3/+4 improve ≥15 but the 0/−2 band pays >5: MIXED — report the
tradeoff, no recommendation without a finer angle sweep. Otherwise NULL.

## What a NULL means (registered)

If no tilt lifts the above-band cells, the binding constraint is not the
FoV ceiling but the TOO-SMALL bucket (35% of blind frames at +3 m): tilt
cannot buy range/pixels, only pointing. The bracket angle then stays sized
by other considerations, and the honest chase height-error band remains
−2..+2 m.

---

## RESULTS (appended after the run, 2026-09-23; `scripts/chase_tilt_ab.py`)

### Part 1 — main grid (n=50 paired seeds/cell-arm, %≤0.35 m)

| alt err | tilt 0° | tilt +10° | tilt +20° |
|--------:|--------:|----------:|----------:|
| −2 m | 78% (med 0.207) | 78% (0.196) | 80% (0.210, p90 1.13) |
| 0 m  | 94% (0.130) | 94% (0.111) | **86%** (0.116, p90 0.36) |
| +1 m | 88% (0.147) | 92% (0.126) | 90% (0.147) |
| +2 m | 72% (0.201) | **86%** (0.132) | 80% (0.204) |
| +3 m | 34% (0.420) | **70%** (0.294) | 68% (0.277) |
| +4 m | 26% (0.497) | 30% (0.508) | 38% (0.409) |

Median decodes (0°/10°/20°): −2: 82/79/74; 0: 94/85/80; +1: 86/90/78;
+2: 66/75/60; +3: 33/60/58; +4: 34/42/50.

### Part 2 — mechanism subset (n=20 traced seeds; TOP/BOT = share of
no-decode approach frames out the top/bottom edge; det_v = mean decoded
box-center row, px, 800-tall frame)

| cell | tilt | mean det_v | med window | %TOP | %BOT | %inframe-nodecode |
|------|-----:|-----------:|-----------:|-----:|-----:|------------------:|
| −2 m |  0°  | 389 | 2.03 s |  0.0 |  9.5 | 64.8 |
| −2 m | 10°  | 443 | 2.03 s |  0.0 |  9.4 | 61.6 |
| −2 m | 20°  | 488 | 2.03 s |  0.0 |  8.4 | 59.3 |
| 0 m  |  0°  | 342 | 1.89 s |  0.2 |  0.0 | 83.9 |
| 0 m  | 10°  | 398 | 1.83 s |  0.0 |  0.2 | 84.5 |
| 0 m  | 20°  | 461 | 1.80 s |  0.0 |  6.5 | 78.1 |
| +3 m |  0°  | 280 | 3.85 s | 40.8 |  0.0 | 57.0 |
| +3 m | 10°  | 390 | 1.81 s | 12.7 |  0.0 | 84.3 |
| +3 m | 20°  | 388 | 1.81 s |  0.6 |  0.0 | 97.9 |

### Read against the registered criteria

- **Prediction 1 — half held.** +3 m improved far past the bar at BOTH
  tilts (+36 pts at 10°, +34 at 20°) and the mechanism is exactly the
  registered one: TOP-exit share 40.8% → 12.7% → 0.6%, approach window
  3.85 → 1.81 s, det_v down-frame. **+4 m did NOT clear ≥15** (+4 pts at
  10°, +12 at 20°) — once the ceiling lifts, its blind frames are ~all
  in-frame-no-decode (too-small/range), the registered NULL mechanism,
  binding only above +3 m.
- **Prediction 2 — held, with the cost landing one cell over.** 0/−2 m at
  +10°: 0 and 0 points. The registered +20° bottom-edge SIGN appeared, but
  at the 0 m cell (94→86, −8 pts; %BOT 0→6.5) rather than at −2 m
  (78→80, though p90 doubled to 1.13 m — the tail noticed).
- **Prediction 3 — held** (det_v rises with tilt in every cell).
- **Adopt criterion — NOT met as registered** (it required BOTH +3 AND +4
  to clear ≥15): +10° fails only the +4 conjunct (band cost is ZERO);
  +20° fails +4 AND costs 0 m > 5. Per the registered letter this is a
  NULL-by-conjunction, and per the registered MIXED/NULL text the honest
  label is: **MIXED — mechanism-confirmed, zero-cost win at +10° for the
  +2/+3 m band; +4 m is range-limited and no tilt rescues it.**

### VERDICT: MIXED (registered criterion unmet on the +4 conjunct alone)

+10° is the standout: +2 m 72→86, +3 m 34→70, decodes 33→60 at +3 m,
mechanism table confirming the raised ceiling did it, and **no measured
cost anywhere** (−2/0/+1 within noise at n=50). +20° buys nothing more at
+3 m and starts paying at 0 m — the registered bottom-edge tax. The +4
conjunct in the registered criterion is the only blocker, and its failure
mode is the registered null mechanism (too-small dominates above +3 m —
tilt buys pointing, not pixels). RECOMMENDATION (within the prereg's own
MIXED path): no bracket-angle adoption from this doc alone; a finer angle
sweep (e.g. 8/12/15°) around +10° would settle the angle, and the +4 m
cell should be treated as outside the honest chase band regardless of
tilt. Decision on re-registering with the +3-only criterion belongs to
the builder/head session, not to a post-hoc reread of this one.

*(Coordinator ruling after the MIXED verdict: the +4 m cell is OUTSIDE the
honest chase height band — range-limited, no tilt rescues it; the boundary
is reported, not buried. Finer sweep authorized as a fresh registration —
prereg #2 below.)*

---

# PREREG #2 — fine angle sweep (2026-09-23, registered BEFORE flying)

FRESH registration; the coarse grid above is evidence-in-hand, not this
sweep's outcome. Goal: a single **bracket-angle RECOMMENDATION** (not an
adoption — the angle is the builder's print-time decision).

## Config (exact)

- New arms: tilt ∈ {+8, +12, +15} deg. The 0° and +10° columns are REUSED
  from Part 1 above (same cells, same seeds 0..49, same config) — not
  reflown. +20° is already measured and disqualified (0 m cost −8).
- Cells: alt-err ∈ {−2, 0, +1, +2, +3} m (+4 m excluded per the ruling
  above), aim 0, rear tag, scatter on, n=50 paired seeds (0..49).
- Same mechanism metrics, traced 20-seed subset at cells {−2, 0, +3}:
  TOP-exit share, mean decoded box-row, decode counts, approach-window
  length.

## Registered criterion for the recommendation

The recommended angle T must:
1. improve the +3 m cell by **≥25 points** vs 0°;
2. cost **≤2 points** on EACH of {−2, 0, +1, +2} m vs 0°;
3. not degrade p90 past **1.5×** the 0° value in any band cell.
Among angles meeting all three, recommend on the evidence (best +3 m lift;
ties broken toward the smaller angle — less bottom-edge exposure in
regimes this grid does not cover).

## Prediction (registered)

The win is roughly FLAT across 8–15° — the +3 m ceiling is fully lifted by
~10° on the 118° lens (the +20° mechanism row already showed TOP-exit at
0.6% with no further hit-rate gain) — and the band cost stays ~zero below
+15°. If instead the win keeps GROWING toward +15° with no cost, the
recommendation shifts up.

## What a NULL would mean (registered)

If no angle in {8, 10, 12, 15} meets the criterion — in particular if the
+3 m lift collapses — then the 34→70 at +10° was seed-luck (unlikely at
n=50 paired, but said here so it cannot be spun), and the recommendation
is withheld pending a larger-n rerun of the +10° column.

---

## PREREG #2 RESULTS (appended after the run, 2026-09-23;
`scripts/chase_tilt_ab.py --fine`; 0°/+10° columns reused from Part 1,
same seeds)

### Full angle table (%≤0.35 m, n=50 paired seeds; Δ vs 0° for +3 m)

| alt err | 0° | +8° | +10° | +12° | +15° | (+20°, disq.) |
|--------:|---:|----:|-----:|-----:|-----:|--------------:|
| −2 m | 78 | 78 | 78 | 78 | 84 | 80 |
|  0 m | 94 | 92 | 94 | 92 | 92 | 86 |
| +1 m | 88 | 94 | 92 | 92 | 92 | 90 |
| +2 m | 72 | 86 | 86 | 88 | 82 | 80 |
| +3 m | 34 | 46 (+12) | 70 (+36) | 76 (+42) | 70 (+36) | 68 |

p90 at +3 m: 4.18 / 4.07 / 1.00 / 0.90 / 0.54 m. Median decodes at +3 m:
33 / 45 / 60 / 61 / 62. Band p90s all within 1.2× of 0° for 8–15°.

### Mechanism (20 traced seeds; +3 m row is the story)

| tilt | +3 m %TOP-exit | +3 m window | 0 m %BOT | 0 m det_v |
|-----:|---------------:|------------:|---------:|----------:|
|  0°  | 40.8% | 3.85 s | 0.0% | 342 |
|  +8° | 23.6% | 2.12 s | 0.0% | 388 |
| +10° | 12.7% | 1.81 s | 0.2% | 398 |
| +12° |  7.4% | 1.81 s | 0.5% | 414 |
| +15° |  3.3% | 1.81 s | 1.5% | 428 |
| +20° |  0.6% | 1.81 s | 6.5% | 461 |

### Read against the registered criterion

- Criterion pass/fail per angle: **+8° FAILS** (#1: +12 < 25 — the ceiling
  is NOT yet lifted at 8°, TOP-exit still 23.6%); **+10°, +12°, +15° all
  PASS** (#1: +36/+42/+36; #2: worst band cost −2 points, at the bound,
  never beyond; #3: all band p90 ratios ≤ ~1.2×).
- Prediction: half-held. Flat across 10–15° as predicted, but NOT down to
  8° — the knee is between 8° and 10°, not below 8°. The win does NOT keep
  growing toward +15° (70 vs 76 vs 70), so the recommendation does not
  shift up. No null: the +10° result reproduced in two adjacent
  independent arms (+12°, +15°), so 34→70 was not seed-luck.
- Best +3 m lift is nominally +12° (76%), but 76 vs 70 is 3 runs at n=50 —
  below this project's ~±5/50 noise scale, i.e. a statistical TIE among
  {10°, 12°, 15°}. The registered tie-break (smaller angle, less
  bottom-edge exposure in uncovered regimes) applies, and the mechanism
  table backs it: the 0 m bottom-edge share creeps monotonically with
  angle (0.2 → 0.5 → 1.5 → 6.5% by +20°).

### RECOMMENDED BRACKET ANGLE: **+10°** (print-time decision = builder's)

Evidence line: at +10° the chase's +3 m-above cell goes 34% → 70% inside
0.35 m (n=50 paired seeds, all realistic errors on, p90 4.18 → 1.00 m,
decodes 33 → 60) by cutting the frame-top-exit share of blind approach
frames 40.8% → 12.7% and halving the acquisition window, at ZERO measured
cost on the −2..+2 m band — and +12°/+15° confirm the same plateau, so the
result is not one arm's luck. The honest chase height band with a +10°
bracket extends to roughly −2..+3 m; +4 m stays OUTSIDE the band
(range/too-small-limited — tilt buys pointing, not pixels; coordinator
ruling above). Numbers are isim (fast-sim) evidence: they rank and locate;
a Gazebo/bench confirmation is the usual next rung before the printed
bracket is treated as validated.
