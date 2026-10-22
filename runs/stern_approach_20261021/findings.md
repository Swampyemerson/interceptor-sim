# Stern-approach geometry sweep — FINDINGS (2026-10-21, registered)

Spec: `isim/specs/stern_approach_2026-10-21.md` (3 pre-data amendments
logged there). 48 cells × 8 paired seeds, isim hybrid concept at the earned
9 m/s point. Raw grid: `results.json` / `cells.jsonl`.

## Verdict: NULL — the defaults stand, the router is NOT justified

Applied adopt/reject rule: change a default only if a cell beats the default
(`d_behind=8, dash_aim_behind=0`) on acquisition probability beyond the
paired-seed noise band AND does not worsen median CPA.

- **No cell qualifies.** The largest acquisition gain anywhere is 62.5% vs
  50.0% at n=8 — exactly one flight, inside the noise band.
- **`dash_aim_behind_m` (the astern dash aim) is actively HARMFUL:** it never
  improves engagement at any error level and monotonically worsens median
  miss (at 8° cue error: 2.09 m → 5.59 m as the astern shift grows 0 → 32 m).
  Aiming the dash behind the believed track spends closing time/geometry and
  buys no acquisition — the lr3 decode-cliff failure mode, induced on purpose.
- **Cue error dominates everything:** engagement 87.5% → 50% → 37.5% and
  median decodes 70 → 4 → 2 as deliberate aim error goes 0° → 4° → 8°,
  swamping every geometry cell within each error level.

## What the null means (registered in advance)

The stern-approach intuition is already fully priced into the existing 8 m
Phase-A trail. The remaining misses are a **cue-error-vs-decode-range
budget** that no dash-geometry change can buy back; the real lever for
"ensure it grabs the intercept" is **acquisition range** (placard size,
camera, detector) and **cue quality** — exactly the two quantities the
tripod day measures. The stage-3 "dash router" (path optimizer) is therefore
NOT built: it would optimize variables measured here as non-binding.

## Footnote (not an adoption)

At zero cue error, `d_behind=4` halves median miss vs the default
(0.24 vs 0.51 m) with equal engagement. Acquisition-gated rule says no
adoption; if terminal CPA at low cue error ever becomes the binding
question, a focused `d_behind ∈ {4,8}` study at n≥32 is the cheap follow-up.

## Honest bounds

isim, fitted envelope, 9 m/s crossing, rear-facing tag, n=8/cell. The
hr-class 36 m/s envelope and real detector were NOT tested (regime rule);
nothing here validates or refutes geometry effects outside this regime.
