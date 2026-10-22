# Target-height cue-error tolerance — FINDINGS (2026-10-22, registered)

Spec: `isim/specs/target_height_error_2026-10-22.md`. 7 offsets × 16 paired
seeds, canon hybrid base, aim error 0 (vertical axis isolated). Raw:
`results.json` / `cells.jsonl`.

| offset (true−believed, m) | engage% | med miss | p90 | decodes |
|---|---|---|---|---|
| 0 | 93.8 | 0.429 | 0.89 | 69.5 |
| +1 / −1 | 87.5 / 87.5 | 0.53 / 0.51 | 1.8 / 2.4 | 59.5 / 56.5 |
| +2 / −2 | 87.5 / 87.5 | 0.34 / 0.39 | 2.2 / 3.0 | 55.5 / 59.5 |
| **+3** | **68.8** | 0.85 | 3.2 | 36.5 |
| −3 | 87.5 | 0.47 | 3.8 | 52.0 |

## Verdict (by the registered rule)

**The cue's height budget is ±2 m**: engagement stays within one paired
seed (2/16) of baseline and median miss ≤2× through ±2 m. At **+3 m (target
ABOVE belief) acquisition breaks** (engage −25 pts, decodes halve); −3 m
still engages. So:

- Prediction 1 (±1 m recovered once acquired) **confirmed**.
- Prediction 2 (±3 breaks acquisition, not the terminal) **half-right** —
  only the above-belief side breaks.
- Prediction 3 (below-belief hurts more) **WRONG — the asymmetry is
  reversed**; mechanism unconfirmed (plausibly the dash's climb + nose-down
  wedge favors re-finding LOW targets; not measured — do not quote a
  mechanism).
- Honest caveat: p90 inflates with |offset| in every cell (1.8–3.8 m) even
  where the median is fine — the tail is the late/never-acquired flights;
  the budget above is an engagement/median budget, not a tail guarantee.

## Register action

`target-height-known` regraded **given-perfect → given-noisy** with the
measured budget: the real cue's target-height feed must be good to **±2 m**
(and above-belief bias is the dangerous side). This is now a written
requirement on the cue interface (GPS/baro of the target), alongside the
horizontal aim budget.

## FOV probe (registered amendment) — hypothesis REFUTED, mechanism OPEN

Measure-only seeker spy, offsets {0, +3, −3}, n=8: pre-acquisition in-FOV
fractions **0.47 / 0.42 / 0.56** — the +3 target is in frame at comparable
rates (nothing like the predicted collapse), yet only 4/8 acquire (vs 7/8
baseline, 6/8 at −3). So the wedge-coverage hypothesis is WRONG: the
asymmetry lives downstream of visibility (candidate suspects, none tested:
apparent-size/decode at the longer slant range to a high target, incidence,
Phase-A vertical aim dynamics). **Per the registration: no "bias the cue
low" sign rule may be quoted until the real mechanism is found.** Honest
note: the probe's v_px channel was vacuous (FrameReport carries no v_px —
that is a Detection field); the verdict rests on the binary in_fov only.
Raw: probe_fov.json.

## Probe round 3 — MECHANISM CHARACTERIZED (standoff-bounded pass)

First decodes occur at ~28–34 m in EVERY cell (decoder identical). The
acquire/fail split is min slant range: acquiring flights close to ≤1.1 m;
every failure bottoms out at 2.2–4.0 m — at +3, at ≈ the vertical standoff.
Mechanism chain: the sparse far decodes race to correct the vertical belief
BEFORE the pass; when they lose, the pass is standoff-bounded at the height
error and the close-range burst that fills the acquire window never
arrives. H3 confirmed with that sharpening (it is a belief-correction race,
not a decode-envelope cliff). STILL OPEN (one layer down): why the race
favors below-belief targets (6/8 vs 4/8 pull-outs) — vertical authority or
Phase-A z-aim asymmetry, unmeasured; the ±2 m budget stands regardless and
still carries no sign rule. Scope note (review R2-F4): baseline also has
one never-got-close failure (seed 5, min_range 2.2 m at zero offset) —
horizontal scatter alone produces the same signature, so the mechanism
explains the +3 EXCESS failures (4/8 vs 1/8), not every close-failure.
Raw: probe_fov.json (round-3 schema).
