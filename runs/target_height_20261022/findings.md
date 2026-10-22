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
