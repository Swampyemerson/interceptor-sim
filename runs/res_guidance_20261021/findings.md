# Findings — resolution × guidance law (vs spec.md + amendment)

Variant A = results.json (fx-only = zoom lens, mislabeled resolution).
Variant B2 = results_b2.json (same-FOV pixel scaling, both cameras scaled —
the honest model; the first variant-B run had a true/nominal cx mismatch,
an instrument bug, results_b.json kept for the record).

## Results (variant B2, med_miss m @ 9 / 12 m/s)

| res scale s (of 1280×800 @118°) | pursuit | tag (predictive PIP) |
|---|---|---|
| 0.5 (≈640 input on the 118° lens) | **1.53 / 3.81 (decoded 23/5!)** | 1.42 / 2.17 |
| 1.0 (1280 native) | 0.148 / 0.200 | 1.47 / 2.25 |
| 2.0 | 0.133 / 0.242 | 2.92 / 3.50 |
| 3.0 | 0.137 / 0.263 | 4.61 / 5.02 |

## Conclusions (per the pre-registered criteria)

1. **NULL on the flip: resolution does NOT re-enable the predictive law.**
   The tag/PIP law never beats pursuit at any s; it gets WORSE with more
   pixels (it decodes less than pursuit at every s — the pointing problem is
   its binding constraint, not bearing noise). The chase/pro-nav hybrid idea
   needs a different mechanism than pixels (e.g. predictive mid-course that
   hands off to chase, or the bank-angle accel cue ADR-0073).
2. **The builder's 640 fear is VALIDATED for the wide lens.** At 118° HFOV a
   640-wide input collapses even the chase terminal (R50(22 px) ≈ 2.6 m —
   under the 8 m terminal floor from runs/nn_requirements_20261021).
3. **The lever is FOV-coupled, in pixels-per-degree:**
   - delivered 70° lens (fx 1104 @1280): 640 input ≈ s 1.4 → OK; 1280 ≈ s 2.9.
   - desired 118° lens (fx 385 @1280): 640 input ≈ s 0.5 → DEAD; native
     1280 ≈ s 1.0 → works, no margin.
   So: if the 118° lens ever returns (the wide-fov constraint says it
   should), the detector MUST run ≥1280-wide-equivalent input — which only
   the accelerator path can do inside the 60 ms latency budget. On the
   delivered 70° lens, 640 clears the terminal spec with margin.
4. Variant A side-finding: a narrower (zoom) lens is NOT a resolution
   substitute — FOV loss collapses pointing for both laws.

## Honesty

Model-derived (isim decode surface, open transfer assumption); diverging,
monotone effects well above run noise. Gazebo re-earn before any lens or
input-size decision is final. n=8/cell.
