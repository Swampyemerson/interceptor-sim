# Pre-registered: does higher detector resolution re-enable predictive guidance?

2026-10-21. Builder question (decision consult, verbatim concern): "640
scares me... If it was higher res, pro nav or a chase pro nav hybrid may
become possible again with higher bearing accuracy. Maybe look into this
more." Builder questions are probable findings — this is the dig.

## Question

The predicted-intercept-point law (`flight/tag_terminal.TagInterceptGuidance`,
isim `terminal="tag"`) was historically beaten by chase pursuit under noisy
monocular bearings. Higher sensor resolution scales BOTH levers at once:
bearing noise ∝ 1/fx and detection range ∝ fx (decode thresholds are in
pixels). Does the law ranking flip as resolution rises?

## Method

isim MC, canonical crossing geometry, scatter on, speeds {9, 12} m/s, n=8
seeds/cell. Axis: resolution scale s ∈ {0.5, 1, 2, 3} applied to
`Scenario.cam_fx_px = 385·s` (0.5 ≈ a 640 downscale of the 118° lens, 1 =
1280 native, 2 ≈ a 2.6 MP-class sensor or narrower lens, 3 = upper bound).
Decode params stay default, so detection range and bearing noise follow fx
naturally. Laws: `terminal="pursuit"` vs `terminal="tag"`. 2×4×2 = 16 cells.

## Pre-registered criteria

- Per cell: med_miss_m, pct_engage (n=8; single-cell deltas < ~0.15 m are
  noise — rank only on consistent cross-speed patterns).
- "Predictive re-enabled at s" iff tag-law med_miss ≤ pursuit med_miss at
  BOTH speeds at that s, with pct_engage ≥ 87.5.
- Null (no flip anywhere): resolution alone does not rescue the predictive
  law in this geometry — the builder's hybrid idea would then need a
  different mechanism (e.g. predictive mid-course + chase endgame), not
  just pixels.

## Honesty

Model-derived (same isim decode surface caveats as
runs/nn_requirements_20261021). Ranking evidence only; Gazebo decides.

## AMENDMENT (post variant A, before variant B ran)

Variant A (fx-only scaling, results.json) conflated resolution with FOCAL
LENGTH: fixed sensor width means fx·s is a ZOOM lens (118° -> ~56° at s=3),
and BOTH laws collapsed at s≥2 because the target leaves the FOV (decoded
count fell 102 -> 33 for pursuit). Finding kept: a narrower lens is NOT the
resolution lever — FOV is load-bearing for pointing.

Variant B (the builder's actual idea): more PIXELS at the SAME lens — scale
{fx, fy, width, height, cx, cy} together via `camera_overrides`
(s ∈ {0.5, 1, 2, 3} of the 1280x800 base). Bearing noise ∝1/s, detection
range ∝s, FOV constant. Same laws/speeds/seeds/criteria as registered.
