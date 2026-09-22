# Dual-tag perpendicular pair (rear + side) — pre-registration (2026-09-21)

Builder question: mount two AprilTags roughly perpendicular at the back of the
target, so the seeker sees one from astern and the other from the side. Does
the second tag pay?

## Config (registered BEFORE the runs)

Native pursuit concept (`concept="pursuit"` — the validated prototype, no
flight-code wrapper confounds), **scatter ON** (`Scatter()` defaults — the
all-errors-on condition where ADR-0103 measured the rear tag's weak cells:
82% nominal but 60%/4% at 20/30° aim error). Both arms wrapped identically in
own-state noise by `build()`, so the comparison is symmetric.

- Control arm: `tag_facing="rear"` only.
- Test arm: `tag_facing="rear"`, `second_tag_facing="side"` (the
  `DualTagSeeker`, prefers the rear tag when both decode).
- Cells: aim_error_deg = 0 / 10 / 20 / 30. n=50 seeds per cell, identical
  seeds both arms. Score: fraction inside 0.35 m + median miss.

## Prediction

Nominal and 10° change little (the rear tag already faces the astern
approach). The 20°/30° cells improve materially in the dual arm — those are
the cells where the approach azimuth shifts and the rear face goes oblique,
which is exactly the viewing-angle coverage a side tag adds.

## What a NULL means

If the pair does NOT recover the high-aim-error cells, the failure there is
not tag visibility (likely the belief geometry itself is too wrong to bring
the seeker into either tag's cone), and a second tag does not buy aim-error
tolerance — the mitigation would have to be upstream (cue accuracy), and the
perpendicular mount is not worth its rigging complexity for that purpose.

## RESULT (same day): a TRADE, not a free win — prediction half right

n=50/cell, identical seeds both arms, fraction inside 0.35 m (median miss):

| aim err | rear-only     | rear+side     |
|--------:|---------------|---------------|
|      0° | 44/50 (0.177) | 39/50 (0.213) |
|     10° | 42/50 (0.193) | 35/50 (0.212) |
|     20° | 35/50 (0.250) | 32/50 (0.252) |
|     30° | 22/50 (0.403) | 28/50 (0.264) |

The registered prediction held at 30° (the pair recovers the weak cell,
44%→56%, median 0.403→0.264 m) but was WRONG about "nominal changes little":
the pair is consistently WORSE at 0–20° (−3 to −7 of 50). Mechanism
hypothesis, not yet traced: the DualTagSeeker falls back to the side tag on
rear-miss ticks, and in the astern geometry the side tag is near-edge-on —
its foreshortened width inflates the range fix, so the fallback injects
worse measurements exactly when they are least needed. At 30° the approach
azimuth is off enough that the side tag gets honest incidence and genuinely
extends coverage.

Honest limits: single-cell deltas of 3–7 out of 50 are suggestive, not
decisive at this n; and the 30° win leans on the sim's oblique-decode model,
which is only validated to ~33° incidence (tripod day A3 measures the
truth). Design note for the mount decision: a smarter chooser (ignore the
second tag above an incidence/width threshold) could plausibly keep the 30°
rescue without the nominal cost — unbuilt, and only worth building if the
cue-error budget actually extends past ~20°.
