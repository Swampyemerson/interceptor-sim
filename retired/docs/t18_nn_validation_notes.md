# T18 NN-centroid validation notes — real-detector σ_R vs the physics model

Companion to `scripts/t18_nn_sigma_validation.py` (run it to reproduce every
number below). This is the NN-centroid extension of `scripts/
t18_sigma_validation.py`'s perfect-centroid floor: same `ground_station.
triangulate.triangulate()` call, same capture
(`logs/rig_captures/full_sweep_20260709T015530Z/`), but the centroid comes
from running `scripts/seeker/weights/ground_v1.pt` (ADR-0047's single-class
ground detector) on the actually-captured frames instead of projecting
ground truth. Pure offline — no Gazebo/PX4, no live sim.

Run:
```
.venv-seeker-train/bin/python scripts/t18_nn_sigma_validation.py
```

## Headline numbers (this run)

- **Detection miss rate:** 4/112 frames (3.6%), all 4 misses on frames
  `label_rig_captures.py` itself rejected as off-frame/too-small (never
  labeled at all — not a detector failure on a labeled target). Every
  labeled frame (train or val) was detected at conf≥0.25.
- **Empirical σ_R** (range-residual std over the 52 pose-pairs with both
  cameras detecting):
  - **ALL** frames: n=52, σ_R = **8.00 m** (mean bias +1.63 m) — dominated
    by 2 outlier poses (see below).
  - **CLEAN** (neither camera frame in ground_v1's TRAIN split): n=10,
    σ_R = **0.48 m** (mean bias −0.16 m).
- **Implied σ_d** (per-camera detected-vs-gt-projected centroid residual,
  pooled Var(u_l)+Var(u_r) convention, comparable to `stereo_model.
  PARAMS["sigma_match_px"]`'s 0.10/0.40/1.00 px tiers):
  - ALL: 3.64 px. CLEAN: **0.615 px**.
- **Measured σ_R vs both models, at this capture's own mean range (~161 m):**
  - Resolution-scaled matching-only model (EXPECTED σ_d=0.4 px, c≈3.67e-05):
    predicts 0.95 m.
  - Mock's assumed EXPECTED constant (ADR-0017, c=4.45e-05, a FULL 4-term
    budget — matching + calibration-drift + sync + distortion, structurally
    larger than the matching-only model by construction): predicts 1.15 m.
  - ALL-population measured (8.00 m) is **NOISIER than both** (6.9–8.4×).
  - CLEAN-subset measured (0.48 m) is **CLEANER than both** (0.42–0.51×,
    i.e. under half the mock's assumed budget).

## Honesty caveats (read before citing any number above)

### 1. Single range band — no range sweep

All 56 captured poses sit at **~160.1–162.7 m** (one static rig pose, one
sweep). This validates σ_R at the **handoff-range band only**. It does
**not** sweep range, and the σ_R ∝ R² scaling the physics model predicts is
**untested here** — would need multiple captures at different ranges. Same
caveat `t18_sigma_validation.py`'s own real-capture check (2) already
discloses for the perfect-centroid mode; it applies identically here.

### 2. The "implied σ_d" is a matching-noise proxy, not independent ground truth

The GT-projected centroid used to score detections
(`triangulate.project_world_to_stereo_pixels`) is the **exact geometric
inverse** of what `triangulate()` uses to turn centroids back into range.
There is no second, independent measurement of "where the prop disc really
sat in the frame" available offline — the projection and the inversion
share the same pinhole model. So "implied σ_d" is an honest
detector-centroid-vs-geometry **matching-noise proxy**, useful for
comparing against `stereo_model.py`'s σ_d tiers, but it is not an
independently-verified pixel-accuracy ground truth in the way a hand-labeled
second annotation pass would be.

### 3. Train/val leakage (found while building this harness — not in the original task brief)

`scripts/seeker/data/ground_dataset_v1/split_meta.csv` (read-only
cross-reference; this harness does not touch that dataset) shows that
`ground_v1.pt` was **trained** using labels manufactured
(`label_rig_captures.py`) from **this exact capture directory**:

| | count | of labeled positives |
|---|---|---|
| TRAIN split | 88 | 81% |
| VAL split | 20 | 19% |
| Never labeled (rejected, off-frame/too-small) | 4 | — |

Running the detector over **all** 112 frames (as the task literally asks)
therefore measures **mostly near-memorized performance**, not held-out
generalization — the ALL-population σ_R (8.00 m) and implied σ_d (3.64 px)
above should be read with that in mind. The **CLEAN** subset (pose pairs
where *neither* camera frame is in ground_v1's TRAIN split — VAL or
never-labeled only, n=10 pairs / n=10-per-camera pixel residuals) is the
honest, non-leaked signal, at the cost of a much smaller n. VAL-split
frames are already independently covered by `check_t17.sh`'s own near/far-
band recall/precision gate (not σ_R — that gate never triangulates).

### 4. Two outliers dominate the ALL-population number

Poses seq=2 (r2l pass) and seq=30 (l2r pass) — both at the very start of
their dash, near the corridor's off-boresight entry where the target is
smallest/most likely to be partially cut off — produced huge range
residuals (+43.6 m, +39.0 m) because the **lower-confidence side** of the
pair (conf 0.31 and 0.58 respectively, vs 0.93-0.96 on the other side and
across the rest of the sweep) placed its box center on a slightly wrong
pixel. At ~160 m with a ~2 m baseline and f_px≈5445, true disparity is only
~5 px — compressing an already-tiny disparity by a couple of pixels blows
up range non-linearly (Z = b·f_px/disparity is ill-conditioned in the
small-disparity regime). These 2 of 52 rows account for essentially all of
the ALL-vs-CLEAN gap; they were **not** filtered by any outlier-rejection
rule in this harness (the per-row CSV flags them with the raw residual so
they're visible, not hidden). Both happen to be TRAIN-split frames, so they
don't appear in the CLEAN subset — that exclusion is incidental to the
small-n split, not a deliberate cleanup.

### 5. The CLEAN subset (n=10) is honest but small

n=10 pose-pairs is not enough to assert a tight confidence interval. The
CLEAN σ_R (0.48 m) landing under both models, while the CLEAN implied σ_d
(0.615 px) landing *above* the EXPECTED tier's 0.4 px assumption, is an
internal tension worth flagging plainly: the two independent statistics
(range-residual std vs per-camera pixel-residual std) computed from the
*same* 10 pose-pairs don't cross-check tightly (cross-check ratio ~0.33×,
printed by the harness) — exactly what wide sampling variance at n=10 looks
like, not a mechanism disagreement. Treat the CLEAN numbers as "promising,
not proven"; a larger held-out (non-train) real-detection sample is the
natural next step before leaning on this number for a design decision.

### 6. Addendum — even the "CLEAN" (val) split is near-duplicate of TRAIN, not a fresh viewpoint

The concurrent T17 verifier finding (logged as ADR-0049 in `docs/
decisions.md`, discovered independently of this harness) sharpens caveat #3
further: `ground_dataset_v1`'s train/val split is held out **by pose
sequence**, but consecutive sequences in this single 56-pose sweep are only
~0.3° apart in off-boresight angle (~0.02% of pixels differ frame-to-frame)
— "held out" here means *interpolation from immediate neighbors*, not a
distinct viewpoint or range. ADR-0049's own conclusion applies directly to
this harness's numbers: **T18's NN-mode σ_R / implied-σ_d at 160 m is a
BEST-CASE lower bound, not a robust envelope measurement** — this includes
the CLEAN (n=10, val-only) subset above, which is train-*adjacent*, not
train-*independent*. Read the CLEAN numbers as "how good the pipeline looks
under near-ideal, near-memorized conditions," not as a generalization
guarantee. A genuine held-out test needs a second, independently-captured
sweep (different day/pose/lighting) that never touched `ground_v1`'s
training data at all — not built here.

## Plain-English bottom line

At the ~160 m handoff-range band, on the frames formally outside ground_v1's
TRAIN split (n=10, honest but small — and per caveat #6, train-*adjacent*
rather than train-*independent*), the real detector's range noise
(σ_R≈0.48 m) comes in **under** both the resolution-scaled physics model
(0.95 m) and the mock's current assumed constant (1.15 m) — so, AS A
BEST-CASE / near-ideal-conditions signal, this check does **not** turn up
evidence that the mock's σ_R assumption is too optimistic. It is **not**
strong enough evidence to call fusion conclusions safe outright: n=10 is
thin, the range band is single and narrow, and even this "clean" split is a
best-case lower bound (ADR-0049), not a true out-of-domain generalization
test. The full-population number (what you'd get by naively running the
detector over "all the frames we have") is dominated by data leakage and 2
low-confidence-side outliers and should **not** be quoted as a real-world
σ_R estimate at all — it overstates the noise by ~7-8× purely from
train-set memorization effects plus two ill-conditioned low-disparity
mispicks, not from a genuine detector-accuracy problem. **Bottom line: no
red flag raised, but this is not yet strong enough evidence to call the
question closed** — the natural next step is a genuinely independent
capture (different day/pose/lighting, never seen by ground_v1's training)
before leaning on this number for a design decision.
