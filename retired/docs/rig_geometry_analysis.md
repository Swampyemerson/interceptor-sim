# Ground stereo rig — geometry + rate analysis

*Companion to `docs/stereo_design.md` and `scripts/stereo_model.py` (the
physics), `scripts/m4_target_mover.py` (the flown geometry), and
`scripts/guidance_lab.py` (the Kalata/alpha-beta filter). Answers three
Phase-2 pre-build questions (`docs/phase2_sim_to_real_plan.md` Sec 5.6 items
#5, #7, #8) that T16 (the stereo rig world) owes before it's built. Pure
math — no Gazebo, no PX4, no sim runs. Everything here is reproducible by
running `scripts/rig_geometry_analysis.py`, which writes the CSVs under
`logs/rig_geometry/` and the plots under `plots/` that this doc cites.*

## Why this exists

The 2026-07-08 adversarial review of the Phase-2 plan (ADR-0045) found the
plan was about to build the stereo rig world (T16) without answering three
questions first:

1. **If Gazebo's real-time factor forces a lower render resolution than the
   1920×1200 design point, how much does that cost?** (a resolution cut
   raises the range-noise constant `c`, invalidating the fixed
   `c = 4.45e-05` the mock currently ships).
2. **Does the rig's narrow field of view actually SEE the target during a
   real engagement?** The rig sees a wedge, not a dome — nothing guaranteed
   the flown dash profile crosses it.
3. **What cue update rate does the mid-course fusion actually need?** The
   fusion arc assumed the cue emits velocity noise σ_v ≈ 0.5 m/s
   (`s2_cue_mock.py --vel-sigma 0.5`) without ever deriving that number from
   a track filter + measurement noise.

This doc answers all three, in order, because #2 feeds #3 (the rig's
pose determines the range at handoff, which sets the measurement noise
that the rate analysis needs) and #1 feeds #3 too (a resolution cut is one
of the two knobs the final question asks about).

---

## 1. What a resolution cut costs

**New term, once — HFOV (horizontal field of view):** the angular width a
camera sees, set by the lens. Wider = easier to search, narrower = more
"zoom" (more pixels on a distant target). `docs/stereo_design.md`'s chosen
rig is a 16 mm lens on the AR0234 sensor giving HFOV ≈ 20°.

The 2026-07-07 demo-camera episode (`worlds/apriltag_demo.sdf`'s
`demo_chase_cam` comment) already found that a SECOND full camera at
1280×720 collapsed Gazebo's real-time factor to ~0.05 — an order of
magnitude below the safe floor — while 960×540 held ~1.0. If the real
2-camera rig world hits the same wall, the fix is a resolution cut, which
this analysis prices out. We hold the LENS fixed (same 20° HFOV — the same
physical field of view, just rendered at fewer pixels, e.g. via binning or
a lower-resolution sensor with a matched lens) and shrink only the pixel
width:

```
f_px = (W/2) / tan(HFOV/2)          -- pixels-per-radian at this resolution
c    = sigma_match(tier) / (b * f_px)   -- the range-noise constant, matching-noise only
```

`sigma_match` is the sub-pixel matching-noise tier from `stereo_design.md`'s
error budget (0.1 / 0.4 / 1.0 px, BEST/EXPECTED/WORST) — the single largest
term (~71% of the variance at the design point), so `c` here is a lower
bound on the true, full 4-term budget (matching + calibration-drift + sync +
distortion). The script cross-checks against the full model too.

| Resolution | f_px | c (EXPECTED, m/m²) | R @ σ_R≤1m, EXPECTED (m) | R @ σ_R≤1m, WORST (m) | Detection floor, 0.3 m/10 px (m) |
|---|---|---|---|---|---|
| 1920×1200 (design) | 5444 | 3.67e-05 | **151** | 66 | 163 |
| 1280×960 | 3630 | 5.51e-05 | 126 | 61 | 109 |
| 960×540 | 2722 | 7.35e-05 | 109 | 57 | 82 |
| 640×480 | 1815 | 1.10e-04 | 90 | 50 | 54 |

*(`logs/rig_geometry/res_summary.csv`; `max_range...full_model_m` column is
the full-model cross-check, quoted above; `res_sigma_table.csv` has σ_R at
every {15, 30, 60, 100, 150 m} × tier × resolution combination; see
`plots/rig_geometry_res_sigmaR.png`.)*

**What this says.** Cutting the render to 960×540 (the RTF-safe resolution
the demo camera already validated) costs about a QUARTER of the usable range
at the 1 m gate (151 m → 109 m EXPECTED, a 28% reduction) — and knocks the detection floor
down from 163 m to 82 m, which is now BELOW the ranging floor: at 960×540,
DETECTION (82 m) becomes the BINDING limit, well short of where ranging
accuracy would still hold (109 m). At 640×480 both numbers keep shrinking together. None
of this rules out a resolution cut — the actual engagement ranges here (see
§2) are 13–160 m, mostly well inside even the 640×480 floor — but it is the
honest cost table the RTF-driven resolution decision needs.

---

## 2. Where to put the rig

**New term, once — wedge:** a stereo rig's field of view isn't a dome
around it, it's a WEDGE — the angular slice its lens actually points at.
Put the rig in the wrong spot and the target can fly right past it,
completely unseen, no matter how good the range accuracy would have been.

### The flown geometry (confirmed against code + a real log, not assumed)

`scripts/m4_target_mover.py` + `scripts/mc_batch.sh`'s `--geometry standard`
flight-plan generator: the interceptor launches near world origin; the
target runs a **straight line at fixed world-X = 6.5 m**, altitude 0.5 m,
sweeping world-Y from ±29.3 m (±jitter) through Y=0 and beyond, at
9–16 m/s. Two mirror-image directions ("r2l": start Y=+29.3 moving toward
−Y; "l2r": the mirror). Cross-checked against a real flown log
(`logs/m4_intercept_pronav_20260708T214600Z.csv`): `gt_tag_x` sits at
exactly 6.5000 for the whole flight while `gt_tag_y` sweeps 28.5 → −14 —
confirms the fixed-X, swept-Y line.

The HANDOFF latch (`m4_intercept.py`'s `S2["HANDOFF_RANGE_M"] = 10.0`) fires
when the camera measures ≥3 consecutive in-range detections. In that same
log, the first post-HANDOFF row shows interceptor at (1.86, 6.87) m, target
at (6.5, 16.04) m, true range 10.28 m — this is the empirical "handoff
point" used below (r2l; l2r is its Y-mirror, a disclosed approximation, not
a second measured flight).

### The wedge model

Two cameras, boresights PARALLEL (no toe-in — a real fronto-parallel stereo
pair), offset ±1.0 m along the rig's lateral axis (the 2.0 m baseline), each
with a 10° half-angle cone (half of the 20° HFOV). A point is "in the
wedge" only if BOTH cameras can see it.

### Seven candidate poses, both dash directions

| Pose | Standoff | Coverage (r2l / l2r) | Continuous cue-wait→handoff? | Range @ handoff (m) | Note |
|---|---|---|---|---|---|
| `origin_bore` (0,0), aimed at CPA | 6.5 m | 0.0% / 0.0% | No | 17.3 | Too close — the 10° cone only covers ±1.1 m either side of dead-ahead; the whole pre-CPA path is outside it |
| `origin_setback` (−10,0) | 16.5 m | 0.0% / 0.0% | No | 23.0 | Same problem, a bit worse |
| `broadside_60m` (−53.5,0) | 60 m (design floor) | 19% / 19% | No | 62.1 | Design-envelope standoff, still narrow |
| `broadside_100m` (−93.5,0) | 100 m | 48% / 48% | No | 101.3 | Better — target in view AT handoff, but not the whole approach |
| **`broadside_160m` (−153.5,0)** | 160 m (design ceiling) | **91% / 91%** | No (misses only the very first ~1 m of cue-wait, farthest/most oblique moment) | 160.8 | **Best practical (non-collinear) pose** |
| `corridor_end_north` (6.5,45), pointed down-range | varies | 100% / 100% | **Yes** | 29.0 / 61.0 | Best score on paper — but **COLLINEAR with the flight corridor** (rig sits ON the line the target and interceptor both fly) |
| `quartering_handoff` (−5,10), aimed at the r2l handoff point | varies | 12% / 0% | No | 13.0 / 28.5 | Aiming at one direction's handoff point buys ~nothing for the mirrored direction — asymmetric by construction |

*(Full table: `logs/rig_geometry/coverage.csv`; map: `plots/rig_geometry_wedge_coverage.png`.)*

### The counter-intuitive finding: farther back is BETTER, not worse

Standing closer to the corridor feels like it should help ("get a better
view"), but the math says the opposite: at fixed 10° half-angle, the
absolute ground swath the cone covers GROWS with standoff distance
(`swath = 2·D·tan(10°)`). At 6.5 m standoff (`origin_bore`) the swath is
only ±1.1 m; at 160 m it's ±28 m — wide enough to hold almost the entire
pre-CPA corridor. This is why `broadside_160m` (pushed all the way to the
rig's OWN design-envelope ceiling) beats every closer broadside candidate.

### The real trade-off this analysis surfaces: coverage vs. cue quality pull in opposite directions

Here is the honest tension the F1 council needs: the pose that WINS on
coverage (`broadside_160m`, 91%) has the WORST range accuracy, because it's
also the FARTHEST rig-to-target range (160.8 m — right at the edge of the
design envelope, where σ_R is already large). The pose with the BEST range
accuracy (`origin_bore`, handoff range only 17.3 m, σ_R ≈ 1.4 cm) has ZERO
coverage. There is no single static pose that wins both — this is exactly
why `scripts/rig_geometry_analysis.py`'s Analysis 3 sweeps EVERY candidate
pose's handoff range, not just the recommended one (see
`logs/rig_geometry/sigma_v_vs_rate.csv`, `range_label` column: `handoff_<pose>`).

### Honesty note (explicit, per the task)

At 13–61 m (most candidates) and even at 160.8 m (`broadside_160m`), the rig
is operating **below or at the low end of its own 60–160 m design
envelope** (`docs/stereo_design.md`). We checked this isn't being swept
under the rug: at these short-to-mid ranges, σ_R itself is trivially small
where it matters most — e.g. at `origin_bore`'s 17.3 m handoff range,
σ_R,EXPECTED ≈ 1.4 cm (`logs/rig_geometry/coverage.csv`,
`sigma_R_expected_at_handoff_m`). Range accuracy is a non-issue at short
range; the BINDING constraint at short standoff is purely the ANGULAR
wedge-coverage geometry computed above, not ranging precision. The one pose
that pushes range accuracy back into "this actually matters" territory
(`broadside_160m`, σ_R ≈ 1.13 m EXPECTED) is also the one that had to be
pushed to the outer edge of the design envelope to win the coverage
argument — so the two effects (short-range coverage failure, long-range
accuracy cost) are directly linked by the same standoff-distance knob, not
independent findings.

### Recommendation

`corridor_end_north` scores perfectly (100% coverage, continuous, both
directions) but sits literally ON the line both drones fly — a real
collision/siting hazard, not a genuine deployment recommendation, despite
winning the math. **`broadside_160m` is the recommended PRACTICAL pose** —
91% coverage, both directions, off the flight line — with the explicit
caveat that it does not achieve strict continuity through the very start of
CUE_WAIT (target is briefly ~1 m outside the cone at the farthest, most
oblique point of the approach) and that its far standoff drives σ_R at
handoff up to ~1.1 m (EXPECTED), feeding directly into §3's rate analysis
below. A genuinely gap-free design likely needs a slewing (pan/tilt, coarse
cue-fed) mount or multiple static rigs — out of scope for this analytic
pass, flagged for the F1 council.

---

## 3. What update rate does the rig camera need?

**New terms, once:**
- **Alpha-beta (g-h) filter:** a simple two-state tracker (position +
  velocity) that predicts forward every tick and corrects toward each new
  measurement by fixed gains α (position) and β (velocity). It's what
  `m4_intercept.py`'s `TargetTracker` already runs on the cue.
- **Kalata's tracking index (Λ):** a single number, `Λ = σ_w·T²/σ_R`
  (target-maneuver-noise × update-interval² ÷ measurement-noise), that
  tells you how much to trust a new measurement vs. your own prediction. A
  noisier/faster-maneuvering target or a slower update rate → larger Λ →
  larger gains (trust the measurement more).

### The formulas (Kalata 1984; reused, not re-derived, from `scripts/guidance_lab.py`'s `kalata_alpha_beta()` — the SAME function `m4_intercept.py`'s filters use)

```
lambda = sigma_w * T^2 / sigma_R
r      = (4 + lambda - sqrt(8*lambda + lambda^2)) / 4
alpha  = 1 - r^2
beta   = 2*(2-alpha) - 4*sqrt(1-alpha)
```

The steady-state velocity-estimate noise and position lag under a
maneuvering target are DERIVED here (from the discrete Riccati fixed point
of this exact filter) and independently VERIFIED against a 400,000-tick
Monte-Carlo replica of the real filter code — the closed-form and the
simulation agreed to within 0.5% at every tested tracking index:

```
Var(velocity error) = (sigma_R^2 / T^2) * ( alpha*beta/(1-alpha) - lambda^2/2 )
sigma_v = sqrt(Var(velocity error))
lag_m   = a * T^2 * (1 - alpha) / beta        -- position lag under constant accel a
```

**Simplification, disclosed:** this treats σ_R (the rig's RADIAL range
noise, the dominant, worse axis) as the Cartesian position-measurement
noise the filter sees. Cross-range (bearing) noise is 1–2 orders of
magnitude smaller (`docs/stereo_design.md`), so using σ_R is a deliberately
PESSIMISTIC, worst-axis input — consistent with this project's
"simulate worse than ideal" mandate.

### Target maneuver classes (per the T21 plan)

| Class | σ_w (target accel, m/s²) |
|---|---|
| gentle | 1.0 |
| weave | 3.0 |
| jink | 9.0 |

### Results — 1920×1200 (design resolution), EXPECTED σ_R

| Range point | σ_R (m) | Rate | gentle σ_v | weave σ_v | jink σ_v |
|---|---|---|---|---|---|
| `broadside_160m` handoff (160.8 m) | 1.131 | 5 Hz | 0.53 ✗ | 1.18 ✗ | 2.58 ✗ |
| | | 10 Hz | **0.38 ✓** | 0.86 ✗ | 1.92 ✗ |
| | | 15 Hz | 0.31 ✓ | 0.71 ✗ | 1.59 ✗ |
| | | 30 Hz | 0.22 ✓ | 0.51 ✗ | 1.14 ✗ |
| design envelope (100 m) | 0.438 | 5 Hz | **0.41 ✓** | 0.90 ✗ | 1.92 ✗ |
| | | 10 Hz | 0.30 ✓ | 0.67 ✗ | 1.47 ✗ |
| | | 15 Hz | 0.25 ✓ | 0.55 ✗ | 1.23 ✗ |
| | | 30 Hz | 0.18 ✓ | **0.40 ✓** | 0.89 ✗ |

*(✓ = meets σ_v ≤ 0.5 m/s, the fusion arc's assumed cue-velocity noise,
`s2_cue_mock.py --vel-sigma 0.5`. Full grid incl. all 4 resolutions × all 7
candidate poses' handoff ranges × 4 rates × 3 accel classes:
`logs/rig_geometry/sigma_v_vs_rate.csv`, 720 rows; plot:
`plots/rig_geometry_sigma_v_vs_rate.png`.)*

**What this says, plainly.**
- Against a **gentle** target (1 m/s²), even the slowest rate tested (5 Hz)
  comfortably meets the 0.5 m/s target at 100 m, and 10 Hz clears it even
  at the far 160.8 m handoff range. Rate is a non-issue here.
- Against a **weave** (3 m/s²), only the fastest rate (30 Hz) clears the bar
  — and only at the 100 m design-envelope range, not at the 160.8 m
  handoff range of the recommended pose.
- Against a **jink** (9 m/s²), **NOTHING TESTED clears 0.5 m/s** — not at
  any rate up to 30 Hz, not at any of the four resolutions, not at either
  range point. The best achievable (30 Hz, 1920×1200, 100 m) is still
  0.89 m/s, nearly double the target.
- Rate helps, but resolution and — most of all — RANGE dominate. Moving the
  rig from the recommended 160.8 m pose to a closer one (e.g.
  `broadside_60m`, 62.1 m) drops σ_R from 1.13 m to 0.17 m and lets even
  **weave** clear the bar at 15 Hz (0.43 m/s) — but that closer pose only
  covers 19% of the corridor and never continuously reaches the handoff
  point (§2). The rate question cannot be answered independently of the
  pose question.

---

## Implications for the F1 council

- **A resolution cut to 960×540 (the RTF-validated safe resolution) costs
  about a quarter of the usable ranging distance** (151 m → 109 m, a 28%
  reduction, at the EXPECTED tier's 1 m gate) and makes DETECTION, not
  ranging, the binding limit (82 m detection floor < 109 m ranging floor). Not disqualifying —
  the real engagement ranges here (13–160 m) mostly still clear it — but
  it's the real cost, not a free lunch.
- **No static, off-corridor rig pose among the 5 tested achieves
  continuous coverage from cue-wait through the 10 m HANDOFF for both dash
  directions.** The only pose that does (`corridor_end_north`) sits
  literally on the flight line — a genuine siting/collision hazard, not a
  deployable answer.
- **Counter-intuitively, standing FARTHER from the corridor gives BETTER
  angular coverage**, because the ground swath a fixed 20° HFOV covers
  grows linearly with standoff. The best practical pose found
  (`broadside_160m`) sits at the rig's own 160 m design-envelope ceiling,
  not close to the action.
- **Coverage and cue quality are in direct tension.** The pose that best
  covers the corridor (far standoff) has the worst σ_R (large range); the
  pose with the best σ_R (close standoff) covers almost none of the
  corridor. There is no free-lunch static pose — a real deployment likely
  needs a slewing mount or multiple rigs to get both.
- **Against a genuinely jinking target (9 m/s²), the σ_v ≤ 0.5 m/s
  assumption baked into the fusion arc (`s2_cue_mock.py --vel-sigma 0.5`)
  is NOT achievable by ANY (rate, resolution) combination tested at the
  recommended pose's range** — the mock's 0.5 m/s figure only holds for
  gentle-to-weave maneuvering, not hard jinks, once real stereo-rig range
  noise is honestly propagated through an alpha-beta filter.
- **Rate alone cannot buy back what range costs.** Going from 5 Hz to 30 Hz
  only shrinks σ_v by roughly the square root of 6× (an alpha-beta filter's
  noise floor falls slower than update count) — nowhere near enough to
  compensate a 160 m vs. 60 m range difference (σ_R itself differs by
  ~7×, R² scaling).
- **This whole chapter argues for the T20 fusion refinement's existing
  bias-state direction, reinforced from a new angle:** if the cue's
  velocity is this noisy against a maneuvering target regardless of rig
  engineering, the fusion layer needs to be robust to a genuinely noisy
  velocity channel, not just a biased one.
- **Concrete next step for T16:** site the rig near `broadside_160m`'s
  standoff (not closer), budget for the coverage gap at the very start of
  CUE_WAIT, and treat σ_v ≈ 0.5 m/s as valid ONLY for gentle/weave
  maneuvers in the fusion arc's flight plan — not as a general assumption.

---

## Reproduce this

```
.venv/bin/python scripts/rig_geometry_analysis.py            # tables + 3 plots
.venv/bin/python scripts/rig_geometry_analysis.py --no-plots  # tables only
```

Outputs: `logs/rig_geometry/res_sigma_table.csv`, `logs/rig_geometry/res_summary.csv`,
`logs/rig_geometry/coverage.csv`, `logs/rig_geometry/sigma_v_vs_rate.csv`;
`plots/rig_geometry_res_sigmaR.png`, `plots/rig_geometry_wedge_coverage.png`,
`plots/rig_geometry_sigma_v_vs_rate.png`.

## Sources

- Stereo range-error physics, error budget, tiers, chosen design point: `docs/stereo_design.md`, `scripts/stereo_model.py` (reused directly, not re-derived).
- Flown dash-profile geometry: `scripts/m4_target_mover.py` (path parameterization), `scripts/mc_batch.sh` (`--geometry standard` flight-plan generator), `logs/m4_intercept_pronav_20260708T214600Z.csv` (real flight, confirms fixed world-X, swept world-Y, and the empirical HANDOFF point coordinates).
- HANDOFF mechanics + range constant: `scripts/m4_intercept.py` (`S2["HANDOFF_RANGE_M"] = 10.0`, `S2["HANDOFF_STREAK_MIN"]`).
- Kalata tracking-index / alpha-beta gains: T.P. Kalata, 1984, "The Tracking Index: A Generalized Parameter for alpha-beta and alpha-beta-gamma Target Trackers," IEEE Trans. Aerosp. Electron. Syst. AES-20(2):174-182; reused from `scripts/guidance_lab.py`'s `kalata_alpha_beta()` (same function `m4_intercept.py`'s filters use).
- Steady-state velocity-noise and maneuver-lag formulas: derived in `scripts/rig_geometry_analysis.py`'s module docstring from the filter's discrete Riccati fixed point; independently verified against a 400k-tick Monte-Carlo replica of the exact filter mechanization (residual < 0.5% at all tested tracking indices; see this task's report for the verification script).
- RTF-vs-resolution precedent: `worlds/apriltag_demo.sdf` (`demo_chase_cam` comment, 2026-07-07 finding: 1280×720 second camera collapsed RTF to ~0.05; 960×540 restored ~1.0).
- Cue-velocity noise assumption being tested: `scripts/s2_cue_mock.py` (`DEFAULT_VEL_SIGMA_M_S = 0.5`, `--vel-sigma`).
- Phase-2 requirements this answers: `docs/phase2_sim_to_real_plan.md` Sec 5.5 (F1 feasibility scouting) and Sec 5.6 items #5, #7, #8 (ADR-0045 adversarial-review amendments).
