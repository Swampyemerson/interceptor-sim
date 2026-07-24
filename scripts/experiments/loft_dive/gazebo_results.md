# Loft-dive + accel-cap — Gazebo A/B results (Phase A confirmation)

*Companion to `gazebo_run_recipe.md` (the recipe this executed) and
`inframe_ab.py` (the analytical A/B this confirms/breaks). "Lab ranks, Gazebo
decides" — this is the Gazebo verdict. Date: 2026-07-22.*

## TL;DR — the two numbers that matter

1. **In-flight 8-12 m REAL-detection recall LIFTED ~12x: 3.0% (baseline, ARM A)
   → 35.4% (lever, ARM B).** The pointing lever works in Gazebo exactly as the
   analytical A/B ranked it — the accel-cap + loft-dive + 10° wedge CENTERS the
   8-12 m band (in-camera vert_cam median **−25.7° → −3.6°**, i.e. from the
   frame-top edge to near-boresight) and the dive HOLDS the near field
   (<5 m REAL recall 25% → 82%, no out-the-bottom). **Mechanism CONFIRMED.**
2. **Real camera-tracking happened in BOTH arms — but the miss got WORSE, not
   better.** REAL-detection ENGAGE on ARM A 8/8, ARM B 7/8 flights (multiple
   sub-1 m camera-tracked intercepts of the 3D quad exist in ARM A). Paired
   miss is **worse on 7/8 ARM-B flights** (median 0.75 m → 2.93 m; Pk@2.5:
   6/8 → 1/8). This is the recipe's **honest expectation**: at 9 m/s the
   accel-cap's closing-speed penalty (handoff speed 4.7 → 3.0 m/s) dominates
   the framing gain. **Pointing is confirmed; as a standalone lever it does
   not net a miss win at 9 m/s — it needs Phases B/C to convert framing → miss,
   or a lighter cap.**

## Config (exact)

Paired seeds, byte-identical geometry (canonical line-9, `--master-seed 123`,
`--x0 6.5 --y0-mag 15.343 --speeds 9.0`, `--directions both`, n=8 each),
markerless seeker on `drone_finetuned_quad_v2.onnx`, quad_enemy world /
fpv_quad_enemy 3D target, orient-to-velocity ON. Launcher: `run_arm.sh`
(mirrors `coded_dash_arm.sh` env). One sim at a time, idle load, sequential.

- **ARM A (baseline / control):** `--coded-dash --fpv --dash-unclamp
  --dash-speed 16 --dash-crossing-bias-deg 30`. Flat 0.5 m dash, level camera.
- **ARM B (lever):** ARM A **+** `--dash-accel-cap 3.57 --dash-loft-m 2
  --dash-loft-dive-s 2.5 --dash-vvert-max 3.0 --cam-mount-up-deg 10`. The 10°
  physical wedge is fitted as a `models/mono_cam` shadow symlink →
  `scripts/experiments/uptilt_mounts/up10` (generated from up15; only the
  imager `<pose>` pitch differs); `--cam-mount-up-deg 10` composes the matching
  math derotation. `run_arm.sh` owns the symlink lifecycle (create→fly→remove).

Smoke test (`run_arm.sh smokeB`, n=1) passed first: the new flags boot and fly,
takeoff pre-climbs to 2.5 m (ALT_REF 0.5 + loft 2.0), CODED_DASH dives back,
`[coded-dash] Phase-A POINTING levers: accel_cap=3.57 loft=2.0` logged, clean
teardown, RTF healthy, GPU render on, dxgk flat.

## The 5 decisive numbers (recipe §"What to measure")

Pooled over n=8 per arm; "REAL" = `coded_dash_summary._is_real_target`
(|meas_range−gt_range|/gt_range < 0.5 — gt for SCORING only, never in the loop).
Band = INBOUND (through-CPA) CODED_DASH+ENGAGE ticks in the gt slant-range band.
Pitch/speed from the matched PX4 ulog `vehicle_attitude`/`vehicle_local_position`
(alt-fit clock alignment, residual < 0.05 m on every flight).

| # | Metric | ARM A (baseline) | ARM B (lever) | Verdict |
|---|---|---|---|---|
| 1 | **8-12 m REAL recall** (headline) | **3.0%** (2/67) | **35.4%** (28/79) | **~12x lift — CONFIRMED** |
| 1b| 8-12 m any-detection | 41.8% | 54.4% | — |
| 2 | body pitch through band (per-flight med) | −31° (min −57°, bimodal) | **−26°** (min −32°, tight) | cap held ~26° (predicted 20°; see caveat) |
| 2b| in-camera vert of target (mount-adj) | −25.7° (near frame-TOP) | **−3.6°** (near-centered) | **centering CONFIRMED** |
| 3 | **<5 m REAL recall** (dive held?) | 25.4% (18/71) | **82.4%** (42/51) | **dive held, no out-the-bottom** |
| 4 | miss median / Pk@2.5 m | **0.75 m / 6/8** | 2.93 m / 1/8 | **miss WORSE (7/8 paired)** |
| 4b| flights with REAL-detection ENGAGE | 8/8 | 7/8 | real camera-track both arms |
| 5 | closing speed at handoff (med) | 4.7 m/s | **3.0 m/s** | cap cost ~1.6 m/s (the tradeoff) |

**Paired miss (B−A):** +0.71, +6.00, +1.96, +2.44, +1.93, −0.23, +2.06, +1.05
→ ARM B worse on **7/8** paired flights (the one near-tie, run 5 r2l, was a
slow low-speed engagement in both arms). ARM B run 1 (r2l) was a near-total
miss (6.68 m, 0 band ticks, 0 real ENGAGE, ~0 handoff speed) that inflates the
ARM-B mean; even excluding it, the paired direction is unchanged (B worse).

## Pitch caveat (recipe §2 / caveat)

The cap held pitch to a **tight ~26°** through the band (per-flight medians
clustered −25±2°, min −32°) vs the baseline's bimodal, spiky −14…−57°. That is
the accel-cap unmistakably doing its job. It sits **above** the predicted
arctan(3.57/9.81)=20° because `--dash-accel-cap` only shapes the *commanded*
velocity while PX4 (`MPC_ACC_HOR_MAX`=12/20 under `--fpv`) can still pitch
harder to *track* the ramp. Per the recipe caveat, also setting
`MPC_ACC_HOR_MAX ≈ 3.57` would pull achieved pitch toward 20°. It was not
needed to confirm the mechanism here (centering already reached vert_cam −3.6°).

## Mechanism read (why the framing win didn't become a miss win)

The analytical A/B's exact prediction reproduces in Gazebo: capping forward
accel holds a tight, bounded body pitch and the 10° wedge + loft put the 8-12 m
band on the boresight (vert_cam −3.6°), lifting REAL recall an order of
magnitude and holding the near field through CPA. That is the pointing wall
coming down — the thing Phase A exists to prove.

But at 9 m/s (ZEM-hard, ADR-0027) the **accel-cap slows the run-in** (handoff
closing speed 4.7 → 3.0 m/s; the quad never nears 16 m/s in the ~1.5-2 s
engagement, and the cap makes it slower still). Lower closing speed against a
9 m/s crosser means less terminal authority to kill the cross-range ZEM, so the
miss grows even though the target is now well framed. **Documented cost, not a
bug:** pointing is necessary but not sufficient at 9 m/s.

## Honesty

- **Own-state trajectory shaping:** `--dash-accel-cap`, `--dash-loft-m`, the
  ramp and the dive are computed in `flight/guidance.py`
  (`dash_forward_speed`, `dash_loft_alt_ref`) from dash parameters + sim-clock
  elapsed only — no camera, no `gt_*`. `gt_*` is used ONLY to SCORE recall/CPA.
- **Terminal detections are REAL:** the REAL-detection split above (ARM A 8/8,
  ARM B 7/8 flights with gt-consistent ENGAGE detections) is the no-cheat
  evidence that the camera terminal engaged on the true target, not the
  own-prop phantom. Multiple ARM-A sub-1 m intercepts are genuinely
  camera-tracked (real ENGAGE detections + miss < 1 m).
- **`audit_per_tick.py` caveat (not a dishonesty finding):** it reports FAIL on
  every coded-dash flight in BOTH arms because check (b) greps the phase literal
  `"DASH"` while coded-dash emits `"CODED_DASH"` (tool written for the S2
  `--handoff` pipeline). The historical canonical baseline
  `mc_coded_dash_qv2_line9_s123.csv` fails identically (12 fail / 4 skip) — a
  pre-existing tool-scope mismatch affecting all coded-dash runs equally, not a
  leak. Its (c) command-vs-lambda correlation bound (0.7) was calibrated on
  sustained S2 tracking and is advisory-quality for these short coded-dash
  engagements.

## Statistics / honest limits

n=8 paired per arm. The **recall lift is a strong, clear signal** (2/67 → 28/79
pooled; ARM B flights hit 67-100% band recall on runs 3/5/7 while ARM A is ~0%
on 6/8). The **miss regression is consistent** (7/8 paired flights worse) but
the per-flight miss spread is large (A 0.47-3.56 m, B 2.44-6.68 m) — call the
miss direction clear, the magnitude noisy at this n. This A/B **concludes** the
pointing mechanism (recall + centering) and **ranks** the miss tradeoff; a
larger n and the sweep below would tighten the miss number.

## Next (per the recipe's sweep + Phase B/C)

The lever confirms as a *pointing* fix but pays a closing-speed toll at 9 m/s.
Two follow-ups, in priority:
1. **Lighter cap / loft-only:** sweep `--dash-accel-cap ∈ {5.66 (θ30°), none}`
   with loft+wedge held — recover closing speed while keeping most of the
   centering (the analytical table says θ30° still frames the band). Find the
   (cap, loft, wedge) point that centers AND keeps handoff speed up.
2. **Phase B (hold-the-residual):** the framing win now gives the terminal
   real detections to work with; a look-angle-constrained PN + IBVS pixel term
   is the lever that turns "framed" into "hit" at 9 m/s
   (`docs/intercept_accuracy_levers.md`, `flight/fov_guidance.py`).

## Audit addendum (2026-07-22, head) — the "camera-tracked" claim is NOT confirmed

The `audit_per_tick.py` phase-name bug (check (a)/(b) greps `"DASH"`; coded-dash
emits `"CODED_DASH"`) is FIXED (DASH_PHASES = ("DASH","CODED_DASH")), so the
no-cheat audit now actually runs on these flights. Re-run on both arms:

- **Checks (a) + (b) PASS on ALL 16 flights.** (a) = zero post-handoff `ext_*`
  (no cue/gt leak) AND every non-detected ENGAGE tick held/hovered exactly; (b) =
  the coded dash ran before ENGAGE. **The honesty core is clean — no cheating.**
- **Check (c) — cmd-velocity-azimuth vs camera `lambda_deg` correlation ≥ 0.7 —
  is WEAK or FAILING on most flights** (ARM A 7/8 overall pass, run 4 fails (c);
  ARM B only 3/8 pass, 5/8 fail (c): corr 0.02–0.69). (c) is the check that says
  the command is actually STEERED BY THE CAMERA LOS, not coincidentally near it.

**Honest correction:** the earlier "multiple sub-1 m genuinely camera-tracked
intercepts" line OVER-CLAIMS. What is established: (1) real (gt-consistent)
detections rose sharply and ENGAGE fired on them, and (2) no honesty violation
(a/b clean). What is NOT established: that the miss was DRIVEN by the camera. (c)
is advisory-quality on these short engagements, but it does not support a
camera-guided claim — and per the project's anti-mirage method (ADR-0076 #18g/#18h,
which caught the identical "camera-guided r2l" mirage), the decisive test is a
**dash-only control arm** (camera terminal disabled): if it scores the same CPA,
the camera added nothing. That control was NOT run here. So: **pointing/recall
CONFIRMED; camera-DRIVEN intercept UNCONFIRMED — treat with mirage caution until a
dash-only control settles it.** This does not change the two headline results
(recall lift + miss regression); it scopes the "camera-tracked" wording honestly.

## Anti-mirage VERDICT (2026-07-24, head) — the dash-only controls settle it

The two dash-only control arms the addendum above asked for are now flown:
**Adash** (baseline + camera terminal disabled) and **Bdash** (lever + camera
disabled), both via `--coded-dash-acquire-range-min 999` so the acquire streak
never closes -> ENGAGE never fires -> the recorded `miss_m` is the pure
open-loop coded-dash CPA (these flights exit `clean=0` / `python_exit_1`; the
CPA is still valid). Paired by `run_idx` (seed-matched geometry) against the
camera-ON arms via `antimirage_verdict.py`. delta = miss(cam ON) − miss(dash-only):

| arm | dir | cam-ON med | dash-only med | paired Δmed | tighter/worse/same | verdict |
|-----|-----|-----------:|--------------:|------------:|:------------------:|---------|
| **A** baseline | l2r | 0.68 | 2.05 | **−1.18** | 4/0/0 | **CAMERA-DRIVEN** (not a mirage) |
| **A** baseline | r2l | 2.08 | 0.77 | +1.31 | 1/2/1 | MIXED (camera worse on 2/4 — aspect bias) |
| **B** lever | l2r | 2.53 | 1.76 | **+0.86** | 0/4/0 | **CAMERA STEERS WRONG** |
| **B** lever | r2l | 3.92 | 2.38 | **+1.54** | 0/4/0 | **CAMERA STEERS WRONG** |

**The two things this settles:**

1. **The camera terminal is REAL, not a mirage — but only when the dash delivers
   a fast approach.** On the flat baseline (Arm A) l2r, the camera terminal
   tightens the miss on 4/4 paired flights (median −1.18 m vs open-loop dash).
   That is a genuine camera-DRIVEN intercept — it REFUTES the pure-mirage worry
   for the baseline l2r and is a stronger result than the weak check-(c)
   correlation could show.

2. **The loft-dive + accel-cap LEVER breaks the camera terminal.** With the lever
   on (Arm B), the camera-ON miss is WORSE than dash-only on **8/8 paired flights**
   (l2r +0.86, r2l +1.54). So the Phase A "sub-1 m camera-tracked intercepts"
   wording for Arm B is **REFUTED**: the lever's dash ballistics alone (Bdash
   1.76/2.38 m) beat the lever's camera intercept (B 2.53/3.92 m) everywhere.
   The accel-cap slows the close so much that once the (now well-framed) target
   is engaged, the terminal converges to a WORSE CPA than just letting the dash
   fly through. The framing win (recall 3%→35%) is real; it does **not** convert
   to a terminal win at 9 m/s because the closing-speed toll dominates.

**Corrected headline for Phase A:** pointing/framing CONFIRMED (Arm A l2r proves
the camera terminal genuinely drives the intercept when closing speed is high);
the loft-dive+accel-cap lever is **NET-NEGATIVE at 9 m/s** — it centers the
target but its closing-speed cost makes the camera terminal worse than the dash
alone. r2l additionally carries the ADR-0056 aspect bias (camera hurts even at
baseline). **This directly motivates the next experiment: a LIGHTER accel-cap
(θ≈30°) / loft-only** that keeps some centering while recovering the closing
speed the terminal needs — see `docs/flight_plan_candidates.md`.

Reproduce the verdict:
```bash
scripts/experiments/loft_dive/run_arm.sh Adash --go   # baseline, camera OFF
scripts/experiments/loft_dive/run_arm.sh Bdash --go   # lever, camera OFF
.venv/bin/python scripts/experiments/loft_dive/antimirage_verdict.py \
    A=logs/mc_loftdive_armA_line9_s123.csv \
    Adash=logs/mc_loftdive_armAdash_line9_s123.csv \
    B=logs/mc_loftdive_armB_line9_s123.csv \
    Bdash=logs/mc_loftdive_armBdash_line9_s123.csv
```

### CORRECTION (2026-07-24, independent Opus 5 review) — the "closing-speed cost" attribution is CONFOUNDED

An independent analysis (`docs/flight_plan_candidates.md` + a validated ballistic
CPA model, `scripts/experiments/flight_plans/dash_cpa_model.py`, MAE 0.29 m vs the
dash-only arms) reproduced the 2×2 above and then found a confound I had under-weighted.
**What still stands, what was over-attributed:**

- **STANDS (within-arm deltas, aim error is common-mode):** baseline A l2r is
  camera-DRIVEN (−1.18 m, 4/4); and with the lever ON, camera-ON is worse than
  dash-only on 8/8 (B vs Bdash — both fly the same +30° bias, so the camera's
  effect is isolated cleanly).
- **CONFOUNDED / softened:** my headline "the lever is net-negative at 9 m/s
  *because the accel-cap's closing-speed cost dominates*." Two problems: **(1)** the
  aim constant is **mis-sized**. `flight.guidance.collision_lead_heading` solves a
  *constant-speed* intercept triangle at `--dash-speed 16`, but the coded dash
  ACCELERATES from rest (`dash_forward_speed`, guidance.py:112), so the flown +30°
  bias is wrong for the real trajectory — the model's optimal bias is ~64° at the
  cap's a≈3.57 and ~20° at the baseline's a≈10. Comparing B (capped, a≈3.57) to A
  (uncapped, a≈10) therefore conflates the cap's effect with a *different aim error*
  at each accel. **(2)** decomposing Phase A's measured +22.1° centring, the accel-cap
  contributed the LEAST (+5.0°; wedge +10.0°, loft +7.1°) while carrying all the cost
  (handoff speed 4.7→3.0 m/s + the aim confound). So "loft-only" (drop the cap) may
  keep most of the framing without the toll.
- **Why the camera hurts is now MEASURED (not inferred):** on Arm B's real-detection
  ENGAGE ticks, `lambda_deg` − gt LOS = **+9…+17° (l2r, 4/4)** and **−17…−21° (r2l,
  3/3)** — the ADR-0056 aspect bias, quantified. The terminal builds its velocity in
  the LOS frame, so that angular error rotates the whole command → ~0.8–1.7 m induced
  miss. **Consequence: Phase B (FOV-hold) is the WRONG next lever** — Arm B was already
  centred at −3.6° and still lost; the failure is bearing BIAS, not the target leaving
  frame. The right lever is aspect-bias compensation (`--terminal-bearing-bias-deg`,
  code-gated), not FOV-hold.

**Decomposition arms now pre-registered** (`docs/flight_plan_candidates.md`,
`scripts/experiments/flight_plans/run_arm.sh`), fly-first order: **G20dash** (bias 20,
dash-only, level) vs Adash isolates the aim-mis-sizing; **C/Cdash** (loft-only) isolates
the cap; **E/Edash** (bias 64) tests the model's cap-optimal aim. Adoption needs pooled
≥13/16 paired (6/8 is explicitly *not* significant). This CORRECTION is the anti-mirage
method working as designed: a committed attribution, an independent check, a fix before
it baked in.

## Reproduce

```bash
# smoke (1 flight) then the paired A/B (sequential, idle load):
scripts/experiments/loft_dive/run_arm.sh smokeB --go
scripts/experiments/loft_dive/run_arm.sh A --go      # baseline
scripts/experiments/loft_dive/run_arm.sh B --go      # lever (fits up10 shadow)
# parse the 5 numbers (ARM B carries the +10 deg physical wedge):
.venv/bin/python scripts/experiments/loft_dive/parse_ab.py \
    logs/mc_loftdive_armA_line9_s123.csv \
    logs/mc_loftdive_armB_line9_s123.csv=10
```
