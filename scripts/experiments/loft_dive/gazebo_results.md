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
