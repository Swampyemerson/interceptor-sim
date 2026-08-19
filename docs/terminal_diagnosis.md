# Terminal-dropout root-cause diagnosis (fast-crosser miss forensics)

**Date:** 2026-07-06 · **Author:** diagnostic pass over existing logs only (no new Gazebo runs)
**Data:** all 41 realistic-cue flights referenced by `logs/mc_p6_{BASE,FUSE,FUSEWARM}_2026-07-06*.csv` (24) and
`logs/mc_realistic_{EMIT,DIFF}_2026-07-06*.csv` (17); per-tick forensics on each flight's `flight_csv_path`
CSV (columns per `scripts/m4_intercept.py` CSV_HEADER). All 41/41 end in
`lost detection for >1.0s inside terminal range (5.0 m)`. Miss mean 1.404 m / median 1.126 m; Pk@1 = 41%, Pk@2 = 76%.
Analysis scripts + per-flight table preserved in-repo at `scripts/forensics/`
(`terminal_forensics.py`, `deep_dive.py`, `quantify.py`, `q6_experiment.py`,
`per_flight_summary.csv`) — copied 2026-07-06 from the ephemeral job dir
`~/.claude/jobs/28aff4e9/tmp/` so this linchpin diagnosis stays
reproducible from the repo alone.
Clock note: the CSV `t` column is WALL time; RTF (sim/wall, measured per flight from d(gt_tag_y)/dt vs the
commanded 6 m/s) averaged **0.483** (0.296–0.514). All durations below are **sim seconds** unless marked wall.

## Verdict (one line)

**The miss is kinematic/guidance-limited, not terminal-perception-limited: 96% of the final-miss variance is
already determined at the moment the camera-only terminal begins (ZEM@handoff vs miss, r² = 0.957, n = 41), and
the celebrated "camera dropout at CPA" starts only 0.074 s (mean) before closest approach and adds −0.031 m
(i.e., nothing) to the miss.** The dropout is the *signature of the flythrough*, not the cause of the miss.

## The forensic chain (each number traces to the per-flight table)

1. **Detection does NOT stop early.** Last real detection event: mean **0.074 s** before CPA (median 0.053,
   max 0.272), at ground-truth range mean **1.69 m** (median 1.38, min 0.49). The detector tracks the tag to
   well inside 2 m on a 6 m/s crosser.
2. **Perception channel is healthy until the geometry ends it.** Tick census over the last 2 s of ENGAGE
   (775 ticks, 41 flights): **81.8% in-FOV + detected, 4.6% in-FOV + undetected, 13.5% out-of-FOV** (the
   out-of-FOV ticks are concentrated in the last ~0.1–0.2 s). Tag apparent size at the last detection:
   mean **219 px** (min 69 px — far above the ~8 px/quad_decimate=2 floor). Board incidence at loss: median
   **19.5°** off face-normal (max 74°). Terminal inter-detection gaps (gt < 5 m, n = 346): median **0.026 s**,
   p90 0.052 s, max 0.161 s. There is no blur model in the Gazebo camera at all (ADR-0014 disclosure), so a
   blur channel cannot exist in this data.
3. **The ">1 s dropout" is post-CPA bookkeeping.** The hold clock (`TERMINAL_HOLD_MAX_S = 1.0`,
   `m4_intercept.py:1914`) runs on WALL time; the blind gap opens on average 0.074 s sim (≈0.15 s wall) before
   CPA, so **≥85% of the counted ">1 s" outage is the outbound flythrough**, when the tag is geometrically
   behind the vehicle and can never come back. It is one single terminal gap, not accumulated short ones.
4. **FOV-escape is real but irrelevant to the miss.** 36/41 flights have the target exit the ±49.85° horizontal
   FOV — but at mean **0.037 s** before CPA (max 0.124 s), at mean range 1.59 m. True LOS rate at the last
   detection: mean **485°/s**, peaking 600–1870°/s in the blind window, vs achieved yaw rate mean-max **44°/s**
   (absolute max seen 124°/s — so the airframe is *not* hard-capped at PX4's ~45°/s default; the deficit is
   ~10:1 regardless). No pointing system survives the endgame of a 1+ m-miss flythrough; this is the geometry
   of any nonzero miss, upstream of any camera property.
5. **The miss is already locked before the camera phase can matter.** Zero-effort-miss (ZEM: the miss if both
   vehicles flew ballistically from t; computed 3-D from gt positions + sim-time-differenced velocities):
   mean **1.69 m at the handoff latch** (r² = 0.957 vs final miss), **1.47 m at the freeze latch** (range
   2.97 m, 0.20 s to go; r² = 0.990), **1.44 m at last detection** vs final miss 1.40 m. Blind-window accrual:
   **−0.031 m**. Best flight (DIFF2#4): ZEM 0.31 m at handoff → miss 0.38 m with detection to 0.49 m range.
   Worst (DIFF2#5): ZEM 3.8 m at handoff → miss 3.37 m with perfect terminal tracking to 3.5 m.
6. **The kinematic floor (shown derivation).** The camera-only terminal lasts median **0.41 s** (0.25–0.62):
   first detection median 7.6 m, handoff latch median 6.2 m, |v_rel| ≈ 12.1 m/s at freeze. Measured achieved
   acceleration (max |ΔV|/Δt over 0.25 s windows in ENGAGE): median **8.7 m/s²** (p90 13.5; MPC cap 12).
   Correction capacity ≈ ½·a·t_go²: **½·8.7·0.41² = 0.72 m from handoff**; ½·8.7·0.20² = **0.17 m from the
   freeze latch**. Realized correction during ENGAGE: mean **0.28 m** (filter settle + freeze eat the rest).
   Census: all 11/41 flights with ZEM@handoff ≤ 1.0 m hit sub-meter (mean 0.69, max 0.98); all 12/41 with
   ZEM@handoff > 2.0 m missed by mean 2.47 m. **Perfect perception cannot fix a flight whose delivered
   ZEM exceeds ~0.7 m + reclaimable mechanization losses (~0.4 m).**
7. **Controlled point-mass check (Q6, `q6_experiment.py`, patched copy of guidance_lab in job tmp, 150 paired
   seeds, S2 config, 6 m/s).** EXPECTED realism: miss 1.003 m, tag-lost-in-terminal 96%. Same cue but a
   PERFECT terminal camera (exact state, no FOV, no dropout, no noise): dropout **96% → 0%**, coverage 1.00,
   miss **0.755 m** — improves only 25%, does not collapse. Perfect cue AND perfect camera: **0.916 m** —
   a 91% kinematic residual (lab caveat: BEST-tier cue is position-only, so this arm lacks velocity emission;
   and the lab is documented optimistic vs Gazebo — used here only for the contrast, which agrees with the logs).

## Ranked root-cause breakdown (share of the ~1.4 m mean miss)

| # | Channel | Share of miss | Evidence |
|---|---------|---------------|----------|
| 1 | **Kinematic: mid-course-delivered ZEM > terminal correction capacity** (0.41 s window × 8.7 m/s² = 0.72 m vs 1.69 m delivered) | **~70%** (locked-in remainder) | items 5–6; r² = 0.957 at handoff; lab all-perfect residual 91% |
| 2 | **Guidance mechanization losses inside the window** (freeze latch discards the last 0.20 s = 0.17 m; α-β settle: realized 0.28 m of an ideal 0.72 m) | **~20–25%** (recoverable ≈ 0.3–0.45 m) | item 6; ADR-0014 lever 2 (split-freeze) targets exactly this; lab perfect-camera Δ −0.25 m is its upper bound |
| 3 | **FOV-escape at CPA** (LOS rate 500–1800°/s vs yaw ≤ 124°/s) | **~2%** (−0.03 m blind accrual) | items 1, 4, 5 — causes the *dropout signature*, not the miss |
| 4 | **Blur / scale / decimation** | **~0%** | item 2: 219 px tag, 19.5° incidence, no blur model exists |
| 5 | **Detection cadence** | **~0%** | item 2: 22–43 events/sim-s terminal (RTF-inflated; ~14–20 Hz wall — at RTF=1 the blind gap ≈ doubles, worth ≤ ½·12·0.15² ≈ 0.14 m, verdict unchanged) |

## So the fix must target…

**Time-to-go and delivered geometry — not the camera's hold through CPA.**
1. **Acquisition range (the one perception lever that matters).** Capacity scales with t_go²: acquiring at
   12 m instead of 6.5 m raises correction capacity 0.72 → ~4.3 m, above the worst delivered ZEM (4.0 m).
   For the real seeker this is *detect range*, not resolution/FOV/deblur at CPA.
2. **Mid-course track quality** (sets ZEM@handoff): velocity-emitting cue already the proven #1 lever
   (ADR-0015); fusion's small −0.08 m (ADR-0018) is this channel; dash lead-law quality belongs here too.
3. **Reclaim the mechanization losses** (~0.3–0.45 m): ADR-0014's split-freeze / later freeze + warm-settled
   filters — bounded, worthwhile, second-order.
Explicitly de-prioritized by this data: yaw-rate authority, camera resolution/FOV, motion-deblur, higher frame
rate, nested tags for hold-through-CPA. The prior "bottleneck = terminal blind window" reading (ADR-0014
addendum, MEMORY) is **corrected** by this analysis: the blind window is the symptom, the handoff ZEM is the disease.

## Known limits of this analysis

- Pitch/roll are not logged; the pixel/FOV projection uses yaw only (vertical margin is large — elevation
  angles are single digits at these ranges — but a hard per-tick vertical-edge check is not possible from these logs).
- Detector latency is not logged; inferred ≈1 frame (β_true − β_meas = 15.4° mean at last detection at ~450°/s).
- The wall-clock hold (`TERMINAL_HOLD_MAX_S`) and RTF 0.30–0.51 mean sim-time semantics of the 1 s hold vary
  per flight; all conclusions above use per-flight-measured RTF.

## ADR-lite

- **Context:** 41/41 realistic-cue fast-crosser flights end "lost detection >1 s at CPA", miss 1.4 m mean;
  team about to invest in one of three solution lanes; needed the mechanism, not the symptom.
- **Finding:** per-tick forensics (ZEM, true-bearing/FOV projection, cadence, incidence) show the miss is
  96%-determined at the handoff latch (r²=0.957); detection persists to 1.7 m / 0.07 s-to-CPA; blind-window
  contribution −0.03 m; FOV-exit happens 0.04 s pre-CPA as a *consequence* of the miss (LOS rate 500–1800°/s).
  Point-mass check: perfect terminal camera removes 100% of dropout, only 25% of miss.
- **Decision:** treat the failure as a time-to-go/geometry problem. Invest in (1) longer-range acquisition,
  (2) mid-course track quality (delivered ZEM), (3) split-freeze mechanization reclaim — in that order.
  Do NOT invest in terminal camera hold (yaw rate, FOV, resolution, deblur, cadence).
- **Date:** 2026-07-06. Evidence: this file; scripts + per-flight table in job 28aff4e9 tmp.
