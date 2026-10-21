# Findings — detector requirement envelope (vs spec.md, run as registered)

58 cells, n=8 seeds each, results.json. Criterion: med_miss ≤ 0.35 m AND
≥7/8 reached ENGAGE.

## The frontier

| Axis (others at anchor) | Passes | Fails | Reading |
|---|---|---|---|
| Latency | 20, 60 ms | **120 ms (0.65 m @9, 4.0 m @12), 200 ms (3.8/5.9 m)** | **THE BINDING AXIS.** Failures are huge, monotone, and worse at 12 m/s — unambiguous at n=8. |
| Update rate | 10–60 Hz (and 5 Hz @9 m/s) | 5 Hz only in the worst cross cell (r50 8 m, 12 m/s: 0.42 m) | Weakly binding: ≥10 Hz is safe, 5 Hz is marginal only when range is also minimal. |
| Detection range R50 | 8 m and up | 6 m (both speeds, ~0.36–0.38 m — just over the bar) | The TERMINAL needs only ~8 m. One 24 m cell @9 m/s read 0.41 m vs 0.29 at 16/32/48 — treated as n=8 noise (run-to-run σ ≈ 0.1–0.2 m here), not a real non-monotonicity. |
| Detect probability p_max | 0.5, 0.7, 0.9 all pass | — | Not binding; the KF rides through dropped frames. |

## What this means (the spec)

1. **Pipeline latency ≤ ~60 ms glass-to-guidance, hard.** 120 ms breaks the
   terminal at 12 m/s outright. This is the decisive requirement for the
   CPU-vs-accelerator choice: a Pi 5 CPU nano-YOLO is 67–88 ms for INFERENCE
   ALONE (before capture/pre/post), i.e. the CPU-only path sits at or past
   the cliff; an NPU at ~10–20 ms end-to-end sits comfortably inside it.
2. **Update rate ≥ 10–15 Hz** (24+ Hz costs nothing and adds margin).
3. **Terminal detection range ≥ ~8–12 m** only — the terminal is nearly
   range-insensitive beyond that (pre-registered null branch: the REAL range
   requirement comes from the acquisition/cue geometry, not the terminal;
   the far-cue dash (hr5) worked with first detection at ~12 m under a
   2.5 m cue error — larger cue errors scale the needed basket).
4. **Per-frame detection probability ≥ 0.5 suffices** at these rates.

## Honesty

Model-derived (isim decode surface; `sim-decode-range-transfers` open).
Latency conclusion is robust (effect sizes 2–17× the bar); range/rate
single-cell verdicts near the bar are within run-to-run noise at n=8 and
are quoted as ranges, not points. No Gazebo cross-check yet; the latency
cliff should be re-earned in Gazebo before it gates a purchase — but the
DIRECTION (latency dominates, range does not, for the terminal) is the
architecture-relevant finding.

## AMENDMENT (2026-10-22 — Gazebo re-earn, runs/gz_latency_20261022)

The paired Gazebo arm CONFIRMED the 120 ms cliff (6/6 degrade, 5.5×) but
REFUTED this sweep's "60 ms passes" line: Gazebo shows 60 ms already costs
2.3× (median 0.55 m — outside the 0.35 m ram radius; 4/6 pairs degrade).
The spec line "latency ≤ ~60 ms" above is superseded: **budget ≲30 ms for
full performance; 60 ms = degraded; 120 ms = broken.** isim's seeker
latency surface under-penalizes mid-latency — re-fit before quoting isim
on latency margins (ledger: isim-latency-surface-optimistic).

## SECOND AMENDMENT (2026-10-22 late — the LC arms close the question)

The "budget ≲30 ms" line above is ALSO superseded. Compensated arms
(KF age assumption set to the true injected latency): 60 ms → 1.19x,
120 ms → 1.05x — FULL recovery. The latency axis is an AGE-KNOWLEDGE axis:
latency ≤120 ms is free when the KF knows it; what binds is (1) knowing it
(calibration or stamps), (2) jitter, (3) rate ≥10-15 Hz, (4) range. Final
table: runs/gz_latency_20261022/spec.md. CPU-only still fails — on RATE
(6.5 Hz at 154 ms) and jitter, not latency.
