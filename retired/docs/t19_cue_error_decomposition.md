# T19 cue-error decomposition — what "0.99 m median" actually measures

**Task #43 (sim-free half), audit findings P-H2 + P-M1. 2026-07-10. Analysis only — no sim runs; all numbers recomputed from existing logged CSVs (sources named per number).**

## The metric being decomposed

ADR-0052's T19 close-out quotes "cue error **0.99 m median / 2.06 m max**" (verifier re-fly: 1.03 / 2.03). That number is `check_t19.sh` assertion 4: the **3D distance between the delivered cue position and the live target's ground-truth position at the tick the cue corrected the tracker** (`ext_fresh=1` rows; world-frame mapping `cue_world_x=ext_y, cue_world_y=ext_x`), from the flight CSV. It is therefore **not** triangulation accuracy — it conflates three terms:

| Term | What it is | Measured value (median) | Source |
|---|---|---|---|
| (a) Triangulation / geometry | stereo solve vs the capture's own baked gt, latency-free by construction | **0.587 m** (max 2.054; conf-gated cache) — almost entirely cross-track: \|cross\| 0.586 m, \|along\| 0.051 m, vert ~0.001 m | `logs/ground_station_20260709T053228Z.csv` + `053704Z.csv` (`x,y,z` vs `gt_target_*`, which station.py passes through from `logs/rig_captures/full_sweep_20260709T015530Z/index.csv`) |
| (b) Cue latency (staleness at use) | cue reports where the target was `ext_age_s` ago; at 9 m/s that is an along-track lag | `ext_age_s` **0.152 s** at use (0.124–0.208 observed) → predicted lag **1.37 m** (0.99–1.92) | flight CSVs (below), `ext_age_s` column × dash speed 9.0 m/s (`index.csv` `dash_speed`) |
| (c) Station/mover clock-epoch skew | the mover's motion timeline starts LATE relative to the station's epoch-rebase t0, so the cue timeline **leads** the live target | **+0.100 s** median (cue ahead ≈ 0.9 m), per-flight medians +0.088 / +0.141 / +0.088 / +0.100 s — **uncontrolled, varies ~50 ms run-to-run** | direct method: audit `frame_t_sim` vs live time the flight-CSV `gt_tag_y` reaches that frame's baked position |

**The skew is real and canceling, exactly as P-H2 suspected.** Two independent estimates agree: the direct frame-time method gives +0.100 s (cue leads), and the lag-residual method (measured along-track lag ÷ speed, minus `ext_age_s`) gives −0.096 s (cue less stale than its age claims). Net measured along-track error at use is only **−0.52 to −0.66 m** (cue behind target) ≈ 9.0 × (0.152 − 0.100) s — i.e. **~0.9 m of latency lag is silently cancelled by the epoch skew**.

**Mechanism (measured):** both processes spawn at CUE_WAIT entry, but the mover's first commanded motion lands **+0.08 to +0.18 s after** the station samples its rebase epoch t0 (station t0 recovered as `frame_t_sim − t_sim_nominal`; per flight: t0=23.724/23.536/23.964/23.968 s vs first-moving gt tick +0.100..+0.152 / +0.124..+0.176 / +0.084..+0.136 / +0.096..+0.148 s later). Process-spawn latency plus the mover's 20 Hz teleport hold — luck of scheduling, not design.

## Honest restatement

> **"0.99 m" = triangulation cross-track error (~0.6 m) ⊕ residual latency lag (~0.55 m) after an accidental ~0.9 m epoch-skew cancellation.** It is a *delivered-cue-at-use* error under a fortuitous timing alignment, not triangulation accuracy.

Counterfactual (same flights, skew removed, full `speed × ext_age_s` lag restored, measured cross/vert kept): the identical metric reads **median 1.59–1.64 m** (pooled 1.60 m, n=59). Quoting triangulation accuracy honestly: **0.59 m median / 2.05 m max** (conf-gated, from the station audit). Quoting delivered-cue accuracy honestly: **~1.6 m median at 9 m/s with a 0.12 s latency floor**, of which today's flights show 0.99–1.03 m only because the epoch skew happens to cancel ~⅔ of the lag.

## Reproduction (per-flight recompute matches ADR-0052 exactly)

| Flight CSV (`logs/`) | Audit CSV (`logs/`) | Conflated median/max (recomputed) | ADR-0052 quote |
|---|---|---|---|
| `m4_intercept_pronav_20260709T053214Z.csv` (headline) | `ground_station_20260709T053228Z.csv` | 0.985 / 2.064 m | "0.99 / 2.06" |
| `m4_intercept_pronav_20260709T053650Z.csv` (verifier) | `ground_station_20260709T053704Z.csv` | 1.030 / 2.031 m | "1.03 / 2.03" |
| `m4_intercept_pronav_20260709T052100Z.csv` (pre-conf-gate) | `ground_station_20260709T052114Z.csv` | 1.016 / 43.623 m | "1.0 median, 43 max" |
| `m4_intercept_pronav_20260709T052352Z.csv` (pre-conf-gate) | `ground_station_20260709T052406Z.csv` | 0.967 / 43.520 m | (same seq=2 outlier) |

Method: for each `ext_fresh=1` row, error vector `e = (ext_y−gt_tag_x, ext_x−gt_tag_y, ext_z−gt_tag_z)` (world frame); project `e` onto the live target-velocity direction (central-difference of `gt_tag_*` over ±0.25 s) → along-track vs cross-track; `age_eff = −e_along/speed`; skew = `age_eff − ext_age_s`. Direct skew: for each audit `emit` row, interpolate the flight CSV's `gt_tag_y(t_sim)` to find when the live target occupied the frame's baked `gt_target_y`, minus the frame's claimed `frame_t_sim`. Triangulation-only: audit `x,y,z` vs `gt_target_*` (both capture-timeline quantities — latency-free pairing; along/cross split against the capture dash direction `(0,−1,0)`).

## What existing logs CANNOT separate (the sim-gated other half of #43)

- **Attribution of the ~0.10 s skew** among mover process-spawn latency, its clock-helper startup, and the 20 Hz teleport hold: the mover CSV (`logs/m4_mover_*.csv`) logs only *relative* `t` (its own `sim_t0` subtracted, never logged). Needed: log the mover's absolute sim `t0` (and ideally accept a shared `--epoch-t0` so station + mover rebase to ONE trigger epoch instead of two independently-sampled ones).
- **Controlling (not just measuring) the skew**: the `--epoch-t0` shared co-start fix in `m4_intercept.py`/`m4_target_mover.py`/`station.py` — sim-gated, not done here.
- Per-sample skew at cue-use ticks with <±0.05 s resolution: both gt sampling (flight 20 Hz tick) and mover teleports (20 Hz) quantize the live trajectory; medians over 25 frames are robust, single samples are not.

*Analysis script (session scratchpad, logic fully specified above): pooled over the 4 epoch-fixed flights — conflated 1.030 m median (n=59); triangulation 0.587 m median / p95 2.054 m (n=102); latency-predicted lag 1.37 m median; measured along-track −0.516 m median; skew −0.096 s (residual) / +0.100 s (direct); no-skew counterfactual 1.603 m median.*
