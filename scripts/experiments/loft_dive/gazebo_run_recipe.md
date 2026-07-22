# Loft-then-dive + accel-cap — Gazebo confirmation recipe (Phase A)

*Companion to `scripts/experiments/loft_dive/inframe_ab.py` (the analytical in-frame
A/B) and `docs/intercept_accuracy_levers.md` (Phase A — POINTING). This note is the
EXACT command + config to confirm the analytical prediction in Gazebo. **Do NOT run
this blind or in parallel — the sim serializes (one at a time, idle load only); the
head coordinates it.** No heavy MC in this doc — a small paired-seed A/B first.*

## What the analytical A/B predicts (the thing Gazebo must confirm or break)

Pure-geometry upper bound over the canonical line-9 m/s crossing
(`logs/mc_coded_dash_qv2_line9_s123.csv`), across the 8–12 m acquisition band
(`.venv/bin/python scripts/experiments/loft_dive/inframe_ab.py`):

- **Baseline today** (flat dash, no wedge, ~40° nose-down): target pinned at
  **vert_med +39°** — the frame-TOP edge (half-VFOV ±41.6°). Geometrically "in frame"
  but at the extreme corner where the center-biased detector fails → the measured
  **~0.8 % in-flight** recall.
- **Accel-cap is the single cleanest lever:** capping the dash to **θ≈20°**
  (`--dash-accel-cap 3.57`) alone drops the band to **vert_med +19° (100 % centered)**
  with **no** maneuver and **no** terminal dump.
- **Fixed-wedge trap:** a wedge big enough (≥20–30°) to center the accel-phase band
  **dumps the <5 m terminal target out the frame BOTTOM** (the quad brakes nose-UP
  there) — "a fixed tilt only relocates the window" (ADR-0076 #18j-fix). Keep the
  wedge **modest** (~10–15°).
- **Loft-dive is the complement:** `--dash-loft-m 2–3` shaves a further ~5–8° of LOS
  depression per 2 m of loft; over-loft → out the bottom. Costs peak descent
  ~1.8–2.8 m/s (needs a raised `--dash-vvert-max`).

**Recommended first Gazebo arm (co-sized):** `θ≈20°` accel-cap + `loft 2 m` +
`wedge 10°`. Predicted band **100 % centered, vert_med ≈ +4°**, <5 m held.

## Config the Gazebo run MUST exercise (the current sim flies FLAT at 0.5 m)

The stock coded dash holds `ALT_REF_M = 0.5 m` for the whole flight (fixed altitude,
`V_VERT_MAX = 0.5 m/s`). To exercise real vertical motion the new flags do three
things — all default-OFF (byte-identical when unset):

1. **`--dash-loft-m H`** — raises the *takeoff* altitude to `0.5 + H` (the quad
   pre-climbs BEFORE the dash; the ~2 s dash is far too short to also climb at
   0.5 m/s), then the CODED_DASH phase DIVES back to 0.5 m over `--dash-loft-dive-s`.
2. **`--dash-vvert-max V`** — REQUIRED with a loft: raises the CODED_DASH vertical
   clamp so the dive (H over the dive window) actually fits. Use **~2–3 m/s**
   (H=2 m over 2.5 s ⇒ ≥1.6 m/s peak; use 3.0 for margin).
3. **`--dash-accel-cap A`** — ramps the commanded dash speed at `A` m/s² instead of
   stepping it, holding body pitch ≈ `arctan(A/g)`. For a HARD physical cap also
   lower PX4 `MPC_ACC_HOR_MAX` to `A` in the same run (see caveat below).

`--cam-mount-up-deg W` is the existing fixed wedge; size it WITH the loft/cap.

> **PX4 accel caveat:** `--dash-accel-cap` only shapes the *commanded* velocity.
> Under `--fpv`, `MPC_ACC_HOR_MAX` is set to 12 (or 20 with `--accel-boost`), so PX4
> can still pitch hard to *track* the ramp early. For a clean pitch cap, ALSO set
> `MPC_ACC_HOR_MAX ≈ A` (edit `FPV["PX4_PARAMS"]` or add a one-off param push). The
> first A/B can skip this to isolate the ramp effect, then add it if the achieved
> pitch (from the ulog `vehicle_attitude`) still exceeds `arctan(A/g)`.

## The exact A/B (paired seeds, byte-identical geometry)

Reproduce the CANONICAL line-9 geometry and master-seed (methodology memory
`reproduce-canonical-gate-geometry`): same `--path line --x0 6.5 --y0-mag 15.343
--speeds 9.0`, same `--master-seed`, `--seeker markerless`. Run arms **sequentially**
(one sim at a time), idle load. Validate any fix on **disjoint** seeds afterward.

```bash
# --- ARM A: BASELINE (flat, today) — the control ---
MC_SEEKER=markerless scripts/mc_batch.sh \
    --mode m4 --path line --x0 6.5 --y0-mag 15.343 --speeds 9.0 \
    --directions both -n 8 --master-seed 123 \
    --extra-args "--coded-dash --fpv --dash-unclamp --dash-speed 16 --dash-crossing-bias-deg 30"

# --- ARM B: LOFT-DIVE + ACCEL-CAP + MODEST WEDGE (the lever) ---
MC_SEEKER=markerless scripts/mc_batch.sh \
    --mode m4 --path line --x0 6.5 --y0-mag 15.343 --speeds 9.0 \
    --directions both -n 8 --master-seed 123 \
    --extra-args "--coded-dash --fpv --dash-unclamp --dash-speed 16 --dash-crossing-bias-deg 30 \
                  --dash-accel-cap 3.57 --dash-loft-m 2 --dash-loft-dive-s 2.5 \
                  --dash-vvert-max 3.0 --cam-mount-up-deg 10"
```

Same `--master-seed 123` ⇒ identical per-flight target geometry across the two arms
(paired). `--directions both` gives l2r + r2l (the asymmetry the crossing-bias
addresses). Each arm writes `logs/mc_*.csv` + per-flight `m4_intercept_*.csv`.

## What to measure (the decisive numbers)

1. **In-flight detection recall in the 8–12 m band** — THE headline. From each
   flight's per-tick CSV, count ticks whose gt slant-range ∈ [8, 12] m during the
   dash, and the fraction with a REAL (gt-consistent, not phantom) detection.
   Baseline is ~0.8 %; the analytical upper bound says centering should lift it
   toward the ~100 % static rate. Reuse the attitude/geometry machinery in
   `scripts/experiments/dash_pitch_probe.py` (it aligns the ulog `vehicle_attitude`
   to the flight CSV and computes the target's in-camera elevation) and the
   real-vs-phantom split from the gt-consistent audit (`scripts/audit_per_tick.py`).
2. **Achieved body pitch** (ulog `vehicle_attitude`) through the 8–12 m band —
   confirm the accel-cap actually held θ ≈ arctan(A/g) (≈20° for A=3.57). If pitch
   still spikes past that, add the `MPC_ACC_HOR_MAX` cap (caveat above).
3. **Did the dive hold the target through <5 m** (no out-the-bottom) — the
   fixed-wedge trap check. Confirm the near-band target stayed in frame.
4. **CPA / miss + Pk@2.5 m** — the downstream payoff. Expect the camera terminal to
   ENGAGE on REAL detections (vs the baseline's phantom-driven or dash-ballistic
   ENGAGE). A camera-tracked <2.5 m intercept of the 3D quad would be the first in
   the dataset (none exists today — ADR-0076 add #18h).
5. **Closing-speed cost of the cap** — log the achieved dash speed at handoff; the
   ramped/capped dash reaches less than 16 m/s in ~2 s, so t_go/ZEM will differ.
   This is the documented tradeoff, not a bug.

## Honesty + hygiene gates (repeat every run)

- **Honesty boundary:** guidance reads camera + own-state EKF only. `--dash-loft-m`,
  `--dash-accel-cap`, the ramp, and the dive are all OWN-STATE trajectory shaping
  (no camera, no `gt_*`). `gt_*` is used ONLY to SCORE recall/CPA, never in the loop.
  Any new camera-guided intercept re-earns the numeric no-cheat audit.
- **Sim-clock, not wall:** the ramp/dive use `sim_clock.t` (not `time.monotonic()`),
  so RTF sag can't distort them. Verify RTF and run at idle load.
- **One sim at a time**, batch arms sequential; never `pkill -f` a literal pattern
  inline (use `scripts/sim_kill.sh`).
- **Statistics before verdicts:** n≥8 paired, mechanism evidence (the recall +
  pitch numbers above), honest "not significant at this n" language. Lab/analytics
  RANK; only this Gazebo A/B turns the ranking into a conclusion.

## Sweep after the first A/B lands (if the lever confirms)

Vary one axis at a time, paired seeds, sequential: `--dash-accel-cap ∈ {2.63, 3.57,
5.66}` (θ 15/20/30°), `--dash-loft-m ∈ {0, 2, 3}`, `--cam-mount-up-deg ∈ {0, 10, 15}`
— to find the joint (cap, loft, wedge) operating point the analytical table flags as
centered in the band AND held through <5 m, at the least closing-speed cost.
