# Pre-registered: detector requirement envelope for the pursuit terminal (NN seeker spec)

2026-10-21, registered BEFORE running. Head-inline (the opus55 worker brief
bounced — docs/subagent_safeguard_log.md 2026-10-21 entry).

## Question

What must a future markerless detector deliver — (a) first-detection range,
(b) update rate, (c) pipeline latency, (d) per-frame detection probability —
for the FLYING pursuit terminal (flight/pursuit_terminal.py via isim
`terminal="pursuit"`) to keep working? Output = the spec the NN/accelerator
choice must meet (feeds the Pi-CPU vs Hailo decision).

## Method

isim Monte-Carlo (`isim.mc.run_many`), canonical crossing geometry
(cross 6.5 m / lead 16.2 m), `tag_facing="camera"` (the detector sees the
AIRFRAME, not a tag face — incidence pinned 0 is the right abstraction),
`scatter=Scatter()` honest run-to-run noise, target 9 and 12 m/s, n=8
seeds/cell. The detector abstraction maps onto the TRUE-seeker knobs via the
new `camera_overrides`/`decode_overrides` Scenario fields (byte-identical
default; isim tests 197 passed after the change):

- RANGE R50: `size50_px = 0.30·fx/R50` (fx=385, target span 0.30 m),
  `size_k_px = size50/5` (default shape ratio), `min_side_px = 0.5·size50`
  (hard floor ≈ 2×R50). Detection probability is a logistic range gate.
- RATE: `fps` ∈ axis. LATENCY: `latency_s` ∈ axis. DETECT-PROB: `p_max`
  pinned (overrides applied AFTER scatter's p_max draw).

Anchor (held while one axis sweeps): R50=16 m, 24 fps, 60 ms, p_max=0.9.

Axes: R50 {6,8,12,16,24,32,48 m}; fps {5,10,15,24,30,60}; latency
{20,60,120,200 ms}; p_max {0.5,0.7,0.9}; plus a range×rate cross
R50 {8,12,16} × fps {5,10,24}.

## Pre-registered criterion

Cell PASSES iff `med_miss_m ≤ 0.35` AND `pct_engage ≥ 87.5` (≥7/8 reached
ENGAGE on real detections). The frontier = cheapest passing value per axis.

## What a null means

- If even R50=48 / 60 fps cells fail → the TERMINAL, not the detector, is
  binding in this geometry; the detector spec cannot be read off this sweep.
- If everything passes down to R50=6 / 5 fps → the terminal is
  detector-insensitive at this geometry and the real requirement comes from
  the cue/dash geometry (acquisition), not the terminal — rerun at a far
  (approach_dash) geometry before specifying hardware.

## Honesty grade

Model-derived (isim's decode surface is a calibrated-but-unmeasured model;
`sim-decode-range-transfers` is an open assumption). The output is a
REQUIREMENT ranking, not a transfer claim: Gazebo/bench re-earns any number
that becomes a purchase criterion.
