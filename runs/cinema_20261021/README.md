# Cinema demo takes — 2026-10-21 (12 m/s camera-only pursuit, Gazebo)

DEMO runs for the builder's video directive (NOT a registered gate; the
standing gate evidence is docs/xcheck_gazebo_pursuit_prereg*.md). Flown by
scripts/cinema/cinema_run.py: real flight code (`--terminal pursuit`,
ADR-0118 flying config + brake), live Gazebo camera, cue error injected
belief-only. Target: fpv_target quad, tag body-bolted, mover
--orient-to-velocity (realistic yaw/pitch attitude), straight 12 m/s track.

| take | geometry (lead/cross) | cue error | CPA | n_det | note |
|---|---|---|---|---|---|
| take1 | -3/4 (tail-chase) | 2.0 m/5° | ABORT (lost 2 s) | 10 | range OPENED past the ~12 m in-sim decode cliff — geometry class rejected |
| take2 | 22/6.5 | 3.0 m/6° | 0.700 m | 56 | first full engagement |
| t3 | 22/4 | 2.5 m/5° | 0.387 m | 51 | |
| t4 | 22/4 | -3.0 m/-6° | 0.301 m | 48 | inside contact radius |
| t5 | 25/6.5 | 2.0 m/4° | 0.237 m | 73 | inside |
| t6 | 22/4 | 2.5 m/5° | **0.194 m** | 53 | **the video take**: gap-free lock from 7.7 m (+3.7 s) to CPA (+8.0 s) |

t6 full evidence here (per-tick csv, rf csv, 6-DoF pose log fused into
t6_run.json, meta). Seeker JPEG frames (118 MB) are local-only:
logs/cinema/t6_frames/ (regenerable by re-flying).
