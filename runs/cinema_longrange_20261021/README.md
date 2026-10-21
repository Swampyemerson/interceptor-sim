# Long-range cinema run — 2026-10-21

**Deliverable: `intercept_500m_36ms.mp4`** (31.7 s, 1080p30). Gazebo
camera-in-the-loop intercept from a 500 m slant cue, re-rendered in Blender
with the technical HUD. The flown take is **hr5**: approach_dash at
vmax 36 m/s (80 mph class, plant peak 35.8), brake a=4, injected cue error,
**CPA 0.087 m, 64 camera decodes**, first detection at GO+17.5 s after a
17 s belief-only dash.

Pipeline + knobs: `scripts/cinema/README.md` (fly → export → Blender →
HUD). Long-range configs: `scripts/cinema/config/{scene,shots}_longrange.json`.
Feature + isim study: `isim/specs/approach_dash_2026-10-21.md`.

## Takes (logs/cinema_lr/, grep CINEMA_RESULT)

| take | envelope | CPA (m) | n_det | note |
|---|---|---|---|---|
| lr1 | 20 m/s, a=3 | 0.272 | 39 | registered pick |
| lr2 | 20 m/s, a=3 | 0.199 | 41 | registered pick |
| lr3 | 20 m/s, a=3 | 7.821 | 7 | cue draw opened range past the decode cliff; SAFE abort |
| hr1 | 36 m/s, a=4 | 0.380 | 42 | |
| hr2 | 36 m/s, a=4 | 0.697 | 45 | |
| hr3 | 36 m/s, a=4 | 0.453 | 40 | |
| hr4 | 36 m/s, a=4 | 0.298 | 63 | |
| **hr5** | 36 m/s, a=4 | **0.087** | 64 | **the video take** |

All numbers are SIM. The real FC keeps stock speed limits until the airframe
is measured (builder_queue `dash-envelope-on-real-fc`). The isim sweep
artifacts (dash_sweep*.json, curve_*.csv) and SESSION_STATE.md (the build
log) live alongside this file.
