# Cinema pipeline — fly a demo run in Gazebo, re-render it beautifully

Built 2026-10-21 (builder directive: "beautiful 12 m/s run video").
Everything is config-driven so a new video is a parameter change, not a rebuild.

## The five stages

```
1. FLY      scripts/cinema/cinema_run.py      (camera-in-the-loop Gazebo flight,
            worlds/cinema.sdf                  rich logging: 6-DoF poses @50 Hz,
                                               every seeker frame as JPEG, per-tick
                                               detections, meta JSON)
2. EXPORT   scripts/cinema/export_run.py      logs -> one <take>_run.json
                                               (uniform 60 Hz timeline + events +
                                               offline re-detected tag boxes)
3. RENDER   scripts/cinema/blender_scene.py   Blender 4.5 Cycles/CUDA (~7 s/frame
            config/scene_field.json            at 1080p/96 samples on the 4070):
            config/shots_default.json          builds the field/forest set, the two
                                               drones (real tag texture), animates
                                               from run.json, renders the shot list
4. HUD      scripts/cinema/hud_compose.py     technical overlay (phase, guidance
                                               source, speeds, closing rate, range,
                                               REAL seeker feed PiP with detection
                                               corners) + end card + ffmpeg mp4
5. (batch)  scripts/cinema/fly_takes.sh       serial multi-take flier (fresh sim
                                               boot per take; batch hygiene)
```

## One full run, end to end

```bash
# 1. fly (boots its own world; one sim at a time, idle machine)
bash scripts/sim_kill.sh && bash scripts/demo_boot_sim.sh /tmp/sim.log cinema
.venv/bin/python scripts/cinema/cinema_run.py --out logs/cinema/t7.csv \
    --lead 22 --cross-range 4 --speed 12 \
    --cue-offset-n 2.5 --cue-vel-rot-deg 5 --window-s 30
# 2. export
.venv/bin/python scripts/cinema/export_run.py logs/cinema/t7
# 3. render (detach it: ~1-1.5 h for ~570 frames)
~/tools/blender45/blender -b --factory-startup -P scripts/cinema/blender_scene.py -- \
    --run logs/cinema/t7_run.json --scene scripts/cinema/config/scene_field.json \
    --shots scripts/cinema/config/shots_default.json --out renders/t7
# 4. composite + encode
.venv/bin/python scripts/cinema/hud_compose.py --renders renders/t7 \
    --run logs/cinema/t7_run.json --out renders/t7/final --name intercept_12ms
```

Fast iteration: `--still "lock@go+6;chase_wide@go+4" --preview` renders single
frames (~45 s each incl. one ~2 min scene build) to judge framing before
committing to a full render.

## The knobs the builder most likely wants

| what | where |
|---|---|
| target speed / geometry (lead, cross-range) | `cinema_run.py` flags (isim-screen a new geometry first — see below) |
| cue error (m, deg, speed frac) | `--cue-offset-e/n --cue-vel-rot-deg --cue-speed-err` |
| target attitude realism / weave | `--orient-max-pitch-deg`, `--path weave` (weave needs start x > ~2.5 m: the mover's ADR-0010 X-floor guard) |
| environment (trees, sun, haze, exposure) | `config/scene_field.json` |
| shots: cuts, cameras, lenses, slow-mo | `config/shots_default.json` (t0/t1 are event-relative: `go-3`, `first_det+1`, `cpa-0.9`; `speed: 0.2` = 5x slow-mo) |
| HUD title/subtitle, freeze length | `hud_compose.py` flags |
| render cost | `scene_field.json: render.samples` (96 ≈ 7 s/frame), `res`, `fps` |

## Hard-won facts (don't re-learn)

* **In-sim decode cliff ≈ 12 m** for the 0.5 m tag at fx 540 (ADR-0124's
  ~22 px sim cliff). A geometry that OPENS range past ~13 m after GO loses
  lock and aborts (take 1 died this way; isim's decode model is more generous
  — Gazebo decides). Crossing/overtake geometries that only shrink range are
  the ones that work.
* The cue error is **belief-only** (`--target-start/--target-vel` handed to
  real_flight); the mover flies truth. Perfect cue = zero offsets.
* `worlds/cinema.sdf` names the target entity `fpv_target` so the mover's
  `--orient-to-velocity` guard (which protects the FLAT apriltag board)
  permits realistic attitude streaming; the tag is body-bolted and cants
  with it — the detector earns decodes under exactly what the render shows.
* Blender cube `size=1` scaled by `s` has EDGE `s` (not `2s`) — placard/board
  sizing bug class.
* AgX + Nishita needs exposure ≈ **-2.8**, and the sky needs a camera-ray
  gain (in `build_environment`) or the horizon renders muddy at that exposure.
* Blender scene build is ~2 min (ops-based primitives); render ≈ 7 s/frame
  full quality on the 4070 (CUDA). Renders > 50 min: detach with `setsid`
  (background-task reaper, see memory).
* Takes flown 2026-10-21 (12 m/s, cue err 2.5-3 m / 5-6°): CPA 0.700 (take2,
  cross 6.5), 0.387 / 0.301 / 0.237 / **0.194 m** (t6, the video take) —
  5/6 engaged cleanly, 4 inside the 0.35 m contact radius. DEMO runs, not a
  registered gate; scope in each take's `_meta.json`.

## Honesty scope (what the video claims and doesn't)

The flight is the real `flight/` code (pursuit terminal, ADR-0118 config +
brake) against the live Gazebo camera; guidance sees camera pixels +
own-state EKF only. GIVENS: cue with the configured error, latched at GO
(better-than-perfect-cue honesty but still a given), windless, sim camera,
0.5 m sim tag (the "real cliff is 14-18 px" caveat, ADR-0124, applies to the
Pi, not here). The 3-D render is a VISUALIZATION of the logged poses; the
PiP seeker view is the actual recorded sim camera feed with offline
re-detections of the same detector. Numbers on the end card trace to the
take's logs (`*_meta.json`, `*_run.json`).
