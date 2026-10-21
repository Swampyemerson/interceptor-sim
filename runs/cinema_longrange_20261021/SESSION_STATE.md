# LONG-RANGE VIDEO — session resume state (written 2026-10-21, pre-compaction)

Builder directive: long-range demo video — cue at 500 m, dash phase "absolutely
ripping" (80 mph asked), decelerating into predicted tag capture.

**UPDATE (later the same session): 80 mph IS flyable.** The 20 m/s "max" is GCS
metadata only — no firmware clamp (scripts/cinema/set_px4_params.py does a raw
set with read-back; 36.0 verified, plant delivered 35.8 m/s). The video take is
**hr5: vmax 36 / a_brake 4, CPA 0.087 m, 64 detections** (logs/cinema_lr/hr5*,
exported hr5_run.json). hr5 timings: go=24.64, first_det=go+17.49, cpa=go+21.06,
peak 35.8 m/s @go+6.7. a_brake MUST be 4 at 36 m/s (isim cliff: 36/3 = 0/8).
Everything below through "IN FLIGHT" is DONE and COMMITTED (77eb94c..06a05c3:
flight code + takes + spec addendum + contract + FC-envelope builder decision;
Pi synced to 06a05c3, 58/58 terminal tests pass ON THE PI; real FC read-only
check: stock 12/5/45/8, nothing written).

## DONE this session (all committed up to f77e707; long-range work UNCOMMITTED yet)

1. **Feature built: `approach_dash`** (flight/pursuit_terminal.py, default OFF,
   byte-identity-pinned): Phase A norm cap widens to `dash_v_max_ms`; the Phase-A
   stopping-distance brake cap applies UNCONDITIONALLY while on (sheds speed on
   sqrt(v_min² + 2·a·d) into the believed rendezvous); Phase-A slew budget =
   `dash_accel_max_ms2` (8). Phase B untouched. CLI: `--pursuit-approach-dash
   --pursuit-dash-vmax 20 --pursuit-dash-accel 8 [--pursuit-dash-brake-accel]`
   (flight/deploy/real_flight.py build_terminal, prints APPROACH DASH ON line).
   Tests: 5 new in flight/tests/test_pursuit_terminal.py (57 passed).

2. **isim study (registered: isim/specs/approach_dash_2026-10-21.md, RESULT
   filled)**: every a≤4 cell 8/8 lock, median miss 0.014–0.067 m. Cliff found:
   vmax36/a=3 at far crossing = 0/8 (do NOT fly). **PICK: vmax 20, a=3 (flying
   brake-package constant), geometry cross 150 / lead 477 = exactly 500 m slant
   cue** → interceptor flies ~345 m own path, peak 19.9 m/s, arrives at first
   decode at ~6.6 m/s (visible 20→7 braking), t_cpa ~27 s, 6/6 lock, miss 0.045.
   Evidence: runs/cinema_longrange_20261021/dash_sweep{,_far}.json + curve_*.csv.
   Sweep scripts in scratchpad (dash_sweep.py, geom_screen.py — regenerate if needed).

3. **Gazebo plumbing**: scripts/cinema/set_px4_params.py (pymavlink on
   udp:14550, sets m4's envelope MPC_XY_VEL_MAX 20 / ACC_HOR 12 / TILT 60 /
   JERK 30 with fresh-request READ-BACK; exit≠0 on any miss). fly_takes.sh now
   runs it between boot and the driver (skips take on FAIL). worlds/cinema.sdf
   ground VISUAL enlarged 500→1300 m (collision untouched; comment in file).

4. **Renderer updates**: hud_compose.py has a FFWD badge (shot speed >1 blue
   badge, <1 keeps REPLAY). blender_scene.py parameterized: ground_size_m,
   scatter_trees.area, bushes.area (defaults = old short-range behavior).
   New scene config: scripts/cinema/config/scene_longrange.json (corridor
   (0,0)→(-160,-150); band north y 25..95; scatter south y<-195; ground 1500).

## IN FLIGHT right now

- Background task: 3 Gazebo takes `lr1/lr2/lr3` via
  `bash scripts/cinema/fly_takes.sh logs/cinema_lr logs/cinema_lr/takes.txt`
  (cross 150, lead 477, speed 12, window 45, cue error ±3 m / ±1.5°,
  --pursuit-approach-dash --pursuit-dash-vmax 20). Results land in
  logs/cinema_lr/run_lr*.log (grep CINEMA_RESULT) + per-take CSVs/frames/meta.

## NEXT STEPS (in order — rewritten after hr5)

1. DONE — takes flown (lr1 0.272 / lr2 0.199 / lr3 7.821 abort; hr1-hr5 at 36 m/s,
   best hr5 0.087 m). Shot list shots_longrange.json written + tuned to hr5;
   framing stills round 2 (decel horizon fix, merge_lock lower camera) in flight.
3. Render (detached setsid, ~1000 frames ≈ 2 h; still-check framing first with
   `--still "decel@cpa-6;launch_rip@go+2" --preview`): blender45 at
   ~/tools/blender45/blender, scene_longrange.json. Then hud_compose → mp4
   (expect speed showing 20 →≈7 → re-overtake ~14 double-hump; PiP real feed).
4. Deliver mp4 (SendUserFile), copy to runs/cinema_longrange_20261021/,
   update contract (plain_log + sim_harness changelog + now; CAPS: where≤300,
   finding≤300, next≤100 chars each; plain_log max 12 → overflow to
   docs/state_archive/plain_log.md), render_mbse → render_dashboard → artifact
   republish (read-gate ALREADY satisfied this session; receipt must differ) →
   **render_sysml.py --png LAST (CHROME_BIN="/mnt/c/Program Files/Google/
   Chrome/Application/chrome.exe") because editor.html embeds the contract
   digest** → run_tests.sh ALL GREEN → commit specific paths + push (remote
   moved once already this session; pull --rebase, regenerate SysML on conflict).

## Key facts to not re-learn

- In-sim decode cliff ≈12 m (0.5 m tag, fx 540). Tail-chase-from-near opens
  range → aborts; geometries that shrink range lock continuously.
- isim ranks, Gazebo decides (isim decode model is more generous).
- Artifact URL eb5e40d1-…; publish via render_dashboard.py --artifact <tmp>;
  the read-gate full chunked read was done this session (v1792359379-3067).
- Short-range video shipped: runs/cinema_20261021/intercept_12ms.mp4 (t6,
  CPA 0.194 m); pipeline knobs in scripts/cinema/README.md.
- Background tasks reaped ~51 min → long renders via setsid + until-loop waiters.
- Suite must be run at the END (SysML digest ordering); machine not idle during
  renders is OK for the offline suite but it slowed one run.
