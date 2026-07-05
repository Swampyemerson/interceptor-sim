# NEXT — top of the stack

## Current: M4.5 realism upgrade (ADR-0010 sequencing) — S1 done-ish, S2 is the enabler

### Where S1 (FPV speed) landed (2026-07-05)
- FPV profile built behind `--fpv` (PX4 param bump via MAVSDK pre-arm, two-speed
  closing law, rescaled terminal ranges, N=5). M4 gate untouched (opt-in flag).
- **Pure pro-nav (N=5) intercepts a 3 m/s crosser at 0.94 m** (clean, <1 m) from a
  hover start. 4 m/s ~1.6 m; 6 m/s uncatchable from hover.
- **PIP (predicted intercept point) was ported and validated in Gazebo — it does
  NOT transfer** (3.0 m at 4 m/s vs pure-PN 1.6 m). Noisy/intermittent monocular
  track gives PIP a bad velocity estimate; pure PN (LOS-rate only) is more robust.
  Kept as `--law pip` for the writeup as a documented negative result. See
  ADR-0011 + addendum.
- **KEY FINDING: hover-start is kinematically speed-limited.** The full FPV target
  band (6-10 m/s) needs S2's external-cue DASH (running start). S1 and S2 are
  coupled (council seats B/C called this). Do NOT keep grinding S1 vs faster
  targets from hover — build the dash.

### Next: S2 — two-stage sensor handoff (the enabler)
External-cue mock (subscription-free process, degraded GT: sigma~0.5 m, ~100 ms
latency, 10 Hz) → interceptor DASHES on the cue (12 m/s) to a running start →
HANDOFF to camera-only terminal (throttled ~5-6 m/s) once the tag is acquired
within ~8 m. This gives the interceptor the closing speed to catch a fast
crosser AND is the comms-denied headline. Then S3 (maneuver paths), S4 (proof),
M5. Full plan: ADR-0010. Gate scripts: check_s1.sh exists; extend for S2.

### M5 (still the finish line — protect it, ADR-0010)
Gate (GOALS.md): Monte-Carlo batch over target speeds/paths, matplotlib
trajectory + miss-distance plots, README with architecture diagram + results +
one GUI demo GIF.

Likely steps (refine before building):
- [ ] Batch runner: N runs × {law × target speed (2.0/2.5/3.0) × path variant
      (crossing y-offsets, maybe a receding case)} reusing check_m4.sh's
      boot-per-flight pattern (or one boot + re-place tag + re-takeoff per run if
      state contamination is manageable — drone lands displaced; probably NOT, see
      the fresh-sim-per-flight lesson). Runs are ~3 min each wall — budget hours,
      run in background, aggregate CSVs.
- [ ] Stats + plots: trajectory overlays (pursuit vs pro-nav, same path), miss
      histogram/CDF per law, maybe miss vs target speed. matplotlib, saved to
      docs/ or plots/. Numbers traced to run stamps (GOALS.md).
- [ ] README: mission, architecture diagram (camera → detector → filters →
      guidance → PX4; mover + ground-truth split), results tables, reproduce
      instructions (check_m0..m4 + batch), demo GIF (GUI + gz camera view?).
- [ ] Demo GIF: screen-record a GUI pursuit-vs-pronav pair (Emerson's standing
      request to watch is also the demo content). Tools: peek/ffmpeg x11grab?
- [ ] Consider small robustness items first: pro-nav miss variance across runs
      was 0.28-0.44 m (fine vs 1.0 gate); RTF-load sensitivity documented.

## Done
- **M4 (2026-07-05):** moving-target intercept, pursuit vs pro-nav — GATE
  PASSING, verifier-confirmed (3 independent gate runs): pro-nav miss
  0.402/0.277/0.443 m (bar <1 m) vs pursuit 2.544/2.109/2.048 m, identical
  2.0 m/s crossing paths, camera-only + no-cheat numerically verified.
  Mechanization: strapdown λ=ψ+β, alpha-beta filters (λ rate gain 0.30),
  a=N·Vc·λ̇ (N=4) integrated into a world-frame lateral velocity, NED velocity
  + absolute-yaw setpoints, constant 3.0 m/s closing, terminal coast at 2.0 m,
  camera-only breakoff. THE debugging saga (5 dev runs + 2 failed gates → root
  cause: RTF ~0.5 under load + wall-clock mover = target effectively 2× speed;
  fixed by sim-time scheduling) is in ADR-0009 + two addenda — read before
  touching anything timing-related. quad_decimate=2 for M4 detector only.
- **M3 (2026-07-05):** static intercept — camera-only P-control (body-frame
  velocity + yawspeed, ADR-0008) closed 4.9 m → 2 m standoff in ~10 s, held.
  Final standoff error 0.018/0.035 m across two verifier runs (bar <0.5 m),
  detection coverage 1.000, zero overshoot. Gate `scripts/check_m3.sh`;
  guidance script `scripts/m3_static_intercept.py` (reuses m2_detect + m0_takeoff
  by import). Verifier numerically proved commands trace to measured range, not
  ground truth. Key integration facts: offboard needs a setpoint streamed BEFORE
  `offboard.start()` and ≥2 Hz after; detection runs on its own thread (latest-
  frame-wins, no queue) at full 20 Hz control coverage; `sleep 5` after boot
  before MAVSDK connect (check_m3.sh) since M3 arms right after health-OK.
- **M0 (2026-07-04):** toolchain — PX4 v1.17.0 built, Gazebo Harmonic 8.14, venv up.
- **M1 (2026-07-04):** camera pipeline — 10/10 frames @ 1280×960 via gz-transport13.
- **M2 (2026-07-04):** AprilTag detection — custom `worlds/apriltag.sdf` +
  `models/apriltag_target/` (tag36h11 id0, 0.625 m plane / 0.5 m black square, tag
  center at (5, 0, 0.5) facing -X); `scripts/m2_detect.py` gate: detection_rate
  1.000, mean err_norm 0.0861 m (bar ≤0.25 m), mean range 4.888 m
  (`logs/m2_detect_20260704T233941Z.csv`). Two real bugs found and fixed along the
  way — see ADR-0006 (ground-truth chain: camera_link composes directly against
  the model, NOT via base_link — a numeric coincidence made the wrong chain look
  right) and ADR-0007 (tag material needed `emissive_map`, not just a lit
  `albedo_map`, to stay legible at range under the world's fixed sun angle).

Key facts for a fresh session:
- PX4 at `~/PX4-Autopilot` (v1.17.0). Launch camera drone on the M2 world:
  `PX4_GZ_WORLD=apriltag GZ_SIM_RESOURCE_PATH=~/interceptor-sim/models HEADLESS=1 make px4_sitl gz_x500_mono_cam`
  (plain `gz_x500_mono_cam` with no world env var still boots the "default" world
  from M0/M1). `worlds/apriltag.sdf` must be symlinked into
  `~/PX4-Autopilot/Tools/simulation/gz/worlds/` — `scripts/check_m2.sh` creates/
  repairs that symlink automatically (see ADR-0005 for why it can't just be an
  env var).
- Camera: 1280×960 @ 30 Hz, hfov 1.74 rad, RGB_INT8. On the M2 world, topics are
  under `/world/apriltag/...` (world name matters for gz-transport topic paths —
  rediscover via `gz topic -l` if world/model names change). Intrinsics measured
  and recorded in `camera_intrinsics.json`: fx=fy≈539.936 px, cx=640, cy=480,
  matching the (fx≈(1280/2)/tan(hfov/2)) cross-check almost exactly.
- Ground truth for M2 (and a template for M3+) comes from `/world/apriltag/pose/info`
  (gz.msgs.Pose_V) — see scripts/m2_detect.py's docstring and ADR-0006 for the
  transform-chain gotcha (camera_link's pose is relative to the model directly,
  NOT to base_link, despite a numeric coincidence suggesting otherwise).
- venv sees system gz bindings via `.venv/.../site-packages/gz_system.pth`
  (python3-gz-transport13 + python3-gz-msgs10 from apt). No scipy — quaternion ->
  rotation matrix is hand-rolled in scripts/m2_detect.py (`quat_to_matrix`) to
  keep the dependency surface minimal (GOALS.md), reused rather than
  re-implemented for M3+ frame math.
- Boot-complete grep: "Startup script returned successfully" (ADR-0004 — never wait
  on "Ready for takeoff!" pre-MAVSDK).
- MAVSDK: `udpin://0.0.0.0:14540`. AprilTag lib: pupil-apriltags (ADR-0003).
- Minor: m0_takeoff.py duplicates its final CSV row — tidy if reused as template.
