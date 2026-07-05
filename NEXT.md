# NEXT — top of the stack

## Current: M4 — moving-target intercept, pursuit vs pro-nav (BUILT, GATE NOT YET RUN)

**State (2026-07-05, session ended on usage limit): pro-nav WORKS.** Last dev run:
miss 0.945 m (< 1 m gate), clean=1, coverage 0.764, breakoff="lost detection
inside terminal range" (expected endgame), `logs/m4_intercept_pronav_20260705T013540Z.csv`.
Design was council-decided (ADR-0009); mechanization + all tuning below is
run-validated through 5 dev iterations. `--bench` sign test PASSED (mean |λ̇|
1.24°/s while spinning at ±20°/s — the λ=ψ+β strapdown compensation is right).

**Remaining to close M4 (in order):**
- [ ] (Optional, cheap insight) one pursuit dev run: `scripts/sim_gui.sh 6.5 -4 0.5`
      then `.venv/bin/python scripts/m4_intercept.py --law pursuit` — expect
      trailing/tail-chase and a larger miss.
- [ ] Official gate: `scripts/check_m4.sh` (boots fresh headless sim per law,
      runs pursuit then pronav, prints comparison; exit 0 iff both clean AND
      pronav < 1.0 m). ~8 min wall.
- [ ] `verifier` subagent on the gate (it must repeat M3's numeric no-cheat
      divergence check on λ̇/Vc inputs — ADR-0009 council requirement).
- [ ] Append run-validated verification numbers to ADR-0009; PROGRESS.md M4 row;
      commit "M4: ... (gate passing)". Then M5 (Monte-Carlo + plots + README).

**Dev-run debugging trail (why the tuning is what it is — full details in git
diff of scripts/m4_intercept.py comments):**
1. Acquire never completed: 10-consecutive-fresh-per-TICK impossible — detection
   produces ~14 Hz vs 20 Hz loop. Fix: streak counts the DETECTION stream.
2. Engaged mid-yaw-slew (β ok for 1 tick but ψ̇≈+16°/s) → started in a hole →
   FOV loss. Fix: ACQUIRE_CENTERED_STREAK=6 (~0.4 s settled) before engage.
3. λ̇ filter lagged (est -3 vs true -16°/s) making yaw feedforward useless →
   FOV loss at 2.5 m/s target. Fixes: yawspeed = λ̇_ff + 3.0·β (was pure P 1.5);
   BETA_GAIN_LAMBDA 0.15→0.30; Vc floor 0.3→1.5 (lead built at 1/5 strength from
   hover); V_PERP_MAX 2→3; target 2.5→2.0 m/s (still ≥2 gate; council member A's
   criterion: yaw pinned at cap = undiagnostic geometry, so open it).
4. Estimator skew note: β content is ~0.1-0.2 s older than ψ → phantom λ̇ during
   hard yaw accel (est -34 vs true -21°/s once). Benign at 2.0 m/s geometry;
   candidate M5 refinement: timestamp-matched ψ or bound |ff|.
5. Sim infra: after several rapid GUI reboots the gz renderer can wedge (camera
   topic silent, GPU idle, PX4 fine) — sim_gui.sh now hard-fails on a 10 s
   camera-topic liveness probe. ALWAYS fresh sim per dev flight (drone lands
   displaced/rotated; reusing the instance breaks the geometry).

Key facts for a fresh session:
- Fly anything: `scripts/sim_gui.sh [x y z]` (GUI, kills stale sims, verifies
  camera, optionally pre-places tag). NEVER inline `pkill -f "gz sim"` etc. —
  it self-matches your own shell (exit 144, three times); use scripts/sim_kill.sh.
- gz-transport Python quirk: a process with ANY subscription never receives
  service RESPONSES (requests still apply). Hence m4_target_mover.py is its own
  subscription-free process (50 Hz set_pose streaming, ~1 ms median).
- Frames (PX4 gz): NED north = gz +Y, east = gz +X; ψ (attitude_euler yaw_deg,
  CW+) ⇒ vehicle facing gz +X reads ψ≈+90°; λ=ψ+β validated by --bench.
- Emerson wants to WATCH milestone flights in the GUI (standing request) —
  demo the pursuit-vs-pronav pair in sim_gui.sh when M4 closes.
- Sonnet agents may use max thinking (user OK'd): council=max, verifier=xhigh
  set in .claude/agents/. sonnet-worker default; bump per-task if hard.

## Done
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
