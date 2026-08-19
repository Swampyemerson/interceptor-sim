# KICKOFF PROMPT — paste everything below the line into Claude Code

> **⚠️ HISTORICAL ARTIFACT (M0 kickoff, 2026-07-04).** This was the project's
> original from-scratch kickoff; it predates the GPU note (an RTX 4070 IS
> available) and M0–M5 are long complete (project is in Phase 2, ADR-0065+).
> Retained verbatim as the record of how the project started — do NOT reuse as
> a session prompt; a fresh session starts from `CLAUDE.md` + `docs/next.md`'s
> CURRENT block instead.

> This is the message to paste into the open Claude Code window (running on Fable 5,
> started inside this project folder). `CLAUDE.md` and `docs/goals.md` are already in the
> folder and auto-load — they carry the mission, the model/council rules, and the
> conventions, so this prompt stays short. Just paste and send.

---

You are the orchestrator for this project, running on **Claude Fable 5**. Two files
are already in this repo and auto-loaded: read **`docs/goals.md`** (what we're building
and why) and **`CLAUDE.md`** (how we work — model orchestration, the decision/council
protocol, and conventions) before doing anything else. First, run `/model` and
confirm you're on Fable 5.

**Your autonomy contract (this is the main change from a normal kickoff — I want low
involvement):**
- **Proceed autonomously through the milestones below.** Do NOT stop for my approval
  at each step. Post a short plan for the current milestone for my visibility, then
  execute it.
- **Delegate the volume work to Sonnet 5 subagents** (`sonnet-worker` for build/code/run,
  `verifier` for gate checks) to conserve my budget. Keep architecture, integration,
  and final decisions on yourself (Fable).
- **Decisions:** make educated calls and log them as ADR-lite entries in
  `docs/decisions.md`. For one-way doors (library choice, guidance law/gain, frame
  conventions, big architecture forks), convene the **Sonnet 5 council** (2-3
  `council-member` subagents in parallel), synthesize, decide, and log it — per `CLAUDE.md`.
- **Only pause and ask me** if you are genuinely blocked, a gate fails twice in a row,
  a step needs a >2 GB download or a system change beyond this project + apt, or a
  decision is both irreversible and truly ambiguous. Otherwise keep going.
- **Checkpoint as you go:** `git commit` at every milestone; keep `CLAUDE.md`,
  `docs/next.md`, and `docs/progress.md` current so a fresh session resumes cleanly.
- **Teach me briefly as you work** (what/why/where, 1-3 sentences per non-trivial
  decision) — I'm new to simulation and guidance theory. See `CLAUDE.md`.

**Environment:** Fresh Ubuntu VM (confirm with `uname -a` and `lsb_release -a` first).
Assume **nothing** is installed beyond git, Python, and Claude Code. You own installing
and verifying the whole toolchain. **No GPU** — Gazebo runs **headless** for all
automated runs (GUI only for a final demo capture, and it will be software-rendered).

**Required stack (do not substitute without a council decision):**
- PX4 SITL (latest stable) + **Gazebo Harmonic**, installed via PX4's standard
  `Tools/setup/ubuntu.sh`.
- The **`gz_x500_mono_cam`** airframe (x500 quad + forward monocular camera) as the interceptor.
- **MAVSDK-Python** for offboard control — **explicitly no ROS 2.** Pull the camera
  stream into Python via the **`gz-transport` Python bindings** (`python3-gz-transport*`),
  not a ROS bridge.
- **OpenCV** + a maintained **AprilTag** library — evaluate `pupil-apriltags` vs
  alternatives (`dt-apriltags`, `cv2.aruco`), pick one, and justify briefly in an ADR.
- Python 3.11+, a virtualenv, and pytest.

**Working practices:** scripted pass/fail gate at every milestone (hand it to
`verifier`; never claim done without showing the check's output); `git init` now and
commit at each gate with a `.gitignore` that excludes PX4/Gazebo build artifacts and
logs; headless by default with `HEADLESS=1` and `PX4_SIM_SPEED_FACTOR` where physics
allows; log telemetry / detections / miss-distance to `logs/` as CSV/ulog and analyze
from the logs; on breakage, read the real error and fix root cause (check PX4/Gazebo
docs + GitHub issues) rather than stacking workarounds.

**Milestones (in order — persist these into `docs/next.md`/`phases/` as you go):**
- **M0 — Toolchain.** Install PX4-Autopilot, Gazebo Harmonic, and the Python env.
  *Gate:* headless `make px4_sitl gz_x500` boots, and a MAVSDK script arms, takes off
  to 2 m, and lands — check script exits 0.
- **M1 — Camera pipeline.** Launch `gz_x500_mono_cam`; bridge the camera into Python
  via gz-transport; save frames. *Gate:* script captures N frames at the expected resolution.
- **M2 — AprilTag detection.** Place a static AprilTag model in a custom Gazebo world;
  detect it in the live feed; compute bearing / relative position from tag pose +
  camera intrinsics. *Gate:* detection rate + pose error logged against sim ground truth.
- **M3 — Static intercept.** Offboard velocity control closes on the static tag and
  holds a 2 m standoff. *Gate:* final standoff error < 0.5 m, logged.
- **M4 — Moving target.** Add a second vehicle / scripted model carrying the tag on
  simple paths. Implement pursuit guidance, then proportional navigation; compare miss
  distances. *Gate:* intercept (< 1 m closest approach) on a straight-line target at >= 2 m/s.
- **M5 — Portfolio polish.** Monte-Carlo batch across target speeds/paths; matplotlib
  trajectory + miss-distance plots; a proper README with architecture diagram, results
  section, and one demo GIF from a GUI run.

**Start now:** confirm you're on Fable 5, confirm the environment, post your brief M0
plan, then begin executing M0 autonomously — delegating to `sonnet-worker` and calling
the council if a real fork appears. Don't wait on me.
