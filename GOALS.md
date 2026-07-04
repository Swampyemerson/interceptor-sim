# Interceptor Simulation — Goals & Context

> Read this file **fully** before planning anything. It tells you *what* we are
> building and *why*. It is self-contained: you do not need the parent project's
> files to start, though a pointer to them is at the bottom for deep context.

## Mission (one line)

Build a **simulation-only** counter-UAS interceptor: a quadcopter that uses its
own **monocular camera** to visually detect an **AprilTag** on a target drone and
autonomously **intercepts** it, first against a static target, then a moving one —
demonstrating **proportional-navigation guidance** with reproducible, logged results.

This is a **portfolio piece for aerospace internship applications.** Code quality,
documentation, and reproducible numbers matter as much as "it flew." Every claim
should be backed by a logged result you can rerun.

## Where this comes from (the parent project)

This sim is the software slice of a larger personal build: a **3D-printed 2.5"
brushless counter-UAS interceptor**, positioned as an Anduril / "Lattice-style"
portfolio project — *cheap, sensor-agnostic interceptors cued by smarter sensors,
with an onboard terminal seeker for jam-resistance.* The parent project spans
hardware, ground sensing, and flight; **this simulation proves the guidance and
vision core of that concept before any hardware exists.**

The single most important idea we are validating: **when the datalink is denied,
the interceptor's own camera locks the target and finishes the intercept.** In the
real design that terminal seeker is a Pi-class camera running classical CV; **in
the sim we replace that seeker's target-lock with an AprilTag** — a clean, robust
fiducial that lets us focus on the *guidance and control* problem instead of the
*perception* problem. Same architecture, one honest simplification.

## Scope — what this sim IS and IS NOT

**IS:**
- PX4 SITL + Gazebo Harmonic, the `gz_x500_mono_cam` airframe (x500 quad + forward mono camera).
- Monocular AprilTag detection -> bearing / relative position from tag pose + camera intrinsics.
- Offboard control via **MAVSDK-Python** (velocity, then attitude setpoints).
- Guidance: pursuit baseline, then **proportional navigation**; compare miss distances.
- Everything **headless** and **logged** (telemetry + detections + miss distance to CSV/ulog).

**IS NOT (deliberately out of scope):**
- No hardware, no real flight. Simulation only.
- **No ROS 2.** Camera comes out of Gazebo via `gz-transport` Python bindings; control via MAVSDK. Keep the dependency surface minimal.
- No ML perception. The AprilTag is the target lock; classical/fiducial only.
- No ground stereo rig, no ExpressLRS, no fusion node. Those are parent-project concerns, mocked away here.

## Parent -> Simulation translation

| Parent (real system) | This simulation | Why |
|---|---|---|
| Onboard terminal seeker (Pi + cam, classical CV) | `gz_x500_mono_cam` monocular camera + AprilTag | Isolate guidance/control from perception; robust ground truth for detection |
| Ground stereo triangulation + EKF track + pro-nav mid-course | Interceptor's own camera -> relative position -> pro-nav | Collapse to a single, self-contained terminal-intercept problem |
| PX4 OFFBOARD via MAVLink-over-ExpressLRS + Pi-over-UART | PX4 OFFBOARD via **MAVSDK-Python** over local UDP | One clean interface, no radio / ROS complexity |
| Hostile UAS being intercepted | Second drone / scripted model carrying the AprilTag | The threat to intercept |
| Comms-denied terminal handoff (the jam-resistance story) | Onboard-camera-only intercept, no ground-truth "cheating" | This is the headline capability the whole project exists to prove |

## The guidance arc (this is the resume line)

Work up the guidance ladder and **measure the difference** at each rung:
1. **Static intercept** — close on a stationary tag, hold a standoff. Proves the perception->control loop.
2. **Pursuit guidance** — steer straight at the current target position. Simple, laggy against motion.
3. **Proportional navigation (pro-nav)** — command acceleration proportional to line-of-sight rotation rate. The classic missile-guidance law; far better against a moving target.
4. **Monte-Carlo** — sweep target speeds/paths, plot trajectories and miss-distance statistics.

Target resume claim to make true and defensible:
> *"Implemented proportional-navigation guidance for autonomous visual intercept in
> PX4/Gazebo SITL; validated via Monte-Carlo miss-distance analysis."*

## Success criteria (mirror the milestone gates)

- **M0** headless `make px4_sitl gz_x500` boots; a MAVSDK script arms, takes off to 2 m, lands. Scripted check exits 0.
- **M1** camera frames captured from `gz_x500_mono_cam` at expected resolution.
- **M2** AprilTag detected in the live feed; detection rate + pose error logged vs. sim ground truth.
- **M3** static intercept holds a 2 m standoff; final standoff error **< 0.5 m**, logged.
- **M4** moving-target intercept: **< 1 m** closest approach on a straight-line target at **>= 2 m/s**; pursuit vs. pro-nav miss distances compared.
- **M5** Monte-Carlo batch, matplotlib trajectory + miss-distance plots, README with architecture diagram, results, and one GUI demo GIF.

## Engineering conventions carried over from the parent project

These are not optional polish — they *are* the portfolio:
- **Reproducibility first.** Headless runs, `PX4_SIM_SPEED_FACTOR` where physics allows, every run writes telemetry / detections / miss-distance to `logs/`. Analyze from logs, not by eyeballing a GUI.
- **Decisions are logged.** Non-trivial choices go in `docs/decisions.md` as short ADR-lite entries (context / options / decision / why). Especially: the AprilTag library choice, the pro-nav gain, coordinate-frame conventions.
- **Numbers trace to something.** Quantitative claims (FoV, standoff, miss distance, detection rate) come from a logged run or a written derivation, not vibes.
- **Teach as you go.** The builder is new to simulation and to guidance theory. For each non-trivial step give a tight *what / why / where-it's-documented* (1-3 sentences), and name new concepts (proportional navigation, offboard control, camera intrinsics, tag pose, EKF) with a one-paragraph primer on first use.
- **Milestone gates are scripted.** Each milestone ends in a pass/fail check (pytest or a script that exits 0/1). Never mark a milestone done without running the check and showing the output.

## Coordinate frames (agree on these once, up front)

- **World:** ENU, origin at the interceptor's start (or world origin).
- **Camera:** OpenCV convention — z forward, x right, y down.
- **Interceptor body:** FRD (forward, right, down). Convert to ENU / NED for setpoints.

Getting these consistent early prevents a whole class of sign-error bugs in the guidance loop.

## Deep context (optional)

The parent hardware project — full architecture, decisions log, tradeoffs, and
airframe sizing math — lives on this machine at:
`Downloads\Simulation Work\Drone interrceptor\` (see `docs/architecture.md`,
`docs/decisions.md`, `docs/tradeoffs.md`). You do **not** need it to build the sim,
but it explains the "why" behind the counter-UAS framing if a decision needs it.
