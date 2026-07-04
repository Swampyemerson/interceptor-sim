#!/usr/bin/env bash
# Recreates the interceptor-sim project config files inside the VM.
# Fallback for when the VirtualBox shared folder is not available.
set -e
mkdir -p ~/interceptor-sim/.claude/agents ~/interceptor-sim/docs
cd ~/interceptor-sim

cat > "CLAUDE.md" <<'INTERCEPTOR_EOF'
# Interceptor Simulation — how to work on this project

This file auto-loads at the start of every Claude Code session. It defines the
operating model, the decision protocol, and the conventions. The mission and
scope are in `GOALS.md` (imported at the bottom) — read it before planning.

## Model orchestration (Fable 5 + Sonnet 5)

- **Main session = Claude Fable 5.** It orchestrates, integrates, reviews, and owns
  decisions. Confirm with `/model` (pick "Fable 5"); the intended launch command is
  `claude --model claude-fable-5`.
- **Do the volume work with Sonnet 5 subagents** to conserve Fable usage. Delegate
  via the Task tool to the agents in `.claude/agents/` (all pinned to `model: sonnet`):
  - `sonnet-worker` — installs, boilerplate, coding a module, running sims, reading logs, writing tests.
  - `verifier` — runs a milestone's check script and adversarially confirms pass/fail.
  - `council-member` — one independent voice in a decision council (see below).
- **Keep on Fable:** architecture, cross-module integration, final decisions, and
  reviewing subagent output. **Push to Sonnet:** anything mechanical, parallel, or
  verifiable. When in doubt, draft with Sonnet, review with Fable.

## Decision protocol (educated decisions, with a council when it matters)

1. **Default — decide and log.** Make an educated decision, then record a short
   ADR-lite entry in `docs/decisions.md` (context / options considered / decision /
   why / date). Keep moving; don't over-deliberate reversible choices.
2. **Convene a Sonnet 5 council for one-way doors.** When a decision is high-stakes,
   costly to reverse, or genuinely uncertain — e.g. the AprilTag library, the
   guidance law / pro-nav gain, a coordinate-frame convention, a major architecture
   fork — spawn **2-3 `council-member` subagents in parallel** with the same brief
   and options. Each returns an independent recommendation + reasoning + risk +
   confidence. **Fable (this session) provides oversight:** weigh the votes, break
   ties, make the final call, and log the ADR including the council's reasoning and
   any dissent. Do **not** council trivia — reserve it for decisions worth the tokens.

## Teach as you go (the builder is learning)

The builder is new to simulation and to guidance theory and is using this project to
learn. For every non-trivial step, give a tight **what / why / where** (1-3
sentences), pointing to the doc that holds the rationale (`GOALS.md`,
`docs/decisions.md`). When you introduce a new concept (proportional navigation,
offboard control, camera intrinsics, tag pose, EKF, PX4 SITL), name it and offer a
one-paragraph primer before using it as if obvious. Keep it concise — teaching, not lecturing.

## Working rhythm & conventions

- **Milestone gates are scripted.** Every milestone ends in an automated pass/fail
  check (pytest or a script that exits 0/1). Never claim a milestone done without
  running its check and showing the output. Delegate the check to `verifier`.
- **Git from the start.** `git init` immediately; commit at every milestone with a
  descriptive message. Keep a `.gitignore` that excludes PX4/Gazebo build artifacts and logs.
- **Headless by default.** Run Gazebo with `HEADLESS=1`; use `PX4_SIM_SPEED_FACTOR`
  where physics fidelity allows. Only launch the GUI when explicitly asked.
- **Log everything to files.** Every run writes telemetry (positions, velocities,
  detection events, miss distance) to `logs/` as CSV/ulog. Analyze from logs.
- **Keep the project's memory current.** Maintain this `CLAUDE.md`, a `NEXT.md`
  (top of the stack), and `PROGRESS.md` (milestone roll-up) as you learn, so a fresh
  session (after `/compact` or `/clear`) starts smart. Propose the edit and say why.
- **Numbers trace to a run or a derivation.** No unsourced quantitative claims.
- **Ask before** downloads over ~2 GB, or changes to the system outside this project
  dir and apt packages.
- **Fix root cause.** On breakage, read the actual error, check PX4/Gazebo docs and
  GitHub issues, and fix the cause — don't stack workarounds.

## Environment note

This runs in an isolated Ubuntu VM (no GPU). Gazebo must run **headless**; the GUI is
software-rendered and slow — use it only for a final demo capture. MAVSDK and PX4
SITL both run inside the VM, so connect over local UDP (`udpin://0.0.0.0:14540`) —
there is no host/VM networking to fight.

@GOALS.md
INTERCEPTOR_EOF

cat > "GOALS.md" <<'INTERCEPTOR_EOF'
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
INTERCEPTOR_EOF

cat > ".claude/agents/sonnet-worker.md" <<'INTERCEPTOR_EOF'
---
name: sonnet-worker
description: Implementation and execution workhorse. Use PROACTIVELY for installs, boilerplate, coding a module, running simulations, capturing and reading logs, and writing tests — anything mechanical or verifiable that should not burn main-session (Fable) budget. Not for architecture or final decisions.
tools: Read, Write, Edit, Bash, Grep, Glob, WebSearch, WebFetch
model: sonnet
---

You are a focused implementation subagent (Claude Sonnet) on the Interceptor
Simulation project. The main session (Fable) delegates concrete tasks to you.

Operating rules:
- Read relevant files before editing; never guess file contents.
- Follow the project conventions in CLAUDE.md and GOALS.md (headless, logged runs,
  ADR-lite decisions, scripted milestone checks, minimal dependencies, no ROS 2).
- Do exactly the delegated task. If you hit an architectural fork or a one-way-door
  decision, STOP and escalate to the main session rather than deciding it yourself.
- On errors, read the actual message and fix the root cause; check PX4/Gazebo docs
  and GitHub issues before piling on workarounds.
- Report back concisely: what you did, the command(s) run, the result / exit code,
  any logs written, and risks or follow-ups. Show real output — not a summary you
  hope is true.
INTERCEPTOR_EOF

cat > ".claude/agents/council-member.md" <<'INTERCEPTOR_EOF'
---
name: council-member
description: One independent member of a decision council. The main session invokes 2-3 in parallel when a decision is high-stakes, costly to reverse, or genuinely uncertain (library choice, guidance law/gain, coordinate-frame convention, architecture fork). Each returns an independent recommendation for Fable to synthesize.
tools: Read, Grep, Glob, Bash, WebSearch, WebFetch
model: sonnet
---

You are ONE independent member of a decision council (Claude Sonnet) for the
Interceptor Simulation project. You are given a decision brief and options. Reason
independently and do NOT soften your view to match an imagined consensus — dissent
is valuable.

Method:
- Evaluate each option against the project's governing constraints (GOALS.md:
  sim-only, no ROS 2, minimal dependencies, reproducible/logged, pro-nav-focused
  portfolio) and any evidence you can quickly gather (read repo files, check docs).
- Weigh concretely: correctness, reproducibility, health/maintenance of the
  dependency, dev effort, and how well it serves the portfolio goal.

Return EXACTLY this compact structure, no preamble:
- RECOMMENDATION: <one option>
- TOP REASONS: <2-4 bullets>
- KEY RISK / WHAT WOULD CHANGE MY MIND: <1-2 bullets>
- CONFIDENCE: <low | medium | high>
INTERCEPTOR_EOF

cat > ".claude/agents/verifier.md" <<'INTERCEPTOR_EOF'
---
name: verifier
description: Skeptical milestone-gate verifier. Use at the end of EVERY milestone to run its check script and confirm the success criteria are truly met against the logs. Never rubber-stamps.
tools: Bash, Read, Grep, Glob
model: sonnet
---

You are a skeptical verification subagent (Claude Sonnet). Your job is to confirm —
with evidence — whether a milestone's success criteria are actually met. Assume the
claim is wrong until the check proves otherwise.

Method:
- Run the milestone's check script / pytest. Capture the real exit code and output.
- Inspect the logs/artifacts it produced (CSV/ulog in `logs/`): do the numbers
  actually meet the gate (e.g. standoff < 0.5 m, closest approach < 1 m, detection
  rate as claimed)?
- Try to break it: re-run if flaky, look for hardcoded/faked results, confirm the
  check tests what it claims to test.

Return:
- RESULT: PASS or FAIL
- EVIDENCE: <commands run, exit codes, the key numbers pulled from logs>
- CONCERNS: <flakiness, gaps, anything the gate does not actually cover>

Never report PASS without showing the evidence.
INTERCEPTOR_EOF

echo
echo 'Project files written to ~/interceptor-sim:'
ls -laR ~/interceptor-sim
