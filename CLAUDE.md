# Interceptor Simulation — how to work on this project

This file auto-loads at the start of every Claude Code session. It defines the
operating model, the decision protocol, and the conventions. The mission and
scope are in `GOALS.md` (imported at the bottom) — read it before planning.

## Model orchestration (Fable 5 oversight · Opus middleground · Sonnet 5 volume)

THIS IS A HARD REQUIREMENT, NOT A PREFERENCE. Three tiers:

- **Main session MUST be Claude Fable 5** (`claude-fable-5`). It orchestrates,
  integrates, reviews, and owns decisions — Fable's judgment is the point of the
  main seat; Opus is NOT an acceptable substitute for it. **First action every
  session: verify the model.** Run `/model` and confirm "Fable 5"; if it shows
  anything else (Opus, Sonnet, …), STOP and switch — either `/model` → Fable 5,
  or relaunch `claude --model claude-fable-5`. The project pins
  `"model": "claude-fable-5"` in `.claude/settings.json` so launches default
  correctly; a wrong model means that pin was overridden at launch — fix it before
  doing real work. (An agent cannot change its own running model mid-session; only
  the operator can, via `/model` or relaunch.)
- **Opus 4.8 (`claude-opus-4-8`) for MIDDLEGROUND tasks** — work that needs strong
  reasoning but not the main seat's oversight, and where Sonnet may be insufficient:
  a tricky implementation, a hard debugging root-cause, a subtle analysis, a
  design prototype. Delegate via the Task tool with an explicit `model: opus`
  override. Use this tier deliberately when a task is above mechanical-Sonnet
  difficulty but you don't want to spend the Fable main seat on it.
- **Sonnet 5 for VOLUME/mechanical/parallel/verifiable work** — conserve the higher
  tiers. The agents in `.claude/agents/` are pinned to `model: sonnet`:
  - `sonnet-worker` — installs, boilerplate, coding a module, running sims, reading logs, writing tests.
  - `verifier` — runs a milestone's check script and adversarially confirms pass/fail.
  - `council-member` — one independent voice in a decision council (see below).
  (To run one of these at the middleground tier, pass a `model: opus` override on
  the Task call.)
- **Routing rule:** Keep on **Fable** — architecture, cross-module integration,
  final decisions, reviewing subagent output. Push to **Opus** — hard-but-scoped
  tasks Sonnet might botch. Push to **Sonnet** — anything mechanical, parallel, or
  verifiable. When in doubt: draft with Sonnet, escalate to Opus if it's hard,
  review with Fable.

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

You run in **WSL2 (Ubuntu 24.04)** on Emerson's Windows PC, with an **NVIDIA RTX 4070 GPU available** (`nvidia-smi` works; libs under `/usr/lib/wsl/lib`). Gazebo can render **GPU-accelerated**, and WSLg gives a display for the GUI. Still prefer **headless** (`HEADLESS=1`) for automated/batch runs (faster + reproducible); use the GPU for camera rendering and the final demo. PX4 and MAVSDK both run here, so connect over local UDP (`udpin://0.0.0.0:14540`).

@GOALS.md

## Your setup & who you're helping (plain English)

You run in **WSL2 (Ubuntu 24.04)** on Emerson's Windows PC, opened from a desktop launcher straight into a terminal. You, PX4, and Gazebo all run together here, so use local connections (e.g. `udpin://0.0.0.0:14540`). You have an **NVIDIA RTX 4070 GPU** available to you.

Who you're working with: Emerson. He is new to simulation and to guidance/controls, and he's building this as a portfolio project for aerospace internships. Talk to him in plain, simple English - short sentences, minimal jargon. When you introduce a new term (proportional navigation, offboard control, camera intrinsics, EKF, etc.), give a one-line "what it is and why it matters" the first time you use it. He is learning as he goes: briefly say what you're doing and why, and flag real decisions - but keep it light, not lecture-length.

How he works with you: he triggers the kickoff, then mostly lets you run autonomously through the milestones. Keep him posted in plain language and don't assume a deep coding or simulation background. You have plenty of CPU/RAM and an **RTX 4070 GPU**; prefer headless Gazebo for batch runs, and use the GPU for camera rendering and the demo.
