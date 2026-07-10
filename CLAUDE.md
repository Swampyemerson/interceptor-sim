# Interceptor Simulation — how to work on this project

This file auto-loads at the start of every Claude Code session. It defines the
operating model, the decision protocol, and the conventions. The mission and
scope are in `GOALS.md` (imported at the bottom) — read it before planning.

## Autonomous operation — THE STANDING ORDER (builder's #1 rule; never lose this)

**When the builder is not present, run FULLY AUTONOMOUSLY and NEVER STOP.** This
overrides any instinct to wind down, and it is the rule most easily lost — restore it
on sight. Concretely:
- **NEVER downshift to "holding on a heartbeat" and NEVER ask "what should I build
  next?" while he is away.** Idling, or surfacing to ask permission to work, IS the
  failure mode — it once turned a genuinely productive night into something that
  *looked* like nothing happened (2026-07-10). Pick the highest-value unblocked item
  yourself, BUILD it, and when it lands pick the next. Surface only for a true one-way
  door or a hard external blocker — and even then keep other work moving.
- **COMMIT VISIBLY and FREQUENTLY — uncommitted work is invisible work.** Do not hoard
  a batch "for his review"; stage specific paths and commit each milestone as it lands
  (the git log IS the proof of work). "Held for decision" applies ONLY to a genuine
  one-way door, never to done/tested/documented results.
- **MAXIMIZE THROUGHPUT under the Fable-seat clock.** Fable access is time-limited and
  the project is bigger than the remaining window — fan out Fable workers/workflows
  aggressively (ultracode), keep the sim busy with the serial gate/batch queue, and
  keep building. Steady VISIBLE progress beats a perfect idle.
- Drive every build through its loop autonomously: build → Fable review → sim gate →
  commit → next. Don't wait to be told to continue.

## Model orchestration (Fable preferred; Opus is the SAFEGUARD FALLBACK; Sonnet for volume)

REALITY (2026-07-05, builder-confirmed with the in-product notice): Fable 5 currently
ships DELIBERATELY BROAD safeguards that, per Anthropic's own message, "may flag safe
and routine coding, cybersecurity, or biology work." This counter-UAS interceptor
project's defense-sim terminology (interceptor, lethal radius, proximity fuse, kill
probability, warhead) TRIPS that broad filter, and when a turn is flagged the Claude
Code client AUTOMATICALLY SWITCHES the session to Opus 4.8. This is a product-level
safety behavior: neither the agent nor the operator can disable or bypass it, it is
NOT operator error or a bad launch, and it may re-trigger even after a manual switch
back. The sanctioned remedy is `/feedback` (the notice invites it; the safeguards are
being refined). Do NOT try to word prompts to evade the classifier.

Implications for how to work:
- **Prefer Fable when it's the active model** (it's the stronger seat for judgment),
  but EXPECT the session to bounce to Opus on flagged turns — that is NORMAL here, not
  a misconfiguration. Check `/model` to know which you're on; the project pins
  `"model": "claude-fable-5"` in `.claude/settings.json` so a fresh launch starts on
  Fable, and the safeguard may still switch it mid-session.
- **Do strong work on whichever model is active.** Correctness is model-agnostic; the
  project has progressed well across both. Don't stall waiting to be on Fable.
- **Hard / high-judgment sub-tasks:** prefer Fable (main session, or a `model: fable`
  subagent — the Task/Agent tool supports it), accepting the same flagging may apply;
  Opus is the practical fallback for this project's flagged content, not a downgrade
  to fight.
- **Sonnet 5 = genuinely mechanical / parallel / verifiable VOLUME** — installs,
  boilerplate, running sim batches, log reading, wide read-only searches. The
  `.claude/agents/` (sonnet-worker, verifier, council-member) are Sonnet-pinned; a
  `model:` override bumps them up.
- **Verify the model first thing** each session with `/model`; if you want Fable and
  the safeguard has switched you, switch back and continue, knowing it may bounce.

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

## Standing methodology rules (hard-won — each traces to a logged incident)

- **Sim time, never wall time.** RTF sags to ~0.3–0.5 under load; anything scheduled
  or measured in wall time silently distorts (cost M4 five dev runs + two failed
  gates, ADR-0009). Movers, holds, durations, latencies: sim-clock only.
- **Batch hygiene.** Gates and Monte-Carlo batches run at IDLE machine load only
  (load confound documented, ADR-0015 2nd addendum). ONE sim at a time; batch arms
  run sequentially. Never `pkill -f`/`pgrep -f` with ANY literal pattern typed
  inline in a tool-call command — the harness evals the command text, so the
  pattern sits in an ancestor's argv, self-matches, and kills the invocation
  (exit 144) even with zero real matches (generalized 2026-07-08 from the
  original "batch script's name" rule; hit 3× that day). Kill/poll logic lives
  in script FILES (scripts/sim_kill.sh, or a scratchpad script) invoked by path.
- **Statistics before verdicts.** Run-to-run terminal-dropout noise is ~1 m; a
  single-flight delta below that is noise. A/B claims need paired seeds (n≥8) plus
  mechanism evidence, and honest "not significant at this n" language.
- **Lab ranks, Gazebo decides.** guidance_lab.py is a design-time surrogate with six
  documented divergences from Gazebo (PIP, Kalata, fusion coverage, …). Its numbers
  rank options; only a Gazebo gate/batch turns a ranking into a conclusion.
- **Simulate worse than ideal.** Three tiers — BEST / EXPECTED / WORST-credible;
  decisions must survive WORST; every realism knob maps to a bench-measurable
  quantity (builder mandate, 2026-07-05).
- **Honesty boundary.** `gt_*` (ground truth) is scoring/logging ONLY; the cue is
  structurally unreadable after handoff; guidance sees camera + own-state EKF only.
  Every new guidance path re-earns the numeric no-cheat audit.
- **Git with background workers.** Stage specific paths — never `git add -A` while
  any background agent may be mid-edit (swept partial work into commits once,
  ADR-0011 3rd addendum). Worktree jobs: symlink the main `.venv`; no remote exists,
  so merge = local `git merge --ff-only` from the main checkout.

## Environment note

You run in **WSL2 (Ubuntu 24.04)** on Emerson's Windows PC, with an **NVIDIA RTX 4070 GPU available** (`nvidia-smi` works; libs under `/usr/lib/wsl/lib`). Gazebo can render **GPU-accelerated**, and WSLg gives a display for the GUI. Still prefer **headless** (`HEADLESS=1`) for automated/batch runs (faster + reproducible); use the GPU for camera rendering and the final demo. PX4 and MAVSDK both run here, so connect over local UDP (`udpin://0.0.0.0:14540`).

@GOALS.md

## Your setup & who you're helping (plain English)

You run in **WSL2 (Ubuntu 24.04)** on Emerson's Windows PC, opened from a desktop launcher straight into a terminal. You, PX4, and Gazebo all run together here, so use local connections (e.g. `udpin://0.0.0.0:14540`). You have an **NVIDIA RTX 4070 GPU** available to you.

Who you're working with: Emerson. He is new to simulation and to guidance/controls, and he's building this as a portfolio project for aerospace internships. Talk to him in plain, simple English - short sentences, minimal jargon. When you introduce a new term (proportional navigation, offboard control, camera intrinsics, EKF, etc.), give a one-line "what it is and why it matters" the first time you use it. He is learning as he goes: briefly say what you're doing and why, and flag real decisions - but keep it light, not lecture-length.

How he works with you: he triggers the kickoff, then mostly lets you run autonomously through the milestones. Keep him posted in plain language and don't assume a deep coding or simulation background. You have plenty of CPU/RAM and an **RTX 4070 GPU**; prefer headless Gazebo for batch runs, and use the GPU for camera rendering and the demo.
