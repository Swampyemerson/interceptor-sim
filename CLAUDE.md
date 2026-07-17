# Interceptor Simulation — how to work on this project

This file auto-loads at the start of every Claude Code session. It defines the
operating model, the decision protocol, and the conventions. The mission and
scope are in `GOALS.md` (imported at the bottom) — read it before planning.

## Autonomous operation — THE STANDING ORDER (builder's #1 rule; never lose this)

**When the builder is not present, run FULLY AUTONOMOUSLY and NEVER STOP, at maximum
throughput.** This overrides any instinct to wind down; it is the rule most easily lost —
restore it on sight. Concretely:
- **NEVER downshift to "holding on a heartbeat" and NEVER ask "what should I build
  next?" while he is away.** Idling, or surfacing to ask permission to work, IS the
  failure mode — it once turned a genuinely productive night into something that
  *looked* like nothing happened (2026-07-10). Pick the highest-value unblocked item —
  or let the **Fable project manager** (see Model orchestration) pick it — BUILD it, and
  when it lands take the next. Surface only for a true one-way door or a hard external
  blocker, and even then keep other work moving.
- **SPEED matters and PARALLELISM is the lever — but spend tokens DELIBERATELY.** Minimize
  wall-clock by running genuinely independent work concurrently: keep the sim busy with the
  serial batch queue while sim-free work proceeds, and use a **Workflow** when a task truly
  decomposes. But tokens are NOT free — fan out a subagent when there is real independent
  work or when Fable's review/judgment earns its cost, NOT reflexively for every small task
  (a Fable agent per doc-nit is waste; do routine work inline). Steady VISIBLE progress
  beats both a perfect idle and a token firehose.
- **COMMIT VISIBLY and FREQUENTLY, and PUSH — uncommitted work is invisible work.** Do not
  hoard a batch "for his review"; stage specific paths and commit each milestone as it
  lands (the git log IS the proof of work), and push to the remote if one is configured.
  "Held for decision" applies ONLY to a genuine one-way door, never to done/tested/
  documented results. The Fable PM audits this so nothing strands uncommitted.
- Drive every build through its loop autonomously: build → Fable review → sim gate →
  commit → next. Don't wait to be told to continue.

## Model orchestration — Fable SELECTIVELY, for the high-leverage work (review, hard tasks, planning)

REALITY (2026-07-05, builder-confirmed): Fable 5 ships broad safeguards that flag this
project's defense-sim terms (interceptor, lethal radius, proximity fuse, kill probability,
warhead); a flagged turn AUTO-SWITCHES the session to Opus 4.8 — product behavior, cannot be
disabled, NOT operator error, may re-trigger after a manual switch back. Sanctioned remedy:
`/feedback`. Do NOT word prompts to evade the classifier.

**The routing rule the builder wants (2026-07-10, refined — a walk-back from "Fable for
everything"):** Fable is the stronger seat for judgment and is notably BETTER AT SEEING
WHAT'S MISSING — reviewing work, catching gaps, and deciding workflow/planning questions.
So spend it THERE, occasionally and deliberately, NOT as a firehose. Tokens are not free;
don't spawn a subagent where doing the thing inline is cheaper.

1. **Reach for a `model: fable` subagent OCCASIONALLY and DELIBERATELY, for high-leverage work:**
   - **REVIEW / gap-spotting** — a Fable pass over a build, a plan, a result set, or a
     decision to catch what was missed BEFORE it's committed or acted on (the "head builds,
     Fable reviews" pattern; Fable earns its cost here). This is its best use.
   - **The MOST COMPLEX / high-judgment tasks** — hard builds, subtle analyses, ambiguous
     design calls where Fable's judgment materially beats doing it on the (often
     Opus-pinned) head.
   - **WORKFLOW / PLANNING decisions** — "what's next, what's most effective, what are we
     missing" — the roadmap / prioritization / project-manager thinking Fable is best at.
     A periodic Fable PM-or-review pass to keep the project honest and well-sequenced is
     worth it; a Fable agent for every doc-nit is not.
2. **The head does the ROUTINE delegatable work itself** — ordinary builds, doc edits, ADR
   prose, commits, management-file upkeep — accepting Opus quality when the safeguard has
   pinned the head there, rather than spawning a Fable agent for each small thing. Reserve
   Fable for where its edge (review, complexity, gap-spotting) actually pays.
3. **SONNET subagents for genuinely low-reasoning VOLUME** — installs, boilerplate, sim
   batches, log/CSV greps, wide read-only searches, mechanical spec'd edits. `.claude/agents/`
   (sonnet-worker, verifier, council-member) are Sonnet-pinned; a `model:` override bumps up.
4. **OPUS subagents ONLY for what Fable can't do because the safeguard blocks it** — the
   flagged defense-framed guidance / jam / honesty builds and their reviews.

Balance: parallelize for SPEED when there is genuinely independent work in flight (keep the
sim busy while sim-free work proceeds), but let each spawn EARN its tokens — a targeted Fable
review, a hard task, or a planning pass, not reflexive fan-out. Verify the model first thing
with `/model`; the project pins `claude-fable-5`, and the safeguard may switch it mid-session
— accept the bounce, keep working.

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
- **Project-state contract:** `docs/project_state.json` is the single source of truth
  for pipeline/stage status (implemented / half-done / idea / rejected / superseded).
  READ it at session start to know what's actually built vs. hypothesis; when any
  status, active version, or decision changes, update it **+ run
  `python3 scripts/render_dashboard.py` the SAME TURN** and commit both (view:
  `docs/dashboard.html`; ritual: `docs/project_state_readme.md`; `run_tests.sh` fails
  on drift). This exists to stop decisions evaporating from chat — the drift that bred
  this session's five mirages.
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
