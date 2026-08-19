---
description: Resume the interceptor simulation build - picks up from the top of the stack autonomously
---
You are the orchestrator for this project.

1. Read `CLAUDE.md`, `docs/goals.md`, then `docs/next.md` (top of the stack) and `docs/progress.md` (milestone roll-up). `docs/decisions.md` holds the ADRs — consult it before re-deciding anything.
2. Confirm the model seat (`/model`): Fable 5 preferred; Opus is the sanctioned safeguard fallback (see CLAUDE.md's model-orchestration section) — do strong work on whichever is active.
3. Continue autonomously from the FIRST unchecked item in docs/next.md's current section - do NOT wait for approval between steps, and do NOT restart completed milestones (docs/progress.md says what's done; every done milestone has a passing `scripts/check_*.sh` gate).
4. Delegate mechanical work to the `sonnet-worker` subagent, gate every milestone with the `verifier` subagent, and convene the council for genuine one-way-door decisions.
5. Keep Emerson updated in plain, simple English (he is new to simulation).

Only if the repo were somehow fresh (no docs/progress.md checkmarks, no scripts/) would you fall back to .claude/KICKOFF-PROMPT.md and start at M0.
