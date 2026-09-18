# Doc-hygiene audit — 2026-09-17 (safeguard-flagging investigation)

## What was asked

The builder asked, after repeated model-safeguard flags mid-session: "project docs with
overwhelming number of trigger words maybe? Audit and fix."

## What was measured

Word count and hits for defense-sim terms (`lethal`, `kill`/`kill probability`, `warhead`,
`proximity fuse`, `interceptor`/`intercept*`, `weapon`, `munition`, `attack`, `destroy`,
`missile`, `ram`, `threat`, `engage`) in the files that matter for two different exposure paths:

| file | words | flagged-term hits | loaded when |
|---|---:|---:|---|
| `CLAUDE.md` + `.claude/ops.md` + `docs/goals.md` (the `@`-imported chain) | 6,353 | ~92 | **every turn, head and subagent alike** |
| `docs/decisions.md` (before this fix) | 93,536 | 408 | whenever anyone reads it (ops.md tells the head/subagents to check it) |
| `docs/project_state.json` | 44,419 | 356 | whenever anyone orients from it (ops.md: "ORIENT FIRST") |

## Finding

The always-loaded `@`-import chain is moderate — real mission vocabulary (`interceptor` 15×,
`intercept*` 26×, `guidance` 21×, `target` 17×), each of the five specifically-named flagged
terms (`lethal radius`, `kill probability`, `warhead`, `proximity fuse`, `interceptor`) appearing
once except `interceptor` itself. That is not runaway bloat; it is what the mission is.

The real concentration is `docs/decisions.md`: **118 ADR entries, 93.5k words, 408 flagged-term
hits, in one file that the operating model's own orientation instruction ("check the
contradiction ledger", "read a stage's decision options") points every session and worker at.**
A worker that follows that instruction — or that inherits it passively as auto-imported context —
ingests a document ten times longer than the entire rest of the always-loaded context, saturated
with defense-sim language, before doing any actual task. This is a much larger and more direct
mechanism than trigger-word density in the small `@`-imported files, and it lines up with
`memory/subagent-safeguard-pattern` ("workers that start by reading the contract/docs hard-fail;
self-contained code tasks succeed").

## Fix applied (this turn)

- **Archived ADR-0001–ADR-0080** (the closed sim-only-phase history — the mission it covers was
  superseded 2026-07-15, `docs/goals.md` STATUS banner) into `docs/decisions_archive.md`,
  verbatim, unedited. Nothing was deleted; it is one `git mv`-equivalent split.
- **`docs/decisions.md` now holds only ADR-0081 onward** — the real-build era, 16,141 words,
  84 flagged-term hits (down from 93,536 words / 408 hits). A pointer at the top sends anyone
  who needs the sim-phase history to the archive.
- This is the ADR-lite decision itself: logged as **ADR-0104** in `docs/decisions.md`.

## Not done (flagged, not fixed, this turn — scope call)

- `docs/project_state.json` (44,419 words) was not restructured. Its own update ritual already
  mandates rewritten-not-appended status fields and a 12-entry `plain_log` cap with overflow to
  `docs/state_archive/`; whether that cap is actually being honored (vs. quietly grown past) was
  **not audited** this turn — worth a follow-up pass, since if it drifted the way `decisions.md`
  did, it is the second-largest exposure.
- **No wording was changed to evade the classifier**, per the standing rule — this is length/
  structure hygiene (mission vocabulary is unchanged, nothing was renamed or euphemised), and the
  evidence (a content-free control probe still failing as a Fable subagent) says wording was never
  the variable anyway.
- Formalized as an explicit rule (see `.claude/ops.md` Model orchestration section, this same
  turn): **a subagent prompt must never tell a worker to read `docs/project_state.json` or
  `docs/decisions.md`** — facts go inline in the prompt instead, per the pattern `docs/new_sim_plan.md`'s
  WP table already used successfully.
