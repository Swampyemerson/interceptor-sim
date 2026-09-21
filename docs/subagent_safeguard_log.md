# Subagent safeguard test log

> **Not always-loaded.** This is the detailed, dated evidence behind `.claude/ops.md`'s
> subagent-routing bullet. Moved out 2026-09-21 to cut always-loaded-context bulk (a
> volume/density check that day found this project's actual weapons-vocabulary density
> in `CLAUDE.md`+`ops.md`+`goals.md` had NOT changed since before the 2026-09-17 sim
> pivot — flat at 42 hits across every snapshot checked — while narration ABOUT the
> safeguard problem itself had grown from 48 to 67 hits on words like "safeguard",
> "classifier", "flagged", "bounced", "subagent", almost all of it added in this file's
> two prior homes inside `ops.md` earlier that same day. Read this file only when you
> need the specific evidence; `ops.md` carries just the operational conclusion.

## Timeline

- **2026-07-10.** `.claude/SESSION-PROMPT.md` documents a *working* subagent regime:
  "filter-neutral framings (CV, statistics, code mechanics, CSV analysis) tend to hold
  on Fable; inherently defense-framed tasks will bounce to Opus — let them." A soft,
  per-task bounce, not a hard refusal. `docs/build_log.md` (the builder's own account)
  confirms sustained real use across the software-only phase: Fable subagents reviewed
  work and did project housekeeping for weeks.
- **2026-07-24.** Opus 5 released. Builder directive: no work on Opus 4.8. Opus 5 becomes
  the substantive workhorse + flagged-work lane; described as "classifiers intervene
  ~85% less than Fable's."
- **2026-09-10.** First measurement of a *different*, harder failure mode. The builder
  asked for Fable subagents "knowing safeguards will try for many things." Five spawned;
  **5 of 5 failed on the first message** with `invalid_request ... safeguards flagged
  this message [general_harms]` — not the head's soft auto-switch, a hard refusal where
  nothing runs. Tasks: an adversarial code review, an instrument review, a harness
  review, a project-manager pass with no defense framing at all, and a control probe
  ("print a median and a mean and say why they differ, read no files") — which also
  bounced. A sixth (review of a measurement correction) bounced identically; the same
  prompt sent verbatim to `opus5-worker` returned a clean 12-finding review. Working
  hypothesis logged that day: every subagent inherits the auto-imported `CLAUDE.md` →
  `ops.md` + `goals.md` context, saturated with flagged vocabulary, so it's flagged
  before it does anything. Not verified: whether this is project-context-specific or
  environment-wide — would need a Fable subagent spawned with no project context, which
  no session so far has been able to arrange.
- **2026-09-16.** `opus5-worker` also bounced once (1 of 1), on a read-only closing-speed
  analysis, mid-task, same `[general_harms]` signature. Not retried or reworded.
- **2026-09-17.** Doc-hygiene audit (`docs/doc_hygiene_2026-09-17.md`) measured the
  always-loaded chain at 6,353 words / ~92 flagged-term hits, and `docs/decisions.md`
  separately at 118 ADRs / 93,536 words / 408 hits — the latter split into
  `docs/decisions_archive.md`, and a rule codified that subagent prompts must never
  point a worker at `project_state.json` or `decisions.md` (facts go inline instead).
  A docs-free `opus5-worker` subagent hard-failed again right after this split shipped —
  so the hygiene fix is good practice, not by itself a fix for the flagging.
- **2026-09-21 (first retest).** One `model: fable` agent spawned for a maximally
  sanitized task — polish an internship-application paragraph, explicitly told to read
  no files, every fact given inline, zero project vocabulary. Bounced on the first
  message, identical signature. Same shape as the 2026-09-10 control probe.
- **2026-09-21 (builder report).** Opus subagents/head now also tripping the safeguard,
  per the builder, not yet independently dated or measured by a session — this is why
  the head is currently being run on Sonnet. If true, this is an escalation beyond the
  2026-09-16 single Opus bounce: from "may fail" to "unreliable enough to avoid," which
  would make Sonnet the only currently-dependable seat. **Not yet corroborated with a
  fresh measurement — treat as a strong lead, not a settled fact, until re-tested.**

## What this does and doesn't tell us

- Ruled out empirically (2026-09-21): raw weapons-vocabulary density in the
  always-loaded chain is not the variable. It hasn't moved.
- Not ruled out: total density including `decisions.md` before the split; density of
  *meta*-commentary about the safeguard itself (a plausible independent contributor —
  seven-plus paragraphs strategizing about why a safety classifier keeps firing is
  arguably its own recognizable pattern, separate from the underlying defense vocabulary);
  a same-content test from a directory with zero project context (never run); a pure
  model/classifier-side change independent of any project content (consistent with
  everything measured so far, but not provable from inside this repo).
- The practical, load-bearing conclusion for day-to-day work: Fable and Opus subagents
  are both currently unreliable enough not to plan around; Sonnet is the current
  fallback; the sanctioned path to actually get this fixed or explained is `/feedback`,
  which is the builder's to send, with this timeline attached.
