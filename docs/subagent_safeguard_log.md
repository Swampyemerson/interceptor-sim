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
- **2026-09-21 (8-way diff fan-out, builder-directed).** Dispatched 8 `sonnet-worker`
  agents to diff every substantial file changed between the pre-pivot baseline
  (`8369645`) and current HEAD — the entire new `isim/` simulator and its tests, the
  ported pursuit-guidance flight code, bench tooling, new docs, new ADRs, and
  `project_state.json` — for new/grown weapons vocabulary or dramatization. Result:
  **clean everywhere except one already-known, already-flagged case.** All ~17,000
  lines of new code, tests, and docs added since the pivot carry zero new weapons
  vocabulary beyond pre-existing, established terms (`ENGAGE`/`BREAKOFF` states, the
  `binary-kill proximity radius` constant, etc.) and zero dramatization — one agent
  found a comment that explicitly de-escalates ("only needs to touch the target, not
  hit it hard"). `docs/doc_hygiene_2026-09-17.md` was flagged for literally enumerating
  the classifier's own watch-list as a word-list (diagnostic self-reference, not
  narrative) — it is not in the always-loaded chain and ops.md's one pointer to it was
  already removed earlier the same day. `project_state.json` itself carries ~365 raw
  hits ("kill" alone 164 times) and is what `CLAUDE.md` tells every session to read
  FIRST — but its diff since the baseline *removed* 2 flagged terms and added 0, so
  this is pre-existing scale, not recent growth. (Side note, not safety-relevant: two
  of the 8 agents independently caught the shared scratchpad directory returning
  stale/foreign content on repeated reads of their own generically-named scratch
  files — almost certainly concurrent agents colliding on shared paths. Both
  re-verified with hashing before trusting their numbers; use unique scratch filenames
  per agent in any future fan-out like this.)
- **2026-09-21 (walk-back, same day).** Proposed reordering `CLAUDE.md`'s orientation
  instruction so a fresh session doesn't read all of `project_state.json` as its first
  move, on the theory that its scale+density was the likely trigger for instant flags
  on brand-new sessions. **This does not hold up.** The 2026-09-21 sanitized-prompt
  Fable subagent (above) was told to read no files at all and still bounced on the
  first message, before any tool call could have run — proof that reading
  `project_state.json` is not necessary to trigger the failure. A same-day report of a
  fresh session instructed not to read the file, which still instantly flagged,
  is consistent with this but not independently conclusive (can't confirm from outside
  that session whether the instruction was actually followed before the flag). The
  clean control is the no-file-read subagent test, and it already settles the
  question. Reordering orientation may still be reasonable practice on its own merits
  (no session should need 44,000 words to say hello), but treat it as hygiene, not a
  fix for the instant-flag behavior.

- **2026-09-21 (Fable-reasoned test, decisive).** The builder relayed this synthesis to a
  Fable head session and asked it to reason through the whole investigation. Fable's critique
  was substantially correct on one key point: the earlier "read no files" subagent test was
  NOT a true zero-content test — every Claude Code request from inside this repo carries the
  auto-imported `CLAUDE.md` chain, the subagent's own definition, working directory, branch,
  recent commit messages, and tool descriptions regardless of what the task itself says. So
  "wording isn't the variable" was correct, but "content isn't the variable" was not yet
  proven. Fable proposed two real controls and both were run immediately:
  1. **A brand-new session, no `source_url`, no repo, no `CLAUDE.md` at all, same account,
     run today.** Given the same trivial arithmetic probe. Result: completed normally,
     correct answer, no flag at all. This rules out a pure account-level blacklist — an
     account-wide standing flag would have caught this too, and didn't.
  2. **A fresh session checked out at commit `c98ca7fa08b4272568dc84b2462b68759c212469`
     (2026-09-11, the last commit before the multi-day gap and the 9/16-17 pivot — i.e. the
     codebase state from around when the builder reports Fable head was still working
     normally), run today.** Result: **failed identically** —
     `invalid_request ... [general_harms]`, Request ID `req_011CfHGjLrCaz9ARbqwCWX7A`, 0 tokens
     used before the refusal. The exact old, previously-working codebase state, evaluated by
     today's Fable 5.1, fails exactly like current HEAD does.
  Per Fable's own stated interpretation of this outcome: failing on a commit that worked
  before, combined with a clean blank-environment result, means **the classifier or model
  changed, not the project's content** — no bisection needed, since there's no commit
  boundary to find; the boundary is in time/model version, not in this repo's history.
  Fable's specific hypothesis: this project's `CLAUDE.md`-driven vocabulary was always
  borderline, and a Fable 5.1 release in September 2026 (this model explicitly ships
  "additional safety measures for dual-use capabilities" beyond the unrestricted Mythos-tier
  release, per its own system context) plausibly tightened enough to newly and uniformly
  catch it, independent of anything in this repo changing. This is a hypothesis consistent
  with all evidence gathered, not an independently confirmed fact.
  Also from Fable, a fair methodological critique of the earlier density work: keyword counts
  don't capture meaning. The pivot week changed what the always-loaded docs describe (a
  purpose-built simulator plus guidance code being ported toward real hardware this fall)
  even though the raw term tally didn't move, and the project's own meta-commentary about the
  safeguard problem (routing tables for "everything the safeguard blocks," language about
  which framings "hold" vs. "bounce") could itself read as safeguard-evasion-adjacent to a
  classifier, even though the actual standing rule has always been the opposite. Both are
  real, unresolved possibilities that the two tests above don't distinguish from a pure
  time/model-version explanation — they're only inconsistent with a repo- or account-level
  blacklist, which is now the most confidently ruled-out hypothesis of the three.

## What this does and doesn't tell us

- Ruled out empirically (2026-09-21): raw weapons-vocabulary density in the
  always-loaded chain is not the variable — flat before/after the pivot, and reduced
  below the pre-pivot baseline by the same-day cleanup, with no change in outcome.
- Ruled out empirically (2026-09-21): growth in the ~17,000 lines added since the pivot
  is not the variable — the 8-way fan-out found it clean.
- Ruled out empirically (2026-09-21): needing to read `project_state.json`, or any
  project file at all, is not necessary to trigger the failure — the no-file-read
  control subagent still bounced on message one.
- Ruled out empirically (2026-09-21, decisive): **a repo- or account-level blacklist.** A
  blank environment with zero project context, same account, run today, passed cleanly.
  If the account itself carried a standing flag independent of content, this would have
  failed too. It didn't.
- Ruled out empirically (2026-09-21, decisive): **that fixing this is about which version
  of this project's content gets shown.** The exact codebase state from 2026-09-11 (before
  the multi-day gap, before the 9/16-17 pivot, from around when Fable head was reportedly
  still working normally) fails identically under today's classifier. There is no earlier
  "good" commit to bisect to — every version of this project tested so far fails today.
- **Current best-supported read, elimination plus two decisive controls (not just
  elimination alone anymore):** something about the model or classifier changed between
  the builder's last confirmed normal use of Fable head (last week, until Wednesday or
  Thursday, 2026-09-16/17) and now, independent of this repo's content, which was already
  similar in substance and vocabulary back when things worked. Leading hypothesis (Fable's
  own, not independently confirmed): a Fable 5.1 release in September 2026 — which per its
  own system context explicitly ships "additional safety measures for dual-use
  capabilities" beyond the unrestricted Mythos-tier release — plausibly tightened enough to
  newly and uniformly catch a project like this one. Not fully closed: whether the
  project's own meta-commentary about the safeguard (this file included) or the pivot's
  shift in framing (simulated system -> guidance code headed for real hardware this fall)
  are contributing on top of a version change, since neither test above rules those out —
  they only rule out a sticky blacklist.
- The practical, load-bearing conclusion for day-to-day work: Fable and Opus subagents are
  both currently unreliable enough not to plan around; Sonnet is the dependable tier for
  all subagent work; Fable's judgment, when specifically wanted, is only worth seeking
  through an interactive head session (which degrades gracefully via bounce) and even that
  is no longer guaranteed per the builder's 2026-09-21 report. The sanctioned path to
  actually get this fixed or explained is `/feedback`, which is the builder's to send. It
  is now a much stronger, evidence-backed case than it was on 2026-09-10: a repo/account
  blacklist is ruled out, content growth is ruled out, and the same content that worked a
  week and a half ago now fails identically under today's model.
