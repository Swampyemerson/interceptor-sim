# The living project-state dashboard — how it works and the ritual that keeps it alive

*Built 2026-07-17 at the builder's request. The problem it solves: decisions that live
only in a conversation evaporate; a day later an approach turns out to have been
quietly bad and no single source of truth caught the drift. This system forces every
stage's state to be EXPLICIT — a box set to implemented / half-done / idea / rejected
cannot quietly vanish.*

## The two layers (no drift between them, by construction)

| layer | file | role |
|---|---|---|
| **THE CONTRACT** | `docs/project_state.json` | Machine-readable truth: every pipeline stage, its status, active version, honest note, evidence pointers, per-stage changelog; plus the goal, the current binding wall, hard constraints, and the graveyard of closed nulls. **A fresh Claude session READS this first and UPDATES it when anything changes.** |
| **THE HUMAN VIEW** | `docs/dashboard.html` | Self-contained page (no network, opens from `file://`, light+dark theme). Flowchart with feed lines + per-stage expanders, constraints cards, graveyard table. It renders ONLY the JSON embedded by the renderer — it cannot say anything the contract doesn't. |

The bridge: `scripts/render_dashboard.py` validates the JSON (status enum, evidence
required, edge integrity) and injects it into the HTML between generated-block markers.
`--check` exits 1 if the two disagree — and **`scripts/run_tests.sh` runs that check**,
so drift fails the test suite.

## Statuses (the honest vocabulary — pick exactly one)

`implemented` (built AND its claim validated at its stated scope) · `half-done`
(built but unvalidated, partially validated, or blocked — say which in the note) ·
`idea` (designed/hypothesized, not built) · `rejected` (tried and killed — move the
one-liner to the graveyard too) · `superseded` (replaced; name the successor).

Schema note: "validated" is deliberately NOT a separate status — this project's five
mirages all lived in the gap between "implemented" and "validated," so the `note`
field must always say which one a `half-done`/`implemented` claim is, with evidence.

## The session-start ritual (a fresh Claude session, or the builder returning)

1. **Read `docs/project_state.json`** (it is small — read all of it). It is the
   current truth; NEXT.md holds the work queue, the ADRs hold the full stories.
2. **Run `python3 scripts/render_dashboard.py --check`** — confirms the view matches
   the contract (also runs inside `run_tests.sh`).
3. **Say out loud what is active** (deployed seeker, guidance config, current wall)
   before planning — plans built on a stale assumption are how approaches go
   quietly bad.

## The update ritual (SAME TURN as the change — this is the whole point)

When a decision lands, a build completes, a result flips a status, or a lever dies:

1. Edit `docs/project_state.json`: change the stage's `status`/`active`/`note`,
   append a one-line `changelog` entry (date + note), add the evidence pointer
   (ADR / log / file), and bump the top-level `updated` date. Rejected levers get a
   `graveyard` row. **Never delete** — statuses move, entries don't vanish.
2. Run `python3 scripts/render_dashboard.py` (re-renders the HTML).
3. Commit **both files together** (plus the change that motivated them).

Cost per update: one small JSON edit + one command. If it ever feels heavier than
that, simplify the schema rather than skip the update — a stale contract is worse
than no contract.

## Rules that keep it honest

- The JSON is the ONLY thing edited by hand. Never hand-edit the dashboard's
  embedded state block (`--check` catches it; the page also fails loud if its
  state is missing/broken).
- Every stage must carry an `evidence` pointer — no unsourced status claims
  (the project-wide "numbers trace to a run or a derivation" rule).
- The dashboard shows a staleness banner when `updated` is >7 days old.
- Keep the file SMALL: ~10 stages, notes of 1–3 sentences, changelogs trimmed to
  the last few entries (the ADRs are the archive). Detail belongs in
  `docs/decisions.md`; this file is the state, not the story.

## CLAUDE.md hook (proposed pointer — one bullet under "Working rhythm")

> - **Project state contract:** `docs/project_state.json` is the single source of
>   truth for pipeline/stage status — read it at session start; when any status,
>   active version, or decision changes, update it + `python3
>   scripts/render_dashboard.py` the SAME TURN and commit both (view:
>   `docs/dashboard.html`; ritual: `docs/project_state_readme.md`).
