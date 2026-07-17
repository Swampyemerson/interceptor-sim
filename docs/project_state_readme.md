# The living project-state dashboard — how it works and the ritual that keeps it alive

*Built 2026-07-17 at the builder's request. The problem it solves: decisions that live
only in a conversation evaporate; a day later an approach turns out to have been
quietly bad and no single source of truth caught the drift. This system forces every
stage's state to be EXPLICIT — a box set to implemented / half-done / idea / rejected
cannot quietly vanish.*

## The two layers (no drift between them, by construction)

| layer | file | role |
|---|---|---|
| **THE CONTRACT** | `docs/project_state.json` | Machine-readable truth (schema v2): every pipeline stage, its status, active version, honest note, evidence pointers, per-stage changelog; plus the goal, the current binding wall, the **key_numbers** cluster, the per-stage **decisions** records, the **contradictions** flag ledger, hard constraints, and the graveyard of closed nulls. **A fresh Claude session READS this first and UPDATES it when anything changes.** |
| **THE HUMAN VIEW** | `docs/dashboard.html` | Self-contained page (no network, opens from `file://`, light+dark theme; "RANGE LOG" technical-memo styling). Title block (REV = git short-sha at render time), binding-wall callout, key-numbers cluster, pipeline flowchart with feed lines + per-stage expanders, nested decision records (stage → decision → each option's full why-choose/pros/cons), the contradiction flag panel, constraints + graveyard tables. It renders ONLY the JSON embedded by the renderer — it cannot say anything the contract doesn't. |

Plus a **hosted snapshot** — the dashboard published as a claude.ai Artifact (URL in the
contract's top-level `artifact_url`), regenerated with `render_dashboard.py --artifact` and
republished to the SAME URL on meaningful changes. The repo files stay the source of truth;
the Artifact is the shareable-anywhere view.

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

## Schema v2 — decisions, contradictions, key numbers

Three top-level arrays were added 2026-07-17 (all validated by `render_dashboard.py`):

- **`key_numbers`** — the instrument-cluster figures on the dashboard. Each entry:
  `label / value / note (optional) / provenance`. The provenance is REQUIRED (project
  rule: numbers trace to a run or a derivation) — an entry without one fails validation.
- **`decisions`** — the informed-decision aid. Each record: `stage_id` (must match a
  stage), `question`, `options[]` (each with `name / summary / why_choose / pros /
  cons / status` where status ∈ `chosen | rejected | deferred | superseded`),
  `chosen_rationale`, `evidence`. A stage may carry several records. The dashboard
  nests them stage → decision → option so each option's full case is one click deep.
  When a decision changes, edit the record (flip option statuses, update the
  rationale) the SAME TURN — these are living state, not append-only ADRs (the ADRs
  in `docs/decisions.md` remain the archive of record).
- **`contradictions`** — the doc-consistency flag ledger. Each entry: `id`, `topic`,
  `verdict` (CONFIRMED/PARTIAL from the adversarial verification), `severity`
  (`high | medium | low`), `status` (`open | resolved`), both quotes with locators
  (`claim_a`/`loc_a` = the stale side, `claim_b`/`loc_b` = the current side),
  `current_truth`, and (once fixed) `resolution` describing the inline supersession
  applied. **Resolved entries stay in the ledger** — the fix is auditable, and the
  panel is the record that the conflict was found and closed.

### The contradiction-flag workflow

1. A contradiction is found (audit pass, Fable review, or mid-work) → add an entry
   with `status: "open"`, both quotes + locators, and the current truth. Re-render.
   The panel shows it as an OPEN flag (loud) until it is fixed.
2. Fix the stale doc CONSERVATIVELY: living docs (NEXT.md, PROGRESS.md, README,
   hardware_order_list, real_build/real_data/quad_retrain docs, GOALS.md) get the
   stale line marked **superseded INLINE** with a short pointer to the current truth
   / `docs/project_state.json` — never delete the historical narrative. Append-only
   ADRs in `docs/decisions.md` are NEVER rewritten — at most append a dated
   correction-pointer addendum when one is genuinely missing.
3. Set the entry's `status` to `"resolved"`, add the `resolution` note, re-render,
   and commit the JSON + HTML + every doc touched, together.
4. If a fix is ambiguous or risky, leave it `open` and surface it to the builder —
   an honest open flag beats a bad edit.

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
3. **Republish the hosted Artifact** (when the change is user-visible/meaningful — batch
   trivial edits): `python3 scripts/render_dashboard.py --artifact /tmp/state.html`, then the
   Artifact tool with `url` = the contract's `artifact_url` — keeps the SAME link.
4. Commit **both files together** (plus the change that motivated them).

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
- Keep the STAGE layer small: ~10 stages, notes of 1–3 sentences, changelogs trimmed
  to the last few entries (the ADRs are the archive). The `decisions` and
  `contradictions` arrays are allowed to be long (they carry quoted evidence), but
  each entry must stay hand-editable — one JSON object per decision/flag.
- The dashboard strips emoji from rendered strings (memo styling carries status via
  glyph + code); don't rely on emoji in JSON fields to convey state.

## CLAUDE.md hook (PLACED 2026-07-17 — the operating model is now dashboard-first)

The contract is the project's compass: see **"The project-state contract — your compass"**
near the top of `CLAUDE.md` (session-start ORIENT → reference-while-acting → update-every-change
incl. Artifact republish). This readme is the schema + ritual detail that section points to.
