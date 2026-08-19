# interceptor-sim — session operating notes

Orient FIRST from `docs/project_state.json` — the machine-readable contract
(stage statuses, constraints, the contradiction ledger, the dead-ideas
graveyard); `docs/dashboard.html` is its rendered view. `docs/next.md`,
`docs/progress.md`, and the ADRs in `docs/decisions.md` are subordinate views.
Mission + scope: `docs/goals.md` (imported below). The full operating model
(autonomy, model routing, decision protocol, standing methodology rules):
`.claude/ops.md` (imported below).

## Build / test

- Offline suite + drift gate: `scripts/run_tests.sh` (pytest over `tests/` +
  `flight/tests/`, then `python3 scripts/render_dashboard.py --check`).
  Green means ran AND passed — a self-test that cannot run is a FAILURE.
- Milestone gates are scripted: each `scripts/check_*.sh` exits 0/1 — never
  claim a milestone done without running its gate and showing the output.
- Sim runs are headless by default (`HEADLESS=1`); see the `px4-gazebo` and
  `run-interceptor-sim` skills. Fresh environment: `scripts/env/bootstrap.sh`.

## The honesty boundary (the project's spine)

- `gt_*` ground-truth topics are scoring/logging ONLY; guidance sees camera
  pixels + own-state EKF, nothing else — enforced by AST-based no-cheat tests.
- The boundary covers pre-flight GIVENS too: every input the system is given
  rather than measures is graded in the contract's assumptions register, and a
  number computed on a `given-perfect` input is a BEST-CASE UPPER BOUND, never
  the claim.
- Numbers trace to a run or a derivation — no unsourced quantitative claims.

## Update ritual (housekeeping rule #1)

The SAME TURN a status, decision, or contradiction changes: edit
`docs/project_state.json` → `python3 scripts/render_dashboard.py` → republish
the Artifact to the stored `artifact_url` → commit both. Status fields are
REWRITTEN (validator-enforced caps), never appended; dated notes go to
`plain_log`. An update that isn't in the contract didn't happen.

@.claude/ops.md
@docs/goals.md
