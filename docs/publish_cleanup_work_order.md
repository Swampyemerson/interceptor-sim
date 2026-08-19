# Pre-publication cleanup — EXECUTED 2026-08-19 (what remains is yours)

The full work order this file used to hold was executed on the
`claude/portfolio-data-github-cleanup-g792ae` branch (commits `7ea21c7`,
`5c40a96`, and the P2/P3 commit that carries this note; the original order
text lives in this file's git history). Done: password line deleted, the
retracted 12/16 kill-radius claim swept from every surface (PROGRESS, NEXT,
the contract, dashboard republished), root de-cluttered (goals/progress/next/
build_log under `docs/`, prompts under `.claude/`, launcher+bootstrap under
`scripts/env/`, intrinsics under `configs/`, `demo_out/` removed, CLAUDE.md
split into a 40-line summary + `.claude/ops.md`), README repackaged (image
rows, systems-engineering section with generated dashboard/MBSE screenshots,
results notes split out, CI badge, 60-second no-sim check, AI disclosure), and
the CI red fixed (the rescore gate test now skips loudly off the dev box; the
suite + drift gate verified green here against a pristine baseline).

## Remaining steps only the builder can do

1. **Rotate the VM's Linux password** (`interceptor123` was in SETUP-GUIDE.md
   until 2026-08-19 and remains in git history — Option A ruling: rotate the
   password, keep history; a rewrite would dangle 39+ commit-hash evidence
   citations).
2. **Re-point the desktop launcher** if its shortcut targets the repo copy —
   the .bat now lives at `scripts/env/Launch Interceptor Sim.bat`.
3. **Merge this branch to `main`** — CI runs on the push and should go green
   (it had been red since 2026-08-11; the fix is in `tests/test_rescore_cpa.py`).
   The README's CI badge shows `main`'s status, so it reads red until merged.
4. **Confirm the MIT license posture** before flipping public
   (`docs/publish_runbook.md` step 4; weights ruling in
   `docs/license_notice_weights.md`).
5. **Flip public** only after re-reading the claims scrub
   (`docs/publish_runbook.md`) and spot-checking the README against it; then
   set the GitHub About description + topics with runbook-compliant wording
   (claim the design, never the unflown result).
6. **Re-shoot the hero GIF on the sim PC** (`scripts/build_demo.py` after a
   new hero flight — the caption fix is in; the retired assets were never
   replaced). This is the single highest-value README addition still open.
7. Optional: enable GitHub Pages on `/docs` so `dashboard.html` / `mbse.html`
   are clickable pages rather than raw files.

Delete this file once the list above is done.
