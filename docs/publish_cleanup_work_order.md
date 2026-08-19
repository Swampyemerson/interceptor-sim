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

**None of these need a Claude session** — each is a hands-on step at the PC,
in a phone browser, or at the sim machine. The same list is on the hosted
dashboard's "Waiting on you" panel.

1. **Rotate the VM's Linux password — FIRST, the repo is ALREADY PUBLIC**
   (`interceptor123` was in SETUP-GUIDE.md until 2026-08-19 and remains in
   public git history; it is a local-VM login, so low real exposure, but
   rotate it. Option A ruling: keep history — a rewrite would dangle 39+
   commit-hash evidence citations).
2. **Re-point the desktop launcher** if its shortcut targets the repo copy —
   the .bat now lives at `scripts/env/Launch Interceptor Sim.bat`.
3. **Merge branch `claude/portfolio-data-github-cleanup-g792ae` into `main`**
   (github.com → the repo → Branches or a PR — doable from a phone). CI on
   the branch is **green** (run 142); main's CI — and the README badge —
   stays red until this merge lands.
4. ~~Confirm the MIT license~~ — **DONE: builder confirmed MIT 2026-08-19**
   (recorded in `docs/publish_runbook.md` step 4). Weights stay out of the
   repo per `docs/license_notice_weights.md`.
5. **Set the GitHub About description + topics** with runbook-compliant
   wording (claim the design, never the unflown result — the claims scrub in
   `docs/publish_runbook.md` is binding).
6. **Re-shoot the hero GIF on the sim PC** (`scripts/build_demo.py` after a
   new hero flight — the caption fix is in; the retired assets were never
   replaced). The single highest-value README addition still open.
7. Optional: enable GitHub Pages on `/docs` so `dashboard.html` / `mbse.html`
   are clickable pages rather than raw files.

Note 2026-08-19: the hosted dashboard artifact now carries the MBSE view as
its own sheet (SHEET 5); the separate MBSE artifact link is superseded and
can be ignored. The emitted artifact build fails if `docs/mbse.html` is stale
(render order: `render_mbse.py` before `render_dashboard.py --artifact`).

Delete this file once the list above is done.
