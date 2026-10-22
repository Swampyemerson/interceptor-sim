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

## Remaining steps

**Status as of 2026-10-22** (one marker per item; checked against the repo,
`git branch -r`, and the GitHub API where possible). Items 1 and 2 are
builder-only actions on his own machine, so their status cannot be checked
from the repo and is recorded as OPEN until he confirms.

1. **[OPEN, builder-only, unverifiable from the repo]**
   **Rotate the VM's Linux password — FIRST, the repo is ALREADY PUBLIC**
   (`interceptor123` was in SETUP-GUIDE.md until 2026-08-19 and remains in
   public git history; it is a local-VM login, so low real exposure, but
   rotate it. Option A ruling: keep history — a rewrite would dangle 39+
   commit-hash evidence citations).
2. **[OPEN, builder-only, unverifiable from the repo]**
   **Re-point the desktop launcher** if its shortcut targets the repo copy —
   the .bat now lives at `scripts/env/Launch Interceptor Sim.bat`.
3. **[DONE]** ~~**Merge branch `claude/portfolio-data-github-cleanup-g792ae` into `main`**~~
   — **DONE 2026-08-29**, merged locally and pushed. It had sat unmerged for
   ten days; nobody noticed because the *published dashboard* was current
   while `main` was not, so every surface a human looks at said the work had
   landed. Found only because a dashboard republish hit a version conflict
   and the 2026-08-19 contract had to be recovered out of the published
   Artifact to see what was missing. Post-merge gate: `run_tests.sh` ALL
   GREEN (941 + 29 tests, both drift checks, every self-test pack).
   **Process fix owed: check for unmerged remote branches at session start.**
   Re-verified 2026-10-22: `origin/claude/portfolio-data-github-cleanup-g792ae`
   has 0 commits not on `main`.
4. **[DONE]** ~~Confirm the MIT license~~ — **DONE: builder confirmed MIT 2026-08-19**
   (recorded in `docs/publish_runbook.md` step 4). Weights stay out of the
   repo per `docs/license_notice_weights.md`.
5. **[OPEN, verified 2026-10-22: description empty, no topics]**
   **Set the GitHub About description + topics** with runbook-compliant
   wording (claim the design, never the unflown result — the claims scrub in
   `docs/publish_runbook.md` is binding).
6. **[OPEN; the tool named here is superseded]** **Re-shoot the hero
   GIF/video on the sim PC.** The script this item originally named,
   `scripts/build_demo.py`, was retired with the T25 demo pipeline (now
   `retired/scripts/build_demo.py`). The current path is the CINEMA pipeline
   (`scripts/cinema/README.md`: boot with `scripts/demo_boot_sim.sh`, fly
   `scripts/cinema/cinema_run.py`, then export / Blender render / HUD
   compose); `scripts/demo_capture_frames.py` is the older raw-frame
   capture tool. No GIF or video is in `docs/images/` yet. Still the single
   highest-value README addition.
7. **[OPEN, verified 2026-10-22: Pages not enabled]** Optional: enable
   GitHub Pages on `/docs` so `dashboard.html` / `mbse.html` are clickable
   pages rather than raw files.
8. **[OPEN, new 2026-10-22] Prune stale remote branches.** `git branch -r`
   lists 10 `origin/claude/*` branches (last commits 2026-08-19 to
   2026-10-19). Against the local remote-tracking refs, **all 10** have 0
   commits not on `main` (`git rev-list --count main..<branch>`), so every
   one is already merged and safe to delete. Re-run that count after a
   `git fetch` before deleting. Deleting remote branches is a push to the
   public remote, so it is the builder's call.

Note 2026-08-19: the hosted dashboard artifact now carries the MBSE view as
its own sheet (SHEET 5); the separate MBSE artifact link is superseded and
can be ignored. The emitted artifact build fails if `docs/mbse.html` is stale
(render order: `render_mbse.py` before `render_dashboard.py --artifact`).

Delete this file once the list above is done.
