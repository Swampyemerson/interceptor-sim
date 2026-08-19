# interceptor-sim: pre-application cleanup work order (v2, amended 2026-08-19)

Paste this file to a Claude Code instance opened in the repo root. Execute in
order. Goal: an engineer skimming for 3–10 minutes hits the results, the honesty
machinery, and the MBSE layer first. Keep all claims exactly as they are; this
is packaging, not content. **Binding companion: the claims-scrub section of
`docs/publish_runbook.md` — every public-facing edit must preserve those scopes.**

**The safety net (use it, every phase):** after each phase, run
`scripts/run_tests.sh` (850 tests incl. the AST no-cheat audit + the
`render_dashboard.py --check` drift gate) and require green before committing.
After every file move, `git grep -n "<old-path>"` and fix every hit. Commit per
phase with a descriptive message. This work order's own last step is to delete
this file.

## P0: secrets + truth sweep (before the link goes on any application)

1. Delete the plaintext credential line in `SETUP-GUIDE.md` (§"Handy things to
   know": `user emerson, password interceptor123`).
2. Emerson: change that Linux password on the VM itself regardless of git.
3. History handling: **Option A (recommended, and sufficient): rotate the
   password, leave history alone.** Option B (`git filter-repo --replace-text`
   + force-push) is NOT recommended: it rewrites every commit hash, and the
   docs/contract cite **≥39 specific commit hashes as evidence provenance**
   (ADR evidence lines, `docs/rescore_2026-08-10.md`, the retraction commit
   `b47ce32` referenced from the README) — all would dangle. Likewise KEEP the
   `Claude-Session:` trailers: P2 step 15 discloses the AI-built workflow
   anyway, and stripping trailers both undercuts that disclosure and forces the
   history rewrite you otherwise avoid.
4. `/home/emerson` paths: the original 4-file list is incomplete — the string
   also appears in `worlds/*.sdf` usage comments (7 files), several scripts
   (`scripts/deploy/qemu_arm_check.sh`, `scripts/seeker/*`,
   `scripts/forensics/*`), and as data columns (`flight_csv_path`,
   `sim_log_path`, `run_log_path`) in MANY committed `logs/*.csv`, not just the
   uptilt four. It is a username (already public via the GitHub handle and
   commit author), not a secret — so either **accept it everywhere** (fine), or
   strip it **uniformly with a script** (`/home/emerson/interceptor-sim` →
   `~/interceptor-sim` across tracked text files), never by hand-editing four
   CSVs. If stripping: evidence CSVs are consumed by gate scripts — run
   `scripts/run_tests.sh` plus `scripts/check_t21.sh` after, and require green.
   Do still fix `docs/review2_silent_failure_findings.md` (~line 126) either way.
5. **(NEW) Stale-claim sweep — the 12/16 retraction did not reach every
   surface.** `PROGRESS.md` (bench row) still says "re-scored, the same flights
   go 0/16 → 12/16 … measurement artefact"; `docs/project_state.json`
   `key_numbers` still carries "Inside the contact radius (UNRESOLVED) 3-12/16"
   and `now.next` still says "Land the scorer fix (#8)". All are superseded by
   commit `b47ce32` + `docs/rescore_2026-08-10.md` (correct: **3/16 logged →
   5/16 interpolated**, PRIMARY centre-interpolated ruler; the "+0.208 m lens
   lever" was landing-gear height; the #8 patch was rejected, `d33d650`). Fix
   PROGRESS.md and the contract, then `python3 scripts/render_dashboard.py`,
   republish the Artifact to the stored `artifact_url`, and commit. A public
   reader cross-checking PROGRESS against the README's retraction note would
   find this contradiction in minutes.
6. **(NEW)** Run a secret scan (gitleaks/trufflehog, or GitHub secret scanning
   once pushed) before flipping public. Manual sweep found nothing else live
   (field scripts are SSH-key based with `<PASSWORD>` placeholders; the
   regulatory site-capture address fields are blank templates).

## P1: root de-clutter (moves only, no logic changes)

Known inbound references to update per move (beyond README/dashboards):

7. Shrink `CLAUDE.md` to ≤40 lines (build/test commands, the honesty-boundary
   rule, the project-state-contract pointer + update ritual, evidence
   conventions). Move the model-routing/autonomy/orchestration body to
   `.claude/ops.md`, import with `@.claude/ops.md`. Remove the PC hardware
   details; the `Downloads\...` personal path lives in **GOALS.md** ("Deep
   context") — strip it there. **The `@GOALS.md` import (currently CLAUDE.md
   line ~340) must change to the new goals path in the same commit.** Sequence
   this step LAST in P1 — CLAUDE.md is the operating contract for the very
   session doing this work.
8. Move `KICKOFF-PROMPT.md`, `SESSION-PROMPT.md` → `.claude/`. Update:
   `.claude/commands/go.md` (references KICKOFF-PROMPT.md), `SETUP-GUIDE.md`.
9. Move `PROJECT_LOG.md`→`docs/build_log.md`, `PROGRESS.md`→`docs/progress.md`,
   `GOALS.md`→`docs/goals.md`, `NEXT.md`→`docs/next.md`. Update the functional
   consumers: `.claude/commands/go.md` (reads NEXT/PROGRESS/GOALS),
   `.claude/skills/sim-milestone/SKILL.md`, `.claude/skills/pronav/SKILL.md`,
   `.claude/skills/run-interceptor-sim/SKILL.md`,
   `.claude/agents/council-member.md`, `.claude/agents/sonnet-worker.md`,
   README, CLAUDE.md. ~30 scripts mention these files in comments — a single
   sed sweep, harmless either way. The drift check does not read them (safe).
10. Move `Launch Interceptor Sim.bat` + `bootstrap.sh` → `scripts/env/`;
    `camera_intrinsics.json` → `configs/`. Updates: `bootstrap.sh` is referenced
    by README (~line 331), `scripts/field/common.sh:107`,
    `scripts/field/selftest.sh:47`, and the run-interceptor-sim skill.
    `camera_intrinsics.json` is referenced by ~20 files (mostly
    comments/docstrings, some CLI defaults) — `git grep -n camera_intrinsics.json`,
    fix every hit, tests green. The .bat contains no repo paths (it is a
    VirtualBox/ttyd launcher) but **Emerson's Windows desktop shortcut may
    target the repo copy — re-point it after the move (needs him at the PC)**.
11. Delete `demo_out/retired_2026-08-10/`; move `demo_out/README.md` →
    `docs/demo_pipeline.md` cut to the ~40 current-pipeline lines; delete
    `demo_out/`. Then update `.gitignore` lines ~72–88 (the demo_out
    keep/ignore block, incl. the `retired_2026-08-10` exceptions). The demo
    scripts (`build_demo.py`, `compose_demo.sh`) create their output dirs and
    keep working.
12. Move `scripts/attic/` → `scripts/experiments/attic/`. One inbound doc ref:
    `docs/audit_2026-07-25_whats_left.md`.
13. **(NEW)** Gitignore `.claude/settings.local.json` (machine-local
    permissions; flagged in the publish runbook). Keeping the rest of
    `.claude/` public is a made decision (runbook: "part of the portfolio
    story") — leave it tracked.

## P2: README repackaging (keep every claim, change the delivery)

14. Add at top: one plain-English paragraph, then the image row
    `docs/images/m5_traj_overlay.png`, `docs/images/hud_overlay_sample.png`,
    `docs/images/m5_pk_vs_radius_by_arm.png` (all verified present and
    caption-honest today).
15. Cut README ~60%: move per-row caveat prose into a linked
    `docs/results_notes.md`. Keep verbatim: the results table, "What is proven,
    and what is not", the architecture block, the Reproduce section — **and
    (amended) the two boxes that differentiate this repo: the "Comms-denied
    status: HELD" box and the Rescoring/retraction note ("an earlier version
    claimed 12/16, and that was wrong")**. No test reads the root README
    (verified) — cutting is safe.
16. Replace the `claude.ai/code/artifact/...` front-door link with in-repo
    links; remove it from the NEXT/PROGRESS headers. **Do NOT remove
    `artifact_url` from `docs/project_state.json`** —
    `tests/test_dashboard_guard.py` requires the key and the republish ritual
    depends on it. The link also appears in PROJECT_LOG.md,
    `configs/px4_6cmini/README.md`, `docs/next_archive.md` — sweep or accept
    (deep docs may keep it).
17. Add a "Systems engineering" subsection: commit PNG screenshots of the
    rendered dashboard + MBSE views to `docs/images/`, link
    `docs/project_state.json`, `docs/mbse.html`, `scripts/render_mbse.py`.
    Optionally enable GitHub Pages on `/docs` so the two HTML views are
    clickable (both are self-contained; Pages just serves the already-public
    dir).
18. Add the three-sentence "Built with AI, disclosed" note linking
    `docs/build_log.md`.

## P3: polish

19. Add the CI badge for `.github/workflows/ci.yml` (exists, verified).
20. Re-shoot the hero GIF with the corrected net-vs-ram caption (the
    `fuse_banner()` fix is in `scripts/build_demo.py`, verified; retired assets
    were never replaced). **Needs the sim machine (PX4/Gazebo) — schedule as a
    home-PC task, not doable from a cloud/cleanup session.** A short seeker-POV
    handoff-to-intercept loop atop the README is the highest-value addition.
21. Add a 60-second no-sim verification block to the README:
    `pip install -r requirements.txt`, then `scripts/run_tests.sh` and
    `scripts/check_t21.sh` re-assert headline numbers from committed CSVs with
    no PX4/Gazebo install.
22. Verify `.gitignore` coherence after the demo_out removal (see step 11).
23. **(NEW)** Repo storefront: set the GitHub About description + topics using
    runbook-compliant wording (claim the design, never the unflown result);
    delete stale merged `claude/*` remote branches; confirm LICENSE posture per
    runbook step 4 (MIT + weights scope is a recommendation the builder
    confirms); spot-check the README render (images, links) after the flip.

## Explicitly keep

Committed `logs/` CSVs (the evidence base), the ~25 MB target mesh (needed sim
asset, plain-git ruling in the runbook), the graveyard and retraction records,
the honesty test machinery (`tests/`, `flight/tests/`, `run_tests.sh`,
`render_dashboard.py --check`), `.claude/` (minus settings.local.json), and the
contract → dashboard → MBSE chain. These are the differentiators.

## Done?

`scripts/run_tests.sh` green; `git grep` clean for every moved path; PROGRESS/
contract/README all agree with `docs/rescore_2026-08-10.md`; artifact
republished; then delete this file in the final commit.
