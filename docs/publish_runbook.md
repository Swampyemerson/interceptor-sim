# Publish runbook — first GitHub push (builder, ~10 minutes)

*Written 2026-07-20 (publish-prep pass, docs/next.md THE PLAN item 6). Everything
below is prepared; only the steps marked **(BUILDER)** need a human. Until a
remote exists the entire evidence base has zero off-machine backup — pushing
private is itself the win; flipping public can wait.*

## The 10-minute checklist

1. **(BUILDER) Create the remote — PRIVATE first.**
   On github.com: New repository → name e.g. `interceptor-sim` → **Private**
   → no README/license/gitignore (the repo has them). Or with the CLI:
   `gh repo create interceptor-sim --private --source . --remote origin`
2. **(BUILDER) Sanity-check the tree, then push.**
   ```bash
   git status                  # expect: clean, or only known in-flight work
   git remote add origin git@github.com:<user>/interceptor-sim.git   # if not created via gh
   git push -u origin main
   ```
   If `git status` shows modified/untracked files another session is still
   working on (e.g. anything under `scripts/seeker/` or
   `scripts/experiments/`), push anyway — push ships **commits**, not the
   dirty working tree. Do NOT `git add -A` to "clean up" first (standing
   rule: background workers may be mid-edit).
3. **Verify CI.** The push triggers `.github/workflows/ci.yml` (offline
   suite + dashboard drift check; the ONNX-parity stage is deliberately
   CI-skipped — see the comment block in the workflow). Expect plausible
   green; the pre-flagged first-run risks are the OSRF apt step and the
   `pupil-apriltags` wheel — the workflow already carries a fallback that
   deselects the four gz-importing test files rather than failing.
4. **Confirm the license posture.** `LICENSE` (MIT + the weights scope
   note) is the **recommendation, not a settled decision** — builder
   confirms MIT is what he wants his name on before the repo goes public.
   Weights decision (a/b/c) is in `docs/license_notice_weights.md`; with
   weights gitignored (verified below), option (a) is a no-op today.
5. **(BUILDER) Flip public — ONLY after re-reading the claims-scrub
   section below** and spot-checking README against it. Settings → General
   → Danger Zone → Change visibility.
6. **After the flip:** check the README renders (images, Mermaid-free plain
   diagram, dashboard link), and that the Artifact dashboard link in the
   README resolves for a logged-out viewer or is acceptable if it doesn't
   (it is a claude.ai artifact — treat it as a bonus view, `docs/dashboard.html`
   in-repo is the durable one).

## Claims scrub — the gate before "public" (BINDING)

These scopes were fought for across three audits (ADR-0066 constraint;
`docs/audit_findings_tracker.md` rows FWD-A1/A2/A5/A6, DEEP-P4/H3/H4;
ADR-0076). Any public-facing regeneration (README edits, a repo description,
a pinned gist, the GitHub "About" blurb) must preserve them:

- **"Works comms-denied" is HELD** (FWD-A1, ADR-0059) — everywhere,
  including the repo description and any resume bullet derived from it. The
  latch defeats a *post*-handoff jam; recovery from a *pre*-acquisition jam
  is an honest NULL. The coded-dash "no datalink to jam" story is
  architectural and unflown — claim the design, never the result.
- **No camera-guided sub-meter claims on the 3D quad target** (ADR-0076
  add #18g/#18h — the five mirages). Sub-meter numbers that ARE quotable:
  M3 standoff, M4 pro-nav @2 m/s (with the ADR-0009 gate-config selection
  disclosure attached), M5 median (AprilTag sensor, stated).
- **Pk 72/72 (95.0% CP-LB @2.8 m) is the flat-BILLBOARD result** and must
  say so; weave/12 m/s only, radius-explicit, never pooled (ADR-0064/0025).
- **r² = 0.96 is variance explained**, never "96% of the miss" — pair it
  with the 0.72 m / 1.69 m capacity bound (DEEP-H5).
- **Superseded numbers stay superseded** (DEEP-P10/FWD-A2): no ADR-0028/
  0030/0031 running-start or degraded-cue figures, no ADR-0029 27%-Pk map,
  quoted as current.
- **Jink 14/15 carries its paired-baseline caveat** (3/15 → 14/15,
  DEEP-P4); **0/155 carries "gross (>8 m)" + the one-empty-world scope**
  (FWD-A4/DEEP-P7); **CP bounds name their denominator** (DEEP-P8).
- The current README (rewritten this pass) already complies; the risk is
  future edits and *new* surfaces. When in doubt, `docs/project_state.json`
  wins over every prose doc.

## .gitignore audit (verified 2026-07-20, `git ls-files` + object sizes)

**Repo footprint:** 454 tracked files, `.git` ≈ 59 MB total — comfortably
inside GitHub norms. Largest tracked file: 22.4 MB (below).

**Intentionally tracked (do not "clean up"):**
- **Evidence CSVs in `logs/` — 99 files**, re-included by explicit `!`
  exceptions (M3/M4 gate runs, `mc_final_*`, `mc_t21_*`, `mc_realistic_*`,
  `mc_pk72_*`, `mc_adr0059_*`, `mc_uptilt_*`, the 16 per-tick r2 flights,
  and the `mc_coded_dash_*` arc). These make the README/ADR numbers survive
  a fresh clone (audit finding DEEP-H1/P-H4). One of them
  (`logs/mc_coded_dash_qv2_line9_conf10_rgate3_s123.csv`) shows local
  modifications at this writing — an in-flight experiment owns it; push
  commits as-is, reconcile that file in its own lane.
- `docs/images/` (26 files, ~3.3 MB) and the committed `plots/stereo_*`/
  `rig_*` design PNGs.
- `scripts/seeker/weights/LICENSES.md` — the provenance table is tracked;
  the weights themselves are not (next section).
- `.claude/` (agents, skills, settings) — the project's operating model is
  part of the portfolio story. **Flag:** `.claude/settings.local.json` is
  tracked; it is a machine-local permissions file and will read as noise in
  a public repo. Harmless content-wise (verify once before the public
  flip), but a candidate to gitignore in a future housekeeping pass —
  builder's call, not blocking.

**Must never be pushed (verified currently ignored / absent from git):**
- **Detector weights** — `scripts/seeker/weights/` (except LICENSES.md),
  `yolo11n.pt` at root, any `*.onnx`/`*.pt`: `git ls-files` confirms **zero
  weights files tracked**. Licensing: `docs/license_notice_weights.md`
  (AGPL-derivative risk). If a weights file is ever wanted in a release,
  re-open that doc's builder decision first.
- The venvs (`.venv*`), `logs/*` bulk, `demo_out/*` media,
  `scripts/seeker/{data,out,eval_out,twostage_out,runs}/`, PX4/Gazebo
  caches — all covered by the existing `.gitignore`.

**git-LFS ruling — the ~25 MB `models/fpv_quad_enemy` meshes: KEEP IN PLAIN
GIT, no LFS.** The flagged candidate is
`models/fpv_quad_enemy/meshes/NXP-HGD-CF.dae` (22.4 MB) + `CF.png` (1.5 MB)
— tracked since 2026-07-11 (the ADR-0072 3D-quad realism fix). Reasoning:
(1) 22 MB is far under GitHub's 100 MB hard limit and under the 50 MB
warning threshold; total repo ≈ 59 MB is unremarkable. (2) The mesh is
load-bearing for reproducing every quad-target run — LFS would make a plain
`git clone` incomplete for anyone without LFS installed and adds a bandwidth
-quota failure mode for a portfolio repo cloned by strangers. (3) The file
is write-once (it has one version; LFS's benefit is churn on big files —
there is none). Revisit only if large binary *churn* ever appears.

## Known-open items this runbook does not solve

- **First CI run may need one dep-pin iteration** (pre-declared in the
  workflow comments since 2026-07-10). Fix forward in `ci.yml`, don't
  weaken `run_tests.sh`.
- The hosted dashboard Artifact URL and the in-repo `docs/dashboard.html`
  must keep matching `project_state.json` (the drift check enforces the
  local pair; the Artifact republish is the session ritual, CLAUDE.md).
- Branch protection / repo description / topics: cosmetic, after the flip.
