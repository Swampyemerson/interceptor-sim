# Deletion / retirement candidates — 2026-10-22 (overnight scout)

Ops duty #3 ("scope grows silently — propose deletions"). READ-ONLY scan;
nothing was deleted. Each row: why it's dead, the superseding pointer, live
inbound references (from CLAUDE.md / ops.md / goals / next / progress /
contract / scripts), and a recommendation. Ranked by review-load saved.
Prior art: `docs/doc_hygiene_2026-09-17.md` (archived ADR-0001–0080; its
method is reused here). **The builder decides; the head executes per row.**

## Tier 1 — docs that cost review attention and serve no live surface

| # | path | why dead | superseded by | inbound | rec |
|---|---|---|---|---|---|
| 1 | `docs/linkedin_copy_draft.md` | one-off social copy, GitHub-skip ruled | the real README/portfolio | 0 | **DELETE** |
| 2 | `docs/audit_targets.md` | one-shot audit prep for a 2026-07-06 branch, long merged | the audit's own results in ADRs | 1 (a comment citing its *pattern*) | **DELETE** (fix the comment to cite the ADR) |
| 3 | `docs/fusion_capstone_design.md` | sim-phase design for fusion that A/B'd NO-TRANSFER, default-OFF (ADR-0017 era) | contract stage note + `flight/range_fusion.py` docstring | 3 (historical mentions) | **ARCHIVE** to state_archive |
| 4 | `docs/compute_setup.md` | ADR-0012/0015-era (sim-phase) compute notes; the real stack is ruled in the contract (Pi 5 + Hailo-8, ADR-0125) | contract `bom_tiers` + `nn_seeker_program.md` | 2 | **ARCHIVE** |
| 5 | `docs/ekf_design_brief.md` | 797 lines, "DESIGN ONLY, gated" since the sim phase; the gate never opened and the chase ruling made it moot | ADR-0103 chase-only; contract | 3 | **ARCHIVE** (keep a 5-line stub pointing at the archive) |
| 6 | `docs/scoring_fix_plan.md` | 514 lines; its own header says §2 is corrected by the 2026-08-10 re-score; plan fully executed | `docs/rescore_2026-08-10.md` (the record) | 6 — *refs are mostly the rescore doc + ADRs citing history* | **ARCHIVE**, re-point the 6 refs |
| 7 | `docs/candidate_nn_shortlist.md` | sim-era NN shopping list; the real-data program + ADR-0125 rulings replaced it | `docs/nn_seeker_program.md`, `docs/nn_tier/PLAN.md` | 5 (contract build_plan history lines) | **ARCHIVE**, keep contract wording as history |

## Tier 2 — memory files (head applies edits; builder nod sufficient)

51 memory files; these are marked-superseded or era-dead but still load into
every session's recall surface:

| path (memory/) | status | rec |
|---|---|---|
| `no-sim-pivot-fix-in-place.md` | header says SUPERSEDED 2026-09-17 | DELETE (new-sim-directive carries it) |
| `venv-seeker-train-cpu-only-torch.md` | superseded by `gpu-training-unlocked` (marked) | DELETE |
| `t25-video-must-feature-tilt.md` | T25/sim-demo era closed by the real-build pivot; tilt ruling lives in ADRs | DELETE after builder confirms the T25 video is dead |
| `markerless-maneuver-fix-directive.md` | marked SATISFIED (d10c9ca) | DELETE (post-M5 roadmap memory overlaps) |
| `adaptive-camera-tilt-idea.md`, `foveated-autocrop-higher-res-idea.md`, `bank-as-accel-cue-idea.md` | sim-era ideas; bank-cue + foveated are partially alive in the NN program docs | CONSOLIDATE into one "parked sim-era ideas" memory with pointers |
| `post-m5-roadmap.md`, `perception-design-pivot.md` | sim-phase status snapshots, all content now in contract/ADRs | DELETE |

## Tier 3 — bulk (disk, not review load; policy proposals, not per-file)

- `logs/` = **2.5 GB, ~12.8k files** (188 `*_stdout` logs). Policy proposal:
  (a) NEVER prune a file referenced by any `evidence`/provenance pointer in
  the contract, a findings.md, or an ADR — provenance must stay resolvable;
  (b) tar-and-move to `bench_archive/` any fleet whose verdict was RETRACTED
  or superseded (the ADR-0076 mirage list is the index); (c) `*_stdout` logs
  of archived fleets go with them. Estimated recovery ~1–1.5 GB.
- `runs/` = 331 MB — fine; no action.
- `.venv-dfc-2204` = **9.6 GB** + `hailo_dataflow_compiler-3.34.0-*.whl` =
  524 MB (both gitignored). KEEP BOTH until the day-1 HAT benchmark passes
  (the venv is the working compile env; repro recipe is in
  `runs/dfc_dryrun/README.md`); then delete the venv, keep the wheel.

## Tier 4 — dead levers (code + tests; delete only with an ADR closing them)

- `pip_aim` / `look_angle_max_deg` (isim/concepts.py:234): registered NULL
  (chase won every cell) with **no explicit keep-in-tree ruling** found —
  candidate to delete lever + tests with a closing ADR line. ~Moderate code.
- `--range-fusion` / `flight/range_fusion.py`: fusion A/B = no-transfer,
  default-OFF since the sim phase; wired into flight code paths. Deleting
  touches the flight tree — propose ONLY if the builder wants the flight
  surface slimmed before hardware; otherwise mark "retired, do not tune."
- NOT candidates (explicit in-tree rulings): commit-sprint lever
  (ADR-0119 "lever + tests in-tree"), stern-geometry knobs (kept for future
  regimes, 2026-10-22), `--pursuit-stamped-age` (validated, pending
  recommendation), detect-then-track for the quad (graveyarded as
  do-not-resurrect but its removal was not ruled).

## Not touched / out of scope

`docs/decisions.md` (live ledger), `next_archive.md`/`decisions_archive.md`
(they ARE the archive), `bird_discrimination_design.md` +
`detection_tracking_methods.md` + `quad_target_retrain.md` (contract-
referenced design records; archive only with contract re-pointing),
`doc_hygiene_2026-09-17.md` (prior art for this file — though once this
file's actions land, both could merge into one hygiene log).
