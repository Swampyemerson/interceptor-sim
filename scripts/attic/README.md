# scripts/attic/ — retired one-off tooling

One-off experiment / plot / probe scripts that outlived their analysis. Moved here
2026-07-25 (`git mv`, history preserved — `git log --follow scripts/attic/<file>`)
rather than deleted, so the analyses stay reproducible.

**Why:** each of these had ZERO references from any tracked file. They inflate the
tree a portfolio reviewer reads, and several are named like live verification tooling
(`verify_autocrop.py`, `verify_grid_labels_v3.py`, `phantom_forensics_v3.py`) — the
kind of thing someone re-runs and trusts even though nothing is wired to it, which is
this project's named defect class (CLAUDE.md, "instruments are evidence").

**Nothing here is wired to a gate.** Do not cite output from an attic script as
evidence without re-establishing that it still matches the code it measures.

| script | retired | what it did |
|---|---|---|
| `plot_t21_maneuver.py` | 07-09 | T21 maneuver trajectory plot |
| `wait_batch_string.sh` | 07-09 | batch-completion poller |
| `analyze_v2_ab.py` | 07-08 | seeker v2 A/B analysis |
| `plot_seeker_ab.py` | 07-08 | seeker A/B plot |
| `phantom_forensics_v3.py` | 07-09 | v3 phantom-detection forensics |
| `verify_grid_labels_v3.py` | 07-09 | v3 grid label check |
| `probe_v2.sh` | 07-11 | v2 probe driver |
| `verify_autocrop.py` | 07-12 | foveated auto-crop check (ADR-0074) |
| `prop_mask.py` | 07-20 | propeller-mask experiment |
| `onnx_smoke_test_aug.py` | 07-21 | nn_tier augmented-model ONNX smoke test |

## Deliberately NOT attic'd

The 2026-07-25 audit listed 12 orphans. Re-verification before the move
(scan corpus = ALL 626 tracked text files, not just `docs/ tests/ flight/ scripts/
.github/`) found **two of the twelve are still referenced**, so they stayed put:

- `scripts/bird_mc_harness.py` — cited by `models/fpv_bird_decoy/model.sdf:5,62` and
  `models/fpv_bird_decoy/model.config:12` as the harness those assets exist for.
- `scripts/gen_demo_ground_texture.py` — cited by `worlds/apriltag_demo.sdf:177`
  ("the generator, keep its ...") and `demo_out/README.md:148`.

Moving either would break a provenance pointer inside a committed world/model asset.
The audit's orphan scan simply did not read `models/`, `worlds/` or `demo_out/`.
