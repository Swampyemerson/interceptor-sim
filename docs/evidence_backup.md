# Evidence backup & deployed-weights checksums

**Created 2026-07-25** (audit: "no test pins the deployed weights"; off-machine-backup
gap). Purpose: make the two things that are NOT in git identifiable and recoverable —
(1) *which* weight file is actually deployed, by content hash, and (2) how much
irreplaceable evidence lives on exactly one disk.

Everything here traces to a command run on this machine on 2026-07-25; re-run the
commands to re-verify.

---

## 1. Deployed-weights checksums

```
sha256sum scripts/seeker/weights/nn_tier/n-mono.onnx \
          scripts/seeker/weights/drone_finetuned_quad_v2.onnx
```

| file | sha256 | bytes | mtime |
|---|---|---|---|
| `scripts/seeker/weights/nn_tier/n-mono.onnx` | `015a5e4c9289da0b1c776d6fcceacdb5e1a8562c6fed6a1fb8cef5f40c8089dd` | 10,604,059 | 2026-07-21 00:57 |
| `scripts/seeker/weights/drone_finetuned_quad_v2.onnx` | `43d501ecffdaa56cc406072619a3c37b5df6860d9bb57bb2813bea99b61386f8` | 10,604,064 | 2026-07-11 21:09 |

The rest of the `nn_tier` sweep family, for completeness (same command):

| file | sha256 | bytes |
|---|---|---|
| `nn_tier/n-mono-aug.onnx` | `197c3ac728aad2511cbea7f2958d2415d42f5892f2adf5af9150f13655ac7cf6` | 10,604,059 |
| `nn_tier/n-color.onnx` | `0127d3ec540f8aa7a6a948ce7b053574c9eb1373cfcff0d3cbd647fa69b4f58a` | 10,604,065 |
| `nn_tier/s-mono.onnx` | `e779d32e70379d70fec7774e306612bfce632dd6829ffcd19279a12ecb4e71bd` | 37,927,763 |

Note the near-identical byte counts across the yolo11n family — **size does not
identify a model here**, only the hash does. That is the whole reason this table
exists.

### OPEN: the deployed-weights name drift (for the contract owner)

The code and the docs disagree about which model is deployed:

| surface | says |
|---|---|
| `flight/deploy/seeker_loop.py:1254-1255` (argparse **default**) | `scripts/seeker/weights/nn_tier/n-mono.onnx` |
| `flight/deploy/seeker_loop.py:14-17` (header) | n-mono, "quad_v2 is BLIND on real imagery" |
| `flight/deploy/README.md:109,119,129` | `drone_finetuned_quad_v2.onnx` |
| `docs/project_state.json:219` (`nn_seeker` role) | `drone_finetuned_quad_v2` |
| `docs/project_state.json:423` (`active`) | `drone_finetuned_quad_v2.onnx @640, conf 0.25` |

`flight/deploy/real_flight.py` does not define its own default — it composes
`seeker_loop.SeekerGuidance` (`real_flight.py:134`, arg group at `:2331`), so it
inherits **n-mono**.

So the flown default is `n-mono.onnx`, and the contract + deploy README still name
`quad_v2` — the model the contract's own `nn_tier` note (`project_state.json:670`)
records as a **confirmed NULL on real imagery** (AP50 0.0003, recall 1.1%, false-fire
88.5%, n=4175). The docs point at the model the project measured as blind. This needs
a contract + `flight/deploy/README.md` edit by their owner; it is deliberately NOT
fixed here.

---

## 2. Off-machine-backup gap

Measured 2026-07-25 (`du -sh`, `git ls-files | du -ch --files0-from=-`):

| what | size | in git? | replaceable? |
|---|---|---|---|
| **tracked repo** (630 files) | **45 MB** | yes | yes — and pushed to `origin` |
| `scripts/seeker/data/` real-media corpus | **17 GB** | no | **hard** — curated source/video/scene-disjoint splits + hand-curated negatives; the sources are public but the split + labels are the work |
| `.venv-seeker-train-gpu/` | 7.1 GB | no | yes — reinstallable (torch cu126) |
| `demo_out/` | 867 MB | no | yes — re-renderable |
| `scripts/seeker/weights/` | 829 MB | no | **hard** — GPU-hours; §1 hashes identify them |
| `scripts/seeker/runs/` (train artefacts) | 399 MB | no | no — the training-run record |
| `logs/` | 379 MB | no | **no** — this is the measured evidence behind every number in the contract |
| **repo dir total** | **31 GB** | — | — |

**The gap:** ~18.5 GB of corpus + weights + run logs exists on **one disk** (`/dev/sdd`,
910 GB free) with **no off-machine copy**. `origin` (private
`Swampyemerson/interceptor-sim`) holds the 45 MB tracked half only — i.e. the code and
the claims are backed up, but **the evidence the claims rest on is not**. A disk loss
would leave a repo full of numbers whose provenance ("numbers trace to a run",
CLAUDE.md) could not be re-established, and would cost GPU-hours to partially rebuild.

Ranked by value-per-byte if only some can be backed up:

1. `logs/` (379 MB) — irreplaceable, small, backs every published number.
2. `scripts/seeker/weights/nn_tier/` (~70 MB for the four sweep models) — GPU-hours,
   and §1's hashes are meaningless without the files.
3. `scripts/seeker/runs/` (399 MB) — the training record.
4. `scripts/seeker/data/` (17 GB) — the expensive one; the *split manifests* alone are
   tiny and are the part that is genuinely hard to reproduce.

Items 1-3 total **under 1 GB** — the high-value 95% of the risk is cheap to close.
No action taken here; this is a record of the gap, not a backup.

---

## 3. Why there is no test pinning the hash

A pytest that asserts a weights sha256 would fail on every clean clone and in CI,
because `scripts/seeker/weights/` is gitignored (`.gitignore:43`) — see
`docs/license_notice_weights.md` for why the weights are never committed. Pinning the
hash in a test would therefore either be vacuously skipped (which
`docs/error_handling_policy.md` forbids as a green-that-did-not-run) or permanently
red. This document is the pin instead: it is tracked, it is diffable, and a model swap
that does not update it is visible in review.
