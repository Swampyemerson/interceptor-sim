# `weights/` — the integrity record for the DEPLOYED detector weights

The weight files themselves are **never committed** (`.gitignore:43
scripts/seeker/weights/`; licensing rationale in `docs/license_notice_weights.md`).
This directory holds the one thing that *can* be committed: their **sha256**.

## Why (audit 2026-07-25, completeness critic, "off-machine backup")

`flight/deploy/seeker_loop.py` flies `scripts/seeker/weights/nn_tier/n-mono.onnx`
by default. 18.5 GB of weights + training corpus live on ONE disk, all gitignored,
with no backup script and — until this file — **no checksum anywhere in the repo**
(the only hash in the tree was an md5 for the retired `ground_v1` in
`docs/decisions.md:1274`). A corrupted or silently swapped `n-mono.onnx` was
therefore undetectable, and the data that could regenerate it sits on the same
disk. A model is a MEASUREMENT INSTRUMENT here: every detection number traces
through it, so an unverifiable model quietly invalidates every number downstream.

## `DEPLOYED.sha256`

Plain `sha256sum` format, paths relative to the repo root, so it verifies with the
standard tool and needs no bespoke parser:

```
sha256sum -c weights/DEPLOYED.sha256      # run from the repo root
```

| file | role |
|---|---|
| `scripts/seeker/weights/nn_tier/n-mono.onnx` | **THE FLIGHT DEFAULT.** Real-data model (YOLO11n, COCO-init, grayscale-native), AP50 0.442 / recall 44.2% on the source-disjoint held-out set (n=4175). This is what `seeker_loop.py` loads with no `--weights`. |
| `scripts/seeker/weights/drone_finetuned_quad_v2.onnx` | The sim-trained HISTORICAL BAR, kept selectable via `--weights`. AP50 0.0003 / recall 1.1% / false-fire 88.5% on the same real held-out set — **must never be the hardware default** (`seeker_loop.py` header). |

Recorded 2026-07-25. `tests/test_deployed_weights.py` re-verifies the flight
default against this manifest whenever the file is present on the machine, and
pins `seeker_loop.py`'s default basename so the manifest cannot silently describe
a model that is no longer the one that flies.

**Regenerate deliberately** (an intentional model swap is an ADR-level decision,
so this is not automated):

```
sha256sum scripts/seeker/weights/nn_tier/n-mono.onnx \
          scripts/seeker/weights/drone_finetuned_quad_v2.onnx \
  > weights/DEPLOYED.sha256
```

**Still open** (not solved by this file): the weights and the 15,391-image corpus
have no off-machine copy. `docs/next.md` and `docs/publish_runbook.md` describe the
private GitHub remote as "instant off-machine backup of the whole evidence base",
which is false — git holds none of it (`git ls-files '*.onnx' '*.pt'` = 0).
