# License notice: repo code vs. detector weights

Written 2026-07-10 as part of publish-prep; updated 2026-10-22 for the public
repo (public since 2026-08-29) and the current weights roles. Not legal advice:
a factual note on a licensing nuance this repo carries.

## The split

The repo's own code (Python, scripts, docs) is **MIT** (see `LICENSE`,
copyright 2026 Emerson; MIT confirmed by the builder 2026-08-19). That covers
everything *we wrote*.

It does **not** cleanly cover the neural-net detector weights the markerless
seeker uses. The ones that matter today:

| File (under `scripts/seeker/weights/`) | Role | Lineage |
|---|---|---|
| `nn_tier/n-mono.onnx` | flight default (`flight/deploy/seeker_loop.py`, pinned by `tests/test_deployed_weights.py`) | YOLO11n, COCO-initialised, trained on the real-photo corpus with Ultralytics tooling (`docs/detection_tracking_methods.md` §B.2) |
| `drone_finetuned_quad_v2.onnx` | sim seeker / historical bar | YOLO11n fine-tuned from `yolo11n.pt` on Gazebo-rendered quad frames (`docs/quad_target_retrain.md`, `scripts/seeker/train_daemon_quad_v2.py`) |
| `yolov8n` | target model of the Hailo-8 NN seeker program (ADR-0125) | Ultralytics YOLOv8n, COCO-pretrained |

YOLO11n and YOLOv8n and their training tooling ship from Ultralytics under
**AGPL-3.0** (`scripts/seeker/weights/LICENSES.md`). A model fine-tuned with
AGPL-licensed tooling from an AGPL base is plausibly a **derivative work**: the
*weights file itself* is not something we can unilaterally relicense MIT just
because our training script is ours.

(The older `drone_finetuned_v2.onnx` this note originally named was the sim-era
deployed weight, ADR-0061; it is no longer the default anywhere in flight code.)

## Current state (checked, not assumed, 2026-10-22)

`scripts/seeker/weights/` is **entirely git-ignored** (`.gitignore`:
`scripts/seeker/weights/`, plus explicit ignore lines for root and in-dir
`yolo11n.pt`). Only `LICENSES.md` (the provenance table) is tracked;
`git ls-files` finds **no `.onnx`, `.pt` or `.hef` file anywhere in the repo**.
So although the repo is public, **no weights file is distributed** through it,
and none is attached as a release asset. The AGPL question below is about what
happens *if/when* that changes (shipping a release asset, un-ignoring a file for
reproducibility, or publishing a compiled Hailo model), not a problem in the
repo today.

`docs/nn_transfer_plan.md` names the clean long-term fix: for a public or
deployable artifact, prefer an **Apache-2.0** model family (**YOLOX** or
**NanoDet-Plus**) fine-tuned in-domain instead of the AGPL Ultralytics line.
That is a real retrain, not a relabel.

## BUILDER DECISION (still open): pick before publishing any weights file

- **(a) Keep weights out of git (status quo) or, if published, ship them
  with an explicit AGPL-3.0 notice scoped to that one file.** Cheapest,
  honest, matches how the repo already documents provenance
  (`LICENSES.md`). Code stays MIT; the weight artifact is separately and
  correctly labeled.
- **(b) Move weights to a release asset (or git-LFS) with its own license
  file**, keeping the main MIT'd source tree free of any AGPL artifact.
- **(c) Retrain on an Apache-2.0 base (YOLOX/NanoDet-Plus)**, so code and
  weights are both permissively licensed with no exception. This is the
  `nn_transfer_plan.md` recommendation for the real hardware system; note it
  interacts with ADR-0125's choice of yolov8n for the Hailo program.

**Status:** the repo is public and **(a) holds by default**: every weights
file is git-ignored, so nothing AGPL is distributed. Re-decide the moment any
weights file is to be published (release asset, un-ignored file, or a
compiled Hailo model): add the per-file AGPL notice then, and weigh (c) if a
clean Apache-only story is judged worth the retrain (real cost: see
`nn_transfer_plan.md`'s YOLOX/NanoDet-Plus discussion).
