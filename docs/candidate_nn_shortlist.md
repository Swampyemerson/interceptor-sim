# Candidate outdoor drone-detection NN shortlist (transfer bet)

*Sim-free scoping doc. No sim/PX4/Gazebo booted to produce this — desk research
(WebSearch/WebFetch + HF/GitHub API checks) plus one offline fetch+export+smoke-
inference test against already-captured frames in `demo_out/onboard_frames/`.
Companion to `docs/nn_transfer_plan.md` (the real-hardware architecture-family
plan, §2.2's licensing table) and `docs/seeker_nn_findings.md` /
`t18_nn_validation_notes.md` (what's already been tried in this repo). This doc
answers a narrower question than `nn_transfer_plan.md`: not "what architecture
should we eventually fine-tune," but "does a READY-MADE, permissively-licensed
drone-detection checkpoint already exist that we could drop in today and get a
free acquisition-range win, the way `drone_yolo11x` almost did?"*

## The honest one-line answer

**No pretrained model was found that is both (a) permissively licensed and
(b) trained on the correct viewing aspect (air-to-air / nose-on-approach,
not ground-observer-looking-up).** The closest aspect match (Det-Fly) is a
*dataset*, not a trained checkpoint — training something on it is a real next
step, not a fetch. The one NEW checkpoint fetched here
(`flying_objects_yolov8m`) is aspect-mismatched the same way `drone_yolo11x`
already was (ADR: "did not transfer" per `drone_detector_eval.py`), but it
adds a genuinely useful **labeled Bird class** this project's own training
data has never had — worth a cheap eval pass, not worth expecting a transfer
win from.

---

## 1. Shortlist table

| # | Candidate | License | Training-domain aspect | Input | Framework | ONNX? | Repo/URL |
|---|---|---|---|---|---|---|---|
| 1 | **`Javvanny/yolov8m_flying_objects_detection`** — **FETCHED this task** | **MIT** (weights; Ultralytics AGPL export tooling — same split as `drone_yolo11x`, see §4) | Real photos, DJI Phantom / Airbus A380 / Boeing 787 / seagull / pigeon — **mixed**, mostly airliner/airport and ground-observer-style shots, not nose-on quad approach | YOLOv8m, 1280² (exported) | Ultralytics YOLOv8 | **Exported here** (99.3 MB) | [HF](https://huggingface.co/Javvanny/yolov8m_flying_objects_detection) |
| 2 | `doguilmak/Drone-Detection-YOLOv11x` — **already in-repo** | MIT (weights) | Real photos, Kaggle "Amateur Drone Detection" set — mostly ground-observer-up | YOLOv11x, 1280² | Ultralytics | Yes, `weights/drone_yolo11x_1280.onnx` | [HF](https://huggingface.co/doguilmak/Drone-Detection-YOLOv11x) |
| 3 | `doguilmak/Drone-Detection-YOLOv8x` / `-YOLOv7` | MIT (GitHub-declared) | Same author, same Kaggle source dataset as #2 — same aspect gap | YOLOv8x / YOLOv7x | Ultralytics / YOLOv7 | Not checked (same dataset as an already-rejected candidate; low incremental value) | [GitHub](https://github.com/doguilmak/Drone-Detection-YOLOv8x) |
| 4 | **Det-Fly** (Zheng et al., IEEE RA-L 2021) | GitHub repo: **MIT**; a Roboflow-hosted mirror lists **CC BY 4.0** — **ambiguous, see §4** | **Air-to-air**, DJI Mavic2 filming another UAV: **36.4% front view, 32.5% top, 31.1% bottom** — closest aspect match to this project's nose-on terminal approach found in this search | N/A — **dataset only, 13k+ images, no checkpoint** | N/A | No | [GitHub](https://github.com/Jake-WU/Det-Fly) |
| 5 | Roboflow-hosted models trained *on* Det-Fly (`det-fly-jafoc`, "Det-Fly: Sunset 1/2/Sky") | Roboflow Universe page: **CC BY 4.0** | Same as #4 (air-to-air) | Unconfirmed (likely YOLOv8, per Roboflow convention) | Roboflow-trained | Unconfirmed — page returned HTTP 403 to the fetch tool; needs a Roboflow account/API key to confirm export formats | [Roboflow](https://universe.roboflow.com/detfly-iwauj/det-fly-jafoc) |
| 6 | **TransVisDrone** (Sangam et al., ICRA 2023) | **MIT** (GitHub API-confirmed) | Air-to-air, three source datasets: NPS-Drones (public), FL-Drones (**permission-gated**, not freely redistributable), AOT/Amazon Airborne Object Tracking (open, `registry.opendata.aws`) | Multi-**frame** stack (CSPDarkNet-53 + VideoSwin spatio-temporal transformer) — **not** a single-frame drop-in | PyTorch (YOLOv5-derived + VideoSwin) | Not confirmed; checkpoints via Google Drive, not HF/GitHub-hosted | [GitHub](https://github.com/tusharsangam/TransVisDrone) |
| 7 | `ZhaoJ9014/Anti-UAV` (official benchmark repo) | MIT (repo code) | RGB+IR tracking benchmark; datasets (Anti-UAV410 etc.) carry their **own** academic-use terms not verified here | Varies (tracker suite, not a single detector) | Mixed | Not checked | [GitHub](https://github.com/ZhaoJ9014/Anti-UAV) |
| — | `StephanST/unidrone` | **REJECTED — already logged** in `weights/LICENSES.md`: modified-MIT excludes defense/military use (conflicts with this project's counter-UAS framing); classes are ground objects, not drones | — | — | — | — | — |
| — | `YOLO-NAS` | **REJECTED — already logged** in `nn_transfer_plan.md` §2.2: pretrained weights need a paid commercial license | — | — | — | — | — |

Rows already covered elsewhere are marked so — this doc adds rows 1, 4, 5, 6, 7
that were not previously in `weights/LICENSES.md` or `nn_transfer_plan.md`.

## 2. What was fetched, and how to run it

`scripts/seeker/fetch_candidate_nn.py` — downloads `Javvanny/yolov8m_flying_objects_detection`'s
`best.pt` (49.6 MB, HEAD-checked and size-verified against the 2 GB ask-first
ceiling before any bytes move) and optionally exports it to ONNX at 1280²
(matching the `drone_yolo11x_1280` precedent — the target is a few-px body at
range; the default 640 letterbox destroys it, per `drone_detector_eval.py`).

```
# download only (any venv with network + no deps needed beyond stdlib):
.venv-seeker/bin/python scripts/seeker/fetch_candidate_nn.py

# download + ONNX export (needs ultralytics+torch -> .venv-seeker-train):
.venv-seeker-train/bin/python scripts/seeker/fetch_candidate_nn.py --onnx
```

**Already run as part of this task** (sim-free, per the hard rules): fetch
succeeded (49.6 MB, sha256[:16]=`d878939944007db4`), export succeeded
(`flying_objects_yolov8m.onnx`, 99.3 MB, 25.8 M params, output shape
`(1, 9, 33600)` — 4 box coords + 5 classes, confirming the label map), and a
raw `onnxruntime` smoke-inference against
`demo_out/onboard_frames/frame_000543.png` ran end-to-end with no errors
(output shape as expected; no crash, no shape mismatch). **This is not a
detection-quality eval** — it only proves the file loads and runs, the same
first checkpoint `drone_detector_eval.py` cleared before its real
acquisition-range comparison. Files land in `scripts/seeker/weights/`
(git-ignored, per `weights/LICENSES.md`'s existing convention):
`flying_objects_yolov8m.pt`, `flying_objects_yolov8m.onnx`,
`flying_objects_yolov8m.classes.txt` (label order: Drone, Airplane,
Helicopter, Bird, Background).

### Integration path into the existing harness

`finetuned_seeker.py` / `drone_detector_eval.py` already define the exact seam:
an ONNX single-class-or-multiclass YOLO run through `onnxruntime`, letterboxed
to a square `imgsz`, NMS'd, self-mask/horizon/aspect-gated (the same three
gates `seeker_nn_findings.md` derived for the untuned-COCO baseline apply
here too — this model was never trained on this project's own airframe, so it
has no reason to be immune to the same own-prop-arm false-lock failure mode).
Concretely, the fastest honest next step is a copy of
`drone_detector_eval.py` pointed at `flying_objects_yolov8m.onnx` instead of
`drone_yolo11x_1280.onnx`, filtering to **class 0 (Drone)** only (this model's
Bird/Airplane/Helicopter classes are backgrounds-of-interest here, not
targets — mapping them to "reject" doubles as a free multi-class hard-negative
filter compared to Ultralytics' single-class fine-tunes, which have no
adjudicated Bird class at all). **Not built as part of this task** — the hard
rules for this task are fetch + scope, not eval or claim a result.

## 3. Free public drone footage for the $0 kill-test

| Source | What it gives | License / access | Caveat |
|---|---|---|---|
| **Pexels / Pixabay / Videezy** | Stock video search for "drone", "fpv drone", "flying drone" — thousands of clips, royalty-free, most no-attribution | Free for commercial use per each platform's terms | **Disambiguate the search intent**: most hits are aerial-*POV* footage (shot *by* a drone looking down), not footage *of* a drone as the visible subject. Filter toward "drone flying in sky" / FPV freestyle / chase-cam clips where the drone itself is in frame — that's the actual positive class needed here. |
| **YouTube (Creative Commons filter)** | FPV freestyle and racing POV/chase footage, often has a second drone in-frame during formation/chase clips | Per-video CC BY license via YouTube's search filter | Spot-check each clip's actual license tag; YouTube's CC filter is self-reported by uploaders and not always accurate. |
| **Det-Fly** (row 4 above) | 13k+ real air-to-air frames, the correct aspect | MIT (GitHub) / CC BY 4.0 (Roboflow mirror) — **ambiguous, see §4** | Distribution is via OneDrive/BaiduNetdisk links in the README, not a direct HTTP fetch — a manual step, not scriptable here. |
| **Drone-vs-Bird Challenge** (`wosdetc/challenge`) | Purpose-built drone-vs-bird video, the exact WOSDETC challenge corpus cited in `bird_discrimination_design.md` | Free **after signing a Data Usage Agreement (DUA)** — not instantly $0-and-anonymous | Already flagged in `bird_discrimination_design.md`; the **YOLOv7-Segmented Drone-vs-Bird** Mendeley set (CC BY 4.0, no DUA, 20,925 imgs) remains this project's best already-adopted license fit for that same need — re-use it rather than re-deriving here. |

## 4. Licensing caveats — read before using anything above

- **"MIT weights, AGPL tooling" is a real pattern, not a one-off.** Both
  `drone_yolo11x` (already in `weights/LICENSES.md`) and the newly fetched
  `flying_objects_yolov8m` are weights the uploader marked MIT, produced by
  fine-tuning Ultralytics' AGPL-3.0 codebase. The MIT claim is the uploader's
  own metadata tag, not independently re-derived from a from-scratch clean-room
  training — treat it as "the weight artifact carries the uploader's MIT
  grant," not "this is provably free of any AGPL entanglement." Good enough
  for a sim-only portfolio eval; **re-verify before any real-product ship**,
  same disclosure `weights/LICENSES.md` already carries for `drone_yolo11x`.
- **Det-Fly's license is genuinely ambiguous, not just under-documented.**
  The GitHub repo carries an MIT `LICENSE` file (GitHub API-confirmed), but
  the README's own text never states a license for the *dataset content*
  itself (images/annotations hosted off-repo on OneDrive/Baidu) — an MIT
  LICENSE file in a code repo does not automatically extend to externally
  hosted image data. A third-party Roboflow re-upload separately tags it CC
  BY 4.0 (Roboflow's own default, not necessarily the original authors'
  grant). **Do not treat this as resolved** — if Det-Fly training ever
  becomes a real next step, email the authors (contact in the README) and
  get the license confirmed in writing before using it in anything beyond
  this project's own non-commercial portfolio sim.
- **Honesty boundary applies to any future eval, not just training.** None of
  this is guidance-facing yet — this task only fetched a checkpoint and ran a
  smoke test on already-captured frames. If/when a real eval is built (the
  `drone_detector_eval.py`-shaped follow-on in §2), it must read camera pixels
  only, exactly like every other seeker lane in this repo (CLAUDE.md's
  `gt_*` boundary) — `eval_tag_boxes.json`-style ground truth may only be used
  for *scoring*, never fed to the detector.

## 5. Frame-eval mAP does NOT predict in-flight recall — the standing caveat

Flag this explicitly, because this project has hit it before: a checkpoint's
own reported mAP (`flying_objects_yolov8m`'s training-domain accuracy,
`drone_yolo11x`'s 0.905 mAP50) describes performance **on its own training
distribution**, not on this project's sim-rendered onboard frames, and
**frame-level mAP has already been shown in this repo to not predict
in-flight recall** — the v3 seeker retrain history
(`docs/post-m5-roadmap.md` / ADR-0061) is the concrete precedent: a model
that looked fine on held-out eval frames still came back a **honest NULL** in
actual flight terms once measured end-to-end, and the v2→rebalance rounds
(`train_daemon_quad_rebal.py`, `merge_quad_v2.py`) show the same
static-mAP-vs-flight-behavior gap repeatedly. Any number this shortlist or a
future eval quotes for `flying_objects_yolov8m` (or any candidate here) is a
**frame-level, offline** number until it has been run through the same
detect-then-track + streak-gate harness (`markerless_loop.py`) the project's
own adopted seeker (ADR-0058) was validated through — do not upgrade a good
offline mAP or a clean smoke-test to "this seeker works" without that step.

## 6. Bottom line / recommendation

1. **No immediate transfer win expected** — same conclusion `drone_yolo11x`
   already reached, for the same underlying reason (aspect/domain mismatch).
   The fetched `flying_objects_yolov8m` is here for completeness and its
   Bird class, not because it's likely to out-detect the sim-native
   fine-tunes already deployed (ADR-0058).
2. **The real prize in this search is Det-Fly's aspect match**, not a
   checkpoint — if this project ever wants a genuinely closer-domain
   pretrained starting point than "COCO nano" or "ground-observer-up real
   photos," training on Det-Fly (once its license is confirmed in writing)
   is a more promising lever than searching for more ready-made checkpoints.
   That is a training-pipeline task, not a fetch — out of scope for this task.
3. **TransVisDrone is architecturally the closest published approach to this
   project's own detect-then-track design** (ADR-0058's two-stage,
   streak-gated architecture already does temporal reasoning ad hoc; TransVisDrone
   formalizes it with a spatio-temporal transformer) — worth reading the paper
   for design ideas even though its multi-frame-stack input means it is not a
   drop-in replacement for the current single-frame `SeekerDetection` interface.
