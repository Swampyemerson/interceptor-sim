# scripts/nn/ — markerless-seeker (NN) program toolkit

Program plan: `docs/nn_seeker_program.md` (ADR-0125). Toolchain:
`docs/hailo_toolchain_runbook.md`. Everything here runs on the dev machine
unless noted; `data/` is gitignored (scripts + manifests are the artifact).

| Script | What it does | Status |
|---|---|---|
| `fetch_aot_gt.py` | Stage 1: pull the AOT ground-truth JSONs (1.4 GB, license-clean) + small-target manifest | RUN (2.77 M boxes / 2.1 M frames) |
| `make_aot_pull_list.py` | Stage 2: stratified sample (size × range strata, per-flight cap) into a pull list | RUN (66,170 frames ≈ 30 GB, awaiting builder go) |
| `convert_detfly.py` | Det-Fly VOC→YOLO + grayscale + box-size histogram | ready (download is manual OneDrive) |
| `autolabel_tag.py` | Tag pose → cuboid projection → YOLO labels on REAL footage; masks the tag (noise/inpaint); counted rejects, fails loud on zero output. Reads both `frames_index.csv` and the deploy recorder's `frames/index.csv` | 13 tests green; SMOKED on runs/desk_20261004T205433Z via the Pi (13/13) |
| `sim_randomize.py` | Offline domain randomization of sim imagery (albumentations 2.x + sky gradient + NIR-leaning gray jitter), deterministic per seed | 13 tests green (shared test file) |
| `build_calib_set.py` | Hailo INT8 calibration set (≥1024 letterboxed frames from the deployment camera) | ready |
| `dfc_dryrun.sh` | Compile yolov8n @640 and @1024 for hailo8 with the DFC (no hardware needed) → context-count verdicts | ready; needs the builder's DevZone wheel |
| `test_nn_pipeline.py` | pytest for the two pipeline scripts | 13 passed |

Pipeline order (training step numbers from the program doc):
fetch/convert datasets → sim_randomize the sim set → pretrain (4070,
`.venv-seeker-train-gpu`) → fly tagged sessions → autolabel_tag →
fine-tune → TAGLESS validation day (recall by pixel-size bin) →
build_calib_set → dfc_dryrun/compile → Pi deploy.

Before trusting autolabel output: measure the target quad's bounding cuboid
with calipers in the TAG frame (x right, y up, z out of the face; 8 corners
JSON — see `autolabel_tag.py --help`) and eyeball one `--viz` overlay on a
real tagged-QUAD frame (the frame-convention check the tests could not
exercise).
