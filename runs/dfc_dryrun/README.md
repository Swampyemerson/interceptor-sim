# DFC pre-purchase dry-run — VERDICTS (2026-10-21)

**Question (ADR-0125 / builder_queue `hailo8-purchase`):** does stock yolov8n
compile for the Hailo-8, in how many contexts, before the $110 HAT is bought —
and does the @1024 native-res variant (the 118° lens branch) fit too?

## Results

| input | compile | contexts | evidence |
|---|---|---|---|
| 640×640 | **OK** | **single context** (context 0/0, inter-context BW 0.0 Mbps) | `compile_640.log` |
| 1024×1024 | **OK** | **single context** | `compile_1024.log` |

Both: "Layers feasibility validated successfully" → "Successful Mapping" →
"Successful Compilation". `yolov8n.hef` in this dir is the @1024 build (the
loop overwrites the name; rebuild per size as needed — the venv persists).

**Purchase implication:** the recommended gate ("order after the dry-run
confirms single-context") is now SATISFIED, at BOTH input sizes — the 8
covers the native-res headroom branch as well, so no larger chip is needed
for any currently-planned scenario. Ordering remains the builder's call.

## Environment (hard-won — 11 defects fixed live, commits e4825e4..ec287b7)

- Compiler: `hailo_dataflow_compiler-3.34.0` wheel (3.x line = Hailo-8; 5.x
  REJECTS hailo8 — staff-confirmed, community.hailo.ai/t/18193).
- Model zoo v2.17 from GitHub (`--no-build-isolation`, after the DFC).
- **Must run on Ubuntu 22.04 / Python 3.10** (container `ubuntu:22.04`):
  on 24.04/py3.12 the optimizer SIGSEGVs at "Starting Model Optimization"
  (glibc 2.39; tcmalloc only silences the message) and MZ 2.17's pins
  (pillow<=9.3, scikit-image==0.20) don't build on 3.12.
- End nodes: the six head convs read from OUR onnx and passed via
  `--end-node-names` (ultralytics exports end in the post-head sigmoid).
- Calibration: 128 SYNTHETIC images (`calib_synth/`) — valid for the
  fit/context question ONLY; **no accuracy/recall number may be quoted from
  these builds** (runbook rule: real calibration comes from the deployment
  camera).
- `USER`/`LOGNAME`/`HOME` must be set (DFC allocator KeyErrors bare).

Repro: `docker run -e USER=root -v <repo>:/work ubuntu:22.04 bash -c
"apt ... && DFC_VENV=/work/.venv-dfc-2204 bash /work/scripts/nn/dfc_dryrun.sh
/work/hailo_dataflow_compiler-3.34.0-*.whl"` — see `scripts/nn/dfc_dryrun.sh`.
