# Markerless NN seeker — Stage-B prototype findings (ADR-0033)

Offline prototype of the **pre-built nano-NN** markerless-seeker lane from
`docs/seeker_design_brief.md` (sec 2.2 / 6, Stage B). Detects the target drone's
**body** (no fiducial) and emits the same bearing/range/confidence the guidance
already consumes. Built and evaluated **offline against pre-captured frames only**
(`demo_out/onboard_frames/`, 867×1280×960) — no sim/PX4/Gazebo was booted.

## What was built
- `scripts/seeker/nn_seeker.py` — `NNSeeker` + `SeekerDetection` (field-compatible
  with `m3_static_intercept.Measurement`; `as_measurement_tuple()` is the drop-in
  seam). Model: **YOLOv8n**, COCO, ONNX, run on CPU via **onnxruntime**.
- `scripts/seeker/eval_nn.py` — detection rate, latency, bearing quality, and the
  body-not-tag proof. Saves annotated frames to `scripts/seeker/eval_out/`.
- Isolated venv `~/interceptor-sim/.venv-seeker` (onnxruntime 1.27.0,
  opencv, numpy). The project `.venv` was never touched.

## Model (honest numbers)
| | value |
|---|---|
| Model | YOLOv8n (COCO, 80-class), ONNX 640² |
| Params / weights | 3.15 M / **12.8 MB** (`scripts/seeker/weights/yolov8n.onnx`, from HF `Kalray/yolov8`) |
| License | **AGPL-3.0** (Ultralytics) — brief R6: prefer NanoDet/MIT for a public repo |
| Measured latency (this box, CPU, 2 threads) | ~48–53 ms/frame → **~19–22 fps** |
| Brief embedded targets | Pi-5 CPU ~13 fps; Pi5+Hailo-8 ~35 fps end-to-end; Hailo HW ceiling ~431 fps |

## Result: honest, and it matches the brief's R2
The COCO model has **no drone class**; the target reads as **"airplane"** when it is
large enough. Out of the box the detector's dominant response was **false**: it
locked the **interceptor's own rotor arms** (dark high-contrast blades at the FOV
periphery, conf up to 0.79) and the **horizon line** — a naive top-1 pick returned
those on 72% of frames while missing the actual target. Three gates fix it, all
physically motivated:
1. **Horizon reject** — drop boxes > 25% of frame area.
2. **Static self-occlusion mask** — drop boxes touching the frame border (own props
   are body-fixed at the periphery).
3. **Aspect-ratio gate** — own rotor blades read as a long thin wedge (~4–8:1); the
   target quad body is compact (measured ~1.4–1.7:1). Reject aspect > 3.

After gating, **genuine** target detections survive only in a **~5-frame terminal
window (frames 540/542/543, range ~1.6–3.0 m)**, where the drone is large and
compact enough for a 640²-downscaled COCO nano to fire. Bearings are sane and
near-boresight (+7.9 / −0.3 / −4.5°); coarse known-size range (1.6–3.0 m) matches
the AprilTag ground-truth range (1.9–3.2 m) within the expected 15–30 % σ.

**Bottom line:** an **untuned COCO nano is not a viable seeker here** — it acquires
the target *later* than even the AprilTag it replaces, exactly the brief's **R2**
(acquisition-range regression on a few-pixel body). The value delivered is the
**working inference + bearing/range path** and a clean seam where a **fine-tuned
single-class drone nano** drops in unchanged (the required next step). The gating
insight (self-arm / horizon / aspect) is reusable by any detector lane.

## Body-not-tag proof (the whole point of "kill the AprilTag")
The target in this demo happens to carry the AprilTag, so we proved detection is of
the **airframe, not the fiducial**:
- **Box extent:** the detection box is **2.6–2.8× the tag size** and centred on the
  **body** (crosshair at body centre, offset from the tag) — see
  `eval_out/det_frame_000543.png`: the box encloses arms, prop discs, and motors.
- **Tag-inpaint test:** paint the tag pixels out (reconstruct body/arm texture — the
  natural "no tag" case) and re-run. The airframe is **still detected in 3/6** close
  frames (conf up to 0.78). A fiducial-decoder would go to zero.
- **Honest caveat:** confidence drops when the tag is removed (its contrast helps),
  and the flat-fill variant kills detection (an unnatural patch). A **fully**
  tag-independent number requires the tag-less Gazebo model
  `models/fpv_target_markerless` (exists) flown headless — flagged as the next step.
  Per the brief, the sim has no motion-blur/vibration model, so this sim seeker
  number is an **upper bound** on a real one.

## Honesty / no-cheat
`NNSeeker` reads only the camera frame + fixed intrinsics; it never touches any
`gt_*` pose. `eval_tag_boxes.json` (AprilTag pixel boxes for the 6 decodable frames)
is **scoring-only** and is never fed to the seeker.

## How to run
```
OMP_NUM_THREADS=2 nice -n 15 \
  ~/interceptor-sim/.venv-seeker/bin/python \
  scripts/seeker/eval_nn.py --n 40          # sample; add --all for full 867 sweep
```
Post-batch TODO: full 867-frame sweep; A/B vs AprilTag on paired seeds; drop in a
fine-tuned MIT/Apache drone nano; bench on Pi5+Hailo (ADR-0033 item 1).
