# Real-data seeker pipeline — AprilTag-supervised, minimal manual labeling

> Builder directive (2026-07-16): *"getting the real data capture pipeline ready is
> smart — I want to do as little feeding in specific data as possible."* And the
> make-or-break finding (ADR-0076 add #18h): the deployed detector's binding wall
> is **approach-aspect recall** (it can't see a small, fast, *approaching* drone at
> range) — so the whole point of this pipeline is to fix that with real data,
> captured cheaply.

## The idea (why it's near-zero manual effort)

You are allowed the AprilTag **for calibration**. So put a `tag36h11` placard on the
**target** drone, fly it, record the interceptor's camera, and **let the tag write the
labels**: the tag's detected pose gives the target's 3-D position, which projects to a
YOLO bounding box automatically. You hand-label **nothing**. Then fine-tune a
**markerless** detector on those auto-labels; at inference the tag is off — the deployed
seeker never needs it (honesty boundary: the tag is a *training-time label source* only,
the same role ground truth plays in `gen_sim_dataset.py`; the deployed detector re-earns
the no-cheat audit; do not leave the tag on the operational target).

```
fly TAGGED target (APPROACHING passes)  ──►  record frames + tag
   └─► autolabel_from_apriltag.py  ──►  YOLO dataset (labels auto-written)
        └─► fine-tune markerless (COCO-init)  ──►  shadow-eval on HELD-OUT real flights
```

This is the real-hardware twin of the sim's `gen_sim_dataset.py` (auto-labels sim frames
from gt). Built: **`scripts/seeker/autolabel_from_apriltag.py`** (reuses the sim's
`project_to_bbox`; `--self-test` green). **Negatives: only from TARGET-FREE footage.** Run
the tool with `--negatives-from-untagged` on explicitly target-free segments (empty
sky/ground, pre-launch) to mine hard negatives (own-prop etc.) for free. On target-PRESENT
approach footage the tool DROPS tag-miss frames by default — because the tag decodes only to
a few metres while the drone is visible from ~30 m, so labeling those frames 'background'
would train the detector to NOT see the approaching target (the exact add-#18h wall). Never
run `--negatives-from-untagged` on footage where the target is present but the tag failed.

## Stages

1. **Capture — the ONE thing you must get right (add #18h).** Record **APPROACHING
   passes**, not hovers: the target flying *toward* the camera at the real engagement
   speed/aspect, from ~30 m in to close, across a few lighting/background conditions. The
   approach geometry is the recall the whole intercept depends on; a hover data set will
   pass val and fail in flight (the exact v3/rebal trap). Tooling: the existing
   `capture_flight_frames.py` (adapt for the Pi camera + a synced tag-pose log), or record
   raw frames and run the tag detection offline inside the auto-labeler.
2. **Auto-label.** `autolabel_from_apriltag.py --frames DIR --calib calib.json
   --tag-size 0.10 --drone-size 0.35 --out DATASET`. Needs the camera calibration
   (`scripts/calibrate_camera.py` → `flight.camera`, already built). Reports the tag's
   approach-detection rate — *that is the ceiling* the markerless detector is chasing.
3. **Retrain.** Fine-tune YOLO11n from **COCO-pretrained** (NOT the sim weights — the
   visual domain is what doesn't transfer, ADR-0061), ~15–20% negatives (the rebal lesson),
   on a dev box / cheap cloud GPU (the Pi never trains; `.venv-seeker-train` is CPU-only).
4. **Validate — anti-mirage gate.** Hold out **whole FLIGHTS** (different day/lighting),
   never random frames (the v3-null trap). Gate on **held-out approach-phase recall vs
   range** + false-positive rate/hour — NOT val mAP. Fly the retrained detector in
   **shadow mode** (log-only, alongside the AprilTag seeker) so you measure real bearing σ,
   phantom rate, and R_acq at zero flight risk before it ever steers.

## What minimizes manual effort (the directive)

- **Labels are automatic** (tag pose → box). You fly and record; that's the human effort.
- **Negatives are free** — but ONLY from target-free segments (`--negatives-from-untagged`);
  tag-miss frames on target-present footage are dropped, never labeled background (see above).
- **No frame-count target to curate** — capture until the held-out approach-recall gate
  passes; stop on the gate, not a number.
- **Calibration is one-time** (`calibrate_camera.py`, already built + self-tested).

## Alternatives considered (why AprilTag-supervised wins)

| approach | manual effort | box quality | verdict |
|---|---|---|---|
| **AprilTag-supervised (this)** | fly+record only | tight (pose-projected) | **chosen** — reuses the allowed tag + the sim's projector |
| GPS/telemetry-supervised | fly+record | coarse (~1 m rel error) boxes | fallback if no tag; too loose for a small target at range |
| Hand-label | high (thousands of frames) | tight | rejected — the effort the builder wants to avoid |
| Foundation/open-vocab auto-label (e.g. Grounding-DINO) | low | variable, needs review | keep as a *negative-set enricher* / cross-check, not the primary |

## Validate the auto-labeler itself (before the hardware, in sim)

The sim's `apriltag` world has BOTH the tag AND gt, so the auto-labeler can be checked now:
record apriltag-world frames, auto-label from the tag, and compare each box to the gt box
`gen_sim_dataset.py` would produce from gt. Agreement proves the tag→box projection before
any real footage exists. (Follow-up capture task; the projection math is already
`--self-test`-green.)

## Status

- ✅ `autolabel_from_apriltag.py` built; projection `--self-test` green; **hardened after the
  Fable round-1 check** — (1) tag-miss frames are now DROPPED by default (only `--negatives-from-
  untagged` on TARGET-FREE footage makes them negatives) so approach footage can't be poisoned
  with false 'background' labels beyond the tag's decode range; (2) `--tag-offset-m` applied in
  the tag frame via `pose_R`; (3) frames undistorted before detect+project.
- ⚠️ **Partial validation only** (do NOT overclaim): the projection math is self-test-green, the
  pose is accurate on one frame (`tag_at_5m.png`: z=4.97 m vs the 5 m world placement, 0.7% error),
  and the end-to-end run caught a real `pose_t` shape bug. But the box-width "match" was circular
  (the same `fx·s/rng` formula twice), and it's one on-axis ideal-camera frame. **The real
  validation — tag-box vs gt-box (IoU) and label-rate, both vs RANGE over a full approach pass in
  the apriltag world — has NOT run yet; it's the next step, and its label-ceiling-vs-range curve
  decides the placard size / far-band labeling strategy before any hardware flies.**
- ✅ `calibrate_camera.py` + `flight.camera` (calibration → intrinsics) built + self-tested.
- ✅ `approach_recall.py` — measures the deployed detector's real-target recall vs range (the wall).
- ⏳ Capture tooling for the Pi camera + the approaching-pass protocol (hardware-gated).
