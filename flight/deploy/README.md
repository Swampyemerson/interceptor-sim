# `flight/deploy` — the real-hardware terminal seeker loop

This is the code the **physical interceptor's Pi 5 will run**. It is the same
camera-only pro-nav terminal the Gazebo sim validates (`scripts/m4_intercept.py`
ENGAGE phase), but wired to **real I/O** and driving the portable, test-pinned
[`flight/`](../) core. Running it on the desk exercises that whole core
end-to-end, so a bug in the frame math, the LOS derotation, the alpha-beta
tracker, or the pro-nav law surfaces **here, on the bench — before it ever costs
a real flight.**

## Pipeline

```
FRAME SOURCE            Picamera2 (Pi) | video file | image dir | synthetic stub
  → ONNX NN DETECT      scripts/seeker/finetuned_seeker.FinetunedNNSeeker  (box)
  → BEARING + RANGE     box centre → flight.camera.CameraModel  (UNDISTORTS)
                        range = fx·span / box_width_px   (span from calib sidecar)
  → LOS AZIMUTH         flight.geometry.derotate_bearing_lambda  (full attitude
                        + fixed mount up-tilt)
  → LEVER-ARM           flight.geometry.camera_to_cg_los  (camera → CG re-anchor)
  → ESTIMATOR           flight.estimator.AlphaBetaFilter  (λ and range channels)
  → PRO-NAV  (N = 5)    flight.guidance.closing_speed + pronav_lateral_accel
  → SETPOINT            NED velocity + absolute yaw → MAVSDK OFFBOARD
                        (`--dry-run` PRINTS the setpoint instead of sending)
```

Every stage is a thin wrapper over an already-tested `flight/` piece. The only
genuinely hardware-specific parts this file adds are **grabbing a frame from a
real camera** and **pushing a setpoint over MAVLink** — both guarded so the desk
path runs without them.

## Honesty / no-cheat boundary (CLAUDE.md #5, ADR-0010)

The loop's **only** inputs are:

1. **camera pixels**, and
2. the flight controller's **own-state EKF** — the body→NED attitude quaternion,
   yaw, and altitude that MAVSDK reports over MAVLink.

There is **no `gt_*`** anywhere in this code (there is no ground truth on real
hardware to read); `grep -n gt_ flight/deploy/seeker_loop.py` returns only the
comments that document this boundary. The `flight/` core it calls is itself
gt-free and test-pinned, and `flight.camera` **undistorts** the target pixel
before any bearing is taken — honesty-critical for the real wide M12 lens, and a
byte-identical no-op for the sim pinhole. The ONNX weights were fine-tuned
**offline** on gt-projected labels (a human-labeler analogue); the **live**
detector at inference is gt-free.

On the **desk** (no flight controller) own-state falls back to a **level hover**
(identity attitude, yaw 0, fixed altitude). That is a benign "assume the camera
is level" stand-in for a bench test — not ground truth.

## Usage

Run everything with the seeker venv (`onnxruntime` + `opencv`):

```bash
# 1) Self-test — synthetic frame → asserts a finite setpoint (no sim/weights/camera)
.venv-seeker/bin/python -m flight.deploy.seeker_loop --self-test        # exits 0/1

# 2) Desk replay over existing captured frames (dry-run PRINTS setpoints)
.venv-seeker/bin/python -m flight.deploy.seeker_loop \
    --source scripts/seeker/data/quad_approach/images \
    --weights scripts/seeker/weights/drone_finetuned_quad_v2.onnx \
    --dry-run --mount-tilt-deg 8

# 3) Desk replay over a recorded flight video
.venv-seeker/bin/python -m flight.deploy.seeker_loop \
    --source flight.mp4 --dry-run

# 4) On the vehicle — live Pi camera, real PX4 OFFBOARD over MAVLink
.venv-seeker/bin/python -m flight.deploy.seeker_loop \
    --source picamera --mavsdk-url udpin://0.0.0.0:14540 \
    --weights scripts/seeker/weights/drone_finetuned_quad_v2.onnx \
    --mount-fwd-m 0.10 --mount-up-m 0.05 --mount-tilt-deg 8
# add --dry-run to PRINT setpoints instead of commanding the vehicle
```

## Configuration

| Flag | Meaning | Default |
|---|---|---|
| `--source` | image dir · video file · `picamera` | (required unless `--self-test`) |
| `--weights` | ONNX detector | `weights/drone_finetuned_quad_v2.onnx` |
| `--intrinsics` | `calibrate_camera.py` / `camera_intrinsics.json` (fx,fy,cx,cy + distortion) | `camera_intrinsics.json` |
| `--mavsdk-url` | MAVLink connect URL; omit for desk dry-run | none |
| `--dry-run` | print setpoints instead of sending | off |
| `--mount-fwd-m` / `--mount-left-m` / `--mount-up-m` | camera lever arm from the CG, body frame (m) | 0.10 / 0 / 0.05 |
| `--mount-tilt-deg` | fixed camera up-tilt of the boresight | 0 |
| `--n-pronav` | pro-nav navigation gain N | 5 |
| `--conf` | detector confidence | 0.25 |
| `--fps` | control-loop rate | 20 |

The **known-size range span** resolves exactly as the sim detector does:
`MARKERLESS_SPAN_M` env → `<weights>.calib.json` sidecar → the config default.
The guidance constants (alpha-beta gains, closing-speed law, velocity clamps,
terminal-coast range) default to the sim's validated **FPV profile**
(`scripts/m4_intercept.py` `FPV{}`, ADR-0010/0011) and live in `GuidanceConfig`.

## Dependencies

- **Required (in `.venv-seeker`):** `numpy`, `opencv`, `onnxruntime`.
- **Guarded (absent on the x86 desk):** `picamera2` (Pi camera) and `mavsdk`
  (MAVLink). The self-test and image/video dry-run paths import neither, so they
  run on this desk unchanged.
