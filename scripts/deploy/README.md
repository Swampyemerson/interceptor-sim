# scripts/deploy/qemu_arm_check.sh — Pi 5 compute de-risk

**Question this answers:** before spending $120+ on a Raspberry Pi 5, does the
seeker software stack (onnxruntime, the `flight/` guidance core, the camera
ingestion path) even *run* on its CPU architecture (aarch64 / arm64)?

**Question this does NOT answer:** how fast. See the loud disclaimer below —
read it before citing this check anywhere.

## Files

- `qemu_arm_check.sh` — the runnable entry point. Detects an aarch64
  emulation backend (currently: Docker with `--platform linux/arm64`, which
  is QEMU user-mode emulation under `binfmt_misc`), runs the check inside it,
  and prints a runbook if nothing usable is installed.
- `arm_stack_check.py` — the actual payload. Pure Python, no argparse
  surprises; runs standalone too (`python3 arm_stack_check.py --help`).

## What it checks (3 stages)

1. **`flight/` core imports and its math is self-consistent** — the
   pure-Python guidance/geometry/estimator/camera modules that have zero
   hardware dependencies. A few cheap identity checks (not a full pytest
   run — `flight/tests/` already owns correctness on x86_64; this only asks
   "same CPU, same answer").
2. **onnxruntime loads `drone_finetuned_quad_v2.onnx` and runs one
   inference** through `scripts/seeker/v3_onnx_infer.py`'s decode path,
   which the codebase documents as byte-identical to the deployed detector
   (`finetuned_seeker.FinetunedNNSeeker`).
3. **`scripts/frame_source.OpenCVFrameSource` opens and decodes a frame** —
   the generic V4L2/USB/GStreamer camera-ingestion path (works via `cv2`
   alone), pointed at a real captured PNG standing in for a live device.

## Why "non-Picamera path" specifically

This project's `frame_source.py` has exactly one hardware-facing class,
`OpenCVFrameSource`, built on `cv2.VideoCapture`. It is deliberately generic:
a USB/UVC webcam, or a CSI camera exposed through V4L2/libcamera, both work
through the same `cv2.VideoCapture(device)` call. That's what stage 3 dry-runs.

It is **not** the Pi-specific `picamera2`/libcamera Python bindings (this
codebase doesn't use them, and doesn't need to — the Arducam cameras on the
BOM expose a V4L2 device node). Those bindings talk to the real Broadcom/RP1
ISP driver stack and can't be meaningfully exercised under CPU emulation
regardless — there's no real camera silicon to emulate. So "non-Picamera
path" isn't a limitation of this check, it's the actual shape of this
codebase's camera interface.

## What this DOES NOT prove — read this before citing the result

- **Not an fps measurement.** QEMU user-mode emulation interprets/JITs
  aarch64 instructions on the host x86_64 CPU. It is not a speed proxy for
  the Pi 5's Cortex-A76 in either direction — could be much slower, could
  (less likely) look deceptively okay on a fast host. `arm_stack_check.py`
  prints inference timing out of curiosity; it is explicitly labeled
  "NOT an fps proxy" in its own output and must not be used to size the
  seeker loop rate.
  - Real fps needs the **physical Pi 5** (AprilTag path, CPU-only, target
    ~30 fps per `docs/hardware_order_list.md` §0c/§C) or the **Hailo AI
    HAT+ vendor bench** (markerless YOLO path — CPU YOLO is ~5-10 fps and
    explicitly NOT viable at terminal LOS rates per the same doc).
- **Not the real camera driver.** `picamera2`/libcamera + the kernel ISP
  driver need physical hardware. Out of scope by construction.
- **Not MAVSDK/PX4 offboard on ARM.** Not imported by this check. MAVSDK
  ships aarch64 wheels, but the actual offboard link needs a flight
  controller to talk to — a post-purchase integration test, not a
  pre-purchase compute check.

## Status in this environment (as of the last run here)

Neither Docker nor `qemu-user-static`/`binfmt-support` is installed on this
machine, and there is no `/proc/sys/fs/binfmt_misc` aarch64 entry — so
`qemu_arm_check.sh` cannot currently do a real ARM run here. Running it
prints the exact install commands (`--backend auto`, the default) and exits
`2`. See the script's own `print_runbook()` output, or run
`scripts/deploy/qemu_arm_check.sh --help`.

The check logic itself **was** self-tested sim-free on the host
(`--backend host-selftest`, x86_64) using the real ONNX weights and a real
captured frame from `scripts/seeker/data/flight_capture/images/` — all 3
stages pass. That only proves the check script's own logic is correct and
that the stack works on x86_64; it explicitly reports
`"validated_arm": false` and the script refuses to word it any other way.
A sample result: `logs/deploy_checks/qemu_arm_check_20260718T034420Z.json`.

## How to actually get the ARM answer

```bash
# 1. one-time setup (needs sudo; ask first if you haven't already):
sudo apt-get update && sudo apt-get install -y docker.io
sudo usermod -aG docker "$USER"        # re-login after this
docker run --privileged --rm tonistiigi/binfmt --install arm64

# 2. sanity check binfmt is really emulating (not silently no-op'ing):
docker run --rm --platform linux/arm64 python:3.12-slim-bookworm uname -m
# must print: aarch64

# 3. run the real check:
scripts/deploy/qemu_arm_check.sh
```

A Docker-less fallback (raw `qemu-aarch64-static` + a debootstrapped Debian
aarch64 chroot) is documented in full inside `qemu_arm_check.sh`'s runbook
output (`--help`) — heavier and needs root + a few hundred MB download, only
worth it on a machine where Docker itself is off the table.

## Usage

```bash
scripts/deploy/qemu_arm_check.sh                       # auto-detect backend
scripts/deploy/qemu_arm_check.sh --backend docker       # force docker, fail loudly if unusable
scripts/deploy/qemu_arm_check.sh --backend host-selftest  # x86_64 self-test only, NOT an ARM answer
scripts/deploy/qemu_arm_check.sh --model path/to/other.onnx --image path/to/frame.png
```

Results are written to `logs/deploy_checks/qemu_arm_check_<UTC timestamp>.json`
(gitignored, like the rest of `logs/`).

## Expectation, not a guarantee

onnxruntime, numpy, and opencv-python-headless all ship official aarch64
manylinux wheels on PyPI, and `flight/` is pure Python — there's no obvious
reason any of this should fail to *run* on a Pi 5. But that's an
expectation from reading package metadata, not a measurement. Don't skip the
Docker setup and just assume — the whole point of this script is to turn
"should work" into "ran, here's the log."
