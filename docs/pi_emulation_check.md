# Pi 5 ARM64 emulation check — the software stack runs on aarch64

**Status: PASS (2026-07-20). All three end-to-end checks pass on emulated
aarch64 — evidence `logs/pi_emu_arm64_20260720_213358.log` — with ONE real
ARM64 finding surfaced and worked around (the `pupil_apriltags` module-name
gap, below; first-run failure evidence `logs/pi_emu_arm64_20260720_212850.log`).**

## The one-line scope disclaimer (repeat it everywhere)

> **Emulation proves the SOFTWARE stack only** — imports resolve and the
> pipeline runs end-to-end on aarch64 — **NEVER fps/performance**. Emulated
> throughput under qemu-user is **not representative of the real Cortex-A76**;
> every wall time in the logs is meaningless for performance. Real fps numbers
> come only from the physical Pi 5 bench (`build_tab` **skr-07**, tripod day).
> This is constraint **`pi5-emulation-gap`** in `docs/project_state.json`.

## What this check is

The P0 "run qemu" item: prove the **Pi 5 software stack** runs on **ARM64
(aarch64)** in emulation **before the hardware arrives**, so hardware day is a
smoke test rather than a porting session. It extends the earlier
`scripts/deploy/qemu_arm_check.sh` pass (2026-07-18, constraint
`qemu-compute-gap`: flight/ core + onnxruntime inference + OpenCV decode on
aarch64) with the three checks that one explicitly did **not** cover:

| # | Check | What it proves on aarch64 |
|---|---|---|
| 1 | `python -m flight.deploy.seeker_loop --self-test` | The deployed terminal-guidance loop end-to-end: synthetic detection → geometry → estimator → pro-nav → finite NED setpoint (7/7 checks). |
| 2 | `scripts/seeker/pi_capture.py --self-test` | The capture pipeline end-to-end on its dir-replay backend, **including a real AprilTag decode on real pixels** (pyapriltags/`pupil_apriltags`) + session layout, index.csv, meta.json, tags.csv contracts. |
| 3 | `pytest flight/tests/` | The flight core unit suite (camera model / undistort, geometry, estimator, guidance) under the Pi's Python. |

Plus an import-level gate before them: `numpy`, `cv2`
(opencv-python-headless), `pupil_apriltags` (provided by **pyapriltags** — the
aarch64 drop-in per ADR-0012 / `requirements-pi.txt`), `mavsdk` (import only,
pinned 3.15.3 to match the repo), and `onnxruntime` (import only — the NN
seeker path is a lazy import that the self-tests do not exercise).

## How it runs

QEMU **user-mode** emulation via Docker: `qemu-user-static` + `binfmt-support`
register an aarch64 interpreter with the `F` (fix_binary) flag, so
`docker run --platform linux/arm64` executes genuine aarch64 binaries on this
x86_64 host. The environment is `python:3.11-slim-bookworm` for arm64 —
Python 3.11 matches Raspberry Pi OS Bookworm's default, so the import surface
matches the Pi. The repo is mounted **read-only** at `/work`; nothing in the
container can touch the checkout.

```
scripts/pi_emu/
  run_arm64_check.sh      # the repeatable entry point (exit 0/1)
  Dockerfile              # arm64 python:3.11-slim-bookworm + the Pi-side deps
  _inside_container.sh    # the three checks, run inside the arm64 container
```

## Re-run it

```bash
scripts/pi_emu/run_arm64_check.sh             # reuses the built arm64 image
scripts/pi_emu/run_arm64_check.sh --rebuild   # force-rebuild the image first
```

- Exit **0** = every check passed; exit **1** = a failure (a real ARM64
  finding — exactly what this exists to surface early).
- Each run writes `logs/pi_emu_arm64_<timestamp>.log` with the full evidence.
- Prereqs (builder-approved apt, already installed 2026-07-20):
  `qemu-user-static`, `binfmt-support`, `docker.io`. The script verifies the
  binfmt registration and runs an arm64 `uname -m` hello before anything else.
- **Be patient**: everything inside the container is qemu-interpreted and slow.
  That slowness is expected and tells you nothing (see the scope disclaimer).

## The expected gap: picamera2

`picamera2` is **not installable** in a generic container — it is the
libcamera binding shipped by Raspberry Pi OS via apt (`python3-picamera2` +
`python3-libcamera`), built against the Pi's own libcamera and kernel driver
(`requirements-pi.txt` "apt, NOT pip"; `docs/camera_paper_check.md` item 5).
The check treats its absence as an **EXPECTED-GAP**, not a failure. Both
Pi-side entry points import it lazily and only on the `picamera2` source path,
so everything else is exercisable without it. **The real Pi validates the
camera path** (skr-02/skr-03/skr-05 in `scripts/pi_setup/README.md`).

## Environment differences vs the real Pi (honest deltas)

- **Container vs Pi OS**: deps here are pip wheels (`numpy`,
  `opencv-python-headless`) where the Pi uses apt (`python3-numpy`,
  `python3-opencv`) in a `--system-site-packages` venv — same import names,
  different builds. Version skew is possible; `provision.sh` + `selftest.sh`
  on the real Pi are the authority.
- **No camera, no UART, no FC**: mavsdk is import-level only; the OFFBOARD
  driver path needs a MAVLink endpoint and is validated by the SITL smoke and
  later on hardware.
- **No fps**: see the scope disclaimer. skr-07 (real-Pi thermal-soak bench) is
  the only source of throughput numbers.

## Results (2026-07-20, `logs/pi_emu_arm64_20260720_213358.log`)

Environment: `arch=aarch64`, Python 3.11.15, container `pi5-arm64-check:latest`
(numpy 2.4.6, opencv-python-headless 5.0.0.93 → `cv2 5.0.0`, pyapriltags
3.4.3.1, mavsdk 3.15.3, onnxruntime 1.27.0 — the exact `requirements-pi.txt`
pins resolved as aarch64 wheels, including the onnxruntime 1.27.0 pin).

| # | Check | Result | Evidence line in the log |
|---|---|---|---|
| 0 | Imports: numpy, cv2, `pupil_apriltags`, mavsdk, onnxruntime | **PASS** | five `[OK] import ...` lines; `[NOTE] pupil_apriltags provider: pyapriltags-shim` |
| 0b | picamera2 | **EXPECTED-GAP** (not a failure) | `[EXPECTED-GAP] picamera2 unavailable ... real Pi validates the camera path` |
| 1 | `flight.deploy.seeker_loop --self-test` | **PASS 7/7** | seven `[PASS]` lines → `[self-test] PASS`, `CHECK1_RC=0` |
| 2 | `pi_capture.py --self-test` (dir-replay + real tag decode) | **PASS 17/17** | `pupil-apriltags decoded >=1 composited tag (4 frames)`; `decoded tag range_m populated + sane (['4.02', '5.05', '6.04', '7.06'])` (truth: 4/5/6/7 m); `CHECK2_RC=0` |
| 3 | `pytest flight/tests/` | **PASS** | `32 passed`, `CHECK3_RC=0` |

Overall: `RESULT: PASS -- Pi 5 software stack runs on aarch64 (fps NOT tested)`,
script exit 0.

## THE ARM64 FINDING — `pupil_apriltags` module name (fix before hardware day)

This is exactly the class of breakage the check exists to catch early:

- **What broke**: the first run (`logs/pi_emu_arm64_20260720_212850.log`) failed
  check 2 with `ModuleNotFoundError: No module named 'pupil_apriltags'` at
  `scripts/seeker/pi_capture.py:135` (`from pupil_apriltags import Detector`).
- **Why**: `requirements-pi.txt` (and its ADR-0012 note) assumes *"pyapriltags
  exposes the SAME `pupil_apriltags` module name, so no code change"*. **That is
  false**: pyapriltags 3.4.3.1 installs only the `pyapriltags` module
  (`importlib.util.find_spec("pupil_apriltags")` → `None`). And installing real
  pupil-apriltags on aarch64 is not an option: `pip install --only-binary :all:
  pupil-apriltags==1.0.4.post11` resolves **"from versions: none"** — no aarch64
  wheel exists at all (confirms ADR-0012's premise, refutes its "same module
  name" rider).
- **The good news**: the `Detector` API is identical — same constructor
  (`families, nthreads, quad_decimate, quad_sigma, refine_edges,
  decode_sharpening, debug, searchpath`) and the same
  `detect(gray, estimate_tag_pose=..., camera_params=..., tag_size=...)`. A
  5-line alias module (`pupil_apriltags.py` re-exporting from `pyapriltags`,
  see `scripts/pi_emu/Dockerfile`, last layer) bridges it, and check 2 passing
  **through that shim with correct recovered ranges** proves the compatibility
  claim on real pixels, not just on paper.
- **Action for the pi_setup owner** (not edited here — `scripts/pi_setup/` is
  owned by another lane): either (a) `provision.sh` drops the same alias module
  into the venv's site-packages, or (b) `pi_capture.py` /
  `autolabel_from_apriltag.py`'s lazy imports gain a
  `try: pupil_apriltags / except: pyapriltags` fallback, and the
  `requirements-pi.txt` comment gets corrected. Until one of those lands, the
  Pi's `--decode-tags` path will crash exactly as the first log shows.
