# ---------------------------------------------------------------------------
# ARM64 (aarch64) emulation image: prove the Pi 5 SOFTWARE stack imports and the
# seeker pipeline runs end-to-end BEFORE the hardware arrives.
#
# SCOPE (constraint 'pi5-emulation-gap', docs/project_state.json): this proves
# the SOFTWARE STACK ONLY -- imports resolve, the pipeline runs end-to-end. It
# NEVER measures fps / throughput: emulated speed under qemu-user is NOT
# representative of the real Cortex-A76. Wall times here are meaningless.
#
# Built for linux/arm64 via qemu-user-static binfmt. Python 3.11 == Raspberry
# Pi OS Bookworm's default python, so the import surface matches the Pi.
# ---------------------------------------------------------------------------
FROM python:3.11-slim-bookworm

# opencv-python-headless needs libglib2.0 (libgthread-2.0.so.0) at import time;
# the rest of its native deps ship inside the wheel. No GUI libs (headless).
RUN apt-get update \
 && apt-get install -y --no-install-recommends libglib2.0-0 \
 && rm -rf /var/lib/apt/lists/*

# Pi-side Python deps, aarch64 wheels.
#   - pyapriltags: the aarch64 drop-in for pupil-apriltags (ADR-0012). NOTE:
#     it does NOT expose the `pupil_apriltags` module name (see the FINDING +
#     shim layer below) -- requirements-pi.txt's "no code change" note is wrong.
#   - numpy + opencv-python-headless: stand in here for the Pi's apt
#     python3-numpy / python3-opencv (the container has no apt camera stack).
#   - mavsdk: pinned to the repo's version so the OFFBOARD API that the sim
#     validated is the one imported here (requirements-pi.txt).
RUN pip install --no-cache-dir \
      numpy \
      opencv-python-headless \
      pyapriltags \
      mavsdk==3.15.3 \
      pytest

# onnxruntime is IMPORT-LEVEL ONLY in this check: the markerless NN seeker path
# in seeker_loop.py is a lazy import and is NOT exercised by the self-tests.
# Pin per requirements-pi.txt; relax to the nearest 1.2x aarch64 wheel if the
# exact pin has no aarch64 wheel (that fallback is itself a documented finding).
RUN pip install --no-cache-dir onnxruntime==1.27.0 \
 || pip install --no-cache-dir "onnxruntime>=1.20,<1.28" \
 || echo "WARN: no aarch64 onnxruntime wheel resolved; NN-seeker import unavailable"

# --- ARM64 FINDING (2026-07-20, this check) + the compatibility shim ---------
# requirements-pi.txt assumed "pyapriltags exposes the SAME pupil_apriltags
# module name, so no code change". FALSE on aarch64 as installed: pyapriltags
# 3.4.3.1 installs module `pyapriltags` ONLY (importlib finds no
# `pupil_apriltags`), while pi_capture.py line 135 lazily does
# `from pupil_apriltags import Detector`. And pupil-apriltags itself has NO
# aarch64 wheel at all (pip --only-binary resolves "from versions: none" --
# confirms ADR-0012). The Detector API is identical (same ctor args + the same
# detect(gray, estimate_tag_pose, camera_params, tag_size)), so a one-file
# alias package bridges it. The real-Pi provisioning must replicate this shim
# in its venv (or pi_capture's lazy import must gain a pyapriltags fallback) --
# flagged to the pi_setup owner. Check 2 passing THROUGH this shim proves the
# API-compatibility claim on real pixels.
RUN printf '%s\n' \
      '# aarch64 alias: pupil-apriltags has no aarch64 wheel (ADR-0012);' \
      '# pyapriltags is the sanctioned drop-in but installs under its OWN' \
      '# module name -- re-export it (scripts/pi_emu/Dockerfile, ARM64 FINDING).' \
      'from pyapriltags import *  # noqa: F401,F403' \
      'from pyapriltags import Detector  # noqa: F401' \
      > /usr/local/lib/python3.11/site-packages/pupil_apriltags.py \
 && python -c "from pupil_apriltags import Detector; print('shim OK:', Detector)"
