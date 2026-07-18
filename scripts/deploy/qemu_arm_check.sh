#!/usr/bin/env bash
# qemu_arm_check.sh -- validate the seeker SOFTWARE STACK runs on aarch64
# (the Raspberry Pi 5's CPU architecture) BEFORE buying the Pi.
#
# WHAT THIS PROVES: onnxruntime loads drone_finetuned_quad_v2.onnx and runs
# one inference, flight/ (guidance/geometry/estimator/camera) imports and its
# math checks out, and scripts/frame_source.OpenCVFrameSource (the generic
# non-Picamera camera-ingestion path) opens and decodes a frame -- ALL on a
# REAL aarch64 CPU target, via QEMU user-mode emulation (through Docker's
# --platform linux/arm64, which is QEMU user-mode + binfmt_misc under the
# hood; the qemu-only chroot path is documented as a fallback below).
#
# WHAT THIS DOES NOT PROVE (do not oversell this):
#   - fps / real-time performance. Emulation interprets/JITs aarch64
#     instructions on the host x86_64 CPU -- it is not a speed proxy for the
#     Pi 5's Cortex-A76 in either direction. Real fps needs the physical Pi
#     (AprilTag ~30 fps CPU per docs/hardware_order_list.md) or the Hailo
#     vendor bench (markerless NN path).
#   - The real camera driver (picamera2/libcamera + kernel driver). Needs
#     physical hardware; explicitly out of scope.
#   - MAVSDK/PX4 offboard control on ARM, or anything needing a flight
#     controller to talk to.
#
# See README.md in this directory for the full writeup and the manual runbook
# to use if this script reports MISSING (no docker / no qemu here).
#
# Usage:
#   scripts/deploy/qemu_arm_check.sh [--backend docker|host-selftest] \
#       [--model PATH] [--image PATH] [--keep-container]
#
# Exit codes:
#   0  stack validated on real aarch64 (or host self-test passed, when
#      explicitly requested with --backend host-selftest).
#   1  the check ran but a stage failed (see output).
#   2  no working aarch64 emulation backend is available in this environment
#      (docker missing, or docker present but can't emulate arm64). The
#      script explains exactly what to install; it does NOT apt-get install
#      anything itself.

set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODEL="scripts/seeker/weights/drone_finetuned_quad_v2.onnx"
IMAGE=""
BACKEND="auto"
KEEP_CONTAINER=0
PY_IMAGE="python:3.12-slim-bookworm"
OUT_DIR="${REPO_ROOT}/logs/deploy_checks"
TS="$(date -u +%Y%m%dT%H%M%SZ)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --backend) BACKEND="$2"; shift 2 ;;
    --model) MODEL="$2"; shift 2 ;;
    --image) IMAGE="$2"; shift 2 ;;
    --keep-container) KEEP_CONTAINER=1; shift ;;
    -h|--help) grep '^#' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

mkdir -p "${OUT_DIR}"
OUT_JSON="${OUT_DIR}/qemu_arm_check_${TS}.json"

banner() { echo; echo "================================================================"; echo "$1"; echo "================================================================"; }

print_runbook() {
  banner "NO WORKING aarch64 EMULATION BACKEND FOUND"
  cat <<'EOF'
This script needs ONE of the following to actually run the check on a real
aarch64 target. Neither was usable in this environment. Nothing was
installed automatically (project policy: ask before apt/system changes).

--------------------------------------------------------------------------
OPTION A (recommended) -- Docker + multi-arch emulation
--------------------------------------------------------------------------
  1. Install Docker Engine:
       sudo apt-get update && sudo apt-get install -y docker.io
       sudo usermod -aG docker "$USER"   # then re-login (or `newgrp docker`)

  2. Install the binfmt_misc handlers so Docker can run arm64 images under
     QEMU user-mode emulation (either works; the tonistiigi image is the
     modern Docker-recommended one-liner and needs no separate apt package):
       docker run --privileged --rm tonistiigi/binfmt --install arm64
     -- or, the classic apt route --
       sudo apt-get install -y qemu-user-static binfmt-support
       sudo update-binfmts --enable qemu-aarch64

  3. Sanity check:
       docker run --rm --platform linux/arm64 python:3.12-slim-bookworm uname -m
     should print: aarch64

  4. Re-run this script:
       scripts/deploy/qemu_arm_check.sh

--------------------------------------------------------------------------
OPTION B (heavier, no Docker) -- raw qemu-user-static + a Debian aarch64
chroot
--------------------------------------------------------------------------
  1. sudo apt-get update && sudo apt-get install -y qemu-user-static \
       binfmt-support debootstrap

  2. Build a minimal aarch64 Debian rootfs (needs root, ~300-500 MB
     download -- ask before running if that crosses your data-cap comfort):
       sudo debootstrap --arch=arm64 --foreign bookworm /tmp/arm64-rootfs \
         http://deb.debian.org/debian
       sudo cp /usr/bin/qemu-aarch64-static /tmp/arm64-rootfs/usr/bin/
       sudo chroot /tmp/arm64-rootfs /debootstrap/debootstrap --second-stage
       sudo chroot /tmp/arm64-rootfs apt-get update
       sudo chroot /tmp/arm64-rootfs apt-get install -y python3 python3-pip

  3. Bind-mount the repo in and run the check directly:
       sudo mount --bind /home/emerson/interceptor-sim /tmp/arm64-rootfs/work
       sudo chroot /tmp/arm64-rootfs /bin/bash -c '
         cd /work &&
         pip3 install --break-system-packages onnxruntime numpy opencv-python-headless &&
         python3 scripts/deploy/arm_stack_check.py --out /work/logs/deploy_checks/manual_chroot_result.json'
       sudo umount /tmp/arm64-rootfs/work

  Option A is strictly less work and is what this script automates; Option B
  is here only for a Docker-less machine.
--------------------------------------------------------------------------

Until one of these is set up, the honest status is: NOT VALIDATED on ARM.
The pure-Python / onnxruntime / opencv stack has no obvious reason to fail
on aarch64 (all three ship official aarch64 wheels/builds), but that is an
expectation, not a measurement -- don't cite it as a measurement.
EOF
}

have_docker_arm64() {
  command -v docker >/dev/null 2>&1 || return 1
  docker info >/dev/null 2>&1 || return 1
  # Real emulation check, not just "the flag was accepted": ask the emulated
  # interpreter what it thinks its own arch is.
  local m
  m="$(docker run --rm --platform linux/arm64 "${PY_IMAGE}" uname -m 2>/dev/null)"
  [[ "$m" == "aarch64" ]]
}

run_docker_backend() {
  banner "Backend: Docker --platform linux/arm64 (QEMU user-mode under binfmt_misc)"
  echo "[+] confirmed emulated interpreter reports: aarch64"

  local rm_flag="--rm"
  [[ "${KEEP_CONTAINER}" -eq 1 ]] && rm_flag=""

  # ONE array for both flags: model_arg is never empty (MODEL always has a
  # default), so the printf %q below always gets >=1 argument. printf on a
  # TRULY empty array still emits one spurious '' (missing-arg default), so
  # never printf an array that could itself be empty -- merge first.
  local extra_args=(--model "${MODEL}")
  [[ -n "${IMAGE}" ]] && extra_args+=(--image "${IMAGE}")

  # Install deps + run inside the container. pip pulls the aarch64 manylinux
  # wheels for onnxruntime/numpy/opencv-python-headless from PyPI (all ship
  # official aarch64 builds) -- this needs network access from the
  # container. Cache pip downloads on the host side across re-runs.
  docker run ${rm_flag} --platform linux/arm64 \
    -v "${REPO_ROOT}:/work:ro" \
    -v "${OUT_DIR}:/out" \
    -v "qemu_arm_check_pip_cache:/root/.cache/pip" \
    -w /work \
    "${PY_IMAGE}" \
    bash -c "
      set -e
      echo '[container] python:' \$(python3 --version) '  arch:' \$(uname -m)
      pip install --quiet --disable-pip-version-check \
        onnxruntime numpy 'opencv-python-headless' 2>&1 | tail -5
      python3 scripts/deploy/arm_stack_check.py \
        --repo-root /work \
        $(printf ' %q' "${extra_args[@]}") \
        --out /out/$(basename "${OUT_JSON}")
    "
  local rc=$?
  echo "[+] container exit code: ${rc}"
  if [[ -f "${OUT_JSON}" ]]; then
    echo "[+] results: ${OUT_JSON}"
  fi
  return ${rc}
}

run_host_selftest_backend() {
  banner "Backend: host-selftest (x86_64) -- NOT an ARM validation"
  echo "This mode only checks arm_stack_check.py's OWN logic sim-free on the"
  echo "host, per project convention. It will correctly report machine=x86_64"
  echo "and validated_arm=false. Use --backend docker for the real answer."
  local py="${REPO_ROOT}/.venv-seeker/bin/python3"
  [[ -x "$py" ]] || py="python3"
  local model_arg=(--model "${MODEL}")
  local image_arg=()
  [[ -n "${IMAGE}" ]] && image_arg=(--image "${IMAGE}")
  "$py" "${REPO_ROOT}/scripts/deploy/arm_stack_check.py" \
    --repo-root "${REPO_ROOT}" "${model_arg[@]}" "${image_arg[@]}" --out "${OUT_JSON}"
}

case "${BACKEND}" in
  docker)
    have_docker_arm64 || { banner "docker --platform linux/arm64 not usable"; print_runbook; exit 2; }
    run_docker_backend; exit $?
    ;;
  host-selftest)
    run_host_selftest_backend; exit $?
    ;;
  auto)
    if have_docker_arm64; then
      run_docker_backend; exit $?
    else
      print_runbook
      exit 2
    fi
    ;;
  *)
    echo "unknown --backend '${BACKEND}' (use: auto | docker | host-selftest)" >&2
    exit 2
    ;;
esac
