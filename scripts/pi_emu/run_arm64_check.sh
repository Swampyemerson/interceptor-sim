#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# run_arm64_check.sh -- prove the Pi 5 SOFTWARE stack runs on ARM64 in emulation
# BEFORE the hardware arrives. Sets up (or reuses) a linux/arm64 container and
# runs three end-to-end checks. Exit 0 = all pass, 1 = a failure/finding.
#
# SCOPE (constraint 'pi5-emulation-gap', docs/project_state.json):
#   This proves the SOFTWARE STACK ONLY -- imports resolve and the pipeline runs
#   end-to-end on aarch64. It NEVER measures fps / throughput: emulated speed
#   under qemu-user is NOT representative of the real Cortex-A76. Any wall time
#   printed here is MEANINGLESS for performance.
#
# Prereqs (builder-approved apt): qemu-user-static, binfmt-support, docker.io.
# The aarch64 binfmt must be registered with the F (fix_binary) flag so the
# container can exec arm64 binaries without qemu inside it.
#
# Usage:
#   scripts/pi_emu/run_arm64_check.sh            # build-if-missing, then check
#   scripts/pi_emu/run_arm64_check.sh --rebuild  # force rebuild the arm64 image
# ---------------------------------------------------------------------------
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../.." && pwd)"
IMAGE="pi5-arm64-check:latest"
PLATFORM="linux/arm64"
TS="$(date +%Y%m%d_%H%M%S)"
LOG="$REPO/logs/pi_emu_arm64_${TS}.log"
mkdir -p "$REPO/logs"

REBUILD=0
[ "${1:-}" = "--rebuild" ] && REBUILD=1

# docker may or may not need sudo on this host.
DOCKER="docker"
if ! docker info >/dev/null 2>&1; then
  DOCKER="sudo docker"
fi

# Everything below is tee'd to the log AND the terminal.
{
  echo "======================================================================"
  echo " Pi 5 ARM64 software-stack emulation check"
  echo " date=$(date -Is)   host_arch=$(uname -m)   emulated_target=aarch64"
  echo " NOTE: SOFTWARE-STACK PROOF ONLY. fps/throughput NOT measured -- qemu"
  echo "       emulation is not representative of the Cortex-A76. Wall times"
  echo "       below are meaningless for performance."
  echo "======================================================================"

  echo
  echo "--- [pre] qemu-aarch64 binfmt registration ---"
  if [ -r /proc/sys/fs/binfmt_misc/qemu-aarch64 ]; then
    cat /proc/sys/fs/binfmt_misc/qemu-aarch64
  else
    echo "MISSING: /proc/sys/fs/binfmt_misc/qemu-aarch64 -- install qemu-user-static"
    echo "OVERALL: FAIL (no aarch64 binfmt)"; exit 1
  fi

  echo
  echo "--- [pre] arm64 hello (uname -m must print aarch64) ---"
  HELLO="$($DOCKER run --rm --platform "$PLATFORM" arm64v8/busybox uname -m 2>&1)"
  echo "arm64 container reports: $HELLO"
  if [ "$HELLO" != "aarch64" ]; then
    echo "OVERALL: FAIL (arm64 hello did not report aarch64)"; exit 1
  fi

  echo
  if [ "$REBUILD" = "1" ] || ! $DOCKER image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "--- [build] building arm64 image $IMAGE (qemu-slow; be patient) ---"
    $DOCKER build --platform "$PLATFORM" -t "$IMAGE" -f "$SCRIPT_DIR/Dockerfile" "$SCRIPT_DIR"
    bexit=$?
    if [ $bexit -ne 0 ]; then
      echo "OVERALL: FAIL (arm64 image build failed, exit $bexit)"; exit 1
    fi
  else
    echo "--- [build] reusing existing arm64 image $IMAGE ---"
  fi

  echo
  echo "--- [run] three end-to-end checks inside the arm64 container ---"
  echo "    (repo mounted read-only at /work)"
  $DOCKER run --rm --platform "$PLATFORM" \
    -v "$REPO":/work:ro -w /work \
    "$IMAGE" bash /work/scripts/pi_emu/_inside_container.sh
  run_rc=$?

  echo
  echo "======================================================================"
  if [ $run_rc -eq 0 ]; then
    echo " RESULT: PASS -- Pi 5 software stack runs on aarch64 (fps NOT tested)"
  else
    echo " RESULT: FAIL -- an ARM64 finding surfaced (exit $run_rc). See above."
  fi
  echo " log: $LOG"
  echo "======================================================================"
  exit $run_rc
} 2>&1 | tee "$LOG"

# Propagate the inner exit code (tee would otherwise mask it).
exit "${PIPESTATUS[0]}"
