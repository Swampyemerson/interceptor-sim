#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Runs INSIDE the linux/arm64 container (invoked by run_arm64_check.sh). The
# repo is mounted read-only at /work. Proves the Pi 5 SOFTWARE stack on aarch64:
#   0) import-level checks (numpy, cv2, pupil_apriltags, mavsdk; onnxruntime
#      best-effort; picamera2 EXPECTED gap)
#   1) flight.deploy.seeker_loop --self-test   (7/7)
#   2) scripts/seeker/pi_capture.py --self-test (dir-replay + real tag decode)
#   3) pytest flight/tests/
#
# SOFTWARE-STACK PROOF ONLY -- never fps/throughput. Emulated wall time is
# meaningless (qemu-user is not representative of the Cortex-A76).
# ---------------------------------------------------------------------------
set -u
cd /work

# Keep every write off the read-only mount.
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPYCACHEPREFIX=/tmp/pyc
export PYTHONPATH=/work

fail=0
echo "######################################################################"
echo "# ARM64 SOFTWARE-STACK CHECK (emulated; correctness only, NOT fps)"
echo "# arch=$(uname -m)  python=$(python --version 2>&1)"
echo "######################################################################"

echo
echo "=== [0] import-level checks ==="
python - <<'PY'
import importlib, sys
required = ["numpy", "cv2", "pupil_apriltags", "mavsdk"]
bad = 0
for m in required:
    try:
        mod = importlib.import_module(m)
        print(f"  [OK]   import {m:<16} version={getattr(mod, '__version__', '?')}")
    except Exception as e:
        print(f"  [FAIL] import {m:<16} {type(e).__name__}: {e}")
        bad = 1
# Honest provenance: on aarch64 `pupil_apriltags` is the alias shim over
# pyapriltags (ADR-0012 + the ARM64 FINDING in scripts/pi_emu/Dockerfile) --
# record which provider actually backs it so the log can't oversell.
try:
    import pupil_apriltags as _pa
    import pyapriltags as _py
    backed = ("pyapriltags-shim"
              if "pyapriltags" in repr(getattr(_pa, "Detector", ""))
              else "native-pupil-apriltags")
    print(f"  [NOTE] pupil_apriltags provider: {backed} "
          f"(pyapriltags {getattr(_py, '__version__', '?')} at {_py.__file__})")
except Exception as e:
    print(f"  [NOTE] pupil_apriltags provenance probe failed: {e}")
# onnxruntime: import-level extra (NN seeker path is lazy, not in the self-tests)
try:
    import onnxruntime as ort
    print(f"  [OK]   import onnxruntime     version={ort.__version__}")
except Exception as e:
    print(f"  [WARN] import onnxruntime     {type(e).__name__}: {e}")
# picamera2: EXPECTED gap -- Pi-OS-specific (apt python3-picamera2 + libcamera),
# not pip-installable in a generic container. The real Pi validates it.
try:
    import picamera2  # noqa: F401
    print("  [??]   import picamera2        UNEXPECTED: imported in container")
except Exception as e:
    print(f"  [EXPECTED-GAP] picamera2 unavailable ({type(e).__name__}) "
          f"-- Pi-OS/libcamera only; real Pi validates the camera path")
sys.exit(bad)
PY
[ $? -ne 0 ] && fail=1

echo
echo "=== [1] flight.deploy.seeker_loop --self-test (expect 7/7) ==="
python -m flight.deploy.seeker_loop --self-test
rc=$?; echo "CHECK1_RC=$rc"; [ $rc -ne 0 ] && fail=1

echo
echo "=== [2] scripts/seeker/pi_capture.py --self-test (dir-replay + tag decode) ==="
python scripts/seeker/pi_capture.py --self-test
rc=$?; echo "CHECK2_RC=$rc"; [ $rc -ne 0 ] && fail=1

echo
echo "=== [3] pytest flight/tests/ ==="
python -m pytest flight/tests/ -q -p no:cacheprovider
rc=$?; echo "CHECK3_RC=$rc"; [ $rc -ne 0 ] && fail=1

echo
echo "######################################################################"
if [ $fail -eq 0 ]; then
  echo "# OVERALL: PASS  (software stack runs on aarch64; fps NOT tested)"
else
  echo "# OVERALL: FAIL  (see [FAIL] lines above -- an ARM64 finding)"
fi
echo "######################################################################"
exit $fail
