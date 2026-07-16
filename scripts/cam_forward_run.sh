#!/usr/bin/env bash
# Run a command with the PX4 x500_mono_cam model SWAPPED to the camera-forward
# variant (ADR-0076 add #16: camera moved x 0.12 -> 0.40, past the front rotors,
# so the own-prop PHANTOM is out of FOV at the source). The original model is
# restored on EXIT via a trap, even on failure/kill -- never leaves the moved
# camera in place to contaminate later runs. Used for the add #18g camera-forward
# × coded-dash test (does removing the phantom yield genuine APPROACH acquisition?).
#
# Usage: [QUAD_ARM_*=...] scripts/cam_forward_run.sh scripts/coded_dash_arm.sh WEIGHTS SEED PATH OUT --go
set -euo pipefail
cd "$(dirname "$0")/.."
PX4_MODEL="${PX4_DIR:-$HOME/PX4-Autopilot}/Tools/simulation/gz/models/x500_mono_cam/model.sdf"
FWD_MODEL="$PWD/scripts/experiments/cam_forward/x500_mono_cam/model.sdf"
[ -f "$PX4_MODEL" ] || { echo "MISSING PX4 model: $PX4_MODEL"; exit 2; }
[ -f "$FWD_MODEL" ] || { echo "MISSING cam-forward model: $FWD_MODEL"; exit 2; }
BAK="${PX4_MODEL}.orig_$$"
cp "$PX4_MODEL" "$BAK"
restore() { cp "$BAK" "$PX4_MODEL" && rm -f "$BAK" && echo "[cam-fwd] restored original PX4 camera model"; }
trap restore EXIT
cp "$FWD_MODEL" "$PX4_MODEL"
echo "[cam-fwd] swapped in the camera-forward model (x=0.40, phantom out of FOV); running: $*"
"$@"
