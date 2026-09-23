#!/usr/bin/env bash
# Run a command with an UP-TILTED seeker camera, via the models/mono_cam shadow
# (a symlink to a scripts/experiments/uptilt_mounts/<TILT>/mono_cam variant that
# GZ_SIM_RESOURCE_PATH resolves before PX4's canonical mono_cam). Removed on EXIT
# via a trap so no leftover shadow contaminates later runs (mc_deployment_arm.sh
# refuses to run if one is left behind). For the add #18j-fix pointing test:
# does an up-tilt keep the APPROACHING target in view through 8-16 m (vs the
# nose-down dash pitch dropping it >12 m)?  #46 / ADR-0065 adaptive-tilt lever.
#
# Usage: [TILT=up35] scripts/uptilt_run.sh <command...>
set -uo pipefail
cd "$(dirname "$0")/.."
TILT="${TILT:-up35}"
SRC="$PWD/scripts/experiments/uptilt_mounts/$TILT/mono_cam"
SHADOW="$PWD/models/mono_cam"
[ -d "$SRC" ] || { echo "MISSING uptilt variant: $SRC"; exit 2; }
[ -e "$SHADOW" ] && { echo "models/mono_cam already exists -- abort to avoid clobber"; exit 2; }
ln -s "$SRC" "$SHADOW"
restore() { rm -f "$SHADOW"; echo "[uptilt] removed models/mono_cam shadow"; }
trap restore EXIT
# Parse the imager <sensor> <pose> PITCH (5th number, rad; NEGATIVE = boresight
# UP) from the variant SDF and export it so the gt chain (m2_detect /
# capture_flight_frames) projects boxes in the ACTUAL tilted sensor frame. WITHOUT
# this the labeler is tilt-BLIND and every gt box is level-projected -- the bug
# that invalidated the up35 capture (Fable round-4 check, ADR-0076 add #18k).
# Take the <pose> INSIDE the <sensor> block (not the model/inertial/visual poses).
SENSOR_POSE_LINE="$(awk '/<sensor /{ins=1} ins && /<pose>/{print; exit}' "$SRC/model.sdf")"
SENSOR_PITCH_RAD="$(printf '%s' "$SENSOR_POSE_LINE" | grep -oP '<pose>\s*[-0-9.]+\s+[-0-9.]+\s+[-0-9.]+\s+[-0-9.]+\s+\K[-0-9.]+')"
export INTERCEPTOR_SENSOR_PITCH_RAD="${SENSOR_PITCH_RAD:-0}"
echo "[uptilt] applied $TILT via models/mono_cam symlink; sensor pitch=${INTERCEPTOR_SENSOR_PITCH_RAD} rad; running: $*"
"$@"
