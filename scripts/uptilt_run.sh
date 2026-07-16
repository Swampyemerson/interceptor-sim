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
echo "[uptilt] applied $TILT via models/mono_cam symlink; running: $*"
"$@"
