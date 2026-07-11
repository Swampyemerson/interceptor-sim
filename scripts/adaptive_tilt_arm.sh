#!/usr/bin/env bash
# =============================================================================
# adaptive_tilt_arm.sh -- #46 adaptive camera tilt (ADR-0065): fly the deployment
# config WITH the pitch-gimbal airframe + m4 --adaptive-tilt.
#
# WHY THE PX4 MODEL SWAP (builder-approved 2026-07-10, "reversible PX4 symlink"):
#   PX4 spawns the airframe via a HARDCODED file://${PX4_GZ_MODELS}/x500_mono_cam/
#   model.sdf (px4-rc.gzsim), which BYPASSES GZ_SIM_RESOURCE_PATH -- so the #40
#   in-project models/ shadow does NOT reach the top-level airframe. To fly the
#   gimbal variant we must make PX4's stock x500_mono_cam/model.sdf BE our variant
#   for the run. This script owns that swap: back up the stock -> swap in our
#   variant -> fly -> ALWAYS restore (trap on EXIT), with auto-recovery of a
#   stranded backup from a killed prior run. Only a PX4 *simulation-model* file is
#   touched, reversibly; nothing else in ~/PX4-Autopilot changes.
#
#   The gimbal variant keeps the model NAME 'x500_mono_cam' + the mono_cam camera
#   (1280x960, same intrinsics + camera_link image topic), so the instance name
#   (x500_mono_cam_0), the NN pipeline, and gt scoring are unchanged -- only the
#   CameraJoint is now a controlled revolute pitch joint on /gimbal/pitch_cmd,
#   which m4 --adaptive-tilt drives from own-state EKF pitch.
#
# USAGE (default ECHO-ONLY; nothing runs, no swap):
#   scripts/adaptive_tilt_arm.sh --path weave --n 8 --out logs/mc_X.csv            # echo
#   scripts/adaptive_tilt_arm.sh --path weave --n 8 --out logs/mc_X.csv --go       # fly
#   scripts/adaptive_tilt_arm.sh --restore                                         # clear a stranded swap
# Flags: --path {line,weave,jink} --n N --out logs/NAME.csv [--lead D=3] [--max D=35]
#        [--go] [--restore]
# =============================================================================
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"

STOCK="$HOME/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam/model.sdf"
BACKUP="${STOCK}.adaptivetilt_stock_backup"
VARIANT="$SCRIPT_DIR/experiments/gimbal_mount/x500_mono_cam/model.sdf"

PATH_MODE="weave"; N=""; OUT_CSV=""; LEAD=3.0; MAXDEG=35.0; ACTION="echo"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --path) PATH_MODE="$2"; shift 2;;
        --n) N="$2"; shift 2;;
        --out) OUT_CSV="$2"; shift 2;;
        --lead) LEAD="$2"; shift 2;;
        --max) MAXDEG="$2"; shift 2;;
        --go) ACTION="go"; shift;;
        --restore) ACTION="restore"; shift;;
        *) echo "[atilt] FAIL: unknown arg '$1'" >&2; exit 2;;
    esac
done

restore_stock() {
    # Restore the canonical PX4 airframe model. Idempotent + robust: if the backup
    # exists (mid-swap or stranded), move it back over the (possibly swapped) stock.
    if [[ -f "$BACKUP" ]]; then
        mv -f "$BACKUP" "$STOCK"
        echo "[atilt] restored stock PX4 x500_mono_cam/model.sdf"
    fi
}

if [[ "$ACTION" == "restore" ]]; then
    restore_stock
    [[ -f "$BACKUP" ]] || echo "[atilt] clean -- no stranded swap."
    exit 0
fi

# ---- preflight -------------------------------------------------------------
fail=0
[[ -f "$VARIANT" ]] || { echo "[atilt] MISSING gimbal variant: $VARIANT" >&2; fail=1; }
[[ -f "$STOCK" || -f "$BACKUP" ]] || { echo "[atilt] MISSING PX4 stock model: $STOCK" >&2; fail=1; }
[[ -n "$N" && -n "$OUT_CSV" ]] || { echo "[atilt] FAIL: --n and --out are required" >&2; fail=1; }
[[ "$fail" -eq 0 ]] || exit 1

# Auto-recover a STRANDED swap (a prior run killed before its trap restored):
# if a backup exists now, that stock is our variant -> restore before anything.
if [[ -f "$BACKUP" ]]; then
    echo "[atilt] WARN: stranded backup found (a prior run didn't restore) -- auto-restoring first."
    restore_stock
fi

print_plan() {
    echo "[atilt] #46 adaptive tilt arm (gimbal airframe + m4 --adaptive-tilt)"
    echo "    swap : $STOCK  <=  $VARIANT  (backup -> $BACKUP, restored on exit)"
    echo "    m4   : --adaptive-tilt --adaptive-tilt-lead-deg $LEAD --adaptive-tilt-max-deg $MAXDEG"
    echo "    fly  : mc_deployment_arm --path $PATH_MODE --n $N --out $OUT_CSV"
}

if [[ "$ACTION" != "go" ]]; then
    print_plan
    echo "[atilt] ECHO-ONLY (default). Pass --go to fly (idle load only)."
    exit 0
fi

if [[ -e "$OUT_CSV" ]]; then
    echo "[atilt] FAIL: $OUT_CSV already exists -- evidence protection." >&2; exit 1
fi
if pgrep -f "bin/px4" >/dev/null 2>&1; then
    echo "[atilt] FAIL: a px4 process is already running -- ONE sim at a time." >&2; exit 1
fi
print_plan

trap restore_stock EXIT
cp -f "$STOCK" "$BACKUP"        # back up the real stock
cp -f "$VARIANT" "$STOCK"       # swap in our gimbal variant (real file; includes resolve via GZ path)
echo "[atilt] gimbal airframe SWAPPED IN (CameraJoint revolute). Verify + fly..."

# mc_deployment_arm owns the idle/one-sim/dxgk guards + the r2-mirrored deployment
# argv; --m4-extra appends the adaptive-tilt flags (disclosed in its plan print).
M4X="--adaptive-tilt --adaptive-tilt-lead-deg $LEAD --adaptive-tilt-max-deg $MAXDEG"
set +e
scripts/mc_deployment_arm.sh --path "$PATH_MODE" --n "$N" --master-seed 42 \
    --out "$OUT_CSV" --m4-extra "$M4X" --go
rc=$?
set -e
restore_stock
echo "[atilt] mc_deployment_arm exit=$rc; stock restored."
echo "[atilt] analyze: .venv/bin/python scripts/mc_analyze.py $OUT_CSV ; audit_per_tick.py $OUT_CSV"
exit "$rc"
