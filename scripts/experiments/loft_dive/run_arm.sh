#!/usr/bin/env bash
# =============================================================================
# run_arm.sh -- Phase A loft-dive/accel-cap Gazebo A/B launcher
# (companion to gazebo_run_recipe.md; the recipe's commands, made executable).
#
# Arms:
#   smokeB  n=1 l2r  ARM B config  -- flag-wiring smoke test (boots 1 flight)
#   A       n=8 both ARM A BASELINE (flat dash control)
#   B       n=8 both ARM B LEVER (accel-cap 3.57 + loft 2 m + dive 2.5 s +
#                                 vvert 3.0 + 10 deg wedge)
#   Adash   n=8 both ARM A + CAMERA TERMINAL DISABLED (anti-mirage dash-only
#   Bdash   n=8 both ARM B + control, ADR-0076 add #18h mechanism:
#                 --coded-dash-acquire-range-min gates the acquire streak shut
#                 so handoff NEVER fires -> pure coded-dash ballistics through
#                 CPA. #18h used min=10 (that geometry's detections all implied
#                 <10 m); HERE min=999 because ARM B by design gets real 8-18 m
#                 implied-range detections that min=10 would NOT block. Same
#                 flag, same code path (update_coded_dash_streak), threshold
#                 chosen to guarantee the documented no-handoff behavior.)
#
# Usage: scripts/experiments/loft_dive/run_arm.sh {smokeB|A|B|Adash|Bdash} [--dry-run|--go]
#        (default --dry-run: mc_batch plan print, boots nothing)
#
# DEVIATIONS from the recipe text (disclosed):
#   * recipe says "-n 8"; mc_batch.sh only parses "--n" -- fixed here.
#   * --cam-mount-up-deg 10 is MATH COMPENSATION only (m4_intercept.py flag
#     help: "MUST match the physically fitted mount"). The PHYSICAL 10 deg
#     wedge is fitted the same way as scripts/uptilt_ab_arm.sh: a
#     models/mono_cam symlink -> scripts/experiments/uptilt_mounts/up10
#     (generated from up15, only the imager <pose> pitch differs). This
#     script owns the symlink lifecycle: create -> fly -> ALWAYS remove
#     (trap on EXIT). ARM A refuses to fly if a shadow exists.
#
# Env mirrors scripts/coded_dash_arm.sh (the canonical coded-dash markerless
# arm that produced logs/mc_coded_dash_qv2_line9_s123.csv): quad_enemy world,
# fpv_quad_enemy target, markerless seeker on drone_finetuned_quad_v2.onnx,
# orient-to-velocity ON.
#
# Safety rails (mirroring scripts/mc_deployment_arm.sh; each traces to a
# logged incident): one-sim-at-a-time pgrep guard + kill patterns live in
# THIS FILE (inline self-match footgun), idle-load gate, dmesg dxgk
# baseline/post counts, foreground mc_batch (launch this script detached;
# wait on the "Batch complete" sentinel in the stdout log ONLY).
# =============================================================================
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO"

ARM="${1:?arm: smokeB|A|B}"
MODE="${2:---dry-run}"
LOAD_MAX="${LOAD_MAX:-3.0}"

# --- markerless coded-dash env (mirrors scripts/coded_dash_arm.sh) ----------
export MC_WORLD="quad_enemy"
export MC_TARGET_MODEL="fpv_quad_enemy"
export MC_SEEKER="markerless"
export MC_VENV_PYTHON="$REPO/.venv-seeker/bin/python"
export MARKERLESS_NN_WEIGHTS="$REPO/scripts/seeker/weights/drone_finetuned_quad_v2.onnx"
export INTERCEPTOR_ORIENT_TO_VEL=1
export INTERCEPTOR_ORIENT_YAW_OFFSET_DEG=0
export INTERCEPTOR_WEAVE_MIRROR=1

BASE_EXTRA="--coded-dash --fpv --dash-unclamp --dash-speed 16 --dash-crossing-bias-deg 30"
LEVER_EXTRA="--dash-accel-cap 3.57 --dash-loft-m 2 --dash-loft-dive-s 2.5 --dash-vvert-max 3.0 --cam-mount-up-deg 10"
# Anti-mirage dash-only control (ADR-0076 add #18h): acquire gate shut ->
# handoff never fires -> ENGAGE never runs -> pure dash ballistics through CPA.
DASHONLY_EXTRA="--coded-dash-acquire-range-min 999"

SHADOW="$REPO/models/mono_cam"
UP10="$REPO/scripts/experiments/uptilt_mounts/up10/mono_cam"

NEED_SHADOW=0
case "$ARM" in
    smokeB) EXTRA="$BASE_EXTRA $LEVER_EXTRA"; N=1; DIRS=l2r
            OUT="logs/mc_loftdive_smokeB.csv"; NEED_SHADOW=1;;
    A)      EXTRA="$BASE_EXTRA"; N=8; DIRS=both
            OUT="logs/mc_loftdive_armA_line9_s123.csv";;
    B)      EXTRA="$BASE_EXTRA $LEVER_EXTRA"; N=8; DIRS=both
            OUT="logs/mc_loftdive_armB_line9_s123.csv"; NEED_SHADOW=1;;
    Adash)  EXTRA="$BASE_EXTRA $DASHONLY_EXTRA"; N=8; DIRS=both
            OUT="logs/mc_loftdive_armAdash_line9_s123.csv";;
    Bdash)  EXTRA="$BASE_EXTRA $LEVER_EXTRA $DASHONLY_EXTRA"; N=8; DIRS=both
            OUT="logs/mc_loftdive_armBdash_line9_s123.csv"; NEED_SHADOW=1;;
    *) echo "[loft-arm] FAIL: unknown arm '$ARM' (smokeB|A|B|Adash|Bdash)" >&2; exit 2;;
esac

CMD=(scripts/mc_batch.sh
     --mode m4 --laws pronav --path line --geometry standard
     --x0 6.5 --y0-mag 15.343 --speeds 9.0
     --directions "$DIRS" --n "$N" --master-seed 123
     --extra-args "$EXTRA"
     --out "$OUT")
STDOUT_LOG="${OUT%.csv}_stdout.log"

# --- preflight --------------------------------------------------------------
fail=0
for f in scripts/mc_batch.sh "$MC_VENV_PYTHON" "$MARKERLESS_NN_WEIGHTS" \
         worlds/quad_enemy.sdf models/fpv_quad_enemy "$UP10"; do
    [[ -e "$f" ]] || { echo "[loft-arm] MISSING: $f" >&2; fail=1; }
done
if [[ "$NEED_SHADOW" -eq 0 && -e "$SHADOW" ]]; then
    echo "[loft-arm] FAIL: $SHADOW exists -- a leftover up-tilt shadow would" >&2
    echo "           silently tilt the BASELINE arm's camera. Remove it first" >&2
    echo "           (scripts/uptilt_ab_arm.sh --cleanup)." >&2
    fail=1
fi
if [[ "$NEED_SHADOW" -eq 1 && -e "$SHADOW" && ! -L "$SHADOW" ]]; then
    echo "[loft-arm] FAIL: $SHADOW exists and is NOT a symlink -- refusing." >&2
    fail=1
fi
[[ "$fail" -eq 0 ]] || exit 1

print_plan() {
    echo "[loft-arm] Phase A loft-dive Gazebo arm '$ARM' (gazebo_run_recipe.md)"
    echo "[loft-arm] env: MC_WORLD=$MC_WORLD MC_TARGET_MODEL=$MC_TARGET_MODEL MC_SEEKER=$MC_SEEKER"
    echo "[loft-arm]      MC_VENV_PYTHON=$MC_VENV_PYTHON"
    echo "[loft-arm]      MARKERLESS_NN_WEIGHTS=$MARKERLESS_NN_WEIGHTS"
    echo "[loft-arm]      ORIENT_TO_VEL=$INTERCEPTOR_ORIENT_TO_VEL WEAVE_MIRROR=$INTERCEPTOR_WEAVE_MIRROR"
    echo "[loft-arm] physical wedge: $([[ "$NEED_SHADOW" -eq 1 ]] && echo "up10 shadow -> $UP10" || echo "NONE (level camera)")"
    { printf '[loft-arm] cmd: '; printf '%q ' "${CMD[@]}"; echo; }
    echo "[loft-arm] stdout log: $STDOUT_LOG"
    echo "[loft-arm] wait on:  grep -q 'Batch complete' $STDOUT_LOG"
}

cleanup_shadow() {
    if [[ -L "$SHADOW" ]]; then
        rm -f "$SHADOW"
        echo "[loft-arm] removed shadow symlink $SHADOW"
    fi
}

case "$MODE" in
--dry-run)
    print_plan
    "${CMD[@]}" --dry-run
    ;;
--go)
    # One sim at a time (pattern lives in this FILE -- immune to the inline
    # self-match footgun, CLAUDE.md).
    if pgrep -f "bin/px4" >/dev/null 2>&1; then
        echo "[loft-arm] FAIL: a px4 process is already running -- ONE sim at a time." >&2
        exit 1
    fi
    load1="$(cut -d' ' -f1 /proc/loadavg)"
    if awk -v l="$load1" -v m="$LOAD_MAX" 'BEGIN{exit !(l>m)}'; then
        echo "[loft-arm] FAIL: 1-min load $load1 > $LOAD_MAX -- batches at IDLE only." >&2
        exit 1
    fi
    DXGK_BASE="$( (dmesg 2>/dev/null | grep -icE dxgk) || echo n/a)"
    echo "[loft-arm] dxgk baseline: $DXGK_BASE   load1: $load1"
    if [[ "$NEED_SHADOW" -eq 1 ]]; then
        trap cleanup_shadow EXIT
        ln -sfn "$UP10" "$SHADOW"
        echo "[loft-arm] up-tilt shadow ACTIVE: $SHADOW -> $(readlink "$SHADOW")"
    fi
    print_plan
    echo "[loft-arm] LAUNCHING (foreground; run this script detached)..."
    set +e
    "${CMD[@]}" > "$STDOUT_LOG" 2>&1
    rc=$?
    set -e
    DXGK_POST="$( (dmesg 2>/dev/null | grep -icE dxgk) || echo n/a)"
    echo "[loft-arm] mc_batch exit=$rc   dxgk post: $DXGK_POST (baseline $DXGK_BASE)"
    if grep -q "Batch complete" "$STDOUT_LOG"; then
        echo "[loft-arm] SENTINEL OK: Batch complete."
    else
        echo "[loft-arm] WARNING: no 'Batch complete' sentinel in $STDOUT_LOG" >&2
    fi
    ;;
*) echo "[loft-arm] FAIL: mode must be --dry-run or --go (got '$MODE')" >&2; exit 2;;
esac
