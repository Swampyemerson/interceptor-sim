#!/usr/bin/env bash
# =============================================================================
# run_arm.sh -- FLIGHT-PLAN CANDIDATES Gazebo A/B launcher (PROPOSED, UNFLOWN)
#
# Extends scripts/experiments/loft_dive/run_arm.sh (Phase A) with the candidate
# flight plans pre-registered in docs/flight_plan_candidates.md. Same rails,
# same env, same canonical line-9 geometry -- only the arm table and the seed
# handling are new.
#
#   *** NOTHING IN HERE HAS BEEN FLOWN. Every arm is PRE-REGISTERED: its
#   *** prediction and its adopt/reject criterion are written in
#   *** docs/flight_plan_candidates.md BEFORE the run, so a result cannot be
#   *** rationalised afterwards. The head serialises all sim (one at a time,
#   *** idle load); launch this script detached and wait on the sentinel.
#
# ARMS (see docs/flight_plan_candidates.md for the physics + criteria)
#   -- Track 1: AIM (accel-consistent lead). Zero code, biggest predicted win.
#   G20dash  n=8 both  baseline dash, crossing bias 20, CAMERA DISABLED
#                      -> the clean ballistic test of the aim model
#   G20      n=8 both  baseline dash, crossing bias 20, camera terminal live
#   -- Track 2: POINTING without the closing-speed toll.
#   C        n=8 both  LOFT-ONLY (loft 2 m + 10 deg wedge, NO accel cap)
#   Cdash    n=8 both  ... camera disabled (anti-mirage control)
#   C3       n=8 both  deeper loft (3 m / 3.5 s dive), 10 deg wedge
#   C15      n=8 both  loft-only with the 15 deg wedge (contradiction band max)
#   CG       n=8 both  loft-only + bias 20 (Track 1 x Track 2 combination)
#   CGdash   n=8 both  ... camera disabled
#   -- Track 3: rehabilitate the accel cap by re-sizing its aim.
#   E        n=8 both  ARM B (cap 3.57 + loft + wedge) with bias 64
#   Edash    n=8 both  ... camera disabled
#   D        n=8 both  light cap 5.66 (theta 30) + loft + wedge, bias 37
#   -- smoke
#   smokeC   n=1 l2r   ARM C config, flag-wiring smoke test
#
# Usage: scripts/experiments/flight_plans/run_arm.sh ARM [--dry-run|--go] [SEED]
#        (default --dry-run: prints the mc_batch plan, boots nothing;
#         default SEED 123 -- use 777 for the disjoint replication)
#
# DASH-ONLY CONTROL (anti-mirage, ADR-0076 add #18g/#18h): the *dash arms add
# --coded-dash-acquire-range-min 999, which gates the acquire streak shut so
# handoff NEVER fires -> zero ENGAGE ticks -> the logged miss IS the open-loop
# dash ballistic CPA. Verified on the flown Adash/Bdash arms (0 ENGAGE ticks,
# miss_m == min gt_range on 8/8 flights).
#
# PHYSICAL WEDGE: --cam-mount-up-deg is MATH compensation only; the physical
# wedge is a models/mono_cam symlink -> scripts/experiments/uptilt_mounts/upNN.
# This script owns the lifecycle (create -> fly -> ALWAYS remove, trap EXIT) and
# refuses to fly a level-camera arm if a shadow is present.
#
# Safety rails (each traces to a logged incident; copied from the loft_dive
# launcher): one-sim-at-a-time pgrep guard with the pattern INSIDE THIS FILE
# (inline self-match footgun, CLAUDE.md), idle-load gate, dmesg dxgk counts,
# foreground mc_batch (launch detached; wait on "Batch complete").
# =============================================================================
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO"

ARM="${1:?arm: see the header for the arm table}"
MODE="${2:---dry-run}"
SEED="${3:-123}"
LOAD_MAX="${LOAD_MAX:-3.0}"

# --- markerless coded-dash env (identical to loft_dive/run_arm.sh) ----------
export MC_WORLD="quad_enemy"
export MC_TARGET_MODEL="fpv_quad_enemy"
export MC_SEEKER="markerless"
export MC_VENV_PYTHON="$REPO/.venv-seeker/bin/python"
export MARKERLESS_NN_WEIGHTS="$REPO/scripts/seeker/weights/drone_finetuned_quad_v2.onnx"
export INTERCEPTOR_ORIENT_TO_VEL=1
export INTERCEPTOR_ORIENT_YAW_OFFSET_DEG=0
export INTERCEPTOR_WEAVE_MIRROR=1

# BASE without the crossing bias: every candidate sets its own bias, because
# the ballistically-correct bias is a FUNCTION of the dash accel
# (scripts/experiments/flight_plans/dash_cpa_model.py --sweep).
BASE="--coded-dash --fpv --dash-unclamp --dash-speed 16"
LOFT="--dash-loft-m 2 --dash-loft-dive-s 2.5 --dash-vvert-max 3.0"
LOFT3="--dash-loft-m 3 --dash-loft-dive-s 3.5 --dash-vvert-max 3.0"
DASHONLY="--coded-dash-acquire-range-min 999"

SHADOW="$REPO/models/mono_cam"
MOUNT_DIR="$REPO/scripts/experiments/uptilt_mounts"

WEDGE=0          # physical wedge (deg); 0 = level camera, no shadow
case "$ARM" in
  # ---- Track 1: accel-consistent AIM (no pointing change) -----------------
  G20)     EXTRA="$BASE --dash-crossing-bias-deg 20"; N=8; DIRS=both;;
  G20dash) EXTRA="$BASE --dash-crossing-bias-deg 20 $DASHONLY"; N=8; DIRS=both;;
  # LEVEL + root-cause LOS-rate lag (direction-agnostic, self-signing) on the BEST arm.
  # G20-level is fast-closing where the ~190 ms lag dominates the terminal error.
  GL)      EXTRA="$BASE --dash-crossing-bias-deg 20 --terminal-los-lag-ms 190"; N=8; DIRS=both;;
  # Accel-aware collision-lead (ADR-0080): DERIVES the aim instead of the hand-set +20.
  # Dash-only -> tests that auto-aim reproduces G20dash's 0.75 m WITHOUT a manual bias.
  AAL)     EXTRA="$BASE --dash-accel-aware-lead $DASHONLY"; N=8; DIRS=both;;
  # AIM-ERROR sweep (the review's last open sim question): does the CAMERA earn its
  # handoff when the open-loop aim is WRONG by a field-realistic amount? At 0 deg
  # error the camera is a slight liability (G20 vs G20dash); the real world never
  # gets 0. Each camera arm ships its dash-only twin -> the claim is the DIFFERENCE.
  # Pre-registered: camera earns it iff it beats its own twin >=6/8, replicated on 777.
  AE5)     EXTRA="$BASE --dash-accel-aware-lead --dash-heading-err-deg 5"; N=8; DIRS=both;;
  AE5dash) EXTRA="$BASE --dash-accel-aware-lead --dash-heading-err-deg 5 $DASHONLY"; N=8; DIRS=both;;
  AE15)    EXTRA="$BASE --dash-accel-aware-lead --dash-heading-err-deg 15"; N=8; DIRS=both;;
  AE15dash) EXTRA="$BASE --dash-accel-aware-lead --dash-heading-err-deg 15 $DASHONLY"; N=8; DIRS=both;;
  # RESIDUAL-BIAS fine sweep (dash-only): the aim curve measured 0 deg -> 0.71 m,
  # +5 -> 0.43, +15 -> 1.43, so the accel-aware lead sits ~5 deg off a real optimum
  # worth ~0.28 m. These bracket it so the optimum can be DERIVED, not hand-found.
  # (Matters because the 5-inch ram envelope is ~0.36 m -- see contract stage `kill`.)
  AE2dash)  EXTRA="$BASE --dash-accel-aware-lead --dash-heading-err-deg 2.5 $DASHONLY"; N=8; DIRS=both;;
  AE7dash)  EXTRA="$BASE --dash-accel-aware-lead --dash-heading-err-deg 7.5 $DASHONLY"; N=8; DIRS=both;;
  AE10dash) EXTRA="$BASE --dash-accel-aware-lead --dash-heading-err-deg 10 $DASHONLY"; N=8; DIRS=both;;
  # Accel-CAP at CORRECT aim (never flown): cap 3.57 + accel-aware lead sized to the
  # ACHIEVED a=4.0. Decouples the cap's closing-speed cost from the aim confound
  # (the 2026-07-24 Bdash correction). Compare to AAL (uncapped, correct aim, 0.71).
  EAL)     EXTRA="$BASE --dash-accel-cap 3.57 --dash-accel-aware-lead --dash-lead-accel-ms2 4.0 $DASHONLY"
           N=8; DIRS=both;;
  G25)     EXTRA="$BASE --dash-crossing-bias-deg 25"; N=8; DIRS=both;;
  G25dash) EXTRA="$BASE --dash-crossing-bias-deg 25 $DASHONLY"; N=8; DIRS=both;;
  # ---- Track 2: pointing without the accel cap ----------------------------
  smokeC)  EXTRA="$BASE --dash-crossing-bias-deg 30 $LOFT --cam-mount-up-deg 10"
           N=1; DIRS=l2r; WEDGE=10;;
  C)       EXTRA="$BASE --dash-crossing-bias-deg 30 $LOFT --cam-mount-up-deg 10"
           N=8; DIRS=both; WEDGE=10;;
  Cdash)   EXTRA="$BASE --dash-crossing-bias-deg 30 $LOFT --cam-mount-up-deg 10 $DASHONLY"
           N=8; DIRS=both; WEDGE=10;;
  C3)      EXTRA="$BASE --dash-crossing-bias-deg 30 $LOFT3 --cam-mount-up-deg 10"
           N=8; DIRS=both; WEDGE=10;;
  C15)     EXTRA="$BASE --dash-crossing-bias-deg 30 $LOFT --cam-mount-up-deg 15"
           N=8; DIRS=both; WEDGE=15;;
  CG)      EXTRA="$BASE --dash-crossing-bias-deg 20 $LOFT --cam-mount-up-deg 10"
           N=8; DIRS=both; WEDGE=10;;
  CGdash)  EXTRA="$BASE --dash-crossing-bias-deg 20 $LOFT --cam-mount-up-deg 10 $DASHONLY"
           N=8; DIRS=both; WEDGE=10;;
  # Aspect-bias compensation on the FRAMED regime (CG + comp). Constants +11.0/-17.8
  # calibrated on ARM B seed 123 (LOO -79% stable); validate on DISJOINT seed 777.
  CH)      EXTRA="$BASE --dash-crossing-bias-deg 20 $LOFT --cam-mount-up-deg 10 --terminal-bearing-bias-l2r-deg 11.0 --terminal-bearing-bias-r2l-deg -17.8"
           N=8; DIRS=both; WEDGE=10;;
  # Root-cause LOS-rate lag variant (direction-agnostic, self-signing, ~190 ms).
  CHL)     EXTRA="$BASE --dash-crossing-bias-deg 20 $LOFT --cam-mount-up-deg 10 --terminal-los-lag-ms 190"
           N=8; DIRS=both; WEDGE=10;;
  # ---- Track 3: re-aimed accel cap ----------------------------------------
  E)       EXTRA="$BASE --dash-crossing-bias-deg 64 --dash-accel-cap 3.57 $LOFT --cam-mount-up-deg 10"
           N=8; DIRS=both; WEDGE=10;;
  Edash)   EXTRA="$BASE --dash-crossing-bias-deg 64 --dash-accel-cap 3.57 $LOFT --cam-mount-up-deg 10 $DASHONLY"
           N=8; DIRS=both; WEDGE=10;;
  D)       EXTRA="$BASE --dash-crossing-bias-deg 37 --dash-accel-cap 5.66 $LOFT --cam-mount-up-deg 10"
           N=8; DIRS=both; WEDGE=10;;
  Ddash)   EXTRA="$BASE --dash-crossing-bias-deg 37 --dash-accel-cap 5.66 $LOFT --cam-mount-up-deg 10 $DASHONLY"
           N=8; DIRS=both; WEDGE=10;;
  *) echo "[fp-arm] FAIL: unknown arm '$ARM' (see the header table)" >&2; exit 2;;
esac

OUT="logs/mc_fp_arm${ARM}_line9_s${SEED}.csv"
STDOUT_LOG="${OUT%.csv}_stdout.log"
UPDIR="$MOUNT_DIR/up$(printf '%02d' "$WEDGE")/mono_cam"

CMD=(scripts/mc_batch.sh
     --mode m4 --laws pronav --path line --geometry standard
     --x0 6.5 --y0-mag 15.343 --speeds 9.0
     --directions "$DIRS" --n "$N" --master-seed "$SEED"
     --extra-args "$EXTRA"
     --out "$OUT")

# --- preflight --------------------------------------------------------------
fail=0
for f in scripts/mc_batch.sh "$MC_VENV_PYTHON" "$MARKERLESS_NN_WEIGHTS" \
         worlds/quad_enemy.sdf models/fpv_quad_enemy; do
    [[ -e "$f" ]] || { echo "[fp-arm] MISSING: $f" >&2; fail=1; }
done
if [[ "$WEDGE" -ne 0 ]]; then
    [[ -e "$UPDIR" ]] || { echo "[fp-arm] MISSING wedge model: $UPDIR" >&2; fail=1; }
fi
if [[ "$WEDGE" -eq 0 && -e "$SHADOW" ]]; then
    echo "[fp-arm] FAIL: $SHADOW exists -- a leftover up-tilt shadow would" >&2
    echo "         silently tilt this LEVEL-camera arm. Remove it first" >&2
    echo "         (scripts/uptilt_ab_arm.sh --cleanup)." >&2
    fail=1
fi
if [[ "$WEDGE" -ne 0 && -e "$SHADOW" && ! -L "$SHADOW" ]]; then
    echo "[fp-arm] FAIL: $SHADOW exists and is NOT a symlink -- refusing." >&2
    fail=1
fi
[[ "$fail" -eq 0 ]] || exit 1

print_plan() {
    echo "[fp-arm] flight-plan candidate arm '$ARM' seed $SEED (docs/flight_plan_candidates.md)"
    echo "[fp-arm] env: MC_WORLD=$MC_WORLD MC_TARGET_MODEL=$MC_TARGET_MODEL MC_SEEKER=$MC_SEEKER"
    echo "[fp-arm]      ORIENT_TO_VEL=$INTERCEPTOR_ORIENT_TO_VEL WEAVE_MIRROR=$INTERCEPTOR_WEAVE_MIRROR"
    echo "[fp-arm] physical wedge: $([[ "$WEDGE" -ne 0 ]] && echo "up${WEDGE} shadow -> $UPDIR" || echo "NONE (level camera)")"
    echo "[fp-arm] camera terminal: $([[ "$EXTRA" == *"acquire-range-min 999"* ]] && echo "DISABLED (dash-only control)" || echo "LIVE")"
    { printf '[fp-arm] cmd: '; printf '%q ' "${CMD[@]}"; echo; }
    echo "[fp-arm] stdout log: $STDOUT_LOG"
    echo "[fp-arm] wait on:  grep -q 'Batch complete' $STDOUT_LOG"
    echo "[fp-arm] parse:    .venv/bin/python scripts/experiments/loft_dive/parse_ab.py $OUT=$WEDGE"
}

cleanup_shadow() {
    if [[ -L "$SHADOW" ]]; then
        rm -f "$SHADOW"
        echo "[fp-arm] removed shadow symlink $SHADOW"
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
        echo "[fp-arm] FAIL: a px4 process is already running -- ONE sim at a time." >&2
        exit 1
    fi
    load1="$(cut -d' ' -f1 /proc/loadavg)"
    if awk -v l="$load1" -v m="$LOAD_MAX" 'BEGIN{exit !(l>m)}'; then
        echo "[fp-arm] FAIL: 1-min load $load1 > $LOAD_MAX -- batches at IDLE only." >&2
        exit 1
    fi
    DXGK_BASE="$( (dmesg 2>/dev/null | grep -icE dxgk) || echo n/a)"
    echo "[fp-arm] dxgk baseline: $DXGK_BASE   load1: $load1"
    if [[ "$WEDGE" -ne 0 ]]; then
        trap cleanup_shadow EXIT
        ln -sfn "$UPDIR" "$SHADOW"
        echo "[fp-arm] up-tilt shadow ACTIVE: $SHADOW -> $(readlink "$SHADOW")"
    fi
    print_plan
    echo "[fp-arm] LAUNCHING (foreground; run this script detached)..."
    set +e
    "${CMD[@]}" > "$STDOUT_LOG" 2>&1
    rc=$?
    set -e
    DXGK_POST="$( (dmesg 2>/dev/null | grep -icE dxgk) || echo n/a)"
    echo "[fp-arm] mc_batch exit=$rc   dxgk post: $DXGK_POST (baseline $DXGK_BASE)"
    if grep -q "Batch complete" "$STDOUT_LOG"; then
        echo "[fp-arm] SENTINEL OK: Batch complete."
    else
        echo "[fp-arm] WARNING: no 'Batch complete' sentinel in $STDOUT_LOG" >&2
    fi
    ;;
*) echo "[fp-arm] FAIL: mode must be --dry-run or --go (got '$MODE')" >&2; exit 2;;
esac
