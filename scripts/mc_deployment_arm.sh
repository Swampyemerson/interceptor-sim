#!/usr/bin/env bash
# =============================================================================
# mc_deployment_arm.sh -- guarded launcher for ONE mc_batch arm flying the
# ADOPTED ADR-0058 deployment config on the markerless seeker.
#
# WHAT THIS VALIDATES / WHY IT EXISTS (2026-07-10, sim-gated-queue prep):
#   Every arm in the post-train idle queue (jink n=16 post-fix, weave n=72
#   stats hardening, up-tilt mount A/B) is the SAME deployment configuration:
#     m4 extra-args : --dash-speed 16 --early-handoff --cue-velocity
#                     --dash-unclamp --fuse-midcourse --track --handoff-cue-gate 8
#     markerless env: MC_WORLD/MC_TARGET_MODEL/MC_SEEKER/MC_VENV_PYTHON
#                     + MARKERLESS_NN_WEIGHTS (drone_finetuned_v2.onnx)
#     realistic cue : S2_CUE_MOCK_EXTRA (sigma-range, datum-bias 0.5,
#                     latency-jitter 0.05, dropout-markov, emit-velocity 0.5)
#   mirrored EXACTLY from the ADR-0058 r2 headline arm
#   (logs/mc_t21_trackgate_weave12_r2_stdout.log -- m4 argv order preserved).
#   The r1 headline attempt was invalidated by a relaunch that dropped this
#   env (16 argparse-rejected boots, ADR-0058 provenance note). Baking env +
#   argv into ONE script file is the fix: no more rebuilt-from-template runs.
#
# USAGE (default is ECHO-ONLY -- prints env + command, runs nothing):
#   scripts/mc_deployment_arm.sh --path jink --n 16 --out logs/mc_X.csv           # echo only
#   scripts/mc_deployment_arm.sh --path jink --n 16 --out logs/mc_X.csv --dry-run # mc_batch plan print (NO sim boots)
#   scripts/mc_deployment_arm.sh --path jink --n 16 --out logs/mc_X.csv --go      # actually fly (idle load only)
# Flags: --path {line,weave,jink}  --n N  --master-seed S (42)  --speeds V (12)
#        --out logs/NAME.csv  [--dry-run | --go]  [--force]
#
# SAFETY RAILS (each traces to a logged incident):
#   * ONE sim at a time: refuses if a live px4 process exists (mc-batch skill).
#   * Idle-load gate: refuses --go when 1-min loadavg > LOAD_MAX (default 3.0)
#     -- batches at IDLE machine load only (ADR-0015 2nd addendum). The v3
#     CPU fine-tune shows ~8.0; a truly idle box shows < 1. --force overrides.
#   * Up-tilt shadow guard: refuses if models/mono_cam exists (a leftover
#     up-tilt A/B symlink would silently tilt EVERY camera in this arm).
#     scripts/uptilt_ab_arm.sh sets UPTILT_EXPECTED=1 when the shadow is
#     intentional. Cleanup: scripts/uptilt_ab_arm.sh --cleanup
#   * dmesg dxgk baseline/post counts logged (GPU-wedge watch, mc-batch skill).
#   * Kill/poll patterns live in THIS FILE, never inline in a tool call.
#   * Runs mc_batch in the foreground: launch THIS script via a tracked
#     background Bash call; wait on the sentinel string "Batch complete" in
#     the stdout log ONLY (never pgrep/pkill with sim-name substrings inline).
#
# Post-run analysis (printed at the end):
#   .venv/bin/python scripts/mc_analyze.py <out.csv>
#   .venv/bin/python scripts/analyze_track_ab.py <out.csv> [comparators...]
#   .venv/bin/python scripts/audit_per_tick.py <out.csv>        # honesty bar
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"

LOAD_MAX="${LOAD_MAX:-3.0}"

PATH_MODE=""
N=""
MASTER_SEED=42
SPEEDS=12
OUT_CSV=""
ACTION="echo"
FORCE=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --path) PATH_MODE="$2"; shift 2;;
        --n) N="$2"; shift 2;;
        --master-seed) MASTER_SEED="$2"; shift 2;;
        --speeds) SPEEDS="$2"; shift 2;;
        --out) OUT_CSV="$2"; shift 2;;
        --dry-run) ACTION="dry-run"; shift;;
        --go) ACTION="go"; shift;;
        --force) FORCE=1; shift;;
        *) echo "[deploy-arm] FAIL: unknown arg '$1'" >&2; exit 2;;
    esac
done

if [[ -z "$PATH_MODE" || -z "$N" || -z "$OUT_CSV" ]]; then
    echo "[deploy-arm] FAIL: --path, --n and --out are required (see header)." >&2
    exit 2
fi
case "$PATH_MODE" in line|weave|jink) ;; *)
    echo "[deploy-arm] FAIL: --path must be line|weave|jink (got '$PATH_MODE')" >&2; exit 2;;
esac

# ---- the ADR-0058 deployment config (DO NOT edit casually: the exact argv
# ---- order below mirrors the r2 headline arm's logged m4 invocation) -------
EXTRA_ARGS_STR="--dash-speed 16 --early-handoff --cue-velocity --dash-unclamp --fuse-midcourse --track --handoff-cue-gate 8"

# ---- the markerless flight env (NEXT.md key-facts block + r2 stdout log) ---
export MC_WORLD="markerless"
export MC_TARGET_MODEL="fpv_target_markerless"
export MC_SEEKER="markerless"
export MC_VENV_PYTHON="$REPO_ROOT/.venv-seeker/bin/python"
export MARKERLESS_NN_WEIGHTS="$REPO_ROOT/scripts/seeker/weights/drone_finetuned_v2.onnx"
export S2_CUE_MOCK_EXTRA="--sigma-range --datum-bias-m 0.5 --latency-jitter-s 0.05 --dropout-markov --emit-velocity --vel-sigma 0.5"

CMD=(scripts/mc_batch.sh
     --laws pronav --speeds "$SPEEDS" --directions both
     --path "$PATH_MODE" --geometry standard
     --n "$N" --y0-mag 29.3 --x0 6.5 --master-seed "$MASTER_SEED"
     --extra-args "$EXTRA_ARGS_STR"
     --out "$OUT_CSV")
STDOUT_LOG="${OUT_CSV%.csv}_stdout.log"

# ---- preflight (all modes) -------------------------------------------------
fail=0
for f in scripts/mc_batch.sh "$MC_VENV_PYTHON" "$MARKERLESS_NN_WEIGHTS" \
         worlds/markerless.sdf models/fpv_target_markerless; do
    if [[ ! -e "$f" ]]; then echo "[deploy-arm] MISSING: $f" >&2; fail=1; fi
done
if [[ -e "models/mono_cam" && "${UPTILT_EXPECTED:-0}" != "1" ]]; then
    echo "[deploy-arm] FAIL: models/mono_cam exists -- a leftover up-tilt shadow" >&2
    echo "             model would silently tilt this arm's camera. Run" >&2
    echo "             scripts/uptilt_ab_arm.sh --cleanup first." >&2
    fail=1
fi
if [[ "${UPTILT_EXPECTED:-0}" == "1" ]]; then
    if [[ -L "models/mono_cam" ]]; then
        echo "[deploy-arm] up-tilt shadow ACTIVE: models/mono_cam -> $(readlink models/mono_cam)"
    else
        echo "[deploy-arm] FAIL: UPTILT_EXPECTED=1 but models/mono_cam is not a symlink." >&2
        fail=1
    fi
fi
[[ "$fail" -eq 0 ]] || exit 1

print_plan() {
    echo "[deploy-arm] ADR-0058 deployment-config arm (markerless, realistic cue)"
    echo "[deploy-arm] env:"
    echo "    MC_WORLD=$MC_WORLD"
    echo "    MC_TARGET_MODEL=$MC_TARGET_MODEL"
    echo "    MC_SEEKER=$MC_SEEKER"
    echo "    MC_VENV_PYTHON=$MC_VENV_PYTHON"
    echo "    MARKERLESS_NN_WEIGHTS=$MARKERLESS_NN_WEIGHTS"
    echo "    S2_CUE_MOCK_EXTRA=\"$S2_CUE_MOCK_EXTRA\""
    echo "[deploy-arm] command (cwd $REPO_ROOT):"
    { printf '    '; printf '%q ' "${CMD[@]}"; echo; }
    echo "[deploy-arm] stdout log: $STDOUT_LOG"
    echo "[deploy-arm] flights: $N x pronav @ ${SPEEDS} m/s, path=$PATH_MODE, master_seed=$MASTER_SEED"
    echo "[deploy-arm] wait on:  for i in \$(seq 1 120); do grep -q \"Batch complete\" $STDOUT_LOG && break; sleep 15; done"
}

case "$ACTION" in
echo)
    print_plan
    echo "[deploy-arm] ECHO-ONLY (default). --dry-run for the mc_batch plan print"
    echo "             (boots nothing), --go to fly (idle load only)."
    ;;
dry-run)
    print_plan
    echo "[deploy-arm] Running mc_batch --dry-run (plan print; no sim, no files)..."
    "${CMD[@]}" --dry-run
    ;;
go)
    # One sim at a time (pattern lives in this FILE -- immune to the inline
    # self-match footgun documented in CLAUDE.md).
    if pgrep -f "bin/px4" >/dev/null 2>&1; then
        echo "[deploy-arm] FAIL: a px4 process is already running -- ONE sim at a time." >&2
        exit 1
    fi
    load1="$(cut -d' ' -f1 /proc/loadavg)"
    if [[ "$FORCE" -ne 1 ]] && awk -v l="$load1" -v m="$LOAD_MAX" 'BEGIN{exit !(l>m)}'; then
        echo "[deploy-arm] FAIL: 1-min load $load1 > $LOAD_MAX -- batches run at IDLE only" >&2
        echo "             (ADR-0015 2nd addendum). Is the v3 fine-tune still running?" >&2
        echo "             Override with --force ONLY if you know the load source is benign." >&2
        exit 1
    fi
    DXGK_BASE="$( (dmesg 2>/dev/null | grep -icE dxgk) || echo "n/a")"
    echo "[deploy-arm] dxgk baseline: $DXGK_BASE   load1: $load1"
    print_plan
    echo "[deploy-arm] LAUNCHING (foreground; run this script backgrounded)..."
    set +e
    "${CMD[@]}" > "$STDOUT_LOG" 2>&1
    rc=$?
    set -e
    DXGK_POST="$( (dmesg 2>/dev/null | grep -icE dxgk) || echo "n/a")"
    echo "[deploy-arm] mc_batch exit=$rc   dxgk post: $DXGK_POST (baseline $DXGK_BASE)"
    if grep -q "Batch complete" "$STDOUT_LOG"; then
        echo "[deploy-arm] SENTINEL OK: Batch complete."
    else
        echo "[deploy-arm] WARNING: no 'Batch complete' sentinel in $STDOUT_LOG -- inspect it." >&2
    fi
    echo "[deploy-arm] next:"
    echo "    .venv/bin/python scripts/mc_analyze.py $OUT_CSV"
    echo "    .venv/bin/python scripts/analyze_track_ab.py $OUT_CSV"
    echo "    .venv/bin/python scripts/audit_per_tick.py $OUT_CSV"
    echo "    cooldown >=120 s + dxgk check before ANY next arm (mc-batch skill)"
    exit "$rc"
    ;;
esac
