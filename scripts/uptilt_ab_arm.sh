#!/usr/bin/env bash
# =============================================================================
# uptilt_ab_arm.sh -- TASK #35 / ADR-0060: up-tilted camera-mount A/B sweep.
#
# WHAT THIS VALIDATES:
#   ADR-0060 measured that the dash pitches nose-down 27-36 deg (max ~52),
#   throwing the target above the FoV for ~35% of dash ticks; first in-range
#   detection threads a 0-6 deg sliver at the top edge (~zero vertical
#   margin). This sweep re-flies the ADOPTED deployment config (ADR-0058,
#   weave 12 m/s, master-seed 42 -> seed-PAIRED across mounts AND to the
#   first-n flights of the r2 headline arm) with the mono_cam sensor up-tilted
#   +15 / +25 / +35 deg, plus a +0 CONTROL arm that flies the identical swap
#   machinery with an unmodified pose (isolates any machinery effect).
#   Measures: (a) dash-detection recovery (first_det_range_m, coverage, FoV
#   fractions), (b) handoff vertical margin, (c) ENGAGE-phase pointing
#   tradeoff -- via scripts/experiments/uptilt_ab_analyze.py.
#
# HOW THE VARIANT SWAPS IN (canonical PX4 model NEVER edited):
#   PX4's gz_env.sh.in builds GZ_SIM_RESOURCE_PATH as
#   "$GZ_SIM_RESOURCE_PATH:$PX4_GZ_MODELS:..." -- user path FIRST -- and
#   mc_batch.sh launches with GZ_SIM_RESOURCE_PATH=$REPO_ROOT/models.
#   x500_mono_cam pulls the camera via `model://mono_cam`, so a
#   models/mono_cam symlink -> scripts/experiments/uptilt_mounts/upNN/mono_cam
#   shadows the canonical model FOR THAT RUN ONLY. This script owns the
#   symlink lifecycle: create -> fly -> ALWAYS remove (trap on EXIT).
#   ASSUMPTION FLAGGED FOR THE LIVE CHECK (first --go flight): shadow
#   precedence + symlink traversal. Mechanism check = up15's first-detection
#   vertical angle shifts ~ +15 deg vs control and first_det_range_m jumps
#   (r2 median was 14.3 m); if up15 matches control exactly, the shadow did
#   NOT take -- STOP and investigate resolution order.
#
#   A LEFTOVER SYMLINK WOULD SILENTLY TILT EVERY FUTURE BATCH. Defense in
#   depth: (1) trap-always cleanup here, (2) scripts/mc_deployment_arm.sh
#   hard-refuses to fly while models/mono_cam exists unless UPTILT_EXPECTED=1
#   (this script sets it), (3) `--cleanup` mode to clear a stranded link.
#
# GUIDANCE CAVEAT (pre-registered, affects metric (c) only): WITHOUT
#   --compensate, m4_intercept.py maps camera bearings to the body frame
#   assuming a ZERO mount angle, so tilted arms fly with an UNCOMPENSATED
#   elevation bias equal to the mount angle. (a)/(b) are pure perception
#   metrics and unaffected; miss/clean in tilted arms measure the
#   uncompensated worst case. That was the flown ADR-0067 sweep.
#
# --compensate (#40 mount-compose re-fly, docs/adr0067_refly_preregistration.md):
#   passes `--cam-mount-up-deg <mount>` through to m4 (via mc_deployment_arm's
#   --m4-extra), so the FIX-A derotation composes the mount rotation -- the
#   compensated A/B ADR-0067 gated everything terminal on. Output CSVs are
#   renamed mc_uptilt_refly_weave12_up{00,15c,...}.csv so the ADR-0067
#   evidence is never touched. up00 passes an explicit `--cam-mount-up-deg 0`
#   (argv-symmetric; byte-identical to no-flag by the m4 guard + tests).
#
# BUDGET: 4 arms x n=8 = 32 boots (< 48-boot proven cluster), sequential,
#   120 s cooldowns; ~10 min/arm + cooldowns ~= 50 min total at idle.
#   (The pre-registered #40 re-fly is 2 arms: --mount 0 --mount 15 --compensate.)
#
# USAGE (default ECHO-ONLY; nothing runs, no symlink touched):
#   scripts/uptilt_ab_arm.sh --all                 # print the 4-arm plan
#   scripts/uptilt_ab_arm.sh --mount 15            # print one arm's plan
#   scripts/uptilt_ab_arm.sh --mount 15 --dry-run  # mc_batch plan print (NO sim)
#   scripts/uptilt_ab_arm.sh --all --go            # fly the sweep (idle only)
#   scripts/uptilt_ab_arm.sh --mount 0 --mount 15 --compensate --go  # #40 re-fly
#   scripts/uptilt_ab_arm.sh --cleanup             # remove a stranded shadow link
# Then analyze (offline, includes r2 as the zero-mount n=16 reference):
#   .venv/bin/python scripts/experiments/uptilt_ab_analyze.py \
#       logs/mc_uptilt_weave12_up00.csv logs/mc_uptilt_weave12_up15.csv \
#       logs/mc_uptilt_weave12_up25.csv logs/mc_uptilt_weave12_up35.csv \
#       --baseline logs/mc_t21_trackgate_weave12_r2.csv
# =============================================================================
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"

VARIANT_ROOT="$SCRIPT_DIR/experiments/uptilt_mounts"
SHADOW="$REPO_ROOT/models/mono_cam"
LAUNCH="$SCRIPT_DIR/mc_deployment_arm.sh"

MOUNTS=()
N=8
ACTION="echo"
COMPENSATE=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --mount) MOUNTS+=("$(printf '%02d' "$((10#$2))")"); shift 2;;
        --all) MOUNTS=(00 15 25 35); shift;;
        --n) N="$2"; shift 2;;
        --compensate) COMPENSATE=1; shift;;
        --dry-run) ACTION="dry-run"; shift;;
        --go) ACTION="go"; shift;;
        --cleanup) ACTION="cleanup"; shift;;
        *) echo "[uptilt-ab] FAIL: unknown arg '$1'" >&2; exit 2;;
    esac
done

cleanup_shadow() {
    if [[ -L "$SHADOW" ]]; then
        rm -f "$SHADOW"
        echo "[uptilt-ab] removed shadow symlink $SHADOW"
    fi
}

# The OUT CSV name for a given zero-padded mount, honoring --compensate (the
# 'c' suffix marks a NON-ZERO compensated re-fly arm; ADR-0067 evidence names
# are reused only in the uncompensated sweep). Single source of truth so the
# preflight collision check and the fly loop agree.
out_for_mount() {
    local m="$1"
    if [[ "$COMPENSATE" -eq 1 ]]; then
        local suffix=""
        [[ "$m" != "00" ]] && suffix="c"
        echo "logs/mc_uptilt_refly_weave12_up${m}${suffix}.csv"
    else
        echo "logs/mc_uptilt_weave12_up$m.csv"
    fi
}

if [[ "$ACTION" == "cleanup" ]]; then
    if [[ -e "$SHADOW" && ! -L "$SHADOW" ]]; then
        echo "[uptilt-ab] FAIL: $SHADOW exists but is NOT a symlink -- refusing to touch it." >&2
        exit 1
    fi
    cleanup_shadow
    [[ -e "$SHADOW" ]] || echo "[uptilt-ab] models/mono_cam clear -- future batches unshadowed."
    exit 0
fi

if [[ "${#MOUNTS[@]}" -eq 0 ]]; then
    echo "usage: $0 --all|--mount {0|15|25|35} [--n 8] [--dry-run|--go|--cleanup]" >&2
    exit 2
fi

for m in "${MOUNTS[@]}"; do
    if [[ ! -f "$VARIANT_ROOT/up$m/mono_cam/model.sdf" ]]; then
        echo "[uptilt-ab] FAIL: variant up$m missing -- run scripts/uptilt_make_variants.py" >&2
        exit 1
    fi
done
if [[ -e "$SHADOW" && ! -L "$SHADOW" ]]; then
    echo "[uptilt-ab] FAIL: $SHADOW exists and is NOT a symlink (unexpected real dir/file)." >&2
    echo "            Refusing: investigate before any camera arm flies." >&2
    exit 1
fi

# PREFLIGHT the OUT-CSV collisions for ALL arms BEFORE flying the first one
# (review finding, #40): the old per-arm mid-loop check let a stale later-arm
# CSV abort the sweep only AFTER earlier arms had flown ~10 min each, and the
# corrected re-run then tripped on the fresh earlier CSV -- stranding a
# half-flown, no-clean-completion paired sweep (the within-sweep-paired-control
# rule this re-fly exists to enforce). Fail fast, atomically, up front.
if [[ "$ACTION" == "go" ]]; then
    collide=0
    for m in "${MOUNTS[@]}"; do
        o="$(out_for_mount "$m")"
        if [[ -e "$o" ]]; then
            echo "[uptilt-ab] FAIL: $o already exists -- evidence protection." >&2
            collide=1
        fi
    done
    if [[ "$collide" -ne 0 ]]; then
        echo "            Move ALL colliding CSVs aside deliberately (as one set," >&2
        echo "            to keep the sweep a single within-sweep paired batch)" >&2
        echo "            before re-flying. Nothing flew." >&2
        exit 1
    fi
fi

trap cleanup_shadow EXIT

first=1
OUTS=()
for m in "${MOUNTS[@]}"; do
    OUT="$(out_for_mount "$m")"
    if [[ "$COMPENSATE" -eq 1 ]]; then
        M4X=(--m4-extra "--cam-mount-up-deg $((10#$m))")
    else
        M4X=()
    fi
    case "$ACTION" in
    echo)
        echo "=== up$m: shadow $SHADOW -> $VARIANT_ROOT/up$m/mono_cam"
        [[ "$COMPENSATE" -eq 1 ]] && echo "    COMPENSATED: --cam-mount-up-deg $((10#$m)) -> $OUT"
        "$LAUNCH" --path weave --n "$N" --master-seed 42 --out "$OUT" "${M4X[@]}"
        ;;
    dry-run)
        "$LAUNCH" --path weave --n "$N" --master-seed 42 --out "$OUT" "${M4X[@]}" --dry-run
        ;;
    go)
        if [[ "$first" -ne 1 ]]; then
            echo "[uptilt-ab] cooldown 120 s before next arm (mc-batch skill)..."
            sleep 120
        fi
        cleanup_shadow
        ln -s "$VARIANT_ROOT/up$m/mono_cam" "$SHADOW"
        echo "[uptilt-ab] shadow ACTIVE: $SHADOW -> $(readlink "$SHADOW")"
        grep -o '<pose>[^<]*</pose><!-- UP-TILT[^>]*>' "$SHADOW/model.sdf" || true
        UPTILT_EXPECTED=1 "$LAUNCH" --path weave --n "$N" --master-seed 42 \
            --out "$OUT" "${M4X[@]}" --go
        cleanup_shadow
        ;;
    esac
    first=0
    OUTS+=("$OUT")
done

if [[ "$ACTION" == "go" ]]; then
    echo "[uptilt-ab] sweep done; shadow removed. Analyze:"
    echo "  .venv/bin/python scripts/experiments/uptilt_ab_analyze.py \\"
    printf '      %s' "${OUTS[0]}"
    for o in "${OUTS[@]:1}"; do printf ' %s' "$o"; done
    if [[ "$COMPENSATE" -eq 1 ]]; then
        # #40 re-fly: the ADR-0067 arms ride along (the uncompensated
        # comparison + the fresh-up00 machinery-stability tripwire). The
        # analyzer picks the WITHIN-SWEEP `refly` up00 as control regardless of
        # argv order and WARNS when >1 mount-0 arm is present (so the legacy
        # up00 cannot silently re-base the PRIMARY parity read -- cross-era
        # pairing is INVALID, ADR-0067 addendum). Bars pre-registered in
        # docs/adr0067_refly_preregistration.md.
        printf ' \\\n      logs/mc_uptilt_weave12_up00.csv logs/mc_uptilt_weave12_up15.csv'
        printf '\n  # control = the refly up00 (within-sweep); the two legacy '
        printf 'up00/up15 are\n  # the uncompensated comparators + machinery '
        printf 'tripwire (see the [WARN] line).\n'
    else
        printf ' \\\n      --baseline logs/mc_t21_trackgate_weave12_r2.csv\n'
    fi
fi
