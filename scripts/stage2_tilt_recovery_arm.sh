#!/usr/bin/env bash
# =============================================================================
# stage2_tilt_recovery_arm.sh -- #40 Stage-2: the TILTED+COMPENSATED comms-jam
# RECOVERY re-test (docs/adr0067_refly_preregistration.md, ADR-0068 addendum).
#
# THE QUESTION (GOALS.md headline). ADR-0059's comms-denied recovery was a NULL:
# under a pre-acquisition cue jam the coast-search reached the basket but the
# CAMERA never reacquired on 10/16 coast flights (15-21 m) -- perception
# AVAILABILITY, not guidance, was the binding limit, because the down-pitched
# dash throws the horizon-level target ABOVE the fixed forward FoV (ADR-0060).
# The #40 re-fly CONFIRMED a fixed +15 up-tilt closes that FoV gap
# (dash-above-FoV 35% -> 0%, first-det +4 m) AND that composing the mount into
# FIX-A recovers the nominal terminal cost (Pk@2.5 4/8 -> 8/8). This arm asks
# the payoff question: does that availability win convert the recovery NULL into
# actual camera-only recovery?
#
# WHAT IT FLIES. The ADR-0059 jam_fixon_coast arm (r18 pre-acquisition cutoff,
# n=16, master-seed 42, adopted config + --coast-search) with the up15 camera
# shadow ACTIVE + --cam-mount-up-deg 15 composed into guidance -- i.e. the exact
# #39 recovery arm, now TILTED+COMPENSATED. Distinct CSV (_tilt15) so the #39
# coast NULL evidence is never touched.
#
# SCORING (pre-registered, no new goalposts beyond the two disclosed deviations
# in docs/adr0067_refly_preregistration.md Stage 2): scripts/check_jam_mc.py with
# the coast CSV as --fixon at the r18 cutoff. RECOVERY DEMONSTRATED iff
# RH >= 11/16 AND J >= 8/16 AND never drops; MECHANISM (the availability claim):
# camera-never-detected <= 4/16 vs the #39 baseline 10/16.
#
# SHADOW LIFECYCLE. THIS script owns models/mono_cam: create up15 -> fly ->
# ALWAYS remove (trap on EXIT). mc_jam_arm.sh has no shadow guard of its own, so
# a leftover link would silently tilt every future batch -- the trap + the
# preflight real-dir refusal are the protection. If a run is killed, clear a
# stranded link with: scripts/uptilt_ab_arm.sh --cleanup
#
# USAGE (default ECHO-ONLY; nothing runs, no symlink touched):
#   scripts/stage2_tilt_recovery_arm.sh            # print the plan
#   scripts/stage2_tilt_recovery_arm.sh --go       # fly (idle load only)
# =============================================================================
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"

MOUNT=15
CUTOFF=18
SUFFIX=_tilt15
LOAD_MAX="${LOAD_MAX:-3.0}"
ACTION="${1:-echo}"

VARIANT="$SCRIPT_DIR/experiments/uptilt_mounts/up$(printf '%02d' "$MOUNT")/mono_cam"
SHADOW="$REPO_ROOT/models/mono_cam"
OUT="logs/mc_adr0059_jam_fixon_coast_r${CUTOFF}${SUFFIX}.csv"
STDOUT_LOG="${OUT%.csv}_stdout.log"

cleanup_shadow() { [[ -L "$SHADOW" ]] && { rm -f "$SHADOW"; echo "[stage2] removed shadow $SHADOW"; }; return 0; }

# ---- preflight (all modes) -------------------------------------------------
fail=0
[[ -f "$VARIANT/model.sdf" ]] || { echo "[stage2] MISSING variant: $VARIANT/model.sdf -- run scripts/uptilt_make_variants.py" >&2; fail=1; }
for f in scripts/mc_jam_arm.sh scripts/check_jam_mc.py .venv-seeker/bin/python \
         worlds/markerless.sdf models/fpv_target_markerless \
         scripts/seeker/weights/drone_finetuned_v2.onnx; do
    [[ -e "$f" ]] || { echo "[stage2] MISSING: $f" >&2; fail=1; }
done
if [[ -e "$SHADOW" && ! -L "$SHADOW" ]]; then
    echo "[stage2] FAIL: $SHADOW exists and is NOT a symlink -- refusing." >&2; fail=1
fi
[[ "$fail" -eq 0 ]] || exit 1

print_plan() {
    echo "[stage2] #40 tilted+compensated comms-jam RECOVERY re-test"
    echo "    arm      : jam_fixon_coast (ADR-0059 #39), TILTED+COMPENSATED"
    echo "    shadow   : $SHADOW -> $VARIANT (up$MOUNT)"
    echo "    m4 extra : ... --coast-search --cam-mount-up-deg $MOUNT"
    echo "    cutoff   : r$CUTOFF (pre-acquisition), n=16, master-seed 42"
    echo "    out      : $OUT   (distinct from the #39 coast NULL)"
    echo "    stdout   : $STDOUT_LOG"
    echo "    score (coast arm as --fixon, alongside the flown ADR-0059 arms):"
    echo "       .venv-seeker/bin/python scripts/check_jam_mc.py \\"
    echo "         --strict logs/mc_adr0059_control_strict.csv \\"
    echo "         --active logs/mc_adr0059_control_active.csv \\"
    echo "         --fixoff logs/mc_adr0059_jam_fixoff_r${CUTOFF}.csv \\"
    echo "         --fixon $OUT --cutoff-range-m $CUTOFF"
    echo "    bars     : RECOVERY iff RH>=11/16 AND J>=8/16 AND never drops;"
    echo "               MECHANISM camera-never-detected <=4/16 (baseline 10/16);"
    echo "               dRH vs the #39 coast NULL (mc_adr0059_jam_fixon_coast_r${CUTOFF}.csv, RH 1/16)"
}

if [[ "$ACTION" != "--go" ]]; then
    print_plan
    echo "[stage2] ECHO-ONLY (default). Pass --go to fly (idle load only)."
    exit 0
fi

# ---- --go: guarded launch --------------------------------------------------
if [[ -e "$OUT" ]]; then
    echo "[stage2] FAIL: $OUT already exists -- evidence protection. Move it aside deliberately." >&2
    exit 1
fi
# One sim at a time (pattern lives in THIS file -- immune to the inline self-match footgun).
if pgrep -f "bin/px4" >/dev/null 2>&1; then
    echo "[stage2] FAIL: a px4 process is already running -- ONE sim at a time." >&2
    exit 1
fi
load1="$(cut -d' ' -f1 /proc/loadavg)"
if awk -v l="$load1" -v m="$LOAD_MAX" 'BEGIN{exit !(l>m)}'; then
    echo "[stage2] FAIL: 1-min load $load1 > $LOAD_MAX -- batches run at IDLE only (ADR-0015)." >&2
    exit 1
fi
DXGK_BASE="$( (dmesg 2>/dev/null | grep -icE dxgk) || echo n/a)"
echo "[stage2] dxgk baseline: $DXGK_BASE   load1: $load1"
print_plan

trap cleanup_shadow EXIT
cleanup_shadow
ln -s "$VARIANT" "$SHADOW"
echo "[stage2] shadow ACTIVE: $SHADOW -> $(readlink "$SHADOW")"
grep -o '<pose>[^<]*</pose><!-- UP-TILT[^>]*>' "$SHADOW/model.sdf" || true
echo "[stage2] LAUNCHING (foreground; run this script backgrounded)..."
set +e
MOUNT_DEG="$MOUNT" OUT_SUFFIX="$SUFFIX" CUTOFF_RANGE_M="$CUTOFF" UPTILT_EXPECTED=1 \
    scripts/mc_jam_arm.sh jam_fixon_coast > "$STDOUT_LOG" 2>&1
rc=$?
set -e
cleanup_shadow
DXGK_POST="$( (dmesg 2>/dev/null | grep -icE dxgk) || echo n/a)"
echo "[stage2] mc_jam_arm exit=$rc   dxgk post: $DXGK_POST (baseline $DXGK_BASE)"
if grep -q "Batch complete" "$STDOUT_LOG"; then
    echo "[stage2] SENTINEL OK: Batch complete."
else
    echo "[stage2] WARNING: no 'Batch complete' sentinel in $STDOUT_LOG -- inspect it." >&2
fi
echo "[stage2] score now:"
echo "    .venv-seeker/bin/python scripts/check_jam_mc.py --fixon $OUT --cutoff $CUTOFF"
exit "$rc"
