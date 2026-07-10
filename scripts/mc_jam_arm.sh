#!/usr/bin/env bash
# ============================================================================
# ADR-0059 comms-denied MC arm harness -- mid-dash cue jam vs the deployment
# config (--track --handoff-cue-gate 8). Validates the ADR-0059 staleness fix.
#
#   *** DO NOT RUN THIS YET ***
#   The machine is running a ~6 h CPU v3 fine-tune. mc_batch is a Gazebo batch
#   and MUST run at IDLE machine load (idle-load rule, ADR-0015 2nd addendum) --
#   RTF sag under load silently distorts results. Run these arms ONE AT A TIME,
#   AFTER the v3 train finishes, via the mc-batch skill (which wraps launch ->
#   wait -> parse -> GPU-health -> cooldown and knows the self-kill footgun).
#   NEVER run two arms at once (they kill each other's sims -- ADR-0015).
#
# WHAT THIS TESTS. Every ADR-0058 arm ran the cue mock full-duration (no jam),
# so the adopted deployment config was NEVER exercised against a mid-dash cue
# jam. ADR-0059 found (code-traced) that it FAILS CLOSED: when the cue link is
# jammed, last_cue_pos FREEZES and both the handoff cue-gate and the detect-then-
# track seeker seed reject the REAL target as it drifts past the gate -> never
# hands off -> a regression vs the ADR-0015 WORST-tier link-cutoff. The fix ages
# the frozen cue out (m4.cue_is_stale + --cue-stale-horizon, default COAST_STALE_S)
# and falls back to the camera-only paths (camera-track-continuity anti-phantom).
#
# THE METRIC (design review G3a / Action 2 + Fable review, corrected). PRIMARY =
# HANDOFF-COUNT + post-handoff attribution via scripts/analyze_track_ab.py:
# intercepts completed under denial CONDITIONED ON A REAL HANDOFF (its Pk (iii)),
# plus the handoff-OUTCOME breakdown (never = the fail-closed count). NOT pooled
# count(miss_m<=2.5): a range-based cutoff can credit a NO-HANDOFF dead-reckon
# flyby as a bogus "intercept", so pooled miss OVER-counts. The mc_batch handoff
# column + breakoff_reason are the quick read; analyze_track_ab.py is the verdict.
#
# THE ARMS (all paired: identical geometry + master-seed 42 = same 16 flights;
# the ONLY differences are the cue cutoff and whether the ADR-0059 fix is live):
#   control_strict : NO cutoff, fix INERT (--cue-stale-horizon 1e9).
#                    No-regression proof: PAIRED-PER-SEED statistical equivalence
#                    to the ADR-0058 r2 headline -- NOT an exact 16/16 count (r2's
#                    max miss 2.48 m sits right under the 2.5 m line and single-
#                    flight terminal noise is ~1 m, so 15/16 by noise alone is
#                    plausible). Check per-seed |miss - r2_miss| is sub-noise with
#                    no systematic shift, and the handoff-outcome breakdown matches.
#   control_active : NO cutoff, fix LIVE (default horizon).
#                    Realistic cue includes --dropout-markov (mean burst 1.5s >
#                    the 1.0s horizon), so the fallback MAY engage on a long
#                    dropout burst -- safely (camera-only dead-reckon, then the
#                    cue recovers). Expect 16/16 -> proves the always-on fix does
#                    not regress the no-jam case even under realistic dropout.
#   jam_fixoff     : cutoff + fix OFF (--cue-stale-horizon 1e9). The FAIL-CLOSED
#                    WITNESS -- reproduces the ADR-0059 bug in-sim (expect the
#                    handoff to collapse: few/zero real handoffs, misses balloon).
#   jam_fixon      : cutoff + fix LIVE (default horizon). The FIX VALIDATION --
#                    expect intercepts COMPLETED camera-only after the jam (the
#                    seeker seed re-acquires via camera-track continuity, the
#                    handoff falls back to the camera-only in-range streak).
#
# The money comparisons: jam_fixoff (bug) vs jam_fixon (fixed) = the fix works
# under denial; control_strict/control_active vs r2 = no regression no-jam.
#
# CUTOFF RANGE. --link-cutoff-range-m R kills the cue permanently once the true
# range < R (s2_cue_mock, ADR-0015 table #6). Default R=15 m is solidly PRE-
# acquisition (HANDOFF_RANGE_M=10 m; ADR-0026 sized the jam envelope ~11.5 m),
# so the cue always dies before the camera can hand off. The jam BITE POINT is
# randomized across the n=16 flights by each seed's weave phase + start geometry
# (the true range crosses R at a different dash-progress every flight). To MAP
# the exposed window, sweep the threshold via the env override, e.g.:
#   CUTOFF_RANGE_M=12 scripts/mc_jam_arm.sh jam_fixon   # -> logs/..._jam_fixon_r12.csv
#   CUTOFF_RANGE_M=18 scripts/mc_jam_arm.sh jam_fixon
# (Per-flight RANDOM ranges with paired seeds aren't done here: mc_batch's seed
# family is generated per-invocation, so an n=1 loop would re-fly flight 0 each
# time -- the design review's own remedy was a NEW --cue-kill-range knob, future
# work. A fixed R with geometry-randomized bite timing + a threshold sweep is the
# clean, paired, buildable-today equivalent.)
# ============================================================================
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARM="${1:-}"

# --- EXACT ADR-0058 r2 deployment config, reproduced from
#     logs/mc_t21_trackgate_weave12_r2_stdout.log (the pre-registered headline). ---
export MC_SEEKER="markerless"
export MC_WORLD="markerless"
export MC_TARGET_MODEL="fpv_target_markerless"
export MC_VENV_PYTHON="$REPO_ROOT/.venv-seeker/bin/python"

# The realistic degraded cue (ADR-0015/0017): range-dependent sigma, datum bias,
# latency jitter, bursty Markov dropout, emitted velocity. IDENTICAL to r2.
REALISTIC_CUE="--sigma-range --datum-bias-m 0.5 --latency-jitter-s 0.05 --dropout-markov --emit-velocity --vel-sigma 0.5"

# The adopted deployment --extra-args (IDENTICAL to r2). --cue-stale-horizon is
# appended per-arm below (default = COAST_STALE_S = 1.0 s when omitted).
DEPLOY_EXTRA="--dash-speed 16 --early-handoff --cue-velocity --dash-unclamp --fuse-midcourse --track --handoff-cue-gate 8"

# #40 Stage-2 recovery re-test hook (ADR-0068 / docs/adr0067_refly_preregistration
# .md): MOUNT_DEG appends --cam-mount-up-deg (composes a fitted up-tilt mount into
# FIX-A's LOS derotation) and OUT_SUFFIX distinguishes the CSV so the #39 coast
# NULL evidence is never overwritten. BOTH default EMPTY => byte-identical to the
# validated ADR-0059 arms (the caller must ALSO set the up15 shadow symlink +
# UPTILT_EXPECTED=1; scripts/stage2_tilt_recovery_arm.sh owns that lifecycle).
MOUNT_DEG="${MOUNT_DEG:-}"
OUT_SUFFIX="${OUT_SUFFIX:-}"
if [[ -n "$MOUNT_DEG" ]]; then
    DEPLOY_EXTRA="$DEPLOY_EXTRA --cam-mount-up-deg $MOUNT_DEG"
fi

CUTOFF_RANGE_M="${CUTOFF_RANGE_M:-15}"     # pre-acquisition cue kill range (m)
BIG_HORIZON="1000000000"                   # --cue-stale-horizon that disables the fix (A/B)

# Geometry + seeds IDENTICAL to the r2 headline arm -> the four arms are paired.
BATCH_GEO=(--mode s2 --laws pronav --speeds 12 --n 16 --directions both
           --path weave --geometry standard --weave-period-s 4.0 --weave-lat-speed 3.0
           --master-seed 42 --x0 6.5 --y0-mag 29.3 --jitter 1.5)

run_arm() {   # <out_name> <cue_extra> <extra_args>
    local out_name="$1" cue_extra="$2" extra_args="$3"
    export S2_CUE_MOCK_EXTRA="${REALISTIC_CUE}${cue_extra:+ ${cue_extra}}"
    local out_csv="$REPO_ROOT/logs/mc_adr0059_${out_name}.csv"
    echo "[adr0059] ================= ARM: ${out_name} ================="
    echo "[adr0059] MC_SEEKER=$MC_SEEKER MC_WORLD=$MC_WORLD MC_VENV_PYTHON=$MC_VENV_PYTHON"
    echo "[adr0059] S2_CUE_MOCK_EXTRA=$S2_CUE_MOCK_EXTRA"
    echo "[adr0059] --extra-args: $extra_args"
    echo "[adr0059] --out: $out_csv"
    echo "[adr0059] (reminder: IDLE load, ONE arm at a time; run via the mc-batch skill)"
    exec "$REPO_ROOT/scripts/mc_batch.sh" "${BATCH_GEO[@]}" \
        --extra-args "$extra_args" --out "$out_csv"
}

case "$ARM" in
    control_strict)
        run_arm "control_strict" "" \
            "$DEPLOY_EXTRA --cue-stale-horizon $BIG_HORIZON" ;;
    control_active)
        run_arm "control_active" "" \
            "$DEPLOY_EXTRA" ;;
    jam_fixoff)
        run_arm "jam_fixoff_r${CUTOFF_RANGE_M}" "--link-cutoff-range-m $CUTOFF_RANGE_M" \
            "$DEPLOY_EXTRA --cue-stale-horizon $BIG_HORIZON" ;;
    jam_fixon)
        run_arm "jam_fixon_r${CUTOFF_RANGE_M}" "--link-cutoff-range-m $CUTOFF_RANGE_M" \
            "$DEPLOY_EXTRA" ;;
    jam_fixon_coast)
        # ADR-0059 RECOVERY arm (task #39). Adds ADR-0015 --coast-search on top
        # of the fix: on cue staleness the interceptor DEAD-RECKONS the dash to
        # the predicted acquisition basket (COAST_ACQ_RANGE_M=10 m) then runs a
        # bounded +/-20 deg yaw sweep to REACQUIRE the target with the onboard
        # camera. The staleness fix (unguarded at both read-sites) keeps the
        # handoff/seed decisions camera-only = anti-phantom under jam; coast-
        # search supplies the reach-the-basket + search that fixon-alone lacked
        # (fixon aborted 13-15/16 as 'never reached handoff'). Compare to
        # jam_fixon (no coast) at the SAME cutoff: does REAL-ish recovery jump?
        # KNOWN RISK: the sweep is lateral (yaw) only; if the dash-pitch throws
        # the target above the FoV (ADR-0060), yaw-only may not reacquire -- the
        # MC will show it (watch the 'never' outcome + first_det coverage).
        run_arm "jam_fixon_coast_r${CUTOFF_RANGE_M}${OUT_SUFFIX}" "--link-cutoff-range-m $CUTOFF_RANGE_M" \
            "$DEPLOY_EXTRA --coast-search" ;;
    dry)
        # Show the planned matrix for the jam_fixon arm without booting a sim.
        export S2_CUE_MOCK_EXTRA="${REALISTIC_CUE} --link-cutoff-range-m ${CUTOFF_RANGE_M}"
        exec "$REPO_ROOT/scripts/mc_batch.sh" "${BATCH_GEO[@]}" \
            --extra-args "$DEPLOY_EXTRA" --dry-run ;;
    *)
        cat >&2 <<EOF
usage: $0 {control_strict|control_active|jam_fixoff|jam_fixon|dry}

Run ONE arm at a time, at IDLE, AFTER the v3 fine-tune. Suggested order:
  1) $0 control_strict     # no-regression: PAIRED-per-seed equivalence vs r2 (NOT a 16/16 count)
  2) $0 jam_fixoff         # the fail-closed WITNESS (reproduces the ADR-0059 bug)
  3) $0 jam_fixon          # the FIX (intercepts completed under denial, REAL-handoff-conditioned)
  4) $0 control_active     # fix live, no jam -> handoff/Pk equivalence to r2 (safe under dropout)

PRIMARY analysis -- handoff-count + REAL-handoff-conditioned post-handoff Pk
(scripts/analyze_track_ab.py; the fail-closed count is the 'never' outcome):
  .venv-seeker/bin/python scripts/analyze_track_ab.py \\
      r2=logs/mc_t21_trackgate_weave12_r2.csv \\
      control_strict=logs/mc_adr0059_control_strict.csv \\
      jam_fixoff=logs/mc_adr0059_jam_fixoff_r15.csv \\
      jam_fixon=logs/mc_adr0059_jam_fixon_r15.csv
  # Read: handoff-OUTCOME breakdown (never = FAIL-CLOSED count) + Pk (iii)
  # REAL-handoff-conditioned = intercepts completed under denial. Expect
  # jam_fixoff handoffs to COLLAPSE (never >> 0) and jam_fixon to RECOVER them;
  # do NOT read pooled count(miss<=2.5) as the headline (a no-handoff flyby can
  # bank a bogus pooled hit under a range cutoff).

No-regression check -- control_strict vs r2, PAIRED per seed (NOT an exact count):
  .venv/bin/python - <<'PY'
  import csv, statistics as s
  def load(p): return {r["cue_seed"]: r for r in csv.DictReader(open(p))}
  a=load("logs/mc_t21_trackgate_weave12_r2.csv")
  b=load("logs/mc_adr0059_control_strict.csv")
  keys=[k for k in a if k in b
        and a[k]["miss_m"] not in ("","nan") and b[k]["miss_m"] not in ("","nan")]
  dd=[float(b[k]["miss_m"])-float(a[k]["miss_m"]) for k in keys]      # signed
  ad=[abs(x) for x in dd]
  print(f"paired Δmiss vs r2: n={len(dd)} median|Δ|={s.median(ad):.3f}m "
        f"max|Δ|={max(ad):.3f}m mean_signed={s.mean(dd):+.3f}m")
  print("=> sub-~1m |Δ| + mean_signed ~0 (no systematic shift) => NO regression")
  PY
EOF
        exit 2 ;;
esac
