# ADR-0059 comms-denied RECOVERY — pre-registration (task #39)

**Written 2026-07-10, BEFORE the `jam_fixon_coast` arms fly.** Purpose: commit the
success criteria in advance so the recovery verdict cannot be reverse-fit to the
result. This is the same discipline that caught the v3 seeker false-win
(ADR-0061): pre-register, then score. No new goalposts — the thresholds below are
lifted verbatim from the already-registered `scripts/check_jam_mc.py` (G3b/G3c),
which was itself written before any jam arm flew (task #31).

## What is being tested

ADR-0059 established, in-sim, that the adopted deployment config
(`--track --handoff-cue-gate 8`) under a pre-acquisition cue jam is **fail-safe
but not recovery**: the staleness fix ages out the frozen cue and aborts cleanly
(phantom-handoffs eliminated 9→1 @18 m, 5→0 @22 m) BUT the flight rarely reaches
a real handoff — 13–15/16 abort as `never` because, with no `--coast-search`, the
dead-reckoned dash drifts and the camera never builds its detection streak. So
"works comms-denied" is HELD as an intercept claim.

The recovery hypothesis (#39): adding **`--coast-search`** (ADR-0015, already in
`m4_intercept.py`) supplies the missing piece — on cue staleness it dead-reckons
the dash to the predicted acquisition basket (`COAST_ACQ_RANGE_M`=10 m) then runs
a bounded ±20° yaw sweep to reacquire the target camera-only. It composes with the
staleness fix (both read-sites stay camera-only = anti-phantom under jam). Does
that convert fail-safe into actual camera-only recovery?

## Arms (all paired, master-seed 42 = the same 16 flights; ADR-0058 r2 config)

| Arm | Cutoff | Fix | Coast | Source |
|---|---|---|---|---|
| `jam_fixoff_r18` / `_r22` | 18 / 22 m | OFF | no | ADR-0059 (already flown) — fail-closed witness |
| `jam_fixon_r18` / `_r22` | 18 / 22 m | ON | no | ADR-0059 (already flown) — fail-SAFE, not recovery |
| **`jam_fixon_coast_r18` / `_r22`** | 18 / 22 m | ON | **yes** | **NEW (#39)** — the recovery arm |

Cutoffs 18 and 22 m are the **pre-acquisition** jams (handoff range 10 m) where
fail-closed was worst — the honest hard case. If recovery holds there, it holds.

## Pre-registered verdict (scored by `check_jam_mc.py` with `--fixon` = the coast CSV, at the matched cutoff)

Let `RH` = REAL(-ish) handoffs / 16 (REAL or RANGE_BIASED_REAL, per the existing
classifier); `J` = joint success / 16 (REAL handoff AND post-handoff min
`gt_range` ≤ 2.5 m, denominator ALL 16). Compare `jam_fixon_coast_rX` against
`jam_fixon_rX` (no-coast) at the same cutoff X.

- **RECOVERY DEMONSTRATED** iff, at ≥1 pre-acquisition cutoff:
  `RH(coast) ≥ 11/16` (G3b) **AND** `J(coast) ≥ 8/16` (G3c) **AND**
  `RH(coast) > RH(fixon-no-coast)` materially (≥ +4/16, i.e. clearly beyond the
  ~1-flight single-seed noise) **AND** the `never` outcome count drops
  correspondingly (coast reaches handoff, not a range-flyby banking a bogus hit).
- **PARTIAL RECOVERY** iff `RH(coast)` beats `RH(fixon-no-coast)` by ≥ +4/16 but
  falls short of 11/16 — coast-search helps but does not fully restore the
  intercept (most likely the ADR-0060 yaw-vs-pitch limit: the ±20° sweep is
  lateral only, so a target thrown above the FoV during the dash is not
  reacquired). Report the stratum: of the `never` flights, how many were
  first-detected-then-lost vs never-detected.
- **NULL** iff `RH(coast) ≈ RH(fixon-no-coast)` (Δ < +4/16) — coast-search does
  not un-hold comms-denied under this jam; the `never` collapse is FOV/steering,
  not cue-gate, exactly the ambiguity `check_jam_mc.py` G9 flagged for the
  no-coast arms, now resolved as a coast-search limitation.

## Honesty scoping (applies to EVERY branch, including DEMONSTRATED)

1. Any recovery claim is **scoped to speed-12 / gate-8 / weave / cutoffs 18–22 m**
   — the checker's own header forbids generalizing beyond the flown config.
2. Even a DEMONSTRATED result un-holds "works comms-denied" only as a **scoped
   recovery** claim ("under a pre-acquisition cue jam at 18–22 m, fix + coast-search
   restores a camera-only intercept in RH/16 flights"), never a blanket one.
3. Watch-metrics that must be reported regardless of the headline: the `never`
   count, `first_det_range_m` coverage (did the camera ever see the target during
   the sweep), `n_reacq_rejected` (did the anti-phantom gate reject a real
   re-acquire — a coasting-safe false-negative), and the PHANTOM decomposition
   (range-noise phantom on a good intercept vs a fooled-ghost phantom).
4. Honesty boundary intact: coast-search + the staleness fix are camera-only +
   dead-reckoned own-state; `gt_*` is scoring/logging only; the per-tick no-cheat
   audit is re-run on the coast arm before any claim.

## Method (fires when the sim frees, after the jink n=16 batch; idle load, one arm at a time)

```
CUTOFF_RANGE_M=18 scripts/mc_jam_arm.sh jam_fixon_coast   # -> logs/mc_adr0059_jam_fixon_coast_r18.csv
CUTOFF_RANGE_M=22 scripts/mc_jam_arm.sh jam_fixon_coast   # -> ..._r22.csv
# score (reuse the pre-registered machinery; --fixon points at the coast arm):
.venv-seeker/bin/python scripts/check_jam_mc.py \
    --strict logs/mc_adr0059_control_strict.csv \
    --fixoff logs/mc_adr0059_jam_fixoff_r18.csv \
    --fixon  logs/mc_adr0059_jam_fixon_coast_r18.csv \
    --active logs/mc_adr0059_control_active.csv
# plus a direct paired table jam_fixon_coast_r18 vs jam_fixon_r18 (R8-style, same seeds).
```

If DEMONSTRATED → ADR-0059 addendum (recovery), un-HOLD comms-denied (scoped),
commit. If PARTIAL/NULL → the coast-search limit is itself a defensible finding
(likely motivates the up-tilted mount, ADR-0060 / #35, so the sweep has a target
to find); record it honestly and keep comms-denied HELD as an intercept claim.
