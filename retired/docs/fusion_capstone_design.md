# Fusion capstone — execution design (ADR-0034 → build plan)

*Status: COUNCIL-REVISED v2, 2026-07-08. Draft v1 was reviewed by a 3-member
council (estimation theory / honesty boundary / experiment design); all three
returned substantive amendments, two backed by numeric experiments run against
`ekf_tracker.py` itself and the project's logs. This revision supersedes v1
wholesale. Logged as ADR-0041 (design ratified; build pending).*

## 0. One-line claim under test

When the terminal seeker is noisy (markerless) and the tracker weights its
sources by live covariance (EKF), does fusing the ground cue mid-course buy
**handoff-reach robustness** — the thing ADR-0018's null could not show
because it ran under a clean tag, a fixed-gain alpha-beta, and σ_R constants
~180× too steep (since corrected, ADR-0017/0036)?

## 1. What exists vs. what must be built (verified against code 2026-07-08)

| Piece | State | Anchor |
|---|---|---|
| EKF tracker (rel-Cartesian CV, Joseph form, chi-square gate) | REAL, 10/10 unit tests | `scripts/ekf_tracker.py:289` |
| EKF flight A/B | DONE — miss/RMSE null, but clean-rate 8/8→2/8 regression from untuned `Q_ACCEL_PSD=64` | ADR-0037 |
| `EKFTracker.correct_cue` | Built but UNWIRED (never called) | `ekf_tracker.py:456` |
| `FusedTrack` (bearing-weighted POLAR fusion) | Wired via `--fuse-midcourse`; always alpha-beta internally | `m4_intercept.py:815,1531` |
| Handoff one-way latch | Wired + statically audited | `m4_intercept.py:1984` |
| Per-tick batch honesty auditor | BUILT 2026-07-08 (`scripts/audit_per_tick.py`, deployment-profile calibrated) | this session |
| "No cue-tainted EKF covariance survives handoff" enforcement + audit | **Does not exist** — and the natural config (`--fuse-midcourse` without `--warm-handoff`) leaves P completely unbounded at latch | council finding |

## 2. Council findings that reshaped this design (evidence-backed)

**F1 — Confident-bias-lock (estimation council, T1-code).** Replaying
`EKFTracker` at the project's own constants: after ~178 cue corrections under
a WORST-tier 2.5 m datum bias, a *sensibly tuned* Q makes the chi-square gate
reject **81–100% of valid post-latch camera corrections** (0% at the broken
Q=64 — i.e., the D1 Q-tune itself arms this failure). The filter's own (wrong)
IID-noise model of a correlated bias legitimately tightens P around a wrong
value; the gate then shields the error. **None of the honesty-audit layers see
this — it is a correctness failure, not a leak.**

**F2 — Bearing poisoning through Cartesian cue updates (estimation council,
T1-code).** `correct_cue` corrects full Cartesian `[dn,de]`, so a cross-range
datum bias leaks into `lambda_hat` — measured **+6.3° at R≈20 m growing to
+9.3° at R≈10 m** (≈1.5 m cross-range aim error, the project's whole miss
scale) despite concurrent unbiased 14 Hz camera updates. This is exactly the
door `FusedTrack`'s polar split was built to close (ADR-0018: "the cue NEVER
touches the angle").

**F3 — Unbounded covariance survival (honesty council, code-traced).** The
warm-handoff seeding path (`seed_from_polar`) *incidentally* resets P to fixed
disclosed constants — but it is gated on `--warm-handoff` only, never on
tracker choice; with `--fuse-midcourse` alone, cue-shaped P survives the latch
unbounded, and the proposed `ekf_cue_updates_post_handoff` counter reads 0
regardless (the seeding path never calls `correct_cue`). The project's own
prior standard (`docs/ekf_design_brief.md`: "re-initialize, or provably decay")
is the testable bar; v1's assertion was not.

**F4 — The track-RMSE metric never measured the EKF (estimation council,
code-traced).** `tgt_n_hat/tgt_e_hat` are always written from the alpha-beta
`TargetTracker` (`m4_intercept.py:2280-2284`) — identical in both ADR-0037
arms *by construction*. The position half of ADR-0037's "track RMSE null" was
structurally guaranteed. The capstone's primary metric NEEDS a new logged
field (the EKF's own relative state, scored in the relative frame).

**F5 — Power (experiment council).** At n=8, paired McNemar needs 6/8
same-direction discordant pairs for p<0.05 — ADR-0037's huge effect barely
cleared it. Binomial metrics (clean-rate, handoff-reach) cannot carry a
verdict at n=8. Measured flight cost is **~65–75 s/flight** (5 recent batches,
sim-boot to sim-boot), so n=16 on the cells that matter costs minutes, not
nights (v1's "overnight" estimate was 10–15× too high and traced to nothing).

**F6 — WORST-tier framing (experiment council).** The worse-than-ideal
mandate's job is "decisions must SURVIVE WORST," not "find the headline at the
tier built to favor the mechanism" (a persistent datum bias is nearly a
best-case for a chi-square gate; and F1 says WORST is where fusion may in fact
*regress*). ADR-0037's real finding appeared at EXPECTED tier — its §4.2
"expect the win at WORST" pre-registration was never even exercised.

## 3. Design decisions (v2, incorporating F1–F6)

**D1 — EKF flight-worthiness prerequisite (P0) — EXECUTED OFFLINE 2026-07-08,
REFRAMED by its own results.** The planned Q-tune ran ahead of schedule
(`scripts/ekf_q_replay.py`, `scripts/ekf_lockout_forensics.py`; ADR-0037 +
ADR-0041 addenda) and found: (a) NO over-wide-P signature at Q=64 — pooled
NIS/gate metrics sit on the TIGHT side, every lower PSD is monotonically
worse, and no lockout cascade exists (max 2 consecutive rejections anywhere);
(b) the ADR-0037 clean-rate "regression" is a **post-CPA scoring artifact**
(all 6 aborted flights intercepted at 0.675–1.339 m, 3/6 beating their paired
alpha-beta twin; the abort fired on the EKF's physically-correct post-CPA
range extrapolation crossing the `TERMINAL_RANGE_M=3.0` branch, which
alpha-beta's impossible negative coasted range never does). **Q-tuning is
RETIRED as a prerequisite. P0 is now:** (i) analyzer-level post-CPA abort
reclassification (zero flight-code change; applied to ALL arms, including a
re-examination of the markerless arms' aborts through the same lens); (ii)
the L0 confirm arm, gated on **EKF clean-rate parity under the corrected
scoring** + no NEW abort signatures. Keep Q=64 unless L0 says otherwise. If
L0 still fails: EKF cells BLOCKED, fall back to the L2 ladder, log honestly.

**D2 — Wiring shape (P1), polar-disciplined.** Under `--tracker ekf
--fuse-midcourse`, route accepted cue datagrams into `correct_cue`, with:
- **Polar split (F2 fix):** decompose the cue position innovation into
  along-LOS (range) and cross-LOS components in the measurement frame; the
  cross-LOS component gets heavily inflated R (or is dropped outright,
  matching `FusedTrack`'s "cue never touches the angle" discipline). The
  camera owns bearing; the cue contributes range/velocity only.
- World→relative conversion via own EKF2 position is legal (existing
  precedent); add a fixed **+0.15 m insurance term** to the cue R for the
  unmeasured own-position error (or measure EKF2-vs-gt for one flight first —
  ten minutes — and size it properly).
- `FusedTrack` is NOT constructed in EKF cells (no double-fusion); the cue →
  dash `TargetTracker` path stays IDENTICAL in all arms (baseline mechanism,
  not treatment). Alpha-beta cells keep today's `FusedTrack` byte-identical.
- New logged fields: the EKF's own `dn_hat/de_hat/dvn_hat/dve_hat` + P diag
  (F4 fix) — scored against `gt` in the RELATIVE frame.

**D3 — Handoff boundary (REDESIGNED per F1/F3; the "re-init or provably
decay" standard).** At the latch, for any EKF that received cue updates:
1. **Unconditional covariance re-inflation to a declared floor** on the
   position block at latch (decoupled from `--warm-handoff`), sized so the
   first post-latch camera updates are accepted — state carries over (warm
   handoff, disclosed), confidence does NOT. This is close to what
   `seed_from_polar` already does incidentally; make it explicit, tracker-
   gated, and unit-tested (P at first post-latch tick == declared floor).
2. **Adaptive gate-recovery (F1 fix):** after N consecutive post-latch camera
   innovations rejected by the chi-square gate, re-inflate P again
   (track-loss-recovery pattern) — the biased-prior escape hatch that static
   floors alone don't give.
3. **Instrumentation:** `ekf_cue_updates_post_handoff` (must be 0) AND
   `ekf_camera_gated_post_handoff` (visibility on F1) in `S2_RESULT`; P
   trace/diag logged at latch; a gain-washout offline metric (ticks post-latch
   until camera-update gain reaches cold-EKF parity — must be a small fraction
   of the 0.41 s terminal window, ADR-0023); static AST extension +
   injected-leak EKF variant in `test_honesty_static.py`; the S2_RESULT line
   marks structural-proof fields vs measured-counter fields so the two
   evidentiary tiers can't be conflated.

**D4 — Ladder (updated).** All arms n=16 where verdict-bearing (F5; cheap).
- **L0** (markerless, EKF-tuned, off) — doubles as the D1 confirm arm.
- **L1** (markerless, EKF-tuned, on) — the capstone cell.
- **L2** (markerless, αβ+FusedTrack, on) — isolates covariance gating vs any
  fusion.
- **L3 stress pair** — L1 vs L2 under WORST cue: reframed per F6 as a
  SURVIVAL check ("does fusion at least not regress under a badly biased
  cue" — F1 says this is a live risk), NOT the headline.
- Controls: (tag,αβ,off) reusable (byte-verified paired plan). The ADR-0037
  (tag,EKF@Q64,off) cell is Q-confounded — excluded from seeker-isolated
  comparisons; a (tag,EKF-tuned,off) re-run is one cheap arm if needed.
  **Pin `MARKERLESS_NN_WEIGHTS` for all markerless cells to the ratified
  default at run time; if that default changed since ADR-0038/0040 (e.g.
  seeker v2 ships), re-run the (markerless,αβ,off) baseline under the pinned
  weights (~10 min) rather than reusing a different instrument's numbers.**

**D5 — Metrics, pre-named (F4/F5/F6).** **Headline comparison: L1 vs L2 at
EXPECTED tier on mid-course RELATIVE-frame track RMSE (new field) — one
comparison, one metric, named now.** Supporting: handoff-reach rate and
dash-abort count reported with Wilson CIs, flagged-not-adjudicated at these n;
end-to-end miss expected NULL (kinematic ceiling, ADR-0023 — a miss null is
not a failure); `ekf_camera_gated_post_handoff` distribution (F1 canary).
SUCCESS = headline RMSE separation beyond paired noise + no survival-check
regression at L3. NULL = no separation (logged; the ADR-0018+0037+this
triple-null would be a coherent negative arc). REGRESSION at L3 = F1
confirmed in flight; the adaptive-recovery parameters get one tuning
iteration, then the honest verdict stands.

**D6 — Batch mechanics.** Unchanged: cells differ only by `MC_*` env +
`--extra-args`. Idle load, one sim, arms sequential, mc-batch skill loop.
Per-tick audit (`audit_per_tick.py`) runs on EVERY arm before its numbers are
used (now that it exists, this is the standing bar for all future arms — the
ADR-0038/0040 retro-audit found (a)/(b) clean 24/24 with three (c) findings
mechanically traced to seeker false-locks, not guidance dishonesty).

## 4. Build plan (ordered; each step separately verifiable)

- **P0** DONE-offline (Q-tune retired; see D1): remaining = post-CPA abort
  reclassification in the analyzers + L0 confirm arm → D1 gate verdict.
- **P1** Wire `correct_cue` polar-split + relative-state logging fields +
  own-position insurance term; flags-gated, byte-identical when OFF.
- **P2** D3 boundary: latch re-inflation + adaptive gate-recovery + both
  counters + P-at-latch logging + static/unit tests + gain-washout metric +
  `audit_per_tick.py` fusion extension (cue-activity marker per tick).
- **P3** Arms L1→L2→L3 (n=16 verdict cells), each per-tick audited.
- **P4** Analyze (relative-frame RMSE headline; Wilson CIs on rates), plots,
  results-ADR, NEXT/PROGRESS update.

Measured cost basis: ~65–75 s/flight ⇒ P0+P3 ≈ 80–100 flights ≈ **~2 h of
sim time** (traces to the 5-batch timing measurement, council review
2026-07-08). CPU-only for P0 replay.

## 5. Sequencing

After seeker v2 closes (the markerless baseline weights must be pinned first
— D4). P0–P2 are sim-free except the L0 confirm arm and can interleave with
other idle-machine work.
