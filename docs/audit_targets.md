# Audit targets — where to point a skeptical reviewer

*Prepared 2026-07-06 for a Fable 5 audit of the `worktree-s2-handoff` branch (26+
commits ahead of main). Ordered by stakes × likelihood-of-error × complexity. Each
item names the artifact, the exact claim under test, why it is load-bearing, and the
specific way it could be wrong. The auditor should try to REFUTE each claim, not
confirm it. Where I already have a doubt, I say so — those are the highest-value looks.*

## How to read this
"Load-bearing" = if this is wrong, conclusions downstream collapse. The single most
consequential recent event is **ADR-0023** (the miss is kinematic, not perceptual):
it overturned ADR-0014, redirected the whole solution effort, and justified the
metric change (ADR-0025). So its validity (Tier-1 item C) is the highest-value audit
in the repo. Start there and at the honesty boundary (A).

---

## Tier 1 — correctness the rest of the project rests on

### A. The one-way handoff honesty boundary (the whole headline)
- **Artifacts:** `scripts/m4_intercept.py` (the HANDOFF latch — cue socket close /
  holder null), `scripts/check_s2.sh` audits (a) zero cue reads post-handoff,
  (b) dash-before-engage, (c) cmd-vs-camera-LOS correlation. ADR-0010 #5, ADR-0013.
- **Claim under test:** after the latch the cue is *structurally* unreadable, and the
  terminal is *camera-only* — guidance never reads ground truth anywhere.
- **Why load-bearing:** the entire "comms-denied, the drone finishes on its own"
  thesis — the reason the project exists — is false if any ground-truth or cue value
  leaks into a terminal command. This is also the portfolio-integrity claim.
- **How it could be wrong / check:** does the audit (c) correlation of 0.7–0.82
  actually *prove* no-cheat, or could a command correlate with camera-LOS while still
  being partly fed by a leaked source? Is `SimClockHolder`'s `/clock` subscription
  truly side-effect-free (it makes no gz service calls — verify)? Does any Tier-1 or
  fusion code path read `ext_*` / cue state after the latch? Trace every consumer of
  the cue holder and confirm it is nulled and not re-populated.
- **Strengthened 2026-07-06 (this audit's own gaps, closed):** `tests/test_honesty_static.py`
  adds a fast, sim-free `ast`-based check of `scripts/m4_intercept.py` itself — every
  `cue_reader` read is guarded (or is the one designated latch site), and every `gt_*`
  read is allowlisted to logging/scoring only, never the command chain (proven by
  injecting a fake `v_perp += gt_range` into a scratch copy and confirming it fails).
  `scripts/check_s2.sh` audit (a) now also asserts every **non-detected** post-handoff
  ENGAGE row holds the previous command or hovers exactly (the old check only looked at
  `ext_*`, blind to what a non-detected tick commands). A new advisory audit (d)
  (`corr(d_cmd, d_gt)`, commanded-minus-camera vs ground-truth-minus-camera LOS
  deviation) was calibrated against every historical S2 flight in `logs/`: honest
  flights span roughly -0.99..+0.92, so no bound separates honest noise from a leak at
  this sample size — it is intentionally advisory/non-gating, not a hard pass/fail.

### B. Terminal guidance mechanization + coordinate-frame/sign conventions
- **Artifacts:** `scripts/m4_intercept.py` terminal law (strapdown `λ = ψ + β`,
  `a = N·Vc·λ̇`, α-β filters on λ and range), the **empirical** world→NED mapping
  (`north = world_y, east = world_x` — counterintuitive, ADR-0013), the FRD/ENU/NED/
  camera-OpenCV frame chain (GOALS.md), ADR-0006 transform-chain gotcha.
- **Claim under test:** the frame conversions and signs are correct end-to-end;
  `λ̇` has the right sign so pro-nav drives LOS-rate to zero (not away).
- **Why load-bearing:** GOALS.md names sign/frame errors as the #1 bug class here;
  ADR-0006 already caught one "same number, wrong parent" coincidence. A hidden sign
  error can still *look* like it works at one geometry and fail at mirror geometries.
- **How it could be wrong / check:** re-derive the world→NED mapping from first
  principles and confirm it matches the empirical one (why is it the "opposite of the
  naive guess"? is that a real gz/PX4 convention or a compensating double-error?).
  Check L2R vs R2L crossings are truly symmetric in the logs (an asymmetry hints at a
  sign bug). Verify α-β `λ̇` sign and the `N·Vc·λ̇` → world-lateral-velocity integration.

### C. ADR-0023 diagnosis validity — the linchpin (audit this hardest)
- **Artifacts:** `docs/terminal_diagnosis.md`, scripts in job tmp
  (`terminal_forensics.py`, `q6_experiment.py`), the ZEM r²=0.99 claim, the
  `½·a·t_go² = 0.72 m` bound, the "perfect camera cuts miss only 25%" experiment.
- **Claim under test:** the miss is ~70% kinematic (locked at handoff), ~20-25%
  recoverable mechanization, ~2% FOV, ~0% perception — so the lever is acquisition
  range + handoff geometry, NOT terminal perception.
- **Why load-bearing:** this reversed ADR-0014, killed the fusion/look-angle/seeker-
  hardware lanes, and justified the ratified proximity metric (ADR-0025). If it's
  wrong, the last day's strategic pivot is wrong.
- **How it could be wrong / check — MY OWN STANDING DOUBTS:**
  1. **Causal interpretation of r²=0.99.** Miss correlates with ZEM@handoff — but is
     that because the terminal window is *kinematically* too short (the claim), or
     because the terminal *perception drops out* so no correction is even attempted
     (the alternative)? The diagnosis rebuts this with the perfect-camera experiment
     (only 25% improvement) — but that is the **point-mass lab, which this project has
     documented 5× as UNDER-pricing terminal perception**. So the 25% could understate
     the true perception benefit. The model-INDEPENDENT backstop is the `½·a·t_go²`
     bound — **audit its inputs from the raw Gazebo logs**: is `t_go ≈ 0.41 s` and
     `a ≈ 8.7 m/s²` (MPC cap 12) correct, and is ZEM-delivered really ~1.69 m? If those
     three numbers hold, the bound holds regardless of the lab. If they're soft, the
     whole conclusion is softer than stated.
  2. **ZEM definition:** computed 3-D from gt positions + sim-time-differenced
     velocities. Is the velocity differencing noisy enough to inflate or deflate ZEM?
  3. Does the conclusion generalize beyond 6 m/s / this geometry (n=41, one speed)?

---

## Tier 2 — the tooling that produces every number

### D. guidance_lab.py — byte-identity claims + lab-over-optimism discipline
- **Artifacts:** `scripts/guidance_lab.py` (very large; studies behind `--adr0014`,
  `--adr0015`, `--p6-fusion`, `--jam-envelope`, `--tier1`), `PERCEPTION_DEFAULTS`.
- **Claim under test:** every study is feature-guarded so the default path is
  BYTE-IDENTICAL to before; and the lab is only ever trusted for RANKING, never
  absolute Pk.
- **Why load-bearing:** if a new RNG draw leaked into the default path, every prior
  "byte-identical baseline" claim (and the comparisons built on them) is compromised.
  And any place a lab absolute number is quoted as a result violates "Gazebo decides."
- **How it could be wrong / check:** independently diff the default `--quick`/`--adr0014`
  output against the pre-study git revisions at several seed counts (I checked `--quick`
  and `--adr0015`; the auditor should check ALL study flags and higher seed counts).
  Grep every doc/ADR for a lab number presented as a conclusion without a Gazebo
  confirmation. Confirm `AlphaBetaFilter.correct(gain_scale=1.0)` is truly IEEE-exact.

### E. Statistical integrity — small-n verdicts + Pk math
- **Artifacts:** `scripts/mc_analyze.py` (Pk-vs-radius, Wilson CIs), all the N=8-12
  paired A/B verdicts (ADR-0015 2nd addendum, ADR-0018 addendum, the Tier-1 result).
- **Claim under test:** the significance language is honest given ~1 m run-to-run
  terminal-dropout noise at n=8-12; the Wilson CIs and Pk counts are computed right.
- **Why load-bearing:** several "verdicts" rest on sub-0.5 m paired deltas at n=8.
  Over-claiming significance would be a portfolio-credibility hit.
- **How it could be wrong / check:** re-derive one Wilson interval by hand against
  `mc_analyze.py`. Audit whether any ADR states a conclusion the n doesn't support
  (I tried to always say "not significant at n=8" — verify I actually did, everywhere).
  Check the matched-load rule was honored on the batches being compared (ADR-0015 2nd
  addendum documents a load confound — are any OTHER cross-batch comparisons confounded?).

### F. s2_cue_mock.py realism fidelity + a KNOWN unadopted correction
- **Artifacts:** `scripts/s2_cue_mock.py` (σ_R, datum bias, latency+jitter, Markov
  dropout, velocity emission), vs the lab's `PERCEPTION_DEFAULTS`, vs ADR-0017.
- **Claim under test:** the Gazebo cue model and the lab cue model represent the SAME
  degraded sensor, so "lab ranks / Gazebo decides" is comparing like with like.
- **Why load-bearing:** if the two realism models diverge, lab rankings don't transfer.
- **KNOWN GAP to verify:** ADR-0017 found the mock's `σ_R = 0.4 + 0.008·R²` is ~180×
  too steep as stereo NOISE and should be `c ≈ 4.45e-05` + a separate `--datum-bias-m`.
  **This correction is NOT yet adopted in `s2_cue_mock.py` defaults** (only the
  jam-envelope lab driver uses it). So every realistic-cue Gazebo batch to date used a
  σ_R that is HARSHER than physics (conservative for Pk, but mis-attributed noise-vs-
  datum). Confirm this is accurately disclaimed everywhere it matters and that no
  conclusion depended on the steep curve's shape.

---

## Tier 3 — sourced design claims (lower stakes, still worth a pass)

### G. stereo_model.py physics + perception-doc external numbers
- **Artifacts:** `scripts/stereo_model.py` (σ_R = R²·σ_d/(b·f), calibration-drift term,
  the 2 m knee), `docs/compute_setup.md` (Hailo ~35 fps / Orin NX fps / latency budget),
  `docs/kill_mechanism.md` / `docs/seeker_upgrades.md` (kill radii, part prices).
- **Claim under test:** the stereo error formula and its three-tier constants are
  physically right; the load-bearing sourced numbers (velocity-emission lever, Hailo
  real-time claim, kill radii) hold.
- **How it could be wrong / check:** re-derive σ_R from the pinhole+disparity model;
  sanity-check the calibration-drift term dominates as claimed; spot-check 2-3 cited
  URLs still support the numbers (vendor-fps claims flagged as marketing?).

### H. Metric ratification honesty + ADR-chain internal consistency
- **Artifacts:** ADR-0025 (proximity metric), ADR-0014 honesty boundary, ADR-0021,
  the README honesty section, `docs/terminal_solutions_plan.md`.
- **Claim under test:** adopting a proximity/lethal-radius metric is honest (radius set
  by mechanism physics, full curve shown) and NOT goalpost-moving to flatter a bad miss.
- **How it could be wrong / check:** pressure-test the strongest counter-argument — "you
  changed the success metric right after finding you couldn't hit the old one." Is the
  defense (ram/net radii are physics-set; the kinematic floor makes a lethal radius
  *required* not chosen; M0-M4 gates unchanged; full curve reported) actually airtight,
  or does it read as convenient? Also check the ADR chain (0008–0025) for internal
  contradictions the rapid iteration may have introduced.

---

## Meta-checks (cheap, high-signal)
- **Reproduce one gate end-to-end** (`check_m4.sh` or `check_s2.sh`) from a clean state
  and confirm the exit code + numbers match the claimed record.
- **Verify the branch is coherent:** no committed secrets, no accidental ground-truth
  in a guidance path, `.gitignore` sane, every ADR referenced by a real commit.
- **Confidence calibration:** for each headline claim, is the stated confidence matched
  by the evidence (n, tiers, lab-vs-Gazebo)? Flag any that read more certain than earned.
