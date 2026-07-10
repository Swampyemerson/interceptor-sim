# #40 mount-compose re-fly — pre-registration (ADR-0067 addendum mandate)

**Written 2026-07-10, BEFORE any re-fly arm flies. This file must be committed
before the first boot.** Purpose: commit the success criteria in advance so the
verdict cannot be reverse-fit to the result — the same discipline that caught
the v3 seeker false-win (ADR-0061) and that the ADR-0067 addendum explicitly
requires for this re-fly ("pre-register the #40 re-fly bar BEFORE it flies").

## What is being tested

ADR-0067 flew the fixed up-tilt mount **uncompensated** (m4 assumed a zero
mount): the availability win is real (dash-above-FoV 32% → 0%, in-FoV 61% →
~90%), the paired acquisition gain is small (+2–3 m vs the within-sweep up00
control), and the terminal cost is dose-dependent (up15: miss median 1.81 →
2.42 m, Pk@2.5 8/8 → 4/8, engBelow 12%). #40 composes the fixed mount rotation
into the FIX-A full-attitude LOS derotation (`m4_intercept.py
--cam-mount-up-deg`, a constant R_y(+mount) pre-rotation on the optical→body
ray — see ADR-0062's loud comment at the permutation).

- **Stage 1 (terminal re-fly):** does compensation restore terminal parity with
  up00 at the up15 candidate angle?
- **Stage 2 (recovery re-test):** does a tilted+compensated camera un-HOLD the
  ADR-0059 comms-denied recovery NULL (which was perception-availability-bound)?

## Stage 1 — arms (weave 12 m/s, adopted deployment config, master-seed 42)

| Arm | Physical mount (shadow variant) | `--cam-mount-up-deg` | Out CSV |
|---|---|---|---|
| up00 re-fly control | up00 (pose unchanged) | 0 (byte-identical by the guard + identity tests) | `logs/mc_uptilt_refly_weave12_up00.csv` |
| up15 compensated | up15 | 15 | `logs/mc_uptilt_refly_weave12_up15c.csv` |

n=8/arm, paired seeds (same master-seed-42 plan; prefix-stable RNG), arms
sequential with ≥120 s cooldown, idle load only, launched via
`scripts/uptilt_ab_arm.sh --compensate` (same shadow-symlink machinery as
ADR-0067, trap-always cleanup). A **fresh up00 control flies within-sweep** per
the ADR-0067 addendum rule: cross-era acquisition comparisons are INVALID;
within-sweep paired controls only.

## Stage 1 — pre-registered metrics and bars

Scored by `scripts/experiments/uptilt_ab_analyze.py`, which now prints
**per-seed deltas + sign counts + signed AND absolute medians** (this
pre-registration binds those, not delta-of-medians, as the reported
statistics). Sign-test note: at n=8 paired, 7/8 one-direction ≈ p 0.035
(one-sided); anything below 7/8 is "not significant at this n".

**Control identity (pinned).** The control is the WITHIN-SWEEP
`logs/mc_uptilt_refly_weave12_up00.csv` — NOT the legacy ADR-0067
`mc_uptilt_weave12_up00.csv` (cross-era acquisition comparisons are formally
INVALID, ADR-0067 addendum). When the ride-along legacy arms are passed too,
the analyzer selects the `refly` mount-0 arm as control and prints a `[WARN]`
line if more than one mount-0 arm is present; the reported PRIMARY read MUST
be the one whose control header names `refly_up00`.

**Denominators (pinned).** The sign bars are counts of **exactly N** (8 at
Stage 1, 16 on extension). If any pair is lost (crashed flight, missing ulog,
empty/`nan` miss) the analyzer prints a `[!] denom != expected` warning and the
lost seed is **re-flown before scoring** — a shrunken denominator is never
scored, because attrition is asymmetric toward PASS (a would-be 7/8 FAIL
becomes 6/7).

**0. Validity gates (checked before any verdict is read):**
- Shadow/mechanism check (CORRECTED — the naive "expect ≈ +mount" reading is
  INVERTED). The analyzer adds the mount ANALYTICALLY, and `vert_body` is
  independent of the sensor pose, so `d(top_margin) = d(vert_body at first-det)
  + mount`. A **FAILED** swap flies a LEVEL camera → identical first-det
  geometry → `d(top_margin) ≈ +mount EXACTLY` **and** `d(first_det_range) ≈ 0`
  — that pair is the INVALID signature (the analyzer flags it). A **WORKING**
  swap detects the target higher/earlier → `d(top_margin)` sits **BELOW**
  +mount (flown ADR-0067: +9.6/+18.6/+27.5 vs 15/25/35) **with a POSITIVE**
  `d(first_det_range)` (+2–3 m). VALID iff `d(top_margin)` is materially below
  +mount AND `d(first_det_range) > 0`; the "≈ +mount, ≈ 0" pattern → INVALID →
  stop.
- Machinery-stability tripwire: fresh refly-up00 vs the ADR-0067 up00 arm (same
  seeds, code byte-identical at mount 0 per the identity tests): the bound
  statistic is the **median-ABSOLUTE** per-seed miss delta (`med|Δ|`, printed
  by the analyzer — a signed median masks symmetric drift) **≤ 1.0 m**, and
  median first-det difference ≤ 2 m. Larger → STOP, diagnose environment drift
  before scoring anything.
- Per-tick no-cheat audit (`scripts/audit_per_tick.py`) re-run on ≥1 flight
  per arm; must PASS (the compensation consumes only the static mount constant
  + own-state EKF attitude + camera measurements — no `gt_*`).

**1. PRIMARY — terminal parity (paired per-seed miss, up15c − up00):**
The bound statistic is the **median of the paired per-seed deltas** (what
`uptilt_ab_analyze.py` prints), NOT the delta of the arm medians — on the
flown ADR-0067 data these differ (+0.84 m paired-median vs +0.61 m
delta-of-medians for uncompensated up15, which was worse on **8/8 seeds**);
binding one statistic up front removes the metric-shopping room.
- **PARITY PASS:** median paired delta ≤ **+0.30 m** AND up15c worse on
  **≤ 5/8** seeds.
- **FAIL:** up15c worse on **≥ 7/8** seeds (sign-significant regression,
  the uncompensated 8/8-worse pattern not broken) OR median paired delta ≥
  **+0.84 m** (no better than the uncompensated penalty on the same seeds).
- **Between → INCONCLUSIVE at n=8** → the n=16 extension below.
- Pk@2.5 and Pk@2.8 reported per arm with Wilson-CI language only — n=8 is
  descriptive, never a binomial headline (ADR-0041 F5).

**n=16 extension bars (pre-registered NOW so they are not post-hoc
selectable).** Extend BOTH arms to n=16 paired (same master-seed plan prefix,
seeds 1–8 retained). The sign bars do NOT scale as the literal `7/8` fraction
(that would imply ≥ 14/16, one-sided sign p = 0.0021 — far stricter than the
registered ≈ 0.035 intent); they are **α-matched** to the n=8 bars:
- **PASS:** median paired delta ≤ +0.30 m AND up15c worse on **≤ 10/16** seeds.
- **FAIL:** up15c worse on **≥ 12/16** seeds (one-sided sign p = 0.0384) OR
  median paired delta ≥ +0.84 m.
- **Between at n=16 → INCONCLUSIVE-FINAL:** recorded as INCONCLUSIVE, **NO
  adoption, no further extension** (the terminal rule — the extension does not
  recurse). Sequential-look note: this is a single planned interim look at
  n=8 then one extension to n=16; no α-spending beyond the two registered
  thresholds is claimed, and both thresholds are fixed here before any arm
  flies.

**2. SECONDARY — acquisition retention (paired first-det range, up15c − up00):**
expected ≈ +2–3 m (ADR-0067). Direction check: median > 0 AND positive on
≥ 5/8 seeds. Compensation acts on guidance, not perception — a clearly negative
result here flags a harness problem (wrong shadow, wrong flag) and triggers
investigation BEFORE any verdict, not a design conclusion.

**3. WATCH — engBelow (engagement-below-FoV fraction):** report per arm +
per-seed deltas; **no pass/fail**. Uncompensated up15 read 12% (worst arm,
ADR-0067 addendum). If it persists ≳10% compensated, that is evidence FOR #46
adaptive tilt (bottom-of-frame loss is camera geometry; guidance compensation
cannot recover pixels that never arrive) — record it either way.

## Stage 1 — pre-registered decision rule

- **PARITY PASS + acquisition retained** → up15 fixed mount **ADOPTED
  (interim, pending the #46 adaptive-tilt A/B)** and Stage 2 flies
  tilted+compensated.
- **FAIL** → NO mount adoption; compensation did not close the terminal gap;
  #46 adaptive tilt becomes the primary perception-availability lever. Stage 2
  may still fly (the availability question stands independently of adoption),
  but no adoption claim of any kind.
- **INCONCLUSIVE at n=8** → n=16 extension with the α-matched bars above;
  a second "between" read is INCONCLUSIVE-FINAL (no adoption, no recursion).

## Stage 2 — tilted+compensated recovery re-test (flies ONLY after the Stage-1 verdict is logged)

Re-run the #39 coast-search jam arm: r18 pre-acquisition cutoff, n=16,
master-seed 42, adopted config + `--coast-search` + up15 shadow +
`--cam-mount-up-deg 15`. Scored by `scripts/check_jam_mc.py` (coast arm as
`--fixon`), exactly as #39.

**Absolute bars lifted from `docs/adr0059_recovery_preregistration.md`; the two
deviations from that registration are disclosed here (NOT "verbatim").** Let
RH = REAL(-ish) handoffs /16, J = joint success /16:

- **RECOVERY DEMONSTRATED** iff RH ≥ 11/16 AND J ≥ 8/16 AND the `never` count
  drops correspondingly. (These absolute bars are verbatim from ADR-0059's
  registration and are self-sufficient: RH ≥ 11 already clears any prior
  baseline, so no cross-sweep delta gates this verdict.)
- **PARTIAL (cross-sweep SUGGESTIVE only)** iff ΔRH ≥ +4/16 over the flown
  coast NULL AND RH < 11/16. **Deviation 1 (disclosed):** ADR-0059 registered
  the +4/16 delta vs `fixon`-no-coast (flown RH 2/16); this doc rebases it to
  the flown coast NULL (RH 1/16), which loosens the PARTIAL floor by one flight
  (RH ≥ 6/16 → ≥ 5/16). **This verdict rests ENTIRELY on a delta against an
  arm flown in a PRIOR sweep** (different camera model, possible environment
  drift) with **no within-sweep untilted-coast control** — the same
  cross-era-invalidity concern Stage 1 guards with a fresh control. Therefore
  PARTIAL is reported as *cross-sweep suggestive*, never as a standalone
  positive result; if a PARTIAL read matters, it is confirmed by flying a
  within-sweep untilted `--coast-search` control pair before any claim.
- **NULL** otherwise.
- **MECHANISM metric — the availability claim itself (this is the load-bearing
  one for the mount).** Camera-never-detected count during coast (#39 baseline:
  10/16). **Deviation 2 (disclosed):** ADR-0059 lists never-detected only as an
  unthresholded watch/stratum metric; the **≤ 4/16** bar here is NEW (pre-hoc,
  registered before flying, legitimate — but not verbatim). If never-detected
  collapses (≤ 4/16) but RH stays < 11/16 → "availability recovered, conversion
  limited": comms-denied stays HELD, re-scoped from FoV availability to
  far-range recall/terminal conversion (ADR-0061 territory).

Only RECOVERY DEMONSTRATED un-HOLDs "works comms-denied", and even then scoped:
weave/12 m/s, r18 cutoff, sim tier, with the mount fitted.

## Honesty constraints (bind the whole re-fly)

- The compensation consumes ONLY: the static mount angle (a configuration
  constant of the airframe, known a priori), own-state EKF attitude, and camera
  measurements. No ground truth. Per-tick no-cheat audit re-run before any
  verdict is logged.
- The acquisition/handoff gates stay yaw-only (ADR-0062 scope boundary,
  reviewed SOUND): at 15° mount the 2D camera-implied-position error at the
  8 m cue gate is ≤ ~1.5 m — margin holds. Gate derotation remains a tracked
  #40 follow-up and is NOT silently changed inside this A/B.
- Evidence CSVs + analyzer output commit with the verdict; THIS file commits
  before the first boot; any deviation from this plan is logged as a deviation,
  not edited away.
