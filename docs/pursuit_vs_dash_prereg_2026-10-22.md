# Pre-registration: chase pursuit vs blind dash under injected aim error (2026-10-22)

**Registered BEFORE any arm ran.** Builder claim to test (ruled in conversation,
2026-10-22 morning): *"the chase pursuit definitely beats a blind dash with injected
error."* Field interception day is this weekend; this study is the evidence under the
weekend's config choice. The standing headline ("the camera has not beaten a well-aimed
blind dash at any speed or aim error tested") predates the pursuit port (ADR-0105/0118)
and was measured on arms whose guidance was structurally blind to the dominant error
term (ledger `terminal-vertical-channel-decided-not-built`); it must be re-earned or
retired on the current architecture.

**Tier: isim (design-time surrogate).** Lab ranks, Gazebo decides: this result is a
RANKING with a documented sim-to-real gap; the confirming step is a Gazebo
camera-in-the-loop rung on the winning cells (queued, machine kept free today for it).

## Config (canon base — the stern/target-height studies' exact base, reproduce-canonical-geometry rule)

`Scenario(target_speed_ms=9.0, cross_range_m=6.5, lead_dist_m=16.2, cam_fx_px=933.0,
tag_facing="rear", scatter=Scatter(), seed=s)` for paired seeds s ∈ 0..15 (n=16/cell).
`cam_fx_px=933` ⇒ HFOV ≈ 69° — this base already models the DELIVERED ~70° lens the
builder just ruled to keep, so the result speaks directly to the weekend hardware.
`Scatter()` = all v5 honest errors on (heading σ2°, height guess, decode realism, …);
note `heading_sigma_deg=2.0` rides ON TOP of the injected `aim_error_deg` in every arm.

**Arms (same seeds, same geometry, only the terminal differs):**
- **BLIND** — `concept="flyby", terminal="stock", decode_overrides={"p_max": 0.0}`:
  the coded dash with the seeker rendered permanently blind (zero decode probability,
  TRUE-seeker override only — guidance untouched). Pure open-loop ballistics + whatever
  the stock terminal does with zero detections (expected: never engages).
- **PURSUIT** — `concept="flyby", terminal="pursuit"`: the RealFlightSM flight-code
  port (parity_trace_2026-09-22) — the closest isim model of what flies Saturday
  (`real_flight.py --terminal pursuit`, the ADR-0118 default).
- **HYBRID** — `concept="hybrid"`: the canon concept of the recent registered studies,
  for continuity with the stern/target-height numbers.

**Axis:** `aim_error_deg` ∈ {0, 5, 10, 20}. (Target-height error is NOT re-swept here —
it has its own registered study, `isim/specs/target_height_error_2026-10-22.md`,
measured budget ±2 m.)

**Metrics per cell:** n flown / n scored (every exclusion counted — no vacuous
verdicts), median & p90 miss, pct inside 0.35 m (`pct_hit`), pct_engage.

## Predictions (written before running)

1. **BLIND collapses by 5°**: the Gazebo seven-point curve measured ~0.1 m of miss per
   degree and 0/8 inside 0.35 m at 5° off; expect isim's blind arm to reproduce that
   shape (median miss roughly ∝ aim error; pct_hit ≈ 0 by 5–10°).
2. **PURSUIT holds through ≥10°**: the chase re-acquires and closes the injected error;
   median miss stays within ~2× its 0° value through 10°, degrading (not cliffing)
   at 20°. Mechanism precedent: the parity work's aim20 cell went 70%→100% after
   defect #5 was fixed, and hybrid_v7's reference run scored 59% at aim 20°.
3. **HYBRID lands between or above PURSUIT at high error** (its sprint leg banks range
   before chasing); no adoption rides on hybrid-vs-pursuit, it is context only.

## Decision rule (before seeing numbers)

- **Claim CONFIRMED** if, at 5° AND 10°, PURSUIT's pct_hit exceeds BLIND's by ≥3/16
  paired flights AND its median miss is lower on ≥12/16 paired seeds per cell. Then the
  contract headline is rewritten: the old "camera never beat a well-aimed dash" line is
  RETIRED (superseded, not deleted — ledger note), replaced by the measured curve, with
  the isim-tier label and the Gazebo rung as the confirming step.
- **NULL / REFUTED** (pursuit does NOT beat blind at 5–10°): the weekend should fly the
  best-aimed blind dash instead of the camera terminal, the builder's claim is reported
  back as not supported at this tier, and the old headline STANDS. A null here is
  load-bearing either way — it picks Saturday's config.
- At 0° no claim is scored (both should work); it anchors the baselines.

## Results (filled in after the run — nothing above this line changed)

Ran 2026-10-22, 192/192 flights scored, zero exclusions. Full per-run data:
`runs/pursuit_vs_dash_20261022/` (perrun.csv, results.json, findings.md).

| aim° | BLIND med / hit | PURSUIT med / hit | HYBRID med / hit | paired: pursuit wins / hit Δ |
|---|---|---|---|---|
| 0  | 1.214 m / 0/16 | 0.204 m / 12/16 | 0.429 m / 7/16 | 15/16, +12 |
| 5  | 1.665 m / 0/16 | 0.394 m / 6/16  | 0.684 m / 3/16 | 16/16, +6 |
| 10 | 2.123 m / 0/16 | 0.263 m / 10/16 | 1.711 m / 3/16 | 15/16, +10 |
| 20 | 3.027 m / 0/16 | 3.292 m / 7/16  | 3.132 m / 0/16 | 7/16, +7 |

**Verdict against the registered rule: CONFIRMED.** At 5° AND 10° pursuit beats blind
on hit delta (+6, +10 ≥ 3) and paired-seed miss (16/16, 15/16 ≥ 12). Pursuit engaged
on 64/64 flights across all cells.

**Prediction scorecard:** (1) blind slope CONFIRMED — 0.091 m/deg, matching the Gazebo
seven-point curve's ~0.1 m/deg — but the intercept differs: blind scores 0/16 even at
0° here, because this is the canon CHASE geometry (crossing offset, stern approach),
not the head-on dash-optimized geometry the 13/16 Gazebo result was flown on. (2)
Pursuit holding through 10° CONFIRMED (median 0.26–0.39 m); the 5°-vs-10° hit
inversion (6/16 vs 10/16) is binomial noise at n=16, not signal. At 20° pursuit is
bimodal — 7/16 still hit, the rest miss ballistically — median is not a useful summary
there. (3) Hybrid below pursuit everywhere, consistent with chase-only (ADR-0103).

**Honest scope limits:** isim tier (ranking; Gazebo rung is the confirming step); the
blind arm here is NOT the old headline's "well-aimed dash on its own geometry" — this
result says pursuit dominates blind ON THE OPERATIONAL CHASE GEOMETRY under cue error,
which is the weekend's question. The old headline is retired as superseded-by-scope,
not refuted on its own geometry. **Saturday's number: the cue must be good to ~10°;
at 20° the kill probability is a coin flip (7/16).**

---

## AMENDMENT — Gazebo confirming rung (registered 2026-10-22, BEFORE any flight)

**Question:** does the isim result's load-bearing half — pursuit degrades gracefully
under injected cue error through ~10° — hold camera-in-the-loop in Gazebo?

**Configuration (one new knob on the canonical harness, nothing else):**
`bash scripts/xcheck_fly.sh <idx> <idx> <outdir> --pursuit-brake --cue-aim-error-deg ±θ`
— the ADR-0118/0120 flying recommended config exactly as re-fly #5 flew it
(prereg5: fresh boot per flight, idle machine, `--engage-max-s 25`, centre-to-centre
CPA from scoring-only gt), plus the new `--cue-aim-error-deg` in
`scripts/gazebo_pursuit_crosscheck.py`, which rotates the BELIEF handed to
real_flight (`--target-start/--target-vel`) about the launch origin by θ using
isim/scenario.py's exact pursuit-family convention (clockwise azimuth, start AND
velocity), while the mover keeps the TRUE track. θ=0 is bit-identical to the
historical path (verified; driver self-test PASS).

**Cells and n:** 5° and 10°, n=8 fresh-boot flights per cell, split 4 at +θ / 4 at −θ
(the Gazebo track is one fixed geometry, so sign alternation is the cheap symmetry
check isim got from its direction field; medians reported pooled per cell with
per-sign counts shown). **0° control: CITED, not re-flown** — re-fly #5
(`docs/xcheck_gazebo_pursuit_prereg5.md`): median 0.243 m, 6/8 ≤ 0.35 m, same
harness, same config, same machine, flown 2026-09-24.

**Declared mismatch vs the isim A/B:** the Gazebo camera is the sim's ~100°-class
optic (live CameraInfo), NOT the delivered-lens fx≈933 the isim sweep used. Wider FOV
makes re-acquisition under cue error EASIER, so this rung bounds the cue-error
mechanism at the sim optic; the 70°-lens funnel re-earn is a separately queued item
(ADR-0126 cost note). No blind-dash arms here — the dash half of the claim is
isim-tier + the standing seven-point Gazebo curve; this rung spends its boot budget
on the half Saturday flies.

**Registered prediction** (from isim 5°: median 0.394 m, 6/16; 10°: 0.263 m, 10/16;
and the 0° Gazebo baselines 0.243/0.343 m): both cells land **median ≤ 0.5 m with
≥3/8 ≤ 0.35 m**, i.e. within the standing campaign bar's median clause and no worse
than ~2× the 0° baseline; no pre-pass failsafe aborts; camera-driven throughout;
`FAULT own_vel_fallback` 0/16.

**Criteria (before flying):**
- **CONFIRM** iff BOTH cells: median ≤ 0.5 m AND ≥3/8 ≤ 0.35 m AND 0 pre-pass aborts.
  → Saturday's ~10° cue budget stands with a Gazebo-tier datum behind it.
- **DEGRADE** if a cell's median ∈ (0.5, 1.0] m: the budget TIGHTENS — plan the cue
  to the last confirmed angle (5°, or 0° if 5° also degrades), fly closer standoffs,
  and say so in kill_day_protocol before Saturday.
- **NULL/FAIL** if a cell's median > 1.0 m or any pre-pass abort: the isim ranking
  did not transfer at this optic — Saturday plans the best-aimed geometry (cue as
  close to truth as procedure allows), treats the chase as the experiment rather
  than the plan, and the discrepancy gets a tick-trace before any further claim.
- Sign asymmetry note: if +θ and −θ counts differ by ≥3/4 within a cell, report the
  asymmetry and median per sign — the pooled median is then geometry-specific, and
  the budget statement must carry the weaker sign.

**Boot budget:** 16 fresh boots (+re-flies only for boot failures, counted), well
under the ~48/arm-cluster proven-safe size; dmesg dxgk baseline taken before the
first arm; ≥120 s cooldown between cells.

### RESULT (2026-10-22, 16 flights flown as registered — scored same session)

**Verdict against the registered criteria: CONFIRMED.**
- **5° cell (n=8, ±4/4):** median **0.259 m**, **6/8 ≤ 0.35 m** (bar: ≤0.5 / ≥3/8). Per
  sign: +5° median 0.249 (4/4 inside); −5° median 0.354 (2/4) — below the registered
  ≥3/4 asymmetry trigger, reported anyway: the −θ side points the belief across the
  track and ran fewer detections (52–60 vs 85–89 consumed).
- **10° cell (n=8, ±4/4):** median **0.244 m**, **8/8 ≤ 0.35 m**; signs symmetric
  (0.242 / 0.246).
- **0° control (cited, re-fly #5):** 0.243 m, 6/8 — i.e. **through 10° of cue error the
  chase costs ~nothing at this optic** (0.243 → 0.259 → 0.244).
- Instruments/safety: 0 pre-pass aborts (16/16 reached breakoff_complete), camera-driven
  throughout (52–93 detections consumed), `FAULT own_vel_fallback` **0/16**, 0 boot
  failures, dxgk delta +123 over 16 boot/kill cycles (benign teardown noise, no wedge),
  teardown clean. Data: `runs/pursuit_vs_dash_20261022/gazebo/` (per-flight CSVs,
  results.csv, summary.json, batch_stdout.log).

**Registered meaning applied:** Saturday's ~10° cue budget now has a Gazebo-tier,
camera-in-the-loop datum behind it. Honest scope, unchanged: sim camera (~100°-class
optic, NOT the delivered 70° lens — wider FOV flatters re-acquisition; the 70° funnel
re-earn stays queued per ADR-0126), perfect-physics windless world, upright tag. The
isim 20° cliff was not re-flown here (boot budget spent on the cells Saturday flies);
the ~10° budget statement stands on isim for the far side and Gazebo for the near side.
