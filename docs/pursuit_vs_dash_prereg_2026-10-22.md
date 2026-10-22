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
