# Pre-registration 3: Gazebo pursuit cross-check re-fly WITH the brake package

Registered 2026-09-24, BEFORE any flight of this arm. Follow-up to prereg 2
(`docs/xcheck_gazebo_pursuit_prereg2.md`, registered NULL-with-instruments-green:
median CPA 0.967 m, range-bias and cadence fixes landed, bar failed) and executes the
builder's rulings of 2026-09-24 (ADR-0115 addenda: option (a) ran and stopped per its
rule; then "yeah do the brake package for gazebo").

## What changed since prereg 2 (one thing)

The pursuit terminal flies the ADOPTED brake package — `--pursuit-brake` on the real
CLI (one switch, no tuning surface): stopping-distance cap on the horizontal relative
command (a = 3 m/s², lead 0.45 s, `brake_horizontal_only`) + vertical arrival-sync
(`brake_vert_sync`). Evidence base: isim/specs/brake_shaping_prereg_2026-09-24.md
(two registered rounds + trace: honest-plant nominal 62→98% contact, closing at CPA
3.1→1.5 m/s; known alt+3 tail documented — NOT this re-fly's geometry). Wiring is
construction-tested (`test_pursuit_brake_flag_sets_the_adopted_package_and_defaults_off`)
and default-off byte-identical. Driver + `xcheck_fly.sh` forward the flag verbatim.

## Registered prediction (run BEFORE the re-fly; scratch harness, pin reproduced)

Matched-optics scenario (canonical crossing, fx 540.3, tag 0.5 m, rear, no scatter),
ENGAGE fit + pose supply, n=50/arm (`logs/brake_shaping_20260924/refly3_prediction.txt`):

| arm | cadence | median | p90 | %≤0.35 |
|---|---|---|---|---|
| pin (prereg2 arm d) | 5.2/s | **0.087 m** (exact reproduction) | 0.177 | 100% |
| brake | 5.2/s | 0.028 m | 0.049 | 100% |
| brake | 10.4/s | 0.020 m | 0.046 | 98% |
| brake | full | 0.093 m | 0.141 | 100% |

isim band with the package: **0.02–0.09 m median**. HONEST expectation: re-fly #2
demonstrated a ~3x isim→Gazebo residual on this scenario even with honest plant +
instruments green, and the brake package is the isim-attributed fix for exactly that
residual — so the registered claim is directional: if the attribution is right, most
of the 0.967 → 0.09-band gap closes; a fair point expectation is **~0.1–0.3 m median**.
Perfect-cue, windless, upright-tag caveats unchanged (best-case framing).

## Re-fly configuration (registered)

Identical to prereg 2 in every respect (fresh boot per flight, idle machine, n = 8,
`bash scripts/xcheck_fly.sh 1 8 <outdir> --pursuit-brake`, centre-to-centre CPA from
scoring-only gt) EXCEPT the one flag above. No other change; no tuning.

## Adopt / reject criteria (registered before flying — SAME bar as preregs 1–2)

PASS iff over the 8 flights: (1) ≥6/8 CPA ≤ 1.0 m; (2) median CPA ≤ 0.5 m;
(3) ≥2/8 ≤ 0.35 m; (4) 0 failsafe aborts before the pass; (5) every flight consumes
camera detections through ENGAGE.

**Instrument clauses (fix-effect-observed, pass/fail independent of CPA):**
- C1/C2 carried over from prereg 2 (KF range bias at 6 m within ±0.30 m; in-ENGAGE
  consumed cadence ≥ 9 det/s with median tick dt ≤ 0.06 s).
- C3 (brake landed): every run log prints the `[terminal] pursuit BRAKE PACKAGE ON`
  line, AND the median closing speed over the last second before CPA is LOWER than
  re-fly #2's same statistic computed with the same instrument on the archived
  `logs/xcheck_gz_20260923_fixed/f*.csv` (comparative, same-ruler — no absolute guess).

## What the outcomes would mean (written now)

- **PASS + C1–C3 green:** the transfer gap is closed as attributed end-to-end
  (range bias + cadence + hot approach). The default-swap question
  (`docs/next.md` 0(c)) reopens for the builder. Direction-only language for any
  isim-vs-Gazebo margin; no real-world claim (perfect cue, no wind, upright tag,
  not the OV9281).
- **PARTIAL (bar failed but median improves ≥40% vs 0.967 m, C green):** braking
  transfers partially; the residual continues OUTSIDE the now-six exonerated
  suspects — next step is a fresh tick-trace on the new CSVs, not tuning.
- **NULL (median improvement <40%, C green):** the isim braking diagnosis does not
  transfer — a genuine isim-fidelity finding; record it against the ENGAGE fit and
  stop (no Gazebo tuning).
- **Any C red:** the package/instruments did not land in the flying code — fix the
  delivery, not the law, and re-fly under this same registration.

## RESULT (2026-09-24, n=8 flown as registered — scored by the head session)

CPAs sorted (m): 0.125 · 0.313 · 0.537 · 0.708 · 1.147 · 1.528 · 1.891 · 2.250.
**Median 0.928 m.** Logs: `logs/xcheck_gz_20260924_brake/` (driver CPA vs independent
tick-trace recompute agree <0.01 m on all 8; instrument.json alongside).

Registered criteria: (1) 4/8 ≤ 1.0 m — FAIL; (2) median 0.928 — FAIL; (3) **2/8 ≤
0.35 m — PASS, the first sub-0.35 flights this cross-check has ever produced** (0.125,
0.313); (4) 0 pre-pass failsafe aborts — PASS; (5) camera-driven throughout — PASS
(65–108 consumed). Median improvement vs re-fly #2: 0.9665 → 0.928 = **4.0% < the 40%
partial bar.**

Instrument clauses: **C1 GREEN** (KF r_hat error median −0.065 m; pose-range bias at
capture −0.010 m — the S2 fix keeps holding; the raw depth channel still reads −1.38,
confirming the pose channel carries the KF). **C2 HALF-RED** (median tick dt 0.056 s ≤
0.06 GREEN, but consumed cadence 8.01 det/s < the 9 det/s clause — the every-tick
mechanism is intact at 0.064 s consumed-frame spacing; the shortfall is decode-success
at the braked arm's longer ranges, a physics consequence, recorded as written). **C3
RED, and diagnostically so:** the package is confirmed ON in all 8 logs, yet the
median closing speed over the last second before CPA is UNCHANGED vs re-fly #2 (2.84
vs 2.84 m/s, same instrument both campaigns) — the brake's isim effect (2.9 → 1.5)
did not materialize in Gazebo.

**Registered verdict: NULL** (median improvement 4% < 40%). The distribution SPLIT
rather than shifted: the brake arm produced both the best flights ever flown here
(0.125/0.313/0.537 vs #2's best 0.545/0.656/0.740) and worse tails (1.89/2.25).
Direction-only language applies; no adoption of anything from this result.

**Why the brake did not act (measured, not guessed — the next attributed suspect):**
the cap computes its stopping envelope from the KF state, and the KF's range error,
−0.065 m overall, collapses to **−2.58 m median over the last 2 s** (last consumed
det 0.02–0.59 s before CPA; pose bias at capture stays ~0) — endgame COAST/
extrapolation error, exactly the regime the cap must read to brake. isim does not
reproduce this signature (its registered declared residual: measurement-noise model
under-carries the live PnP spread ~1.6x; plus the delay/tau split). Next step per
discipline: a registered ENDGAME-ESTIMATOR diagnosis (isim-vs-Gazebo last-2s KF error
under matched conditions), NOT tuning, NOT another Gazebo arm.
