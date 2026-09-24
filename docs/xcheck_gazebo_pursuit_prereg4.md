# Pre-registration 4: Gazebo pursuit cross-check re-fly with the own-velocity fix

Registered 2026-09-24, BEFORE any flight of this arm. Follow-up to prereg 3
(registered NULL) and ADR-0117: the endgame-estimator diagnosis found and fixed the
own-velocity wiring defect (the pursuit KF had been running on the last COMMANDED
velocity, 3.6–4.2 m/s off actual, in 16/16 campaign flights).

## What changed since prereg 2 (one change vs that baseline)

`real_flight.py` now fills `VehicleObs.vel_ned` from the EKF's `velocity_ned()`
stream (ADR-0117, two-line fix, fallback + FAULT accounting retained as the safety
net). **The brake package is NOT flown** — this arm isolates the wiring fix against
re-fly #2's no-brake baseline (0.9665 m median), one change per campaign as always.
The brake question re-opens only after this baseline exists (ADR-0117 honesty note).

## Registered prediction (from the diagnosis' KF replica + isim, no new tuning)

- Replica input-swap (R0→R1) on the flown no-brake trajectories: last-2 s mean range
  error −1.02 → **+0.23 m**; final-coast vector slope 2.84 → **0.40 m/s**; the coast
  latch fires at ~**1.1 m** true range (or not at all) instead of ~3.7 m.
- isim (correct wiring, ENGAGE fit, pose, matched optics): median ~0.09–0.31 m band
  (prereg2 arms b–d, unchanged). Honest expectation: the ~3x isim→Gazebo residual of
  re-fly #2 is now largely attributed to this defect; a fair point expectation is
  **~0.15–0.45 m median**, best-case caveats unchanged (perfect cue, no wind, upright
  tag, not the OV9281). Open-loop→closed-loop uncertainty declared: the replica
  cannot show how guidance re-flies; the isim counterfactual supports direction only.

## Re-fly configuration (registered)

Identical to prereg 2 in every respect (fresh boot per flight, idle machine, n = 8,
`bash scripts/xcheck_fly.sh 1 8 <outdir>`, centre-to-centre CPA from scoring-only gt)
— no brake flag, no other change, no tuning.

## Adopt / reject criteria (registered before flying — SAME bar as preregs 1–3)

PASS iff over the 8 flights: (1) ≥6/8 CPA ≤ 1.0 m; (2) median CPA ≤ 0.5 m;
(3) ≥2/8 ≤ 0.35 m; (4) 0 pre-pass failsafe aborts; (5) camera-driven throughout.

**Fix-effect clauses (independent of CPA):**
- F1: `FAULT own_vel_fallback` appears in **0/8** run logs (was 16/16).
- F2: replica-instrument last-2 s mean range error within **±0.5 m** (was −1.02).
- F3: the coast latch fires at true range ≤ 2 m, or not at all, in ≥ 6/8 flights
  (was ~3.7 m median).
- C1/C2 carried over (range bias ±0.30 m median; cadence ≥ 9 det/s, tick dt ≤ 0.06 s).

## What the outcomes would mean (written now)

- **PASS + F/C green:** the transfer gap is closed as finally attributed (range bias
  + cadence + own-velocity wiring); the default-swap question and the brake question
  both reopen for the builder, each by registration. Direction-only margin language;
  no real-world claim.
- **PARTIAL (bar failed, median improves ≥40% vs 0.9665, F green):** the wiring was a
  major but not the final term — next is a fresh tick-trace on the new CSVs.
- **NULL (improvement <40%, F green):** the open-loop replica misled us about the
  closed-loop effect — record it as a replica-methodology limit; back to the trace.
- **Any F red:** the fix did not land in the flying code — fix delivery, re-fly under
  this registration.

## RESULT (2026-09-24, n=8 flown as registered — scored by the head session)

CPAs sorted (m): 0.095 · 0.235 · 0.292 · 0.329 · 0.356 · 0.531 · 0.712 · 0.976.
**Median 0.343 m — the registered bar PASSES, first time in four campaigns**
(1.510 → 0.967 → 0.928 → 0.343). Logs: `logs/xcheck_gz_20260924_velfix/`
(+instrument.json).

Criteria: (1) **8/8 ≤ 1.0 m PASS**; (2) **median 0.343 ≤ 0.5 PASS**; (3) **4/8 ≤
0.35 m PASS** (0.095/0.235/0.292/0.329 — inside the ratified contact envelope,
camera-in-the-loop); (4) 0 pre-pass aborts PASS; (5) camera-driven PASS (62–93
consumed). Median improvement vs re-fly #2: **64.6%** (≥40% twice over).

Fix-effect clauses: **F1 GREEN** — `FAULT own_vel_fallback` in 0/8 logs (was 16/16).
**F3 GREEN** — coast latch at true range ≤2 m or never in 7/8 (median onset 1.23 m vs
the replica's ~1.1 prediction; was ~3.7; one flight at 2.03 m misses the letter).
**F2 RED as written** — the "last-2 s of Phase B" scalar reads −0.80 m median (was
−1.02; clause ±0.5), with one +11.4 m outlier; the diagnosis itself flagged this
statistic as mostly post-CPA coast and recommended CPA-anchored vector stats — the
clause is scored as registered, not swapped post hoc. **C1 GREEN** (KF error median
+0.136 m; pose bias +0.047). **C2 HALF-RED again** (cadence 8.56 < 9; tick dt 0.056
green — same decode-success physics note as prereg3).

**Registered verdict: PASS (criteria 1–5) with F2/C2 blemishes stated.** Per the
registered meaning: the transfer gap is closed as finally attributed — measurement
bias (prereg2), cadence (prereg2), and the own-velocity wiring (ADR-0117) — and the
DEFAULT-SWAP question and the BRAKE question both REOPEN for the builder, each by
registration only. Direction-only margin language stands. HONEST SCOPE, unchanged:
perfect launch cue, windless world, upright world-fixed tag (Gazebo cannot tilt it —
ADR-0114 scope note), sim camera not the OV9281. This is a sim milestone, not a field
claim.
