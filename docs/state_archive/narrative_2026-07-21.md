# Retired `narrative` prose blocks — archived verbatim 2026-07-26 (tracker redesign)

The contract's `narrative` shrank to `{as_of, program, rulings}`; the five prose blocks
below (`lede`, `reframe`, `big_lever`, `caveat`, `levers`) were removed from
`docs/project_state.json` and are preserved here EXACTLY as last committed
(`narrative.as_of` at removal: 2026-07-24; the blocks date from the 2026-07-21
narrative-first redesign, last touched 2026-07-24/25). Each block is the verbatim JSON
value, so structure (stats tiles, the ZEM meter, the levers table) survives exactly.

## narrative.lede (verbatim JSON)

```json
"Read this first — the current shape of the project in four findings: the REFRAME (what the wall actually is), the one BIGGEST LEVER, the honest CAVEAT that bounds every claim, and the builder's LIVE RULINGS. The 4-phase program in §0.2 follows from them; everything below §0 is the detail. Every number traces to a run or a derivation (provenance on each tile)."
```

## narrative.reframe (verbatim JSON)

```json
{
 "headline": "The intercept is POINTING-limited, not sensor-limited.",
 "text": "To dash forward the quad pitches ~35-40° nose-DOWN, so the fixed camera stares at the ground and a co-altitude target rides the top edge of the frame — or leaves it entirely. The detector was never the problem: the same network reads ~100% with the camera static at 8-22 m but 0.8% in flight, and in the decisive 8-12 m band ~75% of terminal ticks have the target OUT of frame altogether (when it IS in frame, the deployed detector hits 70%). The target simply is not in the picture — which is why every sensor-side lever (retrains, resolution/crop, color) returned NULL, and why the fix is geometry, not weights.",
 "stats": [
  {
   "label": "in-flight approach recall",
   "value": "0.8%",
   "note": "2 real vs 1187 phantom detections across 63 flights",
   "tone": "bad",
   "stamp": "THE WALL",
   "provenance": "ADR-0076 add #18i/#18k · approach_recall.py"
  },
  {
   "label": "same detector, camera static",
   "value": "~100%",
   "note": "8-22 m, both aspects (set-pose sweep) — the sensor is fine",
   "tone": "good",
   "stamp": "NOT THE SENSOR",
   "provenance": "ADR-0076 add #18k · set-pose sweep 2026-07-16"
  },
  {
   "label": "target OUT of frame, 8-12 m band",
   "value": "75%",
   "note": "of terminal ticks under the dash pitch; when in view, detection is 70%",
   "tone": "bad",
   "stamp": "THE MECHANISM",
   "provenance": "docs/inview_probe_results.md (probes flown 2026-07-21)"
  },
  {
   "label": "nose-down dash pitch",
   "value": "35-40°",
   "note": "full-accel dash attitude — the geometry that points the camera at the ground",
   "tone": "warn",
   "stamp": "THE GEOMETRY",
   "provenance": "ADR-0060 · docs/intercept_accuracy_levers.md"
  }
 ],
 "evidence": [
  "docs/inview_probe_results.md",
  "ADR-0076 add #18k",
  "docs/intercept_accuracy_levers.md"
 ]
}
```

## narrative.big_lever (verbatim JSON)

```json
{
 "headline": "SETTLED: correct AIM is the dominant lever (open-loop 0.71 → 0.43 m, ADR-0083) — and the camera does NOT defend imperfect aim: it lost to its own control at 0°, 5° AND 15° aim error. Spend on aim + real-data detection.",
 "text": "The 4-arm decomposition (A/Adash bias30, G20/G20dash bias20) isolated what the camera terminal does. (1) AIM is the dominant, free lever: correcting the crossing-bias +30°→+20° (the collision-lead was mis-sized — it assumes constant speed, the dash accelerates from rest) drops the OPEN-LOOP dash-only combined median 1.37→0.75 m (7/8, G20dash CONFIRMED). (2) The CAMERA is an AIM-ERROR CORRECTOR: on a badly-aimed l2r dash it recovers 2.05→0.68 m (+1.36), but on a well-aimed dash it slightly hurts (aspect bias has nothing to fix), and on r2l it hurts at BOTH biases because the ADR-0056 aspect bias there is large + opposite-signed. So the camera's real value is defending IMPERFECT aim (the real-world case the sim's perfect-aim dash understates), GATED by the aspect bias. REFINED 2026-07-24 (compensation built + validated toward): the 'aspect bias' is largely a ~190 ms LOS-RATE LAG (direction-agnostic --terminal-los-lag-ms knob), and the stable direction-keyed constant is regime-confounded (only ARM B's accel-capped slow approach gives it; the fast uncapped approach is lag-dominated). FRAMING (loft+wedge) HURTS at correct aim (CG 1.92 > level 1.32). So the honest lever stack is: (1) CORRECT AIM is the dominant, free lever (accel-aware collision-lead, ADR-0080/0083) → 0.75 m dash-only. (2) The CAMERA does NOT defend imperfect aim — REFUTED in sim: it lost to its own dash-only control at 0°, 5° AND 15° aim error (see the levers table). (3) The terminal-precision compensations are NULLs — the LOS-lag knob flew (GL) at 1.44 vs the level 1.32 m, and the direction-keyed bias constant is regime-confounded. Keep the terminal SIMPLE and level; the framing levers (loft+wedge) are net-negative at 9 m/s. ⚠️ 2026-07-25 CAVEAT: the coded-dash CAMERA-ARM verdicts above (the '(2) camera adds nothing / can't defend aim' finding) were measured while the terminal's frozen_vworld latch commanded ZERO horizontal velocity (bug fixed 2026-07-25; the auditor now catches it). The AIM conclusion (open-loop dash-only) is unaffected and stands, but the camera-arm numbers are DOWNGRADED to PENDING RE-FLY until re-flown with the steering terminal. The remaining real lever is real-data OUTDOOR detection (a field task).",
 "evidence": [
  "scripts/experiments/loft_dive/inframe_ab.py + inframe_ab_results.csv (80 rows, 2026-07-21)",
  "docs/intercept_accuracy_levers.md (Execution results 2026-07-21)",
  "decision intercept_accuracy (pointing stage) · contradiction wedge-sizing-vs-accel-cap"
 ]
}
```

## narrative.caveat (verbatim JSON)

```json
{
 "headline": "9 m/s is ZEM-hard — no terminal-only trick makes it sub-meter.",
 "text": "At a 9 m/s crossing, the delivered zero-effort-miss (ZEM) at handoff is ~4 m against ~0.27 m of terminal correction capacity from today's geometry — a perfect collision course exists at handoff; the interceptor just doesn't fly it. Correction capacity is ½·a·t_go², growing with time-to-go SQUARED, so the win comes from getting the target IN FRAME during the dash and onto a converged collision course EARLIER, with more t_go: measured, pushing handoff from ~6.5 m out to ~12 m lifts capacity 0.72 m → ~4.3 m, past the ~4 m bar. Earlier framing IS the win; terminal polish (Phases C/D) only harvests it.",
 "meter": {
  "unit": "m",
  "max": 4.6,
  "reference": {
   "value": 4.0,
   "label": "delivered ZEM at a 9 m/s handoff (~4 m) — the correction the terminal must be able to make"
  },
  "bars": [
   {
    "label": "correction capacity, today's geometry",
    "value": 0.27
   },
   {
    "label": "capacity with handoff pushed ~6.5 m → ~12 m out",
    "value": 4.3
   }
  ],
  "provenance": "ADR-0027 (ZEM r²=0.99; capacity = ½·a·t_go²; measured 0.72 m → ~4.3 m)"
 },
 "evidence": [
  "ADR-0027",
  "docs/intercept_accuracy_levers.md (honest gates)"
 ]
}
```

## narrative.levers (verbatim JSON)

```json
{
 "headline": "SETTLED levers (2026-07-24 decomposition) — what actually shrinks the miss, and what was tried and found net-negative",
 "rows": [
  {
   "lever": "Correct the AIM (accel-aware collision-lead)",
   "category": "kinematics / the dominant lever",
   "what": "THE PROVEN LEVER. The collision-lead solved a CONSTANT-SPEED triangle while the dash ACCELERATES from rest, so the aim was mis-sized. Correcting it took the open-loop dash-only miss 1.37 → 0.75 m, then ADR-0083's measured +5° trim to ~0.43 m pooled (r2l 0.33 m = inside the ~0.35 m ram envelope (ADR-0084)), and the accel-aware lead (ADR-0080) now DERIVES the right aim automatically (+21.3° ≈ the confirmed +20°) with no per-config tuning. Free, honesty-clean (pre-flight constants), carries to hardware.",
   "test": "DONE + VALIDATED: G20dash CONFIRMED (7/8 paired, median 0.75); AAL reproduces hand-tuned on seed 123 (0.71) and GENERALIZES on disjoint seed 777 (0.57). flight_plan_candidates.md RESULTS."
  },
  {
   "lever": "The camera as an AIM-ERROR corrector — REFUTED in sim",
   "category": "terminal — TESTED, NULL at every aim error",
   "what": "⛔ The hypothesis was that the camera earns its handoff once the open-loop aim is imperfect (as it always is in the real world). TESTED at 0°, 5° and 15° deliberate heading error, each against its OWN dash-only twin, paired n=8: the camera lost at every cell (tighter on 2/8, 0/8, 3/8; the pre-registered bar was ≥6/8). Binding cause = the ADR-0056 r2l aspect bias: on l2r the camera DOES start to help at 15° (3/4, once −0.43 m), but r2l costs +2.1 m twice and drags the pooled result to null. SCOPE: this indicts the SIM's markerless seeker on one aspect, NOT the camera-terminal architecture — first kills fly the AprilTag, and real-data detection (44.2% vs 1.1% recall on real imagery) may reopen it.",
   "test": "DONE: AE5/AE15 + twins, seed 123 (docs/flight_plan_candidates.md). The sweep is the pre-registered harness to re-ask the question once a real-data detector exists."
  },
  {
   "lever": "Accel-capped dash + loft + wedge (FRAMING)",
   "category": "pointing — TRIED, NET-NEGATIVE at 9 m/s",
   "what": "⛔ NOT a do-first lever. It genuinely fixes POINTING (8-12 m real recall 3.0% → 35.4%, target centred) but the cap costs ~0.34 m of closing speed even at CORRECT aim (EAL 1.05 vs uncapped 0.71), and framing HURTS at correct aim (CG framed 1.92 vs level 1.32). ARM B's original 2.01 m regression = mis-aimed (~1.3 m) + real cap cost (~0.34 m). Keep the dash LEVEL and uncapped; the wedge's own share is still unpriced (see the wedge-vs-level ledger entry).",
   "test": "DONE: Bdash/EAL/CG decomposition, all paired n=8 with dash-only twins."
  },
  {
   "lever": "Terminal-precision compensation (bias constant / LOS lag)",
   "category": "terminal — TRIED, NULL",
   "what": "⛔ BOTH NULL. A direction-keyed aspect-bias constant is regime-confounded (stable only under the accel-cap that itself hurts; unstable elsewhere, LOO +1%). A 190 ms LOS-rate-lag correction shifts error between directions and fails its gate (GL 1.44 vs 1.32, 5/8). The ADR-0056 terminal-bearing wall does not yield to a simple constant or lag — consistent with the ADR-0071 subpixel null.",
   "test": "DONE: GL flown; measure_aspect_bias.py --loo reads a null offline for free before spending an arm."
  },
  {
   "lever": "Real-data outdoor detection",
   "category": "perception — THE REMAINING LEVER",
   "what": "The acquisition wall is OUTDOOR APPEARANCE, and it is a HARDWARE/FIELD task, not a sim one. Measured on real held-out imagery: the deployed sim-trained model scores 1.1% recall / 88.5% false-fire vs the real-data mono model's 44.2% / 4.9%. This is where the remaining intercept improvement lives.",
   "test": "The tripod day's two curves (gated on the measurement layer being fixed — see the build_plan), then capture → retrain validated on held-out FLIGHTS."
  }
 ],
 "note": "Hardware escalations (2-axis gimbal, dual onboard cameras, event camera, motor cant) are gated BEHIND Phase-A validation; research dark-horses (track-before-detect, FOV-constrained RL guidance, bank-as-accel APN, IMU-deblur) are spikes, not commitments. Three ideas were vetted straight into the graveyard (porpoise pulses, impact-angle ram, observability weave) — §6.0.",
 "evidence": [
  "docs/intercept_accuracy_levers.md (DO FIRST table + hardware bets + dark horses)"
 ]
}
```
