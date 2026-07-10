# MORNING REPORT — overnight autonomous session, 2026-07-10

*One doc, everything that happened. Sources: the 3 overnight audits
(`docs/audit_deep_2026-07-10.md`, `docs/audit_forward_2026-07-10.md`,
`docs/audit_pipeline_frames_2026-07-10.md`), ADR-0058..0061 (`docs/decisions.md`),
git log, NEXT.md. Every number below traces to one of those.*

---

## 1. TL;DR

- **The honesty spine held across all 3 deep audits**: zero ground-truth leaks in any live guidance/seeker path, the post-latch cue guards are airtight, and every headline number independently recomputed matched its CSV exactly. The problems found are packaging, durability, and edge-cases — not substance.
- **v3 seeker retrain = honest NULL** (regresses flight recall; v2 stays deployed) — and the forward audit **caught a false-win defect in the scorer before any result existed**. The night's best catch.
- **The comms-denied jam MC flew (8 arms)**: fail-closed **demonstrated** with a clean dose-response witness (12→2→0), and the ADR-0059 fix validated as **FAIL-SAFE (anti-fooled) but NOT recovery** — "works comms-denied" **stays HELD**; recovery = fix + coast-search (#39).
- **6 commits landed** (design review, portfolio surface, doc honesty, evidence CSVs + a new `check_t21.sh` gate, pre-registration scripts, 16 per-tick CSVs). The **jam-fix code + ADR-0059/0060/0061 + the 8 jam CSVs are deliberately uncommitted — held for your call** (decision A below).
- **Option B ratified** (the markerless NN flies for real, Pi 5 + Hailo, bench-gated) via the flight-compute council.
- **Three decisions need you** — section 2, read that first.

---

## 2. ★ DECISIONS THAT NEED EMERSON (read this first)

### A. Commit the jam-fix workstream now, or hold it?
The ADR-0059 staleness fix is built, 86 tests green, and now **MC-validated as fail-safe** (it eliminates being *fooled* into phantom-handoffs onto a dead cue) — but it does **not** restore the jammed intercept (net recovery ~0–1 flight, within noise). Everything is sitting uncommitted, held for you: `m4_intercept.py` + `detect_track.py` (comment) + `tests/test_cue_staleness.py` + `scripts/mc_jam_arm.sh`, ADR-0059/0060/0061 in `decisions.md`, and the 8 jam-arm CSVs (`logs/mc_adr0059_*.csv`).

- **Option A1 — commit now** as a validated fail-safe, default-off safety option. For: the fail-safe property is real and pre-registered; the fresh-cue path is guidance-identical (control arms confirm within Gazebo noise); several *already-committed* docs cite ADR-0059/0060, which don't exist at HEAD (dangling references, flagged by audit 3); holding also traps ADR-0061, which has no jam dependency.
- **Option A2 — hold** until fix + `--coast-search` demonstrates true camera-only recovery (#39), so the commit tells one complete story.
- Middle path (audit 3's suggestion): split the hold — commit ADR-0060/0061 + the ADR-0059 finding/results now, keep only the follow-up open.

### B. Order the Stage-0 cart (~$257) + Hailo — task #34, builder action
Option B is ratified (flight-compute council): the markerless NN flies for real on a **Pi 5 + Hailo NPU**, gated on bench measurements. Nothing hardware-side moves until you order: Pi 5 8GB (~$130 official-reseller pricing, not marketplace), global-shutter cam + wide lens, cooler/PSU/SD, + the $15 yaw-ramp jig the design review moved into this cart. Cart total ~$257 (`docs/stage0_bench_plan.md`, design review §6).

### C. Confirm portfolio priority
The overnight efficiency steer was blunt: **stop adding hardening, make the existing results VISIBLE, ship the T25 video.** Your precondition ("fix markerless maneuvering before the video") is now met — ADR-0058's detect-then-track holds a 14/14 camera-only maneuvering terminal. Confirm: T25 video next on the sim queue (vs. #39 recovery arm or more stats batches first)?

---

## 3. Flagship results

### (i) v3 retrain — honest NULL, and a false win prevented (ADR-0061, task #28 CLOSED)
- **v3 does NOT ship.** Held-out flight recall REGRESSED: maneuver 0.473 → 0.189, line 0.588 → 0.176, worse on **all 10 held-out flights** (paired sign p=0.002) — while acing its own static-grid val split (3 m 0.79→1.00, 18 m 0.96→0.97). Static far-range data did not transfer to the flight far-range problem. Lower phantom count is underpowered and a side-effect of v3 firing 5–20× less.
- **The catch:** the forward audit inspected the verdict script *before results existed* and found it would have declared a **WIN on ~3 correlated noise frames** (a 29-frame bucket, point rule, no CI). Four scorer fixes were pre-registered and committed before scoring (f6427af); with them, flight recall stayed primary and the regression was caught. Without them, the grid probe would have looked like a v3 win.
- **Markerless maneuvering still works** — via detect-then-track + v2 (ADR-0058: 14/14 post-handoff camera-terminal at 12 m/s weave, zero phantom handoffs, 0/155 false terminal dets). v3 was incremental hardening, not load-bearing; `drone_finetuned_v2.onnx` stays deployed.

### (ii) Comms-denied jam MC — ADR-0059 CLOSED (task #31 DONE)
8 arms, n=16 paired flights each (master-seed 42, weave, speed-12/gate-8 only, idle load). Link cutoff fired 16/16 in every jam arm.

| Cutoff range | fixoff REAL-ish handoffs | fixon REAL-ish | Read |
|---|---|---|---|
| 15 m | 12/16 (Pk 9/14, med 2.19 m) | 3/16 | Fail-closed doesn't bite yet; the fix has a **premature age-out cost** here (handoffs 14→10) |
| 18 m | 2/16 (post-gt med 14.0 m) | 2/16 | Clean fail-closed; fixoff "handoffs" are FOOLED phantoms onto the ~14 m stale cue (9→1 under the fix) |
| 22 m | 0/16 (post-gt med 19.5 m) | 1/16 | Strong fail-closed; fix is fail-safe, phantom fully eliminated (5→0) |

- **The monotonic witness collapse — fixoff REAL-ish 12 → 2 → 0 across 15/18/22 m — is a clean dose-response demonstration of the fail-closed bug**, worse as the jam moves pre-acquisition.
- **The fix is a failure-mode FLIP, not a recovery**: net recovery is ~0 @18 m and +1 @22 m (single-flight counts, within noise). But it converts the worst failure — flying at a 14–19 m ghost — into a clean abort. And when fixon did reach a camera-only handoff it **hit well** (3/3, median 0.70 m @18 m): the camera-only terminal works when it engages; without `--coast-search` it rarely gets there.
- **The honest headline, verbatim from ADR-0059's CLOSE** (what is EARNED vs HELD):
  > "under a pre-acquisition cue jam the deployment config fails closed — fooled into phantom-handoffs onto the stale cue, 0/16 real intercepts @22 m; the ADR-0059 staleness fix converts this to FAIL-SAFE (ages out the dead cue, aborts cleanly, phantom eliminated at 22 m) — a validated anti-fooled SAFETY property. Full camera-only RECOVERY = fix + `--coast-search` (ADR-0015's WORST-tier dead-reckon-coast), the documented next arm (task #39), plus a `--cue-stale-horizon` tradeoff sweep (the 1.0 s horizon costs handoffs at shallow cutoffs — the 15 m premature-age-out)."
- **"Works comms-denied" stays HELD** as an intercept claim. Caveats before quoting any number: recovery counts are 1–2 flights (noise); no coast-search was flown so the recovery number is a *lower bound*; scoped to speed-12/gate-8 only; the PHANTOM label conflates range-noise labels on good intercepts with true stale-cue ghosts.

---

## 4. The three audits — findings, what got fixed, what's open

**Audits:** (1) deep codebase audit — 5 dimensions, verifier-adjudicated; (2) forward audit — v3-eval + jam-MC methodology, claims, queue; (3) pipeline/frames audit — stereo T16–T19, fusion/EKF, coordinate frames, commit soundness.

**The spine is sound (all 3 agree):** no active gt leak in any live path; the honesty tests are real (AST-based, mutation-calibrated, not theater); the post-latch cue guards are airtight; every recomputed headline number byte-matched its CSV (mc_final_all n=96, the weave12 14/14, M2/M3/M4); all overnight commits independently reproduced sound.

**Fixed and committed overnight:**
- **Doc honesty pass** (7dad15e): "works comms-denied" now HELD *everywhere* (interviewer_prep led with the unheld claim twice; WRITEUP was two arcs stale, still calling the markerless seeker "not simulated"); the **r²-misread** fixed across 4 surfaces (r²=0.96 is variance-explained by handoff ZEM, not "96% of the miss" — correction capacity says ~75% locked in / perfect seeker cuts ~25%).
- **Evidence-base gate** (2665bbb + 81562b8): the headline CSVs were 100% gitignored — the 14/14 culmination number had no gate and would not survive a disk loss. Now: aggregate CSVs + all 16 per-tick weave12_r2 flight CSVs committed, plus a new `scripts/check_t21.sh` that re-asserts the ADR-0058 numbers from the committed data.
- **Pre-registration scripts** (f6427af): the 4 v3 scorer fixes + `scripts/check_jam_mc.py` (jam verdict thresholds written *before* any jam arm flew) + `phase4_eval_v3.sh` — committed with the train-daemon timestamps as attestation, closing the integrity window audit 3 flagged.

**Open findings, now tasks:**
- **#37 — range-increase breakoff false-trigger** (confirmed, medium→high): `BREAKOFF_RANGE_INCREASES` has no noise deadband or trend gate; pixel-quantized range jitter or a frame-edge clip can fake 3 monotone rises and trigger an early breakoff — and it survives even with the ADR-0056 gates on. Fix designed (deadband + border-box reject); sequenced behind decision A (it edits held files).
- **#38 — roll/pitch never derotated from the LOS bearing** (structural): bearing is yaw-compensated only (`lambda = psi + beta`); at ADR-0060's measured 27–36° dash pitch that's a 12–24% azimuth gain error plus roll cross-coupling — exactly in the maneuvering/dash regime, and it matters more under Option B. The `--bench` "proof" only ever spun level. Fix: quaternion derotation of the camera ray (honesty-legal own-state), then paired A/B.
- **#39 — comms-denied recovery**: fix + `--coast-search` arm + horizon tradeoff sweep. The headline follow-up from ADR-0059.
- **Notes, not yet tasks:** the T19 "cue tracks 0.99 m median" metric conflates triangulation error, latency, and a demonstrably nonzero station/mover clock-epoch skew (latency alone predicts 1.3–2.2 m — skew partially cancels it); decompose before quoting it as accuracy. And the hero-demo CSV (0.632 m, cited plainly at README:727) is absent from disk and git — recover from backup or caveat it.

---

## 5. Decisions ratified overnight

- **Option B — the markerless NN flies for real** (flight-compute council, 3 members): flight compute = **Pi 5 + Hailo NPU**, bench-gated (nothing claimed until Stage-0 measures detection Hz / latency on real hardware). Feeds task #34 (cart + the flight-compute ADR). This resolves the review-flagged contradiction of benching on a Pi 5 while planning to fly a Pi Zero 2 W.
- **v2 stays the deployed detector** (ADR-0061 NULL — see §3i).

---

## 6. Commits landed overnight (6)

| Commit | What |
|---|---|
| 309c24e | Design review + sim-to-real hardening deliverable, v3 retrain tooling, desk-experiment probes (blur, dash-pitch) |
| 1a14869 | Portfolio surface: README headline arc, honest comms-denied HELD box, T25 storyboard |
| 7dad15e | Portfolio doc honesty: HELD-everywhere + r²-misread + stale-numbers fixes (deep-audit H3/H4/H5) |
| 2665bbb | Evidence CSVs committed + new `check_t21.sh` gate for the 14/14 headline (deep-audit H1) |
| f6427af | Pre-registration integrity: v3 scorer fixes + `check_jam_mc.py` committed before results (audit-3 H3) |
| 81562b8 | The 16 per-tick weave12_r2 flight CSVs — fresh-clone reproducibility for the headline (audit-3 H4) |

Held uncommitted (your call, §2A): jam-fix code + tests + harness, ADR-0059/0060/0061, 8 jam CSVs, the 3 audit docs, misc new scripts.

---

## 7. Task queue and suggested priorities

| # | Task | Status / owner |
|---|---|---|
| 30 | $0 desk trio | blur DONE, camera-pitch DONE (ADR-0060); **real-footage eval PENDING — needs your approval** (public datasets are multi-GB, or phone video from you) |
| 31 | Comms-denied jam MC | **DONE** (ADR-0059 CLOSE, this report §3ii) |
| 32 | r_hat campaign | CUT to 2 probes — sim-gated |
| 33 | Stats hardening | jink n=16 post-fix is the top item; note a real "95% Pk" claim needs **n=72 clean** (n=48 only buys a 92.6% lower bound) — sizing decision before data |
| 34 | Stage-0 cart + flight-compute ADR | **BUILDER** — decision B |
| 35 | Up-tilt camera mount A/B | sim-gated (ADR-0060 remedy; conditions later campaigns, keep it early) |
| 36 | Audit findings tracker | sim-free — fold the §4 open items into one tracked list |
| 37 | Breakoff false-trigger fix | code, sequenced behind decision A (held files) |
| 38 | Roll/pitch bearing derotation | sim-gated (attitude fix + bench extension + paired A/B) |
| 39 | Comms-denied recovery (fix + coast-search) | sim-gated — the ADR-0059 follow-up |

**Suggested order:** (1) decision A — it unblocks #37/#38 edits and cleans the dangling ADR references; (2) per the efficiency steer ("make results VISIBLE, ship the video"), **T25 video** next on the sim — the maneuvering precondition is met; (3) then #39 and #33 (jink n=16); sim-free in parallel: #36 tracker, T19 skew decomposition, real-footage harness scaffold pending your approval.

---

## 8. Resume here

- **This report:** `docs/overnight_report_2026-07-10.md`
- **Live state + queue:** `NEXT.md` (the "OVERNIGHT OUTCOME" block near the top points back here)
- **The jam result in full:** ADR-0059 ★RESULTS/CLOSE, `docs/decisions.md` (~line 1369); verdict scripts `scripts/check_jam_mc.py`, arms `scripts/mc_jam_arm.sh`
- **The v3 null in full:** ADR-0061, `docs/decisions.md` (~line 1390); eval `logs/seeker_v3_eval/run_20260710T083822Z/`
- **The three audits:** `docs/audit_deep_2026-07-10.md`, `docs/audit_forward_2026-07-10.md`, `docs/audit_pipeline_frames_2026-07-10.md`
- **Gates you can run right now:** `scripts/check_t21.sh` (the 14/14 headline, from committed CSVs), `scripts/check_m5.sh`
