# FORWARD REPORT — 2026-07-10 overnight (lens synthesis, adversarial verdicts applied)

All findings below survived adversarial re-verification against the code/logs. Dropped: 1 item falsified (flight-compute ADR — already owned by the running council; the lens read a stale NEXT.md snapshot), 3 items overstated and cut down to their residuals (T25 sequencing, #35 demotion, fixon/fixoff contamination). Verifier "missed" items are folded in and marked **[fold]**.

---

## 1. Methodology go/no-go — tonight's validations

### v3 eval: **NO-GO as scored.** Patch `scripts/eval_seeker_v3.py` before the train exports (~30 min out). All fixes are offline script edits; no race with the sim.

The stats library (Wilson CI, sign test, pairing, G1 underpowered guard) is verified correct. The **verdict() composition** is not — three defects, all confirmed against the real eval pool:

| # | Defect | Evidence | Fix (pre-register before results exist) |
|---|---|---|---|
| 1 | **WIN on a 3-frame noise delta.** G2 clause_b is a point rule (`:683-684`), no CI/test; the maneuver∩15-30 m pool is exactly **29 frames** (recounted from labels), so +0.10 = 3 correlated frames. v2 2/29 vs v3 5/29 → WIN with Wilson CIs [.019,.220] vs [.076,.345]. The low-n caveat needs n<20 (`:842`) — dead at n=29; the printed sign tests are never consulted by verdict() (`:1136-1146`). | eval_seeker_v3.py:679-689, 841-872 | clause_b claims improvement only if bucket CIs are disjoint OR a per-flight paired sign test on the bucket is significant; else PASSED-BUT-WITHIN-NOISE → NULL branch. |
| 2 | **G3 tolerance below pool granularity, and abs() fails improvements.** Line pool = 34 frames (14+20, verified); min nonzero delta 1/34=0.029 > 0.02, so ANY single-frame difference — including v3 matching one MORE — fails no-regression and blocks WIN. v2 line-phantom=0 forces v3==0 across 454 frames; one noise det kills it. | :705-731 | Score G3 recall as "within ±1 matched frame" (or CI overlap); drop abs() for the upside. |
| 3 | **[fold] WIN ignores clause_a — a maneuver-recall COLLAPSE can't block it**, and with G1 PASS + G2 fail the report prints NULL_BRANCH_TEXT and "WIN: True" in the same output (contradictory verdict text). v3 could drop pooled maneuver recall 0.60→0.30 and still WIN on 3 bucket frames. | :841, :872, :1214-1215 | `win` must also require clause_a; make the null-branch print condition consistent with win. |

**At-results-time checklist** (hand-apply if not scripted):
- **Span footgun is about to go live**: `weights/` has **no** `drone_finetuned_v3.onnx` and no v3 calib sidecar yet; post-export, `load_span` silently falls back to span=1.0 → v3's M4 range ~8.5% off vs v2's 0.9216 (v3_onnx_infer.py:84-97). Confirm the printed span source is the sidecar; confirm `MARKERLESS_SPAN_M` unset (it would override BOTH models).
- **[fold] no-new-phantom-class is one-sided**: it blocks a phantom increase only with DISJOINT CIs (`:800`) — a ~2.3× increase (8→18/1414) still wins. The flattery asymmetry is two-sided: no significance demanded for the recall gain, significance demanded to register a phantom loss. State this in ADR-0061.
- Cross-tab match/mislock double-counts (center<25 px OR-clause has no size guard — one oversized det can earn recall credit AND count as the failure, :187-223).
- Eyeball evalv3_07/08 (weave-9, 476 frames) — they sit in zero gates; a regression there slips scripted adoption.
- **[fold] ADR language**: (a) grid 3/18 m "held-out" is **v3's own validation split** — split_meta.csv assigns all 144 grid r=3/18 frames to split=val (checkpoint selection + span recal optimized on them); v2 is zero-shot there. Never phrase it as symmetric generalization. (b) Headline claim = "far-range recall improved without regression"; mis-lock counts+CIs on their own line (WIN doesn't require any mis-lock reduction). (c) Disclose Phase-0→reframe sequencing precisely: forensics ran on evalv3_01/02 — **two of the six gate flights**, not merely eval-adjacent data. (d) Results are conditional on these captured renders (one draw of ADR-0057 nondeterminism). (e) Sign test at n=10 flights is near-unpowerable (only 9-1/10-0 reach p<.05) — one sentence, don't lean on it.
- Redirect `--out` — default `logs/seeker_v3_eval` already holds stale forensics artifacts.

### jam MC: **GO to fly, NO-GO to conclude** without a pre-registered verdict script. Plumbing verified sound (env passthrough byte-identical to r2's echo; master-seed-42 pairing deterministic; fixoff is a faithful bug reproduction; Pk(iii) correctly excludes r2's two no-handoff pooled "hits").

Write `scripts/check_jam_mc.py` (exit 0/1) **before the first arm flies**, containing:

1. **Arm completeness [fold — the sharpest miss]**: assert 16 valid rows + existing flight CSVs per arm. analyze_track_ab drops NaN-miss from Pk denominators and classifies a crashed/missing-CSV flight as "never" — **conflating infra failure with the fail-closed signature the arm exists to measure**. Half of r2's rows already carry python_exit_1; jam arms will exit abnormally more often.
2. **Jam-fired tripwire**: grep each flight log for the runtime `LINK CUTOFF at t_sim=` line (not just the startup banner).
3. **Pre-registered thresholds**: jam_fixoff witness = REAL(-ish) handoffs collapse (≤6/16) with "never" ≫ control; jam_fixon = handoffs ≥11/16 (r2 baseline 14/16), PHANTOM=0, and the **headline = joint per-flight P(real handoff AND post-handoff min-gt ≤2.5) vs control_strict**. **[fold]** The joint metric is mandatory, not stylistic: flights that hand off under jam are the geometrically easy survivors, so cross-arm Pk(iii) comparisons condition on outcome-dependent subsets.
4. **Stratify by first-det-vs-cutoff**: the harness comment "solidly PRE-acquisition" is wrong — **6/16 r2 flights first-detect at ≥15 m (up to 23.04 m)**, so ~40% of pairs are jammed after the camera already sees the target (the easy case). Require recovered handoffs specifically in the detected-AFTER-cutoff stratum; fix the comment to "pre-handoff"; queue the CUTOFF_RANGE_M=18 sweep arm (env hook exists, line 92) for a genuinely pre-detection kill.
5. **control_strict equivalence**: median|Δ| < ~0.7 m + sign/Wilcoxon non-significant + per-seed handoff-outcome agreement; max|Δ| diagnostic only (P(max|Δ|<1 m over 16 pairs under the null) ≈ 3e-5 — the current "sub-~1 m" reading falsely fails an inert fix). State the ~0.5 m MDE in the verdict language.
6. **control_active**: compare arm-level **vs control_strict** (identical code, only horizon differs), never per-seed vs r2 (trajectories legitimately diverge once the fallback engages); kill the "Expect 16/16" bar (contradicts the harness's own 15/16-noise caveat); **count engagement** via pre-handoff `cue_stale=1` ticks — expected ~70-80% of flights, and if <6/16 engaged the arm is weak evidence.
7. **Difference-in-differences line**: (jam_fixon−jam_fixoff)−(control_active−control_strict) isolates the jam-specific effect. (The pre-cutoff contamination is a sub-2 s sliver — verifier downgraded it — but DiD is free.)
8. **Per-seed paired table [fold]**: `paired()` auto-triggers only on 'track'/'plain' labels — **no code path produces the fixoff-vs-fixon which-seeds-recovered table** for these labels. Add it.
9. **Attribution logging**: extract per flight the STALE-notice sim-time + `handoff_cue_rejects` (both already logged) to separate the 1.0 s staleness-window transient from real fix failure. **[fold]** Note the arms carry no `--coast-search`, so a fixon non-handoff is ambiguous between gate-fail and steering/FOV loss — annotate, don't over-conclude; a failure cluster triggers a horizon sweep (0.5 s), not fix rejection.
10. **RTF parity [fold]**: record RTF per arm — arms may fly hours apart around the v3 train and ADR-0015's load confound is the reason the DO-NOT-RUN banner exists.

Scope any ADR-0059-CLOSED verdict to speed-12 / gate-8: at low target speed the frozen cue stays inside the 8 m gate and the witness silently fails to bite.

---

## 2. Missed gaps / claim issues worth acting on (ranked)

### A. Portfolio claim integrity — fixable tonight, sim-free

1. **The comms-denied HOLD is not held everywhere** (contradicting README:156-158's own "held everywhere" claim). `interviewer_prep.md:16` leads with the un-held claim, zero ADR-0059 language; `portfolio_visuals.md` (29, 32, 89, 203) labels it unqualified; WRITEUP §1 frames it as proven. **Fix**: add the README:419 scope note (proven post-latch; pre-acquisition jam fails closed, fix in validation) or a "frozen as of 2026-07-07" banner to both; soften README's "everywhere" sentence.
2. **WRITEUP + interviewer_prep are frozen pre-ADR-0036** — they quote 27% Pk / "12 m/s uncatchable" as "the current headline" (superseded: n=96, 100%, whole band catchable) and present the markerless seeker, EKF A/B, and fusion capstone as FUTURE work (all closed, ADR-0037..0044). **[fold]** They also quote ADR-0030 recovery numbers the project formally superseded (wrong σ_R curve), and WRITEUP §5 quotes pooled Pk alone — violating the project's own ADR-0025 per-speed rule. **Fix tonight**: snapshot banner + README link caveat; full rewrite is a queued daytime task.
3. **r² conflation**: "~96% of the miss is locked in at handoff (r²≈0.96)" (bullets:40-41, prep:24, WRITEUP §5) misstates variance-explained as miss-fraction — and contradicts the ~25%-correctable figure in the same bullet. **Fix**: two defensible sentences everywhere — "handoff ZEM predicts miss, r²≈0.96" + "terminal correction capacity covers ~25% of delivered error (ADR-0023 perfect-camera counterfactual), so ~75% is locked in."
4. **jink "1/8" is traceable but unlogged** (verifier: lens headline overstated — it's the ADR-0056 plain-jink arm re-analyzed post-handoff, recorded only in NEXT.md:50, never decisions.md). **Fix**: log an ADR-0056/0058 addendum recording it, or switch all three files to the documented 3/15→14/15. Also split the scoping blur in bullets:58 AND PROGRESS row 18 **[fold]** — "phantoms 12→0" and "0/155" are weave-r2-only (jink flew pre-fix code, 12→2); restore the ">8 m gross" qualifier on the 0/155 claim **[fold]**; add the jink-re-run-queued caveat.
5. **[fold] portfolio_visuals' ADR-0032 note now asserts a falsehood** — it says the 0.632 m addendum "has not yet been logged" and to treat 1.061 m as record; the addendum WAS logged 2026-07-08 (decisions.md:1053-1058) and inverts the instruction. Fix the note.
6. **Under-claim**: the fusion capstone — the project's only clean paired p≈0.008 win (8/8 seeds, −0.356 m median, survives WORST cue, pre-registered null-flip arc) — is absent from the quantified bullets and the pitch. Add the bullet; it's the best answer to "when does fusion help?"
7. Cosmetics: close PROGRESS M4.5 row (folded into M5); pin AprilTag control at 1.64 m; bullets date 07-09→07-10; reconcile $230 vs $257 Stage-0 cost **[fold]**.

### B. Design gaps — desk additions to the review doc + queue (all confirmed)

1. **[fold — verifier's "biggest miss"] Salvo Pk-independence**: the defensible-Pk path is 1−(1−p)^N, which assumes independent misses — but salvo members share the same gust, sun, and above all the same target maneuver; correlated terminal failure (ADR-0023, the project's most-repeated limiter) collapses the stack. kill_mechanism.md:294 self-flags this and nothing owns it. $0 probe: cross-seed miss correlation under shared maneuver. **Pairs with the fratricide gap** — and the sharper angle: bird_discrimination's ">1 candidate in gate ⇒ break off" rule **structurally aborts every salvo member** (a wingman is always near the FOV). The salvo concept and the ambiguity interlock currently contradict each other.
2. **Own-ship GNSS denial**: the jammer that kills the cue kills GPS L1 more easily; DASH navigates to an NED PIP and the 8 m gates compare NED positions → a GPS-jammed dash never reaches handoff — same fail-closed class as ADR-0059 via a different sensor. Zero doc coverage (grep-verified). Cheap sim probe: kill/degrade the GPS plugin mid-dash. Fold into the G3 campaign.
3. **Cold-start launch readiness**: ground-standby/launch-on-detect gives a 5-13 s window; cold GPS lock + EKF align + arming is 30+ s. No readiness variable exists anywhere (the M-1..M-4 latency knob models CUE delivery in ms, not vehicle readiness). Bench-measurable the moment Stage 2 exists; implies warm-standby design.
4. **Single-factor testing everywhere**: every exposure is a one-knob A/B while reality stacks blur+latency+gust+degraded r̂ in the same terminal second against 0.72 m of correction capacity. **Fix**: one standing stacked-EXPECTED regression arm, n≥16 paired — batch time only. **[fold]** Include the everyday joint case: own-GPS degraded + rig datum bias + maneuver vs the 8 m gate.
5. **Cue link has no message auth**: bare UDP JSON → `json.loads` (verified), now the truth reference that arms handoff. HMAC+freshness is $0 while it's one dataclass. **Caveat**: it touches m4_intercept.py, which is HELD uncommitted pending jam validation — **draft the ADR tonight, implement after the jam-fix commit.**
6. Mediums, one line each: adversarial counter-seeker (up-sun/dazzle/**below-horizon false-negatives** — training world is all sky-background; $0 replay probe); threat-envelope disclosure paragraph (everything is one quad body ≤16 m/s; fixed-wing tail-chase is kinematically infeasible); post-engagement (miss-RTL energy spreadsheet + net-debris siting note); seeker-process-death behavior (PX4 500 ms failsafe is a backstop, the intended behavior is undesigned); air-side calibration lifecycle (post-crash boresight check procedure); GPS/USB-camera RF desense bench check (C/N0 with everything transmitting); BOM holes (5 V BEC, Stage-3 cue transport, target Remote ID) → **feed to the running flight-compute council**; **[fold]** ROE/release-authority disclosure paragraph and target-reacts-to-being-attacked scenario noted as unmodeled.

---

## 3. Highest-value next work — queue corrections

**Add (unowned, high value):**
- **Real-footage detector eval** (review Action 1(b)) — the single highest-value unqueued item and the only one that puts a real photon through the pipeline; answers "has this seen a real image?" A negative result is still gold ("measured the domain gap"). Caveat: the bottleneck is footage acquisition — public sets (Anti-UAV, Drone-vs-Bird) are multi-GB (needs builder approval) and phone video needs the builder. Queue the builder ask; scaffold the harness now.
- **Publish track — re-scoped [fold]**: repo is 7.2 MB tracked with a clean .gitignore, so it's not a size problem. The real blocker: README cites gitignored `logs/*.csv` 17× including "every number traces to logs/mc_final_all.csv" — **the traceability claim is false off this machine**. Work = commit key evidence CSVs (or restage claims) + remote + LICENSE ADR + CI on the 88 offline tests ("honesty boundary enforced in CI" is a badge no student portfolio has).
- **#33 is mis-sized**: n=48 clean → 92.6% LCB; the ratified ≥95% needs **n≥72 clean** (0.025^(1/72)=0.950; one failure at 48 → ~89%). Decide the target claim and log the sizing ADR **before** the data. The jink-r2 n=16 half is genuinely needed (adopted config never batch-validated on jink post-fix).
- **G5 vertical probe pinned into #32's arm list** (still unowned — real silent-drop risk): ±1-2 m/s mover altitude schedule, 8 flights, 3D miss. If it shows the expected unguided vertical miss, schedule the vertical pro-nav channel as a named item — new guidance capability, higher portfolio ROI than another bound on an existing number.
- **[fold] G8 clutter/decoy world variant** (review Tier-2 HIGH) has no task number — the adopted config's gate radii have only ever seen one empty world. Give it a slot.
- **[fold] Cue-trust non-jam components unowned**: clock-skew tiers on cue timestamps (the ADR-0052 headline gap) and maneuvering stereo caches through station.py (G6); plus T19's owed items. One queue line each.
- Instrumentation (G11 gt-IoU logging, G12 4-quadrant bench test): write now, but ride the batches **after** the jam commit — instrumenting held code-under-validation before its MC adds diff risk.
- **[fold]** $0 desk residue: rolling-shutter superseding ADR; G10 own-state degradation knob.

**Keep as-is (lens corrections rejected by verifier):** #35 up-tilt stays early — drag likely makes real dash pitch WORSE nose-down, and the mount conditions every later campaign (running stats first then changing the mount would invalidate them); run it as a coarse 0°-vs-one-angle probe with the drag/transfer caveat in the ADR. T25 sequencing is already correct (after jam MC + jink, ahead of #32/#33/#30a); do the sim-free prep (shot-1/2 scaffolding, HUD, end-card copy) in idle windows.

**Cut/tail:** #30a full blur sweep (question answered by the sample; already scoped as unable to close the real-optics question) — tail or cut, redirect batch time to the G5 probe. **Dropped entirely**: flight-compute ADR item — the council is already running.

---

## 4. Overnight action list (sim-free first)

**NOW, before the train exports (~30 min):**
1. Patch `eval_seeker_v3.py`: clause_b noise guard, G3 ±1-frame/no-abs-upside, win requires clause_a, fix the WIN+NULL co-print; extend self-tests; commit as the pre-registration. If the train beats the patch, **hold scoring until it lands** (scoring is offline — no sim cost).
2. Eval preconditions: assert v3 calib sidecar exists before scoring (fail hard, no span=1.0 fallback), `MARKERLESS_SPAN_M` unset, fresh `--out` dir.
3. Write `scripts/check_jam_mc.py` per §1 (thresholds, completeness, stratification, engagement count, cutoff grep, per-seed table, RTF log); fix the "pre-acquisition" comment; stage the R=18 sweep arm config.

**Parallel sim-free (any order):**
4. Portfolio claim batch: HOLD notes/banners (interviewer_prep, visuals, WRITEUP), soften README "everywhere", r² rephrase ×4, jink-1/8 ADR addendum + scoping splits (bullets + PROGRESS row 18 + gross qualifier), visuals ADR-0032 note fix, fusion-capstone bullet, M4.5 close, 1.64 pin, dates/cost pin.
5. Design-review doc additions (§2B rows: salvo independence+fratricide, GNSS denial, launch readiness, stacked-WORST arm, adversarial-seeker, threat envelope, post-engagement, process death, cal lifecycle, RF desense) + NEXT.md queue edits (§3) + HMAC ADR draft (implement post-jam-commit) + feed BOM holes to the council.
6. Real-footage eval harness scaffold + builder ask (footage/download approval).

**Sim critical path (event-driven, one sim, idle load):** v3 export → verify span line → run **patched** eval → ADR-0061 with scoped claims (grid=val-split disclosure, mislock on its own line, Phase-0 sequencing sentence, conditional-on-renders phrasing, 07/08 eyeball, mislock/match cross-tab) → sim frees → jam MC 4 arms sequentially under `check_jam_mc.py` → if fixon validates: un-HOLD comms-denied, commit the held jam paths → jink-r2 n=16 → T25 render prep.

**Morning decisions for Emerson:** Pk-claim sizing (n≥72 vs keep the honest "no failures at n=X" framing); real-footage download approval (>2 GB); publish timing + LICENSE choice; Stage-0 cart (council output, incl. BEC/RID lines); whether to fly the R=18 jam sweep arm.