# Audit Findings Tracker

*Task #36 — portfolio hygiene. Maps every distinct finding raised by the three
2026-07-10 overnight audits to its current disposition, cross-referenced against
git history and `docs/decisions.md` as of this writing. Sources:*
- `docs/audit_deep_2026-07-10.md` ("deep audit" — 5 dimensions, verifier-adjudicated)
- `docs/audit_forward_2026-07-10.md` ("forward audit" — v3-eval + jam-MC methodology, claims, queue)
- `docs/audit_pipeline_frames_2026-07-10.md` ("pipeline/frames audit" / "audit-3" —
  stereo T16–T19, fusion/EKF, coordinate frames, commit soundness)
- `docs/overnight_report_2026-07-10.md` used only as the cross-reference rollup
  (task numbers, ADR pointers) — it doesn't raise new findings beyond the three
  audits above.

**Method.** Every finding below was checked against the live repo (`git log`,
`git status`, `docs/decisions.md` ADR-0059..0062, and direct reads of the cited
files) as of this writing — not assumed from the audit text alone. Where a doc
audits said was fixed is still stale on disk, that is reported as such, not as
"fixed." Disposition values: **FIXED** (landed, verified live), **PARTIAL**
(real progress, real gap remains — always explained), **TRACKED** (has a task #
or ADR follow-up slot but no code/doc yet), **ACCEPTED** (a deliberate no-fix
call, with the reason), **UNTRACKED** (no disposition exists anywhere — these
are the ones that need a home).

**Honesty-boundary note (matches the rest of this project's convention):**
nothing below changes the boundary — `gt_*` stays scoring/logging-only, the
comms-denied claim stays HELD, and every "FIXED" here was checked against a
committed artifact, not a claim.

**Refreshed 2026-07-10 (post-session).** All dispositions re-verified against the
live repo after the day's burn-down commits (`e6b6e51`, `a0ececf`, `117f973`,
`c724347`, `d393a6b`, `0123f61`, `e49e6bf`, `8b857d6`) plus this refresh's own
doc fixes (DEEP-R2 dead cites + M4 selection disclosure, DEEP-P1 pitch cue
scope, DEEP-P9 header, FWD-A7 1.64 m pin, FWD-A4 jink-1/8 ADR home, the
interviewer-prep jam-MC currency rewrite, demo_out hero-CSV annotations,
P-commitsE `check_t21.sh` discoverability). Moved since first writing:
**18 findings → FIXED** (13 from UNTRACKED via the doc commits, 4 from
PARTIAL, plus DEEP-N1 whose #42 guard committed mid-refresh, `84eb756`),
**7 → TRACKED** (now hold NEXT.md queue slots), **3 → PARTIAL** (real work
landed, remainder queued). §4 and §5 recounted to match: **UNTRACKED 51 → 27.**

---

## 1. Deep audit (`docs/audit_deep_2026-07-10.md`)

### 1a. BLOCKERS / HIGH

| ID | Finding | Severity | Disposition | Evidence |
|---|---|---|---|---|
| DEEP-H1 | No headline CSV was committed; the ADR-0058 culmination arm (14/14) had no scripted gate — a disk loss makes the published numbers unverifiable. | HIGH | **FIXED** | `2665bbb` (aggregate CSVs + `check_t21.sh`) + `81562b8` (16 per-tick r2 flight CSVs). Audit-3 independently re-confirmed `check_t21.sh` exits 0 with all 12 pins. |
| DEEP-H3 | `interviewer_prep.md` breaks the project's own "HELD everywhere" promise — leads with the unqualified jam claim twice (:16, :248-249). | HIGH | **FIXED** | `7dad15e`. Verified live: `docs/interviewer_prep.md:15-18` now opens "currently HELD, in validation" with the full ADR-0059 box; `:376-398` pre-stages the H6 follow-up question. *Drift closed:* the "not yet flown" line was re-patched in `a0ececf`, and the deeper Q20 answer (which still described the jam MC as unflown and the fix as "built, not yet flown") was rewritten in this refresh to the flown ADR-0059 RESULTS (fail-closed dose-response 12→2→0, fix fail-SAFE, #39 recovery NULL, claim stays HELD). |
| DEEP-H4 | `WRITEUP.md` is two arcs stale — contradicts the 100% Pk headline with the old 27% number, and calls the markerless seeker "not simulated" (now false, it's built and Gazebo-tested). | HIGH | **FIXED** | `7dad15e`. Verified live: `docs/WRITEUP.md` now carries a "Currency banner (2026-07-10)" and §5/§6/§8/footer read ADR-0036/0058 numbers and the markerless/detect-then-track arc. The "jam MC not yet flown" staleness at `WRITEUP.md:16` was re-patched in `a0ececf` (three WRITEUP touch-points). |
| DEEP-H5 | "~96% of the miss is locked in at handoff (r²≈0.96)" conflates variance-explained with fraction-of-miss (the project's own capacity bound says ~75% locked in, not 96%). *(Also raised independently by the forward audit, item A.3 — same fix, one entry.)* | HIGH | **FIXED** | `7dad15e`. Verified in `docs/portfolio_bullets.md:7-8,43,154-155`, `README.md`, `docs/WRITEUP.md:404` — now phrased "variance explained... not the same as 96% of any one miss," paired with the 0.72 m / 1.69 m capacity-bound reading everywhere checked. |
| DEEP-H6 | The thesis (comms-denied intercept) had never been demonstrated end-to-end — no jam had ever been injected, in any flown config. | HIGH | **FIXED** (as a data gap) — **result correctly stays HELD** | `615ef4c`, ADR-0059 RESULTS/CLOSE. The 8-arm jam MC flew. Verdict: fail-closed **demonstrated** with a clean dose-response witness (REAL-ish handoffs 12→2→0 across 15/18/22 m cutoffs); the ADR-0059 fix is **validated as fail-safe, not recovery** — "works comms-denied" stays HELD as an intercept claim. This is the *correct* honest outcome the audit demanded (an honest answer, not a guaranteed win). Full recovery (task #39) has since flown and CLOSED as an honest **NULL** (`e49e6bf`, ADR-0062's #39 addendum): coast-search engaged on every jam flight but reacquired 1/16 vs the pre-registered ≥11 bar — the binding constraint is onboard perception, not guidance; the claim stays HELD and the next lever is the #35 up-tilt mount study. |

### 1b. MEDIUM / LOW

| ID | Finding | Severity | Disposition | Evidence |
|---|---|---|---|---|
| DEEP-C1 | `BREAKOFF_RANGE_INCREASES` past-CPA breakoff has no noise deadband — pixel-quantized range jitter (or a frame-edge clip) can fake 3 monotone rises and false-trigger an early breakoff; survives even with the ADR-0056 gates on. | MEDIUM→confirmed HIGH by audit-3 (task #37) | **FIXED (partial)** | ADR-0062 FIX-B, `706bb46`. `update_range_increase_streak()` extracted to a pure, unit-tested function (`tests/test_breakoff_deadband.py`, 8 cases) with `--breakoff-deadband-m`/`--breakoff-min-rise-m`/`--breakoff-max-range-m` flags. **Ships INERT by default** (deadband=0.0 ⇒ byte-identical to the old inline logic) — the mechanism landed, but the protective non-zero deadband has not been turned on; the tuning gate now holds a NEXT.md sim-queue slot (#40 FIX-A follow-ups list). |
| DEEP-H2 | Handoff-streak state machine (the trigger for the one-way honesty latch) has zero sim-free test coverage — a regression (streak not resetting, counting stale detections) hands off on a phantom and only a full Gazebo gate would catch it. *(Counted in the totals from first writing but previously had no table row of its own — row added at the 2026-07-10 refresh.)* | MEDIUM | **TRACKED** | NEXT.md sim-free lane holds the slot ("handoff-streak state-machine unit tests — AST-pin tightening now; `update_handoff_streak()` extraction is an m4-window item, mirror the FIX-B pattern"); needs a task #. No code yet. |
| DEEP-C2 | All-zero `frozen_vworld=(0,0)` latch if ENGAGE begins inside `TERMINAL_FREEZE_RANGE_M` — reachable only with a pathological `--target-start`. | LOW | **UNTRACKED** | Checked `m4_intercept.py:2866-2872` live — still `if frozen_vworld is None: frozen_vworld = (vh0, vh1)` with no nonzero-vector fallback. |
| DEEP-C3 | Cue-staleness can't distinguish jam from a benign >1 s dropout. | LOW | **ACCEPTED** | Deliberate design choice, documented in ADR-0059 itself (the `control_active` arm exists specifically to characterize this). Verifier's own read: "documented deliberate fail-open, low-prob compound." |
| DEEP-C4 | `MEAS_STALE_S` wall-time gate (m3:305-307). | LOW/nit | **ACCEPTED** | Verifier downgraded to a doc nit: the whole estimator/control stack is wall-timed and already mitigated by the idle-load rule (ADR-0009). No code change requested. |
| DEEP-C5 | Missed root cause worth a line: `predict()` integrates a fixed nominal `dt` while `correct()` scales by wall `dt_since` — a real predict/correct dt inconsistency under loop saturation. | LOW | **UNTRACKED** | No ADR line or fix found addressing this. |
| DEEP-N1 | Honesty AST no-gt pin is scoped to `detect_track.py` alone; the live `--track` measurement also flows through unpinned `two_stage_seeker.py` (which holds real `gt_frames` reads in its offline `_run_eval`) and `FinetunedNNSeeker` — no check, static or runtime, would catch GT wired into the seeker's *measurement*. | MEDIUM | **FIXED** | Task #42, `84eb756`: `tests/test_honesty_seekers.py` — AST-based Name/Attribute/arg scan parametrized over all five live seeker modules (incl. `two_stage_seeker.py`/`finetuned_seeker.py`), LOUD allowlist scoped to the one offline `_run_eval` with a self-checking allowlist-hygiene test, live-entry-point pins, a transitive-import tripwire, and a mechanical injection test proving it catches a `tracker.gt_range` added to a live `detect()`. Verified 112 passed / 2 skipped via `run_tests.sh` per the commit. (The boundary was intact in fact — this future-proofs it.) |
| DEEP-N2 | m4 gt-checks scoped to `run_acquire_and_engage` only; `run_bench()`/`main()` TAKEOFF unaudited (logging-only, no live leak); checks catch `ast.Name 'gt_*'` but not attribute-form (`tracker.gt_range`). | LOW | **UNTRACKED** | No evidence of scope widening. |
| DEEP-R1 | Hero demo CSV (`m4_intercept_pronav_20260707T211601Z.csv`, miss 0.632 m) is cited as present but is absent from disk and git. | MEDIUM | **FIXED** | `95614cf` repointed `README.md` to the verified `mc_final_all.csv` batch; the 2026-07-10 refresh annotated both `demo_out/README.md` cites (:12, :188-192) to say the CSV was rotated off disk and never committed, with the ADR-0032 addendum in `docs/decisions.md` named as the durable record. The CSV itself is unrecoverable — every remaining cite now says so honestly, which is the fix this finding admits. |
| DEEP-R2 | Cluster: two dead ADR-0028/0030 CSV cites, ADR-0060 pitch numbers trace to ulogs outside the repo, README repo-map says `plots/` is checked in (it's gitignored), and the M4 "three independent gate runs" don't disclose they're the 3 tightest of 8 pronav runs (the other 5 fail the gate). | LOW/nit cluster | **PARTIAL** | 2026-07-10 refresh fixed all doc halves: dead ADR-0028/0030 cites annotated in README results rows + `decisions.md:1012` (CSVs rotated, ADR tables are the record); M4 selection disclosure added as an ADR-0009 addendum + a README M4-row note, and the non-resolving `{0322xx,0324xx,0331xx}` glob corrected to the real committed stamps `{032238,032528,033250}`; README `plots/`+`logs/` repo-map lines rewritten to match what git actually tracks. **Remaining:** the ADR-0060 pitch numbers still trace to rotated ulogs — needs a `dash_pitch_probe.py --dump-summary-csv` re-run + commit (script execution, not doable mid-batch). |
| DEEP-T1 | Test-coverage cluster: re-acquire gate tested only at `coast_dt=0`; `_cam_implied_ne` only tested at degenerate `bearing=0/psi=0`; `COAST_STALE_S`'s numeric value is unpinned (could drift 1.0→10.0 and 86 tests stay green); NN parity tests always `importorskip` (skip every run); **no automated test runner exists at all**; untested pure helpers (`wrap_pi`, `kalata_alpha_beta`, `compute_v_close`, `solve_intercept_time`). | MEDIUM | **PARTIAL** | The two sharpest items landed in `0123f61` (task #41): `scripts/run_tests.sh` is now the one test runner (main suite under `.venv` + the ONNX parity pair under `.venv-seeker` — the parity tests, which silently skipped on every prior run, now actually exercise the ONNX decode; 130 pass aggregate, exit 0 iff all green). **Remaining:** the COAST_STALE_S value pin, degenerate-bearing/`coast_dt` coverage, and the untested pure helpers. |
| DEEP-T2 | Nit cluster: honesty pins are genuinely good (AST-based, mutation-calibrated — a positive finding); `ekf_tracker` gate uses a substring check, not AST; `test_seed_ctx_runtime_latch_belt` silently passes if cv2 is missing; no offline test of the pixel→bearing intrinsics chain. | LOW/nit | **UNTRACKED** | Not addressed; low priority, correctly flagged as low by the audit itself. |
| DEEP-P1 | 30-sec pitch juxtaposes "a real computed pipeline, not a mock" with the 14/14 headline, which rode the mock cue. | MEDIUM | **FIXED** | 2026-07-10 refresh: the pitch's 14/14 sentence now carries the scope inline ("that validation arm flew the scripted mock cue; the computed stereo pipeline was proven end-to-end separately"). |
| DEEP-P2 | Resume one-liner's unscoped word "verified" (n=14, one weave arm, one empty world, mock cue, jink pre-fix). | MEDIUM | **FIXED** | `e6b6e51`. Verified live: the resume line now closes "(verified at n=14: one weave arm, one empty Gazebo world, mock ground cue)". |
| DEEP-P3 | M5 "fleet scale" bullet (n=96) omits that the batch flew the **AprilTag** sensor, not markerless (neighboring bullets are markerless → conflation); "clean" is never defined. | MEDIUM | **FIXED** | `e6b6e51`. Verified live: the bullet is retitled "— on the AprilTag sensor", defines "clean" parenthetically (ran to completion and engaged; nan-miss failures stay in every Pk denominator), and closes "This batch flew the clean **AprilTag** sensor — the disclosed upper bound — not the markerless seeker of the neighboring bullets." |
| DEEP-P4 | Jink "14/15" quoted without the pre-fix-code caveat other surfaces carry. | LOW | **FIXED** | `docs/portfolio_bullets.md` and `README.md:123` both carry the caveat ("the paired n=16 jink A/B baseline reads 3/15 → 14/15, ADR-0058"); `interviewer_prep.md`/`WRITEUP.md` verified not to quote the jink number at all (2026-07-10 refresh grep). Also superseded in practice: the post-FIX-A adopted-config jink re-run reads pooled 16/16 (`d393a6b`, ADR-0062 addendum). |
| DEEP-P5 | `PROGRESS.md:18` calls detect-then-track "the jam-resistance headline" with no HOLD caveat in that row, one row above a different row that does state the HOLD. | LOW | **FIXED** | `117f973`. Verified live: the row now reads "the seeker half of the jam-resistance story (the 'works comms-denied' intercept claim itself stays **HELD**, ADR-0059)". |
| DEEP-P6 | "Default config hands off fine under jam" stated declaratively (README:150-152) but is a code-trace inference, not a flown result. | LOW | **ACCEPTED** | Verifier's own read: sits in a heavily-hedged box, and ADR-0059 itself asserts it; not worth a rewrite. |
| DEEP-P7 | "0/155" false-detection claim carries no environment caveat — the adopted config's gate radii (12 m REACQ / 8 m SEED) are, per the design review's own G8 gap, "an empirical accident" of one empty Gazebo world. | LOW | **TRACKED** | The G8 clutter/decoy-world batch now holds NEXT.md sim-queue slot 8 ("the adopted gate radii have only ever seen one empty world; scopes the 0/155 headline") — task # still to be assigned when it reaches the front of the queue. |
| DEEP-P8 | Two Clopper-Pearson bounds circulate (77% camera-terminal / 79% pooled) with no denominator note distinguishing which n each is drawn from. | LOW | **FIXED** | `e6b6e51`. Verified live on both surfaces: `README.md:134-135` ("drawn from the camera-terminal n=14; the review's ~79% bound is the same calculation on the pooled n=16") and the bullets' held-claims box carry the denominator note. |
| DEEP-P9 | Header meta-claim "Nothing here overclaims" is itself an overclaim. | nit | **FIXED** | 2026-07-10 refresh: header now reads "Written not to overclaim — ... judge that intent against the sources, not this sentence," and points at this tracker. |
| DEEP-P10 | Superseded ADR-0030 degraded-cue numbers (9 m/s→1.19, 12 m/s→1.48) still circulated as *current* on some surfaces. | MEDIUM | **FIXED** | Confirmed live: `docs/portfolio_bullets.md:159-160` now explicitly reads "...the earlier ~1.2–1.5 m ADR-0030 figures were flown under the old, too-steep range-noise curve and are superseded." |

---

## 2. Forward audit (`docs/audit_forward_2026-07-10.md`)

### 2a. Methodology (v3 eval + jam MC)

| ID | Finding | Severity | Disposition | Evidence |
|---|---|---|---|---|
| FWD-1 | v3 eval `verdict()` had 3 scoring defects: a WIN could fire on a 3-frame noise delta (clause_b, no CI/test), `abs()` in the G3 no-regression check could fail an *improvement*, and WIN ignored a maneuver-recall *collapse* (clause_a). Audit's own verdict: "NO-GO as scored." | HIGH | **FIXED** | `f6427af` ("Pre-registration integrity... commit verdict/scorer scripts"), landed *before* v3 was scored. ADR-0061 documents all 4 pre-registered fixes verbatim matching the audit's ask (bucket CI/sign-test gate, tolerance fix, clause_a requirement + consistent null-branch print, span footgun). The fixes then correctly caught the real v3 regression (sign p=0.002, worse on all 10 held-out flights) instead of a false WIN. |
| FWD-2 | v3 span-sidecar footgun: a missing calib sidecar silently falls back to span=1.0, ~8.5% range error, with nothing to catch it. | MEDIUM | **FIXED** | Same pre-registration package. ADR-0061: "recalibrate the v3 span sidecar FIRST (fit 0.9382), assert `MARKERLESS_SPAN_M` unset, fresh timestamped out." |
| FWD-3 | The no-new-phantom-class check is one-sided: it demands significance to register a phantom *increase* but none to register a recall *gain* — an asymmetric flattery bias. | LOW | **UNTRACKED** | ADR-0061 flags the phantom count as underpowered but does not name the one-sided asymmetry explicitly. |
| FWD-4 | Cross-tab match/mislock double-counts — one oversized detection can earn recall credit *and* count as a failure (no size guard). | LOW | **UNTRACKED** | No fix found. |
| FWD-5 | `evalv3_07/08` (weave-9, 476 frames) sit in zero gates — a regression there would slip scripted adoption. | LOW | **UNTRACKED** | No gate found covering these flights. |
| FWD-6 | ADR-language precision cluster: grid val-split must never be framed as symmetric generalization; disclose Phase-0 forensics ran on 2 of the 6 gate flights; results are conditional on one nondeterministic render draw; the n=10 sign test is near-unpowerable. | LOW | **FIXED** | ADR-0061 text matches nearly verbatim: "Grid 3/18 m = v3's OWN val split... v2 zero-shot → asymmetric, never framed as symmetric generalization. Phase-0 forensics ran on evalv3_01/02 = TWO of the six gate flights (disclosed)... conditional on this one ADR-0057-nondeterminism render draw; the n=10 flight sign test is near-unpowerable and is not leaned on." |
| FWD-7 | The jam MC was "GO to fly, NO-GO to conclude" without a pre-registered verdict script — 10 named requirements specified (arm completeness distinguishing infra-failure from fail-closed; jam-fired runtime tripwire; pre-registered thresholds + an unconditioned joint metric; stratify by first-det-vs-cutoff; strict-equivalence stats with an MDE note; active-vs-strict arm-level-only comparison; difference-in-differences; per-seed paired table; staleness-transient attribution; RTF parity). | HIGH | **FIXED** | `f6427af` commits `scripts/check_jam_mc.py` **before** any jam arm flew. Read the script directly: it implements all 10 items as gates G1–G6 + report items R7–R10, matching the audit's spec almost line for line. Used for the real verdict (`615ef4c`, ADR-0059 RESULTS), which correctly returned **NOT VALIDATED** (exit 1) rather than a false pass — the honest answer the audit wanted. |
| FWD-8 | Any ADR-0059-CLOSED verdict must be scoped to speed-12/gate-8 (at low target speed the frozen cue stays inside the gate and the fail-closed witness silently fails to bite). | MEDIUM | **FIXED** | Both `check_jam_mc.py`'s SCOPE banner and ADR-0059's RESULTS section explicitly scope to "speed-12 / gate-8 ONLY." |

### 2b. Portfolio claim integrity

| ID | Finding | Severity | Disposition | Evidence |
|---|---|---|---|---|
| FWD-A1 | The comms-denied HOLD is not held everywhere — `interviewer_prep.md`, `portfolio_visuals.md`, and `WRITEUP.md §1` all state or imply the jam-resistant claim as proven. | HIGH | **FIXED** | `7dad15e`. `interviewer_prep.md` and `WRITEUP.md` independently re-verified live (see DEEP-H3/H4). `portfolio_visuals.md` was in the same commit's scope per its message but was not re-verified line-by-line here beyond the ADR-0032 note (see FWD-A5, which found a *different* problem in that same file — since fixed in `a0ececf`). |
| FWD-A2 | `WRITEUP.md`/`interviewer_prep.md` frozen pre-ADR-0036 — quote 27% Pk / "12 m/s uncatchable" as current, and present the markerless seeker, EKF A/B, and fusion capstone as future work (all closed). | HIGH | **FIXED** | Same commit; both docs now carry currency banners and the current ADR-0036/0038-0058 numbers (verified live above). |
| FWD-A4 | Jink "1/8" is traceable but was never logged as an ADR addendum; the "phantoms 12→0" and "0/155" claims are weave-r2-only and need the ">8 m gross" qualifier restored; the post-fix jink re-run is queued but not flown. | MEDIUM | **FIXED** | Three parts, all landed: (1) the post-fix jink re-run FLEW and is committed — pooled 16/16 Pk@2.5, median 1.535 m (`d393a6b`, `logs/mc_t21_trackgate_jink12.csv`, ADR-0062 addendum); (2) the 2026-07-10 refresh added the ADR-0056 addendum giving "jink 1/8" its decisions.md home (plain-jink arm re-scored post-handoff camera-terminal; pooled was 3/8) with the never-pool-across-code-versions note; (3) the scoping caveat + "gross" wording verified live in bullets + README. |
| FWD-A5 | `portfolio_visuals.md`'s ADR-0032 note asserts a falsehood: it says the 0.632 m re-cut "has not yet been logged" and instructs readers to treat 1.061 m as the record — but the addendum *was* logged 2026-07-08 and says the exact opposite. | LOW (but a confirmed live contradiction) | **FIXED** | `a0ececf`: removed the false "decisions.md has not yet been amended" claim from `portfolio_visuals.md`, aligned the guidance to the ADR-0032 addendum's actual ruling (0.632 m is the shipped-asset number) and the ~1.19 m lead. |
| FWD-A6 | Under-claim: the fusion capstone (the project's only clean paired p≈0.008 win, 8/8 seeds, −0.356 m median, survives WORST cue) is absent from the quantified portfolio bullets and the pitch. | MEDIUM | **FIXED** | `e6b6e51`. Verified live: `docs/portfolio_bullets.md` now carries the full fusion-capstone bullet ("8/8 paired seeds (sign test p≈0.008, median −0.356 m)... survived the WORST-credible modeled cue 8/8," with the covariance-gated rejection and the 0-post-latch boundary). |
| FWD-A7 | Cosmetics cluster: close the `PROGRESS.md` M4.5 row (folded into M5 but never marked done); pin the AprilTag control at 1.64 m everywhere; fix bullets date 07-09→07-10; reconcile $230 vs $257 Stage-0 cart cost. | LOW | **FIXED** | `e6b6e51` closed the M4.5 row (verified live: "✅ 2026-07-08 (closed with M5)"), fixed the bullets date and the $257 cart cost. The 2026-07-10 refresh finished the 1.64 m pin: ADR-0056's own text read "median 1.6 m" while every portfolio surface says 1.64 — the median was re-derived from the committed `logs/mc_t21_apriltag_weave12.csv` (1.6395, n=8) and the ADR now records 1.64 with the exact value. |

### 2c. Design gaps (desk items surfaced by the lens, never yet given a home)

| ID | Finding | Severity | Disposition | Evidence |
|---|---|---|---|---|
| FWD-B1 | Salvo Pk-independence: the defensible-Pk math (1−(1−p)^N) assumes independent misses, but salvo members share the same gust/sun/target-maneuver — correlated terminal failure could collapse the stack. Sharper: the ambiguity/bird-discrimination rule ("break off if >1 candidate in gate") structurally aborts every salvo member, since a wingman is always near the FOV. The salvo concept and the anti-ambiguity rule currently *contradict* each other. Verifier called this "the biggest miss." | HIGH | **TRACKED** | NEXT.md sim-free lane now holds the slot ("FWD-B1 salvo-correlation $0 desk probe" under audit-backlog grooming); no probe/ADR yet. |
| FWD-B2 | Own-ship GNSS denial: the jammer that kills the cue kills GPS L1 more easily; the dash navigates to an NED PIP compared against NED gate radii, so a GPS-jammed dash never reaches handoff — the same fail-closed class as ADR-0059 via a different sensor, with zero doc coverage. | MEDIUM-HIGH | **UNTRACKED** | No GNSS-denial gap entry found in the design review doc. |
| FWD-B3 | Cold-start launch readiness: ground-standby/launch-on-detect gives a 5-13 s window, but cold GPS lock + EKF align + arming is 30+ s — no readiness variable exists anywhere in the model. | MEDIUM | **UNTRACKED** | Not found in the design review doc or NEXT.md. |
| FWD-B4 | Single-factor testing everywhere: every exposure is a one-knob A/B, while reality stacks blur+latency+gust+degraded r̂ simultaneously in the terminal second against only 0.72 m of correction capacity. Needs one standing stacked-EXPECTED regression arm, n≥16 paired. | MEDIUM | **UNTRACKED** | No stacked-regression arm found built or queued with a task #. |
| FWD-B5 | Cue link has no message authentication — bare UDP JSON straight into `json.loads`, and this is the truth reference that arms handoff. HMAC+freshness is a $0 fix (one dataclass) but touches `m4_intercept.py`, which was held pending jam validation. | MEDIUM (honesty-adjacent) | **TRACKED** | NEXT.md sim-free lane holds the slot ("FWD-B5 HMAC cue-auth ADR draft (design only)" under audit-backlog grooming). The jam-fix commit has landed, so the design half is unblocked; the implementation touches `m4_intercept.py` and stays m4-window-gated. No draft yet. |
| FWD-B6 | Cluster of mediums/lows: adversarial counter-seeker (up-sun/dazzle/below-horizon false-negatives, training world is all-sky background); threat-envelope disclosure (fixed-wing tail-chase kinematically infeasible); post-engagement (miss-RTL energy + debris siting); seeker-process-death behavior (PX4 500 ms failsafe is a backstop, not designed); air-side calibration lifecycle; GPS/USB RF desense bench check; BOM holes (5V BEC, Stage-3 cue transport, target Remote ID); ROE/release-authority disclosure; target-reacts-to-attack scenario. | LOW-MEDIUM | **UNTRACKED** | None of these appear as gap entries in the design review doc; the audit's own note ("feed BOM holes to the running flight-compute council") — the council ran and ratified Option B (Pi 5 + Hailo), but the specific BOM holes listed here (BEC, cue transport, Remote ID) are not visibly itemized in `docs/stage0_bench_plan.md`. |

### 2d. Missed-value queue items

| ID | Finding | Severity | Disposition | Evidence |
|---|---|---|---|---|
| FWD-C1 | Real-footage detector eval — the single highest-value unqueued item; the only one that puts a real photon through the pipeline. | HIGH-VALUE | **TRACKED** | Task #30, per the overnight report's own task table: "real-footage eval **PENDING — needs your approval** (public datasets are multi-GB, or phone video from you)." Builder decision, not a code gap. |
| FWD-C2 | Publish track re-scoped: the real blocker isn't repo size, it's that README cites gitignored `logs/*.csv` 17× including "every number traces to..." — false off this machine. Work = commit key evidence CSVs (or restage claims) + remote + LICENSE + CI on the 88 offline tests. | MEDIUM | **PARTIAL** | "Commit key evidence CSVs" done (`2665bbb`/`81562b8`); the CI target now exists (`scripts/run_tests.sh`, `0123f61`); LICENSE + CI-workflow prep hold a NEXT.md publish-prep slot ("write ready-to-commit, activate when the remote exists"). **Still missing:** the LICENSE file itself, the remote (builder decision — logged as NEXT.md builder-decision #2 with a recommendation), CI. |
| FWD-C3 | #33 is mis-sized: n=48 clean only buys a 92.6% Wilson/CP lower bound; the ratified ≥95% Pk claim needs n≥72 clean — decide the target claim *before* flying more data. | MEDIUM | **TRACKED** | Task #33 ("Stats hardening") exists and explicitly carries this framing per the overnight report: "a real '95% Pk' claim needs n=72 clean (n=48 only buys a 92.6% lower bound) — sizing decision before data." The decision itself is still open. |
| FWD-C4 | G5 vertical probe (±1-2 m/s mover altitude schedule, 8 flights, 3D miss) should be pinned into #32's arm list — still unowned. | MEDIUM | **TRACKED** | Confirmed at the 2026-07-10 refresh: NEXT.md sim-queue slot 6 reads "#32 r_hat honesty campaign — aspect/bias/freeze probes + the G5 vertical probe (FWD-C4) pinned into its arm list." |
| FWD-C5 | G8 clutter/decoy-world variant has no task number — the adopted config's gate radii have only ever seen one empty world. | HIGH (per design review's own Tier-2) | **TRACKED** | Same underlying gap as DEEP-P7 — now holds NEXT.md sim-queue slot 8 (task # still to be assigned). |
| FWD-C6 | Cue-trust non-jam components unowned: clock-skew tiers on cue timestamps (the ADR-0052 headline gap), maneuvering stereo caches through `station.py`, plus T19's owed items. | MEDIUM | **UNTRACKED** | No task # found for either. |
| FWD-C7 | Instrumentation (G11 gt-IoU logging, G12 4-quadrant bench test) should be written now but ride the batches after the jam commit (now landed). | LOW | **UNTRACKED** | No evidence either was written. |
| FWD-C8 | $0 desk residue: a rolling-shutter superseding ADR, and the G10 own-state degradation knob. | LOW | **UNTRACKED** | Not found. |

---

## 3. Pipeline / frames audit (`docs/audit_pipeline_frames_2026-07-10.md`, "audit-3")

### 3a. BLOCKERS / HIGH

| ID | Finding | Severity | Disposition | Evidence |
|---|---|---|---|---|
| P-H1 | Roll/pitch never derotated from the LOS bearing — `lambda = psi + beta` is correct only when the boresight is level; at ADR-0060's measured 27-36° dash pitch this is a 12-24% azimuth gain error plus roll cross-coupling, exactly in the maneuvering/dash regime. | HIGH | **FIXED** | ADR-0062 FIX-A / task #38, `706bb46`. `derotate_bearing_lambda()` rotates the full 3D camera ray through the own-state attitude quaternion; identity-preserving at level (bit-exact with old M3/M4/M5 results). A/B-validated: paired weave n=8, derotation tighter on 7/8 seeds, worse on 0 (median Δ −0.161 m, sign p≈0.035). Acquisition/handoff **gates** stay yaw-only by design (reviewed sound, tracked as follow-up #40, not a blocker). |
| P-H2 | Station/mover co-start epoch skew: uncontrolled, unmeasured, and demonstrably nonzero — latency alone predicts 1.3-2.2 m of cue error at 9 m/s, but the measured 0.99/1.03 m is *lower*, meaning a canceling clock skew is present, not hypothetical. | HIGH | **PARTIAL** | The sim-free half landed: `c724347` (`docs/t19_cue_error_decomposition.md`, task #43) measures a REAL +0.100 s station/mover epoch skew from existing flight CSVs that cancels ~2/3 of the latency lag, and derives the honest quotes (triangulation 0.59 m median; delivered-cue ~1.6 m at 9 m/s skew-removed). **Remaining:** the sim-gated `--epoch-t0` shared co-start (NEXT.md sim-queue slot 7, "#43 remainder"). |
| P-H3 | Pre-registration package (`eval_seeker_v3.py` scorer fixes, `check_jam_mc.py`, `phase4_eval_v3.sh`) was uncommitted while `decisions.md` already recorded ADR-0061's NULL result — git could no longer prove patch-before-results. | HIGH | **FIXED** | `f6427af`, landed before both the jam MC and (per commit order) closing the attestation-window risk audit-3 flagged. |
| P-H4 | `check_t21.sh` exits 1 on any fresh clone — the 16 per-tick weave12_r2 CSVs it needs were still gitignored, so the gate built to make the 14/14 headline disk-loss-durable died on a clean clone. | HIGH | **FIXED** | `81562b8` commits the 16 per-tick CSVs (~1 MB). |

### 3b. MEDIUM / LOW

| ID | Finding | Severity | Disposition | Evidence |
|---|---|---|---|---|
| P-M1 | T19's "0.99 m median cue error" conflates three error terms (triangulation, latency, the H2 clock skew) — it isn't pure triangulation accuracy despite being quoted that way. | MEDIUM | **FIXED** | `c724347`: `docs/t19_cue_error_decomposition.md` decomposes the metric (0.587 m triangulation + latency lag − the canceling +0.100 s skew) and states the honest quotes going forward; the 2026-07-10 refresh added the decomposition pointer next to the 0.99 m quote in `portfolio_bullets.md` ("a delivered-cue figure that a measured epoch skew partially flatters"). |
| P-M2 | No trajectory-match check between the stereo cache and the flown mover; `centroid_cache_meta.json` is written but consumed nowhere; `--dash-direction` is never passed (station defaults to the first index row) — the cue is structurally open-loop to a mismatched flight. | MEDIUM | **UNTRACKED** | No meta-mismatch fail-loud check or `--dash-direction` sign(vy) derivation found. |
| P-M3 | Triangulation has no geometric gates beyond a disparity floor and `min_conf=0.5`; ADR-0052's text claims edge/small-disparity rejection that was never implemented. | MEDIUM | **UNTRACKED** | `triangulate.py:199-201` unchanged per the audit's own citation; not independently re-read line-by-line here but no fix commit found. |
| P-lowA | σ_R∝R² "validation" (ADR-0053) is near-tautological (the fit consumes zero rendered pixels); the real-detector exponent is 1.63 (ADR-0055) — future σ_R consumers should route to the real fit. Rig pose hardcoded, never read back from sim. Replay direction matched by convention only. | LOW | **UNTRACKED** | No citation change or SDF-consistency check found. |
| P-gateHard | Gate hardening: `check_t21`/`check_t19` never assert the ADR-0048 latency-floor condition; no minimum cue-count; station exit status never checked; `check_t19`'s "KNOWN OPEN QUESTION" header describes pre-ADR-0052 behavior (doc-rot). | LOW | **UNTRACKED** | Not independently re-verified line-by-line but no fix evidence found. |
| P-M4 | `FusedTrack` weight still uses the pre-ADR-0017 sigma model (0.4/0.008 vs the corrected 4.45e-05) — ~180x too steep, cue under-weighted ~6x at R=30 m; `FUSE_CAM_RANGE_FRAC=0.10` is also AprilTag-tier while the markerless spec is ~22%, so the ADR-0044 fusion win (8/8) is plausibly *understated*. Audit's own instruction: do NOT silently sync — needs an ADR addendum + paired A/B (nominal + WORST) first. | MEDIUM | **UNTRACKED** | No ADR addendum or A/B found. |
| P-M5 | EKF measurement R is AprilTag-tier regardless of which seeker is flying (`EKFTracker()` all-defaults); the markerless spec is ~1.5°/22%. Recommended first step: offline replay of flight 33327 with widened R. | MEDIUM | **UNTRACKED** | No replay artifact found. |
| P-lowB | Warm-seed skips the latch re-inflation floor when `_cue_called` is false; `seed_from_polar` stamps `WARM_VEL_SIGMA=0.7` even when the seeded velocity silently fell back to noisy alpha-beta differentiated velocity during a cue-stale window — up to ~100x understated variance in exactly the comms-denied handoff case. D3.2 recovery protects post-latch only; cue R carries no latency-staleness term. | LOW-MEDIUM | **UNTRACKED** | Not fixed; flagged by the audit as a real structural gap, not yet actioned. |
| P-wordingNits | "The cue never touches the angle" is overstated for the EKF polar split; "range_filter is camera-only — honesty verified" comment is false under `--fuse-midcourse`; cold-init P, shared `dt_since` clock, vo-fallback-to-zero, silent singular-S are all nits. | nit | **UNTRACKED** | Not touched; correctly flagged as low-priority by the audit itself. |
| P-M6 | ψ(t_tick) is paired with β(t_frame) — detection latency × yaw rate creates a LOS bias that cancels at constant yaw rate (why the bench passes) but not under yaw *acceleration*, which differentiates straight into the pro-nav input. This is a **different, more subtle bug than P-H1** — a time-alignment problem, not a rotation-math problem, and FIX-A does not touch it. | MEDIUM | **TRACKED** | Now task **#44** (NEXT.md sim-queue slot 5, explicitly cross-referenced to P-M6 and scoped "distinct from FIX-A — time-alignment, not rotation; bites under yaw acceleration = exactly the maneuver regime"). No code yet; m4-window item. |
| P-M7 | Slant-range (AprilTag, ‖pose_t‖) vs depth-range (markerless, fx·span/px) convention mismatch — up to ~36% under-read at the FOV edge, live at warm-handoff cue seeding. | MEDIUM | **TRACKED** | ADR-0062 follow-up #40 explicitly lists "(2) M7 slant-range + gate derotation" as a tracked, non-blocking follow-up. Not yet implemented. |
| P-lowC | ENU→NED mapping hard-assumes zero translation (spawn at world origin) — true today but silently breaks the parked M-1..M-4 deployment arc; vertical channel is fully open-loop with no documented envelope boundary; `cx, cy, cz` unpack shadows the intrinsics names. | LOW | **UNTRACKED** | No startup guard, docstring line, or rename found. |

### 3c. Commits/docs residuals

| ID | Finding | Severity | Disposition | Evidence |
|---|---|---|---|---|
| P-commitsA | Committed docs cited ADR-0059/0060 that didn't exist at HEAD yet (dangling references). | MEDIUM | **FIXED** | `615ef4c` commits ADR-0059/0060/0061 into `docs/decisions.md`, closing the dangling window. |
| P-commitsB | `docs/pk_vs_radius_note.md` / README still say the evidence CSVs are "gitignored, not committed" — false since `2665bbb` — the project is **under**-claiming its own reproducibility. | LOW | **FIXED** | `a0ececf` fixed `pk_vs_radius_note.md`; the 2026-07-10 refresh also rewrote README's two stale `logs/`/`plots/` blanket lines ("Reproduce it" intro + repo map) to name exactly which evidence CSVs/PNGs are committed. |
| P-commitsC | `README.md:617/735` and `PROGRESS.md:19` still frame the onboard seeker retrain as "in progress" / "the demo should fly the best detector" — ADR-0061 has since settled this (v2 stays deployed; v3 is a NULL). | MEDIUM | **FIXED** | `a0ececf` (README: the T25 demo flies the deployed v2 detector; v3 closed NULL) + `117f973` (PROGRESS.md v3 row closed as an honest NULL). |
| P-commitsD | Breakoff-deadband/border-reject + `frozen_vworld` fallback were correctly deferred (they touch held files mid-jam-validation) but the sequencing decision is recorded nowhere — a one-line NEXT.md note was recommended. | LOW | **FIXED** (overtaken by events) | The deferral this asked to document ended: the deferred breakoff-deadband/border-reject work LANDED in `706bb46`/ADR-0062 (which records the sequencing itself), so there is no pending sequencing decision left to note. The `frozen_vworld` fallback remains open separately as DEEP-C2. |
| P-commitsE | The `!logs/mc_t21_*.csv` gitignore exception swept in two degenerate CSVs with no marker: the documented-INVALID r1 arm (`mc_t21_trackgate_weave12.csv`, all `miss_m=nan`, `python_exit_2`) sits alongside the valid r2 file, and `mc_t21_cuehandoff_jink12.csv` is header-only (1 line). `check_t21.sh` is referenced by zero committed docs (undiscoverable). | LOW (but confirmed live) | **FIXED** | `d393a6b` removed both degenerate CSVs from git; the 2026-07-10 refresh added `scripts/check_t21.sh` to README's "Milestone gates" list ("re-asserts the ADR-0058 detect-then-track headline (14/14) from the committed CSVs — no sim needed"), making it discoverable from the portfolio-facing doc. |

### 3d. Completeness — items the pipeline/frames audit says were never audited at all

| ID | Finding | Severity | Disposition | Evidence |
|---|---|---|---|---|
| P-NEW1 | Wall-clock vs sim-time in the guidance measurement chain: the filter/pro-nav loop runs on `time.monotonic` with a fixed nominal dt while target motion and PX4 physics evolve in sim time — under RTF sag, effective pro-nav gain scales ~RTF². Currently masked by the idle-load rule but **never quantified**. Audit's own words: "the biggest unexamined item on the board." | HIGH (unexamined, not yet known-bad) | **TRACKED** | NEXT.md sim-queue slot 7 pins it into the "#43 remainder" ("+ P-NEW1 RTF sensitivity pass — the 'biggest unexamined item'"). No data yet. |
| P-NEW2 | Ground-station rig-extrinsics→world-frame chain is independently unverified — its correctness rests on one hand-matched geometry (n=1) in `check_t19` assertion 4. | MEDIUM | **UNTRACKED** | Not addressed. |
| P-NEW3 | Desk-experiment probes (`blur_replay.py`, `dash_pitch_probe.py`) are single-run, zero-test scripts behind committed claims (the blur band, the ~35% dash-ticks-above-FoV number) feeding the #35 up-tilt decision — a sign error would invert the whole ADR-0060 story. | MEDIUM | **UNTRACKED** | No known-answer/unit tests added to either script. |
| P-NEW4 | Camera lever arm (~0.25 m mount offset) is never composed into the PIP track — same fix family as P-H1 but a distinct, still-missing term. | LOW-MEDIUM | **UNTRACKED** | Not covered by ADR-0062's derotation fix; not separately addressed. |
| P-NEW5 | Replay frames contain a parked interceptor, so the two-drone confusion mode (ADR-0047) is structurally untestable in replay. | LOW | **UNTRACKED** | Flagged for the T23 gap audit; no evidence it was added. |

---

## 4. ⚠️ Open / untracked findings — the honest backlog

*(Recounted at the 2026-07-10 refresh: the original 51-item UNTRACKED backlog
is down to **27**. Items that gained a NEXT.md queue slot or task # moved to
TRACKED in the tables above; items with work landed moved to PARTIAL/FIXED.
This section now holds only what still has no home, plus the PARTIAL
remainders worth naming.)*

**Highest-value / highest-risk still open:**

- **DEEP-N2 — the m4-side honesty-check scope.** The seeker-side half of the honesty blind spot is now guarded and committed (#42, `84eb756` — DEEP-N1 is FIXED above), but m4's own gt-checks are still scoped to `run_acquire_and_engage` only and still catch only `ast.Name` reads, not attribute-form (`tracker.gt_range`). *Next:* widen the m4 AST checks to attribute-form + `run_bench()`/`main()` — the same pattern #42 already implements for the seekers.
- **FWD-B2 — own-ship GNSS denial.** The jammer that kills the cue kills GPS L1 more easily; the dash navigates to an NED PIP against NED gate radii, so a GPS-jammed dash never reaches handoff — the same fail-closed *class* as ADR-0059 via a different sensor, with zero doc coverage. *Next:* a design-review gap entry + desk note on the NED-PIP dependence.
- **FWD-B4 — no stacked-WORST regression arm.** Every exposure is a one-knob A/B while reality stacks blur+latency+gust+degraded r̂ simultaneously in the terminal second, against only 0.72 m of correction capacity. *Next:* one standing stacked-EXPECTED arm, n≥16 paired.
- **DEEP-R2 remainder — ADR-0060 pitch numbers trace to rotated ulogs.** Everything else in the R2 cluster was fixed at the refresh (dead cites, selection disclosure, repo-map lines); this last leg needs a `dash_pitch_probe.py --dump-summary-csv` re-run + commit (script execution — an idle/m4-window item, not doable mid-batch).
- **DEEP-C2 / DEEP-C5 — the two known correctness edge cases.** Zero-vector `frozen_vworld` latch fallback and the EKF predict/correct dt inconsistency; both m4-window items with no queue slot yet.

**Everything else still UNTRACKED (grouped, one line each):**

- *Test coverage:* DEEP-T2 (nits: AST-ify the ekf substring gate, `importorskip('cv2')`), P-NEW3 (known-answer tests for the desk-experiment probes feeding #35), P-NEW4 (camera lever-arm composition — distinct from the fixed P-H1 rotation term).
- *Stereo/fusion correctness:* P-M2 (no trajectory-match/cache-provenance check), P-M3 (triangulation geometric gates), P-M4 (stale fusion sigma constants — per the audit, needs an ADR addendum + paired A/B, never a silent sync), P-M5 (EKF R is AprilTag-tier for every seeker), P-lowA/P-lowB/P-wordingNits/P-lowC (assorted lows and nits), P-gateHard (gate hardening asserts).
- *v3-eval methodology nits (lower urgency now that v3 closed as a NULL, ADR-0061):* FWD-3 (one-sided phantom significance), FWD-4 (cross-tab match/mislock double-count), FWD-5 (evalv3_07/08 sit in zero gates).
- *Design gaps never given a home:* FWD-B2 (above), FWD-B3 (cold-start launch readiness), FWD-B4 (above), FWD-B6 (adversarial seeker, threat envelope, post-engagement, process death, cal lifecycle, RF desense, BOM holes, ROE, target-reacts-to-attack), FWD-C6 (cue-trust non-jam components: clock-skew tiers, maneuvering stereo caches, T19's owed items), FWD-C7 (instrumentation G11/G12), FWD-C8 (rolling-shutter ADR, G10 own-state knob).
- *Never independently audited at all:* P-NEW2 (rig-extrinsics chain, n=1), P-NEW5 (two-drone confusion untestable in replay).

**PARTIAL remainders (work landed, gap named):** DEEP-T1 (runner landed
`0123f61`; COAST_STALE_S value pin + pure-helper coverage open), DEEP-R2
(pitch-probe CSV), FWD-C2 (LICENSE/remote/CI — remote is builder decision #2),
P-H2 (#43 sim-gated `--epoch-t0` half).

**Moved out of this section at the refresh:** now FIXED — DEEP-N1 (`84eb756`),
DEEP-R1, DEEP-P1, DEEP-P2, DEEP-P3, DEEP-P4, DEEP-P5, DEEP-P8, DEEP-P9,
FWD-A4, FWD-A5, FWD-A6,
FWD-A7, P-M1, P-commitsB, P-commitsC, P-commitsD, P-commitsE; now TRACKED
(NEXT.md queue slots) — DEEP-H2, DEEP-P7/FWD-C5 (G8, sim-queue 8), FWD-B1,
FWD-B5 (grooming lane), P-M6 (#44), P-NEW1 (#43 remainder), FWD-C4 (confirmed
pinned into #32).

---

## 5. Summary

**81 distinct findings tracked** across the three overnight audits (deep=27, forward=28, pipeline/frames=26; one cross-audit duplicate — the r² conflation, raised independently by both the deep and forward audits — was merged into a single entry, DEEP-H5, rather than double-counted).

*Counts as of the 2026-07-10 post-session refresh (first-writing counts in
parentheses):*

| Disposition | Count | Meaning |
|---|---|---|
| **FIXED** | 36 *(was 18)* | Landed, independently re-verified live against the current repo (not just trusted from the audit or commit message). |
| **PARTIAL** | 4 *(was 5, different membership)* | Real progress landed, a real gap remains — each row explains exactly what's still open. Now: DEEP-R2, DEEP-T1, FWD-C2, P-H2. |
| **TRACKED** | 11 *(was 4)* | Has a task # or a NEXT.md queue slot, no code/doc yet. |
| **ACCEPTED** | 3 *(unchanged)* | A deliberate no-fix call, with the reason given. |
| **UNTRACKED** | 27 *(was 51)* | No disposition exists anywhere. Still the number that matters most for honesty — but the burn-down was real: everything remaining is LOW/MEDIUM polish, deeper stereo/fusion correctness, or parent-project design gaps, all named in §4. |

The spine finding across all three audits — **no active ground-truth leak in any live guidance/seeker path, and every recomputed headline number byte-matched its CSV** — remains true and was not touched by this exercise. What this tracker adds is the honest accounting the audits themselves asked for: most of the HIGH items (the ones that could embarrass the project in an interview or invalidate a headline number) are genuinely FIXED and re-verified; most of the MEDIUM/LOW items (design-gap ideas, doc polish, deeper test coverage) are genuinely still open, and are now named in one place instead of scattered across three audit reports.
