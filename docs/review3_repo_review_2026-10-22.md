# Review 3 — full-repo contradiction & drift review, 2026-10-22

**What ran:** a builder-directed 10-minute fan-out review: 15 Fable-5 forks (judgment lanes:
ledger, assumptions, drift, ADRs, graveyard, provenance, ops, flight code, tests, memory,
deletions, red-team, dashboard, hardware docs) + 16 Opus-4.8/5.5 workers (mechanical doc-cluster
scans). The 8 Sonnet workers requested all bounced `[general_harms]` instantly (logged in
`docs/subagent_safeguard_log.md`); their clusters were reassigned to Opus. ~130 raw findings,
deduplicated below.

**How to read this file:** every item carries a status. Lesson from review 2: a findings doc
with no per-item status can't be told apart from an unexecuted wish list.

> **EXECUTION UPDATE 2026-10-22 PM** — the builder approved ("yes fix all that"; stale docs →
> retired/; lens ruled 70°, ADR-0126; kill day THIS WEEKEND, ADR-0127; headline retired,
> ADR-0128) and the execution fleet ran: 23 rows now DONE (statuses flipped in place).
> Highlights: 11 docs + 5 scripts retired with 28 refs repointed; cue_relay audited with an
> auto-closure guard; engage window terminal-mode-aware (ADR-0129); the pursuit-vs-dash A/B
> FLEW and confirmed (runs/pursuit_vs_dash_20261022/). Still OPEN and queued: airframe-numbers
> consistency cluster, EKF design pass (Q retune, staleness clock, trigger_t), vacuous-verdict
> registry test, pi_capture producer-fed contract tests, graveyard #11/#7 scope notes, ADR-0115
> supersession addendum + phantom ADR-0057 tombstone, 3 revived review2 items, next.md 3a/3b/3d
> block archival, isim documented default fx (385/118° → the delivered-lens value; specs still
> describe the superseded nominal — flagged by the A/B fork), wide-lens aim-tolerance re-earn at
> 70° (ADR-0126 cost), and the Gazebo confirming rung for the A/B.

Statuses: **DONE** (fixed in the 2026-10-22 review-sweep commit), **QUEUED** (on `next.md` /
`builder_queue`), **OPEN** (needs a session), **BUILDER** (his ruling), **INFO** (no action).

---

## A. Money-path / wrong-part risks (highest urgency — purchases ride on these)

| st | finding | evidence |
|---|---|---|
| DONE | `hardware_order_list.md` §0④/§C still said buy the **Hailo-8L** + **YOLO11n** — ADR-0125 ruled **Hailo-8 (26 TOPS, ~$110)** + **yolov8n** ("yolo11n is NOT single-context"). A reader following the order list buys the wrong HAT. Swept this commit. | hardware_order_list.md:53-56,243 vs decisions.md ADR-0125 |
| DONE | Same doc §4 Phase 3 named the **AR0234** as the seeker camera; ADR-0078 keeps the OV9281 (AR0234 = gated upgrade only). Swept. | hardware_order_list.md:387 |
| DONE | **Wide-FOV constraint is silently violated by flying hardware**: constraint `wide-fov` says ≥100° HARD / "never fit/fly a narrow lens" (ADR-0024, graveyard #17), the delivered lens measured **69.4°**, `lens_options_2026-10-22.md` recommends keeping it, AND a 110° Arducam M40180H13L was already ordered 2026-10-12 (contract) that the survey doesn't mention. Needs one ruling: which lens flies, and restate the constraint + graveyard #17 scope formally (new ledger entry `wide-fov-vs-delivered-lens`, builder_queue `lens-fov-ruling`). | project_state constraint `wide-fov`; lens_options:48-59; project_state.json:2340 |
| DONE | `detection_tracking_methods.md` B.2/B.3/§E carries the same stale 8L/YOLO11n/"~30 fps decode" trio; smear table derived at 10.85 px/deg vs measured fx=1104 (~19.3) — 5 ms smear is ~47 px, not 26. Re-derive at the measured lens. | detection_tracking_methods.md:218-260,575-584 |
| DONE | `camera_paper_check.md` verdicts ("118° CLEARS, px/deg penalty ~15%") are measurement-superseded by the 70° delivered lens; no banner. | camera_paper_check.md:34,62 |
| DONE | Money-gate framing stale on 3 surfaces: `build_tab.scope`, `bom_tiers[0].purpose`, tripod protocol §intro still say curve (a) gates the $740 **order** — Tier-2 was ordered 2026-09-22 (ADR-0109); it now gates **kill-attempt flights**. | tripod_test_protocol.md:23; project_state build_tab/bom_tiers |
| DONE | Tag money-gate criterion "decode early enough for handoff (t_go ≥ 0.5 s)" was priced for the retired handoff terminal; flying default is chase pursuit (ADR-0118). Re-earn or restate before tripod day scores against it. | hardware_order_list.md:184 |
| OPEN | Airframe-numbers cluster: worst-case T/W at 950 g is **4.6:1** not "≈5.5:1" (fails the ≥5:1 rule as written); pack mass 245 vs 254 g; 4×45 A full-throttle dash exceeds the pack's 150 A burst unflagged; HTML calculator duplicates constants by hand with no test. | hardware_order_list.md:371,218; airframe_numbers.md:40,140,158 |
| BUILDER | Field-kit gaps in no BOM tier: handheld anemometer (wind limits unenforceable without it), tripod + ball head, soldering iron, Kakute microSD. | tripod_test_protocol.md:114; launch_mechanism_plan.md:340-343 |

## B. Contract & queue drift (the review's core)

| st | finding | evidence |
|---|---|---|
| DONE | next.md waiting table rows #2/#4/#5 sat ⬜ although **ADR-0094 ruled all three 2026-08-10** (demos retired; regulatory CLOSED — he flies his own ~50 acres; 2nd TX = ~$65 purchase item). Swept this commit. | next.md:43-45 vs ADR-0094 |
| DONE | next.md:95 "⚠ NEW RULING NEEDED" (slant under-range) — ruled a month ago: ADR-0118 item 2 froze stock paths as-flown. Swept; ledger `current_truth` rewritten too (it still said "OPEN awaiting ruling" with a dangling queue id). | next.md:95; ledger slant-underrange |
| DONE | next.md:96 "Open: port passage window to real_flight.py" — ported 2026-09-22 (`passage_gate_ok` real_flight.py:695). A fresh session would redo shipped work. Swept. | next.md:96 |
| DONE | next.md:98 cue-error sweep still "Blocked by 1" though item 1 was ADOPTED (ADR-0101); unblocked + re-scoped for the pursuit default. | next.md:98 |
| DONE | next.md:106 3c "measure hover steady-state error" — answered in flight 2026-09-16 (−0.022/−0.023 m, loop tracks setpoint). Closed. | next.md:100,106 |
| DONE | next.md:138 "Kakute H7 still in transit" — all target parts are `status: built` in the contract build tab (photo 2026-10-17). Swept. | project_state build_tab |
| DONE | `hardware` stage `active` still said "Hailo AI HAT+ DEFERRED" vs ADR-0125 queued purchase + passing dry-run. Rewritten. | project_state stages[hardware] |
| DONE | **progress.md frozen at 2026-08-10** — no roll-up rows for the entire Sept–Oct arc (ADR-0100/0101/0103/0105/0117/0118, isim, DFC dry-run, recorder gate, bench bring-up), plus 6 dead doc links and rows citing retired gates (check_s1/s2, t16/17/19). Add rows or banner it "rolled up through 2026-08-10". | progress.md:12,19,26-32 |
| QUEUED | 5 builder questions buried in item prose, absent from the waiting table: RealFlightSM re-approach; ADR-0113/0115 hot-approach trade; stamped-age latency adoption (now in builder_queue); money-gate closing-speed pricing; rehearsal-breakoff unblock condition. | next.md:61,76-80,95,109 |
| OPEN | next.md's own line-7 rule violated: superseded blocks 1b/3a-original/3b/3d still in place instead of archived. Partially swept (1b removed); 3-block cleanup remains. | next.md:97,102-107 |
| INFO | Ledger spot-checks clean: seeker-upgrades retraction, rbcd-sub1m, residual-echoes all verified fixed in place; drift gate green; README carries no refuted claims. | ledger fork |
| OPEN | Ledger entry `terminal-vertical-channel-decided-not-built` (the one OPEN entry, since 2026-09-09) is stale against ADR-0118: `pursuit_terminal.py` now has a camera-driven vertical path (:189-211,246-255). Re-adjudicate/narrow it (which paths still lack vertical — stock/tag?). | flight/pursuit_terminal.py |
| OPEN | Contract append-pressure relapse in validator-exempt fields: `stages[terminal].note` = **8,746 chars**, ~30 fields >700 chars, dated `[20..]` inserts in build_plan/bom_tiers/build_tab. Extend caps/no-dated-insert checks to notes, build_plan.summary, current_truth. | render audit fork |
| OPEN | board_snapshot.json fetched 2026-09-23 — Sheet 3 shows month-old board content; drift check verifies embed sync, not freshness. Re-run fetch_board.py + add staleness warning. | docs/board_snapshot.json |
| DONE | review2 findings doc has no per-item status; 3 of its decided mitigations never landed (`mc_row.py`, `pool_arms.py`, `test_contract_code_consistency.py`), 2 cite never-created test files. Add DONE/OPEN column + close or re-decide the 3. | review2_silent_failure_findings.md:250,828,1134 |

## C. Honesty boundary & provenance

| st | finding | evidence |
|---|---|---|
| QUEUED | **New given-perfect input unregistered**: the latency fix feeds the KF the TRUE measurement age — exact in sim, bench-unverified on the Pi. Register `kf-measurement-age-known` (given-perfect; test = stamp-vs-truth bench) and label the "even 120 ms flies" verdict best-case. | assumptions fork #1 |
| OPEN | Pk@2.8 m 95.0% tile: cited CSVs hold 64/72 flights (add `mc_fixA_on_weave.csv` to provenance — number itself re-verified correct); its note "72/72 clean" collides with the CSVs' own `clean` column (27/64 — different meaning); tile's plain text omits the perfect-cue best-case label its own defeater entries require. | key_numbers fork #1,2 |
| OPEN | `trust: "bench"` on paper derivations: T/W ≈7.5, hover ≈10 min, $740 BOM — all calculated, not measured; contradicts the register's `unmeasured` grading. Add a `derivation` trust grade. | key_numbers fork #3,4 |
| OPEN | `sim-camera-photometrics` not in the assumptions register though pointing recall headlines rest on sim rendering; `sim_harness` is the only stage with no `consumes`. | assumptions fork #2,3 |
| OPEN | `pi5-compute` constraint quotes the 2026-07-26 Pi bench (96.6/6.09 fps) with no run artifact in-repo (bench-evidence rule); nearest artifact is pi_nn_bench_20261021 (different measurement). Pull or repoint. | constraints |
| OPEN | approach-recall provenance path dangling (`approach_recall.py` → `scripts/seeker/approach_recall.py`); `narrative` block self-marked SUPERSEDED (as_of 2026-07-24) still sits as a primary field quoting stale numbers. | key_numbers fork #5,6 |
| BUILDER | **AGPL question never closed**: seeker docs say "prefer MIT/Apache over AGPL YOLO for a public repo"; repo went public 2026-08-29 with a YOLO-lineage deployed net; `license_notice_weights.md` names the wrong weights file and still reads "private-repo phase"; weights/LICENSES.md lacks the fine-tuned rows. (No weights are git-tracked, so nothing is distributed today.) | seeker_design_brief.md:323; license_notice_weights.md:12-58 |
| OPEN | README nits: "~60%" tilt+cant recovery is the R2 rung (worst-tier = 48–50%); "140+ decisions" counts 124 unique ADRs. | README.md:74-76,109 |
| OPEN | LinkedIn draft quotes pre-tag-realism ideal numbers (50/50, 0.068 m) with no perfect-cue/no-wind label, and its chosen clip ("camera-guided 3 m/s, 0.238 m") needs target-type + source-run captioning per the publish runbook rule. | linkedin_copy_draft.md:35-57,79-81 |

## D. Code & test gaps

| st | finding | evidence |
|---|---|---|
| DONE | **`flight/cue_relay.py` escapes the AST honesty audit** — it seeds the live terminal's belief but is in neither `_AUDITED_MODULES` nor the must-list; exact recurrence of the seeker_loop escape. Add it + port the auto-import-closure pattern from test_honesty_seekers.py:322 so the list can't rot again. | flight/tests/test_real_flight.py:1114 |
| OPEN | **Cue staleness is never enforced**: `max_staleness_s` only fires if a caller passes `now_t`; no caller does (real_flight.py:3120 omits it deliberately; no ground-side code does it either); cue_relay_plan contradicts itself on which clock stamps `t`; cue never extrapolated to GO though the error budget assumes it. One design pass: pick the clock, wire the gate or document unenforced, decide trigger_t. | flight/cue_relay.py:291,695; cue_relay_plan.md:143,320 |
| DONE | `engage_max_s` default 12.0 s is fly-by-sized; pursuit chases measured 6–13 s to contact and ADR-0105 item (b) (raise/auto-size it) was never closed by the ADR-0118 swap — a default-config real chase can time out mid-engagement (mitigation = a startup warning only). | real_flight.py:265,3294 |
| OPEN | coded_dash stage's claimed active config not reproducible from the repo: "+5° aim trim" and "−0.207 alt offset" default to 0.0 and no script/config pins them — contract prose only. Pin in coded_dash_arm.sh or reword the stage. | m4_intercept.py:2321,2601 |
| OPEN | EKF: Q=64 (8 m/s² PSD) is the documented cause of the 8/8→2/8 regression and still the default ("untuned"); EKF range floor 0.05 m vs fusion path 0.3 m unexplained; no late-measurement handling though the brief budgets 0.12 s delay. | ekf_tracker.py:134,164-174; fusion_capstone_design.md:22 |
| DONE | `check_t21.sh` permanently exits 1 (its analyzer retired to retired/scripts in 3880153); run-interceptor-sim skill still points at it. Retire or repoint. | scripts/check_t21.sh:32 |
| OPEN | Vacuous-verdict + no-new-emitter guard is per-emitter; add a registry-style completeness test for verdict-bearing scripts. Producer-fed contract tests missing for pi_capture→{camera_session_summary, apriltag_decode_summary}. | tests/test_no_vacuous_verdicts.py |
| DONE | real_flight.py:28 docstring still says "NO cue" above the built --cue-json path; `--alt-ref-offset-m` help quotes a different datum/sign than ADR-0100's value (carry-the-datum rule). | real_flight.py:28; m4_intercept.py:1872 |
| INFO | Clean: all 12 check_*.sh fail closed; no orphan/shadowed tests; skip reasons all valid; estimator.py vs ekf_tracker.py are different designs by intent, not drifted copies; all fork spot-checks of wiring (pursuit default, accel lead, streak=5, cue loading) verified live. | scripts/tests forks |

## E. Doc supersession sweeps (banner/strike work, batchable in one session)

| st | finding |
|---|---|
| DONE | `seeker_upgrades.md` §lens still recommends the 40–60° narrow lens + RTK datum (both graveyarded) — live resurrection bait beside the open lens decision; the "2nd wide-FOV camera WITHDRAWN" ruling lives only in its prose (should be a graveyard entry). |
| OPEN | Graveyard scope amendments: #11 (APN) — delimit what stays dead vs the new FOV-constrained-PN idea in hybrid_guidance_notes; #7 — one-line scope fix vs ADR-0124/0125 resolution finding. |
| DONE | `real_build_coded_dash.md` — no banner though pursuit (no sprint/acquire gate) superseded its operating model; internal tilt contradiction (:129 REJECTED vs :200 "dominant lever"). |
| DONE | `transfer_bet_kill_test.md` §6/§7 still fly detect-then-track/markerless_loop vs §0's NN-every-frame; `nn_seeker_program.md` "detect-then-track (adopted)" vs contract "no visual tracker" — decide which is true for the Hailo pipeline and make both surfaces agree. |
| DONE | Dangling-outcome docs needing banners: pursuit_port (items 4/5 actually closed), new_sim_plan (no closing status; A2 ≥90% bar never declared met/waived; 2k-line cap breached at 5.8k unrecorded; grid5.py pointer uncommitted; its deletions ruling — six dead switches + archive guidance_lab.py — never executed), scoring_fix_plan (banner present but stale body numbers unstruck), pointing_prereg:238 unsourced "0.2–0.25 m", inview_probe_results (pointing-verdict superseded), quad_target_retrain, seeker_v3_dataset_plan (closed-NULL), seeker_v2_plan, eval_s-mono, nn_transfer_plan (model choice superseded by ADR-0125), camera_paper_check, perception_design, compute_setup §7, hailo_toolchain_runbook (DFC 3.34 + 22.04-container + answered questions + dead NMS-json lead), pi_emulation_check vs Trixie (field_bringup still says flash Bookworm). |
| DONE | Field docs: regulatory ruling never swept into tripod protocol §1/§9 + kill_day §1.4/§9 (both still call it a blocker — the ruling doc claims edits that were never made); field_bringup §4 pursuit verdict pre-dates ADR-0118; launch_mechanism_plan §5/L1 done-but-unmarked, ±3°→±2.4° stragglers at L61/L362, legal bullets uncswept; tripod §4.2b qd=1.0 bullet vs ADR-0082. |
| OPEN | ADR hygiene: ADR-0115 needs "superseded by 0120" addendum; phantom ADR-0057 cited 6× but exists nowhere (tombstone or recover); ADR-0071 has no heading; 0121–0123 id/date inversion (next free: **ADR-0126**); ADR-0106's "ops.md is stale" flag is itself stale; ADR-0108 adaptive-speed re-decide (its stated blocker passed). |
| DONE | Broken-path sweep: terminal_diagnosis.md cited by 3 live docs (retired); phase2/stage0 plans cited by 3; audit_targets.md 5 dead paths; progress.md 6; archives need a one-line "paths are historical" banner; bootstrap.sh heredoc names 2 nonexistent docs. |

## F. Ops, routing & memory hygiene

| st | finding |
|---|---|
| DONE | Safeguard log updated: forks now 17/17 clean on repo-attached briefs; Sonnet 8/8 instant bounce (reconfirmed); one opus48 spawn was served `claude-fable-5` and bounced — verify opus48 pinning per spawn. |
| DONE | **ops.md bullet 0 needs the rewrite its own rule demands**: un-struck "all subagent work goes through sonnet-worker" line contradicts the 2026-10-14 entry above it; bullet 3/4 routing claims unmarked-superseded; bullet 4 still says "NO Opus 4.8" though opus48-worker is a sanctioned lane (2026-09-22 hook amendment). Rewrite as one dated routing table: forks → opus48/opus55 → head-inline. |
| DONE | `.claude/SESSION-PROMPT.md` (self-labeled current kickoff, 3 months stale) boots a session on the dead `model: fable` lane — regenerate or banner-deprecate. Agent files: opus5-worker.md advertises the stale "85% fewer interventions" AND instructs orient-from-contract (the known trip pattern); sonnet-worker.md says "use PROACTIVELY"; council-member.md grades against the dead sim-only mission. |
| DONE | Memory: expired overnight-standing-order deleted; subagent-routing memory updated (fork lane verified, sonnet dead). |
| DONE | Memory: `model-orchestration-policy.md` is a 2,000-word append-log carrying multiple refuted directives — rewrite as short current truth; `opus5-fable-only-routing.md` delete (superseded twice); `token-burn-window` pointer stale (+ window expires ~2026-10-23); `session-ownership.md` orphan (on disk, unindexed). |

## G. Deletions & scope (PM duty: deleting is a deliverable)

| st | finding |
|---|---|
| DONE | **deletion_candidates_2026-10-22 Tier 1 (7 docs) + Tier 3 (logs policy, ~1–1.5 GB reclaim) + the held t25-video memory** — the scout ran, Tier 2 executed, but the decision request reached no queue surface (the exact ADR-0076 evaporation class). Now in the waiting table. |
| OPEN | Tier-4 dead levers: closing ADR for pip_aim (delete code+tests); "retired, do not tune" marker on range_fusion. Annotate the scout doc rows DONE/PENDING. New candidates: rehearsal spot-check prereg (concluded, 0 inbound refs), audit_targets.md, 4 orphan scripts (commit_sprint_ab, blur_replay, linkedin_fast_cut, actuation_error_budget — check the last one's numbers didn't feed an ADR), tests/fixtures/rig_capture_min (orphaned by 3880153) + stale fixtures README, launch_mechanism_options (mark superseded-by-plan), 5 unmarked sim-era docs (seeker_upgrades, pi_emulation_check, real_data_pipeline, transfer_bet_kill_test, inview_probe_results). |
| OPEN | publish_cleanup_work_order: items 1,2,5,6,7 open with no status; 10 stale origin/claude/* branches (1 already merged); `build_demo.py` pointer dead; the doc says to delete itself when done. |

## H. Red-team verdict (PM duty #2 — the strongest case against the current plan)

The queue's engineering energy flows into the NN/Hailo lane (end-state seeker) while the ruled
first kill flies the AprilTag — and the camera has still never beaten a well-aimed blind dash
(claim last tested BEFORE the pursuit port; a paired-seed pursuit-vs-aimed-dash A/B on the
current config would retire or re-confirm the architecture's premise — pre-register it first).
Meanwhile the kill path's longest-lead items (site logistics are RULED closed per ADR-0094 — the
red-team fork flagged this before that verification; what remains real is target-drone
flash/maiden and tripod day) sit at the bottom of the file. Verdict: direction defensible
(overnight capacity can only do sim/NN/doc work; ADR-0125 was a builder ruling), but the
builder-attention queue was mis-ordered and the stale header made him prioritize off a
superseded picture — both addressed in this sweep. Timebox the NN lane: state explicitly what
tripod day and the tag first-kill need from it (likely nothing).

---
*Compiled from 31 completed agent reports, 2026-10-22. Raw per-agent outputs are in the session
transcript; this file is the durable register. Items marked DONE landed in the same commit as
this file.*
