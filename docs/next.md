# NEXT — the live queue

> **Canonical state → [`project_state.json`](project_state.json)** · view: [dashboard](dashboard.html)
> **Task board → https://github.com/users/Swampyemerson/projects/1** (issues in `Swampyemerson/interceptor-sim`)
> **History → [`next_archive.md`](next_archive.md)** — the full 883-line queue as it stood 2026-07-26, verbatim. Reasoning lives in [`decisions.md`](decisions.md) (the ADRs).
>
> *Rewritten 2026-07-26: this file had grown to 883 lines of stacked superseded banners — the "too much text" problem in miniature. It is now the SHORT live queue only. **Rule: items live here or on the board, not both; superseded blocks go to the archive the same turn they are superseded, never left in place with a strikethrough.***

## Where the project is

**FIELD INTERCEPTION DAY IS THIS WEEKEND (builder ruling 2026-10-22, ADR-0127).** Tripod day is
descoped as a separate gate — its decode measurement folds into kill-day setup or comes from the
attempt's own logs. Nothing goes in front of the weekend except the bench SAFETY gates (hours,
not days). The three same-day rulings: the **~70° lens flies** (ADR-0126 — px/deg range beats
the old wide-FOV rule; the 110° lens in transit becomes the calibrated spare), the old
"camera never beat a well-aimed dash" headline is **retired as stale** (ADR-0128 — pre-pursuit;
a pre-registered pursuit-vs-dash cue-error A/B is running in isim to earn its replacement), and
**stale docs move to `retired/`** (executing).

Standing context: pursuit-only is the flying default (ADR-0103/0118, Gazebo gate median
0.343 m); target-drone parts all built into the stack (2026-10-17); Tier-2 interceptor parts
ordered 2026-09-22; the NN/Hailo lane (ADR-0125) runs in parallel and **nothing in it blocks
the weekend**. Full drift register:
[`review3_repo_review_2026-10-22.md`](review3_repo_review_2026-10-22.md).

## KILL DAY — the weekend plan (2026-10-24/26)

> **OUTCOME (2026-10-25): the scored attempt moved out ~a week; the weekend delivered the
> interceptor's AIRWORTHINESS instead** — maiden flown after a 180° board-rotation root-cause
> (`SENS_BOARD_ROT=4`), RC chain + failsafes verified, multi-minute Position flight. Still in
> front of an attempt: an inner-loop hover wobble (wiring-class suspects queued), the target's
> maiden + box legs, the 2nd TX (one-TX kill contention, ADR-0089), a working interceptor flight
> recorder (FC microSD slot dead), and the camera-mount/calibration/L0–L2 rungs. Live detail:
> `project_state.json` plain_log 2026-10-25. The ladder below remains the checklist for the
> rescheduled attempt.

**Bench ladder first (your hands, ~hours total — these are the only remaining gates):**
1. ⬜ **tgt-01/02** — flash the Kakute H7 (board already in the stack), prove `.BIN` logging
   end-to-end. The dual-GPS logs ARE the scoring instrument (`field_score.py`) — no logs, no
   measured result.
2. ⬜ **Smoke-stopper first power-up, props OFF** (tgt-04 gate; stopper in hand), motor
   directions, **kill switch verified on BOTH aircraft** (the 2nd TX is still unbought — see
   the table below; one TX + Model Match is the fallback, but then target and interceptor
   cannot be armed independently).
3. ⬜ **Calibrate the 70° lens as the flying config** (ADR-0126): checkerboard → RMS ≤ 1 px →
   `flight/camera.py` intrinsics; then a desk AprilTag decode check at distance for a first
   real decode-range datum.
4. ⬜ **Cue-relay end-to-end on the bench** (`--cue-json` path latched at trigger) + a SITL
   pass of `check_real_flight_sitl.sh` / `check_deploy_sitl.sh` on the final flying config.
5. ⬜ **Target maiden + AUTO box legs** at the field before any intercept pass
   (`dash-envelope-on-real-fc` can ride along).
**Buys that must land by Friday:** handheld anemometer (wind limits are unenforceable without
it), tripod + ball head, FC High-Endurance microSD, soldering consumables, 2nd RadioMaster
Pocket (~$65, ADR-0089/0094) if same-day pickup is possible.
**On the day:** `docs/kill_day_protocol.md` is the operative checklist (swept current
2026-10-22). **Plan the launch cue to stay inside ~10° of truth** — the 2026-10-22 isim A/B
shows pursuit converts 6–12/16 through 10° of cue error and only ~half at 20° (GPS fix quality
+ aim procedure are what buy that). Score from dual-GPS logs; pull `runs/` + `sessions/` off
the Pi before leaving the field. Decode-range spot-check rides along if time (ADR-0127: it no
longer gates).
**My side before Saturday (running now):** pursuit-vs-dash cue-error evidence (isim,
pre-registered), flight-code weekend fixes (engage window, honesty-audit closure), repo sweep
to `retired/`, suite green.

**The honest headline:** aiming the launch correctly matters more than the camera does. Sub-metre
interception is real at 10 mph (8/8 flights, median 0.73 m) but it is **ballistic** — camera off.
The camera has not beaten a well-aimed blind dash at any speed or aim error tested.
*(Caveat added 2026-10-22: this claim was last tested before the pursuit port; a paired-seed
pursuit-vs-aimed-dash A/B on the current config is queued below to retire or re-confirm it.)*
**New 2026-09-16 (ADR-0100):** the vehicle had been flying ~0.4 m ABOVE the target — its height
scale starts at its landing gear, the target's at the ground. With the two lined up, sprint-only
flights inside the 0.35 m contact radius went **2/16 → 13/16** (paired, two fresh seeds,
pre-registered) — **and only while the launch aim is within about ±1° of optimum** (seven-point
curve, two seeds: ~0.1 m of miss per degree, 0/8 inside at 5° off). That is a **best-case upper bound, not a kill claim**: camera off, and the sim
knows the target's path AND height exactly. Without the fix the best config was 3/16 logged,
5/16 interpolated (`docs/rescore_2026-08-10.md`, which retracted an interim 12/16 claim). The
launch-aim and target-height givens are both declared in the assumptions register; the cue-error
sweep and a target-height-error sweep are the deliverables that replace them.

**Overnight 2026-09-17, in one paragraph.** The camera story turned out to be three walls hiding
each other. (1) The seeker was almost never shown the target — nose-down pitch puts it off the
top of the picture and the lead course puts it 40–75° off the nose; fixed in sim (central on
2% → 99% of frames, really recognised on 81%). (2) Shown the target, the camera takes over at
once and the terminal flies **2–5× worse than the sprint it replaces** (3.3–3.6 m vs 0.60 m): it
was sized for a slower target and a last-metres takeover, and a closing-speed floor did not save
it (registered null). (3) Whether any camera terminal beats a well-aimed sprint is now, for the
first time, testable — and is a redesign decision, queued for the builder. Details:
`docs/pointing_prereg.md`, `docs/scoring_fix_plan.md` §5b–5e.

## Waiting on the builder

| | what | where |
|---|---|---|
| ✅ | **Launch-aim cue** — RULED 2026-07-26: declare it a project constraint; real interceptor launches off a GPS-derived cue **latched at trigger, link then dies**; inject error to find the tolerance | [#1](https://github.com/Swampyemerson/interceptor-sim/issues/1) |
| ✅ | ~~Stale demo assets~~ — RULED ADR-0094 (2026-08-10): all five retired; `demo_out/t25/` kept. New footage = the CINEMA pipeline's 12 m/s video, awaiting your review | [#2](https://github.com/Swampyemerson/interceptor-sim/issues/2) |
| ✅ | ~~Regulatory/site~~ — CLOSED by your ruling ADR-0094: you fly your own ~50 acres; the capture doc is optional. (Field protocols still called it a blocker — swept per review 3) | [#4](https://github.com/Swampyemerson/interceptor-sim/issues/4) |
| ⬜ | **Buy the 2nd RadioMaster Pocket (~$65)** — ADR-0094 confirmed it is NOT ordered; each aircraft needs its own arm/kill for a deliberate mid-air (ADR-0089) | [#5](https://github.com/Swampyemerson/interceptor-sim/issues/5) |
| ✅ | ~~Lens ruling~~ — RULED 2026-10-22 (ADR-0126): the **~70° lens flies** for px/deg range; constraint rewritten, 110° lens = calibrated spare. Cost recorded: aim-tolerance numbers earned at ~100° are not re-validated at 70° | ADR-0126 |
| ⬜ | **Stamped-age latency mode**: adopt or keep default-OFF (prediction confirmed 1.08×, strict adopt bar missed 3/6 pairs — your call; rec: keep OFF until the Pi stamp bench) | contract `builder_queue` |
| ✅ | ~~Deletions ruling~~ — APPROVED 2026-10-22 ("get rid of those old claims… stale documents go to retired"): Tier-1 + stale-doc retirement EXECUTING. Held: the logs/ retention policy (Tier 3 — deletes unrecoverable run data, needs your explicit yes) | [`deletion_candidates_2026-10-22.md`](deletion_candidates_2026-10-22.md) |
| ⬜ | **Field-kit buys by FRIDAY** (weekend-critical, see Kill Day plan): anemometer, tripod + ball head, FC microSD, 2nd Pocket TX ~$65 | review3 §A / ADR-0127 |
| ⬜ | **Publication checklist** (repo is ALREADY public): rotate the VM password, re-point the launcher, merge the cleanup branch to `main`, set About/topics. MIT license **CONFIRMED 2026-08-19** | [`publish_cleanup_work_order.md`](publish_cleanup_work_order.md) |
| ⬜ | **Order the Hailo-8 AI HAT+ (~$110)** — the DFC dry-run PASSED (single context @640 AND @1024, 2026-10-21); the evidence gate is satisfied, money is yours | [`../runs/dfc_dryrun/README.md`](../runs/dfc_dryrun/README.md) |
| ⬜ | **AOT dataset slice go-ahead** (~30–50 GB, >2 GB rule) for the NN training program | `docs/nn_seeker_program.md` |

## Next work, in order

> **OVERNIGHT QUEUE 2026-10-21→22 (builder standing order ~23:10: "do not stop
> working until 8am — more engineering, more verification, more research;
> subagents per project, Fable 5 not 5.1 for safeguard-touchy work").**
> Compaction-proof copy of the plan; strike items here as they land:
> 1. ✅ DONE (fork, `ed1ed29`+`28026f8`): 1-class compiles in **2 contexts** by
>    TOPOLOGY (both the --classes and NMS-JSON paths); fps cost = day-1 HW
>    benchmark; caveat on the purchase gate (Artifact v115/v117).
> 2. ✅ FLOWN (`eabc8c4`, after a review-caught clock-base fix `1a51984`):
>    prediction CONFIRMED (1.08x baseline, LJ90's 1.2 m tail collapsed); adopt
>    bar missed by its letter (3/6 strict pairs) — recommendation = builder
>    call, default stays OFF. Bonus: true pipeline lag ~80 ms measured.
> 3. ✅ NULL (`fad9ded`): 0.370 vs 0.383 m at n=32 — the n=8 gap was seed luck;
>    8 m default stands; geometry chapter fully closed.
> 4. ✅ Suite ALL GREEN (dev-machine scope) + Fable-fork review: 5 findings,
>    all closed same night (F1 critical clock-base bug fixed BEFORE the arm
>    flew; F2/F3 wording swept; F4 hardened; F5 clean).
> 5. ⬜ **NN program prep needing no downloads/money**: Det-Fly converter dry
>    checks, sim-randomizer banked-target config, single-class export surgery notes.
> 6. ⬜ Housekeeping: contract current at each landing (ritual), push after each.

> **BUILDER DIRECTIVES 2026-09-24 (midday, verbatim intent — queued):**
> 1. ✅ **DONE 2026-09-24 — ADR-0119, registered NULL** (the coast latch opens
>    inside ~1 m while weave misses occur at 2+ m; lever default-OFF, in-tree).
>    *(This block sat stale for a month — swept 2026-10-22.)*
> 2. **Rehearsal break-off toggle** (hardware-preserving practice passes): config-gated
>    mode — converged track + predicted pass inside the contact envelope → veer off at
>    the last safe moment (margin sim-derived from closing speed + both airframes), score
>    the WOULD-HAVE closest approach honestly from the logs, SAFE hover, repeat. Builds
>    on the passage window + SAFE machinery. Build after the estimator diagnosis lands.
> 3. ✅ **Onboard footage recorder — FULLY CLOSED (doc swept 2026-10-17; the work itself
>    landed 2026-10-04 and sat unswept).** Built 2026-09-24 (flight/deploy/frame_recorder.py;
>    14 tests). The two REMAINING items both completed 2026-10-04: (a) the **Pi bench gate
>    RAN** — 900 s live-camera soak with --record on the real Pi 5: **8958 frames written,
>    0 dropped**, no throttling, temps steady (the soak's own fps number is stamped
>    PROVISIONAL/not-gate-eligible — no tag in view and camera delivery sat at the 30 fps
>    default, so "recording costs nothing" is proven at 30 fps delivered with 7.69×
>    compute headroom, not at flight cadence); evidence `runs/recorder_gate_20261004/`
>    (pulled off the Pi 2026-10-17, frames in bench_archive/). (b) the **live picamera2
>    source is wired into run_mavsdk_mission** (commit 1b10547: frame_feed.py adapts the
>    proven seeker_loop sources/detectors into the detect() call site, fail-closed startup
>    refusals, --desk mode that cannot arm; byte-identical without --source).


0. **TERMINAL REDESIGN — RULED: chase only (ADR-0103), port complete through the real CLI (ADR-0105, 2026-09-21).** Pursuit alone replaced the sprint-and-fly-by; `flight/pursuit_terminal.py` is selectable via `real_flight.py --terminal pursuit`, and in pursuit mode GO enters the engagement directly (no sprint, no acquire gate). Open before it is trusted: ~~(a) the 2 s lost-target abort~~ SETTLED 2026-09-21 — the pre-registered A/B came back NULL (byte-identical at 2/4/10/30 s windows; the timer is exonerated, do not spend a ruling on it); (b) ~~the parity gap~~ CLOSED 2026-09-22 — the trace found FOUR coupled port defects (KF covariance Jacobian, capture-instant attitude, yaw-slew seed, slant under-range) and the fixed port now matches/beats the prototype (nominal 24%→100% ≤0.35 m, median 0.068 m; adopt criterion met, head-verified independently; `isim/specs/parity_trace_2026-09-22.md`). Residual: aim20/alt+2 trail the native because RealFlightSM has no re-approach after a missed first pass — a queued builder design question. ~~NEW RULING NEEDED: slant under-range in stock paths~~ RULED (ADR-0118 item 2, 2026-09-24): stock `SeekerGuidance`/`tag_terminal` stay FROZEN as-flown (their validations were earned with the bias baked in); the fix rides only in pursuit; (c) ~~the Gazebo cross-check~~ FLOWN 2026-09-23 (pre-registered, n=8, camera-in-the-loop, `docs/xcheck_gazebo_pursuit_prereg.md`): the shakedown caught parity defect #5 (Phase A yawed at the rendezvous point, freezing the camera off-target when station-keeping; fixed to yaw at the believed target like the prototype, regression-pinned, and the fix closed the isim aim20 cell 70%->100%), then the scored flights FAILED the registered bar (median 1.51 m); the tick-trace attributed the gap, the fixes flew (re-fly #2: median 0.967 m, instruments green, bar still failed), and the residual is now CONFIRMED as the hot approach with a traced side-effect trade queued for the builder (ADR-0113/0115). DEFAULT SWAP EXECUTED 2026-09-24 (ADR-0118): re-fly #4 passed the standing gate (median 0.343 m after the ADR-0117 velocity-wiring fix) and pursuit is now the flying default; stock/tag stay selectable as-flown. NEW 2026-09-22 (builder field-readiness directive, agents running): miss-abort safety (abort→hover/return-home the moment an intercept is missed, + the ADR-0101 passage-window port to real_flight.py) and the GPS cue relay (~3 s of target GPS, ADR-0130 → ground → interceptor, latched at trigger — the piping for the ruled cue constraint, incl. launch climb/altitude profile). Pointing levers remain built and deliberately NOT adopted.
1. **Past-CPA breakoff — NEARLY CLOSED 2026-09-17.** Issue #9 (dropout re-issued a full stop) is fixed and observed fixed on 100+ flights (auditor check (f)). The 5 m range gate FAILED its offline check and did not fly. The own-motion **passage window** (`--breakoff-min-flown-frac 0.8 --breakoff-force-flown-frac 1.3`) met every registered clause on seed 123 (premature/cut-off 0/16, long chases 0/16); **Seed 777 met every clause too — ADOPTED, ADR-0101.** ~~Open: port to real_flight.py~~ PORTED 2026-09-22 (`passage_gate_ok`/`passage_ceiling_hit`, real_flight.py:695/720). Item CLOSED; original 1b text archived (`next_archive.md`, 2026-10-22 section).
2. ✅ **The pursuit-vs-dash cue-error A/B — PRE-REGISTERED AND FLOWN 2026-10-22 (isim), prediction CONFIRMED.** On the operational chase geometry at the delivered-lens FOV (fx≈933), paired seeds n=16/cell, all honest errors on: pursuit median 0.204–0.394 m with 6–12/16 inside 0.35 m through 10° of aim error; the blind dash 0/16 at EVERY error on this geometry (slope 0.091 m/deg, matching the seven-point curve). At 20° pursuit converts only 7/16 — **the derived cue requirement is ~10°**. The old "camera never beat a dash" headline retires as superseded-by-scope (ADR-0128): the dash's 13/16 was head-on Gazebo geometry, not this one. Evidence: `docs/pursuit_vs_dash_prereg_2026-10-22.md` + `runs/pursuit_vs_dash_20261022/`. **Gazebo confirming rung FLOWN same evening (registered amendment): 5° median 0.259 m (6/8 ≤ 0.35), 10° median 0.244 m (8/8 ≤ 0.35) vs the 0° control 0.243 m — through 10° the cue error costs ~nothing camera-in-the-loop (sim optic; the 70°-lens funnel re-earn stays queued per ADR-0126).** Remaining: the 25/30° portfolio arms.
3. **The vertical channel — REPRIORITISED 2026-09-10 after my own measurement was corrected (ADR-0099 second addendum).** The miss is 82% vertical on the adopted config and the terminal has no vertical steering at all; ADR-0085 decided one in July and no code was ever written. **What changed:** the "+0.320 m of dash drift, 66% of the miss, the error is delivered by the dash" finding was measured against a baseline that included the TAKEOFF (altitude reads 0.000 on the ground), and is retracted. Against a settled hover the dash drifts **−0.019 m**. The corrected split is **23% a datum offset present before the dash, 10% dash, 66% after handoff**. So the queue order is now:
   - **MEASURED IN FLIGHT 2026-09-16 (dev machine, check_m4 PASS, n=2 flights of the classic tag scenario — a lead, not a verdict).** In settled hover the camera sits **+0.161 m / +0.204 m above the tag centre** before ENGAGE begins; at CPA the pro-nav flight's miss was 0.384 m = **0.201 m vertical**, 0.327 m horizontal — the hover offset carried all the way through. Hover altitude error is **−0.022 / −0.023 m** (cmd_vd −0.021), NOT the −0.087 m suspected in 3c, so on this config the altitude loop tracks its setpoint. One oddity to chase: `gt_cam_z − alt_m` in hover was 0.184 m on one flight and 0.226 m on the other — a 4 cm flight-to-flight difference in something that should be geometry. **3b is DONE** (commit `ef9e919`: six `att_*` columns, 331/331 rows populated in flight; pitch med −6.7° in ENGAGE, +12° braking). **3a is now an A/B, not a desk edit:** it changes the scenario every baseline was flown on, so pre-register it (prediction: vertical miss at CPA falls by about the hover offset) and fly it paired on the ADOPTED coded-dash config, not just this tag gate.
   - ✅ **3a DONE 2026-09-16 — ADR-0100**: `--alt-ref-offset-m -0.207` adopted on the coded-dash config, 2/16 → 13/16 inside 0.35 m, replicated (docs/vertical_channel_prereg.md §9.13–9.14). **Next vertical levers, in order:** (i) the vehicle still ends ~0.19 m high because it CLIMBS during the dash at −41° to −45° pitch (est. altitude 0.57–0.78 m vs 0.50 m ref; left-to-right worse) — `dash_alt_trim_m` now has real numbers to be derived from; (ii) flight index #4 got worse in BOTH offset arms with a `yaw-check` flag only there — open; (iii) target-height error injection (assumption `target-height-known`). The original 3a text follows for the record.
   - **3a. Reconcile the two altitude datums — DO THIS FIRST, it is the cheapest 23% in the project and needs no arm.** `ALT_REF_M = 0.5` is AGL above the arm point; the target's altitude is world z; nothing reconciles the ~0.22 m the camera sits above ground at rest. One line of scenario setup, no guidance change, and it removes 23% of the vertical error. Desk work.
   - **3b(ii) ANSWERED 2026-09-17 — the wall is POINTING, not recognition.** With attitude logged, `scripts/forensics/inframe_attribution.py` projects the TRUE target into the image on every pre-closest-approach tick of 32 camera flights (projection validated on AprilTag flights: vertical error 4 px, a constant ~7° yaw offset of unknown cause, subtracted). Result: at **8–22 m the target is outside the picture on 86% of ticks (mostly off the TOP — the vehicle is ~43° nose-down — and off the sides), at the edge on 14%, and central on 0 of 858.** When it IS central the seeker really finds it **72% of ticks at 4–8 m and 86% under 4 m** (n = 25 and 42 ticks — small); at the edge, where the seeker masks detections by design, 2–33%. Separately re-confirmed the graveyard entry rather than re-trying it: of 519 fresh pre-closest-approach "detections", ~300 report ~1.5 m and sit nowhere near the target — own-airframe phantoms. **So a pointing lever (up-tilted mount sized to the real dash pitch, or a gimbal) is now an ATTRIBUTED fix, not a guess; what is still untested is in-flight recognition at 8–22 m, because the target has never once been central there.**
   - **3b. Log attitude in the flight CSV** (deep-audit DEEP-R2) — **promoted 2026-09-11; it now gates two separate decisions, not one.** (i) Without it, ~24% of the post-handoff vertical term is un-attributable: `gt_cam_z − alt_m` should be a constant offset and moves +0.087 m hover→CPA, i.e. the camera swinging on its forward boom is being counted as vertical miss. (ii) It is also the ONLY way to decide whether a FOV-holding controller (arXiv:2409.17497's third component) is worth building. Measured on committed logs: in-flight detection is **51.7%** of DASH ticks at 8–22 m (313/606), **4.4%** beyond 22 m (49/1122), and **0 of 361** ENGAGE ticks at 8–22 m — against ~100% STATIC recall at 8–25 m (ADR-0076 add #18k). So the wall is flight-dynamic, and in-flight detection rate — not range accuracy — is what actually caps "acquire farther". But the cause is NOT attributable from these logs: true world-frame target elevation is indistinguishable between detected and undetected DASH ticks at 8–22 m (median −0.6° vs −0.8°), so the discriminator lives in the vehicle's own attitude, motion blur, or aspect, none of which is recorded. **Building a FOV controller before that is building a fix for an unattributed failure** — the pattern this repo keeps retracting.
     - ⚠️ **A correction worth keeping, 2026-09-11.** I first argued FOV holding was low-value because the target sits within 10° of the 50° frame edge on only 1.2% of ENGAGE ticks and LOS rate exceeds the yaw ceiling on 0.4%. Both figures are computed on ticks that HAVE a detection, which is survivorship bias: the ticks where the target left the frame have no detection and were silently excluded from the denominator. The statistic answered the wrong question.
   - ✅ **3c CLOSED 2026-10-22 (was answered in flight 2026-09-16 and never marked):** hover altitude error measured −0.022 / −0.023 m — the loop tracks its setpoint; the −0.087 m suspicion did not reproduce (evidence in the 3a flight block above).
   - **3d. Then the arms.** `dash_alt_trim_m` is a **10%** lever, and on the adopted coded-dash config there is no measured drift to derive it from (no pre-dash hover, so the tool fails closed — that is correct, not a bug). `--dropout-hold-vertical` is registered with a **null** prediction and must fly as a **2×2 with `--terminal-coast-gate`**, never standalone: the far-dropout branch zeroes vertical and horizontal on the same 792 ticks (0 discordant), so a vertical-only arm is not identifiable. Pre-registration: `docs/vertical_channel_prereg.md` §8.
4. ✅ **DONE 2026-09-16 — the gz stub finder is deterministic, and CI's deselect list is down to ONE file.** The `gz`/`mavsdk` stub import finder moved out of `tests/test_rescore_cpa.py` (where it was installed at module scope, so it only rescued files collected LATER in the alphabet — four others silently rode on that) into a new repo-root **`conftest.py`**, which pytest imports before collecting anything. It installs only for roots this interpreter genuinely lacks, is appended so real bindings always win, and never raises. Measured in a from-scratch venv built **without** `--system-site-packages` (gz really absent), `pip install -r requirements.txt`, on a clean `git archive HEAD` tree: **884 → 962 passing** in CI's gz-unavailable branch, 0 failed. `.github/workflows/ci.yml` now deselects only `tests/test_inert_flag_guards.py`, which runs `m4_intercept.py` in a **subprocess** (a child interpreter loads no conftest) — dropping it too puts 10 real failures back. The old ordering-pin test is replaced by one asserting the new property (the four ex-dependents collect **and pass** with `test_rescore_cpa.py` ignored), and it runs everywhere, so one entry came out of `ALLOWED_SKIPS` in `scripts/run_tests.sh`.
5. **Money-gate closing speed — CHECKED on the dev machine 2026-09-16; the alarm does not reproduce, but the real question moved.** `handoff_closing_speed.py --glob 'logs/m4_intercept_pronav_*.csv' --phase CODED_DASH`: last second before handoff **median 8.98 m/s (n=42 flights)** against the gate's 9.0 → ratio 1.00×, "no action". The 17.60 m/s was the retired cue-era two-stage `DASH`, a different config. **Two honest limits:** (i) the denominator shrank hard and is counted, not absorbed — 481 coded-dash files, 407 had no usable segment, 74 measured, 42 with a pre-handoff window; and those 481 mix the speed-ladder rungs (dash median 4.56 m/s says most are slow rungs), so 8.98 is a pooled number, not the adopted config's. (ii) The real-build default is `dash_speed_ms = 16.0` with a `TODO-BUILDER: measure on the airframe`, and the 9 `logs/real_flight*.csv` lack `t_sim`/`gt_range`, so the tool correctly returned UNCERTAIN/VACUOUS on them. **So:** the gate is not mis-priced for what the sim flew, and tripod day itself is a fixed camera with the target passing at 9 m/s, so 9.0 is right *for that test*. What is unpriced is the real interceptor closing at dash + target speed. Gate NOT patched. **Follow-up DONE same day:** the 407 are exactly the coded-dash flights written before 2026-07-25, whose `t_sim` column is blank (they ran on wall time) — buckets sum to 481, the tool is failing closed correctly. That includes the old adopted-config arms, so their closing speed was never measurable. The fresh adopted-config flights (seed 456 control, n=8, sim time) give **12.8 m/s in the last second before closest approach** (11.7–13.4) — neither the gate's 9.0 nor the cue-era 17.6. At 12.8 the real interceptor's streak burn would be 1.42× what 9.0 prices. Tripod day itself is still a 9 m/s pass by a fixed camera, so the gate as a test is unchanged; whether it should price the interceptor's own closing speed is a builder decision.
6a. ✅ **CLOSED 2026-10-17 (evening) — Pi 5 boot-on-power PROVEN on both paths; battery power
   quality measured clean.** The pre-registered cold-cycle experiment ran: **(a) wall PSU →
   boots, (b) battery/BEC → boots**, both with no button press. Per the registration this
   refutes the BEC-ramp theory; the two standby incidents are attributed to **carried-over
   halt/standby state** (a software-halted Pi 5 sits in standby looking dead until the button
   or a TRUE power pull). EEPROM was ruled out on the live board first (no `POWER_OFF_ON_HALT`,
   bootloader 2026-09-25 current). Nothing was changed — boot-on-power works as shipped.
   **Standing rule: always pull Pi power fully (and let it drain); never rely on `halt`.**
   Reopen only if standby recurs from a verified true cold start. Follow-up measured the same
   session (builder reported the LED "dimmer/flickery" on battery): 180 s all-core stress on
   battery — **EXT5V 5.290–5.298 V (≈20 mV sag from 5.317 idle), `throttled=0x0` incl. the
   sticky since-boot bits, clock pinned 2400 MHz, ≤55 °C**. No electrical corroboration of the
   dim-LED observation at the Pi's input — **PARTIALLY WALKED BACK ~20 min later: the pack
   died outright (lights off, total 5 V loss), so the dimness was plausibly the pack sagging,
   invisible on the BEC's regulated side until dropout. Battery-side tests must monitor PACK
   voltage via the FC power module (MAVLink battery_status) alongside EXT5V.** Open residue:
   the flight-length endurance gate (draining pack, camera live, pack-V monitored).
   Evidence + addendum: `runs/bench_20261017_brn05_reverify/pi5_battery_stress_20261017.md`.
6b. ✅ **DONE 2026-10-17 (same evening).** All three stranded dirs pulled off the Pi (213 MB,
   rsync size-verified): small aggregates committed under `runs/` (recorder-gate
   soak.json/verdict/thermals/soak.out + both desk.csv), full frame sets in the gitignored
   `bench_archive/pi_pull_20261017/`. The recorder-gate verdict closed directive #3 above.
   The Pi's clone is synced to current main (was 48 behind; plain HTTPS pull worked).
6. **Bench bring-up — UNBLOCKED, hardware in hand (2026-07-26).** Two independent chains, both fully in hand: the **seeker rig** (`skr-01`→`skr-06`, Pi 5 + OV9281, ends at the calibration gate) and the **brain bench** (`brn-01`→`brn-05`, Pi 5 + 6C Mini + M10, ends at the props-off OFFBOARD gate — the one link the sim never exercised). Packs are written: `scripts/pi_setup/`, `configs/px4_6cmini/`. Pull `skr-03` (exposure ≤1 ms) forward — a null there kills tripod day's motion-blur read.
   - ✅ **`skr-07` DONE remotely 2026-07-26 — and it moved a number the gate turns on.** Sustained on the real Pi 5 through 750 s soaks, tag in frame, **no throttling on any arm**: **96.6 fps** at `quad_decimate=2.0`, **38.2** at 1.0, **CPU-YOLO 6.09 fps**. The seeker had been running at 30 fps because neither `pi_capture.py` nor `seeker_loop.py:PicameraSource` sets picamera2's `FrameDurationLimits` — an inherited default, not the sensor (143 fps) and not the CPU. `R_streak_burn` goes as 1/fps, so this cuts the range burned forming the handoff ~3.2×, and `quad_decimate=1.0` (the range/incidence reclaim lever) is now affordable at 38.2 fps — above the 30 the project assumed at the *coarse* setting. ADR-0090; harness `scripts/seeker/pi_fps_soak.py`; data `runs/skr07_tagged/`.
   - ✅ **The frame-duration cap is WIRED — this line used to say "still open" and was stale (corrected 2026-08-10).** `seeker_loop.py:PicameraSource` pins `FrameDurationLimits` from `--camera-fps`, default `CAMERA_FPS_DEFAULT = 60.0` — exactly ADR-0090's recommendation, not uncapped. It refuses a frame duration shorter than the exposure, and it logs what was **applied** rather than what was intended (an earlier version printed success off `target_fps` alone and a mutation test that deleted the assignment still saw it claim victory). Regression-tested: `flight/tests/test_camera_fps_pinned.py`, 4 passed. Note the **recorder** (`pi_capture.py`) is deliberately separate and still inherits ~30 fps — a live 5-frame check on 2026-08-10 measured `stream_fps=30.799`. That is correct for a bench recorder that pays a per-frame PNG write, but do **not** quote a recorder rate as a seeker cadence.
   - **Still open:** CPU-YOLO at 6.09 fps **confirms** the ~5–10 fps anchor, so the Hailo HAT requirement stands and the frame-rate win does **not** transfer to the markerless path.
7. ~~Tripod day~~ **DESCOPED as a standalone gate 2026-10-22 (ADR-0127)** — folds into kill-day setup or the attempt's own logs; see the Kill Day section at the top. [#6](https://github.com/Swampyemerson/interceptor-sim/issues/6)
8. **Target drone → WEEKEND-CRITICAL, moved into the Kill Day bench ladder (steps 1/2/5 there).** Parts all built into the stack (contract build tab, photo 2026-10-17). | [#7](https://github.com/Swampyemerson/interceptor-sim/issues/7)

## Standing cautions

- **Every first power-up after soldering goes through the smoke stopper, props off** (tgt-04 gate). The stopper is in hand as of 2026-07-26 — the gate is now a step to perform, not a part to wait for.
- **LiPos:** store ~3.8 V/cell, in the bag, never charge unattended.
- **One sim at a time, at idle load.** Gates and batches only when the machine is quiet.
- **A green run is only as good as the machine that produced it** — `docs/verification_environment.md` says what the dev box, CI and a fresh cloud clone each can and cannot verify. Nothing from a cloud session has flown.
- **Pre-register** any arm that could change a belief — prediction, criterion, and what a null means — *before* it flies.
