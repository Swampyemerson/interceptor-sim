### 2026-08-12

The contract now renders a SECOND view: a set of systems-engineering diagrams and traceability tables (MBSE), generated from the same JSON as the dashboard. Nothing on it is hand-drawn.

**So what:** The systems-engineering side of the project is now showable to an interviewer, and a model that disagrees with the build fails the test suite instead of quietly misleading the reader.

*Evidence: scripts/render_mbse.py · docs/mbse.html · scripts/run_tests.sh [3/4]*

### 2026-08-10

A correction on top of a correction. Tonight's headline was that the kill-radius scorer measured to the camera and that fixing it turned 0 of 16 flights into 12 of 16. That was WRONG, and it is now retracted. The '+0.208 m camera offset' it rested on was never a camera offset -- it came from subtracting a height-above-takeoff from a height-above-the-world, which measures the LANDING GEAR. The camera actually sits about 2 mm above the airframe centre.

**So what:** The real correction is small and sideways, not large and vertical: 3 of 16 becomes 5 of 16, from interpolating between samples. So nothing has reliably reached contact range yet -- the honest headline did not change. And the finding this briefly cast doubt on is confirmed: the interceptor flies about 0.37 m LOW, which is now essentially the whole miss, on a vehicle whose targeting math is 2-D horizontal by design.

*Evidence: scripts/rescore_cpa.py - docs/rescore_2026-08-10.md - 168 flights, 36,276 rows, 0 dropped*

### 2026-08-10

Three questions that had been sitting in the Waiting-on-You list are now answered, so the list is EMPTY for the first time. Retire the old demo videos. Regulatory and site: settled — the builder owns the ~50-acre site, so the FRIA-field question never applied. The second transmitter is NOT already ordered, so it becomes something to buy rather than something to check.

**So what:** The board stops asking. Five stale contract lines that still described the site as an open FAA/Remote-ID problem were rewritten to match the ruling, and two more demo cuts were retired than the question named — the honesty fix to the video generator landed 2026-07-25, so EVERY cut rendered on 07-07 carries the misleading 1.5 m caption, not just the two that were flagged.

*Evidence: docs/regulatory_site_capture.md - demo_out/retired_2026-08-10/README.md - ADR-0089 - build_demo.py fix 759e9b0*

### 2026-08-10

The evidence behind two finished bench steps was sitting on ONE microSD card. skr-07's soak results (96.6 / 38.2 / 6.09 fps) and skr-05's session meta (994 us applied exposure, spec met) are quoted all over this contract but had never been copied off the Pi. Both are now in the repo. The Pi's own checkout was also 16 days stale -- including the whole field-day scoring chain -- and carried 461 stray file copies from an old session.

**So what:** A dead SD card would have turned published numbers into unsourced claims. Every stray file was checked against the real repo before deletion, and the 445 MB training set was moved to the path the scripts read, not deleted. The Pi now matches the repo exactly. Camera calibration itself is still not done.

*Evidence: commits fc0f18b + 29ec8bc - runs/skr07_tagged - runs/skr05_smoke_session*

### 2026-07-26

The camera was only ever getting 30 frames a second because of a default nobody set -- not the sensor (capable of 143) and not the Pi (capable of 112). Lifting it gives 96.6 sustained, with a tag in every frame and no overheating.

**So what:** The interceptor burns about three times less distance while the camera locks on, and the finer decode setting is now affordable. This is the first bench number the $740 gate can legitimately use.

*Evidence: ADR-0090 - runs/skr07_tagged/qd2_uncapped - commit f3a9d5e*

### 2026-07-26

Threw away an aim-trim experiment: I ran heavy background work while a measured batch was flying, 3 of 8 flights failed to boot, and my own summary script averaged through the failures.

**So what:** Re-fly at idle with a load guard; the trim question is still open.

*Evidence: commit f49a03a*

### 2026-07-25

Found a scoring flaw that only hurts camera flights: they sometimes quit steering early (a false 'we passed it' trigger the dash-only flights can never fire).

**So what:** The camera-vs-dash verdicts stand, but the SIZE of the camera's deficit is unknown until this is redesigned — a simple threshold fix was measured and cannot work.

*Evidence: commit 8e8eba5 · flight_plan_candidates.md confound section*

### 2026-07-25

Flew the 10 mph test: the pre-programmed dash alone got 8/8 flights under 1 m (median 0.73 m). The camera arm did not beat its dash-only twin (2/8 vs a 6/8 bar). Nothing got inside 0.35 m.

**So what:** Sub-metre is real but it is ballistic — camera off. And sub-metre is still not a kill: 0/16 inside the ram radius.

*Evidence: commit df1b557 · logs/mc_fp_arm**


### 2026-07-25

Physical build started: target frame carbon, Pixhawk 6C Mini, and Pi 5 in hand; frame assembly begun (dry-fit only — no power until the smoke-stopper supplies arrive).

**So what:** The project is now physically moving, not just simulated.

*Evidence: commit e9857df · NEXT.md build note*

### 2026-07-25

Re-flew all camera-vs-dash tests on the repaired terminal (a bug had been freezing its steering). Verdict unchanged: the camera does not earn its handoff at 0, 5, or 15 degrees of aim error.

**So what:** The earlier conclusion survives on a terminal that provably steers — it was not an artifact of the frozen-steering bug.

*Evidence: commit 1b51fb4 · flight_plan_candidates.md re-fly table*

### 2026-07-25

Logged the big loophole in the ledger: the launch aim is computed from the target's exactly-known flight path — a perfect cue no real engagement would provide.

**So what:** Until the cue carries honest error, every 'dash beats camera' result describes a perfect-cue world, not the field.

*Evidence: commit 2a54362 · contradiction launch-aim-derived-from-ground-truth*

## 2026-07-26 (rotated out 2026-08-19)

- **text:** Found a second full-stop bug: on a camera dropout the coded dash re-issues a zero command, braking mid-terminal. Twin of the one fixed yesterday, still live.
- **so_what:** Some of the evidence that 'the camera makes it worse' is this bug, not the camera.
- **evidence:** deep targeting workflow


<!-- overflowed from project_state.json plain_log on 2026-08-29 -->

### 2026-07-26
We were measuring the miss from the camera lens, not the airframe centre -- and the criterion means centre to centre. Re-scored, the adopted setup already reaches contact range on 12 of 16 flights instead of 3.

**So what:** The 'we have never got close enough' headline was a ruler error, not a design failure.

*Evidence:* commit fcb2a0a

### 2026-07-26
My own fix to the miss ruler was wrong too: I corrected the camera's height but not the fact that it also sits forward of the drone's centre and swings as the drone pitches.

**So what:** How many flights actually reach contact range is genuinely unknown -- and the logs do not even record the pitch needed to work it out.

*Evidence:* instrument audit #7


<!-- overflowed from project_state.json plain_log on 2026-09-09 -->

### 2026-07-26
Bench hardware count corrected: the target's flight controller has NOT arrived, but the camera, GPS, cables, cards, tools and safety kit all have -- the contract had several of these still marked in transit.

**So what:** Every bench step is unblocked, so both bring-up gates can be closed now and the day the flight controller lands the only work left is building the target.

*Evidence:* builder at the bench 2026-07-26


<!-- overflowed from project_state.json plain_log on 2026-09-10 -->

### 2026-07-26
Adversarial review of every script run at the bench: 51 defects raised, 48 survived two independent skeptics. The props-off OFFBOARD gate could print PASS without testing anything -- its first setpoint came from a hallucinated detection on a frame with no target, and a single setpoint passed while the line above read 'mean cadence 0.0 Hz'.

**So what:** The gate built to retire the one link the sim never tested would have retired it untested. Now gated on measured cadence, span, gap, and proof that bytes came back over the wire.

*Evidence:* commit 1b89340

- **2026-08-19** — Publication sweep. The repo was de-cluttered for public view: the plaintext VM password line deleted, root files moved under docs/ and .claude/, the README repackaged around the results and the honesty machinery, and the retracted 12-of-16 kill-radius claim corrected on every surface that still carried it. CI had been red since 2026-08-11: the rescore gate test needs the machine-local flight archive and now says so as a named skip instead of failing. *So what:* The repo turned out to be ALREADY PUBLIC, so rotate the VM password first. MIT license: CONFIRMED by the builder 2026-08-19. Still yours: re-point the desktop launcher, merge the cleanup branch, set About/topics, re-shoot the GIF. (commits 7ea21c7, 5c40a96 - docs/publish_cleanup_work_order.md)

- **2026-08-29** — Wind gate flown for the first time. It found two defects in the measurement path before it could answer its own question, then answered it: Gazebo applies the wind force almost exactly (5.259 deg measured vs 5.262 deg derived) — our own driver was cancelling it, because the fix for an earlier bug clears the force on a different topic than it publishes it, with no ordering guarantee. *So what:* The wind arm is PARKED by builder ruling, but the answer is banked and the fix written down, so unparking costs a read rather than a re-derivation. A frozen-instrument bug nearly made the probe condemn working code. (ADR-0098; logs/wind_gate0_20260829T155141Z; logs/wrench_diag_20260829_partial)

- **2026-08-29** — Target flight controller brought up end to end over USB: ArduPilot flashed after a full chip erase, all 44 settings written AND read back to confirm they landed, and the board proved it writes a flight log we can pull off it without touching the SD card. Its own start-up banner names the same firmware build we flashed. *So what:* The first pass reported '23 of 23 written' and looked perfect -- four settings had not actually taken. Only reading them back caught it. The log still has no position track because no GPS is wired, so that gate stays open. (runs/tgt02_bench/summary.json; commit d191d50)

- **2026-09-09** — The builder asked why a late camera adjustment cannot save a slightly-off aim, since he has watched exactly that happen on video. Checked it properly. He is right that nothing in the physics forbids it, and the reason it has never happened here is that the camera terminal has no up-and-down steering at all: it holds a preset height and throws away the target's measured elevation on every single frame. *So what:* The miss we cannot close is 0.37 m vertical against 0.17 m sideways, so the guidance is blind to about 82% of it, and closing the vertical part alone would already be inside the 0.35 m contact radius. A vertical channel was DECIDED in ADR-0085 and never written; it is not in this contract or the queue. So 'the camera does not beat a well-aimed dash' describes the arms we flew, not what a camera can do. (contradiction terminal-vertical-channel-decided-not-built; docs/vertical_channel_analysis.md)

- **2026-09-10** — Measured two things the project had been assuming. The vehicle is off-altitude at the closest point on every single one of the 24 flights whose per-tick data is in the repo, and most of that is altitude it loses or gains during the sprint itself. And the sprint closes on the target at about 17.6 m/s while the purchase gate for the airframe assumes 9. *So what:* The up-and-down error is made during the sprint, so it has to be fixed there -- which is now built and switched off, waiting on a number measured from the right set of flights. The camera terminal has almost no time to fix anything on the AprilTag setup, so my previous answer that it had plenty was wrong and is corrected. And the airframe gate looks about twice as optimistic as it should be, which is worth checking before spending. (ADR-0099; scripts/forensics/{vertical_miss_anatomy,handoff_closing_speed}.py; docs/vertical_channel_prereg.md)

- **2026-09-10** — My own instrument was wrong and it flipped a conclusion. The '+0.320 m of dash altitude drift, 66% of the vertical miss, the error is delivered by the dash' finding was measured against a baseline that included the TAKEOFF -- and altitude reads zero on the ground, so most of that number was the vehicle leaving the pad. Measured properly the dash drifts -0.019 m: essentially nothing. The vertical error is 23% a datum mismatch that exists before the dash starts, 10% dash, and 66% accumulated after the camera takes over. The altitude trim I built last turn addresses the 10%. Also found: a quarter of the post-handoff term is the camera swinging on its boom, not the aircraft moving, and the terminal throws away its altitude hold whenever the camera blinks. That last one is a real bug, fixed behind a default-off flag -- but I cannot claim it explains the miss, because the same code also slams the horizontal from 16 m/s to a dead stop on exactly the same ticks, and the flights that kept their altitude hold climbed anyway. *So what:* The dash altitude trim is a 10% lever, not the fix. The cheapest win is the datum mismatch: 23% of the vertical error exists before the vehicle moves and a one-line scenario change removes it. Nothing flew; no simulator ran this session. (scripts/forensics/vertical_miss_anatomy.py (19 self-tests, mutation-verified); ADR-0099 second addendum; docs/vertical_channel_prereg.md section 8)

<!-- overflow archived 2026-09-21 (12-entry cap) -->

## 2026-09-16

First session back on the simulator machine since the cloud sessions. Pulled their 17 commits, ran the full test suite here (974 passed), and flew two real sim flights. The flight log now records the vehicle's own tilt (pitch and roll) on every tick -- it never did before -- and the first flights show it working: level in hover, nose-down in the chase, nose-up when braking. The classic moving-target gate still passes (pro-nav 0.38 m vs pursuit 2.09 m).

**So what:** Those two flights measured directly what the cloud session could only infer. Before the chase even starts, the camera is hovering 0.16-0.20 m ABOVE the target's centre, and at closest approach the vertical miss was 0.20 m -- the same offset, carried the whole way. So the 'two height references that were never lined up' finding is real, on n=2 flights of the tag scenario. The hover itself holds height to about 2 cm, not the 9 cm suspected. And the purchase-gate speed alarm did not reproduce here: coded-dash flights close at 8.98 m/s against the gate's 9.0 -- though that pools several speeds, and the real vehicle's sprint speed is still unmeasured.

Evidence: logs/m4_intercept_pursuit_20260917T032525Z.csv + logs/m4_intercept_pronav_20260917T032639Z.csv (check_m4 PASS) - commit ef9e919 - docs/next.md items 3 and 5

## 2026-09-11

Read a 2025 IEEE paper (arXiv:2409.17497) that intercepts flying targets by steering on the CAMERA IMAGE directly instead of first working out where the target is in 3-D. That made me check our own range channel, and it is worse than I thought: out to 8 m the seeker's range is within 3% of truth, but it over-reads by about 30% around 8-12 m and past 18 m it reports roughly a THIRD of the true distance (n=64 ticks, so this one is properly powered). Our plan has been to acquire the target FARTHER away, because correction room grows with the square of the time left -- but that is exactly where this channel is worst, and pro-nav's steering command scales with range, so it would be under-commanded by the same factor.

**So what:** The plan and the sensor pull against each other, which nobody had noticed. An image-space law needs range only as a gain, not as an aim point, so it is worth TRYING -- and cheaply, because the guidance layer is already handed the raw pixel box and does the 3-D conversion internally. Nothing has been built or flown.

Evidence: scripts/forensics/range_channel_horizon.py (11 self-tests, 4 mutant classes verified); arXiv:2409.17497 Yan et al., IEEE T-IE 72(11) 2025

- **2026-09-16** — Pre-registered and flew the height fix on the adopted sprint-only setup: 32 flights, two fresh seeds, each flight paired with a twin that differs only in flying height. Inside touching distance went from 2 of 16 to 13 of 16. Before flying, caught that one of our own documents had the direction backwards (it said the vehicle passes below the target; the logs say above) -- a fix built from that sentence would have doubled the error. *So what:* This is the first time most flights meet the project's own kill distance -- but it is a best case, not a kill claim: the camera is off, and the sim knows the target's path and height exactly. A real target's height is a guess, and the camera terminal still cannot steer up or down, so that guess would go straight into the miss. Also done today: Pixhawk GPS + compass verified on the bench, Pi moved to the new Wi-Fi, test suite fixed to run the same everywhere. (ADR-0100 - docs/vertical_channel_prereg.md section 9 - commits 0ee0ea5, 6f29182)

- **2026-09-17** Overnight, on the two defects that made every camera-vs-sprint comparison unquotable. (1) After the sprint, a brief camera dropout made the vehicle command a full stop while closing at ~13 m/s -- a one-line bug, now fixed, with a new automatic check that finds it in the old logs and finds zero cases in 64 new flights. (2) The 'we have passed it' quit trigger fired early on about a third of slow camera flights. The fix the plan had lined up (ignore the trigger beyond 5 m measured range) failed its own offline check when I re-ran it independently, because the measured range is the broken quantity. The plan's fallback -- you cannot have passed a target you have not flown far enough to reach -- cut early quits from 5 of 16 to 1 of 16, and 4 of 8 to 0 of 8 on a second seed.
  evidence: docs/scoring_fix_plan.md 5b-5c - scripts/forensics/breakoff_gate_replay.py - commits d95c787, 3e1a475, c75bfb8

- **2026-09-17** Used the new tilt logging to ask why the camera never finds the target in flight. Answer: it is almost never shown it. The vehicle tips ~43 degrees nose-down to sprint, so the target sits above the top of the picture; and because it aims AHEAD of a crossing target, the target is 40-75 degrees off to the side of where the nose points. At 8-22 m the target was in the middle of the picture on 0 of 858 frames. Built and flew two fixes -- turn the nose toward where the target is predicted to be, and tilt the camera up 25 degrees -- plus a step the sim had always skipped (point the nose before launching). Result: target in the middle of the picture on 99% of frames, and the seeker really recognised it on 81% of them. Recognition was never the problem.
  so_what: Then the bad news, and it is the most important result of the night: with the camera finally seeing the target, the intercept got 2 to 5 times WORSE (3.3-3.6 m against 0.60 m for the sprint alone). The camera takes over in the first instant, before the vehicle has sprinted, and its steering was designed for a slower target and a takeover in the last few metres -- its sideways authority is capped at 8 m/s against a target crossing at 9. For the whole project the camera looked 'about as good as the sprint' only because it was blind until the last 0.2 seconds. The terminal guidance needs a redesign; that is a decision for you, queued below.
  evidence: docs/pointing_prereg.md - scripts/forensics/inframe_attribution.py - commits 3ec9ef5, 58ede84, dc6ebc2, e44770f

### 2026-09-17

Builder ruling: build an entirely new simulator. He is not satisfied with the diagnostics or the results of the Gazebo loop, the real system is an AprilTag and a Raspberry Pi, and the vehicle must intercept even if the sprint is poor or the target is at a different height. The earlier 'no simulator change' decision (ADR-0072) is reversed. His other open questions are settled by my recommendations.

**So what:** The new simulator is a fast one with no Gazebo in the loop: it runs the actual flight code against a camera-and-tag model built from our bench measurements, thousands of runs a minute, and explains every miss. It has to reproduce last night's numbers -- including the camera's failure -- before anyone trusts it. Gazebo stays only as a final cross-check.

**Evidence:** docs/new_sim_plan.md - ADR-0102

- **2026-09-17** — New simulator built in a day by worker agents plus the head: vehicle model fitted to 345 of last night's flights, a camera-and-AprilTag model from the Pi bench numbers, an adapter that drives the unmodified flight code, and a Monte-Carlo runner. Replaying last night's sprint commands through it gives the same misses as Gazebo (aim-tolerance curve 1.03/0.68/0.43/0.31/0.43/0.73/1.27 m against 1.20/0.71/0.40/0.24/0.42/0.74/1.34 m measured; height error passes straight through, +0.42 m against +0.39 m).
  so_what: It found two things on the way. The replay tool had a timing bug (now fixed) that would have hidden everything. And the Gazebo vehicle slides about half a metre sideways during the hard acceleration -- which is why last night's best aim was 5 degrees off-centre. One number in the model was tuned to that curve; six other arms it never saw still match. The baseline sweep says today's flight code passes almost nowhere, so the terminal is the work.
  evidence: commit 8dc72c1 - isim/replay_a0.py - docs/new_sim_plan.md

- **2026-09-17** — In the new simulator the fly-by concept was shown to be the limit, not the steering law: a purpose-built tag terminal scored the same as the old one, because a 0.30 m tag is readable for only ~0.6 s at 19 m/s closing. A slow-arrival pursuit concept (match the target's speed, close at a few m/s) went from 0% to 55-71% inside 0.35 m over three worker rounds; each round's gain came from a bug the new per-miss diagnostics exposed (run scored at the wrong moment, estimator seeded from noise, detection paired with the wrong instant of attitude).
  so_what: A bad sprint stops mattering (30 degrees of aim error: 53% vs 0%). Height offsets and the last second, when the tag leaves the picture, are what is left. A longer lens made it worse, not better, at this geometry. If pursuit reaches 90% it changes the flight profile and tag placement, which is the builder's call.
  evidence: isim/concepts.py - isim/specs/ - docs/new_sim_plan.md

## 2026-09-17 (archived from plain_log 2026-09-23 night, cap-12 overflow)

The pursuit prototype went from 55-71% to 92-98% inside 0.35 m from one fix: a tag detection says where the target WAS when the frame was taken (45 ms earlier), and the filter was treating it as now, so the estimate trailed a 9 m/s target by 0.4 m. A per-tick estimator trace (the diagnostics the new simulator was built for) showed the constant lag in one look, after a worker's six tuning ideas had all come back null.

**So what:** A bad sprint and a target at a different height now mostly do not matter in the simulator. It is written up as a proposal (ADR-0103) because it changes the flight profile, the flight code's breakoff logic and where the tag goes. The simulator's own-state is perfect, so treat the numbers as best-case until noise is added and Gazebo agrees.

**Evidence:** commit 'isim pursuit: delayed-measurement Kalman update' - ADR-0103

## 2026-09-17 (archived from plain_log 2026-09-24, cap-12 overflow)

Tried to break the pursuit result on purpose. With perfect self-knowledge it scored 92-98% inside 0.35 m; adding realistic errors took it to 49% (tag facing camera) and 68% (rear tag), and to 22-26% at double the errors. One at a time: frame-timestamp error 68/88%, attitude error 76/90%, position drift 81/95%, worse tag decoding 77/98%, velocity error no cost. A weaving target: 45% and 22%.

**So what:** The 92-98% was a best case and is labelled as one. The two things that matter most are measurable on the bench: how accurately the Pi timestamps a frame against the flight controller's clock, and attitude accuracy. Both also have software answers that are next in the simulator.

**Evidence:** isim/ownstate.py - isim/specs/pursuit_hardening_v5.md - ADR-0103

## 2026-09-17 (archived from plain_log 2026-09-24, cap-12 overflow)

Built the hybrid in the simulator as ruled and measured it against the other two on the same 6,600 runs. The opening sprint almost never touches a 9 m/s crossing target (first-pass contact 0-2%) and costs position: at the moment it turns around it is about 10.6 m from the target where the slow arrival is already at 5.6 m. Hybrid only wins against slow targets (80-84% at hover).

**So what:** The slow arrival already contains a fast leg -- it flies at up to 16 m/s to get behind the target. The part that does not pay is aiming that fast leg ACROSS the target's path. Bench this week: Kakute log card cleared (488 logs), Pi frame timing measured (14 ms, 0.6 ms jitter), TV calibration tool ready, lens corrected to 118 degrees from the order log.

**Evidence:** isim/specs/hybrid_v7.md - isim/concepts.py - runs/frame_timing/ - scripts/bench/

## 2026-09-17 (archived from plain_log 2026-09-24, cap-12 overflow)

Builder ruled pursuit alone ("chase only"), dropping the sprint-and-fly-by hybrid ADR-0103 had measured as worse (46% vs pursuit's 82% inside 0.35 m, all realistic errors on). Ported the pursuit concept out of the isim prototype into the actual flight code that will fly (flight/pursuit_terminal.py, a relative-state Kalman filter reusing the tag terminal's camera geometry, not its differently-tuned filter), wired it into the isim adapter, and ran it end-to-end through the real state machine -- it acquires, tracks, and closes range on a moving target. Also added a pursuit_mode flag so the existing past-CPA fly-by breakoff, tuned for a fast crossing dash, does not falsely abort pursuit's slow speed-matching approach.

**So what:** The concept is no longer isim-only prototype code; it is unit-tested (10 tests), no-cheat-audited, and validated running through the code path that actually flies. NOT done: the CLI wiring for real hardware (--terminal pursuit needs the dash-endpoint kinematics to seed the pre-flight belief, deferred rather than rushed into the hardware entrypoint) and the A0-style numeric parity check against ADR-0103's own isim numbers. Also: a builder question about safeguard flags being caused by dense project docs was audited (docs/decisions.md was 118 ADRs/93.5k words, now split/archived) but a docs-free opus5-worker subagent still hard-failed right after, so the fix is good hygiene, not a fix for the flagging.

**Evidence:** flight/pursuit_terminal.py; flight/tests/test_pursuit_terminal.py; isim/tests/test_pursuit_terminal_isim.py; ADR-0103, ADR-0104; docs/pursuit_port_2026-09-17.md; docs/doc_hygiene_2026-09-17.md

## 2026-09-21 (archived from plain_log 2026-10-02, cap-12 overflow)

Chase only is now flyable from the real CLI: --terminal pursuit selects the ported pursuit terminal and the GO trigger enters the engagement directly (the old sprint's acquire gate can never fire for a chase that starts before the camera sees anything). The pre-launch aim belief comes from the same latched cue constants the dash aim already used -- audit green, 406 tests passed. Still open: the 2 s lost-target abort fires before pursuit's own 3 s recovery can act (needs a ruling), and the isim parity run (work item 4). ADR-0105. Same session: the Pi pulled the new code over the network and ran the chase-only dry run on its own CPU (audit and dry-run both pass there), and the Pixhawk got a proper read-back check -- a new PX4-safe verifier found the safety-switch parameter added to the pack in July had NEVER landed on the board (it still read the factory bypass value); written, rebooted, re-verified 10 of 10.

**So what:** The ruled engagement concept can now actually be flown/configured on hardware, not just in the prototype sim; the lost-target-timer ruling and the parity run are the remaining gates before it is trusted.

**Evidence:** flight/tests/test_terminal_cli.py (7 tests); flight+isim 406 passed; real_flight.py --audit PASS; ADR-0105; configs/px4_6cmini/verify_params.py (COM5, 2 passes across a reboot); Pi at 547042b

## 2026-09-21 (archived from plain_log 2026-10-04, cap-12 overflow)

Parity check on the ported terminal (isim, 400 paired runs, clean conditions): the prototype meets the 0.35 m bar on every run; the flight-code port on 24% or fewer. A pre-registered A/B on the first suspect (the 2 s abort timer vs the guidance's 3 s recovery) came back NULL -- lengthening or disabling the timer changes nothing, verified with the override proven live in the state machine. The difference sits in the first-pass close (prototype ~0.12 m median, port 0.4-0.9 m); latency-constant mismatch checked and ruled out in the mean. Also measured the builder's perpendicular two-tag mount idea (rear + side tag, pre-registered, all errors on): it is a TRADE -- worse at 0-20 degrees of aim error (the side tag is nearly edge-on astern and its foreshortened width feeds bad range fixes), better only at 30 degrees (44% -> 56% touching). Suggestive at n=50, not decisive; the 30-degree win also leans on the unvalidated oblique-decode model. Altitude-error sweep on the same concept (builder question): flat within noise from 2 m below to 2 m above the guess (74-88% touching), cliff only at 3 m above (42%) -- against the retired sprint's ±0.1 m budget, an order-of-magnitude-plus wider band.

**So what:** The port needs a per-tick trace comparison before it is trusted -- the leading candidate is attitude-at-capture handling. Do not spend a ruling on the abort timer; it is exonerated. Built and measured by a Fable 5 worker, the first to complete here since 2026-09-10.

**Evidence:** isim/specs/parity_flightcode_2026-09-21.md; commit 3a27a58; docs/subagent_safeguard_log.md; isim/specs/dualtag_perpendicular_2026-09-21.md; isim/specs/alt_sensitivity_2026-09-21.md

## 2026-09-22

Field-readiness push, all flight hardware on the bench; all three parallel builds LANDED. (1) The pursuit-port parity gap CLOSED -- a registered tick-by-tick trace found four coupled defects (KF covariance Jacobian, capture-instant attitude, yaw seed, slant under-range); the fixed port went 24% to 100% inside 0.35 m nominal, median 0.068 m, verified independently. (2) The GPS cue relay was designed, built off real bench GPS data, and wired into the flight CLI (--cue-json, fail-closed): serves chase-only (8-13 deg worst-credible vs 10-30 deg tolerance), never the sprint. (3) Miss-abort safety: passage window ported, SAFE now offers hover / land / return-home, and a pursuit miss aborts straight to SAFE. Bench: the real Pi-to-Pixhawk link measured live confirms the failsafe staleness sizing. Full offline suite ALL GREEN (dev machine). Builder ruled same day (ADR-0107): a missed engagement HOVERS in place for the operator (re-approach deferred as a future feature); system-health aborts still land. Late addition (builder ask, built on the Fable 5 lane -- its first clean build here since 2026-09-10): the cue fitter now handles a TURNING target, proving a turn from the GPS's own per-sample velocities (4-sigma test) and extrapolating along the arc with hard refuse-don't-guess caps; velocity-at-latch on a turning target improved 2.8x.

**So what:** The ruled chase-only design is no longer the weaker twin of its prototype, a miss now has a defined safe ending, and the launch cue has honest piping with a measured error budget instead of a hand-typed perfect number. Two rulings queued: the slant under-range geometry also ships in the older stock seeker paths, and whether the vehicle may re-approach after a missed first pass.

**Evidence:** isim/specs/parity_trace_2026-09-22.md - docs/cue_relay_plan.md - cue decision record (contract launch_aim) - logs/bench_telem2_cadence_20260922.json - commits 8a5886c, fa54d7d, 3d1a5e4, 9efbc84, ADR-0107, 18482cf, 35b1f88

## 2026-09-22

LinkedIn media pack built for the builder's project post: five figures (robustness grid, miss CDF, engagement anatomy from the parity trace, and two MBSE views) generated from a fresh n=50-per-cell re-run of the registered parity grid that reproduced the adopted numbers exactly; copy drafted and red-teamed by a Fable reviewer against the repo (docs/linkedin_copy_draft.md). Housekeeping finding: the published MBSE Artifact had been stale since 2026-08-12 while the dashboard stayed current -- republished to today's render the same turn. Also flew a fresh M4-class pro-nav flight on the tag world for the post's video (closest approach 0.413 m, clean, gate criterion <1 m) and cut a 24 s honesty-labeled clip from its live onboard-camera capture (SIMULATION tag on every frame, retiming disclosed, no contact claim -- 0.413 m does not clear the 0.35 m ram bar).

**So what:** The project can now be shown publicly with figures whose every number reproduces from a logged run, and the published systems model matches the build again. Nothing here changes any capability claim.

**Evidence:** logs/linkedin_media_20260922/ - docs/images/linkedin/ - scripts/linkedin_media_figures.py - logs/m4_intercept_pronav_20260923T025539Z.csv - scripts/video/linkedin_hero_cut.py

## 2026-09-23

Recruiter-readiness + validation push, all lanes. (1) Opus 5.5 tested the day it appeared: three clean defense-framed passes on this repo (a guidance-math analysis, the cross-check harness build, the README audit) -- a working high-capability subagent lane again, logged in docs/subagent_safeguard_log.md. (2) 68 superseded files consolidated to retired/ (S2 ground-stereo, T25 demo pipeline, old forensics, tilt/track harnesses, closed planning docs) off a 248-file reference-topology inventory; suite green after. (3) README rebuilt: chase-only framing, status box, intercept GIF above the fold, honest seeker story, fresh screenshots; stale GitHub board snapshot (43 days) refetched. (4) The pre-registered Gazebo cross-check of the pursuit port flew n=8 with real camera-in-the-loop: shakedown caught parity defect #5 (Phase A yawed at the rendezvous point, freezing the camera off-target when station-keeping -- the prototype yaws at the believed target; fixed, regression-pinned, and the fix closed the isim aim20 cell 70%->100%), then the scored flights FAILED the registered bar: camera-driven chase confirmed, CPA 0.55-2.33 m, median 1.51 m vs 0.122 m predicted. (5) Speed work for the video: ladder scouting on the tag scenario (n=1 leads, not verdicts) read 3 m/s -> 0.238 m clean, 4 m/s -> 0.961 m, 6 m/s -> 5.29 m miss; one flight of the ADOPTED 16 m/s open-loop sprint config vs the 9 m/s line-9 crosser scored CPA 0.334 m (inside the contact bar; single flight, best-case givens). A 35 s two-act video was cut from the live onboard captures with honest labels. (6) Fast-intercept limits identified with data (docs/fast_intercept_limits.md): the chase's speed ceiling is OVERTAKE MARGIN (raising the configured v_max 16->20 takes a 15 m/s target from 0% to 100% inside 0.35 m in isim); the classic tag terminal is capped by its July-era 3/3/4 m/s command envelope (raising it took the 4 m/s scout from 0.96 to 0.14 m) and above ~4 m/s by the standing-start geometry itself (the running-start lesson re-derived); and a narrower-FoV/higher-res second camera does NOT help the terminal -- an fx sweep with the honest walkout+blur model made the chase monotonically WORSE (118 deg 100% -> 45 deg 24%), re-confirming the ADR-0024 narrow-lens rejection in the chase regime. Opus 5.5 logged its first bounce (3 passes, 1 [general_harms] fail) -- usable, not uniform. Both stranded remote branches merged under builder authority and deleted.

## 2026-09-23

Adaptive speed governor built and A/B'd in answer to the builder's ask (speed from target speed, phase, lock quality). Cap sizing -- believed target speed plus a 6 m/s overtake margin, clamped 8..24 -- matches a flat raised cap on every cell and takes the cells the legacy 16 m/s cap loses (15 m/s target: 54 to 100 percent contact; 18 m/s: 0 to 78). The lock-quality lever (slow down when decodes go stale) was REJECTED: null on aim-error cells and a 12-14 point cost at 18 m/s -- slowing keeps the tag small, which keeps the decodes sparse.

**So what:** The chase concept now has a recommended config that scales to fast targets without an operator-picked cap (ADR-0108). Still isim-only and NOT the flying default: the Gazebo transfer gap stays the blocker, and speeds past ~16-18 m/s extrapolate the vehicle fit.

**Evidence:** docs/adaptive_speed_prereg.md (registered A/B + 3 amendments, n=50 paired/cell); scripts/adaptive_speed_ab.py; ADR-0108

## 2026-09-23

The Tier-2 interceptor airframe order is IN — placed by the builder 2026-09-22, ahead of the tripod money gate (his call, logged ADR-0109): Source One V6 frame, Skystars 60A ESC, five ECOII motors (arrive ~Oct 5–15), RP3 receiver, Matek 12S-Pro BEC, a replacement PM02, the second Pocket TX (closing the ADR-0089 question), 16 props, two lost-model buzzers, a LiPo bag and zip ties.

**So what:** Every powered part of the interceptor is now in hand or on order. What is still missing for a full intercept is small and mostly print/bench work: the flight controller's ULog memory card (no card = no scored flight), Pi-mount standoffs, the thrust rig, three prints (camera mount, checkerboard, tag placard), the field kit (power bank, tripod, cones, +2 battery packs) and kill-attempt spares. Two bench checks before trusting the new parts: the BEC's 5 V tap must really give 5.2 V under Pi load, and both transmitters need Model Match set up.

**Evidence:** order screenshots (builder, 2026-09-23) - ADR-0109 - docs/hardware_order_list.md §0d

## 2026-09-23 (archived from plain_log 2026-10-17, cap-12 overflow)

Builder-directed refinement day, four parallel lanes. (1) The keep-in-frame climb assist he asked for was built, pre-registered, and honestly NULLED -- at +3 m of height error the tag almost never decodes, so a camera-triggered climb cannot fire; the follow-up attribution put numbers on it (41% of blind frames off the frame top, 35% too small) and surfaced a measured regression for the obvious alternative (Phase-A vertical sweep) that had been hiding in code comments -- both now graveyarded properly. (2) His wind question is answered with code receipts and a registered sweep: the chase corrects wind by feedback (within 6 points of no-wind up to 4.5 m/s); the unmeasured drag table only ever fed the retired sprint's aim solve; what wind costs is decode loss from leaning (8 m/s head: -14 pts). (3) The Gazebo 12x transfer gap is ATTRIBUTED: sim plant ~2x too fast, a 15-25% box-width range bias (the detector's pose range is clean), and a half-rate detect gate; latency and no-re-approach exonerated; evidence-based fixes + re-prediction in flight. (4) The MBSE model gained the week's corrective subsystems, and its open-items view was found rendering every open contradiction as a BLANK row -- fixed fail-loud.

**So what:** Three registered predictions, two nulls, zero spin -- and by end of day the fix list was built, verified, and FLOWN: the re-fly came back NULL-with-instruments-green (chase tightened 1.510 -> 0.967 m median; range bias and cadence fixed as attributed; bar not met). The remaining risk is named: the pursuit law's braking behavior on the real-speed plant, plus the +3 m height cliff already answered by the +10 deg bracket recommendation.

**Evidence:** isim/specs/{xcheck_tick_trace,keepframe_prereg,wind_chase_prereg}_2026-09-23.md - commits 23aba0b, aa47972, 0d28538, 226100f - ADR-0110/0111

## 2026-09-23 (archived from plain_log 2026-10-17, cap-12 overflow)

Overnight (builder asleep, directive: 'make the sim's AprilTag true enough to real life -- tilt, pitch, shake, glare'). Built tag realism v1 into the fast sim: the target now derives a real quad attitude from its trajectory (nose-down cruise pitch, banked turns) with the tag bolted to its body, plus attitude wobble, camera-vibration blur, and sun-geometry glare -- all default-off and byte-identical, every constant graded estimate. Flew the registered ladder (n=50), a disjoint-seed confirmation (n=150), a mechanism probe, and a registered canted-mount A/B (n=100). Nominal chase: realism is an honest NULL (90 -> 87-91% touching). The real finding: the +10 deg camera bracket's height-error win collapses under a body-coupled tag (+21 -> +2..+14 points), because the pitched target tilts its rear tag face up-and-back and close-in decodes halve; canting the placard 12 deg DOWN (a print-time choice on our own mount, index disc records it) recovers the cell to 60/64% and restores the bracket's +20 margin.

**So what:** The builder's suspicion was half right: the tag model WAS hiding a real effect -- not in the flying nominal numbers, but in the pending bracket ruling. That ruling is now 'bracket + canted placard as a pair'. No guidance change was warranted (registered criterion: the deficit was geometric, fixed at the mount). All numbers isim-tier; Gazebo/bench rung and the measured real cruise pitch come next.

**Evidence:** ADR-0114 - isim/specs/tag_realism_v1.md SS F/G - logs/tag_realism_20260923/{ladder_v1,confirm_n150,mech_probe,cant_ab}.txt - commit 7546c12

## 2026-09-24 (archived from plain_log 2026-10-17, cap-12 overflow)

Overnight part 2: the contract's named next item (braking/approach shaping vs the honest ENGAGE plant) ran as two registered rounds plus a per-run trace. A stopping-distance cap on the chase's relative command -- sized from the fitted 5.9 m/s2 braking authority -- was built default-off and swept 6,100 runs across both vehicle fits and the new realistic-tag rungs. It confirms the Gazebo re-fly's residual: braking the approach takes honest-plant nominal contact 62 to 98 percent and halves arrival speed. But it costs 12-28 points on the target-3-m-above cell, and the trace attributed WHY: slowing the horizontal de-synchronizes the vertical intercept -- the climb reaches target altitude early and overshoots 0.2-0.7 m while the braked closure is still coming, so CPA lands just over the bar. Under the registered no-regression rule nothing is adopted; the code sits in-tree, off, fully pinned.

**So what:** The diagnosis is complete on both sides: the pursuit law arrives too hot for the real airframe (worth ~35 points at nominal), and the fix's one side effect is a now-understood vertical timing problem, not a mystery. The trade -- adopt the cap for Gazebo re-fly #3, first add a registered vertical-arrival-sync term, or stop here -- is queued for the builder with the attribution attached.

**Evidence:** ADR-0115 - isim/specs/brake_shaping_prereg_2026-09-24.md (both registrations + tables) - logs/brake_shaping_20260924/

- **2026-09-23** (rolled out 2026-10-14): Recruiter-readiness + validation push, all lanes. (1) Opus 5.5 tested the day it appeared: three clean defense-framed passes on this repo (a guidance-math analysis, the cross-check harness build, the README audit) -- a working high-capability subagent lane again, logged in docs/subagent_safeguard_log.md. (2) 68 superseded files consolidated to retired/ (S2 ground-stereo, T25 demo pipeline, old forensics, tilt/track harnesses, closed planning docs) off a 248-file reference-topology inventory; suite green after. (3) README rebuilt: chase-only framing, status box, intercept GIF above the fold, honest seeker story, fresh screenshots; stale GitHub board snapshot (43 days) refetched. (4) The pre-registered Gazebo cross-check of the pursuit port flew n=8 with real camera-in-the-loop: shakedown caught parity defect #5 (Phase A yawed at the rendezvous point, freezing the camera off-target when station-keeping -- the prototype yaws at the believed target; fixed, regression-pinned, and the fix closed the isim aim20 cell 70%->100%), then the scored flights FAILED the registered bar: camera-driven chase confirmed, CPA 0.55-2.33 m, median 1.51 m vs 0.122 m predicted. (5) Speed work for the video: ladder scouting on the tag scenario (n=1 leads, not verdicts) read 3 m/s -> 0.238 m clean, 4 m/s -> 0.961 m, 6 m/s -> 5.29 m miss; one flight of the ADOPTED 16 m/s open-loop sprint config vs the 9 m/s line-9 crosser scored CPA 0.334 m (inside the contact bar; single flight, best-case givens). A 35 s two-act video was cut from the live onboard captures with honest labels. (6) Fast-intercept limits identified with data (docs/fast_intercept_limits.md): the chase's speed ceiling is OVERTAKE MARGIN (raising the configured v_max 16->20 takes a 15 m/s target from 0% to 100% inside 0.35 m in isim); the classic tag terminal is capped by its July-era 3/3/4 m/s command envelope (raising it took the 4 m/s scout from 0.96 to 0.14 m) and above ~4 m/s by the standing-start geometry itself (the running-start lesson re-derived); and a narrower-FoV/higher-res second camera does NOT help the terminal -- an fx sweep with the honest walkout+blur model made the chase monotonically WORSE (118 deg 100% -> 45 deg 24%), re-confirming the ADR-0024 narrow-lens rejection in the chase regime. Opus 5.5 logged its first bounce (3 passes, 1 [general_harms] fail) -- usable, not uniform. Both stranded remote branches merged under builder authority and deleted.

## 2026-09-24 (archived from plain_log 2026-10-17, cap-12 overflow)

Midday: both morning rulings executed end-to-end. The bracket+cant pair is in the build spec. The brake thread ran its ruled sync round (fixed the traced overshoot, exposed a second decode-starvation class, stopped per its rule), the builder then ruled 'fly the brake package in Gazebo', and re-fly #3 flew as registered: median 0.928 m vs 0.967 -- a registered NULL -- but with the first two sub-0.35 m contact-range flights this cross-check has ever produced. The decisive instrument finding: the brake was provably on yet endgame closing speed did not move, because the Kalman filter's range estimate collapses by ~2.6 m in the final two seconds (measurements clean at capture; the error is coast/extrapolation). Three campaigns in: 1.510 -> 0.967 -> 0.928 m, every registered fix landing as attributed.

**So what:** The transfer gap has now been chased out of the measurement channel, the cadence, the plant fit and the approach speed -- the endgame estimator is the last suspect standing, with a measured -2.6 m signature isim structurally under-models. Next unit of work is a registered estimator diagnosis, not tuning and not another Gazebo campaign.

**Evidence:** ADR-0116 - docs/xcheck_gazebo_pursuit_prereg3.md - logs/xcheck_gz_20260924_brake/

## 2026-09-24 (archived from plain_log 2026-10-17, cap-12 overflow)

Afternoon: the registered estimator diagnosis found the campaign misses' real cause -- a two-line wiring defect in the real flight driver. It subscribed to the EKF velocity but kept only the speed scalar, so the chase's Kalman filter ran on the last COMMANDED velocity (3.6-4.2 m/s off) in all 16 prior flights; the filter absorbed the bias, the range estimate collapsed ~2.6 m in the final coast, steering froze at ~3 m, and the brake cap read the same wrong vector. Fixed (ADR-0117), re-fly #4 registered and flown: median CPA 0.343 m, 8/8 inside 1 m, 4/8 inside the 0.35 m contact envelope, zero aborts -- the registered bar PASSES for the first time across four campaigns (1.51 -> 0.97 -> 0.93 -> 0.34).

**So what:** The camera-driven chase now meets its own bar in the full-physics sim, and the defect it took four campaigns to find would have flown on the real aircraft -- the cross-check loop just paid for itself. Two rulings reopen (default swap; brake, whose earlier verdicts are void), and the honest caveats stand: perfect cue, windless, upright tag, sim camera.

**Evidence:** ADR-0117 (+addendum) - docs/xcheck_gazebo_pursuit_prereg4.md - logs/xcheck_gz_20260924_velfix/ - logs/endgame_diag_20260924/

## 2026-10-02

The hardware now has a systems model: every part actually in the build -- target, interceptor, tripod seeker rig, ground gear -- is written as a SysML v2 text model (docs/sysml/) and drawn as power, data and mechanical wiring diagrams on the MBSE sheet (view 8). The OMG reference SysML v2 tool reads all seven files with zero errors. Each wire says whether it is planned, made, or proven by a named build step: 17 of 60 are proven. Same day, builder ask: the diagrams are now TAPPABLE -- a wiring-editor page queues connection / state / build-status changes from a phone, type-checked in the browser, applied to the model only by a Claude session that re-runs every gate.

## 2026-10-02

The hosted dashboard Artifact (the claude.ai mirror of this contract) cannot currently be republished: the sync turn trips a safety classifier on every model tried (Sonnet through Opus), and the 2026-10-02 attempt was stopped mid-response. Per the standing rule, no rewording or workaround was attempted. The builder sent /feedback the same day with the flagged request IDs (req_011CfcYBK6pdkcZ9gpXss9H8, msg_011CfcYBKh3STKZHrPfAcWfm). WAITING on that; no retry cadence promised by the channel. [RESOLVED since: republishes went through normally from 2026-10-17.]

- **2026-10-02** — Bench: the chaser's (interceptor's) RC kill link is wired and PROVEN at the PX4 level. RP3 ELRS RX soldered to the 6C Mini's RC IN; bind phrase + SBUS protocol configured over the RX's own WiFi (no reflash needed, both ends already 3.3.1); first tests failed silently and the fault was run to ground with instruments, not guesses -- the IO co-processor's own counters showed zero bytes, a 3.2 V probe at the S pin then proved the wire good, isolating a two-fault stack: an unseated DuPont junction plus the wrong SBUS polarity option. With plain SBUS + a re-seated junction: 18-channel frames, full-range stick sweeps, and the no-pulses failsafe (stream dies with the TX, relinks in seconds) all verified via the new rc_check.py bench tool. The lost-model buzzer went on as standalone 2-wire (the M10 GPS already provides FC tones). Kill switch COMPLETED same session: the ch5 bumper mapped (RC_MAP_KILL_SW=5, written + read-back verified across a reboot), and PX4 observed reading both engage and release live via manual_control_switches. Found needed: 2x 18650 cells for the 2nd Pocket (added to gnd-01).

### 2026-10-03

Wiring-editor friction fix, builder ask: changing a wire's state no longer requires naming a build-step id. 'evidence' is now an optional free-form note (a step id, a date, any short text) in the editor form, the queued edits, and the model validator -- recorded and shown when given, never required. linkState tracks bench build progress, not a performance claim, so the honesty surfaces (key_numbers provenance, assumptions register, gt_* boundary) are untouched.

**So what:** The builder can now mark a wire wired/verified from his phone in two taps. The two validator tests that enforced the old rule now pin the new one.

*Evidence: docs/decisions.md ADR-0123 addendum 2026-10-03; scripts/render_sysml.py; tests/test_render_sysml.py*

## 2026-10-04

Pre-battery readiness bench (remote; builder wiring complete except the Pi-rail cap). FC: 11/11 bench params verified across a reboot incl. the new BAT1_N_CELLS=6; M10 3D fix indoors; all 14 PWM output functions confirmed unmapped (nothing can spin even armed); PX4 1.16.0 confirmed. The microSD scare resolved by the builder's challenge: MAVLink storage/log-list queries answer EMPTY on this board, but NSH shows the card mounted with a log/ tree -- an instrument quirk, not a missing card (verify logs via QGC Log Download at the battery session). THE BIG ONE: the props-off TELEM2 OFFBOARD gate (brn-05 class) re-run on the ASSEMBLED airframe harness -- first runs failed on a REAL bug found and fixed the same bench: mavsdk-python pumps mavsdk_server output on a non-daemon thread, so seeker_loop completed rc=0 then hung forever when stdout is a pipe, and the gate mislabelled it a link timeout. Teardown now kills the server explicitly; official gate PASS (300/300 setpoints, 19.5 Hz, max own_age 0.02 s, OFFBOARD accepted disarmed). Pi lane: OS full-upgrade (342 pkgs) + EEPROM + new kernel, camera/venv green, recorder soak clean (8,958/8,958 frames, 0 drops, 47 C max, PROVISIONAL -- no tag in view). Three Opus 5.5 pre-upload reviews landed; executed same turn: target FLTMODE_CH 5->12 (ELRS AUX1 is hard-limited to 2 positions -- the 6-mode table collapsed to Stabilize/Auto; lint invariant added), rc_check docstring corrected (RC-loss announcements are events-only in v1.16), bench_status no-heartbeat vacuous-pass fixed. Version ruling ADR-0121: hold PX4 1.16.0 + ELRS 3.3.1-matched through the 6S gate. EVENING, the chase-code sprint (builder: 'get it going now'): the live camera was never wired into the mission driver -- the perception stack (seeker_loop) and the mission brain (real_flight) were proven separately and never married. Married tonight: flight/deploy/frame_feed.py adapts the proven sources/detectors into run_mavsdk_mission (byte-identical without the new flags), plus a --desk mode that runs the REAL state machine on REAL frames with no MAVSDK import -- structurally cannot arm. Camera CALIBRATED the same evening with zero prints: the repo's screen-checkerboard flow + an auto-cycling board page on the builder's monitor + the airframe camera pointed at it -- 81 views, 9/9 zones, 3/3 distance buckets, RMS 0.731 px (skr-06 bar <=1.0 PASSED); stamped OV9281 intrinsics committed. Desk chain run: 856/856 detector hits, STANDBY->ENGAGE->BREAKOFF->SAFE, 428 frames recorded, 0 drops (first run at the 1 ms flight exposure read a monitor at mean 36 and got 0 hits -- a bench illumination mismatch, fixed with --exposure-us/--gain bench overrides; the flight spec is untouched). FINDING, needs a tape measure: the solved fx=1104 px implies ~60 deg horizontal FoV against the vendor's 118 deg claim -- if the cheap geometric check confirms it, the delivered lens violates the wide-fov HARD constraint (the +/-30 deg aim tolerance depends on >=100 deg). FOV VERDICT, measured twice and matching: the builder's tape measurement (14-in screen exactly fills the frame at 10 in -> 70.0 deg) agrees with the calibration pushed through its own distortion model (69.4 deg; the earlier '~60 deg' was the naive pinhole edge number, wrong under k1=-0.43 barrel). So the solve is independently VALIDATED and the delivered lens is ~70 deg horizontal against the 118 deg listing -- the wide-fov HARD constraint (>=100 deg) is VIOLATED by the delivered optic. At 70 deg the +/-30 deg aim tolerance has the target at the frame edge (asymmetric: 29.7 deg one side). Lens swap decision queued; the M12 mount makes a swap cheap and the screen-calibration flow re-earns intrinsics in minutes afterward. NIGHT CAP -- afr-02 PASSED: first 6S battery power-up through the smoke stopper, props off (builder-confirmed), zero drama. FC battery telemetry live and sane on the new BAT1_N_CELLS=6: 23.2 V = 3.86 V/cell (textbook storage charge), ~59% estimate; current 0.00 A at idle as expected (FC on USB, ESC idle below sensor resolution -- current cal waits for a motor load). The BEC->Pi rail stays open: builder is re-wiring the tap with better wire, cap still inbound, two-rail rule holds. Instrument note: NSH-over-MAVLink answers exactly once per boot on this board, then wedges -- retired for the night; the ULog-writing check moves to QGC Log Download (one click).

**So what:** The 6S smoke-stopper power-up (afr-02) is GO: kill chain proven, battery monitor set, outputs unmapped, the Pi-FC link re-proven through the final harness, and a false-FAIL instrument fixed before it could burn a bench day. Before ARMING (not before power-up): QGC radio calibration, airframe selection (wipes most params -- reload the pack after), and the reviews' pre-arm list. And the chase code went from 'camera not wired in' to running end-to-end on real hardware perception in one evening, with the camera calibrated off the builder's own monitor. Still between here and a flight: the FoV check, a measured tag span, QGC radio cal, and the pre-arm list. The FoV finding is the kind tripod day would have surfaced as a mystery (short acquisition cone); tonight it is a $10-20 M12 lens line item instead. The chaser has now been battery-powered once, safely, with believable telemetry -- the airframe chain's next gates are radio calibration and the QGC airframe/actuator sequence, then motors.

*Evidence:* logs/deploy_seeker_bench_20261004T201641Z.log (Pi) - configs/px4_6cmini/verify_params.py 11/11 - runs/recorder_gate_20261004/ - flight/deploy/seeker_loop.py teardown fix - configs/target_kakute selftest green - ADR-0121 - flight/deploy/frame_feed.py - configs/ov9281_intrinsics.json (81 views, rms 0.731 px) - runs/desk_20261004T205433Z (Pi) - commit 1b10547

## 2026-10-12

Builder ordered the bench consumables and the replacement seeker lens. Amazon: low-ESR 1000 uF caps, 18 AWG silicone wire plus a USB-C pigtail (so the Pi's 5 V feed no longer hangs off thin DuPont pins), solder, a 150 W car inverter for the laptop and a Starlink Mini car adapter for field internet. Lens: an Arducam 1.8 mm M12 rated 110 deg horizontal on this sensor, which answers the lens question in his queue -- the delivered 70 deg lens broke the >=100 deg rule. An earlier list cited a wrong Edmund part number and a European price; the builder's browser check caught it before checkout.

**So what:** The two things blocking the Pi-on-battery wiring (the cap and a wire he can actually solder) arrive this week, and the seeker gets back inside its hard field-of-view requirement once the lens is swapped and re-calibrated. Nothing has been measured yet -- the lens FoV is a listing figure until the screen calibration confirms it.

*Evidence:* gnd-01 (build_tab ladder); builder Amazon order screenshot 2026-10-12; Arducam M40180H13L product page

## 2026-10-13

Replacement Kakute H7 bench bring-up, start to finish in one remote session (builder soldering, Claude driving over USB). Pre-flash: Betaflight-side health probe showed I2C errors 0 (the bus that killed board #1), gyro/baro alive, SD readable; the factory board was then DFU-flashed to ArduCopter 4.7.0 (verified) and the 50-param pack loaded with the two-pass reboot ritual -- the BATT params needed the second pass exactly like September's '23/23 but 4 missing' lesson. GPS (u-blox on UART3) and compass (QMC5883L at I2C 0x0D) detected immediately: the rebuilt wiring is good. Then the wall: motors dead, ESC chirping every ~3 s. Hours of differential work -- DShot rate ladder (dead end: AM32 doesn't even speak DShot150), signal-disable isolation test (invalidated twice: ArduCopter silently re-assigns motor functions on boot), ESC socket jostle test (found a REAL secondary fault: the Tekko32's 8-pin socket is flaky under flex, bypassed with a hand-soldered pigtail to the gold pads), full param diff against board #1's last DataFlash log pulled off the shared SD card (clean -- configs identical, so 'what's different' had to be physical). The actual fault, found by the builder: the harness was plugged into a LOOK-ALIKE WRONG SOCKET on the FC. Moved it; the ESC beacon went silent (it was never boot-looping -- AM32's no-signal beacon, misread); motors spun on the first test. The pigtail's wire order swaps the two left channels and flips both left motors vs board #1's cable: remapped in software (SERVO3/4 swap, RVMASK 6->10), verified motor-by-motor A-D, and target.param updated the same hour so a re-upload can't silently undo it. Board #2 now matches its param file 50/50 by read-back.

**So what:** The target drone is propulsion-complete and config-clean on the new FC -- the biggest setback of the build (the dead board) is fully recovered. Remaining before it flies: accel/compass cal, RC link (blocked on TX batteries), SD reformat after pulling the old logs, and the current-sense wire. Two instrument lessons worth keeping: an ESC's periodic chirp means 'no signal', not 'dying'; and a motor-output isolation test on ArduCopter must re-check SERVOx_FUNCTION after reboot, because the firmware auto-fills them.

*Evidence:* runs/bench_20261013_fc2/{fc1_last_dataflash_log3.bin,fc1_params_from_log3.parm,fc2_params_prerempap.parm} - configs/target_kakute/target.param motor-mapping block (2026-10-13) - AM32 wiki (protocols: PWM/DShot300/600)

## 2026-10-14

Airframe and propulsion numbers written down in one place: docs/airframe_numbers.md, derived by scripts/airframe_numbers.py (pinned by a test), with an interactive calculator page. Interceptor parts roll-up ≈800 g; planned T/W ≈7.5:1 (1.5 kg/motor, listings say 1.4-2.0); momentum-theory hover ≈7.5 A and ≈10 min on a 1500 mAh 6S pack; 6S pack, motor (Kt, RPM, tip speed), drag/trim, air density by field elevation, and carbon-fibre modulus (fibre 230 / one-direction 135 / frame plate ≈56 GPa) with an arm deflection and vibration check.

**So what:** Every physical number now traces to a derivation instead of chat, and the one thing they all rest on (weight and thrust, both unmeasured) is a registered assumption with its bench test named. The older ≈770 g roll-up in decisions.md used the first parts pick; the new table uses the ordered parts.

*Evidence:* docs/airframe_numbers.md · scripts/airframe_numbers.py · tests/test_airframe_numbers.py · docs/airframe_calculator.html


### 2026-10-22 (full overnight text, archived 2026-10-22 ~03:00 when the entry hit 8.7k chars)

Gazebo re-earn of the latency requirement, pre-registered and paired (runs/gz_latency_20261022): cinema_run grew --inject-det-latency-s (driver-side DelayedDetector; 0 = byte-identical validated path), 18 flights at the validated 12 m/s crossing, 6 cue variants x {0, 60, 120 ms}. RESULT: 120 ms arm degraded 6/6 pairs, median 0.238 -> 1.309 m (5.5x) -- the isim cliff CONFIRMED by the registered rule. SURPRISE: the 60 ms arm, which isim called within-noise, degraded 4/6 pairs at 2.3x (median 0.552 m -- outside the 0.35 m ram radius). So the real glass-to-guidance budget is ~30 ms, 60 ms is degraded-but-flyable, 120 ms is broken; the 'latency <=60 ms' spec line was swept off every surface the same turn (findings amendment, program doc, this board) and the ledger carries isim-latency-surface-optimistic (model under-penalizes mid-latency; re-fit before isim is quoted on latency margins). Purchase implication: CPU-only is ruled out at ANY input size (measured 131-154 ms; NCNN-optimistic ~70-90 ms still in the degraded band); the Hailo path must budget its CPU-side pre/post + tracker inside ~30 ms total. Same evening: auto-label + sim-randomizer tooling landed in scripts/nn/ (13 tests) and SMOKED on real Pi desk footage (13/13 labeled, mask on the tag, calibrated intrinsics); Hailo calib-set builder + Det-Fly converter + DFC dry-run script committed. LATER THE SAME DAY, the builder's pro-nav question got its mechanism answer: the literature's fix for the PIP pointing failure (look-angle-constrained biased aim) was implemented as default-off isim levers (pip_aim + look_angle_max_deg, 197 tests green) and a pre-registered study ran it against pure chase -- NULL. Chase won every informative cell (0.17-0.23 m vs 0.35-0.86 m); the PIP-aim arm decoded 5-6x more frames and still missed worse, so the failure is the predicted-point aim itself under bearing-only time-to-go noise, not camera pointing. Chase stays the law (two-study chain: resolution doesn't rescue predictive guidance, and neither does the FOV-constraint fix). Also head-inline (both subagents bounced): the M12 lens memo (docs/lens_options_2026-10-22.md -- keep the 70-deg lens unless the +/-30-deg aim tolerance must stand; an $12 80-deg middle option exists; >=100-deg needs the Hailo native-res path first) and the hybrid-guidance mechanism notes (docs/hybrid_guidance_notes_2026-10-22.md). NIGHT CAP, the latency saga's third act: compensated arms LC60/LC120 (KF told the true age via the new --pursuit-meas-latency-s flag) flew at BASELINE quality (1.19x/1.05x) -- the whole cliff was the fixed 45 ms age assumption, both my pre-registered predictions were wrong in opposite directions, and isim was right all along (it models the age-known case; ledger entry resolved, no re-fit). Final detector spec: latency must be KNOWN (calibrate meas_latency_s or stamp detections -- stamped mode queued), jitter small, rate >=10-15 Hz, range >=8-12 m. The Hailo purchase argument is restated honestly: not the retired latency cliff, but RATE (CPU 154 ms = 6.5 Hz, under the floor), load-jitter, and native-res headroom for the 118-deg lens scenario. FOURTH ARM (jitter, LJ90): +/-60 ms uniform around a correctly-compensated 90 ms mean = 1.53x median with a 1.2 m tail -- on the registered boundary. A calibrated constant is flyable for CPU-class jitter; the tail prices the queued stamped-age change and keeps low-jitter hardware preferred. Latency chapter closed, four arms, full table in runs/gz_latency_20261022/spec.md. And the improvement LJ90 priced got BUILT the same night: --pursuit-stamped-age feeds the KF each detection's true pipeline age (FeedDetector capture stamps, same clock; Gazebo's foreign-clock stamps structurally excluded), with a SUPPORTS_STAMPED_AGE capability flag so no other terminal's interface grows a surprise kwarg. Default OFF, byte-identical; 61/61 terminal tests on the dev machine AND on the Pi after sync. NEXT EVENING (10-21): the builder created the Hailo DevZone account (the queue item's step 1) -- the DFC 3.33.x fetch now waits only on a one-time Chrome login (credential entry is agent-prohibited); the calib-set builder scripts/nn/build_calib_set.py, which the earlier note overstated as committed, was smoke-verified (640x640 gray->3ch letterbox out, fail-loud under --min) and actually committed. THE DRY-RUN VERDICT (same evening): after 11 live-fixed environment defects (wrong 5.x wheel line, MZ not on PyPI, build isolation, py3.12-incompatible MZ pins, WSL2-GPU and glibc-2.39 native crashes -> the compile MUST run in an ubuntu:22.04/py3.10 container, end-node surgery, missing NMS JSON, USER env) stock yolov8n compiled SINGLE-CONTEXT for hailo8 at 640 AND at 1024 -- the purchase gate is satisfied and the native-res headroom question is answered YES on the $110 chip. Calibration was synthetic: fit/context evidence only, no accuracy claims. Also: the builder's blind-dash-turn question was answered against the code (belief-track aim, no gt read, stern trail d_behind=8 already implemented; video take = best of 5 WITH injected cue error) and became a pre-registered geometry-vs-cue-error study (isim/specs/stern_approach_2026-10-21.md), the evidence phase of his requested dash-router expansion. THE STERN-APPROACH SWEEP FLEW AND RETURNED A NULL by its registered rule (48 cells x 8 seeds): no geometry cell beats the default beyond paired-seed noise, the astern dash aim buys no acquisition anywhere and is harmful under cue error (median miss up to 2.7x; a zero-error miss improvement in some cells is the same n=8 seed-luck class -- review F2 correction), and cue error dominates acquisition (engage 88->38% as deliberate aim error goes 0->8 deg) -- so the proposed dash ROUTER is NOT built, the 8 m trail stands, and the honest lever for 'guarantee the grab' is acquisition range + cue quality, i.e. exactly what the tripod day measures (runs/stern_approach_20261021/findings.md; knobs stay in the tree, default-off, for future regimes). The null's footnote closed the same night by its registered n=32 follow-up: the n=8 'd_behind=4 halves miss' gap was seed luck (0.370 vs 0.383 m at n=32, engagement identical) -- the 8 m trail stands everywhere, and the geometry chapter is fully closed. DEPLOYMENT-PATH CHECK (fork, overnight): the 1-CLASS compile via --classes lands in TWO contexts, not one -- the stock single-context headline does NOT transfer to the deployment config on DFC 3.34/MZ 2.17; it still compiles and runs, fps cost = day-1 benchmark question; recovery lead (classes via the NMS JSON) tested the same night. OVERNIGHT REVIEW (Fable fork, adversarial): CRITICAL F1 caught BEFORE the LJ90-S arm flew -- the stamp republish was on the wrong clock base (every age would clamp to 0 and falsely refute the registered prediction, with the unit test pinning the wrong convention); fixed to the monotonic axis + a DEBUG_DET_AGE effect gate on take c1. Also: the findings overclaim corrected (F2), the d_behind-under-cue-error rows marked unreachable-not-flat (F3), and the stock dry-run verdict confirmed clean including pct_engage==acquisition (F5). LJ90-S FLEW (post-fix, c1 age-gated): stamped age under the same jitter = median 0.257 m (1.08x baseline), 0/6 degraded, LJ90's 1.2 m tail collapsed -- the registered PREDICTION confirmed; the adopt bar MISSED BY ITS LETTER (3/6 strict pairwise improvements; the other three inside the 0.2 m noise band), reported as written, recommendation = builder call, default stays OFF. Bonus measurement: true pipeline lag ~80 ms natural (constant arms were ~35-65 ms under-compensated and still flew ~baseline -- mean error that size is cheap; LJ90's 1.53x is genuinely jitter). SIXTH REGISTERED STUDY of the night: target-height cue error (the vertical given) -- budget measured at +/-2 m, and the asymmetry REVERSED the prediction: target-ABOVE-belief breaks acquisition at +3 m (engage 94->69%) while -3 m still engages; target-height-known regraded given-perfect -> given-noisy with the written +/-2 m requirement on the cue interface (runs/target_height_20261022/findings.md). The +3 m asymmetry's first mechanism probe (registered, measure-only) REFUTED the wedge-coverage hypothesis -- the high target is in frame at comparable rates and still fails to acquire -- and two further registered rounds then CHARACTERIZED it: first decodes happen at ~30 m in every cell, and failures are standoff-bounded passes -- the sparse far decodes race to fix the vertical belief before the pass, and losers never get the close burst that fills the acquire window. One layer stays open (why the race favors low targets); no sign rule quoted; the +/-2 m budget stands. Full suite re-run ALL GREEN after the night's flight-code changes.
