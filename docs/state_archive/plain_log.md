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
