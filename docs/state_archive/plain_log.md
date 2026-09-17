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
