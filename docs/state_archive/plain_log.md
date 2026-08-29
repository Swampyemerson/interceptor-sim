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
