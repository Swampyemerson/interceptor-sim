
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
