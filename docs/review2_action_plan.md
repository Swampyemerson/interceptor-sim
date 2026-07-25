# Review-2 action plan — fix the 73 silent-failure gaps + prevent the class (2026-07-25)

**Inputs:** `docs/review2_silent_failure_findings.md` (all 73, file:line + fix), the two full
reviews of 2026-07-24. **Goal:** clear the real defects AND make the *next* silent failure
impossible-to-miss, not just findable by hand.

## Priority principle

Order by **cost of a silent wrong answer × how soon it bites**:
1. **Flight code** — a silent wrong answer becomes a CONTROL ACTION (the aircraft does the wrong
   thing). Highest cost. First.
2. **Evidence chain** — a silent wrong answer mis-decides the ~$740 purchase or mis-scores a kill.
   High cost, next real event. Second.
3. **Field tools** — a silent wrong answer is discovered only after driving home. Third.
4. **Prevention infrastructure** — runs alongside all three; it is what stops the *next* one.

## Waves

### Wave 1 — IN FLIGHT (three parallel streams, 2026-07-25)
- **Flight-code silent failures** (task #19, opus5): terminal-coast freeze latch; dropout yaw-chase
  (~108°); `offboard_active` latched constant + RcChannelTrigger time-base; stale-own-state stand-in
  steering; `--audit` gt-prefix-only gap; bias/lag inert under `--law pip`. Code + unit tests; **head
  runs the SITL gates** after.
- **Evidence-chain silent failures** (task #20, opus5): R_decode90 walk across unsampled/underpopulated
  bins → UNCERTAIN; curve_b sim-intrinsics → session calib; curve_b 10-25 m UNANSWERABLE guard;
  field_score CPA coarse-grid → analytic per-segment + edge-truncation verdict + ULog test;
  pi_capture Ctrl-C → meta.json in a finally + `decode_loop_fps`; antimirage geometry-match guard;
  tag-size single source.
- **Prevention infrastructure** (task #21, Fable): producer→consumer contract tests (P1 = m4 CSV →
  the auditors); no-vacuous-verdicts (fix audit_per_tick, shared assertion); green≠ran (run_tests
  stage-2 fail-on-skip + stage-4 self-tests + CI mirror); drift number-traceability guard in
  `--check` + fail `--check --artifact` + publish receipt; `docs/error_handling_policy.md`.

### Head-owned (done this turn)
- CLAUDE.md standing rules: **"Instruments are evidence"** (no-vacuous-verdicts / fail-closed /
  contract-test) + **"a fix isn't done until its effect is observed; sweep drift the same turn"**
  (my own two errors of 2026-07-24). These load every session — the durable core of the strategy.
- Already fixed inline before the waves: curve_b truth-box geometry BLOCKER (+ regression test);
  ADR-0082's inert fps band materialised; the §0 "camera defends imperfect aim" drift corrected.

### Wave 2 — after Wave 1 lands
- Review each stream's diff (Fable gap-check on the flight-code fixes before I trust them).
- **Head runs the SITL gates** the flight worker flags (`check_real_flight_sitl.sh`,
  `check_deploy_sitl.sh`) — the one thing the workers cannot do.
- Field-tools + protocol doc edits: `04_pull_logs.sh` honor its own exit contract; the range-truth
  chain + auto-sync geometry into `tripod_test_protocol.md` §7; stale lethal-radius prose;
  laptop↔Pi networking dry run.
- Re-verify the **10 findings the review left unverified** (session limit) before acting on them.

### Wave 3 — close-out
- Contract + artifact republish with the fixed state; a clean `run_tests.sh` (now with the new gates)
  as the definition of done; commit the whole tranche in reviewable milestones.

## The prevention strategy in one paragraph

The class is **silent failure**: confident, plausible, WRONG output with nothing raised, while green
checks pass. Three-layer defense, mechanical over discretionary: **(1) contract tests** at every
file boundary, fixtures from the real producer, so a schema break fails in CI not the field; **(2)
no-vacuous-verdicts + fail-closed** so a tool that scored nothing (or substituted an unmeasured
default) says UNCERTAIN instead of PASS; **(3) drift coverage of the surfaces humans read** (the
hand-authored dashboard layer), plus the standing habit that a fix is proven by observing its effect
and a flipped verdict is swept the same turn. Codified in `docs/error_handling_policy.md`, enforced
in `run_tests.sh` + CI, and pinned into CLAUDE.md so it survives a fresh session.
