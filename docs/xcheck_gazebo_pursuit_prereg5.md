# Pre-registration 5: Gazebo cross-check, brake package on the fixed-wiring baseline

Registered 2026-09-24, BEFORE any flight. Executes ADR-0118 item 5 (the brake
re-measure, decided-and-deferred): the ADR-0115/0116 Gazebo brake verdicts are void —
they were measured under the ADR-0117 velocity-wiring defect, where the cap's closing
term read the commanded velocity. Re-fly #4 established the fixed-wiring baseline
(median 0.343 m, PASS). This arm asks the one remaining brake question: does the cap
help or hurt ON that baseline?

## Configuration (registered)

Identical to prereg 4 in every respect EXCEPT `--pursuit-brake`
(`bash scripts/xcheck_fly.sh 1 8 <outdir> --pursuit-brake`). One change, no tuning.

## Registered prediction

isim (matched optics, ENGAGE fit, pose, correct wiring): brake arms 0.020–0.093 m
median vs no-brake 0.087–0.313 (the prereg3 prediction table — those isim numbers
were never touched by the wiring defect). Honest expectation given re-fly #4's
residual (0.343 vs the 0.09–0.31 band): **median ≤ 0.343 m, plausibly 0.15–0.30 m**,
with closing speed at CPA LOWER than re-fly #4's (the fix-effect the defect
suppressed in re-fly #3).

## Criteria (registered)

Same five as preregs 1–4. **Fix-effect clause B1:** median closing speed over the
last second before CPA is LOWER than re-fly #4's same statistic (same instrument,
both campaigns) — the clause that was red in re-fly #3 for the now-understood reason.

**Adopt:** recommend `--pursuit-brake` in the flying recommended-config iff the five
criteria pass AND median ≤ re-fly #4's 0.343 m AND B1 green. **Null branches:** bar
passes but median > 0.343 (brake costs on this plant at nominal — keep it
practice-profile-only, where it is already load-bearing for rehearsal); or B1 red
again (the cap still is not acting — trace before anything else).
