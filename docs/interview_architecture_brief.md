# Interview brief — the software architecture, and what did and didn't work

> **Audience:** the builder, in an interview, on a phone. Everything here traces to
> a source in this repo. **Nothing new is claimed** — this is a reading of
> `docs/project_state.json` (the contract), `docs/decisions.md` (97 ADRs), the audit
> and review docs, `README.md`, and the code itself, arranged for a conversation
> rather than for a reader working front to back.
>
> **Rule this doc obeys:** every number carries where it came from and what tier of
> evidence it is (`sim` / `bench` / `derivation` / `retracted`). If an interviewer
> asks "how do you know that", the answer is in the same line as the claim.
>
> **Readable version (phone-friendly, hosted):**
> https://claude.ai/code/artifact/7970c857-927d-4f8f-839f-cd16529a9ce2
>
> Written 2026-09-09. Canonical state always wins: if this doc and
> `project_state.json` disagree, **the contract is right and this file is stale.**
>
> This is a DERIVED view, like `docs/results_notes.md` — it is not part of the
> contract and nothing depends on it. Delete it if it stops paying for itself.

---

## 1. The 60-second version

A counter-UAS interceptor: a quadcopter that flies a pre-programmed open-loop sprint
at a crossing target, then hands off to its **own camera** for the final homing — no
datalink, because the whole point is that the link is the thing an adversary jams.

Two halves:

- **The simulation** — PX4 SITL + Gazebo Harmonic, headless. About 109,000 lines of
  Python and 12,200 of shell; 813 test functions in 57 files (the runner reported 941
  passing tests at the most recent commit that ran it — parametrisation expands the
  count); ten pipeline stages; 97 architecture decision records; Monte-Carlo batches
  on paired seeds. This is the evidence machine.
- **The real build** — Raspberry Pi 5 seeker, Pixhawk 6C Mini, an ArduPilot target
  aircraft. In progress: the target's flight controller is flashed with all 44
  settings verified by reading them back off the board, the camera is measured on
  hardware, and the airframe order is gated on one field afternoon.

**The headline, stated the way the project itself states it:** the guidance law works
and is measured. The camera has **not** beaten a well-aimed blind sprint in any test
flown. Sub-metre interception is real at 10 mph — 8 of 8 flights, median 0.73 m — but
it is *ballistic*, camera off. That gap is the interesting part of the story, and
leading with it is what makes everything else credible.

---

## 2. Architecture: the data path, one hop at a time

The flight is a state machine — `STANDBY → CODED_DASH → ENGAGE → BREAKOFF → SAFE`,
absorbing at `SAFE` (`flight/deploy/real_flight.py`, ADR-0081). Every transition is
guarded and logged, and a setpoint goes out on **every tick in every state**, because
PX4 drops OFFBOARD on a stream gap of about half a second.

Inside `ENGAGE`, this loop runs to contact:

| Hop | What it does | Where it lives |
|---|---|---|
| **DETECT** | Markerless YOLO on **every** frame at 640 px, confidence gate 0.25. No learned tracker. | `scripts/seeker/finetuned_seeker.py` |
| **MEASURE** | Box centre → undistorted bearing; `range = fx · span / box_width_px`. | `flight/camera.py`, `seeker_loop.measurement_from_box` |
| **DEROTATE** | Camera optical → body FRD → NED with the full attitude quaternion; `λ = atan2(east, north)`. Then a camera-to-CG lever-arm correction. | `flight/geometry.py` |
| **ESTIMATE** | Two scalar alpha-beta filters (angle, range): predict every tick at 20 Hz, correct only on a fresh detection (~14 Hz), coast through dropouts. | `flight/estimator.py` |
| **GUIDE** | Pro-nav: `a = N · Vc · λ̇`. Pursuit kept in-tree as the A/B baseline. | `flight/guidance.py` |
| **ACT** | Command → NED velocity + **absolute** yaw setpoint → PX4 OFFBOARD via MAVSDK. | `flight/deploy/seeker_loop.py` |

**The design choice worth naming out loud:** the thing that tracks the target across
frames is **physics, not a second neural network** — an alpha-beta filter plus
proportional navigation. A learned tracker was built, adopted, and later removed (§5.4).

### 2.1 Constants, so they can be quoted

| Quantity | Value | Note |
|---|---|---|
| Pro-nav gain N | **5.0** deployed / FPV; **4.0** in the non-FPV sim default | lab trade study, ADR-0011 |
| Alpha-beta gains | α = 0.5 both channels; β = 0.30 angle, 0.45 range (FPV) / 0.15 (default) | ADR-0009 + addendum |
| LOS rate cap | 60 °/s on the angle channel; **none on range** (parity with the sim harness) | the hazard three latch guards cover |
| Closing-speed floor | 4.5 m/s deployed, 1.5 m/s non-FPV | the range-rate estimate starts near zero from hover, so the lead would build too weakly |
| Control rate | 20 Hz; camera pinned 60 fps; detector ~14 Hz | staleness bounds 0.25 s, well under PX4's 1.0 s offboard-loss timeout |
| Terminal freeze range | 3.5 m | λ̇ is singular as R → 0 |
| Handoff | **5 consecutive** fresh in-window detections | no new detector result **holds** the streak, since 20 Hz outpaces 14 Hz |

### 2.2 Two conventions worth defending cold

**Coordinate frames.** Camera is OpenCV optical (z forward, x right, y down). Body is
FRD. World is ENU for scoring and NED for setpoints. Headings are compass-style,
`degrees(atan2(east, north))` with 0 = north. A positive bearing means the target is
right of boresight — a load-bearing sign, documented at the seeker's definition.

**Why the derotation chain exists** (ADR-0062, FIX-A). The terminal azimuth was
originally `λ = ψ + β`, vehicle yaw plus the target's horizontal bearing in the camera.
That is correct only when the boresight is level. At the measured 27–36° sprint pitch it
inflates the azimuth by roughly `1/cos(pitch)` — a 12–24% gain error straight into the
guidance input — and under roll it mixes the target's *vertical* off-boresight component
into the azimuth.

The fix rotates the full 3-D camera ray with the attitude quaternion, and carries an
**identity guarantee**: with a pure-yaw quaternion the chain reduces to `ψ + β` exactly,
so every previously validated level-flight result is preserved bit-for-bit and only tilt
introduces a change. A 245-case identity grid pins it to under 1e-9.

Measured effect, paired n=8: median miss 2.172 → 1.897 m, tighter on 7 of 8 seeds and
worse on 0, sign test p ≈ 0.035 (`sim`).

**A second, quieter geometry fix in the same family:** pro-nav commands the vehicle about
its centre of gravity, but the camera measures from the camera, 0.1–0.4 m ahead. The
lever-arm correction rotates the body-frame mount offset into NED, adds it to the ray
endpoint, and re-solves range and bearing. Zero offset returns the inputs unchanged.

### 2.3 The portability split, and how it is enforced

`flight/` is the portable core: 22 files, 9,303 lines, pure Python and `math` in the
math modules, with **no simulator imports, no ground-truth imports, no cue imports**. It
runs unchanged on the real Pi and Pixhawk. The sim harness (`scripts/m4_intercept.py`,
5,482 lines) is the throwaway half.

That separation is not a convention, it is a test — three mechanisms:

1. **An AST import-and-identifier closure audit.** `real_flight.py --audit` walks nine
   vehicle modules and flags any identifier starting with a forbidden prefix, any import
   from a forbidden root, and requires **exactly one** call site that sets the sprint
   heading. Exits 0/1.
2. **Mutation calibration.** Tests copy a module, inject each leak shape, and assert the
   audit returns 1. The audit is proven to *fire*, not just to pass.
3. **A sim-side twin.** `tests/test_honesty_static.py` pins the sim engage loop: a loud
   allowlist for the only two legal ground-truth read sites (the log, and the running
   minimum miss), plus a chain-variable check forbidding a ground-truth right-hand side
   on any of the nine variables that reach the command.

**And the honest limit, stated in the code rather than oversold:** this is a
*naming-convention* check, not an information-flow proof. It cannot see ground truth
handed in by a caller under a clean name. The `ground_truth` prefix had to be added after
a planted read slipped past a `gt_`-only guard, because the repo's real accessors are
named `ground_truth_world_points` and `ground_truth_rel_optical`.

Three constants are deliberately **duplicated** rather than imported across the boundary
(exposure spec, calibration minimum, forbidden-prefix list) and pinned equal by contract
tests, because `flight/` must not import dev tooling that will not exist on the aircraft.

### 2.4 Failsafes, and the one asymmetry that is the whole point

Eight failsafes. The interesting design is that the **radio link and the control link are
monitored in opposite directions**:

- The **RF link** is monitored in `STANDBY` **only**. Post-GO link loss is deliberately
  ignored. That *is* the jam-resistance claim — a failsafe that aborted on it would
  delete the headline capability.
- The **OFFBOARD control path** is monitored **only after** GO, with two arms: mode
  not-OFFBOARD persisting past 0.5 s, and a sample older than 3.0 s or negative, which
  catches a dead subscriber that would make the first arm unreachable.

Other details an interviewer would probe: the GO bit is edge-triggered, hysteretic, and
**refuses a switch already high at boot**; a heading latch raises on a second write; and
the distance failsafe prefers the *measured* ground speed and labels its output
`MODELLED` when it has to fall back — because the acceleration constant it would
otherwise use was measured over-predicting travel about 2.7× in the closest-approach
window.

---

## 3. The honesty boundary — the project's actual spine

If there is one thing to lead with, it is this, because it is the part most portfolio
projects do not have.

**The rule.** Ground-truth topics (`gt_*`) are for scoring and logging only. Guidance
sees camera pixels plus the vehicle's own-state EKF, nothing else. Every new guidance
path re-earns a numeric no-cheat audit confirming commands track the *measured* range
where measured and true diverge.

**The loophole, and who found it** (ledger entry `launch-aim-derived-from-ground-truth`).
The rule above polices *when* a value is read. So "the launch aim is solved from the
target's exactly-known track" passed as clean for months — it is a pre-flight constant,
therefore not a live read. That was load-bearing, because aim turns out to be the
dominant lever, and a zero-error cue quietly reduces the demonstration to a ballistic
solution of a known trajectory. The builder caught it by asking, after audits had missed
it.

**The corrected test:** not *when* the value is read but **whether a real system could
obtain it at that quality**. An external cue is architecturally fine — the concept is
cheap interceptors cued by smarter sensors. A cue with *zero* error is not, because no
real cue has zero error.

So every input the system is **given** rather than **measures** now carries a graded
register entry, and a number computed on a `given-perfect` input is reported as a
**best-case upper bound**, never as the claim.

| Assumption register, today | Count |
|---|---|
| Registered assumptions | 20 |
| Actually measured | 1 |
| Supplied error-free (upper-bound inputs) | 4 |
| Currently marked violated | 5 |
| Planned, not yet tested | 8 |

The four error-free inputs: the launch cue, a windless world, a wind-immune target, and
an unmeasured drag table. The five violated include "the camera terminal earns its
handoff" and "the scorer measures the criterion".

---

## 4. What worked

### 4.1 Pro-nav beats pursuit, camera-only — `sim`
Pro-nav 0.402 / 0.277 / 0.443 m against pursuit 2.544 / 2.109 / 2.048 m on identical
target paths — **4.6 to 7.6× better** — on a 2 m/s crosser with the camera as the only
target sensor (M4 gate, ADR-0009, committed logs).

The engineering underneath the number is what to talk about:

- **Raw bearing rate is structurally near-zero** when the yaw loop is actively nulling
  bearing. The LOS rate must be reconstructed in a yaw-compensated inertial frame — the
  standard strapdown-seeker correction. Three independent reviewers flagged this
  unanimously *before* any code was written.
- **Fair by construction.** Both laws share the same actuation and the same yaw,
  altitude and closing loops. The lateral term is the only independent variable.
- **Crossing geometry, not head-on.** Head-on is undiagnostic: with no LOS rotation the
  two laws are indistinguishable.
- **A bench mode that tests the sign convention** before any run is trusted — spin in
  place against a static target and confirm the rate stays near zero. Measured mean
  1.24 °/s under a ±20 °/s spin.
- **Asymmetric gate criteria, by design.** Pro-nav had to be clean *and* under 1.0 m;
  pursuit only had to genuinely fly the same engagement. Requiring pursuit to be "clean"
  would have contradicted the experiment — its failure to close *is* the result.
- **Disclosure rather than selection.** Three earlier dev-phase pro-nav flights that
  same night read 1.04–1.12 m, over the gate, before the final configuration. That is
  stated in the ADR and in `docs/results_notes.md` rather than omitted.

### 4.2 The diagnosis that reordered the project — `sim`
Sixty of sixty fast-crosser flights ended "lost detection at closest approach", miss
about 1.4 m. The intuitive reading is a perception problem. Per-tick forensics over 41
flights says otherwise:

- Miss tracks **zero-effort miss at handoff** with **r² = 0.957** (n=41, 6 m/s). It
  *sharpens with speed*: 0.818 at 3 m/s, 0.957 at 6, 0.994 at 9 — as it should, because
  the fit loosens as terminal correction capacity approaches the delivered miss.
- The bound that does not depend on the simulator: correction capacity is `½·a·t_go²` =
  `½ · 8.7 m/s² · (0.41 s)²` = **0.72 m**, against **1.69 m** already delivered at
  handoff.
- The decisive disproof: the worst flights held the camera locked to short range and
  **still** missed — one by 3.37 m with detections down to 3.47 m. A *perfect* terminal
  camera, point-mass over 150 seeds, removes 100% of the dropout and cuts the miss only
  **25%** (1.003 → 0.755 m).
- The last real detection lands 0.074 s before closest approach at 1.69 m range; the
  blind-window contribution to the miss is **−0.03 m**, i.e. nothing. Field-of-view
  escape is real (LOS rate 485 °/s mean, peaks to 1870, against achieved yaw ≤ 124 °/s)
  but happens 0.037 s before closest approach, as a *consequence* of a miss already
  committed.

Root-cause split of the 1.4 m: ~70% kinematic, ~20–25% recoverable mechanisation loss,
~2% field-of-view escape, ~0% blur, scale or cadence.

**Why it is a good story:** it inverts the intuitive answer with evidence, produces a
bound independent of the simulator, and reprioritised everything downstream. Capacity
scales with time-to-go squared, so acquiring at 12 m instead of 6.5 m raises capacity
from 0.72 m to about 4.3 m. Terminal perception polish went out; acquisition range and
handoff track quality came in.

**And one caveat to state before being asked:** the popular phrasing "96% of the miss is
locked in at handoff" was **wrong and was corrected** — r² is *variance explained*, not
fraction-of-miss; the capacity reading implies about 75% locked in. Two of three
independent audits raised it, and the surfaces were re-worded.

### 4.3 An extended Kalman filter, A/B'd and rejected on evidence — `sim`
The most-asked guidance-and-control interview question, answered with a measured result
instead of an opinion.

A Cartesian constant-velocity EKF was built behind a flag (byte-identical default), then
run in a **pre-registered** paired-seed A/B — n=8, one master seed, identical cue seeds
per run, the estimator as the only difference:

- Primary metric, miss: **null**, exactly as pre-registered. Paired Δ = −0.009 m, 95% CI
  [−0.127, +0.117].
- Secondary, track RMSE: also null. Every paired delta straddles zero (position-track
  n ≈ 780 ticks per arm).
- The finding the headline metric **hid**: alpha-beta clean 8/8, EKF clean 2/8.

**Then the honest part.** The first write-up traced that regression to untuned process
noise. Later forensics showed that was wrong: all six "aborted" EKF flights had
intercepted at normal quality (0.675–1.339 m; three of six *beat* their paired twin), and
the aborts fired about five seconds *after* the pass — the EKF's **physically correct**
post-pass range growth crossed a branch into an abort clock, while alpha-beta's
**physically impossible** negative coasted range, −12 to −21 m, stayed under the
threshold and rode the terminal-hold branch forever. A replay screen found no
over-wide-covariance signature at all. The decision does not flip — alpha-beta stays
default, the nulls stand — but the project **withdrew its own explanation**.

There is also a clean derivation to offer if pushed: alpha-beta *is* the steady-state
Kalman filter for a constant-velocity target at fixed rate and fixed noise, with the gain
frozen and the covariance bookkeeping discarded. So what a real filter buys is exactly
the two frozen assumptions — variable sample interval, and time-varying measurement noise
(camera range noise scales with range). Neither moves a kinematically capped miss.

### 4.4 The measurement discipline
This is what most distinguishes the project, and it exists because failures forced it
into existence.

- **Paired seeds, n ≥ 8, with control arms.** Run-to-run terminal-dropout noise is about
  1 m, so a single-flight delta below that is noise — and claims say "not significant at
  this n" when that is the truth.
- **Pre-registration.** Before any run that could change a belief: the configuration, the
  prediction, the adopt-or-reject criterion, **and what a null would mean**, in writing,
  then it flies. A criterion chosen after seeing the numbers is not a criterion. It has
  been committed *before* the run and cited afterward.
- **Simulation time, never wall time.** Real-time factor sags to 0.3–0.5 under load. This
  cost five development runs and two failed gates: the target mover scheduled its path in
  wall time, so a "2.0 m/s" target moved about 4 m/s in simulation terms and every
  engagement degenerated into a matched-speed tail chase *by construction*. The tell was
  the autopilot's own log showing near-perfect setpoint tracking over 8.8 simulation
  seconds that the telemetry recorded as 18.6 — exactly the ratio.
- **The lab ranks, Gazebo decides.** A design-time surrogate with six documented
  divergences ranks options; only a full-simulator gate turns a ranking into a conclusion.
- **A threshold validated at one operating point is not validated at another.** The
  past-closest-approach breakoff works at 9 m/s, where true range falls about 1 m per
  detection, and silently breaks at 4.5 m/s, where it falls 0.15 m — comparable to the
  noise. Descending the speed ladder would have manufactured a false "slower is worse".
- **Scripted gates.** Eighteen `scripts/check_*.sh`, each exiting 0 or 1. No milestone is
  claimed without running its gate and showing the output.
- **Statistics stated properly.** Wilson and Clopper-Pearson intervals rather than the
  normal approximation, and hit rate computed **per arm, never pooled** across speeds or
  path types, because a pooled curve hides the hard end.

### 4.5 Test architecture, and the "green means ran" rule
The count is not the interesting part. These are:

- **Stage two routes the model-inference parity tests to the one environment that has the
  runtime, and fails if anything skips.** Before that existed those tests skipped
  themselves and had *never* exercised the real inference path — more than a hundred
  tests passed and nothing ran them together.
- **Undeclared skips fail the suite**, each allowed skip named with a human's written
  reason. This found two tests that had been *structurally incapable of running* since
  the day they were written — one called a function that did not exist, one pointed at a
  path that has never existed in any commit — both guarding numbers feeding a ~$740
  purchase decision.
- **Coverage sets are data, not prose.** Every tracked self-test pack must appear in the
  runner, checked against version control. The continuous-integration exclusion list is
  verified by a test that re-collects the whole suite with the simulator bindings blocked
  and asserts the erroring set is *exactly* the declared list — written because a
  hand-maintained list rotted by three files in one day, and an unlisted file is a
  collection error that kills the entire stage rather than skipping one file.
- **Producer-to-consumer contract tests, with the fixture generated from the producer's
  own writer.** A hand-typed fixture is exactly what hid two schema breaks: both scripts
  passed their own self-tests and nothing tested the pair. Eight such seams are covered.
- **Guards against vacuous verdicts**, mutation-verified: an auditor must return VACUOUS
  rather than PASS on zero units, and an anti-mirage comparison must refuse unmatched
  arms.

### 4.6 Hardware bring-up, and one measurement that moved a decision — `bench`
- The Pi 5 runs the fiducial seeker at **96.6 fps** sustained through a 750-second soak,
  57.9 °C maximum, no throttling. The project had *assumed* 30. Root cause was an
  inherited camera-library frame-duration default, not the sensor (143 fps capable) and
  not the processor. Range burned forming the handoff streak scales inversely with frame
  rate, so this cut it about 3.2×.
- Neural inference on the same processor manages **6.09 fps** at 162 ms — inside the
  predicted 5–10 band and **not viable** at terminal LOS rates. So the markerless path
  genuinely requires an accelerator and first real flights fly the fiducial baseline. A
  measurement that closed an argument in the unwelcome direction.
- The deployed camera source measured **60.27 fps on hardware** at its pinned rate,
  exposure **994 µs**, inside the sub-millisecond specification.
- OFFBOARD control verified over a **real serial link with props off** — the one
  connection the simulation never exercised.
- The seeker source now logs what was **applied**, not what was intended, because a
  mutation test that deleted the assignment still saw the old log claim success.

### 4.7 Systems engineering as generated views, not a parallel model
Asked whether to organise the project in Capella, the recognised aerospace tool. The
answer was no, and the reasoning is the interview-worthy part: the state contract **is
already the model** — functional chain, requirements, decisions with rationale, graded
assumptions, verified quantities with provenance, open items. What was missing was not a
model but *views* of one.

So a renderer generates seven views with no hand-authored content, and bakes a SHA-256 of
the contract into the page so the check **fails** if the contract has moved. A
hand-maintained model drifts, and drift is this project's documented failure mode; a
stale systems model shown in an interview is a liability the first time a reader
cross-checks it. Mutation-verified: perturb one field and the check exits 1 naming both
digests.

A companion guard closes the obvious hole: the drift check also requires **every visible
decimal number in the hand-authored dashboard layer to appear verbatim in the contract**,
so numbers cannot drift into the prose a human actually reads.

---

## 5. What didn't work

Longer than the last section, and that is the point. Every item is a committed, dated
retraction rather than a quiet deletion. The through-line: **most retracted results
root-caused to measurement code, not to experiment design.**

### 5.1 The five mirages

| Mirage | What it looked like | Root cause | Corrected |
|---|---|---|---|
| Camera-guided sub-metre intercepts | Sub-metre closest approach credited to the camera terminal | The summariser counted **"any detection during ENGAGE" as camera-guided**. A control arm with **zero** ENGAGE ticks scored 7/7 under 1 m, median 0.54 m — as good or better. Engage began 0.1–1.1 s before closest approach on a *phantom* streak with LOS error ±120–140°; pro-nav on a body-fixed phantom is a no-op | Sprint-only 0.30 m vs with-terminal 0.29 m. The camera added essentially nothing |
| The rebalanced retrain ships | A phantom-free detector winning 16/16 | **A vacuous selection.** Coverage was 0.000–0.019 on all 16 flights and every one aborted "no camera acquire" — so the recorded miss was the *open-loop sprint's* closest approach. The hit-rate headline alone would have shipped a blind seeker | 0/16 acquisitions. Graveyarded |
| Resolution and cropping as the acquisition lever | Recall 47% → 71% from cropping | **The hit test was backwards** — it asked whether the true point fell inside the padded *detected* box, so a 376×368 shadow blob containing a 60×60 target scored a hit | Identical recall at the 8–12 m wall band: 33% vs 33%, and 0% vs 0% |
| The up-tilted camera capture | A physics reading good enough to feed the hardware plan | **The ground-truth labeller was tilt-blind** — the tilt lives in the sensor pose *inside* the camera link and the labeller composed the wrong chain, so every truth box was projected as if the camera were level. That the tilted result replicated the level pattern was the *tell*, not the signal | Invalid, retracted before it reached the hardware plan |
| A static range-and-aspect detection wall | Recall collapsing at range and off-aspect — a sensor limit | The same scoring artifact, plus bins that were 75–87% grounded takeoff ticks and an earlier version that counted post-pass receding ticks as "approach" | The static wall **does not exist**: 100% recall at every range 8–25 m, both aspects, at both confidence thresholds |

A near-sixth, in the same family: frame-top recall 19% and banked-attitude recall 0%,
which happened to *confirm the story the project already believed*. The truth boxes were
labelled at boresight scale with no off-axis widening, against a detector that over-boxes
about 2×, so the size-ratio gate false-rejected exactly the detections being measured.
Re-scoring flipped frame-top **19% → 95%** and banked **0% → 100%**, with the
clean-negative control unchanged at 0/692 — proving the fix did not merely loosen the gate.

### 5.2 The zero-command terminal — the best story in the repo

A single uninitialised variable made the camera arm *brake instead of steer* for whole
engagements, and the honesty auditor was structurally incapable of noticing.

**The mechanism.** Two locals were initialised to zero and, in the coded-sprint branch,
**never assigned** — that branch builds its command directly. The terminal-freeze latch
then captured `(0.0, 0.0)`. Because the markerless detector's monocular range at handoff
reads 1.5–3.5 m while the true range is 10–12 m, the vehicle was *already inside* the
3.5 m freeze radius on the **first** ENGAGE tick, so the block that computes those locals
never ran. Every later tick re-issued a zero horizontal velocity.

**Incidence, measured from the flight logs rather than asserted:** one arm was 7/7 flights
and **661 of 668 ENGAGE ticks (99%)** zero-command; across six arms, **29 of 44 engaged
flights were more than 75% zero-command**.

**Why nothing fired.** The run log literally printed the smoking gun — "velocity vector
frozen at (0.00, 0.00) world" — and no check read it. Worse, the per-tick honesty audit's
only check that tests whether the command follows the camera **discards zero-command ticks
as missing data**. On the broken flights it dropped 88 of 89 usable rows, leaving n=1,
which tripped its own "underpowered" branch — and the flight reported PASS.

> **The line to quote:** the worse the guidance failure, the fewer rows survive the
> filter, the more certainly the check stops gating. *The audit was anti-correlated with
> the defect it existed to catch.*

**Why it manufactured a fake effect.** The defect lives only in the ENGAGE terminal, so it
fired on nearly every camera flight and on **0 of 8** sprint-only flights. The control arm
**structurally could not experience it**. A camera-arm *handicap* therefore read as a
*seeker deficiency*, and the direction of every camera-versus-sprint verdict was
confounded.

**How it was resolved, and this is the honest half.** The fix ported the deployed loop's
law-aware construction so each guidance law latches its own vector, one code path shared
by sim and hardware. A new gating check fails any run that is mostly zero-command. The
verdicts were downgraded to *pending re-fly*, and the 40-flight camera-arm fleet was
re-flown that evening — **reusing the sprint-only twins**, because re-flying them would
have broken the pairing.

The re-fly **confirmed the direction**: zero of 674 pre-closest-approach ENGAGE ticks were
zero-command, and the camera beat its twin on 3/8, 1/8 and 4/8 seeds — never near the
pre-registered 6/8 bar. It also produced one honest **softening**: at correct aim the
camera arm is now roughly at **parity** with the open-loop sprint rather than a slight
liability. And the re-fly surfaced two further blind spots in the auditor itself, both
fixed.

**A residual, tracked rather than hidden:** a second, smaller zero-command path survives —
24 ticks across 5 of 16 flights, worst case five consecutive zero ticks while closing from
7.3 to 4.6 m at 13 m/s. The new check only fails above 50% zero-command, so a 24-tick
burst sails under it. The prescribed fix requires a stricter check **demonstrated to fire
on the old logs** before being trusted on new ones.

### 5.3 The seventh instrument defect was inside the fix for the sixth

A re-score claimed the miss scorer ranged to the camera lens, "verified at **+0.208 m**
above the airframe datum, median over 1,748 ticks", and that correcting it would move the
adopted configuration from **3/16 to 12/16** inside the kill radius — reversing the
project's central negative finding. It was published to the repository front page, the
contract, and the dashboard.

**The defect.** That figure is the median of a *world* z minus a height above the *takeoff
point*. Subtracting one from the other measures the **landing-gear height**. It looked
like a lever arm and had the magnitude of one, so nothing questioned it.

**The true geometry, three independent ways.** The resolved model chain gives +0.002 m. The
physics — lowest collision geometry against the on-pad reading — gives +0.002 m. The
on-pad horizontal readings match the declared offset to four decimal places. Decisive: a
0.242 m mount would make a *parked* vehicle report 0.469 m. It reports 0.229 m.

**The near-miss.** The proposed patch was to return the model origin instead. The model
origin is 0.240 m *below* the airframe datum. It scores 13/16 — a **larger** bias than the
reference it replaced, in the **flattering** direction. A fix for a wrong ruler that would
have installed a worse one, and it would have shipped looking like a correction.

**Why the third pass caught it.** Not "checking harder"; every step had been checked. The
re-scorer *had to state its datum explicitly* to compute anything, and it carries a
pad-height self-check that fails closed — 168 of 168 pass. The lesson: a derived quantity
must carry **what it was measured from**, and a tool forced to name its reference frame
surfaces an error that prose comparing two numbers never will. The tool also reproduces
the *wrong* published table to within 0.0004 m under a mode named `adhoc-published`, which
is how it proved what produced the old figure rather than merely asserting it was wrong.

**Corrected result: 3/16 logged, 5/16 interpolated.** Across 21 arms exactly one
inside-count moves at all. The interpolation fix is real, worth about 0.03–0.06 m, and had
simply been bundled with a vertical drop that does not exist.

**And the retraction *restored* a finding:** re-measured at the corrected centre-to-centre
closest approach over 168 flights, the vertical bias is **median −0.374 m** against
−0.375 m at the camera. It does not move. With horizontal error at 0.174 m, **vertical is
essentially the whole miss** — on a vehicle whose lead solve is explicitly two-dimensional
and whose camera measures target elevation every frame and discards it.

Method integrity worth knowing: yaw is calibrated per flight from on-pad ticks (this world
carries a ~6.2° yaw-datum offset), cross-checked against each flight's own ground-truth
velocity heading; interpolation is closed-form per segment with **no clock in the
computation**, so the wall-time hazard does not apply; 168/168 flights scored, 0 rows
dropped, 0 closest approaches claimed across a logging gap; and the kill radius is
hardcoded **with a test asserting there is no way to tune it**.

### 5.4 Detect-then-track: adopted, then removed
A learned-tracker hybrid — the network acquires once, a classical tracker follows frame to
frame, the network re-validates every eight frames — was the fix for a real problem. At
12 m/s with manoeuvring, the detector threw high-confidence phantom boxes about 18 m off
the true target, and **confidence was inverted**: phantom median 0.34 against real 0.17,
so a confidence threshold is structurally dead. The hybrid recovered post-handoff hit rate
from 3/16 to 14/14 with **zero** phantom handoffs, and the honesty boundary was re-earned
with two independent belts plus a mutation-calibrated static pin.

Then the flat billboard target was replaced with a properly banking 3-D quadcopter, and
**every** tracker tested — the classical one and a learned transformer tracker (0/8) —
*hurt*. With a phantom storm of a median 17 boxes per frame in the 8–12 m band, the
tracker's *seed* is the weak link: a tracker locked onto a phantom is worse than no
tracker, because it is temporally **stable about being wrong**. Network-only on every frame
won the paired A/B, and the adopted configuration today has no tracker.

**Say the lesson out loud:** the tracker was validated against a target shape that
flattered it. A flat board does not bank.

### 5.5 Rejected guidance sophistication, each with a mechanism
The fancier law kept losing to the simpler one, for reasons that were *measured*:

- **Predicted-intercept-point lead law** — worked in the offline lab, did not transfer to
  the full simulator. Plain pro-nav is the robust choice.
- **Augmented pro-nav and constant-acceleration filtering** — a kinematic floor plus a
  signal-to-noise wall below one. Even a clean fiducial bearing floors at 1.64 m under a
  12 m/s weave.
- **Bank angle as an acceleration cue** — read the target's roll to feed forward its
  lateral acceleration. Killed at a pre-code feasibility gate on real banked imagery:
  under about 23 px of terminal motion smear, which *spans the whole 8–15 px target*, bank
  signal-to-noise collapses to 1–2 and the acceleration uncertainty is 6–12 m/s²,
  **larger than the 4.7 m/s² manoeuvre signal it would feed forward**. It injects more
  error than signal. Independently, on a straight-line target the augmentation term is
  identically zero.
- **Adaptive (Kalata) filter gains** — won 24–28% in the lab, then **lost every Gazebo
  flight** (3.24 m against a 1.09–2.25 m cluster). The mechanism was caught in the logged
  effective gains: Gazebo's correction cadence is bimodal — ~14 Hz bursts plus
  multi-second gaps — and the lab-tuned noise parameters went degenerate at *both* ends.
  The range channel dropped to α = 0.031 with β ≈ 0 at burst cadence, so the rate filter
  went deaf and the closing speed pinned at its floor, starving the lead.
- **Look-angle-constrained guidance** — pre-coded, tested, parked: framing at correct aim
  measured net-negative, so there was no framing benefit to convert.
- **Sub-pixel bearing** — a win at n=8 that vanished at n=31. Kept default-off.
- **Impact-angle-constrained approach** — the constraint provably eats lateral-acceleration
  authority, degrading exactly the miss-nulling that sets most of the miss.
- **A guidance recovery behaviour for a jammed cue** — dead-reckon plus a yaw sweep. It
  engaged on every jam flight and was a clean null: on 10 of 16 flights the camera never
  detected the target at all during the sweep. A guidance behaviour cannot fix a
  perception-availability problem, and a lateral sweep cannot bring a *vertically*
  out-of-frame target into view.

### 5.6 A hardware recommendation reversed by measurement
The seeker study recommended a **narrower lens** for longer detection range, explicitly
reversing an earlier wide-field plan. The simulator then measured that 60° is **too
narrow** for fast crossers — it *hurt* the acquisition latch, 0 of 12 — and the
recommendation was rejected. Wide field of view is now a **hard constraint**, not a
tunable, because the ±30° open-loop aim tolerance depends on it.

Two more in the same family: the acceleration-capped sprint costs about 0.34 m of closing
speed even when correctly aimed, so the answer is not to cap but to auto-correct aim; and
the phantom range-plausibility gates fail because a phantom-seeded range estimate makes
the gate reject the *true* closing detections, regressing acquisition from 81% to 25%.

### 5.7 Measurement-layer bugs worth having ready
Beyond the mirages, these have the cleanest mechanisms:

- **The closest-approach scorer for real flights quantised to the log rate.** It took the
  minimum over a resampled grid at the coarser log's interval, so the error is about
  `v_rel · dt / 2` — **1 to 2 m at 20 m/s closing, two to four times the kill radius**.
  Worked example: a true 0.30 m hit scores **2.022 m miss** at a 5 Hz position log. The
  correct closed form was already in the file and ran only inside the self-test, which
  passed 5/5 while hard-coding a fine interval — so the production path was untested by
  construction.
- **A missing `1/s` factor displaced a truth box radially outward, growing with off-axis
  angle** — on precisely the position-in-frame axis the measurement existed to
  characterise. Pre-fix error **53.9 px with 43.5% of truthed frames dropped**; post-fix
  0.0 px and none dropped. It would have printed roughly 100% centre and 0% edge recall —
  again, the story already believed.
- **An invented frame rate decided a ~$740 purchase.** The published threshold divided by
  a hard-coded, unsourced 30 fps. Corrected, the threshold moves from 6.58 m at 30 fps to
  8.96 m at 14. Then the *fix for it shipped inert*: the corrected band constant was
  declared and **never read anywhere**, and re-running the documented command reproduced
  the pre-correction numbers exactly. That incident produced the standing rule that **a
  fix is not done until its effect is observed end-to-end.**
- **Then the next fix substituted a measured number of the wrong quantity** — the
  recorder-loop rate (frame grab plus a synchronous file write, with decoding often off)
  accepted in place of the onboard decode cadence at flight time, because the guard was a
  blacklist rather than a whitelist. Break-even for the decision is 24 fps. A number that
  is wrong but now carries the word "measured" is worse than the invented one.
- **Latched gate inputs.** A pre-arm check read a flag written once at initialisation and
  never refreshed. Generalised into a rule: *if a gate input can be written once and never
  refreshed, the gate does not exist.* A sibling bug fed an absolute timestamp into a
  component polled with an elapsed one, so a staleness age came out large and **negative**
  and the first advertised failsafe could never fire — while the log actively asserted the
  link was healthy. The adopted guard: treat a negative age as not-OK, because it is
  physically impossible and means two call sites disagree.
- **A yaw command that chased the vehicle it was steering.** On a frame with no detection,
  the absolute yaw command was re-evaluated as *live* heading plus *stale* bearing — a
  pure positive-feedback integrator, measured at 90–108° of uncommanded yaw in two
  seconds. Now the absolute command is latched and coasted at the estimated LOS rate,
  bounded by the estimator's rate cap.
- **A range channel with no rate cap.** One oversized or merged box at 12 m injects a
  range rate near −72 m/s; the filter then coasts the estimate through the freeze radius
  and the camera is disconnected for the rest of the engagement, silently. Three guards
  answer it, and notably **none of them rejects a measurement** — that family is
  graveyarded: an arm-count before latching, a release condition, and a health flag that
  inhibits arming and gives a failsafe something to break off on.
- **A shell log-pull printed "OK: 1 file(s)" and a final PASS when zero files landed** —
  copy status unchecked, counter incremented unconditionally. With more than one log
  requested it also handed the scorer the *oldest* file, because a plain copy restamps
  modification time and the newest-first selection inverts.
- **Counting the denominator.** Multiple tools dropped unusable units silently, so a
  shrinking sample looked like a clean one. The rule now: count the drops and print the
  count — "scored N; M dropped: K off-frame, J unreadable".

### 5.8 The gate that was not gating
The test runner was **red at the head of the branch for 12 days across 9 commits**, and
how it survived is the lesson. One commit's message certified "Suite 519 passed" — and 519
is *exactly* what `pytest tests/` returns. The **173-test portable-core half**, which the
runner also runs and where the contradicting test lived, **was never executed**. The
commit certified a suite it had not run, and nine commits landed on top.

Two tests asserted opposite things about one constant. **The code was right and the test
was the stale artifact**, and its docstring asserted a claim the hardware bench had
already falsified.

Found in the same audit: a range calibration where the producer grades its own fit and
**writes the sidecar before that check**, so a three-sample fit the producer itself graded
a failure arrived at the consumer indistinguishable from a 57-sample fit and was consumed
as calibrated. Not cosmetic — that span sets range, range multiplies closing speed into
the guidance command, and it keys every range threshold. Fixed on both sides plus a seam
contract test, mutation-verified: restoring the defect kills four tests.

Continuous integration has its own version of the same story: it **failed 73 of its first
73 runs** while a project surface asserted it was green, root-caused to two independent
causes on a clean clone, plus a workflow detail where later stages default to running only
if earlier ones succeeded — so the stages added specifically to close a coverage hole
**had never once executed**. It is green on the default branch now.

### 5.9 The wind model: a working feature condemned by a broken instrument
Wind was built as a world-side drag force with standard low-altitude turbulence, tiers
sourced to the Beaufort scale rather than invented, and the worst-credible tier written
into the field protocol as a no-fly ceiling so simulation coverage and field exposure
coincide. Then the physics probe found **two defects in the measurement path before any
answer was possible**:

1. The driver computed drag from a **frozen position** — it matched the airframe's *link*
   entity, and a nested link's pose in the simulator's pose topics is relative to its
   enclosing model, so it read a constant however the aircraft flew. All 4,789 pose
   callbacks reported the same altitude, and the vehicle velocity term was structurally
   zero in every applied row.
2. The probe's own file reader fed a provenance banner to the parser as a header, so a
   599-row file read as zero rows.

**Defect two is what saved the project.** With the pose frozen, all phases read 0.000° —
**indistinguishable from "the simulator never applied the force"**. Had the parser worked,
the probe would have returned a confident **null and condemned a driver that was working
correctly**. A false null is the most expensive outcome available: it retires a working
capability and sends you back to rejected alternatives. The probe now fails **void** when
the ground-truth pose never moves.

**The actual answer.** The simulator applies the force essentially exactly: a settled tilt
of **5.259°** against a derivation of **5.262°**, agreeing to 0.003° in magnitude and 0.2°
in bearing. The null's root cause was the *previous* decision's own fix. It correctly
found that persistent forces **append** rather than replace, and fixed the accumulation by
clearing before every publish — but clear and publish go out on **two different topics**,
and there is no cross-topic ordering guarantee, so "clear then publish" frequently landed
as "publish then clear". Same force: **5.259°** published once, **0.167°** under the 20 Hz
clear-and-publish recipe.

**The generalisable finding, and the line to quote:** this is the instrument-defect class
one layer down. Not a bug in a *scorer* but a bug in an **actuator whose log reports intent
rather than effect**. Any component that logs what it *commanded* rather than what was
*applied* can diverge silently.

The arm is parked by builder ruling on cost grounds, and the reasoning it rests on — the
terminal camera absorbing the wind — is recorded as a **declared assumption, not a
result**, because wind's dominant damage is to the open-loop sprint, before any camera
lock. The known one-line-class fix is written down, unspent, so it is not re-derived.

### 5.10 Also tried and buried
| Approach | Why it died |
|---|---|
| Onboard acoustic tracking | Own-propeller noise is about 40 dB the wrong way |
| Live mid-course datalink cue | It *is* the link an adversary jams — and it measured *worse* aim than the open-loop sprint, 0 of 8 against 5 of 8 |
| Autonomous launch on detection | Zero added acquisition range, reaction-time starved against a 9 m/s inbound, and firing on an unmeasured outdoor false-positive rate is the least defensible rule of engagement available |
| Pitch-up reframing pulses mid-sprint | Manufactures the exact attitude swing the design is trying to escape; each relaxation spends 25–33% of the terminal engagement decelerating |
| Pivot to a photoreal simulator | Invalidates roughly 70 measured decisions, and both realism complaints were fixable where we were |
| The fiducial as terminal seeker in a crossing sprint | The directional tag is invisible during a 16 m/s crossing pass — zero detections, worse than the network |
| A sim-trained detector transferring to real imagery | AP50 **0.0003**, recall 1.1%, false-fire on **88.5%** of drone-free frames, and recall **0.000 below 24 px — the entire terminal band**. The real-data retrain reads AP50 0.44 with 4.9% false-fire on the same held-out set |
| Covariance-gated cue fusion | Variance blow-up under the worst-credible cue; the hand-set polar fusion won 8/8 paired |
| Two detector retrains (manoeuvring domain, phantom-rebalanced) | Both ace their own frame evaluation and fail in flight — one regressed all 10 held-out flights, sign test p = 0.002 |

---

## 6. The wall, stated honestly

**The binding constraint is not guidance. It is flight-dynamic detector recall.**

The same detector at the same threshold reads **100% recall statically at 8–22 m** and
**0.8% on the approach in flight** — over 63 flights, **2 real detections against 1,187
phantoms**. The dominant mechanism is pointing: the sprint pitches nose-down 27–36°, which
parks a co-altitude target at the top of the frame or out of it. In the 8–12 m band only
about **25%** of terminal samples have the target in frame at all; when it *is* in view
the deployed detector hits 70%, with an any-box ceiling of 81%.

Every competing explanation was tested and eliminated:

| Candidate mechanism | Verdict |
|---|---|
| Range, resolution, cropping | Eliminated — identical recall at the wall band; resolution matters only past ~24 m static |
| Ground-clutter background | Refuted — 99–100% on ground background against 92% on sky, at every range bin |
| Phantom competition | Confirmed but modest — masks about 12% of in-frame samples; a top-3 read recovers 89% |
| Target aspect | Eliminated — the static wall does not exist |
| **Pointing** | **Confirmed and dominant** |

The pointing fix — a fixed up-tilt mount sized to the *measured* real sprint pitch —
lifted 8–12 m recall from **3.0% to 35.4%** on paired seeds, a geometry change with no
detector change, and centred the target in elevation from −25.7° to −3.6°. It also made
the miss *worse* at 9 m/s, because the acceleration cap it flew with slowed the run-in.
Necessary, not sufficient.

### 6.1 The arithmetic that turns recall into a range requirement
Worth being able to derive on a whiteboard. Time-to-go after handoff is
`(R_acq − R_burn) / V_closing`, where `R_burn = (frames_to_confirm / fps) · V_closing`.
Because handoff needs **five consecutive** detections, the honest model is the Bernoulli
run-length `E[T] = (1 − pᵏ)/(pᵏ(1 − p))`, **not** the mean-rate `k/p`.

At k=5, 9 m/s, 30 fps: perfect detection burns 1.5 m. At the measured in-view recall of
0.70 it burns 4.95 m. At the real-imagery recall of 0.44 it burns **31.4 m** — the streak
is *unformable*. The mean-rate model is therefore optimistic by 2.3× at p=0.70 and
**9.25× at p=0.44, always in the direction that flatters a purchase decision**.

And the discipline: **the fix is deliberately not applied**, because changing it can flip
a published pass/fail on the airframe order, and that must be a logged decision rather
than a quiet patch. (The measured 96.6 fps cuts every burn about 3.2×, but that is the
*fiducial* cadence, not the network's — so "unformable" is re-opened, not overturned.)

### 6.2 Sim-to-real gaps: 30 enumerated, 23 of them flattering
The framing rule is the quotable part: *the single biggest way this blueprint could
mislead the hardware build is a fidelity gap that flatters the design — treat a too-good
simulation number as a bug to investigate, not a win.* Every row names what the simulation
does, what reality does, the severity, the **direction** (flatters / conservative /
uncertain), and the **bench-measurable quantity that would close it**.

The load-bearing ones:

- **No motion blur and no rolling shutter, anywhere.** Named existential in three separate
  decision records and called **the single most-repeated known optimism source in the
  codebase**. Terminal LOS rate is 485 °/s mean, peaking to 1870; at the measured
  pixels-per-degree a 5 ms exposure smears **26 px** across a 3–18 px target, and 1 ms
  smears 5.3. The defence is the sub-millisecond exposure discipline — measured 994 µs on
  hardware — and it explicitly does *not* rescue the closest-approach window.
- **The cue link runs on loopback** — zero loss, no bandwidth ceiling, no radio
  propagation. That is the transport of the headline jam-resistance claim.
- **The adversary's strength is chosen by us.** Spoof and bias injection are entirely
  self-authored; the note says plainly not to fake this with more simulation knobs.
- **Clocks align by construction** — one process, one simulation clock — where reality
  needs pulse-per-second and satellite-disciplined time across two embedded computers. The
  simulation *caught* the real version of this as a clock-epoch bug.
- **An ideal pinhole camera**: zero distortion, exact field of view, no exposure hunting
  or glare, intrinsics that never drift — where a 2% scale error biases every range by 2%.
- **No aerodynamic drag plugin exists at all**, so every speed and acceleration number is
  drag-free, and payload mass, inertia and centre-of-gravity shift are unmodelled.
- **Not flattering, and said so:** there is no thermal or night model anywhere, the
  discrimination interlock defaults to refusing everything until bench data exists, and
  the hardware kill link has no simulation analogue — *which is itself the gap*.

The bench order is prioritised by cost: the blur-and-yaw-rate ramp first, then lens
calibration, then sustained detection rate on the real board. The kill-link latency test is
flagged as a **safety** item that should not be deferred merely because it comes later in
the build.

### 6.3 And one reason the camera could not have won, found 2026-09-09

Every seeker computes the target's vertical look angle. **Nothing in `flight/` consumes
it.** The elevation enters the derotation chain only to get the *horizontal* azimuth
right; the sole vertical command anywhere is an altitude-hold P-loop to a preset height.

ADR-0095 measured the adopted configuration's residual as **0.374 m vertical against
0.174 m horizontal**. So the terminal steers the axis carrying 18% of the squared error
and is blind to the axis carrying 82%. Null the vertical term alone and the median is
0.174 m, **inside** the 0.35 m ram radius.

ADR-0085 **decided** a camera-driven vertical channel with a hard altitude floor in July.
Neither half was ever written, and that ADR appears nowhere in the contract or the queue.

This does not say the camera works. It says the strongest camera-versus-dash comparison
has never been run.

**Two things were then measured, and one of them corrected the first draft of this
brief.** The closing speed in the last second before handoff is **17.60 m/s**, not the
9.0 m/s the airframe purchase gate assumes. Re-derived at that speed the fiducial path
has **0.02 to 0.17 m** of terminal correction capacity, against a need of roughly 0.4 m.
So on the fiducial baseline the short window really is close to a physics limit, and the
earlier claim that capacity exceeded need by two to four times was computed off the
gate's own assumption. The markerless path at a 20 m acquisition range still has 3.9 m.
The limit is the tag's, not the camera's.

The second measurement says the vertical error is **delivered by the dash**: the fleet is
off-altitude at closest approach on 24 of 24 committed flights, and dash-time altitude
drift accounts for 66% of it. So the fix is a pre-flight altitude trim, the analogue of
the crossing-bias aim calibration, not a terminal law with no authority left to spend.
That is **ADR-0099**, built default-off and byte-identical, ten tests,
mutation-verified, and never flown.

Full arithmetic: `docs/vertical_channel_analysis.md`. Ledger entry
`terminal-vertical-channel-decided-not-built`. Pre-registration, committed before any
arm: `docs/vertical_channel_prereg.md`.

---

## 7. Questions to expect, and the honest answer to each

**"Does it work?"**
The guidance law works and is measured. The full camera-guided intercept of a realistic
3-D target does not exist in the dataset. Sub-metre interception is real at 10 mph — 8 of
8 flights, median 0.73 m — but it is *ballistic*, camera off. Nothing yet lands reliably
inside the 0.35 m contact radius that defines a kill: best configuration 3 of 16 logged,
5 of 16 interpolated.

**"Why is the kill radius 0.35 m?"**
Derived, not chosen: the sum of the two ordered airframes' half-spans, 177.8 + 174.8 mm =
a 352.5 mm contact envelope. It replaced an earlier 0.5 m that described two larger
aircraft, and the aim budget re-derives from it to ±2.4°.

**"You quote a 95% hit rate. Against what?"**
A 2.8 m proximity radius, a **flat billboard** target, one path type, one speed, never
pooled — 72 of 72, 95.0% Clopper-Pearson lower bound. The ratified physical contact
envelope is 0.35 m, about **seven times tighter**. The registered assumption that the
proximity proxy stands in for a kill is marked **violated**: the best fleet ever flown,
8 of 8 under 1 m, put **0 of 16** inside 0.35 m. Sub-metre is not a kill.

**"Is it really comms-denied?"**
The status is **held**, deliberately. The handoff latch is real and structural — once the
camera terminal latches, the cue channel is closed and unreadable, so a link jammed
*after* handoff cannot touch the terminal. But a flown eight-arm jamming campaign showed
the cue-era configuration **fails closed** when jammed *before* acquisition, with a clean
dose-response as the jam moves earlier: 12 of 16 real handoffs, then 2, then 0. The
staleness fix validated fail-*safe*, not recovery, and the recovery arm was an honest null.
The real build's answer is architectural — the coded sprint has no datalink to jam — but
that machine has not flown, so nothing is claimed for it.

**"How could a paired A/B miss a bug?"**
Two ways, and the second is worse. A defect in a **shared** instrument is common-mode: it
perturbs both arms in the same direction and largely cancels out of the *difference* while
corrupting the *levels* both arms are reported at. So the control certifies the delta and
says nothing about whether either number means what it claims.

A defect only **one arm can suffer** is not common-mode at all — it becomes a treatment
you did not intend to apply. That is exactly §5.2. Before trusting any A/B, ask which
failure modes are reachable by *each* arm; if a defect is arm-asymmetric, the *direction*
may survive but the *margin* is not quantitative, and you say that in words instead of
quoting a delta.

**"So how do you catch a bug in your own instrument?"**
Four things, all mechanical rather than discretionary. No vacuous verdicts — a verdict
computed on zero units returns UNCERTAIN and a non-zero exit, and every verdict line
prints the n it used and the n it required. Fail closed on measured quantities — never
substitute a default for something that should have been measured, and a file that exists
but does not parse is an error, never a fallback. Producer-to-consumer contract tests whose
fixtures come from the producer's own writer. And a fix is not done until its effect is
observed end-to-end: reproduce the pre-fix behaviour, then demonstrate the post-fix
behaviour.

**"Is the short terminal window a physics limit?"**
No, and separating the two halves is the answer. The *fiducial's read range* is a real
optical limit: AprilTag needs about 22 px of tag edge, so at the ordered lens a 0.35 m
placard reads to roughly 6 m, and the placard is already at the carry limit. But the
markerless detector has a 24 m static ceiling. First kills fly the tag because the Pi
runs the tag at 96.6 fps and the network at 6.09, so the short acquisition range is a
consequence of deferring a $70 accelerator, not of camera guidance.

The *correction capacity* splits by seeker, and the honest numbers use the **measured**
closing speed of 17.60 m/s before handoff rather than the gate's 9.0. On the fiducial
path that leaves 0.02 to 0.17 m against a need of roughly 0.4 m, so there the window
genuinely is the constraint. On the markerless path at 20 m acquisition it is 3.9 m,
which is ten times the need. Capacity scales as time-to-go squared, which is why
acquisition range is the lever, and why a pilot tracking from 30 m has about 18 times
the authority of a seeker whose first look is at 7 m.

**"What would you do differently?"**
Register assumptions from day one. Every mirage traces to something never written down —
the graveyard stops you re-trying dead ideas and the contradiction ledger stops you
re-asserting refuted claims, but neither catches a thing that was never questioned.

And build the measurement tools with the same adversarial scrutiny as the flight code. The
paired-control machinery catches the *symptom*, a too-good result. The *cause* was found
every single time by someone reading the scorer.

**"What is unproven right now?"**
Of 20 registered assumptions, **one** is measured. Five are marked violated. Four inputs
are supplied error-free, so any number resting on them is an upper bound. The largest
single one: the launch aim is solved from the target's exactly known path — a zero-error
cue no real ground sensor provides. Seven of ten pipeline stages are half-done.

**"What is the next thing you would build?"**
The queue is public. First, a replacement for the past-closest-approach breakoff
discriminator, because the measured-range-rise test carries no information — false rises
median 0.175 m against true rises at 0.152 m — and it is arm-asymmetric, so it blocks
everything below it. Then the cue-error sensitivity sweep out to 20, 25 and 30 degrees, to
find the crossover where the seeker starts earning its place. That curve is the
deliverable, and it is also the derived accuracy requirement for the cue.

---

## 8. Numbers, with their evidence tier

| Quantity | Value | Tier | Source |
|---|---|---|---|
| Pro-nav vs pursuit, 2 m/s crosser | 4.6–7.6× less miss | sim | M4 gate · ADR-0009 |
| M3 static standoff error (bar 0.5 m) | 0.018 / 0.035 m | sim | `check_m3.sh` · ADR-0008 |
| M5 final Monte-Carlo, n=96 | median 0.93 m, 96.9% clean | sim | ADR-0036 · `check_m5.sh` |
| Hit rate at 2.8 m radius, n=72 | 95.0% CP lower bound | sim | ADR-0064 (flat billboard) |
| Miss vs zero-effort-miss at handoff | r² 0.957 at 6 m/s, 0.994 at 9 | sim | ADR-0023 / 0027, n=41 |
| Terminal correction capacity | 0.72 m vs 1.69 m delivered | derivation | ADR-0023 |
| Ram / kill radius | 0.35 m | bench | ADR-0084 (airframe geometry) |
| In-flight approach recall | 0.8% — 2 real vs 1,187 phantom, 63 flights | sim | ADR-0076 #18i/#18k |
| Static recall, same detector, 8–22 m | 100% | sim | ADR-0076 #18k |
| Open-loop aim tolerance | ±30°, 48/48 engaged | sim | ADR-0076 #18b/#18c |
| Inside contact radius, best configuration | 3/16 logged, 5/16 interpolated | sim | `docs/rescore_2026-08-10.md` |
| Vertical share of the miss, 168 flights | median −0.374 m vs 0.174 m horizontal | sim | ADR-0095 |
| Fiducial seeker frame rate, real Pi 5 | 96.6 fps (assumed 30) | bench | ADR-0090, 750 s soak |
| Neural inference, same processor | 6.09 fps, 162 ms | bench | ADR-0090 |
| Flight camera source, on hardware | 60.27 fps, exposure 994 µs | bench | ADR-0090 addendum |
| Real-data detector vs sim-trained, real mono | AP50 0.44 vs 0.0003; false-fire 4.9% vs 88.5% | bench | `docs/nn_tier/PLAN.md` |
| Wind force applied vs derived | 5.259° vs 5.262° | sim | ADR-0098 |
| Interceptor bill of materials | ~$740 | bench | `hardware_order_list.md` §0c |

---

## 9. Where to look in the repo

| Want | Path |
|---|---|
| Canonical state: stages, assumptions, ledger | `docs/project_state.json` |
| Human and systems views of the same | `docs/dashboard.html` · `docs/mbse.html` |
| Every decision with rationale | `docs/decisions.md` (97 records) |
| Portable flight core | `flight/` |
| Real-vehicle state machine | `flight/deploy/real_flight.py` |
| Deployed terminal loop | `flight/deploy/seeker_loop.py` |
| Simulation harness | `scripts/m4_intercept.py` |
| Test gate · milestone gates | `scripts/run_tests.sh` · `scripts/check_*.sh` |
| Failure policy | `docs/error_handling_policy.md` |
| Caveat text behind every headline number | `docs/results_notes.md` |
| The silent-failure review (73 findings) | `docs/review2_silent_failure_findings.md` |
| The re-score that retracted a headline | `docs/rescore_2026-08-10.md` |
| Sim-to-real gap register | `docs/sim_to_real_gaps.md` |
| Why the camera has not saved the aim | `docs/vertical_channel_analysis.md` |
| Pre-registered vertical arm | `docs/vertical_channel_prereg.md` |
