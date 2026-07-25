# Counter-UAS Interceptor — Sim-Proven Guidance, Real Build In Progress

A quadcopter interceptor that flies a **coded open-loop dash** at a moving
target, then finishes with a **camera-only proportional-navigation terminal**
— no datalink, no ground cue, nothing steers it mid-course except its own
camera. The guidance core was built and validated in **PX4 SITL + Gazebo
Harmonic** across a ~70-ADR measured campaign (2026-07-04 → 07-16); since the
**2026-07-15 real-build pivot** the repo also carries the physical
interceptor's build path, with Tier-1 hardware **ordered 2026-07-20**.

This is a portfolio project for aerospace internship applications. The claim
discipline is the point: every number below traces to a committed log, a gate
script, or an ADR in [`docs/decisions.md`](docs/decisions.md) — and the
project's retractions are documented with the same care as its wins, because
**the negative results are load-bearing**. The honest resume line:

> *"Proportional-navigation terminal guidance handed off from a no-datalink
> coded dash, validated by Monte-Carlo miss statistics in PX4/Gazebo SITL;
> quantified the perception acquisition envelope and identified the
> flight-dynamic detector-recall limit."*

**Canonical live state:** [`docs/project_state.json`](docs/project_state.json)
(the machine-readable contract: stage statuses, constraints, a contradiction
ledger, and a dead-ideas graveyard), rendered to
[`docs/dashboard.html`](docs/dashboard.html) and hosted at
<https://claude.ai/code/artifact/eb5e40d1-c12a-4b87-bca0-589ad5af96fc>.
Where anything in this README and that contract disagree, **the contract
wins**. Mission and scope history: [`GOALS.md`](GOALS.md); milestone roll-up:
[`PROGRESS.md`](PROGRESS.md); working front: [`NEXT.md`](NEXT.md).

**Read "What is proven / what is not" before the results table.** Several
headline-looking numbers from this project's own history were retracted by
its own audits; this README quotes only what survived.

---

## Current mission (the real build)

**An imprecise coded-dash interceptor hits a moving target flying ≥9 m/s
(20 mph), outdoors, camera-only markerless terminal.** Success is a **binary
kill** (contact), confirmed by seeker video + phone slow-motion + both
aircraft flight logs — deliberately *not* a measured sub-meter CPA (the RTK
metrology pair was cut with the 2026-07-15 binary-kill re-scope). The
AprilTag is sanctioned for calibration, training-time auto-labels, and the
staged first-kill baseline seeker — never as input to the deployed markerless
seeker. Source: `docs/project_state.json` (goal), `docs/real_build_coded_dash.md`,
`docs/hardware_order_list.md` §0b/§0c.

Hard constraints (full list with evidence in the contract):

- **No guidance datalink.** The dash is open-loop; the RC link is kill/arm
  only. Jam resistance by architecture, not by protocol.
- **Wide FoV (~100°) is non-negotiable** — the ±30° open-loop aim tolerance
  depends on it (ADR-0024; a 60° lens was flown and rejected).
- **Pi 5 CPU flies the AprilTag baseline in real time; markerless YOLO
  requires the deferred Hailo NPU** (~5–10 fps on CPU is not viable at
  terminal LOS rates). First real kills therefore fly the tag.
- **Prop clearance is a geometry problem** (nose-cantilever camera mount +
  a hard bench gate) — every software fix for the own-prop phantom NULLed.

---

## What is proven, and what is not

**Proven (gated, committed evidence):**

- **The guidance law and loop.** Pro-nav (`a = N·Vc·λ̇`, N=5) beats pursuit
  4.6–7.6× on a 2 m/s crosser, camera-only (M4 gate); the two-stage
  dash→camera-terminal architecture holds to 12 m/s maneuvering targets
  against a cooperative (billboard/AprilTag-class) sensor picture. The
  portable [`flight/`](flight/) core (LOS derotation, alpha-beta estimation,
  pro-nav, undistortion, camera lever-arm) is pure math with its own test
  suite and no sim imports — it runs unchanged on the real Pi.
- **The methodology.** Paired seeds (n≥8), control arms, pre-registered
  verdict scripts, a numeric no-cheat audit on every guidance path, and a
  statistics-before-verdicts rule. This machinery caught every false
  positive below *before* it shipped.

**Not proven — the honest wall (this is the interesting part):**

- **No camera-guided intercept of the realistic 3D quad target exists in the
  dataset.** When the flat billboard target was replaced with a proper 3D
  quad, a series of sub-meter "camera-guided" results were retracted as
  **open-loop dash-ballistics mirages** — a control arm with the camera
  contributing nothing scored the same (dash-only 0.30 m vs with-terminal
  0.29 m; ADR-0076 add #18g/#18h). Five such mirages were caught and
  retracted in that arc alone (ADR-0076; `docs/project_state.json`
  graveyard).
- **The binding wall is flight-dynamic detector recall, not guidance.** The
  deployed markerless detector reads **100% recall statically at 8–22 m**
  but **~0.8% on the approach in flight** (ADR-0076 add #18i/#18k). The
  un-eliminated mechanisms are ground-clutter background under the
  nose-down dash pitch and phantom competition — both sim-testable, both
  open. The dash pitches nose-down ~27–36° (median; ADR-0060), which parks
  a co-altitude target at the top of the frame; the pointing fix is a fixed
  up-tilt mount sized to the *measured* real dash pitch (build item, not
  yet validated).
- **"Works comms-denied" stays HELD** — see the box in the results section.

---

## Headline results (each traced; scope stated inline)

| Result | Number | Scope / caveat | Source |
|---|---|---|---|
| M0–M2 foundations (boot, camera, AprilTag detection) | detection rate 1.000, mean pose error 0.0861 m @ ~4.9 m | wide (99.7°) lens; sim lighting | gates `check_m0/1/2.sh`, 2026-07-04; `PROGRESS.md` |
| M3 static intercept, hold 2 m standoff | final error **0.018 / 0.035 m** (bar < 0.5 m) | two verifier-confirmed runs | `scripts/check_m3.sh`; ADR-0008; committed `logs/m3_intercept_*.csv` |
| M4 pro-nav vs pursuit, 2.0 m/s crosser, camera-only | pro-nav **0.402 / 0.277 / 0.443 m** vs pursuit **2.544 / 2.109 / 2.048 m** | official gate-config runs; three earlier dev-phase pro-nav flights that night read 1.04–1.12 m, over the gate, before the final config (disclosed, ADR-0009 addendum) | `scripts/check_m4.sh`; ADR-0009; committed `logs/m4_intercept_*_20260705T03*.csv` |
| Two-stage handoff (S2), 6 m/s crosser | miss 1.1–2.3 m, handoff latches, honesty audits pass | proves the *architecture* (running start + structural handoff), not sub-meter precision | `scripts/check_s2.sh`; ADR-0010/0013 |
| Why fast crossers miss ~1.4 m (41-flight forensics) | terminal correction capacity **½·a·t_go² ≈ 0.72 m** vs **1.69 m** already delivered at handoff → a perfect terminal camera cuts the miss only ~25% | miss tracks zero-effort-miss at handoff with r² = 0.96 — *variance explained*, not "96% of any one miss" | ADR-0023/0027; `docs/terminal_diagnosis.md` |
| M5 final Monte-Carlo, n=96 (pursuit vs pro-nav × 6/9/12 m/s × line/maneuver/oblique) | **96.9% clean** (ran to completion and engaged; failures stay in every Pk denominator), mean miss 1.08 m, median 0.93 m; per-speed Pk per ADR-0025, never pooled | flew the **clean AprilTag sensor** (the disclosed perception upper bound), flat-board target; laws tied within the ~1 m run-to-run noise at these speeds | ADR-0036; `scripts/check_m5.sh`; committed `logs/mc_final_all.csv`; plots `docs/images/m5_*.png` |
| Markerless seeker v2 (kill the AprilTag) | false-detection pollution **0.751 → 0.000**, range honesty **0.056 → 0.935**; ~+1 m median miss vs the tag = bearing *quality* (box-center vs subpixel corners) | in-sim markerless; guidance-side recovery levers pre-registered and NULL | ADR-0038/0040/0042/0043; `scripts/check_seeker_v2.sh` |
| Detect-then-track maneuvering terminal (billboard era) | post-handoff camera-terminal Pk@2.5 m: weave **3/16 → 14/14**, jink **1/8 → 14/15** (paired n=16 baseline reads 3/15 → 14/15); phantom handoffs 12 → 0; zero gross (>8 m) false terminal detections (0/155) in the headline arm | one empty Gazebo world (gate radii never saw clutter — disclosed); **the CSRT tracker was later dropped for the 3D quad target** — every tracker slips on a banking quad; the deployed config is NN-only every frame (ADR-0076 add #2) | ADR-0058; committed `logs/mc_t21_*.csv`; `scripts/check_t21.sh` |
| Pk statistics hardening, n=72 | **Pk@2.8 m 72/72 — 95.0% Clopper-Pearson lower bound** (clears the ratified ≥95%-CI bar); Pk@2.5 m 71/72 = 98.6% point / 92.5% CP-LB | **flat-billboard target**, weave path + 12 m/s only, never pooled across paths/speeds, radius always stated; the realistic 3D-quad target later exposed the perception wall this target shape masked | ADR-0064/0025; committed `logs/mc_pk72_weave_s*.csv` |
| Perception wall, quantified | in-flight approach recall **0.8%** vs **100% static** at 8–22 m, same detector, same threshold | the wall is flight-dynamic (pointing + background + phantom competition), **not** range/resolution/aspect — each of those was tested and eliminated | ADR-0076 add #18i/#18k; `scripts/seeker/approach_recall.py` |
| Open-loop dash robustness | 48/48 flights still acquired/engaged with up to **±30° aim error** | dash-robustness only — *not* perception proof (ENGAGE streaks can be phantom, add #18g) | ADR-0076 add #18b/#18c |

> **Comms-denied status: HELD — tested, not merely untested.** The one-way
> handoff latch is real and structural: once the camera terminal latches, the
> cue channel is closed and unreadable, so a link jammed *after* handoff
> cannot touch the terminal. But the flown 8-arm jam Monte-Carlo (ADR-0059)
> showed the cue-era deployment config **fails closed** under a jam
> *before* camera acquisition (real handoffs 12/16 → 2/16 → 0/16 as the jam
> moves earlier — a clean dose-response witness); the staleness fix
> validated **fail-safe, not recovery**, and a dedicated recovery arm was an
> honest NULL (the camera never reacquired at 15–21 m — a perception limit,
> not guidance). So "works comms-denied" is **HELD everywhere in this
> project's materials**. The real build's answer is architectural: the coded
> dash has **no datalink to jam** — but that machine has not flown yet, so
> nothing is claimed for it.

Superseded results (ADR-0028/0030/0031 running-start and degraded-cue
figures, the ADR-0029 hover-geometry regime map, the cue-era fusion numbers)
are kept in `docs/decisions.md` with their supersession notes and are not
quoted here.

---

## Architecture (current: coded dash → camera-only terminal)

One NN, the whole flight — there is no learned tracker. The "tracking" that
flies the intercept is physics: an alpha-beta filter plus proportional
navigation.

```
CODED_DASH  open-loop collision-lead heading from pre-flight target
            kinematics (a constant, not a live sensor read) + per-direction
            crossing bias; 5 consecutive fresh detections hand off
   ↓
DETECT      markerless YOLO (drone_finetuned_quad_v2 @640) on EVERY frame
MEASURE     box center + intrinsics (Brown-Conrady undistort) → bearing;
            calibrated box width → range
ESTIMATE    full-attitude LOS derotation (FIX-A) + fixed-gain alpha-beta
            filters; predict 20 Hz, correct on detections, coast dropouts
GUIDE       pro-nav, N = 5  (a = N · Vc · λ̇); pursuit kept as A/B baseline
ACT         velocity/attitude setpoint → PX4 OFFBOARD (MAVSDK)
   ↓
KILL        sim: proximity Pk@2.5 m (ADR-0025, ground truth scoring-only);
            real: binary kill on video + both aircraft logs
```

The sim harness (`scripts/m4_intercept.py --coded-dash`) and the portable
[`flight/`](flight/) package implement this; `flight/` has no gz/ground-truth/
cue imports (enforced by AST-based honesty tests) and drives the real
vehicle via `flight/deploy/seeker_loop.py` (SITL-validated,
`scripts/check_deploy_sitl.sh`).

The block/process diagrams in `docs/images/` (`architecture_block_diagram.png`,
`process_flow_diagram.png`) show the **retired sim-phase two-sensor
architecture** (ground stereo cue → jammable link → handoff) — kept as
history of the phase that produced the M0–M5 results; the ground-sensing
half remains a parent-project concept, not part of the current interceptor.

**Honesty boundary (unchanged since M0):** `gt_*` ground-truth topics are
scoring/logging only; guidance sees camera pixels + own-state EKF, nothing
else. Every gate re-derives the numeric no-cheat check, and
`tests/test_honesty_static.py` / `tests/test_honesty_seekers.py` pin it
statically (AST scans over every live seeker module, mutation-calibrated).

---

## The real build (Tier-1, in progress)

**Status 2026-07-25** — **Tier-1 is fully ordered** (target-drone stack, seeker
kit, interceptor flight controller, and the build/field consumables top-up).
The 2026-07-25 what's-left build push
([`docs/audit_2026-07-25_whats_left.md`](docs/audit_2026-07-25_whats_left.md))
closed the desk backlog before the parts land: the **ram/kill radius is
ratified at 0.35 m** (ADR-0084 — the ordered 5-inch pair's contact envelope, so
every Pk figure is now quoted against a radius the hardware can actually
deliver; the 5-inch carries the full seeker at T/W ~6:1); the field-day P0s are
closed (`--quad-decimate` plumbed, tripod mission geometry corrected, capture
matrix re-briefed for the beam-facing placard, range-truth integrity hardened);
the print artifacts (placard, checkerboard, mount) are built and dimensionally
verified; `FAILSAFE 7` (post-GO offboard loss → SAFE) and the kill-day protocol
are written; a sim-terminal zero-command bug was fixed (the coded-dash camera-arm
verdicts are re-flying); and **CI is green** (it had been red since the first
push). The Tier-1 software layer, all adversarially reviewed:

- `scripts/seeker/pi_capture.py` — Pi 5 / OV9281 session recorder (frames +
  index + metadata; logs the actual applied exposure vs the ≤1 ms spec).
- `scripts/seeker/tripod_score.py` — the **two-curve money-gate scorer**:
  curve (a) AprilTag decode envelope → gates the ~$740 Tier-2 interceptor
  order (explicit t_go ≥ 0.5 s arithmetic incl. the 5-detection streak
  burn); curve (b) NN recall vs range × position-in-frame → gates only the
  Hailo/markerless phase.
- `scripts/field_score.py` — binary-kill metrology from both aircraft logs
  (PX4 ULog + ArduPilot DataFlash .BIN), with the GPS relative-accuracy
  caveat stated (inter-receiver bias rivals the kill radius; video is the
  primary kill evidence).
- `flight/deploy/seeker_loop.py` MAVSDK OFFBOARD path SITL-validated
  (three real defects found and fixed); camera-offset/model-swap guard
  (a mismatch aborts the run rather than flying uncompensated).
- Camera paper-check (`docs/camera_paper_check.md`): the OV9281 clears the
  wide-FoV and exposure requirements on paper (118° horizontal, ~40 µs
  exposure floor); the mono-vs-color training confound stays open until
  real data.

The in-person ladder (P0 desk checks → tripod afternoon → Tier-2 order →
bench gates → kill ladder), with its go/no-go money gates, lives in
`docs/project_state.json` (`build_plan`) and `docs/hardware_order_list.md`.
A gate failure loops back to the cheapest upstream fix, never forward to
more spend.

---

## Methodology (the rules that caught the mirages)

- **Lab ranks, Gazebo decides.** The point-mass guidance lab screens ideas;
  only a Gazebo gate turns a ranking into a conclusion. The lab was wrong in
  the optimistic direction five documented times (PIP, Kalata gains,
  emit-vs-differentiate, fusion coverage, absolute miss).
- **Statistics before verdicts.** Run-to-run noise is ~1 m; single-flight
  deltas below that are noise. A/B claims need paired seeds (n≥8) and
  mechanism evidence, with honest "not significant at this n" language.
- **Anti-mirage rule** (earned, not decorative): any new "win" needs a
  control arm (dash-only / ground-truth-consistent audit), paired seeds, and
  validation on held-out *flights* — never frame-eval, never single-seed.
  This is the rule that caught all five coded-dash mirages.
- **Simulate worse than ideal.** Design numbers carry BEST / EXPECTED /
  WORST-credible tiers; decisions must survive WORST.
- **Sim time, never wall time**, and batches only on an idle machine
  (measured RTF-under-load confound, ADR-0009/0015).
- **The graveyard is binding.** Dead ideas (foveated crop, v3/rebal
  retrains, range-plausibility gates, APN/Kalman-CA-PIP, narrow lens, sim
  pivot, onboard acoustics…) are recorded with the evidence that killed
  them, and are not resurrected without new evidence
  (`docs/project_state.json`).

Known sim-to-real gaps are cataloged, not hidden: no motion blur, no
vibration, no outdoor clutter or lighting in the sim
(`docs/sim_to_real_gaps.md`); the tripod session exists precisely to measure
what the sim cannot.

---

## Reproduce it

Everything runs headless and writes CSV telemetry to `logs/`. The key
evidence CSVs behind every number quoted above are **committed** (M3/M4
gates, `mc_final_*`, `mc_t21_*`, `mc_pk72_*`, jam-MC and up-tilt arms); bulk
per-run logs are gitignored and regenerable. All Python runs through the
project venv (`.venv/bin/python`).

**Requirements:** Ubuntu 24.04, PX4-Autopilot (SITL) + Gazebo Harmonic,
Python 3.12, `pip install -r requirements.txt` (plus the apt-installed
`python3-gz-transport13`/`python3-gz-msgs10` bindings — see
`requirements.txt` comments).

**Tests (sim-free, CI-runnable):**

```bash
scripts/run_tests.sh        # offline suite (.venv) + ONNX parity (.venv-seeker)
                            # + the project-state dashboard drift check
```

**Milestone gates** (each a scripted pass/fail; exit 0 = pass):

```bash
scripts/check_m0.sh    # PX4+Gazebo boot headless; MAVSDK arms/takes off/lands
scripts/check_m1.sh    # camera frames via gz-transport
scripts/check_m2.sh    # AprilTag detected live (also repairs the world symlink, ADR-0005)
scripts/check_m3.sh    # static intercept, < 0.5 m standoff error
scripts/check_m4.sh    # pursuit vs pro-nav on a mover, pro-nav < 1.0 m
scripts/check_s1.sh    # FPV-speed crossing
scripts/check_s2.sh    # two-stage handoff + camera-only terminal
scripts/check_t21.sh   # re-asserts the ADR-0058 headline from committed CSVs (no sim needed)
scripts/check_deploy_sitl.sh  # deployment seeker loop end-to-end vs SITL
```

**Monte-Carlo batches** (the evidence machine):

```bash
scripts/mc_batch.sh --dry-run
scripts/mc_batch.sh --n 20 --laws pronav,pip --speeds 6.0,9.0 --directions both --master-seed 42
.venv/bin/python scripts/mc_analyze.py     # newest logs/mc_batch_*.csv
```

One sim at a time, idle machine only — batch numbers are only comparable at
matched load. `bootstrap.sh` recreates the config files on a fresh VM; a
normal clone doesn't need it.

---

## Repo map

- `flight/` — the portable real-build guidance core (geometry, estimator,
  pro-nav, deploy loop) + its tests. No sim imports; runs on the Pi.
- `scripts/` — sim harnesses (`m4_intercept.py` and friends), milestone
  gates, the Monte-Carlo batch runner/analyzer, the guidance lab,
  `ground_station/` (the retired-but-kept stereo spine), `seeker/` (the
  markerless detector lane: capture, training, scoring, tripod harness).
- `worlds/`, `models/` — the Gazebo worlds and targets (`apriltag_target`,
  the 3D `fpv_quad_enemy` quad, the stereo rig).
- `docs/` — `project_state.json` (**the contract**) + `dashboard.html`,
  `decisions.md` (the full ADR log), the audit findings tracker, design
  docs, `hardware_order_list.md`, `real_build_coded_dash.md`,
  `publish_runbook.md`.
- `logs/` — committed evidence CSVs (see above); everything else gitignored.
- `tests/` — the offline suite, including the AST-based honesty pins.
- `PROGRESS.md` / `NEXT.md` / `GOALS.md` — roll-up, working front, mission.

## License

Code and docs: **MIT** (`LICENSE`). The fine-tuned detector **weights are
not in this repo** (gitignored) and carry a separate licensing nuance
(AGPL-3.0 upstream tooling) — see
[`docs/license_notice_weights.md`](docs/license_notice_weights.md) before
redistributing any weights file.
