# Sweep — unresolved software tasks actionable without hardware or the sim

*Read-only discovery pass, 2026-08-10, against HEAD `9bf71b2`. Every item below was
verified against the live repo, not trusted from a tracker doc. Nothing was edited.*

**Out of scope by instruction (another agent is mid-edit):** `scripts/m4_intercept.py`,
`docs/scoring_fix_plan.md` — which is the contract's `scorer-measures-the-criterion`
assumption (`state: violated`, `test.status: not_run`). Not duplicated here.

---

## Tier 1 — correctness, do these first

### 1. The offline test suite is RED on `main`, and has been since ADR-0093
`flight/tests/test_seeker_loop_coast.py:373`

```
assert sp2 is not None and tel2.own_age_s == 1.50
E       assert (None is not None)
```

`test_a_stale_own_state_is_refused_once_a_threshold_is_configured` asserts that the
DEFAULT `GuidanceConfig` accepts a 1.50 s-old own-state sample. ADR-0093 (commit
`4df285f`, 2026-07-29) set `own_state_max_age_s = 0.25` — gate ON —
(`flight/deploy/seeker_loop.py:188`), so the default now refuses it. The test and its
docstring ("The AGE gate is off by default on purpose") were never swept.

Reproduced locally: `1 failed, 691 passed, 4 skipped`. Every CI run since has failed
(`gh run list`: 31456275749, 31456180473, 31455946363, 31452118965, … 8 consecutive).
ADR-0093 claims "Full suite 519 passed / 3 skipped" — that is `tests/` alone;
`run_tests.sh` stage 1 also runs `flight/tests/`, which was never executed.

- **Actionable without hardware/sim:** yes.
- **Size:** small (<30 min).
- **Value:** correctness of the project's *definition of done*. `run_tests.sh` green is
  the gate every ADR cites; it is currently false, and the drift check stays green
  because it is JSON-only.

### 2. Two comments in the flight seeker assert the opposite of the code
`flight/deploy/seeker_loop.py:1209-1211` and `:1619-1621`

> "This is NOT `GuidanceConfig.own_state_max_age_s` — that constant is **deliberately
> unset** because brn-05 exists to MEASURE the own-state latency that sizes it"

> "(This is also the run that MEASURES the own_age distribution
> `GuidanceConfig.own_state_max_age_s` **is waiting on**…)"

brn-05 ran 2026-07-29 and the constant was set the same day. Same class as the
`crossing_sign` docstring that sent a 2026-07-25 audit reader to a wrong finding.

- **Actionable:** yes. **Size:** small (same turn as #1). **Value:** anti-drift.

### 3. A producer→consumer contract test on the $740 gate's divisor skips forever
`tests/test_decode_fps_bound_contract.py:101-106`

```python
for cand in (os.path.join(os.path.dirname(HERE), "sessions", "smoke", "meta.json"),):
    if not os.path.exists(cand):
        pytest.skip("the skr-05 smoke session is not on this machine")
```

`<repo>/sessions/smoke/meta.json` does not exist. The session was rescued into the repo
on 2026-08-10 (commit `29ec8bc`) at **`runs/skr05_smoke_session/meta.json`**, and it is
exactly the artefact this test wants: `decode_loop_fps: 89.532`, `stream_fps: 30.14`,
`git_rev: 15b88b0` (one commit before the clamp), no bound mark in
`decode_loop_fps_source`. The regression that stops an unbounded 89.5 fps feeding the
purchase gate has therefore never run against the real artefact.

The whole point of the 08-10 rescue was that these numbers survive the Pi's SD card.
The test that consumes them was not repointed.

- **Actionable:** yes. **Size:** small (<30 min). **Value:** high — silent-failure class,
  on the number that decides ~$740.

### 4. The pinned-frame-rate drift guard also skips forever
`flight/tests/test_camera_fps_pinned.py:43-47`

```python
ap = seeker_loop.build_argparser() if hasattr(seeker_loop, "build_argparser") else None
if ap is None:
    pytest.skip("no separable argparser to introspect")
```

`seeker_loop` has no `build_argparser`; the parser is built inline in `main()`
(`flight/deploy/seeker_loop.py:1698`, `--camera-fps` at `:1737`). So
`test_cli_default_tracks_the_constant` — the check that `--camera-fps` cannot drift away
from `CAMERA_FPS_DEFAULT = 60.0` — is a permanent skip. `docs/next.md:37` cites this file as
"Regression-tested … 4 passed"; one of the four never ran.

- **Actionable:** yes. **Size:** small-medium (extract `build_argparser()` from `main()`).
- **Value:** high — green≠ran on the constant ADR-0090/0093 rest on.

### 5. The AprilTag first-kill seeker does not exist in the flight executor
`flight/deploy/seeker_loop.py:1683` (`build_detector`)

`build_detector` can construct **only** `FinetunedNNSeeker`. There is no tag path and no
`--detector` selector — `--weights` is the only choice, and it selects between two ONNX
models.

Against that, the contract states in four places that the first kills fly the AprilTag on
the Pi 5 CPU because the markerless NN needs the deferred Hailo HAT:
`docs/project_state.json:585` (build_plan summary), `:1073` (hardware stage note),
`:1467` (Tier-1 purpose), `:2592` (flight-compute decision option). ADR-0090 then
*measured* CPU-YOLO at **6.09 fps** and AprilTag at **96.6 / 38.2 fps**. So the only
detector the deployed loop can run is the one now proven too slow on the hardware in
hand, and the one proven fast is unreachable from flight code.

Raised as `[open]` by the 2026-07-25 audit (`docs/audit_2026-07-25_whats_left.md:24`)
and still open 16 days later.

The interface already exists and is stable: `SeekerDetection`
(`scripts/seeker/classical_seeker.py:58`) with `detect(frame_bgr, t_mono)`
(`scripts/seeker/finetuned_seeker.py:286`). `quad_decimate` plumbing exists at three
sites to copy (`scripts/seeker/pi_capture.py:252`, `tripod_score.py:552`,
`autolabel_from_apriltag.py:106`). Test fixtures exist:
`scripts/seeker/synth_tag_frames.py`.

- **Actionable without hardware/sim:** yes, entirely.
- **Size:** medium-large (a detector class + `--detector` flag + unit tests on synthetic
  tag frames). The largest item here, and the only one on the critical path.
- **Value:** highest strategic — this is the gap between the written plan and the code.

### 6. The money gate's range requirement is still reasoned at the superseded 30 fps
`scripts/seeker/tripod_score.py:76-80`, `:150-156`, `:1060-1063`, `:1165`, `:2770`

Every worked example still reads *"30 fps → t_go 0.558 s PASS | 20 Hz → 0.442 s FAIL,
break-even 24.0 fps"* at the predicted 7.10 m `R_decode90`. The flying camera is now
pinned at 60 fps (`flight/deploy/seeker_loop.py:725`, ADR-0090 addendum + ADR-0093
ratified on hardware), which roughly halves `R_streak_burn = (E[T]/fps) × V_closing` and
therefore lowers the `R_decode90` the tripod day must deliver.

**The scorer's CODE is fine** — `resolve_stream_fps` fails closed and never invents an
fps (`tripod_score.py:1161-1168`). What is stale is the *documented derivation* the
builder and the protocol read to pick station distances
(`docs/tripod_test_protocol.md:232`, `:1026`).

This is literally the contract's own open item: `docs/project_state.json` →
`now.next[1]` = *"Feed 96.6 fps into the gate scorer and re-quote the range needed."*

- **Actionable:** yes — pure arithmetic, `scripts/seeker/streak_burn_derivation.py --fps 60`.
- **Size:** medium. **Value:** high — it changes what the single field day has to achieve.

### 7. `streak_burn_derivation.py --fps` is half-wired; the output mislabels itself
`scripts/seeker/streak_burn_derivation.py:38` and `:52`

Run `--fps 60` and the §A.4 table body correctly uses 60 (burn 1.04 m at p=0.9), but the
header still prints a hardcoded `"(k=5, V=9 m/s, 30 fps)"`, and the M-of-N block
hardcodes `/30` (`closure at 30 fps …`). A 60-fps table under a 30-fps title is exactly
how a superseded constant walks back into a doc.

- **Actionable:** yes. **Size:** small. **Value:** medium — this is the instrument feeding #6.

### 8. Superseded qd=1.0 rate still live in that same derivation
`scripts/seeker/streak_burn_derivation.py:23` — *"and 40.3 fps at qd=1.0"*.

ADR-0090 addendum 1 (`docs/decisions.md:2260`) corrected this to **38.2** and states it
"supersedes 40.3 everywhere". Every other surface carries 38.2
(`docs/tripod_test_protocol.md:961`, `flight/deploy/seeker_loop.py:716`,
`docs/project_state.json:327/1156/1824`). This one line is the last 40.3 in the tree.

- **Actionable:** yes. **Size:** trivial. **Value:** small, but it rides along with #7.

---

## Tier 2 — always-visible surfaces the contract says must not drift

### 9. Contract §0 `now` is 15 days stale and both its "next" items are resolved or restated
`docs/project_state.json` → `now.as_of = "2026-07-26"` while `updated = "2026-08-10"`.

`now.next[0]` = "Ratify the 60 fps camera cap on the props-off bench (brn-05)" — **done**
(commits `31590a7`, `4c285c5`, ADR-0093). `now.next[1]` is item #6 above, still open.

This block is the first thing the builder reads on his phone.

- **Size:** small (rewrite + `render_dashboard.py` + republish the Artifact).
- **Value:** medium-high — the compass is pointing at completed work.

### 10. `docs/progress.md` stops at 2026-07-25
The milestone roll-up has no row for the entire bench arc: skr-05/06/07, brn-01…brn-05,
the 30-fps discovery and the 60 fps pin, the M10-lands-on-GPS2 finding (ADR-0091), the
motor-screw finding (ADR-0092), the Pi evidence rescue, ADRs 0090-0094.

- **Size:** small-medium. **Value:** medium (fresh-session orientation; CLAUDE.md makes it
  a subordinate view that must stay consistent with the contract).

### 11. `README.md:184-192` "Status 2026-07-25"
Never mentions that real hardware bring-up happened or that the seeker's frame rate was
measured. It is the surface a reviewer sees if the repo ever flips public.

- **Size:** small. **Value:** medium.

---

## Tier 3 — real, smaller, or needs a decision

### 12. Bare `pytest` at the repo root fails at collection
`scripts/seeker/nn_tier/onnx_smoke_test.py:21` imports `onnxruntime`, which `.venv` does
not have, and the filename matches pytest's default `*_test.py` discovery pattern:

```
ERROR scripts/seeker/nn_tier/onnx_smoke_test.py
!!!! Interrupted: 1 error during collection !!!!
696 tests collected, 1 error
```

`scripts/run_tests.sh:40` dodges it by naming `tests/ flight/tests/` explicitly. Anyone —
or any agent — running bare `pytest` sees a red wall instead of the suite.

- **Size:** small (a `norecursedirs`/`collect_ignore`, or rename the file).
- **Value:** small-medium; it is an agent trap.

### 13. Three built, tested, unwired flight modules — needs a ruling, not a build
`docs/phase_bcd_wiring.md:12-16` documents `--fov-hold`, `--range-fusion`,
`--terminal-coast` as "proposed" flags for `flight/fov_guidance.py`,
`flight/range_fusion.py`, `flight/terminal_coast.py`. None of the three flags exists
anywhere in code. The doc is honest ("PRE-CODED, OPT-IN, NOT SIM-VALIDATED, 2026-07-21").

**Not actionable tonight** — wiring touches `m4_intercept.py` (in use) and validation needs
the sim. Listed because they are 20 days old, the sim phase is closed, and the honest
choice is *wire-and-validate* or *retire to the attic*. That is a builder/PM call.

### 14. `scripts/forensics/q6_experiment.py:15-16` imports from an agent scratch dir
```python
sys.path.insert(0, "~/.claude/jobs/28aff4e9/tmp")
import guidance_lab_copy as gl
```
The directory happens to still exist, but the script is unreproducible for anyone else
and one cleanup away from dead. Either vendor the module into the repo or retire the
script.

- **Size:** small. **Value:** low-medium (reproducibility).

### 15. `docs/audit_findings_tracker.md` is a month-old ledger citing nine deleted docs
Dated 2026-07-10, calls itself "the sole canonical ledger", and its §4 backlog of 27
UNTRACKED findings has had no disposition update since. Its Evidence cells cite
`docs/WRITEUP.md` (×10), `docs/portfolio_bullets.md` (×9),
`docs/interviewer_prep.md` (×4) and the three 07-10 audit sources — all deleted under
ADR-0066. Either re-verify against HEAD or retire it to `docs/state_archive/`.

- **Size:** medium (re-verification) or small (retire). **Value:** medium — 27 named open
  findings that nobody can currently tell are live or dead is worse than none.

---

## Low value / consider DELETING

Deleting is a deliverable (CLAUDE.md §"What the BUILDER cannot see").

| What | Why it is dead weight |
|---|---|
| `docs/audit_2026-07-25_whats_left.md` (861 lines) | I spot-checked its RANK 1-8: `--quad-decimate` plumbed (`tests/test_quad_decimate_and_incidence.py`), FAILSAFE 7 built (`real_flight.py:313`, ADR-0086), `COAST_STALE_S` pinned (`tests/test_cue_staleness.py:79`), `solve_intercept_time` covered (`tests/test_solve_intercept_time.py`), print artifacts built (`hardware/prints/`), CI fixed (then re-broken by #1). It is a **closed** audit still readable as a live to-do list. Needs a CLOSED banner or `docs/state_archive/`. |
| `docs/fpv_fidelity_design.md` | A design study for `--fpv-fast` — a flag that exists nowhere in code, for a sim phase that is closed. 22 mentions of a thing that will never be built. |
| `docs/demo_plan.md` | Its `## TODO` list (`:100`) plans demo assets that were **retired 2026-08-10** (`demo_out/retired_2026-08-10/`, ADR-0094). Also cites `scripts/demo_run.sh` and `scripts/check_demo.sh`, neither of which exists. |
| `scripts/seeker/eval_classical.py` | Module-level `cv2.TrackerKCF_create` breaks under `.venv`; under `.venv-seeker` it has no argparse at all, so `--help` **runs the full analysis** and writes into `scripts/seeker/out/`. Classical/CSRT tracking is a closed dead idea (contract `detector` stage: "detect-then-track/CSRT dropped … do NOT resurrect"). Attic. |
| `scripts/adaptive_tilt_arm.sh` + `scripts/experiments/gimbal_mount/` | Adaptive tilt was **REJECTED** by the builder 2026-07-17 (contract `pointing` stage). Parked lever, sim-only. |
| `scripts/experiments/actuation_error_budget.py`, `scripts/video/make_stereo_shots.py`, `experiments/darkhorse/darkhorse_common.py`, `scripts/seeker/stop_skr07.sh` | Zero references from any tracked file in the repo. |
| Triplicated skr-07 evidence: `runs/skr07/`, `runs/skr07_tagged/`, `logs/skr07_2026-07-26/` | `docs/next.md:36` cites `runs/skr07_tagged/`; `docs/project_state.json:1824` cites `logs/skr07_2026-07-26/`. Two of the three should go, and the survivor should be cited consistently. |

---

## Categories that turned up NOTHING (stated plainly)

- **Syntax errors / broken imports:** none. All tracked `.py` parse; every non-stdlib
  import resolves to a real dependency or a repo-local module. Only oddity is #14.
- **`NotImplementedError` stubs:** all deliberate, documented and fail-safe — not tasks.
  `scripts/bird_mc_harness.py:211` (`classify()` stub whose absence is treated as a
  VETO-ALL by `interlock_decision()`), `scripts/ground_station/station.py:580`
  (`--spoof` is a declared T19a no-op that refuses to fake a mode),
  `scripts/frame_source.py:97-100` (abstract base).
- **`--help` smoke across every argparse entry point:** one failure,
  `scripts/seeker/drone_detector_eval.py` needs `onnxruntime` → runs fine under
  `.venv-seeker`. Not a defect.
- **TODO/FIXME/XXX/HACK markers:** ~60 hits, and after filtering docstring history they
  are almost entirely (a) ~20 `TODO-BUILDER` bench constants in
  `flight/deploy/real_flight.py`, hardware-gated by design and printed as a loud banner
  at `:2485`, and (b) 8 `TODO(sim)` verification tags in
  `scripts/seeker/seeker_v3_capture.py`, sim-gated. The only desk-shaped one,
  `scripts/field/common.sh:165` (capture real `lsusb` VID:PID), needs the boards plugged
  in. This matches the 2026-07-25 audit's own read
  (`docs/audit_2026-07-25_whats_left.md:626`) — the marker scan is not where this
  project's work is hiding.
- **Contract stages marked `idea`:** none. All ten are `implemented` or `half-done`; the
  seven `half-done` stages' next steps are hardware (`hardware`, `real_data`) or sim
  (`terminal`, `handoff`, `kill`, `pointing`, `detector`) — item #5 is the one
  pure-software next step among them.
- **Open contradictions in the ledger:** zero of 38.

---

## Suggested order for tonight

1, 2, 3, 4 as one commit (~1 h, turns the suite and CI green and makes two skipped
guards real) → 8, 7, 6 as one commit (the fps derivation sweep) → 9, 10, 11 as one
contract/docs sweep turn → then 5, which is the big one and deserves its own night.
