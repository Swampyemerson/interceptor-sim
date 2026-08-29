# Dark-horse levers — pre-coded experiments & testing guide

> **What this is.** Four "research dark-horse" levers from
> `docs/intercept_accuracy_levers.md` ("Research dark-horses" section),
> pre-coded so Emerson can TEST them, not just read about them. Every script has
> a **synthetic self-test that runs NOW** (proves the algorithm + the pass/fail
> plumbing) and, where real data exists, a **real-data path** that runs on the
> repo's logged frames / datasets. No Gazebo/PX4 sim is launched by anything
> here (the sim serializes; these are all offline). Built in an isolated
> worktree; touches only `experiments/darkhorse/`.
>
> **Honesty boundary (holds for all four).** Camera + own-state inputs only;
> `gt_*` (filename-encoded range/bank, gt boxes) is used for SCORING /
> degradation-modelling ONLY, never as an estimator input. **The sim has no
> motion-blur model**, so every acquisition-range number here is an **UPPER
> BOUND** — the Stage-0 bench decides. That is *why* three of these scripts
> inject a bench blur+noise model: the lever only lives in the degraded regime
> the sim omits.

## Environment

Scripts invoke Python as `../../.venv/bin/python` (two levels up from
`experiments/darkhorse`). **In the main checkout that resolves to the real
`~/interceptor-sim/.venv` — nothing to set up.** Only an isolated
git *worktree* needs a convenience symlink at the worktree root:
`ln -s ~/interceptor-sim/.venv <worktree-root>/.venv`.

- **`--self-test` on all scripts, and the whole of `bank_feasibility.py`**, need
  only **numpy + cv2** → run with the project **`.venv`** (`../../.venv/bin/python`).
- **`track_before_detect.py --sweep`** additionally needs **onnxruntime + the
  seeker weights** → run with **`.venv-seeker`**
  (`~/interceptor-sim/.venv-seeker/bin/python`). CPU inference over a
  few hundred logged frames; no GPU, no sim. (The `.venv-seeker-train-gpu` GPU
  env is only for training, not needed here.)
- Data (`scripts/seeker/data/*`, weights) is **gitignored** — it lives only in
  the **main checkout**. Scripts resolve it via `DARKHORSE_REPO`
  (default `~/interceptor-sim`) or `--repo`. Run from the worktree,
  data reads from main.

## Status at a glance

| # | Dark horse | Script | Runs now (self-test) | Real-data path | Verdict from the pre-code |
|---|---|---|---|---|---|
| 1 | Track-before-detect | `track_before_detect.py` | PASS (√N proven) | **ready** (set-pose sweep + v2 ONNX) | **Promising, markerless-only.** Buys ~8→16 m acquisition in the *noise-limited* regime |
| 2 | IMU motion-deblur | `imu_deblur_bench.py` | PASS (restore proven) | bench (no sim blur) | **Conditional.** Helps blur-limited/low-noise; useless above the noise wall |
| 3 | Bank-as-accel | `bank_feasibility.py` | PASS (collapse shown) | **ready** (`quad_dataset_banked`) | **Likely DEAD.** Under terminal blur σ_a 6–12 m/s² > 4.7 m/s² jink signal |
| 4 | Event camera | `event_camera_plan.md` | n/a (plan) | bench/HIL only | **Gated escalation.** Bench-only plan; pointing + software levers first |

---

## 1. Track-before-detect — `track_before_detect.py`

**Idea.** Integrate the detector's SUB-threshold response across N frames along
the motion hypothesis; effective SNR rises ~√N, pulling the target out of noise
at a range where one frame is blind. A *temporal* acquisition lever the
resolution-rejection experiment (spatial, per-frame) never tested.

**Run it now (self-test, project .venv):**
```bash
../../.venv/bin/python track_before_detect.py --self-test
```
Synthetic faint moving blob, per-frame sub-threshold. Verifies aligned SNR grows
~N^0.5 (measured slope ~0.58), clears a 5-σ bar at N=48, and beats a motion-blind
naive stack ~15×. **PASS.**

**Run it on real logged frames (seeker .venv):**
```bash
~/interceptor-sim/.venv-seeker/bin/python track_before_detect.py \
    --sweep --n-integrate 1,4,8,16 --blur-px 0 --noise-dn 22 --limit-per-range 24
```
Re-runs the deployed `drone_finetuned_quad_v2.onnx` heatmap over the set-pose
recall sweep (551 in-frame frames at 4.9/8/12/16/20/25 m — target *guaranteed* in
level FoV, isolating the detector from the pointing confound). **Key measured
result:** on CLEAN sim frames v2 saturates (detects single-frame to 25 m → no
sub-threshold regime, TBD can't help — an honest artifact of the no-blur/no-noise
sim). Add the bench **noise** model and the lever appears: at `--noise-dn 22`,
single-frame detection collapses at ≥8 m, and **frame-stacking recovers 8 & 12 m
at N=4 and 16 m at N=16**; 20–25 m stay dark (resolution floor, not noise). TBD
integrates at the **raw-frame** level (register + average → denoise, then
detect): integrating the detector's *output confidence* does **not** work — a
trained net collapses sub-threshold signal internally.

- **Data it needs:** per-frame NN response over logged frames. Uses the existing
  `setpose_sweep` + `drone_finetuned_quad_v2.onnx`. The **decisive real test**
  wants a *continuous framed far→near approach* (a pointing-fixed flight or a
  scripted teleport ramp) so the motion-hypothesis registration is exercised — the
  set-pose frames are stationary per range (zero-shift). `--frames <dir>` is
  scaffolded for that sequence when it exists.
- **Only helps the markerless NN seeker**, not the AprilTag baseline (a binary
  decoder emits no sub-threshold response to integrate).
- **PASS/FAIL bar:** on a framed approach sequence, TBD lifts the range at which
  detection first becomes reliable by **≥ 1.3×** vs single-frame at matched
  false-alarm rate, in the noise-limited regime — and the lift scales ~√N.
- **Status: READY-TO-TEST** (real-data path runs today; decisive version needs a
  framed-approach capture).

## 2. IMU-guided motion-deblur — `imu_deblur_bench.py`

**Idea.** Build the terminal blur PSF from IMU/commanded rates + exposure and
deconvolve the track ROI — no new hardware. **Honest wall:** deconvolution
inverts a *known* blur; it cannot recover signal below noise.

**Run it now (project .venv):**
```bash
../../.venv/bin/python imu_deblur_bench.py --self-test     # blur a chip, recover it
../../.venv/bin/python imu_deblur_bench.py --noise-sweep   # find the SNR breakpoint
../../.venv/bin/python imu_deblur_bench.py --psf-from-rates 900 --exposure-ms 5
```
Self-test blurs a known chip (13.7 px smear), Wiener-deconvolves, and confirms
the restored target-shape match to truth rises (NCC 0.84→0.89) with correct
localization — **PASS**. Noise-sweep shows the honest breakpoint: matched-Wiener
holds the target to **~16 DN** noise; a **naive** (under-regularized) inverse
fails by **~1 DN** (the "deconv amplifies noise" catastrophe); above the wall
neither invents SNR — that's the event camera's / short-exposure's job.

- **Data it needs:** none to run the bench self-test. The real version needs
  **bench captures**: a target chip imaged sharp on the OV9281, then panned at a
  known rate with a known exposure (the sim has no blur to deconvolve). PSF built
  from own-state rates (`build_psf_from_rates`) — honesty-clean.
- **PASS/FAIL bar:** on real bench chips, deblur raises detector objectness (or
  bearing σ) at the measured terminal noise level; if the sensor's real noise DN
  at the required short exposure sits **above** the sweep's breakpoint, deblur is
  a no-op and the event camera is the only fix.
- **Status: READY-TO-TEST as a Stage-0 bench tool** (runs now on synthetic;
  needs bench captures for the real verdict).

## 3. Bank-as-accel feasibility gate — `bank_feasibility.py`

**Idea.** Estimate target lateral accel from its bank (a_lat = g·tanφ) as a
*leading* APN feedforward, no differentiation. **Gated HARD** because (a) on the
straight-line mission target the APN term is identically zero (jink-insurance
only) and (b) reading bank off an ~8–15 px blurred silhouette is likely
impossible.

**Run it now (project .venv):**
```bash
../../.venv/bin/python bank_feasibility.py --self-test
```
Synthetic bank-bars: bank-discrimination SNR is huge clean and **collapses ~95×
under terminal blur** — the mechanism the real gate confirms. **PASS.**

**Run the real gate on the banked dataset (project .venv, no ONNX needed):**
```bash
# clean (optimistic, sim-like): bank looks readable
../../.venv/bin/python bank_feasibility.py --gate --noise-dn 0.05
# terminal-realistic (blur the 8-15 px target): the decisive run
../../.venv/bin/python bank_feasibility.py --gate --blur-px 23 --noise-dn 0.30
```
Uses `quad_dataset_banked` (1344 chips, banks −50/−30/+30/+50, ranges 2–8 m),
grouped into 336 nuisance cells (same range/lateral/height/yaw/pitch, vary only
bank). **Measured verdict:** clean, bank is readable (σ_a < 0.9 m/s² to 8 px);
but under the **terminal 23 px smear** — which spans the whole 8–15 px target —
bank-SNR collapses to ~1–2 and **σ_a = 6–12 m/s², LARGER than the 4.7 m/s² jink
signal** it would feed forward. **DEAD at every terminal size once terminal blur
is present**, confirming the "optically implausible" prediction with a number.

- **Data it needs:** `quad_dataset_banked` (present). For a photoreal terminal
  read, real bench chips at 8–15 px would sharpen the number, but the sim-data
  gate already returns the verdict.
- **PASS/FAIL bar:** σ_a (1-σ target-accel error) must be **< ½·a_jink ≈
  2.35 m/s²** at terminal pixel size for the cue to help. Measured σ_a ≥ 6 m/s²
  under terminal blur → **FAILS the gate → idea DEAD** (as flagged). Only revisit
  if a future sensor keeps the target well above ~15 px through the terminal.
- **Status: READY-TO-TEST, and the pre-code already returns DEAD** under the
  honest terminal-blur condition.

## 4. Event / neuromorphic camera — `event_camera_plan.md`

**Idea.** A Prophesee GenX320 event sensor (no exposure window → no motion smear)
as a **terminal-only** blur-killer alongside the OV9281. **Not sim-scorable**
(Gazebo has no event model) → a **bench/HIL plan**, not code.

- **The plan specifies:** M-EV-1 usable-bearing depth vs angular rate
  (sweep 50→1870 °/s, event vs short-exposure OV9281); M-EV-2 low-light floor;
  M-EV-3 ego-motion event-segmentation recall under the real pitch/roll profile
  (the research-risk probe); M-EV-4 Pi-5 event-processing budget. Plus
  cost/weight and the honesty caveats (new sensor re-earns the no-cheat audit;
  ego-motion segmentation from a pitching quad is itself unsolved).
- **PASS/FAIL bar (to earn a prototype):** event usable-bearing depth **≥ 2×**
  the best OV9281 exposure at ≥485 °/s, **and** de-rotated segmentation recall
  **≥ 80 %** under ego-motion, **and** Pi-5 sustains the terminal event rate. Any
  one failing → dead or deferred.
- **Status: NEEDS-DATA / BENCH-ONLY.** Read the decision flow in the plan; it is
  an escalation gated behind the pointing fix and the two software levers above.

---

## Recommended order for the builder

1. **Track-before-detect (#1)** — highest upside, real-data path runs today; the
   one worth a *framed-approach capture* to make decisive.
2. **Bank-as-accel (#3)** — already returns **DEAD** under terminal blur; one
   `--gate` run to confirm on your machine, then log the ADR and close it.
3. **IMU-deblur (#2)** — keep the bench tool; run for real only when bench chips
   exist, to decide whether #4 is even needed.
4. **Event camera (#4)** — last, and only if pointing is fixed and #1/#2 leave a
   residual terminal-blur wall.

Everything traces to `docs/intercept_accuracy_levers.md`,
`docs/seeker_acquisition_range_note.md` (§3.5), and the ADRs cited in each script
header. Numbers above are from the self-tests / real-data gates run at build time
(machine idle, no sim); rerun to reproduce.
