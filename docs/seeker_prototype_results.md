# Markerless seeker — prototype results & honest review (ADR-0033 Stage A + B)

*Synthesis and adversarial review of the two offline markerless-seeker prototypes
built against pre-captured onboard frames. Companion to `docs/seeker_design_brief.md`
(the design/interface/eval plan) and `docs/seeker_nn_findings.md` (NN lane's own
writeup). **No sim/PX4/Gazebo was booted for this review**; a Monte-Carlo batch was
running on the machine, so all live-sim validation is **deferred** (see §7).*

Reviewer note: I re-read both prototypes' source, re-checked the annotated evidence
frames myself, and re-ran the classical detector on the terminal frames its own eval
sample skipped. The headline classical result **did not survive** that check. Read §3.

---

## 1. What each lane built

| | Classical lane | NN lane |
|---|---|---|
| File | `scripts/seeker/classical_seeker.py` | `scripts/seeker/nn_seeker.py` |
| Eval | `scripts/seeker/eval_classical.py` → `out/eval_classical.csv` | `scripts/seeker/eval_nn.py` → `eval_out/*.png` |
| Method | Dark + low-saturation threshold → morphology → `connectedComponentsWithStats` → pick blob maximizing `contrast·log(area)`; KCF re-detect every 12 frames | YOLOv8n (COCO, ONNX) on CPU via onnxruntime; conf + horizon + self-mask + aspect gates; top-1 box |
| Weights / license | Pure OpenCV, no model (permissive) | `weights/yolov8n.onnx`, 12.8 MB, **AGPL-3.0** (R6) |
| Output | `SeekerDetection` → `to_measurement_fields()` | `SeekerDetection` → `as_measurement_tuple()` |
| Venv | `~/interceptor-sim/.venv-seeker` (isolated) | same isolated venv |

Both correctly implement the **interface contract** (brief §1): `bearing_rad`
(`atan2((u−cx)/fx, 1)`, +=right), coarse known-size range (`fx·W/w_px`), confidence in
the `decision_margin` slot, `None`-fill on no-detect. **The seam is right in both** —
that part is real and reusable. The problem is upstream, in *what pixels the detector
locks onto*.

**Isolation verified:** both lanes used `.venv-seeker`; the batch venv
`~/interceptor-sim/.venv` was not touched. `.venv-seeker` is **not** in
`.gitignore` — exclude it before any commit (flagged, not fixed — no existing file was
edited per the task constraints).

---

## 2. NN lane (Stage B) — honest, correct, low-recall

**Verdict: genuine target-body detection, honestly reported. Body-not-tag PASSES.**

I confirmed against `eval_out/det_frame_000543.png`: the red box encloses the **whole
airframe** — arms, red motors, prop discs, *and* the tag — with the box centroid
(yellow +) on the **body centre, offset from the tag plane**. Box is 2.6–2.8× the tag
size. This is airframe detection, not fiducial decoding. The tag-inpaint test
(reconstruct body texture, re-run) still fires 3/6 close frames. The honest caveats the
lane disclosed are real and correctly stated: confidence drops when the tag is removed
(its contrast helps), and a flat-gray fill kills detection (unnatural patch) — so a
*fully* tag-independent number still needs the tag-less Gazebo model.

Measured (their numbers, which I find consistent with the CSV/frames):
- **Detection rate:** genuine target only in a **~5-frame terminal window** (frames
  540/542/543, range **1.6–3.0 m**). 3/74 sample ≈ 4%; 3/50 terminal ≈ 6%.
- **Bearing:** +7.9 / −0.3 / −4.5° near boresight — physically sane; sign verified
  (+=right). vs AprilTag-centroid bearing: mean offset −7.7°, std **2.1°** (the offset is
  largely a *real* body-vs-tag-mount displacement; the 2.1° std sits in the brief's
  ~1–2° ML band, §5).
- **Range:** 1.6–3.0 m vs tag GT 1.9–3.2 m, within the expected 15–30% σ (range is
  bearing-subordinate by design).
- **Latency:** ~48–53 ms/frame (~19–22 fps) desktop CPU, 2 threads. Brief embedded
  targets: Pi-5 CPU ~13 fps, Pi5+Hailo-8 ~35 fps end-to-end (§2.2).

**The honest negative result:** an untuned COCO nano is **not a viable seeker** — no
drone class (reads "airplane" only when large, ≤3 m), and raw top-1 locked the
interceptor's own rotor arms + horizon on 72% of frames until three physically-motivated
gates removed them. What it *does* deliver: a proven inference→bearing→range seam where a
fine-tuned single-class drone nano drops in unchanged, plus a reusable
self-arm/horizon/aspect gating insight. This is exactly the brief's predicted **R2**
(acquisition regresses on a few-pixel body).

---

## 3. Classical lane (Stage A) — the headline did not survive review

**Verdict: the reported "80% detection, body-not-tag" is MISLEADING. As evaluated, the
detector is locking the interceptor's OWN prop-arm, not the target drone. Body-not-tag
is technically true but hollow — it is also NOT-TARGET for the entire sampled approach.**

### 3.1 What the annotated frames actually show
`out/annot_0000.png`, `annot_0424.png`, `annot_0565.png` all put the green box on the
**same large, smooth, dark tapering wedge pinned to the LEFT frame edge** (box `(0, 192,
488, 134)` etc., always touching x=0). That wedge:
- is **static in size and position across the entire 28-second cut** (identical box
  `(0,192,488,134)` for the first 9 sampled frames, 0→141) — an approaching target grows
  and moves; this does not;
- is a **featureless blade** with no visible arms, motors, props, or tag — unlike the
  real target;
- has a **mirror-image twin on the RIGHT edge** (visible in `annot_0565.png`, and it is
  what the CSV's positive-bearing boxes at x≥969 lock onto).

That is the **interceptor's own rotor arm / prop** intruding from the FOV periphery —
the *identical* self-occlusion false positive the NN lane found (conf 0.79 on frame 0's
own arm) and deliberately gated out. The classical lane did not gate it and reported it
as a valid target detection.

### 3.2 The eval sample never even tested the real target
The CSV samples frames 0, 18, 35, …, 530, **547**, 565, … — it **jumps straight over
540–545**, the only window where the real target is present (per the NN lane's proof).
So **all 40 of the classical CSV's "detections" are own-airframe edge wedges; ~zero are
genuine target detections.** The four body-not-tag proofs (tag-white 0.000, real-tags 0,
dark-blob structure, masked-tag survives) are all correct — but they only establish
"not the fiducial" on frames that are **not the target either**. Masking the tag region
of an own-prop box proves nothing about target detection.

### 3.3 The detector *can* find the target — but only terminally
I re-ran `ClassicalSeeker.detect()` on the skipped terminal frames (isolated venv,
single-thread, nice'd):

```
frame 540: box cx=1131  bearing=+43.5°  area=10115   <- own RIGHT prop
frame 541: box cx=1149  bearing=+44.4°  area=7553    <- own RIGHT prop
frame 542: box cx=717   bearing=+8.3°   area=3142    <- REAL target (NN: u=715, +7.9°)
frame 543: box cx=688   bearing=+5.4°   area=4845    <- REAL target
frame 544: box cx=596   bearing=-4.3°   area=10814   <- REAL target, looming
frame 545: box cx=498   bearing=-14.6°  area=21377   <- REAL target, looming
```

So the detector locks the true target **only in the last ~4 frames**, once the target
looms large enough to out-score the own-prop wedge on `contrast·log(area)`. That is the
same terminal-only acquisition as the NN — **not** an 80% detection rate.

### 3.4 Why this is worse than low recall (guidance hazard)
Throughout the approach the classical seeker emits a **high-confidence (conf=1.0) bearing
of −38 to −47° (or +43 to +48°)** pointing at the interceptor's own airframe. Wired into
`detection_loop`, that is not a harmless dropout — the α-β LOS filter (brief §1.3) would
**latch onto a confident phantom at the FOV edge and steer pro-nav toward it**. A missing
detection the filter *coasts* through (tolerated by design); a confident *wrong* bearing
it does not. **This detector must not feed guidance as a standalone bearing source until
the own-airframe lock is fixed.**

---

## 4. Head-to-head (what pro-nav actually needs)

| Metric (brief §5) | Classical (as built) | NN (as built) | Tag baseline |
|---|---|---|---|
| **Genuine target detection** | terminal ~4 frames only; CSV "80%" is own-airframe FPs | terminal ~3 frames (4–6%) | ~14 Hz, ~9–12 m acq |
| **Bearing (the thing that matters)** | correct *when* on target (matches NN at 542); **confident-wrong on own-prop otherwise** — a guidance hazard | σ≈2.1° vs tag, near-boresight, honest | sub-degree |
| **Range (coarse, gating only)** | known-size, biased (broadside aspect) | known-size, within 15–30% σ | 5–8% |
| **Acquisition range (#1 miss lever)** | terminal only (~1.5–3 m) | terminal only (~1.6–3 m) | 9–12 m |
| **Latency / embeddability** | few ms (untimed here); trivially Pi-real-time | ~50 ms desktop; Pi-5 ~13 fps, +Hailo ~35 fps | ~14 Hz w/ quad_decimate=2 |
| **License** | permissive (OpenCV) | **AGPL-3.0** (R6 — prefer NanoDet/MIT) | n/a |
| **Body-not-tag** | technically yes, but locks the WRONG body | **yes — airframe, verified** | (is the tag) |

**Does either retire the #1 disclosed risk (reliable onboard acquire-and-hold of a small
comms-denied drone, ADR-0015 seat-5)?** **No.** Both acquire only *terminally* (~1.5–3 m
in this footage) vs the tag's 9–12 m — a stark **R2 acquisition regression**, precisely
the brief's prediction that *the seeker, not the guidance, is the honest bottleneck*.
Since correction capacity ∝ t_go² (ADR-0023/0024), acquiring at ~3 m instead of ~9–12 m
is a large ZEM/miss penalty. What the prototypes *do* retire is **interface risk**: the
`Measurement` seam, the coarse-range/bearing-only framing, and the self-occlusion problem
are now concrete and demonstrated — converting the portfolio from "guidance demo with a
solved perception problem" into "seeker development with a measured, honest bottleneck"
(ADR-0033). Neither is yet a *working markerless intercept*.

**Caveat on all of the above:** this footage is a stitched establish + fly-by demo cut,
not a clean monotonic approach, and it **carries no ground-truth pose** (manifest has only
idx/sim_t/file). So absolute σ_β, σ_R, and a real `P_detect(R)`-vs-range curve **cannot**
be measured here — they require the tag-less Gazebo model flown with gt logging (§6). The
offline numbers *rank and diagnose*; only a Gazebo gate/batch concludes ("lab ranks,
Gazebo decides").

---

## 5. Body-not-tag verdict (the whole point of "kill the AprilTag")

- **NN lane — PASS.** Box covers the full airframe (arms, motors, tag), centroid on the
  body offset from the tag, inpaint survives; own-airframe correctly rejected. Genuine,
  honestly-terminal target-body detection.
- **Classical lane — FAIL (in the sense that matters).** It does not lock the fiducial —
  but for the entire sampled approach it locks the interceptor's **own prop-arm**, not the
  target. "Not-tag" here is hollow because it is also "not-target." It reaches the true
  target only in the ~4 terminal frames its own eval sample skipped. This is not a data
  fabrication — the CSV, frames, and cross-checks are all honestly computed — it is a
  **detector-selection failure hidden by an unlucky eval sample**.

---

## 6. Recommended NEXT step (gated integration, post-M5)

**Wire the NN seam — but fix the classical own-airframe lock first, and never ship
classical as a standalone bearing source.** Concretely, in priority order:

1. **Fix the classical self-lock before trusting Stage A.** Add the NN lane's
   self-occlusion / horizon / aspect gates (or a *static self-mask* derived from the known
   fixed airframe geometry — the own props are body-fixed at the periphery). Better still,
   demote the classical blob detector to the **motion/blob PROPOSAL layer** feeding a
   classifier (true detect-then-track, brief §6) — which is its actual role in the parent
   architecture — rather than a standalone bearing emitter. Rationale: a confident wrong
   bearing corrupts the α-β LOS filter (§3.4); a dropout does not.
2. **Integrate the NN seeker behind `m4_intercept.py`'s `detect()` seam** as
   `--seeker nn`, using `SeekerDetection.as_measurement_tuple()` as the drop-in — but with
   a **fine-tuned single-class drone nano (MIT/Apache, R6)**, not the COCO model (which
   only fires "airplane" at ≤3 m). The seam is proven; the model is the swap.
3. **Build `models/fpv_target_markerless`** (primitive body + props, tag plane removed;
   separate model so every gated world is untouched, brief §4) and run the **paired-seed
   A/B (n≥8) vs the AprilTag baseline** on the same mover paths (L→R, R→L, weave, jink,
   oblique_close) and speeds (6/9/12 m/s), **with gt logging** so `P_detect(R)` and σ_β
   become real numbers. **One sim at a time, at idle load** (batch hygiene).
4. **Re-earn the numeric no-cheat audit** for this new guidance path (`gt_*`
   scoring-only, `docs/audit_targets.md` pattern) before any A/B number is quoted.
5. **WORST-tier honesty:** inject measured σ_β, σ_R, `P_detect(R)`, and LOS-rate dropout;
   disclose that the no-blur/no-vibration sim makes sim seeker Pk an **upper bound**
   (ADR-0024).

---

## 7. Deferred / not done (explicitly)

- **All live-sim validation is deferred** — an MC batch was running; no sim/PX4/Gazebo
  booted, no `mc_batch.sh` / `check_*.sh` / flight run, no existing script or ADR/NEXT/
  PROGRESS edited.
- **Full 867-frame sweep is post-batch** — both lanes sampled ~50–74 frames to keep load
  modest; run `eval_classical.py` / `eval_nn.py --all` when the machine is idle.
- **No ground-truth pose in this footage** → absolute σ_β, σ_R, `P_detect(R)` unmeasured;
  require the tag-less Gazebo flight (§6).
- **`.venv-seeker` is untracked and not git-ignored** — exclude before commit.
- **Pi-5 + Hailo-8 bench** (ADR-0033 item 1) deferred.

---

### Bottom line

The NN lane is the **honest, correct base to build on** despite low recall: it genuinely
detects the target *body*, correctly rejects the interceptor's own airframe, and reports
its terminal-only acquisition truthfully. The classical lane's *seam and cross-check
tooling are good*, but as evaluated it **detects the wrong body** (own prop-arm) and its
80% headline is an artifact of a sample that skipped the real target — it needs the same
self-masking the NN lane already built before it can feed guidance. **Neither retires the
#1 acquisition-range risk**; both prove the seam and correctly surface that the *seeker*
is now the bottleneck — which is itself the portfolio-flipping result ADR-0033 sought.
