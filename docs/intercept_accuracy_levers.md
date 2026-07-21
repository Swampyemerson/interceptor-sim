# Improving the camera-only ram intercept — research synthesis & ranked levers

> **Status: RESEARCH SYNTHESIS (2026-07-21).** Builder ask: "do heavy research and get
> creative with improving that intercept distance." Builder rulings the same day: RAM
> kill CONFIRMED (net parked as a fallback only); launch mechanism PARKED until real
> flight. Produced by a fan-out research workflow — 4 web-SOTA lenses (terminal-guidance
> law, small-target detection, alt-sensing/endgame, real counter-UAS + kinematics) + a
> repo-grounding brief → 2 creative ideators (34 ideas) → adversarial per-category
> vetting vs the honesty boundary + graveyard + physics (27 survived) → synthesis. 59
> external sources. This note RANKS levers; only a gated Gazebo/bench run turns a ranking
> into a decision ("lab ranks, Gazebo decides, the bench decides hardware numbers").
> Contract home: decision `intercept_accuracy` on the `pointing`/`terminal` stages.

## What "intercept distance" means here, and the three facts that reframe it

**Intercept distance = terminal closest-approach (CPA) of the interceptor body to the
target body.** For a RAM (hit-to-kill, no payload) the bar is a **~0.30–0.50 m contact**
— two ~0.3 m bodies must physically overlap. That is *much* tighter than the Pk@2.5 m
proximity proxy the earlier Monte-Carlo work reported: a 2.3 m "Pk kill" is a fly-through,
not a contact. Three measured facts decide everything downstream:

1. **The binding wall is POINTING, not the sensor.** The quad pitches 35–40° nose-*down*
   to dash forward, tipping the fixed camera down so a co-altitude target sits at the
   frame **top edge or out of frame**. The detector is ~100% reliable **static** at
   8–22 m but **0.8% in flight** (2 real vs 1187 phantom detections across 63 flights);
   ~75% of ticks in the 8–12 m band have the target **out of frame**. The target isn't
   in the picture — it's not a resolution or weights problem. The committed fixed up-tilt
   wedge is necessary but **not sufficient**: the repo is explicit that a fixed tilt
   *"does not close the dash-pitch gap, it only relocates the in-view window."*
2. **Miss is ~96% set by the zero-effort-miss (ZEM) at handoff** (r²=0.99). The terminal
   FOV-hold / sub-1 m blind window is only ~2–4% of the miss.
3. **Correction capacity = ½·a·t_go², which grows with time-to-go SQUARED.** Measured:
   pushing handoff from ~6.5 m to ~12 m lifted capacity 0.72 m → ~4.3 m. So earlier
   acquisition (more t_go) and more lateral-accel authority both buy miss reduction
   super-linearly.

**The honest, load-bearing caveat (ADR-0027):** at 9 m/s the *delivered* ZEM is ~4 m
against a ~0.27 m terminal correction capacity from today's geometry. **No terminal-only
sensing or guidance trick makes a 9 m/s crosser sub-meter.** A perfect collision course
provably exists at handoff; the interceptor just doesn't fly it. So the win is dominated
by getting the target **in frame** during the dash and putting the interceptor onto a
**converged collision course EARLIER, with more t_go** — pointing is the gate; sensing,
guidance, and terminal polish are all downstream of it.

## The single biggest lever — fix the dash GEOMETRY, not the mount alone

**Loft-then-dive + accel-capped constant-known-pitch dash, co-sized with the wedge.**
This is the only lever that attacks the #1 wall at its geometric root, is sim-cheap, and
is honesty-clean (pure trajectory/own-state, no new sensor). It converts the system from
~0% camera-tracked (today's "kills" are open-loop dash ballistics) to having a live
terminal track *at all* — the precondition every other lever needs. **The wedge angle
must be chosen JOINTLY with the dash profile; freezing the wedge in isolation locks in
the wrong angle** (loft flips the sign of the pointing error the wedge is sized against).

## The combined program — a causal 4-phase stack (pointing gates everything)

- **Phase A — POINTING (unblocks everything).** Loft-then-dive + accel-capped
  constant-pitch dash, co-sized with the wedge. Restores in-flight detection from 0.8%
  toward the ~100% static rate *and* adds a dive g-component to closing speed while
  shrinking t_go. **Do this and validate it before freezing the wedge.**
- **Phase B — HOLD THE RESIDUAL (only once framed).** A look-angle-constrained PN + IBVS
  pixel-centering term that absorbs the +40°→−35° pitch swing the static wedge can't
  track — operating in pixel space, so it never differentiates a noisy bearing (the thing
  that sank the earlier APN/PIP attempts).
- **Phase C — SHARPEN ZEM/t_go (now that detections exist).** Fuse a monocular
  size-based range prior (R = fx·W/w_px from the bbox, W≈0.3 m known) into the existing
  Kalata range channel; add track-continuity/de-lag *only if* measured detection
  staleness warrants after pointing is fixed.
- **Phase D — ENDGAME (marginal → contact).** Terminal IMU-coast through the sub-1 m
  camera blind window (camera dies inside ~1 m on 20/20 flights; terminal LOS
  485–1870°/s), armed by a tau (time-to-contact) clock, propagating the commanded ZEM
  correction through impact.

## DO FIRST — sim-cheap, honesty-clean levers to test now

| Lever | Category | What it does | Decisive experiment |
|---|---|---|---|
| **Loft-then-dive dash** | pointing / kinematics | Climb 2–4 m early, dive on the co-altitude target so the nose-down pitch aims the camera *at* it instead of over it. **Biggest lever.** | Replay canonical mc_batch geometry, lofted-dive vs flat run-in; log **fraction-of-ticks target-in-frame across 8–12 m** (vs measured ~75% out). |
| **Accel-capped constant-pitch dash** | kinematics / pointing | Cap forward accel so body pitch holds one bounded value (θ=arctan(a/g)) for the whole run-in; size the wedge to exactly that pitch; reserve high thrust-to-weight for *terminal lateral* correction. | Paired-seed A/B (n≥8): sweep accel cap, jointly choose wedge+cap; log in-frame fraction, miss, and the closing-speed penalty. |
| **Terminal IMU-coast** | endgame-grab | On the FOV-edge/track-loss flag inside ~1 m, freeze the last good LOS-rate and fly the last ~0.15 s open-loop on the own-state EKF; arm with a tau clock. Highest-confidence terminal lever. | On flights with a real terminal track, A/B freeze-and-propagate vs hold-last-command; compare CPA. |
| **Size-based range → Kalata fusion** | sensing | R = fx·W/w_px from the bbox already computed, fused as a coarse prior into the range channel to sharpen t_go (which squares into 96%-of-miss). Not a resolution lever. | guidance_lab A/B (n≥8): size-R prior vs geometry-only, with W-foreshortening injected as noise; measure miss + t_go error. |
| **FOV-holding guidance term** | guidance-law | Look-angle-constrained PN + IBVS pixel-centering; wedge sets static bias, this absorbs the dynamic residual; no bearing differentiation. Prototype on the 30 fps AprilTag baseline. | *After* pointing frames the target: A/B wedge-only-PN vs +look-angle+pixel-centering; measure detection density through the pitch swing + miss. |

## Hardware bets (escalate only if the cheap geometry fixes leave residual)

- **2-axis seeker gimbal** — the *only* full elevation decoupler; kills the attitude
  coupling that causes the 0.8% recall and revives the narrow-FOV reach lever. Cost:
  servos/mass/power/complexity on a sacrificial ~950 g frame (the reason the wedge was
  chosen for simplicity). **Gate:** pursue only if wedge+loft+constant-pitch still loses
  the target during the dash.
- **Dual onboard cameras (wide cue + narrow reach)** — R_acq ~linear in focal length, so
  a narrow lens ~doubles acquisition reach, all onboard/jam-proof (the honest version of
  the fielded radar→seeker handoff). **Two adversarial holes:** it presupposes pointing
  is solved (a 2nd camera does nothing for pointing), and the staging is likely
  *backwards* — narrow-for-far-acquisition (low LOS rate) then WIDE-near-terminal (where
  the target streaks at 485–1870°/s). Dual-stream YOLO is also unaffordable on the Pi-5
  CPU without the deferred Hailo. **Gate:** validate pointing first; resolve handoff
  direction; bench the two-stream frame budget.
- **Fixed motor cant** — cant thrust axes a few degrees so some forward accel comes from
  thrust vectoring, shaving δ off the nose-down pitch (a cheap partial Roadrunner-style
  decouple). **Mostly redundant with the wedge** and small against the ~50° full-accel
  pitch. **Gate:** run the free sim proxy (camera-elevation offset sweep) first.
- **Event/neuromorphic camera (Prophesee GenX320, ~grams, GenX320 has a Pi-5 kit)** as a
  terminal-only blur-killer — no exposure window → no motion smear; <150 µs latency;
  >140 dB HDR. Attacks a real wall mono can't beat, but only bites *after* pointing is
  fixed and inside the last few meters, and event segmentation from a violently pitching
  quad is itself a research problem. **Gate:** not Gazebo-scorable — Stage-0 bench/HIL
  only; try software IMU-deblur first.

## Research dark-horses (high upside, spike before committing)

- **Track-before-detect** — integrate sub-threshold detector responses across N frames
  along the straight-line motion hypothesis; effective SNR rises ~√N, pulling the target
  out of noise at a range where a single frame is sub-threshold. A *temporal* acquisition
  lever the resolution-rejection experiment never tested (it varied spatial pixels,
  per-frame). **First probe:** pure offline post-processing of an existing flight's NN
  response maps — cheapest possible decisive test. (Needs the markerless heatmap, so it
  doesn't help the AprilTag baseline.)
- **FOV-constrained optimal / learned (RL) visual guidance** — the north-star of which
  the analytic look-angle PN is the cheap closed-form approximation; an angle-only RL
  policy (Gaudet et al.) reportedly matches augmented-ZEM-optimal miss using *only* LOS
  angle+rate, no range, sub-ms on a Pi. **First probe:** guidance_lab bake-off of analytic
  look-angle PN vs a small RL policy on capturable-region width. (Re-earns the no-cheat
  audit; sim-to-real of a learned policy is fragile.)
- **Bank-as-accel feedforward into a re-tested APN** — estimate target lateral accel from
  its bank (tanφ=a_lat/g) as a *leading* feedforward, no differentiation. Satisfies the
  standing "re-test APN" directive via a path that bypasses the noise wall — **but the
  mission target is straight-line, so the term is identically zero (== PN); pure jink
  insurance.** And reading bank off an ~8 px blurred silhouette is optically implausible.
  **Gate HARD:** offline feasibility on quad_dataset_banked first; likely dead.
- **IMU-guided motion-deblur** — estimate the terminal blur PSF from the IMU-measured
  rates and deconvolve the track ROI. Same wall as the event camera, zero new hardware —
  but deconvolution can't recover signal below noise. **First probe:** Stage-0 bench,
  before any event-camera spend.

## Deliberately excluded (and why — so we don't re-open them)

- **More megapixels / foveated crop / higher-res sensor as the acquisition lever** —
  tested & rejected: identical recall at the 8–12 m wall band; resolution matters only
  past ~24 m static. The wall is pointing, not pixels. Dead unless first paired with a
  pointing fix.
- **Color camera** (ADR-0078: 3× worse on desert clutter), **IR/thermal**, **narrow
  fixed FOV alone** (ADR-0024: crosser walks out, 0/12), **detector retrains** (detector
  is ~100% static; retraining doesn't move pointing).
- **Servo/adaptive tilt as the committed pointing fix** — rejected for simplicity
  (2026-07-17); the gimbal is the open escalation, not a servo tilt.
- **Porpoise / pitch-up re-framing pulses** — dominated by loft-then-dive (continuous
  framing for free) and manufactures the exact pitch swing we're escaping.
- **Early observability weave** — spends the scarcest resource (FOV margin) to buy a range
  the straight-line PN barely needs; gain likely below the ~1 m dropout noise floor.
- **Impact-angle-constrained ram** — an impact-angle constraint provably eats the
  lateral-accel authority that sets 96% of miss, for an unproven cross-section gain.
- **Live datalink cue / autonomous auto-fire** — violates the honesty boundary; parked.
- **Delayed-Kalman de-lag / track-continuity as headline levers** — at 0.8% in-flight
  detection there's almost nothing to de-lag; fold into Phase C only if measured staleness
  exceeds ~1 frame after pointing is fixed.

## Honest gates (repeat before any claim)

- **The sim has no motion-blur model** → every acquisition-range/detection number here is
  an **UPPER BOUND** the Stage-0 bench must confirm (terminal LOS 485–1870°/s; a 5 ms
  exposure smears ~23 px). Short-exposure global shutter is mandatory.
- **9 m/s is ZEM-hard** (ADR-0027) → Phase A (get on a converged course earlier, framed)
  is where the win comes from, not Phase C/D terminal tricks.
- **First kills fly the AprilTag** (9–12 m decode, 30 fps CPU); markerless YOLO (~5–10 fps
  CPU) needs the deferred Hailo — some levers (track-before-detect, dense pixel-centering)
  are markerless-only and don't help the AprilTag baseline.

## Key sources (external, accessed 2026-07-21)

*Guidance (FOV-constrained & angle-only):* Capturability with FOV limit (AIAA JGCD,
`10.2514/1.g003860`); Optimal Look-Angle Guidance for strapdown (`10.1155/2022/6057998`);
Moving-target any-impact-angle FOV law (AIAA `2021-1462`); RL angle-only intercept, no
range (arXiv `1906.02113`); LOS-curvature meta-RL (arXiv `2205.00085`); bearings-only
observability-enhanced PN (AIAA `10.2514/1.G003003`). *Visual servoing interceptors:*
High-speed interception multicopter by IBVS (arXiv `2409.17497`); autonomous intercept
drone IBVS (IEEE `9197539`); precise IBVS interception of flight targets. *Sensing/
endgame:* Prophesee GenX320 event sensor + Pi-5 kit; drone detection with event cameras
(arXiv `2508.04564`); event-camera obstacle dodging (Science Robotics `aaz9712`); tau /
time-to-contact perching & landing (SAGE, `10.1177/01423312221104424`); looming/LGMD
bio-collision; monocular range for forward-collision (PMC `3914606`). *Real C-UAS &
kinematics:* Anduril Anvil / Roadrunner (thrust-vector decouple); Raytheon Coyote; Fortem
DroneHunter; Ukrainian STING machine-vision interceptors; missile lofting; omnidirectional
tiltrotor (Voliro). Full list with per-finding claims in the workflow journal.

---

*Provenance: workflow `intercept-distance-levers` (15 agents; one repo-ledger lens failed
on a structured-output cap — the grounding brief carried the repo facts, and the synthesis
is repo-grounded to ADR-0023/0024/0027/0056/0064/0065/0073/0076/0078). Idea funnel: 34
generated → 34 unique → 27 survived vetting. Ram bar, pointing wall, ZEM/t_go² relation,
and the 9 m/s ZEM-hard caveat all trace to the repo; external techniques trace to the
sources above. Numbers from the no-blur sim are upper bounds pending the Stage-0 bench.*
