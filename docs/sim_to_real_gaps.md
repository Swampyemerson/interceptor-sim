# Sim-to-real shortcomings audit (T23)

*Design doc only — no sim run, no code, no commit. Companion to
`docs/phase2_sim_to_real_plan.md` §5 (the "pivotal risk") and Track D's T23
item. Written for a builder who will eventually stand next to real hardware
and needs to know, precisely, where this sim's numbers are trustworthy and
where they are not.*

## 0. Why this doc exists

*New term, used throughout — a "sim-to-real gap" is any place where the
simulation's math, rendering, or timing is easier/cleaner than what the same
physical measurement would show on real hardware. A gap that makes the sim
look BETTER than reality is dangerous — the plan calls it "flattering" — because
it can talk the builder into buying an airframe, trusting a Pk number, or
skipping a safety fix that reality won't actually support.*

Phase 2's own plan names this doc's job directly (`phase2_sim_to_real_plan.md`
§5): *"the single biggest way this blueprint could MISLEAD the hardware build
is a sim-to-real fidelity gap that flatters the design — the T23 audit exists
to surface those; treat a too-good sim number as a bug to investigate, not a
win."* This project has a track record of exactly that pattern showing up and
getting caught — six named "lab ranks, Gazebo decides" divergences already
(PIP, calibration, Kalata, absolute-Pk, mid-course-coverage, and the ADR-0052
clock-epoch bug this session). This doc is the systematic sweep: every place
the simulation stands in for something physical, is it disclosed, and does a
bench-measurable quantity exist that would replace the assumption with a real
number. Nothing here blocks Phase 2 — it is a map for prioritizing Stage-0/1
bench work and for writing an honest README caveat next to every headline
number.

## 1. Gap table, grouped by T23's dimensions

Columns: **what sim does** / **what reality does** / **severity**
(critical / major / minor — critical = touches the headline resume claim or a
safety-critical decision) / **direction** (**FLATTERS** = sim looks better
than reality, the dangerous kind; *conservative* = sim is harsher than
reality; *uncertain* = untested either way) / **bench-measurable quantity**
that would close the gap, plus the earning ADR/doc.

### Frames / rendering

| # | What sim does | What reality does | Severity | Direction | Bench-measurable | Source |
|---|---|---|---|---|---|---|
| F1 | Ground-rig L/R frames are pre-rendered once along ONE deterministic mover path per seed/direction, then replayed through the real detect→triangulate→track code at flight time ("trajectory-matched offline-render replay") | A live rig continuously exposes on whatever is actually crossing the wedge — frame timing, coverage, exposure/focus, and unscripted occlusion all vary in real time | major | **FLATTERS** | Live-rig throughput/coverage probe: fraction of a moving target's wedge-transit time actually captured + detected by cameras streaming in real time (the pre-registered `(b)` comeback gate in ADR-0046 — needs low-res + d3d12 + a real binary-subscriber drain) | ADR-0046 |
| F2 | Ground detector's negative (no-target) training/eval set is 60 byte-identical frames of one flat, textureless ground + uniform sky | Real backgrounds are cluttered — horizon line, structures, foliage, sun glare, birds — the actual false-positive challenge | critical | **FLATTERS** hard | Real-sky negative capture campaign (hours of footage, varied backgrounds/times) → measured false-positive rate/hour | ADR-0049 |
| F3 | Ground detector's val positives span **160.06–161.93 m**, one fixed apparent box size (35.2×35.2 px); "near/far" split is off-boresight ANGLE, not range | Real targets cross the full ~60–160 m EO envelope (ADR-0017/0019); apparent size shrinks toward the ~10 px detection floor | major | **FLATTERS** | Multi-range capture at several rig standoffs/target ranges → detector recall vs apparent pixel size, and σ_R vs R² scaling (stereo_design.md bench test #5, extended) | ADR-0049, ADR-0050 |
| F4 | L/R frames captured from an identical frozen pose per snapshot (teleport pattern) → sync error is exactly zero by construction | Even a hardware-GPIO-triggered rig has nonzero sync error (ADR-0017 target: <20 µs / ~0.1% of variance); a software-synced rig would add ~0.75 m at 100 m vs a fast crosser | minor–major (depends on real trigger quality) | **FLATTERS** | Strobe a blinking LED, compare captured phase between the two cameras (stereo_design.md bench test #3) | ADR-0046, ADR-0017 |
| F5 | One static rig pose (`broadside_160m`); coverage is a single wedge, 91%/91% both dash directions, matched to the flown corridor by construction | A fielded rig must cover an unknown approach azimuth — "coverage is a WEDGE, not a dome; wide area = multiple rigs" | major | **FLATTERS** | Field-survey candidate siting + compute per-site wedge coverage against a real (or conservatively assumed) threat-approach-azimuth spread; size the multi-rig count from that | ADR-0017, ADR-0046 |
| F6 | Acquire-then-track + a downlinked-position identity gate is a *design*, never flight-tested with both airframes visible/near-coincident in the rig's 2D view | When interceptor and target are genuinely 2D-coincident in the ground camera, only 3D/range separation or the downlink disambiguates — a named open weak point | major | **FLATTERS** (untested = optimistic by omission) | Fly both airframes on crossing paths through the rig FOV; verify track custody holds under the ADR-0047 identity gate | ADR-0047 |

### Timing / latency

| # | What sim does | What reality does | Severity | Direction | Bench-measurable | Source |
|---|---|---|---|---|---|---|
| T1 | (Fixed this session) Cache-clock is rebased to the live `/clock` at station spawn, so ground↔drone timestamps align by software construction, in one process, on one shared simulation clock | Ground rig and drone are two independent embedded clocks; a shared time datum (PPS pulse + RTK-disciplined clock) is required, and even a good one leaves a nonzero residual skew | **critical** | **FLATTERS** | Log timestamped events on both real nodes against a shared PPS/RTK reference; report residual skew distribution (target: sub-µs per the design, ADR-0015/0017) | ADR-0052 (headline finding this session) |
| T2 | Ground detection is PRECOMPUTED offline over deterministic frames, then replayed as cached centroids through live triangulation + a modeled latency floor — zero flight-time detection compute cost | A real ground detector runs concurrently with everything else on shared/adjacent compute — genuine GPU/CPU contention is a live, variable latency source | major | **FLATTERS** | Profile real ground-detector inference latency (Jetson Orin NX / Hailo-8) WHILE the rest of the real pipeline (triangulation, track, link) runs concurrently — p50/p90/p99 vs the sim's fixed floor | ADR-0051 |
| T3 | The only measured detection latency in the project is `.venv-seeker`'s CPU-only onnxruntime (~0.85–1.25 s/frame-pair, no CUDA) | Deployment hardware is Hailo-8 (~29 ms EXPECTED, ADR-0016) or Jetson TensorRT (~15–20 fps/stream) — a different accelerator/software stack entirely | minor | uncertain (not comparable, not necessarily optimistic — CPU is slower, so this specific number undersells rather than flatters) | Export the same detector to Hailo-8/TensorRT and measure real end-to-end latency on the actual target accelerator | ADR-0051, ADR-0016 |
| T4 | The whole ground+air pipeline (detect, triangulate, fuse, guide) runs today on one desktop PC's CPU/GPU, not on the embedded targets the BOM commits to (Jetson Orin NX ground / Pi 5 + Hailo-8 air) | Embedded SoCs have real thermal throttling, shared-bus contention, and power ceilings a desktop doesn't | major | **FLATTERS** | Run the exact exported models on the real Jetson Orin NX / Hailo-8 hardware; measure latency + thermal throttling over a sustained (not single-shot) session | ADR-0016, ADR-0051 |
| T5 | Headless render silently defaults to `llvmpipe` (CPU rasterizer) unless `GALLIUM_DRIVER=d3d12` is exported to use the RTX 4070; RTF — and so any wall-clock-derived timing claim — depends heavily on which was active | No analog — a real system's camera + GPU/NPU are always "on," there's no renderer-backend choice | minor (sim-internal bookkeeping hazard, not a physical-world gap) | *methodological hazard, not flatter/conservative* | None (not a physical quantity) — mitigation is procedural: always record `GALLIUM_DRIVER` in run metadata before citing an RTF or wall-time number | ADR-0046 addendum, ADR-0009 |
| T6 | Cue delivery latency is a controlled sim-time floor (dual-gate buffer-release), decoupled from wall-clock compute under the sim-time-discipline rule | Real end-to-end latency is genuinely chained wall time: NN inference + triangulation + serialization + radio + own-EKF fusion (ADR-0016's sourced BEST ~20 / EXPECTED ~90 / WORST ~210 ms) | major | **FLATTERS if the offline floor under-samples true worst-case contention** (the ADR-0048 §3 "flip evidence" case) | Measure end-to-end wall latency of the deployed pipeline at the chosen radio, repeated under load; compare distribution to the sim's fixed floor | ADR-0016, ADR-0048 §3 |

### Comms / link

| # | What sim does | What reality does | Severity | Direction | Bench-measurable | Source |
|---|---|---|---|---|---|---|
| C1 | The `:47800` UDP JSON cue link runs on `localhost` loopback — zero packet loss, no bandwidth ceiling, no RF propagation; the ~90-byte/7.2 kbps "real radio" comparison is a *written* derivation, never enforced in code | A real 32 kbps SiK radio (or jam-resistant MANET) has range-dependent SNR, multipath, and is the actual target of jamming | **critical** (this is the headline jam-resistance claim's transport) | **FLATTERS** | Run the chosen real radio over a real link (range test, RF interference) and measure actual throughput/packet-loss vs the assumed 7.2 kbps / dropout model | ADR-0048 §1, ADR-0016 |
| C2 | Cue dropout is modeled as a hand-tuned Markov bursty process (`--dropout-markov`), sized by "tail shape > wider Gaussian" reasoning, not a real link capture | Real dropout statistics depend on the actual radio, terrain, and (adversarially) a real jammer's waveform | major | **FLATTERS** (the model is a designer's best guess, always tunable to look survivable) | Capture real dropout statistics from the chosen radio under representative RF conditions; refit the Markov parameters to the real trace | ADR-0016 (latency verdict), s2_cue_mock.py dropout model |
| C3 | Datum-bias/spoof injection strength (`--datum-bias-m`, the track-message spoof injector) is entirely sim-authored — the project picks the adversary's power | A real spoofer's capability (GPS-spoof sophistication, RF power, timing-injection precision) is adversary-controlled and unbounded by our tiers | **critical** (the whole fusion-survives-jam story is tested only against a self-chosen threat) | **FLATTERS** | Not a simple bench number — needs a controlled RF-range red-team test against the real datalink hardware to characterize achievable spoof power/precision, then re-tune the WORST tier from that | ADR-0045 §5.6 #2, ADR-0035 (jam doctrine) |

### GPS / datum

| # | What sim does | What reality does | Severity | Direction | Bench-measurable | Source |
|---|---|---|---|---|---|---|
| G1 | Datum bias (0.3 / 0.5 / 2.5 m BEST/EXPECTED/WORST) is a constant *sourced from RTK/PPS literature*, never measured on this project's own hardware | Real residual bias depends on satellite geometry, per-site multipath, base-station survey accuracy, and rover convergence time — time-varying and site-specific | **critical** (this constant sets the whole mid-course fusion accuracy budget, ADR-0018, and gates T20 adoption) | *uncertain* (sourced, plausible, but unverified — not yet shown to flatter or be conservative) | Side-by-side RTK-GNSS survey of the rig position and the drone's own GPS/EKF2-estimated position at a known true offset; log residual bias over an hour under real sky | ADR-0017 |
| G2 | The mock's single `--datum-bias-m` constant models ONLY the rig-side offset; the drone's own position (used to convert the ground track into its own NED frame) is treated as effectively exact | The drone's own GPS/EKF2 has its own, separate error that stacks on top of the rig's survey error | major | **FLATTERS** (sim implicitly halves the real combined error budget) | Fly the real drone with logged EKF2 position vs an independent RTK ground truth to measure real own-position error; combine (not replace) with the rig-side survey error | `docs/fusion_capstone_design.md:105-107` |
| G3 | Rig and drone share ONE Gazebo world coordinate frame — perfect global registration except for the explicitly injected `--datum-bias-m` | Rig and drone each run their own independent GPS/EKF2/survey stack in the real world; there is no shared ground-truth frame to fall back on | minor–major | **FLATTERS** mildly | Same as G1/G2 — this row is the structural reason G1/G2 exist; no separate bench test | `phase2_sim_to_real_plan.md` §5.5, ADR-0045 §5.6 #2 |

### Camera intrinsics vs real lens

| # | What sim does | What reality does | Severity | Direction | Bench-measurable | Source |
|---|---|---|---|---|---|---|
| L1 | Gazebo's camera sensor is an ideal pinhole model — zero radial/tangential distortion, zero chromatic aberration, zero vignetting; the onboard `mono_cam` HFOV is exactly 1.74 rad (99.7°) with no calibration error | Any real lens (even a good C-mount/M12) has measurable distortion; a naive port that skips distortion correction misplaces bearing, especially off-boresight | major | **FLATTERS** | Checkerboard/ChArUco calibration of the real lens (`calibrate_camera.py`, already staged); compare measured distortion coefficients + reprojection RMS to the assumed zero (target ≤1.0 px) | `docs/stage0_bench_plan.md` §2.4 |
| L2 | The camera renders a mathematically sharp frame every tick regardless of angular rate — no motion blur, no rolling-shutter skew modeled at all, even though global shutter was chosen precisely to *minimize* (not eliminate) this in the real design | Even a global-shutter sensor has finite exposure time; at terminal-phase LOS rates (measured 32–58°/s at CPA, yaw clamp 60°/s) real frames will show motion blur exactly when the geometry is most demanding | **critical** — repeatedly named "existential, not a tuning nuisance" across ADR-0012/0015/0023 and the seeker docs | **FLATTERS** hard — the single most-repeated known optimism source in this codebase | Mount the real camera on a rotating fixture at terminal-phase yaw rates (ramp to ≥60°/s); measure detection rate falloff (`docs/stage0_bench_plan.md` §3c, "yaw-rate/motion-blur test") | ADR-0007, ADR-0010, ADR-0012, ADR-0023, `seeker_design_brief.md` R3 |
| L3 | Ideal, uniform, emissive-boosted contrast; the tag/target material needed an emissive-lighting hack (ADR-0007) just to be bright enough for the sim's small on-screen cells at range — there is no auto-exposure hunting, glare, haze, or backlight | Real EO detection is contrast- and lighting-dependent; a washed-out sky, backlight, or low sun measurably shrinks the detection envelope, on top of the already-disclosed daytime-only limit (ADR-0019) | major | **FLATTERS** | Repeat the detection-floor test across real lighting conditions (indoor / outdoor-sun / backlit), log recall vs range (`docs/stage0_bench_plan.md` §3d) | ADR-0007, `stage0_bench_plan.md` §3d |
| L4 | Sim intrinsics (fx=fy=539.9 px, HFOV 99.7°, 1280×960) are fixed by construction and never drift | Real intrinsics belong to the specific lens/body pairing and must be measured; a 2% tag-size or fx error biases every range by 2% | major | **FLATTERS** | `calibrate_camera.py` on the real rig, watch RMS reprojection error (recapture until ≤1.0 px) | `stage0_bench_plan.md` §2.4/§2.5 |

### IMU / EKF2

| # | What sim does | What reality does | Severity | Direction | Bench-measurable | Source |
|---|---|---|---|---|---|---|
| I1 | PX4 SITL's own-state estimate (`attitude_euler`, `position_velocity_ned`, `relative_altitude_m`) comes from a simulated IMU/GPS/EKF2 stack; per `compute_setup.md`, sim clocks tick in **lockstep** with GPS time — own-state timing is synthetically exact, and this own-state feeds the guidance's yaw-compensated LOS math (λ = ψ + β) directly | A real Pixhawk 6C's EKF2 runs on a real IMU under real airframe vibration, real magnetometer interference near motors/ESCs, real GPS multipath/dropout, with no lockstep guarantee | major (own-state error propagates straight into commanded acceleration) | **FLATTERS** — the same "own-state is legal, and near-perfect" gap already flagged for perception, extended explicitly to IMU/EKF2 | Log real EKF2 innovations/variance and attitude vs an independent reference (mocap or a second high-grade IMU) during representative maneuvers on the real airframe | `compute_setup.md:20`; "own-state ... legal" language repeated across ADR-0008/ekf_design_brief.md/terminal_guidance_realworld.md |
| I2 | Rigid-body physics only — no motor/prop vibration coupling into any simulated IMU reading | Vibration is a first-order EKF2 health/tuning concern on real multirotors (PX4 tracks it as a dedicated telemetry field) and degrades attitude estimate quality, feeding straight into λ | major | **FLATTERS** | Log PX4's own vibration metric at cruise/dash/terminal throttle on the real airframe; compare EKF2 attitude variance across throttle regimes | Cross-cutting — named in ADR-0012 #1 risk and every seeker doc's "no vibration model" caveat |

### Thermal / night

| # | What sim does | What reality does | Severity | Direction | Bench-measurable | Source |
|---|---|---|---|---|---|---|
| TH1 | The entire pipeline (onboard + ground) is daylight EO only; no IR/thermal sensor model exists anywhere in the repo | Deployment envelope explicitly excludes night/dawn/dusk/low-contrast without a staged mono-LWIR add-on; a fielded EO-only system is blind exactly when many drone incursions occur | **critical**, but **honestly disclosed already** — this is a scope boundary the project states outright, not a hidden trap | *not flattering — openly parked, no sim number claims night capability* | The staged thermal add-on needs its own bench (a mono uncooled LWIR core, co-boresighted) — out of scope for this audit, deferred by design | ADR-0019, ADR-0015 |

### Safety interlocks

| # | What sim does | What reality does | Severity | Direction | Bench-measurable | Source |
|---|---|---|---|---|---|---|
| S1 | The bird/hostile `P(hostile)` interlock is NOT ratifiable/buildable as written; it defaults to **VETO-ALL** (engages nothing) until 8 red-team fixes ship and a Stage-0 bench produces real `P_correct(R)`; any sim gate for it is explicitly disclaimed as "plumbing-validated," an upper bound, never a real bird-rejection number | Real bird rejection needs a measured ROC against real birds/decoys under WORST-tier conditions (matched-footprint decoy, flapping, real sky) | **critical** (this is the safety-critical interlock; a false engage on a bird — or worse — is the worst failure mode of a weapon-adjacent system) | *no flattering sim number exists here* — the danger is a **future** sim ROC getting mistaken for a real one, which ADR-0035 already warns against by name | The Stage-0 bench's own prerequisite: WORST-tier decoy + real-sky `P_correct(R)` sweep (ADR-0035), gated on Stage-0 hardware existing | ADR-0035 |
| S2 | Arm/engage state transitions are pure software (detection streak, range thresholds); "abort" in sim just stops commanding velocity | A real system needs a hardware kill link — an ELRS RX manual-override, never read by guidance — that doesn't exist yet and has never been bench-tested for cutoff latency/reliability | **critical** (a kill switch too slow is a real safety incident) | *no sim analog exists* — nothing in sim speaks to this at all, which is itself the gap | Command an abort via the real ELRS RX at various ranges/throttle settings; measure actual time-to-motor-cutoff | ADR-0012 (link/integration line), ADR-0035 item 4b |

### Airframe / physics realism (extra group — flagged, not in the plan's original 8-item list)

*T22's FPV-fidelity design work surfaced two concrete, bench-measurable gaps
this session that don't cleanly fit any of the plan's eight named dimensions.
Included here transparently rather than force-fit or dropped; recommend the
main session fold this group into T23's dimension list if it agrees.*

| # | What sim does | What reality does | Severity | Direction | Bench-measurable | Source |
|---|---|---|---|---|---|---|
| A1 | Grepped every `.sdf` in the x500 model tree: only the multicopter-motor-model plugin exists (4×, one per rotor) — **no** `LiftDrag`/aerodynamics plugin anywhere; a payload's parasitic drag is entirely unrepresented, and no PX4 parameter can add it | A real Pi5+camera-pod payload punching through the air at 25+ m/s has real frontal-area drag that costs top speed, acceleration, and endurance, and shifts trim | major (directly caps the credibility of any `--fpv-fast` dash-speed number) | **FLATTERS** | Thrust-stand + GPS-logged max-speed flight test of the real payload-carrying airframe (with/without a payload mockup) vs the sim's commanded profile | `docs/fpv_fidelity_design.md` §3, §6 |
| A2 | The FPV profile's committed `MPC_TILTMAX_AIR = 60°` (70° under `--dash-unclamp`) already exceeds the x500's own derived physical steady-state tilt ceiling (~53°, from T/W≈1.67); PX4's attitude loop degrades "gracefully" — horizontal saturates or altitude sags — with no crash and no controllability-loss modeled | A marginal real 7" build pushed past its own T/W ceiling risks real altitude/control-margin loss under gusts, ESC/motor thermal limits, or battery sag the idealized controller may not represent | minor–major | **FLATTERS** mildly | Fly the real airframe at commanded tilts approaching/exceeding its T/W-derived ceiling (with margin/altitude), log achieved tilt/altitude-hold/battery sag vs commanded | `docs/fpv_fidelity_design.md` §3 |
| A3 | True payload mass/inertia/CG shift is NOT modeled without an SDF edit; the available proxy (thrust-derate via `MPC_THR_MAX`/`MPC_ACC_HOR_MAX`) changes only what the controller is *allowed* to ask for, not the vehicle's real mass, inertia, CG, or drag — and structurally can't capture that a heavier craft is also harder to *stop* (more momentum into the terminal phase) | A real heavier payload changes stopping distance/terminal dynamics directly | major | **FLATTERS** | Weigh + measure CG of the real payload-carrying build; compare achieved deceleration/terminal dynamics against the sim's thrust-derate proxy | `docs/fpv_fidelity_design.md` §6 |

## 2. Which gaps FLATTER the design (the dangerous subset)

Per the plan's own rule — *"treat a too-good sim number as a bug to
investigate, not a win"* — this is the list to re-check before trusting any
sim number against a real build decision. All 20 rows below carry a sim
number, structure, or default that reads better than what the equivalent real
measurement would likely show:

- **F1** — offline replay guarantees the target is always exactly where the pipeline expects; a live rig can miss/mistime frames the way ADR-0052 just proved a *software* clock can.
- **F2** — the detector has never seen a real false-positive stress case (clutter, birds, glare) — its true false-positive rate is unmeasured, not zero.
- **F3** — detection/σ_R is validated at ONE range/apparent-size; the envelope edges (near the 60 m detection floor) are unproven.
- **F4** — stereo sync error is zero by construction; a real rig has to earn a sub-20 µs trigger.
- **F5** — the target always flies into the rig's known wedge; an off-azimuth approach is never tested.
- **F6** — two-drones-in-frame disambiguation is a design, not a proven mechanism.
- **T1** — the epoch-rebase fix makes clock alignment perfect by software construction; a real PPS/RTK datum only ever achieves a bounded residual skew.
- **T2** — precomputed detection means the sim has never once modeled real GPU/compute contention.
- **T4** — desktop-class compute stands in for two power/thermal-constrained embedded targets.
- **T6** — the modeled latency floor may under-sample true worst-case compute contention (the ADR-0048 §3 "flip evidence" case, unverified).
- **C1** — a lossless loopback UDP link stands in for a jammable RF radio; this is the crux of the whole jam-resistance headline.
- **C2** — the dropout model is a self-authored guess, always tunable to look survivable.
- **C3** — the spoof/jam adversary's strength is chosen by us, not by a real adversary.
- **G2** — the drone's own GPS/EKF2 error is missing from the combined datum-bias budget, likely understating it.
- **G3** — one shared Gazebo world frame stands in for two independently-surveyed real ones.
- **L1/L4** — zero lens distortion and exact intrinsics stand in for a real, uncalibrated lens.
- **L2** — **the single most-repeated known gap in this codebase**: zero motion blur / rolling-shutter effects, named "existential" three separate times (ADR-0012, ADR-0015, ADR-0023).
- **L3** — ideal, emissive-boosted contrast stands in for real outdoor lighting.
- **I1** — PX4 SITL's own-state (attitude/position) is near-ideal and lockstep-timed; a real EKF2 fights vibration, magnetic interference, and multipath.
- **I2** — zero vibration coupling into the simulated IMU at all.
- **A1** — zero aerodynamic drag anywhere in the airframe model; every FPV speed/accel number is drag-free.
- **A2** — the sim shows graceful degradation past the airframe's own physical tilt ceiling; a marginal real build might not.
- **A3** — a thrust-derate proxy stands in for real mass/inertia/CG/drag changes.

*Not on this list, and worth naming explicitly: **S1/S2** carry no
falsely-reassuring sim number at all — the interlock defaults to veto-all and
no kill-link exists in sim to be optimistic about. That is a different, but
equally real, kind of risk: an unbuilt safety gap, not a flattering one. **TH1**
is likewise excluded — it is openly disclosed scope (EO-only, no night claim
made), not a trap. **T3** measures the wrong hardware but in the pessimistic
direction (CPU is slower than the real accelerator), so it doesn't flatter.
**G1** is sourced from literature but unverified on real hardware — treated as
uncertain, not yet shown to flatter.*

## 3. What to measure first on the Stage-0 bench (prioritized)

Prioritized against what hardware actually exists today: **Stage-0 is a
single Pi 5 + camera bench** (`docs/stage0_bench_plan.md`, fully designed,
**parts not yet ordered** — NEXT.md open item). The ground stereo rig, a real
radio, a real airframe, and RF/red-team test ranges are all later stages. This
list is ordered so the cheapest, already-planned, highest-leverage
measurements come first.

**(a) Measurable TODAY with the already-designed Stage-0 cart (~$257, design-review §6 re-cost — order now; closes L1–L4, informs T3/T4):**
1. **L2 — motion-blur/yaw-rate ramp** (`stage0_bench_plan.md` §3c). Highest priority: this is the single gap named "existential" more than any other in the project's own history. Answers whether the sim's zero-blur assumption is safe up to the ~60°/s terminal LOS rate.
2. **L1/L4 — lens calibration + distortion** (§2.4). Cheap, mandatory before any other number on real hardware is trustworthy (a bad fx silently rescales every range).
3. **Sustained detection Hz on the Pi 5 CPU** (§3b) — the project's own "headline unknown ADR-0012 could not answer"; also the first real data point for T3/T4 (embedded-vs-desktop compute).
4. **L3 — lighting variation** (§3d). Cheap add-on to the same rig.
5. **F3-adjacent — static detection rate/pose error vs range** (§3a). Only exercises the *onboard* detector, not the ground rig, but is the same "does detection hold across the range envelope" question F3 asks, and it's the cheapest place to start answering it.

**(b) Needs the (not-yet-built) ground stereo rig — closes F3–F5, G1:**
6. **F4 — hardware trigger sync verification** (blinking-LED phase test, stereo_design.md bench test #3). Cheapest of the rig-dependent tests once the rig exists.
7. **F3/F5 — multi-range, multi-pose capture campaign** (stereo_design.md bench test #5, extended per ADR-0050's consolidated follow-up). Validates σ_R∝R² AND detector generalization AND provides the truly held-out data T20's adoption gate needs — one campaign, three deliverables.
8. **G1 — real RTK/PPS datum-bias measurement.** Side-by-side survey of rig position vs drone GPS/EKF2 position at a known offset.
9. **T1 — residual clock-skew measurement** after a real PPS/RTK sync is wired up.

**(c) Needs the real airframe / flight hardware — closes I1–I2, A1–A3, S2:**
10. **I1/I2 — real EKF2-vs-reference attitude/position error** under real vibration, at real throttle settings.
11. **A1/A2 — real payload drag + tilt-ceiling flight test.**
12. **S2 — kill-link (ELRS RX) latency test.** Should not be deferred just because it's "later stage" — this is a safety item, not a performance one; consider pulling forward alongside first real flights regardless of Stage-0/1/2 sequencing.

**(d) Needs an RF/red-team test range or organizational scope beyond a single builder — lowest near-term priority, flag rather than schedule:**
13. **C1/C2 — real radio packet-loss/bandwidth characterization.**
14. **C3 — spoof/jam capability characterization.** Genuinely needs a threat-model/red-team exercise, not a simple bench number; do not attempt to fake this with more sim knobs.
15. **S1 — the bird-discrimination bench.** Explicitly gated on Stage-0 hardware existing AND the 8 red-team mechanization fixes shipping first (ADR-0035) — correctly sequenced last, not neglected.

## 4. Honest limits + what's explicitly out of scope

- **The honesty boundary is a strength here, not a gap — say so plainly.**
  `gt_*` fields are scoring-only and structurally unreadable by any guidance
  or fusion path after handoff (`audit_per_tick.py`, `tests/test_honesty_static.py`,
  re-earned on every new cue path per CLAUDE.md). The ground detector's
  training LABELS use ground truth (`gt_*`) at **training time only** — the
  same boundary a human labeler would use, not a leak into the live pipeline.
  This should not be read as one of the flattering gaps above; it's the one
  place this project has deliberately made the *harder* honest choice.
- **Not every row here is "measured from a logged run."** Some (A1's plugin
  grep, F1's replay-mechanism description, L1's ideal-pinhole claim) are
  structural/architectural facts about the code and models, correctly cited
  to the ADR/doc where they were established, rather than a fresh CSV. That
  distinction is called out per-row above; nothing here is asserted without a
  source.
- **Explicitly out of scope for this doc** (belong to other named work, not
  duplicated here): T24's real-world NN transfer plan (a separate design doc
  problem — MIT model vs fine-tune, licensing); T21's higher-speed/maneuver
  coverage question (a sim-only finding, not a sim-vs-real gap); T20's
  fusion-refinement adoption criteria (depends on data this audit doesn't
  generate); the full bird-discrimination ROC/`P_correct(R)` study itself
  (ADR-0035, gated on Stage-0 hardware); a wide-area multi-rig siting study
  (parked in `deployment_phases_design_brief.md`). This doc names the gaps and
  the bench tests that would close them; it does not run any of them.
- **This is a living map, not a closed audit.** New findings from T18's second
  multi-range capture, T19/T20's real-cue flights, T21's faster/maneuvering
  arms, and any Stage-0/1 bench results should be folded back in here (or a
  numbered addendum) rather than left scattered across ADRs — that's the
  whole point of having one place a reader can check "is this number real."

## Sources

Everything above traces to an ADR in `docs/decisions.md` or a design doc in
`docs/`, cited inline per-row. No number in this doc is asserted without a
source; several rows (G1, T6, C1–C3) are explicitly flagged **uncertain/needs
a real measurement** rather than given a false-confidence severity rating.
