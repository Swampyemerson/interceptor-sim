# Interceptor Sim — Final Design Review: What the Sim Proves, What Reality Will Test

*Deliverable synthesizing three verified analysis dimensions (design gaps, interfering real-world variables, cost-effective implementation), with adversarial-review corrections applied. All numbers trace to a logged run, an ADR, a repo document, or a cited price search; estimates and uncertainties are flagged inline.*

---

## 1. Executive summary

The simulation has genuinely proven the core of the concept: a quadcopter running **proportional navigation** (a guidance law that steers on the *rotation rate* of the line-of-sight to the target, not the target's position) can take a ground-station cue, acquire a target with its own camera — including a markerless neural-net seeker with no fiducial — and complete intercepts against 12 m/s maneuvering targets, with a clean kinematic diagnosis (miss distance is a near-deterministic function of the geometry at camera handoff, r² = 0.96, ADR-0023) and a real computed stereo cue flying the pipeline end-to-end (ADR-0045..0052). That is a real result and it is honestly instrumented.

The gap to a real intercept is concentrated in four places. First, **every perception number is an upper bound from clean renders**: no motion blur, no exposure dynamics, no clutter, no vibration — and the terminal phase, where the design has ~0.72 m of correction capacity and a 0.41 s median window, is exactly where those effects peak. Second, the **comms-denied headline currently fails closed**: verified in code, a cue jammed *before* camera acquisition leaves the anti-phantom gate comparing against a frozen stale position, which rejects the real target — the drone doesn't chase phantoms, it simply never hands off. That is a mission-kill against the exact scenario the project exists to prove. Third, several load-bearing estimator inputs — range-from-box-width, the mock cue's white noise, lockstep-perfect own-state — have never been stressed in the regime where they gate decisions. Fourth, the statistics are thinner than the claims: 16/16 with zero failures gives a 95% lower confidence bound of ~79% Pk (Clopper-Pearson), honest as "no failures observed," not yet as "Pk ≥ 95%." Everything below is a costed plan to close these in de-risk-per-dollar order, starting with $0 desk work available today.

---

## 2. Design gaps (ranked, corrections applied)

Each entry: what it is, the assumption it leans on, why it matters, whether the sim masks it, and the cheapest exposure. Ranking follows the adversarially corrected priority order. Two items from the original analysis were merged (terminal-freeze folded into the range-fragility item) and two were downgraded with corrected mechanisms (own-state yaw, CSRT drift).

### Tier 1 — attack these first

**G1. Terminal motion blur / rolling shutter / vibration — CRITICAL**
- **What:** All detection statistics, tag and markerless, come from instantaneous, emissive-boosted Gazebo frames. The real terminal phase runs 32–58 deg/s line-of-sight rates (ADR-0023) — the target smears across pixels exactly where the design has the least slack (coast begins at 2–3.5 m by design; median terminal window 0.41 s).
- **Load-bearing assumption:** Real terminal imagery yields detection rate and bearing noise comparable to clean renders.
- **Why it matters:** Bearing noise is the identified residual gap vs. the tag seeker (ADR-0042/0043), and the terminal budget (0.72 m correction capacity) has almost no headroom to absorb a degraded seeker. This is the project's own most-repeated "existential" gap (sim_to_real_gaps.md L2).
- **Masked by sim:** Yes, completely.
- **Cheapest exposure:** Free, today, offline — post-process already-captured sim frames with synthetic blur matched to logged LOS rate × candidate exposure times (2/5/10 ms), replay through the NN+CSRT chain, plot detection rate and bearing error vs. blur length. This also settles the plausible-but-unevidenced claim that CSRT (the correlation tracker) degrades faster under blur than the NN. The Stage-0 yaw-ramp bench (~$255 cart, already build-queue #1) is the hardware closer.

**G2. Range-from-box-width is calibrated to one frozen target aspect — CRITICAL** *(freeze/throttle item merged in)*
- **What:** Markerless range (r_hat) comes from bounding-box width, calibrated to R² = 0.935 (T17-v2) — but against a target whose mover sets position only, never attitude (ADR-0010 #6), with an NN trained on that same fixed aspect. A real quad yawing and banking through a weave changes apparent width — roughly 1.5–2× between front and side planform (estimated, not measured; props and bank add more) — which maps directly into range error. Note the honest scoping: viewing-geometry aspect change *is* in the training domain; target-*attitude*-driven aspect change is the untested piece.
- **Load-bearing assumption:** Box width is a function of range alone.
- **Why it matters:** r_hat is not a passive log. It drives closing speed Vc (hence the pro-nav gain product N·Vc), the ≤10 m handoff gate, the V_CLOSE throttle band, TERMINAL_FREEZE (the deliberate endgame concession where commanded velocity freezes because the LOS-rate math goes singular at close range), breakoff arming, and the camera-implied NED position feeding *both* ADR-0058 anti-phantom gates. A range bias correlated with the maneuver poisons all of them at once — worst exactly when the target is hardest. The freeze failure is asymmetric and costs either way: range biased high → freeze fires late → the loop is live inside the singular region (the −53 deg/s blowup the freeze exists to prevent); biased low → early freeze → longer ballistic coast → more conceded miss.
- **Masked by sim:** Yes — the mover never rotates and the freeze constants were tuned with honest tag-geometry range.
- **Cheapest exposure:** Sim, $0: (a) give the mover a yaw schedule (±90° through the crossing), re-run the T17 range-honesty regression plus one weave-12 arm n=8; (b) inject range bias directly — multiply the measured range by fixed 0.7×/1.3× and by an aspect-correlated schedule, paired n=8 per case. One env-var knob in the measurement path.

**G3. Cue trustworthiness — the cue was promoted to "truth reference" without re-validation — CRITICAL** *(corrected mechanism + two missing project-critical items folded in)*
- **What:** The ADR-0058 anti-phantom fix ("phantom" = a high-confidence false detection; pre-fix, phantoms ~18 m off caused 12/16 false handoffs with *inverted* confidence) validates handoff and seeding against the latest cue position. That promoted the cue from steering hint to gating truth reference. Three unexamined dependencies follow. **(a) Corrected jam mechanism:** verified in code, `last_cue_pos` is set only on fresh cue datagrams and never expires — a cue jammed mid-dash leaves the gate comparing against a *frozen stale* position, which against a 12 m/s target rejects the real detections within ~1 s. The system **fails closed**: no phantom chase, but no handoff either — a mission-kill in the comms-denied headline scenario. (The original "reverts to 12/16 phantoms" mechanism was wrong and is withdrawn.) **(b) Ground detector false positives (gap F2, project-rated CRITICAL):** the ground detector's training negatives were ~60 near-identical frames of flat ground+sky; a ground-side false track steers the dash *and* validates a wrong seed *and* passes the handoff gate simultaneously — the cue's own detections have never been shown trustworthy on real backgrounds. **(c) Time sync (gap T1, the project's own headline critical, ADR-0052):** residual ground-to-air clock skew shifts the PIP lead solve (~1.6 m per 100 ms at 16 m/s dash) and the gate comparisons; the sim fixed the epoch bug by software construction, which reality cannot.
- **Load-bearing assumption:** The cue is alive, honest, and time-aligned through the moment of camera handoff.
- **Why it matters:** The jam-resistance thesis is "when the datalink is denied, the camera finishes the intercept." Right now that holds only for link loss *after* handoff — cooperative timing an adversary jammer does not provide.
- **Masked by sim:** Yes — every validated batch flew a healthy, lossless, perfectly-synced loopback cue to the handoff moment.
- **Cheapest exposure:** Sim, $0 in parts: (a) a `--cue-kill-range` fault knob stopping cue packets at randomized ranges before expected acquisition, re-fly weave-12 n=16 — the metric is **intercepts completed under pre-acquisition denial** (expect near zero; quantify the exposed window), *not* phantom count; (b) injected clock-skew tiers on cue timestamps; (c) desk work, today: run the exact ground detector against real-sky/treeline footage for a first false-positive rate (see Section 5, Action 1).

**G4. Markerless acquisition range is the binding Pk lever — and it is open — HIGH/CRITICAL** *(added; named missing item)*
- **What:** ADR-0023 established acquisition range / handoff geometry as *the* Pk lever (Pk = probability of kill). The markerless seeker acquires terminal-only; 2/16 headline flights never handed off at all. The reviewed analyses critiqued gate correctness and estimator inputs but the open constraint that directly attacks the kinematic linchpin is simply: at what range does the NN see a real, fast, maneuvering quad? The false-*negative* side of sim-to-real NN transfer (does a Gazebo-trained, single-body, fixed-aspect net detect a real quad at range at all?) is unmeasured; T24 is a named but unwritten plan.
- **Load-bearing assumption:** Real acquisition range is at least the sim's terminal-acquisition envelope.
- **Masked by sim:** Yes — one rendered target, one world, in-domain everything.
- **Cheapest exposure:** $0 desktop first probe: run the exact detector on real outdoor footage of a hobby quad at taped ranges/aspects; report detection rate vs. range. Then Stage-1 outdoor sweeps.

**G5. Strictly planar engagement — no vertical guidance channel exists — HIGH, upgraded** *(promoted per adversarial verdict)*
- **What:** Pro-nav here is horizontal-plane-only. Altitude is an independent P-loop to a *fixed* 0.5 m reference (m4_intercept.py:247); the target never changes altitude; camera elevation is measured but unused. A target that climbs or descends at even 1–2 m/s creates vertical miss the design cannot correct — the loop tracks a fixed reference, not the target.
- **Load-bearing assumption:** Target altitude is constant and equals the interceptor's reference.
- **Why it matters:** Vertical evasion is the cheapest possible counter to a planar interceptor, and every Pk curve is conditioned on a geometry the scenario enforces rather than the vehicle earns. Within sim-portfolio scope it is a disclosed limit; for any deployment-facing claim it ranks with the criticals. Unlike most gaps, there is no bench fix — it requires a guidance design change (a vertical pro-nav channel).
- **Masked by sim:** Yes, by scenario construction.
- **Cheapest exposure:** One-line mover change ($0): ±1–2 m/s sinusoidal altitude schedule, 8 flights with the current planar law, report 3D miss. This directly sizes the vertical-channel work.

**G6. All maneuvering results ride the MOCK cue — HIGH**
- **What:** The 14/14 headline and everything in ADR-0056–0058 used the mock cue (white Gaussian noise + latency at 10 Hz). The real stereo pipeline has only been validated on straight-line trajectories. A maneuvering target produces *correlated* cue errors the mock cannot: aspect-dependent centroid shift, detector lag on reversals, gimbal tracking lag. Correlated bias is exactly the error class that broke covariance fusion (7.66 m bias-lock tail, ADR-0044) — and the cue now arms the handoff gate and seeds the tracker.
- **Load-bearing assumption:** Real stereo cue error on a maneuvering target is no worse-structured than white noise + latency.
- **Why it matters:** Mock whiteness is now a load-bearing assumption of the gating architecture, not a realism nicety.
- **Masked by sim:** Yes.
- **Cheapest exposure:** Render maneuvering-trajectory detection caches and replay through the existing station.py live-triangulation path (infrastructure exists; only caches are missing); measure cue-error autocorrelation and gate false-accept/false-reject rates; re-run the headline arm with `--cue-source stereo`.

**G7. End-to-end sensing-to-command latency is unbudgeted against a 0.41 s window — HIGH**
- **What:** Sim latency is effectively zero (desktop GPU, lossless localhost, lockstep). The real chain — exposure + Pi-class inference (~29 ms claimed for Hailo, unmeasured) + CSRT (tens of ms per frame on Pi-class CPUs, plausibly exceeding the 71 ms frame budget *by itself* — and CSRT timing is currently absent from the Stage-0 bench plan) + UART + PX4 loops — plausibly stacks 100–200 ms, i.e. 25–50% of the median terminal window.
- **Load-bearing assumption:** Pipeline latency is small vs. the terminal window and 20 Hz control period.
- **Why it matters:** Latency is kinematically equivalent to handing off with extra ZEM (zero-effort miss — the miss you'd get if nobody corrected from here on), the exact quantity ADR-0023 proved locks the miss. And it is a bias, not noise — the filter cannot average it away.
- **Masked by sim:** Yes.
- **Cheapest exposure:** Sim: measurement-delay tiers (80/150/250 ms sim-time), paired n=8, reusing the cue-mock latency machinery. Bench: add CSRT loop timing explicitly to the Stage-0 Pi measurement, and the ~$15 LED-to-logic-analyzer photon-to-command test.

### Tier 2 — real, cheaper to carry, still required before hardware claims

**G8. Gate generality beyond the empty world (12 m REACQ cap + 8 m SEED gate + multi-target) — HIGH** *(F6 folded in)*
- **What:** The 12 m re-acquire cap defeats phantoms ~18 m off because that is where *this* detector hallucinates in *this* empty world. The offset distribution is an empirical accident of detector weights × background × target appearance. The pre-handoff SEED gate (8 m) has the same empty-world-only history. And a *second real drone* — a decoy, or a wingman in the parent salvo concept — inside the gate radius defeats the anti-phantom logic by being real (gap F6, design-never-flown): the gates test position consistency, not identity.
- **Load-bearing assumption:** False and confusing detections stay farther from the truth than the gate radii, in every environment.
- **Masked by sim:** Yes — n=16, one world, no clutter, no second target.
- **Cheapest exposure:** Drop 3–4 clutter models plus a static decoy drone into the world, re-fly weave-12 n=8, histogram detection offsets against *both* radii. No code changes, one evening of batch time.

**G9. Airframe transfer of the kinematic budget + wind — HIGH** *(corrected for direction; wind added)*
- **What:** The x500 model has zero aerodynamic drag (no LiftDrag plugin — verified by grep), ideal actuators, no battery sag, and **no wind exists anywhere in the sim** (wind is absent even from the project's own 26-gap table — newly surfaced here). The 16 m/s dash, the 60° tilt cap (already above the derived ~53° thrust ceiling), and the 0.72 m correction capacity are properties of this fictional airframe. Correction from review: the direction is *ambiguous*, not one-sided — the parent 2.5-inch airframe at T/W 4.6–7 with claimed 8–10 g momentary capability may have *more* lateral authority than the sim x500, making 0.72 m conservative; the genuinely flattering pieces are drag at dash speed, actuator lag, sag, and gusts. A 5 m/s gust in the last second is several times the correction capacity, and the deliberate terminal freeze means late gust displacement goes uncorrected *by design*.
- **Load-bearing assumption:** Sim authority brackets real authority, and calm air.
- **Masked by sim:** Yes.
- **Cheapest exposure:** Sim: drag + first-order motor lag (or derate *and uprate* arms), re-run the ADR-0023 ZEM regression; Gazebo wind-plugin tiers on the paired-seed weave batch — with the caveat that the zero-drag model may not respond to wind realistically, so a drag model is a prerequisite for the wind sweep to mean anything. Bench: thrust-stand measurement (also retires the explicitly-pending 170 g/motor sizing number).

**G10. Own-state degradation — the DYNAMIC component only — HIGH, downgraded from critical** *(mechanism corrected)*
- **What:** The inertial LOS is built as λ = ψ + β (EKF yaw plus camera bearing) and pro-nav differentiates it. Correction from review: a *constant* magnetometer yaw bias adds a constant to λ and **differentiates out of λ̇** — pure pro-nav is rate-based, so static bias mis-points the camera-implied NED gates, seed validation, and breakoff geometry, but not the guidance derivative. The guidance-channel threats are yaw **jitter, latency, and maneuver transients** (which N·Vc amplifies up to 5×9 into commanded acceleration). No own-state degradation knob exists — verified by grep, all worse-than-ideal tiers are target-side.
- **Load-bearing assumption:** Own-state yaw is fresh and smooth enough that λ̇ noise is camera-dominated.
- **Masked by sim:** Yes — lockstep yaw is near-perfect and near-instant.
- **Cheapest exposure:** One afternoon of software: an own-state knob applied to the guidance-side ψ only (constant-bias tier separately — it hits the gates, not λ̇; jitter σ≈1° + 50 ms latency tiers hit the law), paired weave n=8 per tier; extend `--bench` to run under it.

### Tier 3 — carry with named tests; do not let them silently port

**G11. CSRT drift between validations — MEDIUM, downgraded** *(mechanism corrected)*
- Verified in code: the every-8-frames NN re-validation *does* re-seed post-handoff at IoU ≥ 0.2 (detect_track.py:406), so gradual drift is corrected; only a discontinuous jump onto background (IoU < 0.2) loses its correction channel, with the degenerate-box guard as partial backstop. Still worth watching: 8 frames is 0.57 s ≈ 7–9 m of closure at terminal speeds, and clean renders mean CSRT has essentially never been observed drifting. **Adopt regardless (free, honesty-legal):** log per-frame IoU between the CSRT box and the ground-truth-projected box in every batch (gt is scoring-only, so this is legal), then one clutter/occluder world variant.

**G12. Frame conventions at the sim/real seam — MEDIUM**
- The north=world_y/east=world_x mapping is a sim-side construct (real NED comes from EKF2/geodesy natively), so the risk is not literal carry-over but *re-derivation under port-time pressure* of ground-station frames and the λ = ψ + β sign convention. `--bench` covers one channel. **Fix:** extend `--bench` into a 4-quadrant static-geometry self-test (target at known N/E/S/W offsets; assert commanded-velocity direction and camera-implied NED for each) — runs in sim today, on the bench in minutes, makes the convention falsifiable instead of documented.

**G13. Cadence-shaped filter gains + the abort branch — MEDIUM**
- Alpha-beta gains (a simple two-state position/velocity tracking filter) were tuned at Gazebo's ~14 Hz cadence — the same cadence artifact that made Kalata gains degenerate (ADR-0013) is direct evidence the tuning is cadence-shaped. The `r_hat ≥ TERMINAL_RANGE → 5 s abort` branch is flight code that already misclassified 6/8 healthy flights once (ADR-0037; fixed analyzer-side only). **Fix:** frame-drop cadence sweep (5/8/20 Hz + jitter), paired n=8; plus a unit test feeding the abort branch a physically-correct growing post-CPA range trace.

**G14. Statistics too thin for the 95% Pk goal — MEDIUM, act before resume language**
- 16/16 pooled → 95% Clopper-Pearson lower bound 79.4% (0.025^(1/16); a Clopper-Pearson bound is the standard "worst case consistent with the data" interval for pass/fail trials). The maneuvering noise floor is ~5 m (an n=2 win was already retracted once, ADR-0057), and the jink numbers in the headline are pre-fix; the adopted `--track --handoff-cue-gate` config has only been batch-validated on weave. **Fix:** weave arm to n=48 (~3–4 h at ~70 s/flight), jink r2 arm n=16 on post-fix code. Pure batch time.

**G15. Physical basis of the Pk conversion — MINOR, disclose loudly** *(added missing item)*
- Kill radii are narrative assumptions (disclosed, ADR-0025) with no collision-volume or fuze/net model; nothing stress-tests the miss-to-Pk mapping the headline metric rests on. Keep the disclosure prominent in portfolio material; a sensitivity note (Pk vs. assumed radius) is a $0 analyzer addition.

---

## 3. Real-world variables that will interfere

Every variable maps to a bench-measurable quantity, per the builder's standing mandate. Corrections applied: time sync raised to critical; yaw-bias and boresight mechanisms fixed (constant bias differentiates out of the rate law — only the time-varying component threatens guidance); prop wash downgraded to low; five missing variables added.

### Critical and high tier (ranked)

| # | Variable | Category | Sim treatment | Real-world effect | Severity | Bench-measurable quantity | Cheapest experiment |
|---|---|---|---|---|---|---|---|
| 1 | Real-clutter false positives (birds, trees, poles) | optical | omitted | Failure class already proven in-sim (ADR-0056 phantoms, inverted confidence); the 12 m REACQ cap is tuned to one phantom population; real clutter yields a different, possibly nearer, distribution | **critical** | FP/hour and phantom-offset distribution vs. the 12 m/8 m gate radii; confidence histograms phantom vs. real | $0 today: exact detector offline on real sky/treeline footage; later Stage-0 camera captures |
| 2 | Terminal motion blur | optical | omitted | 32–58 deg/s LOS rates smear the target exactly in the 2–3.5 m freeze window; every sim detection number is an upper bound | **critical** | Detection % and bearing error (deg RMS) vs. slew rate; knee rate where detection <90%; blur px = ω·t_exp·f_px | $0 synthetic-blur replay now; Stage-0 yaw-ramp bench (~$15 jig, in cart) |
| 3 | Sustained detection Hz + thermal throttling on Pi-class compute | compute | idealized (desktop stands in) | All timing constants assume ~14 Hz never measured on target hardware; 6–8 Hz post-soak trips dropout-holds and starves the handoff streak | **critical** | Sustained Hz over 10 min with SoC temp; derate % hot vs. cold | Stage-0 as scoped (~$255 cart, already queued) — but bench the compute that will actually fly (see §4, flight-compute decision) |
| 4 | Exposure / dynamic range / sky contrast / AE hunting | optical | omitted (fixed ideal light, emissive boost) | Dark quad vs. bright sky silhouettes or saturates; auto-exposure hunts for 100s of ms on ground→sky pitch at the handoff moment; longer AE exposure multiplies blur. (Sun-angle "~30°" figure is illustrative, unsourced) | **critical** | Detection % vs. scene contrast ratio and sun angle; AE settle time (ms) after a 2-decade step; min contrast at 10 m | Outdoor Stage-0 captures + lamp-step AE timing; ~$20 lux meter (shared with #16) |
| 5 | End-to-end photon-to-command latency | compute | idealized (~zero) | 80–200 ms plausible (exposure + inference ~29 ms claimed/unmeasured + CSRT + UART + PX4); at 58 deg/s that is a 5–12° stale bearing — a bias the filter cannot average, equivalent to extra ZEM at handoff | **critical** | Photon-to-detection and photon-to-setpoint latency (ms, p50/p95), measured not summed | ~$15 GPIO LED in-frame + logic analyzer on Stage-0 rig; add CSRT loop timing explicitly |
| 6 | Ground-to-drone time synchronization | navigation | partial (epoch bug fixed by software construction) | **Raised to critical** (project's own T1 rating, ADR-0052): 50 ms skew ≈ 0.8 m in the PIP lead solve and 8 m gates at 16 m/s; silent systematic bias, invisible in any single log | **critical** | Residual clock offset (ms) and drift (ppm) under the chosen sync scheme | $0: two Pis logging NTP offset over the real link; upgrade path GPS-PPS (~$60–230) |
| 7 | Cue datalink latency/jitter/dropout on a real radio | comms | partial (self-authored 0.12 s + Markov guess) | Jam-resistance is proven post-latch, but Pk rides the cue through DASH/handoff (the #1 lever); 100 ms unmodeled jitter = 1.6 m lead error; degraded-but-alive is an untested regime distinct from clean-or-dead | **critical** | One-way latency (p50/p95) and loss % of the ~90-byte 10 Hz message vs. range and interference | ~$60–80 SiK radio pair, walk-out test + deliberate interferer |
| 8 | False-NEGATIVE NN transfer / real acquisition range *(added)* | optical | omitted (trained on one rendered body) | Acquisition range is THE Pk lever (ADR-0023); markerless already acquires terminal-only in-sim; real detection range vs. a real quad is unmeasured (T24 unwritten) | **critical** | Detection % vs. taped range and aspect against a real quad body | $0 desktop eval on real footage; Stage-1 taped-range sweeps |
| 9 | Own-ship GNSS/EKF position error | navigation | idealized (gap G2) | Budgeting cue datum error but not own GPS error implicitly halves registration error; 2–5 m multipath eats most of the 8 m gate margin — degrading the adopted anti-phantom defense re-opens the phantom hole | **high (borderline critical)** | Static CEP (m) and velocity RMS over 30 min, open-sky vs. near-structure; EKF2 innovations in flight logs | $0 with BOM parts on a surveyed point |
| 10 | Own-state heading error — dynamic component *(mechanism corrected)* | navigation | idealized (lockstep) | Constant mag bias differentiates out of λ̇ (rate law) — it mis-points the gates, not the law; the threats are throttle-dependent mag shift, in-maneuver EKF transients, jitter and latency, amplified ×N into a_cmd | **high** | Static bias (deg) vs. surveyed heading; dynamic error (deg RMS) in yaw ramps; mag shift motors-off vs. bench full throttle | ~$0–30, BOM H743 on a rotating jig + motor running alongside |
| 11 | Airframe vibration → image jello + IMU corruption | mechanical | omitted (gap I2, rated "major") | Attacks BOTH terms of λ = ψ + β at once (image bearing and EKF attitude via IMU aliasing); routinely the dominant image-quality problem on 2.5-inch frames | **high (borderline critical)** | Accel PSD at camera mount vs. throttle; Laplacian-variance sharpness vs. throttle; bearing scatter (deg RMS) vs. throttle | ~$50: motor+prop on the real mount, clamped, Stage-0 camera on a fixed target |
| 12 | Wind, gusts, low-altitude turbulence *(absent from the project's own 26-gap table)* | aerodynamic | omitted | Gust in the freeze window is uncorrectable by design and commensurate with the whole 0.72 m budget; steady wind shifts the PIP solve and handoff geometry; the TARGET is also wind-blown (scripted mover is not) | **high** | Hover hold error (m RMS) and velocity tracking error vs. anemometer wind; sim: Pk delta vs. wind tier | $0 Gazebo wind-plugin paired-seed sweep — **prerequisite: add a drag model or the sweep is meaningless**; ~$25 anemometer later |
| 13 | Range-from-appearance calibration on a real target | optical | partial (single-body fixed-aspect, ADR-0055) | The entire terminal state machine (Vc, throttle band, freeze, breakoff, 8–10 m gates) keys on r_hat; 20% range error skews all of it; markerless-path-specific | **high** | Range error (% of true) vs. taped range 2–20 m at 3–4 yaw aspects | ~$0 on the Stage-1 range sweep with a real quad body |
| 14 | Battery voltage sag under dash/terminal load | power | omitted | 2S LiHV sags ~0.5–1.0 V/cell at dash current; T/W falls toward ~3 exactly at the endgame; stacks with the A2 tilt-ceiling gap | **high** | V(t) and thrust (g) at hover vs. dash current; usable mAh above cutoff | ~$30 electronic load, or ~$60 thrust stand (also retires the 170 g/motor sizing assumption) |
| 15 | Baro altitude error + the missing vertical channel | navigation | idealized | Real baro ±0.5–1 m at 0.5 m AGL is a ground strike; deeper issue is structural — no vertical guidance channel exists (fixed ALT_REF P-loop), so any target altitude offset is an unguided miss component | **high** | Baro bias/RMS vs. tape or lidar at 0.5/1/2 m AGL; ground-effect shift vs. AGL | $0 sim first: raise ALT_REF, inject baro noise, and run the G5 vertical-mover probe; ~$15 lidar reference later |
| 16 | Airframe dynamics stand-in mismatch beyond drag *(added)* | aerodynamic | idealized (x500 ~2 kg/500 mm stands in for 125 mm/149 g) | Inner-loop bandwidth, real lateral-g (8–10 g momentary claim unverified), velocity-setpoint abstraction vs. real attitude dynamics, momentum-into-terminal (gap A3: a thrust derate "structurally can't capture that a heavier craft is harder to stop"); the 0.72 m capacity number rests on sim authority | **high** | Thrust-stand g/motor; step-response lateral accel from flight logs vs. sim commanded profiles | ~$60 thrust stand now; step-response flight test airframe-gated |
| 17 | 3D / aspect-varying target motion *(added)* | environmental | omitted (movers set position only, planar) | Combines with #15: any vertical maneuver is a direct unguided miss; aspect change breaks both the box-width range channel and the fixed-aspect-trained detector; no bench fix exists — requires a guidance design change | **high** | Sim: 3D miss vs. target climb rate; detection/range error vs. commanded target attitude schedule | $0: mover altitude + attitude schedules (one-line changes), 8 flights each |
| 18 | Ground-rig angular bias (gimbal pointing, extrinsics drift, tripod shake) *(added)* | mechanical | omitted (WORST tier tested only a fixed 2.5 m datum translation) | Angle-proportional bias scales with range (50–160 m) into the cue that sets handoff geometry — a different error family from both noise and fixed datum shift; hobby-servo (STS3215) pointing repeatability is unmeasured | **high** | Gimbal pointing repeatability (deg) vs. surveyed target; overnight extrinsics drift (deg) | $0–25: surveyed-target repeatability test + overnight drift log on the rig tier |
| 19 | Camera pointing vs. body pitch at dash speed *(added)* | mechanical | omitted (zero-drag sim reaches speed with transient tilt) | A real quad at 16 m/s pitches near the ~53° ceiling, pointing a fixed forward camera well below the horizon exactly when handoff acquisition must happen; interacts with the 2–3 consecutive-detection handoff streak | **high** | Camera-axis elevation vs. flight phase (log in existing sim runs first); real pitch-at-speed from flight logs | $0 sim probe today (log camera-axis elevation vs. phase); real answer needs airframe or a mount-angle decision |

### Long tail (medium / low)

| Variable | Category | Sim treatment | Real-world effect | Severity | Bench quantity | Cheapest experiment |
|---|---|---|---|---|---|---|
| Rolling shutter skew | optical | omitted | ~30 ms readout at 58 deg/s skews ~1.7° across frame, corrupting width (range) and bearing; retired for the bench by ADR-0012, **but the parent per-round BOM still lists Cam Module 3 (rolling) — a silent re-entry at fielding; close with a superseding ADR now, $0** | medium | Row readout (µs/row); edge skew (px) at known slew | ~$10 blinking-LED/spinning-disc test, only if a rolling SKU survives |
| Lens distortion / intrinsics error | optical | idealized pinhole | Wide M12 barrel distortion biases β 1–3° at FOV edges — where a crosser sits at terminal; a bias, but fully closable by standard calibration | medium | Post-cal reprojection RMS (px); residual bearing error at 0/50/90% field height | ~$5 checkerboard + protractor check |
| Boresight misalignment + mount flex *(mechanism corrected)* | mechanical | omitted | Static offset adds a constant to β and differentiates out of the rate law (mis-points, doesn't gut it); the real content is throttle-dependent flex, indistinguishable in-flight from mag error in the same λ channel | medium | Boresight (deg) at zero vs. full bench throttle; delta = flex | $0 on the vibration bench |
| Low-light / illumination envelope | environmental | omitted | Detection range collapses with lux; AE compensates with exposure, multiplying blur; sets operational-hours claims, not usually single-intercept outcome. **Merge into one bench campaign with #4 — same rig, lux meter, mechanism** | medium | Detection range/rate vs. lux; min lux for 90% at 10 m at capped exposure | Dusk sweep, shared ~$20 lux meter |
| Endurance / energy budget | power | omitted | ~1.8 min hover endurance vs. ~21 s measured takeoff tax plus dash current well above hover: thinner than it sounds; little loiter margin for launch-on-detect cue confirmation | medium | Usable Wh above cutoff; hover and dash A → engagement time budget | $0 incremental on the sag test + a spreadsheet |
| Prop wash / target downwash at close pass *(downgraded)* | aerodynamic | omitted | ~0.1–0.3 s wash transit during a phase where guidance has already frozen; cm-scale displacement; the ram-radius case it affects is already conceded (Pk@0.5 ~9%) and the 2 m net radius is unaffected | **low** | Induced flow (m/s) at 0.5/1/2 m offsets under a hovering donor | ~$25 anemometer map, airframe-gated flyby later |
| Fixed focus / depth of field | optical | omitted | Hyperfocal of the ~1.6 mm lens is well under 1 m — very likely a non-issue; verify once and close | low | MTF50 or bearing error vs. range 0.5–10 m at fixed focus | $0, fold into the Stage-1 range sweep |
| Temperature extremes | environmental | omitted | Cold packs lose 20–30% deliverable current (worsens sag at dash); hot ambient worsens Pi throttling; a modifier on two listed items, not independent | low | Sag at dash current 0 °C vs. 25 °C; sustained Hz at 35 °C vs. 20 °C | $0: refrigerate the pack, rerun; warm-day soak |
| Rain / dust / lens contamination | environmental | omitted | Droplets at 16 m/s destroy detection; honest answer for this project class is a weather no-fly envelope — document and move on | low | Detection % misted vs. clean at 10 m | $0 spray-bottle test |
| ESC / electrical noise on camera & compute rails | mechanical | omitted | Frame drops / brownouts under burst load; fixable with LC filter + wiring discipline; measure once so a mid-flight frame-drop mystery isn't misdiagnosed as perception | low | 5 V ripple (mVpp) at full throttle; frame drops/min motors-on vs. off | $0 on the vibration bench with a scope |

---

## 4. Cost-effective implementation path

The staged ladder, corrected. Ordering philosophy is unchanged and matches the repo's ratified plan (docs/stage0_bench_plan.md, NEXT.md build-queue #1): **no BOM dollar fixes perception risk — only a measurement retires it** — so perception is bench-proven before any airframe money. Five corrections from adversarial review are applied: a $0 desk stage inserted before any purchase; Stage 0 scoped to what it honestly tests; the flight-compute contradiction surfaced as a decision; three BOM holes filled in Stage 2; and Stage 3's target-drone plan checked against an observability budget it currently fails.

### Priority 0 — $0 desk stage (start today, no purchase gates this)

Run offline on the existing desktop:
1. **Synthetic-blur replay:** existing sim frames + motion blur matched to logged LOS rate × 2/5/10 ms exposures, through the exact NN+CSRT chain → detection rate and bearing error vs. blur (gap G1).
2. **Real-footage eval:** ground_v2 and drone_finetuned_v2 against real-sky/treeline drone footage (public datasets or phone video) → first honest F2 false-positive rate and first T24 transfer probe (gaps G3b, G4). "First probe," not "validation" — T24's plan is explicitly unwritten (NEXT.md item 9), and expect poor transfer.

### Stage 0 — Desk perception bench (~$255)

**Goal (honestly scoped):** run the repo's **CPU pyapriltags AprilTag bench** (stage0_bench_plan.md §2.2) on a Pi 5 + global-shutter camera: sustained detection Hz (T4 for the AprilTag stack), motion blur on a measured yaw ramp (L2), lighting/AE behavior, real lens intrinsics. **Not claimed here (corrected):** this is not "the exact ADR-0058 pipeline" — that is YOLO+CSRT whose deployed air target is Pi 5 + Hailo-8, and the ground NN already ran ~0.85–1.25 s/frame on a *desktop* CPU (T19b); NN-at-rate on a bare Pi 5 CPU is not the deployed configuration. F2/T24 are Priority-0 desktop jobs, not bench outcomes.

| Part | Qty | Spec | Est. USD | Source note |
|---|---|---|---|---|
| Raspberry Pi 5 8GB | 1 | Quad A76, per stage0_bench_plan.md | 130 | **Corrected:** buy via official reseller (PiShop.us/rpilocator) at ~$130 — the repo's own Apr-2026 number; the ~$199 figure is Amazon-marketplace RAM-crisis markup, don't pay it. Trim: 4GB ~$85–100 (fine for pyapriltags per the repo plan, but forecloses the same-board Hailo/NN follow-on) |
| Arducam OV9281 USB global-shutter cam (B0332) | 1 | 1MP mono, 100–120 fps, M12 — global shutter required for an honest blur test | 66 | WebSearch 2026-07: ~EUR 57 Welectron / Amazon; repo plan ~$65 — consistent |
| Wide M12 lens (~1.6 mm, ~100° HFOV) | 1 | Matches the sim's 99.7° lens | 18 | Repo stage0 plan price; ESTIMATE |
| **Yaw-ramp jig (hobby servo + bracket)** | 1 | Repeatable angular-rate sweep — **moved into this cart (corrected): it enables Stage 0's own #1 claim and the repo plan flags building it as THE schedule risk** | 15 | ESTIMATE; an STS3215 ($20 class, parent BOM) can be bought early here |
| Pi 5 Active Cooler | 1 | Required for the sustained-Hz soak | 5 | Repo plan / official accessory; ESTIMATE |
| 27 W USB-C PSU (official) | 1 | | 14 | Repo plan; ESTIMATE |
| 32 GB microSD (A2) | 1 | | 9 | Repo plan; ESTIMATE |

**Stage subtotal: ~$257** (trimmable to ~$210 with the 4GB Pi).

### Flight-compute decision — $0, must precede Stage-2 money

**Corrected (internal contradiction surfaced):** the plan benches a Pi 5 but the Stage-2 BOM flies a Pi Zero 2 W — so the headline T4 Hz number would never transfer to the compute that actually flies, and a 149 g AUW airframe cannot lift the Pi 5 it benched. The parent design ran classical frame-differencing CV on the Zero; ADR-0016's air target is Pi 5 + Hailo-8. **Decide which algorithm on which board flies the terminal seeker, then bench THAT configuration on the Stage-0 rig.** If the answer trends Pi 5 + Hailo, adding a Hailo-8 (~$70–110, explicit decision) to the bench is the honest test; if Zero-2W + classical CV, bench that stack instead. This is a decision, not a line item — log it as an ADR.

### Stage 1 — Outdoor camera characterization (~$79)

**Goal:** Stage-0 rig on battery power outdoors: calibrate the real lens, sweep detection vs. range and tag size, run yaw ramps at the sim's 32–58 deg/s terminal rates, log false positives against real sky/birds/clutter.

**What it validates (relabeled, corrected):** L1–L4 lens gaps (real distortion/intrinsics; the blur-vs-rate curve feeds back as a sim realism knob), the **onboard monocular** range/bearing error and detection envelope (gap-table rows 3/4 — the TERM_RANGE_NOISE_FRAC knob), and real-sky false-positive data against the gate radii. **Two withdrawn claims:** this does *not* measure F3 (that is the ground rig's 60–160 m envelope on rig optics), and it cannot touch the c=4.45e-05 σ_R∝R² constant — that is the *stereo-triangulation* error model (2.0 m baseline, 16 mm lenses, ADR-0017/0053), physically unmeasurable with one mono wide-angle camera.

| Part | Qty | Est. USD | Source note |
|---|---|---|---|
| Foam-board AprilTag prints, 3 sizes | 3 | 12 | ESTIMATE (commodity) |
| Tripod + pan head | 1 | 25 | ESTIMATE; doubles as the ground-cue mount later |
| USB-C PD power bank (≥25 W) | 1 | 22 | ESTIMATE (commodity) |
| Checkerboard/ChArUco calibration target | 1 | 5 | Parent BOM test-gear line |
| *(Yaw-ramp jig — moved to Stage 0)* | — | — | — |

**Stage subtotal: ~$64** (was $79; jig moved up).

### Stage 2 — Cheap 2.5-inch PX4 airframe, static-target flight (~$530)

**Goal:** build one round of the parent BOM, bring up PX4 on the H743-MINI, bench-measure motor thrust, then fly the M3-equivalent: hover, offboard velocity setpoints, camera-logged approach to a static tag, with a hard RC kill channel built and tested *before anything autonomous* (matches the gaps doc's own flag to pull S2 forward as safety).

**What it honestly validates (corrected):** I1/I2 (real EKF own-state + vibration), S2 (the kill link that exists nowhere in sim), first real ELRS/MAVLink latency point (C1), the full perception-to-control loop on hardware, and the pending 170 g/motor thrust number. **Withdrawn claims:** hover + slow static approach does *not* exercise A1 drag (binds at 16–25 m/s dash) or A2 tilt saturation — those wait for fast Stage-3+ flight.

**Prerequisite:** Stage 1 shows detection survives real optics/blur at terminal LOS rates. If it doesn't, fix perception before spending this.

| Part | Qty | Est. USD | Source note |
|---|---|---|---|
| Matek H743-MINI flight controller | 1 | 70 | WebSearch 2026-07: $69.99 Banggood USA; budget ~$95 domestic (DefianceRC) if avoiding overseas lead time |
| **GPS/compass module (Matek M10Q class)** | 1 | 30 | **ADDED (corrected):** PX4 refuses OFFBOARD velocity setpoints and cannot RTL without a position source — the stage was unflyable outdoors as previously specced. ESTIMATE $25–35 |
| **Thrust scale / load-cell fixture** | 1 | 20 | **ADDED (corrected):** "bench-measure motor thrust" was a named goal with nothing in any cart able to measure it; retires the sizing math's pending 114–174 g bracket. ESTIMATE |
| **LiPo safety (charge bag + XT30 smoke-stopper)** | 1 | 25 | **ADDED (corrected):** first-build 2S program, new-to-hardware builder. ESTIMATE |
| Flywoo ROBO 1303 6000KV motors | 4 | 60 | WebSearch 2026-07: $14.99 ea, flywoo.net |
| 20×20 4-in-1 ESC ≥20 A | 1 | 40 | ESTIMATE $35–45; availability confirmed, no clean price |
| Happymodel EP1 ELRS RX | 1 | 15 | ESTIMATE at known street $13–17 (search returned an implausible EUR 124 bundle — not used) |
| RadioMaster Pocket ELRS TX | 1 | 78 | WebSearch 2026-07: $77.99 Rotor Riot; ~$65 some retailers |
| 18650 cells for the Pocket | 2 | 12 | ESTIMATE (commodity) |
| 3D-printed PLA+ frame + M2 hardware | 1 | 15 | Parent BOM; printing free at Bend Central Library |
| Gemfan 2540 props (2 sets of 4) | 8 | 12 | ESTIMATE; parent BOM ~$1.25/prop — consistent |
| GNB 2S 450 mAh LiHV | 4 | 36 | WebSearch 2026-07: $5.99–10.99 ea; $9 used |
| ToolkitRC M6 charger + 12 V supply | 1 | 58 | WebSearch 2026-07: M6 V2 ~$42.72 Banggood + $15 ESTIMATE DC brick |
| Flight seeker compute | 1 | 18–? | **Held pending the flight-compute decision** (Pi Zero 2 W $18 ESTIMATE vs. Pi 5+Hailo path) — do not buy until the ADR is logged |
| ~~Pi Camera Module 3~~ → reuse Stage-0 OV9281 as flight cam | — | 0 | **CORRECTED:** bench-static is not flight-static (vibration + yaw make rolling shutter worse in flight); the OV9281 USB board (~10–20 g) removes the shutter confound and saves $28 |
| Wiring, XT30 pigtails, heatshrink, straps | 1 | 12 | ESTIMATE (commodity) |

**Stage subtotal: ~$525–545** (was $454 before the BOM holes were filled and the CM3 dropped).

### Stage 3 — Moving-target intercept (~$65 now; target drone deferred pending a math check)

**Goal:** reproduce the M4/M5 arc in reality at low speed — zipline first, live drone target second.

**Corrections applied, both load-bearing:**
- **The whoop-with-tag concept likely fails a first-order observability budget.** A 65 mm whoop lifts ~5–10 g → at most a ~0.1 m tag; at w_px ≈ 540·size/range with the ~45 px detection floor (stage0_bench_plan.md §2.5), that is detectable only to ~1.0–1.2 m on the ~100° lens — *inside* TERMINAL_FREEZE range, so it can never drive acquisition or handoff. Run this math before spending ~$150 on the target drone; the fixes are a larger target (5-inch class carrying a 0.3 m board), the narrower 70° stock lens, or the markerless seeker. The **zipline with a 0.3–0.5 m tag board is the valid mover** — repeatable, cheap, crash-proof, 2–5 m/s.
- **"Bearing-only ground cue" cannot feed the system as-is.** The cue wire format is 3D position(+velocity); DASH's PIP solve and the 8 m SEED/handoff gates compare 3D positions — bearing alone provides no range. Cheap fix that keeps the claim honest: the zipline's surveyed line + camera bearing gives pseudo-range by bearing–line intersection — one small script, and then the handoff gates *can* be exercised for real.

**Prerequisite:** Stage 2 static approach with clean logs and a proven kill switch; FAA recreational/Part-107 rules and a legal field sorted (both aircraft are yours; no third-party overflight).

| Part | Qty | Est. USD | Source note |
|---|---|---|---|
| Zipline target rig (paracord, pulleys, 0.3–0.5 m tag board, stakes) | 1 | 20 | ESTIMATE (hardware-store commodity) |
| Interceptor spares (props, 1 motor, 2 more 2S packs) | 1 | 45 | ESTIMATE from parent BOM unit prices |
| 1S whoop target (e.g. Meteor65 ELRS BNF) + batteries | 1 | ~150 | **DEFERRED pending the observability check above.** ESTIMATE $110–130 base + $30 consumables; search only surfaced the $286 Pro HD variant, not used |

**Stage subtotal: ~$65 now, ~$215 if the target-drone path survives the math.**

### Totals and the cheapest-first recommendation

- **Grand total, honest envelope: ~$1,050–1,250 all stages** (the prior $950–1,150 arithmetic was verified but predated the Stage-2 BOM holes; corrected for +GPS/thrust/safety, −Pi overprice, −CM3). Entry point: **$0 today** (Priority-0 desk evals), then **~$257 Stage-0 cart**.
- **Single cheapest-first purchase: the corrected Stage-0 cart (~$257)** — Pi 5 8GB at official-reseller pricing, OV9281 global-shutter camera, wide lens, and the yaw-ramp jig moved in. It is the repo's own ratified next action, and it retires the three measurements everything downstream assumes: real blur behavior at terminal LOS rates, real sustained detection Hz on embedded compute, real optics. If the seeker fails on this bench, no amount of airframe spending (~$530+) would have saved it.
- **Deferred, correctly:** the $1,160 stereo rig, Jetson-class compute, and RF/red-team work. The plan's biggest structural call — defer the stereo rig until a cheaper cue proves insufficient — survives review intact.
- **Disclosed limit:** no stage through Stage 3 measures real radio dropout statistics or range (C2) — Stage 2 yields an ELRS latency point only. The jam-story *transport* remains unmeasured until the ~$60–80 SiK-pair test is run; say so wherever the jam story is claimed.

---

## 5. The top 5 things to tackle first

Ranked by de-risk-per-dollar, weaving gaps × variables × experiments.

**1. The $0 desk trio — run this week, no purchases, no flights.**
(a) Synthetic-blur replay of existing sim frames through the exact NN+CSRT chain (gap G1 / variable #2 — the "existential" gap gets its first quantitative curve, and settles whether CSRT degrades faster than the NN). (b) Offline eval of ground_v2 + drone_finetuned_v2 on real-sky drone footage (gaps G3b/G4, variables #1/#8 — first honest false-positive rate against the 12 m/8 m gate radii, first NN-transfer probe). (c) Log camera-axis elevation vs. flight phase in existing runs (variable #19). **Cost: $0. Type: desktop/sim.**

**2. The cue-trustworthiness sim campaign — the corrected comms-denied test.**
Add `--cue-kill-range` (cue packets stop at randomized pre-acquisition ranges) and re-fly weave-12 n=16, measuring **intercepts completed under pre-acquisition denial** — the corrected fail-closed metric, not phantom count. Add clock-skew tiers on cue timestamps (T1, raised to critical). Render maneuvering detection caches and replay through station.py live triangulation to test the real stereo cue's error structure where it now gates (gaps G3/G6, variables #6/#7/#18). **Cost: $0, ~2 evenings of batch time. Type: sim.** This is the test that decides whether the headline jam-resistance claim needs a coast-through-denial design change before it is ever spoken to an interviewer.

**3. The r_hat honesty campaign — aspect, bias, and the freeze.**
Mover yaw schedule (±90° through the crossing) + T17 range-honesty regression + weave n=8; range-bias injection at 0.7×/1.3× fixed and aspect-correlated, paired n=8 per case, watching the freeze/throttle/handoff consumers (gap G2, variable #13). Piggyback the vertical probe: ±1–2 m/s mover altitude schedule, 8 flights, report 3D miss (gap G5, variables #15/#17 — sizes the vertical-channel design work, which no bench can fix). **Cost: $0. Type: sim.**

**4. Order the corrected Stage-0 cart (~$257) and log the flight-compute ADR.**
The bench closes what desk work cannot: real blur on a measured yaw ramp, real sustained Hz with thermal soak, real intrinsics, AE behavior, and photon-to-command latency (+$15 LED/logic-analyzer; add CSRT loop timing explicitly — it is missing from the current bench plan). Before Stage-2 money, decide which compute+algorithm actually flies (Zero-2W classical CV vs. Pi 5+Hailo) so the bench tests the deployed configuration (gaps G1/G7, variables #2/#3/#4/#5). **Cost: ~$257 + $15. Type: bench/hardware.**

**5. Buy the statistics and harden the gates — before any resume-level Pk language.**
Weave headline arm to n=48 (~3–4 h), jink r2 arm n=16 on post-fix code (gap G14); clutter/decoy world variant with offset histograms against both the 12 m and 8 m radii, including a second-drone confusion case (gap G8); adopt the free gt-IoU drift logging in every batch (gap G11) and the 4-quadrant frame self-test in `--bench` (gap G12). **Cost: $0, batch time. Type: sim.** Cheap insurance that the portfolio's numbers survive a skeptical reader — which is the audience that matters.

---

## 6. How this maps to the existing roadmap

**Deployment phases M-1..M-4 (parked arc, ratified design brief).** This review strengthens rather than replaces that arc. M-1 (ground standby / launch-on-detect) inherits the endurance finding (thin loiter margin — variable "endurance," medium) and the cue-trustworthiness campaign (Action 2) as prerequisites. M-3's climb-out, already flagged "genuinely open" in the brief, connects directly to the promoted planar-guidance gap (G5): the vertical-channel probe (one-line mover change) should run *before* M-3 design work so the vertical guidance channel is sized by data. M-4 (max-speed vision) is exactly where the newly surfaced camera-pitch-at-dash coupling (variable #19) and the drag/wind prerequisites (G9) bind — note the corrected sequencing: add a drag model before the wind sweep, or the sweep is meaningless.

**T23 gap audit (the project's own 26-gap table).** Largely vindicated: this review's two folded-in "missing" criticals — T1 time sync and F2 real-background false positives — were already rated critical *there*; the review's contribution is re-ranking them into the cue-trustworthiness cluster (G3) now that ADR-0058 made the cue a gating truth reference, plus the corrected fail-closed jam mechanism, which is new. Already covered by the audit and confirmed here: L2 blur, T4 compute, C1/C2 link realism, A1–A3 airframe, I1/I2 own-state/vibration, G1/G2 GNSS tiers, S2 kill link. **Newly surfaced by this review (not in the 26-gap table or any ADR):** wind and gusts; the camera-pitch-at-dash coupling; CSRT loop timing absent from the Stage-0 bench plan; the rolling-shutter Cam-Module-3 re-entry path in the parent BOM (close with a $0 superseding ADR); the Stage-2 BOM holes (GPS/compass, thrust rig, LiPo safety); the flight-compute contradiction (bench Pi 5, fly Pi Zero 2 W); the Stage-3 tag-observability failure; and the free gt-IoU drift instrumentation.

**Stage-0 parts (NEXT.md build-queue #1, BUILDER action).** The already-queued cart survives review with three amendments: buy the Pi 5 at official-reseller pricing (~$130, not the ~$199 marketplace channel — the cart lands at ~$257, close to the repo's own ~$240 figure); move the $15 yaw-ramp jig into this cart because it enables the bench's own #1 measurement and the repo plan names building it as the schedule risk; and scope the bench claims honestly — it tests the pyapriltags stack, blur, lighting, and intrinsics, while F2/T24 are $0 desktop jobs available *before* the cart even ships. The one Stage-0-adjacent decision that must precede any airframe money is the flight-compute ADR (Action 4).

**Bottom line for the portfolio narrative.** The defensible claims today: pro-nav visual intercept validated in PX4/Gazebo SITL with Monte-Carlo miss-distance analysis; a markerless seeker with a verified anti-phantom architecture (16/16 observed, 95% lower bound ~79%); a kinematic diagnosis that makes the design's sensitivities quantitative. The claims to hold until the tests above run: "Pk ≥ 95%," "works comms-denied" (currently fails closed under pre-acquisition jam), and anything implying the numbers transfer to real optics, real compute, real wind, or a real airframe. The plan above buys each of those claims in order of cost — starting at zero dollars, today.