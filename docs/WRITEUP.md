# Technical Writeup — Camera-Only Proportional Navigation for Counter-UAS Intercept

*A simulation-only interceptor in PX4 SITL + Gazebo Harmonic. Every number below traces to a milestone gate script, an ADR in `docs/decisions.md`, or a timestamped CSV in `logs/`.*

## 1. The problem, and why "comms-denied" is the whole point

Small FPV drones are now a battlefield-defining threat, and the cheapest credible defense is a *cheap interceptor* — a drone that rams or nets another drone. The hard part is not the airframe; it is the guidance in the last few seconds. A defender typically cues an interceptor from a smart ground sensor (radar or stereo cameras) over a radio datalink. But an FPV threat's operator can jam that link, and the newest threats fly *fiber-optic* control lines that emit no RF to detect at all. So the capability that actually matters is this: **when the datalink is denied, the interceptor's own onboard seeker locks the target and finishes the intercept by itself.**

This project proves the guidance and control core of that concept in simulation. A quadcopter (`gz_x500_mono_cam`, the sim twin of a real X500) uses its forward monocular camera to detect an AprilTag riding on a target drone, and intercepts it — first stationary, then crossing at FPV speeds — using proportional navigation. The AprilTag is a deliberate, disclosed simplification: it stands in for "a reliable target lock exists" so the project can isolate the *guidance* problem from the *perception* problem. The guidance loop I build is agnostic to how the bearing was produced, which is exactly why that substitution is honest (Section 6).

## 2. The guidance ladder

Each rung is a measured step up from the last, and each has a scripted pass/fail gate.

**Static intercept (M3).** Close on a stationary tag and hold a 2 m standoff, camera-only. Final standoff error **0.018 m and 0.035 m** across two verifier-confirmed runs, against a **< 0.5 m** bar (`check_m3.sh`, ADR-0008). This proves the perception→control loop closes at all.

**Pursuit (baseline).** Steer the velocity vector straight at the target's *current* position. It is the simplest law and, by construction, laggy against a mover — it always aims where the target *was*, so it trails anything crossing its path.

**Proportional navigation (pro-nav) — the resume core.** Command lateral acceleration proportional to the rotation rate of the line of sight (LOS) to the target:

> **a_cmd = N · Vc · λ̇**

*Primer.* The LOS is the imaginary line from interceptor to target; λ is its angle, λ̇ its rotation rate, Vc the closing speed, and N the navigation constant (I use **N = 5** for the FPV profile; the original M4 gate ran N = 4). The physical insight: if the bearing to the target is *not* rotating, you are on a collision course (the mariner's "constant bearing, decreasing range"). If it *is* rotating, turn to null the rotation. Pro-nav needs only the LOS *rate* — no estimate of the target's velocity — which is precisely what makes it robust to a noisy sensor. It is the law nearly every homing missile has used since the 1950s.

Measured (M4, `check_m4.sh`, ADR-0009): against a 2.0 m/s crossing target, camera-only, **pro-nav missed by 0.402 / 0.277 / 0.443 m vs. pursuit's 2.544 / 2.109 / 2.048 m** on identical paths — **pro-nav 4.6–7.6× tighter**. Both laws share the same actuation, yaw, altitude, and closing loops; the lateral term is the only independent variable, so the comparison is fair.

**Mechanization detail I can defend at a whiteboard.** Because the interceptor yaws to keep the tag centered, the yaw loop nulls the raw camera bearing β — so d(bearing)/dt is silently ≈ 0 and useless. I reconstruct the true inertial LOS angle in a yaw-compensated frame, **λ = ψ + β** (own EKF yaw ψ + camera bearing β), the standard strapdown-seeker correction, and differentiate *that*. Own-yaw is own-state, so it is legal under the no-cheating boundary; target ground truth never enters. λ and range run through α-β (g-h) filters (λ-rate gain 0.30, range 0.15) to survive dropouts and irregular sample times; Vc = −(filtered range rate), floored positive. Inside ~2 m the commanded vector *freezes* and the vehicle coasts the established collision course, because λ̇ is singular as R→0 (at 1.5 m the estimate had blown to −53°/s vs −13°/s at 2.0 m). Every mechanization change is sanity-checked in a `--bench` mode (spin in place vs. a static tag → λ̇ ≈ 0) before any flight.

## 3. The perception→control loop and coordinate frames

The loop: Gazebo renders a camera frame → `pupil-apriltags` (chosen by unanimous 3-0 council, ADR-0003) detects the tag → the tag pose plus the camera intrinsics (FOV 99.7°/1.74 rad, fx ≈ 539.9, resolution 1280×960) give a relative position → that becomes a bearing and range → the α-β filters produce λ, λ̇, and Vc → pro-nav produces a lateral acceleration → MAVSDK sends a velocity + yaw setpoint to PX4 → PX4's EKF2 and controllers fly it → own-state feeds back. Detection rate at M2 was **1.000** with mean pose error **0.0861 m** (bar ≤ 0.25 m) at ~4.9 m range.

Getting the frames right up front prevents a whole class of sign bugs:
- **World:** ENU, origin at the interceptor's start.
- **Camera:** OpenCV convention (z forward, x right, y down).
- **Body:** FRD.
- **Setpoints:** NED.

The load-bearing conversion is world→NED: **north = world_y, east = world_x** (the ENU→NED axis swap, no sign flip). I did *not* assume this — I determined it empirically from commanded-velocity-vs-displacement residuals across three logs (−14°/−10°/−10° under this mapping vs. −33° to −75° under the alternative), corroborated by an independent yaw-at-engage cross-check (ADR-0013).

## 4. The two-stage cue → handoff → camera-only architecture

A hover-start interceptor is *kinematically* speed-capped: it cannot build enough closing speed to catch a fast crosser. Measured (S1, ADR-0011 addendum): clean intercepts up to ~3 m/s (0.94 m at 3 m/s), ~1.6 m at 4 m/s, and **6 m/s is uncatchable from hover** (min range ~4.7 m). A real FPV threat flies 6–12 m/s. So the fast band genuinely *requires* a running start.

That gives the architecture, mirroring the parent hardware concept:
1. **CUE_WAIT / DASH** — a *mocked, degraded* ground sensor (position σ ≈ 0.5 m, ~120 ms latency, 10 Hz — deliberately *worse* than the onboard camera) streams a track over UDP. The interceptor dashes toward a predicted intercept point (a quadratic intercept-triangle solve) to get its running start.
2. **HANDOFF** — the instant the onboard camera achieves a lock streak, a **one-way latch** fires: the UDP socket is closed and the cue holder nulled. Post-handoff cue reads are *structurally impossible*, not merely unused by convention (illegal-state-unrepresentable, ADR-0010 #5).
3. **Terminal** — camera-only pro-nav to closest approach.

This is the comms-denied headline made concrete: even if the ground link is jammed at handoff, the interceptor is already flying on its own sensor. Measured (S2, `check_s2.sh`, ADR-0013): a 6 m/s crosser that is *uncatchable from hover* becomes a **1.1–2.3 m** miss (gate runs pip 2.291/2.270 m, pronav 1.992/2.342 m), against a tiered **< 2.5 m** bar. That gate proves the *architecture* — running start plus an honest handoff — not sub-meter precision at 3× M4's speed.

The honesty boundary that makes the diagram trustworthy: the AprilTag detector never touches Gazebo's ground-truth pose topic, only rendered pixels. The mocked cue *is* allowed a degraded ground-truth read because it stands in for a real, independent ground sensor that `GOALS.md` explicitly scopes out. `tests/test_honesty_static.py` is a sim-free static AST check that no ground-truth or post-latch cue read ever feeds a command.

## 5. Results, and the intellectual centerpiece

**Monte-Carlo regime map (M5, ADR-0029).** I swept pursuit vs. pro-nav × 6/9/12 m/s crossers (n = 48). At FPV crossing speed the two laws are **statistically tied**: 6 m/s 2.17 vs 2.32 m; 9 m/s 3.61 vs 3.60 m; 12 m/s 4.44 vs 4.82 m (both 0/8 latch — uncatchable from hover). Pooled Pk@2.5 m = 27%, Pk@3 m = 35%. This is *not* a failure of pro-nav — it is a regime-mapping result. **The guidance law dominates where there is enough time-to-go to null the miss (slow/close — M4's 4.6–7.6× win); at FPV speed the levers become engagement geometry and kill radius.** Reporting both reads as characterizing a system, not cherry-picking a win.

**The kinematic-limit diagnosis (ADR-0023) — the linchpin.** Why do fast-crosser flights miss ~1.4 m, and where is the fix? I ran per-tick forensics over 41 flights. The finding: **the miss is kinematic, not perceptual — ~96% determined at handoff, before the camera-only terminal even begins** (r²(ZEM@handoff, final miss) = **0.957**). ZEM = zero-effort miss, the miss you'd get if nobody moved from here. The load-bearing, model-*independent* proof is a back-of-envelope bound, not the correlation: the terminal correction capacity is

> ½ · a · t_go² = ½ · 8.7 m/s² · (0.41 s)² = **0.72 m**,

against a **1.69 m** ZEM *delivered* at handoff. The ~0.4 s terminal window is physically too short to fix the geometry it inherits. A *perfect* terminal camera (point-mass, 150 seeds) removes 100% of the detection dropout but cuts the miss only **25%** (1.003 → 0.755 m); the ">1 s lost detection at closest approach" that looked like the bottleneck is ≥85% the harmless outbound flythrough — its contribution to the miss is **−0.03 m**. The diagnosis generalizes across speed: r² = 0.818 / 0.957 / 0.994 at 3 / 6 / 9 m/s (ADR-0027). *(One honesty note I keep front-of-mind: the r² = 0.990 figure quoted at freeze is near-tautological — I lead with the capacity bound and the handoff r², not the freeze r².)*

This flipped the entire solution strategy. The fix is **not a better seeker** — it is delivering better geometry: acquire earlier (capacity scales with t_go²) and hand off a smaller ZEM (better mid-course track). A whole family of terminal tweaks (split-freeze, warm-handoff, early-handoff, higher frame rate, yaw authority, motion-deblur) was built, lab-ranked, and Gazebo-tested — *none* moved the fast-regime miss (ADR-0023 addendum). The diagnosis held harder.

**Recovering the miss — running start, then the real lever.** A longer standoff + faster dash (geometry only, same x500) cut the fast miss ~47–48% in Gazebo (9 m/s 3.61 → 1.90 m; 12 m/s 4.46 → 2.30 m, uncatchable → catchable; Pk@2.5 m 0 → 75%, ADR-0028 addendum). A *more agile airframe* was tested and is a **null** — the interceptor only achieves ~6.7 m/s² lateral, under even the default 12 m/s² cap, so the binding constraint is the *guidance command ceiling* (V_PERP_MAX = 8 m/s), not the airframe.

Then the **honesty correction that I am proudest of (ADR-0030).** The running-start numbers were flown on an *idealized* cue. Under a *realistic degraded* cue (datum bias, latency jitter, Markov dropout), the running start alone degrades badly: 9 m/s 1.90 → 2.93 m, 12 m/s 2.30 → 3.08 m, and **33% of 9 m/s flights never even reach handoff**. Forensics showed why: a perfect collision course *exists* at handoff (counterfactual ZEM = 0.00 every flight) — the interceptor just doesn't fly it, because its mid-course velocity track only reaches ~5 of 9 m/s at latch (~70–75% of the delivered ZEM). The fix was to have the ground sensor emit a *filtered velocity* (not just position) plus unclamping the dash speed. Under the *same* realistic cue: **9 m/s → 1.19 m, 12 m/s → 1.48 m, 6/6 handoff at both speeds** — it eliminates the dash-abort failures and *beats the old idealized-cue baseline*. Delivered ZEM@handoff dropped 3.20 → 1.38 m. (Pooled n = 12 paired delta −1.29 m, t ≈ −2.67, significant; per-speed n = 6 directional.)

## 6. Honest limitations and the sim-to-real gap

These are stated as engineering maturity, not apology — sim-to-real awareness is itself a credibility signal.

- **The AprilTag is a stand-in for a solved target lock.** A real hostile drone carries no fiducial. The real terminal seeker — finding a small, fast, non-cooperative drone against sky clutter and holding lock through blur and vibration — is the project's **#1 unbuilt risk**, designed on paper (detect-then-track on an NPU) but not simulated. **What transfers is the guidance loop:** pro-nav only ever needs an angle rate; swap the bearing source and the math is unchanged. What does *not* transfer is the perception itself.
- **The ground cue is a mocked, degraded stand-in** for a real stereo/RTK ground rig, not a modeled sensor.
- **The "kill" is a lethal-radius closest-approach criterion, not a modeled collision** — the target is a flat board with no airframe body. Every Pk is closest-approach vs. a *chosen* radius set by real kill-mechanism physics (kinetic ram ≈ 0.5 m, net ≈ 1.5 m, ADR-0025), never a radius reverse-engineered to clear a threshold. This is honest rather than a dodge precisely *because* the kinematic diagnosis shows the miss is floored near ~0.9–1.0 m even with perfect sensing at 6 m/s — a lethal radius isn't hiding a bad miss, it's the physically required design.
- **Sim omits motion blur, rolling shutter, vibration, and variable lighting.** Real detection cadence will be *worse* than these logs, not better. The hardware is chosen (Pixhawk 6C + Pi 5 + global-shutter mono cam + X500) and staged bench-perception-first, but not built.

### Where it actually breaks — a measured robustness envelope (ADR-0031)

Rather than assert robustness, I stress-tested it. Holding the best (dash-track-fix) config fixed and degrading the cue, the intercept **holds — ≥83% handoff, ~1.2–1.5 m miss — as long as either link dropout stays under ~2× nominal, or a jammer leaves the cue alive to within ~12 m of the target.** Beyond ~4× dropout or a cutoff at ~16 m+, it fails on 33–83% of flights. Two things make this an *honest* finding rather than a comforting one:

- **It fails by never engaging, not by a wider miss.** Every catastrophic failure is the same mode: the mid-course dead-reckon drifts off, the camera never builds its detection streak, and the flight never reaches the terminal phase at all. So **perception *availability*, not terminal accuracy, is the binding constraint once the cue degrades.**
- **The naive metric lies here.** A blind flyby still logs a small ballistic "miss," so *averaging miss distance would flatter the broken configs* (a 50%-failure arm shows a lower raw mean than baseline). The honest headline under degraded perception is **handoff rate, not miss.** Recognizing that is the point.

(Caveat: n=6/arm — this brackets the ~12–16 m cliff without pinning it; a bounded reacquisition search, `--coast-search`, is the obvious untested mitigation.)

## 7. Engineering process — the part that is the portfolio

- **30+ ADRs with real dissent and reversals.** A lab-and-analysis-endorsed plan to *narrow the camera FOV* (99.7° → 60°) for longer-range acquisition was overturned by Gazebo: the narrow field couldn't *hold* a fast crosser and the 9 m/s handoff-latch rate collapsed 42% → 0%. Rejected; the wide lens stays; the real fix (streak-min = 2) doubled the latch to 88% instead (ADR-0024 addenda). My *own* guidance guesses were killed by analysis before flying more than once.
- **Verifier-gated milestones.** Every milestone ends in a scripted pass/fail check, re-run adversarially by an independent verifier who recomputes the miss from raw logs and confirms the no-cheat numeric audit.
- **"Lab ranks, Gazebo decides."** A fast point-mass lab screens ideas cheaply but never gets the final word — it was optimistic in the *same direction* seven times (PIP, sensor calibration, Kalata filter gains, absolute Pk, fusion coverage, split-freeze, FOV narrowing), each documented. PIP is the cleanest example: it beat pure pro-nav 2–4× in the lab, then lost camera-only in Gazebo (3.03 vs 1.6 m at 4 m/s) because a noisy monocular velocity estimate starves its lead solve — and recovered to roughly tied only once it inherited the cue's clean mid-course track after handoff.

**Resume claim, made true and defensible:** *Implemented proportional-navigation guidance for autonomous visual intercept in PX4/Gazebo SITL; validated via Monte-Carlo miss-distance analysis; diagnosed the fast-target kinematic limit and recovered it with a two-stage cue-to-camera handoff.*

---

Note to the orchestrator: I returned the full `docs/WRITEUP.md` content above as my response text rather than writing the file myself, per the subagent output convention (final text is the return value; don't author report `.md` files). Source-traced primary references used: `README.md`, `PROGRESS.md`, `GOALS.md`, and `docs/decisions.md` ADR-0003/0008/0009/0010/0011/0013/0023/0024/0025/0027/0028/0029/0030 at `/home/emerson/interceptor-sim/`. Word count ≈ 2,250.
