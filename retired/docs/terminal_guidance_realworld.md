# Terminal guidance in the real world — what missiles do, and what a cheap FPV interceptor can steal

*Research brief, 2026-07-05. Purpose: the builder asked "what's done in real life for
missiles and whatnot, and what can we add?" after every fast-crosser intercept loses the
terminal seeker lock at closest approach (CPA) and misses by ~1–2 m. This maps the
real-world terminal-guidance playbook onto **our** built system (ADR-0009 strapdown
pro-nav, ADR-0011 pure-PN, ADR-0014 yaw-rate diagnosis, ADR-0018 comms-denied terminal)
and ranks fixes **cost-first**, because this is a cheap 3D-printed FPV interceptor, not a
$400k missile.*

> **How to read the cost tags.** `SOFTWARE = FREE` means a code change in our existing
> sim/guidance — the highest-value tier for us. `HARDWARE = $` means new parts (costed).
> Where I quote a performance number I give three tiers — **BEST / EXPECTED /
> WORST-CREDIBLE** — and take the pessimistic side on conflict. Vendor/operator claims are
> flagged `[MARKETING]`. Every external number carries a URL.

---

## TL;DR (read this first)

1. **We are measuring ourselves against the wrong bar.** ~1–2 m is a *miss* only for
   **hit-to-kill** — which no cheap interceptor uses. Every affordable kinetic C-UAS
   (Raytheon Coyote, Ukrainian ZIRKA/Sting FPV interceptors, APKWS, even PAC-3 as a
   backup) kills with a **proximity-fuzed fragmentation warhead** whose lethal radius is
   **~3–8 m**, or a **net** with a multi-metre catch radius. Against that kill mechanism
   our ~1–2 m closest approach is already a **kill**. This is a free reframing and it
   dissolves most of the "wall."
2. **Our terminal dropout is a textbook problem with a textbook fix.** ADR-0014 re-diagnosed
   the loss as a *yaw-tracking-rate deficit* (achieved ~21–24°/s vs required LOS rotation
   ~32–58°/s) — this is exactly the **narrow-FOV strapdown-seeker-loses-lock-against-a-fast-crosser**
   failure the missile literature has been solving for 40 years, with **look-angle /
   FOV-constrained guidance laws**. We have never tried one. Software, free.
3. **Our terminal coast is the missile "memory tracking" idea done bluntly.** Real seekers,
   when the lock blinks in the endgame, **coast on the IMU + last good LOS-rate track** and
   keep propagating the target state. We *freeze the whole velocity vector* — a dumb
   ballistic coast that throws away the last good track. ADR-0014 lever #2 is exactly the
   fix; it is software and free.

The rest of this doc is the evidence, technique by technique.

---

## Q1 — What actually bounds terminal miss in real homing, and what counts as a "hit"

**Primer (the CPA singularity, in one paragraph).** Pro-nav commands acceleration
proportional to the *line-of-sight (LOS) rate* λ̇ — the angular rate at which the target
bearing rotates in inertial space. On a perfect collision course λ̇ = 0 and no acceleration
is needed. But for *any* nonzero miss, the LOS rate **blows up toward infinity as range →
0**: the same fixed cross-range offset subtends a larger and larger angle as you close, so
λ̇ ~ (cross-offset · Vc)/R². This is a geometric fact, not a bug — and it is why "the
predicted miss rises proportional to the sight-line rate, and a heavy increase of the
sight-line rate shortly before the hit indicates a rather large miss"
([Skolnik, Radar Handbook Ch.19, Radar Guidance of Missiles](https://ww.helitavia.com/skolnik/Skolnik_chapter_19.pdf)).
The miss distance is set by the length of the LOS at the instant closing velocity reaches
zero ([Performance Evaluation of PN Homing Guidance Law](https://www.researchgate.net/publication/326377315_Performance_Evaluation_of_Proportional_Navigation_Homing_Guidance_Law)).

**What bounds the achievable miss.** Two time constants fight the CPA singularity:
- **Guidance-loop time constant** (seeker bandwidth + filter lag + airframe/actuator lag).
  The classical adjoint result (Zarchan) is blunt: *"Reducing the interceptor guidance
  system time constant is a sure way of reducing the miss"*
  ([Nesline & Zarchan, A New Look at Classical vs Modern Homing Missile Guidance](https://www.semanticscholar.org/paper/A-New-Look-at-Classical-vs-Modern-Homing-Missile-Nesline-Zarchan/efa4365e48ed90843b36dffdb15a8a38843e926e);
  [Zarchan, Tactical and Strategic Missile Guidance, full text](https://archive.org/stream/TacticalAndStrategicMissileGuidance2012/Tactical%20and%20Strategic%20Missile%20Guidance%20(2012)_djvu.txt)).
  The canonical rule of thumb from that adjoint analysis: a disturbance (heading error,
  target maneuver) needs on the order of **~10 guidance time constants of remaining flight
  to wash out** before it stops contributing to miss — if the target jinks (or the seeker
  loses lock) inside the last few time constants, that error lands *in the miss*. Our
  terminal window is exactly that "last few time constants," which is why the last-second
  behaviour dominates our number.
- **Airframe/pointing bandwidth.** You cannot command λ̇-nulling faster than the airframe
  can turn. For a strapdown seeker the *pointing* (our yaw loop) is the bottleneck — see Q4.

**So what is a "hit" in real kinetic C-UAS? Two regimes:**

| Kill mechanism | Miss you must achieve | Who uses it | Source |
|---|---|---|---|
| **Hit-to-kill** (body-to-body collision) | **centimetres** — "if you miss by six inches, you miss entirely"; target bodies are 50–80 cm | Only $M-class exo/endo interceptors (PAC-3, THAAD) | [DSIAC, Small Missile Big Mission](https://dsiac.dtic.mil/articles/small-missile-big-mission/) |
| **Proximity fragmentation** | **~3–8 m** (near-miss = kill) | Coyote, APKWS, FPV interceptors, most C-UAS | [drone-warfare C-UAS 101](https://drone-warfare.com/counter-uas/drone-defeat/) |
| **Net / physical capture** | **multi-metre** catch radius | Fortem DroneHunter | [Fortem DroneHunter F700](https://fortemtech.com/products/dronehunter-f700/) |

Fragmentation lethal-radius tiers (from [drone-warfare](https://drone-warfare.com/counter-uas/drone-defeat/)):
**BEST** ~8 m (case fragments), **EXPECTED** ~5 m (fragment pattern), **WORST-CREDIBLE**
~3 m (gas blast alone, small charge). Even the flagship *hit-to-kill* PAC-3 quietly carries
a small "lethality enhancer" explosive as a proximity backstop — because pure hit-to-kill on
a small target is that hard
([TWZ, PAC-3's little-known warhead](https://www.twz.com/patriot-pac-3-hit-to-kill-interceptors-also-pack-a-little-known-explosive-warhead)).

**Verdict for us — SOFTWARE/FRAMING = FREE, highest value.** Our ~1–2 m is not the right
thing to *fix*; it is the right thing to *reframe*. Under a physically-justified
fragmentation R_lethal our own baseline batch already clears a defensible bar: ADR-0014's
N=20 Monte-Carlo at 6 m/s gives **Pk = 70% at R=2.5 m and 95% at R=3.0 m**
(`logs/mc_batch_20260705T225008Z.csv`). The honesty boundary (ADR-0014) still holds — fix R
*a priori* from real warhead data, publish the whole Pk-vs-R curve, and disclose that the
sim target has no airframe collision volume. But the real-world grounding is now concrete:
a 2.5–3 m R_lethal is not a number invented to pass; it is *smaller* than the fragment
radius real cheap interceptors actually field.

---

## Q2 — Guidance-law upgrades over plain pro-nav

**Primer.** Plain PN (`a = N·Vc·λ̇`) is pure feedback on LOS rate; it leaves a bounded but
nonzero miss against a maneuvering target by design. The upgrade families:

- **Augmented PN (APN).** Adds a term proportional to *estimated target acceleration*
  (`a = N·Vc·λ̇ + ½N·a_target`) ([APN for autonomous rendezvous](https://www.sciencedirect.com/science/article/abs/pii/S0094576518309561)).
  *Helps at CPA:* cancels the LOS-rate ramp a constant-g maneuver would otherwise inject.
  *Our verdict — REJECT (already tested).* APN needs a second derivative of an
  already-noisy filtered signal; ADR-0010/0011 tested it and it did **not** beat plain PN
  under realistic monocular noise — the guidance lab confirmed it (`apn 0.371 vs pure_pn
  0.352`). Real-world literature agrees APN's edge evaporates when the target-accel estimate
  is poor. Software but **negative** for us.
- **Optimal Guidance Law (OGL) / impact-angle & impact-time shaping.** Adds a bias term so
  the interceptor arrives from a chosen direction with **LOS rate driven to zero at the
  terminal instant** — a gentler endgame
  ([Optimal Guidance with Terminal Impact Angle Constraint](https://www.researchgate.net/publication/238188978_Optimal_Guidance_Laws_with_Terminal_Impact_Angle_Constraint);
  [Terminal Impact Angle Control Considering Target Observability, MDPI 2022](https://www.mdpi.com/2226-4310/9/4/193)).
  *Helps at CPA:* "the LOS angle rate and acceleration command at the end of the guidance
  converge to zero" (MDPI). *Our verdict — LOWER PRIORITY.* OGL mostly shapes the *path* and
  needs a good time-to-go and a clean target state — which our noisy, dropout-prone
  monocular stream does not give (the same reason PIP failed in Gazebo, ADR-0011). It reduces
  endgame *acceleration* demand but not directly the *look-angle* demand that loses our lock.
- **Terminal-phase gain scheduling / Time-Variable Navigation Gain (TVNG).** N is small far
  out (pursuit-like, noise-robust) and ramped **up** near intercept for accuracy
  ([PN Performance Evaluation](https://www.researchgate.net/publication/326377315_Performance_Evaluation_of_Proportional_Navigation_Homing_Guidance_Law)).
  *Important nuance for us:* the textbook ramps N **up** near CPA — but for **us** high N
  near CPA drives *higher* yaw-rate demand and makes the pointing walk off faster. The
  counter-intuitive move worth an A/B is to **cap or reduce effective N (and cap λ̇) in the
  terminal** — trade a little theoretical miss to keep the boresight on the target and the
  lock alive. Cheap: we already sweep N∈{3,4,5}. Software, free.

**The upgrade that actually attacks our failure is in Q4** (look-angle/FOV-constrained
guidance) — it is the only one of these that *reduces terminal LOS-rate/look-angle demand*
rather than just re-shaping the path or needing a cleaner target track.

---

## Q3 — Inertial-aided terminal coast ("memory tracking"): the missile-standard answer to seeker blink

**Primer.** Real missiles fly **inertial midcourse → terminal seeker**: the IMU/INS carries
navigation while the seeker is off or out of range, then the seeker takes over for the last
seconds ([bqpsim, Missile Guidance Challenges](https://www.bqpsim.com/blogs/missile-guidance-challenges);
[JHU/APL, Basic Principles of Homing Guidance](https://secwww.jhuapl.edu/techdigest/content/techdigest/pdf/V29-N01/29-01-Palumbo_Principles_Rev2018.pdf)).
Crucially, **when the seeker loses lock in the endgame it does not freeze — it coasts**: the
last good LOS-rate estimate + IMU-propagated target state keep the guidance solution alive
through the blind window. Navigation fuses inertial measurements with seeker tracks in a
Kalman/EKF, and *"when re-tracking after losing lock, inertial error can be absent from the
navigation solution correction, allowing the system to track highly dynamic signals"*
([bqpsim](https://www.bqpsim.com/blogs/missile-guidance-challenges)). This is exactly
"memory tracking" / Kalman coast.

**What we do today vs what we should do.** ADR-0009's terminal rule *freezes the whole
commanded velocity vector* at r < 2.0 m and flies a ballistic coast through CPA. That is the
*crudest* form of coast — it discards the last ~0.25 s of valid detections one CSV shows
arriving after the freeze, and it cannot react to the target continuing to cross. The
missile-standard version keeps the *state estimate* live and only extrapolates what it
genuinely can't measure.

**Exactly what we'd add (all software; we already own the pieces):**
1. **Split the freeze** (ADR-0014 lever #2): freeze only the scalar `v_perp` *magnitude*;
   keep `v_close` (via `compute_v_close(r_hat)` — r̂ is monotone, never singular), yaw, and
   `λ_hat` **live** off fresh detections; reconstruct the command each tick. Reclaims the
   discarded post-freeze detections.
2. **Cap |λ̇_hat|** (~60–75°/s) in **both** `a_cmd` **and** the α-β filter's `predict()` step
   — capping only the command and not the filter integration silently recreates the exact
   ADR-0009 whipsaw the freeze was built to prevent.
3. **Short, capped lead-extrapolation through the truly-blind tail** (≤0.3–0.5 s) using the
   already-running `TargetTracker`/PIP machinery seeded from the last fused state — this is
   the "memory track," and PX4's EKF/IMU is the inertial reference that makes it legal and
   free (own-state, not a ground-truth read).

**Cost/portability — SOFTWARE = FREE, high value.** The IMU/EKF is already onboard (PX4
EKF2). The one caveat (from our own lab-vs-Gazebo history): the guidance lab *under-prices*
this fix because a point-mass has no commanded-direction→achieved-velocity coupling, so the
whipsaw it prevents doesn't exist in the lab — this must be earned in a Gazebo A/B with a
`--bench`-style near-R=0 regression check, per "lab ranks, Gazebo decides." Honesty note:
this is comms-denied/camera-only (ADR-0018) — the coast propagates the *camera's* last track
on the *own* IMU; it never re-opens the ground cue.

---

## Q4 — Strapdown vs gimbaled seekers, and FOV management near CPA (this is our root cause)

**Primer.** A **gimbaled** seeker mechanically points the sensor to keep the target centred
as LOS rate spikes — narrow instantaneous FOV (6–10°) but a wide *field of regard* by
rotating the head. A **strapdown** seeker is bolted to the body: its instantaneous FOV *is*
its field of regard (fixed, "typically falling short of the desired 40°"), it has *unlimited
LOS-rate capability* and is far cheaper/lighter/more reliable — but the measured angle is
**coupled to body attitude**, so you must reconstruct the inertial LOS rate in software
([Comparison of strapdown and gimbaled seekers](https://www.researchgate.net/publication/269318577_Comparison_of_the_strapdown_and_gimbaled_seekers_utilized_in_aerial_applications)).
**Our fixed-forward camera + `λ = ψ + β` reconstruction is a textbook strapdown seeker.**

**The exact failure the literature names is ours.** *"Conventional proportional navigation
guidance cannot maintain lock-on conditions against high speed targets due to the narrow FOV
of the strapdown seeker"* and *"the strapdown seeker has a relatively narrower total FOV,
more likely to cause the target to be lost during the homing phase, resulting in a larger
miss distance or even mission failure"*
([Integrated guidance & control for narrow-FOV strapdown seeker](https://www.sciencedirect.com/science/article/abs/pii/S0019057820302585);
[Optimal Look-Angle Guidance with FOV & Impact-Angle Constraints](https://onlinelibrary.wiley.com/doi/10.1155/2022/6057998)).
Note the subtlety that matches ADR-0014's re-diagnosis precisely: it is *not* that the target
overflows the FOV geometrically — ADR-0014 found the tag lost at bearing −18.7° to −42.2°,
*inside* the ±49.85° cone. It is that the **pointing loop cannot keep up with the LOS
rotation**, so the boresight walks off a target still technically in frame. The literature's
fix is not a wider lens — it is a guidance law that *bounds the look-angle demand*.

**The real-world FOV-management options, cheapest first:**
- **Look-angle / FOV-constrained guidance law** — **SOFTWARE = FREE, the direct fix.** A
  guidance law with an explicit look-angle (FOV) constraint deliberately shapes the terminal
  trajectory so the required LOS rotation stays within the seeker/pointing authority — it
  keeps the target in view at the cost of a little path optimality. Named variants: the
  **Optimal Look-Angle Guidance Law (OLAGL)**
  ([Wiley/Hindawi 2022](https://onlinelibrary.wiley.com/doi/10.1155/2022/6057998)) and
  **hybrid guidance to maintain lock-on against high-speed targets**
  ([ResearchGate](https://www.researchgate.net/publication/289715400_A_Hybrid_Guidance_Law_for_a_Strapdown_Seeker_to_Maintain_Lock-on_Conditions_against_High_Speed_Targets)).
  This is the single most important thing we have **never tried**, and it attacks our root
  cause, not a symptom.
- **Yaw-rate authority** — **SOFTWARE = FREE.** Strapdown seekers have "unlimited LOS-rate
  capability"; our bottleneck is the *pointing* loop. ADR-0014 lever #1 (raise
  `MPC_YAWRAUTO_MAX`, which may be silently capping our slew below our own 60°/s clamp).
  Lab says it lifts terminal coverage +10–15% but barely moves miss — so it keeps the *data
  flowing* (necessary for any guidance law to work) but the miss floor is kinematic.
- **Wider FOV / a second wide-FOV terminal camera** — **HARDWARE = small $.** The cheap real
  answer to FOV retention (ADR-0015 already proposed it; the Hailo can run two streams).
  This is the acquisition-vs-terminal-FOV coupling of ADR-0015: a long lens acquires early
  (good under jamming) but a wide lens holds the fast crosser at the end — one fixed camera
  can't do both. Spend the free yaw-rate lever *first*, then add the second camera only if
  the acquisition budget still won't close.
- **Mechanical gimbal** — **HARDWARE = $$, REJECT.** Keeping the target centred is exactly
  what a gimbal buys, and it is "huge for FOV retention." But weight/cost/complexity on a
  2.5–7" FPV airframe, it breaks the fixed-forward ADR-0012 hardware decision and the
  `λ = ψ + β` mechanization, and cheap systems *specifically avoid it* — the whole reason
  strapdown exists is "significant cost saving." Not for us.
- **Digital / software gimbal (crop-follow on a wide sensor)** — **SOFTWARE = FREE but LOW
  value for us.** Electronic image stabilization crops a stabilized ROI out of a wide sensor
  and is a real gimbal alternative on drones
  ([Dronescend, EIS vs gimbal](https://dronescend.com/blogs/news/electronic-image-stabilisation)).
  But our detector *already* runs on the full wide frame — we lose the tag because the
  pointing walked off, not because we cropped. A digital gimbal doesn't retain a target the
  body isn't pointing at; it mainly helps *pose stability under vibration* (a real hardware
  concern later, not our sim failure).

---

## Q5 — Terminal closing-speed management

**Primer.** LOS-rate demand scales as `λ̇ ~ V_closing/R × (cross-range component)`. Two knobs:
raw closing speed, and the *geometry* you close on.

- **Slowing at terminal** trades speed for time-to-track — more 14 Hz detections per metre
  and a smaller m/s multiplier on any blind interval. We already do this (ADR-0010 #2 throttles
  terminal closing). *But ADR-0014 found slower terminal closing is **monotonically worse**
  below ~4 m/s* — it recreates the M4 too-slow tail-chase (the target outruns the interceptor
  inside 2.5 m). So raw slowdown is a spent lever; don't push it further.
- **Lead-angle / collision-course bias is the better real-world knob.** Flying a *constant
  lead angle* (lead/deviated pursuit — "a fixed angle between the velocity vector and the
  LOS") puts you on a collision triangle where **LOS rate stays near zero the whole way**, so
  the endgame never develops the λ̇ spike that loses our lock
  ([FAS, Guidance & Control Ch.15](https://man.fas.org/dod-101/navy/docs/fun/part15.htm);
  [Intercept geometry: lead collision](https://flyandwire.com/2021/03/26/intercept-geometry-part-viii-intercept-progression-lead-collision-p-825-02/)).
  This is what real interceptors do when they have a *running start* and a target track.
  **We have exactly that running start now — the S2 dash.** ADR-0009 deliberately chose a
  pure *crossing* geometry to make PN-vs-pursuit diagnostic, which is the **worst case** for
  FOV retention; a lead-collision bias during the dash (aim at where the target *will be*, not
  where it is) hands the terminal phase a low-LOS-rate geometry to finish. Software, free;
  couples with Q4's look-angle constraint and Q3's coast.

**Verdict — SOFTWARE = FREE.** The optimal terminal-speed profile for us is *not* "as slow
as possible"; it is "fast enough to catch (dash), then a moderate terminal speed on a
lead-collision course so the LOS rate stays low." No new hardware.

---

## Q6 — What CHEAP real systems actually do (the most relevant prior art)

| System | Class / cost | Terminal homing | Kill mechanism | Published Pk / miss |
|---|---|---|---|---|
| **Ukrainian FPV interceptors** (ZIRKA, Sting, etc.) | ~**$1–2k** | Operator locks target, then **automatic onboard terminal guidance** (RF / radar / thermal lock) | **~2 kg fragmentation warhead** (proximity) | Kyiv batch **>70%** of incoming Shaheds `[MARKETING/operator]`; Sting "**1000+** UAVs" `[MARKETING]` — [united24](https://united24media.com/defense-tech/ukraine-introduces-zirka-interceptor-drone-to-counter-shaheds-20321), [TWZ](https://www.twz.com/news-features/inside-ukraines-interceptor-drone-innovations-swatting-down-thousands-of-shahed-drones), [MilitaryTimes $1k](https://www.militarytimes.com/news/pentagon-congress/2026/03/11/these-are-ukraines-1000-interceptor-drones-the-pentagon-wants-to-buy/) |
| **Anduril Anvil** | Group 1/2 kinetic quad-interceptor (cheapest Anduril tier) | Autonomous **onboard terminal guidance** after ground cue | **Kinetic collision (hit-to-kill)**; frag variant exists | "minimal collateral damage" `[MARKETING]` — [Anduril Anvil](https://www.anduril.com/anvil), [DefensePost Falcon Peak](https://thedefensepost.com/2025/10/20/anduril-demos-cuas-falcon-peak/) |
| **Raytheon Coyote Block 2** | jet, **~$100–125k**, 10–15 km | Advanced RF seeker (KuRFS cue) | **Blast-fragmentation** warhead, ring-shaped pattern (proximity) | — [TWZ jet Coyote](https://www.twz.com/43799/this-footage-of-jet-powered-coyote-drones-obliterating-other-drones-is-incredible), [cost](https://nationalinterest.org/blog/buzz/the-navys-roadrunner-and-coyote-anti-drone-systems-in-development) |
| **Anduril Roadrunner-M** | reusable VTOL, **~$500k** | AI autonomy | **High-explosive** (proximity) | "near-zero cost relaunch" `[MARKETING]` — [NationalInterest](https://nationalinterest.org/blog/buzz/the-navys-roadrunner-and-coyote-anti-drone-systems-in-development) |
| **Fortem DroneHunter F700** | reusable multirotor | AI radar/EO tracking | **Tethered net capture** (multi-metre catch radius) | — [Fortem](https://fortemtech.com/products/dronehunter-f700/) |
| **APKWS** (70 mm laser-guided, air-to-air C-UAS) | rocket, far cheaper than a missile | Semi-active laser | Proximity-fuzed frag variant for drones | [AF air-to-air APKWS](https://www.armyrecognition.com/news/aerospace-news/2026/u-s-air-force-approves-145m-dual-mode-apkws-ii-air-to-air-rocket-to-counter-drone-swarms) |

**The pattern cheap kinetic interceptors converge on:**
1. **Proximity fragmentation, not hit-to-kill** (or a net). The direct-hit precision problem
   is *bought out* with a few-metre lethal radius. Hit-to-kill is reserved for
   $M-class systems against 50–80 cm bodies at hypersonic closing speeds
   ([DSIAC](https://dsiac.dtic.mil/articles/small-missile-big-mission/)).
2. **A "smarter sensor" cues, a cheap seeker finishes.** Every one of these is ground/air-cued
   then hands off to a modest onboard terminal seeker — *exactly our two-sensor→one-sensor
   architecture* (ADR-0010/0015).
3. **The seeker is whatever's cheap** (RF, thermal, EO, or our fiducial stand-in) — the value
   is in *holding the lock into the last window*, which is precisely the risk ADR-0015 named.

**This is the strongest external validation the project has:** our architecture (cheap
cued interceptor + onboard terminal seeker + comms-denied finish) *is* what the $1–2k
Ukrainian FPV interceptors field today, and they succeed with fragmentation, not hit-to-kill.

---

## Q7 — Things the six questions missed

- **Seeker–guidance co-design (Integrated Guidance & Control, IGC).** The narrow-FOV
  strapdown literature increasingly designs the guidance law *and* the autopilot/pointing
  loop **together** so the look-angle constraint is enforced end-to-end
  ([IGC for narrow-FOV strapdown seeker](https://www.sciencedirect.com/science/article/abs/pii/S0019057820302585)).
  For us this means: don't tune the yaw loop and the guidance law in isolation (we've been
  doing that) — the yaw-rate authority (Q4) and the look-angle-constrained law (Q4) are one
  coupled design. Software, free, but a design-discipline point.
- **Navigation-ratio behaviour near CPA.** Higher N gives a shorter, lower-total-effort
  trajectory *early*, but concentrates acceleration (and thus our yaw-rate demand) *late*
  ([optimal PN gain setting, AIAA](https://arc.aiaa.org/doi/10.2514/1.59226)). Our N=5 is
  the lab's best for *miss*, but a terminal N-taper (Q2) may be the right co-move with the
  look-angle constraint.
- **Parasitic/radome-loop analog.** In real seekers, imperfect body-motion decoupling (radome
  refraction, seeker disturbance-rejection rate) feeds body rate back into the *measured* LOS
  rate, forming a **parasitic loop that increases miss and can destabilize** the terminal
  homing ([Radome error & DRR parasitic loop, MPE 2018](https://onlinelibrary.wiley.com/doi/10.1155/2018/1890426)).
  **Our direct analog:** the `λ = ψ + β` strapdown reconstruction *is* a body-motion
  decoupling, and any error in the EKF yaw ψ or in the yaw-rate feedforward couples straight
  into λ̇ — the same parasitic mechanism. Worth watching if the terminal starts chattering
  after we add the look-angle law.
- **Target glint analog.** Real seekers suffer low-frequency "glint" (the aim-point wanders
  over the target body) that is hard to filter and raises miss
  ([seeker isolation & glint](https://www.researchgate.net/publication/290318193_Effect_of_seeker_isolation_on_guidance_and_control_of_terminal_guided_projectile)).
  Our analog is AprilTag *pose jitter / corner glint* at close range and oblique aspect
  (the ADR-0003 monocular-planar-ambiguity risk) — a real terminal noise source the sim
  partly reproduces and a real seeker will have worse.
- **Sensor latency vs time-to-go.** The whole homing-time-constant story (Q1) says the
  *ratio* of loop latency to remaining time-to-go is what matters. At FPV terminal speeds
  time-to-go collapses to fractions of a second, so a fixed ~70 ms detect+filter latency
  (plausible on the Pi-5, ADR-0012) is a *larger fraction of t_go* than in the sim — a
  known, disclosed sim-vs-real gap that makes any sim Pk an **upper bound**.

---

## Summary table (technique → CPA benefit → cost → portability → expected Pk impact)

| Technique | Why it helps at CPA | Cost | Portable to our sim? | Expected Pk impact |
|---|---|---|---|---|
| **Proximity-fragmentation kill metric** (R_lethal ~2.5–3 m) | Doesn't need a small miss — a few-metre lethal radius makes ~1–2 m a kill | **FREE (framing)** | Software/metric only | **Large** — our own batch: 70% @2.5 m, 95% @3.0 m |
| **Memory-tracking inertial coast** (split-freeze + λ̇ cap + short lead-extrapolation) | Propagates target state through the seeker-blink instead of a dumb ballistic freeze | **FREE** | Software; PX4 IMU/EKF already onboard | Medium (lab under-prices; Gazebo A/B) |
| **Look-angle / FOV-constrained guidance law** (OLAGL / hybrid) | Bounds terminal look-angle demand so the boresight keeps the lock — attacks root cause | **FREE** | Software A/B; never tried | Medium-high (fixes the 60/60 dropout) |
| **Yaw-rate authority** (`MPC_YAWRAUTO_MAX`) | Lets pointing keep up with LOS rotation; keeps data flowing | **FREE** | Software (runtime param) | Low on miss, +10–15% coverage |
| **Lead-collision geometry bias** (during the dash) | Constant lead angle ⇒ near-zero endgame LOS rate ⇒ no λ̇ spike | **FREE** | Software; needs the S2 dash (built) | Medium |
| **Terminal N-taper / λ̇ cap** | Lower late acceleration/yaw demand, keep lock (vs textbook ramp-up) | **FREE** | Software A/B; N sweep exists | Low-medium |
| Impact-angle / OGL shaping | Drives LOS rate to zero at terminal instant (gentle endgame) | FREE | Software but needs clean t_go/state | Uncertain (state-limited, like PIP) |
| Second wide-FOV terminal camera | Physically retains a fast crosser in view | **HARDWARE ~$60–75** | Yes (Hailo dual-stream, ADR-0015) | Medium (after free yaw lever) |
| APN (target-accel term) | Cancels maneuver-induced LOS-rate ramp | FREE | **Already tested — REJECT** | None (noisy 2nd derivative) |
| Mechanical gimbal | Keeps target centred as LOS rate spikes | HARDWARE $$ + weight | **REJECT** (breaks strapdown mechanization) | N/A |
| IR seeker | — | HARDWARE $$ | **REJECT** (ADR-0014; bottleneck is kinematic) | None |

---

## What to add first (ranked cheapest-highest-impact)

1. **Adopt the proximity-fragmentation Pk metric — FREE, do it first.** Fix R_lethal a
   priori from real fragmentation data (headline 2.5–3 m, show the full curve, disclose the
   no-collision-volume simplification per ADR-0014's honesty boundary). This alone reframes
   the "wall": under the kill mechanism cheap interceptors actually use, we are already at
   70–95% Pk. It costs nothing and is the highest-leverage single change.
2. **Memory-tracking inertial terminal coast — FREE.** Implement ADR-0014 lever #2 in full:
   split-freeze (keep v_close/yaw/λ_hat live), cap |λ̇| in *both* command and filter predict,
   and a ≤0.5 s capped lead-extrapolation off the TargetTracker seeded from the last state.
   This is the literal missile answer to seeker blink and we already own the IMU/EKF. Gate it
   with a near-R=0 `--bench` regression before trusting a flight.
3. **Look-angle / FOV-constrained guidance-law A/B — FREE.** Add a terminal look-angle
   constraint (OLAGL-style) alongside pure-PN, plus the free yaw-rate-authority param and a
   lead-collision geometry bias during the dash. This is the direct real-world fix for the
   yaw-tracking-rate deficit and the one technique the project has genuinely never tried.
   Lab-rank it, then Gazebo-decide it (our standing rule).
4. **Only if 1–3 leave the acquisition budget short:** a second wide-FOV terminal camera
   (~$60–75, ADR-0015). Small hardware, real FOV retention — but spend the free levers first.

**Rejected as costly/ineffective for a cheap FPV interceptor:** mechanical gimbal, IR seeker,
APN, further terminal slowdown, and any tag/FOV inflation that launders "made it easier" as
"improved the algorithm" (ADR-0010/0014 honesty rulings hold).

---

## Answers to the builder's two direct questions

- **"Is our ~1–2 m even the right thing to fix?"** Mostly **no**. It's a miss only for
  hit-to-kill, which no cheap interceptor uses. Reframe to a proximity kill (few-metre lethal
  radius) and it's already a hit. The *residual* thing worth fixing is the terminal **lock
  dropout** (so the guidance has data to the end), and that's what items 2–3 address.
- **"Hit-to-kill or proximity for a cheap interceptor?"** **Proximity fragmentation** (or a
  net), unambiguously. Hit-to-kill needs centimetre precision, a fast closing speed, and an
  expensive seeker/airframe — it's a $M-class technique. Every $1k–$125k kinetic C-UAS in the
  field kills with fragmentation or capture.

---

## ADR-lite — proposed decision (not yet ratified; council-worthy given the metric change)

- **Context:** Every FPV-speed crosser loses the terminal AprilTag at CPA and misses ~1–2 m
  (60/60, ADR-0014/0018). Builder asked what real missiles do and what's cheap to add.
- **Options:** (a) keep chasing sub-metre hit-to-kill miss; (b) reframe to a proximity Pk
  metric + add the software terminal levers; (c) buy hardware (gimbal / second camera / IR).
- **Decision (proposed):** **(b).** Adopt a proximity-fragmentation Pk-vs-R metric (R fixed
  a priori from real warhead data, full curve published, no-collision-volume disclosed), then
  add three FREE software levers in order: memory-tracking inertial coast (split-freeze + λ̇
  cap + short lead-extrapolation), a look-angle/FOV-constrained guidance A/B (with free
  yaw-rate authority + lead-collision dash geometry). Defer the second wide-FOV camera; reject
  gimbal, IR, APN, further slowdown.
- **Why:** it's what cheap real interceptors (Coyote, ZIRKA/Sting, APKWS) actually do; the
  levers attack our re-diagnosed root cause (strapdown narrow-FOV lock loss + seeker-blink
  coast) with zero BOM cost; the honesty rulings (ADR-0010/0014) are preserved.
- **Risk / caveat:** the miss floor is partly *kinematic* at FPV crossing speed (ADR-0014) —
  coverage fixes may not move miss much; lab under-prices the coast/whipsaw, so every lever
  is "lab ranks, Gazebo decides," gated with a near-R=0 regression before any flight.
- **Date:** 2026-07-05 (research brief; ratify via a guidance council if the metric change is
  accepted, since it touches the resume-line success definition).
