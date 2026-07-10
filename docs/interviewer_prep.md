# DELIVERABLE A — ONE-PAGE RECRUITER SUMMARY

```markdown
# Counter-UAS Interceptor Sim — Camera-Only Proportional Navigation (PX4 / Gazebo SITL)

**Autonomous visual intercept of a moving drone, guided by the same law used in homing missiles since the 1950s — validated headless, logged, and reproducible.**

> **Resume line (made true and defensible):**
> *"Implemented proportional-navigation guidance for autonomous visual intercept in
> PX4/Gazebo SITL; validated via Monte-Carlo miss-distance analysis; diagnosed the
> fast-target kinematic limit and recovered it with a two-stage cue-to-camera handoff."*

---

## The capability this exists to prove (the headline — currently HELD, in validation)
**Design intent: when the datalink is jammed, the interceptor's own camera locks the target and finishes the intercept on its own.** An external ground cue steers a fast mid-course dash, then hands off — one-way, the channel is *structurally closed* — to a camera-only terminal phase. That comms-denied "the drone finishes itself" handoff is the whole point of the architecture.

> **Status: HELD (ADR-0059) — architecture built, not yet a proven capability.** The post-latch half is real and tested: once the camera terminal latches, the cue channel is structurally unreadable, so a link jammed *after* handoff cannot touch the terminal. But the project's own design review found the adopted anti-phantom config **fails closed under a cue jammed mid-dash, before camera acquisition** — the anti-phantom gates compare against the last-received cue position, which freezes when the link dies, and the frozen reference rejects the *real* target: the interceptor never hands off. A fix is **implemented and unit/honesty-tested** (86 tests pass; inert when the cue is fresh) and the paired jam Monte-Carlo **has now flown** (ADR-0059, 8 arms): it **demonstrated** the fail-closed bug with a clean dose-response witness collapse (REAL-ish handoffs 12→2→0 across 15/18/22 m cutoffs), and **validated the fix as fail-SAFE** — under jam it ages out the dead cue and aborts cleanly, eliminating the phantom-handoff, but it does **not yet restore** the intercept (without a search behavior, 13–15/16 abort before reaching handoff). Full camera-only **recovery** = fix + `--coast-search`, in validation now (task #39). Until that recovery batch closes, "works comms-denied" stays a held claim everywhere in this project's materials.

---

## Four headline numbers
| # | Result | What it means |
|---|---|---|
| **1** | **Pro-nav flew 4.6–7.6× tighter than pursuit** — 0.28–0.44 m vs 2.0–2.5 m miss against a moving target, camera-only | The classic guidance result, measured, not asserted — 3 gated runs on identical paths |
| **2** | **Diagnosed the fast-target miss as kinematic, not perceptual** — the final miss is a near-deterministic function of handoff geometry (r²≈0.96 vs zero-effort-miss@handoff, i.e. 96% of the run-to-run *variance*, not 96% of the miss itself), and the terminal window's physical correction capacity (**0.72 m**) sits far below the **1.69 m** delivered — a *perfect* seeker cuts only ~25%. Then recovered it: the running-start + velocity-emitting-cue profile makes the whole 6–12 m/s FPV band catchable — **n=96 final batch: 96.9% clean, mean miss 1.08 m, median 0.93 m** (ADR-0036) | Root-caused a hard limit and engineered around it, instead of blaming the sensor |
| **3** | **Removed the fiducial and survived maneuvers** — a markerless NN seeker plus an anti-phantom **detect-then-track** terminal holds camera-only intercepts against a 12 m/s weaving target: post-handoff camera-terminal Pk@2.5 m **weave 3/16 → 14/14**, phantom handoffs **12 → 0**, **0 of 155** terminal detections false (ADR-0056..0058) | The disclosed #1 risk (the AprilTag stand-in) was attacked head-on, with an AprilTag control isolating the failure to perception |
| **4** | **~60 logged decision records with real reversals and public retractions + verifier-gated milestones + an automated no-cheating test** | Engineering process a defense/GNC team can audit — the portfolio *is* the rigor |

Foundational gates all pass headless: camera-only static intercept held a 2 m standoff to **0.018–0.035 m** error (bar < 0.5 m); AprilTag detection at **1.000** rate, 0.086 m pose error.

---

## The honest limitation (stated up front, because owning it is the point)
**The AprilTag was a stand-in for "a reliable target lock exists" — and the project then removed it.** The current seeker is a markerless neural net with a detect-then-track terminal, built and Gazebo-tested against maneuvering targets (ADR-0038..0058). What remains honestly un-simulated is *real-world* perception — motion blur, vibration, real optics, real compute — so every perception number is an upper bound from clean rendered frames, and sim-to-real transfer is the disclosed #1 remaining risk (`docs/sim_to_real_gaps.md`). Comms-denied operation is a **held** claim (ADR-0059) until its jam batch lands.

---

## Where the hero media goes
- **[HERO VIDEO / GIF — top of page]** Side-by-side screen recording: pursuit vs. pro-nav intercepting the *same* crossing target. Pursuit trails and misses; pro-nav flies the lead and closes. This is the 10-second "it works" proof. *(Demo built — ADR-0032; a post-ADR-0058 re-cut on the markerless detect-then-track terminal is storyboarded in `docs/t25_storyboard.md`.)*
- **[ARCHITECTURE DIAGRAM — below the fold]** The two-sensor split: ground cue → mid-course dash → one-way handoff → camera-only terminal → PX4/MAVSDK. (Mermaid source already in README.)
- **[RESULTS PLOTS]** Pk-vs-lethal-radius curve and miss-distance CDF from the Monte-Carlo batch.

*Stack: PX4 SITL + Gazebo Harmonic, `gz_x500_mono_cam`, MAVSDK-Python, `pupil-apriltags`. Headless, every run logged to CSV. No ROS 2 — minimal dependency surface by design.*
```

---

# DELIVERABLE B — INTERVIEWER Q&A / DEFENSE PREP

```markdown
# Interviewer Q&A — Defense Prep (Aerospace / GNC)

Twenty of the hardest questions a GNC interviewer would actually ask, each with a crisp,
honest, defensible answer. Every number traces to an ADR, a gate script, or a log.
*(Currency note, 2026-07-10: Q3/Q16/Q17/Q18 were written before the markerless-seeker,
EKF, and fusion arcs flew — each now carries a dated status update rather than a silent
rewrite, because the reasoning-then-result arc is itself the portfolio. Q20 is the
comms-denied question, answered under the ADR-0059 HOLD.)*

---

### 1. Explain proportional navigation. Why does it work?
Pro-nav commands lateral acceleration proportional to how fast the line of sight (LOS) to
the target is rotating: `a = N · Vc · λ̇`, where `λ̇` is the LOS turn rate, `Vc` the closing
speed, and `N` the navigation constant. The geometric insight: if the bearing to the target
*isn't rotating* while range is closing, you're on a collision course — so driving `λ̇` to
zero *is* the intercept condition. It's robust because it only needs an **angle rate**, not a
target-velocity estimate — which is exactly why a noisy monocular seeker can still fly it.

### 2. Why N=5 for the navigation constant?
The M4 council started at **N=4** — theory says any N≥3 collapses the miss against a
constant-velocity target, and N=4 leaves margin for filter lag without over-commanding. I
then swept {3,4,5} in the offline lab and in Gazebo; **N=5** gave the best miss and the gain
fit back to exactly **5.000 with r²=1.000 in both crossing directions** (a clean symmetry
check that also ruled out a sign bug). N is deliberately *fixed per run and swept in the
Monte-Carlo* rather than adapted in-flight — an adaptive N is a new failure mode I chose not
to add. So N=5 is empirical-best-with-margin, not a textbook default I copied. *(ADR-0009,
ADR-0011, ADR-0024)*

### 3. What does the AprilTag hide? What's the real perception problem?
The AprilTag is a **stand-in for "a reliable bearing to the target exists"** — nothing more.
It lets me isolate and prove the guidance/control loop, which is *agnostic to how the bearing
was produced* (pro-nav only ever consumes an angle rate). What it hides is the genuinely hard
part: a real terminal seeker must **find** a small, fast, non-cooperative drone against sky
clutter with **no fiducial**, then **hold** that lock through motion blur and airframe
vibration in the final 1–2 seconds. I've *designed* that half on paper — detect-then-track
(motion proposal + small ML classifier + correlation tracker) on a Pi 5 + Hailo NPU, which
raised tiny-target recall from 0.41 to 0.86 in the cited study — but I have **not** simulated
it, and I name it as the project's **#1 existential risk**. No guidance cleverness closes it.
*(ADR-0015)*

*(Status update, 2026-07-10: this answer has since been overtaken by the work it called for.
The markerless seeker was built and Gazebo-flown (ADR-0038..0043 — matches the tag's miss when
it acquires early; hard-negative retraining took false-detection pollution 0.751 → 0.000), its
12 m/s maneuvering failure mode — high-confidence phantom detections ~18 m off target — was
isolated with an AprilTag control, and a **detect-then-track** terminal fixed it: post-handoff
camera-terminal Pk@2.5 m weave 3/16 → **14/14**, phantom handoffs 12 → 0 (ADR-0056..0058).
What stays true: REAL-hardware perception — blur, vibration, real optics, real compute — is
still un-simulated and is the #1 sim-to-real risk, `docs/sim_to_real_gaps.md`.)*

### 4. Why did your running-start number change between versions? *(the honesty correction)*
This is my favorite thing to own. I first showed a "running start" (ground launch + longer
standoff + faster dash) cutting a 9 m/s crosser's miss ~47% — **but that ran on an idealized
cue** (flat 0.5 m noise, fixed latency, no dropout). When I re-flew it under a **realistic
degraded cue** the number got *worse*: 9 m/s went 1.90 → **2.93 m**, and **33% of flights
never even reached handoff**. Rather than bury that, I logged it as a formal honesty
correction and chased the root cause: the real lever wasn't the running start itself, it was
the **mid-course velocity track** (~70–75% of the delivered miss). The cue was emitting
position only, so the interceptor's velocity estimate never converged — plus a real bug where
a "16 m/s dash" was silently clamped to 13. Having the ground sensor **emit a filtered
velocity** and unclamping the dash gave, *under the same realistic cue*, **1.19 m @ 9 m/s and
1.48 m @ 12 m/s with 6/6 handoff** — beating even the old idealized-cue number. The lesson: an
optimistic assumption inflated a headline; finding that made the final result both better and
trustworthy. *(ADR-0028, ADR-0030)*

### 5. What breaks on real hardware?
In rough order: **(a) terminal perception** — no fiducial, real blur, vibration, uncontrolled
lighting, and a slower embedded CPU all shrink the already-marginal camera-only window
(existential, not a tuning nuisance). **(b) Every timing constant** — filter gains, the
acquire streak, terminal ranges — was tuned to the sim's ~14 Hz desktop detection cadence;
I explicitly assume *none* port unchanged and gate the hardware plan on a bench measurement of
real detection Hz first. **(c) The physics I don't model** — no motion blur, rolling shutter,
or vibration, so real detection cadence is worse than my logs, never better. **(d)
Cross-platform datum/clock skew** between the ground sensor and the drone (RTK vs standard GPS
= 2–3 m common-mode) — the sim uses a perfect world→NED mapping the real system has to earn
with a shared RTK base and PPS-disciplined clocks. Hardware is *chosen* (Pixhawk 6C, Pi 5 +
Hailo, global-shutter cam, X500) and *staged bench-perception-first*, but not built.
*(ADR-0012, ADR-0015)*

### 6. Why ENU, FRD, and NED — why not pick one frame?
Because each stage has a *natural* frame and forcing one frame everywhere adds sign-error
surface. **World = ENU** (origin at the interceptor's start) for logging and geometry.
**Camera = OpenCV convention** (z forward, x right, y down) because that's what the detector's
pose output already uses. **Interceptor body = FRD** (forward-right-down). The key design
choice: the camera measurement is inherently *body-relative* (a bearing off the nose), so I
command **body-frame velocity + yawspeed** — which needs no compass/heading conversion at all,
eliminating a whole class of sign bugs. PX4's own setpoints are NED, so the one conversion I
do keep (world→NED) I **determined empirically**, not by assumption: I measured
commanded-velocity-vs-actual-displacement residuals and found `north=world_y, east=world_x` —
the *opposite* of the naive guess. Agreeing these frames up front, and verifying the one
mapping with data, is exactly how you avoid the classic guidance sign-flip. *(GOALS.md,
ADR-0008, ADR-0013)*

### 7. How do you know the guidance isn't cheating with ground truth?
Three layers. **(1) Structural:** the AprilTag detector and filters only ever touch rendered
camera pixels — they never subscribe to the ground-truth pose topic. The mocked ground cue
*is* allowed a *degraded* version of truth, because it stands in for a real independent sensor
that GOALS.md scopes out — but at handoff that channel is **closed and the holder nulled**, so
a post-handoff cue read is *structurally impossible*, not just avoided by convention. **(2)
Numeric, per gate:** every gate re-derives that the commanded velocity's azimuth correlates
with the *camera-measured* LOS, not the scoring feed, and asserts zero external-labeled ticks
after handoff. **(3) Static:** `tests/test_honesty_static.py` is a sim-free AST check on the
guidance source itself — it fails the build if any ground-truth or post-latch cue read ever
feeds a command. Ground truth streams to the CSV logs for *scoring only*. The verifier also
independently confirmed the detector and truth ranges are never identical (0/67 rows, ~0.7 m
mean offset) — a genuinely separate measurement chain. *(README honesty section, ADR-0010 #5,
ADR-0026)*

### 8. Why is pursuit ≈ pro-nav at high speed, when you also claim pro-nav wins 4.6–7.6×?
Both are true, and together they're the more interesting result. Pro-nav's decisive win is a
**slow-target** result: at 2 m/s it was 4.6–7.6× tighter than pursuit because there's enough
time-to-go for the law to null the miss. At **FPV crossing speed (6–12 m/s) the two laws tie**
— not because pro-nav failed, but because the miss becomes **kinematically limited**: the
geometry is delivered before the terminal law can act, so *which* law you run stops mattering.
So the guidance *law* dominates in the slow/close regime, and the engagement *geometry*
(running start) and kill *radius* dominate in the fast regime. That regime map is a more
mature systems finding than "pro-nav always wins," and I'd rather present that than
cherry-pick the speed where my law looks best. *(ADR-0029, ADR-0009)*

### 9. You call the fast-target miss "kinematic, not perceptual." How do you know it isn't just your seeker being bad?
Two model-independent arguments, not just a correlation. **(1) A capacity bound:** the terminal
window is ~0.4 s, and the interceptor's demonstrated lateral acceleration (~8.7 m/s²) can
correct at most `½·a·t_go² ≈ 0.72 m` in that window — but the geometry *delivered* at handoff
was a **1.69 m** zero-effort-miss. The physics simply can't null 1.7 m in 0.4 s, regardless of
sensor quality. **(2) A counterfactual:** substituting a *perfect* terminal camera cuts the
miss only ~25%. And **(3)** the correlation backs both up — r²(miss-at-handoff, final miss) =
0.96 at 6 m/s, and it *sharpens* with speed (0.82 → 0.96 → 0.99 across 3/6/9 m/s), reconfirmed
across FOV, gate, and streak changes. The ">1 s lost detection at closest approach" that looks
like a perception failure is ≥85% the harmless *outbound flythrough* — a symptom, not the
cause. That's why the fix was earlier acquisition and a better handoff track, not a better
seeker. *(ADR-0023, ADR-0026, ADR-0027)*

### 10. Is the lethal-radius (Pk) metric self-serving — a way to hide a bad miss?
Fair challenge, and I built the guardrails specifically to answer it. **The radius is fixed
from a real kill mechanism's physics *before* looking at results** — kinetic ram ≈ 0.5 m
(what cheap FPV interceptors actually do) and net ≈ 1.5 m — never reverse-engineered to clear
a threshold. I **headline the full Pk-vs-radius curve**, per-speed with confidence intervals,
so a reader applies their own bar, and I explicitly *refuse* to headline R ≥ 2 m (that region
mostly scores ballistic coast). Critically, it's honest *because* of the kinematic diagnosis:
the miss is floored near ~1 m at 6 m/s **even with perfect sensing**, so a proximity kill
isn't hiding a bad miss — it's the physically *required* design, since you cannot null the
last meter in the time available. That's also what every fielded cheap kinetic counter-UAS
does; hit-to-kill is a $M-class technique. The honest caveat I always state: the sim target is
a flat board with no collision volume, so *any* R is a disclosed narrative assumption, not a
simulated hit. *(ADR-0025, ADR-0021, README honesty section)*

### 11. Why did PIP — the theoretically superior lead-guidance law — lose to plain pro-nav?
This is one of my favorite negative results. In the offline lab, PIP (a Predicted-Intercept-
Point lead solve) beat pure pro-nav 2–4× on every path. In Gazebo, camera-only, it was
**worse** — 3.03 m vs 1.6 m. Why: PIP's lead solve needs a clean target *velocity* estimate,
and a real monocular stream (~14 Hz, 30–40% edge dropouts) starves that estimate, so its
predicted intercept point is simply wrong. Pure pro-nav needs only the LOS *rate*, which
survives the noisy, intermittent stream far better. The resume-worthy phrasing: *the
theoretically superior law degraded under realistic sensor noise; the simpler, sensor-light
law proved more robust.* It taught me the project's core discipline — **"lab ranks, Gazebo
decides"** — and PIP does recover to roughly tied once it inherits the cue's clean mid-course
track after handoff, which is itself an informative data point. *(ADR-0011)*

### 12. "Lab ranks, Gazebo decides" — quantify that. How often was your fast offline model wrong?
The offline point-mass lab exists to screen ideas cheaply, but it does **not** get the final
word — and I keep score. It ranked optimistically, in the same direction, in **7 of 8**
head-to-head checks: PIP (lost in Gazebo), Kalata-index filter gains (won −24–28% in lab,
worse in every Gazebo flight), differentiate-vs-emit velocity (2.1 m lab swing shrank to a
not-yet-significant 0.41 m), mid-course fusion (recovered coverage 0.08→0.96 in lab, flat in
Gazebo), and the FOV-narrowing (below). The lab was recalibrated to match Gazebo's real
failure modes, but it's *trusted for ranking candidates, never for an absolute number* — only
a Gazebo gate is. Documenting where my own fast surrogate misled me is, I think, more
convincing than a model that was never wrong. *(ADR-0011 3rd addendum, ADR-0013, ADR-0018)*

### 13. Give me a decision you got wrong and reversed.
Narrowing the camera FOV from 99.7° to 60°. The analytic model *and* the offline lab both
predicted a win — a narrower lens resolves the tag farther out (~15 m vs ~6 m), which the fast
regime is starved for. Gazebo overruled it: the narrow field detects farther but **can't
hold** a fast crosser as it swings off the boresight, and the 9 m/s handoff-latch rate
collapsed from 42% to **0%**. I did *not* adopt it — the sim keeps the validated wide lens —
and the real acquisition fix turned out to be a looser lock requirement (2 detections instead
of 3). One keeper: the exercise produced a script that now *measures* the detection envelope
instead of assuming it. A lab-endorsed idea, empirically overturned, the right fix found, the
fundamental limit reconfirmed — that arc is the methodology, not a footnote. *(ADR-0024)*

### 14. Why simulation only, and why no ROS 2 — doesn't that limit what this proves?
Simulation-only is a deliberate scoping choice: it lets me prove the **guidance and control
core** — the part that ports to real hardware nearly as-is — with reproducible, logged,
re-runnable numbers, before spending on an airframe. The honest boundary is drawn explicitly:
what transfers (the guidance loop) vs. what doesn't (perception, real physics). **No ROS 2** is
a minimal-dependency-surface decision — camera frames come straight out of Gazebo via
gz-transport, control via MAVSDK over local UDP. It also drove a real hardware finding: it
initially argued against a Jetson (whose only edge was CUDA AprilTag via Isaac ROS = ROS 2),
though I later reversed the *compute* choice to Pi 5 + a Hailo NPU once I confirmed real ML
ships a standalone Python API and doesn't force ROS 2. Fewer moving parts, every interface
auditable. *(GOALS.md, ADR-0012, ADR-0015)*

### 15. What would you do next?
In priority order: **(1) The perception half** — build detect-then-track on the Pi 5 + Hailo
bench against a printed tag at real size, in outdoor light, spinning to reproduce yaw-rate
blur, and *measure* real detection Hz. That's the go/no-go gate on the whole hardware plan and
directly attacks the #1 risk. **(2) Stress perception availability** — the realistic-cue
dash-aborts showed a degraded ground link causes *catastrophic* mid-course failures, so I'd
quantify the jam/dropout envelope. **(3) Tighten the statistics** — my Monte-Carlo cells are
n=6–8; the qualitative findings are robust but the per-speed deltas need a larger batch before
any headline. **(4) Ship the portfolio** — the demo GIF and writeup, disclosing the perception
gap as future work. What I would *not* do is more guidance tuning — the analysis puts the
theory floor near where I am, and the remaining gap is genuinely the faked perception half, not
the guidance math. *(ADR-0030, ADR-0029)*

---

### Bonus — the one-way handoff: why does it matter and how is it enforced?
The comms-denied story only means something if the ground channel is *provably* gone during
the terminal phase. So handoff isn't "stop reading the cue" — at the latch tick the UDP socket
is **closed and the holder set to null**, making a post-handoff cue read raise rather than
return stale data (illegal-state-unrepresentable). The mock keeps logging as
available-but-unread evidence. **Scope, stated honestly (ADR-0059):** the latch proves the
claim for a link jammed *after* the camera terminal locks — a post-latch jam structurally
cannot touch the terminal, and that is backed by a static test, not a promise. It does **not**
yet prove a jam *before* camera acquisition: the adopted anti-phantom config fails closed
there (found by the project's own design review), the fix is implemented and unit-tested but
the jam Monte-Carlo has not flown — so the full "jammed mid-flight" sentence is a **held**
claim (see Q20). *(ADR-0010 #5, ADR-0059)*

---

### 16. Your seeker uses an AprilTag — a printed marker. Isn't that cheating? A real drone has no marker.
Yes, and I say so before you have to ask — it's the project's **#1 disclosed risk**, not a
thing I hope you don't notice. GOALS.md makes exactly *one* honest simplification: the
AprilTag stands in for "a reliable target lock exists," so the sim can isolate and prove
the **guidance/control** core, which is agnostic to how the bearing was produced. What the
tag hides is the genuinely hard part — finding and holding a lock on a small, fast,
non-cooperative drone against sky clutter with no fiducial. The next build-queue item
(ADR-0033 item 2) **deletes the tag**: a markerless seeker that detects the target drone's
own body and feeds the *same* `Measurement` bearing/range interface the guidance already
consumes — one dataclass, one thread, the perception→control seam was built for this swap.
Plan is classical detect-then-track first (ego-motion-compensated frame differencing → blob
→ correlation tracker → the existing α-β filter), then a lightweight pre-built neural
detector (NanoDet-Plus or an MIT drone fine-tune — I avoid AGPL YOLO for a public repo) as
the classifier stage. The guidance math doesn't change, because pro-nav only ever needs an
**angle rate**; what changes is the camera degrades from sub-degree/~5% (tag) to
~1–2°/15–30% (a real box), and range comes from known-size pixel scaling instead of a pose
solve. And per my own ZEM analysis (Q9), that's the *tolerable* degradation — bearing
quality dominates, range only throttles closing speed. So the honest headline is: the
guidance is real and ports as-is; the perception is faked *on purpose* and un-faking it is
the disclosed next step. *(GOALS.md, ADR-0015, ADR-0033)*

*(Status update, 2026-07-10: DONE — the tag is gone. See Q3's status update for the full arc:
markerless seeker flown, ADR-0038..0043; maneuvering phantom failure isolated with an AprilTag
control and fixed by detect-then-track, 14/14 camera-terminal at 12 m/s weave, ADR-0056..0058.
The guidance math was indeed byte-unchanged — the prediction in this answer held.)*

### 17. Did you use a Kalman filter?
Right now, effectively yes — just the frozen special case of one. The target track runs a
pair of **alpha-beta (g-h) filters**, and alpha-beta *is* the steady-state Kalman filter for
a constant-velocity target evaluated at a fixed sample rate and fixed noise, with the gain
frozen at its converged value and the covariance bookkeeping thrown away (you can derive it
straight from the Kalata tracking index). So I already ship the Kalman gain — it's just
hard-coded as α=0.5, β=0.30 on the LOS channel. The planned upgrade (ADR-0033 item 3) is a
proper **EKF** — EKF, not plain KF, because the camera measures *polar* (bearing, range) of a
naturally *Cartesian* target state, and fusing the Cartesian ground cue makes it nonlinear
either way. Here's the part I'd want to say out loud: I **pre-register a null on
end-to-end miss.** The miss is kinematic (Q9) — a near-deterministic function of handoff
geometry (r²≈0.96 vs ZEM, i.e. variance explained), with only ~0.72 m of the 1.69 m delivered
error physically recoverable in the terminal window — so a better estimator can only touch
that ~25% recoverable slice, and most of *that* is control logic, not estimation. Where the EKF *should* genuinely help is the covariance-derived gain surviving
our bimodal cadence (~14 Hz bursts and multi-second dropout gaps) — the exact thing that
made a naive adaptive-gain attempt (Kalata) win in the lab and then *diverge* in every Gazebo
flight (ADR-0013). So my honest expected result is "track RMSE and LOS-rate error improve,
end-to-end miss stays tied" — and I'd A/B it under the project's own discipline (paired
seeds, n≥8, report "not significant at this n"). Knowing when the fancy tool is *not* the
bottleneck is the point. *(ADR-0013, ADR-0023, ADR-0033; `docs/ekf_design_brief.md`)*

*(Status update, 2026-07-10: flown — ADR-0037 + addendum. The pre-registered null on miss
CONFIRMED (Δ −0.009 m, CI straddles 0). Bonus twist: an apparent 8/8 → 2/8 clean-rate collapse
was overturned by offline forensics — the EKF's physically-correct post-CPA range extrapolation
was tripping an abort-timer branch that alpha-beta's impossible negative coasted range never
does; the scoring lens, not the filter, was at fault. Corrected, the EKF stands at full parity.
Alpha-beta stays the default for measured reasons.)*

### 18. You have a ground sensor and an onboard camera — how would you fuse them?
With a covariance-gated mid-course EKF (ADR-0034), and I'd be careful about *where* fusion is
allowed to act. The intuition — "fuse when the ground cue helps, fall back to the camera when
it looks worse" — is *exactly* what a correctly-specified EKF does natively: it weights each
source by its live covariance (a far-range, datum-biased, or stale cue gets demoted
continuously), and its **innovation gate** *rejects* a cue measurement outright when it
disagrees with the prediction beyond a chi-square threshold — a literal "this ground reading
looks wrong, ignore it" switch that falls out of the estimator, not a hand-tuned `if`. The
interesting history: I already tested fusion once and it came out **default-OFF** (ADR-0018) —
but under two conditions rigged against it, a clean AprilTag (so the cue could only dilute an
already-excellent camera) and a fixed-gain tracker (which can't weight by instantaneous
quality). The markerless seeker makes the camera genuinely noisier and the EKF adds live
covariance weighting, so flipping both is the honest reason to re-open that "settled" null.
Two hard boundaries: (a) the payoff is **mid-course robustness — earlier lock, fewer
dash-aborts — not the terminal miss**, which the kinematic ceiling still caps; and (b) my
whole story is *comms-denied terminal*, so fusion stays **mid-course only** and the terminal
degrades to camera-only. That forces a sharper honesty audit than alpha-beta needed: not just
"no cue *reads* after handoff" but "**no cue-tainted filter *state* survives handoff**" — I
have to re-initialize or provably decay the cue's contribution to the state and covariance at
the latch, or the jam-resistance claim quietly breaks. *(ADR-0018, ADR-0034, ADR-0023)*

*(Status update, 2026-07-10: flown — ADR-0041/0044. Half right, half honestly wrong. With the
noisy markerless seeker, mid-course cue fusion WORKS — the hand-set polar `FusedTrack` beat
fusion-off on 8/8 paired seeds (median −0.356 m, p≈0.008) and survived the WORST-credible cue;
the ADR-0018 null flipped exactly as this answer hypothesized. But covariance gating — the
mechanism this answer advocates — did NOT earn its keep: a wash at the realistic tier and a
regression under WORST-tier cue bias (a 7.66 m flyby — the council's predicted bias-lock,
confirmed in flight). Fixed weights with "the cue never touches the angle" win. The honesty
rail held: zero post-latch cue updates across all 32 EKF flights, live-counted.)*

### 19. Does any of this survive real hardware? How would you find out cheaply?
I'd find out for ~$257 before spending a dollar on an airframe — that's the whole design of
the **Stage-0 bench** (ADR-0012/0033 item 1). A Raspberry Pi 5 + a global-shutter mono camera
runs the *exact* detection code against a *printed* AprilTag and produces a measured
**sim-vs-bench gap table**: static detection rate and range/bearing error vs. a tape-measured
reference, sustained detection Hz, the yaw-rate/motion-blur threshold where detection falls
off, and lighting robustness. It deliberately measures the three sim knobs I currently
*assume* — terminal range σ, bearing σ, and the high-LOS-rate dropout the sim doesn't model
at all — plus the single biggest unmeasured number in the whole project: the Pi-5 detection
rate for this detector, which every filter gain and terminal-timing constant was tuned
against assuming ~14 Hz desktop cadence. It's structured as an honest go/no-go: if sustained
Hz drops below ~8 or detection dies below ~30°/s of yaw, the camera-only terminal window
collapses on real hardware exactly as the council feared — and that's a *successful* ~$257
result, because it redirects effort to the Hailo/ML seeker path *before* the ~$260–530
airframe spend. The bench is perception-only — no flight, no PX4 — precisely because
perception is the risk; the guidance already reproduces from logs. *(ADR-0012, ADR-0015,
ADR-0033; `docs/stage0_bench_plan.md`)*

### 20. Your thesis is comms-denied intercept. Show me one flight where the link was jammed and the intercept completed. *(the ADR-0059 question)*
There isn't one — and I'll say that before you find it. **No flown arm, in any configuration,
has ever had the cue jammed**: every batch ran the cue mock full-duration (ADR-0059 is
explicit about this). Here is exactly what *is* demonstrated versus what isn't.
**Demonstrated:** the one-way handoff latch is real and structural — at the latch the UDP
socket is closed and the holder nulled, so a jam *after* camera lock cannot touch the
terminal; that's enforced by AST-level static tests and per-gate numeric audits. Strong
evidence, but evidence *by construction* — a different kind than a flown jam.
**Found by my own design review:** the adopted anti-phantom deployment config
(`--track --handoff-cue-gate 8`) **fails closed** under a jam *before* camera acquisition —
the anti-phantom gates compare against the last-received cue position, which freezes when the
link dies, and within ~1 s the frozen reference rejects the *real* target. No phantom chase,
but a mission kill in exactly the scenario the project exists to prove. The genuine tension:
the default config hands off fine under jam but eats phantoms; the config that beats phantoms
is the one that failed under jam. **Built, not yet flown:** a sim-time cue-staleness age-out
with camera-only fallback (inert while the cue is fresh; 86 unit/honesty tests pass, including
a regression witness that reproduces the frozen-cue failure and an anti-phantom check that the
fallback doesn't re-admit the ~18 m phantom) plus a paired jam Monte-Carlo harness
(`scripts/mc_jam_arm.sh`). **And the follow-up you should ask** — "so the one scenario the
project exists to prove has never been demonstrated end-to-end?" — *correct.* I'd rather hand
you that sentence myself. The claim is HELD project-wide until the jam batch lands; catching
the hole in my own review, retracting the claim everywhere, and building the fix and the
harness before flying it is the process this portfolio is actually selling. *(ADR-0059,
ADR-0060; `docs/design_review_sim_to_real_2026-07-10.md`)*
```

---

Both documents above are complete and self-contained. Every number traces to a milestone gate script, a timestamped CSV in `logs/`, or an ADR in `docs/decisions.md` (ADR-0003 through ADR-0060), with `GOALS.md` for the coordinate-frame conventions. Numeric conventions held throughout: the terminal window is **~0.4 s**; the load-bearing kinematic proof is the **capacity bound (0.72 m vs 1.69 m ZEM)**, with **r²≈0.96 vs ZEM@handoff** as the supporting correlation — r² is *variance explained* and is never rendered as "% of the miss" (the 0.99 freeze figure is near-tautological and not led with); the running-start honesty correction and dash-track fix (idealized 1.90/2.30 m → realistic-degraded 2.93/3.08 m → fixed 1.19/1.48 m, ADR-0028/0030) are the historical arc, flown under the old too-steep σ_R curve. The current statistical headline is the **M5 final batch (ADR-0036, n=96: 96.9% clean, mean 1.08 m, median 0.93 m)**, which supersedes the ADR-0029/0030 numbers; the current capability headline is the **markerless detect-then-track terminal (ADR-0058: 14/14 post-handoff camera-terminal at 12 m/s weave)**; and **comms-denied is HELD (ADR-0059)** until the jam Monte-Carlo flies. Where this doc and the README disagree, the README wins.
