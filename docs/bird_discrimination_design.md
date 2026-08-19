# Bird-vs-hostile-UAS discrimination — the SAFETY-CRITICAL design

> **Status:** research + design brief. **A DIRECTION, not yet ratifiable and not yet
> buildable as written** — an adversarial red-team (2026-07) found the architecture
> *direction* sound but the mechanization carrying **structural fail-UNSAFE holes** that
> the doc's own cited numbers expose (terminal-veto timing, confidence-signal semantics,
> streak/track binding). This revision folds in the eight required hardening mitigations;
> the design is safe to **pursue**, not to **ratify/build**, until it is hardened here AND
> the Stage-0 bench sets the thresholds. **Nothing built, nothing run. All operating-point
> thresholds are UNSET, and the interlock DEFAULTS TO VETO-ALL (engage nothing) until the
> bench measures them.** See **§0 Red-team hardening** first. Gated behind the M5 finish
> and the markerless-seeker / EKF-fusion capstone (ADR-0033 item 2, ADR-0034). This is the
> full synthesis of four research lanes (appearance-CV, kinematic + multi-modal, hardware +
> fusion, safety-interlock + sim-test), hardened against the red-team verdict.
>
> **The one question this doc answers:** *before the interceptor commits, how does it
> positively identify that the target is a hostile UAS and NOT a bird — and how does that
> decision survive the comms-denied terminal where the onboard camera is finishing alone?*
>
> **The HARD RULE this doc exists to enforce:** the interceptor MUST positively identify a
> hostile UAS before any commit, and MUST fail SAFE (break off) on doubt. This is both an
> ethics requirement (do not harm wildlife) and the project's own #1 fielded-C-UAS failure
> mode (ADR-0013; ADR-0019 §3: birds are *the* false-alarm source). Every quantitative claim
> below is cited to a paper, dataset, vendor page, or an in-repo ADR/brief — no invented numbers.

**New terms, one line each (defined once, then used freely):**
- **Positive ID / PID** — an affirmative *"this is a valid hostile target,"* stronger than mere
  *detection* (*"something is there"*). PID is the safety gate that authorizes a commit.
- **Micro-Doppler** — the tiny Doppler modulation that a spinning or flapping part adds on top
  of a target's bulk body motion. A drone's rigid props (blade rate ~50–100 Hz) and a bird's
  wings (~4–10 Hz) leave very different radar fingerprints — the classic bird discriminant.
- **Decision-level fusion** — each sensor emits its *own* class decision + confidence and a rule
  (Bayesian log-likelihood, or Dempster–Shafer) combines those *decisions*. Distinct from the
  EKF, which fuses *state* (position / velocity). We run BOTH channels on the same track.
- **Comms-denied terminal** — the project thesis (docs/goals.md): a jammer cuts the datalink in the
  last seconds, so the onboard seeker finishes ALONE. The whole safety problem is that **bird
  rejection cannot depend on a link the threat model says is denied.**
- **Fail-safe** — on *any* doubt the system does the inert thing (break off / do not engage).
  The interlock is a **veto** — it can only *prevent* an engage, never *cause* one.
- **P(hostile)** — a **dedicated hostile-class posterior**: the probability the tracked
  object is a hostile UAS. It is **LOW for a confidently-detected bird** and is the *only*
  signal the interlock reads. It is **NOT** detection/decode confidence (`decision_margin`),
  which is HIGH for a clearly-seen bird. Detection confidence and classification confidence
  are **orthogonal and must never share a slot** (§2c, §5.1).

---

## 0. Red-team hardening (safety review) — READ FIRST

An adversarial red-team reviewed this design (2026-07). Verdict, quoted:

> *"Right architecture, safe to PURSUE as a direction, but NOT safe to ratify/build as
> written — structural fail-unsafe holes the doc's own cited numbers expose. Needs a
> fundamental hardening pass on terminal-veto timing, confidence-signal semantics, and
> streak/track binding — not a redesign."*

**What that means for this doc's standing:** the three-phase architecture (confirm
mid-course, carry a committed decision, veto onboard) is the right *direction* and is
retained. But as originally written it would have **failed UNSAFE** in several concrete
ways its own numbers predict. This section is the load-bearing summary; the sections below
are rewritten to enforce it. **Until every item here is satisfied AND the Stage-0 bench has
run, this design is not ratifiable, not buildable, and the interlock DEFAULTS TO VETO-ALL
(engages nothing).** Thresholds are **UNSET on purpose** — committing a number before the
bench measures it is itself a fail-unsafe act (§6).

The eight structural fixes, each folded into the sections named:

1. **Terminal veto must be FAST and ASYMMETRIC, not a streak-to-abort (§2 diagram, §5.2,
   §5.3).** The measured terminal window is **0.41 s median** (~6–12 frames at 14–20 Hz;
   ADR-0023, terminal_diagnosis.md item 6). A 12-of-15 sustained-abort (~0.8–1.0 s) is
   *longer than the window it must fire inside*, and the old break-off exit fires ~5 s after
   CPA — structurally too late. **Fix:** to *hold* an engage in the terminal, the onboard
   must show **continuously-sustained POSITIVE hostile ID**; **any gap** in positive
   confirmation, **any 2-of-3 bird-like frames, or a single high-confidence bird frame**
   forces **immediate** break-off + standoff-widen. Positive-ID-to-hold, not streak-to-abort.
2. **The interlock reads P(hostile), never `decision_margin` (§2c, §5.1).** `decision_margin`
   is detection-*presence*/decode confidence — a clearly-detected bird scores it HIGH, so
   gating on it would *engage the bird*. The interlock must read a **dedicated hostile-class
   posterior** that is LOW for a confident bird. The two confidences are orthogonal and get
   separate fields in the contract.
3. **Bind the hostile streak to a tracked OBJECT IDENTITY, not the EKF track slot (§2c,
   §5.2, §5.3).** On any re-association / ID switch / detection gap, **reset the streak and
   re-earn PID from scratch**. Inside terminal, a re-association / fresh-target event is
   *itself* a mandatory break-off (0.41 s is too short to re-classify a swapped target).
4. **The carried mid-course prior is VETO-ONLY and asymmetric (§5.4).** A ground/mid-course
   prior may only **RAISE suspicion / LENGTHEN** the required onboard confirmation (bias
   toward break-off). It may **NEVER shorten N or lower the confidence bar toward engage.**
   The onboard must **re-earn** hostile ID after handoff; the prior can abort but never
   accelerate a commit. This is the honest reading of `classification_reads_post_handoff=0`:
   the ground label may bias **toward-safe only**, never toward-engage.
5. **ARM requires an AFFIRMATIVE powered-drone signature, never "failed to look bird-like"
   (§2b, §5.3).** True-size only separates *large* birds: stereo σ_R ≈ 0.45 m at 100 m
   (ADR-0017) **cannot** separate a 0.3–0.5 m bird from a 0.3 m quad; a straight glide shows
   LOW jerk (looks drone-like) and **never hovers** — and *absence of hover is not presence
   of a bird*. **Fix:** commit only on an observed **powered-drone signature** — hover /
   vertical-climb, radar rotor-line micro-Doppler, or a sustained powered-flight kinematic
   signature. No affirmative drone signature ⇒ **fail safe / break off.**
6. **The sim gate proves PLUMBING ONLY and is an UPPER BOUND (§6).** The clean primitive
   decoy (no flap, no motion blur, no vibration) **cannot exercise** the wingbeat/appearance/
   blur discriminants, so a green `check_bird_reject` is **wiring validated, NOT bird
   rejection validated.** Any bird-reject *claim* needs a **WORST-tier decoy with matched
   pixel footprint AND a flapping actor.** **Operating-point thresholds (PID_HIGH,
   PID_ONBOARD_MIN, N-of-M) MUST NOT be committed from the sim ROC** — they come from the
   measured `P_correct(R)` Stage-0 bench, and **until that bench runs the interlock defaults
   to VETO-ALL.**
7. **Stale / dropout / dead classifier == P(hostile)=0 == break-off, by construction (§5.2,
   §5.3).** Absent, stale (older than 1–2 frames), or timed-out classification is treated as
   a *bird vote*. `hostile_streak` / `hostile_conf` initialize to **non-hostile** at ENGAGE
   entry, and a **classifier-liveness watchdog** treats a stalled head identically to "bird."
8. **Flocks / merged tracks ⇒ ambiguity ⇒ break-off (§2c, §5.3, §6).** Any contested
   association (>1 candidate in the gate) or a recent merge/split event forbids engage. The
   sim plan adds multi-target / flock scenes.

**The asymmetric cost stays central (it is why all eight fail toward break-off):** a missed
drone is a recoverable bad day; a killed bird is irreversible (§1). Every ambiguity above is
resolved by *not engaging*. But that cost is **real, finite, and context-dependent, not
infinite** — the operating point on the Pk-vs-false-engage curve is a **rules-of-engagement
parameter** set by the deployment context (§1.5 CONOPS doctrine), while the **fail-safe envelope
this section builds** (affirmative positive-ID required, ambiguity → break-off, thresholds UNSET
⇒ VETO-ALL until benched) is **invariant and does not move with ROE.** The hardening is the
floor the doctrine stands on, not a ceiling on capability.

---

## 1. Why this is crucial — the asymmetric cost that drives the whole design

An autonomous engager makes two kinds of error, and **they are not equal.**

- A **false negative** (the gate aborts on a *real* hostile UAS) costs one missed intercept.
  It is recoverable — and in this project's own data the terminal *already* aborts routinely:
  the universal M4/S2 failure mode is "lost detection >1.0 s inside terminal range" (ADR-0023,
  ADR-0030). A miss is a bad day; nothing is harmed.
- A **false positive** (the system engages a *bird*) is an **irreversible kinetic act against
  wildlife**, and it is the single most-cited real-world counter-UAS failure. Fielded-C-UAS
  operators report "false alarms are more disruptive than missed detections," and "the RCS of a
  consumer quadcopter is often indistinguishable from that of a bird"
  ([moneyprouav operational reality](https://www.moneyprouav.com/capabilities-limitations-and-operational-reality-in-defense-grade-counter-uas-systems/);
  [SentryCS](https://sentrycs.com/the-counter-drone-blog/counter-uas-technology-myths-and-reality/)).
  The repo already names this its #1 risk (ADR-0013; ADR-0019 §3; seeker brief risk R6/R7).

**But the asymmetry is real, finite, and context-dependent — NOT infinite.** A missed drone is
recoverable; a killed bird is not — yet a system so bird-cautious that it lets an *armed hostile*
reach the people it defends is **also** a failure, and a worse one when the threat is real. Both
errors carry a cost; the design must weigh them, not zero one out. This is the doctrine that
governs the whole design and it gets its own section: **§1.5 (Concept of operations & rules of
engagement).**

**Design axiom:** deliberately trade recall for precision — but by a **tunable amount**, not to
an absolute. In ROC terms we pick a **high-precision operating point** on the Pk-vs-false-engage
curve: raise the confidence threshold, give up some true-positive recall, push the false-positive
rate down. In **peacetime testing** that operating point sits at **maximum precision** (the ≈0
target below); *where* it sits when actively defending people is a **rules-of-engagement
parameter, not a hard-coded constant** (§1.5). The move itself is the textbook safety-critical
one: *"the cost of one type of misclassification is more critical than another"*
([threshold-moving for imbalanced classification](https://machinelearningmastery.com/threshold-moving-for-imbalanced-classification/)) —
we simply refuse to freeze that cost ratio at infinity.

> **HEADLINE SAFETY METRIC (peacetime/testing target — the *fielded* operating point is ROE-set,
> §1.5):** *false-engage rate on birds ≈ 0.* The **≈0 is the maximum-precision operating point used
> for bring-up and peacetime testing**; the fielded operating point is a **tunable point on the
> Pk-vs-false-engage ROC set by rules of engagement** (§1.5), still **fail-safe by default**
> (ambiguity → break off) and still requiring **affirmative positive-ID** — ROE moves *where* on
> the curve, **never *whether* PID is required.** The testing target is **0 / N over the entire
> bird corpus**, reported as k/N with a one-sided upper confidence bound (e.g. "0/200, 95% upper
> bound < 1.5%"). Correct-engage rate on drones (recall) is a secondary metric and is *allowed* to
> be < 100% — a missed drone is the acceptable error, though (§1.5) **not an infinitely acceptable
> one.**

This is the resume upgrade too: from *"it intercepts"* to *"it intercepts **and refuses to** —
an autonomous **veto** the machine can assert without a human (out-of-the-loop is safe *in the
break-off direction*), wrapped in **human-on-the-loop authority over the commit/abort** (§1.5.2),
a measured ~0 false-engage rate on wildlife at the peacetime operating point, an explicit
asymmetric-cost rationale that is **finite and ROE-tunable** (§1.5), and a comms-denied
classification-honesty boundary that mirrors the guidance-honesty boundary." That is exactly the
responsible-autonomy story an aerospace / defense-autonomy reviewer looks for.

---

## 1.5 Concept of operations & rules of engagement (doctrine)

> **Why this section exists.** The §0 red-team hardening deliberately pushed the *mechanization*
> toward maximum conservatism — thresholds UNSET, interlock **defaults to VETO-ALL until benched.**
> That is the correct **engineering** default under measurement uncertainty, and it stays. But it
> must not be mistaken for the operating **doctrine.** The point of this whole design is
> **defense-in-depth, not infinite bird-caution.** A system so afraid of a bird that it lets an
> armed hostile reach the people it defends has not been "safe" — it has *failed at the job it
> exists to do.* This section states the doctrine that sits **on top of** the hardening (it does
> not loosen a single §0 fix) and keeps the two failure modes in honest balance.

### 1.5.1 The asymmetric cost is real, but FINITE and CONTEXT-DEPENDENT — the ROE-tunable operating point

Plainly: **a dead bird sucks, but a dead soldier sucks more.** The asymmetry in §1 is real — a
missed drone is recoverable, a killed bird is not — but it is **finite, not infinite,** and it
**depends on context.** Against a confirmed armed hostile closing on the people the system
defends, a reflexive "never risk a bird" is *itself* a way to get someone killed. Both errors are
real; the design weighs them, it does not zero one out.

**Resolution — no single hard-coded ultra-conservative constant.** Replace any fixed "drive
false-positives to zero, full stop" threshold with a **tunable operating point on the
Pk-vs-false-engage-on-birds ROC**, chosen by **rules of engagement / the deployment context:**
- **Peacetime · testing · bring-up →** maximum precision, false-engage target **≈0** (the §1 headline).
- **Actively defending people against a CONFIRMED threat →** the **acceptable false-positive
  *bound* may shift toward engagement** (buy back recall), because a missed hostile now carries a
  direct human cost that must be weighed against bird-precision.

**What ROE sets, and what it NEVER touches.** ROE sets **only** the acceptable false-positive
bound — *where* on the *measured* ROC the operating point sits. It is a **commander / doctrine
input, not an engineering constant.** Three invariants do **not** move with ROE — they are the
**fail-safe envelope the operating point slides *inside* of:**
1. **Affirmative positive-ID is always required.** ROE never authorizes "engage on doubt." It
   changes the acceptable false-positive *rate*; it never removes the requirement for an
   **affirmative powered-drone signature** (§2b-1, §0 item 5).
2. **Ambiguity always breaks off.** Genuine ambiguity — contested association, flock, stale/dead
   classifier, re-association inside the terminal — fails safe at **every** ROE setting (§5.2–5.3).
3. **Pre-bench, thresholds are UNSET ⇒ the interlock VETOES ALL.** ROE cannot conjure an operating
   point on a curve the Stage-0 bench has not measured (§0 item 6, §6.5). ROE tunes a *measured*
   ROC; it does not lower an *unmeasured* bar.

**Said once, unambiguously: this is NOT "engage on doubt."** Doubt always breaks off. ROE moves
the precision/recall trade **within** the fail-safe envelope; it can **never** switch off
positive-ID or the fail-safe veto. The most aggressive *lawful* ROE setting still demands an
affirmative hostile ID and still breaks off on ambiguity — it only accepts a higher (still
measured, still bounded, still audited) chance that a target which produced a genuine powered-drone
signature was in fact a bird. The knob is the *acceptable false-positive bound*; it is not a knob
on whether we must positively identify the target.

### 1.5.2 Operational mitigations — layered defense, not a perfect classifier

The technical interlock (§5) is **one layer, not the only layer.** The doctrine is explicitly
**layered — technical interlock + operational controls + human abort** — so that **no single
component, least of all the fragile few-pixel classifier, has to be perfect** for the system to be
safe. These mitigations are first-class defenses, not afterthoughts; their job is to **reduce the
reliance on a perfect classifier:**

- **(a) Range clearance — geographic + temporal bird deconfliction.** During real-world testing,
  operate **only where and when birds are excluded or deconflicted** (cleared range, closed
  airspace window, off-migration timing, live spotters). A classifier miss then **cannot harm
  wildlife during bring-up**, because there is no wildlife in the engagement volume to harm. This
  removes the bird-safety risk *operationally*, before the classifier is ever trusted to remove it
  *technically.*
- **(b) Human-on-the-loop with manual abort authority.** A human **can override and abort a running
  engagement at any time**, and at low autonomy levels **must positively authorize the commit**
  before it proceeds. This is the ultimate safety layer and the accepted **ethical and legal norm
  for lethal autonomy** — *meaningful human control*, the machine's autonomy bounded by a human who
  can always say stop. Manual abort commands the same fail-safe break-off path the interlock uses
  (§5.3): a human veto and a classifier veto exit through one door.
- **Bring-up protocol (builder mandate, 2026-07-08):** *test with **no birds present**, and be
  **ready to abort manually** the moment it starts acting wrong.* This is the standing rule for
  every real-world bring-up run until the classifier and interlock are bench-proven — **range
  clearance removes the target, human abort removes the runaway.**

The layers stack and cover each other: the technical interlock catches most; range clearance
removes the stakes during test; human abort catches whatever the machine gets wrong. **The design
does not bet a bird's life on any single one of them** — that is what "defense-in-depth" means
here, and it is why the residual fragility of the terminal classifier (below) is tolerable.

### 1.5.3 Ground-stereo is the PRIMARY discriminator; the onboard carries a committed decision + fast veto

The CONOPS role of each node, stated flatly (the mechanism is in §2b/§2c/§5 — this fixes the
**doctrine**: which node actually manufactures the positive-ID):

**The ground stereo rig MANUFACTURES the positive-ID BEFORE the interceptor commits.** It has the
four things the **0.41 s** onboard terminal structurally does **not:**
- **True size in absolute metres** — from stereo *range* (§2b-2). A monocular terminal frame
  physically cannot produce absolute size; mono size is a guess scaled by an *assumed* drone width.
- **The kinematic track time-series** — the **affirmative powered-drone signature** (sustained
  hover / vertical-climb / powered cruise a bird cannot hold), read from the *whole mid-course
  track*, not a single frame (§2b-1). This is the positive evidence that authorizes a commit.
- **Mid-course TIME** — seconds to accumulate a sustained N-of-M vote while every sensor is live,
  versus the ~6–12 frames a terminal ever gets.
- **Optional micro-Doppler radar** — the cleanest bird-vs-rotor physics of any modality, when the
  budget carries it (§2b-3).

**The onboard seeker carries the COMMITTED decision plus a fast veto** (§5.2b). It **never
originates the classification from cold** — it *holds* an already-manufactured PID and *revokes* it
fast on any bird-like evidence. **Ground manufactures; onboard carries and vetoes.** That division
is the whole architecture in one line, and it is why removing the strong ground channels from the
terminal (jamming) degrades the system to *veto-only*, never to *guess-and-engage*.

**The honest residual (already in §2b, §7 — the doctrine names what covers it).** Under jamming the
ground confirm is **gone at the terminal**, and a **small bird (0.3–0.5 m) defeats true-size**
(inside stereo σ_R ≈ 0.45 m at 100 m, §2b-2). That residual is real and **cannot be closed by the
classifier alone.** It is **exactly** what the operational mitigations (§1.5.2) and the fail-safe
veto (§5.2b) exist to cover: **range clearance** removes the bird from the test volume, **human
abort** stops a bad commit, and the **fast asymmetric positive-ID-to-hold veto** breaks off on a
single bird-like frame. The layered defense is the answer to the residual — **because there is no
perfect terminal classifier, and the doctrine does not pretend there is.**

---

## 2. The layered discrimination architecture (the crux, stated once)

**Bird rejection cannot depend on a link the threat model denies.** So PID must be
**manufactured mid-course while every sensor is live**, and the **onboard seeker must carry a
good-enough classifier through the terminal ALONE.** Three phases:

```
   MID-COURSE (link UP)          ARM GATE (the commit)          TERMINAL (link DENIED)
   ────────────────────          ─────────────────────          ──────────────────────
   fuse channels into ONE        latch ONLY IF ALL hold:        POSITIVE-ID-TO-HOLD,
   hostile-class posterior        (a) fused P(hostile)          not streak-to-abort.
   P(hostile):                        ≥ PID_HIGH                To HOLD the engage the
    • onboard appearance CNN      (b) onboard-ALONE             onboard must re-confirm
    • ground kinematic track          P(hostile)               hostile EVERY frame.
    • ground stereo TRUE SIZE         ≥ PID_ONBOARD_MIN         IMMEDIATE break-off on
    • optional radar µ-Doppler    (c) AFFIRMATIVE powered-      ANY of:
   on the ADR-0034 EKF track,         drone signature           • a gap in positive ID
   bound to a tracked OBJECT          (hover / vert-climb /     • 2-of-3 bird-like frames
   IDENTITY, not a track slot.        rotor µ-Dopp / powered    • one high-conf bird frame
                                      flight) — NOT "not        • track re-assoc / new tgt
   A carried prior is VETO-ONLY:      bird-like"               • contested association
   may RAISE suspicion /          (d) association UNCONTESTED   • stale / dead classifier
   lengthen confirmation,             (≤1 candidate in gate)   ⇒ break-off + standoff-widen.
   NEVER shorten N or lower       Fail ANY ⇒ BREAK OFF.        Engage REVERSIBLE to impact;
   the commit bar.                Makes onboard-alone           onboard must RE-EARN PID
                                  sufficiency a PRECONDITION    camera-only after handoff.
                                  of commit, not a hope.
```

This mirrors, on the **classification** channel, exactly what ADR-0034 already does on the
**state** channel: when the link drops, the ground sources' weight is driven to zero and the
estimate rides the camera. Bird rejection must degrade the *same* way — gracefully to
camera-only — or the jam-resistance thesis quietly leaks through the classifier's memory.

### 2a. Onboard seeker (Pi 5 + Hailo-8) — the terminal, jam-immune node

The core difficulty, stated plainly: **single-frame appearance is often insufficient at the
range where the engage decision is actually made.** At acquisition range a small quad and a bird
are *both* ~10×10 px: YOLOMG measures a drone at 100 m as exactly 10×10 px in 1080p; 42.18% of
ARD-100 targets are <12×12 px; BirDrone's smallest drone (7×5 px) and smallest bird (6×7 px) are
the same size ([ARD-100 / YOLOMG](https://arxiv.org/abs/2503.07115);
[BirDrone / YOLOBirDrone](https://arxiv.org/html/2601.08319v1)). **At that resolution there is
essentially no reliable appearance signal to separate a quad from a bird.** The published crop
classifiers that hit 93–99% (below) are measured on *resolved* targets (tens–hundreds of px) and
do **not** hold at the few-pixel range. The best concrete single-frame false-positive number,
YOLOBirDrone's **3.73% bird-as-drone on resolved crops** — ~1 in 27 — is already far too high to
gate a lethal, wildlife-safety-critical decision, and it is *effectively undefined* (likely much
worse) at 10×10 px. **Conclusion: appearance alone can never be the bird-reject gate.**

So the onboard seeker carries a **stack of cheap, camera-only discriminants** and treats the
whole stack as a *terminal veto*, not a from-cold authority:
1. **Appearance (CNN).** A MobileNet-class head on the detection ROI. Moderate, domain-fragile
   (§3) — the baseline vote, never the whole answer.
2. **Wing-flap vs rigid-body micro-motion — the camera's analog of micro-Doppler.** Birds flap
   at **~4–6 Hz** (Harris Hawk ~4, Eagle Owl ~5, Hawk Owl ~6 Hz — micro-Doppler measured,
   [Rahman & Robertson, *Sci. Rep.* 2018](https://www.nature.com/articles/s41598-018-35880-9));
   a multirotor is a rigid body with sub-pixel props. A ~14–30 Hz camera can Nyquist-sample a
   4–6 Hz flap with a short blob-area/aspect periodicity test. **But the same pixel wall applies:**
   a flapping silhouette is only observable once the bird spans enough pixels to *have* a
   silhouette — unavailable at first-detection range, and *gliding* birds show no flap. A useful
   **secondary** confirm at medium range for a lock already held, never a primary gate.
3. **Image-plane trajectory smoothness.** Drones fly smooth, constrained, near-rigid paths; birds
   flap-glide, wander, and thermal-soar. Features (mean speed, turn rate, curvature, wingbeat
   jitter) → a small RF/SVM/LSTM on the existing α-β/EKF track
   ([Kengeskanov et al. 2023, track-trajectory drone/bird](https://hal.science/hal-04101996)).
   Cheap; runs off own-state + bearing history; available onboard.
4. **Size-vs-looming consistency.** The known-size range estimate (`range ≈ fx·W_real/w_px`,
   seeker brief §3) *assumes* a drone width; cross-check it against the *looming rate* (box
   expansion → independent closing speed). A target whose looming-implied range disagrees with
   its size-implied range is the wrong physical size for a drone — a bird up close. Weak alone
   (15–30% range σ, seeker brief §3), a useful tie-breaker.

Each discriminant emits toward **one dedicated hostile-class posterior `P(hostile)`** (§2c),
NOT a detection-confidence scalar — a confidently-detected bird must drive `P(hostile)` *down*.

**Honest onboard verdict:** the seeker can carry a *good-enough terminal veto* from
appearance + flap-motion + trajectory + size-consistency, but every one is pixel-limited and
domain-fragile. It is **not** trustworthy enough to be the *sole* PID authority from cold — which
is precisely why the architecture confirms mid-course, then commits. **Two asymmetries make the
terminal veto safe despite that fragility:** (1) it operates *positive-ID-to-hold* — the engage
holds only while every frame keeps re-confirming hostile, so the fragile channel's *failure mode
is break-off, not engage*; and (2) none of these onboard channels is an **affirmative
powered-drone signature** on its own (a glide looks smooth/rigid; "not flapping" is not "is a
drone"), so the onboard veto can *revoke* a commit but the *affirmative* drone evidence that
authorized the commit had to be earned mid-course (§2b, item 5 of §0). Absence of bird-like
evidence is never treated as presence of a hostile.

### 2b. Ground stereo rig (Jetson Orin NX) — the heavy classifier + what onboard-mono CANNOT get

The ground node has what the terminal lacks: **time** (the whole mid-course), **compute** (no
power/thermal limit), and **two physical measurements a monocular terminal frame cannot produce.**
Bird-rejection is *structurally* a ground-node job — ADR-0019 already re-attributed it AWAY from
thermal (birds are warm-blooded, so LWIR does not separate them) and TOWARD kinematics + radar.

1. **Kinematic track classification — the ground's core lever, and the home of the AFFIRMATIVE
   drone signature.** Birds and multirotors separate in the *distributions* of speed,
   acceleration, curvature and turn-rate along a track, not in any single sample. Published track
   classifiers report ~99% on clean/simulated tracks and **~91–95% (Random Forest, hybrid
   features) on real surveillance-radar tracks** (NATO STO-MP-MSG-SET-183; Liu & Xu, IEEE 2021 —
   take ~99% as an optimistic ceiling, ~91–95% as the field-representative EXPECTED tier per the
   worse-than-ideal mandate). **The one feature the ARM gate must POSITIVELY observe is a
   powered-flight signature that a bird physically cannot produce: sustained hover, a vertical
   climb, or a sustained powered cruise that no glide/flap-glide can hold.** This is the
   mitigation the red-team demanded (§0 item 5): the ARM gate must see an *affirmative* drone
   signature, **never merely "the track failed to look bird-like."** The distinction is
   load-bearing because the dangerous adversarial case — a bird in a **straight glide** — shows
   **low jerk and smooth curvature that look drone-like**, and *never hovering is not the same as
   being a drone*. Supporting features (mean **and variance / coefficient of variation** of
   velocity/acceleration per axis, path curvature, turn-rate) *raise or lower* `P(hostile)` but
   do not themselves authorize a commit; **speed alone is weak** (modeled bird cruise ~10–25 m/s
   overlaps the drone band). No observed powered-flight signature ⇒ `P(hostile)` stays below the
   ARM bar ⇒ **fail safe / break off.** This reuses the velocity track the rig must already emit
   (ADR-0015's velocity-emission was the #1 Pk lever) — a near-free ground-node add that never
   touches the comms-denied terminal thesis.
2. **Stereo TRUE SIZE — a strong passive discriminant AGAINST LARGE birds ONLY, which
   onboard-mono cannot replicate.** Stereo gives *range*, so angular size → **absolute**
   body/wingspan in metres (ADR-0017: σ_R ≈ 0.45 m at 100 m, cross-range ≈ 1 cm at 100 m). A
   0.3 m rigid quad and a **1.0–1.5 m-wingspan raptor** at 40 m *are* cleanly separable — a
   *physics* measurement, not a learned appearance, so it does **not** suffer the 93→66% domain
   drop, and avian prior art measured wingspan to **3.7% error even at 1.1 km**
   ([US Patent 11,544,490]). **But — and the original draft oversold this as "trivially
   separable," which is FALSE for the case that matters —** true-size only works when the size
   *gap* exceeds the range error. At 100 m, σ_R ≈ 0.45 m means the along-range measurement
   **cannot** separate a **0.3–0.5 m small bird** (swift, starling, small falcon) from a 0.3 m
   quad; the two are the *same physical size within the measurement noise*. So true-size is a
   discriminant against **large** birds, not **small** ones — and small birds are exactly the
   hard, non-safe case. Two honesty caveats compound it: stereo range error grows as R²
   (ADR-0017), weakening the gate further out than ~50–160 m; and the gate is *passive* (a
   small non-flapping target simply "could be a small drone"). **Conclusion: size may CONVICT a
   large bird, but its silence about a small target is not evidence of a drone — pair it with an
   AFFIRMATIVE powered-drone signature (below), never rely on either alone, and NEVER let
   "size didn't rule it out" stand in for a positive drone ID.**
3. **Optional radar micro-Doppler — the all-weather bird-truth layer, budget-gated.** The physics
   is the cleanest of any modality: a rigid rotating prop throws a **wide, symmetric micro-Doppler
   spread** with periodic rotor flashes / HERM lines (~20–40 dB below the body return), a bird
   gives only **narrow periodic flashes at its ~4–6 Hz wingbeat** (~0–10 dB below body) — measured
   at K-band 24 GHz and W-band 94 GHz ([Rahman & Robertson 2018](https://www.nature.com/articles/s41598-018-35880-9)).
   Reported: SVM ~**96% drone-vs-bird** on micro-Doppler spectrograms
   ([MDPI *Signals* 2023](https://www.mdpi.com/2624-6120/4/2/18)); a TI IWR6843 CNN 95.7–98.8%.
   **But** per ADR-0019 addendum, on a 0.01–0.1 m² small-drone RCS a ~$283 TI IWR6843 only reaches
   **~25–80 m** with coarse ~15° azimuth; real ~1 km cueing radar is $50k+ (Echodyne/Robin). So
   radar is a **confirm layer when available, never a dependency** — and note it *also* lives on
   the ground and cannot confirm through the jammer either.

**The comms-denied crux made explicit:** the ground node is where *high-confidence* PID is
manufactured (true-size *against large birds* + an **affirmative powered-flight kinematic
signature** + optional µ-Doppler — physics-based, but note each has the coverage limits above, so
they must AGREE, not be OR-ed), **but its confidence is only usable before the link is cut.** The architecture must therefore *transfer a
committed decision, not stream a live one* — and the onboard seeker must independently re-confirm
hostile in the terminal to hold the engage. A single terminal frame structurally cannot make the
bird/drone call (no time-series, no true size, no micro-Doppler), so the onboard seeker only *holds
and vetoes* an already-classified lock; it never *originates* the classification from cold.

### 2c. The fusion — ONE hostile-class posterior `P(hostile)`, riding on the ADR-0034 EKF

ADR-0034's EKF fuses *state* (position/velocity), weighting each source by live covariance and
rejecting outliers with a chi-square innovation gate. Classification runs as a **second,
decision-level channel on the same track**, and its output is a **single dedicated hostile-class
posterior `P(hostile)`** — the *only* number the interlock ever reads.

**The confidence contract (red-team fix #2 — the interlock must NOT reuse `decision_margin`).**
`decision_margin` is **detection-presence / decode confidence** — it answers *"is something
clearly there?"* and a crisply-imaged bird scores it **HIGH**. `P(hostile)` answers a completely
different question — *"is that clearly-seen thing a hostile UAS?"* — and for a confident bird it
is **LOW**. These two confidences are **orthogonal and must never share a slot**: gating on
`decision_margin` would *engage the well-detected bird*. So the classification channel emits
`P(hostile)` as a **distinct field in the measurement/classification contract**, alongside (never
overwriting) the existing `decision_margin` detection scalar. *(This doc only specifies the field;
it does not edit the seeker `Measurement` dataclass — the seeker worker owns that file.)*

- Each source emits a per-frame hostile likelihood: `L_appear` (onboard CNN), `L_kin`
  (ground/onboard trajectory + the affirmative powered-flight signature, §2b-1), `L_size`
  (ground stereo true-size — informative only when the size gap clears σ_R, §2b-2), `L_uD`
  (radar µ-Doppler, if present). These combine into `P(hostile)`.
- Combine with a **decision-level rule** — a Bayesian **log-likelihood-ratio sum** (the honest,
  interpretable, monotone default) or **Dempster–Shafer** when sources conflict / are uncertain
  (DS is built for conflicting-sensor, no-prior fusion and reaches ~99%-class target-recognition
  in the literature; [DS decision-fusion review, *Sensors* 2019](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6865203/)).
  When two independent heads exist, **AND them, never OR** (OR maximizes false positives; AND
  demands agreement — the precision-first choice).
- **Bind the hostile streak to a tracked OBJECT IDENTITY, not the EKF track slot (red-team fix
  #3).** An EKF track *slot* can silently re-associate onto a different physical object — a bird
  crossing behind the drone, a flock member swapped in — while carrying the accumulated
  hostile-streak of the *previous* object. That is a fail-unsafe stale streak: the slot says
  "confirmed hostile" about a target it is no longer looking at. So the classification streak is
  keyed to a **continuously-tracked object identity**, and on **any re-association, ID switch, or
  detection gap the streak RESETS and PID must be re-earned from scratch.** Inside terminal, a
  re-association / fresh-target-association event is not merely a reset — it is itself a
  **mandatory break-off** (0.41 s is too short to re-classify a swapped target; §5.2).
- **Contested association ⇒ ambiguity ⇒ veto (red-team fix #8).** If more than one candidate
  falls in the association gate, or a merge/split event occurred recently, the identity of "the
  target" is ambiguous and **engage is forbidden** — break off. Flocks and crossing birds are the
  realistic source of this, so the sim plan exercises it (§6.3).
- **This inherits the EKF's adaptivity for free (the elegant part).** The classification channel
  uses the *same track association and the same chi-square gate*: a ground measurement that fails
  the state gate is not fed to the classifier either. And when the link drops, the ground
  likelihoods simply stop arriving — `P(hostile)` rides `L_appear` (+ onboard `L_kin`, `L_size`)
  alone, exactly as the EKF state estimate rides the camera. **Graceful degradation to camera-only
  falls out of the estimator; you do not hand-code the fallback if-statement** — but note (§5.4)
  that this graceful degradation is VETO-preserving: it may only *remove* the ground's engage
  support, never carry a ground engage-authorization past the handoff.

---

## 3. Real-hardware feasibility — what runs where (cited)

**The headline finding: the classifier is NOT the compute bottleneck — pixels-on-target,
domain fragility, and thermals are.** No model on any hardware separates a bird from a quad at
10×10 px, so the discrimination budget is spent on *temporal accumulation + ground-cue fusion*,
not on a bigger onboard net.

| Node / function | Runs here | Latency / throughput | Power / size | Cited |
|---|---|---|---|---|
| **Onboard detector** (NanoDet-Plus-m or YOLOv8n) | Pi 5 + Hailo-8 | detect-then-track ~**35 fps / ~29 ms** end-to-end; Hailo raw ceiling YOLOv8n ~431 fps @640² (batch1) / 136.7 fps (batch8) | Hailo-8 **26 TOPS @ ~2.5 W typ**, M.2 thermal-rated **7 W max** | ADR-0016; [Seeed/Hailo bench](https://wiki.seeedstudio.com/benchmark_of_multistream_inference_on_raspberrypi5_with_hailo8/); [Hailo-8 datasheet](https://hailo.ai/products/ai-accelerators/hailo-8-m2-ai-acceleration-module/) |
| **Onboard drone/bird classifier HEAD** (MobileNetV3-Large-class) | Pi 5 + Hailo-8 | **<1 GFLOP on the ROI crop**; small add, does **not** move the 35-fps budget; ~93% acc on *resolved* crops | shares the 26 TOPS; negligible extra | [MobileNetV3 bird-vs-drone 93%](https://www.mdpi.com/2673-2688/6/3/57) |
| **Onboard wingbeat / trajectory test** | Pi 5 CPU | short blob time-series; near-free | negligible | Rahman 2018; Kengeskanov 2023 |
| **Ground detector + heavy classifier** (YOLOv8s two-stream + LSTM/RF) | Jetson Orin NX | ~**15–20 fps/stream** honest under TensorRT; Kalman track ≥10 Hz | no flight power/thermal limit | ADR-0016 |
| **Ground stereo true-size** | Jetson Orin NX | arithmetic once disparity → range | free (reuses stereo) | ADR-0017 |
| **Optional radar µ-Doppler** | ground peripheral | CNN 95.7–98.8% on-device (Nano-class) | TI IWR6843 **~$283**, ~25–80 m on small drone | ADR-0019 addendum; [MDPI *Signals* 2023](https://www.mdpi.com/2624-6120/4/2/18) |

**The three honest onboard binders (not compute):**
- **Pixels-on-target.** NN classification "degrades severely below ~10×10 px"
  ([FLIR C-UAS](https://oem.flir.com/learn/discover/thermal-infrared-sensor-design-considerations-for-counter-uas-defense/));
  Johnson's criteria put *recognition* ("drone not bird") at ~8 px (ground_modality.md §1). A
  drone at 100 m is ~10×10 px in 1080p. **This sets the ARM range** — the same acquisition-range
  lever that dominates the miss (ADR-0023/0024).
- **Domain fragility.** A classifier scoring 93.55% on known scenes fell to **66% AP on unknown
  scenes** (ground_modality.md §3;
  [drone-warfare EO/IR](https://drone-warfare.com/counter-uas/eo-ir-detection/)). This is *why*
  onboard-alone must be *confirmed* mid-course, not trusted from cold.
- **Thermal/power on a vibrating FPV airframe.** 2.5 W typ is trivial, but sustained inference on
  a poorly-ventilated 2.5" airframe pushes toward the 7 W envelope and the Pi 5 throttles under
  sustained load — a **bench-measurable** item (§6), not a spec to assume.

**License note (portfolio-relevant, R6):** YOLOv8/YOLO11 are **AGPL-3.0** (network copyleft) — a
risk for a public repo. **NanoDet-Plus-m (Apache-2.0, 1.17 M params / 1.2 MB int8)** is the
license-clean nano detector; **MobileNetV3-Large / MobileNetV2** are the license-clean crop
classifiers. Prefer these over AGPL YOLO for the public portfolio.

---

## 4. Datasets + candidate models to actually build it (with licenses)

**The single most important dataset fact for a safety-critical bird-reject task: most
drone-detection datasets have NO labeled bird class.** The canonical Drone-vs-Bird set has birds
*in scene* but historically *not* bbox-annotated — you cannot train a positive bird-reject
classifier on drone-only labels (multiple studies had to hand-annotate a few videos to get bird
boxes). Train only on the sets that carry a real bird class.

### Datasets

| Dataset | Modality / size | Bird class? | License | Note |
|---|---|---|---|---|
| **YOLOv7 Segmented Drone-vs-Bird** (Mendeley) | RGB, **20,925 imgs** (8,451 bird / 12,474 drone), 640² | **YES** | **CC BY 4.0** ✅ (attribution only) | **Best license fit** for the sim's hostile/bird classifier. [DOI 10.17632/6ghdz52pd7.5](https://data.mendeley.com/datasets/6ghdz52pd7/5) |
| **Svanström / Halmstad Multi-Sensor** | IR+RGB+audio, **203,328 frames** | **YES** (drone/bird/airplane/heli) | Open (Zenodo, CC-family) ✅ | Best *bird-inclusive multi-class real* set; ranges to 200 m. [PMC8573135](https://pmc.ncbi.nlm.nih.gov/articles/PMC8573135/) |
| **BirDrone / YOLOBirDrone** | RGB, 11,495 imgs, 13,881 drone + 15,867 bird ann. | **YES** | GitHub, license unstated (confirm) | Smallest drone 7×5 px / bird 6×7 px; **FP 3.73%**, mAP@0.5 0.948. [arXiv 2601.08319](https://arxiv.org/html/2601.08319v1) |
| **SimD3** (synthetic, UE5, Jan 2026) | Synthetic RGB, 6-cam 360 rig | **YES** — 8 bird-species animated hard-negative distractors | Synthetic (check repo) | Built *because* "drones are frequently confused with birds"; augments rare bird negatives. [arXiv 2601.14742](https://arxiv.org/abs/2601.14742) |
| **Drone-vs-Bird / WOSDETC** | RGB video, 77 seq, avg target ~34×23 px | Birds in-scene, **largely UN-annotated** | Data-Usage-Agreement (not open) | THE reference set, weak as a bird-label source out of the box. [github.com/wosdetc/challenge](https://github.com/wosdetc/challenge) |
| **ARD-100 / USC-Drone / MAV-VID / Anti-UAV410** | RGB or IR, drone-only | **No bird class** | ARD/USC permissive; Anti-UAV MIT | Drone imagery / few-pixel reality; ARD-100 42% <12×12 px. [YOLOMG](https://github.com/Irisky123/YOLOMG) |
| **DIAT-µSAT** (radar) | 4,849 micro-Doppler images (incl. bionic bird) | drone + bionic-bird | Open academic | Closest open µ-Doppler set; exact band unconfirmed. [IEEE DataPort](https://ieee-dataport.org/documents/diat-msat-micro-doppler-signature-dataset-small-unmanned-aerial-vehicle-suav) |
| **LAT-BirdDrone** | low-altitude bird+drone **trajectories** | YES (kinematic) | ScienceDirect data article | For the *kinematic* classifier (§2b). [PMC12769828](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12769828/) |

> *"Halcyon"* could **not** be verified as a distinct published drone-vs-bird dataset — drop it or
> supply a citation.

### Candidate models

- **Detector (license-clean):** **NanoDet-Plus-m** — 1.17 M params / 1.2 MB int8, 27.0 mAP@320,
  **Apache-2.0** ([RangiLyu/nanodet](https://github.com/RangiLyu/nanodet)). Fine-tune required (a
  COCO "drone" class does not fire on a small quad vs sky).
- **Crop classifier (license-clean, edge):** **MobileNetV3-Large ~93%** / **MobileNetV2
  transfer-learning** on *resolved* crops. EfficientNet-B6 (98.12%) / Xception (99.18%) are heavy
  **ceiling references only** — too big for the Pi.
- **Single-stage both-class reference:** YOLOBirDrone (precision 0.949, recall 0.917, mAP@0.5
  0.948, **FP 3.73% / FN 11.61%**) — the concrete numbers that argue against single-frame gating.
- **Kinematic classifier (method, no license constraint):** Random Forest / SVM / LSTM on track
  features (~91–95% RF on real tracks; NATO STO-183; Kengeskanov 2023).
- **Micro-Doppler classifier (method):** SVM / spectrogram-CNN (~96% / 95.7–98.8%; Rahman 2018;
  MDPI *Signals* 2023).

**Recommended license-clean training trio:** YOLOv7-Segmented (CC BY 4.0) + Halmstad
(open, multi-class) + SimD3 (synthetic hard bird negatives), paired with NanoDet-Plus +
MobileNetV3 — avoid AGPL YOLO for the public repo.

---

## 5. The engage INTERLOCK, the safety metric, and fail-safe behavior

### 5.1 Added AND-clauses on the existing gate — no new lethal path

The interlock is a **hostile-UAS precondition on the existing M4 state machine**, not a new phase.
`scripts/m4_intercept.py`'s `run_acquire_and_engage` (from line 1415) transitions ACQUIRE → ENGAGE
through a conjunction (lines **1747–1749**) that today confirms only *geometry*:

```
consecutive_fresh >= ACQUIRE_MIN_DETECTIONS   (= 10, line 274)
AND acquire_elapsed   >= ACQUIRE_MIN_S         (= 1.0 s, line 275)
AND centered_streak   >= ACQUIRE_CENTERED_STREAK (= 6, ~0.4 s, line 281)
```

The interlock adds a **positive-hostile-ID precondition** — a structural sibling of the existing
acquire gate. It reads the **dedicated hostile posterior `P(hostile)`** (§2c), **never**
`decision_margin`:

```
AND hostile_streak    >= CLASSIFY_MIN_STREAK   (N-of-M sustained P(hostile), bound to OBJECT ID)
AND fused_P_hostile   >= PID_HIGH              (fused hostile posterior floor)
AND onboard_P_hostile >= PID_ONBOARD_MIN       (onboard-ALONE posterior floor — sufficiency
                                                precondition for the camera-only terminal)
AND drone_signature_observed                   (AFFIRMATIVE powered-flight signature, §2b-1)
AND association_uncontested                    (≤1 candidate in the gate, §2c)
```

**Illegal-state-unrepresentable:** because ENGAGE spawns the mover and commits closing speed, and
it is reached ONLY through this conjunction, *"no positively-classified hostile ⇒ no ENGAGE"* is
enforced by construction — the same way the repo already makes cheating structurally impossible
(ADR-0008/0010 honesty lineage).

**The confidence signal is a NEW dedicated field — do NOT reuse `decision_margin` (red-team fix
#2).** The original draft proposed riding the classifier confidence in on the seeker brief's
`decision_margin` slot ("detector confidence scalar," seeker_design_brief.md:47). **That is a
fail-unsafe bug:** `decision_margin` is *detection-presence / decode* confidence and a clearly-seen
bird scores it HIGH — gating on it would engage the bird. Detection confidence and classification
confidence are **orthogonal** (§2c). The interlock therefore requires a **distinct `P(hostile)`
field** in the measurement/classification contract, carried *alongside* `decision_margin`, not in
it. The plumbing note is corrected: this **does** add one new field (a hostile-class posterior);
its bandwidth is a single scalar per detection, trivial, but it is not a repurpose of an existing
slot. *(This doc specifies the field only; it does not edit `m4_intercept.py` or the seeker
`Measurement` dataclass — the seeker worker owns those files.)*

### 5.2 Temporal confirmation is ASYMMETRIC — slow to commit, fast to veto

The original draft used **one** timer — a 12-of-15 (~0.8–1.0 s) sustained-hostile streak — for
both committing *and* aborting. **The red-team showed that is fail-unsafe in the terminal
(fix #1):** the measured camera-only terminal lasts **0.41 s median** (~6–12 frames at 14–20 Hz;
ADR-0023, terminal_diagnosis.md item 6), so a ~0.8–1.0 s "streak-to-abort" is *longer than the
entire window it must fire inside*, and the old break-off exit (`LOST_TAG_ABORT_S`=5 s) fires
long after CPA. A slow symmetric streak cannot protect a bird that appears inside the terminal.
The fix is to make confirmation **asymmetric: slow, N-of-M, and sustained to COMMIT; instant and
one-sided to VETO.**

**(a) MID-COURSE, to COMMIT — slow N-of-M, where there is time.** A single-frame classification
on a few-pixel, motion-blurred body is noisy (§2a). The parent design is explicitly
**detect-then-track, not single-frame YOLO** (ADR-0013), which lifted tiny-target recall
**0.405 → 0.861 at 0.981 precision**. So the ARM gate requires `P(hostile)` above threshold for
**N of the last M classifier updates** (proposal M≈15, N≈12, ≈0.8–1.0 s at the ~14 Hz cadence —
parallel to the existing `ACQUIRE_MIN_DETECTIONS=10` / `ACQUIRE_CENTERED_STREAK=6` timers so the
ID vote and the geometry vote converge together). This streak is **bound to the tracked object
identity (§2c)** and **resets on any re-association / ID switch / gap.** A sustained vote both
suppresses per-frame flicker and demands the object *behave* like a drone over time. **(These
N/M numbers are placeholders — UNSET until the Stage-0 bench; see §5.2c and §6.)**

**(b) TERMINAL, to HOLD — positive-ID-to-hold + fast one-sided veto.** After handoff the logic
**inverts**. The engage is *held* only while the onboard keeps **continuously re-confirming
hostile every frame**; the *default* is break-off and each fresh positive frame merely buys the
next few frames of hold. Break-off is **immediate** on ANY of:

- a **single high-confidence bird frame** (`P(hostile)` collapses on one clear frame), OR
- **2-of-3 recent frames bird-like** (suppresses one-frame flicker without waiting a streak), OR
- **any gap in positive hostile confirmation** (no fresh hostile frame within the liveness
  window — silence is a bird vote, §5.2c), OR
- **any track re-association / fresh-target association / ID switch** (§2c — 0.41 s is too short
  to re-classify a swapped target; a re-assoc inside terminal is a mandatory break-off, not a
  streak reset), OR
- **any contested association / merge-split event** (§2c, §6.3).

Break-off is **not passive**: it commands the existing `BREAKOFF` path (climb + peel-off) **and
widens the standoff** to actively hold off (§5.3). Because a bird's flap-glide-wander cannot
*continuously* re-present a hostile signature the way a rigid powered quad can, positive-ID-to-hold
turns the classifier's fragility into a *safe* failure: when it is unsure, the engage lapses.

### 5.2c Fail-safe classifier defaults — stale / dead == bird (red-team fix #7)

The interlock is only as safe as its behavior when the classifier says **nothing**. By
construction:

- **Absent, stale (older than 1–2 frames / the liveness window), or timed-out classification ⇒
  `P(hostile)` = 0 ⇒ break-off.** There is no "carry the last value" path; a missing hostile vote
  is a bird vote.
- **`hostile_streak` and `hostile_conf` initialize to NON-HOSTILE at ENGAGE entry** — the engage
  begins in the "not confirmed" state and must be *actively* held, never inheriting stale
  confidence from ACQUIRE.
- **A classifier-liveness watchdog** treats a stalled / crashed / silent classification head
  **identically to a "bird" vote** — a dead head can never hold an engage. (This mirrors the
  existing detection-staleness gate, seeker_design_brief.md:88, `MEAS_STALE_S`, extended to the
  classification field.)

### 5.3 Fail-safe branches — all roads lead to break-off, never to engage

| Situation | What could go wrong | Design response (fail SAFE) |
|---|---|---|
| Bird enters at long range | onboard CNN too few px, guesses "drone" | ARM needs ground true-size + kinematics AND onboard-min; a lone onboard guess cannot commit |
| **No affirmative powered-drone signature** (small bird, straight glide — low jerk, never hovers) | size + "not bird-like" mistaken for "is a drone" | ARM requires an **observed** hover / vert-climb / rotor µ-Doppler / sustained powered-flight signature (§2b-1). **"Failed to look bird-like" ≠ drone ⇒ break off** |
| Ground says drone, onboard disagrees at ARM | committing to an onboard-only terminal the onboard can't hold | ARM requires BOTH; disagreement ⇒ break off |
| Low posterior (`P(hostile)` < `PID_*`) | flicker | streak resets, stay in ACQUIRE; unresolved by `ACQUIRE_TIMEOUT_S` (=20, line 282) ⇒ existing abort. **Doubt = do-not-engage** |
| **Classification stale / absent / timed-out / head crashed** | engage held on a dead or silent classifier | `P(hostile)`=0 by construction; liveness watchdog treats silence as a **bird** vote ⇒ break off (§5.2c). No "carry last value" |
| Link cut mid-course before PID confirmed | no confirmed target, onboard alone from cold | do NOT commit on onboard-alone-from-cold; loiter/abort — bird rejection may not depend on the link |
| Two heads disagree (appearance vs motion) | OR-ing maximizes false positives | **AND** the heads; disagreement ⇒ veto |
| **Track re-association / ID switch / fresh-target inside terminal** | stale hostile streak carried onto a swapped object (a bird) | streak is bound to OBJECT IDENTITY (§2c); inside terminal a re-assoc is a **mandatory immediate break-off** — 0.41 s is too short to re-classify |
| **Flock / contested association / recent merge-split** | ambiguous "which one is the target" | >1 candidate in the gate ⇒ ambiguity ⇒ **break off**; engage forbidden while association contested (§2c, §6.3) |
| Committed target turns bird-like in terminal | irreversible engage on a bird | **positive-ID-to-hold** (§5.2b): a single high-conf bird frame, 2-of-3 bird-like frames, or any gap in positive confirmation **immediately** triggers the EXISTING `BREAKOFF` maneuver (climb `CLIMB_V`=1.0 line 302) — **fired by the classifier veto, NOT by waiting out `LOST_TAG_ABORT_S`=5 s** (that 5 s exit is far longer than the 0.41 s window). Bird-rejection **reuses the break-off maneuver — no new lethal path**, but on a fast asymmetric trigger |
| Bird actively identified (any bird-like frame in terminal) | passive fail-to-engage insufficient | hard veto **and widen standoff** — actively hold off |
| Radar unavailable (budget/weather) | lose one affirmative-signature source | radar is confirm-only; the affirmative signature can still come from observed hover/vert-climb kinematics; disclose reduced margin — but **no affirmative signature at all ⇒ no commit** |

Note the terminal camera-dropout that is the project's systemic limiter (ADR-0023) *already* forces
break-off — so classification-loss and detection-loss share **one** safe exit. The key correction
the red-team forced: that exit must be reachable **on the 0.41 s terminal timescale** (the fast
classifier veto), not only via the 5 s lost-tag timeout.

### 5.4 The comms-denied classification honesty boundary (extends the ADR-0034 audit)

The interlock must respect the same honesty boundary the fusion capstone does. ADR-0034's
non-negotiable — `cue_reads_post_handoff = 0`, no cue-tainted EKF state/covariance survives handoff
— **extends verbatim to the classification label:**

- **The carried prior is VETO-ONLY and ASYMMETRIC (red-team fix #4).** The original draft let the
  warm ground ID "bias the prior / shorten N." **That is an unsafe direction:** a ground label that
  *shortens* the required onboard confirmation lets a ground miss (a bird the ground mis-called
  hostile) *accelerate* a terminal commit through a weaker onboard check — exactly the leak the
  thesis forbids. So the prior is one-sided. The ground rig (richer classifier: 2 m stereo, Orin
  compute, optional µ-Doppler) may **WARM/prime the ID pre-handoff**, but that prior may **only
  RAISE suspicion — LENGTHEN the required onboard confirmation, raise the onboard bar, or abort
  outright.** It may **NEVER shorten N, lower `PID_ONBOARD_MIN`, or otherwise move the onboard bar
  toward engage.** A ground "looks hostile" does **not** buy the onboard any confirmation credit; a
  ground "looks like a bird" **can** abort. **The prior can veto, never accelerate a commit.**
- **After handoff the onboard seeker must independently RE-EARN hostile ID, camera-only, to hold
  the engage,** and the committed PID must remain *revocable* by onboard evidence to impact.
  Concretely: add a **`classification_reads_post_handoff = 0`** audit analog to the existing cue
  audit (audit_targets.md §A; scripts/check_s2.sh). Its two clauses: (1) after handoff the engage
  decision must trace to *onboard-camera* classifier evidence — no ground-derived label/likelihood
  may *lock out* the onboard veto or *substitute* for onboard re-confirmation; and (2) the only
  legal cross-handoff influence of the ground label is **toward-safe** (raise suspicion / lengthen
  confirmation / abort), so the audit checks that no ground input ever *lowered* the onboard commit
  bar. This is the honest, tightened reading of the audit: **the ground may bias toward break-off,
  never toward engage.** Otherwise the jam-resistance thesis leaks through the classifier's memory
  exactly as it would leak through the filter's. **The same covariance/gate discipline that governs
  *where* the target is also governs *what* it is** — a strong portfolio point.

---

## 6. Sim validation plan — bird decoy + drones-and-birds Monte-Carlo (gated)

### 6.1 Gating (be honest about sequencing)

This is gated behind **(a) the M5 finish** (a Monte-Carlo batch is running; the standing rule is
ONE sim at a time, batches at idle load — MEMORY / ADR-0015), **(b) the markerless-seeker
classifier existing** — the interlock is a no-op until there is a `hostile`/`bird` class (today's
AprilTag has no notion of "hostile") — and **(c) the Stage-0 bench having produced the
`P_correct(R)` curve that SETS the operating-point thresholds.** Until (c), the thresholds are
UNSET and **the interlock defaults to VETO-ALL (engages nothing)**; the sim can validate the
*wiring* but must not be used to *choose* `PID_HIGH` / `PID_ONBOARD_MIN` / N-of-M (§6.4, red-team
fix #6). It sits naturally in the ADR-0034 capstone ladder (seeker + EKF + fusion), adding a
*classification* axis to the existing fusion × tracker × seeker cross.

### 6.2 Bird decoy model — a structural twin of `models/fpv_target_markerless/`

`models/fpv_target_markerless/model.sdf` is a static, collision-free, purely-visual quad silhouette
driven kinematically by `scripts/m4_target_mover.py`. **Author a new sibling `models/bird_decoy/`**
(do not edit the existing model). **Two decoy tiers, and only the second licenses a bird-reject
claim (red-team fix #6):**

- **PLUMBING-tier decoy (minimum).** A `gt_type=bird` label routed through the classification
  contract so the interlock's *wiring* can be exercised: does a "bird" verdict route to break-off,
  does the streak reset on re-association, does a stale head force break-off? A clean primitive body
  with **no flap model, no motion blur, no airframe vibration exercises NONE of the actual
  wingbeat / appearance / blur discriminants** — so a green result here is **interlock-wiring
  validated, not bird-rejection validated.** State that verbatim in any writeup.
- **WORST-tier decoy (required before ANY bird-rejection claim).** Must add the realism that
  actually stresses the discriminants:
  - **Matched pixel footprint:** small elongated body + two flat wing planes (no 4 arms / prop-discs,
    no red "enemy" motor accents), **~0.3–0.5 m span** so at range it subtends *similar pixel
    counts* to the FPV target — discrimination must be *earned by shape/motion*, not gifted by a
    size gap (and per §2b-2 a 0.3–0.5 m span is inside stereo σ_R at 100 m, the genuinely hard case).
  - **A flapping actor:** a Gazebo **actor** (COLLADA skeleton animation) or scripted wing-plane
    dihedral oscillation ([Gazebo Harmonic actors](https://gazebosim.org/docs/harmonic/actors/)),
    so the wingbeat discriminant has something to measure.
  - **The adversarial straight-glide case:** a **non-flapping gliding** bird (low jerk, smooth,
    never hovers) that *looks drone-like* — the case §2b/§0-item-5 says size + "not bird-like"
    cannot resolve and that must be caught by *requiring an affirmative drone signature*, not by
    the decoy conveniently flapping.
- **Kinematics (the real discriminant, sourced):** pigeon wingbeat ≈ **5.48 Hz with ~20 mm body
  bob** ([PMC6228479](https://pmc.ncbi.nlm.nih.gov/articles/PMC6228479/)), 5–8 Hz typical, curved
  undulating slower paths vs a powered quad's smooth near-rigid ≥2 m/s run. **The existing mover
  already supports the trajectory side** — `m4_target_mover.py` has `line` / `weave`
  (`--weave-period-s`, `--weave-lat-speed`) / `jink` schedules; a bird trajectory is a slower,
  curved, oscillating waypoint set, matching the "every realism knob maps to a bench-measurable
  quantity" mandate. **Even the WORST-tier decoy remains an UPPER BOUND** — Gazebo has no real
  motion blur (ADR-0014) or feather-level appearance — so the flap/appearance/blur discriminants
  are ultimately a **bench** claim (§6.5), never a sim one.

### 6.3 Monte-Carlo with BOTH object classes

Extend the M5 batch harness (`scripts/mc_batch.sh` / `scripts/mc_analyze.py` — reference, do not
edit) with scenes covering **four situations**, not just one-drone-one-bird:
1. **Single drone** — correct-engage baseline.
2. **Single bird** (both flapping and the adversarial straight-glide, §6.2) — false-engage baseline;
   bird-only runs are the "there is no valid target" case.
3. **Mixed drone + bird(s) in scene** — the discriminator must engage the drone and veto the bird.
4. **Flock / multi-target / merged-track scenes (red-team fix #8):** several birds, or a bird
   crossing the drone's track so the association gate holds **>1 candidate**, plus deliberate
   **merge/split** events. Expected correct behavior is **break-off on the contested association**
   (§2c, §5.3) — engage is forbidden while identity is ambiguous. This scene type also exercises the
   **object-identity streak binding** (§2c): a re-association must reset the streak, and inside
   terminal must force break-off.

Sweep seeds/paths/speeds per the existing **paired-seed n≥8** discipline. Metrics:
- **Correct-engage rate on drones** (recall — allowed < 100%; missed drones are the acceptable error).
- **False-engage rate on birds → target 0** — reported k/N + one-sided upper CI (e.g. "0/200, 95%
  upper bound < 1.5%"). **Read this as a PLUMBING / interlock-wiring number and an explicit UPPER
  BOUND, never as "bird rejection validated"** (red-team fix #6): the clean sim decoy cannot exercise
  the flap/appearance/blur discriminants (§6.2), so a 0/N here means "the wiring vetoes when told to,"
  not "the classifier will reject a real bird."
- **Confusion matrix** {drone, bird} × {engaged, vetoed/broke-off}, plus a **DIAGNOSTIC ROC / PR
  sweep of `P(hostile)` thresholds** to *visualize* where the zero-false-positive region sits and to
  *expose the recall paid*. **This sweep is expository ONLY — it MUST NOT be used to COMMIT
  `PID_HIGH` / `PID_ONBOARD_MIN` / N-of-M** (fix #6): operating-point thresholds come from the
  measured `P_correct(R)` Stage-0 bench (§6.5), not the sim ROC, and until that bench runs the
  interlock defaults to VETO-ALL.
- **Honesty guard:** the engage decision reads the *classifier `P(hostile)` output only*; `gt_*`
  object type is scoring/labeling ONLY (which row of the confusion matrix), **never** an interlock
  input — the same `gt_*`-is-scoring-only boundary the project already enforces. The sim is an
  **UPPER BOUND** (clean synthetic bird vs the messy real sky).

### 6.4 Pass/fail gate (scripted, per the milestone convention)

A `check_bird_reject.sh`-style script (new; author when built) exits 0 iff false-engage-on-birds
== 0 over the corpus AND correct-engage-on-drones ≥ a disclosed floor AND every contested-association
/ flock scene broke off — the same scripted pass/fail ethos as `scripts/check_m4.sh`. **Naming and
claim discipline (red-team fix #6):** a green gate is *`interlock wiring validated`* — it proves the
break-off paths fire when the ground truth says they should. It is **NOT** *`bird rejection
validated`*; no writeup, README, or ADR may claim bird rejection from this gate alone. Bird
rejection is a **bench** claim (§6.5, WORST-tier decoy + real-sky `P_correct(R)`), and until the
bench sets the thresholds the shipped interlock **vetoes all** (engages nothing).

### 6.5 What the Stage-0 bench (Pi 5 + cam) validates on real hardware

The sim proves *plumbing*; the Stage-0 bench (stage0_bench_plan.md) proves the *onboard-carriable*
discriminants against reality:
- **Classifier confidence vs range → `P_correct(R)`** — the analog of the bench's `P_detect(R)`;
  **this sets the ARM range**, the single most decision-relevant onboard number.
- **Detector + classifier throughput/latency/thermals** *with the head attached* — confirms the
  35-fps/29-ms budget and the <7 W / no-throttle envelope hold.
- **Wing-flap detectability vs range** (pairs with the bench's yaw-rate/motion-blur test).
- **Trajectory-smoothness features** (from per-frame track logs; no extra rig).
- **Size-vs-looming** — bounded by the already-measured 15–30% range σ; no new measurement.

**NOT bench-measurable (disclose):** ground stereo true-size (needs the ADR-0017 rig), radar
µ-Doppler (needs the ~$300 TI module — a separate optional bench that demos the 96% physics at
≤~80 m, not a fielded cue), full cross-sensor fusion (validate the *math* in the ADR-0034 capstone).

---

## 7. Honest limitations + open questions

**Limitations (state plainly next to any bird-rejection claim):**
- **The few-pixel wall is real and physics-bound.** At ~10×10 px (drone at 100 m in 1080p) *no*
  appearance model on *any* hardware separates a quad from a bird; the best published single-frame
  FP is 3.73% on *resolved* crops and is effectively undefined (worse) at acquisition range. Bird
  rejection is earned by *temporal accumulation + ground-cue fusion*, not a bigger onboard net.
- **Sim = upper bound, and the sim gate proves PLUMBING only (ADR-0024; red-team fix #6).** The sim
  renders a clean primitive body with no flap model, no motion blur, no vibration, and no *real*
  birds — so any sim PID / bird-rejection number is an UPPER BOUND on real, and a green
  `check_bird_reject` gate is *interlock-wiring validated, NOT bird-rejection validated* (§6.4). A
  clean synthetic bird *flatters* the false-engage rate; the bench `P_correct(R)` on the WORST-tier
  decoy is the reality check, and until it runs the interlock defaults to VETO-ALL.
- **Small birds defeat true-size AND kinematics (red-team fix #5).** Stereo σ_R ≈ 0.45 m at 100 m
  (ADR-0017) **cannot** separate a 0.3–0.5 m bird from a 0.3 m quad — true-size only convicts *large*
  birds — and a straight-gliding small bird shows low jerk and never hovers, so it looks drone-like
  *and* the "absence of hover" is not "presence of bird." The design's answer is to require an
  **affirmative powered-drone signature** to ARM (§2b-1), never "failed to look bird-like"; the
  residual risk is a bird whose track *coincidentally* mimics a powered signature, which the WORST-
  tier straight-glide sim scene and the bench must probe.
- **Single-sensor terminal risk, on a 0.41 s clock.** Post-handoff the veto rides camera-only
  discriminants, each pixel-limited and domain-fragile (93→66% off-domain), inside the measured
  **0.41 s median** terminal window (ADR-0023). The architecture mitigates this by (a) requiring
  mid-course multi-sensor + affirmative-signature confirmation *before* committing, and (b) a **fast
  asymmetric positive-ID-to-hold veto** (§5.2b) that breaks off on a single bird-like frame — the old
  slow streak-to-abort could not have fired in time. It still cannot *eliminate* the residual risk
  that a bird enters the field *after* commit; the fast veto + standoff-widen is the last line, and it
  is honest to say it is a last line, not a guarantee.
- **Kinematic overlap.** Bird cruise (~10–25 m/s) overlaps the drone speed band, so speed alone is
  weak; the gate leans on the affirmative powered-flight signature + accel/jerk variance, which need
  a track buffer of some minimum duration that the pre-handoff lead-time budget (Tier-2 acquisition
  range, still open) sets.

**Open questions (for the ADR / future benches):**
1. What `PID_HIGH` / `PID_ONBOARD_MIN` / N-of-M values actually put the operating point in the
   zero-false-positive region at ≥95%-Pk-equivalent PID? Must be *measured* from a `P_correct(R)`
   curve that does not exist until the Stage-0 bench runs — **no committed number yet, and the
   interlock defaults to VETO-ALL until it does (red-team fix #6). These MUST NOT be back-fit from
   the sim ROC** (§6.3–6.4).
2. Does a nano wingbeat/temporal head add usable discrimination at the medium ranges the interceptor
   cares about, once motion blur + airframe vibration are included? Bench-only; currently an
   assumption (sim models none of it).
3. Exact permissible form of the **VETO-ONLY carried prior** across handoff — a prior that may only
   raise suspicion / lengthen onboard confirmation / abort, and may **never** shorten N or lower the
   commit bar (red-team fix #4) — needs the two-clause `classification_reads_post_handoff=0` audit
   spec (trace-to-onboard + ground-biases-toward-safe-only) written with ADR-0034's rigor.
4. LLR-sum vs Dempster–Shafer for the decision-level channel — an A/B to run in the ADR-0034
   capstone sim, not settled here.
5. The cross-dataset domain haircut for kinematic / µ-Doppler classifiers on out-of-distribution
   birds is unmeasured here (the EO analog dropped 93.6→66%).
6. Confirm the exact licenses of BirDrone and the WOSDETC data before any public-repo redistribution;
   YOLOv7-Segmented (CC BY 4.0) and Halmstad are the safe defaults.

---

## 8. Proposed ADR skeleton (a DIRECTION to pursue — NOT ratifiable/buildable until hardened + benched)

> **ADR-00XX — Bird-vs-hostile-UAS discrimination: confirm mid-course (multi-sensor decision-level
> fusion on the ADR-0034 EKF track), commit at a multi-part ARM gate that demands an affirmative
> powered-drone signature, and hold through the comms-denied terminal with a FAST ASYMMETRIC
> positive-ID-to-hold veto.**
>
> **STATUS — not yet ratifiable.** An adversarial red-team (2026-07, §0) found the *direction* sound
> but the original mechanization **fail-unsafe** on three axes (terminal-veto timing, confidence-signal
> semantics, streak/track binding). This skeleton reflects the **hardened** design. It is safe to
> **pursue**; it is **not** safe to **ratify or build** until (i) the eight §0 fixes are implemented
> and (ii) the Stage-0 bench has produced `P_correct(R)` and set the thresholds. **Until then all
> operating-point thresholds are UNSET and the interlock DEFAULTS TO VETO-ALL (engages nothing).**
>
> **Context.** Removing the AprilTag (ADR-0033 item 2) forfeits the fiducial's implicit "this IS the
> target" guarantee; a real sky has birds — the #1 fielded-C-UAS false-alarm source (ADR-0013/0019),
> and a consumer quad's signature is "often indistinguishable from a bird." Safety + ethics require
> positive hostile-UAS ID before commit and fail-safe on doubt. The threat model denies the datalink
> in the terminal, so bird rejection cannot depend on the link. Single-frame appearance is
> insufficient at acquisition range (both ~10×10 px; best published FP 3.73% on *resolved* crops). And
> the measured terminal window is only **0.41 s** (ADR-0023), which dictates the veto timing below.
>
> **Options considered.** (a) Onboard single-frame classifier gates engage — *rejected*: FP far too
> high at few-pixel range, domain-fragile, decision made where sensing is weakest. (b) Ground rig
> decides and streams the label into terminal — *rejected*: violates the comms-denied thesis (label
> leaks through a denied link). (c) **Confirm mid-course, carry a committed decision, hold with an
> onboard veto** — *adopted, as hardened below*.
>
> **Decision (hardened).** (1) **Decision-level classification channel** (Bayesian LLR-sum default,
> Dempster–Shafer upgrade) fusing onboard appearance + ground kinematics + ground stereo true-size +
> optional radar µ-Doppler into a **dedicated hostile-class posterior `P(hostile)`** — a NEW field,
> **never** the `decision_margin` detection scalar (fix #2). It runs **on the same track + chi-square
> gate as the ADR-0034 EKF** (degrades to camera-only natively), and the hostile streak is **bound to
> a tracked OBJECT IDENTITY, resetting on any re-association / gap** (fix #3). (2) **ARM gate** =
> fused `P(hostile)` ≥ `PID_HIGH` **AND** onboard-alone `P(hostile)` ≥ `PID_ONBOARD_MIN` **AND** an
> **affirmative powered-drone signature** observed (hover / vertical-climb / rotor µ-Doppler /
> sustained powered flight — never "failed to look bird-like," fix #5) **AND** association uncontested
> (fix #8), with a slow N-of-M mid-course confirmation streak; fail ANY ⇒ break off (added AND-clauses
> on m4_intercept.py:1747–1749, no new lethal path). (3) **Terminal veto — FAST + ASYMMETRIC
> (fix #1):** the engage is *held* only by continuously-sustained positive hostile ID; a single
> high-confidence bird frame, 2-of-3 bird-like frames, any gap in positive confirmation, any track
> re-association, or any contested association triggers **immediate** break-off + standoff-widen (fired
> by the classifier veto, not by waiting out the 5 s lost-tag timeout). Absent/stale/dead
> classification == `P(hostile)`=0 == break-off, by construction, with a liveness watchdog (fix #7).
> (4) **Comms-denied honesty:** a two-clause `classification_reads_post_handoff=0` audit extending
> audit_targets.md §A — post-handoff the engage traces to onboard evidence, and the carried ground
> prior is **VETO-ONLY**: it may raise suspicion / lengthen confirmation / abort, but may **never**
> shorten N or lower the commit bar (fix #4). Radar is confirm-only, never a dependency.
>
> **Headline metric.** False-engage rate on birds ≈ 0 (target 0/N, one-sided upper CI) — the
> **peacetime/testing target**; the *fielded* operating point is a **rules-of-engagement-tunable
> point on the Pk-vs-false-engage ROC** (§1.5 CONOPS doctrine), still fail-safe by default and
> still requiring affirmative positive-ID, with **operational mitigations (range clearance,
> human-on-the-loop abort)** as first-class layers. Recall on drones is secondary and may be
> < 100%. The drones-and-birds + **flock/contested-association**
> Monte-Carlo (confusion matrix + a **diagnostic-only** `P(hostile)` ROC sweep, `gt_*` scoring-only)
> validates **interlock WIRING and is an UPPER BOUND — NOT "bird rejection validated"** (fix #6);
> thresholds come from the Stage-0 `P_correct(R)` bench with a WORST-tier flapping/matched-footprint
> decoy, **not** from the sim ROC. Gated behind M5 + the markerless classifier + the bench.
>
> **Why.** PID must be manufactured where the strong physics-based ground channels are live
> (true-size against large birds, µ-Doppler, affirmative kinematics) and *carried veto-only*, not
> streamed, into a 0.41 s terminal the onboard finishes alone; the multi-part ARM gate makes
> onboard-alone sufficiency AND an affirmative drone signature *preconditions of commit*, not hopes;
> the fast asymmetric veto keeps the engage reversible on the terminal's own timescale; the asymmetric
> cost (a missed drone beats a killed bird) fixes the *default* operating point at high precision and
> makes every ambiguity fail toward break-off — but that cost is **finite and context-dependent, so
> the operating point is a rules-of-engagement parameter that slides *inside* a fixed fail-safe
> envelope** (affirmative PID + break-off-on-ambiguity + veto-all-until-benched never move; §1.5),
> backed by **operational defense-in-depth** (range clearance during test + human-on-the-loop manual
> abort) so no single fragile classifier has to be perfect. This turns the project's #1 named failure
> mode into a design whose safety holes were found and closed *before* build — the honest version of
> the responsible-autonomy story, not an oversold one.

---

### Sources

**Repo / ADR:** `docs/goals.md`; `docs/decisions.md` ADR-0013/0015/0016/0017/0018/0019(+addendum)/0023/
0024/0025/0033/0034; `docs/seeker_design_brief.md` (§, R6/R7, line 47); `docs/stage0_bench_plan.md`;
`docs/ground_modality.md`; `docs/audit_targets.md`; `docs/ekf_design_brief.md`;
`scripts/m4_intercept.py` (lines 274/275/281/282/300/302, conjunction 1747–1749, `run_acquire_and_engage`
1415); `scripts/m4_target_mover.py` (line/weave/jink); `models/fpv_target_markerless/`.

**Web (accessed 2026-07):**
[moneyprouav C-UAS operational reality](https://www.moneyprouav.com/capabilities-limitations-and-operational-reality-in-defense-grade-counter-uas-systems/) ·
[SentryCS myths & reality](https://sentrycs.com/the-counter-drone-blog/counter-uas-technology-myths-and-reality/) ·
[threshold-moving for imbalanced classification](https://machinelearningmastery.com/threshold-moving-for-imbalanced-classification/) ·
[Hailo-8 M.2 (26 TOPS)](https://hailo.ai/products/ai-accelerators/hailo-8-m2-ai-acceleration-module/) ·
[Seeed/Hailo-8 YOLOv8n benchmark](https://wiki.seeedstudio.com/benchmark_of_multistream_inference_on_raspberrypi5_with_hailo8/) ·
[FLIR C-UAS pixels-on-target](https://oem.flir.com/learn/discover/thermal-infrared-sensor-design-considerations-for-counter-uas-defense/) ·
[drone-warfare EO/IR (domain drop, false-alarm)](https://drone-warfare.com/counter-uas/eo-ir-detection/) ·
[YOLOMG / ARD-100 (10 px @ 100 m)](https://arxiv.org/abs/2503.07115) ·
[BirDrone / YOLOBirDrone (FP 3.73%)](https://arxiv.org/html/2601.08319v1) ·
[MobileNetV3 bird-vs-drone 93%](https://www.mdpi.com/2673-2688/6/3/57) ·
[NanoDet (Apache-2.0)](https://github.com/RangiLyu/nanodet) ·
[YOLOv7-Segmented Drone-vs-Bird (CC BY 4.0)](https://data.mendeley.com/datasets/6ghdz52pd7/5) ·
[Halmstad Multi-Sensor](https://pmc.ncbi.nlm.nih.gov/articles/PMC8573135/) ·
[SimD3 synthetic bird distractors](https://arxiv.org/abs/2601.14742) ·
[WOSDETC Drone-vs-Bird challenge](https://github.com/wosdetc/challenge) ·
[Rahman & Robertson µ-Doppler (wingbeat 4–6 Hz, K/W-band)](https://www.nature.com/articles/s41598-018-35880-9) ·
[MDPI *Signals* 2023 µ-Doppler ~96%](https://www.mdpi.com/2624-6120/4/2/18) ·
[Kengeskanov 2023 trajectory drone/bird](https://hal.science/hal-04101996) ·
[LAT-BirdDrone trajectories](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12769828/) ·
[DIAT-µSAT µ-Doppler](https://ieee-dataport.org/documents/diat-msat-micro-doppler-signature-dataset-small-unmanned-aerial-vehicle-suav) ·
[Dempster–Shafer decision-fusion review, *Sensors* 2019](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6865203/) ·
[Gazebo Harmonic actors](https://gazebosim.org/docs/harmonic/actors/) ·
[pigeon wingbeat 5.48 Hz / 20 mm bob](https://pmc.ncbi.nlm.nih.gov/articles/PMC6228479/).
