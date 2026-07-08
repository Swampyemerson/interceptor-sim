# Ground stereo rig vs. onboard seeker — division of labor

*Systems note answering two builder questions raised after the markerless-seeker
prototype (`docs/seeker_prototype_results.md`): **Q4 — how much does the ground stereo
rig fix the "onboard only sees the target terminally (1.6–3 m)" problem?** and **Q5 —
how does the ground rig help with bird prevention?** Desk synthesis only — no sim was
booted (a Monte-Carlo batch was running). Every number traces to an ADR or a logged
result; where a claim is a design target rather than a measured result, it is labelled.*

Companions: `docs/seeker_design_brief.md` (the seeker interface + range plan),
`docs/seeker_prototype_results.md` (the measured terminal-only acquisition),
`docs/stereo_design.md` / ADR-0017 (the rig physics), `docs/bird_discrimination_design.md`
+ ADR-0035 (the discrimination design), ADR-0034 (the fusion capstone).

New terms, one line each (defined once, used freely after):
- **Cue / mid-course track** — the position+velocity message the *ground* rig sends the
  interceptor so it can fly toward the target *before* its own camera can see it.
- **Handoff** — the moment the interceptor's own camera takes over from the ground cue
  and finishes the intercept alone (the "terminal" phase).
- **Coast-blind window** — the last few meters where the datalink is jammed, the cue is
  gone, and the onboard camera must acquire the target by itself.
- **PID (positive identification)** — affirmative evidence that a track is a hostile
  drone (not a bird, not clutter) — the thing you must have *before* you commit.

---

## 0. The one-paragraph answer

The onboard camera is a **terminal** sensor, not a **search** sensor. The seeker
prototype confirmed it acquires the target body only at **1.6–3.0 m** (~4–6% recall,
terminal-only — `docs/seeker_prototype_results.md` §2–3), and even the clean AprilTag
stand-in only reaches out to **9–12 m** (ADR-0015 row 10 / ADR-0024). Neither is a way to
*find* a drone. Finding it is the **ground stereo rig's** job: it detects and tracks a
0.3 m quad out to roughly **60–160 m in daylight** (ADR-0019, reconciled with ADR-0017)
and flies the interceptor to the target via the cue, so **onboard detection is not needed
until the terminal basket.** That division of labor is the whole architecture (ADR-0015/
0016). It has exactly **one irreducible seam**: under jamming the cue dies and the onboard
must acquire *alone* in the last ~10–15 m (§2). And the same ground rig is also the
**primary bird discriminator** (§3), because true-size and a kinematic time-series — the
strong physics — only exist on the ground channel, not in the 0.41 s onboard terminal.

---

## 1. Division of labor — who sees what, and at what range

The system is deliberately **two sensors with two jobs**, not one camera trying to do
both search and terminal (ADR-0015 hybrid split; ADR-0016 makes the arithmetic concrete).

### 1.1 The range picture (this is the crux)

| Sensor | Job | Acquisition / useful range | Source |
|---|---|---|---|
| **Ground EO stereo rig** | Search + mid-course track (position **and** velocity) | **~120–160 m** classified & bird-rejected track on a 0.3 m FPV in good daylight (EXPECTED); ~250–440 m bare detection at BEST; **~59 m WORST** (small/side-on, overcast/dusk) | ADR-0019 envelope; ADR-0017 rig (σ_R ≈ 0.45 m at 100 m, detection floor ~160 m) |
| **Onboard camera — AprilTag stand-in** | Terminal lock (the sim's clean fiducial) | **~9–12 m** | ADR-0015 row 10 / ADR-0024/0028 |
| **Onboard camera — real markerless body** | Terminal lock (what the tag stands in for) | **~1.6–3.0 m**, ~4–6% recall, *terminal-only* (measured on the prototype) | `docs/seeker_prototype_results.md` §2–4 |

Read the gap: the ground rig sees the target at **~60–160 m**; the real onboard seeker,
per the prototype, doesn't get a genuine target lock until **~1.6–3 m**. That is **one to
two orders of magnitude**. If the interceptor had to *find* the target with its own
camera, it could not launch until the target was almost on top of the defended point.
The ground rig closes that gap by **flying the interceptor most of the way there blind**
(to its own camera), on the cue.

### 1.2 Why the split is the right architecture, not a workaround

Heavy compute (a two-stream small-object detector + stereo triangulation + a Kalman
track) sits on the ground where power and weight are free; the jam-critical *terminal* is
onboard, where it is jam-immune (ADR-0016). The ground rig emits a **~90-byte track
message** (position, filtered velocity, covariance, quality) at ≥10 Hz — **7.2 kbps, ~22%
of a cheap SiK radio's capacity** — whereas streaming video would be **60–125× over that
same link** (ADR-0016). So "track, not video" is what makes the mid-course cue affordable
and jam-resistant at once. This is also *the* reason the running-start/dash geometry works
at all: without a mid-course cue the interceptor is back to a ~3 m/s hover-start cap
(ADR-0011), and **12 m/s is uncatchable from a hover-start** (0/8 both guidance laws,
ADR-0028). The cue is what lets the interceptor already be moving, pointed the right way,
when the terminal begins.

### 1.3 So — how much does the ground rig "fix" the late-onboard-detection problem?

**Almost entirely, in the nominal (un-jammed) case, and by design.** The prototype's
"onboard only sees it at 1.6–3 m" is only a *problem* if onboard detection has to happen
early. It doesn't: the ground cue supplies position+velocity from ~60–160 m and flies the
dash, so the onboard camera only has to acquire inside the **terminal basket**, which is
exactly the range band where even the weak markerless seeker starts to work (the target
looms to enough pixels to detect — prototype §3.3). The late onboard acquisition is a
**terminal-phase** number; the ground rig makes the *mid-course* someone else's job.

**But it does not make onboard acquisition range irrelevant** — for two reasons that §2
develops: (a) the terminal is *deliberately* comms-denied, so the cue is *supposed* to be
gone by the end; and (b) correction capacity ∝ t_go² (ADR-0023/0024), so *how early* the
onboard reacquires after handoff still sets the achievable miss. The ground rig moves the
hard problem from "search" to "terminal reacquire under jamming" — it does not delete it.

---

## 2. The jamming residual — the coast-blind window (the irreducible job)

This is the seam the ground rig **cannot** cover, and it is the reason the whole project
exists (GOALS.md: *"when the datalink is denied, the interceptor's own camera locks the
target and finishes the intercept"*).

### 2.1 What happens when the link is jammed

The threat model denies the datalink in the terminal (fiber-optic FPV is RF-silent and
the incoming C-UAS link can be jammed — ADR-0019). When the link is cut:
1. the ground cue **stops arriving** at the interceptor;
2. the interceptor **dead-reckons** (coasts on the last cue velocity) toward the predicted
   basket;
3. it must **acquire the target with its own camera, alone**, in roughly the **last
   ~10–15 m** before closest approach — the **coast-blind window**.

That window is the irreducible comms-denied job. No amount of ground compute reaches into
it, because the link that would carry the ground's help is the exact thing that's been
denied. This is why the seeker's **onboard acquisition range** (the markerless-seeker
tasks) is the binding number: the earlier the onboard camera reacquires inside that
window, the more t_go remains to null the miss.

### 2.2 Why acquisition range (not terminal accuracy) is the lever

ADR-0023 measured this on the clean AprilTag terminal and it is stark: the terminal miss
is ~96% set by the **zero-effort-miss (ZEM) at handoff** (r² = 0.99). The camera-only
terminal window is only **0.41 s median**, and within it the correctable amount is
`½·a·t_go² = ½·8.7·(0.41)² ≈ 0.72 m` against a **1.69 m** delivered ZEM — physically too
short to fix a bad delivered geometry (ADR-0023). Corollary: acquiring at ~12 m vs ~6.5 m
moves correction capacity **0.72 → ~4.3 m** (ADR-0024/0028). So the coast-blind window's
*length* — set by onboard acquisition range — dominates; terminal range *accuracy* does
not. A markerless seeker that only reacquires at 1.6–3 m (prototype) instead of the tag's
9–12 m is a real **R2 acquisition regression** and is the honest cost of removing the tag.

### 2.3 How the fusion capstone (ADR-0034) helps — and where it can't

The forward answer to the jamming residual is the **covariance-gated mid-course EKF
fusion** capstone (ADR-0034), which unifies the seeker + EKF + fusion threads:

- **What it buys — mid-course robustness, not terminal meters.** Under a *degraded* cue,
  ADR-0030 found the binding failure is not miss accuracy but **DASH-ABORTS: 33% of 9 m/s
  flights never reached handoff at all** (mid-course track collapse). ADR-0034's expected
  payoff is precisely *fewer* of those — an **earlier, warmer, longer-reaching handoff** so
  the onboard camera enters the coast-blind window already pointed correctly and closer.
  A warmer handoff effectively **lengthens the usable terminal**, which is the ZEM lever.
- **How the fallback is native, not bolted on.** Give the EKF both the ground cue and the
  onboard bearing with honest per-source noise: (a) **covariance weighting** auto-drives
  the ground cue's weight toward zero as its uncertainty blows up (far range, datum bias,
  stale/dropped link), so the estimate rides the camera; (b) the **innovation (chi-square)
  gate** rejects an individual ground reading that disagrees with the prediction. "Fall
  back to camera when the ground looks worse" falls *out* of the estimator — you don't
  hand-code the if-statement (ADR-0034). Note this **re-opens** the earlier fusion null
  (ADR-0018 addendum): that null was measured with a *clean* AprilTag terminal + a
  fixed-gain α-β tracker — the two conditions that structurally suppress fusion's value.
  A *noisier* markerless camera + a covariance-weighting EKF is exactly where it can flip.
- **Where it structurally cannot help — the honesty boundary.** Fusion stays
  **mid-course only**; the terminal must still degrade to **camera-only**, or the
  jam-resistance thesis quietly breaks through the filter's memory. The audit must extend
  to *"no cue-tainted EKF state OR covariance survives the handoff"* (ADR-0034). And the
  kinematic ceiling still caps the terminal: a better track cannot out-run t_go at 12 m/s
  from a bad geometry (ADR-0023). So the capstone buys **handoff-reach rate and mid-course
  track RMSE** — a stronger, more honest headline than chasing the final meters — but the
  coast-blind acquire itself remains the onboard camera's lonely job.

**Net:** the ground rig + fusion shrink the coast-blind window (warmer, closer, better-
aimed handoff) but cannot erase it. Its remaining length is bought only by a better
*onboard* seeker — which is why the markerless-seeker acquisition-range work is the
irreducible core, not optional polish.

---

## 3. Bird prevention — the ground rig is the PRIMARY discriminator

Birds are the **#1 fielded C-UAS false-alarm source** (ADR-0013; ADR-0019 §3). Removing
the AprilTag forfeits the fiducial's implicit "this IS the target" guarantee (ADR-0035),
so the system now has to *earn* positive identification. The onboard camera **cannot** do
this alone: at acquisition range a drone and a bird are both **~10×10 px**, best published
single-frame false-positive is **3.73% on *resolved* crops** (worse at few-pixel range),
and the onboard terminal is only **0.41 s** — no time for a time-series (ADR-0035;
`docs/bird_discrimination_design.md`). So, exactly as with detection, PID must be
**manufactured on the ground, before commit, and carried veto-only into the terminal.**

### 3.1 The four ground discriminants (why the rig, not the seeker, owns bird-rejection)

The ground rig has three things the onboard seeker structurally lacks — **absolute range,
a long observation time, and heavy compute** — and those unlock the strong discriminants:

1. **TRUE SIZE in absolute metres (the headline, and mono-impossible).** Stereo gives
   *range*, so angular size → **absolute** body/wingspan in metres (ADR-0017: σ_R ≈ 0.45 m,
   cross-range ≈ 1 cm, both at 100 m). A 0.3 m rigid quad and a **1.0–1.5 m-wingspan
   raptor** at 40 m are cleanly separable. A **monocular** onboard frame physically *cannot*
   produce absolute size — mono "size" is a guess scaled by an *assumed* target width
   (`docs/bird_discrimination_design.md` §2b-2). This is the single biggest reason bird
   rejection lives on the ground.
2. **The kinematic track time-series — the affirmative powered-drone signature.** Over the
   mid-course seconds the rig's Kalman track measures sustained powered flight (station-
   keeping, powered acceleration against wind, non-ballistic curvature) that a coasting/
   flapping bird does not show. Crucially the ARM logic requires an **affirmative** powered
   signature — never "failed to look bird-like" (ADR-0035 fix; a null observation must fail
   safe, not commit).
3. **Mid-course TIME and COMPUTE.** The rig has *seconds* and ground-class silicon to run a
   proper detect-then-track classifier and accumulate evidence; the onboard terminal has
   0.41 s and a Pi-class budget. Evidence integration is a ground luxury.
4. **Optional micro-Doppler radar (budget-gated confirm-only).** The cleanest bird-vs-rotor
   physics of any modality — a rigid rotating prop throws a wide symmetric micro-Doppler
   signature; reported ~96% drone-vs-bird on spectrograms (ADR-0019 addendum; ADR-0035).
   Confirm-only, never a dependency.

These fuse into **one dedicated hostile-class posterior `P(hostile)`** riding on the
ADR-0034 EKF track — a *new* field, never the `decision_margin` detection scalar (a
confidently-detected *bird* must drive `P(hostile)` **down**, not up). The onboard seeker
then **carries** that committed decision and runs a **fast asymmetric veto** ("positive-ID
to HOLD," not "streak to abort," on the 0.41 s clock); it never originates classification
from cold (ADR-0035 decision 1–2).

### 3.2 The honest residual (what the ground rig does NOT fix)

Bird rejection has the **same two holes** as detection, and they are disclosed, not
hidden (ADR-0035; `docs/bird_discrimination_design.md` §2):

- **Jamming removes the ground confirm at the terminal.** The comms-denied window that
  makes onboard acquisition hard *also* removes the ground's PID help in the last seconds.
  The mitigation is the **carried, veto-only prior** (it may raise suspicion / lengthen
  confirmation / force break-off — but may **never** shorten the confirmation or lower the
  commit bar), plus the **fast onboard veto**, plus the honesty audit that post-handoff
  engagement traces only to onboard evidence (ADR-0035 decision 5).
- **Small 0.3–0.5 m birds defeat true-size.** A swift/starling/small-falcon is the *same
  physical size within measurement noise* as a 0.3 m quad, so the headline discriminant
  (true-size) fails for them (`docs/bird_discrimination_design.md` §2b-2 — the honest
  correction to "size is always separable"). What covers this residual is the **layered
  defense**, not a single perfect classifier: the affirmative powered-flight kinematic
  requirement, optional micro-Doppler, and — first-class — the **operational mitigations**:
  range clearance (geographic + temporal bird deconfliction during bring-up), human-on-the-
  loop with manual abort authority, and the **default VETO-ALL** posture until the Stage-0
  bench measures `P_correct(R)` and sets thresholds (ADR-0035 decision 3–4; status line).

### 3.3 Doctrine guardrails (so "bird prevention" doesn't over-rotate)

The asymmetric bird-vs-hostile cost is real but **finite and context-dependent, not
infinite** — a system so bird-cautious it lets an armed hostile reach the people it
defends is *also* a failure (ADR-0035 context). So the operating point is a **tunable point
on a *measured* Pk-vs-false-engage ROC**, set by ROE, sliding *inside* a fixed fail-safe
envelope. ROE sets only the acceptable false-positive *bound*; it can **never** switch off
the three invariants: affirmative PID always required, ambiguity always breaks off,
pre-bench ⇒ VETO-ALL. This is **not** "engage on doubt." And the sim gate proves
**plumbing only** — a green `check_bird_reject` means "wiring validated," not "bird
rejection validated," which is a **bench** claim (WORST-tier flapping + matched-footprint
decoy + real-sky `P_correct(R)`, ADR-0035 honesty line).

---

## 4. Bottom line for the portfolio

- **Detection (Q4):** the ground stereo rig turns "my camera can't see the drone until
  1.6–3 m" into a non-problem for the *mid-course* by detecting at **~60–160 m** and flying
  the interceptor there on a cheap, jam-resistant track message. It **fixes the search
  problem entirely in the nominal case** — but it moves the residual, it doesn't delete it.
- **The residual (Q4→jamming):** the deliberately comms-denied terminal leaves a
  **coast-blind window** (~last 10–15 m) that only the onboard seeker can close, and
  because correction capacity ∝ t_go² the onboard's *acquisition range* is the binding
  lever. The ADR-0034 fusion capstone shrinks that window (warmer, closer handoff, fewer
  dash-aborts) but is structurally barred from the terminal itself.
- **Bird prevention (Q5):** the ground rig is the **primary discriminator** because
  absolute true-size, a kinematic time-series, mid-course time/compute, and optional
  micro-Doppler are all ground-only physics; PID is manufactured there and carried
  **veto-only** into the terminal. The residual (jamming + small 0.3–0.5 m birds that
  defeat true-size) is covered by layered defense and operational mitigations, not by any
  single classifier — and the whole thing defaults to **VETO-ALL until benched.**

The clean one-liner: **the ground rig does the seeing and the deciding; the onboard camera
does the finishing — alone, on purpose, in the last few meters where the link is denied.**

---

### Sources

Repo/ADR: `GOALS.md`; `docs/decisions.md` ADR-0011/0013/0015/0016/0017/0018/0019/0023/
0024/0028/0030/0033/0034/0035; `docs/seeker_prototype_results.md`;
`docs/seeker_design_brief.md`; `docs/stereo_design.md`; `docs/ground_modality.md`;
`docs/bird_discrimination_design.md`; `docs/compute_setup.md`.

Key measured/derived numbers cited above (each traces to the ADR named inline): onboard
markerless acquisition 1.6–3.0 m / ~4–6% (`seeker_prototype_results.md`); AprilTag
9–12 m (ADR-0015/0024); ground EO envelope 59–160 m (ADR-0019/0017); stereo σ_R ≈ 0.45 m
@100 m (ADR-0017); track message 7.2 kbps / ~22% link (ADR-0016); ZEM r²=0.99, 0.41 s
terminal, 0.72 vs 1.69 m (ADR-0023); acquisition-capacity 0.72→4.3 m (ADR-0024/0028);
33% dash-abort under degraded cue (ADR-0030); best single-frame bird FP 3.73% on resolved
crops, ~96% micro-Doppler (ADR-0035 / `bird_discrimination_design.md`).
