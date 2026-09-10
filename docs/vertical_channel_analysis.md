# Why a late camera correction has never saved the aim

*The camera has no vertical channel. And on the AprilTag path the terminal
window really is too short — but that is the tag's limit, not the camera's.*

> **Origin.** Builder question, 2026-09-09: *"I've literally watched a video of an aim
> get completely saved by a quick adjustment at the end with the camera. And the
> ballistic aims won't be far off. And why? Is there really a physics limit to where
> the AprilTag can be read and adjustments made in time?"*
>
> **Verdict: he is right, and one specific thing is missing.** Nothing in the physics
> forbids the late save at this project's operating point. The camera terminal has
> never been able to attempt it, because it has **no vertical steering at all** and the
> residual miss is 82% vertical. A vertical channel was **decided** in ADR-0085 and
> never written.
>
> **Nothing here says the camera works.** It says the strongest camera-versus-dash test
> has not been run. Ledger entry: `terminal-vertical-channel-decided-not-built`.
> Arithmetic: `scripts/forensics/terminal_capacity.py` (offline, exits 0, no sim).
>
> ---
>
> ## ⚠️ CORRECTED 2026-09-10, and one half of it corrects THIS DOCUMENT
>
> Two measurements were taken after the first draft, and they change §2:
>
> * **Closing speed is not 9 m/s.** `scripts/forensics/handoff_closing_speed.py`
>   measures **17.60 m/s** in the last second before handoff (median, n=14) and
>   **14.64 m/s** after handoff pre-CPA, against the money gate's 9.0. Re-derived at
>   those speeds the AprilTag path has **0.02–0.17 m** of terminal correction
>   capacity, **not** the 0.6–1.8 m this document first claimed off the gate's own
>   assumption. Against a ~0.4 m need that is **insufficient**. So for the TAG
>   baseline the short window IS close to a real limit — §2's original framing was
>   too strong and is superseded by §2.1 below. The markerless path at 20 m
>   acquisition still has ~3.9 m, so the limit remains the TAG's, not the camera's.
> * ~~**The vertical error is delivered by the DASH**, 24/24 flights, +0.320 m of
>   dash drift, 66% of the miss.~~ **RETRACTED 2026-09-10 — the baseline included
>   the TAKEOFF.** `alt_m` is height above the arm point, so it reads 0.000 on the
>   ground; the old figure was mostly the vehicle leaving the pad. Against a
>   settled hover the dash drifts **−0.019 m**. The vertical error decomposes into
>   `origin` **+0.127 m / 23%** (a datum mismatch, present before the dash starts),
>   `dash_delta` **+0.012 m / 10%**, and `post_dash_delta` **+0.348 m / 66%**
>   (after handoff). ADR-0099's reason (1) is **refuted**; its trim is a 10% lever.
>   §7 has the corrected arithmetic and §2.1's flight count is 16/16, not 24/24.
>
> The §3 finding — no vertical channel, and ADR-0085 decided one that was never
> written — is **unaffected**: it is about which axis is steered, not how long there
> is. Decision and build: **ADR-0099**. Pre-registration:
> `docs/vertical_channel_prereg.md`.

---

## 1. The claim being questioned, and what it actually rests on

The standing headline is that the camera terminal does not beat a well-aimed open-loop
dash. That is **measured**, and it survived the 2026-07-25 re-fly after the zero-command
bug was fixed: camera-beats-twin 3/8, 1/8, 4/8 against a pre-registered 6/8 bar.

It is easy to hear that as *"the terminal window is too short to fix anything"*, and to
reach for ADR-0023's bound to justify it. **That inference does not hold**, for two
independent reasons, one of which is a project-rule violation.

---

## 2. The kinematic bound is being quoted outside the regime it was measured in

ADR-0023's bound is `½·a·t_go²` = **0.72 m** of correction capacity against **1.69 m**
of zero-effort miss already delivered at handoff. Capacity loses by 2.3×, so the miss is
locked in. That is correct — *for the flights it was measured on*, the 6 m/s cue-era
campaign with a poor delivered geometry.

At **correct aim** the delivered error is not 1.69 m. ADR-0095 re-scored the adopted
configuration at a median centre-to-centre closest approach of **0.412 m** (0.435 m
including the interpolation term). Against the ratified 0.35 m ram radius the median
needs about **0.06 m** of correction, and the largest single component is a **0.374 m
vertical bias**.

So the ratio inverts **on the delivered-error side**: the geometry handed to the
terminal at correct aim is 0.41 m, not 1.69 m. Whether the terminal can then fix it is
a separate question, and §2.1 answers it with measured closing speeds rather than the
gate's assumption. Short version: on the AprilTag path it cannot.

Re-using a threshold across that gap is exactly the failure mode CLAUDE.md already has a
standing rule against: *a threshold validated at one operating point is NOT validated at
another*. The bound was earned at 1.69 m of delivered error. It is being spent at 0.41 m.

### 2.1 The capacity at MEASURED closing speeds — the rows to trust

The money gate prices both the streak burn and the surviving time-to-go at a single
9.0 m/s. The streak actually forms during `CODED_DASH`, at `dash_speed_ms = 16.0`;
`ENGAGE` only then commands 9.0 tapering to 5.5. Measured out of the committed
per-tick logs:

| Window | Measured closing speed |
|---|---:|
| During the dash | 14.52 m/s (median, n=16) |
| Last 1.0 s before handoff | **17.60 m/s** (n=14) |
| After handoff, pre-CPA | 14.64 m/s (n=13) |

Re-deriving with the burn at 17.60 and the time-to-go at 14.64:

| Configuration | Burn | t_go | Capacity |
|---|---:|---:|---:|
| Tag `qd=2.0`, R90 7.10 m | 6.10 m | 0.068 s | **0.02 m** |
| Tag `qd=1.0`, R90 8.97 m | 6.10 m | 0.196 s | **0.17 m** |
| Markerless, R_acq 20 m | 6.10 m | 0.949 s | **3.92 m** |

**So the answer splits.** On the AprilTag path there is effectively no terminal
authority, and the builder's question has a genuine "yes, close to a physics limit"
answer — driven by the tag's ~6 m read range against dash-speed closure, not by
camera guidance. On the markerless path at a 20 m acquisition range there is ten
times the authority needed. The tag is the limit.

**And a live consequence for the money gate**, flagged in §6 and deliberately not
acted on: pricing the burn at 9.0 m/s when it is paid at 17.60 under-prices it by
about 1.96×, in the flattering direction, on a ~$740 decision.

### 2.2 The same arithmetic at the GATE's assumption, for comparison only

Handoff needs `k=5` consecutive decodes, so the range burned forming the streak uses the
run-length expectation `E[T] = (1−pᵏ)/(pᵏ(1−p))` (ADR-0079), not the mean rate. `a = 8.7
m/s²` is the *measured* median lateral acceleration in ENGAGE (`terminal_diagnosis.md`).

**As the money gate computes it** (burn and time-to-go both at 9 m/s):

| Configuration | Burn | t_go | Capacity |
|---|---:|---:|---:|
| Tag `qd=2.0`, R90 7.10 m, 20 Hz loop, p=0.9 | 3.12 m | 0.442 s | **0.85 m** |
| Tag `qd=1.0`, R90 8.97 m, 20 Hz loop, p=0.9 | 3.12 m | 0.650 s | **1.84 m** |
| Tag `qd=2.0` pessimistic, R90 5.98 m, 14 Hz | 4.46 m | 0.169 s | 0.12 m |

These are the numbers the money gate believes, and they are the ones this document
originally quoted. **Do not use them.** Every row here except the pessimistic corner
appears to clear the ~0.4 m that matters, and §2.1 shows that is an artefact of pricing
the burn at 9.0 m/s when it is paid at 17.60. They are kept only so the size of the
gate's optimism is visible.

**Sensitivity to lateral authority**, at t_go = 0.65 s — a time-to-go the AprilTag
path does not actually reach at measured closing speeds, so read this as "what more
authority would buy IF the acquisition range were there", i.e. the markerless case:

| a (m/s²) | Capacity |
|---:|---:|
| 6.7 — actually achieved (ADR-0028 addendum) | 1.42 m |
| 8.7 — measured median in ENGAGE | 1.84 m |
| 12.0 — `MPC_ACC_HOR_MAX` cap | 2.54 m |
| 20.0 | 4.23 m |

And the airframe is not the limit: ADR-0028's addendum measured the interceptor achieving
only **6.7 m/s²**, well under its own 12 m/s² cap, because the binding constraint is the
**guidance command ceiling** (`V_PERP_MAX = 8`, `V_TOTAL_MAX = 13`). The real 5-inch
carries T/W of 6.3–7.8:1 (ADR-0084 sizing), far more authority than the sim x500.

---

## 3. The structural reason: there is no vertical channel

This is the finding.

**Every seeker computes the target's vertical look angle.** `bearing_vert_rad` is
produced by `finetuned_seeker.py`, `nn_seeker.py`, `two_stage_seeker.py` and
`classical_seeker.py`.

**Nothing in `flight/` consumes it.** A repo-wide grep returns only sim-side evaluation
tooling. The elevation *does* enter the derotation chain — FIX-A rotates the full 3-D
ray, which is how roll cross-coupling is captured — but it is used **only to get the
horizontal azimuth right**, never to command a vertical correction.

**The only vertical command is an altitude-hold P-loop to a preset height**, in both the
deployed loop and the harness that produced every measured arm:

```
v_down = clamp(kp_alt · (alt − alt_ref), ±v_vert_max)      kp_alt = 1.0, v_vert_max = 2.0
```

`flight/deploy/seeker_loop.py:579` · `scripts/m4_intercept.py:1747, 3692, 3737, 3899,
4282, 4696`

The other two vertical mechanisms in the tree are not homing either: `dash_loft_alt_ref`
is an open-loop loft-then-dive profile, and `flight/fov_guidance.py` is a **framing**
aid — it centres the target in the image to keep it in view. That module is parked
because framing at correct aim measured net-negative. Framing and homing share a command
but are not the same objective, so the parking rationale does not transfer.

### 3.1 What that costs, measured

ADR-0095, adopted configuration, n=16, corrected centre-to-centre ruler:

| Component | Value |
|---|---:|
| Vertical | **−0.374 m** |
| Horizontal | 0.174 m |
| Magnitude | 0.412 m |
| Vertical share of the squared error | **82%** |
| Kill radius | 0.35 m |
| **Miss if the vertical term alone were nulled** | **0.174 m — inside the radius** |

So the terminal steers the axis carrying 18% of the squared error and is blind to the
axis carrying 82%. "The camera does not beat a well-aimed dash" is a true statement about
arms whose guidance was **structurally incapable** of correcting the dominant term.

### 3.2 It was decided, and never built

ADR-0085 (2026-07-25) reads: *"In the terminal, the vertical channel follows the
camera-derived target elevation within a bounded band, floored by a hard AGL minimum."*

Neither half exists. `grep -n "min_agl\|agl_m\|elevation" flight/deploy/real_flight.py`
returns nothing. `fov_guidance.py` appears in that file only inside the honesty-audit
module list — audited, never called. And `ADR-0085` appears **zero** times in
`project_state.json` and **zero** times in `docs/next.md`.

This is the class the project already has a rule for — *a fix is not done until its
effect is observed end-to-end* — one step earlier: an ADR with no code at all.

### 3.3 The mechanism — proposed, tested, and half refuted

**What I proposed.** A P-only altitude loop cannot null a persistent disturbance; it
settles at whatever error commands enough climb. During the dash the vehicle pitches
nose-down 27–36°, so vertical thrust falls to `cos(35°) ≈ 0.82` of hover. A systematic
**sag** is exactly what that predicts, matching the README's *the interceptor flies low*.

**What the data said, in two stages — and the second stage refuted the first.**

*Stage 1 (wrong).* The tool reported the interceptor **above** the target on 24 of 24
flights (median +0.485 m) with altitude **rising +0.320 m** across the dash. Read as:
sag refuted, general "altitude-hold drift across the dash" supported at 66% of the miss.

*Stage 2 (2026-09-10, after review).* Both halves were wrong. The **24/24** count
included 8 files that carry no dash phase at all and were being counted as measured —
it is **16/16** (median **+0.508 m**). And the **+0.320 m** was measured against "the
median of every tick before the dash", which on this fleet pools 87–108 `TAKEOFF` ticks
with 21–24 settled hover ticks. `alt_m` is AGL above the arm point, so it reads **0.000
on the ground** and the `TAKEOFF` median is +0.005 m: the "drift" was largely **the
takeoff climb**. Against a settled hover the dash drifts **−0.019 m**, robust from
−0.012 to −0.025 m across six baseline windows.

**So neither hypothesis survived.** Not sag, and not climb-during-the-dash either: the
dash barely moves the altitude at all. The exact additive decomposition (verified per
flight on 16/16, paired per-flight medians, n=13):

| Term | Median | Paired share | Range |
|---|---|---|---|
| `origin` — settled pre-dash separation | **+0.127 m** | 23% | 15–30% |
| `dash_delta` — hover → dash end | **+0.012 m** | 10% | 1–43% |
| `post_dash_delta` — dash end → CPA | **+0.348 m** | 66% | 29–78% |

**What that changes.** ADR-0099's reason (1) — "the error is delivered by the dash, so
correct it there" — is refuted, and `dash_alt_trim_m` acts on the 10% term. It is still
true that a wrong-signed trim **doubles** the error, which is why the lever takes a
measured drift as input and carries a `share` parameter; but on the **adopted**
coded-dash configuration there is no measured drift at all, because that configuration
goes `TAKEOFF → CODED_DASH` with no hover and the tool now correctly **fails closed**
rather than returning a takeoff number.

**Where the error actually is.** 23% is a **two-datum mismatch**: `ALT_REF_M = 0.5` is
AGL above the arm point while the target's altitude is world z, and nothing reconciles
the ~0.22 m the camera sits above ground at rest. That is real separation and cannot be
subtracted from a miss — but it is **aim, not guidance**, and one line of scenario
setup removes it. It is the cheapest 23% in the project.

**Two honest gaps.** (1) The committed CSVs carry **no attitude columns** (deep-audit
DEEP-R2), so any pitch-to-drift link is reasoning, not data. (2) Worse, the camera rides
a forward boom, so `gt_cam_z − alt_m` — which ought to be a fixed geometric offset —
moves **+0.033 m** hover→dash-tail and **+0.087 m** hover→CPA. That is **24% of
`post_dash_delta`**: roughly a quarter of the "vertical miss" is the camera swinging,
not the airframe moving, and it cannot be removed from these logs. An earlier version of
the tool's docstring asserted the offset "is HORIZONTAL and does not enter a vertical
measurement", which is true only while level.

### 3.4 What was built for it (ADR-0099)

Default-OFF and byte-identical, proven by test rather than asserted:

- `flight.guidance.derive_dash_alt_trim_m(measured_drift)` — the trim, **derived** from
  a measurement rather than tuned, per ADR-0080's rule.
- `flight.guidance.apply_alt_ref_trim(alt_ref, trim, min_agl)` — applies it, and
  enforces ADR-0085's **hard AGL floor**, which beats the trim rather than averaging
  with it.
- Applied in `real_flight._v_down`, the one function every state's vertical command
  passes through, so the floor cannot be forgotten at one of five call sites.

Ten tests, mutation-verified: making the lever inert fails six of them. Nothing has
flown. Pre-registration with the adopt/reject criterion and what a null would mean is
committed **before** any arm: `docs/vertical_channel_prereg.md`.

---

## 4. Is there a real physics limit on tag read range? Yes — and it is the tag's

This part of the question has a hard answer, and it is worth separating from the
guidance question.

AprilTag decode needs roughly **22 px of tag edge** (the sim cliff). For the ordered lens,
`fx ≈ 385 px` (118° across 1280). With the adopted 0.35 m placard:

```
R ≈ 0.35 × 385 / 22 ≈ 6.1 m
```

which matches the independently derived 5.98 m conservative / 7.10 m middle prediction in
`placard_sizing.md` §4. The placard is **already at the carry limit**, so there is nothing
larger to choose. That is a genuine optical limit.

But it is a limit of **the baseline seeker choice**, not of camera guidance:

| Seeker | Acquisition range | Pi 5 rate |
|---|---|---|
| AprilTag, 0.35 m placard | **6–9 m** (optical, hard) | 96.6 fps |
| Markerless detector | **~24 m static ceiling**, 100% recall 8–22 m | 6.09 fps on CPU |

First kills fly the tag because the Pi CPU runs the tag at 96.6 fps and the network at
6.09. So the short acquisition range — and therefore the tight time-to-go — is a
consequence of **deferring the ~$70 accelerator**, not of the camera being weak.

The arithmetic makes that concrete. At a 20 m acquisition range the capacity is
**11.2 m** against the same ~0.4 m need. That is the difference between marginal and
irrelevant, and it is the strongest argument for the accelerator in the project — stronger
than any accuracy claim, because it is a pure `t_go²` statement.

---

## 5. Why the video is consistent with all of this

Correction capacity scales as `t_go²`. A pilot tracking continuously from 30 m has about
`(30/7)² ≈ 18×` the authority of a seeker whose first look is at 7 m, at the same closing
speed. And the pilot's "quick adjustment at the end" is the *last* of a continuous series
— the aim was being nulled the whole way in, so what looks like a late save is the tail of
a long correction.

Our seeker's first look is at the fiducial's read limit. The video therefore demonstrates
that **acquisition range is the lever**, which is ADR-0023's own conclusion, arrived at
from the opposite direction. It contradicts nothing measured. It contradicts the
*inference* people draw from the headline.

---

## 6. A second finding, separable and cheaper to check

**The money gate may compute the streak burn at the wrong speed.**

The handoff streak forms in `CODED_DASH`, at `dash_speed_ms = 16.0`. Only after the streak
completes does `ENGAGE` begin and command `v_close_runin = 9.0` tapering to 5.5. The gate
computes **both** the burn and the time-to-go at 9 m/s.

Recomputed with the burn at dash speed:

| Configuration | Gate's t_go | Burn at 16 m/s | Burn at 13.2 m/s |
|---|---:|---:|---:|
| Tag `qd=2.0`, R90 7.10 m | 0.442 s | **0.172 s** | — |
| Tag `qd=1.0`, R90 8.97 m | 0.650 s | **0.380 s** | 0.488 s |

13.2 m/s is the along-line-of-sight closing speed for a perfect collision triangle at
16 m/s against a 9 m/s pure crosser (`√(16² − 9²)`).

Under any of those the planned `qd=1.0` tripod-day configuration drops **below the 0.5 s
gate threshold**, and `qd=2.0` collapses. This is a ~$740 decision, and the direction of
the error is the flattering one — the same shape as the invented 30 fps that ADR-0082
caught.

**This WAS a question and it has since been ANSWERED — §2.1 above closes it.** This
section was written saying "it is not verified from a log"; that sentence contradicted
§2.1 of the same document, which measures it. `scripts/forensics/handoff_closing_speed.py`
over the committed logs: **17.60 m/s** in the last second before handoff (n=14) against
the gate's 9.0 m/s, a **1.96×** under-pricing, and **14.64 m/s** after handoff pre-CPA
(n=13). So the concern is confirmed, at a ratio worse than the 16 m/s row above assumes.

What remains open is narrower: these are cue-era `DASH` flights, not `CODED_DASH`, so the
magnitude has to be re-read on the dev machine before it changes a published gate result
(`--phase CODED_DASH`). `placard_sizing.md` records that the 20 m/s scenario needs
R90 ≥ 13.3 m and *no candidate reaches it*, so the answer matters. Do that read before
tripod day, not after — and note the gate's assumption is deliberately **not** changed
here, because it can flip a PASS/FAIL on a ~$740 order and that must be a logged
decision, not a side effect.

---

## 7. What follows, and what does not

**Does not follow:** that the camera works, that the vertical fix will land, or that any
published number changes. Nothing here has flown.

**Does follow:**

1. The strongest camera-versus-dash comparison has never been run, because the guidance
   was blind to 82% of the squared miss.
2. The cheapest known path from 5/16 to inside the ram radius is a vertical channel that
   was already decided.
3. Quoting the ADR-0023 bound at correct aim is a regime error by this project's own rule.

**Recommended, unratified — the builder's call:**

- Build the vertical channel as the **rate** form, pro-nav on elevation, not the
  pixel-centring form. M4 measured the rate form beating the angle form **4.6–7.6×** on
  the horizontal axis; there is no reason to expect the vertical axis to differ. Keep
  ADR-0085's hard AGL floor, which is a flight-safety item independent of the guidance.
- Sequence it **after** the past-CPA breakoff fix (live queue item 1), because that defect
  is arm-asymmetric and confounds any camera-versus-dash margin.
- **Pre-register it**, including what a null means: if a vertical channel does not tighten
  the paired camera arm, the vertical bias is a datum or altitude-hold problem rather than
  a guidance one, and the fix moves to the altitude loop.
- Read the actual handoff closing speed out of the per-tick archive before tripod day.

---

*This is a derived analysis, not part of the contract. Its verifiable half is recorded as
contradiction `terminal-vertical-channel-decided-not-built`; delete this file once that
entry resolves.*
