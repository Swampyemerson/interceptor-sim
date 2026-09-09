# Why a late camera correction has never saved the aim — and why that is not physics

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

So the ratio inverts. Capacity at the planned tripod-day decode setting is **0.6 to
1.8 m** against a need of roughly **0.4 m** — capacity exceeds need by two to four times.

Re-using a threshold across that gap is exactly the failure mode CLAUDE.md already has a
standing rule against: *a threshold validated at one operating point is NOT validated at
another*. The bound was earned at 1.69 m of delivered error. It is being spent at 0.41 m.

### 2.1 Capacity table (`scripts/forensics/terminal_capacity.py`)

Handoff needs `k=5` consecutive decodes, so the range burned forming the streak uses the
run-length expectation `E[T] = (1−pᵏ)/(pᵏ(1−p))` (ADR-0079), not the mean rate. `a = 8.7
m/s²` is the *measured* median lateral acceleration in ENGAGE (`terminal_diagnosis.md`).

**As the money gate computes it** (burn and time-to-go both at 9 m/s):

| Configuration | Burn | t_go | Capacity |
|---|---:|---:|---:|
| Tag `qd=2.0`, R90 7.10 m, 20 Hz loop, p=0.9 | 3.12 m | 0.442 s | **0.85 m** |
| Tag `qd=1.0`, R90 8.97 m, 20 Hz loop, p=0.9 | 3.12 m | 0.650 s | **1.84 m** |
| Tag `qd=2.0` pessimistic, R90 5.98 m, 14 Hz | 4.46 m | 0.169 s | 0.12 m |

Every row except the pessimistic corner clears the ~0.4 m that matters. The middle row
is the planned tripod-day setting.

**Sensitivity to lateral authority**, at t_go = 0.65 s:

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

### 3.3 A mechanism for the bias, testable offline

A P-only altitude loop cannot null a persistent disturbance; it settles at whatever error
commands enough climb. During the dash the vehicle pitches nose-down 27–36°, so the
vertical thrust component falls to `cos(35°) ≈ 0.82` of hover. A systematic sag is exactly
what that predicts, and the README already describes the symptom as *the interceptor flies
low*. That is a hypothesis, not a result — it is checkable from the existing per-tick
archive with no new flights.

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

**This is a question, not a claim.** It is not verified from a log: the actual closing
speed at handoff, and whether the vehicle is already decelerating as the streak forms, are
both measurable from the existing per-tick archive. `placard_sizing.md` already records
that the 20 m/s scenario needs R90 ≥ 13.3 m and *no candidate reaches it*, so the answer
matters. Do that read before tripod day, not after.

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
