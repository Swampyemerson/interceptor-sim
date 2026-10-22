# EKF target-track upgrade — design brief (post-M5 queue item 3, ADR-0033)

> **Status:** DESIGN ONLY. No code changes now. Implementation is gated behind the
> M5 final batch being captured on a frozen config (ADR-0033 item 0). This document
> is the design + the interview-prep dossier; it is meant to be read cover-to-cover
> before a single line of the estimator is written.
>
> **Two audiences, one document.** This is engineering *and* interview preparation in
> equal measure (ADR-0033 item 3: "The Kalman filter is the most-asked GNC interview
> topic"). Every design choice below is written so you can **derive it from first
> principles and defend it out loud** — not just cite it. Where a number appears, its
> source is named (a repo file, an ADR, or a cited result). New terms get a one-line
> definition the first time they appear.

---

## 0. The one-paragraph executive summary (read this even if you read nothing else)

We currently estimate the target track with a pair of **alpha-beta filters** (a
fixed-gain, constant-velocity smoother — defined in §1). The proposal is to replace
them with a proper **Kalman filter** (KF) — or, because one measurement channel is
nonlinear, an **Extended Kalman Filter** (EKF) — and A/B it against the alpha-beta
baseline using the project's own statistical discipline (paired seeds, n ≥ 8,
mechanism evidence, honest "not significant at this n" language). **The most likely
outcome is a NULL on end-to-end miss**, because ADR-0023 proved the miss is
*kinematic* — 96% locked at handoff, `r²(ZEM, miss) = 0.990` — not
estimator-limited. That predicted null is *not* a failure of the exercise: a
carefully-run EKF A/B that improves track quality but not miss, and that *explains
why*, is exactly the "sophisticated, defensible answer" ADR-0033 asks for. The
precedent is Kalata (ADR-0013): a filter upgrade that won big in the lab, lost in
Gazebo, and is now a documented, reproducible negative result we can talk about
intelligently. This is the same playbook.

---

## 1. Primer — what we have, and what a Kalman filter actually is

### 1.1 What the current tracker does (the baseline you are replacing)

You cannot argue for the upgrade without describing the incumbent precisely. Here is
the alpha-beta tracker exactly as it lives in `scripts/m4_intercept.py`.

**An alpha-beta filter** (also called a *g-h filter*) tracks one scalar quantity that
is assumed to move at roughly constant velocity. It holds two numbers — an estimate
of the value `x_hat` and an estimate of its rate `xdot_hat` — and runs two steps:

- **predict** (every control tick, 20 Hz — `CONTROL_RATE_HZ = 20.0`):
  `x_hat += xdot_hat * dt`. Coast the estimate forward on the current rate.
- **correct** (only on ticks where a genuinely fresh camera detection arrived):
  form the residual `r = measurement − x_hat`, then nudge
  `x_hat += alpha * r` and `xdot_hat += beta * r / dt_since`
  (`AlphaBetaFilter.correct`, `m4_intercept.py:662`).

The project runs **two independent scalar alpha-beta filters** (the tracker the task
asked me to find):

| channel | tracks | gains (`m4_intercept.py:237–246`) |
|---|---|---|
| `lambda_filter` (`angular=True`) | inertial LOS azimuth λ = ψ + β | ALPHA = 0.5, BETA_GAIN_LAMBDA = 0.30 |
| `range_filter` | range R to the target | ALPHA = 0.5, BETA_GAIN_RANGE = 0.15 (FPV: 0.45) |

- **λ = ψ + β** is the strapdown-seeker trick (module docstring, `m4_intercept.py:9–34`;
  pronav SKILL.md line 22): the camera measures **bearing β** (angle of the tag off the
  nose, from the AprilTag pose via `atan2(x, z)` in the camera frame,
  `m3_static_intercept.py:284`), and **ψ** is the vehicle's own yaw from PX4's EKF
  (own-state, legal per the ADR-0008 honesty boundary). Because the yaw loop actively
  centers the tag, raw `dβ/dt ≈ 0`; only the *inertial* azimuth λ has a real rate.
- **Range** comes from the same AprilTag pose — `range_m = ‖relative position‖`
  (`m3_static_intercept.py:282`). So the camera actually delivers a full 3-D relative
  position vector; the guidance loop consumes only its polar projection (bearing +
  range).
- **What pro-nav needs from all this:** `a_cmd = N · Vc · lambda_dot_hat`
  (`m4_intercept.py:1716`; pronav SKILL.md line 14), where `Vc = −rdot_hat` floored
  and N = 4 (M4) / 5 (FPV). **The single output that matters is `lambda_dot_hat`, the
  LOS rate.** Everything about estimator quality reduces to: *how clean and how prompt
  is that rate, especially through the ratty terminal cadence.*

Two more estimators ride alongside, both built from the same `AlphaBetaFilter`:

- **`TargetTracker`** — two alpha-beta filters on the target's absolute (north, east)
  position (gains 0.6 / 0.2, `PIP_TRACK_ALPHA/BETA`), giving a position+velocity
  estimate for the Predicted-Intercept-Point lead law and the dash
  (`m4_intercept.py:707`).
- **`FusedTrack`** — a bearing-weighted *polar* fusion used pre-handoff when
  `--fuse-midcourse` is on: the **camera owns the angle**, the **cue's range folds in
  inverse-variance-weighted**, the cue's emitted velocity is used when fresh, and the
  handoff rates are *geometric* (λ̇ = cross(û, v_rel)/R) rather than the
  still-converging alpha-beta rates (`m4_intercept.py:805–895`).

**The handoff warm-start** (`--warm-handoff`, `m4_intercept.py:1951–2017`): at the
one-way cue latch, the terminal filters (λ, λ̇, R, Ṙ), `v_perp`, and the PIP track are
*seeded* from the best available track (fused polar state, or the dash tracker), so the
terminal does not fly its first ~1–2 s on cold, still-converging rate states. Note the
tell: warm-start can seed the **state** but not the **confidence** — alpha-beta has no
covariance to transfer. Hold that thought; it becomes the cleanest EKF win in §3.

**No innovation gating exists today.** `correct()` accepts every fresh measurement.
That is deliberate: a fiducial has a near-zero false-positive rate, and ADR-0013 found
range-outlier gating *negative* in the lab ("nothing to catch in Gaussian noise").

### 1.2 What a Kalman filter is (and why alpha-beta is a special case of it)

A **Kalman filter** is the recursive, optimal (minimum-mean-square-error) estimator for
a **linear system driven by Gaussian noise**. It carries not just a state estimate `x`
but a **covariance `P`** — a running, quantified statement of *how uncertain it is about
every state and how those uncertainties correlate*. That covariance is the entire point;
it is what alpha-beta throws away.

The two steps, with the standard matrices (state `x`, dynamics `F`, process-noise `Q`,
measurement `z`, measurement model `H`, measurement-noise `R`):

**Predict** (propagate the estimate and *grow* its uncertainty by the process noise):

```
x⁻ = F x                     (coast the state on the motion model)
P⁻ = F P Fᵀ + Q              (uncertainty grows — Q says how much the target can surprise us)
```

**Update** (fold in a measurement, weighted by how much you trust it vs. the model):

```
y = z − H x⁻                 innovation  (the measurement "surprise")
S = H P⁻ Hᵀ + R              innovation covariance (how surprised we EXPECTED to be)
K = P⁻ Hᵀ S⁻¹                Kalman gain  (the optimal blend)
x = x⁻ + K y                 correct the state
P = (I − K H) P⁻             uncertainty SHRINKS (we just learned something)
```

Read the gain `K = P⁻ Hᵀ S⁻¹` physically. If the measurement is noisy (`R` large), `S`
is large, `K` is small → trust the model, barely move. If the state is uncertain (`P⁻`
large), `K` is large → trust the measurement, snap to it. **The gain is not a tuning
constant; it is computed every step from the current uncertainty and the sensor's
known noise.** That is the whole difference from alpha-beta.

### 1.3 Alpha-beta is a Kalman filter with the covariance deleted and the gain frozen

Here is the derivation that unlocks the entire interview topic. Take a Kalman filter
for a 1-D **constant-velocity** model — state `[position, velocity]`,
`F = [[1, T],[0, 1]]`, measuring position only (`H = [1, 0]`). Run it at a **fixed
sample interval T with fixed Q and R**. The covariance `P` and therefore the gain `K`
**converge to constant steady-state values**. Those two steady-state gain components
*are* alpha and beta: α multiplies the residual into position, β/T into velocity.

So:

> **Alpha-beta = the steady-state Kalman filter for a constant-velocity target,
> evaluated at a fixed sample rate and fixed noise, with the gain frozen at its
> converged value and the covariance bookkeeping discarded.**

The map from noise assumptions to (α, β) has a closed form — the **Kalata tracking
index** `Λ = σ_process · T² / σ_meas` (ratio of maneuver noise to measurement noise;
Kalata 1984, ported verbatim in `kalata_alpha_beta`, `m4_intercept.py:569`). Large Λ
(twitchy target, clean sensor) → large α, β → trust measurements. Small Λ → smooth,
trust the model.

This immediately tells you **exactly what a real KF buys over our alpha-beta**, because
it buys back the two assumptions we just listed as fixed:

1. **Variable sample interval T.** A KF re-derives its gain from `P`, which has grown
   by `Q·Δt` over a long gap. Our alpha-beta uses a *constant* α, β regardless of the
   gap. (Kalata *tried* to fix this by recomputing α, β from the actual `dt` each
   correction — and it **blew up in Gazebo**; §3 explains why the KF's `P`-based
   version is the principled fix where Kalata's closed-form hack was not.)
2. **Time-varying measurement noise R.** Our camera range noise is
   *range-dependent* (σ ≈ frac · R). A KF folds that into `R` every update; a
   fixed-gain filter cannot.

### 1.4 Is this problem even nonlinear? (KF vs EKF — walk the linearization)

An **Extended Kalman Filter (EKF)** is a KF for a *nonlinear* system: where the
measurement `z = h(x)` is not a linear matrix multiply, you replace `H` with the
**Jacobian** `H = ∂h/∂x` evaluated at the current estimate — a first-order Taylor
linearization, recomputed every step.

**Is our problem nonlinear? It depends on how you parameterize the state — and the
*fused* problem is nonlinear no matter what.** This is a genuinely good thing to be able
to reason about in an interview:

- **Cartesian state, polar camera → nonlinear (needs an EKF).** Let the state be the
  target's `[n, e, vn, ve]` in NED. The camera measures **bearing and range**, which
  are polar. With own position `(n_o, e_o)` known (own-state, legal):

  ```
  dn = n − n_o,   de = e − e_o,   R = √(dn² + de²)
  h(x) = [ λ ,  R ]  =  [ atan2(de, dn) ,  √(dn² + de²) ]
  ```

  The Jacobian:

  ```
        ∂λ/∂n   ∂λ/∂e   ∂λ/∂vn  ∂λ/∂ve       −de/R²   dn/R²   0   0
  H  =                                    =
        ∂R/∂n   ∂R/∂e   ∂R/∂vn  ∂R/∂ve        dn/R    de/R    0   0
  ```

  **Look at `∂λ/∂position ~ 1/R`.** The bearing's information about *cross-range
  position* grows without bound as `R → 0`. That is the geometric dual of the LOS-rate
  singularity the project already handles by freezing the command near CPA
  (`TERMINAL_FREEZE_RANGE_M`, `m4_intercept.py:296`; ADR-0023). **An EKF inherits this
  singularity in its own linearization — it does not rescue the terminal.** This is the
  single most important honest point about the EKF: the R→0 blow-up that produces the
  kinematic miss is *not* an estimator artifact you can filter away; it is geometry, and
  it bites the EKF's `H` exactly where it bites the alpha-beta λ̇.

- **Polar state, polar camera → linear (a plain KF suffices for the camera channel).**
  Let the state be `[λ, λ̇, R, Ṙ]` — which is *exactly what the two alpha-beta channels
  already track*. Now the camera measurement is `z = [λ_meas, R_meas]` with
  `H = [[1,0,0,0],[0,0,1,0]]` — constant, linear. The nonlinearity has moved into the
  **motion model** instead: true polar kinematics couple the channels
  (`λ̈ = −2 Ṙ λ̇ / R`, the "Coriolis"-like term), which a KF can carry and our two
  *independent* alpha-beta filters silently ignore.

- **The cue makes it nonlinear either way.** The ground cue measures **Cartesian
  position (and optionally velocity)**. If the state is Cartesian, the cue update is
  linear but the camera update is nonlinear; if the state is polar, the cue update is
  nonlinear. **A fused camera + cue estimator has one nonlinear channel regardless →
  EKF (or a polar-state KF with a nonlinear cue update).** That is the honest answer to
  "is it nonlinear": each *single* source can be made linear by matching the
  parameterization, but *fusing the two* cannot.

---

## 2. Design options (state, frames, measurement models, tuning, init, gating)

### 2.1 State vector — CV vs CA, Cartesian vs polar

Four candidate parameterizations, ranked for this problem:

| option | state | camera update | cue update | notes |
|---|---|---|---|---|
| **A (recommended baseline EKF)** | Cartesian CV `[n, e, vn, ve]` | nonlinear (EKF) | linear | this is the existing `TargetTracker` *with a real covariance*; LOS rate is a **derived** output |
| B | Polar CV `[λ, λ̇, R, Ṙ]` | linear (KF) | nonlinear | closest KF analog of today's two filters, now *coupled* with a joint P + the λ̈ term |
| C | Cartesian CA `[n, e, vn, ve, an, ae]` | nonlinear (EKF) | linear | models a maneuvering target; +2 states that Q must excite |
| D | Bias-augmented A `[n, e, vn, ve, b_n, b_e]` | nonlinear (EKF) | linear + bias | estimates the cue **datum bias** (§2.3) as extra states |

**Recommendation: start with Option A (Cartesian CV EKF), pre-register Option C
(CA) and Option D (bias-augmented) as A/B arms.** Reasons:

- Option A reuses the *exact* geometry the code already trusts: own-NED position + the
  camera relative vector, with the world→NED mapping `north = world_y, east = world_x`
  empirically nailed in ADR-0013 (`m4_intercept.py:125–158`). It is the "4-state CV
  Kalman" ADR-0013 already prototyped in the lab and shelved as "mixed vs the simpler
  fixes; kept lab-only for future Gazebo fat-tail work" — so we are resuming a *known,
  documented* candidate, not inventing one.
- **LOS rate becomes a derived, covariance-aware quantity** instead of a numerical
  derivative of a filtered angle:
  `λ̇ = (dn·(ve − ve_o) − de·(vn − vn_o)) / R²`. This is *precisely* what
  `FusedTrack.state()` already computes geometrically (`m4_intercept.py:893`). A
  Cartesian velocity estimate gives a cleaner λ̇ than differentiating λ, because the
  target-velocity state absorbs measurement noise before it reaches the rate — and pro-nav
  consumes the rate.

**CV vs CA (constant-velocity vs constant-acceleration).** CA adds `[an, ae]` so the
filter can *follow* a maneuver instead of lagging it. But every extra state must be
"excited" by process noise, and against a straight-line crosser CA mostly adds a noisier
velocity estimate for no benefit. **Decision rule, honestly stated:** choose CA only if
the M5 path suite actually maneuvers hard enough to matter. It does contain S-weave and
jink schedules (`mc_batch.sh --path weave`, `WEAVE_LAT_SPEED = 3.0`; ADR-0033 item 0b),
so **run CA as its own A/B arm on the weave/jink paths** and let the paired delta decide
— do not adopt it by assertion.

### 2.2 Frames (agree once, per docs/goals.md)

- **Target state in PX4 local NED** — the guidance/tracker frame. World→NED:
  `north = world_y, east = world_x`, no sign flip (ADR-0013, empirically verified two
  independent ways; `m4_intercept.py:125–158`). PX4's local NED origin is the spawn
  point, so no translation term.
- **Own position/velocity from PX4's EKF** (`state.pos_n/e`, `state.vel_n/e`) —
  own-state, legal per the honesty boundary. It appears only in `h(x)` (the relative
  geometry) and never as target information.
- **Camera bearing enters as inertial LOS azimuth** λ = ψ + β, ψ from
  `attitude_euler()` (`m4_intercept.py:1590`). Same sign convention as today.
- **Honesty boundary re-earned.** `gt_*` columns (`gt_cam_*`, `gt_tag_*`, `gt_range`)
  are scoring/logging ONLY; the EKF reads camera + cue + own-state, never `gt_*`. Every
  new estimator path re-runs the ADR-0008 grep-for-`gt_` no-cheat audit and the S2
  command-vs-LOS correlation audit (`check_s2.sh` audit (c)).

### 2.3 Measurement models — the two channels, and their *real* error structure

This is where a proper filter earns its keep, because the KF's `R` matrix is where the
sensor physics live. The measurement models must encode the error structure ADR-0015/
0016/0017 documented — not a flat noise number.

**Camera channel** `z_cam = [λ_meas, R_meas]`:

- **Bearing σ:** sub-degree for the AprilTag in-sim; **1–2° for the real ML seeker**
  (ADR-0015 row 9). This is the clean channel — it is why the design keeps the camera as
  the angle authority everywhere (FusedTrack, pronav).
- **Range σ is RANGE-DEPENDENT:** AprilTag ~5–8% of R in-sim (`FUSE_CAM_RANGE_FRAC = 0.10`
  is the conservative match, `m4_intercept.py:800`); **15–30% for a real ML box**
  (ADR-0015 row 8). So `R_cam` must be rebuilt every update: `σ_R = frac · R_hat`, i.e.
  the measurement-noise entry *changes with the state*. A KF does this natively; a
  fixed-gain alpha-beta structurally cannot. **This is principled-mechanism #1.**

**Cue channel** `z_cue = [n_cue, e_cue]` (+ `[vn_cue, ve_cue]` when
`--emit-velocity`):

- **Velocity emission is the #1 lever in the whole system** (ADR-0015 2nd addendum,
  ADR-0030): a cue that *emits* a filtered velocity (σ_v ≈ 0.5 m/s) beats one that makes
  the drone *differentiate* a noisy position stream (~7 m/s of velocity noise, a
  PIP-killer). In KF terms this is a **direct velocity measurement** (`H` has a row on the
  velocity states) instead of relying on the filter to infer velocity through the
  position residual — cleaner and lag-free. The EKF must ingest it as a first-class
  measurement, exactly as `TargetTracker.set_velocity` does today (`m4_intercept.py:725`).
- **Range σ_R(R) = a + c·R²** with the **ADR-0017-corrected** constants: `(a, c) =
  (0.000, 4.45e-05)` EXPECTED (the old `s2_cue_mock` curve `0.4 + 0.008·R²` was ~180×
  too steep as *stereo noise*; ADR-0017, adopted per ADR-0033 item 0a).
- **The cue error is BIAS-dominated, not noise-dominated — and this violates a core KF
  assumption.** ADR-0017's correction is that the 2–4 m handoff-range cue error is not
  stereo noise; it is the **GPS datum / clock offset — a per-run *constant* offset**
  (0.3 / 0.5 / 2.5 m for RTK / shared-RTK / standard-GPS; ADR-0015 row 1, ADR-0016). A
  Kalman filter assumes **zero-mean** measurement noise. A constant bias is *not*
  zero-mean, so a vanilla KF that models the cue as pure Gaussian noise **will bias the
  state toward the datum-offset track.** Two honest treatments, both worth an A/B arm:
  1. **Inflate `R_cue`** to swallow the bias budget (crude — RSS the noise σ with the
     datum budget, exactly as `FusedTrack` already does:
     `σ_cue = √(σ_R(R)² + FUSE_DATUM_BUDGET_M²)`, `m4_intercept.py:863`). Loses
     information but is trivial and honest.
  2. **Estimate the bias as state** (Option D, bias-augmented EKF): add `[b_n, b_e]`
     to the state; the cue measures `position + bias`, the camera measures
     `position` (no bias). The bias is **observable only when both sources see the
     target simultaneously** — i.e. inside the pre-latch fusion window — which is a
     clean, interview-grade observability argument (§5, Q7). This is the *principled*
     way to honor the ADR-0017 error structure.
- **Latency / OOSM.** The cue is delivered **0.12 s mean + 0.05 s jitter** late
  (ADR-0015 row 3, budget confirmed BEST 20 / EXPECTED 90 / WORST 210 ms in ADR-0016;
  add the 0.20 s WORST tier per ADR-0033 item 0a). A measurement stamped in the past is
  an **out-of-sequence measurement (OOSM)**. The textbook-correct handling is
  **retrodiction**: roll the state/covariance back to the measurement's timestamp,
  update there, roll forward (Bar-Shalom OOSM). The project already ships a *lite*
  version — advance the stale sample forward along the velocity estimate by its known
  age (`--cue-latency-comp`, `m4_intercept.py:1673`; measured `ext_age_s` 0.116–0.152 s,
  ADR-0013). A KF gives the principled version because it *has* the covariance to roll
  back. **Principled-mechanism #3.**

### 2.4 Process noise Q — how to tune it *defensibly*, not magically

`Q` answers one question: **"how far can the target's true motion deviate from my
constant-velocity model between updates?"** Tune it from physics, not by eyeballing a
trajectory.

For a CV model with a **continuous white-noise-acceleration (CWNA)** driving term, the
per-axis process-noise block is standard (Bar-Shalom):

```
Q_axis = q · [[ T³/3 ,  T²/2 ],
              [ T²/2 ,   T   ]]
```

where `q` is the **power spectral density of the acceleration noise** (units m²/s³),
and T is the interval since the last update. Set `q` from the **worst credible target
maneuver in the M5 path suite**: for a weave at `WEAVE_LAT_SPEED = 3.0 m/s`
(`mc_batch.sh`) and weave period τ, the peak lateral accel is `a_max ≈ (2π/τ)·v_lat`;
choose `σ_a ≈ a_max` and `q ≈ σ_a²·τ_correlation`. **Every number traces to the path
suite's own maneuver schedule — a sim-measurable quantity — not a knob turned until the
plot looked nice.** (This is the "simulate worse than ideal / every realism knob maps to
a bench-measurable quantity" mandate applied to Q.)

**Validate Q the disciplined way — filter consistency, not eyeballing.** After picking
Q and R, run a batch and check the filter's own **NIS** (Normalized Innovation Squared,
`NIS = yᵀ S⁻¹ y`) and **NEES** (Normalized Estimation Error Squared, against `gt_*`,
scoring-only). For a well-tuned filter, average NIS ≈ the measurement dimension and NEES
≈ the state dimension, and both sit inside their chi-square confidence bands. If NIS runs
high → Q too small (filter overconfident, ignoring measurements → the classic
divergence path). If NIS runs low → Q too big (filter sluggish and noisy). **This is the
defensible replacement for hand-tuning gains** — and it is a strong thing to say you did.

**The Kalata cautionary tale, stated precisely (ADR-0013, the precedent this work must
not repeat naively).** Kalata *also* tried to make the gain adapt to `dt` — by
recomputing α, β from a closed-form tracking index at the actual interval each
correction. It **won −24/−28% in the lab and then lost every Gazebo flight** (3.24 vs
the 1.09–2.25 m no-flag cluster). Mechanism, caught directly in the logged effective
gains: Gazebo's real correction cadence is **bimodal** — ~14 Hz bursts *and*
multi-second dash/handoff gaps — and the lab-tuned σ's went **degenerate at both
extremes**: the range channel hit α = 0.031 / β ≈ 0 at burst cadence (rate filter goes
*deaf* → `Vc` pinned at floor → `a_cmd = N·Vc·λ̇` starved of lead exactly like the
ADR-0009 pathology), and the λ channel hit α = 0.999 / β = 1.876 after multi-second gaps.
**Why the KF is the principled fix where Kalata was the naive one:** the KF's gain comes
from the *running covariance P*, which is bounded and physically meaningful — after a
long gap P has grown by `Q·Δt` so the gain rises *but stays tied to actual predicted
uncertainty*, and at burst cadence P shrinks and the gain settles *without going deaf*.
Kalata evaluated a steady-state closed form at a wildly off-nominal `dt`; the KF never
assumes steady state. **This is the headline "what could improve," and the interview
story writes itself.** But — see §3 — winning the *conditioning* argument does not
guarantee winning the *miss* argument.

### 2.5 Initialization and the handoff warm-start

- **Cold init.** First camera measurement sets position; velocity is unknown, so
  initialize `P` with a **large velocity variance** (say σ_v ≈ the full target speed
  band, ~12 m/s). This is strictly cleaner than alpha-beta's implicit "position = meas,
  rate = 0 with unstated confidence," which takes ~1–2 s to settle — and ADR-0023
  quantified that settle as part of the recoverable ~20–25% mechanization loss.
- **Warm handoff — seed P, not just x.** This is the clean, small, *honest* EKF win.
  Today's warm-start seeds the *state* (λ, λ̇, R, Ṙ, PIP pos/vel) from the fused/dash
  track (`m4_intercept.py:1980–2011`) but cannot seed *confidence*, so the terminal
  filter effectively "re-learns" that its velocity is already good. A KF seeds `x̂` **and**
  `P̂` — the fused track's velocity came from an *emitted* cue velocity with a *known*
  σ_v = 0.5, so seed a **small** velocity covariance. The terminal then does not spend its
  first corrections re-earning confidence it was handed. This directly attacks the
  filter-settle slice of the mechanization loss — **the one slice §3 says is actually
  addressable.** Stay honest about size: ADR-0023 says the *whole* mechanization loss is
  ~0.3–0.45 m, and filter-settle is only part of it.

### 2.6 Innovation gating for the dropout bursts

With `S` in hand, gating is free: reject a measurement whose **NIS = yᵀ S⁻¹ y** exceeds
a chi-square threshold (e.g. χ²(dof, 0.99)). The natural targets are the **Markov
dropout-burst re-acquire glitches** (ADR-0015 row 4: bursty 0.5–3 s outages, p_out
5–20%) and any real-seeker fat-tail outlier (ADR-0015 rows 7–9). **Honesty flag,
pre-registered:** ADR-0013 already found range-outlier gating *negative* in-sim
("nothing to catch in Gaussian noise" — the AprilTag is clean). So in the current sim,
gating is **likely inert (a null)**; it earns its keep only under the real seeker's fatter
tails or the cue's re-acquire glitches. Build it, flag it, expect a null in-sim, and say
so — same treatment as Kalata.

---

## 3. What could ACTUALLY improve vs alpha-beta — and what will NOT

### 3.1 Honest mechanism list (where a KF/EKF can genuinely help)

1. **Principled covariance under variable-rate / dropout measurements.** The
   `P`-carried gain is the correct fix for the bimodal-cadence degeneracy that killed
   Kalata (ADR-0013). *Strongest* mechanism — the KF is specifically the right tool for
   the exact thing that broke the naive adaptive-gain attempt.
2. **Range-dependent R + correct cross-source fusion weighting.** Inverse-variance
   weighting falls out of the Kalman gain automatically, *including cross-correlations*
   the hand-rolled polar `FusedTrack` ignores by treating channels independently.
3. **Latency compensation done right** — OOSM retrodiction vs. the current
   forward-extrapolation-lite `--cue-latency-comp`.
4. **Datum-bias estimation** (bias-augmented state, Option D) — models the ADR-0017 cue
   error structure honestly instead of swallowing a per-run constant bias in `R`.
5. **A consistent LOS-rate estimate *with* an uncertainty** — pro-nav gets a quality
   signal, and gating (§2.6) becomes possible.
6. **Warm-start confidence transfer** (seed `P`, not just `x̂`) — attacks the
   filter-settle mechanization slice (ADR-0023).

### 3.2 What will NOT change — the kinematic miss floor (say it plainly)

**The end-to-end miss is kinematic, not estimator-limited. A better filter will very
likely NOT move it. State this up front, in the README, and in the interview.**

ADR-0023 is unambiguous and its evidence is model-independent:

- **`r²(ZEM@freeze, miss) = 0.990`** — the miss is 96%+ determined at the handoff/freeze
  latch, *before the camera-only terminal even begins*. ("ZEM" = **zero-effort miss**:
  how far you'd miss if you stopped steering right now.)
- **The correction budget is physically too small.** `½·a·t_go² = ½·8.7·0.41² = 0.72 m`
  of correction capacity vs **1.69 m** of ZEM delivered at handoff. The terminal window is
  too short to fix the delivered geometry, *no matter how good the estimator is.*
- **A perfect terminal camera** (point-mass, 150 seeds) removes 100% of the dropout and
  cuts the miss only **25%** (1.003 → 0.755 m). An estimator upgrade is strictly *less*
  than a perfect camera.
- **Root-cause split of the ~1.4 m miss:** ~70% kinematic (delivered ZEM > terminal
  capacity), ~20–25% recoverable *mechanization* loss, ~2% FOV-escape, ~0%
  blur/scale/cadence. **The EKF can only touch part of that 20–25% mechanization slice**
  — and *most* of that slice is the freeze-latch discarding the last 0.20 s (a *control-logic*
  fix, not estimation) and the filter *settle* (realized only 0.28 m of an available
  0.72 m). Filter-settle is the sliver an estimator addresses.

The levers that actually move the miss are **acquisition range** (capacity scales with
`t_go²` — acquire at 12 m not 6.5 m and capacity goes 0.72 → ~4.3 m) and **mid-course
track quality** (which *sets* the delivered ZEM — the velocity-emitting cue and the dash
lead-law, ADR-0030). The EKF is neither of those. **Pre-registered expectation: NULL on
miss, possible real improvement on track RMSE / LOS-rate error.** That split *is* the
result.

### 3.3 The fusion capstone — where the ADR-0018 null can finally flip (the strongest arm)

The single most interesting EKF experiment is not the estimator swap in the *default*
(sequenced, either/or) pipeline — it is **fusion-ON + EKF + markerless seeker**, run as
one experiment. The reasoning unifies all three post-M5 threads (seeker, EKF, fusion),
and it hinges on *why* the earlier fusion result came out default-OFF.

**The ADR-0018 fusion null was measured under the two conditions rigged against fusion:**

1. **Clean AprilTag terminal data.** The camera was already sub-degree in bearing and
   ~5% in range, so mixing in a noisier ground cue could only *dilute* it — fusion had
   nothing to add because the camera was already excellent. (Real drones don't carry the
   tag; the markerless seeker, ADR-0033 item 2, makes the camera ~1–2° bearing / 15–30%
   range — now the cue genuinely carries information the camera lacks.)
2. **The fixed-gain alpha-beta tracker.** `FusedTrack` blends the two sources by a
   hand-set polar rule (§1.1) — it *cannot* weight them by how good each one actually is
   at that instant. That is blind fusion. **An EKF weights each source by its live
   covariance** (mechanism #2, §3.1), so as the cue degrades (far range, datum bias,
   stale link) the filter *automatically* drives its weight toward zero and the estimate
   rides the camera.

Flip *both* of those and the null can plausibly reverse. **This is the honest reason to
re-run the "settled" ADR-0018 result: it was measured where fusion could not win.**

**The adaptive fallback is native, not bolted on.** The intuition "fuse when the ground
cue helps, fall back to camera when it looks worse" is *exactly* what a correctly-specified
EKF does: (a) covariance-weighting demotes a high-uncertainty cue continuously, and (b)
the **innovation gate** (§2.6) *rejects a cue measurement outright* when it disagrees with
the prediction beyond the χ² threshold — a literal "this ground reading looks wrong,
ignore it" switch that falls out of the estimator, not a hand-tuned `if`. That is a strong
interview point: **you designed adaptive sensor fusion and the adaptivity fell out of the
covariance, not a heuristic.**

**Three caveats so this is not over-sold:**

- **The kinematic ceiling (ADR-0023) still caps the terminal miss.** So fusion's real
  payoff is **mid-course robustness — earlier lock, fewer dash-aborts — not the final
  meters.** And that is the *valuable* target: ADR-0030 showed the binding degraded-cue
  failure is the **dash track collapsing before handoff (33% of 9 m/s flights never
  reached handoff)**. Fusion+EKF attacking *that* failure mode is a stronger story than
  chasing the terminal miss. The headline metric here is **handoff-reach / latch rate and
  mid-course track RMSE**, not min-`gt_range` (consistent with ADR-0031's "under a degraded
  cue, handoff-rate is the honest headline").
- **The honesty boundary must hold — and gains a new, sharper obligation.** The project's
  headline is *comms-denied* terminal, so fusion stays **mid-course only** (exactly what
  `--fuse-midcourse` is scoped to) and the terminal still degrades to camera-only. New
  subtlety an EKF introduces that alpha-beta did not: **the filter must not smuggle
  cue-derived state or covariance *across* the handoff.** The existing `cue_reads_post_handoff=0`
  audit checks that no cue *reads* happen after latch — it must be **extended to "no
  cue-tainted filter state survives handoff"** (re-initialize, or provably decay, the
  cue's contribution to `x̂`/`P` at latch), or the jam-resistance claim quietly breaks.
  This is a first-class item for the no-cheat audit, not an afterthought.
- **It is a real test matrix, not a quick check** — fusion {on, off} × estimator {αβ, EKF}
  × seeker {tag, markerless}. Run a **focused ladder** (the diagonal that isolates each
  change), not the full cross.

**Sequencing:** this stays behind the M5 finish (frozen-config rule) and behind at least a
*prototype* markerless seeker, but it is the natural **capstone** that turns three separate
post-M5 items into one coherent result: *"I gave the seeker noisier data, gave the
estimator honest covariance, and the mid-course fusion that was worthless with a clean tag
became the thing that keeps the dash alive to handoff."* Origin: main-session synthesis
with the builder, 2026-07-07 (folded in from the live architecture Q&A thread).

---

## 4. A/B protocol (pre-registered — so the conclusion cannot be post-hoc)

The whole point is to run this with the discipline the project already enforces
(CLAUDE.md "Statistics before verdicts"; `mc_analyze.py` already implements the paired
test). **Write the success/null criteria BEFORE flying** so nothing is reverse-engineered
after seeing the misses (the ADR-0014 honesty boundary (b): never pick the bar after the
data).

### 4.1 Pairing and power

- **Paired seeds, n ≥ 8 per cell** (ADR-0033 item 3). Reuse `mc_batch.sh --master-seed 42`:
  identical cue seed + crossing geometry per run index across both arms, so the *only*
  difference between paired flights is the estimator. The arm difference is the two
  `--tracker` values; everything else byte-matches.
- **The significance test is already built.** `mc_analyze.py` computes the paired diffs,
  their sample SD `sd_d`, and requires the 95% CI `mean_d ± Z₉₅·sd_d/√n` to **clear zero
  entirely** before printing "favours" — otherwise "tied" (`mc_analyze.py:520–530`). Use
  it unchanged.
- **Power caveat, pre-stated.** Run-to-run terminal-dropout noise is ~1 m/flight
  (CLAUDE.md; ADR-0013). At n = 8 the paired test resolved the fusion delta's σ_Δ ≈
  0.18–0.21 m (ADR-0018 addendum) — so the *paired* design has more power than the raw
  per-arm spread, but still **cannot resolve a sub-~0.15 m effect at n = 8.** Since §3
  predicts the miss effect is ≈0, **n = 8 will almost certainly print "tied" on miss —
  and that is the honest, expected answer, not an inconclusive one.** A larger n (M5-batch
  power) would only tighten a CI around zero.

### 4.2 Cells to run

- **Estimator:** `--tracker {alphabeta, ekf}` (× the CA and bias-augmented arms if
  Option C/D are built).
- **Speeds:** 6 / 9 / 12 m/s (ADR-0033 item 0d — comparable with ADR-0029/0028/0030;
  supersedes the old 6/8/10 note).
- **Geometry:** `standard` L2R/R2L **and** `oblique_close` (exercises the never-tested
  east/world_x sign channel; ADR-0033 addendum). A frame/EKF sign error would surface as
  a systematic miss on `oblique_close`.
- **Cue tiers — this is where the mechanism should show if anywhere:**
  1. **Clean cue** (idealized) — baseline sanity.
  2. **Realistic cue** — the ADR-0030 config: `--sigma-range --datum-bias-m 0.5
     --emit-velocity --cue-velocity --latency-jitter-s 0.05 --dropout-markov`, with the
     ADR-0017-corrected constants.
  3. **WORST tier** — 0.20 s latency (ADR-0016) + heavier Markov dropout. **Pre-register
     this as the discriminating cell:** mechanism #1 (P-based gain through bimodal
     cadence) predicts the EKF's advantage, if real, shows up *here* — under stress —
     more than in the clean arm. If the EKF beats alpha-beta *anywhere*, expect it here.
- **The deployment/running-start geometry** (`--y0-mag 29.3 --dash-speed 16
  --dash-unclamp`, ADR-0028/0030) so the A/B rides the *adopted* profile, not the legacy
  short-standoff one.
- **Fusion arm (the §3.3 capstone) — the strongest cell, run once a markerless-seeker
  prototype exists:** add `--fuse-midcourse {off, on}` crossed with `--tracker {alphabeta,
  ekf}` and seeker `{tag, markerless}`, as a *focused ladder* not the full cross. Primary
  metric shifts to **handoff-reach / latch rate + mid-course track RMSE** (§3.3), not
  terminal miss. Requires the extended "no cue-tainted filter state survives handoff"
  audit before any number is trusted.

### 4.3 Metrics

- **PRIMARY (expect NULL):** end-to-end miss = min `gt_range`; Pk-vs-R per arm with
  Wilson 95% CIs (`mc_analyze.py`); paired-delta 95% CI on miss.
- **SECONDARY (where the mechanism lives — expect the real signal here if anywhere):**
  - **Track RMSE vs `gt_*`** (scoring-only, honesty boundary): position RMSE and
    velocity RMSE of the target estimate. Reconstruct truth from `gt_tag_*`/`gt_cam_*`.
  - **LOS-rate error** — `lambda_dot_hat` (logged) vs the true LOS rate reconstructed
    from `gt_cam_*`/`gt_tag_*`. **This is the pro-nav quality metric** (pronav SKILL.md:
    "LOS rate quality is what matters") — the estimator's whole job.
  - **Handoff latch rate** — does the estimator change acquisition timing? (Under
    degraded cue, handoff-rate is the *honest headline* metric, not mean miss — ADR-0031.)
  - **Terminal filter-settle time** and **NIS/NEES consistency** — is the EKF's own
    covariance honest? (An EKF that is *inconsistent* is worse than a tuned alpha-beta.)
- **The honest split, pre-declared:** *if* track RMSE / LOS-rate error improve but the
  miss delta straddles zero, **that is the expected result and a clean portfolio story** —
  it isolates "the estimator got better, but the kinematics dominate the miss" (ADR-0023).

### 4.4 Pre-registered SUCCESS / PARTIAL / NULL criteria

Fix these now, before any flight:

- **SUCCESS → adopt EKF as the default** (`--tracker ekf`): the paired-delta miss 95% CI
  **excludes zero and favors EKF** at ≥ 1 speed cell, **with a mechanism trace** (lower
  LOS-rate error on the *same* paired flights); **AND no regression** (no cell's
  paired-delta CI favors alpha-beta); **AND** the S2 gate still passes byte-clean.
- **PARTIAL → keep EKF opt-in** (`--tracker ekf`, documented): track RMSE and/or
  LOS-rate error improve significantly, but the end-to-end miss delta **straddles zero**.
  This is the **most likely outcome** and is the ADR-0023-kinematic-floor-holds result —
  a fully defensible portfolio/interview answer.
- **NULL / REJECT → keep alpha-beta default, document EKF as tested-and-rejected**
  (exactly like Kalata, ADR-0013): no significant improvement on *any* metric, or *worse*
  conditioning (NIS/NEES inconsistent, terminal divergence). A reproducible negative
  result is a *result*, not a wasted effort.

### 4.5 Discipline (non-negotiable, from CLAUDE.md / MEMORY)

Batches at **idle machine load only** (matched-load rule; ADR-0015 confound). One sim at
a time, arms sequential (`mc_batch` self-kill hazard — never `pkill` from a shell whose
args contain the batch script name). Every EKF path **re-earns the numeric no-cheat
audit** (`gt_*` grep). Report **"not significant at this n"** wherever the paired CI
straddles zero — do not launder a directional delta into a claim.

---

## 5. Interview appendix — the 8–10 most-asked Kalman questions, answered *from this project*

Each answer is 2–4 sentences, grounded in numbers you can defend.

**Q1. Derive the Kalman gain. Why `K = P⁻Hᵀ(HP⁻Hᵀ+R)⁻¹`?**
The gain is chosen to minimize the trace of the posterior covariance `P⁺ =
(I−KH)P⁻(I−KH)ᵀ + KRKᵀ`; setting `d tr(P⁺)/dK = 0` gives `K = P⁻HᵀS⁻¹` with `S =
HP⁻Hᵀ+R`. Physically it is an information-weighted blend: small when the sensor is noisy
(`R` large → `S` large), large when the state is uncertain (`P⁻` large). In this project,
our alpha-beta gains (λ: 0.5 / 0.30, `m4_intercept.py:239–245`) *are* the frozen
steady-state `K` of a constant-velocity KF — so I already ship the Kalman gain, just
hard-coded.

**Q2. What is the innovation, and why does it matter?**
The innovation `y = z − Hx⁻` is the measurement "surprise" — the part of the measurement
the prediction didn't anticipate; it is what drives every correction, and its covariance
`S` tells you how surprised you *expected* to be. `S` is the yardstick for both gating
(reject `yᵀS⁻¹y` above a chi-square threshold) and consistency checking. Our alpha-beta
already stores the residual (`last_innovation`, `m4_intercept.py:689`) but throws away
`S`, so it can neither gate nor self-check.

**Q3. Alpha-beta vs Kalman — when is each right?**
Alpha-beta *is* the steady-state KF for a constant-velocity target at a fixed sample rate
and fixed noise. It's the right, cheap choice when both hold; it fails when the interval
varies (dropouts) or the noise varies (range-dependent σ) — exactly our two conditions.
Our correction cadence is *bimodal* (~14 Hz bursts + multi-second gaps, ADR-0013), so the
fixed gain is a compromise, and the naive per-`dt` adaptive fix (Kalata) diverged — which
is precisely the case for a `P`-carrying KF.

**Q4. Is this problem nonlinear? KF or EKF?**
The camera measures **polar** (bearing, range) quantities of what is naturally a
**Cartesian** target state, and that map is nonlinear → EKF (linearize `h` via its
Jacobian each step). You *can* dodge it for the camera alone by using a polar state, but
then the ground cue's Cartesian measurement becomes the nonlinear one — so a *fused*
camera+cue estimator is nonlinear either way. Crucially `∂λ/∂position ~ 1/R` blows up as
`R→0`, the linearization dual of the λ̇ singularity, which is exactly why we freeze the
command near CPA (`TERMINAL_FREEZE_RANGE_M`; ADR-0023) — the EKF inherits, not cures,
that singularity.

**Q5. How do you tune Q and R defensibly?**
`R` comes from sensor specs: camera range σ = frac·R_hat (range-dependent, ~10% sim /
15–30% real; ADR-0015), bearing σ sub-degree sim / 1–2° real; cue σ_R(R) = 4.45e-05·R²
plus a datum-bias budget (ADR-0017). `Q` comes from the worst credible target maneuver in
the path suite (weave lateral 3.0 m/s → a_max → CWNA `q`; `mc_batch.sh`). Then I *validate*
the pair with NIS/NEES chi-square consistency over a batch — average NIS ≈ measurement
dimension — rather than eyeballing a trajectory; high NIS means Q too small (the classic
divergence path).

**Q6. What causes a Kalman filter to diverge?**
Four classics: `Q` too small (filter grows overconfident and ignores real measurements);
**unmodeled bias** (our cue datum offset is a per-run *constant*, not zero-mean, so a naive
KF drifts toward it — ADR-0017); **linearization error** in an EKF near a singularity (our
`1/R` at CPA); and numerical loss of covariance positive-definiteness (mitigate with
Joseph-form or a square-root filter). Kalata diverged in Gazebo via a fifth route:
degenerate adaptive gains at the extremes of a bimodal cadence (α = 0.031/β ≈ 0 at burst,
α = 0.999 at multi-second gaps; ADR-0013).

**Q7. Observability — is target velocity observable from bearing alone?**
No — a single observer measuring **bearing only** cannot observe range or the full
velocity without an ownship maneuver (classic bearings-only target-motion analysis). That
is *why* this design measures range too (monocular size-scaling) and why the ground cue's
**emitted velocity** is the #1 lever (ADR-0015/0030) — it directly observes the component
bearing-only cannot. The same logic makes the **datum bias** observable only when camera
and cue see the target *simultaneously* (the pre-latch fusion window), which is exactly
where a bias-augmented EKF (Option D) could estimate it.

**Q8. What's an OOSM, and how do you handle a late cue?**
An out-of-sequence measurement arrives stamped in the past (our cue: 0.12 s + jitter late;
measured `ext_age_s` 0.116–0.152 s, ADR-0013). Textbook handling is retrodiction — roll the
state and covariance back to the measurement time, update, roll forward — which needs the
covariance a KF carries. Our current lite fix advances the stale sample forward along the
velocity estimate by its known age (`--cue-latency-comp`), which mostly corrects position
bias but not the velocity slope PIP inherits.

**Q9. EKF pitfalls specifically?**
The Jacobian must be correct and re-evaluated each step; linearization bias grows when `P`
is large or `h` is strongly curved (our `1/R` at CPA); and the EKF carries **no guarantee
of optimality or even consistency**, so it needs NIS/NEES monitoring. For strong
nonlinearity you'd reach for a UKF (sigma-points, no Jacobian) or a particle filter — but
those are overkill here, since our nonlinearity is a benign polar↔Cartesian map away from
the CPA singularity, which no filter fixes anyway.

**Q10. Why might a better filter NOT improve the intercept? (the answer that matters
most)**
Because at fast-crosser speeds the miss is **~96% kinematic** — `r²(ZEM,miss)=0.990`,
floored by correction capacity `½·a·t_go² ≈ 0.72 m` against a delivered ZEM of 1.69 m
(ADR-0023). The estimator touches only the ~20–25% *mechanization* slice, and most of
*that* is the freeze-latch (control logic, not estimation), leaving a sliver of
filter-settle. The right levers are **acquisition range** (scales `t_go²`) and
**mid-course track quality** (sets the delivered ZEM) — so I'd expect and *pre-register* a
null on miss, a possible win on LOS-rate error, and I'd report both. **Knowing when the
fancy tool is not the bottleneck is the point.**

---

## 6. Implementation sketch (NO code now — module boundaries and flag layout only)

Implementation is **gated until the M5 final batch is captured on a frozen config**
(ADR-0033 item 0: finish M5 first; the EKF is item 3, pure software, may interleave with
the hardware bench / markerless seeker but must not perturb the frozen M5 deployment
profile).

**Module boundary.** Introduce a small estimator interface (e.g. `scripts/estimators.py`)
that both trackers satisfy, so the guidance loop is agnostic to which is running:

```
predict(dt)
correct_camera(lambda_meas, range_m, t)     # inertial LOS azimuth + range
correct_cue(pos_ned, vel_ned_or_None, t)    # pre-latch only; camera has priority
# read-outs the guidance loop already consumes:
lambda_hat, lambda_dot_hat, r_hat, rdot_hat  (+ optional covariance / NIS handles)
```

The existing `AlphaBetaFilter` pair becomes the `alphabeta` implementation (refactor
into the interface, verified byte-identical); the EKF becomes a drop-in sibling that
fills the same read-outs (deriving `lambda_hat/lambda_dot_hat/r_hat/rdot_hat` from its
Cartesian state via the same geometry `FusedTrack.state()` already uses,
`m4_intercept.py:876–895`).

**Flag layout.** `--tracker {alphabeta, ekf}`, **default `alphabeta`** → the S2 gate and
every existing default-path flight are **byte-identical** (the project's iron rule; verify
the same way every prior port did — `--quick` numeric output and the S2 gate must match
HEAD exactly). The EKF touches only flights that pass `--tracker ekf`. Option C/D ride as
sub-flags of the EKF arm (`--ekf-model {cv,ca}`, `--ekf-estimate-datum-bias`), all
default-off.

**Build hygiene.** Develop in a git worktree with the main `.venv` symlinked; no remote
exists, so merge = local `git merge --ff-only` from the main checkout, only when finished,
clean, and the machine is idle (MEMORY; ADR-0011). Stage specific paths — never
`git add -A` while a background worker may be mid-edit.

**Order of build:** (1) refactor the interface + prove alpha-beta byte-identical; (2)
Cartesian CV EKF (Option A) with sim-spec R and path-suite-derived Q; (3) NIS/NEES
consistency harness (this is also the tuning tool from §2.4); (4) the paired A/B (§4)
against the pre-registered criteria; (5) write the addendum ADR with the verdict —
SUCCESS, PARTIAL, or NULL — treating a null with the same respect as Kalata's.

---

## 7. Sources

- `scripts/m4_intercept.py` — `AlphaBetaFilter` (`:601`), `TargetTracker` (`:707`),
  `FusedTrack` (`:805`), `kalata_alpha_beta` (`:569`), guidance loop + λ = ψ+β
  (`:1580–1719`), warm-start (`:1951–2017`), gains/constants (`:237–246`), world→NED
  (`:125–158`), OOSM/cue-latency-comp (`:1673`).
- `scripts/m3_static_intercept.py` — measurement construction (bearing = `atan2(x,z)`,
  range = ‖rel‖; `:282–284`), `Measurement` (`:189`), staleness (`:298`).
- `scripts/mc_batch.sh` / `scripts/mc_analyze.py` — paired batch runner + the paired
  significance test (`mc_analyze.py:520–530`), Wilson CIs (`:192`), per-arm Pk (`:301`).
- `.claude/skills/pronav/SKILL.md` — LOS-rate-quality mandate, strapdown λ = ψ+β,
  negative-results log, the ADR-0023 kinematic reality.
- `docs/decisions.md` — **ADR-0013** (Kalata tested & rejected: bimodal-cadence degenerate
  gains; the 4-state CV Kalman shelved lab-only), **ADR-0023** (miss is kinematic,
  r²=0.990, ½·a·t_go²=0.72 m vs ZEM 1.69 m, root-cause split), **ADR-0015 / 0015-2nd**
  (cue error structure: σ_R∝R², datum bias, latency/jitter/dropout; velocity emission =
  #1 lever), **ADR-0016** (latency budget, 0.20 s WORST tier), **ADR-0017** (corrected
  σ_R constants c=4.45e-05, datum-vs-noise split), **ADR-0018** (fusion Δ−0.088 m,
  not significant at n=8), **ADR-0029/0030/0031** (M5 regime map, running-start,
  degraded-cue envelope), **ADR-0033** (this queue item's mandate).
- Standard references (theory, not fetched): R.E. Kalman (1960) original filter; E.B.
  Wilson (1927) score interval (used in `mc_analyze.py`); T.P. Kalata (1984) "The Tracking
  Index" (steady-state α-β ↔ tracking index, ported in-repo); Bar-Shalom, Li & Kirubarajan,
  *Estimation with Applications to Tracking and Navigation* (CWNA Q block, EKF, OOSM
  retrodiction, NIS/NEES consistency).
```
