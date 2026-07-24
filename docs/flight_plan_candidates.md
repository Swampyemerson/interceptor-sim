# Flight-plan candidates — PRE-REGISTERED experiment plan to shrink the 9 m/s intercept miss

> **Status: PRE-REGISTRATION + DESIGN (2026-07-24). Nothing here has been flown.**
> Every candidate below carries its exact arm definition, its predicted result, and
> its **adopt/reject criterion written BEFORE the run** — so a result cannot be
> rationalised after the fact (the project's recurring failure mode: five mirages,
> ADR-0076). Companion docs: `docs/intercept_accuracy_levers.md` (the ranked lever
> research), `scripts/experiments/loft_dive/gazebo_results.md` (the Phase A verdict
> this follows), `docs/project_state.json` (the contract — this note changes nothing
> in it; the head owns the contract).
>
> **Labelling convention used throughout:** *MEASURED* = from a logged run;
> *DERIVED* = from a written derivation/model; *HYPOTHESIS* = a prediction with a
> pre-registered test. Nothing here is a decision until a Gazebo arm says so
> ("lab ranks, Gazebo decides").

---

## 1. Where we actually are (the honest problem statement)

### 1.1 What Phase A settled — and what it cost

*MEASURED* (`scripts/experiments/loft_dive/gazebo_results.md`, n=8 paired, seed 123,
canonical line-9 @9 m/s):

| | ARM A (flat dash, level cam) | ARM B (cap 3.57 + loft 2 m + 10° wedge) |
|---|---|---|
| 8–12 m REAL-detection recall | 3.0 % (2/67) | **35.4 % (28/79)** |
| in-camera vertical angle of target | −25.7° (frame top) | **−3.6° (centred)** |
| <5 m REAL recall | 25.4 % | **82.4 %** |
| miss median / Pk@2.5 m | **0.75 m / 6 of 8** | 2.93 m / 1 of 8 |
| closing speed at handoff | **4.7 m/s** | 3.0 m/s |

The pointing mechanism is **confirmed**. The miss got **worse**. The published
explanation was "the accel-cap's closing-speed cost dominates at 9 m/s". This note
argues — from the flown dash-only control arms and a validated ballistic model —
that **the closing-speed story is only part of it, and the larger part is a fixable
aim bug.**

### 1.2 The dash-only controls are IN, and they change the reading

The anti-mirage control arms (`Adash`, `Bdash`: camera terminal gated shut with
`--coded-dash-acquire-range-min 999`, so handoff never fires → **0 ENGAGE ticks** →
the logged miss *is* the open-loop dash ballistic CPA) have both been flown
(`logs/mc_loftdive_armAdash_line9_s123.csv` 2026-07-21,
`logs/mc_loftdive_armBdash_line9_s123.csv` 2026-07-24). Verified in this analysis:
every flight has 0 ENGAGE ticks and `miss_m` == min `gt_range`.

*MEASURED*, paired by seed (same 8 geometries in every arm):

| arm | camera | miss median | Pk@2.5 | ≤0.5 m (ram bar) | l2r median | r2l median |
|---|---|---|---|---|---|---|
| A | live | **0.75 m** | 6/8 | 1/8 | 0.69 | 2.09 |
| Adash | **off** | 1.37 m | 7/8 | 1/8 | 2.05 | **0.77** |
| B | live | 2.93 m | 1/8 | 0/8 | 2.53 | 3.92 |
| Bdash | **off** | 2.01 m | 7/8 | 0/8 | 1.76 | 2.38 |

Three readings fall straight out, and they reframe the whole problem:

1. **The camera terminal is NET-NEGATIVE in the framed arm.** B is worse than Bdash
   on **8 of 8** paired flights (two-sided sign test p = 0.008). Phase A gave the
   terminal ~12× more real detections and the terminal **spent them making the miss
   bigger**. Framing is necessary; it is not sufficient, and today's terminal
   actively converts it into harm.
2. **The camera terminal is direction-split in the baseline arm.** A beats Adash on
   **4 of 4 l2r** flights (median 0.69 vs 2.05 m — the camera is worth ~1.4 m there)
   and loses on 3 of 4 r2l. Net 5/8 (p = 0.73, **not significant**). So "the camera
   does nothing" (#18h) is too strong and "camera-guided intercept" is too strong —
   the honest statement is *the camera helps l2r, hurts r2l.*
3. **The capped dash's BALLISTICS are worse, mostly in r2l** (Bdash r2l 2.38 m vs
   Adash r2l 0.77 m). A pure closing-speed story would not be that asymmetric.

### 1.3 The mechanism nobody had priced: the lead solve assumes a speed the quad never has

*DERIVED* (`scripts/experiments/flight_plans/dash_cpa_model.py`; new, this note).

`flight.guidance.collision_lead_heading` solves a **constant-speed** intercept
triangle at `--dash-speed` (16 m/s). *(Primer: a "lead" or collision-course heading
aims at where the target **will be**, not where it is — like passing a football.)*
But the quad starts **at rest** and PX4 caps horizontal acceleration at
`MPC_ACC_HOR_MAX` = 12 m/s² under `--fpv`, so it needs ~1.3 s just to reach 16 m/s
— and the whole engagement is ~1.5–2.0 s. The dash therefore flies a lead solution
for a vehicle that does not exist, and the error is absorbed by the empirically
tuned `--dash-crossing-bias-deg 30`.

That makes the crossing bias an **acceleration-dependent constant**. Model it
honestly: interceptor from rest, constant accel `a`, clamped at `v_max`,
`s(t) = ½at²`; target constant velocity; CPA = min over t of the separation. Fit `a`
to the two dash-only arms (the only clean ballistic data in the repo):

*MEASURED vs DERIVED* (`dash_cpa_model.py --validate`):

| arm | flown config | best-fit effective accel | model MAE |
|---|---|---|---|
| Adash | uncapped (`MPC_ACC_HOR_MAX` 12) | **10 m/s²** | 0.58 m |
| Bdash | `--dash-accel-cap 3.57` | **4.0 m/s²** (cmd 3.57) | **0.29 m** |

The model reproduces the capped arm to 0.29 m mean absolute error and the uncapped
arm to 0.58 m (with a **direction-signed residual**: it under-predicts l2r by
~0.5–1.0 m and over-predicts r2l by ~0.2–0.7 m — an asymmetry the point-mass model
does not contain; see §6 honesty). Good enough to **rank**, not to conclude.

Then the headline table (`--sweep`) — the ballistically-optimal crossing bias as a
function of dash accel, over the 8 canonical geometries:

| dash accel a | θ = atan(a/g) | model-optimal bias | model CPA at optimum | model CPA at the flown 30° |
|---|---|---|---|---|
| 3.57 (ARM B cap) | 20° | **64°** | 0.28 m | 2.33 m |
| 4.5 | 24.6° | 48° | 0.24 m | 1.58 m |
| 5.66 | 30° | 37° | 0.21 m | 0.79 m |
| 7.0 | 35.5° | 29° | 0.19 m | 0.20 m |
| 10.0 (ARM A fitted) | 45.6° | **20°** | 0.16 m | 1.26 m |
| 12.0 | 50.7° | 16° | 0.14 m | 1.92 m |

**Two consequences, and they are the spine of this plan:**

- **ARM B was flown with the wrong aim.** At its accel the ballistic optimum is
  ~64°, it flew 30°. The model attributes ~1.5–2 m of ARM B's regression to that
  aim mismatch, *not* to closing speed. **The Phase-A cap has never been tested
  with a correctly-sized aim.** *(HYPOTHESIS, testable — arm E.)*
- **The BASELINE may be mis-aimed too.** At the fitted a≈10 m/s² the model optimum
  is ~20°, not the flown 30° — worth ~1.1 m of ballistic CPA. If that holds, it is
  the single largest miss reduction available anywhere in this document, it costs
  **zero code**, and it transfers directly to the real aircraft (which also starts
  from rest). *(HYPOTHESIS, testable — arm G20.)*

> **Why this matters beyond the sim:** the real interceptor's coded dash uses the
> same `collision_lead_heading`. If this holds, the hardware would fly the same
> systematic aim error, and the fix is a ~15-line honest change (solve the lead with
> the vehicle's own acceleration profile instead of a constant speed) — see the
> ADR-lite in §8.

### 1.4 Why the camera makes it worse: a measured terminal LOS bias

*MEASURED* (this analysis, from the per-tick flight CSVs): for every ENGAGE tick
with a **real** (gt-consistent) detection, compare the estimated inertial LOS
`lambda_deg` against the gt LOS azimuth (gt used for SCORING only).

| arm | l2r median LOS error | r2l median LOS error |
|---|---|---|
| ARM B (framed) | **+9° … +17°** (4/4 flights) | **−17° … −21°** (3/3 flights with real ticks) |
| ARM A (unframed) | wildly scattered, −89°…+51° (1–7 real ticks/flight) | −44°…+23° |

ARM B's LOS is now *consistent* — and consistently **biased by ~15–20° with a sign
that flips with the crossing direction**. That is the ADR-0056 aspect bias, finally
visible because there are enough real detections to see it. And it matters
structurally: the terminal builds its velocity command in the LOS frame
(`u = (cos λ̂, sin λ̂)`, `p = ⊥`, m4_intercept.py:3439-3450), so a 15–20° LOS bias
**rotates the entire commanded velocity vector** by 15–20°. At 3–5 m/s closing over
~1 s that is ~0.8–1.7 m of induced miss — the same size as the measured B−Bdash
penalty. *(DERIVED; mechanism consistent with the measurement.)*

### 1.5 The three tracks that follow

| track | what it attacks | code needed | expected size |
|---|---|---|---|
| **1 — AIM** | the accel-vs-constant-speed lead mismatch (§1.3) | none (flag only) | ~1 m of miss *(HYPOTHESIS)* |
| **2 — POINTING at no cost** | keep Phase A's framing without the cap's speed/aim toll (§1.1) | none (flag only) | recall 3 % → ≥15 % at ARM-A ballistics *(HYPOTHESIS)* |
| **3 — CONVERSION** | make the terminal *use* the frames instead of harming them (§1.4) | small (flag + audit) | up to ~1 m *(HYPOTHESIS)* |

The ZEM caveat still governs (ADR-0027): at 9 m/s the delivered zero-effort-miss is
~4 m against ~0.27 m of terminal correction capacity, so **no terminal trick alone
gets to the 0.3–0.5 m ram bar**. Tracks 1 and 2 both work by *reducing the ZEM the
terminal is handed* — which is exactly what that caveat says the win must be.

---

## 2. Ranked candidate flight plans

Every arm below is a concrete `--dash-*` combination, launchable from the
pre-registered harness `scripts/experiments/flight_plans/run_arm.sh` (PROPOSED,
unflown; extends `scripts/experiments/loft_dive/run_arm.sh` — same env, same rails,
same canonical geometry). Common base, identical to the flown arms except that the
crossing bias moves out of the base (because it must be co-sized with the accel):

```sh
BASE="--coded-dash --fpv --dash-unclamp --dash-speed 16"
LOFT="--dash-loft-m 2 --dash-loft-dive-s 2.5 --dash-vvert-max 3.0"
LOFT3="--dash-loft-m 3 --dash-loft-dive-s 3.5 --dash-vvert-max 3.0"
DASHONLY="--coded-dash-acquire-range-min 999"     # anti-mirage control
```

All arms: `--mode m4 --laws pronav --path line --geometry standard --x0 6.5
--y0-mag 15.343 --speeds 9.0 --directions both --n 8 --master-seed 123` — i.e.
**n=8 paired, both directions, byte-identical geometry to the flown A/B/Adash/Bdash
arms** (verified: the dry-run plan reproduces the same 8 `start_y` values).
Promising arms replicate on the **disjoint seed 777**.

---

### G20 / G20dash — accel-consistent AIM on the baseline dash *(rank 1)*

```sh
# G20dash  (fly FIRST: pure ballistics, no camera in the loop)
EXTRA="$BASE --dash-crossing-bias-deg 20 $DASHONLY"
# G20      (fly SECOND, only if G20dash confirms)
EXTRA="$BASE --dash-crossing-bias-deg 20"
```
Physical wedge: **none** (level camera, as ARM A). Parse with `=0`.

**Physics.** Nothing about the vehicle changes — only the pre-flight aim constant.
The dash heading is `collision_lead_heading(target_start, target_vel, 16)` minus a
sign-keyed bias; the bias is currently 30° (tuned empirically on a different
geometry, ADR-0076 #18e). The model says the ballistic optimum at this dash's
*measured* effective accel (10 m/s²) is 20°. 20° is also the **robust** choice
across accel uncertainty: model CPA ≤0.71 m for a ∈ [8, 12] m/s² (≤0.97 m out to
a=14), versus 0.45–2.46 m at the current 30° over the same range.

**Side benefit (DERIVED):** a smaller bias points the dash closer to the target's
*current* bearing, so at dash start the target sits **29° off boresight instead of
43°** (sim half-HFOV ±49.8°) — in frame from t=0 either way, but with three times
the margin. Aim and framing move the *same* way here — no tradeoff.

**Pre-registered prediction (`dash_cpa_model.py --predict`).** Per-flight ballistic
CPA at a=10, bias 20: `[0.17, 0.02, 0.12, 0.11, 0.25, 0.35, 0.08, 0.16]`, median
**0.14 m** (vs the same model's 1.26 m at bias 30, and Adash's *measured* 1.37 m).
Given the model's 0.58 m MAE and its direction-signed residual, the honest
prediction is **G20dash median ≤ 0.90 m** and **improvement on ≥6/8 paired flights
vs Adash**.

**PRE-REGISTERED CRITERION (decided before the run):**
- **CONFIRM the aim model** iff G20dash beats Adash on **≥6/8** paired flights AND
  median miss ≤ **0.90 m**. → fly G20 (camera live) next, then replicate both on
  seed 777.
- **REFUTE** iff G20dash is worse on ≥5/8 OR median ≥ 1.37 m (no better than
  Adash). → the ballistic model does not describe this vehicle; drop Track 1,
  record the null in `docs/decisions.md`, and do **not** touch the bias again
  without new evidence.
- **INCONCLUSIVE** (5/8 either way, median between): say exactly that — "not
  significant at n=8" — and decide on the seed-777 replication before claiming.
- **ADOPT into the contract** only after pooled seeds 123+777 give **≥13/16**
  paired improvements (two-sided sign test p = 0.021).

**Metrics to parse:** miss median + min–max **per direction**, Pk@2.5, fraction
≤0.5 m (the ram bar), clean %, and — for G20 only — 8–12 m REAL recall, handoff
speed, ENGAGE-tick LOS error.

---

### C / Cdash — LOFT-ONLY pointing (drop the accel cap) *(rank 2)*

```sh
# C
EXTRA="$BASE --dash-crossing-bias-deg 30 $LOFT --cam-mount-up-deg 10"
# Cdash
EXTRA="$BASE --dash-crossing-bias-deg 30 $LOFT --cam-mount-up-deg 10 $DASHONLY"
```
Physical wedge: **up10 shadow** (`scripts/experiments/uptilt_mounts/up10`, owned by
the harness). Parse with `=10`. **Single-axis change from the flown ARM B: the
accel cap is removed.** Nothing else moves.

**Physics — why the cap is the least valuable of Phase A's three knobs.** Decompose
the measured centring gain (ARM A −25.7° → ARM B −3.6°, i.e. +22.1° of correction)
against the measured pitch and the loft geometry:

| contributor | value | *derivation* |
|---|---|---|
| fixed wedge | **+10.0°** | the fitted `up10` mount |
| loft (height advantage) | **≈ +7.1°** | LOS depression `asin(h/R)`; h ≈ 1.2–1.4 m residual at the band, R ≈ 10 m |
| accel cap | **≈ +5.0°** | measured body pitch through the band: −31° (A) → −26° (B) |

The cap contributes the **smallest** share of the centring and carries **all** of
the cost (handoff speed 4.7 → 3.0 m/s, plus the §1.3 aim mismatch). Drop it and the
wedge+loft still deliver ~17° of the 22°.

**Pre-registered prediction (HYPOTHESIS).** Band vertical angle
`vert_cam ≈ δ_loft + wedge − |pitch|`. Uncapped, the band is reached earlier
(t ≈ 0.6–1.0 s → loft residual 1.3–1.7 m → δ ≈ 7.5–9.9° at R=10 m), pitch ≈ 31°,
wedge 10° → **vert_cam ≈ −11° ± 5°** (ARM A −25.7°, ARM B −3.6°). Recall should
land **between** the two arms; ballistics should match ARM A (identical dash
dynamics).

**PRE-REGISTERED CRITERION:**
- **ADOPT loft-only as the Phase-A operating point** iff **all three** hold:
  (i) 8–12 m REAL recall **≥ 15 %** (vs 3.0 % baseline);
  (ii) `<5 m` REAL recall **≥ 50 %** (vs 25 % baseline — the out-the-bottom guard);
  (iii) handoff speed **≥ 4.0 m/s** (vs A 4.7, B 3.0) AND paired miss **not worse
  than ARM A on ≥6/8** flights.
  Then replicate on seed 777 before it enters the contract.
- **REJECT** iff recall < 8 % (framing didn't survive the cap removal) OR `<5 m`
  recall < 30 % (the loft dumped the terminal out the frame bottom).
- **PARTIAL / escalate to C3 or C15** iff recall ≥ 15 % but `vert_cam` median is
  worse than **−15°** (under-centred) → try `C3` (loft 3 m / dive 3.5 s) first
  (more height, no wedge risk), then `C15` (wedge 15°) — in that order, because the
  wedge is the knob the open contradiction constrains (§7).
- **Cdash is mandatory before any camera claim** (§4).

---

### E / Edash — the accel cap, RE-AIMED *(rank 3)*

```sh
# E
EXTRA="$BASE --dash-crossing-bias-deg 64 --dash-accel-cap 3.57 $LOFT --cam-mount-up-deg 10"
# Edash
EXTRA="$BASE --dash-crossing-bias-deg 64 --dash-accel-cap 3.57 $LOFT --cam-mount-up-deg 10 $DASHONLY"
```
Physical wedge: **up10**. Parse with `=10`. **Single-axis change from ARM B: the
crossing bias 30 → 64** (the model optimum at a=3.57).

**Why bother, given C?** Because it *isolates the §1.3 mechanism* and settles
whether Phase A's headline lever was killed by physics or by a mis-sized constant.
It also matters for the real build: the capped dash is **far more forgiving of aim
error** (§3 of `docs/launch_mechanism_plan.md`: ±13° for a 1 m CPA versus ±6.5° at
the uncapped dash) — if a re-aimed cap gets close to ARM A's miss, it is the safer
hardware configuration for a hand-aimed launch.

**Pre-registered prediction.** Model per-flight CPA at a=3.57, bias 64:
`[0.16, 0.12, 0.07, 0.05, 0.61, 0.80, 0.01, 0.45]`, median **0.14 m** (model at the
flown bias 30: 2.25 m; Bdash *measured* 2.01 m). With the model's 0.29 m MAE on
this arm: **Edash median ≤ 0.8 m**, improvement on **≥7/8** vs Bdash.

**PRE-REGISTERED CRITERION:**
- **CONFIRM "ARM B's regression was an aim artefact"** iff Edash beats Bdash on
  ≥7/8 paired flights AND median ≤ 0.8 m. Then the accel-cap returns to the live
  option set (contract note required) and E (camera live) decides whether the cap
  or loft-only is the better operating point, on paired miss.
- **REFUTE (cap cost is real)** iff Edash median > 1.5 m or improvement ≤ 5/8. Then
  the closing-speed explanation in `gazebo_results.md` stands as written and the
  accel cap stays out of the operating point.
- Either way, log the achieved 8–12 m recall: a *bigger* bias points the dash
  further ahead of the target, which by geometry delays azimuth framing (target
  enters the ±49.8° FOV at t≈1.29 s instead of t=0, at R≈7.5 m instead of 16.7 m).
  **Watch for a recall collapse** — that is the pre-registered cost of this arm,
  and if recall drops below 15 % the arm is an aim result only, not a pointing one.

---

### D — light cap θ≈30° with a co-sized aim *(rank 4 — the `gazebo_results.md` "Next" item, corrected)*

```sh
EXTRA="$BASE --dash-crossing-bias-deg 37 --dash-accel-cap 5.66 $LOFT --cam-mount-up-deg 10"
```
Physical wedge: **up10**. Parse with `=10`.

The Phase-A "Next" section proposed `--dash-accel-cap 5.66` (θ30°) with everything
else held. **Do not fly it at bias 30**: at a=5.66 the model optimum is 37°, and
flying 30 would repeat the ARM-B confound at smaller scale (model CPA 0.79 m vs
0.21 m). Rank 4 rather than 2 because the *measured* pitch benefit of a cap is only
~5° at a=3.57 and would be smaller here (~2–3°) — i.e. this arm probably buys the
least pointing of the three pointing arms while still paying some speed.

**PRE-REGISTERED CRITERION:** adopt only if it beats **both** C and E on paired
miss **and** holds recall ≥ 15 % — otherwise it is dominated and gets recorded as
such (no partial credit).

---

### CG / CGdash — the combination *(rank 5, gated)*

```sh
EXTRA="$BASE --dash-crossing-bias-deg 20 $LOFT --cam-mount-up-deg 10"
```
Loft-only pointing **×** accel-consistent aim. **Gated:** fly only after *both*
G20dash and C have passed their criteria — combining two unvalidated changes is how
you get an uninterpretable win. Pre-registered expectation: ARM-A-class ballistics
at bias 20 (model 0.14 m) with C's recall. This is the candidate most likely to be
the new operating point *(HYPOTHESIS)*.

---

### F — Phase B FOV-hold *(rank 6, CODE-GATED)*

`flight/fov_guidance.py` exists, is unit-tested, and is **not wired into
`scripts/m4_intercept.py`** (grep: no `--fov-hold` flag). Wiring is ~30 lines,
specified in `docs/phase_bcd_wiring.md`, and it adds a **new pixel→command path**,
so it **re-earns the numeric no-cheat audit** before any number is quoted. Two
warnings from that note that must be honoured: use a dedicated `--fov-vert-max`
(~2.5 m/s), **not** the 0.5 m/s altitude-hold clamp (which would gut the lever and
produce a false negative); and hoist the config construction out of the tick loop.

**Sequencing judgement:** Phase B is *conversion*, and §1.4 says the conversion
problem today is a **15–20° LOS bias**, not FOV loss — ARM B already held the target
near boresight (−3.6°) and still lost. So **H (below) should be tested before F.**

---

### H — terminal LOS bias correction *(rank 7, CODE-GATED — the real Track-3 lever)*

**Not yet buildable:** no flag applies a bias to the measured camera bearing (grep
of `m4_intercept.py`: only the *dash* `--dash-crossing-bias-deg` exists). Proposed
`--terminal-bearing-bias-deg B`: subtract a sign-keyed constant from `bearing_rad`
before `derotate_bearing_lambda`, the sign keyed on the pre-flight crossing
direction exactly as the dash bias is (`dash × Vt`) — a **pre-flight constant, no
live sensor read**, same honesty class as the dash bias, and it must re-earn the
no-cheat audit. Pre-registered magnitude from §1.4: **B = 15°**. Pre-registered
criterion: with the framed arm (C or CG), the camera-live arm must beat its own
dash-only control on **≥6/8** paired flights — i.e. the camera must finally be
*worth having*. That, and only that, would justify the first "camera-driven
intercept" claim in this project (§4).

---

## 3. Metrics to parse (identical for every arm)

`.venv/bin/python scripts/experiments/loft_dive/parse_ab.py <arm.csv>=<wedge_deg>`
already emits all of these; report them in this order:

1. **miss median + min–max, split l2r / r2l** (the l2r/r2l asymmetry is real and
   every historical conclusion that ignored it has been wrong).
2. **Pk@2.5 m** (the sanctioned sim proxy, ADR-0025) **and** the fraction ≤0.5 m
   (the actual RAM bar — a 2.3 m "Pk kill" is a fly-through, `kill_mechanism.md`).
3. **clean %** (ARM B lost 2/8 flights to `python_exit_1`, both r2l — a config that
   aborts flights is not a better config).
4. **8–12 m REAL-detection recall** (pooled *and* per-flight median — ticks within a
   flight are correlated, so the pooled number alone overstates significance).
5. **`<5 m` REAL recall** (the out-the-bottom guard).
6. **in-camera vertical angle (`vert_cam`) median through the band**, mount-adjusted.
7. **closing speed at handoff** + dash peak speed.
8. **ENGAGE-tick LOS error vs gt** (scoring only) — the §1.4 diagnostic; add it to
   the parser or run the snippet in §9.

**Statistics (pre-registered, per CLAUDE.md "statistics before verdicts"):** paired
two-sided sign test. At n=8: 8/8 → p=0.008, 7/8 → p=0.070, 6/8 → p=0.289. So **6/8
is NOT significant** — it justifies a seed-777 replication, never a verdict. Pooled
n=16 (seeds 123+777): 14/16 → p=0.004, 13/16 → p=0.021, 12/16 → p=0.077. **Contract
adoption requires ≥13/16.** Report the sign-test p with every claim, and use the
words "not significant at this n" when it applies.

---

## 4. Anti-mirage plan (mandatory, non-negotiable)

The project has caught five mirages; two of them (ADR-0076 add **#18g** — "camera-
guided r2l" — and **#18h** — the dash-only control that scored the same CPA with
zero real detections) are precisely this experiment's failure mode. Rules:

1. **Every camera-live arm ships with its paired dash-only twin** (`Xdash`,
   `--coded-dash-acquire-range-min 999`). Same seed, same geometry, same flags.
   Budget it: a camera arm without its control is not a result, it is a rumour.
2. **The camera claim is the DIFFERENCE, never the level.** "Camera-driven" requires
   the camera arm to beat its own dash-only control on ≥6/8 paired flights, with the
   sign-test p reported. Today: ARM A 5/8 (p=0.73, *not* camera-driven), ARM B 0/8
   (the camera is actively harmful).
3. **Run `scripts/audit_per_tick.py` on every flight** (the phase-name bug is fixed;
   it accepts `CODED_DASH`). Checks (a)/(b) must PASS on 100 % of flights — that is
   the honesty core (no `ext_*` leak post-handoff, dash-before-ENGAGE). Check (c)
   (command-vs-camera-LOS correlation ≥0.7) is **advisory** on these short
   engagements; quote it, never lean on it.
4. **`gt_*` is scoring only.** The recall/CPA/LOS-error numbers above all use gt to
   *score*. Any new guidance path (F, H) re-earns the numeric no-cheat audit before
   a number is quoted.
5. **Report the null loudly.** If a candidate is inconclusive, the deliverable is
   the pre-registered "not significant at n=8" sentence plus the raw paired table.

---

## 5. Sequencing — what to fly first (one sim at a time, idle load)

Each 8-flight arm took ~9 minutes of wall clock in the flown Phase-A batch
(`logs/mc_loftdive_arm*_stdout.log` timestamps, GPU render on, RTF ~0.95) — cheap.
The scarce resource is **interpretation**, not sim time, so fly in an order where
each result changes what you do next.

| # | arm | why first | cost |
|---|---|---|---|
| **1** | **G20dash** | Decisive test of the §1.3 aim model with **zero camera noise** — the cleanest possible read, against an already-flown control (Adash). If it confirms, it is the biggest single miss reduction in this document and it applies to *every* later arm. | 8 flights |
| **2** | **C** (+ **Cdash** immediately after) | Answers the Phase-A "Next" question directly: does the framing survive without the cap's toll? Two arms so the camera claim is attributable. | 16 flights |
| **3** | **G20** | Converts the confirmed aim fix into the flying config (camera live) — gated on #1 passing. | 8 flights |

Then, decided by those results: **E/Edash** (rehabilitate the cap) → **CG/CGdash**
(combine) → seed-777 replication of whatever survived → **H** (code) → **F** (code).
Do not fly D unless C and E both under-perform.

**Hygiene, every arm:** one sim at a time; idle load (`LOAD_MAX` gate is in the
harness); sim-clock only (the ramp/dive already use `sim_clock.t`); kill patterns
live in script files (`scripts/sim_kill.sh`), never inline; the harness owns the
`models/mono_cam` wedge symlink lifecycle and refuses to fly a level-camera arm if
one is present (checked: no stale shadow at the time of writing).

---

## 6. Honest limits of this plan

- **The ballistic model is a ranking tool.** MAE 0.29 m (capped) / 0.58 m
  (uncapped), and its residual on the uncapped arm is **direction-signed**
  (l2r under-predicted, r2l over-predicted by ~0.5–1 m). It contains no yaw
  dynamics, no attitude transient, no camera, no wind. Its *ordinal* claim (optimal
  bias falls as accel rises) is pure geometry and robust; its *point* predictions
  are not. This is exactly the "lab ranks, Gazebo decides" boundary.
- **The empirical bias of 30° was measured, not guessed** (ADR-0076 #18e, 2 seeds).
  The model disagrees with it at the baseline accel. One of them is wrong — that is
  what G20dash is for. Note the model and the old sweep disagree *most* on r2l,
  where the model over-predicts; a per-direction optimum may exist that the
  symmetric `--dash-crossing-bias-deg` flag cannot express (it applies one magnitude
  with opposite signs). If the sweep shows a per-direction split, that is a finding,
  and the fix is a per-direction constant — not more sim.
- **n=8 per arm is small.** 6/8 is not significant. Every promising result gets seed
  777 before it touches the contract.
- **Sim has no motion-blur model**, so all recall numbers are **upper bounds** that
  the Stage-0 bench must confirm (terminal LOS 485–1870°/s → ~23 px smear at 5 ms).
- **This is the markerless (`drone_finetuned_quad_v2`) regime.** First real kills fly
  the AprilTag; the aim results (Tracks 1) transfer, the recall results do not
  automatically.

---

## 7. The open contradiction: `wedge-sizing-vs-accel-cap`

The contract carries an **OPEN** contradiction: the committed pointing decision says
"wedge = the measured dash pitch (25–40°)", while the Phase-A in-frame A/B says
"keep it modest (~10–15°), because a wedge ≥20° centres the mid-band but **dumps the
<5 m terminal target out the frame bottom** (the quad brakes nose-UP there)".

**This plan honours the modest band and does not attempt to resolve the
contradiction by fiat.** Every arm here uses **10°** (the flown, measured-good
value: `<5 m` REAL recall 82 % in ARM B), and the only escalation offered is **15°**
(`C15`), the top of the sanctioned band, and only if `C` measures under-centred.
Reason: the arms that *raise* pointing demand here are the ones that also *remove*
the accel cap — and removing the cap means a harder terminal brake, i.e. **more**
nose-up at short range, i.e. **more** out-the-bottom risk. A bigger wedge is
therefore the *most* dangerous knob in exactly the configuration we are moving
toward. The `<5 m` REAL-recall ≥50 % guard in every criterion is the tripwire.
Resolution of the contradiction stays with the head + the real ULog dash pitch.

---

## 8. ADR-lite decisions taken in writing this plan

1. **The crossing bias must be co-sized with the dash acceleration.** *Context:* the
   lead solve is constant-speed, the vehicle accelerates from rest. *Options:*
   (a) leave the bias empirical and re-tune per config; (b) make the lead solve
   accel-aware in `flight/guidance.py`; (c) treat the bias as accel-dependent and
   size it from the model. *Decision:* **(c) now, (b) if G20dash/E confirm.** *Why:*
   (c) is zero-code and testable immediately; (b) is the right engineering fix but
   changes the portable core, so it should be paid for with evidence. *Proposed (b):*
   `accel_lead_heading(target_pos, target_vel, accel, v_max, origin)` solving
   `s(t)=½at²` clamped at `v_max` — same inputs, same honesty class, ~15 lines,
   directly applicable to the real aircraft (which also starts from rest).
2. **No arm changes two things at once.** *Why:* ARM B changed pitch **and**
   (implicitly) the aim validity, and the result is un-attributable a year later.
   Every arm above is a single-axis move from a flown reference, with `CG` explicitly
   gated behind both of its parents.
3. **The terminal LOS bias is tested before Phase B.** *Why:* the measured failure in
   the framed arm is a 15–20° signed LOS bias, not FOV loss; Phase B holds a target
   in frame that is *already* in frame.
4. **Pk@2.5 m is reported but never celebrated.** The ram bar is 0.3–0.5 m; every
   table carries the ≤0.5 m column.

---

## 9. Considered and EXCLUDED (graveyard check — do not re-litigate)

Checked against `docs/project_state.json` → `graveyard` before writing:

| idea | why excluded here |
|---|---|
| Porpoise / pitch-up re-framing pulses | **Graveyarded.** Dominated by loft-then-dive and manufactures the pitch swing we are escaping. The loft arms are the sanctioned continuous-framing form. |
| Impact-angle-constrained ram | **Graveyarded.** Eats the lateral authority that sets 96 % of the miss. |
| Early observability weave | **Graveyarded.** Spends FOV margin — the scarcest resource — for a range the straight-line PN barely needs. |
| Phantom range-plausibility window | **Graveyarded** as an *acquisition* gate. Used here only in its sanctioned #18h role: the dash-only control switch. |
| Subpixel / centroid bearing | **Graveyarded** (NULL at n=31). Would otherwise look attractive against the §1.4 LOS bias — it is not the fix; a signed pre-flight constant is. |
| Fixed up-tilt adoption (cue-era up15) | **Graveyarded** in its cue-era terminal-parity context. The wedge here is the *new* acquisition-first decision (10°, co-sized), not a resurrection — and it stays inside the contradiction's modest band. |
| APN / Kalata / PIP, accel-predictive guidance | **Graveyarded** (SNR<1 noise wall). Nothing here differentiates a bearing. |
| v3 / rebal retrains, foveated crop, higher-res, narrow lens, colour, IR | **Graveyarded.** All sensor-side; the wall is geometry. |
| Bank-as-accel APN | **Graveyarded** (measured dead: σ_a 6–12 m/s² vs a 4.7 m/s² signal). |
| Live datalink cue / auto-fire | **Graveyarded + constraint `no-datalink`.** Every constant used here is pre-flight. |
| Sim pivot | **Graveyarded.** |
| Raising `--dash-speed` / `--accel-boost` for more closing speed | **Not graveyarded, but excluded here:** more accel means *more* nose-down pitch (θ=atan(a/g)), which is the pointing wall — and the model says the aim would need re-sizing again. Revisit only after Track 1 lands. |
| Lowering `MPC_ACC_HOR_MAX` to make the cap physical (recipe caveat) | **Excluded from the first wave:** it makes the pitch cap exact but costs even more closing speed and aim validity. Useful only as a mechanism-isolation arm if E is ambiguous. |
| "Dive through the intercept" (keep descending past CPA) | **Deferred:** genuinely interesting (adds vertical closing speed, keeps the target below boresight in the endgame) but ENGAGE re-acquires `ALT_REF_M`=0.5 m, so it needs a code change. Log it as an idea; do not fly it in this wave. |

*Diagnostic snippet used for §1.4 (offline, gt for scoring only):* for each ENGAGE
tick with `detected` and gt-consistent range, compare `lambda_deg` against
`atan2(gt_tag_x−gt_cam_x, gt_tag_y−gt_cam_y)` in degrees, and report the per-flight
median. Fold this into `parse_ab.py` when the next arm lands.

---

## 10. Provenance

- Flown data: `logs/mc_loftdive_arm{A,B,Adash,Bdash}_line9_s123.csv` + per-flight
  `logs/m4_intercept_pronav_*.csv` (verified: `miss_m` == min `gt_range`; dash-only
  arms have 0 ENGAGE ticks).
- Ballistic model + every table in §1.3/§2:
  `scripts/experiments/flight_plans/dash_cpa_model.py` (`--validate --sweep
  --sensitivity --predict`), which reads the flown arm CSVs for its geometries and
  its validation.
- Arm harness (proposed, unflown): `scripts/experiments/flight_plans/run_arm.sh`,
  extending `scripts/experiments/loft_dive/run_arm.sh`.
- Prior art this builds on: `scripts/experiments/loft_dive/gazebo_results.md`,
  `scripts/experiments/loft_dive/gazebo_run_recipe.md`,
  `docs/intercept_accuracy_levers.md`, `docs/phase_bcd_wiring.md`, ADR-0027 (ZEM),
  ADR-0056 (aspect-biased bearing), ADR-0076 add #18e/#18g/#18h/#18k,
  `docs/project_state.json` (contradiction `wedge-sizing-vs-accel-cap`, graveyard).

---

## RESULTS (live — head, 2026-07-24)

### G20dash — CONFIRMED. Correct aim alone ~halves the open-loop dash miss.

First decomposition arm flown (`logs/mc_fp_armG20dash_line9_s123.csv`, n=8 paired,
seed 123, level camera, **dash-only** control via `--coded-dash-acquire-range-min 999`).
Tests the aim-mis-sizing hypothesis: bias **20°** (near the ballistic model's optimum for
the baseline accel a≈10) vs the previously-flown **Adash** at bias 30°.

| dir | G20dash (bias 20) median | Adash (bias 30) median | G20 better / paired |
|-----|-------------------------:|-----------------------:|:-------------------:|
| l2r | **0.73 m** (0.41–0.85)   | 2.05 m                 | **4/4** |
| r2l | 0.81 m (0.60–1.19)       | 0.77 m                 | 3/4 |
| **combined** | **0.75 m**      | 1.37 m                 | **7/8** |

**Pre-registered criterion:** CONFIRM iff ≥6/8 paired better AND median ≤0.90 m.
**Result: 7/8 better AND median 0.75 m → CONFIRMED.**

**Reading.** The +30° crossing-bias was mis-sized (the collision-lead solves a
constant-speed triangle; the dash accelerates from rest). Correcting it to +20° for the
baseline accel drops the open-loop **dash-only ballistic** combined median **1.37 → 0.75 m**
— almost entirely from the **l2r** side (2.05 → 0.73 m), which the wrong aim had been
throwing wide; r2l was already near-optimal at 30° and is unchanged. The aim constant is a
**free, honest, pre-flight lever** (a launch-time constant, no gt, no live cue) — the single
cheapest miss-reduction found for the coded dash. **Caveat:** 0.75 m is the DASH-ONLY
ballistic floor (camera disabled); the point-mass model predicted ~0.14 m, so it correctly
ranked the aim but over-predicted the achievable absolute (real quad dynamics + the 0.5 m
ALT_REF terminal). Whether the camera terminal improves on 0.75 m at correct aim is the next
arm (**G20**, camera-live bias 20). Contract stage `launch_aim` updated.

### G20 — camera-live at correct aim: the terminal does NOT beat the well-aimed dash.

`logs/mc_fp_armG20_line9_s123.csv` (n=8 paired, camera LIVE, bias 20, level) vs the
G20dash dash-only control (same aim). The camera engaged genuinely on 7/8 (clean=1).

| dir | G20 camera median | G20dash dash-only median | camera tighter / paired |
|-----|------------------:|-------------------------:|:-----------------------:|
| l2r | 0.93 m (0.71–1.13) | 0.73 m                  | 0/4 |
| r2l | 1.08 m (0.64–2.43) | 0.81 m                  | 2/4 |
| **combined** | **1.02 m** | **0.75 m**             | **2/8** |
Pk@2.5 = 8/8, Pk@1.0 = 4/8.

**Reading — the key decomposition result.** With the aim CORRECT, letting the camera
terminal take over makes the median miss WORSE (0.75 → 1.02 m), tighter on only 2/8. The
measured terminal aspect bias (ADR-0056: λ−gtLOS +9..+17° l2r / −17..−21° r2l) steers a
trajectory that correct pre-flight aim had already put near-optimal. So in THIS regime —
line-crossing at 9 m/s, aim known — the well-aimed **open-loop coded dash is the strongest
performer**, and the camera as currently built is a slight liability.

**Scope / honesty (important).** This does NOT mean the camera is useless. The sim's
open-loop dash computes its collision-lead from the target's known pre-flight kinematics —
i.e. it assumes the aim IS right. In the real world the pre-flight aim will be WRONG (target
motion uncertainty, wind), and the camera's job is to correct that residual (the ±30° aim-error
acquisition envelope, ADR-0076 #18b, is where it earns its keep). So the correct reading is:
**at correct aim the camera can't beat the dash and its aspect bias hurts; its value is
aim-ERROR robustness + the two levers that would let it actually improve on 0.75 m are (1)
aspect-bias COMPENSATION (`--terminal-bearing-bias-deg`, direction-keyed pre-flight constant,
honesty-legal, UNBUILT) and (2) the aim-error-recovery test (camera vs dash at a DELIBERATELY
wrong bias — the real-world case, not yet flown).** These two are now the highest-value next steps,
above finishing the cap/loft characterization arms.

### SYNTHESIS (cross-arm, existing data — the decomposition payoff)

Reading A/Adash (bias 30) and G20/G20dash (bias 20) together isolates what the camera
terminal actually DOES. Δ = dash_only_miss − camera_miss (paired medians; Δ>0 = camera
tightens/recovers, Δ<0 = camera hurts):

| condition | l2r dash | l2r cam | l2r Δ | r2l dash | r2l cam | r2l Δ |
|-----------|---------:|--------:|------:|---------:|--------:|------:|
| bias 30 — l2r WRONG aim, r2l ~ok | 2.05 | 0.68 | **+1.36** | 0.77 | 2.08 | −1.32 |
| bias 20 — l2r CORRECT aim, r2l ~ok | 0.73 | 0.93 | −0.20 | 0.81 | 1.08 | −0.27 |

**The camera is an AIM-ERROR CORRECTOR, gated by the aspect bias.**
- When the open-loop dash is MIS-aimed (l2r @ bias 30), the camera recovers it hugely
  (2.05 → 0.68 m). This is the camera architecture's whole premise validated: real pre-flight
  aim is never perfect (the sim's is, because it reads the target's true kinematics to size the
  lead), and the camera pulls the residual out.
- When the dash is ALREADY well-aimed (l2r @ bias 20), the camera's aspect bias has nothing to
  fix and costs a little (−0.20 m).
- On **r2l the camera HURTS at both biases** (worst 0.77 → 2.08) because the ADR-0056 aspect bias
  there is large and opposite-signed (λ−gtLOS −17..−21° vs l2r's +9..+17°) — it steers a
  well-aimed r2l dash off.

**Consequence for "smallest intercept distance":** the two honest levers are now sharp.
(1) **Correct the aim** (a free pre-flight constant / an accel-aware collision-lead) → gets the
open-loop dash to ~0.75 m and is what the camera then defends. (2) **Compensate the aspect bias**
(`--terminal-bearing-bias-deg`, direction-keyed pre-flight constant) → would let the camera
recover aim error on BOTH aspects instead of hurting r2l — the lever that makes the camera a
reliable corrector, not just an l2r one. Build + validate on a DISJOINT seed (calibrate the bias
on seed 123, test on 777 — no fitting to the test flights). This, not more cap/loft
characterization, is the path to a sub-0.75 m camera-guided kill.

### Aspect-bias COMPENSATION validation (seed 777, disjoint from the seed-123 calibration)

The `--terminal-bearing-bias`/`--terminal-los-lag` compensation was built and calibrated on
ARM B seed 123 (framed, LOO −79% stable, +11.0/−17.8). Validating on the disjoint seed 777:

**Uncompensated framed baseline — CG-777** (bias 20 + loft + up10, camera-live):
combined median **1.92 m** (l2r 1.68, r2l 2.35), Pk@2.5 6/8, Pk@1.0 0/8.

Two findings that redirect the validation (both from `measure_aspect_bias.py --loo`, offline,
BEFORE spending more sim — the "readable null for free" the tool exists for):
1. **Framing HURTS at correct aim.** CG framed (1.92 m) is WORSE than G20 level (1.32 m) on the
   same seed 777. The loft + up10 wedge degrades the terminal geometry once the aim is already
   correct — reinforcing that correct aim is the dominant lever and the framing levers are
   net-negative at 9 m/s.
2. **CG's own aspect bias is NOT STABLE** (l2r spread 74° sign-flipping, r2l spread 20°,
   LOO **+1%** — compensation makes it WORSE). The STABLE, calibratable bias existed only in
   ARM B, whose **accel-cap** slowed the approach → many stable ENGAGE ticks. Without the cap
   (CG), the fast approach is dominated by the **LOS-rate LAG** (finding A), which is large and
   flight-varying → no fixed constant fits it. **⇒ CH (the direction-keyed constant on CG) is a
   predicted NULL — SKIPPED** per the offline tool's "do not spend the arm."

**Redirect:** the decisive remaining test is the **lag knob on the BEST (level) arm** —
**GL = G20 + `--terminal-los-lag-ms 190`** — because the lag is direction-agnostic + self-signing
(needs no stable constant) and the level arm is fast-closing where the ~190 ms lag dominates the
terminal error. Gate: GL-777 beats the G20-777 baseline (combined 1.32 m) → the lag is a real,
transferable terminal lever. Non-improvement = honest null (the in-flight lag isn't the offline
regression's 190 ms, or α-β already absorbs it).

### GL — lag knob on the level arm: NULL on the primary gate.

`logs/mc_fp_armGL_line9_s777.csv` (level, bias 20, `--terminal-los-lag-ms 190`) vs the
G20-777 uncompensated baseline (1.32 m):

| dir | GL (lag) median | G20-777 median | GL better / paired |
|-----|----------------:|---------------:|:------------------:|
| l2r | 0.78 (0.62–1.58) | 0.61 | 2/4 (lag HURTS the already-tight side) |
| r2l | 1.80 (1.29–2.40) | 2.13 | 3/4 (lag HELPS the aspect-biased side) |
| **combined** | **1.44** | **1.32** | **5/8** |

**Verdict: NULL on the pre-registered gate** (combined 1.44 > 1.32, 5/8 < 6/8). The fixed
190 ms shifts error from r2l (helped, 2.13→1.80) to l2r (hurt, 0.61→0.78) — too much for
l2r, not enough for r2l — so the in-flight lag is not a clean direction-agnostic constant
(the α-β filter already absorbs a direction-varying share). Both terminal-precision
compensations are now nulls at the level (constant = regime-confounded, lag = this).

**Honest bottom line for smallest-miss:** the terminal-bearing wall (ADR-0056) does not yield
to a simple constant or lag correction — consistent with the prior terminal-precision nulls
(subpixel ADR-0071). **Correct AIM is the dominant, proven lever** (0.75 m open-loop dash-only,
now DERIVED by the accel-aware lead ADR-0080); the camera's role is defending imperfect
real-world aim, not beating a well-aimed dash. Keep the terminal simple; spend the effort on
aim (auto-corrected) + real-data detection (the outdoor acquisition gap), not terminal tricks.

### AAL — accel-aware lead (ADR-0080) VALIDATED: auto-aim reproduces the hand-tuned 0.75 m.

`logs/mc_fp_armAAL_line9_s123.csv` (`--dash-accel-aware-lead`, NO manual `--dash-crossing-bias-deg`,
dash-only) vs G20dash-123 (hand-tuned +20):

| dir | AAL (auto-aim) median | G20dash (+20 hand) median | within 0.2 m of pair |
|-----|----------------------:|--------------------------:|:--------------------:|
| l2r | 0.74 (0.64–0.82) | 0.73 | 3/4 |
| r2l | 0.69 (0.57–0.81) | 0.81 | 3/4 |
| **combined** | **0.71** | 0.75 | **6/8** |

**Pre-registered gate MET** (median 0.71 in 0.75±0.15 AND 6/8 within 0.2 m). The physics-derived
aim reproduces — and marginally beats — the hand-tuned constant, with NO per-config tuning. This
validates ADR-0080's auto-correction in flight: the crossing-bias is a derived kinematic constant,
not a knob to hand-set per airframe/speed. It removes the aim-tuning burden from the real build and
generalizes across the sweep. Disjoint seed 777 replication in flight.
