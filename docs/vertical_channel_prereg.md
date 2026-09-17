# Pre-registration — the vertical channel (dash altitude trim, then a terminal law)

> **Written 2026-09-10, BEFORE any arm flies.** The project's rule: the config, the
> prediction, the adopt/reject criterion, **and what a null would mean**, in writing
> and committed, then it flies. A criterion chosen after seeing the numbers is not a
> criterion.
>
> **Nothing in this document has flown.** Not in sim, not on hardware. The code it
> pre-registers is committed **default-OFF and byte-identical**, and the byte-identity
> is proven by test, not asserted.
>
> Origin: builder question 2026-09-09 ("I've watched a video of an aim get completely
> saved by a quick adjustment at the end with the camera... is there really a physics
> limit?"). Ledger: `terminal-vertical-channel-decided-not-built`.
> Analysis: `docs/vertical_channel_analysis.md`.
>
> ## ⚠️ SECTIONS 1-5 REST ON A MEASUREMENT THAT WAS WRONG. READ §8 FIRST.
>
> The `+0.320 m` dash altitude drift and the `66%` share that motivate H1 below
> were measured against a baseline that pooled **every** pre-dash tick, including
> 87-108 `TAKEOFF` ticks. `alt_m` is height above the arm point, so it reads
> `0.000` **on the ground** — most of that number was the takeoff climb.
>
> Corrected on 2026-09-10 against a settled pre-dash hover: the dash drift is
> **−0.019 m**, and only **10%** of the vertical error is a dash term. **66%
> arrives after handoff.** H1's premise — "the vertical error is delivered by the
> dash" — is **refuted on this fleet**, and §4's adopt criterion no longer
> transfers. §8 carries the corrected numbers, the second lever, and the
> registered prediction (a null). Sections 1-5 are kept unedited below as the
> record of what was believed, with each superseded number struck.

---

## 1. What is already measured, and what is only inferred

**Measured (offline, from committed logs, reproducible today):**

| Quantity | Value | Tool |
|---|---|---|
| Vertical separation at closest approach, cue-era fleet | ~~+0.485 m, 24 of 24~~ → **+0.508 m** median, above the target on **16 of 16** measurable flights (8 files carry no dash phase and were being counted as measured) | `scripts/forensics/vertical_miss_anatomy.py` |
| Altitude drift across the dash, same fleet | ~~+0.320 m~~ → **−0.019 m** median, n=16, against a SETTLED baseline (robust −0.012 to −0.025 across six baseline windows). The old figure was the takeoff climb. | same |
| Drift as a share of the vertical miss | ~~66%~~ → **10%** (paired per-flight median, range 1-43%). The post-handoff term is **66%**; a further 23% is a pre-dash datum offset. | same |
| Closing speed, last 1.0 s before handoff | **17.60 m/s** median, n=14 | `scripts/forensics/handoff_closing_speed.py` |
| Closing speed after handoff, pre-CPA | **14.64 m/s** median, n=13 | same |
| Terminal correction capacity at those speeds, AprilTag path | **0.02–0.17 m** | `scripts/forensics/terminal_capacity.py` |

**Measured on a different, gitignored fleet (ADR-0095, adopted coded-dash config,
n=16):** vertical **−0.374 m** against horizontal 0.174 m, i.e. **82% of the squared
miss** on the vertical axis, and the median 0.412 m against a 0.35 m ram radius.

**Inferred, not measured:**

- That the drift is caused by a P-only altitude loop losing authority under the
  27–36° dash pitch. The committed CSVs carry **no attitude columns** (deep-audit
  DEEP-R2), so the pitch-to-drift link is reasoning, not data.
- That the cue-era drift number transfers to the coded-dash arms. **It does not
  transfer in sign**: the cue-era arms end **high**, ADR-0095's fleet ended **low**.
  The mechanism transfers; the number does not.

---

## 2. The prediction, stated before flying

**H1 (the dash trim). ⚠️ PREMISE REFUTED 2026-09-10 — see §8.** As written: "the
vertical error is delivered by the dash, so setting `dash_alt_trim_m` to
`derive_dash_alt_trim_m(measured drift of THIS fleet)` will reduce the vertical
component of closest approach by most of that drift." Measured against a settled
baseline the dash delivers **−0.019 m**, a 10% term. The hypothesis is kept
verbatim here because it was registered; it is not the live prediction.

**H2 (the terminal law, NOT yet built).** A terminal vertical law will do almost
nothing on the AprilTag path, because capacity there is 0.02–0.17 m. It should only
be expected to pay on the markerless path at a 20 m acquisition range, where capacity
is about 3.9 m.

**Ordering claim.** H1 must be tested first. Running H2 first on the tag path would
produce a null that says nothing about vertical guidance and everything about
time-to-go, and that null would then be quoted for years.

---

## 3. Configuration to fly

Everything below is committed already; only the flag values change.

**Step 0 — derive the constant on the right fleet.** On the dev machine, where the
per-tick archive lives:

```
python3 scripts/forensics/vertical_miss_anatomy.py --glob 'logs/mc_fp_*.csv' --phase CODED_DASH
python3 scripts/forensics/handoff_closing_speed.py --glob 'logs/mc_fp_*.csv' --phase CODED_DASH
```

Take the printed dash drift, pass it through `derive_dash_alt_trim_m`, and use that.
**Do not reuse −0.32 m from this document** — it is the cue-era number and its sign is
configuration-dependent.

**Arms**, paired seeds, n = 8 minimum, one master seed, identical seeds per arm, the
adopted coded-dash configuration, and the trim as the **only** difference:

| Arm | Configuration |
|---|---|
| A (control) | adopted config, `dash_alt_trim_m = 0.0` |
| B | adopted config, `dash_alt_trim_m` = derived value |
| C (optional) | adopted config, trim at `share=0.5` — a dose-response point |

**Prerequisite, not optional.** The past-closest-approach breakoff discriminator
(live queue item 1) is **arm-asymmetric**: it fires in the ENGAGE terminal, so it can
only affect arms whose camera engages. Until it is fixed, any camera-involving margin
is confounded. If arms A and B both run the same terminal this confound is common-mode
and cancels from the *difference*, which is why this arm is runnable before that fix
— but the **absolute** numbers are not quotable until it lands. Say that in the result.

---

## 4. Adopt / reject criterion, fixed now

Primary metric: **the vertical component of closest approach**, centre to centre,
interpolated, as `scripts/rescore_cpa.py` computes it.

- **ADOPT** if the paired vertical component tightens on **at least 6 of 8** seeds and
  the median vertical magnitude falls by **at least half the derived drift**.
- **REJECT** if it tightens on 4 or fewer of 8, or if total closest approach gets
  worse on 5 or more seeds (a trim that fixes vertical by spending horizontal is not
  a win).
- **UNDERPOWERED** — say so, adopt nothing — if the suite does not complete 8 paired
  seeds. n below the floor is not a small result, it is no result.

Secondary, reported but not decisive: total closest approach, clean rate, and the
count inside the 0.35 m ram radius. The count inside the radius is the goal but it is
a coarse, low-n statistic; the vertical component is the quantity the lever acts on.

---

## 5. What a NULL means — written before the numbers exist

**If the trim does not move the vertical component**, then the vertical error is not
delivered by the dash in the way the cue-era fleet suggests, and the drift measurement
does not generalise. In that case the next candidates, in order, are:

1. **The altitude datum.** ADR-0085 chose the arm-point local datum; if the target's
   own altitude is offset from it, no trim in the world fixes a bookkeeping mismatch.
   That would make this a **scoring** finding rather than a guidance one — the same
   class as the ruler retraction, and it would be the third instrument-layer defect in
   the vertical channel.
2. **The target's commanded altitude** in the sim scenario, which is a harness
   constant, not a vehicle behaviour.
3. **A terminal vertical law after all**, but only on the markerless path where there
   is capacity to spend.

**§5's first candidate turned out to be the right one.** "The altitude datum" is
listed below as what to look at if H1 nulls — and the corrected decomposition puts
**+0.127 m (23%)** in exactly that term, a mismatch between an AGL-above-arm-point
reference and a world-z target altitude. §8.5 has the arithmetic. Written before
the numbers existed, which is the whole point of the ritual.

**A null does not restore "the camera cannot help".** It relocates the vertical error,
which is 82% of the squared miss on the adopted config either way. The finding that the
terminal has no vertical channel stands regardless of this arm's outcome, because it is
a fact about the code and not about a measurement.

**And if the trim works, one thing it still does not prove:** that the camera terminal
earns its handoff. The trim is an open-loop pre-flight constant. A tighter intercept
from a better-aimed dash is still a ballistic result, and the registered assumption
`camera-terminal-earns-handoff` stays **violated** until a camera arm beats its
dash-only twin. Do not let a vertical win be reported as a seeker win.

---

## 6. The AGL floor is not part of this experiment

`min_agl_m` is a **flight-safety backstop**, not a guidance term: the altitude
reference is never allowed below it, whatever the trim or a future seeker elevation
asks for. It is `None` by default, which is the pre-existing unfloored behaviour and
is **not a safe default for a props-on flight**.

It needs a value from the site's own geometry before the first real flight, and it is
gated on the builder, not on any arm here. It is tested (the floor beats the trim, and
it applies in every state, not only the dash) but it has never been exercised on
hardware.

---

## 7. Honesty accounting

The trim is a **pre-flight constant** derived from the vehicle's own past logs. Under
the extended boundary rule — not *when* a value is read, but *whether a real system
could obtain it at that quality* — a real interceptor genuinely can measure its own
dash altitude drift from its own flight logs and trim for it. So this is
architecturally clean in a way the launch cue is not.

What it needs in the assumptions register, once a value is non-zero:

- `dash-alt-trim-transfers` — **unmeasured**. The trim is derived from sim logs and
  applied to a real airframe whose drift has never been measured. If wrong, the lead
  solve is vertically mis-aimed by the difference, and the sign is
  configuration-dependent, so a wrong-signed trim **doubles** the error rather than
  leaving it unchanged. That asymmetry is the reason for the `share` parameter.

The AST honesty audit passes with the change in place (0 ground-truth identifiers,
0 simulator imports, dash heading latched at exactly one site).

---

## 8. Pre-registration — the ENGAGE dropout vertical hold (`--dropout-hold-vertical`)

> **Written 2026-09-10, BEFORE any arm flies.** Added because the code comment in
> `flight/guidance.py` and the `--dropout-hold-vertical` help text both claimed to
> be "pre-registered in this document" while this section did not exist (review
> finding S3). A false pre-registration claim is the worst place in the project
> for a false claim, so it is either written or struck; this is written.

### 8.1 What changed above, and why H1 is now expected to be a null

Sections 1–5 were written on a **measurement that has since been corrected**.
The dash altitude drift was `+0.320 m`, measured against a baseline that pooled
every pre-dash tick — including 87–108 `TAKEOFF` ticks, and `alt_m` reads `0.000`
on the ground, so most of that number was *the takeoff climb*. Against a settled
pre-dash hover the drift is **−0.019 m** (robust from −0.012 to −0.025 m across
six baseline windows). The additive decomposition, paired per-flight medians,
n=13 of the 16 committed cue-era flights:

| Term | Median | Paired share | Range |
|---|---|---|---|
| `origin` — settled pre-dash separation | **+0.127 m** | 23% | 15–30% |
| `dash_delta` — hover → dash end | **+0.012 m** | 10% | 1–43% |
| `post_dash_delta` — dash end → CPA | **+0.348 m** | 66% | 29–78% |

**So H1's premise is refuted on this fleet.** The vertical error is not delivered
by the dash, and `dash_alt_trim_m` acts on the 10% term. H1 may still be run — a
trim that nulls 10% is not worthless — but it must be reported as such, and its
adopt criterion in §4 ("at least half the derived drift") is now a criterion on a
drift of 19 mm, i.e. on nothing. **§4's criterion does not transfer; do not reuse
it.** Re-derive the constant on the coded-dash fleet first (§3 Step 0), and expect
`vertical_miss_anatomy.py` to report **no settled baseline** there, because the
coded-dash configuration goes `TAKEOFF → CODED_DASH` with no hover. That is the
tool failing closed, not a bug, and it means **there is no measured drift to trim
on the adopted configuration at all**.

### 8.2 The lever

`--dropout-hold-vertical` (default OFF, byte-identical when off). The ENGAGE
terminal recomputes an altitude-hold P-loop on every tick with a fresh detection
but abandons it on a dropout: the far branch issues an all-zero setpoint and
stores it as `last_cmd`, the near branch re-issues that zero. Measured on ENGAGE
ticks where the loop would have commanded >0.01 m/s: vertical zeroed on **0/150**
ticks *with* a detection and **792/877** *without* one, latching for the rest of
the terminal on **10 of the 14 flights that reached ENGAGE** (2 of the 16 never
engaged — the denominator is 14).

### 8.3 The prediction, stated before flying: **a NULL**

**H3. Enabling the dropout vertical hold alone will NOT measurably reduce the
vertical component of closest approach.** Two independent reasons, both measured
before the arm:

1. **The vertical and horizontal zeros are the same event.** The far-dropout
   branch zeroes all three velocity components together: both zeroed on 792
   ticks, vertical-only on **0**, horizontal-only on **0**. Perfectly collinear,
   no discordant tick. And the horizontal event is far larger — commanded
   horizontal speed goes from **16.00 m/s at the dash tail to 0.00 m/s** on the
   first far-dropout tick. A multicopter handed a full-stop brake at 16 m/s
   pitches up and balloons, which is a sufficient and much more energetic
   explanation for a +0.35 m climb than a missing 0.5 m/s-authority altitude
   hold.
2. **The available control arm argues against it.** On the 4 flights where the
   vertical command *survived*, the vehicle climbed **0.28–0.65 m anyway**, while
   its mean commanded vertical velocity was a *descent*. Their post-handoff term
   is indistinguishable from the latched flights (+0.429 m, n=4 vs +0.498 m,
   n=10). The latch is also confounded with terminal duration — latched terminals
   run 2.2–2.8 s against 0.4–0.8 s — so "latched" is largely a proxy for "the
   terminal lasted longer".

**Therefore this arm must NOT be run standalone.** Because (1) makes the two
channels collinear, a vertical-only arm cannot attribute any result. Run it as a
**2×2 with `--terminal-coast-gate`**: {vertical hold off/on} × {coast gate
off/on}, paired seeds, n≥8 per cell. The coast-gate factor is what breaks the
collinearity; without it there is no identifiable vertical effect to measure.

### 8.4 Adopt / reject criterion, fixed now

Primary metric: the vertical component of closest approach, as §4 defines it.

- **ADOPT** only if the vertical component tightens in the 2×2's vertical main
  effect on **at least 6 of 8** paired seeds *with the coast gate held constant*,
  and the effect survives at both coast-gate levels.
- **REJECT** — and this is the expected outcome — if the vertical main effect is
  within run-to-run noise (~1 m terminal-dropout noise, so any effect below that
  at n=8 is not an effect).
- **UNDERPOWERED**, adopt nothing, if any cell does not complete 8 paired seeds.
- **KEEP THE CODE EITHER WAY.** The argument for the fix is architectural, not
  empirical: `alt_m` is own-state, carries no target information, and is available
  every tick, so abandoning the altitude hold during a *camera* dropout needs
  nothing the vehicle has lost. It buys no robustness; it is a pure loss. A null
  says the defect is not what dominates the vertical miss — not that the defect
  should stay.

### 8.5 What a NULL means — written before the numbers exist

A null does **not** exonerate the dropout branch, and it does **not** restore
"the dash delivers the error". It relocates the vertical error to the two terms
this lever does not touch:

1. **`origin`, +0.127 m (23%) — a two-datum bookkeeping mismatch, and the
   cheapest 23% in the project.** `ALT_REF_M = 0.5` is AGL above the *arm point*;
   the target's altitude is *world* z. Nothing reconciles the ~0.22 m the camera
   sits above ground at rest. It is real separation — the interceptor genuinely
   passes 0.127 m above the tag centre and a real intercept has to close it — so
   it cannot be subtracted from a miss statistic. But it is **aim, not guidance**,
   and putting the two references in one frame removes it with a one-line
   scenario change and no guidance work. That is the highest-value item this
   analysis produced and it should be done before either lever flies.
2. **The horizontal brake**, per §8.3(1) — the 16 m/s → 0 full-stop command,
   which is the `coast_zero` defect's own lever, not this one.
3. **The camera lever arm.** `gt_cam_z − alt_m` should be a constant geometric
   offset; measured it moves **+0.033 m** hover→dash-tail and **+0.087 m**
   hover→CPA, i.e. **24% of `post_dash_delta` is the camera swinging on its
   forward boom, not the airframe moving**. It cannot be removed from these logs
   (no attitude columns, deep-audit DEEP-R2), so a chunk of the "vertical miss"
   is currently un-attributable. Log attitude before quoting a vertical budget
   again.

### 8.6 A prior question neither lever answers

The altitude loop shows a persistent **≈ −0.087 m steady-state error in hover**
(`alt_m` 0.4135 against `ALT_REF_M` 0.5) while continuously commanding
−0.087 m/s and gaining ~0.001 m. **The commanded vertical velocity is not being
delivered.** Both this lever and ADR-0099's trim assume that loop has authority.
This cannot be root-caused without a sim run, and it should be measured before
either lever is priced — a loop that does not track its own setpoint will not
track a trimmed one either.

### 8.7 Honesty accounting

`alt_m` is own-state (EKF/barometer) and carries no target information, so
recomputing the altitude hold during a camera dropout is architecturally clean:
it uses nothing the vehicle lost and nothing it could not measure. The AST
honesty audit passes with the change in place.

**Configuration caveat that must accompany every number in this section.** They
come from the **cue-era two-stage `--handoff` fleet of 2026-07-09, phase `DASH`,
not `CODED_DASH`** — off the adopted configuration. 2 of 16 flights never
engaged. The terminal had a detection on **155 of 1032 ENGAGE ticks (15%)**, so
these "camera terminals" were mostly hovering, not seeking. Nothing here has
flown with the lever on, in sim or otherwise, and no number in this section was
produced by a simulator during this session — they are all offline scores of
committed logs from a container with no sim (`docs/verification_environment.md`).

---

## 9. Pre-registration — reconciling the two altitude datums (`--alt-ref-offset-m`)

> **Written 2026-09-16, BEFORE any arm flies.** The code is committed
> **default-OFF (`0.0`) and behaviour-identical**, and the identity is proven by
> test (`tests/test_rescore_cpa.py::test_alt_ref_default_is_identity` plus two
> mutant-style tests that were checked by actually breaking the code and watching
> them fail), not asserted. Nothing in this section has flown.
>
> Origin: `docs/next.md` item 3a. This started life as "a desk edit, one line of
> scenario setup". It is not: it changes the altitude every baseline in the project
> was flown at, so it is an A/B and it gets a pre-registration like any other.

### 9.1 The thing in one paragraph, for a reader who has not been following

The interceptor holds its height against **its own relative altitude** — MAVSDK's
`relative_altitude_m`, which this code calls `alt_m` and which reads **0.000 while the
aircraft is still standing on its landing gear**. The target's height, by contrast,
is a **world** height: the tag's centre sits at world *z* = 0.5 m. The hold reference
`ALT_REF_M` is also 0.5. Those two 0.5s look like they line up. They do not, because
`alt_m = 0` is not world *z* = 0 — it is the top of the landing gear. The aircraft
therefore hovers with its **body about 0.23 m higher than the number suggests**, and
that same ~0.20 m comes back at the end of the flight as the **vertical part of the
closest approach**. It is a bookkeeping error in the scenario, not a guidance failure.

**Carry the datum with the number** (the rule from ADR-0095, which is exactly the
error this section could repeat): the ~0.23 m is **landing-gear rest height**, *not* a
camera boom. The camera is **2 mm** above `base_link`. Two independent measurements
in `scripts/rescore_cpa.py` agree to under 1 mm: the resolved x500 SDF chain gives
camera − `base_link` = +0.002 m, and the **on-pad ticks of every flight in `logs/`**
read `gt_cam_z = 0.2290 m` at `alt_m = 0` against a computed leg rest height of
0.2270 m. Anyone who writes "the camera sits 0.23 m above the airframe" has re-made
the retracted `+0.208 m` claim.

### 9.2 The lever

`--alt-ref-offset-m <metres>` is **added** to the altitude reference everywhere the
hold and the takeoff use it (one helper, `alt_ref_m(args)`, resolved once per flight
and printed once into the run log). Negative lowers the aircraft. `0.0` is the shipped
default and is the old behaviour exactly.

It is a **pre-flight constant of the vehicle's own geometry**, typed at launch the same
way `--wind-trim-mps` is. It is never computed in flight, and never from anything the
vehicle could not measure about itself.

### 9.3 How the number is measured — and which route is the better one

**Route A (the one to use): the pad.** Every flight begins with the aircraft at rest,
so every flight already contains the constant. Take the on-pad ticks (`alt_m` ≈ 0,
before takeoff) and read `median(gt_cam_z)`. That is the camera's height above the
`alt_m` datum. `rescore_cpa.py` already computes and **checks** this per flight against
`BASE_REST_ABOVE_GROUND_M + CAM_UP_M` with a 0.02 m tolerance, and fails closed when it
disagrees — a tool that has to name its reference frame to run at all, which is the only
kind of check that catches this class of error.

**Route B (the confirmation, and what item 3 actually measured): a settled hover.**
Fly, hold height, and over the settled hover window — after the takeoff transient, before
the dash — take `median(gt_cam_z − gt_tag_z)` per flight. That is the camera's height
above the target's centre, and it already folds in whatever steady-state error the
altitude loop carries (measured −0.022 / −0.023 m, so small). Set

    --alt-ref-offset-m = −median(gt_cam_z − gt_tag_z)

Both `gt_*` columns are **scoring/logging** and are read **offline, after the flight**.
Nothing in the tick loop touches them; the honesty boundary is not in play here. The
flag's value is typed by hand before the next flight.

**Predicted value: about −0.20 m.** From Route A, 0.2290 m of gear, plus the loop's
−0.022 m steady error, minus nothing else: 0.2290 − 0.022 = 0.207 m of camera above a
0.5 m target. The two flights of 2026-09-16 measured 0.161 m and 0.204 m.

**The 0.161 m flight is a discrepancy that must be resolved before the number is
fixed, not after.** Its `gt_cam_z − alt_m` was 0.184 m against the pad's 0.2290 m — a
**4.5 cm** gap in a quantity that is supposed to be rigid geometry, and larger than
`rescore_cpa.py`'s own 0.02 m pad tolerance. Two candidate explanations, neither
confirmed: the camera is 0.120 m **forward** of `base_link`, so a nose-down attitude
drops it (0.12·sin θ; it needs θ ≈ −21° to explain 4.5 cm, far more than a hover should
hold), or the `alt_m` datum itself moved between flights. **Action before the arms fly:**
run the pad check over the two flights and take Route A's pooled pad value if Route B's
two numbers still disagree by more than 2 cm. Do not average a rigid constant with an
anomaly.

**Honesty grading (assumptions register).** Two inputs hide inside one number:

| input | grade | note |
|---|---|---|
| camera height above the `alt_m` datum (~0.229 m) | `measured` | The vehicle's own geometry, from two independent routes. A real airframe measures this with a tape measure. Clean. |
| the target's centre height (world *z* = 0.5 m) | `given-perfect` | **Not** vehicle geometry. In sim the scenario places the tag at exactly 0.5 m and the code knows it. A real operator estimates the threat's height and is wrong by some amount. |

Those two only collapse into one clean vehicle constant **because the arm point sits on
the same ground plane the target's 0.5 m is measured from, and the target's height
happens to equal `ALT_REF_M`.** The moment the target flies at a height the operator has
to guess, this lever inherits that guess's error — exactly the shape of the launch-aim
loophole, one datum over. Register it; do not let a win here be reported as a result
that transfers to an unknown-altitude target.

### 9.4 Configuration to fly

The **adopted coded-dash configuration**, from `docs/project_state.json` (stage
`coded_dash`, `active`: "`--coded-dash` + `--dash-crossing-bias-deg` (per-direction aim
bias auto-keyed on crossing direction); dash-speed 16; cue/FusedTrack/handoff-gate/
coast-search stack DROPPED") and its concrete launcher base in
`docs/flight_plan_candidates.md`:

```sh
BASE="--coded-dash --fpv --dash-unclamp --dash-speed 16"
# plus the per-direction crossing bias, and the canonical geometry:
# --mode m4 --laws pronav --path line --geometry standard --x0 6.5 --y0-mag 15.343
# --speeds 9.0 --directions both --n 8 --master-seed 123
```

| Arm | Configuration |
|---|---|
| **A (control)** | adopted config, `--alt-ref-offset-m 0.0` (i.e. omitted — the shipped default) |
| **B** | adopted config, `--alt-ref-offset-m −d`, `d` = the measured camera-above-target height from §9.3 |

Paired seeds, **n ≥ 8**, one master seed, identical seeds in both arms, both crossing
directions, and the offset as the **only** difference. Run the arms **sequentially**, at
idle machine load.

**Do not fly this on the tag gate and call it done.** The 2026-09-16 measurement is
n = 2 flights of the classic AprilTag scenario, which has an `ACQUIRE` hover the adopted
coded-dash configuration does not have (it goes `TAKEOFF` → `CODED_DASH` with no settled
hover at all). That is why Route A exists.

**The standing confound, unchanged:** the past-closest-approach breakoff discriminator
(live queue item 1) fires only in the `ENGAGE` terminal. If both arms run the same
terminal it is common-mode and cancels from the **difference**, which is why this arm is
runnable before that fix — but the **absolute** numbers are not quotable until it lands.
Say so in the result.

### 9.5 The prediction, stated before flying

1. **The vertical component of closest approach falls by roughly `d`** (≈ 0.20 m). On
   the 2026-09-16 pro-nav flight the vertical component *was* 0.201 m against a hover
   offset of 0.204 m — the offset was carried through the whole flight essentially
   untouched, which is what makes the prediction a near-identity rather than a hope.
2. **The horizontal component is unchanged.** The lever touches only the vertical
   channel. Horizontal on that flight was 0.327 m and should stay there, within noise.
3. **Total closest approach therefore falls**, but by less than `d`, because it is the
   hypotenuse: √(0.327² + 0.201²) = 0.384 m becomes ≈ 0.327 m if the vertical goes to
   zero. **That is a 0.06 m improvement in the headline number from nulling a 0.20 m
   error** — say that out loud now, so nobody is disappointed later by a correct result.
4. **No effect on acquisition, engage rate, or abort rate** is predicted. If one moves,
   see §9.7 — it is probably an arm-asymmetric artefact, not a finding.

### 9.6 Adopt / reject criterion, fixed now

Primary metric: **the vertical component of closest approach**, centre to centre,
interpolated, as `scripts/rescore_cpa.py` computes it.

- **ADOPT** if the paired **vertical magnitude** tightens on **at least 6 of 8** seeds
  **and** the median vertical magnitude falls by **at least half of `d`**.
- **REJECT** if it tightens on **4 or fewer of 8**, **or** if **total** closest approach
  gets worse on **5 or more** seeds (buying vertical with horizontal is not a win), **or**
  if arm B's clean-flight count drops — a lever that trades accuracy for aborts is worse
  than the error it removes.
- **UNDERPOWERED — adopt nothing, and say so** — if the suite does not complete 8 paired
  seeds per direction, or if any flight in either arm fails `rescore_cpa.py`'s pad check
  (a failed pad check means the geometry the whole lever is derived from did not hold on
  that flight, so the number has no denominator).
- **A verdict computed on zero usable flights is UNCERTAIN, never PASS.** Count the
  dropped flights and report the count; never absorb it.

Secondary, reported but not decisive: total closest approach, the count inside the
0.35 m ram radius, engage rate, abort reasons.

### 9.7 Which failure modes each arm can reach (the arm-asymmetry check)

This is the check that matters most here, because **arm B flies 0.20 m lower than any
baseline in this project** and several failure modes are reachable by only one arm. A
defect that only one arm can suffer manufactures a fake effect; when that happens the
**direction** may survive but the **margin is not quantitative**.

| Failure mode | Arm A (offset 0) | Arm B (lower) | Why it is asymmetric |
|---|---|---|---|
| **Ground proximity / rotor ground effect** | far from it | closer to it | Arm B's rotor plane sits 0.20 m nearer the ground, so aerodynamic ground effect and any terrain contact risk rise for B only. The size of this is **unmeasured** — derive it from the model's rotor diameter before flying, do not assume it is negligible. |
| **PX4 flooring a very low takeoff altitude** | requests 0.5 m | requests ~0.30 m | The code already warns that "PX4 may floor very low takeoff altitudes". If PX4 floors B's takeoff, B never reaches its own reference and the arm silently becomes a partial A. **Check the run log's printed altitude reference against the flown `alt_m` on every B flight.** |
| **Takeoff-complete gate** | 0.8 × 0.5 = 0.40 m | 0.8 × 0.30 = 0.24 m | `ALTITUDE_SUCCESS_FRACTION` scales with the reference, so B's gate is *easier* in absolute terms and could pass while still in ground effect. |
| **Target leaving the frame vertically (upward)** | baseline framing | target sits **higher** in frame | Lowering the aircraft raises the target's elevation angle in the image. On this configuration that should *help* (the known loss is off the **top** of the frame during the nose-down dash), but it is a framing change and it is only reachable by B. |
| **`min_agl_m` safety floor** | never engages | could engage | `None` by default (§6), so today it is inert in both arms — but if it is ever set, it will clip B's reference and not A's, turning the arm into a no-op without saying so. |
| **Past-CPA breakoff discriminator** | both | both | Common-mode if both arms run the same terminal. Cancels from the difference; still poisons the absolute numbers. |

**If arm B's abort rate, engage rate, or takeoff behaviour differs from A's, stop and
attribute it before reading the miss numbers at all.** A vertical-miss improvement
measured on a set of B flights that aborted differently from A's is not a paired
comparison.

### 9.8 Constants whose validity depends on altitude, and must be re-earned

A threshold validated at one operating point is not validated at another. Lowering the
flight by 0.20 m puts these on the list:

1. **`--wind-height-m` and the whole Dryden gust model.** The driver already refuses to
   run below 3.048 m AGL without `--wind-allow-below-10ft`, because the MIL-F-8785C
   low-altitude form is not valid there and its gust correlation time swings from 9.59 s
   at 6 m to 0.88 s at 0.5 m. Arm B is lower still. If any wind arm is flown at the new
   reference, the height of record changes and the extrapolation gets worse. Re-state it;
   do not reuse a wind arm flown at 0.5 m as B's control.
2. **The altitude loop's steady-state tracking error (−0.022 m, measured at
   `ALT_REF_M` = 0.5).** Measured at one reference, in one part of the envelope. If
   ground effect changes the thrust the loop needs, this number moves — and both this
   lever and ADR-0099's trim assume the loop has authority. Re-measure it in B's hover.
3. **`BASE_REST_ABOVE_GROUND_M` / the pad check tolerance (0.02 m).** Unchanged as
   geometry, but it is now load-bearing for a *flight* decision rather than only for
   scoring, so its per-flight check becomes a gate, not a diagnostic.
4. **The dash loft stack (`--dash-loft-m`, `--dash-loft-dive-s`, `--dash-vvert-max`).**
   The loft is added **on top of** the reference, so the offset shifts the loft apex and
   the dive's end height by the same amount. If a loft arm is flown, its dive was sized
   against the old reference and must be re-sized or explicitly declared unchanged.
5. **Camera framing constants sized against the level datum** — `--cam-mount-up-deg`,
   the adaptive-tilt lead/max angles, and the acquisition band. All were chosen to put
   the target in frame with the aircraft 0.20 m higher than it will now be.
6. **`BREAKOFF_ARM_RANGE_M` and the breakoff climb.** Breaking off upward from a lower
   start is a different manoeuvre in a different part of the altitude envelope.
7. **The 0.35 m ram radius.** Unchanged — it is a property of the airframe, not of the
   altitude — and it is the one number on this list that does travel.

### 9.9 What a NULL means — written before the numbers exist

**If the vertical component does not fall by roughly `d`,** then the hover offset is
*not* simply carried through to closest approach, and the 2026-09-16 n = 2 reading —
"the offset was carried the whole way" — does not generalise. That would be a genuine
and useful finding, because it says something *else* dominates the vertical channel
between handoff and CPA. In that case, in order:

1. **Check that arm B actually flew lower.** The run log now prints the effective
   reference once; the flown `alt_m` must match it. A floored takeoff or an engaged
   `min_agl_m` makes a null meaningless. **This check comes before any interpretation.**
2. **Look at the post-handoff term.** §8 already measured that **66%** of the vertical
   error arrives *after* handoff. If nulling the pre-existing 23% offset leaves the total
   vertical roughly where it was, the post-handoff term grew to fill the gap — which
   would point at the terminal, at the pitch-on-a-forward-camera-boom geometry, or at the
   descent the vehicle is commanded into, and would make the terminal vertical law
   (ADR-0085, still unbuilt) the live question rather than this one.
3. **Do not re-run with a bigger offset looking for an effect.** The offset is a measured
   constant, not a tuning knob. Sweeping it to find a better miss is fitting the scenario,
   and it would be a new experiment needing its own pre-registration.

**If it works, here is what it still does not prove.** It does not make the camera
terminal earn its handoff. This is an open-loop, pre-flight constant applied to a scenario
whose target height is known; a tighter intercept from a better-set-up dash is still a
**ballistic** result. The registered assumption `camera-terminal-earns-handoff` stays
**violated** until a camera arm beats its own dash-only twin. And per §9.3 the result is
scoped to a target at a **known** altitude — it says nothing about a threat whose height
the operator has to guess.

### 9.10 Honesty accounting

`alt_m` is own-state (EKF/barometer). The offset is a number typed on the command line
before launch. Neither reads the target, and `alt_ref_m(args)` is AST-checked to read
nothing but its `args` parameter and the module constant
(`tests/test_rescore_cpa.py::test_offset_is_a_preflight_constant_not_a_ground_truth_read_MUTANT`).
`tests/test_honesty_static.py` passes with the change in place.

The `gt_cam_z` / `gt_tag_z` columns used to *derive* the constant are scoring/logging,
read offline after the flight, by a human, into a flag. On real hardware the same
constant comes off a tape measure. The residual honesty debt is the one named in §9.3:
the target's height is `given-perfect` in sim, and this lever quietly depends on it.

### 9.11 The §9.3 discrepancy, resolved before flying (2026-09-16, head session)

The pad check was run on both 2026-09-16 flights: `median(gt_cam_z)` on the on-pad ticks
is **0.2290 m on both** (n = 22 and n = 64 ticks). The rigid constant is rigid. The 4.5 cm
gap is somewhere else: in hover the vehicle's **own altitude estimate** read 0.478 m and
0.477 m, while its true height above its pad rest position was **0.432 m** and **0.475 m**.
So on one flight the estimator was wrong by **+0.046 m** and on the other by +0.002 m.
Plain English: the vehicle holds the height it *believes* to about 2 cm, but what it
believes can itself be off by ~5 cm, flight to flight.

**Consequences, fixed now so they cannot be chosen later:**
- The value flown in arm B is Route A's: `d = 0.2290 − 0.022 = 0.207 m`, i.e.
  `--alt-ref-offset-m -0.207`. Not an average with the anomalous flight.
- A preset-height scheme has a **floor of roughly ±5 cm of vertical error that this lever
  cannot touch**, because it comes from the altitude estimate, not from the reference.
  n = 2, so "±5 cm" is an order of magnitude, not a statistic. The A/B's per-flight
  vertical misses will scatter by at least that; §9.6's "falls by ≥ d/2" criterion (≈0.10 m)
  is still twice that scatter, so it stands unchanged.
- Arm B's pre-check (§9.9): confirm from the run log that the flown reference printed
  0.293 m and that hover/takeoff `alt_m` actually reached it (PX4 may floor low takeoffs).

### 9.12 Sign check on the ADOPTED config, done before flying (2026-09-16, head session)

`docs/flight_plan_candidates.md` said the adopted arm (`AE5dash`) flies **below** the target
by 0.37 m. If true, arm B (which LOWERS the vehicle) would be flown in the wrong direction.
Checked against the per-tick logs of all 8 seed-123 `AE5dash` flights: at closest approach the
camera is at z = 0.79–0.94 m and the target at 0.50 m — the vehicle is **ABOVE** by
0.29–0.44 m (median ≈ 0.35 m). The "−0.37" was target-minus-interceptor and the prose read
the sign backwards; corrected there. So lowering is the right direction.

It also sharpens the prediction for this config, in writing before the flight:
- Of the ≈0.35 m, **≈0.21 m is the height-reference offset** this lever removes. The other
  **≈0.15 m is the vehicle climbing during the dash** (estimated altitude 0.58–0.73 m at
  closest approach against a 0.5 m reference), which this lever does NOT touch.
- **Prediction:** arm B's median vertical offset falls from ≈0.35 m to ≈0.15 m (still above).
  With the horizontal median at 0.226 m that predicts a 3-D median near
  √(0.226² + 0.15²) ≈ 0.27 m and **clearly more than 3/8 flights inside the 0.35 m contact
  radius**. If B instead lands at ≈0.35 m vertical, the offset did not reach the dash phase
  (wiring) — check the printed reference and `alt_m` before believing a null.
- **Arms as flown:** `AE5dash` exactly as in `scripts/experiments/flight_plans/run_arm.sh`,
  seed **456** (disjoint from the 123/777 flights the 0.37 figure came from), n = 8, both
  directions; A = no offset, B = `--alt-ref-offset-m -0.207`. Both dash-only (camera off), so
  the past-closest-approach breakoff defect cannot reach either arm.
- **Whatever it shows is a BEST-CASE number**: it leans on the target's exactly-known height
  (assumption `target-height-known`, grade given-perfect).

### 9.13 RESULT, seed 456 (2026-09-16) — criterion MET; replication on seed 789 launched under the SAME criterion

Both arms dash-only (0 ENGAGE ticks on 16/16 flights, so the breakoff defect reached neither).
Arm B's run log printed `Altitude reference: 0.293 m`, so the lever was live. Scored with the
project's primary ruler (`scripts/rescore_cpa.py`, airframe centre, interpolated):

| | A: adopted config | B: + `--alt-ref-offset-m -0.207` |
|---|---|---|
| median closest approach | **0.563 m** | **0.215 m** |
| inside the 0.35 m contact radius | **1/8** | **7/8** |
| median vertical offset at closest approach (lens, logged tick) | +0.412 m (above) | +0.185 m (above) |
| median horizontal (lens, logged tick) | 0.287 m | 0.114 m |

Paired by seed, B is closer on **8/8** flights (0.657→0.257, 0.541→0.148, 0.581→0.335,
0.392→0.173, 0.567→0.494, 0.313→0.073, 0.559→0.298, 0.643→0.144). Median vertical fell
0.227 m against the 0.207 m commanded — §9.6's bar was ≥6/8 and ≥0.10 m. **ADOPT, pending the
seed-789 replication** (launched before this section was written; its criterion is §9.6,
unchanged, and a failure there overrides this).

**What did NOT go as predicted, and is not explained:**
- **Horizontal got better too (0.287 → 0.114 m). §9.5 predicted "unchanged".** I do not have
  a mechanism. Candidates, none tested: at ~17 m/s closing and 20 Hz logging the vehicle moves
  ~0.85 m per tick, so "horizontal at the 3-D-closest tick" is not the horizontal closest
  approach and a big vertical offset shifts which tick is picked; or flying 0.2 m lower changes
  the dash itself. Until that is explained, quote the 3-D number and do not claim a horizontal gain.
- **The control is worse than the seeds the 0.37 figure came from** (vertical +0.41 vs +0.35;
  1/8 vs 3/16 inside). Same config, different seed — run-to-run spread, or something changed
  since July. The replication's control arm is the check.
- **The vehicle still ends ~0.19 m high**, because it CLIMBS during the dash: estimated altitude
  at closest approach is 0.57–0.78 m against a 0.50 m reference in arm A, at **−41° to −45° of
  pitch** (the new attitude columns — first time this has been visible). Left-to-right flights
  climb more (0.74–0.78 m) than right-to-left (0.57–0.65 m). That is the next ~0.2 m, and it is
  what `dash_alt_trim_m` was built for.
- One arm-B flight (#4, 0.494 m) carries the scorer's `yaw-check 25.9 deg` flag and barely
  improved. Counted, not dropped.

**Scope, so this cannot be over-read:** n = 8, one seed, sim only, camera OFF, launch aim
solved from the target's exactly-known path, target height exactly known
(`launch-cue-error-free`, `target-height-known` — both given-perfect). It is a BEST-CASE UPPER
BOUND on a ballistic pass, not a camera-guided kill.

### 9.14 REPLICATION, seed 789 (2026-09-16) — criterion MET again; ADOPTED (ADR-0100)

Same ruler, same criterion (§9.6), both arms dash-only:

| seed | A inside 0.35 m | B inside 0.35 m | A median | B median | paired B closer | median vertical fell |
|---|---|---|---|---|---|---|
| 456 | 1/8 | 7/8 | 0.563 m | 0.215 m | 8/8 | 0.227 m |
| 789 | 1/8 | 6/8 | 0.457 m | 0.249 m | 6/8 | 0.200 m |
| **pooled** | **2/16** | **13/16** | | | **14/16** | |

- **The seed-456 horizontal gain did NOT replicate** (789: 0.163 → 0.232 m, slightly worse).
  So §9.5's "horizontal unchanged" stands, and the 456 gain was the 20 Hz tick-selection
  effect or plain spread. Claim nothing horizontal.
- **One flight per B arm got worse, and it is the SAME flight index both times** (#4,
  left-to-right; 0.567→0.494 on 456, 0.486→0.877 on 789), each carrying the scorer's
  `yaw-check` flag (25.9°, 29.8°). No A-arm flight carries that flag. Two of sixteen is not
  a pattern yet, but it is arm-specific, so it is OPEN, not noise: check whether the lower
  takeoff (0.29 m + ground effect, §9.7) disturbs the standby yaw on that geometry.
- Still ~0.19 m high at closest approach in arm B — the dash climb (§9.13). Next lever.

## 10. The dash climb — analysis and pre-registration (2026-09-16, head session)

### 10.1 What the 32 flights show (0 dropped; per direction n = 4 per arm-seed, 16 per direction overall)

Plain English: the sprint lasts only about **1.45 s**, and in that time the vehicle rises about
**0.35 m**. The height-hold notices, but it is far too gentle to do anything about it in a
second and a half.

| | left-to-right | right-to-left |
|---|---|---|
| sprint duration (sim time) | 1.42–1.48 s | 1.47–1.52 s |
| height vs reference at sprint start (own estimate) | −0.06 to −0.10 m | same |
| height vs reference at closest approach — own estimate | **+0.23 to +0.28 m** | **+0.11 to +0.17 m** |
| … TRUE (world z, corrected for the camera dipping 0.120·sin(pitch)) | +0.29 to +0.34 m | +0.19 to +0.22 m |
| push-down command at closest approach | 0.23–0.28 m/s | 0.11–0.16 m/s |
| … largest at any point | 0.29–0.35 m/s | 0.22–0.27 m/s |
| pitch at closest approach | −43° to −44° | −41° to −43° |

Medians per arm-seed-direction cell; all four arms agree, so the height-reference offset did not
change the climb. Three readings:
1. **The loop is barely trying, not failing.** Its clamp is 0.5 m/s and it never commands more
   than 0.35. With `KP_ALT = 1.0 /s` the loop's time constant is ~1 s against a 1.45 s sprint. It
   is a gain problem, not an authority problem.
2. **The vehicle's own height estimate lags the truth by 5–8 cm during the sprint** (estimate
   +0.11…+0.28 vs true +0.19…+0.34). No outer-loop gain can remove what the estimate cannot see.
3. **Left-to-right climbs ~0.10 m more than right-to-left**, with ~2° more nose-down pitch.
   Not explained. The +5° aim trim is one-signed, so the two directions do fly different turns.
   It also starts the sprint 6–10 cm BELOW the reference — the takeoff gate releases at 80% of
   the target height — which currently hides part of the climb.

### 10.2 The lever, and why this one first

`--dash-alt-kp 4 --dash-alt-vmax 1.5` — a stiffer height-hold during the sprint only (built
today, default OFF, identity proven by test). Chosen over a constant push-down feed-forward
because a feed-forward is a number tuned on this sim's thrust curve and would not transfer,
whereas a gain acts on the vehicle's own measured height and needs no calibration. Honesty
grade: **own-state only** — it reads the altitude estimate and nothing about the target.

### 10.3 Configuration

Control = the adopted `AE5dashZ`. Arm `AE5dashZK` = the same plus the two flags. **Seed 321**
(disjoint from 123/777/456/789), n = 8 paired, both directions, dash-only, sequential, idle load.

### 10.4 Prediction (written before flying)

A disturbance that produces ~0.24 m/s of climb against a gain of 4 /s settles near
0.24/4 ≈ 0.06 m of *estimated* error; add the 5–8 cm estimator lag → **true height at closest
approach ≈ +0.10 to +0.15 m above the reference**, down from +0.19…+0.34 m. Median vertical
offset at closest approach (lens, logged tick) falls from ≈0.19 m to ≈0.08 m. Horizontal
unchanged. The left/right asymmetry should shrink with it.

### 10.5 Adopt / reject

**ADOPT** if the true height error at closest approach is smaller in ≥6/8 pairs AND its median
falls by ≥0.05 m AND the median horizontal miss is not worse by more than 0.05 m.
**REJECT** if ≤4/8, or horizontal worsens by >0.05 m, or any flight shows a vertical
oscillation (sign change of the height error more than twice inside the sprint) — a stiff outer
gain over PX4's own velocity loop can ring, and a ringing vehicle is worse than a high one.
Anything between is UNDERPOWERED — say so, fly a second seed, do not pick a side.

### 10.6 What a NULL would mean

The outer gain is not the limit. The climb is then being set by PX4's inner vertical loop and/or
the estimator lag — i.e. by things a setpoint cannot fix in 1.5 s — and the honest options become
a feed-forward (tuned, non-transferable, to be labelled as such) or accepting ~0.2 m of vertical
error as the floor of a preset-height sprint, which would make ADR-0085's camera-driven vertical
channel the only real fix.

### 10.7 Arm asymmetries and regime-dependent constants

- Only arm B can ring (10.5 guards it). Only arm B can command >0.5 m/s downward at 0.29 m
  above the ground at sprint start: check the minimum TRUE height of every B flight; any flight
  below 0.10 m is a ground-strike risk and is reported, not averaged.
- `KP_ALT = 1.0` and `V_VERT_MAX = 0.5` were tuned for hover-and-approach at walking speeds; this
  changes them only inside CODED_DASH. ACQUIRE/ENGAGE keep the stock values.
- Both arms are dash-only, so the breakoff defect reaches neither.

### 10.8 RESULT, seed 321 (2026-09-17) — criterion MET; replication on seed 654 launched under the same criterion

| | A: adopted (`AE5dashZ`) | B: + `--dash-alt-kp 4 --dash-alt-vmax 1.5` |
|---|---|---|
| TRUE height error at closest approach, median | **+0.271 m** | **−0.031 m** |
| pairs where B's height error is smaller | — | **8/8** |
| median horizontal (lens, logged tick) | 0.276 m | 0.261 m |
| primary ruler (centre, interpolated): inside 0.35 m | 6/8 | 7/8 |
| … median closest approach | 0.273 m | **0.192 m** |
| height-error sign changes inside the sprint (ringing guard: >2) | — | 1–2 on every flight |
| lowest TRUE height of any B flight (guard: <0.10 m) | — | 0.21 m |
| largest push-down/up command | ≤0.35 m/s | 0.79–1.05 m/s |

All three ADOPT conditions met. **Better than predicted** (+0.10…+0.15 m predicted, −0.03 m
got), and the reason is partly luck, so it is written down: in arm B the vehicle's own estimate
reads **−0.137 m** at closest approach while the truth is −0.031 m. The estimator's lag (~0.10 m
here, larger than the 5–8 cm seen in arm A) happens to cancel most of what the stiffer loop
over-corrects. **The loop is steering the estimate, not the truth; landing near zero is not a
designed property** and should not be expected to hold at another speed, pitch or sprint length.

With the vertical error gone, **what is left is horizontal**, and it is scatter, not bias:
per-flight 0.009–0.49 m in arm B. That is the launch-aim term, and it is the one the camera
terminal is supposed to earn its place on.

### 10.9 REPLICATION, seed 654 (2026-09-17) — vertical confirmed, but the HORIZONTAL clause REJECTS. NOT ADOPTED.

| seed | true height error A → B | pairs B smaller | median horizontal A → B | inside 0.35 m (primary ruler) A → B |
|---|---|---|---|---|
| 321 | +0.271 → −0.031 m | 8/8 | 0.276 → 0.261 m | 6/8 → 7/8 |
| 654 | +0.286 → −0.053 m | 8/8 | 0.275 → **0.347 m** | 6/8 → **4/8** |

The vertical effect is as solid as anything this project has measured: 16/16 pairs, both seeds
within 2 cm of each other. But §10.5 says REJECT if the median horizontal worsens by more than
0.05 m, and on seed 654 it worsened by 0.072 m and cost two contacts. One seed adopts, one
rejects → **not adopted**. The criterion was written to stop exactly this trade being waved
through.

**Why it happens (mechanism evidence, pooled over both seeds, n = 8 per cell):**

| | pitch at closest approach | speed at closest approach | cross-range error (left-to-right flights) |
|---|---|---|---|
| A | −43° | 11.5 m/s | −0.06 m |
| B | **−36°** | 11.1 m/s | **+0.29 m** |

Pushing down harder changes the sprint itself: the vehicle flies ~7° less nose-down and a
little slower, so on left-to-right flights it stops ~0.3 m short of the target's track.
Right-to-left flights are unaffected (+0.08 → −0.04 m). Pooled horizontal medians: left-to-right
0.27 → 0.41 m, right-to-left 0.29 → 0.25 m. The launch aim was solved for the OLD sprint
profile; change the sprint and the aim no longer fits. This is the same coupling the project
already knows from the accel-cap arms ("the aim must be co-sized with the dash").

**What this means, in order:**
1. The vertical and horizontal problems are **not independent** on this vehicle — a fix to one
   moves the other. Any future vertical lever must be flown with the aim re-derived for the
   sprint it produces, and judged on the 3-D result.
2. Next arm (not yet registered): `AE5dashZK` with the lead re-solved for the measured B
   profile. Until then the adopted config stays `AE5dashZ` (ADR-0100).
3. The left/right asymmetry keeps showing up in every table today (climb, pitch, cross-range).
   It deserves its own look before more levers are stacked on it.
