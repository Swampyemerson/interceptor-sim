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
