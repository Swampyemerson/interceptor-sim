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

---

## 1. What is already measured, and what is only inferred

**Measured (offline, from committed logs, reproducible today):**

| Quantity | Value | Tool |
|---|---|---|
| Vertical separation at closest approach, cue-era fleet | **+0.485 m** median, off-altitude on **24 of 24** flights | `scripts/forensics/vertical_miss_anatomy.py` |
| Altitude drift across the dash, same fleet | **+0.320 m** median, n=16 | same |
| Drift as a share of the vertical miss | **66%** | same |
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

**H1 (the dash trim).** The vertical error is delivered by the dash, so setting
`dash_alt_trim_m` to `derive_dash_alt_trim_m(measured drift of THIS fleet)` will
reduce the vertical component of closest approach by most of that drift, and will
move the total closest approach by roughly the same amount when vertical is the
dominant term.

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
