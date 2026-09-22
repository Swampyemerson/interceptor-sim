# Cue relay plan — getting ~2 s of target GPS from the target to the interceptor

> **Status: DESIGN + BUILDABLE CODE (2026-09-22), not flown.** This is the
> piping for the launch-aim cue that `docs/next.md` already RULED on
> ("real interceptor launches off a GPS-derived cue **latched at trigger,
> link then dies**" — the "Launch-aim cue" row, 2026-07-26). That ruling said
> *what* the cue is; this doc says *how it physically gets from the target's
> GPS to the interceptor's brain*, and *how good it can possibly be*. Code:
> `flight/cue_relay.py`. Tests: `flight/tests/test_cue_relay.py` (18/18
> passing, incl. one against a REAL bench GPS log — see §5).

## 0. Plain-English summary (read this if you only read one section)

The interceptor needs to know roughly where the target is and which way it's
going **before it launches** — that's its "radar cue." We can't give it a
live radar; what we *do* have is the target's own GPS, which it already
broadcasts over its radio link (MAVLink telemetry, the same protocol
language PX4/ArduPilot both speak). So: **watch the target's GPS for about 2
seconds on a laptop at the field, do a little math to turn that into "the
target is X metres that-a-way, moving at Y m/s," and hand that one number to
the interceptor's Raspberry Pi before it takes off.** After that, the link
dies (by design — that's the whole "jam-resistant" point of this project) and
the interceptor is on its own.

**The punchline, worked out below:** a real GPS cue is noisy enough (several
metres of position error, roughly a mile-per-hour of velocity error) that it
is **nowhere near good enough** for the *old* plan (a single open-loop dash
that has to be aimed within about 1 degree). It is, however, **easily good
enough** for the *new* plan (`pursuit_terminal.py`, ADR-0103) — the "chase"
terminal that flies toward the target's rough area and then lets the camera
do the precise work once it's close enough to see it. **So: build this relay,
but only for the pursuit terminal.**

## 1. Hardware correction (read before anything else here)

This task's brief said the target FC is a SpeedyBee F405. **That's stale.**
The SpeedyBee path died in July (F405 V4 discontinued, V5 unobtainable —
`docs/hardware_order_list.md` §0b) and the ordered, currently-flying board is
different:

| | what's actually flying |
|---|---|
| Target FC | **Holybro Kakute H7 v1.5**, ArduCopter **V4.7.0** (`1511f271`) |
| Target GPS | **Matek M10Q-5883** (u-blox M10 chipset), soldered to UART3 |
| Bench-verified | 2026-09-17, `runs/tgt02_gps/summary.json`: live 3D fix, **11 satellites, HDOP 0.99**, 97 `GPS_RAW_INT` messages in 25 s over USB |
| Real log on file | `runs/tgt02_gps/00000304.BIN` — a real ArduPilot dataflash log, downloaded over MAVLink (`runs/tgt02_gps/pull_newest.py`), 27 position samples, GPS-UTC-synced |

The design below is unaffected (both boards are ArduPilot, both talk MAVLink
`GLOBAL_POSITION_INT`/`GPS_RAW_INT`/`GPA` over their telemetry link — the
schema and fitter don't care which FC silicon is underneath), but every
number quoted from real hardware in §5 comes from **this** board, not the
brief's.

**Bonus fact this bench session hands us for free:** `pull_newest.py` proves
the **MAVLink software path from the target FC to a laptop already works**,
right now, on real hardware — over USB today, but the protocol plumbing
(`pymavlink`, `GLOBAL_POSITION_INT`/`GPA` parsing) is identical whether the
bytes arrive over a USB cable or a radio link. The only engineering gap left
for the field is the **RF hop** (see §2), not the MAVLink/software side.

## 2. The three relay paths, compared

**Leg 1 (target → ground laptop)** and **Leg 2 (ground laptop → the
interceptor's Pi)** are separate hops; a real relay chains one of each.

| # | Path | Leg 1 (target→ground) | Leg 2 (ground→Pi) | New hardware? | Verdict |
|---|---|---|---|---|---|
| **(a)** | **MAVLink telemetry → laptop → Pi over field WiFi** | Target FC's own MAVLink stream, read by `mavproxy`/`pymavlink` on a ground laptop | `flight/cue_relay.write_cue_jsonl` → push the small JSON-lines file to the Pi over the field WiFi it already runs pre-launch (hostname `interceptor-seeker`, memory `pi-access-and-sync`) | Leg 2: **none** (reuses infrastructure already relied on for dev/SSH). Leg 1: **not yet provisioned** — see below. | **RECOMMENDED** |
| **(b)** | **Laptop → SiK uplink → Pi** | same as (a) | push the cue over the interceptor's own SiK radio pair | The ordered SiK pair (`docs/hardware_order_list.md` (D), $75) is wired **FC TELEM1 ↔ ground USB**, i.e. it lands on the interceptor's **flight controller**, not the Pi (the Pi's dedicated UART is TELEM2). Getting cue bytes onto the Pi over SiK needs either a second serial port the 6C Mini doesn't have spare (GPS1/TELEM1/TELEM2 are already spoken for, only GPS2 is free — `hardware_order_list.md` (B)) or relying on PX4's MAVLink router to forward a message from TELEM1 to TELEM2, which is untested and adds FC-side configuration risk for a link that's also supposed to stay "monitoring only" (§0⑤, `no-datalink`). | **REJECTED for this purpose** — architecturally awkward, and SiK is currently DEFERRED hardware anyway (`hardware_order_list.md` §0c). |
| **(c)** | **Manual entry fallback** | operator reads the target's *programmed* AUTO-leg waypoints (we wrote the mission, so we know it) | operator types `--target-start`/`--target-vel` on the Pi's CLI | **$0 — already built.** This is `real_flight.py`'s existing default behaviour (`--target-start "0,16.7" --target-vel "9,0"`). | **Keep as the fallback, always.** See the honesty flag below — it is not actually free of the problem this doc exists to fix. |

**Leg 1's real gap:** nothing in the ordered BOM currently gets MAVLink
telemetry OFF the target aircraft wirelessly. Its ELRS link (RadioMaster
Pocket M2 TX ↔ RP1 RX) is control-only in the current plan. Two ways to close
this, cheapest first:
1. **ELRS telemetry backpack (WiFi)** — modern ExpressLRS firmware can tunnel
   MAVLink over the existing control link and rebroadcast it over the TX's
   WiFi backpack to a laptop. **$0, but unverified on this hardware/firmware
   version — bench it before relying on it.**
2. **A small dedicated telemetry radio on the target** (a second cheap
   SiK-class or similar UART radio, ~$20–40, wired to the Kakute's spare
   UART) if (1) doesn't pan out. Guaranteed to work; small added cost/weight
   the target can absorb (it's the expendable side).

**Failure modes of the recommended path (a):**
- **Leg 1 not yet working** (see above) — the single open item standing
  between "designed" and "flyable." Bench it next.
- **Field WiFi range/dropout.** The project's own experience with the Pi's
  WiFi is that it drops SSH and needs a retry (memory `pi-access-and-sync`)
  — treat the relay as **best-effort and gate on the quality check** (§4),
  never assume the stream arrived.
- **Clock skew across three machines** (target FC, laptop, Pi). Sidestepped
  by design: `CueRecord.t` is defined as **receive time on the last hop**
  (the Pi's own clock), not the target FC's onboard timestamp — see
  `flight/cue_relay.py`'s module docstring. This folds any transit latency
  into the "relay latency" budget line (§6) instead of a clock-sync bug.
- **The relay hardware fails outright at the field.** Falls back to (c) —
  which is why (c) staying built and rehearsed matters, not just (a).

## 3. Honesty framing — why this fixes a real loophole, not a nice-to-have

Before this module, the *only* way to fill `--target-start`/`--target-vel`
was to type in the AUTO-leg waypoints we ourselves programmed — a
**GIVEN-PERFECT** input (CLAUDE.md's honesty rule: *"a number computed on a
`given-perfect` input is a BEST-CASE UPPER BOUND, never the claim"*). That's
structurally the SAME mirage the ledger already caught once
(`launch-aim-derived-from-ground-truth`): the launch aim is honest only if the
real flight tracks its plan exactly, which no real flight does (wind, GPS
drift in the target's own nav, mission timing).

**A relayed GPS cue is the fix, precisely because it is worse.** It reports
where the target's GPS *actually* says it is, with GPS's actual error baked
in. That is why it is graded **`given-noisy`**, not `given-perfect`, in the
assumptions register below — and per CLAUDE.md, *"an external cue is
architectural and fine... a cue with ZERO error is not, because no real cue
has zero error."*

**Assumptions register entry:**

| input | source | grade | why |
|---|---|---|---|
| target position/velocity at trigger | `flight.cue_relay.fit_cue` on ~2 s of the target's own GPS, relayed pre-flight | **given-noisy** | carries the target GPS's real, measured error (§6) — not zero, not assumed away |
| interceptor's own launch position | its own GPS fix, read before GO | **given-noisy** (not separately budgeted here — same receiver class, same order of error; treat the whole cue chain's error as dominated by the TARGET leg, since both ends share similar GPS hardware) | pre-flight, read once |
| manual-entry fallback (c) | operator-typed programmed AUTO-leg waypoints | **given-perfect** (disclosed) | only as accurate as the real flight tracks its own plan — the SAME loophole this module exists to close; use only when the relay fails, and disclose it as the weaker fallback it is |

## 4. The module: `flight/cue_relay.py`

Pure stdlib + numpy; `pymavlink` is optional and imported only inside
`mavlink_stream_to_cue_records` (confirmed the rest of the module needs
nothing extra — 18/18 tests pass with zero radio, zero ArduPilot, zero sim).

**JSON-lines schema** (one object per line):
```json
{"t": 1234.56, "lat": 39.99191, "lon": -105.24535, "alt_m_msl": 1648.9,
 "vn": 8.7, "ve": 0.3, "vd": -0.1}
```
`t` = seconds on the RECEIVING end's own clock (recommended: the Pi's
monotonic receive-time — see §2's clock-skew note). `vn`/`ve`/`vd` are
optional (the FC's own reported Doppler/EKF ground velocity, NED, m/s).

**`fit_cue(records, own_lat, own_lon, own_alt_m_msl, ...) -> CueSolution`**
turns records + the interceptor's own pre-launch GPS fix into
`belief_r0_ned`/`belief_vel0_ned` — an ordinary least-squares straight-line
fit of each record's local-tangent-plane NED position against time (see
`lla_to_ned`), anchored so the intercept term IS the position at the last
(latest) sample — i.e. **the fit itself averages down single-fix noise**,
it doesn't just report the last raw fix. If every record in the window
carries a reported `vn/ve/vd`, those are averaged and preferred
(`vel_source="reported"`); otherwise the OLS slope is used
(`vel_source="fit"`) — **never silently mixed per-axis.**

**Quality gate — FAILS CLOSED** (`CueQualityError`, never a degraded
default): too few points (`min_points`, default 8), too short a time span to
fit a velocity at all (`min_span_s`, default 1.0 s), stale data at emission
(`max_staleness_s`, default 1.0 s, only checked if the caller passes `now_t`),
a non-converging/degenerate fit (duplicate timestamps → rank-deficient), or a
position-fit residual too large to trust (`max_residual_m`, default 5 m —
catches a bad fix, multipath, or a target that maneuvered mid-window instead
of flying straight).

**`CueSolution.extrapolate_to(trigger_t)`** — the *one* legal post-latch
operation: advances the belief position by the already-latched constant
velocity to the real trigger instant. Pure arithmetic on frozen numbers; does
not re-read anything.

**Integration loader** (no edit to `real_flight.py` — see §8 for the
proposed hookup): `cue_to_target_start_arg`/`cue_to_target_vel_arg` format a
`CueSolution` into the exact `"east,north"` strings
`build_config`/`build_terminal` already parse; `load_cue_for_real_flight`
chains read → fit → (optional) extrapolate → those two strings in one call.

## 5. Real hardware data used (not simulated)

`runs/tgt02_gps/00000304.BIN` — the real dataflash log from §1 — was read
through the project's **existing** `.BIN` consumer,
`scripts.field_score.load_track_from_bin` (the `tgt-02` gate), and fed
through `flight.cue_relay.fit_cue` as a **producer→consumer contract test**
(`flight/tests/test_cue_relay.py::test_contract_real_ardupilot_bench_log`) —
not a hand-typed fixture. Measured, reproducible (`.venv/bin/python -m
pytest flight/tests/test_cue_relay.py -k bench_log -q -s`):

| quantity | value | source |
|---|---|---|
| samples / span | 27 POS rows / 2.60 s (vehicle STATIONARY on the bench) | `.BIN`, `dataflash:POS` |
| GPA `HAcc` (receiver's own reported 1σ-class horizontal accuracy) | **2.29 m** (2.26–2.30 across 13 samples) | `.BIN` `GPA` rows |
| GPA `VAcc` | 2.84 m | `.BIN` `GPA` rows |
| GPA `SAcc` (receiver's own reported speed-accuracy estimate) | **1.72 m/s** | `.BIN` `GPA` rows |
| Raw single-epoch Doppler `GPS.Spd` on a STATIONARY vehicle | mean 0.62, std 0.48, max 1.72 m/s | `.BIN` `GPS` rows (true value = 0, so this IS the noise) |
| `fit_cue`'s OLS position-fit "speed" on the SAME stationary window (27 pts/2.6 s) | **0.255 m/s** | `flight.cue_relay.fit_cue`, `vel_source="fit"` |
| `fit_cue`'s position-fit residual RMS | 0.265 m | same run |

**Takeaway, stated plainly:** the receiver's own conservative accuracy
estimate (`HAcc`≈2.3 m, `SAcc`≈1.7 m/s) is markedly worse than what the
position-fit actually achieved on this bench window (0.27 m / 0.26 m/s) by
averaging 27 samples — a real, measured illustration of why `fit_cue`
regresses over the whole window instead of trusting one fix. **Caveat,
disclosed:** this is a single 2.6 s STATIC capture, not a moving-target
validation — real motion changes multipath geometry and EKF innovation
dynamics. `TODO-BUILDER`: repeat this capture with the target flying a
straight AUTO leg once airborne, and feed that log through the same test.

## 6. Error budget — converted to launch-aim degrees

Reusing the geometry `docs/launch_mechanism_plan.md` §3 already validates:
canonical range at launch **R ≈ 16.7 m**, and its CPA-vs-heading-error table
(`dash_cpa_model.py`, validated to 0.29–0.58 m MAE against flown arms).
Converting a position/velocity error to an aim-heading error:
`θ_err ≈ atan(position_error_m / R)`.

**Two error sources, one time lever:**
1. **GPS position error at fit time** — worst-credible = `HAcc` **2.29 m**
   (§5); best-case achieved (this bench) = **0.27 m**.
2. **GPS velocity error, growing with elapsed time `T`** between the cue's
   last sample and the moment it's actually flown (staleness + relay
   latency + however long the operator waits before GO — **all of which
   `extrapolate_to()` cancels EXCEPT the velocity error itself**, so the
   residual growth is `velocity_error × T`). Worst-credible velocity error,
   MC-verified below; best-case = the bench-achieved 0.13 m/s (derived from
   the same OLS formula at the bench's tighter achieved noise).

**MC verification of the velocity-error claim**
(`flight/tests/test_cue_relay.py::test_fit_cue_velocity_accuracy_matches_error_budget_claim`,
400 trials, `sigma_pos=2.29 m` from `HAcc`, N=15 samples over 1.82 s ≈ 7.5–8
Hz — a realistic ~2 s relay window):

```
measured 2D speed-error RMS = 1.450 m/s
analytic OLS prediction     = 1.599 m/s   (Var(v)=12σ²/(N·T²), summed over 2 axes)
```
Measured and derived agree within the pre-registered 1.5× band — the
derivation is trustworthy, not just asserted.

**Combined table** (`position_total = sqrt(pos_err² + (vel_err·T)²)`,
`θ_err = atan(position_total / 16.7)`; `T` = time from last GPS sample to
actual dash/GO, the operator-controlled lever — see §7):

| `T` (elapsed) | WORST (HAcc 2.29 m, vel 1.60 m/s) | BEST achieved (0.27 m, 0.13 m/s) |
|---|---|---|
| 0.5 s (re-latch right at GO) | pos 2.43 m → **8.3°** | pos 0.28 m → **1.0°** |
| 2 s (typical climb, re-fit once at top) | pos 3.93 m → **13.2°** | pos 0.38 m → **1.3°** |
| 5 s (slow/cautious climb, no re-fit) | pos 8.32 m → **26.5°** | pos 0.70 m → **2.4°** |

## 7. Which terminal a GPS-grade cue can serve — stated plainly

| terminal | tolerance | can a GPS relay cue serve it? |
|---|---|---|
| **stock/sprint** (open-loop dash to contact) | **~±1°** for the sprint acquisition window (`docs/next.md`: ~0.1 m miss per degree, 0/8 inside at 5°); **±2.4°** for a 0.35 m ram (`launch_mechanism_plan.md` §3.1) | **NO — not even in the BEST-achieved case at the fastest re-latch (1.0° vs a ±1° window has zero margin), and the WORST-credible case (8.3–26.5°) is 3–10× over budget.** CLAUDE.md's rule is decisions must survive WORST, not best; this fails WORST badly at every `T`. |
| **pursuit / "chase only"** (ADR-0103/0105) | **91–96% inside 0.35 m at 10–30° aim error**, tag-facing; **60% at 20°** rear-tag | **YES, comfortably, if `T` is kept short.** WORST-credible at `T≤2 s` (8.3–13.2°) sits inside the well-populated part of the tag-facing curve. At `T=5 s` (26.5°) it's beyond the 20° rear-tag measurement and near the edge of the facing curve's 30° rung — **keep `T` under ~2 s (§7 below) or budget for the rear-tag degradation.** |

**Bottom line: build and fly this relay only against `--terminal pursuit`.**
It was never going to serve the sprint terminal (that terminal's whole
weakness — ADR-0103's own numbers — is that it needs aim this tight from
ANY pre-flight source, not just a GPS one), and pursuit's camera-corrected
Phase B is exactly the kind of terminal a somewhat-noisy pre-flight belief is
supposed to feed: the cue only has to get the vehicle to the right
neighbourhood and pointed roughly the right way for the camera to acquire —
final accuracy comes from the camera, already measured separately.

## 8. Launch profile

**Standby altitude (`C3`, `MissionConfig.standby_alt_m`)** — unchanged
principle from `launch_mechanism_plan.md` §1: `standby_alt_m` = target
altitude (now read from the cue's `belief_r0_ned` down-component, i.e.
`cfg.dash_loft_m` above whatever altitude the cue believes the target holds)
+ the loft height. With the cue relay, this is no longer a hand-typed number
— `build_terminal()` already derives the pursuit belief's down-component as
`+dash_loft_m` relative to `standby_alt_m` (existing code, unchanged; see
`real_flight.py` `build_terminal` around the belief-seed comment).

**Climb-rate vs. cue staleness — the real lever is `T`, not the climb rate
itself.** Every second between "the cue's last GPS sample" and "the vehicle
actually flies on it" ages the belief by `velocity_error × that_many_seconds`
(§6) — **not** by the full target speed, because `extrapolate_to()` already
removes the target's believed constant-velocity motion; what's left is only
the ERROR in that velocity estimate. So the climb itself can be a normal
1–2 m/s rate; the important discipline is:

- **Keep the cue stream running continuously through STANDBY**, not just
  once at power-on. `fit_cue` is cheap (pure numpy, sub-millisecond) — re-run
  it on the latest ~2 s rolling window every time a new record arrives.
- **Take the FRESHEST `CueSolution` at the GO edge, not the one from before
  the climb.** This is what keeps `T` near the 0.5 s row of §6's table
  instead of the 5 s row — a >3× difference in worst-case aim error.
- If the relay stream dies mid-standby, the LAST good `CueSolution` is still
  usable but its effective `T` (and therefore its error) grows every second
  it goes un-refreshed — the quality gate's `max_staleness_s` should be set
  to whatever the operator is willing to fly on, then enforced, not left at
  the library default without a decision.

**Recommended field sequence:**
1. **Power on** both aircraft; confirm the target's GPS has a 3D fix (mirror
   the `runs/tgt02_gps` bench check: `NSats`≥6, `HDOP`<2).
2. **Start the cue stream** (leg 1+2, §2) — wait for the Pi to report a
   PASSING `fit_cue` (≥8 points, ≥1 s span, low residual) before proceeding;
   this can happen with both aircraft still on the ground.
3. **Climb to standby** (`C3`) — cue stream keeps running and re-fitting the
   whole time.
4. **Hold at standby**, yawed to C1 — take the freshest `CueSolution` each
   loop; this is the state the vehicle sits in until the operator commits.
5. **Latch** — at (or just before) the GO edge, extrapolate the freshest
   solution to the actual trigger instant (`extrapolate_to`); this is the
   number that gets frozen.
6. **GO.** No further reads of the cue stream after this point (the existing
   latch-once honesty audit, `launch_mechanism_plan.md` §5, must be extended
   to cover this new variable the same way it covers the bearing latch).

## 9. `real_flight.py` hookup — APPLIED 2026-09-22 (head), tests in `flight/tests/test_cue_relay.py`

> Applied with one change from the proposal below: **no `now_t` staleness check on the Pi** —
> record timestamps come from the GROUND producer's clock, and comparing them against the Pi's
> clock is a cross-clock-base fault (the `TriggerState.clock_fault` class). Staleness is enforced
> ground-side, where the clock matches the records (§8's T budget). Original proposal kept for the record:

```diff
--- a/flight/deploy/real_flight.py
+++ b/flight/deploy/real_flight.py
@@ aim = ap.add_argument_group("pre-flight aim constants (C1/C2/C3)")
+    aim.add_argument("--cue-json", default=None,
+                     help="flight.cue_relay JSON-lines cue file "
+                          "(docs/cue_relay_plan.md) -- overrides "
+                          "--target-start/--target-vel with a GPS-fitted "
+                          "belief. Requires --own-lat/--own-lon/--own-alt-m-msl.")
+    aim.add_argument("--own-lat", type=float, default=None)
+    aim.add_argument("--own-lon", type=float, default=None)
+    aim.add_argument("--own-alt-m-msl", type=float, default=None)
@@ def build_config(args) -> MissionConfig:
     """Resolve the pre-flight constants ONCE, before anything flies."""
+    if args.cue_json:
+        from flight.cue_relay import load_cue_for_real_flight, CueQualityError
+        try:
+            args.target_start, args.target_vel, cue_sol = load_cue_for_real_flight(
+                args.cue_json, args.own_lat, args.own_lon, args.own_alt_m_msl)
+        except CueQualityError as e:
+            raise SystemExit(f"[cue] REFUSED (fail-closed): {e} -- retype "
+                             f"--target-start/--target-vel by hand instead")
+        print(f"[cue] GPS-fitted belief: target-start={args.target_start} "
+              f"target-vel={args.target_vel} (n={cue_sol.n_points} pts, "
+              f"residual={cue_sol.residual_rms_m:.2f} m, "
+              f"source={cue_sol.vel_source})")
     heading = args.dash_heading_deg
```
Read-once discipline: `build_config` runs exactly once, before the state
machine starts (same place `heading`/`t_lead` are resolved today) — nothing
downstream re-reads `args.cue_json` or re-imports `cue_relay`, so the
existing "no post-latch cue read" audit extends to this input by the same
mechanism that already covers the bearing latch (one call site, checked once).

## 10. Tests

`.venv/bin/python -m pytest flight/tests/test_cue_relay.py -q` → **18
passed** (incl. the real-hardware contract test against
`runs/tgt02_gps/00000304.BIN`, skipped only if that file is absent from the
checkout). Full flight suite: see the handback report for the combined run.

## 11. Open risks / TODO-BUILDER

- **Leg 1 (target→ground) is the one real gap** — bench the ELRS WiFi
  backpack telemetry passthrough first ($0); fall back to a small dedicated
  telemetry radio on the target (~$20–40) if it doesn't pan out.
- **§5's numbers are a STATIC bench sample** — re-run the same contract test
  against a moving-target `.BIN` once one exists; the position/velocity
  numbers here are a noise floor, not a full characterization.
- **End-to-end relay latency is not yet bench-measured** — `TODO-BUILDER`:
  time power-on-stream to Pi-receipt on the actual field WiFi, feed the
  measured number back into §6/§8 in place of the placeholder discussion.
- **The rear-tag 20°/60% cell (§7) is the weak point** — if `T` creeps above
  ~2 s in practice, budget for that degradation explicitly rather than
  assuming the facing-tag numbers.
- **The latch-once honesty audit needs extending** to the new `--cue-json`
  variable (§9) before this is ever flown for real — not done here (no code
  edit to the audited module in this task).
- **Manual-entry fallback (c) is the honesty-weaker path** — fine as an
  emergency fallback, but never report a launch aim as `given-noisy` if it
  actually came from (c); log which path produced each flight's aim.
- **THE VERTICAL CHANNEL IS THE UNPRICED HALF (head review, 2026-09-22).**
  §6 converts HORIZONTAL error to aim degrees; the cue's DOWN component
  (target altitude) gets no equivalent treatment, and it is not symmetric
  with it: the measured GPA `VAcc` here is **2.84 m** (§5, worse than the
  2.29 m HAcc), while the pursuit altitude-error sweep
  (`isim/specs/alt_sensitivity_2026-09-21.md`, commit `8940a6b`) found
  chase-only is flat within noise from −2 m to +2 m of altitude error but
  **cliffs at +3 m ABOVE the target (74–88% → 42% touching)**. A one-sigma
  GPS altitude error therefore sits AT the measured cliff on the high side.
  Two consequences before this flies: (1) bias the derived standby/belief
  altitude LOW, not centred — the tolerance band is asymmetric and the cheap
  side is below; (2) prefer a non-GPS vertical source for the target when
  one exists (briefed AUTO-leg altitude, or the target's baro via the same
  telemetry stream) and grade whichever is used in the assumptions register.
  An isim arm injecting the measured 2.84 m vertical error into the pursuit
  belief is the cheap pre-registered test that settles it.

---
*Provenance: `docs/launch_mechanism_plan.md` §§1-3 (launch geometry, aim-error
budget, honesty stack), `docs/launch_mechanism_options.md` (relay-path
tradeoffs framework), ADR-0103/ADR-0105 (`docs/decisions.md`, the pursuit
terminal and its measured tolerance), `docs/next.md` ("Launch-aim cue" ruling),
`docs/hardware_order_list.md` (BOM facts, §1's correction),
`runs/tgt02_gps/summary.json` + `00000304.BIN` (real bench GPS data),
`flight/pursuit_terminal.py` (belief_r0_ned/belief_vel0_ned contract),
`flight/deploy/real_flight.py` (`resolve_preflight_heading`, `MissionConfig`,
`build_config`/`build_terminal` — read-only in this task), ledger entry
`launch-aim-derived-from-ground-truth` (why a noisy cue is the honest choice).*
