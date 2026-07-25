# First-kill day protocol — build_plan P5 (decision-grade)

> **Read this before you leave the house, and again on the tailgate.** This is the
> session that produces the project's *stated deliverable*: a **binary kill** of a
> flying target by a camera-only interceptor. It is also the first time two
> aircraft are deliberately flown into each other, which makes it the highest
> physical-risk day in the whole program. Everything below is written to be
> followed **in order**; the ladder in §4 exists precisely so that the first
> two-aircraft mid-air happens after four cheaper things have already proven the
> stack.
>
> Sources: `docs/project_state.json` (`build_plan` P4/P5, `kill` stage, constraints
> `binary-kill`, `no-datalink`, `target-is-ardupilot`, `prop-clearance`),
> `docs/launch_mechanism_plan.md` §1/§4/§7, `docs/hardware_order_list.md` §0c,
> `flight/deploy/real_flight.py` (the onboard state machine + FAILSAFES 1–8),
> `scripts/field_score.py`, and — for the kill radius — **the ratified ram-radius
> decision in `docs/decisions.md`** (do not re-derive it here; see §8.3).
>
> Sibling document: `docs/tripod_test_protocol.md` (build_plan P2). This one is
> its P5 counterpart and deliberately mirrors its shape.

---

## 0. What this session decides — READ FIRST

**The criterion is BINARY: did the interceptor take the target out — yes or no.**

| The evidence | What it is | Status |
|---|---|---|
| **Seeker video** (interceptor onboard) | The kill from the weapon's eye; also the honest record of what the camera actually had | **Primary** |
| **Phone slow-mo** (ground, third-party view) | The contact itself, at a frame rate that can resolve it | **Primary** |
| **Both aircraft's flight logs** (interceptor PX4 ULog + target ArduPilot `.BIN`) | The kinematic half: range history + CPA, scored by `scripts/field_score.py` | **Supporting** |

**A citable sub-meter CPA number is EXPLICITLY NOT REQUIRED** (constraint
`binary-kill`; the RTK pair was cut for exactly this reason, and consumer GPS
cannot measure sub-metre inter-aircraft separation anyway — the inter-receiver
bias alone is ~1–3 m). So:

- **A kill visible on video with both logs recovered = SUCCESS**, even if
  `field_score.py` returns `verdict_uncertain` on the GPS geometry.
- **A low `field_score` CPA with airframes that visibly missed on video = a MISS.**
  Video adjudicates; the logs quantify. Never the other way round.
- **A kill on video with a MISSING log = a kill, recorded with an asterisk** — fly
  it again for the clean evidence set, do not retro-fit the missing half.

**What this day does NOT decide:** the markerless/NN question (first kills fly the
AprilTag on Pi 5 CPU — constraint `pi5-compute`), the Hailo purchase, or any
Pk-vs-radius curve (that is the SIM metric, ADR-0025; the field metric is contact).

---

## 1. Pre-session desk checklist (must be GREEN before you drive out)

### 1.1 Bench gates — build_plan P4, all four, no exceptions

- [ ] **PROP-CLEARANCE corner-spin at LOADED throttle, across the FULL tilt range**
      (constraint `prop-clearance`; it is a HARD gate, not a check-if-time). The
      flush mount clears by only ~11° at level and the flown tilt is far past
      that. A clearance FAIL means re-print the mount — **it does not mean fly
      carefully**.
- [ ] **Compute bench**: sustained end-to-end AprilTag Hz on the Pi 5 **through a
      thermal soak**. This number is `stream_fps` everywhere downstream.
- [ ] **Camera on the final boom**: calibration re-checked after mounting, and
      **frame-arrival rate logged SEPARATELY from detection rate** (a vibrating
      CSI ribbon drops frames in a pattern that mimics detection dropouts —
      constraint `forward-camera-boom` item 5).
- [ ] Prop-in-frame + bench hard-negative footage grabbed (free training data).

### 1.2 The onboard state machine — its own gates, run today, output pasted in the log

```bash
.venv/bin/python -m flight.deploy.real_flight --audit        # honesty audit
.venv/bin/python -m flight.deploy.real_flight --self-test    # pure-logic failsafes
scripts/check_real_flight_sitl.sh                            # full SITL mission
```

All three exit 0, or you do not fly. The SITL run also prints
`max flight_mode sample age observed: …s` — **read it and compare it to
`--offboard-stale-s`** (§5.3, FAILSAFE 7 arm 2).

### 1.3 The TODO-BUILDER constants — the ones that are still guesses

`flight/deploy/real_flight.py` carries ~20 constants marked **TODO-BUILDER**: they
are sim-derived or assumed, not measured on this airframe. The ones that decide
whether a props-on flight is safe rather than merely optimal:

| Constant | What it must come from | Bench step |
|---|---|---|
| `standby_alt_m` (C3) | target's briefed AGL + the loft height | pre-flight arithmetic |
| `dash_speed_ms`, `dash_accel_ms2` | the **first real dash ULog**, not the sim fit | L9 |
| `dash_max_s`, `dash_max_dist_m` | sized to the ABORT VOLUME in §5.1, not to taste | L9 + §5.1 |
| `link_timeout_s`, RC channel index, `rc_go_us`/`rc_low_us` | measured ELRS/PX4 RC map + endpoints | L3/L6 |
| `offboard_lost_s`, `offboard_stale_s` (FAILSAFE 7) | the measured Pi↔TELEM2 flight-mode cadence | L3/L6 |
| `target_span_m` | the deployed detector's range calibration | L4/L5 |
| `breakoff_arm_range_m`, `breakoff_min_rise_m`, `breakoff_hard_floor_m` | measured detector range noise | L5 |

- [ ] Every constant in that table either **measured and written into the flight
      card (§11)**, or explicitly accepted as a guess **with the reason written
      down**. A guess you have chosen is fine; a guess you did not notice is not.

### 1.4 Regulatory — a legal blocker, not a data blocker

- [ ] **`docs/regulatory_site_capture.md` is FILLED IN** — regime, site, Remote ID
      path for BOTH >250 g aircraft, and specifically **written permission for a
      deliberate two-aircraft mid-air at this site**. The tripod day's venue may
      not be this day's venue; the tripod day did not involve intentional contact.

### 1.5 Logistics

- [ ] Batteries charged (both aircraft + spares) and the charger present — the
      ladder in §4 is many short flights, not a few long ones.
- [ ] **Spares for the sacrificial parts**: the camera leads the contact, so the
      boom + camera are consumable *per successful kill* (constraint
      `forward-camera-boom` item 4). Arms, props, a spare motor.
- [ ] SD cards in BOTH flight controllers, **verified empty-and-writable today**,
      and logging confirmed ON in both (§6.1 — this is the single most common way
      to lose the evidence half of a successful kill).

---

## 2. What to bring

- Both aircraft, both TXs (or one TX with both models bound — know which model is
  selected before every arm), all packs, charger, LiPo-safe bag.
- **Laptop** — the interceptor's Pi is started over ssh, exactly as on the tripod
  day (`docs/field_bringup.md` §3.0). No laptop → no dash. Phone-ssh backup app
  installed and tested.
- **Phone on a tripod/gimbal for the slow-mo**, and a second person to hold it if
  possible. Frame the *expected contact volume*, not the interceptor.
- Phone hotspot (network + the Pi's NTP source — §6.2 depends on it).
- Tape measure, cones, GPS-marked launch plate position.
- Full safety kit (§10). Fire blanket and a plan for a downed, damaged LiPo in dry
  grass — after a successful kill there are two damaged aircraft on the ground.

---

## 3. Setting up the geometry

### 3.1 The three pre-flight aim constants (C1/C2/C3)

The whole coded-dash architecture is: *the operator never aims the aircraft; the
operator places the launch plate.* (`docs/launch_mechanism_plan.md` §4.) Before
any dash, you compute and write down:

- **C1 — dash heading** (NED compass azimuth). Either operator-entered
  (`--dash-heading-deg`) or solved from the target's programmed AUTO leg
  (`--target-start` / `--target-vel`, accel-aware lead solve, ADR-0080).
- **C2 — lead correction**, only if you fly the optional trigger-instant bearing
  latch. **Default OFF**, and it must stay off until the camera intrinsics and the
  latched-bearing 1σ error are measured (bench L4/L5).
- **C3 — standby altitude** = the target's briefed AGL **plus** the loft height.

Write all three on the flight card (§11) **before** the aircraft is armed. They are
latched at the GO edge and are then immutable for the whole flight (`LatchOnce`;
the honesty audit proves there is exactly one write site).

### 3.2 The target's leg

The target flies a **pre-programmed ArduPilot AUTO leg** — that is what makes the
aim honestly computable ahead of time and what makes runs repeatable. Fly the leg
**once with no interceptor** at the start of every ladder rung to confirm the track
and the speed before anything is aimed at it.

### 3.3 The launch plate

Placed so the dash line is clear of people, the pilot, the camera operator and any
obstacle for the WHOLE dash-max distance (§5.1) — not just to the intercept point.
A dash that fails to acquire flies on until FAILSAFE 3 or 4 stops it.

---

## 4. The kill ladder — do NOT skip rungs

Each rung has its own pre-registered success and abort criteria. **A rung is
passed only when its criterion is met on a flight where nothing was overridden by
hand.** If a rung fails, fix the cause and re-fly THAT rung — never "try the next
one and see".

### L0 — props-off rehearsal (no flight)

Full sequence on the bench with props removed: power up, ssh in, start the state
machine, watch `STANDBY` hold, flip the GO switch, watch `CODED_DASH` command a
setpoint, kill the TX, watch it keep going, then let a failsafe end it.

- **PASS:** every transition appears in the log with a reason, and the mission ends
  in `SAFE` with `land_requested`.
- **ABORT the day if:** the GO switch does not produce exactly one rising edge, or
  the arm gate passes while a precondition is visibly wrong.

### L1 — standby hover only (interceptor alone, no target, no GO)

Take off to C3, hold the aim yaw, land. Measures the standby endurance you are
actually buying (`standby_max_s` is currently a **30–60 s planning guess**, not a
measurement) and confirms the yaw controller holds C1 within the arm gate's ±2°.

- **PASS:** holds altitude ±0.3 m and yaw ±2° for the full planned standby, lands
  clean, ULog written.
- **Record:** measured standby endurance → update `standby_max_s`.

### L2 — dash with NO target (interceptor alone)

GO, full coded dash on C1 into empty air, no acquisition, let **FAILSAFE 3 (dash
timeout)** or **FAILSAFE 4 (distance bound)** end it. This is the flight that
measures the real dash: pull `dash_speed_ms` and `dash_accel_ms2` out of THIS ULog
and put the measured numbers back into the config (build_plan P3 / bench L9).

- **PASS:** the dash flies the commanded heading, the failsafe fires at the
  expected time/distance, `SAFE` is reached, aircraft lands under control.
- **ABORT if:** the aircraft departs the commanded heading by more than the aim
  budget, or the dash does not stop where the failsafe says it should — the abort
  volume in §5.1 is built on that bound being true.

### L3 — static intercept (first two-aircraft engagement)

Target **hovering** at a fixed, briefed point and altitude, tag/placard facing the
dash line. Interceptor: standby → GO → dash → 5-detection handoff → camera-only
terminal → contact.

This is `build_plan` P5 task 1 and it is **the first binary-kill attempt**.

- **PASS (the criterion):** contact visible on video, both logs recovered.
- **Partial:** clean handoff and a terminal that visibly steered, but a miss —
  a valid, loggable result. Re-fly; do not start changing gains at the field.
- **ABORT the rung if:** the handoff never forms (no acquisition in 3 attempts),
  or the terminal steers *away* from the target on video.

### L4 — moving straight-line target, ≥2 m/s

Same as L3 with the target flying its AUTO leg at a low, briefed speed. This is
where the pre-flight lead solve (C1) actually earns its keep.

- **PASS:** contact on video, both logs, at ≥2 m/s.
- **Record:** miss direction (early/late/high/low) from the seeker video — this is
  the aim-error signal that sizes the next rung's C1/C2.

### L5 — the speed ladder to ≥9 m/s

Step the target's leg speed up: **2 → 4 → 6 → 9 m/s**, one step at a time, **at
least two attempts per step** before stepping up.

- **PASS (the project's headline):** a binary kill at **≥9 m/s**, on video, both
  logs recovered.
- **Step-up rule (pre-registered):** advance only after a step yields **either a
  kill, or two consecutive attempts with a clean camera handoff and a terminal
  that visibly closed**. A step that produces neither is a step you re-fly, not a
  step you climb past.
- **Stop-the-ladder rule:** two consecutive attempts at the same speed where the
  handoff never forms → stop, go home, look at the seeker video. That is a
  perception problem, and no amount of field iteration fixes it in daylight.

---

## 5. Safety geometry — the two-aircraft mid-air

> This is the section that is different from every other day in this project. You
> are deliberately colliding two multirotors. Read it out loud in the pre-flight
> brief with everyone present.

### 5.1 The abort volume

Define, **on the ground, with the tape measure and cones, before the first arm**:

- **The dash corridor** — the straight line from the launch plate along C1, out to
  the FULL dash bound (`dash_max_s × dash_speed_ms`, or `dash_max_dist_m` if set).
  Nothing and nobody stands in it. It is longer than the intercept point.
- **The engagement box** — the volume the contact is expected in, plus the
  BREAKOFF climb-out, plus the ballistic fall of two damaged aircraft. Everyone
  stands **outside and behind** it.
- **The PX4 geofence** is sized to that box, and breach → RTL/LAND
  (`docs/launch_mechanism_plan.md` §4). Set it; do not fly on the default.
- **Minimum standoff:** every person (pilot, camera operator, spotter, bystanders)
  is at least the engagement-box radius away, and never downrange.

### 5.2 Authority — who can stop what

| Role | Has | Authority |
|---|---|---|
| **Interceptor pilot** | Interceptor TX: GO switch + **hardware kill switch** | Fires the GO edge; can kill the interceptor at ANY time. The kill switch stays live for the entire flight — killing is not guiding. |
| **Target pilot** | Target TX | Can abort the AUTO leg / RTL the target at any time, including mid-dash |
| **Spotter** | Voice | **Can call an abort that either pilot must execute without discussion.** Assign this explicitly; if the pilots are watching a laptop, nobody is watching the sky. |

Pre-registered abort calls, briefed before the first flight: **"ABORT"** (both
aircraft to a safe state), **"KILL"** (interceptor motors cut, now).

### 5.3 The onboard failsafes — what the aircraft does when you are not fast enough

Every one of these is a guarded, logged transition into `SAFE` (zero horizontal
velocity, land requested). There is no path in the state machine that continues
blindly. Know these before the brief:

| # | Trigger | Result |
|---|---|---|
| 1 | RF link denied **in STANDBY** | SAFE — no dash |
| 2 | Standby battery/time budget exceeded | SAFE |
| 3 | Dash time bound with no acquisition | SAFE |
| 4 | Dead-reckoned / EKF-integrated dash distance bound | SAFE |
| 5 | Terminal lost the target and did not recover | BREAKOFF → SAFE |
| 6 | Terminal time bound | BREAKOFF → SAFE |
| **7** | **OFFBOARD control path lost AFTER the GO edge** (PX4 left OFFBOARD, or the flight-mode sample went stale/impossible) | **SAFE** |
| 8 | Terminal range channel diverged (non-physical range/range-rate) | BREAKOFF → SAFE |

**FAILSAFE 7 deserves the brief line of its own** (builder decision #22): if the
pilot takes manual control mid-engagement — or PX4 drops OFFBOARD for any other
reason — the state machine goes SAFE, and **SAFE is absorbing**. That means when
OFFBOARD is handed back, the vehicle receives a zero-velocity hold and a land
request, **not** a live half-ramped dash command that would fire the instant
control returns. It is safe to take the aircraft back manually. Note the
deliberate asymmetry with FAILSAFE 1: **post-GO RF LINK loss is expected and
ignored** — that is the jam-resistance claim, and §7 tests it on purpose.

**Also note what the failsafes are NOT:** every threshold in that table is a
constant from §1.3. A failsafe with an unmeasured bound is a failsafe you have not
tested — which is why L2 exists.

### 5.4 The link-denied test arm (the jam-resistance deliverable)

On at least one successful engagement, **kill the interceptor's TX immediately
after the GO edge** and let the intercept complete unaided
(`docs/launch_mechanism_plan.md` §4/§7, constraint `no-datalink`, flight arm L11).

- Do this **only after a rung has already produced a kill with the link up** — it
  removes your kill switch, so it is the last thing you do at a rung, never the
  first, and it is briefed as such.
- Recover the kill switch by re-powering the TX; know how long that takes before
  you try it.

---

## 6. What to record — every attempt

### 6.1 The evidence set (all four, or the attempt is unscoreable)

- [ ] **Interceptor ULog** — PX4, on the 6C Mini's SD. *Confirm logging is armed
      and the card is present BEFORE the first flight of the day, and pull the log
      after every attempt, not at the end of the day.*
- [ ] **Target `.BIN`** — ArduPilot DataFlash, on the target FC's SD (constraint
      `target-is-ardupilot`: this is a `.BIN`, not a ULog; `field_score.py` reads
      it with `--bin-b` via pymavlink, or `--csv-b` as the export fallback).
- [ ] **Seeker video** — the interceptor's own camera recording. The one piece of
      evidence that shows what the guidance actually had.
- [ ] **Phone slow-mo** — framed on the expected contact volume, rolling BEFORE
      the GO call.
- [ ] Per-tick mission CSV from the state machine (`--log-csv`, auto-named under
      `logs/`) — carries state, streak, setpoints, guidance telemetry, the
      flight-mode age, and the `safe_reason`.
- [ ] The **flight card** (§11).

### 6.2 CLOCK SYNC between the two aircraft — one per battery segment, non-negotiable

Two independent flight controllers **do not share a boot clock**. `field_score.py`
recovers a common UTC axis when both logs have a GPS time fix (PX4 from
`vehicle_gps_position.time_utc_usec`; ArduPilot from the DataFlash `GPS`
week/ms fields). When a log has no GPS fix the tool falls back to boot-relative
time and flags `utc_synced: false` — and a CPA computed across two unsynced boot
clocks is meaningless.

So, **exactly like the tripod day's §4.6b sync pass**, do all three:

1. **Both aircraft get a GPS 3D fix and hold it for ≥60 s before the first arm of
   each battery segment.** No fix → no UTC → no scoreable CPA. Write the fix
   status on the card.
2. **Mark ONE sync event visible in BOTH logs per attempt** — the simplest is
   *both aircraft armed within a few seconds of each other, in a briefed order,
   with the wall-clock time of each arm written down*. Record the raw pair, not
   the difference:
   ```
   t_arm(interceptor, wall clock): ____________
   t_arm(target,      wall clock): ____________
   ```
3. **Check `utc_synced` in the `field_score.py` output that evening.** If it is
   `false`, say so wherever the CPA is quoted — do not quietly report the number.

The evening's scoring cannot reconstruct any of this. A missing GPS fix or a
missing arm-time pair costs that attempt its kinematic half, permanently.

---

## 7. Scoring — after the session

```bash
.venv/bin/python scripts/field_score.py \
    --ulog-a logs/field/<interceptor>.ulg   --label-a interceptor \
    --bin-b  logs/field/<target>.BIN        --label-b target \
    --lethal-radius <THE RATIFIED RAM RADIUS — see §8.3> \
    --video-a <seeker video>  --video-b <phone slow-mo> \
    --out-dir logs/field_score --tag L5_attempt3
```

- **Run it per attempt**, including the misses. The miss geometry is the data that
  sizes the next attempt's C1.
- Output: a JSON verdict + a two-panel PNG (range-vs-time with the radius line and
  the CPA marked; top-down East–North with the CPA marked).
- The CPA is **analytic** (exact per-segment closed form on the union of both logs'
  sample times) — `--dt` affects only the plot, not the verdict.
- **`verdict_uncertain` is not a failure of the day.** Consumer-GPS inter-receiver
  bias (~1–3 m) rivals the kill radius, so the tool flags when the geometry cannot
  support the classification. Under the binary-kill criterion the video already
  decided; the CPA is supporting evidence (§0).
- **Do not omit `--lethal-radius`.** Pass the ratified value explicitly, per §8.3,
  so the report says on its face which radius it was scored against.

Also, that evening: run the seeker video and the mission CSV side by side and
answer, for every attempt, **"did the camera terminal actually steer, and toward
what?"** `sp_source == guided` in the CSV is the machine-readable half; the video
is the honest half.

---

## 8. Pre-registered criteria — fill these in BEFORE the first flight

> Pre-registration is the whole point: write the bar down before you see the
> result, so a marginal outcome cannot be argued into a pass at the field.

### 8.1 SUCCESS (the deliverable)

**A binary kill — contact between the interceptor and the target, confirmed on
video (seeker AND/OR phone slow-mo), with both aircraft's logs recovered for the
same attempt — against a straight-line target at ≥9 m/s (L5).**

Intermediate, separately claimable successes, in order: a kill at **L3** (static),
a kill at **L4** (≥2 m/s), a kill at any completed step of the **L5** ladder.
Record which rung a kill was scored at — a static kill is a real result and it is
not a 9 m/s kill.

### 8.2 What is NOT required

- **A citable sub-metre CPA number.** Explicitly out of scope (constraint
  `binary-kill`); the RTK pair that would have measured it was cut. `field_score`
  returning `verdict_uncertain` does not downgrade a kill seen on video.
- A Pk-vs-radius curve (that is the sim metric — ADR-0025).
- Markerless/NN performance (first kills fly the tag — constraint `pi5-compute`).
  Shadow-log the NN alongside the tag if it costs nothing; it gates nothing today.

### 8.3 The kill radius

`field_score.py --lethal-radius` classifies KILL/MISS in the kinematic half. **Use
the ratified ram-radius decision in `docs/decisions.md`** — it is derived from the
ordered airframes' actual contact envelope, and it is the number every Pk claim on
every visible surface is now quoted against. Do not re-derive it here, do not use
a remembered figure, and do not rely on the tool's built-in default without
checking it equals the ratified value. If the two disagree, that is a contract
drift to report, not a field-day judgement call.

### 8.4 ABORT criteria — pre-registered, any one ends the session

- Prop-clearance, geofence, kill switch or Remote ID **not** verified (§1) — do not
  fly at all.
- A failsafe fires in a way you cannot explain from the log **and the next flight
  would be a two-aircraft engagement**.
- Any uncommanded motion after a `SAFE` transition (this is the FAILSAFE 7 hazard
  made real — stop the day and read the mission CSV).
- Damage you cannot fully inspect; wind above what L1/L2 flew cleanly in; light
  failing (the slow-mo needs light).
- Any person, at any point, inside the dash corridor or the engagement box.
- **Two consecutive rung failures with the same signature** — the field is not
  where that gets debugged.

---

## 9. Regulatory

See **`docs/regulatory_site_capture.md`** — it must be filled in before this day,
and it specifically must record **written permission for a deliberate two-aircraft
mid-air at this site**, plus the Remote ID compliance path for **both** >250 g
aircraft. The tripod day's venue and this day's venue may differ; capture both.

---

## 10. Safety checklist

- [ ] Safety glasses (ANSI Z87) on **any time props are on and a pack is in** —
      for everyone present, not just the pilot.
- [ ] Fireproof LiPo bag; never charge unattended; fire blanket on-site.
- [ ] LiPo cell checker / low-voltage alarm before every flight, both aircraft.
- [ ] Smoke-stopper inline for the first power-up after any wiring change.
- [ ] Props OFF for all bench work. Motor directions + **kill switch verified live
      in QGC/the TX before the first armed flight of the day** (`launch_mechanism_plan`
      §4) — and re-verified after any RC/TX change.
- [ ] Geofence loaded and verified on the interceptor.
- [ ] Line of sight maintained on BOTH aircraft at all times; briefed RTL/abort
      plan for each.
- [ ] **Spotter assigned and briefed with abort authority** (§5.2). Mandatory here
      — more so than at the tripod, because both pilots have a reason to be looking
      at something other than the sky.
- [ ] Post-contact plan rehearsed: **nobody approaches a downed aircraft until the
      other one is landed and disarmed**, and damaged packs go into the LiPo bag,
      not into a car.
- [ ] First-aid kit; phone signal confirmed; nearest hospital known.

---

## 11. Flight card (fill in per attempt, field-side)

**Session header — once per day:**

```
Date: ________  Site: ____________________  (regulatory capture doc filled? Y/N)
Wind: ______ m/s, dir ______   Light/ceiling: ______________
Interceptor: SD card in + logging ON? Y/N     Target: SD card in + logging ON? Y/N
Geofence loaded + verified? Y/N               Kill switch verified live? Y/N
Prop-clearance bench gate (P4) PASSED on: __________   Pi 5 AprilTag fps: ______
Bench gates run today: --audit ___  --self-test ___  check_real_flight_sitl ___
Abort volume marked (corridor length ____ m, box radius ____ m)? Y/N
Spotter: ______________   Abort words briefed ("ABORT" / "KILL")? Y/N
```

**Per attempt:**

```
Attempt #: ___  Rung: L0 / L1 / L2 / L3 / L4 / L5   Target speed: ____ m/s
C1 dash heading (deg): ______   C2 lead corr (deg): ______  (bearing latch ON/OFF)
C3 standby alt (m): ______      Loft (m): ______   Dash speed cmd (m/s): ______
Failsafe bounds flown: dash_max_s ____  dash_max_dist_m ____  engage_max_s ____
                       offboard_lost_s ____  offboard_stale_s ____

GPS: interceptor 3D fix? Y/N   target 3D fix? Y/N   (held >=60 s before arm? Y/N)
CLOCK SYNC -- t_arm interceptor (wall): __________  t_arm target (wall): __________

Link-denied arm flown this attempt (TX killed after GO)? Y/N
Outcome:  KILL / MISS / NO-HANDOFF / ABORT(reason: ________________________)
Terminal steered on video? Y / N / no-handoff       Miss direction: early/late/high/low
Final state + safe_reason from the log: ______________________
Evidence captured:  interceptor ULog ___  target .BIN ___  seeker video ___
                    phone slow-mo ___  mission CSV ___
Anomalies: ____________________________________________________________
```

Keep one card per attempt. The GPS-fix line and the two arm times are the entries
that **cannot be reconstructed afterwards** — without them that attempt has no
kinematic half, permanently.

---

## 12. After the session — what "done" looks like

- [ ] All four evidence artefacts archived per attempt, named by the §11 card
      (both logs + both videos + the mission CSV).
- [ ] `field_score.py` run **per attempt**, with the ratified `--lethal-radius`
      passed explicitly; verdict JSON + PNG archived; `utc_synced` and
      `verdict_uncertain` recorded alongside every CPA quoted.
- [ ] The **kill claim written with its rung and its evidence**: "binary kill at
      L__ (target ____ m/s), confirmed on <video>, both logs recovered" — never a
      bare "it worked".
- [ ] The measured constants from L1/L2 (standby endurance, real dash speed and
      acceleration) written back into `flight/deploy/real_flight.py` and into the
      §1.3 table, replacing the guesses **with the ULog they came from cited**.
- [ ] Any failsafe that fired, listed with the log line that shows it and whether
      its bound was right — especially FAILSAFE 7, which has never fired on real
      hardware.
- [ ] Result written back into `docs/project_state.json` (`build_plan` P5 + the
      `kill` stage note) and `NEXT.md` — this day exists to move the project's
      headline claim, so log it where the next session reads it.
- [ ] Every number quoted anywhere traces to a log file or a card (CLAUDE.md:
      numbers trace to a run or a derivation).
