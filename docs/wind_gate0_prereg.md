# Wind Gate-0 (W3) physics probe — PRE-REGISTRATION

**Written 2026-08-29, BEFORE the probe flew.** Config, prediction, criterion and the
meaning of a null are all fixed here first. A criterion chosen after seeing the numbers is
not a criterion (CLAUDE.md, standing methodology rules).

Parent decision: **ADR-0096** — which ends with the sentence this document exists to
discharge: *"NOT YET DONE, and nothing may be quoted until it is: the W3 physics probe …
No wind arm has flown. Every published number in this project still assumes a windless
world."*

---

## 1. What is actually being tested (and what is NOT)

`scripts/wind_driver.py` computes a drag force and publishes it as a `gz.msgs.EntityWrench`
on `/world/apriltag/wrench/persistent`. Its offline self-test (13/13) proves the **force
vector** and the **publish path**. It cannot prove the one link that matters:

> **Does Gazebo actually put that force on the airframe?**

That is the whole question. Everything downstream — every wind arm, the sag table, the
`--wind-trim-*` pre-flight correction — is built on an affirmative answer that nobody has
checked. This probe is an **actuator verification**, in the sense of ADR-0096's own closing
generalisation: *any component that logs what it commanded rather than what was applied can
diverge silently.* The driver's CSV logs the force it put on the wire. This probe measures
what the aircraft did.

**NOT being tested here, and must not be claimed from this probe:**

- **The drag model is not being validated.** The prediction below is computed from the same
  `MCOEF`/`BCOEF` the driver is fed, so agreement says the force *arrived*, not that the
  force is *right for a real 5-inch airframe*. Those parameters are PX4 defaults plus the
  SDF mass (contract assumption `wind-sag-table-unmeasured`, grade `given-perfect`), and
  this probe does not upgrade them. It cannot.
- **Turbulence is not being tested.** Gusts are deliberately OFF (`--gust-factor 1.0`,
  which resolves `sigma_mps=0.0`). A settled tilt is only readable in a steady field. The
  Dryden path stays unverified in Gazebo after this probe and must be said so.
- **Nothing about the real aircraft.** SITL only.

## 2. Configuration (fixed before flight)

| | |
|---|---|
| Airframe / world | `gz_x500_mono_cam`, `PX4_GZ_WORLD=apriltag` — the same boot as every other arm |
| Hover altitude | **6.0 m AGL** (above the 3.048 m Dryden floor, so no `--allow-below-10ft`) |
| Wind | `--mean-mps 5.0 --dir-from-deg 270 --height-m 6.0 --seed 7 --gust-factor 1.0` |
| Resolved field | `sigma_mps = 0.0` — steady, no gusts (confirmed by `--dry-run`) |
| Direction | FROM 270° = **from the west**, so the aircraft is pushed **east** and must tilt **west, into the wind**, to hold position |
| Drag params | `mass 2.114308 kg`, `MCOEF 0.15 s⁻¹`, `BCOEF 100 kg/m²`, `rho 1.225` (`DragParams.px4_x500_mono_cam_sitl`) |
| Velocity filter | `--velocity-lowpass-tau-s 0.0` (OFF, raw) — test what actually flies, not a tuned variant |
| Attitude source | MAVSDK `telemetry.attitude_euler()` (the estimator's `vehicle_attitude`), logged at 10 Hz |
| Machine load | idle, one sim only (batch hygiene, ADR-0015) |

### Three phases in ONE flight, so the control is paired within the run

| Phase | Sim seconds | Driver | What it establishes |
|---|---|---|---|
| **A — baseline** | 10 s after hover settles | not running | Tilt with no external force. Position hold at zero wind requires zero tilt. |
| **B — wind on** | 30 s (read the **last 15 s**) | steady 5 m/s from 270° | The measurement. |
| **C — wrench cleared** | 15 s (read the **last 10 s**) | killed; clears on exit | Whether the force is actually *removed* — the other half of ADR-0096's stop sign. |

Phase C is not decoration. `ApplyLinkWrench` persists a wrench until it is replaced or
cleared, and the accumulating-wrench defect ADR-0096 fixed lived in exactly this machinery.
A wrench that cannot be removed would contaminate every subsequent flight in the same sim
boot.

## 3. The physics, derived before the flight

In position hold, the only way an aircraft produces a steady horizontal force is by tilting
its thrust vector. At zero groundspeed the relative air velocity is the wind speed, so:

```
a_drag = MCOEF·V + rho·V²/(2·BCOEF)
       = 0.15·5 + 1.225·25/(2·100)
       = 0.750 + 0.153 = 0.9031 m/s²          (F = m·a = 1.909 N)

theta  = atan(a_drag / g) = atan(0.9031 / 9.80665) = 5.262°
```

`wind_driver.py --dry-run` prints the same 5.26° independently, labelled *"DERIVED — the
Gate-0 probe has not confirmed it."* This probe is what removes that label, or doesn't.

**A sustained non-zero tilt at zero groundspeed IS the evidence.** With no external force,
holding position requires a level attitude; there is no other explanation available for a
steady tilt that appears when the driver starts and disappears when it stops.

## 4. Criteria (adopt / reject), fixed now

The probe **PASSES** only if all four hold:

1. **A — baseline level.** Phase-A mean tilt magnitude **< 1.0°**.
2. **B — magnitude.** Phase-B settled mean tilt within **±30%** of 5.262°, i.e.
   **3.683° ≤ |theta| ≤ 6.840°** (the ±30% band is ADR-0096's own, set before this run).
3. **B — direction.** The tilt leans **into the wind** (westward, −East component of the
   thrust-tilt), within **±30°** of the 270° bearing. A correct magnitude in the wrong
   direction is a FAIL, not a partial pass — a sign error in `force_to_world_xyz` is
   precisely the defect class this catches.
4. **C — removable.** Phase-C mean tilt magnitude **< 1.0°**, i.e. back to the phase-A
   regime, and the driver reports a clean wrench clear on exit.

Additionally, the run is **VOID (exit ≠ 0, never PASS)** if any of these hold — no vacuous
verdicts:

- the driver's `WIND_DRIVER_RESULT` line reports `published=0`, any `publish_failed>0`, or
  `pose_updates=0`;
- fewer than 50 attitude samples land in any phase window;
- the aircraft drifts more than 3 m from its hover position (position hold lost — the tilt
  then reflects a translation transient, not a force balance, and the derivation does not
  apply);
- the mean commanded force in the phase-B window departs from 1.909 N by more than 10%
  (the driver was not commanding the field this prediction assumes).

## 5. What each outcome MEANS — including the null

**PASS.** Gazebo applies the persistent wrench, at the commanded magnitude, in the correct
direction, and releases it. ADR-0096's stop sign lifts: wind arms may fly and their results
may be quoted — *with the standing caveat that the drag parameters remain PX4 defaults, not
airframe measurements.* `zero-wind-environment` stays `given-perfect` for every historical
result (they were flown windless and that does not change); the wind arm becomes the tool
that grades it going forward.

**NULL — no tilt, or tilt far below prediction.** This is the outcome the probe exists to
catch and it is a genuine finding, not a failure of the day: **the persistent-wrench route
does not drive this airframe**, and `scripts/wind_driver.py` is inert in exactly the way its
own CSV cannot show. Consequences, stated now so they cannot be softened later:

- Every wind capability in the repo is unproven and must be marked so on the dashboard, in
  ADR-0096, and in the MBSE views the same turn.
- The `--wind-trim-*` lever loses its only evidentiary basis; the sag table cannot be
  populated from sim, so it stays `given-perfect` with no route to `measured`.
- The rejected routes in ADR-0096 (bare `<wind>` SDF, WindEffects plugin) come back off the
  shelf and get re-examined — the ADR's rejections were reasoned but not, in that case,
  load-bearing-tested.
- **The wind question does not get quietly dropped.** A null here means the honest project
  position is "wind is unmodelled and we could not model it", which is a weaker but true
  claim, and it goes on the always-visible surfaces.

**PARTIAL — tilt present, correct direction, magnitude outside ±30%.** The transport works;
the calibration does not match the derivation. This is *not* a pass. It is a specific new
question (physics-step aliasing at 20 Hz driver rate vs 250 Hz physics, or a mass/entity
mismatch), it gets its own investigation, and no wind number is quoted until it resolves.

**WRONG DIRECTION.** Treated as a hard FAIL and a bug hunt in `force_to_world_xyz` /
the ENU conversion, regardless of how good the magnitude looks.

## 6. Provenance

- Harness: `scripts/check_wind_gate0.sh` → `scripts/wind_gate0_probe.py`
- Raw output: `logs/wind_gate0_<UTC>/` (attitude CSV, driver applied-wrench CSV, sim log,
  `verdict.json`)
- Parent: ADR-0096. Result recorded as an ADR-0096 addendum and in `docs/project_state.json`
  the same turn, whichever way it goes.
