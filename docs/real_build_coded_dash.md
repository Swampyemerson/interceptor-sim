# The real interceptor: coded-dash → camera-only (build reference)

*Consolidates the 2026-07-15 real-build pivot (ADR-0076 add #18a–c, memory
`real-build-pivot`). This is the operating model the physical interceptor in
`docs/hardware_order_list.md` actually flies — it supersedes the cue-guided /
FusedTrack architecture of `deployment_phases_design_brief.md` and
`phase2_sim_to_real_plan.md` for the onboard interceptor (those remain the
reference for the parent project's ground-sensing half).*

## Operating model (one line)

**A CODED, open-loop DASH pointed roughly at the target, then a CAMERA-ONLY
proportional-navigation terminal to impact.** No ground-sensor cue, no datalink
mid-course, no cue-fusion / handoff-gate / coast-search. The dash is
hand-programmed (or fired on a rough ground bearing); the onboard camera does the
rest.

Phase machine (in `scripts/m4_intercept.py --coded-dash`, and the target of the
`flight/` port): `TAKEOFF → CODED_DASH → ENGAGE → BREAKOFF → LAND`.
- **CODED_DASH** — fly a fixed heading + speed. Auto-heading solves the
  collision-lead azimuth from the *known* launch kinematics (a pre-flight
  constant, not a live sensor read — honesty boundary intact); `--dash-heading-deg`
  overrides it (the "human points it" case). Hand off the instant the detector
  holds 5 consecutive fresh detections.
- **ENGAGE** — the unchanged camera-only pro-nav terminal: `a = N·Vc·λ̇` on the
  full-attitude-derotated LOS. `BREAKOFF` past closest approach; `LAND`.

## Why the pivot (what it fixed)

The sim spent weeks on the realistic-quad **r2l** failure inside the cue-guided
pipeline (ADR-0076 add #13–#17). The real interceptor has **no cue**, so much of
that debugging was on machinery the hardware won't have. Testing the actual flight
architecture (coded-dash) both re-scoped the problem and *improved* it.

### Validation — paired A/B (quad_v2, seed 123, weave, n=8/dir, geometry byte-identical)

| architecture | dir | acquire | Pk@2.5 m | miss mean (m) |
|---|---|---|---|---|
| cue-guided (full deployment cfg) | l2r | 8/8 | 8/8 | 0.61 |
| cue-guided | r2l | 8/8 | **0/8** | 3.97 |
| **coded-dash** | l2r | 8/8 | 8/8 | 1.37 |
| **coded-dash** | r2l | 8/8 | **5/8** | 2.68 |

The saga's "r2l 0%" was **0/8 inside the 2.5 m Pk gate** (it engaged 8/8 but at
3.25–4.27 m), *not* a no-engagement null. Dropping the fused dash (which
under-committed east because the aspect-biased r2l bearing corrupted its aim)
lifts **r2l 0/8 → 5/8 @Pk, mean 3.97 → 2.68 m**. Cost: l2r loosens 0.61 → 1.37 m
(the cue genuinely helped l2r) but stays 8/8 @Pk. Net: the 6.5× l2r/r2l asymmetry
collapses to 2×; combined Pk@2.5 m **8/16 → 13/16**. *~~This is the number that
matters for the hardware — no cue exists on it.~~* *[caveat — ADR-0076 add #18g/#18h:
the 13/16 combined is largely open-loop DASH BALLISTICS; a dash-only control scored
the same, and no camera-guided 3D-quad intercept exists in sim. The dash AIM is the
real result; the camera terminal added ~nothing. See `docs/project_state.json`.]* Data:
`logs/mc_coded_dash_qv2_weave.csv` vs `logs/mc_quad_v2_s123_weave.csv`;
`scripts/coded_dash_summary.py` reproduces the table.

### Robustness — the hand-programmed dash carries operator aim error

Heading-error sweep (`--dash-heading-err-deg`, 48 flights over 0/+15/+30°):

| heading err | l2r Pk@2.5 | l2r miss | r2l Pk@2.5 | r2l miss |
|---|---|---|---|---|
| 0° | 8/8 | 1.37 | 5/8 | 2.68 |
| +15° | 8/8 | 2.33 | 7/8 | 1.60 |
| +30° | 0/8 | 3.25 | **8/8** | **0.78** |

1. **Acquisition survives ≥30° aim error** — all 48 flights acquired and
   engaged. The dash only has to point *roughly* right~~; the camera terminal
   finds the target once it enters frame. This is the core viability result~~.
   **⛔ SUPERSEDED (ADR-0076 add #18g/#18h — see the retraction banner below):**
   "acquired and engaged" means the handoff state machine fired; the gt-consistent
   audit showed those ENGAGE detections were overwhelmingly PHANTOM (~0 in-flight
   approach recall, add #18i; #18k localizes the wall to dash-pitch pointing).
   The camera terminal did NOT find the target. What this sweep actually shows:
   the **OPEN-LOOP DASH AIM tolerates ≥30° heading error** — dash-aim robustness,
   not perception proof (`docs/project_state.json` launch_aim).
2. **l2r Pk tolerance ≈ 15–20°.**
3. **r2l improves monotonically with east bias** → sub-meter 8/8 @0.78 m by +30°.
   So the r2l residual is a *systematic, fully-correctable east-aim deficit*, not
   a stochastic perception floor. **[⛔ per add #18g/#18h the 0.78 m is open-loop
   dash ballistics — aim-calibration evidence, not camera-terminal performance;
   "not a perception floor" is retracted: the flight-dynamic perception wall
   (add #18k) IS the binding wall.]**

## The portable core — `flight/` (runs identically in sim and on the Pi)

Pure Python + `math`, **no gz / ground-truth / cue imports** → the same code
flies in Gazebo SITL and on the real Pixhawk-6C/PX4 + Pi5. 25 tests
(`scripts/run_tests.sh`), and `m4_intercept.py` already calls `flight.guidance`
for its coded-dash aim (byte-identical).

| module | what it is | hardware role |
|---|---|---|
| `flight/geometry.py` | camera→body→NED LOS derotation (full attitude + mount tilt), `wrap_pi` | turns a camera bearing + EKF attitude into an inertial LOS azimuth |
| `flight/estimator.py` | alpha-beta (g-h) tracker for the λ and R channels + Kalata adaptive gains | smooths the noisy, intermittent LOS/range for pro-nav |
| `flight/guidance.py` | `collision_lead_heading` (coded-dash aim), `closing_speed`, `pronav_lateral_accel` (`a=N·Vc·λ̇`) | the dash aim + the terminal guidance law |
| `flight/camera.py` | pinhole + Brown-Conrady lens model; `pixel_to_ray` / `pixel_to_bearing` | **undistorts** the wide-M12 pixel before the bearing (see below) |

The perception→guidance chain on hardware:
`pixel (YOLO/AprilTag) → CameraModel.pixel_to_ray → derotate_bearing_lambda →
AlphaBetaFilter → pronav_lateral_accel`.

## Camera pipeline (calibrate → model → bearing) — closed and validated

The real Arducam AR0234 + ~100° M12 lens **barrel-distorts**; a bearing from a raw
pixel is wrong toward the frame edges (>1° at the edge on a representative model)
→ corrupts the LOS. So the seeker must undistort first.

- `scripts/calibrate_camera.py` — checkerboard → intrinsics + Brown-Conrady
  distortion (`--live` or `--images`). **`--self-test`** validates the whole
  calibration math on synthetic data (recovers fx/fy/cx/cy + k1/k2 exactly,
  rms 0.0 px) and confirms `flight.camera` loads the result — CI-gateable before
  the hardware exists.
- `flight.camera.CameraModel.from_json(calib.json)` loads it; zero-distortion is
  the Gazebo pinhole special case (byte-identical if wired into the sim seeker).

## Hardware mapping (`docs/hardware_order_list.md`, ~$1,089)

> ⚠️ **SUPERSEDED totals/parts:** this table predates the §0c binary-kill lean re-scope
> (`docs/hardware_order_list.md` §0c, 2026-07-15) — interceptor **~$740**; camera = **OV9281 mono
> GS wide** (AR0234 below = upgrade path only); **Hailo HAT DEFERRED** — first kills fly the
> AprilTag baseline on the Pi 5 CPU. Current state: `docs/project_state.json`.

| real part | why it transfers |
|---|---|
| **Pixhawk 6C Mini / PX4** | the exact stack the sim validated — params + MAVLink/OFFBOARD guidance transfer directly |
| Pi 5 8GB + Hailo-8L | YOLO11n @640 on the NPU; CPU free for MAVLink *(HAT deferred to the markerless phase, §0c)* |
| Arducam AR0234 global-shutter + ~100° M12 | the terminal seeker (AprilTag was the sim stand-in); calibrate with `calibrate_camera.py` *(upgrade path; lean build = OV9281 mono, §0c)* |
| adjustable up-tilt / **prop-clearance** bracket | prop-clearance per §0② (add #18h); a FIXED tilt does NOT close the dash-pitch gap — it only relocates the in-view window (add #18j-fix; no fixed tilt adopted, ADR-0067/0068 — adaptive tilt #46 is the open lever) |
| ArduPilot GPS quad, AUTO box | the target (straight legs ≥2 m/s) |

## Build path when the hardware arrives (open, largely hardware-gated)

1. **Frame source** — Picamera2/libcamera → the seeker (`Measurement` seam).
2. **MAVSDK over TELEM2 serial** (921600) + PX4 params + a **safety module**
   (RC kill live, geofence, dash timeout) + a wall-clock (not sim-clock) audit.
3. **AprilTag on the Pi** = the flight baseline seeker; then the Hailo YOLO path
   (needs a real-data retrain). Do the prop-clearance geometry check first.
4. **`real_flight.py`** — wire `flight/` + MAVSDK + frame source into
   TAKEOFF → coded dash → camera-terminal ENGAGE → breakoff → land.
5. **`field_score.py`** — CPA from the two GPS logs, the real-world miss metric.

## Goal-condition results + where the sim work lands (2026-07-15, ADR-0076 add #18e/f → CORRECTED by #18g)

> **⛔ RETRACTED (Fable adversarial verification, ADR-0076 add #18g).** An earlier version of
> this section claimed the guidance was "solved" and the camera terminal produced a sub-meter
> r2l kill. That was a MIRAGE: the sub-meter r2l numbers are the OPEN-LOOP DASH's own
> ballistics, NOT camera guidance — a control arm that never engaged the camera scored the same
> ~0.54 m, and the ENGAGE detections were phantom (LOS error ±120–140°). **No camera-tracked <1 m
> intercept of the 3D quad exists in the dataset.** What is real: the crossing-bias is a good
> OPEN-LOOP AIM CALIBRATION (r2l 3.46 → dash 0.54 m). The real wall is that the detector has
> **~0 recall on the APPROACHING target** (2 real vs 1187 phantom detections on approach across
> 63 flights); the phantom is the false-handoff hazard, not the acquisition blocker, so
> "prop-clearance fixes acquisition" is UNTESTED. Next sim test: camera-forward × coded-dash
> (`scripts/experiments/cam_forward/` + `--cam-fwd-offset-m 0.40`) — see add #18g. *(Since FLOWN →
> add #18h: phantom removed at source, approach recall still ~0 — prop-clearance is NOT the
> acquisition fix; the wall is flight-dynamic pointing, add #18k.)* The
> superseded text below is kept only for the audit trail.

The goal crisped to: intercept a **≥20 mph (~9 m/s)** target to **<1 m**, camera-only. *(METRIC
SUPERSEDED same day, 2026-07-15: success re-scoped to a BINARY contact KILL confirmed on seeker
video + phone slow-mo + both ULogs, NOT a measured <1 m CPA — literal <1 m @9 m/s is graveyarded;
see `docs/hardware_order_list.md` §0c + `docs/project_state.json`.)*
The sim was pushed hard on the actual goal geometry (a straight line @9 m/s, not the
easier weave). ~~Honest landing:~~ *(superseded — see the retraction above)*

- **GUIDANCE / aim is largely SOLVED.** The straight-line baseline missed by ~3.4 m
  because it unmasks a direction-dependent **aspect-biased aim** (ADR-0056). Built
  `--dash-crossing-bias-deg` — a per-direction correction whose sign auto-keys on the
  crossing direction (dash × `--target-vel`, a pre-flight constant, honest, no gt).
  **Validated on 2 seeds: r2l median 0.72 m, 9/16 within 1 m, best 0.37 m (contact)** —
  the markerless *aim* can be corrected to the edge of kill range with no perception
  change. But the correction is **speed/aspect-specific** (the +30° tuned for 9 m/s
  over-corrects at 5 m/s), which points at a real-data detector — not a hand-tuned
  bias — as the robust fix.
- **PERCEPTION is the wall, and it's HARDWARE/real-data-gated (proven, not assumed).**
  l2r locks only ~half the time because the detector locks the **own-prop phantom**
  (median implied range 1.6 m) while the real target at 16 m is barely seen. Every
  software separator fails — a handoff range-plausibility gate is NULL (phantom
  overlaps the real detections in implied range, ADR-0076 add #13), the phantom-free
  retrain is NULL in flight (add #18d), ~12 prior levers NULL. **The fix is the
  hardware prop-clearance mount (§0② of the BOM — remove the phantom at the source)
  plus a real-data-trained detector** (no aspect-bias, no prop-lock).

## Honest open items

- **Success criterion (current):** BINARY KILL on video + both ULogs (§0c re-scope,
  2026-07-15) — not a measured <1 m CPA.
- ~~**The markerless kill is guidance-ready but perception-blocked.** At 9 m/s, r2l
  reaches contact on the good flights (not reliably); l2r is phantom-blocked. The two
  remaining walls — l2r phantom-acquisition and r2l bearing-tightening — are the known
  perception limits that need **hardware (prop-clearance) + real-flight data**, not
  more sim levers. The sim has extracted the guidance win and named the hardware
  requirements; the next real progress is BUILDING (R4+).~~
  **⛔ SUPERSEDED (ADR-0076 add #18g/#18h/#18k — see the retraction banner above; current
  state: `docs/project_state.json`):** no camera-tracked <1 m intercept of the 3D quad
  exists; the 9 m/s r2l contact-range flights were open-loop dash ballistics (#18g,
  confirmed by the #18h dash-only control — one hit 0.32 m with ZERO real ENGAGE
  detections). The binding wall is FLIGHT-DYNAMIC pointing (~40° nose-down dash pitch
  parks the target at the frame-top edge; detector 100% static 8–22 m, 0.8% in flight —
  #18k) → adaptive camera tilt (#46/ADR-0065) is the dominant, sim-testable lever; the
  real-data detector is a HYPOTHESIS for the outdoor-appearance gap; prop-clearance
  removes the false-handoff phantom hazard only, not acquisition.
- **Everything past `flight/` is untested** until the hardware exists — the
  frame source, serial MAVSDK, and the real seeker are hardware-gated. The
  guidance core (incl. `--dash-crossing-bias` mechanism) is validated and transfers;
  its magnitudes must be re-tuned on real data.
