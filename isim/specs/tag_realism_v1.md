# tag_realism_v1 — make the simulated AprilTag behave like one bolted to a real drone

Builder directive (2026-09-23 night): *"make sure the current way the sim is replicating the
april tag is true enough to real life… a real drone will tilt with it, pitch with it, shake
some, have glare, etc."* This spec is the design; the builder's three words map to sections
A (tilt/pitch), B (shake), C (glare). Section D is wiring, E is tests, F is the pre-registered
measurement plan.

## Ground rules (same as every isim spec)

- **Default-off, bit-identical.** Every new knob defaults to a value that reproduces today's
  behaviour exactly (guard rng draws so the default path draws nothing new). Extend the
  existing pin test pattern (`test_scenario.test_scatter_none_is_bit_identical_to_no_scatter`,
  and the byte-identical guard used for `min_decode_interval_s`).
- **Honesty grading.** Every new constant is `estimate` (literature/physics-plausible, not
  bench-measured) unless stated. Add every new unmeasured field name to `isim.seeker.UNMEASURED`
  (or a parallel tuple in the new module). Numbers derived under these knobs are model outputs.
- **No behaviour change to guidance.** Guidance still sees only `Detection`. All new truth goes
  to `FrameReport` (scoring/diagnostics only), as NEW fields with defaults (additive-only).
- Files: new module `isim/target_attitude.py`; edits to `isim/types.py` (additive),
  `isim/seeker.py`, `isim/scenario.py`; tests in `isim/tests/test_target_attitude.py` and
  extensions to `test_seeker.py` / `test_scenario.py`.
- Style: match the existing isim code (NED/FRD/OpenCV frames, (w,x,y,z) quats, docstrings that
  say what is measured vs estimated, `_EPS` guards, no global rng).

## A. Target attitude + body-mounted tag (tilt/pitch with the drone)

Today `TargetState` is a point: the tag hangs in space, always upright
(`tag_corners_ned` builds e1 ⊥ world-down), its normal fixed or velocity-slaved. A real
multirotor banks to turn and pitches nose-down to cruise, and the tag is bolted to it.

**A1. `TargetState` gains optional attitude** (in `isim/types.py`, additive):
```python
quat_wxyz: Optional[Tuple[float, float, float, float]] = None  # body->NED; None = legacy upright
ang_vel_body: Optional[Vec3] = None                            # rad/s, body frame; None = zero
```
`None` means "no attitude model" and every existing consumer keeps today's behaviour.

**A2. Pure-function attitude derivation** — new module `isim/target_attitude.py`.
Targets stay stateless pure functions of `t` (that property is load-bearing for
reproducibility), so attitude is derived, not integrated:

```python
@dataclass
class TargetAttitudeParams:
    accel_stencil_s: float = 0.10   # half-width h of the central difference; doubles as the low-pass
    max_tilt_deg: float = 35.0      # cap (SpeedChangeTarget has a true velocity step -> accel pulse)
    drag_tilt_at_9ms_deg: float = 12.0  # nose-down pitch needed to hold 9 m/s cruise; estimate
    rate_stencil_s: float = 0.02    # half-width for the angular-rate difference

class AttitudeTarget:                # wraps any TargetModel
    def __init__(self, inner: TargetModel, prm: TargetAttitudeParams = ...): ...
    def state(self, t: float) -> TargetState:   # inner state + derived quat + ang_vel
```

Physics, evaluated at time `t` (all finite differences call `inner.state()`, which is pure):
- `a(t) = (vel(t+h) - vel(t-h)) / (2h)` with `h = accel_stencil_s`. The wide stencil is the
  low-pass: a 2 m/s velocity step (SpeedChangeTarget) becomes a ~10 m/s² pulse spread over
  2h ≈ a real quad's brief pitch transient, then the cap clamps it.
- Drag: a quad holding speed `v` tilts into its drag. Model drag accel as quadratic,
  calibrated by the one knob: `a_drag(v) = g·tan(drag_tilt_at_9ms_deg) · (v/9)²`, directed
  opposite the horizontal velocity. Total specific force the rotors must produce:
  `f = a - g_ned + a_drag_vec`, with `g_ned = (0, 0, +9.81)` (so `-g_ned` points up).
- Body z (down) axis = `-f/|f|`. Yaw: nose along horizontal velocity when horizontal speed
  > 0.5 m/s, else hold the last-computable heading by evaluating at the nearest time the
  speed was above threshold — simpler and acceptable: use velocity at `t` when speed > 0.5,
  else north (hover targets get yaw north; fine, tag facing dominates). Build the body->NED
  rotation from (z_body, yaw) the standard way (tilt applied about the horizontal axis ⊥ f).
- Tilt cap: if the angle between `-f` and up exceeds `max_tilt_deg`, slerp the z-axis back to
  the cap. Monotone, pure.
- `ang_vel_body`: central difference of the derived quaternion at `t ± rate_stencil_s`
  (reuse `seeker._omega_body`).

Sanity anchors (write these into the tests): constant-velocity 9 m/s target → level roll,
nose-down pitch ≈ `drag_tilt_at_9ms_deg`, zero rate. Weave amp 2 m / period 6 s @ cruise → peak
bank ≈ atan(amp·ω²/g) ≈ 13° oscillating at the weave period. Hover → identity-ish (yaw north).

**A3. Tag bolted to the body** (`isim/seeker.py::TagParams`): new mode
```python
body_normal_frd: Optional[Tuple[float, float, float]] = None  # tag normal in TARGET body FRD
```
When set AND the `TargetState` carries a quat: normal = `R_target_body_to_ned @ body_normal_frd`,
and the tag's in-plane corner basis is ALSO body-fixed (e1 = the body axis ⊥ normal chosen
horizontal-at-level, rotated by the target attitude) — so a banked target visibly rotates the
square. When the state has no quat, fall back to the legacy normal path (facing modes
unchanged). `faces_camera` stays the idealized best-case cheat, explicitly unaffected.
Mapping used by `scenario._tag_for` when attitude is on: `rear` → `body_normal_frd=(-1,0,0)`,
`side` → `(0,1,0)` sign-matched to today's world-frame choice. Keep incidence ≥ 90° → p=0
(back of tag) — with a nose-down cruise pitch a rear tag now faces up-and-back, which is the
kind of geometry shift this whole spec exists to expose.

**A4. Blur from target rotation.** `_blur_px` today uses only translational relative velocity
+ own body rate. A rotating tag smears its own corners: add corner speed `|ω_tgt × r|` with
`r = side/2` — i.e. add `fx · |ω_tgt| · (side_m/2) / z · exposure_s` (worst-corner, small-angle)
to the image-plane smear (root-sum-square with the translational term is fine and stated).
Uses `TargetState.ang_vel_body`; zero when absent.

## B. Shake (vibration, both vehicles)

**B1. Target attitude wobble** — a hovering/translating quad visibly wobbles 1–3° at a few Hz
(gusts + control dither). This is TRUE tag motion, so it lives in the SEEKER's view of the tag
(keeps targets pure; the seeker already has per-run rng + state):
```python
# DecodeParams (or a new ShakeParams dataclass on the seeker):
tgt_shake_rms_deg: float = 0.0      # 1-sigma roll/pitch wobble ANGLE; 0 = off (default)
tgt_shake_bw_hz: float = 3.0        # bandwidth of the wobble
```
Implement as a 2-axis OU process sampled at frame times (state in the seeker, reset() clears;
`dt` between frames from capture times; no draws when rms = 0). The drawn wobble angles tilt
the tag normal/basis (small-angle rotation about the two in-plane axes) BEFORE incidence and
corner projection; the OU RATE (analytic: rate_rms ≈ 2π·bw·angle_rms, use the actual process
increment / dt) adds to the A4 rotational blur term.

**B2. Own-camera vibration** — prop-induced angular vibration on the interceptor. OV9281 is
global-shutter and the measured outdoor exposure is ~1 ms, so vibration shows up as a small
extra smear, not jello:
```python
# CameraParams:
vib_rate_rms_dps: float = 0.0       # 1-sigma body angular-rate vibration, deg/s; 0 = off
```
Adds `fx · radians(draw) · exposure_s` of blur (one Gaussian |draw| per frame when > 0).
Bench-measurable later: motors-on static frame-to-frame corner jitter. estimate for now.

## C. Glare / illumination (sun geometry)

Per-run `p_max` scatter already models "a worse lighting day" GLOBALLY. What it cannot model
is geometry-gated loss — glare that switches on exactly when the approach angle lines up with
the sun, i.e. exactly during the terminal. Add sun geometry:

```python
@dataclass
class GlareParams:                   # isim/seeker.py
    sun_azimuth_deg: float = 180.0   # world NED azimuth the SUN sits at (0 = north)
    sun_elevation_deg: float = 35.0  # above horizon
    specular_strength: float = 0.0   # 0 = off (default). Matte print ~0.4, glossy/laminated ~0.9
    specular_width_deg: float = 12.0 # half-width of the glare lobe
    backlight_kill_deg: float = 0.0  # 0 = off. Cone half-angle around the sun where flare
                                     # crushes contrast (camera looking INTO the sun)
    backlight_strength: float = 0.9  # decode-prob multiplier depth inside that cone
```
Per frame (only when a strength > 0):
- Sun unit vector `s_ned` from az/el (pointing FROM the scene TOWARD the sun).
- Specular: reflect `-s_ned` about the tag plane normal → `r`; θ = angle(r, tag→camera unit).
  `p *= 1 - specular_strength · exp(-(θ/specular_width_deg)²)`.
  Skip when the sun is behind the tag plane (`dot(s, n) <= 0` → no glare).
- Backlight: φ = angle(camera boresight (+z_cam in NED), s_ned); if φ < backlight_kill_deg:
  `p *= 1 - backlight_strength · (1 - φ/backlight_kill_deg)` (linear ramp to full kill on-axis).
- Report both multipliers in `FrameReport` (C-diag below).

## D. Wiring (`isim/scenario.py`)

- `Scenario.target_attitude: bool = False` — when True, `build()` wraps the target in
  `AttitudeTarget` and `_tag_for` emits body-mounted tags per A3 (rear/side); "camera" facing
  ignores attitude by design. Also new plain fields `sun_azimuth_deg/sun_elevation_deg`
  passthrough (picklable floats, defaults as above).
- New `Scatter` fields (all drawn from the same per-run rng, documented in the same style,
  each independently zeroable; EXPECTED-tier defaults in parentheses — but the FIELD defaults
  must be the OFF values so `Scatter()` today's meaning changes only where stated below):
  - `tgt_drag_tilt_sigma_deg: float = 3.0` — 1-sigma scatter on `drag_tilt_at_9ms_deg`
    (drawn only when `target_attitude`; clipped ≥ 0). estimate.
  - `tgt_shake_rms_min_deg / max_deg: float = 0.5 / 2.5` — uniform per-run wobble RMS
    (applied only when `target_attitude`). estimate.
  - `own_vib_rate_rms_max_dps: float = 40.0` — uniform 0..max per run. estimate.
  - `sun_azimuth_uniform: bool = True` (uniform 0–360 per run) and
    `sun_elevation_min/max_deg = 15/60` — drawn only when glare is enabled for the run.
  - `specular_strength_min/max = 0.2/0.6` (matte print assumed — NOTE FOR THE BENCH: print
    matte, laminate = glossy tier 0.6/0.95), `backlight_kill_max_deg = 25.0` uniform 0..max.
  - Gate all glare draws behind `Scenario.target_attitude`? NO — glare is independent; gate
    them behind a new `Scenario.glare: bool = False` instead.
- `Scenario.target_attitude=False` and `glare=False` (defaults) ⇒ byte-identical runs (pin it).

## E. Tests (extend the existing suites' style; every new default path pinned)

1. `AttitudeTarget` physics anchors from A2 (level cruise pitch, weave bank ≈ 13°, hover, cap,
   SpeedChangeTarget transient stays finite and ≤ cap; attitude is a pure function: two calls
   at the same `t` bit-equal).
2. Corners rotate with the body: banked target → corner set rotates by the bank about the LOS
   (compare to hand-rotated corners).
3. Rear tag + nose-down pitch: incidence from dead-astern INCREASES by the pitch (regression
   number, not just monotonicity).
4. Blur: spinning target (synthetic ang_vel) adds the predicted `fx·ω·(side/2)/z·exposure`.
5. Shake OU: rms converges to the knob (statistical, seeded), off = no draws (rng call-count
   guard, same trick as min_decode_interval_s), reset() clears state.
6. Glare: exact specular geometry → multiplier = 1-strength; sun behind tag → no effect;
   backlight on-axis → strength kill; both off → no draws.
7. Bit-identical defaults: full `run_one` (or `build()`+engine) with defaults, hash of the
   trace equal before/after the change (the existing pin-test pattern).
8. `FrameReport` new fields default so old constructors still work.

## F. Pre-registered measurement plan (fly AFTER the build is green — rules: prereg 2026-07-25)

Baseline to reproduce first (regression check): v6 grid, pursuit, all v5 errors on, fx 385,
tilt 12, n=100/cell — rear ≈ 68% inside 0.35 m / 91% inside 1.0 m; camera-facing ≈ 49/83%.
(2026-09-17 numbers; if the repro shifts > ~5 points, STOP and find why before layering
realism.)

Then the realism ladder, same grid, n≥100/cell, one factor at a time then all-on:
1. `target_attitude` only; 2. + shake; 3. + own vibration; 4. + glare (EXPECTED matte tier);
5. all-on WORST tier (glossy specular 0.95, shake 2.5–4°, drag-tilt +2σ).

**Predictions (written before flying):** (P1) attitude coupling costs the REAR tag the most —
nose-down cruise pitch + weave bank pushes astern incidence toward the p-decode knee; expect
rear nominal to drop 5–15 points, camera-facing ~0 (it's the cheat mode). (P2) shake/vibration
are small at 1 ms exposure (< 3 points). (P3) glare is bimodal per run — most runs unaffected,
sun-aligned runs lose the terminal entirely; expect a fat lower tail rather than a mean shift,
EXPECTED tier 3–10 points, WORST/glossy much worse. (P4, the builder's bet) if rear-tag all-on
drops below ~55% inside 0.35 m, the algorithm needs work; candidate levers, in order: approach
geometry that respects the PITCHED tag normal (aim slightly above/below to flatten incidence),
last-decode coast that uses the target's estimated attitude, dual/angled tags (hardware).

**Config amendment (registered 2026-09-23, before the ladder flew — supersedes the
"v6 grid" baseline above, which predates the fx-385 lens and the port arm):** the ladder
runs on the CANONICAL registered config of the 2026-09-23 A/Bs — the PORT arm
(`Scenario(concept="flyby", terminal="pursuit", scatter=Scatter())`), paired seeds 0..49,
n=50/cell — via `scripts/tag_realism_ladder.py`. Rungs: R0 base (must REPRODUCE
`chase_tilt_ab` tilt-0: alt+3 = 34% ≤0.35 m — the regression check), R1 attitude only,
R2 +shake, R3 +vibration (40 dps), R4 +glare EXPECTED, R5 WORST (glossy 0.6–0.95,
shake 2.5–4°, vib 80, flare cone 35°). Cells: rear nom / aim20 / alt+3 / weave, plus
nom & alt+3 at cam_tilt_up_deg=10 (the pending ADR-0112 bracket recommendation — a
nose-down target tilts its rear tag UP-and-back, so target attitude plausibly interacts
with camera tilt; prediction P5: the +10° alt+3 win of ADR-0112 SURVIVES attitude
coupling, i.e. stays ≥ +15 points over tilt-0 at the same rung), and camera-facing nom
(idealized control, prediction ≈ flat across rungs by design).

**Adopt/reject:** the realism knobs land regardless (they're truth-model, not a tuned lever) —
what's decided is the HEADLINE: if all-on EXPECTED drops the standing numbers by > 5 points,
the contract's terminal-stage numbers get rewritten to the realism-on figures and the old grid
is regraded a best-case upper bound. A NULL (no material drop) means the tag model was already
adequate: report that straight, do NOT invent a degradation, and the algorithm-refinement half
of the night is judged unnecessary — the builder's "if I was right" was conditional.

## G. Canted-mount A/B (registered 2026-09-23 night, BEFORE flying — follows the ladder,
the n=150 confirmation and the mechanism probe)

The ladder found the pitch coupling (a nose-down cruiser tilts its rear tag face
up-and-back; close-range decode p inside 6 m halves at the below-target/up-tilt geometry,
0.435 → 0.194, `logs/tag_realism_20260923/mech_probe.txt`) and the countermeasure is a
TARGET-side print decision we control: cant the rear tag's face DOWN by the cruise pitch.
Knob: `Scenario.tag_mount_pitch_deg` (default 0, byte-identical; geometry pinned by
`test_tag_mount_pitch_cants_the_rear_tag_and_cancels_cruise_pitch`).

- **Arms:** cant {0, 12°} × cells {nom, alt+3, alt+3+tilt10} × rungs {R2 expected,
  R5 worst}, seeds 0..99 (n=100), same port-arm config as §F.
- **Predictions:** (P6) cant-12 at alt3_t10/R2 recovers ≥ HALF of the realism-induced
  loss (pooled n=200: R0 64% → R2 ~43%; bar = back to ≥ 53%). (P7) nominal cells move
  ≤ ±5 points (level-view incidence 12° → 0° may even help slightly).
- **Adopt:** recommend the canted placard mount to the builder (the placard mount's
  index disc records the angle — docs/placard_mount.md) iff P6 holds and no cell
  degrades > 5 points. A NULL means the coupling is not mount-fixable and the honest
  height-band numbers stand as measured.

## RESULTS (flown 2026-09-23 night → 2026-09-24; verbatim tables; local copies in logs/tag_realism_20260923/)

### §F ladder, seeds 0..49 (n=50/cell)
```
         cell      rung     med     p90  <=0.35  <=1.0  med_dec
     rear_nom   R0_base   0.130   0.252     94%    96%       94
     rear_nom    R1_att   0.130   0.333     92%    96%       90
     rear_nom  R2_shake   0.148   0.247     94%    98%       90
     rear_nom    R3_vib   0.123   0.294     94%    96%       90
     rear_nom  R4_glare   0.128   0.294     94%    96%       90
     rear_nom  R5_worst   0.138   0.305     92%    94%       89

   rear_aim20   R0_base   0.149   4.341     80%    82%       79
   rear_aim20    R1_att   0.154   4.341     76%    80%       78
   rear_aim20  R2_shake   0.179   4.138     78%    80%       74
   rear_aim20    R3_vib   0.152   4.459     74%    76%       72
   rear_aim20  R4_glare   0.152   4.459     74%    76%       72
   rear_aim20  R5_worst   0.140   4.346     76%    78%       69

   rear_alt+3   R0_base   0.420   4.180     34%    74%       33
   rear_alt+3    R1_att   0.373   4.180     50%    76%       34
   rear_alt+3  R2_shake   0.401   4.074     46%    84%       38
   rear_alt+3    R3_vib   0.445   4.259     38%    68%       22
   rear_alt+3  R4_glare   0.458   4.259     36%    68%       24
   rear_alt+3  R5_worst   0.480   4.259     38%    64%       23

   rear_weave   R0_base   0.531   2.152     28%    78%       60
   rear_weave    R1_att   0.568   2.081     26%    76%       54
   rear_weave  R2_shake   0.498   1.591     34%    78%       58
   rear_weave    R3_vib   0.612   2.041     24%    72%       52
   rear_weave  R4_glare   0.630   2.315     26%    70%       52
   rear_weave  R5_worst   0.528   1.808     32%    76%       54

 rear_nom_t10   R0_base   0.111   0.260     94%    94%       85
 rear_nom_t10    R1_att   0.123   0.395     86%    94%       84
 rear_nom_t10  R2_shake   0.134   0.292     92%    92%       84
 rear_nom_t10    R3_vib   0.105   0.306     92%    94%       80
 rear_nom_t10  R4_glare   0.109   0.306     92%    94%       80
 rear_nom_t10  R5_worst   0.110   0.276     92%    94%       82

rear_alt3_t10   R0_base   0.294   1.003     70%    90%       60
rear_alt3_t10    R1_att   0.275   4.074     64%    86%       50
rear_alt3_t10  R2_shake   0.434   4.181     32%    80%       42
rear_alt3_t10    R3_vib   0.376   4.180     44%    84%       50
rear_alt3_t10  R4_glare   0.377   4.180     42%    84%       50
rear_alt3_t10  R5_worst   0.362   3.675     44%    86%       47

      cam_nom   R0_base   0.132   1.042     86%    90%       98
      cam_nom    R1_att   0.132   1.042     86%    90%       98
      cam_nom  R2_shake   0.132   1.042     86%    90%       98
      cam_nom    R3_vib   0.162   0.780     86%    90%       86
      cam_nom  R4_glare   0.151   0.780     86%    90%       86
      cam_nom  R5_worst   0.140   0.717     88%    90%       88

```

### Confirmation, DISJOINT seeds 50..199 (n=150/cell)
```
         cell      rung     med  <=0.35  <=1.0  med_dec
     rear_nom   R0_base   0.120     90%    93%       92
     rear_nom    R1_att   0.119     87%    92%       90
     rear_nom  R2_shake   0.128     90%    97%       90
     rear_nom  R5_worst   0.122     91%    95%       88

   rear_aim20   R0_base   0.164     76%    82%       78
   rear_aim20    R1_att   0.157     76%    80%       74
   rear_aim20  R2_shake   0.140     75%    81%       72
   rear_aim20  R5_worst   0.199     73%    76%       66

   rear_alt+3   R0_base   0.379     46%    77%       43
   rear_alt+3    R1_att   0.418     40%    69%       32
   rear_alt+3  R2_shake   0.398     39%    65%       28
   rear_alt+3  R5_worst   0.497     34%    66%       23

rear_alt3_t10   R0_base   0.282     62%    89%       56
rear_alt3_t10    R1_att   0.352     49%    81%       45
rear_alt3_t10  R2_shake   0.379     46%    77%       48
rear_alt3_t10  R5_worst   0.351     50%    81%       44

```

### Mechanism probe (20 seeds/rung, per-frame FrameReports; p@<6m = median decode p inside 6 m)
```
== rear_alt3_t10 ==
     rung  frames  inFOV  att%  dec%  inc_med  inc_p90  blur_med   p_med   p@<6m
  R0_base   19100  11531   27%   10%      8.4    155.6     0.008   0.000   0.435
   R1_att   19100  12624   23%    7%     17.0    157.6     0.007   0.000   0.227
 R2_shake   19100  11827   24%    6%     17.2    157.4     0.009   0.000   0.194

== rear_alt+3_t0 ==
     rung  frames  inFOV  att%  dec%  inc_med  inc_p90  blur_med   p_med   p@<6m
  R0_base   19100  11921   19%    5%      7.3    155.6     0.007   0.000   0.317
   R1_att   19100  12958   17%    4%     17.1    157.5     0.006   0.000   0.207
 R2_shake   19100  13631   20%    6%     13.9    157.0     0.007   0.000   0.279

== rear_nom ==
     rung  frames  inFOV  att%  dec%  inc_med  inc_p90  blur_med   p_med   p@<6m
  R0_base   19100  16599   25%   11%      1.7    156.9     0.006   0.000   0.617
   R1_att   19100  17627   24%   10%     13.0    152.2     0.005   0.000   0.606
 R2_shake   19100  15882   26%   11%     13.2    152.6     0.007   0.000   0.585

```

### §G canted-mount A/B, seeds 0..99 (n=100/cell-arm)
```
         cell      rung  cant     med  <=0.35  <=1.0  med_dec
     rear_nom  R2_shake     0   0.143     91%    98%       90
     rear_nom  R2_shake    12   0.131     91%    98%       90
     rear_nom  R5_worst     0   0.135     91%    95%       87
     rear_nom  R5_worst    12   0.140     88%    96%       90

   rear_alt+3  R2_shake     0   0.401     40%    76%       28
   rear_alt+3  R2_shake    12   0.375     40%    75%       39
   rear_alt+3  R5_worst     0   0.508     30%    64%       24
   rear_alt+3  R5_worst    12   0.421     40%    72%       32

rear_alt3_t10  R2_shake     0   0.406     41%    76%       42
rear_alt3_t10  R2_shake    12   0.312     60%    89%       52
rear_alt3_t10  R5_worst     0   0.358     48%    85%       47
rear_alt3_t10  R5_worst    12   0.282     64%    88%       55

```

### Registered verdicts

- **R0 regression check: PASS** — reproduces the ADR-0112 anchors exactly (alt+3 tilt-0
  34%, tilt-10 70%, seeds 0..49).
- **P1 (rear nominal drops 5–15):** NOT confirmed — nominal is an honest NULL
  (pooled n=200: 91.0% → 87.8–91.3% across rungs). The tag model was already adequate
  at nominal; per the registered criterion, no contract rewrite of nominal numbers and
  no algorithm refinement warranted.
- **P2 (shake/vib < 3 pts):** confirmed at nominal.
- **P3 (glare = fat tail, 3–10 pts):** smaller than predicted (≤ ~2 pts at nominal;
  glare is bimodal and the vulnerable cells were already decode-starved).
- **P4 (all-on rear < 55%):** nominal stays ≥ 88% — the builder's bet did not land on
  the flying config.
- **P5 (+10° alt+3 win survives, ≥ +15 pts): FAILED** — pooled n=200 the naked
  bracket's margin is +2..+14 by rung (upright-tag +21). The n=50 "attitude helps
  alt+3 tilt-0" (+16) REVERSED on disjoint seeds (−6): quote pooled numbers only.
- **P6 (cant recovers ≥ half at alt3_t10/R2, bar 53%): PASS** — 41% → 60% (R5: 48% →
  64%). **P7 (nominal ≤ ±5): PASS** (−3 worst). **ADOPT**: recommend bracket +10° AND
  placard canted 12° down as a PAIR (ADR-0114); with both, the bracket's alt+3 margin
  over no-bracket is restored (+20/+24).
