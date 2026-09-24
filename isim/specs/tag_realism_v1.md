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
(idealized control, prediction ≈ flat across rungs by design). (they're truth-model, not a tuned lever) —
what's decided is the HEADLINE: if all-on EXPECTED drops the standing numbers by > 5 points,
the contract's terminal-stage numbers get rewritten to the realism-on figures and the old grid
is regraded a best-case upper bound. A NULL (no material drop) means the tag model was already
adequate: report that straight, do NOT invent a degradation, and the algorithm-refinement half
of the night is judged unnecessary — the builder's "if I was right" was conditional.
