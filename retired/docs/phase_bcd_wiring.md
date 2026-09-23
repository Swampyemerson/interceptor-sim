# Wiring note — Phases B/C/D intercept-accuracy levers (pre-coded, default-OFF)

> **Status: PRE-CODED, OPT-IN, NOT SIM-VALIDATED (2026-07-21).** These three
> modules implement Phases B, C, D of `docs/intercept_accuracy_levers.md` as
> self-contained, flag-gated `flight/` core modules with unit tests. They are
> **built ahead of the sim A/B on purpose**: Phase A (loft-then-dive pointing)
> must land and validate first — every lever here is "only once framed" (levers
> doc). This note is the exact hook-up for the head/builder to A/B each behind a
> flag in `scripts/m4_intercept.py`. **Nothing here edits `m4_intercept.py`,
> `guidance.py`, or `estimator.py`** — they compose as wrappers.

| Module | Phase | Flag (proposed) | One line |
|---|---|---|---|
| `flight/fov_guidance.py` | B — hold the residual | `--fov-hold` | Vertical FOV-hold: IBVS pixel-centering + look-angle barrier + a pitch-headroom accel cap, on the pitch-coupling axis. |
| `flight/range_fusion.py` | C — sharpen ZEM/t_go | `--range-fusion` | bbox size-range prior `R=fx·W/w`, closing rate, and τ, fused as a coarse prior into the range channel. |
| `flight/terminal_coast.py` | D — endgame | `--terminal-coast` | τ-armed freeze-the-LOS-rate coast through the sub-1 m camera blind window. |

All three are **byte-identical no-ops when their flag is off** (and inert on
coasting/no-detection ticks even when on), so they can land dark and be A/B'd
one at a time.

---

## Honesty status (read before wiring — each adds a pixel→loop path)

Every module reads **only** camera pixels/bearing + own-state EKF (attitude,
velocity, sim clock). No `gt_*`, no datalink. But each introduces or rides a
**new pixel-into-guidance path**, which per the CLAUDE.md honesty boundary
**re-earns the numeric no-cheat audit** the same way any new guidance path does:

- **B** steers the vertical channel on the target's **image position** (`eps_v`
  from `meas_xyz` / the undistorted pixel). New pixel→command path.
- **C** steers the **range channel** on the detector **bbox width**. New
  pixel→range path. `W ≈ 0.3 m` is a *pre-briefed known constant* (same status
  as the AprilTag side length), not a measurement of this target.
- **D** adds no new sensor, but its **τ arm signal rides C's bbox path**, so the
  B+C+D composition is audited as one.

Run the audit (grep the guidance path for `gt_`, confirm `meas_xyz`/`bearing`/
`w_px` are the only exogenous inputs, confirm every `t` is a sim-clock value)
**before any A/B number is quoted** — same drill as `derotate_bearing_lambda`.

---

## Phase B — `flight/fov_guidance.py` behind `--fov-hold`

**What it is.** The wedge sets the *static* camera-elevation bias; this absorbs
the *dynamic* residual of the +40°→−35° dash pitch swing that a fixed wedge
cannot track. It is formulated on the **vertical** (pitch-coupling) axis — the
measured failure is vertical (target parks at the frame **top**), so the
horizontal law `a=N·Vc·λ̇` is left completely untouched.

**Three pieces** (`augment_pronav` returns all in a `FovHoldCommand`):
1. `ibvs_centering_accel` — gentle always-on `a_up = −k_center·Vc·eps_v`
   (position-only image feedback; for small angles `eps_v == (v−cy)/fy`).
2. `fov_edge_barrier_accel` — quadratic soft barrier, off inside `engage_frac`
   of the usable half-FOV, hard push-back near the edge.
3. `pitch_headroom_accel_cap` — the **look-angle CONSTRAINT**: the max forward
   accel whose steady quad pitch `θ=−atan(a/g)` keeps the target in-frame given
   where it sits *now*. This is the hook that co-sizes the accel-capped dash
   with the wedge (Phase A/B joint sizing).

**Wire-up in `m4_intercept.py` (ENGAGE, ~line 2812 where `a_cmd` is formed).**
Construct the config + imports ONCE (hoist out of the per-tick loop — review
finding 7), e.g. next to the filter construction ~line 2187:
```python
if args.fov_hold:
    from flight.fov_guidance import augment_pronav, FovHoldConfig, half_fov_from_intrinsics
    fov_cfg = FovHoldConfig(half_vfov_rad=half_fov_from_intrinsics(fy, height))
    FOV_VERT_MAX = args.fov_vert_max     # dedicated clamp, NOT V_VERT_MAX (below)
    v_up = 0.0
```
then per ENGAGE tick:
```python
# after: a_cmd = N_PRONAV * vc * lambda_dot_hat  (unchanged horizontal law)
if args.fov_hold and meas is not None and detected:
    pitch_rad = math.radians(state.pitch_deg) if state.pitch_deg is not None else None
    fov = augment_pronav(a_cmd, meas.meas_xyz, vc, fov_cfg, pitch_rad=pitch_rad)
    # vertical: integrate a_up into a v_up setpoint (NED: v_down_extra = -v_up).
    # CLAMP with a DEDICATED FOV_VERT_MAX, *not* V_VERT_MAX=0.5 m/s (review
    # finding 2). V_VERT_MAX is the altitude-hold TRIM limit; at 0.5 m/s the
    # barrier physically cannot re-center a 0.45 rad offset inside ~1.6 s
    # (the module's own closed-loop test needs ~1 m of climb) -> a gutted
    # lever and a false-negative A/B. Size FOV_VERT_MAX from the vertical
    # authority budget (a_up_max * a settling time), e.g. ~2-3 m/s.
    v_up = _clamp(v_up + fov.a_up_m_s2 * dt, -FOV_VERT_MAX, FOV_VERT_MAX)
    # v_down currently comes from the altitude-hold KP_ALT block (~line 3403);
    # under --fov-hold ADD -v_up to it (FOV-hold owns the vertical channel in
    # the terminal, altitude-hold becomes the trim). Log fov.eps_v_rad,
    # fov.barrier_active, fov.a_fwd_cap_m_s2 to new CSV columns.
    # a_fwd_cap: min() it into the dash/closing-speed planner's own accel cap.
```
- `pitch_rad` uses `state.pitch_deg`, which m4 ALREADY maintains (m4:1372,
  read ~line 2470) — no need to re-derive Euler from the quaternion.
- Get `fy`,`height` from `get_camera_intrinsics(...)` (already called ~line 4055).
- Add `--fov-vert-max` (default e.g. 2.5) so the vertical authority is a
  tunable A/B knob, not a magic constant.
- **A/B:** wedge-only PN vs `--fov-hold`; metric = **fraction-of-ticks
  target-in-frame across 8–12 m** (vs measured ~75% out) + miss. Paired seeds
  n≥8 (statistics-before-verdicts rule).

**Real-lens note:** in sim, `meas.meas_xyz` is already an ideal-pinhole ray.
On the hardware wide M12 lens, feed the **undistorted** ray
(`flight.camera.CameraModel.pixel_to_ray(u,v)`) as `meas_xyz`, or use
`pixel_v_to_look_angle` on the undistorted `v` — not the raw distorted pixel.

---

## Phase C — `flight/range_fusion.py` behind `--range-fusion`

**What it is.** `R = fx·W/w_px` from the detector bbox width (`W≈0.3 m` known),
run through a width-channel α-β/Kalata filter so range, closing rate
(`Rdot=−fx·W·ẇ/w²`) and **τ=w/ẇ** fall out **without differentiating range**.
Folded into the *existing* range filter as a **coarse prior** through the
filter's audited `correct(meas,t,gain_scale=w)` port — the same inverse-variance
mechanization `FusedTrack.update_cue` uses for the cue range. **Honest noise:**
`σ_R/R = σ_w/w` RSS'd with a bank-foreshorten fraction — a ±1 px error on a few-px
box is ~10–30% range error, hence *prior, never primary*.

**Wire-up in `m4_intercept.py`:**
```python
# construct once alongside range_filter (~line 2187):
if args.range_fusion:
    from flight.range_fusion import SizeRangeEstimator
    size_est = SizeRangeEstimator(fx)          # W defaults to 0.30 m

# every control tick, mirror the range_filter.predict cadence (~line 2480):
if args.range_fusion:
    size_est.predict(dt)

# feed the WIDTH channel only on a fresh detection with a bbox width
# (in the `if fresh:` block, alongside range_filter.correct ~line 2607):
if args.range_fusion and fresh and box_w_px is not None:
    size_est.correct(box_w_px, tick_start)     # SIM time

# fuse EVERY ENGAGE tick (NOT only in the fresh block) -- review finding 1.
# fuse_into's min_dt_s guard makes it a NO-OP on the tick the camera just
# corrected range_filter (folding a coarse prior at the SAME sim time would
# collapse dt_since to 1e-3 and inject tens of m/s of Rdot into the DEFAULT
# fixed-gain range filter -> vc=-Rdot -> a_cmd blows up). The prior's value
# is exactly the COASTING ticks (sparse in-flight detection), which the guard
# lets through:
if args.range_fusion and phase == "ENGAGE":
    size_est.fuse_into(range_filter, tick_start)   # coarse prior, gain-scaled + dt-guarded
```
- **`box_w_px` is the missing input on the AprilTag baseline.** The `Measurement`
  dataclass (`m3_static_intercept.py:189`) carries no bbox width today. Either
  (a) add a defaulted `box_w_px: Optional[float] = None` field (defaulted →
  every existing 6-positional construction stays byte-identical, exactly how
  `source` was added, ADR-0058), populated by the markerless detector
  (`nn_seeker.py` has `box_xywh`, line ~117) and by an AprilTag corner-span for
  the tag baseline; or (b) pass the detector box straight through. This is the
  one small non-`flight/` edit Phase C needs — flagged so it is visible.
- `SizeRangeEstimator.tau_s` is **also the Phase D arm signal** — expose it.
- **Disclose in the A/B writeup (review finding 6):** the prior shares the
  ONE range α-β filter, so each fused correction shortens `dt_since` for the
  *next* camera correction (the β/dt term) — it subtly rewires the primary's
  rate response even at small weight. This is intrinsic to composing one
  filter and is the SAME accepted property as the cue-range `update_cue`
  precedent (m4:1214-1218); `fusion_weight` mirrors that mechanization exactly.
- **A/B (guidance_lab first, then Gazebo):** size-R prior vs geometry-only, with
  W-foreshortening injected as noise; metric = miss + t_go error.

---

## Phase D — `flight/terminal_coast.py` behind `--terminal-coast`

**What it is.** On a FOV-edge/track-loss flag **inside the τ-armed endgame**,
freeze the last-good LOS-rate + Vc and propagate the PN command open-loop on the
own-state EKF for the measured ~0.1–0.2 s blind window. Terminal LOS is
485–1870°/s, so a stale *command/angle* diverges within a frame — freezing the
**rate** and continuing to integrate the constant `N·Vc·λ̇` holds the commanded
ZEM correction through impact. **Armed by τ (time-to-contact from bbox growth),
never wall-clock**; duration bounded by `max_coast_s` of **sim** time.

**Name-collision warning (do not conflate):**
- `--terminal-coast-gate` (existing) = a **phantom-rejection range gate**
  (ADR-0057). Unrelated.
- The existing inline "terminal coast" (`frozen_vworld` at
  `TERMINAL_FREEZE_RANGE_M=2.0`, m4:3392-3401) freezes the **velocity vector**
  on a RANGE trigger (r_hat < 2 m, live track) and drops further correction.
  **That block IS the Phase D A/B baseline** — this module is the
  freeze-*and-propagate* alternative, triggered on **track loss**. A/B = the
  two against each other on the same flights, compare CPA.

**IMPORTANT — the two must be mutually exclusive (review finding 3).** The
existing `frozen_vworld` block triggers on range and this module triggers on
loss, so both can fire in sequence (freeze the velocity at 2 m, then a loss
un-freezes it as this coast takes over) — an incoherent hybrid that measures
neither arm. So **`--terminal-coast` MUST bypass the `frozen_vworld` block**
(guard it `if frozen_vworld is None and not args.terminal_coast:`), making the
A/B a clean either/or. Log which arm produced each terminal command.

**Wire-up in `m4_intercept.py`:**
```python
# construct at ENGAGE start:
if args.terminal_coast:
    from flight.terminal_coast import TerminalCoast
    coast = TerminalCoast(n_gain=N_PRONAV)     # arm_tau_s/max_coast_s/tau_stale_s tunable

# every ENGAGE tick WITH a live track (after λ/R filters + size_est update):
if args.terminal_coast:
    coast.update_track(lambda_hat, lambda_dot_hat, vc, tick_start,
                       r_hat_m=r_hat, tau_s=(size_est.tau_s if args.range_fusion else None))

# on a track-loss tick (INSTEAD of the frozen_vworld branch, which is bypassed):
if args.terminal_coast and coast.on_track_loss(tick_start):
    cc = coast.command(tick_start)
    if not cc.expired:
        u = (math.cos(cc.lambda_rad), math.sin(cc.lambda_rad))
        p = (-u[1], u[0])
        v_perp = _clamp(v_perp + cc.a_lat_m_s2 * dt, -V_PERP_MAX, V_PERP_MAX)
        # Along-LOS speed: keep the LIVE commanded closing speed (a COMMANDED
        # constant, not a sensed quantity that dies with the camera) so the
        # coast tests ONLY the freeze-the-LOS-rate concept, with no unrelated
        # closing-speed STEP at coast entry (review finding 3). cc.vc_m_s is
        # the frozen -Rdot estimate; use it for the frozen a_lat above, not
        # for the along-LOS term.
        v_close_coast = compute_v_close(cc.r_m if cc.r_m is not None else r_hat, coast_fpv)
        vh0 = v_close_coast * u[0] + v_perp * p[0]
        vh1 = v_close_coast * u[1] + v_perp * p[1]
    # else: expired -> fall through to existing hold/breakoff policy.
```
- **τ dependency:** `--terminal-coast` is only *armable* when it gets a τ. With
  `--range-fusion` off (no bbox stream) `tau_s=None` → the coast **never arms**
  and is inert (safe default). So the natural A/B order is C then D, or D as a
  refinement of the existing `frozen_vworld` block using its own range-derived τ.
- **τ is STICKY (review finding 4):** a single noisy `wdot` dip → `tau_s=None`
  on the final live tick does NOT disarm — the last valid τ keeps the arm alive
  for `tau_stale_s` (0.30 s default), long enough to cover the loss-detection
  gap, then goes stale so an old opening geometry can't hold it armed.
- Coast is a **sim-time** window (`max_coast_s`, default 0.25 s > measured
  0.1–0.2 s); an RTF sag cannot stretch it (ADR-0009).
- **A/B:** on flights with a real terminal track, freeze-and-propagate (this) vs
  hold-last-command (`frozen_vworld`); metric = **CPA** (the ram bar, 0.35 m, ADR-0084
  contact, not the 2.5 m Pk proxy).

---

## Test + gate

```
.venv/bin/python -m pytest flight/tests/test_fov_guidance.py \
    flight/tests/test_range_fusion.py flight/tests/test_terminal_coast.py -q
# 32 passed
```
Also runnable standalone (exit 0/1) like the other `flight/tests/`:
`.venv/bin/python flight/tests/test_fov_guidance.py`. They join the main offline
suite automatically (`scripts/run_tests.sh` globs `flight/tests/`). All math is
pure Python — **no sim, no Gazebo** — exercising each lever on synthetic inputs:
size-range recovers a known R, τ counts down ~1 s/s, the FOV term reduces a
pixel offset and holds the target in-FOV over a closing run, and the coast
propagation matches a constant-velocity LOS.
