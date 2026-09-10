"""flight.guidance -- the coded-dash aim and (later) the pro-nav terminal law.

The real interceptor's operating model (memory [[real-build-pivot]], ADR-0076
add #18): an OPEN-LOOP coded dash pointed the right way, then a CAMERA-ONLY
pro-nav terminal. `collision_lead_heading` computes the "right way" -- a
pre-flight constant from the KNOWN launch kinematics (where the target is and
where it's heading), exactly what a human or a ground cue programs at launch. It
reads no live sensor and no ground truth.
"""

import math

from flight.geometry import wrap_pi


def collision_lead_heading(target_pos, target_vel, dash_speed, origin=(0.0, 0.0)):
    """Pre-flight COLLISION-LEAD azimuth (deg) for the coded open-loop dash.

    Solve the constant-speed intercept triangle: launching from `origin` at
    `dash_speed`, find the smallest time t>0 at which the interceptor reaches the
    target -- which starts at `target_pos` and moves at constant `target_vel` --
    then aim at that lead/collision point. Frame: horizontal (x=east, y=north)
    per ADR-0013; the returned compass azimuth is atan2(east, north) in DEGREES
    (0=north, 90=east).

    Why LEADING is essential (ADR-0076 add #18b): a naive aim at the target's
    STALE initial position points the nose camera where the target WAS, so a fast
    crossing target leaves frame before the seeker can acquire -- measured as an
    l2r "no acquire" abort. Aiming at the lead point keeps the target near the
    camera boresight through the dash so the terminal can lock.

    This is a PURE, PRE-FLIGHT computation from KNOWN launch kinematics -- it
    reads no live sensor / ground truth (honesty boundary intact).

    Args:
        target_pos: (x, y) target position at launch, meters (east, north).
        target_vel: (vx, vy) target velocity, m/s (east, north).
        dash_speed: interceptor dash speed, m/s (> 0).
        origin: (x, y) interceptor launch position, meters (default world origin).

    Returns:
        (heading_deg, t_lead): the dash azimuth in degrees, and the intercept
        time t_lead in seconds. If the target cannot be caught by a pure lead
        (no positive real root -- e.g. a faster receding target), falls back to
        aiming at the target's INITIAL position and returns t_lead=None.
    """
    px, py = float(target_pos[0]), float(target_pos[1])
    vx, vy = float(target_vel[0]), float(target_vel[1])
    ox, oy = float(origin[0]), float(origin[1])
    vi = float(dash_speed)
    # R0 = target initial position relative to launch origin.
    r0x, r0y = px - ox, py - oy
    # |R0 + Vt*t| = Vi*t  ->  a t^2 + b t + c = 0
    a = vx * vx + vy * vy - vi * vi
    b = 2.0 * (r0x * vx + r0y * vy)
    c = r0x * r0x + r0y * r0y
    t_lead = None
    if abs(a) < 1e-6:
        # target speed == dash speed: linear b t + c = 0
        if abs(b) > 1e-9:
            t = -c / b
            t_lead = t if t > 1e-3 else None
    else:
        disc = b * b - 4.0 * a * c
        if disc >= 0.0:
            sq = math.sqrt(disc)
            roots = [(-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)]
            pos = [r for r in roots if r > 1e-3]
            t_lead = min(pos) if pos else None
    if t_lead is not None:
        lx, ly = px + vx * t_lead, py + vy * t_lead  # lead / collision point
    else:
        lx, ly = px, py  # uncatchable by pure lead -> aim at initial position
    heading_deg = math.degrees(math.atan2(lx - ox, ly - oy))  # atan2(east, north)
    return heading_deg, t_lead


def closing_speed(rdot_hat, floor):
    """Closing speed Vc for the pro-nav command: -Rdot once it exceeds `floor`,
    else the floor. Rationale (m4 dev-run T012356Z): the range-rate estimate
    starts ~0 from hover and lags under sparse detection, so without a floor the
    pro-nav lead `N*Vc*lambda_dot` builds at a fraction of strength early and the
    target's LOS walks out of frame before the lead catches up. Measured -Rdot
    takes over the moment it exceeds the floor. `rdot_hat` may be None (range
    filter not yet initialized) -> return the floor."""
    if rdot_hat is None:
        return floor
    return max(floor, -rdot_hat)


def pronav_lateral_accel(n_gain, vc, los_rate):
    """Proportional-navigation lateral acceleration command: a = N * Vc * lambda_dot,
    where N is the navigation gain (typ. 3-5), Vc the closing speed, and lambda_dot
    the inertial LOS azimuth rate. The classic missile-guidance law: command
    acceleration proportional to the line-of-sight rotation rate, which drives
    lambda_dot -> 0 (a collision course). `los_rate` may be None (LOS filter not
    initialized) -> 0.0."""
    if los_rate is None:
        return 0.0
    return n_gain * vc * los_rate


# --- POINTING levers for the coded dash (Phase A, docs/intercept_accuracy_levers.md).
# Both are PURE own-state trajectory shaping -- no camera, no gt (honesty boundary
# intact) -- and default OFF (byte-identical). The wall they attack: to dash forward
# the quad pitches nose-DOWN by theta = arctan(a_forward / g), tipping the body-fixed
# camera down so a co-altitude target sits at/above the frame-TOP edge (detector ~100%
# static 8-22 m, ~0.8% in flight; ADR-0076 add #18k). The analytical A/B is
# scripts/experiments/loft_dive/inframe_ab.py. -----------------------------------

GRAVITY_MS2 = 9.80665


def dash_forward_speed(dash_speed, accel_cap_ms2, t_since_dash_start):
    """Forward speed setpoint for the ACCEL-CAPPED constant-pitch dash.

    The plain coded dash commands `dash_speed` as a STEP from t=0, so PX4 pitches
    to whatever accel MPC_ACC_HOR_MAX allows -- ~40 deg nose-down at full accel,
    which points the fixed camera over the co-altitude target. Ramping the
    commanded speed at `accel_cap_ms2` instead bounds the demanded forward accel,
    so the body pitch holds ONE known value theta ~ arctan(accel_cap/g) for the
    whole run-in -- the value the fixed wedge can then be sized to (instead of the
    +40 -> -35 deg swing a fixed wedge cannot track). Cost: a gentler accel reaches
    `dash_speed` later (or never within a ~2 s engagement), so closing speed / t_go
    suffer -- the documented tradeoff (intercept_accuracy_levers.md).

    accel_cap_ms2 None or <= 0 -> return `dash_speed` unchanged (byte-identical
    default). Otherwise the linearly-ramped speed, clamped to `dash_speed`.
    `t_since_dash_start` is SIM-clock seconds since CODED_DASH entry (never wall
    time -- the RTF-sag rule)."""
    if accel_cap_ms2 is None or accel_cap_ms2 <= 0.0:
        return dash_speed
    return min(dash_speed, accel_cap_ms2 * max(0.0, t_since_dash_start))


# --- ACCEL-AWARE COLLISION LEAD (docs/flight_plan_candidates.md Sec 1.3 + the
#     G20dash RESULT).  Default OFF at the caller -> byte-identical. ----------
#
# THE BUG IT FIXES (MEASURED).  `collision_lead_heading` solves a CONSTANT-SPEED
# triangle: it assumes the interceptor covers `dash_speed * t` metres by time t.
# The real coded dash starts AT REST and ramps (PX4's MPC_ACC_HOR_MAX, or the
# commanded ramp of `dash_forward_speed` under --dash-accel-cap), so by time t it
# has covered only the INTEGRAL of that ramp -- far less.  Needing longer to get
# there, the true intercept happens LATER, so the true collision point is FURTHER
# along the target's track: the constant-speed solve under-leads.
#
# On the flown arms that shortfall was absorbed by the hand-tuned
# `--dash-crossing-bias-deg`, which therefore is not a perception constant at all
# but an ACCELERATION-dependent kinematic one -- change the dash accel without
# re-tuning it and you inject a systematic aim error.  MEASURED consequence
# (docs/flight_plan_candidates.md RESULTS, seed 123, n=8 paired, dash-only):
# re-sizing the bias 30 -> 20 deg for the baseline accel halved the open-loop
# ballistic miss, combined median 1.37 -> 0.75 m (7/8 paired better).
#
# WHAT THIS FUNCTION DOES.  It solves the SAME triangle with the accelerating
# side: |R0 + Vt*t| = s(t), where s(t) is the exact integral of
# `dash_forward_speed` -- so the aim is automatically right for whatever accel
# the vehicle actually flies, with no hand-tuned bias.  At the fitted baseline
# accel (10 m/s^2) it reproduces +20.3 deg of crossing bias on the canonical
# geometries -- i.e. it DERIVES the empirically-confirmed constant (test
# `test_accel_lead_reproduces_the_confirmed_20deg_bias`).
#
# HONESTY.  Inputs are the SAME CLASS the constant-speed lead already uses: the
# operator's pre-flight target kinematics estimate plus one own-vehicle spec (the
# dash acceleration -- a commanded cap, or a number fitted OFFLINE from past
# flights by scripts/experiments/flight_plans/dash_cpa_model.py --validate,
# exactly as camera intrinsics are).  Nothing here reads a live sensor or any
# gt_* at inference; it runs ONCE, before launch.


def dash_ramp_distance(dash_speed, accel_ms2, t):
    """Along-heading distance (m) covered by the ramped dash by sim-time `t`.

    The EXACT integral of `dash_forward_speed(dash_speed, accel_ms2, .)` from 0
    to t (pinned against numerical integration of that function in
    flight/tests/test_guidance.py):

        s(t) = 0.5*a*t^2                    while a*t <= dash_speed
             = dash_speed*t - dash_speed^2/(2a)   after the clamp

    `accel_ms2` None or <= 0 means "no ramp" (the instantaneous-speed idealisation
    the constant-speed lead assumes) -> dash_speed * t.  t <= 0 -> 0.0.
    `t` is SIM-clock seconds (never wall -- the RTF-sag rule)."""
    t = float(t)
    if t <= 0.0:
        return 0.0
    if accel_ms2 is None or accel_ms2 <= 0.0:
        return float(dash_speed) * t
    v = float(dash_speed)
    a = float(accel_ms2)
    t_clamp = v / a
    if t <= t_clamp:
        return 0.5 * a * t * t
    return v * t - 0.5 * v * t_clamp          # = v*t - v^2/(2a)


def collision_lead_heading_accel(target_pos, target_vel, dash_speed, accel_ms2,
                                 origin=(0.0, 0.0), t_max_s=30.0,
                                 scan_steps=256, bisect_iters=64):
    """ACCEL-AWARE pre-flight collision-lead azimuth (deg) for the coded dash.

    Same contract, frame and return signature as `collision_lead_heading`
    (compass azimuth atan2(east, north) in degrees, plus the intercept time) --
    but the "where can I be at time t" side of the triangle uses the ACCELERATING
    profile `dash_ramp_distance` instead of `dash_speed * t`.

    METHOD (deterministic, bounded).  Define
        f(t) = s(t) - |R0 + Vt*t|        (s = dash_ramp_distance)
    f(0) = -|R0| < 0; the wanted answer is the SMALLEST t > 0 with f(t) = 0.  s is
    quadratic-then-linear and |R0 + Vt*t| grows at most linearly, so:

      1. UPPER BOUND (closed form).  Past the clamp s(t) = v*t - v^2/(2a), and
         |R0 + Vt*t| <= |R0| + |Vt|*t, so for |Vt| < v every root satisfies
             t <= ( |R0| + v^2/(2a) ) / ( v - |Vt| ).
         (If |Vt| >= v -- a target at least as fast as the dash -- no such bound
         exists and the search horizon `t_max_s` is used instead.)
      2. SCAN for the FIRST sign change of f on [0, t_hi] with `scan_steps`
         equal steps (256 -> ~0.03 s resolution on the canonical ~8 s horizon).
      3. BISECT that bracket `bisect_iters` times (64 -> machine precision).

    Worst case scan_steps + bisect_iters = 320 evaluations, no iteration to
    convergence, no randomness: the same inputs always give the same aim.

    LIMIT: `accel_ms2` None or <= 0 (or a degenerate geometry -- target on top of
    the launch point, or a non-positive dash speed) DELEGATES to
    `collision_lead_heading`, so the infinite-accel case is EXACTLY the
    constant-speed answer, bit for bit.  A large finite accel converges to it
    (1e8 m/s^2 -> within 1e-6 deg; test `test_accel_lead_infinite_accel_limit`).

    FALLBACK: if no root exists inside the horizon (an uncatchable target), aim at
    the target's INITIAL position and return t_lead=None -- identical to the
    constant-speed function's fallback contract.

    Args:
        target_pos: (x, y) target position at launch, m (east, north).
        target_vel: (vx, vy) target velocity, m/s (east, north).
        dash_speed: dash speed the ramp clamps to, m/s (> 0).
        accel_ms2: the forward acceleration the dash will ACTUALLY fly, m/s^2 --
            the commanded --dash-accel-cap when capped, else the vehicle's
            measured effective accel (see the caller in scripts/m4_intercept.py).
        origin: (x, y) launch position, m (default world origin).
        t_max_s: search horizon (s) used only when the closed-form bound does not
            exist (target at least as fast as the dash).
        scan_steps / bisect_iters: the bounded-iteration knobs above.

    Returns:
        (heading_deg, t_lead) -- t_lead None if uncatchable within the horizon.
    """
    px, py = float(target_pos[0]), float(target_pos[1])
    vx, vy = float(target_vel[0]), float(target_vel[1])
    ox, oy = float(origin[0]), float(origin[1])
    v = float(dash_speed)
    r0x, r0y = px - ox, py - oy
    r0 = math.hypot(r0x, r0y)
    if accel_ms2 is None or accel_ms2 <= 0.0 or v <= 0.0 or r0 <= 1e-9:
        # No ramp to model (or a degenerate geometry): the constant-speed
        # contract IS the answer -- delegate so the two agree exactly.
        return collision_lead_heading(target_pos, target_vel, dash_speed, origin)
    a = float(accel_ms2)

    def f(t):
        return dash_ramp_distance(v, a, t) - math.hypot(r0x + vx * t, r0y + vy * t)

    vt = math.hypot(vx, vy)
    if vt < v:
        t_hi = min(float(t_max_s), max((r0 + v * v / (2.0 * a)) / (v - vt), 1e-3))
    else:
        t_hi = float(t_max_s)
    dt = t_hi / int(scan_steps)
    t_lead = None
    prev_t, prev_f = 0.0, f(0.0)
    for i in range(1, int(scan_steps) + 1):
        t = i * dt
        ft = f(t)
        if prev_f < 0.0 <= ft:
            lo, hi = prev_t, t                     # f(lo) < 0 <= f(hi)
            for _ in range(int(bisect_iters)):
                mid = 0.5 * (lo + hi)
                if mid <= lo or mid >= hi:         # bracket at float resolution
                    break
                if f(mid) < 0.0:
                    lo = mid
                else:
                    hi = mid
            t_lead = 0.5 * (lo + hi)
            break
        prev_t, prev_f = t, ft
    if t_lead is not None:
        lx, ly = px + vx * t_lead, py + vy * t_lead   # lead / collision point
    else:
        lx, ly = px, py            # uncatchable -> aim at the initial position
    return math.degrees(math.atan2(lx - ox, ly - oy)), t_lead


def dash_loft_alt_ref(base_alt_m, loft_m, t_since_dash_start, dive_dur_s):
    """Altitude REFERENCE for the loft-then-dive dash: hold +loft at dash entry,
    then a raised-cosine DIVE to co-altitude over `dive_dur_s`.

    The interceptor is expected to already be lofted to `base_alt_m + loft_m` when
    the dash begins (climbed during takeoff/positioning -- see the caller; at the
    stock V_VERT_MAX 0.5 m/s it CANNOT climb 2-4 m inside the ~2 s dash, so the
    climb must precede it). Diving from +loft onto the co-altitude target puts the
    interceptor ABOVE it, so the LOS points DOWN -- toward where the nose-down dash
    pitch already aims the camera -- restoring in-frame detection through the
    acquisition band. Raised cosine: reference = base + loft at t=0, = base at
    t=dive_dur_s, smooth (bounded vertical rate/accel) at both ends.

    OVERSHOOT WARNING (intercept_accuracy_levers.md): too much loft drives the LOS
    depression past the nose-down pitch and the target exits the frame BOTTOM. Size
    `loft_m` jointly with the wedge + accel-cap, per the analytical sweep.

    loft_m <= 0 -> return `base_alt_m` unchanged (byte-identical flat dash).
    `t_since_dash_start` is SIM-clock seconds."""
    if loft_m <= 0.0 or dive_dur_s <= 0.0:
        return base_alt_m
    frac = min(max(t_since_dash_start / dive_dur_s, 0.0), 1.0)
    return base_alt_m + loft_m * 0.5 * (1.0 + math.cos(math.pi * frac))


# --- TERMINAL LOS BEARING-BIAS COMPENSATION (docs/flight_plan_candidates.md Sec 1.4
#     + SYNTHESIS; ADR-0056 aspect bias). Default OFF (byte-identical). ---------
#
# THE PROBLEM (MEASURED). The camera-only terminal builds its whole velocity
# command in the LOS frame:  u = (cos(lambda), sin(lambda)), p = u rotated 90 deg,
# v = v_close*u + v_perp*p.  So ANY constant error in the estimated LOS azimuth
# `lambda` ROTATES the entire commanded velocity vector by that angle.  On the
# flown arms the estimated LOS carries exactly such an offset, and its SIGN FLIPS
# with the crossing direction:
#
#     ARM B (framed, 100 REAL ENGAGE ticks, seed 123):
#         l2r  median(lambda - gt_LOS) = +11.0 deg   (per-flight medians +9.0..+16.6)
#         r2l  median(lambda - gt_LOS) = -17.8 deg   (per-flight medians -16.6..-21.4)
#
# At 3-5 m/s closing over ~1 s that is ~0.8-1.7 m of induced miss -- the same size
# as the measured "camera makes a well-aimed dash worse" penalty (the SYNTHESIS
# table: r2l 0.77 m dash-only -> 2.08 m camera-live).
#
# TWO MECHANISMS ARE MIXED IN THAT NUMBER, and this module offers a knob for each:
#
#  (1) A DIRECTION-KEYED CONSTANT -- the ADR-0056 aspect bias proper: the
#      markerless box centre does not sit on the target's centroid, and which way
#      it slides depends on which aspect the target presents.  Corrected by
#      `bias_deg` below.  MEASURED to be stable flight-to-flight in the framed
#      regime: a leave-one-flight-out fit on ARM B cut the held-out median |LOS
#      error| from 16.6 deg to 3.5 deg, with the fitted constant varying only
#      +10.6..+12.0 (l2r) / -16.8..-19.0 (r2l) across folds.
#
#  (2) A LOS-RATE LAG -- the estimate is old: the detector's frame is captured,
#      inferred and filtered before it reaches guidance (MEAS_STALE_S 0.4 s allows
#      a measurement to be re-used, the seeker corrects on only ~1/3 of the 20 Hz
#      ticks, and the alpha-beta filter adds its own lag).  Against a terminal LOS
#      slewing at 40-500 deg/s that alone is tens of degrees.  Regressing the
#      measured LOS error on the gt LOS RATE over all three flown camera arms
#      (n=147 REAL ENGAGE ticks) gives err = -0.19 * lambda_dot with an implied
#      lag tau ~ 190 ms, and removing it cuts the median |LOS error| 15.7 -> 5.8
#      deg.  Corrected by `lag_s` below, which needs NO direction key at all: the
#      sign flip falls out of the sign of lambda_dot, which is why the raw bias
#      LOOKS direction-keyed in the first place.
#
# HONESTY (CLAUDE.md "Honesty boundary" -- this path re-earns the no-cheat audit).
# Nothing here reads ground truth at inference:
#   * `lambda_rad` / `los_rate_rad_s` are the seeker's own camera-derived LOS
#     estimate and its rate (already the pro-nav input).
#   * `bias_deg` and `lag_s` are PRE-FLIGHT CALIBRATION CONSTANTS, the same class
#     as camera intrinsics or a lens-distortion coefficient: measured OFFLINE on
#     previously logged flights by scripts/experiments/flight_plans/
#     measure_aspect_bias.py, then compiled in.  The human-labeller analogue.
#   * the CROSSING DIRECTION that keys the sign is known at LAUNCH, from the same
#     target kinematics that already size the coded dash's collision lead -- it is
#     the operator's aim input, not a live sensor read.  `crossing_sign` computes
#     it from pre-flight numbers only, exactly as --dash-crossing-bias-deg does.
# CALIBRATE AND FLY ON DISJOINT SEEDS: fitting the constant on the same flights it
# is then scored on is the fit-and-test mirage (ADR-0061, memory
# [[reproduce-canonical-gate-geometry]]).


def crossing_sign(dash_heading_deg, target_vel):
    """Which way does the target cross the dash line?  +1 = l2r, -1 = r2l, 0 = neither.

    The 2-D cross product (dash_direction x target_velocity) in the horizontal
    (east, north) frame.  `dash_heading_deg` is a compass azimuth (0 = north,
    90 = east) -- the same convention `collision_lead_heading` returns;
    `target_vel` is (vx, vy) = (east, north) m/s.

    Both inputs are PRE-FLIGHT constants (the programmed dash aim and the
    operator's known target track), so the returned key is a launch-time
    constant, never a live sensor read.

    THIS IS NOW THE FLOWN SIGN KEY (corrected 2026-07-25). It began as a MIRROR
    of the key scripts/m4_intercept.py computed inline for --dash-crossing-bias-deg
    (`sin(h)*vy - cos(h)*vx`, no dead-band), and that inline path was deliberately
    left untouched for byte-identity. Commit 1911ce9 (2026-07-24) REVERSED that:
    m4_intercept.py now imports and calls this function at all three sign-key
    sites -- the collision-lead heading (`_csign_lead`), --dash-crossing-bias-deg
    (`_cs`), and the terminal bearing-bias (`_csign`); find them with
    `grep -n crossing_sign scripts/m4_intercept.py` -- so the float-dust bug below
    is fixed in the flown sim path too and there is no longer a separate inline
    copy to mirror. (Deliberately NO line numbers: the first version of this note
    pinned :2409/:2464/:2498, they drifted the same day, and a stale pointer in
    THIS docstring is what manufactured the false "the bug is still live" finding
    it was written to retire.)
    flight/tests/test_guidance.py still pins this function against the old inline
    expression on the canonical crossing geometries (byte-identical there; the ONLY
    intended divergence is the near-head-on dead-band).
    """
    h = math.radians(float(dash_heading_deg))
    vx, vy = float(target_vel[0]), float(target_vel[1])
    cross = math.sin(h) * vy - math.cos(h) * vx
    # RELATIVE dead-band. `cross` = |Vt| * sin(angle between the aim and the
    # target track), so dividing by |Vt| turns the test into a pure angle test.
    # A bare `cross != 0.0` (what m4's inline dash-bias key uses) calls a
    # perfectly head-on geometry "l2r" off 1e-16 of float dust in cos(90 deg) --
    # and then rotates the LOS by a full ~18 deg on the strength of it. 1e-9 is
    # ~6e-8 deg of crossing angle: far below any real geometry, far above the dust.
    speed = math.hypot(vx, vy)
    if speed <= 0.0 or abs(cross) <= 1e-9 * speed:
        return 0
    return 1 if cross > 0.0 else -1


def resolve_terminal_bearing_bias_deg(cross_sign, symmetric_deg=0.0,
                                      l2r_deg=None, r2l_deg=None):
    """The SIGNED LOS bias constant (deg) to remove, for this crossing direction.

    Two ways to specify it, per-direction winning where given:

      * `l2r_deg` / `r2l_deg` -- SIGNED per-direction constants, in the exact
        convention measure_aspect_bias.py prints (median of lambda - gt_LOS).  Use
        these: the measured bias is ASYMMETRIC (+11 l2r vs -18 r2l, a factor 1.6),
        which a single magnitude cannot express.
      * `symmetric_deg` -- one MAGNITUDE with the sign auto-keyed from the crossing
        direction (l2r -> +B, r2l -> -B), matching the measured sign pattern.  One
        knob, so it is the cheaper thing to sweep and the harder thing to overfit
        (docs/flight_plan_candidates.md Sec 2 arm H pre-registers B = 15 deg).

    `cross_sign` == 0 (a head-on / non-crossing geometry, where "l2r" and "r2l"
    are not defined) -> 0.0: no correction rather than a coin-flip sign.
    """
    if cross_sign > 0:
        return float(l2r_deg) if l2r_deg is not None else float(symmetric_deg)
    if cross_sign < 0:
        return float(r2l_deg) if r2l_deg is not None else -float(symmetric_deg)
    return 0.0


def compensate_terminal_los(lambda_rad, bias_deg=0.0, los_rate_rad_s=None,
                            lag_s=0.0):
    """Correct the terminal LOS azimuth before the velocity command is built.

        lambda_cmd = wrap_pi( lambda_rad - radians(bias_deg) + lag_s * lambda_dot )

    - `bias_deg`: the signed aspect-bias constant from
      `resolve_terminal_bearing_bias_deg`.  SUBTRACTED, because it was measured as
      (estimate - truth): removing it moves the estimate back onto the truth.
    - `lag_s` + `los_rate_rad_s`: a first-order LEAD that extrapolates the (old)
      estimate forward by the seeker+filter transport lag.  ADDED, because the
      estimate is behind: lambda_hat ~ lambda_true(t - tau) ~ lambda_true(t) -
      tau*lambda_dot.  `los_rate_rad_s` is the filter's OWN rate estimate (the
      pro-nav input) -- own-state, no ground truth, and it is what makes the
      correction self-signing across crossing direction.

    DEFAULT IS EXACT IDENTITY: when NO term is actually live the input object is
    returned unchanged (no wrap, no float round-trip), so an unflagged run is
    byte-identical -- and so is a lag-only run on the ticks where the rate
    estimate is not yet available (`radians(0.0)` then `wrap_pi` is NOT the
    identity in floating point: it moves the last bit).  Returns `lambda_rad`
    untouched if it is None.
    """
    if lambda_rad is None:
        return lambda_rad
    lead = bool(lag_s) and los_rate_rad_s is not None
    if not bias_deg and not lead:
        return lambda_rad
    out = lambda_rad
    if bias_deg:
        out -= math.radians(bias_deg)
    if lead:
        out += lag_s * los_rate_rad_s
    return wrap_pi(out)


# --- DASH ALTITUDE TRIM + HARD AGL FLOOR ------------------------------------
#     ADR-0085 decided a camera-driven terminal vertical channel with a hard AGL
#     minimum. NEITHER HALF WAS EVER WRITTEN (contradiction
#     `terminal-vertical-channel-decided-not-built`, 2026-09-09). This is the
#     half that the measurements say actually matters, plus the safety floor.
#     Default-OFF and byte-identical, per this module's convention.
#
# WHY A DASH TRIM AND NOT A TERMINAL LAW -- the reasoning is measured, not assumed:
#
#  (1) The vertical error is DELIVERED BY THE DASH.
#      `scripts/forensics/vertical_miss_anatomy.py` over the committed per-tick
#      logs: the interceptor is off-altitude at CPA on 24 of 24 flights (median
#      +0.485 m, i.e. ABOVE the target), and altitude drifts +0.320 m across the
#      dash -- same sign, and 66% of the miss. The reference is not held through
#      the dash, and whatever the terminal does afterwards inherits that offset.
#
#  (2) The terminal has almost no authority left to spend on it.
#      `scripts/forensics/handoff_closing_speed.py` measured the pre-handoff
#      closing speed at 17.60 m/s (median, 14 flights), not the 9.0 m/s the money
#      gate assumes. Re-deriving 1/2*a*t_go^2 at the measured speeds leaves the
#      AprilTag path 0.02-0.17 m of correction capacity. A terminal vertical law
#      cannot null a ~0.4 m delivered offset out of 0.17 m of authority.
#
#  (3) The project already has the right precedent. Horizontal aim error was NOT
#      fixed in the terminal either: ADR-0080 DERIVED the coded-dash crossing
#      bias as a pre-flight constant ("DERIVE it, don't tune it") and ADR-0083
#      adopted a measured +5 deg trim. This is the altitude analogue.
#
# DIRECTION OF THE SIGN, stated once because it is the easy thing to get backwards:
# a POSITIVE measured drift means the vehicle ends the dash ABOVE where it should
# be, so the reference must be LOWERED. `derive_dash_alt_trim_m` returns the
# NEGATED drift for exactly that reason, and the test asserts it.
#
# NOT VALIDATED. Nothing here has flown, in sim or on hardware. The drift above
# was measured on the CUE-ERA two-stage arms (phase `DASH`) because the coded-dash
# per-tick archive is gitignored; the drift's SIGN is configuration-dependent (the
# cue-era arms fly a running start and a loft, which climb) and ADR-0095's fleet
# was LOW by 0.374 m. So the MECHANISM transfers and the NUMBER does not: re-run
# both forensics with `--phase CODED_DASH` on the dev machine and derive the trim
# from that fleet before any arm flies with a non-zero value.
# Pre-registration: docs/vertical_channel_prereg.md.

ALT_TRIM_SHARE_DEFAULT = 1.0     # correct the whole measured drift, not a fraction


def derive_dash_alt_trim_m(measured_drift_m, share=ALT_TRIM_SHARE_DEFAULT):
    """Altitude-reference trim (m) that cancels a MEASURED dash altitude drift.

    `measured_drift_m` is (altitude at dash end) - (pre-dash altitude), the exact
    quantity `scripts/forensics/vertical_miss_anatomy.py` prints as "altitude sag
    over the dash". Positive = the vehicle ends the dash HIGH.

    Returns the offset to ADD to the dash altitude reference, i.e. the negated
    drift scaled by `share`. `share` < 1 exists so a first arm can correct only
    part of a large drift rather than betting the whole engagement on one
    unflown number; it is NOT a tuning knob and the default corrects in full.

    DERIVED, never tuned (ADR-0080's rule): the input is a measurement, so a
    reviewer can re-derive this from the same logs. Zero drift -> exactly 0.0, so
    a flight with no measured drift is byte-identical to no trim at all.
    """
    d = float(measured_drift_m)
    if d != d:
        raise ValueError("derive_dash_alt_trim_m: measured_drift_m is NaN -- a "
                         "trim derived from a missing measurement is exactly the "
                         "substitute-a-default failure the error policy forbids")
    sh = float(share)
    if sh != sh or not (0.0 <= sh <= 1.0):
        raise ValueError(f"derive_dash_alt_trim_m: share={share!r} outside [0, 1]. "
                         f"share exists to correct only PART of a measured drift; "
                         f"above 1.0 it is an unbounded tuning knob, which this "
                         f"project's own rule (ADR-0080, derive don't tune) "
                         f"forbids on a guidance path.")
    if d == 0.0:
        return 0.0
    return -d * sh


def apply_alt_ref_trim(alt_ref_m, trim_m=0.0, min_agl_m=None):
    """The dash/terminal altitude reference, trimmed and floored.

    Two independent corrections, both default-inert:

      * `trim_m`   -- the pre-flight constant from `derive_dash_alt_trim_m`.
      * `min_agl_m` -- ADR-0085's HARD FLOOR: the reference is never allowed
        below it, whatever the trim or a seeker elevation asks for. This is a
        FLIGHT-SAFETY backstop, not a guidance term, and it is the reason the
        floor lives here rather than in a caller: every reference in the flight
        driver passes through one function, so the floor cannot be forgotten at
        one call site. `None` = no floor (the pre-existing behaviour).

    Returns `alt_ref_m` UNCHANGED (same value, no arithmetic applied) when the
    trim is zero and no floor bites, so every previously validated result is
    preserved exactly -- the same identity guarantee as
    `compensate_terminal_los` and `camera_to_cg_los`.
    """
    if trim_m == 0.0 and min_agl_m is None:
        return alt_ref_m
    # FAIL CLOSED ON NaN (review finding F10, 2026-09-10). `out < min_agl_m` is
    # False for NaN, so a NaN trim used to sail straight past the safety floor and
    # a NaN floor used to disable it silently -- the one input class a backstop
    # must refuse. Neither is reachable from a config today; both are reachable
    # from a future derived value, which is exactly how this class bites.
    if min_agl_m is not None and min_agl_m != min_agl_m:
        raise ValueError("apply_alt_ref_trim: min_agl_m is NaN -- a safety floor "
                         "that cannot be compared is not a floor")
    if trim_m != trim_m:
        raise ValueError("apply_alt_ref_trim: trim_m is NaN -- refusing to bias "
                         "the altitude reference by an uncomputable value")
    out = alt_ref_m + float(trim_m) if trim_m != 0.0 else alt_ref_m
    if min_agl_m is not None and out < min_agl_m:
        return float(min_agl_m)
    return out


def floor_v_down(v_down, alt_m, min_agl_m=None, kp=1.0, v_vert_max=2.0):
    """Clamp a COMMANDED vertical velocity so it can never fly below the floor.

    THIS IS THE HALF THAT WAS MISSING (review finding F3, 2026-09-10). Trimming
    the altitude REFERENCE only floors the states whose command is built from a
    reference. The guided terminal does not use one: `_step_engage` returns the
    seeker's own setpoint, and BREAKOFF and SAFE emit hardcoded vertical rates. So
    the floor was absent from precisely the state ADR-0085 wrote it for -- "below
    which the vehicle will not descend regardless of the seeker". A 30-line probe
    falsified the claim: ENGAGE commanded +2.0 m/s down while 2 m under its own
    floor.

    SCOPE OF THAT CLAIM, corrected 2026-09-10 (review finding S11). This is
    wired at ONE site: `flight/deploy/real_flight.py`'s `_decide`, the single exit
    every state's setpoint funnels through there -- so within THAT module it
    covers every state, including a setpoint the module never built. It is NOT
    wired into `scripts/m4_intercept.py` at all, so the sim path has no AGL floor.
    An earlier version of this sentence read as a project-wide guarantee, which is
    the same over-broad "one function, so it cannot be forgotten" claim that
    ADR-0099 finding F3 already had to retract once.

    `v_down` is positive DOWN (NED). At or below the floor a descent is refused
    and replaced by a proportional CLIMB, so the vehicle recovers rather than
    hovering at the boundary. Above the floor the command passes through
    UNCHANGED, so this is byte-identical whenever the floor is not set or not
    breached.

    Returns (v_down, floored) so a caller can log that it bit -- an actuator that
    silently modifies a command is the defect class ADR-0096 was written about.
    """
    if min_agl_m is None or alt_m is None:
        return v_down, False
    if min_agl_m != min_agl_m or alt_m != alt_m:
        raise ValueError("floor_v_down: NaN altitude or floor -- refusing to "
                         "decide a safety clamp on an uncomputable comparison")
    if alt_m > min_agl_m:
        return v_down, False
    # At or below the floor: never descend, and climb back proportionally.
    climb = _clamp_symmetric(kp * (alt_m - min_agl_m), v_vert_max)
    return min(v_down, climb, 0.0), True


def _clamp_symmetric(x, lim):
    lim = abs(float(lim))
    return max(-lim, min(lim, x))


# --------------------------------------------------------------------------
# The vertical channel during a camera DROPOUT
# --------------------------------------------------------------------------
# WHY THIS EXISTS (2026-09-10; NOT SIM-VALIDATED, and read the ATTRIBUTION
# section before quoting any number from it).
#
# WHAT IS MEASURED. `scripts/forensics/vertical_miss_anatomy.py` decomposes the
# vertical separation at closest approach on the 16 committed cue-era flights
# into three additive terms. Paired per-flight medians, n=13:
#
#     origin (settled pre-dash offset)   +0.127 m   23%  (range 15-30%)
#     dash_delta (hover -> dash end)     +0.012 m   10%  (range  1-43%)
#     post_dash_delta (dash end -> CPA)  +0.348 m   66%  (range 29-78%)
#
# So the vertical error is NOT delivered by the dash. Measured against a SETTLED
# pre-dash hover the altitude hold is essentially perfect across the dash
# (-0.019 m, robust from -0.012 to -0.025 across six baseline windows); most of
# the excursion arrives after handoff. ADR-0099 reason (1) -- "the error is
# delivered by the dash, so correct it there" -- is REFUTED on this fleet.
#
# THE CODE DEFECT, which is real and is why this lever exists. The ENGAGE
# terminal recomputes an altitude-hold P-loop on every tick WITH a fresh
# detection -- `cmd_vd` matches `clamp(KP_ALT*(alt_m - ALT_REF_M))` on 150/150
# such ticks -- but its dropout branch does not:
#
#   * FAR dropout issues `cmd = (0.0, 0.0, 0.0, psi)`, abandoning the altitude
#     hold, and stores that tuple as `last_cmd`;
#   * NEAR dropout issues `cmd = last_cmd`, re-issuing that zero indefinitely.
#
# On ENGAGE ticks where the P-loop would have commanded more than 0.01 m/s the
# vertical was zeroed on 0/150 ticks WITH a detection and 792/877 WITHOUT one,
# and the zero latched for the rest of the terminal on 10 of the 14 flights that
# reached ENGAGE (2 of the 16 never engaged at all -- the denominator is 14, not
# 16).
#
# ATTRIBUTION -- WHAT THIS FLEET CANNOT SHOW, AND WHY THE HONEST PREDICTION IS A
# NULL. Two independent checks say the vertical zero is NOT established as the
# cause of the excursion:
#
#  1. THE VERTICAL AND HORIZONTAL ZEROS ARE THE SAME EVENT. The far-dropout
#     branch zeroes all three velocity components together. Over the same ENGAGE
#     ticks: both zeroed 792, vertical-only 0, horizontal-only 0 -- perfectly
#     collinear, no discordant tick. And the horizontal event is far more
#     energetic: commanded horizontal speed goes from 16.00 m/s at the dash tail
#     to 0.00 m/s on the first far-dropout tick. A multicopter handed a full-stop
#     brake at 16 m/s pitches up and BALLOONS, which is a sufficient -- and much
#     larger -- explanation for a +0.35 m climb than a missing 0.5 m/s-authority
#     altitude hold. This is the separately-documented `coast_zero` defect
#     (docs/audit_2026-07-25_whats_left.md). Because the two are collinear,
#     SEPARATING them is what makes attribution impossible, not what enables it.
#
#  2. THE AVAILABLE CONTROL ARM ARGUES AGAINST IT. On the 4 flights where the
#     vertical command SURVIVED, the vehicle climbed anyway -- 0.28 to 0.65 m --
#     while its mean commanded vertical velocity was a DESCENT (+0.07 to
#     +0.26 m/s down). Their post-handoff term is statistically indistinguishable
#     from the latched flights (+0.429 m for n=4 against +0.498 m for n=10). The
#     latch is also confounded with terminal duration: latched terminals last
#     2.2-2.8 s against 0.4-0.8 s, so "latched" is largely a proxy for "the
#     terminal lasted longer".
#
# There is also a prior question neither lever answers: the altitude loop shows a
# persistent ~-0.087 m steady-state error in hover while continuously commanding
# -0.087 m/s, i.e. the commanded vertical velocity is not being delivered. Both
# this lever and ADR-0099's trim assume that loop has authority. That cannot be
# root-caused without a sim run.
#
# SO THE CLAIM THIS LEVER MAKES is architectural, not empirical: `alt_m` is
# own-state (EKF/barometer), carries no target information, and is available on
# every tick, so abandoning the altitude hold during a CAMERA dropout needs
# nothing the vehicle has lost. It buys no robustness; it is a pure loss. Fixing
# it is right on its own merits. It is NOT established as the fix for the 66%,
# and the pre-registered prediction for a vertical-only arm is a NULL
# (docs/vertical_channel_prereg.md section 8, which also registers why the arm
# must be run as a 2x2 with the coast gate rather than standalone).
#
# A quarter of the post-handoff term is not the airframe at all: `gt_cam_z -
# alt_m`, which should be a constant geometric offset, moves +0.087 m from hover
# to CPA (24% of post_dash_delta) because the camera rides a forward boom that
# lifts under pitch.
#
# SCOPE, stated plainly: the two branches above are LIVE in the current
# `scripts/m4_intercept.py`; every magnitude here is from the cue-era two-stage
# `--handoff` fleet of 2026-07-09 (phase `DASH`, not `CODED_DASH`), which is OFF
# the adopted configuration, and whose terminal had a detection on only 155 of
# 1032 ENGAGE ticks. The honesty boundary is untouched -- no `gt_*` value enters
# this function. This lever has NEVER FLOWN, in sim or otherwise; it is
# default-OFF and byte-identical when off.
#
# WHAT THIS DELIBERATELY DOES NOT DO: fix the horizontal half of the same latch.
# That is a bigger change with its own lever (`--terminal-coast-gate`) and its own
# evidence. Per (1) above, that ordering means a vertical-only arm is NOT
# interpretable on its own -- which is registered as a constraint on the arm, not
# hidden as a caveat here.


def preserve_dropout_vertical(cmd, v_down_fresh, enable=False):
    """Restore a FRESHLY COMPUTED vertical term on a dropout command.

    `cmd` is a 4-tuple `(vn, ve, v_down, yaw)` in the NED velocity-setpoint
    convention PX4's `set_velocity_ned` takes; `v_down` is positive DOWN.

    Default OFF, and when off this returns the SAME OBJECT it was given -- not an
    equal copy -- so the setpoint stream is byte-identical and a test can assert
    identity rather than approximate equality. It is also a no-op when
    `v_down_fresh` is None (no altitude estimate this tick), because inventing a
    vertical command from a missing measurement is exactly the fail-closed
    violation `docs/error_handling_policy.md` forbids.

    Only the vertical term is touched. The horizontal hold and the yaw setpoint
    are passed through untouched by construction, so this cannot change the
    horizontal behaviour whose defect is tracked separately.
    """
    if not enable or v_down_fresh is None:
        return cmd
    v = float(v_down_fresh)
    if v != v:
        raise ValueError(
            "preserve_dropout_vertical: v_down_fresh is NaN -- refusing to "
            "command a vertical velocity from a broken altitude estimate. "
            "Pass None to hold the existing command instead.")
    vn, ve, _vd, yaw = cmd
    return (vn, ve, v, yaw)
