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

    MIRRORS the sign key that scripts/m4_intercept.py computes inline for
    --dash-crossing-bias-deg (`sin(h)*vy - cos(h)*vx`); that flown code path is
    deliberately left untouched (byte-identity), and flight/tests/test_guidance.py
    pins this function to agree with it on the canonical geometries.
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
