"""flight.guidance -- the coded-dash aim and (later) the pro-nav terminal law.

The real interceptor's operating model (memory [[real-build-pivot]], ADR-0076
add #18): an OPEN-LOOP coded dash pointed the right way, then a CAMERA-ONLY
pro-nav terminal. `collision_lead_heading` computes the "right way" -- a
pre-flight constant from the KNOWN launch kinematics (where the target is and
where it's heading), exactly what a human or a ground cue programs at launch. It
reads no live sensor and no ground truth.
"""

import math


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
