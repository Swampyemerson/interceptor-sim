"""flight.fov_guidance -- Phase B of the intercept-accuracy program
(docs/intercept_accuracy_levers.md): HOLD THE RESIDUAL. A look-angle-
constrained PN augmentation + an IBVS pixel-centering term that keep the
target inside the seeker FOV through the dash pitch swing (+40 deg loft ->
-35 deg dive) that the static wedge cannot track. The wedge removes the
STATIC pointing bias; this module absorbs the DYNAMIC residual.

STATUS: PRE-CODED, OPT-IN, NOT SIM-VALIDATED. Default-off behind --fov-hold
(wiring: docs/phase_bcd_wiring.md). Phase A (loft-then-dive pointing) must
land first -- this lever only bites once the target is framed at all
("only once framed", levers doc Phase B).

THE AXIS: everything here is formulated on the VERTICAL (pitch-coupling)
look-angle axis, NOT the classic horizontal-plane FOV-limited PN. The
measured failure mode (levers doc fact #1) is vertical: the nose-down dash
pitch drops the camera boresight below a co-altitude target, parking it at
the frame TOP edge or out of frame (~75% of ticks out-of-frame at 8-12 m).
The horizontal base law (a = N*Vc*lambda_dot) is left untouched.

THE PHYSICS (why each term looks the way it does):
  * Vertical look angle eps_v = atan2(y_opt, z_opt) of the target ray in the
    camera OPTICAL frame (x right, y down, z forward -- flight.camera /
    flight.geometry convention). eps_v < 0 = target ABOVE boresight (frame
    top, the dash failure mode); eps_v > 0 = below (frame bottom).
  * A vertical (climb/descend) acceleration a_up moves the interceptor
    relative to the target, rotating the LOS elevation at ~v_up/R; scaling
    the command with the closing speed Vc ~ R/t_go makes the re-centering
    settle in a roughly constant fraction of time-to-go -- the same
    normalization argument as PN itself.
  * Sign: target at frame top (eps_v < 0) -> CLIMB (a_up > 0). Climbing
    raises the interceptor, the relative target position gains a down
    component, the target image moves down toward center. a_up therefore
    always carries the OPPOSITE sign of eps_v.
  * Pitch coupling / look-angle constraint: a quad's forward acceleration
    sets its steady pitch, theta = -atan(a_fwd / g) (nose-down), and the
    body-fixed camera pitches with it: d(eps_v)/d(theta) = +1. So the
    current in-frame position eps_v bounds how much MORE forward accel can
    be commanded before the pitch change itself pushes the target out the
    top: that bound is `pitch_headroom_accel_cap`. This is the look-angle
    CONSTRAINT half of the term -- it constrains, it does not steer.

NO BEARING DIFFERENTIATION (hard rule): every term here is pure POSITION
feedback on the look angle / pixel offset of the CURRENT frame. Nothing
stores a previous bearing, nothing divides an angle change by dt. That is
the design condition from the levers doc -- differentiating a noisy metric
bearing is what sank the earlier APN/PIP attempts (graveyard, ADR-0069).

HONESTY BOUNDARY (ADR-0008/0010): inputs are the camera target ray /
pixel (meas_xyz or (u,v) via flight.camera.CameraModel) and the vehicle's
OWN-state EKF pitch. No gt_*, no datalink. NOTE: this adds a NEW
pixel-into-loop path (the vertical channel steers on image position), so
wiring it into guidance RE-EARNS the numeric no-cheat audit before any
result is claimed (CLAUDE.md honesty boundary).

TRADE, STATED HONESTLY: the centering/barrier terms spend real vertical
authority to buy perception availability -- a deliberate trajectory bias,
the same trade the loft makes. The deadband (barrier engages only past
`engage_frac` of the usable half-FOV) and the `a_up_max` cap bound how much
authority it can take from the base law. Whether the trade wins is exactly
the Phase B A/B (wedge-only PN vs +look-angle+centering) -- lab ranks,
Gazebo decides.
"""

import math
from dataclasses import dataclass
from typing import Optional


def _clamp(x, lo, hi):
    """Clamp x into [lo, hi] (same helper as flight.estimator)."""
    return max(lo, min(hi, x))


def vertical_look_angle(meas_xyz):
    """Vertical look angle eps_v (rad) of the target ray in the camera
    OPTICAL frame (x right, y down, z forward): atan2(y, z). Negative =
    target ABOVE boresight (frame top). Pure position -- no history, no
    differentiation. Returns None if the ray is missing."""
    if meas_xyz is None:
        return None
    return math.atan2(float(meas_xyz[1]), float(meas_xyz[2]))


def horizontal_look_angle(meas_xyz):
    """Horizontal look angle (rad): atan2(x_right, z_forward) -- identical to
    the seeker's bearing_rad; provided for symmetry/diagnostics."""
    if meas_xyz is None:
        return None
    return math.atan2(float(meas_xyz[0]), float(meas_xyz[2]))


def pixel_v_to_look_angle(v_px, cy, fy):
    """Vertical pixel row -> vertical look angle (rad): atan((v-cy)/fy).
    The pixel-space form of `vertical_look_angle` for callers that have only
    the detector box center, not a metric ray. For the real distorted lens,
    undistort first (flight.camera.CameraModel.pixel_to_ray) and use
    `vertical_look_angle` on the ray instead."""
    return math.atan2((float(v_px) - float(cy)) / float(fy), 1.0)


def half_fov_from_intrinsics(f_px, size_px):
    """Half field-of-view (rad) of an ideal pinhole axis: atan((size/2)/f).
    E.g. half-VFOV = half_fov_from_intrinsics(fy, height_px). Use this at
    wiring time so the config traces to the calibrated intrinsics instead of
    a typed-in constant (numbers trace to a derivation)."""
    return math.atan2(float(size_px) / 2.0, float(f_px))


@dataclass
class FovHoldConfig:
    """Tuning for the vertical FOV-hold. Defaults are STARTING POINTS for the
    Phase B A/B, not validated numbers.

    half_vfov_rad: vertical half field-of-view. Sim x500_mono_cam nominal:
        ~100 deg HFOV at 1280x720 -> half-VFOV = atan(tan(50 deg)*720/1280)
        ~= 0.59 rad (33.8 deg). Derive from intrinsics at wiring time via
        half_fov_from_intrinsics(fy, height).
    edge_margin_rad: keep-out margin inside the hard edge (detector needs the
        whole box in frame, not just its center; ~0.05 rad ~= 3 deg).
    engage_frac: barrier deadband -- fraction of the usable half-FOV
        (half_vfov - margin) inside which the barrier is OFF (the wedge +
        base law own the center of the frame).
    k_barrier: barrier gain, accel per (Vc * unitless-excess^2).
    k_center: IBVS centering gain, accel per (Vc * rad). Small and always-on:
        the gentle residual absorber; the barrier is the hard stop.
    a_up_max_m_s2: cap on the total vertical command (authority bound).
    """

    half_vfov_rad: float = 0.59
    edge_margin_rad: float = 0.05
    engage_frac: float = 0.6
    k_barrier: float = 1.0
    k_center: float = 0.15
    a_up_max_m_s2: float = 6.0

    @property
    def eps_limit_rad(self) -> float:
        """Usable half-FOV: hard edge minus the keep-out margin."""
        return max(1e-6, self.half_vfov_rad - self.edge_margin_rad)

    @property
    def eps_engage_rad(self) -> float:
        """Look angle at which the barrier starts to push."""
        return self.engage_frac * self.eps_limit_rad


def ibvs_centering_accel(eps_v, vc, cfg: FovHoldConfig):
    """IBVS pixel-centering term: a_up = -k_center * Vc * eps_v (m/s^2,
    positive = climb). Continuous, proportional, position-only -- classic
    image-based visual servoing on the vertical image coordinate (for small
    angles eps_v == (v-cy)/fy, so this IS pixel feedback). Opposite sign of
    eps_v: target at frame top -> climb."""
    if eps_v is None:
        return 0.0
    return -cfg.k_center * max(0.0, vc) * eps_v


def fov_edge_barrier_accel(eps_v, vc, cfg: FovHoldConfig):
    """Soft-barrier term: zero inside the deadband (|eps_v| <= engage), then
    a quadratic ramp in the normalized excess toward the usable edge,
    pushing the target back toward center (opposite sign of eps_v). The
    quadratic keeps the augmentation C1-smooth at engagement (no command
    step for the velocity loop to chase)."""
    if eps_v is None:
        return 0.0
    span = cfg.eps_limit_rad - cfg.eps_engage_rad
    if span <= 0.0:
        return 0.0
    excess = (abs(eps_v) - cfg.eps_engage_rad) / span
    if excess <= 0.0:
        return 0.0
    return -math.copysign(cfg.k_barrier * max(0.0, vc) * excess * excess, eps_v)


def fov_hold_vertical_accel(eps_v, vc, cfg: FovHoldConfig):
    """Total vertical FOV-hold command: centering + barrier, capped at
    +-a_up_max. Positive = climb (NED wiring: a_down = -a_up)."""
    a = ibvs_centering_accel(eps_v, vc, cfg) + fov_edge_barrier_accel(eps_v, vc, cfg)
    return _clamp(a, -cfg.a_up_max_m_s2, cfg.a_up_max_m_s2)


def pitch_headroom_accel_cap(eps_v, pitch_rad, cfg: FovHoldConfig,
                             g_m_s2: float = 9.81):
    """Look-angle CONSTRAINT: the largest forward acceleration whose steady
    quad pitch keeps the target inside the usable vertical FOV, given where
    the target sits in frame RIGHT NOW.

    Steady-state pitch under forward accel: theta = -atan(a_fwd/g) (nose
    down). The body-fixed camera pitches with the vehicle, d(eps_v)/d(theta)
    = +1, so pitching further down by delta moves the target UP in frame by
    delta. The top-edge constraint eps_v_new >= -eps_limit gives the most
    nose-down admissible pitch  theta_min = theta_now - (eps_v + eps_limit),
    hence a_fwd_max = g * tan(-theta_min) (0 if even the current pitch
    violates -- target already past the usable edge).

    Inputs: eps_v (camera, position-only) + pitch_rad (OWN-state EKF pitch,
    rad, nose-down negative) -- honesty-clean. Returns a_fwd_max in m/s^2.
    This is the hook for co-sizing the accel-capped dash with the wedge
    (levers doc: "reserve high thrust-to-weight for terminal lateral"):
    the dash/velocity planner takes min(its own cap, this cap)."""
    if eps_v is None or pitch_rad is None:
        return None
    theta_min = pitch_rad - (eps_v + cfg.eps_limit_rad)
    if theta_min >= 0.0:
        return 0.0
    # tan blows up near -90 deg; clamp to a physical 85 deg nose-down.
    theta_min = max(theta_min, math.radians(-85.0))
    return g_m_s2 * math.tan(-theta_min)


@dataclass
class FovHoldCommand:
    """One tick's FOV-hold augmentation.

    a_lat_m_s2: the base horizontal PN command, UNMODIFIED (passed through
        so the caller has one struct to consume).
    a_up_m_s2: vertical FOV-hold command, positive = climb (0.0 when
        inactive). NED wiring: v_down_extra integrates -a_up.
    a_fwd_cap_m_s2: pitch-headroom forward-accel cap (None if pitch or ray
        unavailable). Advisory -- the dash/closing-speed planner min()s it in.
    eps_v_rad: measured vertical look angle (None if no ray).
    barrier_active: True when |eps_v| is past the deadband (diagnostics/CSV).
    """

    a_lat_m_s2: float
    a_up_m_s2: float
    a_fwd_cap_m_s2: Optional[float]
    eps_v_rad: Optional[float]
    barrier_active: bool


def augment_pronav(a_lat_base, meas_xyz, vc, cfg: FovHoldConfig,
                   pitch_rad=None):
    """THE ENTRY POINT: augment a base PN command with the vertical FOV-hold.

    Args:
        a_lat_base: the base law's horizontal lateral accel (N*Vc*lambda_dot),
            returned untouched -- the horizontal law is not this module's axis.
        meas_xyz: target ray in the camera OPTICAL frame (Measurement.meas_xyz
            or CameraModel.pixel_to_ray output); None = no detection this tick.
        vc: closing speed (m/s), e.g. flight.guidance.closing_speed output.
        cfg: FovHoldConfig.
        pitch_rad: own-state EKF pitch (rad), for the accel-cap advisory;
            None skips it.

    Returns a FovHoldCommand. With meas_xyz None the command is inert
    (a_up=0, cap None) -- coasting ticks get pure base-law behavior, so the
    default-off flag plus a missing ray are both byte-identical to today.
    """
    eps_v = vertical_look_angle(meas_xyz)
    if eps_v is None:
        return FovHoldCommand(a_lat_base, 0.0, None, None, False)
    a_up = fov_hold_vertical_accel(eps_v, vc, cfg)
    cap = pitch_headroom_accel_cap(eps_v, pitch_rad, cfg)
    return FovHoldCommand(
        a_lat_m_s2=a_lat_base,
        a_up_m_s2=a_up,
        a_fwd_cap_m_s2=cap,
        eps_v_rad=eps_v,
        barrier_active=abs(eps_v) > cfg.eps_engage_rad,
    )
