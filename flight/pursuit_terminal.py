"""flight.pursuit_terminal -- the "chase only" camera terminal (ADR-0103,
supersedes the sprint-and-fly-by hybrid ruling; docs/pursuit_port_2026-09-17.md).

Ported from `isim.concepts.PursuitRendezvousGuidance` -- validated ONLY inside
isim's own prototype so far (82% inside 0.35 m nominal, all realistic errors
on, fx 385, rear tag -- ADR-0103 v7) -- into the SAME duck-typed contract as
`flight.tag_terminal.TagInterceptGuidance`, so it drops into an UNMODIFIED
`flight.deploy.real_flight.RealFlightSM`:

    step(det_box_xywh, own: OwnState, t: float) -> (Optional[Setpoint], StepTelemetry)

CONCEPT: arrive slowly instead of crossing fast. Phase A flies to a point
`d_behind_m` behind a BELIEF of the target's track (seeded from the same
pre-flight constant `flight.guidance.collision_lead_heading` already uses --
honesty-clean, never a live/ground-truth read), aiming to arrive at the
target's own believed speed. Once enough fresh tag decodes land inside a short
window, Phase B takes over: a Kalman filter tracks the target's RELATIVE
position and ABSOLUTE velocity from the camera, and the vehicle closes the
last few metres on a SCHEDULE (1.5-6 m/s, not a hard dash) with a lateral
term that nulls the eventual miss component directly.

RELATIVE STATE, NOT ABSOLUTE OWN POSITION -- this is the one significant
deviation from `isim.concepts.PursuitRendezvousGuidance`'s own internals, and
it is load-bearing, not cosmetic. isim's own-state is ground truth inside the
engine, so its port `_ConstVelKF` tracks the TARGET's absolute (pos, vel) and
subtracts a known `own_pos` to get range. The real flight-code contract's
`OwnState` carries NO position field at all (only attitude/altitude/velocity
-- see `flight.deploy.real_flight.VehicleObs`/`OwnState`), by the SAME design
choice that made `flight.tag_terminal._RelStateKF` track (r, v_target) in
RELATIVE coordinates instead of needing an absolute own-position estimate. An
earlier draft of this module dead-reckoned an absolute own position from
integrated `vel_ned` as a workaround; that reintroduces unbounded drift this
codebase deliberately designed around, so it is NOT what shipped. Instead,
EVERYTHING here is relative: the state is `(r, v_t)` = (target position minus
own position, target's absolute velocity), predicted forward each tick using
the INSTANTANEOUS own velocity as a control input (`r' = r + (v_t -
v_own)*dt`), exactly `_RelStateKF`'s own pattern, generalized to Phase A's
open-loop (pre-acquisition, no KF yet) propagation too. `d_behind_m`'s aim
offset is a CONSTANT vector subtracted from `r`, so it needs no separate
integrator. The one place this needs a genuine pre-flight input is the
CONSTRUCTOR: `belief_r0_ned`, the target's position relative to the vehicle
AT THE INSTANT ENGAGE BEGINS -- derived by the caller from the same pre-flight
target belief and dash-endpoint kinematics `collision_lead_heading` already
uses (honesty-clean; not a live read).

REUSED, NOT RE-DERIVED (`flight.tag_terminal`/`flight.deploy.seeker_loop`):
`measurement_from_box` (box -> undistorted bearing/range/optical ray),
`_optical_vec_to_ned`, `_cam_offset_ned` (the same box->NED-RELATIVE-vector
geometry `TagInterceptGuidance` already uses -- note this already returns a
vector relative to the vehicle's own CG, which is exactly the `r` this module
needs, no own-position subtraction required), `own_state_status` (own-state
precondition).

NOT REUSED: `_RelStateKF`'s TUNING or `range_measurement_noise`'s formula --
pursuit's validated numbers depend on its OWN tuned constants
(`cross_sigma_floor_m`, `kf_q_accel_ms2`, the closing schedule; swept v2-v6,
`isim/specs/pursuit_concept*.md`), so mixing in the PIP terminal's differently
-tuned filter would invalidate them. This module's KF is its own class,
`_ConstVelKF`, structurally like `_RelStateKF` (relative-state, control-input
predict) but with pursuit's own Q/R.

LATENCY (revised 2026-09-22, isim/specs/parity_trace_2026-09-22.md). The
contract still has no per-detection capture timestamp (`step` gets only `t`,
"now"), so a fixed `meas_latency_s` stands in for the true age -- that part
of the simplification is unchanged and disclosed. What CHANGED, after the
registered parity trace attributed the port's 0.3-0.8 m first-pass loss:
  (1) the pixel box is converted to NED with the own attitude interpolated
      from a short internal ring buffer AT the assumed capture instant
      `t - meas_latency_s`, not with the current-tick attitude (the native
      prototype's v3 #1, rebuilt in-contract: the buffer is filled from the
      `own` handed to every `step()` call, no new inputs). The current-tick
      conversion injected a 0.2-0.3 m median (1.5 m max) error per fix while
      the vehicle's attitude swung during the braking close.
  (2) the KF's measurement model carries the age explicitly
      (H = [I, -age*I] plus the known own-displacement term) instead of
      pre-extrapolating the measurement by `v_rel_hat * meas_latency_s` with
      H = [I, 0] -- so position fixes correctly update the VELOCITY states.
  (3) the covariance predict uses the true Jacobian (dr'/dv_t = dt*I). The
      old F = I propagated the STATE with the control-input coupling but the
      COVARIANCE without it, leaving v_t nearly unobservable; the traced
      consequence was a velocity estimate 5-6.5 m/s wrong at CPA (native:
      0.05-0.12 m/s) feeding Phase B's velocity feed-forward.

SKIPPED ON PURPOSE (measured negative or not adopted in the source concept,
default OFF there too): camera-frame close-in steering, a KF timestamp-bias
state, Phase-A vertical/yaw search sweeps. Only the core, adopted-default
path is built here.

HONESTY (CLAUDE.md; ADR-0008/0010). Inputs are (a) a detector box (camera
pixels), (b) `own: OwnState` (own-state EKF), and (c) `belief_r0_ned`/
`belief_vel0_ned` -- a PRE-FLIGHT constant, not a live or ground-truth read.
No `gt_*` anywhere. This module belongs in `real_flight._AUDITED_MODULES`
(the no-cheat AST audit), same as `seeker_loop.py`/`tag_terminal.py`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from flight.camera import CameraModel
from flight.tag_terminal import _cam_offset_ned, _optical_vec_to_ned
from flight.deploy.seeker_loop import (
    GuidanceConfig,
    OwnState,
    Setpoint,
    StepTelemetry,
    measurement_from_box,
    own_state_status,
)

__all__ = ["PursuitTerminalConfig", "PursuitTerminalGuidance"]


@dataclass
class PursuitTerminalConfig:
    """Every tunable of the "pursuit" concept. Defaults are the validated
    concept's own (`isim/specs/pursuit_concept*.md`, ADR-0103) -- none of
    these are bench-fit; they are isim-swept."""

    # --- Phase A: belief-only rendezvous, no tag required -------------------
    d_behind_m: float = 8.0
    kp_pos: float = 0.8
    v_max_ms: float = 16.0
    accel_max_ms2: float = 6.0

    # --- acquisition gate: Phase A -> Phase B --------------------------------
    acquire_n: int = 2
    acquire_window_s: float = 0.3

    # --- Phase B: KF-tracked terminal ----------------------------------------
    k_close: float = 0.6
    v_close_min_ms: float = 1.5
    v_close_max_ms: float = 6.0
    kp_lat: float = 1.5
    accel_max_horiz_b_ms2: float = 8.0
    accel_max_vert_b_ms2: float = 6.0
    fallback_s: float = 3.0
    hold_range_m: float = 1.0
    no_fallback_range_m: float = 3.0
    fallback_speed_cap_mult: float = 2.0
    fallback_speed_floor_ms: float = 3.0

    # --- KF tuning ------------------------------------------------------------
    kf_q_accel_ms2: float = 1.0
    kf_p0_pos_m: float = 3.0
    kf_p0_vel_ms: float = 5.0

    # --- measurement-noise model (own, NOT tag_terminal's) --------------------
    sigma_px: float = 0.3
    sigma_side_px: float = 0.3 * math.sqrt(2.0)
    cross_sigma_floor_m: float = 0.20

    # --- latency (fixed constant, mirrors TagTerminalConfig) ------------------
    meas_latency_s: float = 0.045
    # Own-attitude ring-buffer depth for the capture-instant conversion (module
    # docstring LATENCY (1)). Must exceed meas_latency_s by a healthy margin.
    att_history_s: float = 0.4

    # --- yaw slew ---------------------------------------------------------------
    yaw_rate_max_deg_s: float = 180.0

    # --- adaptive speed governor (2026-09-23, default OFF = legacy) -----------
    # Sizes the speed cap from NEED instead of a fixed constant, per the
    # fast-intercept-limits finding (docs/fast_intercept_limits.md #1: the
    # ceiling is overtake margin, a config number). Inputs are all
    # guidance-visible (pre-flight belief / own KF), never ground truth.
    #   Phase A cap = clamp(|believed target vel| + overtake_margin_ms,
    #                       adaptive_v_floor_ms, v_hw_max_ms)
    #   Phase B     = the same cap on the total command (so the target-
    #                 velocity feedforward for a fast target is not strangled
    #                 by the legacy fixed cap).
    # A lock-quality closure modulation (slow down when decodes go stale) was
    # built, A/B'd and REJECTED 2026-09-23: null on aim-error cells and a
    # 12-14 point contact cost at 18 m/s (timidity feedback: slowing keeps
    # the tag small and the decodes sparse) -- docs/adaptive_speed_prereg.md
    # amendments 1-2. Cap sizing only.
    # v_hw_max_ms is the assumed vehicle ceiling -- for the real airframe it
    # is an UNMEASURED given (`dash-accel-profile` register entry) until the
    # first real dash ULog; isim numbers above the fitted ~16-18 m/s
    # extrapolate the vehicle fit (docs/fast_intercept_limits.md #5).
    adaptive_speed: bool = False
    v_hw_max_ms: float = 24.0
    overtake_margin_ms: float = 6.0
    adaptive_v_floor_ms: float = 8.0

    # --- keep-in-frame vertical assist (2026-09-23, default OFF = legacy) -----
    # Attacks the +3 m-above altitude-error cliff (isim/specs/
    # alt_sensitivity_2026-09-21.md: 74-88% inside 0.35 m from -2..+2 m, 42%
    # at +3 m): a target well ABOVE sits near the TOP edge of the frame and
    # decodes go sparse/lost faster than the position-driven vertical
    # correction (kp_lat * r_perp) converges. When a FRESH decode's box
    # center lies within `keepframe_margin_frac` of the half-frame height
    # (cam.cy px from center to top edge) of the TOP edge AND the capture-
    # instant NED conversion (mount_up_rad + vehicle attitude included, via
    # the same quat_cap path every measurement uses) confirms the target is
    # actually ABOVE the vehicle, a bounded climb term (negative NED down-
    # velocity, up to `keepframe_vz_max_ms`, proportional to how deep into
    # the margin the box sits) is ADDED to the command BEFORE the norm clip
    # and the slew -- so it never bypasses the speed cap or the accel
    # budgets. Active in Phase A and non-coasting Phase B. During dropout
    # the last term is HELD for at most `keepframe_hold_s`, then zeroed --
    # no open-loop climb beyond that; the KF prediction steers the coast.
    # Bottom edge is out of scope (the measured cliff is top-side only).
    # Inputs: box pixels + own-state attitude ONLY (honesty boundary).
    # Pre-registration + A/B: isim/specs/keepframe_prereg_2026-09-23.md.
    keepframe_assist: bool = False
    keepframe_margin_frac: float = 0.35   # fraction of cy (half-frame height)
    keepframe_vz_max_ms: float = 2.5      # max ADDED climb rate, m/s
    keepframe_hold_s: float = 0.5         # max hold of the last term, dropout

    # --- stopping-distance brake cap (2026-09-24, default OFF = legacy) -------
    # Attacks the "hot approach / late braking" residual on the slow honest
    # ENGAGE plant (isim/specs/brake_shaping_prereg_2026-09-24.md: fitted
    # braking authority 5.94 m/s^2, ~0.3 s velocity-loop lag, so a 16 m/s
    # approach needs ~28 m to stop while Phase B's schedule only tapers
    # inside 10 m). When ON, the RELATIVE command (cmd - believed target
    # velocity) is capped, direction preserved, by the speed the configured
    # deceleration can still shed before the remaining range runs out:
    #   closing   = max((own_vel - v_t) . unit(r), 0)
    #   d_eff     = max(range - brake_lead_s * closing, 0)
    #   v_rel_cap = sqrt(v_close_min_ms^2 + 2 * brake_accel_ms2 * d_eff)
    # Phase B: range = |KF r| (the target). Phase A: range = the believed
    # RENDEZVOUS point (r_aim, where the relative speed is meant to reach
    # zero), and only once that range is inside 2x the current stopping
    # envelope (closing^2 / (2*brake_accel_ms2) + brake_lead_s*closing).
    # Applied BEFORE the keepframe term, the norm clip and the slew. Does NOT
    # read lock quality (the ADR-0108 timidity rejection is staleness-
    # modulated closure, not range physics). Inputs: own-state velocity +
    # the belief/KF only (honesty boundary).
    brake_shaping: bool = False
    brake_accel_ms2: float = 4.0   # TODO-BUILDER estimate: under the fitted
    #                                5.94 m/s^2 ENGAGE braking authority with
    #                                margin; first real braking ULog replaces it
    brake_lead_s: float = 0.45     # TODO-BUILDER estimate: ~latency (0.14 s) +
    #                                1/kp (0.30 s) of the honest ENGAGE fit
    # AMENDMENT #1 (prereg doc): the cap's physics (braking authority, tilt,
    # drag) are HORIZONTAL, and the registered sweep measured the 3-D cap
    # scaling the CLIMB out of a climbing (alt-above) approach. True
    # (default): closing/range/cap on the horizontal components only, the
    # vertical command passes through. False = the flown 3-D cap, kept
    # reachable for the record.
    brake_horizontal_only: bool = True
    # AMENDMENT #2 (prereg doc, builder-ruled option (a)): Phase-B vertical
    # ARRIVAL SYNCHRONIZATION. The traced alt+3 failure is the climb
    # finishing early (vehicle 0.2-0.7 m ABOVE the target at CPA while the
    # braked horizontal closure lags). When ON, the vertical relative command
    # is scheduled from the HORIZONTAL time-to-go so both gaps zero together:
    #   t_go_h  = d_h / max(closing_h, v_close_min_ms)
    #   v_z_des = clamp(dz / t_go_h, +-v_max_ms)   (later budgets still apply)
    # Phase A is untouched (gross positioning; the overshoot is a Phase-B
    # terminal effect). False (default) = amendment-#1 behaviour exactly.
    brake_vert_sync: bool = False

    # --- rehearsal break-off (2026-09-24, default OFF = legacy) ---------------
    # PRACTICE mode, builder directive 2026-09-24: break off at the last
    # moment a collision is as certain as the track can make it, so many
    # scored passes fit on one battery/airframe before a real contact test.
    # Pre-registration: isim/specs/rehearsal_breakoff_prereg_2026-09-24.md.
    # TRIGGER (Phase B only, all three at once):
    #   * >= rehearsal_min_updates CONSUMED KF updates inside the trailing
    #     rehearsal_fresh_s (a converged, currently-fed track -- a coasting
    #     or phantom track must NOT trigger),
    #   * closing > 0 on the KF state (-(r . v_rel)/|r|),
    #   * |KF r| <= rehearsal_range_m.
    # At the trigger the KF state is recorded -- the onboard WOULD-HAVE
    # estimate is the ZEM (zero-effort miss: |r| minimised along the current
    # KF relative velocity) -- and the command switches to the EVADE, latched
    # at the trigger: full-authority climb (-v_max_ms down-velocity) plus a
    # lateral push of v_close_max_ms AWAY from the target's predicted path
    # (the side the vehicle already sits on, from KF v_t x (own - target);
    # fallback: right of the target's course), riding the KF target velocity
    # so the along-path closure is nulled; yaw held. The norm cap and the
    # Phase-B slew budgets still apply. After rehearsal_evade_s the terminal
    # raises `rehearsal_complete` and RealFlightSM ends the engagement via
    # SAFE with its own reason `rehearsal_breakoff` (a MISS-class SAFE:
    # hover per ADR-0107). Inputs: KF + own-state only (honesty boundary).
    rehearsal_breakoff: bool = False
    rehearsal_range_m: float = 2.5     # TODO-BUILDER estimate: isim-derived,
    #                                    set by the registered margin sweep
    #                                    (scripts/rehearsal_margin_sweep.py);
    #                                    an isim number until a Gazebo spot-
    #                                    check + reduced-speed field passes
    rehearsal_min_updates: int = 3     # TODO-BUILDER estimate: "certain" gate
    rehearsal_fresh_s: float = 0.5     # TODO-BUILDER estimate: ~5 decode
    #                                    intervals at the isim ~10 Hz decode rate
    rehearsal_evade_s: float = 1.5     # TODO-BUILDER estimate: evade duration
    #                                    before the SAFE hover


def _unit(v: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return (v / n) if n > 1e-9 else np.asarray(fallback, dtype=np.float64)


def _nlerp_quat(q0, q1, w: float):
    """Normalized linear interpolation between two (w,x,y,z) quaternions --
    exact enough for the tens-of-ms gaps the attitude ring buffer holds
    (the native prototype's `_interp_own_state` uses the same approach)."""
    a = np.asarray(q0, dtype=np.float64)
    b = np.asarray(q1, dtype=np.float64)
    if float(np.dot(a, b)) < 0.0:
        b = -b
    q = (1.0 - w) * a + w * b
    n = float(np.linalg.norm(q))
    return tuple(q / n) if n > 1e-12 else tuple(a)


def _interp_att_hist(hist, t_query: float):
    """Interpolate the (t, quat) ring buffer at `t_query`. Clamped at both
    ends (a query before the first sample returns the first sample)."""
    if not hist:
        return None
    if t_query <= hist[0][0]:
        return hist[0][1]
    for i in range(len(hist) - 1, 0, -1):
        t0, q0 = hist[i - 1]
        t1, q1 = hist[i]
        if t0 <= t_query <= t1:
            w = 0.0 if t1 <= t0 else (t_query - t0) / (t1 - t0)
            return _nlerp_quat(q0, q1, w)
    return hist[-1][1]


def _clip_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v if n <= max_norm or n < 1e-12 else v * (max_norm / n)


def _wrap_deg(a: float) -> float:
    return (float(a) + 180.0) % 360.0 - 180.0


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _yaw_toward(delta_ned: np.ndarray, fallback_deg: float) -> float:
    n = float(np.linalg.norm(delta_ned[0:2]))
    if n < 1e-6:
        return fallback_deg
    return math.degrees(math.atan2(float(delta_ned[1]), float(delta_ned[0])))


def _slew_combined(cmd: np.ndarray, prev: np.ndarray, accel_max_ms2: float,
                    dt: float) -> np.ndarray:
    """Phase A: ONE combined-norm slew budget across all three axes."""
    if dt <= 0.0:
        return prev.copy()
    step = np.clip(cmd - prev, -accel_max_ms2 * dt, accel_max_ms2 * dt)
    return prev + step


def _slew_split(cmd: np.ndarray, prev: np.ndarray, accel_horiz_ms2: float,
                 accel_vert_ms2: float, dt: float) -> np.ndarray:
    """Phase B: SEPARATE horizontal (north/east) / vertical (down) budgets."""
    if dt <= 0.0:
        return prev.copy()
    max_h = accel_horiz_ms2 * dt
    dv_h = cmd[0:2] - prev[0:2]
    n = float(np.linalg.norm(dv_h))
    if n > max_h and n > 1e-12:
        dv_h = dv_h * (max_h / n)
    dv_v = _clamp(float(cmd[2] - prev[2]), -accel_vert_ms2 * dt, accel_vert_ms2 * dt)
    return np.array([prev[0] + dv_h[0], prev[1] + dv_h[1], prev[2] + dv_v])


def _pursuit_measurement_noise(range_m: float, dir_ned: np.ndarray,
                                cfg: "PursuitTerminalConfig", cam: CameraModel,
                                tag_side_m: float) -> np.ndarray:
    """3x3 measurement-noise covariance for one fix, NED -- pursuit's OWN
    tuning (a `cross_sigma_floor_m` floor `tag_terminal.range_measurement_
    noise` does not have; see module docstring for why this is not shared
    code with the PIP terminal)."""
    r = max(range_m, 0.0)
    cross_sigma = max(r * cfg.sigma_px / cam.fx, cfg.cross_sigma_floor_m)
    along_sigma = (r ** 2) * cfg.sigma_side_px / (cam.fx * max(tag_side_m, 1e-6))
    n = np.asarray(dir_ned, dtype=np.float64)
    norm = float(np.linalg.norm(n))
    if norm < 1e-9:
        return np.eye(3) * max(cross_sigma, 1e-3) ** 2
    e_los = n / norm
    outer = np.outer(e_los, e_los)
    return (cross_sigma ** 2) * np.eye(3) + (along_sigma ** 2 - cross_sigma ** 2) * outer


class _ConstVelKF:
    """6-state (r, v_t) constant-velocity Kalman filter in RELATIVE
    coordinates -- `r` = target position MINUS own position, `v_t` = the
    target's ABSOLUTE velocity, NED. Predict takes the CURRENT own velocity
    as a known control input (`r' = r + (v_t - v_own)*dt`), exactly
    `flight.tag_terminal._RelStateKF`'s pattern, so this filter never needs
    (and never accumulates) an absolute own-position estimate. Deliberately
    NOT `_RelStateKF` itself -- see module docstring for why the tuning is
    not shared."""

    def __init__(self, q_accel_ms2: float) -> None:
        self.q = q_accel_ms2 ** 2
        self.x = np.zeros(6)
        self.P = np.eye(6)
        self.initialized = False

    def init(self, r0: np.ndarray, v_t0: np.ndarray, p0_pos: float, p0_vel: float) -> None:
        self.x = np.concatenate([np.asarray(r0, dtype=np.float64),
                                  np.asarray(v_t0, dtype=np.float64)])
        self.P = np.diag([p0_pos ** 2] * 3 + [p0_vel ** 2] * 3)
        self.initialized = True

    def predict(self, dt: float, v_own: np.ndarray) -> None:
        if dt <= 0.0 or not self.initialized:
            return
        r, v_t = self.x[0:3], self.x[3:6]
        r = r + (v_t - np.asarray(v_own, dtype=np.float64)) * dt
        self.x = np.concatenate([r, v_t])
        # COVARIANCE JACOBIAN (parity-trace fix, isim/specs/parity_trace_
        # 2026-09-22.md): dr'/dv_t = dt*I even though the STATE was advanced
        # explicitly above -- the known own-velocity term is a control INPUT
        # (no state dependence, so it does not appear in F), but v_t's effect
        # on r' does. The previous F = I propagated the state with the
        # coupling and the covariance without it, so P never built the
        # pos-vel correlation the update needs to correct v_t from position
        # fixes, and the traced velocity estimate diverged to 5-6.5 m/s of
        # error by CPA (native prototype, correct F: 0.05-0.12 m/s).
        F = np.eye(6)
        F[0:3, 3:6] = np.eye(3) * dt
        qpp, qpv, qvv = (dt ** 3 / 3.0) * self.q, (dt ** 2 / 2.0) * self.q, dt * self.q
        Q = np.zeros((6, 6))
        Q[0:3, 0:3] = np.eye(3) * qpp
        Q[0:3, 3:6] = np.eye(3) * qpv
        Q[3:6, 0:3] = np.eye(3) * qpv
        Q[3:6, 3:6] = np.eye(3) * qvv
        self.P = F @ self.P @ F.T + Q

    def update(self, z_r: np.ndarray, R: np.ndarray, age_s: float = 0.0,
               v_own: Optional[np.ndarray] = None) -> None:
        """`z_r` is the relative position AT CAPTURE (age_s ago), while the
        state is "now". With r(t) = p_tgt(t) - p_own(t) and a CV target:

            z = r(t) - v_t*age + v_own*age    (own displacement over the age,
                                               approximated with the current
                                               own velocity -- a known input)

        so H = [I, -age*I] and the known v_own*age term joins the predicted
        measurement. This replaces the old pre-extrapolation of z by
        `v_rel_hat*age` with H = [I, 0], which had the same mean but gave
        position fixes no gain into the velocity states (parity-trace fix,
        with the predict() Jacobian above)."""
        H = np.zeros((3, 6))
        H[:, 0:3] = np.eye(3)
        H[:, 3:6] = -float(age_s) * np.eye(3)
        vo = np.zeros(3) if v_own is None else np.asarray(v_own, dtype=np.float64)
        z_pred = H @ self.x + vo * float(age_s)
        y = np.asarray(z_r, dtype=np.float64) - z_pred
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    @property
    def r(self) -> np.ndarray:
        return self.x[0:3].copy()

    @property
    def v_t(self) -> np.ndarray:
        return self.x[3:6].copy()


class PursuitTerminalGuidance:
    """SAME CONTRACT AS `TagInterceptGuidance` (duck-typed, no `isinstance`
    check anywhere on `RealFlightSM.guidance`):

        step(det_box_xywh, own: OwnState, t: float) -> (Optional[Setpoint], StepTelemetry)

    `step()` never raises: a fault is reported on `StepTelemetry.health`;
    returning `(None, tel)` means "no trustworthy command" -- the caller
    holds the last dash velocity, exactly as for `SeekerGuidance`/
    `TagInterceptGuidance`.

    POSE-RANGE CHANNEL (2026-09-23, pursuit-scoped ONLY -- the S2 fix of
    isim/specs/xcheck_tick_trace_2026-09-23.md): `step()` additionally
    accepts an OPTIONAL keyword `det_range_pose_m` -- the same detection's
    detector-PnP slant range (camera pixels + known tag size, honesty-clean:
    the same input class as the box itself). The class attribute
    `SUPPORTS_POSE_RANGE = True` advertises this so `RealFlightSM` can pass
    it without breaking the other duck-typed terminals (whose AABB range
    channel is under the separate ADR-0105 open ruling and is deliberately
    NOT changed here). `det_range_pose_m=None` (or calling with the old
    3-argument form) is byte-identical to the pre-change module.
    """

    #: `RealFlightSM._step_engage` passes `det_range_pose_m=` only when the
    #: terminal advertises it -- SeekerGuidance/TagInterceptGuidance do not.
    SUPPORTS_POSE_RANGE = True

    def __init__(self, cfg: PursuitTerminalConfig, cam: CameraModel, tag_side_m: float,
                 gcfg: GuidanceConfig, belief_r0_ned, belief_vel0_ned,
                 go_at_s: float, initial_yaw_deg: float = 0.0) -> None:
        self.cfg = cfg
        self.cam = cam
        self.tag_side_m = tag_side_m
        self.gcfg = gcfg
        self.go_at_s = float(go_at_s)
        self.initial_yaw_deg = float(initial_yaw_deg)
        self._belief_vel0 = np.asarray(belief_vel0_ned, dtype=np.float64).copy()
        # Phase A running state: `r_track` = target belief position minus own
        # position (relative, per the module docstring); `v_track` = the
        # believed target velocity (constant while in Phase A).
        self._r_track = np.asarray(belief_r0_ned, dtype=np.float64).copy()
        self._v_track = self._belief_vel0.copy()
        self._phase = "A"
        self._kf = _ConstVelKF(cfg.kf_q_accel_ms2)
        # (t, quat) ring buffer for the capture-instant attitude lookup --
        # filled from every step()'s own-state, module docstring LATENCY (1).
        self._att_hist: List[Tuple[float, Tuple[float, float, float, float]]] = []
        self._decode_positions: List[Tuple[float, np.ndarray]] = []
        self._last_decode_t: float = -math.inf
        self._last_t: Optional[float] = None
        self._prev_v_cmd = np.zeros(3)
        self._prev_yaw_deg = self.initial_yaw_deg
        # The yaw-slew seed is re-anchored to the vehicle's OWN EKF yaw on the
        # first post-GO tick (parity-trace fix: both real callers construct
        # this object without `initial_yaw_deg`, so the slew walked the yaw
        # command from 0 deg through a ~75 deg off-target sweep at ENGAGE
        # entry and the camera missed the whole first close pass of decodes).
        self._yaw_seeded = False
        # Keep-in-frame vertical assist state (cfg.keepframe_assist; inert --
        # never read or written -- while the flag is off).
        self._keepframe_vz = 0.0
        self._keepframe_t: float = -math.inf
        # Rehearsal break-off state (cfg.rehearsal_breakoff; inert -- never
        # appended to, and the trigger never evaluated -- while the flag is
        # off). The public attributes are the mode's telemetry: RealFlightSM
        # reads `rehearsal_trigger_t`/`rehearsal_complete` duck-typed (absent
        # on every other terminal) and writes the numbers into its mission
        # log line; the isim sweep scores them.
        self._rehearsal_upd_t: List[float] = []
        self._rehearsal_v_ff = np.zeros(3)
        self._rehearsal_lat = np.zeros(3)
        self.rehearsal_trigger_t: Optional[float] = None
        self.rehearsal_trigger_range_m: Optional[float] = None
        self.rehearsal_zem_m: Optional[float] = None     # onboard would-have
        self.rehearsal_t_go_s: Optional[float] = None
        self.rehearsal_side: Optional[str] = None        # "left"/"right"/"right_fallback"
        self.rehearsal_complete = False
        self._warned: set = set()
        self.n_track_broken = 0

    def _emit_fault(self, tel: StepTelemetry, flag: str, msg: str, once: bool = False) -> None:
        tel.health.append(flag)
        if not once or flag not in self._warned:
            self._warned.add(flag)
            print(f"[pursuit_terminal] FAULT {flag}: {msg}")

    # ------------------------------------------------------------- geometry

    def _measured_r_ned(self, det_box_xywh, quat, range_override_m=None
                         ) -> Tuple[np.ndarray, np.ndarray, float]:
        """-> (measured target-relative-to-own NED vector, unit LOS
        direction NED, range_m). `_optical_vec_to_ned`/`_cam_offset_ned`
        already yield a vector relative to the vehicle's own CG -- no own-
        position subtraction needed (see module docstring).

        `range_override_m` (POSE-RANGE CHANNEL, 2026-09-23 tick-trace fix S2,
        isim/specs/xcheck_tick_trace_2026-09-23.md): an externally supplied
        SLANT range (camera origin -> tag centre) for THIS SAME detection --
        in practice the AprilTag detector's own PnP pose range, measured at
        -0.03 m bias / 0.109 m spread over 415 live Gazebo detections while
        the box-width channel below under-ranged 15-25% (the AABB of a
        rotated/perspective tag is wider than the pinhole side). When
        present, the DIRECTION still comes from the box centre (undistorted
        unit ray, unchanged) and only the vector's LENGTH is set from the
        override; the slant correction is skipped because a PnP |t| already
        IS the slant range, not an optical-axis depth. When None the
        box-width path below runs byte-identically to before this parameter
        existed."""
        _bearing_h, range_m, meas_xyz = measurement_from_box(
            det_box_xywh, self.gcfg, self.cam, self.tag_side_m)
        if range_override_m is not None and math.isfinite(range_override_m) \
                and range_override_m > 0.0:
            unit_ray = np.asarray(meas_xyz, dtype=np.float64) / max(range_m, 1e-9)
            meas_xyz = unit_ray * float(range_override_m)
            range_m = float(range_override_m)
        else:
            # SLANT CORRECTION (parity-trace fix, candidate (e)): the box width
            # measures the pinhole DEPTH z (side_px = fx*side/z), but
            # `measurement_from_box` places that depth along the UNIT ray, leaving
            # the vector short of the true slant by cos(off-axis) -- a systematic
            # 0.15-0.37 m along-LOS under-range in the traced close. The unit
            # ray's z-component IS cos(off-axis), so dividing by it restores
            # v = (x_n, y_n, 1)*z exactly.
            ray_z = float(meas_xyz[2]) / max(range_m, 1e-9)
            if ray_z > 1e-6:
                meas_xyz = np.asarray(meas_xyz, dtype=np.float64) / ray_z
                range_m = range_m / ray_z
        cam_to_tgt_ned = _optical_vec_to_ned(meas_xyz, quat, self.gcfg.mount_up_rad)
        cam_ofs_ned = _cam_offset_ned(quat, self.gcfg.cam_offset_body)
        r_ned = cam_to_tgt_ned + cam_ofs_ned          # target relative to own CG, NED
        dir_ned = _unit(r_ned, np.array([1.0, 0.0, 0.0]))
        return r_ned, dir_ned, range_m

    # ------------------------------------------------------------------ step

    def step(self, det_box_xywh, own: OwnState, t: float,
              det_range_pose_m: Optional[float] = None
              ) -> Tuple[Optional[Setpoint], StepTelemetry]:
        cfg = self.cfg
        dt = 0.05 if self._last_t is None else max(1e-3, t - self._last_t)
        self._last_t = t

        tel = StepTelemetry(t=t, detected=det_box_xywh is not None)
        tel.own_age_s = own.age_s

        own_ok, own_why = own_state_status(own, self.gcfg)
        tel.own_state_ok = own_ok
        if not own_ok:
            self._emit_fault(
                tel, own_why,
                "own-state EKF unavailable/stale -- pursuit_terminal refusing "
                "to steer (see GuidanceConfig.require_own_attitude)", once=True)
            return None, tel

        quat = own.quat
        # Capture-instant attitude (LATENCY (1)): buffer the current attitude,
        # then look up the attitude at the assumed exposure instant
        # `t - meas_latency_s` for the pixel->NED conversion. Falls back to
        # the current quat while the buffer is still filling.
        self._att_hist.append((t, tuple(float(c) for c in quat)))
        cutoff_att = t - max(self.cfg.att_history_s, 2.0 * self.cfg.meas_latency_s)
        while len(self._att_hist) > 2 and self._att_hist[0][0] < cutoff_att:
            self._att_hist.pop(0)
        quat_cap = _interp_att_hist(self._att_hist, t - self.cfg.meas_latency_s) \
            if self.cfg.meas_latency_s else quat
        if quat_cap is None:
            quat_cap = quat
        if own.vel_ned is not None:
            own_vel = np.array(own.vel_ned, dtype=float)
        else:
            own_vel = self._prev_v_cmd.copy()
            self._emit_fault(
                tel, "own_vel_fallback",
                "OwnState.vel_ned is None -- falling back to the last "
                "COMMANDED velocity for the relative-velocity bookkeeping",
                once=True)

        if t < self.go_at_s:
            tel.phase = "STANDBY"
            self._prev_v_cmd = np.zeros(3)
            self._prev_yaw_deg = self.initial_yaw_deg
            sp = Setpoint(0.0, 0.0, 0.0, self.initial_yaw_deg)
            tel.setpoint = sp
            return sp, tel

        # First post-GO tick: seed the yaw-slew state from the OWN EKF yaw
        # (own-state, honesty-clean) -- the vehicle is holding the standby aim
        # yaw at the GO edge, and slewing from the constructor default instead
        # points the camera away exactly when Phase A needs it on the belief.
        if not self._yaw_seeded:
            self._yaw_seeded = True
            if own.psi_rad is not None:
                self._prev_yaw_deg = math.degrees(float(own.psi_rad))

        # Phase A's relative belief propagates on EVERY tick (own_vel is
        # known instantaneously; no absolute position needed) -- this is the
        # same "control-input predict" the KF uses, just without a filter
        # (nothing to correct yet in Phase A).
        if self._phase == "A":
            self._r_track = self._r_track + (self._v_track - own_vel) * dt

        if det_box_xywh is not None:
            meas_r, _dir, _rng = self._measured_r_ned(det_box_xywh, quat_cap,
                                                       det_range_pose_m)
            self._decode_positions.append((t, meas_r))
            cutoff = t - cfg.acquire_window_s
            self._decode_positions = [p for p in self._decode_positions if p[0] >= cutoff]
            if cfg.keepframe_assist:
                self._keepframe_update(det_box_xywh, meas_r, t)

        if self._phase == "A" and len(self._decode_positions) >= cfg.acquire_n:
            self._start_phase_b(t)

        if self._phase == "B":
            self._kf.predict(dt, own_vel)
            if det_box_xywh is not None:
                self._update_kf(det_box_xywh, quat_cap, own_vel, t,
                                det_range_pose_m)
            range_est = float(np.linalg.norm(self._kf.r))
            dropout_s = t - self._last_decode_t
            if dropout_s > cfg.fallback_s and range_est >= cfg.no_fallback_range_m:
                safe_vel = self._sane_fallback_vel(self._kf.v_t)
                self._r_track = self._kf.r.copy()
                self._v_track = safe_vel
                self._phase = "A"

        broken = self._phase == "B" and not (
            np.all(np.isfinite(self._kf.r)) and np.all(np.isfinite(self._kf.v_t)))
        if broken:
            tel.track_broken = True
            self.n_track_broken += 1
            self._emit_fault(tel, "track_broken",
                              "relative-position/velocity estimate is non-finite",
                              once=True)
            return None, tel

        if cfg.rehearsal_breakoff and self._phase == "B" \
                and self.rehearsal_trigger_t is None:
            self._rehearsal_check(t, own_vel)

        v_cap = self._speed_cap()
        if self.rehearsal_trigger_t is not None:
            # REHEARSAL EVADE (cfg.rehearsal_breakoff docstring): the latched
            # escape command under the same norm cap and Phase-B slew
            # budgets; yaw held at the last command. Checked before the phase
            # branch so a (rare) Phase-B->A fallback cannot resume the chase.
            cmd_v = self._rehearsal_v_ff + self._rehearsal_lat * cfg.v_close_max_ms \
                + np.array([0.0, 0.0, -cfg.v_max_ms])
            cmd_v = _clip_norm(cmd_v, v_cap)
            cmd_v = _slew_split(cmd_v, self._prev_v_cmd,
                                 cfg.accel_max_horiz_b_ms2,
                                 cfg.accel_max_vert_b_ms2, dt)
            yaw = self._prev_yaw_deg
            tel.phase = self._phase
            if self._phase == "B":
                tel.r_hat_m = float(np.linalg.norm(self._kf.r))
            if (t - self.rehearsal_trigger_t) >= cfg.rehearsal_evade_s:
                self.rehearsal_complete = True
        elif self._phase == "A":
            cmd_v, yaw = self._phase_a_cmd()
            if cfg.brake_shaping:
                cmd_v = self._brake_cap_phase_a(cmd_v, own_vel)
            if cfg.keepframe_assist:
                # ADDED before the norm clip and the slew: the assist trades
                # horizontal speed for climb under the same caps/budgets.
                cmd_v = cmd_v + np.array([0.0, 0.0, -self._keepframe_term(t)])
            cmd_v = _clip_norm(cmd_v, v_cap)
            cmd_v = _slew_combined(cmd_v, self._prev_v_cmd, cfg.accel_max_ms2, dt)
            tel.phase = "A"
        else:
            cmd_v, yaw, coasting = self._phase_b_cmd(own_vel)
            if cfg.keepframe_assist and not coasting:
                # Not during terminal coast -- the held-command latch must
                # stay byte-frozen (mirrors TagInterceptGuidance semantics).
                cmd_v = cmd_v + np.array([0.0, 0.0, -self._keepframe_term(t)])
            cmd_v = _clip_norm(cmd_v, v_cap)
            if not coasting:
                cmd_v = _slew_split(cmd_v, self._prev_v_cmd,
                                     cfg.accel_max_horiz_b_ms2,
                                     cfg.accel_max_vert_b_ms2, dt)
            tel.phase = "B"
            tel.r_hat_m = float(np.linalg.norm(self._kf.r))
            tel.terminal_coast = coasting

        yaw = self._slew_yaw(yaw, dt)
        self._prev_v_cmd = cmd_v
        self._prev_yaw_deg = yaw

        sp = Setpoint(float(cmd_v[0]), float(cmd_v[1]), float(cmd_v[2]), yaw)
        tel.setpoint = sp
        return sp, tel

    # --------------------------------------------------------------- helpers

    def _slew_yaw(self, yaw_target: float, dt: float) -> float:
        delta = _wrap_deg(yaw_target - self._prev_yaw_deg)
        max_step = self.cfg.yaw_rate_max_deg_s * dt
        delta = _clamp(delta, -max_step, max_step)
        return _wrap_deg(self._prev_yaw_deg + delta)

    # ------------------------------------------------ adaptive speed governor

    def _speed_cap(self) -> float:
        """Effective norm cap for this tick's velocity command. Legacy
        (adaptive_speed=False): the fixed v_max_ms. Adaptive: sized from the
        believed target speed plus the overtake margin -- Phase A reads the
        belief track, Phase B the KF's target-velocity estimate passed
        through the same sanity clamp Phase-A fallback uses (a corrupted KF
        velocity must not command the hardware ceiling)."""
        cfg = self.cfg
        if not cfg.adaptive_speed:
            return cfg.v_max_ms
        if self._phase == "B":
            tgt_speed = float(np.linalg.norm(self._sane_fallback_vel(self._kf.v_t)))
        else:
            tgt_speed = float(np.linalg.norm(self._v_track))
        return _clamp(tgt_speed + cfg.overtake_margin_ms,
                      cfg.adaptive_v_floor_ms, cfg.v_hw_max_ms)

    # ------------------------------------------- keep-in-frame vertical assist

    def _keepframe_update(self, det_box_xywh, meas_r_ned: np.ndarray, t: float) -> None:
        """Refresh the assist term from ONE fresh decode (called only with
        cfg.keepframe_assist on). Trigger: box CENTER within
        `keepframe_margin_frac * cam.cy` px of the TOP edge (cy = principal-
        point-to-top-edge distance, the half-frame height) AND the capture-
        instant NED measurement (mount tilt + vehicle attitude already folded
        in by `_measured_r_ned`) says the target is ABOVE own CG (down < 0) --
        the attitude gate stops a transient nose-down pitch, which also pushes
        the box toward the top edge, from commanding a spurious climb. A fresh
        decode that does NOT trigger CANCELS the assist immediately (a
        centered target needs no help). Inputs: camera pixels + own-state
        attitude only -- never target ground truth."""
        cfg = self.cfg
        _x0, y0, _bw, bh = det_box_xywh
        v_center_px = float(y0) + float(bh) / 2.0
        margin_px = cfg.keepframe_margin_frac * self.cam.cy
        if v_center_px < margin_px and float(meas_r_ned[2]) < 0.0:
            depth = _clamp((margin_px - v_center_px) / max(margin_px, 1e-6),
                           0.0, 1.0)
            self._keepframe_vz = depth * cfg.keepframe_vz_max_ms
            self._keepframe_t = t
        else:
            self._keepframe_vz = 0.0
            self._keepframe_t = -math.inf

    def _keepframe_term(self, t: float) -> float:
        """Climb magnitude (m/s, >= 0; caller applies it as NEGATIVE NED down)
        for this tick: the last decode's term, held at most
        `keepframe_hold_s` past that decode, then zero -- no open-loop climb
        beyond the short clamped hold; the KF prediction steers a coast."""
        if (t - self._keepframe_t) <= self.cfg.keepframe_hold_s:
            return self._keepframe_vz
        return 0.0

    def _phase_a_cmd(self) -> Tuple[np.ndarray, float]:
        """`self._r_track` is already RELATIVE (target belief minus own
        position), so the aim offset (a constant `d_behind_m` behind along
        the believed track) needs no separate integrator: `r_aim = r_track -
        d_behind_m*track_dir` has exactly `r_track`'s own dynamics, since the
        subtracted term is constant."""
        cfg = self.cfg
        track_dir = _unit(self._v_track, np.array([1.0, 0.0, 0.0]))
        r_aim = self._r_track - cfg.d_behind_m * track_dir
        cmd_v = self._v_track + cfg.kp_pos * r_aim
        # Yaw at the believed TARGET (self._r_track), matching the prototype
        # (isim.concepts: _yaw_toward(pos_belief - own_pos, ...)). Yawing at
        # r_aim instead froze the camera off-target the moment the vehicle
        # station-kept at the rendezvous point (r_aim ~ 0 -> fallback holds the
        # arrival bearing), so Phase B could never start -- found by the Gazebo
        # cross-check flight 1, 2026-09-23 (docs/xcheck_gazebo_pursuit_prereg.md);
        # invisible to the isim parity grid, whose approach geometry left the
        # frozen bearing pointing along-track anyway.
        yaw = _yaw_toward(self._r_track, self._prev_yaw_deg)
        return cmd_v, yaw

    def _phase_b_cmd(self, own_vel: np.ndarray) -> Tuple[np.ndarray, float, bool]:
        """-> (cmd_v, yaw, coasting). `coasting=True` means the previous
        command is held unchanged (inside `hold_range_m` -- "fly through";
        mirrors `TagInterceptGuidance`'s `freeze_range_m` latch semantics so
        `RealFlightSM` reads `tel.terminal_coast` the same way for either
        terminal)."""
        cfg = self.cfg
        r_est, v_t = self._kf.r, self._kf.v_t
        range_est = float(np.linalg.norm(r_est))
        if range_est < cfg.hold_range_m:
            return self._prev_v_cmd.copy(), self._prev_yaw_deg, True

        los_dir = _unit(r_est, np.array([1.0, 0.0, 0.0]))
        v_close = _clamp(cfg.k_close * range_est, cfg.v_close_min_ms, cfg.v_close_max_ms)
        v_rel = v_t - own_vel
        ref_dir = _unit(v_rel, _unit(v_t, los_dir))
        r_perp = r_est - float(np.dot(r_est, ref_dir)) * ref_dir

        cmd_v = v_t + v_close * los_dir + cfg.kp_lat * r_perp
        if cfg.brake_shaping:
            cmd_v = self._brake_cap(cmd_v, v_t, r_est, own_vel)
            if cfg.brake_vert_sync:
                cmd_v = self._brake_sync_vertical(cmd_v, v_t, r_est, own_vel)
        yaw = _yaw_toward(r_est, self._prev_yaw_deg)
        return cmd_v, yaw, False

    # ------------------------------------------ stopping-distance brake cap

    def _brake_closing(self, r_ned: np.ndarray, v_t: np.ndarray,
                        own_vel: np.ndarray) -> float:
        """Own speed RELATIVE to the target, projected on the direction to
        `r_ned`, floored at 0 (an opening geometry needs no braking lead)."""
        dir_ned = _unit(r_ned, np.array([1.0, 0.0, 0.0]))
        return max(float(np.dot(np.asarray(own_vel, dtype=np.float64) - v_t, dir_ned)),
                   0.0)

    def _brake_rel_cap(self, range_m: float, closing_ms: float) -> float:
        """The relative speed the configured deceleration can still shed
        before `range_m` runs out, with the plant's response lead taken off
        the range first (cfg.brake_shaping docstring)."""
        cfg = self.cfg
        d_eff = max(range_m - cfg.brake_lead_s * closing_ms, 0.0)
        return math.sqrt(cfg.v_close_min_ms ** 2 + 2.0 * cfg.brake_accel_ms2 * d_eff)

    def _brake_cap(self, cmd_v: np.ndarray, v_t: np.ndarray, r_ned: np.ndarray,
                   own_vel: np.ndarray) -> np.ndarray:
        """Cap |cmd_v - v_t| at `_brake_rel_cap(|r_ned|, closing)`,
        direction preserved. Called only with cfg.brake_shaping on."""
        v_t = np.asarray(v_t, dtype=np.float64)
        if self.cfg.brake_horizontal_only:
            # Amendment #1: horizontal-only. Closing/range on the horizontal
            # plane; only the horizontal part of the relative command is
            # capped, the vertical passes through (a climbing approach keeps
            # its climb -- the measured alt+3 failure of the 3-D cap).
            r_h = np.array([r_ned[0], r_ned[1], 0.0])
            rel = np.asarray(cmd_v, dtype=np.float64) - v_t
            rel_h = np.array([rel[0], rel[1], 0.0])
            closing = self._brake_closing(r_h, v_t, own_vel)
            cap = self._brake_rel_cap(float(np.linalg.norm(r_h)), closing)
            return v_t + _clip_norm(rel_h, cap) + np.array([0.0, 0.0, rel[2]])
        closing = self._brake_closing(r_ned, v_t, own_vel)
        cap = self._brake_rel_cap(float(np.linalg.norm(r_ned)), closing)
        return v_t + _clip_norm(cmd_v - v_t, cap)

    def _brake_cap_phase_a(self, cmd_v: np.ndarray, own_vel: np.ndarray) -> np.ndarray:
        """Phase A: the relative command toward the believed RENDEZVOUS point
        (`r_aim`, where the relative speed is meant to reach zero), capped
        only once that range is inside 2x the current stopping envelope --
        further out the vehicle may still accelerate freely."""
        cfg = self.cfg
        track_dir = _unit(self._v_track, np.array([1.0, 0.0, 0.0]))
        r_aim = self._r_track - cfg.d_behind_m * track_dir
        r_gate = np.array([r_aim[0], r_aim[1], 0.0]) \
            if cfg.brake_horizontal_only else r_aim
        closing = self._brake_closing(r_gate, self._v_track, own_vel)
        d_stop = closing ** 2 / (2.0 * cfg.brake_accel_ms2) + cfg.brake_lead_s * closing
        if float(np.linalg.norm(r_gate)) > 2.0 * d_stop:
            return cmd_v
        return self._brake_cap(cmd_v, self._v_track, r_aim, own_vel)

    def _brake_sync_vertical(self, cmd_v: np.ndarray, v_t: np.ndarray,
                             r_ned: np.ndarray, own_vel: np.ndarray) -> np.ndarray:
        """Amendment #2 (cfg.brake_vert_sync docstring): replace the vertical
        relative command with dz / t_go_h so the climb finishes WITH the
        horizontal closure. Called only with brake_shaping AND brake_vert_sync
        on, Phase B only."""
        cfg = self.cfg
        v_t = np.asarray(v_t, dtype=np.float64)
        r_h = np.array([r_ned[0], r_ned[1], 0.0])
        d_h = float(np.linalg.norm(r_h))
        closing_h = self._brake_closing(r_h, v_t, own_vel)
        t_go = max(d_h, 1e-6) / max(closing_h, cfg.v_close_min_ms)
        v_z_des = _clamp(float(r_ned[2]) / max(t_go, 1e-6),
                         -cfg.v_max_ms, cfg.v_max_ms)
        out = np.asarray(cmd_v, dtype=np.float64).copy()
        out[2] = float(v_t[2]) + v_z_des
        return out

    # ------------------------------------------------ rehearsal break-off

    def _rehearsal_gate_ok(self, t: float) -> bool:
        """The "as certain as possible" gate: at least rehearsal_min_updates
        consumed KF updates inside the trailing rehearsal_fresh_s. A coasting
        track (no recent updates) fails it by construction."""
        cfg = self.cfg
        cutoff = t - cfg.rehearsal_fresh_s
        self._rehearsal_upd_t = [u for u in self._rehearsal_upd_t if u >= cutoff]
        return len(self._rehearsal_upd_t) >= cfg.rehearsal_min_updates

    def _rehearsal_check(self, t: float, own_vel: np.ndarray) -> None:
        """Phase B, flag on, not yet triggered: evaluate the trigger on the
        CURRENT KF state and, if it fires, record the onboard would-have
        estimate and latch the evade reference (cfg.rehearsal_breakoff
        docstring)."""
        cfg = self.cfg
        if not self._rehearsal_gate_ok(t):
            return
        r = self._kf.r
        rng = float(np.linalg.norm(r))
        if not (rng <= cfg.rehearsal_range_m) or rng < 1e-9:
            return
        v_t = self._sane_fallback_vel(self._kf.v_t)
        v_rel = self._kf.v_t - np.asarray(own_vel, dtype=np.float64)
        closing = -float(np.dot(r, v_rel)) / rng
        if not closing > 0.0:
            return
        # Onboard WOULD-HAVE estimate: the ZEM, |r + v_rel*t_go| at the
        # t_go minimising it along the current KF relative velocity.
        vv = float(np.dot(v_rel, v_rel))
        t_go = max(-float(np.dot(r, v_rel)) / vv, 0.0) if vv > 1e-12 else 0.0
        self.rehearsal_zem_m = float(np.linalg.norm(r + v_rel * t_go))
        self.rehearsal_t_go_s = t_go
        self.rehearsal_trigger_t = t
        self.rehearsal_trigger_range_m = rng
        # Evade reference, latched. Side: the vehicle's horizontal offset
        # from the target (-r) against the target's KF course; NED cross_z > 0
        # = the vehicle sits RIGHT of the path -> push further right.
        course = _unit(np.array([v_t[0], v_t[1], 0.0]),
                       _unit(np.array([r[0], r[1], 0.0]),
                             np.array([1.0, 0.0, 0.0])))
        right = np.array([-course[1], course[0], 0.0])
        cross_z = float(course[0] * (-r[1]) - course[1] * (-r[0]))
        if cross_z > 0.05:
            self._rehearsal_lat, self.rehearsal_side = right, "right"
        elif cross_z < -0.05:
            self._rehearsal_lat, self.rehearsal_side = -right, "left"
        else:
            self._rehearsal_lat, self.rehearsal_side = right, "right_fallback"
        self._rehearsal_v_ff = np.array([v_t[0], v_t[1], 0.0])

    def _sane_fallback_vel(self, kf_v_t: np.ndarray) -> np.ndarray:
        """Clamp a KF-derived velocity before trusting it as a fresh Phase-A
        belief: a velocity estimate seeded/corrupted by a brief noisy
        acquisition can be tens of m/s off, and Phase A has no further
        correction once it adopts one. Clamp to a multiple of the ORIGINAL
        pre-flight believed target speed (never a live read)."""
        belief_speed = float(np.linalg.norm(self._belief_vel0))
        cap = max(belief_speed * self.cfg.fallback_speed_cap_mult,
                   self.cfg.fallback_speed_floor_ms)
        n = float(np.linalg.norm(kf_v_t))
        return kf_v_t * (cap / n) if n > cap else kf_v_t

    def _update_kf(self, det_box_xywh, quat_cap, own_vel: np.ndarray, t: float,
                   det_range_pose_m: Optional[float] = None) -> None:
        """`quat_cap` is the attitude at the ASSUMED capture instant
        (t - meas_latency_s), so `meas_r` is the relative position AT
        CAPTURE; the KF's measurement model carries the age explicitly
        (see `_ConstVelKF.update`) instead of pre-extrapolating here.
        `det_range_pose_m`: see `_measured_r_ned` (the measurement-NOISE
        model is deliberately left on the box-width formula either way --
        conservative at range for the pose channel, and re-tuning R is a
        separate, pre-registerable decision, not part of the bias fix)."""
        meas_r, dir_ned, range_m = self._measured_r_ned(det_box_xywh, quat_cap,
                                                         det_range_pose_m)
        r_noise = _pursuit_measurement_noise(range_m, dir_ned, self.cfg, self.cam,
                                              self.tag_side_m)
        self._kf.update(meas_r, r_noise, age_s=self.cfg.meas_latency_s,
                        v_own=own_vel)
        self._last_decode_t = t
        if self.cfg.rehearsal_breakoff:
            self._rehearsal_upd_t.append(t)

    def _start_phase_b(self, t: float) -> None:
        r0 = self._decode_positions[-1][1]
        self._kf.init(r0, self._v_track, self.cfg.kf_p0_pos_m, self.cfg.kf_p0_vel_ms)
        self._last_decode_t = self._decode_positions[-1][0]
        self._phase = "B"
