"""isim-native "pursuit" engagement concept (see `isim/specs/pursuit_concept.md`).

WHY (measured in isim, 2026-09-17). The deployed "flyby" concept is a
high-speed open-loop dash across the target's path, closing at roughly
19 m/s. A 0.30 m AprilTag decodes only inside about 13 m at fx=933 px, so the
camera gets about 0.6 s and about 15 decodes, and the quad's nose-down pitch
while accelerating hides the tag from the seeker. No terminal law can absorb
2 m of height error or 15 deg of aim error in 0.6 s.

CONCEPT ("pursuit"): arrive slowly instead of crossing fast.

  Phase A (no tag seen yet) flies to a moving aim point `d_behind_m` behind
  the PRE-FLIGHT BELIEF of the target -- no tag needed, using only the same
  belief information the "flyby" concept's heading solve is given (see
  `isim.scenario.build()`).

  Phase B (tag acquired: >= `acquire_n` detections within `acquire_window_s`)
  estimates the target's absolute NED position/velocity with a small
  constant-velocity Kalman filter fed by `Detection.range_m/bearing_deg/
  elevation_deg` plus the vehicle's own attitude quaternion and the
  (nominal, guidance-side) camera mount tilt, and closes on the estimate with
  a range-scheduled closing speed (slow and deliberate in the final metres --
  see `PursuitConfig.k_close`) plus a lateral correction on the component of
  the relative position that is perpendicular to the current relative-
  velocity direction (that component IS the eventual miss, for two straight-
  line tracks). Inside `hold_range_m` the last command is held ("fly
  through"); inside `no_fallback_range_m` a dropout never triggers a
  fallback (the tag legitimately fills/leaves the frame at very close range
  -- that is not a real track loss). A dropout longer than `fallback_s`
  outside that range reverts to a fresh Phase A, seeded from the KF's last
  estimate (speed-capped -- see `_sane_fallback_vel`).

v2 (isim/specs/pursuit_concept_v2.md) fixed two bugs found by re-measuring
with a longer scoring window (`isim.scenario.Scenario.pursuit_window_s`):
seeding the KF's initial velocity from a noise-amplifying 2-point finite
difference, and trusting an unbounded KF velocity snapshot as a permanent
new belief on fallback. See `_start_phase_b`/`_sane_fallback_vel` and the
task's final report for the trace evidence.

HONESTY. `PursuitRendezvousGuidance.step()` receives only a `VehicleState`
and an `Optional[Detection]`, per `isim.types.Guidance` -- never a
`TargetState` or a `FrameReport`. Its Phase-A belief trajectory
(`belief_pos0_ned`/`belief_vel_ned`) is a plain constructor argument computed
by the CALLER (`isim.scenario.build()`) from the same pre-flight-belief
inputs the "flyby" concept's heading solve uses; this module never reads
ground truth itself.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from isim.seeker import quat_to_rot
from isim.types import Detection, VehicleState, VelCmd

_EPS = 1e-9


# --------------------------------------------------------------- pure helpers

def _unit(v: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > _EPS else fallback


def _clip_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if max_norm > 0.0 and n > max_norm:
        return v * (max_norm / n)
    return v


def _slew(cmd: np.ndarray, prev: np.ndarray, accel_max_horiz: float,
          accel_max_vert: float, dt: float) -> np.ndarray:
    """Rate-limit the change from `prev` to `cmd`: horizontal (north/east)
    jointly, vertical (down) separately, each to at most accel_max_* * dt."""
    if dt <= 0.0:
        return prev.copy()
    d = cmd - prev
    d_h = _clip_norm(d[:2], accel_max_horiz * dt)
    max_dv = accel_max_vert * dt
    d_v = d[2]
    if abs(d_v) > max_dv:
        d_v = math.copysign(max_dv, d_v)
    return np.array([prev[0] + d_h[0], prev[1] + d_h[1], prev[2] + d_v])


def _yaw_toward(delta_ned: np.ndarray, fallback_deg: float) -> float:
    """Compass azimuth (deg, 0=N, 90=E, matching `flight.guidance`) of the
    horizontal component of `delta_ned`; holds `fallback_deg` when the
    horizontal component is ~zero (nothing to point at)."""
    n, e = float(delta_ned[0]), float(delta_ned[1])
    if math.hypot(n, e) < 1e-6:
        return fallback_deg
    return math.degrees(math.atan2(e, n))


# ---------------------------------------------------------------------- config

@dataclass
class PursuitConfig:
    """Every tunable of the "pursuit" concept. Defaults are the spec's own
    (`isim/specs/pursuit_concept.md`) -- none of these are bench-fit."""

    # --- Phase A: belief-only rendezvous, no tag required ---
    d_behind_m: float = 8.0          # m, aim point trails the belief along its track
    kp_pos: float = 0.8              # 1/s, position-tracking gain on the aim point
    v_max_ms: float = 16.0           # m/s, command magnitude ceiling (both phases)
    accel_max_ms2: float = 6.0       # m/s^2, slew limit, ALL THREE axes, Phase A
    # (gentle on purpose -- less pitch than the flyby's hard dash, see module docstring)

    # --- acquisition gate: Phase A -> Phase B ---
    acquire_n: int = 2               # detections required...
    acquire_window_s: float = 0.3    # ...within this trailing window

    # --- Phase B: KF-tracked terminal ---
    # v2 (pursuit_concept_v2.md #2): closing speed is a SCHEDULE, not a
    # constant -- clamp(k_close*range_est, v_close_min_ms, v_close_max_ms).
    # Flying the last few metres slowly and deliberately matches the mission
    # (this vehicle only needs to TOUCH the target, not hit it hard).
    k_close: float = 0.6             # 1/s, closing-speed gain on range_est
    v_close_min_ms: float = 1.5      # m/s, closing-speed floor (near range)
    v_close_max_ms: float = 6.0      # m/s, closing-speed ceiling (far range)
    kp_lat: float = 1.5              # 1/s, lateral (miss-component) correction gain
    # v2 (#3): renamed from `kp_cross` -- same role, now multiplying `r_perp`
    # (the component of r_est perpendicular to the CURRENT RELATIVE-VELOCITY
    # direction, i.e. the part that becomes the miss) instead of a component
    # relative to the target's own track. See `_phase_b_cmd`'s docstring for
    # why this supersedes v1's judgment call #1.
    accel_max_horiz_b_ms2: float = 8.0   # m/s^2, Phase B horizontal slew limit
    accel_max_vert_b_ms2: float = 6.0    # m/s^2, Phase B vertical slew limit
    coast_s: float = 1.0             # s, documented dropout threshold (see NOTE below)
    fallback_s: float = 3.0          # s, no decode this long -> back to a fresh Phase A
    hold_range_m: float = 1.0        # m, inside this: freeze the command ("fly through")
    # v2 (#4): the tag legitimately fills/leaves the frame at very close
    # range -- that is not a real track loss, so fallback is suppressed
    # entirely inside this range regardless of `fallback_s`.
    no_fallback_range_m: float = 3.0
    # v2 (#1 fix, "Bug B" -- see the task's final report): a KF velocity
    # estimate adopted as a fresh Phase-A belief on fallback is clamped to at
    # most `fallback_speed_cap_mult` x the ORIGINAL pre-flight believed
    # target speed (floor `fallback_speed_floor_ms`, for the target_speed=0
    # test case) -- guards one bad KF snapshot sending the vehicle off to a
    # velocity-extrapolated point it can never recover from.
    fallback_speed_cap_mult: float = 2.0
    fallback_speed_floor_ms: float = 3.0

    # NOTE on `coast_s` vs `fallback_s`: the spec names both a 1.0 s "keep
    # flying on the estimate" threshold and a 3.0 s "fall back to Phase A"
    # threshold. This implementation predicts the KF through EVERY gap
    # regardless of length (that is what "keep flying on the estimate"
    # means -- there is no second, different behaviour to switch to at
    # 1.0 s), so `coast_s` is kept as a documented constant but does not
    # gate a distinct code path; only `fallback_s` changes state. Flagged as
    # a judgment call in the task's final report.

    # Guidance's OWN ASSUMED sensor-noise model, for the KF's measurement
    # covariance -- a design constant the guidance would carry from bench
    # calibration, never a live read of the seeker's true noise.
    sigma_px: float = 0.3
    sigma_side_px: float = 0.3 * 1.414
    fx_px: float = 540.0
    tag_side_m: float = 0.30
    # v2 TUNED FIX (found while chasing pursuit_concept_v2.md #1's "camera
    # never converges" bug -- see the task's final report for the trace
    # evidence): the literal cross_sigma = range*sigma_px/fx is a few
    # MILLIMETRES at typical acquisition range, i.e. the KF is told cross-
    # track position is nearly noiseless. That is fine while the LOS is
    # nearly static, but under a fast crossing pass the LOS direction
    # rotates tens of degrees per second, so the (along, cross, cross) R
    # basis itself rotates every update -- an almost-zero-noise axis that
    # keeps pointing a different way each frame drags the position estimate
    # (and, through the predict step's pos-vel coupling, the VELOCITY
    # estimate) into a chaotic, tens-of-m/s-off state within a few tenths of
    # a second (measured, seed 0, tag_facing="camera": kfvel went from a
    # sane (9.7, 0.6, 0) m/s to (7.5,-14.8,18.9) m/s two guidance ticks
    # later). A floor on cross_sigma -- standing in for own-attitude and
    # camera-latency angular uncertainty the pure pixel-noise formula never
    # modelled -- fixes it: BEFORE (no floor) 10 seeds of the nominal
    # crossing case, tag_facing="camera", median miss 2.63 m (one seed never
    # recovered from the runaway, ending 24 m out); AFTER (0.2 m floor)
    # same 10 seeds, median 0.52 m. Swept 0.05/0.10/0.15/0.20/0.25/0.30 m;
    # 0.20 m is the adopted value (see the report for the sweep table) --
    # not a bench measurement, an isim-tuning choice.
    cross_sigma_floor_m: float = 0.20

    # KF process noise: piecewise-constant-acceleration white-noise model.
    kf_q_accel_ms2: float = 1.0      # m/s^2, 1-sigma unmodelled target acceleration
    kf_p0_pos_m: float = 3.0         # m, 1-sigma initial position uncertainty
    kf_p0_vel_ms: float = 5.0        # m/s, 1-sigma initial velocity uncertainty


# ------------------------------------------------------------- Phase A belief

@dataclass
class _BeliefTrack:
    """A moving point: pos(t) = pos_ref + vel_ref*(t - t_ref) for t >= t_ref,
    held at pos_ref before it -- mirrors `isim.scenario._DelayedTarget`'s own
    convention, so Phase A's belief starts moving at exactly the GO edge (or,
    after a B->A fallback, at the fallback instant)."""

    pos_ref: np.ndarray
    vel_ref: np.ndarray
    t_ref: float

    def at(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        if t <= self.t_ref:
            return self.pos_ref.copy(), np.zeros(3)
        return self.pos_ref + self.vel_ref * (t - self.t_ref), self.vel_ref.copy()


# ------------------------------------------------------------------ Phase B KF

class _ConstVelKF:
    """6-state (pos, vel) constant-velocity Kalman filter in NED. Q is the
    standard piecewise-constant-acceleration white-noise-acceleration model;
    R is supplied per-update by the caller (it depends on that detection's
    range, per the spec)."""

    def __init__(self, q_accel_ms2: float) -> None:
        self.q = q_accel_ms2 ** 2
        self.x = np.zeros(6)
        self.P = np.eye(6)

    def init(self, pos: np.ndarray, vel: np.ndarray, p0_pos: float, p0_vel: float) -> None:
        self.x = np.concatenate([np.asarray(pos, dtype=np.float64),
                                 np.asarray(vel, dtype=np.float64)])
        self.P = np.diag([p0_pos ** 2] * 3 + [p0_vel ** 2] * 3)

    def predict(self, dt: float) -> None:
        if dt <= 0.0:
            return
        F = np.eye(6)
        F[0:3, 3:6] = np.eye(3) * dt
        qpp, qpv, qvv = (dt ** 3 / 3.0) * self.q, (dt ** 2 / 2.0) * self.q, dt * self.q
        Q = np.zeros((6, 6))
        Q[0:3, 0:3] = np.eye(3) * qpp
        Q[0:3, 3:6] = np.eye(3) * qpv
        Q[3:6, 0:3] = np.eye(3) * qpv
        Q[3:6, 3:6] = np.eye(3) * qvv
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z_pos: np.ndarray, R: np.ndarray) -> None:
        H = np.zeros((3, 6))
        H[:, 0:3] = np.eye(3)
        y = np.asarray(z_pos, dtype=np.float64) - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    @property
    def pos(self) -> np.ndarray:
        return self.x[0:3].copy()

    @property
    def vel(self) -> np.ndarray:
        return self.x[3:6].copy()


@dataclass
class _Decision:
    """Just enough of `flight.deploy.real_flight`'s `Decision` shape for
    `isim.mc._run_one` to read `.state` -- this concept has no failsafe
    reasons, so nothing else is needed."""
    state: str


@dataclass
class PursuitDebug:
    """Per-tick self-report, updated at the end of every `step()` call --
    pursuit_concept_v2.md #5 ("a miss should explain itself"). `r_est`/
    `v_t_est` are this guidance's OWN belief (Phase A: the belief track;
    Phase B: the KF) of the target's relative position / absolute velocity
    -- never ground truth; the SCORING side (`isim.mc`) is the only place
    that may compare this to a trace's truth, and only after the run ends."""
    r_est: np.ndarray
    v_t_est: np.ndarray
    phase: str


# --------------------------------------------------------------------- the law

class PursuitRendezvousGuidance:
    """isim `Guidance`: Phase A (belief rendezvous, no tag) -> Phase B
    (KF-tracked terminal, driven by `Detection`s) -> falls back to a fresh
    Phase A on a long dropout. See `isim/specs/pursuit_concept.md`.

    `state_log`/`last_decision` mirror `isim.flight_adapter.RealFlightGuidance`'s
    shape (a `(t, state_str)` list and a `.state`-bearing object) so
    `isim.mc._run_one` works unmodified for either concept. States used:
    "STANDBY" (before GO), "APPROACH" (Phase A), "ENGAGE" (Phase B) -- chosen
    so `reached_engage` (`isim.mc`'s literal "ENGAGE" check) means the same
    thing for both concepts: the terminal law actually took over from a tag.
    """

    def __init__(self, cfg: PursuitConfig, belief_pos0_ned: np.ndarray,
                 belief_vel_ned: np.ndarray, go_at_s: float,
                 initial_yaw_deg: float, cam_mount_tilt_up_deg: float = 0.0) -> None:
        self.cfg = cfg
        self._belief_pos0 = np.asarray(belief_pos0_ned, dtype=np.float64).copy()
        self._belief_vel0 = np.asarray(belief_vel_ned, dtype=np.float64).copy()
        self.go_at_s = float(go_at_s)
        self.initial_yaw_deg = float(initial_yaw_deg)
        self._mount_tilt_rad = math.radians(cam_mount_tilt_up_deg)
        self.state_log: List[Tuple[float, str]] = []
        self.events: List[str] = []
        self.last_decision: Optional[_Decision] = None
        self.debug: PursuitDebug = PursuitDebug(r_est=np.zeros(3), v_t_est=np.zeros(3),
                                                phase="STANDBY")
        self.reset()

    def reset(self) -> None:
        self._track = _BeliefTrack(self._belief_pos0.copy(), self._belief_vel0.copy(),
                                    self.go_at_s)
        self._phase = "STANDBY"
        self._kf = _ConstVelKF(self.cfg.kf_q_accel_ms2)
        self._decode_positions: List[Tuple[float, np.ndarray]] = []
        self._last_decode_t = -math.inf
        self._prev_v_cmd = np.zeros(3)
        self._prev_yaw_deg = self.initial_yaw_deg
        self._last_t: Optional[float] = None
        self.state_log = []
        self.events = []
        self.last_decision = None
        self.debug = PursuitDebug(r_est=np.zeros(3), v_t_est=np.zeros(3), phase="STANDBY")

    # ---------------------------------------------------------------- step()

    def step(self, t: float, own: VehicleState, det: Optional[Detection]) -> VelCmd:
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        own_pos = np.asarray(own.pos_ned, dtype=np.float64)

        if t < self.go_at_s:
            self._set_state("STANDBY", t)
            self._prev_v_cmd = np.zeros(3)
            self._prev_yaw_deg = self.initial_yaw_deg
            self._last_t = t
            return VelCmd(0.0, 0.0, 0.0, self.initial_yaw_deg)

        if self._phase == "STANDBY":
            self._phase = "A"

        if det is not None:
            self._decode_positions.append((det.t_capture, self._measure_pos_ned(own, det)))
            cutoff = t - self.cfg.acquire_window_s
            self._decode_positions = [p for p in self._decode_positions if p[0] >= cutoff]

        if self._phase == "A" and len(self._decode_positions) >= self.cfg.acquire_n:
            self._start_phase_b(t)

        if self._phase == "B":
            self._kf.predict(dt)
            if det is not None:
                self._update_kf(own, det, t)
            range_est = float(np.linalg.norm(self._kf.pos - own_pos))
            dropout_s = t - self._last_decode_t
            # v2 #4: the tag legitimately fills/leaves the frame very close
            # in -- that is not a real track loss, so suppress the fallback
            # inside `no_fallback_range_m` regardless of dropout duration.
            if dropout_s > self.cfg.fallback_s and range_est >= self.cfg.no_fallback_range_m:
                safe_vel = self._sane_fallback_vel(self._kf.vel)
                self._track = _BeliefTrack(self._kf.pos, safe_vel, t)
                self._phase = "A"

        self._set_state("ENGAGE" if self._phase == "B" else "APPROACH", t)

        if self._phase == "A":
            cmd_v = self._phase_a_cmd(t, own_pos)
            pos_belief, vel_belief = self._track.at(t)
            yaw = _yaw_toward(pos_belief - own_pos, self._prev_yaw_deg)
            accel_h = accel_v = self.cfg.accel_max_ms2
            self.debug = PursuitDebug(r_est=pos_belief - own_pos, v_t_est=vel_belief,
                                      phase=self._phase)
        else:
            cmd_v, yaw = self._phase_b_cmd(own, own_pos)
            accel_h, accel_v = self.cfg.accel_max_horiz_b_ms2, self.cfg.accel_max_vert_b_ms2
            self.debug = PursuitDebug(r_est=self._kf.pos - own_pos, v_t_est=self._kf.vel,
                                      phase=self._phase)

        cmd_v = _clip_norm(cmd_v, self.cfg.v_max_ms)
        cmd_v = _slew(cmd_v, self._prev_v_cmd, accel_h, accel_v, dt)

        self._prev_v_cmd = cmd_v
        self._prev_yaw_deg = yaw
        self._last_t = t
        return VelCmd(v_north=float(cmd_v[0]), v_east=float(cmd_v[1]),
                      v_down=float(cmd_v[2]), yaw_deg=yaw)

    def _sane_fallback_vel(self, kf_vel: np.ndarray) -> np.ndarray:
        """Clamp a KF-derived velocity before trusting it as a fresh Phase-A
        belief (pursuit_concept_v2.md #1, "Bug B"): a velocity estimate
        seeded or corrupted by a brief, noisy acquisition can be tens of
        m/s off (see the task's final report's trace excerpt), and Phase A
        has no further correction mechanism once it adopts one -- clamping
        it to a multiple of the ORIGINAL pre-flight believed target speed
        (never a live read; `_belief_vel0` is a constructor argument) turns
        a runaway extrapolation into, at worst, a lost-but-bounded chase."""
        belief_speed = float(np.linalg.norm(self._belief_vel0))
        cap = max(belief_speed * self.cfg.fallback_speed_cap_mult,
                 self.cfg.fallback_speed_floor_ms)
        n = float(np.linalg.norm(kf_vel))
        return kf_vel * (cap / n) if n > cap else kf_vel

    def _set_state(self, state: str, t: float) -> None:
        if not self.state_log or self.state_log[-1][1] != state:
            self.state_log.append((t, state))
        self.last_decision = _Decision(state=state)

    # ------------------------------------------------------------- Phase A law

    def _phase_a_cmd(self, t: float, own_pos: np.ndarray) -> np.ndarray:
        cfg = self.cfg
        pos_belief, vel_belief = self._track.at(t)
        track_dir = _unit(vel_belief, np.array([1.0, 0.0, 0.0]))
        aim = pos_belief - cfg.d_behind_m * track_dir
        return vel_belief + cfg.kp_pos * (aim - own_pos)

    # ------------------------------------------------------------- Phase B law

    def _phase_b_cmd(self, own: VehicleState, own_pos: np.ndarray) -> Tuple[np.ndarray, float]:
        """v2 (pursuit_concept_v2.md #2/#3):

        v_close = clamp(k_close*range_est, v_close_min_ms, v_close_max_ms)
        v_cmd   = v_target_est + v_close*unit(r_est) + kp_lat*r_perp

        `r_perp` is the component of `r_est` perpendicular to the CURRENT
        RELATIVE-VELOCITY direction (`kf_vel - own_vel`) -- for two bodies on
        straight-line paths, that component IS exactly the eventual miss
        distance (the along-that-direction component only changes the TIME
        of closest approach, not its size), so driving it to zero targets the
        miss itself. This SUPERSEDES v1's judgment call #1 (cross-track
        relative to the target's own track, `unit(kf_vel)`): v2 gives an
        unambiguous formula, so that is now the fallback reference direction
        only (used when the relative-velocity direction is degenerate, e.g.
        own_vel already matches kf_vel almost exactly) -- `unit(r_est)`
        itself is the last-resort fallback (guaranteeing r_perp=0, i.e. no
        lateral term, rather than a divide-by-~0 direction)."""
        cfg = self.cfg
        kf_pos, kf_vel = self._kf.pos, self._kf.vel
        r_est = kf_pos - own_pos
        range_est = float(np.linalg.norm(r_est))
        if range_est < cfg.hold_range_m:
            return self._prev_v_cmd.copy(), self._prev_yaw_deg   # "fly through"

        los_dir = _unit(r_est, np.array([1.0, 0.0, 0.0]))
        v_close = float(np.clip(cfg.k_close * range_est, cfg.v_close_min_ms, cfg.v_close_max_ms))

        own_vel = np.asarray(own.vel_ned, dtype=np.float64)
        v_rel = kf_vel - own_vel
        ref_dir = _unit(v_rel, _unit(kf_vel, los_dir))
        r_perp = r_est - float(np.dot(r_est, ref_dir)) * ref_dir

        cmd_v = kf_vel + v_close * los_dir + cfg.kp_lat * r_perp
        yaw = _yaw_toward(r_est, self._prev_yaw_deg)
        return cmd_v, yaw

    # --------------------------------------------------------------- KF glue

    def _dir_ned(self, own: VehicleState, det: Detection) -> np.ndarray:
        """Unit LOS direction in NED from bearing/elevation (camera-relative,
        per `isim.types.Detection`), the nominal camera mount tilt, and the
        vehicle's own attitude quaternion -- the same body-relative
        bearing/elevation -> direction construction `isim.stubs.PursuitGuidance`
        uses, extended by the mount-tilt rotation `isim.seeker.CameraParams`
        applies (this guidance never touches the seeker's true camera)."""
        b, e = math.radians(det.bearing_deg), math.radians(det.elevation_deg)
        dir_boresight = np.array([math.cos(e) * math.cos(b), math.cos(e) * math.sin(b),
                                  -math.sin(e)])
        ct, st = math.cos(self._mount_tilt_rad), math.sin(self._mount_tilt_rad)
        ry = np.array([[ct, 0.0, st], [0.0, 1.0, 0.0], [-st, 0.0, ct]])
        dir_body = ry @ dir_boresight
        r_bn = quat_to_rot(own.quat_wxyz)
        return _unit(r_bn @ dir_body, dir_body)

    def _measure_pos_ned(self, own: VehicleState, det: Detection) -> np.ndarray:
        own_pos = np.asarray(own.pos_ned, dtype=np.float64)
        return own_pos + det.range_m * self._dir_ned(own, det)

    def _measurement_r(self, det: Detection, dir_ned: np.ndarray) -> np.ndarray:
        """Diagonal in a (along-LOS, cross, cross) basis aligned with
        `dir_ned`, per the spec: cross-range sigma = range*sigma_px/fx
        (floored -- see `PursuitConfig.cross_sigma_floor_m`'s docstring),
        along-range sigma = range^2*sigma_side_px/(fx*tag_side)."""
        cfg = self.cfg
        r = max(det.range_m, 1e-3)
        cross_sigma = max(r * cfg.sigma_px / cfg.fx_px, cfg.cross_sigma_floor_m)
        along_sigma = (r ** 2) * cfg.sigma_side_px / (cfg.fx_px * cfg.tag_side_m)
        outer = np.outer(dir_ned, dir_ned)
        return (cross_sigma ** 2) * np.eye(3) + (along_sigma ** 2 - cross_sigma ** 2) * outer

    def _update_kf(self, own: VehicleState, det: Detection, t: float) -> None:
        dir_ned = self._dir_ned(own, det)
        z = np.asarray(own.pos_ned, dtype=np.float64) + det.range_m * dir_ned
        self._kf.update(z, self._measurement_r(det, dir_ned))
        self._last_decode_t = t

    def _start_phase_b(self, t: float) -> None:
        """v2 (pursuit_concept_v2.md #1, "Bug A" -- see the task's final
        report): the KF's velocity used to be seeded from a 2-point finite
        difference of the acquire window's decode positions. `acquire_n=2`
        within `acquire_window_s=0.3 s` can be satisfied by two decodes a
        SINGLE frame period apart (<0.03 s at 38 fps); the along-range
        measurement noise scales as range^2/(fx*tag_side) and is commonly
        O(0.1-1 m) at real acquisition ranges, so differencing over such a
        short baseline amplifies that noise into a velocity error of tens of
        m/s. Seeding from the Phase-A belief velocity instead is a far safer
        prior (it is exactly what the aim has been trusting up to this
        instant); the KF's own subsequent position UPDATES refine it from
        there as real decodes accumulate."""
        pos0 = self._decode_positions[-1][1]
        _, vel0 = self._track.at(t)
        self._kf.init(pos0, vel0, self.cfg.kf_p0_pos_m, self.cfg.kf_p0_vel_ms)
        self._last_decode_t = self._decode_positions[-1][0]
        self._phase = "B"
