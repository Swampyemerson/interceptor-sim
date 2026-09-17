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

v3 (isim/specs/pursuit_concept_v3.md) fixed the remaining estimator bias:
`Detection`s arrive `latency_s` after their frame was exposed, and the
measurement was being converted to NED using the vehicle's own position/
attitude AT ARRIVAL, not at `det.t_capture`. A short ring buffer of own
(t, pos, vel, quat) samples (`_own_hist`/`_interp_own_state`) fixes this --
see `step()`/`_measure_pos_ned`/`_update_kf` and the task's final report.
(v3's item #2, "re-formulate as an absolute-state filter", needed NO code
change: the KF here always estimated the target's ABSOLUTE NED state, never
a relative one -- the "own acceleration looks like target motion" symptom
was entirely the capture-time bug above, using the wrong own position as
the absolute reference.)

v4 (isim/specs/pursuit_concept_v4.md) targets the "last second"/height/aim-
cliff failure modes the v3 diagnostics pointed at, all Phase-B/A additions,
no further bug fixes:
  - Phase B prioritises nulling the LATERAL miss component over closing
    speed inside `last_line_range_m` (`kp_lat_near_mult`) -- "arrive on a
    line that keeps the tag centred" (#1b).
  - Phase A, once it has ARRIVED at its aim point with no decode for
    `vsearch_delay_s`, starts a slow vertical sweep (`vsearch_*`) and a yaw
    sweep (`yaw_search_*`) around the aim point/belief bearing -- "it
    should not sit there" (#2/#3).
  - `DualTagSeeker` (a plain composite, no `isim.seeker` changes) covers
    two MEASURE-only hardware ideas from the same mechanism: a smaller,
    closer-range co-located tag (#1d) and a second tag at a different
    mount orientation (#3's "two-tag target"); see `isim.scenario.Scenario.
    inner_tag_side_m`/`second_tag_facing`. Neither is a new default.
  - `PursuitDebug.last_decode_t` + `isim.mc`'s new "at last decode" query
    resolve v4 #4's unexplained oddity -- see the task's final report.

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
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import numpy as np

from flight.guidance import dash_forward_speed

from isim.seeker import quat_to_rot
from isim.types import Detection, FrameReport, SeekerModel, VehicleState, VelCmd

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


# `_OwnSample` = (t, pos_ned, vel_ned, quat_wxyz) -- one ring-buffer entry.
_OwnSample = Tuple[float, np.ndarray, np.ndarray, Tuple[float, float, float, float]]


def _interp_own_state(hist: Deque[_OwnSample], t_query: float) -> _OwnSample:
    """Linear-interpolate (nlerp + renormalize for the quaternion -- exact
    for small angle gaps, which this always is: the ring buffer is sampled
    every `guidance_dt`, tens of ms) the own-state ring buffer at `t_query`.
    Clamps to the buffer's ends rather than extrapolating. `hist` must be
    non-empty (the caller only calls this once at least one sample exists,
    which is guaranteed by go_at_s < any det.t_capture the guidance acts on)."""
    if t_query <= hist[0][0]:
        return hist[0]
    if t_query >= hist[-1][0]:
        return hist[-1]
    for i in range(1, len(hist)):
        t0, p0, v0, q0 = hist[i - 1]
        t1, p1, v1, q1 = hist[i]
        if t0 <= t_query <= t1:
            frac = 0.0 if t1 <= t0 else (t_query - t0) / (t1 - t0)
            pos = p0 + frac * (p1 - p0)
            vel = v0 + frac * (v1 - v0)
            q = np.asarray(q0, dtype=np.float64) * (1.0 - frac) + np.asarray(q1) * frac
            qn = float(np.linalg.norm(q))
            quat = tuple(q / qn) if qn > _EPS else q1
            return (t_query, pos, vel, quat)
    return hist[-1]


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
    #
    # v3 RE-SWEPT (pursuit_concept_v3.md #3), with the capture-time fix (#1)
    # in place, at 0.20/0.10/0.05/0.03/0.02/0.01/0.005 m, n=60 x 2 facings:
    # lowering the floor is NOT a clean win -- it monotonically HELPS
    # tag_facing="rear" (median miss 0.213->0.218m but pct<=0.35m 70%->80%
    # at 0.01m) while making tag_facing="camera" WORSE (median miss
    # 0.325->0.355m, pct<=0.35m 57%->50% at 0.01m), even though camera's
    # median ESTIMATOR error keeps improving (1.34->0.72m) as the floor
    # drops -- camera decodes far more densely/rapidly during a fast
    # crossing, and a tight floor appears to let a handful of individually
    # noisy detections perturb the filter enough to occasionally cost more
    # than the improved typical-case accuracy buys (a distributional/tail
    # effect the median estimator error alone doesn't show). KEPT AT 0.20
    # -- the best single value for the concept's stronger (camera) facing;
    # full sweep table in the task's final report for whoever wants to
    # trade this differently.
    cross_sigma_floor_m: float = 0.20

    # KF process noise: piecewise-constant-acceleration white-noise model.
    kf_q_accel_ms2: float = 1.0      # m/s^2, 1-sigma unmodelled target acceleration
    kf_p0_pos_m: float = 3.0         # m, 1-sigma initial position uncertainty
    kf_p0_vel_ms: float = 5.0        # m/s, 1-sigma initial velocity uncertainty

    # v3 (pursuit_concept_v3.md #1): a Detection arrives `latency_s` after its
    # frame was exposed (`det.t_capture` vs the tick it is handed to step()).
    # The own (pos, vel, quat) used to turn that Detection into a NED position
    # must be looked up AT `t_capture`, not at arrival -- own_state_history_s
    # sizes the ring buffer `_interp_own_state` interpolates into (0.3 s is
    # generous against latencies of tens of ms).
    own_state_history_s: float = 0.3

    # --- v4 #1b: "arrive on a line that keeps the tag centred" -- inside
    # last_line_range_m, the lateral (miss) correction is weighted more
    # heavily relative to closing, so the vehicle nulls its lateral
    # relative velocity EARLY and then closes on a straight line, instead
    # of closing and correcting laterally at the same rate the whole way
    # in.
    #
    # MEASURED NULL/NEGATIVE (n=100 x 2 facings, swept 1.0/1.5/2.0/2.5/4.0):
    # boosting the near-range lateral gain makes BOTH facings monotonically
    # WORSE -- camera pct<=0.35m 55%->38% at 2.5x, rear 71%->53% at 2.5x
    # (rear falls to 29% by 4.0x). The likely mechanism: the estimate is at
    # its noisiest, relative to the shrinking range, exactly where this
    # multiplier is largest, so a bigger gain there amplifies noise into
    # over-correction rather than damping genuine lateral drift. KEPT AT
    # 1.0 (no-op/off) -- the idea did not survive measurement; see the
    # task's final report for the full sweep table.
    kp_lat_near_mult: float = 1.0
    last_line_range_m: float = 3.0

    # --- v4 #2/#3: Phase A search. Once the vehicle is CLOSE ENOUGH to its
    # aim point (within `vsearch_arrival_range_m`) and has gone
    # `vsearch_delay_s` with no decode of any kind, it stops flying a
    # silent, un-perturbed pursuit line and starts a slow vertical sweep
    # (amplitude/period below) around the aim point's altitude, AND (v4 #3)
    # a yaw sweep around the belief bearing, on a DIFFERENT period so the
    # two sweeps slowly precess relative to each other and cover more of
    # the search volume over time than either alone.
    #
    # MEASURED NEGATIVE ON BALANCE (n=100 x 2 facings x 5 cases: nominal,
    # alt +-, aim 20/30 deg) -- DEFAULT OFF (both amplitudes 0.0). It only
    # ever helps the single worst case it targets (tag_facing="rear",
    # aim_error=30deg: %engage 5%->10%, but pct<=0.35m stays ~flat, 2%->3%)
    # while making every case that was ALREADY mostly working WORSE for
    # tag_facing="rear" (nominal pct<=0.35m 71%->67%; alt-2m 74%->68%;
    # alt+3m 43%->32%; aim_error=20deg 54%->40%, %engage 64%->55%) --
    # perturbing a Phase-A trajectory that was about to succeed anyway
    # costs more often than a genuinely-stuck trajectory gets rescued.
    # tag_facing="camera" is nearly unaffected either way (it rarely
    # reaches this trigger at all -- see the report). The mechanism is
    # implemented and tested (`test_phase_a_search_sweeps_...`); kept
    # available via `Scenario.pursuit_overrides` for anyone who wants a
    # different worst-case/typical-case tradeoff, but not adopted as the
    # default. See the task's final report for the full before/after table.
    vsearch_arrival_range_m: float = 2.0
    vsearch_delay_s: float = 2.0
    vsearch_amplitude_m: float = 0.0
    vsearch_period_s: float = 8.0
    yaw_search_amplitude_deg: float = 0.0
    yaw_search_period_s: float = 7.0

    # --- v6 #A: close-in steering in the CAMERA/BODY frame (isim/specs/
    # pursuit_v6.md). Inside cam_frame_range_m, the lateral/vertical
    # command is built from the detection's bearing/elevation/range
    # directly (NO attitude needed to place the tag in the image) rather
    # than from the attitude-dependent absolute-frame KF; only the FINAL
    # body->NED rotation (to issue a NED velocity setpoint) touches the
    # (possibly-perturbed) attitude, which is now a bounded, second-order
    # rotation of a command rather than a first-order corruption of an
    # absolute position estimate that WORSENS as range closes.
    #
    # MEASURED (n=100 x 2 facings, v5 all-errors-on): NOT the "expected
    # biggest win" -- mixed and mostly disappointing. A REAL BUG was found
    # first: the literal spec (yaw = atan2 of the camera-frame offset)
    # combined with a defensive `max(forward, 1e-6)` clamp created a
    # discontinuous ~180deg yaw JUMP every time the target's forward
    # offset crossed zero (an overshoot) -- the vehicle then spun trying
    # to chase the jump and never recovered (traced directly, seed 0,
    # tag_facing="camera": yaw commands cycling through the full circle
    # for 3+ seconds with zero decodes). Fixed (plain `atan2`, no clamp --
    # see `_phase_b_cmd_camera_frame`), which alone recovered much of the
    # loss but still left the mechanism net-negative for tag_facing=
    # "camera" at the literal spec's cam_frame_range_m=5m (49%->48%
    # pct<=0.35m, i.e. flat/slightly worse). Swept kp_cam
    # (0/0.75/1.5/3/5) x cam_frame_range_m (1/2/3/5) x the `cam_frame_yaw`
    # ablation below: the best found combination (kp_cam=0.75,
    # cam_frame_yaw=False, cam_frame_range_m=2.0) gives camera 49%->55%
    # (a real but modest +6 point gain) while making REAR WORSE AT EVERY
    # RADIUS TRIED (68%->50-55%, worse the larger the radius). Net: NOT
    # adopted as the default (mixed across facings, and rear is the
    # STRONGER facing in the v5 baseline) -- kept at 0.0 (off); the report
    # measures "A" with the best-found override
    # (kp_cam=0.75/cam_frame_yaw=False/cam_frame_range_m=2.0) applied via
    # `pursuit_overrides`, not as a new default.
    cam_frame_range_m: float = 0.0
    kp_cam: float = 1.5              # 1/s, lateral/vertical body-frame gain; 0.75 measured best
    # Ablation (see cam_frame_range_m's note above for the measured
    # numbers): False (fall back to the OLD absolute-frame LOS yaw while
    # still using camera-frame steering for the VELOCITY command) measured
    # BETTER than True (the literal spec's atan2-based yaw, which remains
    # geometrically sensitive near a small/negative forward offset even
    # after the discontinuity bug above was fixed) -- kept True as the
    # dataclass default (matches the literal spec unless overridden); the
    # report's "A" config overrides it to False.
    cam_frame_yaw: bool = True

    # --- v6 #B2: estimate the guidance-visible frame-timestamp bias as a
    # 7th KF state (see `_ConstVelKF`) instead of trusting the believed
    # age_s outright.
    #
    # MEASURED NEGATIVE (n=100 x 2 facings, v5 all-errors-on, WITH the
    # measured-best "A" config already applied): camera 55%->41%, rear
    # 55%->29% -- a clear, uniform regression on TOP of A, not a further
    # gain. Matches the spec's own warning ("may be weakly observable --
    # report honestly"): an isolated synthetic KF test (see
    # `test_kf_bias_state_does_not_diverge_and_residual_shrinks`) shows
    # `t_bias` is ALGEBRAICALLY DEGENERATE with a constant shift of the
    # position state (`h(x)=pos-vel*(age+t_bias)`; a `pos` shift of
    # `+vel*t_bias` mimics `t_bias_hat=0` for ANY age_s) when the target
    # moves at exactly constant velocity and `own` never otherwise enters
    # the measurement -- adding a 7th, weakly-observable state to a
    # 6-state filter that was already only weakly excited (v5's own noise
    # sources reduce decode density) adds estimation VARIANCE that costs
    # more than the bias-removal buys. NOT adopted; off by default.
    estimate_ts_bias: bool = False
    kf_p0_bias_s: float = 0.030       # s, 1-sigma initial uncertainty on t_bias


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
    """6-state (pos, vel) constant-velocity Kalman filter in NED, OPTIONALLY
    extended to a 7th state `t_bias` (v6 #B2 -- isim/specs/pursuit_v6.md):
    the guidance-visible frame-timestamp bias is unknown but roughly
    CONSTANT for a run, so it is estimable the same way `_start_phase_b`'s
    velocity is -- as an extra state the measurement updates refine. Q is
    the standard piecewise-constant-acceleration white-noise-acceleration
    model on (pos, vel); R is supplied per-update by the caller."""

    def __init__(self, q_accel_ms2: float, estimate_ts_bias: bool = False) -> None:
        self.q = q_accel_ms2 ** 2
        self.estimate_ts_bias = estimate_ts_bias
        self.n = 7 if estimate_ts_bias else 6
        self.x = np.zeros(self.n)
        self.P = np.eye(self.n)

    def init(self, pos: np.ndarray, vel: np.ndarray, p0_pos: float, p0_vel: float,
            p0_bias_s: float = 0.030) -> None:
        parts = [np.asarray(pos, dtype=np.float64), np.asarray(vel, dtype=np.float64)]
        diag = [p0_pos ** 2] * 3 + [p0_vel ** 2] * 3
        if self.estimate_ts_bias:
            parts.append(np.zeros(1))       # no prior belief about the sign/size
            diag.append(p0_bias_s ** 2)
        self.x = np.concatenate(parts)
        self.P = np.diag(diag)

    def predict(self, dt: float) -> None:
        if dt <= 0.0:
            return
        n = self.n
        F = np.eye(n)
        F[0:3, 3:6] = np.eye(3) * dt
        qpp, qpv, qvv = (dt ** 3 / 3.0) * self.q, (dt ** 2 / 2.0) * self.q, dt * self.q
        Q = np.zeros((n, n))
        Q[0:3, 0:3] = np.eye(3) * qpp
        Q[0:3, 3:6] = np.eye(3) * qpv
        Q[3:6, 0:3] = np.eye(3) * qpv
        Q[3:6, 3:6] = np.eye(3) * qvv
        # t_bias has NO process noise: it is modelled as a true per-run
        # constant (a random walk here would let it silently re-absorb
        # noise the way the position bias in v5 #3 partly, imperfectly,
        # canceled -- the point of estimating it explicitly is to pin it
        # down, not to let it wander).
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z_pos: np.ndarray, R: np.ndarray, age_s: float = 0.0) -> None:
        # `z_pos` is where the target WAS `age_s` ago (frame capture), while the
        # state is "now": the measurement model is pos - vel*age. Ignoring this
        # makes the estimate trail a 9 m/s target by speed x latency (~0.4 m).
        # v6 #B2: `age_s` is the GUIDANCE-BELIEVED age (built from a possibly
        # timestamp-biased `det.t_capture`); when `estimate_ts_bias`, the TRUE
        # age is modelled as `age_s + t_bias` and `t_bias` is a 7th state, so
        # H is linearized (EKF-style) around the CURRENT bias estimate:
        # h(x) = pos - vel*(age_s + t_bias); dh/dvel = -(age_s+t_bias)*I;
        # dh/d(t_bias) = -vel (evaluated at the current vel estimate).
        n = self.n
        H = np.zeros((3, n))
        H[:, 0:3] = np.eye(3)
        eff_age = float(age_s) + (float(self.x[6]) if self.estimate_ts_bias else 0.0)
        H[:, 3:6] = -eff_age * np.eye(3)
        if self.estimate_ts_bias:
            H[:, 6] = -self.x[3:6]
        y = np.asarray(z_pos, dtype=np.float64) - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(n) - K @ H) @ self.P

    @property
    def pos(self) -> np.ndarray:
        return self.x[0:3].copy()

    @property
    def vel(self) -> np.ndarray:
        return self.x[3:6].copy()

    @property
    def bias_s(self) -> float:
        return float(self.x[6]) if self.estimate_ts_bias else 0.0


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
    that may compare this to a trace's truth, and only after the run ends.
    `last_decode_t` (v4 #4) is `self._last_decode_t` (-inf if never) -- lets
    the scoring side ALSO query the estimate/control error AT THE LAST
    DECODE, not just at CPA (see the task's final report for why "error at
    CPA" is a misleading statistic once the vehicle is coasting blind)."""
    r_est: np.ndarray
    v_t_est: np.ndarray
    phase: str
    last_decode_t: float = -math.inf


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
        self._kf = _ConstVelKF(self.cfg.kf_q_accel_ms2, estimate_ts_bias=self.cfg.estimate_ts_bias)
        self._cam_offset_body: Optional[np.ndarray] = None   # v6 #A state
        self._decode_positions: List[Tuple[float, np.ndarray]] = []
        self._last_decode_t = -math.inf
        self._prev_v_cmd = np.zeros(3)
        self._prev_yaw_deg = self.initial_yaw_deg
        self._last_t: Optional[float] = None
        self._own_hist: Deque[_OwnSample] = deque()
        self._last_any_decode_t = -math.inf        # v4 #2/#3: Phase-A search trigger
        self._last_search_elapsed: Optional[float] = None
        self.state_log = []
        self.events = []
        self.last_decision = None
        self.debug = PursuitDebug(r_est=np.zeros(3), v_t_est=np.zeros(3), phase="STANDBY")

    # ---------------------------------------------------------------- step()

    def step(self, t: float, own: VehicleState, det: Optional[Detection]) -> VelCmd:
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        own_pos = np.asarray(own.pos_ned, dtype=np.float64)

        # v3 #1: ring buffer of own (pos, vel, quat), so a Detection can be
        # converted to NED using the own-state AT det.t_capture, not at
        # arrival (`t`) -- maintained unconditionally (cheap; before GO it
        # is simply never looked up).
        self._own_hist.append((t, own_pos.copy(), np.asarray(own.vel_ned, dtype=np.float64).copy(),
                               tuple(float(c) for c in own.quat_wxyz)))
        cutoff_hist = t - self.cfg.own_state_history_s
        while len(self._own_hist) > 2 and self._own_hist[0][0] < cutoff_hist:
            self._own_hist.popleft()

        if t < self.go_at_s:
            self._set_state("STANDBY", t)
            self._prev_v_cmd = np.zeros(3)
            self._prev_yaw_deg = self.initial_yaw_deg
            self._last_t = t
            return VelCmd(0.0, 0.0, 0.0, self.initial_yaw_deg)

        if self._phase == "STANDBY":
            self._phase = "A"

        if det is not None:
            self._last_any_decode_t = t   # v4 #2/#3 search trigger -- ANY decode, any phase
            _, pos_cap, _vel_cap, quat_cap = _interp_own_state(self._own_hist, det.t_capture)
            self._decode_positions.append(
                (det.t_capture, self._measure_pos_ned(pos_cap, quat_cap, det)))
            cutoff = t - self.cfg.acquire_window_s
            self._decode_positions = [p for p in self._decode_positions if p[0] >= cutoff]

        if self._phase == "A" and len(self._decode_positions) >= self.cfg.acquire_n:
            self._start_phase_b(t)

        if self._phase == "B":
            self._kf.predict(dt)
            if det is not None:
                self._update_kf(det, t)
            range_est = float(np.linalg.norm(self._kf.pos - own_pos))
            dropout_s = t - self._last_decode_t
            # v2 #4: the tag legitimately fills/leaves the frame very close
            # in -- that is not a real track loss, so suppress the fallback
            # inside `no_fallback_range_m` regardless of dropout duration.
            if dropout_s > self.cfg.fallback_s and range_est >= self.cfg.no_fallback_range_m:
                safe_vel = self._sane_fallback_vel(self._kf.vel)
                self._track = _BeliefTrack(self._kf.pos, safe_vel, t)
                self._cam_offset_body = None
                self._phase = "A"

        self._set_state("ENGAGE" if self._phase == "B" else "APPROACH", t)

        if self._phase == "A":
            cmd_v = self._phase_a_cmd(t, own_pos)
            pos_belief, vel_belief = self._track.at(t)
            yaw = _yaw_toward(pos_belief - own_pos, self._prev_yaw_deg)
            # v4 #3: once arrived-with-no-decode, also sweep yaw (+/- amplitude)
            # around the belief bearing -- same trigger as the vertical sweep
            # `_phase_a_cmd` just applied to `cmd_v`'s aim point, on a
            # DIFFERENT period so the two sweeps slowly precess.
            if self._last_search_elapsed is not None:
                yaw = yaw + self.cfg.yaw_search_amplitude_deg * math.sin(
                    2.0 * math.pi * self._last_search_elapsed / self.cfg.yaw_search_period_s)
            accel_h = accel_v = self.cfg.accel_max_ms2
            self.debug = PursuitDebug(r_est=pos_belief - own_pos, v_t_est=vel_belief,
                                      phase=self._phase, last_decode_t=self._last_decode_t)
        else:
            cmd_v, yaw = self._phase_b_cmd(dt, own, own_pos, det)
            accel_h, accel_v = self.cfg.accel_max_horiz_b_ms2, self.cfg.accel_max_vert_b_ms2
            self.debug = PursuitDebug(r_est=self._kf.pos - own_pos, v_t_est=self._kf.vel,
                                      phase=self._phase, last_decode_t=self._last_decode_t)

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

    def _search_elapsed(self, t: float, range_to_aim: float) -> Optional[float]:
        """None unless Phase A is CLOSE ENOUGH to its aim point
        (`range_to_aim < vsearch_arrival_range_m`, checked fresh every tick
        -- not a sustained-arrival latch, see below) AND it has been
        `vsearch_delay_s` since the last decode of ANY kind (including one
        that didn't reach acquire_n); else seconds into the search sweep.

        v4 #2/#3: "if the vehicle reaches its aim point and sees nothing,
        it should not sit there." FIRST IMPLEMENTATION required arrival to
        be CONTINUOUSLY held for `vsearch_delay_s` (a latch cleared the
        instant range_to_aim grew again) -- measured to never fire on the
        cases it targets: chasing a fast-moving, aim-error-rotated belief
        point makes the vehicle overshoot in and out of a small capture
        radius (arrived_t reset every ~6.5 s in one seed 0 trace, never
        holding continuously for 2 s) even while making no real progress.
        This version decouples "close enough right now" from "how long
        since anything was seen" -- the SUM of both signals genuinely
        found the search regime the spec asked for; see the task's final
        report for the before/after."""
        if range_to_aim >= self.cfg.vsearch_arrival_range_m:
            return None
        blind_s = t - max(self._last_any_decode_t, self.go_at_s)
        if blind_s < self.cfg.vsearch_delay_s:
            return None
        return blind_s - self.cfg.vsearch_delay_s

    def _phase_a_cmd(self, t: float, own_pos: np.ndarray) -> np.ndarray:
        cfg = self.cfg
        pos_belief, vel_belief = self._track.at(t)
        track_dir = _unit(vel_belief, np.array([1.0, 0.0, 0.0]))
        aim = pos_belief - cfg.d_behind_m * track_dir

        range_to_aim = float(np.linalg.norm(aim - own_pos))
        search_elapsed = self._search_elapsed(t, range_to_aim)
        self._last_search_elapsed = search_elapsed   # step() reuses this for the yaw sweep
        if search_elapsed is not None:
            v_offset = cfg.vsearch_amplitude_m * math.sin(
                2.0 * math.pi * search_elapsed / cfg.vsearch_period_s)
            aim = aim + np.array([0.0, 0.0, v_offset])   # NED down -- vertical sweep

        return vel_belief + cfg.kp_pos * (aim - own_pos)

    # ------------------------------------------------------------- Phase B law

    def _phase_b_cmd(self, dt: float, own: VehicleState, own_pos: np.ndarray,
                     det: Optional[Detection]) -> Tuple[np.ndarray, float]:
        """Dispatch: `hold_range_m` freeze, then CAMERA-FRAME steering
        (v6 #A) inside `cam_frame_range_m`, else the original NED-frame law
        (v2-v4). `cam_frame_range_m` is compared against the ABSOLUTE-FRAME
        filter's own `range_est` -- v6 #A keeps that filter running for far
        range and for the target-velocity feed-forward, per the spec."""
        cfg = self.cfg
        kf_pos, kf_vel = self._kf.pos, self._kf.vel
        r_est = kf_pos - own_pos
        range_est = float(np.linalg.norm(r_est))
        if range_est < cfg.hold_range_m:
            return self._prev_v_cmd.copy(), self._prev_yaw_deg   # "fly through"
        if range_est < cfg.cam_frame_range_m:
            return self._phase_b_cmd_camera_frame(dt, own, own_pos, det, kf_vel, range_est)
        self._cam_offset_body = None   # re-entry later always re-seeds fresh
        return self._phase_b_cmd_ned_frame(own, own_pos, kf_pos, kf_vel, r_est, range_est)

    def _phase_b_cmd_ned_frame(self, own: VehicleState, own_pos: np.ndarray,
                               kf_pos: np.ndarray, kf_vel: np.ndarray, r_est: np.ndarray,
                               range_est: float) -> Tuple[np.ndarray, float]:
        """v2 (pursuit_concept_v2.md #2/#3), used OUTSIDE `cam_frame_range_m`:

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
        los_dir = _unit(r_est, np.array([1.0, 0.0, 0.0]))
        v_close = float(np.clip(cfg.k_close * range_est, cfg.v_close_min_ms, cfg.v_close_max_ms))

        own_vel = np.asarray(own.vel_ned, dtype=np.float64)
        v_rel = kf_vel - own_vel
        ref_dir = _unit(v_rel, _unit(kf_vel, los_dir))
        r_perp = r_est - float(np.dot(r_est, ref_dir)) * ref_dir

        # v4 #1b: "arrive on a line that keeps the tag centred" -- inside
        # last_line_range_m, weight the lateral (miss) term more heavily so
        # lateral relative velocity gets nulled EARLY, before the final
        # straight-line close, instead of correcting and closing at the same
        # rate the whole way in.
        kp_lat_eff = cfg.kp_lat * (cfg.kp_lat_near_mult if range_est < cfg.last_line_range_m
                                   else 1.0)

        cmd_v = kf_vel + v_close * los_dir + kp_lat_eff * r_perp
        yaw = _yaw_toward(r_est, self._prev_yaw_deg)
        return cmd_v, yaw

    def _phase_b_cmd_camera_frame(self, dt: float, own: VehicleState, own_pos: np.ndarray,
                                  det: Optional[Detection], kf_vel: np.ndarray,
                                  range_est: float) -> Tuple[np.ndarray, float]:
        """v6 #A (isim/specs/pursuit_v6.md): steer on the tag's position IN
        THE IMAGE, not on an attitude-dependent absolute-frame position
        difference. An attitude bias rotates every NED-converted
        measurement by (bias x range), and that error GROWS as range
        closes -- worse, a filter reads the CHANGE as target velocity. The
        tag's body-frame offset (from bearing/elevation/range) needs no
        attitude at all; only the FINAL body->NED rotation (to issue a NED
        setpoint) touches the (possibly-perturbed) attitude, and that is
        now a bounded, SECOND-ORDER rotation of a command, not a growing
        first-order corruption of a position estimate.

        `self._cam_offset_body` (FRD: forward, right, down) is:
          - RECOMPUTED exactly from a fresh detection's bearing/elevation/
            range (no attitude);
          - else PROPAGATED using the relative body-frame velocity (own's
            own velocity and the filter's target-velocity feed-forward,
            BOTH rotated into body via the CURRENT attitude -- attitude
            error here is a per-tick rotation of a SHORT-lived integration,
            not a compounding absolute-position bias);
          - else (very first tick in this regime, no detection yet) seeded
            ONCE from the absolute-frame filter (an attitude-dependent
            seed, but a one-time one, not an ongoing corruption source).

        Command: body-frame forward = the filter's forward feed-forward +
        the EXISTING closing schedule (`k_close*range_est`, from the
        absolute-frame filter, per the spec); body-frame lateral/vertical =
        the filter's feed-forward + `kp_cam * offset`. The whole body-frame
        vector is rotated to NED by the (possibly-perturbed) attitude --
        the ONLY place attitude enters this command."""
        cfg = self.cfg
        r_bn = quat_to_rot(own.quat_wxyz)   # CURRENT (possibly perturbed) attitude

        if det is not None:
            dir_body = self._dir_body_boresight(det)
            self._cam_offset_body = det.range_m * dir_body
        elif self._cam_offset_body is None:
            self._cam_offset_body = r_bn.T @ (self._kf.pos - own_pos)
        else:
            own_vel_body = r_bn.T @ np.asarray(own.vel_ned, dtype=np.float64)
            v_ff_body = r_bn.T @ kf_vel
            self._cam_offset_body = self._cam_offset_body + (v_ff_body - own_vel_body) * dt

        v_ff_body = r_bn.T @ kf_vel
        v_close = float(np.clip(cfg.k_close * range_est, cfg.v_close_min_ms, cfg.v_close_max_ms))
        cmd_body = v_ff_body + np.array([v_close,
                                        cfg.kp_cam * self._cam_offset_body[1],
                                        cfg.kp_cam * self._cam_offset_body[2]])
        cmd_ned = r_bn @ cmd_body
        if not cfg.cam_frame_yaw:
            # ABLATION (measured -- see the task's final report): fall back
            # to the OLD, absolute-frame LOS yaw. `_dir_body_boresight`-based
            # yaw is sensitive to atan2 near a small/negative forward offset
            # (an overshoot, or a target that has passed abeam) -- a real,
            # continuous geometric sensitivity (fixed the discontinuous
            # version of this bug below), not just a tuning artifact; this
            # flag lets the VELOCITY command benefit from camera-frame
            # steering (v6 #A) while the YAW command keeps using the
            # (attitude-dependent, but empirically steadier) absolute-frame
            # law from v2-v4.
            r_est = self._kf.pos - own_pos
            return cmd_ned, _yaw_toward(r_est, self._prev_yaw_deg)
        # BUG FOUND BY MEASUREMENT (v6, see the task's final report): clamping
        # the forward component to a tiny POSITIVE floor before atan2 (instead
        # of just calling atan2, which already handles negative x correctly)
        # forced a ~180deg yaw DISCONTINUITY every time the target's body-
        # forward offset crossed zero (e.g. on an overshoot) -- the vehicle
        # then span trying to chase the artificial jump, corrupting the
        # attitude the propagation step (above) itself depends on, and never
        # recovered. `atan2(y, x)` alone is continuous and correct for a
        # target BEHIND the vehicle (x<0) too; only true (0,0) is degenerate,
        # guarded by `_unit`'s fallback via the offset's own norm instead.
        offset_body = self._cam_offset_body
        if float(np.linalg.norm(offset_body[:2])) < 1e-6:
            yaw = self._prev_yaw_deg
        else:
            yaw = math.degrees(own.yaw_rad) + math.degrees(
                math.atan2(offset_body[1], offset_body[0]))
        return cmd_ned, yaw

    # --------------------------------------------------------------- KF glue

    def _dir_body_boresight(self, det: Detection) -> np.ndarray:
        """Mount-tilt-corrected boresight direction in body FRD -- v6 #A:
        ATTITUDE-INDEPENDENT by construction. Only `det.bearing_deg`/
        `elevation_deg` (camera-relative) and the FIXED, always-correctly-
        known camera mount tilt are used; no vehicle attitude at all."""
        b, e = math.radians(det.bearing_deg), math.radians(det.elevation_deg)
        dir_boresight = np.array([math.cos(e) * math.cos(b), math.cos(e) * math.sin(b),
                                  -math.sin(e)])
        ct, st = math.cos(self._mount_tilt_rad), math.sin(self._mount_tilt_rad)
        ry = np.array([[ct, 0.0, st], [0.0, 1.0, 0.0], [-st, 0.0, ct]])
        return ry @ dir_boresight

    def _dir_ned(self, quat_wxyz: Tuple[float, float, float, float],
                det: Detection) -> np.ndarray:
        """Unit LOS direction in NED: `_dir_body_boresight` (attitude-
        independent) rotated into NED by an attitude quaternion. v3 #1: the
        caller passes the quaternion AT `det.t_capture` (interpolated from
        the own-state ring buffer), never the quaternion at arrival."""
        dir_body = self._dir_body_boresight(det)
        r_bn = quat_to_rot(quat_wxyz)
        return _unit(r_bn @ dir_body, dir_body)

    def _measure_pos_ned(self, pos_at_capture: np.ndarray,
                         quat_at_capture: Tuple[float, float, float, float],
                         det: Detection) -> np.ndarray:
        """z = own position AT t_capture + range * (LOS direction using the
        attitude AT t_capture) -- v3 #1. `pos_at_capture`/`quat_at_capture`
        must already be looked up at `det.t_capture`, not at arrival."""
        return np.asarray(pos_at_capture, dtype=np.float64) + \
            det.range_m * self._dir_ned(quat_at_capture, det)

    def _measurement_r(self, det: Detection, dir_ned: np.ndarray) -> np.ndarray:
        """Diagonal in a (along-LOS, cross, cross) basis aligned with
        `dir_ned`, per the spec: cross-range sigma = range*sigma_px/fx
        (floored -- see `PursuitConfig.cross_sigma_floor_m`'s docstring),
        along-range sigma = range^2*sigma_side_px/(fx*tag_side). Algebraically
        this IS `T @ diag(along^2, cross^2, cross^2) @ T.T` for any
        orthonormal `T` whose first column is `dir_ned` (v3 #3 asked to
        "build R in the camera LOS frame and rotate it into NED properly" --
        the closed form below is exactly that rotation, since for such a T,
        `T@diag(a,c,c)@T.T = a*dir(x)dir + c*(I - dir(x)dir)`, i.e. the line
        below; no explicit T is needed)."""
        cfg = self.cfg
        r = max(det.range_m, 1e-3)
        cross_sigma = max(r * cfg.sigma_px / cfg.fx_px, cfg.cross_sigma_floor_m)
        along_sigma = (r ** 2) * cfg.sigma_side_px / (cfg.fx_px * cfg.tag_side_m)
        outer = np.outer(dir_ned, dir_ned)
        return (cross_sigma ** 2) * np.eye(3) + (along_sigma ** 2 - cross_sigma ** 2) * outer

    def _update_kf(self, det: Detection, t: float) -> None:
        _, pos_cap, _vel_cap, quat_cap = _interp_own_state(self._own_hist, det.t_capture)
        dir_ned = self._dir_ned(quat_cap, det)
        z = np.asarray(pos_cap, dtype=np.float64) + det.range_m * dir_ned
        self._kf.update(z, self._measurement_r(det, dir_ned),
                        age_s=max(0.0, t - det.t_capture))
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
        self._kf.init(pos0, vel0, self.cfg.kf_p0_pos_m, self.cfg.kf_p0_vel_ms,
                     p0_bias_s=self.cfg.kf_p0_bias_s)
        self._last_decode_t = self._decode_positions[-1][0]
        self._phase = "B"


# ------------------------------------------------------------ v4 dual seeker

class DualTagSeeker:
    """isim `SeekerModel`: two independent tag seekers on the SAME mount,
    sharing every tick's (own, tgt) truth (v4 #1d/#3, MEASURE-only -- never
    a new default; see `isim.scenario.Scenario.inner_tag_side_m`/
    `second_tag_facing`). Two hardware ideas share this one mechanism:

      - a smaller, closer-range co-located tag next to the main one
        (`inner_tag_side_m`, #1d): the small tag decodes at short range
        where the big one may have left frame/blurred out;
      - a second tag at a DIFFERENT mount orientation (`second_tag_facing`,
        #3's "two-tag target"): covers a wider range of viewing angles than
        either orientation alone.

    Prefers `first`'s Detection when both decode this tick (in both use
    cases `first` is the bigger/nominal tag, whose along-range precision is
    at least as good as the second's at any range where both are visible);
    falls back to `second` only when `first` did not decode. The returned
    `FrameReport` is `first`'s if it exposed one, else `second`'s --
    documented modelling simplification (a real dual-tag rig would log
    both; isim's `FrameReport` is one-per-tick by the `SeekerModel`
    protocol), not a claim that only one tag's blur/incidence is real."""

    def __init__(self, first: SeekerModel, second: SeekerModel) -> None:
        self.first = first
        self.second = second

    def reset(self, rng: np.random.Generator) -> None:
        rng_first, rng_second = rng.spawn(2)
        self.first.reset(rng_first)
        self.second.reset(rng_second)

    def observe(self, t: float, own: VehicleState, tgt
               ) -> Tuple[Optional[Detection], Optional[FrameReport]]:
        det1, rep1 = self.first.observe(t, own, tgt)
        det2, rep2 = self.second.observe(t, own, tgt)
        det = det1 if det1 is not None else det2
        rep = rep1 if rep1 is not None else rep2
        return det, rep


# ---------------------------------------------------------------- v7 hybrid

@dataclass
class HybridSprintConfig:
    """Every tunable of Phase S/T (isim/specs/hybrid_v7.md). Defaults are
    reasonable starting points, not bench-fit -- see the task's final
    report for what was measured/tuned."""

    # --- Phase S: open-loop sprint (same kinematics as the flyby's dash) ---
    # TUNED 2026-09-17 (task's final report): the old 8.0 s default (matching
    # MissionConfig.dash_max_s) let Phase S run ~6.5 s PAST the target's true
    # crossing (t_cpa really lands ~1.4 s after go, at this nominal
    # geometry/speed) before the timer finally cut it -- at 9 m/s rear-tag,
    # all v5 errors on, that cost was catastrophic: 8.0 s -> 3% overall hit
    # vs plain pursuit's 82%, because it burned 8+ of the pursuit_window_s=25 s
    # scoring budget on a sprint that (honestly) almost never converts to a
    # first-pass hit for a rear tag (0-3/100 across every sprint_max_s tried,
    # rear or rear_dual35) before falling back to the same Phase A/B pursuit
    # law that alone gets 82%. Shorter sprints uniformly score higher because
    # they leave more of the budget for that proven fallback -- 0.5 s -> 81%
    # (a rear tag), i.e. sprint_max_s this short means Phase T fires almost
    # immediately after go_at_s and hybrid degenerates to near-pure-pursuit.
    # 2.0 s is chosen as the default -- NOT the top scorer (0.5 s scores
    # higher, see the report) -- because it is the shortest value at which
    # the sprint genuinely reaches the natural pass (t_cpa) and the
    # `growing`-after-pass trigger becomes reachable at all (11/100 dual-tag
    # runs) rather than the timer firing before the pass ever happens; the
    # spec calls for a sprint that "opportunistically" tries the pass, and a
    # sprint that never reaches the pass point cannot try anything. This is a
    # disclosed tradeoff, not a free win -- see the report's sprint_max_s
    # sweep table for the full curve and reconsider if the geometry changes.
    sprint_max_s: float = 2.0
    kp_alt_sprint: float = 1.0       # 1/s, altitude-hold gain during the sprint
    v_vert_max_sprint_ms: float = 3.0   # m/s, vertical speed cap during the sprint

    # --- Phase S: "steer the pass" -- a BOUNDED correction near the pass,
    # only while the background filter is healthy and the estimated time-
    # to-closest-approach is inside `correction_window_s` ("the last ~1s
    # before the pass"). Bounded to `max_correction_ms` -- "limited to what
    # keeps the tag in view", i.e. a nudge, not a re-aim. ---
    correction_window_s: float = 1.0
    kp_correction: float = 1.0       # 1/s, lateral/vertical correction gain
    max_correction_ms: float = 3.0   # m/s, correction magnitude cap

    # --- Phase S -> T trigger: range (from the background filter, if
    # healthy) growing for `growth_hold_s` past its own running minimum by
    # more than `growth_margin_m`, OR the sprint timer expires first. ---
    growth_margin_m: float = 0.5
    growth_hold_s: float = 0.3


class HybridGuidance:
    """isim `Guidance`: Phase S (open-loop sprint, same aim/speed/accel
    profile as the "flyby" concept's coded dash, holding the believed
    altitude, with a background target filter and a bounded pass-steering
    correction) -> Phase T (turn-around: brake + hand over to a fresh
    `PursuitRendezvousGuidance`, seeded with the REAL current velocity/yaw
    for a physically continuous brake, and with the background filter's
    state if it is healthy). See isim/specs/hybrid_v7.md.

    Phase S's OWN filter reuses `self._pursuit`'s pure geometry helpers
    (`_dir_body_boresight`/`_dir_ned`/`_measure_pos_ned`/`_measurement_r`)
    by calling them directly on the held (not-yet-driving) instance --
    zero duplication, and `self._pursuit` itself is never mutated by this
    until the Phase S->T handover, so its own Phase-A/B state machine
    cannot be polluted by Phase S's activity."""

    def __init__(self, scfg: HybridSprintConfig, pcfg: PursuitConfig,
                heading_deg: float, dash_speed_ms: float, dash_accel_ms2: float,
                believed_alt_m: float, go_at_s: float,
                belief_pos0_ned: np.ndarray, belief_vel_ned: np.ndarray,
                cam_mount_tilt_up_deg: float = 0.0) -> None:
        self.scfg = scfg
        self.pcfg = pcfg
        self.heading_deg = float(heading_deg)
        self.dash_speed_ms = float(dash_speed_ms)
        self.dash_accel_ms2 = float(dash_accel_ms2)
        self.believed_alt_m = float(believed_alt_m)
        self.go_at_s = float(go_at_s)
        self.belief_vel_ned = np.asarray(belief_vel_ned, dtype=np.float64).copy()
        self._mount_tilt_rad = math.radians(cam_mount_tilt_up_deg)
        # Held from construction so Phase S can reuse its pure helpers; only
        # DRIVES the vehicle once Phase T hands over (see `_start_turn`).
        self._pursuit = PursuitRendezvousGuidance(
            pcfg, belief_pos0_ned, belief_vel_ned, go_at_s=go_at_s,
            initial_yaw_deg=heading_deg, cam_mount_tilt_up_deg=cam_mount_tilt_up_deg)
        self.state_log: List[Tuple[float, str]] = []
        self.events: List[str] = []
        self.last_decision: Optional[_Decision] = None
        # v7: "report the time lost in the turn-around and how far behind
        # the vehicle ends up" -- read directly by isim.mc (plain public
        # attributes, matching how `state_log`/`last_decision` are already
        # read off guidance objects elsewhere in this codebase).
        self.t_turn_start: Optional[float] = None
        self.turn_reason: Optional[str] = None
        self.range_est_at_turn: Optional[float] = None
        self.reset()

    def reset(self) -> None:
        self._phase = "STANDBY"
        self._dash_start_t: Optional[float] = None
        self._last_t: Optional[float] = None
        self._kf = _ConstVelKF(self.pcfg.kf_q_accel_ms2, estimate_ts_bias=self.pcfg.estimate_ts_bias)
        self._decode_positions: List[Tuple[float, np.ndarray]] = []
        self._last_decode_t = -math.inf
        self._own_hist: Deque[_OwnSample] = deque()
        self._healthy = False
        self._min_range_seen = math.inf
        self._t_min_range = 0.0
        self.t_turn_start = None
        self.turn_reason = None
        self.range_est_at_turn = None
        self._pursuit.reset()
        self.state_log = []
        self.events = []
        self.last_decision = None

    def _set_state(self, state: str, t: float) -> None:
        if not self.state_log or self.state_log[-1][1] != state:
            self.state_log.append((t, state))
        self.last_decision = _Decision(state=state)

    # ------------------------------------------------------- Phase S filter

    def _observe_own(self, t: float, own: VehicleState) -> None:
        own_pos = np.asarray(own.pos_ned, dtype=np.float64)
        self._own_hist.append((t, own_pos.copy(), np.asarray(own.vel_ned, dtype=np.float64).copy(),
                               tuple(float(c) for c in own.quat_wxyz)))
        cutoff = t - self.pcfg.own_state_history_s
        while len(self._own_hist) > 2 and self._own_hist[0][0] < cutoff:
            self._own_hist.popleft()

    def _observe_detection(self, t: float, det: Optional[Detection]) -> None:
        """Phase S's background filter: accumulate toward acquisition, or
        (once healthy) update the KF -- the SAME acquisition/measurement
        logic as `PursuitRendezvousGuidance`, reusing its pure helpers on
        `self._pursuit` (never mutating it) rather than duplicating them."""
        if det is None:
            return
        _, pos_cap, _vel_cap, quat_cap = _interp_own_state(self._own_hist, det.t_capture)
        if not self._healthy:
            self._decode_positions.append(
                (det.t_capture, self._pursuit._measure_pos_ned(pos_cap, quat_cap, det)))
            cutoff = t - self.pcfg.acquire_window_s
            self._decode_positions = [p for p in self._decode_positions if p[0] >= cutoff]
            if len(self._decode_positions) >= self.pcfg.acquire_n:
                pos0 = self._decode_positions[-1][1]
                # v2 Bug A fix, reused: seed velocity from the pre-flight
                # belief, never a 2-point finite difference.
                self._kf.init(pos0, self.belief_vel_ned, self.pcfg.kf_p0_pos_m,
                             self.pcfg.kf_p0_vel_ms, p0_bias_s=self.pcfg.kf_p0_bias_s)
                self._last_decode_t = self._decode_positions[-1][0]
                self._healthy = True
        else:
            dir_ned = self._pursuit._dir_ned(quat_cap, det)
            z = np.asarray(pos_cap, dtype=np.float64) + det.range_m * dir_ned
            self._kf.update(z, self._pursuit._measurement_r(det, dir_ned),
                            age_s=max(0.0, t - det.t_capture))
            self._last_decode_t = t

    # -------------------------------------------------------- Phase S: sprint

    def _sprint_cmd(self, t: float, own: VehicleState, own_pos: np.ndarray) -> np.ndarray:
        """Open-loop sprint (same aim/speed/accel as the flyby's coded
        dash) holding the believed altitude, plus a BOUNDED lateral/
        vertical correction in the last `correction_window_s` before the
        estimated pass, if the background filter is healthy -- "an
        improved fly-by, not a blind one"."""
        # BUG FOUND BY THE TEST SUITE (v7): `self._dash_start_t or t` returns
        # `t` whenever `_dash_start_t` is falsy -- which includes the
        # perfectly legitimate value 0.0 (a `go_at_s=0.0` engagement), not
        # just `None`. Must be an explicit `is not None` check.
        dash_start = self._dash_start_t if self._dash_start_t is not None else t
        elapsed = t - dash_start
        fwd_speed = dash_forward_speed(self.dash_speed_ms, self.dash_accel_ms2, elapsed)
        heading_rad = math.radians(self.heading_deg)
        v_n = fwd_speed * math.cos(heading_rad)
        v_e = fwd_speed * math.sin(heading_rad)
        alt_now_m = -float(own_pos[2])
        alt_err = self.believed_alt_m - alt_now_m
        v_d = float(np.clip(-self.scfg.kp_alt_sprint * alt_err,
                            -self.scfg.v_vert_max_sprint_ms, self.scfg.v_vert_max_sprint_ms))
        cmd = np.array([v_n, v_e, v_d])

        if self._healthy:
            r_est = self._kf.pos - own_pos
            v_rel = self._kf.vel - np.asarray(own.vel_ned, dtype=np.float64)
            denom = float(np.dot(v_rel, v_rel))
            t_to_cpa = (-float(np.dot(r_est, v_rel)) / denom) if denom > 1e-9 else math.inf
            if 0.0 <= t_to_cpa <= self.scfg.correction_window_s:
                ref_dir = _unit(v_rel, np.array([1.0, 0.0, 0.0]))
                r_perp = r_est - float(np.dot(r_est, ref_dir)) * ref_dir
                correction = _clip_norm(self.scfg.kp_correction * r_perp,
                                        self.scfg.max_correction_ms)
                cmd = cmd + correction
        return cmd

    # -------------------------------------------------- Phase S -> T handover

    def _start_turn(self, t: float, own: VehicleState, own_pos: np.ndarray, reason: str) -> None:
        self._phase = "T"
        self.t_turn_start = t
        self.turn_reason = reason
        self.range_est_at_turn = (float(np.linalg.norm(self._kf.pos - own_pos))
                                  if self._healthy else None)
        self._pursuit.reset()
        if self._healthy:
            # Hand the ALREADY-ACQUIRED filter state straight to pursuit's
            # Phase B -- "turn toward the target's predicted position using
            # the filter if it has one". Sharing the object is fine: Phase
            # S never touches it again after this call.
            self._pursuit._kf = self._kf
            self._pursuit._phase = "B"
            self._pursuit._last_decode_t = self._last_decode_t
        # else: pursuit stays in its freshly-reset Phase A, which already
        # extrapolates belief_pos0/vel0 forward from go_at_s -- "else the
        # pre-flight belief", with no extra code needed.

        # "Brake": seed pursuit's OWN slew-limiter with the REAL current
        # velocity/yaw (not the zero/initial-yaw its reset() assumes), so
        # its first commands decelerate FROM the true fast sprint velocity
        # at accel_max_horiz_b_ms2/accel_max_vert_b_ms2, not from a
        # fictitious rest state.
        self._pursuit._prev_v_cmd = np.asarray(own.vel_ned, dtype=np.float64).copy()
        self._pursuit._prev_yaw_deg = math.degrees(own.yaw_rad)

    # ------------------------------------------------------------------ step

    def step(self, t: float, own: VehicleState, det: Optional[Detection]) -> VelCmd:
        own_pos = np.asarray(own.pos_ned, dtype=np.float64)
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)

        if t < self.go_at_s:
            self._set_state("STANDBY", t)
            self._last_t = t
            return VelCmd(0.0, 0.0, 0.0, self.heading_deg)

        if self._phase == "STANDBY":
            self._phase = "S"
            self._dash_start_t = t

        if self._phase == "S":
            self._observe_own(t, own)
            if self._healthy:
                self._kf.predict(dt)
            self._observe_detection(t, det)

            if self._healthy:
                range_est = float(np.linalg.norm(self._kf.pos - own_pos))
                if range_est < self._min_range_seen:
                    self._min_range_seen = range_est
                    self._t_min_range = t
                growing = (range_est > self._min_range_seen + self.scfg.growth_margin_m
                          and t - self._t_min_range > self.scfg.growth_hold_s)
            else:
                growing = False
            dash_start = self._dash_start_t if self._dash_start_t is not None else t
            timer_expired = (t - dash_start) > self.scfg.sprint_max_s
            if growing or timer_expired:
                self._start_turn(t, own, own_pos, "growing" if growing else "timer")

        if self._phase == "S":
            cmd_v = self._sprint_cmd(t, own, own_pos)
            yaw = self.heading_deg
            self._set_state("SPRINT", t)
        else:
            pursuit_cmd = self._pursuit.step(t, own, det)
            cmd_v = np.array([pursuit_cmd.v_north, pursuit_cmd.v_east, pursuit_cmd.v_down])
            yaw = pursuit_cmd.yaw_deg
            inner_state = (self._pursuit.last_decision.state
                           if self._pursuit.last_decision is not None else "T")
            self._set_state(inner_state, t)

        self._last_t = t
        return VelCmd(v_north=float(cmd_v[0]), v_east=float(cmd_v[1]), v_down=float(cmd_v[2]),
                      yaw_deg=yaw)
