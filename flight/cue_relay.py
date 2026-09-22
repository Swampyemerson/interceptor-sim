"""flight.cue_relay -- pre-flight GPS cue relay: target-quad GPS -> the
interceptor's belief seed (design doc: docs/cue_relay_plan.md).

CONTEXT (docs/launch_mechanism_plan.md Sec 2, ADR-0103/0105, constraint
`no-datalink`, docs/next.md "Launch-aim cue -- RULED 2026-07-26"): the real
interceptor launches off a GPS-derived estimate of the target's position and
velocity, LATCHED at the trigger instant, after which no further target
information reaches the vehicle -- the ELRS link is arm/kill only and the SiK
link is telemetry-monitoring only (constraint `no-datalink`). This module is
the PIPING: ~2 s of the target's own GPS telemetry, relayed target -> ground
-> Pi pre-launch, fitted into exactly the two numbers
`flight.deploy.real_flight` already knows how to consume --
`--target-start`/`--target-vel` (the C1 lead-solve inputs) and the pursuit
terminal's `belief_r0_ned`/`belief_vel0_ned` (`build_terminal()`).

HONESTY (CLAUDE.md "the boundary covers pre-flight GIVENS too"; ledger entry
`launch-aim-derived-from-ground-truth`). Before this module existed, the only
way to fill `--target-start`/`--target-vel` was to type in the operator-
programmed AUTO-leg waypoints -- a GIVEN-PERFECT input, honest only insofar as
the real flight tracks its plan exactly. A relayed GPS cue is the fix: it
reports where the target's GPS ACTUALLY says it is, with GPS's actual error
baked in, which is why it is graded `given-noisy` in the plan doc's
assumptions register, not `given-perfect`. Every value this module emits is
read BEFORE the GO edge; `CueSolution` carries its own latch time so the one
legal post-latch arithmetic op (extrapolate the constant-velocity belief
forward to the trigger instant) is an explicit, single method call, not an
implicit re-read.

FAIL CLOSED (CLAUDE.md "instruments are evidence" / "fail-closed on measured
quantities"). `fit_cue()` never returns a degraded/default CueSolution -- on
too few points, too short a span, stale data, a non-converging (rank-
deficient/degenerate-timestamp) fit, or a residual that says the data isn't
self-consistent, it raises `CueQualityError`. The caller's job (the
`--cue-json` hookup sketched in docs/cue_relay_plan.md) is to treat that as a
hard abort to the manual-entry fallback, never to fly on a bad cue.

Pure stdlib + numpy. `pymavlink` is OPTIONAL and imported only inside
`mavlink_stream_to_cue_records` -- the schema, the fitter, the quality gate,
and the CLI-arg loader have zero MAVLink dependency and are fully testable
without a radio, ArduPilot, or pymavlink installed (confirmed: pymavlink is
NOT in this repo's venv as of this module's introduction).
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field, replace
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

__all__ = [
    "CueRecord", "CueSolution", "CueQualityError",
    "lla_to_ned", "fit_cue",
    "write_cue_jsonl", "read_cue_jsonl",
    "cue_record_from_global_position_int", "mavlink_stream_to_cue_records",
    "cue_to_target_start_arg", "cue_to_target_vel_arg",
    "load_cue_for_real_flight",
]

# --------------------------------------------------------------- quality gate

DEFAULT_MIN_POINTS = 8          # ~1-1.6 s of stream at 5-8 Hz
DEFAULT_MIN_SPAN_S = 1.0        # need real time-base to fit a velocity at all
DEFAULT_MAX_STALENESS_S = 1.0   # last record must be this fresh at emission
DEFAULT_MAX_RESIDUAL_M = 5.0    # position-fit RMS residual ceiling

# ---- turning-target (constant-turn-rate) model constants -------------------
# (docs/cue_relay_plan.md "curved tracks"; every one is a DETECTION/SAFETY
# threshold, not a tuning knob -- the model is selected by evidence, below.)
#
# The turn must be TURN_T_STAT_MIN-sigma significant before the CT model is
# believed. This is the F-test on the one nested turn parameter (F = t^2):
# a curve indistinguishable from noise MUST select CV, because a spurious
# turn on a straight track injects error while a missed weak turn costs
# almost nothing (CV is then nearly right). Two-sided false-positive rate at
# 4.0 is ~6e-5; at the bench noise a 20 deg/s turn measures t ~ 5.8 (see the
# plan doc's arithmetic), so real turns clear it with margin.
TURN_T_STAT_MIN = 4.0
# Below this horizontal speed a heading -- and therefore a turn RATE -- is
# not meaningfully defined from the data; stay CV (the static bench log
# lands here by design).
TURN_SPEED_FLOOR_MS = 1.0
# A fitted |turn rate| above this is outside the coordinated-turn regime this
# model (and the chase terminal downstream) can use -- ~69 deg/s is ~1 g of
# lateral acceleration at the 9 m/s target speed. Such a track falls through
# to the CV path, whose residual gate then refuses it (fail closed), rather
# than emitting a cue we know the model cannot represent.
TURN_OMEGA_MAX_RAD_S = 1.2
# extrapolate_to() caps for a CT solution (both raise CueQualityError --
# REFUSE, never silently straighten a known-turning belief): the 1-sigma
# heading uncertainty accumulated over the extrapolation must stay under
# ~20 deg (0.35 rad -- the edge of the chase terminal's comfortable
# aim-error band), and the extrapolated arc itself must stay under a
# quarter turn (beyond that "constant turn rate" is speculation).
CT_EXTRAP_MAX_HEADING_SIGMA_RAD = 0.35
CT_EXTRAP_MAX_ARC_RAD = math.pi / 2.0


class CueQualityError(RuntimeError):
    """Raised by `fit_cue` when the quality gate is not met. Every raise is a
    real, specific reason the data must not be trusted -- there is no path in
    this module that substitutes a default value for a measured quantity
    (CLAUDE.md "fail-closed on measured quantities")."""


# ------------------------------------------------------------------- schema
#
# JSON-lines cue record, one object per line, written/read by
# `write_cue_jsonl`/`read_cue_jsonl`:
#
#   {"t": <float>, "lat": <deg>, "lon": <deg>, "alt_m_msl": <m>,
#    "vn": <m/s, optional>, "ve": <m/s, optional>, "vd": <m/s, optional>}
#
# Field meanings:
#   t          seconds, on the clock of whichever machine LAST TOUCHED the
#              record before it reached the Pi (recommended: the Pi's own
#              monotonic receive-time -- see docs/cue_relay_plan.md "why
#              receive-time, not FC time"). Only needs to be self-consistent
#              across one file's records (relative spacing + the caller's own
#              `now_t`/`trigger_t` matter, not the absolute epoch).
#   lat, lon   WGS84 degrees, decimal (NOT the raw MAVLink 1e7-scaled int).
#   alt_m_msl  metres, any ONE consistent vertical datum used by both the
#              target's report and `own_alt_m_msl` passed to `fit_cue` --
#              MSL is the MAVLink GLOBAL_POSITION_INT convention, hence the
#              field name.
#   vn/ve/vd   OPTIONAL m/s, NED -- the FC's own reported ground velocity
#              (Doppler/EKF-derived, typically several times more precise
#              than a position-difference fit; see the plan doc's error
#              budget). When every record in a fit carries these, `fit_cue`
#              prefers them over the position-slope fit and reports
#              `vel_source="reported"`; otherwise it falls back to the
#              position fit and reports `vel_source="fit"`. Never silently
#              mixed per-axis.


@dataclass
class CueRecord:
    t: float
    lat: float
    lon: float
    alt_m_msl: float
    vn: Optional[float] = None
    ve: Optional[float] = None
    vd: Optional[float] = None


@dataclass
class CueSolution:
    """The fitted pre-flight belief -- everything `flight.deploy.real_flight`
    needs, plus enough provenance to audit/log it. `belief_r0_ned`/
    `belief_vel0_ned` are (north, east, down) metres/m-per-s, the TARGET
    relative to `origin_lla` (the interceptor's own pre-launch GPS fix --
    itself a pre-flight given, never a live read after latch)."""

    latch_t: float
    belief_r0_ned: Tuple[float, float, float]
    belief_vel0_ned: Tuple[float, float, float]
    speed_ms: float
    heading_deg: float
    residual_rms_m: float
    n_points: int
    span_s: float
    vel_source: str                       # "reported" or "fit"
    origin_lla: Tuple[float, float, float]
    # Motion model the fit SELECTED (docs/cue_relay_plan.md "curved tracks"):
    #   "cv"  constant velocity (straight) -- the default, and the only model
    #         a turn indistinguishable from noise is allowed to produce;
    #   "ct"  constant turn rate, horizontal (coordinated turn) + CV vertical.
    # `omega_rad_s` is the fitted heading rate (>0 = heading increasing,
    # NED/compass sense) and `omega_sigma_rad_s` its 1-sigma from the same
    # fit -- both 0.0 for "cv".
    model: str = "cv"
    omega_rad_s: float = 0.0
    omega_sigma_rad_s: float = 0.0

    def extrapolate_to(self, trigger_t: float) -> "CueSolution":
        """The ONE legal post-latch operation (docs/cue_relay_plan.md "latch
        semantics"): advance the belief to the actual trigger instant ALONG
        THE FITTED MODEL. Pure arithmetic on numbers already frozen at fit
        time -- does not read any sensor.

        "cv": straight, at the latched constant velocity (unchanged
        behaviour). "ct": along the fitted circular arc -- the velocity
        vector rotates at `omega_rad_s` and the position follows the arc;
        the vertical channel stays CV. FAIL-CLOSED CAPS on "ct" (raise
        CueQualityError, never silently straighten a known-turning belief):
        the accumulated 1-sigma heading uncertainty `omega_sigma * dt` must
        stay under CT_EXTRAP_MAX_HEADING_SIGMA_RAD, and the arc `omega * dt`
        under CT_EXTRAP_MAX_ARC_RAD -- past either bound the honest move is
        to re-latch fresher data, not to guess."""
        dt = float(trigger_t) - self.latch_t
        vn, ve, vd = self.belief_vel0_ned
        n0, e0, d0 = self.belief_r0_ned
        w = self.omega_rad_s
        if self.model == "ct" and abs(w) > 1e-9 and dt != 0.0:
            if self.omega_sigma_rad_s * abs(dt) > CT_EXTRAP_MAX_HEADING_SIGMA_RAD:
                raise CueQualityError(
                    f"CT extrapolation refused: heading uncertainty "
                    f"{math.degrees(self.omega_sigma_rad_s * abs(dt)):.1f} deg "
                    f"(omega_sigma {math.degrees(self.omega_sigma_rad_s):.2f} "
                    f"deg/s x {abs(dt):.2f} s) exceeds "
                    f"{math.degrees(CT_EXTRAP_MAX_HEADING_SIGMA_RAD):.0f} deg "
                    f"-- re-latch fresher data instead of guessing")
            if abs(w * dt) > CT_EXTRAP_MAX_ARC_RAD:
                raise CueQualityError(
                    f"CT extrapolation refused: arc {math.degrees(abs(w * dt)):.0f} "
                    f"deg exceeds {math.degrees(CT_EXTRAP_MAX_ARC_RAD):.0f} deg "
                    f"-- a quarter-turn of 'constant turn rate' is speculation; "
                    f"re-latch fresher data")
            s, c = math.sin(w * dt), math.cos(w * dt)
            # position: integral of the rotating velocity over dt (exact arc)
            dn = (vn * s - ve * (1.0 - c)) / w
            de = (ve * s + vn * (1.0 - c)) / w
            vn2, ve2 = vn * c - ve * s, ve * c + vn * s
            r_new = (n0 + dn, e0 + de, d0 + vd * dt)
            return replace(
                self, belief_r0_ned=r_new, belief_vel0_ned=(vn2, ve2, vd),
                heading_deg=math.degrees(math.atan2(ve2, vn2)) % 360.0,
                latch_t=float(trigger_t))
        r_new = (n0 + vn * dt, e0 + ve * dt, d0 + vd * dt)
        return replace(self, belief_r0_ned=r_new, latch_t=float(trigger_t))


# --------------------------------------------------------------- geometry

_EARTH_R_M = 6378137.0  # WGS84 semi-major axis; tangent-plane approx is
# accurate to << 1 cm at the <=100 m ranges this project's engagements
# happen at (curvature error grows as range^3, irrelevant here) -- adequate
# for a cue whose OWN GPS noise is 1.5-2.5 m; see docs/cue_relay_plan.md.


def lla_to_ned(lat: float, lon: float, alt_m_msl: float,
                origin_lat: float, origin_lon: float, origin_alt_m_msl: float
                ) -> Tuple[float, float, float]:
    """Flat-earth local tangent-plane projection -> (north, east, down)
    metres relative to `origin_*`. See module docstring for the accuracy
    note."""
    lat0_rad = math.radians(origin_lat)
    north = math.radians(lat - origin_lat) * _EARTH_R_M
    east = math.radians(lon - origin_lon) * _EARTH_R_M * math.cos(lat0_rad)
    down = -(alt_m_msl - origin_alt_m_msl)
    return north, east, down


# ------------------------------------------------------------------- fitter


def fit_cue(records: Sequence[CueRecord], own_lat: float, own_lon: float,
            own_alt_m_msl: float, *,
            now_t: Optional[float] = None,
            min_points: int = DEFAULT_MIN_POINTS,
            min_span_s: float = DEFAULT_MIN_SPAN_S,
            max_staleness_s: float = DEFAULT_MAX_STALENESS_S,
            max_residual_m: float = DEFAULT_MAX_RESIDUAL_M) -> CueSolution:
    """Fit ~2 s of target GPS records + the interceptor's own pre-launch
    position into a `CueSolution`. FAILS CLOSED (raises `CueQualityError`,
    never degrades) on: too few points, too short a span to fit a velocity,
    stale data (only checked if `now_t` given), a non-converging/degenerate
    fit, or a residual that says NEITHER motion model explains the points.

    MOTION MODELS (docs/cue_relay_plan.md "curved tracks"): the fitter tries
    constant-velocity (CV, straight) and constant-turn-rate (CT, a
    coordinated horizontal turn + CV vertical) and selects CT only when the
    turn is TURN_T_STAT_MIN-sigma significant -- the F-test on the one
    nested turn parameter -- AND physically in-regime (speed above
    TURN_SPEED_FLOOR_MS, |omega| under TURN_OMEGA_MAX_RAD_S). The turn is
    measured from the REPORTED per-sample Doppler velocities when every
    record carries them (a linear trend in the velocity vector, evaluated at
    the latch instant -- far cleaner than differentiating positions), else
    from a quadratic position fit (which at a ~2 s window is too noisy to
    clear the significance bar, by design -- position-only curve detection
    needs a ~4-5 s window; the arithmetic is in the plan doc).

    `belief_r0_ned`/`belief_vel0_ned` are the fitted state AT THE LAST
    RECORD's time (the "latch" instant) -- an average over all `n_points`
    samples via the regression (rotated to the latch instant for CT), not
    just the raw last fix, so their noise is already reduced relative to a
    single GPS sample (see docs/cue_relay_plan.md error budget)."""
    if len(records) < min_points:
        raise CueQualityError(
            f"only {len(records)} cue records, need >= {min_points}")
    recs = sorted(records, key=lambda r: r.t)
    span = recs[-1].t - recs[0].t
    if span < min_span_s:
        raise CueQualityError(
            f"cue span {span:.2f}s < {min_span_s:.2f}s -- not enough time "
            f"to fit a velocity")
    if now_t is not None and (now_t - recs[-1].t) > max_staleness_s:
        raise CueQualityError(
            f"stale: last record is {now_t - recs[-1].t:.2f}s old "
            f"(limit {max_staleness_s:.2f}s)")

    t_latch = recs[-1].t
    ts = np.array([r.t - t_latch for r in recs], dtype=np.float64)  # <=0, 0 at latch
    if np.ptp(ts) < 1e-6:
        raise CueQualityError(
            "degenerate timestamps (all records at ~the same time) -- "
            "cannot fit a velocity: non-converging fit")

    pos = np.array(
        [lla_to_ned(r.lat, r.lon, r.alt_m_msl, own_lat, own_lon, own_alt_m_msl)
         for r in recs], dtype=np.float64)  # (N, 3) NED

    A = np.vstack([ts, np.ones_like(ts)]).T  # columns: [t, 1] -> slope, intercept
    try:
        sol, _res, rank, _sv = np.linalg.lstsq(A, pos, rcond=None)
    except np.linalg.LinAlgError as e:
        raise CueQualityError(f"position fit did not converge: {e}") from e
    if rank < 2 or not np.all(np.isfinite(sol)):
        raise CueQualityError(
            "position fit is rank-deficient or produced a non-finite "
            "result -- non-converging fit")

    v_fit = sol[0]   # (3,) NED m/s, from the position slope
    p0_fit = sol[1]  # (3,) NED m, position AT t=0 == t_latch
    pred_cv = A @ sol
    residual_cv = float(np.sqrt(np.mean(np.sum((pos - pred_cv) ** 2, axis=1))))

    reported = [(r.vn, r.ve, r.vd) for r in recs
                if r.vn is not None and r.ve is not None and r.vd is not None]
    all_reported = len(reported) == len(recs)
    vel_source = "reported" if all_reported else "fit"

    # ---- CT candidate (turn detection) -- docs/cue_relay_plan.md ----------
    ct = _detect_turn(ts, pos, reported if all_reported else None)

    if ct is not None:
        # residual of the SELECTED (CT) model: quadratic horizontal (the arc's
        # local expansion over the short window) + linear vertical.
        residual_ct = ct["residual_rms_m"]
        if not math.isfinite(residual_ct) or residual_ct > max_residual_m:
            raise CueQualityError(
                f"turning-model residual {residual_ct:.2f} m exceeds "
                f"{max_residual_m:.2f} m -- the points fit neither a straight "
                f"line nor a coordinated turn (bad fix, multipath, or a "
                f"non-turn maneuver); refusing to emit a cue")
        vn, ve, vd = ct["v_latch_ned"]
        r0 = ct["r0_ned"]
        return CueSolution(
            latch_t=t_latch, belief_r0_ned=r0, belief_vel0_ned=(vn, ve, vd),
            speed_ms=math.hypot(vn, ve),
            heading_deg=math.degrees(math.atan2(ve, vn)) % 360.0,
            residual_rms_m=residual_ct, n_points=len(recs), span_s=span,
            vel_source=vel_source, origin_lla=(own_lat, own_lon, own_alt_m_msl),
            model="ct", omega_rad_s=ct["omega_rad_s"],
            omega_sigma_rad_s=ct["omega_sigma_rad_s"])

    # ---- CV (straight) path -- byte-identical to the pre-curve fitter -----
    if not math.isfinite(residual_cv) or residual_cv > max_residual_m:
        raise CueQualityError(
            f"position-fit residual {residual_cv:.2f} m exceeds "
            f"{max_residual_m:.2f} m -- the points don't lie near a "
            f"straight constant-velocity line (bad fix, multipath, or a "
            f"maneuvering target); refusing to emit a cue")
    if all_reported:
        vn = float(np.mean([v[0] for v in reported]))
        ve = float(np.mean([v[1] for v in reported]))
        vd = float(np.mean([v[2] for v in reported]))
    else:
        vn, ve, vd = float(v_fit[0]), float(v_fit[1]), float(v_fit[2])

    return CueSolution(
        latch_t=t_latch,
        belief_r0_ned=(float(p0_fit[0]), float(p0_fit[1]), float(p0_fit[2])),
        belief_vel0_ned=(vn, ve, vd),
        speed_ms=math.hypot(vn, ve),
        heading_deg=math.degrees(math.atan2(ve, vn)) % 360.0,
        residual_rms_m=residual_cv, n_points=len(recs), span_s=span,
        vel_source=vel_source, origin_lla=(own_lat, own_lon, own_alt_m_msl))


def _detect_turn(ts: np.ndarray, pos: np.ndarray,
                 reported: Optional[List[Tuple[float, float, float]]]):
    """Try the constant-turn-rate model. Returns None (stay CV) unless the
    turn is TURN_T_STAT_MIN-sigma significant AND in-regime; else a dict with
    the CT state at the latch instant (t=0; `ts` are <=0).

    TWO SOURCES, in preference order (docs/cue_relay_plan.md):
      * reported per-sample Doppler velocities (`reported` is the full list):
        fit a LINEAR TREND v(t) per horizontal axis; the trend's slope is the
        target's acceleration, whose component PERPENDICULAR to the velocity
        is the turn: omega = (vn*ae - ve*an)/|v|^2. Doppler velocities are
        several times cleaner than differentiated positions, which is what
        makes a 2 s window enough here (sigma_a ~ sigma_v*sqrt(12/N)/T).
      * position-only fallback (`reported` is None): quadratic fit per
        horizontal axis; acceleration = 2x the quadratic coefficient. At a
        2 s window its noise (~sigma_pos*sqrt(720/N)/T^2) is at or below the
        size of a real turn, so the significance bar keeps this path CV
        unless the window is long (~4-5 s+) or the turn violent -- by design.

    The significance statistic is |omega|/sigma_omega, i.e. the t of the one
    turn parameter (F = t^2 against the nested straight model), with sigma
    taken from the fit's OWN residuals -- no assumed noise level.

    ONCE DETECTED, the latch-instant velocity comes from rotating every
    velocity sample forward to the latch by the fitted omega and averaging
    (exact for true CT motion, noise ~sigma_v/sqrt(N)) -- this removes the
    chord bias that makes a whole-window MEAN velocity point the wrong way
    on a turning target. The latch position comes from the quadratic
    horizontal fit evaluated at the latch (the linear fit's endpoint is
    biased low by ~a*T^2/6 on a curve).

    DETECT-THEN-REFINE: the linear trend is the DETECTOR (its t-stat is the
    selection test) but its omega under-reads on a strong turn (the fitted
    slope averages the rotating derivative over the window -- measured ~13%
    low at 25 deg/s over 2 s). The ESTIMATOR is therefore an EXACT
    constant-turn-rate profile fit: for a fixed omega the CT model is LINEAR
    in the remaining parameters, so RSS(omega) is minimized by a bounded 1-D
    golden-section search around the detector's estimate, each step one tiny
    least-squares solve. Unbiased on noise-free CT motion (verified in the
    tests to <1e-3 relative). `omega_sigma` stays the detector's -- the
    exact fit's scatter matches it (MC-checked within 25%)."""
    n_pts = len(ts)
    A1 = np.vstack([ts, np.ones_like(ts)]).T            # [t, 1]
    A2 = np.vstack([ts * ts, ts, np.ones_like(ts)]).T   # [t^2, t, 1]

    def _ct_vel_solve(w: float, vel_ne: np.ndarray):
        """Exact CT velocity model v(t) = R(w*t) v_latch -- linear in
        v_latch for fixed w. Returns (v_latch(2,), rss)."""
        c, s = np.cos(w * ts), np.sin(w * ts)
        # rows: [vn_i; ve_i] = [[c,-s],[s,c]] @ vL
        D = np.zeros((2 * n_pts, 2))
        D[0::2, 0], D[0::2, 1] = c, -s
        D[1::2, 0], D[1::2, 1] = s, c
        y = vel_ne.reshape(-1)
        sol_, rss_, _rk, _sv = np.linalg.lstsq(D, y, rcond=None)
        r = y - D @ sol_
        return sol_, float(np.dot(r, r))

    def _ct_pos_solve(w: float, pos_ne: np.ndarray):
        """Exact CT position model p(t) = r0 + A(w,t) v_latch (the arc
        integral) -- linear in (r0, v_latch) for fixed w. Returns
        (r0(2,), v_latch(2,), rss)."""
        if abs(w) < 1e-9:
            a11, a12 = ts, np.zeros_like(ts)
        else:
            a11 = np.sin(w * ts) / w
            a12 = -(1.0 - np.cos(w * ts)) / w
        D = np.zeros((2 * n_pts, 4))
        D[0::2, 0] = 1.0
        D[1::2, 1] = 1.0
        D[0::2, 2], D[0::2, 3] = a11, a12
        D[1::2, 2], D[1::2, 3] = -a12, a11
        y = pos_ne.reshape(-1)
        sol_, rss_, _rk, _sv = np.linalg.lstsq(D, y, rcond=None)
        r = y - D @ sol_
        return sol_[0:2], sol_[2:4], float(np.dot(r, r))

    def _golden_min(f, lo: float, hi: float, iters: int = 40) -> float:
        """Bounded golden-section minimum of f on [lo, hi]."""
        g = (math.sqrt(5.0) - 1.0) / 2.0
        a, b = lo, hi
        x1 = b - g * (b - a)
        x2 = a + g * (b - a)
        f1, f2 = f(x1), f(x2)
        for _ in range(iters):
            if f1 <= f2:
                b, x2, f2 = x2, x1, f1
                x1 = b - g * (b - a)
                f1 = f(x1)
            else:
                a, x1, f1 = x1, x2, f2
                x2 = a + g * (b - a)
                f2 = f(x2)
        return 0.5 * (a + b)

    if reported is not None:
        if n_pts < 4:
            return None
        vel = np.asarray(reported, dtype=np.float64)     # (N, 3) NED
        try:
            solv, _r, rankv, _s = np.linalg.lstsq(A1, vel[:, 0:2], rcond=None)
        except np.linalg.LinAlgError:
            return None
        if rankv < 2 or not np.all(np.isfinite(solv)):
            return None
        a_ne = solv[0]                                   # accel (n, e)
        v0_ne = solv[1]                                  # velocity AT latch
        resid = vel[:, 0:2] - A1 @ solv
        dof = 2 * (n_pts - 2)
        s2 = float(np.sum(resid ** 2)) / max(dof, 1)
        # Var(slope) per axis = s2 / sum((t - tbar)^2)
        tbar = float(np.mean(ts))
        sxx = float(np.sum((ts - tbar) ** 2))
        sigma_a = math.sqrt(max(s2, 1e-12) / max(sxx, 1e-12))
        speed0 = float(np.hypot(v0_ne[0], v0_ne[1]))
        if speed0 < TURN_SPEED_FLOOR_MS:
            return None
        omega = float(v0_ne[0] * a_ne[1] - v0_ne[1] * a_ne[0]) / (speed0 ** 2)
        sigma_omega = sigma_a / speed0
        if (not math.isfinite(omega) or sigma_omega <= 0.0
                or abs(omega) / sigma_omega < TURN_T_STAT_MIN
                or abs(omega) > TURN_OMEGA_MAX_RAD_S):
            return None
        # REFINE (module docstring "detect-then-refine"): exact CT profile
        # fit over a bounded omega bracket around the detector's estimate.
        # Bracket must cover BOTH the noise (4 sigma) and the detector's
        # known multiplicative under-read (up to ~30% at strong turns/long
        # windows) -- in the near-noise-free case sigma alone is far too
        # tight and would pin the search at the bracket edge.
        half = max(4.0 * sigma_omega, 0.5 * abs(omega) + 0.05)
        lo = max(omega - half, -TURN_OMEGA_MAX_RAD_S)
        hi = min(omega + half, TURN_OMEGA_MAX_RAD_S)
        omega = _golden_min(lambda w: _ct_vel_solve(w, vel[:, 0:2])[1], lo, hi)
        vl_ne, _rss = _ct_vel_solve(omega, vel[:, 0:2])
        if abs(omega) > TURN_OMEGA_MAX_RAD_S or not np.all(np.isfinite(vl_ne)):
            return None
        v_latch = (float(vl_ne[0]), float(vl_ne[1]), float(np.mean(vel[:, 2])))
    else:
        if n_pts < 5:
            return None
        try:
            solq, _r, rankq, _s = np.linalg.lstsq(A2, pos[:, 0:2], rcond=None)
        except np.linalg.LinAlgError:
            return None
        if rankq < 3 or not np.all(np.isfinite(solq)):
            return None
        a_ne = 2.0 * solq[0]
        v0_ne = solq[1]
        residq = pos[:, 0:2] - A2 @ solq
        dof = 2 * (n_pts - 3)
        s2 = float(np.sum(residq ** 2)) / max(dof, 1)
        try:
            cov00 = float(np.linalg.inv(A2.T @ A2)[0, 0])
        except np.linalg.LinAlgError:
            return None
        sigma_a = 2.0 * math.sqrt(max(s2, 1e-12) * max(cov00, 0.0))
        speed0 = float(np.hypot(v0_ne[0], v0_ne[1]))
        if speed0 < TURN_SPEED_FLOOR_MS:
            return None
        omega = float(v0_ne[0] * a_ne[1] - v0_ne[1] * a_ne[0]) / (speed0 ** 2)
        sigma_omega = sigma_a / speed0
        if (not math.isfinite(omega) or sigma_omega <= 0.0
                or abs(omega) / sigma_omega < TURN_T_STAT_MIN
                or abs(omega) > TURN_OMEGA_MAX_RAD_S):
            return None
        # REFINE on the exact CT position model (same profile-fit approach).
        half = max(4.0 * sigma_omega, 0.5 * abs(omega) + 0.05)
        lo = max(omega - half, -TURN_OMEGA_MAX_RAD_S)
        hi = min(omega + half, TURN_OMEGA_MAX_RAD_S)
        omega = _golden_min(lambda w: _ct_pos_solve(w, pos[:, 0:2])[2], lo, hi)
        _r0_ne, vl_ne, _rss = _ct_pos_solve(omega, pos[:, 0:2])
        if abs(omega) > TURN_OMEGA_MAX_RAD_S or not np.all(np.isfinite(vl_ne)):
            return None
        v_latch = (float(vl_ne[0]), float(vl_ne[1]), float(np.linalg.lstsq(
            A1, pos[:, 2], rcond=None)[0][0]))

    # CT latch POSITION + residual: exact CT arc horizontal (at the refined
    # omega), linear vertical.
    try:
        r0_ne, vl_pos, _rss = _ct_pos_solve(omega, pos[:, 0:2])
        sold, _r2, rankd, _s2 = np.linalg.lstsq(A1, pos[:, 2:3], rcond=None)
    except np.linalg.LinAlgError:
        return None
    if rankd < 2 or not (np.all(np.isfinite(r0_ne)) and np.all(np.isfinite(sold))):
        return None
    if abs(omega) < 1e-9:
        a11, a12 = ts, np.zeros_like(ts)
    else:
        a11 = np.sin(omega * ts) / omega
        a12 = -(1.0 - np.cos(omega * ts)) / omega
    pred_n = r0_ne[0] + a11 * vl_pos[0] + a12 * vl_pos[1]
    pred_e = r0_ne[1] - a12 * vl_pos[0] + a11 * vl_pos[1]
    pred_d = (A1 @ sold)[:, 0]
    resid3 = np.stack([pos[:, 0] - pred_n, pos[:, 1] - pred_e,
                       pos[:, 2] - pred_d], axis=1)
    residual_rms = float(np.sqrt(np.mean(np.sum(resid3 ** 2, axis=1))))
    r0 = (float(r0_ne[0]), float(r0_ne[1]), float(sold[1][0]))
    return {"omega_rad_s": float(omega), "omega_sigma_rad_s": float(sigma_omega),
            "v_latch_ned": v_latch, "r0_ned": r0,
            "residual_rms_m": residual_rms}


# ---------------------------------------------------------------- persistence


def write_cue_jsonl(records: Iterable[CueRecord], path: str) -> None:
    """Write the JSON-lines cue-record schema (module docstring)."""
    with open(path, "w") as f:
        for r in records:
            row = {"t": r.t, "lat": r.lat, "lon": r.lon, "alt_m_msl": r.alt_m_msl}
            if r.vn is not None and r.ve is not None and r.vd is not None:
                row["vn"] = r.vn
                row["ve"] = r.ve
                row["vd"] = r.vd
            f.write(json.dumps(row) + "\n")


def read_cue_jsonl(path: str) -> List[CueRecord]:
    records: List[CueRecord] = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            records.append(CueRecord(
                t=float(d["t"]), lat=float(d["lat"]), lon=float(d["lon"]),
                alt_m_msl=float(d["alt_m_msl"]),
                vn=(float(d["vn"]) if "vn" in d else None),
                ve=(float(d["ve"]) if "ve" in d else None),
                vd=(float(d["vd"]) if "vd" in d else None)))
    return records


# --------------------------------------------------------- MAVLink adapter


def cue_record_from_global_position_int(msg, t: float) -> CueRecord:
    """Adapter: one MAVLink GLOBAL_POSITION_INT -> one CueRecord.

    `msg` is DUCK-TYPED (any object with `.lat`/`.lon`/`.alt`/`.vx`/`.vy`/
    `.vz` in the standard MAVLink GLOBAL_POSITION_INT units -- lat/lon in
    degE7, alt in mm MSL, vx/vy/vz in cm/s NED). This function has NO
    pymavlink import, so it is fully testable (and IS tested, as the
    producer half of the producer->consumer contract test) without the
    optional dependency installed. `t` is the caller's own clock (recommended:
    receive-time, not the FC's onboard timestamp -- see module docstring)."""
    return CueRecord(
        t=float(t),
        lat=msg.lat / 1e7,
        lon=msg.lon / 1e7,
        alt_m_msl=msg.alt / 1000.0,
        vn=msg.vx / 100.0,
        ve=msg.vy / 100.0,
        vd=msg.vz / 100.0,
    )


def mavlink_stream_to_cue_records(connection_str: str, duration_s: float = 2.5,
                                   msg_type: str = "GLOBAL_POSITION_INT"
                                   ) -> List[CueRecord]:
    """Read `duration_s` seconds of `msg_type` off a live MAVLink connection
    and return CueRecords timestamped at RECEIVE time. The only function in
    this module that needs `pymavlink` -- imported here, guarded, so the rest
    of the module (and every other test) never needs it installed."""
    try:
        from pymavlink import mavutil  # noqa: PLC0415 -- deliberately local
    except ImportError as e:
        raise RuntimeError(
            "pymavlink is not installed (pip install pymavlink). It is "
            "needed only by mavlink_stream_to_cue_records -- the schema, "
            "fitter, quality gate, and CLI-arg loader in this module all "
            "work without it.") from e
    conn = mavutil.mavlink_connection(connection_str)
    records: List[CueRecord] = []
    t_end = time.monotonic() + duration_s
    while time.monotonic() < t_end:
        msg = conn.recv_match(type=msg_type, blocking=True, timeout=0.5)
        if msg is None:
            continue
        records.append(cue_record_from_global_position_int(msg, time.monotonic()))
    return records


# --------------------------------------------------------- integration loader
#
# Turns a CueSolution into the values flight.deploy.real_flight already
# knows how to consume, WITHOUT editing that (currently being edited by
# another agent) module. See docs/cue_relay_plan.md for the proposed
# `--cue-json` CLI hookup diff.


def cue_to_target_start_arg(sol: CueSolution) -> str:
    """-> the `--target-start EAST,NORTH` string real_flight.py's
    `build_config`/`build_terminal` parse (`args.target_start.split(',')`)."""
    n, e, _d = sol.belief_r0_ned
    return f"{e:.3f},{n:.3f}"


def cue_to_target_vel_arg(sol: CueSolution) -> str:
    """-> the `--target-vel EAST,NORTH` string, same convention."""
    vn, ve, _vd = sol.belief_vel0_ned
    return f"{ve:.3f},{vn:.3f}"


def load_cue_for_real_flight(cue_json_path: str, own_lat: float, own_lon: float,
                              own_alt_m_msl: float, *,
                              now_t: Optional[float] = None,
                              trigger_t: Optional[float] = None,
                              **gate_kwargs) -> Tuple[str, str, CueSolution]:
    """Read + fit + (optionally) extrapolate a cue file into
    (`target_start_arg`, `target_vel_arg`, `CueSolution`) -- everything the
    proposed `--cue-json` hookup (docs/cue_relay_plan.md) needs to override
    `args.target_start`/`args.target_vel` before `build_config`/
    `build_terminal` run. Raises `CueQualityError` (fail closed) rather than
    emit a bad cue; the caller must treat that as a hard abort to the
    manual-entry fallback (`--target-start`/`--target-vel` typed by the
    operator), never a silent fly-on-anyway."""
    records = read_cue_jsonl(cue_json_path)
    sol = fit_cue(records, own_lat, own_lon, own_alt_m_msl, now_t=now_t,
                  **gate_kwargs)
    if trigger_t is not None:
        sol = sol.extrapolate_to(trigger_t)
    return cue_to_target_start_arg(sol), cue_to_target_vel_arg(sol), sol
