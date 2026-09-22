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

    def extrapolate_to(self, trigger_t: float) -> "CueSolution":
        """The ONE legal post-latch operation (docs/cue_relay_plan.md "latch
        semantics"): advance the belief position by the already-latched
        CONSTANT velocity to the actual trigger instant. Pure arithmetic on
        numbers already frozen at fit time -- does not read any sensor, does
        not change `belief_vel0_ned` (a straight AUTO leg is assumed constant-
        velocity over the short latch-to-trigger gap; see the plan doc for
        what breaks that assumption)."""
        dt = float(trigger_t) - self.latch_t
        vn, ve, vd = self.belief_vel0_ned
        n0, e0, d0 = self.belief_r0_ned
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
    fit, or a residual that says the points don't lie near a straight
    constant-velocity line.

    `belief_r0_ned` is the fitted position AT THE LAST RECORD's time (the
    "latch" instant) -- an average over all `n_points` samples via the
    regression, not just the raw last fix, so its noise is already reduced
    relative to a single GPS sample (see docs/cue_relay_plan.md error
    budget)."""
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

    pred = A @ sol
    residual_rms = float(np.sqrt(np.mean(np.sum((pos - pred) ** 2, axis=1))))
    if not math.isfinite(residual_rms) or residual_rms > max_residual_m:
        raise CueQualityError(
            f"position-fit residual {residual_rms:.2f} m exceeds "
            f"{max_residual_m:.2f} m -- the points don't lie near a "
            f"straight constant-velocity line (bad fix, multipath, or a "
            f"maneuvering target); refusing to emit a cue")

    reported = [(r.vn, r.ve, r.vd) for r in recs
                if r.vn is not None and r.ve is not None and r.vd is not None]
    if len(reported) == len(recs):
        vn = float(np.mean([v[0] for v in reported]))
        ve = float(np.mean([v[1] for v in reported]))
        vd = float(np.mean([v[2] for v in reported]))
        vel_source = "reported"
    else:
        vn, ve, vd = float(v_fit[0]), float(v_fit[1]), float(v_fit[2])
        vel_source = "fit"

    speed = math.hypot(vn, ve)
    heading = math.degrees(math.atan2(ve, vn)) % 360.0

    return CueSolution(
        latch_t=t_latch,
        belief_r0_ned=(float(p0_fit[0]), float(p0_fit[1]), float(p0_fit[2])),
        belief_vel0_ned=(vn, ve, vd),
        speed_ms=speed, heading_deg=heading,
        residual_rms_m=residual_rms, n_points=len(recs), span_s=span,
        vel_source=vel_source, origin_lla=(own_lat, own_lon, own_alt_m_msl))


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
