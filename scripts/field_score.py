#!/usr/bin/env python3
"""field_score.py -- binary-kill metrology for a REAL flight test (P5 scoring tool).

Scope (honesty boundary, CLAUDE.md / docs/hardware_order_list.md Sec.0c): this
tool scores an ACTUAL flight test from the two aircrafts' flight-controller
logs. That is legitimate ground truth for grading a real intercept -- it is
NOT the sim's gt_* guidance-cheat (which is about what the FLYING software is
allowed to see mid-flight). Here both aircraft have already landed; we are
just measuring how close they got, after the fact, from recorded telemetry.

The builder re-scoped the real-hardware success criterion to a BINARY KILL
("did it take the drone out, yes/no") confirmed primarily by VIDEO (seeker
recording + phone slow-mo) -- see docs/hardware_order_list.md Sec.0c. This
script is the supporting KINEMATIC half of that evidence: it computes the
inter-aircraft range history and the CPA (closest point of approach) from
both flight logs, and classifies KILL/MISS against a lethal-radius threshold.
It does not replace the video call (a low kinematic CPA with airframes that
visibly missed on video is still a miss); it quantifies the geometry so the
video verdict has a number behind it.

INPUTS
------
Primary path -- two PX4 ULogs (one per aircraft):
    --ulog-a INTERCEPTOR.ulg --ulog-b TARGET.ulg
Parsed with pyulog (deps_needed: pyulog -- already present in .venv here).
Position is read from `vehicle_global_position` (preferred: EKF-fused lat/lon/
alt, already in degrees/metres) or `vehicle_gps_position` (raw GPS, int32
1e-7 deg / mm -- auto-scaled). Two INDEPENDENT flight controllers do not share
a boot-time clock, so naive boot-relative timestamps from the two logs are NOT
directly comparable. This script derives a boot-clock -> UTC offset per log
from `vehicle_gps_position.time_utc_usec` (when GPS had a time fix) and shifts
that log's samples onto a common UTC axis before comparing the two aircraft.
If no GPS UTC fix is present in a log, it falls back to boot-relative time and
flags the run as `utc_synced: false` in the report (the two aircraft's boot
clocks are then assumed already aligned, e.g. a common trigger event -- treat
that report's CPA time as approximate).

Fallback path -- pre-exported CSV (e.g. from `ulog2csv`, hand-built, or a
non-PX4 log converted externally):
    --csv-a INTERCEPTOR.csv --csv-b TARGET.csv
Columns: a time column (`t_utc_s` preferred, `t_s` accepted with a
same-caveat warning) plus EITHER `lat_deg,lon_deg,alt_m` OR
`east_m,north_m[,up_m]` (already in one shared local frame -- your
responsibility to guarantee that if you hand-build this).

OUTPUT
------
A JSON verdict + a 2-panel PNG (range-vs-time with the lethal-radius line and
CPA marked; top-down East-North trajectory with CPA marked) written to
--out-dir (default logs/field_score/).

SELF-TEST
---------
`--self-test` runs entirely offline on two synthetic crossing trajectories
with an analytically-known CPA (closed-form straight-line minimum-distance,
computed independently of the interpolation/argmin code path under test) and
asserts the tool's computed CPA distance and time match to within a tight
tolerance. No PX4/Gazebo, no real log files, no network. Run:
    .venv/bin/python scripts/field_score.py --self-test
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

EARTH_RADIUS_M = 6371000.0  # mean Earth radius; flat-earth (equirectangular)
                             # approx is accurate to << 1 cm over the ~100 m-
                             # scale engagement ranges this project cares
                             # about (error grows with range^3/R^2, negligible
                             # here). Documented approximation, not a bug.

# Mechanism-anchored lethal radii referenced in docs/decisions.md (ADR-0025)
# and docs/hardware_order_list.md Sec.0c: ~0.5 m for a kinetic ram, ~1.5 m for
# a net/frag-style kill radius. Default here is the more conservative (larger)
# "net" number; override with --lethal-radius for a ram-only claim.
DEFAULT_LETHAL_RADIUS_M = 1.5


# --------------------------------------------------------------------------
# Geodesy: local tangent-plane (East-North-Up) <-> lat/lon/alt
# --------------------------------------------------------------------------

def latlon_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """Equirectangular projection onto a local ENU plane tangent at
    (lat0, lon0, alt0). Exact inverse of enu_to_latlon() for the same
    reference point (used by the self-test to build synthetic GPS tracks)."""
    lat = np.asarray(lat, dtype=np.float64)
    lon = np.asarray(lon, dtype=np.float64)
    alt = np.asarray(alt, dtype=np.float64)
    lat0_r = np.radians(lat0)
    north = np.radians(lat - lat0) * EARTH_RADIUS_M
    east = np.radians(lon - lon0) * EARTH_RADIUS_M * np.cos(lat0_r)
    up = alt - alt0
    return east, north, up


def enu_to_latlon(east, north, up, lat0, lon0, alt0):
    """Inverse of latlon_to_enu() -- used only by the self-test to build
    synthetic lat/lon/alt tracks from a known ENU ground truth."""
    east = np.asarray(east, dtype=np.float64)
    north = np.asarray(north, dtype=np.float64)
    up = np.asarray(up, dtype=np.float64)
    lat0_r = np.radians(lat0)
    lat = lat0 + np.degrees(north / EARTH_RADIUS_M)
    lon = lon0 + np.degrees(east / (EARTH_RADIUS_M * np.cos(lat0_r)))
    alt = alt0 + up
    return lat, lon, alt


# --------------------------------------------------------------------------
# Track loading
# --------------------------------------------------------------------------

@dataclass
class Track:
    label: str
    t_utc_s: np.ndarray                 # (N,) monotonic non-decreasing
    latlon: Optional[np.ndarray] = None  # (N,3) lat_deg, lon_deg, alt_m
    enu: Optional[np.ndarray] = None     # (N,3) east_m, north_m, up_m
    utc_synced: bool = True
    source: str = ""
    warnings: list = field(default_factory=list)

    @property
    def mode(self) -> str:
        return "enu" if self.enu is not None else "latlon"


def _sorted_by_time(t, *arrays):
    order = np.argsort(t)
    return (t[order],) + tuple(a[order] for a in arrays)


def load_track_from_ulog(path: Path, label: str) -> Track:
    from pyulog import ULog  # deps_needed: pyulog (present in .venv)

    ulog = ULog(str(path))

    def get_dataset(name):
        try:
            return ulog.get_dataset(name)
        except (KeyError, IndexError):
            return None

    gps = get_dataset("vehicle_gps_position")
    gpos = get_dataset("vehicle_global_position")
    if gps is None and gpos is None:
        raise ValueError(
            f"{path}: neither 'vehicle_global_position' nor "
            f"'vehicle_gps_position' found in this ULog -- cannot recover "
            f"a global position track. Use --csv-a/--csv-b instead."
        )

    warnings = []
    utc_offset_us = None
    if gps is not None:
        d = gps.data
        t_boot = np.asarray(d["timestamp"], dtype=np.float64)
        utc = np.asarray(
            d.get("time_utc_usec", np.zeros_like(t_boot)), dtype=np.float64
        )
        # A sane UTC-microsecond epoch is >> 1e15 us (year 2001+); PX4 logs
        # this field as 0 until GPS achieves a time fix.
        valid = utc > 1e15
        if valid.any():
            utc_offset_us = float(np.median(utc[valid] - t_boot[valid]))

    if gpos is not None:
        d = gpos.data
        t_boot = np.asarray(d["timestamp"], dtype=np.float64)
        lat = np.asarray(d["lat"], dtype=np.float64)   # already degrees
        lon = np.asarray(d["lon"], dtype=np.float64)   # already degrees
        alt = np.asarray(d["alt"], dtype=np.float64)   # already metres AMSL
        source = "vehicle_global_position"
    else:
        d = gps.data

        def _pick(*names):
            # PX4 v1.17 renamed vehicle_gps_position -> SensorGps fields
            # (lat/lon/alt -> latitude_deg/longitude_deg/altitude_msl_m); accept
            # both generations so a modern log doesn't KeyError-crash the fallback.
            for n in names:
                if n in d:
                    return np.asarray(d[n], dtype=np.float64)
            raise KeyError(f"raw-GPS topic has none of {names} "
                           f"(fields present: {sorted(d)[:10]})")

        t_boot = np.asarray(d["timestamp"], dtype=np.float64)
        lat_raw = _pick("lat", "latitude_deg")
        lon_raw = _pick("lon", "longitude_deg")
        alt_raw = _pick("alt", "altitude_msl_m")
        # vehicle_gps_position stores lat/lon as int32 1e-7 deg and alt as
        # int32 mm. Scale defensively off magnitude rather than assuming a
        # fixed PX4 message version (no real ULog available on this dev
        # machine to hard-validate against -- flagged in the honesty notes).
        lat = lat_raw / 1e7 if np.abs(lat_raw).max() > 1000 else lat_raw
        lon = lon_raw / 1e7 if np.abs(lon_raw).max() > 1000 else lon_raw
        alt = alt_raw / 1e3 if np.abs(alt_raw).max() > 10000 else alt_raw
        source = "vehicle_gps_position"
        warnings.append(
            "position source is raw vehicle_gps_position (no "
            "vehicle_global_position in this log) -- unfiltered GPS, "
            "noisier than the EKF-fused global-position estimate."
        )

    if utc_offset_us is None:
        warnings.append(
            "no GPS UTC time fix found in this log -- falling back to "
            "boot-relative timestamps. Cross-aircraft alignment then "
            "assumes the two flight controllers' boot clocks are already "
            "synchronized (NOT generally true for two independent FCs); "
            "treat the CPA time as approximate."
        )
        t_utc_s = t_boot / 1e6
        utc_synced = False
    else:
        t_utc_s = (t_boot + utc_offset_us) / 1e6
        utc_synced = True

    t_utc_s, lat, lon, alt = _sorted_by_time(t_utc_s, lat, lon, alt)
    return Track(
        label=label,
        t_utc_s=t_utc_s,
        latlon=np.column_stack([lat, lon, alt]),
        utc_synced=utc_synced,
        source=source,
        warnings=warnings,
    )


def load_track_from_csv(path: Path, label: str) -> Track:
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"{path}: empty CSV")
    cols = set(rows[0].keys())

    if "t_utc_s" in cols:
        tcol, utc_synced = "t_utc_s", True
    elif "t_s" in cols:
        tcol, utc_synced = "t_s", False
    else:
        raise ValueError(f"{path}: CSV needs a 't_utc_s' or 't_s' time column")
    t = np.array([float(r[tcol]) for r in rows], dtype=np.float64)

    warnings = []
    if not utc_synced:
        warnings.append(
            "CSV used 't_s' (not 't_utc_s') -- alignment assumes this "
            "track's clock is already synchronized with the other track's."
        )

    if {"lat_deg", "lon_deg", "alt_m"} <= cols:
        lat = np.array([float(r["lat_deg"]) for r in rows])
        lon = np.array([float(r["lon_deg"]) for r in rows])
        alt = np.array([float(r["alt_m"]) for r in rows])
        t, lat, lon, alt = _sorted_by_time(t, lat, lon, alt)
        return Track(label=label, t_utc_s=t, latlon=np.column_stack([lat, lon, alt]),
                      utc_synced=utc_synced, source="csv:latlon", warnings=warnings)
    elif {"east_m", "north_m"} <= cols:
        e = np.array([float(r["east_m"]) for r in rows])
        n = np.array([float(r["north_m"]) for r in rows])
        u = np.array([float(r.get("up_m", 0.0) or 0.0) for r in rows])
        t, e, n, u = _sorted_by_time(t, e, n, u)
        return Track(label=label, t_utc_s=t, enu=np.column_stack([e, n, u]),
                      utc_synced=utc_synced, source="csv:enu", warnings=warnings)
    else:
        raise ValueError(
            f"{path}: CSV needs either lat_deg,lon_deg,alt_m columns or "
            f"east_m,north_m[,up_m] columns"
        )


# --------------------------------------------------------------------------
# Core scoring: common-frame projection, time alignment, CPA
# --------------------------------------------------------------------------

def project_to_common_enu(track_a: Track, track_b: Track):
    """Return (posA_enu, posB_enu) in one shared local tangent plane."""
    if track_a.mode != track_b.mode:
        raise ValueError(
            "cannot mix an ENU-frame track with a lat/lon track -- provide "
            "both tracks in the same frame (both ULog/lat-lon CSV, or both "
            "already-local ENU CSV in a frame you guarantee is shared)"
        )
    if track_a.mode == "enu":
        return track_a.enu, track_b.enu
    lat0, lon0, alt0 = track_a.latlon[0]
    east_a, north_a, up_a = latlon_to_enu(*track_a.latlon.T, lat0, lon0, alt0)
    east_b, north_b, up_b = latlon_to_enu(*track_b.latlon.T, lat0, lon0, alt0)
    return (np.column_stack([east_a, north_a, up_a]),
            np.column_stack([east_b, north_b, up_b]))


def compute_range_track(t_a, pos_a, t_b, pos_b, dt: Optional[float] = None):
    """Resample both tracks onto a shared UTC time grid and return the
    range history. Raises ValueError if the two logs have no overlapping
    time window (can't score a CPA without simultaneous coverage)."""
    lo, hi = max(t_a.min(), t_b.min()), min(t_a.max(), t_b.max())
    if hi <= lo:
        raise ValueError(
            f"no overlapping time window between the two tracks "
            f"(A spans [{t_a.min():.3f},{t_a.max():.3f}], "
            f"B spans [{t_b.min():.3f},{t_b.max():.3f}])"
        )
    if dt is None:
        dt_a = float(np.median(np.diff(t_a))) if len(t_a) > 1 else 0.05
        dt_b = float(np.median(np.diff(t_b))) if len(t_b) > 1 else 0.05
        dt = min(max(min(dt_a, dt_b), 0.01), 0.2)
    t_grid = np.arange(lo, hi, dt)
    if len(t_grid) < 2:
        raise ValueError("overlapping window too short to resample")
    pos_a_i = np.column_stack(
        [np.interp(t_grid, t_a, pos_a[:, k]) for k in range(3)]
    )
    pos_b_i = np.column_stack(
        [np.interp(t_grid, t_b, pos_b[:, k]) for k in range(3)]
    )
    rng = np.linalg.norm(pos_a_i - pos_b_i, axis=1)
    return t_grid, pos_a_i, pos_b_i, rng, dt


@dataclass
class ScoreResult:
    t_grid: np.ndarray
    pos_a: np.ndarray
    pos_b: np.ndarray
    range_m: np.ndarray
    dt_s: float
    cpa_idx: int
    cpa_m: float
    cpa_t_utc_s: float
    lethal_radius_m: float
    verdict: str
    overlap_s: float
    track_a: Track
    track_b: Track


def score_engagement(track_a: Track, track_b: Track, lethal_radius_m: float,
                      dt: Optional[float] = None) -> ScoreResult:
    pos_a_full, pos_b_full = project_to_common_enu(track_a, track_b)
    t_grid, pos_a, pos_b, rng, used_dt = compute_range_track(
        track_a.t_utc_s, pos_a_full, track_b.t_utc_s, pos_b_full, dt=dt
    )
    cpa_idx = int(np.argmin(rng))
    cpa_m = float(rng[cpa_idx])
    cpa_t = float(t_grid[cpa_idx])
    verdict = "KILL" if cpa_m <= lethal_radius_m else "MISS"
    return ScoreResult(
        t_grid=t_grid, pos_a=pos_a, pos_b=pos_b, range_m=rng, dt_s=used_dt,
        cpa_idx=cpa_idx, cpa_m=cpa_m, cpa_t_utc_s=cpa_t,
        lethal_radius_m=lethal_radius_m, verdict=verdict,
        overlap_s=float(t_grid[-1] - t_grid[0]),
        track_a=track_a, track_b=track_b,
    )


# --------------------------------------------------------------------------
# Reporting: JSON + plot
# --------------------------------------------------------------------------

def write_report(result: ScoreResult, out_dir: Path, video_a: Optional[str],
                  video_b: Optional[str], tag: str) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    report = {
        "tag": tag,
        "verdict": result.verdict,
        "cpa_m": round(result.cpa_m, 3),
        "cpa_t_utc_s": result.cpa_t_utc_s,
        "lethal_radius_m": result.lethal_radius_m,
        "time_overlap_s": round(result.overlap_s, 3),
        "resample_dt_s": result.dt_s,
        "track_a": {
            "label": result.track_a.label, "source": result.track_a.source,
            "utc_synced": result.track_a.utc_synced,
            "n_samples": int(len(result.track_a.t_utc_s)),
            "warnings": result.track_a.warnings,
        },
        "track_b": {
            "label": result.track_b.label, "source": result.track_b.source,
            "utc_synced": result.track_b.utc_synced,
            "n_samples": int(len(result.track_b.t_utc_s)),
            "warnings": result.track_b.warnings,
        },
        "video_evidence": {
            "video_a": video_a, "video_a_exists": bool(video_a and Path(video_a).exists()),
            "video_b": video_b, "video_b_exists": bool(video_b and Path(video_b).exists()),
        },
        "honesty_note": (
            "This is REAL-FLIGHT scoring from recorded logs after the fact -- "
            "not the sim's gt_* guidance-cheat boundary (that boundary is "
            "about what the flying software may read mid-flight; it does not "
            "apply to post-hoc scoring of a completed real test). Per "
            "docs/hardware_order_list.md Sec.0c the primary kill evidence is "
            "VIDEO (seeker + phone slow-mo); this CPA is the supporting "
            "kinematic number, not a substitute for the video call."
        ),
        "accuracy_caveat": (
            "cpa_m differences TWO INDEPENDENT GPS/EKF absolute positions; the "
            "inter-receiver bias is ~1-3 m horizontal (worse vertical) -- "
            "comparable to or larger than the 0.5-1.5 m kill radius. So near the "
            "threshold the KILL/MISS call is UNCERTAIN; trust the video, not "
            "cpa_m's 3rd decimal."
        ),
        "verdict_uncertain": bool(abs(result.cpa_m - result.lethal_radius_m) < 2.0),
    }
    if not (result.track_a.utc_synced and result.track_b.utc_synced):
        report["honesty_note"] += (
            " WARNING: at least one track had no GPS UTC fix, so the two "
            "logs' clocks were NOT independently cross-synchronized -- "
            "treat cpa_t_utc_s as approximate."
        )

    json_path = out_dir / f"field_score_{tag}.json"
    json_path.write_text(json.dumps(report, indent=2))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax_range, ax_traj) = plt.subplots(2, 1, figsize=(9, 10))

    t_rel = result.t_grid - result.t_grid[0]
    ax_range.plot(t_rel, result.range_m, lw=1.5, label="inter-aircraft range")
    ax_range.axhline(result.lethal_radius_m, color="crimson", ls="--", lw=1,
                      label=f"lethal radius = {result.lethal_radius_m:.2f} m")
    ax_range.axvline(t_rel[result.cpa_idx], color="grey", ls=":", lw=1)
    ax_range.plot(t_rel[result.cpa_idx], result.cpa_m, "o", color="crimson", ms=8,
                  label=f"CPA = {result.cpa_m:.2f} m  [{result.verdict}]")
    ax_range.set_xlabel("time since window start (s)")
    ax_range.set_ylabel("range (m)")
    ax_range.set_title(f"field_score: {result.track_a.label} vs {result.track_b.label}")
    ax_range.legend(loc="best", fontsize=8)
    ax_range.grid(alpha=0.3)

    ax_traj.plot(result.pos_a[:, 0], result.pos_a[:, 1], lw=1.5,
                 label=result.track_a.label, color="tab:blue")
    ax_traj.plot(result.pos_b[:, 0], result.pos_b[:, 1], lw=1.5,
                 label=result.track_b.label, color="tab:orange")
    ax_traj.plot(result.pos_a[result.cpa_idx, 0], result.pos_a[result.cpa_idx, 1],
                 "o", color="tab:blue", ms=9, mec="k")
    ax_traj.plot(result.pos_b[result.cpa_idx, 0], result.pos_b[result.cpa_idx, 1],
                 "o", color="tab:orange", ms=9, mec="k")
    ax_traj.plot([result.pos_a[result.cpa_idx, 0], result.pos_b[result.cpa_idx, 0]],
                 [result.pos_a[result.cpa_idx, 1], result.pos_b[result.cpa_idx, 1]],
                 "k--", lw=1, label=f"CPA link ({result.cpa_m:.2f} m)")
    ax_traj.set_xlabel("east (m)")
    ax_traj.set_ylabel("north (m)")
    ax_traj.set_title("top-down trajectories (local ENU)")
    ax_traj.set_aspect("equal", adjustable="datalim")
    ax_traj.legend(loc="best", fontsize=8)
    ax_traj.grid(alpha=0.3)

    fig.tight_layout()
    png_path = out_dir / f"field_score_{tag}.png"
    fig.savefig(png_path, dpi=130)
    plt.close(fig)

    report["json_path"] = str(json_path)
    report["png_path"] = str(png_path)
    return report


# --------------------------------------------------------------------------
# Self-test: synthetic crossing trajectories, analytic CPA ground truth
# --------------------------------------------------------------------------

def _analytic_straight_line_cpa(p0_rel, v_rel, t_lo, t_hi):
    """Closed-form closest-approach of two constant-velocity straight lines,
    computed independently of score_engagement()'s grid-interpolation path.
    Returns (t_star, distance_at_t_star), t_star clamped to [t_lo, t_hi]."""
    p0_rel = np.asarray(p0_rel, dtype=np.float64)
    v_rel = np.asarray(v_rel, dtype=np.float64)
    denom = float(np.dot(v_rel, v_rel))
    t_star = 0.0 if denom == 0.0 else -float(np.dot(p0_rel, v_rel)) / denom
    t_star = min(max(t_star, t_lo), t_hi)
    p_star = p0_rel + v_rel * t_star
    return t_star, float(np.linalg.norm(p_star))


def _synthetic_track(label, pos0, vel, duration, hz, t0_utc, lat0, lon0, alt0):
    n = max(int(round(duration * hz)) + 1, 2)
    t = t0_utc + np.linspace(0.0, duration, n)
    t_rel = np.linspace(0.0, duration, n)
    pos = pos0[None, :] + t_rel[:, None] * vel[None, :]
    lat, lon, alt = enu_to_latlon(pos[:, 0], pos[:, 1], pos[:, 2], lat0, lon0, alt0)
    return Track(label=label, t_utc_s=t, latlon=np.column_stack([lat, lon, alt]),
                 utc_synced=True, source="synthetic")


def self_test() -> bool:
    ok = True
    lat0, lon0, alt0 = 34.05, -118.25, 100.0  # arbitrary reference point
    base_utc = 1_752_700_000.0  # arbitrary common UTC epoch shared by both a/c

    def run_case(name, posA0, vA, posB0, vB, duration, hzA, hzB, radius, tol_m, tol_t):
        nonlocal ok
        t_star, cpa_true = _analytic_straight_line_cpa(
            np.array(posA0) - np.array(posB0), np.array(vA) - np.array(vB), 0.0, duration
        )
        track_a = _synthetic_track("interceptor", np.array(posA0, dtype=np.float64),
                                    np.array(vA, dtype=np.float64), duration, hzA,
                                    base_utc, lat0, lon0, alt0)
        track_b = _synthetic_track("target", np.array(posB0, dtype=np.float64),
                                    np.array(vB, dtype=np.float64), duration, hzB,
                                    base_utc, lat0, lon0, alt0)
        result = score_engagement(track_a, track_b, lethal_radius_m=radius, dt=0.01)
        err_m = abs(result.cpa_m - cpa_true)
        err_t = abs(result.cpa_t_utc_s - (base_utc + t_star))
        expect_verdict = "KILL" if cpa_true <= radius else "MISS"
        pass_ = err_m <= tol_m and err_t <= tol_t and result.verdict == expect_verdict
        print(f"[self-test] {name}: cpa_true={cpa_true:.4f} m  cpa_got={result.cpa_m:.4f} m  "
              f"err={err_m:.4f} m (tol {tol_m})  t_err={err_t:.4f}s (tol {tol_t})  "
              f"verdict={result.verdict} (expect {expect_verdict})  "
              f"{'PASS' if pass_ else 'FAIL'}")
        ok = ok and pass_
        return pass_

    # Case 1: wide miss (~16.4 m), different sample rates (20 Hz / 5 Hz).
    run_case(
        "case1_miss", posA0=(0, 0, 50), vA=(12, 3, -1),
        posB0=(100, -20, 45), vB=(-4, 9, -0.3),
        duration=10.0, hzA=20.0, hzB=5.0,
        radius=3.0, tol_m=0.05, tol_t=0.05,
    )

    # Case 2: exact-by-construction 2.0 m CPA at t=5.0s, target stationary,
    # differing sample rates (25 Hz / 8 Hz) -- exercises the interpolation
    # path with a hand-verifiable closed form (t_star = 185/37 = 5.0 exactly,
    # p_star=(0,0,2), cpa=2.0 m).
    run_case(
        "case2_kill_exact", posA0=(-30, 5, 2), vA=(6, -1, 0),
        posB0=(0, 0, 0), vB=(0, 0, 0),
        duration=10.0, hzA=25.0, hzB=8.0,
        radius=3.0, tol_m=0.02, tol_t=0.02,
    )

    # Case 3: no time overlap -> must raise, not silently produce a number.
    try:
        track_a = _synthetic_track("a", np.zeros(3), np.array([1.0, 0, 0]),
                                    5.0, 10.0, base_utc, lat0, lon0, alt0)
        track_b = _synthetic_track("b", np.zeros(3), np.array([1.0, 0, 0]),
                                    5.0, 10.0, base_utc + 100.0, lat0, lon0, alt0)
        score_engagement(track_a, track_b, lethal_radius_m=1.0)
        print("[self-test] case3_no_overlap: FAIL (expected ValueError, got a result)")
        ok = False
    except ValueError as e:
        print(f"[self-test] case3_no_overlap: PASS (raised ValueError: {e})")

    print(f"[self-test] {'PASS' if ok else 'FAIL'}")
    return ok


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--ulog-a", type=Path, help="interceptor PX4 ULog (.ulg)")
    ap.add_argument("--ulog-b", type=Path, help="target PX4 ULog (.ulg)")
    ap.add_argument("--csv-a", type=Path, help="interceptor track CSV (fallback)")
    ap.add_argument("--csv-b", type=Path, help="target track CSV (fallback)")
    ap.add_argument("--label-a", default="interceptor")
    ap.add_argument("--label-b", default="target")
    ap.add_argument("--lethal-radius", type=float, default=DEFAULT_LETHAL_RADIUS_M,
                     help=f"kill classification threshold, metres "
                          f"(default {DEFAULT_LETHAL_RADIUS_M} m, the ADR-0025 "
                          f"'net' radius; use ~0.5 m for a ram-only claim)")
    ap.add_argument("--dt", type=float, default=None,
                     help="resample grid step, seconds (default: auto from "
                          "the coarser log's sample rate)")
    ap.add_argument("--video-a", default=None, help="path to interceptor seeker/onboard video (metadata only)")
    ap.add_argument("--video-b", default=None, help="path to chase/phone slow-mo video (metadata only)")
    ap.add_argument("--out-dir", type=Path, default=Path("logs/field_score"))
    ap.add_argument("--tag", default=None, help="report filename tag (default: UTC timestamp)")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        return 0 if self_test() else 1

    have_ulog = args.ulog_a and args.ulog_b
    have_csv = args.csv_a and args.csv_b
    if not (have_ulog or have_csv):
        ap.error("provide --ulog-a/--ulog-b, or --csv-a/--csv-b, or --self-test")
    if have_ulog and have_csv:
        ap.error("provide either the ULog pair or the CSV pair, not both")

    try:
        if have_ulog:
            track_a = load_track_from_ulog(args.ulog_a, args.label_a)
            track_b = load_track_from_ulog(args.ulog_b, args.label_b)
        else:
            track_a = load_track_from_csv(args.csv_a, args.label_a)
            track_b = load_track_from_csv(args.csv_b, args.label_b)

        for t in (track_a, track_b):
            for w in t.warnings:
                print(f"[warn] {t.label}: {w}", file=sys.stderr)

        result = score_engagement(track_a, track_b, args.lethal_radius, dt=args.dt)
    except (ValueError, ImportError, OSError) as e:
        print(f"[field_score] ERROR: {e}", file=sys.stderr)
        return 1

    import datetime
    tag = args.tag or datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    report = write_report(result, args.out_dir, args.video_a, args.video_b, tag)

    print(f"verdict          : {result.verdict}")
    print(f"CPA              : {result.cpa_m:.3f} m  @ t_utc={result.cpa_t_utc_s:.3f}")
    print(f"lethal radius    : {result.lethal_radius_m:.3f} m")
    print(f"time overlap     : {result.overlap_s:.3f} s  (resample dt={result.dt_s:.3f} s)")
    print(f"report           : {report['json_path']}")
    print(f"plot             : {report['png_path']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
