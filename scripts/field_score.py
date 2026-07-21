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
Each aircraft's track is supplied INDEPENDENTLY -- one source flag per side --
so a mixed engagement (PX4-ULog interceptor vs ArduPilot-.BIN target, the
actual Tier-1 hardware pairing, constraint `target-is-ardupilot`) is a
first-class case, not a workaround. The three per-side sources:

PX4 ULog (`--ulog-a` / `--ulog-b`) -- parsed with pyulog (deps_needed: pyulog,
already present in .venv). Position is read from `vehicle_global_position`
(preferred: EKF-fused lat/lon/alt, already degrees/metres) or
`vehicle_gps_position` (raw GPS, int32 1e-7 deg / mm -- auto-scaled).

ArduPilot DataFlash .BIN (`--bin-a` / `--bin-b`) -- the Tier-1 target/practice
quad is an ArduPilot build (Kakute H7 / Matek H743, BOM Sec.E) that logs a
DataFlash .BIN to onboard SD, NOT a PX4 ULog. Parsed with pymavlink's DFReader
(deps_needed: pymavlink -- `pip install pymavlink` into the .venv). Position is
read from the `POS` message (PREFERRED: the EKF-fused vehicle position estimate,
ArduPilot's analog of PX4 `vehicle_global_position` -- lower noise, and the FC's
own best position) and falls back to the raw `GPS` message (receiver fix, analog
of `vehicle_gps_position`) if no POS is logged. DataFlash `Lat`/`Lng` are int32
1e-7 deg and DFReader applies that scaling for us; `Alt` is metres.

Cross-aircraft time base: two INDEPENDENT flight controllers do not share a
boot-time clock, so naive boot-relative timestamps from the two logs are NOT
directly comparable. Both native paths recover a common UTC axis when GPS had a
time fix: pyulog from `vehicle_gps_position.time_utc_usec`; DFReader from the
DataFlash `GPS` message's GPS-week/ms fields (`GWk`/`GMS`) -- the modern-usec
clock then stamps every message as seconds-since-1970. If a log has no GPS UTC
fix its samples come out boot-relative; the tool detects that (timestamps < the
year-2001 epoch), falls back to boot-relative time, and flags the run as
`utc_synced: false` (the two aircraft's boot clocks are then assumed already
aligned, e.g. a common trigger event -- treat that report's CPA time as
approximate).

Fallback path -- pre-exported CSV (`--csv-a` / `--csv-b`; e.g. from `ulog2csv`,
a Mission Planner "Convert .bin to .csv" / DFReader dump for the ArduPilot side
when pymavlink is NOT installed, hand-built, or any log converted externally):
Columns: a time column (`t_utc_s` preferred, `t_s` accepted with a
same-caveat warning) plus EITHER `lat_deg,lon_deg,alt_m` OR
`east_m,north_m[,up_m]` (already in one shared local frame -- your
responsibility to guarantee that if you hand-build this). This is the
zero-dependency escape hatch for the ArduPilot log if pymavlink is unavailable.

OUTPUT
------
A JSON verdict + a 2-panel PNG (range-vs-time with the lethal-radius line and
CPA marked; top-down East-North trajectory with CPA marked) written to
--out-dir (default logs/field_score/).

SELF-TEST
---------
`--self-test` runs entirely offline (no PX4/Gazebo, no real log files, no
network, no hardware). It covers:
  1-3. Two synthetic crossing trajectories with an analytically-known CPA
       (closed-form straight-line minimum-distance, computed independently of
       the interpolation/argmin code path under test); a wide miss, an
       exact-by-construction kill, and a no-overlap error case.
  4.   A REAL synthetic ArduPilot DataFlash .BIN (written byte-for-byte to a
       temp file and parsed back through pymavlink's DFReader -- the actual
       .BIN code path, not a mock) exercised END-TO-END: .BIN target vs a
       synthetic lat/lon interceptor, CPA asserted against the analytic ground
       truth. Also asserts POS-preferred extraction, UTC recovery, and the
       boot-relative `utc_synced: false` fallback when the log has no GPS fix.
If pymavlink is not installed the .BIN cases print SKIP (they do not fail the
suite) -- but on the .venv here pymavlink is present and they run for real. Run:
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


# A UTC epoch-seconds timestamp is >> the year-2001 mark (~9.78e8); a
# boot-relative timestamp starts near 0. Same idea as the ULog path's
# `utc > 1e15` microsecond test, one thousand-thousandth the scale because
# DFReader hands us SECONDS not microseconds.
_UTC_EPOCH_FLOOR_S = 1.0e9


def load_track_from_bin(path: Path, label: str) -> Track:
    """Load an ArduPilot DataFlash .BIN track (constraint target-is-ardupilot).

    Prefers the EKF-fused POS message over raw GPS (see module docstring). The
    timestamps DFReader returns are seconds-since-1970 when the log had a GPS
    UTC fix (its modern-usec clock derives the base from the GPS week/ms), or
    boot-relative seconds otherwise -- we detect which by magnitude and set
    `utc_synced` accordingly, mirroring the ULog path's UTC/boot-relative logic.
    """
    try:
        from pymavlink import DFReader  # deps_needed: pymavlink (pip into .venv)
    except ImportError as e:
        raise ImportError(
            f"pymavlink is required to read ArduPilot .BIN logs ({e}). Install "
            f"it into the venv (`.venv/bin/pip install pymavlink`), or export "
            f"the .BIN to CSV (Mission Planner 'Convert .bin to .csv', columns "
            f"t_utc_s/lat_deg/lon_deg/alt_m) and use --csv-{label[:1] or 'a'}."
        )

    reader = DFReader.DFReader_binary(str(path))

    # Collect POS and GPS series in one pass. Real flight logs are tens of MB;
    # recv_match filters to just these two types, so this stays cheap. Each
    # sample is (utc_or_boot_seconds, lat_deg, lon_deg, alt_m).
    pos_rows, gps_rows = [], []

    def _row(m):
        lat = getattr(m, "Lat", None)
        lon = getattr(m, "Lng", None)
        alt = getattr(m, "Alt", None)
        if lat is None or lon is None or alt is None:
            return None
        return (float(m._timestamp), float(lat), float(lon), float(alt))

    while True:
        m = reader.recv_match(type=["POS", "GPS"])
        if m is None:
            break
        row = _row(m)
        if row is None:
            continue
        (pos_rows if m.get_type() == "POS" else gps_rows).append(row)

    warnings = []
    if pos_rows:
        rows, source = pos_rows, "dataflash:POS"
    elif gps_rows:
        rows, source = gps_rows, "dataflash:GPS"
        warnings.append(
            "position source is raw DataFlash GPS (no POS/EKF-fused message in "
            "this log) -- unfiltered receiver fix, noisier than the POS estimate."
        )
    else:
        raise ValueError(
            f"{path}: no POS or GPS position messages found in this DataFlash "
            f".BIN -- cannot recover a global position track. Check the "
            f"ArduPilot LOG_BITMASK includes POS/GPS, or use --csv instead."
        )

    arr = np.asarray(rows, dtype=np.float64)
    t, lat, lon, alt = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]

    # utc_synced iff DFReader resolved a real UTC clock (timestamps land in the
    # 21st century, not near boot-zero). The floor is compared on the median so
    # a stray early sample can't flip the decision.
    utc_synced = bool(np.median(t) > _UTC_EPOCH_FLOOR_S)
    if not utc_synced:
        warnings.append(
            "no GPS UTC time fix found in this DataFlash log -- falling back to "
            "boot-relative timestamps. Cross-aircraft alignment then assumes the "
            "two flight controllers' boot clocks are already synchronized (NOT "
            "generally true for two independent FCs); treat the CPA time as "
            "approximate."
        )

    t, lat, lon, alt = _sorted_by_time(t, lat, lon, alt)
    return Track(
        label=label,
        t_utc_s=t,
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


# --- Synthetic ArduPilot DataFlash .BIN writer (self-test fixture only) -------
# Writes a byte-for-byte real DataFlash log so the self-test drives the ACTUAL
# pymavlink DFReader parse path (not a monkeypatch). Format reference:
# DFReader.FORMAT_TO_STRUCT and the message framing (HEAD1=0xA3, HEAD2=0x95,
# type byte, little-endian body). Only the handful of format chars this fixture
# uses are mapped; the reader itself supports the full set.
_DF_HEAD1, _DF_HEAD2, _DF_FMT_TYPE = 0xA3, 0x95, 0x80
# DataFlash format char -> python struct char (subset; mirrors FORMAT_TO_STRUCT:
# L is a *signed* int32 scaled 1e-7, hence 'i' not 'L').
_DF_TO_STRUCT = {"Q": "Q", "L": "i", "f": "f", "B": "B", "H": "H", "I": "I",
                 "i": "i", "n": "4s", "N": "16s", "Z": "64s"}


def _df_struct_fmt(fmt: str) -> str:
    return "<" + "".join(_DF_TO_STRUCT[c] for c in fmt)


def _df_fmt_msg(type_id, name, fmt, columns) -> bytes:
    import struct
    length = 3 + struct.calcsize(_df_struct_fmt(fmt))  # incl. 3-byte header
    body = struct.pack("<BB4s16s64s", type_id, length,
                       name.encode(), fmt.encode(), columns.encode())
    return bytes([_DF_HEAD1, _DF_HEAD2, _DF_FMT_TYPE]) + body


def _df_data_msg(type_id, fmt, values) -> bytes:
    import struct
    return bytes([_DF_HEAD1, _DF_HEAD2, type_id]) + struct.pack(
        _df_struct_fmt(fmt), *values)


def _utc_to_gps_week_ms(utc_s: float):
    """Invert DFReaderClock._gpsTimeToTime so a synthetic GPS message anchors the
    reader's clock to a chosen UTC epoch (18 leap seconds, matching DFReader)."""
    gps_epoch = 86400 * (10 * 365 + int((1980 - 1969) / 4) + 1 + 6 - 2)  # 315964800
    s = utc_s - gps_epoch + 18.0
    week = int(s // (86400 * 7))
    ms = (s - week * 86400 * 7) * 1000.0
    return week, ms


def _write_synthetic_bin(path, pos, t_rel, base_utc, lat0, lon0, alt0,
                         with_gps_utc=True):
    """Write a minimal valid ArduPilot DataFlash .BIN: FMT(FMT), FMT(GPS),
    FMT(POS), one anchoring GPS message, then a POS sample per row. `pos` is an
    (N,3) ENU array; converted to lat/lon/alt via the same enu_to_latlon the
    other synthetic tracks use. with_gps_utc=False writes GWk=0 (no fix) so the
    reader falls back to boot-relative time -- exercises the utc_synced=False leg.
    """
    GPS_ID, POS_ID = 0x81, 0x82
    GPS_FMT = "QBIHBfLLf"
    GPS_COLS = "TimeUS,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt"
    POS_FMT = "QLLfff"
    POS_COLS = "TimeUS,Lat,Lng,Alt,RelHomeAlt,RelOriginAlt"

    lat, lon, alt = enu_to_latlon(pos[:, 0], pos[:, 1], pos[:, 2], lat0, lon0, alt0)
    gwk, gms = _utc_to_gps_week_ms(base_utc)

    out = bytearray()
    out += _df_fmt_msg(_DF_FMT_TYPE, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns")
    out += _df_fmt_msg(GPS_ID, "GPS", GPS_FMT, GPS_COLS)
    out += _df_fmt_msg(POS_ID, "POS", POS_FMT, POS_COLS)
    if with_gps_utc:
        out += _df_data_msg(GPS_ID, GPS_FMT, (
            0, 3, int(round(gms)), int(gwk), 12, 0.8,
            int(round(lat[0] * 1e7)), int(round(lon[0] * 1e7)), float(alt[0])))
    else:
        out += _df_data_msg(GPS_ID, GPS_FMT, (
            0, 1, 0, 0, 5, 5.0,
            int(round(lat[0] * 1e7)), int(round(lon[0] * 1e7)), float(alt[0])))
    for i in range(len(t_rel)):
        out += _df_data_msg(POS_ID, POS_FMT, (
            int(round(t_rel[i] * 1e6)),
            int(round(lat[i] * 1e7)), int(round(lon[i] * 1e7)),
            float(alt[i]), 0.0, 0.0))
    Path(path).write_bytes(bytes(out))


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

    # Case 4: ArduPilot DataFlash .BIN, END-TO-END through the real DFReader.
    # A synthetic .BIN target vs a synthetic lat/lon interceptor, scored against
    # the analytic CPA -- plus POS-preferred/UTC-recovery and the boot-relative
    # utc_synced=False fallback. Skips (does NOT fail) if pymavlink is absent.
    try:
        import pymavlink  # noqa: F401
        have_pymavlink = True
    except ImportError:
        have_pymavlink = False

    if not have_pymavlink:
        print("[self-test] case4_bin_end_to_end: SKIP (pymavlink not installed "
              "-- .BIN path untested here; `pip install pymavlink` to run it)")
    else:
        import tempfile

        # Interceptor A (synthetic lat/lon) and target B (real .BIN), both on the
        # shared UTC axis; stationary-target geometry -> analytic CPA = 2.0 m @ 5s.
        duration = 10.0
        posA0, vA = np.array([-30.0, 5.0, 2.0]), np.array([6.0, -1.0, 0.0])
        posB0, vB = np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])
        t_star, cpa_true = _analytic_straight_line_cpa(
            posA0 - posB0, vA - vB, 0.0, duration)

        track_a = _synthetic_track("interceptor", posA0, vA, duration, 20.0,
                                   base_utc, lat0, lon0, alt0)
        n_b = int(round(duration * 25.0)) + 1
        t_rel_b = np.linspace(0.0, duration, n_b)
        pos_b = posB0[None, :] + t_rel_b[:, None] * vB[None, :]

        with tempfile.TemporaryDirectory() as td:
            bin_path = Path(td) / "target_ardupilot.BIN"
            _write_synthetic_bin(bin_path, pos_b, t_rel_b, base_utc,
                                 lat0, lon0, alt0, with_gps_utc=True)
            track_b = load_track_from_bin(bin_path, "target")

            src_ok = track_b.source == "dataflash:POS"
            utc_ok = track_b.utc_synced is True
            n_ok = len(track_b.t_utc_s) == n_b
            result = score_engagement(track_a, track_b, lethal_radius_m=3.0, dt=0.01)
            err_m = abs(result.cpa_m - cpa_true)
            err_t = abs(result.cpa_t_utc_s - (base_utc + t_star))
            pass_e2e = (src_ok and utc_ok and n_ok and err_m <= 0.05
                        and err_t <= 0.05 and result.verdict == "KILL")
            print(f"[self-test] case4_bin_end_to_end: cpa_true={cpa_true:.4f} m  "
                  f"cpa_got={result.cpa_m:.4f} m  err={err_m:.4f} m (tol 0.05)  "
                  f"t_err={err_t:.4f}s (tol 0.05)  source={track_b.source}  "
                  f"utc_synced={track_b.utc_synced}  n={len(track_b.t_utc_s)}  "
                  f"verdict={result.verdict}  {'PASS' if pass_e2e else 'FAIL'}")
            ok = ok and pass_e2e

            # Boot-relative leg: no GPS UTC fix -> utc_synced must be False + warn.
            bin_boot = Path(td) / "target_noutc.BIN"
            _write_synthetic_bin(bin_boot, pos_b, t_rel_b, base_utc,
                                 lat0, lon0, alt0, with_gps_utc=False)
            track_boot = load_track_from_bin(bin_boot, "target")
            boot_ok = (track_boot.utc_synced is False
                       and any("boot-relative" in w for w in track_boot.warnings))
            print(f"[self-test] case4b_bin_boot_relative: "
                  f"utc_synced={track_boot.utc_synced} (expect False)  "
                  f"warned={boot_ok}  {'PASS' if boot_ok else 'FAIL'}")
            ok = ok and boot_ok

    print(f"[self-test] {'PASS' if ok else 'FAIL'}")
    return ok


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--ulog-a", type=Path, help="side-A PX4 ULog (.ulg) [interceptor]")
    ap.add_argument("--ulog-b", type=Path, help="side-B PX4 ULog (.ulg) [target]")
    ap.add_argument("--bin-a", type=Path,
                     help="side-A ArduPilot DataFlash log (.bin) [needs pymavlink]")
    ap.add_argument("--bin-b", type=Path,
                     help="side-B ArduPilot DataFlash log (.bin) -- the Tier-1 "
                          "target is ArduPilot (constraint target-is-ardupilot)")
    ap.add_argument("--csv-a", type=Path, help="side-A track CSV (zero-dep fallback)")
    ap.add_argument("--csv-b", type=Path,
                     help="side-B track CSV fallback -- for the ArduPilot .bin "
                          "when pymavlink is absent, export via Mission Planner "
                          "'Convert .bin to .csv' (columns t_utc_s,lat_deg,lon_deg,alt_m)")
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

    # Each side takes exactly ONE source (ULog | .bin | CSV), chosen
    # independently -- so a PX4-ULog interceptor + ArduPilot-.bin target (the
    # real Tier-1 pairing, constraint target-is-ardupilot) is a first-class
    # combination, not a special case.
    def _select_side(side, ulog, bin_, csv):
        given = [(ulog, load_track_from_ulog), (bin_, load_track_from_bin),
                 (csv, load_track_from_csv)]
        given = [(p, fn) for p, fn in given if p]
        if len(given) == 0:
            ap.error(f"side {side}: provide one of --ulog-{side} / --bin-{side} "
                     f"/ --csv-{side}")
        if len(given) > 1:
            ap.error(f"side {side}: provide only ONE source "
                     f"(--ulog-{side} / --bin-{side} / --csv-{side})")
        return given[0]

    if not (args.ulog_a or args.bin_a or args.csv_a or
            args.ulog_b or args.bin_b or args.csv_b):
        ap.error("provide one source per side (--ulog-* / --bin-* / --csv-*), "
                 "or --self-test")
    path_a, load_a = _select_side("a", args.ulog_a, args.bin_a, args.csv_a)
    path_b, load_b = _select_side("b", args.ulog_b, args.bin_b, args.csv_b)

    try:
        track_a = load_a(path_a, args.label_a)
        track_b = load_b(path_b, args.label_b)

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
