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
import math
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

# Mechanism-anchored kill radii referenced in docs/decisions.md and
# docs/hardware_order_list.md Sec.0c: ~0.35 m for a kinetic ram, ~1.5 m for
# a net/frag-style kill radius.
#
# DEFAULT CORRECTED 2026-07-24 (full project review): the default was 1.5 m -- the
# NET number -- but the net was PARKED by builder ruling (RAM confirmed, contract
# stage `kill`). Scoring a ram attempt against a net radius scores the wrong
# mechanism and flatters the result by 3x.
#
# RATIFIED 2026-07-25 (ADR-0084): the ram radius is 0.35 m, DERIVED from the
# ordered 5-INCH pair, not the retired 7-inch 0.5 m. Arithmetic: interceptor
# Mark5 Pro half-span (226 mm wheelbase + 129.5 mm 5.1" prop)/2 = 177.8 mm;
# target Source One V6 half-span (220+129.5)/2 = 174.8 mm; contact envelope
# = sum of half-spans = 352.5 mm -> 0.35 m (rounded down, conservative). The
# old 0.5 m = two 7-inch aircraft (2 x 238.9 mm half-span = 0.48 m). Pass
# --lethal-radius 1.5 explicitly for a net-class claim; 0.5 only to reproduce
# a historical 7-inch-anchored table.
DEFAULT_LETHAL_RADIUS_M = 0.35

# The floor of the KILL/MISS uncertainty band, metres. WHY 2.0: cpa_m differences
# two INDEPENDENT GPS/EKF ABSOLUTE positions, and the inter-receiver bias between
# two different receivers/firmwares is ~1-3 m horizontal (worse vertical) -- an
# error `eph` cannot see, because eph is each EKF's estimate of its OWN error.
# So the band is WIDEN-ONLY: measured per-sample accuracy can only make it
# larger, never smaller (2026-07-26 -- a naive band = RSS(eph) would report a
# confident call on a 0.75 m CPA at the 0.35 m ram radius, which the geometry
# cannot support).
POSITION_BIAS_FLOOR_M = 2.0

# Residual cross-aircraft TIME-ALIGNMENT error, seconds, used to bound the CPA.
# The two FCs' position axes are each early/late by their own GNSS fix latency,
# and only ONE half of that is sourced: PX4's EKF2_GPS_DELAY default is 110 ms
# (PX4-Autopilot v1.17.0, src/modules/ekf2/params_gnss.yaml:30-37, verified on
# this machine). The ArduPilot side's lag is UNMEASURED here (no ArduPilot
# checkout available), and the residual is the DIFFERENCE of two same-class
# u-blox latencies, so 0.10 s is a conservative GIVEN, not a measurement --
# it is stamped as such in the report. Override with --time-align-uncertainty-s.
DEFAULT_TIME_ALIGN_SIGMA_S = 0.10

# Lateral acceleration used to bound what a LOG DROPOUT can hide, m/s^2. Both
# tracks are linearly interpolated between samples, so a gap of T seconds can
# hide a chord deviation of up to a*T^2/8. 9.81 = 1 g, a modest quad manoeuvre
# (the sim's jinking target pulls more). Setting that chord equal to the lethal
# radius gives the longest gap this tool will bridge and still call KILL/MISS:
# sqrt(8*0.35/9.81) = 0.53 s at the ram radius.
GAP_LATERAL_ACCEL_MPS2 = 9.81


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
    # COUNTED DROPS (2026-07-26). n_nonfinite_dropped: samples whose time or any
    # position channel was NaN/inf (ArduPilot writes quiet_nanf when the EKF has
    # no datum; PX4 publishes a dead-reckoned global position with
    # lat_lon_valid=false). n_invalid_dropped: samples the FLIGHT CONTROLLER
    # ITSELF disclaimed (lat_lon_valid=0 / dead_reckoning=1). Both are reported,
    # never absorbed -- a denominator that shrinks without saying so is this
    # project's whole failure class (docs/error_handling_policy.md Sec.5).
    n_total_raw: int = 0
    n_nonfinite_dropped: int = 0
    n_invalid_dropped: int = 0
    # Per-sample horizontal/vertical position sigma when the log carries it
    # (PX4 vehicle_global_position eph/epv). None = the log has no such field,
    # which is stamped as "unmeasured" rather than silently read as "good".
    eph_m: Optional[np.ndarray] = None
    epv_m: Optional[np.ndarray] = None

    @property
    def mode(self) -> str:
        return "enu" if self.enu is not None else "latlon"

    @property
    def pos(self) -> np.ndarray:
        return self.enu if self.enu is not None else self.latlon

    def gaps(self):
        """(max_gap_s, median_dt_s) of this track's OWN sample times."""
        if len(self.t_utc_s) < 2:
            return float("nan"), float("nan")
        d = np.diff(self.t_utc_s)
        return float(np.max(d)), float(np.median(d))

    def gap_bracketing(self, t: float) -> float:
        """Length of THIS track's own sample interval containing time `t`.

        NOT the union-knot interval analytic_cpa works on: a dense partner track
        keeps every union interval short even when this one has a multi-second
        hole, so the union number cannot see a one-sided dropout (2026-07-26)."""
        tt = self.t_utc_s
        if len(tt) < 2:
            return float("nan")
        i = int(np.searchsorted(tt, t))
        i = min(max(i, 1), len(tt) - 1)
        return float(tt[i] - tt[i - 1])


def _sorted_by_time(t, *arrays):
    order = np.argsort(t)
    return (t[order],) + tuple(a[order] for a in arrays)


def _drop_nonfinite(path, label, t, a, b, c, warnings):
    """Drop samples whose time or ANY position channel is non-finite, COUNT them,
    and REFUSE if nothing survives.

    WHY (2026-07-26, review 3 BLOCKER): ArduPilot's DataFlash writes quiet_nanf
    for an altitude the EKF has no datum for, and a single NaN row propagates
    through np.interp into the CPA. The old code let that reach `cpa_m > radius`,
    which for the `inf` initialiser is True -- so a real kill printed a confident
    `MISS` with exit 0, no warning, and verdict_uncertain false. A sample that is
    not a number is not a measurement; it is dropped, counted, and reported."""
    t = np.asarray(t, dtype=np.float64)
    good = (np.isfinite(t) & np.isfinite(np.asarray(a, dtype=np.float64))
            & np.isfinite(np.asarray(b, dtype=np.float64))
            & np.isfinite(np.asarray(c, dtype=np.float64)))
    n_bad = int((~good).sum())
    if n_bad:
        if good.sum() == 0:
            raise ValueError(
                f"{path}: EVERY one of the {len(t)} position sample(s) in this "
                f"log is non-finite (NaN/inf lat/lon/alt or timestamp) -- there "
                f"is no track to score. Usual cause: the EKF never got an "
                f"absolute position datum (no GPS origin set). Score from video."
            )
        warnings.append(
            f"{n_bad}/{len(t)} position sample(s) were NON-FINITE (NaN/inf) and "
            f"were DROPPED -- the track is interpolated across those times, so a "
            f"manoeuvre inside a dropped span is invisible.")
    return (t[good], np.asarray(a, dtype=np.float64)[good],
            np.asarray(b, dtype=np.float64)[good],
            np.asarray(c, dtype=np.float64)[good], n_bad)


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

    eph = epv = None
    pos_valid = None
    if gpos is not None:
        d = gpos.data
        t_boot = np.asarray(d["timestamp"], dtype=np.float64)
        lat = np.asarray(d["lat"], dtype=np.float64)   # already degrees
        lon = np.asarray(d["lon"], dtype=np.float64)   # already degrees
        alt = np.asarray(d["alt"], dtype=np.float64)   # already metres AMSL
        source = "vehicle_global_position"
        # THE FC'S OWN QUALITY FLAGS (2026-07-26). vehicle_global_position carries
        # lat_lon_valid / alt_valid / dead_reckoning / eph / epv (PX4-Autopilot
        # v1.17.0 msg/versioned/VehicleGlobalPosition.msg, checked on this
        # machine). This tool used to ingest a DEAD-RECKONED position -- one the
        # flight controller itself disclaims -- as if it were a fix, and then
        # apply the same invented +/-2 m band it applies to an RTK-quality
        # flight. Fields absent in an older log stay None and are reported as
        # "unmeasured", never read as "good".
        eph = np.asarray(d["eph"], dtype=np.float64) if "eph" in d else None
        epv = np.asarray(d["epv"], dtype=np.float64) if "epv" in d else None
        ok = np.ones(len(t_boot), dtype=bool)
        have_flag = False
        for f, want in (("lat_lon_valid", True), ("alt_valid", True),
                        ("dead_reckoning", False)):
            if f in d:
                have_flag = True
                v = np.asarray(d[f]).astype(bool)
                ok &= v if want else ~v
        pos_valid = ok if have_flag else None
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

    n_raw = len(t_utc_s)
    n_invalid = 0
    if pos_valid is not None and not pos_valid.all():
        n_invalid = int((~pos_valid).sum())
        if pos_valid.sum() == 0:
            raise ValueError(
                f"{path}: EVERY vehicle_global_position sample is flagged "
                f"INVALID by the flight controller (lat_lon_valid/alt_valid "
                f"false, or dead_reckoning true) -- the EKF never had a global "
                f"position it stood behind. There is nothing to score here.")
        warnings.append(
            f"{n_invalid}/{n_raw} sample(s) were flagged lat_lon_valid=false / "
            f"alt_valid=false / dead_reckoning=true by the FC and were DROPPED "
            f"-- the flight controller itself disclaimed those positions.")
        t_utc_s, lat, lon, alt = (t_utc_s[pos_valid], lat[pos_valid],
                                  lon[pos_valid], alt[pos_valid])
        if eph is not None:
            eph = eph[pos_valid]
        if epv is not None:
            epv = epv[pos_valid]

    t_utc_s, lat, lon, alt, n_bad = _drop_nonfinite(
        path, label, t_utc_s, lat, lon, alt, warnings)
    if n_bad and eph is not None and len(eph) != len(t_utc_s):
        eph = epv = None            # index alignment lost; report as unmeasured

    order = np.argsort(t_utc_s)
    t_utc_s, lat, lon, alt = (t_utc_s[order], lat[order], lon[order], alt[order])
    return Track(
        label=label,
        t_utc_s=t_utc_s,
        latlon=np.column_stack([lat, lon, alt]),
        utc_synced=utc_synced,
        source=source,
        warnings=warnings,
        n_total_raw=n_raw,
        n_nonfinite_dropped=n_bad,
        n_invalid_dropped=n_invalid,
        eph_m=(eph[order] if eph is not None else None),
        epv_m=(epv[order] if epv is not None else None),
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
    n_raw = len(arr)
    t, lat, lon, alt = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]
    # ArduPilot logs quiet_nanf for a field the EKF has no datum for, so this is
    # the .BIN's own version of the non-finite blocker -- drop, count, refuse if
    # nothing survives (2026-07-26).
    t, lat, lon, alt, n_bad = _drop_nonfinite(path, label, t, lat, lon, alt,
                                              warnings)

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
        n_total_raw=n_raw,
        n_nonfinite_dropped=n_bad,
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
    # DON'T TRUST THE COLUMN NAME (2026-07-26). A `t_utc_s` header whose values
    # sit below the 1e9 UTC epoch floor is a boot-relative column that was merely
    # renamed -- `sed '1s/t_s/t_utc_s/'` would otherwise defeat the unsynced-clock
    # refusal entirely. Same magnitude test the ULog/.BIN readers already use.
    if utc_synced and len(t) and float(np.median(t)) <= _UTC_EPOCH_FLOOR_S:
        utc_synced = False
        warnings.append(
            f"the time column is named 't_utc_s' but its median value "
            f"({float(np.median(t)):.1f}) is below the {_UTC_EPOCH_FLOOR_S:.0e} "
            f"UTC epoch floor -- these are BOOT-RELATIVE seconds, not UTC. "
            f"Treating this track as NOT UTC-synced.")
    if not utc_synced:
        warnings.append(
            "this track's time base is NOT UTC -- alignment assumes its clock "
            "is already synchronized with the other track's, which two "
            "independent flight controllers do not do on their own."
        )

    n_raw = len(t)
    if {"lat_deg", "lon_deg", "alt_m"} <= cols:
        lat = np.array([float(r["lat_deg"]) for r in rows])
        lon = np.array([float(r["lon_deg"]) for r in rows])
        alt = np.array([float(r["alt_m"]) for r in rows])
        t, lat, lon, alt, n_bad = _drop_nonfinite(path, label, t, lat, lon, alt,
                                                  warnings)
        t, lat, lon, alt = _sorted_by_time(t, lat, lon, alt)
        return Track(label=label, t_utc_s=t, latlon=np.column_stack([lat, lon, alt]),
                      utc_synced=utc_synced, source="csv:latlon", warnings=warnings,
                      n_total_raw=n_raw, n_nonfinite_dropped=n_bad)
    elif {"east_m", "north_m"} <= cols:
        e = np.array([float(r["east_m"]) for r in rows])
        n = np.array([float(r["north_m"]) for r in rows])
        # A MISSING up_m IS NOT A ZERO ALTITUDE. Defaulting it silently flattens
        # this track and biases the CPA toward a SMALLER (kill-flattering) number
        # when only one side omits the column, so it is named in the source tag
        # and warned about (2026-07-26).
        has_up = "up_m" in cols
        u = np.array([float(r.get("up_m", 0.0) or 0.0) for r in rows])
        if not has_up:
            warnings.append(
                "this CSV has NO up_m column -- the track is treated as PLANAR "
                "(up = 0). If the other side carries a real altitude, the "
                "vertical separation is fiction and cpa_m is biased SMALL.")
        t, e, n, u, n_bad = _drop_nonfinite(path, label, t, e, n, u, warnings)
        t, e, n, u = _sorted_by_time(t, e, n, u)
        return Track(label=label, t_utc_s=t, enu=np.column_stack([e, n, u]),
                      utc_synced=utc_synced,
                      source="csv:enu" + ("" if has_up else " (up_m ABSENT -> 0)"),
                      warnings=warnings,
                      n_total_raw=n_raw, n_nonfinite_dropped=n_bad)
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
    # np.arange is half-open, so the grid stops up to one dt SHORT of the real
    # end of the overlap. That is only a plot artefact now that the verdict uses
    # the true window (see score_engagement), but the range curve must still
    # cover the span the CPA marker can land in -- otherwise the marker is drawn
    # past the end of the plotted history (2026-07-26).
    if len(t_grid) and t_grid[-1] < hi - 1e-12:
        t_grid = np.append(t_grid, hi)
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


# THE INCONCLUSIVE FAMILY. Every one of these means "this tool could not make
# the measurement", and main() returns EXIT_UNCERTAIN (3) for all of them, per
# the shell exit-code contract in scripts/field/common.sh and
# docs/error_handling_policy.md Sec.2 -- "could not decide" must never share a
# code with PASS. (Before 2026-07-26 even the truncation verdict exited 0, so
# 04_pull_logs.sh printed PASS over a report that says it decided nothing.)
VERDICT_TRUNCATED = "INCONCLUSIVE_LOG_TRUNCATED"
VERDICT_UNSYNCED = "INCONCLUSIVE_CLOCKS_UNSYNCED"
VERDICT_GAPPED = "INCONCLUSIVE_LOG_GAP"
INCONCLUSIVE_VERDICTS = (VERDICT_TRUNCATED, VERDICT_UNSYNCED, VERDICT_GAPPED)
EXIT_UNCERTAIN = 3


def analytic_cpa(t_a, pos_a, t_b, pos_b, lo: float, hi: float):
    """EXACT closest approach of the two piecewise-linear tracks over [lo, hi].

    Returns (t_cpa, cpa_m, pos_a_at_cpa, pos_b_at_cpa, at_edge, edge_side).

    WHY THIS EXISTS (defect fixed 2026-07-25, review 2 BLOCKER). The CPA used to
    be `min(range)` over a UNIFORM RESAMPLE GRID whose step was the finer log's
    median sample interval clamped to <=0.2 s. The quantization error of a grid
    argmin is ~v_rel*dt/2 -- 1-2 m at a 20 m/s closing speed, i.e. 2-4x the 0.5 m
    lethal radius -- so KILL/MISS was decided by the LOG RATE, not by geometry.
    PX4's default logger writes vehicle_global_position (the topic this scorer
    PREFERS) at 200 ms = 5 Hz, and no caller passes --dt, so this was the
    production path. Measured on the old code: a true 0.30 m KILL at 5/5 Hz
    reported 2.022 m MISS.

    The fix removes the quantization TERM, it does not shrink it. Both tracks are
    linear between their OWN samples, so on the UNION of both sample-time sets
    the relative motion is exactly linear inside every knot interval, and the
    minimum of |p0 + v*(t-t0)| on an interval is the closed form
    t* = t0 - (p0.v)/(v.v) clamped to the interval. Taking the best over all
    intervals is therefore the exact CPA of the interpolated tracks -- no residual
    grid error at any log rate.

    at_edge is True only when the winning interval's UNCLAMPED minimum lies
    OUTSIDE the observed overlap window, i.e. the aircraft were still closing
    when the logs stopped overlapping. That is not a CPA, it is a truncation, and
    score_engagement turns it into INCONCLUSIVE_LOG_TRUNCATED rather than a
    confident MISS/KILL at the boundary sample.
    """
    knots = np.unique(np.concatenate([
        np.asarray(t_a, dtype=np.float64), np.asarray(t_b, dtype=np.float64),
        np.array([lo, hi], dtype=np.float64)]))
    knots = knots[(knots >= lo - 1e-12) & (knots <= hi + 1e-12)]
    if len(knots) < 2:
        raise ValueError("overlapping window too short for an analytic CPA")
    a_i = np.column_stack([np.interp(knots, t_a, pos_a[:, k]) for k in range(3)])
    b_i = np.column_stack([np.interp(knots, t_b, pos_b[:, k]) for k in range(3)])
    rel = a_i - b_i

    best = (float("inf"), float(knots[0]), False, None)
    for i in range(len(knots) - 1):
        t0, t1 = float(knots[i]), float(knots[i + 1])
        seg = t1 - t0
        if seg <= 0:
            continue
        p0 = rel[i]
        v = (rel[i + 1] - p0) / seg
        vv = float(np.dot(v, v))
        t_raw = t0 if vv == 0.0 else t0 - float(np.dot(p0, v)) / vv
        t_star = min(max(t_raw, t0), t1)
        d = float(np.linalg.norm(p0 + v * (t_star - t0)))
        if d < best[0]:
            # Edge-clamped only if the true (unclamped) minimum is OUTSIDE the
            # whole observed window -- an interior clamp just means the minimum
            # sits in a neighbouring interval, which is not a truncation.
            side = None
            if i == 0 and t_raw < lo - 1e-9:
                side = "start"
            elif i == len(knots) - 2 and t_raw > hi + 1e-9:
                side = "end"
            best = (d, t_star, side is not None, side)
    cpa_m, t_cpa, at_edge, side = best
    if not np.isfinite(cpa_m):
        # The `inf` initialiser must never escape as a distance: `inf > radius`
        # is True, so it used to print a confident MISS (2026-07-26 BLOCKER).
        # With the loaders rejecting non-finite samples this is unreachable --
        # it is kept as an assertion, not as a code path.
        raise ValueError(
            "the closest approach came out NON-FINITE over "
            f"[{lo:.3f}, {hi:.3f}] -- no usable segment in the overlap. This is "
            "not a MISS; there is no measurement here.")
    pa = np.array([float(np.interp(t_cpa, t_a, pos_a[:, k])) for k in range(3)])
    pb = np.array([float(np.interp(t_cpa, t_b, pos_b[:, k])) for k in range(3)])
    return t_cpa, cpa_m, pa, pb, at_edge, side


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
    # Analytic-CPA provenance (2026-07-25). cpa_m/cpa_t_utc_s above are the
    # ANALYTIC values; these expose what the old grid argmin would have said and
    # whether the minimum is real or a log truncation.
    cpa_grid_m: float = float("nan")
    cpa_grid_t_utc_s: float = float("nan")
    cpa_at_window_edge: bool = False
    cpa_edge_side: Optional[str] = None
    cpa_pos_a: Optional[np.ndarray] = None
    cpa_pos_b: Optional[np.ndarray] = None
    # ENU reference the two positions above are expressed in (lat0, lon0, alt0),
    # or None for an already-local ENU CSV pair. Without it cpa_pos_* are
    # uninterpretable and the run is not reproducible (2026-07-26).
    enu_origin: Optional[tuple] = None
    # WHAT A LOG DROPOUT COULD HIDE (2026-07-26). Both tracks are straight lines
    # between their own samples, so a hole is bridged silently. cpa_gap_*_s are
    # EACH track's own sample interval bracketing the CPA (never the union-knot
    # interval, which a dense partner track keeps short); the chord term is the
    # worst-case deviation a 1 g manoeuvre could hide inside the longer of them.
    cpa_gap_a_s: float = float("nan")
    cpa_gap_b_s: float = float("nan")
    cpa_gap_chord_m: float = 0.0
    gap_bridged: bool = False
    # Sensitivity of cpa_m to the residual cross-aircraft clock error, measured
    # by re-running the same closed form with track A shifted +/- delta. This is
    # geometry-aware: a head-on pass is insensitive (displacement along v_rel
    # cancels at the minimum), a crossing pass is not.
    time_align_sigma_s: float = 0.0
    cpa_time_sensitivity_m: float = 0.0
    uncertainty_band_m: float = POSITION_BIAS_FLOOR_M
    uncertainty_band_source: str = ""
    clock_offset_b_s: float = 0.0
    clocks_assumed_aligned: bool = False

    @property
    def cpa_grid_quantization_m(self) -> float:
        """How far the pre-2026-07-25 grid argmin was from the true CPA on this
        engagement -- the error term the analytic CPA removes. One-sided: a grid
        minimum can only ever be too LARGE."""
        return abs(self.cpa_grid_m - self.cpa_m)


def _eph_at(track: Track, t: float, which: str = "eph") -> Optional[float]:
    """The track's own logged position sigma at time `t`, or None if this log
    carries no such field (then it is UNMEASURED, never assumed good)."""
    arr = track.eph_m if which == "eph" else track.epv_m
    if arr is None or len(arr) != len(track.t_utc_s) or len(arr) == 0:
        return None
    v = float(np.interp(t, track.t_utc_s, arr))
    return v if np.isfinite(v) else None


def score_engagement(track_a: Track, track_b: Track, lethal_radius_m: float,
                      dt: Optional[float] = None,
                      clock_offset_b_s: float = 0.0,
                      assume_clocks_aligned: bool = False,
                      time_align_sigma_s: float = DEFAULT_TIME_ALIGN_SIGMA_S,
                      ) -> ScoreResult:
    pos_a_full, pos_b_full = project_to_common_enu(track_a, track_b)
    # A MEASURED offset between the two logs' time bases (from the protocol's
    # arm-time / common-trigger pair) is APPLIED here and stamped in the report;
    # it is never defaulted to something other than 0.
    t_b_full = track_b.t_utc_s + float(clock_offset_b_s)
    t_grid, pos_a, pos_b, rng, used_dt = compute_range_track(
        track_a.t_utc_s, pos_a_full, t_b_full, pos_b_full, dt=dt
    )
    # The resample grid is kept for the range PLOT and the report; the VERDICT
    # number comes from the exact per-segment closed form (analytic_cpa).
    grid_idx = int(np.argmin(rng))
    cpa_grid_m = float(rng[grid_idx])
    cpa_grid_t = float(t_grid[grid_idx])

    # THE SCORING WINDOW IS THE TRUE LOG OVERLAP, NOT THE GRID (2026-07-26
    # BLOCKER). `np.arange(lo, hi, dt)` is half-open, so taking the window from
    # t_grid[0]/t_grid[-1] cut up to one full dt (0.2 s at PX4's default 5 Hz =
    # 2.4 m of closing at 12 m/s) off the END of the scored span -- exactly where
    # the CPA of a successful ram lives, since the interceptor's log stops on
    # impact. That turned interior KILLs into INCONCLUSIVE_LOG_TRUNCATED and made
    # the verdict a function of --dt, which the --dt help text promised it was not.
    lo = max(float(track_a.t_utc_s.min()), float(t_b_full.min()))
    hi = min(float(track_a.t_utc_s.max()), float(t_b_full.max()))
    cpa_t, cpa_m, pa, pb, at_edge, side = analytic_cpa(
        track_a.t_utc_s, pos_a_full, t_b_full, pos_b_full, lo, hi)

    # ---- what a DROPOUT could hide, per track (never the union interval) ----
    gap_a = track_a.gap_bracketing(cpa_t)
    gap_b = track_b.gap_bracketing(cpa_t - float(clock_offset_b_s))
    _med_a, _med_b = track_a.gaps()[1], track_b.gaps()[1]
    gap_worst = float(np.nanmax([gap_a, gap_b]))
    chord_m = GAP_LATERAL_ACCEL_MPS2 * gap_worst ** 2 / 8.0
    # Bridgeable-gap ceiling, PER TRACK: the longer of (3x THAT track's own
    # median dt -- a slow logger is not a dropout) and the gap whose 1 g chord
    # equals the lethal radius (sqrt(8R/a); 0.53 s at the 0.35 m ram radius).
    # Per track, not pooled: a 0.5 s hole in a 25 Hz log is a dropout even when
    # the other aircraft logs at 5 Hz.
    _floor = math.sqrt(8.0 * lethal_radius_m / GAP_LATERAL_ACCEL_MPS2)
    gap_bridged = any(
        np.isfinite(g) and np.isfinite(m) and g > max(3.0 * m, _floor)
        for g, m in ((gap_a, _med_a), (gap_b, _med_b)))

    # ---- how much the residual clock error could move cpa_m ----
    # Re-run the SAME closed form with A's clock shifted +/- delta. Displacement
    # along the relative-velocity direction cancels at a minimum, so this is ~0
    # for a head-on pass and large for a crossing one -- which is why it is
    # measured on the flown geometry instead of quoted as v_closing x delta.
    d = abs(float(time_align_sigma_s))
    sens = 0.0
    if d > 0:
        for sgn in (-1.0, +1.0):
            try:
                t_a_sh = track_a.t_utc_s + sgn * d
                lo_s = max(float(t_a_sh.min()), float(t_b_full.min()))
                hi_s = min(float(t_a_sh.max()), float(t_b_full.max()))
                if hi_s <= lo_s:
                    continue
                _t, c_sh, _pa, _pb, _e, _s = analytic_cpa(
                    t_a_sh, pos_a_full, t_b_full, pos_b_full, lo_s, hi_s)
                sens = max(sens, abs(c_sh - cpa_m))
            except ValueError:
                continue

    # ---- the uncertainty band: WIDEN-ONLY, and it says where it came from ----
    eph_a, eph_b = _eph_at(track_a, cpa_t), _eph_at(track_b, cpa_t)
    band = POSITION_BIAS_FLOOR_M
    band_src = (f"literature floor {POSITION_BIAS_FLOOR_M:.1f} m "
                f"(GPS inter-receiver bias; per-sample eph unmeasured in "
                f"{'both logs' if (eph_a is None and eph_b is None) else 'one log'})")
    if eph_a is not None or eph_b is not None:
        rss = math.sqrt((eph_a or 0.0) ** 2 + (eph_b or 0.0) ** 2)
        meas = 2.0 * rss          # k=2, ~95%
        if meas > band:
            band = meas
            band_src = (f"2 x RSS(eph) = 2 x sqrt({(eph_a or float('nan')):.2f}^2 + "
                        f"{(eph_b or float('nan')):.2f}^2) m at the CPA "
                        f"(WIDER than the {POSITION_BIAS_FLOOR_M:.1f} m floor)")
        else:
            band_src += (f"; logged eph at the CPA "
                         f"(a={eph_a}, b={eph_b}) implies only {meas:.2f} m, "
                         f"which does NOT narrow the floor")
    band += sens + chord_m

    clocks_ok = bool(track_a.utc_synced and track_b.utc_synced)
    if at_edge:
        verdict = VERDICT_TRUNCATED
    elif not (clocks_ok or assume_clocks_aligned):
        # TWO BOOT-RELATIVE CLOCKS ALMOST ALWAYS OVERLAP BY ACCIDENT (both start
        # near zero), so the no-overlap refusal never fires and the alignment is
        # pure fiction: a demonstrated 5 s boot skew scored a true collision as a
        # confident 25.000 m MISS, exit 0 (2026-07-26). An unverifiable time base
        # is not a verdict.
        verdict = VERDICT_UNSYNCED
    elif gap_bridged:
        verdict = VERDICT_GAPPED
    else:
        verdict = "KILL" if cpa_m <= lethal_radius_m else "MISS"
    cpa_idx = int(np.argmin(np.abs(t_grid - cpa_t)))
    return ScoreResult(
        t_grid=t_grid, pos_a=pos_a, pos_b=pos_b, range_m=rng, dt_s=used_dt,
        cpa_idx=cpa_idx, cpa_m=cpa_m, cpa_t_utc_s=cpa_t,
        lethal_radius_m=lethal_radius_m, verdict=verdict,
        overlap_s=hi - lo,
        track_a=track_a, track_b=track_b,
        cpa_grid_m=cpa_grid_m, cpa_grid_t_utc_s=cpa_grid_t,
        cpa_at_window_edge=at_edge, cpa_edge_side=side,
        cpa_pos_a=pa, cpa_pos_b=pb,
        enu_origin=(tuple(float(v) for v in track_a.latlon[0])
                    if track_a.mode == "latlon" else None),
        cpa_gap_a_s=gap_a, cpa_gap_b_s=gap_b, cpa_gap_chord_m=chord_m,
        gap_bridged=gap_bridged,
        time_align_sigma_s=d, cpa_time_sensitivity_m=sens,
        uncertainty_band_m=band, uncertainty_band_source=band_src,
        clock_offset_b_s=float(clock_offset_b_s),
        clocks_assumed_aligned=bool(assume_clocks_aligned and not clocks_ok),
    )


# --------------------------------------------------------------------------
# Reporting: JSON + plot
# --------------------------------------------------------------------------

def _sanitize_nonfinite(obj, path=""):
    """Replace every non-finite float anywhere in the report with None and
    return (clean_obj, [dotted field names that were replaced]). Recursive, so a
    field added later is covered without anyone remembering to add a flag."""
    bad = []
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            out[k], b = _sanitize_nonfinite(v, f"{path}.{k}" if path else str(k))
            bad += b
        return out, bad
    if isinstance(obj, (list, tuple)):
        out = []
        for i, v in enumerate(obj):
            c, b = _sanitize_nonfinite(v, f"{path}[{i}]")
            out.append(c)
            bad += b
        return out, bad
    if isinstance(obj, float) and not math.isfinite(obj):
        return None, [path]
    return obj, bad


def write_report(result: ScoreResult, out_dir: Path, video_a: Optional[str],
                  video_b: Optional[str], tag: str) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    report = {
        "tag": tag,
        "verdict": result.verdict,
        "cpa_m": round(result.cpa_m, 3),
        "cpa_t_utc_s": result.cpa_t_utc_s,
        "cpa_method": ("analytic per-segment closed form on the UNION of both "
                       "logs' sample times (exact for piecewise-linear tracks; "
                       "no resample-grid quantization)"),
        # What the pre-2026-07-25 grid argmin would have reported, and the error
        # it carried. At PX4's default 5 Hz global-position logging this term was
        # 1-2 m at a 20 m/s closing speed -- 2-4x the lethal radius.
        "cpa_grid_argmin_m": round(result.cpa_grid_m, 3),
        "cpa_grid_quantization_error_m": round(
            abs(result.cpa_grid_m - result.cpa_m), 3),
        "cpa_at_window_edge": bool(result.cpa_at_window_edge),
        "cpa_edge_side": result.cpa_edge_side,
        "lethal_radius_m": result.lethal_radius_m,
        "time_overlap_s": round(result.overlap_s, 3),
        "resample_dt_s": result.dt_s,
        # WHERE THE AIRCRAFT ACTUALLY WERE (2026-07-26). cpa_m is a 3-D norm, and
        # its VERTICAL channel differences two AMSL datums produced by two
        # independent baro/GPS/EKF stacks -- a constant offset of a few metres
        # between a Kakute H7 (ArduPilot) and a 6C Mini (PX4) is routine and would
        # read as a MISS on every attempt of the day. Serialising the per-axis
        # split, the horizontal-only number, and both positions (with the ENU
        # origin they are expressed in) is what makes such an offset visible --
        # and removable -- after the fact, from a report already written.
        "cpa_pos_a_enu_m": (None if result.cpa_pos_a is None
                            else [round(float(v), 3) for v in result.cpa_pos_a]),
        "cpa_pos_b_enu_m": (None if result.cpa_pos_b is None
                            else [round(float(v), 3) for v in result.cpa_pos_b]),
        "enu_origin_lat_lon_alt": (list(result.enu_origin) if result.enu_origin
                                   else None),
        "cpa_delta_enu_m": (None if (result.cpa_pos_a is None or result.cpa_pos_b is None)
                            else [round(float(v), 3) for v in
                                  (np.asarray(result.cpa_pos_a) - np.asarray(result.cpa_pos_b))]),
        # The HORIZONTAL component OF THE REPORTED CPA -- not the minimum
        # horizontal separation over the window, which is a different number.
        "horizontal_sep_at_cpa_m": (
            None if (result.cpa_pos_a is None or result.cpa_pos_b is None)
            else round(float(np.linalg.norm(
                (np.asarray(result.cpa_pos_a) - np.asarray(result.cpa_pos_b))[:2])), 3)),
        "median_sample_dt_s": {
            result.track_a.label: (
                round(float(np.median(np.diff(result.track_a.t_utc_s))), 4)
                if len(result.track_a.t_utc_s) > 1 else None),
            result.track_b.label: (
                round(float(np.median(np.diff(result.track_b.t_utc_s))), 4)
                if len(result.track_b.t_utc_s) > 1 else None),
        },
        # DROPOUTS, COUNTED AND LOCALISED. max_gap is over the whole track;
        # cpa_gap is that track's OWN sample interval bracketing the CPA, which
        # is the one the verdict turns on (a dense partner track hides a
        # one-sided hole in any union-based statistic).
        "max_sample_gap_s": {
            result.track_a.label: round(result.track_a.gaps()[0], 4),
            result.track_b.label: round(result.track_b.gaps()[0], 4),
        },
        "cpa_bracketing_gap_s": {
            result.track_a.label: round(result.cpa_gap_a_s, 4),
            result.track_b.label: round(result.cpa_gap_b_s, 4),
        },
        "cpa_gap_chord_uncertainty_m": round(result.cpa_gap_chord_m, 3),
        "cpa_gap_bridged": bool(result.gap_bridged),
        # Cross-aircraft time alignment: the assumed residual and what it is
        # worth in metres ON THIS GEOMETRY (measured by re-running the CPA with
        # A's clock shifted +/- that amount), not a v_closing x dt hand-wave.
        "time_align_sigma_s": result.time_align_sigma_s,
        "time_align_sigma_source": (
            "GIVEN, not measured: PX4 EKF2_GPS_DELAY default 110 ms "
            "(PX4-Autopilot v1.17.0 src/modules/ekf2/params_gnss.yaml); the "
            "ArduPilot-side fix latency is UNMEASURED here, and the residual is "
            "the DIFFERENCE of the two. Override with --time-align-uncertainty-s."),
        "cpa_time_sync_sensitivity_m": round(result.cpa_time_sensitivity_m, 3),
        "clock_offset_b_applied_s": result.clock_offset_b_s,
        "clocks_assumed_aligned": bool(result.clocks_assumed_aligned),
        "track_a": {
            "label": result.track_a.label, "source": result.track_a.source,
            "utc_synced": result.track_a.utc_synced,
            "n_samples": int(len(result.track_a.t_utc_s)),
            "n_samples_raw": int(result.track_a.n_total_raw),
            "n_nonfinite_dropped": int(result.track_a.n_nonfinite_dropped),
            "n_fc_invalid_dropped": int(result.track_a.n_invalid_dropped),
            "eph_at_cpa_m": _eph_at(result.track_a, result.cpa_t_utc_s),
            "epv_at_cpa_m": _eph_at(result.track_a, result.cpa_t_utc_s, "epv"),
            "warnings": result.track_a.warnings,
        },
        "track_b": {
            "label": result.track_b.label, "source": result.track_b.source,
            "utc_synced": result.track_b.utc_synced,
            "n_samples": int(len(result.track_b.t_utc_s)),
            "n_samples_raw": int(result.track_b.n_total_raw),
            "n_nonfinite_dropped": int(result.track_b.n_nonfinite_dropped),
            "n_fc_invalid_dropped": int(result.track_b.n_invalid_dropped),
            "eph_at_cpa_m": _eph_at(result.track_b,
                                    result.cpa_t_utc_s - result.clock_offset_b_s),
            "epv_at_cpa_m": _eph_at(result.track_b,
                                    result.cpa_t_utc_s - result.clock_offset_b_s, "epv"),
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
            f"cpa_m differences TWO INDEPENDENT GPS/EKF absolute positions; the "
            f"inter-receiver bias is ~1-3 m horizontal (worse vertical) -- many "
            f"times the {result.lethal_radius_m:.2f} m radius it is being "
            f"compared against. On top of that sit the residual clock alignment "
            f"({result.cpa_time_sensitivity_m:.2f} m on this geometry) and what a "
            f"log dropout could hide ({result.cpa_gap_chord_m:.2f} m). So near "
            f"the threshold the KILL/MISS call is UNCERTAIN; trust the video, "
            f"not cpa_m's 3rd decimal."
        ),
        "uncertainty_band_m": round(result.uncertainty_band_m, 3),
        "uncertainty_band_source": result.uncertainty_band_source,
        # The band is now COMPUTED (position floor, widened by the logged eph
        # when that is larger, plus the measured clock-sensitivity and
        # dropout-chord terms) instead of the flat 2.0 m literal it used to be.
        "verdict_uncertain": bool(
            abs(result.cpa_m - result.lethal_radius_m) < result.uncertainty_band_m),
    }
    # A MISS THE VERTICAL CHANNEL PRODUCED IS NEVER A CONFIDENT MISS. The two
    # AMSL datums are independent, so a constant vertical offset is the single
    # most likely way for a real hit to score as a miss (2026-07-26).
    d_enu = report.get("cpa_delta_enu_m")
    if (report["verdict"] == "MISS" and d_enu is not None
            and abs(d_enu[2]) > report["horizontal_sep_at_cpa_m"]):
        report["verdict_uncertain"] = True
        report["vertical_dominated_miss"] = True
        report["honesty_note"] = (
            f"VERTICAL-DOMINATED MISS: the separation at the CPA is "
            f"{abs(d_enu[2]):.2f} m vertical vs "
            f"{report['horizontal_sep_at_cpa_m']:.2f} m horizontal. The vertical "
            f"channel differences two INDEPENDENT AMSL datums (ArduPilot baro/EKF "
            f"vs PX4 EKF), so a constant few-metre datum offset produces exactly "
            f"this shape on every attempt of the day. Check "
            f"horizontal_sep_at_cpa_m and the pre-launch ground altitudes before "
            f"accepting this MISS. "
        ) + report["honesty_note"]
    if result.cpa_at_window_edge:
        report["honesty_note"] = (
            f"LOG TRUNCATED: the two tracks were still closing at the "
            f"{result.cpa_edge_side} of their overlap window, so the minimum "
            f"range observed ({result.cpa_m:.3f} m) is NOT a closest approach -- "
            f"the real CPA lies outside the logged overlap. Verdict is "
            f"{VERDICT_TRUNCATED}, not KILL/MISS. Recover the missing log span "
            f"(both cards) or score from video. "
        ) + report["honesty_note"]
        report["verdict_uncertain"] = True
    if result.verdict == VERDICT_UNSYNCED:
        report["honesty_note"] = (
            f"CLOCKS NOT CROSS-SYNCHRONIZED: "
            f"{result.track_a.label} utc_synced={result.track_a.utc_synced}, "
            f"{result.track_b.label} utc_synced={result.track_b.utc_synced}. "
            f"Neither log can be put on a common absolute time axis, and two "
            f"BOOT-RELATIVE clocks always overlap by accident (both start near "
            f"zero) -- so the no-overlap refusal cannot save you and the "
            f"{result.cpa_m:.3f} m below is an artefact of an unknown time skew, "
            f"not a measurement. Verdict is {VERDICT_UNSYNCED}. Fixes: fly with a "
            f"GPS time fix on both aircraft; or supply the MEASURED offset with "
            f"--clock-offset-b-s (protocol Sec.6.2's arm-time pair); or accept "
            f"the assumption explicitly with --assume-clocks-aligned. "
        ) + report["honesty_note"]
        report["verdict_uncertain"] = True
    elif not (result.track_a.utc_synced and result.track_b.utc_synced):
        report["honesty_note"] += (
            " WARNING: at least one track had no GPS UTC fix, so the two "
            "logs' clocks were NOT independently cross-synchronized. You passed "
            "--assume-clocks-aligned, so this report ASSUMES the two boot clocks "
            "were started together; nothing here verifies that."
        )
    if result.verdict == VERDICT_GAPPED:
        report["honesty_note"] = (
            f"LOG DROPOUT ACROSS THE CPA: the closest approach falls inside a "
            f"sample gap of {max(result.cpa_gap_a_s, result.cpa_gap_b_s):.3f} s "
            f"(track gaps: {result.track_a.label} {result.cpa_gap_a_s:.3f} s, "
            f"{result.track_b.label} {result.cpa_gap_b_s:.3f} s). Both tracks are "
            f"STRAIGHT LINES between their own samples, so a manoeuvre inside "
            f"that hole is invisible: at 1 g it could hide up to "
            f"{result.cpa_gap_chord_m:.2f} m of deviation, against a "
            f"{result.lethal_radius_m:.2f} m radius. The reported "
            f"{result.cpa_m:.3f} m is what the interpolation says, not what was "
            f"measured. Verdict is {VERDICT_GAPPED}. Score from video. "
        ) + report["honesty_note"]
        report["verdict_uncertain"] = True

    json_path = out_dir / f"field_score_{tag}.json"
    # STRICT JSON, AND NOTHING NON-FINITE PRESENTED AS A NUMBER. Python's default
    # json.dumps writes bare `Infinity`/`NaN` tokens, which are not valid JSON
    # (RFC 8259) and which a browser/jq consumer either rejects or misparses --
    # on precisely the report that most needs reading. Non-finite values become
    # null and are NAMED in nonfinite_fields; allow_nan=False then asserts that
    # the sanitiser missed nothing. Sanitising happens BEFORE the assert so this
    # can never abort the run and cost the builder his PNG (2026-07-26).
    report, nonfinite = _sanitize_nonfinite(report)
    if nonfinite:
        report["nonfinite_fields"] = nonfinite
    json_path.write_text(json.dumps(report, indent=2, allow_nan=False))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax_range, ax_traj) = plt.subplots(2, 1, figsize=(9, 10))

    t_rel = result.t_grid - result.t_grid[0]
    # The marker is drawn at the ANALYTIC CPA (t, range), not at the grid argmin
    # -- otherwise the plot would contradict the printed/reported number.
    t_cpa_rel = result.cpa_t_utc_s - result.t_grid[0]
    ax_range.plot(t_rel, result.range_m, lw=1.5, label="inter-aircraft range")
    ax_range.axhline(result.lethal_radius_m, color="crimson", ls="--", lw=1,
                      label=f"lethal radius = {result.lethal_radius_m:.2f} m")
    ax_range.axvline(t_cpa_rel, color="grey", ls=":", lw=1)
    ax_range.plot(t_cpa_rel, result.cpa_m, "o", color="crimson", ms=8,
                  label=f"CPA = {result.cpa_m:.2f} m  [{result.verdict}]")
    if abs(result.cpa_grid_m - result.cpa_m) > 1e-6:
        ax_range.plot(result.cpa_grid_t_utc_s - result.t_grid[0],
                      result.cpa_grid_m, "x", color="grey", ms=8,
                      label=(f"grid argmin = {result.cpa_grid_m:.2f} m "
                             f"(quantization the analytic CPA removes)"))
    # SHADE EVERY DROPOUT the range curve is interpolated straight through --
    # otherwise the "look at the PNG" backstop shows a smooth curve across a hole
    # where nothing was measured (2026-07-26).
    _shaded = 0
    for _tr, _off in ((result.track_a, 0.0), (result.track_b, result.clock_offset_b_s)):
        _t = _tr.t_utc_s + _off
        _mx, _md = _tr.gaps()
        if not (np.isfinite(_mx) and np.isfinite(_md)) or len(_t) < 2:
            continue
        for _i in np.nonzero(np.diff(_t) > max(3.0 * _md, 0.5))[0]:
            ax_range.axvspan(_t[_i] - result.t_grid[0], _t[_i + 1] - result.t_grid[0],
                             color="orange", alpha=0.18,
                             label=("log dropout (range is INTERPOLATED here)"
                                    if _shaded == 0 else None))
            _shaded += 1
    ax_range.set_xlabel("time since window start (s)")
    ax_range.set_ylabel("range (m)")
    ax_range.set_title(f"field_score: {result.track_a.label} vs {result.track_b.label}")
    ax_range.legend(loc="best", fontsize=8)
    ax_range.grid(alpha=0.3)

    ax_traj.plot(result.pos_a[:, 0], result.pos_a[:, 1], lw=1.5,
                 label=result.track_a.label, color="tab:blue")
    ax_traj.plot(result.pos_b[:, 0], result.pos_b[:, 1], lw=1.5,
                 label=result.track_b.label, color="tab:orange")
    pa_c = result.cpa_pos_a if result.cpa_pos_a is not None else result.pos_a[result.cpa_idx]
    pb_c = result.cpa_pos_b if result.cpa_pos_b is not None else result.pos_b[result.cpa_idx]
    ax_traj.plot(pa_c[0], pa_c[1], "o", color="tab:blue", ms=9, mec="k")
    ax_traj.plot(pb_c[0], pb_c[1], "o", color="tab:orange", ms=9, mec="k")
    ax_traj.plot([pa_c[0], pb_c[0]], [pa_c[1], pb_c[1]],
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

def _lethal_radius_help() -> str:
    """The --lethal-radius help text, factored out so the self-test can assert it
    does not mislabel the mechanism (review 2: the default flipped 1.5 -> 0.5 m
    but the help kept calling 0.5 'the net radius', and every self-test case
    passed an explicit radius so nothing could catch it)."""
    return (f"kill classification threshold, metres (default "
            f"{DEFAULT_LETHAL_RADIUS_M} m = the ADR-0084 kinetic-RAM radius, "
            f"DERIVED from the ordered 5-inch pair's contact envelope (sum of "
            f"half-spans, 177.8 + 174.8 mm); pass 1.5 explicitly for a net-class "
            f"claim, or 0.5 to reproduce a historical 7-inch-anchored table. The "
            f"mechanism labels are canonically defined in scripts/render_hud.py.)")


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
                 "i": "i", "n": "4s", "N": "16s", "Z": "64s",
                 # ATT's angle fields: 'c' = int16 centidegrees (x0.01),
                 # 'C' = uint16 centidegrees. The READER applies the 0.01 scale,
                 # so the fixture must write the raw scaled ints -- writing
                 # floats here would build a log ArduPilot never produces and
                 # would hide a units bug in anything that reads ATT.
                 "c": "h", "C": "H"}


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
                         with_gps_utc=True, att=None, att_t_rel=None):
    """Write a minimal valid ArduPilot DataFlash .BIN: FMT(FMT), FMT(GPS),
    FMT(POS), one anchoring GPS message, then a POS sample per row. `pos` is an
    (N,3) ENU array; converted to lat/lon/alt via the same enu_to_latlon the
    other synthetic tracks use. with_gps_utc=False writes GWk=0 (no fix) so the
    reader falls back to boot-relative time -- exercises the utc_synced=False leg.

    `att` (added 2026-07-25) optionally writes ATT records too: an (M,3) array of
    (roll_deg, pitch_deg, yaw_deg), timestamped by `att_t_rel` (defaults to
    `t_rel`, i.e. one ATT per POS). This is what lets the incidence producer
    (scripts/seeker/placard_incidence.py) be tested against a REAL DFReader parse
    instead of a hand-typed attitude fixture -- the pattern that hid two schema
    breaks before (docs/error_handling_policy.md: the fixture must come from the
    producer's own writer). ATT is written with ArduPilot's real field types
    ('c'/'C' centidegrees), so the reader's 0.01 scaling is exercised and Yaw
    comes back in the 0..360 convention a real log uses.
    """
    GPS_ID, POS_ID, ATT_ID = 0x81, 0x82, 0x83
    GPS_FMT = "QBIHBfLLf"
    GPS_COLS = "TimeUS,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt"
    POS_FMT = "QLLfff"
    POS_COLS = "TimeUS,Lat,Lng,Alt,RelHomeAlt,RelOriginAlt"
    ATT_FMT = "QccccCCff"
    ATT_COLS = "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRP,ErrYaw"

    lat, lon, alt = enu_to_latlon(pos[:, 0], pos[:, 1], pos[:, 2], lat0, lon0, alt0)
    gwk, gms = _utc_to_gps_week_ms(base_utc)

    out = bytearray()
    out += _df_fmt_msg(_DF_FMT_TYPE, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns")
    out += _df_fmt_msg(GPS_ID, "GPS", GPS_FMT, GPS_COLS)
    out += _df_fmt_msg(POS_ID, "POS", POS_FMT, POS_COLS)
    if att is not None:
        out += _df_fmt_msg(ATT_ID, "ATT", ATT_FMT, ATT_COLS)
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
    if att is not None:
        att = np.asarray(att, dtype=float).reshape(-1, 3)
        att_t = np.asarray(t_rel if att_t_rel is None else att_t_rel, dtype=float)
        if len(att_t) != len(att):
            raise ValueError("att and att_t_rel must have the same length")
        for i in range(len(att)):
            roll, pitch, yaw = att[i]
            out += _df_data_msg(ATT_ID, ATT_FMT, (
                int(round(att_t[i] * 1e6)),
                0, int(round(roll * 100.0)),          # DesRoll, Roll   ('c')
                0, int(round(pitch * 100.0)),         # DesPitch, Pitch ('c')
                0, int(round((yaw % 360.0) * 100.0)),  # DesYaw, Yaw    ('C')
                0.0, 0.0))                            # ErrRP, ErrYaw
    Path(path).write_bytes(bytes(out))


class _StubULogDataset:
    def __init__(self, data):
        self.data = data


class _StubULog:
    """Minimal stand-in for pyulog.ULog: only .get_dataset(name).data is used by
    load_track_from_ulog. Raises KeyError for absent topics, like the real one."""

    def __init__(self, datasets):
        self._d = datasets

    def get_dataset(self, name):
        if name not in self._d:
            raise KeyError(name)
        return _StubULogDataset(self._d[name])


def _install_stub_pyulog(datasets):
    """Put a fake `pyulog` module on sys.modules so load_track_from_ulog's
    function-level `from pyulog import ULog` picks it up. Returns the previous
    entry (or None) so the caller can restore it."""
    import types
    prev = sys.modules.get("pyulog")
    mod = types.ModuleType("pyulog")
    mod.ULog = lambda _path: _StubULog(datasets)
    sys.modules["pyulog"] = mod
    return prev


def _self_test_ulog_reader(lat0, lon0, alt0, base_utc) -> bool:
    ok = True
    n = 50
    t_boot_us = np.arange(n, dtype=np.float64) * 200_000.0     # 5 Hz, PX4 default
    utc_us = t_boot_us + base_utc * 1e6
    east = np.linspace(0.0, 40.0, n)
    north = np.full(n, 5.0)
    up = np.full(n, 3.0)                                       # LOW-AMSL site
    lat, lon, alt = enu_to_latlon(east, north, up, lat0, lon0, alt0)

    prev = _install_stub_pyulog({
        "vehicle_gps_position": {"timestamp": t_boot_us, "time_utc_usec": utc_us,
                                 "lat": (lat * 1e7), "lon": (lon * 1e7),
                                 "alt": (alt * 1e3)},
        "vehicle_global_position": {"timestamp": t_boot_us, "lat": lat,
                                    "lon": lon, "alt": alt},
    })
    try:
        tr = load_track_from_ulog(Path("stub.ulg"), "interceptor")
        e, nn, u = latlon_to_enu(*tr.latlon.T, lat0, lon0, alt0)
        c_a = (tr.source == "vehicle_global_position" and tr.utc_synced is True
               and len(tr.t_utc_s) == n
               and abs(tr.t_utc_s[0] - base_utc) < 1e-3
               and float(np.max(np.abs(e - east))) < 0.05
               and float(np.max(np.abs(u - up))) < 0.05)
        print(f"[self-test] case5a_ulog_globalpos_preferred: source={tr.source} "
              f"utc_synced={tr.utc_synced} n={len(tr.t_utc_s)} "
              f"max_pos_err={float(np.max(np.abs(e - east))):.4f} m  "
              f"{'PASS' if c_a else 'FAIL'}")
        ok = ok and c_a

        # (b/c) raw-GPS fallback, OLD field names, int-scaled -- at a 3 m AMSL
        # site, where a missed /1e3 would read 3000 m and turn a kill into a
        # kilometres-wide MISS with a clean exit 0.
        _install_stub_pyulog({
            "vehicle_gps_position": {"timestamp": t_boot_us, "time_utc_usec": utc_us,
                                     "lat": np.round(lat * 1e7), "lon": np.round(lon * 1e7),
                                     "alt": np.round(alt * 1e3)}})
        tr2 = load_track_from_ulog(Path("stub.ulg"), "interceptor")
        _e2, _n2, u2 = latlon_to_enu(*tr2.latlon.T, lat0, lon0, alt0)
        c_b = (tr2.source == "vehicle_gps_position"
               and float(np.max(np.abs(u2 - up))) < 0.05
               and any("raw vehicle_gps_position" in w for w in tr2.warnings))
        print(f"[self-test] case5b_ulog_rawgps_scaling: alt_err="
              f"{float(np.max(np.abs(u2 - up))):.4f} m (mm-scaled at a "
              f"{alt0 + up[0]:.0f} m AMSL site) warned={bool(tr2.warnings)}  "
              f"{'PASS' if c_b else 'FAIL'}")
        ok = ok and c_b

        # (b') NEW PX4 v1.17 SensorGps field names, already in degrees/metres.
        _install_stub_pyulog({
            "vehicle_gps_position": {"timestamp": t_boot_us, "time_utc_usec": utc_us,
                                     "latitude_deg": lat, "longitude_deg": lon,
                                     "altitude_msl_m": alt}})
        tr3 = load_track_from_ulog(Path("stub.ulg"), "interceptor")
        _e3, _n3, u3 = latlon_to_enu(*tr3.latlon.T, lat0, lon0, alt0)
        c_c = float(np.max(np.abs(u3 - up))) < 0.05
        print(f"[self-test] case5c_ulog_v117_fieldnames: alt_err="
              f"{float(np.max(np.abs(u3 - up))):.4f} m  {'PASS' if c_c else 'FAIL'}")
        ok = ok and c_c

        # (d) no GPS UTC fix -> boot-relative time + a LOUD warning.
        _install_stub_pyulog({
            "vehicle_gps_position": {"timestamp": t_boot_us,
                                     "time_utc_usec": np.zeros(n),
                                     "lat": lat * 1e7, "lon": lon * 1e7,
                                     "alt": alt * 1e3},
            "vehicle_global_position": {"timestamp": t_boot_us, "lat": lat,
                                        "lon": lon, "alt": alt}})
        tr4 = load_track_from_ulog(Path("stub.ulg"), "interceptor")
        c_d = (tr4.utc_synced is False
               and any("boot-relative" in w for w in tr4.warnings)
               and abs(tr4.t_utc_s[0]) < 1e-6)
        print(f"[self-test] case5d_ulog_no_utc_fix: utc_synced={tr4.utc_synced} "
              f"warned={any('boot-relative' in w for w in tr4.warnings)}  "
              f"{'PASS' if c_d else 'FAIL'}")
        ok = ok and c_d

        # (e) neither topic present -> ValueError, never a silent empty track.
        _install_stub_pyulog({})
        try:
            load_track_from_ulog(Path("stub.ulg"), "interceptor")
            print("[self-test] case5e_ulog_no_topics: FAIL (expected ValueError)")
            ok = False
        except ValueError:
            print("[self-test] case5e_ulog_no_topics: PASS (raised ValueError)")
    finally:
        if prev is None:
            sys.modules.pop("pyulog", None)
        else:
            sys.modules["pyulog"] = prev
    return ok


def self_test() -> bool:
    ok = True
    lat0, lon0, alt0 = 34.05, -118.25, 100.0  # arbitrary reference point
    base_utc = 1_752_700_000.0  # arbitrary common UTC epoch shared by both a/c

    def run_case(name, posA0, vA, posB0, vB, duration, hzA, hzB, radius, tol_m, tol_t,
                 dt=0.01, t0_b_offset=0.0, expect=None):
        nonlocal ok
        # Ground truth is computed on the SHARED time axis; a t0 offset on B only
        # changes the sample PHASE (both tracks are linear), never the geometry.
        t_star, cpa_true = _analytic_straight_line_cpa(
            np.array(posA0) - np.array(posB0), np.array(vA) - np.array(vB), 0.0, duration
        )
        track_a = _synthetic_track("interceptor", np.array(posA0, dtype=np.float64),
                                    np.array(vA, dtype=np.float64), duration, hzA,
                                    base_utc, lat0, lon0, alt0)
        # Sample B on its own phase-shifted grid over the SAME world line, so its
        # samples land off A's grid (the adversarial phase the old grid argmin
        # needed to be caught).
        n_b = max(int(round(duration * hzB)) + 1, 2)
        t_rel_b = np.linspace(0.0, duration, n_b) + t0_b_offset
        pos_b = np.array(posB0, dtype=np.float64)[None, :] + \
            t_rel_b[:, None] * np.array(vB, dtype=np.float64)[None, :]
        lat_b, lon_b, alt_b = enu_to_latlon(pos_b[:, 0], pos_b[:, 1], pos_b[:, 2],
                                            lat0, lon0, alt0)
        track_b = Track(label="target", t_utc_s=base_utc + t_rel_b,
                        latlon=np.column_stack([lat_b, lon_b, alt_b]),
                        utc_synced=True, source="synthetic")
        result = score_engagement(track_a, track_b, lethal_radius_m=radius, dt=dt)
        err_m = abs(result.cpa_m - cpa_true)
        err_t = abs(result.cpa_t_utc_s - (base_utc + t_star))
        expect_verdict = expect or ("KILL" if cpa_true <= radius else "MISS")
        pass_ = err_m <= tol_m and err_t <= tol_t and result.verdict == expect_verdict
        print(f"[self-test] {name}: cpa_true={cpa_true:.4f} m  cpa_got={result.cpa_m:.4f} m  "
              f"err={err_m:.4f} m (tol {tol_m})  t_err={err_t:.4f}s (tol {tol_t})  "
              f"grid_argmin={result.cpa_grid_m:.4f} m (dt={result.dt_s:.3f})  "
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

    # Case 2b: THE PRODUCTION dt PATH (dt=None) at PX4's real logging rate, with
    # the CPA deliberately OFF-GRID (2026-07-25, review 2). Every pre-existing
    # case pinned dt=0.01, so the auto-dt path the field actually runs (no caller
    # passes --dt; 04_pull_logs.sh never has) was never exercised -- and its CPA
    # was a grid argmin whose quantization is ~v_rel*dt/2. Geometry: A closes on
    # a stationary B at 12 m/s with a true 0.300 m CPA at t=5.05 s; BOTH log at
    # PX4's default 5 Hz (vehicle_global_position, 200 ms), and B's start is
    # phase-shifted 0.15 s so the CPA lands exactly HALFWAY between resample-grid
    # points -- the adversarial phase. Grid argmin then reads
    # sqrt(0.30^2 + (12*0.1)^2) = 1.237 m = a confident MISS at the ram radius;
    # the analytic CPA returns 0.300 m = KILL, from the same samples.
    #
    # The verdict is asserted against DEFAULT_LETHAL_RADIUS_M (the module
    # constant, never a literal), so a radius regression fails here too.
    run_case(
        "case2b_autodt_offgrid_KILL", posA0=(-60.6, 0.30, 0.0), vA=(12.0, 0.0, 0.0),
        posB0=(0.0, 0.0, 0.0), vB=(0.0, 0.0, 0.0),
        duration=10.0, hzA=5.0, hzB=5.0,
        radius=DEFAULT_LETHAL_RADIUS_M, tol_m=0.01, tol_t=0.02,
        dt=None, t0_b_offset=0.15, expect="KILL",
    )
    # Case 2c: the THRESHOLD case. The analytic CPA (1.050 m) sits STRICTLY
    # between the ram radius (0.35 m, ADR-0084) and the net radius (1.5 m), and the expected
    # verdict is pinned to MISS while the radius is passed as the module CONSTANT
    # -- so if DEFAULT_LETHAL_RADIUS_M ever regresses to the net number this case
    # flips to KILL and FAILS. Same off-grid phase, same auto-dt path.
    run_case(
        "case2c_autodt_offgrid_MISS", posA0=(-60.6, 1.05, 0.0), vA=(12.0, 0.0, 0.0),
        posB0=(0.0, 0.0, 0.0), vB=(0.0, 0.0, 0.0),
        duration=10.0, hzA=5.0, hzB=5.0,
        radius=DEFAULT_LETHAL_RADIUS_M, tol_m=0.01, tol_t=0.02,
        dt=None, t0_b_offset=0.15, expect="MISS",
    )
    # Case 2d: the DEFAULT is the RAM radius and its help text does not mislabel
    # the mechanism. Every other case passes an explicit radius, so the default
    # and its help were untestable-by-construction -- which is how the 1.5 -> 0.5
    # correction left contradicting prose behind a green 5/5 PASS.
    _help = _lethal_radius_help()
    c_default = (DEFAULT_LETHAL_RADIUS_M == 0.35 and "net radius" not in _help.lower()
                 and "0.35" in _help and "adr-0084" in _help.lower())
    print(f"[self-test] case2d_default_radius: DEFAULT_LETHAL_RADIUS_M="
          f"{DEFAULT_LETHAL_RADIUS_M} help mislabels 'net radius'="
          f"{'net radius' in _help.lower()}  {'PASS' if c_default else 'FAIL'}")
    ok = ok and c_default

    # Case 2e: TRUNCATED LOG. A is still closing when the overlap window ends
    # (the true CPA is 2 s beyond it). The minimum range INSIDE the window is a
    # boundary value, not a closest approach, so the verdict must be
    # INCONCLUSIVE_LOG_TRUNCATED -- never a confident MISS at the last sample.
    tr_a = _synthetic_track("interceptor", np.array([-60.0, 0.3, 0.0]),
                            np.array([12.0, 0.0, 0.0]), 4.0, 5.0,
                            base_utc, lat0, lon0, alt0)
    tr_b = _synthetic_track("target", np.zeros(3), np.zeros(3), 4.0, 10.0,
                            base_utc, lat0, lon0, alt0)
    res_tr = score_engagement(tr_a, tr_b, lethal_radius_m=DEFAULT_LETHAL_RADIUS_M)
    c_trunc = (res_tr.verdict == VERDICT_TRUNCATED and res_tr.cpa_at_window_edge
               and res_tr.cpa_edge_side == "end")
    print(f"[self-test] case2e_truncated_log: verdict={res_tr.verdict} "
          f"(expect {VERDICT_TRUNCATED}) edge={res_tr.cpa_edge_side} "
          f"min_range_in_window={res_tr.cpa_m:.3f} m  "
          f"{'PASS' if c_trunc else 'FAIL'}")
    ok = ok and c_trunc

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

    # Case 5: THE ULog READER -- the interceptor's own log, i.e. half the binary
    # kill verdict, which had NO test anywhere (review 2). Driven through a stub
    # `pyulog.ULog` (a .get_dataset(name).data dict), so no byte-level ULog
    # writer is needed. Covers (a) vehicle_global_position PREFERRED, (b) the
    # vehicle_gps_position fallback under BOTH PX4 field generations, (c) the
    # 1e-7 deg / mm scaling, and (d) the no-UTC-fix boot-relative leg.
    ok = ok and _self_test_ulog_reader(lat0, lon0, alt0, base_utc)

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

    ok = _self_test_window_and_guards(lat0, lon0, alt0, base_utc) and ok

    print(f"[self-test] {'PASS' if ok else 'FAIL'}")
    return ok


def _write_csv_track(path, t, enu, tcol):
    """Write a track CSV in the documented east_m/north_m/up_m schema via the
    real csv writer, so the CSV cases below go through load_track_from_csv's
    actual parse rather than a hand-built Track."""
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([tcol, "east_m", "north_m", "up_m"])
        for i in range(len(t)):
            w.writerow([f"{t[i]:.6f}", f"{enu[i, 0]:.6f}",
                        f"{enu[i, 1]:.6f}", f"{enu[i, 2]:.6f}"])


def _self_test_window_and_guards(lat0, lon0, alt0, base_utc) -> bool:
    """The 2026-07-26 review-3 legs: the scoring WINDOW, and the three guards
    that stop this tool printing a confident number it cannot support."""
    import tempfile
    ok = True

    def check(cond, msg):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[self-test] {msg}  {'PASS' if cond else 'FAIL'}")

    # ---- case2f: THE CPA IN THE FINAL dt OF THE OVERLAP -------------------
    # The shape of a SUCCESSFUL ram: the interceptor's log stops within ~0.2 s of
    # contact. Every pre-existing case had a long tail past the CPA (case2b's CPA
    # sits at t=5.05 of a 10 s track -- deep interior), which is exactly why the
    # grid-endpoint window survived review 2. Geometry: A jinks past B at 0.60 m
    # (t=2.0, interior), then comes back and rams at 0.30 m at t=4.95, while the
    # overlap ends at t=5.00. Both tracks log at PX4's default 5 Hz.
    t_a = base_utc + 0.02 + np.arange(27) * 0.2          # [0.02, 5.22]
    t_b = base_utc + np.arange(26) * 0.2                 # [0.00, 5.00]
    tr = t_a - base_utc
    x = np.where(tr <= 2.6, -24.0 + 12.0 * tr,
                 np.where(tr <= 3.7, 7.2 + 12.0 * (tr - 2.6),
                          20.4 - 16.32 * (tr - 3.7)))
    y = np.where(tr <= 3.7, 0.60, 0.30)
    enu_a = np.column_stack([x, y, np.zeros_like(x)])
    lat_a, lon_a, alt_a = enu_to_latlon(enu_a[:, 0], enu_a[:, 1], enu_a[:, 2],
                                        lat0, lon0, alt0)
    track_a = Track(label="interceptor", t_utc_s=t_a,
                    latlon=np.column_stack([lat_a, lon_a, alt_a]),
                    utc_synced=True, source="synthetic")
    lat_b, lon_b, alt_b = enu_to_latlon(np.zeros(26), np.zeros(26), np.zeros(26),
                                        lat0, lon0, alt0)
    track_b = Track(label="target", t_utc_s=t_b,
                    latlon=np.column_stack([lat_b, lon_b, alt_b]),
                    utc_synced=True, source="synthetic")
    got = []
    for d in (None, 0.2, 0.1, 0.01):
        r = score_engagement(track_a, track_b, DEFAULT_LETHAL_RADIUS_M, dt=d)
        got.append((d, r.verdict, round(r.cpa_m, 3)))
    same = len({(v, c) for _d, v, c in got}) == 1
    check(same and got[0][1] == "KILL" and abs(got[0][2] - 0.30) < 0.01,
          f"case2f_cpa_in_final_dt: {got} -- the ram at t=4.95 s is INSIDE the "
          f"overlap that ends at 5.00 s, so it must score KILL 0.300 m at EVERY "
          f"--dt (the grid-endpoint window scored the 0.60 m jink instead)")

    # ---- case2g: A DROPOUT ACROSS THE CPA IS NOT A VERDICT ----------------
    # NaN altitudes (ArduPilot's quiet_nanf when the EKF has no datum) written
    # through the REAL .BIN writer and parsed by the REAL DFReader. The samples
    # must be dropped AND COUNTED, and because the resulting 1 s hole brackets
    # the CPA the verdict must be INCONCLUSIVE_LOG_GAP -- never a confident
    # number interpolated straight across the contact.
    try:
        import pymavlink  # noqa: F401
        have_pymavlink = True
    except ImportError:
        have_pymavlink = False
    if not have_pymavlink:
        print("[self-test] case2g_nonfinite_and_gap: SKIP (pymavlink absent)")
    else:
        with tempfile.TemporaryDirectory() as td:
            dur, hz = 10.0, 25.0
            n = int(dur * hz) + 1
            t_rel = np.linspace(0.0, dur, n)
            pos_b = np.column_stack([np.zeros(n), np.zeros(n), np.zeros(n)])
            hole = (t_rel >= 4.5) & (t_rel <= 5.5)        # 1.04 s, over the CPA
            pos_b[hole, 2] = np.nan
            p = Path(td) / "target_nan.BIN"
            _write_synthetic_bin(p, pos_b, t_rel, base_utc, lat0, lon0, alt0,
                                 with_gps_utc=True)
            tb = load_track_from_bin(p, "target")
            ta = _synthetic_track("interceptor", np.array([-60.6, 0.30, 0.0]),
                                  np.array([12.0, 0.0, 0.0]), dur, 5.0,
                                  base_utc, lat0, lon0, alt0)
            res = score_engagement(ta, tb, DEFAULT_LETHAL_RADIUS_M)
            c = (tb.n_nonfinite_dropped == int(hole.sum())
                 and any("NON-FINITE" in w for w in tb.warnings)
                 and res.verdict == VERDICT_GAPPED)
            check(c, f"case2g_nonfinite_and_gap: dropped "
                     f"{tb.n_nonfinite_dropped}/{tb.n_total_raw} NaN sample(s) "
                     f"(counted, warned={any('NON-FINITE' in w for w in tb.warnings)}) "
                     f"-> verdict {res.verdict} (expect {VERDICT_GAPPED}), "
                     f"gap {max(res.cpa_gap_a_s, res.cpa_gap_b_s):.2f} s could "
                     f"hide {res.cpa_gap_chord_m:.2f} m at 1 g")

            # ALL non-finite -> refuse at LOAD time, never an empty-track verdict.
            pos_all = pos_b.copy()
            pos_all[:, 2] = np.nan
            p2 = Path(td) / "target_allnan.BIN"
            _write_synthetic_bin(p2, pos_all, t_rel, base_utc, lat0, lon0, alt0,
                                 with_gps_utc=True)
            try:
                load_track_from_bin(p2, "target")
                check(False, "case2g2_all_nonfinite: expected a refusal")
            except ValueError as e:
                check("non-finite" in str(e),
                      f"case2g2_all_nonfinite: refused at load "
                      f"({str(e)[:60]}...)")

    # ---- case2h: TWO UNVERIFIABLE CLOCKS ARE NOT AN ALIGNMENT -------------
    # Two boot-relative CSVs describing the SAME trajectory with a 5 s boot skew
    # between the FCs. They overlap by accident (both start near 0), so the
    # no-overlap refusal never fires; the old code printed MISS 25.000 m, exit 0,
    # verdict_uncertain False on a true collision.
    with tempfile.TemporaryDirectory() as td:
        t = np.arange(0.0, 20.0, 0.2)
        path = np.column_stack([-50.0 + 5.0 * t, np.zeros_like(t), np.zeros_like(t)])
        pa, pb = Path(td) / "a.csv", Path(td) / "b.csv"
        _write_csv_track(pa, t, path, "t_s")
        _write_csv_track(pb, t + 5.0, path, "t_s")       # same flight, skewed clock
        ta, tb = load_track_from_csv(pa, "a"), load_track_from_csv(pb, "b")
        r = score_engagement(ta, tb, DEFAULT_LETHAL_RADIUS_M)
        check(r.verdict == VERDICT_UNSYNCED,
              f"case2h_boot_relative_clocks: verdict={r.verdict} "
              f"(expect {VERDICT_UNSYNCED}); the raw number it declines to "
              f"stand behind is {r.cpa_m:.3f} m")
        r2 = score_engagement(ta, tb, DEFAULT_LETHAL_RADIUS_M,
                              assume_clocks_aligned=True)
        check(r2.verdict in ("KILL", "MISS") and r2.clocks_assumed_aligned,
              f"case2h2_escape_hatch: --assume-clocks-aligned restores "
              f"{r2.verdict} and stamps the assumption")
        # A `t_utc_s` HEADER over boot-relative VALUES must not defeat the guard.
        pc = Path(td) / "c.csv"
        _write_csv_track(pc, t, path, "t_utc_s")
        tc = load_track_from_csv(pc, "c")
        check(tc.utc_synced is False
              and any("epoch floor" in w for w in tc.warnings),
              f"case2h3_renamed_column: a 't_utc_s' column whose values are "
              f"below the UTC epoch floor is still boot-relative "
              f"(utc_synced={tc.utc_synced})")

    # ---- case2i: WHAT THE RESIDUAL CLOCK ERROR IS WORTH, IN METRES --------
    # Geometry-aware, not v_closing x dt: displacement along the relative-velocity
    # direction cancels at a minimum, so a HEAD-ON pass is insensitive to a clock
    # shift and a CROSSING pass is not. A naive v_closing x dt band would report
    # the head-on case as the more uncertain of the two -- backwards.
    head_a = _synthetic_track("a", np.array([-60.0, 0.3, 0.0]),
                              np.array([12.0, 0.0, 0.0]), 10.0, 5.0,
                              base_utc, lat0, lon0, alt0)
    head_b = _synthetic_track("b", np.zeros(3), np.zeros(3), 10.0, 5.0,
                              base_utc, lat0, lon0, alt0)
    cross_b = _synthetic_track("b", np.array([0.0, -30.0, 0.0]),
                               np.array([0.0, 6.0, 0.0]), 10.0, 5.0,
                               base_utc, lat0, lon0, alt0)
    s_head = score_engagement(head_a, head_b, DEFAULT_LETHAL_RADIUS_M)
    s_cross = score_engagement(head_a, cross_b, DEFAULT_LETHAL_RADIUS_M)
    check(s_head.cpa_time_sensitivity_m < 0.05
          and s_cross.cpa_time_sensitivity_m > 0.2
          and s_head.uncertainty_band_m >= POSITION_BIAS_FLOOR_M,
          f"case2i_time_sensitivity: head-on "
          f"{s_head.cpa_time_sensitivity_m:.3f} m vs crossing "
          f"{s_cross.cpa_time_sensitivity_m:.3f} m at "
          f"+/-{s_head.time_align_sigma_s:.2f} s (band never narrows below the "
          f"{POSITION_BIAS_FLOOR_M:.1f} m floor: {s_head.uncertainty_band_m:.2f} m)")

    # ---- case2j: THE CLI's ERROR PATH AND THE JSON IT WRITES --------------
    # A corrupt log must produce this tool's own one-line error and exit 1, not a
    # nine-frame traceback at the field; and the report must be STRICT JSON.
    with tempfile.TemporaryDirectory() as td:
        bad = Path(td) / "corrupt.ulg"
        bad.write_bytes(b"")
        import contextlib
        import io
        err = io.StringIO()
        with contextlib.redirect_stderr(err), contextlib.redirect_stdout(io.StringIO()):
            rc = main(["--ulog-a", str(bad), "--ulog-b", str(bad),
                       "--out-dir", str(Path(td) / "out")])
        msg = err.getvalue()
        check(rc == 1 and "[field_score] ERROR" in msg and "Traceback" not in msg,
              f"case2j_corrupt_log_cli: exit {rc} with a clean "
              f"'[field_score] ERROR' line and no traceback")

        out = Path(td) / "out2"
        r = score_engagement(head_a, head_b, DEFAULT_LETHAL_RADIUS_M)
        rep = write_report(r, out, None, None, "strictjson")
        raw = (out / "field_score_strictjson.json").read_text()
        strict = ("NaN" not in raw and "Infinity" not in raw)
        check(strict and json.loads(raw)["horizontal_sep_at_cpa_m"] is not None
              and rep["cpa_delta_enu_m"] is not None,
              f"case2j2_strict_json: no bare NaN/Infinity token in the report, "
              f"and the per-axis CPA breakdown is serialised "
              f"(delta_enu={rep['cpa_delta_enu_m']})")
    return ok


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def main(argv=None) -> int:
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
                     help=_lethal_radius_help())
    ap.add_argument("--dt", type=float, default=None,
                     help="resample grid step, seconds, for the range PLOT and "
                          "the reported range history only (default: auto from "
                          "the finer log's sample rate, clamped to <=0.2 s). "
                          "Since 2026-07-25 the CPA itself is ANALYTIC (exact "
                          "per-segment closed form on the union of both logs' "
                          "sample times) and since 2026-07-26 its window is the "
                          "true log overlap rather than the grid's endpoints, so "
                          "the verdict does not depend on this at all -- pinned "
                          "by self-test case2f.")
    ap.add_argument("--clock-offset-b-s", type=float, default=0.0,
                     help="seconds to ADD to side B's timestamps before scoring "
                          "-- the MEASURED offset between the two logs' time "
                          "bases (e.g. from the protocol's arm-time pair or a "
                          "common trigger event). Recorded in the report.")
    ap.add_argument("--assume-clocks-aligned", action="store_true",
                     help="proceed to a KILL/MISS verdict even when a log has no "
                          "GPS UTC fix, ASSUMING the two flight controllers' boot "
                          "clocks were started together. Without this the verdict "
                          "is INCONCLUSIVE_CLOCKS_UNSYNCED (exit 3): two "
                          "boot-relative clocks always overlap by accident, so the "
                          "alignment is unverifiable. The assumption is stamped "
                          "into the report.")
    ap.add_argument("--time-align-uncertainty-s", type=float,
                     default=DEFAULT_TIME_ALIGN_SIGMA_S,
                     help=f"residual cross-aircraft clock error to bound the CPA "
                          f"against, seconds (default {DEFAULT_TIME_ALIGN_SIGMA_S}; "
                          f"see DEFAULT_TIME_ALIGN_SIGMA_S for the provenance -- "
                          f"it is a GIVEN, not a measurement). The report shows "
                          f"what it is worth in metres on the flown geometry.")
    ap.add_argument("--video-a", default=None, help="path to interceptor seeker/onboard video (metadata only)")
    ap.add_argument("--video-b", default=None, help="path to chase/phone slow-mo video (metadata only)")
    ap.add_argument("--out-dir", type=Path, default=Path("logs/field_score"))
    ap.add_argument("--tag", default=None, help="report filename tag (default: UTC timestamp)")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args(argv)

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

    # TWO CATCHES, NOT ONE (2026-07-26). The narrow one turns an EXPECTED bad
    # input (a corrupt/foreign/empty log) into the clean one-line error the
    # error-handling policy promises -- pyulog raises TypeError on some malformed
    # files and this tool's own PX4-field-rename diagnostic is a KeyError, and
    # neither was in the old tuple, so the operator got a nine-frame traceback.
    # The broad one keeps a BUG IN THE SCORER distinguishable from a bad log, and
    # deliberately still prints the traceback -- suppressing that would make a
    # measurement-code defect harder to see, which is backwards for this project.
    try:
        track_a = load_a(path_a, args.label_a)
        track_b = load_b(path_b, args.label_b)

        for t in (track_a, track_b):
            for w in t.warnings:
                print(f"[warn] {t.label}: {w}", file=sys.stderr)

        result = score_engagement(
            track_a, track_b, args.lethal_radius, dt=args.dt,
            clock_offset_b_s=args.clock_offset_b_s,
            assume_clocks_aligned=args.assume_clocks_aligned,
            time_align_sigma_s=args.time_align_uncertainty_s)
    except (ValueError, TypeError, KeyError, ImportError, OSError) as e:
        print(f"[field_score] ERROR: {type(e).__name__}: {e}", file=sys.stderr)
        print(f"[field_score] could not score {path_a} x {path_b} -- check each "
              f"file is the format its flag claims (--ulog-* wants a PX4 .ulg, "
              f"--bin-* an ArduPilot .BIN, --csv-* a track CSV) and is "
              f"non-empty. NO VERDICT PRODUCED.", file=sys.stderr)
        return 1
    except Exception as e:                       # noqa: BLE001 -- see above
        import traceback
        traceback.print_exc()
        print(f"[field_score] INTERNAL ERROR (this is a bug in field_score, not "
              f"in your log): {type(e).__name__}: {e}", file=sys.stderr)
        return 1

    import datetime
    tag = args.tag or datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    report = write_report(result, args.out_dir, args.video_a, args.video_b, tag)

    print(f"verdict          : {result.verdict}")
    print(f"CPA              : {result.cpa_m:.3f} m  @ t_utc={result.cpa_t_utc_s:.3f}"
          f"   [analytic, exact for the interpolated tracks]")
    print(f"lethal radius    : {result.lethal_radius_m:.3f} m")
    print(f"time overlap     : {result.overlap_s:.3f} s  (resample dt={result.dt_s:.3f} s)")
    # These two lines used to exist only inside the JSON, where nobody reads
    # them on a field day (review 2). verdict_uncertain is what stops a wrong
    # reading being packed up and driven home.
    print(f"grid argmin      : {result.cpa_grid_m:.3f} m  "
          f"(quantization removed by the analytic CPA: "
          f"{abs(result.cpa_grid_m - result.cpa_m):.3f} m)")
    if report.get("horizontal_sep_at_cpa_m") is not None:
        print(f"at CPA           : horizontal {report['horizontal_sep_at_cpa_m']:.3f} m, "
              f"vertical {report['cpa_delta_enu_m'][2]:+.3f} m  "
              f"(the vertical differences TWO independent AMSL datums)")
    print(f"verdict_uncertain: {report['verdict_uncertain']}  "
          f"[uncertainty band +/-{result.uncertainty_band_m:.2f} m = "
          f"{result.uncertainty_band_source}, + clock "
          f"{result.cpa_time_sensitivity_m:.2f} m + dropout "
          f"{result.cpa_gap_chord_m:.2f} m; a CPA within that of the radius is "
          f"not a call -- trust the VIDEO]")
    if result.cpa_at_window_edge:
        print(f"** {VERDICT_TRUNCATED}: still closing at the "
              f"{result.cpa_edge_side} of the logged overlap -- the true CPA is "
              f"OUTSIDE the window. This is not a MISS.", file=sys.stderr)
    if result.verdict in INCONCLUSIVE_VERDICTS:
        # "COULD NOT MEASURE" MUST NOT SHARE AN EXIT CODE WITH PASS. Exit 3 is
        # the shell contract's UNCERTAIN (scripts/field/common.sh); 04_pull_logs.sh
        # branches on it and stops printing PASS over a report that decided
        # nothing. Note this changes INCONCLUSIVE_LOG_TRUNCATED from exit 0 to
        # exit 3 as well -- that path was the same latent defect.
        print(f"** {result.verdict}: this run produced NO KILL/MISS verdict. "
              f"Read 'honesty_note' in {report['json_path']}.", file=sys.stderr)
        return EXIT_UNCERTAIN
    return 0


if __name__ == "__main__":
    sys.exit(main())
