#!/usr/bin/env python3
"""range_truth_join.py -- the MISSING LINK of the tripod field day: write a
per-frame TRUE RANGE into a `pi_capture` session so `tripod_score.py` can bin
curve (a)/(b) at all.

THE DEFECT THIS CLOSES (found 2026-07-24, offline audit). The tripod afternoon
produces the two curves that gate the ~$740 Tier-2 interceptor order
(docs/tripod_test_protocol.md §8.1). `tripod_score.py` needs a TRUE range per
frame -- it reads `index.csv:true_range_m`, else a `_r<num>_` filename token.
`pi_capture.py` writes NEITHER (its INDEX_HEADER is frame_idx/frame_path/
t_mono_s/t_wall_unix/exposure_us/gain), and NOTHING else in the repo produced
`true_range_m`. So every real frame was unbinnable, `n_binnable == 0`, and the
money gate returned `UNCERTAIN: no TRUE ranges in the session` -- no verdict, no
purchase decision, from a field day that cannot be re-run cheaply.

The data to compute the range DOES exist and is already on the §6 record:
  * the target's ArduPilot DataFlash `.BIN` (or a PX4 ULog, or an exported CSV)
    -- the position track, protocol §6 "The target's ULog ... independent range
    ground truth beyond the tag's decode ceiling";
  * a SURVEYED tripod position (protocol §6 "A surveyed/GPS-logged tripod
    position + the target's GPS track gives that");
  * `t_wall_unix` per frame, which `pi_capture` already records precisely so the
    frames can be put on the log's clock (protocol §6.1 time sync).

This tool joins the three and writes `true_range_m` (+ quality columns) back
into `index.csv`. It NEVER destroys an existing column, writes atomically
(temp file + os.replace), and keeps a pristine `.bak`.

HONESTY BOUNDARY (CLAUDE.md constraint `honesty-boundary`): this is OFFLINE
SCORING/CALIBRATION tooling, the same class as `field_score.py` -- it runs
after the aircraft has landed, on recorded logs, to grade what happened. It is
NOT the sim's `gt_*` guidance-cheat (that boundary governs what the FLYING
software may read mid-flight). Nothing here is imported by, or reachable from,
any flight path; the flying seeker never sees a GPS-derived range. Using the
AprilTag decode for the clock cross-check is likewise the tag's sanctioned use
(calibration / auto-labels / staged baseline seeker).

WHAT IT REUSES (no re-implementation -- CLAUDE.md "fix root cause", one loader):
  `scripts/field_score.py` :: load_track_from_bin / load_track_from_ulog /
  load_track_from_csv (DataFlash POS-preferred + UTC recovery, ULog, CSV) and
  latlon_to_enu (equirectangular local tangent plane). Same parser, same
  warnings, same UTC-vs-boot-relative detection as the kill scorer.

THE CLOCK-OFFSET PROBLEM (why `--auto-sync` exists). Frames are stamped with the
Pi's WALL clock; the log is on GPS-UTC. Even an NTP-synced Pi (protocol §6.1)
can sit tens/hundreds of ms off, and a phone-hotspot NTP sync in a field can be
worse. Protocol §6.1's own bound: one 2 m range bin is 0.22 s of flight at
9 m/s, so sync error must stay WELL under that. Two knobs:
  --clock-offset-s   add a KNOWN offset (from the §6.1 marked sync event) to
                     every frame time before querying the log.
  --auto-sync        ESTIMATE the offset from the data: the AprilTag-decoded
                     range (tags.csv) and the log-derived range are the same
                     physical quantity, so the offset that best aligns the two
                     range histories is the clock offset. Robust objective:
                     minimize MAD(R_tag - R_log(t+off)) about its own median, so
                     a constant bias (tag placard vs GPS antenna, camera vs
                     survey point) shifts the reported bias, not the offset.
It REFUSES (non-zero exit) rather than emitting a silently-bad join when: the
log does not overlap the frame span, too few tag ranges to sync on, the residual
is too large (wrong log / wrong tripod position), the constant bias is
implausible (survey or altitude-datum blunder), or the offset is UNIDENTIFIABLE
-- meaning the range RATE barely varies across the tag window. Because the
objective absorbs a constant bias, a straight-in ramp at a constant closing rate
makes a clock error mathematically indistinguishable from a range bias; only a
pass that flies THROUGH a CPA pins the clock. The reported uncertainty is
residual / spread(dR/dt), and it must clear --max-offset-sigma-s.

PER-FRAME QUALITY (written into index.csv so tripod_score can see it):
  range_quality   ok | gap | extrapolated | no_frame_time | low_fix
  range_src_dt_s  seconds to the nearest real log sample (interpolation gap)
  range_sigma_m   per-frame 1-sigma range uncertainty, combining GPS position
                  sigma, the interpolation gap x range-rate, and the clock
                  offset uncertainty x range-rate.
`extrapolated` / `no_frame_time` frames get an EMPTY true_range_m -- unbinnable
by construction, because a fabricated range on those frames is exactly the
silent-bad-join failure this tool exists to prevent.

Usage:
    # ArduPilot target log + surveyed tripod lat/lon/alt, estimate the clock offset
    scripts/seeker/range_truth_join.py --session-dir sessions/pass01 \\
        --bin target_flight.BIN --tripod-lat 34.0501 --tripod-lon -118.2502 \\
        --tripod-alt 104.0 --auto-sync
    # known offset from the §6.1 sync event, PX4 ULog target
    scripts/seeker/range_truth_join.py --session-dir sessions/pass01 \\
        --ulog target.ulg --tripod-lat ... --tripod-lon ... --tripod-alt ... \\
        --clock-offset-s -0.42
    # already-local ENU CSV track + tripod in the same ENU frame
    scripts/seeker/range_truth_join.py --session-dir S --csv track.csv \\
        --tripod-enu 0,0,1.5
    scripts/seeker/range_truth_join.py --self-test   # synthetic .BIN + session, no hardware

Exit codes: 0 = joined, 1 = usage/IO error, 2 = REFUSED (unsafe join).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
REPO_ROOT = os.path.dirname(SCRIPTS)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

# REUSED verbatim from the kill scorer -- one .BIN/ULog/CSV parser for the
# project, one geodesy helper (docstring "WHAT IT REUSES").
from field_score import (  # noqa: E402
    Track,
    enu_to_latlon,
    latlon_to_enu,
    load_track_from_bin,
    load_track_from_csv,
    load_track_from_ulog,
)

# --------------------------------------------------------------------------
# Sourced constants -- every number traces to a doc (CLAUDE.md rule).
# --------------------------------------------------------------------------
# Columns this tool ADDS to index.csv. `true_range_m` is the name tripod_score
# already reads (tripod_score.truth_range_for); the rest are its quality view.
COL_RANGE = "true_range_m"
COL_QUALITY = "range_quality"
COL_DT = "range_src_dt_s"
COL_SIGMA = "range_sigma_m"
ADDED_COLUMNS = [COL_RANGE, COL_QUALITY, COL_DT, COL_SIGMA]

# An interpolation gap wider than this flags the frame `gap`. 0.25 s ~= half of
# protocol §6.1's one-bin time-of-flight (2 m bin / 9 m/s = 0.22 s), i.e. a gap
# big enough to smear a frame across a range bin.
DEFAULT_MAX_GAP_S = 0.25
# Auto-sync search half-window. An NTP-synced Pi should be milliseconds out; a
# hand-set clock can be minutes. 30 s covers a sloppy manual set without letting
# the search wander into a different part of the flight.
DEFAULT_SYNC_WINDOW_S = 30.0
DEFAULT_SYNC_COARSE_S = 0.05      # grid step; refined 10x around the minimum
# Auto-sync acceptance. The residual is a ROBUST SCALE (MAD about the median) of
# (tag range - log range): tag range error at the decode ceiling is ~0.2-0.5 m
# (docs/placard_sizing.md range-accuracy discussion) and GPS adds ~1-3 m
# (field_score.write_report accuracy_caveat), so >2 m of unexplained scatter
# means the two series are not the same engagement.
DEFAULT_MAX_RESIDUAL_M = 2.0
# Constant bias between the two range series absorbs the tag-vs-GPS-antenna and
# camera-vs-survey-point offsets (decimetres). >5 m means a survey or altitude
# DATUM blunder (e.g. AMSL vs AGL) -- refuse rather than bake it in.
DEFAULT_MAX_BIAS_M = 5.0
# Clock-offset uncertainty ceiling: protocol §6.1 requires sync error "well
# under" the 0.22 s one-bin time-of-flight; half a bin (0.10 s) is the bar here.
DEFAULT_MAX_OFFSET_SIGMA_S = 0.10
DEFAULT_MIN_SYNC_SAMPLES = 8      # CLAUDE.md's n>=8 as a floor for any fit
# Fraction of frames that must land inside the log's time span, else REFUSE
# (a log that covers a third of the session cannot truth the session).
DEFAULT_MIN_COVERAGE = 0.5
# Absolute GPS position sigma feeding range_sigma_m. field_score's own
# accuracy_caveat: "inter-receiver bias is ~1-3 m horizontal (worse vertical)".
# 2.0 m is the middle of that documented band; override with --gps-sigma-m.
DEFAULT_GPS_SIGMA_M = 2.0
# GPS fix-quality floor for the `low_fix` flag (ArduPilot GPS.NSats / GPS.HDop).
DEFAULT_MIN_NSATS = 6
DEFAULT_MAX_HDOP = 3.0
# Sanity ceiling on the target's height above the tripod for a tripod test
# (protocol §4 layout: passes at a few tens of metres). A larger median means a
# vertical-datum mismatch, which would corrupt every 3-D range.
IMPLAUSIBLE_UP_M = 150.0

_QUALITY_OK = "ok"
_QUALITY_GAP = "gap"
_QUALITY_EXTRAP = "extrapolated"
_QUALITY_NOTIME = "no_frame_time"
_QUALITY_LOWFIX = "low_fix"
# Flags whose frames carry NO range (unbinnable by construction).
QUALITY_NO_RANGE = (_QUALITY_EXTRAP, _QUALITY_NOTIME)


class Refuse(Exception):
    """Raised when a join would be silently wrong. Maps to exit code 2."""


def _pf(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(f) else f


# --------------------------------------------------------------------------
# Session I/O (pi_capture layout)
# --------------------------------------------------------------------------
def read_index(session_dir):
    """Return (fieldnames, rows) from SESSION/index.csv (pi_capture layout)."""
    path = os.path.join(session_dir, "index.csv")
    if not os.path.exists(path):
        raise Refuse(f"no index.csv in {session_dir} -- not a pi_capture session")
    with open(path, newline="") as f:
        rd = csv.DictReader(f)
        rows = list(rd)
        fieldnames = list(rd.fieldnames or [])
    if not rows:
        raise Refuse(f"{path} has no rows")
    return fieldnames, rows


def frame_wall_times(rows):
    """Per-row Pi wall-clock time (t_wall_unix), NaN where blank.

    `t_wall_unix` is the ONLY column that can be put on the log's UTC axis --
    t_mono_s is a boot-relative monotonic clock (and, for the dir-replay
    backend, synthetic). A session captured with the replay backend therefore
    has no joinable time and is refused up front, not fudged."""
    out = []
    for r in rows:
        v = _pf(r.get("t_wall_unix"))
        out.append(np.nan if v is None else v)
    return np.asarray(out, dtype=float)


def read_tag_ranges(session_dir, rows, tags_csv=None):
    """{row_index: tag_range_m} from a tags.csv, tolerant of BOTH schemas:

      pi_capture  : frame_idx, frame_path, n_tags, ..., range_m
      tripod_score: frame, decoded, tag_range_m, u_px, v_px, n_tags

    Multiple rows per frame (one per tag) collapse to the NEAREST tag, matching
    tripod_score.decode_frames' `min(dets, key=pose_t[2])` rule."""
    path = tags_csv or os.path.join(session_dir, "tags.csv")
    if not os.path.exists(path):
        return {}
    by_idx, by_base = {}, {}
    for i, r in enumerate(rows):
        iv = _pf(r.get("frame_idx"))
        if iv is not None:
            by_idx[int(iv)] = i
        fp = (r.get("frame_path") or "").strip()
        if fp:
            by_base[os.path.splitext(os.path.basename(fp))[0]] = i
    out = {}
    with open(path, newline="") as f:
        for tr in csv.DictReader(f):
            rng = _pf(tr.get("tag_range_m"))
            if rng is None:
                rng = _pf(tr.get("range_m"))
            if rng is None or rng <= 0:
                continue
            i = None
            fn = (tr.get("frame_path") or tr.get("frame") or
                  tr.get("filename") or "").strip()
            if fn:
                i = by_base.get(os.path.splitext(os.path.basename(fn))[0])
            if i is None:
                iv = _pf(tr.get("frame_idx"))
                if iv is not None:
                    i = by_idx.get(int(iv))
            if i is None:
                continue
            if i in out:
                out[i] = min(out[i], rng)   # nearest tag wins
            else:
                out[i] = rng
    return out


def write_index_atomic(session_dir, fieldnames, rows):
    """Rewrite index.csv atomically (temp file + os.replace), preserving every
    original column and keeping a PRISTINE .bak (written once, never
    overwritten by a re-run -- so a second join can't destroy the raw record)."""
    path = os.path.join(session_dir, "index.csv")
    bak = path + ".bak"
    made_bak = False
    if not os.path.exists(bak):
        shutil.copy2(path, bak)
        made_bak = True
    fd, tmp = tempfile.mkstemp(prefix=".index.", suffix=".csv.tmp",
                               dir=os.path.abspath(session_dir))
    try:
        with os.fdopen(fd, "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            wr.writeheader()
            wr.writerows(rows)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return path, bak, made_bak


# --------------------------------------------------------------------------
# Track -> tripod-relative range
# --------------------------------------------------------------------------
def track_range_series(track: Track, tripod_llh=None, tripod_enu=None):
    """Project a field_score Track into the TRIPOD's local ENU frame and return
    (t, enu_Nx3, range_m, up_m). The tripod is the ENU origin for a lat/lon
    track, so the range is just |enu|."""
    if track.mode == "latlon":
        if tripod_llh is None:
            raise Refuse("this track is lat/lon, so the surveyed tripod position "
                         "is required: --tripod-lat/--tripod-lon/--tripod-alt")
        lat0, lon0, alt0 = tripod_llh
        e, n, u = latlon_to_enu(*np.asarray(track.latlon).T, lat0, lon0, alt0)
        enu = np.column_stack([e, n, u])
    else:
        if tripod_enu is None:
            raise Refuse("this track is already local ENU, so the tripod's "
                         "position in the SAME frame is required: --tripod-enu E,N,U")
        enu = np.asarray(track.enu, dtype=float) - np.asarray(tripod_enu, dtype=float)
    rng = np.linalg.norm(enu, axis=1)
    return np.asarray(track.t_utc_s, dtype=float), enu, rng, enu[:, 2]


def interp_positions(t_query, t_track, enu_track):
    """Interpolate the ENU track (component-wise, then norm -- not the range
    directly) at the query times. Returns (range_m, nearest_sample_dt_s,
    inside_span_mask)."""
    t_query = np.asarray(t_query, dtype=float)
    inside = (t_query >= t_track[0]) & (t_query <= t_track[-1]) & np.isfinite(t_query)
    comps = [np.interp(t_query, t_track, enu_track[:, k]) for k in range(3)]
    rng = np.linalg.norm(np.column_stack(comps), axis=1)
    idx = np.searchsorted(t_track, t_query)
    idx = np.clip(idx, 1, len(t_track) - 1)
    dt = np.minimum(np.abs(t_query - t_track[idx - 1]),
                    np.abs(t_track[idx] - t_query))
    return rng, dt, inside


def range_rate_series(t_track, rng):
    """SIGNED dR/dt along the track. Magnitude drives the per-frame range
    uncertainty (a timing error costs rate x dt of range); its SPREAD drives
    whether the clock offset is identifiable at all (see offset_identifiability)."""
    if len(t_track) < 2:
        return np.zeros_like(rng)
    return np.gradient(rng, t_track)


def offset_identifiability(rate_at_frames):
    """Robust SPREAD (1.4826 x MAD ~ sigma) of dR/dt across the sync window.

    WHY THE SPREAD AND NOT THE RATE ITSELF: the sync objective absorbs a CONSTANT
    bias (tag placard vs GPS antenna, tripod-survey error, GPS datum -- metres,
    per field_score's accuracy_caveat), so to first order a clock error shifts
    the residual by `off_err * (dR/dt - mean(dR/dt))`. On a straight-in run with a
    near-CONSTANT range rate that term vanishes: a timing error and a constant
    bias are then INDISTINGUISHABLE, and a confident-looking offset would be
    fabricated. Only a CHANGING range rate (a pass with a real CPA -- the
    geometry protocol §4 actually flies) makes the offset observable."""
    r = np.asarray(rate_at_frames, dtype=float)
    if r.size < 2:
        return 0.0
    return float(1.4826 * np.median(np.abs(r - np.median(r))))


# --------------------------------------------------------------------------
# GPS fix quality (best-effort; .BIN + CSV only)
# --------------------------------------------------------------------------
def load_gps_quality_from_bin(path):
    """(t, nsats, hdop) from the DataFlash GPS message, or None. Separate pass
    from field_score.load_track_from_bin (which returns POS-preferred position
    only and does not carry fix quality)."""
    try:
        from pymavlink import DFReader
    except ImportError:
        return None
    try:
        reader = DFReader.DFReader_binary(str(path))
    except Exception:
        return None
    ts, ns, hd = [], [], []
    while True:
        m = reader.recv_match(type=["GPS"])
        if m is None:
            break
        nsat = getattr(m, "NSats", None)
        hdop = getattr(m, "HDop", None)
        ts.append(float(m._timestamp))
        ns.append(float(nsat) if nsat is not None else np.nan)
        hd.append(float(hdop) if hdop is not None else np.nan)
    if not ts:
        return None
    o = np.argsort(np.asarray(ts))
    return (np.asarray(ts)[o], np.asarray(ns)[o], np.asarray(hd)[o])


def _nearest(t_query, t_ref, values):
    if t_ref is None or len(t_ref) == 0:
        return np.full(len(t_query), np.nan)
    idx = np.clip(np.searchsorted(t_ref, t_query), 0, len(t_ref) - 1)
    idx_lo = np.clip(idx - 1, 0, len(t_ref) - 1)
    pick = np.where(np.abs(t_ref[idx] - t_query) <= np.abs(t_query - t_ref[idx_lo]),
                    idx, idx_lo)
    return values[pick]


# --------------------------------------------------------------------------
# Clock offset: --auto-sync
# --------------------------------------------------------------------------
def _sync_objective(offset, t_frames, r_tag, t_track, enu_track, min_samples):
    """(robust residual, constant bias, n_used) at one candidate offset.

    residual = MAD-style robust scale of (R_tag - R_log) about its own median,
    so a CONSTANT bias (tag placard vs GPS antenna; camera vs survey point) is
    reported separately instead of polluting the timing estimate."""
    r_log, _dt, inside = interp_positions(t_frames + offset, t_track, enu_track)
    if inside.sum() < min_samples:
        return float("inf"), float("nan"), int(inside.sum())
    diff = r_tag[inside] - r_log[inside]
    bias = float(np.median(diff))
    resid = float(np.median(np.abs(diff - bias)))
    return resid, bias, int(inside.sum())


def auto_sync(t_frames, r_tag, t_track, enu_track, rng_track,
              window_s=DEFAULT_SYNC_WINDOW_S, coarse_s=DEFAULT_SYNC_COARSE_S,
              min_samples=DEFAULT_MIN_SYNC_SAMPLES,
              max_residual_m=DEFAULT_MAX_RESIDUAL_M,
              max_bias_m=DEFAULT_MAX_BIAS_M,
              max_offset_sigma_s=DEFAULT_MAX_OFFSET_SIGMA_S,
              seed_offset=0.0):
    """Estimate (and VET) the Pi-wall-clock -> log-UTC offset.

    Returns a dict with offset_s, residual_m, bias_m, offset_sigma_s,
    sensitivity_mps, n_used, grid_median_residual_m. Raises Refuse when the
    offset cannot be resolved honestly. `seed_offset` centres the search (use a
    coarse hand offset when the Pi clock was never NTP'd)."""
    if len(t_frames) < min_samples:
        raise Refuse(
            f"--auto-sync needs >={min_samples} frames with BOTH a tag range and "
            f"a t_wall_unix; got {len(t_frames)}. Re-run with an explicit "
            f"--clock-offset-s from the protocol §6.1 sync event.")

    grid = np.arange(seed_offset - window_s, seed_offset + window_s + coarse_s, coarse_s)
    res = np.full(len(grid), np.inf)
    for i, off in enumerate(grid):
        res[i], _b, _n = _sync_objective(off, t_frames, r_tag, t_track,
                                         enu_track, min_samples)
    if not np.isfinite(res).any():
        raise Refuse(
            "--auto-sync found NO offset in the search window that puts >= "
            f"{min_samples} tag-ranged frames inside the log's time span -- the "
            "log almost certainly does not cover this pass (or the clock is off "
            "by more than the +/-%.0f s window; widen --sync-window-s)." % window_s)
    k = int(np.nanargmin(res))
    # Refine 10x around the coarse minimum.
    fine = np.arange(grid[k] - coarse_s, grid[k] + coarse_s + coarse_s / 10.0,
                     coarse_s / 10.0)
    best_off, best_res, best_bias, best_n = grid[k], res[k], float("nan"), 0
    for off in fine:
        r, b, n = _sync_objective(off, t_frames, r_tag, t_track, enu_track, min_samples)
        if r < best_res:
            best_off, best_res, best_bias, best_n = float(off), r, b, n
    if not math.isfinite(best_res):
        raise Refuse("--auto-sync: no finite residual at the refined minimum")
    if best_n == 0:
        _r, best_bias, best_n = _sync_objective(best_off, t_frames, r_tag,
                                                t_track, enu_track, min_samples)

    # Boundary hit -> the true offset is outside the window (unresolved), not at
    # its edge. Refuse instead of clamping.
    if abs(best_off - grid[0]) <= coarse_s or abs(best_off - grid[-1]) <= coarse_s:
        raise Refuse(
            f"--auto-sync minimum landed on the search-window EDGE "
            f"(offset {best_off:+.3f} s, window +/-{window_s:.0f} s around "
            f"{seed_offset:+.3f} s) -- the real offset is outside the window. "
            f"Widen --sync-window-s or supply --clock-offset-s.")

    # IDENTIFIABILITY: with a constant bias absorbed, only a CHANGING range rate
    # pins the clock. sigma_offset ~ residual / spread(dR/dt) over the window.
    r_log, _dt, inside = interp_positions(t_frames + best_off, t_track, enu_track)
    rate_track = range_rate_series(t_track, rng_track)
    rate_at_frames = (np.interp(t_frames[inside] + best_off, t_track, rate_track)
                      if inside.sum() else np.zeros(0))
    sens = offset_identifiability(rate_at_frames)
    rate_med = float(np.median(np.abs(rate_at_frames))) if rate_at_frames.size else 0.0
    off_sigma = (best_res / sens) if sens > 1e-6 else float("inf")

    info = {
        "offset_s": round(float(best_off), 4),
        "residual_m": round(float(best_res), 4),
        "bias_m": round(float(best_bias), 4),
        "sensitivity_mps": round(sens, 4),
        "median_abs_range_rate_mps": round(rate_med, 4),
        "offset_sigma_s": (round(off_sigma, 4) if math.isfinite(off_sigma) else None),
        "n_used": int(best_n),
        "grid_median_residual_m": round(float(np.nanmedian(res[np.isfinite(res)])), 4),
        "search_window_s": window_s,
    }
    # ORDER MATTERS: (1) residual = "is this even the same engagement?";
    # (2) identifiability = "could this geometry pin a clock at all?" -- it must
    # precede the bias test, because a flat objective valley aliases a timing
    # error into the absorbed bias, and "UNIDENTIFIABLE geometry" is the true
    # diagnosis there, not "survey blunder"; (3) bias = the survey/datum test.
    if best_res > max_residual_m:
        raise Refuse(
            f"--auto-sync residual {best_res:.2f} m > --max-residual-m "
            f"{max_residual_m:.2f} m at the best offset ({best_off:+.3f} s). The "
            f"tag-derived and log-derived ranges do not describe the same "
            f"engagement -- wrong log file, wrong pass, or a bad tripod survey. "
            f"REFUSING to write a join. (bias {best_bias:+.2f} m, n={best_n})")
    if not math.isfinite(off_sigma) or off_sigma > max_offset_sigma_s:
        raise Refuse(
            f"--auto-sync offset is UNIDENTIFIABLE: the target's range RATE varies "
            f"by only {sens:.3f} m/s (robust spread) across the tag window "
            f"(median |dR/dt| {rate_med:.3f} m/s), so the residual {best_res:.2f} m "
            f"corresponds to a clock uncertainty of "
            f"{off_sigma if math.isfinite(off_sigma) else float('inf'):.3f} s "
            f"(> --max-offset-sigma-s {max_offset_sigma_s:.2f} s; protocol §6.1 "
            f"needs sync well inside a 2 m bin = 0.22 s at 9 m/s). A CONSTANT range "
            f"rate is indistinguishable from a constant range bias -- sync on a pass "
            f"that flies THROUGH a CPA (the range rate reverses), or supply "
            f"--clock-offset-s from the §6.1 marked sync event.")
    if abs(best_bias) > max_bias_m:
        raise Refuse(
            f"--auto-sync constant bias {best_bias:+.2f} m exceeds "
            f"--max-bias-m {max_bias_m:.2f} m. A metre-scale bias is expected "
            f"(tag placard vs GPS antenna, camera vs survey point); THIS is a "
            f"survey or ALTITUDE-DATUM blunder (e.g. tripod alt given AGL while "
            f"the log logs AMSL). REFUSING. (residual {best_res:.2f} m)")
    return info


# --------------------------------------------------------------------------
# The join
# --------------------------------------------------------------------------
def join_session(session_dir, track, tripod_llh=None, tripod_enu=None,
                 clock_offset_s=0.0, do_auto_sync=False, tags_csv=None,
                 max_gap_s=DEFAULT_MAX_GAP_S, min_coverage=DEFAULT_MIN_COVERAGE,
                 gps_sigma_m=DEFAULT_GPS_SIGMA_M, gps_quality=None,
                 min_nsats=DEFAULT_MIN_NSATS, max_hdop=DEFAULT_MAX_HDOP,
                 sync_kwargs=None, dry_run=False, quiet=False):
    """Join `track` to the session's frames and write true_range_m back.
    Returns the provenance dict (also written to range_truth.json)."""
    fieldnames, rows = read_index(session_dir)
    t_frames_all = frame_wall_times(rows)
    n_rows = len(rows)
    n_timed = int(np.isfinite(t_frames_all).sum())
    if n_timed == 0:
        raise Refuse(
            "no frame carries a t_wall_unix -- nothing can be put on the log's "
            "UTC axis. (The pi_capture `dir=` REPLAY backend leaves t_wall_unix "
            "blank by design; a field session must be captured with the "
            "picamera2/v4l2 backend.)")

    t_track, enu_track, rng_track, up_track = track_range_series(
        track, tripod_llh=tripod_llh, tripod_enu=tripod_enu)
    if len(t_track) < 2:
        raise Refuse(f"target track has only {len(t_track)} sample(s)")
    if not track.utc_synced:
        raise Refuse(
            "the target log has NO GPS UTC fix -- its timestamps are "
            "boot-relative, so they cannot be aligned to the frames' wall clock. "
            "(field_score tolerates this for a two-log CPA because both logs are "
            "boot-relative; here one side is a wall clock.) Supply an exported "
            "CSV with a real t_utc_s column, or a log with a GPS time fix.")

    med_up = float(np.median(up_track))
    warn_alt = abs(med_up) > IMPLAUSIBLE_UP_M

    # ---- clock offset ----
    sync_info = None
    tag_ranges = read_tag_ranges(session_dir, rows, tags_csv=tags_csv)
    if do_auto_sync:
        idx = [i for i in sorted(tag_ranges)
               if i < n_rows and np.isfinite(t_frames_all[i])]
        tf = t_frames_all[idx]
        rt = np.asarray([tag_ranges[i] for i in idx], dtype=float)
        sync_info = auto_sync(tf, rt, t_track, enu_track, rng_track,
                              seed_offset=float(clock_offset_s),
                              **(sync_kwargs or {}))
        clock_offset_s = sync_info["offset_s"]
        offset_sigma_s = sync_info["offset_sigma_s"] or 0.0
    else:
        offset_sigma_s = 0.0

    # ---- interpolate ----
    t_q = t_frames_all + float(clock_offset_s)
    rng_q, dt_q, inside = interp_positions(t_q, t_track, enu_track)
    have_time = np.isfinite(t_frames_all)
    inside = inside & have_time
    coverage = float(inside.sum()) / float(n_timed)
    if inside.sum() == 0:
        raise Refuse(
            f"ZERO frames fall inside the log's time span after the clock offset "
            f"({clock_offset_s:+.3f} s). Frames span "
            f"[{np.nanmin(t_frames_all):.3f}, {np.nanmax(t_frames_all):.3f}]; log "
            f"spans [{t_track[0]:.3f}, {t_track[-1]:.3f}] "
            f"({t_track[-1] - t_track[0]:.1f} s). Wrong log, wrong pass, or an "
            f"unresolved clock offset -- REFUSING to fabricate ranges.")
    if coverage < min_coverage:
        raise Refuse(
            f"the log covers only {100 * coverage:.1f}% of the timed frames "
            f"(< --min-coverage {100 * min_coverage:.0f}%). A partial log cannot "
            f"truth this session; trim the session, fix the offset, or lower "
            f"--min-coverage deliberately.")

    rate_track = np.abs(range_rate_series(t_track, rng_track))
    rate_q = np.interp(np.clip(t_q, t_track[0], t_track[-1]), t_track, rate_track)

    # ---- fix quality (optional) ----
    nsats = hdop = None
    if gps_quality is not None:
        tq, ns, hd = gps_quality
        nsats = _nearest(np.clip(t_q, tq[0], tq[-1]), tq, ns)
        hdop = _nearest(np.clip(t_q, tq[0], tq[-1]), tq, hd)

    # ---- per-frame write-back ----
    counts = {_QUALITY_OK: 0, _QUALITY_GAP: 0, _QUALITY_EXTRAP: 0,
              _QUALITY_NOTIME: 0, _QUALITY_LOWFIX: 0}
    sigmas = []
    for i, r in enumerate(rows):
        if not have_time[i]:
            r[COL_RANGE] = ""
            r[COL_QUALITY] = _QUALITY_NOTIME
            r[COL_DT] = ""
            r[COL_SIGMA] = ""
            counts[_QUALITY_NOTIME] += 1
            continue
        if not inside[i]:
            r[COL_RANGE] = ""
            r[COL_QUALITY] = _QUALITY_EXTRAP
            r[COL_DT] = f"{dt_q[i]:.4f}"
            r[COL_SIGMA] = ""
            counts[_QUALITY_EXTRAP] += 1
            continue
        q = _QUALITY_OK
        if dt_q[i] > max_gap_s:
            q = _QUALITY_GAP
        if nsats is not None and np.isfinite(nsats[i]) and nsats[i] < min_nsats:
            q = _QUALITY_LOWFIX
        if hdop is not None and np.isfinite(hdop[i]) and hdop[i] > max_hdop:
            q = _QUALITY_LOWFIX
        # 1-sigma range uncertainty: absolute GPS + interpolation gap x rate +
        # clock-offset uncertainty x rate (independent -> quadrature).
        sig = math.sqrt(gps_sigma_m ** 2
                        + (rate_q[i] * dt_q[i]) ** 2
                        + (rate_q[i] * offset_sigma_s) ** 2)
        r[COL_RANGE] = f"{rng_q[i]:.3f}"
        r[COL_QUALITY] = q
        r[COL_DT] = f"{dt_q[i]:.4f}"
        r[COL_SIGMA] = f"{sig:.3f}"
        counts[q] += 1
        sigmas.append(sig)

    out_fields = list(fieldnames) + [c for c in ADDED_COLUMNS if c not in fieldnames]
    n_written = counts[_QUALITY_OK] + counts[_QUALITY_GAP] + counts[_QUALITY_LOWFIX]

    prov = {
        "tool": "scripts/seeker/range_truth_join.py",
        "session_dir": os.path.abspath(session_dir),
        "n_frames": n_rows,
        "n_frames_timed": n_timed,
        "n_true_range_written": n_written,
        "quality_counts": counts,
        "coverage_frac": round(coverage, 4),
        "clock_offset_s": round(float(clock_offset_s), 4),
        "clock_offset_source": ("auto-sync (tag vs log range)" if do_auto_sync
                                else "explicit --clock-offset-s"),
        "auto_sync": sync_info,
        "track_source": track.source,
        "track_label": track.label,
        "track_n_samples": int(len(t_track)),
        "track_span_utc_s": [float(t_track[0]), float(t_track[-1])],
        "track_utc_synced": bool(track.utc_synced),
        "track_warnings": list(track.warnings),
        "tripod_llh": list(tripod_llh) if tripod_llh else None,
        "tripod_enu": list(tripod_enu) if tripod_enu is not None else None,
        "median_target_up_m": round(med_up, 3),
        "gps_sigma_m": gps_sigma_m,
        "range_sigma_median_m": (round(float(np.median(sigmas)), 3) if sigmas else None),
        "range_sigma_p90_m": (round(float(np.percentile(sigmas, 90)), 3) if sigmas else None),
        "max_gap_s": max_gap_s,
        "range_span_m": ([round(float(np.nanmin(rng_q[inside])), 3),
                          round(float(np.nanmax(rng_q[inside])), 3)] if inside.any() else None),
        "honesty": ("OFFLINE post-flight scoring/calibration (field_score class). "
                    "No flight path reads this; not the sim's gt_* boundary."),
        "warnings": ([] if not warn_alt else [
            f"median target height above the tripod is {med_up:.1f} m -- "
            f"implausible for a tripod pass; check the ALTITUDE DATUM "
            f"(--tripod-alt must match the log's alt datum, typically AMSL)."]),
        "dry_run": bool(dry_run),
    }

    if not dry_run:
        path, bak, made_bak = write_index_atomic(session_dir, out_fields, rows)
        prov["index_csv"] = path
        prov["backup_csv"] = bak
        prov["backup_created_this_run"] = made_bak
        with open(os.path.join(session_dir, "range_truth.json"), "w") as f:
            json.dump(prov, f, indent=2)
    if not quiet:
        _print_report(prov)
    return prov


def _print_report(prov):
    print("=" * 70)
    print("RANGE-TRUTH JOIN — per-frame true_range_m for the tripod scorer")
    print("=" * 70)
    print(f"  session       : {prov['session_dir']}")
    print(f"  target track  : {prov['track_source']} "
          f"({prov['track_n_samples']} samples, "
          f"{prov['track_span_utc_s'][1] - prov['track_span_utc_s'][0]:.1f} s span)")
    for w in prov["track_warnings"]:
        print(f"    [warn] {w}")
    print(f"  clock offset  : {prov['clock_offset_s']:+.4f} s "
          f"[{prov['clock_offset_source']}]")
    s = prov.get("auto_sync")
    if s:
        print(f"    residual {s['residual_m']:.3f} m (robust MAD)  "
              f"bias {s['bias_m']:+.3f} m  n={s['n_used']}")
        print(f"    identifiability: dR/dt spread {s['sensitivity_mps']:.3f} m/s "
              f"(median |dR/dt| {s['median_abs_range_rate_mps']:.3f} m/s) -> "
              f"offset sigma {s['offset_sigma_s']} s  "
              f"(grid-median residual {s['grid_median_residual_m']:.3f} m)")
    print(f"  frames        : {prov['n_frames']} total, {prov['n_frames_timed']} timed, "
          f"{prov['n_true_range_written']} given a true_range_m "
          f"({100 * prov['coverage_frac']:.1f}% inside the log span)")
    print(f"  quality       : {prov['quality_counts']}")
    if prov["range_span_m"]:
        print(f"  true range    : {prov['range_span_m'][0]:.2f} .. "
              f"{prov['range_span_m'][1]:.2f} m")
    print(f"  uncertainty   : median sigma {prov['range_sigma_median_m']} m, "
          f"p90 {prov['range_sigma_p90_m']} m "
          f"(GPS sigma {prov['gps_sigma_m']} m + gap*rate + offset_sigma*rate)")
    for w in prov["warnings"]:
        print(f"  [WARN] {w}")
    if prov.get("dry_run"):
        print("  DRY RUN — index.csv NOT modified")
    else:
        print(f"  wrote         : {prov['index_csv']}  "
              f"(pristine backup: {prov['backup_csv']}"
              f"{' [created now]' if prov.get('backup_created_this_run') else ' [pre-existing]'})")
        print(f"  provenance    : {os.path.join(prov['session_dir'], 'range_truth.json')}")
    print("  NEXT: scripts/seeker/tripod_score.py SESSION --calib calib.json "
          "--stream-fps <MEASURED>")


# --------------------------------------------------------------------------
# Self-test: synthetic .BIN byte stream + synthetic pi_capture session
# --------------------------------------------------------------------------
def _synth_session(dirpath, t_wall, tag_ranges=None, index_header=None,
                   tags_header=None):
    """Write a pi_capture-LAYOUT session (real INDEX_HEADER, real filenames,
    frames/ present) with the given per-frame wall times. tag_ranges (or None
    per frame) become a pi_capture-schema tags.csv."""
    frames_dir = os.path.join(dirpath, "frames")
    os.makedirs(frames_dir, exist_ok=True)
    hdr = list(index_header)
    with open(os.path.join(dirpath, "index.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(hdr)
        for i, tw in enumerate(t_wall):
            rel = os.path.join("frames", f"{i:06d}.png")
            open(os.path.join(dirpath, rel), "wb").close()
            wr.writerow([i, rel, f"{i * 0.05:.6f}", f"{tw:.6f}", "", ""])
    if tag_ranges is not None:
        with open(os.path.join(dirpath, "tags.csv"), "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(list(tags_header))
            for i, r in enumerate(tag_ranges):
                rel = os.path.join("frames", f"{i:06d}.png")
                if r is None:
                    wr.writerow([i, rel, 0, "", "", "", "", "", "", ""])
                else:
                    wr.writerow([i, rel, 1, 7, "42.000", 0, "640.0", "400.0",
                                 "0:0;1:0;1:1;0:1", f"{r:.4f}"])
    return dirpath


def self_test():
    import tempfile as _tf
    sys.path.insert(0, HERE)
    from field_score import _write_synthetic_bin  # self-test fixture, reused
    from pi_capture import INDEX_HEADER, TAGS_HEADER

    ok = True

    def check(cond, msg):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[self-test] {'OK  ' if cond else 'FAIL'} {msg}")

    lat0, lon0, alt0 = 34.0500, -118.2500, 100.0     # surveyed tripod
    base_utc = 1_752_700_000.0
    # A realistic tripod PASS (protocol §4): the target flies past the tripod at
    # an 8 m cross-range offset, 8 m up, 4.5 m/s -- so the range sweeps IN to a
    # CPA and back OUT. The reversing range rate is what makes the clock offset
    # identifiable at all (see offset_identifiability); a straight-in ramp is
    # correctly refused, and case 3b below proves it.
    dur, hz = 12.0, 25.0
    n = int(dur * hz) + 1
    t_rel = np.linspace(0.0, dur, n)
    north = 40.0 - 4.5 * t_rel            # closing in +N, passes through 0
    pos = np.column_stack([np.full(n, 8.0), north, np.full(n, 8.0)])
    true_rng = np.linalg.norm(pos, axis=1)

    # Frames at 20 Hz over the middle of the pass, on a Pi clock offset by a
    # KNOWN amount (the thing --auto-sync must recover).
    TRUE_OFFSET = -0.37            # t_utc = t_wall + TRUE_OFFSET
    n_fr = 120
    # Frames cover the CLOSE half of the pass, t=6..11.95 s: through the CPA at
    # t=8.9 s (range ~11.3 m) and out again -- the band where a real placard
    # decodes and where the range rate reverses.
    t_utc_frames = base_utc + np.linspace(6.0, 6.0 + (n_fr - 1) / 20.0, n_fr)
    t_wall_frames = t_utc_frames - TRUE_OFFSET

    # Tag "decodes" (nearest 60 frames) with a realistic +0.25 m bias & noise.
    rng_at_frames = np.interp(t_utc_frames - base_utc, t_rel, true_rng)
    rs = np.random.default_rng(7)
    tag_r = [None] * n_fr
    for i in range(n_fr):
        if rng_at_frames[i] <= 30.0:
            tag_r[i] = float(rng_at_frames[i] + 0.25 + rs.normal(0, 0.12))

    with _tf.TemporaryDirectory() as td:
        bin_path = os.path.join(td, "target.BIN")
        _write_synthetic_bin(bin_path, pos, t_rel, base_utc, lat0, lon0, alt0,
                             with_gps_utc=True)
        track = load_track_from_bin(Path(bin_path), "target")
        check(track.source == "dataflash:POS" and track.utc_synced,
              f"synthetic .BIN parsed through the REAL DFReader "
              f"({track.source}, utc_synced={track.utc_synced}, n={len(track.t_utc_s)})")

        # ---- 1. explicit offset join ----
        s1 = _synth_session(os.path.join(td, "s1"), t_wall_frames, tag_r,
                            INDEX_HEADER, TAGS_HEADER)
        prov = join_session(s1, track, tripod_llh=(lat0, lon0, alt0),
                            clock_offset_s=TRUE_OFFSET, quiet=True)
        with open(os.path.join(s1, "index.csv"), newline="") as f:
            rows1 = list(csv.DictReader(f))
        check(all(c in rows1[0] for c in INDEX_HEADER),
              "join PRESERVED every original pi_capture column")
        check(all(c in rows1[0] for c in ADDED_COLUMNS),
              f"join ADDED {ADDED_COLUMNS}")
        got = np.asarray([float(r[COL_RANGE]) for r in rows1 if r[COL_RANGE]])
        want = rng_at_frames[:len(got)]
        err = float(np.max(np.abs(got - want)))
        check(len(got) == n_fr and err < 0.02,
              f"true_range_m matches the analytic range on all {len(got)} frames "
              f"(max err {err:.4f} m < 0.02)")
        check(os.path.exists(os.path.join(s1, "index.csv.bak")),
              "pristine index.csv.bak kept")
        check(not [p for p in os.listdir(s1) if p.endswith(".tmp")],
              "atomic write left no temp file")
        check(prov["quality_counts"][_QUALITY_OK] == n_fr,
              f"all {n_fr} frames flagged '{_QUALITY_OK}' "
              f"({prov['quality_counts']})")

        # ---- 2. --auto-sync recovers the injected offset ----
        s2 = _synth_session(os.path.join(td, "s2"), t_wall_frames, tag_r,
                            INDEX_HEADER, TAGS_HEADER)
        prov2 = join_session(s2, track, tripod_llh=(lat0, lon0, alt0),
                             do_auto_sync=True, quiet=True)
        est = prov2["auto_sync"]["offset_s"]
        check(abs(est - TRUE_OFFSET) < 0.05,
              f"--auto-sync recovered {est:+.4f} s vs true {TRUE_OFFSET:+.4f} s "
              f"(err {abs(est - TRUE_OFFSET) * 1000:.1f} ms < 50 ms); residual "
              f"{prov2['auto_sync']['residual_m']:.3f} m, bias "
              f"{prov2['auto_sync']['bias_m']:+.3f} m (injected +0.25 m), "
              f"sigma {prov2['auto_sync']['offset_sigma_s']} s")

        # ---- 3. REFUSALS ----
        # (a) log does not overlap the frames at all
        s3 = _synth_session(os.path.join(td, "s3"), t_wall_frames + 5000.0,
                            tag_r, INDEX_HEADER, TAGS_HEADER)
        try:
            join_session(s3, track, tripod_llh=(lat0, lon0, alt0), quiet=True)
            check(False, "no-overlap log must REFUSE")
        except Refuse as e:
            check("ZERO frames" in str(e) or "cover" in str(e),
                  f"no-overlap log REFUSED ({str(e)[:60]}...)")

        # (b1) unidentifiable offset: target HOVERS (range ~constant)
        pos_h = np.column_stack([np.zeros(n), np.full(n, 20.0), np.full(n, 8.0)])
        bin_h = os.path.join(td, "hover.BIN")
        _write_synthetic_bin(bin_h, pos_h, t_rel, base_utc, lat0, lon0, alt0, True)
        track_h = load_track_from_bin(Path(bin_h), "target")
        rh = float(np.linalg.norm([0, 20.0, 8.0]))
        tag_h = [rh + 0.25 + rs.normal(0, 0.12) for _ in range(n_fr)]
        s4 = _synth_session(os.path.join(td, "s4"), t_wall_frames, tag_h,
                            INDEX_HEADER, TAGS_HEADER)
        try:
            join_session(s4, track_h, tripod_llh=(lat0, lon0, alt0),
                         do_auto_sync=True, quiet=True)
            check(False, "hovering target must make --auto-sync REFUSE")
        except Refuse as e:
            check("UNIDENTIFIABLE" in str(e),
                  f"hover (no range rate) -> --auto-sync REFUSED ({str(e)[:60]}...)")

        # (b2) unidentifiable offset: STRAIGHT-IN run at a constant closing rate.
        # The whole point of offset_identifiability -- median |dR/dt| is a healthy
        # ~4 m/s here, but its SPREAD is tiny, so a clock error is aliased into
        # the absorbed constant bias. Must refuse, not return a confident number.
        pos_s = np.column_stack([np.zeros(n), 60.0 - 4.5 * t_rel, np.full(n, 3.0)])
        bin_s = os.path.join(td, "straight.BIN")
        _write_synthetic_bin(bin_s, pos_s, t_rel, base_utc, lat0, lon0, alt0, True)
        track_s = load_track_from_bin(Path(bin_s), "target")
        rng_s = np.interp(t_utc_frames - base_utc, t_rel,
                          np.linalg.norm(pos_s, axis=1))
        tag_s = [float(v + 0.25 + rs.normal(0, 0.12)) for v in rng_s]
        s4b = _synth_session(os.path.join(td, "s4b"), t_wall_frames, tag_s,
                             INDEX_HEADER, TAGS_HEADER)
        try:
            join_session(s4b, track_s, tripod_llh=(lat0, lon0, alt0),
                         do_auto_sync=True, quiet=True)
            check(False, "constant-rate straight-in pass must make --auto-sync REFUSE")
        except Refuse as e:
            check("UNIDENTIFIABLE" in str(e),
                  f"straight-in constant rate -> --auto-sync REFUSED "
                  f"(bias/timing aliasing caught)")

        # (c1) wrong tripod SURVEY (~1.1 km off) -> must refuse. It surfaces as
        # UNIDENTIFIABLE because a distant tripod flattens the pass geometry.
        s5 = _synth_session(os.path.join(td, "s5"), t_wall_frames, tag_r,
                            INDEX_HEADER, TAGS_HEADER)
        try:
            join_session(s5, track, tripod_llh=(lat0 + 0.0100, lon0, alt0),
                         do_auto_sync=True, quiet=True)
            check(False, "wrong tripod survey must REFUSE")
        except Refuse as e:
            check(True, f"bad tripod survey (~1.1 km off) REFUSED ({str(e)[:55]}...)")

        # (c2) the BIAS leg specifically: geometry intact, but every tag range is
        # 8 m long (a mis-entered --tag-size / placard-vs-antenna datum blunder).
        tag_off = [None if r is None else r + 8.0 for r in tag_r]
        s5b = _synth_session(os.path.join(td, "s5b"), t_wall_frames, tag_off,
                             INDEX_HEADER, TAGS_HEADER)
        try:
            join_session(s5b, track, tripod_llh=(lat0, lon0, alt0),
                         do_auto_sync=True, quiet=True)
            check(False, "an 8 m constant range bias must REFUSE")
        except Refuse as e:
            check("constant bias" in str(e),
                  f"8 m tag-vs-log constant bias REFUSED ({str(e)[:55]}...)")

        # (d) replay session (no t_wall_unix) -> refuse
        s6 = os.path.join(td, "s6")
        os.makedirs(os.path.join(s6, "frames"), exist_ok=True)
        with open(os.path.join(s6, "index.csv"), "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(INDEX_HEADER)
            for i in range(5):
                wr.writerow([i, f"frames/{i:06d}.png", f"{i * 0.03:.6f}", "", "", ""])
        try:
            join_session(s6, track, tripod_llh=(lat0, lon0, alt0), quiet=True)
            check(False, "session with no t_wall_unix must REFUSE")
        except Refuse as e:
            check("t_wall_unix" in str(e),
                  f"replay session (blank t_wall_unix) REFUSED ({str(e)[:60]}...)")

        # ---- 4. partial coverage -> gap/extrapolated flags, no fabricated range ----
        t_wall_wide = np.concatenate([t_wall_frames - 20.0, t_wall_frames])
        s7 = _synth_session(os.path.join(td, "s7"), t_wall_wide, None,
                            INDEX_HEADER, TAGS_HEADER)
        prov7 = join_session(s7, track, tripod_llh=(lat0, lon0, alt0),
                             clock_offset_s=TRUE_OFFSET, min_coverage=0.4, quiet=True)
        with open(os.path.join(s7, "index.csv"), newline="") as f:
            rows7 = list(csv.DictReader(f))
        extrap = [r for r in rows7 if r[COL_QUALITY] == _QUALITY_EXTRAP]
        check(len(extrap) == n_fr and all(r[COL_RANGE] == "" for r in extrap),
              f"{len(extrap)} out-of-span frames flagged '{_QUALITY_EXTRAP}' with "
              f"an EMPTY true_range_m (never fabricated)")

        # ---- 5. GPS fix quality read from the .BIN ----
        q = load_gps_quality_from_bin(bin_path)
        check(q is not None and len(q[0]) >= 1 and np.isfinite(q[1][0]),
              f"GPS fix quality read from the .BIN (NSats={q[1][0] if q else '?'}, "
              f"HDop={q[2][0] if q else '?'})")

    print(f"[self-test] {'PASS' if ok else 'FAIL'}")
    return ok


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def _parse_enu(s):
    parts = [p for p in str(s).replace(";", ",").split(",") if p.strip() != ""]
    if len(parts) not in (2, 3):
        raise argparse.ArgumentTypeError("--tripod-enu wants E,N[,U] in metres")
    vals = [float(p) for p in parts]
    while len(vals) < 3:
        vals.append(0.0)
    return vals


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session-dir", help="pi_capture session dir (frames/ + index.csv)")
    ap.add_argument("--bin", dest="bin_path", help="target ArduPilot DataFlash .BIN "
                    "(constraint target-is-ardupilot; needs pymavlink)")
    ap.add_argument("--ulog", help="target PX4 ULog (.ulg)")
    ap.add_argument("--csv", dest="csv_path",
                    help="target track CSV (t_utc_s + lat_deg/lon_deg/alt_m, or "
                         "east_m/north_m[,up_m]) -- the zero-dependency fallback")
    ap.add_argument("--tripod-lat", type=float, help="surveyed tripod latitude (deg)")
    ap.add_argument("--tripod-lon", type=float, help="surveyed tripod longitude (deg)")
    ap.add_argument("--tripod-alt", type=float,
                    help="surveyed tripod altitude (m) in the LOG'S datum "
                         "(ArduPilot POS/GPS Alt is AMSL)")
    ap.add_argument("--tripod-enu", type=_parse_enu, default=None,
                    help="tripod position E,N,U (m) when the track is already local ENU")
    ap.add_argument("--clock-offset-s", type=float, default=0.0,
                    help="seconds to ADD to each frame's t_wall_unix to land on the "
                         "log's UTC axis (from the protocol §6.1 sync event). With "
                         "--auto-sync this is the search CENTRE.")
    ap.add_argument("--auto-sync", action="store_true",
                    help="estimate the clock offset by aligning the tag-decoded range "
                         "(tags.csv) with the log-derived range; refuses if the offset "
                         "is unresolvable")
    ap.add_argument("--tags-csv", default=None,
                    help="tags.csv for --auto-sync (default SESSION/tags.csv; point at "
                         "an OFFLINE re-decode per protocol §6)")
    ap.add_argument("--sync-window-s", type=float, default=DEFAULT_SYNC_WINDOW_S,
                    help=f"--auto-sync search half-window (default {DEFAULT_SYNC_WINDOW_S} s)")
    ap.add_argument("--max-residual-m", type=float, default=DEFAULT_MAX_RESIDUAL_M,
                    help=f"--auto-sync robust residual ceiling (default {DEFAULT_MAX_RESIDUAL_M} m)")
    ap.add_argument("--max-bias-m", type=float, default=DEFAULT_MAX_BIAS_M,
                    help=f"--auto-sync constant-bias ceiling (default {DEFAULT_MAX_BIAS_M} m; "
                         f"catches a survey / altitude-datum blunder)")
    ap.add_argument("--max-offset-sigma-s", type=float, default=DEFAULT_MAX_OFFSET_SIGMA_S,
                    help=f"--auto-sync offset-uncertainty ceiling (default "
                         f"{DEFAULT_MAX_OFFSET_SIGMA_S} s; protocol §6.1 wants << 0.22 s)")
    ap.add_argument("--min-sync-samples", type=int, default=DEFAULT_MIN_SYNC_SAMPLES,
                    help=f"minimum tag-ranged frames for --auto-sync (default {DEFAULT_MIN_SYNC_SAMPLES})")
    ap.add_argument("--max-gap-s", type=float, default=DEFAULT_MAX_GAP_S,
                    help=f"interpolation gap that flags a frame '{_QUALITY_GAP}' "
                         f"(default {DEFAULT_MAX_GAP_S} s)")
    ap.add_argument("--min-coverage", type=float, default=DEFAULT_MIN_COVERAGE,
                    help=f"minimum fraction of timed frames inside the log span "
                         f"(default {DEFAULT_MIN_COVERAGE}); below this the join REFUSES")
    ap.add_argument("--gps-sigma-m", type=float, default=DEFAULT_GPS_SIGMA_M,
                    help=f"absolute GPS position sigma feeding range_sigma_m "
                         f"(default {DEFAULT_GPS_SIGMA_M} m, field_score accuracy_caveat)")
    ap.add_argument("--min-nsats", type=int, default=DEFAULT_MIN_NSATS)
    ap.add_argument("--max-hdop", type=float, default=DEFAULT_MAX_HDOP)
    ap.add_argument("--dry-run", action="store_true",
                    help="compute + report, do NOT touch index.csv")
    ap.add_argument("--self-test", action="store_true",
                    help="synthetic .BIN + synthetic session, no hardware; exit 0/1")
    args = ap.parse_args()

    if args.self_test:
        return 0 if self_test() else 1
    if not args.session_dir:
        ap.error("--session-dir is required (or --self-test)")
    srcs = [(args.bin_path, load_track_from_bin), (args.ulog, load_track_from_ulog),
            (args.csv_path, load_track_from_csv)]
    srcs = [(p, fn) for p, fn in srcs if p]
    if len(srcs) != 1:
        ap.error("provide EXACTLY ONE target log source: --bin | --ulog | --csv")
    path, loader = srcs[0]

    tripod_llh = None
    if args.tripod_lat is not None or args.tripod_lon is not None:
        if None in (args.tripod_lat, args.tripod_lon, args.tripod_alt):
            ap.error("--tripod-lat/--tripod-lon/--tripod-alt must be given together")
        tripod_llh = (args.tripod_lat, args.tripod_lon, args.tripod_alt)
    if tripod_llh is None and args.tripod_enu is None:
        ap.error("the surveyed tripod position is required: --tripod-lat/--tripod-lon/"
                 "--tripod-alt (lat/lon track) or --tripod-enu (local-ENU track)")

    try:
        track = loader(Path(path), "target")
        for w in track.warnings:
            print(f"[warn] target track: {w}", file=sys.stderr)
        join_session(
            args.session_dir, track, tripod_llh=tripod_llh,
            tripod_enu=args.tripod_enu, clock_offset_s=args.clock_offset_s,
            do_auto_sync=args.auto_sync, tags_csv=args.tags_csv,
            max_gap_s=args.max_gap_s, min_coverage=args.min_coverage,
            gps_sigma_m=args.gps_sigma_m,
            gps_quality=(load_gps_quality_from_bin(path) if args.bin_path else None),
            min_nsats=args.min_nsats, max_hdop=args.max_hdop,
            sync_kwargs=dict(window_s=args.sync_window_s,
                             min_samples=args.min_sync_samples,
                             max_residual_m=args.max_residual_m,
                             max_bias_m=args.max_bias_m,
                             max_offset_sigma_s=args.max_offset_sigma_s),
            dry_run=args.dry_run)
    except Refuse as e:
        print(f"[range_truth_join] REFUSED: {e}", file=sys.stderr)
        return 2
    except (ValueError, ImportError, OSError) as e:
        print(f"[range_truth_join] ERROR: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
