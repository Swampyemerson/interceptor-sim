#!/usr/bin/env python3
"""placard_incidence.py -- reconstruct the PLACARD's viewing incidence for every
captured frame, from the target's logged ATTITUDE + the surveyed tripod position
+ the mount's index angle.

WHY THIS EXISTS (docs/tripod_test_protocol.md §4.2b, §7.1 item 4; the table it
makes reachable). `tripod_score.py` can score curve (a) two ways:

  * with a per-frame `incidence_deg` in index.csv for EVERY binnable frame -> a
    real decode-RATE vs (range x incidence) curve, because the MISSES carry an
    incidence too and each cell has an honest denominator;
  * with only the tag's OWN recovered pose -> incidence exists only where the tag
    DECODED, so the rate would be identically 100% -- the tool honestly degrades
    to an ANNOTATION (the incidence distribution of the decodes).

Nothing in the repo produced the first column, so the rate row of that table was
unreachable: the protocol named the recipe (target log attitude + surveyed tripod
+ the pass card's index angle) and no code implemented it. This module is that
recipe. `range_truth_join.py` calls it while it is already joining positions to
frames, because that join ALREADY holds the two other ingredients (the target's
position at each frame time, and the surveyed tripod position).

THE GEOMETRY (docs/placard_mount.md §3.4, §3.6 DECISION 1)
----------------------------------------------------------
The placard is ONE flat 450 x 450 mm panel held VERTICALLY in the target's
fore-aft centreline plane, tag36h11 printed right-reading on BOTH faces, on a
15-degree-indexable disc. So in the target's BODY frame (FRD: x forward, y right,
z down) the panel's outward normal is

    n_body(delta) = ( sin(delta), cos(delta), 0 )

    delta = 0  deg  -> normal out the BEAM (body +y / -y)   [the default mount]
    delta = 90 deg  -> normal out the NOSE (body +x)        [the <=4 m/s block]

`delta` is the pass card's MOUNT INDEX ANGLE (protocol §11 item 11: "without the
index angle, no frame's incidence can be reconstructed"). It is never guessed
here -- `range_truth_join --incidence` REFUSES without `--mount-index-deg`.

Rotate n_body into the world with the logged attitude (ArduPilot ATT Roll/Pitch/
Yaw = a 3-2-1 Euler sequence from NED to body), take the line of sight from the
placard to the SURVEYED CAMERA position, and the incidence is the angle between
them. Because the tag is printed on BOTH faces, the near face is whichever one
looks at the camera, so the incidence is measured with |cos| and lands in
[0, 90] deg. `both_faces=False` scores a single-sided panel instead, and then a
back-lit frame honestly reports > 90 deg (structurally undecodable).

Sanity checks this reproduces from the mount doc (see the unit tests):
  * BEAM mount, level flight: PITCH changes the incidence by exactly ZERO
    (a rotation about body y leaves y-hat fixed -- placard_mount §3.4). This is
    the whole reason the beam orientation was chosen, so the code must show it.
  * BEAM mount: ROLL (bank) phi tilts the normal out of horizontal 1:1.
  * A camera BELOW the target costs elevation: at an 8 m cross-range and 6 m of
    height the incidence is atan(6/8) = 36.9 deg even at the exact CPA.

WHAT IS NOT MODELLED (say it out loud, per docs/error_handling_policy.md)
  * The GPS antenna / placard lever arm (~0.2 m) is neglected: at the 4-20 m
    stations that is <= 3 deg and it is a POSITION error, not an attitude one.
  * The EKF's own attitude error is not in the reported sigma unless the caller
    supplies `--attitude-sigma-deg` -- ArduPilot's ATT record carries no
    covariance, and inventing one is exactly the fail-closed rule's prohibition.

HONESTY BOUNDARY: like `range_truth_join`/`field_score`, this is OFFLINE
post-flight scoring. No flight path imports it; the flying seeker never reads a
log-derived incidence.
"""
from __future__ import annotations

import collections
import csv
import math
import os

import numpy as np

# The mount's index step (docs/placard_mount.md §3.6: "15-degree-indexable disc").
MOUNT_INDEX_STEP_DEG = 15.0
# Above this the cos-theta decode law is HYPOTHESIS, not measurement: the sim
# sweep's most oblique decoding placement was 32.6 deg (placard_mount §3.2,
# logs/autolabel_sim_20260721T032626Z.csv). Mirrors
# tripod_score.COS_LAW_VALIDATED_TO_DEG; re-stated here so this module can be
# read alone, and asserted EQUAL by the contract test so the two cannot drift.
COS_LAW_VALIDATED_TO_DEG = 32.6

AttitudeTrack = collections.namedtuple(
    "AttitudeTrack", "t_s roll_deg pitch_deg yaw_deg source n warnings")


class NoAttitude(Exception):
    """The requested attitude could not be loaded/honestly used. The caller
    (range_truth_join) maps this to its own Refuse -> exit 2."""


# --------------------------------------------------------------------------
# Geometry
# --------------------------------------------------------------------------
def placard_normal_body(index_deg: float) -> np.ndarray:
    """Placard outward normal in the target's BODY frame (FRD), unit length.

    index 0 deg = beam (body +y), 90 deg = nose-on (body +x) -- placard_mount
    §3.6 DECISION 1's index disc."""
    d = math.radians(float(index_deg))
    return np.array([math.sin(d), math.cos(d), 0.0], dtype=float)


def normals_enu(roll_deg, pitch_deg, yaw_deg, index_deg, yaw_only=False):
    """Placard normals in ENU for each logged attitude sample. Returns (N,3).

    ArduPilot ATT is a 3-2-1 (yaw-pitch-roll) Euler set taking NED to body, so
    the body axes expressed in NED are the COLUMNS of the transpose:

        x_ned = ( cos p cos y,  cos p sin y, -sin p )
        y_ned = ( sin r sin p cos y - cos r sin y,
                  sin r sin p sin y + cos r cos y,
                  sin r cos p )

    and n_ned = sin(delta)*x_ned + cos(delta)*y_ned. ENU is then
    (E, N, U) = (n_ned[1], n_ned[0], -n_ned[2]).

    `yaw_only=True` zeroes roll and pitch -- the approximation the caller can ask
    for explicitly; `range_truth_join` reports the measured difference between
    the two so the approximation is BOUNDED by data, never asserted."""
    r = np.zeros_like(np.asarray(yaw_deg, dtype=float)) if yaw_only else \
        np.radians(np.asarray(roll_deg, dtype=float))
    p = np.zeros_like(np.asarray(yaw_deg, dtype=float)) if yaw_only else \
        np.radians(np.asarray(pitch_deg, dtype=float))
    y = np.radians(np.asarray(yaw_deg, dtype=float))
    sd, cd = math.sin(math.radians(float(index_deg))), math.cos(math.radians(float(index_deg)))

    x_n = np.cos(p) * np.cos(y)
    x_e = np.cos(p) * np.sin(y)
    x_d = -np.sin(p)
    y_n = np.sin(r) * np.sin(p) * np.cos(y) - np.cos(r) * np.sin(y)
    y_e = np.sin(r) * np.sin(p) * np.sin(y) + np.cos(r) * np.cos(y)
    y_d = np.sin(r) * np.cos(p)

    n_n = sd * x_n + cd * y_n
    n_e = sd * x_e + cd * y_e
    n_d = sd * x_d + cd * y_d
    out = np.column_stack([n_e, n_n, -n_d])          # ENU
    nrm = np.linalg.norm(out, axis=1, keepdims=True)
    return out / np.where(nrm > 1e-12, nrm, 1.0)


def incidence_deg_from(normal_enu, los_enu, both_faces=True):
    """Incidence angle(s) in degrees between the placard normal and the line of
    sight PLACARD -> CAMERA. `los_enu` need not be normalised.

    both_faces=True (the adopted double-sided print, placard_mount §3.6) folds
    the back face in with |cos|, so the result is the angle to the NEAR face and
    lies in [0, 90]. both_faces=False keeps the sign, so a frame looking at the
    unprinted back reports > 90 deg -- structurally undecodable, and visibly so."""
    n = np.atleast_2d(np.asarray(normal_enu, dtype=float))
    l = np.atleast_2d(np.asarray(los_enu, dtype=float))
    ln = np.linalg.norm(l, axis=1, keepdims=True)
    with np.errstate(invalid="ignore", divide="ignore"):
        u = l / np.where(ln > 1e-12, ln, np.nan)
    c = np.sum(n * u, axis=1)
    if both_faces:
        c = np.abs(c)
    th = np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))
    return th if th.size > 1 else float(th[0])


def normal_rate_deg_s(t_s, normals):
    """Angular rate (deg/s) of the placard normal, sample to sample. Feeds the
    per-frame incidence sigma: an interpolation gap or a clock-offset error costs
    rate x dt of aspect. Same shape as t_s."""
    t = np.asarray(t_s, dtype=float)
    if t.size < 2:
        return np.zeros_like(t)
    d = np.sum(normals[:-1] * normals[1:], axis=1)
    ang = np.degrees(np.arccos(np.clip(d, -1.0, 1.0)))
    dt = np.diff(t)
    seg = np.where(dt > 1e-9, ang / np.where(dt > 1e-9, dt, 1.0), 0.0)
    rate = np.empty_like(t)
    rate[0], rate[-1] = seg[0], seg[-1]
    if t.size > 2:
        rate[1:-1] = np.maximum(seg[:-1], seg[1:])
    return rate


def interp_normals(t_query, t_att, normals):
    """Interpolate the normal DIRECTION at the query times, component-wise then
    renormalised -- exactly how range_truth_join interpolates position (and why
    it is done on the vector, not on the Euler angles: yaw wraps at +/-180 and a
    linear interpolation across the wrap would swing the aspect 359 deg).

    Returns (normals_q (N,3), nearest_sample_dt_s, inside_span_mask). NOTHING is
    extrapolated: a frame outside the attitude span is marked False and the
    caller must leave its incidence EMPTY."""
    t_query = np.asarray(t_query, dtype=float)
    t_att = np.asarray(t_att, dtype=float)
    inside = ((t_query >= t_att[0]) & (t_query <= t_att[-1]) &
              np.isfinite(t_query))
    tq = np.where(np.isfinite(t_query), t_query, t_att[0])
    comps = [np.interp(tq, t_att, normals[:, k]) for k in range(3)]
    nq = np.column_stack(comps)
    nrm = np.linalg.norm(nq, axis=1, keepdims=True)
    nq = nq / np.where(nrm > 1e-12, nrm, 1.0)
    idx = np.clip(np.searchsorted(t_att, tq), 1, len(t_att) - 1)
    dt = np.minimum(np.abs(tq - t_att[idx - 1]), np.abs(t_att[idx] - tq))
    return nq, dt, inside


# --------------------------------------------------------------------------
# Attitude loaders
# --------------------------------------------------------------------------
def load_attitude_from_bin(path) -> AttitudeTrack:
    """Roll/Pitch/Yaw (deg) + timestamps from an ArduPilot DataFlash ATT record.

    Same reader, same clock as field_score.load_track_from_bin (DFReader gives
    seconds-since-1970 once the log has a GPS UTC fix), so the attitude lands on
    the SAME axis as the positions this is joined against."""
    try:
        from pymavlink import DFReader
    except ImportError as e:
        raise NoAttitude(
            f"pymavlink is required to read ATT from an ArduPilot .BIN ({e}). "
            f"Install it into the venv, or export the attitude to CSV "
            f"(t_utc_s,roll_deg,pitch_deg,yaw_deg) and use --attitude-csv.")
    try:
        reader = DFReader.DFReader_binary(str(path))
    except Exception as e:                                    # noqa: BLE001
        raise NoAttitude(f"could not open {path} as a DataFlash log: {e}")
    ts, ro, pi, ya = [], [], [], []
    while True:
        m = reader.recv_match(type=["ATT"])
        if m is None:
            break
        r, p, y = (getattr(m, "Roll", None), getattr(m, "Pitch", None),
                   getattr(m, "Yaw", None))
        if r is None or p is None or y is None:
            continue
        ts.append(float(m._timestamp))
        ro.append(float(r))
        pi.append(float(p))
        ya.append(float(y))
    if len(ts) < 2:
        raise NoAttitude(
            f"{path} carries {len(ts)} usable ATT record(s) -- the per-frame "
            f"incidence cannot be reconstructed from it. ATT is in ArduPilot's "
            f"default LOG_BITMASK; if this log was trimmed or the bitmask was "
            f"cut, either re-pull the full .BIN or supply --attitude-csv. "
            f"REFUSING to score aspect without a measured attitude.")
    o = np.argsort(np.asarray(ts, dtype=float))
    return AttitudeTrack(np.asarray(ts, dtype=float)[o],
                         np.asarray(ro, dtype=float)[o],
                         np.asarray(pi, dtype=float)[o],
                         np.asarray(ya, dtype=float)[o],
                         f"dataflash:ATT ({os.path.basename(str(path))})",
                         len(ts), [])


def load_attitude_from_csv(path, allow_yaw_only=False) -> AttitudeTrack:
    """Attitude from an exported CSV: `t_utc_s` + `yaw_deg` (or `Yaw`), with
    `roll_deg`/`pitch_deg` (or `Roll`/`Pitch`).

    FAIL-CLOSED: a CSV that carries yaw but NOT roll/pitch is refused unless the
    caller passed --incidence-yaw-only, because silently substituting roll=0,
    pitch=0 would be inventing a measured quantity (error policy §5)."""
    if not os.path.exists(path):
        raise NoAttitude(f"attitude CSV not found: {path}")
    t, ro, pi, ya = [], [], [], []
    have_rp = None
    with open(path, newline="") as f:
        rd = csv.DictReader(f)
        cols = {c.strip().lower(): c for c in (rd.fieldnames or [])}

        def pick(*names):
            for nm in names:
                if nm in cols:
                    return cols[nm]
            return None

        c_t = pick("t_utc_s", "t_s", "time_s", "timestamp")
        c_y = pick("yaw_deg", "yaw", "hdg_deg", "heading_deg")
        c_r = pick("roll_deg", "roll")
        c_p = pick("pitch_deg", "pitch")
        if c_t is None or c_y is None:
            raise NoAttitude(
                f"{path}: an attitude CSV needs a time column (t_utc_s) and a "
                f"yaw column (yaw_deg); found {rd.fieldnames}")
        have_rp = (c_r is not None and c_p is not None)
        if not have_rp and not allow_yaw_only:
            raise NoAttitude(
                f"{path}: carries yaw but NO roll/pitch column. Substituting "
                f"level flight would INVENT a measured quantity (a 20 deg bank "
                f"is 6% of decode range, placard_mount §3.4). Export roll/pitch, "
                f"or pass --incidence-yaw-only to accept the approximation "
                f"deliberately (it is stamped into range_truth.json).")
        for row in rd:
            try:
                tv = float(row[c_t])
                yv = float(row[c_y])
            except (TypeError, ValueError, KeyError):
                continue
            rv = pv = 0.0
            if have_rp:
                try:
                    rv, pv = float(row[c_r]), float(row[c_p])
                except (TypeError, ValueError, KeyError):
                    continue
            t.append(tv)
            ya.append(yv)
            ro.append(rv)
            pi.append(pv)
    if len(t) < 2:
        raise NoAttitude(f"{path}: fewer than 2 usable attitude rows")
    o = np.argsort(np.asarray(t, dtype=float))
    warn = ([] if have_rp else
            ["attitude CSV has no roll/pitch: yaw-only normals "
             "(--incidence-yaw-only was given)"])
    return AttitudeTrack(np.asarray(t, dtype=float)[o],
                         np.asarray(ro, dtype=float)[o],
                         np.asarray(pi, dtype=float)[o],
                         np.asarray(ya, dtype=float)[o],
                         f"csv:{os.path.basename(str(path))}", len(t), warn)


# --------------------------------------------------------------------------
# Self-test (geometry only -- no I/O, no hardware). The pipeline-level contract
# test lives in tests/test_incidence_rate_curve_contract.py.
# --------------------------------------------------------------------------
def self_test():
    ok = True

    def check(cond, msg):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[placard_incidence] {'OK  ' if cond else 'FAIL'} {msg}")

    # Target heading NORTH, level, beam placard -> normal points EAST.
    n = normals_enu([0.0], [0.0], [0.0], 0.0)[0]
    check(np.allclose(n, [1, 0, 0], atol=1e-9),
          f"beam placard, heading north -> normal EAST {np.round(n, 6)}")
    # Index 90 deg = nose-on -> normal NORTH.
    n90 = normals_enu([0.0], [0.0], [0.0], 90.0)[0]
    check(np.allclose(n90, [0, 1, 0], atol=1e-9),
          f"index 90 deg -> NOSE-ON normal {np.round(n90, 6)}")
    # placard_mount §3.4: pitch costs a beam placard EXACTLY zero incidence.
    n_p = normals_enu([0.0], [35.0], [0.0], 0.0)[0]
    check(np.allclose(n_p, n, atol=1e-12),
          "35 deg of PITCH moves the beam placard's normal by 0 (mount §3.4)")
    # ...and roll tilts it 1:1.
    n_r = normals_enu([20.0], [0.0], [0.0], 0.0)[0]
    check(abs(math.degrees(math.asin(abs(n_r[2]))) - 20.0) < 1e-9,
          "20 deg of ROLL tilts the beam normal 20 deg out of horizontal")
    # Camera 6 m below, 8 m abeam, at the CPA -> 36.87 deg by elevation alone.
    los = np.array([[-8.0, 0.0, -6.0]])
    check(abs(incidence_deg_from(n, los) - math.degrees(math.atan2(6, 8))) < 1e-9,
          "elevation-only incidence at the CPA = atan(6/8) = 36.87 deg")
    # Both faces: the back-lit case folds to the same near-face angle.
    check(abs(incidence_deg_from(n, -los) - incidence_deg_from(n, los)) < 1e-12,
          "double-sided print -> incidence is the NEAR face either way")
    # Here the EAST-facing normal points AWAY from a camera that is west of the
    # target, so a single-sided panel is back-lit: 143 deg, reported as such.
    check(abs(incidence_deg_from(n, los, both_faces=False) - 143.13) < 0.01
          and abs(incidence_deg_from(n, -los, both_faces=False)
                  - math.degrees(math.atan2(6, 8))) < 1e-9,
          "single-sided print -> the back-lit frame reports 143 deg, not a fold")
    # Interpolation across the yaw wrap must not swing 359 deg.
    t = np.array([0.0, 1.0])
    nn = normals_enu([0, 0], [0, 0], [179.0, -179.0], 0.0)
    nq, _dt, inside = interp_normals(np.array([0.5]), t, nn)
    mid = normals_enu([0], [0], [180.0], 0.0)[0]
    check(bool(inside[0]) and np.linalg.norm(nq[0] - mid) < 1e-3,
          "yaw wrap 179 -> -179 interpolates through 180, not through 0")
    nq2, _d2, ins2 = interp_normals(np.array([5.0]), t, nn)
    check(not bool(ins2[0]),
          "a frame outside the attitude span is NOT extrapolated (inside=False)")
    print(f"[placard_incidence] {'PASS' if ok else 'FAIL'}")
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if self_test() else 1)
