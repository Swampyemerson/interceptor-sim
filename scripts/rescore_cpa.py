#!/usr/bin/env python3
"""rescore_cpa.py -- STANDALONE re-scorer for closest-point-of-approach (CPA).

WHAT THIS IS (issue #8, docs/scoring_fix_plan.md Section 2): every published miss
number in this project was computed as

    min over LOGGED TICKS of |p_target_world - p_CAMERA_world|

i.e. it ranges to the CAMERA LENS and it takes the smallest LOGGED SAMPLE rather
than the true minimum BETWEEN samples.  ADR-0084 defines the criterion as
CENTRE-TO-CENTRE contact between two airframes (R_ram = 0.35 m = the sum of the
two half-spans).  This tool re-scores existing flight CSVs offline -- it flies
nothing, it edits nothing, and it does not touch scripts/m4_intercept.py.

It reports, per flight and per arm, the three rulers the issue asks for:

  1. c2c interpolated  (PRIMARY)      airframe centre, true CPA between ticks
  2. c2c logged-tick   (conservative) airframe centre, nearest logged sample
  3. lens logged-tick  (LEGACY)       what is published today -- the old ruler

plus two extra reference modes that exist so the reader can see exactly where
the numbers come from (`--mode` / the per-arm table prints all of them).

--------------------------------------------------------------------------
THE FRAME CHAIN -- READ THIS BEFORE CHANGING ANY CONSTANT
--------------------------------------------------------------------------
`gt_cam_x/y/z` in the flight CSV is the world-ENU position of `camera_link`,
composed by m3_static_intercept.ground_truth_world_points() as
model_pose (x) camera_link_pose.  The x500_mono_cam model resolves as:

    x500_mono_cam (model frame, = the pose gz publishes for the model)
      |- include merge x500 -> merge x500_base, whose model.sdf carries
      |     <model name='x500_base'><pose>0 0 .24 0 0 0</pose>
      |     => base_link sits at (0, 0, +0.240) in the MODEL frame
      '- include mono_cam at <pose>.12 .03 .242</pose>  (MODEL frame)
            => camera_link sits at (0.12, 0.03, +0.242) in the MODEL frame

    => camera_link - base_link = (0.120, 0.030, +0.002) in body FLU.

THE CAMERA IS ~2 mm ABOVE THE AIRFRAME DATUM, NOT 0.208 m.  The "+0.208 m"
in issue #8 is `median(gt_cam_z - alt_m)`, and `alt_m` is MAVSDK
`relative_altitude_m` -- altitude ABOVE THE TAKEOFF POINT, not world z.  That
difference is dominated by the x500's LANDING-GEAR HEIGHT (base_link rests
0.224 m above the ground plane; leg collision boxes at z=-0.123, size
0.015x0.015x0.21, rolled +-0.35 rad => lowest point -0.2242), not by a camera
lever arm.  Two independent measurements agree:

  * SDF chain (above):                       camera - base_link = +0.002 m
  * on-pad ticks of every flight in logs/:   gt_cam_z = 0.2290 m at alt_m = 0,
    minus the computed leg rest height 0.2242 m => +0.0048 m
    (residual = contact penetration; the two agree to 3 mm)

  * the same on-pad ticks read gt_cam_x/y = EXACTLY (0.1200, 0.0300) -- the
    model spawns at world (0,0) with yaw 0, so the horizontal lever is
    confirmed to 4 decimals, and the model ORIGIN rests at z = -0.013 m
    (0.240 m below base_link), which is why it must not be used as the
    scoring reference either (see MODE `model`).

If the camera really sat 0.242 m above base_link, an x500 at rest on the pad
would report gt_cam_z = 0.224 + 0.242 = 0.466 m.  It reports 0.229 m.

CONSEQUENCE: the correct centre-to-centre correction is almost entirely
HORIZONTAL (0.120 m aft of the lens along the body forward axis, 0.030 m
right), NOT a 0.208 m vertical drop.  See --gate for what that does to the
published table.

--------------------------------------------------------------------------
YAW (the horizontal lever has to be rotated into world axes)
--------------------------------------------------------------------------
`psi_deg` is the PX4 (NED) yaw estimate; the CSV positions are Gazebo world
ENU.  The naive mapping heading_ENU = (sin psi, cos psi) is right in FORM but
carries this world's ~6.2 deg yaw-datum offset (the pad ticks read psi = 96.2
deg while the airframe truly points along world +x).  So the tool CALIBRATES
the offset per flight from the pad ticks, where the true body yaw is known
(the model spawns yaw = 0, verified by the exact (0.12, 0.03) camera offset):

    theta_world_ccw(t) = theta_pad - (psi(t) - psi_pad)          [ENU, CCW]

and falls back to the uncalibrated (sin, cos) form -- FLAGGED, counted -- when
the pad check fails.  Every flight is additionally checked against its own
ground-truth velocity heading during the fast part of the dash.

--------------------------------------------------------------------------
ATTITUDE (the acknowledged approximation, and its measured bound)
--------------------------------------------------------------------------
Pitch/roll are NOT logged in the historical era (ADR-0062 deferred them), so
mode `centre` rotates the lever by YAW ONLY -- a level-flight assumption.  A
dashing quad is nose-down 5-35 deg, which tips the lever: the vertical
component goes 0.002*cos(d) - 0.120*sin(d) (i.e. the camera swings BELOW
base_link) and the horizontal goes 0.120*cos(d) + 0.002*sin(d).

Mode `centre-tilt` recovers the tilt per tick from the one attitude-bearing
signal the old logs DO carry -- `alt_m` against the per-flight pad calibration
(z_base(t) = alt_m(t) + z_pad) -- and re-rotates the lever.  The tool always
reports BOTH, and their difference IS the empirical bound on the level-flight
approximation (printed per arm as `recon_bound_m`).  It is not a hand-wave: it
is measured on the same flights being scored.

If a log carries the exact `gt_intc_x/y/z` columns (the additive schema the
plan adds to m4_intercept.py), they are used directly (mode `exact`) and the
reconstruction error is reported against them -- so the bound becomes a
measurement the first time a new-format flight is scored.

--------------------------------------------------------------------------
HARD RULES HONOURED (each traces to a logged incident -- CLAUDE.md)
--------------------------------------------------------------------------
* NO VACUOUS VERDICTS: a flight with zero usable gt rows, a missing column, or
  an all-blank required column is a HARD ERROR naming the file and the column;
  the run exits non-zero and no number is emitted for it.  Every dropped row
  and every dropped flight is COUNTED and PRINTED, never absorbed.
* FAIL CLOSED: no default is ever substituted for a value that should have been
  measured (no assumed yaw datum without the pad check, no assumed tilt without
  alt_m, no assumed lever without the SDF chain above).
* THE RADIUS DOES NOT MOVE: R_ram = 0.35 m (ADR-0084), hardcoded, no flag.
* Sim time / wall time is IRRELEVANT here by construction: the interpolation
  parameterises each segment by s in [0,1] between consecutive logged samples,
  so no clock enters the CPA computation at all.

Usage
  rescore_cpa.py --gate                       # the issue-#8 acceptance table
  rescore_cpa.py --all [--markdown OUT.md]    # every arm in the sweep set
  rescore_cpa.py --arm logs/mc_fp_armG20_line9_s123.csv [...]
  rescore_cpa.py --flight logs/m4_intercept_pronav_...csv [...]
  rescore_cpa.py --arm ... --json out.json --verbose
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import statistics
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# --- ADR-0084. Ratified 0.35 m = sum of the two 5-inch half-spans. NO FLAG. ---
RAM_RADIUS_M = 0.35

# --- Camera lever arm, body FLU (forward, left, up), metres. -----------------
# Provenance: the resolved x500_mono_cam SDF chain in the module docstring,
# cross-checked to 3 mm against the on-pad ticks of every flight in logs/.
CAM_FWD_M = 0.120
CAM_LEFT_M = 0.030
CAM_UP_M = 0.002
# base_link's height above the MODEL origin (x500_base's own <pose> 0 0 .24).
BASE_ABOVE_MODEL_M = 0.240
# The camera's height above the MODEL origin (the mono_cam include pose).
CAM_ABOVE_MODEL_M = 0.242

# base_link's resting height above a z=0 ground plane, from the x500_base
# collision geometry: the lowest collisions are the landing-gear feet
# (base_link_collision_3/4, box 0.25x0.015x0.015 at z=-0.2195) -> -0.2270.
# This is the SECOND, INDEPENDENT route to CAM_UP_M: every flight's on-pad
# ticks read gt_cam_z = 0.2290, and 0.2290 - 0.2270 = 0.0020 = CAM_UP_M, which
# is what the SDF chain says (0.242 - 0.240). The two agree to <1 mm. The
# scorer CHECKS this on every flight rather than trusting the constant.
BASE_REST_ABOVE_GROUND_M = 0.2270
PAD_Z_TOL_M = 0.02

# Pad-calibration acceptance: |camera xy offset| must match hypot(fwd, left).
PAD_XY_NOMINAL_M = math.hypot(CAM_FWD_M, CAM_LEFT_M)
PAD_XY_TOL_M = 0.02
PAD_ALT_TOL_M = 0.05
PAD_MIN_TICKS = 3

# Yaw self-check: median angle between the reconstructed body heading and the
# ground-truth velocity heading over fast ticks. Warn above SOFT, fail above HARD.
#
# ONLY THE CODED_DASH PHASE IS ELIGIBLE, and that restriction is not a
# convenience -- it is the whole validity of the check. In CODED_DASH the
# vehicle flies an open-loop straight run with yaw slaved to the aim point, so
# heading == velocity direction to a few degrees (measured 2.5-7.3 deg across
# the arms in logs/). In ENGAGE and BREAKOFF the pro-nav terminal commands a
# large lateral (v_perp up to 8 m/s) while yaw is held, so the airframe CRABS:
# measured medians of 91-173 deg on real flights that are perfectly healthy.
# Checking those phases would fail-closed on real sideslip and prove nothing.
YAW_CHECK_PHASES = ("CODED_DASH",)
YAW_CHECK_MIN_SPEED_M_S = 3.0
YAW_CHECK_MIN_TICKS = 10
YAW_CHECK_SOFT_DEG = 15.0
YAW_CHECK_HARD_DEG = 45.0

# Interpolation guard: a segment whose RELATIVE displacement exceeds this is a
# logging gap, not a 20 Hz sample step. Counted; flagged if the CPA lands on one.
MAX_SEG_REL_M = 2.0

MODES = ("lens", "centre", "centre-tilt", "model", "adhoc-published", "exact")
PRIMARY_MODE = "centre"

REQUIRED_COLS = ("gt_cam_x", "gt_cam_y", "gt_cam_z",
                 "gt_tag_x", "gt_tag_y", "gt_tag_z")

# The published ad-hoc table this tool must be checked against (issue #8).
PUBLISHED_GATE = {
    "lens_logged": {"median": 0.445, "inside": 3},
    "c2c_logged": {"median": 0.293, "inside": 12},
    "c2c_interp": {"median": 0.237, "inside": 16},
    "n": 16,
}
GATE_ARMS = ("logs/mc_fp_armAE5dash_line9_s123.csv",
             "logs/mc_fp_armAE5dash_line9_s777.csv")

# The sweep set for --all (issue #8: "re-score every historical arm").
SWEEP_GLOBS = ("logs/mc_fp_arm*.csv", "logs/mc_speed10_*.csv",
               "logs/mc_loftdive_arm*.csv")

# Era: the target end is only verified centre==model-origin for fpv_quad_enemy
# (world `quad_enemy`). Anything else gets the caveat, never a silent number.
VERIFIED_TARGET_WORLDS = ("quad_enemy", "quad_enemy_verify")


class HardError(Exception):
    """A measurement-layer failure. Never downgraded to a number."""


# ---------------------------------------------------------------- geometry --


def _seg_min(d0, d1):
    """Closed-form minimum of |d(s)| for d(s) = d0 + s*(d1-d0), s in [0,1].

    Both world positions are linearly interpolated across the tick pair, so the
    relative vector is linear in s and |d(s)|^2 is a quadratic with a single
    interior minimum at s* = -(d0.dd)/(dd.dd). Clamped to the segment; the
    endpoints are already covered by the logged-tick minimum.
    Returns (dist, s) for the interior stationary point, or None.
    """
    dd = [d1[k] - d0[k] for k in range(3)]
    a = dd[0] * dd[0] + dd[1] * dd[1] + dd[2] * dd[2]
    if a <= 0.0:
        return None
    b = d0[0] * dd[0] + d0[1] * dd[1] + d0[2] * dd[2]
    s = -b / a
    if not (0.0 < s < 1.0):
        return None
    v = [d0[k] + s * dd[k] for k in range(3)]
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]), s


def _tilt_from_vertical_lever(lever_z):
    """Nose-down tilt d (rad) satisfying CAM_UP*cos d - CAM_FWD*sin d = lever_z.

    Written as R*cos(d + phi) with R = hypot(up, fwd), phi = atan2(fwd, up);
    d = acos(lever_z / R) - phi, clamped to [0, 60 deg] (a quad that is nose
    down more than 60 deg is not in a dash, it is falling).
    """
    r = math.hypot(CAM_UP_M, CAM_FWD_M)
    phi = math.atan2(CAM_FWD_M, CAM_UP_M)
    x = max(-1.0, min(1.0, lever_z / r))
    d = math.acos(x) - phi
    return max(0.0, min(d, math.radians(60.0)))


# ------------------------------------------------------------------- input --


def _f(row, key):
    v = row.get(key, "")
    if v is None or v == "":
        return None
    try:
        x = float(v)
    except ValueError:
        return None
    if math.isnan(x) or math.isinf(x):
        return None
    return x


class Flight:
    """One per-tick flight CSV, parsed and pad-calibrated."""

    def __init__(self, path):
        self.path = path
        self.ticks = []            # dicts of parsed values
        self.n_rows = 0
        self.n_dropped = 0
        self.drop_reasons = {}
        self.has_exact = False
        self.flags = []            # soft flags (counted + printed)
        self.yaw_offset_deg = None  # pad-calibrated psi at true-yaw-zero
        self.pad_z_base = None      # base_link world z on the pad
        self.pad_source = None
        self.yaw_check_deg = None

    # -- parsing ------------------------------------------------------------
    def load(self):
        if not os.path.exists(self.path):
            raise HardError(f"{self.path}: flight CSV does not exist")
        with open(self.path, newline="") as f:
            reader = csv.DictReader(f)
            header = reader.fieldnames
            if not header:
                raise HardError(f"{self.path}: empty file (no CSV header)")
            missing = [c for c in REQUIRED_COLS if c not in header]
            if missing:
                raise HardError(
                    f"{self.path}: required ground-truth column(s) absent from "
                    f"the header: {missing} -- this file cannot be re-scored")
            self.has_exact = all(
                c in header for c in ("gt_intc_x", "gt_intc_y", "gt_intc_z"))
            for row in reader:
                self.n_rows += 1
                self._add(row)
        if not self.ticks:
            raise HardError(
                f"{self.path}: 0 usable ticks out of {self.n_rows} rows "
                f"(drop reasons: {self.drop_reasons or 'none recorded'}) -- "
                "refusing to emit a CPA computed on zero samples")
        self._calibrate_pad()
        self._check_yaw()

    def _add(self, row):
        cam = [_f(row, "gt_cam_x"), _f(row, "gt_cam_y"), _f(row, "gt_cam_z")]
        tgt = [_f(row, "gt_tag_x"), _f(row, "gt_tag_y"), _f(row, "gt_tag_z")]
        if any(v is None for v in cam):
            self._drop("gt_cam_* blank/NaN")
            return
        if any(v is None for v in tgt):
            self._drop("gt_tag_* blank/NaN")
            return
        exact = None
        if self.has_exact:
            e = [_f(row, "gt_intc_x"), _f(row, "gt_intc_y"), _f(row, "gt_intc_z")]
            if not any(v is None for v in e):
                exact = e
        self.ticks.append({
            "cam": cam, "tgt": tgt, "exact": exact,
            "psi": _f(row, "psi_deg"),
            "alt": _f(row, "alt_m"),
            "phase": (row.get("phase") or "").strip(),
        })

    def _drop(self, reason):
        self.n_dropped += 1
        self.drop_reasons[reason] = self.drop_reasons.get(reason, 0) + 1

    # -- calibration --------------------------------------------------------
    def _calibrate_pad(self):
        """Yaw datum + base_link pad altitude, from the on-pad ticks.

        On the pad the airframe is level and (for every arm in logs/) spawned at
        world (0,0) with yaw 0, so the camera's world xy IS the lever arm and
        the psi reading at that moment IS the yaw-datum offset.
        """
        pad = [t for t in self.ticks
               if t["alt"] is not None and abs(t["alt"]) <= PAD_ALT_TOL_M
               and t["phase"].upper() in ("TAKEOFF", "", "INIT")]
        pad = [t for t in pad if t["psi"] is not None]
        if len(pad) < PAD_MIN_TICKS:
            self.flags.append("no-pad-ticks(yaw datum uncalibrated)")
            self.pad_source = "uncalibrated"
            return
        xy = statistics.median([math.hypot(t["cam"][0], t["cam"][1]) for t in pad])
        if abs(xy - PAD_XY_NOMINAL_M) > PAD_XY_TOL_M:
            self.flags.append(
                f"pad-xy {xy:.4f} m != lever {PAD_XY_NOMINAL_M:.4f} m "
                "(spawn not at world origin? yaw datum uncalibrated)")
            self.pad_source = "uncalibrated"
            return
        # true body yaw on the pad, ENU CCW from +x
        ax = statistics.median([t["cam"][0] for t in pad])
        ay = statistics.median([t["cam"][1] for t in pad])
        theta_pad = math.atan2(ay, ax) - math.atan2(CAM_LEFT_M, CAM_FWD_M)
        psi_pad = statistics.median([t["psi"] for t in pad])
        # psi grows clockwise (NED), theta grows counter-clockwise (ENU)
        self.yaw_offset_deg = psi_pad + math.degrees(theta_pad)
        pad_cam_z = statistics.median([t["cam"][2] for t in pad])
        self.pad_z_base = pad_cam_z - CAM_UP_M
        self.pad_source = "pad-calibrated"
        # Independent check of the VERTICAL lever against the airframe's
        # collision-derived rest height. This is the check that would have
        # caught the "camera sits 0.208 m above the airframe" error: on the pad
        # the vehicle is level and at rest, so gt_cam_z MUST equal
        # BASE_REST_ABOVE_GROUND_M + CAM_UP_M for the constants to be right.
        expect = BASE_REST_ABOVE_GROUND_M + CAM_UP_M
        if abs(pad_cam_z - expect) > PAD_Z_TOL_M:
            self.flags.append(
                f"pad-z {pad_cam_z:.4f} m != rest height + lever "
                f"{expect:.4f} m (the vertical lever constant does not match "
                "this airframe/world -- re-derive it before trusting a c2c "
                "number from this flight)")

    def heading(self, psi_deg):
        """(heading, left) unit vectors in world ENU for a PX4 yaw reading."""
        if psi_deg is None:
            return None, None
        if self.yaw_offset_deg is None:
            th = math.radians(90.0 - psi_deg)     # uncalibrated fallback
        else:
            th = math.radians(self.yaw_offset_deg - psi_deg)
        h = (math.cos(th), math.sin(th))
        return h, (-h[1], h[0])

    def _check_yaw(self):
        """Reconstructed body heading vs ground-truth velocity heading, over
        the CODED_DASH phase only (see YAW_CHECK_PHASES for why)."""
        errs = []
        for i in range(len(self.ticks) - 1):
            a, b = self.ticks[i], self.ticks[i + 1]
            if a["phase"].upper() not in YAW_CHECK_PHASES:
                continue
            dx = b["cam"][0] - a["cam"][0]
            dy = b["cam"][1] - a["cam"][1]
            if math.hypot(dx, dy) < YAW_CHECK_MIN_SPEED_M_S * 0.05:
                continue
            h, _ = self.heading(a["psi"])
            if h is None:
                continue
            n = math.hypot(dx, dy)
            dot = max(-1.0, min(1.0, (h[0] * dx + h[1] * dy) / n))
            errs.append(abs(math.degrees(math.acos(dot))))
        if len(errs) < YAW_CHECK_MIN_TICKS:
            self.flags.append(
                f"yaw-check-unverified({len(errs)} fast CODED_DASH ticks)")
            return
        self.yaw_check_deg = statistics.median(errs)
        if self.yaw_check_deg > YAW_CHECK_HARD_DEG:
            raise HardError(
                f"{self.path}: reconstructed heading disagrees with the "
                f"ground-truth velocity heading by a median "
                f"{self.yaw_check_deg:.1f} deg over {len(errs)} fast "
                "CODED_DASH ticks -- the psi_deg -> world-ENU convention does "
                "not hold for this log; refusing to rotate a lever arm with it")
        if self.yaw_check_deg > YAW_CHECK_SOFT_DEG:
            self.flags.append(f"yaw-check {self.yaw_check_deg:.1f} deg")

    # -- reference points ---------------------------------------------------
    def points(self, mode):
        """[(p_interceptor, p_target)] under one reference convention."""
        if mode not in MODES:
            raise HardError(f"unknown mode {mode!r} (modes: {MODES})")
        if mode == "exact" and not self.has_exact:
            raise HardError(
                f"{self.path}: --mode exact requires gt_intc_x/y/z; this log "
                "predates those columns (historical era) -- use `centre`")
        need_psi = mode in ("centre", "centre-tilt", "model")
        need_alt = mode in ("centre-tilt", "adhoc-published")
        if need_psi and all(t["psi"] is None for t in self.ticks):
            raise HardError(
                f"{self.path}: psi_deg is blank on every usable tick -- mode "
                f"{mode!r} cannot rotate the camera lever arm")
        if need_alt and all(t["alt"] is None for t in self.ticks):
            raise HardError(
                f"{self.path}: alt_m is blank on every usable tick -- mode "
                f"{mode!r} needs it")
        if mode == "centre-tilt" and self.pad_z_base is None:
            raise HardError(
                f"{self.path}: mode 'centre-tilt' needs the on-pad calibration "
                "and this flight has no usable pad ticks")

        out = []
        dropped = 0
        for t in self.ticks:
            cam, tgt = t["cam"], t["tgt"]
            if mode == "lens":
                p = list(cam)
            elif mode == "exact":
                if t["exact"] is None:
                    dropped += 1
                    continue
                p = list(t["exact"])
            elif mode == "adhoc-published":
                if t["alt"] is None:
                    dropped += 1
                    continue
                p = [cam[0], cam[1], t["alt"]]
            else:
                h, l = self.heading(t["psi"])
                if h is None:
                    dropped += 1
                    continue
                if mode == "model":
                    fwd, left, up = CAM_FWD_M, CAM_LEFT_M, CAM_ABOVE_MODEL_M
                elif mode == "centre":
                    fwd, left, up = CAM_FWD_M, CAM_LEFT_M, CAM_UP_M
                else:  # centre-tilt
                    if t["alt"] is None:
                        dropped += 1
                        continue
                    lever_z = cam[2] - (t["alt"] + self.pad_z_base)
                    d = _tilt_from_vertical_lever(lever_z)
                    fwd = CAM_FWD_M * math.cos(d) + CAM_UP_M * math.sin(d)
                    left = CAM_LEFT_M
                    up = CAM_UP_M * math.cos(d) - CAM_FWD_M * math.sin(d)
                p = [cam[0] - fwd * h[0] - left * l[0],
                     cam[1] - fwd * h[1] - left * l[1],
                     cam[2] - up]
            out.append((p, tgt))
        if not out:
            raise HardError(
                f"{self.path}: mode {mode!r} produced 0 scorable ticks "
                f"({dropped} dropped for missing psi_deg/alt_m) -- refusing a "
                "verdict on zero samples")
        return out, dropped

    # -- scoring ------------------------------------------------------------
    def cpa(self, mode):
        pts, dropped = self.points(mode)
        logged = None
        logged_i = None
        for i, (a, b) in enumerate(pts):
            d = math.dist(a, b)
            if logged is None or d < logged:
                logged, logged_i = d, i
        interp = logged
        interp_i = None
        long_segs = 0
        interp_on_long = False
        for i in range(len(pts) - 1):
            a0, b0 = pts[i]
            a1, b1 = pts[i + 1]
            d0 = [b0[k] - a0[k] for k in range(3)]
            d1 = [b1[k] - a1[k] for k in range(3)]
            step = math.dist(d0, d1)
            is_long = step > MAX_SEG_REL_M
            if is_long:
                long_segs += 1
            r = _seg_min(d0, d1)
            if r is not None and r[0] < interp:
                interp, interp_i, interp_on_long = r[0], i, is_long
        return {
            "mode": mode,
            "logged_m": logged,
            "logged_tick": logged_i,
            "interp_m": interp,
            "interp_segment": interp_i,
            "n_scored": len(pts),
            "n_mode_dropped": dropped,
            "long_segments": long_segs,
            "interp_on_long_segment": interp_on_long,
        }


# -------------------------------------------------------------- aggregation --


def summarise(values):
    """Both median conventions, explicitly. The published ad-hoc table used the
    UPPER median (sorted[n//2]); statistics.median averages the middle two on an
    even n. Reporting both so nobody has to guess which one a number is."""
    if not values:
        raise HardError("summarise() called on 0 flights -- vacuous verdict")
    s = sorted(values)
    return {
        "n": len(s),
        "median": statistics.median(s),
        "median_hi": s[len(s) // 2],
        "min": s[0],
        "max": s[-1],
        "inside_ram": sum(1 for v in s if v < RAM_RADIUS_M),
    }


def arm_world(index_path, row):
    """The gz world an arm flew in, read from its own run log -- the era test.
    Returns (world_name|None, source)."""
    for key in ("run_log_path", "sim_log_path"):
        p = row.get(key) or ""
        if p and os.path.exists(p):
            try:
                with open(p, errors="replace") as f:
                    head = f.read(20000)
            except OSError:
                continue
            for tok in head.split("/world/")[1:]:
                name = tok.split("/")[0].strip()
                if name:
                    return name, os.path.basename(p)
    return None, "not found"


def load_arm(index_path):
    if not os.path.exists(index_path):
        raise HardError(f"{index_path}: arm index CSV does not exist")
    with open(index_path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise HardError(f"{index_path}: arm index has 0 rows")
    if "flight_csv_path" not in rows[0]:
        raise HardError(
            f"{index_path}: no flight_csv_path column -- not an mc_batch arm "
            f"index (columns: {sorted(rows[0])[:8]}...)")
    world, world_src = arm_world(index_path, rows[0])
    flights = []
    for r in rows:
        p = r.get("flight_csv_path") or ""
        if not p:
            raise HardError(
                f"{index_path}: run_idx {r.get('run_idx')} has an empty "
                "flight_csv_path -- cannot re-score it, refusing to drop it "
                "silently")
        flights.append((p, r))
    return flights, world, world_src


def score_arm(index_path, modes, verbose=False):
    flights, world, world_src = load_arm(index_path)
    per_flight = []
    for path, row in flights:
        fl = Flight(path)
        fl.load()
        rec = {
            "flight_csv": path,
            "run_idx": row.get("run_idx"),
            "published_miss_m": _f(row, "miss_m"),
            "n_rows": fl.n_rows,
            "n_ticks": len(fl.ticks),
            "n_dropped": fl.n_dropped,
            "drop_reasons": fl.drop_reasons,
            "flags": list(fl.flags),
            "pad_source": fl.pad_source,
            "yaw_offset_deg": fl.yaw_offset_deg,
            "yaw_check_deg": fl.yaw_check_deg,
            "modes": {},
        }
        for m in modes:
            if m == "exact" and not fl.has_exact:
                continue
            rec["modes"][m] = fl.cpa(m)
        per_flight.append(rec)
        if verbose:
            bits = " ".join(
                f"{m}={rec['modes'][m]['logged_m']:.3f}/"
                f"{rec['modes'][m]['interp_m']:.3f}"
                for m in modes if m in rec["modes"])
            print(f"    {os.path.basename(path):46s} pub={rec['published_miss_m']} "
                  f"{bits}"
                  + (f"  FLAGS {rec['flags']}" if rec["flags"] else ""))
    if not per_flight:
        raise HardError(f"{index_path}: 0 flights scored -- vacuous")
    summary = {}
    for m in modes:
        vals_l = [f["modes"][m]["logged_m"] for f in per_flight if m in f["modes"]]
        vals_i = [f["modes"][m]["interp_m"] for f in per_flight if m in f["modes"]]
        if not vals_l:
            continue
        summary[m] = {"logged": summarise(vals_l), "interp": summarise(vals_i)}
    pub = [f["published_miss_m"] for f in per_flight
           if f["published_miss_m"] is not None]
    bound = None
    if "centre" in summary and "centre-tilt" in summary:
        bound = max(
            abs(f["modes"]["centre"]["interp_m"] - f["modes"]["centre-tilt"]["interp_m"])
            for f in per_flight
            if "centre" in f["modes"] and "centre-tilt" in f["modes"])
    return {
        "arm": os.path.basename(index_path),
        "index_path": index_path,
        "world": world,
        "world_source": world_src,
        "era_verified_target": world in VERIFIED_TARGET_WORLDS,
        "n_flights": len(per_flight),
        "published": summarise(pub) if pub else None,
        "summary": summary,
        "recon_bound_m": bound,
        "flights": per_flight,
        "flagged_flights": sum(1 for f in per_flight if f["flags"]),
    }


# ------------------------------------------------------------------ output --


def print_arm(res, modes):
    print(f"\n=== {res['arm']}   n={res['n_flights']} flights   "
          f"world={res['world']} ({res['world_source']})")
    if not res["era_verified_target"]:
        print("    ERA CAVEAT: target centre == model origin is verified only "
              "for fpv_quad_enemy (world quad_enemy). Interceptor-end "
              "correction still applies; the TARGET end is unverified here.")
    if res["published"] is not None:
        p = res["published"]
        print(f"    published miss_m (arm index): median {p['median']:.3f} "
              f"(hi {p['median_hi']:.3f})  inside {RAM_RADIUS_M} m: "
              f"{p['inside_ram']}/{p['n']}")
    print(f"    {'mode':16s} {'ruler':10s} {'median':>8s} {'med_hi':>8s} "
          f"{'min':>8s} {'max':>8s}  inside {RAM_RADIUS_M} m")
    for m in modes:
        if m not in res["summary"]:
            continue
        for ruler in ("logged", "interp"):
            s = res["summary"][m][ruler]
            print(f"    {m:16s} {ruler:10s} {s['median']:8.3f} "
                  f"{s['median_hi']:8.3f} {s['min']:8.3f} {s['max']:8.3f}  "
                  f"{s['inside_ram']:2d}/{s['n']}")
    if res["recon_bound_m"] is not None:
        print(f"    reconstruction bound (level vs tilt-corrected, worst "
              f"flight): {res['recon_bound_m']:.3f} m")
    if res["flagged_flights"]:
        print(f"    FLAGGED flights: {res['flagged_flights']}/{res['n_flights']}")
        for f in res["flights"]:
            if f["flags"]:
                print(f"      {os.path.basename(f['flight_csv'])}: {f['flags']}")
    dropped = sum(f["n_dropped"] for f in res["flights"])
    if dropped:
        print(f"    rows dropped (no gt): {dropped} of "
              f"{sum(f['n_rows'] for f in res['flights'])} total rows")


def gate(modes, verbose=False):
    """The issue-#8 acceptance gate over AE5dash s123+s777 (n=16)."""
    print("=" * 78)
    print("ACCEPTANCE GATE -- issue #8 published table vs this tool")
    print("=" * 78)
    all_flights = []
    results = []
    for arm in GATE_ARMS:
        p = arm if os.path.isabs(arm) else os.path.join(REPO_ROOT, arm)
        res = score_arm(p, modes, verbose=verbose)
        results.append(res)
        all_flights.extend(res["flights"])
    n = len(all_flights)
    if n != PUBLISHED_GATE["n"]:
        raise HardError(
            f"gate expected {PUBLISHED_GATE['n']} flights, scored {n} -- the "
            "gate set moved; refusing to compare against the published table")
    pooled = {}
    for m in modes:
        if not all(m in f["modes"] for f in all_flights):
            continue
        pooled[m] = {
            "logged": summarise([f["modes"][m]["logged_m"] for f in all_flights]),
            "interp": summarise([f["modes"][m]["interp_m"] for f in all_flights]),
        }
    print(f"\nPooled n={n} (AE5dash line9, seeds 123+777).")
    print(f"{'ruler':38s} {'median':>8s} {'med_hi':>8s}  inside 0.35 m")
    rows = [
        ("PUBLISHED lens, logged tick", None, None,
         PUBLISHED_GATE["lens_logged"]["median"],
         PUBLISHED_GATE["lens_logged"]["inside"]),
        ("PUBLISHED centre, logged tick", None, None,
         PUBLISHED_GATE["c2c_logged"]["median"],
         PUBLISHED_GATE["c2c_logged"]["inside"]),
        ("PUBLISHED centre, interpolated", None, None,
         PUBLISHED_GATE["c2c_interp"]["median"],
         PUBLISHED_GATE["c2c_interp"]["inside"]),
    ]
    for label, _a, _b, med, inside in rows:
        print(f"{label:38s} {med:8.3f} {'':>8s}  {inside:2d}/16")
    print()
    for m in modes:
        if m not in pooled:
            continue
        for ruler in ("logged", "interp"):
            s = pooled[m][ruler]
            print(f"{('this tool  ' + m + ', ' + ruler):38s} "
                  f"{s['median']:8.3f} {s['median_hi']:8.3f}  "
                  f"{s['inside_ram']:2d}/16")

    # -- the legacy reproduction check (did we identify what produced the table?)
    ok = True
    notes = []
    if "adhoc-published" in pooled and "lens" in pooled:
        checks = [
            ("lens, logged tick", pooled["lens"]["logged"],
             PUBLISHED_GATE["lens_logged"]),
            ("centre, logged tick", pooled["adhoc-published"]["logged"],
             PUBLISHED_GATE["c2c_logged"]),
            ("centre, interpolated", pooled["adhoc-published"]["interp"],
             PUBLISHED_GATE["c2c_interp"]),
        ]
        print("\nLEGACY REPRODUCTION (mode `adhoc-published` = the ad-hoc method:")
        print("  interceptor reference taken as (gt_cam_x, gt_cam_y, alt_m) --")
        print("  i.e. RELATIVE altitude used as a WORLD z, horizontal lever arm")
        print("  ignored entirely). Reproducing it proves we know what the")
        print("  published table measured; it is NOT the recommended ruler.")
        for label, got, want in checks:
            dm = abs(got["median_hi"] - want["median"])
            hit = dm <= 0.005 and got["inside_ram"] == want["inside"]
            ok = ok and hit
            print(f"  {label:24s} published {want['median']:.3f} / "
                  f"{want['inside']:2d}/16   tool {got['median_hi']:.3f} / "
                  f"{got['inside_ram']:2d}/16   "
                  f"{'MATCH' if hit else 'DIFFERS'}  (|d_median| {dm:.4f})")
            if not hit:
                notes.append(label)
    print("\n" + "-" * 78)
    if "centre" in pooled:
        c = pooled["centre"]
        print("PRIMARY RULER (mode `centre`: airframe centre = base_link, the "
              "\nSDF-resolved lever arm (0.120 fwd, 0.030 left, 0.002 up) "
              "rotated by\nthe pad-calibrated yaw):")
        print(f"  centre, logged tick  median {c['logged']['median']:.3f} "
              f"(hi {c['logged']['median_hi']:.3f})  "
              f"{c['logged']['inside_ram']:2d}/16 inside {RAM_RADIUS_M} m")
        print(f"  centre, interpolated median {c['interp']['median']:.3f} "
              f"(hi {c['interp']['median_hi']:.3f})  "
              f"{c['interp']['inside_ram']:2d}/16 inside {RAM_RADIUS_M} m")
        print("\nDISCREPANCY WITH THE PUBLISHED TABLE -- see the module "
              "docstring:\n  the published table's 0.208 m vertical drop is "
              "NOT a camera lever arm. It is\n  median(gt_cam_z - alt_m), and "
              "alt_m is altitude above the TAKEOFF POINT,\n  not world z, so "
              "that number is dominated by the x500's LANDING-GEAR\n  HEIGHT "
              "(base_link rests 0.224 m above the pad). The camera sits ~0.002 m"
              "\n  above base_link and 0.120 m FORWARD of it. Correcting to the "
              "airframe\n  centre therefore moves the reference BACKWARD along "
              "the body axis, not\n  DOWNWARD -- and the miss numbers do not "
              "improve the way the table says.")
    print("-" * 78)
    return ok, notes, results, pooled


def markdown_table(results, modes, out_path):
    """The old-vs-new per-arm table. `modes` is asserted, not decorative: the
    columns below are fixed, so a caller that dropped one of them would
    otherwise print a silently empty column."""
    for need in ("lens", "centre", "centre-tilt"):
        if need not in modes:
            raise HardError(
                f"--markdown needs mode {need!r} (got {modes}) -- refusing to "
                "write a table with a silently empty column")
    lines = []
    lines.append("| arm | n | published median / inside | "
                 "lens logged | centre logged | centre interp | "
                 "centre-tilt interp | recon bound |")
    lines.append("|---|---|---|---|---|---|---|---|")
    for r in results:
        p = r["published"]
        pub = (f"{p['median']:.3f} / {p['inside_ram']}/{p['n']}" if p else "-")

        def cell(mode, ruler):
            if mode not in r["summary"]:
                return "-"
            s = r["summary"][mode][ruler]
            return f"{s['median']:.3f} / {s['inside_ram']}/{s['n']}"

        bound = ("-" if r["recon_bound_m"] is None
                 else f"{r['recon_bound_m']:.3f}")
        era = "" if r["era_verified_target"] else " *(era caveat)*"
        lines.append(
            f"| {r['arm'].replace('.csv', '')}{era} | {r['n_flights']} | {pub} | "
            f"{cell('lens', 'logged')} | {cell('centre', 'logged')} | "
            f"{cell('centre', 'interp')} | {cell('centre-tilt', 'interp')} | "
            f"{bound} |")
    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return out_path


# --------------------------------------------------------------------- CLI --


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Re-score flight CSVs to airframe-centre CPA (issue #8).",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flight", nargs="*", default=[],
                    help="per-tick flight CSV(s) written by m4_intercept.py")
    ap.add_argument("--arm", nargs="*", default=[],
                    help="mc_batch arm index CSV(s) (with flight_csv_path)")
    ap.add_argument("--all", action="store_true",
                    help="every arm in the sweep set "
                         f"({', '.join(SWEEP_GLOBS)})")
    ap.add_argument("--gate", action="store_true",
                    help="the issue-#8 acceptance gate (AE5dash s123+s777)")
    ap.add_argument("--mode", nargs="*", default=None,
                    help=f"reference modes to report (default: all of {MODES})")
    ap.add_argument("--json", default=None, help="write full results as JSON")
    ap.add_argument("--markdown", default=None,
                    help="write the per-arm old-vs-new table as markdown")
    ap.add_argument("--verbose", action="store_true",
                    help="per-flight lines")
    args = ap.parse_args(argv)

    modes = list(args.mode) if args.mode else [m for m in MODES if m != "exact"]
    for m in modes:
        if m not in MODES:
            print(f"[rescore] unknown --mode {m!r}; modes: {MODES}",
                  file=sys.stderr)
            return 2

    if not (args.flight or args.arm or args.all or args.gate):
        ap.print_help()
        return 2

    payload = {"ram_radius_m": RAM_RADIUS_M,
               "lever_body_flu_m": [CAM_FWD_M, CAM_LEFT_M, CAM_UP_M],
               "modes": modes, "arms": [], "flights": []}
    hard_errors = []
    results = []

    try:
        if args.gate:
            ok, notes, gate_results, pooled = gate(modes, verbose=args.verbose)
            payload["gate"] = {
                "published": PUBLISHED_GATE,
                "legacy_reproduced": ok,
                "mismatched_rows": notes,
                "pooled": pooled,
            }
            results.extend(gate_results)
            if not ok:
                print("\n[rescore] GATE: the legacy ad-hoc table did NOT "
                      "reproduce -- investigate before trusting either number.",
                      file=sys.stderr)

        arms = list(args.arm)
        if args.all:
            found = []
            for g in SWEEP_GLOBS:
                found.extend(sorted(glob.glob(os.path.join(REPO_ROOT, g))))
            if not found:
                raise HardError(
                    f"--all matched 0 arm index files under {REPO_ROOT}/logs "
                    f"(globs {SWEEP_GLOBS}) -- refusing an empty sweep")
            arms.extend(found)
        for a in arms:
            try:
                res = score_arm(a, modes, verbose=args.verbose)
            except HardError as e:
                hard_errors.append(str(e))
                print(f"[rescore] HARD ERROR: {e}", file=sys.stderr)
                continue
            results.append(res)
            print_arm(res, modes)

        for fpath in args.flight:
            try:
                fl = Flight(fpath)
                fl.load()
                rec = {"flight_csv": fpath, "n_rows": fl.n_rows,
                       "n_ticks": len(fl.ticks), "n_dropped": fl.n_dropped,
                       "drop_reasons": fl.drop_reasons, "flags": fl.flags,
                       "pad_source": fl.pad_source,
                       "yaw_check_deg": fl.yaw_check_deg, "modes": {}}
                for m in modes:
                    if m == "exact" and not fl.has_exact:
                        continue
                    rec["modes"][m] = fl.cpa(m)
                payload["flights"].append(rec)
                print(f"\n--- {fpath}  ({len(fl.ticks)}/{fl.n_rows} usable "
                      f"ticks, {fl.pad_source})")
                for m, r in rec["modes"].items():
                    print(f"    {m:16s} logged {r['logged_m']:.4f} m   "
                          f"interp {r['interp_m']:.4f} m")
                if fl.flags:
                    print(f"    FLAGS: {fl.flags}")
            except HardError as e:
                hard_errors.append(str(e))
                print(f"[rescore] HARD ERROR: {e}", file=sys.stderr)
    except HardError as e:
        print(f"[rescore] HARD ERROR: {e}", file=sys.stderr)
        return 1

    # de-duplicate arms (the gate arms may also appear in --all)
    seen = set()
    uniq = []
    for r in results:
        if r["index_path"] in seen:
            continue
        seen.add(r["index_path"])
        uniq.append(r)
    payload["arms"] = uniq

    if args.markdown:
        if not uniq:
            print("[rescore] HARD ERROR: --markdown requested but 0 arms were "
                  "scored -- refusing to write an empty table", file=sys.stderr)
            return 1
        try:
            markdown_table(uniq, modes, args.markdown)
        except HardError as e:
            print(f"[rescore] HARD ERROR: {e}", file=sys.stderr)
            return 1
        print(f"\n[rescore] markdown table -> {args.markdown}")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(payload, f, indent=1, default=str)
        print(f"[rescore] json -> {args.json}")

    n_flights = sum(r["n_flights"] for r in uniq) + len(payload["flights"])
    n_flagged = sum(r["flagged_flights"] for r in uniq)
    print(f"\n[rescore] scored {n_flights} flights across {len(uniq)} arms; "
          f"{n_flagged} flagged; {len(hard_errors)} hard errors")
    if hard_errors:
        print("[rescore] HARD ERRORS (no numbers were emitted for these):",
              file=sys.stderr)
        for e in hard_errors:
            print(f"  - {e}", file=sys.stderr)
        return 1
    if n_flights == 0:
        print("[rescore] HARD ERROR: 0 flights scored -- vacuous run",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
