#!/usr/bin/env python3
"""P0.3 -- the TWO-CURVE tripod SCORER: turn one capture session into the two
MONEY-GATE curves the field afternoon exists to produce (build_plan P2,
docs/tripod_test_protocol.md, NEXT.md P0/R5).

ONE capture set (target quad + AprilTag placard, camera on a fixed tripod),
scored two ways, gating two different purchases:

  Curve (a) -- AprilTag DECODE ENVELOPE: decode rate + recovered range vs the
    frame's TRUE range. Gates the ~$740 Tier-2 interceptor order -- the staged
    first kills fly the tag36h11 placard on the Pi 5 CPU (docs/hardware_order_
    list.md §0c/§0d), so the tag must decode the approaching placard early
    enough to leave t_go >= 0.5 s post-handoff at the closing speed (the NEXT.md
    R5 kill-number applied to the tag). This is the curve that stops -- or
    unlocks -- $740.

  Curve (b) -- NN APPROACH RECALL vs range × position-in-frame: the deployed
    markerless detector's (drone_finetuned_quad_v2) real-target recall on the
    SAME footage, tag-truthed (the autolabel approach -- project the drone box
    from the decoded tag pose, then ask whether the NN fires there). Binned by
    range AND by position-in-frame, because recall-vs-range ALONE hid a
    100%-static-vs-0.8%-in-flight gap that only position-in-frame explained
    (ADR-0076 add #18k). Gates ONLY the deferred $70 Hailo HAT+ / markerless
    phase -- NEVER the interceptor order.

HONESTY (CLAUDE.md, constraint honesty-boundary): this is an OFFLINE SCORING
tool run after the flight has landed -- the same class as field_score.py, NOT
the sim's gt_* guidance-cheat (which is about what the FLYING software may read
mid-flight). Decoding the AprilTag is exactly its sanctioned use (calibration /
baseline seeker / auto-labels); using the tag pose to truth the NN is the
sanctioned training/scoring label source (autolabel_from_apriltag.py). Nothing
here feeds live guidance.

SESSION LAYOUT (what scripts/seeker/pi_capture.py ACTUALLY writes -- verified
against its INDEX_HEADER/TAGS_HEADER, 2026-07-24):

    session_dir/
      frames/*.png   raw frames (PNG, not compressed video -- protocol §6),
                     zero-padded capture order: 000000.png, 000001.png, ...
      index.csv      one row per frame. pi_capture's columns are
                     `frame_idx, frame_path, t_mono_s, t_wall_unix,
                      exposure_us, gain`; older/synthetic sessions may instead
                     key on `frame`/`filename`/`name`. BOTH bind (load_session
                     joins on frame_path's basename, then on frame_idx).
                     The scorer needs a TRUE range per frame -- it reads
                     `true_range_m`, which scripts/seeker/range_truth_join.py
                     writes by joining the target's flight log + the surveyed
                     tripod position to each frame's `t_wall_unix` (protocol
                     §6/§6.1) -- else parses `_r<num>_` from the frame filename
                     (the synth_tag_frames / resolution_probe convention), else
                     that frame is unbinnable and skipped. range_truth_join also
                     writes `range_quality` / `range_src_dt_s` / `range_sigma_m`;
                     frames it could not truth carry an EMPTY true_range_m and a
                     reject flag, never a fabricated number.
      meta.json      session metadata: `tag_size_m`, `drone_size_m`,
                     `stream_fps` (+ `stream_fps_source`), `closing_speed_mps`,
                     `tilt_deg`, `aspect`...
      tags.csv       OPTIONAL pre-computed AprilTag decode per frame. BOTH
                     schemas are read: pi_capture's capture-time QC record
                     (`frame_idx,frame_path,n_tags,tag_id,...,range_m`, one row
                     per tag) and this scorer's own output
                     (`frame,decoded,tag_range_m,u_px,v_px,n_tags`). If present
                     it is reused (decouples decode from scoring, protocol §6);
                     `--redecode` forces a fresh offline decode instead, which
                     is what protocol §6 actually asks for. If absent the scorer
                     decodes with pupil-apriltags and writes a tags.csv into the
                     OUTPUT dir.

STREAM FPS IS A MEASUREMENT, NOT A DEFAULT (2026-07-24). The gate's streak burn
is `R_streak_burn = (E[T] / stream_fps) x V_closing`, so the frame rate moves the
verdict directly. At the PREDICTED R_decode90 for the adopted 0.35 m placard
(7.10 m realistic, docs/placard_sizing.md §4 / placard_mount.md §11) with k=5 and
V=9 m/s:

    p = 0.90 (the WORST rate the >=90% sustain band allows -- E[T]=6.94 fr):
        30 fps -> t_go 0.558 s PASS | 20 Hz -> 0.442 s FAIL | 14 Hz -> 0.293 s FAIL
        break-even 24.0 fps
    p = 1.00 (a perfect band -- E[T]=5 fr):
        30 fps -> t_go 0.622 s PASS | 20 Hz -> 0.539 s PASS | 14 Hz -> 0.432 s FAIL
        break-even 17.3 fps

So the SAME optical data buys or blocks the ~$740 order depending on a number
that was previously an unsourced 30.0 constant -- while the deployed flight loop
runs 20 Hz and the only measured detector cadence in this repo is ~14 Hz. The
scorer NO LONGER invents one: the fps must come from the session's
`meta.json:stream_fps` (measured by pi_capture from real capture timestamps) or
an explicit `--stream-fps` (bench it -- protocol §7.3 Pi 5 compute bench). With
neither, the gate returns UNCERTAIN naming the missing measurement.

DEPS / WHICH VENV (mirrors the repo's .venv vs .venv-seeker split):
  * Curve (a) + plots  -> pupil-apriltags + cv2 + matplotlib (the MAIN .venv).
  * Curve (b) NN half  -> onnxruntime + cv2 (.venv-seeker; onnxruntime is NOT
    in the main .venv). If onnxruntime or the .onnx weights are unavailable the
    NN half CLEANLY NO-OPS with a message -- curve (a) + the money gate still
    produce their answer. Plots also degrade gracefully if matplotlib is absent.

Usage:
    scripts/seeker/tripod_score.py SESSION_DIR --calib calib.json           # curve (a) + gate
    scripts/seeker/tripod_score.py SESSION_DIR --calib calib.json \
        --weights scripts/seeker/weights/drone_finetuned_quad_v2.onnx        # + curve (b)
    scripts/seeker/tripod_score.py --self-test    # synthetic session, no hardware; exit 0/1
"""
import argparse
import collections
import csv
import glob
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

# ONE implementation of the two capture-side primitives (CLAUDE.md "fix root
# cause", one code path): the recorder OWNS the detector's quad_decimate default
# and the tag-incidence geometry, and the scorer must not fork either. pi_capture
# imports only the stdlib at module scope, so this is safe under .venv-seeker too.
from pi_capture import (  # noqa: E402
    DEFAULT_QUAD_DECIMATE,
    _neutralize_detector_del,
    tag_incidence_deg,
)

# --------------------------------------------------------------------------
# Sourced constants -- every number traces to a doc/ADR (CLAUDE.md rule).
# --------------------------------------------------------------------------
# The money-gate kill number: the tag must decode early enough to leave this
# much t_go AFTER the handoff streak forms (NEXT.md R5, tripod_test_protocol.md
# §8.1, build_plan P2 gate, hardware_order_list.md §0d).
DEFAULT_TGO_MIN_S = 0.5
# Conservative closing speed = target-only ~9 m/s (>=20 mph goal, NEXT.md R5:
# "R_acq must leave t_go >=0.5 s post-handoff (≈>=20 m for 9 m/s)"). The gate
# decision is made against THIS scenario (protocol §8.1 "GO: t_go >= 0.5 s under
# the conservative (9 m/s) scenario at minimum").
DEFAULT_V_CLOSING_MPS = 9.0
# Aggressive scenario (reported for context, NOT the gate): interceptor dash
# ~16 m/s combined with the target head-on ~20-25 m/s (protocol §8.1).
DEFAULT_V_CLOSING_HI_MPS = 20.0
# Handoff streak = 5 CONSECUTIVE detections (NEXT.md R5 / protocol §8.1
# "the 5-consecutive-detection handoff streak"). The coded-dash code's own
# HANDOFF_STREAK_MIN is smaller (3, m4_intercept.py); R5 pre-registers 5 for the
# tag gate, so 5 is the sourced number here (overridable).
DEFAULT_HANDOFF_STREAK = 5
# Stream/decode rate: there is deliberately NO DEFAULT (2026-07-24). The
# ~30 fps figure in protocol §7.3 is a PAPER ANCHOR ("AprilTag ~30 fps
# CPU-real-time") that §7.3 itself schedules to be BENCH-MEASURED on the real
# Pi 5 -- and the verdict flips across it (module docstring: 30 fps PASS vs
# 20 Hz FAIL at the predicted 7.10 m R_decode90; break-even 24.0 fps). Using it
# as a silent fallback would decide ~$740 on an unmeasured constant. It is kept
# ONLY as the help-text anchor and as the self-test's stand-in.
PI5_APRILTAG_FPS_PAPER_ANCHOR = 30.0
# meta.json `stream_fps_source` values containing this substring are NOT a
# capture-rate measurement (pi_capture's dir-replay backend stamps synthetic
# timestamps) and are refused as a gate input. RETAINED as a second belt behind
# the source WHITELIST below -- it catches a replay-flavoured rate even inside an
# otherwise-sanctioned session.
FPS_SOURCE_REJECT_MARK = "replay"
# CAPTURE-SOURCE WHITELIST for the gate's fps divisor (BUILDER RULING 2026-07-25:
# "may a --local/v4l2 desk rehearsal ever supply the gate's fps?" -> NO).
#
# These are pi_capture's `meta.json:source` values that MAY feed the money gate's
# 1/fps divisor. `picamera2` is the real Pi 5 + libcamera + OV9281 path -- the
# only capture in this project whose cadence is a property of the hardware that
# will actually fly. Everything else is refused BY DEFAULT, including:
#   v4l2  a laptop/USB webcam desk rehearsal -- a different CPU, a different
#         driver, a different sensor; its rate says nothing about the Pi 5
#   dir    replay of stored frames -- synthetic timestamps
#   (absent) an unidentified or hand-written meta.json -- unknown provenance
# WHY A WHITELIST AND NOT A BLACKLIST: the check this replaces enumerated the BAD
# case ("replay") and accepted everything else, so a v4l2 desk session with decode
# on silently supplied the divisor for a ~$740 verdict. Enumerating the ONE good
# case fails closed when a new source appears.
# The sanctioned override is the protocol's own: bench the Pi 5 (§7.3) and pass
# --stream-fps explicitly.
ALLOWED_FPS_CAPTURE_SOURCES = ("picamera2",)
# ADOPTED placard: tag36h11 black-square edge 0.350 m on a 0.450 m sheet
# (docs/placard_mount.md §2 table "Tag black-square edge = 0.350 m"; the carry
# limit from docs/placard_sizing.md §4). This is the AprilTag black-square edge
# that pupil-apriltags' `tag_size` measures -- NOT the printed sheet.
#
# CORRECTED 2026-07-25 (review 2, MEDIUM "silently substitutes defaults"): this
# constant was 0.3 while the adopted placard is 0.35, so any session whose
# meta.json lacked tag_size_m had EVERY tag-recovered range read ~14% short
# (range scales linearly in the assumed tag edge) -- which feeds curve (a)'s
# accuracy column, curve (b)'s truth boxes and range_truth_join's bias test.
# Falling back at all is still second-best: run() now also names the SOURCE of
# the number (see tag_size_source in gate.json / the PARAMETER PROVENANCE block).
DEFAULT_TAG_SIZE_M = 0.35
# Target drone extent for the NN truth box (5" quad ~0.35 m; autolabel default).
DEFAULT_DRONE_SIZE_M = 0.35
# Markerless detector for curve (b). TWO models are scored on the SAME frames:
#
#   PRIMARY (the candidate)  n-mono -- YOLO11n, COCO-init, GRAYSCALE-native, trained
#     on the 15,391-image real-media corpus. On a source-disjoint REAL held-out set
#     (n=4175): AP50 0.442 / recall 44.2% / precision 71.4% / false-fire 4.9%
#     (logs/nn_tier/eval_n-mono_heldout.csv, docs/nn_tier/PLAN.md).
#   BAR (historical)  drone_finetuned_quad_v2 -- the SIM-trained deployed model, which
#     on that same real held-out set is BLIND: AP50 0.0003 / recall 1.1% / false-fire
#     88.5% (logs/nn_tier/eval_s-mono_summary_cmp1.csv).
#
# WHY BOTH: scoring curve (b) with quad_v2 ALONE produces a near-zero recall curve on
# real frames that, read naively, would wrongly damn the markerless phase (and the
# $70 Hailo HAT it gates) when what it actually measures is the already-known
# sim->real NULL. Reporting both makes the sim-vs-real delta explicit on OUR frames.
# Curve (b) gates ONLY the Hailo/markerless phase -- never the interceptor order.
DEFAULT_WEIGHTS = os.path.join(HERE, "weights", "nn_tier", "n-mono.onnx")
DEFAULT_WEIGHTS_BAR = os.path.join(HERE, "weights", "drone_finetuned_quad_v2.onnx")
DEFAULT_NN_CONF = 0.25  # ADR-0058 deployed conf (project_state.json)

# Data-presence floor for the verdict -- NOT a statistical bar. A field
# afternoon cannot buy CLAUDE.md's n>=8 paired-seed standard (protocol §4.6),
# so this only guards against "essentially no data"; the small-n caveat is
# ALWAYS printed regardless.
DEFAULT_MIN_DECODED = 5

# Per-BIN sample floor for the R_decode90 walk (2026-07-25, review 2 HIGH).
# A range bin may not EXTEND the >=90% envelope on fewer than this many binnable
# frames: one lucky far decode reads rate 1.00 and would push the envelope (and
# therefore the ~$740 gate) a whole bin outward on n=1. Mirrors
# DEFAULT_MIN_DECODED's spirit, applied per bin instead of per session.
DEFAULT_MIN_BIN_N = 5

# R_decode90 walk stop reasons. Only `rate_below_90` is a MEASURED cutoff; the
# other three mean the envelope is bounded by MISSING DATA, which gate_verdict
# turns into UNCERTAIN rather than PASS/FAIL (see r90_walk / gate_verdict).
R90_STOP_RATE = "rate_below_90"
R90_STOP_ABSENT = "bin_absent"
R90_STOP_UNDERPOPULATED = "bin_underpopulated"
R90_STOP_END = "end_of_data"
R90_DATA_BOUNDED = (R90_STOP_ABSENT, R90_STOP_UNDERPOPULATED, R90_STOP_END)

# INCIDENCE (2026-07-25, protocol §4.2b + docs/placard_mount.md §11 item 9).
# The adopted placard is a single BEAM-FACING panel (placard_mount DECISION 1), so
# decode range falls as R(0)*cos(theta) with theta = the angle between the tag
# normal and the line of sight. A pass flown at 30 deg incidence and a pass flown
# at 0 deg are DIFFERENT EXPERIMENTS -- pooling them smears curve (a), and a GO
# measured at 0 deg does not transfer to a 30 deg engagement (§8.1).
# 15 deg bins because the mount is 15-deg-INDEXABLE (placard_mount §3.6), so the
# bin edges line up with the settings the operator can actually fly.
DEFAULT_INCIDENCE_BIN_DEG = 15.0
# The sim evidence for the cos-theta law stops at 32.6 deg; above 33 deg it is a
# listed HYPOTHESIS (placard_mount §13). Cells beyond it are FLAGGED, not dropped.
COS_LAW_VALIDATED_TO_DEG = 32.6

_RANGE_RE = re.compile(r"_r(\d+\.?\d*)_")   # synth_tag_frames / resolution_probe convention
_TRUE = ("1", "true", "yes", "t")
_REPO_ROOT_TS = os.path.dirname(os.path.dirname(HERE))


def _stem(path):
    """Filesystem-safe model label from a weights path ('.../n-mono.onnx' -> 'n-mono')."""
    return re.sub(r"[^A-Za-z0-9._-]", "_",
                  os.path.splitext(os.path.basename(str(path)))[0]) or "model"


# --------------------------------------------------------------------------
# Session + calibration loading
# --------------------------------------------------------------------------
def load_calib(path):
    """fx,fy,cx,cy + resolution + distortion. Accepts calibrate_camera.py /
    flight.camera / plain-intrinsics JSON (same reader as
    autolabel_from_apriltag._load_calib)."""
    d = json.loads(open(path).read())
    fx, fy, cx, cy = float(d["fx"]), float(d["fy"]), float(d["cx"]), float(d["cy"])
    res = d.get("resolution", {})
    w = int(res.get("width", d.get("width", 0)))
    h = int(res.get("height", d.get("height", 0)))
    dist = d.get("dist_coeffs", d.get("dist", [0.0] * 5))
    return fx, fy, cx, cy, w, h, np.asarray(dist, dtype=float).ravel()


def _read_csv(path):
    if not os.path.exists(path):
        return None
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


# Columns any of the session CSVs may use to name a frame. `frame_path` is what
# pi_capture ACTUALLY writes (INDEX_HEADER/TAGS_HEADER); the other three are the
# older synth/probe convention. Missing this list was the silent schema break
# that made every real session unbinnable (n_binnable=0 -> UNCERTAIN), fixed
# 2026-07-24 -- see tests/test_tripod_pipeline_join.py.
_FRAME_NAME_COLS = ("frame", "filename", "name", "frame_path", "file", "path")
_FRAME_IDX_COLS = ("frame_idx", "idx", "index", "i")


def _row_frame_key(row, base_set, bases):
    """Resolve one CSV row to a frame basename, trying (1) any filename-ish
    column, (2) the zero-padded frame_idx (pi_capture names frames %06d.png),
    (3) frame_idx as a POSITION in capture order. Returns None if unresolvable.

    THE NAME IS AUTHORITATIVE (2026-07-26). A row that NAMES a frame file which
    is not on disk is UNRESOLVABLE -- it is never re-bound to some other frame.
    The old code fell through to the positional branch (3), so deleting the
    ground/setup frames (where the placard is held NEAR the camera) before
    scoring re-bound those rows' 1-3 m capture-time tag ranges onto the first
    IN-FLIGHT frames, and the nearest-tag merge then let a deleted frame's
    smaller range replace a surviving frame's correct decode. Curve (a)'s range
    accuracy, the survey cross-check and curve (b)'s truth boxes all shifted
    toward NEAR with every check green. Positional binding survives ONLY for a
    legacy row that carries no filename column at all."""
    fn = ""
    for c in _FRAME_NAME_COLS:
        v = (row.get(c) or "").strip()
        if v:
            fn = v
            break
    if fn:
        key = os.path.splitext(os.path.basename(fn))[0]
        return key if key in base_set else None
    for c in _FRAME_IDX_COLS:
        iv = _pf(row.get(c))
        if iv is None:
            continue
        i = int(iv)
        cand = f"{i:06d}"
        if cand in base_set:
            return cand
        if 0 <= i < len(bases):
            return bases[i]
    return None


def _tag_row_range(row):
    """AprilTag range from EITHER tags.csv schema (tripod_score's `tag_range_m`
    or pi_capture's `range_m`)."""
    r = _pf(row.get("tag_range_m"))
    return r if r is not None else _pf(row.get("range_m"))


def _merge_tag_rows(old, new):
    """pi_capture writes ONE ROW PER TAG, so a frame can repeat. Keep the
    NEAREST tag -- the same rule decode_frames uses on live detections
    (`min(dets, key=pose_t[2])`). Rows without a range keep the first seen."""
    ro, rn = _tag_row_range(old), _tag_row_range(new)
    if rn is None:
        return old
    if ro is None:
        return new
    return new if rn < ro else old


def load_session(session_dir, ignore_tags_csv=False, stats=None):
    """Return (frames, index_map, meta, tags_map). `frames` is a sorted list of
    (path, basename); `index_map`/`tags_map` are {basename: row} or None.

    `stats`, when a dict is passed in, is filled with the ORPHAN ROW COUNTS --
    rows naming a frame that is not in frames/. Dropping them silently would be
    the same uncounted-denominator defect as the misbinding they replace, so the
    caller (run()) reports them and refuses by default. The return arity is
    deliberately unchanged; several tests unpack it as a 4-tuple."""
    frames_dir = os.path.join(session_dir, "frames")
    if not os.path.isdir(frames_dir):
        raise SystemExit(f"no frames/ subdir in session {session_dir}")
    paths = sorted(glob.glob(os.path.join(frames_dir, "*.png")) +
                   glob.glob(os.path.join(frames_dir, "*.jpg")))
    if not paths:
        raise SystemExit(f"no frames in {frames_dir}")
    frames = [(p, os.path.splitext(os.path.basename(p))[0]) for p in paths]
    bases = [b for _p, b in frames]
    base_set = set(bases)

    counts = {}

    def _keymap(rows, which, merge=None):
        if rows is None:
            return None
        out, orphans = {}, 0
        for r in rows:
            key = _row_frame_key(r, base_set, bases)
            if not key:
                orphans += 1
                continue
            if key in out and merge is not None:
                out[key] = merge(out[key], r)
            else:
                out[key] = r
        counts[which] = {"rows": len(rows), "orphan_rows": orphans}
        return out

    index_map = _keymap(_read_csv(os.path.join(session_dir, "index.csv")), "index")
    tags_map = None if ignore_tags_csv else _keymap(
        _read_csv(os.path.join(session_dir, "tags.csv")), "tags",
        merge=_merge_tag_rows)
    if stats is not None:
        stats.update(counts)
    meta = {}
    mp = os.path.join(session_dir, "meta.json")
    if os.path.exists(mp):
        try:
            meta = json.loads(open(mp).read())
        except (ValueError, OSError):
            meta = {}
    return frames, index_map, meta, tags_map


def _pf(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


# Per-frame quality flags range_truth_join.py writes. A frame it could not truth
# carries an EMPTY true_range_m AND one of these, so it is unbinnable either way;
# `--require-quality ok` additionally drops the flagged-but-usable ones.
QUALITY_REJECT = ("extrapolated", "no_frame_time")


def truth_range_for(basename, index_map, accept_quality=None):
    """TRUE range for a frame: index.csv `true_range_m` (written by
    scripts/seeker/range_truth_join.py from the target's flight log + the
    surveyed tripod position, protocol §6/§6.1), else `_r<num>_` parsed from the
    filename, else None (unbinnable).

    `accept_quality` (a set, optional) restricts which `range_quality` flags are
    binnable; QUALITY_REJECT flags are ALWAYS dropped so a frame the join could
    not truth can never sneak in as a number."""
    if index_map and basename in index_map:
        row = index_map[basename]
        q = (row.get("range_quality") or "").strip()
        if q and (q in QUALITY_REJECT or
                  (accept_quality is not None and q not in accept_quality)):
            return None
        r = _pf(row.get("true_range_m") or row.get("gt_range") or
                row.get("range_m"))
        if r is not None:
            return r
    m = _RANGE_RE.search(basename)
    return float(m.group(1)) if m else None


def quality_summary(frames, index_map):
    """{range_quality flag: n frames} over the session (blank -> 'unflagged').
    Surfaces the join's honesty flags in the scorer's own report."""
    counts = {}
    if not index_map:
        return counts
    for _p, base in frames:
        row = index_map.get(base)
        if row is None:
            counts["no_index_row"] = counts.get("no_index_row", 0) + 1
            continue
        q = (row.get("range_quality") or "").strip() or "unflagged"
        counts[q] = counts.get(q, 0) + 1
    return counts


# --------------------------------------------------------------------------
# AprilTag decode (curve a raw input)
# --------------------------------------------------------------------------
# Per-frame decode record. It IS a 6-tuple, so every legacy `decode[base][0]`
# index still works, and a caller that hand-builds the historical 5-tuple
# (decoded, range, u, v, n_tags) still binds through as_obs() with
# incidence_deg=None. New code should read the NAMED fields.
TagObs = collections.namedtuple(
    "TagObs", "decoded range_m u v n_tags incidence_deg")


def as_obs(value):
    """Coerce a decode-map value to TagObs. Accepts TagObs, the historical
    5-tuple, or any short tuple (missing fields become None -- never 0.0, which
    would be a fabricated measurement)."""
    if isinstance(value, TagObs):
        return value
    t = tuple(value)
    if len(t) > 6:
        t = t[:6]
    return TagObs(*(t + (None,) * (6 - len(t))))


def check_decimate_consistency(values, allow_mismatch=False):
    """REFUSE to pool decodes produced at DIFFERENT quad_decimate settings
    (2026-07-25, ADR-0082).

    `values` = {label: quad_decimate or None}. Returns a dict describing what was
    compared; raises SystemExit when two RECORDED values disagree and
    `allow_mismatch` is False.

    WHY THIS IS A REFUSAL AND NOT A WARNING. quad_decimate is a RANGE MULTIPLIER:
    qd=1.0 decodes ~1.5x farther and over a +-48-56 deg incidence cone vs +-32 deg
    at qd=2.0 (docs/placard_mount.md §3.3). Curve (a) is a decode-vs-range curve,
    so mixing two decimations inside one curve -- or scoring a session at a
    decimate other than the one that produced its tags.csv -- silently reports an
    envelope no single configuration ever flew, and the ~$740 gate reads it as
    optical truth. Protocol §4.2b's legitimate qd in {2.0, 1.0} comparison is a
    DELIBERATE re-decode of the same frames: run it with --allow-decimate-mismatch,
    which stamps the divergence into gate.json + verdict.txt.

    A value of None means the session predates the stamp (or was captured by a
    tool that does not record it). That is UNKNOWN, not equal: it cannot trigger a
    mismatch refusal (there is nothing to compare), and the caller must surface it
    as unknown provenance rather than as agreement."""
    known = {k: float(v) for k, v in values.items() if v is not None}
    unknown = sorted(k for k, v in values.items() if v is None)
    distinct = sorted(set(known.values()))
    mismatch = len(distinct) > 1
    info = {
        "values": {k: float(v) if v is not None else None
                   for k, v in values.items()},
        "unknown_sources": unknown,
        "distinct_recorded": distinct,
        "mismatch": mismatch,
        "mismatch_allowed": bool(allow_mismatch),
    }
    if mismatch and not allow_mismatch:
        detail = ", ".join(f"{k}={known[k]}" for k in sorted(known))
        raise SystemExit(
            f"[tripod] REFUSED: quad_decimate MISMATCH ({detail}). quad_decimate "
            f"is a range multiplier (~1.5x range and a +-48-56 deg vs +-32 deg "
            f"incidence cone at 1.0 -- docs/placard_mount.md §3.3), so decodes "
            f"produced at different settings do not belong on one curve (a) and "
            f"the ~$740 gate would read an envelope no configuration ever flew. "
            f"Re-run at the value the session recorded, or -- for protocol §4.2b's "
            f"deliberate qd in {{2.0, 1.0}} reclaim comparison -- pass "
            f"--allow-decimate-mismatch (it is stamped into gate.json/verdict.txt "
            f"and the --stream-fps you supply must be the §7.3 Pi 5 bench AT THE "
            f"SCORED DECIMATE, not at the flying one).")
    return info


# Detector objects are CACHED per quad_decimate and never destroyed. The
# pupil_apriltags C Detector.__del__ frees native state that SIGSEGVs when
# construction/destruction churns in one process (measured on CI: scoring
# several sessions in one pytest process crashed at tripod_score.run). A single
# scoring pass builds one Detector, so the cache is a no-op there; it only
# matters across many in-process calls (the test suite, a batch tool). Mirrors
# scripts/seeker/pi_capture.py:_make_detector.
_DETECTOR_CACHE = {}


def _make_detector(quad_decimate=DEFAULT_QUAD_DECIMATE):
    key = float(quad_decimate)
    det = _DETECTOR_CACHE.get(key)
    if det is not None:
        return det
    try:
        from pupil_apriltags import Detector
    except ImportError:  # aarch64: pyapriltags drop-in (docs/pi_emulation_check.md)
        from pyapriltags import Detector
    _neutralize_detector_del(Detector)
    det = Detector(families="tag36h11", quad_decimate=key)
    _DETECTOR_CACHE[key] = det
    return det


def decode_frames(frames, calib, tag_size_m, tags_map=None, quiet=False,
                  allow_partial_tags=False,
                  quad_decimate=DEFAULT_QUAD_DECIMATE):
    """Per-frame AprilTag decode -> {basename: (decoded, tag_range_m, u, v, n)}.
    Reuses a provided tags.csv (tags_map) verbatim; otherwise decodes the frames
    with pupil-apriltags (undistorting first, like autolabel_from_apriltag).

    PARTIAL-COVERAGE REFUSAL (2026-07-25, review 2). A frame with no tags.csv row
    scores as NOT DECODED, so a tags.csv truncated by an interrupted capture or a
    partial copy silently converts its missing (typically NEAR, high-rate) frames
    into decode FAILURES -- deflating curve (a), collapsing R_decode90 and moving
    the ~$740 gate, with no error. The join is now held to range_truth_join's
    standard: refuse unless every frame is covered. `--allow-partial-tags` is the
    deliberate escape hatch and stamps the count into the summary."""
    fx, fy, cx, cy, w, h, dist = calib
    results = {}
    if tags_map is not None:
        missing = [b for _, b in frames if b not in tags_map]
        if missing and not allow_partial_tags:
            raise SystemExit(
                f"[tripod] REFUSED: tags.csv covers {len(frames) - len(missing)} of "
                f"{len(frames)} frames (first missing: {missing[0]}). Uncovered "
                f"frames would score as decode FAILURES and silently deflate curve "
                f"(a). Re-copy the session, re-run with --redecode, or accept the "
                f"deflation explicitly with --allow-partial-tags.")
        if missing and not quiet:
            print(f"[tripod] [WARN] --allow-partial-tags: {len(missing)} of "
                  f"{len(frames)} frames have NO tags.csv row and are scored as "
                  f"decode FAILURES")
        for _, base in frames:
            row = tags_map.get(base)
            if row is None:
                results[base] = TagObs(False, None, None, None, 0, None)
                continue
            # BOTH tags.csv schemas (see load_session's docstring): this
            # scorer's `decoded` boolean, or pi_capture's `n_tags`/`tag_id`
            # capture-time QC record. Reading only `decoded` silently scored
            # every pi_capture row as NOT decoded (schema break, fixed
            # 2026-07-24).
            n_tags = int(_pf(row.get("n_tags")) or 0)
            dv = str(row.get("decoded", "")).strip()
            if dv:
                dec = dv.lower() in _TRUE
            else:
                dec = n_tags > 0 or bool(str(row.get("tag_id", "")).strip())
            u = _pf(row.get("u_px"))
            v = _pf(row.get("v_px"))
            if u is None:
                u = _pf(row.get("center_u"))
            if v is None:
                v = _pf(row.get("center_v"))
            # incidence_deg is pi_capture's capture-time pose record (2026-07-25);
            # absent in a legacy tags.csv, which stays None (unknown), never 0.
            results[base] = TagObs(dec, _tag_row_range(row), u, v, n_tags,
                                   _pf(row.get("incidence_deg")))
        if not quiet:
            n_dec = sum(1 for r in results.values() if r[0])
            # Disambiguate: "row absent" must never read as "tag not decoded".
            print(f"[tripod] curve(a): reused provided tags.csv "
                  f"({n_dec} decoded / {len(missing)} no-row / {len(frames)} frames)")
        return results

    import cv2
    undistort = bool(np.asarray(dist).any()) and w and h
    map1 = map2 = None
    if undistort:
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)
        map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, K, (w, h), cv2.CV_16SC2)
    # quad_decimate is EXPLICIT (ADR-0082): it used to be the silent library
    # default 2.0, which is the configuration placard_sizing predicts FAILS the
    # money gate at 20/14 Hz. It is a range multiplier, so it belongs on the
    # record next to the curve it produced -- see check_decimate_consistency.
    det = _make_detector(quad_decimate)
    for p, base in frames:
        img = cv2.imread(p)
        if img is None:
            results[base] = TagObs(False, None, None, None, 0, None)
            continue
        if undistort:
            img = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        dets = det.detect(gray, estimate_tag_pose=True,
                          camera_params=(fx, fy, cx, cy), tag_size=tag_size_m)
        if not dets:
            results[base] = TagObs(False, None, None, None, 0, None)
            continue
        d = min(dets, key=lambda t: float(t.pose_t.reshape(3)[2]))  # nearest
        rng = float(np.linalg.norm(d.pose_t.reshape(3)))
        u, v = float(d.center[0]), float(d.center[1])
        results[base] = TagObs(True, rng, u, v, len(dets),
                               tag_incidence_deg(d.pose_R, d.pose_t))
    if not quiet:
        n_dec = sum(1 for r in results.values() if r[0])
        print(f"[tripod] curve(a): decoded {n_dec}/{len(frames)} frames "
              f"(pupil-apriltags, tag_size={tag_size_m:.3f} m, "
              f"quad_decimate={float(quad_decimate)})")
    return results


# --------------------------------------------------------------------------
# Curve (a): decode envelope
# --------------------------------------------------------------------------
def r90_walk(bins, bin_m, min_bin_n=DEFAULT_MIN_BIN_N):
    """STEPPED >=90% walk outward from the nearest bin. Returns
    (r90, stop_reason, stop_lo, n_at_r90_bin, coverage_gaps).

    DEFECT THIS REPLACES (found 2026-07-25, review 2 HIGH; it flips the ~$740
    gate FAIL->PASS on real data). The old walk was `for lo in sorted(bins)`,
    which iterates only the bins that EXIST. A range bin with zero binnable
    frames is simply ABSENT from the dict -- a coverage hole (a GPS/telemetry
    dropout whose frames are all range_quality=gap, an occluded mid-band, or two
    passes shot at different stations) -- so the walk JUMPED the hole and kept
    extending R_decode90 into ranges that were never sampled. There was also no
    per-bin sample floor, so ONE lucky far decode read rate 1.00 and extended the
    envelope by a whole bin on n=1.

    The walk now STEPS `lo += bin_m` and stops on:
      rate_below_90        a MEASURED cutoff (the only reason that may PASS/FAIL)
      bin_absent           coverage hole -- the next bin has no binnable frames
      bin_underpopulated   the next bin has < min_bin_n binnable frames
      end_of_data          walked past the farthest sampled bin
    The last three mean the envelope is bounded by MISSING DATA, not by measured
    decode failure -- gate_verdict returns UNCERTAIN for them (never PASS).

    `coverage_gaps` lists every absent bin `lo` between the nearest and farthest
    PRESENT bin, so verdict.txt can name the stations that need re-shooting.
    """
    if not bins or bin_m <= 0:
        return 0.0, R90_STOP_END, None, 0, []
    los = sorted(bins)
    lo_min, lo_max = los[0], los[-1]
    # Step index k = (lo - lo_min)/bin_m, so the walk is integer-stepped and
    # float error in `lo_min + k*bin_m` can never fake a hole.
    by_step = {int(round((lo - lo_min) / bin_m)): lo for lo in los}
    k_max = int(round((lo_max - lo_min) / bin_m))
    coverage_gaps = [round(lo_min + k * bin_m, 6)
                     for k in range(k_max + 1) if k not in by_step]

    r90 = 0.0
    n_at_r90 = 0
    stop_reason = R90_STOP_END
    stop_lo = None
    for k in range(k_max + 2):
        lo = lo_min + k * bin_m
        key = by_step.get(k)
        if key is None:
            stop_reason = R90_STOP_END if k > k_max else R90_STOP_ABSENT
            stop_lo = round(lo, 6)
            break
        n_dec, n_tot, _ = bins[key]
        if n_tot < min_bin_n:
            # Underpopulated BEFORE rate: a rate measured on n<min_bin_n is not
            # a measurement, so it may neither extend the envelope (the old n=1
            # inflation) nor stand as the measured cutoff that permits PASS/FAIL.
            stop_reason, stop_lo = R90_STOP_UNDERPOPULATED, round(lo, 6)
            break
        if not n_tot or (n_dec / n_tot) < 0.9:
            stop_reason, stop_lo = R90_STOP_RATE, round(lo, 6)
            break
        r90 = lo + bin_m
        n_at_r90 = n_tot
    return r90, stop_reason, stop_lo, n_at_r90, coverage_gaps


def curve_a(frames, decode, index_map, bin_m, max_m, accept_quality=None,
            min_bin_n=DEFAULT_MIN_BIN_N):
    """Bin decode success by TRUE range. Returns (bins, R_decode_any,
    R_decode90, accuracy_rows, n_binnable, n_unbinnable).

      bins        {range_lo: [n_decoded, n_total, [decoded_ranges]]}
      R_decode_any  farthest true-range of ANY successful decode (m)
      R_decode90    farthest range where the decode rate SUSTAINS >=90% inward
                    (protocol §7.1) -- the outer edge of the contiguous
                    >=90% region starting from the nearest bin (0.0 if even the
                    nearest bin is <90%). See r90_walk() for the stop rules and
                    for the coverage-hole defect they close; call r90_walk()
                    directly when the stop REASON is needed (run() does).
    """
    bins = {}
    acc = []
    n_binnable = n_unbinnable = 0
    r_any = 0.0
    for _, base in frames:
        gr = truth_range_for(base, index_map, accept_quality=accept_quality)
        if gr is None or gr <= 0 or gr > max_m:
            n_unbinnable += 1
            continue
        n_binnable += 1
        dec, drange, _u, _v, _n, _inc = as_obs(decode[base])
        b = int(gr // bin_m) * bin_m
        cell = bins.setdefault(b, [0, 0, []])
        cell[1] += 1
        if dec:
            cell[0] += 1
            r_any = max(r_any, gr)
            if drange is not None:
                cell[2].append(drange)
                acc.append((gr, drange))

    # R_decode90: STEPPED walk outward -- stops on a coverage hole or an
    # underpopulated bin instead of jumping them (r90_walk docstring).
    r90 = r90_walk(bins, bin_m, min_bin_n=min_bin_n)[0]
    return bins, r_any, r90, acc, n_binnable, n_unbinnable


def decode_rate_near(bins, r90, bin_m):
    """Sustained per-frame decode rate in the bin AT R_decode90 -- the rate used
    to size the streak burn (protocol §8.1: 'compute it from this session's own
    rate curve'). By construction this is >=0.9 when r90>0.

    BUG FIXED 2026-07-24 (latent; only bites on REAL data). curve_a returns
    `r90 = lo + bin_m` of the LAST bin that sustained >=90%, i.e. the band's OUTER
    EDGE -- so the bin keyed exactly `r90` is the FIRST FAILING bin. The old
    `for b in sorted(bins): if b <= r90: best = b` selected that failing bin
    whenever the bins were CONTIGUOUS, handing the gate the sub-90% rate (0.0 in
    a clean cutoff), which inflates R_streak_burn to infinity and forces FAIL.
    It went unnoticed because the only session ever run through it (the synthetic
    self-test) has a GAP between its near and far ranges, so `b <= r90` happened
    to land on the last sustaining bin. Now the bin CONTAINING (r90 - bin_m/2) is
    selected explicitly. Regression: tests/test_tripod_pipeline_join.py."""
    if r90 <= 0 or not bins:
        return 0.0
    target_lo = math.floor((r90 - bin_m / 2.0) / bin_m) * bin_m
    cands = [b for b in bins if b <= target_lo + 1e-9]
    best = max(cands) if cands else min(bins, key=lambda b: abs(b - target_lo))
    n_dec, n_tot, _ = bins.get(best, [0, 0, []])
    return (n_dec / n_tot) if n_tot else 0.0


# --------------------------------------------------------------------------
# Curve (a) x INCIDENCE (protocol §4.2b; placard_mount §11 item 9)
# --------------------------------------------------------------------------
def frame_incidence_deg(basename, index_map, obs):
    """(incidence_deg, source) for ONE frame. Source order matters:

      1. `index.csv:incidence_deg` -- computable for EVERY frame (decoded or not)
         from the target log's attitude + the surveyed tripod position + the
         mount index angle (protocol §4.2b: "compute per-frame incidence
         offline... this costs ZERO field time"). This is the ONLY source that
         can support a decode-RATE-vs-incidence curve, because a rate needs a
         DENOMINATOR that includes the frames where the tag did NOT decode.
      2. the tag's own recovered pose -- available on DECODED FRAMES ONLY, so it
         can annotate the decodes but can never furnish that denominator.

    Returns (None, "none") when neither exists -- never a substituted 0.0."""
    if index_map:
        row = index_map.get(basename)
        if row is not None:
            v = _pf(row.get("incidence_deg"))
            if v is not None:
                return v, "index"
    inc = as_obs(obs).incidence_deg if obs is not None else None
    if inc is not None:
        return inc, "tag_pose"
    return None, "none"


def curve_a_incidence(frames, decode, index_map, bin_m, max_m,
                      inc_bin_deg=DEFAULT_INCIDENCE_BIN_DEG,
                      accept_quality=None):
    """Score curve (a) against (range x INCIDENCE) instead of range alone.

    THE HONESTY PROBLEM THIS ENCODES (and does not paper over). The tag's
    incidence is recovered FROM THE TAG POSE, which exists only on frames where
    the tag DECODED. So with tag-pose incidence alone, every cell's denominator
    is "frames that decoded", the rate is identically 1.0, and a decode-rate-vs-
    incidence curve is NOT COMPUTABLE -- printing one would be exactly the
    confident-but-vacuous verdict class this project keeps retracting. Only a
    per-frame incidence written into index.csv (from the target log's attitude +
    the surveyed tripod position + the mount index, protocol §4.2b) covers the
    misses too.

    So: `rate_computable` is True ONLY when every binnable frame carries an
    index-supplied incidence. Otherwise the return value is an ANNOTATION -- the
    incidence distribution of the DECODES, which still answers "what aspects did
    this session actually sample?" and is what tells you whether a GO was measured
    at 0 deg (§8.1) -- and it says so in `note`.

    Returns a dict (cells keyed (range_lo, incidence_lo))."""
    cells = {}
    n_binnable = 0
    n_with_axis = 0
    sources = {"index": 0, "tag_pose": 0, "none": 0}
    decoded_inc = []
    for _p, base in frames:
        gr = truth_range_for(base, index_map, accept_quality=accept_quality)
        if gr is None or gr <= 0 or gr > max_m:
            continue
        n_binnable += 1
        obs = as_obs(decode[base])
        th, src = frame_incidence_deg(base, index_map, obs)
        sources[src] += 1
        if th is None:
            continue
        n_with_axis += 1
        if obs.decoded:
            decoded_inc.append(th)
        if src != "index":
            continue          # cannot form an honest denominator -- see docstring
        key = (int(gr // bin_m) * bin_m,
               int(abs(th) // inc_bin_deg) * inc_bin_deg)
        cell = cells.setdefault(key, [0, 0])
        cell[1] += 1
        cell[0] += int(bool(obs.decoded))

    rate_computable = bool(n_binnable > 0 and sources["index"] == n_binnable)
    # PARTIAL index coverage: SOME frames carry an index incidence and some do
    # not. The cells above are then built from that SUBSET, so their denominator
    # is not the session's -- a SHRUNKEN denominator, which this project's error
    # policy says must be COUNTED and reported, never absorbed. Two things used
    # to go wrong here (fixed 2026-07-25, alongside the incidence PRODUCER
    # `range_truth_join --incidence`, which refuses to emit a partial column in
    # the first place): (1) with tag_pose == 0 the note fell through to "index.csv
    # carries no incidence_deg column", which is a CONFIDENT WRONG DIAGNOSIS when
    # some frames plainly do carry one; (2) nothing said that non-empty cells were
    # computed on a subset. `cells_are_partial` now says so in gate.json, and
    # write_curve_a_incidence_csv already stamps rate_is_measured=0 per row.
    cells_are_partial = bool(cells and not rate_computable)
    axis_source = ("index.csv incidence_deg (ALL frames -- rate is a measurement)"
                   if rate_computable else
                   (f"index.csv incidence_deg on only {sources['index']} of "
                    f"{n_binnable} binnable frames -- PARTIAL, cells are a subset, "
                    f"NOT a rate" if sources["index"] else
                    ("tag pose (DECODED frames only -- annotation, NOT a rate)"
                     if sources["tag_pose"] else
                     ("none -- NO BINNABLE FRAMES (run range_truth_join.py first)"
                      if n_binnable == 0 else "none (no incidence on any frame)"))))
    if rate_computable:
        note = (f"decode rate binned by range x incidence over {n_binnable} "
                f"frames ({inc_bin_deg:.0f} deg bins = the mount's index step).")
    elif sources["tag_pose"] or sources["index"]:
        note = (
            f"NOT A RATE CURVE: only {sources['index']} of {n_binnable} binnable "
            f"frames carry a per-frame incidence from index.csv, so the misses "
            f"have no incidence and no cell has an honest DENOMINATOR. What is "
            f"reported below is the incidence DISTRIBUTION OF THE DECODES "
            f"(n={len(decoded_inc)}), which answers 'what aspects did this session "
            f"sample' but NOT 'what is the decode rate at 30 deg'. To get the rate "
            f"curve protocol §4.2b asks for, write a per-frame `incidence_deg` "
            f"column into index.csv from the target log attitude + the surveyed "
            f"tripod position + the mount index angle "
            f"(scripts/seeker/range_truth_join.py --incidence --mount-index-deg)."
            + (f" WARNING: {len(cells)} cell(s) ARE shown, but they are built from "
               f"the {sources['index']} frames that DO carry an index incidence -- "
               f"a SHRUNKEN denominator missing "
               f"{n_binnable - sources['index']} binnable frame(s). Every row is "
               f"stamped rate_is_measured=0; do not read those rates as the "
               f"session's decode rate." if cells_are_partial else ""))
    elif n_binnable == 0:
        # DISTINGUISH THE TWO ZEROS. "No incidence" because nothing was BINNABLE
        # (range_truth_join has not run) is a missing PIPELINE STEP, not an
        # optical result -- saying "the tag never decoded" there would be a
        # confident wrong diagnosis from the instrument itself.
        note = ("not evaluated: ZERO frames carry a TRUE range, so nothing is "
                "binnable. This is the missing-step case, NOT an aspect result -- "
                "run scripts/seeker/range_truth_join.py first (protocol §7.0 "
                "step (b)), then re-score.")
    else:
        note = (f"no incidence available on any of the {n_binnable} binnable "
                f"frames: no frame decoded a tag pose AND index.csv carries no "
                f"incidence_deg column.")
    stats = None
    if decoded_inc:
        arr = np.asarray(decoded_inc, dtype=float)
        stats = {
            "n": int(arr.size),
            "median_deg": round(float(np.median(arr)), 2),
            "p90_deg": round(float(np.percentile(arr, 90)), 2),
            "max_deg": round(float(arr.max()), 2),
            "n_beyond_validated_cos_law": int((arr > COS_LAW_VALIDATED_TO_DEG).sum()),
        }
    return {
        "cells": cells,
        "incidence_bin_deg": inc_bin_deg,
        "range_bin_m": bin_m,
        "n_binnable": n_binnable,
        "n_with_incidence": n_with_axis,
        "incidence_source_counts": sources,
        "axis_source": axis_source,
        "rate_computable": rate_computable,
        "cells_are_partial": cells_are_partial,
        "decoded_incidence": stats,
        "cos_law_validated_to_deg": COS_LAW_VALIDATED_TO_DEG,
        "note": note,
    }


def read_join_mount_index(session_dir):
    """The placard index angle the per-frame `incidence_deg` column was actually
    built with, from `range_truth.json` (written by range_truth_join
    --incidence). None when the session was never joined with an aspect."""
    p = os.path.join(session_dir or "", "range_truth.json")
    if not os.path.exists(p):
        return None
    try:
        prov = json.loads(open(p).read())
    except (ValueError, OSError):
        return None
    return _pf((prov.get("incidence") or {}).get("mount_index_deg"))


def resolve_mount_index(cli_value, meta, session_dir):
    """(mount_index_deg, source, warning) -- with the PRODUCER cross-check.

    `range_truth_join --incidence` stamps the index angle it used into
    `range_truth.json`, and EVERY incidence cell is binned on the aspect that
    angle produced. If this scoring pass names a DIFFERENT index, gate.json would
    record an angle that contradicts its own axis -- a silent producer/consumer
    disagreement, the failure class the contract-test rule exists for. So:
      * no index here + a joined one -> ADOPT the join's (never record None
        beside a populated aspect axis);
      * both present and different -> a LOUD warning, and the mismatch is written
        into the provenance string, not just printed."""
    value, source = cli_value, "--mount-index-deg (protocol §4.2b pass card)"
    if value is None:
        value = _pf((meta or {}).get("mount_index_deg"))
        source = "meta.json mount_index_deg"
    join_index = read_join_mount_index(session_dir)
    warning = None
    if join_index is not None:
        if value is None:
            value, source = join_index, (
                "range_truth.json -- the index the per-frame incidence_deg "
                "column was actually built with")
        elif abs(value - join_index) > 1e-6:
            warning = (
                f"MOUNT INDEX MISMATCH: this score says {value:.1f} deg but the "
                f"per-frame incidence_deg column was built at {join_index:.1f} "
                f"deg (range_truth.json). Every incidence cell is binned on the "
                f"JOIN's angle; one of the two is wrong -- check the pass card "
                f"before quoting an aspect.")
            source += (f" -- MISMATCH vs range_truth.json ({join_index:.1f} deg, "
                       f"which built the column)")
    return value, source, warning


def write_curve_a_incidence_csv(path, inc):
    """One row per (range x incidence) cell. `rate_is_measured` is per-row and
    false when the cell came from decoded-only incidence -- a reader must never
    have to infer that from prose."""
    with open(path, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["range_lo_m", "range_hi_m", "incidence_lo_deg",
                     "incidence_hi_deg", "n_decoded", "n_total", "decode_rate",
                     "rate_is_measured", "beyond_validated_cos_law"])
        bm, ib = inc["range_bin_m"], inc["incidence_bin_deg"]
        for (rlo, ilo) in sorted(inc["cells"]):
            n_dec, n_tot = inc["cells"][(rlo, ilo)]
            wr.writerow([rlo, rlo + bm, ilo, ilo + ib, n_dec, n_tot,
                         round(n_dec / n_tot, 4) if n_tot else "",
                         int(bool(inc["rate_computable"])),
                         int(ilo + ib > COS_LAW_VALIDATED_TO_DEG)])


# --------------------------------------------------------------------------
# The money gate (pure arithmetic -- unit-testable independent of the pipeline)
# --------------------------------------------------------------------------
def _streak_burn_frames(p, k):
    """Expected #frames to the FIRST RUN of `k` consecutive successes, each with
    probability `p` (independent-Bernoulli approximation):

        E[T] = (1 - p^k) / (p^k * (1 - p))        [classic run-length result]

    Guards: p>=1 -> exactly k (a certain run of k needs k trials); p<=0 -> inf
    (the run never forms). This is the GATING model (ADR-0079) -- see gate_verdict.
    """
    if p >= 1.0:
        return float(k)
    if p <= 0.0:
        return float("inf")
    pk = p ** k
    return (1.0 - pk) / (pk * (1.0 - p))


def gate_verdict(r90, r_any, decode_rate, stream_fps, streak_n, v_closing,
                 tgo_min, n_decoded=None, min_decoded=DEFAULT_MIN_DECODED,
                 have_truth=True, r90_stop_reason=R90_STOP_RATE,
                 r90_stop_lo=None, n_at_r90_bin=None,
                 engagement_incidence_deg=0.0):
    """Apply the curve-(a) money gate (protocol §8.1 / NEXT.md R5):

        R_eff         = R_decode90 · cos(theta_eng)       [incidence-aware form]
        R_streak_burn = (E[T] / stream_fps) × V_closing   [run-length, ADR-0079]
        E[T]          = (1 - p^k) / (p^k * (1 - p)),  p=decode_rate, k=streak_n
        t_go          = (R_eff − R_streak_burn) / V_closing

    INCIDENCE-AWARE (2026-07-25, docs/placard_mount.md §11 item 10). The placard
    is a flat fiducial on a BEAM-FACING mount, and decode range falls as
    `R(theta) = R(0)·cos theta` (placard_mount §3.2). `engagement_incidence_deg`
    is the worst-case incidence the REAL engagement will present; it DEFAULTS TO
    0.0, which reproduces the pre-2026-07-25 arithmetic exactly, and the caller is
    responsible for saying so out loud — a GO measured at 0 deg and applied to a
    30 deg engagement is not a GO. NOTE the measured law is only validated to
    32.6 deg in sim (placard_mount §13, HYPOTHESIS beyond).

    GO (PASS) iff t_go >= tgo_min at the conservative closing speed. Returns a
    dict with the verdict, every intermediate, and a human-readable arithmetic
    string. UNCERTAIN when the data can't decide (no truth ranges, no sustained
    handoff-quality band, or below the data-presence floor).

    STREAM FPS IS REQUIRED, NEVER INVENTED (2026-07-24). `stream_fps=None` (no
    `meta.json:stream_fps`, no `--stream-fps`) returns UNCERTAIN naming the
    missing measurement instead of falling back to a constant: the burn scales
    as 1/fps, so at the predicted R_decode90 (7.10 m, docs/placard_sizing.md §4)
    with p=0.9/k=5/9 m/s the SAME optical data gives t_go 0.558 s PASS at 30 fps
    and 0.442 s FAIL at the 20 Hz the deployed flight loop actually runs
    (break-even 24.0 fps). A verdict that does NOT depend on fps is still
    returned -- `R_decode90 == 0` (no bin sustains >=90%) is FAIL on decode
    evidence alone -- so the missing measurement only blocks the calls it
    actually decides.

    GATING MODEL (ADR-0079, adopted 2026-07-24 -- docs/decisions.md): the handoff
    needs `streak_n` CONSECUTIVE fresh detections, so the burn is the run-length
    expectation E[T], NOT the earlier MEAN-RATE burn (streak_n / decode_Hz). The
    mean-rate model is OPTIMISTIC -- it understates the burn, always in the
    dangerous direction for a ~$740 purchase gate -- and the gap grows fast as the
    decode rate falls (k=5: p=0.9 ~1.2×, p=0.7 ~2.3×, p=0.5 ~6.2×). In this gate's
    intended regime (p at R_decode90 is >=0.9 by construction) the two nearly
    agree, so adopting run-length costs ~1.2× in-regime while protecting the
    marginal/low-p case. The mean-rate burn is still REPORTED (R_streak_burn_meanrate_m)
    for transparency. CAVEAT: real decodes are temporally CORRELATED, not
    independent Bernoulli, so BOTH analytic models are surrogates -- the EMPIRICAL
    per-session streak-formation range (protocol §8.1) is the preferred input when
    a real session provides it; this analytic gate is the fallback.
    Reproduce the comparison: scripts/seeker/streak_burn_derivation.py

    R_decode90 MUST BE MEASURED, NOT MISSING (2026-07-25, review 2 HIGH).
    `r90_stop_reason` says WHY the >=90% walk stopped (see r90_walk). Only
    `rate_below_90` is a measured cutoff -- the tag was tried at that range and
    failed. `bin_absent` / `bin_underpopulated` / `end_of_data` mean the band
    ends where the SESSION ends, so R_decode90 is a lower bound set by what was
    never shot, and the honest verdict is UNCERTAIN ("re-shoot that station"),
    never PASS. Default is `rate_below_90` so a programmatic caller passing a
    hand-built r90 (the self-tests, the pure-arithmetic unit tests) keeps the
    pre-2026-07-25 behaviour.
    """
    th_eng = float(engagement_incidence_deg or 0.0)
    cos_eng = math.cos(math.radians(th_eng))
    r90_eff = r90 * cos_eng
    # THE DERATE LAW IS ONLY VALIDATED TO 32.6 deg (2026-07-26). verdict.txt
    # flagged the hypothesis for incidence CELLS but never for theta_eng, so a
    # gate quoted at --engagement-incidence-deg 45 printed a clean PASS/FAIL
    # resting on an extrapolation. It travels with the gate dict (both speeds,
    # both early returns) so gate.json is machine-checkable, and the caveat is
    # OPTIMISTIC in direction: placard_mount §3.2's literature note says decode
    # can fall FASTER than cos near P_min.
    cos_extrapolated = th_eng > COS_LAW_VALIDATED_TO_DEG
    _cos_caveat = (
        f"  [HYPOTHESIS] theta_eng {th_eng:.1f} deg EXCEEDS the "
        f"{COS_LAW_VALIDATED_TO_DEG} deg the cos-law is validated to "
        f"(placard_mount §3.2/§13) -- R_eff is an EXTRAPOLATION, not a measured "
        f"derate, and decode may fall FASTER than cos near P_min, so this is the "
        f"OPTIMISTIC direction\n" if cos_extrapolated else "")
    fps_ok = stream_fps is not None and stream_fps > 0
    decode_hz = decode_rate * stream_fps if fps_ok else None
    # Mean-rate burn: optimistic, reported for transparency only (not gating).
    r_burn_meanrate = ((streak_n / decode_hz) * v_closing
                       if (fps_ok and decode_hz > 0) else float("inf"))
    # Run-length burn: the GATING model (ADR-0079), conservative for the purchase gate.
    e_frames = _streak_burn_frames(decode_rate, streak_n)
    if fps_ok and math.isfinite(e_frames):
        r_streak_burn = (e_frames / stream_fps) * v_closing
    else:
        r_streak_burn = float("inf")
    t_go = ((r90_eff - r_streak_burn) / v_closing
            if (fps_ok and v_closing > 0) else float("-inf"))

    data_bounded = r90_stop_reason in R90_DATA_BOUNDED
    _where = f"{r90_stop_lo:.2f} m" if r90_stop_lo is not None else f"{r90:.2f} m"
    _bounded_reason = {
        R90_STOP_ABSENT: (
            f"the >=90% band ends at a COVERAGE HOLE: the bin starting {_where} "
            f"holds ZERO binnable frames, so R_decode90={r90:.2f} m is bounded by "
            f"what was never shot, not by a measured decode failure. RE-SHOOT that "
            f"station (or re-run with --max-range at the last sampled bin)."),
        R90_STOP_UNDERPOPULATED: (
            f"the >=90% band ends at an UNDERPOPULATED bin starting {_where} "
            f"(< the --min-bin-n floor), so its rate is not a measurement and "
            f"R_decode90={r90:.2f} m is a lower bound. RE-SHOOT that station."),
        R90_STOP_END: (
            f"the >=90% band ran off the END OF THE DATA at {_where} -- every "
            f"sampled bin sustained >=90%, so R_decode90={r90:.2f} m is a lower "
            f"bound set by the farthest station shot, not by the tag's ceiling. "
            f"Shoot a FARTHER station to find the real cutoff."),
    }.get(r90_stop_reason, "")

    if not have_truth:
        verdict, reason = "UNCERTAIN", "no TRUE ranges in the session (index.csv true_range_m / filename _r_) -- cannot bin curve (a). Run scripts/seeker/range_truth_join.py first (target log + surveyed tripod position -> true_range_m)."
    elif n_decoded is not None and n_decoded < min_decoded:
        verdict, reason = "UNCERTAIN", (f"only {n_decoded} decoded frame(s) < data floor {min_decoded} -- too thin to decide")
    elif r90 <= 0 and not data_bounded:
        # No bin sustained >=90%: the tag never forms a handoff-quality stream.
        # This leg is fps-INDEPENDENT, so it is decided before the fps check.
        verdict = "FAIL" if r_any > 0 else "UNCERTAIN"
        reason = ("no range bin sustains >=90% decode (R_decode90=0) -- "
                  + ("sparse decodes exist but no clean handoff band"
                     if r_any > 0 else "the tag never decoded"))
    elif data_bounded:
        # MISSING DATA, not a measured cutoff -> never PASS (review 2 HIGH).
        # This leg is fps-INDEPENDENT and is decided before the fps check: no
        # frame rate can rescue an envelope whose outer edge was never sampled.
        verdict, reason = "UNCERTAIN", _bounded_reason
        if n_at_r90_bin is not None:
            reason += f" [n at the last sustaining bin = {n_at_r90_bin}]"
    elif not fps_ok:
        verdict, reason = "UNCERTAIN", (
            "MISSING MEASUREMENT: capture/decode stream_fps. The streak burn is "
            "(E[T]/stream_fps) x V_closing, so the verdict turns on it -- at this "
            f"R_decode90={r90:.2f} m the gate flips between ~24 fps and below. "
            "Supply the BENCH-MEASURED rate (protocol §7.3 Pi 5 compute bench) via "
            "--stream-fps, or capture with pi_capture so meta.json records "
            "stream_fps from real capture timestamps. NO default is assumed.")
    elif t_go >= tgo_min:
        verdict, reason = "PASS", "t_go clears the pre-registered floor"
    else:
        verdict, reason = "FAIL", "t_go below the pre-registered floor"

    if not fps_ok:
        arithmetic = (
            f"decode_rate p = {decode_rate:.3f}  (streak k = {streak_n})\n"
            f"E[T] run-length = (1 - p^k)/(p^k*(1-p)) = "
            f"{e_frames:.2f} frames\n"
            f"R_streak_burn = (E[T] / stream_fps) x {v_closing:.1f} m/s = "
            f"UNKNOWN -- stream_fps NOT SUPPLIED\n"
            f"t_go = (R_decode90 {r90:.2f} m - R_streak_burn ?) / {v_closing:.1f} m/s "
            f"= UNCOMPUTABLE\n"
            f"gate: -> {verdict} "
            f"({'fps-independent' if (r90 <= 0 or data_bounded) else 'needs the fps measurement'})"
        ) if math.isfinite(e_frames) else (
            f"decode_rate p = {decode_rate:.3f} -> streak never forms -> {verdict}")
        return {
            "verdict": verdict, "reason": reason,
            "R_decode90_m": round(r90, 3), "R_decode_any_m": round(r_any, 3),
            "engagement_incidence_deg": round(th_eng, 3),
            "R_decode90_eff_m": round(r90_eff, 3),
            "cos_law_validated_to_deg": COS_LAW_VALIDATED_TO_DEG,
            "theta_eng_beyond_validated_cos_law": bool(cos_extrapolated),
            "r90_stop_reason": r90_stop_reason,
            "r90_stop_lo_m": r90_stop_lo,
            "r90_data_bounded": bool(data_bounded),
            "n_at_r90_bin": n_at_r90_bin,
            "decode_rate": round(decode_rate, 4), "decode_Hz": None,
            "streak_burn_model": "run-length (ADR-0079)",
            "E_streak_frames": round(e_frames, 3) if math.isfinite(e_frames) else None,
            "R_streak_burn_m": None, "R_streak_burn_meanrate_m": None,
            "t_go_s": None,
            "V_closing_mps": v_closing, "tgo_min_s": tgo_min,
            "handoff_streak": streak_n, "stream_fps": None,
            "arithmetic": arithmetic,
        }

    _inc_line = (
        f"R_eff = R_decode90 {r90:.2f} m x cos(theta_eng {th_eng:.1f} deg) = "
        f"{r90_eff:.2f} m   [incidence-aware, placard_mount §11 item 10]\n"
        + _cos_caveat
        if th_eng else "")
    arithmetic = (
        f"decode_rate p = {decode_rate:.3f}  (streak k = {streak_n})\n"
        + _inc_line +
        f"E[T] run-length = (1 − p^k)/(p^k·(1−p)) = {e_frames:.2f} frames "
        f"(mean-rate {streak_n}/p = {streak_n / decode_rate:.2f} frames)\n"
        f"R_streak_burn = ({e_frames:.2f} / {stream_fps:.1f} fps) × {v_closing:.1f} m/s = "
        f"{r_streak_burn:.2f} m   (mean-rate burn {r_burn_meanrate:.2f} m)\n"
        f"t_go = ({'R_eff' if th_eng else 'R_decode90'} {r90_eff:.2f} m − "
        f"R_streak_burn {r_streak_burn:.2f} m) / "
        f"{v_closing:.1f} m/s = {t_go:.3f} s\n"
        f"gate: t_go {t_go:.3f} s {'>=' if t_go >= tgo_min else '<'} {tgo_min:.2f} s -> {verdict}"
    ) if decode_rate > 0 else (
        f"decode_rate p = 0 -> streak never forms (R_streak_burn = inf) -> {verdict}"
    )
    return {
        "verdict": verdict, "reason": reason,
        "R_decode90_m": round(r90, 3), "R_decode_any_m": round(r_any, 3),
        "engagement_incidence_deg": round(th_eng, 3),
        "R_decode90_eff_m": round(r90_eff, 3),
        "cos_law_validated_to_deg": COS_LAW_VALIDATED_TO_DEG,
        "theta_eng_beyond_validated_cos_law": bool(cos_extrapolated),
        "r90_stop_reason": r90_stop_reason,
        "r90_stop_lo_m": r90_stop_lo,
        "r90_data_bounded": bool(data_bounded),
        "n_at_r90_bin": n_at_r90_bin,
        "decode_rate": round(decode_rate, 4), "decode_Hz": round(decode_hz, 3),
        "streak_burn_model": "run-length (ADR-0079)",
        "E_streak_frames": round(e_frames, 3) if math.isfinite(e_frames) else None,
        "R_streak_burn_m": round(r_streak_burn, 3) if math.isfinite(r_streak_burn) else None,
        "R_streak_burn_meanrate_m": round(r_burn_meanrate, 3) if math.isfinite(r_burn_meanrate) else None,
        "t_go_s": round(t_go, 4) if math.isfinite(t_go) else None,
        "V_closing_mps": v_closing, "tgo_min_s": tgo_min,
        "handoff_streak": streak_n, "stream_fps": stream_fps,
        "arithmetic": arithmetic,
    }


# --------------------------------------------------------------------------
# Curve (b): NN approach recall vs range × position-in-frame (no-ops cleanly)
# --------------------------------------------------------------------------
def _pos_band(v_px, h_px, n_bands):
    """top / middle / bottom third of the frame from the truth box centre v."""
    if h_px <= 0:
        return "?"
    frac = min(max(v_px / h_px, 0.0), 0.999)
    idx = int(frac * n_bands)
    if n_bands == 3:
        return ("top", "middle", "bottom")[idx]
    return f"band{idx}"


def curve_b(frames, decode, calib, weights, conf, drone_size_m, bin_m, max_m,
            n_bands=3):
    """Recall of the markerless NN on tag-truthed frames, binned by range ×
    position-in-frame. Returns (bins, note) or (None, note) if the NN half
    cannot run on this box. bins: {(range_lo, band): [hits, total]}.

    Scope note: truthed ONLY on frames where the tag decoded (the autolabel
    approach the task names). Far-band frames beyond the tag's decode ceiling
    have no tag truth box here -- scoring them needs ULog-derived truth boxes
    (protocol §7.2), a future enhancement, so curve (b)'s far edge is bounded by
    curve (a)'s decode ceiling. Flagged, not silently dropped."""
    if not weights or not os.path.exists(weights):
        return None, f"NN half SKIPPED: weights not found ({weights}) -- gates only the Hailo/markerless phase, not the interceptor"
    try:
        sys.path.insert(0, HERE)
        import cv2  # noqa: F401
        from finetuned_seeker import FinetunedNNSeeker
        from gen_sim_dataset import project_to_bbox
    except ImportError as e:
        return None, f"NN half SKIPPED: {e} (onnxruntime is not in the main .venv -- run curve (b) under .venv-seeker)"
    try:
        fx, fy, cx, cy, w, h, _dist = calib
        seeker = FinetunedNNSeeker(fx, fy, cx, cy, weights, conf_thres=conf)
    except Exception as e:  # onnxruntime import / session load failure
        return None, f"NN half SKIPPED: could not load the ONNX runtime/weights ({e})"

    # THE one box-hit gate (box_scoring is PURE -- no cv2, no onnxruntime -- so
    # this import cannot fail for env reasons; let an ImportError be fatal).
    # 2026-07-25: this used to import from resolution_probe (which needlessly
    # drags in cv2 + finetuned_seeker) behind a LOCAL FALLBACK that was the
    # pre-2026-07-21 gate verbatim -- no sec^2 widening, no gt_scale, tol=15.
    # A silent fallback to a known-wrong scorer is exactly this file's failure
    # class, so the fallback is deleted.
    from box_scoring import box_hits_gt

    import cv2
    bins = {}
    n_scored = 0
    n_offframe = 0        # truth box projected outside the image
    n_unreadable = 0      # frame file unreadable
    for p, base in frames:
        dec, drange, _u, _v, _n, _inc = as_obs(decode[base])
        if not dec or drange is None:
            continue  # no tag truth box on this frame
        gr = drange
        if gr <= 0 or gr > max_m:
            continue
        # Truth box: project the drone extent at the tag's recovered pose.
        # We only have the tag's RANGE + image centre here (u,v), so place the
        # box on the boresight ray through the tag centre at that range.
        u0, v0 = _u, _v
        if u0 is None or v0 is None:
            u0, v0 = cx, cy
        # BUG FIXED 2026-07-24 (review 2, blocker). x and y were scaled by the RANGE
        # gr while z was scaled by gr/s (s = sqrt(1+a^2+b^2)), so the re-projected
        # truth centre landed at u = cx + (u0-cx)*s instead of u0 -- displaced
        # RADIALLY OUTWARD by a factor that GROWS with off-axis angle (measured:
        # 6.9 px at 16 deg, 72.7 px at 34 deg, 234 px at 45 deg = off-frame).
        # That is exactly the position-in-frame axis curve (b) exists to measure, so
        # a PERFECT detector would score MISS toward the edges and the artefact would
        # read as CONFIRMATION of this project's own "position-in-frame explains the
        # recall gap" belief (ADR-0076 add #18k). Correct: the bearing ray (a, b, 1)
        # normalised to length gr, i.e. x = a*z, y = b*z -- which is what the
        # IoU-0.965-validated autolabel_from_apriltag path already does.
        _a = (u0 - cx) / fx
        _b = (v0 - cy) / fy
        z = gr / math.sqrt(1.0 + _a * _a + _b * _b)
        x = _a * z
        y = _b * z
        gt = project_to_bbox(x, y, z, gr, fx, fy, cx, cy, drone_size_m, w or 1280, h or 960)
        if gt is None:
            n_offframe += 1     # SILENT-DROP GUARD: a denominator that shrinks
            continue            # without saying so is the whole failure class.
        gcx, gcy, gw, gh = gt
        frame = cv2.imread(p)
        if frame is None:
            n_unreadable += 1
            continue
        # SESSION intrinsics, not box_scoring's SIM defaults (2026-07-25, review
        # 2 HIGH). box_hits_gt widens the truth box by sec^2(theta) off-axis with
        # tan(theta_x) = (gcx - cx)/fx; defaulted, that ran fx=fy=539.936,
        # cx=640, cy=480 (the gz_x500_mono_cam @1280x960) on a REAL 1280x800
        # 118-deg OV9281 frame (fx~=385, cy=400), under-widening the gt box by up
        # to ~1.46x horizontally / ~1.48x vertically at the edges -- i.e. real,
        # correctly-placed detections scored MISS on exactly the
        # position-in-frame axis curve (b) exists to measure.
        # gt_scale=1.0 is CORRECT here and is passed explicitly: the truth box
        # was just projected at `drone_size_m` (the REAL target's extent, from
        # meta.json/--drone-size), so it is already at the scoring extent --
        # box_scoring.TARGET_EXTENT_M=0.52 is the SIM fpv_quad_enemy silhouette
        # and must NOT be applied to real imagery of the real 5" quad.
        hit = box_hits_gt(seeker._infer_boxes(frame), gcx, gcy, gw, gh,
                          fx=fx, fy=fy, cx=cx, cy=cy, gt_scale=1.0)
        band = _pos_band(gcy, h or frame.shape[0], n_bands)
        key = (int(gr // bin_m) * bin_m, band)
        cell = bins.setdefault(key, [0, 0])
        cell[0] += int(hit)
        cell[1] += 1
        n_scored += 1
    note = (f"NN half: scored {n_scored} tag-truthed frames "
            f"(recall bounded by curve (a)'s decode ceiling -- protocol §7.2)")
    if n_offframe or n_unreadable:
        note += (f" | DROPPED {n_offframe + n_unreadable}: "
                 f"{n_offframe} truth-box off-frame, {n_unreadable} unreadable")
    return bins, note


def bins_overall(bins):
    """(hits, total, recall) rolled up over all range x position cells."""
    if not bins:
        return 0, 0, None
    h = sum(c[0] for c in bins.values())
    t = sum(c[1] for c in bins.values())
    return h, t, (h / t if t else None)


# Protocol §8.2's working threshold: ">=50% recall across the 10-25 m
# operational band". Curve (b) can only truth frames where the TAG decoded, and
# the tag's predicted ceiling is 7.10 m (docs/placard_sizing.md §4) -- so on a
# real session the §8.2 cells come back EMPTY and `overall_recall` is measured
# on a completely different (near) band.
SECTION_8_2_BAND_M = (10.0, 25.0)


def curve_b_band_coverage(bins, bin_m, band=SECTION_8_2_BAND_M):
    """Is protocol §8.2 ANSWERABLE from these curve-(b) cells? (2026-07-25,
    review 2.)

    Returns a dict: n_in_band / n_total / answerable / scored_lo_m / scored_hi_m.
    `answerable` is True only when at least one scored cell OVERLAPS the §8.2
    band. When it is False the caller must NOT hand `overall_recall` over as the
    §8.2 answer -- it is a number for a different range band, which is the
    "confident, plausible, wrong" shape this whole review hunts.
    """
    lo_b, hi_b = band
    n_total = sum(c[1] for c in bins.values()) if bins else 0
    n_in = 0
    los = []
    for (lo, _band), cell in (bins or {}).items():
        los.append(lo)
        if lo < hi_b and (lo + bin_m) > lo_b:      # cell overlaps the band
            n_in += cell[1]
    return {
        "band_lo_m": lo_b, "band_hi_m": hi_b,
        "n_frames_in_band": n_in, "n_frames_scored": n_total,
        "scored_lo_m": (min(los) if los else None),
        "scored_hi_m": (max(los) + bin_m if los else None),
        "answerable": bool(n_in > 0),
    }


def curve_b_multi(frames, decode, calib, models, conf, drone_size_m, bin_m, max_m,
                  n_bands=3):
    """Score curve (b) for SEVERAL models on the SAME frames (2026-07-24).

    `models` = [(label, weights_path), ...]; the FIRST entry is the primary
    (candidate) model and keeps the canonical output filenames. Returns
    [(label, weights, bins_or_None, note), ...] in the given order.

    Rationale (see DEFAULT_WEIGHTS above): the sim-trained quad_v2 is measured
    BLIND on real imagery, so a single-model curve (b) run with it would look like
    a markerless failure rather than the known sim->real NULL. Each model is loaded
    with its OWN input modality (finetuned_seeker.resolve_input_modality: gray for
    the gray-native nn_tier weights, color for the sim weights) -- the frames are
    read once per model, never converted globally."""
    out = []
    for label, w in models:
        bins, note = curve_b(frames, decode, calib, w, conf, drone_size_m,
                             bin_m, max_m, n_bands=n_bands)
        out.append((label, w, bins, note))
    return out


# --------------------------------------------------------------------------
# Outputs: CSVs, plots, plain-text summary
# --------------------------------------------------------------------------
def write_curve_a_csv(path, bins, bin_m):
    with open(path, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["range_lo_m", "range_hi_m", "n_decoded", "n_total",
                     "decode_rate", "median_decoded_range_m", "median_range_err_m"])
        for lo in sorted(bins):
            n_dec, n_tot, dranges = bins[lo]
            med = float(np.median(dranges)) if dranges else ""
            err = (float(np.median([abs(dr - (lo + bin_m / 2)) for dr in dranges]))
                   if dranges else "")
            wr.writerow([lo, lo + bin_m, n_dec, n_tot,
                         round(n_dec / n_tot, 4) if n_tot else "",
                         round(med, 3) if med != "" else "",
                         round(err, 3) if err != "" else ""])


def write_curve_b_csv(path, bins):
    with open(path, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["range_lo_m", "position_band", "hits", "total", "recall"])
        for (lo, band) in sorted(bins, key=lambda k: (k[0], k[1])):
            h, t = bins[(lo, band)]
            wr.writerow([lo, band, h, t, round(h / t, 4) if t else ""])


def plot_curve_a(path, bins, acc, gate, bin_m):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    los = sorted(bins)
    centers = [lo + bin_m / 2 for lo in los]
    rates = [bins[lo][0] / bins[lo][1] if bins[lo][1] else 0 for lo in los]
    ax1.step(centers, [r * 100 for r in rates], where="mid", lw=2, color="tab:blue")
    ax1.axhline(90, color="grey", ls=":", lw=1, label="90% sustain line")
    if gate["R_decode90_m"] > 0:
        ax1.axvline(gate["R_decode90_m"], color="tab:green", ls="--", lw=1.5,
                    label=f"R_decode90 = {gate['R_decode90_m']:.1f} m")
    if gate["R_decode_any_m"] > 0:
        ax1.axvline(gate["R_decode_any_m"], color="crimson", ls=":", lw=1.5,
                    label=f"R_decode_any = {gate['R_decode_any_m']:.1f} m")
    ax1.set_xlabel("true range (m)")
    ax1.set_ylabel("AprilTag decode rate (%)")
    ax1.set_title(f"Curve (a): decode envelope  [GATE {gate['verdict']}]")
    ax1.set_ylim(-3, 103)
    ax1.legend(loc="best", fontsize=8)
    ax1.grid(alpha=0.3)
    if acc:
        tr = [a[0] for a in acc]
        dr = [a[1] for a in acc]
        lim = max(max(tr), max(dr)) * 1.1
        ax2.plot([0, lim], [0, lim], "k--", lw=1, label="y = x")
        ax2.scatter(tr, dr, s=18, alpha=0.6, color="tab:orange")
        ax2.set_xlim(0, lim)
        ax2.set_ylim(0, lim)
        ax2.set_xlabel("true range (m)")
        ax2.set_ylabel("tag-recovered range (m)")
        ax2.set_title("Range accuracy (decoded frames)")
        ax2.legend(loc="best", fontsize=8)
        ax2.grid(alpha=0.3)
    else:
        ax2.text(0.5, 0.5, "no decoded frames", ha="center", va="center")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def plot_curve_b(path, bins):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    if not bins:
        return None
    bands = sorted({k[1] for k in bins})
    los = sorted({k[0] for k in bins})
    fig, ax = plt.subplots(figsize=(8, 5))
    for band in bands:
        xs, ys = [], []
        for lo in los:
            if (lo, band) in bins:
                h, t = bins[(lo, band)]
                xs.append(lo)
                ys.append(100 * h / t if t else 0)
        ax.plot(xs, ys, marker="o", label=f"{band}")
    ax.set_xlabel("range bin lo (m)")
    ax.set_ylabel("NN recall (%)")
    ax.set_title("Curve (b): NN recall vs range × position-in-frame")
    ax.set_ylim(-3, 103)
    ax.legend(title="frame position", loc="best", fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def write_tags_csv(path, frames, decode):
    with open(path, "w", newline="") as f:
        wr = csv.writer(f)
        # incidence_deg (2026-07-25): carried through so a re-decode's own
        # tags.csv is a superset of pi_capture's, not a lossy copy.
        wr.writerow(["frame", "decoded", "tag_range_m", "u_px", "v_px", "n_tags",
                     "incidence_deg"])
        for _, base in frames:
            dec, dr, u, v, n, inc = as_obs(decode[base])
            wr.writerow([base + ".png", int(dec),
                         round(dr, 4) if dr is not None else "",
                         round(u, 2) if u is not None else "",
                         round(v, 2) if v is not None else "", n,
                         round(inc, 2) if inc is not None else ""])


SMALL_N_CAVEAT = (
    "SMALL-N CAVEAT (protocol §4.6): a field afternoon cannot buy CLAUDE.md's "
    "n>=8 paired-seed statistics -- treat curve (a)/(b) as a first honest read, "
    "not a statistically tight one."
)


def build_summary(gate, gate_hi, curve_a_bins, curve_b_note, meta, n_binnable,
                  n_unbinnable, bin_m, fps_source=None, quality=None,
                  range_truth=None, param_provenance=None, coverage_gaps=None,
                  min_bin_n=DEFAULT_MIN_BIN_N, band_coverage=None,
                  incidence=None, decimate_info=None):
    L = []
    L.append("=" * 70)
    L.append("TRIPOD TWO-CURVE SCORE — build_plan P2 / docs/tripod_test_protocol.md")
    L.append("=" * 70)
    if meta:
        keys = ("aspect", "background", "speed", "tilt_deg", "tag_size_m",
                "stream_fps", "closing_speed_mps")
        shown = {k: meta[k] for k in keys if k in meta}
        if shown:
            L.append(f"session meta: {shown}")
    L.append(f"frames binned by true range: {n_binnable}  (unbinnable/skipped: {n_unbinnable})")
    if quality:
        L.append(f"range-truth quality (range_truth_join): {quality}")
    if range_truth:
        L.append(f"range truth: {range_truth.get('n_true_range_written')} frames from "
                 f"{range_truth.get('track_source')} @ clock offset "
                 f"{range_truth.get('clock_offset_s')} s "
                 f"[{range_truth.get('clock_offset_source')}], median sigma "
                 f"{range_truth.get('range_sigma_median_m')} m")
        for w in range_truth.get("warnings", []):
            L.append(f"  [WARN] {w}")
    L.append(f"stream fps source: {fps_source or 'NOT SUPPLIED'}")
    # PARAMETER PROVENANCE: absence of a line must never be the only signal that
    # a load-bearing number came from a DEFAULT rather than from the session.
    if param_provenance:
        L.append("PARAMETER PROVENANCE:")
        for k, (val, src) in sorted(param_provenance.items()):
            L.append(f"  {k:<16} {val}   [{src}]")
    L.append("")
    L.append("--- CURVE (a): AprilTag decode envelope ---")
    L.append(f"  {'range(m)':<10}{'decode%':>9}{'n_dec':>8}{'n_tot':>8}")
    for lo in sorted(curve_a_bins):
        n_dec, n_tot, _ = curve_a_bins[lo]
        rate = 100 * n_dec / n_tot if n_tot else 0
        thin = "  <- n < min-bin-n, cannot extend the band" if n_tot < min_bin_n else ""
        L.append(f"  {f'{lo:.0f}-{lo+bin_m:.0f}':<10}{rate:>8.0f}%{n_dec:>8}{n_tot:>8}{thin}")
    for g in (coverage_gaps or []):
        L.append(f"  [WARN] coverage gap at {g:.0f}-{g + bin_m:.0f} m "
                 f"(no binnable frames) -- the >=90% walk STOPS at a hole, it "
                 f"does not jump it")
    L.append(f"  R_decode_any = {gate['R_decode_any_m']:.2f} m (farthest ANY decode)")
    L.append(f"  R_decode90   = {gate['R_decode90_m']:.2f} m (farthest sustained >=90% inward)")
    L.append(f"  R_decode90 walk stopped: {gate.get('r90_stop_reason')}"
             + (f" at {gate['r90_stop_lo_m']:.0f} m" if gate.get("r90_stop_lo_m") is not None else "")
             + (f"; n at the last sustaining bin = {gate['n_at_r90_bin']}"
                if gate.get("n_at_r90_bin") is not None else ""))
    if gate.get("r90_data_bounded"):
        L.append("  [WARN] R_decode90 is bounded by MISSING DATA, not by a measured "
                 "decode failure -- it is a LOWER BOUND and the gate cannot PASS on it.")
    if gate.get("n_at_r90_bin") is not None and 0 < gate["n_at_r90_bin"] < min_bin_n:
        L.append(f"  [WARN] R_decode90 was set by a bin with n={gate['n_at_r90_bin']} "
                 f"(< --min-bin-n {min_bin_n})")
    if decimate_info is not None and decimate_info.get("mismatch"):
        L.append(f"  >>> quad_decimate MISMATCH ACCEPTED via --allow-decimate-mismatch: "
                 f"{decimate_info['values']}. This curve pools/compares decodes made "
                 f"at DIFFERENT decimations (a range multiplier). Read it as the "
                 f"§4.2b reclaim comparison, and make sure --stream-fps is the Pi 5 "
                 f"bench AT THE SCORED DECIMATE. <<<")
    if decimate_info is not None and decimate_info.get("unknown_sources"):
        L.append(f"  [WARN] quad_decimate UNRECORDED by: "
                 f"{', '.join(decimate_info['unknown_sources'])} -- that decode's "
                 f"decimation is UNKNOWN, not assumed equal. Capture with "
                 f"pi_capture (it stamps meta.json) so the §4.2b reclaim lever "
                 f"survives the session.")
    L.append("")
    # --- CURVE (a) x INCIDENCE (protocol §4.2b) ---------------------------
    if incidence is not None:
        L.append("--- CURVE (a) x INCIDENCE (protocol §4.2b; the beam-facing placard) ---")
        L.append(f"  axis source: {incidence['axis_source']}")
        for line in str(incidence["note"]).split(". "):
            if line.strip():
                L.append(f"  {line.strip().rstrip('.')}.")
        if incidence["rate_computable"] and incidence["cells"]:
            ib, bm = incidence["incidence_bin_deg"], incidence["range_bin_m"]
            L.append(f"  {'range(m)':<10}{'incid(deg)':>12}{'decode%':>9}"
                     f"{'n_dec':>8}{'n_tot':>8}")
            for (rlo, ilo) in sorted(incidence["cells"]):
                n_dec, n_tot = incidence["cells"][(rlo, ilo)]
                rate = 100 * n_dec / n_tot if n_tot else 0
                flag = ("  <- beyond the cos-law's validated "
                        f"{COS_LAW_VALIDATED_TO_DEG} deg (HYPOTHESIS)"
                        if ilo + ib > COS_LAW_VALIDATED_TO_DEG else "")
                L.append(f"  {f'{rlo:.0f}-{rlo+bm:.0f}':<10}"
                         f"{f'{ilo:.0f}-{ilo+ib:.0f}':>12}{rate:>8.0f}%"
                         f"{n_dec:>8}{n_tot:>8}{flag}")
        d = incidence.get("decoded_incidence")
        if d:
            L.append(f"  DECODED-frame incidence: median {d['median_deg']:.1f} deg, "
                     f"p90 {d['p90_deg']:.1f} deg, max {d['max_deg']:.1f} deg "
                     f"(n={d['n']}; {d['n_beyond_validated_cos_law']} beyond the "
                     f"{COS_LAW_VALIDATED_TO_DEG} deg the cos-law is validated to)")
        else:
            L.append("  DECODED-frame incidence: none (no decoded frame carried a pose)")
        th = gate.get("engagement_incidence_deg") or 0.0
        if th:
            L.append(f"  gate applied at theta_eng = {th:.1f} deg -> R_eff = "
                     f"{gate.get('R_decode90_eff_m')} m (§8.1 incidence-aware form)")
            if gate.get("theta_eng_beyond_validated_cos_law"):
                L.append(f"  [HYPOTHESIS] that theta_eng is BEYOND the "
                         f"{COS_LAW_VALIDATED_TO_DEG} deg the cos-law is "
                         f"validated to -- R_eff is an EXTRAPOLATION")
        else:
            L.append("  [WARN] the gate above was computed at theta_eng = 0 deg. A GO "
                     "measured dead-on is NOT a GO at the incidence the real "
                     "engagement presents -- re-run with --engagement-incidence-deg "
                     "(placard_mount §11 item 10; the cos-law is a HYPOTHESIS "
                     f"above {COS_LAW_VALIDATED_TO_DEG} deg).")
        L.append("")
    L.append("--- MONEY GATE (curve a -> the ~$740 Tier-2 interceptor order) ---")
    L.append(f"  conservative closing speed = {gate['V_closing_mps']:.1f} m/s (NEXT.md R5)")
    for line in gate["arithmetic"].splitlines():
        L.append("  " + line)
    L.append(f"  reason: {gate['reason']}")
    if gate_hi is not None:
        L.append(f"  [context] aggressive {gate_hi['V_closing_mps']:.0f} m/s -> "
                 f"t_go {gate_hi['t_go_s']} s ({gate_hi['verdict']}) — informational, "
                 f"the gate decides on the conservative speed (protocol §8.1)")
    L.append("")
    L.append(f"  >>> GATE {gate['verdict']} <<<   "
             f"(PASS -> unlock the interceptor order; FAIL -> bigger placard / "
             f"camera upgrade; UNCERTAIN -> re-shoot / more data)")
    # The caveat must sit NEXT TO THE VERDICT, not only in the incidence section
    # -- that whole section is skipped when no incidence axis exists, and this
    # gate can still have been derated by an unvalidated theta_eng (2026-07-26).
    if gate.get("theta_eng_beyond_validated_cos_law"):
        L.append(f"  [WARN] this gate was derated by cos(theta_eng "
                 f"{gate.get('engagement_incidence_deg')} deg), BEYOND the "
                 f"{COS_LAW_VALIDATED_TO_DEG} deg the cos-law is validated to "
                 f"(placard_mount §3.2/§13) -- it is a HYPOTHESIS-based "
                 f"extrapolation, and the optimistic direction. Do not quote it "
                 f"as a measured incidence-aware verdict.")
    L.append("")
    L.append("--- CURVE (b): NN approach recall (gates ONLY the Hailo/markerless phase) ---")
    for line in str(curve_b_note).splitlines() or [""]:
        L.append(f"  {line}")
    # §8.2 ANSWERABILITY (2026-07-25, review 2): curve (b) truths only frames
    # where the TAG decoded (ceiling ~7.10 m predicted), while §8.2's threshold
    # is ">=50% recall across the 10-25 m band". Say so, loudly, instead of
    # letting a reader quote overall_recall as the §8.2 answer.
    if band_coverage is not None:
        lo_b, hi_b = band_coverage["band_lo_m"], band_coverage["band_hi_m"]
        if band_coverage["answerable"]:
            L.append(f"  §8.2 answerable: {band_coverage['n_frames_in_band']} of "
                     f"{band_coverage['n_frames_scored']} scored frames fall in the "
                     f"{lo_b:.0f}-{hi_b:.0f} m band.")
        else:
            span = ("no cells scored" if band_coverage["scored_lo_m"] is None else
                    f"{band_coverage['scored_lo_m']:.0f}-{band_coverage['scored_hi_m']:.0f} m only")
            L.append(f"  >>> §8.2 UNANSWERABLE: 0 of {band_coverage['n_frames_scored']} "
                     f"scored frames fall in the {lo_b:.0f}-{hi_b:.0f} m threshold band "
                     f"({span}). The recall above is NOT the §8.2 answer -- curve (b) "
                     f"truths only tag-decoded frames, so its far edge is curve (a)'s "
                     f"decode ceiling. Do NOT quote overall_recall against §8.2. <<<")
    L.append("")
    L.append(SMALL_N_CAVEAT)
    return "\n".join(L)


# --------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------
def capture_source_allowed(meta):
    """(ok, detail) -- may THIS session's own recorded rate feed the money gate?

    BUILDER RULING 2026-07-25 (review 2's open question, answered NO): a
    `--local`/v4l2 DESK REHEARSAL -- a laptop webcam on a desk -- must never
    supply the ~$740 gate's 1/fps divisor. Its cadence is a property of a USB UVC
    webcam and a laptop CPU; the modelled quantity is the sustained AprilTag
    decode cadence of a Raspberry Pi 5 at the flying quad_decimate (protocol
    §7.3). Substituting one for the other is exactly the "measured number of the
    wrong quantity" defect that already flipped this gate once (the invented
    30 fps).

    WHITELIST, NOT BLACKLIST (the fix this replaces). The old check rejected only
    `stream_fps_source` strings containing "replay", so EVERY other source --
    v4l2 webcam, an absent `source` key, a hand-written meta.json -- was accepted
    by default. A guard that enumerates the bad cases is wrong the moment a new
    one appears; this one enumerates the ONE sanctioned case and refuses the rest.

    The escape hatch is the correct one and is already the protocol's instruction:
    pass the bench number explicitly with `--stream-fps` (§7.3). That is a
    deliberate, recorded act by the operator, not a silent substitution.
    """
    src = str(meta.get("source") or "").strip()
    if not src:
        return False, ("meta.json records NO `source` -- provenance unknown, so "
                       "the rate cannot be attributed to a sanctioned capture")
    if src in ALLOWED_FPS_CAPTURE_SOURCES:
        return True, f"meta.json source='{src}' (sanctioned Pi-5 capture)"
    return False, (f"meta.json source='{src}' is NOT a sanctioned Pi-5 capture "
                   f"source (allowed: {', '.join(ALLOWED_FPS_CAPTURE_SOURCES)})")


def resolve_stream_fps(args, meta):
    """(stream_fps or None, human-readable source). NO DEFAULT -- see the module
    docstring: the burn scales as 1/fps and the verdict flips across ~24 fps at
    the predicted R_decode90, so an invented rate would decide ~$740.

    Accepted, in order: (1) an explicit --stream-fps (the one-line override for a
    bench-measured number, protocol §7.3); (2) meta.json `decode_loop_fps` -- the
    WRITE-EXCLUSIVE grab+decode cadence, which is the quantity the burn model
    actually wants; (3) meta.json `stream_fps` -- UNLESS `stream_fps_source` says
    it came from pi_capture's dir-REPLAY backend, whose timestamps are synthetic
    and therefore not a capture-rate measurement.

    (2) AND (3) ARE BOTH GATED ON THE CAPTURE-SOURCE WHITELIST (2026-07-25,
    builder ruling -- see capture_source_allowed). A rate the session measured on
    a laptop webcam, or on an unidentified source, is REFUSED outright; the gate
    then returns UNCERTAIN naming the missing measurement, which is the correct
    fail-closed answer for a desk rehearsal. `--stream-fps` is unaffected: an
    explicit bench number is the sanctioned path.

    WRONG-QUANTITY WARNING (2026-07-25, review 2). `stream_fps` is the RECORDER
    LOOP rate: it pays a per-frame PNG write the flight loop never pays, and with
    --no-decode-tags it pays no AprilTag decode cost at all, while the burn model
    `R_streak_burn = (E[T]/stream_fps) x V_closing` needs the ONBOARD DECODE
    cadence at flight time. pi_capture now records `decode_loop_fps` for exactly
    this (write-exclusive), so it is preferred; when only `stream_fps` is
    available the returned source string NAMES the discrepancy rather than
    presenting a differently-measured number as the modelled one.
    """
    if getattr(args, "stream_fps", None) is not None:
        return float(args.stream_fps), "--stream-fps (explicit; bench-measured, protocol §7.3)"
    # THE SOURCE WHITELIST comes FIRST: it decides whether this session's own
    # numbers may be consumed at all, before which of them is preferred.
    src_ok, src_detail = capture_source_allowed(meta)
    if not src_ok and (_pf(meta.get("decode_loop_fps")) or _pf(meta.get("stream_fps"))):
        return None, (
            f"REFUSED (capture-source whitelist): {src_detail}. A desk-rehearsal "
            f"or unidentified capture may NOT supply the money gate's 1/fps "
            f"divisor -- the modelled quantity is the SUSTAINED Pi 5 AprilTag "
            f"decode cadence at the flying quad_decimate (protocol §7.3). Bench "
            f"it and pass --stream-fps.")
    dec_fps = _pf(meta.get("decode_loop_fps"))
    if dec_fps is not None and dec_fps > 0:
        return dec_fps, ("meta.json decode_loop_fps (grab+decode, PNG-write "
                         "EXCLUDED -- the modelled quantity; still a TRIPOD-rig "
                         "rate, not the flying Pi 5 at the flight quad_decimate)")
    fps = _pf(meta.get("stream_fps"))
    src = str(meta.get("stream_fps_source") or "").strip()
    if fps is not None and fps > 0:
        if FPS_SOURCE_REJECT_MARK in src.lower():
            return None, (f"REJECTED meta.json stream_fps={fps} — stream_fps_source="
                          f"'{src}' is not a capture-rate measurement")
        warn = ""
        if not meta.get("tag_decode"):
            warn = (" [WRONG QUANTITY: this session captured with tag decode OFF, "
                    "so the rate carries NO decode cost -- it is a recorder "
                    "throughput, not a decode cadence. Prefer --stream-fps <§7.3 "
                    "Pi 5 bench at the flying quad_decimate>]")
        elif meta.get("decode_loop_fps") is None:
            warn = (" [DOWNGRADED: recorder-loop rate, INCLUDES the per-frame PNG "
                    "write the flight loop never pays -> pessimistic; no "
                    "decode_loop_fps in this session]")
        return fps, f"meta.json stream_fps ({src or 'source unspecified'}){warn}"
    return None, ("NOT SUPPLIED — no --stream-fps and no meta.json stream_fps "
                  "(pi_capture records it from real capture timestamps; bench it "
                  "per protocol §7.3)")


def run(args):
    session_stats = {}
    frames, index_map, meta, tags_map = load_session(
        args.session_dir, ignore_tags_csv=getattr(args, "redecode", False),
        stats=session_stats)
    # ORPHAN ROWS ARE COUNTED AND REFUSED (2026-07-26), the mirror of
    # decode_frames' partial-coverage refusal: there, frames/ has rows tags.csv
    # is missing; here, the CSVs name frames that frames/ no longer holds. Both
    # mean the session on disk and the session that was recorded are not the same
    # session -- most often because the ground/setup frames were deleted before
    # scoring, which is exactly the case whose rows used to be re-bound onto
    # surviving in-flight frames.
    n_orph = sum(v["orphan_rows"] for v in session_stats.values())
    if n_orph:
        detail = ", ".join(f"{k}.csv {v['orphan_rows']}/{v['rows']}"
                           for k, v in sorted(session_stats.items())
                           if v["orphan_rows"])
        if not getattr(args, "allow_orphan_rows", False):
            raise SystemExit(
                f"[tripod] REFUSED: {detail} row(s) name a frame that is NOT in "
                f"frames/. The session's CSVs and its frames disagree -- usually "
                f"frames were deleted after capture (the near-field ground/setup "
                f"shots). Those rows are DROPPED, so the curve is scored on a "
                f"smaller population than the CSVs describe. Re-copy the session, "
                f"or accept the shrunken denominator explicitly with "
                f"--allow-orphan-rows (the count is stamped into the summary).")
        print(f"[tripod] [WARN] --allow-orphan-rows: {detail} row(s) name a frame "
              f"not in frames/ and were DROPPED (counted, not absorbed)")
    calib = load_calib(args.calib)

    # PARAMETER PROVENANCE (2026-07-25, review 2): every load-bearing scalar
    # resolves to (value, SOURCE) so verdict.txt/gate.json can say when a number
    # came from a DEFAULT rather than from the session. `_pf(...) or DEFAULT`
    # also silently swallowed a legitimate 0.
    def _resolve(cli_val, meta_key, default, label):
        if cli_val is not None:
            return float(cli_val), f"--{label} (explicit)"
        mv = _pf(meta.get(meta_key))
        if mv is not None:
            return mv, f"meta.json {meta_key}"
        return default, "DEFAULT -- not in meta.json"

    tag_size, tag_size_src = _resolve(args.tag_size, "tag_size_m",
                                      DEFAULT_TAG_SIZE_M, "tag-size")
    if tag_size_src.startswith("DEFAULT"):
        print(f"[tripod] [WARN] tag_size not supplied (meta.json missing/null) -- "
              f"using DEFAULT {tag_size:.3f} m. Capture with --tag-size so the "
              f"session records it: every tag-recovered range scales LINEARLY in "
              f"this number (docs/placard_mount.md §2: the ADOPTED tag edge is "
              f"0.350 m on a 0.450 m sheet).")
    stream_fps, fps_source = resolve_stream_fps(args, meta)
    v_close, v_close_src = _resolve(args.closing_speed, "closing_speed_mps",
                                    DEFAULT_V_CLOSING_MPS, "closing-speed")
    drone_size, drone_size_src = _resolve(args.drone_size, "drone_size_m",
                                          DEFAULT_DRONE_SIZE_M, "drone-size")
    min_bin_n = getattr(args, "min_bin_n", DEFAULT_MIN_BIN_N)

    # ---- quad_decimate (ADR-0082): the decode setting is a RECORDED, CHECKED
    # parameter, not a silent library default. The session's own value comes from
    # pi_capture's meta.json stamp; --quad-decimate overrides it for an offline
    # re-decode (the §4.2b reclaim lever), and the two must agree unless the
    # operator says otherwise.
    session_qd = _pf(meta.get("quad_decimate"))
    if getattr(args, "quad_decimate", None) is not None:
        scoring_qd = float(args.quad_decimate)
        scoring_qd_src = "--quad-decimate (explicit)"
    elif session_qd is not None:
        scoring_qd, scoring_qd_src = session_qd, "meta.json quad_decimate"
    else:
        scoring_qd = DEFAULT_QUAD_DECIMATE
        scoring_qd_src = (f"DEFAULT {DEFAULT_QUAD_DECIMATE} -- the session does "
                          f"NOT record one (capture with pi_capture, which stamps it)")
    decimate_info = check_decimate_consistency(
        {"session meta.json (capture)": session_qd,
         "this scoring pass": scoring_qd},
        allow_mismatch=getattr(args, "allow_decimate_mismatch", False))
    if session_qd is None:
        print(f"[tripod] [WARN] this session does NOT record quad_decimate -- the "
              f"decimation that produced its frames/tags.csv is UNKNOWN. Scoring at "
              f"{scoring_qd}; capture with pi_capture (>=2026-07-25) so meta.json "
              f"stamps it (ADR-0082 / protocol §4.2b).")
    if decimate_info["mismatch"]:
        print(f"[tripod] [WARN] quad_decimate MISMATCH accepted: "
              f"{decimate_info['values']} -- this is the §4.2b reclaim comparison, "
              f"not a like-for-like re-score; --stream-fps must be the §7.3 bench "
              f"AT THE SCORED DECIMATE.")

    mount_index, mount_index_src, mount_warn = resolve_mount_index(
        _pf(getattr(args, "mount_index_deg", None)), meta, args.session_dir)
    if mount_warn:
        print(f"[tripod] [WARN] {mount_warn}")
    th_eng = float(getattr(args, "engagement_incidence_deg", 0.0) or 0.0)
    inc_bin = float(getattr(args, "incidence_bin_deg", DEFAULT_INCIDENCE_BIN_DEG))

    param_provenance = {
        "tag_size_m": (f"{tag_size:.3f} m", tag_size_src),
        "drone_size_m": (f"{drone_size:.3f} m", drone_size_src),
        "V_closing_mps": (f"{v_close:.1f} m/s", v_close_src),
        "stream_fps": (f"{stream_fps}", fps_source),
        "quad_decimate": (f"{scoring_qd}", scoring_qd_src),
        "theta_eng_deg": (f"{th_eng:.1f}", "--engagement-incidence-deg (§8.1)"
                          if th_eng else "DEFAULT 0 deg -- gate measured DEAD-ON"),
        "mount_index_deg": (f"{mount_index}",
                            mount_index_src
                            if mount_index is not None else
                            "NOT RECORDED -- §4.2b wants it on every pass card"),
    }
    accept_quality = ({"ok"} if getattr(args, "require_quality", "any") == "ok"
                      else None)

    os.makedirs(args.out_dir, exist_ok=True)
    tag = args.tag or os.path.basename(os.path.normpath(args.session_dir))
    outp = os.path.join(args.out_dir, tag)
    os.makedirs(outp, exist_ok=True)

    # Provenance of the TRUE ranges, if range_truth_join wrote one.
    range_truth = None
    rtp = os.path.join(args.session_dir, "range_truth.json")
    if os.path.exists(rtp):
        try:
            range_truth = json.loads(open(rtp).read())
        except (ValueError, OSError):
            range_truth = None

    # Curve (a)
    if tags_map is not None and not getattr(args, "redecode", False):
        print("[tripod] NOTE: reusing the session's tags.csv. If it is "
              "pi_capture's CAPTURE-TIME decode, protocol §6 asks for an OFFLINE "
              "re-decode instead (decouples capture from detector) -- use "
              "--redecode.")
    decode = decode_frames(
        frames, calib, tag_size, tags_map=tags_map,
        allow_partial_tags=getattr(args, "allow_partial_tags", False),
        quad_decimate=scoring_qd)
    if tags_map is None:
        write_tags_csv(os.path.join(outp, "tags.csv"), frames, decode)
    bins, r_any, r90, acc, n_binnable, n_unbinnable = curve_a(
        frames, decode, index_map, args.range_bin, args.max_range,
        accept_quality=accept_quality, min_bin_n=min_bin_n)
    r90w, r90_stop, r90_stop_lo, n_at_r90, coverage_gaps = r90_walk(
        bins, args.range_bin, min_bin_n=min_bin_n)
    assert abs(r90w - r90) < 1e-9, "curve_a and r90_walk disagree"
    n_decoded = sum(1 for _, b in frames if decode[b][0])
    have_truth = n_binnable > 0
    quality = quality_summary(frames, index_map)

    incidence = curve_a_incidence(frames, decode, index_map, args.range_bin,
                                  args.max_range, inc_bin_deg=inc_bin,
                                  accept_quality=accept_quality)
    if not incidence["rate_computable"]:
        print(f"[tripod] [WARN] curve (a) x INCIDENCE is an ANNOTATION, not a rate: "
              f"{incidence['incidence_source_counts']['index']} of "
              f"{incidence['n_binnable']} binnable frames carry a per-frame "
              f"incidence_deg in index.csv, so the NON-decodes have no incidence "
              f"and no cell has an honest denominator (protocol §4.2b)")
        if incidence.get("cells_are_partial"):
            print(f"[tripod] [WARN] ...and the {len(incidence['cells'])} cell(s) "
                  f"below come from that SUBSET only -- a shrunken denominator "
                  f"missing {incidence['n_binnable'] - incidence['incidence_source_counts']['index']} "
                  f"binnable frame(s). Re-join with range_truth_join.py "
                  f"--incidence --mount-index-deg so EVERY frame has an aspect.")

    d_rate = decode_rate_near(bins, r90, args.range_bin)
    _gate_kw = dict(n_decoded=n_decoded, min_decoded=args.min_decoded,
                    have_truth=have_truth, r90_stop_reason=r90_stop,
                    r90_stop_lo=r90_stop_lo, n_at_r90_bin=n_at_r90,
                    engagement_incidence_deg=th_eng)
    gate = gate_verdict(r90, r_any, d_rate, stream_fps, args.handoff_streak,
                        v_close, args.tgo_min, **_gate_kw)
    gate_hi = gate_verdict(r90, r_any, d_rate, stream_fps, args.handoff_streak,
                           args.closing_speed_hi, args.tgo_min, **_gate_kw)
    for g in coverage_gaps:
        print(f"[tripod] [WARN] coverage gap at {g:.0f}-{g + args.range_bin:.0f} m: "
              f"NO binnable frames -- the >=90% walk stops at a hole, it does not "
              f"jump it (re-shoot that station)")
    if gate.get("r90_data_bounded"):
        print(f"[tripod] [WARN] R_decode90 = {r90:.2f} m is bounded by MISSING DATA "
              f"({r90_stop}) -- a LOWER BOUND, so the gate returns UNCERTAIN, never PASS")
    if 0 < n_at_r90 < min_bin_n:
        print(f"[tripod] [WARN] R_decode90 was set by a bin holding only "
              f"n={n_at_r90} frames (< --min-bin-n {min_bin_n})")

    write_curve_a_csv(os.path.join(outp, "curve_a_decode.csv"), bins, args.range_bin)
    write_curve_a_incidence_csv(os.path.join(outp, "curve_a_incidence.csv"),
                                incidence)
    plot_curve_a(os.path.join(outp, "curve_a.png"), bins, acc, gate, args.range_bin)

    # Curve (b) -- PRIMARY (real-data candidate) + the historical SIM bar, same frames.
    # getattr-guarded so any programmatic caller predating --weights-bar (e.g. the
    # self-test namespace) keeps the old single-model behaviour exactly.
    bar_w = None if getattr(args, "no_weights_bar", False) else getattr(args, "weights_bar", None)
    models = [(_stem(args.weights), args.weights)]
    if bar_w and os.path.abspath(bar_w) != os.path.abspath(args.weights):
        models.append((_stem(bar_w), bar_w))
    b_results = curve_b_multi(frames, decode, calib, models, args.nn_conf,
                              drone_size, args.range_bin, args.max_range,
                              n_bands=args.pos_bands)
    b_note_lines, b_written = [], []
    for i, (label, w, bins_i, note_i) in enumerate(b_results):
        role = "PRIMARY (candidate)" if i == 0 else "BAR (historical/sim)"
        b_note_lines.append(f"[{role}] {label}  ({os.path.relpath(w, _REPO_ROOT_TS)})")
        b_note_lines.append(f"    {note_i}")
        if bins_i is None:
            continue
        if i == 0:
            # PRIMARY keeps the canonical filenames (backward compatible).
            csv_name, png_name = "curve_b_recall.csv", "curve_b.png"
        else:
            csv_name, png_name = f"curve_b_recall_{label}.csv", f"curve_b_{label}.png"
        write_curve_b_csv(os.path.join(outp, csv_name), bins_i)
        plot_curve_b(os.path.join(outp, png_name), bins_i)
        b_written.append(csv_name)
        h, t, rec = bins_overall(bins_i)
        b_note_lines.append(
            f"    overall recall {100 * rec:.1f}% ({h}/{t}) -> {csv_name}"
            if rec is not None else f"    no scorable cells -> {csv_name}")
    if len(b_results) > 1 and all(r[2] is not None for r in b_results):
        r0 = bins_overall(b_results[0][2])[2]
        r1 = bins_overall(b_results[1][2])[2]
        if r0 is not None and r1 is not None:
            b_note_lines.append(
                f"  DELTA (primary - bar) = {100 * (r0 - r1):+.1f} pts overall. A LOW bar "
                f"number is the EXPECTED sim->real NULL, not a markerless verdict; read the "
                f"PRIMARY row against the §8.2 threshold.")
    b_note = "\n".join(b_note_lines)

    # §8.2 band coverage is read off the PRIMARY model's cells (the candidate
    # the threshold is about); None when the NN half did not run at all.
    primary_bins = b_results[0][2] if b_results else None
    band_cov = (curve_b_band_coverage(primary_bins, args.range_bin)
                if primary_bins is not None else None)
    if band_cov is not None and not band_cov["answerable"]:
        print(f"[tripod] [WARN] §8.2 UNANSWERABLE: 0 of {band_cov['n_frames_scored']} "
              f"scored curve-(b) frames fall in the "
              f"{band_cov['band_lo_m']:.0f}-{band_cov['band_hi_m']:.0f} m threshold "
              f"band -- do NOT quote overall_recall as the §8.2 answer")

    summary = build_summary(gate, gate_hi, bins, b_note, meta, n_binnable,
                            n_unbinnable, args.range_bin, fps_source=fps_source,
                            quality=quality, range_truth=range_truth,
                            param_provenance=param_provenance,
                            coverage_gaps=coverage_gaps, min_bin_n=min_bin_n,
                            band_coverage=band_cov, incidence=incidence,
                            decimate_info=decimate_info)
    with open(os.path.join(outp, "verdict.txt"), "w") as f:
        f.write(summary + "\n")
    with open(os.path.join(outp, "gate.json"), "w") as f:
        json.dump({"conservative": gate, "aggressive": gate_hi,
                   "tag_size_m": tag_size, "drone_size_m": drone_size,
                   "n_decoded": n_decoded, "n_binnable": n_binnable,
                   # Rows the session's CSVs carry for frames that are NOT on
                   # disk: dropped, and COUNTED here so the denominator this
                   # curve was scored on is on the record (2026-07-26).
                   "session_csv_rows": session_stats,
                   "stream_fps": stream_fps, "stream_fps_source": fps_source,
                   # PARAMETER PROVENANCE: which numbers came from the session
                   # and which from a DEFAULT (review 2).
                   "tag_size_source": tag_size_src,
                   "drone_size_source": drone_size_src,
                   "V_closing_source": v_close_src,
                   # THE DECODE SETTING (ADR-0082) + its consistency check, and
                   # the incidence axis protocol §4.2b mandates.
                   "quad_decimate": scoring_qd,
                   "quad_decimate_source": scoring_qd_src,
                   "quad_decimate_check": decimate_info,
                   "mount_index_deg": mount_index,
                   "engagement_incidence_deg": th_eng,
                   "curve_a_incidence": {
                       k: v for k, v in incidence.items() if k != "cells"},
                   "curve_a_incidence_cells": [
                       {"range_lo_m": rlo, "range_hi_m": rlo + args.range_bin,
                        "incidence_lo_deg": ilo,
                        "incidence_hi_deg": ilo + inc_bin,
                        "n_decoded": incidence["cells"][(rlo, ilo)][0],
                        "n_total": incidence["cells"][(rlo, ilo)][1]}
                       for (rlo, ilo) in sorted(incidence["cells"])],
                   # curve (a) walk provenance: the per-bin counts the >=90%
                   # envelope was walked over, the stop reason, and every
                   # coverage hole between the nearest and farthest bin.
                   "curve_a_bins": [
                       {"lo_m": lo, "hi_m": lo + args.range_bin,
                        "n_decoded": bins[lo][0], "n_total": bins[lo][1]}
                       for lo in sorted(bins)],
                   "min_bin_n": min_bin_n,
                   "coverage_gaps_m": coverage_gaps,
                   "r90_stop_reason": r90_stop,
                   "r90_stop_lo_m": r90_stop_lo,
                   "n_at_r90_bin": n_at_r90,
                   # §8.2 answerability: False means NO scored cell overlaps the
                   # 10-25 m threshold band, so overall_recall is NOT the §8.2
                   # answer (review 2; the six 15-20 m NN passes score zero
                   # tag-truthed frames because the tag's ceiling is ~7 m).
                   "section_8_2_answerable": (band_cov["answerable"]
                                              if band_cov is not None else None),
                   "curve_b_band_coverage": band_cov,
                   "range_quality_counts": quality,
                   "range_truth_provenance": range_truth,
                   # curve (b) provenance: which model produced which recall number.
                   # Curve (b) gates ONLY the Hailo/markerless phase (protocol §8.2).
                   "curve_b_models": [
                       {"role": "primary" if i == 0 else "bar",
                        "label": lab, "weights": os.path.relpath(w, _REPO_ROOT_TS),
                        "scored": bins_i is not None,
                        "hits": bins_overall(bins_i)[0] if bins_i else None,
                        "total": bins_overall(bins_i)[1] if bins_i else None,
                        "overall_recall": bins_overall(bins_i)[2] if bins_i else None}
                       for i, (lab, w, bins_i, _n) in enumerate(b_results)]},
                  f, indent=2)
    print(summary)
    extra = ("" if not b_written else ", " + ", ".join(b_written))
    print(f"\n[tripod] outputs -> {outp}/  (curve_a_decode.csv, curve_a.png, "
          f"verdict.txt, gate.json{extra})")
    # EXIT CODE. Default stays 0 for EVERY verdict so no existing manual/doc
    # invocation changes. With --gate the code CARRIES the verdict, because
    # "could not decide" and "passed" must not be the same signal to a wrapper
    # (review 2 HIGH: a FAIL and a PASS were indistinguishable to any script
    # that ever automates the ~$740 order on this tool's exit code).
    if getattr(args, "gate", False):
        return {"PASS": 0, "FAIL": 1}.get(gate["verdict"], 2)
    return 0


# --------------------------------------------------------------------------
# Self-test: build a synthetic session, score curve (a) + the gate, no hardware
# --------------------------------------------------------------------------
def _build_synth_session(out_dir):
    """Render tags at KNOWN ranges via synth_tag_frames' pinhole composite,
    writing the pi_capture layout (frames/ + index.csv + meta.json). Near ranges
    -> big decodable tags; far ranges -> tags too small to decode (a clean
    monotone falloff). Returns (near_ranges, far_ranges)."""
    import cv2
    sys.path.insert(0, HERE)
    import synth_tag_frames as S
    tag_img = cv2.imread(S.TAG_PNG)
    if tag_img is None:
        raise SystemExit(f"cannot read tag texture {S.TAG_PNG}")
    rng = np.random.default_rng(0)
    frames_dir = os.path.join(out_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)
    tag_size = 0.5  # plumbing fixture size (matches synth default); clean falloff
    # 9.0 m added 2026-07-25 so the 8-10 m bin carries 6 frames instead of 3 and
    # therefore clears DEFAULT_MIN_BIN_N. Without it the stepped walk stops one
    # bin EARLIER (bin_underpopulated at 8 m) and R_decode90 would move 10 -> 8 m
    # for a fixture reason rather than an optical one. 30 px at 9 m still decodes
    # 3/3 (measured), so the near band stays a clean 100%.
    near = [4.0, 5.0, 6.0, 7.0, 8.0, 9.0]  # 30-84 px -> decode ~100%
    far = [24.0, 28.0, 32.0]              # 10-14 px -> ~0 after quad_decimate 2.0
    offsets = (-1.0, 0.0, 1.0)
    idx_rows = []
    i = 0
    for r in near + far:
        for off in offsets:
            i += 1
            bg = S._load_background(rng, cv2)
            res = S.composite_tag(bg, tag_img, cv2, r, tag_size, off_x_m=off)
            if res is None:
                continue
            frame = res[0]
            fn = f"synth_{i:04d}_r{r:05.1f}_x{off:+.1f}.png"
            cv2.imwrite(os.path.join(frames_dir, fn), frame)
            idx_rows.append({"frame": fn, "idx": i, "true_range_m": f"{r:.2f}"})
    with open(os.path.join(out_dir, "index.csv"), "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=["frame", "idx", "true_range_m"])
        wr.writeheader()
        wr.writerows(idx_rows)
    calib = dict(fx=S.FX, fy=S.FY, cx=S.CX, cy=S.CY,
                 resolution=dict(width=S.W, height=S.H), dist_coeffs=[0.0] * 5)
    with open(os.path.join(out_dir, "calib.json"), "w") as f:
        json.dump(calib, f)
    with open(os.path.join(out_dir, "meta.json"), "w") as f:
        # stream_fps is a MEASUREMENT in a real session; the self-test stands in
        # the protocol §7.3 paper anchor and LABELS it as a stand-in, so the
        # no-invented-fps rule stays visible here too.
        # tag_decode=True: the §7.3 paper anchor IS a decode-inclusive AprilTag
        # cadence, so resolve_stream_fps must not flag it as the wrong quantity.
        json.dump(dict(tag_size_m=tag_size, drone_size_m=DEFAULT_DRONE_SIZE_M,
                       stream_fps=PI5_APRILTAG_FPS_PAPER_ANCHOR,
                       stream_fps_source="self-test stand-in (protocol §7.3 paper anchor)",
                       # source: the fixture stands in for a REAL Pi capture so
                       # the capture-source WHITELIST's accept path is exercised
                       # end-to-end (2026-07-25 builder ruling). The refuse path
                       # is pinned separately, on explicit metas, in self_test().
                       source=ALLOWED_FPS_CAPTURE_SOURCES[0],
                       tag_decode=True, aspect="approach",
                       # the ADR-0082 stamp: the fixture decodes at the library
                       # default, and run() must agree with it, not assume it.
                       quad_decimate=DEFAULT_QUAD_DECIMATE), f)
    return near, far


def self_test():
    import tempfile
    ok = True
    print("[self-test] --- part 1: gate arithmetic (pure function) ---")

    # Contrived PASS: R90=8, p=0.9, 30 fps, streak 5, 9 m/s. RUN-LENGTH gate (ADR-0079):
    #   E[T]=(1-0.9^5)/(0.9^5*0.1)=6.935 fr, burn=(6.935/30)*9=2.080, t_go=(8-2.080)/9=0.658 -> PASS
    g = gate_verdict(8.0, 8.0, 0.9, 30.0, 5, 9.0, 0.5, n_decoded=40)
    exp_burn = (_streak_burn_frames(0.9, 5) / 30.0) * 9.0
    exp_tgo = (8.0 - exp_burn) / 9.0
    c1 = (abs(g["R_streak_burn_m"] - exp_burn) < 1e-3 and
          abs(g["t_go_s"] - exp_tgo) < 1e-3 and g["verdict"] == "PASS")
    print(f"[self-test] PASS-case: burn={g['R_streak_burn_m']} (exp {exp_burn:.4f}) "
          f"t_go={g['t_go_s']} (exp {exp_tgo:.4f}) verdict={g['verdict']} : {'OK' if c1 else 'FAIL'}")
    ok = ok and c1
    # Run-length must be >= mean-rate (conservative direction).
    c1b = g["R_streak_burn_m"] >= g["R_streak_burn_meanrate_m"]
    print(f"[self-test] run-length {g['R_streak_burn_m']} >= mean-rate "
          f"{g['R_streak_burn_meanrate_m']} : {'OK' if c1b else 'FAIL'}")
    ok = ok and c1b

    # Contrived FAIL: aggressive 20 m/s, R90=6. burn=(6.935/30)*20=4.623, t_go=(6-4.623)/20=0.069 -> FAIL
    g2 = gate_verdict(6.0, 6.0, 0.9, 30.0, 5, 20.0, 0.5, n_decoded=40)
    exp_tgo2 = (6.0 - (_streak_burn_frames(0.9, 5) / 30.0) * 20.0) / 20.0
    c2 = abs(g2["t_go_s"] - exp_tgo2) < 1e-3 and g2["verdict"] == "FAIL"
    print(f"[self-test] FAIL-case: t_go={g2['t_go_s']} (exp {exp_tgo2:.4f}) "
          f"verdict={g2['verdict']} : {'OK' if c2 else 'FAIL'}")
    ok = ok and c2

    # UNCERTAIN: thin data (below the floor).
    g3 = gate_verdict(8.0, 8.0, 0.9, 30.0, 5, 9.0, 0.5, n_decoded=2, min_decoded=5)
    c3 = g3["verdict"] == "UNCERTAIN"
    print(f"[self-test] thin-data -> {g3['verdict']} : {'OK' if c3 else 'FAIL'}")
    ok = ok and c3
    # UNCERTAIN: no truth ranges.
    g4 = gate_verdict(0.0, 0.0, 0.0, 30.0, 5, 9.0, 0.5, have_truth=False)
    c4 = g4["verdict"] == "UNCERTAIN"
    print(f"[self-test] no-truth -> {g4['verdict']} : {'OK' if c4 else 'FAIL'}")
    ok = ok and c4

    # UNCERTAIN: NO stream_fps supplied -> the gate must NOT invent 30 fps
    # (2026-07-24). Same optical data, three rates, three different answers.
    g6 = gate_verdict(8.0, 8.0, 0.9, None, 5, 9.0, 0.5, n_decoded=40)
    c6a = (g6["verdict"] == "UNCERTAIN" and "stream_fps" in g6["reason"]
           and g6["t_go_s"] is None and g6["stream_fps"] is None)
    print(f"[self-test] no-fps -> {g6['verdict']} (t_go={g6['t_go_s']}) : "
          f"{'OK' if c6a else 'FAIL'}")
    ok = ok and c6a
    # The flip itself, at the PREDICTED R_decode90 for the adopted 0.35 m
    # placard (7.10 m, docs/placard_sizing.md §4): 30 fps PASS vs 20 Hz FAIL.
    g30 = gate_verdict(7.10, 7.10, 0.9, 30.0, 5, 9.0, 0.5, n_decoded=40)
    g20 = gate_verdict(7.10, 7.10, 0.9, 20.0, 5, 9.0, 0.5, n_decoded=40)
    c6b = g30["verdict"] == "PASS" and g20["verdict"] == "FAIL"
    print(f"[self-test] fps FLIPS the $740 gate at R90=7.10 m: 30 fps t_go="
          f"{g30['t_go_s']} -> {g30['verdict']} | 20 Hz t_go={g20['t_go_s']} -> "
          f"{g20['verdict']} : {'OK' if c6b else 'FAIL'}")
    ok = ok and c6b
    # An fps-INDEPENDENT verdict must still come out: R_decode90=0 with sparse
    # decodes is FAIL on decode evidence alone.
    g0 = gate_verdict(0.0, 5.0, 0.4, None, 5, 9.0, 0.5, n_decoded=40)
    c6c = g0["verdict"] == "FAIL"
    print(f"[self-test] fps-independent leg (R90=0, no fps) -> {g0['verdict']} : "
          f"{'OK' if c6c else 'FAIL'}")
    ok = ok and c6c

    # CAPTURE-SOURCE WHITELIST (2026-07-25 builder ruling: a desk rehearsal may
    # NOT supply the money gate's 1/fps divisor). The old check was a BLACKLIST
    # of stream_fps_source strings containing "replay", so a v4l2 laptop-webcam
    # session with decode on -- a different CPU, driver and sensor from the Pi 5
    # the burn model is about -- fed the ~$740 verdict silently.
    _A = lambda v=None: type("A", (), {"stream_fps": v})()
    _pi = {"source": "picamera2", "tag_decode": True, "decode_loop_fps": 18.0}
    _desk = {"source": "v4l2", "tag_decode": True, "decode_loop_fps": 31.0,
             "stream_fps": 29.0}
    _nosrc = {"tag_decode": True, "stream_fps": 18.5,
              "stream_fps_source": "measured: median inter-frame dt"}
    f_pi, s_pi = resolve_stream_fps(_A(), _pi)
    f_desk, s_desk = resolve_stream_fps(_A(), _desk)
    f_nosrc, _s_nosrc = resolve_stream_fps(_A(), _nosrc)
    f_ovr, s_ovr = resolve_stream_fps(_A(22.0), _desk)
    c6d = (f_pi == 18.0 and "picamera2" not in s_desk.lower().split("source='")[0]
           and f_desk is None and "REFUSED" in s_desk
           and f_nosrc is None and f_ovr == 22.0 and "--stream-fps" in s_ovr)
    print(f"[self-test] fps capture-source WHITELIST: picamera2 -> {f_pi}, "
          f"v4l2 desk rehearsal -> {f_desk} (REFUSED), no `source` -> {f_nosrc}, "
          f"explicit --stream-fps still {f_ovr} : {'OK' if c6d else 'FAIL'}")
    ok = ok and c6d
    # ...and the REFUSAL reaches the VERDICT: a desk session cannot produce a
    # PASS/FAIL, only UNCERTAIN naming the missing measurement.
    g_desk = gate_verdict(7.10, 7.10, 0.9, f_desk, 5, 9.0, 0.5, n_decoded=40)
    c6e = g_desk["verdict"] == "UNCERTAIN" and "stream_fps" in g_desk["reason"]
    print(f"[self-test] a desk-rehearsal session's rate cannot decide the gate "
          f"-> {g_desk['verdict']} : {'OK' if c6e else 'FAIL'}")
    ok = ok and c6e

    print("[self-test] --- part 1b: R_decode90 walk refuses missing data ---")
    # A MID-BAND HOLE (no 8-10 m frames at all). The old `for lo in sorted(bins)`
    # walk jumped the hole and reported 14 m; the stepped walk must stop AT it.
    holed = {4.0: [10, 10, []], 6.0: [10, 10, []],
             10.0: [10, 10, []], 12.0: [10, 10, []]}
    r, why, lo_stop, n_at, gaps = r90_walk(holed, 2.0)
    ch1 = (r == 8.0 and why == R90_STOP_ABSENT and lo_stop == 8.0 and gaps == [8.0])
    gh = gate_verdict(r, 12.0, 1.0, 30.0, 5, 9.0, 0.5, n_decoded=40,
                      r90_stop_reason=why, r90_stop_lo=lo_stop, n_at_r90_bin=n_at)
    ch1 = ch1 and gh["verdict"] == "UNCERTAIN" and "HOLE" in gh["reason"]
    print(f"[self-test] mid-band hole: r90={r} (not 14.0) stop={why} gaps={gaps} "
          f"gate={gh['verdict']} : {'OK' if ch1 else 'FAIL'}")
    ok = ok and ch1
    # ONE lucky far decode (n=1) must not extend the envelope or feed the gate.
    thin = {4.0: [10, 10, []], 6.0: [10, 10, []], 8.0: [1, 1, []]}
    r2, why2, lo2, n_at2, _g2 = r90_walk(thin, 2.0)
    g_thin = gate_verdict(r2, 8.5, decode_rate_near(thin, r2, 2.0), 30.0, 5, 9.0,
                          0.5, n_decoded=40, r90_stop_reason=why2,
                          r90_stop_lo=lo2, n_at_r90_bin=n_at2)
    ch2 = (r2 == 8.0 and why2 == R90_STOP_UNDERPOPULATED
           and g_thin["verdict"] == "UNCERTAIN")
    print(f"[self-test] n=1 far bin: r90={r2} (not 10.0) stop={why2} "
          f"gate={g_thin['verdict']} : {'OK' if ch2 else 'FAIL'}")
    ok = ok and ch2
    # Walking off the END of the data is also missing data, not a measured cutoff.
    full = {4.0: [10, 10, []], 6.0: [10, 10, []], 8.0: [10, 10, []]}
    r3, why3, lo3, n3, _g3 = r90_walk(full, 2.0)
    g_end = gate_verdict(r3, 10.0, 1.0, 30.0, 5, 9.0, 0.5, n_decoded=40,
                         r90_stop_reason=why3, r90_stop_lo=lo3, n_at_r90_bin=n3)
    ch3 = r3 == 10.0 and why3 == R90_STOP_END and g_end["verdict"] == "UNCERTAIN"
    print(f"[self-test] end-of-data: r90={r3} stop={why3} gate={g_end['verdict']} "
          f": {'OK' if ch3 else 'FAIL'}")
    ok = ok and ch3
    # A MEASURED cutoff still decides PASS/FAIL as before.
    meas = {4.0: [10, 10, []], 6.0: [10, 10, []], 8.0: [2, 10, []]}
    r4, why4, lo4, n4, _g4 = r90_walk(meas, 2.0)
    g_meas = gate_verdict(r4, 9.0, decode_rate_near(meas, r4, 2.0), 30.0, 5, 9.0,
                          0.5, n_decoded=40, r90_stop_reason=why4,
                          r90_stop_lo=lo4, n_at_r90_bin=n4)
    ch4 = r4 == 8.0 and why4 == R90_STOP_RATE and g_meas["verdict"] in ("PASS", "FAIL")
    print(f"[self-test] measured cutoff: r90={r4} stop={why4} "
          f"gate={g_meas['verdict']} : {'OK' if ch4 else 'FAIL'}")
    ok = ok and ch4
    # §8.2 answerability: a tag-decode-limited curve (b) cannot answer 10-25 m.
    near_cells = {(4.0, "middle"): [3, 4], (6.0, "middle"): [1, 4]}
    far_cells = {(4.0, "middle"): [3, 4], (12.0, "middle"): [1, 4]}
    cov_near = curve_b_band_coverage(near_cells, 2.0)
    cov_far = curve_b_band_coverage(far_cells, 2.0)
    ch5 = (cov_near["answerable"] is False and cov_near["n_frames_in_band"] == 0
           and cov_near["n_frames_scored"] == 8
           and cov_far["answerable"] is True and cov_far["n_frames_in_band"] == 4)
    print(f"[self-test] §8.2 answerable: near-only={cov_near['answerable']} "
          f"with-far={cov_far['answerable']} : {'OK' if ch5 else 'FAIL'}")
    ok = ok and ch5

    print("[self-test] --- part 1c: quad_decimate is checked, not assumed ---")
    # A recorded capture decimate that disagrees with the scoring decimate is a
    # REFUSAL (the curve would pool two different range multipliers), and
    # --allow-decimate-mismatch is the explicit §4.2b escape hatch. An UNRECORDED
    # value is unknown, not "equal" -- it must not manufacture a mismatch.
    try:
        check_decimate_consistency({"capture": 2.0, "scoring": 1.0})
        cq1 = False
    except SystemExit as e:
        cq1 = "MISMATCH" in str(e) and "--allow-decimate-mismatch" in str(e)
    ok_i = check_decimate_consistency({"capture": 2.0, "scoring": 1.0},
                                      allow_mismatch=True)
    same = check_decimate_consistency({"capture": 1.0, "scoring": 1.0})
    unk = check_decimate_consistency({"capture": None, "scoring": 2.0})
    three = check_decimate_consistency({"pass01": 2.0, "pass02": 2.0,
                                        "pass03": 1.0}, allow_mismatch=True)
    cq1 = (cq1 and ok_i["mismatch"] and ok_i["mismatch_allowed"]
           and not same["mismatch"] and not unk["mismatch"]
           and unk["unknown_sources"] == ["capture"]
           and three["distinct_recorded"] == [1.0, 2.0])
    print(f"[self-test] quad_decimate mismatch REFUSED (and allowed only "
          f"explicitly); unrecorded stays UNKNOWN : {'OK' if cq1 else 'FAIL'}")
    ok = ok and cq1
    # The incidence primitive, on synthetic poses (shared with pi_capture).
    c30, s30 = math.cos(math.radians(30.0)), math.sin(math.radians(30.0))
    R30 = np.array([[c30, 0.0, s30], [0.0, 1.0, 0.0], [-s30, 0.0, c30]])
    ci = (abs(tag_incidence_deg(np.eye(3), np.array([0.0, 0.0, 5.0]))) < 1e-9
          and abs(tag_incidence_deg(R30, np.array([0.0, 0.0, 5.0])) - 30.0) < 1e-9
          # a tag FACING the camera but seen off-axis is still oblique to the LOS
          and tag_incidence_deg(np.eye(3), np.array([5.0, 0.0, 5.0])) > 44.0
          and tag_incidence_deg(np.eye(3), np.zeros(3)) is None)
    print(f"[self-test] tag_incidence_deg on synthetic poses (0 / 30 / off-axis / "
          f"degenerate) : {'OK' if ci else 'FAIL'}")
    ok = ok and ci
    # The incidence AXIS rule: a rate needs a denominator that includes MISSES,
    # so tag-pose incidence (decoded frames only) may annotate but never rate.
    fr = [("/x/a.png", "a"), ("/x/b.png", "b")]
    dec_ann = {"a": TagObs(True, 5.0, 640.0, 400.0, 1, 12.0),
               "b": TagObs(False, None, None, None, 0, None)}
    imap_ann = {"a": {"true_range_m": "5.0"}, "b": {"true_range_m": "5.5"}}
    ann = curve_a_incidence(fr, dec_ann, imap_ann, 2.0, 40.0)
    imap_rate = {"a": {"true_range_m": "5.0", "incidence_deg": "12.0"},
                 "b": {"true_range_m": "5.5", "incidence_deg": "40.0"}}
    rate = curve_a_incidence(fr, dec_ann, imap_rate, 2.0, 40.0)
    ci2 = (ann["rate_computable"] is False and ann["cells"] == {}
           and ann["decoded_incidence"]["n"] == 1
           and "NOT A RATE CURVE" in ann["note"]
           and rate["rate_computable"] is True
           and rate["cells"] == {(4.0, 0.0): [1, 1], (4.0, 30.0): [0, 1]}
           and rate["decoded_incidence"]["n_beyond_validated_cos_law"] == 0)
    print(f"[self-test] incidence axis: tag-pose-only = ANNOTATION (no cells), "
          f"index.csv-supplied = a real rate curve : {'OK' if ci2 else 'FAIL'}")
    ok = ok and ci2
    # The incidence-aware gate (placard_mount §11 item 10): cos(theta_eng) derates
    # R_decode90, and theta_eng=0 must reproduce the old arithmetic EXACTLY.
    g_0 = gate_verdict(8.0, 8.0, 0.9, 30.0, 5, 9.0, 0.5, n_decoded=40)
    g_30 = gate_verdict(8.0, 8.0, 0.9, 30.0, 5, 9.0, 0.5, n_decoded=40,
                        engagement_incidence_deg=30.0)
    # theta=45 is past the cos-law's validated 32.6 deg -- used here only to prove
    # the ARITHMETIC flips the verdict, which is the whole point of §11 item 10.
    g_45 = gate_verdict(8.0, 8.0, 0.9, 30.0, 5, 9.0, 0.5, n_decoded=40,
                        engagement_incidence_deg=45.0)
    ci3 = (g_0["R_decode90_eff_m"] == 8.0 and g_0["verdict"] == "PASS"
           and abs(g_30["R_decode90_eff_m"] - 8.0 * math.cos(math.radians(30))) < 1e-3
           and g_30["t_go_s"] < g_0["t_go_s"] and g_30["verdict"] == "PASS"
           and g_45["verdict"] == "FAIL"
           and "cos(theta_eng" in g_45["arithmetic"]
           # ...and the derate ABOVE 32.6 deg says out loud that it is a
           # hypothesis, in the arithmetic AND as a machine-readable key, while
           # a validated 30 deg does not (2026-07-26).
           and g_45["theta_eng_beyond_validated_cos_law"] is True
           and "HYPOTHESIS" in g_45["arithmetic"]
           and g_30["theta_eng_beyond_validated_cos_law"] is False
           and "HYPOTHESIS" not in g_30["arithmetic"]
           and g_0["cos_law_validated_to_deg"] == COS_LAW_VALIDATED_TO_DEG)
    print(f"[self-test] incidence-aware gate: theta=0 -> t_go {g_0['t_go_s']} "
          f"{g_0['verdict']} | theta=30 -> R_eff {g_30['R_decode90_eff_m']} m, "
          f"t_go {g_30['t_go_s']} {g_30['verdict']} | theta=45 -> t_go "
          f"{g_45['t_go_s']} {g_45['verdict']} (the SAME optics, a different "
          f"engagement aspect) : {'OK' if ci3 else 'FAIL'}")
    ok = ok and ci3

    print("[self-test] --- part 2: end-to-end curve (a) on a synthetic session ---")
    with tempfile.TemporaryDirectory() as td:
        sess = os.path.join(td, "synthA")
        os.makedirs(sess, exist_ok=True)
        try:
            near, far = _build_synth_session(sess)
        except SystemExit as e:
            print(f"[self-test] synth session build FAILED: {e}")
            return False
        frames, index_map, meta, tags_map = load_session(sess)
        calib = load_calib(os.path.join(sess, "calib.json"))
        decode = decode_frames(frames, calib, float(meta["tag_size_m"]), quiet=True)
        bins, r_any, r90, acc, n_bin, n_unbin = curve_a(
            frames, decode, index_map, 2.0, 40.0)
        los = sorted(bins)
        rates = [bins[lo][0] / bins[lo][1] if bins[lo][1] else 0 for lo in los]
        print("[self-test] decode rate by 2 m bin:")
        for lo, rt in zip(los, rates):
            print(f"[self-test]   {lo:.0f}-{lo+2:.0f} m: {100*rt:.0f}% "
                  f"({bins[lo][0]}/{bins[lo][1]})")

        # (i) monotone non-increasing falloff (physics: a bigger tag decodes
        # more, so a farther bin can never out-decode a nearer one).
        mono = all(rates[i + 1] <= rates[i] + 1e-9 for i in range(len(rates) - 1))
        print(f"[self-test] monotone non-increasing falloff : {'OK' if mono else 'FAIL'}")
        ok = ok and mono
        # (ii) near band decodes, far band does not (a real envelope, not all-0/all-1).
        near_rate = np.mean([r for lo, r in zip(los, rates) if lo < 10])
        far_rate = np.mean([r for lo, r in zip(los, rates) if lo >= 18] or [0])
        falls = near_rate >= 0.9 and far_rate <= 0.1
        print(f"[self-test] near(<10 m)={100*near_rate:.0f}% far(>=18 m)={100*far_rate:.0f}% "
              f"(envelope falls off) : {'OK' if falls else 'FAIL'}")
        ok = ok and falls

        # (iii) end-to-end gate arithmetic matches the pure function on this
        # session, and R_decode90 landed at the near/far boundary (10 m = the
        # outer edge of the last sustaining bin, UNCHANGED by the 2026-07-25
        # stepped walk -- what changed is that the walk now STOPS at the 10-24 m
        # coverage hole and says so, instead of silently jumping it).
        d_rate = decode_rate_near(bins, r90, 2.0)
        w90, why90, lo90, n90, gaps90 = r90_walk(bins, 2.0)
        g5 = gate_verdict(r90, r_any, d_rate, PI5_APRILTAG_FPS_PAPER_ANCHOR,
                          DEFAULT_HANDOFF_STREAK, DEFAULT_V_CLOSING_MPS,
                          DEFAULT_TGO_MIN_S, n_decoded=sum(1 for _, b in frames if decode[b][0]),
                          r90_stop_reason=why90, r90_stop_lo=lo90, n_at_r90_bin=n90)
        ref = gate_verdict(r90, r_any, d_rate, PI5_APRILTAG_FPS_PAPER_ANCHOR,
                           DEFAULT_HANDOFF_STREAK, DEFAULT_V_CLOSING_MPS,
                           DEFAULT_TGO_MIN_S, n_decoded=999)
        c5 = (abs(r90 - 10.0) < 1e-9 and w90 == r90 and
              abs(g5["t_go_s"] - ref["t_go_s"]) < 1e-9 and
              g5["verdict"] in ("PASS", "UNCERTAIN"))
        print(f"[self-test] R_decode90={r90:.1f} m R_decode_any={r_any:.1f} m "
              f"t_go={g5['t_go_s']} s verdict={g5['verdict']} : {'OK' if c5 else 'FAIL'}")
        ok = ok and c5
        # (iii-b) ...and the hole IS reported, so the verdict is UNCERTAIN, not
        # the pre-2026-07-25 PASS on a session with a 14 m unsampled gap.
        c5b = (why90 == R90_STOP_ABSENT and lo90 == 10.0
               and g5["verdict"] == "UNCERTAIN" and g5["r90_data_bounded"] is True
               and [g for g in gaps90 if 10.0 <= g < 24.0])
        print(f"[self-test] synth session's 10-24 m coverage hole is REPORTED "
              f"(stop={why90} at {lo90} m, {len(gaps90)} gap bins) -> "
              f"{g5['verdict']} : {'OK' if c5b else 'FAIL'}")
        ok = ok and c5b

        # (iii-c) DELETED FRAMES MUST NOT RE-BIND THEIR ROWS (2026-07-26).
        # THE FIELD SHAPE: the operator deletes the ground/setup frames -- where
        # the placard was held 1-3 m from the camera -- before scoring. Those
        # rows' NEAR tag ranges used to land on the first surviving IN-FLIGHT
        # frames via the positional fallback, and the nearest-tag merge then let
        # the deleted frame's smaller range replace the survivor's real one:
        # curve (a)'s range accuracy, the survey cross-check and curve (b)'s
        # truth boxes all pulled toward NEAR with every check green. Fixture is
        # written from pi_capture's OWN header constants (producer->consumer).
        sess_del = os.path.join(td, "deleted_frames")
        os.makedirs(os.path.join(sess_del, "frames"), exist_ok=True)
        sys.path.insert(0, HERE)
        from pi_capture import INDEX_HEADER, TAGS_HEADER
        N_SETUP, N_TOTAL = 3, 8          # 3 near ground shots, then the pass
        with open(os.path.join(sess_del, "index.csv"), "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=INDEX_HEADER)
            wr.writeheader()
            for i in range(N_TOTAL):
                wr.writerow({"frame_idx": i, "frame_path": f"frames/{i:06d}.png",
                             "t_mono_s": f"{i * 0.05:.3f}",
                             "t_wall_unix": f"{1_752_700_000 + i * 0.05:.3f}",
                             "exposure_us": 2000, "gain": 1.0})
        with open(os.path.join(sess_del, "tags.csv"), "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=TAGS_HEADER)
            wr.writeheader()
            for i in range(N_TOTAL):
                wr.writerow({"frame_idx": i, "frame_path": f"frames/{i:06d}.png",
                             "n_tags": 1, "tag_id": 7, "decision_margin": 40.0,
                             "hamming": 0, "center_u": 640.0, "center_v": 400.0,
                             "corners_px": "", "incidence_deg": "",
                             # the tell-tale: setup shots are held NEAR
                             "range_m": f"{1.5 if i < N_SETUP else 12.0:.3f}"})
        for i in range(N_TOTAL):
            open(os.path.join(sess_del, "frames", f"{i:06d}.png"), "wb").write(b"")
        for i in range(N_SETUP):        # the operator deletes the ground shots
            os.remove(os.path.join(sess_del, "frames", f"{i:06d}.png"))
        st = {}
        _fr_d, imap_d, _m_d, tmap_d = load_session(sess_del, stats=st)
        c_del = (st["tags"]["orphan_rows"] == N_SETUP
                 and st["index"]["orphan_rows"] == N_SETUP
                 and sorted(tmap_d) == [f"{i:06d}" for i in range(N_SETUP, N_TOTAL)]
                 and all(int(r["frame_idx"]) == int(b) for b, r in tmap_d.items())
                 and all(abs(_tag_row_range(r) - 12.0) < 1e-9
                         for r in tmap_d.values()))
        print(f"[self-test] deleted setup frames: {st} -- every surviving frame "
              f"keeps its OWN row (ranges "
              f"{sorted({_tag_row_range(r) for r in tmap_d.values()})}), the "
              f"{N_SETUP} orphan rows are COUNTED : {'OK' if c_del else 'FAIL'}")
        ok = ok and c_del

        # (iv) NN half no-ops cleanly (onnxruntime absent in the main .venv, or
        # weights missing) -- returns None + a message, never crashes.
        b_bins, b_note = curve_b(frames, decode, calib, "/no/such/weights.onnx",
                                 DEFAULT_NN_CONF, DEFAULT_DRONE_SIZE_M, 2.0, 40.0)
        c6 = b_bins is None and "SKIP" in b_note
        print(f"[self-test] NN no-op path: {b_note[:70]}... : {'OK' if c6 else 'FAIL'}")
        ok = ok and c6

        # (iv-b) DUAL-MODEL curve (b) plumbing (2026-07-24): two models are scored
        # on the same frames, in order, each no-opping cleanly on missing weights.
        multi = curve_b_multi(frames, decode, calib,
                              [("primary", "/no/such/a.onnx"), ("bar", "/no/such/b.onnx")],
                              DEFAULT_NN_CONF, DEFAULT_DRONE_SIZE_M, 2.0, 40.0)
        c6b = (len(multi) == 2
               and [m[0] for m in multi] == ["primary", "bar"]
               and all(m[2] is None and "SKIP" in m[3] for m in multi)
               and _stem("/x/weights/nn_tier/n-mono.onnx") == "n-mono"
               and bins_overall({(8.0, "mid"): [3, 4]}) == (3, 4, 0.75)
               and bins_overall({})[2] is None)
        print(f"[self-test] dual-model curve (b) plumbing + label/rollup helpers : "
              f"{'OK' if c6b else 'FAIL'}")
        ok = ok and c6b

        # (v) full run() writes the expected artifacts under logs-style out dir.
        import types
        a = types.SimpleNamespace(
            session_dir=sess, calib=os.path.join(sess, "calib.json"),
            tag_size=None, drone_size=None, stream_fps=None, closing_speed=None,
            closing_speed_hi=DEFAULT_V_CLOSING_HI_MPS, handoff_streak=DEFAULT_HANDOFF_STREAK,
            tgo_min=DEFAULT_TGO_MIN_S, range_bin=2.0, max_range=40.0, pos_bands=3,
            weights="/no/such/weights.onnx", weights_bar=None, no_weights_bar=True,
            nn_conf=DEFAULT_NN_CONF,
            min_decoded=DEFAULT_MIN_DECODED, out_dir=os.path.join(td, "out"), tag="synthA")
        run(a)
        want = ["curve_a_decode.csv", "curve_a_incidence.csv", "curve_a.png",
                "verdict.txt", "gate.json"]
        made = [os.path.exists(os.path.join(td, "out", "synthA", w)) for w in want]
        c7 = all(made)
        print(f"[self-test] run() artifacts {dict(zip(want, made))} : {'OK' if c7 else 'FAIL'}")
        ok = ok and c7
        # END-TO-END: gate.json carries the decode setting + the incidence block,
        # and the fixture's stamp AGREES with the scoring pass (no mismatch).
        with open(os.path.join(td, "out", "synthA", "gate.json")) as f:
            gj = json.load(f)
        c7b = (gj["quad_decimate"] == DEFAULT_QUAD_DECIMATE
               and gj["quad_decimate_check"]["mismatch"] is False
               and gj["curve_a_incidence"]["rate_computable"] is False
               and gj["curve_a_incidence"]["decoded_incidence"]["n"] > 0
               and gj["engagement_incidence_deg"] == 0.0)
        print(f"[self-test] gate.json records quad_decimate="
              f"{gj['quad_decimate']} (check {gj['quad_decimate_check']['mismatch']}) "
              f"+ the incidence block (rate_computable="
              f"{gj['curve_a_incidence']['rate_computable']}, decoded n="
              f"{gj['curve_a_incidence']['decoded_incidence']['n']}) : "
              f"{'OK' if c7b else 'FAIL'}")
        ok = ok and c7b
        # ...and a scoring decimate that disagrees with the session's stamp is
        # REFUSED end-to-end, not merely inside the pure function.
        import types as _t
        a_bad = _t.SimpleNamespace(**{**vars(a), "quad_decimate": 1.0,
                                      "tag": "synthA_qd1"})
        try:
            run(a_bad)
            c7c = False
        except SystemExit as e:
            c7c = "quad_decimate MISMATCH" in str(e)
        a_ok = _t.SimpleNamespace(**{**vars(a), "quad_decimate": 1.0,
                                     "allow_decimate_mismatch": True,
                                     "tag": "synthA_qd1"})
        run(a_ok)
        with open(os.path.join(td, "out", "synthA_qd1", "gate.json")) as f:
            gj1 = json.load(f)
        c7c = c7c and gj1["quad_decimate"] == 1.0 and gj1["quad_decimate_check"]["mismatch"]
        print(f"[self-test] run() REFUSES a decimate that disagrees with the "
              f"session stamp, and --allow-decimate-mismatch records it "
              f"(scored qd={gj1['quad_decimate']}) : {'OK' if c7c else 'FAIL'}")
        ok = ok and c7c

        # (vi) THE SCHEMA JOIN (2026-07-24 defect): a REAL pi_capture index.csv
        # keys frames by `frame_path` / `frame_idx`, not `frame`/`filename`/
        # `name`. Before this fix none of them bound, index_map came out empty,
        # and every session scored n_binnable=0 -> UNCERTAIN. End-to-end proof:
        # tests/test_tripod_pipeline_join.py.
        bases = ["000000", "000001", "000002"]
        bset = set(bases)
        c8 = (_row_frame_key({"frame_path": "frames/000001.png"}, bset, bases) == "000001"
              and _row_frame_key({"frame_idx": "2"}, bset, bases) == "000002"
              and _row_frame_key({"frame": "000000.png"}, bset, bases) == "000000"
              and _row_frame_key({"nothing": "x"}, bset, bases) is None)
        print(f"[self-test] pi_capture schema keys (frame_path / frame_idx / legacy "
              f"frame) all bind : {'OK' if c8 else 'FAIL'}")
        ok = ok and c8
        # (vi-b) A ROW THAT NAMES AN ABSENT FRAME IS UNRESOLVABLE, never
        # re-bound positionally (2026-07-26). Positional binding survives only
        # for a legacy row with NO filename column, which is why the third
        # assertion uses non-%06d frame names.
        legacy = ["shot_a", "shot_b", "shot_c"]
        c8b = (_row_frame_key({"frame_path": "frames/000009.png"}, bset, bases) is None
               and _row_frame_key({"frame_path": "frames/000009.png",
                                   "frame_idx": "1"}, bset, bases) is None
               and _row_frame_key({"idx": "1"}, set(legacy), legacy) == "shot_b")
        print(f"[self-test] a row naming an ABSENT frame resolves to None (no "
              f"positional re-bind); a legacy no-filename row still binds by "
              f"position : {'OK' if c8b else 'FAIL'}")
        ok = ok and c8b
        # pi_capture's tags.csv schema (n_tags / range_m / center_u|v) reads as a
        # decode; multi-row frames collapse to the NEAREST tag.
        pc_tags = {"000000": {"frame_idx": "0", "frame_path": "frames/000000.png",
                              "n_tags": "1", "tag_id": "7", "center_u": "640.0",
                              "center_v": "400.0", "range_m": "5.2500"}}
        dec_pc = decode_frames([("/x/000000.png", "000000")],
                               (600.0, 600.0, 640.0, 400.0, 1280, 800, np.zeros(5)),
                               0.3, tags_map=pc_tags, quiet=True)["000000"]
        merged = _merge_tag_rows({"range_m": "9.0"}, {"range_m": "4.0"})
        c9 = (dec_pc[0] is True and abs(dec_pc[1] - 5.25) < 1e-6
              and dec_pc[2] == 640.0 and dec_pc[3] == 400.0
              and _tag_row_range(merged) == 4.0)
        print(f"[self-test] pi_capture tags.csv schema decodes ({dec_pc}) + nearest-tag "
              f"merge : {'OK' if c9 else 'FAIL'}")
        ok = ok and c9
        # range_truth_join quality flags are honoured: an `extrapolated` frame is
        # NEVER binnable even if a stale range sits in the row.
        imap = {"f1": {"true_range_m": "12.0", "range_quality": "ok"},
                "f2": {"true_range_m": "12.0", "range_quality": "extrapolated"},
                "f3": {"true_range_m": "12.0", "range_quality": "gap"}}
        c10 = (truth_range_for("f1", imap) == 12.0
               and truth_range_for("f2", imap) is None
               and truth_range_for("f3", imap) == 12.0
               and truth_range_for("f3", imap, accept_quality={"ok"}) is None)
        print(f"[self-test] range_quality gating (extrapolated dropped; "
              f"--require-quality ok strict) : {'OK' if c10 else 'FAIL'}")
        ok = ok and c10

    print(f"[self-test] {'PASS' if ok else 'FAIL'}")
    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir", nargs="?", help="capture-session dir (frames/ + index.csv + meta.json [+ tags.csv])")
    ap.add_argument("--calib", help="camera intrinsics JSON (calibrate_camera.py output)")
    ap.add_argument("--weights", default=DEFAULT_WEIGHTS,
                    help="PRIMARY markerless NN .onnx for curve (b). Default = the "
                         "real-data n-mono model (AP50 0.442 / recall 44.2%% on real "
                         "held-out imagery, n=4175). NN half no-ops if absent/unrunnable.")
    ap.add_argument("--weights-bar", default=DEFAULT_WEIGHTS_BAR,
                    help="HISTORICAL bar model, scored on the SAME frames for contrast "
                         "(default = the sim-trained drone_finetuned_quad_v2, which is "
                         "measured BLIND on real imagery: AP50 0.0003 / recall 1.1%%). "
                         "Skipped automatically if it resolves to --weights.")
    ap.add_argument("--no-weights-bar", action="store_true",
                    help="score curve (b) with the PRIMARY model only (single-model, "
                         "pre-2026-07-24 behaviour)")
    ap.add_argument("--nn-conf", type=float, default=DEFAULT_NN_CONF,
                    help=f"NN confidence threshold (default {DEFAULT_NN_CONF}, ADR-0058 deployed)")
    ap.add_argument("--tag-size", type=float, default=None,
                    help=f"AprilTag black-square edge (m); default meta.json/{DEFAULT_TAG_SIZE_M} (BOM §E)")
    ap.add_argument("--drone-size", type=float, default=None,
                    help=f"target extent (m) for the NN truth box; default meta.json/{DEFAULT_DRONE_SIZE_M}")
    ap.add_argument("--closing-speed", type=float, default=None,
                    help=f"conservative closing speed m/s for the GATE; default meta.json/"
                         f"{DEFAULT_V_CLOSING_MPS} (NEXT.md R5)")
    ap.add_argument("--closing-speed-hi", type=float, default=DEFAULT_V_CLOSING_HI_MPS,
                    help=f"aggressive closing speed m/s (context only, default {DEFAULT_V_CLOSING_HI_MPS}, protocol §8.1)")
    ap.add_argument("--stream-fps", type=float, default=None,
                    help="MEASURED capture/decode stream fps for the streak burn. "
                         "REQUIRED unless the session's meta.json carries a measured "
                         "stream_fps -- there is deliberately NO default: the verdict "
                         f"flips across ~24 fps at the predicted R_decode90 (the "
                         f"{PI5_APRILTAG_FPS_PAPER_ANCHOR:.0f} fps in protocol §7.3 is a "
                         "PAPER anchor to be bench-measured, and the flight loop runs "
                         "20 Hz). Without it the gate returns UNCERTAIN.")
    ap.add_argument("--quad-decimate", type=float, default=None,
                    help=f"AprilTag detector quad_decimate for the OFFLINE decode. "
                         f"Default: the value the session's meta.json recorded, "
                         f"else {DEFAULT_QUAD_DECIMATE} (the pupil-apriltags LIBRARY "
                         f"default, stated explicitly). 1.0 = full resolution, "
                         f"~1.5x range and a +-48-56 deg incidence cone vs +-32 deg "
                         f"(ADR-0082 / docs/placard_mount.md §3.3) -- protocol "
                         f"§4.2b's reclaim lever. A value that DISAGREES with the "
                         f"session's stamp is REFUSED unless "
                         f"--allow-decimate-mismatch.")
    ap.add_argument("--allow-decimate-mismatch", action="store_true",
                    help="score even when the decode's quad_decimate differs from "
                         "the one the session recorded (default: REFUSE). This is "
                         "the DELIBERATE protocol §4.2b qd in {2.0, 1.0} reclaim "
                         "comparison; the mismatch is stamped into gate.json and "
                         "verdict.txt, and --stream-fps must then be the §7.3 Pi 5 "
                         "bench AT THE SCORED DECIMATE.")
    ap.add_argument("--incidence-bin-deg", type=float,
                    default=DEFAULT_INCIDENCE_BIN_DEG,
                    help=f"incidence bin width for the curve-(a) x incidence table "
                         f"(default {DEFAULT_INCIDENCE_BIN_DEG:.0f} deg = the "
                         f"placard mount's INDEX step, docs/placard_mount.md §3.6)")
    ap.add_argument("--mount-index-deg", type=float, default=None,
                    help="the placard mount's index angle for this pass (0 = beam / "
                         "90 = nose-on). Protocol §4.2b requires it on every pass "
                         "card; recorded into gate.json so a frame's incidence can "
                         "be reconstructed after the day. Falls back to meta.json "
                         "`mount_index_deg`.")
    ap.add_argument("--engagement-incidence-deg", type=float, default=0.0,
                    help="theta_eng: the WORST-CASE tag incidence the real "
                         "engagement will present. The gate is then stated in the "
                         "incidence-aware form t_go = (R_decode90(0)*cos theta_eng "
                         "- R_streak_burn)/V (docs/placard_mount.md §11 item 10). "
                         "Default 0 = the dead-on gate; a GO at 0 deg is NOT a GO "
                         "at 30 deg, and verdict.txt says so.")
    ap.add_argument("--require-quality", choices=("any", "ok"), default="any",
                    help="which range_truth_join `range_quality` flags may be binned: "
                         "'any' (default: ok+gap+low_fix; extrapolated/no_frame_time are "
                         "ALWAYS dropped) or 'ok' (strict)")
    ap.add_argument("--redecode", action="store_true",
                    help="ignore the session's tags.csv and re-decode the frames "
                         "offline (what protocol §6 actually asks for -- do not rely "
                         "on pi_capture's capture-time decode)")
    ap.add_argument("--handoff-streak", type=int, default=DEFAULT_HANDOFF_STREAK,
                    help=f"consecutive detections to form a handoff (default {DEFAULT_HANDOFF_STREAK}, NEXT.md R5)")
    ap.add_argument("--tgo-min", type=float, default=DEFAULT_TGO_MIN_S,
                    help=f"minimum post-handoff t_go for GO (default {DEFAULT_TGO_MIN_S} s, NEXT.md R5)")
    ap.add_argument("--range-bin", type=float, default=2.0, help="range bin size m (default 2, approach_recall convention)")
    ap.add_argument("--max-range", type=float, default=40.0, help="max range m to score (default 40)")
    ap.add_argument("--pos-bands", type=int, default=3, help="position-in-frame bands for curve (b) (default 3: top/mid/bottom)")
    ap.add_argument("--min-decoded", type=int, default=DEFAULT_MIN_DECODED,
                    help=f"data-presence floor for the verdict (default {DEFAULT_MIN_DECODED}; not a statistical bar)")
    ap.add_argument("--min-bin-n", type=int, default=DEFAULT_MIN_BIN_N,
                    help=f"per-BIN sample floor for the R_decode90 walk (default "
                         f"{DEFAULT_MIN_BIN_N}): a bin with fewer binnable frames may "
                         f"not extend the >=90%% envelope, and stopping there returns "
                         f"UNCERTAIN (the band is bounded by missing data, not by a "
                         f"measured decode failure)")
    ap.add_argument("--allow-partial-tags", action="store_true",
                    help="score even when the reused tags.csv does not cover every "
                         "frame (default: REFUSE -- uncovered frames would count as "
                         "decode FAILURES and silently deflate curve (a))")
    ap.add_argument("--allow-orphan-rows", action="store_true",
                    help="score even when index.csv/tags.csv rows name frames "
                         "that are not in frames/ (default: REFUSE -- the "
                         "session's CSVs and its frames describe different "
                         "populations; the dropped count is stamped into "
                         "gate.json either way)")
    ap.add_argument("--gate", action="store_true",
                    help="exit 0=PASS / 1=FAIL / 2=UNCERTAIN so a wrapper can read "
                         "the verdict from the exit code (default: always 0)")
    ap.add_argument("--out-dir", default="logs/tripod_score", help="output dir (default logs/tripod_score)")
    ap.add_argument("--tag", default=None, help="output subdir name (default: session dir basename)")
    ap.add_argument("--self-test", action="store_true", help="synthetic session, no hardware; exit 0/1")
    args = ap.parse_args()

    if args.self_test:
        return 0 if self_test() else 1
    if not (args.session_dir and args.calib):
        ap.error("session_dir and --calib are required (or --self-test)")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
