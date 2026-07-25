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
import csv
import glob
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

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
# timestamps) and are refused as a gate input.
FPS_SOURCE_REJECT_MARK = "replay"
# BOM default placard: tag36h11 printed AS LARGE as the quad carries, ~0.25-0.35 m
# (hardware_order_list.md §E line 268). 0.3 m default; a real session overrides
# via meta.json / --tag-size. This is the AprilTag black-square edge that
# pupil-apriltags' `tag_size` measures.
DEFAULT_TAG_SIZE_M = 0.3
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
    (3) frame_idx as a POSITION in capture order. Returns None if unresolvable."""
    fn = ""
    for c in _FRAME_NAME_COLS:
        v = (row.get(c) or "").strip()
        if v:
            fn = v
            break
    key = os.path.splitext(os.path.basename(fn))[0] if fn else None
    if key is not None and key in base_set:
        return key
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
    return key  # may be a name with no matching frame file (kept, harmless)


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


def load_session(session_dir, ignore_tags_csv=False):
    """Return (frames, index_map, meta, tags_map). `frames` is a sorted list of
    (path, basename); `index_map`/`tags_map` are {basename: row} or None."""
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

    def _keymap(rows, merge=None):
        if rows is None:
            return None
        out = {}
        for r in rows:
            key = _row_frame_key(r, base_set, bases)
            if not key:
                continue
            if key in out and merge is not None:
                out[key] = merge(out[key], r)
            else:
                out[key] = r
        return out

    index_map = _keymap(_read_csv(os.path.join(session_dir, "index.csv")))
    tags_map = None if ignore_tags_csv else _keymap(
        _read_csv(os.path.join(session_dir, "tags.csv")), merge=_merge_tag_rows)
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
def decode_frames(frames, calib, tag_size_m, tags_map=None, quiet=False):
    """Per-frame AprilTag decode -> {basename: (decoded, tag_range_m, u, v, n)}.
    Reuses a provided tags.csv (tags_map) verbatim; otherwise decodes the frames
    with pupil-apriltags (undistorting first, like autolabel_from_apriltag)."""
    fx, fy, cx, cy, w, h, dist = calib
    results = {}
    if tags_map is not None:
        for _, base in frames:
            row = tags_map.get(base)
            if row is None:
                results[base] = (False, None, None, None, 0)
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
            results[base] = (dec, _tag_row_range(row), u, v, n_tags)
        if not quiet:
            n_dec = sum(1 for r in results.values() if r[0])
            print(f"[tripod] curve(a): reused provided tags.csv "
                  f"({n_dec}/{len(frames)} frames decoded)")
        return results

    import cv2
    try:
        from pupil_apriltags import Detector
    except ImportError:  # aarch64: pyapriltags drop-in (docs/pi_emulation_check.md)
        from pyapriltags import Detector
    undistort = bool(np.asarray(dist).any()) and w and h
    map1 = map2 = None
    if undistort:
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)
        map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, K, (w, h), cv2.CV_16SC2)
    det = Detector(families="tag36h11")
    for p, base in frames:
        img = cv2.imread(p)
        if img is None:
            results[base] = (False, None, None, None, 0)
            continue
        if undistort:
            img = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        dets = det.detect(gray, estimate_tag_pose=True,
                          camera_params=(fx, fy, cx, cy), tag_size=tag_size_m)
        if not dets:
            results[base] = (False, None, None, None, 0)
            continue
        d = min(dets, key=lambda t: float(t.pose_t.reshape(3)[2]))  # nearest
        rng = float(np.linalg.norm(d.pose_t.reshape(3)))
        u, v = float(d.center[0]), float(d.center[1])
        results[base] = (True, rng, u, v, len(dets))
    if not quiet:
        n_dec = sum(1 for r in results.values() if r[0])
        print(f"[tripod] curve(a): decoded {n_dec}/{len(frames)} frames "
              f"(pupil-apriltags, tag_size={tag_size_m:.3f} m)")
    return results


# --------------------------------------------------------------------------
# Curve (a): decode envelope
# --------------------------------------------------------------------------
def curve_a(frames, decode, index_map, bin_m, max_m, accept_quality=None):
    """Bin decode success by TRUE range. Returns (bins, R_decode_any,
    R_decode90, accuracy_rows, n_binnable, n_unbinnable).

      bins        {range_lo: [n_decoded, n_total, [decoded_ranges]]}
      R_decode_any  farthest true-range of ANY successful decode (m)
      R_decode90    farthest range where the decode rate SUSTAINS >=90% inward
                    (protocol §7.1) -- the outer edge of the contiguous
                    >=90% region starting from the nearest bin (0.0 if even the
                    nearest bin is <90%).
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
        dec, drange, _u, _v, _n = decode[base]
        b = int(gr // bin_m) * bin_m
        cell = bins.setdefault(b, [0, 0, []])
        cell[1] += 1
        if dec:
            cell[0] += 1
            r_any = max(r_any, gr)
            if drange is not None:
                cell[2].append(drange)
                acc.append((gr, drange))

    # R_decode90: walk bins from nearest outward while rate >= 0.9.
    r90 = 0.0
    for lo in sorted(bins):
        n_dec, n_tot, _ = bins[lo]
        if n_tot and (n_dec / n_tot) >= 0.9:
            r90 = lo + bin_m
        else:
            break
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
                 have_truth=True):
    """Apply the curve-(a) money gate (protocol §8.1 / NEXT.md R5):

        R_streak_burn = (E[T] / stream_fps) × V_closing   [run-length, ADR-0079]
        E[T]          = (1 - p^k) / (p^k * (1 - p)),  p=decode_rate, k=streak_n
        t_go          = (R_decode90 − R_streak_burn) / V_closing

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
    """
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
    t_go = ((r90 - r_streak_burn) / v_closing
            if (fps_ok and v_closing > 0) else float("-inf"))

    if not have_truth:
        verdict, reason = "UNCERTAIN", "no TRUE ranges in the session (index.csv true_range_m / filename _r_) -- cannot bin curve (a). Run scripts/seeker/range_truth_join.py first (target log + surveyed tripod position -> true_range_m)."
    elif n_decoded is not None and n_decoded < min_decoded:
        verdict, reason = "UNCERTAIN", (f"only {n_decoded} decoded frame(s) < data floor {min_decoded} -- too thin to decide")
    elif r90 <= 0:
        # No bin sustained >=90%: the tag never forms a handoff-quality stream.
        # This leg is fps-INDEPENDENT, so it is decided before the fps check.
        verdict = "FAIL" if r_any > 0 else "UNCERTAIN"
        reason = ("no range bin sustains >=90% decode (R_decode90=0) -- "
                  + ("sparse decodes exist but no clean handoff band"
                     if r_any > 0 else "the tag never decoded"))
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
            f"gate: -> {verdict} ({'fps-independent' if r90 <= 0 else 'needs the fps measurement'})"
        ) if math.isfinite(e_frames) else (
            f"decode_rate p = {decode_rate:.3f} -> streak never forms -> {verdict}")
        return {
            "verdict": verdict, "reason": reason,
            "R_decode90_m": round(r90, 3), "R_decode_any_m": round(r_any, 3),
            "decode_rate": round(decode_rate, 4), "decode_Hz": None,
            "streak_burn_model": "run-length (ADR-0079)",
            "E_streak_frames": round(e_frames, 3) if math.isfinite(e_frames) else None,
            "R_streak_burn_m": None, "R_streak_burn_meanrate_m": None,
            "t_go_s": None,
            "V_closing_mps": v_closing, "tgo_min_s": tgo_min,
            "handoff_streak": streak_n, "stream_fps": None,
            "arithmetic": arithmetic,
        }

    arithmetic = (
        f"decode_rate p = {decode_rate:.3f}  (streak k = {streak_n})\n"
        f"E[T] run-length = (1 − p^k)/(p^k·(1−p)) = {e_frames:.2f} frames "
        f"(mean-rate {streak_n}/p = {streak_n / decode_rate:.2f} frames)\n"
        f"R_streak_burn = ({e_frames:.2f} / {stream_fps:.1f} fps) × {v_closing:.1f} m/s = "
        f"{r_streak_burn:.2f} m   (mean-rate burn {r_burn_meanrate:.2f} m)\n"
        f"t_go = (R_decode90 {r90:.2f} m − R_streak_burn {r_streak_burn:.2f} m) / "
        f"{v_closing:.1f} m/s = {t_go:.3f} s\n"
        f"gate: t_go {t_go:.3f} s {'>=' if t_go >= tgo_min else '<'} {tgo_min:.2f} s -> {verdict}"
    ) if decode_rate > 0 else (
        f"decode_rate p = 0 -> streak never forms (R_streak_burn = inf) -> {verdict}"
    )
    return {
        "verdict": verdict, "reason": reason,
        "R_decode90_m": round(r90, 3), "R_decode_any_m": round(r_any, 3),
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

    # Reuse resolution_probe's verified box-hit test (centre-in-gt + size-match).
    try:
        from resolution_probe import box_hits_gt
    except ImportError:
        def box_hits_gt(boxes, gcx, gcy, gw, gh, tol=15):
            for _s, u, v, bw, bh in boxes:
                if (abs(u - gcx) <= gw / 2 + tol and abs(v - gcy) <= gh / 2 + tol
                        and gw / 3 <= bw <= 3 * gw and gh / 3 <= bh <= 3 * gh):
                    return True
            return False

    import cv2
    bins = {}
    n_scored = 0
    for p, base in frames:
        dec, drange, _u, _v, _n = decode[base]
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
        x = (u0 - cx) / fx * gr
        y = (v0 - cy) / fy * gr
        z = gr / math.sqrt(1.0 + ((u0 - cx) / fx) ** 2 + ((v0 - cy) / fy) ** 2)
        gt = project_to_bbox(x, y, z, gr, fx, fy, cx, cy, drone_size_m, w or 1280, h or 960)
        if gt is None:
            continue
        gcx, gcy, gw, gh = gt
        frame = cv2.imread(p)
        if frame is None:
            continue
        hit = box_hits_gt(seeker._infer_boxes(frame), gcx, gcy, gw, gh)
        band = _pos_band(gcy, h or frame.shape[0], n_bands)
        key = (int(gr // bin_m) * bin_m, band)
        cell = bins.setdefault(key, [0, 0])
        cell[0] += int(hit)
        cell[1] += 1
        n_scored += 1
    note = (f"NN half: scored {n_scored} tag-truthed frames "
            f"(recall bounded by curve (a)'s decode ceiling -- protocol §7.2)")
    return bins, note


def bins_overall(bins):
    """(hits, total, recall) rolled up over all range x position cells."""
    if not bins:
        return 0, 0, None
    h = sum(c[0] for c in bins.values())
    t = sum(c[1] for c in bins.values())
    return h, t, (h / t if t else None)


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
        wr.writerow(["frame", "decoded", "tag_range_m", "u_px", "v_px", "n_tags"])
        for _, base in frames:
            dec, dr, u, v, n = decode[base]
            wr.writerow([base + ".png", int(dec),
                         round(dr, 4) if dr is not None else "",
                         round(u, 2) if u is not None else "",
                         round(v, 2) if v is not None else "", n])


SMALL_N_CAVEAT = (
    "SMALL-N CAVEAT (protocol §4.6): a field afternoon cannot buy CLAUDE.md's "
    "n>=8 paired-seed statistics -- treat curve (a)/(b) as a first honest read, "
    "not a statistically tight one."
)


def build_summary(gate, gate_hi, curve_a_bins, curve_b_note, meta, n_binnable,
                  n_unbinnable, bin_m, fps_source=None, quality=None,
                  range_truth=None):
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
    L.append("")
    L.append("--- CURVE (a): AprilTag decode envelope ---")
    L.append(f"  {'range(m)':<10}{'decode%':>9}{'n_dec':>8}{'n_tot':>8}")
    for lo in sorted(curve_a_bins):
        n_dec, n_tot, _ = curve_a_bins[lo]
        rate = 100 * n_dec / n_tot if n_tot else 0
        L.append(f"  {f'{lo:.0f}-{lo+bin_m:.0f}':<10}{rate:>8.0f}%{n_dec:>8}{n_tot:>8}")
    L.append(f"  R_decode_any = {gate['R_decode_any_m']:.2f} m (farthest ANY decode)")
    L.append(f"  R_decode90   = {gate['R_decode90_m']:.2f} m (farthest sustained >=90% inward)")
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
    L.append("")
    L.append("--- CURVE (b): NN approach recall (gates ONLY the Hailo/markerless phase) ---")
    for line in str(curve_b_note).splitlines() or [""]:
        L.append(f"  {line}")
    L.append("")
    L.append(SMALL_N_CAVEAT)
    return "\n".join(L)


# --------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------
def resolve_stream_fps(args, meta):
    """(stream_fps or None, human-readable source). NO DEFAULT -- see the module
    docstring: the burn scales as 1/fps and the verdict flips across ~24 fps at
    the predicted R_decode90, so an invented rate would decide ~$740.

    Accepted, in order: (1) an explicit --stream-fps (the one-line override for a
    bench-measured number, protocol §7.3); (2) meta.json `stream_fps` -- UNLESS
    `stream_fps_source` says it came from pi_capture's dir-REPLAY backend, whose
    timestamps are synthetic and therefore not a capture-rate measurement."""
    if getattr(args, "stream_fps", None) is not None:
        return float(args.stream_fps), "--stream-fps (explicit; bench-measured, protocol §7.3)"
    fps = _pf(meta.get("stream_fps"))
    src = str(meta.get("stream_fps_source") or "").strip()
    if fps is not None and fps > 0:
        if FPS_SOURCE_REJECT_MARK in src.lower():
            return None, (f"REJECTED meta.json stream_fps={fps} — stream_fps_source="
                          f"'{src}' is not a capture-rate measurement")
        return fps, f"meta.json stream_fps ({src or 'source unspecified'})"
    return None, ("NOT SUPPLIED — no --stream-fps and no meta.json stream_fps "
                  "(pi_capture records it from real capture timestamps; bench it "
                  "per protocol §7.3)")


def run(args):
    frames, index_map, meta, tags_map = load_session(
        args.session_dir, ignore_tags_csv=getattr(args, "redecode", False))
    calib = load_calib(args.calib)

    tag_size = args.tag_size if args.tag_size is not None else \
        _pf(meta.get("tag_size_m")) or DEFAULT_TAG_SIZE_M
    stream_fps, fps_source = resolve_stream_fps(args, meta)
    v_close = args.closing_speed if args.closing_speed is not None else \
        _pf(meta.get("closing_speed_mps")) or DEFAULT_V_CLOSING_MPS
    drone_size = args.drone_size if args.drone_size is not None else \
        _pf(meta.get("drone_size_m")) or DEFAULT_DRONE_SIZE_M
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
    decode = decode_frames(frames, calib, tag_size, tags_map=tags_map)
    if tags_map is None:
        write_tags_csv(os.path.join(outp, "tags.csv"), frames, decode)
    bins, r_any, r90, acc, n_binnable, n_unbinnable = curve_a(
        frames, decode, index_map, args.range_bin, args.max_range,
        accept_quality=accept_quality)
    n_decoded = sum(1 for _, b in frames if decode[b][0])
    have_truth = n_binnable > 0
    quality = quality_summary(frames, index_map)

    d_rate = decode_rate_near(bins, r90, args.range_bin)
    gate = gate_verdict(r90, r_any, d_rate, stream_fps, args.handoff_streak,
                        v_close, args.tgo_min, n_decoded=n_decoded,
                        min_decoded=args.min_decoded, have_truth=have_truth)
    gate_hi = gate_verdict(r90, r_any, d_rate, stream_fps, args.handoff_streak,
                           args.closing_speed_hi, args.tgo_min,
                           n_decoded=n_decoded, min_decoded=args.min_decoded,
                           have_truth=have_truth)

    write_curve_a_csv(os.path.join(outp, "curve_a_decode.csv"), bins, args.range_bin)
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

    summary = build_summary(gate, gate_hi, bins, b_note, meta, n_binnable,
                            n_unbinnable, args.range_bin, fps_source=fps_source,
                            quality=quality, range_truth=range_truth)
    with open(os.path.join(outp, "verdict.txt"), "w") as f:
        f.write(summary + "\n")
    with open(os.path.join(outp, "gate.json"), "w") as f:
        json.dump({"conservative": gate, "aggressive": gate_hi,
                   "tag_size_m": tag_size, "drone_size_m": drone_size,
                   "n_decoded": n_decoded, "n_binnable": n_binnable,
                   "stream_fps": stream_fps, "stream_fps_source": fps_source,
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
    near = [4.0, 5.0, 6.0, 7.0, 8.0]      # 42-84 px -> decode ~100%
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
        json.dump(dict(tag_size_m=tag_size, drone_size_m=DEFAULT_DRONE_SIZE_M,
                       stream_fps=PI5_APRILTAG_FPS_PAPER_ANCHOR,
                       stream_fps_source="self-test stand-in (protocol §7.3 paper anchor)",
                       aspect="approach"), f)
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
        # session, and R_decode90 landed at the near/far boundary (~8 m).
        d_rate = decode_rate_near(bins, r90, 2.0)
        g5 = gate_verdict(r90, r_any, d_rate, PI5_APRILTAG_FPS_PAPER_ANCHOR,
                          DEFAULT_HANDOFF_STREAK, DEFAULT_V_CLOSING_MPS,
                          DEFAULT_TGO_MIN_S, n_decoded=sum(1 for _, b in frames if decode[b][0]))
        ref = gate_verdict(r90, r_any, d_rate, PI5_APRILTAG_FPS_PAPER_ANCHOR,
                           DEFAULT_HANDOFF_STREAK, DEFAULT_V_CLOSING_MPS,
                           DEFAULT_TGO_MIN_S, n_decoded=999)
        c5 = (r90 >= 6.0 and abs(g5["t_go_s"] - ref["t_go_s"]) < 1e-9 and
              g5["verdict"] in ("PASS", "UNCERTAIN"))
        print(f"[self-test] R_decode90={r90:.1f} m R_decode_any={r_any:.1f} m "
              f"t_go={g5['t_go_s']} s verdict={g5['verdict']} : {'OK' if c5 else 'FAIL'}")
        ok = ok and c5

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
        want = ["curve_a_decode.csv", "curve_a.png", "verdict.txt", "gate.json"]
        made = [os.path.exists(os.path.join(td, "out", "synthA", w)) for w in want]
        c7 = all(made)
        print(f"[self-test] run() artifacts {dict(zip(want, made))} : {'OK' if c7 else 'FAIL'}")
        ok = ok and c7

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
