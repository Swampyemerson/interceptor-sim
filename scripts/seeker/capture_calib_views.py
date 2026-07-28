#!/usr/bin/env python3
"""Headless, SSH-friendly chessboard-view capture for skr-06 (camera intrinsics).

WHY THIS EXISTS. The builder is at the bench with a HEADLESS Raspberry Pi 5 +
innomaker OV9281 (mono global shutter, 1280x800, libcamera/picamera2), reached
over ssh with NO display. build_tab step `skr-06` needs >=15 chessboard views in
the final tripod geometry, acceptance RMS reprojection <= 1.0 px -- it is the
FIRST bench gate, and every range and bearing number in the campaign is wrong
until it runs.

`scripts/calibrate_camera.py --live` cannot do this job: it opens the camera with
cv2.VideoCapture (V4L2 -- the OV9281 is on the libcamera stack) and shows the
board in a cv2.imshow window (there is no display). So the live half of that tool
is unusable here, while its OFFLINE half (`--images GLOB`) is exactly what we
want. This script is the missing front end: it captures views over ssh and tells
you AFTER EVERY SHOT whether the board was found and WHERE IN THE FRAME it
landed, so you do not discover at the end of the session that 12 of 20 views were
unusable or that all 20 were clustered in the middle of the frame.

It deliberately does NOT calibrate. It writes PNGs that `calibrate_camera.py
--images` consumes, and it uses that script's OWN `_find_corners()` for the
found/not-found call -- the same detector, the same flags, the same sub-pixel
refinement. A view this tool calls FOUND is a view calibrate_camera also accepts;
there is no second corner finder to drift.

--------------------------------------------------------------------------
WHY A LAPTOP SCREEN IS A VALID CALIBRATION TARGET (and why --square is not
critical for intrinsics). The intrinsics -- fx, fy, cx, cy and the distortion
coefficients -- are INVARIANT to the scale of the object points. Scaling every
square size by k multiplies each object point by k, which cv2.calibrateCamera
absorbs entirely into the EXTRINSIC translation of each view (t -> k*t); the
projection u = fx*X/Z + cx is unchanged because X and Z scale together. Only the
recovered board DISTANCES change. So a chessboard displayed on a flat, glare-free
laptop/tablet SCREEN -- where the exact printed square size is awkward to know --
still gives correct intrinsics, as long as you enter *a* consistent --square. Get
the number right anyway if you can (it makes the reported extrinsics meaningful,
and it costs nothing), but do not let an unknown square size stop the session.
What DOES matter is that the board is rigidly FLAT (a curled printout is the
classic source of a fake distortion estimate) and that the views cover the frame,
especially the CORNERS, where the wide lens distorts most.
--------------------------------------------------------------------------

WHAT IT PRINTS AFTER EACH SHOT
  - FOUND     -> saves calib_NNN.png and reports which cell of a 3x3 zone grid
                 the board's centroid fell in, how big the board is as a
                 percentage of frame area (NEAR / MID / FAR), and running counts.
  - NOT FOUND -> saves nothing, and names the two things that are almost always
                 the cause (board not fully in frame / too dark or blurred).

AND ON EXIT, an honest verdict: total views, the zone coverage map with the EMPTY
cells called out by name, the near/mid/far mix, and READY or NOT READY. READY
requires >=15 saved views AND >=7 of the 9 zones non-empty AND all three distance
buckets non-empty. Exit 0 for READY, 1 for NOT READY. It will never print READY
off zero views (no-vacuous-verdicts, CLAUDE.md / docs/error_handling_policy.md).

Usage:
    # on the Pi, over ssh (interactive)
    scripts/seeker/capture_calib_views.py --out ~/calib_views
    # replay/classify an existing folder of stills -- no camera needed
    scripts/seeker/capture_calib_views.py --out /tmp/v --source dir=PATH --auto 40
    # offline proof the whole path works (no camera, no display), exits 0/1
    scripts/seeker/capture_calib_views.py --self-test

Requires cv2 + numpy. On the Pi run it under the venv that carries picamera2.
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

# --- imports from sibling/parent scripts (reuse, do not reimplement) -------
_HERE = os.path.dirname(os.path.abspath(__file__))          # .../scripts/seeker
_SCRIPTS = os.path.dirname(_HERE)                           # .../scripts
for _p in (_HERE, _SCRIPTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# The picamera2/libcamera backend. It imports picamera2 LAZILY and raises a clear
# SystemExit off-Pi, which is exactly the behaviour we want here too.
from pi_capture import EXPOSURE_SPEC_US, iter_dir, iter_picamera2  # noqa: E402
# THE corner finder -- calibrate_camera.py's own, not a copy. If this tool says
# FOUND, `calibrate_camera.py --images` will also accept the view.
from calibrate_camera import _find_corners  # noqa: E402

LOG = "[calib-views]"

# OV9281 native geometry (docs/project_state.json hardware node: innomaker
# OV9281, 1280x800 mono global-shutter).
DEF_WIDTH, DEF_HEIGHT = 1280, 800

# A 10x7-SQUARE board has 9x6 INNER corners -- inner corners are what OpenCV
# counts, and the #1 way a calibration session produces zero usable views is
# passing the square count here.
DEF_COLS, DEF_ROWS = 9, 6

# CALIBRATION-ONLY EXPOSURE DEVIATION -- read this before copying the number.
# The project's exposure SPEC is <=1 ms (pi_capture.EXPOSURE_SPEC_US, protocol
# §3.3/§4.4): a short exposure on a global shutter is the whole reason this
# sensor was chosen, because it is what stops real motion blur at >=9 m/s from
# eating the tag decode range and NN recall. That spec is about a target moving
# across the frame. The calibration chessboard is STATIC on a desk or tripod, so
# there is no motion to blur -- while 1 ms indoors gives a near-black frame in
# which findChessboardCorners simply fails, which would burn the bench session
# hunting a "bad board" that is really an underexposed one. So the default here
# is 8x the spec ON PURPOSE. The run header states this out loud so that nobody
# later mistakes a calibration session's meta/exposure for a motion-blur
# measurement. If the room is bright, drop it; if the frames are still dark,
# raise --gain before --exposure-us (gain costs noise, exposure costs nothing
# here).
DEF_EXPOSURE_US = 8000
DEF_GAIN = 2.0

# How many queued frames to throw away before the one we keep. picamera2 keeps a
# small queue of completed requests behind the consumer (video configurations
# allocate on the order of 4-6 buffers), so the FIRST frame the generator hands
# back after the operator has spent several seconds repositioning the board shows
# where the board WAS, not where it is now. A stale frame here is worse than no
# frame: it is found, saved, and filed into the WRONG zone, which corrupts the
# coverage map that is this tool's entire reason to exist. Draining costs one
# frame period each (~17 ms at 60 fps) and is invisible to the operator.
DRAIN_FRAMES = 5

# 3x3 zone grid, row-major from the top-left of the image.
ZONE_NAMES = ("top-left", "top-centre", "top-right",
              "middle-left", "middle-centre", "middle-right",
              "bottom-left", "bottom-centre", "bottom-right")
# The four zones that matter most: a wide lens's distortion is largest at the
# frame corners, and k1/k2 are estimated almost entirely from views that put
# corners THERE. A set with empty corner zones fits distortion from the flat
# middle of the field and extrapolates -- which is how you get a calibration with
# a beautiful RMS that is wrong exactly where the target appears at handoff.
CORNER_ZONES = (0, 2, 6, 8)

# Board bounding-box area as a percentage of frame area. Distance diversity is
# what separates focal length from board distance in the solve -- an all-NEAR set
# leaves fx/fy poorly conditioned.
NEAR_MIN_PCT = 25.0     # >= 25%  -> NEAR
MID_MIN_PCT = 8.0       # 8-25%   -> MID ; < 8% -> FAR
BUCKETS = ("NEAR", "MID", "FAR")

# skr-06 acceptance inputs (docs/project_state.json build_tab step skr-06:
# ">=15 checkerboard views ... acceptance RMS reprojection <= 1.0 px").
MIN_VIEWS = 15
MIN_ZONES = 7           # of 9
SAVE_PREFIX = "calib_"


# --------------------------------------------------------------------------
# Classification
# --------------------------------------------------------------------------

def classify(corners, frame_w, frame_h):
    """Where in the frame the board landed, and how big it is.

    `corners` is the (N,1,2) float32 array `_find_corners` returns (the INNER
    corner grid). Returns a dict: centroid, zone index/name, bbox area as a
    percentage of frame area, and the distance bucket."""
    import numpy as np
    pts = np.asarray(corners, dtype=float).reshape(-1, 2)
    cx = float(pts[:, 0].mean())
    cy = float(pts[:, 1].mean())
    x0, y0 = float(pts[:, 0].min()), float(pts[:, 1].min())
    x1, y1 = float(pts[:, 0].max()), float(pts[:, 1].max())
    frame_area = float(frame_w) * float(frame_h)
    area_pct = (100.0 * (x1 - x0) * (y1 - y0) / frame_area) if frame_area > 0 else 0.0
    col = min(2, max(0, int(cx * 3.0 / frame_w))) if frame_w > 0 else 1
    row = min(2, max(0, int(cy * 3.0 / frame_h))) if frame_h > 0 else 1
    zone = row * 3 + col
    if area_pct >= NEAR_MIN_PCT:
        bucket = "NEAR"
    elif area_pct >= MID_MIN_PCT:
        bucket = "MID"
    else:
        bucket = "FAR"
    return {"centroid": (cx, cy), "zone": zone, "zone_name": ZONE_NAMES[zone],
            "bbox": (x0, y0, x1, y1), "area_pct": area_pct, "bucket": bucket}


class Coverage:
    """Running tally of what the session has actually banked."""

    def __init__(self):
        self.zone_counts = [0] * 9
        self.bucket_counts = {b: 0 for b in BUCKETS}
        self.saved = 0
        self.not_found = 0

    def add(self, info):
        self.zone_counts[info["zone"]] += 1
        self.bucket_counts[info["bucket"]] += 1
        self.saved += 1

    @property
    def zones_filled(self):
        return sum(1 for c in self.zone_counts if c > 0)

    def empty_zones(self):
        return [i for i, c in enumerate(self.zone_counts) if c == 0]

    def empty_buckets(self):
        return [b for b in BUCKETS if self.bucket_counts[b] == 0]


def summarize(cov, out_dir, cols, rows):
    """(ready, lines). The verdict is fail-closed: every condition must be met
    on views that were actually SAVED, so a zero-view run can only ever come out
    NOT READY (no-vacuous-verdicts)."""
    L = []
    L.append(f"{LOG} ---------------- session summary ----------------")
    L.append(f"{LOG} saved views: {cov.saved}   (need >= {MIN_VIEWS})"
             + (f"   |  frames rejected (no board): {cov.not_found}"
                if cov.not_found else ""))
    L.append(f"{LOG} zone coverage (views per cell of the frame):")
    L.append(f"{LOG}            LEFT  CENTRE   RIGHT")
    for r, label in enumerate(("TOP   ", "MIDDLE", "BOTTOM")):
        cells = "".join(f"{cov.zone_counts[r * 3 + c]:>8}" for c in range(3))
        L.append(f"{LOG}    {label}{cells}")
    empty_z = cov.empty_zones()
    if empty_z:
        L.append(f"{LOG} EMPTY zones ({len(empty_z)}): "
                 + ", ".join(ZONE_NAMES[i] for i in empty_z))
    else:
        L.append(f"{LOG} EMPTY zones: none -- full frame coverage")
    L.append(f"{LOG} distance mix: "
             + "  ".join(f"{b}={cov.bucket_counts[b]}" for b in BUCKETS)
             + f"   (NEAR >= {NEAR_MIN_PCT:.0f}% of frame area, "
               f"MID {MID_MIN_PCT:.0f}-{NEAR_MIN_PCT:.0f}%, "
               f"FAR < {MID_MIN_PCT:.0f}%)")

    missing = []
    if cov.saved < MIN_VIEWS:
        missing.append(f"{MIN_VIEWS - cov.saved} more view(s) "
                       f"({cov.saved} of {MIN_VIEWS})")
    if cov.zones_filled < MIN_ZONES:
        missing.append(f"{MIN_ZONES - cov.zones_filled} more zone(s) covered "
                       f"({cov.zones_filled} of 9 filled, need {MIN_ZONES})")
    empty_b = cov.empty_buckets()
    if empty_b:
        missing.append("at least one view at each distance -- nothing in "
                       + "/".join(empty_b)
                       + " (move the board "
                       + ("closer" if "NEAR" in empty_b else "further away")
                       + ")")
    empty_corners = [i for i in CORNER_ZONES if cov.zone_counts[i] == 0]

    ready = not missing
    if ready:
        L.append(f"{LOG} VERDICT: READY -- {cov.saved} views, "
                 f"{cov.zones_filled}/9 zones, all three distances covered.")
    else:
        L.append(f"{LOG} VERDICT: NOT READY -- still missing: "
                 + "; ".join(missing))
    if empty_corners:
        # Printed on READY runs too: 7-of-9 zones can technically be met while
        # leaving corners empty, and that is the weakest kind of pass.
        L.append(f"{LOG} WARNING: {len(empty_corners)} of the 4 frame CORNERS "
                 f"have no view ("
                 + ", ".join(ZONE_NAMES[i] for i in empty_corners)
                 + "). Wide-lens distortion is estimated FROM the corners, so "
                   "these are the views that matter most -- get the board fully "
                   "into each corner, tilted, before you stop.")
    L.append(f"{LOG} ------------------------------------------------")
    L.append(f"{LOG} next command (paste this, with YOUR measured square size):")
    L.append("")
    L.append(f'    scripts/calibrate_camera.py --images "{os.path.join(out_dir, SAVE_PREFIX)}*.png" '
             f"--cols {cols} --rows {rows} \\")
    L.append(f"        --square <YOUR_SQUARE_METRES> --out "
             f"{os.path.join(out_dir, 'calib.json')}")
    L.append("")
    L.append(f"{LOG} --square is the size of ONE square in METRES (a 25 mm "
             f"square is 0.025, not 25).")
    if not ready:
        L.append(f"{LOG} (that command will still run on what you have, but a "
                 f"set this thin gives an over-fitted, untrustworthy solve -- "
                 f"skr-06 wants RMS <= 1.0 px on a set that covers the frame.)")
    return ready, L


# --------------------------------------------------------------------------
# Shot handling
# --------------------------------------------------------------------------

def _next_index(out_dir):
    """Continue the numbering of any calib_*.png already in the directory so a
    resumed session never overwrites earlier views."""
    n = -1
    for p in glob.glob(os.path.join(out_dir, SAVE_PREFIX + "*.png")):
        stem = os.path.basename(p)[len(SAVE_PREFIX):-4]
        if stem.isdigit():
            n = max(n, int(stem))
    return n + 1


def handle_frame(cv2, gray, cov, out_dir, cols, rows, idx, save=True,
                 label=None, quiet=False):
    """Run the shared corner finder on one frame, save + classify it if a board
    is there. Returns (info_or_None, saved_path_or_None)."""
    if gray is None:
        if not quiet:
            print(f"{LOG} NOT FOUND -- unreadable frame"
                  + (f" [{label}]" if label else ""))
        cov.not_found += 1
        return None, None
    h, w = gray.shape[:2]
    corners = _find_corners(gray, cols, rows)
    if corners is None:
        cov.not_found += 1
        if not quiet:
            print(f"{LOG} NOT FOUND"
                  + (f" [{label}]" if label else "")
                  + f" -- nothing saved. Two usual causes: (1) the board is not "
                    f"FULLY in frame (all {cols}x{rows} inner corners plus a "
                    f"white margin must be visible), (2) too dark / blurred / "
                    f"glared (raise --gain or --exposure-us, kill the "
                    f"reflection, hold still).")
        return None, None
    info = classify(corners, w, h)
    path = None
    if save:
        path = os.path.join(out_dir, f"{SAVE_PREFIX}{idx:03d}.png")
        if not cv2.imwrite(path, gray):
            raise SystemExit(f"{LOG} FAILED to write {path}")
    cov.add(info)
    if not quiet:
        print(f"{LOG} FOUND -> {os.path.basename(path) if path else '(not saved)'}"
              f"  zone={info['zone_name']}  size={info['area_pct']:.1f}% of frame "
              f"({info['bucket']})  |  saved={cov.saved}  zones={cov.zones_filled}/9"
              f"  NEAR/MID/FAR={cov.bucket_counts['NEAR']}/"
              f"{cov.bucket_counts['MID']}/{cov.bucket_counts['FAR']}")
    return info, path


def preload_existing(cv2, out_dir, cols, rows, cov, quiet=False):
    """Re-classify calib_*.png already sitting in the output directory.

    The verdict must describe the set `calibrate_camera.py --images` will
    actually consume, which is the WHOLE directory -- not just what this process
    happened to grab. Without this, a second session in the same folder reports
    "3 views, NOT READY" while 18 good views sit on disk (or, worse, reports
    coverage it does not have)."""
    paths = sorted(glob.glob(os.path.join(out_dir, SAVE_PREFIX + "*.png")))
    if not paths:
        return 0
    if not quiet:
        print(f"{LOG} found {len(paths)} existing view(s) in {out_dir} -- "
              f"re-checking them so the verdict covers everything "
              f"calibrate_camera.py will read...")
    n = 0
    for p in paths:
        img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        info, _ = handle_frame(cv2, img, cov, out_dir, cols, rows, 0,
                               save=False, label=os.path.basename(p), quiet=True)
        if info is not None:
            n += 1
    if not quiet:
        print(f"{LOG}   ...{n} of {len(paths)} still detect a board "
              f"({cov.zones_filled}/9 zones so far)")
    return n


# --------------------------------------------------------------------------
# Modes
# --------------------------------------------------------------------------

def _header(args, interactive):
    print(f"{LOG} headless chessboard capture for skr-06 (camera intrinsics)")
    print(f"{LOG} board: {args.cols}x{args.rows} INNER corners "
          f"(that is a {args.cols + 1}x{args.rows + 1}-SQUARE board -- inner "
          f"corners, not squares)")
    print(f"{LOG} output: {args.out}   (files {SAVE_PREFIX}NNN.png)")
    if interactive:
        print(f"{LOG} exposure {args.exposure_us} us, gain {args.gain} -- "
              f"CALIBRATION-ONLY DEVIATION from the <= {EXPOSURE_SPEC_US} us "
              f"project spec.")
        print(f"{LOG}   The <=1 ms spec exists to kill MOTION blur at >=9 m/s. "
              f"The board is STATIC, so blur is irrelevant here, while 1 ms "
              f"indoors gives a near-black frame the corner finder fails on. "
              f"Do NOT read this session as a motion-blur measurement.")
    print(f"{LOG} goal: >= {MIN_VIEWS} views, >= {MIN_ZONES} of 9 frame zones, "
          f"and all three distances (NEAR/MID/FAR).")
    print(f"{LOG} tilt the board ~20-45 deg for most views, and push it right "
          f"into the four frame CORNERS -- that is where the wide lens's "
          f"distortion is measured.")


def run_interactive(args):
    import cv2
    cov = Coverage()
    os.makedirs(args.out, exist_ok=True)
    _header(args, interactive=True)
    preload_existing(cv2, args.out, args.cols, args.rows, cov)
    idx = _next_index(args.out)

    # n_frames=0 / duration=None -> an unbounded stream (see iter_picamera2).
    gen = iter_picamera2(cv2, args.width, args.height, args.exposure_us,
                         args.gain, 0, None)
    print(f"{LOG} camera starting...")
    try:
        # Pull one frame up front so a camera failure surfaces NOW, not after
        # the operator has set up the first view.
        first = next(gen)
        print(f"{LOG} camera live at {first.gray.shape[1]}x{first.gray.shape[0]}"
              + (f", exposure {first.exposure_us:.0f} us"
                 if first.exposure_us is not None else "")
              + (f", gain {first.gain:.2f}" if first.gain is not None else ""))
        print(f"{LOG} ENTER = grab a view | s = skip | q = done + verdict")
        while True:
            try:
                line = input(f"[{cov.saved} views] ENTER=grab  s=skip  q=done > ")
            except EOFError:
                # stdin closed (piped input, dropped ssh session): finish
                # cleanly and still print the verdict.
                print(f"\n{LOG} stdin closed -- finishing the session.")
                break
            cmd = line.strip().lower()
            if cmd in ("q", "quit", "done"):
                break
            if cmd in ("s", "skip"):
                continue
            if cmd not in ("", "g", "grab"):
                print(f"{LOG} unrecognised '{cmd}' -- ENTER grabs, s skips, "
                      f"q finishes.")
                continue
            # Drain the queue so we classify what the camera sees NOW.
            for _ in range(DRAIN_FRAMES):
                try:
                    next(gen)
                except StopIteration:
                    break
            try:
                fr = next(gen)
            except StopIteration:
                print(f"{LOG} the camera stopped delivering frames -- "
                      f"finishing the session.", file=sys.stderr)
                break
            _info, path = handle_frame(cv2, fr.gray, cov, args.out, args.cols,
                                       args.rows, idx)
            if path is not None:
                idx += 1
    finally:
        gen.close()   # runs picam2.stop() in the generator's finally

    ready, lines = summarize(cov, args.out, args.cols, args.rows)
    print("\n".join(lines))
    return 0 if ready else 1


def run_auto(args, path, quiet=False):
    """Non-interactive: classify up to --auto N stills from PATH through exactly
    the same corner finder / classifier / verdict. This is what makes the tool
    testable without a camera."""
    import cv2
    cov = Coverage()
    os.makedirs(args.out, exist_ok=True)
    if not quiet:
        _header(args, interactive=False)
        print(f"{LOG} source: dir={path} (non-interactive, "
              f"limit={args.auto or 'all'})")
    preload_existing(cv2, args.out, args.cols, args.rows, cov, quiet=quiet)
    idx = _next_index(args.out)
    n_seen = 0
    for fr in iter_dir(cv2, path, args.auto or 0, 30.0):
        n_seen += 1
        _info, saved = handle_frame(cv2, fr.gray, cov, args.out, args.cols,
                                    args.rows, idx, label=f"#{n_seen}",
                                    quiet=quiet)
        if saved is not None:
            idx += 1
    ready, lines = summarize(cov, args.out, args.cols, args.rows)
    if not quiet:
        print("\n".join(lines))
    return (0 if ready else 1), cov, "\n".join(lines)


# --------------------------------------------------------------------------
# Self-test -- no camera, no display, exits 0/1
# --------------------------------------------------------------------------

def _synth_board(cv2, np, square_px, cols_sq=10, rows_sq=7, margin_sq=0.6):
    """A real chessboard image: `cols_sq` x `rows_sq` alternating squares plus a
    white quiet-zone margin (findChessboardCorners needs the white border).
    Returns (img, margin_px, square_px)."""
    m = int(round(margin_sq * square_px))
    w = cols_sq * square_px + 2 * m
    h = rows_sq * square_px + 2 * m
    img = np.full((h, w), 255, np.uint8)
    for r in range(rows_sq):
        for c in range(cols_sq):
            if (r + c) % 2 == 0:
                y0, x0 = m + r * square_px, m + c * square_px
                img[y0:y0 + square_px, x0:x0 + square_px] = 0
    return img, m, square_px


def _place_board(cv2, np, canvas, board, scale, tx, ty, tilt=0.0):
    """Paste `board` onto `canvas` scaled by `scale`, top-left at (tx,ty), with
    an optional mild perspective `tilt` (fraction of width). A pure scale+shift
    (tilt=0) maps board pixels to frame pixels EXACTLY, which lets the self-test
    predict the inner-corner bbox analytically instead of trusting the tool."""
    bh, bw = board.shape[:2]
    src = np.float32([[0, 0], [bw, 0], [bw, bh], [0, bh]])
    dw, dh = bw * scale, bh * scale
    dx = tilt * dw
    dst = np.float32([[tx, ty + dx * 0.5], [tx + dw, ty],
                      [tx + dw, ty + dh], [tx, ty + dh - dx * 0.5]])
    M = cv2.getPerspectiveTransform(src, dst)
    cv2.warpPerspective(board, M, (canvas.shape[1], canvas.shape[0]), canvas,
                        flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
    return canvas


def _frame_with_board(cv2, np, w, h, square_px, scale, tx, ty, tilt=0.0):
    canvas = np.full((h, w), 235, np.uint8)
    board, m, sq = _synth_board(cv2, np, square_px)
    _place_board(cv2, np, canvas, board, scale, tx, ty, tilt)
    # Analytic inner-corner bbox for a pure scale+shift placement: inner corners
    # sit at margin + i*square for i in 1..cols_sq-1 (9 of them) and
    # margin + j*square for j in 1..rows_sq-1 (6).
    inner_w = 8 * sq * scale
    inner_h = 5 * sq * scale
    x0 = tx + (m + sq) * scale
    y0 = ty + (m + sq) * scale
    exp = {"bbox": (x0, y0, x0 + inner_w, y0 + inner_h),
           "centroid": (x0 + inner_w / 2.0, y0 + inner_h / 2.0),
           "area_pct": 100.0 * inner_w * inner_h / float(w * h)}
    return canvas, exp


def _expected_zone(cx, cy, w, h):
    return min(2, int(cy * 3 / h)) * 3 + min(2, int(cx * 3 / w))


def _expected_bucket(area_pct):
    return ("NEAR" if area_pct >= NEAR_MIN_PCT
            else "MID" if area_pct >= MID_MIN_PCT else "FAR")


def self_test():
    import io
    import shutil
    import tempfile
    from contextlib import redirect_stdout

    import numpy as np
    import cv2

    W, H = DEF_WIDTH, DEF_HEIGHT
    checks = []

    def ck(name, ok, detail=""):
        checks.append((name, bool(ok), detail))
        print(f"{LOG}   [{'PASS' if ok else 'FAIL'}] {name}"
              + (f" -- {detail}" if detail else ""))

    tmp = tempfile.mkdtemp(prefix="calibviews_selftest_")
    try:
        def args_for(out):
            return argparse.Namespace(out=out, cols=DEF_COLS, rows=DEF_ROWS,
                                      auto=0, exposure_us=DEF_EXPOSURE_US,
                                      gain=DEF_GAIN, width=W, height=H)

        def run_over(frames, out_name, limit=0):
            """Write `frames` into a source dir, run the real dir-mode path."""
            src = os.path.join(tmp, out_name + "_src")
            out = os.path.join(tmp, out_name + "_out")
            os.makedirs(src, exist_ok=True)
            os.makedirs(out, exist_ok=True)
            for i, f in enumerate(frames):
                cv2.imwrite(os.path.join(src, f"{i:04d}.png"), f)
            a = args_for(out)
            a.auto = limit
            buf = io.StringIO()
            with redirect_stdout(buf):
                code, cov, _summary = run_auto(a, src)
            return code, cov, buf.getvalue(), out

        # ---- check 1: a big central board -> FOUND, saved, right zone+bucket
        f_near, exp_near = _frame_with_board(cv2, np, W, H, 62, 1.0, 300, 130)
        code, cov, out_txt, outdir = run_over([f_near], "near")
        z = _expected_zone(*exp_near["centroid"], W, H)
        b = _expected_bucket(exp_near["area_pct"])
        saved_files = sorted(glob.glob(os.path.join(outdir, "calib_*.png")))
        ck("found board is SAVED as calib_000.png",
           len(saved_files) == 1 and saved_files[0].endswith("calib_000.png"),
           f"{[os.path.basename(p) for p in saved_files]}")
        ck("found board classified into the analytically-expected ZONE",
           cov.saved == 1 and cov.zone_counts[z] == 1,
           f"expected {ZONE_NAMES[z]}, counts={cov.zone_counts}")
        ck("found board classified into the analytically-expected BUCKET",
           cov.bucket_counts.get(b) == 1,
           f"expected {b} (analytic area {exp_near['area_pct']:.1f}%), "
           f"got {dict(cov.bucket_counts)}")
        ck("a single view is NOT READY and exits 1", code == 1
           and "VERDICT: READY" not in out_txt,
           f"exit={code}")

        # ---- check 1b: measured area_pct agrees with the analytic value
        cov_probe = Coverage()
        info, _ = handle_frame(cv2, f_near, cov_probe, tmp, DEF_COLS, DEF_ROWS,
                               0, save=False, quiet=True)
        ck("measured board area matches the analytic bbox (<0.5% abs)",
           info is not None
           and abs(info["area_pct"] - exp_near["area_pct"]) < 0.5,
           f"measured {info['area_pct']:.2f}% vs analytic "
           f"{exp_near['area_pct']:.2f}%" if info else "no board found")

        # ---- check 2: a small top-left board -> FAR bucket, corner zone
        f_far, exp_far = _frame_with_board(cv2, np, W, H, 26, 1.0, 20, 20)
        code, cov, out_txt, _ = run_over([f_far], "far")
        z2 = _expected_zone(*exp_far["centroid"], W, H)
        b2 = _expected_bucket(exp_far["area_pct"])
        ck("small corner board -> expected zone + distance bucket",
           cov.saved == 1 and cov.zone_counts[z2] == 1
           and cov.bucket_counts.get(b2) == 1,
           f"expected {ZONE_NAMES[z2]}/{b2} "
           f"(analytic {exp_far['area_pct']:.1f}%), counts={cov.zone_counts}, "
           f"{dict(cov.bucket_counts)}")
        ck("the two probe frames landed in DIFFERENT zones and buckets",
           z != z2 and b != b2, f"{ZONE_NAMES[z]}/{b} vs {ZONE_NAMES[z2]}/{b2}")

        # ---- check 3: blank + noise -> NOT FOUND, nothing saved, exit 1
        rng = np.random.default_rng(7)
        blank = np.full((H, W), 200, np.uint8)
        noise = rng.integers(0, 255, (H, W), dtype=np.uint8)
        code, cov, out_txt, outdir = run_over([blank, noise], "junk")
        ck("blank/noise frames are NOT FOUND and save nothing",
           cov.saved == 0 and cov.not_found == 2
           and not glob.glob(os.path.join(outdir, "calib_*.png")),
           f"saved={cov.saved} not_found={cov.not_found}")
        ck("ZERO usable views exits 1 and NEVER prints READY",
           code == 1 and "VERDICT: READY" not in out_txt
           and "VERDICT: NOT READY" in out_txt, f"exit={code}")

        # ---- check 4: a full, well-spread set -> READY, exit 0
        # 16 views: every zone, all three distance buckets, some tilted. The
        # square sizes are chosen so the INNER-corner bbox lands in the intended
        # bucket by construction (inner bbox = 8*sq x 5*sq = 40*sq^2 px, so
        # sq=82 -> 26.3% NEAR, sq=34 -> 4.5%... see the per-group notes), and the
        # placements are checked to keep the whole board inside 1280x800 -- a
        # board clipped by the frame edge is genuinely NOT FOUND, which would
        # make this check flaky for the wrong reason.
        spread = []
        # NEAR: sq=82 -> inner 656x410 = 26.3% of frame (>= 25%). Board is
        # 918x672 px, so tx <= 362 and ty <= 128.
        for (tx, ty, tl) in [(20, 10, 0.08), (340, 10, -0.06),
                             (20, 120, 0.05), (340, 120, -0.05)]:
            spread.append(_frame_with_board(cv2, np, W, H, 82, 1.0, tx, ty, tl)[0])
        # MID: sq=34 -> inner 272x170 = 4.5%... which is FAR. Use sq=50 ->
        # inner 400x250 = 9.8%, inside 8-25%. Board is 560x410, so tx <= 720,
        # ty <= 390; the three row/column bands below put one view in each of
        # the 9 zones (centroid = tx+280 in x, ty+205 in y).
        mid_pos = [(10, 10), (350, 10), (700, 10),
                   (10, 200), (350, 200), (700, 200),
                   (10, 385), (350, 385), (700, 385)]
        for (tx, ty) in mid_pos:
            spread.append(_frame_with_board(cv2, np, W, H, 50, 1.0, tx, ty,
                                            0.05)[0])
        # FAR: sq=22 -> inner 176x110 = 1.9%. Board is 246x180.
        for (tx, ty) in [(15, 15), (1020, 15), (15, 605)]:
            spread.append(_frame_with_board(cv2, np, W, H, 22, 1.0, tx, ty)[0])
        code, cov, out_txt, _ = run_over(spread, "ready")
        ck("a well-spread set is READY and exits 0",
           code == 0 and "VERDICT: READY" in out_txt,
           f"exit={code} saved={cov.saved} zones={cov.zones_filled}/9 "
           f"buckets={dict(cov.bucket_counts)}")
        ck("the READY set really met all three thresholds",
           cov.saved >= MIN_VIEWS and cov.zones_filled >= MIN_ZONES
           and not cov.empty_buckets(),
           f"saved={cov.saved} zones={cov.zones_filled} "
           f"empty_buckets={cov.empty_buckets()}")

        # ---- check 5: the COUNT threshold flips the verdict
        # Same well-spread frames, capped by --auto to one short of MIN_VIEWS.
        # (Uses --auto, so it also exercises the limit.)
        n_ok = cov.saved
        code_lo, cov_lo, txt_lo, _ = run_over(spread, "count_lo",
                                              limit=MIN_VIEWS - 1)
        ck("one view short of the minimum flips READY -> NOT READY",
           code_lo == 1 and cov_lo.saved < MIN_VIEWS
           and "VERDICT: READY" not in txt_lo
           and f"{MIN_VIEWS - cov_lo.saved} more view(s)" in txt_lo,
           f"saved={cov_lo.saved}/{MIN_VIEWS} exit={code_lo} (full set had "
           f"{n_ok})")

        # ---- check 6: enough views, but clustered -> NOT READY on ZONES
        clustered = []
        for i in range(MIN_VIEWS + 3):
            dx = (i % 3) * 6
            clustered.append(_frame_with_board(cv2, np, W, H, 34, 1.0,
                                               470 + dx, 290, 0.04)[0])
        code_c, cov_c, txt_c, _ = run_over(clustered, "clustered")
        ck("enough views but clustered -> NOT READY naming the missing zones",
           code_c == 1 and cov_c.saved >= MIN_VIEWS
           and cov_c.zones_filled < MIN_ZONES
           and "more zone(s) covered" in txt_c
           and "VERDICT: READY" not in txt_c,
           f"saved={cov_c.saved} zones={cov_c.zones_filled}/9 exit={code_c}")
        ck("a clustered set is warned about the empty frame CORNERS",
           "frame CORNERS" in txt_c and "distortion is estimated FROM the "
           "corners" in txt_c)

        # ---- check 7: bucket threshold alone can fail an otherwise-good set
        cov_b = Coverage()
        for zone_i in range(9):
            cov_b.zone_counts[zone_i] = 2
        cov_b.saved = MIN_VIEWS + 3
        cov_b.bucket_counts = {"NEAR": 9, "MID": 9, "FAR": 0}
        ready_b, lines_b = summarize(cov_b, "/tmp/x", DEF_COLS, DEF_ROWS)
        txt_b = "\n".join(lines_b)
        ck("all zones + enough views but an EMPTY distance bucket -> NOT READY",
           (not ready_b) and "nothing in FAR" in txt_b
           and "VERDICT: READY" not in txt_b)
        cov_b.bucket_counts["FAR"] = 4
        ready_b2, lines_b2 = summarize(cov_b, "/tmp/x", DEF_COLS, DEF_ROWS)
        ck("filling that bucket flips it to READY",
           ready_b2 and "VERDICT: READY" in "\n".join(lines_b2))

        # ---- check 8: the next-command line carries the REAL output dir
        ck("summary prints the ready-to-paste calibrate_camera command",
           f'--images "{os.path.join(tmp, "ready_out", SAVE_PREFIX)}*.png"'
           in out_txt and "--square <YOUR_SQUARE_METRES>" in out_txt
           and f"--cols {DEF_COLS} --rows {DEF_ROWS}" in out_txt)

        # ---- check 9: the detector really is calibrate_camera's own
        import calibrate_camera as cc
        ck("uses calibrate_camera._find_corners (no second corner finder)",
           _find_corners is cc._find_corners)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    n_fail = sum(1 for _n, ok, _d in checks if not ok)
    print(f"{LOG} SELF-TEST {'PASS' if n_fail == 0 else 'FAIL'} "
          f"({len(checks) - n_fail}/{len(checks)} checks passed)")
    return 0 if n_fail == 0 else 1


# --------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Headless chessboard-view capture for skr-06 camera "
                    "calibration (tells you after EVERY shot whether the board "
                    "was found and where it landed).")
    ap.add_argument("--out", help="directory for the captured calib_NNN.png "
                                  "views (created if needed)")
    ap.add_argument("--cols", type=int, default=DEF_COLS,
                    help=f"INNER corners per row (default {DEF_COLS}; a "
                         f"10-square row has 9)")
    ap.add_argument("--rows", type=int, default=DEF_ROWS,
                    help=f"INNER corners per column (default {DEF_ROWS})")
    ap.add_argument("--exposure-us", type=int, default=DEF_EXPOSURE_US,
                    help=f"exposure in microseconds (default {DEF_EXPOSURE_US}; "
                         f"deliberately above the {EXPOSURE_SPEC_US} us motion-"
                         f"blur spec -- the board is static, see the header)")
    ap.add_argument("--gain", type=float, default=DEF_GAIN,
                    help=f"analogue gain (default {DEF_GAIN})")
    ap.add_argument("--width", type=int, default=DEF_WIDTH)
    ap.add_argument("--height", type=int, default=DEF_HEIGHT)
    ap.add_argument("--source", default="picamera2",
                    help="picamera2 (default, interactive) | dir=PATH "
                         "(non-interactive replay of existing stills)")
    ap.add_argument("--auto", type=int, default=0, metavar="N",
                    help="with --source dir=PATH: classify at most N stills "
                         "(0 = all). Non-interactive.")
    ap.add_argument("--self-test", action="store_true",
                    help="offline proof of the whole path (no camera, no "
                         "display); exits 0/1")
    args = ap.parse_args(argv)

    if args.self_test:
        return self_test()
    if not args.out:
        ap.error("--out DIR is required (or use --self-test)")
    if args.cols < 2 or args.rows < 2:
        ap.error("--cols/--rows are INNER corner counts and must be >= 2")

    if args.source.startswith("dir="):
        path = args.source[len("dir="):]
        if not os.path.isdir(path):
            raise SystemExit(f"{LOG} --source dir={path!r} is not a directory")
        return run_auto(args, path)[0]
    if args.source == "picamera2":
        if args.auto:
            # Fail closed rather than silently ignore: --auto means "do not
            # prompt me", and going interactive anyway would strand an operator
            # who expected an unattended run.
            raise SystemExit(f"{LOG} --auto is only meaningful with "
                             f"--source dir=PATH (the picamera2 mode is "
                             f"interactive by design -- you need to move the "
                             f"board between shots).")
        return run_interactive(args)
    raise SystemExit(f"{LOG} unknown --source {args.source!r} "
                     f"(use picamera2 or dir=PATH)")


if __name__ == "__main__":
    sys.exit(main())
