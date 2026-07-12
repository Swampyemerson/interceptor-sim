"""DETECT-THEN-TRACK for the markerless onboard seeker (ADR-0058, builder-directed).

THE PROBLEM (ADR-0056/0057). At 12 m/s + weave/jink the markerless NN throws
gross PHANTOMS (a big high-confidence box ~18 m off the true target) intermixed
with a few low-confidence GOOD looks at the real, few-pixel target. ADR-0057's
characterization: over ~217 ENGAGE detections across 16 weave flights, 195 were
FALSE (range err > 8 m) vs 5 GOOD. Confidence is INVERTED (phantoms score higher),
so a confidence gate is dead. The maneuver removes the temporal averaging that had
been smoothing the straight-line box-center bearing noise, so guidance chases
phantoms -> 4-9 m misses while the clean-AprilTag control holds 8/8 at 1.6 m.

THE FIX (the classic solution). The NN IDENTIFIES the target ONCE, validated so we
seed on the REAL target not a phantom; then a CHEAP VISUAL TRACKER (cv2 CSRT)
FOLLOWS that same visual blob frame-to-frame -- temporally consistent, denser than
the ~14 Hz NN, and it IGNORES phantom NN boxes elsewhere in the frame. This is the
camera-only "temporal fusion to reject false positives" the builder asked for.

  ACQUIRE  -- run the NN. On a VALIDATED detection -- cue-consistent PRE-handoff;
              POST-handoff consistent with the guidance's own velocity-aware
              camera-track prediction inside a coast-time-aged radius (NEVER a
              confidence gate: ADR-0057 measured confidence INVERTED, phantoms
              score HIGHER than the real target) -- init a CSRT tracker on its
              box. Publish the NN detection.
  TRACK    -- each frame: tracker.update(frame) -> box -> Measurement via the SAME
              box->measurement conversion the NN uses (seeker.box_to_detection).
              This dense, temporally-coherent camera track is the jam-resistant
              terminal seeker.
  RE-VALIDATE / DRIFT -- every N frames re-run the NN. If an NN box OVERLAPS the
              tracker box (IoU >= thresh), re-seed to it (correct drift). If the NN
              DISAGREES: pre-handoff, re-seed only if the cue says the NN box is
              right AND the tracker box is wrong (real drift); post-handoff (no
              cue) KEEP the tracker and ignore the NN (the anti-phantom rule). If
              the tracker fails or the box degenerates, coast (no detection) and
              re-acquire via a fresh validated NN detection.

HONESTY BOUNDARY (CLAUDE.md / ADR-0010). The tracker operates on IMAGE FRAMES ONLY
(camera pixels + fixed intrinsics). The SEED may use the external cue ONLY
PRE-handoff -- exactly where the cue is legitimately available (its whole job is to
guide the dash until the camera truly acquires). POST-handoff `SeekerSeedContext`
stops handing the cue over (handoff_done -> cue_pos=None) and the tracker runs
CAMERA-ONLY, so the terminal is jam-resistant. NEVER reads gt_*; own pos/yaw come
from PX4's own EKF (own-state, legal, ADR-0008 split).

Deps: cv2 (opencv, TrackerCSRT/KCF/MIL/Vit) from .venv-seeker / the combined
flight venv. VIT (MARKERLESS_TRACK_KIND=VIT, opt-in, CSRT stays default) adds
one small ONNX weight file (scripts/seeker/weights/object_tracking_vittrack_
2023sep.onnx, OpenCV Zoo, BSD license, ~700 KB) -- see weights/LICENSES.md.
No new Python dependency: cv2.dnn runs the ONNX directly, same as the NN seeker.
"""
from __future__ import annotations

import math
import os

import numpy as np   # ADR-0071 subpixel-centroid bearing
from typing import Optional, Tuple

import cv2

from nn_seeker import SeekerDetection  # the None-fill / field contract


# --------------------------------------------------------------------- kind gate
# Trackers this module supports. KCF is deliberately EXCLUDED (see
# validate_track_kind): it has no scale adaptation, so the range-from-box-width
# channel would freeze at the seed value. VIT (ADR-0076 fix #2) is a learned
# Siamese/transformer tracker (cv2.TrackerVit, OpenCV Zoo vittrack ONNX) --
# robust to the frame-to-frame appearance change a correlation tracker (CSRT)
# loses lock on for a banking/maneuvering target; scale-adaptive (verified:
# its box width tracks a growing/shrinking target), so range-from-box-width
# behaves the same as under CSRT. Opt-in only -- CSRT stays the default.
_VALID_TRACK_KINDS = ("CSRT", "MIL", "VIT")

# Default VIT model path (OpenCV Zoo vittrack, BSD license -- see
# weights/LICENSES.md). Resolved relative to THIS FILE's directory, not the
# caller's cwd, so the tracker builds regardless of where a script is invoked
# from. MARKERLESS_VIT_MODEL overrides it (e.g. to point at a different/newer
# vittrack export) without a code change.
_VIT_MODEL_DEFAULT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "weights", "object_tracking_vittrack_2023sep.onnx")


def validate_track_kind(kind: Optional[str] = None) -> str:
    """Validate MARKERLESS_TRACK_KIND and return its canonical UPPER name, or
    RAISE on a refused/unknown value (review finding #4).

    Call this in the MAIN thread BEFORE the detector thread starts (see
    m4_intercept.py): the tracker is otherwise constructed INSIDE the detection
    thread (markerless_loop.py), where a raise is a silent thread death visible
    only as a stderr traceback + a downstream staleness abort. Validating up
    front turns a bad value into a loud, immediate, main-thread error.
    DetectThenTracker.__init__ also calls this, so the two paths cannot disagree.

    KCF is refused outright: it has NO scale adaptation, so range-from-box-width
    (which feeds Vc / breakoff / the handoff trigger) would freeze at the seed
    value. Unknown values are refused (not silently coerced to CSRT, the old
    _new_tracker fall-through behaviour). VIT (cv2.TrackerVit, a learned/
    transformer tracker, ADR-0076 fix #2) is accepted as an opt-in A/B arm
    against CSRT; it needs its ONNX weight file present (see _VIT_MODEL_DEFAULT
    / MARKERLESS_VIT_MODEL), checked in DetectThenTracker.__init__ so a missing
    weight is also a loud immediate error, not a silent thread death."""
    if kind is None:
        kind = os.environ.get("MARKERLESS_TRACK_KIND", "CSRT")
    k = str(kind).upper()
    if k == "KCF":
        raise ValueError(
            "MARKERLESS_TRACK_KIND=KCF refused: KCF has no scale adaptation, "
            "so range-from-box would be frozen at the seed value (feeds Vc/"
            "breakoff/handoff). Use CSRT (default, scale-adaptive), MIL, or VIT.")
    if k not in _VALID_TRACK_KINDS:
        raise ValueError(
            f"MARKERLESS_TRACK_KIND={kind!r} is not a known tracker; use one of "
            f"{_VALID_TRACK_KINDS} (default CSRT). KCF is refused separately "
            "(no scale adaptation).")
    return k


# --------------------------------------------------------------------------- ctx
class SeekerSeedContext:
    """Own-state + pre-handoff cue snapshot shared from the guidance loop
    (m4_intercept.run_acquire_and_engage) into the markerless detection thread,
    so the detect-then-track SEED can be CUE-VALIDATED (the camera-implied target
    position must agree with where the cue says the target is) before we trust it.

    Latest-wins, single-attribute atomic assignment (the same lock-free pattern as
    m3_static_intercept.MeasurementHolder / LatestFrame): the guidance loop writes
    each field as ONE assignment per tick, the detector thread reads each field
    ONCE per frame. `own`, `cue_pos` and `cam_track` are written as whole tuples
    so a reader never sees a half-updated (n, e).

    POST-handoff honesty (two belts, review finding #1): `update()` forces
    cue_pos=None once handoff_done (the null-under-latch), AND every consumer
    must read the cue through `legal_cue_pos()`, which RAISES if a cue position
    is ever observable after the latch (a structural violation -- e.g. something
    assigning .cue_pos directly). A raise kills the detection thread, so
    measurements stop and the guidance coasts/aborts: fail-safe -- never fly on
    illegal data (ADR-0010 #5). Both belts are pinned by
    tests/test_honesty_static.py.

    `cam_track` is the guidance loop's own TargetTracker position estimate
    (tgt_n_hat, tgt_e_hat): the alpha-beta CAMERA track, predict()ed every tick,
    i.e. a VELOCITY-AWARE prediction through a detection coast. LEGAL post-
    handoff: post-latch that tracker is corrected ONLY from camera measurements
    (the cue channel is closed one-way at the latch); its pre-latch cue warm-up
    carrying forward is the same legality as --warm-handoff (ADR-0015/0018)."""

    def __init__(self):
        self.own: Optional[Tuple[float, float, float]] = None   # (pos_n, pos_e, psi_rad)
        self.cue_pos: Optional[Tuple[float, float]] = None      # (n, e) cue target NED
        self.cam_track: Optional[Tuple[float, float]] = None    # (n, e) camera-track est
        self.handoff_done: bool = False

    def update(self, pos_n, pos_e, psi_rad, cue_pos, handoff_done, cam_track=None):
        self.own = (
            (pos_n, pos_e, psi_rad)
            if (pos_n is not None and pos_e is not None and psi_rad is not None)
            else None
        )
        self.cue_pos = None if handoff_done else cue_pos
        self.cam_track = cam_track
        self.handoff_done = bool(handoff_done)

    def legal_cue_pos(self):
        """The ONLY sanctioned cue read (honesty belt #2). Returns the pre-handoff
        cue position, or None post-latch -- and RAISES if a cue position is ever
        present after the latch. update() makes that unrepresentable; a raise here
        means something bypassed it, and the fail-safe is to kill the detection
        thread rather than fly on post-latch cue data."""
        if self.handoff_done:
            if self.cue_pos is not None:
                raise RuntimeError(
                    "HONESTY VIOLATION (ADR-0010 #5): SeekerSeedContext.cue_pos "
                    "is non-None after the handoff latch -- post-latch cue reads "
                    "are structurally forbidden")
            return None
        return self.cue_pos


# ------------------------------------------------------------------------- boxes
def _iou(a, b) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ax2, ay2, bx2, by2 = ax + aw, ay + ah, bx + bw, by + bh
    ix1, iy1 = max(ax, bx), max(ay, by)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def _box_center_dist(a, b) -> float:
    acx, acy = a[0] + a[2] / 2.0, a[1] + a[3] / 2.0
    bcx, bcy = b[0] + b[2] / 2.0, b[1] + b[3] / 2.0
    return math.hypot(acx - bcx, acy - bcy)


# -------------------------------------------------------------------- the tracker
class DetectThenTracker:
    """NN-acquire -> CSRT-track -> NN-re-validate state machine. Drop-in for
    `seeker.detect(frame_bgr, t_mono)`: call `.process(frame_bgr, t_mono)` per
    frame and it returns a `SeekerDetection` exactly like the bare seeker, but
    sourced from a temporally-consistent visual track once acquired.

    Tunables are read from the environment (MARKERLESS_TRACK_*) so the live A/B
    can sweep them without a code edit; all default to sensible values."""

    def __init__(self, seeker, fx, fy, cx, cy, seed_ctx: Optional[SeekerSeedContext] = None):
        self.seeker = seeker
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.seed_ctx = seed_ctx

        def _envf(name, default):
            try:
                return float(os.environ.get(name, default))
            except (TypeError, ValueError):
                return float(default)

        def _envi(name, default):
            try:
                return int(os.environ.get(name, default))
            except (TypeError, ValueError):
                return int(default)

        # Tracker kind, VALIDATED (KCF refused for its frozen range channel;
        # unknown values refused, not silently coerced -- review finding #4).
        # validate_track_kind() is ALSO called in the MAIN thread (m4_intercept)
        # before this thread starts, so a bad value is a loud immediate error
        # rather than a silent detector-thread death here.
        self.tracker_kind = validate_track_kind()
        # VIT model path (only resolved/checked when actually selected, so CSRT/
        # MIL construction never touches the filesystem for a file they don't
        # need). Fail LOUD here (main-thread __init__, same pattern as the KCF
        # refuse above) rather than inside _new_tracker deep in the ACQUIRE path.
        self.vit_model_path = os.environ.get("MARKERLESS_VIT_MODEL", _VIT_MODEL_DEFAULT)
        if self.tracker_kind == "VIT" and not os.path.isfile(self.vit_model_path):
            raise FileNotFoundError(
                f"MARKERLESS_TRACK_KIND=VIT needs the vittrack ONNX weight at "
                f"{self.vit_model_path!r} (missing). Set MARKERLESS_VIT_MODEL to "
                "point at a different file, or fetch the default one -- see "
                "scripts/seeker/weights/LICENSES.md.")
        # SEED gate: camera-implied target within this many m of the cue (option a,
        # the primary, trustworthy seed gate). 8 m matches --handoff-cue-gate.
        self.seed_cue_gate_m = _envf("MARKERLESS_TRACK_SEED_GATE_M", 8.0)
        # NN re-validation cadence (frames) + IoU to accept an NN box as the same
        # target (drift correction).
        self.revalidate_every = max(1, _envi("MARKERLESS_TRACK_REVALIDATE", 8))
        self.reseed_iou = _envf("MARKERLESS_TRACK_RESEED_IOU", 0.2)
        # SUBPIXEL bearing (ADR-0071, default 0 = OFF, byte-identical): the
        # published terminal bearing comes from the tracker box CENTER, which
        # jitters ~box-width/2 frame-to-frame -- the ~1.5 m terminal miss FLOOR
        # (ADR-0056/0070: even a clean AprilTag seeker sits 1.64 m). When ON,
        # replace the box center with a DARKNESS-WEIGHTED CENTROID of the small
        # dark target within the box (drone vs the lighter sky/ground) -> a
        # subpixel, edge-jitter-immune target center. Camera pixels only (no cue,
        # no gt) -> honesty-clean. Only the PUBLISHED bearing is refined; the
        # tracker box + all gating stay byte-identical.
        self.subpixel = _envi("MARKERLESS_TRACK_SUBPIXEL", 0)
        self.subpixel_dark_pctl = _envf("MARKERLESS_TRACK_SUBPIXEL_PCTL", 40.0)
        # POST-HANDOFF RE-ACQUIRE gate (review finding #2): a re-seed with no cue
        # must be consistent with the guidance's own VELOCITY-AWARE camera-track
        # prediction (seed_ctx.cam_track), within a radius that AGES with coast
        # time (prediction error grows while coasting) at a bounded rate, hard-
        # capped. NO confidence gate: ADR-0057 measured confidence INVERTED
        # (phantom conf ~0.34 > real ~0.17), so a conf floor keeps phantoms and
        # drops the real target -- structurally useless here.
        self.reacq_base_m = _envf("MARKERLESS_TRACK_REACQ_BASE_M", 8.0)
        self.reacq_rate_m_s = _envf("MARKERLESS_TRACK_REACQ_RATE_M_S", 5.0)
        # Cap default 12 m (was 20). The canonical phantom sits ~18 m off the
        # true target (ADR-0057), so a cap at/above ~18 would re-admit exactly
        # the phantom this gate exists to reject once a long coast ages the
        # radius out to the cap. 12 m (< 18) keeps the phantom rejected at EVERY
        # coast time while still tolerating real velocity-aware prediction error.
        # Env-tunable (MARKERLESS_TRACK_REACQ_CAP_M) for the A/B sweep.
        self.reacq_cap_m = _envf("MARKERLESS_TRACK_REACQ_CAP_M", 12.0)
        # FRAME-BASED COAST CLOCK (review finding #1 / ADR-0009): the coast that
        # ages the re-acquire radius is measured in FRAMES-since-loss * this
        # nominal sim-time-per-processed-frame, NOT wall time. time.monotonic()
        # sagged 2-3x under RTF load, so wall-time aging grew the radius that much
        # too fast per sim-second; frames are the detection loop's natural clock
        # (the same basis as the every-N-frames re-validation cadence). NOTE
        # (ADR-0058 residual #1, softened here): this is RTF-ROBUST, not fully
        # RTF-invariant -- FRAME_DT_S embeds the RTF~1 NN cadence, so under RTF sag
        # the ACQUIRE loop turns camera-bound (30 Hz sim) and the coast clock can
        # run up to ~30/14 ~ 2.1x fast in sim-seconds. The REACQ_CAP_M cap carries
        # the anti-phantom guarantee regardless (the ~18 m phantom is rejected at
        # EVERY coast age), and batch arms fly at RTF~1, so this is not batch-
        # invalidating. Default 1/14 s ~ the detector's measured ~14 Hz effective
        # cadence. Env-tunable.
        self.frame_dt_s = _envf("MARKERLESS_TRACK_FRAME_DT_S", 1.0 / 14.0)
        # Degeneracy guards for a bad tracker box.
        self.min_box_px = _envf("MARKERLESS_TRACK_MIN_BOX_PX", 4.0)
        self.max_box_frac = _envf("MARKERLESS_TRACK_MAX_BOX_FRAC", 0.9)
        # FRAME-BORDER COAST (audit #37): the NN's own detect() self-masks
        # edge-touching boxes (finetuned_seeker.edge_margin_px), but a CSRT
        # TRACKER box bypasses every mask -- and a box clipped by the frame edge
        # has a truncated width (under-reading the width-based range, feeding a
        # false past-CPA range rise) and a biased centre (skewing bearing). When
        # >0, COAST (publish nothing, keep the lock) on any frame where the
        # tracker box comes within this many px of a frame edge. Default 0 =
        # disabled = byte-identical. Deployment: ~8 px.
        self.border_margin_px = _envf("MARKERLESS_TRACK_BORDER_MARGIN_PX", 0.0)

        self.tracker = None
        self.last_box: Optional[Tuple[float, float, float, float]] = None
        self.last_score = 0.0
        self.since_nn = 0
        # Coast clock: PROCESSED frames since the lock was lost (None = never
        # locked / currently locked). Ages the post-handoff re-acquire radius on
        # an RTF-invariant frame basis (review finding #1), never wall time.
        self.frames_since_loss: Optional[int] = None
        # diagnostics (read by the smoke test / logged on stop)
        self.n_seed = self.n_reseed = self.n_lost = 0
        self.n_track_frames = self.n_nn_frames = self.n_phantom_ignored = 0
        self.n_reacq_rejected = 0
        self.n_border_coast = 0   # frames coasted for a frame-edge-clipped box (audit #37)

    # ---- seeker-agnostic box -> Measurement (SAME conversion the NN uses) ----
    def _box_to_det(self, box, t_mono, score, n) -> SeekerDetection:
        return self.seeker.box_to_detection(tuple(box), t_mono, score, n)

    def _subpixel_center(self, frame_bgr, box):
        """ADR-0071 darkness-weighted subpixel centroid of the dark target within
        `box`. Returns (cx, cy) image px, or None (caller keeps the box center).
        The small drone is darker than the sky/ground, so weight each pixel by how
        much darker it is than the ROI's low percentile (robust to a loose box +
        a bright background). Camera pixels ONLY -- no cue, no gt: honesty-clean."""
        x, y, w, h = box
        H, W = frame_bgr.shape[:2]
        x0, y0 = int(max(0, math.floor(x))), int(max(0, math.floor(y)))
        x1, y1 = int(min(W, math.ceil(x + w))), int(min(H, math.ceil(y + h)))
        if x1 - x0 < 3 or y1 - y0 < 3:
            return None
        roi = frame_bgr[y0:y1, x0:x1]
        gray = roi.mean(axis=2) if roi.ndim == 3 else roi.astype(np.float64)
        thr = np.percentile(gray, self.subpixel_dark_pctl)
        wgt = np.clip(thr - gray, 0.0, None)      # darker than thr -> positive weight
        tot = float(wgt.sum())
        if tot <= 1e-6:
            return None
        ys, xs = np.mgrid[0:gray.shape[0], 0:gray.shape[1]]
        return (x0 + float((wgt * xs).sum() / tot),
                y0 + float((wgt * ys).sum() / tot))

    def _recenter(self, frame_bgr, box):
        """Shift `box` so its center = the subpixel target centroid, keeping w,h
        (the width-based range is unchanged -- only the bearing is refined)."""
        c = self._subpixel_center(frame_bgr, box)
        if c is None:
            return box
        x, y, w, h = box
        return (c[0] - w / 2.0, c[1] - h / 2.0, w, h)

    # ---- geometry helper: camera-implied target NED from a detection ----
    @staticmethod
    def _cam_implied_ne(det, own):
        pos_n, pos_e, psi = own
        lam = psi + det.bearing_rad
        return (pos_n + det.range_m * math.cos(lam),
                pos_e + det.range_m * math.sin(lam))

    def _cue_gap(self, det, own, cue_pos) -> float:
        cam_n, cam_e = self._cam_implied_ne(det, own)
        return math.hypot(cam_n - cue_pos[0], cam_e - cue_pos[1])

    # ---- is this NN detection trustworthy to SEED the tracker on? ----
    def _seed_ok(self, det) -> bool:
        ctx = self.seed_ctx
        if ctx is None:
            # OFFLINE/DEV ONLY (frame-replay smoke, no guidance loop): accept any
            # NN detection. m4_intercept ALWAYS constructs a SeekerSeedContext
            # under --track, so a live flight never takes this branch.
            return True
        own = ctx.own
        cue_pos = ctx.legal_cue_pos()
        # PRIMARY (pre-handoff): cue-consistency. The real target's camera-implied
        # position agrees with the cue; a phantom (~1.8 m ahead while the cue says
        # ~20 m away) does not.
        if cue_pos is not None and own is not None:
            return self._cue_gap(det, own, cue_pos) <= self.seed_cue_gate_m
        # NO-CUE branch (review finding #2): reached when legal_cue_pos() is None.
        # That happens POST-handoff (the cue channel is closed one-way at the
        # latch) AND -- ADR-0059 -- PRE-handoff when the cue is STALE/jammed and
        # m4_intercept feeds cue_pos=None, so a frozen cue can no longer reject the
        # real target and this camera-track-continuity gate becomes the anti-
        # phantom lever under jam. The re-seed must be consistent with the
        # guidance's own velocity-aware CAMERA-track prediction (cam_track =
        # TargetTracker pos_hat, predict()ed every tick -- camera-corrected only
        # post-latch; pre-handoff under jam it is the DEAD-RECKONED track, carrying
        # the last legitimate cue state forward, the same legality as --coast-
        # search / --warm-handoff). Acceptance radius AGES with coast time at a
        # bounded rate (prediction error grows while coasting), hard-capped.
        # Deliberately NO confidence gate (ADR-0057: confidence is INVERTED --
        # phantoms score HIGHER than the real few-pixel target) and NO gating on
        # the stale last_box pixel location (arbitrary after a coast; it burned
        # weave runs 2/4 with 8 and 7 FALSE post-handoff dets). No legal ref -> coast.
        cam_track = ctx.cam_track
        if cam_track is None or own is None:
            return False
        # Coast time from the RTF-invariant frame count, NOT det.t_mono (wall
        # time -- ADR-0009 violation, review finding #1). frames_since_loss is
        # None until the first loss (-> coast_dt 0), and always >= 0 otherwise.
        coast_dt = (0.0 if self.frames_since_loss is None
                    else self.frames_since_loss * self.frame_dt_s)
        radius = min(self.reacq_cap_m,
                     self.reacq_base_m + self.reacq_rate_m_s * coast_dt)
        cam_n, cam_e = self._cam_implied_ne(det, own)
        ok = math.hypot(cam_n - cam_track[0], cam_e - cam_track[1]) <= radius
        if not ok:
            self.n_reacq_rejected += 1
        return ok

    # ---- during TRACK: should a DISAGREEING NN box replace the tracker? ----
    def _nn_beats_track(self, det, track_box) -> bool:
        """Only pre-handoff, and only when the cue confirms the NN box is the real
        target AND the tracker box has drifted off it. Post-handoff (no cue) this
        is always False -> the tracker keeps its lock and the phantom NN is ignored
        (the anti-phantom rule)."""
        ctx = self.seed_ctx
        own = ctx.own if ctx else None
        cue_pos = ctx.legal_cue_pos() if ctx else None
        if cue_pos is None or own is None:
            return False
        nn_gap = self._cue_gap(det, own, cue_pos)
        tdet = self._box_to_det(track_box, det.t_mono, self.last_score, 1)
        trk_gap = self._cue_gap(tdet, own, cue_pos)
        return nn_gap <= self.seed_cue_gate_m and trk_gap > self.seed_cue_gate_m

    def _new_tracker(self):
        if self.tracker_kind == "MIL":
            return cv2.TrackerMIL_create()
        if self.tracker_kind == "VIT":
            return self._new_vit_tracker()
        return cv2.TrackerCSRT_create()

    def _new_vit_tracker(self):
        """cv2.TrackerVit (OpenCV Zoo vittrack ONNX): a learned Siamese/
        transformer tracker, robust to appearance change frame-to-frame (the
        exact failure mode CSRT hits on the banking/maneuvering target, ADR-
        0076 fix #2). `.update()` returns `(ok, box)` exactly like CSRT/MIL --
        verified empirically (standalone check, not just API-shape) that its
        box also SCALE-ADAPTS (width tracks a growing/shrinking target), so the
        downstream box->range/bearing conversion (_box_to_det et al.) needs no
        changes; it is genuinely a drop-in tracker backend.

        cv2's Python binding surfaces two equivalent spellings for the params-
        constructor overload -- the flat `cv2.TrackerVit_create(params)` and
        the nested `cv2.TrackerVit.create(params)` -- both present in this repo's
        cv2 build (5.0.0). Prefer the flat name; fall back to the nested one so
        this also works on a cv2 build that only exposes one spelling."""
        params = cv2.TrackerVit_Params()
        params.net = self.vit_model_path
        if hasattr(cv2, "TrackerVit_create"):
            return cv2.TrackerVit_create(params)
        return cv2.TrackerVit.create(params)

    def _init_tracker(self, frame_bgr, box_xywh) -> bool:
        """Build + init a NEW tracker; only REPLACE the current lock on success
        (review minor: the old replace-before-init destroyed a live lock when
        init threw, then published the stale box with no tracker behind it)."""
        x, y, w, h = box_xywh
        if w <= 0 or h <= 0:
            return False
        trk = self._new_tracker()
        try:
            trk.init(frame_bgr, (int(round(x)), int(round(y)),
                                 int(round(w)), int(round(h))))
        except cv2.error:
            return False
        self.tracker = trk
        return True

    def _degenerate(self, box, W, H) -> bool:
        x, y, w, h = box
        if w < self.min_box_px or h < self.min_box_px:
            return True
        if w > self.max_box_frac * W or h > self.max_box_frac * H:
            return True
        cx, cy = x + w / 2.0, y + h / 2.0
        if not (0 <= cx <= W and 0 <= cy <= H):
            return True
        return False

    def _touches_border(self, box, W, H) -> bool:
        """True if any edge of `box` sits within border_margin_px of a frame
        edge (audit #37). Disabled (always False) when the margin is <= 0, so
        the default path is byte-identical."""
        if self.border_margin_px <= 0:
            return False
        x, y, w, h = box
        m = self.border_margin_px
        return (x <= m or y <= m
                or (x + w) >= (W - m) or (y + h) >= (H - m))

    def _none(self, t_mono) -> SeekerDetection:
        return SeekerDetection(t_mono, None, None, None, None, None, 0)

    # ------------------------------------------------------------------ main step
    def process(self, frame_bgr, t_mono) -> SeekerDetection:
        H, W = frame_bgr.shape[:2]

        if self.tracker is None:
            # ------------------------------- ACQUIRE (NN, validated) -----------
            # Age the coast clock one frame per ACQUIRE pass (RTF-invariant frame
            # basis; None until the first loss). Feeds _seed_ok's re-acq radius.
            if self.frames_since_loss is not None:
                self.frames_since_loss += 1
            det = self.seeker.detect(frame_bgr, t_mono)
            self.n_nn_frames += 1
            if (det.range_m is not None and det.box_xywh is not None
                    and self._seed_ok(det)):
                if self._init_tracker(frame_bgr, det.box_xywh):
                    self.last_box = tuple(float(v) for v in det.box_xywh)
                    self.last_score = det.decision_margin or 0.0
                    self.since_nn = 0
                    self.frames_since_loss = None   # locked -> coast clock off
                    self.n_seed += 1
                    return det   # publish the validated NN acquisition
            return self._none(t_mono)

        # ----------------------------------- TRACK (CSRT, dense) ---------------
        ok, box = self.tracker.update(frame_bgr)
        self.n_track_frames += 1
        box = tuple(float(b) for b in box)
        if not ok or self._degenerate(box, W, H):
            self.tracker = None
            self.last_box = None       # stale after a loss -- never gate against it
            self.frames_since_loss = 0  # start the RTF-invariant coast clock
            self.n_lost += 1
            return self._none(t_mono)   # coast; ACQUIRE re-seeds next frames

        # Frame-border coast (audit #37): a box clipped by the frame edge yields
        # a truncated-width (under-read) range + biased bearing. COAST -- publish
        # nothing this frame but KEEP the tracker alive (unlike a loss), so
        # tracking resumes cleanly if the box re-enters the interior. Default
        # margin 0 -> _touches_border always False -> byte-identical.
        if self._touches_border(box, W, H):
            self.n_border_coast += 1
            return self._none(t_mono)

        self.since_nn += 1
        # ---- periodic NN re-validation / drift correction ----
        if self.since_nn >= self.revalidate_every:
            self.since_nn = 0
            det = self.seeker.detect(frame_bgr, t_mono)
            self.n_nn_frames += 1
            if det.range_m is not None and det.box_xywh is not None:
                if _iou(box, det.box_xywh) >= self.reseed_iou:
                    # NN agrees with the tracker -> re-seed to the fresh NN box
                    if self._init_tracker(frame_bgr, det.box_xywh):
                        box = tuple(float(v) for v in det.box_xywh)
                        self.last_score = det.decision_margin or self.last_score
                        self.n_reseed += 1
                elif self._nn_beats_track(det, box):
                    # tracker drifted; cue confirms the NN box is the real target
                    if self._init_tracker(frame_bgr, det.box_xywh):
                        box = tuple(float(v) for v in det.box_xywh)
                        self.last_score = det.decision_margin or self.last_score
                        self.n_reseed += 1
                else:
                    self.n_phantom_ignored += 1   # ignore the phantom, keep lock

        self.last_box = box   # tracker box drives gating + next-frame update (unchanged)
        # ADR-0071: publish the bearing off the SUBPIXEL centroid, not the box
        # center (default OFF = byte-identical). Only the published box is
        # recentered; last_box / the tracker / all gating are untouched.
        pub_box = self._recenter(frame_bgr, box) if self.subpixel else box
        return self._box_to_det(pub_box, t_mono, self.last_score, 1)

    def stats_line(self) -> str:
        return (f"[detect-track] seeds={self.n_seed} reseeds={self.n_reseed} "
                f"lost={self.n_lost} track_frames={self.n_track_frames} "
                f"nn_frames={self.n_nn_frames} phantoms_ignored={self.n_phantom_ignored} "
                f"reacq_rejected={self.n_reacq_rejected} border_coast={self.n_border_coast} "
                f"kind={self.tracker_kind} revalidate_every={self.revalidate_every} "
                f"reseed_iou={self.reseed_iou} seed_gate_m={self.seed_cue_gate_m}")
