"""AUTO-CROP high-resolution seeker mode (ADR-0074).

THE PROBLEM (ADR-0076: the ~50% NN-coverage ceiling). The camera is 1280x960
native; the fine-tuned NN runs at imgsz 640, so today's plain `--seeker
markerless` path DOWNSCALES the full frame 1280->640 before every inference
call. A far/small/edge-on/banking target is already only ~12-46 native px --
downscaling by 2x shrinks it further, and (ADR-0076 add #3) a 640-trained
model fed a scale it never saw in training detects noisier.

THE FIX. Once we have ANY recent detection, we already know roughly where on
the SENSOR the target sits. Instead of downscaling the whole frame, cut a
640x640 NATIVE-resolution window around that last-known pixel and run the
(crop-trained) NN on THAT -- no downscale, ~2x the native pixels on target
vs. the full-frame path. If there's no recent detection to center on, fall
back to the ordinary full-frame path to RE-ACQUIRE (a crop centered on a
stale/absent pixel could crop the real target out of frame entirely).

CONVENTION (ADR-0074 -- MUST match the TRAINING capture side,
render_sim_dataset_crop.py, both built on crop_geom.py so they cannot drift
apart):
  - crop is 640x640 NATIVE px cut from the 1280x960 frame, CLAMPED fully
    inside the frame bounds (shifted, never padded) -- crop_geom.crop_bounds.
  - TRAINING centers the crop on the gt-projected target pixel (offline,
    label-legal -- same boundary as render_sim_dataset_banked.py).
  - INFERENCE (this file) centers the crop on the LAST DETECTION's full-frame
    pixel -- camera-only, honesty-clean; NEVER gt_*.
  - a detection box returned in CROP-LOCAL coords is mapped back to
    FULL-FRAME coords (crop_geom.box_crop_to_full, i.e. + the crop x0,y0
    offset) BEFORE any bearing/range math -- done by handing the full-frame
    box to the wrapped seeker's own `box_to_detection()`, the SAME conversion
    the plain full-frame path and the detect-then-track tracker both already
    use (finetuned_seeker.FinetunedNNSeeker.box_to_detection).

STALENESS: if the last detection is >= MARKERLESS_AUTOCROP_STALE_N frames old
(default 5) -- or there has never been one -- `detect()` runs the wrapped
seeker's normal full-frame path instead of cropping, to RE-ACQUIRE.

DEFAULT OFF: `MARKERLESS_AUTOCROP` defaults to "0". When off, `detect()` is a
PURE PASSTHROUGH -- `return self.seeker.detect(frame_bgr, t_mono)`, the exact
same call with the exact same arguments markerless_loop.py made before this
wrapper existed -- so wrapping every seeker in an AutoCropSeeker unconditionally
is byte-identical to not wrapping it at all.

HONESTY (CLAUDE.md / ADR-0010): the only state this wrapper keeps is the last
CAMERA detection's own full-frame pixel (from SeekerDetection.box_xywh, which
itself came from live camera pixels) -- never gt_*. Same boundary as
detect_track.py's tracker state.

Drop-in seam: exposes `.detect(frame_bgr, t_mono) -> SeekerDetection` (same
contract as any seeker) AND `.box_to_detection(...)` (delegated straight to
the wrapped seeker), so an AutoCropSeeker can itself be handed to
detect_track.DetectThenTracker exactly like a bare seeker -- the tracker's
own ACQUIRE/re-validate calls to `.detect()` transparently go through the
crop logic too.
"""
from __future__ import annotations

import os
from typing import Optional, Tuple

from crop_geom import DEFAULT_CROP_SIZE, box_crop_to_full, crop_frame
from nn_seeker import SeekerDetection


def _env_bool(name: str, default: bool = False) -> bool:
    v = os.environ.get(name)
    if v is None:
        return default
    return v.strip().lower() not in ("0", "", "false", "no", "off")


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return int(default)


class AutoCropSeeker:
    """Wraps any seeker exposing `.detect(frame_bgr, t_mono) -> SeekerDetection`
    with the AUTO-CROP high-res mode (ADR-0074).

    Parameters
    ----------
    seeker : the wrapped detector (e.g. finetuned_seeker.FinetunedNNSeeker,
        or two_stage_seeker.TwoStageSeeker). Only actually CROPS if it also
        exposes `.raw_boxes_crop(crop_bgr) -> (score, box_xywh_or_None)` and
        `.box_to_detection(box_xywh, t_mono, score, n_hits, class_name=...)`
        (FinetunedNNSeeker has both); otherwise AUTO-CROP degrades to a
        passthrough even when enabled (loud one-time warning), since there is
        no honest way to crop-infer through a seeker that never exposed the
        raw-box seam.
    crop_size : native px, default 640 (matches the NN's imgsz and the
        training capture's crop, render_sim_dataset_crop.py).
    stale_n : MARKERLESS_AUTOCROP_STALE_N env override, default 5.
    enabled : MARKERLESS_AUTOCROP env override (falsy/"0"/unset -> OFF).
    """

    def __init__(self, seeker, crop_size: int = DEFAULT_CROP_SIZE,
                 stale_n: Optional[int] = None, enabled: Optional[bool] = None):
        self.seeker = seeker
        self.crop_size = crop_size
        self.stale_n = (stale_n if stale_n is not None
                        else _env_int("MARKERLESS_AUTOCROP_STALE_N", 5))
        self.enabled = (_env_bool("MARKERLESS_AUTOCROP", False)
                        if enabled is None else bool(enabled))
        self._crop_capable = (hasattr(seeker, "raw_boxes_crop")
                              and hasattr(seeker, "box_to_detection"))
        # last-detection FULL-FRAME pixel (u, v) -- camera-only state, never gt_*
        self._last_px: Optional[Tuple[float, float]] = None
        self._age: Optional[int] = None   # detect() calls since the last hit
        # diagnostics (read by the offline smoke test / can be logged on stop)
        self.n_crop_frames = 0
        self.n_full_frames = 0

        if self.enabled and not self._crop_capable:
            print(f"[auto-crop] MARKERLESS_AUTOCROP=1 but {type(seeker).__name__} "
                  "has no raw_boxes_crop()/box_to_detection() seam -- AUTO-CROP "
                  "has no honest way to crop-infer through it; falling back to "
                  "the full-frame path every call (behaves as if OFF).")
        elif self.enabled:
            print(f"[auto-crop] ON: crop={self.crop_size}x{self.crop_size} "
                  f"native px, stale_n={self.stale_n} detect() calls, "
                  f"wrapping {type(seeker).__name__}")

    # ---- drop-in seam for detect_track.DetectThenTracker -------------------
    def box_to_detection(self, *args, **kwargs):
        return self.seeker.box_to_detection(*args, **kwargs)

    # ---- internal bookkeeping -----------------------------------------------
    def _fresh(self) -> bool:
        return (self._last_px is not None and self._age is not None
                and self._age < self.stale_n)

    def _remember_full_frame_det(self, det: SeekerDetection) -> None:
        """Update last-detection state from a detection already in FULL-FRAME
        coords (either the plain full-frame path's own output, or a crop
        detection already mapped back to full-frame by `detect()`)."""
        if det.range_m is not None and det.box_xywh is not None:
            x, y, w, h = det.box_xywh
            self._last_px = (x + w / 2.0, y + h / 2.0)
            self._age = 0
        elif self._age is not None:
            self._age += 1
        # else: _age stays None (never had a detection yet) -- _fresh() is
        # False either way, so the next call takes the full-frame branch.

    # ---- main step -----------------------------------------------------------
    def detect(self, frame_bgr, t_mono: Optional[float] = None) -> SeekerDetection:
        if not self.enabled or not self._crop_capable:
            # BYTE-IDENTICAL passthrough: the exact call markerless_loop.py
            # made before this wrapper existed.
            return self.seeker.detect(frame_bgr, t_mono)

        if self._fresh():
            crop, x0, y0 = crop_frame(frame_bgr, self._last_px[0], self._last_px[1],
                                      self.crop_size)
            self.n_crop_frames += 1
            score, crop_box = self.seeker.raw_boxes_crop(crop)
            if crop_box is None:
                det = self._none(t_mono)
                self._remember_full_frame_det(det)   # ages the staleness clock
                return det
            full_box = box_crop_to_full(crop_box, x0, y0)
            det = self.seeker.box_to_detection(full_box, t_mono, score, 1,
                                               class_name="drone_crop")
            self._remember_full_frame_det(det)
            return det

        # stale / never detected -- re-acquire via the ordinary full-frame path
        det = self.seeker.detect(frame_bgr, t_mono)
        self.n_full_frames += 1
        self._remember_full_frame_det(det)
        return det

    @staticmethod
    def _none(t_mono) -> SeekerDetection:
        return SeekerDetection(t_mono, None, None, None, None, None, 0)

    def stats_line(self) -> str:
        return (f"[auto-crop] crop_frames={self.n_crop_frames} "
                f"full_frames={self.n_full_frames} enabled={self.enabled} "
                f"crop_capable={self._crop_capable} crop_size={self.crop_size} "
                f"stale_n={self.stale_n}")
