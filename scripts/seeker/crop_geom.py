"""Shared 640x640 NATIVE-RESOLUTION crop convention (ADR-0074).

WHY THIS IS ITS OWN FILE: the AUTO-CROP high-res seeker mode needs the EXACT
SAME crop math on both sides of the boundary --

  * TRAINING capture (render_sim_dataset_crop.py) cuts a native-res crop
    around the gt-projected target pixel (offline, label-legal) and writes
    the crop + its remapped box as a training example.
  * INFERENCE (auto_crop_seeker.AutoCropSeeker) cuts the SAME-shaped crop
    around the last camera DETECTION's pixel (no gt, honesty-clean) and maps
    the NN's returned box back to full-frame pixels.

If those two crop conventions ever drifted apart (different clamping, a
padded vs. shifted edge case, an off-by-one in the offset), the crop-trained
model would be fed slightly-different-looking crops at inference than it was
trained on -- a silent accuracy regression that would look like "the model
just isn't very good" rather than "the two crop windows don't match". Keeping
the geometry in ONE module that both sides import removes that failure mode
by construction.

CONVENTION: a `crop_size` x `crop_size` (default 640) native-pixel window cut
from the full frame (no resize, no letterbox), CLAMPED so it sits fully
INSIDE the frame -- i.e. SHIFTED near an edge, never padded. Native px only:
this module never resizes anything, so there is no scale factor to track.

Pure stdlib arithmetic (no numpy/cv2 import at module level) so it is
importable from ANY venv on either side of the boundary (the sim-capture
venv and the flight/.venv-seeker inference venv both already have whatever
they need for their own frame arrays; this module doesn't add a dependency).
"""
from __future__ import annotations

from typing import Optional, Tuple

DEFAULT_CROP_SIZE = 640


def crop_bounds(cx: float, cy: float, frame_w: int, frame_h: int,
                crop_size: int = DEFAULT_CROP_SIZE) -> Tuple[int, int]:
    """Top-left (x0, y0) integer pixel of a `crop_size` x `crop_size` window
    centered on (cx, cy), CLAMPED fully inside [0, frame_w) x [0, frame_h) --
    shifted, never padded. Raises if `crop_size` cannot fit inside the frame
    at all (a loud misconfiguration, not a silent negative-origin window)."""
    if frame_w < crop_size or frame_h < crop_size:
        raise ValueError(
            f"crop_size {crop_size} exceeds frame {frame_w}x{frame_h} -- "
            "cannot clamp a crop that big fully inside the frame")
    half = crop_size / 2.0
    x0 = cx - half
    y0 = cy - half
    x0 = max(0.0, min(x0, float(frame_w - crop_size)))
    y0 = max(0.0, min(y0, float(frame_h - crop_size)))
    return int(round(x0)), int(round(y0))


def crop_frame(frame, cx: float, cy: float, crop_size: int = DEFAULT_CROP_SIZE):
    """Cut the `crop_size` x `crop_size` window out of `frame` (a (H, W[, C])
    array -- numpy, but duck-typed: anything slice-able with frame.shape[:2]
    works), centered (clamped) on (cx, cy). Returns (crop, x0, y0)."""
    h, w = frame.shape[:2]
    x0, y0 = crop_bounds(cx, cy, w, h, crop_size)
    return frame[y0:y0 + crop_size, x0:x0 + crop_size], x0, y0


def box_crop_to_full(box_xywh: Tuple[float, float, float, float],
                      x0: int, y0: int) -> Tuple[float, float, float, float]:
    """Map a detection box in CROP-LOCAL pixel coords -> FULL-FRAME pixel
    coords (add the crop offset). box_xywh = (x, y, w, h), top-left."""
    x, y, w, h = box_xywh
    return (x + x0, y + y0, w, h)


def box_full_to_crop(
        box_xywh: Tuple[float, float, float, float], x0: int, y0: int,
        crop_size: int = DEFAULT_CROP_SIZE
) -> Optional[Tuple[float, float, float, float]]:
    """Map a box in FULL-FRAME pixel coords -> CROP-LOCAL pixel coords
    (subtract the crop offset), CLIPPED to the crop bounds. Returns None if
    the box has no positive-area overlap with the crop after clipping (used
    by the TRAINING capture side: a target whose box doesn't survive the crop
    should not be written as a label)."""
    x, y, w, h = box_xywh
    bx0, by0 = x - x0, y - y0
    bx1, by1 = bx0 + w, by0 + h
    cbx0, cby0 = max(0.0, bx0), max(0.0, by0)
    cbx1, cby1 = min(float(crop_size), bx1), min(float(crop_size), by1)
    bw, bh = cbx1 - cbx0, cby1 - cby0
    if bw <= 0 or bh <= 0:
        return None
    return (cbx0, cby0, bw, bh)
