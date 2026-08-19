#!/usr/bin/env python3
"""Sim-free verification for the AUTO-CROP high-resolution seeker mode
(ADR-0074). NO Gazebo/PX4, NO onnxruntime -- exercises crop_geom.py and
auto_crop_seeker.AutoCropSeeker directly against synthetic frames and fake
seeker test doubles.

Checks (per the ADR-0074 build brief):
  (a) the crop function cuts a 640x640 window centered (and CLAMPED near an
      edge) on a known pixel, and the target is ~2x bigger fraction-of-frame
      in the native crop than in today's downscaled full-frame path.
  (b) a box in CROP coords maps to FULL-FRAME coords correctly (and back).
  (c) MARKERLESS_AUTOCROP=0 (the default) is a byte-identical passthrough --
      same frame object, same t_mono, same return object, seeker.detect()
      called exactly once, raw_boxes_crop() never touched.
  (d) (bonus) MARKERLESS_AUTOCROP=1 exercises the crop branch and the
      staleness fallback back to full-frame.

Run: .venv-seeker/bin/python scripts/seeker/verify_autocrop.py
"""
from __future__ import annotations

import os
import sys

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from crop_geom import (  # noqa: E402
    DEFAULT_CROP_SIZE, crop_frame, box_crop_to_full, box_full_to_crop,
)
from nn_seeker import SeekerDetection            # noqa: E402
from auto_crop_seeker import AutoCropSeeker      # noqa: E402
from finetuned_seeker import _letterbox          # noqa: E402 (production letterbox, reused for the fair comparison in (a3))

FRAME_W, FRAME_H = 1280, 960


def _make_frame_with_blob(cx, cy, radius=10, frame_w=FRAME_W, frame_h=FRAME_H):
    """A mid-gray frame with a dark circular blob centered at (cx, cy)."""
    frame = np.full((frame_h, frame_w, 3), 200, dtype=np.uint8)
    cv2.circle(frame, (int(round(cx)), int(round(cy))), radius, (20, 20, 20), -1)
    return frame


def _dark_blob_width_px(img, thresh=100):
    """Bounding-box WIDTH (px) of the dark region -- the LINEAR pixel extent
    a detector's box would report (what feeds the known-size range formula,
    fx*span/box_width_px), not its area. Native-res halves the letterbox
    scale factor -> this should be ~2x, whereas raw dark-pixel AREA would be
    ~4x (area scales with the square of a linear factor) -- the metric the
    ADR-0074 "~2x the pixels" claim means is this linear one."""
    gray = img.mean(axis=2)
    cols = np.where((gray < thresh).any(axis=0))[0]
    if cols.size == 0:
        return 0
    return int(cols.max() - cols.min() + 1)


def check(name, cond, detail=""):
    status = "PASS" if cond else "FAIL"
    print(f"[{status}] {name}" + (f" -- {detail}" if detail and not cond else ""))
    return cond


def main() -> int:
    failures = 0

    # ---------------------------------------------------------------- (a)
    print("=== (a) crop framing + clamping + native-res pixel-density gain ===")
    # (a1) a CENTRAL blob -> crop is exactly crop_size and centered on it.
    cx, cy = 700.0, 500.0
    frame = _make_frame_with_blob(cx, cy, radius=10)
    crop, x0, y0 = crop_frame(frame, cx, cy, DEFAULT_CROP_SIZE)
    ok = crop.shape[:2] == (DEFAULT_CROP_SIZE, DEFAULT_CROP_SIZE)
    failures += not check("crop is 640x640", ok, f"got {crop.shape}")
    expect_x0, expect_y0 = int(round(cx - 320)), int(round(cy - 320))
    ok = (x0, y0) == (expect_x0, expect_y0)
    failures += not check("central crop is centered on the blob pixel", ok,
                          f"got ({x0},{y0}) expected ({expect_x0},{expect_y0})")

    # (a2) an EDGE-adjacent blob (top-right corner) -> crop CLAMPED fully
    # inside the frame -- shifted, never padded.
    ex, ey = 1270.0, 15.0
    eframe = _make_frame_with_blob(ex, ey, radius=10)
    ecrop, ex0, ey0 = crop_frame(eframe, ex, ey, DEFAULT_CROP_SIZE)
    ok = ecrop.shape[:2] == (DEFAULT_CROP_SIZE, DEFAULT_CROP_SIZE)
    failures += not check("edge crop is STILL 640x640 (shifted, never padded)", ok,
                          f"got {ecrop.shape}")
    ok = (ex0, ey0) == (FRAME_W - DEFAULT_CROP_SIZE, 0)
    failures += not check("edge crop clamped fully inside the frame bounds", ok,
                          f"got ({ex0},{ey0}) expected "
                          f"({FRAME_W - DEFAULT_CROP_SIZE}, 0)")
    blob_in_crop = (ex0 <= ex - 10 and ex + 10 <= ex0 + DEFAULT_CROP_SIZE
                    and ey0 <= ey - 10 and ey + 10 <= ey0 + DEFAULT_CROP_SIZE)
    failures += not check("the blob is fully inside the clamped crop", blob_in_crop)

    # (a3) native-res pixel-density gain: the SAME blob's LINEAR pixel extent
    # (bounding-box width -- what feeds the known-size range formula,
    # fx*span/box_width_px) is ~2x bigger, and its fraction-of-frame width is
    # ~2x, in the native 640 crop vs. today's downscaled 640 full-frame path
    # (1280x960 -letterbox-> 640x640, r=min(640/960,640/1280)=0.5 -- the
    # EXACT production letterbox finetuned_seeker._letterbox applies, reused
    # here for a fair comparison). NOTE: raw pixel-COUNT/area would show ~4x
    # (area scales with the square of the linear factor) -- that is a
    # correct but different number from the "~2x the pixels" (linear, box-
    # width) claim this mode is built on, so we check the linear metric.
    down, r, _ = _letterbox(frame, DEFAULT_CROP_SIZE)
    down_w = _dark_blob_width_px(down)
    crop_w = _dark_blob_width_px(crop)
    down_frac = down_w / down.shape[1]
    crop_frac = crop_w / crop.shape[1]
    ratio = crop_frac / down_frac if down_frac else float("nan")
    ok = 1.8 <= ratio <= 2.2   # ~2x, small slack for pixel discretization
    failures += not check(
        "native crop blob's fraction-of-frame WIDTH is ~2x the downscaled "
        "full-frame path's",
        ok, f"crop_w_px={crop_w} ({crop_frac:.4f} of frame) "
            f"down_w_px={down_w} ({down_frac:.4f} of frame) "
            f"ratio={ratio:.2f} (letterbox r={r}); "
            f"[note: raw pixel-AREA ratio would read ~4x, not ~2x, since "
            f"area scales with the square of the linear factor]")

    # ---------------------------------------------------------------- (b)
    print("\n=== (b) crop-local <-> full-frame box coordinate mapping ===")
    full = box_crop_to_full((10, 10, 20, 20), 300, 200)
    failures += not check(
        "crop-local (10,10,20,20) + offset (300,200) -> full-frame (310,210,20,20)",
        full == (310, 210, 20, 20), f"got {full}")
    back = box_full_to_crop(full, 300, 200, DEFAULT_CROP_SIZE)
    failures += not check("mapping back full-frame -> crop-local recovers the original",
                          back == (10, 10, 20, 20), f"got {back}")
    clipped = box_full_to_crop((930, 210, 30, 10), 300, 200, DEFAULT_CROP_SIZE)
    failures += not check("a box straddling the crop's right edge is CLIPPED to it",
                          clipped == (630.0, 10.0, 10.0, 10.0), f"got {clipped}")
    outside = box_full_to_crop((0, 0, 5, 5), 300, 200, DEFAULT_CROP_SIZE)
    failures += not check("a box entirely outside the crop -> None",
                          outside is None, f"got {outside}")

    # ---------------------------------------------------------------- (c)
    print("\n=== (c) MARKERLESS_AUTOCROP=0 (default) is byte-identical ===")
    os.environ.pop("MARKERLESS_AUTOCROP", None)   # ensure unset -> default OFF

    class _FakeSeeker:
        """Records exactly what detect()/raw_boxes_crop() were called with."""
        def __init__(self):
            self.detect_calls = []
            self.crop_calls = []
            self._next = SeekerDetection(1.0, 5.0, 0.1, 0.0, np.array([1, 2, 5]),
                                        0.9, 1, (100, 100, 20, 20), "drone")

        def detect(self, frame_bgr, t_mono=None):
            self.detect_calls.append((frame_bgr, t_mono))
            return self._next

        def raw_boxes_crop(self, crop_bgr):
            self.crop_calls.append(crop_bgr)
            return 0.8, (5, 5, 20, 20)

        def box_to_detection(self, box_xywh, t_mono, score, n_hits, class_name="track"):
            return SeekerDetection(t_mono, 4.0, 0.0, 0.0, np.array([0, 0, 4]),
                                   score, n_hits, box_xywh, class_name)

    seeker = _FakeSeeker()
    acs = AutoCropSeeker(seeker)   # no explicit enabled= -> reads env (unset -> OFF)
    failures += not check("AutoCropSeeker.enabled is False with the env unset",
                          not acs.enabled)

    probe_frame = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    ret = acs.detect(probe_frame, t_mono=42.0)
    failures += not check("exactly one call reached the wrapped seeker.detect()",
                          len(seeker.detect_calls) == 1,
                          f"got {len(seeker.detect_calls)}")
    failures += not check("the SAME frame object (no crop / no copy) was passed through",
                          seeker.detect_calls[0][0] is probe_frame)
    failures += not check("t_mono passed through unchanged",
                          seeker.detect_calls[0][1] == 42.0)
    failures += not check("raw_boxes_crop was NEVER called while OFF",
                          len(seeker.crop_calls) == 0)
    failures += not check("the return value IS the seeker's own object (no rewrapping)",
                          ret is seeker._next)

    acs.detect(probe_frame, t_mono=43.0)   # a 2nd call -- OFF stays OFF regardless of history
    failures += not check("still a pure passthrough on a 2nd call (no state leak)",
                          len(seeker.crop_calls) == 0 and len(seeker.detect_calls) == 2)

    # ---------------------------------------------------------------- (d)
    print("\n=== (d) bonus: MARKERLESS_AUTOCROP=1 crop path + staleness fallback ===")
    seeker2 = _FakeSeeker()
    acs2 = AutoCropSeeker(seeker2, enabled=True, stale_n=3)
    frame2 = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)

    acs2.detect(frame2, t_mono=1.0)   # no prior detection -> full-frame branch
    failures += not check("first call (no prior detection) takes the full-frame branch",
                          len(seeker2.detect_calls) == 1 and len(seeker2.crop_calls) == 0)
    failures += not check("last-detection pixel recorded from the full-frame hit "
                          "(box (100,100,20,20) -> center (110,110))",
                          acs2._last_px == (110.0, 110.0), f"got {acs2._last_px}")

    acs2.detect(frame2, t_mono=2.0)   # fresh -> crop branch
    failures += not check("second call (fresh) takes the CROP branch",
                          len(seeker2.crop_calls) == 1 and len(seeker2.detect_calls) == 1)
    failures += not check("the crop handed to raw_boxes_crop is 640x640 native",
                          seeker2.crop_calls[0].shape[:2]
                          == (DEFAULT_CROP_SIZE, DEFAULT_CROP_SIZE))

    class _MissSeeker(_FakeSeeker):
        def raw_boxes_crop(self, crop_bgr):
            self.crop_calls.append(crop_bgr)
            return 0.0, None

    seeker3 = _MissSeeker()
    acs3 = AutoCropSeeker(seeker3, enabled=True, stale_n=2)
    acs3.detect(frame2, t_mono=1.0)   # full-frame hit -> last_px set, age=0
    acs3.detect(frame2, t_mono=2.0)   # fresh (age 0<2) -> crop, miss -> age=1
    acs3.detect(frame2, t_mono=3.0)   # fresh (age 1<2) -> crop, miss -> age=2
    acs3.detect(frame2, t_mono=4.0)   # stale (age 2>=2) -> full-frame re-acquire
    failures += not check("staleness fallback: after stale_n misses, re-acquires "
                          "via the full-frame path",
                          len(seeker3.detect_calls) == 2 and len(seeker3.crop_calls) == 2,
                          f"detect_calls={len(seeker3.detect_calls)} "
                          f"crop_calls={len(seeker3.crop_calls)}")

    print(f"\n{'ALL PASS' if failures == 0 else f'{failures} FAILURE(S)'}")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
