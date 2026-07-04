#!/usr/bin/env python3
"""One-off utility: build the AprilTag 36h11 id-0 texture used by the
models/apriltag_target Gazebo model. See GOALS.md milestone M2.

What: downloads the canonical tag36h11 id-0 PNG from AprilRobotics/apriltag-imgs
(the reference repo pupil-apriltags itself is generated from) and upscales it
nearest-neighbor to 512x512 so Gazebo's renderer has enough texture resolution
to shade it crisply from a few meters away, without ever blurring the crisp
black/white edges the detector depends on (any interpolation --INTER_LINEAR/
CUBIC-- would smear bit edges and hurt detection).

CRITICAL GEOMETRY FACT (also documented in models/apriltag_target/model.sdf
and scripts/m2_detect.py -- Fable reviews this number):
The source PNG is the FULL textured tag image, 10x10 cells:
    - 6x6 cells of data bits (the actual tag36h11 payload)
    - +1 cell of black border on each side  -> 8x8
    - +1 cell of white border on each side   -> 10x10
So the BLACK SQUARE (the thing pupil-apriltags' `tag_size` parameter measures
-- the outer edge of the black border, which is what the detector locks onto)
is 8/10 = 0.8 of the full textured square's edge length. If the textured
plane (this PNG, stretched over the model's visual geometry) is built at
0.625 m per side, then:
    tag_size = 0.625 m * 0.8 = 0.5 m
Those are the exact numbers used in models/apriltag_target/model.sdf (0.625 m
plane) and scripts/m2_detect.py (tag_size=0.5). Verified empirically: the
source PNG is confirmed 10x10 px below before resizing (see printed shape).

Run manually (writes models/apriltag_target/tag36h11_00000.png):

    .venv/bin/python scripts/make_tag_texture.py
"""

import os
import sys
import urllib.request

import cv2
import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH = os.path.join(
    REPO_ROOT, "models", "apriltag_target", "tag36h11_00000.png"
)
SOURCE_URL = (
    "https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/"
    "master/tag36h11/tag36_11_00000.png"
)
OUT_SIZE = 512
EXPECTED_SOURCE_SIZE = 10  # 6x6 data + 1-cell black border + 1-cell white border


def main():
    print(f"[make_tag_texture] Downloading {SOURCE_URL}")
    with urllib.request.urlopen(SOURCE_URL, timeout=30) as resp:
        raw = resp.read()
    print(f"[make_tag_texture] Downloaded {len(raw)} bytes")

    arr = np.frombuffer(raw, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
    if img is None:
        print("[make_tag_texture] FAILED: could not decode downloaded PNG")
        return 1

    h, w = img.shape[:2]
    print(f"[make_tag_texture] Source image: {w}x{h}, {img.shape[2]} channels")
    if (w, h) != (EXPECTED_SOURCE_SIZE, EXPECTED_SOURCE_SIZE):
        print(
            f"[make_tag_texture] FAILED: expected a {EXPECTED_SOURCE_SIZE}x"
            f"{EXPECTED_SOURCE_SIZE} source (6x6 data + 2x border cells each "
            f"side), got {w}x{h}. Geometry math in the docstring / model.sdf "
            "would need re-deriving -- do not silently proceed."
        )
        return 1

    # Nearest-neighbor only: preserves hard bit edges the detector needs.
    # 512 / 10 = 51.2, not an integer multiple, so block widths are uneven
    # (51 or 52 px) -- still a clean nearest-neighbor upscale, just not
    # perfectly uniform blocks. Detection doesn't care; it's a printed/
    # rendered fiducial, not a per-pixel grid reader.
    upscaled = cv2.resize(
        img, (OUT_SIZE, OUT_SIZE), interpolation=cv2.INTER_NEAREST
    )

    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    cv2.imwrite(OUT_PATH, upscaled)
    print(f"[make_tag_texture] Wrote {OUT_SIZE}x{OUT_SIZE} texture -> {OUT_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
