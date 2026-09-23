#!/usr/bin/env python3
"""Generate the tiled ground-grid texture for worlds/apriltag_demo.sdf
(portfolio demo polish: a flat single-color ground plane gives no sense of
speed/motion/parallax when the drones cross it -- builder feedback).

WHY A PRE-BAKED TILED PNG (not a `<uv_scale>` or material-script trick):
SDF's <geometry><plane> element only has <normal> and <size> (checked
against the installed sdformat14 1.9 schema, /usr/share/sdformat14/1.9/
plane_shape.sdf -- no UV-tiling child element exists), and gz-sim's PBR
<material> has no scale/repeat field either -- a single <albedo_map> always
maps UV [0,1] across the WHOLE plane. So a "5 m grid on a 500 m plane"
needs the repetition BAKED INTO THE TEXTURE PIXELS themselves, not
requested via a scale factor. This script draws N x N grid cells directly
onto one big texture: cell_size_m = plane_size_m / n_cells, and this file's
constants must stay in sync with the ground_plane <geometry><plane><size>
in worlds/apriltag_demo.sdf (500 m) if that ever changes.

Output: models/demo_ground_grid/grid_ground.png (referenced by
worlds/apriltag_demo.sdf's ground_plane visual as
model://demo_ground_grid/grid_ground.png). Demo-world-only asset -- the
gated worlds/apriltag.sdf ground plane is untouched.
"""
import os

import numpy as np
from PIL import Image

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(REPO_ROOT, "models", "demo_ground_grid")
OUT_PATH = os.path.join(OUT_DIR, "grid_ground.png")

PLANE_SIZE_M = 500.0   # must match worlds/apriltag_demo.sdf ground_plane <size>
GRID_SPACING_M = 5.0   # subtle "survey grid" spacing -- visible near the drones
                        # (engagement corridor ~x:[0,15] y:[-30,5]) without being
                        # so fine it moires at distance.
TEXTURE_PX = 2048       # -> px_per_cell = 2048 / (500/5) = ~20.5 px/cell, plenty
                         # for a 1-2 px line with no visible aliasing up close.

# Muted field-green base (matches the world file's existing flat ambient/
# diffuse ~0.28-0.38 green) with a slightly darker grid line -- subtle, not
# a garish checkerboard.
BASE_RGB = (74, 87, 51)     # ~ (0.29, 0.34, 0.20) * 255, matches old diffuse
LINE_RGB = (54, 64, 38)     # darker green, low contrast on purpose
LINE_PX = 2


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    n_cells = round(PLANE_SIZE_M / GRID_SPACING_M)
    px_per_cell = TEXTURE_PX / n_cells

    img = np.empty((TEXTURE_PX, TEXTURE_PX, 3), dtype=np.uint8)
    img[:, :] = BASE_RGB

    for i in range(n_cells + 1):
        pos = round(i * px_per_cell)
        if pos >= TEXTURE_PX:
            continue
        lo = max(0, pos - LINE_PX // 2)
        hi = min(TEXTURE_PX, lo + LINE_PX)
        img[lo:hi, :] = LINE_RGB
        img[:, lo:hi] = LINE_RGB

    Image.fromarray(img, mode="RGB").save(OUT_PATH)
    print(f"[gen_demo_ground_texture] wrote {OUT_PATH} "
          f"({TEXTURE_PX}x{TEXTURE_PX}px, {n_cells}x{n_cells} cells, "
          f"{GRID_SPACING_M} m/cell over a {PLANE_SIZE_M} m plane)")


if __name__ == "__main__":
    main()
