#!/usr/bin/env python3
"""Generate the PRINT-READY AprilTag placard PDFs -- at true, verified scale.

    .venv/bin/python scripts/print_artifacts/gen_placard_pdf.py            # all variants + verify
    .venv/bin/python scripts/print_artifacts/gen_placard_pdf.py --verify-only

WHAT IT MAKES (into hardware/prints/):
    placard_tag36h11_id0_tiled_a4.pdf       home printer, A4,     1 instructions page + N tiles
    placard_tag36h11_id0_tiled_letter.pdf   home printer, Letter, same
    placard_tag36h11_id0_poster_a1.pdf      copy shop, A1 594x841 mm  <-- the recommended order
    placard_tag36h11_id0_poster_arch24x36.pdf  copy shop, US 24"x36"
    placard_tag36h11_id0_poster_bleed480.pdf   480x480 mm sheet, 15 mm bleed + crop marks
    placard_tag36h11_id0_poster_exact450.pdf   exactly 450x450 mm, artwork only

THE DIMENSION THAT MATTERS, AND WHICH ONE 0.35 m IS
---------------------------------------------------
`0.35 m` is the **AprilTag BLACK SQUARE edge** -- the outer edge of the tag's
1-cell black border, which is exactly the quantity `pupil_apriltags` is handed
as `tag_size`. It is NOT the printed sheet. Traced:

  * docs/placard_mount.md:68   "Tag **black-square** edge | **0.350 m**"
  * docs/placard_mount.md:95-98
        cell size             = 0.350 m / 8      = 43.75 mm
        minimum printed sheet = 0.350 m x 10/8   = 437.5 mm
        recommended sheet     = 450 mm  (50 mm white all round = 1.14 quiet-zone cells)
  * scripts/seeker/tripod_score.py:148-151  "# ADOPTED placard: tag36h11
        black-square edge 0.350 m on a 0.450 m sheet ... that pupil-apriltags'
        `tag_size` measures -- NOT the printed sheet."
        (DEFAULT_TAG_SIZE_M = 0.35, tripod_score.py:160)
  * scripts/make_tag_texture.py header: the source PNG is 10x10 cells
        (6x6 data + 1 black border + 1 white border), so the black square is
        8/10 of the full textured image -- the same 0.8 factor the sim uses
        (0.625 m plane -> tag_size 0.5 m).

So this generator draws a 350.000 mm black square (8 cells @ 43.75 mm) centred
on a 450.000 mm sheet, with the tag36h11 id-0 bit pattern inside it. The 50 mm
white ring is the decoder's QUIET ZONE: nothing -- no crop mark, no ruler, no
label -- is ever printed inside the 450 mm sheet.

ART SOURCE. The bit pattern is read from models/apriltag_target/tag36h11_00000.png
(the canonical AprilRobotics id-0 image, nearest-neighbour upscaled to 512 px by
scripts/make_tag_texture.py) by sampling the centre of each of the 10x10 cells.
It is then re-drawn as VECTOR rectangles at true size -- the 512 px bitmap is
never scaled onto the page, so there is no resampling error and no cell-edge
blur. The 512 px texture is not an exact multiple of 10 (51.2 px/cell), which is
precisely why re-drawing beats scaling the bitmap.

INK ORDER. One solid black 350 mm square is laid down first and the WHITE data
cells are painted on top. Two reasons: (1) the black border ring and the black
data cells never meet at an abutting vector seam, so no RIP can leave a hairline
white line through the tag; (2) the outer black edge -- the only dimension that
feeds `tag_size` -- is a single primitive, so it cannot drift by accumulated
cell rounding.

SCALE VERIFICATION (why this script refuses to just trust itself)
----------------------------------------------------------------
A mis-scaled placard does not fail loudly: the tag still decodes, curve (a)
still plots, and every tag-recovered range is quietly wrong by the scale error
-- the "instruments are evidence" failure class (CLAUDE.md 2026-07-25). So after
writing each PDF this script RE-OPENS THE FILE, inflates the page content
streams, walks the drawing operators, and measures the black square in
millimetres (scripts/print_artifacts/pdf_measure.py). Every page must land
within --tol-mm (default 0.5 mm) or the script exits non-zero and the PDF is
reported FAILED. On the tiled variants each tile is checked against its own
expected clipped extent, and the tiles' art-space coverage must reassemble to
the full 350 mm square with no gap.

That check proves the FILE is right. It cannot prove your PRINTER is right --
which is what the printed 200.0 mm / 100.0 mm verification bars are for, and
why the last assembly step is "measure the black square, expect 350.0 mm".
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use("pdf")
matplotlib.rcParams["pdf.fonttype"] = 42          # embed TrueType; glyphs are not fills
matplotlib.rcParams["pdf.compression"] = 6
import matplotlib.pyplot as plt                                   # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages              # noqa: E402
from matplotlib.patches import Rectangle                          # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pdf_measure                                                # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TAG_PNG = os.path.join(REPO, "models", "apriltag_target", "tag36h11_00000.png")
OUT_DIR = os.path.join(REPO, "hardware", "prints")

# ---- the geometry, all traced (see module docstring) ----------------------
TAG_EDGE_MM = 350.0                      # BLACK SQUARE edge  (placard_mount.md:68)
CELL_MM = TAG_EDGE_MM / 8.0              # 43.75             (placard_mount.md:95)
IMAGE_MM = CELL_MM * 10.0                # 437.5             (placard_mount.md:96)
SHEET_MM = 450.0                         # printed sheet     (placard_mount.md:97)
BLACK_X0 = (SHEET_MM - TAG_EDGE_MM) / 2.0            # 50.0
BLACK_X1 = BLACK_X0 + TAG_EDGE_MM                    # 400.0
IMAGE_X0 = (SHEET_MM - IMAGE_MM) / 2.0               # 6.25

PAPERS = {                    # portrait, mm
    "a4": (210.0, 297.0),
    "letter": (215.9, 279.4),
    "a1": (594.0, 841.0),
    "arch24x36": (609.6, 914.4),
}

PREVIEW_DIR = None     # set by --preview: also dump every page as a PNG so a
                       # human can LOOK at what will print (the read-back
                       # proves the scale; only eyes prove the layout)
MARK = "0.45"          # crop / registration / instruction grey (strokes only)
FAINT = "0.72"

# ---------------------------------------------------------------------------


def tag_grid(png_path=TAG_PNG):
    """Read the canonical id-0 image and return a 10x10 grid, 1 = white."""
    from PIL import Image
    import numpy as np
    a = np.array(Image.open(png_path).convert("L"))
    h, w = a.shape
    g = np.zeros((10, 10), dtype=int)
    for r in range(10):
        for c in range(10):
            g[r, c] = 1 if a[int((r + 0.5) * h / 10), int((c + 0.5) * w / 10)] > 127 else 0
    # structural asserts on the canonical family layout -- fail loud, never silently
    assert (g[0, :] == 1).all() and (g[9, :] == 1).all(), "outer ring must be white"
    assert (g[:, 0] == 1).all() and (g[:, 9] == 1).all(), "outer ring must be white"
    assert (g[1, 1:9] == 0).all() and (g[8, 1:9] == 0).all(), "border ring must be black"
    assert (g[1:9, 1] == 0).all() and (g[1:9, 8] == 0).all(), "border ring must be black"
    return g


def white_cells(grid):
    """Art-space (x0, y0, w, h) of every WHITE cell inside the black square."""
    out = []
    for r in range(1, 9):
        for c in range(1, 9):
            if grid[r, c] == 1:
                x0 = IMAGE_X0 + c * CELL_MM
                y0 = IMAGE_X0 + (9 - r) * CELL_MM      # row 0 is the TOP row
                out.append((x0, y0, CELL_MM, CELL_MM))
    return out


# ---------------------------------------------------------------------------
# drawing helpers (all coordinates in mm, origin bottom-left of the PAGE)
# ---------------------------------------------------------------------------

_PREVIEW_N = [0]


def save_page(pdf, fig, tag):
    pdf.savefig(fig)
    if PREVIEW_DIR:
        _PREVIEW_N[0] += 1
        fig.savefig(os.path.join(PREVIEW_DIR, f"{tag}_{_PREVIEW_N[0]:02d}.png"),
                    dpi=110)
    plt.close(fig)


def new_page(pdf, w_mm, h_mm):
    fig = plt.figure(figsize=(w_mm / 25.4, h_mm / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, w_mm)
    ax.set_ylim(0, h_mm)
    ax.set_axis_off()
    return fig, ax


def rect(ax, x, y, w, h, color):
    if w <= 0 or h <= 0:
        return
    ax.add_patch(Rectangle((x, y), w, h, fc=color, ec="none", lw=0))


def clip_rect(r, win):
    """Intersect art-space rect (x0,y0,w,h) with window (x0,y0,x1,y1)."""
    x0 = max(r[0], win[0])
    y0 = max(r[1], win[1])
    x1 = min(r[0] + r[2], win[2])
    y1 = min(r[1] + r[3], win[3])
    if x1 <= x0 + 1e-9 or y1 <= y0 + 1e-9:
        return None
    return (x0, y0, x1 - x0, y1 - y0)


def ruler(ax, x, y, length, horizontal=True, thick=3.0, label=None,
          major=50.0, minor=10.0, tick_dir=1, numbers=True):
    """A pure-black verification bar of EXACTLY `length` mm, with stroked ticks."""
    if horizontal:
        rect(ax, x, y, length, thick, "black")
    else:
        rect(ax, x, y, thick, length, "black")
    n = int(round(length / minor))
    for i in range(n + 1):
        d = i * minor
        big = abs(d % major) < 1e-6
        t = 4.0 if big else 2.2
        if horizontal:
            y0 = y + thick if tick_dir > 0 else y
            ax.plot([x + d, x + d], [y0, y0 + tick_dir * t], color="black", lw=0.6)
            if big and numbers:
                ax.text(x + d, y0 + tick_dir * (t + 1.0), f"{d:.0f}", ha="center",
                        va="bottom" if tick_dir > 0 else "top",
                        fontsize=5.5, color="black")
        else:
            ax.plot([x + thick, x + thick + t], [y + d, y + d], color="black", lw=0.6)
            if big and numbers:
                ax.text(x + thick + t + 1.0, y + d, f"{d:.0f}", ha="left",
                        va="center", fontsize=5.5, color="black")
    if label:
        if horizontal:
            ax.text(x, y - 1.5, label, ha="left", va="top", fontsize=7,
                    color="black", weight="bold")
        else:
            ax.text(x + thick + 12.0, y + length / 2, label, ha="left", va="center",
                    fontsize=7, color="black", weight="bold", rotation=90)


def crop_marks(ax, x0, y0, x1, y1, out=8.0, gap=0.0, color=MARK, lw=0.5):
    """Standard corner crop marks OUTSIDE the trim box (never inside it)."""
    for (cx, sx) in ((x0, -1), (x1, +1)):
        for (cy, sy) in ((y0, -1), (y1, +1)):
            ax.plot([cx + sx * gap, cx + sx * (gap + out)], [cy, cy], color=color, lw=lw)
            ax.plot([cx, cx], [cy + sy * gap, cy + sy * (gap + out)], color=color, lw=lw)


# ---------------------------------------------------------------------------
# poster (single-sheet) variant
# ---------------------------------------------------------------------------

def build_poster(path, paper_mm, grid, art_top_gap=None):
    """One page: the 450 mm sheet placed on `paper_mm`, crop marks + ruler in
    the margin. If paper == 450x450 exactly, artwork only (quiet zone is holy)."""
    pw, ph = paper_mm
    assert pw >= SHEET_MM and ph >= SHEET_MM, "poster paper must be >= 450 mm"
    exact = abs(pw - SHEET_MM) < 1e-6 and abs(ph - SHEET_MM) < 1e-6

    ox = (pw - SHEET_MM) / 2.0
    oy = (ph - SHEET_MM) / 2.0 if art_top_gap is None else (ph - art_top_gap - SHEET_MM)

    with PdfPages(path) as pdf:
        fig, ax = new_page(pdf, pw, ph)

        # --- artwork (nothing else may enter [ox, ox+450] x [oy, oy+450]) ---
        rect(ax, ox + BLACK_X0, oy + BLACK_X0, TAG_EDGE_MM, TAG_EDGE_MM, "black")
        for (x, y, w, h) in white_cells(grid):
            rect(ax, ox + x, oy + y, w, h, "white")

        if exact:
            # trim line == page edge; a faint hairline ON the edge, trimmed away
            ax.plot([0, pw, pw, 0, 0], [0, 0, ph, ph, 0], color=FAINT, lw=0.4)
        else:
            crop_marks(ax, ox, oy, ox + SHEET_MM, oy + SHEET_MM, out=10.0, gap=2.0)
            ax.plot([ox, ox + SHEET_MM, ox + SHEET_MM, ox, ox],
                    [oy, oy, oy + SHEET_MM, oy + SHEET_MM, oy],
                    color=FAINT, lw=0.35, ls=(0, (6, 4)))
            # margin furniture: ruler + the print instructions
            band_h = oy                       # free space under the artwork
            if band_h >= 26.0:
                ruler(ax, ox, oy - 20.0, 200.0, horizontal=True, thick=3.0,
                      label="SCALE CHECK: this bar must measure EXACTLY 200.0 mm")
                lines = [
                    "AprilTag tag36h11 id 0 -- interceptor target placard.  "
                    "PRINT AT 100% / ACTUAL SIZE. Do NOT 'fit to page' or 'scale to fit'.",
                    "MATTE toner/laser on plain bond. NO gloss, NO lamination "
                    "(specular sun glare wipes out the black/white contrast the decode needs).",
                    f"Trim on the dashed line: sheet = {SHEET_MM:.0f} x {SHEET_MM:.0f} mm.  "
                    f"BLACK SQUARE (= pupil-apriltags tag_size) = {TAG_EDGE_MM:.1f} mm, "
                    f"cell = {CELL_MM:.2f} mm.",
                    "ACCEPTANCE: measure the black square edge with a steel tape. "
                    "350.0 +/- 1.0 mm, both axes, or reprint. Print TWO -- one per panel face, "
                    "both right-reading (docs/placard_mount.md 3.6).",
                ]
                for i, t in enumerate(lines):
                    ax.text(ox, oy - 30.0 - i * 5.0, t, ha="left", va="top",
                            fontsize=7, color="black")
            else:
                ax.text(ox, oy - 4.0, "PRINT AT 100% / ACTUAL SIZE -- black square must "
                        "measure 350.0 mm", ha="left", va="top", fontsize=7, color="black")

        save_page(pdf, fig, os.path.splitext(os.path.basename(path))[0])

    return {"kind": "poster", "paper_mm": [pw, ph], "art_origin_mm": [ox, oy],
            "exact": exact,
            "expect_black_mm": [ox + BLACK_X0, oy + BLACK_X0,
                                ox + BLACK_X1, oy + BLACK_X0 + TAG_EDGE_MM]}


# ---------------------------------------------------------------------------
# tiled variant
# ---------------------------------------------------------------------------

def tile_layout(paper_mm, margin, header, overlap):
    pw, ph = paper_mm
    win_w = pw - 2 * margin
    win_h = ph - 2 * margin - header
    step_x = win_w - overlap
    step_y = win_h - overlap
    ncols = int(math.ceil((SHEET_MM - overlap) / step_x))
    nrows = int(math.ceil((SHEET_MM - overlap) / step_y))
    return dict(win_w=win_w, win_h=win_h, step_x=step_x, step_y=step_y,
                ncols=ncols, nrows=nrows, margin=margin, header=header,
                overlap=overlap, paper=(pw, ph))


def build_tiled(path, paper_mm, grid, margin=8.0, header=15.0, overlap=10.0):
    L = tile_layout(paper_mm, margin, header, overlap)
    pw, ph = paper_mm
    ncols, nrows = L["ncols"], L["nrows"]
    tiles = []

    with PdfPages(path) as pdf:
        # ---------------- page 1: instructions + scale bars ----------------
        fig, ax = new_page(pdf, pw, ph)
        y = ph - margin - 4.0
        ax.text(margin, y, "AprilTag placard -- tag36h11 id 0 -- TILED HOME PRINT",
                fontsize=13, weight="bold", va="top", color="black")
        y -= 8.0
        ax.text(margin, y,
                f"Black square (= pupil-apriltags tag_size) = {TAG_EDGE_MM:.1f} mm   |   "
                f"cell = {CELL_MM:.2f} mm   |   sheet = {SHEET_MM:.0f} x {SHEET_MM:.0f} mm   |   "
                f"{ncols} x {nrows} = {ncols*nrows} tiles",
                fontsize=8, va="top", color="black")
        y -= 10.0
        body = [
            ("1.  PRINT SETTINGS -- get these wrong and every range the campaign "
             "measures is wrong.", True),
            ("     Scale: 100% / 'Actual size'.  NEVER 'Fit to page', 'Shrink oversized "
             "pages' or 'Scale to fit'.", False),
            ("     Orientation portrait. Duplex OFF. Paper: "
             f"{'A4 210 x 297 mm' if abs(pw-210) < 1 else 'US Letter 8.5 x 11 in'}.", False),
            ("     Toner/laser on plain MATTE bond. No gloss, no lamination -- specular sun "
             "glare kills the decode", False),
            ("     (docs/placard_mount.md 2; Gazebo has no glare model so no sim result "
             "covers it).", False),
            ("", False),
            ("2.  VERIFY THE PRINTER before cutting anything: measure the two bars below "
             "with a steel rule.", True),
            ("     Vertical bar = 200.0 mm.  Horizontal bar = 150.0 mm.  Off by more than "
             "1 mm -> fix the print dialog, do not proceed.", False),
            ("", False),
            ("3.  ASSEMBLE.  Tiles are labelled C<col>R<row>, row 1 = TOP.  Each tile "
             f"overlaps its neighbours by {overlap:.0f} mm.", True),
            ("     a) Lay out the grid face up in the C/R order printed in each header.", False),
            ("     b) Trim the RIGHT and BOTTOM white edge off every tile except the last "
             "column / last row,", False),
            ("        cutting on the dashed 'NEXT TILE STARTS HERE' line.", False),
            ("     c) Slide each trimmed tile onto its neighbour until the TAG "
             "ARTWORK lines up across the seam --", False),
            ("        the black cell edges are the registration, and they are a "
             "finer cue than any printed mark.", False),
            ("     d) Tape the seams on the BACK. Any tape on the front adds gloss -> glare.",
             False),
            ("     e) Trim the assembled sheet on the dashed 450 mm trim line / corner crop "
             "marks.", False),
            ("", False),
            ("4.  ACCEPTANCE CHECK (do not skip -- this is the one that catches a bad "
             "assembly).", True),
            ("     Measure the BLACK SQUARE edge, both axes: 350.0 +/- 1.0 mm. Measure both "
             "diagonals: equal within 2 mm.", False),
            ("     Then measure the white sheet: 450 x 450 mm. If the black square is right "
             "but the sheet is not, re-trim;", False),
            ("     if the BLACK SQUARE is wrong, the print scaled -- reprint. The quiet zone "
             "(50 mm of white all round)", False),
            ("     must be clean: no tape, no pen marks, no crop marks inside the sheet.",
             False),
            ("", False),
            ("5.  PRINT TWO of these -- one per panel face, both RIGHT-READING. Do NOT "
             "photocopy one and flip it:", True),
            ("     a mirrored tag36h11 is not a member of the family and will simply never "
             "decode (placard_mount.md 3.6).", False),
        ]
        for txt, bold in body:
            ax.text(margin, y, txt, fontsize=7.2, va="top", color="black",
                    weight="bold" if bold else "normal")
            y -= 4.4

        # Scale bars. LAYOUT CONSTRAINT: a 200 mm bar does not fit HORIZONTALLY
        # inside A4's printable width (210 - 2*8 = 194 mm) or Letter's (199.9),
        # so the 200 mm reference has to run vertically, up the right-hand
        # gutter, clear of the text block (which ends by x ~ 160 mm).
        bar_y = margin + 14.0
        vbar_x = pw - 30.0
        ruler(ax, vbar_x, bar_y, 200.0, horizontal=False, thick=3.0,
              label="THIS BAR MUST MEASURE EXACTLY 200.0 mm")
        ruler(ax, margin, margin + 4.0, 150.0, horizontal=True, thick=3.0,
              label="THIS BAR MUST MEASURE EXACTLY 150.0 mm")

        # tile map
        map_w = 46.0
        mx = margin + 100.0
        my = bar_y + 30.0
        cw = map_w / ncols
        chh = map_w / nrows
        ax.text(mx, my + nrows * chh + 3.0, "tile map (as taped up)", fontsize=6.5,
                color="black", va="bottom")
        for c in range(ncols):
            for r in range(nrows):
                ax.plot([mx + c * cw, mx + (c + 1) * cw, mx + (c + 1) * cw,
                         mx + c * cw, mx + c * cw],
                        [my + (nrows - 1 - r) * chh, my + (nrows - 1 - r) * chh,
                         my + (nrows - r) * chh, my + (nrows - r) * chh,
                         my + (nrows - 1 - r) * chh], color=MARK, lw=0.5)
                ax.text(mx + (c + 0.5) * cw, my + (nrows - r - 0.5) * chh,
                        f"C{c+1}R{r+1}", fontsize=6, ha="center", va="center",
                        color="black")
        save_page(pdf, fig, os.path.splitext(os.path.basename(path))[0])

        # ---------------- the tiles ----------------
        whites = white_cells(grid)
        for r in range(nrows):
            for c in range(ncols):
                ax_x0 = c * L["step_x"]
                ax_y1 = SHEET_MM - r * L["step_y"]                 # art y of window TOP
                win = (ax_x0, ax_y1 - L["win_h"], ax_x0 + L["win_w"], ax_y1)

                fig, ax = new_page(pdf, pw, ph)

                def to_page(ax_, ay_):
                    return (margin + (ax_ - win[0]), margin + (ay_ - win[1]))

                # header strip
                hy = ph - margin - header
                ax.plot([margin, pw - margin], [hy, hy], color=MARK, lw=0.4)
                ax.text(margin, ph - margin - 1.5,
                        f"PLACARD tag36h11 id 0   |   TILE  C{c+1}R{r+1}  of {ncols}x{nrows}"
                        f"   |   PRINT AT 100% / ACTUAL SIZE -- NO fit-to-page",
                        fontsize=7.2, weight="bold", va="top", color="black")
                ruler(ax, margin, hy + 6.0, 100.0, horizontal=True, thick=2.2,
                      major=50.0, minor=10.0, label=None, tick_dir=-1,
                      numbers=False)
                ax.text(margin + 104.0, hy + 7.1,
                        "<-- SCALE CHECK: this bar = EXACTLY 100.0 mm",
                        fontsize=6.5, va="center", color="black")

                # artwork, geometrically clipped to this tile's window
                cr = clip_rect((BLACK_X0, BLACK_X0, TAG_EDGE_MM, TAG_EDGE_MM), win)
                if cr:
                    px, py = to_page(cr[0], cr[1])
                    rect(ax, px, py, cr[2], cr[3], "black")
                for wc in whites:
                    wcr = clip_rect(wc, win)
                    if wcr:
                        px, py = to_page(wcr[0], wcr[1])
                        rect(ax, px, py, wcr[2], wcr[3], "white")

                # 450 mm trim line (the portion crossing this tile) + corner crop marks
                for xt in (0.0, SHEET_MM):
                    if win[0] - 1e-9 <= xt <= win[2] + 1e-9:
                        y0 = max(win[1], 0.0)
                        y1 = min(win[3], SHEET_MM)
                        if y1 > y0:
                            p0, p1 = to_page(xt, y0), to_page(xt, y1)
                            ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=FAINT,
                                    lw=0.4, ls=(0, (6, 4)))
                for yt in (0.0, SHEET_MM):
                    if win[1] - 1e-9 <= yt <= win[3] + 1e-9:
                        x0 = max(win[0], 0.0)
                        x1 = min(win[2], SHEET_MM)
                        if x1 > x0:
                            p0, p1 = to_page(x0, yt), to_page(x1, yt)
                            ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=FAINT,
                                    lw=0.4, ls=(0, (6, 4)))
                for (cx, cy) in ((0.0, 0.0), (SHEET_MM, 0.0), (0.0, SHEET_MM),
                                 (SHEET_MM, SHEET_MM)):
                    if win[0] <= cx <= win[2] and win[1] <= cy <= win[3]:
                        px, py = to_page(cx, cy)
                        sx = -1 if cx == 0.0 else 1
                        sy = -1 if cy == 0.0 else 1
                        ax.plot([px + sx * 2, px + sx * 9], [py, py], color=MARK, lw=0.5)
                        ax.plot([px, px], [py + sy * 2, py + sy * 9], color=MARK, lw=0.5)

                # NOTE: no registration crosses are drawn inside the artwork.
                # A grey cross on top of the tag's black cells is invisible, and
                # a white one would punch a hole in the tag. The ARTWORK is the
                # registration: both tiles show the same 10 mm of high-contrast
                # cell edges in the overlap band, which is a finer alignment cue
                # than any printed mark.

                # "next tile starts here" cut guides
                # Seam guides: STUBS in the page margin, not lines across the
                # artwork (a guide line over the tag would be invisible on black
                # and would add ink to the tag on white). Lay a straight edge
                # between the two stubs to trim.
                if c < ncols - 1:
                    px, _ = to_page((c + 1) * L["step_x"], win[1])
                    for (ya, yb) in ((margin - 6.0, margin), 
                                     (margin + L["win_h"], margin + L["win_h"] + 6.0)):
                        ax.plot([px, px], [ya, yb], color=MARK, lw=0.6)
                    ax.text(px - 1.2, margin + L["win_h"] + 1.0,
                            "TRIM: next tile (C%d) starts here -->" % (c + 2),
                            fontsize=5.5, va="bottom", ha="right", color=MARK)
                if r < nrows - 1:
                    _, py = to_page(win[0], SHEET_MM - (r + 1) * L["step_y"])
                    for (xa, xb) in ((margin - 6.0, margin),
                                     (margin + L["win_w"], margin + L["win_w"] + 6.0)):
                        ax.plot([xa, xb], [py, py], color=MARK, lw=0.6)
                    ax.text(margin + 2.0, py + 1.0,
                            "TRIM: next tile (R%d) starts here" % (r + 2),
                            fontsize=5.5, va="bottom", ha="left", color=MARK)

                save_page(pdf, fig, os.path.splitext(os.path.basename(path))[0])
                # NOTE: only the tile's art WINDOW is recorded. The expected black
                # extent is recomputed in verify() from the traced constants
                # (BLACK_X0 / TAG_EDGE_MM) -- never from the geometry that was
                # actually drawn. A mutant that shrinks the drawn square must fail
                # the read-back, and an expectation copied from the drawing could
                # not see it (verified by a deliberate -1.5 mm mutant, 2026-07-25).
                tiles.append({"col": c, "row": r, "window_art_mm": list(win)})

    return {"kind": "tiled", "paper_mm": [pw, ph], "layout": {
        k: (list(v) if isinstance(v, tuple) else v) for k, v in L.items()},
        "tiles": tiles,
        "header_band_mm": [margin, ph - margin - header, pw - margin, ph - margin]}


# ---------------------------------------------------------------------------
# raster decode check -- is the ink we drew actually a decodable tag36h11 id 0?
# ---------------------------------------------------------------------------

def decode_check(grid, dpi=200.0, out_png=None, tol_mm=0.5):
    """Rasterise the SAME primitives at a known dpi, run the REAL detector on it,
    and convert the detector's own recovered corners back to millimetres.

    This is the complement to the vector read-back: the PDF check proves the
    black square is 350.000 mm, this proves the ink inside it is a valid
    `tag36h11` id 0 (right family, right id, not mirrored, right polarity) and
    that the detector recovers the 350 mm edge from pixels. `pupil_apriltags`
    reports the corners of the BLACK SQUARE -- exactly the edge `tag_size`
    means -- so the px->mm conversion is a direct measurement of the thing the
    money gate depends on.
    """
    import numpy as np
    try:
        from pupil_apriltags import Detector
    except ImportError:
        return {"ran": False, "reason": "pupil_apriltags not installed"}

    import matplotlib
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    fig = Figure(figsize=(SHEET_MM / 25.4, SHEET_MM / 25.4), dpi=dpi)
    canvas = FigureCanvasAgg(fig)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, SHEET_MM)
    ax.set_ylim(0, SHEET_MM)
    ax.set_axis_off()
    ax.add_patch(Rectangle((BLACK_X0, BLACK_X0), TAG_EDGE_MM, TAG_EDGE_MM,
                           fc="black", ec="none", lw=0))
    for (x, y, w, h) in white_cells(grid):
        ax.add_patch(Rectangle((x, y), w, h, fc="white", ec="none", lw=0))
    canvas.draw()
    buf = np.asarray(canvas.buffer_rgba())[:, :, :3]
    gray = (0.299 * buf[:, :, 0] + 0.587 * buf[:, :, 1] + 0.114 * buf[:, :, 2])
    gray = np.ascontiguousarray(gray.astype(np.uint8))
    if out_png:
        from PIL import Image
        Image.fromarray(gray).save(out_png)

    px_per_mm = gray.shape[0] / SHEET_MM
    det = Detector(families="tag36h11")          # library default quad_decimate=2.0
    res = det.detect(gray)
    if len(res) != 1:
        return {"ran": True, "ok": False, "n_detections": len(res),
                "reason": f"expected exactly 1 detection, got {len(res)}"}
    d = res[0]
    c = np.asarray(d.corners, dtype=float)       # 4x2, black-square corners, px
    edges_px = [float(np.linalg.norm(c[(i + 1) % 4] - c[i])) for i in range(4)]
    edges_mm = [e / px_per_mm for e in edges_px]
    err = max(abs(e - TAG_EDGE_MM) for e in edges_mm)
    return {
        "ran": True,
        "ok": bool(d.tag_id == 0 and d.hamming == 0 and err <= tol_mm),
        "raster_px": int(gray.shape[0]), "dpi": dpi,
        "px_per_mm": round(px_per_mm, 6),
        "tag_family": d.tag_family.decode() if isinstance(d.tag_family, bytes)
        else str(d.tag_family),
        "tag_id": int(d.tag_id), "hamming": int(d.hamming),
        "decision_margin": round(float(d.decision_margin), 2),
        "edges_mm": [round(e, 4) for e in edges_mm],
        "max_edge_err_mm": round(err, 4),
        "centre_mm": [round(float(d.center[0]) / px_per_mm, 3),
                      round(SHEET_MM - float(d.center[1]) / px_per_mm, 3)],
    }


# ---------------------------------------------------------------------------
# verification -- re-open the emitted bytes and MEASURE
# ---------------------------------------------------------------------------

def verify(path, spec, tol_mm=0.5):
    checks = []

    def chk(name, ok, detail):
        checks.append({"check": name, "ok": bool(ok), "detail": detail})

    pages = pdf_measure.page_streams(path)

    if spec["kind"] == "poster":
        chk("page_count", len(pages) == 1, f"{len(pages)} page(s)")
        pw, ph = spec["paper_mm"]
        mw, mh = pages[0]["media_box_mm"]
        chk("media_box_mm", abs(mw - pw) < 1e-3 and abs(mh - ph) < 1e-3,
            f"{mw:.4f} x {mh:.4f} mm (expect {pw} x {ph})")
        ex = spec["expect_black_mm"]
        # artwork region only: the ruler bar lives below the trim box
        bb = pdf_measure.black_bbox(pages[0]["content"],
                                    region=(ex[0] - 60, ex[1] - 60, ex[2] + 60, ex[3] + 60))
        w, h = bb[2] - bb[0], bb[3] - bb[1]
        chk("black_square_edge_mm",
            abs(w - TAG_EDGE_MM) <= tol_mm and abs(h - TAG_EDGE_MM) <= tol_mm,
            f"measured {w:.4f} x {h:.4f} mm (expect {TAG_EDGE_MM:.1f}; "
            f"err {abs(w-TAG_EDGE_MM)*1000:.1f} / {abs(h-TAG_EDGE_MM)*1000:.1f} um)")
        chk("black_square_position_mm",
            max(abs(bb[i] - ex[i]) for i in range(4)) <= tol_mm,
            f"x {bb[0]:.3f}..{bb[2]:.3f}, y {bb[1]:.3f}..{bb[3]:.3f} "
            f"(expect {ex[0]:.3f}..{ex[2]:.3f} / {ex[1]:.3f}..{ex[3]:.3f})")
        # quiet zone: nothing but the tag inside the 450 mm sheet
        ox, oy = spec["art_origin_mm"]
        stray = []
        for f in pdf_measure.black_fills(pages[0]["content"]):
            x0, y0, x1, y1 = f.bbox
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            inside = (ox <= cx <= ox + SHEET_MM and oy <= cy <= oy + SHEET_MM)
            in_tag = (ex[0] - 1e-6 <= cx <= ex[2] + 1e-6 and ex[1] - 1e-6 <= cy <= ex[3] + 1e-6)
            if inside and not in_tag:
                stray.append(f.bbox)
        chk("quiet_zone_clean", not stray,
            "no black ink inside the 450 mm sheet outside the tag square"
            if not stray else f"{len(stray)} stray fill(s): {stray[:3]}")
        if not spec["exact"] and spec["paper_mm"][1] - SHEET_MM >= 52.0:
            bars = [f for f in pdf_measure.black_fills(pages[0]["content"])
                    if f.is_rect() and abs(f.size[0] - 200.0) < tol_mm and f.size[1] < 6]
            chk("ruler_bar_200mm", len(bars) == 1,
                f"{len(bars)} bar(s) of 200.0 mm" +
                (f"; measured {bars[0].size[0]:.4f} mm" if bars else ""))

    else:  # tiled
        L = spec["layout"]
        exp_pages = 1 + len(spec["tiles"])
        chk("page_count", len(pages) == exp_pages,
            f"{len(pages)} page(s) (1 instructions + {len(spec['tiles'])} tiles)")

        # instructions page: the two verification bars
        f0 = [f for f in pdf_measure.black_fills(pages[0]["content"]) if f.is_rect()]
        v = [f for f in f0 if abs(f.size[1] - 200.0) < tol_mm and f.size[0] < 6]
        hbar = [f for f in f0 if abs(f.size[0] - 150.0) < tol_mm and f.size[1] < 6]
        chk("instructions_bar_200mm", len(v) == 1,
            (f"measured {v[0].size[1]:.4f} mm" if v else f"{len(v)} candidates"))
        chk("instructions_bar_150mm", len(hbar) == 1,
            (f"measured {hbar[0].size[0]:.4f} mm" if hbar else f"{len(hbar)} candidates"))

        hb = spec["header_band_mm"]
        cover_x, cover_y = [], []
        worst = 0.0
        for i, t in enumerate(spec["tiles"]):
            page = pages[1 + i]
            mw, mh = page["media_box_mm"]
            ok_mb = (abs(mw - spec["paper_mm"][0]) < 1e-3
                     and abs(mh - spec["paper_mm"][1]) < 1e-3)
            # header scale bar, measured in the header band only
            bars = [f for f in pdf_measure.black_fills(page["content"])
                    if f.is_rect() and hb[1] <= (f.bbox[1] + f.bbox[3]) / 2 <= hb[3]]
            bar_ok = len(bars) == 1 and abs(bars[0].size[0] - 100.0) <= tol_mm
            # artwork, measured below the header band
            bb = pdf_measure.black_bbox(page["content"], region=(-1e6, -1e6, 1e6, hb[1]))
            # expectation from the TRACED CONSTANTS + this tile's window, never
            # from what the drawing code produced (see build_tiled's note)
            w = t["window_art_mm"]
            cr = clip_rect((BLACK_X0, BLACK_X0, TAG_EDGE_MM, TAG_EDGE_MM),
                           (w[0], w[1], w[2], w[3]))
            if cr is None:
                e = None
            else:
                e = (L["margin"] + (cr[0] - w[0]), L["margin"] + (cr[1] - w[1]),
                     L["margin"] + (cr[0] - w[0]) + cr[2],
                     L["margin"] + (cr[1] - w[1]) + cr[3])
            if e is None:
                art_ok = bb is None
                detail = "tile carries no artwork (expected none)"
            elif bb is None:
                art_ok = False
                detail = ("NO black ink found, but this tile should carry "
                          f"{e[2]-e[0]:.3f} x {e[3]-e[1]:.3f} mm of the tag square")
            else:
                err = max(abs(bb[k] - e[k]) for k in range(4))
                worst = max(worst, err)
                art_ok = err <= tol_mm
                detail = (f"black {bb[2]-bb[0]:.4f} x {bb[3]-bb[1]:.4f} mm at "
                          f"({bb[0]:.3f}, {bb[1]:.3f}); expected "
                          f"{e[2]-e[0]:.4f} x {e[3]-e[1]:.4f} at ({e[0]:.3f}, {e[1]:.3f}); "
                          f"max corner err {err*1000:.1f} um")
                cover_x.append((max(w[0], BLACK_X0), min(w[2], BLACK_X1)))
                cover_y.append((max(w[1], BLACK_X0), min(w[3], BLACK_X1)))
            chk(f"tile_C{t['col']+1}R{t['row']+1}",
                ok_mb and bar_ok and art_ok,
                f"MediaBox {mw:.3f}x{mh:.3f}; 100 mm bar "
                f"{'%.4f mm' % bars[0].size[0] if bars else 'MISSING'}; {detail}")

        def covered(spans, lo, hi):
            spans = sorted(s for s in spans if s[1] > s[0])
            cur = lo
            for a, b in spans:
                if a > cur + 1e-6:
                    return False
                cur = max(cur, b)
            return cur >= hi - 1e-6

        chk("tiles_reassemble_x", covered(cover_x, BLACK_X0, BLACK_X1),
            f"art-space x coverage of the black square {BLACK_X0}..{BLACK_X1} mm")
        chk("tiles_reassemble_y", covered(cover_y, BLACK_X0, BLACK_X1),
            f"art-space y coverage of the black square {BLACK_X0}..{BLACK_X1} mm")
        chk("worst_tile_error", worst <= tol_mm,
            f"worst corner error across all tiles {worst*1000:.1f} um (tol {tol_mm*1000:.0f} um)")

    return checks


# ---------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default=OUT_DIR)
    ap.add_argument("--tol-mm", type=float, default=0.5,
                    help="scale/position tolerance for the PDF read-back (default 0.5 mm)")
    ap.add_argument("--tile-margin-mm", type=float, default=8.0,
                    help="unprintable-edge allowance on the tiled variants")
    ap.add_argument("--tile-overlap-mm", type=float, default=10.0)
    ap.add_argument("--verify-only", action="store_true",
                    help="re-measure the existing PDFs without regenerating")
    ap.add_argument("--preview", default=None, metavar="DIR",
                    help="also dump every page as a PNG here, to eyeball the layout")
    ap.add_argument("--report", default=None,
                    help="write the verification report JSON here")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    if args.preview:
        global PREVIEW_DIR
        PREVIEW_DIR = args.preview
        os.makedirs(PREVIEW_DIR, exist_ok=True)
    grid = tag_grid()
    print("[placard] tag36h11 id 0 grid read from",
          os.path.relpath(TAG_PNG, REPO))
    for r in range(10):
        print("[placard]   " + "".join("." if v else "#" for v in grid[r]))
    print(f"[placard] BLACK SQUARE {TAG_EDGE_MM:.1f} mm  (= pupil-apriltags tag_size; "
          f"docs/placard_mount.md:68)")
    print(f"[placard] cell {CELL_MM:.3f} mm | 10-cell image {IMAGE_MM:.1f} mm | "
          f"sheet {SHEET_MM:.1f} mm (placard_mount.md:95-97)")

    jobs = [
        ("placard_tag36h11_id0_poster_a1.pdf", "poster", PAPERS["a1"], dict(art_top_gap=40.0)),
        ("placard_tag36h11_id0_poster_arch24x36.pdf", "poster", PAPERS["arch24x36"],
         dict(art_top_gap=45.0)),
        ("placard_tag36h11_id0_poster_bleed480.pdf", "poster", (480.0, 512.0), dict()),
        ("placard_tag36h11_id0_poster_exact450.pdf", "poster", (450.0, 450.0), dict()),
        ("placard_tag36h11_id0_tiled_a4.pdf", "tiled", PAPERS["a4"], dict()),
        ("placard_tag36h11_id0_tiled_letter.pdf", "tiled", PAPERS["letter"], dict()),
    ]

    report = {"tag_edge_mm": TAG_EDGE_MM, "cell_mm": CELL_MM, "sheet_mm": SHEET_MM,
              "tol_mm": args.tol_mm, "files": []}
    specs_path = os.path.join(args.out_dir, ".placard_specs.json")
    specs = {}
    if args.verify_only and os.path.exists(specs_path):
        specs = json.load(open(specs_path))

    failed = 0
    for name, kind, paper, kw in jobs:
        path = os.path.join(args.out_dir, name)
        if args.verify_only:
            spec = specs.get(name)
            if spec is None:
                print(f"[placard] {name}: no stored spec, cannot verify -- regenerate")
                failed += 1
                continue
        elif kind == "poster":
            spec = build_poster(path, paper, grid, **kw)
        else:
            spec = build_tiled(path, paper, grid, margin=args.tile_margin_mm,
                               overlap=args.tile_overlap_mm, **kw)
        specs[name] = spec

        checks = verify(path, spec, tol_mm=args.tol_mm)
        ok = all(c["ok"] for c in checks)
        failed += 0 if ok else 1
        npages = len(pdf_measure.page_streams(path))
        size_kb = os.path.getsize(path) / 1024.0
        print(f"\n[placard] {name}  ({paper[0]:.1f} x {paper[1]:.1f} mm, "
              f"{npages} page(s), {size_kb:.0f} kB)")
        for c in checks:
            print(f"[placard]   {'PASS' if c['ok'] else 'FAIL'}  {c['check']}: {c['detail']}")
        print(f"[placard]   ==> {'PASS' if ok else 'FAIL'}")
        report["files"].append({"file": name, "paper_mm": list(paper),
                                "pages": npages, "ok": ok, "checks": checks})

    if not args.verify_only:
        with open(specs_path, "w") as fh:
            json.dump(specs, fh, indent=1)

    dc = decode_check(grid, tol_mm=args.tol_mm,
                      out_png=os.path.join(args.out_dir,
                                           "placard_tag36h11_id0_proof.png"))
    report["decode_check"] = dc
    print("\n[placard] RASTER DECODE CHECK (real pupil_apriltags detector on the "
          "same primitives)")
    if not dc.get("ran"):
        print(f"[placard]   SKIPPED: {dc.get('reason')}")
    else:
        print(f"[placard]   raster {dc['raster_px']} px @ {dc['dpi']:.0f} dpi "
              f"({dc['px_per_mm']:.4f} px/mm)")
        print(f"[placard]   family={dc.get('tag_family')} id={dc.get('tag_id')} "
              f"hamming={dc.get('hamming')} decision_margin={dc.get('decision_margin')}")
        print(f"[placard]   detector-recovered black-square edges (mm): "
              f"{dc.get('edges_mm')}  -> max err {dc.get('max_edge_err_mm')} mm")
        print(f"[placard]   tag centre {dc.get('centre_mm')} mm "
              f"(expect [{SHEET_MM/2:.1f}, {SHEET_MM/2:.1f}])")
        print(f"[placard]   ==> {'PASS' if dc.get('ok') else 'FAIL'}")
        if not dc.get("ok"):
            failed += 1

    report["all_ok"] = failed == 0
    rep_path = args.report or os.path.join(args.out_dir, "placard_verification.json")
    with open(rep_path, "w") as fh:
        json.dump(report, fh, indent=1)
    print(f"\n[placard] report -> {os.path.relpath(rep_path, REPO)}")
    print(f"[placard] {'ALL VARIANTS PASS' if failed == 0 else f'{failed} VARIANT(S) FAILED'}")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
