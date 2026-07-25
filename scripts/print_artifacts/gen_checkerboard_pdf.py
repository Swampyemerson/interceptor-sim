#!/usr/bin/env python3
"""Generate the PRINT-READY camera-calibration checkerboard PDFs -- true scale.

    .venv/bin/python scripts/print_artifacts/gen_checkerboard_pdf.py

WHAT IT MAKES (into hardware/prints/):
    checkerboard_9x6_25mm_a4.pdf        A4 landscape      <-- matches the pinned command
    checkerboard_9x6_25mm_letter.pdf    US Letter landscape
    checkerboard_9x6_25mm_a3.pdf        A3 landscape (roomy quiet zone; copy-shop path)
    checkerboard_9x6_20mm_a4.pdf        A4 landscape, 20 mm squares -> --square 0.020

THE PATTERN IS ALREADY PINNED -- THIS SCRIPT DOES NOT CHOOSE IT
---------------------------------------------------------------
9 x 6 INNER CORNERS, 25.0 mm squares. Traced, four independent places:

  * docs/tripod_test_protocol.md:330-336
        "1. Print/mount the checkerboard (9x6 inner corners) rigidly and flat."
        "scripts/calibrate_camera.py --images ... --cols 9 --rows 6 --square 0.025"
  * docs/field_bringup.md:325
        "scripts/calibrate_camera.py --images 'calib/*.png' --cols 9 --rows 6 --square 0.025"
  * scripts/calibrate_camera.py (module docstring)
        "Chessboard: give the INNER-corner count (e.g. a 10x7-square board has
         9x6 inner corners: --cols 9 --rows 6) and the square size in metres."
  * docs/project_state.json:1061 / :1365
        bom_tiers "Checkerboard calibration board (9x6 inner corners, rigid backing)"

9 x 6 inner corners  =>  10 x 7 SQUARES  =>  250.0 x 175.0 mm of board.
(That odd-x-even inner-corner count is the OpenCV convention: it makes the
board's orientation unambiguous, so corner ordering cannot flip between views.)

WHY THE SQUARE SIZE IS A MEASUREMENT, NOT A SETTING
---------------------------------------------------
`calibrate_camera.py` builds its object points as `mgrid * square` and hands
them to `cv2.calibrateCamera`. The printed square size is therefore the ONLY
physical scale in the whole calibration: fx/fy come out proportional to it. A
board printed 3 % small makes fx 3 % wrong, which makes every range and every
bearing in the entire campaign 3 % wrong -- silently, with a perfectly healthy
RMS reprojection error, because a uniform scale error is exactly consistent
with itself (docs/sim_to_real_gaps.md L1/L4; calibrate_camera.py's own header:
"Fly with the sim's intrinsics on real hardware and every range is wrong ...
a silent, total failure of the intercept").

So this generator does the same thing the placard generator does: it re-opens
the emitted PDF bytes, walks the drawing operators, and MEASURES every square
in millimetres (scripts/print_artifacts/pdf_measure.py), then rasterises the
board and runs the REAL `cv2.findChessboardCorners` on it to confirm the
pattern is findable at (9, 6) and that the recovered corner pitch converts
back to 25.000 mm.

PAGE LAYOUT. Page 1 is instructions + the printed 200.0 / 150.0 mm scale bars.
Page 2 is the board and NOTHING ELSE -- no header, no crop marks, no ruler.
That is deliberate: `findChessboardCorners` needs a clean white quiet zone
around the board, and on A4 the margin (17.5 mm short axis) is already less
than one square. On A3, where the margin is >60 mm, a ruler is added to the
board page as well, kept >= one square clear of the board.

BUYING INSTEAD OF PRINTING. bom_tiers prices a rigid pre-printed board at ~$8
and that remains the better artifact (flatness is the dominant calibration
error after scale, and a home print taped to cardboard bows). If you buy one,
BUY 9x6 INNER CORNERS and read its actual square size off the vendor spec --
then pass that number to `--square`, do not assume 25 mm. This PDF is the
zero-lead-time fallback so the FIRST bench gate (build_tab skr-06) is never
blocked on shipping.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import matplotlib
matplotlib.use("pdf")
matplotlib.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams["pdf.compression"] = 6
import matplotlib.pyplot as plt                                   # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages              # noqa: E402
from matplotlib.patches import Rectangle                          # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pdf_measure                                                # noqa: E402
from gen_placard_pdf import new_page, rect, ruler                 # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT_DIR = os.path.join(REPO, "hardware", "prints")

INNER_COLS = 9        # tripod_test_protocol.md:330-336  (--cols 9)
INNER_ROWS = 6        # tripod_test_protocol.md:330-336  (--rows 6)

PAPERS_LANDSCAPE = {                 # (w, h) mm, landscape
    "a4": (297.0, 210.0),
    "letter": (279.4, 215.9),
    "a3": (420.0, 297.0),
}

MARK = "0.45"


def board_size(square_mm):
    return ((INNER_COLS + 1) * square_mm, (INNER_ROWS + 1) * square_mm)


def draw_board(ax, ox, oy, square_mm):
    """Black squares of a (cols+1) x (rows+1) chessboard, origin bottom-left."""
    nx, ny = INNER_COLS + 1, INNER_ROWS + 1
    n_black = 0
    for i in range(nx):
        for j in range(ny):
            if (i + j) % 2 == 0:
                rect(ax, ox + i * square_mm, oy + j * square_mm,
                     square_mm, square_mm, "black")
                n_black += 1
    return n_black


def build(path, paper, square_mm):
    pw, ph = paper
    bw, bh = board_size(square_mm)
    assert bw <= pw and bh <= ph, f"board {bw}x{bh} does not fit {pw}x{ph}"
    ox, oy = (pw - bw) / 2.0, (ph - bh) / 2.0
    sq_m = square_mm / 1000.0
    cmd = (f"scripts/calibrate_camera.py --images 'calib/*.png' "
           f"--cols {INNER_COLS} --rows {INNER_ROWS} --square {sq_m:.3f} "
           f"--out calib.json")

    with PdfPages(path) as pdf:
        # ---------------- page 1: instructions + scale bars ----------------
        fig, ax = new_page(pdf, pw, ph)
        y = ph - 12.0
        ax.text(14.0, y, "Camera calibration checkerboard -- "
                f"{INNER_COLS} x {INNER_ROWS} INNER CORNERS, {square_mm:.1f} mm squares",
                fontsize=13, weight="bold", va="top", color="black")
        y -= 8.0
        ax.text(14.0, y,
                f"board {bw:.1f} x {bh:.1f} mm  ({INNER_COLS+1} x {INNER_ROWS+1} squares)   |   "
                f"inner-corner span {(INNER_COLS-1)*square_mm:.1f} x "
                f"{(INNER_ROWS-1)*square_mm:.1f} mm   |   page 2 is the board",
                fontsize=8, va="top", color="black")
        y -= 11.0
        body = [
            ("1.  PRINT SETTINGS. Scale 100% / 'Actual size'. NEVER 'fit to page' or "
             "'shrink oversized pages'.", True),
            ("     Landscape. Duplex OFF. MATTE plain paper -- gloss reflects the light "
             "you are calibrating under.", False),
            ("", False),
            ("2.  VERIFY THE PRINTER on the bars below BEFORE you mount the board.", True),
            ("     200.0 mm bar and 150.0 mm bar, measured with a steel rule. Out by >1 mm "
             "-> fix the dialog and reprint.", False),
            ("", False),
            ("3.  MOUNT IT DEAD FLAT. Spray-mount page 2 to foamboard, MDF, or glass -- "
             "no bubbles, no bow, no curl.", True),
            ("     After scale, FLATNESS is the dominant calibration error: a 2 mm bow "
             "across the board biases the", False),
            ("     distortion coefficients and the RMS will still look fine. Tape at the "
             "corners only is NOT enough.", False),
            ("", False),
            ("4.  MEASURE THE BOARD ITSELF (the board is its own ruler -- do this even if "
             "you bought a board).", True),
            (f"     Outermost inner corner to outermost inner corner: "
             f"{(INNER_COLS-1)*square_mm:.1f} mm along the LONG axis, "
             f"{(INNER_ROWS-1)*square_mm:.1f} mm along the short axis.", False),
            (f"     Full black extent: {bw:.1f} x {bh:.1f} mm. Divide the long span by "
             f"{INNER_COLS-1} to get your ACTUAL square size", False),
            ("     and pass THAT to --square. Do not assume the nominal value.", False),
            ("", False),
            ("5.  CAPTURE >= 15 views (build_tab skr-06 acceptance: RMS reprojection "
             "<= 1.0 px).", True),
            ("     Vary angle, distance and position; fill the corners of the frame, not "
             "just the centre; keep the board", False),
            ("     sharp (it is the LENS you are measuring, not the paper). Shoot in the "
             "FINAL tripod geometry, and", False),
            ("     re-shoot a short set at end of session to catch thermal drift "
             "(tripod_test_protocol.md 5).", False),
            ("", False),
            ("6.  RUN:", True),
            (f"     {cmd}", False),
            ("", False),
            ("     Every range and bearing the campaign produces is wrong until this "
             "passes. A uniform print-scale error", False),
            ("     is INVISIBLE in the RMS -- it is consistent with itself. The bars in "
             "step 2 and the measurement in", False),
            ("     step 4 are the only things standing between a 3% print error and a 3% "
             "error in every number.", False),
        ]
        for txt, bold in body:
            ax.text(14.0, y, txt, fontsize=7.2, va="top", color="black",
                    weight="bold" if bold else "normal",
                    family="monospace" if txt.strip().startswith("scripts/") else None)
            y -= 4.4

        ruler(ax, 14.0, 16.0, 200.0, horizontal=True, thick=3.0,
              label="THIS BAR MUST MEASURE EXACTLY 200.0 mm")
        ruler(ax, 14.0, 32.0, 150.0, horizontal=True, thick=3.0,
              label="THIS BAR MUST MEASURE EXACTLY 150.0 mm")
        pdf.savefig(fig)
        plt.close(fig)

        # ---------------- page 2: the board, clean ----------------
        fig, ax = new_page(pdf, pw, ph)
        n_black = draw_board(ax, ox, oy, square_mm)
        margin_y = oy
        ruler_on_board_page = margin_y >= (square_mm + 14.0)
        if ruler_on_board_page:
            ruler(ax, ox, oy - square_mm - 10.0, 200.0, horizontal=True, thick=2.5,
                  label="SCALE CHECK: exactly 200.0 mm")
            ax.text(ox, ph - 8.0,
                    f"{INNER_COLS} x {INNER_ROWS} inner corners, {square_mm:.1f} mm squares "
                    f"-- print at 100% / actual size",
                    fontsize=7, va="top", color=MARK)
        pdf.savefig(fig)
        plt.close(fig)

    return {"paper_mm": [pw, ph], "square_mm": square_mm,
            "board_mm": [bw, bh], "origin_mm": [ox, oy], "n_black": n_black,
            "ruler_on_board_page": ruler_on_board_page, "command": cmd}


# ---------------------------------------------------------------------------


def verify(path, spec, tol_mm=0.5, tol_rel=1e-3):
    """`tol_mm` is the absolute ceiling; `tol_rel` (default 0.1 %) additionally
    binds each dimension to its own nominal size. Without the relative term a
    flat 0.5 mm tolerance on a 25 mm square is a 2 % scale licence -- and a 2 %
    scale error in the printed square is a 2 % error in fx and therefore in
    every range the campaign reports (a deliberate 0.4 % mutant passed the
    absolute-only check, 2026-07-25)."""
    checks = []

    def chk(name, ok, detail):
        checks.append({"check": name, "ok": bool(ok), "detail": detail})

    pages = pdf_measure.page_streams(path)
    chk("page_count", len(pages) == 2, f"{len(pages)} page(s) (1 instructions + 1 board)")
    pw, ph = spec["paper_mm"]
    for i, p in enumerate(pages):
        mw, mh = p["media_box_mm"]
        chk(f"page{i+1}_media_box_mm", abs(mw - pw) < 1e-3 and abs(mh - ph) < 1e-3,
            f"{mw:.4f} x {mh:.4f} mm (expect {pw} x {ph})")

    f0 = [f for f in pdf_measure.black_fills(pages[0]["content"]) if f.is_rect()]
    b200 = [f for f in f0 if abs(f.size[0] - 200.0) < tol_mm and f.size[1] < 6]
    b150 = [f for f in f0 if abs(f.size[0] - 150.0) < tol_mm and f.size[1] < 6]
    chk("instructions_bar_200mm", len(b200) == 1,
        f"measured {b200[0].size[0]:.4f} mm" if b200 else f"{len(b200)} candidates")
    chk("instructions_bar_150mm", len(b150) == 1,
        f"measured {b150[0].size[0]:.4f} mm" if b150 else f"{len(b150)} candidates")

    # --- the board page. Expectations come from the PINNED constants, never
    #     from the drawing code (see gen_placard_pdf's mutant note).
    sq = spec["square_mm"]
    exp_bw = (INNER_COLS + 1) * sq
    exp_bh = (INNER_ROWS + 1) * sq
    exp_n = ((INNER_COLS + 1) * (INNER_ROWS + 1) + 1) // 2
    ox = (pw - exp_bw) / 2.0
    oy = (ph - exp_bh) / 2.0

    squares = [f for f in pdf_measure.black_fills(pages[1]["content"])
               if f.is_rect() and (oy - 1e-6) <= (f.bbox[1] + f.bbox[3]) / 2 <= (oy + exp_bh + 1e-6)
               and (ox - 1e-6) <= (f.bbox[0] + f.bbox[2]) / 2 <= (ox + exp_bw + 1e-6)]
    chk("square_count", len(squares) == exp_n,
        f"{len(squares)} black squares (expect {exp_n} for a "
        f"{INNER_COLS+1}x{INNER_ROWS+1} chessboard)")
    if squares:
        t_sq = min(tol_mm, tol_rel * sq)
        t_bw = min(tol_mm, tol_rel * exp_bw)
        t_bh = min(tol_mm, tol_rel * exp_bh)
        werr = max(abs(f.size[0] - sq) for f in squares)
        herr = max(abs(f.size[1] - sq) for f in squares)
        chk("square_size_mm", werr <= t_sq and herr <= t_sq,
            f"every square {sq:.3f} mm within {max(werr, herr)*1000:.1f} um "
            f"(worst of {len(squares)}; tol {t_sq*1000:.0f} um = "
            f"{tol_rel*100:.2f}% of {sq:.0f} mm)")
        x0 = min(f.bbox[0] for f in squares)
        y0 = min(f.bbox[1] for f in squares)
        x1 = max(f.bbox[2] for f in squares)
        y1 = max(f.bbox[3] for f in squares)
        chk("board_extent_mm",
            abs((x1 - x0) - exp_bw) <= t_bw and abs((y1 - y0) - exp_bh) <= t_bh,
            f"measured {x1-x0:.4f} x {y1-y0:.4f} mm (expect {exp_bw:.1f} x {exp_bh:.1f}; "
            f"tol {t_bw*1000:.0f} / {t_bh*1000:.0f} um)")
        chk("board_centred_mm", abs(x0 - ox) <= tol_mm and abs(y0 - oy) <= tol_mm,
            f"origin ({x0:.3f}, {y0:.3f}) mm (expect ({ox:.3f}, {oy:.3f}))")
        sx, sy = (INNER_COLS - 1) * sq, (INNER_ROWS - 1) * sq
        chk("inner_corner_span_mm",
            abs((x1 - x0 - 2 * sq) - sx) <= min(tol_mm, tol_rel * sx)
            and abs((y1 - y0 - 2 * sq) - sy) <= min(tol_mm, tol_rel * sy),
            f"{(x1-x0-2*sq):.4f} x {(y1-y0-2*sq):.4f} mm "
            f"(expect {sx:.1f} x {sy:.1f}; tol {min(tol_mm, tol_rel*sx)*1000:.0f} um) -- "
            f"this is the span you measure with a steel rule")
    return checks


def detect_check(square_mm, dpi=150.0, tol_mm=0.5, out_png=None):
    """Rasterise the board and run the REAL cv2.findChessboardCorners on it."""
    import numpy as np
    try:
        import cv2
    except ImportError:
        return {"ran": False, "reason": "cv2 not installed"}
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    bw, bh = board_size(square_mm)
    quiet = 2.0 * square_mm
    W, H = bw + 2 * quiet, bh + 2 * quiet
    fig = Figure(figsize=(W / 25.4, H / 25.4), dpi=dpi)
    canvas = FigureCanvasAgg(fig)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_axis_off()
    for i in range(INNER_COLS + 1):
        for j in range(INNER_ROWS + 1):
            if (i + j) % 2 == 0:
                ax.add_patch(Rectangle((quiet + i * square_mm, quiet + j * square_mm),
                                       square_mm, square_mm, fc="black", ec="none", lw=0))
    canvas.draw()
    buf = np.asarray(canvas.buffer_rgba())[:, :, :3]
    gray = np.ascontiguousarray(
        (0.299 * buf[:, :, 0] + 0.587 * buf[:, :, 1] + 0.114 * buf[:, :, 2]).astype(np.uint8))
    if out_png:
        from PIL import Image
        Image.fromarray(gray).save(out_png)

    px_per_mm = gray.shape[1] / W
    ok, corners = cv2.findChessboardCorners(
        gray, (INNER_COLS, INNER_ROWS),
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return {"ran": True, "ok": False,
                "reason": f"findChessboardCorners({INNER_COLS}, {INNER_ROWS}) FAILED"}
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
    c = corners.reshape(INNER_ROWS, INNER_COLS, 2)
    dx = np.linalg.norm(c[:, 1:, :] - c[:, :-1, :], axis=2) / px_per_mm
    dy = np.linalg.norm(c[1:, :, :] - c[:-1, :, :], axis=2) / px_per_mm
    span_x = float(np.linalg.norm(c[0, -1] - c[0, 0])) / px_per_mm
    span_y = float(np.linalg.norm(c[-1, 0] - c[0, 0])) / px_per_mm
    err = max(abs(float(dx.mean()) - square_mm), abs(float(dy.mean()) - square_mm))
    return {
        "ran": True,
        "ok": bool(err <= tol_mm
                   and abs(span_x - (INNER_COLS - 1) * square_mm) <= tol_mm
                   and abs(span_y - (INNER_ROWS - 1) * square_mm) <= tol_mm),
        "raster_px": [int(gray.shape[1]), int(gray.shape[0])], "dpi": dpi,
        "px_per_mm": round(px_per_mm, 6),
        "n_corners": int(corners.shape[0]),
        "pitch_x_mm": [round(float(dx.mean()), 4), round(float(dx.std()), 4)],
        "pitch_y_mm": [round(float(dy.mean()), 4), round(float(dy.std()), 4)],
        "inner_span_mm": [round(span_x, 4), round(span_y, 4)],
        "expected_inner_span_mm": [(INNER_COLS - 1) * square_mm,
                                   (INNER_ROWS - 1) * square_mm],
        "max_pitch_err_mm": round(err, 4),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default=OUT_DIR)
    ap.add_argument("--tol-mm", type=float, default=0.5,
                    help="absolute ceiling for the PDF read-back (mm)")
    ap.add_argument("--tol-rel", type=float, default=1e-3,
                    help="relative tolerance per dimension (default 0.1%%)")
    ap.add_argument("--report", default=None)
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    print(f"[checker] PINNED pattern: {INNER_COLS} x {INNER_ROWS} INNER CORNERS "
          f"(docs/tripod_test_protocol.md:330-336, docs/field_bringup.md:325,")
    print(f"[checker]   scripts/calibrate_camera.py header, "
          f"docs/project_state.json:1061)")

    jobs = [
        ("checkerboard_9x6_25mm_a4.pdf", PAPERS_LANDSCAPE["a4"], 25.0),
        ("checkerboard_9x6_25mm_letter.pdf", PAPERS_LANDSCAPE["letter"], 25.0),
        ("checkerboard_9x6_25mm_a3.pdf", PAPERS_LANDSCAPE["a3"], 25.0),
        ("checkerboard_9x6_20mm_a4.pdf", PAPERS_LANDSCAPE["a4"], 20.0),
    ]
    report = {"inner_cols": INNER_COLS, "inner_rows": INNER_ROWS,
              "tol_mm": args.tol_mm, "tol_rel": args.tol_rel, "files": []}
    failed = 0
    for name, paper, sq in jobs:
        path = os.path.join(args.out_dir, name)
        spec = build(path, paper, sq)
        checks = verify(path, spec, tol_mm=args.tol_mm, tol_rel=args.tol_rel)
        ok = all(c["ok"] for c in checks)
        failed += 0 if ok else 1
        bw, bh = spec["board_mm"]
        mx = (paper[0] - bw) / 2.0
        my = (paper[1] - bh) / 2.0
        print(f"\n[checker] {name}  (paper {paper[0]:.1f} x {paper[1]:.1f} mm, "
              f"board {bw:.1f} x {bh:.1f} mm, quiet margin {mx:.1f} / {my:.1f} mm, "
              f"{os.path.getsize(path)/1024:.0f} kB)")
        if min(mx, my) < sq:
            print(f"[checker]   NOTE: short-axis margin {my:.1f} mm < one square "
                  f"({sq:.0f} mm). findChessboardCorners still works, but if your "
                  f"printer's unprintable edge eats it, use the A3 file or the "
                  f"20 mm A4 file.")
        for c in checks:
            print(f"[checker]   {'PASS' if c['ok'] else 'FAIL'}  {c['check']}: {c['detail']}")
        print(f"[checker]   command: {spec['command']}")
        print(f"[checker]   ==> {'PASS' if ok else 'FAIL'}")
        report["files"].append({"file": name, "paper_mm": list(paper), "square_mm": sq,
                                "board_mm": [bw, bh], "ok": ok, "checks": checks,
                                "command": spec["command"]})

    dc = detect_check(25.0, tol_mm=args.tol_mm,
                      out_png=os.path.join(args.out_dir, "checkerboard_9x6_25mm_proof.png"))
    report["detect_check_25mm"] = dc
    print("\n[checker] RASTER DETECT CHECK (real cv2.findChessboardCorners, 25 mm board)")
    if not dc.get("ran"):
        print(f"[checker]   SKIPPED: {dc.get('reason')}")
    else:
        if dc.get("ok") is False and "reason" in dc:
            print(f"[checker]   {dc['reason']}")
        else:
            print(f"[checker]   raster {dc['raster_px']} px @ {dc['dpi']:.0f} dpi "
                  f"({dc['px_per_mm']:.4f} px/mm); {dc['n_corners']} corners found "
                  f"(expect {INNER_COLS*INNER_ROWS})")
            print(f"[checker]   recovered pitch x {dc['pitch_x_mm'][0]:.4f} mm "
                  f"(sd {dc['pitch_x_mm'][1]:.4f}), y {dc['pitch_y_mm'][0]:.4f} mm "
                  f"(sd {dc['pitch_y_mm'][1]:.4f}); expect 25.0000")
            print(f"[checker]   recovered inner span {dc['inner_span_mm']} mm "
                  f"(expect {dc['expected_inner_span_mm']})")
        print(f"[checker]   ==> {'PASS' if dc.get('ok') else 'FAIL'}")
        if not dc.get("ok"):
            failed += 1

    report["all_ok"] = failed == 0
    rep = args.report or os.path.join(args.out_dir, "checkerboard_verification.json")
    with open(rep, "w") as fh:
        json.dump(report, fh, indent=1)
    print(f"\n[checker] report -> {os.path.relpath(rep, REPO)}")
    print(f"[checker] {'ALL VARIANTS PASS' if failed == 0 else f'{failed} FAILED'}")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
