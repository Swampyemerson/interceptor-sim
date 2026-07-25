#!/usr/bin/env python3
"""Independent, dependency-free measurement of what a PDF actually DRAWS.

WHY THIS EXISTS (read this before you trust any placard/checkerboard print).
A mis-scaled fiducial silently invalidates every number that depends on it.
`pupil_apriltags` is told `tag_size` in metres; if the printed black square is
348 mm and we tell the decoder 350 mm, every tag-recovered range is ~0.6 % long
and NOTHING complains -- the decode still succeeds, the curve still plots, and
the money gate is scored against a quietly-wrong truth source. Same for the
checkerboard: `calibrate_camera.py --square 0.025` bakes the printed square
size straight into fx/fy, so a 3 % print scaling error is a 3 % error in every
range and bearing the whole campaign produces (docs/sim_to_real_gaps.md L4).

That is the project's "instruments are evidence" failure class (CLAUDE.md,
2026-07-25): a defect in a MEASUREMENT artifact cannot be caught by any paired
control, because both arms share the instrument. So the generator does not get
to assert its own correctness. This module re-opens the emitted PDF BYTES,
inflates the page content streams, walks the graphics operators, and reports
the geometry that will actually hit paper -- in millimetres.

WHAT IT PARSES. Only what the matplotlib PDF backend emits for filled patches:
  q Q cm                 graphics state + CTM
  m l c v y re h         path construction
  f f* F b b* B B*       fill operators (paths that put ink down)
  g rg k / G RG K        colour (non-stroking + stroking)
Stroked-only paths (`S`, `s`) are IGNORED on purpose: in the generators every
rule, crop mark and tick is a STROKE and every piece of real artwork (tag cells,
checkerboard squares, ruler bars) is a FILL, so "black fills" isolates the
artwork exactly. Text is drawn with font operators, not paths, so glyphs never
pollute a measurement (`pdf.fonttype = 42`).

UNITS. PDF user space is points (1 pt = 1/72 in). Everything returned here is
converted to millimetres: mm = pt * 25.4 / 72.

Self-test:  .venv/bin/python scripts/print_artifacts/pdf_measure.py --selftest
"""

from __future__ import annotations

import re
import zlib

PT_PER_MM = 72.0 / 25.4
MM_PER_PT = 25.4 / 72.0


# --------------------------------------------------------------------------
# minimal PDF object access
# --------------------------------------------------------------------------

_OBJ_RE = re.compile(rb"(\d+)\s+0\s+obj\b")


def _objects(raw: bytes) -> dict[int, tuple[int, int]]:
    """Map object number -> (start, end) byte span of its body."""
    hits = [(int(m.group(1)), m.end()) for m in _OBJ_RE.finditer(raw)]
    out: dict[int, tuple[int, int]] = {}
    for i, (num, start) in enumerate(hits):
        end = raw.find(b"endobj", start)
        if end < 0:
            end = hits[i + 1][1] if i + 1 < len(hits) else len(raw)
        out[num] = (start, end)
    return out


def _stream_bytes(raw: bytes, span: tuple[int, int]) -> bytes:
    """Return the (inflated) stream payload of the object at `span`."""
    start, end = span
    body = raw[start:end]
    m = re.search(rb"stream\r?\n", body)
    if not m:
        return b""
    s = start + m.end()
    e = raw.find(b"endstream", s)
    data = raw[s:e]
    if b"/FlateDecode" in body[: m.start()]:
        data = zlib.decompress(data)
    return data


def page_streams(path: str) -> list[dict]:
    """Return one dict per page: {'media_box_mm': (w,h), 'content': bytes}."""
    raw = open(path, "rb").read()
    objs = _objects(raw)
    pages = []
    for num, span in sorted(objs.items()):
        body = raw[span[0]: span[1]]
        head = body[:400]
        if b"/Type /Page" not in head or b"/Type /Pages" in head:
            continue
        mb = re.search(
            rb"/MediaBox\s*\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]", body)
        cm = re.search(rb"/Contents\s+(\d+)\s+0\s+R", body)
        if not (mb and cm):
            continue
        x0, y0, x1, y1 = (float(v) for v in mb.groups())
        content = _stream_bytes(raw, objs[int(cm.group(1))])
        pages.append({
            "media_box_mm": ((x1 - x0) * MM_PER_PT, (y1 - y0) * MM_PER_PT),
            "content": content,
        })
    return pages


# --------------------------------------------------------------------------
# content-stream walker
# --------------------------------------------------------------------------

_STRING_RE = re.compile(rb"\((?:\\.|[^\\()])*\)|<[0-9A-Fa-f\s]*>", re.S)


def _mul(a, b):
    """Multiply 2x3 affine matrices [a b c d e f] (PDF order): a then b."""
    a0, a1, a2, a3, a4, a5 = a
    b0, b1, b2, b3, b4, b5 = b
    return (
        a0 * b0 + a1 * b2, a0 * b1 + a1 * b3,
        a2 * b0 + a3 * b2, a2 * b1 + a3 * b3,
        a4 * b0 + a5 * b2 + b4, a4 * b1 + a5 * b3 + b5,
    )


def _apply(m, x, y):
    return (m[0] * x + m[2] * y + m[4], m[1] * x + m[3] * y + m[5])


class Fill:
    """One filled path, in millimetres, with its non-stroking colour."""

    __slots__ = ("pts", "gray")

    def __init__(self, pts, gray):
        self.pts = pts
        self.gray = gray

    @property
    def bbox(self):
        xs = [p[0] for p in self.pts]
        ys = [p[1] for p in self.pts]
        return (min(xs), min(ys), max(xs), max(ys))

    @property
    def size(self):
        x0, y0, x1, y1 = self.bbox
        return (x1 - x0, y1 - y0)

    def is_rect(self, tol=1e-6):
        """True if the path's vertices only take 2 distinct x and 2 distinct y."""
        xs = sorted({round(p[0], 6) for p in self.pts})
        ys = sorted({round(p[1], 6) for p in self.pts})
        return len(xs) == 2 and len(ys) == 2

    def __repr__(self):
        x0, y0, x1, y1 = self.bbox
        return (f"Fill(gray={self.gray:.2f}, x={x0:.3f}..{x1:.3f}, "
                f"y={y0:.3f}..{y1:.3f} mm)")


def fills(content: bytes) -> list[Fill]:
    """Walk a page content stream and return every FILLED path, in mm."""
    txt = _STRING_RE.sub(b" ", content)
    toks = txt.split()

    ctm = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    stack: list[tuple] = []
    gray = 0.0
    gray_stack: list[float] = []
    cur: list[tuple[float, float]] = []
    sub_start = None
    out: list[Fill] = []
    nums: list[float] = []

    def num(i):
        return nums[i]

    for tok in toks:
        try:
            nums.append(float(tok))
            if len(nums) > 8:
                nums.pop(0)
            continue
        except ValueError:
            pass
        op = tok.decode("latin1", "replace")

        if op == "q":
            stack.append(ctm)
            gray_stack.append(gray)
        elif op == "Q":
            if stack:
                ctm = stack.pop()
            if gray_stack:
                gray = gray_stack.pop()
        elif op == "cm" and len(nums) >= 6:
            ctm = _mul(tuple(nums[-6:]), ctm)
        elif op == "g" and len(nums) >= 1:
            gray = num(-1)
        elif op == "rg" and len(nums) >= 3:
            r, g_, b = nums[-3:]
            gray = (r + g_ + b) / 3.0
        elif op == "k" and len(nums) >= 4:
            c, m_, y_, k_ = nums[-4:]
            gray = (1 - min(1.0, c + k_) + 1 - min(1.0, m_ + k_) + 1 - min(1.0, y_ + k_)) / 3.0
        elif op == "m" and len(nums) >= 2:
            sub_start = _apply(ctm, num(-2), num(-1))
            cur.append(sub_start)
        elif op == "l" and len(nums) >= 2:
            cur.append(_apply(ctm, num(-2), num(-1)))
        elif op == "c" and len(nums) >= 6:
            for i in (-6, -4, -2):
                cur.append(_apply(ctm, nums[i], nums[i + 1]))
        elif op in ("v", "y") and len(nums) >= 4:
            for i in (-4, -2):
                cur.append(_apply(ctm, nums[i], nums[i + 1]))
        elif op == "re" and len(nums) >= 4:
            x, y, w, h = nums[-4:]
            for (px, py) in ((x, y), (x + w, y), (x + w, y + h), (x, y + h)):
                cur.append(_apply(ctm, px, py))
            sub_start = _apply(ctm, x, y)
        elif op == "h":
            if sub_start is not None:
                cur.append(sub_start)
        elif op in ("f", "F", "f*", "b", "b*", "B", "B*"):
            if cur:
                out.append(Fill([(x * MM_PER_PT, y * MM_PER_PT) for x, y in cur], gray))
            cur, sub_start = [], None
        elif op in ("S", "s", "n"):
            cur, sub_start = [], None
        nums = []

    return out


def black_fills(content: bytes, max_gray=0.10) -> list[Fill]:
    return [f for f in fills(content) if f.gray <= max_gray]


def black_bbox(content: bytes, region=None, max_gray=0.10):
    """Bounding box (mm) of all black ink on a page, optionally restricted to
    `region` = (x0, y0, x1, y1) in page mm (fills whose CENTRE is inside)."""
    sel = []
    for f in black_fills(content, max_gray):
        if region is not None:
            x0, y0, x1, y1 = f.bbox
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            if not (region[0] <= cx <= region[2] and region[1] <= cy <= region[3]):
                continue
        sel.append(f)
    if not sel:
        return None
    xs0 = min(f.bbox[0] for f in sel)
    ys0 = min(f.bbox[1] for f in sel)
    xs1 = max(f.bbox[2] for f in sel)
    ys1 = max(f.bbox[3] for f in sel)
    return (xs0, ys0, xs1, ys1)


# --------------------------------------------------------------------------
# self-test: draw a known rectangle, read it back, prove the loop closes
# --------------------------------------------------------------------------

def _selftest() -> int:
    import os
    import tempfile
    import matplotlib
    matplotlib.use("pdf")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    W, H = 210.0, 297.0
    truth = (37.0, 51.0, 123.456, 88.5)  # x, y, w, h in mm
    fig = plt.figure(figsize=(W / 25.4, H / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_axis_off()
    ax.add_patch(Rectangle(truth[:2], truth[2], truth[3], fc="black", ec="none"))
    ax.add_patch(Rectangle((5, 5), 30, 30, fc="0.5", ec="none"))     # grey decoy
    ax.plot([5, 205], [290, 290], color="black", lw=1)                # stroke decoy
    ax.text(10, 200, "decoy text 350.0 mm", color="black", fontsize=9)
    path = os.path.join(tempfile.mkdtemp(), "selftest.pdf")
    matplotlib.rcParams["pdf.fonttype"] = 42
    fig.savefig(path)
    plt.close(fig)

    pages = page_streams(path)
    assert len(pages) == 1, f"expected 1 page, got {len(pages)}"
    mw, mh = pages[0]["media_box_mm"]
    bb = black_bbox(pages[0]["content"])
    got = (bb[0], bb[1], bb[2] - bb[0], bb[3] - bb[1])
    errs = [abs(g - t) for g, t in zip(got, truth)]
    print(f"[pdf_measure] MediaBox  {mw:.4f} x {mh:.4f} mm  (expect {W} x {H})")
    print(f"[pdf_measure] truth rect {truth}")
    print(f"[pdf_measure] read  rect ({got[0]:.4f}, {got[1]:.4f}, {got[2]:.4f}, {got[3]:.4f})")
    print(f"[pdf_measure] max abs error {max(errs) * 1000:.4f} um")
    ok = (max(errs) < 1e-3 and abs(mw - W) < 1e-3 and abs(mh - H) < 1e-3)
    print("[pdf_measure] SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(_selftest())
    print(__doc__)
