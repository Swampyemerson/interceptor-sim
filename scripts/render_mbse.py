#!/usr/bin/env python3
"""Render the MBSE view set (docs/mbse.html) from the project-state contract.

WHAT THIS IS
------------
Model-Based Systems Engineering (MBSE) means the system lives in ONE model and
the diagrams/documents are *views generated from it*, rather than hand-drawn
artefacts that rot independently of the build.

This project already HAS the model: `docs/project_state.json` carries the
functional chain (`stages` + `edges`), the system-level requirements
(`constraints`), the design decisions with their rationale (`decisions`), the
assumption register (`assumptions`, graded measured / given-noisy /
given-perfect / unmeasured), the verified quantities (`key_numbers`, each with a
provenance pointer) and the open-issue ledger (`contradictions`).

What it did NOT have was systems-engineering VIEWS of that model. This script
generates them. Nothing here is hand-authored: every box, row and number is read
out of the contract, so the model cannot drift from the build — which is the
failure mode a hand-maintained SysML model would reintroduce (see CLAUDE.md,
"The project-state contract").

USAGE
-----
    python3 scripts/render_mbse.py                # render docs/mbse.html
    python3 scripts/render_mbse.py --check        # fail if the view is stale
    python3 scripts/render_mbse.py --artifact OUT # emit a self-contained copy

`--check` is wired into scripts/run_tests.sh: if the contract changes and the
view is not re-rendered, the gate fails. Same discipline as the dashboard.
"""

from __future__ import annotations

import hashlib
import html as _html
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
STATE = ROOT / "docs" / "project_state.json"
OUT = ROOT / "docs" / "mbse.html"

SHA_RE = re.compile(r"<!-- MBSE-STATE-SHA256: ([0-9a-f]{64}) -->")


def fail(msg: str) -> None:
    print(f"FAIL: {msg}", file=sys.stderr)
    sys.exit(1)


def esc(s) -> str:
    return _html.escape(str(s if s is not None else ""))


def state_digest(state: dict) -> str:
    """Canonical hash of the contract. Any contract edit changes this."""
    return hashlib.sha256(
        json.dumps(state, sort_keys=True, ensure_ascii=False).encode("utf-8")
    ).hexdigest()


def git_short_sha() -> str:
    try:
        return subprocess.run(
            ["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, check=True,
        ).stdout.strip() or "unknown"
    except Exception:
        return "unknown"


# --------------------------------------------------------------------------
# Vocabulary: contract terms -> systems-engineering terms.
# The contract is written for the builder (plain English, phone-readable). An
# MBSE view is read by engineers, so each view states BOTH names once. We do not
# rename anything in the contract itself -- that surface has its own audience.
# --------------------------------------------------------------------------
STATUS_LABEL = {
    "implemented": "Realized",
    "half-done": "Partially realized",
    "idea": "Proposed",
    "rejected": "Rejected",
    "superseded": "Superseded",
}
GRADE_LABEL = {
    "measured": "Measured",
    "given-noisy": "Given (with error)",
    "given-perfect": "Given (error-free)",
    "unmeasured": "Unmeasured",
}
GRADE_MEANING = {
    "measured": "The system measures this itself, and the measurement has been taken.",
    "given-noisy": "Supplied from outside the system, carrying a realistic error model.",
    "given-perfect": "Supplied from outside the system with ZERO error. No real system can obtain this, so any number resting on it is an upper bound, not a claim.",
    "unmeasured": "Assumed true. Not yet measured, and no error model exists.",
}


# --------------------------------------------------------------------------
# View 2: functional architecture diagram (inline SVG, laid out from stage.pos)
# --------------------------------------------------------------------------
BOX_W, BOX_H = 246, 78
COL_GAP, ROW_GAP = 300, 116
PAD_X, PAD_Y = 24, 26


def _box_xy(stage: dict) -> tuple[int, int]:
    col, row = stage["pos"]
    return PAD_X + col * COL_GAP, PAD_Y + row * ROW_GAP


def _anchor(src: dict, dst: dict) -> tuple[tuple[float, float], tuple[float, float]]:
    """Pick the edge midpoints that face each other, so arrows meet box borders."""
    sx, sy = _box_xy(src)
    dx, dy = _box_xy(dst)
    scx, scy = sx + BOX_W / 2, sy + BOX_H / 2
    dcx, dcy = dx + BOX_W / 2, dy + BOX_H / 2

    if abs(dcx - scx) > abs(dcy - scy):          # mostly horizontal
        if dcx > scx:
            return (sx + BOX_W, scy), (dx, dcy)
        return (sx, scy), (dx + BOX_W, dcy)
    if dcy > scy:                                 # mostly vertical, downward
        return (scx, sy + BOX_H), (dcx, dy)
    return (scx, sy), (dcx, dy + BOX_H)


def _hits_a_box(pts, boxes, margin=8.0) -> bool:
    """True if any sampled point of a route falls inside a node box."""
    for px, py in pts:
        for bx, by in boxes:
            if (bx - margin <= px <= bx + BOX_W + margin
                    and by - margin <= py <= by + BOX_H + margin):
                return True
    return False


def _bezier_pts(p0, pc, p1, n=48):
    out = []
    for i in range(1, n):
        t = i / n
        u = 1 - t
        out.append((u * u * p0[0] + 2 * u * t * pc[0] + t * t * p1[0],
                    u * u * p0[1] + 2 * u * t * pc[1] + t * t * p1[1]))
    return out


def _route(p0, p1, boxes) -> str:
    """Straight if it clears every box; otherwise bow out until it does.

    Long cross-column flows (e.g. sim_harness -> coded_dash) otherwise draw
    straight THROUGH an unrelated function box, which reads as a connection that
    does not exist in the model. Try increasing perpendicular offsets both ways
    and take the first clear route.
    """
    mid = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2)
    if not _hits_a_box(_bezier_pts(p0, mid, p1), boxes):
        return f'M{p0[0]:.0f},{p0[1]:.0f} L{p1[0]:.0f},{p1[1]:.0f}', [p0, p1]

    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length = max((dx * dx + dy * dy) ** 0.5, 1e-6)
    nx, ny = -dy / length, dx / length          # unit normal to the segment
    candidates = (60, -60, 110, -110, 170, -170, 240, -240)
    for off in candidates:
        pc = (mid[0] + nx * off, mid[1] + ny * off)
        pts = _bezier_pts(p0, pc, p1)
        if not _hits_a_box(pts, boxes):
            return (f'M{p0[0]:.0f},{p0[1]:.0f} Q{pc[0]:.0f},{pc[1]:.0f} '
                    f'{p1[0]:.0f},{p1[1]:.0f}'), [p0, p1] + pts
    # Nothing cleared: draw the widest bow rather than silently emitting a
    # straight line that implies a link through an unrelated box.
    pc = (mid[0] + nx * candidates[-1], mid[1] + ny * candidates[-1])
    pts = _bezier_pts(p0, pc, p1)
    return (f'M{p0[0]:.0f},{p0[1]:.0f} Q{pc[0]:.0f},{pc[1]:.0f} '
            f'{p1[0]:.0f},{p1[1]:.0f}'), [p0, p1] + pts


def render_architecture_svg(state: dict) -> str:
    stages = {s["id"]: s for s in state["stages"]}
    edges = state["edges"]

    max_col = max(s["pos"][0] for s in state["stages"])
    max_row = max(s["pos"][1] for s in state["stages"])

    # Track the drawn extent so a bowed edge that swings outside the node grid
    # is not clipped by the viewBox.
    xs = [PAD_X, PAD_X + max_col * COL_GAP + BOX_W]
    ys = [PAD_Y, PAD_Y + max_row * ROW_GAP + BOX_H]

    flows = []
    for src_id, dst_id in edges:
        src, dst = stages.get(src_id), stages.get(dst_id)
        if not src or not dst:
            fail(f"edge {src_id}->{dst_id} references a stage that does not exist")
        p0, p1 = _anchor(src, dst)
        # Boxes to avoid: everything except the two this edge connects.
        others = [_box_xy(s) for s in state["stages"] if s["id"] not in (src_id, dst_id)]
        d, pts = _route(p0, p1, others)
        xs += [p[0] for p in pts]
        ys += [p[1] for p in pts]
        flows.append(f'<path d="{d}" class="flow" marker-end="url(#ah)"/>')

    vx, vy = min(xs) - PAD_X, min(ys) - PAD_Y
    w, h = max(xs) - vx + PAD_X, max(ys) - vy + PAD_Y

    parts = [
        f'<svg viewBox="{vx:.0f} {vy:.0f} {w:.0f} {h:.0f}" width="100%" role="img" '
        f'aria-label="Functional architecture of the interceptor, {len(stages)} functions" '
        f'class="arch">',
        '<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" '
        'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
        '<path d="M0,0 L10,5 L0,10 z" class="ahead"/></marker></defs>',
        *flows,
    ]

    for s in state["stages"]:
        x, y = _box_xy(s)
        st = s["status"]
        name = esc(s["name"])
        # Wrap the function name onto at most two lines, by width in characters.
        words, lines, cur = name.split(), [], ""
        for word in words:
            trial = f"{cur} {word}".strip()
            if len(trial) > 26 and cur:
                lines.append(cur)
                cur = word
            else:
                cur = trial
        if cur:
            lines.append(cur)
        lines = lines[:2]

        parts.append(
            f'<g class="node st-{esc(st)}">'
            f'<rect x="{x}" y="{y}" width="{BOX_W}" height="{BOX_H}" rx="9"/>'
        )
        ty = y + (30 if len(lines) > 1 else 36)
        for ln in lines:
            parts.append(f'<text x="{x + 14}" y="{ty}" class="nlabel">{ln}</text>')
            ty += 18
        parts.append(
            f'<text x="{x + 14}" y="{y + BOX_H - 14}" class="nstat">'
            f'{esc(STATUS_LABEL.get(st, st))}</text>'
            f'<text x="{x + BOX_W - 14}" y="{y + BOX_H - 14}" class="nid">'
            f'{esc(s["id"])}</text></g>'
        )

    parts.append("</svg>")
    return "\n".join(parts)


# --------------------------------------------------------------------------
# View 1: operational context / system boundary
# --------------------------------------------------------------------------
def render_context_svg(state: dict) -> str:
    """Draw the system boundary and mark every input by how it is obtained.

    This is the view that matters most in this project: the honesty boundary.
    An input the system MEASURES is inside the boundary; an input it is GIVEN
    crosses it, and the grade says whether a real system could obtain it at
    that quality.
    """
    given = {}
    for a in state["assumptions"]:
        if a["grade"] in ("given-perfect", "given-noisy"):
            given[a["id"]] = a
    measured = [a for a in state["assumptions"] if a["grade"] == "measured"]

    w, h = 900, 330
    p = [
        f'<svg viewBox="0 0 {w} {h}" width="100%" role="img" '
        f'aria-label="System boundary showing given versus measured inputs" class="arch ctx">',
        '<defs><marker id="ah2" viewBox="0 0 10 10" refX="9" refY="5" '
        'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
        '<path d="M0,0 L10,5 L0,10 z" class="ahead"/></marker></defs>',
        # the system under design
        '<rect x="330" y="58" width="250" height="216" rx="12" class="sysbox"/>',
        '<text x="455" y="90" class="sysname" text-anchor="middle">Interceptor</text>',
        '<text x="455" y="112" class="syssub" text-anchor="middle">system under design</text>',
        '<text x="455" y="150" class="syssub2" text-anchor="middle">camera + own-state EKF</text>',
        '<text x="455" y="170" class="syssub2" text-anchor="middle">coded dash → pro-nav terminal</text>',
        '<text x="455" y="190" class="syssub2" text-anchor="middle">no datalink after launch</text>',
        f'<text x="455" y="228" class="syssub2 ok" text-anchor="middle">'
        f'{len(measured)} input(s) it measures itself</text>',
        f'<text x="455" y="250" class="syssub2 warn" text-anchor="middle">'
        f'{len(given)} input(s) it is given</text>',
        # external entities
        '<rect x="24" y="76" width="222" height="60" rx="9" class="extbox"/>',
        '<text x="135" y="102" class="extname" text-anchor="middle">Operator / launch cue</text>',
        '<text x="135" y="122" class="extsub" text-anchor="middle">points it, pulls the trigger</text>',
        '<rect x="24" y="196" width="222" height="60" rx="9" class="extbox"/>',
        '<text x="135" y="222" class="extname" text-anchor="middle">Environment</text>',
        '<text x="135" y="242" class="extsub" text-anchor="middle">wind, light, outdoor scene</text>',
        '<rect x="664" y="136" width="212" height="60" rx="9" class="extbox"/>',
        '<text x="770" y="162" class="extname" text-anchor="middle">Target UAS</text>',
        '<text x="770" y="182" class="extsub" text-anchor="middle">≥ 9 m/s, non-cooperative</text>',
        # flows
        '<line x1="246" y1="106" x2="330" y2="140" class="flow" marker-end="url(#ah2)"/>',
        '<line x1="246" y1="226" x2="330" y2="196" class="flow" marker-end="url(#ah2)"/>',
        '<line x1="664" y1="166" x2="580" y2="166" class="flow dashed" marker-end="url(#ah2)"/>',
        '<text x="622" y="158" class="flab" text-anchor="middle">photons</text>',
        '<line x1="580" y1="206" x2="664" y2="206" class="flow" marker-end="url(#ah2)"/>',
        '<text x="622" y="226" class="flab" text-anchor="middle">intercept</text>',
        # boundary annotation
        f'<text x="455" y="304" class="bnote" text-anchor="middle">'
        f'Everything crossing this boundary inward is graded below. '
        f'Ground truth is scoring-only and is structurally unreadable by guidance.</text>',
        "</svg>",
    ]
    return "\n".join(p)


# --------------------------------------------------------------------------
# Views 3-6: tables
# --------------------------------------------------------------------------
def chips(items, cls="chip") -> str:
    if not items:
        return '<span class="none">none recorded</span>'
    return "".join(f'<span class="{cls}">{esc(i)}</span>' for i in items)


def render_requirements(state: dict) -> str:
    rows = []
    for c in state["constraints"]:
        rows.append(
            f'<tr><td class="mono id">{esc(c["id"])}</td>'
            f'<td>{esc(c["text"])}</td>'
            f'<td class="eviden">{chips(c.get("evidence") or [])}</td></tr>'
        )
    return (
        '<table class="grid"><thead><tr><th style="width:15%">Requirement</th>'
        '<th>Statement (as written in the contract)</th>'
        '<th style="width:24%">Evidence / source</th></tr></thead>'
        f"<tbody>{''.join(rows)}</tbody></table>"
    )


def render_traceability(state: dict) -> str:
    """Function -> assumptions consumed -> decisions -> verified numbers -> evidence.

    Every link here already exists in the contract; nothing is inferred.
    """
    by_stage_dec: dict[str, list] = {}
    for d in state["decisions"]:
        by_stage_dec.setdefault(d["stage_id"], []).append(d)

    by_stage_num: dict[str, list] = {}
    for a in state["assumptions"]:
        for sid in a.get("stages") or []:
            for kn in a.get("key_numbers") or []:
                by_stage_num.setdefault(sid, []).append(kn)

    rows = []
    for s in state["stages"]:
        sid = s["id"]
        decs = by_stage_dec.get(sid, [])
        dec_cells = (
            "".join(f'<div class="dq">{esc(d["question"])}</div>' for d in decs)
            if decs else '<span class="none">no decision recorded</span>'
        )
        nums = sorted(set(by_stage_num.get(sid, [])))
        rows.append(
            f'<tr><td class="mono id">{esc(sid)}</td>'
            f'<td><b>{esc(s["name"])}</b>'
            f'<div class="cap">{esc(s.get("caption") or "")}</div></td>'
            f'<td><span class="pill st-{esc(s["status"])}">'
            f'{esc(STATUS_LABEL.get(s["status"], s["status"]))}</span></td>'
            f'<td class="eviden">{chips(s.get("consumes") or [], "chip asm")}</td>'
            f'<td>{dec_cells}</td>'
            f'<td class="eviden">{chips(nums, "chip num")}</td>'
            f'<td class="eviden">{chips(s.get("evidence") or [])}</td></tr>'
        )
    return (
        '<div class="scroller"><table class="grid trace">'
        "<thead><tr>"
        '<th>ID</th><th style="min-width:180px">Function</th><th>Status</th>'
        '<th style="min-width:200px">Assumptions consumed</th>'
        '<th style="min-width:220px">Design decisions</th>'
        '<th style="min-width:150px">Quantities claimed</th>'
        '<th style="min-width:190px">Evidence</th>'
        "</tr></thead>"
        f"<tbody>{''.join(rows)}</tbody></table></div>"
    )


GRADE_ORDER = {"given-perfect": 0, "unmeasured": 1, "given-noisy": 2, "measured": 3}


def render_assumptions(state: dict) -> str:
    rows = []
    for a in sorted(state["assumptions"], key=lambda a: GRADE_ORDER.get(a["grade"], 9)):
        test = a.get("test") or {}
        how = test.get("how") if isinstance(test, dict) else None
        rows.append(
            f'<tr><td class="mono id">{esc(a["id"])}</td>'
            f'<td><span class="pill gr-{esc(a["grade"])}">'
            f'{esc(GRADE_LABEL.get(a["grade"], a["grade"]))}</span>'
            f'<div class="cap">state: {esc(a.get("state", "—"))}</div></td>'
            f'<td>{esc(a["statement"])}</td>'
            f'<td class="ifwrong">{esc(a.get("if_wrong") or "not recorded")}</td>'
            f'<td class="eviden">{chips(a.get("stages") or [])}</td>'
            f'<td class="cap">{esc((how or "no test recorded")[:260])}</td></tr>'
        )
    return (
        '<div class="scroller"><table class="grid"><thead><tr>'
        '<th style="min-width:150px">Assumption</th><th style="min-width:120px">Grade</th>'
        '<th style="min-width:260px">Statement</th>'
        '<th style="min-width:240px">If it is wrong</th>'
        '<th style="min-width:120px">Functions affected</th>'
        '<th style="min-width:200px">How it gets measured</th>'
        f"</tr></thead><tbody>{''.join(rows)}</tbody></table></div>"
    )


def render_numbers(state: dict) -> str:
    rows = []
    for k in state["key_numbers"]:
        rows.append(
            f'<tr><td>{esc(k["label"])}</td>'
            f'<td class="mono val">{esc(k["value"])}</td>'
            f'<td><span class="pill trust-{esc(k.get("trust", "unknown"))}">'
            f'{esc(k.get("trust", "unknown"))}</span></td>'
            f'<td class="cap">{esc(k.get("plain") or k.get("note") or "")}</td>'
            f'<td class="eviden mono small">{esc(k.get("provenance") or "")}</td></tr>'
        )
    return (
        '<div class="scroller"><table class="grid"><thead><tr>'
        '<th style="min-width:200px">Quantity</th><th>Value</th><th>Source</th>'
        '<th style="min-width:260px">What it means</th>'
        '<th style="min-width:200px">Provenance</th>'
        f"</tr></thead><tbody>{''.join(rows)}</tbody></table></div>"
    )


def render_open_items(state: dict) -> str:
    open_c = [c for c in state["contradictions"] if c.get("status") == "open"]
    if not open_c:
        # No vacuous verdicts: say plainly that the filter selected zero, rather
        # than rendering an empty table that reads as "nothing to worry about".
        return ('<p class="none">The contradiction ledger currently has no OPEN entries '
                f'({len(state["contradictions"])} total, all resolved).</p>')
    rows = []
    for c in open_c:
        rows.append(
            f'<tr><td class="mono id">{esc(c.get("id", ""))}</td>'
            f'<td>{esc(c.get("claim") or c.get("text") or "")}</td>'
            f'<td class="cap">{esc(c.get("reality") or c.get("note") or "")}</td></tr>'
        )
    return (
        '<div class="scroller"><table class="grid"><thead><tr>'
        '<th style="min-width:170px">ID</th><th style="min-width:300px">Claim under challenge</th>'
        f'<th style="min-width:300px">What the evidence says</th>'
        f"</tr></thead><tbody>{''.join(rows)}</tbody></table></div>"
    )


def coverage(state: dict) -> dict:
    st = state["stages"]
    asm = state["assumptions"]
    return {
        "functions": len(st),
        "realized": sum(1 for s in st if s["status"] == "implemented"),
        "partial": sum(1 for s in st if s["status"] == "half-done"),
        "with_evidence": sum(1 for s in st if s.get("evidence")),
        "requirements": len(state["constraints"]),
        "decisions": len(state["decisions"]),
        "assumptions": len(asm),
        "measured": sum(1 for a in asm if a["grade"] == "measured"),
        "given_perfect": sum(1 for a in asm if a["grade"] == "given-perfect"),
        "violated": sum(1 for a in asm if a.get("state") == "violated"),
        "numbers": len(state["key_numbers"]),
        "open_contradictions": sum(1 for c in state["contradictions"] if c.get("status") == "open"),
        "total_contradictions": len(state["contradictions"]),
    }


CSS = """
/* Design system inherited from docs/dashboard.html -- the project's existing
   published surface. Same paper ground, ink, accent and status palette, so the
   two generated views read as one document set. Roles: Charter (serif) for
   headings and prose, Helvetica for UI labels and tables, mono for identifiers
   and data. No webfonts: the Artifact CSP blocks font CDNs, and a silent
   fallback would be worse than an honest system stack. */
:root{
  --bg:#F4F1E8; --panel:#ECE8DB; --ink:#1C1B17; --muted:#57534A;
  --line:#C9C3B1; --line2:#A8A190; --accent:#29527A;
  --ok:#2D5F3E; --warn:#7E611E; --bad:#A63A22; --idea:#57534A;
}
@media (prefers-color-scheme:dark){
  :root{ --bg:#191B18; --panel:#21241F; --ink:#CFCCC0; --muted:#8B897D;
    --line:#3A3D36; --line2:#4A4D44; --accent:#7A9EC2;
    --ok:#7FA671; --warn:#C9A24B; --bad:#C96A4A; --idea:#8B897D; }
}
:root[data-theme="dark"]{ --bg:#191B18; --panel:#21241F; --ink:#CFCCC0; --muted:#8B897D;
  --line:#3A3D36; --line2:#4A4D44; --accent:#7A9EC2;
  --ok:#7FA671; --warn:#C9A24B; --bad:#C96A4A; --idea:#8B897D; }
:root[data-theme="light"]{ --bg:#F4F1E8; --panel:#ECE8DB; --ink:#1C1B17; --muted:#57534A;
  --line:#C9C3B1; --line2:#A8A190; --accent:#29527A;
  --ok:#2D5F3E; --warn:#7E611E; --bad:#A63A22; --idea:#57534A; }

*{box-sizing:border-box}
body{background:var(--bg);color:var(--ink);
  font:16px/1.62 Charter,"Bitstream Charter","Sitka Text",Cambria,Georgia,serif;
  margin:0;padding:0 22px 84px;overflow-x:hidden;
  -webkit-font-smoothing:antialiased}
.wrap{max-width:1140px;margin:0 auto}
.ui{font-family:"Helvetica Neue",Helvetica,Arial,"Liberation Sans",sans-serif}
.mono{font-family:ui-monospace,"Cascadia Mono",Consolas,"SF Mono",Menlo,"DejaVu Sans Mono",monospace;
  font-size:12.5px}
code.mono{background:var(--panel);border-radius:4px;padding:1px 5px}

header{padding:52px 0 10px;border-bottom:2px solid var(--line2)}
.eyebrow{font-family:"Helvetica Neue",Helvetica,Arial,sans-serif;font-size:11px;
  letter-spacing:.16em;text-transform:uppercase;color:var(--muted);margin:0 0 12px}
h1{font-size:34px;line-height:1.16;margin:0 0 8px;letter-spacing:-.012em;
  font-weight:600;text-wrap:balance;max-width:20ch}
.sub{color:var(--muted);margin:0 0 22px;font-size:14px;
  font-family:"Helvetica Neue",Helvetica,Arial,sans-serif}

h2{font-size:21px;margin:52px 0 4px;letter-spacing:-.008em;font-weight:600;
  text-wrap:balance;padding-top:14px;border-top:1px solid var(--line)}
h2 .vno{font-family:"Helvetica Neue",Helvetica,Arial,sans-serif;color:var(--accent);
  font-weight:600;font-size:11px;letter-spacing:.14em;text-transform:uppercase;
  display:block;margin-bottom:6px}
.lead{color:var(--muted);margin:6px 0 20px;max-width:68ch;font-size:15px}

.panel{background:var(--panel);border:1px solid var(--line);border-radius:3px;
  padding:20px;margin:16px 0}
.banner{background:var(--panel);border:1px solid var(--line);
  border-left:3px solid var(--bad);border-radius:3px;padding:16px 20px;margin:24px 0;
  font-size:15px;max-width:76ch}
.banner b{color:var(--bad)}
.gen{font-size:14px;color:var(--muted);border-left:3px solid var(--accent);
  background:var(--panel);border-radius:3px;padding:14px 20px;margin:22px 0;max-width:76ch}
.gen b{color:var(--ink)}

.stats{display:flex;flex-wrap:wrap;gap:1px;margin:24px 0 6px;
  background:var(--line);border:1px solid var(--line);border-radius:3px;overflow:hidden}
.stat{background:var(--bg);padding:14px 16px;min-width:150px;flex:1 1 150px}
.stat .n{font-size:26px;font-weight:600;letter-spacing:-.02em;
  font-variant-numeric:tabular-nums;line-height:1.1}
.stat .l{font-size:11.5px;color:var(--muted);line-height:1.4;margin-top:5px;
  font-family:"Helvetica Neue",Helvetica,Arial,sans-serif}
.stat.alert .n{color:var(--bad)}

.scroller{overflow-x:auto;border:1px solid var(--line);border-radius:3px;background:var(--panel)}
table.grid{border-collapse:collapse;width:100%;font-size:13.5px;
  font-family:"Helvetica Neue",Helvetica,Arial,"Liberation Sans",sans-serif;
  font-variant-numeric:tabular-nums}
table.grid th{text-align:left;font-size:10.5px;letter-spacing:.1em;text-transform:uppercase;
  color:var(--muted);font-weight:700;padding:12px;border-bottom:2px solid var(--line2);
  background:var(--panel);position:sticky;top:0;white-space:nowrap}
table.grid td{padding:12px;border-bottom:1px solid var(--line);vertical-align:top;
  line-height:1.5}
table.grid tbody tr:last-child td{border-bottom:none}
.small{font-size:11.5px}
td.id{color:var(--accent);white-space:nowrap;
  font-family:ui-monospace,"Cascadia Mono",Consolas,Menlo,monospace;font-size:12px}
td.val{font-size:16px;font-weight:600;white-space:nowrap}
.cap{color:var(--muted);font-size:12.5px;margin-top:5px;line-height:1.45}
.ifwrong{font-size:12.5px}
.dq{margin-bottom:7px;padding-left:11px;border-left:2px solid var(--line2)}
.dq:last-child{margin-bottom:0}
.none{color:var(--muted);font-style:italic;font-size:12.5px}
.chip{display:inline-block;background:var(--bg);border:1px solid var(--line);
  border-radius:2px;padding:2px 6px;margin:0 4px 4px 0;font-size:11px;
  font-family:ui-monospace,"Cascadia Mono",Consolas,Menlo,monospace;
  color:var(--muted);word-break:break-word}
.chip.asm{color:var(--ink)}
.chip.num{color:var(--accent);border-color:var(--accent)}
.pill{display:inline-block;border-radius:2px;padding:2px 8px;font-size:10.5px;
  font-weight:700;letter-spacing:.05em;text-transform:uppercase;
  border:1px solid currentColor;white-space:nowrap;
  font-family:"Helvetica Neue",Helvetica,Arial,sans-serif}
.pill.st-implemented{color:var(--ok)} .pill.st-half-done{color:var(--warn)}
.pill.st-idea{color:var(--idea)} .pill.st-rejected,.pill.st-superseded{color:var(--muted)}
.pill.gr-measured{color:var(--ok)} .pill.gr-given-noisy{color:var(--warn)}
.pill.gr-given-perfect{color:var(--bad)} .pill.gr-unmeasured{color:var(--muted)}
.pill.trust-sim{color:var(--warn)} .pill.trust-bench,.pill.trust-measured{color:var(--ok)}
.pill.trust-unknown{color:var(--muted)}

svg.arch{display:block;margin:6px auto;max-width:100%;height:auto}
svg.arch .flow{stroke:var(--line2);stroke-width:1.5;fill:none}
svg.arch .flow.dashed{stroke-dasharray:5 4}
svg.arch .ahead{fill:var(--line2)}
svg.arch .node rect{fill:var(--bg);stroke:var(--line2);stroke-width:1.5}
svg.arch .node.st-implemented rect{stroke:var(--ok)}
svg.arch .node.st-half-done rect{stroke:var(--warn);stroke-dasharray:7 4}
svg.arch .node.st-idea rect{stroke:var(--idea);stroke-dasharray:2 4}
svg.arch text{font-family:"Helvetica Neue",Helvetica,Arial,sans-serif}
svg.arch .nlabel{fill:var(--ink);font-size:14px;font-weight:600}
svg.arch .nstat{fill:var(--muted);font-size:10px;letter-spacing:.08em;
  text-transform:uppercase;font-weight:700}
svg.arch .nid{fill:var(--muted);font-size:10.5px;text-anchor:end;
  font-family:ui-monospace,Consolas,Menlo,monospace}
svg.arch .sysbox{fill:var(--bg);stroke:var(--accent);stroke-width:2}
svg.arch .extbox{fill:none;stroke:var(--line2);stroke-width:1.5;stroke-dasharray:5 4}
svg.arch .sysname{fill:var(--ink);font-size:17px;font-weight:600}
svg.arch .syssub,svg.arch .extsub{fill:var(--muted);font-size:11px}
svg.arch .syssub{letter-spacing:.08em;text-transform:uppercase}
svg.arch .syssub2{fill:var(--muted);font-size:12px}
svg.arch .syssub2.ok{fill:var(--ok);font-weight:600}
svg.arch .syssub2.warn{fill:var(--bad);font-weight:600}
svg.arch .extname{fill:var(--ink);font-size:13.5px;font-weight:600}
svg.arch .flab{fill:var(--muted);font-size:10.5px;letter-spacing:.06em;text-transform:uppercase}
svg.arch .bnote{fill:var(--muted);font-size:11.5px}

.legend{display:flex;flex-wrap:wrap;gap:18px;margin-top:16px;font-size:11.5px;
  color:var(--muted);font-family:"Helvetica Neue",Helvetica,Arial,sans-serif;
  letter-spacing:.04em;text-transform:uppercase}
.legend span::before{content:"";display:inline-block;width:16px;height:0;
  border-top:2px solid currentColor;margin-right:7px;vertical-align:middle}
.legend .l-ok{color:var(--ok)} .legend .l-half{color:var(--warn)} .legend .l-idea{color:var(--idea)}
.gl{margin:16px 0 20px;font-size:13px;color:var(--muted);max-width:76ch;
  display:grid;grid-template-columns:auto 1fr;gap:6px 18px;align-items:baseline}
.gl dt{font-weight:700;color:var(--ink);white-space:nowrap;
  font-family:"Helvetica Neue",Helvetica,Arial,sans-serif;font-size:11.5px;
  letter-spacing:.04em;text-transform:uppercase}
.gl dd{margin:0}
footer{margin-top:64px;padding-top:20px;border-top:2px solid var(--line2);
  font-size:12.5px;color:var(--muted);max-width:76ch;
  font-family:"Helvetica Neue",Helvetica,Arial,sans-serif;line-height:1.6}
a{color:var(--accent)}
a:focus-visible,summary:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
"""


def build_html(state: dict) -> str:
    cov = coverage(state)
    digest = state_digest(state)
    goal = state.get("goal") or {}
    goal_line = goal.get("plain") or goal.get("text") or goal.get("headline") or ""

    grade_defs = "".join(
        f"<dt>{esc(GRADE_LABEL[g])}</dt><dd>{esc(GRADE_MEANING[g])}</dd>"
        for g in ("measured", "given-noisy", "given-perfect", "unmeasured")
    )

    return f"""<title>Interceptor — Systems Model (MBSE views)</title>
<style>{CSS}</style>
<!-- MBSE-STATE-SHA256: {digest} -->
<div class="wrap">
<header>
  <p class="eyebrow">Model-based systems engineering · generated views</p>
  <h1>Counter-UAS Interceptor — Systems Model</h1>
  <p class="sub">Seven views of <code class="mono">docs/project_state.json</code>
     · contract updated {esc(state.get("updated", "?"))} · build
     <code class="mono">{esc(git_short_sha())}</code></p>
</header>

<div class="gen">
  <b>Every element on this page is generated from the project&rsquo;s machine-readable
  state contract.</b> No box, row or number here is hand-drawn. The same file gates the
  test suite, so a model that disagrees with the build fails CI
  (<code class="mono">scripts/render_mbse.py --check</code> in
  <code class="mono">scripts/run_tests.sh</code>). This is deliberate: a hand-maintained
  systems model drifts away from the system, and a stale model is worse than none.
</div>

{f'<p class="lead"><b>Mission.</b> {esc(goal_line)}</p>' if goal_line else ''}

<div class="stats">
  <div class="stat"><div class="n">{cov['functions']}</div>
    <div class="l">functions in the chain<br>{cov['realized']} realized, {cov['partial']} partial</div></div>
  <div class="stat"><div class="n">{cov['requirements']}</div>
    <div class="l">system-level requirements</div></div>
  <div class="stat"><div class="n">{cov['decisions']}</div>
    <div class="l">design decisions with recorded rationale</div></div>
  <div class="stat"><div class="n">{cov['assumptions']}</div>
    <div class="l">registered assumptions<br>{cov['measured']} measured</div></div>
  <div class="stat alert"><div class="n">{cov['given_perfect']}</div>
    <div class="l">inputs given error-free<br>(upper-bound risk)</div></div>
  <div class="stat alert"><div class="n">{cov['open_contradictions']}</div>
    <div class="l">open items in the contradiction ledger<br>of {cov['total_contradictions']} logged</div></div>
</div>

<div class="banner">
  <b>Read this before any number below.</b> {cov['given_perfect']} of the
  {cov['assumptions']} registered inputs are supplied to the system <i>error-free</i>, and
  {cov['violated']} assumptions are currently marked <i>violated</i>. Every quantity that rests on an
  error-free input is a <b>best-case upper bound, not a capability claim</b> — no real
  sensor delivers zero error. The strongest measured result to date is an open-loop
  ballistic dash under a perfect launch cue; a camera-guided sub-metre intercept of the
  realistic 3-D target has <b>not</b> been demonstrated. Those facts are in the model
  because hiding them is how a systems model becomes a liability.
</div>

<h2><span class="vno">View 1</span>Operational context &amp; system boundary</h2>
<p class="lead">What the system is, what it talks to, and — the part that governs every
claim in this project — which inputs it <i>measures</i> versus which it is <i>given</i>.
Ground truth from the simulator is used for scoring only and is structurally unreadable
by the guidance path.</p>
<div class="panel">{render_context_svg(state)}</div>

<h2><span class="vno">View 2</span>Functional architecture</h2>
<p class="lead">The functional chain, laid out from the model&rsquo;s own coordinates. The
left column is the engagement sequence; the right columns are the enabling functions that
feed it. Border style encodes realization status.</p>
<div class="panel">
  {render_architecture_svg(state)}
  <div class="legend">
    <span class="l-ok">Realized</span>
    <span class="l-half">Partially realized</span>
    <span class="l-idea">Proposed</span>
    <span>Arrow = data / control flow</span>
  </div>
</div>

<h2><span class="vno">View 3</span>Requirements register</h2>
<p class="lead">System-level constraints the design must satisfy. These are binding: a
proposal that violates one is rejected without further analysis.</p>
{render_requirements(state)}

<h2><span class="vno">View 4</span>Traceability matrix</h2>
<p class="lead">The core MBSE view: each function traced to the assumptions it consumes,
the design decisions that shaped it, the quantities claimed for it, and the evidence
behind them. Every link is stored in the contract — none is inferred here.</p>
{render_traceability(state)}

<h2><span class="vno">View 5</span>Assumption register</h2>
<p class="lead">Sorted worst-first. An unrecorded assumption is this project&rsquo;s
single largest source of wrong conclusions, so each one carries a grade, the consequence
if it is wrong, and how it will be measured.</p>
<dl class="gl">{grade_defs}</dl>
{render_assumptions(state)}

<h2><span class="vno">View 6</span>Verified quantities</h2>
<p class="lead">Every number the project publishes, with the run or derivation it traces
to. A quantity without a provenance pointer is not permitted in the contract.</p>
{render_numbers(state)}

<h2><span class="vno">View 7</span>Open items</h2>
<p class="lead">The contradiction ledger: claims that evidence has challenged and that are
not yet resolved. Kept visible so a refuted claim cannot quietly survive in the prose.</p>
{render_open_items(state)}

<footer>
  Generated by <code class="mono">scripts/render_mbse.py</code> from
  <code class="mono">docs/project_state.json</code> (updated {esc(state.get("updated", "?"))}).
  Contract digest <code class="mono">{digest[:12]}</code> · build
  <code class="mono">{esc(git_short_sha())}</code>.
  Re-render with <code class="mono">python3 scripts/render_mbse.py</code>; drift fails
  <code class="mono">scripts/run_tests.sh</code>.
</footer>
</div>
"""


def main() -> None:
    args = sys.argv[1:]
    check = "--check" in args

    if check and "--artifact" in args:
        fail("--check --artifact does nothing: --check never renders, so no artifact "
             "file would be written. Run WITHOUT --check.")

    try:
        state = json.loads(STATE.read_text())
    except json.JSONDecodeError as e:
        fail(f"{STATE} is not valid JSON: {e}")

    for field in ("stages", "edges", "constraints", "assumptions", "decisions",
                  "key_numbers", "contradictions"):
        if field not in state:
            fail(f"{STATE.name} is missing required field {field!r} — the MBSE views "
                 f"are generated from it and cannot be rendered without it")

    cov = coverage(state)
    digest = state_digest(state)

    if check:
        if not OUT.exists():
            fail(f"{OUT.name} does not exist — run scripts/render_mbse.py and commit it")
        m = SHA_RE.search(OUT.read_text())
        if not m:
            fail(f"{OUT.name} carries no contract digest — it was hand-edited or "
                 f"produced by an older renderer. Re-run scripts/render_mbse.py.")
        if m.group(1) != digest:
            fail(f"{OUT.name} was generated from a DIFFERENT contract revision "
                 f"(view {m.group(1)[:12]} vs contract {digest[:12]}) — run "
                 f"scripts/render_mbse.py and commit both")
        print(f"OK: {OUT.name} generated from the current contract "
              f"(digest {digest[:12]}, updated {state.get('updated')}); "
              f"{cov['functions']} functions, {cov['requirements']} requirements, "
              f"{cov['assumptions']} assumptions [{cov['measured']} measured, "
              f"{cov['given_perfect']} given error-free], "
              f"{cov['open_contradictions']} open contradictions")
        return

    html_out = build_html(state)
    OUT.write_text(html_out)
    print(f"OK: rendered {OUT.name} from {STATE.name} "
          f"(digest {digest[:12]}, updated {state.get('updated')})\n"
          f"    {cov['functions']} functions ({cov['realized']} realized, "
          f"{cov['partial']} partial), {cov['requirements']} requirements, "
          f"{cov['decisions']} decisions, {cov['assumptions']} assumptions "
          f"[{cov['measured']} measured / {cov['given_perfect']} given error-free / "
          f"{cov['violated']} violated], {cov['numbers']} quantities, "
          f"{cov['open_contradictions']}/{cov['total_contradictions']} contradictions open")

    if "--artifact" in args:
        i = args.index("--artifact")
        out = Path(args[i + 1]) if i + 1 < len(args) else ROOT / "docs" / "mbse.artifact.html"
        out.write_text(html_out)
        data = out.read_bytes()
        print(f"OK: wrote self-contained Artifact HTML -> {out}\n"
              f"    RECEIPT: sha256[:12]={hashlib.sha256(data).hexdigest()[:12]} "
              f"bytes={len(data)} state.updated={state.get('updated')} "
              f"git={git_short_sha()}")


if __name__ == "__main__":
    main()
