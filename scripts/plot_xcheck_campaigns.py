#!/usr/bin/env python3
"""Plot the five Gazebo chase cross-check campaigns as a per-flight dot chart (SVG).

Reads the registered RESULT section of each docs/xcheck_gazebo_pursuit_prereg*.md
("CPAs sorted (m): ..." line) so the figure cannot drift from the scored numbers.
Fails closed: every campaign must yield exactly 8 flights.

    python3 scripts/plot_xcheck_campaigns.py [out.svg]
"""
import pathlib
import re
import statistics
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
CAMPAIGNS = [  # (doc suffix, short label for what changed)
    ("", "First port"),
    ("2", "Range + timing"),
    ("3", "Approach brake"),
    ("4", "Velocity bug"),
    ("5", "Brake, re-flown"),
]
CONTACT_M = 0.35  # ADR-0084 contact radius
INK, MUTED, ACC, RULE = "#18202C", "#69788C", "#2F5687", "#E5EAF0"


def read_cpas(suffix):
    doc = ROOT / "docs" / f"xcheck_gazebo_pursuit_prereg{suffix}.md"
    text = doc.read_text()
    result = text[text.index("## RESULT"):]
    m = re.search(r"CPAs,? sorted \(m\):(.*?)Median", result, re.S)
    if not m:
        raise SystemExit(f"{doc.name}: no 'CPAs sorted (m):' line in RESULT")
    cpas = [float(x) for x in re.findall(r"\d+\.\d+", m.group(1))]
    if len(cpas) != 8:
        raise SystemExit(f"{doc.name}: expected 8 flights, parsed {len(cpas)}")
    return cpas


def svg(camps):
    W, H = 420, 236
    L, R, T, B = 34, 10, 12, 42
    pw, ph = W - L - R, H - T - B
    ymax = 2.5

    def y(v):
        return T + ph * (1 - v / ymax)

    colw = pw / len(camps)
    s = [f'<svg viewBox="0 0 {W} {H}" width="{W*2}" height="{H*2}" '
         'xmlns="http://www.w3.org/2000/svg" '
         'font-family="Inter, Helvetica, Arial, sans-serif">',
         '<title>Gazebo chase cross-check: closest approach per flight, five campaigns</title>',
         f'<rect width="{W}" height="{H}" fill="#ffffff"/>']
    for t in [0, 0.5, 1.0, 1.5, 2.0, 2.5]:
        s.append(f'<line x1="{L}" x2="{W-R}" y1="{y(t):.1f}" y2="{y(t):.1f}" '
                 f'stroke="{RULE}" stroke-width="0.8"/>')
        s.append(f'<text x="{L-6}" y="{y(t)+3:.1f}" font-size="8" fill="{MUTED}" '
                 f'text-anchor="end">{t:g}</text>')
    s.append(f'<text x="9" y="{T+ph/2:.1f}" font-size="8" fill="{MUTED}" text-anchor="middle" '
             f'transform="rotate(-90 9 {T+ph/2:.1f})">closest approach (m)</text>')
    s.append(f'<line x1="{L}" x2="{W-R}" y1="{y(CONTACT_M):.1f}" y2="{y(CONTACT_M):.1f}" '
             f'stroke="{ACC}" stroke-width="1.2" stroke-dasharray="4 3"/>')
    s.append(f'<text x="{L+6}" y="{y(CONTACT_M)+11:.1f}" font-size="8" fill="{ACC}" '
             f'font-weight="600">0.35 m contact radius</text>')
    for i, (label, cpas) in enumerate(camps):
        cx = L + colw * (i + 0.5)
        med = statistics.median(cpas)
        for j, v in enumerate(sorted(cpas)):
            dx = (j % 2 * 2 - 1) * 5
            fill = ACC if v <= CONTACT_M else "#ffffff"
            s.append(f'<circle cx="{cx+dx:.1f}" cy="{y(v):.1f}" r="3.2" fill="{fill}" '
                     f'stroke="{ACC}" stroke-width="1.2"/>')
        s.append(f'<line x1="{cx-17:.1f}" x2="{cx+17:.1f}" y1="{y(med):.1f}" y2="{y(med):.1f}" '
                 f'stroke="{INK}" stroke-width="2" stroke-linecap="round"/>')
        s.append(f'<text x="{cx+20:.1f}" y="{y(med)+3:.1f}" font-size="8.5" fill="{INK}" '
                 f'font-weight="600">{med:.2f}</text>')
        s.append(f'<text x="{cx:.1f}" y="{H-B+14}" font-size="8.5" fill="{INK}" '
                 f'text-anchor="middle" font-weight="600">Campaign {i+1}</text>')
        s.append(f'<text x="{cx:.1f}" y="{H-B+25}" font-size="7.6" fill="{MUTED}" '
                 f'text-anchor="middle">{label}</text>')
    s.append("</svg>")
    return "\n".join(s) + "\n"


def main():
    out = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else \
        ROOT / "docs" / "images" / "xcheck_campaigns.svg"
    camps = [(label, read_cpas(sfx)) for sfx, label in CAMPAIGNS]
    out.write_text(svg(camps))
    for i, (label, c) in enumerate(camps, 1):
        inside = sum(v <= CONTACT_M for v in c)
        print(f"campaign {i} ({label}): median {statistics.median(c):.3f} m, "
              f"{inside}/8 inside {CONTACT_M} m")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
