#!/usr/bin/env python3
"""Plot the guidance-law campaign arc as a per-flight dot chart (SVG).

Six Gazebo campaigns, 8 flights each, all on the canonical 9 m/s crossing
(cross-range 6.5 m) and judged against the same 0.35 m contact radius:

  1. Pro-nav stock terminal, camera shown the target (PYAWTA, 2026-09-17) —
     the measurement that forced the chase switch (ADR-0103).
     Source: logs/mc_fp_armPYAWTA_line9_s123.csv (miss_m column).
  2. Chase pursuit first port (docs/xcheck_gazebo_pursuit_prereg.md).
  3. Chase, velocity-wiring bug fixed (prereg4, ADR-0117).
  4. Chase + brake, the flying config (prereg5, ADR-0118/0120).
  5./6. Flying config under injected cue error, ±5° / ±10° pooled
     (runs/pursuit_vs_dash_20261022/gazebo/results.csv).

Campaigns 2 and 3 of the five-campaign cross-check figure (range+timing,
brake-under-the-bug) are deliberately omitted here: interim tuning rungs, and
the prereg3 brake verdict was voided by the ADR-0117 wiring defect. The full
five-campaign record stays in docs/images/xcheck_campaigns.svg.

Fails closed: every campaign must parse to exactly 8 flights, and each median
must match the registered record to within 1 mm.

    python3 scripts/plot_campaign_arc.py [out.svg]
"""
import csv
import pathlib
import re
import statistics
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
CONTACT_M = 0.35  # ADR-0084 contact radius
INK, MUTED, ACC, RULE = "#18202C", "#69788C", "#2F5687", "#E5EAF0"

# Registered medians (m): doc-of-record value per campaign; the script refuses
# to draw if a re-parse of the sources disagrees by more than 1 mm.
EXPECTED = [3.601, 1.510, 0.343, 0.243, 0.259, 0.244]


def read_prereg_cpas(suffix):
    doc = ROOT / "docs" / f"xcheck_gazebo_pursuit_prereg{suffix}.md"
    text = doc.read_text()
    result = text[text.index("## RESULT"):]
    m = re.search(r"CPAs,? sorted \(m\):(.*?)Median", result, re.S | re.I)
    if not m:
        raise SystemExit(f"{doc.name}: no 'CPAs sorted (m):' line in RESULT")
    cpas = [float(x) for x in re.findall(r"\d+\.\d+", m.group(1))]
    if len(cpas) != 8:
        raise SystemExit(f"{doc.name}: expected 8 flights, parsed {len(cpas)}")
    return cpas


def read_pronav():
    path = ROOT / "logs" / "mc_fp_armPYAWTA_line9_s123.csv"
    with path.open() as f:
        rows = list(csv.DictReader(f))
    if len(rows) != 8 or any(r["law"] != "pronav" for r in rows):
        raise SystemExit(f"{path.name}: expected 8 pronav rows, got {len(rows)}")
    return [float(r["miss_m"]) for r in rows]


def read_cue_error(deg):
    path = ROOT / "runs" / "pursuit_vs_dash_20261022" / "gazebo" / "results.csv"
    with path.open() as f:
        rows = list(csv.DictReader(f))
    vals = [float(r["cpa_m"]) for r in rows if abs(int(r["err_deg"])) == deg]
    if len(vals) != 8:
        raise SystemExit(f"{path.name}: expected 8 flights at ±{deg}°, got {len(vals)}")
    if any(r["aborted"] != "0" for r in rows):
        raise SystemExit(f"{path.name}: aborted flight in the record")
    return vals


def svg(camps):
    W, H = 480, 322
    L, R, T, B = 36, 10, 30, 66
    pw, ph = W - L - R, H - T - B
    ymax = 5.5

    def y(v):
        return T + ph * (1 - v / ymax)

    colw = pw / len(camps)
    s = [f'<svg viewBox="0 0 {W} {H}" width="{W*2}" height="{H*2}" '
         'xmlns="http://www.w3.org/2000/svg" '
         'font-family="Inter, Helvetica, Arial, sans-serif">',
         '<title>Guidance-law arc: closest approach per flight, pro-nav to '
         'chase pursuit, then chase under injected cue error</title>',
         f'<rect width="{W}" height="{H}" fill="#ffffff"/>']
    for t in [0, 1, 2, 3, 4, 5]:
        s.append(f'<line x1="{L}" x2="{W-R}" y1="{y(t):.1f}" y2="{y(t):.1f}" '
                 f'stroke="{RULE}" stroke-width="0.8"/>')
        s.append(f'<text x="{L-6}" y="{y(t)+3:.1f}" font-size="8" fill="{MUTED}" '
                 f'text-anchor="end">{t:g}</text>')
    s.append(f'<text x="9" y="{T+ph/2:.1f}" font-size="8" fill="{MUTED}" text-anchor="middle" '
             f'transform="rotate(-90 9 {T+ph/2:.1f})">closest approach (m)</text>')
    s.append(f'<line x1="{L}" x2="{W-R}" y1="{y(CONTACT_M):.1f}" y2="{y(CONTACT_M):.1f}" '
             f'stroke="{ACC}" stroke-width="1.2" stroke-dasharray="4 3"/>')
    s.append(f'<text x="{L+6}" y="{y(CONTACT_M)-5:.1f}" font-size="8" fill="{ACC}" '
             f'font-weight="600">0.35 m contact radius</text>')
    # law-group separators + headers
    for gx in (1, 4):
        x = L + colw * gx
        s.append(f'<line x1="{x:.1f}" x2="{x:.1f}" y1="{T-18}" y2="{y(0):.1f}" '
                 f'stroke="{RULE}" stroke-width="0.8"/>')
    for label, c0, c1 in [("PRO-NAV TERMINAL", 0, 1),
                          ("CHASE PURSUIT · perfect launch cue", 1, 4),
                          ("CHASE · cue error injected", 4, 6)]:
        s.append(f'<text x="{L + colw*(c0+c1)/2:.1f}" y="{T-8}" font-size="7.5" '
                 f'fill="{MUTED}" text-anchor="middle" font-weight="600" '
                 f'letter-spacing="0.4">{label}</text>')
    for i, (name, sub, cpas) in enumerate(camps):
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
                 f'text-anchor="middle" font-weight="600">{name}</text>')
        s.append(f'<text x="{cx:.1f}" y="{H-B+25}" font-size="7.6" fill="{MUTED}" '
                 f'text-anchor="middle">{sub}</text>')
    s.append(f'<text x="{L}" y="{H-25}" font-size="6.8" fill="{MUTED}">'
             '48 flights, 8 per campaign · same 9 m/s crossing geometry and '
             '0.35 m bar throughout</text>')
    s.append(f'<text x="{L}" y="{H-16}" font-size="6.8" fill="{MUTED}">'
             'Filled dots are inside the contact radius · sim camera ~100° FOV '
             '(the 70° lens re-fly is queued, ADR-0126)</text>')
    s.append(f'<text x="{L}" y="{H-7}" font-size="6.8" fill="{MUTED}">'
             'Pro-nav arm scored by the batch scorer; chase campaigns by '
             'centre-to-centre CPA</text>')
    s.append("</svg>")
    return "\n".join(s) + "\n"


def main():
    out = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else \
        ROOT / "docs" / "images" / "campaign_arc.svg"
    camps = [
        ("Pro-nav", "stock terminal, Sep 17", read_pronav()),
        ("First port", "chase in Gazebo", read_prereg_cpas("")),
        ("Velocity bug", "found + fixed", read_prereg_cpas("4")),
        ("Flying config", "brake adopted", read_prereg_cpas("5")),
        ("±5° cue error", "flying config", read_cue_error(5)),
        ("±10° cue error", "flying config", read_cue_error(10)),
    ]
    for (name, _, cpas), exp in zip(camps, EXPECTED):
        med = statistics.median(cpas)
        if abs(med - exp) > 0.001:
            raise SystemExit(f"{name}: median {med:.4f} != registered {exp}")
    out.write_text(svg(camps))
    for i, (name, _, c) in enumerate(camps, 1):
        inside = sum(v <= CONTACT_M for v in c)
        print(f"campaign {i} ({name}): median {statistics.median(c):.3f} m, "
              f"{inside}/8 inside {CONTACT_M} m")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
