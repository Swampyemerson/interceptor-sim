#!/usr/bin/env python3
"""Render/verify the project-state dashboard.

The CONTRACT is docs/project_state.json (hand-edited, small).
The HUMAN VIEW is docs/dashboard.html, which embeds a copy of that JSON
between marker comments so it opens from file:// with zero network deps.

  python3 scripts/render_dashboard.py           # validate JSON, inject into the HTML
  python3 scripts/render_dashboard.py --check   # exit 1 if HTML's embedded state != the JSON file

Ritual: edit the JSON -> run this script -> commit BOTH files together.
--check is the drift alarm (safe to wire into run_tests.sh).
"""
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
STATE = ROOT / "docs" / "project_state.json"
HTML = ROOT / "docs" / "dashboard.html"

BEGIN = "<!-- PROJECT_STATE_JSON:BEGIN (generated - edit docs/project_state.json and run scripts/render_dashboard.py; never hand-edit this block) -->"
END = "<!-- PROJECT_STATE_JSON:END -->"

STATUSES = {"implemented", "half-done", "idea", "rejected", "superseded"}
STAGE_KEYS = {"id", "name", "pos", "status", "active", "note", "evidence", "changelog"}


def fail(msg: str) -> None:
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


def validate(state: dict) -> None:
    for key in ("schema_version", "updated", "goal", "headline", "stages", "edges", "constraints", "graveyard"):
        if key not in state:
            fail(f"missing top-level key: {key}")
    if not re.fullmatch(r"\d{4}-\d{2}-\d{2}", state["updated"]):
        fail(f"'updated' must be YYYY-MM-DD, got {state['updated']!r}")
    ids = []
    for st in state["stages"]:
        missing = STAGE_KEYS - set(st)
        if missing:
            fail(f"stage {st.get('id', '?')!r} missing keys: {sorted(missing)}")
        if st["status"] not in STATUSES:
            fail(f"stage {st['id']!r} has status {st['status']!r}; allowed: {sorted(STATUSES)}")
        if not (isinstance(st["pos"], list) and len(st["pos"]) == 2 and all(isinstance(v, int) for v in st["pos"])):
            fail(f"stage {st['id']!r} pos must be [row, col] ints")
        if not st["evidence"]:
            fail(f"stage {st['id']!r} has no evidence pointer (numbers trace to a run or a derivation)")
        ids.append(st["id"])
    if len(ids) != len(set(ids)):
        fail("duplicate stage ids")
    for a, b in state["edges"]:
        if a not in ids or b not in ids:
            fail(f"edge [{a!r}, {b!r}] references an unknown stage id")
    for c in state["constraints"]:
        if not {"id", "text", "evidence"} <= set(c):
            fail(f"constraint {c.get('id', '?')!r} needs id/text/evidence")
    for g in state["graveyard"]:
        if not {"what", "why", "evidence"} <= set(g):
            fail("graveyard entries need what/why/evidence")


def embedded_block(html: str) -> str:
    i, j = html.find(BEGIN), html.find(END)
    if i < 0 or j < 0:
        fail(f"markers not found in {HTML}")
    return html[i + len(BEGIN):j]


def extract_embedded_json(html: str) -> dict:
    block = embedded_block(html)
    m = re.search(r'<script id="state" type="application/json">(.*)</script>', block, re.S)
    if not m:
        fail("embedded <script id=\"state\"> not found between markers")
    return json.loads(m.group(1))


def main() -> None:
    check = "--check" in sys.argv[1:]
    try:
        state = json.loads(STATE.read_text())
    except json.JSONDecodeError as e:
        fail(f"{STATE} is not valid JSON: {e}")
    validate(state)
    html = HTML.read_text()

    if check:
        if extract_embedded_json(html) != state:
            fail(f"{HTML.name} embedded state DIFFERS from {STATE.name} — run scripts/render_dashboard.py and commit both")
        print(f"OK: {STATE.name} valid; {HTML.name} in sync (updated {state['updated']}, {len(state['stages'])} stages)")
        return

    # '</' -> '<\/' keeps the payload valid JSON while making '</script>' inert.
    payload = json.dumps(state, indent=1, ensure_ascii=False).replace("</", "<\\/")
    injected = f'{BEGIN}\n<script id="state" type="application/json">{payload}</script>\n{END}'
    i, j = html.find(BEGIN), html.find(END)
    if i < 0 or j < 0:
        fail(f"markers not found in {HTML}")
    HTML.write_text(html[:i] + injected + html[j + len(END):])
    print(f"OK: rendered {HTML.name} from {STATE.name} (updated {state['updated']}, {len(state['stages'])} stages) — commit both files")


if __name__ == "__main__":
    main()
