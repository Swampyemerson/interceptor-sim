#!/usr/bin/env python3
"""Render/verify the project-state dashboard (schema v2).

The CONTRACT is docs/project_state.json (hand-edited, small-ish).
The HUMAN VIEW is docs/dashboard.html, which embeds a copy of that JSON
between marker comments so it opens from file:// with zero network deps.

  python3 scripts/render_dashboard.py           # validate JSON, inject into the HTML
  python3 scripts/render_dashboard.py --check   # exit 1 if HTML's embedded state != the JSON file

Schema v2 adds (all validated here):
  key_numbers    — the instrument-cluster figures (label/value/provenance; numbers trace to a run)
  decisions      — per-stage decision records (stage_id/question/options[]/chosen_rationale/evidence)
  contradictions — the doc-consistency flag ledger (id/topic/severity/status open|resolved/claims/locs/current_truth)
Schema v2.1 adds (2026-07-17 clarity pass, all validated here):
  architecture   — the at-a-glance data-flow strip (summary + steps[] of code/name/role + evidence)
  bom_tiers      — the staged buy list (tier/name/purpose/gate/total/items[] of item/qty/price/why)
Schema v2.2 adds (2026-07-17 build-plan pass, validated here):
  build_plan     — the IN-PERSON build/validate ladder rendered as the SECOND flowchart (§2.2):
                   summary + phases[] of code/name/cost/tasks[] (+ optional gate = the go/no-go
                   money logic; at least one phase MUST carry a gate) + evidence
Schema v2.3 adds (2026-07-20 Tier-1 order pass, validated here):
  build_tab      — the workbench Build sheet (dashboard tab bar State|Build; #build in the
                   URL hash bookmarks it): scope + subsystems[] (id/name/role + parts[] of
                   name/status/role/notes with status ordered|must-add|print + connections[]
                   of from/to/medium + steps[] of id/text with optional gate/why) + ladder
                   (summary/rungs[]/gate; rung refs point at step ids) + evidence.
                   Step ids are UNIQUE across ALL subsystems — they key the Build sheet's
                   localStorage checkboxes — and at least one step MUST carry gate=true
                   (the hard stop-points: smoke stopper, props-off, calibration, legal).

The renderer also stamps the live git short-sha into the title block between
REV:BEGIN/END markers (--check ignores the REV stamp; only the state block is compared).

Ritual: edit the JSON -> run this script -> commit BOTH files together.
--check is the drift alarm (wired into run_tests.sh).
"""
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
STATE = ROOT / "docs" / "project_state.json"
HTML = ROOT / "docs" / "dashboard.html"

BEGIN = "<!-- PROJECT_STATE_JSON:BEGIN (generated - edit docs/project_state.json and run scripts/render_dashboard.py; never hand-edit this block) -->"
END = "<!-- PROJECT_STATE_JSON:END -->"
REV_RE = re.compile(r"(<!-- REV:BEGIN -->)(.*?)(<!-- REV:END -->)", re.S)

STATUSES = {"implemented", "half-done", "idea", "rejected", "superseded"}
STAGE_KEYS = {"id", "name", "pos", "status", "active", "note", "evidence", "changelog"}
OPTION_STATUSES = {"chosen", "rejected", "deferred", "superseded"}
OPTION_KEYS = {"name", "summary", "why_choose", "pros", "cons", "status"}
DECISION_KEYS = {"stage_id", "question", "options", "chosen_rationale", "evidence"}
SEVERITIES = {"high", "medium", "low"}
FLAG_STATUSES = {"open", "resolved"}
CONTRA_KEYS = {"id", "topic", "severity", "status", "claim_a", "loc_a", "claim_b", "loc_b", "current_truth"}
KEYNUM_KEYS = {"label", "value", "provenance"}
ARCH_KEYS = {"summary", "steps", "evidence"}
ARCH_STEP_KEYS = {"code", "name", "role"}
BOM_TIER_KEYS = {"tier", "name", "purpose", "total", "items"}
BOM_ITEM_KEYS = {"item", "qty", "price", "why"}
BUILD_PLAN_KEYS = {"summary", "phases", "evidence"}
BP_PHASE_KEYS = {"code", "name", "cost", "tasks"}
BUILD_TAB_KEYS = {"scope", "subsystems", "ladder", "evidence"}
BT_SUB_KEYS = {"id", "name", "role", "parts", "connections", "steps"}
BT_PART_KEYS = {"name", "status", "role"}
BT_PART_STATUSES = {"ordered", "must-add", "print"}
BT_CONN_KEYS = {"from", "to", "medium"}
BT_STEP_KEYS = {"id", "text"}
BT_LADDER_KEYS = {"summary", "rungs", "gate"}
BT_RUNG_KEYS = {"code", "name", "goal", "refs"}


def fail(msg: str) -> None:
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


def validate(state: dict) -> None:
    for key in ("schema_version", "updated", "goal", "headline", "architecture", "build_plan",
                "stages", "edges", "constraints", "graveyard", "key_numbers", "bom_tiers",
                "build_tab", "decisions", "contradictions"):
        if key not in state:
            fail(f"missing top-level key: {key}")
    bt = state["build_tab"]
    missing = BUILD_TAB_KEYS - set(bt)
    if missing:
        fail(f"build_tab missing keys: {sorted(missing)}")
    if not bt["subsystems"]:
        fail("build_tab has no subsystems")
    bt_sub_ids, bt_step_ids, bt_any_gate = [], [], False
    for ss in bt["subsystems"]:
        missing = BT_SUB_KEYS - set(ss)
        if missing:
            fail(f"build_tab subsystem {ss.get('id', '?')!r} missing keys: {sorted(missing)}")
        bt_sub_ids.append(ss["id"])
        if not ss["parts"] or not ss["connections"] or not ss["steps"]:
            fail(f"build_tab subsystem {ss['id']!r} needs non-empty parts, connections and steps")
        for p in ss["parts"]:
            missing = BT_PART_KEYS - set(p)
            if missing:
                fail(f"build_tab part {p.get('name', '?')!r} missing keys: {sorted(missing)}")
            if p["status"] not in BT_PART_STATUSES:
                fail(f"build_tab part {p['name']!r} has status {p['status']!r}; allowed: {sorted(BT_PART_STATUSES)}")
        for c in ss["connections"]:
            missing = BT_CONN_KEYS - set(c)
            if missing:
                fail(f"build_tab connection in {ss['id']!r} missing keys: {sorted(missing)}")
        for st in ss["steps"]:
            missing = BT_STEP_KEYS - set(st)
            if missing:
                fail(f"build_tab step {st.get('id', '?')!r} ({ss['id']}) missing keys: {sorted(missing)}")
            bt_step_ids.append(st["id"])
            if st.get("gate"):
                bt_any_gate = True
    if len(bt_sub_ids) != len(set(bt_sub_ids)):
        fail("duplicate build_tab subsystem ids")
    if len(bt_step_ids) != len(set(bt_step_ids)):
        fail("duplicate build_tab step ids (they key the Build sheet's localStorage checkboxes)")
    if not bt_any_gate:
        fail("build_tab has no gate step — the hard stop-points (smoke stopper, props-off, calibration) must be explicit")
    ladder = bt["ladder"]
    missing = BT_LADDER_KEYS - set(ladder)
    if missing:
        fail(f"build_tab ladder missing keys: {sorted(missing)}")
    if not ladder["rungs"]:
        fail("build_tab ladder has no rungs")
    bt_sid_set = set(bt_step_ids)
    for r in ladder["rungs"]:
        missing = BT_RUNG_KEYS - set(r)
        if missing:
            fail(f"build_tab ladder rung {r.get('code', '?')!r} missing keys: {sorted(missing)}")
        if not r["refs"]:
            fail(f"build_tab ladder rung {r['code']!r} has no step refs")
        for ref in r["refs"]:
            if ref not in bt_sid_set:
                fail(f"build_tab ladder rung {r['code']!r} references unknown step id {ref!r}")
    bp = state["build_plan"]
    missing = BUILD_PLAN_KEYS - set(bp)
    if missing:
        fail(f"build_plan missing keys: {sorted(missing)}")
    if not bp["phases"]:
        fail("build_plan has no phases")
    for ph in bp["phases"]:
        missing = BP_PHASE_KEYS - set(ph)
        if missing:
            fail(f"build_plan phase {ph.get('code', '?')!r} missing keys: {sorted(missing)}")
        if not ph["tasks"] or not all(isinstance(t, str) and t for t in ph["tasks"]):
            fail(f"build_plan phase {ph.get('code', '?')!r} needs a non-empty list of task strings")
    if not any(ph.get("gate") for ph in bp["phases"]):
        fail("build_plan has no gates — the go/no-go money logic must be explicit")
    arch = state["architecture"]
    missing = ARCH_KEYS - set(arch)
    if missing:
        fail(f"architecture missing keys: {sorted(missing)}")
    if not arch["steps"]:
        fail("architecture has no steps")
    for st in arch["steps"]:
        missing = ARCH_STEP_KEYS - set(st)
        if missing:
            fail(f"architecture step {st.get('code', '?')!r} missing keys: {sorted(missing)}")
    for t in state["bom_tiers"]:
        missing = BOM_TIER_KEYS - set(t)
        if missing:
            fail(f"bom_tier {t.get('name', '?')!r} missing keys: {sorted(missing)}")
        if not t["items"]:
            fail(f"bom_tier {t.get('name', '?')!r} has no items")
        for it in t["items"]:
            missing = BOM_ITEM_KEYS - set(it)
            if missing:
                fail(f"bom item {it.get('item', '?')!r} missing keys: {sorted(missing)}")
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
    id_set = set(ids)
    for a, b in state["edges"]:
        if a not in id_set or b not in id_set:
            fail(f"edge [{a!r}, {b!r}] references an unknown stage id")
    for c in state["constraints"]:
        if not {"id", "text", "evidence"} <= set(c):
            fail(f"constraint {c.get('id', '?')!r} needs id/text/evidence")
        if not isinstance(c["evidence"], list) or not c["evidence"]:
            fail(f"constraint {c['id']!r} evidence must be a non-empty LIST "
                 "(a bare string broke the rendered constraints table, caught 2026-07-17)")
    for g in state["graveyard"]:
        if not {"what", "why", "evidence"} <= set(g):
            fail("graveyard entries need what/why/evidence")
    for kn in state["key_numbers"]:
        missing = KEYNUM_KEYS - set(kn)
        if missing:
            fail(f"key_number {kn.get('label', '?')!r} missing keys: {sorted(missing)} (numbers trace to a run)")
    for d in state["decisions"]:
        missing = DECISION_KEYS - set(d)
        if missing:
            fail(f"decision for stage {d.get('stage_id', '?')!r} missing keys: {sorted(missing)}")
        if d["stage_id"] not in id_set:
            fail(f"decision references unknown stage_id {d['stage_id']!r}")
        if not d["options"]:
            fail(f"decision {d['question'][:50]!r} has no options")
        for o in d["options"]:
            missing = OPTION_KEYS - set(o)
            if missing:
                fail(f"option {o.get('name', '?')!r} missing keys: {sorted(missing)}")
            if o["status"] not in OPTION_STATUSES:
                fail(f"option {o['name']!r} has status {o['status']!r}; allowed: {sorted(OPTION_STATUSES)}")
    cids = []
    for c in state["contradictions"]:
        missing = CONTRA_KEYS - set(c)
        if missing:
            fail(f"contradiction {c.get('id', '?')!r} missing keys: {sorted(missing)}")
        if c["severity"] not in SEVERITIES:
            fail(f"contradiction {c['id']!r} severity {c['severity']!r}; allowed: {sorted(SEVERITIES)}")
        if c["status"] not in FLAG_STATUSES:
            fail(f"contradiction {c['id']!r} status {c['status']!r}; allowed: {sorted(FLAG_STATUSES)}")
        cids.append(c["id"])
    if len(cids) != len(set(cids)):
        fail("duplicate contradiction ids")


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


def emit_artifact(html: str, out: Path) -> None:
    """Write a claude.ai-Artifact-flavored copy of the rendered dashboard: the
    Artifact publisher supplies its own <!doctype>/<html>/<head>/<body> skeleton,
    so strip our wrappers and keep the <style> (from <head>) + the <body> inner
    (which already holds the embedded state JSON + the render script). Self-
    contained; only the SVG-namespace URI remains. Publish with the Artifact tool
    (url = state['artifact_url'] to keep the same link)."""
    styles = "\n".join(re.findall(r"<style\b.*?</style>", html, re.S))
    body = re.search(r"<body\b[^>]*>(.*)</body>", html, re.S)
    if not body:
        fail("rendered HTML has no <body> to extract for --artifact")
    tm = re.search(r"<title\b[^>]*>(.*?)</title>", html, re.S)
    title = f"<title>{tm.group(1)}</title>\n" if tm else ""
    out.write_text(title + styles + "\n" + body.group(1))


def git_short_sha() -> str:
    try:
        r = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT,
                           capture_output=True, text=True, timeout=10)
        sha = r.stdout.strip()
        return sha if r.returncode == 0 and sha else "n/a"
    except Exception:
        return "n/a"


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
        n_open = sum(1 for c in state["contradictions"] if c["status"] == "open")
        print(f"OK: {STATE.name} valid; {HTML.name} in sync (updated {state['updated']}, "
              f"{len(state['stages'])} stages, {len(state['decisions'])} decisions, "
              f"{len(state['contradictions'])} contradictions [{n_open} open])")
        return

    # '</' -> '<\/' keeps the payload valid JSON while making '</script>' inert.
    payload = json.dumps(state, indent=1, ensure_ascii=False).replace("</", "<\\/")
    injected = f'{BEGIN}\n<script id="state" type="application/json">{payload}</script>\n{END}'
    i, j = html.find(BEGIN), html.find(END)
    if i < 0 or j < 0:
        fail(f"markers not found in {HTML}")
    html = html[:i] + injected + html[j + len(END):]
    html = REV_RE.sub(lambda m: m.group(1) + git_short_sha() + m.group(3), html)
    HTML.write_text(html)
    print(f"OK: rendered {HTML.name} from {STATE.name} (updated {state['updated']}, "
          f"{len(state['stages'])} stages, {len(state['decisions'])} decisions, "
          f"{len(state['contradictions'])} contradictions) — commit both files")

    if "--artifact" in sys.argv[1:]:
        i = sys.argv.index("--artifact")
        out = Path(sys.argv[i + 1]) if i + 1 < len(sys.argv) else ROOT / "docs" / "dashboard.artifact.html"
        emit_artifact(html, out)
        print(f"OK: wrote self-contained Artifact HTML -> {out}\n"
              f"    publish with the Artifact tool, url={state.get('artifact_url', '<set artifact_url in project_state.json>')}")


if __name__ == "__main__":
    main()
