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
                   name/status/role/notes with status ordered|must-add|print|in hand|built + connections[]
                   of from/to/medium + steps[] of id/text with optional gate/why) + ladder
                   (summary/rungs[]/gate; rung refs point at step ids) + evidence.
                   Step ids are UNIQUE across ALL subsystems — they key the Build sheet's
                   localStorage checkboxes — and at least one step MUST carry gate=true
                   (the hard stop-points: smoke stopper, props-off, calibration, legal).
Schema v2.4 adds (2026-07-21 narrative-first redesign, validated here):
  narrative      — the §0 STORY layer the dashboard leads with (PRESENTATION-ONLY — a
                   distillation of findings that live in the stages/decisions/docs; it must
                   never contradict them): lede + reframe (headline/text/stats[] of
                   label/value/provenance — numbers trace to a run — + optional
                   note/tone/stamp) + big_lever (headline/text/evidence) + caveat
Schema v2.5 adds (2026-07-25 board pass) — a SECOND embedded document, not a
state key: docs/board_snapshot.json (the GitHub work board, written by
scripts/fetch_board.py) is baked into a <script id="board"> block the same way
the contract is, because a published Artifact runs under a CSP that forbids
fetching it live. The snapshot is OPTIONAL: with no file the render still
succeeds and the Board sheet says "not yet connected". --check verifies that
embed against the file exactly as it does for the state — including the
gone-file case (a deleted snapshot must not keep rendering as current data).
                   (headline/text/meter of unit/max/reference{value,label}/bars[]{label,
                   value}/provenance) + rulings[] (date/text/evidence — the builder's live
                   calls) + program (headline/summary/phases[] of code/name/status/tone/
                   what/test — the 4-phase A→D roadmap) + levers (headline/rows[] of
                   lever/category/what/test + note).

The renderer also stamps the live git short-sha into the title block between
REV:BEGIN/END markers (--check ignores the REV stamp; only the state block is compared).

Ritual: edit the JSON -> run this script -> commit BOTH files together.
--check is the drift alarm (wired into run_tests.sh).
"""
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
STATE = ROOT / "docs" / "project_state.json"
HTML = ROOT / "docs" / "dashboard.html"
BOARD = ROOT / "docs" / "board_snapshot.json"

BEGIN = "<!-- PROJECT_STATE_JSON:BEGIN (generated - edit docs/project_state.json and run scripts/render_dashboard.py; never hand-edit this block) -->"
END = "<!-- PROJECT_STATE_JSON:END -->"
B_BEGIN = "<!-- BOARD_SNAPSHOT_JSON:BEGIN (generated - run scripts/fetch_board.py then scripts/render_dashboard.py; never hand-edit this block) -->"
B_END = "<!-- BOARD_SNAPSHOT_JSON:END -->"
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
BT_PART_STATUSES = {"ordered", "must-add", "print", "in hand", "built"}
BT_CONN_KEYS = {"from", "to", "medium"}
BT_STEP_KEYS = {"id", "text"}
BT_LADDER_KEYS = {"summary", "rungs", "gate"}
BT_RUNG_KEYS = {"code", "name", "goal", "refs"}
NARR_KEYS = {"lede", "reframe", "big_lever", "caveat", "rulings", "program", "levers"}
NARR_STAT_KEYS = {"label", "value", "provenance"}
NARR_PHASE_KEYS = {"code", "name", "status", "tone", "what", "test"}
NARR_RULING_KEYS = {"date", "text", "evidence"}
NARR_LEVER_KEYS = {"lever", "category", "what", "test"}


def fail(msg: str) -> None:
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


def validate(state: dict) -> None:
    for key in ("schema_version", "updated", "goal", "headline", "narrative", "architecture",
                "build_plan", "stages", "edges", "constraints", "graveyard", "key_numbers",
                "bom_tiers", "build_tab", "decisions", "contradictions", "artifact_url"):
        if key not in state:
            fail(f"missing top-level key: {key}")
    narr = state["narrative"]
    missing = NARR_KEYS - set(narr)
    if missing:
        fail(f"narrative missing keys: {sorted(missing)}")
    for blk in ("reframe", "big_lever", "caveat"):
        for k in ("headline", "text", "evidence"):
            if not narr[blk].get(k):
                fail(f"narrative.{blk} needs a non-empty {k!r}")
    if not narr["reframe"].get("stats"):
        fail("narrative.reframe has no stats tiles")
    for s in narr["reframe"]["stats"]:
        missing = NARR_STAT_KEYS - set(s)
        if missing:
            fail(f"narrative stat {s.get('label', '?')!r} missing keys: {sorted(missing)} (numbers trace to a run)")
    meter = narr["caveat"].get("meter") or {}
    if not ({"unit", "max", "reference", "bars", "provenance"} <= set(meter)):
        fail("narrative.caveat.meter needs unit/max/reference/bars/provenance")
    if not ({"value", "label"} <= set(meter["reference"])):
        fail("narrative.caveat.meter.reference needs value+label")
    if not meter["bars"] or not all({"label", "value"} <= set(b) for b in meter["bars"]):
        fail("narrative.caveat.meter.bars need label+value each")
    if not narr["rulings"]:
        fail("narrative has no rulings")
    for r in narr["rulings"]:
        missing = NARR_RULING_KEYS - set(r)
        if missing:
            fail(f"narrative ruling {r.get('date', '?')!r} missing keys: {sorted(missing)}")
    prog = narr["program"]
    for k in ("headline", "summary", "phases", "evidence"):
        if not prog.get(k):
            fail(f"narrative.program needs a non-empty {k!r}")
    for p in prog["phases"]:
        missing = NARR_PHASE_KEYS - set(p)
        if missing:
            fail(f"narrative program phase {p.get('code', '?')!r} missing keys: {sorted(missing)}")
    lev = narr["levers"]
    for k in ("headline", "rows", "evidence"):
        if not lev.get(k):
            fail(f"narrative.levers needs a non-empty {k!r}")
    for row in lev["rows"]:
        missing = NARR_LEVER_KEYS - set(row)
        if missing:
            fail(f"narrative lever {row.get('lever', '?')!r} missing keys: {sorted(missing)}")
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


BOARD_TOP_KEYS = {"schema_version", "fetched_at", "source", "columns", "counts", "warnings", "items"}
BOARD_SOURCE_KEYS = {"kind", "project_number", "project_title", "url", "repo", "tool"}
BOARD_ITEM_KEYS = {"id", "number", "title", "column", "column_source", "state", "labels",
                   "assignee", "url", "updated_at", "type", "fields"}
BOARD_ERR_KEYS = {"at", "cause", "exit_code", "message"}
ISO_Z_RE = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$")


def validate_board(board: dict) -> None:
    """CONSUMER-side schema check for docs/board_snapshot.json (schema v1,
    written by scripts/fetch_board.py — its own producer-side validator is
    fetch_board.validate_snapshot). Two validators is deliberate: the SEAM
    between them is pinned by tests/test_board_snapshot_contract.py, whose
    fixture comes from the producer's own writer (error_handling_policy §6).
    Strict on unknown keys — a schema change must stop the render, not render
    a half-understood board."""
    if not isinstance(board, dict):
        fail(f"{BOARD.name} is not a JSON object")
    unknown = set(board) - BOARD_TOP_KEYS - {"last_error"}
    missing = BOARD_TOP_KEYS - set(board)
    if missing:
        fail(f"{BOARD.name} missing keys: {sorted(missing)}")
    if unknown:
        fail(f"{BOARD.name} has unknown top-level keys: {sorted(unknown)} "
             f"(schema changed? update validate_board + the Board sheet)")
    if board["schema_version"] != 1:
        fail(f"{BOARD.name} schema_version {board['schema_version']!r} — this renderer speaks 1")
    if not ISO_Z_RE.match(str(board["fetched_at"])):
        fail(f"{BOARD.name} fetched_at {board['fetched_at']!r} is not ISO-8601 UTC "
             f"(the page stamps this age; an unparseable timestamp would hide staleness)")
    src = board["source"]
    if not isinstance(src, dict) or set(src) != BOARD_SOURCE_KEYS:
        fail(f"{BOARD.name} source keys {sorted(set(src)) if isinstance(src, dict) else src!r} "
             f"!= {sorted(BOARD_SOURCE_KEYS)}")
    if src["kind"] not in ("project", "issues-only"):
        fail(f"{BOARD.name} source.kind {src['kind']!r}; allowed: project | issues-only")
    if not isinstance(board["columns"], list) or not all(isinstance(c, str) for c in board["columns"]):
        fail(f"{BOARD.name} columns must be a list of strings")
    if not isinstance(board["warnings"], list) or not all(isinstance(w, str) for w in board["warnings"]):
        fail(f"{BOARD.name} warnings must be a list of strings")
    items = board["items"]
    if not isinstance(items, list):
        fail(f"{BOARD.name} items must be a list")
    for it in items:
        if not isinstance(it, dict) or set(it) != BOARD_ITEM_KEYS:
            fail(f"{BOARD.name} item {it.get('id', '?') if isinstance(it, dict) else it!r} keys "
                 f"{sorted(set(it)) if isinstance(it, dict) else '?'} != {sorted(BOARD_ITEM_KEYS)}")
        if not it["title"]:
            fail(f"{BOARD.name} item {it['id']!r} has an empty title")
        if it["column"] not in board["columns"]:
            fail(f"{BOARD.name} item {it['id']!r} sits in column {it['column']!r}, which is not "
                 f"in columns {board['columns']} — it would render nowhere (a silently "
                 f"dropped work item)")
    counts = board["counts"]
    if not isinstance(counts, dict) or set(counts) != {"items", "by_column"}:
        fail(f"{BOARD.name} counts must hold exactly items + by_column")
    # Anti-partial-snapshot invariants: the count a reader sees must equal the
    # rows actually present, and no item may vanish between the two.
    if counts["items"] != len(items):
        fail(f"{BOARD.name} counts.items {counts['items']} != {len(items)} rendered items "
             f"(partial write / edited snapshot — re-run scripts/fetch_board.py)")
    if sum(counts["by_column"].values()) != len(items):
        fail(f"{BOARD.name} counts.by_column sums to {sum(counts['by_column'].values())} but "
             f"carries {len(items)} items")
    le = board.get("last_error")
    if le is not None:
        if not isinstance(le, dict) or set(le) != BOARD_ERR_KEYS:
            fail(f"{BOARD.name} last_error keys must be exactly {sorted(BOARD_ERR_KEYS)}")
        if not ISO_Z_RE.match(str(le["at"])):
            fail(f"{BOARD.name} last_error.at {le['at']!r} is not ISO-8601 UTC")


def load_board():
    """Returns the validated snapshot, or None when the builder has not
    connected the board yet. ABSENT is a legitimate, renderable state; a
    PRESENT-but-broken file is not (it fails the render loudly)."""
    if not BOARD.exists():
        return None
    try:
        board = json.loads(BOARD.read_text())
    except json.JSONDecodeError as e:
        fail(f"{BOARD} is not valid JSON: {e} — re-run scripts/fetch_board.py "
             f"(never hand-edit the snapshot)")
    validate_board(board)
    return board


def extract_embedded_board(html: str):
    """The board the HTML currently carries: the parsed snapshot, or None for
    the 'not yet connected' sentinel (an embedded JSON `null`)."""
    i, j = html.find(B_BEGIN), html.find(B_END)
    if i < 0 or j < 0:
        fail(f"board markers not found in {HTML} (expected {B_BEGIN[:40]}...)")
    block = html[i + len(B_BEGIN):j]
    m = re.search(r'<script id="board" type="application/json">(.*)</script>', block, re.S)
    if not m:
        fail('embedded <script id="board"> not found between the board markers')
    return json.loads(m.group(1))


def board_status_line(board) -> str:
    """One human line naming exactly what the board embed is — printed by both
    render and --check so an operator can never mistake 'not connected' for
    'empty' or for 'stale after a failed fetch'."""
    if board is None:
        return "board: NOT CONNECTED (no docs/board_snapshot.json — run scripts/fetch_board.py)"
    n = board["counts"]["items"]
    bits = [f"board: {n} item(s)" + (" [EMPTY BOARD]" if n == 0 else ""),
            f"source {board['source']['kind']}",
            f"fetched {board['fetched_at']}"]
    if board.get("last_error"):
        bits.append(f"LAST FETCH FAILED [{board['last_error']['cause']}] — data above is STALE")
    return ", ".join(bits)


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


def hand_layer_visible_numbers(html: str) -> list:
    """Decimal numbers a human can SEE in the hand-authored layer: everything
    OUTSIDE the generated embedded-state block, minus <style>/<script>/HTML
    comments/tags. These are the headline claims the builder actually reads."""
    i, j = html.find(BEGIN), html.find(END)
    hand = (html[:i] + html[j + len(END):]) if (i >= 0 and j >= 0) else html
    hand = re.sub(r"<style\b.*?</style>", " ", hand, flags=re.S | re.I)
    hand = re.sub(r"<script\b.*?</script>", " ", hand, flags=re.S | re.I)
    hand = re.sub(r"<!--.*?-->", " ", hand, flags=re.S)
    visible = re.sub(r"<[^>]+>", " ", hand)
    return sorted(set(re.findall(r"\d+\.\d+", visible)))


def check_hand_layer_numbers(html: str, state_raw: str) -> int:
    """NUMBER-TRACEABILITY GUARD (review-2 structural finding, 2026-07-24).

    --check used to validate ONLY the embedded JSON, so the hand-authored
    prose/SVG could assert REVERSED conclusions while the check passed green
    (it happened twice on 2026-07-24: the hero SVG sold a refuted fix, and a
    stage read 'NOTHING ORDERED YET' four days after ~$1,000 shipped).
    Mechanical floor: every decimal number VISIBLE in the hand-authored layer
    must appear VERBATIM in project_state.json's raw text — 'numbers trace to
    a run', enforced at the layer a human reads. Verified green on the tree
    this shipped on (20 visible numbers, all traced). Scope honesty: this
    guards NUMBERS only; a pure-prose flip (e.g. NET-NEGATIVE -> NET-POSITIVE)
    still passes and remains a human-review control.

    Returns the count of visible numbers traced; exits 1 naming the orphans.
    """
    nums = hand_layer_visible_numbers(html)
    missing = [n for n in nums if n not in state_raw]
    if missing:
        fail(f"hand-authored dashboard layer carries number(s) with no verbatim "
             f"source in {STATE.name}: {missing} — every visible number must "
             f"trace to the contract (fix the prose/SVG or land the number in "
             f"project_state.json first)")
    return len(nums)


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
    state_raw = STATE.read_text()
    try:
        state = json.loads(state_raw)
    except json.JSONDecodeError as e:
        fail(f"{STATE} is not valid JSON: {e}")
    validate(state)
    board = load_board()
    html = HTML.read_text()

    if check:
        if "--artifact" in sys.argv[1:]:
            # Silent-failure fix (review 2): this combination used to write
            # NOTHING and print OK/exit 0 — an operator believed a fresh
            # artifact copy existed when none was emitted.
            fail("--check --artifact does nothing: --check never renders, so no "
                 "artifact file would be written. Run WITHOUT --check to render "
                 "and emit the artifact copy.")
        if extract_embedded_json(html) != state:
            fail(f"{HTML.name} embedded state DIFFERS from {STATE.name} — run scripts/render_dashboard.py and commit both")
        emb_board = extract_embedded_board(html)
        if emb_board != board:
            if board is None:
                fail(f"{HTML.name} still embeds a board snapshot but {BOARD.name} is GONE — "
                     f"the page would serve deleted data as current. Re-run "
                     f"scripts/render_dashboard.py (or restore the snapshot with "
                     f"scripts/fetch_board.py) and commit both")
            if emb_board is None:
                fail(f"{BOARD.name} exists but {HTML.name} embeds no board — run "
                     f"scripts/render_dashboard.py and commit both")
            fail(f"{HTML.name} embedded board DIFFERS from {BOARD.name} (embedded fetched_at "
                 f"{emb_board.get('fetched_at')!r} vs file {board.get('fetched_at')!r}) — run "
                 f"scripts/render_dashboard.py and commit both")
        n_traced = check_hand_layer_numbers(html, state_raw)
        n_open = sum(1 for c in state["contradictions"] if c["status"] == "open")
        # Scope-honest OK line: say what was checked, not more (review 2 —
        # "in sync" used to claim more scope than the check had).
        print(f"OK: {STATE.name} valid; {HTML.name} embedded state in sync; "
              f"hand-authored layer: {n_traced} visible numbers traced to the contract "
              f"(prose wording NOT checked). updated {state['updated']}, "
              f"{len(state['stages'])} stages, {len(state['decisions'])} decisions, "
              f"{len(state['contradictions'])} contradictions [{n_open} open]")
        print(f"OK: {board_status_line(board)} — embed in sync "
              f"(board CONTENT is not verified against GitHub; it is as of fetched_at)")
        return

    # '</' -> '<\/' keeps the payload valid JSON while making '</script>' inert.
    payload = json.dumps(state, indent=1, ensure_ascii=False).replace("</", "<\\/")
    injected = f'{BEGIN}\n<script id="state" type="application/json">{payload}</script>\n{END}'
    i, j = html.find(BEGIN), html.find(END)
    if i < 0 or j < 0:
        fail(f"markers not found in {HTML}")
    html = html[:i] + injected + html[j + len(END):]

    # Second embedded document: the GitHub board snapshot (CSP forbids the
    # published Artifact from fetching it live). `null` is the explicit
    # not-yet-connected sentinel — it must OVERWRITE any previous embed, so a
    # deleted snapshot can never keep rendering as current data.
    b_payload = json.dumps(board, indent=1, ensure_ascii=False).replace("</", "<\\/")
    b_injected = f'{B_BEGIN}\n<script id="board" type="application/json">{b_payload}</script>\n{B_END}'
    bi, bj = html.find(B_BEGIN), html.find(B_END)
    if bi < 0 or bj < 0:
        fail(f"board markers not found in {HTML} — the Board sheet's embed block is missing")
    html = html[:bi] + b_injected + html[bj + len(B_END):]

    html = REV_RE.sub(lambda m: m.group(1) + git_short_sha() + m.group(3), html)
    HTML.write_text(html)
    # Same number-traceability guard as --check, so an orphan number fails at
    # RENDER time (loudly, after the write) instead of surviving to the gate.
    n_traced = check_hand_layer_numbers(html, state_raw)
    print(f"OK: rendered {HTML.name} from {STATE.name} (updated {state['updated']}, "
          f"{len(state['stages'])} stages, {len(state['decisions'])} decisions, "
          f"{len(state['contradictions'])} contradictions; hand-authored layer: "
          f"{n_traced} visible numbers traced) — commit both files")
    print(f"OK: {board_status_line(board)}")

    if "--artifact" in sys.argv[1:]:
        i = sys.argv.index("--artifact")
        out = Path(sys.argv[i + 1]) if i + 1 < len(sys.argv) else ROOT / "docs" / "dashboard.artifact.html"
        emit_artifact(html, out)
        # PUBLISH RECEIPT: ties the emitted bytes to the contract revision so a
        # republish can be audited ("which state did the hosted Artifact carry?").
        data = out.read_bytes()
        print(f"OK: wrote self-contained Artifact HTML -> {out}\n"
              f"    publish with the Artifact tool, url={state.get('artifact_url', '<set artifact_url in project_state.json>')}\n"
              f"    RECEIPT: sha256[:12]={hashlib.sha256(data).hexdigest()[:12]} "
              f"bytes={len(data)} state.updated={state['updated']} git={git_short_sha()}")


if __name__ == "__main__":
    main()
