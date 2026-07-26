#!/usr/bin/env python3
"""Pull the GitHub work board (Projects v2, or issues-only) into a snapshot the
dashboard BAKES IN at render time.

WHY A SNAPSHOT AND NOT A LIVE FETCH
-----------------------------------
The published dashboard is a claude.ai Artifact. Artifacts run under a strict
CSP: no fetch/XHR to any external host, and the only live-data path is a
claude.ai *connector* (the builder has none for GitHub). So the board CANNOT be
loaded by the page. It is pulled HERE, written to docs/board_snapshot.json, and
embedded by scripts/render_dashboard.py into docs/dashboard.html exactly the way
docs/project_state.json already is. Consequence the reader must always see: the
board is AS OF `fetched_at`, never live — the page stamps that timestamp and its
age wherever board data appears.

SNAPSHOT SCHEMA (docs/board_snapshot.json, schema_version 1)
------------------------------------------------------------
{
  "schema_version": 1,
  "fetched_at": "2026-07-25T18:20:03Z",      # UTC, when THIS data was pulled
  "source": {
    "kind":            "project" | "issues-only",
    "project_number":  int | null,           # null in issues-only mode
    "project_title":   str | null,
    "url":             str | null,           # project URL, or the repo issues URL
    "repo":            str | null,           # "owner/name"
    "tool":            str                   # "gh 2.45.0" — which CLI produced it
  },
  "columns": [str, ...],                     # render order; the Status field's
                                             # option order when available
  "counts": {"items": int, "by_column": {col: int, ...}},
  "warnings": [str, ...],                    # non-fatal caveats ride WITH the
                                             # data (docs/error_handling_policy.md §1)
  "items": [{
    "id":            str,                    # node id (project item / issue)
    "number":        int | null,             # issue/PR number; null for drafts
    "title":         str,
    "column":        str,                    # never invented — see column_source
    "column_source": str,                    # "project-field:Status" | "issue-state"
                                             #  | "label:status:" | "no-status"
    "state":         str | null,             # OPEN / CLOSED (issues+PRs)
    "labels":        [str, ...],
    "assignee":      str | null,             # first assignee login
    "url":           str | null,             # null for draft issues (they have none)
    "updated_at":    str | null,             # ISO-8601 Z, or null if unavailable
    "type":          str,                    # Issue | PullRequest | DraftIssue
    "fields":        {name: value, ...}      # custom project fields (phase,
                                             # blocked-by, iteration, ...)
  }, ...],
  "last_error": {                            # PRESENT ONLY after a FAILED fetch
    "at": ISO-Z, "cause": str, "exit_code": int, "message": str
  }
}

HONESTY RULES (docs/error_handling_policy.md — this repo's dominant defect class
is silent failure)
------------------------------------------------------------------------------
* A failed fetch NEVER writes a snapshot, and never empties an existing one.
  "board is empty" (items: [], counts.items: 0 — a perfectly valid snapshot) and
  "the fetch broke" are DIFFERENT states and must render differently.
* If a fetch fails while an older snapshot exists, the old data is left intact
  but a `last_error` block is stamped into it, so the page shows BOTH the
  failure and the real age of the data it is still displaying. Stale data is
  never re-served as current.
* Every failure exits with a cause-specific code (below) and prints a specific,
  actionable one-liner (the exact command to run).
* Anything dropped is COUNTED and reported in `warnings` — a denominator that
  shrinks silently is the whole failure class.

EXIT CODES (0/2 follow scripts/field/common.sh; 10-16 are this tool's causes)
----------------------------------------------------------------------------
   0  OK          snapshot written
   2  USAGE       bad arguments / operator error
  10  NO_GH       the `gh` CLI is not installed or not on PATH
  11  NO_AUTH     gh is installed but not authenticated
  12  NO_SCOPE    token lacks read:project  -> gh auth refresh -s read:project
  13  NO_PROJECT  owner has no such project (or several, and none was named)
  14  NO_REPO     repo not found / no access (issues-only path)
  15  GH_FAILED   gh call failed for another reason (network, rate limit, ...)
  16  BAD_SHAPE   gh returned output we could not parse -> the API/CLI changed

CLI
---
  scripts/fetch_board.py                          # auto: find the owner's single project
  scripts/fetch_board.py --project 3              # a specific project number
  scripts/fetch_board.py --issues-only            # no project: plain repo issues
  scripts/fetch_board.py --owner X --repo O/R     # non-default owner/repo
  scripts/fetch_board.py --out /tmp/board.json    # write elsewhere
  scripts/fetch_board.py --limit 200              # max items (default 100)
  scripts/fetch_board.py --dry-run                # print the snapshot, write nothing

Offline, stdlib only (shells out to `gh`). Tests: tests/test_fetch_board.py and
the producer->consumer seam test tests/test_board_snapshot_contract.py.
"""
import argparse
import json
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BOARD = ROOT / "docs" / "board_snapshot.json"

DEFAULT_OWNER = "Swampyemerson"
DEFAULT_REPO = "Swampyemerson/interceptor-sim"

SCHEMA_VERSION = 1
NO_STATUS = "(no status)"          # explicit bucket; never a silent default
STATUS_LABEL_PREFIX = "status:"    # issues-only: `status: In progress` label wins over state

EX_OK = 0
EX_USAGE = 2
EX_NO_GH = 10
EX_NO_AUTH = 11
EX_NO_SCOPE = 12
EX_NO_PROJECT = 13
EX_NO_REPO = 14
EX_GH_FAILED = 15
EX_BAD_SHAPE = 16

CAUSE = {
    EX_NO_GH: "NO_GH",
    EX_NO_AUTH: "NO_AUTH",
    EX_NO_SCOPE: "NO_SCOPE",
    EX_NO_PROJECT: "NO_PROJECT",
    EX_NO_REPO: "NO_REPO",
    EX_GH_FAILED: "GH_FAILED",
    EX_BAD_SHAPE: "BAD_SHAPE",
    EX_USAGE: "USAGE",
}

ISO_Z = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$")


class BoardFetchError(Exception):
    """A named, actionable failure. `fix` is the exact command to run next."""

    def __init__(self, code: int, message: str, fix: str = ""):
        super().__init__(message)
        self.code = code
        self.message = message
        self.fix = fix

    @property
    def cause(self) -> str:
        return CAUSE.get(self.code, "UNKNOWN")


# --------------------------------------------------------------------------
# gh plumbing
# --------------------------------------------------------------------------

def utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def gh_path() -> str:
    p = shutil.which("gh")
    if not p:
        raise BoardFetchError(
            EX_NO_GH, "the GitHub CLI `gh` is not installed or not on PATH",
            "sudo apt install gh   (then: gh auth login -s read:project)")
    return p


def gh_version() -> str:
    rc, out, _ = run_gh(["--version"])
    if rc != 0:
        return "gh (version unknown)"
    m = re.search(r"gh version (\S+)", out)
    return f"gh {m.group(1)}" if m else "gh (version unknown)"


def run_gh(args, timeout: int = 60):
    """Run `gh <args>`; return (rc, stdout, stderr). The ONE seam the tests
    monkeypatch — every gh call in this file goes through here."""
    try:
        p = subprocess.run([gh_path()] + list(args), capture_output=True,
                           text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        raise BoardFetchError(
            EX_GH_FAILED, f"`gh {' '.join(args[:3])} ...` timed out after {timeout}s",
            "check the network, then re-run scripts/fetch_board.py")
    return p.returncode, p.stdout, p.stderr


def classify_gh_error(rc: int, out: str, err: str) -> BoardFetchError:
    """Map a failed gh invocation onto a specific cause. Order matters: the
    scope message is the one the builder will actually hit today."""
    blob = (err or "") + "\n" + (out or "")
    low = blob.lower()
    if "read:project" in low or "insufficient_scopes" in low or "missing required scopes" in low:
        return BoardFetchError(
            EX_NO_SCOPE,
            "the gh token lacks the read:project scope, so Projects v2 is unreadable "
            f"(gh said: {first_line(blob)})",
            "gh auth refresh -s read:project   # then re-run scripts/fetch_board.py")
    if ("not logged" in low or "authentication token not found" in low
            or "gh auth login" in low or "requires authentication" in low
            or "bad credentials" in low):
        return BoardFetchError(
            EX_NO_AUTH, f"gh is not authenticated ({first_line(blob)})",
            "gh auth login -s read:project")
    if "could not resolve to a repository" in low or ("not found" in low and "repository" in low):
        return BoardFetchError(
            EX_NO_REPO, f"repository not found or not accessible ({first_line(blob)})",
            "check --repo OWNER/NAME and that the token can see it")
    if "could not resolve to a projectv2" in low or "no project" in low:
        return BoardFetchError(
            EX_NO_PROJECT, f"project not found ({first_line(blob)})",
            "gh project list --owner <login>   # then pass --project <number>")
    return BoardFetchError(
        EX_GH_FAILED, f"gh exited {rc}: {first_line(blob) or '(no output)'}",
        "re-run the gh command by hand to see the full error")


def first_line(s: str) -> str:
    for ln in (s or "").splitlines():
        ln = ln.strip()
        if ln:
            return ln[:300]
    return ""


def gh_json(args, timeout: int = 60):
    rc, out, err = run_gh(args, timeout=timeout)
    if rc != 0:
        raise classify_gh_error(rc, out, err)
    try:
        return json.loads(out)
    except json.JSONDecodeError as e:
        raise BoardFetchError(
            EX_BAD_SHAPE,
            f"`gh {' '.join(args[:3])} ...` returned output that is not JSON ({e}); "
            f"first line: {first_line(out)!r}",
            "run the gh command by hand; the CLI's --format json shape may have changed")


def require_auth() -> None:
    rc, out, err = run_gh(["auth", "status"], timeout=30)
    if rc == 0:
        return
    if "scope" in (err + out).lower():
        raise classify_gh_error(rc, out, err)
    raise BoardFetchError(EX_NO_AUTH,
                          f"gh is not authenticated ({first_line(err + out)})",
                          "gh auth login -s read:project")


# --------------------------------------------------------------------------
# GraphQL — Projects v2
# --------------------------------------------------------------------------
# One query gets everything the snapshot needs (the `gh project item-list`
# exporter in gh 2.45.0 emits only id/title/body/type/url plus camel-cased field
# values, and carries NO updatedAt — verified against cli/cli v2.45.0
# queries.go:serializeProjectWithItems). `%s` is the owner root field: `user` or
# `organization`.
ITEMS_QUERY = """
query($login: String!, $number: Int!, $after: String) {
  owner: %s(login: $login) {
    projectV2(number: $number) {
      number
      title
      url
      field(name: "Status") {
        ... on ProjectV2SingleSelectField { name options { name } }
      }
      items(first: 100, after: $after) {
        totalCount
        pageInfo { hasNextPage endCursor }
        nodes {
          id
          type
          isArchived
          updatedAt
          fieldValues(first: 50) {
            nodes {
              __typename
              ... on ProjectV2ItemFieldTextValue { text field { ... on ProjectV2FieldCommon { name } } }
              ... on ProjectV2ItemFieldNumberValue { number field { ... on ProjectV2FieldCommon { name } } }
              ... on ProjectV2ItemFieldDateValue { date field { ... on ProjectV2FieldCommon { name } } }
              ... on ProjectV2ItemFieldSingleSelectValue { name field { ... on ProjectV2FieldCommon { name } } }
              ... on ProjectV2ItemFieldIterationValue { title field { ... on ProjectV2FieldCommon { name } } }
            }
          }
          content {
            __typename
            ... on DraftIssue { title updatedAt }
            ... on Issue {
              number title url state updatedAt
              labels(first: 20) { nodes { name } }
              assignees(first: 10) { nodes { login } }
            }
            ... on PullRequest {
              number title url state updatedAt
              labels(first: 20) { nodes { name } }
              assignees(first: 10) { nodes { login } }
            }
          }
        }
      }
    }
  }
}
"""


def graphql_items(login: str, number: int, limit: int, root: str = "user"):
    """Page through the project's items. Returns (project_dict, raw_nodes)."""
    nodes, after, project = [], None, None
    while True:
        args = ["api", "graphql", "-f", "query=" + (ITEMS_QUERY % root),
                "-f", f"login={login}", "-F", f"number={number}"]
        if after:
            args += ["-f", f"after={after}"]
        payload = gh_json(args)
        if payload.get("errors"):
            raise classify_gh_error(1, "", json.dumps(payload["errors"]))
        data = payload.get("data") or {}
        owner = data.get("owner")
        if owner is None:
            raise BoardFetchError(
                EX_NO_PROJECT,
                f"GitHub returned no {root} named {login!r} (or it is not visible to this token)",
                f"check --owner; for an organization board the owner root differs "
                f"(this tool retries `organization` automatically)")
        project = owner.get("projectV2")
        if project is None:
            raise BoardFetchError(
                EX_NO_PROJECT, f"{login} has no Projects-v2 board number {number}",
                f"gh project list --owner {login}   # pick a number, then --project N")
        items = project.get("items") or {}
        page = items.get("nodes")
        if page is None:
            raise BoardFetchError(
                EX_BAD_SHAPE, "projectV2.items.nodes missing from the GraphQL response",
                "the Projects v2 GraphQL shape changed; update ITEMS_QUERY")
        nodes.extend(page)
        info = items.get("pageInfo") or {}
        if not info.get("hasNextPage") or len(nodes) >= limit:
            break
        after = info.get("endCursor")
        if not after:
            break
    return project, nodes[:limit]


def field_value(fv: dict):
    """One fieldValues node -> (field name, python value). Returns (None, None)
    for a value type we did not ask for — the caller COUNTS those."""
    name = ((fv.get("field") or {}).get("name")) or None
    t = fv.get("__typename")
    if t == "ProjectV2ItemFieldTextValue":
        return name, fv.get("text")
    if t == "ProjectV2ItemFieldNumberValue":
        return name, fv.get("number")
    if t == "ProjectV2ItemFieldDateValue":
        return name, fv.get("date")
    if t == "ProjectV2ItemFieldSingleSelectValue":
        return name, fv.get("name")
    if t == "ProjectV2ItemFieldIterationValue":
        return name, fv.get("title")
    return None, None


TYPE_NAME = {"ISSUE": "Issue", "PULL_REQUEST": "PullRequest",
             "DRAFT_ISSUE": "DraftIssue", "REDACTED": "Redacted"}


def normalize_project_item(node: dict, status_field: str = "Status"):
    """GraphQL project item -> snapshot item. Returns (item, unmapped_typenames)."""
    content = node.get("content") or {}
    kind = TYPE_NAME.get(node.get("type") or "", content.get("__typename") or "Unknown")
    column, column_source, fields, unmapped = None, "no-status", {}, []
    for fv in ((node.get("fieldValues") or {}).get("nodes") or []):
        name, val = field_value(fv)
        if name is None:
            tn = fv.get("__typename")
            if tn and tn != "ProjectV2ItemFieldTextValue":
                unmapped.append(tn)
            continue
        if val is None or val == "":
            continue
        if name == status_field:
            column, column_source = str(val), f"project-field:{name}"
        elif name.lower() == "title":
            continue                      # the built-in Title field duplicates content.title
        else:
            fields[name] = val
    title = content.get("title") or ""
    if not title:
        raise BoardFetchError(
            EX_BAD_SHAPE,
            f"project item {node.get('id')!r} has no title in its content "
            f"(content={json.dumps(content)[:160]})",
            "the Projects v2 content shape changed; update ITEMS_QUERY/normalize_project_item")
    labels = [n.get("name") for n in ((content.get("labels") or {}).get("nodes") or []) if n.get("name")]
    assignees = [n.get("login") for n in ((content.get("assignees") or {}).get("nodes") or []) if n.get("login")]
    item = {
        "id": node.get("id") or "",
        "number": content.get("number"),
        "title": title,
        "column": column if column else NO_STATUS,
        "column_source": column_source,
        "state": content.get("state"),
        "labels": labels,
        "assignee": assignees[0] if assignees else None,
        "url": content.get("url"),
        "updated_at": content.get("updatedAt") or node.get("updatedAt"),
        "type": kind,
        "fields": fields,
    }
    return item, unmapped


# --------------------------------------------------------------------------
# issues-only path
# --------------------------------------------------------------------------
# Field list EXECUTION-VERIFIED against gh 2.45.0 + the live repo on 2026-07-25
# (accepted, returned [] — the repo has no issues yet).
ISSUE_JSON_FIELDS = "id,number,title,state,labels,assignees,url,updatedAt,milestone"


def normalize_issue(raw: dict) -> dict:
    """`gh issue list --json ...` row -> snapshot item. Column rule (documented
    on the page, never invented): a `status: X` label wins; otherwise the
    issue's OPEN/CLOSED state IS the column."""
    title = raw.get("title") or ""
    if not title:
        raise BoardFetchError(
            EX_BAD_SHAPE, f"issue row has no title: {json.dumps(raw)[:160]}",
            "`gh issue list --json` shape changed; update normalize_issue")
    labels = [l.get("name") for l in (raw.get("labels") or []) if l.get("name")]
    assignees = [a.get("login") for a in (raw.get("assignees") or []) if a.get("login")]
    column, source = None, None
    for l in labels:
        if l.lower().startswith(STATUS_LABEL_PREFIX):
            column = l[len(STATUS_LABEL_PREFIX):].strip() or None
            source = f"label:{STATUS_LABEL_PREFIX}"
            break
    if not column:
        st = (raw.get("state") or "").upper()
        if st not in ("OPEN", "CLOSED"):
            raise BoardFetchError(
                EX_BAD_SHAPE, f"issue #{raw.get('number')} has unexpected state {raw.get('state')!r}",
                "`gh issue list --json state` shape changed; update normalize_issue")
        column, source = ("Open" if st == "OPEN" else "Closed"), "issue-state"
    ms = raw.get("milestone") or {}
    fields = {}
    if ms.get("title"):
        fields["Milestone"] = ms["title"]
    return {
        "id": raw.get("id") or f"issue-{raw.get('number')}",
        "number": raw.get("number"),
        "title": title,
        "column": column,
        "column_source": source,
        "state": (raw.get("state") or "").upper() or None,
        "labels": labels,
        "assignee": assignees[0] if assignees else None,
        "url": raw.get("url"),
        "updated_at": raw.get("updatedAt"),
        "type": "Issue",
        "fields": fields,
    }


# --------------------------------------------------------------------------
# snapshot assembly / writing / validation
# --------------------------------------------------------------------------
TOP_KEYS = {"schema_version", "fetched_at", "source", "columns", "counts", "warnings", "items"}
SOURCE_KEYS = {"kind", "project_number", "project_title", "url", "repo", "tool"}
ITEM_KEYS = {"id", "number", "title", "column", "column_source", "state", "labels",
             "assignee", "url", "updated_at", "type", "fields"}


def order_columns(items, declared) -> list:
    """Render order: the Status field's own option order first (only those in
    use plus the empty declared ones — an empty column is real information),
    then any column seen that the field never declared, then the no-status
    bucket last."""
    seen = []
    for it in items:
        if it["column"] not in seen:
            seen.append(it["column"])
    cols = [c for c in (declared or []) if c != NO_STATUS]
    for c in seen:
        if c not in cols and c != NO_STATUS:
            cols.append(c)
    if NO_STATUS in seen:
        cols.append(NO_STATUS)
    return cols


def build_snapshot(source: dict, items: list, declared_columns=None,
                   warnings=None, fetched_at=None) -> dict:
    cols = order_columns(items, declared_columns)
    by_col = {c: 0 for c in cols}
    for it in items:
        by_col[it["column"]] = by_col.get(it["column"], 0) + 1
    snap = {
        "schema_version": SCHEMA_VERSION,
        "fetched_at": fetched_at or utc_now(),
        "source": source,
        "columns": cols,
        "counts": {"items": len(items), "by_column": by_col},
        "warnings": list(warnings or []),
        "items": items,
    }
    validate_snapshot(snap)
    return snap


def validate_snapshot(snap: dict) -> None:
    """PRODUCER-side schema check, run on every write. The CONSUMER
    (scripts/render_dashboard.py:validate_board) has its own; the seam between
    them is pinned by tests/test_board_snapshot_contract.py, whose fixture is
    written by THIS function's caller — never hand-typed."""
    if not isinstance(snap, dict):
        raise BoardFetchError(EX_BAD_SHAPE, "snapshot is not a JSON object", "")
    extra = set(snap) - TOP_KEYS - {"last_error"}
    missing = TOP_KEYS - set(snap)
    if missing:
        raise BoardFetchError(EX_BAD_SHAPE, f"snapshot missing keys: {sorted(missing)}", "")
    if extra:
        raise BoardFetchError(EX_BAD_SHAPE, f"snapshot has unknown keys: {sorted(extra)}", "")
    if snap["schema_version"] != SCHEMA_VERSION:
        raise BoardFetchError(
            EX_BAD_SHAPE,
            f"snapshot schema_version {snap['schema_version']} != {SCHEMA_VERSION}", "")
    if not ISO_Z.match(str(snap["fetched_at"])):
        raise BoardFetchError(
            EX_BAD_SHAPE, f"fetched_at {snap['fetched_at']!r} is not ISO-8601 UTC (…Z)", "")
    src = snap["source"]
    if set(src) != SOURCE_KEYS:
        raise BoardFetchError(
            EX_BAD_SHAPE,
            f"source keys {sorted(set(src))} != {sorted(SOURCE_KEYS)}", "")
    if src["kind"] not in ("project", "issues-only"):
        raise BoardFetchError(EX_BAD_SHAPE, f"source.kind {src['kind']!r} unknown", "")
    if not isinstance(snap["columns"], list) or not all(isinstance(c, str) for c in snap["columns"]):
        raise BoardFetchError(EX_BAD_SHAPE, "columns must be a list of strings", "")
    if not isinstance(snap["warnings"], list) or not all(isinstance(w, str) for w in snap["warnings"]):
        raise BoardFetchError(EX_BAD_SHAPE, "warnings must be a list of strings", "")
    items = snap["items"]
    if not isinstance(items, list):
        raise BoardFetchError(EX_BAD_SHAPE, "items must be a list", "")
    for it in items:
        if set(it) != ITEM_KEYS:
            raise BoardFetchError(
                EX_BAD_SHAPE,
                f"item {it.get('id', '?')!r} keys {sorted(set(it))} != {sorted(ITEM_KEYS)}", "")
        if not it["title"]:
            raise BoardFetchError(EX_BAD_SHAPE, f"item {it['id']!r} has an empty title", "")
        if it["column"] not in snap["columns"]:
            raise BoardFetchError(
                EX_BAD_SHAPE,
                f"item {it['id']!r} column {it['column']!r} is not in columns {snap['columns']}", "")
    counts = snap["counts"]
    if set(counts) != {"items", "by_column"}:
        raise BoardFetchError(EX_BAD_SHAPE, "counts must hold exactly items + by_column", "")
    # The anti-partial-write invariant: a truncated/edited snapshot fails here.
    if counts["items"] != len(items):
        raise BoardFetchError(
            EX_BAD_SHAPE,
            f"counts.items {counts['items']} != len(items) {len(items)} (partial write?)", "")
    if sum(counts["by_column"].values()) != len(items):
        raise BoardFetchError(
            EX_BAD_SHAPE,
            f"counts.by_column sums to {sum(counts['by_column'].values())} but there "
            f"are {len(items)} items (an item was dropped from a column)", "")
    for c in counts["by_column"]:
        if c not in snap["columns"]:
            raise BoardFetchError(
                EX_BAD_SHAPE, f"counts.by_column has column {c!r} not in columns", "")


def write_snapshot(path: Path, snap: dict) -> None:
    """THE writer. Every snapshot on disk — including every test fixture — comes
    through here (docs/error_handling_policy.md §6: a hand-typed fixture is
    exactly what hid two schema breaks in this repo)."""
    validate_snapshot(snap)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(snap, indent=1, ensure_ascii=False) + "\n")


def record_fetch_failure(path: Path, err: BoardFetchError) -> str:
    """A fetch failed. NEVER write a snapshot; if an old one exists, stamp the
    failure into it so the page shows the break AND the real age of the data it
    is still displaying. Returns a one-line description of what was done."""
    path = Path(path)
    if not path.exists():
        return f"no snapshot at {path} — the dashboard will render 'board not yet connected'"
    try:
        snap = json.loads(path.read_text())
    except Exception as e:                                   # corrupt file: say so, touch nothing
        return f"existing {path.name} is unreadable ({e}); left untouched"
    snap["last_error"] = {
        "at": utc_now(),
        "cause": err.cause,
        "exit_code": err.code,
        "message": err.message + (f" — fix: {err.fix}" if err.fix else ""),
    }
    path.write_text(json.dumps(snap, indent=1, ensure_ascii=False) + "\n")
    return (f"stamped last_error into the EXISTING {path.name} (data still as of "
            f"{snap.get('fetched_at', '?')}; the page shows the failure + that age)")


# --------------------------------------------------------------------------
# fetch drivers
# --------------------------------------------------------------------------

def discover_project(owner: str) -> dict:
    """`gh project list --owner X --format json` ->
    {"projects":[{number,title,url,...}],"totalCount":N}  (verified against
    cli/cli v2.45.0 queries.go:Projects.ExportData)."""
    payload = gh_json(["project", "list", "--owner", owner, "--format", "json", "--limit", "50"])
    projects = payload.get("projects")
    if projects is None:
        raise BoardFetchError(
            EX_BAD_SHAPE, "`gh project list --format json` had no 'projects' key",
            "gh version changed the export shape; update discover_project()")
    live = [p for p in projects if not p.get("closed")]
    if not live:
        raise BoardFetchError(
            EX_NO_PROJECT,
            f"{owner} has no open Projects-v2 board, so there is no board to snapshot",
            "create one at https://github.com/users/%s/projects/new , or run "
            "scripts/fetch_board.py --issues-only" % owner)
    if len(live) > 1:
        listing = ", ".join(f"#{p.get('number')} {p.get('title')!r}" for p in live)
        raise BoardFetchError(
            EX_NO_PROJECT,
            f"{owner} has {len(live)} open projects and none was named: {listing}",
            "re-run with --project <number>")
    return live[0]


def fetch_project(owner: str, repo: str, number, limit: int) -> dict:
    warnings = []
    if number is None:
        p = discover_project(owner)
        number = p.get("number")
        if not isinstance(number, int):
            raise BoardFetchError(
                EX_BAD_SHAPE, f"discovered project has a non-integer number: {number!r}", "")
    try:
        project, nodes = graphql_items(owner, number, limit, root="user")
    except BoardFetchError as e:
        if e.code != EX_NO_PROJECT:
            raise
        # The owner may be an organization, whose board hangs off a different
        # GraphQL root. Retry once, then give up with the ORIGINAL message.
        try:
            project, nodes = graphql_items(owner, number, limit, root="organization")
            warnings.append(f"owner {owner} resolved as an organization, not a user")
        except BoardFetchError:
            raise e
    status_field = project.get("field") or {}
    declared = [o.get("name") for o in (status_field.get("options") or []) if o.get("name")]
    if not declared:
        warnings.append(
            "the project has no single-select field named 'Status' — columns are "
            "the values actually seen on items, in first-seen order")
    items, unmapped, archived = [], [], 0
    for n in nodes:
        if n.get("isArchived"):
            archived += 1
            continue
        it, un = normalize_project_item(n, status_field=status_field.get("name") or "Status")
        items.append(it)
        unmapped.extend(un)
    if archived:
        warnings.append(f"{archived} archived item(s) excluded from this snapshot")
    if unmapped:
        kinds = sorted(set(unmapped))
        warnings.append(
            f"{len(unmapped)} custom field value(s) of type {kinds} were not read "
            f"(fetch_board reads text/number/date/single-select/iteration only)")
    total = ((project.get("items") or {}).get("totalCount"))
    if isinstance(total, int) and total > len(nodes):
        warnings.append(f"showing {len(nodes)} of {total} items (--limit {limit})")
    source = {
        "kind": "project",
        "project_number": project.get("number", number),
        "project_title": project.get("title"),
        "url": project.get("url"),
        "repo": repo,
        "tool": gh_version(),
    }
    return build_snapshot(source, items, declared, warnings)


def fetch_issues(repo: str, limit: int) -> dict:
    rows = gh_json(["issue", "list", "--repo", repo, "--state", "all",
                    "--limit", str(limit), "--json", ISSUE_JSON_FIELDS])
    if not isinstance(rows, list):
        raise BoardFetchError(
            EX_BAD_SHAPE, "`gh issue list --json` did not return a JSON array",
            "gh changed its export shape; update fetch_issues()")
    items = [normalize_issue(r) for r in rows]
    warnings = []
    if len(rows) == limit:
        warnings.append(f"hit the --limit {limit} ceiling; there may be more issues")
    warnings.append(
        "issues-only mode: there is no Projects board, so the column is the "
        "issue's OPEN/CLOSED state unless it carries a 'status: X' label")
    source = {
        "kind": "issues-only",
        "project_number": None,
        "project_title": None,
        "url": f"https://github.com/{repo}/issues",
        "repo": repo,
        "tool": gh_version(),
    }
    # Declared order keeps a familiar left-to-right reading even with 0 items.
    return build_snapshot(source, items, ["Open", "Closed"], warnings)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="Snapshot the GitHub work board into docs/board_snapshot.json "
                    "for scripts/render_dashboard.py to bake into the dashboard.")
    ap.add_argument("--owner", default=DEFAULT_OWNER, help="project owner login")
    ap.add_argument("--repo", default=DEFAULT_REPO, help="OWNER/NAME for issues + provenance")
    ap.add_argument("--project", type=int, default=None, metavar="N",
                    help="project number (default: auto-discover the owner's single project)")
    ap.add_argument("--issues-only", action="store_true",
                    help="no Projects board: snapshot the repo's issues instead")
    ap.add_argument("--out", default=str(BOARD), help=f"snapshot path (default {BOARD})")
    ap.add_argument("--limit", type=int, default=100, help="max items (default 100)")
    ap.add_argument("--dry-run", action="store_true", help="print the snapshot, write nothing")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    out = Path(args.out)
    if args.issues_only and args.project is not None:
        print("fetch_board: USAGE — --issues-only and --project are mutually exclusive "
              "(pick the board OR the issues; a silent fallback between them would hide "
              "which source you are reading)", file=sys.stderr)
        return EX_USAGE
    if args.limit < 1:
        print(f"fetch_board: USAGE — --limit must be >= 1 (got {args.limit})", file=sys.stderr)
        return EX_USAGE
    try:
        gh_path()
        require_auth()
        snap = (fetch_issues(args.repo, args.limit) if args.issues_only
                else fetch_project(args.owner, args.repo, args.project, args.limit))
    except BoardFetchError as e:
        note = record_fetch_failure(out, e)
        print(f"fetch_board: FAILED [{e.cause}] {e.message}", file=sys.stderr)
        if e.fix:
            print(f"  fix: {e.fix}", file=sys.stderr)
        print(f"  snapshot: {note}", file=sys.stderr)
        print(f"  exit {e.code} ({e.cause}) — NO snapshot was written; an empty board and a "
              f"broken fetch stay distinguishable in the dashboard.", file=sys.stderr)
        return e.code
    if args.dry_run:
        print(json.dumps(snap, indent=1, ensure_ascii=False))
        print(f"fetch_board: DRY RUN — nothing written to {out}", file=sys.stderr)
        return EX_OK
    write_snapshot(out, snap)
    n = snap["counts"]["items"]
    src = snap["source"]
    where = (f"project #{src['project_number']} {src['project_title']!r}"
             if src["kind"] == "project" else f"issues of {src['repo']}")
    # n is printed even when 0: "connected, board is EMPTY" is a real, valid
    # result and must read differently from a failure.
    print(f"OK: wrote {out} — {n} item(s) from {where} as of {snap['fetched_at']} "
          f"({len(snap['columns'])} columns"
          + (", BOARD IS EMPTY" if n == 0 else "") + ")")
    for w in snap["warnings"]:
        print(f"  warning: {w}")
    print("  next: python3 scripts/render_dashboard.py   # bakes the snapshot into the dashboard")
    return EX_OK


if __name__ == "__main__":
    sys.exit(main())
