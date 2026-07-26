"""Unit tests for scripts/fetch_board.py — normalization + EVERY failure mode.

WHY THIS FILE EXISTS (docs/error_handling_policy.md): the board fetch is a
measurement-layer tool, and this repo's dominant defect class is the silent
failure — a confident, plausible, WRONG output behind a green check. The two
outputs that must NEVER be confusable here are:

    "the board is empty"   (a valid snapshot, counts.items == 0, exit 0)
    "the fetch broke"      (NO snapshot written, cause-specific exit code)

so both are asserted explicitly, along with the rule that a failed fetch never
empties, overwrites, or re-dates an existing snapshot.

Offline: every gh call goes through fetch_board.run_gh, which is the one seam
these tests monkeypatch. No network, stdlib + pytest only.
"""
import json
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import fetch_board as fb                                            # noqa: E402


# --------------------------------------------------------------------------
# fixtures shaped like the REAL gh output (see the provenance notes)
# --------------------------------------------------------------------------
def gql_item(**over):
    """One `items.nodes[]` entry of fetch_board.ITEMS_QUERY. The QUERY itself was
    verified schema-valid against live GitHub on 2026-07-25 (the API returned
    15 INSUFFICIENT_SCOPES errors and ZERO validation errors, which means every
    field/fragment resolved); the response shape below follows from it, but has
    NOT been seen with real data — the token still lacks read:project."""
    node = {
        "id": "PVTI_lADO",
        "type": "ISSUE",
        "isArchived": False,
        "updatedAt": "2026-07-20T09:00:00Z",
        "fieldValues": {"nodes": [
            {"__typename": "ProjectV2ItemFieldTextValue", "text": "Tripod day",
             "field": {"name": "Title"}},
            {"__typename": "ProjectV2ItemFieldSingleSelectValue", "name": "In progress",
             "field": {"name": "Status"}},
            {"__typename": "ProjectV2ItemFieldSingleSelectValue", "name": "P2",
             "field": {"name": "Phase"}},
            {"__typename": "ProjectV2ItemFieldTextValue", "text": "#4",
             "field": {"name": "Blocked by"}},
        ]},
        "content": {
            "__typename": "Issue", "number": 12, "title": "Tripod day: decode gate",
            "url": "https://github.com/Swampyemerson/interceptor-sim/issues/12",
            "state": "OPEN", "updatedAt": "2026-07-24T18:30:00Z",
            "labels": {"nodes": [{"name": "hardware"}, {"name": "gate"}]},
            "assignees": {"nodes": [{"login": "Swampyemerson"}]},
        },
    }
    node.update(over)
    return node


def issue_row(**over):
    """One row of `gh issue list --json number,title,state,labels,assignees,url,
    updatedAt,milestone`. EXECUTION-VERIFIED: that exact --json field list was
    accepted by gh 2.45.0 against the live repo on 2026-07-25 (it returned []
    — the repo has no issues yet)."""
    row = {
        "number": 7, "title": "Wire the Hailo phase gate", "state": "OPEN",
        "labels": [{"name": "seeker"}], "assignees": [{"login": "Swampyemerson"}],
        "url": "https://github.com/Swampyemerson/interceptor-sim/issues/7",
        "updatedAt": "2026-07-22T12:00:00Z", "milestone": {"title": "Tier 2"},
    }
    row.update(over)
    return row


# --------------------------------------------------------------------------
# normalization
# --------------------------------------------------------------------------
def test_normalize_project_item_pulls_every_snapshot_field():
    it, unmapped = fb.normalize_project_item(gql_item())
    assert unmapped == []
    assert it["number"] == 12
    assert it["title"] == "Tripod day: decode gate"
    assert it["column"] == "In progress"
    assert it["column_source"] == "project-field:Status"
    assert it["labels"] == ["hardware", "gate"]
    assert it["assignee"] == "Swampyemerson"
    assert it["url"].endswith("/issues/12")
    assert it["state"] == "OPEN"
    assert it["type"] == "Issue"
    # content.updatedAt wins over the project-item updatedAt (it is the work item's own)
    assert it["updated_at"] == "2026-07-24T18:30:00Z"
    # custom fields survive; the built-in Title field is NOT duplicated into them
    assert it["fields"] == {"Phase": "P2", "Blocked by": "#4"}
    assert set(it) == fb.ITEM_KEYS


def test_item_with_no_status_lands_in_an_explicit_bucket_not_a_guess():
    node = gql_item(fieldValues={"nodes": [
        {"__typename": "ProjectV2ItemFieldTextValue", "text": "x", "field": {"name": "Title"}}]})
    it, _ = fb.normalize_project_item(node)
    assert it["column"] == fb.NO_STATUS
    assert it["column_source"] == "no-status"      # never silently "Todo"


def test_draft_issue_has_no_url_and_says_so():
    node = gql_item(type="DRAFT_ISSUE", content={
        "__typename": "DraftIssue", "title": "spike: bank cue", "updatedAt": "2026-07-01T00:00:00Z"})
    it, _ = fb.normalize_project_item(node)
    assert it["type"] == "DraftIssue"
    assert it["url"] is None and it["number"] is None
    assert it["updated_at"] == "2026-07-01T00:00:00Z"


def test_unread_custom_field_types_are_counted_not_dropped_silently():
    node = gql_item(fieldValues={"nodes": [
        {"__typename": "ProjectV2ItemFieldSingleSelectValue", "name": "Todo", "field": {"name": "Status"}},
        {"__typename": "ProjectV2ItemFieldUserValue"},          # not requested by ITEMS_QUERY
        {"__typename": "ProjectV2ItemFieldMilestoneValue"},
    ]})
    _, unmapped = fb.normalize_project_item(node)
    assert sorted(unmapped) == ["ProjectV2ItemFieldMilestoneValue", "ProjectV2ItemFieldUserValue"]


def test_titleless_item_is_a_loud_shape_break_not_an_empty_row():
    with pytest.raises(fb.BoardFetchError) as e:
        fb.normalize_project_item(gql_item(content={"__typename": "Issue", "number": 1}))
    assert e.value.code == fb.EX_BAD_SHAPE


def test_normalize_issue_state_is_the_column_unless_a_status_label_overrides():
    it = fb.normalize_issue(issue_row())
    assert (it["column"], it["column_source"]) == ("Open", "issue-state")
    assert it["fields"] == {"Milestone": "Tier 2"}
    assert set(it) == fb.ITEM_KEYS
    closed = fb.normalize_issue(issue_row(state="CLOSED"))
    assert closed["column"] == "Closed"
    labelled = fb.normalize_issue(issue_row(labels=[{"name": "status: In review"}]))
    assert (labelled["column"], labelled["column_source"]) == ("In review", "label:status:")


def test_unexpected_issue_state_fails_loudly():
    with pytest.raises(fb.BoardFetchError) as e:
        fb.normalize_issue(issue_row(state="WEIRD"))
    assert e.value.code == fb.EX_BAD_SHAPE


# --------------------------------------------------------------------------
# snapshot assembly + the producer-side validator
# --------------------------------------------------------------------------
def _src():
    return {"kind": "project", "project_number": 1, "project_title": "Interceptor",
            "url": "https://github.com/users/x/projects/1", "repo": "x/y", "tool": "gh 2.45.0"}


def test_build_snapshot_orders_declared_columns_first_and_counts_them():
    a, _ = fb.normalize_project_item(gql_item())                       # In progress
    b, _ = fb.normalize_project_item(gql_item(id="PVTI_b", fieldValues={"nodes": []}))  # no status
    snap = fb.build_snapshot(_src(), [a, b], ["Todo", "In progress", "Done"])
    assert snap["columns"] == ["Todo", "In progress", "Done", fb.NO_STATUS]  # empties kept, bucket last
    assert snap["counts"]["items"] == 2
    assert snap["counts"]["by_column"] == {"Todo": 0, "In progress": 1, "Done": 0, fb.NO_STATUS: 1}


def test_empty_board_is_a_valid_snapshot():
    snap = fb.build_snapshot(_src(), [], ["Todo", "Done"])
    assert snap["counts"]["items"] == 0 and snap["items"] == []


@pytest.mark.parametrize("mutate", [
    lambda s: s.update({"unexpected": 1}),
    lambda s: s.pop("columns"),
    lambda s: s["counts"].update({"items": 99}),
    lambda s: s.update({"fetched_at": "yesterday"}),
    lambda s: s["items"][0].update({"column": "Nowhere"}),
    lambda s: s["items"][0].pop("labels"),
    lambda s: s.update({"schema_version": 2}),
])
def test_producer_validator_rejects_broken_snapshots(mutate, tmp_path):
    it, _ = fb.normalize_project_item(gql_item())
    snap = fb.build_snapshot(_src(), [it], ["In progress"])
    mutate(snap)
    with pytest.raises(fb.BoardFetchError) as e:
        fb.write_snapshot(tmp_path / "b.json", snap)
    assert e.value.code == fb.EX_BAD_SHAPE
    assert not (tmp_path / "b.json").exists()      # a rejected snapshot is never written


# --------------------------------------------------------------------------
# failure modes — one cause, one exit code, one actionable message
# --------------------------------------------------------------------------
def _run(monkeypatch, tmp_path, gh_impl, argv=None, have_gh=True):
    """Drive main() with a fake gh. Returns (exit_code, stdout, stderr, out_path)."""
    out = tmp_path / "board_snapshot.json"
    monkeypatch.setattr(fb.shutil, "which", lambda n: "/usr/bin/gh" if have_gh else None)
    monkeypatch.setattr(fb, "run_gh", gh_impl)
    code = fb.main((argv or []) + ["--out", str(out)])
    return code, out


def _gh(responses):
    """responses: list of (match_substring, rc, stdout, stderr) tried in order."""
    def impl(args, timeout=60):
        joined = " ".join(args)
        for m, rc, so, se in responses:
            if m in joined:
                return rc, so, se
        return 0, "{}", ""
    return impl


AUTH_OK = ("auth status", 0, "", "Logged in to github.com account Swampyemerson")
VERSION = ("--version", 0, "gh version 2.45.0 (2025-07-18)\n", "")

SCOPE_ERR = ("error: your authentication token is missing required scopes "
             "[read:project]\nTo request it, run:  gh auth refresh -s read:project")


def test_missing_gh_binary_exit_10(monkeypatch, tmp_path, capsys):
    code, out = _run(monkeypatch, tmp_path, _gh([]), have_gh=False)
    err = capsys.readouterr().err
    assert code == fb.EX_NO_GH == 10
    assert "not installed" in err and "apt install gh" in err
    assert not out.exists()


def test_unauthenticated_exit_11(monkeypatch, tmp_path, capsys):
    code, out = _run(monkeypatch, tmp_path, _gh([
        ("auth status", 1, "", "You are not logged into any GitHub hosts. Run gh auth login")]))
    err = capsys.readouterr().err
    assert code == fb.EX_NO_AUTH == 11
    assert "gh auth login" in err
    assert not out.exists()


def test_missing_read_project_scope_exit_12_with_the_exact_refresh_command(monkeypatch, tmp_path, capsys):
    """TODAY'S REAL CASE — reproduced live on 2026-07-25 against the builder's
    own token (gh scopes: gist, read:org, repo, workflow)."""
    code, out = _run(monkeypatch, tmp_path, _gh([
        AUTH_OK, ("project list", 1, "", SCOPE_ERR)]))
    err = capsys.readouterr().err
    assert code == fb.EX_NO_SCOPE == 12
    assert "read:project" in err
    assert "gh auth refresh -s read:project" in err
    assert not out.exists()


def test_no_project_exists_exit_13_and_points_at_issues_only(monkeypatch, tmp_path, capsys):
    code, out = _run(monkeypatch, tmp_path, _gh([
        AUTH_OK, ("project list", 0, json.dumps({"projects": [], "totalCount": 0}), "")]))
    err = capsys.readouterr().err
    assert code == fb.EX_NO_PROJECT == 13
    assert "--issues-only" in err
    assert not out.exists()


def test_ambiguous_projects_refuse_to_guess_exit_13(monkeypatch, tmp_path, capsys):
    two = {"projects": [{"number": 1, "title": "A", "closed": False},
                        {"number": 5, "title": "B", "closed": False}], "totalCount": 2}
    code, _ = _run(monkeypatch, tmp_path, _gh([AUTH_OK, ("project list", 0, json.dumps(two), "")]))
    err = capsys.readouterr().err
    assert code == fb.EX_NO_PROJECT
    assert "--project <number>" in err and "#1" in err and "#5" in err


def test_repo_not_found_exit_14(monkeypatch, tmp_path, capsys):
    code, out = _run(monkeypatch, tmp_path, _gh([
        AUTH_OK, ("issue list", 1, "", "GraphQL: Could not resolve to a Repository with the name 'x/y'.")]),
        argv=["--issues-only"])
    err = capsys.readouterr().err
    assert code == fb.EX_NO_REPO == 14
    assert not out.exists()


def test_other_gh_failure_exit_15_quotes_gh(monkeypatch, tmp_path, capsys):
    code, out = _run(monkeypatch, tmp_path, _gh([
        AUTH_OK, ("issue list", 1, "", "error connecting to api.github.com: dial tcp: no route to host")]),
        argv=["--issues-only"])
    err = capsys.readouterr().err
    assert code == fb.EX_GH_FAILED == 15
    assert "no route to host" in err
    assert not out.exists()


def test_unparseable_gh_output_exit_16(monkeypatch, tmp_path, capsys):
    code, out = _run(monkeypatch, tmp_path, _gh([
        AUTH_OK, ("issue list", 0, "<!DOCTYPE html> a login page", "")]), argv=["--issues-only"])
    err = capsys.readouterr().err
    assert code == fb.EX_BAD_SHAPE == 16
    assert "not JSON" in err
    assert not out.exists()


def test_mutually_exclusive_sources_are_usage_exit_2(monkeypatch, tmp_path, capsys):
    code, out = _run(monkeypatch, tmp_path, _gh([AUTH_OK]), argv=["--issues-only", "--project", "3"])
    err = capsys.readouterr().err
    assert code == fb.EX_USAGE == 2
    assert "mutually exclusive" in err
    assert not out.exists()


def test_every_failure_code_is_distinct():
    codes = [fb.EX_NO_GH, fb.EX_NO_AUTH, fb.EX_NO_SCOPE, fb.EX_NO_PROJECT,
             fb.EX_NO_REPO, fb.EX_GH_FAILED, fb.EX_BAD_SHAPE, fb.EX_USAGE]
    assert len(set(codes)) == len(codes)
    assert fb.EX_OK not in codes                 # nothing failing shares the success code


# --------------------------------------------------------------------------
# THE headline distinction: empty board vs failed fetch
# --------------------------------------------------------------------------
def test_empty_board_writes_a_real_snapshot_and_exits_0(monkeypatch, tmp_path, capsys):
    code, out = _run(monkeypatch, tmp_path, _gh([AUTH_OK, VERSION, ("issue list", 0, "[]", "")]),
                     argv=["--issues-only"])
    cap = capsys.readouterr()
    assert code == fb.EX_OK
    snap = json.loads(out.read_text())
    assert snap["counts"]["items"] == 0 and snap["items"] == []
    assert "BOARD IS EMPTY" in cap.out            # said out loud, not implied by silence
    assert "last_error" not in snap               # an empty board is NOT an error state


def test_failed_fetch_leaves_an_existing_snapshot_intact_and_stamps_the_failure(
        monkeypatch, tmp_path, capsys):
    """The stale-data rule: a broken fetch must not empty the board, must not
    re-date it, and must not pass the old data off as current."""
    # 1) a good snapshot exists, written by the producer's own writer
    it, _ = fb.normalize_project_item(gql_item())
    good = fb.build_snapshot(_src(), [it], ["In progress"], fetched_at="2026-07-01T00:00:00Z")
    out = tmp_path / "board_snapshot.json"
    fb.write_snapshot(out, good)

    # 2) the next fetch fails on the scope
    monkeypatch.setattr(fb.shutil, "which", lambda n: "/usr/bin/gh")
    monkeypatch.setattr(fb, "run_gh", _gh([AUTH_OK, ("project list", 1, "", SCOPE_ERR)]))
    code = fb.main(["--out", str(out)])
    err = capsys.readouterr().err
    assert code == fb.EX_NO_SCOPE

    after = json.loads(out.read_text())
    assert after["items"] == good["items"]                    # data untouched
    assert after["fetched_at"] == "2026-07-01T00:00:00Z"      # NOT re-dated as fresh
    assert after["counts"]["items"] == 1                      # not emptied
    assert after["last_error"]["cause"] == "NO_SCOPE"
    assert after["last_error"]["exit_code"] == 12
    assert "read:project" in after["last_error"]["message"]
    assert "still as of 2026-07-01T00:00:00Z" in err          # operator told the real age


def test_successful_fetch_clears_a_previous_last_error(monkeypatch, tmp_path):
    out = tmp_path / "board_snapshot.json"
    fb.write_snapshot(out, fb.build_snapshot(_src(), [], ["Open", "Closed"]))
    fb.record_fetch_failure(out, fb.BoardFetchError(fb.EX_NO_SCOPE, "boom", "fix me"))
    assert "last_error" in json.loads(out.read_text())
    monkeypatch.setattr(fb.shutil, "which", lambda n: "/usr/bin/gh")
    monkeypatch.setattr(fb, "run_gh", _gh([AUTH_OK, VERSION, ("issue list", 0, "[]", "")]))
    assert fb.main(["--issues-only", "--out", str(out)]) == fb.EX_OK
    assert "last_error" not in json.loads(out.read_text())


def test_dry_run_writes_nothing(monkeypatch, tmp_path, capsys):
    code, out = _run(monkeypatch, tmp_path,
                     _gh([AUTH_OK, VERSION, ("issue list", 0, json.dumps([issue_row()]), "")]),
                     argv=["--issues-only", "--dry-run"])
    printed = json.loads(capsys.readouterr().out)
    assert code == fb.EX_OK
    assert printed["counts"]["items"] == 1
    assert not out.exists()


def test_full_project_path_end_to_end_with_a_fake_gh(monkeypatch, tmp_path):
    """The whole project driver (discover -> graphql -> normalize -> write),
    with gh faked at the run_gh seam."""
    project = {"number": 3, "title": "Interceptor board",
               "url": "https://github.com/users/Swampyemerson/projects/3",
               "field": {"name": "Status", "options": [{"name": "Todo"}, {"name": "In progress"},
                                                       {"name": "Done"}]},
               "items": {"totalCount": 2, "pageInfo": {"hasNextPage": False, "endCursor": None},
                         "nodes": [gql_item(),
                                   gql_item(id="PVTI_arch", isArchived=True)]}}
    gh = _gh([
        AUTH_OK, VERSION,
        ("project list", 0, json.dumps({"projects": [{"number": 3, "title": "Interceptor board",
                                                      "closed": False}], "totalCount": 1}), ""),
        ("api graphql", 0, json.dumps({"data": {"owner": {"projectV2": project}}}), ""),
    ])
    code, out = _run(monkeypatch, tmp_path, gh)
    assert code == fb.EX_OK
    snap = json.loads(out.read_text())
    assert snap["source"] == {"kind": "project", "project_number": 3,
                              "project_title": "Interceptor board",
                              "url": "https://github.com/users/Swampyemerson/projects/3",
                              "repo": fb.DEFAULT_REPO, "tool": "gh 2.45.0"}
    assert snap["columns"] == ["Todo", "In progress", "Done"]
    assert snap["counts"]["by_column"] == {"Todo": 0, "In progress": 1, "Done": 0}
    assert any("archived" in w for w in snap["warnings"])   # the dropped item is COUNTED
    fb.validate_snapshot(snap)
