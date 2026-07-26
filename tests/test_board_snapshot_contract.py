"""PRODUCER -> CONSUMER contract test for the board snapshot.

    scripts/fetch_board.py  --(docs/board_snapshot.json)-->  scripts/render_dashboard.py

docs/error_handling_policy.md §6: when a tool crosses a file boundary it earns
one test whose FIXTURE COMES FROM THE PRODUCER'S OWN WRITER, run against the
REAL consumer. Hand-typed fixtures are exactly what hid two schema breaks in
this repo — both scripts passed their own self-tests; nothing tested the pair.
So every snapshot below is built by fetch_board.build_snapshot + written by
fetch_board.write_snapshot, and then rendered/checked by the real
render_dashboard entry points (the two validators are deliberately separate
implementations; THIS is what binds them).

Also pinned here — the three board states the dashboard must keep distinct:
  absent snapshot -> renders, embeds `null`, says "not yet connected"
  empty snapshot  -> renders, embeds a real snapshot with 0 items
  deleted after a render -> --check FAILS (the page must not serve dead data)

Offline, stdlib + pytest only.
"""
import json
import os
import shutil
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import fetch_board as fb                                            # noqa: E402
import render_dashboard as rd                                       # noqa: E402


# --------------------------------------------------------------------------
# helpers: a sandbox copy of the real dashboard + producer-written fixtures
# --------------------------------------------------------------------------
@pytest.fixture
def sandbox(tmp_path, monkeypatch):
    """Point the renderer at a COPY of the real docs/dashboard.html so the test
    exercises the shipped page (markers, embed blocks, guards) without touching
    the repo file. The state stays the real contract."""
    html = tmp_path / "dashboard.html"
    shutil.copyfile(rd.HTML, html)
    board = tmp_path / "board_snapshot.json"
    monkeypatch.setattr(rd, "HTML", html)
    monkeypatch.setattr(rd, "BOARD", board)
    return html, board


def producer_snapshot(items_raw=None, columns=None, fetched_at=None):
    """THE fixture factory: normalized + assembled + validated by fetch_board's
    own code path. Nothing here is hand-typed JSON."""
    items = []
    for raw in (items_raw or []):
        it, _ = fb.normalize_project_item(raw)
        items.append(it)
    src = {"kind": "project", "project_number": 3, "project_title": "Interceptor board",
           "url": "https://github.com/users/Swampyemerson/projects/3",
           "repo": "Swampyemerson/interceptor-sim", "tool": "gh 2.45.0"}
    return fb.build_snapshot(src, items, columns or ["Todo", "In progress", "Done"],
                             warnings=["1 archived item(s) excluded from this snapshot"],
                             fetched_at=fetched_at)


def node(id_, title, status, number, **over):
    n = {
        "id": id_, "type": "ISSUE", "isArchived": False, "updatedAt": "2026-07-20T09:00:00Z",
        "fieldValues": {"nodes": [
            {"__typename": "ProjectV2ItemFieldSingleSelectValue", "name": status,
             "field": {"name": "Status"}},
            {"__typename": "ProjectV2ItemFieldSingleSelectValue", "name": "P2",
             "field": {"name": "Phase"}}]},
        "content": {"__typename": "Issue", "number": number, "title": title,
                    "url": "https://github.com/Swampyemerson/interceptor-sim/issues/%d" % number,
                    "state": "OPEN", "updatedAt": "2026-07-24T18:30:00Z",
                    "labels": {"nodes": [{"name": "hardware"}]},
                    "assignees": {"nodes": [{"login": "Swampyemerson"}]}},
    }
    n.update(over)
    return n


def render(argv=None):
    """Call the REAL renderer entry point (it reads sys.argv)."""
    old = sys.argv
    sys.argv = ["render_dashboard.py"] + list(argv or [])
    try:
        rd.main()
    finally:
        sys.argv = old


# --------------------------------------------------------------------------
# the seam
# --------------------------------------------------------------------------
def test_producer_output_passes_the_consumer_validator_and_round_trips(sandbox, capsys):
    html, board_path = sandbox
    snap = producer_snapshot([node("PVTI_a", "Tripod day: decode gate", "In progress", 12),
                              node("PVTI_b", "Order the Hailo", "Todo", 13)])
    fb.write_snapshot(board_path, snap)          # producer's own writer

    render()                                     # real consumer
    out = capsys.readouterr().out
    assert "2 item(s)" in out and "fetched" in out

    embedded = rd.extract_embedded_board(html.read_text())
    assert embedded == json.loads(board_path.read_text()) == snap
    rd.validate_board(embedded)                  # consumer schema check on producer bytes

    render(["--check"])                          # drift check green on the pair
    assert "embed in sync" in capsys.readouterr().out


def test_populated_board_actually_reaches_the_rendered_page(sandbox):
    """A fix/feature is not done until its EFFECT is observed: the item's title
    and URL must be present in the bytes the browser will parse."""
    html, board_path = sandbox
    fb.write_snapshot(board_path, producer_snapshot(
        [node("PVTI_a", "Tripod day: decode gate", "In progress", 12)]))
    render()
    text = html.read_text()
    assert "Tripod day: decode gate" in text
    assert "https://github.com/Swampyemerson/interceptor-sim/issues/12" in text
    assert '<script id="board" type="application/json">' in text
    assert "view-board" in text and "Sheet 3" in text


def test_absent_snapshot_still_renders_and_embeds_the_null_sentinel(sandbox, capsys):
    html, board_path = sandbox
    assert not board_path.exists()
    render()
    assert "NOT CONNECTED" in capsys.readouterr().out
    assert rd.extract_embedded_board(html.read_text()) is None
    render(["--check"])
    assert "NOT CONNECTED" in capsys.readouterr().out


def test_empty_board_and_absent_board_are_different_embeds(sandbox):
    """The headline distinction, at the file boundary: 0 items is a real
    snapshot object; not-connected is `null`. A reader (and the page) can tell
    them apart without guessing."""
    html, board_path = sandbox
    render()
    assert rd.extract_embedded_board(html.read_text()) is None
    fb.write_snapshot(board_path, producer_snapshot([]))
    render()
    emb = rd.extract_embedded_board(html.read_text())
    assert emb is not None and emb["counts"]["items"] == 0


def test_deleting_the_snapshot_after_a_render_fails_check_loudly(sandbox, capsys):
    """Stale-data guard: the page may not keep serving a snapshot whose source
    file is gone."""
    html, board_path = sandbox
    fb.write_snapshot(board_path, producer_snapshot([node("PVTI_a", "x", "Todo", 1)]))
    render()
    board_path.unlink()
    with pytest.raises(SystemExit):
        render(["--check"])
    assert "GONE" in capsys.readouterr().err


def test_stale_embed_after_a_refetch_fails_check(sandbox, capsys):
    html, board_path = sandbox
    fb.write_snapshot(board_path, producer_snapshot([], fetched_at="2026-07-01T00:00:00Z"))
    render()
    fb.write_snapshot(board_path, producer_snapshot([], fetched_at="2026-07-25T00:00:00Z"))
    with pytest.raises(SystemExit):
        render(["--check"])
    err = capsys.readouterr().err
    assert "embedded board DIFFERS" in err and "2026-07-01T00:00:00Z" in err


def test_a_failed_fetch_stamps_last_error_and_the_consumer_renders_it(sandbox, capsys):
    """End-to-end on the nastiest path: old data + a failed refresh. The
    snapshot keeps its ORIGINAL fetched_at, the renderer accepts last_error,
    and the failure text reaches the page."""
    html, board_path = sandbox
    fb.write_snapshot(board_path, producer_snapshot(
        [node("PVTI_a", "Tripod day", "Todo", 12)], fetched_at="2026-07-01T00:00:00Z"))
    fb.record_fetch_failure(board_path, fb.BoardFetchError(
        fb.EX_NO_SCOPE, "the gh token lacks the read:project scope",
        "gh auth refresh -s read:project"))
    render()
    out = capsys.readouterr().out
    assert "LAST FETCH FAILED" in out and "STALE" in out
    emb = rd.extract_embedded_board(html.read_text())
    assert emb["fetched_at"] == "2026-07-01T00:00:00Z"
    assert emb["last_error"]["cause"] == "NO_SCOPE"
    assert "read:project" in html.read_text()


@pytest.mark.parametrize("mutate,needle", [
    (lambda s: s.update({"surprise_field": 1}), "unknown top-level keys"),
    (lambda s: s["counts"].update({"items": 42}), "counts.items"),
    (lambda s: s["items"][0].update({"column": "Ghost"}), "not in columns"),
    (lambda s: s.update({"fetched_at": "2026-07-25 12:00"}), "not ISO-8601"),
    (lambda s: s["source"].pop("tool"), "source keys"),
    (lambda s: s.update({"last_error": {"cause": "X"}}), "last_error keys"),
])
def test_consumer_refuses_a_hand_edited_snapshot(sandbox, mutate, needle, capsys):
    """The snapshot is machine-written; if someone hand-edits it (or a future gh
    changes shape), the RENDER stops rather than drawing a half-understood
    board."""
    html, board_path = sandbox
    snap = producer_snapshot([node("PVTI_a", "x", "Todo", 1)])
    mutate(snap)
    board_path.write_text(json.dumps(snap))      # bypass the writer on purpose
    with pytest.raises(SystemExit):
        render()
    assert needle in capsys.readouterr().err


def test_consumer_and_producer_agree_on_the_key_sets():
    """Cheap drift alarm between the two hand-maintained schemas."""
    assert rd.BOARD_TOP_KEYS == fb.TOP_KEYS
    assert rd.BOARD_SOURCE_KEYS == fb.SOURCE_KEYS
    assert rd.BOARD_ITEM_KEYS == fb.ITEM_KEYS


def test_repo_dashboard_carries_the_board_markers_and_sheet():
    """The shipped page (not a copy) must actually have the Sheet 3 plumbing."""
    text = rd.HTML.read_text()
    for needle in (rd.B_BEGIN, rd.B_END, 'id="view-board"', 'id="tab-board"',
                   'id="bd-cols"', 'id="bd-state-msg"'):
        assert needle in text, needle
