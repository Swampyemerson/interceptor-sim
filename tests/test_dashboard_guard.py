"""Drift guard for the HAND-AUTHORED dashboard layer + the --artifact contract.

WHY (review-2 structural finding, 2026-07-24): render_dashboard --check
validated only the embedded JSON, so the hand-authored prose/SVG could assert
REVERSED conclusions while run_tests.sh stayed green — it happened twice that
day (hero SVG sold a refuted fix; a stage said 'NOTHING ORDERED YET' four days
after ~$1,000 shipped). The mechanical floor added 2026-07-25: every decimal
number VISIBLE in the hand-authored layer must appear verbatim in
project_state.json's raw text. Also pinned here: `--check --artifact` must
fail loudly (it used to silently write nothing and print OK/exit 0), and
`artifact_url` is a REQUIRED contract key (the republish ritual depends on it).

Offline, stdlib only. Run: .venv/bin/python -m pytest tests/test_dashboard_guard.py -v
"""
import json
import os
import subprocess
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import render_dashboard as rd                                     # noqa: E402

RD = os.path.join(SCRIPTS, "render_dashboard.py")


def _mini_html(hand_text):
    """A minimal dashboard shape: hand-authored text + an embedded-state block,
    plus numbers in style/script/comment that must all be IGNORED."""
    return (
        "<html><head><style>.x{margin:1.25em;line-height:3.14}</style></head>"
        f"<body><p>{hand_text}</p>\n"
        f"{rd.BEGIN}\n<script id=\"state\" type=\"application/json\">"
        "{\"only\": 88.88}</script>\n"
        f"{rd.END}\n"
        "<script>var y = 9.42;</script><!-- 7.77 in a comment -->"
        "</body></html>")


def test_guard_is_green_on_the_current_tree():
    """Ship-gate: the guard must pass clean on HEAD (verified 20 visible
    numbers all traced on 2026-07-25) — a gate that is born red is noise."""
    n = rd.check_hand_layer_numbers(rd.HTML.read_text(), rd.STATE.read_text())
    assert n >= 1, "dashboard hand layer suddenly has no visible numbers?"


def test_guard_fails_on_an_orphan_visible_number():
    """THE mutant: a hand-layer claim ('9.99 m') with no source in the contract
    must fail the check — this is the reversed-conclusion drift signature."""
    html = _mini_html("miss improved to 9.99 m after the fix")
    with pytest.raises(SystemExit):
        rd.check_hand_layer_numbers(html, '{"miss_m": 1.23}')


def test_guard_traces_a_sourced_number_and_ignores_nonvisible_ones():
    html = _mini_html("miss improved to 1.23 m")
    # 1.23 is in the contract text; 1.25/3.14 (style), 9.42 (script), 7.77
    # (comment), 88.88 (embedded state block) are all non-visible and ignored.
    assert rd.check_hand_layer_numbers(html, '{"miss_m": 1.23}') == 1
    assert rd.check_hand_layer_numbers(_mini_html("no decimals here"), "{}") == 0


def test_check_still_green_end_to_end_and_states_its_scope():
    proc = subprocess.run([sys.executable, RD, "--check"],
                          capture_output=True, text=True, cwd=REPO_ROOT)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "visible numbers traced" in proc.stdout
    assert "prose wording NOT checked" in proc.stdout  # scope honesty


def test_check_artifact_combo_fails_loudly_and_writes_nothing(tmp_path):
    """Pre-fix: --check --artifact wrote NOTHING and printed OK/exit 0 — an
    operator believed a fresh artifact copy existed."""
    out = tmp_path / "artifact.html"
    proc = subprocess.run([sys.executable, RD, "--check", "--artifact", str(out)],
                          capture_output=True, text=True, cwd=REPO_ROOT)
    assert proc.returncode != 0
    assert "--check --artifact" in proc.stderr
    assert not out.exists()


def test_artifact_url_is_a_required_contract_key():
    state = json.loads(rd.STATE.read_text())
    rd.validate(state)  # current contract passes
    state.pop("artifact_url")
    with pytest.raises(SystemExit):
        rd.validate(state)
