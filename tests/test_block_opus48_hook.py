#!/usr/bin/env python3
"""Tests for `scripts/hooks/block_opus48.py` -- the PreToolUse hook that DENIES
any subagent/Workflow spawn on Opus 4.8 (builder directive 2026-07-24: "I don't
want it accidentally used"; wired in `.claude/settings.json` on `Agent|Workflow`).

WHY THIS FILE EXISTS. The hook landed 2026-07-25 (c289086/fad4c11) with ZERO
coverage: it is an ORCHESTRATION INSTRUMENT sitting outside every gate the
project had just built, and its two failure modes are asymmetric and both bad --
a missed deny silently runs work on a forbidden model, and a FALSE deny strands
parallel work (the standing order's worst case, because a refused spawn looks
like nothing happened). The audit's completeness critic found exactly one live
false-deny: the Workflow regex ran against the RAW script text, so a workflow
whose COMMENT quoted `model: 'opus'` -- e.g. one documenting this very rule --
was denied. `strip_comments()` and the last group of tests here are that fix.

The hook is invoked as a SUBPROCESS by the harness, so these tests drive it as
one (real stdin -> real stdout -> real exit code); a couple of unit tests cover
`strip_comments` directly because its quote-awareness is the subtle part.

CONTRACT UNDER TEST (docs, .claude/settings.json):
  * deny  = exit 0 with a JSON hookSpecificOutput permissionDecision "deny"
  * allow = exit 0 with NO output (the hook stays silent and the harness proceeds)
  * FAIL OPEN on garbage stdin -- a harness format change must never brick every
    spawn, so malformed input is an ALLOW, not a crash.
"""
import json
import os
import subprocess
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HOOK = os.path.join(REPO_ROOT, "scripts", "hooks", "block_opus48.py")

sys.path.insert(0, os.path.join(REPO_ROOT, "scripts", "hooks"))
from block_opus48 import strip_comments  # noqa: E402


# ---------------------------------------------------------------- helpers
def run_hook(payload, raw=None):
    """Drive the hook exactly as the harness does. Returns (exit_code, decision)
    where decision is 'deny', or None when the hook allowed (silent)."""
    stdin = raw if raw is not None else json.dumps(payload)
    p = subprocess.run([sys.executable, HOOK], input=stdin,
                       capture_output=True, text=True, timeout=30)
    out = p.stdout.strip()
    if not out:
        return p.returncode, None, ""
    data = json.loads(out)
    hso = data["hookSpecificOutput"]
    assert hso["hookEventName"] == "PreToolUse"
    return p.returncode, hso["permissionDecision"], hso.get("permissionDecisionReason", "")


def agent(**tool_input):
    return {"tool_name": "Agent", "tool_input": tool_input}


def workflow(script):
    return {"tool_name": "Workflow", "tool_input": {"script": script}}


# ---------------------------------------------------------------- Agent: DENY
@pytest.mark.parametrize("model", [
    "opus",             # THE ambiguous alias -- resolves per running CLI binary
    "Opus",             # case
    "  opus  ",         # whitespace (the hook .strip()s)
    "claude-opus-4-8",
    "claude-opus-4-1-20250805",
    "CLAUDE-OPUS-4-5",
])
def test_agent_spawn_on_opus48_or_the_bare_alias_is_denied(model):
    rc, decision, reason = run_hook(agent(model=model, prompt="do a thing"))
    assert rc == 0, "the hook must always exit 0; it signals via JSON, not status"
    assert decision == "deny", f"model {model!r} must be denied"
    assert "block_opus48" in reason and "opus5-worker" in reason, (
        "the deny reason must name the blocker and the sanctioned way forward")


# ---------------------------------------------------------------- Agent: ALLOW
@pytest.mark.parametrize("tool_input", [
    {"model": "claude-opus-5", "prompt": "x"},          # the explicit pin
    {"model": "claude-opus-5[1m]", "prompt": "x"},      # long-context variant
    {"model": "claude-sonnet-5", "prompt": "x"},        # the volume lane
    {"model": "claude-fable-5", "prompt": "x"},         # review/judgment lane
    {"prompt": "x"},                                    # no model key at all
    {"model": None, "prompt": "x"},                     # explicit null
    {"subagent_type": "opus5-worker", "prompt": "x"},   # the pinned agent type
    {"model": "", "prompt": "x"},                       # empty string
])
def test_allowed_agent_spawns_pass_silently(tool_input):
    rc, decision, _ = run_hook(agent(**tool_input))
    assert rc == 0
    assert decision is None, f"{tool_input} must pass with NO hook output"


def test_opus5_is_not_caught_by_the_claude_opus_4_prefix_rule():
    """Regression pin on the blocked-model regex: `claude-opus-4.*` must not be
    loosened to `claude-opus-.*`, which would block the workhorse lane."""
    rc, decision, _ = run_hook(agent(model="claude-opus-5", prompt="x"))
    assert decision is None


def test_a_non_spawn_tool_is_never_touched():
    """The hook is wired on Agent|Workflow, but it must be inert if the matcher
    ever widens -- a Bash command mentioning `model: 'opus'` is not a spawn."""
    rc, decision, _ = run_hook(
        {"tool_name": "Bash", "tool_input": {"command": "echo \"model: 'opus'\""}})
    assert rc == 0 and decision is None


# ------------------------------------------------------------- Workflow: DENY
def test_workflow_script_pinning_the_bare_opus_alias_is_denied():
    script = """
const a = agent({ agentType: 'general', model: 'opus', prompt: 'go' });
await a.result();
"""
    rc, decision, reason = run_hook(workflow(script))
    assert rc == 0
    assert decision == "deny"
    assert "opus" in reason


def test_workflow_script_pinning_an_explicit_opus4_id_is_denied():
    script = "agent({ model: \"claude-opus-4-8\", prompt: 'go' })"
    rc, decision, reason = run_hook(workflow(script))
    assert decision == "deny"
    assert "claude-opus-4-8" in reason


def test_workflow_scriptpath_is_read_and_denied(tmp_path):
    """A workflow can arrive as a PATH instead of inline text; the file contents
    must be scanned too, or the block is trivially bypassable."""
    p = tmp_path / "wf.js"
    p.write_text("agent({ model: 'opus', prompt: 'go' })")
    rc, decision, _ = run_hook(
        {"tool_name": "Workflow", "tool_input": {"scriptPath": str(p)}})
    assert decision == "deny"


def test_workflow_scriptpath_that_does_not_exist_fails_open(tmp_path):
    rc, decision, _ = run_hook(
        {"tool_name": "Workflow",
         "tool_input": {"scriptPath": str(tmp_path / "nope.js")}})
    assert rc == 0 and decision is None


# ------------------------------------------------------------ Workflow: ALLOW
def test_workflow_using_the_opus5_worker_agent_type_is_allowed():
    script = """
const a = agent({ agentType: 'opus5-worker', prompt: 'do the substantive build' });
await a.result();
"""
    rc, decision, _ = run_hook(workflow(script))
    assert rc == 0 and decision is None


def test_workflow_pinning_claude_opus_5_explicitly_is_allowed():
    rc, decision, _ = run_hook(
        workflow("agent({ model: 'claude-opus-5', prompt: 'go' })"))
    assert decision is None


# ------------------- THE FALSE-DENY the audit found (fixed 2026-07-25) -------
def test_workflow_whose_LINE_COMMENT_mentions_opus_is_allowed():
    """THE regression this file was written for. A workflow documenting the rule
    -- `// never pass model: 'opus'` -- was DENIED before strip_comments(),
    stranding legitimate parallel work with a refusal that looks like silence."""
    script = """
// Per CLAUDE.md: never pass model: 'opus' -- the bare alias is binary-dependent.
const a = agent({ agentType: 'opus5-worker', prompt: 'build it' });
await a.result();
"""
    rc, decision, reason = run_hook(workflow(script))
    assert rc == 0
    assert decision is None, f"false deny on a comment-only mention: {reason}"


def test_workflow_whose_BLOCK_COMMENT_mentions_opus_is_allowed():
    script = """
/* Historical note: spawns used to read
       model: 'claude-opus-4-8'
   which is now forbidden. */
agent({ model: 'claude-opus-5', prompt: 'go' })
"""
    rc, decision, reason = run_hook(workflow(script))
    assert decision is None, f"false deny on a block-comment mention: {reason}"


def test_a_real_pin_on_the_SAME_LINE_as_a_comment_is_still_denied():
    """Stripping must not become a bypass: code before a comment still counts."""
    script = "agent({ model: 'opus' }); // documented above\n"
    rc, decision, _ = run_hook(workflow(script))
    assert decision == "deny"


def test_a_pin_after_a_block_comment_is_still_denied():
    script = "/* model: 'opus' is banned */ agent({ model: 'opus' })"
    rc, decision, _ = run_hook(workflow(script))
    assert decision == "deny"


# ---------------------------------------------------------- strip_comments
def test_strip_comments_leaves_string_contents_alone():
    """A `//` inside a quoted string is NOT a comment -- otherwise a URL or a
    prompt in a workflow would silently swallow the rest of its line, which is
    how a naive strip turns into a MISSED deny."""
    src = "const u = 'https://example.com/x'; agent({ model: 'opus' })"
    assert "model: 'opus'" in strip_comments(src)


def test_strip_comments_preserves_line_structure():
    src = "a\n// gone\n/* also\ngone */\nb\n"
    assert strip_comments(src).count("\n") == src.count("\n")


def test_strip_comments_handles_escaped_quotes():
    src = r"""const s = 'it\'s // not a comment'; model: 'opus'"""
    assert "model: 'opus'" in strip_comments(src)


def test_strip_comments_handles_an_unterminated_block_comment():
    assert strip_comments("ok /* never closed") == "ok "


# ---------------------------------------------------------------- fail-open
@pytest.mark.parametrize("raw", [
    "",                       # empty stdin
    "not json at all",
    "{ broken",
    "null",
    "[]",                     # valid JSON, wrong shape
    '{"tool_name": "Agent"}',            # no tool_input key
    '{"tool_name": "Agent", "tool_input": null}',
    '{"tool_input": {"model": "opus"}}',  # no tool_name -> not a spawn we know
])
def test_malformed_or_unexpected_stdin_fails_open_with_exit_0(raw):
    """A harness payload-format change must NOT brick every subagent spawn."""
    rc, decision, _ = run_hook(None, raw=raw)
    assert rc == 0, f"hook must exit 0 on {raw!r}"
    assert decision is None, f"hook must ALLOW on {raw!r}, not deny"


def test_agent_model_of_a_nonstring_type_fails_open():
    rc, decision, _ = run_hook(agent(model={"weird": 1}, prompt="x"))
    assert rc == 0 and decision is None


# ------------------------------------------------------- wiring (the instrument)
def test_hook_is_actually_wired_in_claude_settings():
    """A perfect hook that nothing invokes is inert -- the project's own
    'a fix is not done until its effect is observed' rule, applied to config."""
    with open(os.path.join(REPO_ROOT, ".claude", "settings.json")) as f:
        settings = json.load(f)
    blob = json.dumps(settings)
    assert "block_opus48.py" in blob, (
        "scripts/hooks/block_opus48.py is not referenced from .claude/settings.json")
    pre = settings.get("hooks", {}).get("PreToolUse", [])
    matchers = [entry.get("matcher", "") for entry in pre
                if "block_opus48" in json.dumps(entry)]
    assert matchers, "block_opus48 is not registered under hooks.PreToolUse"
    for m in matchers:
        assert "Agent" in m and "Workflow" in m, (
            f"matcher {m!r} must cover BOTH spawn surfaces")
