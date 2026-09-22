#!/usr/bin/env python3
"""PreToolUse hook: block AMBIGUOUS or unsanctioned Opus-4 subagent spawns.

HISTORY. Builder directive 2026-07-24: NO work on Opus 4.8 ("I don't want it
accidentally used"). AMENDED by builder directive 2026-09-22: with Opus 5 and
Fable 5.1 subagents bouncing on this repo (docs/subagent_safeguard_log.md),
the sanctioned subagent lanes are now Fable 5, **Opus 4.8**, and Sonnet 5 —
so an EXPLICIT `claude-opus-4-8` pin is ALLOWED again (use subagent_type:
opus48-worker). What stays blocked is the ACCIDENT surface the 2026-07-24
rule was really about: the bare 'opus' alias, whose resolution is
VERSION-DEPENDENT on the running CLI binary (root-caused 2026-07-25: a
long-lived `claude --continue` process on 2.1.204 mapped
opus->claude-opus-4-8 while the on-disk 2.1.220 maps opus->claude-opus-5),
and any OTHER claude-opus-4* id (4-1, 4-5, ...), which nobody sanctioned.

Covers the two spawn surfaces:
  - Agent tool calls with model: "opus" or "claude-opus-4*"
  - Workflow scripts (inline or scriptPath) whose agent() opts pin
    model: 'opus' / 'claude-opus-4*'

Fails OPEN on unparseable hook payloads (a harness format change must not
brick every subagent spawn); the deny paths themselves are exact-match.

FALSE-DENY FIXED 2026-07-25 (audit, completeness critic #3): the Workflow regex
was applied to the RAW script text, so a workflow whose COMMENT merely quoted
`model: 'opus'` -- e.g. one documenting this very rule -- was denied. A denied
spawn strands parallel work silently, which the standing order treats as the
worst case, so comments are now stripped before matching. Comment stripping is
string-aware: `//` inside a quoted string (a URL, a prompt) is NOT a comment.
"""
import json
import re
import sys

# `claude-opus-4-8...` (any suffix: dated ids, [1m]) is the 2026-09-22
# sanctioned lane and passes; the bare alias and every OTHER opus-4 id deny.
BLOCKED_MODEL = re.compile(r"^(opus|claude-opus-4(?!-8).*)$", re.IGNORECASE)
BLOCKED_IN_SCRIPT = re.compile(
    r"model\s*:\s*['\"](opus|claude-opus-4(?!-8)[^'\"]*)['\"]", re.IGNORECASE
)


def strip_comments(src: str) -> str:
    """Blank out // line comments and /* */ block comments in JS-ish source,
    leaving everything else (including string contents and line structure)
    byte-for-byte. Quote-aware, so `"https://x"` and a prompt string that
    mentions a comment marker survive intact.

    Newlines inside stripped regions are PRESERVED so any future line-number
    reporting stays honest."""
    out = []
    i, n = 0, len(src)
    quote = None
    while i < n:
        ch = src[i]
        if quote:
            out.append(ch)
            if ch == "\\" and i + 1 < n:      # escape: copy the pair verbatim
                out.append(src[i + 1])
                i += 2
                continue
            if ch == quote:
                quote = None
            i += 1
            continue
        if ch in ("'", '"', "`"):
            quote = ch
            out.append(ch)
            i += 1
            continue
        if ch == "/" and i + 1 < n and src[i + 1] == "/":
            while i < n and src[i] != "\n":
                i += 1
            continue
        if ch == "/" and i + 1 < n and src[i + 1] == "*":
            i += 2
            while i < n and not (src[i] == "*" and i + 1 < n and src[i + 1] == "/"):
                if src[i] == "\n":
                    out.append("\n")
                i += 1
            i += 2
            continue
        out.append(ch)
        i += 1
    return "".join(out)


def deny(reason: str) -> None:
    print(
        json.dumps(
            {
                "hookSpecificOutput": {
                    "hookEventName": "PreToolUse",
                    "permissionDecision": "deny",
                    "permissionDecisionReason": reason,
                }
            }
        )
    )
    sys.exit(0)


def main() -> None:
    try:
        data = json.load(sys.stdin)
    except (json.JSONDecodeError, ValueError):
        sys.exit(0)
    # FAIL OPEN on a well-formed-JSON but wrong-SHAPE payload too (`null`, a
    # list, a string). Before 2026-07-25 this raised AttributeError on the
    # .get() below and the hook exited 1 with a traceback -- an error surface on
    # every spawn, not the documented silent allow. Found by
    # tests/test_block_opus48_hook.py.
    if not isinstance(data, dict):
        sys.exit(0)
    tool = data.get("tool_name", "")
    tool_input = data.get("tool_input")
    if not isinstance(tool_input, dict):
        tool_input = {}

    if tool == "Agent":
        model = tool_input.get("model")
        if isinstance(model, str) and BLOCKED_MODEL.match(model.strip()):
            deny(
                f"BLOCKED by scripts/hooks/block_opus48.py: model '{model}' is the "
                "ambiguous 'opus' alias (resolution depends on the running CLI "
                "binary; a stale --continue process resolved it to claude-opus-4-8 "
                "on 2026-07-25) or an unsanctioned claude-opus-4* id. Sanctioned "
                "subagent lanes (builder 2026-09-22): subagent_type 'opus48-worker' "
                "(pinned claude-opus-4-8), 'opus5-worker' (pinned claude-opus-5), "
                "'sonnet-worker', or an explicit model id."
            )

    elif tool == "Workflow":
        script = tool_input.get("script") or ""
        script_path = tool_input.get("scriptPath")
        if isinstance(script_path, str) and script_path:
            try:
                with open(script_path, "r", encoding="utf-8", errors="replace") as f:
                    script += "\n" + f.read()
            except OSError:
                pass
        match = BLOCKED_IN_SCRIPT.search(strip_comments(script))
        if match:
            deny(
                f"BLOCKED by scripts/hooks/block_opus48.py: this workflow script pins "
                f"an agent to '{match.group(1)}' -- the ambiguous 'opus' alias or an "
                "unsanctioned claude-opus-4* id. Sanctioned (builder 2026-09-22): "
                "agentType 'opus48-worker'/'opus5-worker'/'sonnet-worker', or an "
                "explicit model id ('claude-opus-4-8', 'claude-opus-5') in the "
                "agent() opts."
            )

    sys.exit(0)


if __name__ == "__main__":
    main()
