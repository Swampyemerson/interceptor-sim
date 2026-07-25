#!/usr/bin/env python3
"""PreToolUse hook: block any subagent spawn on Opus 4.8.

Builder directive 2026-07-24: NO work on Opus 4.8. The Agent tool's bare
'opus' alias still resolves to claude-opus-4-8 (re-verified live 2026-07-25
in a fresh session), so both the alias and any explicit claude-opus-4* id
are denied here. Opus 5 stays available via subagent_type: opus5-worker
(pinned claude-opus-5) or an explicit 'claude-opus-5' model string.

Covers the two spawn surfaces:
  - Agent tool calls with model: "opus" or "claude-opus-4*"
  - Workflow scripts (inline or scriptPath) whose agent() opts pin
    model: 'opus' / 'claude-opus-4*'

Fails OPEN on unparseable hook payloads (a harness format change must not
brick every subagent spawn); the deny paths themselves are exact-match.
"""
import json
import re
import sys

BLOCKED_MODEL = re.compile(r"^(opus|claude-opus-4.*)$", re.IGNORECASE)
BLOCKED_IN_SCRIPT = re.compile(
    r"model\s*:\s*['\"](opus|claude-opus-4[^'\"]*)['\"]", re.IGNORECASE
)


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
    tool = data.get("tool_name", "")
    tool_input = data.get("tool_input") or {}

    if tool == "Agent":
        model = tool_input.get("model")
        if isinstance(model, str) and BLOCKED_MODEL.match(model.strip()):
            deny(
                f"BLOCKED by scripts/hooks/block_opus48.py: model '{model}' is Opus 4.8 "
                "(the bare 'opus' alias still resolves to claude-opus-4-8, verified "
                "2026-07-25). Builder directive 2026-07-24: NO work on Opus 4.8. "
                "Spawn via subagent_type: 'opus5-worker' (pinned claude-opus-5) or "
                "pass model 'claude-opus-5' explicitly."
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
        match = BLOCKED_IN_SCRIPT.search(script)
        if match:
            deny(
                f"BLOCKED by scripts/hooks/block_opus48.py: this workflow script pins "
                f"an agent to Opus 4.8 (found model: '{match.group(1)}'; the bare "
                "'opus' alias resolves to claude-opus-4-8). Builder directive "
                "2026-07-24: NO work on Opus 4.8. Use agentType: 'opus5-worker' or "
                "model: 'claude-opus-5' in the agent() opts instead."
            )

    sys.exit(0)


if __name__ == "__main__":
    main()
