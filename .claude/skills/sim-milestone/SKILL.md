---
name: sim-milestone
description: The procedure for closing out a project milestone - write a scripted pass/fail check, verify it, commit to git, and update the notes. Use at the end of every milestone (M0-M5).
---
# Closing a milestone

Never call a milestone done by eyeballing it. Every time:

1. Write an automated gate check for the milestone's success criterion (a pytest or a script that exits 0 on pass, non-zero on fail). Put it in `tests/` or `scripts/`.
2. Run it, then hand it to the `verifier` subagent to confirm against the logs (real numbers, e.g. standoff < 0.5 m, closest approach < 1 m).
3. Only if it genuinely passes: `git add -A && git commit -m "Mx: <what landed> (gate passing)"`.
4. Update `NEXT.md` (top of the stack) and `PROGRESS.md` (roll-up), and log any non-trivial choice in `docs/decisions.md`.
5. Tell Emerson in plain English: what passed, the key number, what's next.

If a gate fails twice in a row, stop and surface the blocker - do not stack workarounds.