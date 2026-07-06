---
name: sim-milestone
description: The procedure for closing out a project milestone - write a scripted pass/fail check, verify it, commit to git, and update the notes. Use at the end of every milestone (M0-M5, S1-S4).
---
# Closing a milestone

Never call a milestone done by eyeballing it. Every time:

1. Write an automated gate check for the milestone's success criterion (a pytest or a script that exits 0 on pass, non-zero on fail). Put it in `scripts/` (pattern: `check_mX.sh`). For guidance milestones the gate must include the numeric no-cheat audit (commands trace to `meas_*`, never `gt_*`/post-handoff `ext_*`).
2. Run it AT IDLE MACHINE LOAD (no other sims, batches, or heavy sessions — RTF sag corrupts gate numbers), then hand it to the `verifier` subagent to confirm against the logs (real numbers, e.g. standoff < 0.5 m, closest approach < 1 m). Remember run-to-run terminal-dropout noise is ~1 m: gates that compare configs need paired runs, not single flights.
3. Only if it genuinely passes: commit — staging SPECIFIC paths only (`git add <files changed for this milestone>`), never `git add -A` (background agents may be mid-edit; this swept partial work into commits once — ADR-0011 3rd addendum). Message: `"Mx: <what landed> (gate passing)"`.
4. Update `NEXT.md` (top of the stack) and `PROGRESS.md` (roll-up), and log any non-trivial choice in `docs/decisions.md`.
5. Tell Emerson in plain English: what passed, the key number, what's next.

If a gate fails twice in a row, stop and surface the blocker - do not stack workarounds. (The M4 saga: 2 failed gates → root cause was RTF/wall-clock, not guidance. Read ADR-0009 before touching anything timing-related.)
