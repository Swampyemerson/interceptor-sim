#!/usr/bin/env python3
"""SessionStart hook: warn about remote branches that were never merged.

WHY THIS EXISTS (2026-08-29, and it cost ten days).
    A cloud session did a real piece of work on 2026-08-19 -- the
    pre-publication secrets sweep, a CI-red fix, the README repackage --
    pushed it to `claude/portfolio-data-github-cleanup-g792ae`, and nothing
    ever merged it. `main` sat five commits behind for ten days with a known
    credential fix waiting on the shelf, and its CI stayed red.

    NOBODY NOTICED, and the reason is the interesting part: the PUBLISHED
    DASHBOARD was current while `main` was not. Every surface a human looks
    at said the work had landed. The drift check compares the contract to its
    own rendering, so it is structurally blind to a contract that was never
    committed at all -- it cannot see work that is not in the repo.

    It surfaced only by accident: a dashboard republish hit a version
    conflict, and the 2026-08-19 contract had to be recovered out of the
    published Artifact to work out what `main` was missing.

    This hook makes that failure mode loud at session start, which is the
    only moment it is cheap to fix. Same family as the stranded-worktree
    check the project already learned once ("main sat 26 behind").

DESIGN NOTES
    - NEVER FETCHES. A session-start hook must not block on the network, and
      a stale answer that appears in two seconds beats a fresh one that hangs
      for thirty. It reads refs already on disk, so it reports what the last
      fetch saw. It says so in its own output rather than implying currency.
    - FAILS OPEN. Not a git repo, no remote, git missing, anything unexpected
      -> exit 0 silently. A session-start hook that blocks the session is
      worse than the problem it reports.
    - Reports only branches with commits `main` does not have. A branch that
      is merged, or is a strict ancestor, is not stranded.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys

MAIN = "main"
# Branches that are expected to live on the remote without being merged.
IGNORE_SUFFIXES = ("/HEAD",)
MAX_REPORTED = 8


def git(*args, cwd=None):
    try:
        p = subprocess.run(["git", *args], cwd=cwd, capture_output=True,
                           text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return None
    if p.returncode != 0:
        return None
    return p.stdout.strip()


def main() -> int:
    root = os.environ.get("CLAUDE_PROJECT_DIR") or os.getcwd()
    if git("rev-parse", "--git-dir", cwd=root) is None:
        return 0
    if not git("remote", cwd=root):
        return 0

    listing = git("for-each-ref", "--format=%(refname:short)",
                  "refs/remotes/", cwd=root)
    if not listing:
        return 0

    stranded = []
    for ref in listing.splitlines():
        ref = ref.strip()
        if not ref or ref.endswith(IGNORE_SUFFIXES):
            continue
        if ref.split("/", 1)[-1] == MAIN:
            continue
        # Commits on `ref` that `main` does not contain. Empty = nothing
        # stranded, whatever the branch's age.
        ahead = git("rev-list", "--count", f"{MAIN}..{ref}", cwd=root)
        if not ahead or ahead == "0":
            continue
        when = git("log", "-1", "--date=short", "--format=%ad", ref, cwd=root)
        subject = git("log", "-1", "--format=%s", ref, cwd=root) or ""
        stranded.append((ref, int(ahead), when or "?", subject[:70]))

    if not stranded:
        return 0

    stranded.sort(key=lambda r: r[1], reverse=True)
    lines = [
        f"UNMERGED REMOTE BRANCHES: {len(stranded)} branch(es) carry commits "
        f"`{MAIN}` does not have.",
        "(Read from refs already on disk — no fetch, so this is as of the last "
        "fetch. `git fetch` for a current answer.)",
        "",
    ]
    for ref, ahead, when, subject in stranded[:MAX_REPORTED]:
        lines.append(f"  {ref}  +{ahead} commit(s), last {when}")
        lines.append(f"      {subject}")
    if len(stranded) > MAX_REPORTED:
        lines.append(f"  ... and {len(stranded) - MAX_REPORTED} more")
    lines += [
        "",
        "Work stranded on a branch is invisible to every check this project "
        "runs: the drift check compares the contract to its own rendering, so "
        "it cannot see a contract that was never committed. On 2026-08-19 a "
        "secrets fix sat unmerged for ten days while the published dashboard "
        "showed it as done. Decide per branch — merge it or delete it.",
    ]
    print(json.dumps({
        "hookSpecificOutput": {
            "hookEventName": "SessionStart",
            "additionalContext": "\n".join(lines),
        }
    }))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        # Fail open, always. A session-start hook must never block a session.
        sys.exit(0)
