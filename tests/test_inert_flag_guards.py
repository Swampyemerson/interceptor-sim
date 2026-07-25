"""REGRESSION PIN: a flag that cannot affect the run must be REFUSED, not logged
as ACTIVE.

THE DEFECT (review-2 Wave-1 row #6, landed 2026-07-25). scripts/m4_intercept.py
computes `lambda_cmd` and printed "[m4] Terminal LOS compensation ACTIVE ..."
on the SHARED path, before branching on --law. But the `--law pip` branch builds
its command from `target_tracker.pos_hat` (corrected off the RAW LOS azimuth) and
consumes lambda_cmd ONLY in its tracker-not-ready fallback. So a PIP calibration
arm run with --terminal-bearing-bias-* / --terminal-los-lag-ms would print that
the compensation was active, carry the flags in its config string, and return
"no effect" -- and get recorded as a FOURTH phantom terminal-compensation NULL
when the knob was never in the loop.

Both halves are pinned here: the parse-time refusal, and the relocation of the
"ACTIVE" print into the only branch that consumes lambda_cmd.

Offline, no sim. Run: .venv/bin/python -m pytest tests/test_inert_flag_guards.py -v
"""
import os
import subprocess
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
M4 = os.path.join(REPO_ROOT, "scripts", "m4_intercept.py")
PY = os.path.join(REPO_ROOT, ".venv", "bin", "python")
if not os.path.exists(PY):
    PY = sys.executable

INERT_UNDER_PIP = [
    ["--terminal-bearing-bias-deg", "15"],
    ["--terminal-bearing-bias-l2r-deg", "11.0"],
    ["--terminal-bearing-bias-r2l-deg", "-17.8"],
    ["--terminal-los-lag-ms", "190"],
]


def _run(argv):
    return subprocess.run([PY, M4] + argv, capture_output=True, text=True,
                          timeout=120, cwd=REPO_ROOT)


@pytest.mark.parametrize("flag", INERT_UNDER_PIP, ids=lambda f: f[0])
def test_pip_refuses_the_terminal_compensations(flag):
    """argparse error -> exit 2, with a message that says WHY (so the operator
    does not just delete the flag and re-fly the same phantom)."""
    r = _run(["--law", "pip"] + flag)
    assert r.returncode == 2, (r.returncode, r.stdout, r.stderr)
    assert "require --law pursuit|pronav" in r.stderr, r.stderr
    assert "SILENTLY INERT" in r.stderr


@pytest.mark.parametrize("flag", INERT_UNDER_PIP, ids=lambda f: f[0])
def test_pronav_and_pursuit_still_accept_them(flag):
    """The guard must be scoped to pip -- pursuit and pronav share the
    compensated (u, p) LOS basis and the knob is genuinely live there.
    (--help exits 0 after parse-time validation would have run.)"""
    for law in ("pronav", "pursuit"):
        r = _run(["--law", law] + flag + ["--help"])
        assert r.returncode == 0, (law, r.stderr)
        assert "require --law pursuit|pronav" not in r.stderr


def test_zero_valued_flags_are_not_refused_under_pip():
    """The defaults are the OFF values (0.0 / None) and must stay compatible
    with --law pip -- refusing those would break every existing pip arm."""
    r = _run(["--law", "pip", "--terminal-bearing-bias-deg", "0",
              "--terminal-los-lag-ms", "0", "--help"])
    assert r.returncode == 0, r.stderr


def test_the_active_print_moved_out_of_the_shared_path():
    """Belt-and-braces (the guard could be relaxed later): the false-affirmative
    log must live inside the branch that actually consumes lambda_cmd."""
    src = open(M4).read()
    active = src.index("Terminal LOS compensation ACTIVE (first ENGAGE")
    pip_branch = src.index('if args.law == "pip":\n                        # Predicted')
    assert active > pip_branch, (
        "the 'compensation ACTIVE' print is back on the shared path, above the "
        "law branch -- it can again claim ACTIVE on a law that ignores it")


def test_help_text_names_the_law_scope():
    r = _run(["--help"])
    assert r.returncode == 0
    assert r.stdout.count("PRONAV/PURSUIT ONLY") >= 4, (
        "all four terminal-compensation flags must say which laws consume them")
