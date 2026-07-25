#!/usr/bin/env python3
"""CI's gz-fallback deselect list must cover EXACTLY the gz-importing test files.

THE HAZARD, observed 2026-07-25. `.github/workflows/ci.yml` installs the Gazebo
python bindings from the OSRF apt repo BEST-EFFORT; if that step fails it falls
back to running pytest with `--ignore=` on the test files that reach
`scripts/m4_intercept.py` (or exec-load `scripts/m4_target_mover.py`) at MODULE
scope -- both do `from gz.transport13 import Node` unconditionally.

That list lived only in YAML prose, and prose rots. On one day THREE new test
files landed unlisted (`test_solve_intercept_time.py`, `test_dash_clock.py`,
`test_terminal_coast_latch.py`), which turns the fallback branch into:

    ERROR tests/test_solve_intercept_time.py
    ModuleNotFoundError: No module named 'gz'
    !!!! Interrupted: 3 errors during collection !!!!

Note the failure MODE: a collection error interrupts the ENTIRE stage, so one
unlisted file does not cost one file's coverage -- it costs all of it. And the
fallback only fires when the apt step is already broken, i.e. exactly when CI
most needs to still work. So the defect would sit dormant until the worst moment.

HOW THIS TEST MEASURES IT -- no heuristic. A first attempt scanned the source for
module-scope references and over-matched by 8 files (a docstring mentioning
"m4_intercept.CueReader" is not an import). This instead reproduces the real
thing: re-collect the whole suite in a SUBPROCESS with `gz` blocked by a meta-path
finder, and read off which files error. That is definitionally the set the
fallback branch must deselect.

Both directions are asserted: a MISSING entry breaks the fallback branch; a STALE
entry silently drops a file's coverage from it.

Run: `.venv/bin/python -m pytest tests/test_ci_gz_deselect_list.py -v`
"""
import os
import re
import subprocess
import sys
import tempfile

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CI_YML = os.path.join(REPO_ROOT, ".github", "workflows", "ci.yml")

# Injected via PYTHONPATH so it runs at interpreter startup, before pytest
# imports anything -- the same moment the real (absent) bindings would fail.
_SITECUSTOMIZE = '''
import sys


class _BlockGz:
    def find_spec(self, name, path=None, target=None):
        if name == "gz" or name.startswith("gz."):
            raise ModuleNotFoundError("No module named %r" % name)
        return None


sys.meta_path.insert(0, _BlockGz())
'''


def ci_ignore_set():
    """The `--ignore=<path>` arguments in ci.yml's stage-1 fallback branch."""
    with open(CI_YML) as f:
        return set(re.findall(r"--ignore=(\S+?\.py)", f.read()))


@pytest.fixture(scope="module")
def gz_erroring_files(tmp_path_factory):
    """Repo-relative test files whose COLLECTION fails when gz is unavailable."""
    shim_dir = tmp_path_factory.mktemp("gzblock")
    (shim_dir / "sitecustomize.py").write_text(_SITECUSTOMIZE)

    env = dict(os.environ)
    env["PYTHONPATH"] = str(shim_dir) + os.pathsep + env.get("PYTHONPATH", "")
    env.pop("PYTHONDONTWRITEBYTECODE", None)

    proc = subprocess.run(
        [sys.executable, "-m", "pytest", "tests/", "flight/tests/",
         "--collect-only", "-q", "-p", "no:cacheprovider"],
        cwd=REPO_ROOT, env=env, capture_output=True, text=True, timeout=300)
    out = proc.stdout + proc.stderr

    # Sanity: the shim must actually have bitten. If gz were importable in the
    # subprocess we would find zero errors and every assertion below would pass
    # while measuring nothing -- the vacuous-verdict shape.
    assert "No module named" in out and "gz" in out, (
        "the gz-blocking shim did not take effect in the subprocess -- this "
        f"test measured nothing.\n{out[-3000:]}")
    return set(re.findall(r"^ERROR (\S+\.py)", out, re.M)), out


def test_the_measurement_is_not_vacuous(gz_erroring_files):
    files, out = gz_erroring_files
    assert files, (
        "collection with gz blocked produced ZERO erroring files, which "
        f"contradicts m4_intercept.py's unconditional gz import.\n{out[-2000:]}")
    assert len(files) >= 5, f"suspiciously few: {sorted(files)}"


def test_ci_deselect_list_is_not_empty():
    assert ci_ignore_set(), (
        "ci.yml's stage-1 fallback carries no --ignore arguments at all -- the "
        "gz-unavailable branch would abort on a collection error")


def test_every_gz_importing_test_file_is_deselected_in_the_ci_fallback(gz_erroring_files):
    files, _out = gz_erroring_files
    missing = sorted(files - ci_ignore_set())
    assert not missing, (
        f"{len(missing)} test file(s) fail COLLECTION without the gz bindings but "
        f"are NOT in ci.yml's --ignore fallback list: {missing}\n"
        f"If the OSRF apt step ever fails, pytest aborts the WHOLE stage with a "
        f"collection error -- not just these files. Add each to the --ignore "
        f"list in .github/workflows/ci.yml.")


def test_the_ci_deselect_list_has_no_stale_entries(gz_erroring_files):
    """The other direction: an --ignore for a file that collects fine silently
    drops that file's coverage from the fallback branch. Entries whose file no
    longer EXISTS are reported separately -- they are dead, not wrong."""
    files, _out = gz_erroring_files
    ignored = ci_ignore_set()
    stale, dead = [], []
    for rel in sorted(ignored - files):
        (dead if not os.path.isfile(os.path.join(REPO_ROOT, rel)) else stale).append(rel)
    assert not stale, (
        f"ci.yml deselects {stale}, but those file(s) collect FINE without gz. "
        f"Remove the --ignore so they run in the fallback branch too.")
    assert not dead, (
        f"ci.yml deselects {dead}, which no longer exist. Remove the stale "
        f"--ignore argument(s).")


def test_the_fallback_branch_actually_collects_clean_once_deselected(gz_erroring_files):
    """END-TO-END: with gz blocked AND ci.yml's deselect list applied, collection
    must succeed. This is the branch itself, exercised -- the list being 'in sync'
    is a means, this is the property that matters."""
    _files, _out = gz_erroring_files          # ordering: measure first, then prove
    with tempfile.TemporaryDirectory() as td:
        with open(os.path.join(td, "sitecustomize.py"), "w") as f:
            f.write(_SITECUSTOMIZE)
        env = dict(os.environ)
        env["PYTHONPATH"] = td + os.pathsep + env.get("PYTHONPATH", "")
        args = [sys.executable, "-m", "pytest", "tests/", "flight/tests/",
                "--collect-only", "-q", "-p", "no:cacheprovider"]
        args += [f"--ignore={p}" for p in sorted(ci_ignore_set())]
        proc = subprocess.run(args, cwd=REPO_ROOT, env=env,
                              capture_output=True, text=True, timeout=300)
    assert proc.returncode == 0, (
        "ci.yml's gz-unavailable fallback branch does NOT collect cleanly:\n"
        + (proc.stdout + proc.stderr)[-3000:])
    assert "error" not in proc.stdout.lower().split("collected")[-1], proc.stdout[-1500:]
