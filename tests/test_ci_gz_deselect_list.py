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

THE SECOND HOLE, found 2026-09-09. Collection errors were the only thing measured,
and that is NOT the whole hazard. Two files -- `test_ekf_tracker.py` (imports
m4_intercept INSIDE a test) and `test_inert_flag_guards.py` (runs it as a
SUBPROCESS) -- collect perfectly well without gz and then FAIL at run time, 14
tests' worth. So the fallback branch collected clean, reported failures, and the
stage went red anyway: the guard passed while the branch it guards was broken.
Measured on a clean clone with no gz bindings. The fix is `gz_unusable_files`
below = collection-erroring UNION run-time-gz-failing, and both direction tests
now use that union.

RESIDUAL, stated rather than papered over: the run-time half runs only the
CANDIDATE files (those whose source names a gz-importing script -- see
`_candidate_files`), because running the entire suite in a subprocess on every
suite run would roughly double its wall time. A file that reaches gz without
naming any of those scripts would still be missed. That is a much smaller hole
than the one it replaces, and it is a NAMED one.

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


# Scripts that do `from gz.transport13 import Node` at module scope. A test file
# that names one of these can reach gz at RUN time (direct import, importlib, or
# a subprocess), which collection-only measurement cannot see.
_GZ_IMPORTING_SCRIPTS = ("m4_intercept", "m4_target_mover", "ekf_tracker")

# A failure is attributed to the missing bindings only on this exact signature.
# Anything else is somebody else's failing test and must NOT be laundered into
# "the deselect list is wrong".
_GZ_FAILURE_RE = re.compile(r"No module named ['\"]gz['\"]?")


def _candidate_files():
    """Repo-relative test files whose SOURCE names a gz-importing script."""
    out = []
    for d in ("tests", os.path.join("flight", "tests")):
        ad = os.path.join(REPO_ROOT, d)
        for name in sorted(os.listdir(ad)):
            if not (name.startswith("test_") and name.endswith(".py")):
                continue
            rel = os.path.join(d, name).replace(os.sep, "/")
            if rel.endswith("test_ci_gz_deselect_list.py"):
                continue                      # never recurse into ourselves
            with open(os.path.join(ad, name), encoding="utf-8") as f:
                src = f.read()
            if any(tok in src for tok in _GZ_IMPORTING_SCRIPTS):
                out.append(rel)
    return out


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


@pytest.fixture(scope="module")
def gz_runtime_failing_files(tmp_path_factory, gz_erroring_files):
    """Repo-relative test files that COLLECT fine without gz but FAIL when run.

    This is the half the original guard could not see. Runs the candidate files
    with gz blocked and attributes a file only when its failure output carries
    the missing-gz signature.

    COLLECTION-ERRORING FILES ARE EXCLUDED, and that is not tidiness -- a
    collection error INTERRUPTS the whole pytest invocation, so leaving one in
    pass 1 means no summary line, no per-file attribution, and a fixture that
    silently measures one file instead of twenty-five. (Observed while building
    this: test_target_orientation.py aborted the batch.) Those files are already
    measured by `gz_erroring_files`; this fixture covers the OTHER mechanism.
    """
    collect_err, _ = gz_erroring_files
    candidates = [c for c in _candidate_files() if c not in collect_err]
    if not candidates:
        pytest.fail("every candidate file fails COLLECTION, or _candidate_files() "
                    "is broken -- either way the run-time half of this guard "
                    "would measure nothing")

    shim_dir = tmp_path_factory.mktemp("gzblock_run")
    (shim_dir / "sitecustomize.py").write_text(_SITECUSTOMIZE)
    env = dict(os.environ)
    env["PYTHONPATH"] = str(shim_dir) + os.pathsep + env.get("PYTHONPATH", "")
    env.pop("PYTHONDONTWRITEBYTECODE", None)

    def _run(paths):
        proc = subprocess.run(
            [sys.executable, "-m", "pytest", *paths, "-q", "--tb=line", "-rfE",
             "-p", "no:cacheprovider"],
            cwd=REPO_ROOT, env=env, capture_output=True, text=True, timeout=600)
        return proc.returncode, proc.stdout + proc.stderr

    # TWO PASSES, so the common (green) case costs ONE pytest startup instead of
    # one per candidate. Pass 1 runs every candidate together and asks only WHICH
    # FILES failed at all. Pass 2 re-runs just those, one at a time, to attribute
    # the cause -- because a file failing for an unrelated reason must NOT be
    # laundered into "the deselect list is wrong". With the list correct, pass 2
    # runs zero subprocesses.
    rc_all, out_all = _run(candidates)
    suspects = sorted({m.group(1) for m in
                       re.finditer(r"^(?:FAILED|ERROR) (\S+?\.py)", out_all, re.M)})
    log = [f"--- pass 1: {len(candidates)} candidate(s), rc={rc_all}, "
           f"suspects={suspects} ---\n{out_all[-2000:]}"]

    failing = set()
    for rel in suspects:
        rc, out = _run([rel])
        log.append(f"--- pass 2: {rel} (rc={rc}) ---\n{out[-1500:]}")
        if rc != 0 and _GZ_FAILURE_RE.search(out):
            failing.add(rel)
    return failing, "\n".join(log)


@pytest.fixture(scope="module")
def gz_unusable_files(gz_erroring_files, gz_runtime_failing_files):
    """Every test file that cannot run without gz, by EITHER mechanism.

    This union -- not the collection-error set alone -- is what ci.yml's fallback
    branch has to deselect for the stage to come back green.
    """
    collect_err, _ = gz_erroring_files
    runtime_fail, _ = gz_runtime_failing_files
    return collect_err | runtime_fail


def test_the_runtime_half_is_not_vacuous(gz_runtime_failing_files):
    """The run-time measurement must actually exercise something.

    It is legitimate for the set to be EMPTY once ci.yml is correct, but it is
    never legitimate for it to be empty because nothing ran. Assert on the
    evidence, not on the count.
    """
    _files, out = gz_runtime_failing_files
    assert "pass 1:" in out and "rc=" in out, (
        f"no candidate file was executed -- nothing was measured.\n{out[-2000:]}")
    # The shim's effect is visible even when every gz-dependent file is correctly
    # deselected, because the CANDIDATE set is not filtered by ci.yml: the files
    # that reach gz are still run here and must still report it. If this ever
    # stops holding, the run-time half has gone blind and must be re-grounded.
    m = re.search(r"(\d+) (?:passed|failed)", out)
    assert m, f"pass 1 produced no pytest summary line at all\n{out[-2000:]}"


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


def test_every_gz_importing_test_file_is_deselected_in_the_ci_fallback(gz_unusable_files):
    missing = sorted(gz_unusable_files - ci_ignore_set())
    assert not missing, (
        f"{len(missing)} test file(s) cannot run without the gz bindings -- by "
        f"collection error OR by run-time failure -- but are NOT in ci.yml's "
        f"--ignore fallback list: {missing}\n"
        f"A collection error aborts the WHOLE stage; a run-time failure turns it "
        f"red. Either way the fallback branch does not work, and it only fires "
        f"when the OSRF apt step is already broken. Add each to the --ignore "
        f"list in .github/workflows/ci.yml.")


def test_the_ci_deselect_list_has_no_stale_entries(gz_unusable_files):
    """The other direction: an --ignore for a file that runs fine silently drops
    that file's coverage from the fallback branch. Entries whose file no longer
    EXISTS are reported separately -- they are dead, not wrong.

    Judged against the UNION, so a file that collects fine but fails at run time
    is correctly treated as legitimately deselected rather than stale."""
    files = gz_unusable_files
    ignored = ci_ignore_set()
    stale, dead = [], []
    for rel in sorted(ignored - files):
        (dead if not os.path.isfile(os.path.join(REPO_ROOT, rel)) else stale).append(rel)
    assert not stale, (
        f"ci.yml deselects {stale}, but those file(s) both COLLECT and RUN fine "
        f"without gz. Remove the --ignore so they run in the fallback branch too.")
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


def test_the_fallback_branch_has_no_surviving_gz_failures(gz_runtime_failing_files):
    """END-TO-END, the run-time half: with ci.yml's deselect list applied, no file
    that still runs may fail for want of the gz bindings.

    Collecting clean is not enough -- that was the 2026-09-09 hole. This asserts
    the property the fallback branch exists to deliver: the stage comes back
    green rather than merely starting.

    DERIVED from pass 1 rather than re-run. Pass 1 already executed every
    collectable candidate with gz blocked and WITHOUT the ignore list, so the
    survivors that would fail are exactly `runtime_failing - ignored`. Re-running
    them in a second subprocess measured the same thing and cost another 11 s of
    every suite run. Failures that are NOT gz-attributable are deliberately out
    of scope here; an unrelated broken test is somebody else's red.
    """
    runtime_failing, out = gz_runtime_failing_files
    survivors = sorted(runtime_failing - ci_ignore_set())
    assert not survivors, (
        f"with ci.yml's deselect list applied, {len(survivors)} file(s) still fail "
        f"for want of the gz bindings: {survivors}\nThe fallback branch would go "
        f"RED, not green. Add them to the --ignore list.\n{out[-2000:]}")
