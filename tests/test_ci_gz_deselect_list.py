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

THE THIRD CHAPTER, 2026-09-16 -- THE STUB MOVED TO A CONFTEST (docs/next.md
item 4). The stub meta-path finder that made `m4_intercept` importable without
gz used to be installed at MODULE SCOPE inside `tests/test_rescore_cpa.py`, so
which test files it rescued depended on ALPHABETICAL COLLECTION ORDER. It now
lives in the repo-root `conftest.py`, imported before any collection.

That changes what this file measures, and the change is large: with the stub
installed deterministically, NO test file errors at collection without gz any
more, and the only file that still cannot run is `test_inert_flag_guards.py`,
which launches `scripts/m4_intercept.py` as a SUBPROCESS (a child interpreter
never loads our conftest). Measured on a from-scratch gz-less venv: the fallback
branch went from 884 passed to 954 passed + 10 failed with no deselect list at
all, i.e. exactly one file still needs deselecting.

So "zero collection errors" is now the EXPECTED answer, which would make the old
`>= 5 erroring files` sanity check backwards and every verdict here vacuous. The
non-vacuity guarantee is therefore re-grounded on two independent probes:
`test_the_shim_is_provably_active` (a bare `find_spec('gz')` in the shimmed
subprocess) and `test_the_conftest_stub_is_what_keeps_collection_clean` (the same
collection with `--noconftest`, which must still break ~9 files). Together they
say: the shim really bites, and the conftest really is what neutralises it.

RESIDUAL, stated rather than papered over: the run-time half runs only the
CANDIDATE files (those whose source names a gz-importing script -- see
`_candidate_files`), because running the entire suite in a subprocess on every
suite run would roughly double its wall time. A file that reaches gz without
naming any of those scripts would still be missed. That is a much smaller hole
than the one it replaces, and it is a NAMED one. Second residual: a stub
attribute is an empty class, so a test that asserted on real gz BEHAVIOUR could
in principle now pass vacuously without the bindings. The files involved hold
pure-function unit tests that merely import the module for helpers; conftest.py
names this residual too.

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
# IT MODELS ABSENCE, NOT PROHIBITION (rewritten 2026-09-10, review finding H3).
#
# The previous version inserted a finder at `sys.meta_path[0]` that RAISED
# ModuleNotFoundError for gz. That is strictly stronger than the condition it
# claims to emulate, and the difference was load-bearing: the repo-root
# `conftest.py` handles a missing gz correctly by APPENDING a stub finder to
# `sys.meta_path` (in 2026-09-10 that stub still lived inside
# tests/test_rescore_cpa.py). A raising finder at index 0 pre-empts every later
# finder, so the stub was never reached and the file looked gz-unusable. It was
# therefore listed in ci.yml's --ignore fallback, and its 14 tests were dropped
# from CI for a condition that does not exist. Verified against a genuinely
# gz-less machine, where the same file passes 14/14.
#
# This version instead hides gz from the PATH-BASED finder only -- exactly what
# "the package is not installed" looks like -- and leaves every other finder,
# including one a test installs later, free to behave as it really would.
#
# THE INSTRUMENT IS ALSO NOW CROSS-CHECKED against reality: see
# `test_the_shim_agrees_with_a_genuinely_gz_less_interpreter` below. The
# anti-stale test judges staleness with this same shim, so without that
# cross-check an over-blocking shim can never be reported as the cause of a
# stale entry -- it would mark the file unusable and thereby exonerate itself.
# That is the shared-instrument failure mode docs/error_handling_policy.md warns
# about: a defect in a measurement tool that a paired control cannot see.
_SITECUSTOMIZE = '''
import sys
import importlib.machinery

_PF = importlib.machinery.PathFinder


class _GzHidingPathFinder:
    """Make `gz` invisible to the path-based finder, as if it were absent."""

    @classmethod
    def find_spec(cls, name, path=None, target=None):
        if name == "gz" or name.startswith("gz."):
            return None
        return _PF.find_spec(name, path, target)


sys.meta_path = [_GzHidingPathFinder if m is _PF else m for m in sys.meta_path]
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


def _shim_env(shim_dir):
    """An environment whose interpreter cannot see gz via the path finder."""
    with open(os.path.join(str(shim_dir), "sitecustomize.py"), "w") as f:
        f.write(_SITECUSTOMIZE)
    env = dict(os.environ)
    env["PYTHONPATH"] = str(shim_dir) + os.pathsep + env.get("PYTHONPATH", "")
    env.pop("PYTHONDONTWRITEBYTECODE", None)
    return env


def _gz_visible(env=None):
    """Can a BARE interpreter (no pytest, no conftest) find gz under `env`?"""
    proc = subprocess.run(
        [sys.executable, "-c",
         "import importlib.util as u; print(u.find_spec('gz') is not None)"],
        cwd=REPO_ROOT, env=env, capture_output=True, text=True, timeout=120)
    return proc.stdout.strip().endswith("True")


def ci_ignore_set():
    """The `--ignore=<path>` arguments in ci.yml's stage-1 fallback branch."""
    with open(CI_YML) as f:
        return set(re.findall(r"--ignore=(\S+?\.py)", f.read()))


@pytest.fixture(scope="module")
def gz_erroring_files(tmp_path_factory):
    """Repo-relative test files whose COLLECTION fails when gz is unavailable."""
    env = _shim_env(tmp_path_factory.mktemp("gzblock"))

    proc = subprocess.run(
        [sys.executable, "-m", "pytest", "tests/", "flight/tests/",
         "--collect-only", "-q", "-p", "no:cacheprovider"],
        cwd=REPO_ROOT, env=env, capture_output=True, text=True, timeout=300)
    out = proc.stdout + proc.stderr

    # Sanity: the shim must actually have bitten. If gz were importable in the
    # subprocess we would find zero errors and every assertion below would pass
    # while measuring nothing -- the vacuous-verdict shape.
    #
    # REWRITTEN 2026-09-16: this used to look for "No module named ... gz" in the
    # OUTPUT, which was only a valid proxy while a missing gz still broke
    # collection. Since the stub finder moved to the repo-root conftest.py, clean
    # output is the expected result, and that check would have failed for the
    # right behaviour. The probe below asks the interpreter directly instead --
    # no pytest, no conftest, so nothing can mask it.
    assert not _gz_visible(env), (
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

    env = _shim_env(tmp_path_factory.mktemp("gzblock_run"))

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


def test_the_shim_is_provably_active(tmp_path_factory):
    """NON-VACUITY, probe 1 of 2 (2026-09-16).

    Every collection measurement in this file is taken through the gz-hiding
    shim, and since the stub finder moved to conftest.py the expected result of
    those measurements is ZERO errors -- indistinguishable from "the shim did
    nothing". So ask the interpreter directly, in a subprocess with no pytest and
    no conftest in the way: under the shim, `find_spec('gz')` must come back
    None, and (on this dev machine, where the bindings are really installed)
    without the shim it must come back not-None. If the second half ever stops
    holding here it only means this machine has no gz, which is fine -- it is
    then the real thing rather than an emulation."""
    env = _shim_env(tmp_path_factory.mktemp("gzprobe"))
    assert not _gz_visible(env), (
        "the gz-blocking shim did NOT hide gz from a bare interpreter, so every "
        "collection measured through it is measuring an unblocked environment. "
        "Fix _SITECUSTOMIZE.")
    # The mirror half: on a machine WITH the bindings, the un-shimmed control
    # must see them. Otherwise the probe above proves nothing about the shim.
    if not _gz_visible():
        pytest.skip("gz IS genuinely absent on this interpreter, so the shim has "
                    "nothing to hide -- the real condition, not an emulation")


@pytest.fixture(scope="module")
def gz_erroring_files_noconftest(tmp_path_factory):
    """Collection errors with gz blocked AND the repo-root conftest disabled.

    The control for the conftest's own effect: `--noconftest` is the one switch
    that turns the stub finder off without editing anything.
    """
    env = _shim_env(tmp_path_factory.mktemp("gzblock_noconftest"))
    proc = subprocess.run(
        [sys.executable, "-m", "pytest", "tests/", "flight/tests/",
         "--collect-only", "-q", "-p", "no:cacheprovider", "--noconftest"],
        cwd=REPO_ROOT, env=env, capture_output=True, text=True, timeout=300)
    out = proc.stdout + proc.stderr
    return set(re.findall(r"^ERROR (\S+\.py)", out, re.M)), out


def test_the_conftest_stub_is_what_keeps_collection_clean(
        gz_erroring_files, gz_erroring_files_noconftest):
    """NON-VACUITY, probe 2 of 2 -- AND the effect of conftest.py, observed.

    With gz blocked, the suite now collects CLEAN. That is only meaningful if
    something is actively making it so; otherwise "no errors" could equally mean
    "nothing was measured". Re-run the same collection with `--noconftest`, which
    is the one switch that turns the repo-root stub finder off: it must break a
    pile of files with `No module named 'gz'`.

    This is the fix-is-not-done-until-its-effect-is-observed rule applied to the
    conftest itself (CLAUDE.md): the mutant (no conftest) fails, the real path
    passes."""
    clean, out_clean = gz_erroring_files
    assert not clean, (
        f"collection with gz blocked now errors on {sorted(clean)}. The "
        f"repo-root conftest.py stub finder should make every module importable "
        f"without the bindings; a file listed here either imports gz under "
        f"another root name or fails for an unrelated reason.\n{out_clean[-2000:]}")

    without_conftest, out = gz_erroring_files_noconftest
    assert len(without_conftest) >= 5, (
        f"with gz blocked AND --noconftest, only {sorted(without_conftest)} "
        f"failed to collect. m4_intercept.py imports gz.transport13 "
        f"unconditionally and ~9 test files import it at module scope, so this "
        f"should be a long list. Either the shim stopped working or this guard "
        f"has gone blind.\n{out[-2000:]}")
    assert _GZ_FAILURE_RE.search(out), (
        "the --noconftest collection failed for some reason OTHER than the "
        f"missing gz bindings, so it does not prove what it claims.\n{out[-2000:]}")


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


@pytest.fixture(scope="module")
def gz_is_genuinely_absent():
    """True when this MACHINE really has no gz bindings.

    On such a machine the shim is unnecessary, which makes it an INDEPENDENT
    instrument: the same measurement can be taken with and without the shim and
    the two must agree.

    MEASURED IN A SUBPROCESS, and that is not fussiness (caught 2026-09-16 by a
    run in a from-scratch gz-less venv). This used to call
    `importlib.util.find_spec("gz")` in-process -- but the repo-root conftest.py
    has by then APPENDED a stub finder for exactly that name, so on a genuinely
    gz-less machine find_spec returns a STUB spec and the fixture answered
    "installed". The cross-check below then skipped itself in the one place it
    was designed to run. An availability probe must not run inside the process
    whose availability it fakes.
    """
    return not _gz_visible()


def test_the_shim_agrees_with_a_genuinely_gz_less_interpreter(
        gz_is_genuinely_absent, gz_erroring_files_noconftest, tmp_path_factory):
    """THE INSTRUMENT IS CHECKED AGAINST REALITY (2026-09-10, finding H3).

    Every other test in this file measures "which files need gz" THROUGH the
    shim, and the anti-stale test then judges ci.yml with that same measurement.
    So a shim that over-blocks marks the file unusable and thereby exonerates
    itself -- a defect a paired control structurally cannot see
    (docs/error_handling_policy.md, the shared-instrument rule).

    This is the missing independent instrument. Where gz is genuinely absent --
    CI's fallback branch and every cloud session, i.e. exactly where the deselect
    list matters -- collect the suite with NO shim at all and require the same
    set of collection errors. A disagreement in either direction is a defect in
    the shim, not in ci.yml, and it is reported as such.

    This is what caught the raising `meta_path[0]` shim: it reported
    tests/test_rescore_cpa.py as gz-unusable when in reality it passes 14/14,
    which had cost 86 tests in the fallback branch.

    BOTH SIDES NOW RUN WITH `--noconftest` (2026-09-16). With the repo-root
    conftest's stub finder in play, both sides are legitimately EMPTY, and
    comparing two empty sets is a verdict computed on zero units. Turning the
    stub off on both sides keeps the comparison about the SHIM, which is what
    this test exists to check, and keeps it non-vacuous.
    """
    if not gz_is_genuinely_absent:
        pytest.skip("gz IS installed here, so there is no shim-free control to "
                    "compare against -- run this on a machine without the "
                    "bindings (CI's fallback branch, or any cloud session)")

    with_shim, _out = gz_erroring_files_noconftest

    proc = subprocess.run(
        [sys.executable, "-m", "pytest", "tests/", "flight/tests/",
         "--collect-only", "-q", "-p", "no:cacheprovider", "--noconftest"],
        cwd=REPO_ROOT, capture_output=True, text=True, timeout=300)
    out = proc.stdout + proc.stderr
    without_shim = set(re.findall(r"^ERROR (\S+\.py)", out, re.M))

    # NOT VACUOUS: if the shim-free control finds no errors at all then either gz
    # is importable after all or the command failed to run, and comparing two
    # empty sets would "pass" while measuring nothing.
    assert without_shim, (
        "the shim-free control produced ZERO collection errors on an interpreter "
        "that reports no gz bindings. Either the detection above is wrong or the "
        f"pytest invocation failed; this comparison measured nothing.\n"
        f"{out[-3000:]}")

    over = sorted(with_shim - without_shim)
    under = sorted(without_shim - with_shim)
    assert not over, (
        f"THE SHIM OVER-BLOCKS. It reports {over} as unable to collect without "
        f"gz, but on this genuinely gz-less interpreter they collect fine. The "
        f"shim is stricter than the condition it emulates, so every verdict in "
        f"this file that rests on it -- including which ci.yml --ignore entries "
        f"look justified -- is wrong in the direction of deselecting too much. "
        f"Fix _SITECUSTOMIZE, not ci.yml.")
    assert not under, (
        f"THE SHIM UNDER-BLOCKS. {under} fail to collect on this gz-less "
        f"interpreter but collect fine under the shim, so the shim is hiding a "
        f"real gz dependency and the deselect list measured through it is too "
        f"short. CI's fallback branch would go red. Fix _SITECUSTOMIZE.")


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


def test_the_dependents_no_longer_need_test_rescore_cpa(tmp_path_factory):
    """THE PROPERTY THAT REPLACED THE ORDERING PIN (2026-09-16, next.md item 4).

    Until today `tests/test_rescore_cpa.py` installed the gz/mavsdk stub finder
    at MODULE scope, so four other files -- test_solve_intercept_time.py,
    test_target_orientation.py, test_terminal_coast_latch.py (collection) and
    test_ekf_tracker.py (run time) -- only worked without gz when that file
    happened to be collected FIRST, which is nothing but alphabetical luck.
    Ignoring that one file broke the other four, and the old test in this slot
    PINNED that coupling so it would at least fail loudly.

    The stub now lives in the repo-root conftest.py, so the coupling is gone and
    the assertion inverts: with gz blocked, those four files must collect AND
    pass with test_rescore_cpa.py nowhere in the invocation. Both halves are
    asserted, because collecting clean and then failing at run time is exactly
    the hole this file grew in 2026-09-09.

    Runs everywhere, including the dev machine, because it uses the shim rather
    than requiring a genuinely gz-less interpreter -- which is why its old
    "gz IS installed here" entry could come out of run_tests.sh's ALLOWED_SKIPS.
    """
    DEPENDENTS = [
        "tests/test_ekf_tracker.py",
        "tests/test_solve_intercept_time.py",
        "tests/test_target_orientation.py",
        "tests/test_terminal_coast_latch.py",
    ]
    env = _shim_env(tmp_path_factory.mktemp("gzblock_nostub"))

    # NOT VACUOUS: the shim must really be hiding gz from this environment,
    # otherwise the whole test is a tautology about an unblocked interpreter.
    assert not _gz_visible(env), "the gz-blocking shim did not take effect"

    # (a) the whole suite still collects clean with that file EXCLUDED
    proc = subprocess.run(
        [sys.executable, "-m", "pytest", "tests/", "flight/tests/",
         "--collect-only", "-q", "-p", "no:cacheprovider",
         "--ignore=tests/test_rescore_cpa.py"],
        cwd=REPO_ROOT, env=env, capture_output=True, text=True, timeout=300)
    out = proc.stdout + proc.stderr
    errors = set(re.findall(r"^ERROR (\S+\.py)", out, re.M))
    assert not errors, (
        f"with gz blocked and tests/test_rescore_cpa.py ignored, {sorted(errors)} "
        f"fail to COLLECT. The repo-root conftest.py stub finder is supposed to "
        f"make that file irrelevant to everyone else; if it is back to being "
        f"load-bearing, the ordering coupling has returned.\n{out[-2000:]}")
    for rel in DEPENDENTS:
        assert rel + "::" in out, (
            f"{rel} produced no collected items in the shimmed run, so this "
            f"test proved nothing about it.\n{out[-2000:]}")

    # (b) and they PASS when run entirely on their own, gz blocked. This is the
    # half a collection-only check cannot see (test_ekf_tracker.py imports
    # m4_intercept INSIDE a test).
    proc = subprocess.run(
        [sys.executable, "-m", "pytest", *DEPENDENTS, "-q", "--tb=line",
         "-p", "no:cacheprovider"],
        cwd=REPO_ROOT, env=env, capture_output=True, text=True, timeout=600)
    run_out = proc.stdout + proc.stderr
    assert proc.returncode == 0, (
        f"with gz blocked, the four ex-dependents fail when run WITHOUT "
        f"tests/test_rescore_cpa.py in the invocation:\n{run_out[-3000:]}")
    assert re.search(r"\d+ passed", run_out), (
        f"no tests ran in the dependent-only invocation -- nothing was "
        f"measured.\n{run_out[-2000:]}")
