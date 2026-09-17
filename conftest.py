"""Repo-root pytest conftest -- installs the gz/mavsdk STUB IMPORT FINDER.

WHY THIS FILE EXISTS (docs/next.md item 4, 2026-09-16).

`scripts/m4_intercept.py` does `from gz.transport13 import Node` at module
scope, and `mavsdk` likewise, so roughly a dozen offline unit-test files -- which
only want that module's PURE FUNCTIONS -- could not even be imported on a machine
without the Gazebo python bindings. Those bindings are NOT pip-installable
(requirements.txt says so); they come from the OSRF apt repo, whose CI step is
best-effort. So "no gz here" is a real, routine environment: CI's fallback
branch, a clean clone, any cloud session.

Until today the workaround lived at MODULE SCOPE inside
`tests/test_rescore_cpa.py`: it appended a stub finder for `gz.*`/`mavsdk.*` to
`sys.meta_path`, which is a PROCESS-WIDE side effect, so every test module
collected AFTER it alphabetically inherited a working `m4_intercept` import and
every module collected BEFORE it did not. Three files (test_solve_intercept_time,
test_target_orientation, test_terminal_coast_latch) plus test_ekf_tracker.py at
run time silently depended on that ordering. Ignoring that ONE file in CI
therefore broke FOUR others, with a bare `No module named 'gz'` and no hint why
(that cost 106 tests in the fallback branch before it was diagnosed --
.github/workflows/ci.yml's stage-1 comment carries the history).

A conftest at the REPO ROOT is imported by pytest before ANY test module is
collected, for both `tests/` and `flight/tests/` in one invocation, so the stub
is there for every module regardless of name or collection order. That is the
whole point: DETERMINISTIC, not alphabetical.

THREE PROPERTIES THIS KEEPS FROM THE ORIGINAL, each of which was learned the
hard way (see the deleted comment in tests/test_rescore_cpa.py):

  1. REAL BINDINGS ALWAYS WIN. The finder is only installed for roots that are
     genuinely absent from this interpreter (`find_spec` says None), and it is
     APPENDED to `sys.meta_path`, behind the real PathFinder. On the dev machine,
     where gz and mavsdk are both installed, this file installs NOTHING and
     changes NOTHING.
  2. IT NEVER RAISES. A finder that raises from `sys.meta_path` pre-empts every
     later finder and is strictly stronger than "the package is absent"; the
     previous measuring shim in tests/test_ci_gz_deselect_list.py made exactly
     that mistake and mis-classified a file as gz-unusable.
  3. STUB PACKAGES CARRY `__path__`. The import machinery iterates a parent
     package's `__path__` when importing a submodule; a catch-all `__getattr__`
     that hands back a class instead produces "TypeError: 'type' object is not
     iterable" at collection time.

NAMED RESIDUAL, stated rather than papered over: a stub attribute is an empty
class, so a test that genuinely exercises gz/mavsdk BEHAVIOUR is not rescued by
this -- it will fail or error, which is correct (green must mean it ran). What
the stub buys is import-time satisfaction for pure-function tests. Two things it
deliberately does NOT cover: a test that runs a flight script in a SUBPROCESS
(the child interpreter never loads this conftest -- tests/test_inert_flag_guards.py
is the example, and it stays in ci.yml's --ignore list because of it), and
`pytest` invoked with a bare filename from INSIDE tests/ (pytest then takes
tests/ as rootdir and never reaches this file). Run pytest from the repo root,
as scripts/run_tests.sh and .github/workflows/ci.yml both do.

Guarded end-to-end by tests/test_ci_gz_deselect_list.py.
"""
import importlib.abc
import importlib.util
import sys
import types

# The two import roots the offline suite is allowed to fake. Deliberately a
# closed list: stubbing anything else would hide a real missing dependency.
STUBBABLE_ROOTS = ("gz", "mavsdk")

# Marker attribute, so a second import of this module (or of a copy of it) does
# not stack a second finder onto sys.meta_path.
_MARKER = "_interceptor_sim_gz_stub_finder"


def _root_is_really_installed(root):
    """True when THIS interpreter can really find the top-level package."""
    try:
        return importlib.util.find_spec(root) is not None
    except (ImportError, ValueError, AttributeError):
        # find_spec raises for a half-initialised or shadowed name; treat that
        # as "cannot be relied on" and stub it. Never propagate.
        return False


class _StubLoader(importlib.abc.Loader):
    """Creates an empty package whose every attribute is a throwaway class."""

    def create_module(self, spec):
        mod = types.ModuleType(spec.name)
        mod.__path__ = []                      # property 3 above
        mod.__getattr__ = lambda _n: type("_S", (), {})  # noqa: E731
        return mod

    def exec_module(self, mod):
        pass


class _StubFinder(importlib.abc.MetaPathFinder):
    """Serves `<root>.*` for roots this interpreter genuinely lacks."""

    def __init__(self, roots):
        self.roots = frozenset(roots)
        setattr(self, _MARKER, True)

    def find_spec(self, fullname, path=None, target=None):
        try:
            if fullname.split(".", 1)[0] not in self.roots:
                return None
            return importlib.util.spec_from_loader(
                fullname, _StubLoader(), is_package=True)
        except Exception:                      # property 2 above: never raise
            return None


def install_gz_stub_finder():
    """Append the stub finder for absent roots. Returns the roots stubbed."""
    for finder in sys.meta_path:
        if getattr(finder, _MARKER, False):
            return tuple(sorted(getattr(finder, "roots", ())))
    missing = tuple(r for r in STUBBABLE_ROOTS if not _root_is_really_installed(r))
    if not missing:
        return ()
    sys.meta_path.append(_StubFinder(missing))
    return missing


# Runs at conftest import time -- i.e. before pytest collects a single module.
STUBBED_ROOTS = install_gz_stub_finder()
