#!/usr/bin/env python3
"""Static, sim-free honesty guard for the LIVE markerless seeker chain
(audit finding DEEP-N1; companion to tests/test_honesty_static.py).

WHY THIS FILE EXISTS
--------------------
CLAUDE.md's honesty boundary: `gt_*` (ground truth) is scoring/logging ONLY --
NEVER an input to a live guidance/measurement path. test_honesty_static.py
already pins m4_intercept.py's command chain and detect_track.py. But the live
`--track` measurement ALSO flows through five seeker modules that were UNPINNED:

    markerless_loop.py   -> markerless_detection_loop (the live thread target)
    two_stage_seeker.py  -> TwoStageSeeker.detect / .box_to_detection
    finetuned_seeker.py  -> FinetunedNNSeeker.detect / .box_to_detection
    nn_seeker.py         -> NNSeeker.detect / ._candidates (+ SeekerDetection)
    classical_seeker.py  -> ClassicalSeeker.detect / .propose / .update

Verified manually 2026-07-10: the boundary is currently INTACT (the only gt_
references in these modules are docstrings plus two_stage_seeker._run_eval's
OFFLINE eval scoring). So this guard is future-proofing, not a leak fix: a
`gt_` read added to a live detect()/update() tomorrow must fail CI today.

WHAT IS FLAGGED (all `ast`-based, never regex over source -- docstrings and
comments that MENTION gt_* can never false-positive):
  * ast.Name whose id starts with "gt_" or "ground_truth" (any ctx -- a
    gt_-named variable in live code deserves review even as a Store),
  * ast.Attribute whose attr starts with "gt_" or "ground_truth" -- the
    `tracker.gt_range` / `self.gt_x` attribute form,
  * ast.arg (function parameter) named gt_*/ground_truth* -- the shape a
    "plumb gt into the live signature" regression takes.
  A `ground_truth_*(...)` CALL is covered by the first two shapes (its func
  node is a Name or Attribute starting with "ground_truth").

ALLOWLIST PHILOSOPHY (mirrors test_honesty_static's LOUD allowlist): every
flagged node must sit inside a function explicitly listed in
OFFLINE_ALLOWLIST for its module, each entry carrying a WHY. A brand-new gt_
reference anywhere else fails BY DEFAULT, forcing a human to extend the
allowlist (and say why) rather than silently passing.
test_allowlist_entries_are_real_and_earning keeps the allowlist honest in the
other direction: an entry that names a vanished function, or shields zero
flagged nodes, is stale and fails too.

test_injected_live_gt_reads_are_caught is the calibration: it mutates
THROWAWAY temp copies of the real modules with all three leak shapes
(`meas = tracker.gt_range` in a live detect(), a bare `gt_range` read in a
module that HAS an allowlist entry -- proving the allowlist doesn't bleed
onto live code -- and a `ground_truth_*()` call in a live update()) and
asserts the checker rejects every one. Never touches the real files.

Run directly: `python tests/test_honesty_seekers.py` (exit 0/1, PASS/FAIL per
check; stdlib-only -- no cv2/onnxruntime/sim, everything is source-level AST).
Also pytest-collectible as-is (plain `assert`, no pytest import needed).
"""
import ast
import os
import sys
import tempfile

# Reuse the existing scan's AST plumbing (same directory): _load_tree parses
# and back-links .parent on every node; _ancestor_chain walks a node up to the
# module root. pytest's default prepend import mode and the standalone
# `python tests/...` runner both put tests/ on sys.path; the insert below
# additionally covers `python -c` invocations from elsewhere.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_honesty_static import _load_tree, _ancestor_chain  # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEEKER_DIR = os.path.join(REPO_ROOT, "scripts", "seeker")

# The naming convention the honesty boundary is written in (CLAUDE.md:
# "gt_* (ground truth) is scoring/logging ONLY"). Prefix match, so gt_range,
# gt_frames, ground_truth_world_points all flag; tag_gt / height do not.
GT_PREFIXES = ("gt_", "ground_truth")

# The five modules the live --seeker markerless [--track] measurement path
# flows through (markerless_loop is the thread target m4_intercept swaps in;
# the rest are the seekers/geometry it constructs). detect_track.py is NOT
# here because test_honesty_static.py already pins it (see IMPORT_EXEMPT).
LIVE_SEEKER_MODULES = (
    "two_stage_seeker.py",
    "finetuned_seeker.py",
    "nn_seeker.py",
    "markerless_loop.py",
    "classical_seeker.py",
)

# module -> {dotted function scope: WHY it may touch gt_* (offline only)}.
# A scope entry also covers functions nested inside it. KEEP THIS TIGHT: an
# entry must be an OFFLINE function (eval harness / dataset builder), never
# anything reachable from markerless_detection_loop.
OFFLINE_ALLOWLIST = {
    "two_stage_seeker.py": {
        "_run_eval": (
            "offline --eval harness only: scores proposal/e2e recall against "
            "eval_tag_boxes.json AprilTag boxes (EVAL-ONLY ground truth, per "
            "the module docstring's HONESTY BOUNDARY note) on saved frames "
            "under demo_out/. Unreachable from the live loop: "
            "markerless_loop constructs TwoStageSeeker and calls only "
            "detect()/box_to_detection(); _run_eval is invoked solely by "
            "_main() under the --eval CLI flag."
        ),
    },
    # finetuned_seeker.py / nn_seeker.py / markerless_loop.py /
    # classical_seeker.py: NO entries -- their gt_ mentions are docstring
    # text only, which an AST scan never flags. Adding a gt_ read anywhere
    # in them fails until a human lists the (offline) function here with a
    # reason.
}

# Anti-vacuous pin: the live entry points this guard exists to protect. If a
# rename/restructure moves the live path out from under these names, fail
# loudly instead of silently scanning dead code.
LIVE_ENTRY_POINTS = {
    "two_stage_seeker.py": ("TwoStageSeeker.detect",
                            "TwoStageSeeker.box_to_detection"),
    "finetuned_seeker.py": ("FinetunedNNSeeker.detect",
                            "FinetunedNNSeeker.box_to_detection"),
    "nn_seeker.py": ("NNSeeker.detect", "NNSeeker._candidates"),
    "markerless_loop.py": ("markerless_detection_loop",),
    "classical_seeker.py": ("ClassicalSeeker.detect", "ClassicalSeeker.propose",
                            "ClassicalSeeker.update"),
}

# Local (scripts/seeker) modules a covered module may import WITHOUT being in
# LIVE_SEEKER_MODULES, each with the reason it is guarded elsewhere.
IMPORT_EXEMPT = {
    "detect_track.py": (
        "pinned gt-free by test_honesty_static.py::"
        "test_detect_track_never_reads_ground_truth (plus its cue-latch pins)"
    ),
}


# --------------------------------------------------------------------------
# Core scan
# --------------------------------------------------------------------------

def _module_path(fname):
    return os.path.join(SEEKER_DIR, fname)


def _scope_path(node):
    """Dotted enclosing-scope path of `node` (outermost first), built from
    every FunctionDef/AsyncFunctionDef/ClassDef above it. '' = module level.
    e.g. a node inside FinetunedNNSeeker.detect -> 'FinetunedNNSeeker.detect'."""
    parts = []
    for _, anc in _ancestor_chain(node):
        if isinstance(anc, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            parts.append(anc.name)
    return ".".join(reversed(parts))


def _def_qualname(node):
    """Dotted qualname of a def node itself (its scope path + its own name)."""
    scope = _scope_path(node)
    return f"{scope}.{node.name}" if scope else node.name


def _gt_flagged(node):
    """A short description if `node` is a ground-truth-shaped reference the
    honesty boundary cares about, else None. See the module docstring for the
    three shapes and why calls need no fourth."""
    if isinstance(node, ast.Name) and node.id.startswith(GT_PREFIXES):
        return f"Name {node.id!r} ({type(node.ctx).__name__})"
    if isinstance(node, ast.Attribute) and node.attr.startswith(GT_PREFIXES):
        return f"Attribute .{node.attr}"
    if isinstance(node, ast.arg) and node.arg.startswith(GT_PREFIXES):
        return f"function parameter {node.arg!r}"
    return None


def _is_allowlisted(scope, allow):
    return any(scope == entry or scope.startswith(entry + ".") for entry in allow)


def _check_seeker_module(path, allowlist):
    """Returns a list of violation strings (empty = clean): every
    gt_*/ground_truth* Name, Attribute, or parameter in `path` whose enclosing
    function is not in `allowlist`."""
    tree = _load_tree(path)
    violations = []
    for node in ast.walk(tree):
        desc = _gt_flagged(node)
        if desc is None:
            continue
        scope = _scope_path(node)
        if _is_allowlisted(scope, allowlist):
            continue
        violations.append(
            f"{os.path.basename(path)}:{node.lineno}: {desc} in "
            f"{scope or '<module level>'} -- LIVE seeker code must never "
            "touch ground truth (CLAUDE.md honesty boundary). If this is a "
            "genuinely OFFLINE function (eval harness / dataset builder), "
            "add it to OFFLINE_ALLOWLIST in tests/test_honesty_seekers.py "
            "with a one-line WHY."
        )
    return violations


def _assert_module_gt_free(fname):
    violations = _check_seeker_module(_module_path(fname),
                                      OFFLINE_ALLOWLIST.get(fname, {}))
    assert not violations, (
        f"honesty-boundary violation(s) in {fname}:\n" + "\n".join(violations))


# --------------------------------------------------------------------------
# The parametrized guard: one test per live seeker module (an explicit table
# expansion rather than pytest.mark.parametrize, so this file stays
# standalone-runnable with no pytest import, like test_honesty_static.py).
# --------------------------------------------------------------------------

def test_two_stage_seeker_live_path_is_gt_free():
    _assert_module_gt_free("two_stage_seeker.py")


def test_finetuned_seeker_live_path_is_gt_free():
    _assert_module_gt_free("finetuned_seeker.py")


def test_nn_seeker_live_path_is_gt_free():
    _assert_module_gt_free("nn_seeker.py")


def test_markerless_loop_live_path_is_gt_free():
    _assert_module_gt_free("markerless_loop.py")


def test_classical_seeker_live_path_is_gt_free():
    _assert_module_gt_free("classical_seeker.py")


def test_module_table_matches_expansion():
    """The five test_*_live_path_is_gt_free functions above ARE the expansion
    of LIVE_SEEKER_MODULES; fail if the table gains a module without a
    matching test (or vice versa), so the two can't drift apart."""
    expanded = {name[len("test_"):-len("_live_path_is_gt_free")] + ".py"
                for name in globals()
                if name.startswith("test_") and name.endswith("_live_path_is_gt_free")}
    assert expanded == set(LIVE_SEEKER_MODULES), (
        f"per-module tests {sorted(expanded)} != LIVE_SEEKER_MODULES "
        f"{sorted(LIVE_SEEKER_MODULES)} -- add/remove the matching "
        "test_<module>_live_path_is_gt_free function")


# --------------------------------------------------------------------------
# Allowlist hygiene + structure pins
# --------------------------------------------------------------------------

def test_allowlist_entries_are_real_and_earning():
    """Every OFFLINE_ALLOWLIST entry must (a) carry a non-empty reason,
    (b) name a function that still exists in its module, and (c) actually
    shield at least one flagged node. A stale entry (renamed/removed function,
    or one whose gt_ use is gone) fails, forcing the allowlist to shrink back
    -- the same keep-it-loud discipline as adding to it."""
    for fname, entries in OFFLINE_ALLOWLIST.items():
        assert fname in LIVE_SEEKER_MODULES, (
            f"OFFLINE_ALLOWLIST names {fname!r}, which is not a covered live "
            "module -- allowlist entries only make sense for scanned modules")
        tree = _load_tree(_module_path(fname))
        def_quals = {_def_qualname(n) for n in ast.walk(tree)
                     if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
        flagged_scopes = [_scope_path(n) for n in ast.walk(tree) if _gt_flagged(n)]
        for entry, reason in entries.items():
            assert isinstance(reason, str) and reason.strip(), (
                f"{fname}: allowlist entry {entry!r} has no WHY -- every "
                "entry must justify itself")
            assert entry in def_quals, (
                f"{fname}: allowlist entry {entry!r} names no existing "
                f"function (defs found: {sorted(def_quals)}) -- the function "
                "moved/renamed; update or remove the entry")
            assert any(s == entry or s.startswith(entry + ".")
                       for s in flagged_scopes), (
                f"{fname}: allowlist entry {entry!r} shields zero gt_ "
                "references -- stale, remove it (an over-broad allowlist is "
                "a future hole in the boundary)")


def test_live_entry_points_still_exist():
    """Anti-vacuous pin: the live functions this guard protects must still
    exist under their expected qualnames, so a restructure can't leave this
    file green while the real measurement path lives somewhere unscanned."""
    for fname, quals in LIVE_ENTRY_POINTS.items():
        tree = _load_tree(_module_path(fname))
        def_quals = {_def_qualname(n) for n in ast.walk(tree)
                     if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
        for q in quals:
            assert q in def_quals, (
                f"{fname}: expected live entry point {q!r} not found -- if "
                "the live seeker path was restructured, update "
                "LIVE_ENTRY_POINTS (and re-check OFFLINE_ALLOWLIST) in "
                "tests/test_honesty_seekers.py")


def test_locally_imported_seeker_modules_are_covered():
    """Transitive-coverage tripwire: any scripts/seeker-local module a covered
    module imports (even lazily, inside a function -- ast.walk sees those)
    must itself be in LIVE_SEEKER_MODULES or IMPORT_EXEMPT. Closes the 'a new
    module gets spliced into the live path and nobody pins it' gap."""
    local_py = {f for f in os.listdir(SEEKER_DIR) if f.endswith(".py")}
    covered = set(LIVE_SEEKER_MODULES) | set(IMPORT_EXEMPT)
    for fname in LIVE_SEEKER_MODULES:
        tree = _load_tree(_module_path(fname))
        imported = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported |= {alias.name.split(".")[0] for alias in node.names}
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module.split(".")[0])
        for mod in sorted(imported):
            mod_py = mod + ".py"
            if mod_py not in local_py:
                continue  # stdlib / third-party, not a seeker-dir module
            assert mod_py in covered, (
                f"{fname} imports local seeker module {mod_py}, which is in "
                "neither LIVE_SEEKER_MODULES nor IMPORT_EXEMPT -- a module on "
                "the live measurement path is escaping the honesty scan; add "
                "it (plus entry points / allowlist as needed)")


# --------------------------------------------------------------------------
# Calibration: prove the checker fires on all three leak shapes
# --------------------------------------------------------------------------

# (module, unique anchor line (stripped), injected statement, token the
#  violation must mention, live scope the violation must land in). The
# injection is inserted immediately BEFORE the anchor at the anchor's own
# indentation, so it lands at statement level inside the live function.
_INJECTIONS = (
    # The attribute form, in a live detect() -- the exact hypothetical from
    # audit DEEP-N1 ("meas = tracker.gt_range inside a live detect()").
    ("finetuned_seeker.py",
     "img, r, (pad_l, pad_t) = _letterbox(frame_bgr, self.imgsz)",
     "meas = tracker.gt_range  # INJECTED for honesty-guard calibration -- must never pass",
     ".gt_range",
     "FinetunedNNSeeker.detect"),
    # The bare-name form, in the live detect() of the ONE module that has an
    # allowlist entry -- proves _run_eval's allowance doesn't bleed onto
    # live code in the same file.
    ("two_stage_seeker.py",
     "survivors = [p for p in proposals",
     "rng = gt_range  # INJECTED for honesty-guard calibration -- must never pass",
     "gt_range",
     "TwoStageSeeker.detect"),
    # The ground_truth_*() call form, in a live detect-then-track update().
    ("classical_seeker.py",
     "det = self.detect(frame_bgr, t_mono)",
     "xyz = ground_truth_world_points()  # INJECTED for honesty-guard calibration -- must never pass",
     "ground_truth_world_points",
     "ClassicalSeeker.update"),
)


def test_injected_live_gt_reads_are_caught():
    """Mutate a THROWAWAY temp copy of each targeted module with the leak
    shape and assert _check_seeker_module rejects it, with the violation
    naming both the gt token and the live scope. Never touches real files."""
    for fname, anchor, payload, token, scope in _INJECTIONS:
        path = _module_path(fname)
        with open(path) as f:
            lines = f.readlines()
        hits = [i for i, l in enumerate(lines) if l.strip() == anchor]
        assert len(hits) == 1, (
            f"{fname}: calibration anchor {anchor!r} matched {len(hits)} "
            "line(s), expected exactly 1 -- the source moved; pick a new "
            "unique statement inside the same live function")
        idx = hits[0]
        indent = lines[idx][: len(lines[idx]) - len(lines[idx].lstrip())]
        mutated = lines[:idx] + [f"{indent}{payload}\n"] + lines[idx:]

        with tempfile.TemporaryDirectory() as tmp_dir:
            mutated_path = os.path.join(
                tmp_dir, fname[:-3] + "_MUTATED_for_test.py")
            with open(mutated_path, "w") as f:
                f.writelines(mutated)
            violations = _check_seeker_module(
                mutated_path, OFFLINE_ALLOWLIST.get(fname, {}))

        stmt = payload.split("  #")[0]
        assert violations, (
            f"{fname}: injected `{stmt}` into {scope} passed clean -- the "
            "honesty guard has a gap for this leak shape")
        assert any(token in v and scope in v for v in violations), (
            f"{fname}: guard fired, but not clearly on `{stmt}` in {scope}: "
            f"{violations}")


# --------------------------------------------------------------------------
# Runner (so `python tests/test_honesty_seekers.py` works with no pytest)
# --------------------------------------------------------------------------

def _all_tests():
    return [(name, fn) for name, fn in sorted(globals().items())
            if name.startswith("test_") and callable(fn)]


if __name__ == "__main__":
    failures = 0
    for name, fn in _all_tests():
        try:
            fn()
        except AssertionError as e:
            failures += 1
            print(f"FAIL {name}\n  {e}")
        except Exception as e:  # noqa: BLE001 -- report, don't hide, unexpected errors too
            failures += 1
            print(f"ERROR {name}: {type(e).__name__}: {e}")
        else:
            print(f"PASS {name}")
    total = len(_all_tests())
    print(f"\n{total - failures}/{total} passed")
    sys.exit(0 if failures == 0 else 1)
