#!/usr/bin/env python3
"""Static, sim-free regression test of the honesty boundary in
scripts/m4_intercept.py (CLAUDE.md's "Honesty boundary" standing rule:
"gt_* (ground truth) is scoring/logging ONLY; the cue is structurally
unreadable after handoff; guidance sees camera + own-state EKF only. Every
new guidance path re-earns the numeric no-cheat audit."). scripts/check_s2.sh
already audits this boundary at RUNTIME, against one live flight's CSV. This
test catches the same class of regression by reading the SOURCE -- in
milliseconds, with no PX4/Gazebo boot -- so it can run on every commit, not
just at a gate.

Everything here is `ast`-based, never regex/string-matching Python source:
a renamed variable or a reflowed comment should never flip a real answer.

Three structural invariants, all scoped to `run_acquire_and_engage` (the only
function that ever touches `cue_reader`, calls `correct_cue`, or builds a
velocity command):

  (A) CUE_READER LATCH (ADR-0010 #5): every place `cue_reader` is READ (an
      attribute access, e.g. `.read()` / `.close()` / `.n_received`) sits
      behind a `cue_reader is not None` guard, and the two sites that don't
      ALSO require `args.handoff` in that guard (`.n_received` and
      `.close()`) are independently justified below (CUE_READER_SITE_RULES
      + test_cue_wait_and_dash_phases_are_handoff_gated). Any new
      `cue_reader` read that doesn't fit one of these shapes fails loudly --
      that is exactly the shape a future "coast on the cue after all"
      regression would take.

  (B) NO GROUND TRUTH IN THE COMMAND CHAIN: `run_acquire_and_engage` samples
      ground truth (`ground_truth_world_points`) purely to (1) log it and
      (2) score the running-min miss distance (`min_gt_range_running`) --
      see the module docstring's "CAMERA-ONLY BREAKOFF, GROUND-TRUTH-ONLY
      MISS" section. test_ground_truth_reads_are_allowlisted enforces a LOUD
      allowlist: the only places any `gt_*`-named value may be read are
      those two sites -- a brand new gt_ reference (new variable, new call,
      or an old name used somewhere new) fails BY DEFAULT, forcing a human
      to extend the allowlist (and explain why) rather than silently
      passing. test_ground_truth_never_feeds_command_chain is a
      complementary, more literal check: no assignment to a guidance-command
      variable (cmd, vh0, vh1, v_perp, a_cmd, vc, v_down, yaw_deg, ...) may
      read a `gt_*` name or call a `ground_truth_*` function on its RHS.
      test_injected_ground_truth_leak_is_caught proves both checks actually
      fire, by mutating a throwaway copy of the file with a fake
      `v_perp += gt_range` line and asserting they reject it.

  (C) FUSION CAPSTONE CUE-INTO-EKF BOUNDARY (design D2/D3, ADR-0041): every
      `correct_cue(...)` call site (the mechanism that actually feeds an
      accepted cue datagram into the EKF's covariance-aware state) must sit
      behind the SAME `cue_reader is not None` guard shape (A) requires --
      test_correct_cue_calls_are_cue_reader_guarded.
      test_injected_post_latch_correct_cue_leak_is_caught proves it fires,
      by mutating a throwaway copy with a fake correct_cue(...) call
      injected right after the one-way latch (`cue_reader = None`, outside
      any guard) and asserting it's rejected. (correct_cue also has its own
      RUNTIME belt-and-suspenders in ekf_tracker.py -- it raises and
      increments a counter if ever called post-latch(); this static check
      is the SOURCE-level tripwire that a call site was written unguarded
      in the first place.)

Run directly: `python tests/test_honesty_static.py` (exit 0/1, prints a
PASS/FAIL line per check). Also pytest-collectible as-is: every function
named test_* uses plain `assert`, and pytest rewrites those without needing
`import pytest` in this file.
"""
import ast
import os
import sys
import tempfile

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
M4_PATH = os.path.join(REPO_ROOT, "scripts", "m4_intercept.py")

# Guidance-command chain (module docstring: "the ONLY difference [pursuit vs
# pro-nav] is the lateral velocity term"; cmd is built from these and only
# these each control tick). Traced by hand once from run_acquire_and_engage
# and re-verified by test_ground_truth_reads_are_allowlisted finding nothing
# else gt_-prefixed in the function.
CHAIN_VARS = {
    "cmd", "vh0", "vh1", "v_perp", "a_cmd", "vc", "v_down", "yaw_deg",
    "last_cmd", "frozen_vworld",
}

# Every attribute read of cue_reader (cue_reader.<attr>) must appear in this
# table. needs_handoff=True means the guarding conditional's test must ALSO
# contain `args.handoff` (not just a None-check); False means the None-check
# alone is accepted, WITH THE REASON WRITTEN HERE -- read it before adding a
# new True/False entry.
CUE_READER_SITE_RULES = {
    "read": dict(
        needs_handoff=True,
        reason="pre-handoff cue consumption -- must be args.handoff-gated, "
               "not just non-None (ADR-0010 #5); a None-check alone would "
               "silently start reading the cue again if some future bug "
               "left cue_reader non-None outside --handoff",
    ),
    "n_received": dict(
        needs_handoff=False,
        reason="CUE_WAIT-phase-only read (cue-lock-detection heuristic). "
               "phase can only equal 'CUE_WAIT' under args.handoff -- see "
               "test_cue_wait_and_dash_phases_are_handoff_gated, which "
               "verifies that structurally -- so the bare None-check is "
               "sufficient here.",
    ),
    "close": dict(
        needs_handoff=False,
        reason="close() is idempotent cleanup: either the one-way latch "
               "itself (immediately followed by cue_reader = None, see "
               "test_cue_reader_latch_is_the_only_close_then_null) or the "
               "post-loop defensive close for a run that aborted before "
               "ever reaching HANDOFF. cue_reader can only be non-None if "
               "args.handoff was set at init (see "
               "test_cue_reader_created_only_under_handoff), so the "
               "None-check alone is a sufficient guard for a close().",
    ),
}


# --------------------------------------------------------------------------
# AST plumbing
# --------------------------------------------------------------------------

def _load_tree(path):
    with open(path) as f:
        src = f.read()
    tree = ast.parse(src, filename=path)
    # ast doesn't link parents by default; every check below walks UP from a
    # usage site to find its enclosing if/ifexp, so this is the load-bearing
    # step that makes that possible.
    for node in ast.walk(tree):
        for child in ast.iter_child_nodes(node):
            child.parent = node
    return tree


def _find_function(tree, name):
    # run_acquire_and_engage is `async def` -> ast.AsyncFunctionDef, not
    # ast.FunctionDef; accept either so this doesn't silently stop matching
    # if a plain def ever gets used somewhere else in this file.
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name:
            return node
    raise AssertionError(f"function {name!r} not found (has m4_intercept.py been restructured?)")


def _ancestor_chain(node):
    """Yield (child, parent) walking from `node` up to the module root --
    `child` is always the exact node whose `.parent is parent`."""
    cur = node
    parent = getattr(cur, "parent", None)
    while parent is not None:
        yield cur, parent
        cur = parent
        parent = getattr(cur, "parent", None)


def _branch(child, anc):
    """Which field of the If/IfExp `anc` does `child` sit in?"""
    if child is anc.test:
        return "test"
    body = anc.body if isinstance(anc.body, list) else [anc.body]
    orelse = anc.orelse if isinstance(anc.orelse, list) else [anc.orelse]
    if child in body:
        return "body"
    if child in orelse:
        return "orelse"
    return None


def _is_attr_chain(node, dotted):
    """True if `node` is exactly the 2-part attribute access for `dotted`
    (e.g. "args.handoff" -> Attribute(value=Name('args'), attr='handoff'))."""
    a, b = dotted.split(".")
    return (
        isinstance(node, ast.Attribute) and node.attr == b
        and isinstance(node.value, ast.Name) and node.value.id == a
    )


def _contains_attr_chain(expr, dotted):
    return any(_is_attr_chain(n, dotted) for n in ast.walk(expr))


def _is_name(node, name):
    return isinstance(node, ast.Name) and node.id == name


def _is_none_const(node):
    return isinstance(node, ast.Constant) and node.value is None


def _find_cue_none_compare(test):
    """First `cue_reader is [not] None` Compare found anywhere in `test`."""
    for node in ast.walk(test):
        if isinstance(node, ast.Compare) and len(node.ops) == 1 and isinstance(node.ops[0], (ast.Is, ast.IsNot)):
            left, right = node.left, node.comparators[0]
            if (_is_name(left, "cue_reader") and _is_none_const(right)) or \
               (_is_name(right, "cue_reader") and _is_none_const(left)):
                return node
    return None


def _is_ground_truth_call(func):
    if isinstance(func, ast.Name):
        return func.id.startswith("ground_truth")
    if isinstance(func, ast.Attribute):
        return func.attr.startswith("ground_truth")
    return False


def _is_name_or_attr(node, name):
    return (isinstance(node, ast.Name) and node.id == name) or \
           (isinstance(node, ast.Attribute) and node.attr == name)


def _is_within(node, container):
    if node is container:
        return True
    for _, anc in _ancestor_chain(node):
        if anc is container:
            return True
    return False


# --------------------------------------------------------------------------
# (A) cue_reader latch checks
# --------------------------------------------------------------------------

def _cue_reader_load_usages(fn):
    return [
        n for n in ast.walk(fn)
        if isinstance(n, ast.Name) and n.id == "cue_reader" and isinstance(n.ctx, ast.Load)
    ]


def _find_latch_close_call(fn):
    """The ONE cue_reader.close() call immediately followed (same statement
    body) by `cue_reader = None` -- that pair IS the one-way latch (ADR-0010
    #5, "the only latch site is m4_intercept.py:1795-96"). Returns the Call
    node, or None if no such pair exists."""
    close_calls = [
        n for n in ast.walk(fn)
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
        and n.func.attr == "close" and _is_name(n.func.value, "cue_reader")
    ]
    latches = []
    for call in close_calls:
        expr_stmt = call.parent
        if not isinstance(expr_stmt, ast.Expr):
            continue
        owner = expr_stmt.parent
        body = owner.body if isinstance(getattr(owner, "body", None), list) else []
        if expr_stmt not in body:
            continue
        idx = body.index(expr_stmt)
        if idx + 1 < len(body):
            nxt = body[idx + 1]
            if isinstance(nxt, ast.Assign) and len(nxt.targets) == 1 \
                    and _is_name(nxt.targets[0], "cue_reader") and _is_none_const(nxt.value):
                latches.append(call)
    if len(latches) == 1:
        return latches[0]
    return None


def _check_cue_reader_guards(path):
    """Returns a list of violation strings (empty = clean)."""
    tree = _load_tree(path)
    fn = _find_function(tree, "run_acquire_and_engage")
    violations = []
    latch_call = _find_latch_close_call(fn)

    for name_node in _cue_reader_load_usages(fn):
        parent = name_node.parent

        # Shape 1: operand of `cue_reader is [not] None`. Inherently safe --
        # it can only ever answer "is the channel latched closed", never
        # hand back a value. No further guard required.
        if isinstance(parent, ast.Compare):
            if _find_cue_none_compare(parent) is not None:
                continue
            violations.append(
                f"line {name_node.lineno}: cue_reader used in an unexpected "
                f"Compare shape ({ast.dump(parent)}) -- review needed"
            )
            continue

        # Shape 2: cue_reader.<attr> (a real attribute access). Must be in
        # CUE_READER_SITE_RULES and sit behind a matching guard -- UNLESS it
        # is the designated one-way latch call itself (ADR-0010 #5): that
        # call needs no None-check guard by design, since it runs at most
        # once, unconditionally, at the exact instant HANDOFF triggers (see
        # test_cue_reader_latch_is_the_only_close_then_null, which verifies
        # there is exactly one such call).
        if parent is getattr(latch_call, "func", None):
            continue

        if not isinstance(parent, ast.Attribute):
            violations.append(
                f"line {name_node.lineno}: cue_reader used in an unexpected "
                f"shape ({ast.dump(parent)}) that is neither a None-compare "
                "nor an attribute access -- review needed"
            )
            continue

        attr = parent.attr
        rule = CUE_READER_SITE_RULES.get(attr)
        if rule is None:
            violations.append(
                f"line {name_node.lineno}: cue_reader.{attr} is not in "
                "CUE_READER_SITE_RULES -- new attribute reads must be added "
                "there with a stated reason before they can pass"
            )
            continue

        guard = None
        for child, anc in _ancestor_chain(name_node):
            if not isinstance(anc, (ast.If, ast.IfExp)):
                continue
            branch = _branch(child, anc)
            if branch not in ("body", "orelse"):
                continue
            none_cmp = _find_cue_none_compare(anc.test)
            if none_cmp is None:
                continue
            op_is_not = isinstance(none_cmp.ops[0], ast.IsNot)
            direction_ok = (branch == "body") if op_is_not else (branch == "orelse")
            if direction_ok:
                guard = anc
                break

        if guard is None:
            violations.append(
                f"line {name_node.lineno}: cue_reader.{attr} is read with no "
                "enclosing `cue_reader is [not] None` guard on the reachable "
                "branch -- this is exactly the shape a cue-leak regression "
                "would take"
            )
            continue

        if rule["needs_handoff"] and not _contains_attr_chain(guard.test, "args.handoff"):
            violations.append(
                f"line {name_node.lineno}: cue_reader.{attr} is None-checked "
                "but its guard does not also test args.handoff, and "
                f"CUE_READER_SITE_RULES[{attr!r}] requires it "
                f"({rule['reason']})"
            )

    return violations


def test_cue_reader_reads_are_guarded():
    violations = _check_cue_reader_guards(M4_PATH)
    assert not violations, "cue_reader guard violation(s):\n" + "\n".join(violations)
    # Sanity: make sure this test isn't vacuously passing because the S2
    # handoff code got moved/renamed out from under it.
    tree = _load_tree(M4_PATH)
    fn = _find_function(tree, "run_acquire_and_engage")
    assert len(_cue_reader_load_usages(fn)) >= 5, (
        "expected several cue_reader reads in run_acquire_and_engage -- "
        "did the S2 handoff code move?"
    )


def test_cue_reader_created_only_under_handoff():
    """cue_reader may only ever be assigned a non-None value inside a block
    guarded by args.handoff -- the invariant CUE_READER_SITE_RULES['close']
    and ['n_received'] both lean on to justify skipping the handoff check."""
    tree = _load_tree(M4_PATH)
    fn = _find_function(tree, "run_acquire_and_engage")
    n_none_assigns = 0
    for node in ast.walk(fn):
        if not (isinstance(node, ast.Assign) and len(node.targets) == 1
                and _is_name(node.targets[0], "cue_reader")):
            continue
        if _is_none_const(node.value):
            n_none_assigns += 1
            continue
        # Non-None assignment (the CueReader(...) construction): must sit in
        # an if-body whose test contains args.handoff.
        guarded = False
        for child, anc in _ancestor_chain(node):
            if isinstance(anc, ast.If) and _branch(child, anc) == "body" \
                    and _contains_attr_chain(anc.test, "args.handoff"):
                guarded = True
                break
        assert guarded, (
            f"line {node.lineno}: cue_reader assigned a non-None value "
            "outside an args.handoff-guarded block"
        )
    assert n_none_assigns >= 1, "expected at least one `cue_reader = None` (the init and/or the latch)"


def test_cue_wait_and_dash_phases_are_handoff_gated():
    """Backs CUE_READER_SITE_RULES['n_received']'s reasoning: 'CUE_WAIT' is
    only ever assigned to `phase` inside the args.handoff ternary, and 'DASH'
    is only ever assigned inside a branch keyed on `phase == "CUE_WAIT"` --
    so by induction neither phase is reachable without args.handoff, and the
    n_received read (which only executes in the CUE_WAIT branch) needs no
    separate args.handoff check of its own."""
    tree = _load_tree(M4_PATH)
    fn = _find_function(tree, "run_acquire_and_engage")

    def _phase_string_assigns(literal):
        found = []
        for node in ast.walk(fn):
            # Direct assign: phase = "DASH"
            if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                    and _is_name(node.targets[0], "phase") \
                    and isinstance(node.value, ast.Constant) and node.value.value == literal:
                found.append(node)
            # Ternary assign: phase = "CUE_WAIT" if args.handoff else "ACQUIRE"
            if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                    and _is_name(node.targets[0], "phase") \
                    and isinstance(node.value, ast.IfExp):
                ifexp = node.value
                for branch_val in (ifexp.body, ifexp.orelse):
                    if isinstance(branch_val, ast.Constant) and branch_val.value == literal:
                        found.append((node, ifexp, branch_val))
        return found

    cue_wait_sites = _phase_string_assigns("CUE_WAIT")
    assert len(cue_wait_sites) == 1, f"expected exactly 1 'CUE_WAIT' assignment to phase, found {len(cue_wait_sites)}"
    node, ifexp, branch_val = cue_wait_sites[0]
    assert branch_val is ifexp.body, "'CUE_WAIT' must be the args.handoff-True branch of the ternary, not the else"
    assert _contains_attr_chain(ifexp.test, "args.handoff"), "the phase-init ternary's test must be (or contain) args.handoff"

    dash_sites = _phase_string_assigns("DASH")
    assert len(dash_sites) == 1, f"expected exactly 1 plain 'DASH' assignment to phase, found {len(dash_sites)}"
    dash_assign = dash_sites[0]
    guarded = False
    for child, anc in _ancestor_chain(dash_assign):
        if isinstance(anc, ast.If) and _branch(child, anc) == "body":
            test = anc.test
            if isinstance(test, ast.Compare) and _is_name(test.left, "phase") \
                    and len(test.ops) == 1 and isinstance(test.ops[0], ast.Eq) \
                    and isinstance(test.comparators[0], ast.Constant) and test.comparators[0].value == "CUE_WAIT":
                guarded = True
                break
    assert guarded, "phase = 'DASH' must sit directly inside an `if phase == \"CUE_WAIT\":` branch"


def test_cue_reader_latch_is_the_only_close_then_null():
    """Locates every cue_reader.close() call and asserts EXACTLY ONE of them
    is immediately followed (same statement body) by `cue_reader = None` --
    that pair IS the one-way latch (ADR-0010 #5, "the only latch site is
    m4_intercept.py:1795-96"). A second close-then-null pair, or none at
    all, means the latch moved or was duplicated and needs a human look.
    (_find_latch_close_call is the same helper _check_cue_reader_guards()
    uses to exempt the latch call from needing its own None-check guard --
    this test is what makes that exemption trustworthy.)"""
    tree = _load_tree(M4_PATH)
    fn = _find_function(tree, "run_acquire_and_engage")

    close_calls = [
        n for n in ast.walk(fn)
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
        and n.func.attr == "close" and _is_name(n.func.value, "cue_reader")
    ]
    assert len(close_calls) >= 1, "expected at least one cue_reader.close() call"
    assert _find_latch_close_call(fn) is not None, (
        f"expected exactly 1 of {len(close_calls)} cue_reader.close() call(s) to be "
        "immediately followed by `cue_reader = None` (the one-way latch) -- found 0 or >1"
    )


# --------------------------------------------------------------------------
# (C) fusion capstone: correct_cue() call sites (design D2/D3, ADR-0041)
# --------------------------------------------------------------------------

def _correct_cue_call_sites(fn):
    return [
        n for n in ast.walk(fn)
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
        and n.func.attr == "correct_cue"
    ]


def _check_correct_cue_guards(path):
    """Returns a list of violation strings (empty = clean). The fusion
    capstone (design D2) routes accepted cue datagrams into
    EKFTracker.correct_cue -- the mechanism that actually feeds the cue into
    the (potentially post-handoff-persisting) EKF state. Every call site
    must sit behind the SAME `cue_reader is not None` guard shape the
    cue_reader reads themselves require (reuses _find_cue_none_compare, the
    exact helper _check_cue_reader_guards uses) -- a correct_cue call
    reachable without that guard is exactly the shape a post-handoff cue
    leak into the EKF would take, structurally distinct from (but just as
    dangerous as) a bare cue_reader read."""
    tree = _load_tree(path)
    fn = _find_function(tree, "run_acquire_and_engage")
    violations = []
    for call in _correct_cue_call_sites(fn):
        guard = None
        for child, anc in _ancestor_chain(call):
            if not isinstance(anc, (ast.If, ast.IfExp)):
                continue
            branch = _branch(child, anc)
            if branch not in ("body", "orelse"):
                continue
            none_cmp = _find_cue_none_compare(anc.test)
            if none_cmp is None:
                continue
            op_is_not = isinstance(none_cmp.ops[0], ast.IsNot)
            direction_ok = (branch == "body") if op_is_not else (branch == "orelse")
            if direction_ok:
                guard = anc
                break
        if guard is None:
            violations.append(
                f"line {call.lineno}: correct_cue(...) is called with no "
                "enclosing `cue_reader is not None` guard on the reachable "
                "branch -- this is exactly the shape a post-handoff cue "
                "leak into the EKF would take"
            )
    return violations


def test_correct_cue_calls_are_cue_reader_guarded():
    violations = _check_correct_cue_guards(M4_PATH)
    assert not violations, "correct_cue guard violation(s):\n" + "\n".join(violations)
    # Sanity: at least one real call site exists (the P1 fusion-capstone
    # wiring) so this test isn't vacuously passing because the call got
    # removed or renamed out from under it.
    tree = _load_tree(M4_PATH)
    fn = _find_function(tree, "run_acquire_and_engage")
    assert len(_correct_cue_call_sites(fn)) >= 1, (
        "expected at least one correct_cue(...) call in run_acquire_and_engage "
        "-- did the P1 fusion-capstone cue-routing move?"
    )


def test_injected_post_latch_correct_cue_leak_is_caught():
    """Calibration: prove _check_correct_cue_guards actually fires, by
    mutating a THROWAWAY temp copy of m4_intercept.py with a fake
    correct_cue(...) call injected immediately after the one-way latch
    (`cue_reader = None`) -- outside any `cue_reader is not None` guard,
    structurally identical to a real post-handoff cue leak into the EKF --
    and asserting the checker rejects it. Never touches the real file."""
    with open(M4_PATH) as f:
        lines = f.readlines()
    target = "cue_reader = None  # illegal-state-unrepresentable past this point\n"
    idx = next(i for i, l in enumerate(lines) if l.strip() == target.strip())
    indent = lines[idx][: len(lines[idx]) - len(lines[idx].lstrip())]
    injected = (
        f"{indent}ekf_tracker.correct_cue((1.0, 1.0), None, tick_start)  "
        "# INJECTED for honesty-test calibration -- must never pass\n"
    )
    mutated = lines[: idx + 1] + [injected] + lines[idx + 1:]

    with tempfile.TemporaryDirectory() as tmp_dir:
        mutated_path = os.path.join(tmp_dir, "m4_intercept_MUTATED_for_test.py")
        with open(mutated_path, "w") as f:
            f.writelines(mutated)
        violations = _check_correct_cue_guards(mutated_path)

    assert violations, (
        "expected the injected post-latch correct_cue call to trip the "
        "guard check, but it passed clean -- checker has a gap"
    )
    assert any("correct_cue" in v for v in violations), (
        f"guard check fired, but not clearly on the injected call: {violations}"
    )


# --------------------------------------------------------------------------
# (B) ground-truth boundary checks
# --------------------------------------------------------------------------

def _check_ground_truth_allowlist(path):
    """Returns a list of violation strings (empty = clean). See the module
    docstring's invariant (B): the only legal gt_* reads are the logging
    call and the min_gt_range_running scoring block."""
    tree = _load_tree(path)
    fn = _find_function(tree, "run_acquire_and_engage")

    gt_calls = [n for n in ast.walk(fn) if isinstance(n, ast.Call) and _is_ground_truth_call(n.func)]
    if len(gt_calls) != 1:
        return [f"expected exactly 1 ground_truth_*() call in run_acquire_and_engage, found {len(gt_calls)}"]
    gt_call = gt_calls[0]
    gt_assign = gt_call.parent
    if not (isinstance(gt_assign, ast.Assign) and len(gt_assign.targets) == 1
            and isinstance(gt_assign.targets[0], ast.Tuple)):
        return ["the ground_truth_*() call is not a simple tuple-unpack assignment -- allowlist logic below assumes that shape, needs review"]
    gt_names = {elt.id for elt in gt_assign.targets[0].elts if isinstance(elt, ast.Name)}
    if not gt_names or not all(n.startswith("gt_") for n in gt_names):
        return [f"ground_truth_*() unpack targets {sorted(gt_names)} don't all start with 'gt_' -- update this checker's assumptions"]

    # The scoring block: every Assign/AugAssign to min_gt_range_running,
    # plus its enclosing If (if any) -- that If's test is where gt_range
    # gets legitimately read to decide "is this a new running minimum".
    scoring_nodes = set()
    for node in ast.walk(fn):
        if isinstance(node, (ast.Assign, ast.AugAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            if any(_is_name(t, "min_gt_range_running") for t in targets):
                scoring_nodes.add(node)
                if isinstance(node.parent, ast.If):
                    scoring_nodes.add(node.parent)
    if not scoring_nodes:
        return ["found no assignment to min_gt_range_running -- has the miss-distance scoring block moved?"]

    logging_calls = [n for n in ast.walk(fn) if isinstance(n, ast.Call) and _is_name_or_attr(n.func, "write_row_m4")]
    if len(logging_calls) != 1:
        return [f"expected exactly 1 write_row_m4(...) call in run_acquire_and_engage, found {len(logging_calls)}"]
    logging_call = logging_calls[0]

    violations = []
    for node in ast.walk(fn):
        if not (isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load) and node.id.startswith("gt_")):
            continue
        if node.id not in gt_names:
            violations.append(
                f"line {node.lineno}: unrecognized gt_-prefixed name {node.id!r} "
                f"(not one of {sorted(gt_names)}) -- a new ground-truth value "
                "needs review before it can be allowlisted"
            )
            continue
        if _is_within(node, logging_call) or any(_is_within(node, s) for s in scoring_nodes):
            continue
        violations.append(
            f"line {node.lineno}: {node.id} is read outside the write_row_m4 "
            "logging call and the min_gt_range_running scoring block -- this "
            "is exactly the shape a ground-truth leak into guidance would "
            "take. If this is a genuinely new legitimate site, extend the "
            "allowlist in _check_ground_truth_allowlist() and say why."
        )
    return violations


def _check_ground_truth_chain(path):
    """Returns a list of violation strings (empty = clean): no assignment to
    a CHAIN_VARS name may read a gt_* name or call ground_truth_*() on its
    RHS. More literal (and more shallow -- single assignment hop only) than
    the allowlist check above; kept as a second, independent tripwire with a
    sharper error message for exactly the injected-leak shape."""
    tree = _load_tree(path)
    fn = _find_function(tree, "run_acquire_and_engage")
    violations = []
    for node in ast.walk(fn):
        if not isinstance(node, (ast.Assign, ast.AugAssign)):
            continue
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        hit_names = sorted({t.id for t in targets if isinstance(t, ast.Name)} & CHAIN_VARS)
        if not hit_names:
            continue
        for sub in ast.walk(node.value):
            if isinstance(sub, ast.Name) and sub.id.startswith("gt_"):
                violations.append(
                    f"line {node.lineno}: assignment to guidance-chain "
                    f"variable(s) {hit_names} reads {sub.id!r} on its RHS"
                )
            elif isinstance(sub, ast.Call) and _is_ground_truth_call(sub.func):
                violations.append(
                    f"line {node.lineno}: assignment to guidance-chain "
                    f"variable(s) {hit_names} calls a ground_truth_*() function"
                )
    return violations


def test_ground_truth_reads_are_allowlisted():
    violations = _check_ground_truth_allowlist(M4_PATH)
    assert not violations, "ground-truth allowlist violation(s):\n" + "\n".join(violations)


def test_ground_truth_never_feeds_command_chain():
    violations = _check_ground_truth_chain(M4_PATH)
    assert not violations, "ground-truth-in-command-chain violation(s):\n" + "\n".join(violations)


def test_injected_ground_truth_leak_is_caught():
    """Calibration: prove the two checks above actually fire, by mutating a
    THROWAWAY temp copy of m4_intercept.py with a fake `v_perp += gt_range`
    line (the exact shape of a partial ground-truth leak into pro-nav's
    lateral term) and asserting both checks reject it. Never touches the
    real file."""
    with open(M4_PATH) as f:
        lines = f.readlines()
    target = "v_perp = _clamp(v_perp + a_cmd * dt, -V_PERP_MAX, V_PERP_MAX)\n"
    idx = next(i for i, l in enumerate(lines) if l.strip() == target.strip())
    indent = lines[idx][: len(lines[idx]) - len(lines[idx].lstrip())]
    injected = f"{indent}v_perp += gt_range  # INJECTED for honesty-test calibration -- must never pass\n"
    mutated = lines[: idx + 1] + [injected] + lines[idx + 1 :]

    with tempfile.TemporaryDirectory() as tmp_dir:
        mutated_path = os.path.join(tmp_dir, "m4_intercept_MUTATED_for_test.py")
        with open(mutated_path, "w") as f:
            f.writelines(mutated)

        chain_violations = _check_ground_truth_chain(mutated_path)
        allowlist_violations = _check_ground_truth_allowlist(mutated_path)

    assert chain_violations, "expected the injected `v_perp += gt_range` to trip the chain check, but it passed clean -- checker has a gap"
    assert any("v_perp" in v and "gt_range" in v for v in chain_violations), \
        f"chain check fired, but not clearly on the injected line: {chain_violations}"
    assert allowlist_violations, "expected the injected gt_range read to also trip the allowlist check (new gt_ reference outside the allowlist)"


# --------------------------------------------------------------------------
# Runner (so `python tests/test_honesty_static.py` works with no pytest)
# --------------------------------------------------------------------------

def _all_tests():
    return [(name, fn) for name, fn in sorted(globals().items()) if name.startswith("test_") and callable(fn)]


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
