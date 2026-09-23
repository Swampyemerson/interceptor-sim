#!/usr/bin/env python3
"""Reproducibility-baseline PIN tests for the cue path, written BEFORE the
T19 --cue-source {mock,stereo} change is built (docs/decisions.md ADR-0048,
decision "4. Reproducibility guarantee -- scope + test").

WHAT THIS FILE IS FOR: ADR-0048 SS4 says the mock-branch cue path must stay a
provably byte-identical / structurally-identical sub-case of the new
--cue-source {mock,stereo} flag T19 is about to add. These tests capture
CURRENT (pre-T19) reality as five pinned facts, so that when T19 lands, any
unintended diff to the mock path -- not just the intended "wrap it in a
branch" change -- gets caught by a red test instead of a silent regression.
They pin what exists TODAY on main and therefore MUST PASS right now, before
any T19 code is written. Two of the five ((d) and (e)) pin an ABSENCE (the
flag doesn't exist yet, no gated script mentions it yet) and are explicitly
marked to flip their assertion once T19 lands -- see each docstring.

SS4 sub-item -> test map:
  SS4a s2_cue_mock.py byte-identity      -> test_s2_cue_mock_full_file_hash_pin
  SS4b CueReader class-body zero-diff    -> test_cuereader_class_body_hash_pin
  SS4c mock spawn argv (AST-token, not   -> test_mock_spawn_argv_token_pin
       a raw string match)
  SS4d --cue-source default == "mock"    -> test_cue_source_flag_absent_today
       (not yet built; pins ABSENCE now)    (FLIP-LATER, see docstring)
  SS4e no gated check_*.sh references    -> test_gated_scripts_do_not_reference_cue_source
       --cue-source yet                     (FLIP-LATER, see docstring)

AST, NEVER REGEX/STRING-MATCHING SOURCE (same idiom as
tests/test_honesty_static.py, deliberately NOT the idiom
tests/test_ekf_tracker.py's test_alphabeta_construction_source_is_byte_identical
uses): that older test asserts an exact multi-line string appears verbatim in
the file. That idiom is fine for a leaf statement that truly must never move,
but it is the WRONG tool for SS4c's mock-spawn-argv pin, because ADR-0048's
own design (decision 2/lifecycle + SS4f "keep the mock block textually
untouched-and-moved under a conditional") means T19 is expected to wrap
today's unconditional `cue_cmd = [...]` construction in something like
`if cue_source == "mock":`, which re-indents every line one level deeper.
A raw-string test pinning today's exact leading whitespace would FALSE-FAIL
on that purely-cosmetic reindent even though the mock path is byte-identical
in every way that matters (same tokens, same order, same values) -- exactly
the false-failure trap ADR-0048 SS4c calls out by name. Parsing with `ast` and
comparing structure/tokens (which ast.parse ignores indentation depth for,
as long as the source stays syntactically valid) survives that reindent and
only fires on a REAL change to the argv the mock is launched with.

Run: `.venv/bin/python -m pytest tests/test_cue_source_static.py -v`
"""
import ast
import hashlib
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
M4_PATH = os.path.join(SCRIPTS, "m4_intercept.py")
S2_MOCK_PATH = os.path.join(SCRIPTS, "s2_cue_mock.py")


def _read_text(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def _load_tree(path):
    src = _read_text(path)
    return ast.parse(src, filename=path), src


def _find_function(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name:
            return node
    raise AssertionError(f"function {name!r} not found in {M4_PATH!r}")


def _find_class(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == name:
            return node
    raise AssertionError(f"class {name!r} not found in {M4_PATH!r}")


# ==========================================================================
# SS4a -- s2_cue_mock.py full-file byte-identity pin
# ==========================================================================
# Regenerate ONLY when an intentional mock change is reviewed; this file's
# byte-identity IS the mock-branch reproducibility guarantee (ADR-0048 SS4a).
S2_CUE_MOCK_SHA256 = "c167528fb554ebcf67bb354203b14b309f87c3c11c3d6c905a5d50bd873de109"


def test_s2_cue_mock_full_file_hash_pin():
    """scripts/s2_cue_mock.py's bytes must not change: its RNG stream/
    schedule (seeded `random.Random(args.seed)`, sample/latency timing, CSV
    schema) can't drift if the file that defines them can't change. ADR-0048
    lifecycle (decision 2) has T19's stereo producer (ground_station.py) live
    as a SEPARATE script spawned under --cue-source stereo -- this file is
    never touched to add that branch, so a passing pin here is a real
    guarantee, not a coincidence of "nobody happened to edit it yet"."""
    actual = hashlib.sha256(_read_text_bytes(S2_MOCK_PATH)).hexdigest()
    assert actual == S2_CUE_MOCK_SHA256, (
        f"scripts/s2_cue_mock.py bytes changed (sha256 {actual} != pinned "
        f"{S2_CUE_MOCK_SHA256}). If this is an INTENTIONAL, reviewed mock "
        "change, regenerate the pin (recompute the hash and update "
        "S2_CUE_MOCK_SHA256 with a comment explaining why); if this is "
        "T19 spillover into the mock file itself, that violates ADR-0048 "
        "SS4a/SS4f (mock stays untouched-and-moved, never edited) -- stop "
        "and fix the T19 change instead of the pin."
    )


def _read_text_bytes(path):
    with open(path, "rb") as f:
        return f.read()


# ==========================================================================
# SS4b -- CueReader class-body zero-diff pin
# ==========================================================================
# CueReader parsing must never change regardless of --cue-source (ADR-0048
# SS5.6 #3: "the one thing that must never change"). Hashing the CLASS BODY
# (not the whole file) is deliberate: m4_intercept.py legitimately grows a
# --cue-source flag, an argparse choice, and a branch around the spawn-argv
# construction elsewhere in this same file (SS4c) -- none of that should ever
# touch CueReader's __init__/_run/read/close, which is the sole consumer of
# the UDP wire format on the receiving side and stays IDENTICAL whether the
# sender is the mock or ground_station.py (ADR-0048 decision 1: same wire).
CUEREADER_CLASS_BODY_SHA256 = "07e240cc5a6755f1f41924e3049627a1182e378ff0c80bfb7af3eef570a3f231"


def test_cuereader_class_body_hash_pin():
    tree, src = _load_tree(M4_PATH)
    cls = _find_class(tree, "CueReader")
    seg = ast.get_source_segment(src, cls)
    assert seg is not None, "ast.get_source_segment returned None for CueReader -- unexpected"
    actual = hashlib.sha256(seg.encode("utf-8")).hexdigest()
    assert actual == CUEREADER_CLASS_BODY_SHA256, (
        f"CueReader class body changed (sha256 {actual} != pinned "
        f"{CUEREADER_CLASS_BODY_SHA256}). Per ADR-0048 SS5.6 #3, CueReader's "
        "parsing must never change regardless of --cue-source -- if T19 "
        "needs to touch it, that is itself the finding to escalate, not a "
        "pin to silently update."
    )


# ==========================================================================
# SS4c -- mock-branch spawn argv, pinned at the AST-TOKEN level
# ==========================================================================
# The `cue_cmd = [...]` list literal built (today, unconditionally) right
# before `subprocess.Popen(cue_cmd, ...)` under `if args.handoff:`. Found by
# STRUCTURE, not by the variable name `cue_cmd` (which T19 could legitimately
# rename, e.g. to `mock_cmd`, when it becomes one branch of two): any List
# literal inside run_acquire_and_engage whose elements include a bare
# reference to CUE_MOCK_SCRIPT *and* all three of the string constants
# "--port"/"--seed"/"--latency-s" is accepted as *the* mock spawn-argv list.
REQUIRED_ARGV_MARKERS = {"--port", "--seed", "--latency-s"}

# The pinned ORDERED token sequence for the 14-element literal as it exists
# today (sys.executable, the script path, then 6 "--flag", value pairs).
# Each token is either (a) a string constant's literal value, or (b) a
# normalized dotted/call expression for anything else -- see
# _normalize_argv_element. This is intentionally NOT the CSV of the actual
# runtime values (e.g. not "47800") -- it pins the SHAPE of the expression
# (e.g. "str(s2params['CUE_PORT'])"), which is what actually determines
# reproducibility (the numbers already come from s2params / args and are
# not this test's concern).
MOCK_SPAWN_ARGV_TOKENS = [
    "sys.executable",
    "CUE_MOCK_SCRIPT",
    "--port",
    "str(s2params['CUE_PORT'])",
    "--seed",
    "str(args.cue_seed)",
    "--duration",
    "str(s2params['CUE_MOCK_RUN_DURATION_S'])",
    "--sigma",
    "str(s2params['CUE_MOCK_SIGMA_M'])",
    "--latency-s",
    "str(s2params['CUE_MOCK_LATENCY_S'])",
    "--rate",
    "str(s2params['CUE_MOCK_RATE_HZ'])",
]


def _normalize_argv_element(node, top=False):
    """Normalize one argv-list element to a token that is stable across a
    pure reindent/reflow but changes if the underlying expression changes:
      - a top-level string constant  -> its literal value (e.g. "--port")
      - a bare Name                  -> the name (e.g. "CUE_MOCK_SCRIPT")
      - a.b attribute access         -> "a.b" (recursing on `a`)
      - a[b] subscript                -> "a[b]" (recursing on both sides;
                                          non-top-level string constants are
                                          repr()'d so "CUE_PORT" the dict key
                                          is visually distinct from a literal
                                          argv token)
      - f(a, b) call                  -> "f(a, b)"
    Anything else falls back to a loud ast.dump() so a genuinely new shape
    is never silently mis-tokenized into a false pass.
    """
    if isinstance(node, ast.Constant):
        if top and isinstance(node.value, str):
            return node.value
        return repr(node.value)
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return _normalize_argv_element(node.value) + "." + node.attr
    if isinstance(node, ast.Subscript):
        return _normalize_argv_element(node.value) + "[" + _normalize_argv_element(node.slice) + "]"
    if isinstance(node, ast.Call):
        args = ", ".join(_normalize_argv_element(a) for a in node.args)
        return _normalize_argv_element(node.func) + "(" + args + ")"
    return "UNRECOGNIZED_SHAPE:" + ast.dump(node)


def _find_mock_spawn_argv_list(fn):
    """Returns the ast.List node for the mock spawn-argv literal, found by
    content (see REQUIRED_ARGV_MARKERS above), not by variable name."""
    candidates = []
    for node in ast.walk(fn):
        if not isinstance(node, ast.List):
            continue
        has_script_ref = any(
            isinstance(e, ast.Name) and e.id == "CUE_MOCK_SCRIPT" for e in node.elts
        )
        str_consts = {
            e.value for e in node.elts
            if isinstance(e, ast.Constant) and isinstance(e.value, str)
        }
        if has_script_ref and REQUIRED_ARGV_MARKERS.issubset(str_consts):
            candidates.append(node)
    return candidates


def test_mock_spawn_argv_token_pin():
    """Pins the ORDERED token sequence of the mock spawn-argv list literal.

    COVERAGE -- read before extending: this pins exactly the 14-element
    UNCONDITIONAL list literal (sys.executable, CUE_MOCK_SCRIPT, and 6
    "--flag"/value pairs for port/seed/duration/sigma/latency-s/rate). It
    does NOT cover the conditional dev-flight passthrough immediately after
    it in source (`cue_extra = os.environ.get(CUE_MOCK_EXTRA_ENV, "").strip()`
    / `if cue_extra: cue_cmd.extend(shlex.split(cue_extra))`): that extend()
    call is data-dependent on an environment variable at RUN time, not part
    of the static list literal, so there is nothing fixed in the AST to pin
    -- pinning it would either be vacuous (it's almost always a no-op) or
    require executing the function, which this static/offline test suite
    deliberately does not do. This is a real, documented coverage gap, not a
    silent one: if T19 changes how CUE_MOCK_EXTRA_ENV passthrough works for
    the mock branch specifically, this test will not catch it.
    """
    tree, _src = _load_tree(M4_PATH)
    fn = _find_function(tree, "run_acquire_and_engage")
    candidates = _find_mock_spawn_argv_list(fn)
    assert len(candidates) == 1, (
        f"expected exactly 1 List literal in run_acquire_and_engage containing "
        f"a CUE_MOCK_SCRIPT reference and all of {sorted(REQUIRED_ARGV_MARKERS)} "
        f"as string elements, found {len(candidates)} -- has the mock spawn-argv "
        "construction moved or been restructured in a way this finder can't "
        "recognize?"
    )
    lst = candidates[0]
    tokens = [_normalize_argv_element(e, top=True) for e in lst.elts]
    assert tokens == MOCK_SPAWN_ARGV_TOKENS, (
        "mock spawn-argv token sequence changed:\n"
        f"  pinned:  {MOCK_SPAWN_ARGV_TOKENS}\n"
        f"  actual:  {tokens}\n"
        "If this is the intentional T19 --cue-source={mock,stereo} change and "
        "the mock branch's tokens are UNCHANGED (only reindented/wrapped in "
        "an `if cue_source == \"mock\":`), update MOCK_SPAWN_ARGV_TOKENS' "
        "comment to note the T19 landing and re-pin; if the TOKENS themselves "
        "differ, that is a real behavior change to the mock spawn path and "
        "needs review against ADR-0048 SS4c/SS4f (mock block must stay "
        "textually untouched-and-moved, never edited)."
    )


# ==========================================================================
# SS4d -- --cue-source default == "mock" (FLIP-LATER: the flag does not
# exist yet, so this pins its ABSENCE; once T19 adds it, replace this test's
# body with the real assertion `default == "mock"`, per ADR-0048 SS4d)
# ==========================================================================
def _add_argument_calls(fn):
    return [
        n for n in ast.walk(fn)
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
        and n.func.attr == "add_argument"
    ]


def _flag_strings(call):
    return [
        a.value for a in call.args
        if isinstance(a, ast.Constant) and isinstance(a.value, str)
    ]


def _kwarg(call, name):
    for kw in call.keywords:
        if kw.arg == name:
            return kw.value
    return None


def test_cue_source_default_is_mock():
    """ADR-0048 SS4d (FLIPPED 2026-07-08 when T19 landed): --cue-source now
    exists; its default MUST be the literal "mock" so every gated path that
    omits the flag spawns the mock byte-identically (council decision 2:
    "gated --cue-source {mock,stereo}, default mock"). Choices must be exactly
    {mock, stereo}."""
    tree, _src = _load_tree(M4_PATH)
    fn = _find_function(tree, "parse_args")
    calls = _add_argument_calls(fn)
    cue_src_calls = [c for c in calls if "--cue-source" in _flag_strings(c)]
    assert len(cue_src_calls) == 1, (
        f"expected exactly one --cue-source add_argument call, found "
        f"{len(cue_src_calls)}")
    default_kw = _kwarg(cue_src_calls[0], "default")
    assert isinstance(default_kw, ast.Constant) and default_kw.value == "mock", (
        "--cue-source default must be the literal string 'mock' (ADR-0048 SS4d "
        "byte-identical gated-default guarantee); got "
        f"{ast.dump(default_kw) if default_kw is not None else 'no default kwarg'}")
    choices_kw = _kwarg(cue_src_calls[0], "choices")
    choice_vals = set()
    if isinstance(choices_kw, (ast.List, ast.Tuple)):
        choice_vals = {e.value for e in choices_kw.elts
                       if isinstance(e, ast.Constant)}
    assert choice_vals == {"mock", "stereo"}, (
        f"--cue-source choices must be exactly mock/stereo; got {choice_vals}")


# ==========================================================================
# SS4e -- gated check_*.sh scripts do not reference --cue-source (FLIP-LATER:
# once T19 lands and a gate legitimately needs the flag, this enumeration/
# assertion should be revisited per-script rather than blanket-flipped)
# ==========================================================================
GATED_CHECK_SCRIPTS = [
    "check_m0.sh",
    "check_m1.sh",
    "check_m2*.sh",  # glob: covers check_m2.sh and any check_m2_*.sh variant
    "check_m3.sh",
    "check_m4.sh",
    "check_m5.sh",
    "check_s1.sh",
    "check_s2.sh",
    "check_seeker_v2.sh",
    "check_t16.sh",
]


def _resolve_gated_scripts():
    """Expand each entry in GATED_CHECK_SCRIPTS against scripts/, skipping
    (not failing) any that don't exist on this checkout -- returns
    (found_paths, missing_patterns)."""
    import glob as glob_mod
    found = []
    missing = []
    for pattern in GATED_CHECK_SCRIPTS:
        matches = sorted(glob_mod.glob(os.path.join(SCRIPTS, pattern)))
        if matches:
            found.extend(matches)
        else:
            missing.append(pattern)
    return found, missing


def test_gated_scripts_do_not_reference_cue_source():
    """*** FLIP-LATER ***: ADR-0048 SS4e requires that no gated check_*.sh is
    edited to accommodate the new flag ("no gated script edited to
    accommodate the flag"). Today NONE of them can possibly reference it
    (the flag doesn't exist -- SS4d), so this is a real but weak assertion
    right now. Once T19 lands, re-run this and confirm it STILL passes (that
    is the actual SS4e guarantee: the flag existing elsewhere must not leak
    into these gates); if a gated script is intentionally updated to pass
    --cue-source explicitly, that is a deliberate SS4e exception and this
    list/test should be revisited with that reasoning written down, not
    silently deleted.

    KNOWN COVERAGE NOTE: this enumerates the exact script list given at
    task time. scripts/check_t17.sh also exists on this checkout (a T17
    detector gate) but is intentionally NOT included here -- it was outside
    the delegated scope for this pin, and a T17 worker was concurrently
    active in scripts/seeker/ at the time this test was written. If T19
    needs check_t17.sh covered too, add it to GATED_CHECK_SCRIPTS explicitly
    (with reasoning), rather than assuming this test already covers it.
    """
    found, missing = _resolve_gated_scripts()
    assert found, (
        f"none of {GATED_CHECK_SCRIPTS} matched anything under {SCRIPTS} -- "
        "has the gated-script layout moved?"
    )
    violations = []
    for path in found:
        text = _read_text(path)
        if "--cue-source" in text:
            violations.append(path)
    assert not violations, (
        "the following gated check_*.sh scripts reference --cue-source: "
        f"{violations}. Per ADR-0048 SS4e, no gated script should be edited "
        "to accommodate the new flag -- if this is an intentional, reviewed "
        "exception, document why here; otherwise this is exactly the "
        "gate-creep SS4e exists to prevent."
    )
    # Not an assertion failure -- just keep the skip list visible in -v
    # output so a future reader can see which patterns matched nothing
    # (expected to be empty today; every listed script exists).
    if missing:
        print(f"\n[test_gated_scripts_do_not_reference_cue_source] "
              f"patterns with no match on this checkout (skipped): {missing}")
