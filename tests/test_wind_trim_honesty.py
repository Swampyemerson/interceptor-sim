#!/usr/bin/env python3
"""HONESTY-BOUNDARY tests for the wind error-correction path
(scripts/wind_trim.py). These are the tests that must FAIL if a gt_* /
ground-truth input is ever consumed by the correction.

CLAUDE.md standing rule: "gt_* (ground truth) is scoring/logging ONLY;
guidance sees camera + own-state EKF only. Every new guidance path re-earns
the numeric no-cheat audit." The 2026-07-25 builder ruling widened it: the
real test is not WHEN a value is read but WHETHER A REAL SYSTEM COULD GET IT
AT THAT QUALITY. The wind trim is a PRE-FLIGHT constant, so the timing rule
alone would wave it through -- which is precisely the loophole that let "the
launch aim is solved from the target's exactly-known track" pass for months.
So this file checks the SOURCE of the number, not its timing:

  the operator's handheld anemometer reading  -> ALLOWED (a real operator
      really can walk to the launch point and read one; it is graded
      given-noisy in docs/project_state.json assumptions[])
  the simulator's true wind, a wind-driver state, an EKF2 wind estimate,
      any gt_* column                          -> FORBIDDEN, and unreachable
      by construction

FOUR LAYERS, so a leak has to defeat all four:

  L1 IMPORT ALLOWLIST      scripts/wind_trim.py imports stdlib only, so it
                           cannot reach the sim, the wind driver, the
                           anemometer instrument model, MAVSDK, or a scorer.
  L2 IDENTIFIER AUDIT      no name/attribute/key in its code (docstrings and
                           comments excluded, deliberately -- see
                           _audit_identifiers) matches a ground-truth pattern.
  L3 RUNTIME REFUSAL       AnemometerReading.from_mapping / load_reading_json
                           REJECT any mapping carrying a gt_-ish or unknown
                           key, so a leaky producer is a loud error, never a
                           quiet cheat.
  L4 TRIPWIRE OBJECT       the correction is run against a reading proxy that
                           RAISES if any truth-shaped attribute is touched --
                           a positive runtime proof that the code path asks
                           for nothing but the instrument reading.

AND EACH AUDIT IS PROVEN TO FIRE (CLAUDE.md: "a fix is not done until its
EFFECT is observed end-to-end -- the mutant fails, the real path changes"):
test_injected_* mutate a THROWAWAY COPY of wind_trim.py -- adding a gt_ read,
adding an import of the sim-side instrument model -- and assert the audits
reject the mutant. An audit that has never been seen to fail is not evidence.

Run: `.venv/bin/python -m pytest tests/test_wind_trim_honesty.py -v`
"""
import ast
import os
import sys
import tempfile

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
sys.path.insert(0, SCRIPTS)

WIND_TRIM_PATH = os.path.join(SCRIPTS, "wind_trim.py")
ANEMOMETER_SIM_PATH = os.path.join(SCRIPTS, "wind_anemometer_sim.py")

from wind_trim import (  # noqa: E402
    AnemometerReading,
    GroundTruthLeakError,
    SagTable,
    WindTrim,
    apply_wind_trim,
    load_reading_json,
)

# Ground-truth-shaped substrings. Matched case-insensitively against
# identifiers and against strings used as KEYS (subscripts, dict literals,
# getattr) -- the places a value is actually consumed.
GT_PATTERNS = (
    "gt_", "ground_truth", "groundtruth", "truth", "true_wind", "true_dir",
    "sim_wind", "world_wind", "driver_wind", "wind_driver", "actual_wind",
    "ekf2_wind", "windspeed_est", "telemetry", "mavsdk", "cheat",
)

# Everything scripts/wind_trim.py is allowed to import. Anything outside this
# set is either a project module (which could carry truth) or an I/O surface
# that could fetch one. Keep it boring on purpose.
ALLOWED_IMPORT_ROOTS = frozenset({
    "__future__", "json", "math", "dataclasses", "typing", "argparse", "sys",
})


def _read(path):
    with open(path, encoding="utf-8") as fh:
        return fh.read()


# =====================================================================
# L1 -- import allowlist
# =====================================================================
def _audit_imports(path):
    """Returns the sorted list of DISALLOWED import roots in `path`."""
    tree = ast.parse(_read(path), filename=path)
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                roots.add(a.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            if node.level:                      # a relative import IS a project import
                roots.add("." * node.level + (node.module or ""))
            elif node.module:
                roots.add(node.module.split(".")[0])
    return sorted(r for r in roots if r not in ALLOWED_IMPORT_ROOTS)


def test_wind_trim_imports_stdlib_only():
    """The honesty partition is a FILE BOUNDARY, so it is machine-checkable:
    the flight-side correction cannot import the sim, the wind driver, the
    instrument model, MAVSDK or a scorer, because it cannot import anything
    but a short stdlib list."""
    bad = _audit_imports(WIND_TRIM_PATH)
    assert bad == [], (
        f"scripts/wind_trim.py imports {bad}, which is outside the stdlib allowlist "
        f"{sorted(ALLOWED_IMPORT_ROOTS)}. The correction path may consume ONLY the "
        "operator's anemometer reading; any project import is a route for the "
        "simulator's true wind (or a gt_* column) to reach it. If a new stdlib module "
        "is genuinely needed, add it here WITH the reason -- never add a project module."
    )


def test_wind_trim_does_not_import_the_instrument_simulator():
    """The dependency arrow points ONE way: the sim-side instrument model
    (producer) imports the flight-side value objects, never the reverse. This
    is asserted separately from the allowlist so the intent is unmistakable in
    the failure message."""
    src = _read(WIND_TRIM_PATH)
    tree = ast.parse(src, filename=WIND_TRIM_PATH)
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module)
    leaky = sorted(m for m in imported
                   if "anemometer_sim" in m or "wind_driver" in m or "m4_" in m)
    assert leaky == [], (
        f"scripts/wind_trim.py imports {leaky} -- that reverses the producer->consumer "
        "arrow and hands the correction a module that legitimately reads the "
        "simulator's TRUE wind.")
    # and the arrow really does point the other way (the producer imports us)
    prod = ast.parse(_read(ANEMOMETER_SIM_PATH), filename=ANEMOMETER_SIM_PATH)
    prod_imports = {n.module for n in ast.walk(prod)
                    if isinstance(n, ast.ImportFrom) and n.module}
    assert "wind_trim" in prod_imports, (
        "scripts/wind_anemometer_sim.py no longer imports wind_trim -- the producer is "
        "supposed to build the CONSUMER's value object, so the wire form cannot drift.")


# =====================================================================
# L2 -- identifier / key audit
# =====================================================================
# LOUD ALLOWLIST -- the only ground-truth-shaped identifiers permitted in
# scripts/wind_trim.py, each of which IS the guard rather than a consumer:
#   GroundTruthLeakError   the exception the guard raises
#   _check_no_ground_truth the guard function itself
# Anything else fails BY DEFAULT, forcing a human to justify the addition in
# writing rather than silently widening the boundary. Note the entries are
# exact finding tokens, so e.g. a new `_check_no_ground_truth_but_allow_wind`
# would NOT match and would still fail.
GUARD_NAME_ALLOWLIST = frozenset({
    "class:GroundTruthLeakError",
    "name:GroundTruthLeakError",
    "def:_check_no_ground_truth",
    "name:_check_no_ground_truth",
})


def _audit_identifiers(path):
    """Returns a sorted list of "kind:value" findings that match a
    ground-truth pattern, minus GUARD_NAME_ALLOWLIST.

    SCOPE, stated so the gap is documented and not silent: this audits
    IDENTIFIERS (names, attributes, function/class/arg names, keyword names)
    and STRINGS USED AS KEYS (subscript slices, dict-literal keys, getattr
    arguments) -- i.e. the places a value is actually CONSUMED. It
    deliberately does NOT audit docstrings, comments or free-text message
    strings: scripts/wind_trim.py's whole job is to explain the honesty
    boundary, so its prose says "gt_*" and "ground truth" many times, and its
    own guard table _FORBIDDEN_KEY_SUBSTRINGS necessarily contains those very
    substrings as data. Auditing prose would make the guard's own definition
    a violation -- the reductio that tells you prose is the wrong target.
    test_forbidden_key_table_still_covers_the_patterns pins that guard table
    separately, so weakening it is caught even though the audit skips it.
    """
    src = _read(path)
    tree = ast.parse(src, filename=path)

    def bad(text):
        low = str(text).lower()
        return any(p in low for p in GT_PATTERNS)

    findings = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and bad(node.id):
            findings.append(f"name:{node.id}")
        elif isinstance(node, ast.Attribute) and bad(node.attr):
            findings.append(f"attribute:.{node.attr}")
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and bad(node.name):
            findings.append(f"def:{node.name}")
        elif isinstance(node, ast.ClassDef) and bad(node.name):
            findings.append(f"class:{node.name}")
        elif isinstance(node, ast.arg) and bad(node.arg):
            findings.append(f"arg:{node.arg}")
        elif isinstance(node, ast.keyword) and node.arg and bad(node.arg):
            findings.append(f"kwarg:{node.arg}")
        elif isinstance(node, ast.Subscript) and isinstance(node.slice, ast.Constant) \
                and isinstance(node.slice.value, str) and bad(node.slice.value):
            findings.append(f"subscript-key:{node.slice.value}")
        elif isinstance(node, ast.Dict):
            for k in node.keys:
                if isinstance(k, ast.Constant) and isinstance(k.value, str) and bad(k.value):
                    findings.append(f"dict-key:{k.value}")
        elif isinstance(node, ast.Call) and isinstance(node.func, ast.Name) \
                and node.func.id in ("getattr", "setattr"):
            for a in node.args[1:2]:
                if isinstance(a, ast.Constant) and isinstance(a.value, str) and bad(a.value):
                    findings.append(f"getattr-key:{a.value}")
    return sorted(set(findings) - GUARD_NAME_ALLOWLIST)


def test_wind_trim_consumes_no_ground_truth_identifier_or_key():
    findings = _audit_identifiers(WIND_TRIM_PATH)
    assert findings == [], (
        f"scripts/wind_trim.py consumes ground-truth-shaped identifiers/keys: "
        f"{findings}. The pre-flight wind trim may read the OPERATOR'S ANEMOMETER "
        "READING and nothing else -- not the simulator's true wind, not a wind-driver "
        "state, not an EKF2 wind estimate, not any gt_* column. If a real correction "
        "genuinely needs one of these, that is a finding to escalate (it means the "
        "correction is not implementable on the real vehicle), NOT an allowlist to widen."
    )


def test_forbidden_key_table_still_covers_the_patterns():
    """The runtime guard's own data, pinned. The identifier audit skips
    free-text/data strings (see _audit_identifiers' docstring), so this is the
    check that stops someone quietly emptying the guard table."""
    import wind_trim as wt
    table = {s.lower() for s in wt._FORBIDDEN_KEY_SUBSTRINGS}
    for required in ("gt_", "ground_truth", "truth", "true_", "sim_wind", "world_wind"):
        assert required in table, (
            f"scripts/wind_trim.py's _FORBIDDEN_KEY_SUBSTRINGS no longer contains "
            f"{required!r} -- the runtime refusal (L3) has been weakened.")


# =====================================================================
# MUTANTS -- proving L1 and L2 actually fire
# =====================================================================
def _mutate(original_src, needle, replacement):
    assert needle in original_src, f"mutation anchor {needle!r} not found"
    mutated = original_src.replace(needle, replacement, 1)
    fd, path = tempfile.mkstemp(suffix="_mutant.py", text=True)
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        fh.write(mutated)
    return path


def test_injected_ground_truth_read_is_caught():
    """MUTANT: a gt_ read inside the correction. The identifier audit must
    reject the mutated copy -- and must still pass on the real file (proving
    the mutation, not a pre-existing condition, is what fired it)."""
    src = _read(WIND_TRIM_PATH)
    anchor = "    sag = trim.table.sag_mps(headwind)"
    path = _mutate(src, anchor,
                   "    sag = trim.table.sag_mps(headwind) + gt_wind_mps * 0.0\n" + anchor)
    try:
        findings = _audit_identifiers(path)
        assert any("gt_wind_mps" in f for f in findings), (
            f"the identifier audit did NOT catch an injected gt_ read; findings={findings}. "
            "An audit that has never been seen to fail is not evidence.")
    finally:
        os.unlink(path)
    assert _audit_identifiers(WIND_TRIM_PATH) == [], "the real file must still be clean"


def test_injected_true_wind_argument_is_caught():
    """MUTANT: the correction quietly grows a `true_wind_mps` parameter --
    the exact shape a 'just pass the sim's wind in, it's pre-flight anyway'
    change would take."""
    src = _read(WIND_TRIM_PATH)
    anchor = "def wind_components(dash_heading_deg: float,"
    path = _mutate(src, anchor,
                   "def wind_components(true_wind_mps, dash_heading_deg: float,")
    try:
        findings = _audit_identifiers(path)
        assert any("true_wind_mps" in f for f in findings), (
            f"the identifier audit did NOT catch an injected true-wind parameter; "
            f"findings={findings}")
    finally:
        os.unlink(path)


def test_injected_sim_module_import_is_caught():
    """MUTANT: importing the sim-side instrument model (which legitimately
    reads the true wind) from the flight-side correction."""
    src = _read(WIND_TRIM_PATH)
    path = _mutate(src, "import json", "import json\nimport wind_anemometer_sim")
    try:
        bad = _audit_imports(path)
        assert "wind_anemometer_sim" in bad, (
            f"the import allowlist did NOT catch an injected sim import; disallowed={bad}")
    finally:
        os.unlink(path)


# =====================================================================
# L3 -- runtime refusal of a leaky producer
# =====================================================================
def test_reading_mapping_rejects_every_ground_truth_key_shape():
    base = AnemometerReading(4.5, 0.0, 0.3, 15.0, 0.0, "unit test").to_mapping()
    for leaky_key in ("gt_wind_mps", "ground_truth_wind", "true_wind_mps",
                      "sim_wind_e", "world_wind_n", "driver_wind_mps",
                      "wind_mps_actual", "TRUTH_dir"):
        data = dict(base)
        data[leaky_key] = 4.5
        with pytest.raises(GroundTruthLeakError) as e:
            AnemometerReading.from_mapping(data)
        assert leaky_key in str(e.value)


def test_load_reading_json_refuses_a_poisoned_file(tmp_path):
    """The producer->consumer fail-closed guarantee: if the sim-side writer
    ever started emitting the simulator's true wind alongside the reading, the
    CONSUMER refuses the file rather than using it."""
    import json
    good = AnemometerReading(4.5, 270.0, 0.3, 15.0, 0.0, "unit test").to_mapping()
    ok_path = tmp_path / "ok.json"
    ok_path.write_text(json.dumps(good), encoding="utf-8")
    assert load_reading_json(str(ok_path)).speed_mps == 4.5

    poisoned = dict(good, gt_wind_mps=4.5, gt_wind_dir_deg=270.0)
    bad_path = tmp_path / "poisoned.json"
    bad_path.write_text(json.dumps(poisoned), encoding="utf-8")
    with pytest.raises(GroundTruthLeakError):
        load_reading_json(str(bad_path))


# =====================================================================
# L4 -- runtime tripwire: the path asks for nothing but the reading
# =====================================================================
class _TripwireReading:
    """Wraps a real reading and EXPLODES if anything asks it for a
    truth-shaped attribute. Because the wrapped dataclass's real fields are
    served from `_inner`, any attribute the correction asks for that is not a
    legitimate instrument field lands here -- so this is a positive runtime
    proof, not an absence argument."""

    def __init__(self, inner):
        object.__setattr__(self, "_inner", inner)
        object.__setattr__(self, "touched", [])

    def __getattr__(self, name):
        low = name.lower()
        if any(p in low for p in GT_PATTERNS):
            raise AssertionError(
                f"HONESTY VIOLATION: the wind-trim path asked the reading for {name!r}, "
                "a ground-truth-shaped attribute. The correction may consume only what "
                "a real handheld anemometer shows.")
        self.touched.append(name)
        return getattr(self._inner, name)


def test_correction_touches_only_instrument_fields():
    inner = AnemometerReading(4.5, 0.0, 0.3, 15.0, 0.0, "unit test")
    tw = _TripwireReading(inner)
    table = SagTable(points=((0.0, 0.0), (4.5, 0.6), (8.0, 1.8)),
                     provenance="SYNTHETIC unit-test table -- NOT MEASURED")
    res = apply_wind_trim(16.0, 0.0, WindTrim(enabled=True, reading=tw, table=table))
    assert res.corrected_speed_mps == pytest.approx(15.4)
    # Whitelist of what it was allowed to ask for -- an instrument reading and
    # its declared error, nothing else.
    allowed = {"speed_mps", "dir_from_deg", "taken_at_sim_t", "provenance",
               "sigma_speed_mps", "sigma_dir_deg"}
    # NO VACUOUS VERDICT: an empty `touched` would make the subtraction below
    # trivially pass while proving nothing. The path MUST have gone through the
    # tripwire for its "nothing but instrument fields" verdict to mean anything.
    assert {"speed_mps", "dir_from_deg"} <= set(tw.touched), (
        f"the tripwire recorded {tw.touched} -- the correction did not read the "
        "reading through the proxy at all, so this test proves nothing. Fix the test "
        "before trusting its verdict.")
    unexpected = sorted(set(tw.touched) - allowed)
    assert unexpected == [], (
        f"the wind-trim path read {unexpected} off the reading object; the only "
        f"legitimate inputs are {sorted(allowed)} (what an operator's instrument "
        "actually shows, plus its data-sheet sigma).")


def test_tripwire_itself_fires():
    """Proves L4's instrument works (an instrument that cannot fail is not
    evidence): asking the tripwire for a truth field raises."""
    tw = _TripwireReading(AnemometerReading(4.5, 0.0, 0.3, 15.0, 0.0, "unit test"))
    with pytest.raises(AssertionError) as e:
        _ = tw.gt_wind_mps
    assert "HONESTY VIOLATION" in str(e.value)


# =====================================================================
# The limitation, asserted as a fact about the code
# =====================================================================
def test_no_in_flight_wind_correction_exists_in_this_module():
    """design.correction's core finding, pinned: the open-loop dash has NO
    in-flight wind measurement, so there is NO in-flight correction. The trim
    is resolved ONCE, pre-flight, from constants -- nothing here consumes a
    live vehicle state, a tick, a telemetry stream or a filter.

    Mechanised as: no public function takes a per-tick/live-state argument.
    (`now_sim_t` is the staleness clock for a pre-flight reading and is
    explicitly allowed; it is a timestamp, not a wind measurement.)"""
    tree = ast.parse(_read(WIND_TRIM_PATH), filename=WIND_TRIM_PATH)
    live_names = ("state", "telemetry", "tick", "odom", "estimator", "ekf",
                  "wind_est", "airspeed", "pitot", "vehicle")
    offenders = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            for a in node.args.args + node.args.kwonlyargs:
                if any(p in a.arg.lower() for p in live_names):
                    offenders.append(f"{node.name}({a.arg})")
    assert offenders == [], (
        f"scripts/wind_trim.py grew a live-state parameter: {offenders}. The open-loop "
        "dash has no wind sensor and no datalink, so an in-flight wind correction is "
        "not implementable on the real vehicle -- if one appears here, the sim has "
        "started flattering the hardware.")
