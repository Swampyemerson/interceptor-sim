#!/usr/bin/env python3
"""PHANTOM range-plausibility gate: sim-free unit + wiring tests.

THE PROBLEM (ADR-0076 add #5/#6). Flying the quad r2l, the markerless NN
periodically locks a big FALSE box (a phantom) onto the background. A big
box reads as a SHORT implied range (meas_range, from box width) -- e.g.
~1.5 m -- while the TRUE range is 15-20 m. Guidance chases the phantom ->
~7 m miss -> Pk 0/16 that direction (l2r is fine at 81%, so guidance/
geometry are sound; this is purely a perception false-positive).

THE FIX (scripts/m4_intercept.py). Reject a fresh detection whose implied
range grossly disagrees with an EXPECTED range -- the S2 cue's range
pre-handoff (legal: the cue already feeds mid-course), or the alpha-beta
SMOOTHED camera range r_hat post-handoff (camera-only, cue is latched
closed). Rejected -> treated exactly like a no-detection tick (does not
update the filters). Default OFF via MARKERLESS_PHANTOM_RANGE_GATE (unset
or "0") -> byte-identical.

WHAT THESE TESTS PROVE.
  * PHANTOM REJECTED: meas_range 1.5 vs expected 15 (the diagnosed case).
  * REAL DETECTION PASSES: meas_range 14 vs expected 15 (ordinary box-width
    noise on a real target, well inside the gross-outlier bounds).
  * WARMUP NOT GATED: expected_range=None (no cue reading yet AND r_hat not
    initialized) -> never rejects, so early acquisition is never blocked.
  * DEFAULT-OFF ENV WIRING: MARKERLESS_PHANTOM_RANGE_GATE unset/"0" ->
    _env_bool returns False -> the m4 call site's `phantom_gate_on` is False
    -> phantom_range_reject() is structurally short-circuited (AST pin),
    i.e. byte-identical to before this gate existed.
  * HONESTY: the gate's inputs (meas_range, expected_range) are plain
    floats; no gt_*-named value anywhere in phantom_range_reject() or its
    m4 call site (AST-scanned).

Run: `.venv/bin/python -m pytest tests/test_phantom_range_gate.py -v`
"""
import ast
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
M4_PATH = os.path.join(SCRIPTS, "m4_intercept.py")
sys.path.insert(0, SCRIPTS)

from m4_intercept import (                                   # noqa: E402
    phantom_range_reject,
    _env_bool,
    _env_float,
    MARKERLESS_PHANTOM_RANGE_GATE_ENV,
    MARKERLESS_PHANTOM_GATE_LO_ENV,
    MARKERLESS_PHANTOM_GATE_HI_ENV,
    PHANTOM_GATE_LO_DEFAULT,
    PHANTOM_GATE_HI_DEFAULT,
)

GATE_LO = PHANTOM_GATE_LO_DEFAULT   # 0.35
GATE_HI = PHANTOM_GATE_HI_DEFAULT   # 2.8


# ---------------------------------------------------------------- (a) phantom rejected
def test_phantom_rejected_diagnosed_case():
    # The exact ADR-0076 add #5/#6 diagnosis: meas_range 1.5 m, true/expected
    # range ~15 m -> ratio 0.10, well under GATE_LO (0.35) -> REJECTED.
    assert phantom_range_reject(1.5, 15.0, GATE_LO, GATE_HI) is True


def test_phantom_rejected_gross_over_range_too():
    # Symmetric gross-outlier case: implied range grossly LARGER than expected
    # (e.g. a distant false lock while the real target is close) is also caught.
    assert phantom_range_reject(50.0, 15.0, GATE_LO, GATE_HI) is True


# ---------------------------------------------------------------- (b) real detection passes
def test_real_detection_passes():
    # Ordinary box-width noise: 14 m measured vs 15 m expected (ratio 0.93) --
    # comfortably inside [0.35, 2.8] -> NOT rejected.
    assert phantom_range_reject(14.0, 15.0, GATE_LO, GATE_HI) is False


def test_real_detection_passes_at_gate_boundaries():
    # Just inside both bounds.
    assert phantom_range_reject(0.36 * 15.0, 15.0, GATE_LO, GATE_HI) is False
    assert phantom_range_reject(2.79 * 15.0, 15.0, GATE_LO, GATE_HI) is False
    # Just outside both bounds.
    assert phantom_range_reject(0.34 * 15.0, 15.0, GATE_LO, GATE_HI) is True
    assert phantom_range_reject(2.81 * 15.0, 15.0, GATE_LO, GATE_HI) is True


# ---------------------------------------------------------------- (c) warmup: not gated
def test_warmup_no_expected_never_gates():
    # No cue reading yet AND r_hat not initialized -> expected_range is None
    # -> must NEVER reject, however extreme meas_range is, so a not-yet-
    # converged estimate can never block initial acquisition.
    assert phantom_range_reject(1.5, None, GATE_LO, GATE_HI) is False
    assert phantom_range_reject(500.0, None, GATE_LO, GATE_HI) is False


def test_no_measurement_never_gates():
    # meas_range=None (no detection to judge) -> never rejects.
    assert phantom_range_reject(None, 15.0, GATE_LO, GATE_HI) is False


def test_degenerate_expected_never_gates():
    # expected_range <= 0 is degenerate (e.g. own position exactly on the cue
    # point) -> must not reject (would be a divide-by-zero-flavored false
    # positive otherwise).
    assert phantom_range_reject(5.0, 0.0, GATE_LO, GATE_HI) is False
    assert phantom_range_reject(5.0, -1.0, GATE_LO, GATE_HI) is False


# ---------------------------------------------------------------- (d) default-OFF env wiring
def _clear_env(monkeypatch):
    for name in (MARKERLESS_PHANTOM_RANGE_GATE_ENV, MARKERLESS_PHANTOM_GATE_LO_ENV,
                 MARKERLESS_PHANTOM_GATE_HI_ENV):
        monkeypatch.delenv(name, raising=False)


def test_env_gate_default_off_when_unset(monkeypatch):
    _clear_env(monkeypatch)
    assert _env_bool(MARKERLESS_PHANTOM_RANGE_GATE_ENV, False) is False


def test_env_gate_off_when_explicit_zero(monkeypatch):
    _clear_env(monkeypatch)
    monkeypatch.setenv(MARKERLESS_PHANTOM_RANGE_GATE_ENV, "0")
    assert _env_bool(MARKERLESS_PHANTOM_RANGE_GATE_ENV, False) is False


def test_env_gate_on_when_explicit_one(monkeypatch):
    _clear_env(monkeypatch)
    monkeypatch.setenv(MARKERLESS_PHANTOM_RANGE_GATE_ENV, "1")
    assert _env_bool(MARKERLESS_PHANTOM_RANGE_GATE_ENV, False) is True


def test_env_gate_bounds_default_and_tunable(monkeypatch):
    _clear_env(monkeypatch)
    assert _env_float(MARKERLESS_PHANTOM_GATE_LO_ENV, PHANTOM_GATE_LO_DEFAULT) == 0.35
    assert _env_float(MARKERLESS_PHANTOM_GATE_HI_ENV, PHANTOM_GATE_HI_DEFAULT) == 2.8
    monkeypatch.setenv(MARKERLESS_PHANTOM_GATE_LO_ENV, "0.5")
    monkeypatch.setenv(MARKERLESS_PHANTOM_GATE_HI_ENV, "2.0")
    assert _env_float(MARKERLESS_PHANTOM_GATE_LO_ENV, PHANTOM_GATE_LO_DEFAULT) == 0.5
    assert _env_float(MARKERLESS_PHANTOM_GATE_HI_ENV, PHANTOM_GATE_HI_DEFAULT) == 2.0


# ---------------------------------------------------------------- (d) structural byte-identity pin
def _m4_src():
    with open(M4_PATH, encoding="utf-8") as f:
        return f.read()


def _m4_tree():
    return ast.parse(_m4_src(), filename=M4_PATH)


def test_m4_call_site_gated_on_phantom_gate_on_flag():
    # The per-tick call site must be `if fresh and phantom_gate_on and ...:` --
    # a BoolOp(And) whose operands include the Name `phantom_gate_on`. Since
    # `phantom_gate_on` is read ONCE per run from the default-OFF env var, this
    # structurally guarantees the whole gate body is skipped (byte-identical
    # output) whenever MARKERLESS_PHANTOM_RANGE_GATE is unset/"0".
    found = False
    for node in ast.walk(_m4_tree()):
        if isinstance(node, ast.If) and isinstance(node.test, ast.BoolOp) \
                and isinstance(node.test.op, ast.And):
            names = {n.id for n in node.test.values if isinstance(n, ast.Name)}
            if "phantom_gate_on" in names and "fresh" in names:
                found = True
                break
    assert found, (
        "no `if fresh and phantom_gate_on and ...:` call site found -- the "
        "phantom-range gate must be structurally short-circuited by the "
        "default-OFF phantom_gate_on flag")


def test_m4_phantom_gate_on_sourced_from_default_off_env_helper():
    # phantom_gate_on = _env_bool(MARKERLESS_PHANTOM_RANGE_GATE_ENV, False)
    tree = _m4_tree()
    assigns = [
        n for n in ast.walk(tree)
        if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "phantom_gate_on" for t in n.targets)
    ]
    assert assigns, "no assignment to phantom_gate_on found"
    v = assigns[-1].value
    assert (isinstance(v, ast.Call) and isinstance(v.func, ast.Name)
            and v.func.id == "_env_bool"), (
        "phantom_gate_on must be sourced from _env_bool(...)")
    assert len(v.args) >= 2 and isinstance(v.args[1], ast.Constant) \
        and v.args[1].value is False, (
        "_env_bool's default for phantom_gate_on must be False (default-OFF)")


def test_m4_rejected_tick_does_not_update_filters():
    # Structural pin mirroring the ADR-0056/0057 gates: on rejection, BOTH
    # `fresh` and `detected` must be set False (so the downstream `if fresh:`
    # filter-correct block -- range_filter.correct / lambda_filter.correct /
    # target_tracker.correct / fused.update_camera -- never runs on a
    # rejected tick).
    tree = _m4_tree()
    # Find the `if phantom_range_reject(...):` block and check its body sets
    # both names False.
    call_ifs = [
        n for n in ast.walk(tree)
        if isinstance(n, ast.If) and isinstance(n.test, ast.Call)
        and isinstance(n.test.func, ast.Name) and n.test.func.id == "phantom_range_reject"
    ]
    assert call_ifs, "no `if phantom_range_reject(...):` block found"
    body = call_ifs[0].body
    set_false = set()
    for stmt in body:
        if isinstance(stmt, ast.Assign) and isinstance(stmt.value, ast.Constant) \
                and stmt.value.value is False:
            for t in stmt.targets:
                if isinstance(t, ast.Name):
                    set_false.add(t.id)
    assert {"fresh", "detected"} <= set_false, (
        f"phantom_range_reject rejection branch must set fresh=False and "
        f"detected=False, found {set_false}")


def test_m4_counter_reported_in_summary():
    # phantom_range_rejects must reach the return dict AND the printed summary
    # line, same as terminal_rejects/coast_gate_rejects/handoff_cue_rejects.
    src = _m4_src()
    assert '"phantom_range_rejects": phantom_range_rejects' in src
    assert "phantom_range_rejects={result.get('phantom_range_rejects', 0)}" in src


# ---------------------------------------------------------------- honesty
def test_phantom_range_reject_is_gt_free():
    # AST-scan phantom_range_reject()'s own body: no gt_*-named reference.
    tree = _m4_tree()
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == "phantom_range_reject")
    names = {a.attr for a in ast.walk(fn) if isinstance(a, ast.Attribute)}
    names |= {n.id for n in ast.walk(fn) if isinstance(n, ast.Name)}
    gt_names = {n for n in names if n.startswith("gt_")}
    assert not gt_names, f"phantom_range_reject must be gt_*-free, found {gt_names}"


def test_m4_call_site_expected_range_is_gt_free():
    # AST-scan the small block that computes `phantom_expected` (the call
    # site's own logic, distinct from the pure predicate above): must only
    # ever read last_cue_pos / state.pos_n / state.pos_e / range_filter.x_hat
    # (cue-legal-pre-handoff + camera-only) -- never a gt_*-named value.
    tree = _m4_tree()
    assigns = [
        n for n in ast.walk(tree)
        if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "phantom_expected" for t in n.targets)
    ]
    assert len(assigns) >= 2, "expected two assignments to phantom_expected (init None + branches)"
    for a in assigns:
        names = {x.attr for x in ast.walk(a.value) if isinstance(x, ast.Attribute)}
        names |= {x.id for x in ast.walk(a.value) if isinstance(x, ast.Name)}
        gt_names = {n for n in names if n.startswith("gt_")}
        assert not gt_names, f"phantom_expected computation must be gt_*-free, found {gt_names}"


if __name__ == "__main__":
    import subprocess
    rc = subprocess.call([sys.executable, "-m", "pytest", __file__, "-v"])
    sys.exit(rc)
