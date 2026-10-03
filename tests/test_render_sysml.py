"""Contract + mutation tests for scripts/render_sysml.py (the SysML v2 hardware model).

Each mutation test edits ONE thing in a scratch copy of docs/sysml/ and asserts the
renderer refuses it with the right reason -- a check that has never been seen to
fail protects nothing (docs/error_handling_policy.md). Stdlib + pytest only.
"""

import copy
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import render_sysml as rs  # noqa: E402


@pytest.fixture(scope="module")
def contract():
    return rs.load_contract()


@pytest.fixture
def model_copy(tmp_path):
    d = tmp_path / "sysml"
    d.mkdir()
    for f in rs.MODEL_FILES:
        shutil.copy(rs.MODEL_DIR / f, d / f)
    return d


def mutate(d: Path, fname: str, old: str, new: str) -> None:
    p = d / fname
    text = p.read_text()
    assert old in text, f"mutation anchor {old!r} not found in {fname} -- update the test"
    p.write_text(text.replace(old, new, 1))


def expect_fail(d, contract, *needles):
    with pytest.raises(rs.ModelError) as ei:
        rs.load_model(d, contract)
    msg = str(ei.value)
    for n in needles:
        assert n in msg, f"expected {n!r} in:\n{msg}"


# --------------------------------------------------------------------- the real model
def test_real_model_is_valid_and_every_view_renders(contract):
    m = rs.load_model(rs.MODEL_DIR, contract)
    out = rs.render_all(m)
    assert set(out) == {v[0] for v in rs.VIEWS}
    for vid, (svg, routed, _ports) in out.items():
        assert svg.startswith("<svg") and svg.rstrip().endswith("</svg>"), vid
        for _, iu, _, pts in routed:
            assert iu.name in svg, f"{vid}: wire {iu.name} missing from its wire list"
            assert len(pts) >= 2


def test_every_connection_lands_in_at_least_one_view(contract):
    m = rs.load_model(rs.MODEL_DIR, contract)
    out = rs.render_all(m)
    drawn = {(r[1].owner, r[1].name) for _, (_, routed, _p) in out.items() for r in routed}
    for s in m.systems():
        for iu in s.interfaces:
            assert (s.name, iu.name) in drawn, f"{s.name}.{iu.name} is in no view"


def test_committed_views_are_current():
    r = subprocess.run([sys.executable, str(ROOT / "scripts" / "render_sysml.py"), "--check"],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_render_is_byte_identical_across_hash_seeds():
    code = ("import sys; sys.path.insert(0, 'scripts'); import render_sysml as r; "
            "print(r.views_digest(r.render_all(r.load_model())))")
    digests = set()
    for seed in ("0", "1", "4242"):
        env = dict(os.environ, PYTHONHASHSEED=seed)
        digests.add(subprocess.run([sys.executable, "-c", code], cwd=ROOT, env=env,
                                   capture_output=True, text=True, check=True).stdout.strip())
    assert len(digests) == 1, digests


# --------------------------------------------------------------------- wire checks
def test_wire_drawn_backwards_is_refused(model_copy, contract):
    mutate(model_copy, "Target.sysml", "connect battery.main to esc.batt",
           "connect esc.batt to battery.main")
    expect_fail(model_copy, contract, "mainPower", "drawn backwards")


def test_wire_into_the_wrong_kind_of_port_is_refused(model_copy, contract):
    mutate(model_copy, "Interceptor.sysml", "connect fc.sdSlot to fcSd.contacts",
           "connect fc.sdSlot to gps.mast")
    expect_fail(model_copy, contract, "ulogCard", "gps.mast is ~MountPort", "~SDHostPort")


def test_port_that_does_not_exist_is_refused(model_copy, contract):
    mutate(model_copy, "Interceptor.sysml", "connect fc.telem2 to pi.gpioUart",
           "connect fc.telem1 to pi.gpioUart")
    expect_fail(model_copy, contract, "companion", "has no port 'telem1'")


def test_verified_without_evidence_is_accepted(model_copy, contract):
    # Builder decision 2026-10-03: evidence is an optional free-form note, so a
    # wire can be marked wired/verified with no evidence (or any text) at all.
    mutate(model_copy, "Target.sysml", 'attribute :>> evidence = "tgt-04";', "")
    m = rs.load_model(model_copy, contract)
    tgt = next(s for s in m.systems() if s.name == "TargetDrone")
    iu = next(i for i in tgt.interfaces if i.name == "mainPower")
    assert iu.attrs.get("linkState") == "verified"
    assert not iu.attrs.get("evidence")


def test_free_form_evidence_is_accepted_and_kept(model_copy, contract):
    mutate(model_copy, "Target.sysml", 'evidence = "tgt-04"',
           'evidence = "bench session, 2026-10-03"')
    m = rs.load_model(model_copy, contract)
    tgt = next(s for s in m.systems() if s.name == "TargetDrone")
    iu = next(i for i in tgt.interfaces if i.name == "mainPower")
    assert iu.attrs.get("evidence") == "bench session, 2026-10-03"


def test_unknown_link_state_is_refused(model_copy, contract):
    mutate(model_copy, "Target.sysml", "LinkState::verified", "LinkState::probably")
    expect_fail(model_copy, contract, "linkState must be one of")


# --------------------------------------------------------------------- build-sheet coverage
def test_bom_that_matches_no_row_is_refused(model_copy, contract):
    mutate(model_copy, "Target.sysml", 'bomName = "Tekko32"', 'bomName = "Tekko64"')
    expect_fail(model_copy, contract, "'Tekko64' matches 0 rows")


def test_ambiguous_bom_is_refused(model_copy, contract):
    mutate(model_copy, "Target.sysml", 'bomName = "Tekko32"', 'bomName = "e"')
    expect_fail(model_copy, contract, "must be exactly 1")


def test_build_sheet_row_nobody_draws_is_refused(model_copy, contract):
    state, rows, steps = contract
    rows = copy.deepcopy(rows)
    rows["airframe"].append({"name": "Mystery widget", "status": "ordered", "role": "", "notes": ""})
    expect_fail(model_copy, (state, rows, steps), "'Mystery widget'", "neither drawn")


def test_row_both_drawn_and_excluded_is_refused(model_copy, contract):
    anchor = '@NotModelled { tab = "ground"; bomName = "Tool layer"; reason = "tools"; }'
    mutate(model_copy, "GroundSegment.sysml", anchor,
           anchor + '\n    @NotModelled { tab = "target"; bomName = "Tekko32"; reason = "x"; }')
    expect_fail(model_copy, contract, "'Tekko32 F4 Metal 65A 4-in-1 ESC' is BOTH drawn")


def test_part_without_bom_or_external_is_refused(model_copy, contract):
    mutate(model_copy, "SeekerRig.sysml", "            @External;\n            @Layout { col = 2; row = 2; }",
           "            @Layout { col = 2; row = 2; }")
    expect_fail(model_copy, contract, "hotspot needs @BOM")


def test_two_parts_in_one_layout_cell_are_refused(model_copy, contract):
    mutate(model_copy, "Target.sysml", "@Layout { col = 0; row = 2; }", "@Layout { col = 0; row = 1; }")
    expect_fail(model_copy, contract, "share layout cell")


# --------------------------------------------------------------------- parser fail-closed
def test_unsupported_syntax_fails_loudly_instead_of_being_dropped(model_copy, contract):
    mutate(model_copy, "Target.sysml", "        // ---------------------------------------------------------- power\n",
           "        connect battery.main to esc.batt;\n")
    expect_fail(model_copy, contract, "unsupported member 'connect'")


def test_new_sysml_file_must_be_registered(model_copy, contract):
    (model_copy / "Extra.sysml").write_text("package Extra { }\n")
    expect_fail(model_copy, contract, "MODEL_FILES")
