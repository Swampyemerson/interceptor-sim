"""Avionics-pod analysis: self-test must pass and the report must run on the
committed inputs (mass_report.json, fea_results/)."""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "hardware" / "avionics_pod" / "analysis.py"


def test_self_test_passes():
    r = subprocess.run([sys.executable, str(SCRIPT), "--self-test"], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "FAIL" not in r.stdout


def test_report_runs_and_has_every_fea_case():
    r = subprocess.run([sys.executable, str(SCRIPT)], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    for case in ("camera ", "camera_soft_plastic", "no_camera ", "no_camera_soft_plastic",
                 "nose_post ", "nose_post_soft_plastic"):
        assert case in r.stdout, case
    assert "D  pod, camera on the dock's nose post" in r.stdout


def test_nose_post_is_stiffer_than_the_lid_cradle_in_the_saved_results():
    import json
    d = ROOT / "hardware" / "avionics_pod" / "fea_results"
    nose = json.loads((d / "nose_post.json").read_text())["freqs_hz"][0]
    lid = json.loads((d / "camera.json").read_text())["freqs_hz"][0]
    assert nose > 1.5 * lid, (nose, lid)


def test_pod_folder_holds_only_current_3d_files():
    """ops.md: no superseded fabrication files. The folder's STL/STEP files must be
    exactly the parts pod.py exports (its EXPORT tuple) plus the one assembly."""
    import ast
    pod_dir = ROOT / "hardware" / "avionics_pod"
    consts = {}
    for node in ast.parse((pod_dir / "pod.py").read_text()).body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") in ("EXPORT", "ASSEMBLY"):
            consts[node.targets[0].id] = ast.literal_eval(node.value)
    assert len(consts.get("EXPORT", ())) >= 4 and consts.get("ASSEMBLY"), consts
    want = {f"{n}.stl" for n in consts["EXPORT"]} | {f"{n}.step" for n in consts["EXPORT"]}
    want.add(consts["ASSEMBLY"] + ".step")
    have = {f.name for f in pod_dir.iterdir() if f.suffix.lower() in (".stl", ".step", ".stp")}
    assert have == want, {"stale or unexpected": sorted(have - want), "missing": sorted(want - have)}


def test_pod_readme_names_no_removed_file_as_current():
    """Every STL the folder README's file table lists must exist."""
    import re
    pod_dir = ROOT / "hardware" / "avionics_pod"
    text = (pod_dir / "README.md").read_text()
    table = text.split("## Files in this folder", 1)[1].split("\n## ", 1)[0]
    names = re.findall(r"`([a-z_]+\.stl)`", table)
    assert len(names) >= 4, names
    for n in names:
        assert (pod_dir / n).exists(), n


def test_insert_pockets_leave_room_for_melt_and_thread():
    """2026-11-20: the Pi-boss pockets were 1 mm deeper than their inserts, the melted
    plastic rose into the threads and all four standoffs stripped. pod.py checks this
    before exporting (insert_pocket_check); this mirrors the rule where the suite runs
    without CadQuery, reading the numbers from pod.py itself."""
    import ast
    c = {}
    for node in ast.parse((ROOT / "hardware" / "avionics_pod" / "pod.py").read_text()).body:
        if not isinstance(node, ast.Assign):
            continue
        tgt, val = node.targets[0], node.value
        names = [tgt] if isinstance(tgt, ast.Name) else list(getattr(tgt, "elts", []))
        vals = [val] if isinstance(tgt, ast.Name) else list(getattr(val, "elts", []))
        for n, v in zip(names, vals):
            if isinstance(n, ast.Name) and isinstance(v, ast.Constant) and isinstance(v.value, (int, float)):
                c[n.id] = float(v.value)
    need = ("INSERT_RELIEF_MIN", "INSERT_M25_DEPTH", "INSERT_M25_L", "M25_STUD_L", "INSERT_M3_L",
            "BOSS_M3_DEPTH", "REAR_SCREW_L", "REAR_HAT_ROOF", "REAR_HAT_GAP", "NOSE_EAR_T", "DOCK_T")
    missing = [k for k in need if k not in c]
    assert not missing, missing          # a renamed constant must fail, not pass empty
    assert c["INSERT_RELIEF_MIN"] >= 2.0
    assert c["INSERT_M25_DEPTH"] >= c["INSERT_M25_L"] + c["INSERT_RELIEF_MIN"]
    assert c["INSERT_M25_DEPTH"] >= c["M25_STUD_L"] + 1.0
    reach = c["REAR_SCREW_L"] - c["REAR_HAT_ROOF"] - c["REAR_HAT_GAP"]
    assert c["BOSS_M3_DEPTH"] >= c["INSERT_M3_L"] + c["INSERT_RELIEF_MIN"]
    assert c["BOSS_M3_DEPTH"] >= reach + 1.0 and reach >= 3.0
    assert 5.0 + c["DOCK_T"] - c["BOSS_M3_DEPTH"] >= 0.8
    assert c["INSERT_M3_L"] <= c["NOSE_EAR_T"]


# --- GPS mast and deck (2026-11-22) ---------------------------------------------

def _pod_constants():
    import ast
    c = {}
    for node in ast.parse((ROOT / "hardware" / "avionics_pod" / "pod.py").read_text()).body:
        if not isinstance(node, ast.Assign):
            continue
        tgt, val = node.targets[0], node.value
        names = [tgt] if isinstance(tgt, ast.Name) else list(getattr(tgt, "elts", []))
        vals = [val] if isinstance(tgt, ast.Name) else list(getattr(val, "elts", []))
        for n, v in zip(names, vals):
            if isinstance(n, ast.Name) and isinstance(v, ast.Constant) and isinstance(v.value, (int, float)):
                c[n.id] = float(v.value)
    return c


def test_gps_mast_inserts_screws_and_no_steel_rule():
    """Mirrors pod.py's mast_check where the suite runs without CadQuery: the deck's
    insert pockets leave room for melt and thread, the nylon screws reach through the
    inserts, and the post tops have wall round them."""
    c = _pod_constants()
    reach = c["DECK_SCREW_L"] - c["DECK_T"]
    assert c["DECK_INSERT_DEPTH"] >= c["INSERT_M3_L"] + c["INSERT_RELIEF_MIN"]
    assert c["DECK_INSERT_DEPTH"] >= reach + 1.0
    assert reach >= c["INSERT_M3_L"]
    assert c["MAST_BOSS_H"] >= c["DECK_INSERT_DEPTH"] + 1.5
    assert (c["MAST_BOSS_D"] - c["INSERT_M3_D"]) / 2 >= 2.0
    assert c["MAST_POST_INSET"] - c["MAST_FOOT_D"] / 2 - 2.5 >= 1.5, "no screwdriver room at the lid screws"
    text = (ROOT / "hardware" / "avionics_pod" / "README.md").read_text()
    section = text.split("## GPS mast and deck", 1)[1].split("\n## ", 1)[0]
    assert "nylon M3" in section and "brass M3" in section, "the no-steel hardware must be named"


def test_gps_mast_saved_vibration_result_clears_the_target_and_matches_the_design():
    """The saved FEA rows belong to the mast height and deck size pod.py draws today,
    and even the soft-plastic row is above the 80 Hz target."""
    import json
    c = _pod_constants()
    d = ROOT / "hardware" / "avionics_pod" / "fea_results"
    for name in ("gps_mast.json", "gps_mast_soft_plastic.json"):
        r = json.loads((d / name).read_text())
        assert r["gps_mast"] == {"mast_h_mm": c["MAST_H"], "deck_d_mm": c["DECK_D"], "gps_g": c["GPS_MASS"]}, name
        assert r["freqs_hz"][0] > 80.0, (name, r["freqs_hz"][0])
    soft = json.loads((d / "gps_mast_soft_plastic.json").read_text())["freqs_hz"][0]
    stiff = json.loads((d / "gps_mast.json").read_text())["freqs_hz"][0]
    assert soft < stiff


def test_gps_mast_numbers_in_the_readme_match_the_reports():
    import json, re
    pod_dir = ROOT / "hardware" / "avionics_pod"
    rep = json.loads((pod_dir / "mass_report.json").read_text())
    gm, pg = rep["gps_mast"], rep["printed_g"]
    section = (pod_dir / "README.md").read_text().split("## GPS mast and deck", 1)[1].split("\n## ", 1)[0]
    assert f"{gm['added_printed_g_over_plain_lid']} g" in section
    assert f"{pg['pod_lid_mast']} g" in section and f"{pg['gps_deck']} g" in section
    assert f"{gm['gps_antenna_above_pi_board_mm']} mm" in section
    assert f"{gm['gps_top_above_top_plate_mm']} mm" in section
    f0 = json.loads((pod_dir / "fea_results" / "gps_mast.json").read_text())["freqs_hz"]
    s0 = json.loads((pod_dir / "fea_results" / "gps_mast_soft_plastic.json").read_text())["freqs_hz"]
    for v in (f0[0], f0[1], s0[0], s0[1]):
        assert re.search(rf"\b{round(v)}\b", section), round(v)
