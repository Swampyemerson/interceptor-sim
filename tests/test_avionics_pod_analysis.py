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
