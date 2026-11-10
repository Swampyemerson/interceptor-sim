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
