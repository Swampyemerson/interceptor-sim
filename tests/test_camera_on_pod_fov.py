"""The camera-on-pod prop-clearance derivation: its self-test must pass, and the
README's headline margins must be what the script computes."""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "hardware" / "avionics_pod"))
import camera_on_pod_fov as C  # noqa: E402


def test_self_test_passes():
    r = subprocess.run([sys.executable, str(ROOT / "hardware/avionics_pod/camera_on_pod_fov.py"),
                        "--self-test"], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_readme_margins_match_the_script():
    want = {"OV9281 as delivered (70 deg)": (29.5, 44.5, 64.5),
            "AR0234 stock lens (80 deg)": (25.5, 40.5, 60.5),
            "OV9281 published lens (118 deg)": (7, 22, 42)}
    for name, vals in want.items():
        hfov, asp = C.CAMERAS[name]
        for tilt, v in zip((10, 25, 45), vals):
            got = C.margin_deg(60.0, 68.0, tilt, hfov, asp, 0.0)
            assert abs(got - v) <= 0.6, (name, tilt, got, v)


def test_a_camera_in_the_prop_plane_between_the_props_sees_them():
    # mutant guard: the check must be able to say IN FRAME
    assert C.margin_deg(20.0, 0.0, 0.0, 118.0, 0.625) < 0
