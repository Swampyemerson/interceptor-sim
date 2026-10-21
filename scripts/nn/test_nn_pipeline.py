"""Tests for autolabel_tag.py and sim_randomize.py (no real tags / pupil_apriltags needed)."""
from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

import autolabel_tag as al
import sim_randomize as sr

W, H = 1280, 800


def intr(dist=(0, 0, 0, 0, 0)) -> al.Intrinsics:
    K = np.array([[1000.0, 0, 640.0], [0, 1000.0, 400.0], [0, 0, 1]])
    return al.Intrinsics(K=K, dist=np.array(dist, dtype=float), width=W, height=H)


# Cuboid in the documented out_of_face frame: x right, y up, z out of the tag face.
# The body sits BEHIND the tag face (z in [-0.2, 0]).
CUBOID_OOF = np.array([[x, y, z] for x in (-0.1, 0.1) for y in (-0.05, 0.05) for z in (-0.2, 0.0)])


class FakeDetector:
    def __init__(self, dets_by_call):
        self.dets_by_call = list(dets_by_call)
        self.calls = 0

    def detect(self, img):
        d = self.dets_by_call[min(self.calls, len(self.dets_by_call) - 1)]
        self.calls += 1
        return d


# ---------------------------------------------------------------- geometry
def test_projected_box_matches_hand_computation():
    # rvec=0, tvec=(0,0,2): native tag frame == camera frame shifted 2 m forward.
    # out_of_face -> native is (x, -y, -z): z_cam in [2.0, 2.2].
    # u = 640 + 1000 x / z_cam: extremes at z_cam=2 -> 590 .. 690 ; v -> 375 .. 425.
    # 5% shrink: w=100 -> 5 px, h=50 -> 2.5 px  => (595, 377.5, 685, 422.5)
    native = al.cuboid_to_native(CUBOID_OOF, "out_of_face")
    box, trunc = al.cuboid_bbox_px(native, np.zeros(3), np.array([0, 0, 2.0]), intr())
    assert not trunc
    np.testing.assert_allclose(box, (595.0, 377.5, 685.0, 422.5), atol=1.0)
    cx, cy, w, h = al.bbox_to_yolo(box, W, H)
    np.testing.assert_allclose((cx, cy, w, h), (640 / W, 400 / H, 90 / W, 45 / H), atol=1e-3)


def test_box_clipped_and_truncated_flag():
    native = al.cuboid_to_native(CUBOID_OOF, "out_of_face")
    box, trunc = al.cuboid_bbox_px(native, np.zeros(3), np.array([-1.3, 0, 2.0]), intr())
    # unclipped u spans -60 .. 95 px -> left edge clipped to the image border
    assert trunc and box[0] == 0.0 and 80 < box[2] < 95


def test_cuboid_behind_camera_raises():
    native = al.cuboid_to_native(CUBOID_OOF, "out_of_face")
    toward_cam = CUBOID_OOF * np.array([1, 1, -1.5])  # body sticks 0.3 m OUT of the face
    native = al.cuboid_to_native(toward_cam, "out_of_face")
    with pytest.raises(ValueError):
        al.cuboid_bbox_px(native, np.zeros(3), np.array([0, 0, 0.1]), intr())


def test_pnp_recovers_known_pose_with_distortion():
    it = intr(dist=(-0.12, 0.03, 0.001, -0.0005, 0.0))
    rvec_true = np.array([0.25, -0.35, 0.1])
    tvec_true = np.array([0.15, -0.05, 1.8])
    corners = al.project(al.tag_object_points(0.1), rvec_true, tvec_true, it)
    pose = al.solve_tag_pose(corners, 0.1, it)
    assert pose.reproj_err_px < 1e-3
    np.testing.assert_allclose(pose.tvec, tvec_true, atol=1e-4)
    np.testing.assert_allclose(pose.rvec, rvec_true, atol=1e-4)
    R, _ = cv2.Rodrigues(rvec_true)
    expect = np.degrees(np.arccos(abs(R[:, 2] @ tvec_true) / np.linalg.norm(tvec_true)))
    assert abs(pose.incidence_deg - expect) < 1e-3


def test_mask_fills_exactly_quad_plus_dilation():
    quad = np.array([[500.3, 450.7], [610.2, 430.1], [640.8, 330.4], [520.6, 345.9]])
    m = al.tag_mask(quad, (H, W), dilate_px=2)
    poly = quad.astype(np.float32).reshape(-1, 1, 2)
    ys, xs = np.mgrid[300:480, 470:680]
    for x, y in zip(xs.ravel(), ys.ravel()):
        d = cv2.pointPolygonTest(poly, (float(x), float(y)), True)  # +inside
        if d >= 0.0:
            assert m[y, x] == 255, (x, y, d)
        elif d < -(2 + 1.0):  # dilation radius + corner rounding/pixel slop
            assert m[y, x] == 0, (x, y, d)
    # Nothing outside the inspected window.
    outside = m.copy()
    outside[300:480, 470:680] = 0
    assert outside.sum() == 0
    # Area check: quad area + perimeter*2 approx (within rounding).
    area = cv2.contourArea(poly)
    perim = cv2.arcLength(poly, True)
    n = int((m > 0).sum())
    assert area < n < area + perim * 2 + 4 * np.pi + perim * 0.75


def test_noise_mask_matches_ring_statistics_and_touches_only_mask():
    rng = np.random.default_rng(1)
    img = np.clip(rng.normal(60, 5, (H, W)), 0, 255).astype(np.uint8)
    quad = np.array([[600, 440], [680, 440], [680, 360], [600, 360]], float)
    m = al.tag_mask(quad, (H, W))
    img[m > 0] = 250  # bright "tag"
    out = al.apply_mask(img, m, "noise", np.random.default_rng(0))
    assert out.ndim == 2 and out.dtype == np.uint8
    assert abs(out[m > 0].mean() - 60) < 2 and abs(out[m > 0].std() - 5) < 1.5
    np.testing.assert_array_equal(out[m == 0], img[m == 0])
    inp = al.apply_mask(img, m, "inpaint", np.random.default_rng(0))
    assert inp.ndim == 2 and inp[m > 0].mean() < 120


# ------------------------------------------------------------- end to end
def _make_frames(tmp: Path, n: int = 4, dt: float = 0.25) -> tuple[Path, Path, Path]:
    fdir = tmp / "frames"
    fdir.mkdir()
    rows = ["frame,t_mono,path"]
    rng = np.random.default_rng(0)
    for i in range(n):
        img = np.clip(rng.normal(80, 6, (H, W)), 0, 255).astype(np.uint8)
        cv2.imwrite(str(fdir / f"f{i:04d}.jpg"), img)
        rows.append(f"{i},{100 + i * dt:.3f},f{i:04d}.jpg")
    (fdir / "frames_index.csv").write_text("\n".join(rows) + "\n")
    ij = tmp / "intr.json"
    ij.write_text(json.dumps({"resolution": {"width": W, "height": H}, "fx": 1000.0, "fy": 1000.0,
                              "cx": 640.0, "cy": 400.0, "dist_coeffs": [-0.1, 0.02, 0, 0, 0]}))
    cj = tmp / "cuboid.json"
    cj.write_text(json.dumps({"corners_tag_frame": CUBOID_OOF.tolist()}))
    return fdir, ij, cj


def test_end_to_end_with_injected_detections(tmp_path):
    fdir, ij, cj = _make_frames(tmp_path)
    it = al.Intrinsics.from_json(ij)
    rvec, tvec = np.array([0.1, 0.2, 0.0]), np.array([0.05, 0.02, 1.5])
    corners = al.project(al.tag_object_points(0.1), rvec, tvec, it)
    det = SimpleNamespace(corners=corners, tag_id=0, hamming=0)
    fake = FakeDetector([[det], []])  # 2nd sampled frame: no tag
    out = tmp_path / "out"
    rc = al.main(["--frames", str(fdir), "--intrinsics", str(ij), "--cuboid", str(cj),
                  "--out", str(out), "--viz", "1"], detector=fake)
    assert rc == 0
    man = json.loads((out / "manifest.json").read_text())
    s = man["summary"]
    assert (s["n_frames_seen"], s["n_sampled"], s["n_labeled"]) == (4, 2, 1)  # 2 Hz of a 4 Hz index
    assert s["reject_counts_by_reason"] == {"no_tag": 1}
    lab = (out / "labels" / "f0000.txt").read_text().split()
    exp_box, _ = al.cuboid_bbox_px(al.cuboid_to_native(CUBOID_OOF), rvec, tvec, it)
    np.testing.assert_allclose([float(v) for v in lab[1:]], al.bbox_to_yolo(exp_box, W, H), atol=2e-4)
    assert lab[0] == "0"
    img = cv2.imread(str(out / "images" / "f0000.jpg"), cv2.IMREAD_UNCHANGED)
    assert img.ndim == 2
    rec = man["frames"][0]
    assert rec["masked"] is True and rec["reproj_err_px"] < 0.01
    assert man["frames"][1]["masked"] is False and man["frames"][1]["reject_reason"] == "no_tag"
    assert len(list((out / "viz").iterdir())) == 1


def test_rejects_small_tag_and_incidence(tmp_path):
    fdir, ij, cj = _make_frames(tmp_path)
    it = al.Intrinsics.from_json(ij)
    small = al.project(al.tag_object_points(0.1), np.zeros(3), np.array([0, 0, 8.0]), it)  # ~12 px
    oblique = al.project(al.tag_object_points(0.1), np.array([0, np.radians(65), 0]), np.array([0, 0, 1.0]), it)
    fake = FakeDetector([[SimpleNamespace(corners=small)], [SimpleNamespace(corners=oblique)]])
    rc = al.main(["--frames", str(fdir), "--intrinsics", str(ij), "--cuboid", str(cj),
                  "--out", str(tmp_path / "o")], detector=fake)
    assert rc == al.EXIT_NO_LABELS
    s = json.loads((tmp_path / "o" / "manifest.json").read_text())["summary"]
    assert s["reject_counts_by_reason"] == {"incidence": 1, "tag_too_small": 1}


def test_zero_labeled_exits_nonzero(tmp_path, capsys):
    fdir, ij, cj = _make_frames(tmp_path)
    rc = al.main(["--frames", str(fdir), "--intrinsics", str(ij), "--cuboid", str(cj),
                  "--out", str(tmp_path / "o")], detector=FakeDetector([[]]))
    assert rc == al.EXIT_NO_LABELS != 0
    assert "zero frames labeled" in capsys.readouterr().err


def test_detector_exception_is_counted(tmp_path):
    class Boom:
        def detect(self, img):
            raise RuntimeError("detector crashed")
    fdir, ij, cj = _make_frames(tmp_path)
    rc = al.main(["--frames", str(fdir), "--intrinsics", str(ij), "--cuboid", str(cj),
                  "--out", str(tmp_path / "o")], detector=Boom())
    assert rc == al.EXIT_NO_LABELS
    s = json.loads((tmp_path / "o" / "manifest.json").read_text())["summary"]
    assert s["reject_counts_by_reason"] == {"exception:RuntimeError": 2}


# ---------------------------------------------------------- sim_randomize
def _make_sim_set(tmp: Path) -> tuple[Path, Path]:
    idir, ldir = tmp / "img", tmp / "lab"
    idir.mkdir()
    ldir.mkdir()
    rng = np.random.default_rng(0)
    specs = {
        "a": (rng.integers(0, 255, (120, 160, 3), dtype=np.uint8), "0 0.5 0.5 0.2 0.3\n"),
        "b": (rng.integers(0, 255, (120, 160), dtype=np.uint8), "0 0.05 0.05 0.1 0.1\n0 0.95 0.9 0.1 0.2\n"),
        "c": (np.full((100, 100), 230, np.uint8), "0 0.5 0.5 1.0 1.0\n"),
    }
    for stem, (img, lab) in specs.items():
        cv2.imwrite(str(idir / f"{stem}.png"), img)
        (ldir / f"{stem}.txt").write_text(lab)
    return idir, ldir


def test_randomize_counts_bounds_gray_and_determinism(tmp_path):
    idir, ldir = _make_sim_set(tmp_path)
    o1, o2 = tmp_path / "o1", tmp_path / "o2"
    assert sr.main(["--images", str(idir), "--labels", str(ldir), "--out", str(o1), "--copies", "3"]) == 0
    man = json.loads((o1 / "aug_manifest.json").read_text())
    assert man["n_out"] == man["n_in"] * 3 == 9
    labs = sorted((o1 / "labels").glob("*.txt"))
    assert len(labs) == 9 and len(list((o1 / "images").glob("*.jpg"))) == 9
    for lp in labs:
        lines = lp.read_text().splitlines()
        src_n = len((ldir / f"{lp.stem.split('_aug')[0]}.txt").read_text().splitlines())
        assert len(lines) == src_n
        for line in lines:
            vals = [float(v) for v in line.split()[1:]]
            assert all(0.0 <= v <= 1.0 for v in vals), line
            cx, cy, w, h = vals
            assert cx - w / 2 >= -1e-6 and cx + w / 2 <= 1 + 1e-6
            assert cy - h / 2 >= -1e-6 and cy + h / 2 <= 1 + 1e-6
    for ip in (o1 / "images").glob("*.jpg"):
        assert cv2.imread(str(ip), cv2.IMREAD_UNCHANGED).ndim == 2
    a_w = [f["gray_weights_rgb"] for f in man["files"] if f.get("stem") == "a"]
    assert all(w is not None and abs(sum(w) - 1) < 1e-9 for w in a_w)
    assert len({tuple(w) for w in a_w}) == 3  # weights jitter per copy
    # Determinism: same seed -> identical bytes; different seed -> differs.
    assert sr.main(["--images", str(idir), "--labels", str(ldir), "--out", str(o2), "--copies", "3"]) == 0
    for ip in (o1 / "images").glob("*.jpg"):
        assert ip.read_bytes() == (o2 / "images" / ip.name).read_bytes()
    o3 = tmp_path / "o3"
    assert sr.main(["--images", str(idir), "--labels", str(ldir), "--out", str(o3), "--copies", "3",
                    "--seed", "7"]) == 0
    assert any(ip.read_bytes() != (o3 / "images" / ip.name).read_bytes() for ip in (o1 / "images").glob("*.jpg"))


def test_sky_gradient_amplitude_bounded():
    t = sr.LinearSkyGradient(amplitude_dn=25, p=1.0)
    img = np.full((50, 80), 100, np.uint8)
    out = t.apply(img, angle=0.7, amp=25.0).astype(int) - 100
    assert out.max() <= 25 and out.min() >= -25 and out.max() - out.min() >= 48


def test_randomize_missing_label_and_empty_fail_loudly(tmp_path):
    idir, ldir = _make_sim_set(tmp_path)
    (ldir / "c.txt").unlink()
    rc = sr.main(["--images", str(idir), "--labels", str(ldir), "--out", str(tmp_path / "o"), "--copies", "2"])
    assert rc == sr.EXIT_REJECTS
    man = json.loads((tmp_path / "o" / "aug_manifest.json").read_text())
    assert man["n_out"] == 4 and man["reject_counts_by_reason"] == {"missing_label": 2}
    empty = tmp_path / "empty"
    empty.mkdir()
    assert sr.main(["--images", str(empty), "--labels", str(ldir), "--out", str(tmp_path / "o2")]) == sr.EXIT_NO_OUTPUT
