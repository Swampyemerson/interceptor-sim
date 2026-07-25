#!/usr/bin/env python3
"""REGRESSION: curve (b) must score REAL imagery with the REAL camera's optics.

THE DEFECT (review 2, 2026-07-25 -- docs/review2_silent_failure_findings.md
"tripod_score.curve_b scores real imagery with the SIM camera's hardcoded
intrinsics"). `curve_b` unpacked the session's real calibration and then called

    box_hits_gt(seeker._infer_boxes(frame), gcx, gcy, gw, gh)   # no fx/fy/cx/cy

so the hit test's sec^2 off-axis widening and its principal point came from
box_scoring's SIM defaults -- fx=fy=539.936, cx=640, cy=480, the
gz_x500_mono_cam at 1280x960 -- while the ordered OV9281 is 1280x800 at 118 deg
HFoV (fx~385, cy=400). The effective gt box was therefore UNDER-WIDENED by up to
~1.46x horizontally and ~1.48x vertically toward the frame edges, so real,
correctly-placed detections scored MISS.

That is not a random error: it is systematically worst OFF-BORESIGHT, which is
exactly the position-in-frame axis curve (b) exists to measure, and it depresses
recall in the top/bottom bands -- reading as CONFIRMATION of this project's own
"position-in-frame explains the recall gap" prior. It is the same shape as the
frametop 19%-vs-95% scoring artifact the project already got burned by, and
curve (b) is the number that decides the $70 Hailo HAT / the markerless phase.

Also pinned here: the local `def box_hits_gt` FALLBACK that used to sit behind
the import (the pre-2026-07-21 gate verbatim -- no sec^2, no gt_scale, tol=15)
is gone; box_scoring is pure, so the import must be fatal, never silently
downgraded to a known-wrong scorer.

Offline: a stub NN (no onnxruntime, no weights), tiny generated PNGs, a
NON-SIM calibration.

Run: .venv/bin/python -m pytest tests/test_curve_b_scoring.py -v
"""
import os
import sys

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEEKER = os.path.join(REPO_ROOT, "scripts", "seeker")
sys.path.insert(0, SEEKER)

import tripod_score as TS                                     # noqa: E402
import box_scoring as BS                                      # noqa: E402
import finetuned_seeker                                       # noqa: E402

# The REAL camera (docs/project_state.json hardware node: innomaker OV9281,
# 1280x800 mono global shutter, ~118 deg HFoV -> fx = 1280/(2*tan(59 deg)) ~ 385).
REAL_FX = REAL_FY = 384.6
REAL_CX, REAL_CY = 640.0, 400.0
REAL_W, REAL_H = 1280, 800
RANGE_M = 6.0
DRONE_SIZE_M = 0.35

# Truth-box centres: dead centre, hard right, and low. The off-axis two are
# where the sim-default intrinsics under-widen the gate.
CASES = [(640.0, 400.0), (1000.0, 520.0), (640.0, 700.0)]


def _realistic_detection(gcx, gcy, gw, gh, du, dv):
    """A plausible real NN box: correct-ish centre with an offset, and the ~1.8x
    over-box a YOLO detector actually produces. (score, u, v, w, h)."""
    return (0.83, gcx + du, gcy + dv, gw * 1.8, gh * 1.8)


class _StubSeeker:
    """Stands in for FinetunedNNSeeker. Returns the box registered for the frame
    it is handed -- the frame's (0,0) pixel carries its index, so this is robust
    to whatever order curve_b iterates in."""

    def __init__(self, *_a, **_k):
        self.boxes_by_idx = _StubSeeker.BOXES

    def _infer_boxes(self, frame):
        idx = int(frame[0][0] if frame.ndim == 2 else frame[0][0][0])
        return list(self.boxes_by_idx.get(idx, []))


def _gt_box(u0, v0):
    a = (u0 - REAL_CX) / REAL_FX
    b = (v0 - REAL_CY) / REAL_FY
    z = RANGE_M / np.sqrt(1.0 + a * a + b * b)
    from gen_sim_dataset import project_to_bbox
    return project_to_bbox(a * z, b * z, z, RANGE_M, REAL_FX, REAL_FY,
                           REAL_CX, REAL_CY, DRONE_SIZE_M, REAL_W, REAL_H)


def _build_session(tmp_path, offsets):
    """frames/ + a decode map placing the tag at each CASES centre."""
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    frames, decode = [], {}
    boxes = {}
    for i, (u0, v0) in enumerate(CASES):
        img = np.zeros((REAL_H, REAL_W), dtype=np.uint8)
        img[0][0] = i                       # frame identity for the stub seeker
        p = str(frames_dir / f"{i:06d}.png")
        cv2.imwrite(p, img)
        frames.append((p, f"{i:06d}"))
        decode[f"{i:06d}"] = (True, RANGE_M, u0, v0, 1)
        gt = _gt_box(u0, v0)
        assert gt is not None, "truth box must project on-frame"
        gcx, gcy, gw, gh = gt
        du, dv = offsets[i]
        boxes[i] = [_realistic_detection(gcx, gcy, gw, gh, du, dv)]
    return frames, decode, boxes


@pytest.fixture()
def calib():
    return (REAL_FX, REAL_FY, REAL_CX, REAL_CY, REAL_W, REAL_H, np.zeros(5))


# --------------------------------------------- the mechanism, at the unit level
def test_sim_default_intrinsics_reject_a_real_off_axis_detection():
    """The seam itself: the SAME detection is a HIT with the session's optics and
    a MISS with box_scoring's sim defaults. If this ever stops being true the
    integration test below is no longer measuring anything."""
    for (u0, v0), (du, dv) in zip(CASES[1:], [(34.0, 0.0), (0.0, 31.0)]):
        gcx, gcy, gw, gh = _gt_box(u0, v0)
        box = [_realistic_detection(gcx, gcy, gw, gh, du, dv)]
        with_real = BS.box_hits_gt(box, gcx, gcy, gw, gh, fx=REAL_FX, fy=REAL_FY,
                                   cx=REAL_CX, cy=REAL_CY, gt_scale=1.0)
        with_sim = BS.box_hits_gt(box, gcx, gcy, gw, gh)       # sim defaults
        assert with_real is True, f"real optics must accept ({u0},{v0})"
        assert with_sim is False, (
            f"sim defaults must reject ({u0},{v0}) -- that is the artifact")


# ------------------------------------------------------- the integration path
def test_curve_b_scores_off_axis_detections_as_HITS(tmp_path, calib, monkeypatch):
    """PRE-FIX: the two off-axis frames scored MISS and curve (b) reported ~33%
    recall with a clean-looking top/middle/bottom breakdown. POST-FIX all three
    are HITs and no truthed frame is dropped."""
    frames, decode, boxes = _build_session(
        tmp_path, offsets=[(10.0, 0.0), (34.0, 0.0), (0.0, 31.0)])
    _StubSeeker.BOXES = boxes
    monkeypatch.setattr(finetuned_seeker, "FinetunedNNSeeker", _StubSeeker)
    weights = tmp_path / "stub.onnx"
    weights.write_bytes(b"stub")

    bins, note = TS.curve_b(frames, decode, calib, str(weights), 0.25,
                            DRONE_SIZE_M, 2.0, 40.0)
    assert bins is not None, note
    hits, total, recall = TS.bins_overall(bins)
    assert total == len(CASES), f"a truthed frame was DROPPED: {note}"
    assert hits == len(CASES), (
        f"off-axis detections scored MISS ({hits}/{total}) -- the sim-intrinsics "
        f"artifact on exactly the axis curve (b) measures. {note}")
    assert recall == 1.0
    assert "DROPPED" not in note


def test_curve_b_still_scores_a_genuine_miss_as_a_MISS(tmp_path, calib, monkeypatch):
    """The fix widens the gate to the REAL optics -- it must not make the gate
    unconditionally generous. A box 6x the gt extent, far off the target, is
    still a MISS (box_scoring's blob guard)."""
    frames, decode, boxes = _build_session(
        tmp_path, offsets=[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)])
    gcx, gcy, gw, gh = _gt_box(*CASES[1])
    boxes[1] = [(0.9, gcx + 400.0, gcy, gw * 6.0, gh * 6.0)]   # a ground blob
    _StubSeeker.BOXES = boxes
    monkeypatch.setattr(finetuned_seeker, "FinetunedNNSeeker", _StubSeeker)
    weights = tmp_path / "stub.onnx"
    weights.write_bytes(b"stub")

    bins, note = TS.curve_b(frames, decode, calib, str(weights), 0.25,
                            DRONE_SIZE_M, 2.0, 40.0)
    hits, total, _rec = TS.bins_overall(bins)
    assert total == 3 and hits == 2, f"blob must not count as a hit: {note}"


def test_curve_b_has_no_silent_local_box_hits_gt_fallback():
    """The deleted fallback was the pre-2026-07-21 gate verbatim (no sec^2, no
    gt_scale, tol=15). Behind an except ImportError it would have produced the
    exact wrong answer this file exists to prevent, silently. box_scoring is pure
    (no cv2, no onnxruntime), so the import cannot fail for env reasons."""
    src = open(os.path.join(SEEKER, "tripod_score.py")).read()
    assert "def box_hits_gt(" not in src, (
        "a local box_hits_gt definition is a silent downgrade path")
    assert "from box_scoring import box_hits_gt" in src


def test_curve_b_passes_the_session_intrinsics_and_an_explicit_gt_scale(
        tmp_path, calib, monkeypatch):
    """Behavioural pin on the ARGUMENTS, not just the outcome: box_hits_gt must
    receive the SESSION's fx/fy/cx/cy, and gt_scale explicitly.

    gt_scale=1.0 is CORRECT here and must be EXPLICIT: the truth box is projected
    at `drone_size_m` (the REAL target's extent, from meta.json/--drone-size), so
    it is already at the scoring extent. box_scoring.TARGET_EXTENT_M = 0.52 is
    the SIM fpv_quad_enemy silhouette and must never be applied to real imagery
    -- leaving it implicit is how the frametop scoring artifact happened."""
    seen = []
    real = BS.box_hits_gt

    def _spy(*a, **kw):
        seen.append(kw)
        return real(*a, **kw)

    monkeypatch.setattr(BS, "box_hits_gt", _spy)
    frames, decode, boxes = _build_session(tmp_path, offsets=[(0.0, 0.0)] * 3)
    _StubSeeker.BOXES = boxes
    monkeypatch.setattr(finetuned_seeker, "FinetunedNNSeeker", _StubSeeker)
    weights = tmp_path / "stub.onnx"
    weights.write_bytes(b"stub")

    TS.curve_b(frames, decode, calib, str(weights), 0.25, DRONE_SIZE_M, 2.0, 40.0)
    assert seen, "box_hits_gt was never called (or was not the box_scoring one)"
    for kw in seen:
        assert kw.get("fx") == REAL_FX and kw.get("fy") == REAL_FY
        assert kw.get("cx") == REAL_CX and kw.get("cy") == REAL_CY
        assert kw.get("gt_scale") == 1.0
        # never the sim camera
        assert kw["fx"] != BS.FX and kw["cy"] != BS.CY


# -------------------------------------------------------- §8.2 answerability
def test_a_tag_limited_curve_b_reports_section_8_2_UNANSWERABLE(tmp_path, calib,
                                                                monkeypatch):
    """All three frames are truthed at 6 m -- inside the tag's decode ceiling and
    nowhere near protocol §8.2's 10-25 m threshold band. The scorer must say the
    §8.2 question is UNANSWERABLE rather than let a reader quote overall_recall
    as the answer."""
    frames, decode, boxes = _build_session(
        tmp_path, offsets=[(0.0, 0.0)] * 3)
    _StubSeeker.BOXES = boxes
    monkeypatch.setattr(finetuned_seeker, "FinetunedNNSeeker", _StubSeeker)
    weights = tmp_path / "stub.onnx"
    weights.write_bytes(b"stub")
    bins, _note = TS.curve_b(frames, decode, calib, str(weights), 0.25,
                             DRONE_SIZE_M, 2.0, 40.0)
    cov = TS.curve_b_band_coverage(bins, 2.0)
    assert cov["answerable"] is False
    assert cov["n_frames_in_band"] == 0 and cov["n_frames_scored"] == 3

    summary = TS.build_summary(
        TS.gate_verdict(6.0, 6.0, 1.0, 30.0, 5, 9.0, 0.5, n_decoded=40),
        None, {4.0: [3, 3, []]}, "note", {}, 3, 0, 2.0, band_coverage=cov)
    assert "§8.2 UNANSWERABLE" in summary
    assert "Do NOT quote overall_recall" in summary


def test_gate_json_carries_section_8_2_answerable(tmp_path):
    """The machine-readable half: a downstream reader/agent must be able to see
    the §8.2 question was not answered, not just a recall number."""
    src = open(os.path.join(SEEKER, "tripod_score.py")).read()
    assert '"section_8_2_answerable"' in src
    assert '"curve_b_band_coverage"' in src
