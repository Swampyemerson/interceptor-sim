#!/usr/bin/env python3
r"""T19a OFFLINE unit tests for `scripts/ground_station/station.py` (the
detect->triangulate->emit ground-station orchestrator, docs/decisions.md
ADR-0048/ADR-0046/ADR-0050). NO Gazebo/PX4/gz -- pure synthetic-clock +
synthetic-or-real-detector checks, runs in a few seconds under the MAIN
`.venv` (which has pytest but neither onnxruntime nor gz-transport13 --
`station.py`'s module docstring explains why that's fine: every gz/
onnxruntime/cv2 import in `station.py`/`detect.py` is DEFERRED, so this
whole file collects and runs without either dependency).

VENV: `.venv/bin/python -m pytest tests/test_ground_station.py -v` (matches
`tests/test_t18_scaffold.py`'s own documented convention -- main `.venv`,
NOT `.venv-seeker`, which has no pytest).

Six groups, mapping to the task's requirements:
  (1) THE DUAL GATE (ADR-0048 SS3, the core deliverable) -- `CueScheduler`
      exercised directly and structurally: not released before the deadline
      even when ready; not released past the deadline when not ready
      (`floor_violated` logged exactly once); released when BOTH hold; FIFO
      ordering blocks a later-ready item behind an earlier not-ready one.
  (2) t_sim PROVENANCE -- the emitted payload's `"t_sim"` is the frame's
      baked capture time, provably independent of the sim-time value the
      item happens to be released at.
  (3) WIRE SCHEMA -- a built payload is the exact `:47800` JSON schema and
      round-trips like `m4_intercept.CueReader` parses it (5-key and
      8-key/with-velocity forms).
  (4) --datum-bias-m -- exercised THROUGH `station.process_frame()` (not
      just `triangulate.py` directly, which `tests/test_t18_scaffold.py`
      already covers) on 3 REAL cached capture rows, reusing
      `triangulate.py`'s proven exact-shift algebra.
  (5) HONESTY -- `station.py`'s source holds no gt/pose gz subscription,
      only `/clock`.
  (6) REAL DETECTOR (optional, `pytest.importorskip`'d) -- `GroundDetector`
      on 2 real cached frames, cross-checked against the T18 harness's own
      ultralytics-derived reference centroids
      (`logs/t18_nn_sigma_validation/nn_detections_per_frame.csv`).
      SKIPPED under the main `.venv` (no onnxruntime there by design --
      see `station.py`'s interpreter note); would run under `.venv-seeker`
      if pytest were added there.
"""
import json
import math
import os
import sys

import pytest

SCRIPTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts")
sys.path.insert(0, SCRIPTS)

from ground_station.station import (  # noqa: E402
    CueScheduler,
    DEFAULT_CAPTURE_DIR,
    DEFAULT_LATENCY_S,
    apply_spoof,
    frame_paths,
    load_capture,
    process_frame,
    select_dash_direction,
)
from ground_station.triangulate import (  # noqa: E402
    apply_datum_bias,
    build_cue_payload,
    project_world_to_stereo_pixels,
    RigConfig,
)
from ground_station.detect import DetectionResult, box_center  # noqa: E402

STATION_PATH = os.path.join(SCRIPTS, "ground_station", "station.py")
CAPTURE_DIR = DEFAULT_CAPTURE_DIR


# ============================================================================
# fixtures
# ============================================================================
@pytest.fixture(scope="module")
def rig():
    return RigConfig.from_capture_meta(os.path.join(CAPTURE_DIR, "capture_meta.json"))


@pytest.fixture(scope="module")
def capture_rows():
    _meta, rows = load_capture(CAPTURE_DIR)
    selected, _direction, _all = select_dash_direction(rows, "r2l")
    return selected


# ============================================================================
# (1) THE DUAL GATE -- ADR-0048 SS3
# ============================================================================
def test_not_released_before_deadline_even_when_result_ready():
    """`sim_t < deliver_t_sim`, result_ready=True -- must NOT release, and
    must NOT log a floor_violated (the deadline itself hasn't passed)."""
    s = CueScheduler()
    s.schedule(seq=1, frame_t_sim=0.0, deliver_t_sim=1.0)
    s.complete(1, payload={"seq": 1, "t_sim": 0.0, "x": 1.0, "y": 2.0, "z": 3.0})

    released = s.poll(0.5)

    assert released == []
    assert s.floor_violation_log == []
    assert s.n_pending == 1


def test_not_released_past_deadline_when_result_not_ready_logs_floor_violated():
    """`sim_t >= deliver_t_sim`, result_ready=False -- must NOT release, and
    the result-not-ready clause is the binding one -> floor_violated logged."""
    s = CueScheduler()
    s.schedule(seq=1, frame_t_sim=0.0, deliver_t_sim=1.0)

    released = s.poll(1.5)  # deadline passed, never complete()'d

    assert released == []
    assert len(s.floor_violation_log) == 1
    assert s.floor_violation_log[0]["seq"] == 1
    assert s.n_pending == 1


def test_released_when_both_conditions_true_no_floor_violation():
    """The clean path: result becomes ready BEFORE the deadline passes ->
    release happens with floor_violated=False and nothing in the violation
    log (result-not-ready was never the binding clause)."""
    s = CueScheduler()
    s.schedule(seq=1, frame_t_sim=0.0, deliver_t_sim=1.0)
    s.complete(1, payload={"seq": 1, "t_sim": 0.0, "x": 1.0, "y": 2.0, "z": 3.0})

    released = s.poll(1.0)

    assert len(released) == 1
    item = released[0]
    assert item.seq == 1
    assert item.floor_violated is False
    assert s.floor_violation_log == []
    assert s.n_pending == 0


def test_floor_violated_logged_exactly_once_iff_not_ready_was_binding():
    """floor_violated logged IFF the result-not-ready clause was binding on
    some poll -- logged once when first observed, NOT re-logged on repeated
    polls while still not-ready, and the flag carries through to release
    (item.floor_violated=True) once the result finally arrives, without a
    second log-list entry."""
    s = CueScheduler()
    s.schedule(seq=1, frame_t_sim=0.0, deliver_t_sim=1.0)

    r1 = s.poll(1.2)   # deadline passed, not ready -> violation observed
    assert r1 == []
    assert len(s.floor_violation_log) == 1

    r2 = s.poll(1.3)   # still not ready -> must NOT double-log
    assert r2 == []
    assert len(s.floor_violation_log) == 1

    s.complete(1, payload={"seq": 1, "t_sim": 0.0, "x": 9.0, "y": 9.0, "z": 9.0})
    r3 = s.poll(1.4)

    assert len(r3) == 1
    assert r3[0].floor_violated is True          # this item's history: it WAS violated
    assert len(s.floor_violation_log) == 1        # still exactly one log entry, not re-logged at release


def test_clean_release_never_appears_in_floor_violation_log():
    """Complement of the above: an item whose result was ready BEFORE its
    deadline passed must never appear in floor_violation_log, even after
    many polls before the deadline."""
    s = CueScheduler()
    s.schedule(seq=1, frame_t_sim=0.0, deliver_t_sim=1.0)
    s.complete(1, payload={"seq": 1, "t_sim": 0.0, "x": 0.0, "y": 0.0, "z": 0.0})
    for t in (0.1, 0.3, 0.6, 0.9):
        assert s.poll(t) == []
    released = s.poll(1.0)
    assert len(released) == 1
    assert released[0].floor_violated is False
    assert s.floor_violation_log == []


def test_fifo_blocks_later_ready_item_behind_earlier_not_ready_item():
    """A real UDP cue stream must stay seq-ordered: seq=2's result being
    ready does NOT let it jump ahead of seq=1, which is still not ready,
    even though both deadlines have passed."""
    s = CueScheduler()
    s.schedule(seq=1, frame_t_sim=0.0, deliver_t_sim=1.0)
    s.schedule(seq=2, frame_t_sim=0.1, deliver_t_sim=1.1)
    s.complete(2, payload={"seq": 2, "t_sim": 0.1, "x": 0.0, "y": 0.0, "z": 0.0})

    released = s.poll(1.2)  # both deadlines passed; seq=1 still not ready
    assert released == []
    assert s.n_pending == 2
    assert len(s.floor_violation_log) == 1  # seq=1's violation observed
    assert s.floor_violation_log[0]["seq"] == 1

    s.complete(1, payload={"seq": 1, "t_sim": 0.0, "x": 1.0, "y": 1.0, "z": 1.0})
    released = s.poll(1.2)
    assert [it.seq for it in released] == [1, 2]
    assert released[0].floor_violated is True    # seq=1 WAS blocked past its deadline
    assert released[1].floor_violated is False   # seq=2 was ready well before ITS deadline


def test_cancel_removes_item_and_does_not_log_floor_violated():
    """A detection MISS (cancel()) is not a floor violation -- there is no
    result coming, not "not ready yet". It must not block the queue and
    must not appear in floor_violation_log."""
    s = CueScheduler()
    s.schedule(seq=1, frame_t_sim=0.0, deliver_t_sim=1.0)
    s.schedule(seq=2, frame_t_sim=0.1, deliver_t_sim=1.1)
    s.cancel(1, reason="miss")
    s.complete(2, payload={"seq": 2, "t_sim": 0.1, "x": 0.0, "y": 0.0, "z": 0.0})

    released = s.poll(1.2)
    assert [it.seq for it in released] == [2]
    assert s.floor_violation_log == []
    assert s.cancelled == [(1, "miss")]


# ============================================================================
# (2) t_sim PROVENANCE (ADR-0048 SS3 (1)) -- emitted t_sim is the BAKED
# frame time, never the sim clock's value at release.
# ============================================================================
def test_emitted_t_sim_is_baked_frame_time_not_release_clock():
    s = CueScheduler()
    frame_t_sim = 3.7
    deliver_t_sim = frame_t_sim + DEFAULT_LATENCY_S
    payload = build_cue_payload(5, frame_t_sim, 10.0, 20.0, 0.5)
    s.schedule(seq=5, frame_t_sim=frame_t_sim, deliver_t_sim=deliver_t_sim)
    s.complete(5, payload)

    release_sim_t = 999.999  # deliberately a wildly different "current" sim time
    released = s.poll(release_sim_t)

    assert len(released) == 1
    assert released[0].payload["t_sim"] == frame_t_sim
    assert released[0].payload["t_sim"] != release_sim_t
    assert released[0].frame_t_sim == frame_t_sim


# ============================================================================
# (3) WIRE SCHEMA -- reuses ground_station.triangulate.build_cue_payload
# (the proven :47800 schema), round-trips like m4_intercept.CueReader parses.
# ============================================================================
def test_payload_schema_5key_round_trips():
    payload = build_cue_payload(7, 1.23, 4.0, 5.0, 6.0)
    parsed = json.loads(json.dumps(payload).encode("utf-8").decode("utf-8"))
    assert set(parsed.keys()) == {"seq", "t_sim", "x", "y", "z"}
    assert parsed["seq"] == 7
    assert parsed["t_sim"] == 1.23
    # CueReader's own tolerant parsing: msg.get("vx") is None for this payload
    assert parsed.get("vx") is None
    assert parsed.get("vy") is None
    assert parsed.get("vz") is None


def test_payload_schema_8key_with_velocity_round_trips():
    payload = build_cue_payload(8, 2.0, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
    parsed = json.loads(json.dumps(payload))
    assert set(parsed.keys()) == {"seq", "t_sim", "x", "y", "z", "vx", "vy", "vz"}
    assert parsed["vx"] == pytest.approx(0.1)
    assert parsed["vy"] == pytest.approx(0.2)
    assert parsed["vz"] == pytest.approx(0.3)


# ============================================================================
# (4) --datum-bias-m -- through station.process_frame(), on 3 REAL cached
# capture rows (synthetic noise-free centroids via project_world_to_stereo_
# pixels, the exact inverse of triangulate() -- same round-trip convention
# tests/test_t18_scaffold.py's own datum-bias test uses, but exercised HERE
# through station.py's own frame-processing entry point, not triangulate.py
# directly).
# ============================================================================
class StubDetector:
    """Test double for GroundDetector: `.detect_path(path) -> DetectionResult`
    returns pre-baked results keyed by path. No onnxruntime/cv2 import."""

    def __init__(self, results: dict):
        self.results = results

    def detect_path(self, path):
        return self.results.get(path, DetectionResult(False, None, None))


def _synthetic_stub_detector(rig, rows, capture_dir):
    """Zero-size boxes (u, v, u, v) so box_center() returns exactly (u, v) --
    the noise-free ground-truth-projected centroid for each row's
    gt_target_{x,y,z}, computed by the exact geometric inverse of
    triangulate()."""
    results = {}
    for row in rows:
        gt = (float(row["gt_target_x"]), float(row["gt_target_y"]), float(row["gt_target_z"]))
        uv = project_world_to_stereo_pixels(gt, rig.true_pose, rig.baseline_m, rig.f_px, rig.cx, rig.cy)
        assert uv is not None, f"seq={row['seq']} gt point projects behind a camera -- bad test fixture row"
        u_l, v_l, u_r, v_r = uv
        left_path, right_path = frame_paths(capture_dir, row["seq"])
        results[left_path] = DetectionResult(True, 0.99, (u_l, v_l, u_l, v_l))
        results[right_path] = DetectionResult(True, 0.99, (u_r, v_r, u_r, v_r))
    return StubDetector(results)


def test_datum_bias_shifts_triangulated_position_through_process_frame(rig, capture_rows):
    subset = capture_rows[5:8]  # 3 REAL cached rows from the on-disk T16 capture
    assert len(subset) == 3
    for row in subset:
        left_path, right_path = frame_paths(CAPTURE_DIR, row["seq"])
        assert os.path.exists(left_path) and os.path.exists(right_path), (
            "test fixture expects real cached frames on disk (pixels are not "
            "opened -- centroids are synthesized -- but the paths must be real)"
        )
    detector = _synthetic_stub_detector(rig, subset, CAPTURE_DIR)

    biased_pose = apply_datum_bias(rig.true_pose, 0.5)
    for row in subset:
        res_no_bias = process_frame(row, CAPTURE_DIR, rig, rig.true_pose, detector)
        res_biased = process_frame(row, CAPTURE_DIR, rig, biased_pose, detector)
        assert res_no_bias["status"] == "hit"
        assert res_biased["status"] == "hit"
        shift = math.sqrt(
            (res_biased["x"] - res_no_bias["x"]) ** 2
            + (res_biased["y"] - res_no_bias["y"]) ** 2
            + (res_biased["z"] - res_no_bias["z"]) ** 2
        )
        assert shift == pytest.approx(0.5, abs=1e-6), (
            f"seq={row['seq']}: expected a ~0.5 m constant-vector shift "
            f"(triangulate.py's own proven datum-bias algebra), got {shift:.6f} m"
        )
        # no-bias case round-trips back to gt (zero-jitter synthetic centroids)
        gt = (float(row["gt_target_x"]), float(row["gt_target_y"]), float(row["gt_target_z"]))
        rt = math.sqrt(
            (res_no_bias["x"] - gt[0]) ** 2 + (res_no_bias["y"] - gt[1]) ** 2 + (res_no_bias["z"] - gt[2]) ** 2
        )
        assert rt < 1e-6


def test_detection_miss_yields_status_miss_and_no_wire_payload():
    """A camera miss must never produce an (x, y, z) to emit -- status
    'miss', no triangulation attempted."""
    rows = [{
        "seq": 999, "t_sim_nominal": 0.0, "dash_direction": "r2l",
        "gt_target_x": "1.0", "gt_target_y": "2.0", "gt_target_z": "0.5",
    }]
    detector = StubDetector({})  # every path misses (empty results dict)
    rig = RigConfig.from_capture_meta(os.path.join(CAPTURE_DIR, "capture_meta.json"))
    res = process_frame(rows[0], CAPTURE_DIR, rig, rig.true_pose, detector)
    assert res["status"] == "miss"
    assert "x" not in res


# ============================================================================
# --spoof: T19a documented no-op stub
# ============================================================================
def test_spoof_none_is_a_byte_identical_noop():
    payload = build_cue_payload(1, 0.0, 1.0, 2.0, 3.0)
    assert apply_spoof(payload, None) is payload
    assert apply_spoof(payload, "none") is payload


def test_spoof_unimplemented_mode_raises_loudly():
    payload = build_cue_payload(1, 0.0, 1.0, 2.0, 3.0)
    with pytest.raises(NotImplementedError):
        apply_spoof(payload, "jam_offset")


# ============================================================================
# (5) HONESTY -- station.py holds no gt/pose gz subscription, only /clock.
# ============================================================================
def test_no_gt_or_pose_subscription_in_station_source():
    """AST-based (not raw-string), same idiom as tests/test_honesty_static.py
    -- string-matching "node.subscribe(" would false-positive on this very
    file's own module docstring, which legitimately CROSS-REFERENCES the one
    real call in prose (`make_live_node_and_clock()`'s docstring). Parsing
    finds the one REAL .subscribe(...) call in the AST (docstrings are string
    literals, not Call nodes) and asserts its target is (Clock, CLOCK_TOPIC)
    -- i.e. /clock and nothing else, ever, anywhere in this file."""
    import ast

    with open(STATION_PATH, encoding="utf-8") as f:
        src = f.read()

    # No pose-topic gz message type anywhere (import or otherwise) -- these
    # two are unambiguous regardless of docstring vs code.
    assert "Pose_V" not in src
    assert "pose_v_pb2" not in src

    tree = ast.parse(src, filename=STATION_PATH)
    subscribe_calls = [
        n for n in ast.walk(tree)
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == "subscribe"
    ]
    assert len(subscribe_calls) == 1, (
        f"expected exactly 1 real .subscribe(...) call in the AST (docstring "
        f"mentions don't count), found {len(subscribe_calls)}"
    )
    call = subscribe_calls[0]
    arg_names = [a.id for a in call.args if isinstance(a, ast.Name)]
    assert "Clock" in arg_names and "CLOCK_TOPIC" in arg_names, (
        f"the one real subscribe() call must target (Clock, CLOCK_TOPIC, ...), "
        f"got positional arg names {arg_names}"
    )
    assert 'CLOCK_TOPIC = "/clock"' in src


def test_station_module_imports_without_onnxruntime_or_cv2():
    """The whole point of the deferred-import design (module docstring's
    interpreter note): `import ground_station.station` must not force-import
    onnxruntime/cv2 (the main `.venv` this test suite runs under has neither
    -- this whole test file's own successful COLLECTION already proves that
    in-process, but a same-process `sys.modules` check would be fragile and
    order-dependent here: `tests/test_ekf_tracker.py` legitimately
    `import m4_intercept`, which DOES pull in gz-transport13 (present in
    main `.venv` -- gz was never the concern, onnxruntime/cv2 are), so
    whether 'gz' is already in `sys.modules` depends on which OTHER test
    files ran first in this session, not on this module's own import
    behavior. A fresh subprocess sidesteps that entirely and is the actual
    load-bearing claim: a brand-new interpreter that imports
    ground_station.station never touches onnxruntime/cv2."""
    import subprocess

    code = (
        "import sys; sys.path.insert(0, %r); "
        "import ground_station.station; import ground_station.detect; "
        "assert 'onnxruntime' not in sys.modules, sys.modules.keys(); "
        "assert 'cv2' not in sys.modules, sys.modules.keys(); "
        "print('OK')" % SCRIPTS
    )
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, f"stdout={result.stdout!r} stderr={result.stderr!r}"
    assert result.stdout.strip() == "OK"


# ============================================================================
# (6) REAL DETECTOR on 2 real cached frames (optional -- importorskip'd;
# SKIPPED under main .venv by design, see module docstring).
# ============================================================================
def test_real_detector_matches_t18_reference_centroids():
    pytest.importorskip("onnxruntime")
    pytest.importorskip("cv2")
    from ground_station.detect import GroundDetector

    det = GroundDetector()
    left_path, right_path = frame_paths(CAPTURE_DIR, 15)
    dl = det.detect_path(left_path)
    dr = det.detect_path(right_path)
    assert dl.hit and dr.hit
    u_l, v_l = box_center(dl.box_xyxy)
    u_r, v_r = box_center(dr.box_xyxy)

    # Reference: logs/t18_nn_sigma_validation/nn_detections_per_frame.csv,
    # seq=15 (ultralytics-derived, .venv-seeker-train run) -- generous 1 px
    # tolerance for the letterbox-resize implementation-detail difference
    # documented in detect.py's module docstring.
    assert u_l == pytest.approx(456.7567, abs=1.0)
    assert v_l == pytest.approx(644.3714, abs=1.0)
    assert u_r == pytest.approx(388.3046, abs=1.0)
    assert v_r == pytest.approx(644.2438, abs=1.0)
