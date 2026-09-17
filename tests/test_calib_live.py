"""Offline, no-camera checks for scripts/bench/calib_live.py + the board data
embedded in scripts/bench/calib_pattern.html (docs/bench_specs/tv_calibration.md).

Nothing here touches a real camera or display: detection is exercised on
boards *rendered* by cv2 itself (from calib_live's own board_cells_json()),
poses are synthesized with cv2.projectPoints/solvePnP against KNOWN
parameters, and the web server is hit on an ephemeral loopback port.

THE PRODUCER/CONSUMER SEAM THIS FILE GUARDS (docs/error_handling_policy.md):
`scripts/bench/calib_pattern.html` embeds a JSON literal describing the
ChArUco board's per-cell bit layout; `scripts/bench/calib_live.py`'s
board_cells_json() computes that layout independently from cv2. If the two
ever diverge (a board-shape edit on one side, forgotten on the other), a
photo of the TV would decode WRONG marker ids -- a mis-keyed board is a
silent, structured wrong-answer, not a crash. test_calib_pattern_html_matches_
board_cells_json below is the guard.
"""
import json
import os
import re
import sys
import urllib.request

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_BENCH = os.path.join(REPO, "scripts", "bench")
_SCRIPTS = os.path.join(REPO, "scripts")
_SEEKER = os.path.join(_SCRIPTS, "seeker")
for _p in (REPO, _SCRIPTS, _SEEKER, _BENCH):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import calib_live as cl  # noqa: E402

HTML_PATH = os.path.join(_BENCH, "calib_pattern.html")

pytestmark = pytest.mark.skipif(not cl.HAS_ARUCO,
                                 reason="cv2.aruco not available in this cv2 build")


# --------------------------------------------------------------------------
# Synthetic image helpers -- reconstruct a board image FROM board_cells_json
# (the same data the HTML draws from) and project it into a camera frame via
# a real pinhole homography (exact for a planar target, unlike a hand-faked
# 2-D skew), so detection + pose exercises the real pipeline end to end.
# --------------------------------------------------------------------------

def _render_board_image(spec, cell_px=60):
    n = spec["cellBits"]
    w, h = spec["squaresX"] * cell_px, spec["squaresY"] * cell_px
    img = np.full((h, w), 255, np.uint8)
    marker_total_px = spec["markerLength"] / spec["squareLength"] * cell_px
    bit_px = marker_total_px / n
    margin = (cell_px - marker_total_px) / 2.0
    for cell in spec["cells"]:
        y0, x0 = cell["row"] * cell_px, cell["col"] * cell_px
        if cell["type"] == "black":
            img[y0:y0 + cell_px, x0:x0 + cell_px] = 0
            continue
        bits = np.array(cell["bits"], dtype=np.uint8)
        my0, mx0 = y0 + margin, x0 + margin
        for r in range(n):
            for c in range(n):
                v = 255 if bits[r, c] else 0
                yy0, yy1 = int(my0 + r * bit_px), int(my0 + (r + 1) * bit_px)
                xx0, xx1 = int(mx0 + c * bit_px), int(mx0 + (c + 1) * bit_px)
                img[yy0:yy1, xx0:xx1] = v
    return img


def _project_view(board_img, spec, canvas_w, canvas_h, rvec, tvec, K):
    """Warp the flat board image into `canvas_w x canvas_h` as a real pinhole
    camera at (rvec, tvec, K) would see it -- exact for a planar target: the
    board's 4 outer corners are projected with cv2.projectPoints and the rest
    follows from the perspective homography between the flat image and those
    4 points."""
    bh, bw = board_img.shape[:2]
    sx, sy = spec["squaresX"], spec["squaresY"]
    obj_corners = np.array([[0, 0, 0], [sx, 0, 0], [sx, sy, 0], [0, sy, 0]],
                           dtype=np.float64)
    img_corners, _ = cv2.projectPoints(obj_corners, rvec, tvec, K, None)
    img_corners = img_corners.reshape(-1, 2).astype(np.float32)
    src = np.float32([[0, 0], [bw, 0], [bw, bh], [0, bh]])
    M = cv2.getPerspectiveTransform(src, img_corners)
    canvas = np.full((canvas_h, canvas_w), 235, np.uint8)
    cv2.warpPerspective(board_img, M, (canvas_w, canvas_h), canvas,
                        flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
    return canvas


@pytest.fixture(scope="module")
def board_spec():
    return cl.board_cells_json(1.0)


@pytest.fixture(scope="module")
def board_img(board_spec):
    return _render_board_image(board_spec)


# --------------------------------------------------------------------------
# Producer/consumer contract: calib_pattern.html vs board_cells_json()
# --------------------------------------------------------------------------

def test_calib_pattern_html_matches_board_cells_json(board_spec):
    assert os.path.exists(HTML_PATH), "calib_pattern.html is missing"
    with open(HTML_PATH) as f:
        html = f.read()
    m = re.search(r"var BOARD = (\{.*?\});", html, re.S)
    assert m, "could not find 'var BOARD = {...};' in calib_pattern.html"
    embedded = json.loads(m.group(1))
    assert embedded == board_spec, (
        "calib_pattern.html's embedded board JSON has DRIFTED from "
        "calib_live.board_cells_json() -- regenerate with "
        "'scripts/bench/calib_live.py --emit-board-json' and paste the "
        "result into calib_pattern.html's BOARD literal.")


def test_board_cells_json_shape(board_spec):
    assert len(board_spec["cells"]) == board_spec["squaresX"] * board_spec["squaresY"]
    marker_cells = [c for c in board_spec["cells"] if c["type"] == "marker"]
    black_cells = [c for c in board_spec["cells"] if c["type"] == "black"]
    assert len(marker_cells) + len(black_cells) == len(board_spec["cells"])
    # ids assigned in reading order, sequential from 0 (regression: this is
    # what makes calib_live's independently-constructed CharucoBoard object
    # match the SAME ids at the SAME cells the HTML draws).
    assert [c["id"] for c in marker_cells] == list(range(len(marker_cells)))
    for c in marker_cells:
        bits = np.array(c["bits"])
        assert bits.shape == (board_spec["cellBits"], board_spec["cellBits"])


# --------------------------------------------------------------------------
# Region / tilt classification
# --------------------------------------------------------------------------

@pytest.mark.parametrize("u,v,w,h,expect", [
    (0, 0, 1280, 800, 0),                 # top-left corner
    (1279, 0, 1280, 800, cl.REGION_COLS - 1),  # top-right corner
    (0, 799, 1280, 800, (cl.REGION_ROWS - 1) * cl.REGION_COLS),  # bottom-left
    (800, 400, 1280, 800, 1 * cl.REGION_COLS + 2),  # interior point
])
def test_classify_region(u, v, w, h, expect):
    assert cl.classify_region(u, v, w, h) == expect


def test_classify_tilt_bins():
    lo, hi = cl.TILT_THRESH_DEG
    assert cl.classify_tilt(0.0) == 0
    assert cl.classify_tilt(lo - 0.1) == 0
    assert cl.classify_tilt(lo + 0.1) == 1
    assert cl.classify_tilt(hi - 0.1) == 1
    assert cl.classify_tilt(hi + 0.1) == 2


def test_estimate_tilt_deg_analytic(board_spec):
    board = cl.CharucoBoardAdapter(square_mm=40.0).board
    objp = board.getChessboardCorners()
    K = cl.approx_k_guess(1280, 800)
    img_flat, _ = cv2.projectPoints(objp, np.zeros(3), np.array([0.0, 0.0, 5.0]), K, None)
    tilt_flat = cl.estimate_tilt_deg(objp, img_flat.reshape(-1, 2), K)
    assert tilt_flat is not None and tilt_flat < 1.0

    rvec = np.array([np.radians(50.0), 0.0, 0.0])
    img_steep, _ = cv2.projectPoints(objp, rvec, np.array([0.0, 0.0, 5.0]), K, None)
    tilt_steep = cl.estimate_tilt_deg(objp, img_steep.reshape(-1, 2), K)
    assert tilt_steep is not None and abs(tilt_steep - 50.0) < 1.0


# --------------------------------------------------------------------------
# Detection on rendered / warped synthetic frames
# --------------------------------------------------------------------------

def test_charuco_adapter_detects_frontal_render(board_img):
    adapter = cl.CharucoBoardAdapter(square_mm=40.0)
    canvas = np.full((800, 1280), 235, np.uint8)
    bh, bw = board_img.shape
    scale = 1.5
    dw, dh = int(bw * scale), int(bh * scale)
    resized = cv2.resize(board_img, (dw, dh), interpolation=cv2.INTER_NEAREST)
    y0, x0 = (800 - dh) // 2, (1280 - dw) // 2
    canvas[y0:y0 + dh, x0:x0 + dw] = resized
    det = adapter.detect(canvas)
    assert det is not None
    assert len(det.obj_pts) == adapter.full_n_points  # fully in frame -> all 24


def test_checkerboard_adapter_fallback_detects():
    from calibrate_camera import _object_points  # noqa: F401  (already reused by calib_live)
    cols, rows, sq_px = 9, 6, 40
    board = np.full((rows + 2) * sq_px, dtype=np.uint8, fill_value=255).reshape(-1, 1)
    # build a simple (cols+1)x(rows+1)-square checkerboard with a white margin
    w_sq, h_sq = cols + 1, rows + 1
    img = np.full(((h_sq + 2) * sq_px, (w_sq + 2) * sq_px), 255, np.uint8)
    for r in range(h_sq):
        for c in range(w_sq):
            if (r + c) % 2 == 0:
                y0, x0 = (r + 1) * sq_px, (c + 1) * sq_px
                img[y0:y0 + sq_px, x0:x0 + sq_px] = 0
    canvas = np.full((800, 1280), 235, np.uint8)
    ch, cw = img.shape
    y0, x0 = (800 - ch) // 2, (1280 - cw) // 2
    canvas[y0:y0 + ch, x0:x0 + cw] = img
    adapter = cl.CheckerboardAdapter(square_mm=25.0)
    det = adapter.detect(canvas)
    assert det is not None
    assert len(det.obj_pts) == cols * rows


def test_project_view_warped_board_still_detects(board_img, board_spec):
    """A genuinely tilted synthetic view (real pinhole homography, not a
    hand-faked 2-D skew) is still detected, and its recovered tilt lands in
    the MEDIUM/STEEP bucket rather than LEVEL."""
    adapter = cl.CharucoBoardAdapter(square_mm=40.0)
    k_guess = cl.approx_k_guess(1280, 800)
    K_synth = np.array([[750.0, 0, 640.0], [0, 750.0, 400.0], [0, 0, 1.0]])
    rvec = np.array([0.6, 0.1, 0.0])
    tvec = np.array([-3.5, -2.5, 6.0])
    canvas = _project_view(board_img, board_spec, 1280, 800, rvec, tvec, K_synth)
    det = adapter.detect(canvas)
    assert det is not None
    tilt_deg = cl.estimate_tilt_deg(det.obj_pts, det.img_pts, k_guess)
    assert tilt_deg is not None
    assert cl.classify_tilt(tilt_deg) in (1, 2)


# --------------------------------------------------------------------------
# Sharpness gate
# --------------------------------------------------------------------------

def test_sharpness_score_rejects_blur(board_img):
    canvas = np.full((800, 1280), 235, np.uint8)
    bh, bw = board_img.shape
    canvas[100:100 + bh, 100:100 + bw] = board_img
    bbox = (100, 100, 100 + bw, 100 + bh)
    sharp = cl.sharpness_score(canvas, bbox)
    blurred = cv2.GaussianBlur(canvas, (25, 25), 8.0)
    blurred_score = cl.sharpness_score(blurred, bbox)
    assert sharp > blurred_score * 5
    assert blurred_score < cl.DEFAULT_MIN_SHARPNESS <= sharp


# --------------------------------------------------------------------------
# consider_frame: keep / reject / novelty-cap
# --------------------------------------------------------------------------

def test_consider_frame_caps_redundant_views(board_img, board_spec):
    adapter = cl.CharucoBoardAdapter(square_mm=40.0)
    k_guess = cl.approx_k_guess(1280, 800)
    K_synth = np.array([[750.0, 0, 640.0], [0, 750.0, 400.0], [0, 0, 1.0]])
    rvec = np.zeros(3)
    tvec = np.array([-3.5, -2.5, 6.0])  # centers the 7x5-unit board on-axis
    canvas = _project_view(board_img, board_spec, 1280, 800, rvec, tvec, K_synth)
    cov = cl.Coverage()
    outcomes = [cl.consider_frame(canvas, adapter, cov, k_guess, min_sharpness=1.0,
                                  max_per_bucket=2)
               for _ in range(4)]
    assert [o["kept"] for o in outcomes] == [True, True, False, False]
    assert outcomes[-1]["reason"] == "pose too similar to views already kept"
    assert cov.saved == 2


def test_consider_frame_rejects_blur(board_img, board_spec):
    adapter = cl.CharucoBoardAdapter(square_mm=40.0)
    k_guess = cl.approx_k_guess(1280, 800)
    K_synth = np.array([[750.0, 0, 640.0], [0, 750.0, 400.0], [0, 0, 1.0]])
    canvas = _project_view(board_img, board_spec, 1280, 800, np.zeros(3),
                           np.array([-3.5, -2.5, 6.0]), K_synth)
    # Mild enough blur that the board is still DETECTED (found=True) but its
    # Laplacian-variance score drops below DEFAULT_MIN_SHARPNESS -- a heavier
    # blur (e.g. ksize=25) kills detection outright and would not exercise
    # the blur gate at all.
    blurred = cv2.GaussianBlur(canvas, (9, 9), 4.0)
    cov = cl.Coverage()
    result = cl.consider_frame(blurred, adapter, cov, k_guess,
                               min_sharpness=cl.DEFAULT_MIN_SHARPNESS)
    assert result["found"] is True
    assert result["kept"] is False
    assert "blurry" in result["reason"]
    assert cov.saved == 0 and cov.rejected_blur == 1


# --------------------------------------------------------------------------
# Coverage.is_ready threshold
# --------------------------------------------------------------------------

def test_coverage_is_ready_requires_views_and_union_coverage():
    cov = cl.Coverage()
    dummy_view = lambda: {"obj_pts": np.zeros((4, 3)), "img_pts": np.zeros((4, 2))}
    # enough views, but region 0 never touched -> not ready
    for i in range(cl.MIN_VIEWS + 5):
        region = 1 + (i % (cl.N_REGIONS - 1))
        tilt = i % len(cl.TILT_BINS)
        cov.add(region, tilt, dummy_view())
    assert cov.saved >= cl.MIN_VIEWS
    assert not cov.is_ready()
    assert cov.empty_regions() == [0]
    # fill region 0 -> now ready
    cov.add(0, 0, dummy_view())
    assert cov.is_ready()


def test_coverage_is_ready_requires_all_tilt_bins():
    cov = cl.Coverage()
    dummy_view = lambda: {"obj_pts": np.zeros((4, 3)), "img_pts": np.zeros((4, 2))}
    for i in range(cl.MIN_VIEWS + cl.N_REGIONS):
        cov.add(i % cl.N_REGIONS, 0, dummy_view())  # tilt bin 0 only, ever
    assert cov.saved >= cl.MIN_VIEWS
    assert not cov.empty_regions()
    assert not cov.is_ready(), "tilt bins 1 and 2 were never touched"
    cov.add(0, 1, dummy_view())
    cov.add(0, 2, dummy_view())
    assert cov.is_ready()


# --------------------------------------------------------------------------
# Solvers: recover fx within 1% on synthetic views of KNOWN intrinsics
# --------------------------------------------------------------------------

def test_solve_pinhole_recovers_fx_within_1pct():
    K_true = np.array([[380.0, 0, 640.0], [0, 380.0, 400.0], [0, 0, 1.0]])
    dist_true = np.array([-0.28, 0.08, 0.001, -0.001, 0.0])
    rng = np.random.default_rng(11)
    board_pts = np.zeros((24, 3), np.float64)
    board_pts[:, :2] = np.mgrid[0:6, 0:4].T.reshape(-1, 2).astype(np.float64) * 0.04
    obj_list, img_list = [], []
    for _ in range(28):
        n = rng.integers(8, 24)
        idx = rng.choice(24, size=n, replace=False)
        op = board_pts[idx]
        rvec = rng.uniform(-0.5, 0.5, 3)
        tvec = np.array([rng.uniform(-0.15, 0.15), rng.uniform(-0.15, 0.15),
                         rng.uniform(0.4, 0.9)])
        ip, _ = cv2.projectPoints(op.reshape(-1, 1, 3), rvec, tvec, K_true, dist_true)
        obj_list.append(op)
        img_list.append(ip.reshape(-1, 2))
    rms, K, dist, n_used = cl.solve_pinhole(obj_list, img_list, (1280, 800))
    assert abs(K[0, 0] - K_true[0, 0]) / K_true[0, 0] < 0.01
    assert abs(K[1, 1] - K_true[1, 1]) / K_true[1, 1] < 0.01
    assert rms < 1.0
    assert n_used == len(obj_list)


def test_solve_pinhole_retries_after_dropping_sparse_views_on_cv2_error(monkeypatch):
    """cv2.calibrateCamera can raise a raw `initIntrinsicParams2D` assertion
    when one view's points are too sparse/near-collinear (see solve_pinhole's
    docstring) -- a real failure mode reached by this repo's own synthetic
    test corpus, not a hypothetical. This monkeypatches cv2.calibrateCamera
    to fail once (deterministically) so the retry-and-report path is
    exercised without depending on reproducing OpenCV's exact degenerate
    geometry."""
    board_pts = np.zeros((24, 3), np.float64)
    board_pts[:, :2] = np.mgrid[0:6, 0:4].T.reshape(-1, 2).astype(np.float64) * 0.04
    obj_list = [board_pts] * 20 + [board_pts[:cl.MIN_POINTS_PER_VIEW]] * 3
    img_list = [board_pts[:, :2] * 100 + np.array([300, 200])] * 20 + \
               [board_pts[:cl.MIN_POINTS_PER_VIEW, :2] * 100 + np.array([300, 200])] * 3

    real_calibrate = cv2.calibrateCamera
    calls = []

    def flaky_calibrate(objp, imgp, *a, **k):
        calls.append(len(objp))
        if len(calls) == 1:
            raise cv2.error("synthetic initIntrinsicParams2D failure")
        return real_calibrate(objp, imgp, *a, **k)

    monkeypatch.setattr(cl.cv2, "calibrateCamera", flaky_calibrate)
    rms, K, dist, n_used = cl.solve_pinhole(obj_list, img_list, (1280, 800))
    assert len(calls) == 2, "expected exactly one retry after the first failure"
    assert calls[0] == 23          # the full (unfiltered) view count
    assert n_used == 20            # the 3 MIN_POINTS_PER_VIEW-sized views were dropped
    assert calls[1] == n_used


def test_solve_fisheye_recovers_fx_within_1pct():
    K_true = np.array([[380.0, 0, 640.0], [0, 380.0, 400.0], [0, 0, 1.0]])
    D_true = np.array([0.05, -0.01, 0.001, -0.0002])
    rng = np.random.default_rng(12)
    board_pts = np.zeros((24, 3), np.float64)
    board_pts[:, :2] = np.mgrid[0:6, 0:4].T.reshape(-1, 2).astype(np.float64) * 0.04
    obj_list, img_list = [], []
    tries = 0
    while len(obj_list) < 25 and tries < 300:
        tries += 1
        rvec = rng.uniform(-0.6, 0.6, 3)
        tvec = np.array([rng.uniform(-0.15, 0.15), rng.uniform(-0.15, 0.15),
                         rng.uniform(0.35, 0.9)])
        ip, _ = cv2.fisheye.projectPoints(board_pts.reshape(-1, 1, 3), rvec, tvec,
                                          K_true, D_true)
        pts = ip.reshape(-1, 2)
        if (pts[:, 0].min() < 0 or pts[:, 0].max() > 1280
                or pts[:, 1].min() < 0 or pts[:, 1].max() > 800):
            continue
        obj_list.append(board_pts)
        img_list.append(pts)
    assert len(obj_list) >= 25, "synthetic search failed to build enough in-frame views"
    fe = cl.solve_fisheye(obj_list, img_list, (1280, 800), 24)
    assert fe is not None
    rms, K, _D, n_full = fe
    assert n_full == 25
    assert abs(K[0, 0] - K_true[0, 0]) / K_true[0, 0] < 0.01
    assert rms < 1.0


def test_solve_fisheye_skips_below_min_full_views():
    board_pts = np.zeros((24, 3), np.float64)
    board_pts[:, :2] = np.mgrid[0:6, 0:4].T.reshape(-1, 2).astype(np.float64) * 0.04
    obj_list = [board_pts[:10]] * 3   # only 3 views, and not even full-count
    img_list = [np.zeros((10, 2))] * 3
    assert cl.solve_fisheye(obj_list, img_list, (1280, 800), 24) is None


# --------------------------------------------------------------------------
# Writer + downstream consumer contract
# --------------------------------------------------------------------------

def test_write_intrinsics_json_matches_calibrate_camera_schema(tmp_path):
    K = np.array([[400.0, 0, 640.0], [0, 400.0, 400.0], [0, 0, 1.0]])
    dist = np.array([-0.2, 0.05, 0.001, -0.001, 0.0])
    out_path = tmp_path / "intrinsics.json"
    out = cl.write_intrinsics_json(str(out_path), K, dist, (1280, 800), 25, 0.42,
                                    source="charuco_tv")
    assert out_path.exists()

    from calibrate_camera import _write_json
    ref_path = tmp_path / "ref.json"
    _write_json(str(ref_path), K, dist, (1280, 800), 25, 0.42)
    with open(ref_path) as f:
        ref = json.load(f)
    assert set(out.keys()) == set(ref.keys()), (
        "calib_live's written schema has drifted from "
        "scripts/calibrate_camera.py's own _write_json")


def test_write_intrinsics_json_loads_into_cameramodel(tmp_path):
    from flight.camera import CameraModel
    K = np.array([[400.0, 0, 640.0], [0, 400.0, 400.0], [0, 0, 1.0]])
    dist = np.array([-0.2, 0.05, 0.001, -0.001, 0.0])
    out_path = tmp_path / "intrinsics.json"
    out = cl.write_intrinsics_json(str(out_path), K, dist, (1280, 800), 25, 0.42,
                                    source="charuco_tv")
    m = CameraModel.from_dict(out)
    assert abs(m.fx - 400.0) < 1e-9
    assert m.has_distortion
    assert m.width == 1280 and m.height == 800
    assert m.source == "charuco_tv"


# --------------------------------------------------------------------------
# Web server
# --------------------------------------------------------------------------

def test_web_server_serves_status_frame_and_index():
    state = cl.SharedState()
    state.set_status({"n_kept": 3, "min_views": 25, "instruction": "test instruction",
                      "regions_filled": 2, "n_regions": 12, "region_counts": [0] * 12,
                      "tilt_counts": [1, 0, 0], "tilt_names": list(cl.TILT_BINS),
                      "rejected_no_board": 0, "rejected_blur": 0, "rejected_redundant": 0,
                      "ready": False, "done": False})
    ok, jpeg = cv2.imencode(".jpg", np.zeros((10, 10), np.uint8))
    state.set_frame(jpeg.tobytes())
    server, _t = cl.start_server(state, "127.0.0.1", 0)
    try:
        port = server.server_address[1]
        base = f"http://127.0.0.1:{port}"
        with urllib.request.urlopen(base + "/status.json", timeout=5) as r:
            assert r.status == 200
            body = json.loads(r.read())
            assert body["instruction"] == "test instruction"
            assert body["n_kept"] == 3
        with urllib.request.urlopen(base + "/frame.jpg", timeout=5) as r:
            assert r.status == 200
            data = r.read()
            assert len(data) > 0
            decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
            assert decoded is not None and decoded.shape == (10, 10)
        with urllib.request.urlopen(base + "/", timeout=5) as r:
            assert r.status == 200
            html = r.read().decode()
            assert "<html" in html.lower()
    finally:
        server.shutdown()


# --------------------------------------------------------------------------
# Full end-to-end: --source dir=PATH reaches READY and writes intrinsics
# --------------------------------------------------------------------------

def _synth_frame_dir(tmp_path, board_img, board_spec, n=220, seed=99):
    """Randomized poses wide enough to genuinely reach all 12 regions (the
    board must sometimes clip a frame edge/corner, and sometimes sit at a
    close distance to reach steep-corner detections) and all 3 tilt bins --
    tuned empirically (2026-09-17) against the real detector + classify_*
    functions, not guessed."""
    rng = np.random.default_rng(seed)
    K_synth = np.array([[750.0, 0, 640.0], [0, 750.0, 400.0], [0, 0, 1.0]])
    d = tmp_path / "frames"
    d.mkdir()
    n_written = 0
    for i in range(n):
        rvec = np.array([rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), 0.0])
        tvec = np.array([rng.uniform(-10, 10), rng.uniform(-7, 7), rng.uniform(1.5, 8.0)])
        canvas = _project_view(board_img, board_spec, 1280, 800, rvec, tvec, K_synth)
        cv2.imwrite(str(d / f"{i:04d}.png"), canvas)
        n_written += 1
    return d, n_written


def test_end_to_end_dir_replay_reaches_ready_and_writes_intrinsics(tmp_path, board_img, board_spec):
    frame_dir, n_written = _synth_frame_dir(tmp_path, board_img, board_spec, n=220)
    out_path = tmp_path / "intrinsics_out.json"
    rc = cl.main([
        "--source", f"dir={frame_dir}",
        "--square-mm", "40.0",
        "--min-sharpness", "1.0",       # crisp renders; decouple from the tuned default
        "--out", str(out_path),
        "--host", "127.0.0.1", "--port", "0",
    ])
    assert rc == 0, (f"end-to-end dir-replay session did not reach READY out of "
                     f"{n_written} synthetic views -- widen the pose sampling "
                     f"in _synth_frame_dir if this becomes flaky")
    assert out_path.exists()
    with open(out_path) as f:
        out = json.load(f)
    assert out["n_views"] >= cl.MIN_VIEWS
    assert out["source"] == "charuco_tv"
    assert out["rms_reproj_error_px"] < 1.0

    from flight.camera import CameraModel
    m = CameraModel.from_dict(out)
    assert m.has_distortion
    assert m.width == 1280 and m.height == 800


def test_end_to_end_dir_replay_not_ready_reports_exit_1(tmp_path, board_img, board_spec):
    """A short/uncovering session (--auto caps it early) reports NOT READY,
    exit 1, and does not fabricate an output file -- the no-vacuous-verdicts
    rule (CLAUDE.md / docs/error_handling_policy.md)."""
    frame_dir, _n = _synth_frame_dir(tmp_path, board_img, board_spec, n=5)
    out_path = tmp_path / "intrinsics_out.json"
    rc = cl.main([
        "--source", f"dir={frame_dir}",
        "--square-mm", "40.0",
        "--min-sharpness", "1.0",
        "--out", str(out_path),
        "--host", "127.0.0.1", "--port", "0",
        "--auto", "5",
    ])
    assert rc == 1
    assert not out_path.exists()
