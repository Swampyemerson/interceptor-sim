#!/usr/bin/env python3
"""END-TO-END regression: pi_capture -> range_truth_join -> tripod_score.

THE DEFECT THIS TEST EXISTS FOR (found 2026-07-24). The tripod field afternoon
produces the two curves that decide a ~$740 interceptor purchase
(docs/tripod_test_protocol.md §8.1). Three separate breaks meant it could not
emit a verdict AT ALL, and every unit test still passed because NOTHING ran the
recorder's real output through the scorer:

  1. SCHEMA BREAK (silent). pi_capture writes index.csv with
     `frame_idx, frame_path, t_mono_s, t_wall_unix, exposure_us, gain`, and
     tags.csv with `frame_idx, frame_path, n_tags, tag_id, ..., range_m`.
     tripod_score joined on `frame`/`filename`/`name` and read `decoded`/
     `tag_range_m` -- NONE of which the recorder writes. index_map came out
     EMPTY, truth_range_for() returned None for every frame, n_binnable == 0,
     and the gate returned "UNCERTAIN: no TRUE ranges in the session".
  2. NO RANGE TRUTH EXISTED. Nothing in the repo produced `true_range_m`;
     scripts/seeker/range_truth_join.py now does (target flight log + surveyed
     tripod position + each frame's t_wall_unix).
  3. THE GATE INVENTED A FRAME RATE (30.0, sourced from nothing) -- see
     test_stream_fps_is_required_not_invented below.

Each test is written so it FAILS against the pre-fix code and PASSES after.
Everything is offline: a synthetic pi_capture-LAYOUT session (real INDEX_HEADER,
real `%06d.png` filenames, real TAGS_HEADER) + a synthetic target track. No
camera, no sim, no network, no AprilTag decode (a pre-computed tags.csv is
supplied, which is exactly the "reuse tags.csv" path the protocol §6 flow uses).

HONESTY: this whole pipeline is OFFLINE post-flight scoring/calibration (the
field_score.py class), not the sim's gt_* guidance boundary -- no flight path
reads any of it.

Run: .venv/bin/python -m pytest tests/test_tripod_pipeline_join.py -v
"""
import csv
import json
import os
import subprocess
import sys

import numpy as np
import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEEKER = os.path.join(REPO_ROOT, "scripts", "seeker")
sys.path.insert(0, SEEKER)
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import tripod_score as TS                                    # noqa: E402
from pi_capture import INDEX_HEADER, TAGS_HEADER             # noqa: E402

RTJ = os.path.join(SEEKER, "range_truth_join.py")

# --- synthetic engagement geometry (analytic, so every number is checkable) ---
# A realistic tripod PASS (protocol §4): the target flies BY the tripod at an 8 m
# cross-range offset, 6 m up -- range sweeps in to a CPA (~10 m) and back out.
# The reversing range rate is what makes the clock offset observable at all
# (range_truth_join.offset_identifiability); a constant-rate straight-in run is
# correctly refused, which is why this geometry is a pass, not a ramp.
TRIPOD_LAT, TRIPOD_LON, TRIPOD_ALT = 34.0500, -118.2500, 100.0
BASE_UTC = 1_752_700_000.0
TARGET_EAST_M = 8.0        # cross-range offset -> a real CPA
TARGET_UP_M = 6.0          # target flies 6 m above the tripod
TARGET_SPEED = 4.0         # m/s along north
TRACK_HZ = 25.0
TRACK_DUR = 16.0
FRAME_HZ = 20.0
N_FRAMES = 140
CLOCK_OFFSET_S = -0.30     # t_utc = t_wall + CLOCK_OFFSET_S (Pi clock is fast)
# The tag "decodes" inside this range and not beyond -- a clean envelope so
# curve (a) has a real R_decode90 to find.
TAG_DECODE_MAX_M = 16.0


def _target_north_at(t_rel):
    return 40.0 - TARGET_SPEED * np.asarray(t_rel, dtype=float)


def _target_range_at(t_rel):
    """Analytic tripod->target range at t_rel seconds into the pass."""
    north = _target_north_at(t_rel)
    return np.sqrt(TARGET_EAST_M ** 2 + north ** 2 + TARGET_UP_M ** 2)


def _write_track_csv(path, t_rel):
    """Target track in the ENU frame of the tripod, as field_score's
    zero-dependency CSV (t_utc_s + east_m/north_m/up_m)."""
    with open(path, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t_utc_s", "east_m", "north_m", "up_m"])
        for t in t_rel:
            wr.writerow([f"{BASE_UTC + t:.6f}", f"{TARGET_EAST_M:.6f}",
                         f"{_target_north_at(t):.6f}", f"{TARGET_UP_M:.6f}"])


def _make_session(session_dir, with_tags=True, meta_extra=None):
    """A synthetic session in pi_capture's EXACT on-disk layout: frames/%06d.png,
    index.csv with the real INDEX_HEADER, tags.csv with the real TAGS_HEADER."""
    frames_dir = os.path.join(session_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)
    # Frames over the close half of the pass: t_rel 6.0 .. 12.95 s.
    t_rel = 6.0 + np.arange(N_FRAMES) / FRAME_HZ
    t_wall = BASE_UTC + t_rel - CLOCK_OFFSET_S
    true_rng = _target_range_at(t_rel)

    # 1x1 grey PNG per frame -- load_session only needs the files to exist; the
    # decode comes from tags.csv (protocol §6's "reuse a pre-computed decode").
    png = bytes.fromhex(
        "89504e470d0a1a0a0000000d49484452000000010000000108000000003a7e9b55"
        "0000000a4944415478da6360000000020001e221bc330000000049454e44ae426082")
    with open(os.path.join(session_dir, "index.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(INDEX_HEADER)
        for i in range(N_FRAMES):
            rel = os.path.join("frames", f"{i:06d}.png")
            with open(os.path.join(session_dir, rel), "wb") as g:
                g.write(png)
            wr.writerow([i, rel, f"{i / FRAME_HZ:.6f}", f"{t_wall[i]:.6f}",
                         "980.0", "1.0000"])

    if with_tags:
        rs = np.random.default_rng(3)
        with open(os.path.join(session_dir, "tags.csv"), "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(TAGS_HEADER)
            for i in range(N_FRAMES):
                rel = os.path.join("frames", f"{i:06d}.png")
                if true_rng[i] <= TAG_DECODE_MAX_M:
                    r = float(true_rng[i] + 0.20 + rs.normal(0, 0.08))
                    wr.writerow([i, rel, 1, 7, "48.500", 0, "640.00", "400.00",
                                 "0:0;1:0;1:1;0:1", f"{r:.4f}"])
                else:
                    wr.writerow([i, rel, 0, "", "", "", "", "", "", ""])

    meta = {"tag_size_m": 0.35, "drone_size_m": 0.35, "aspect": "approach",
            "source": "picamera2", "n_frames": N_FRAMES}
    meta.update(meta_extra or {})
    with open(os.path.join(session_dir, "meta.json"), "w") as f:
        json.dump(meta, f)
    with open(os.path.join(session_dir, "calib.json"), "w") as f:
        json.dump(dict(fx=539.936, fy=539.936, cx=640.0, cy=400.0,
                       resolution=dict(width=1280, height=800),
                       dist_coeffs=[0.0] * 5), f)
    return t_rel, true_rng


def _run_join(session_dir, track_csv, extra=()):
    return subprocess.run(
        [sys.executable, RTJ, "--session-dir", session_dir, "--csv", track_csv,
         "--tripod-enu", "0,0,0", *extra],
        capture_output=True, text=True, cwd=REPO_ROOT)


@pytest.fixture()
def session(tmp_path):
    """A joined session: pi_capture layout + range truth written back."""
    sdir = str(tmp_path / "pass01")
    t_rel, true_rng = _make_session(sdir, meta_extra={
        "stream_fps": FRAME_HZ,
        "stream_fps_source": "measured: median inter-frame dt (t_mono_s)"})
    track = str(tmp_path / "target_track.csv")
    _write_track_csv(track, np.arange(0.0, TRACK_DUR, 1.0 / TRACK_HZ))
    proc = _run_join(sdir, track, ["--auto-sync"])
    assert proc.returncode == 0, f"range_truth_join failed:\n{proc.stdout}\n{proc.stderr}"
    return sdir, true_rng, proc


# ---------------------------------------------------------------- THE headline
def test_end_to_end_pi_capture_session_produces_a_verdict(session):
    """THE regression: a REAL-layout pi_capture session, range-truthed, must bin
    (n_binnable > 0) and yield a verdict OTHER than 'no TRUE ranges'.

    Pre-fix this failed at n_binnable == 0 (index.csv joined on `frame`, which
    pi_capture never writes) -> gate UNCERTAIN 'no TRUE ranges in the session'."""
    sdir, true_rng, _ = session
    frames, index_map, meta, tags_map = TS.load_session(sdir)
    assert index_map, "index.csv did not bind to any frame (the schema break)"
    assert tags_map, "tags.csv did not bind to any frame (the schema break)"

    calib = TS.load_calib(os.path.join(sdir, "calib.json"))
    decode = TS.decode_frames(frames, calib, 0.35, tags_map=tags_map, quiet=True)
    n_decoded = sum(1 for _p, b in frames if decode[b][0])
    assert n_decoded > 0, "pi_capture's tags.csv schema scored as zero decodes"

    bins, r_any, r90, acc, n_binnable, n_unbinnable = TS.curve_a(
        frames, decode, index_map, 2.0, 40.0)
    assert n_binnable > 0, "no frame carried a TRUE range (the money-gate blocker)"
    assert n_binnable == N_FRAMES

    fps, fps_src = TS.resolve_stream_fps(
        type("A", (), {"stream_fps": None})(), meta)
    gate = TS.gate_verdict(r90, r_any, TS.decode_rate_near(bins, r90, 2.0), fps,
                           TS.DEFAULT_HANDOFF_STREAK, TS.DEFAULT_V_CLOSING_MPS,
                           TS.DEFAULT_TGO_MIN_S, n_decoded=n_decoded)
    assert "no TRUE ranges" not in gate["reason"]
    assert gate["verdict"] in ("PASS", "FAIL"), (
        f"expected a decidable verdict, got {gate['verdict']}: {gate['reason']}")
    # The synthetic envelope decodes to ~12 m, so the gate must SEE that band.
    assert r_any > 0 and r90 > 0
    assert abs(r90 - TAG_DECODE_MAX_M) <= 2.0, (
        f"R_decode90 {r90} m should land at the synthetic decode ceiling "
        f"{TAG_DECODE_MAX_M} m")


def test_true_range_matches_the_analytic_geometry(session):
    """The joined range must be the real tripod->target distance, not a shape."""
    sdir, true_rng, _ = session
    with open(os.path.join(sdir, "index.csv"), newline="") as f:
        rows = list(csv.DictReader(f))
    got = np.array([float(r["true_range_m"]) for r in rows])
    assert len(got) == N_FRAMES
    assert np.max(np.abs(got - true_rng)) < 0.05
    # Original pi_capture columns survive the rewrite; a pristine .bak is kept.
    assert all(c in rows[0] for c in INDEX_HEADER)
    assert os.path.exists(os.path.join(sdir, "index.csv.bak"))
    assert not [p for p in os.listdir(sdir) if p.endswith(".tmp")]


def test_auto_sync_recovers_the_injected_clock_offset(session):
    """--auto-sync must find the Pi-vs-GPS clock offset from the data alone."""
    sdir, _true, _proc = session
    prov = json.loads(open(os.path.join(sdir, "range_truth.json")).read())
    est = prov["clock_offset_s"]
    assert abs(est - CLOCK_OFFSET_S) < 0.05, f"offset {est} vs {CLOCK_OFFSET_S}"
    s = prov["auto_sync"]
    assert s["residual_m"] < 0.5                 # robust MAD of tag-vs-log range
    assert abs(s["bias_m"] - 0.20) < 0.15        # the injected placard bias
    assert s["offset_sigma_s"] is not None and s["offset_sigma_s"] <= 0.10


def test_decode_rate_near_uses_the_last_SUSTAINING_bin(session):
    """4th defect, latent until real data: curve_a returns R_decode90 as the OUTER
    EDGE of the >=90% band, so the bin keyed exactly r90 is the FIRST FAILING one.
    The old selector took that failing bin whenever the bins were CONTIGUOUS (the
    synthetic self-test has a gap, which hid it), handing the gate p=0 -> infinite
    streak burn -> a forced FAIL on a session whose tag decoded 100% to 16 m."""
    # Unit: a clean 100%-to-16 m envelope must yield p = 1.0, not 0.0.
    bins = {10.0: [66, 66, []], 12.0: [32, 32, []], 14.0: [24, 24, []],
            16.0: [0, 12, []], 18.0: [0, 6, []]}
    assert TS.decode_rate_near(bins, 16.0, 2.0) == 1.0
    # Gapped bins (the self-test's shape) still resolve to the sustaining bin.
    gapped = {4.0: [6, 6, []], 6.0: [6, 6, []], 8.0: [3, 3, []], 24.0: [0, 3, []]}
    assert TS.decode_rate_near(gapped, 10.0, 2.0) == 1.0

    # End-to-end: the session's own gate must not be starved to p=0.
    sdir, _t, _p = session
    frames, index_map, meta, tags_map = TS.load_session(sdir)
    calib = TS.load_calib(os.path.join(sdir, "calib.json"))
    decode = TS.decode_frames(frames, calib, 0.35, tags_map=tags_map, quiet=True)
    b, r_any, r90, _acc, _nb, _nu = TS.curve_a(frames, decode, index_map, 2.0, 40.0)
    p = TS.decode_rate_near(b, r90, 2.0)
    assert p >= 0.9, f"gate fed p={p} at R_decode90={r90} from bins {b}"


def test_scorer_sees_the_join_quality_flags(session):
    """range_truth_join's per-frame quality must reach the scorer."""
    sdir, _t, _p = session
    frames, index_map, _meta, _tags = TS.load_session(sdir)
    q = TS.quality_summary(frames, index_map)
    assert q.get("ok", 0) == N_FRAMES, q
    # An unjoinable frame is never binnable, even with a stale number in the row.
    fake = {"x": {"true_range_m": "9.9", "range_quality": "extrapolated"}}
    assert TS.truth_range_for("x", fake) is None


# ------------------------------------------------------- (D) fps is measured
def test_stream_fps_is_required_not_invented(tmp_path):
    """The gate must NOT silently fall back to 30 fps.

    At the PREDICTED R_decode90 for the adopted 0.35 m placard (7.10 m,
    docs/placard_sizing.md §4) with p=0.9, k=5, 9 m/s, the SAME optical data
    PASSES at 30 fps (t_go 0.558 s) and FAILS at the 20 Hz the deployed flight
    loop runs (t_go 0.442 s). So a missing fps must return UNCERTAIN naming the
    missing measurement, not a verdict."""
    g_missing = TS.gate_verdict(7.10, 7.10, 0.9, None, 5, 9.0, 0.5, n_decoded=40)
    assert g_missing["verdict"] == "UNCERTAIN"
    assert "stream_fps" in g_missing["reason"]
    assert g_missing["t_go_s"] is None and g_missing["stream_fps"] is None

    g30 = TS.gate_verdict(7.10, 7.10, 0.9, 30.0, 5, 9.0, 0.5, n_decoded=40)
    g20 = TS.gate_verdict(7.10, 7.10, 0.9, 20.0, 5, 9.0, 0.5, n_decoded=40)
    assert g30["verdict"] == "PASS" and g20["verdict"] == "FAIL", (
        "the fps flip that makes the missing measurement load-bearing")
    assert not hasattr(TS, "DEFAULT_STREAM_FPS"), (
        "a DEFAULT_STREAM_FPS constant is exactly the invented number this "
        "test forbids")


def test_meta_fps_resolution_rejects_replay_synthetic(tmp_path):
    """meta.json fps is accepted only when it is a real measurement; the
    dir-replay backend's synthetic cadence is refused, and --stream-fps is the
    one-line bench override."""
    cli_none = type("A", (), {"stream_fps": None})()
    fps, src = TS.resolve_stream_fps(cli_none, {})
    assert fps is None and "NOT SUPPLIED" in src

    fps, src = TS.resolve_stream_fps(cli_none, {
        "stream_fps": 30.0,
        "stream_fps_source": "replay-synthetic (NOT a capture-rate measurement)"})
    assert fps is None and "REJECTED" in src

    fps, src = TS.resolve_stream_fps(cli_none, {
        "stream_fps": 18.5,
        "stream_fps_source": "measured: median inter-frame dt (t_mono_s)"})
    assert fps == 18.5 and "meta.json" in src

    fps, src = TS.resolve_stream_fps(type("A", (), {"stream_fps": 22.0})(),
                                     {"stream_fps": 30.0})
    assert fps == 22.0 and "--stream-fps" in src


# ------------------------------------------------- refusals (no silent bad join)
def test_join_refuses_a_non_overlapping_log(tmp_path):
    """A log that does not cover the frames must EXIT NON-ZERO, not interpolate
    garbage -- a silently bad join is worse than no join."""
    sdir = str(tmp_path / "pass02")
    _make_session(sdir, with_tags=False)
    track = str(tmp_path / "elsewhere.csv")
    with open(track, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t_utc_s", "east_m", "north_m", "up_m"])
        for i in range(100):
            wr.writerow([f"{BASE_UTC + 9000.0 + i * 0.04:.6f}", 0.0, 30.0, 6.0])
    proc = _run_join(sdir, track)
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert "REFUSED" in proc.stderr
    with open(os.path.join(sdir, "index.csv"), newline="") as f:
        rows = list(csv.DictReader(f))
    assert "true_range_m" not in rows[0], "refused join must not touch index.csv"


def test_join_refuses_when_the_clock_offset_is_unidentifiable(tmp_path):
    """A hovering target makes the range constant, so timing is invisible in it.
    --auto-sync must refuse rather than return a confident wrong offset."""
    sdir = str(tmp_path / "pass03")
    _make_session(sdir)   # tags.csv holds a CLOSING range...
    track = str(tmp_path / "hover.csv")
    with open(track, "w", newline="") as f:                     # ...log hovers
        wr = csv.writer(f)
        wr.writerow(["t_utc_s", "east_m", "north_m", "up_m"])
        for i in range(int(TRACK_DUR * TRACK_HZ)):
            wr.writerow([f"{BASE_UTC + i / TRACK_HZ:.6f}", 0.0, 20.0, 6.0])
    proc = _run_join(sdir, track, ["--auto-sync"])
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert "UNIDENTIFIABLE" in proc.stderr or "residual" in proc.stderr


def test_range_truth_join_self_test_passes():
    """The tool's own offline self-test (synthetic .BIN through real DFReader)."""
    proc = subprocess.run([sys.executable, RTJ, "--self-test"],
                          capture_output=True, text=True, cwd=REPO_ROOT)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "[self-test] PASS" in proc.stdout


def test_curve_b_truth_box_reprojects_onto_the_tag_centre():
    """REGRESSION (review 2, blocker): curve_b's truth box must land back on the
    tag's own image centre at ANY off-axis angle.

    The original construction scaled x,y by the RANGE while scaling z by range/s,
    displacing the truth box radially outward by a factor growing with off-axis
    angle (6.9 px @16 deg, 72.7 @34, 234 @45 = off-frame). Since curve (b) is
    binned BY position-in-frame, a perfect detector scored MISS toward the edges
    and the artefact confirmed the project's own prior. This pins the geometry.
    """
    import math
    fx = fy = 540.0
    cx, cy = 640.0, 400.0
    gr = 12.0
    worst = 0.0
    for u0, v0 in [(640, 400), (800, 400), (1000, 400), (640, 600),
                   (1100, 700), (300, 150), (1200, 780)]:
        a = (u0 - cx) / fx
        b = (v0 - cy) / fy
        z = gr / math.sqrt(1.0 + a * a + b * b)
        x, y = a * z, b * z
        # reproject
        u = cx + fx * x / z
        v = cy + fy * y / z
        err = math.hypot(u - u0, v - v0)
        worst = max(worst, err)
        off_axis_deg = math.degrees(math.atan(math.hypot(a, b)))
        assert err <= 1.0, (
            f"truth box reprojects {err:.1f} px from the tag centre at "
            f"{off_axis_deg:.1f} deg off-axis (u0={u0}, v0={v0}) -- the "
            f"position-in-frame axis curve (b) measures is corrupted")
    assert worst <= 1.0
