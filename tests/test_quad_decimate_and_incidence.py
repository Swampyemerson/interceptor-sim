#!/usr/bin/env python3
"""quad_decimate plumbing + tag-INCIDENCE scoring — the two field-day defects the
2026-07-25 what's-left audit ranked #1 and #3.

WHY THESE TESTS EXIST

(1) quad_decimate (audit RANK 1 / ADR-0082). Every decode site in the repo
    constructed `Detector(families="tag36h11")` with NO quad_decimate argument, so
    all of them silently ran the pupil-apriltags default 2.0 — the configuration
    docs/placard_sizing.md predicts FAILS the ~$740 t_go gate at both 20 Hz
    (0.442 s) and 14 Hz (0.294 s) with the adopted 0.35 m placard. ADR-0082
    promoted qd=1.0 to the PLANNED tripod-day setting. Two compounding harms:
    the day would have been captured in the predicted-FAIL config, and because the
    value was never recorded, the frames could not be re-scored afterwards to
    recover the lever — protocol §4.2b's "re-decode at qd ∈ {2.0, 1.0}" was
    unrunnable and would have been LOST WITH THE SESSION.

(2) INCIDENCE (audit RANK 3, "the single largest information gain available from
    the session"). docs/placard_mount.md DECISION 1 committed a beam-facing
    placard, so decode range falls as R(0)·cos θ and a pass at 30° incidence is a
    DIFFERENT EXPERIMENT from one at 0°. `grep -c incidence tripod_score.py` was 0.
    The trap this test guards is the tempting wrong fix: incidence is recovered
    FROM the tag pose, which exists only on DECODED frames, so a decode-RATE vs
    incidence curve built on it has a denominator of "frames that decoded" — it is
    identically 100% and vacuous. The scorer must produce a rate ONLY when a
    per-frame incidence covers the misses too.

Everything here is offline (synthetic frames, no camera, no sim). Fixtures come
from the PRODUCER's own writer (pi_capture.record_session / its TAGS_HEADER), not
from hand-typed CSVs — a hand-typed fixture is exactly what hid the earlier
producer→consumer schema breaks (docs/error_handling_policy.md §6).

Run: .venv/bin/python -m pytest tests/test_quad_decimate_and_incidence.py -v
"""
import argparse
import csv
import json
import math
import os
import sys
import types

import numpy as np
import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEEKER = os.path.join(REPO_ROOT, "scripts", "seeker")
for p in (SEEKER, os.path.join(REPO_ROOT, "scripts")):
    if p not in sys.path:
        sys.path.insert(0, p)

import pi_capture as PC                                        # noqa: E402
import tripod_score as TS                                      # noqa: E402

cv2 = pytest.importorskip("cv2")

# The range at which the decimation actually MATTERS on the repo's synthetic tag
# fixture: an 11.2 px tag (0.5 m at 24 m through synth_tag_frames' K) decodes at
# quad_decimate=1.0 and does NOT decode at 2.0. Measured on this box, identical
# for 4 different backgrounds, so the assertion is about the detector setting and
# not about which background was drawn.
DECIMATE_CLIFF_RANGE_M = 24.0
FIXTURE_TAG_SIZE_M = 0.5


def _synth_frame(range_m, seed=0, off_x_m=0.0):
    """One synthetic frame with the tag composited at a known range."""
    import synth_tag_frames as S
    tag_img = cv2.imread(S.TAG_PNG)
    assert tag_img is not None, f"missing tag texture {S.TAG_PNG}"
    rng = np.random.default_rng(seed)
    bg = S._load_background(rng, cv2)
    res = S.composite_tag(bg, tag_img, cv2, float(range_m), FIXTURE_TAG_SIZE_M,
                          off_x_m=off_x_m)
    assert res is not None, f"composite failed at {range_m} m"
    return res[0], S


def _write_frames(dirpath, ranges, seed=0):
    os.makedirs(dirpath, exist_ok=True)
    S = None
    for i, r in enumerate(ranges):
        frame, S = _synth_frame(r, seed=seed + i)
        cv2.imwrite(os.path.join(dirpath, f"f_{i:03d}_r{r:05.1f}_.png"), frame)
    return S


def _calib_json(path, S):
    with open(path, "w") as f:
        json.dump(dict(fx=S.FX, fy=S.FY, cx=S.CX, cy=S.CY,
                       resolution=dict(width=S.W, height=S.H),
                       dist_coeffs=[0.0] * 5), f)
    return path


def _capture(out_dir, frames_dir, calib_path, quad_decimate):
    """Drive the REAL pi_capture entry point (capture()), which is what the field
    day runs — not a re-implementation of it."""
    ns = argparse.Namespace(
        source=f"dir={frames_dir}", out=out_dir, n_frames=None, replay_fps=30.0,
        device="0", width=1280, height=960, exposure_us=PC.DEF_EXPOSURE_US,
        gain=1.0, duration=None, decode_tags=True, calib=calib_path,
        tag_size=FIXTURE_TAG_SIZE_M, run_tag="qdtest",
        quad_decimate=quad_decimate)
    rc = PC.capture(ns)
    with open(os.path.join(out_dir, "meta.json")) as f:
        meta = json.load(f)
    return rc, meta


# ===========================================================================
# 1. quad_decimate is WIRED (the flag changes the decode), not merely accepted
# ===========================================================================
def test_quad_decimate_actually_changes_the_decode(tmp_path):
    """THE anti-inert-flag test (CLAUDE.md: "a fix isn't done until its EFFECT is
    observed"). A flag that is parsed, stamped and never reaches the detector
    would pass every other test in this file. The same frame must decode at
    qd=1.0 and NOT decode at qd=2.0."""
    frame, _S = _synth_frame(DECIMATE_CLIFF_RANGE_M)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame

    det_default = PC._make_detector()                    # library default 2.0
    det_full = PC._make_detector(1.0)
    n_default = len(det_default.detect(gray))
    n_full = len(det_full.detect(gray))

    assert PC.DEFAULT_QUAD_DECIMATE == 2.0, (
        "the stated default must be the pupil-apriltags library default, so "
        "existing sessions keep their meaning")
    assert n_default == 0 and n_full == 1, (
        f"quad_decimate did not reach the detector: same {DECIMATE_CLIFF_RANGE_M} m "
        f"frame decoded {n_default} tags at qd=2.0 and {n_full} at qd=1.0 "
        f"(expected 0 and 1) -- the flag is INERT")


def test_scorer_decode_frames_honours_quad_decimate(tmp_path):
    """The same effect through tripod_score.decode_frames -- the path the money
    gate's curve (a) is computed on."""
    frames_dir = str(tmp_path / "frames")
    S = _write_frames(frames_dir, [DECIMATE_CLIFF_RANGE_M])
    calib = TS.load_calib(_calib_json(str(tmp_path / "calib.json"), S))
    frames = [(os.path.join(frames_dir, p), os.path.splitext(p)[0])
              for p in sorted(os.listdir(frames_dir))]

    d2 = TS.decode_frames(frames, calib, FIXTURE_TAG_SIZE_M, quiet=True,
                          quad_decimate=2.0)
    d1 = TS.decode_frames(frames, calib, FIXTURE_TAG_SIZE_M, quiet=True,
                          quad_decimate=1.0)
    b = frames[0][1]
    assert TS.as_obs(d2[b]).decoded is False
    assert TS.as_obs(d1[b]).decoded is True, (
        "curve (a) would be scored at the library default regardless of the "
        "requested decimate -- the ~$740 envelope would not be the flown config")


# ===========================================================================
# 2. THE STAMP: every session records what produced it
# ===========================================================================
def test_meta_json_stamps_the_quad_decimate_that_produced_the_session(tmp_path):
    """Without the stamp the §4.2b reclaim lever is lost WITH the session: you
    cannot read a decimation off a PNG afterwards."""
    frames_dir = str(tmp_path / "frames")
    S = _write_frames(frames_dir, [4.0, 5.0])
    calib = _calib_json(str(tmp_path / "calib.json"), S)

    rc_d, meta_d = _capture(str(tmp_path / "s_default"), frames_dir, calib,
                            PC.DEFAULT_QUAD_DECIMATE)
    rc_1, meta_1 = _capture(str(tmp_path / "s_qd1"), frames_dir, calib, 1.0)

    assert rc_d == 0 and rc_1 == 0
    assert meta_d["quad_decimate"] == 2.0
    assert meta_1["quad_decimate"] == 1.0
    # ...and the SOURCE, so "2.0" from a default is distinguishable from "2.0"
    # deliberately chosen.
    assert "DEFAULT" in meta_d["quad_decimate_source"]
    assert "explicit" in meta_1["quad_decimate_source"]
    # The stamp is written even when the decode is off -- it is a property of the
    # session, and an offline re-decode still has to match it.
    ns = argparse.Namespace(
        source=f"dir={frames_dir}", out=str(tmp_path / "s_nodecode"),
        n_frames=None, replay_fps=30.0, device="0", width=1280, height=960,
        exposure_us=PC.DEF_EXPOSURE_US, gain=1.0, duration=None,
        decode_tags=False, calib=None, tag_size=None, run_tag="nodec",
        quad_decimate=1.0)
    PC.capture(ns)
    with open(os.path.join(str(tmp_path / "s_nodecode"), "meta.json")) as f:
        assert json.load(f)["quad_decimate"] == 1.0


# ===========================================================================
# 3. THE MISMATCH REFUSAL (fail-closed, with a named escape hatch)
# ===========================================================================
def test_check_decimate_consistency_refuses_mixed_decimations():
    """Pure function: two RECORDED values that disagree is a refusal; an
    UNRECORDED value is unknown, not 'equal'; the escape hatch must be named in
    the message (a refusal you cannot act on is a dead end at 9 pm)."""
    with pytest.raises(SystemExit) as e:
        TS.check_decimate_consistency({"capture": 2.0, "scoring": 1.0})
    msg = str(e.value)
    assert "MISMATCH" in msg and "--allow-decimate-mismatch" in msg
    assert "stream-fps" in msg or "--stream-fps" in msg, (
        "the refusal must warn that the fps bench belongs to the decimate scored")

    ok = TS.check_decimate_consistency({"capture": 2.0, "scoring": 1.0},
                                       allow_mismatch=True)
    assert ok["mismatch"] and ok["mismatch_allowed"]

    same = TS.check_decimate_consistency({"a": 1.0, "b": 1.0, "c": 1.0})
    assert not same["mismatch"] and same["distinct_recorded"] == [1.0]

    # Aggregating THREE sessions: two at 2.0 and one at 1.0 must not be pooled.
    with pytest.raises(SystemExit):
        TS.check_decimate_consistency({"pass01": 2.0, "pass02": 2.0,
                                       "pass03": 1.0})

    unknown = TS.check_decimate_consistency({"legacy session": None,
                                             "scoring": 2.0})
    assert not unknown["mismatch"]
    assert unknown["unknown_sources"] == ["legacy session"], (
        "an unrecorded decimate must be reported as UNKNOWN, never silently "
        "treated as agreeing")


def _score(session_dir, out_dir, calib, **over):
    a = types.SimpleNamespace(
        session_dir=session_dir, calib=calib, tag_size=None, drone_size=None,
        stream_fps=30.0, closing_speed=None,
        closing_speed_hi=TS.DEFAULT_V_CLOSING_HI_MPS,
        handoff_streak=TS.DEFAULT_HANDOFF_STREAK, tgo_min=TS.DEFAULT_TGO_MIN_S,
        range_bin=2.0, max_range=40.0, pos_bands=3,
        weights="/no/such/weights.onnx", weights_bar=None, no_weights_bar=True,
        nn_conf=TS.DEFAULT_NN_CONF, min_decoded=TS.DEFAULT_MIN_DECODED,
        redecode=True, out_dir=out_dir, tag="t")
    for k, v in over.items():
        setattr(a, k, v)
    TS.run(a)
    with open(os.path.join(out_dir, a.tag, "gate.json")) as f:
        return json.load(f)


def test_scorer_refuses_a_session_whose_stamp_disagrees(tmp_path, capsys):
    """END-TO-END: a session captured at 2.0, re-scored at 1.0, must REFUSE --
    otherwise curve (a) reports a decode envelope produced by a configuration
    that never flew, and the gate reads it as optical truth."""
    frames_dir = str(tmp_path / "frames")
    S = _write_frames(frames_dir, [4.0, 5.0, 6.0])
    calib = _calib_json(str(tmp_path / "calib.json"), S)
    sess = str(tmp_path / "sess")
    _capture(sess, frames_dir, calib, 2.0)

    with pytest.raises(SystemExit) as e:
        _score(sess, str(tmp_path / "out1"), calib, quad_decimate=1.0)
    assert "quad_decimate MISMATCH" in str(e.value)

    gate = _score(sess, str(tmp_path / "out2"), calib, quad_decimate=1.0,
                  allow_decimate_mismatch=True)
    assert gate["quad_decimate"] == 1.0
    assert gate["quad_decimate_check"]["mismatch"] is True
    # ...and the divergence is on the human-readable surface too, not only JSON.
    with open(os.path.join(str(tmp_path / "out2"), "t", "verdict.txt")) as f:
        assert "quad_decimate MISMATCH ACCEPTED" in f.read()

    # A matching re-score is the normal path and records the agreement.
    gate_ok = _score(sess, str(tmp_path / "out3"), calib)
    assert gate_ok["quad_decimate"] == 2.0
    assert gate_ok["quad_decimate_check"]["mismatch"] is False
    assert "meta.json" in gate_ok["quad_decimate_source"]


def test_unrecorded_decimate_is_flagged_not_assumed(tmp_path):
    """A legacy session with no stamp must score (we cannot refuse all history)
    but must SAY the decimation is unknown -- on both the JSON and text surface."""
    frames_dir = str(tmp_path / "frames")
    S = _write_frames(frames_dir, [4.0, 5.0, 6.0])
    calib = _calib_json(str(tmp_path / "calib.json"), S)
    sess = str(tmp_path / "sess")
    _capture(sess, frames_dir, calib, 2.0)
    mp = os.path.join(sess, "meta.json")
    meta = json.loads(open(mp).read())
    del meta["quad_decimate"]                       # simulate a pre-stamp session
    json.dump(meta, open(mp, "w"))

    gate = _score(sess, str(tmp_path / "out"), calib)
    assert gate["quad_decimate"] == TS.DEFAULT_QUAD_DECIMATE
    assert "DEFAULT" in gate["quad_decimate_source"]
    assert gate["quad_decimate_check"]["unknown_sources"] == [
        "session meta.json (capture)"]
    with open(os.path.join(str(tmp_path / "out"), "t", "verdict.txt")) as f:
        assert "quad_decimate UNRECORDED" in f.read()


# ===========================================================================
# 4. INCIDENCE: the geometry primitive
# ===========================================================================
def _rot_y(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_x(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


@pytest.mark.parametrize("R,t,expect", [
    (np.eye(3), (0.0, 0.0, 5.0), 0.0),            # dead-on
    (_rot_y(15.0), (0.0, 0.0, 5.0), 15.0),        # the mount's index step
    (_rot_y(30.0), (0.0, 0.0, 5.0), 30.0),        # the design cone edge
    (_rot_y(-30.0), (0.0, 0.0, 5.0), 30.0),       # sign-symmetric: cost is |θ|
    (_rot_x(45.0), (0.0, 0.0, 5.0), 45.0),        # the other axis behaves the same
    (_rot_y(90.0), (0.0, 0.0, 5.0), 90.0),        # edge-on: the null aspect
])
def test_tag_incidence_on_synthetic_poses(R, t, expect):
    """Incidence is the angle between the tag NORMAL and the LINE OF SIGHT. These
    synthetic poses pin the definition, so a later 'simplification' to the
    off-boresight BEARING (a different quantity that happens to agree on-axis)
    fails here."""
    got = PC.tag_incidence_deg(R, np.asarray(t))
    assert got == pytest.approx(expect, abs=1e-6)


def test_incidence_is_measured_against_the_line_of_sight_not_the_boresight():
    """A tag whose normal is parallel to the camera axis but which sits far
    off-axis is NOT at 0 deg incidence -- it is foreshortened by the viewing
    angle. Getting this wrong would over-report the usable cone."""
    th = PC.tag_incidence_deg(np.eye(3), np.array([5.0, 0.0, 5.0]))
    assert th == pytest.approx(45.0, abs=1e-6)


def test_incidence_returns_none_for_a_degenerate_pose():
    """Unknown must stay unknown: a substituted 0.0 would read as a perfect
    dead-on measurement (error policy §5, typed sentinels)."""
    assert PC.tag_incidence_deg(np.eye(3), np.zeros(3)) is None
    assert PC.tag_incidence_deg(np.zeros((3, 3)), np.array([0, 0, 1.0])) is None


# ===========================================================================
# 5. INCIDENCE: what may be called a RATE, and what may not
# ===========================================================================
def _frames(n):
    return [(f"/x/{i:06d}.png", f"{i:06d}") for i in range(n)]


def test_tag_pose_incidence_gives_an_annotation_never_a_rate():
    """THE trap. Incidence is recovered from the tag pose, which exists only on
    DECODED frames -- so binning decode rate on it yields cells whose denominator
    is 'frames that decoded' (100% by construction). The scorer must refuse to
    call that a curve."""
    fr = _frames(4)
    decode = {
        "000000": TS.TagObs(True, 5.0, 640.0, 400.0, 1, 5.0),
        "000001": TS.TagObs(True, 5.2, 640.0, 400.0, 1, 25.0),
        "000002": TS.TagObs(False, None, None, None, 0, None),   # a MISS: no pose
        "000003": TS.TagObs(False, None, None, None, 0, None),
    }
    index_map = {b: {"true_range_m": "5.0"} for _p, b in fr}
    inc = TS.curve_a_incidence(fr, decode, index_map, 2.0, 40.0)

    assert inc["rate_computable"] is False
    assert inc["cells"] == {}, "no rate cells may be emitted without the misses"
    assert "NOT A RATE CURVE" in inc["note"]
    assert inc["decoded_incidence"]["n"] == 2
    assert inc["decoded_incidence"]["max_deg"] == pytest.approx(25.0)
    assert inc["incidence_source_counts"] == {"index": 0, "tag_pose": 2, "none": 2}


def test_index_supplied_incidence_gives_a_real_rate_curve():
    """With a per-frame incidence for EVERY frame (protocol §4.2b: from the
    target log attitude + the surveyed tripod + the mount index), the misses have
    an incidence too and the rate is a measurement."""
    fr = _frames(4)
    decode = {
        "000000": TS.TagObs(True, 5.0, 640.0, 400.0, 1, 5.0),
        "000001": TS.TagObs(True, 5.2, 640.0, 400.0, 1, 5.0),
        "000002": TS.TagObs(False, None, None, None, 0, None),
        "000003": TS.TagObs(False, None, None, None, 0, None),
    }
    index_map = {
        "000000": {"true_range_m": "5.0", "incidence_deg": "3.0"},
        "000001": {"true_range_m": "5.0", "incidence_deg": "7.0"},
        "000002": {"true_range_m": "5.0", "incidence_deg": "40.0"},
        "000003": {"true_range_m": "5.0", "incidence_deg": "44.0"},
    }
    inc = TS.curve_a_incidence(fr, decode, index_map, 2.0, 40.0)

    assert inc["rate_computable"] is True
    assert inc["cells"] == {(4.0, 0.0): [2, 2], (4.0, 30.0): [0, 2]}, (
        "decode succeeded in the 0-15 deg bin and failed in the 30-45 deg bin -- "
        "the cos-law signal the session exists to measure")
    assert inc["incidence_bin_deg"] == TS.DEFAULT_INCIDENCE_BIN_DEG == 15.0


def test_cells_beyond_the_validated_cos_law_are_flagged(tmp_path):
    """The cos-theta law is MEASURED only to 32.6 deg (sim) and is a HYPOTHESIS
    beyond. A cell past that must be marked in the CSV, so nobody quotes a 45 deg
    number as if it were validated physics."""
    fr = _frames(2)
    decode = {"000000": TS.TagObs(True, 5.0, 1.0, 1.0, 1, 40.0),
              "000001": TS.TagObs(True, 5.0, 1.0, 1.0, 1, 40.0)}
    index_map = {"000000": {"true_range_m": "5.0", "incidence_deg": "40.0"},
                 "000001": {"true_range_m": "5.0", "incidence_deg": "41.0"}}
    inc = TS.curve_a_incidence(fr, decode, index_map, 2.0, 40.0)
    out = str(tmp_path / "inc.csv")
    TS.write_curve_a_incidence_csv(out, inc)
    rows = list(csv.DictReader(open(out)))
    assert rows and rows[0]["beyond_validated_cos_law"] == "1"
    assert rows[0]["rate_is_measured"] == "1"
    assert inc["decoded_incidence"]["n_beyond_validated_cos_law"] == 2


def test_incidence_aware_gate_derates_r90_by_cos_theta():
    """§8.1 in the form placard_mount §11 item 10 asks for. theta_eng=0 must
    reproduce the historical arithmetic EXACTLY (no silent re-baselining of every
    past number), and a real engagement incidence must be able to flip the GO."""
    g0 = TS.gate_verdict(8.0, 8.0, 0.9, 30.0, 5, 9.0, 0.5, n_decoded=40)
    g0_explicit = TS.gate_verdict(8.0, 8.0, 0.9, 30.0, 5, 9.0, 0.5, n_decoded=40,
                                  engagement_incidence_deg=0.0)
    assert g0["t_go_s"] == g0_explicit["t_go_s"] == pytest.approx(0.6577, abs=1e-3)
    assert g0["R_decode90_eff_m"] == 8.0

    g45 = TS.gate_verdict(8.0, 8.0, 0.9, 30.0, 5, 9.0, 0.5, n_decoded=40,
                          engagement_incidence_deg=45.0)
    assert g45["R_decode90_eff_m"] == pytest.approx(8.0 * math.cos(math.radians(45)),
                                                    abs=1e-3)
    assert g45["verdict"] == "FAIL" and g0["verdict"] == "PASS", (
        "the SAME optics must be able to fail at an oblique engagement aspect -- "
        "that is the whole point of the incidence-aware form")
    assert g45["R_decode90_m"] == 8.0, "the MEASURED number must not be overwritten"


def test_dead_on_gate_carries_a_warning_on_the_visible_surface(tmp_path):
    """A GO measured at theta=0 that does not SAY it was measured at theta=0 is
    the reversed-conclusion-in-prose failure mode. verdict.txt must warn."""
    frames_dir = str(tmp_path / "frames")
    S = _write_frames(frames_dir, [4.0, 5.0, 6.0])
    calib = _calib_json(str(tmp_path / "calib.json"), S)
    sess = str(tmp_path / "sess")
    _capture(sess, frames_dir, calib, 2.0)
    _score(sess, str(tmp_path / "out"), calib)
    txt = open(os.path.join(str(tmp_path / "out"), "t", "verdict.txt")).read()
    assert "theta_eng = 0 deg" in txt and "NOT a GO" in txt
    assert "CURVE (a) x INCIDENCE" in txt


# ===========================================================================
# 6. PRODUCER -> CONSUMER: pi_capture's tags.csv is read by tripod_score
# ===========================================================================
def test_tags_csv_incidence_column_survives_the_file_boundary(tmp_path):
    """CONTRACT TEST with a fixture from the PRODUCER's own writer. pi_capture
    writes tags.csv; tripod_score reads it. Both passed their own self-tests while
    the schemas disagreed once before (2026-07-24, `decoded` vs `n_tags`), so the
    incidence column gets a pair test from day one."""
    frames_dir = str(tmp_path / "frames")
    S = _write_frames(frames_dir, [4.0, 5.0])
    calib_path = _calib_json(str(tmp_path / "calib.json"), S)
    sess = str(tmp_path / "sess")
    _capture(sess, frames_dir, calib_path, 2.0)

    # the producer's header is the contract
    assert PC.TAGS_HEADER[-1] == "incidence_deg"
    rows = list(csv.DictReader(open(os.path.join(sess, "tags.csv"))))
    assert rows and all("incidence_deg" in r for r in rows)
    written = [float(r["incidence_deg"]) for r in rows if r["incidence_deg"]]
    assert written, "pi_capture recorded no incidence for a decoded tag"

    # the consumer binds it
    frames, index_map, meta, tags_map = TS.load_session(sess)
    decode = TS.decode_frames(frames, TS.load_calib(calib_path),
                              FIXTURE_TAG_SIZE_M, tags_map=tags_map, quiet=True)
    read_back = [TS.as_obs(v).incidence_deg for v in decode.values()
                 if TS.as_obs(v).decoded]
    assert read_back and all(v is not None for v in read_back), (
        "tripod_score did not read pi_capture's incidence_deg column")
    assert sorted(round(v, 2) for v in read_back) == sorted(
        round(v, 2) for v in written)


def test_legacy_five_tuple_decode_maps_still_bind():
    """Back-compat: callers (and older tests) that hand-build the historical
    5-tuple must keep working, with incidence UNKNOWN rather than 0.0."""
    obs = TS.as_obs((True, 5.0, 640.0, 400.0, 1))
    assert obs.decoded is True and obs.range_m == 5.0 and obs.incidence_deg is None
    assert obs[0] is True and obs[4] == 1          # index access unchanged
    assert TS.as_obs(TS.TagObs(False, None, None, None, 0, 12.0)).incidence_deg == 12.0
