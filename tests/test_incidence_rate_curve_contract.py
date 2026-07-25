#!/usr/bin/env python3
"""PRODUCER -> CONSUMER contract: range_truth_join --incidence  ->  tripod_score's
decode-RATE vs (range x incidence) curve.

THE DEFECT THIS TEST EXISTS FOR (2026-07-25). docs/tripod_test_protocol.md §4.2b
lists two things the scorer can produce, and only ONE of them was reachable:

  | index.csv carries a per-frame incidence for EVERY binnable frame | a real
  | decode-RATE vs (range x incidence) curve                         |
  | only the tag's own pose (the default)                            | an
  | ANNOTATION: the incidence distribution of the decodes            |

The rate row needs an incidence on the MISSES, which the protocol says must come
from the target log's ATTITUDE + the surveyed tripod position + the pass card's
placard index angle. NOTHING in the repo produced that column, so the aspect axis
of the curve that gates the ~$740 Tier-2 order could only ever be an annotation --
and §7.1 item 4 told the operator to score `(range x incidence)` anyway.

`range_truth_join.py --incidence --mount-index-deg` is that producer. This test
runs it FOR REAL (subprocess, on a synthetic ArduPilot .BIN carrying POS *and*
ATT built by field_score's own fixture writer) and then feeds its on-disk output
to tripod_score's OWN curve-(a)-x-incidence code. Both halves already had
self-tests; nothing tested the PAIR, which is exactly how this project's schema
breaks have hidden before (docs/error_handling_policy.md, the contract-test rule).

Everything is offline: no camera, no sim, no network, no AprilTag decode (a
pre-computed tags.csv supplies the decodes, the protocol §6 "reuse the decode"
path). HONESTY: this is post-flight scoring, the field_score class, not the sim's
gt_* guidance boundary.

Run: .venv/bin/python -m pytest tests/test_incidence_rate_curve_contract.py -v
"""
import csv
import json
import math
import os
import subprocess
import sys

import numpy as np
import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEEKER = os.path.join(REPO_ROOT, "scripts", "seeker")
sys.path.insert(0, SEEKER)
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import placard_incidence as PI                                 # noqa: E402
import tripod_score as TS                                      # noqa: E402
from field_score import _write_synthetic_bin                   # noqa: E402
from pi_capture import INDEX_HEADER, TAGS_HEADER               # noqa: E402

RTJ = os.path.join(SEEKER, "range_truth_join.py")

# --- synthetic engagement (protocol §3.1: the tripod ABEAM the leg) -----------
# The target flies a straight leg due SOUTH at 4 m/s, 6 m abeam the tripod and
# 2 m above it, carrying the adopted BEAM-facing placard at index 0 deg -- so its
# normal points WEST, at the tripod. Range sweeps in to a 6.3 m CPA and out.
TRIPOD_LAT, TRIPOD_LON, TRIPOD_ALT = 34.0500, -118.2500, 100.0
BASE_UTC = 1_752_700_000.0
EAST_M = 6.0               # cross-range standoff (the tripod is due WEST)
UP_M = 2.0                 # target flies 2 m above the tripod
SPEED = 4.0                # m/s, southbound
YAW_DEG = 180.0            # heading SOUTH
PITCH_DEG = -8.0           # nose-down cruise trim: costs a BEAM placard NOTHING
ROLL_DEG = 0.0
TRACK_HZ, TRACK_DUR = 25.0, 8.0
FRAME_HZ, N_FRAMES = 20.0, 120
CLOCK_OFFSET_S = -0.30     # t_utc = t_wall + CLOCK_OFFSET_S
NORTH0 = 12.0              # leg starts 12 m north of the CPA
# Synthetic decode envelope, shaped by the cos-theta law (placard_mount §3.2):
# the tag decodes while its INCIDENCE-CORRECTED range R/cos(theta) is inside this.
DECODE_R_EFF_M = 12.0
RANGE_BIN_M, INC_BIN_DEG, MAX_RANGE_M = 2.0, 15.0, 40.0


def _north_at(t_rel):
    return NORTH0 - SPEED * np.asarray(t_rel, dtype=float)


def _range_at(t_rel):
    return np.sqrt(EAST_M ** 2 + _north_at(t_rel) ** 2 + UP_M ** 2)


def _incidence_at(t_rel):
    """ANALYTIC incidence, derived here from scratch so the test does not simply
    re-run the producer's own arithmetic.

    The placard normal is horizontal and points due WEST (heading south, beam
    mount, index 0). The line of sight from the placard to the tripod is
    -(E, N, U) = (-6, -north, -2). cos(theta) = |n . los| / |los| = 6 / R."""
    return np.degrees(np.arccos(EAST_M / _range_at(t_rel)))


def _write_bin(path, t_rel, with_att=True):
    n = len(t_rel)
    pos = np.column_stack([np.full(n, EAST_M), _north_at(t_rel), np.full(n, UP_M)])
    att = None
    if with_att:
        att = np.column_stack([np.full(n, ROLL_DEG), np.full(n, PITCH_DEG),
                               np.full(n, YAW_DEG)])
    _write_synthetic_bin(path, pos, t_rel, BASE_UTC, TRIPOD_LAT, TRIPOD_LON,
                         TRIPOD_ALT, True, att=att)


def _make_session(session_dir):
    """A synthetic session in pi_capture's EXACT on-disk layout (real
    INDEX_HEADER, real %06d.png names, real TAGS_HEADER)."""
    frames_dir = os.path.join(session_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)
    t_rel = 1.0 + np.arange(N_FRAMES) / FRAME_HZ          # 1.0 .. 6.95 s
    t_wall = BASE_UTC + t_rel - CLOCK_OFFSET_S
    rng = _range_at(t_rel)
    inc = _incidence_at(t_rel)
    decoded = (rng / np.cos(np.radians(inc))) <= DECODE_R_EFF_M

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

    rs = np.random.default_rng(11)
    with open(os.path.join(session_dir, "tags.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(TAGS_HEADER)
        for i in range(N_FRAMES):
            rel = os.path.join("frames", f"{i:06d}.png")
            if decoded[i]:
                # A decoded row carries the tag's OWN recovered pose: a range
                # (+ the usual placard-vs-antenna bias) and an incidence. That
                # tag-pose incidence is what the scorer must refuse to call a
                # rate when the index column is absent.
                r = float(rng[i] + 0.20 + rs.normal(0, 0.05))
                wr.writerow([i, rel, 1, 0, "48.500", 0, "640.00", "400.00",
                             "0:0;1:0;1:1;0:1", f"{r:.4f}", f"{inc[i]:.3f}"])
            else:
                wr.writerow([i, rel, 0] + [""] * (len(TAGS_HEADER) - 3))

    with open(os.path.join(session_dir, "meta.json"), "w") as f:
        json.dump({"tag_size_m": 0.35, "drone_size_m": 0.35, "aspect": "crossing",
                   "source": "picamera2", "n_frames": N_FRAMES,
                   "stream_fps": FRAME_HZ,
                   "stream_fps_source": "measured: median inter-frame dt (t_mono_s)"},
                  f)
    with open(os.path.join(session_dir, "calib.json"), "w") as f:
        json.dump(dict(fx=539.936, fy=539.936, cx=640.0, cy=400.0,
                       resolution=dict(width=1280, height=800),
                       dist_coeffs=[0.0] * 5), f)
    return t_rel, rng, inc, decoded


def _run_join(session_dir, bin_path, extra=()):
    return subprocess.run(
        [sys.executable, RTJ, "--session-dir", session_dir, "--bin", bin_path,
         "--tripod-lat", str(TRIPOD_LAT), "--tripod-lon", str(TRIPOD_LON),
         "--tripod-alt", str(TRIPOD_ALT),
         "--clock-offset-s", str(CLOCK_OFFSET_S), *extra],
        capture_output=True, text=True, cwd=REPO_ROOT)


def _expected_cells(rng, inc, decoded):
    """The (range x incidence) cells the scorer MUST produce, binned here by the
    test's own arithmetic."""
    cells = {}
    for r, th, d in zip(rng, inc, decoded):
        if r <= 0 or r > MAX_RANGE_M:
            continue
        key = (int(r // RANGE_BIN_M) * RANGE_BIN_M,
               int(abs(th) // INC_BIN_DEG) * INC_BIN_DEG)
        c = cells.setdefault(key, [0, 0])
        c[1] += 1
        c[0] += int(bool(d))
    return cells


@pytest.fixture(scope="module")
def joined(tmp_path_factory):
    """The REAL producer run: a synthetic .BIN with POS+ATT, joined with
    --incidence --mount-index-deg 0."""
    tmp = tmp_path_factory.mktemp("inc")
    sdir = str(tmp / "pass01")
    t_rel, rng, inc, decoded = _make_session(sdir)
    binp = str(tmp / "target.BIN")
    _write_bin(binp, np.arange(0.0, TRACK_DUR, 1.0 / TRACK_HZ))
    proc = _run_join(sdir, binp, ["--incidence", "--mount-index-deg", "0"])
    assert proc.returncode == 0, f"join failed:\n{proc.stdout}\n{proc.stderr}"
    return sdir, binp, t_rel, rng, inc, decoded, proc


# =========================================================================
# THE headline: the producer's output makes the consumer emit a RATE curve
# =========================================================================
def test_joined_incidence_makes_curve_a_a_real_rate_curve(joined):
    sdir, _binp, _t, rng, inc, decoded, _proc = joined
    frames, index_map, _meta, tags_map = TS.load_session(sdir)
    calib = TS.load_calib(os.path.join(sdir, "calib.json"))
    decode = TS.decode_frames(frames, calib, 0.35, tags_map=tags_map, quiet=True)

    out = TS.curve_a_incidence(frames, decode, index_map, RANGE_BIN_M,
                               MAX_RANGE_M, inc_bin_deg=INC_BIN_DEG)

    assert out["rate_computable"] is True, out["note"]
    assert out["n_binnable"] == N_FRAMES
    assert out["incidence_source_counts"]["index"] == N_FRAMES, (
        "every binnable frame must take its incidence from index.csv -- one "
        "frame short and the whole session drops back to the annotation")
    assert out["incidence_source_counts"]["tag_pose"] == 0
    assert "rate" in out["axis_source"] and "ALL frames" in out["axis_source"]

    assert out["cells"] == _expected_cells(rng, inc, decoded), (
        "the (range x incidence) cells must match the analytic geometry")
    # The cells must actually SEPARATE decode from no-decode -- a curve where
    # every cell reads 1.0 (or 0.0) would pass a shape check while measuring
    # nothing.
    rates = {k: v[0] / v[1] for k, v in out["cells"].items()}
    assert max(rates.values()) == 1.0 and min(rates.values()) == 0.0
    # ...and the MISSES are in the denominator, which is the whole point.
    assert sum(v[1] for v in out["cells"].values()) == N_FRAMES
    assert sum(v[0] for v in out["cells"].values()) == int(decoded.sum()) < N_FRAMES


def test_rate_curve_csv_marks_the_cells_as_measured(joined, tmp_path):
    """The file a human reads must say, per row, that the rate is a MEASUREMENT
    (rate_is_measured=1) -- never leaving it to be inferred from prose."""
    sdir, *_ = joined
    frames, index_map, _m, tags_map = TS.load_session(sdir)
    calib = TS.load_calib(os.path.join(sdir, "calib.json"))
    decode = TS.decode_frames(frames, calib, 0.35, tags_map=tags_map, quiet=True)
    out = TS.curve_a_incidence(frames, decode, index_map, RANGE_BIN_M,
                               MAX_RANGE_M, inc_bin_deg=INC_BIN_DEG)
    path = str(tmp_path / "curve_a_incidence.csv")
    TS.write_curve_a_incidence_csv(path, out)
    rows = list(csv.DictReader(open(path)))
    assert rows, "a rate curve with zero rows is a vacuous verdict"
    assert all(r["rate_is_measured"] == "1" for r in rows)
    assert all(int(r["n_total"]) >= int(r["n_decoded"]) for r in rows)
    # Any cell past the sim-validated 32.6 deg must be flagged hypothesis-land.
    beyond = [r for r in rows
              if float(r["incidence_hi_deg"]) > TS.COS_LAW_VALIDATED_TO_DEG]
    assert beyond and all(r["beyond_validated_cos_law"] == "1" for r in beyond)


def test_written_incidence_matches_the_analytic_geometry(joined):
    """The column itself, on disk, against the closed-form aspect."""
    sdir, _binp, _t, _rng, inc, _d, _p = joined
    with open(os.path.join(sdir, "index.csv"), newline="") as f:
        rows = list(csv.DictReader(f))
    got = np.array([float(r["incidence_deg"]) for r in rows])
    assert len(got) == N_FRAMES
    assert np.max(np.abs(got - inc)) < 0.05, "incidence is not the true aspect"
    # The original pi_capture columns and the RANGE columns both survive.
    assert all(c in rows[0] for c in INDEX_HEADER)
    assert all(c in rows[0] for c in
               ("true_range_m", "range_quality", "incidence_quality",
                "incidence_src_dt_s", "incidence_sigma_deg"))
    assert all(r["incidence_quality"] == "ok" for r in rows)
    assert os.path.exists(os.path.join(sdir, "index.csv.bak"))


def test_provenance_records_the_index_angle_and_bounds_the_yaw_approximation(joined):
    sdir, *_ = joined
    prov = json.loads(open(os.path.join(sdir, "range_truth.json")).read())
    inc = prov["incidence"]
    assert inc["mount_index_deg"] == 0.0
    assert inc["attitude_source"].startswith("dataflash:ATT")
    assert inc["n_with_incidence"] == N_FRAMES
    assert inc["n_ranged_without_incidence"] == 0
    assert inc["both_faces"] is True and inc["yaw_only"] is False
    # -8 deg of PITCH must move a BEAM placard's aspect by EXACTLY zero
    # (docs/placard_mount.md §3.4 -- the reason the beam mount was chosen).
    a = inc["yaw_only_vs_full_attitude_deg"]
    assert a["n"] == N_FRAMES and a["max_deg"] == pytest.approx(0.0, abs=1e-6)
    # The sigma must NOT quietly include an invented attitude covariance.
    assert inc["attitude_sigma_deg"] is None
    assert "EKF" in (inc["sigma_excludes"] or "")
    assert inc["sigma_median_deg"] > 0


# =========================================================================
# The NEGATIVE half: no attitude -> ANNOTATION, and it says so LOUDLY
# =========================================================================
def test_without_incidence_the_same_session_degrades_to_an_annotation(tmp_path):
    """Same frames, same decodes, same range join -- but no --incidence. The
    scorer must NOT produce rate cells, and must say why."""
    sdir = str(tmp_path / "pass_no_inc")
    _t, _rng, _inc, decoded = _make_session(sdir)
    binp = str(tmp_path / "target.BIN")
    _write_bin(binp, np.arange(0.0, TRACK_DUR, 1.0 / TRACK_HZ))
    proc = _run_join(sdir, binp)                     # <-- no --incidence
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "NOT COMPUTED" in proc.stdout and "ANNOTATION" in proc.stdout, (
        "a join without incidence must SAY that the aspect curve will be an "
        "annotation, not leave the operator to discover it after the field day")

    with open(os.path.join(sdir, "index.csv"), newline="") as f:
        rows = list(csv.DictReader(f))
    assert "true_range_m" in rows[0]
    assert "incidence_deg" not in rows[0]

    frames, index_map, _m, tags_map = TS.load_session(sdir)
    calib = TS.load_calib(os.path.join(sdir, "calib.json"))
    decode = TS.decode_frames(frames, calib, 0.35, tags_map=tags_map, quiet=True)
    out = TS.curve_a_incidence(frames, decode, index_map, RANGE_BIN_M,
                               MAX_RANGE_M, inc_bin_deg=INC_BIN_DEG)
    assert out["rate_computable"] is False
    assert out["cells"] == {}, "no rate cells may exist without the misses"
    assert "NOT A RATE CURVE" in out["note"]
    assert out["incidence_source_counts"]["tag_pose"] == int(decoded.sum())
    assert out["decoded_incidence"]["n"] == int(decoded.sum())


# =========================================================================
# Fail-closed: the producer refuses every shortcut that would fake the axis
# =========================================================================
def test_incidence_without_the_mount_index_is_refused(tmp_path):
    """No default index angle. A guessed index rotates EVERY frame's aspect by
    the guess (protocol §11 item 11)."""
    sdir = str(tmp_path / "p")
    _make_session(sdir)
    binp = str(tmp_path / "target.BIN")
    _write_bin(binp, np.arange(0.0, TRACK_DUR, 1.0 / TRACK_HZ))
    proc = _run_join(sdir, binp, ["--incidence"])
    assert proc.returncode != 0
    assert "mount-index-deg" in (proc.stderr + proc.stdout)
    with open(os.path.join(sdir, "index.csv"), newline="") as f:
        assert "incidence_deg" not in list(csv.DictReader(f))[0]


def test_a_log_without_ATT_is_refused_not_assumed_level(tmp_path):
    """A .BIN whose LOG_BITMASK dropped ATT must refuse. Substituting level
    flight would invent the very quantity the curve is binned on."""
    sdir = str(tmp_path / "p")
    _make_session(sdir)
    binp = str(tmp_path / "no_att.BIN")
    _write_bin(binp, np.arange(0.0, TRACK_DUR, 1.0 / TRACK_HZ), with_att=False)
    proc = _run_join(sdir, binp, ["--incidence", "--mount-index-deg", "0"])
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert "ATT" in proc.stderr and "REFUS" in proc.stderr.upper()


def test_a_position_only_track_cannot_silently_supply_an_attitude(tmp_path):
    """A `--csv` position track carries no attitude. Asking for --incidence with
    nothing to read it from must refuse and NAME the missing input, not fall back
    to level flight."""
    sdir = str(tmp_path / "p")
    _make_session(sdir)
    track = str(tmp_path / "track.csv")
    t = np.arange(0.0, TRACK_DUR, 1.0 / TRACK_HZ)
    with open(track, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t_utc_s", "east_m", "north_m", "up_m"])
        for tv in t:
            wr.writerow([f"{BASE_UTC + tv:.6f}", EAST_M, _north_at(tv), UP_M])
    proc = subprocess.run(
        [sys.executable, RTJ, "--session-dir", sdir, "--csv", track,
         "--tripod-enu", "0,0,0", "--clock-offset-s", str(CLOCK_OFFSET_S),
         "--incidence", "--mount-index-deg", "0"],
        capture_output=True, text=True, cwd=REPO_ROOT)
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert "attitude source" in proc.stderr and "level flight" in proc.stderr


def test_attitude_that_misses_the_frames_is_a_refusal_not_an_empty_column(tmp_path):
    """NO VACUOUS VERDICTS: zero aspected frames is a failed join, not a curve
    with no cells."""
    sdir = str(tmp_path / "p")
    _make_session(sdir)
    binp = str(tmp_path / "target.BIN")
    _write_bin(binp, np.arange(0.0, TRACK_DUR, 1.0 / TRACK_HZ))
    att_csv = str(tmp_path / "att_elsewhere.csv")
    with open(att_csv, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t_utc_s", "roll_deg", "pitch_deg", "yaw_deg"])
        for i in range(200):                       # a different hour entirely
            wr.writerow([f"{BASE_UTC + 9000 + i * 0.04:.6f}", 0.0, 0.0, 180.0])
    proc = _run_join(sdir, binp, ["--incidence", "--mount-index-deg", "0",
                                  "--attitude-csv", att_csv])
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert "ZERO frames with an aspect" in proc.stderr


def test_yaw_only_attitude_csv_needs_the_explicit_flag(tmp_path):
    """An attitude export with no roll/pitch is refused unless the operator
    ACCEPTS the yaw-only approximation by name."""
    sdir = str(tmp_path / "p")
    _make_session(sdir)
    binp = str(tmp_path / "target.BIN")
    _write_bin(binp, np.arange(0.0, TRACK_DUR, 1.0 / TRACK_HZ))
    att_csv = str(tmp_path / "yaw_only.csv")
    t = np.arange(0.0, TRACK_DUR, 1.0 / TRACK_HZ)
    with open(att_csv, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t_utc_s", "yaw_deg"])
        for tv in t:
            wr.writerow([f"{BASE_UTC + tv:.6f}", YAW_DEG])
    proc = _run_join(sdir, binp, ["--incidence", "--mount-index-deg", "0",
                                  "--attitude-csv", att_csv])
    assert proc.returncode == 2
    assert "roll/pitch" in proc.stderr and "INVENT" in proc.stderr

    proc2 = _run_join(sdir, binp, ["--incidence", "--mount-index-deg", "0",
                                   "--attitude-csv", att_csv,
                                   "--incidence-yaw-only"])
    assert proc2.returncode == 0, proc2.stdout + proc2.stderr
    prov = json.loads(open(os.path.join(sdir, "range_truth.json")).read())
    assert prov["incidence"]["yaw_only"] is True
    assert any("yaw-only" in w for w in prov["incidence"]["attitude_warnings"])


def test_partial_attitude_coverage_refuses_by_default(tmp_path):
    """A ranged-but-aspectless frame costs the SESSION its rate curve (the
    consumer is all-or-nothing), so the loss is counted and refused here, where
    the logs are still in hand."""
    sdir = str(tmp_path / "p")
    _make_session(sdir)
    binp = str(tmp_path / "target.BIN")
    _write_bin(binp, np.arange(0.0, TRACK_DUR, 1.0 / TRACK_HZ))
    att_csv = str(tmp_path / "att_half.csv")
    t = np.arange(0.0, 4.0, 1.0 / TRACK_HZ)          # covers only the first half
    with open(att_csv, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t_utc_s", "roll_deg", "pitch_deg", "yaw_deg"])
        for tv in t:
            wr.writerow([f"{BASE_UTC + tv:.6f}", 0.0, PITCH_DEG, YAW_DEG])
    proc = _run_join(sdir, binp, ["--incidence", "--mount-index-deg", "0",
                                  "--attitude-csv", att_csv])
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert "could NOT be given an incidence" in proc.stderr
    assert "ANNOTATION" in proc.stderr

    proc2 = _run_join(sdir, binp, ["--incidence", "--mount-index-deg", "0",
                                   "--attitude-csv", att_csv,
                                   "--max-incidence-gap-frac", "1.0"])
    assert proc2.returncode == 0, proc2.stdout + proc2.stderr
    prov = json.loads(open(os.path.join(sdir, "range_truth.json")).read())
    assert prov["incidence"]["n_ranged_without_incidence"] > 0
    # ...and the consumer then refuses to call it a rate, loudly, on its own.
    frames, index_map, _m, tags_map = TS.load_session(sdir)
    calib = TS.load_calib(os.path.join(sdir, "calib.json"))
    decode = TS.decode_frames(frames, calib, 0.35, tags_map=tags_map, quiet=True)
    out = TS.curve_a_incidence(frames, decode, index_map, RANGE_BIN_M,
                               MAX_RANGE_M, inc_bin_deg=INC_BIN_DEG)
    assert out["rate_computable"] is False
    # THE SUBTLE HALF (found by this contract test, fixed 2026-07-25): with a
    # PARTIAL incidence column the scorer still emits cells -- built from the
    # frames that DO carry an aspect. That shrunken denominator must be named,
    # not absorbed, and the note must not claim there is no incidence column.
    assert out["cells_are_partial"] is True and out["cells"]
    assert "SHRUNKEN denominator" in out["note"]
    assert f"missing {out['n_binnable'] - out['incidence_source_counts']['index']} " \
           in out["note"]
    assert "PARTIAL" in out["axis_source"]
    assert "carries no" not in out["note"], (
        "the old fall-through said index.csv carried NO incidence_deg column "
        "while some frames plainly did -- a confident wrong diagnosis")


# =========================================================================
# The geometry the mount doc claims -- asserted, not assumed
# =========================================================================
def test_mount_index_is_load_bearing_and_beam_pitch_is_free():
    """placard_mount §3.4/§3.6, as code: the index angle rotates the aspect, and
    pitch costs a BEAM placard exactly nothing."""
    beam = PI.normals_enu([0.0], [0.0], [YAW_DEG], 0.0)[0]
    nose = PI.normals_enu([0.0], [0.0], [YAW_DEG], 90.0)[0]
    assert np.allclose(beam, [-1, 0, 0], atol=1e-9)     # heading south -> west
    assert np.allclose(nose, [0, -1, 0], atol=1e-9)     # ...nose-on -> south
    pitched = PI.normals_enu([0.0], [35.0], [YAW_DEG], 0.0)[0]
    assert np.allclose(pitched, beam, atol=1e-12)
    rolled = PI.normals_enu([20.0], [0.0], [YAW_DEG], 0.0)[0]
    assert math.degrees(math.asin(abs(rolled[2]))) == pytest.approx(20.0)
    # A nose-on placard pays the pitch 1:1 -- the mirror image, and the reason
    # the 90 deg index is capped at <=4 m/s (placard_mount §8).
    nose_pitched = PI.normals_enu([0.0], [35.0], [YAW_DEG], 90.0)[0]
    assert math.degrees(math.asin(abs(nose_pitched[2]))) == pytest.approx(35.0)


def test_scorer_cross_checks_the_index_angle_the_column_was_built_with(joined,
                                                                      tmp_path):
    """A score that names a DIFFERENT placard index than the join used would put
    an aspect angle in gate.json that contradicts the axis every cell is binned
    on. The scorer must adopt the join's index when none is given, and say so
    loudly when the two disagree."""
    sdir, *_ = joined
    # no index on the command line, none in meta -> adopt the JOIN's
    val, src, warn = TS.resolve_mount_index(None, {}, sdir)
    assert val == 0.0 and "range_truth.json" in src and warn is None
    # a contradicting index -> loud, and the mismatch is in the provenance too
    val2, src2, warn2 = TS.resolve_mount_index(90.0, {}, sdir)
    assert val2 == 90.0
    assert warn2 and "MOUNT INDEX MISMATCH" in warn2
    assert "MISMATCH" in src2 and "0.0 deg" in src2
    # an unjoined session keeps the old behaviour exactly
    empty = str(tmp_path / "unjoined")
    os.makedirs(empty, exist_ok=True)
    val3, src3, warn3 = TS.resolve_mount_index(None, {"mount_index_deg": 15.0},
                                               empty)
    assert val3 == 15.0 and "meta.json" in src3 and warn3 is None
    assert TS.resolve_mount_index(None, {}, empty) == (None, "meta.json "
                                                       "mount_index_deg", None)


def test_cos_law_validity_edge_agrees_across_the_two_modules():
    """The 33 deg edge-of-evidence must be ONE number, not two that drift."""
    assert PI.COS_LAW_VALIDATED_TO_DEG == TS.COS_LAW_VALIDATED_TO_DEG


def test_placard_incidence_self_test_passes():
    proc = subprocess.run(
        [sys.executable, os.path.join(SEEKER, "placard_incidence.py")],
        capture_output=True, text=True, cwd=REPO_ROOT)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "[placard_incidence] PASS" in proc.stdout
