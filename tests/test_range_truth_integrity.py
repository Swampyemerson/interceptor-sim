#!/usr/bin/env python3
"""range_truth_join integrity checks on the SHARED clock path (audit RANK 6).

THE DEFECT THESE TESTS PIN. Every residual / bias / survey / altitude-datum test
in scripts/seeker/range_truth_join.py lived inside `if do_auto_sync:`. But
docs/tripod_test_protocol.md §6.1/§7.0 make `--clock-offset-s` THE PLAN and
`--auto-sync` a cross-check runnable on ONE pass per battery segment (it refuses a
constant-closing-rate approach BY DESIGN, §4.6b). So the field day's dominant
geometry ran the ONE path with no integrity checks at all, and `IMPLAUSIBLE_UP_M`
only ever set a warning flag. Reproduced by the audit through the module's own
fixture: a 1.1 km survey error, an AMSL-vs-AGL altitude-datum blunder (median
height ~106 m, which slips UNDER the 150 m warn-only guard) and a 3 s clock error
each exited 0 with empty warnings and a confidently-wrong `true_range_m` on every
frame -- feeding BOTH curves and the ~$740 gate.

`true_range_m` is the money gate's own truth source, so this is the
"instruments are evidence" class (CLAUDE.md): a bug here cannot be caught by any
paired control, because every arm shares the instrument.

Each test below is written so it FAILS against the pre-fix behaviour. Where the
point is "the old code produced a confident wrong answer", the old behaviour is
reproduced explicitly by relaxing the ceilings to infinity (that IS the pre-hoist
shared path: no bias test, no residual test) and the WRONGNESS of the resulting
join is measured, not asserted by assumption.

Everything is offline: a synthetic ENU track CSV + a synthetic pi_capture-layout
session. No camera, no sim, no network.

Run: .venv/bin/python -m pytest tests/test_range_truth_integrity.py -v
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
for p in (SEEKER, os.path.join(REPO_ROOT, "scripts")):
    if p not in sys.path:
        sys.path.insert(0, p)

import range_truth_join as RTJ                                  # noqa: E402
from field_score import load_track_from_csv                     # noqa: E402
from pi_capture import INDEX_HEADER, TAGS_HEADER                # noqa: E402
from pathlib import Path                                        # noqa: E402

# ---- a realistic tripod pass (protocol §4.6b): the target crosses at an 8 m
# offset, 6 m up, 4 m/s along north -- the range closes to a CPA and opens again.
BASE_UTC = 1_752_700_000.0
EAST_M, UP_M, SPEED = 8.0, 6.0, 4.0
TRACK_HZ, TRACK_DUR = 25.0, 16.0
FRAME_HZ, N_FRAMES = 20.0, 140
TRUE_OFFSET_S = -0.30          # t_utc = t_wall + TRUE_OFFSET_S
TAG_BIAS_M = 0.20              # placard-vs-GPS-antenna, the expected small bias
TAG_DECODE_MAX_M = 16.0


def _north(t):
    return 40.0 - SPEED * np.asarray(t, dtype=float)


def _range(t):
    return np.sqrt(EAST_M ** 2 + _north(t) ** 2 + UP_M ** 2)


def _track_csv(path, up_m=UP_M):
    t = np.arange(0.0, TRACK_DUR, 1.0 / TRACK_HZ)
    with open(path, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t_utc_s", "east_m", "north_m", "up_m"])
        for tt in t:
            wr.writerow([f"{BASE_UTC + tt:.6f}", f"{EAST_M:.6f}",
                         f"{_north(tt):.6f}", f"{up_m:.6f}"])
    return path


def _session(dirpath, with_tags=True):
    """pi_capture's EXACT on-disk layout (real INDEX_HEADER / TAGS_HEADER)."""
    os.makedirs(os.path.join(dirpath, "frames"), exist_ok=True)
    t_rel = 6.0 + np.arange(N_FRAMES) / FRAME_HZ
    t_wall = BASE_UTC + t_rel - TRUE_OFFSET_S
    true_rng = _range(t_rel)
    with open(os.path.join(dirpath, "index.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(INDEX_HEADER)
        for i in range(N_FRAMES):
            rel = os.path.join("frames", f"{i:06d}.png")
            open(os.path.join(dirpath, rel), "wb").close()
            wr.writerow([i, rel, f"{i / FRAME_HZ:.6f}", f"{t_wall[i]:.6f}",
                         "980.0", "1.0000"])
    if with_tags:
        rs = np.random.default_rng(3)
        with open(os.path.join(dirpath, "tags.csv"), "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(TAGS_HEADER)
            for i in range(N_FRAMES):
                rel = os.path.join("frames", f"{i:06d}.png")
                if true_rng[i] <= TAG_DECODE_MAX_M:
                    r = float(true_rng[i] + TAG_BIAS_M + rs.normal(0, 0.08))
                    wr.writerow([i, rel, 1, 7, "48.5", 0, "640.0", "400.0",
                                 "0:0;1:0;1:1;0:1", f"{r:.4f}", "3.10"])
                else:
                    wr.writerow([i, rel, 0, "", "", "", "", "", "", "", ""])
    return dirpath, t_rel, true_rng


def _joined_ranges(session_dir):
    rows = list(csv.DictReader(open(os.path.join(session_dir, "index.csv"))))
    return np.array([float(r["true_range_m"]) for r in rows if r["true_range_m"]])


# ===========================================================================
# 1. THE HEADLINE: a wrong --clock-offset-s on the SHARED path
# ===========================================================================
def test_wrong_clock_offset_was_a_confident_wrong_join_and_is_now_refused(tmp_path):
    """A 3 s clock error on the path the protocol tells the operator to use.

    PART 1 reproduces the OLD behaviour (the pre-hoist shared path had no bias or
    residual ceiling at all -- modelled here by setting both to infinity): the
    join succeeds, writes a range to every frame, and is wrong by ~12 m. Nothing
    in the output says so.

    PART 2 is the fix: identical inputs, default ceilings -> REFUSED."""
    sdir, t_rel, true_rng = _session(str(tmp_path / "pass01"))
    track = load_track_from_csv(Path(_track_csv(str(tmp_path / "trk.csv"))), "t")
    bad_offset = TRUE_OFFSET_S + 3.0

    # --- PART 1: the pre-2026-07-25 shared path (no integrity ceilings) -------
    prov_old = RTJ.join_session(
        sdir, track, tripod_enu=[0.0, 0.0, 0.0], clock_offset_s=bad_offset,
        max_residual_m=float("inf"), max_bias_m=float("inf"), quiet=True)
    got = _joined_ranges(sdir)
    err = float(np.median(np.abs(got - true_rng[:len(got)])))
    assert prov_old["n_true_range_written"] == N_FRAMES, (
        "the old path wrote a range to every frame...")
    # Measured on this fixture: median ~6.4 m of range error, i.e. THREE of the
    # scorer's 2 m curve-(a) bins, silently mis-binning every frame in the session.
    assert err > 5.0, (
        f"...and it was wrong by a median {err:.1f} m -- more than two 2 m curve-(a) "
        f"bins. This is the confidently-wrong join the money gate would have "
        f"binned on")
    assert not [w for w in prov_old["warnings"] if "OVERRIDDEN" not in w], (
        "the old path emitted NO warning about a 12 m error")

    # --- PART 2: the hoisted check, same inputs ------------------------------
    sdir2, _t, _r = _session(str(tmp_path / "pass02"))
    with pytest.raises(RTJ.Refuse) as e:
        RTJ.join_session(sdir2, track, tripod_enu=[0.0, 0.0, 0.0],
                         clock_offset_s=bad_offset, quiet=True)
    msg = str(e.value)
    assert "CONSTANT BIAS" in msg or "residual" in msg
    assert "explicit --clock-offset-s" in msg, (
        "the refusal must name WHICH offset it checked, so the operator knows to "
        "go back to the §6.1 sync card")
    assert "true_range_m" not in (
        list(csv.DictReader(open(os.path.join(sdir2, "index.csv"))))[0]), (
        "a refused join must not touch index.csv")


def test_a_correct_clock_offset_still_joins_and_records_what_it_checked(tmp_path):
    """The fix must not cost the normal path: the correct offset joins, and the
    integrity block records the numbers, so a PASSED check is distinguishable
    from a check that never ran."""
    sdir, t_rel, true_rng = _session(str(tmp_path / "pass01"))
    track = load_track_from_csv(Path(_track_csv(str(tmp_path / "trk.csv"))), "t")
    prov = RTJ.join_session(sdir, track, tripod_enu=[0.0, 0.0, 0.0],
                            clock_offset_s=TRUE_OFFSET_S, quiet=True)
    got = _joined_ranges(sdir)
    assert np.max(np.abs(got - true_rng)) < 0.05
    sb = prov["integrity"]["survey_bias"]
    assert sb["ran"] is True
    assert sb["offset_source"] == "explicit --clock-offset-s"
    assert abs(sb["bias_m"] - TAG_BIAS_M) < 0.15, (
        "the small placard-vs-antenna bias must be REPORTED, not refused")
    assert sb["residual_m"] < 0.5 and sb["n_used"] > 100


# ===========================================================================
# 2. SURVEY + ALTITUDE DATUM on the shared path
# ===========================================================================
def test_amsl_vs_agl_datum_blunder_is_refused_though_it_clears_the_height_guard(tmp_path):
    """The audit's exact example: a tripod altitude given AGL instead of AMSL puts
    the target ~106 m 'above' the tripod -- UNDER the 150 m implausible-height
    ceiling, so only the hoisted BIAS check can catch it, and it must."""
    sdir, _t, _r = _session(str(tmp_path / "pass01"))
    track = load_track_from_csv(Path(_track_csv(str(tmp_path / "trk.csv"))), "t")
    # tripod 106 m BELOW the truth == every target sample 106 m higher
    with pytest.raises(RTJ.Refuse) as e:
        RTJ.join_session(sdir, track, tripod_enu=[0.0, 0.0, -106.0],
                         clock_offset_s=TRUE_OFFSET_S, quiet=True)
    assert "BIAS" in str(e.value) or "residual" in str(e.value)
    # ...and prove the height guard alone would NOT have stopped it.
    info, warns = RTJ.altitude_datum_check(106.0 + UP_M)
    assert info["refused"] is False
    assert warns and "ALTITUDE DATUM" in warns[0], (
        "below the ceiling it must still ADVISE, since that band is where a "
        "few-tens-of-metres datum error lands")


def test_implausible_geometry_is_fail_closed_not_a_warning(tmp_path):
    """IMPLAUSIBLE_UP_M used to only set a warning flag on a provenance blob."""
    sdir, _t, _r = _session(str(tmp_path / "pass01"))
    track = load_track_from_csv(Path(_track_csv(str(tmp_path / "trk.csv"))), "t")
    with pytest.raises(RTJ.Refuse) as e:
        RTJ.join_session(sdir, track, tripod_enu=[0.0, 0.0, -300.0],
                         clock_offset_s=TRUE_OFFSET_S, quiet=True)
    assert "IMPLAUSIBLE" in str(e.value) and "DATUM" in str(e.value)

    # the deliberate override proceeds AND stamps itself
    sdir2, _t2, _r2 = _session(str(tmp_path / "pass02"))
    prov = RTJ.join_session(sdir2, track, tripod_enu=[0.0, 0.0, -300.0],
                            clock_offset_s=TRUE_OFFSET_S,
                            allow_implausible_geometry=True,
                            max_bias_m=float("inf"),
                            max_residual_m=float("inf"), quiet=True)
    assert prov["integrity"]["altitude_datum"]["refused"] is True
    assert prov["integrity"]["altitude_datum"]["override"] is True
    assert any("OVERRIDDEN" in w for w in prov["warnings"])


def test_a_gross_survey_error_is_refused_on_the_explicit_path(tmp_path):
    """A tripod surveyed ~100 m off laterally: the geometry is intact (no datum
    problem, no clock problem) but the log-derived range no longer describes the
    same engagement as the tag-derived one."""
    sdir, _t, _r = _session(str(tmp_path / "pass01"))
    track = load_track_from_csv(Path(_track_csv(str(tmp_path / "trk.csv"))), "t")
    with pytest.raises(RTJ.Refuse) as e:
        RTJ.join_session(sdir, track, tripod_enu=[100.0, 0.0, 0.0],
                         clock_offset_s=TRUE_OFFSET_S, quiet=True)
    assert "residual" in str(e.value) or "BIAS" in str(e.value)


# ===========================================================================
# 3. NO VACUOUS PASS: an un-run check is not a passed check
# ===========================================================================
def test_a_pass_with_no_tag_ranges_refuses_unless_explicitly_overridden(tmp_path):
    """With no tag decodes nothing cross-checks the survey/datum/clock. The old
    code wrote the join anyway and said nothing (docs/error_handling_policy.md
    §3: a verdict computed on ZERO units is UNCERTAIN, never PASS)."""
    sdir, _t, _r = _session(str(tmp_path / "pass01"), with_tags=False)
    track = load_track_from_csv(Path(_track_csv(str(tmp_path / "trk.csv"))), "t")

    with pytest.raises(RTJ.Refuse) as e:
        RTJ.join_session(sdir, track, tripod_enu=[0.0, 0.0, 0.0],
                         clock_offset_s=TRUE_OFFSET_S, quiet=True)
    msg = str(e.value)
    assert "COULD NOT RUN" in msg
    assert "--allow-unverified-survey" in msg, (
        "a refusal must name the way forward or it strands the operator")

    prov = RTJ.join_session(sdir, track, tripod_enu=[0.0, 0.0, 0.0],
                            clock_offset_s=TRUE_OFFSET_S,
                            allow_unverified_survey=True, quiet=True)
    sb = prov["integrity"]["survey_bias"]
    assert sb["ran"] is False and sb["override"] is True
    assert sb["n_tag_ranged_frames"] == 0
    assert any("OVERRIDDEN" in w for w in prov["warnings"]), (
        "the override must be visible in the provenance the scorer prints")


def test_integrity_is_recorded_on_the_auto_sync_path_too(tmp_path):
    """Both paths write the same integrity block, so a reader never has to know
    which path produced a session to know what was checked."""
    sdir, _t, _r = _session(str(tmp_path / "pass01"))
    track = load_track_from_csv(Path(_track_csv(str(tmp_path / "trk.csv"))), "t")
    prov = RTJ.join_session(sdir, track, tripod_enu=[0.0, 0.0, 0.0],
                            do_auto_sync=True, quiet=True)
    sb = prov["integrity"]["survey_bias"]
    assert sb["ran"] is True and "auto-sync" in sb["offset_source"]
    assert abs(prov["clock_offset_s"] - TRUE_OFFSET_S) < 0.05
    assert prov["integrity"]["altitude_datum"]["refused"] is False


# ===========================================================================
# 4. The CLI honours all of it (exit 2 = REFUSED), and --help does not crash
# ===========================================================================
def _cli(session_dir, track_csv, *extra):
    return subprocess.run(
        [sys.executable, os.path.join(SEEKER, "range_truth_join.py"),
         "--session-dir", session_dir, "--csv", track_csv,
         "--tripod-enu", "0,0,0", *extra],
        capture_output=True, text=True, cwd=REPO_ROOT)


def test_cli_exits_2_on_a_wrong_offset_and_0_on_the_right_one(tmp_path):
    sdir, _t, _r = _session(str(tmp_path / "pass01"))
    track = _track_csv(str(tmp_path / "trk.csv"))
    bad = _cli(sdir, track, "--clock-offset-s", str(TRUE_OFFSET_S + 3.0))
    assert bad.returncode == 2, bad.stdout + bad.stderr
    assert "REFUSED" in bad.stderr

    good = _cli(sdir, track, "--clock-offset-s", str(TRUE_OFFSET_S))
    assert good.returncode == 0, good.stdout + good.stderr
    assert "integrity" in good.stdout
    prov = json.loads(open(os.path.join(sdir, "range_truth.json")).read())
    assert prov["integrity"]["survey_bias"]["ran"] is True


@pytest.mark.parametrize("tool", ["range_truth_join.py", "tripod_score.py",
                                  "pi_capture.py", "autolabel_from_apriltag.py"])
def test_help_does_not_crash(tool):
    """eb748e1 fixed `tripod_score.py --help` dying on a bare %% in an argparse
    string -- a tool whose own entry point was never exercised. On the field day a
    --help crash costs a drive home; these flags are new, so pin it."""
    p = subprocess.run([sys.executable, os.path.join(SEEKER, tool), "--help"],
                       capture_output=True, text=True, cwd=REPO_ROOT)
    assert p.returncode == 0, p.stdout + p.stderr
    assert "usage:" in p.stdout
