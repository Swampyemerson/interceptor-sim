#!/usr/bin/env python3
"""REGRESSION: the anti-mirage verdict must refuse to pair unmatched arms.

THE DEFECT (review 2, 2026-07-25 -- docs/review2_silent_failure_findings.md
"antimirage_verdict.py pairs arms by run_idx without verifying the control arm's
direction or geometry match"). `paired()` selected indices with

    sorted(i for i in cam if i in dash and cam[i]["dir"] == d)

-- the CAMERA arm's direction only. It never asserted that dash[i] was the same
direction, the same cue_seed, or the same target start geometry, and `load()`
DISCARDED those columns outright, so nothing could check. Since 2026-07-24 the
launcher writes `logs/mc_fp_arm${ARM}_line9_s${SEED}.csv` and arms are routinely
replicated on the disjoint seed 777, so seed-123 and seed-777 files sit side by
side: a one-character slip pairs run_idx 0 of one geometry against run_idx 0 of a
completely different one, and the tool prints a full per-flight table plus a
categorical "CAMERA-DRIVEN ... NOT a mirage" verdict with no warning.

This is the gate that stands between the project and a repeat of the retracted
"--track breakthrough" geometry artifact (ADR-0076 add #7/#8), and the aim-error
sweep is the pre-registered harness a FUTURE session will re-run against a log
directory holding many more same-shaped, different-seed arms.

THE FIX pinned here: per run_idx, direction + cue_seed + path_seed +
target_start_x/y + target_vy must all match; on any mismatch the tool names the
offending flights and exits 2 WITHOUT printing a verdict.

NOTE (not a retraction): the PUBLISHED loft-dive A/Adash/B/Bdash quad was
re-verified correct -- all 8 indices match on all keys -- and the committed
verdicts reproduce byte-for-byte under the guard. The last test here pins that.

Run: .venv/bin/python -m pytest tests/test_antimirage_pairing.py -v
"""
import csv
import os
import subprocess
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOL = os.path.join(REPO_ROOT, "scripts", "experiments", "loft_dive",
                    "antimirage_verdict.py")
LOGS = os.path.join(REPO_ROOT, "logs")

HEADER = ["run_idx", "law", "target_vx", "target_vy", "target_start_y",
          "cue_seed", "miss_m", "clean", "handoff", "direction",
          "target_start_x", "path_seed"]


def _write_arm(path, *, misses, cue_seeds, start_ys, dirs=None, path_seeds=None):
    n = len(misses)
    dirs = dirs or (["l2r", "r2l"] * n)[:n]
    path_seeds = path_seeds or [str(1000 + i) for i in range(n)]
    with open(path, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(HEADER)
        for i in range(n):
            wr.writerow([i, "pronav_9.000", 0.0, 9.0, f"{start_ys[i]:.6f}",
                         cue_seeds[i], f"{misses[i]:.3f}", 1, "", dirs[i],
                         "0.000000", path_seeds[i]])


def _run(*pairs):
    return subprocess.run([sys.executable, TOOL, *pairs],
                          capture_output=True, text=True, cwd=REPO_ROOT)


def test_matched_arms_still_produce_a_verdict(tmp_path):
    cam = str(tmp_path / "mc_x_armA_line9_s123.csv")
    dash = str(tmp_path / "mc_x_armAdash_line9_s123.csv")
    seeds = [str(100 + i) for i in range(4)]
    ys = [1.0, 2.0, 3.0, 4.0]
    _write_arm(cam, misses=[0.5, 0.6, 0.55, 0.62], cue_seeds=seeds, start_ys=ys)
    _write_arm(dash, misses=[2.0, 2.1, 2.2, 2.3], cue_seeds=seeds, start_ys=ys)
    p = _run(f"A={cam}", f"Adash={dash}")
    assert p.returncode == 0, p.stdout + p.stderr
    assert "CAMERA-DRIVEN" in p.stdout
    # identity of BOTH arms is printed, so a slip is visible in the output itself
    assert os.path.basename(cam) in p.stdout
    assert os.path.basename(dash) in p.stdout


def test_cross_seed_arms_are_REFUSED_with_no_verdict(tmp_path):
    """The reproduced failure: two arms from different master seeds. PRE-FIX this
    printed per-flight deltas, paired_delta_med=-0.90 and
    'CAMERA-DRIVEN (tightens, consistent) -> NOT a mirage'."""
    cam = str(tmp_path / "mc_x_armA_line9_s123.csv")
    dash = str(tmp_path / "mc_x_armAdash_line9_s777.csv")
    _write_arm(cam, misses=[0.5, 0.6, 0.55, 0.62],
               cue_seeds=["100", "101", "102", "103"], start_ys=[1.0, 2.0, 3.0, 4.0])
    _write_arm(dash, misses=[2.0, 2.1, 2.2, 2.3],
               cue_seeds=["900", "901", "902", "903"], start_ys=[7.0, 8.0, 9.0, 1.0])
    p = _run(f"A={cam}", f"Adash={dash}")
    assert p.returncode == 2, p.stdout + p.stderr
    assert "GEOMETRY MISMATCH" in p.stderr
    assert "cue_seed" in p.stderr
    assert "CAMERA-DRIVEN" not in p.stdout, "a verdict must NOT be printed"
    assert "MIRAGE" not in p.stdout


def test_a_direction_slip_in_the_CONTROL_arm_is_caught(tmp_path):
    """An l2r-only control paired against a both-directions camera arm silently
    crossed directions: the camera arm's direction was the only one checked."""
    cam = str(tmp_path / "mc_x_armB_line9_s123.csv")
    dash = str(tmp_path / "mc_x_armBdash_line9_s123.csv")
    seeds = [str(100 + i) for i in range(4)]
    ys = [1.0, 2.0, 3.0, 4.0]
    _write_arm(cam, misses=[1.0] * 4, cue_seeds=seeds, start_ys=ys,
               dirs=["l2r", "r2l", "l2r", "r2l"])
    _write_arm(dash, misses=[2.0] * 4, cue_seeds=seeds, start_ys=ys,
               dirs=["l2r", "l2r", "l2r", "l2r"])
    p = _run(f"B={cam}", f"Bdash={dash}")
    assert p.returncode == 2
    assert "direction" in p.stderr


def test_a_geometry_drift_below_the_seed_level_is_caught(tmp_path):
    """Same seeds, but the control arm was flown from a different start-y (a
    re-run after an aborted flight shifts run_idx and the geometry with it)."""
    cam = str(tmp_path / "mc_x_armC_line9_s123.csv")
    dash = str(tmp_path / "mc_x_armCdash_line9_s123.csv")
    seeds = [str(100 + i) for i in range(4)]
    _write_arm(cam, misses=[1.0] * 4, cue_seeds=seeds, start_ys=[1.0, 2.0, 3.0, 4.0])
    _write_arm(dash, misses=[2.0] * 4, cue_seeds=seeds, start_ys=[1.0, 2.0, 3.5, 4.0])
    p = _run(f"A={cam}", f"Adash={dash}")
    assert p.returncode == 2
    assert "target_start_y" in p.stderr


@pytest.mark.skipif(
    not os.path.exists(os.path.join(LOGS, "mc_loftdive_armA_line9_s123.csv")),
    reason="published loft-dive arm CSVs not present in this checkout")
def test_the_published_loftdive_quad_still_pairs_and_reproduces():
    """NO RETRACTION: the settled 2x2 (gazebo_results.md; contract SETTLED
    2026-07-24) is correctly seed-matched, so the guard must pass it and the
    headline numbers must reproduce exactly."""
    p = _run(*[f"{k}={os.path.join(LOGS, f'mc_loftdive_arm{a}_line9_s123.csv')}"
               for k, a in (("A", "A"), ("Adash", "Adash"),
                            ("B", "B"), ("Bdash", "Bdash"))])
    assert p.returncode == 0, p.stdout + p.stderr
    assert "GEOMETRY MISMATCH" not in p.stderr
    assert "paired_delta_med=-1.18" in p.stdout, "ARM A l2r headline moved"
    assert "CAMERA-DRIVEN (tightens, consistent)" in p.stdout
    assert p.stdout.count("CAMERA STEERS WRONG") == 2, "ARM B camera-worse legs"
