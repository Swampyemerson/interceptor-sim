"""airframe.params must agree with the newest full dump of the live board.

The 2026-11-09 wobble hunt found the board flying a motor layout and gains that
no file in the repo recorded. This pins the as-flown airframe settings to a
file, and fails if someone changes the board (new dump) without changing the
file, or the file without a dump that shows the board holds it.
"""
import glob
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(ROOT, "configs", "px4_6cmini")


def _utc(path):
    with open(path, encoding="utf-8") as f:
        head = f.readline()
    assert "utc=" in head, f"{path} has no utc= header -- cannot order dumps"
    return head.split("utc=")[1].split()[0]


def _newest_dump():
    # ordered by the capture time written in each dump's header, not file mtime
    dumps = sorted(glob.glob(os.path.join(CFG, "dumps", "*.tsv")), key=_utc)
    assert dumps, "no dumps in configs/px4_6cmini/dumps -- nothing to compare (not a pass)"
    return dumps[-1]


def _run(dump):
    return subprocess.run([sys.executable, os.path.join(CFG, "verify_params.py"),
                           "--dump", dump, "--file", "airframe.params"],
                          capture_output=True, text=True)


def test_airframe_pack_matches_newest_dump():
    r = _run(_newest_dump())
    assert r.returncode == 0, r.stdout + r.stderr


def test_mismatch_is_detected():
    # the pre-fix dump still has the Quad + layout and stock gains: must FAIL
    r = _run(os.path.join(CFG, "dumps", "20261109_before_geometry_fix.tsv"))
    assert r.returncode == 1 and "SYS_AUTOSTART" in r.stdout, r.stdout + r.stderr
