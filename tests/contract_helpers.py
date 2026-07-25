"""Shared fixture builders for the producer->consumer GOLDEN CONTRACT tests.

THE RULE (review 2, 2026-07-24): a contract fixture is generated from the
PRODUCER'S OWN writer/header, never hand-typed. Hand-typed fixtures are what
let both 2026-07-24 schema breaks pass every unit test while the real pair
silently returned garbage (recorder wrote frame_idx/..., scorer joined on
frame/... -> empty join -> calm UNCERTAIN verdict).

Producers covered here:
  * scripts/m4_intercept.py per-flight CSV  -> CSV_HEADER (the module's own list)
  * scripts/mc_batch.sh arm CSV             -> the header list in its own heredoc

This module never skips: if the gz bindings needed to IMPORT m4_intercept are
absent (CI's OSRF apt step is best-effort), the SAME assignment is ast-parsed
out of the producer's source instead — green must mean the contract ran.
"""
import ast
import csv
import math
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
M4_PATH = os.path.join(REPO_ROOT, "scripts", "m4_intercept.py")
MC_BATCH_PATH = os.path.join(REPO_ROOT, "scripts", "mc_batch.sh")


def m4_csv_header():
    """(header_list, source_str) for the per-flight CSV schema, from the
    producer itself. Prefers a real import (catches a dynamically-built or
    renamed CSV_HEADER); falls back to ast-parsing the same assignment when
    the module-scope gz imports are unavailable."""
    try:
        if os.path.join(REPO_ROOT, "scripts") not in sys.path:
            sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
        import m4_intercept  # module-scope gz.transport13/gz.msgs10 imports
        return list(m4_intercept.CSV_HEADER), "import m4_intercept"
    except Exception:
        pass
    with open(M4_PATH) as f:
        tree = ast.parse(f.read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(
                getattr(t, "id", "") == "CSV_HEADER" for t in node.targets):
            val = ast.literal_eval(node.value)
            if isinstance(val, list) and val and all(isinstance(c, str) for c in val):
                return val, "ast-parsed scripts/m4_intercept.py"
    raise AssertionError(
        "CSV_HEADER not found in scripts/m4_intercept.py -- the producer "
        "schema moved; update the golden contract tests WITH it")


def mc_arm_header():
    """The mc_batch.sh arm-CSV header, extracted from the producer's own
    heredoc (the `header = [...]` python block that writes the CSV's first
    row). Not importable (bash), so the literal is parsed out of the script
    text -- still the producer's writer, not a hand-typed copy."""
    with open(MC_BATCH_PATH) as f:
        src = f.read()
    m = re.search(r"^header = (\[.*?^\])\s*$", src, re.S | re.M)
    assert m, ("mc_batch.sh: the arm-CSV `header = [...]` writer block was "
               "not found -- the producer moved; update the golden contract "
               "tests WITH it")
    val = ast.literal_eval(m.group(1))
    assert isinstance(val, list) and val and all(isinstance(c, str) for c in val)
    return val


def write_flight_csv(path, header, phantom=False, engage=True):
    """A synthetic flight written through the producer's schema (csv.DictWriter
    over the REAL header; empty string for unset columns = m4's own convention).

    Layout: 3 TAKEOFF + 5 CODED_DASH ticks, then (engage=True) 20 detected
    ENGAGE ticks whose commanded-velocity azimuth tracks lambda_deg EXACTLY
    (honest camera-driven kinematics: audit_per_tick check (c) corr = 1.0,
    azimuth std ~5.8 deg > the 2-deg power floor, vc > 0 pre-CPA).

    phantom=True keeps the geometry but writes meas_range = 1.6 m against
    gt_range 16 -> 4.6 m -- the own-prop phantom signature (implied range
    disagrees with gt by >> 50%), so every ENGAGE detection must classify
    as NOT the real target.

    engage=False stops after the dash (a legitimate pre-ENGAGE abort: the
    audit_per_tick SKIPPED / vacuous-arm case).
    """
    rows = []
    t = 0.0
    for _ in range(3):
        rows.append({"t": f"{t:.2f}", "phase": "TAKEOFF", "law": "pronav",
                     "detected": "0", "cmd_vn": "0.0", "cmd_ve": "0.0",
                     "cmd_vd": "0.0", "alt_m": "2.0"})
        t += 0.2
    for i in range(5):
        rows.append({"t": f"{t:.2f}", "phase": "CODED_DASH", "law": "pronav",
                     "detected": "0", "cmd_vn": "8.0", "cmd_ve": "0.0",
                     "cmd_vd": "0.0", "alt_m": "12.0",
                     "gt_range": f"{18.0 - 0.4 * i:.2f}"})
        t += 0.2
    if engage:
        for i in range(20):
            gt = 16.0 - 0.6 * i               # closes 16.0 -> 4.6 m
            lam = -10.0 + i                   # 20-deg LOS sweep
            mr = 1.6 if phantom else gt * 1.05
            rows.append({
                "t": f"{t:.2f}", "phase": "ENGAGE", "law": "pronav",
                "detected": "1",
                "meas_range": f"{mr:.3f}", "gt_range": f"{gt:.3f}",
                "lambda_deg": f"{lam:.3f}",
                "cmd_vn": f"{8.0 * math.cos(math.radians(lam)):.6f}",
                "cmd_ve": f"{8.0 * math.sin(math.radians(lam)):.6f}",
                "cmd_vd": "0.0", "vc_m_s": "6.0", "alt_m": "12.0",
                "gt_cam_x": "0.0", "gt_cam_y": "0.0", "gt_cam_z": "12.0",
                "gt_tag_x": f"{gt * math.sin(math.radians(lam)):.3f}",
                "gt_tag_y": f"{gt * math.cos(math.radians(lam)):.3f}",
                "gt_tag_z": "12.0",
            })
            t += 0.2
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=header, restval="")
        w.writeheader()
        for r in rows:
            unknown = set(r) - set(header)
            assert not unknown, (
                f"fixture writes column(s) the PRODUCER does not: {sorted(unknown)}"
                " -- the contract moved; update this helper from CSV_HEADER")
            w.writerow(r)
    return path


def write_arm_csv(path, header, flight_rows):
    """An mc_batch-shaped arm CSV through the producer's own header.
    flight_rows: dicts keyed by arm-CSV column names (unset -> '')."""
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=header, restval="")
        w.writeheader()
        for r in flight_rows:
            unknown = set(r) - set(header)
            assert not unknown, (
                f"arm fixture writes column(s) the PRODUCER does not: {sorted(unknown)}")
            w.writerow(r)
    return path
