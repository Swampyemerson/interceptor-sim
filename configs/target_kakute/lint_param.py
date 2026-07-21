#!/usr/bin/env python3
"""lint_param.py -- assert configs/target_kakute/target.param is a well-formed
ArduPilot / Mission Planner parameter file.

Checks (exit 0 = pass, 1 = fail):
  * every non-comment, non-blank line is exactly  PARAM_NAME,VALUE
  * PARAM_NAME is a valid ArduPilot param token: [A-Z][A-Z0-9_]* and <=16 chars
  * VALUE parses as a float (ArduPilot params are numeric)
  * NO duplicate parameter names (the classic "last one silently wins" footgun)
  * spot-checks that the task-mandated params are present with sane values

No hardware, no sim, no third-party deps. Usage:
  python3 lint_param.py [path-to-target.param]
"""
from __future__ import annotations

import os
import re
import sys

NAME_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")
DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "target.param")

# (param, predicate, human description) -- the task's required, sane values.
REQUIRED = [
    ("MOT_PWM_TYPE",  lambda v: 4 <= v <= 7,        "DShot (Tekko32 4-in-1)"),
    ("SERVO_BLH_AUTO", lambda v: v == 1,            "BLHeli passthrough on"),
    ("BATT_MONITOR",  lambda v: v == 4,             "analog V+I current sense"),
    ("BRD_ALT_CONFIG", lambda v: v == 1,            "R6/T6 remapped to USART6 for CRSF"),
    ("SERIAL6_PROTOCOL", lambda v: v == 23,         "RCIN (CRSF) on SERIAL6"),
    ("RSSI_TYPE",     lambda v: v == 3,             "RSSI from receiver protocol"),
    ("WPNAV_SPEED",   lambda v: v >= 900,           ">=9 m/s AUTO legs"),
    ("LOG_BITMASK",   lambda v: v > 0,              "DataFlash logging on (POS+GPS)"),
    ("ARMING_CHECK",  lambda v: v == 1,             "all arming checks ON"),
    ("FS_OPTIONS",    lambda v: True,               "failsafe options (continue-in-AUTO)"),
    ("FENCE_ENABLE",  lambda v: v == 1,             "geofence on"),
    ("FS_THR_ENABLE", lambda v: v >= 1,             "RC failsafe armed"),
]


def lint(path: str) -> int:
    errors: list[str] = []
    seen: dict[str, int] = {}
    values: dict[str, float] = {}

    with open(path) as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.rstrip("\n")
            stripped = line.strip()
            if stripped == "" or stripped.startswith("#"):
                continue
            if "," not in stripped:
                errors.append(f"line {lineno}: no comma separator: {stripped!r}")
                continue
            parts = stripped.split(",")
            if len(parts) != 2:
                errors.append(
                    f"line {lineno}: expected NAME,VALUE (one comma), got {stripped!r}")
                continue
            name, val = parts[0].strip(), parts[1].strip()
            if not NAME_RE.match(name):
                errors.append(f"line {lineno}: invalid param name {name!r}")
                continue
            if len(name) > 16:
                errors.append(f"line {lineno}: param name >16 chars: {name!r}")
            try:
                fval = float(val)
            except ValueError:
                errors.append(f"line {lineno}: value {val!r} is not numeric ({name})")
                continue
            if name in seen:
                errors.append(
                    f"line {lineno}: DUPLICATE param {name} (first seen line {seen[name]})")
            else:
                seen[name] = lineno
                values[name] = fval

    # required-param spot checks
    for name, pred, desc in REQUIRED:
        if name not in values:
            errors.append(f"MISSING required param {name} ({desc})")
        elif not pred(values[name]):
            errors.append(
                f"param {name}={values[name]} fails the sane-value check ({desc})")

    if errors:
        print(f"LINT FAILED ({path}):")
        for e in errors:
            print("  - " + e)
        return 1
    print(f"LINT PASSED: {len(values)} params, no duplicates, "
          f"all {len(REQUIRED)} required params present + sane ({path}).")
    return 0


if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH
    sys.exit(lint(target))
