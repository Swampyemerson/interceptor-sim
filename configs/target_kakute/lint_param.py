#!/usr/bin/env python3
"""lint_param.py -- assert configs/target_kakute/target.param is a well-formed
ArduPilot / Mission Planner parameter file.

Checks (exit 0 = pass, 1 = fail):
  * every non-comment, non-blank line is exactly  PARAM_NAME,VALUE
  * PARAM_NAME is a valid ArduPilot param token: [A-Z][A-Z0-9_]* and <=16 chars
  * VALUE parses as a float (ArduPilot params are numeric)
  * NO duplicate parameter names (the classic "last one silently wins" footgun)
  * the required params are PRESENT with SANE VALUES (REQUIRED), including the
    whole OPTION-A safety set -- fence, battery thresholds and actions, RC kill
  * cross-param INVARIANTS (INVARIANTS) no per-param range check can express

NOT CHECKED, and say so rather than implying otherwise: whether a param name
exists in the target firmware AT ALL. That needs a parameter manifest for the
pinned release, and ArduPilot publishes no versioned apm.pdef for 4.7.0 (only up
to stable-4.6.0, checked 2026-07-26) -- and the 4.6.0 manifest still lists the
RETIRED GPS_TYPE, so a naive name-membership test would have passed the very
defect that motivated this. The renames are instead pinned by naming the 4.7
spellings in REQUIRED, so a revert to an old name FAILS as a missing param.

No hardware, no sim, no third-party deps, NO network. Usage:
  python3 lint_param.py [path-to-target.param]
"""
from __future__ import annotations

import os
import re
import sys

NAME_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")
DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "target.param")

# FIRMWARE PIN. Param NAMES and UNITS move between ArduPilot releases (4.6 renamed
# GPS_TYPE -> GPS1_TYPE; 4.7 renamed WPNAV_* -> WP_* and changed cm/s -> m/s,
# ANGLE_MAX -> ATC_ANGLE_MAX cdeg -> deg, ARMING_CHECK -> ARMING_SKIPCHK with the
# sense INVERTED). Both this list and target.param are written for the version
# below; see target.param's header for the per-name source citations.
TARGET_FIRMWARE = "ArduCopter 4.7.0"

# (param, predicate, human description) -- the required params with SANE-VALUE
# checks. Two jobs, both of which the old 12-entry list did badly:
#   (a) REQUIRED-PRESENT is also the typo net. `FENCE_RADUS,80` leaves
#       FENCE_RADIUS missing -> a named failure, instead of a silent revert to
#       the 300 m firmware default with LINT PASSED printed over it.
#   (b) VALUE checks on the params that carry the safety argument. OPTION A's
#       whole case is "continue the AUTO pass on RC-loss, with a TIGHT geofence
#       as the hard bound" -- so the fence and battery numbers ARE the safety
#       envelope, and none of them was value-checked at all. FS_OPTIONS's
#       predicate was literally `lambda v: True`, i.e. presence reported as
#       "sane" while the description claimed "continue-in-AUTO".
# Predicates are written against the FIRMWARE's own ranges/defaults where one
# exists, so they can actually fail, rather than restating target.param's value.
REQUIRED = [
    ("MOT_PWM_TYPE",  lambda v: 4 <= v <= 7,        "DShot (Tekko32 4-in-1)"),
    ("SERVO_BLH_AUTO", lambda v: v == 1,            "BLHeli passthrough on"),
    ("BATT_MONITOR",  lambda v: v == 4,             "analog V+I current sense"),
    ("BRD_ALT_CONFIG", lambda v: v == 1,            "R6/T6 remapped to USART6 for CRSF"),
    ("SERIAL6_PROTOCOL", lambda v: v == 23,         "RCIN (CRSF) on SERIAL6"),
    ("RSSI_TYPE",     lambda v: v == 3,             "RSSI from receiver protocol"),
    # 4.7 names. Present here so a revert to the 4.5/4.6 spelling FAILS the lint
    # instead of being silently dropped by Mission Planner on build day.
    ("GPS1_TYPE",     lambda v: v >= 0,             "GPS type set (4.6+ name, was GPS_TYPE)"),
    ("WP_SPD",        lambda v: v >= 9.0,           ">=9 m/s AUTO legs (m/s! 4.7 name+unit)"),
    ("WP_ACC",        lambda v: 0.5 <= v <= 5.0,    "AUTO accel, firmware range 0.50-5.00 m/s/s"),
    ("ATC_ANGLE_MAX", lambda v: 10.0 <= v <= 80.0,  "lean limit, firmware range 10-80 DEG"),
    ("ARMING_SKIPCHK", lambda v: v == 0,            "skip NO arming checks (inverted 4.7 name)"),
    ("LOG_BITMASK",   lambda v: v > 0,              "DataFlash logging on (POS+GPS)"),
    # OPTION A's safety set. FENCE_* ranges/defaults from AC_Fence.cpp at
    # Copter-4.7.0: RADIUS @Range 30 10000, ALT_MAX @Range 10 1000, TYPE bitmask
    # bit0=Max altitude(1) bit1=Circle Centered on Home(2) bit2=Incl/Excl(4),
    # ACTION 0=Report Only .. 5=SmartRTL or Land.
    ("FS_OPTIONS",    lambda v: int(v) & 1 == 1,    "bit0 SET: continue if in AUTO on RC failsafe"),
    ("FS_THR_ENABLE", lambda v: v >= 1,             "RC failsafe armed"),
    ("FENCE_ENABLE",  lambda v: v == 1,             "geofence on"),
    ("FENCE_TYPE",    lambda v: int(v) & 3 == 3,    "altitude ceiling AND circle both armed"),
    ("FENCE_RADIUS",  lambda v: 30 <= v <= 150,     "TIGHT fence: >=30 m (firmware min) and well inside the 300 m default"),
    ("FENCE_ALT_MAX", lambda v: 10 <= v <= 120,     "altitude ceiling: >=10 m (firmware min), <=120 m"),
    ("FENCE_ACTION",  lambda v: v != 0,             "NOT 0 (Report Only) -- a fence that only reports is not a bound"),
    # 6S pack. Checked as VOLTS PER CELL, not against a copy of the file's own
    # number: a hardcoded 21.6 tests the restatement and false-fails the first
    # legitimate re-tune.
    ("BATT_LOW_VOLT", lambda v: 3.3 <= v / 6.0 <= 3.8, "6S low threshold, 3.3-3.8 V/cell"),
    ("BATT_CRT_VOLT", lambda v: 3.0 <= v / 6.0 <= 3.6, "6S critical threshold, 3.0-3.6 V/cell"),
    ("BATT_FS_LOW_ACT", lambda v: v != 0,           "low-battery action set (not None)"),
    ("BATT_FS_CRT_ACT", lambda v: v != 0,           "critical-battery action set (not None)"),
    ("RC6_OPTION",    lambda v: v == 31,            "ch6 = Motor Emergency Stop (RC kill)"),
]

# Cross-param invariants -- (description, predicate over the {name: value} dict).
# No per-param range check can express these, and each is a real fat-finger:
# every individual value can pass its own bounds while the PAIR is nonsense.
INVARIANTS = [
    ("BATT_CRT_VOLT < BATT_LOW_VOLT (critical must trip BELOW low)",
     lambda v: v["BATT_CRT_VOLT"] < v["BATT_LOW_VOLT"]),
    ("BATT_LOW_VOLT < MOT_BAT_VOLT_MAX (low threshold below the pack's full charge)",
     lambda v: v["BATT_LOW_VOLT"] < v["MOT_BAT_VOLT_MAX"]),
    ("FS_OPTIONS bit0 (continue-in-AUTO) requires a REAL fence: FENCE_ENABLE=1 "
     "and the circle bit set -- the geofence IS the safety argument for it",
     lambda v: (int(v["FS_OPTIONS"]) & 1) == 0
     or (v["FENCE_ENABLE"] == 1 and int(v["FENCE_TYPE"]) & 2 == 2)),
    ("FENCE_RADIUS > FENCE_MARGIN (a fence inside its own margin is instantly breached)",
     lambda v: "FENCE_MARGIN" not in v or v["FENCE_RADIUS"] > v["FENCE_MARGIN"]),
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

    # cross-param invariants (skipped, NOT silently passed, if an input is absent
    # -- the missing name is already reported above)
    for desc, pred in INVARIANTS:
        try:
            ok = pred(values)
        except KeyError as e:
            errors.append(f"INVARIANT unchecked, {e} is missing: {desc}")
            continue
        if not ok:
            errors.append(f"INVARIANT VIOLATED: {desc}")

    if errors:
        print(f"LINT FAILED ({path}):")
        for e in errors:
            print("  - " + e)
        return 1
    print(f"LINT PASSED [{TARGET_FIRMWARE}]: {len(values)} params, no duplicates, "
          f"all {len(REQUIRED)} required params present + sane, "
          f"{len(INVARIANTS)} cross-param invariants hold ({path}).")
    return 0


if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH
    sys.exit(lint(target))
