#!/usr/bin/env python3
"""Replay the past-closest-approach BREAKOFF events in flown camera-arm logs and ask:
would an absolute-range gate (--breakoff-max-range-m) have blocked the PREMATURE
ones and left the LEGITIMATE ones alone?  (docs/scoring_fix_plan.md section 5, issue #3)

SCORING/FORENSIC tool: reads gt_range (ground truth) to label events. Not on any
guidance path.

  premature  = the breakoff fired, then the TRUE range still closed > 0.5 m
  legitimate = everything else (true closest approach was already behind it)
  fire range = the last camera-MEASURED range before the first BREAKOFF tick --
               the quantity the gate actually tests in flight

Only breakoffs whose OWN run-log `breakoff_reason` names the range-increase rule are
counted; dropout/timeout breakoffs are tallied separately and NOT judged (the gate
does not act on them). Every arm row lands in exactly one bucket and the buckets
are printed, so a shrinking denominator is visible.

usage: breakoff_gate_replay.py [--gate 5.0] ARM_INDEX.csv [...]      exit 0 always
       breakoff_gate_replay.py --self-test                            exit 0/1
"""
import argparse
import csv
import os
import sys
import tempfile

PREMATURE_CLOSE_M = 0.5


def _f(row, key):
    v = row.get(key)
    return float(v) if v not in (None, "") else None


def classify_flight(rows):
    """-> dict(fire_range, closed_after) or None when the flight never reaches BREAKOFF."""
    bi = next((i for i, r in enumerate(rows) if r.get("phase") == "BREAKOFF"), None)
    if bi is None:
        return None
    fire_range = None
    for r in reversed(rows[:bi]):
        if r.get("detected") == "1" and _f(r, "meas_range") is not None:
            fire_range = _f(r, "meas_range")
            break
    gt_fire = next((_f(r, "gt_range") for r in reversed(rows[:bi + 1])
                    if _f(r, "gt_range") is not None), None)
    after = [_f(r, "gt_range") for r in rows[bi:] if _f(r, "gt_range") is not None]
    if fire_range is None or gt_fire is None or not after:
        return {"fire_range": fire_range, "closed_after": None}
    return {"fire_range": fire_range, "closed_after": gt_fire - min(after)}


def m4_breakoff_reason(run_log_path):
    """The flight's OWN stated reason, from its '[m4] law=... breakoff_reason=...'
    summary line. (The arm index's breakoff_reason column is a batch-level status
    like 'python_exit_1', not this.) '' when the log or the line is absent."""
    try:
        for line in open(run_log_path, errors="replace"):
            if line.startswith("[m4] law=") and "breakoff_reason=" in line:
                return line.split("breakoff_reason=", 1)[1].split(" aborted=", 1)[0]
    except OSError:
        pass
    return ""


def replay(arm_paths, gate_m):
    buckets = {"no_breakoff": 0, "other_reason": 0, "unscorable": 0,
               "missing_log": 0, "judged": 0}
    events = []
    for arm in arm_paths:
        for a in csv.DictReader(open(arm)):
            reason = m4_breakoff_reason(a.get("run_log_path") or "")
            p = a.get("flight_csv_path") or ""
            if not p or not os.path.exists(p):
                buckets["missing_log"] += 1
                continue
            rows = list(csv.DictReader(open(p)))
            ev = classify_flight(rows)
            if ev is None:
                buckets["no_breakoff"] += 1
            elif "range increased" not in reason.lower():
                buckets["other_reason"] += 1
            elif ev["closed_after"] is None:
                buckets["unscorable"] += 1
            else:
                buckets["judged"] += 1
                ev["frac"] = passage_fraction(a, rows)
                ev.update(arm=os.path.basename(arm), idx=a.get("run_idx"),
                          premature=ev["closed_after"] > PREMATURE_CLOSE_M,
                          blocked=ev["fire_range"] > gate_m)
                events.append(ev)
    return buckets, events


def passage_fraction(arm_row, rows):
    """flown / planned at the first BREAKOFF tick, for the PASSAGE gate
    (m4_intercept.py --breakoff-min-flown-frac). planned = the pre-flight accel-aware
    lead solve's intercept distance, recomputed here from the arm row's target
    start/velocity with the same solver and the same fitted constants (16 m/s,
    10 m/s^2). flown = displacement from the first CODED_DASH tick, using gt_cam as
    the offline stand-in for the vehicle's own EKF position (in flight the gate
    reads own-state only). None when it cannot be computed -- counted by the caller."""
    import math
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    from flight.guidance import collision_lead_heading_accel, dash_ramp_distance
    try:
        tp = (float(arm_row["target_start_x"]), float(arm_row["target_start_y"]))
        tv = (float(arm_row["target_vx"]), float(arm_row["target_vy"]))
    except (KeyError, ValueError):
        return None
    _, t_lead = collision_lead_heading_accel(tp, tv, 16.0, 10.0)
    if t_lead is None:
        return None
    planned = dash_ramp_distance(16.0, 10.0, t_lead)
    d = [r for r in rows if r.get("phase") in ("CODED_DASH", "ENGAGE", "BREAKOFF")
         and _f(r, "gt_cam_x") is not None and _f(r, "gt_cam_y") is not None]
    bi = next((i for i, r in enumerate(d) if r["phase"] == "BREAKOFF"), None)
    if bi is None or not d or d[0]["phase"] != "CODED_DASH" or planned <= 0:
        return None
    flown = math.hypot(_f(d[bi], "gt_cam_x") - _f(d[0], "gt_cam_x"),
                       _f(d[bi], "gt_cam_y") - _f(d[0], "gt_cam_y"))
    return flown / planned


def report(buckets, events, gate_m):
    total = sum(buckets.values())
    print(f"[breakoff_gate_replay] {total} arm rows: " +
          ", ".join(f"{k}={v}" for k, v in buckets.items()))
    if not events:
        print("UNCERTAIN / VACUOUS: zero range-increase breakoff events were judged.")
        return
    prem = [e for e in events if e["premature"]]
    legit = [e for e in events if not e["premature"]]
    for name, grp in (("PREMATURE", prem), ("legitimate", legit)):
        fr = sorted(e["fire_range"] for e in grp)
        rng = f"{fr[0]:.2f}-{fr[-1]:.2f} m" if fr else "n/a"
        print(f"  {name:10s} n={len(grp):3d}  fire range {rng}  "
              f"blocked by a {gate_m:.1f} m gate: {sum(e['blocked'] for e in grp)}/{len(grp)}")
    have = [e for e in events if e.get("frac") is not None]
    print(f"  PASSAGE gate (flown/planned), computable on {len(have)}/{len(events)} events:")
    for thr in (0.7, 0.8, 0.9):
        p = [e for e in have if e["premature"]]
        l = [e for e in have if not e["premature"]]
        print(f"    allow breakoff only at >= {thr:.1f} x planned: blocks premature "
              f"{sum(e['frac'] < thr for e in p)}/{len(p)}, blocks legitimate "
              f"{sum(e['frac'] < thr for e in l)}/{len(l)}")
    for e in sorted(prem, key=lambda e: -e["closed_after"]):
        print(f"    premature: {e['arm']} #{e['idx']} fired at measured {e['fire_range']:.2f} m, "
              f"true range then closed {e['closed_after']:.2f} m more "
              f"-> {'BLOCKED' if e['blocked'] else 'NOT blocked'}")


def self_test():
    ok = True

    def case(name, cond):
        nonlocal ok
        print(("  ok   " if cond else "  FAIL ") + name)
        ok = ok and cond

    def flight(fire_meas, gts_after):
        rows = [{"phase": "ENGAGE", "detected": "1", "meas_range": "9.0", "gt_range": "9.0"},
                {"phase": "ENGAGE", "detected": "1", "meas_range": str(fire_meas), "gt_range": "6.0"}]
        rows += [{"phase": "BREAKOFF", "detected": "0", "meas_range": "", "gt_range": str(g)}
                 for g in gts_after]
        return rows

    ev = classify_flight(flight(6.2, [5.5, 3.0, 2.0, 2.5]))
    case("premature: true range closes 3.5 m after the first BREAKOFF tick",
         abs(ev["closed_after"] - 3.5) < 1e-9)
    case("fire range is the last MEASURED range", ev["fire_range"] == 6.2)
    ev = classify_flight(flight(2.0, [6.1, 6.5, 7.0]))
    case("legitimate: nothing left to close", ev["closed_after"] <= 0.0 + 1e-9)
    case("no BREAKOFF phase -> None, not a verdict",
         classify_flight([{"phase": "ENGAGE", "gt_range": "3"}]) is None)
    with tempfile.TemporaryDirectory() as d:
        arm = os.path.join(d, "arm.csv")
        with open(arm, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["run_idx", "run_log_path", "flight_csv_path"])
            w.writerow(["0", os.path.join(d, "nolog.log"), os.path.join(d, "missing.csv")])
        b, e = replay([arm], 5.0)
        case("a missing per-tick log is COUNTED, not dropped", b["missing_log"] == 1 and not e)
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("arms", nargs="*")
    ap.add_argument("--gate", type=float, default=5.0)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        sys.exit(self_test())
    if not args.arms:
        ap.error("give at least one arm index CSV")
    report(*replay(args.arms, args.gate), args.gate)
