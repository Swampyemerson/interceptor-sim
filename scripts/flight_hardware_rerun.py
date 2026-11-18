#!/usr/bin/env python3
"""Approach dash on the FLYING hardware configuration (isim, real flight code).
PRE-REGISTERED: isim/specs/flight_hardware_rerun_prereg_2026-11-18.md -- read it
before interpreting output.

The 2026-11-18 drag screen (scripts/approach_dash_drag_sweep.py) ran on October's
Gazebo-matched optics: fx 540.3, a 0.5 m tag, no camera up-tilt. This runner
re-asks the same questions with the camera, tag, tilt, decode cadence and
flight-controller envelope the real aircraft has. It imports the sweep's
scenario builder and per-run function unchanged; only Scenario fields differ.

  --stage check     my wrapper, fed October's optics, must reproduce 16 rows of
                    runs/approach_dash_drag_20261118/stage1.json exactly
  --stage a         no random errors: 3 geometries x drag D0/D1/D2 x envelope
                    now/lean60 x brake 2/3/4, tilt 10
  --stage c         camera tilt 15 / 20 / 25 (tilt 10 is stage a), D1 and D2
  --stage brackets  fx 933 instead of 1104; side-facing tag instead of rear
  --stage b         all errors on, 50 seeds: speed-belief error 10/4/2/1/0 % x
                    straight/weave x near/demo x lost-target clock 2 s / 6 s
  --stage btilt     stage b's 4 % and 1 % straight cells at tilt 20
Rows land in runs/flight_hardware_rerun_20261118/<stage>.json.
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import numpy as np                                                    # noqa: E402
import approach_dash_drag_sweep as sw                                 # noqa: E402
from approach_dash_drag_sweep import run_one, run_jobs, scenario      # noqa: E402,F401

OUT = os.path.join(ROOT, "runs/flight_hardware_rerun_20261118")

# ---- the flying configuration (sources in the prereg's table) ---------------
FX_CAL = 1104.0          # configs/ov9281_intrinsics.json (checkerboard, measured)
FX_FOV_EQUIV = 933.0     # the repo's earlier "70 degree" pinhole stand-in
TAG_M = 0.35             # docs/placard_sizing.md: the print-ready placard's black square
DECODE_DT = 0.05         # flight loop --fps 20 (the Pi decodes at 96.6 fps, measured)
DASH_CAP = 33            # the board's speed limit as of 2026-11-18
OPTICS = dict(cam_fx_px=FX_CAL, cam_width_px=1280.0, cam_height_px=800.0,
              tag_side_m=TAG_M, tag_facing="rear",
              decode_overrides={"min_decode_interval_s": DECODE_DT})
# Truth-side flight-controller envelopes laid over the ENGAGE fit. Guidance is
# not told. Thrust 4 g on both (the real airframe measured about 5 g; the x500
# fit's 1.86 g cannot hold height at a 60 degree lean).
_BASE = {"tilt_sat_vertical_priority": True, "thrust_max": 39.2,
         "max_setpoint_speed_horiz": 33.0, "max_setpoint_accel_horiz": 5.0}
ENV = {"now": dict(_BASE, max_tilt_deg=45.0),      # as read off the board today
       "lean60": dict(_BASE, max_tilt_deg=60.0)}   # the open "raise the lean limit" option
GEOMS = ("near", "demo", "tail")
BRAKES = (2, 3, 4)


def fh_jobs(tag, geom, a, drag, env, seeds, *, tilt=10.0, optics=None, scatter=False,
            speed_err=None, motion="straight", clock=None, vmax=DASH_CAP):
    over = dict(ENV[env]) if isinstance(env, str) else dict(env)
    over["drag_quad_horiz"] = sw.DRAG[drag]
    extra = dict(OPTICS)
    extra.update(optics or {})
    extra["cam_tilt_up_deg"] = float(tilt)
    extra["target_motion"] = motion
    so = None if speed_err is None else {"target_speed_belief_sigma_frac": float(speed_err)}
    jobs = []
    for s in seeds:
        scn = scenario(geom, vmax, a, s, veh_over=over, scatter=scatter,
                       extra_scn=extra, scatter_over=so)
        jobs.append((tag, scn) if clock is None
                    else (tag, scn, dict(engage_lost_target_s=float(clock))))
    return jobs


def camera_kill(r):
    """A kill the camera earned: inside 0.35 m AND decoded before closest approach."""
    return (r["miss"] <= 0.35 and r["first_det"] is not None
            and r["first_det"] < r["t_cpa"])


def blind_kill(r):
    return r["miss"] <= 0.35 and not camera_kill(r)


def go_time(r):
    for part in r["reasons"].split(" || "):
        if part.startswith("ENGAGE@"):
            return float(part.split("@")[1].split(":")[0])
    return 0.0


def table(rows, expect_n):
    cells = {}
    for r in rows:
        cells.setdefault(r["tag"], []).append(r)
    print(f"{'cell':>34} {'n':>3} {'kill':>5} {'cam':>4} {'blind':>5} {'nodet':>5} {'med':>7} "
          f"{'p90':>7} {'t_cpa':>6} {'late':>4} {'peak':>5} {'v@det':>6} {'lean@det':>8} {'ndec':>5}")
    bad = 0
    for tag, rs in cells.items():
        if len(rs) != expect_n:
            print(f"{tag:>34} UNCERTAIN -- {len(rs)} rows, expected {expect_n}")
            bad += 1
            continue
        m = np.array([r["miss"] for r in rs])
        det = [r for r in rs if r["first_det"] is not None]
        med = lambda k, src: (np.median([r[k] for r in src]) if src else float("nan"))  # noqa: E731
        late = sum(1 for r in rs if r["miss"] <= 0.35 and r["t_cpa"] - go_time(r) > 30.0)
        print(f"{tag:>34} {len(rs):3d} {int((m <= 0.35).sum()):5d} "
              f"{sum(camera_kill(r) for r in rs):4d} {sum(blind_kill(r) for r in rs):5d} "
              f"{len(rs) - len(det):5d} {np.median(m):7.3f} {np.percentile(m, 90):7.3f} "
              f"{med('t_cpa', rs):6.1f} {late:4d} {med('peak_v', rs):5.1f} "
              f"{med('v_first', det):6.1f} {med('tilt_first', det):8.1f} {med('n_dec', rs):5.0f}")
    return bad


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", required=True,
                    choices=["check", "a", "c", "brackets", "b", "btilt"])
    ap.add_argument("--workers", type=int, default=12)
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)

    if args.stage == "check":
        ref = {r["seed"]: r for r in json.load(open(os.path.join(
            ROOT, "runs/approach_dash_drag_20261118/stage1.json")))
            if r["tag"] == "near|v36a4|D1|R"}
        october = dict(cam_fx_px=540.3, tag_side_m=0.5, tag_facing="rear",
                       decode_overrides={"min_decode_interval_s": 0.096})
        jobs = fh_jobs("check", "near", 4, "D1", sw.PLANTS["R"], range(16), tilt=0.0,
                       optics=october, vmax=36)
        ok = len(ref) == 16
        for r in run_jobs(jobs, args.workers):
            o = ref[r["seed"]]
            same = abs(r["miss"] - o["miss"]) < 1e-9 and abs(r["t_cpa"] - o["t_cpa"]) < 1e-9
            ok &= same
            if not same:
                print(f"seed {r['seed']}: {r['miss']:.6f} vs {o['miss']:.6f} DIFFERS")
        print("INSTRUMENT CHECK", "PASS (16 of 16 rows identical)" if ok else "FAIL")
        return 0 if ok else 1

    jobs = []
    if args.stage == "a":
        n = 16
        for geom in GEOMS:
            for env in ENV:
                for drag in sw.DRAG:
                    for a in BRAKES:
                        jobs += fh_jobs(f"{geom}|{env}|{drag}|a{a}|t10", geom, a, drag, env, range(n))
    elif args.stage == "c":
        n = 16
        for tilt in (15, 20, 25):
            for geom in GEOMS:
                for env in ENV:
                    for drag in ("D1", "D2"):
                        jobs += fh_jobs(f"{geom}|{env}|{drag}|a3|t{tilt}", geom, 3, drag, env,
                                        range(n), tilt=tilt)
    elif args.stage == "brackets":
        n = 16
        for geom in GEOMS:
            for env in ENV:
                jobs += fh_jobs(f"fx933|{geom}|{env}|D1|a3|t10", geom, 3, "D1", env, range(n),
                                optics=dict(cam_fx_px=FX_FOV_EQUIV))
                jobs += fh_jobs(f"sidetag|{geom}|{env}|D1|a3|t10", geom, 3, "D1", env, range(n),
                                optics=dict(tag_facing="side"))
    elif args.stage == "b":
        n = 50
        for motion in ("straight", "weave"):
            for geom in ("near", "demo"):
                for err in (0.10, 0.04, 0.02, 0.01, 0.0):
                    for clock in (2.0, 6.0):
                        tag = f"{motion[:2]}|{geom}|e{err * 100:g}|T{clock:g}"
                        jobs += fh_jobs(tag, geom, 3, "D1", "now", range(n), scatter=True,
                                        speed_err=err, motion=motion, clock=clock)
    else:  # btilt
        n = 50
        for geom in ("near", "demo"):
            for err in (0.04, 0.01):
                jobs += fh_jobs(f"st|{geom}|e{err * 100:g}|T2|t20", geom, 3, "D1", "now", range(n),
                                tilt=20.0, scatter=True, speed_err=err, clock=2.0)

    rows = run_jobs(jobs, args.workers)
    json.dump(rows, open(os.path.join(OUT, f"{args.stage}.json"), "w"))
    bad = table(rows, n)
    if len(rows) != len(jobs) or bad:
        print(f"UNCERTAIN: {len(rows)} rows for {len(jobs)} jobs, {bad} short cells")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
