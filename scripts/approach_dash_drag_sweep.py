#!/usr/bin/env python3
"""Approach-dash drag robustness sweep (isim, real flight code port arm).
PRE-REGISTERED: isim/specs/approach_dash_drag_prereg_2026-11-18.md -- read it
before interpreting output.

Reconstructs the 2026-10-21 approach-dash sweep setup (isim/specs/
approach_dash_2026-10-21.md; `--stage check` reproduces rows of
runs/cinema_longrange_20261021/dash_sweep.json) and adds truth-only body drag
through Scenario.vehicle_overrides. Guidance is never told the drag.

  --stage check   instrument check against the October table (must match)
  --stage 1       no scatter, 16 paired seeds, 3 geometries x 3 profiles x
                  3 drag tiers x 2 plants
  --stage 2       all errors on (Scatter()), 50 paired seeds, plant R
Rows land in runs/approach_dash_drag_20261118/stage{1,2}.json."""
import contextlib, dataclasses, io, json, math, os, sys
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import numpy as np
from isim.replay_a0 import load_params
from isim.scenario import Scenario, Scatter, build
from isim.engine import run_engagement

FIT = os.path.join(ROOT, "isim/fits/vehicle_gazebo_x500_engage.json")
GEOMS = {"near": dict(cross_range_m=6.5, lead_dist_m=500.0),    # October registered
         "demo": dict(cross_range_m=150.0, lead_dist_m=477.0),  # Gazebo take geometry
         "tail": dict(cross_range_m=500.0, lead_dist_m=500.0)}  # longest transit
PROFILES = [(20, 3), (36, 4), (36, 3)]          # (dash_v_max_ms, brake_accel_ms2)
# Quadratic horizontal body drag, 1/m, ADDED to the fitted linear term.
# k = 0.5 * rho * CdA / m with rho 1.042 (Boulder), m 0.853 kg.
DRAG = {"D0": 0.0, "D1": 0.0086, "D2": 0.0171}  # none / CdA 0.014 m2 / twice that
PLANTS = {"X": {},                               # ENGAGE fit as fitted (tilt 37.7 deg)
          "R": {"max_tilt_deg": 60.0, "thrust_max": 39.2}}  # demo tilt limit, 4 g
OUT = os.path.join(ROOT, "runs/approach_dash_drag_20261118")


def scenario(geom, vmax, a, seed, veh_over=None, scatter=False, extra_port=None,
             extra_scn=None):
    port = dict(approach_dash=True, dash_v_max_ms=float(vmax), brake_shaping=True,
                brake_accel_ms2=float(a), brake_vert_sync=True)
    if extra_port:
        port.update(extra_port)
    kw = dict(concept="flyby", terminal="pursuit", tag_facing="rear",
              tag_side_m=0.5, cam_fx_px=540.3, target_speed_ms=12.0,
              pursuit_window_s=120.0,
              decode_overrides={"min_decode_interval_s": 0.096},
              port_pursuit_overrides=port, seed=int(seed),
              vehicle_overrides=dict(veh_over) if veh_over else None,
              scatter=Scatter() if scatter else None, **GEOMS[geom])
    if extra_scn:
        kw.update(extra_scn)
    return Scenario(**kw)


def tilt_deg(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))


def run_one(job):
    tag, scn = job
    vp = load_params(FIT)
    ecfg, vehicle, target, seeker, guidance, init_state = build(scn, vp)
    guidance.supply_pose_range = True
    with contextlib.redirect_stdout(io.StringIO()):
        res = run_engagement(ecfg, vehicle, target, seeker, guidance, init_state,
                             record_trace=True)
    tr = res.trace
    n = int(np.count_nonzero(tr["t"])) + 1
    t = tr["t"][:n]; v = np.linalg.norm(tr["own_vel"][:n], axis=1)
    vh = np.linalg.norm(tr["own_vel"][:n, :2], axis=1)
    tl = tilt_deg(tr["quat"][:n]); det = tr["det_new"][:n] > 0.5
    rng = tr["range"][:n]
    cmd = np.linalg.norm(tr["cmd"][:n, :2], axis=1)
    row = dict(tag=tag, seed=scn.seed, miss=float(res.miss_m), t_cpa=float(res.t_cpa),
               miss_h=float(res.miss_horiz_m), miss_v=float(res.miss_vert_m),
               n_dec=int(res.n_decoded), n_in_fov=int(res.n_in_fov),
               frac_sat=float(res.frac_saturated), peak_v=float(v.max()),
               max_tilt=float(tl.max()), closing=float(res.closing_speed_ms))
    if det.any():
        i = int(np.argmax(det))
        row.update(first_det=float(t[i]), v_first=float(v[i]), tilt_first=float(tl[i]),
                   range_first=float(rng[i]), lock_s=float(res.t_cpa - t[i]))
    else:
        row.update(first_det=None, v_first=None, tilt_first=None, range_first=None,
                   lock_s=None)
    # tracking shortfall while fast: largest (cmd speed - own horiz speed) once cmd > 15
    m = cmd > 15.0
    row["cmd_peak"] = float(cmd.max())
    row["shortfall_peak"] = float((cmd[m] - vh[m]).max()) if m.any() else 0.0
    # altitude excursion (down axis) over the whole run, relative to start
    d = tr["own_pos"][:n, 2]
    row["alt_drop_max"] = float((d - d[0]).max()); row["alt_rise_max"] = float((d[0] - d).max())
    st = list(getattr(guidance, "state_log", []))
    row["transitions"] = "|".join(f"{s}@{tt:.1f}" for tt, s in st)
    return row


def run_jobs(jobs, workers=12):
    ctx = get_context("spawn")
    with ProcessPoolExecutor(max_workers=workers, mp_context=ctx) as ex:
        return list(ex.map(run_one, jobs, chunksize=1))


def cell_jobs(geom, vmax, a, drag, plant, seeds, scatter=False, extra_scn=None, label=None):
    over = dict(PLANTS[plant]); over["drag_quad_horiz"] = DRAG[drag]
    tag = label or f"{geom}|v{vmax}a{a}|{drag}|{plant}"
    return [(tag, scenario(geom, vmax, a, s, veh_over=over, scatter=scatter,
                           extra_scn=extra_scn)) for s in seeds]


def summarize(rows, expect_n):
    """One line per cell. A cell with a wrong row count is UNCERTAIN, never a pass."""
    cells = {}
    for r in rows:
        cells.setdefault(r["tag"], []).append(r)
    print(f"{'cell':>26} {'n':>3} {'kill':>6} {'<=1m':>5} {'med':>7} {'p90':>7} "
          f"{'t_cpa':>6} {'peak':>5} {'v@det':>6} {'tilt@det':>8} {'ndec':>5} {'nodet':>5}")
    bad = 0
    for tag, rs in cells.items():
        if len(rs) != expect_n:
            print(f"{tag:>26} UNCERTAIN -- {len(rs)} rows, expected {expect_n}"); bad += 1
            continue
        m = np.array([r["miss"] for r in rs])
        det = [r for r in rs if r["first_det"] is not None]
        med = lambda k, src: (np.median([r[k] for r in src]) if src else float("nan"))
        print(f"{tag:>26} {len(rs):3d} {int((m <= 0.35).sum()):3d}/{len(rs):<2d} "
              f"{int((m <= 1.0).sum()):5d} {np.median(m):7.3f} {np.percentile(m, 90):7.3f} "
              f"{med('t_cpa', rs):6.1f} {med('peak_v', rs):5.1f} {med('v_first', det):6.1f} "
              f"{med('tilt_first', det):8.1f} {med('n_dec', rs):5.0f} {len(rs) - len(det):5d}")
    return bad


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", required=True, choices=["check", "1", "2"])
    ap.add_argument("--workers", type=int, default=12)
    args = ap.parse_args()
    if args.stage == "check":
        ref = {(r_["vmax"], r_["a"]): r_ for r_ in json.load(open(os.path.join(
            ROOT, "runs/cinema_longrange_20261021/dash_sweep.json"))) if r_["seed"] == 0}
        # The October 16 m/s row is the legacy control: approach_dash OFF.
        jobs = [(f"{v}|{a}", scenario("near", v, a, 0, extra_port=(
                    dict(approach_dash=False) if v == 16 else None)))
                for v, a in ((16, 3), (20, 3), (36, 3), (36, 4))]
        ok = True
        for r in run_jobs(jobs, 4):
            v, a = (float(x) for x in r["tag"].split("|")); o = ref[(v, a)]
            same = (abs(r["miss"] - o["miss"]) < 1e-6 and abs(r["t_cpa"] - o["t_cpa"]) < 1e-6
                    and abs(r["peak_v"] - o["peak_v"]) < 1e-6)
            ok &= same
            print(f"v{v:.0f} a{a:.0f}: miss {r['miss']:.6f} vs {o['miss']:.6f}  "
                  f"t_cpa {r['t_cpa']:.4f} vs {o['t_cpa']:.4f}  {'MATCH' if same else 'DIFFERS'}")
        print("INSTRUMENT CHECK", "PASS" if ok else "FAIL")
        return 0 if ok else 1
    os.makedirs(OUT, exist_ok=True)
    jobs = []
    if args.stage == "1":
        n = 16
        for geom in GEOMS:
            for vmax, a in PROFILES:
                for plant in PLANTS:
                    for drag in DRAG:
                        jobs += cell_jobs(geom, vmax, a, drag, plant, range(n))
    else:
        n = 50
        for motion in ("straight", "weave"):
            for geom in ("near", "demo"):
                for vmax, a in ((20, 3), (36, 4)):
                    for drag in DRAG:
                        jobs += cell_jobs(geom, vmax, a, drag, "R", range(n), scatter=True,
                                          extra_scn=dict(target_motion=motion),
                                          label=f"{motion[:2]}|{geom}|v{vmax}a{a}|{drag}")
    rows = run_jobs(jobs, args.workers)
    json.dump(rows, open(os.path.join(OUT, f"stage{args.stage}.json"), "w"))
    bad = summarize(rows, n)
    if len(rows) != len(jobs) or bad:
        print(f"UNCERTAIN: {len(rows)} rows for {len(jobs)} jobs, {bad} short cells")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
