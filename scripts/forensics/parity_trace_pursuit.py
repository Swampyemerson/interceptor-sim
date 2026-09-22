#!/usr/bin/env python3
"""Per-tick side-by-side estimator trace: native PursuitRendezvousGuidance vs
the flight-code port (flight/pursuit_terminal.py under RealFlightSM).

Registered instrument of isim/specs/parity_trace_2026-09-22.md -- read that
spec for the candidates, their predicted signatures, and the decision rule.

HONESTY: this is a FORENSICS/SCORING tool. It reads engine truth (result
trace) and Detection.t_capture to DIAGNOSE the guidance; nothing here feeds a
command. Guidance under test receives exactly what the harness always hands
it.

Usage:
  .venv/bin/python scripts/forensics/parity_trace_pursuit.py --seeds 0,1,2,3,4
Writes logs/parity_trace_2026-09-22/seed{S}_{arm}.csv and prints the summary
table the spec's decision rule reads.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from typing import List, Optional, Tuple

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

from isim.engine import run_engagement                      # noqa: E402
from isim.replay_a0 import load_params                      # noqa: E402
from isim.scenario import Scenario, build                   # noqa: E402
from flight.deploy.seeker_loop import measurement_from_box  # noqa: E402
from flight.tag_terminal import _cam_offset_ned, _optical_vec_to_ned  # noqa: E402

OUT_DIR = os.path.join(_REPO, "logs", "parity_trace_2026-09-22")
LAT_ASSUMED_S = 0.045          # the port's fixed meas_latency_s
FINAL_WINDOW_S = 3.0           # the spec's "final 3 s before CPA" window


def _nlerp_quat(q0, q1, w: float) -> Tuple[float, float, float, float]:
    a = np.asarray(q0, dtype=np.float64)
    b = np.asarray(q1, dtype=np.float64)
    if float(np.dot(a, b)) < 0.0:
        b = -b
    q = (1.0 - w) * a + w * b
    n = float(np.linalg.norm(q))
    return tuple((q / n) if n > 1e-12 else a)


class _Tracer:
    """Read-only Guidance wrapper: records per-tick internals of either arm.
    Implements the isim Guidance protocol by delegation."""

    def __init__(self, inner, arm: str):
        self.inner = inner
        self.arm = arm                     # "native" | "port"
        self.rows: List[dict] = []
        self._quat_hist: List[Tuple[float, Tuple[float, float, float, float]]] = []
        self.last_decode_t: Optional[float] = None

    # -- Guidance protocol -------------------------------------------------
    def reset(self) -> None:
        self.inner.reset()
        self.rows = []
        self._quat_hist = []
        self.last_decode_t = None

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def _quat_at(self, t_query: float):
        h = self._quat_hist
        if not h:
            return None
        if t_query <= h[0][0]:
            return h[0][1]
        for i in range(len(h) - 1, 0, -1):
            t0, q0 = h[i - 1]
            t1, q1 = h[i]
            if t0 <= t_query <= t1:
                w = 0.0 if t1 <= t0 else (t_query - t0) / (t1 - t0)
                return _nlerp_quat(q0, q1, w)
        return h[-1][1]

    def step(self, t: float, own, det):
        self._quat_hist.append((t, tuple(float(c) for c in own.quat_wxyz)))
        if len(self._quat_hist) > 40:      # ~0.8 s of history, plenty for 45 ms
            self._quat_hist.pop(0)

        row = {
            "t": t, "det": 0, "t_capture": math.nan, "age_s": math.nan,
            "e_att_m": math.nan, "e_jit_partial": math.nan, "e_slant_m": math.nan,
            "meas_range_m": math.nan,
        }

        # ---- port-arm counterfactual conversion errors (spec candidates a/b/e)
        if self.arm == "port" and det is not None:
            pt = self.inner._sm.guidance   # PursuitTerminalGuidance
            half = 0.5 * det.side_px
            box = (det.u_px - half, det.v_px - half, det.side_px, det.side_px)
            _b, range_m, meas_xyz = measurement_from_box(
                box, pt.gcfg, pt.cam, pt.tag_side_m)

            def conv(quat):
                v = _optical_vec_to_ned(meas_xyz, quat, pt.gcfg.mount_up_rad)
                return v + _cam_offset_ned(quat, pt.gcfg.cam_offset_body)

            q_now = tuple(float(c) for c in own.quat_wxyz)
            q_cap = self._quat_at(det.t_capture)
            if q_cap is not None:
                row["e_att_m"] = float(np.linalg.norm(conv(q_now) - conv(q_cap)))
            age = t - det.t_capture
            row["age_s"] = age
            row["e_jit_partial"] = abs(age - LAT_ASSUMED_S)   # x |v_rel_true| later
            # slant shortfall: |meas| is z-depth; true slant = z*sec(offaxis)
            ray = np.asarray(meas_xyz, dtype=np.float64) / max(range_m, 1e-9)
            sec = 1.0 / max(abs(float(ray[2])), 1e-6)
            row["e_slant_m"] = range_m * (sec - 1.0)
            row["meas_range_m"] = range_m

        if det is not None:
            row["det"] = 1
            row["t_capture"] = det.t_capture
            self.last_decode_t = t

        cmd = self.inner.step(t, own, det)

        own_pos = np.asarray(own.pos_ned, dtype=np.float64)
        own_vel = np.asarray(own.vel_ned, dtype=np.float64)
        if self.arm == "native":
            g = self.inner
            phase = g._phase
            if phase == "B":
                r_est = g._kf.pos - own_pos
                v_t_est = g._kf.vel
            else:
                pos_b, vel_b = g._track.at(t)
                r_est = pos_b - own_pos
                v_t_est = vel_b
        else:
            pt = self.inner._sm.guidance
            phase = f"{self.inner._sm.state}/{pt._phase}"
            if pt._phase == "B":
                r_est = pt._kf.r
                v_t_est = pt._kf.v_t
            else:
                r_est = pt._r_track
                v_t_est = pt._v_track

        row.update(
            phase=phase,
            r_est_n=float(r_est[0]), r_est_e=float(r_est[1]), r_est_d=float(r_est[2]),
            vt_est_n=float(v_t_est[0]), vt_est_e=float(v_t_est[1]), vt_est_d=float(v_t_est[2]),
            own_vel_n=float(own_vel[0]), own_vel_e=float(own_vel[1]), own_vel_d=float(own_vel[2]),
            cmd_n=cmd.v_north, cmd_e=cmd.v_east, cmd_d=cmd.v_down, cmd_yaw=cmd.yaw_deg,
        )
        self.rows.append(row)
        return cmd


def _run_arm(seed: int, arm: str):
    scn = Scenario(tag_facing="rear", seed=seed) if arm == "native" else None
    if arm == "native":
        scn = Scenario(concept="pursuit", tag_facing="rear", seed=seed)
    else:
        scn = Scenario(concept="flyby", terminal="pursuit", tag_facing="rear", seed=seed)
    ecfg, vehicle, target, seeker, guidance, init_state = build(scn, load_params())
    tracer = _Tracer(guidance, arm)
    import contextlib, io
    with contextlib.redirect_stdout(io.StringIO()):
        result = run_engagement(ecfg, vehicle, target, seeker, tracer, init_state,
                                record_trace=True)
    return result, tracer, ecfg


def _truth_at(result, t: float, dt: float):
    tr = result.trace
    k = min(int(round(t / dt)), len(tr["t"]) - 1)
    r_true = tr["tgt_pos"][k] - tr["own_pos"][k]
    v_t_true = tr["tgt_vel"][k]
    v_rel_true = tr["tgt_vel"][k] - tr["own_vel"][k]
    return r_true, v_t_true, v_rel_true


def _analyze(seed: int, arm: str, result, tracer, ecfg) -> dict:
    dt = ecfg.dt
    rows = tracer.rows
    for row in rows:
        r_true, v_t_true, v_rel_true = _truth_at(result, row["t"], dt)
        r_est = np.array([row["r_est_n"], row["r_est_e"], row["r_est_d"]])
        v_est = np.array([row["vt_est_n"], row["vt_est_e"], row["vt_est_d"]])
        row["est_err_m"] = float(np.linalg.norm(r_est - r_true))
        row["vel_err_ms"] = float(np.linalg.norm(v_est - v_t_true))
        row["range_true_m"] = float(np.linalg.norm(r_true))
        row["v_rel_true_ms"] = float(np.linalg.norm(v_rel_true))
        if row["det"] and not math.isnan(row.get("e_jit_partial", math.nan)):
            row["e_jit_m"] = row["e_jit_partial"] * row["v_rel_true_ms"]
        else:
            row["e_jit_m"] = math.nan

    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f"seed{seed}_{arm}.csv")
    cols = ["t", "phase", "det", "t_capture", "age_s", "range_true_m",
            "est_err_m", "vel_err_ms", "e_att_m", "e_jit_m", "e_slant_m",
            "meas_range_m", "r_est_n", "r_est_e", "r_est_d",
            "vt_est_n", "vt_est_e", "vt_est_d", "v_rel_true_ms",
            "cmd_n", "cmd_e", "cmd_d", "cmd_yaw"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for row in rows:
            w.writerow(row)

    t_cpa = result.t_cpa
    win = [r for r in rows if t_cpa - FINAL_WINDOW_S <= r["t"] <= t_cpa]
    win_b = [r for r in win if r["phase"].endswith("B")]
    dets = [r for r in win if r["det"]]
    last_dec = None
    for r in rows:
        if r["det"] and r["t"] <= t_cpa:
            last_dec = r

    def med(vals):
        vals = [v for v in vals if not math.isnan(v)]
        return float(np.median(vals)) if vals else math.nan

    return {
        "seed": seed, "arm": arm, "miss_m": result.miss_m, "t_cpa": t_cpa,
        "n_decoded": result.n_decoded,
        "n_det_final3s": len(dets),
        "est_err_last_decode_m": (last_dec["est_err_m"] if last_dec else math.nan),
        "med_est_err_final3s_m": med([r["est_err_m"] for r in win_b]),
        "med_vel_err_final3s_ms": med([r["vel_err_ms"] for r in win_b]),
        "med_e_att_final3s_m": med([r["e_att_m"] for r in dets]),
        "max_e_att_final3s_m": (max([r["e_att_m"] for r in dets
                                     if not math.isnan(r["e_att_m"])], default=math.nan)),
        "med_e_jit_final3s_m": med([r["e_jit_m"] for r in dets]),
        "med_e_slant_final3s_m": med([r["e_slant_m"] for r in dets]),
        "csv": os.path.join("logs", "parity_trace_2026-09-22", f"seed{seed}_{arm}.csv"),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="0,1,2,3,4")
    args = ap.parse_args()
    seeds = [int(s) for s in args.seeds.split(",")]

    summaries = []
    for seed in seeds:
        for arm in ("native", "port"):
            result, tracer, ecfg = _run_arm(seed, arm)
            summaries.append(_analyze(seed, arm, result, tracer, ecfg))

    cols = ["seed", "arm", "miss_m", "t_cpa", "n_decoded", "n_det_final3s",
            "est_err_last_decode_m", "med_est_err_final3s_m",
            "med_vel_err_final3s_ms", "med_e_att_final3s_m",
            "max_e_att_final3s_m", "med_e_jit_final3s_m",
            "med_e_slant_final3s_m"]
    print(("{:>5} {:>7} " + "{:>10}" * (len(cols) - 2)).format(*cols))
    for s in summaries:
        print("{:>5} {:>7} ".format(s["seed"], s["arm"]) + "".join(
            "{:>10}".format("" if isinstance(v := s[c], float) and math.isnan(v)
                            else (f"{v:.3f}" if isinstance(v, float) else v))
            for c in cols[2:]))
    print(f"\nper-tick CSVs in {OUT_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
