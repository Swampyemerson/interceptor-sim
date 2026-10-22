#!/usr/bin/env python3
"""Measure-only FOV probe for the +/-3 m asymmetry -- see the spec amendment.
Wraps the seeker to record FrameReports pre-acquisition; guidance untouched."""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np
from isim.engine import run_engagement
from isim.replay_a0 import load_params
from isim.scenario import Scenario, Scatter, build

N = 8
OFFSETS = [0.0, 3.0, -3.0]


class SeekerSpy:
    """Records every observation's FrameReport (in_fov/v_px); passes both
    return values through untouched. The engine calls seeker.observe(t, own,
    tgt) -> (det, rep) and seeker.reset(rng) (isim/engine.py)."""

    def __init__(self, inner):
        self._inner = inner
        self.frames = []          # (t, in_fov, v_px)

    def __getattr__(self, name):
        return getattr(self._inner, name)

    def reset(self, rng):
        return self._inner.reset(rng)

    def observe(self, t, own, tgt):
        det, rep = self._inner.observe(t, own, tgt)
        # TRUE slant range: scoring-only read (this spy is outside guidance).
        import numpy as _np
        rng = float(_np.linalg.norm(_np.asarray(tgt.pos_ned) - _np.asarray(own.pos_ned)))
        self.min_range = min(getattr(self, "min_range", float("inf")), rng)
        if det is not None and getattr(self, "first_decode", None) is None:
            self.first_decode = (t, rng)
        if rep is not None:
            self.frames.append((t, bool(rep.in_fov),
                                float(getattr(rep, "side_px", float("nan"))),
                                det is not None))
        return det, rep


def one(off, seed):
    scn = Scenario(concept="hybrid", target_speed_ms=9.0, cross_range_m=6.5,
                   lead_dist_m=16.2, cam_fx_px=933.0, tag_facing="rear",
                   scatter=Scatter(), seed=seed, aim_error_deg=0.0,
                   target_alt_offset_m=off)
    ecfg, vehicle, target, seeker, guidance, init = build(scn, load_params())
    spy = SeekerSpy(seeker)
    res = run_engagement(ecfg, vehicle, target, spy, guidance, init)
    # first acquisition time: guidance state log (Phase B/T entry) if present
    t_acq = None
    for t, state in getattr(guidance, "state_log", []):
        if state in ("B", "T", "ENGAGE", "PHASE_B"):
            t_acq = t
            break
    pre = [f for f in spy.frames if t_acq is None or f[0] < t_acq]
    n = len(pre)
    infov = [f for f in pre if f[1]]
    side = [f[2] for f in infov if f[2] == f[2]]
    dec = sum(1 for f in infov if f[3])
    fd = getattr(spy, "first_decode", None)
    return {"seed": seed, "n_pre_frames": n,
            "pre_infov_frac": (len(infov) / n) if n else None,
            "med_side_px": float(np.median(side)) if side else None,
            "decode_per_infov": (dec / len(infov)) if infov else None,
            "first_decode_t": fd[0] if fd else None,
            "first_decode_range_m": fd[1] if fd else None,
            "min_range_m": getattr(spy, "min_range", None),
            "acquired": t_acq is not None}


def main():
    out = {}
    for off in OFFSETS:
        rows = [one(off, s) for s in range(N)]
        fr = [r["pre_infov_frac"] for r in rows if r["pre_infov_frac"] is not None]
        dr = [r["decode_per_infov"] for r in rows if r["decode_per_infov"] is not None]
        sp = [r["med_side_px"] for r in rows if r["med_side_px"] is not None]
        acq = sum(r["acquired"] for r in rows)
        out[str(off)] = {"rows": rows,
                         "med_pre_infov_frac": float(np.median(fr)) if fr else None,
                         "med_decode_per_infov": float(np.median(dr)) if dr else None,
                         "med_side_px": float(np.median(sp)) if sp else None,
                         "n_acquired": acq}
        print(f"offset={off:+4.1f}: acquired {acq}/{N}", flush=True)
        for r in rows:
            print(f"   seed {r['seed']}: acq={int(r['acquired'])} "
                  f"first_decode_range={r['first_decode_range_m'] if r['first_decode_range_m'] is None else round(r['first_decode_range_m'],1)} "
                  f"min_range={round(r['min_range_m'],1)}", flush=True)
    with open(os.path.join(os.path.dirname(__file__), "probe_fov.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    print("wrote probe_fov.json")


if __name__ == "__main__":
    main()
