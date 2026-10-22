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
        if rep is not None:
            self.frames.append((t, bool(rep.in_fov),
                                float(getattr(rep, "v_px", float("nan")))))
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
    infov = sum(1 for f in pre if f[1])
    vpx = [f[2] for f in pre if f[1] and f[2] == f[2]]
    return {"seed": seed, "n_pre_frames": n,
            "pre_infov_frac": (infov / n) if n else None,
            "med_v_px": float(np.median(vpx)) if vpx else None,
            "acquired": t_acq is not None}


def main():
    out = {}
    for off in OFFSETS:
        rows = [one(off, s) for s in range(N)]
        fr = [r["pre_infov_frac"] for r in rows if r["pre_infov_frac"] is not None]
        acq = sum(r["acquired"] for r in rows)
        med = float(np.median(fr)) if fr else None
        out[str(off)] = {"rows": rows, "med_pre_infov_frac": med,
                         "n_acquired": acq}
        print(f"offset={off:+4.1f}: med pre-acq in-FOV frac="
              f"{med if med is not None else float('nan'):.3f} "
              f"acquired {acq}/{N}", flush=True)
    with open(os.path.join(os.path.dirname(__file__), "probe_fov.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    print("wrote probe_fov.json")


if __name__ == "__main__":
    main()
