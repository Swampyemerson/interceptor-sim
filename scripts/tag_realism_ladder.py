#!/usr/bin/env python3
"""tag_realism_v1 measurement ladder (2026-09-23 night). PRE-REGISTERED:
isim/specs/tag_realism_v1.md §F (+ the config amendment) -- read it before
interpreting output.

Port arm (real flight code), paired seeds 0..49, all v5 errors on
(scatter=Scatter()), exactly the chase_tilt_ab.py base config. Rungs stack
the tag-realism truth-model one factor at a time; every rung reuses the SAME
seeds and the draw-isolation design keeps the older draws identical, so
rung-to-rung deltas are paired.
"""
import dataclasses
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from isim.scenario import Scatter, Scenario
from isim import mc

N_SEEDS = 50


def _scat(**kw) -> Scatter:
    return dataclasses.replace(Scatter(), **kw)


# Rungs: (name, scenario overrides, scatter overrides)
RUNGS = [
    # R0: today's model -- must REPRODUCE the chase_tilt_ab tilt-0 numbers
    # (same seeds/config; the pin test says byte-identical).
    ("R0_base", dict(), dict()),
    # R1: target attitude only (tag bolted to a pitching/banking body; wobble off)
    ("R1_att", dict(target_attitude=True),
     dict(tgt_shake_rms_min_deg=0.0, tgt_shake_rms_max_deg=0.0)),
    # R2: + attitude wobble (default 0.5-2.5 deg RMS)
    ("R2_shake", dict(target_attitude=True), dict()),
    # R3: + own-camera vibration (spec EXPECTED tier 40 dps max)
    ("R3_vib", dict(target_attitude=True), dict(own_vib_rate_rms_max_dps=40.0)),
    # R4: + sun glare, EXPECTED (matte) tier
    ("R4_glare", dict(target_attitude=True, glare=True),
     dict(own_vib_rate_rms_max_dps=40.0)),
    # R5: WORST-credible tier (glossy tag, harder shake/vib, wider flare cone)
    ("R5_worst", dict(target_attitude=True, glare=True),
     dict(own_vib_rate_rms_max_dps=80.0,
          tgt_shake_rms_min_deg=2.5, tgt_shake_rms_max_deg=4.0,
          specular_strength_min=0.6, specular_strength_max=0.95,
          backlight_kill_max_deg=35.0)),
]

# Cells: (name, facing, scenario overrides)
CELLS = [
    ("rear_nom",      "rear",   dict()),
    ("rear_aim20",    "rear",   dict(aim_error_deg=20.0)),
    ("rear_alt+3",    "rear",   dict(target_alt_offset_m=3.0)),
    ("rear_weave",    "rear",   dict(target_motion="weave")),
    ("rear_nom_t10",  "rear",   dict(cam_tilt_up_deg=10.0)),
    ("rear_alt3_t10", "rear",   dict(target_alt_offset_m=3.0, cam_tilt_up_deg=10.0)),
    ("cam_nom",       "camera", dict()),
]


def main():
    only = sys.argv[1] if len(sys.argv) > 1 else None
    print(f"{'cell':>13} {'rung':>9} {'med':>7} {'p90':>7} {'<=0.35':>7} "
          f"{'<=1.0':>6} {'med_dec':>8}")
    for cell_name, facing, cell_kw in CELLS:
        if only and only not in cell_name:
            continue
        for rung_name, scn_kw, scat_kw in RUNGS:
            base = Scenario(concept="flyby", terminal="pursuit",
                            tag_facing=facing, scatter=_scat(**scat_kw),
                            **cell_kw, **scn_kw)
            scens = [dataclasses.replace(base, seed=s) for s in range(N_SEEDS)]
            rows = mc.run_many(scens, workers=10)
            miss = np.array([float(r["miss_m"]) for r in rows])
            dec = np.median([float(r["n_decoded"]) for r in rows])
            print(f"{cell_name:>13} {rung_name:>9} {np.median(miss):7.3f} "
                  f"{np.percentile(miss, 90):7.3f} "
                  f"{100 * np.mean(miss <= 0.35):6.0f}% "
                  f"{100 * np.mean(miss <= 1.0):5.0f}% {dec:8.0f}", flush=True)
        print(flush=True)


if __name__ == "__main__":
    main()
