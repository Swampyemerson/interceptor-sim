#!/usr/bin/env python3
"""Camera focal-length sweep for the chase terminal (builder question
2026-09-23: would a second, much narrower-FoV / higher-resolution camera
help, and what limits faster intercepts?).

Sweeps cam_fx_px on the PORT chase arm (flyby + pursuit terminal, rear tag,
n=50 seeds) at target 9 and 15 m/s. fx=385 is the real OV9281 at 118 deg
HFOV (the flying camera); larger fx models a longer lens on the same
1280 px sensor: 540 ~ 100 deg, 770 ~ 79 deg, 1155 ~ 58 deg, 1540 ~ 45 deg.
The isim seeker honestly charges both sides of the trade: more px on the
tag (longer decode range, lower bearing/range noise) AND a narrower frame
(walkout) with the measured-exposure blur model.

SCOPE: single-camera sweep. The result is an UPPER BOUND on what a second
narrow camera adds as a terminal precision/range lever (a real dual-camera
system still needs wide-camera acquisition + a cross-camera handoff); if
even this upper bound is flat, the answer to the builder is no.
"""
import dataclasses

import numpy as np

from isim.scenario import Scenario
from isim import mc

FXS = [385.0, 540.0, 770.0, 1155.0, 1540.0]
SPEEDS = [9.0, 15.0]


def main():
    print(f"{'speed':>6} {'fx':>6} {'HFOV':>6} {'med':>7} {'p90':>7} "
          f"{'<=0.35':>7} {'<=1.0':>6} {'med_dec':>8}")
    for spd in SPEEDS:
        for fx in FXS:
            scens = [dataclasses.replace(
                Scenario(concept="flyby", terminal="pursuit",
                         tag_facing="rear", target_speed_ms=spd,
                         cam_fx_px=fx), seed=s) for s in range(50)]
            rows = mc.run_many(scens, workers=10)
            miss = np.array([float(r["miss_m"]) for r in rows])
            dec = np.median([float(r["n_decoded"]) for r in rows])
            hfov = 2 * np.degrees(np.arctan(640.0 / fx))
            print(f"{spd:6.0f} {fx:6.0f} {hfov:5.0f}° {np.median(miss):7.3f} "
                  f"{np.percentile(miss, 90):7.3f} "
                  f"{100 * np.mean(miss <= 0.35):6.0f}% "
                  f"{100 * np.mean(miss <= 1.0):5.0f}% {dec:8.0f}")


if __name__ == "__main__":
    main()
