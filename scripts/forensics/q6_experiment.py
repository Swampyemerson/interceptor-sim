#!/usr/bin/env python3
"""Q6 controlled point-mass experiment: does a PERFECT terminal camera
collapse the miss, holding the realistic mid-course cue fixed?

Arms (all: two_stage_dash S2 config, dash 10 m/s, handoff 10 m, pure_pn
terminal, crossing target at 6 m/s, seeds 0..N-1):
  A EXPECTED           realistic cue + realistic terminal camera (P6 EXPECTED tier)
  B EXPECTED+PERFCAM   SAME realistic cue, but terminal camera PERFECT
                       (exact bearing/range, no FOV limit, no dropout)
  C ALLPERFECT         clean cue (BEST tier) + PERFECT camera
  D BEST               clean cue, normal lab camera (the ADR-0015 idealized baseline)
Lab caveat: the lab has documented optimism vs Gazebo (5 cases); we use it
only for the CONTRAST between arms, not absolute miss."""
import importlib, statistics, sys
sys.path.insert(0, "/home/emerson/.claude/jobs/28aff4e9/tmp")
import guidance_lab_copy as gl
import numpy as np

N_SEEDS = 150
SPEED = 6.0

def run(perception, perfect_terminal, n=N_SEEDS):
    gl.PERFECT_TERMINAL = perfect_terminal
    method_params = dict(
        DASH_SPEED=gl.P6_DASH_SPEED, HANDOFF_RANGE=gl.P6_HANDOFF_RANGE,
        TERMINAL_METHOD="pure_pn",
    )
    path_params = dict(gl.S2_PATH_PARAMS, speed=SPEED, handoff_range=gl.P6_HANDOFF_RANGE)
    rows = []
    for seed in range(n):
        rows.append(gl.simulate("two_stage_dash", method_params, gl.P6_PATH,
                                dict(path_params), seed, two_stage=True,
                                perception=dict(perception) if perception else None))
    return rows

def report(name, rows):
    miss = np.array([r["miss_distance"] for r in rows])
    tc = [r["terminal_coverage"] for r in rows if r["terminal_coverage"] is not None]
    lost = [r["tag_lost_in_terminal"] for r in rows]
    pk1 = float(np.mean(miss < 1.0)); pk2 = float(np.mean(miss < 2.0))
    print(f"{name:22s} n={len(miss)}  miss mean {miss.mean():.3f} med {np.median(miss):.3f} "
          f"p90 {np.percentile(miss,90):.3f}  Pk@1 {pk1*100:4.0f}%  Pk@2 {pk2*100:4.0f}%  "
          f"term-coverage {np.mean(tc) if tc else float('nan'):.2f}  "
          f"lost-in-term {100*np.mean(lost):.0f}%")
    return miss

if __name__ == "__main__":
    exp = gl.P6_TIERS["EXPECTED"]
    print(f"S2 two-stage, pure_pn terminal, target {SPEED} m/s, seeds {N_SEEDS} (paired)")
    a = report("A EXPECTED", run(exp, False))
    b = report("B EXPECTED+PERFCAM", run(exp, True))
    c = report("C ALLPERFECT", run(None, True))
    d = report("D BEST(idealized)", run(None, False))
    print(f"\npaired deltas vs A: B {np.mean(b-a):+.3f} m (perfect terminal camera)  "
          f"C {np.mean(c-a):+.3f} m (perfect everything)")
    print(f"terminal-camera share of A's miss: {(np.mean(a)-np.mean(b))/np.mean(a)*100:.0f}%  |  "
          f"mid-course-cue share: {(np.mean(b)-np.mean(c))/np.mean(a)*100:.0f}%  |  "
          f"kinematic residual (C): {np.mean(c)/np.mean(a)*100:.0f}%")
