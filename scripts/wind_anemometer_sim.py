#!/usr/bin/env python3
"""SIM-HARNESS ONLY: the handheld-anemometer INSTRUMENT MODEL.

==========================================================================
THIS FILE NEVER RUNS ON THE VEHICLE, AND scripts/wind_trim.py NEVER IMPORTS IT
==========================================================================
It stands in for a human operator walking to the launch point with a $20
handheld anemometer, reading the wind, and typing two numbers before launch.
In the field that step involves no software at all. In the sim we have to
manufacture the operator's reading from the wind the sim is actually blowing,
so THIS FILE -- and only this file -- is allowed to look at the simulator's
configured true wind.

The honesty partition is a FILE BOUNDARY, so it is machine-checkable:

    scripts/wind_anemometer_sim.py   (this file, PRODUCER)
        may read the sim's true wind; adds instrument error; writes a JSON
        reading containing ONLY what a real instrument would show.
                       |
                       |  reading JSON (no truth fields -- enforced both ends)
                       v
    scripts/wind_trim.py             (CONSUMER, the flight-side correction)
        stdlib-only, cannot import this file, and its
        AnemometerReading.from_mapping() REFUSES any gt_/true_/truth key.

`tests/test_wind_trim_honesty.py` asserts the direction of that arrow (and
proves the assertion fires, by mutating a copy of wind_trim.py to import this
module and watching the test go red). `tests/test_wind_trim_contract.py` is
the producer->consumer contract test whose fixture is built by THIS FILE'S
OWN WRITER -- a hand-typed fixture is exactly what has hidden schema breaks in
this project before (CLAUDE.md, "Instruments are evidence").

WHAT THE INSTRUMENT ERROR MODEL IS
----------------------------------
  reading_speed = clamp0( true_speed + bias_speed + N(0, sigma_eff(true_speed)) )
  reading_dir   = wrap360( true_dir  + bias_dir   + N(0, sigma_dir) )

with sigma_eff the data sheet's "+/-(0.3 m/s or 5% of reading, whichever is
greater)" form (AnemometerSpec.sigma_speed_at). The bias terms are a PER-UNIT
calibration offset -- a real instrument's error is not zero-mean, and the
project has been bitten before by treating a bias-dominated error as noise
(scripts/ekf_tracker.py, the cue datum bias). They are 0.0 in the placeholder
spec because no unit has been bought and no bias has been MEASURED; 0.0 there
means unknown, not unbiased.

Randomness is a local `random.Random(seed)` -- never the global RNG -- so a
gust/instrument realization PAIRS across A/B arms driven by the same master
seed, and re-running a flight reproduces its reading exactly.

The true wind values it consumes are SIM CONFIGURATION (what the wind driver
is told to blow), quoted in m/s and met-convention degrees. They are on the
gt_* side of the honesty boundary: scoring and logging only. Nothing
downstream of the reading JSON ever sees them.

Usage (writes the reading a flight harness would hand the trim, and prints the
exact CLI values an operator would have typed):

    python3 scripts/wind_anemometer_sim.py \
        --true-wind-mps 4.5 --true-dir-from-deg 270 --seed 7 \
        --taken-at-sim-t 0.0 --out logs/wind_reading_seed7.json
"""
from __future__ import annotations

import json
import os
import random
import sys
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from wind_trim import (  # noqa: E402  (path shim above must run first)
    PLACEHOLDER_ANEMOMETER_SPEC,
    READING_SCHEMA,
    AnemometerReading,
    AnemometerSpec,
)

__all__ = ["simulate_reading", "write_reading_json", "main"]


def simulate_reading(true_wind_mps: float,
                     true_dir_from_deg: float,
                     seed: int,
                     spec: AnemometerSpec = PLACEHOLDER_ANEMOMETER_SPEC,
                     taken_at_sim_t: Optional[float] = None) -> AnemometerReading:
    """Turn the sim's CONFIGURED TRUE WIND into what the operator's instrument
    would have shown. Deterministic in `seed`.

    Args:
      true_wind_mps: the steady wind speed the sim's wind driver is blowing
        (m/s). SIM CONFIGURATION -- the gt side of the boundary.
      true_dir_from_deg: met-convention direction the wind blows FROM (deg).
      seed: per-flight seed; pair it with the arm's master seed so the
        instrument realization is identical across A/B arms.
      spec: the instrument's declared error (no defaults inside AnemometerSpec;
        the module-level PLACEHOLDER is loudly labelled as unmeasured).
      taken_at_sim_t: SIM seconds at which the operator took the reading
        (never wall clock).

    Returns an AnemometerReading carrying ONLY instrument-visible quantities.
    """
    if float(true_wind_mps) < 0.0:
        raise ValueError("true wind speed must be >= 0 m/s")
    rng = random.Random(int(seed))
    sigma_s = spec.sigma_speed_at(float(true_wind_mps))
    speed = float(true_wind_mps) + float(spec.bias_speed_mps) + rng.gauss(0.0, sigma_s)
    speed = max(0.0, speed)  # a cup anemometer cannot read negative
    direction = (float(true_dir_from_deg) + float(spec.bias_dir_deg)
                 + rng.gauss(0.0, float(spec.sigma_dir_deg))) % 360.0
    return AnemometerReading(
        speed_mps=speed,
        dir_from_deg=direction,
        sigma_speed_mps=sigma_s,
        sigma_dir_deg=float(spec.sigma_dir_deg),
        taken_at_sim_t=(None if taken_at_sim_t is None else float(taken_at_sim_t)),
        # The provenance names the SEED, not the true wind: a log reader must be
        # able to reproduce the reading without the reading itself carrying the
        # truth it was made from.
        provenance=(f"SIMULATED handheld anemometer "
                    f"(scripts/wind_anemometer_sim.py, seed={int(seed)}); "
                    f"spec: {spec.provenance}"),
    )


def write_reading_json(path: str, reading: AnemometerReading) -> str:
    """THE PRODUCER'S OWN WRITER -- the contract test's fixture source.

    It serializes exactly `AnemometerReading.to_mapping()`, so a field can
    never appear on the wire that the consumer's allowlist does not know
    about, and the truth values used to make the reading are structurally
    absent (they are not fields of the reading at all).
    """
    payload = reading.to_mapping()
    assert payload.get("schema") == READING_SCHEMA, "writer/consumer schema drift"
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True)
        fh.write("\n")
    return path


def main(argv=None) -> int:
    import argparse

    ap = argparse.ArgumentParser(
        description="Simulate the operator's pre-flight anemometer reading "
                    "(SIM HARNESS ONLY -- never runs on the vehicle).")
    ap.add_argument("--true-wind-mps", type=float, required=True,
                    help="steady wind speed the sim's wind driver is blowing (m/s)")
    ap.add_argument("--true-dir-from-deg", type=float, required=True,
                    help="met-convention direction the wind blows FROM (deg, 0=N)")
    ap.add_argument("--seed", type=int, required=True,
                    help="per-flight seed; pair with the arm's master seed")
    ap.add_argument("--taken-at-sim-t", type=float, default=None,
                    help="SIM seconds at which the reading was taken (not wall clock)")
    ap.add_argument("--out", default=None, help="write the reading JSON here")
    args = ap.parse_args(argv)

    reading = simulate_reading(args.true_wind_mps, args.true_dir_from_deg,
                               args.seed, taken_at_sim_t=args.taken_at_sim_t)
    print(f"[anemometer-sim] true wind {args.true_wind_mps:.2f} m/s from "
          f"{args.true_dir_from_deg:.1f} deg (SIM CONFIG, gt side) -> operator would "
          f"read {reading.speed_mps:.2f} m/s from {reading.dir_from_deg:.1f} deg "
          f"(sigma {reading.sigma_speed_mps:.2f} m/s / {reading.sigma_dir_deg:.1f} deg)")
    print(f"[anemometer-sim] CLI the operator would type:  "
          f"--wind-trim-mps {reading.speed_mps:.2f} "
          f"--wind-trim-dir-deg {reading.dir_from_deg:.1f}")
    if args.out:
        write_reading_json(args.out, reading)
        print(f"[anemometer-sim] wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
