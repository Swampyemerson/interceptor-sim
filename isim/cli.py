"""`python -m isim run|bench` -- see isim/__main__.py for the entry point."""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

from isim.engine import EngagementConfig, run_engagement
from isim.stubs import FirstOrderVehicle, PerfectSeeker, PursuitGuidance, yaw_to_quat_wxyz
from isim.targets import ConstantVelocityTarget
from isim.trace import write_trace_csv
from isim.types import VehicleState

_RANGE0_M = 120.0          # initial north separation
_INTERCEPTOR_SPEED_MS = 12.0  # guidance commanded speed, < FirstOrderVehicle default vmax=15


def _build_default_engagement(
    seed: int,
    target_speed: float = 9.0,
    cross_range: float = 0.0,
    alt_offset: float = 0.0,
):
    """Shared geometry for both `run` and `bench`: interceptor starts at the
    NED origin, target starts `_RANGE0_M` north with a `cross_range` east
    offset and `alt_offset` metres ABOVE the interceptor, and flies east
    (crossing) at `target_speed`. Interceptor's initial yaw points at the
    target's initial position so it starts in FOV."""
    tgt_pos0 = np.array([_RANGE0_M, cross_range, -alt_offset])
    tgt_vel0 = np.array([0.0, target_speed, 0.0])
    target = ConstantVelocityTarget(tgt_pos0, tgt_vel0)

    yaw0 = math.atan2(cross_range, _RANGE0_M)
    init_state = VehicleState(
        t=0.0,
        pos_ned=np.zeros(3),
        vel_ned=np.zeros(3),
        quat_wxyz=yaw_to_quat_wxyz(yaw0),
        yaw_rad=yaw0,
    )

    vehicle = FirstOrderVehicle()
    seeker = PerfectSeeker()
    guidance = PursuitGuidance(speed=_INTERCEPTOR_SPEED_MS)
    cfg = EngagementConfig(seed=seed)
    return cfg, vehicle, target, seeker, guidance, init_state


def _cmd_run(args: argparse.Namespace) -> int:
    cfg, vehicle, target, seeker, guidance, init_state = _build_default_engagement(
        seed=args.seed,
        target_speed=args.target_speed,
        cross_range=args.cross_range,
        alt_offset=args.alt_offset,
    )
    result = run_engagement(
        cfg, vehicle, target, seeker, guidance, init_state, record_trace=bool(args.trace)
    )
    if args.trace:
        write_trace_csv(args.trace, result.trace)
    print(
        f"ISIM_RESULT miss={result.miss_m:.4f} t_cpa={result.t_cpa:.3f} "
        f"horiz={result.miss_horiz_m:.4f} vert={result.miss_vert_m:.4f} "
        f"frames={result.n_frames} decoded={result.n_decoded}"
    )
    return 0


def _cmd_bench(args: argparse.Namespace) -> int:
    n = args.n
    t0 = time.perf_counter()
    for i in range(n):
        cfg, vehicle, target, seeker, guidance, init_state = _build_default_engagement(seed=i)
        run_engagement(cfg, vehicle, target, seeker, guidance, init_state, record_trace=False)
    elapsed = time.perf_counter() - t0
    rate = (n / elapsed * 60.0) if elapsed > 0 else float("inf")
    print(f"ISIM_BENCH n={n} elapsed_s={elapsed:.3f} engagements_per_min={rate:.1f}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="isim", description="isim -- fast headless engagement simulator")
    sub = p.add_subparsers(dest="command", required=True)

    p_run = sub.add_parser("run", help="run one engagement with the reference stubs")
    p_run.add_argument("--seed", type=int, default=0)
    p_run.add_argument("--target-speed", type=float, default=9.0)
    p_run.add_argument("--cross-range", type=float, default=0.0)
    p_run.add_argument("--alt-offset", type=float, default=0.0)
    p_run.add_argument("--trace", type=str, default=None, help="write per-step CSV trace to this path")
    p_run.set_defaults(func=_cmd_run)

    p_bench = sub.add_parser("bench", help="benchmark engagements/minute with the reference stubs")
    p_bench.add_argument("--n", type=int, default=2000)
    p_bench.set_defaults(func=_cmd_bench)

    return p


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
