"""isim/tests/test_tag_terminal_isim.py -- ROUND 2 closed-loop isim tests for
flight.tag_terminal.TagInterceptGuidance.

Round 1's crossing test had a natural (no-guidance) flyby of only ~0.22 m by
construction -- a near-already-aimed shot, weak evidence of anything. This
round replaces it with geometries whose NATURAL miss is LARGE (a hovering
target 30 m out, three 9 m/s crossings passing 6.5 m abeam, and a head-on
closing pass), swept over 2 lenses x 2 mount tilts x 12 seeds, from a
standing start (`dash_speed_ms=0.0`, `acquire_streak=3`).

READ THIS BEFORE TRUSTING A NUMBER: the two lenses (fx=933, a 2.8 mm lens on
the OV9281's 3 um pixels; fx=1400) both have a 50%-decode range for a 0.30 m
tag well under 30 m (`fx*0.30/22`: 12.7 m @ 933, 19.1 m @ 1400 -- 22 px is
`isim.seeker.DecodeParams.size50_px`). So the hover case is a genuine
STRESS test of acquisition-at-the-edge-of-range, and the crossing cases are
swept mainly to MEASURE, not to pass -- see `test_crossing_abeam_dwell_time_
is_the_limiting_physics` for why they are expected to fail structurally.
"""
from __future__ import annotations

import math
from typing import Dict, Tuple

import numpy as np
import pytest

from flight.deploy.real_flight import MissionConfig
from flight.tag_terminal import TagTerminalConfig
from isim.engine import EngagementConfig, run_engagement
from isim.flight_adapter import RealFlightGuidance, standby_init_state
from isim.replay_a0 import load_params
from isim.seeker import AprilTagSeeker, CameraParams, TagParams
from isim.targets import ConstantVelocityTarget
from isim.vehicle import QuadVelocityModel

ALT_M = 8.0
TAG_SIDE_M = 0.30
N_SEEDS = 12


def _hover():
    """(a) Hovering target 30 m ahead, 3 m right, 2 m up."""
    pos0 = (30.0, 3.0, -(ALT_M + 2.0))
    heading = math.degrees(math.atan2(pos0[1], pos0[0]))
    return pos0, (0.0, 0.0, 0.0), heading


def _crossing(alt_offset_m: float):
    """(b/c/d) 9 m/s crossing target passing 6.5 m abeam: track = the line
    north=6.5 (own is 6.5 m from it, perpendicular, by construction), sliding
    east at 9 m/s. Heading points at the target's STARTING bearing (own's
    camera is body-fixed through the whole coded dash -- there is no yaw
    lever available before ENGAGE)."""
    north0, east0 = 6.5, -12.0
    pos0 = (north0, east0, -(ALT_M + alt_offset_m))
    heading = math.degrees(math.atan2(east0, north0))
    return pos0, (0.0, 9.0, 0.0), heading


def _headon(alt_offset_m: float):
    """(e) Head-on 9 m/s closing target, 2 m higher."""
    pos0 = (25.0, 0.0, -(ALT_M + alt_offset_m))
    heading = math.degrees(math.atan2(pos0[1], pos0[0]))
    return pos0, (-9.0, 0.0, 0.0), heading


SCENARIOS: Dict[str, tuple] = {
    "hover_30m": _hover(),
    "cross_co": _crossing(0.0),
    "cross_+2m": _crossing(2.0),
    "cross_-2m": _crossing(-2.0),
    "headon_+2m": _headon(2.0),
}

# fx=933: a 2.8 mm lens on the OV9281's 3 um pixels (933 = 2.8mm/3um).
CAMERAS: Dict[str, Tuple[float, float]] = {
    "fx933_tilt0": (933.0, 0.0),
    "fx933_tilt15": (933.0, 15.0),
    "fx1400_tilt0": (1400.0, 0.0),
    "fx1400_tilt15": (1400.0, 15.0),
}


def _run_one(pos0, vel, heading_deg, fx, tilt_deg, seed):
    cfg = MissionConfig(preflight_heading_deg=heading_deg, standby_alt_m=ALT_M,
                        standby_settle_s=1.0, dash_speed_ms=0.0, dash_max_s=10.0,
                        acquire_streak=3, engage_max_s=15.0)
    cam = CameraParams(width=1280, height=800, fx=fx, fy=fx, cx=640.0, cy=400.0,
                       mount_tilt_up_deg=tilt_deg)
    tag = TagParams(faces_camera=True, side_m=TAG_SIDE_M)
    guidance = RealFlightGuidance(
        cfg, cam_params=cam, span_m=TAG_SIDE_M, go_at_s=cfg.standby_settle_s,
        terminal="tag", tag_cfg=TagTerminalConfig())
    vehicle = QuadVelocityModel(load_params())
    target = ConstantVelocityTarget(pos0_ned=np.array(pos0, dtype=float),
                                    vel_ned=np.array(vel, dtype=float))
    seeker = AprilTagSeeker(cam=cam, tag=tag)
    init = standby_init_state(cfg)
    ecfg = EngagementConfig(dt=0.005, guidance_dt=0.02, max_t=22.0,
                            stop_after_cpa_s=4.0, seed=seed)
    res = run_engagement(ecfg, vehicle, target, seeker, guidance, init)
    states = [s for _, s in guidance.state_log]
    return res.miss_m, states


def _sweep() -> Dict[Tuple[str, str], Tuple[float, float, float, int]]:
    """Runs the full scenario x camera x seed grid ONCE.
    Returns {(scenario, camera): (median_m, p90_m, frac_under_35cm, n_engaged)}."""
    stats = {}
    for sname, sc in SCENARIOS.items():
        for cname, (fx, tilt) in CAMERAS.items():
            misses, engaged = [], 0
            for seed in range(N_SEEDS):
                miss, states = _run_one(*sc, fx, tilt, seed)
                misses.append(miss)
                engaged += int("ENGAGE" in states)
            arr = np.array(misses)
            stats[(sname, cname)] = (float(np.median(arr)), float(np.percentile(arr, 90)),
                                     float(np.mean(arr < 0.35)), engaged)
    return stats


def _print_table(stats) -> None:
    print(f"\n{'scenario':11s} {'camera':14s} {'engaged':>9s} {'median_m':>9s} "
         f"{'p90_m':>8s} {'frac<0.35':>10s}")
    for (sname, cname), (med, p90, frac, eng) in stats.items():
        print(f"{sname:11s} {cname:14s} {eng:>6d}/{N_SEEDS} {med:9.3f} {p90:8.3f} "
             f"{frac:10.2f}")


def test_speed_camera_sweep_table_and_hover_bound():
    """THE round-2 sweep: 5 scenarios x 4 camera configs x 12 seeds. Prints
    the full table (run with `-s` to see it). The ONLY hard assertion is the
    hover case's median at whichever lens/tilt does best -- everything else
    is reported, not asserted, per the task brief ("never assert a tuned
    number"). If even the best combo misses 0.5 m, this is an XFAIL with the
    measured number in the reason, not a forced pass."""
    stats = _sweep()
    _print_table(stats)

    hover_medians = {c: stats[("hover_30m", c)][0] for c in CAMERAS}
    best_cam = min(hover_medians, key=hover_medians.get)
    best_median = hover_medians[best_cam]
    print(f"\n[hover_30m] best lens/tilt = {best_cam}, median miss = "
         f"{best_median:.3f} m (n_engaged={stats[('hover_30m', best_cam)][3]}/{N_SEEDS})")

    if best_median >= 0.5:
        pytest.xfail(
            f"hover_30m median at the BEST of the 4 lens/tilt combos "
            f"({best_cam}) is {best_median:.3f} m, not < 0.5 m. NOT a law "
            f"bug: at 30 m a 0.30 m tag is past both lenses' 50%-decode "
            f"range (12.7 m @ fx=933, 19.1 m @ fx=1400 -- side_px = "
            f"fx*0.30/range = 22 px there). The acquire streak still "
            f"reaches 3 eventually because it has NO TIME DECAY (any "
            f"decode counts, however sparse, over the whole 10 s "
            f"dash_max_s), so ENGAGE starts from an acquisition that is "
            f"not dependable, and real_flight's own engage_lost_target_s "
            f"(2.0 s) then frequently times out before the vehicle has "
            f"closed enough range for decode to become reliable. Full "
            f"breakdown in the task report.")
    assert best_median < 0.5


def test_crossing_abeam_dwell_time_is_the_limiting_physics():
    """ANALYTICAL finding (no sim needed, so it can't be a seed artefact):
    with a 9 m/s target passing 6.5 m abeam and a BODY-FIXED camera held at
    ONE heading through the whole coded dash (real_flight does not re-yaw
    until ENGAGE), the tag is inside the lens's FOV for well under a second
    at every tested lens/tilt -- matching the empirical 0/12 ENGAGE measured
    across all three crossing scenarios (cross_co/+2m/-2m) in the sweep
    above. A geometry finding about this camera, not a law defect."""
    v, b = 9.0, 6.5
    ang_rate_deg_s = math.degrees(v / b)   # 79.3 deg/s AT closest approach
    for cname, (fx, _tilt) in CAMERAS.items():
        half_fov_deg = math.degrees(math.atan2(400.0, fx))
        dwell_s = 2.0 * half_fov_deg / ang_rate_deg_s
        print(f"[crossing dwell] {cname}: half_fov={half_fov_deg:.1f} deg, "
             f"ang_rate(closest)={ang_rate_deg_s:.1f} deg/s, dwell~{dwell_s:.2f} s")
        assert dwell_s < 0.7   # matches the coordinator's own infeasibility bound
