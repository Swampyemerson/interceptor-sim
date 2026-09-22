"""isim/tests/test_pursuit_terminal_isim.py -- closed-loop isim SMOKE test for
`flight.pursuit_terminal.PursuitTerminalGuidance`, driven through the REAL
flight code via `isim.flight_adapter.RealFlightGuidance(terminal="pursuit")`.

This is NOT the A0-style parity check against `isim.concepts.
PursuitRendezvousGuidance`'s own validated numbers (ADR-0103) -- that needs
the dash-endpoint kinematics wired through `isim.mc`'s CLI/Scenario, deferred
per `docs/pursuit_port_2026-09-17.md` work item 4. This is the narrower,
cheaper claim: the PORT runs end-to-end through the real state machine and
actually closes range and makes contact on a simple scenario, using
`dash_speed_ms=0.0` (no dash) so the vehicle's position at `go_at_s` is
EXACTLY the known standby position -- letting `belief_r0_ned`/
`belief_vel0_ned` be computed exactly, with no dash-endpoint estimation
needed, rather than testing that estimation too in the same pass.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from flight.deploy.real_flight import MissionConfig
from flight.pursuit_terminal import PursuitTerminalConfig
from isim.engine import EngagementConfig, run_engagement
from isim.flight_adapter import RealFlightGuidance, standby_init_state
from isim.replay_a0 import load_params
from isim.seeker import AprilTagSeeker, CameraParams, TagParams
from isim.targets import ConstantVelocityTarget
from isim.vehicle import QuadVelocityModel

ALT_M = 8.0
TAG_SIDE_M = 0.30


def _run_one(pos0, vel, heading_deg, fx, tilt_deg, seed, tag_facing="camera"):
    cfg = MissionConfig(preflight_heading_deg=heading_deg, standby_alt_m=ALT_M,
                        standby_settle_s=1.0, dash_speed_ms=0.0, dash_max_s=10.0,
                        acquire_streak=3, engage_max_s=25.0, pursuit_mode=True)
    cam = CameraParams(width=1280, height=800, fx=fx, fy=fx, cx=640.0, cy=400.0,
                       mount_tilt_up_deg=tilt_deg)
    tag = TagParams(faces_camera=(tag_facing == "camera"), side_m=TAG_SIDE_M)

    # dash_speed_ms=0.0 -> the vehicle never leaves standby before go_at_s, so
    # its position there is EXACTLY standby_init_state(cfg)'s -- no dash-
    # endpoint estimate needed for this smoke test (see module docstring).
    own_pos0 = np.array([0.0, 0.0, -ALT_M])
    belief_r0_ned = tuple(np.array(pos0, dtype=float) - own_pos0)

    guidance = RealFlightGuidance(
        cfg, cam_params=cam, span_m=TAG_SIDE_M, go_at_s=cfg.standby_settle_s,
        terminal="pursuit", pursuit_cfg=PursuitTerminalConfig(),
        belief_r0_ned=belief_r0_ned, belief_vel0_ned=tuple(vel))
    vehicle = QuadVelocityModel(load_params())
    target = ConstantVelocityTarget(pos0_ned=np.array(pos0, dtype=float),
                                    vel_ned=np.array(vel, dtype=float))
    seeker = AprilTagSeeker(cam=cam, tag=tag)
    init = standby_init_state(cfg)
    ecfg = EngagementConfig(dt=0.005, guidance_dt=0.02, max_t=30.0,
                            stop_after_cpa_s=4.0, seed=seed)
    res = run_engagement(ecfg, vehicle, target, seeker, guidance, init)
    states = [s for _, s in guidance.state_log]
    return res.miss_m, states


def test_pursuit_terminal_runs_end_to_end_and_reaches_phase_b_on_a_hover_target():
    """Hovering target 15 m ahead, 2 m up, camera-facing tag (the simpler
    facing to set up for a head-on approach smoke test -- rear-facing needs a
    receding-target geometry, deferred). Not a statistical claim (n=1 seed) -- the
    thing under test is "does the PORT run through the real flight code and
    do something sane", not the concept's own already-measured numbers."""
    pos0 = (10.0, 0.0, -(ALT_M + 2.0))
    heading = math.degrees(math.atan2(pos0[1], pos0[0]))
    miss, states = _run_one(pos0, (0.0, 0.0, 0.0), heading, fx=933.0, tilt_deg=12.0,
                            seed=0, tag_facing="camera")
    assert "ENGAGE" in states, f"never reached Phase B/ENGAGE -- states={states}"
    assert miss < 15.0, f"miss={miss:.2f} m -- the vehicle did not close range at all"


def test_pursuit_terminal_closes_meaningfully_on_a_moving_target():
    """Target moving away at 4 m/s -- pursuit's slow-arrival concept should
    still close most of the way in (n=1 seed, smoke-level claim only)."""
    pos0 = (10.0, 2.0, -(ALT_M + 1.0))
    heading = math.degrees(math.atan2(pos0[1], pos0[0]))
    miss, states = _run_one(pos0, (4.0, 0.0, 0.0), heading, fx=933.0, tilt_deg=12.0,
                            seed=1, tag_facing="camera")
    assert "ENGAGE" in states, f"never reached Phase B/ENGAGE -- states={states}"
    print(f"\n[pursuit smoke, moving target] miss={miss:.2f} m, states={sorted(set(states))}")
    assert miss < 5.0, f"miss={miss:.2f} m -- too far to call this 'closing meaningfully'"


def test_flyby_terminal_pursuit_runs_through_the_scenario_harness():
    """The FULL scenario harness path (isim.scenario.build, not this file's
    hand-rolled _run_one): Scenario(concept="flyby", terminal="pursuit") must
    wire belief_r0_ned/belief_vel0_ned + pursuit_mode itself, reach ENGAGE
    directly off the GO edge (pursuit_mode's chase-only entry), and CLOSE
    RANGE on the default crossing geometry (start range ~17.5 m; n=1 seed,
    smoke-level claim only -- the parity numbers live in
    isim/specs/parity_flightcode_2026-09-21.md)."""
    from isim.replay_a0 import load_params
    from isim.scenario import Scenario, build

    scn = Scenario(concept="flyby", terminal="pursuit", tag_facing="rear", seed=0)
    ecfg, vehicle, target, seeker, guidance, init = build(scn, load_params())
    res = run_engagement(ecfg, vehicle, target, seeker, guidance, init)
    states = [s for _, s in guidance.state_log]
    assert "ENGAGE" in states, f"never reached ENGAGE -- states={states}"
    r0 = float(np.linalg.norm(target.state(0.0).pos_ned - init.pos_ned))
    assert res.miss_m < 2.0, (f"miss={res.miss_m:.2f} m from a {r0:.1f} m start -- "
                              "the port did not close range through the harness")
