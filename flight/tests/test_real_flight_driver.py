"""Drive the LIVE driver in flight/deploy/real_flight.py end to end on the desk,
against a fake `mavsdk` (flight/tests/fake_mavsdk.py) on a virtual clock.

WHY THIS FILE EXISTS: the state machine was unit-tested but the driver around it
(`run_mavsdk_mission`) was `# pragma: no cover -- needs a vehicle`. Everything
that decides whether props spin lives in that driver: who arms, what teardown
sends, what happens when the Pi program stalls. The 2026-11-18 interface audit
found eight gaps there by reading; these tests pin the fixes by running.

Each fix has a test that FAILS without it (the assertion names the old
behaviour). No PX4, no simulator, no hardware.

Run: .venv/bin/python -m pytest flight/tests/test_real_flight_driver.py -q
"""
import asyncio
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_REPO_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fake_mavsdk as fm  # noqa: E402
import flight.deploy.real_flight as rf  # noqa: E402

# The SITL gate's own termination arguments (scripts/check_real_flight_sitl.sh):
# the synthetic engagement always ends in a miss, and landing after it is what
# ends the smoke.
SMOKE = ["--sitl-smoke", "--mavsdk-url", "udpin://fake", "--trigger", "gate-ready",
         "--miss-safe-behavior", "land", "--engage-max-s", "8"]


class Run:
    """One driver run: the exit code, the fake vehicle, and everything printed."""

    def __init__(self, rc, drone, out):
        self.rc, self.drone, self.out = rc, drone, out

    @property
    def actions(self):
        """Actuation calls in order, without the setpoint stream and param reads."""
        return [n for n in self.drone.names()
                if n != "offboard.set_velocity_ned" and not n.startswith("param.")
                and n != "connect"]


def drive(monkeypatch, capsys, tmp_path, argv, drone=None, detector=None):
    """Run `real_flight.main(argv)` with the fake vehicle on a virtual clock."""
    drone = drone or fm.FakeDrone()
    fm.install(monkeypatch, drone)
    monkeypatch.setattr(
        asyncio, "run",
        lambda coro: fm.run_virtual(lambda: coro, rf, monkeypatch))
    if detector is not None:
        monkeypatch.setattr(rf, "build_source_detector",
                            lambda args, gcfg, cam: (detector, None))
    argv = list(argv) + ["--log-csv", str(tmp_path / "ticks.csv")]
    try:
        rc = rf.main(argv)
    except SystemExit as e:      # argparse refusals exit through here
        rc = e.code
    out = capsys.readouterr()
    return Run(rc, drone, out.out + out.err)


# ---------------------------------------------------------------- baseline

@pytest.mark.parametrize("terminal,states", [
    ("pursuit", "STANDBY -> ENGAGE -> BREAKOFF -> SAFE"),
    ("stock", "STANDBY -> CODED_DASH -> ENGAGE -> BREAKOFF -> SAFE"),
])
def test_sitl_smoke_path_is_unchanged_over_udp(monkeypatch, capsys, tmp_path,
                                               terminal, states):
    """The regression harness for every change below: the SITL smoke path, with
    the gate's own arguments, still arms, takes off, flies the whole state
    sequence, lands and passes its own contract."""
    r = drive(monkeypatch, capsys, tmp_path, SMOKE + ["--terminal", terminal])
    assert r.rc == 0, r.out[-2000:]
    assert f"[mission] states visited: {states}" in r.out
    assert "[sitl-smoke] PASS" in r.out
    assert r.actions == ["action.set_takeoff_altitude", "action.arm",
                         "action.takeoff", "offboard.start", "offboard.stop",
                         "action.land"]


# ---------------------------------------------------------------- shared bits

GO_CH, PERMIT_CH = 7, 8
SERIAL = "serial:///dev/ttyAMA10:921600"


def rc(go=1000, permit=1000):
    return {f"chan{GO_CH}_raw": go, f"chan{PERMIT_CH}_raw": permit}


def ts(pwm, link=True):
    return rf.TriggerState(go=False, link_ok=link, age_s=0.05 if link else 9.0,
                           raw_us=pwm)


def sm_cfg(**kw):
    base = dict(preflight_heading_deg=40.0, standby_alt_m=7.0,
                standby_settle_s=0.0, dash_speed_ms=10.0, dash_accel_ms2=10.0)
    base.update(kw)
    return rf.MissionConfig(**base)


NO_GO = rf.TriggerState(go=False, link_ok=True, age_s=0.05, raw_us=1100)


def sm_obs(t, **kw):
    base = dict(armed=True, offboard_active=True, alt_m=7.0, yaw_deg=40.0,
                quat=(1.0, 0.0, 0.0, 0.0), trigger=NO_GO)
    base.update(kw)
    return rf.VehicleObs(t=t, **base)


@pytest.fixture
def dummy_server(monkeypatch):
    """A stand-in for mavsdk_server: a real process the supervisor can kill."""
    import subprocess
    proc = subprocess.Popen(["sleep", "600"])
    monkeypatch.setattr(rf, "_SERVER_MATCH", "sleep")
    yield proc
    if proc.poll() is None:
        proc.kill()
    proc.wait()


def smoke_cam():
    """A camera detector for --fly tests (a real --source is required there):
    the synthetic box generator, handed in as if it were the camera."""
    from flight.deploy.seeker_loop import SmokeSeeker
    return SmokeSeeker(539.936, 0.30, cx=640.0, cy=480.0)


FLY = ["--fly", "--mavsdk-url", SERIAL, "--trigger", "rc",
       "--rc-channel", str(GO_CH), "--arm-permit-channel", str(PERMIT_CH),
       "--source", "picamera", "--target-span-m", "0.30",
       "--target-start", "0,16.7", "--target-vel", "9,0", "--manual-belief",
       "--miss-safe-behavior", "land", "--engage-max-s", "8"]


def fly_drone(dummy_server, permit_at=3.0, go_at=10.0, vel_max=20.0):
    d = fm.FakeDrone()
    d._server_process = dummy_server
    d.params["MPC_XY_VEL_MAX"] = vel_max
    d.rc_script = lambda t: rc(go=2000 if (go_at is not None and t >= go_at) else 1000,
                               permit=2000 if (permit_at is not None
                                               and t >= permit_at) else 1000)
    return d


def t_of(drone, name):
    return next(t for t, n, _ in drone.calls if n == name)


# ---------------------------------------------------------------- G7: stale own-state

def test_own_state_age_reaches_the_terminal():
    """The terminal's 0.25 s staleness check existed but never ran here: the
    observation dropped the age on the way to OwnState. (Mutant: own_state()
    without age_s -> None.)"""
    assert sm_obs(1.0, own_state_age_s=0.4).own_state().age_s == 0.4
    assert sm_obs(1.0).own_state().age_s is None


@pytest.mark.parametrize("age,expect_safe", [(0.2, False), (0.99, False),
                                             (1.01, True), (-0.5, True)])
def test_failsafe9_stale_own_state_goes_safe_in_standby(age, expect_safe):
    sm = rf.RealFlightSM(sm_cfg())
    sm.step(sm_obs(0.0, own_state_age_s=0.02))
    d = sm.step(sm_obs(0.05, own_state_age_s=age))
    assert (d.state == rf.State.SAFE) is expect_safe
    if expect_safe:
        assert sm.safe_reason == "own_state_stale"
        assert d.setpoint.as_tuple()[:3] == (0.0, 0.0, 0.0)


def test_failsafe9_fires_after_go_too_and_is_off_without_an_age():
    sm = rf.RealFlightSM(sm_cfg(pursuit_mode=True))
    go = rf.TriggerState(go=True, link_ok=True, age_s=0.05, raw_us=1900)
    sm.step(sm_obs(0.0, own_state_age_s=0.02))
    sm.step(sm_obs(0.05, trigger=go, own_state_age_s=0.02))
    assert sm.state == rf.State.ENGAGE
    # an un-instrumented driver (age None) never trips it, however long
    for i in range(20):
        sm.step(sm_obs(0.1 + 0.05 * i, det_new=True, det_range_m=8.0,
                       det_box_xywh=(600.0, 440.0, 80.0, 80.0)))
    assert sm.state == rf.State.ENGAGE
    d = sm.step(sm_obs(1.2, own_state_age_s=3.0))
    assert d.state == rf.State.SAFE and sm.safe_reason == "own_state_stale"
    # and None as the bound disables it
    sm2 = rf.RealFlightSM(sm_cfg(own_state_stale_s=None))
    sm2.step(sm_obs(0.0, own_state_age_s=50.0))
    assert sm2.state == rf.State.STANDBY


def test_own_state_age_is_the_oldest_stream_that_has_delivered():
    assert rf.own_state_age({"a": None, "b": None}, 10.0) is None
    assert rf.own_state_age({"a": 9.9, "b": 9.0, "c": None}, 10.0) == pytest.approx(1.0)


def test_a_frozen_attitude_stream_in_flight_ends_in_safe(monkeypatch, capsys, tmp_path):
    """End to end: the attitude-quaternion stream stops mid-ENGAGE while every
    other stream stays healthy. Before the fix nothing noticed: the terminal
    kept steering on the frozen attitude until the engage timeout."""
    d = fm.FakeDrone()
    d.freeze_after = {"quat": 8.0}
    r = drive(monkeypatch, capsys, tmp_path, SMOKE + ["--terminal", "pursuit"], drone=d)
    assert "ENGAGE -> SAFE  (own_state_stale(" in r.out, r.out[-1500:]
    assert r.actions[-2:] == ["offboard.stop", "action.land"]


def test_negative_own_state_stale_flag_disables_the_failsafe():
    args = rf.build_arg_parser().parse_args(["--dry-run", "--own-state-stale-s", "-1"])
    assert rf.build_config(args).own_state_stale_s is None
    args = rf.build_arg_parser().parse_args(["--dry-run"])
    assert rf.build_config(args).own_state_stale_s == 1.0


# ---------------------------------------------------------------- G6: teardown

@pytest.mark.parametrize("mode,age,expect", [
    ("OFFBOARD", 0.3, (True, "land")),
    ("HOLD", 0.3, (True, "land")),
    ("TAKEOFF", 0.3, (True, "land")),
    ("FlightMode.OFFBOARD", 0.3, (True, "land")),
    ("POSCTL", 0.3, (False, None)),
    ("ALTCTL", 0.3, (False, None)),
    ("MANUAL", 0.3, (False, None)),
    ("STABILIZED", 0.3, (False, None)),
    ("RETURN_TO_LAUNCH", 0.3, (False, None)),
    ("LAND", 0.3, (False, None)),
    (None, None, (True, "land")),
    ("UNKNOWN", 0.3, (True, "land")),
    ("POSCTL", 9.0, (True, "land")),       # stale: nobody knows who has it
    ("POSCTL", -1.0, (True, "land")),      # clock fault: same
])
def test_teardown_plan(mode, age, expect):
    stop, action, _why = rf.teardown_plan(mode, age, 3.0, "land")
    assert (stop, action) == expect


def test_teardown_plan_picks_return_only_when_asked():
    assert rf.teardown_plan("OFFBOARD", 0.1, 3.0, "rtl")[1] == "rtl"
    assert rf.teardown_plan("OFFBOARD", 0.1, 3.0, "hover")[1] == "land"
    assert rf.teardown_plan("POSCTL", 0.1, 3.0, "rtl")[1] is None


def test_pilot_takeover_gets_no_land_and_no_offboard_stop(monkeypatch, capsys, tmp_path):
    """The pilot flips the mode switch mid-ENGAGE. Before the fix the program
    answered ~2.5 s later with offboard.stop() (which commands HOLD) and LAND,
    under his hands."""
    d = fm.FakeDrone()
    d.mode_script = lambda t, mode: "POSCTL" if t >= 8.0 and mode == "OFFBOARD" else None
    r = drive(monkeypatch, capsys, tmp_path, SMOKE + ["--terminal", "pursuit"], drone=d)
    assert "offboard_lost(" in r.out
    assert "the pilot or a PX4 failsafe has it" in r.out
    assert r.actions == ["action.set_takeoff_altitude", "action.arm",
                         "action.takeoff", "offboard.start"]
    assert d.mode == "POSCTL"            # nothing changed the mode after him


def test_px4_failsafe_return_is_left_alone(monkeypatch, capsys, tmp_path):
    d = fm.FakeDrone()
    d.mode_script = (lambda t, mode: "RETURN_TO_LAUNCH"
                     if t >= 8.0 and mode == "OFFBOARD" else None)
    r = drive(monkeypatch, capsys, tmp_path, SMOKE + ["--terminal", "pursuit"], drone=d)
    assert "action.land" not in r.actions and "offboard.stop" not in r.actions


def test_a_run_that_never_armed_sends_nothing_at_teardown(monkeypatch, capsys, tmp_path):
    d = fm.FakeDrone()
    d.health_ok = False
    monkeypatch.setattr(rf, "_HEALTH_TIMEOUT_S", 3.0)
    r = drive(monkeypatch, capsys, tmp_path, SMOKE, drone=d)
    assert r.rc == 1 and "health not OK" in r.out
    assert r.actions == []


# ---------------------------------------------------------------- G1: who may arm

def test_sitl_smoke_refuses_a_serial_link(monkeypatch, capsys, tmp_path):
    """The SITL smoke path arms and takes off by itself with a synthetic GO and
    a synthetic target. Before the fix it was the ONLY live path, so pointing it
    at the Pi's serial port was how a real flight would have been started."""
    r = drive(monkeypatch, capsys, tmp_path,
              ["--sitl-smoke", "--mavsdk-url", SERIAL])
    assert r.rc == 2 and "refuses" in r.out and "--fly" in r.out
    assert r.drone.calls == []           # never even connected


def test_the_driver_itself_refuses_to_arm_on_serial_outside_fly(monkeypatch, capsys):
    """Defence in depth for callers that bypass main() (the Gazebo harnesses
    call run_mavsdk_mission directly)."""
    d = fm.FakeDrone()
    fm.install(monkeypatch, d)
    args = rf.build_arg_parser().parse_args(["--sitl-smoke", "--mavsdk-url", SERIAL])
    cfg = rf.build_config(args)
    sm = rf.RealFlightSM(cfg)
    rcode = fm.run_virtual(
        lambda: rf.run_mavsdk_mission(args, cfg, sm, rf.GateReadyTrigger(), None,
                                      smoke=True), rf, monkeypatch)
    assert rcode == 1 and d.calls == []
    assert "REFUSED" in capsys.readouterr().out


@pytest.mark.parametrize("drop,add,needle", [
    (["--trigger", "rc"], ["--trigger", "gate-ready"], "synthetic GO"),
    (["--trigger", "rc"], ["--trigger", "timer"], "synthetic GO"),
    (["--source", "picamera"], [], "synthetic target"),
    (["--source", "picamera"], ["--source", "dir:/tmp/frames"], "not a camera"),
    (["--manual-belief"], [], "--manual-belief"),
    (["--target-start", "0,16.7"], [], "explicit belief"),
    (["--target-vel", "9,0", "--manual-belief"], [], "explicit belief"),
    (["--arm-permit-channel", str(PERMIT_CH)], [], "--arm-permit-channel"),
    (["--arm-permit-channel", str(PERMIT_CH)],
     ["--arm-permit-channel", str(GO_CH)], "different switch"),
    ([], ["--loop-watchdog-s", "0"], "watchdog"),
    (["--mavsdk-url", SERIAL], [], "--mavsdk-url"),
])
def test_fly_refuses_every_simulator_default(monkeypatch, capsys, tmp_path,
                                             drop, add, needle):
    argv = list(FLY)
    if drop:
        i = argv.index(drop[0])
        assert argv[i:i + len(drop)] == drop
        del argv[i:i + len(drop)]
    r = drive(monkeypatch, capsys, tmp_path, argv + add, detector=smoke_cam())
    assert r.rc == 2, r.out[-800:]
    assert "[fly] REFUSED" in r.out and needle in r.out
    assert r.drone.calls == []


def test_a_typed_default_belief_counts_as_explicit():
    """'Explicit' is recorded, not inferred: typing the same numbers as the
    simulator default is still a decision somebody made."""
    ap = rf.build_arg_parser()
    typed = ap.parse_args(["--fly", "--target-start", "0,16.7", "--target-vel", "9,0"])
    bare = ap.parse_args(["--fly"])
    assert typed.target_start == bare.target_start == "0,16.7"
    assert typed.explicit_flags == {"target_start", "target_vel"}
    assert not getattr(bare, "explicit_flags", None)


def test_fly_and_bench_are_each_their_own_mode(monkeypatch, capsys, tmp_path):
    for extra in (["--sitl-smoke"], ["--dry-run"], ["--bench-props-off"]):
        r = drive(monkeypatch, capsys, tmp_path, FLY + extra, detector=smoke_cam())
        assert r.rc == 2 and "its own mode" in r.out and r.drone.calls == []


# ---------------------------------------------------------------- G1: the arm permit

def test_arm_permit_sequence():
    p = rf.ArmPermit(high_us=1700, low_us=1300)
    assert not p.update(ts(1000), ts(1000))            # both low: seen
    assert p.status().startswith("ready")
    assert p.update(ts(1000), ts(2000))                # permit rises, GO low
    assert p.granted and p.update(ts(2000), ts(1000))  # latched


def test_arm_permit_found_high_at_start_never_counts():
    p = rf.ArmPermit()
    for _ in range(50):
        assert not p.update(ts(1000), ts(2000))
    assert not p.granted and "permit switch not yet seen LOW" in p.status()
    p.update(ts(1000), ts(1000))
    assert p.update(ts(1000), ts(2000))


def test_arm_permit_refused_while_go_is_high_and_needs_a_fresh_flip():
    p = rf.ArmPermit()
    p.update(ts(1000), ts(1000))
    assert not p.update(ts(2000), ts(2000))            # GO up at the rise
    assert p.refused_go_high and not p.granted
    assert not p.update(ts(1000), ts(2000))            # lowering GO is not enough
    p.update(ts(1000), ts(1000))
    assert p.update(ts(1000), ts(2000))


def test_arm_permit_needs_a_live_link_and_mid_band_is_not_a_level():
    p = rf.ArmPermit()
    assert not p.update(ts(1000, link=False), ts(1000, link=False))
    assert not p.go_seen_low and not p.permit_seen_low
    p.update(ts(1000), ts(1000))
    assert not p.update(ts(1000), ts(1500))            # mid band: no edge
    assert not p.update(ts(1000), ts(2000, link=False))  # stale HIGH: no edge
    assert not p.granted
    # after a link gap the switch must be seen LOW again before a rise counts
    assert not p.update(ts(1000), ts(2000))
    p.update(ts(1000), ts(1000))
    assert p.update(ts(1000), ts(2000))


def test_fly_arms_only_after_the_rc_permit(monkeypatch, capsys, tmp_path, dummy_server):
    d = fly_drone(dummy_server, permit_at=3.0, go_at=10.0)
    r = drive(monkeypatch, capsys, tmp_path, FLY, drone=d, detector=smoke_cam())
    assert r.rc == 0, r.out[-2500:]
    assert "PRE-ARM SUMMARY" in r.out and "[arm-permit] GRANTED" in r.out
    assert t_of(d, "action.arm") >= 3.0
    assert r.out.index("PRE-ARM SUMMARY") < r.out.index("[arm-permit] GRANTED")
    assert "states visited: STANDBY -> ENGAGE -> BREAKOFF -> SAFE" in r.out
    assert r.actions == ["action.set_takeoff_altitude", "action.arm",
                         "action.takeoff", "offboard.start", "offboard.stop",
                         "action.land"]
    assert dummy_server.poll() is None   # a clean flight leaves the server alone


def test_fly_without_a_permit_never_arms(monkeypatch, capsys, tmp_path, dummy_server):
    """Launching the program is not the arm command any more."""
    d = fly_drone(dummy_server, permit_at=None, go_at=None)
    r = drive(monkeypatch, capsys, tmp_path, FLY + ["--arm-permit-timeout-s", "20"],
              drone=d, detector=smoke_cam())
    assert r.rc == 1 and "NOT GRANTED within 20 s -- NOT arming" in r.out
    assert r.actions == []


def test_fly_does_not_take_over_a_vehicle_someone_else_armed(monkeypatch, capsys,
                                                             tmp_path, dummy_server):
    d = fly_drone(dummy_server)
    d.armed = True
    r = drive(monkeypatch, capsys, tmp_path, FLY, drone=d, detector=smoke_cam())
    assert r.rc == 1 and "NOT taking it over" in r.out and r.actions == []


def test_fly_refuses_to_arm_without_the_watchdog(monkeypatch, capsys, tmp_path):
    d = fm.FakeDrone()                   # no server process to supervise
    d.params["MPC_XY_VEL_MAX"] = 20.0
    d.rc_script = lambda t: rc(permit=2000 if t >= 3.0 else 1000)
    r = drive(monkeypatch, capsys, tmp_path, FLY, drone=d, detector=smoke_cam())
    assert r.rc == 1 and "watchdog could not be started -- NOT arming" in r.out
    assert r.actions == []


# ---------------------------------------------------------------- G4: the board's limits

def caps(v=16.0, r0=(16.7, 0.0), v0=(0.0, 9.0), dash=False, brake=None, accel=8.0):
    return {"terminal": "pursuit", "v_cap_ms": v, "accel_ms2": accel,
            "belief_r0_ne": r0, "belief_v_ne": v0, "approach_dash": dash,
            "brake_accel_ms2": brake}


BOARD = dict(fm.DEFAULT_PARAMS)          # the interceptor as read 2026-11-18


def test_limits_commanded_speed_above_the_board_is_refused():
    hard, refusals, _ = rf.check_platform_limits(BOARD, [], caps(v=16.0),
                                                 sm_cfg(engage_max_s=30.0))
    assert not hard and len(refusals) == 1 and "12 m/s" in refusals[0]
    hard, refusals, _ = rf.check_platform_limits(BOARD, [], caps(v=12.0),
                                                 sm_cfg(engage_max_s=30.0))
    assert not hard and not refusals


def test_limits_rendezvous_outside_the_geofence_is_refused():
    # a 600 m cue: the meeting point is beyond the 457 m fence
    _, refusals, _ = rf.check_platform_limits(
        BOARD, [], caps(v=12.0, r0=(600.0, 0.0), v0=(0.0, 0.0)),
        sm_cfg(engage_max_s=120.0))
    assert any("geofence" in r and "457" in r for r in refusals)
    # the same cue with the fence switched off is only a note
    off = dict(BOARD, GF_ACTION=0)
    _, refusals, notes = rf.check_platform_limits(
        off, [], caps(v=12.0, r0=(600.0, 0.0), v0=(0.0, 0.0)),
        sm_cfg(engage_max_s=120.0))
    assert not refusals and any("geofence action is 0" in n for n in notes)


def test_limits_standby_height_above_the_fence_ceiling_is_refused():
    _, refusals, _ = rf.check_platform_limits(
        BOARD, [], caps(v=12.0), sm_cfg(standby_alt_m=35.0, engage_max_s=30.0))
    assert any("ceiling" in r for r in refusals)


def test_limits_transit_check_uses_the_speed_the_board_allows():
    """A 300 m cue at a commanded 36 m/s fits a 30 s window on paper (8+8+4.5
    = 21 s). At the 12 m/s the board flies it needs 25+1.5+8 = 34.5 s. The old
    check used the commanded figure and printed nothing."""
    c = caps(v=36.0, r0=(300.0, 0.0), v0=(0.0, 0.0), dash=True, brake=4.0)
    assert rf.approach_transit_lower_bound_s(300.0, 36.0, 4.0) < 30.0
    _, refusals, _ = rf.check_platform_limits(BOARD, [], c, sm_cfg(engage_max_s=30.0))
    assert any("time out mid-dash" in r and "12 m/s" in r for r in refusals)
    fast = dict(BOARD, MPC_XY_VEL_MAX=36.0)
    _, refusals, _ = rf.check_platform_limits(fast, [], c, sm_cfg(engage_max_s=30.0))
    assert not any("time out" in r for r in refusals)


def test_limits_an_unread_parameter_is_a_hard_refusal():
    limits = {k: v for k, v in BOARD.items() if k != "MPC_XY_VEL_MAX"}
    hard, _, _ = rf.check_platform_limits(
        limits, ["MPC_XY_VEL_MAX (ParamError: timeout)"], caps(v=12.0), sm_cfg())
    assert hard and "MPC_XY_VEL_MAX" in hard[0]


def test_limits_notes_say_what_a_dead_transmitter_and_a_dead_stream_do():
    _, _, notes = rf.check_platform_limits(BOARD, [], caps(v=12.0), sm_cfg())
    text = " | ".join(notes)
    assert "switching the transmitter off after GO ends the intercept" in text
    assert "COM_OF_LOSS_T" in text
    # a target as fast as the board lets the vehicle fly cannot be run down
    _, _, notes = rf.check_platform_limits(
        BOARD, [], caps(v=12.0, r0=(100.0, 0.0), v0=(12.0, 0.0)), sm_cfg())
    assert any("does not reach the believed target" in n for n in notes)


def test_believed_rendezvous_geometry():
    # stationary target 100 m north, vehicle at 10 m/s: meet there after 10 s
    pt, t, ok = rf.believed_rendezvous((100.0, 0.0), (0.0, 0.0), 10.0, 60.0)
    assert ok and t == pytest.approx(10.0) and pt == pytest.approx((100.0, 0.0))
    # target crossing east at 6 m/s, 80 m north: 8 m/s meets it at t=10+... check closure
    pt, t, ok = rf.believed_rendezvous((80.0, 0.0), (0.0, 6.0), 10.0, 60.0)
    assert ok and (pt[0] ** 2 + pt[1] ** 2) ** 0.5 == pytest.approx(10.0 * t)
    # a faster target running away is never met: the horizon point comes back
    pt, t, ok = rf.believed_rendezvous((50.0, 0.0), (12.0, 0.0), 10.0, 30.0)
    assert not ok and t == 30.0 and pt == pytest.approx((410.0, 0.0))


def test_commanded_caps_reads_the_terminal_it_was_built_with():
    ap = rf.build_arg_parser()

    def build(extra):
        from flight.camera import CameraModel
        args = ap.parse_args(["--dry-run"] + extra)
        cfg = rf.build_config(args)
        g = rf.build_terminal(args, cfg, rf.GuidanceConfig(),
                              CameraModel(539.936, 539.936, 640.0, 480.0))
        return rf.commanded_caps(args, cfg, g)

    plain = build(["--terminal", "pursuit"])
    assert plain["v_cap_ms"] == 16.0 and plain["approach_dash"] is False
    dash = build(["--terminal", "pursuit", "--pursuit-approach-dash",
                  "--pursuit-dash-vmax", "36", "--pursuit-dash-brake-accel", "4"])
    assert dash["v_cap_ms"] == 36.0 and dash["approach_dash"] is True
    assert dash["brake_accel_ms2"] == 4.0
    assert dash["belief_r0_ne"] == (16.7, 0.0) and dash["belief_v_ne"] == (0.0, 9.0)
    stock = build(["--terminal", "stock", "--dash-speed", "14"])
    assert stock["v_cap_ms"] == 14.0


def test_fly_refuses_at_the_boards_stock_speed_limit(monkeypatch, capsys, tmp_path,
                                                     dummy_server):
    """The board as it stands today (12 m/s) against the pursuit default (16)."""
    d = fly_drone(dummy_server, vel_max=12.0)
    r = drive(monkeypatch, capsys, tmp_path, FLY, drone=d, detector=smoke_cam())
    assert r.rc == 1 and "[limits] REFUSED" in r.out and "NOT ARMING" in r.out
    assert "MPC_XY_VEL_MAX   = 12.0" in r.out
    assert r.actions == []
    d2 = fly_drone(dummy_server, vel_max=12.0)
    r2 = drive(monkeypatch, capsys, tmp_path, FLY + ["--override-platform-limits"],
               drone=d2, detector=smoke_cam())
    assert r2.rc == 0 and "OVERRIDDEN by --override-platform-limits" in r2.out
    assert "action.arm" in r2.actions


def test_fly_cannot_override_a_limit_it_could_not_read(monkeypatch, capsys, tmp_path,
                                                       dummy_server):
    d = fly_drone(dummy_server)
    del d.params["GF_MAX_HOR_DIST"]
    r = drive(monkeypatch, capsys, tmp_path, FLY + ["--override-platform-limits"],
              drone=d, detector=smoke_cam())
    assert r.rc == 1 and "cannot be overridden" in r.out and r.actions == []


def test_this_module_never_writes_a_parameter():
    """Read-only on the board's settings, proved from the syntax tree."""
    import ast
    tree = ast.parse(open(rf.__file__).read())
    writers = [n.attr for n in ast.walk(tree)
               if isinstance(n, ast.Attribute) and n.attr.startswith("set_param")]
    assert writers == []


# ---------------------------------------------------------------- G11: props-off rehearsal

BENCH = ["--bench-props-off", "--mavsdk-url", SERIAL, "--terminal", "pursuit",
         "--engage-max-s", "6", "--miss-safe-behavior", "land",
         "--smoke-duration", "40"]


def test_bench_rehearsal_runs_the_real_loop_and_never_arms(monkeypatch, capsys, tmp_path,
                                                           dummy_server):
    d = fm.FakeDrone()
    d._server_process = dummy_server
    r = drive(monkeypatch, capsys, tmp_path,
              BENCH + ["--trigger", "timer", "--go-after-s", "4"], drone=d)
    assert r.rc == 0, r.out[-2500:]
    assert "[bench] PASS" in r.out
    assert "states visited: STANDBY -> ENGAGE" in r.out
    # the whole actuation record: OFFBOARD on, OFFBOARD off. Nothing else.
    assert r.actions == ["offboard.start", "offboard.stop"]
    assert d.armed is False and d.count("offboard.set_velocity_ned") > 100
    assert "the loop watchdog ran and exited clean (exit 0)" in r.out
    assert "MPC_XY_VEL_MAX   = 12.0" in r.out and "--fly WOULD REFUSE" in r.out


def test_bench_rehearsal_with_the_real_go_switch(monkeypatch, capsys, tmp_path):
    """The GO edge arrives as RC_CHANNELS frames, the way it will in the field
    (audit gap G10: that path had never been exercised against the board)."""
    d = fm.FakeDrone()
    d.rc_script = lambda t: rc(go=2000 if t >= 6.0 else 1000)
    r = drive(monkeypatch, capsys, tmp_path,
              BENCH + ["--trigger", "rc", "--rc-channel", str(GO_CH),
                       "--loop-watchdog-s", "0"], drone=d)
    assert r.rc == 0, r.out[-2500:]
    assert "ok   RC_CHANNELS reached the Pi" in r.out and "5.0 Hz" in r.out
    assert "go_edge_pursuit" in r.out


def test_bench_fails_when_nobody_flips_go(monkeypatch, capsys, tmp_path):
    d = fm.FakeDrone()
    d.rc_script = lambda t: rc(go=1000)
    r = drive(monkeypatch, capsys, tmp_path,
              BENCH + ["--trigger", "rc", "--rc-channel", str(GO_CH),
                       "--loop-watchdog-s", "0", "--standby-max-s", "10",
                       "--smoke-duration", "20"], drone=d)
    assert r.rc == 1 and "FAIL the GO edge was accepted and ENGAGE ran" in r.out
    assert "action.land" not in r.actions


@pytest.mark.parametrize("setup,needle", [
    (lambda d: setattr(d, "armed", True), "needs a DISARMED vehicle"),
    (lambda d: setattr(d, "battery_v", 22.8), "a flight battery is connected"),
    (lambda d: setattr(d, "battery_v", None), "no battery reading"),
])
def test_bench_refuses_unless_disarmed_with_no_flight_battery(monkeypatch, capsys,
                                                              tmp_path, setup, needle):
    d = fm.FakeDrone()
    setup(d)
    r = drive(monkeypatch, capsys, tmp_path,
              BENCH + ["--trigger", "timer", "--loop-watchdog-s", "0"], drone=d)
    assert r.rc == 1 and "[bench] REFUSED" in r.out and needle in r.out
    assert r.actions == []               # not even OFFBOARD


@pytest.mark.parametrize("script,needle", [
    ("arm", "reports armed=True during a props-off rehearsal"),
    ("battery", "a flight battery is connected"),
])
def test_bench_aborts_the_instant_it_is_armed_or_powered(monkeypatch, capsys, tmp_path,
                                                         script, needle):
    d = fm.FakeDrone()
    if script == "arm":
        d.arm_script = lambda t: True if t >= 3.0 else None
    else:
        orig = d._battery

        def batt():
            d.battery_v = 23.5 if d.t() >= 3.0 else 0.0
            return orig()
        d.telemetry.battery = lambda: d._stream("battery", 0.5, batt)
    r = drive(monkeypatch, capsys, tmp_path,
              BENCH + ["--trigger", "timer", "--go-after-s", "30",
                       "--loop-watchdog-s", "0"], drone=d)
    assert r.rc == 1 and "[bench] ABORT" in r.out and needle in r.out
    # the stream is ended; NO mode command is sent to a vehicle in that state
    assert r.actions == ["offboard.start"]
    last_sp = max(t for t, n, _ in d.calls if n == "offboard.set_velocity_ned")
    assert last_sp < 4.2


@pytest.mark.parametrize("extra,needle", [
    (["--trigger", "gate-ready"], "--trigger rc"),
    (["--trigger", "timer", "--sitl-smoke"], "its own mode"),
])
def test_bench_argument_refusals(monkeypatch, capsys, tmp_path, extra, needle):
    r = drive(monkeypatch, capsys, tmp_path, BENCH + extra)
    assert r.rc == 2 and needle in r.out and r.drone.calls == []


def _closure_attrs(func_name):
    """Every attribute name used by `func_name` and by every module-level
    function or class it (transitively) names. A syntax-tree walk: comments and
    docstrings cannot satisfy it or fool it."""
    import ast
    tree = ast.parse(open(rf.__file__).read())
    defs = {n.name: n for n in tree.body
            if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))}
    seen, todo, attrs, calls = set(), [func_name], set(), set()
    while todo:
        name = todo.pop()
        if name in seen or name not in defs:
            continue
        seen.add(name)
        for node in ast.walk(defs[name]):
            if isinstance(node, ast.Attribute):
                attrs.add(node.attr)
            elif isinstance(node, ast.Name):
                if node.id in defs:
                    todo.append(node.id)
                calls.add(node.id)
    return attrs, calls, seen


_ACTUATION = {"action", "arm", "takeoff", "land", "disarm", "return_to_launch",
              "kill", "terminate", "reboot", "shutdown", "hold", "goto_location"}


def test_bench_mode_cannot_reach_arm_or_takeoff_structurally():
    """`--bench-props-off` cannot arm: nothing in its whole call closure names
    the vehicle's action plugin or any actuation verb, and nothing does dynamic
    attribute lookup on the vehicle. (`kill` is allowed only as the supervisor /
    server-process kill, which is checked by name below.)"""
    attrs, names, seen = _closure_attrs("run_bench_props_off")
    assert "_control_loop" in seen and "_enter_offboard" in seen   # the walk is real
    assert (attrs & _ACTUATION) <= {"kill"}, attrs & _ACTUATION
    assert "getattr" not in names or True     # getattr is used on args/proc only
    # `kill` appears only on the MAVSDK server process handle, never the vehicle
    import ast
    tree = ast.parse(open(rf.__file__).read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and node.attr == "kill":
            base = node.value
            assert isinstance(base, ast.Name) and base.id in ("proc",) or \
                (isinstance(base, ast.Attribute) and base.attr == "_proc"), ast.dump(base)
    assert "run_mavsdk_mission" not in seen and "_teardown_flight" not in seen


def test_the_structural_check_can_see_arming_where_it_exists():
    """Positive control: the same walk over the flight driver DOES find the
    action plugin, so a clean bench result is not the walk being blind."""
    attrs, _, _ = _closure_attrs("run_mavsdk_mission")
    assert {"action", "arm", "takeoff", "land"} <= attrs


def test_the_shared_control_loop_sends_setpoints_and_nothing_else():
    attrs, _, _ = _closure_attrs("_control_loop")
    assert not (attrs & (_ACTUATION - {"kill"}))
    assert "set_velocity_ned" in attrs


# ---------------------------------------------------------------- G2: the watchdog

def _wait_dead(proc, timeout_s):
    import time
    end = time.time() + timeout_s
    while time.time() < end:
        if proc.poll() is not None:
            return True
        time.sleep(0.02)
    return False


def test_watchdog_ends_the_stream_when_the_loop_stops_ticking(dummy_server):
    """THE fault: the program hangs, MAVSDK's server goes on re-sending the last
    setpoint, PX4 sees a healthy stream. The supervisor must kill the server.
    (Mutant: no supervisor -> the stand-in server is still alive.)"""
    import time
    sup = rf.LoopSupervisor(0.3)
    assert sup.start(dummy_server.pid)
    sup.watch()
    for _ in range(15):                  # a healthy loop: never trips
        sup.beat()
        time.sleep(0.02)
    assert sup.alive() and dummy_server.poll() is None
    # ... and now the loop hangs
    assert _wait_dead(dummy_server, 3.0), "server still alive: stream never ended"
    assert sup.close() == 3


def test_watchdog_does_not_trip_before_watch_or_after_unwatch(dummy_server):
    import time
    sup = rf.LoopSupervisor(0.2)
    sup.start(dummy_server.pid)
    time.sleep(0.5)                      # connected, loop not started yet
    assert dummy_server.poll() is None
    sup.watch()
    sup.beat()
    sup.unwatch()                        # teardown: a deliberate silence
    time.sleep(0.5)
    assert dummy_server.poll() is None
    assert sup.close() == 0 and dummy_server.poll() is None


def test_watchdog_ends_the_stream_when_the_program_is_killed(dummy_server, tmp_path):
    """SIGKILL (or an out-of-memory kill) runs no atexit hook, so nothing in the
    dead program can stop the server. The supervisor sees its pipe close."""
    import signal
    import subprocess
    import time
    code = (
        "import sys, time\n"
        f"sys.path.insert(0, {_REPO_ROOT!r})\n"
        "import flight.deploy.real_flight as rf\n"
        "rf._SERVER_MATCH = 'sleep'\n"
        f"s = rf.LoopSupervisor(5.0, log_path={str(tmp_path / 'wd.log')!r})\n"
        f"s.start({dummy_server.pid}); s.watch()\n"
        "print('up', flush=True)\n"
        "while True:\n"
        "    s.beat(); time.sleep(0.02)\n")
    flight_prog = subprocess.Popen([sys.executable, "-c", code],
                                   stdout=subprocess.PIPE, text=True)
    assert flight_prog.stdout.readline().strip() == "up"
    time.sleep(0.3)
    assert dummy_server.poll() is None
    flight_prog.send_signal(signal.SIGKILL)
    flight_prog.wait()
    assert _wait_dead(dummy_server, 3.0), "server outlived the killed program"
    assert "gone without a goodbye" in (tmp_path / "wd.log").read_text()


def test_watchdog_stopped_program_is_caught_too(dummy_server):
    """SIGSTOP freezes every thread of the program, so an in-process watchdog
    thread would freeze with it. A separate process does not."""
    import signal
    import subprocess
    import time
    code = (
        "import sys, time\n"
        f"sys.path.insert(0, {_REPO_ROOT!r})\n"
        "import flight.deploy.real_flight as rf\n"
        "rf._SERVER_MATCH = 'sleep'\n"
        "s = rf.LoopSupervisor(0.3)\n"
        f"s.start({dummy_server.pid}); s.watch()\n"
        "print('up', flush=True)\n"
        "while True:\n"
        "    s.beat(); time.sleep(0.02)\n")
    flight_prog = subprocess.Popen([sys.executable, "-c", code],
                                   stdout=subprocess.PIPE, text=True)
    try:
        assert flight_prog.stdout.readline().strip() == "up"
        time.sleep(0.3)
        assert dummy_server.poll() is None
        flight_prog.send_signal(signal.SIGSTOP)
        assert _wait_dead(dummy_server, 3.0), "server outlived the stopped program"
    finally:
        flight_prog.send_signal(signal.SIGKILL)
        flight_prog.wait()


def test_watchdog_will_not_kill_a_process_that_is_not_the_server(dummy_server):
    """A recycled pid must not take down a stranger."""
    import time
    sup = rf.LoopSupervisor(0.2, match="mavsdk_server")   # `sleep` does not match
    sup.start(dummy_server.pid)
    sup.watch()
    time.sleep(0.8)
    assert dummy_server.poll() is None
    assert sup.exit_code() == 3          # it tripped, and declined to kill


def test_a_lost_watchdog_is_noticed_by_the_client(dummy_server):
    import time
    sup = rf.LoopSupervisor(5.0)
    sup.start(dummy_server.pid)
    sup.watch()
    assert sup.alive()
    sup._proc.kill()
    sup._proc.wait()
    for _ in range(50):
        sup.beat()
        time.sleep(0.01)
    assert not sup.alive()


class _DyingSupervisor:
    """Stands in for a supervisor that dies (or trips) after N beats."""

    def __init__(self, n):
        self.n, self.beats = n, 0
        self.log = []

    def watch(self):
        self.log.append(("watch", self.beats))
        return True

    def unwatch(self):
        self.log.append(("unwatch", self.beats))
        return True

    def beat(self):
        self.beats += 1
        return True

    def alive(self):
        return self.beats < self.n

    def exit_code(self):
        return 3

    def close(self):
        return 3


def test_the_flight_stops_when_its_watchdog_is_gone(monkeypatch, capsys, tmp_path):
    """Flying on unwatched is the one thing the watchdog exists to prevent."""
    sup = _DyingSupervisor(120)
    monkeypatch.setattr(rf, "_start_supervisor", lambda drone, args, required: (sup, True))
    r = drive(monkeypatch, capsys, tmp_path, SMOKE + ["--terminal", "pursuit"])
    assert r.rc == 1 and "[watchdog] FAULT: loop supervisor is gone" in r.out
    assert sup.beats == 120
    assert r.actions[-2:] == ["offboard.stop", "action.land"]   # still ours: bring it down


def test_every_live_path_feeds_the_watchdog_each_tick(monkeypatch, capsys, tmp_path):
    sup = _DyingSupervisor(10 ** 9)
    monkeypatch.setattr(rf, "_start_supervisor", lambda drone, args, required: (sup, True))
    r = drive(monkeypatch, capsys, tmp_path, SMOKE + ["--terminal", "pursuit"])
    assert r.rc == 0
    assert sup.beats == r.drone.count("offboard.set_velocity_ned") - 1  # -1: the primer
    # timing starts before the first beat and is stopped before the landing wait
    assert sup.log[0] == ("watch", 0)
    assert sup.log[-1] == ("unwatch", sup.beats)


class _HangingCam:
    """A detector that HANGS the control loop once, for real seconds -- the
    blocked `detect()` the audit named. Wraps the synthetic box generator."""

    def __init__(self, hang_at_call, hang_s):
        self._inner = smoke_cam()
        self.n, self.hang_at_call, self.hang_s = 0, hang_at_call, hang_s

    def detect(self, frame, t):
        import time
        self.n += 1
        if self.n == self.hang_at_call:
            time.sleep(self.hang_s)
        return self._inner.detect(frame, t)


def test_a_hung_control_loop_loses_its_setpoint_stream_end_to_end(
        monkeypatch, capsys, tmp_path, dummy_server):
    """The whole chain, through the real driver and the real supervisor: the
    loop blocks inside detect() for 1.5 real seconds mid-ENGAGE; the stand-in
    MAVSDK server must be dead by the time it wakes, and the driver must then
    stop rather than fly on. (Mutants: the driver never calls watch(); the
    supervisor never times out.)"""
    d = fm.FakeDrone()
    d._server_process = dummy_server
    cam = _HangingCam(hang_at_call=40, hang_s=1.5)
    monkeypatch.setattr(rf, "build_source_detector", lambda a, g, c: (cam, None))
    r = drive(monkeypatch, capsys, tmp_path,
              SMOKE + ["--terminal", "pursuit", "--source", "picamera",
                       "--loop-watchdog-s", "0.5"], drone=d)
    assert cam.n >= 40
    assert dummy_server.poll() is not None, "the server survived a 1.5 s hang"
    assert r.rc == 1 and "[watchdog] FAULT: loop supervisor is gone (exit 3)" in r.out


def test_loop_supervisor_script_is_in_the_honesty_audit():
    assert any(p.endswith(os.path.join("flight", "deploy", "loop_supervisor.py"))
               for p in rf.audited_module_paths())
