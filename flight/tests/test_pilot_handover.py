"""`--pilot-handover` (2026-11-26): the pilot hovers, the Pixhawk's offboard
switch hands the aircraft to the program and takes it back, and every command
is slow and level. Driven end to end on the fake vehicle."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fake_mavsdk as fm  # noqa: E402
import flight.deploy.real_flight as rf  # noqa: E402
from flight.cue_link import encode_record  # noqa: E402
from flight.cue_relay import CueRecord  # noqa: E402
from test_cue_go import capture_trigger, lla  # noqa: E402
from test_real_flight_driver import (_ACTUATION, _closure_attrs, GO_CH, SERIAL,  # noqa: E402,F401
                                     drive, dummy_server, rc)

HANDOVER = ["--pilot-handover", "--mavsdk-url", SERIAL, "--terminal", "pursuit",
            "--trigger", "cue", "--rc-channel", str(GO_CH), "--cue-udp-port", "0",
            "--cue-allow-unsynced",
            "--standby-max-s", "600", "--smoke-duration", "120"]
P0, V, UP = (12.0, 5.0), (0.6, 0.6), 2.0     # a slow target 13 m off, 2 m higher


def slow_feeder(drone, box, period=0.5, delay_s=0.2):
    """The target's reports at the control-link route's 2 a second."""
    st = {"k": 0}

    def hook(t):
        trig = box.get("trig")
        if trig is None:
            return
        t_abs0 = drone.now() - t
        while st["k"] * period + delay_s <= t:
            ts = st["k"] * period
            la, lo, al = lla(P0[0] + V[0] * ts, P0[1] + V[1] * ts, up=UP, own=drone.lla)
            trig.receiver.feed(encode_record(CueRecord(0.0, la, lo, al, V[0], V[1], 0.0),
                                             t_src=ts, t_utc=drone.utc0 + ts),
                               t_rx=t_abs0 + ts + delay_s)
            st["k"] += 1
    return hook


def hovering_drone(server, go_up=10.0, go_down=40.0, offboard_switch=True):
    """Armed, in the air at 5 m in Position mode. The GO switch goes up at
    `go_up` and down at `go_down`; with `offboard_switch` the fake Pixhawk does
    what RC_MAP_OFFB_SW does: OFFBOARD while it is up, Position when it drops."""
    d = fm.FakeDrone()
    d._server_process = server
    d.armed, d.mode, d.alt, d.yaw_deg = True, "POSCTL", 5.0, 80.0
    d.landed = fm.LandedState.IN_AIR
    d.battery_v = 23.0
    up = lambda t: go_up <= t < go_down                                # noqa: E731
    d.rc_script = lambda t: rc(go=2000 if up(t) else 1000)
    if offboard_switch:
        d.mode_script = lambda t, m: "OFFBOARD" if up(t) else "POSCTL"
    return d


def sent(d):
    import types
    return [(t, types.SimpleNamespace(north_m_s=a[0], east_m_s=a[1], down_m_s=a[2],
                                      yaw_deg=a[3]))
            for t, name, a in d.calls if name == "offboard.set_velocity_ned"]


def run(monkeypatch, capsys, tmp_path, d, extra=()):
    box = capture_trigger(monkeypatch)
    d.tick_hook = slow_feeder(d, box)
    r = drive(monkeypatch, capsys, tmp_path, HANDOVER + list(extra), drone=d)
    return r, box["trig"]


def test_handover_flies_toward_the_target_slowly_level_and_gives_back(monkeypatch, capsys,
                                                                     tmp_path, dummy_server):
    d = hovering_drone(dummy_server)
    r, trig = run(monkeypatch, capsys, tmp_path, d)
    assert r.rc == 0, r.out[-3000:]
    # it never armed, took off, landed, or commanded a mode
    assert r.actions == [], r.actions
    assert trig.fired and "go_edge_pursuit" in r.out
    sps = sent(d)
    assert len(sps) > 200
    for t, sp in sps:
        assert math.hypot(sp.north_m_s, sp.east_m_s) <= 0.9 + 1e-6, (t, sp)
        assert sp.down_m_s == 0.0, (t, sp)
    # before the switch: hold still at the yaw the pilot has
    before = [sp for t, sp in sps if t < 9.5]
    assert before and all(sp.north_m_s == 0 and sp.east_m_s == 0 for sp in before)
    assert all(abs(sp.yaw_deg - 80.0) < 1e-6 for sp in before)
    # after GO it moved toward the target (north-east), at the cap, at 5 m still
    assert d.pos_ne[0] > 5.0 and d.pos_ne[1] > 2.0, d.pos_ne
    assert abs(d.alt - 5.0) < 1e-6
    assert max(math.hypot(sp.north_m_s, sp.east_m_s) for t, sp in sps) > 0.85
    # the pilot lowered the switch: Position mode, program ended, still armed
    assert d.mode == "POSCTL" and d.armed is True
    assert max(t for t, _ in sps) < 42.5          # ended within 2.5 s of the switch
    assert "largest horizontal command sent" in r.out


def test_no_offboard_means_no_go_and_nothing_but_holds(monkeypatch, capsys, tmp_path,
                                                       dummy_server):
    """The switch is raised but the Pixhawk does not enter OFFBOARD (the board
    setting is missing): the program must not fire GO and must not move."""
    d = hovering_drone(dummy_server, offboard_switch=False, go_down=60.0)
    r, trig = run(monkeypatch, capsys, tmp_path, d,
                  ["--smoke-duration", "30"])
    assert r.rc == 0, r.out[-2000:]
    assert not trig.fired and "go_edge" not in r.out
    assert r.actions == []
    assert all(sp.north_m_s == 0 and sp.east_m_s == 0 and sp.down_m_s == 0
               for t, sp in sent(d))
    assert d.pos_ne == [0.0, 0.0]


def test_program_finished_but_switch_still_up_holds_still(monkeypatch, capsys, tmp_path,
                                                          dummy_server):
    """The engagement ends by its time limit with the switch still raised: the
    aircraft is still in OFFBOARD, so the stream goes on as a hold."""
    d = hovering_drone(dummy_server, go_down=70.0)
    r, trig = run(monkeypatch, capsys, tmp_path, d, ["--engage-max-s", "8"])
    assert r.rc == 0, r.out[-3000:]
    assert trig.fired and "BREAKOFF -> SAFE" in r.out
    assert "not OFFBOARD: ending the command stream" in r.out
    assert max(t for t, _ in sent(d)) < 72.0      # it stopped when he took it back
    late = [sp for t, sp in sent(d) if 45.0 < t < 69.0]
    assert len(late) > 100
    assert all(sp.north_m_s == 0 and sp.east_m_s == 0 and sp.down_m_s == 0 for sp in late)
    assert r.actions == []


def test_already_in_offboard_at_start_is_refused(monkeypatch, capsys, tmp_path, dummy_server):
    d = hovering_drone(dummy_server)
    d.mode, d.mode_script = "OFFBOARD", None
    r, _ = run(monkeypatch, capsys, tmp_path, d)
    assert r.rc == 1 and "ALREADY in OFFBOARD" in r.out
    assert sent(d) == []


@pytest.mark.parametrize("extra,needle", [
    (["--trigger", "rc"], "needs --trigger cue"),
    (["--cue-no-enable"], "needs the enable switch"),
    (["--source", "picamera"], "drop --source"),
    (["--handover-max-speed-ms", "5"], "outside 0 to 3"),
    (["--fly"], "its own mode"),
])
def test_argument_refusals(monkeypatch, capsys, tmp_path, extra, needle):
    r = drive(monkeypatch, capsys, tmp_path, HANDOVER + extra, drone=fm.FakeDrone())
    assert r.rc == 2 and needle in r.out, r.out[-800:]


def test_limits_unit():
    lim = rf.HandoverLimits(0.9)
    S = rf.Setpoint
    out = lim.limit(S(12.0, -5.0, -8.96, 30.0), rf.State.ENGAGE, 10.0)
    assert math.hypot(out.v_north, out.v_east) == pytest.approx(0.9)
    assert out.v_north / out.v_east == pytest.approx(12.0 / -5.0)
    assert out.v_down == 0.0 and out.yaw_deg == 30.0
    small = lim.limit(S(0.3, 0.2, 1.0, 5.0), rf.State.ENGAGE, 10.0)
    assert (small.v_north, small.v_east, small.v_down) == (0.3, 0.2, 0.0)
    for st in (rf.State.STANDBY, rf.State.SAFE):
        assert lim.limit(S(4.0, 4.0, -2.0, 54.4), st, 10.0).as_tuple() == (0.0, 0.0, 0.0, 10.0)
    nan = lim.limit(S(float("nan"), 1.0, 0.0, 30.0), rf.State.ENGAGE, 10.0)
    assert nan.as_tuple() == (0.0, 0.0, 0.0, 10.0)
    brk = lim.limit(S(3.0, 0.0, -4.0, 0.0), rf.State.BREAKOFF, 10.0)
    assert brk.as_tuple() == pytest.approx((0.9, 0.0, 0.0, 0.0))


def test_handover_cannot_reach_arm_takeoff_or_a_mode_command_structurally():
    attrs, names, seen = _closure_attrs("run_pilot_handover")
    assert "_control_loop" in seen
    assert (attrs & _ACTUATION) <= {"kill"}, attrs & _ACTUATION
    assert "_enter_offboard" not in seen and "run_mavsdk_mission" not in seen
    assert "_teardown_flight" not in seen
    # and it never calls offboard.start / offboard.stop itself
    import ast
    import inspect
    src = inspect.getsource(rf.run_pilot_handover)
    calls = {n.attr for n in ast.walk(ast.parse(src.lstrip())) if isinstance(n, ast.Attribute)}
    assert "start" not in calls and "stop" not in calls
