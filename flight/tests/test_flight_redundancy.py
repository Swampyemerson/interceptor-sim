"""The 2026-11-26 flight hardening, pinned by running it.

  * flight/deploy/link_redundancy.py -- two MAVLink links used as one
  * flight/deploy/board_overrides.py -- board settings changed for a flight
  * real_flight.py -- the battery rule (before GO / after, never between), the
    program's own GPS and battery checks before takeoff, the last guard on a
    non-finite command, and a control loop that a detector fault, a
    state-machine exception or one failed send no longer ends.

No PX4, no simulator, no hardware.
Run: .venv/bin/python -m pytest flight/tests/test_flight_redundancy.py -q
"""
import asyncio
import json
import math
import os
import subprocess
import sys
import time
import types

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_REPO_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fake_mavsdk as fm  # noqa: E402
import flight.deploy.real_flight as rf  # noqa: E402
from flight.deploy import board_overrides as bo  # noqa: E402
from flight.deploy import link_redundancy as lr  # noqa: E402
from test_real_flight_driver import (FLY, Run, fly_drone, rc, sm_cfg, sm_obs,  # noqa: E402
                                     smoke_cam, t_of)

BACKUP_URL = "udpout://127.0.0.1:14550"


def run(coro):
    return asyncio.new_event_loop().run_until_complete(coro)


# ---------------------------------------------------------------- prefer_primary

class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def scripted(items, clock):
    """An async stream that yields (value) at the given clock times."""
    async def gen():
        for at, val in items:
            while clock.t < at:
                await asyncio.sleep(0)
            yield val
    return gen


async def collect(merged, clock, until, step=0.01):
    out = []

    async def reader():
        async for v in merged:
            out.append((round(clock.t, 2), v))

    task = asyncio.ensure_future(reader())
    while clock.t < until:
        clock.t += step
        for _ in range(6):
            await asyncio.sleep(0)
    task.cancel()
    return out


def test_backup_samples_are_dropped_while_the_primary_is_alive():
    clock = Clock()
    prim = scripted([(0.1 * i, f"p{i}") for i in range(1, 11)], clock)
    back = scripted([(0.1 * i + 0.05, f"b{i}") for i in range(1, 11)], clock)
    st = lr.StreamStats("x")
    # (stop before the scripted primary runs out: an ENDED primary stream
    # hands over to the backup at once, which is the next test's subject)
    got = run(collect(lr.prefer_primary(prim, back, 0.3, st, clock), clock, 1.03))
    assert [v for _, v in got if v.startswith("b")] == []
    assert st.n_primary == 10 and st.n_backup == 0 and st.n_backup_dropped >= 9
    assert st.switches == 0


def test_backup_takes_over_after_the_quiet_time_and_hands_back():
    """The primary stops at 0.3 s and returns at 1.5 s. The backup's samples
    are used only once the primary has been silent for the quiet time, and are
    dropped again when it is back."""
    clock = Clock()
    prim = scripted([(0.1, "p1"), (0.2, "p2"), (0.3, "p3"),
                     (1.5, "p4"), (1.6, "p5")], clock)
    back = scripted([(0.05 + 0.1 * i, f"b{i}") for i in range(0, 17)], clock)
    said = []
    st = lr.StreamStats("attitude")
    got = run(collect(lr.prefer_primary(prim, back, 0.25, st, clock, said.append),
                      clock, 1.9))
    used_b = [(t, v) for t, v in got if v.startswith("b")]
    # before the primary ever spoke the backup is used (nothing better exists)
    assert used_b[0][1] == "b0"
    mid = [t for t, v in used_b if 0.3 < t < 1.5]
    assert mid and min(mid) >= 0.3 + 0.25 - 0.02      # not before the quiet time
    # none while the primary is back (its script ends at 1.6 s; an ENDED
    # primary hands over again, which is right)
    assert not [t for t, _ in used_b if 1.5 + 0.02 < t <= 1.6]
    assert st.switches >= 2
    assert any("using the BACKUP link" in s for s in said)
    assert any("primary link is back" in s for s in said)


def test_merged_stream_survives_one_source_dying_and_reports_it():
    clock = Clock()

    def dying():
        async def gen():
            yield "p1"
            raise RuntimeError("serial port gone")
        return gen()

    back = scripted([(0.1 * i, f"b{i}") for i in range(1, 8)], clock)
    said = []
    got = run(collect(lr.prefer_primary(dying, back, 0.2, None, clock, said.append),
                      clock, 1.0))
    assert [v for _, v in got][0] == "p1"
    assert [v for _, v in got if v.startswith("b")]      # the backup carried on
    assert any("primary link's stream died" in s for s in said)


# ---------------------------------------------------------------- RedundantSystem

class Proc:
    def __init__(self):
        self.pid, self.killed = 4242, False

    def poll(self):
        return 0 if self.killed else None

    def kill(self):
        self.killed = True


class StubSystem:
    def __init__(self, name):
        self.name = name
        self.sent = []
        self.fail = None          # None | "raise" | "hang"
        self._server_process = Proc()
        self.params = {}
        self.calls = []
        self.offboard = types.SimpleNamespace(
            set_velocity_ned=self._send, start=self._start, stop=self._stop)
        self.action = types.SimpleNamespace(land=self._land)
        self.param = types.SimpleNamespace(get_param_int=self._get,
                                           set_param_int=self._set)

    async def _send(self, sp):
        if self.fail == "hang":
            await asyncio.sleep(3600)
        if self.fail == "raise":
            raise RuntimeError("NO_SYSTEM")
        self.sent.append(sp)

    async def _start(self):
        self.calls.append("start")

    async def _stop(self):
        self.calls.append("stop")

    async def _land(self):
        if self.fail == "raise":
            raise RuntimeError("TIMEOUT")
        self.calls.append("land")

    async def _get(self, name):
        return self.params[name]

    async def _set(self, name, value):
        self.params[name] = value


def pair(**kw):
    said = []
    p, b = StubSystem("p"), StubSystem("b")
    return p, b, lr.RedundantSystem(p, b, say=said.append, **kw), said


def test_setpoints_go_out_on_both_links():
    p, b, rs, _ = pair()
    run(rs.offboard.set_velocity_ned("sp1"))
    assert p.sent == ["sp1"] and b.sent == ["sp1"]


def test_a_link_that_keeps_failing_is_ended_and_the_other_flies_on():
    """Three failed sends running end the primary: its server is KILLED (it
    would otherwise keep re-sending its last setpoint) and the backup carries
    the stream. (Mutant: never ending the link leaves the stale re-sender up.)"""
    p, b, rs, said = pair()
    p.fail = "raise"

    async def go():
        for i in range(5):
            await rs.offboard.set_velocity_ned(f"sp{i}")
    run(go())
    assert b.sent == [f"sp{i}" for i in range(5)]
    assert p._server_process.killed and not b._server_process.killed
    assert not rs.primary_alive() and rs.backup_alive()
    assert any("primary link is ENDED" in s for s in said)
    assert rs._server_processes  # still reports both for the watchdog/teardown


def test_a_hung_link_is_ended_at_once_and_does_not_stall_the_loop():
    p, b, rs, _ = pair(send_timeout_s=0.05)
    b.fail = "hang"
    t0 = time.monotonic()
    run(rs.offboard.set_velocity_ned("sp"))
    assert time.monotonic() - t0 < 0.5
    assert p.sent == ["sp"] and b._server_process.killed and not rs.backup_alive()


def test_no_link_left_raises_linkdown():
    p, b, rs, _ = pair(send_fails_to_end=1)
    p.fail = b.fail = "raise"
    with pytest.raises(lr.LinkDown):
        run(rs.offboard.set_velocity_ned("sp"))
    with pytest.raises(lr.LinkDown):
        run(rs.offboard.set_velocity_ned("sp"))


def test_commands_fall_back_to_the_backup_link():
    p, b, rs, said = pair()
    p.fail = "raise"
    run(rs.action.land())
    assert b.calls == ["land"] and any("failed on the primary" in s for s in said)


def test_offboard_stop_reaches_every_live_link():
    p, b, rs, _ = pair()
    run(rs.offboard.stop())
    assert p.calls == ["stop"] and b.calls == ["stop"]


# ---------------------------------------------------------------- the watchdog, two servers

def test_the_watchdog_kills_every_server(monkeypatch):
    """With a backup link there are two servers re-sending setpoints. A stalled
    loop must end BOTH. (Mutant: one --server-pid -> the second survives.)"""
    procs = [subprocess.Popen(["sleep", "600"]) for _ in range(2)]
    try:
        sup = rf.LoopSupervisor(0.3, match="sleep")
        assert sup.start([p.pid for p in procs])
        sup.watch()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and any(p.poll() is None for p in procs):
            time.sleep(0.05)
        assert all(p.poll() is not None for p in procs)
        sup.close()
    finally:
        for p in procs:
            if p.poll() is None:
                p.kill()
            p.wait()


# ---------------------------------------------------------------- battery rule (state machine)

GO = rf.TriggerState(go=True, link_ok=True, age_s=0.05, raw_us=1900)


def test_low_battery_in_standby_lands_after_the_hold_time():
    sm = rf.RealFlightSM(sm_cfg(battery_low_frac=0.07, battery_low_hold_s=3.0))
    sm.step(sm_obs(0.0, battery_frac=0.50))
    sm.step(sm_obs(1.0, battery_frac=0.05))
    assert sm.state == rf.State.STANDBY          # a dip, not yet low for 3 s
    sm.step(sm_obs(2.0, battery_frac=0.50))      # recovers: the clock resets
    sm.step(sm_obs(3.0, battery_frac=0.05))
    d = sm.step(sm_obs(5.9, battery_frac=0.05))
    assert d.state == rf.State.STANDBY
    d = sm.step(sm_obs(6.1, battery_frac=0.05))
    assert d.state == rf.State.SAFE and sm.safe_reason == "battery_low_before_go"
    assert d.land_requested


def test_low_battery_after_go_is_ignored_and_said_once():
    """THE RULING: once GO is given the battery cannot end the intercept."""
    sm = rf.RealFlightSM(sm_cfg(pursuit_mode=True, battery_low_frac=0.07,
                                battery_low_hold_s=1.0))
    sm.step(sm_obs(0.0, battery_frac=0.6))
    sm.step(sm_obs(0.05, trigger=GO, battery_frac=0.6))
    assert sm.state == rf.State.ENGAGE
    events = []
    for i in range(80):
        d = sm.step(sm_obs(0.1 + 0.05 * i, battery_frac=0.0, det_new=True,
                           det_range_m=8.0,
                           det_box_xywh=(600.0, 440.0, 80.0, 80.0)))
        events += d.events
    assert sm.state == rf.State.ENGAGE
    assert sum("IGNORED after GO" in e for e in events) == 1


def test_low_battery_ends_the_after_miss_hover_with_a_landing():
    sm = rf.RealFlightSM(sm_cfg(battery_low_frac=0.07, battery_low_hold_s=1.0,
                                miss_safe_behavior="hover"))
    sm.step(sm_obs(0.0, battery_frac=0.5))
    sm.state, sm.t_state, sm.safe_reason = rf.State.SAFE, 0.0, "pursuit_miss"
    assert sm.effective_safe_behavior() == "hover"
    d = sm.step(sm_obs(5.0, battery_frac=0.5))
    assert not d.land_requested and not d.terminated      # hovering
    sm.step(sm_obs(6.0, battery_frac=0.03))
    d = sm.step(sm_obs(7.1, battery_frac=0.03))
    assert d.land_requested and d.terminated


def test_battery_rule_is_inert_without_a_threshold_or_a_reading():
    sm = rf.RealFlightSM(sm_cfg())                         # no threshold
    for i in range(10):
        sm.step(sm_obs(float(i), battery_frac=0.0))
    assert sm.state == rf.State.STANDBY
    sm2 = rf.RealFlightSM(sm_cfg(battery_low_frac=0.07, battery_low_hold_s=0.0))
    for i in range(10):
        sm2.step(sm_obs(float(i)))                         # no reading
    assert sm2.state == rf.State.STANDBY


def test_a_stale_or_unknown_battery_reading_is_no_reading():
    st = {"t_batt": 100.0, "batt_rem": 0.03}
    assert rf.fresh_battery_frac(st, 102.0) == 0.03
    assert rf.fresh_battery_frac(st, 110.0) is None
    assert rf.fresh_battery_frac({"t_batt": None, "batt_rem": None}, 1.0) is None


# ---------------------------------------------------------------- last guard on the command

class NanTerminal:
    """A terminal that returns a non-finite command (the review's finding: a
    NaN own velocity made every later setpoint NaN, with nothing to stop it)."""

    def step(self, *a, **k):
        from flight.deploy.seeker_loop import Setpoint
        return Setpoint(float("nan"), 1.0, 0.0, 40.0)


def test_a_non_finite_command_never_leaves_the_state_machine():
    sm = rf.RealFlightSM(sm_cfg(pursuit_mode=True))
    sm.step(sm_obs(0.0))
    sm.step(sm_obs(0.05, trigger=GO))
    assert sm.state == rf.State.ENGAGE
    real = sm._step_engage
    from flight.deploy.seeker_loop import Setpoint
    sm._step_engage = lambda obs, events: (Setpoint(float("nan"), 1.0, 0.0, 40.0),
                                           None)
    d = sm.step(sm_obs(0.10))
    assert all(math.isfinite(x) for x in d.setpoint.as_tuple())
    assert d.state == rf.State.SAFE and sm.safe_reason == "nonfinite_setpoint"
    sm._step_engage = real


def test_non_finite_detections_are_stopped_at_the_door():
    ok = types.SimpleNamespace(range_m=8.0, box_xywh=(1.0, 2.0, 3.0, 4.0))
    assert rf._det_is_finite(ok)
    assert not rf._det_is_finite(types.SimpleNamespace(
        range_m=float("nan"), box_xywh=(1.0, 2.0, 3.0, 4.0)))
    assert not rf._det_is_finite(types.SimpleNamespace(
        range_m=8.0, box_xywh=(1.0, float("inf"), 3.0, 4.0)))
    assert not rf._det_is_finite(types.SimpleNamespace(
        range_m=8.0, box_xywh=(1.0, 2.0, 3.0)))
    assert rf._det_is_finite(types.SimpleNamespace(range_m=None, box_xywh=None))


# ---------------------------------------------------------------- pre-arm checks

LIM = {"BAT_LOW_THR": 0.15, "BAT_CRIT_THR": 0.07, "EKF2_REQ_NSATS": 6,
       "EKF2_REQ_EPH": 3.0, "EKF2_REQ_SACC": 1.0}
GOOD = {"sats": 14, "fix": 3, "hacc": 0.8, "sacc": 0.2}


def test_gps_prearm_check_applies_the_boards_own_thresholds():
    assert rf.gps_prearm_refusals([GOOD] * 5, LIM) == []
    assert rf.gps_prearm_refusals([], LIM)                       # no reading
    assert rf.gps_prearm_refusals([GOOD], {})                    # no threshold
    one_bad = [GOOD, dict(GOOD, sats=5), GOOD]
    assert any("5 satellites" in r for r in rf.gps_prearm_refusals(one_bad, LIM))
    assert any("speed accuracy" in r for r in
               rf.gps_prearm_refusals([dict(GOOD, sacc=1.4)], LIM))
    assert any("3D" in r for r in rf.gps_prearm_refusals([dict(GOOD, fix=2)], LIM))
    assert rf.gps_prearm_refusals([dict(GOOD, hacc=None)], LIM)


def test_battery_prearm_check_fails_closed():
    assert rf.battery_prearm_refusal(0.8, LIM) is None
    assert "below" in rf.battery_prearm_refusal(0.10, LIM)
    assert "no battery reading" in rf.battery_prearm_refusal(None, LIM)
    assert "not read" in rf.battery_prearm_refusal(0.8, {})


# ---------------------------------------------------------------- board overrides

def test_only_these_three_settings_can_ever_be_written():
    """The list is the whole write surface of the flight software. Changing it
    is a decision, so it fails a test."""
    assert [(n, v) for n, v, _ in bo.OVERRIDES] == [
        ("COM_LOW_BAT_ACT", 0), ("COM_FLTT_LOW_ACT", 0), ("EKF2_GPS_CHECK", 0)]


def board():
    s = StubSystem("fc")
    s.params = {"COM_LOW_BAT_ACT": 3, "COM_FLTT_LOW_ACT": 3, "EKF2_GPS_CHECK": 1023}
    return s


def test_apply_saves_the_originals_first_then_restore_puts_them_back(tmp_path):
    path = str(tmp_path / "state.json")
    fc, said = board(), []
    assert run(bo.apply(fc, False, state_path=path, say=said.append))
    assert fc.params == {"COM_LOW_BAT_ACT": 0, "COM_FLTT_LOW_ACT": 0,
                         "EKF2_GPS_CHECK": 0}
    saved = json.load(open(path))
    assert saved["originals"] == {"COM_LOW_BAT_ACT": 3, "COM_FLTT_LOW_ACT": 3,
                                  "EKF2_GPS_CHECK": 1023}
    assert bo.pending(path) is not None
    assert run(bo.restore(fc, False, state_path=path, say=said.append))
    assert fc.params == {"COM_LOW_BAT_ACT": 3, "COM_FLTT_LOW_ACT": 3,
                         "EKF2_GPS_CHECK": 1023}
    assert bo.pending(path) is None and not os.path.exists(path)


def test_nothing_is_written_on_an_armed_aircraft(tmp_path):
    path = str(tmp_path / "state.json")
    fc = board()
    assert not run(bo.apply(fc, True, state_path=path, say=lambda s: None))
    assert not run(bo.apply(fc, None, state_path=path, say=lambda s: None))
    assert fc.params["COM_LOW_BAT_ACT"] == 3 and not os.path.exists(path)
    # and a restore owed in the air stays owed
    assert run(bo.apply(fc, False, state_path=path, say=lambda s: None))
    assert not run(bo.restore(fc, True, state_path=path, say=lambda s: None))
    assert fc.params["COM_LOW_BAT_ACT"] == 0 and bo.pending(path) is not None


def test_a_write_that_does_not_land_refuses_and_restores(tmp_path):
    """The read-back uses the other link. Here the board silently drops one
    write: apply must say no and leave the board as it found it."""
    path = str(tmp_path / "state.json")
    fc = board()
    real_set = fc._set

    async def lossy(name, value):
        if name == "EKF2_GPS_CHECK" and value == 0:
            return
        await real_set(name, value)

    fc.param.set_param_int = lossy
    said = []
    assert not run(bo.apply(fc, False, state_path=path, say=said.append))
    assert fc.params == {"COM_LOW_BAT_ACT": 3, "COM_FLTT_LOW_ACT": 3,
                         "EKF2_GPS_CHECK": 1023}
    assert bo.pending(path) is None
    assert any("EKF2_GPS_CHECK" in s and "read back 1023" in s for s in said)


def test_a_run_that_died_is_restored_by_the_next_one(tmp_path):
    path = str(tmp_path / "state.json")
    fc = board()
    assert run(bo.apply(fc, False, state_path=path, say=lambda s: None))
    # ... the program dies here; the board keeps the flight values ...
    said = []
    assert run(bo.apply(fc, False, state_path=path, say=said.append))
    assert any("earlier run left the flight values" in s for s in said)
    assert json.load(open(path))["originals"]["EKF2_GPS_CHECK"] == 1023


def test_an_unreadable_state_file_counts_as_pending(tmp_path):
    path = str(tmp_path / "state.json")
    open(path, "w").write("{not json")
    assert bo.pending(path)["unreadable"]
    assert not run(bo.restore(board(), False, state_path=path, say=lambda s: None))


# ---------------------------------------------------------------- the driver, end to end

def drive2(monkeypatch, capsys, tmp_path, argv, primary, backup=None, detector=None):
    """`real_flight.main(argv)` on the virtual clock with one or two fake links."""
    made = []

    def system(*a, **k):
        d = primary if not made else backup
        made.append(k)
        return d

    fm.install(monkeypatch, primary)
    monkeypatch.setitem(sys.modules["mavsdk"].__dict__, "System", system)
    monkeypatch.setattr(asyncio, "run",
                        lambda coro: fm.run_virtual(lambda: coro, rf, monkeypatch))
    if detector is not None:
        monkeypatch.setattr(rf, "build_source_detector",
                            lambda args, gcfg, cam: (detector, None))
    argv = list(argv) + ["--log-csv", str(tmp_path / "ticks.csv"),
                         "--board-state-path", str(tmp_path / "board.json")]
    try:
        code = rf.main(argv)
    except SystemExit as e:
        code = e.code
    out = capsys.readouterr()
    r = Run(code, primary, out.out + out.err)
    r.made = made
    return r


class Mirror(fm.FakeDrone):
    """The SAME vehicle seen through a second link: its telemetry reads the
    primary fake's state, and a setpoint sent here reaches that vehicle."""

    def __init__(self, src, server):
        super().__init__()
        self.src = src
        self._server_process = server
        s = src
        self.telemetry = types.SimpleNamespace(
            attitude_quaternion=lambda: self._stream("quat", 0.02, s._quat),
            attitude_euler=lambda: self._stream("euler", 0.02, s._euler),
            position=lambda: self._stream("pos", 0.02, s._pos),
            velocity_ned=lambda: self._stream("vel", 0.033, s._vel),
            armed=lambda: self._stream("armed", 0.1, lambda: s.armed),
            landed_state=lambda: self._stream("landed", 0.2, lambda: s.landed),
            flight_mode=lambda: self._stream("mode", 0.1, lambda: fm._Mode(s.mode)),
            battery=lambda: self._stream("battery", 0.5, s._battery),
            gps_info=s.telemetry.gps_info, raw_gps=s.telemetry.raw_gps,
            health=s._health)
        self.param = s.param
        self.mavlink_direct = types.SimpleNamespace(message=s._mav_message)

    async def connect(self, system_address=None):
        self.connect_url = system_address
        self.t0 = self.now()
        self._rec("connect", system_address)

    async def _set_velocity_ned(self, sp):
        await super()._set_velocity_ned(sp)
        self.src._sp = self._sp           # PX4 flies setpoints from either port


@pytest.fixture
def two_servers(monkeypatch):
    procs = [subprocess.Popen(["sleep", "600"]) for _ in range(2)]
    monkeypatch.setattr(rf, "_SERVER_MATCH", "sleep")
    yield procs
    for p in procs:
        if p.poll() is None:
            p.kill()
        p.wait()


PRIMARY_STREAMS = ("quat", "euler", "pos", "vel", "mode", "armed", "landed",
                   "battery")


def test_fly_suspends_the_board_failsafes_and_restores_them(monkeypatch, capsys,
                                                            tmp_path, two_servers):
    d = fly_drone(two_servers[0])
    r = drive2(monkeypatch, capsys, tmp_path, FLY, d, detector=smoke_cam())
    assert r.rc == 0, r.out[-3000:]
    sets = [(t, a) for t, n, a in d.calls if n == "param.set"]
    assert [a for _, a in sets[:3]] == [("COM_LOW_BAT_ACT", 0),
                                        ("COM_FLTT_LOW_ACT", 0),
                                        ("EKF2_GPS_CHECK", 0)]
    assert max(t for t, _ in sets[:3]) <= t_of(d, "action.arm")    # before arming
    assert [a for _, a in sets[3:]] == [("COM_LOW_BAT_ACT", 3),
                                        ("COM_FLTT_LOW_ACT", 3),
                                        ("EKF2_GPS_CHECK", 1023)]
    assert min(t for t, _ in sets[3:]) >= t_of(d, "action.land")   # after landing
    assert d.params["COM_LOW_BAT_ACT"] == 3 and d.params["EKF2_GPS_CHECK"] == 1023
    assert not os.path.exists(tmp_path / "board.json")
    assert "COM_LOW_BAT_ACT -> 0" in r.out and "board settings restored" in r.out
    assert not os.path.exists(str(tmp_path / "ticks.csv") + ".live")


def test_fly_with_board_failsafes_kept_writes_nothing(monkeypatch, capsys, tmp_path,
                                                      two_servers):
    d = fly_drone(two_servers[0])
    r = drive2(monkeypatch, capsys, tmp_path, FLY + ["--board-failsafes", "keep"],
               d, detector=smoke_cam())
    assert r.rc == 0, r.out[-3000:]
    assert d.count("param.set") == 0
    assert "board failsafes: KEPT" in r.out


@pytest.mark.parametrize("attr,value,text", [
    ("gps_sats", 4, "4 satellites"),
    ("gps_sacc", 1.6, "speed accuracy 1.60"),
    ("battery_pct", 10.0, "battery at 10%"),
    ("battery_pct", None, "no battery reading"),
])
def test_fly_does_not_arm_on_weak_gps_or_a_low_pack(monkeypatch, capsys, tmp_path,
                                                    two_servers, attr, value, text):
    d = fly_drone(two_servers[0])
    setattr(d, attr, value)
    r = drive2(monkeypatch, capsys, tmp_path, FLY, d, detector=smoke_cam())
    assert r.rc == 1 and text in r.out, r.out[-2500:]
    assert "action.arm" not in d.names() and d.count("param.set") == 0


def test_fly_does_not_arm_when_a_board_setting_will_not_take(monkeypatch, capsys,
                                                             tmp_path, two_servers):
    d = fly_drone(two_servers[0])
    d.param_set_ignored = {"EKF2_GPS_CHECK"}
    r = drive2(monkeypatch, capsys, tmp_path, FLY, d, detector=smoke_cam())
    assert r.rc == 1 and "could not be confirmed" in r.out
    assert "action.arm" not in d.names()


def test_a_dead_primary_link_no_longer_ends_the_flight(monkeypatch, capsys, tmp_path,
                                                       two_servers):
    """THE POINT OF THE BACKUP LINK. The primary's telemetry freezes two
    seconds after GO. Alone, that is FAILSAFE 9 (own-state stale -> SAFE).
    With the backup, the flight goes on to its normal end."""
    def freeze(d):
        d.freeze_after = {k: 12.0 for k in PRIMARY_STREAMS}

    # control: one link
    d1 = fly_drone(two_servers[0], permit_at=3.0, go_at=10.0)
    freeze(d1)
    r1 = drive2(monkeypatch, capsys, tmp_path, FLY, d1, detector=smoke_cam())
    assert "own_state_stale" in r1.out and "BREAKOFF" not in r1.out.split(
        "states visited:")[1].splitlines()[0]

    # two links
    d2 = fly_drone(two_servers[0], permit_at=3.0, go_at=10.0)
    freeze(d2)
    b2 = Mirror(d2, two_servers[1])
    r2 = drive2(monkeypatch, capsys, tmp_path, FLY + ["--mavsdk-backup-url",
                BACKUP_URL], d2, b2, detector=smoke_cam())
    assert r2.rc == 0, r2.out[-3500:]
    assert "own_state_stale" not in r2.out
    assert "states visited: STANDBY -> ENGAGE -> BREAKOFF -> SAFE" in r2.out
    assert "using the BACKUP link" in r2.out
    assert b2.count("offboard.set_velocity_ned") > 100         # both carried setpoints
    assert d2.count("offboard.set_velocity_ned") > 100
    assert r2.made[0].get("port") != r2.made[1].get("port")    # two servers
    assert "[link] telemetry.attitude_quaternion: fell back" in r2.out


def test_a_primary_that_refuses_setpoints_is_ended_and_the_backup_flies(
        monkeypatch, capsys, tmp_path, two_servers):
    d = fly_drone(two_servers[0], permit_at=3.0, go_at=10.0)
    b = Mirror(d, two_servers[1])
    real = d._set_velocity_ned

    async def dying(sp):
        if d.t() >= 12.0:
            raise RuntimeError("CONNECTION_ERROR")
        await real(sp)

    d.offboard.set_velocity_ned = dying
    d.freeze_after = {k: 12.0 for k in PRIMARY_STREAMS}
    r = drive2(monkeypatch, capsys, tmp_path, FLY + ["--mavsdk-backup-url",
               BACKUP_URL], d, b, detector=smoke_cam())
    assert r.rc == 0, r.out[-3500:]
    assert "the primary link is ENDED" in r.out
    assert two_servers[0].poll() is not None        # its server was killed
    # The backup's server carried the flight to its end (rc 0 above); the
    # program ends it last, when the run is over.
    assert r.out.index("the primary link is ENDED") < r.out.index(
        "server process(es) ended; the vehicle link is free")
    assert "states visited: STANDBY -> ENGAGE -> BREAKOFF -> SAFE" in r.out
    assert b.names().count("action.land") == 1      # the landing went out on the backup


def test_a_backup_that_was_asked_for_and_does_not_connect_is_a_refusal(
        monkeypatch, capsys, tmp_path, two_servers):
    d = fly_drone(two_servers[0])
    b = Mirror(d, two_servers[1])

    async def never(system_address=None):
        await asyncio.sleep(3600)

    b.connect = never
    r = drive2(monkeypatch, capsys, tmp_path, FLY + ["--mavsdk-backup-url",
               BACKUP_URL], d, b, detector=smoke_cam())
    assert r.rc == 1 and "the backup link" in r.out and "did not connect" in r.out
    assert "action.arm" not in d.names()


class FaultyCam:
    """A detector that raises on some frames (a cv2 / PnP error)."""

    def __init__(self):
        self.inner, self.n = smoke_cam(), 0

    def detect(self, frame, t):
        self.n += 1
        if self.n % 7 == 3:
            raise ValueError("solvePnP failed")
        return self.inner.detect(frame, t)


def test_a_detector_fault_is_a_missed_detection_not_the_end_of_the_flight(
        monkeypatch, capsys, tmp_path, two_servers):
    """It used to escape the control loop: the aircraft landed where it was
    and the tick log was lost."""
    d = fly_drone(two_servers[0])
    r = drive2(monkeypatch, capsys, tmp_path, FLY, d, detector=FaultyCam())
    assert r.rc == 0, r.out[-3000:]
    assert "[detector] FAULT #1" in r.out and "the flight goes on" in r.out
    assert "states visited: STANDBY -> ENGAGE" in r.out
    assert os.path.getsize(tmp_path / "ticks.csv") > 1000


def test_a_state_machine_exception_holds_the_command_then_hands_over(
        monkeypatch, capsys, tmp_path, two_servers):
    """One raise: the last command is held and the flight goes on. A run of
    them: the loop stops and teardown lands, with the tick log on the card."""
    d = fly_drone(two_servers[0])
    real_step = rf.RealFlightSM.step
    n = {"k": 0}

    def flaky(self, obs):
        if self.state == rf.State.ENGAGE:
            n["k"] += 1
            if n["k"] == 5 or n["k"] >= 40:
                raise TypeError("unsupported operand: NoneType")
        return real_step(self, obs)

    monkeypatch.setattr(rf.RealFlightSM, "step", flaky)
    r = drive2(monkeypatch, capsys, tmp_path, FLY, d, detector=smoke_cam())
    assert r.out.count("FAULT step_raised") >= 2
    assert "FAULT step_raised #1" in r.out and "the state machine raised" not in \
        r.out.split("FAULT step_raised #2")[0]
    assert r.rc == 1 and d.names().count("action.land") == 1
    rows = open(tmp_path / "ticks.csv").read().splitlines()
    assert len(rows) > 50


def test_the_tick_log_is_on_the_card_while_the_flight_runs(tmp_path):
    """A crash used to lose every row. The live copy holds all but the last
    second."""
    final = str(tmp_path / "ticks.csv")
    tl = rf.TickLog(final, flush_s=0.05)
    for i in range(200):
        tl.offer(f"{i},x\n")
    time.sleep(0.3)
    rows = open(final + ".live").read().splitlines()   # read while still open
    assert rows[0] == rf._CSV_HEADER.strip() and len(rows) >= 150
    tl.close(remove=False)
    assert len(open(final + ".live").read().splitlines()) == 201
    tl2 = rf.TickLog(final)
    tl2.close(remove=True)
    assert not os.path.exists(final + ".live")


# ---------------------------------------------------------------- dead camera

def test_a_dead_camera_ends_the_engagement_and_an_empty_sky_does_not():
    """Total loss of the only sensor ends it; frames with no target do not."""
    def engaged():
        sm = rf.RealFlightSM(sm_cfg(pursuit_mode=True, camera_dead_s=3.0))
        sm.step(sm_obs(0.0, camera_age_s=0.02))
        sm.step(sm_obs(0.05, trigger=GO, camera_age_s=0.02))
        assert sm.state == rf.State.ENGAGE
        return sm

    sm = engaged()                       # frames keep coming, nothing in them
    for i in range(200):
        sm.step(sm_obs(0.1 + 0.05 * i, camera_age_s=0.02))
    assert sm.state == rf.State.ENGAGE

    sm = engaged()
    sm.step(sm_obs(1.0, camera_age_s=2.9))
    assert sm.state == rf.State.ENGAGE
    d = sm.step(sm_obs(1.2, camera_age_s=3.1))
    assert d.state == rf.State.SAFE and sm.safe_reason == "camera_dead"

    sm = engaged()                       # no live feed to ask: inactive
    sm.step(sm_obs(1.0))
    assert sm.state == rf.State.ENGAGE
    off = rf.RealFlightSM(sm_cfg(pursuit_mode=True))       # default: off
    off.step(sm_obs(0.0))
    off.step(sm_obs(0.05, trigger=GO))
    off.step(sm_obs(1.0, camera_age_s=60.0))
    assert off.state == rf.State.ENGAGE


def test_the_camera_feed_reports_the_age_of_its_newest_frame():
    from flight.deploy.frame_feed import FrameFeed
    clock = Clock()

    class Src:
        def frames(self):
            return iter(())

    feed = FrameFeed(Src(), threaded=True, clock=clock)
    assert feed.frame_age_s() is None
    feed._slot = ("frame", 10.0, 1)
    clock.t = 12.5
    assert feed.frame_age_s() == 2.5
    assert rf._camera_age(types.SimpleNamespace(feed=feed)) == 2.5
    assert rf._camera_age(object()) is None


def test_cue_records_with_non_finite_numbers_are_refused_not_fatal():
    """json.loads accepts NaN and Infinity; a huge integer overflowed float()
    and killed the receive thread (review 2026-11-26)."""
    from flight.cue_link import CueReceiver
    rx = CueReceiver(bind="127.0.0.1", port=0)
    try:
        good = b'{"lat": 40.0, "lon": -105.0, "alt_m_msl": 1650.0, "vn": 1.0, "ve": 2.0, "vd": 0.0}\n'
        assert rx.feed(good, 1.0) == 1
        for bad in (b'{"lat": 40.0, "lon": -105.0, "alt_m_msl": NaN}\n',
                    b'{"lat": 40.0, "lon": -105.0, "alt_m_msl": 1.0, "vn": Infinity}\n',
                    b'{"lat": 40.0, "lon": -105.0, "alt_m_msl": 1' + b"0" * 400 + b'}\n'):
            assert rx.feed(bad, 2.0) == 0
    finally:
        rx.close()


# ---------------------------------------------------------------- upside-down camera

def test_a_rotated_image_needs_a_calibration_captured_rotated():
    """The nose camera is upside down. Read unrotated, or rotated against a
    calibration captured unrotated, every bearing is mirrored and the aircraft
    steers away from the target; a bench decode cannot show it."""
    from flight.camera import CameraModel
    from flight.deploy import frame_feed as ff
    base = dict(fx=900.0, fy=900.0, cx=650.0, cy=390.0,
                resolution={"width": 1280, "height": 800}, source="checkerboard")
    plain = CameraModel.from_dict(base)
    flipped = CameraModel.from_dict(dict(base, image_flip_deg=180))
    assert plain.image_flip_deg is None and flipped.image_flip_deg == 180
    assert ff.flip_refusal(plain, False) is None
    assert ff.flip_refusal(flipped, True) is None
    assert "steer AWAY" in ff.flip_refusal(plain, True)
    assert "steer AWAY" in ff.flip_refusal(flipped, False)
    with pytest.raises(ff.RefusedSource):
        ff.open_source("picamera", plain, 60.0, flip180=True)


def test_fly_refuses_until_the_camera_mount_is_stated(monkeypatch, capsys, tmp_path,
                                                      two_servers):
    argv = [a for a in FLY if a not in ("--camera-mount", "upright")]
    d = fly_drone(two_servers[0])
    r = drive2(monkeypatch, capsys, tmp_path, argv, d, detector=smoke_cam())
    assert r.rc != 0 and "--camera-mount" in r.out
    assert "action.arm" not in d.names()
