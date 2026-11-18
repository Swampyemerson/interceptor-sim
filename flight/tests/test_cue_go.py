"""The cue is the GO command, timestamped (builder rulings 2026-11-18).

What is pinned here, each with the behaviour that fails when the new piece is
switched off or bypassed:
  * timestamps: with UTC on both ends each record's link delay is MEASURED and
    removed; without it the belief lags by the delay; an implausible clock
    refuses instead of guessing;
  * `CueTrigger`: GO fires only after the arm gate, the enable switch and a
    fresh quality-gated fit; the belief is latched once and handed over once;
  * the state machine writes that belief into the pursuit terminal at the GO
    edge, exactly once, and goes SAFE rather than fly on a placeholder;
  * the lost-target clock switches and the one flying preset;
  * the honesty audit sees a second belief write and a second link latch.

Offline: no PX4, no hardware. One test (`test_cue_go_over_real_udp...`) runs in
real time over loopback UDP and takes about 12 s.

Run: .venv/bin/python -m pytest flight/tests/test_cue_go.py -q
"""
import asyncio
import math
import os
import socket
import sys
import threading
import time

import numpy as np
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_REPO_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fake_mavsdk as fm  # noqa: E402
import flight.deploy.real_flight as rf  # noqa: E402
from flight.cue_link import (CueLinkError, CueReceiver, MonoToUtc,  # noqa: E402
                             SystemClockUtc, encode_record)
from flight.cue_relay import (BootToUtc, CueQualityError, CueRecord,  # noqa: E402
                              lla_to_ned)
from test_real_flight_driver import (GO_CH, PERMIT_CH, SERIAL, drive,  # noqa: E402,F401
                                     dummy_server, rc, sm_cfg, sm_obs, smoke_cam)

OWN = (40.0, -105.0, 1650.0)
_R = 6378137.0
UTC0 = 1.8e9            # "UTC" at local clock zero, for the fake clock sources
P0 = (200.0, -300.0)    # target north, east at t = 0 (m, about OWN)
V = (0.0, 12.0)         # target velocity north, east (m/s)
UP = 10.0               # target height above OWN (m)


def lla(n, e, up=UP, own=OWN):
    return (own[0] + math.degrees(n / _R),
            own[1] + math.degrees(e / (_R * math.cos(math.radians(own[0])))),
            own[2] + up)


def truth(t, p0=P0, v=V):
    return (p0[0] + v[0] * t, p0[1] + v[1] * t)


class FixedUtc:
    """A UTC source that is exactly `offset` ahead of the local clock."""
    source = "test"
    ok = True

    def __init__(self, offset=UTC0):
        self.offset = offset

    def utc_now(self, mono=None):
        return None if self.offset is None else mono + self.offset


def payload(t_sample, *, t_src=True, t_utc=True, utc_err=0.0, v=V, p0=P0):
    n, e = truth(t_sample, p0, v)
    la, lo, al = lla(n, e)
    return encode_record(CueRecord(0.0, la, lo, al, v[0], v[1], 0.0),
                         t_src=(t_sample if t_src else None),
                         t_utc=(UTC0 + t_sample + utc_err if t_utc else None))


def feed(rx, t0, t1, *, rate=5.0, delay=(0.05, 0.05), seed=0, **kw):
    """Samples taken at t in [t0, t1), each delivered `delay` later."""
    rng = np.random.default_rng(seed)
    k = 0
    while t0 + k / rate < t1 - 1e-9:
        t = t0 + k / rate
        rx.feed(payload(t, **kw), t_rx=t + rng.uniform(*delay))
        k += 1


# ============================================================ timestamps

def test_utc_stamps_measure_the_link_delay_and_remove_it():
    """A slow, jittery link (0.3 to 0.8 s). With UTC on both ends the belief at
    the latch is the truth at the latch; the measured delay is reported."""
    rx = CueReceiver(clock=lambda: 0.0, utc=FixedUtc(), allow_unsynced=False)
    feed(rx, 0.0, 20.0, delay=(0.3, 0.8), seed=4)
    go = 20.9
    sol = rx.latch(*OWN, now_t=go, max_staleness_s=2.0)
    n, e = truth(go)
    assert abs(sol.belief_r0_ned[0] - n) < 0.05 and abs(sol.belief_r0_ned[1] - e) < 0.05
    info = rx.latch_info
    assert info["time_base"] == "utc"
    assert 0.29 < info["delay_min_s"] < 0.45 and 0.65 < info["delay_max_s"] < 0.81


def test_without_a_shared_clock_the_same_link_leaves_the_delay_in_the_belief():
    """The mutant of the test above: same records, no UTC source. The belief
    then lags the truth by the link's minimum delay (0.3 s x 12 m/s)."""
    rx = CueReceiver(clock=lambda: 0.0, utc=None, allow_unsynced=True)
    feed(rx, 0.0, 20.0, delay=(0.3, 0.8), seed=4)
    go = 20.9
    sol = rx.latch(*OWN, now_t=go, max_staleness_s=2.0)
    lag_m = truth(go)[1] - sol.belief_r0_ned[1]
    assert 3.3 < lag_m < 4.2, lag_m
    assert rx.latch_info["time_base"] == "min_delay"


def test_no_shared_clock_refuses_go_unless_unsynced_is_allowed():
    rx = CueReceiver(clock=lambda: 0.0, utc=None, allow_unsynced=False)
    feed(rx, 0.0, 10.0)
    with pytest.raises(CueLinkError, match="no shared time base"):
        rx.peek(*OWN, now_t=10.0)
    rx2 = CueReceiver(clock=lambda: 0.0, utc=FixedUtc(), allow_unsynced=False)
    feed(rx2, 0.0, 10.0, t_utc=False)                 # sender has no GPS time
    with pytest.raises(CueLinkError, match="carries no t_utc"):
        rx2.peek(*OWN, now_t=10.0)


@pytest.mark.parametrize("kw,needle", [
    (dict(utc_err=+0.5), "from the future"),          # sender clock ahead
    (dict(utc_err=-6.0), "old on arrival"),           # sender clock far behind
])
def test_an_implausible_clock_refuses(kw, needle):
    rx = CueReceiver(clock=lambda: 0.0, utc=FixedUtc(), allow_unsynced=False)
    feed(rx, 0.0, 10.0, **kw)
    with pytest.raises(CueLinkError, match=needle):
        rx.peek(*OWN, now_t=10.0)


def test_a_sender_clock_mapping_that_jumps_mid_window_refuses():
    rx = CueReceiver(clock=lambda: 0.0, utc=FixedUtc(), allow_unsynced=False)
    feed(rx, 0.0, 7.0)
    feed(rx, 7.0, 10.0, utc_err=-0.4)                 # the mapping moved 0.4 s
    with pytest.raises(CueLinkError, match="time mapping jumped"):
        rx.peek(*OWN, now_t=10.0)


def test_a_receiver_utc_source_with_no_time_refuses():
    utc = FixedUtc(offset=None)
    rx = CueReceiver(clock=lambda: 0.0, utc=utc, allow_unsynced=False)
    feed(rx, 0.0, 10.0)
    with pytest.raises(CueLinkError, match="no valid time when a record arrived"):
        rx.peek(*OWN, now_t=10.0)


def test_an_implausible_clock_falls_back_only_when_allowed_and_says_so():
    rx = CueReceiver(clock=lambda: 0.0, utc=FixedUtc(), allow_unsynced=True)
    feed(rx, 0.0, 10.0, utc_err=+0.5)
    rx.latch(*OWN, now_t=10.0)
    assert rx.latch_info["time_base"] == "min_delay"
    assert "from the future" in rx.latch_info["utc_refusal"]


def test_peek_changes_nothing_and_matches_the_latch():
    rx = CueReceiver(clock=lambda: 0.0, utc=FixedUtc(), allow_unsynced=False)
    feed(rx, 0.0, 10.0)
    a = rx.peek(*OWN, now_t=10.0)
    assert rx.latched is None and rx.stats(now_t=10.0).latched is False
    b = rx.latch(*OWN, now_t=10.0)
    assert a.belief_r0_ned == b.belief_r0_ned and a.belief_vel0_ned == b.belief_vel0_ned
    with pytest.raises(CueLinkError, match="nothing reads the link after the GO"):
        rx.peek(*OWN, now_t=10.5)


def test_a_hole_in_the_window_refuses_until_the_window_is_clean():
    rx = CueReceiver(clock=lambda: 0.0, utc=FixedUtc(), allow_unsynced=False)
    feed(rx, 0.0, 4.0)
    feed(rx, 6.0, 8.0)                                # a 2 s dropout
    with pytest.raises(CueQualityError, match="hole"):
        rx.peek(*OWN, now_t=8.0, max_gap_s=1.0)
    feed(rx, 8.0, 13.0)                               # refill a clean window
    rx.peek(*OWN, now_t=13.0, max_gap_s=1.0)


def test_mono_to_utc_fails_closed_until_it_can_be_trusted():
    clk = [100.0]
    m = MonoToUtc(need=3, clock=lambda: clk[0])
    assert m.utc_now() is None
    for i in range(3):
        m.ingest((UTC0 + 100.0 + i - 0.004 * (i == 1)) * 1e6, t_rx=100.0 + i)
        clk[0] = 100.0 + i
    # three agreeing messages: trusted, and the least-delayed one sets it
    assert abs(m.utc_now(102.0) - (UTC0 + 102.0)) < 1e-6
    m.ingest(0.0, t_rx=103.0)                         # the vehicle lost GPS time
    assert m.utc_now(103.0) is None and m.n_invalid == 1
    for i in range(3):
        m.ingest((UTC0 + 104.0 + i) * 1e6, t_rx=104.0 + i)
    assert m.utc_now(106.0) is not None
    m.ingest((UTC0 + 107.0 + 5.0) * 1e6, t_rx=107.0)  # the offset jumps 5 s
    assert m.utc_now(107.0) is None and m.n_jumps == 1
    for i in range(3):
        m.ingest((UTC0 + 108.0 + i) * 1e6, t_rx=108.0 + i)
    assert m.utc_now(120.0) is None                   # and it goes stale


def test_boot_to_utc_is_exact_and_fails_closed():
    b = BootToUtc()
    assert b.utc_of_boot_ms(5000) is None
    b.ingest(1.8e15, 5000)
    assert not b.ok                                   # one message is not enough
    b.ingest(1.8e15 + 1e6, 6000)
    assert b.utc_of_boot_ms(6500) == pytest.approx(1.8e9 + 1.5)
    b.ingest(1.8e15 + 9e6, 7000)                      # jumped
    assert b.utc_of_boot_ms(7000) is None and b.n_jumps == 1
    b.ingest(12_000_000, 8000)                        # no GPS time at all
    assert not b.ok and b.n_invalid == 1


def test_system_clock_utc_maps_a_past_instant():
    s = SystemClockUtc(clock=lambda: 50.0, wall=lambda: UTC0 + 50.0)
    assert s.utc_now(48.5) == pytest.approx(UTC0 + 48.5)


# ============================================================ CueTrigger

class NoRc:
    """Stand-in for the enable switch's RC reader."""

    def __init__(self):
        self.pwm, self.link = 1000, True

    def poll(self, t):
        return rf.TriggerState(go=self.link and self.pwm > 1700, link_ok=self.link,
                               age_s=0.05 if self.link else 9.0, raw_us=self.pwm)


def make_trigger(enable=None, own=OWN, **kw):
    rx = CueReceiver(clock=lambda: 0.0, utc=FixedUtc(), allow_unsynced=False)
    said = []
    trig = rf.CueTrigger(rx, own_lla=lambda: own, enable=enable, log=said.append, **kw)
    return trig, rx, said


def settle(trig, n=4):
    for _ in range(n):
        trig.observe_gate(True)


def test_cue_fires_go_only_after_the_arm_gate_has_held():
    trig, rx, said = make_trigger()
    feed(rx, 0.0, 10.0)
    assert trig.poll(10.0).go is False and "not yet steady" in trig.status
    settle(trig, 3)
    assert trig.poll(10.0).go is False                # 3 of 4 ticks
    trig.observe_gate(False)                          # a flicker resets it
    settle(trig, 3)
    assert trig.poll(10.0).go is False
    trig.observe_gate(True)
    st = trig.poll(10.0)
    assert st.go and st.link_ok and trig.fired and trig.fired_t == 10.0
    assert any("GO: fit of" in m and "measured" in m and "removed" in m for m in said)


def test_the_belief_at_go_is_the_truth_at_go_relative_to_where_the_vehicle_is():
    """Own position is read at GO, so a vehicle that has drifted from the launch
    point (and sits at standby height) still gets the target relative to ITSELF."""
    own = lla(15.0, -8.0, up=7.0)                     # 15 m north, 8 m west, 7 m up
    trig, rx, _ = make_trigger(own=own)
    feed(rx, 0.0, 30.0, delay=(0.2, 0.5), seed=2)
    settle(trig)
    assert trig.poll(30.6).go
    r0, v0 = trig.take_belief()
    n, e = truth(30.6)
    assert abs(r0[0] - (n - 15.0)) < 0.1 and abs(r0[1] - (e + 8.0)) < 0.1
    assert abs(r0[2] - (7.0 - UP)) < 0.05             # down: own height - target's
    assert abs(v0[0] - V[0]) < 0.05 and abs(v0[1] - V[1]) < 0.05


def test_the_belief_is_handed_over_once_and_the_link_is_dead_after_go():
    trig, rx, _ = make_trigger()
    feed(rx, 0.0, 10.0)
    settle(trig)
    trig.poll(10.0)
    first = trig.take_belief()
    assert first is not None and trig.take_belief() is None
    n_before = rx.stats(now_t=10.0).n_rx
    feed(rx, 10.0, 12.0)                              # records keep arriving
    st = trig.poll(12.0)
    assert st.go and trig.take_belief() is None
    assert rx.stats(now_t=12.0).n_rx == n_before and rx.stats(now_t=12.0).n_after_latch >= 10
    assert trig.solution.latch_t == 10.0


def test_enable_switch_is_the_veto_and_never_counts_if_found_high():
    en = NoRc()
    en.pwm = 2000                                     # left HIGH at start-up
    trig, rx, said = make_trigger(enable=en)
    feed(rx, 0.0, 10.0)
    settle(trig)
    assert trig.poll(10.0).go is False and "already HIGH at start" in trig.status
    en.pwm = 1000
    st = trig.poll(10.1)
    assert st.go is False and st.link_ok and "your veto" in trig.status
    feed(rx, 10.0, 11.0)
    en.pwm = 2000
    assert trig.poll(11.0).go


def test_no_rc_link_for_the_enable_switch_is_link_denied():
    en = NoRc()
    en.link = False
    trig, rx, _ = make_trigger(enable=en)
    feed(rx, 0.0, 10.0)
    settle(trig)
    st = trig.poll(10.0)
    assert st.go is False and st.link_ok is False     # FAILSAFE 1's input


def test_a_dead_stream_never_fires_and_a_returning_one_must_refill_the_window():
    trig, rx, said = make_trigger()
    feed(rx, 0.0, 6.0)
    settle(trig)
    assert trig.poll(9.0).go is False and "stale" in trig.status      # died at 6 s
    feed(rx, 8.0, 10.0)                               # back after a 2 s dropout
    assert trig.poll(10.0).go is False and "hole" in trig.status
    feed(rx, 10.0, 12.0)                              # the old records age out...
    assert trig.poll(12.0).go is False and "span" in trig.status
    feed(rx, 12.0, 13.0)                              # ...and 5 s of clean stream
    assert trig.poll(13.0).go
    # the wait reasons were said once each, not once per tick
    assert sum("stale" in m for m in said) == 1


def test_a_stream_that_has_only_just_started_does_not_fire_go():
    """The 5 s window is what buys the speed estimate: 1.5 s of data clears
    the fit's own floor (8 points, 1 s) but not the trigger's."""
    trig, rx, _ = make_trigger()
    feed(rx, 0.0, 1.6)
    settle(trig)
    assert trig.poll(1.6).go is False and "span" in trig.status
    rx.peek(*OWN, now_t=1.6)                          # the bare fit would accept it
    feed(rx, 1.6, 4.6)
    assert trig.poll(4.6).go


def test_no_own_position_and_a_refused_check_both_hold_go():
    own = [None]
    rx = CueReceiver(clock=lambda: 0.0, utc=FixedUtc(), allow_unsynced=False)
    trig = rf.CueTrigger(rx, own_lla=lambda: own[0], log=lambda m: None,
                         go_check=lambda sol: "too far")
    feed(rx, 0.0, 10.0)
    settle(trig)
    assert trig.poll(10.0).go is False and "own GPS position" in trig.status
    own[0] = OWN
    assert trig.poll(10.0).go is False and "too far" in trig.status
    assert rx.latched is None                         # held, not latched
    trig.go_check = None
    assert trig.poll(10.0).go


def test_unsynced_records_refuse_go_through_the_trigger():
    rx = CueReceiver(clock=lambda: 0.0, utc=None, allow_unsynced=False)
    trig = rf.CueTrigger(rx, own_lla=lambda: OWN, log=lambda m: None)
    feed(rx, 0.0, 10.0)
    settle(trig)
    assert trig.poll(10.0).go is False and "no shared time base" in trig.status


def test_go_time_check_geofence_range_and_catchability():
    caps = {"v_cap_ms": 36.0}
    limits = {"MPC_XY_VEL_MAX": 33.0, "GF_ACTION": 2, "GF_MAX_HOR_DIST": 457.0}

    class Sol:
        def __init__(self, r, v):
            self.belief_r0_ned, self.belief_vel0_ned = r, v
    chk = rf.make_cue_go_check(limits, caps, max_range_m=None)
    assert chk(Sol((200.0, -300.0, 0.0), (0.0, 12.0, 0.0))) is None
    assert "geofence" in chk(Sol((600.0, 0.0, 0.0), (5.0, 0.0, 0.0)))
    assert "does not" in chk(Sol((200.0, 0.0, 0.0), (40.0, 0.0, 0.0)))   # faster than us
    assert "beyond" in rf.make_cue_go_check(limits, caps, 300.0)(
        Sol((200.0, -300.0, 0.0), (0.0, 12.0, 0.0)))


# ============================================================ GO-edge latch

class HookTerminal:
    """A terminal that can take a belief at GO (records what it was given)."""

    def __init__(self):
        self.got = []
        self.phase, self.r_hat, self.searching = "A", None, False

    def set_belief_once(self, r0, v0, t=None):
        self.got.append((r0, v0, t))

    def step(self, box, own, t, **kw):
        tel = rf.StepTelemetry(t=t, detected=box is not None)
        tel.phase, tel.r_hat_m = self.phase, self.r_hat
        return rf.Setpoint(1.0, 0.0, 0.0, 40.0), tel


GO = rf.TriggerState(go=True, link_ok=True, age_s=0.05, raw_us=2000)
BELIEF = ((200.0, -60.0, -3.0), (0.0, 12.0, 0.0))


def cue_sm(guidance, **cfg):
    sm = rf.RealFlightSM(sm_cfg(pursuit_mode=True, cue_go=True, **cfg),
                         guidance=guidance)
    sm.step(sm_obs(0.0))
    sm.step(sm_obs(0.05))
    return sm


def test_go_edge_writes_the_cue_belief_into_the_terminal_once():
    term = HookTerminal()
    sm = cue_sm(term)
    sm.step(sm_obs(0.10, trigger=GO, cue_belief=BELIEF))
    assert sm.state == rf.State.ENGAGE
    assert term.got == [(BELIEF[0], BELIEF[1], 0.10)]
    assert sm.cue_belief.is_set and sm.cue_belief.value == BELIEF
    with pytest.raises(rf.LatchViolation):
        sm.cue_belief.set(BELIEF, t=1.0)
    sm.step(sm_obs(0.15, trigger=GO, cue_belief=BELIEF))      # later ticks: ignored
    assert len(term.got) == 1


def test_go_without_a_belief_goes_safe_instead_of_flying_the_placeholder():
    term = HookTerminal()
    sm = cue_sm(term)
    sm.step(sm_obs(0.10, trigger=GO))                 # GO, but no belief came with it
    assert sm.state == rf.State.SAFE and sm.safe_reason == "cue_go_without_belief"
    assert term.got == []


def test_go_with_a_terminal_that_cannot_take_a_belief_goes_safe():
    class Deaf(HookTerminal):
        set_belief_once = None
    sm = cue_sm(Deaf())
    sm.step(sm_obs(0.10, trigger=GO, cue_belief=BELIEF))
    assert sm.state == rf.State.SAFE
    assert sm.safe_reason == "cue_go_terminal_cannot_take_belief"


def test_without_cue_go_a_stray_belief_on_the_observation_is_ignored():
    """Default behaviour is unchanged: the field is read only under cue_go."""
    term = HookTerminal()
    sm = rf.RealFlightSM(sm_cfg(pursuit_mode=True), guidance=term)
    sm.step(sm_obs(0.0))
    sm.step(sm_obs(0.05))
    sm.step(sm_obs(0.10, trigger=GO, cue_belief=BELIEF))
    assert sm.state == rf.State.ENGAGE and term.got == [] and not sm.cue_belief.is_set


def _pursuit_terminal(**cfg_kw):
    from flight.camera import CameraModel
    from flight.pursuit_terminal import PursuitTerminalConfig, PursuitTerminalGuidance
    cam = CameraModel(540.0, 540.0, 640.0, 480.0)
    return PursuitTerminalGuidance(
        PursuitTerminalConfig(**cfg_kw), cam, 0.5, rf.GuidanceConfig(),
        belief_r0_ned=(0.0, 16.7, 1.0), belief_vel0_ned=(9.0, 0.0, 0.0), go_at_s=0.0)


def test_pursuit_terminal_takes_the_belief_once_and_only_before_it_flies():
    term = _pursuit_terminal(arrival_search=True)
    term.set_belief_once((200.0, -60.0, -3.0), (0.0, 12.0, 0.0), t=31.5)
    assert np.allclose(term._r_track, (200.0, -60.0, -3.0))
    assert np.allclose(term._v_track, (0.0, 12.0, 0.0))
    assert np.allclose(term._belief_vel0, (0.0, 12.0, 0.0))
    assert term._search_t0 == 31.5          # the search's uncertainty clock starts at GO
    with pytest.raises(RuntimeError, match="already latched"):
        term.set_belief_once((1.0, 2.0, 3.0), (0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        _pursuit_terminal().set_belief_once((float("nan"), 0.0, 0.0), (0.0, 0.0, 0.0))


def test_pursuit_terminal_refuses_a_belief_after_it_has_flown_on_one():
    term = _pursuit_terminal()
    own = rf.OwnState(quat=(1.0, 0.0, 0.0, 0.0), psi_rad=0.0, alt_m=7.0,
                      vel_ned=(0.0, 0.0, 0.0))
    sp, _tel = term.step(None, own, 0.05)
    assert sp is not None
    with pytest.raises(RuntimeError, match="already flown"):
        term.set_belief_once((200.0, -60.0, -3.0), (0.0, 12.0, 0.0))


def test_the_first_command_after_go_flies_at_the_latched_belief():
    """End to end through the real state machine and the real pursuit terminal:
    the placeholder belief is north-east; the cue says the target is due WEST."""
    from flight.pursuit_terminal import PursuitTerminalGuidance
    term = _pursuit_terminal()
    assert isinstance(term, PursuitTerminalGuidance)
    sm = cue_sm(term)
    west = ((0.0, -200.0, 0.0), (0.0, 0.0, 0.0))
    sm.step(sm_obs(0.10, trigger=GO, cue_belief=west))
    for k in range(40):
        dec = sm.step(sm_obs(0.15 + 0.05 * k, trigger=GO,
                             vel_ned=(0.0, 0.0, 0.0)))
    assert dec.state == rf.State.ENGAGE
    assert dec.setpoint.v_east < -3.0 and abs(dec.setpoint.v_north) < 1.0


# ============================================================ lost-target clock

DET = dict(det_new=True, det_range_m=20.0, det_box_xywh=(600.0, 440.0, 20.0, 20.0))


def engaged(term, **cfg):
    sm = rf.RealFlightSM(sm_cfg(pursuit_mode=True, **cfg), guidance=term)
    sm.step(sm_obs(0.0))
    sm.step(sm_obs(0.05))
    sm.step(sm_obs(0.10, trigger=GO))
    assert sm.state == rf.State.ENGAGE
    return sm


def run_to(sm, t0, t1, **obs_kw):
    t = t0
    while t < t1 - 1e-9 and sm.state == rf.State.ENGAGE:
        sm.step(sm_obs(t, trigger=GO, **obs_kw))
        t += 0.05
    return sm


def test_one_stray_decode_in_the_belief_transit_ends_the_engagement_by_default():
    """Pins the defect (guidance review F1) on the DEFAULT path."""
    term = HookTerminal()
    sm = engaged(term)
    sm.step(sm_obs(1.0, trigger=GO, **DET))           # one decode, no track
    run_to(sm, 1.05, 3.2)
    assert sm.state == rf.State.BREAKOFF
    assert "target_lost(2.0s)" in sm.transitions[-1].reason


def test_needs_track_keeps_the_clock_off_until_there_is_a_track():
    term = HookTerminal()
    sm = engaged(term, pursuit_lost_clock_needs_track=True)
    sm.step(sm_obs(1.0, trigger=GO, **DET))
    run_to(sm, 1.05, 9.0)
    assert sm.state == rf.State.ENGAGE                # still flying on belief
    # once there IS a track, losing it counts again
    term.phase, term.r_hat = "B", 8.0
    sm.step(sm_obs(9.0, trigger=GO, **DET))
    run_to(sm, 9.05, 11.2)
    assert sm.state == rf.State.BREAKOFF


def test_off_in_search_stops_the_clock_while_the_terminal_is_searching():
    def fly(**cfg):
        term = HookTerminal()
        sm = engaged(term, **cfg)
        term.phase, term.r_hat = "B", 8.0
        sm.step(sm_obs(1.0, trigger=GO, **DET))       # a track, then...
        term.phase, term.searching = "A", True        # ...fallback into the search
        return run_to(sm, 1.05, 9.0)
    assert fly().state == rf.State.BREAKOFF           # default: the clock ran
    assert fly(pursuit_lost_clock_off_in_search=True).state == rf.State.ENGAGE


def test_far_loss_clock_applies_beyond_the_miss_range_only():
    def fly(r_hat, **cfg):
        term = HookTerminal()
        sm = engaged(term, **cfg)
        term.phase, term.r_hat = "B", r_hat
        sm.step(sm_obs(1.0, trigger=GO, **DET))
        return run_to(sm, 1.05, 12.0)
    far = fly(6.0, engage_far_lost_target_s=6.0)
    assert far.state == rf.State.BREAKOFF
    assert "target_lost(6.0s)" in far.transitions[-1].reason
    assert far.transitions[-1].t == pytest.approx(7.05, abs=0.06)
    # default: the same far loss ends at 2 s
    assert fly(6.0).transitions[-1].t == pytest.approx(3.05, abs=0.06)
    # the close-miss rule is untouched by the far clock
    close = fly(2.0, engage_far_lost_target_s=6.0)
    assert close.state == rf.State.SAFE and close.safe_reason == "pursuit_miss"
    assert close.transitions[-1].t == pytest.approx(3.05, abs=0.06)


def test_arrival_search_is_off_unless_asked_for_and_reaches_the_terminal(capsys):
    """The flight program can now switch the arrival search on (it could only
    be reached from the fast sim before); off by default."""
    from flight.camera import CameraModel
    cam = CameraModel(540.0, 540.0, 640.0, 480.0)

    def build(extra):
        args = rf.build_arg_parser().parse_args(
            ["--sitl-smoke", "--target-start", "0,16.7", "--target-vel", "9,0",
             "--terminal", "pursuit"] + extra)
        return rf.build_terminal(args, rf.build_config(args), rf.GuidanceConfig(), cam)
    assert build([]).cfg.arrival_search is False
    on = build(["--pursuit-arrival-search", "stern",
                "--pursuit-search-speed-sigma", "0.04"])
    assert on.cfg.arrival_search and on.cfg.search_start == "stern"
    assert on.cfg.search_sigma_speed_frac == 0.04
    assert "ARRIVAL SEARCH ON" in capsys.readouterr().out


def test_launch_lines_warn_when_the_clock_cannot_outlast_the_terminals_fallback():
    from flight.pursuit_terminal import PursuitTerminalConfig
    p = PursuitTerminalConfig()
    assert any("WARNING" in ln for ln in rf.lost_clock_lines(sm_cfg(pursuit_mode=True), p))
    cfg = sm_cfg(pursuit_mode=True, engage_far_lost_target_s=6.0,
                 pursuit_lost_clock_needs_track=True,
                 pursuit_lost_clock_off_in_search=True)
    lines = rf.lost_clock_lines(cfg, p)
    assert not any("WARNING" in ln for ln in lines)
    assert "6.0 s beyond it" in lines[0] and "arrival search" in lines[0]


# ============================================================ the flying preset

def parse(argv):
    return rf.build_arg_parser().parse_args(argv)


BASE = ["--target-start", "0,16.7", "--target-vel", "9,0", "--terminal", "pursuit"]


def test_fly_takes_the_preset_and_no_other_mode_does(capsys):
    fly = rf.build_config(parse(["--fly"] + BASE))
    assert fly.pursuit_lost_clock_needs_track is rf.FLY_PRESET["pursuit_lost_clock_needs_track"]
    assert fly.pursuit_lost_clock_off_in_search is rf.FLY_PRESET["pursuit_lost_clock_off_in_search"]
    assert fly.engage_far_lost_target_s == rf.FLY_PRESET["engage_far_lost_target_s"]
    assert math.isinf(fly.engage_max_s) and math.isinf(fly.mission_max_s)
    for mode in ("--sitl-smoke", "--bench-props-off", "--dry-run"):
        c = rf.build_config(parse([mode] + BASE))
        assert (c.pursuit_lost_clock_needs_track, c.pursuit_lost_clock_off_in_search,
                c.engage_far_lost_target_s, c.engage_max_s, c.mission_max_s) == (
                    False, False, None, 30.0, 120.0), mode
    stock = rf.build_config(parse(["--fly", "--target-start", "0,16.7",
                                   "--target-vel", "9,0", "--terminal", "stock"]))
    assert stock.engage_max_s == 12.0 and stock.mission_max_s == 120.0   # pursuit only


def test_a_typed_value_beats_the_preset(capsys):
    c = rf.build_config(parse(["--fly"] + BASE + [
        "--engage-max-s", "45", "--mission-max-s", "200",
        "--lost-clock-needs-track", "off", "--lost-clock-off-in-search", "off",
        "--engage-far-lost-target-s", "-1"]))
    assert (c.engage_max_s, c.mission_max_s) == (45.0, 200.0)
    assert (c.pursuit_lost_clock_needs_track, c.pursuit_lost_clock_off_in_search,
            c.engage_far_lost_target_s) == (False, False, None)
    on = rf.build_config(parse(["--sitl-smoke"] + BASE + [
        "--lost-clock-needs-track", "on", "--engage-far-lost-target-s", "5"]))
    assert on.pursuit_lost_clock_needs_track and on.engage_far_lost_target_s == 5.0


def _terminal_from(argv):
    from flight.camera import CameraModel
    args = parse(argv)
    return rf.build_terminal(args, rf.build_config(args), rf.GuidanceConfig(),
                             CameraModel(540.0, 540.0, 640.0, 480.0)), args


def test_fly_preset_turns_on_the_search_the_guard_and_the_soft_brake(capsys):
    """Builder ruling 2026-11-18 ("whatever gets highest intercept rate") and
    the flight-hardware rerun: --fly searches on arrival (`believed`), guards
    against stale decodes, and brakes the approach dash at 3 m/s^2 with the
    vertical arrival-sync it was simulated with. No other mode does."""
    fly, _ = _terminal_from(["--fly"] + BASE + ["--pursuit-approach-dash"])
    p = fly.cfg
    assert p.arrival_search and p.search_start == rf.FLY_PRESET["pursuit_arrival_search"] == "believed"
    assert p.stale_decode_guard is True
    assert p.brake_accel_ms2 == rf.FLY_PRESET["pursuit_dash_brake_accel"] == 3.0
    assert p.brake_vert_sync is True
    out = capsys.readouterr().out
    assert "ARRIVAL SEARCH ON" in out and "STALE-DECODE GUARD ON" in out
    for mode in ("--sitl-smoke", "--bench-props-off", "--dry-run"):
        other, _ = _terminal_from([mode] + BASE + ["--pursuit-approach-dash"])
        q = other.cfg
        assert (q.arrival_search, q.stale_decode_guard, q.brake_accel_ms2,
                q.brake_vert_sync) == (False, False, 4.0, False), mode
    # without the approach dash the brake preset has nothing to apply to
    plain, _ = _terminal_from(["--fly"] + BASE)
    assert plain.cfg.arrival_search and plain.cfg.brake_accel_ms2 == 4.0


def test_typed_search_guard_and_brake_beat_the_preset(capsys):
    off, _ = _terminal_from(["--fly"] + BASE + [
        "--pursuit-approach-dash", "--pursuit-arrival-search", "off",
        "--pursuit-stale-decode-guard", "off", "--pursuit-dash-brake-accel", "4"])
    assert (off.cfg.arrival_search, off.cfg.stale_decode_guard,
            off.cfg.brake_accel_ms2) == (False, False, 4.0)
    stern, _ = _terminal_from(["--fly"] + BASE + ["--pursuit-arrival-search", "stern"])
    assert stern.cfg.search_start == "stern"
    on, _ = _terminal_from(["--sitl-smoke"] + BASE + ["--pursuit-stale-decode-guard", "on"])
    assert on.cfg.stale_decode_guard is True and on.cfg.arrival_search is False


def test_prearm_summary_says_what_the_search_and_the_brake_will_do(capsys):
    term, args = _terminal_from(["--fly", "--mavsdk-url", SERIAL, "--trigger", "rc",
                                 "--arm-permit-channel", str(PERMIT_CH),
                                 "--source", "picamera", "--manual-belief"]
                                + BASE + ["--pursuit-approach-dash"])
    cfg = rf.build_config(args)
    text = "\n".join(rf.prearm_summary(args, cfg, rf.commanded_caps(args, cfg, term)))
    assert "ARRIVAL SEARCH is ON ('believed')" in text
    assert "without having seen it" in text
    assert "brakes into the believed rendezvous at 3.0 m/s^2 with vertical arrival-sync" in text
    off, args2 = _terminal_from(["--fly", "--mavsdk-url", SERIAL, "--trigger", "rc",
                                 "--arm-permit-channel", str(PERMIT_CH),
                                 "--source", "picamera", "--manual-belief"]
                                + BASE + ["--pursuit-arrival-search", "off"])
    cfg2 = rf.build_config(args2)
    text2 = "\n".join(rf.prearm_summary(args2, cfg2, rf.commanded_caps(args2, cfg2, off)))
    assert "arrival search is OFF" in text2 and "ARRIVAL SEARCH is ON" not in text2


def test_an_unlimited_engagement_never_times_out_and_says_what_ends_it():
    term = HookTerminal()
    sm = engaged(term, engage_max_s=math.inf, mission_max_s=math.inf)
    run_to(sm, 0.15, 400.0)
    assert sm.state == rf.State.ENGAGE                # 400 s and still engaged
    bounded = engaged(HookTerminal(), engage_max_s=30.0)
    run_to(bounded, 0.15, 40.0)
    assert "engage_timeout" in bounded.transitions[-1].reason
    ends = " ".join(rf.what_ends_an_unbounded_engagement())
    for word in ("pilot", "geofence", "battery", "watchdog", "lost-target"):
        assert word in ends


def test_limit_check_copes_with_no_time_limit():
    caps = {"v_cap_ms": 12.0, "belief_r0_ne": (0.0, 16.7), "belief_v_ne": (12.0, 0.0),
            "approach_dash": False, "brake_accel_ms2": None, "accel_ms2": 6.0}
    limits = {"MPC_XY_VEL_MAX": 12.0, "GF_ACTION": 2, "GF_MAX_HOR_DIST": 457.0,
              "GF_MAX_VER_DIST": 30.5}
    hard, refusals, notes = rf.check_platform_limits(
        limits, [], caps, sm_cfg(pursuit_mode=True, engage_max_s=math.inf))
    text = " ".join(refusals + notes)
    assert "nan" not in text and "inf" not in text
    assert "no engage time limit" in text and "geofence" in text


# ============================================================ argument rules

FLY_CUE = ["--fly", "--mavsdk-url", SERIAL, "--trigger", "cue", "--terminal", "pursuit",
           "--rc-channel", str(GO_CH), "--arm-permit-channel", str(PERMIT_CH),
           "--source", "picamera", "--target-span-m", "0.30",
           "--standby-alt-m", "7", "--standby-yaw-deg", "40",
           "--cue-udp-port", "0", "--miss-safe-behavior", "land"]


def test_fly_accepts_the_cue_as_go_without_a_typed_belief():
    assert rf.validate_fly_args(parse(FLY_CUE)) == []


@pytest.mark.parametrize("change,needle", [
    (lambda a: a + ["--cue-no-enable"], "only human veto"),
    (lambda a: ["stock" if x == "pursuit" else x for x in a], "--terminal pursuit"),
    (lambda a: a[:a.index("--standby-alt-m")] + a[a.index("--standby-alt-m") + 2:],
     "--standby-alt-m typed"),
    (lambda a: a + ["--cue-utc-source", "none"], "can never fire GO"),
])
def test_fly_cue_refusals(change, needle):
    why = rf.validate_fly_args(parse(change(list(FLY_CUE))))
    assert any(needle in w for w in why), why


# ============================================================ audit

def _mutant(tmp_path, anchor, extra):
    src = open(os.path.join(_REPO_ROOT, "flight", "deploy", "real_flight.py")).read()
    assert src.count(anchor) == 1, anchor
    p = tmp_path / "mutant.py"
    p.write_text(src.replace(anchor, extra + anchor, 1))
    return str(p)


def test_audit_passes_and_covers_the_cue_link():
    assert rf.honesty_audit(verbose=False) == 0
    assert any(p.endswith(os.path.join("flight", "cue_link.py"))
               for p in rf.audited_module_paths())


def test_a_second_cue_belief_write_fails_the_audit(tmp_path):
    p = _mutant(tmp_path, "            self.t_dash_start = obs.t\n",
                "            self.cue_belief.set(obs.cue_belief, t=obs.t)\n")
    assert rf.honesty_audit(path=p, verbose=False) == 1


def test_a_second_latch_of_the_live_link_fails_the_audit(tmp_path):
    """Reading the link again after GO would be a datalink in flight."""
    p = _mutant(tmp_path,
                "        if self.fired:\n            # After the latch the link is closed",
                "        self.receiver.latch(0.0, 0.0, 0.0)\n")
    assert rf.honesty_audit(path=p, verbose=False) == 1


# ============================================================ the driver, end to end

def capture_trigger(monkeypatch):
    box = {}
    orig = rf.build_cue_trigger

    def wrapped(args, cfg):
        box["trig"] = orig(args, cfg)
        return box["trig"]
    monkeypatch.setattr(rf, "build_cue_trigger", wrapped)
    return box


def cue_feeder(drone, box, *, delay_s=0.4, until=None, start=0.0, period=0.2):
    """A tick hook that plays the target's GPS into the receiver on the fake
    vehicle's clock: each sample is delivered `delay_s` after it was taken, and
    stamped with the vehicle's own notion of UTC (the fake's SYSTEM_TIME)."""
    state = {"k": 0}
    t_abs0 = {"v": None}

    def hook(t):
        trig = box.get("trig")
        if trig is None:
            return
        if t_abs0["v"] is None:
            t_abs0["v"] = drone.now() - t
        while True:
            t_sample = start + state["k"] * period
            if t_sample + delay_s > t or (until is not None and t_sample > until):
                return
            n, e = truth(t_sample)
            la, lo, al = lla(n, e, own=drone.lla)
            pay = encode_record(CueRecord(0.0, la, lo, al, V[0], V[1], 0.0),
                                t_src=t_sample, t_utc=drone.utc0 + t_sample)
            trig.receiver.feed(pay, t_rx=t_abs0["v"] + t_sample + delay_s)
            state["k"] += 1
    return hook, t_abs0


BENCH_CUE = ["--bench-props-off", "--mavsdk-url", SERIAL, "--terminal", "pursuit",
             "--trigger", "cue", "--cue-no-enable", "--cue-udp-port", "0",
             "--loop-watchdog-s", "0", "--smoke-duration", "20"]


def test_bench_cue_is_the_go_command_and_the_delay_is_taken_out(monkeypatch, capsys,
                                                               tmp_path):
    """The whole path on the fake vehicle: SYSTEM_TIME gives the Pi UTC, records
    arrive 0.4 s late, GO fires by itself, and the belief at GO is the truth at
    GO (0.4 s x 12 m/s = 4.8 m would be left in it otherwise)."""
    d = fm.FakeDrone()
    box = capture_trigger(monkeypatch)
    d.tick_hook, t_abs0 = cue_feeder(d, box, delay_s=0.4)
    r = drive(monkeypatch, capsys, tmp_path, BENCH_CUE, drone=d)
    trig = box["trig"]
    assert r.rc == 0, r.out[-3000:]
    assert trig.fired and "go_edge_pursuit" in r.out
    assert "cue belief LATCHED" in r.out and "measured" in r.out and "removed" in r.out
    assert "time base" in r.out and "'utc'" in r.out
    t_go = trig.fired_t - t_abs0["v"]                 # on the fake's mission clock
    n, e = truth(t_go)
    r0 = trig.solution.belief_r0_ned
    assert abs(r0[0] - n) < 0.2 and abs(r0[1] - e) < 0.2, (r0, n, e)
    assert r.actions == ["offboard.start", "offboard.stop"] and d.armed is False
    assert "link closed" in r.out and "GO fired" in r.out


def test_bench_cue_without_the_vehicle_clock_refuses_to_go(monkeypatch, capsys, tmp_path):
    """Switch the shared clock off (the vehicle has no GPS time): no GO."""
    d = fm.FakeDrone()
    d.utc0 = None
    box = capture_trigger(monkeypatch)
    hook, _ = cue_feeder(d, box)
    d.utc0 = None

    def hook2(t):                                     # records still carry a t_utc
        d.utc0 = 1.8e9
        hook(t)
        d.utc0 = None
    d.tick_hook = hook2
    r = drive(monkeypatch, capsys, tmp_path, BENCH_CUE + ["--standby-max-s", "12"],
              drone=d)
    assert r.rc == 1 and not box["trig"].fired
    assert "no shared time base" in r.out and "go_edge" not in r.out
    assert "GO never fired" in r.out


def test_a_stream_that_dies_in_standby_never_dashes(monkeypatch, capsys, tmp_path):
    d = fm.FakeDrone()
    box = capture_trigger(monkeypatch)
    d.tick_hook, _ = cue_feeder(d, box, until=0.6)    # dies before the gates hold
    r = drive(monkeypatch, capsys, tmp_path, BENCH_CUE + ["--standby-max-s", "10"],
              drone=d)
    assert r.rc == 1 and not box["trig"].fired
    assert "standby_timeout" in r.out and "go_edge" not in r.out


def test_fly_with_the_cue_needs_the_permit_then_the_enable_switch(monkeypatch, capsys,
                                                                 tmp_path, dummy_server):
    """--fly: nothing arms before the permit; the cue cannot fire while the
    enable switch is LOW; raising it lets the cue fire GO by itself."""
    d = fm.FakeDrone()
    d._server_process = dummy_server
    d.params["MPC_XY_VEL_MAX"] = 33.0
    d.rc_script = lambda t: rc(go=2000 if t >= 25.0 else 1000,
                               permit=2000 if t >= 3.0 else 1000)
    box = capture_trigger(monkeypatch)
    d.tick_hook, t_abs0 = cue_feeder(d, box, delay_s=0.3)
    r = drive(monkeypatch, capsys, tmp_path,
              FLY_CUE + ["--engage-max-s", "6"], drone=d, detector=smoke_cam())
    trig = box["trig"]
    assert r.rc == 0, r.out[-3000:]
    assert "belief: NONE YET. The cue is the GO command" in r.out
    assert "keep it LOW to hold fire" in r.out
    t_arm = next(t for t, n, _ in d.calls if n == "action.arm")
    assert t_arm >= 3.0
    t_go = trig.fired_t - t_abs0["v"]
    assert t_go >= 25.0, t_go                         # held by the LOW enable switch
    assert "your veto" in r.out and "cue belief LATCHED" in r.out
    n, e = truth(t_go)
    assert abs(trig.solution.belief_r0_ned[1] - e) < 0.5


def test_fly_pre_arm_summary_states_no_time_limit(monkeypatch, capsys, tmp_path,
                                                  dummy_server):
    d = fm.FakeDrone()
    d._server_process = dummy_server
    d.params["MPC_XY_VEL_MAX"] = 33.0
    d.rc_script = lambda t: rc(go=1000, permit=1000)  # never permitted: just the text
    r = drive(monkeypatch, capsys, tmp_path, FLY_CUE + ["--arm-permit-timeout-s", "5"],
              drone=d, detector=smoke_cam())
    assert r.rc == 1 and r.actions == []
    assert "NO TIME LIMIT" in r.out and "the board's geofence" in r.out
    assert "lost-target rule" in r.out


def test_cue_go_over_real_udp_with_a_delayed_sender(monkeypatch, capsys, tmp_path):
    """Real sockets, real time (about 12 s): a sender thread plays the target
    over loopback UDP, each record 0.25 s late and stamped with the wall clock;
    the program uses the system clock as its UTC (the declared fallback)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    d = fm.FakeDrone()
    fm.install(monkeypatch, d)
    box = capture_trigger(monkeypatch)
    stop = threading.Event()
    t_wall0 = time.time()

    def sender():
        out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        k = 0
        while not stop.is_set():
            t_sample = time.time() - t_wall0 - 0.25   # taken 0.25 s ago
            n, e = truth(t_sample)
            la, lo, al = lla(n, e, own=d.lla)
            out.sendto(encode_record(CueRecord(0.0, la, lo, al, V[0], V[1], 0.0),
                                     t_src=t_sample, t_utc=t_wall0 + t_sample, seq=k),
                       ("127.0.0.1", port))
            k += 1
            time.sleep(0.2)
    th = threading.Thread(target=sender, daemon=True)
    th.start()
    try:
        rc_exit = rf.main(["--bench-props-off", "--mavsdk-url", SERIAL,
                           "--terminal", "pursuit", "--trigger", "cue",
                           "--cue-no-enable", "--cue-udp-port", str(port),
                           "--cue-utc-source", "system", "--loop-watchdog-s", "0",
                           "--smoke-duration", "11",
                           "--log-csv", str(tmp_path / "ticks.csv")])
    finally:
        stop.set()
        th.join(timeout=2.0)
    out = capsys.readouterr().out
    trig = box["trig"]
    assert rc_exit == 0, out[-3000:]
    assert trig.fired and "'utc'" in out and "UTC source 'system'" in out
    info = trig.receiver.latch_info
    assert info["utc_source"] == "system-clock"
    assert info["time_base"] == "utc" and 0.2 < info["delay_median_s"] < 0.45
    # truth at the GO instant, on the wall clock the sender used
    t_go = (time.time() - (time.monotonic() - trig.fired_t)) - t_wall0
    n, e = truth(t_go)
    r0 = trig.solution.belief_r0_ned
    assert abs(r0[0] - n) < 0.5 and abs(r0[1] - e) < 0.5, (r0, n, e)
