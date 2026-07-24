"""Pin the REAL interceptor's onboard flight state machine
(flight/deploy/real_flight.py): every transition, every guard, every failsafe.

WHY THESE TESTS EXIST: this is the code that will fly a physical, spinning-prop
aircraft. The state machine is deliberately a PURE step function -- it takes an
observation (which carries its own clock) and returns a decision -- so the whole
safety contract can be exercised on the desk with an injected fake vehicle, a
fake clock and a fake RC source. No MAVSDK, no sim, no camera, no hardware.

The four safety properties that matter most, each with its own test:
  * LINK-DENIED -> NO DASH (in STANDBY), and link loss AFTER the GO edge is
    IGNORED (that asymmetry IS the jam-resistance claim, constraint no-datalink)
  * DASH TIMEOUT -> SAFE, never a blind continue
  * the 5-consecutive-fresh-detection streak is the ONLY route into ENGAGE, and
    it matches the flown sim logic case-for-case
  * the dash heading is LATCHED ONCE at the GO edge and can never be re-written

Run: .venv/bin/python -m pytest flight/tests/test_real_flight.py -q
"""

import math
import os
import sys

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from flight.deploy.real_flight import (  # noqa: E402
    FakeVehicle,
    GateReadyTrigger,
    LatchOnce,
    LatchViolation,
    MissionConfig,
    PwmSwitch,
    RcChannelTrigger,
    RealFlightSM,
    ScriptedTrigger,
    Setpoint,
    State,
    TriggerState,
    VehicleObs,
    honesty_audit,
    latch_heading_from_bearing,
    resolve_preflight_heading,
    run_offline,
    update_acquire_streak,
    update_recede_streak,
    wrap_deg,
)
from flight.geometry import wrap_pi  # noqa: E402
from flight.guidance import dash_forward_speed, dash_loft_alt_ref  # noqa: E402

DT = 0.05                     # 20 Hz control loop, as flown
AIM = 40.0                    # the scenarios' pre-flight dash heading C1


def cfg(**kw) -> MissionConfig:
    """A MissionConfig whose arm gate is satisfied by `obs()` below."""
    base = dict(preflight_heading_deg=AIM, standby_alt_m=7.0,
                standby_settle_s=0.0, dash_speed_ms=10.0, dash_accel_ms2=10.0)
    base.update(kw)
    return MissionConfig(**base)


GO = TriggerState(go=True, link_ok=True, age_s=0.05, raw_us=1900)
NO = TriggerState(go=False, link_ok=True, age_s=0.05, raw_us=1100)
DEAD = TriggerState(go=False, link_ok=False, age_s=9.0, raw_us=None)
DEAD_HIGH = TriggerState(go=True, link_ok=False, age_s=9.0, raw_us=1900)


def obs(t, trigger=NO, **kw) -> VehicleObs:
    """An observation that SATISFIES the arm gate unless a field is overridden:
    armed, in OFFBOARD, at the standby altitude, holding the aim yaw."""
    base = dict(armed=True, offboard_active=True, alt_m=7.0, yaw_deg=AIM,
                quat=(1.0, 0.0, 0.0, 0.0))
    base.update(kw)
    return VehicleObs(t=t, trigger=trigger, **base)


def run(sm, n, t0=0.0, trigger=NO, **kw):
    """Step the machine n ticks at 20 Hz; returns the last Decision."""
    d = None
    for i in range(n):
        d = sm.step(obs(t0 + i * DT, trigger=trigger, **kw))
    return d


def dash_now(sm, t0=0.0, **kw):
    """Drive STANDBY -> CODED_DASH via a clean low->high GO edge.
    Returns the time of the GO tick. `kw` overrides observation fields (e.g. an
    alt_m matching a non-default standby altitude, which the arm gate checks)."""
    sm.step(obs(t0, trigger=NO, **kw))
    sm.step(obs(t0 + DT, trigger=GO, **kw))
    assert sm.state == State.CODED_DASH, f"GO did not dash: {sm.transitions}"
    return t0 + DT


# ============================================================ pure helpers


def test_wrap_deg_is_the_degree_twin_of_wrap_pi():
    """[-180, 180), same formula as flight.geometry.wrap_pi -- so +180 maps to
    -180 in BOTH, and a heading never disagrees between the two frames."""
    assert wrap_deg(0.0) == 0.0
    assert wrap_deg(180.0) == -180.0
    assert wrap_deg(-180.0) == -180.0
    for a in (-359.0, -180.0, -1.0, 0.0, 1.0, 179.0, 180.0, 361.0):
        assert wrap_deg(a) == pytest.approx(math.degrees(wrap_pi(math.radians(a))),
                                            abs=1e-9)
    assert wrap_deg(190.0) == pytest.approx(-170.0)
    assert wrap_deg(-190.0) == pytest.approx(170.0)
    assert wrap_deg(720.0 + 33.0) == pytest.approx(33.0)


def test_acquire_streak_contract():
    """The handoff streak: HOLD on no new result, +1 on a new in-window result,
    RESET on a new result that saw nothing."""
    assert update_acquire_streak(3, False, None) == 3        # no new result: HOLD
    assert update_acquire_streak(3, False, 9.0) == 3         # ... even with a range
    assert update_acquire_streak(3, True, 9.0) == 4          # new + detected: +1
    assert update_acquire_streak(3, True, None) == 0         # new + nothing: RESET
    # the (retired) plausibility window still behaves as m4 documents it
    assert update_acquire_streak(3, True, 1.5, 4.0, 30.0) == 0
    assert update_acquire_streak(3, True, 9.0, 4.0, 30.0) == 4


def test_acquire_streak_matches_the_flown_sim_logic():
    """PARITY: the real machine's handoff must behave EXACTLY like the flown
    scripts/m4_intercept.update_coded_dash_streak. Skipped (not failed) if the
    sim module's heavy gz/mavsdk imports are unavailable in this venv."""
    sys.path.insert(0, os.path.join(_REPO_ROOT, "scripts"))
    m4 = pytest.importorskip("m4_intercept",
                             reason="sim module needs gz-transport + mavsdk")
    cases = [
        (0, False, None, None, None), (0, True, 9.0, None, None),
        (4, True, 9.0, None, None), (4, True, None, None, None),
        (2, False, 9.0, None, None), (2, True, 1.5, 4.0, 30.0),
        (2, True, 9.0, 4.0, 30.0), (7, True, 31.0, 4.0, 30.0),
    ]
    for c in cases:
        assert update_acquire_streak(*c) == m4.update_coded_dash_streak(*c), c
    # and the streak LENGTH the real machine defaults to is the flown one
    assert MissionConfig().acquire_streak == m4.CODED_DASH_ACQUIRE_STREAK


def test_recede_streak_has_a_deadband():
    s, last = update_recede_streak(0, None, 3.0, 0.05)
    assert (s, last) == (0, 3.0)                  # first fresh range: seed only
    s, last = update_recede_streak(s, last, 3.5, 0.05)
    assert s == 1                                 # a real rise
    s2, last2 = update_recede_streak(s, last, 3.52, 0.05)
    assert (s2, last2) == (1, 3.5)                # within-deadband creep: HOLD
    s3, _ = update_recede_streak(s2, last2, 2.0, 0.05)
    assert s3 == 0                                # a genuine drop: RESET


def test_preflight_heading_is_accel_aware_by_default():
    """The accel-aware solve must LEAD FURTHER than the constant-speed one for a
    crossing target, because the real dash starts at rest (ADR-0080)."""
    tstart, tvel, v = (0.0, 16.7), (9.0, 0.0), 16.0
    h_acc, t_acc = resolve_preflight_heading(tstart, tvel, v, 10.0, accel_aware=True)
    h_const, _ = resolve_preflight_heading(tstart, tvel, v, 10.0, accel_aware=False)
    assert h_acc > h_const                     # more lead into the crossing
    assert t_acc is not None and t_acc > 0.0
    # a huge acceleration collapses onto the constant-speed answer
    h_inf, _ = resolve_preflight_heading(tstart, tvel, v, 1e8, accel_aware=True)
    assert h_inf == pytest.approx(h_const, abs=1e-4)   # bisection resolution


def test_bearing_latch_composes_yaw_bearing_and_lead():
    assert latch_heading_from_bearing(10.0, 30.0, 0.0) == pytest.approx(40.0)
    assert latch_heading_from_bearing(-5.0, 30.0, 20.0) == pytest.approx(45.0)
    assert latch_heading_from_bearing(20.0, 175.0, 0.0) == pytest.approx(-165.0)


def test_pwm_switch_has_hysteresis():
    sw = PwmSwitch(high_us=1700, low_us=1300)
    assert sw.update(1000) is False
    assert sw.update(1500) is False        # dead band holds the previous state
    assert sw.update(1800) is True
    assert sw.update(1500) is True         # ... in both directions
    assert sw.update(1200) is False
    assert sw.update(None) is False        # a dropped frame never flips it


def test_rc_trigger_reports_link_loss_by_staleness():
    trg = RcChannelTrigger(channel=7, link_timeout_s=1.0)
    assert trg.poll(0.0).link_ok is False              # nothing received yet
    trg.ingest({"chan7_raw": 1900}, t=10.0)
    st = trg.poll(10.1)
    assert st.link_ok is True and st.go is True and st.raw_us == 1900
    stale = trg.poll(12.0)
    assert stale.link_ok is False
    assert stale.go is False, "a stale link must not report GO"


# ============================================================ latch-once


def test_latch_once_raises_on_a_second_write():
    """The negative test the flight module cannot contain: a second `.set(` there
    would be a second STATIC write site and would fail --audit."""
    cell = LatchOnce("dash_heading_deg")
    assert cell.is_set is False
    cell.set(42.0, t=1.0)
    assert cell.value == 42.0 and cell.t_set == 1.0
    with pytest.raises(LatchViolation):
        cell.set(43.0, t=2.0)
    assert cell.value == 42.0, "a rejected write must not corrupt the latch"


def test_dash_heading_is_latched_at_the_go_edge_and_never_again():
    sm = RealFlightSM(cfg())
    assert sm.dash_heading.is_set is False
    t_go = dash_now(sm)
    assert sm.state == State.CODED_DASH
    assert sm.dash_heading.value == pytest.approx(AIM)
    assert sm.dash_heading.t_set == pytest.approx(t_go)
    # 200 further ticks through dash/engage must not touch it
    run(sm, 200, t0=t_go + DT, det_new=True, det_range_m=9.0)
    assert sm.dash_heading.t_set == pytest.approx(t_go)
    with pytest.raises(LatchViolation):
        sm.dash_heading.set(999.0, t=99.0)


def test_honesty_audit_passes_on_the_flight_module():
    assert honesty_audit(verbose=False) == 0


# ============================================================ STANDBY guards


def test_standby_holds_position_altitude_and_the_aim_yaw():
    sm = RealFlightSM(cfg())
    d = sm.step(obs(0.0, alt_m=7.6))          # 0.6 m high
    assert d.state == State.STANDBY
    assert d.setpoint.v_north == 0.0 and d.setpoint.v_east == 0.0
    assert d.setpoint.v_down > 0.0, "high -> commanded DOWN (NED down positive)"
    assert d.setpoint.yaw_deg == pytest.approx(AIM)


def test_a_go_edge_with_every_guard_satisfied_dashes():
    sm = RealFlightSM(cfg())
    dash_now(sm)
    assert sm.state == State.CODED_DASH
    assert [t.reason for t in sm.transitions] == ["go_edge"]


@pytest.mark.parametrize("bad,expect", [
    (dict(armed=False), "not_armed"),
    (dict(offboard_active=False), "not_offboard"),
    (dict(alt_m=5.0), "alt_off"),
    (dict(yaw_deg=AIM + 25.0), "yaw_off"),
    (dict(alt_m=None), "no_altitude"),
    (dict(yaw_deg=None), "no_yaw"),
])
def test_a_go_with_a_failed_arm_gate_aborts_and_does_not_dash(bad, expect):
    """docs/launch_mechanism_plan.md Sec 4: the Pi REFUSES to dash unless armed,
    in OFFBOARD, at the standby altitude +-0.3 m and holding C1 +-2 deg -- and a
    GO in that state ABORTS loudly rather than dashing or silently ignoring."""
    sm = RealFlightSM(cfg())
    sm.step(obs(0.0, trigger=NO, **bad))
    sm.step(obs(DT, trigger=GO, **bad))
    assert sm.state == State.SAFE
    assert sm.safe_reason == "arm_gate_failed_at_go"
    assert expect in sm.transitions[-1].reason
    assert sm.dash_heading.is_set is False, "an aborted GO must not latch a heading"


def test_the_settle_time_is_part_of_the_gate():
    sm = RealFlightSM(cfg(standby_settle_s=2.0))
    sm.step(obs(0.0, trigger=NO))
    sm.step(obs(DT, trigger=GO))              # only 0.05 s settled
    assert sm.state == State.SAFE and "not_settled" in sm.transitions[-1].reason


def test_a_switch_already_high_at_boot_is_never_a_go():
    """Powering up with the aux switch left ON must not fire the dash. It must be
    seen LOW first, then go HIGH."""
    sm = RealFlightSM(cfg())
    for i in range(20):
        sm.step(obs(i * DT, trigger=GO))
    assert sm.state == State.STANDBY, "no rising edge was ever presented"
    assert sm.dash_heading.is_set is False
    sm.step(obs(20 * DT, trigger=NO))          # now the switch is seen LOW
    sm.step(obs(21 * DT, trigger=GO))          # ... and a real edge fires
    assert sm.state == State.CODED_DASH


def test_a_held_high_switch_does_not_re_trigger():
    sm = RealFlightSM(cfg())
    dash_now(sm)
    n_before = len(sm.transitions)
    run(sm, 20, t0=2 * DT, trigger=GO)         # switch stays high through the dash
    assert len([t for t in sm.transitions if t.reason == "go_edge"]) == 1
    assert len(sm.transitions) >= n_before


# ============================================================ LINK failsafe


def test_link_loss_in_standby_refuses_to_dash_and_goes_safe():
    """THE mandatory link-denied failsafe: if the RC link is gone, do NOT dash."""
    sm = RealFlightSM(cfg())
    sm.step(obs(0.0, trigger=NO))
    d = sm.step(obs(DT, trigger=DEAD))
    assert sm.state == State.SAFE
    assert sm.safe_reason == "link_denied:link_lost"
    assert d.setpoint.v_north == 0.0 and d.setpoint.v_east == 0.0
    assert d.land_requested is True
    assert sm.dash_heading.is_set is False


def test_a_go_arriving_on_a_dead_link_cannot_fire_the_dash():
    sm = RealFlightSM(cfg())
    sm.step(obs(0.0, trigger=NO))
    sm.step(obs(DT, trigger=DEAD_HIGH))       # switch high BUT the link is stale
    assert sm.state == State.SAFE
    assert sm.safe_reason.startswith("link_denied")


def test_a_link_that_never_starts_aborts_after_the_grace_window():
    """A never-seen RC stream is tolerated briefly (MAVLink subscriptions take a
    moment to start) and then aborts -- it must not abort on tick 0."""
    never = TriggerState(go=False, link_ok=False, age_s=None)
    sm = RealFlightSM(cfg(link_acquire_grace_s=3.0))
    sm.step(obs(0.0, trigger=never))
    assert sm.state == State.STANDBY, "must not abort before the grace window"
    run(sm, 80, t0=DT, trigger=never)          # 4 s
    assert sm.state == State.SAFE
    assert sm.safe_reason == "link_denied:link_never_acquired"


def test_link_loss_AFTER_the_go_edge_is_ignored():
    """THE JAM-RESISTANCE DELIVERABLE (constraint no-datalink, flight arm L11):
    kill the TX right after the GO edge and the dash + terminal must complete
    unaided. A failsafe that aborted on post-GO link loss would delete the
    project's headline capability, so this asymmetry is tested explicitly."""
    sm = RealFlightSM(cfg(dash_max_s=30.0))
    dash_now(sm)
    run(sm, 100, t0=2 * DT, trigger=DEAD)      # 5 s with the link fully dead
    assert sm.state == State.CODED_DASH
    # ... and it still hands off to the camera terminal with no link at all
    for i in range(5):
        sm.step(obs(5.2 + i * DT, trigger=DEAD, det_new=True, det_range_m=9.0))
    assert sm.state == State.ENGAGE


# ============================================================ CODED_DASH


def test_the_dash_setpoint_is_the_portable_guidance_functions():
    """The dash must BE flight.guidance, not a re-implementation: velocity is the
    ramp function resolved onto the latched heading, and yaw is that heading."""
    c = cfg(dash_speed_ms=16.0, dash_accel_cap_ms2=4.0, dash_loft_m=2.0,
            dash_loft_dive_s=2.0, standby_alt_m=9.0)
    sm = RealFlightSM(c)
    t_go = dash_now(sm, alt_m=9.0)      # gate: must be AT the standby altitude
    for k in range(1, 20):
        t = t_go + k * DT
        d = sm.step(obs(t, alt_m=9.0))
        elapsed = t - t_go
        v = dash_forward_speed(c.dash_speed_ms, c.dash_accel_cap_ms2, elapsed)
        h = math.radians(AIM)
        assert d.setpoint.v_north == pytest.approx(v * math.cos(h))
        assert d.setpoint.v_east == pytest.approx(v * math.sin(h))
        assert d.setpoint.yaw_deg == pytest.approx(AIM)
        # the loft dive reference is the guidance function, keyed to dash entry
        ref = dash_loft_alt_ref(c.dash_base_alt_m, c.dash_loft_m, elapsed,
                                c.dash_loft_dive_s)
        assert d.setpoint.v_down == pytest.approx(
            max(-c.v_vert_max_ms, min(c.v_vert_max_ms, c.kp_alt * (9.0 - ref))))


def test_the_loft_dive_starts_at_the_standby_altitude_and_ends_co_altitude():
    c = cfg(standby_alt_m=9.0, dash_loft_m=2.0, dash_loft_dive_s=2.0)
    assert c.dash_base_alt_m == pytest.approx(7.0)
    assert dash_loft_alt_ref(c.dash_base_alt_m, c.dash_loft_m, 0.0,
                             c.dash_loft_dive_s) == pytest.approx(9.0)
    assert dash_loft_alt_ref(c.dash_base_alt_m, c.dash_loft_m, 2.0,
                             c.dash_loft_dive_s) == pytest.approx(7.0)


def test_five_consecutive_fresh_detections_hand_off_to_engage():
    sm = RealFlightSM(cfg())
    t = dash_now(sm)
    for i in range(4):
        t += DT
        sm.step(obs(t, det_new=True, det_range_m=9.0))
        assert sm.state == State.CODED_DASH, f"handed off early at {i + 1}"
    t += DT
    d = sm.step(obs(t, det_new=True, det_range_m=9.0))
    assert sm.state == State.ENGAGE and d.streak == 5
    assert sm.transitions[-1].reason == "acquire_streak>=5"


def test_the_handoff_tick_still_flies_the_dash_command():
    """Ordering mirrors the sim: the tick that fires the handoff still sends the
    DASH setpoint; the terminal takes over on the NEXT tick."""
    sm = RealFlightSM(cfg())
    t = dash_now(sm)
    for i in range(5):
        t += DT
        d = sm.step(obs(t, det_new=True, det_range_m=9.0))
    assert sm.state == State.ENGAGE
    assert d.setpoint.yaw_deg == pytest.approx(AIM), "handoff tick flew the dash"


def test_a_detection_gap_resets_the_streak_but_a_slow_detector_does_not():
    sm = RealFlightSM(cfg())
    t = dash_now(sm)
    for i in range(4):
        t += DT
        sm.step(obs(t, det_new=True, det_range_m=9.0))
    # a NEW frame that saw nothing RESETS
    t += DT
    d = sm.step(obs(t, det_new=True, det_range_m=None))
    assert d.streak == 0 and sm.state == State.CODED_DASH
    # ... whereas ticks with NO new detector result merely HOLD
    for i in range(3):
        t += DT
        sm.step(obs(t, det_new=True, det_range_m=9.0))
    for i in range(10):
        t += DT
        d = sm.step(obs(t, det_new=False))
    assert d.streak == 3 and sm.state == State.CODED_DASH


def test_dash_timeout_with_no_acquire_goes_safe_not_a_blind_continue():
    sm = RealFlightSM(cfg(dash_max_s=1.0))
    t = dash_now(sm)
    d = run(sm, 60, t0=t + DT)
    assert sm.state == State.SAFE
    assert sm.safe_reason == "dash_timeout_no_acquire"
    assert d.setpoint.v_north == 0.0 and d.setpoint.v_east == 0.0, \
        "SAFE must STOP, not keep flying open-loop"
    assert d.land_requested is True


def test_the_optional_dead_reckoned_distance_bound_fires():
    sm = RealFlightSM(cfg(dash_speed_ms=10.0, dash_accel_ms2=10.0,
                          dash_max_s=60.0, dash_max_dist_m=20.0))
    t = dash_now(sm)
    run(sm, 200, t0=t + DT)
    assert sm.state == State.SAFE
    assert sm.safe_reason == "dash_distance_bound"


# ============================================================ ENGAGE / BREAKOFF


def engaged(c=None):
    """A machine parked in ENGAGE with no seeker attached (so ENGAGE flies the
    open-loop coast fallback and the terminal's own math stays out of the way)."""
    sm = RealFlightSM(c or cfg())
    t = dash_now(sm)
    for i in range(5):
        t += DT
        sm.step(obs(t, det_new=True, det_range_m=9.0))
    assert sm.state == State.ENGAGE
    return sm, t


def test_engage_without_a_seeker_lock_coasts_on_the_dash_velocity():
    """The setpoint stream must never gap (PX4 drops OFFBOARD), and hovering
    would throw away the closing speed the dash just built."""
    sm, t = engaged()
    d = sm.step(obs(t + DT, det_new=False))
    assert d.state == State.ENGAGE
    assert math.hypot(d.setpoint.v_north, d.setpoint.v_east) > 1.0


def test_engage_timeout_breaks_off_then_reaches_safe():
    sm, t = engaged(cfg(engage_max_s=1.0, breakoff_s=0.5))
    run(sm, 80, t0=t + DT, det_new=True, det_range_m=9.0)
    reasons = [tr.reason for tr in sm.transitions]
    assert any(r.startswith("engage_timeout") for r in reasons)
    assert sm.state == State.SAFE and sm.safe_reason == "breakoff_complete"


def test_a_lost_target_breaks_off():
    sm, t = engaged(cfg(engage_lost_target_s=0.5, engage_max_s=60.0))
    run(sm, 40, t0=t + DT, det_new=False)
    assert any(r.startswith("target_lost") for r in [x.reason for x in sm.transitions])
    assert sm.state in (State.BREAKOFF, State.SAFE)


def test_the_hard_range_floor_breaks_off_immediately():
    sm, t = engaged(cfg(breakoff_hard_floor_m=0.5))
    d = sm.step(obs(t + DT, det_new=True, det_range_m=0.4))
    assert d.state == State.BREAKOFF
    assert "hard_floor" in sm.transitions[-1].reason
    assert d.setpoint.v_down < 0.0, "breakoff climbs away (NED down negative = up)"


def test_a_receding_range_past_cpa_breaks_off():
    sm, t = engaged(cfg(breakoff_arm_range_m=4.0, breakoff_range_increases=3,
                        engage_max_s=60.0))
    for r in (3.0, 2.0, 1.0):                      # closing: arms the logic
        t += DT
        sm.step(obs(t, det_new=True, det_range_m=r))
    assert sm.state == State.ENGAGE
    for r in (1.5, 2.2, 3.1):                      # receding past CPA
        t += DT
        sm.step(obs(t, det_new=True, det_range_m=r))
    assert sm.state == State.BREAKOFF
    assert "past_cpa" in sm.transitions[-1].reason


def test_breakoff_does_not_arm_before_the_arm_range():
    """A range that only ever INCREASES far away (e.g. a bad early estimate) must
    not fire the past-CPA breakoff."""
    sm, t = engaged(cfg(breakoff_arm_range_m=4.0, engage_max_s=60.0))
    for r in (9.0, 10.0, 11.0, 12.0, 13.0):
        t += DT
        sm.step(obs(t, det_new=True, det_range_m=r))
    assert sm.state == State.ENGAGE


# ============================================================ SAFE / backstops


def test_safe_is_absorbing_and_terminates_after_the_hold():
    sm = RealFlightSM(cfg(safe_hold_s=0.5))
    sm.step(obs(0.0, trigger=NO))
    sm.step(obs(DT, trigger=DEAD))
    assert sm.state == State.SAFE
    d = run(sm, 40, t0=2 * DT, trigger=GO, det_new=True, det_range_m=9.0)
    assert sm.state == State.SAFE, "nothing may leave SAFE -- not even a new GO"
    assert d.terminated is True and d.land_requested is True
    assert d.setpoint.v_north == 0.0 and d.setpoint.v_east == 0.0


def test_standby_timeout_is_a_battery_failsafe():
    sm = RealFlightSM(cfg(standby_max_s=1.0))
    run(sm, 60, t0=0.0, trigger=NO)
    assert sm.state == State.SAFE and sm.safe_reason == "standby_timeout"


def test_the_mission_backstop_catches_any_state():
    sm = RealFlightSM(cfg(mission_max_s=1.0, dash_max_s=60.0,
                          standby_max_s=60.0))
    t = dash_now(sm)
    run(sm, 60, t0=t + DT)
    assert sm.state == State.SAFE and sm.safe_reason == "mission_timeout"


def test_every_tick_in_every_state_emits_a_setpoint():
    """PX4 fails out of OFFBOARD if the setpoint stream gaps for ~0.5 s, so a
    None setpoint anywhere in this machine is a flight-safety bug."""
    sm = RealFlightSM(cfg(engage_max_s=1.0, breakoff_s=0.4, safe_hold_s=0.4))
    seen = set()
    t = 0.0
    trig = NO
    for i in range(400):
        d = sm.step(obs(t, trigger=trig, det_new=(i % 2 == 0),
                        det_range_m=max(0.3, 9.0 - 0.2 * i)))
        assert isinstance(d.setpoint, Setpoint), f"no setpoint in {d.state}"
        assert all(math.isfinite(v) for v in d.setpoint.as_tuple())
        seen.add(d.state)
        trig = GO if i == 0 else NO
        t += DT
        if d.terminated:
            break
    assert seen == {State.STANDBY, State.CODED_DASH, State.ENGAGE,
                    State.BREAKOFF, State.SAFE}


def test_transitions_are_all_logged_with_a_reason():
    sm = RealFlightSM(cfg(engage_max_s=0.5, breakoff_s=0.3))
    t = dash_now(sm)
    for i in range(5):
        t += DT
        sm.step(obs(t, det_new=True, det_range_m=9.0))
    run(sm, 60, t0=t + DT, det_new=False)
    assert [x.frm for x in sm.transitions][0] == State.STANDBY
    for tr in sm.transitions:
        assert tr.reason and isinstance(tr.reason, str)
        assert tr.frm in (State.STANDBY, State.CODED_DASH, State.ENGAGE,
                          State.BREAKOFF)
        assert tr.t >= 0.0
    assert sm.visited[-1] == State.SAFE


# ============================================================ offline driver


def test_the_offline_dry_run_walks_the_whole_path():
    """The --dry-run harness (fake vehicle + gate-ready trigger + stub seeker) is
    itself pinned, so the head can trust its exit code without a sim."""
    sm, rows = run_offline(cfg(engage_max_s=3.0, breakoff_s=0.5, safe_hold_s=0.5),
                           GateReadyTrigger(), guidance=None, fps=20.0,
                           max_s=60.0, verbose=False)
    assert sm.visited[0] == State.STANDBY
    assert State.CODED_DASH in sm.visited
    assert State.ENGAGE in sm.visited
    assert sm.visited[-1] == State.SAFE
    assert sm.dash_heading.is_set
    assert len(rows) > 50 and rows[0].count(",") == 17


def test_the_scripted_trigger_can_model_a_dying_link():
    trg = ScriptedTrigger(go_at_s=1.0, link_dies_at_s=2.0)
    assert trg.poll(0.5).go is False and trg.poll(0.5).link_ok is True
    assert trg.poll(1.5).go is True and trg.poll(1.5).link_ok is True
    assert trg.poll(2.5).link_ok is False


def test_the_fake_vehicle_integrates_the_commanded_setpoint():
    veh = FakeVehicle(alt_m=5.0, yaw_deg=0.0, yaw_rate_deg_s=90.0)
    veh.apply(Setpoint(0.0, 0.0, -1.0, 45.0), dt=1.0)
    assert veh.alt_m == pytest.approx(6.0)      # NED: v_down -1 climbs
    assert veh.yaw_deg == pytest.approx(45.0)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
