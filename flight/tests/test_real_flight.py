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
    _CSV_FIELDS,
    _CSV_HEADER,
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
    StepTelemetry,
    TriggerState,
    VehicleObs,
    audited_module_paths,
    honesty_audit,
    is_offboard_mode,
    latch_heading_from_bearing,
    make_mode_subscriber,
    resolve_preflight_heading,
    run_offline,
    trigger_poll_time,
    update_acquire_streak,
    update_recede_streak,
    wrap_deg,
)
from flight.geometry import wrap_pi  # noqa: E402
from flight.guidance import (  # noqa: E402
    dash_forward_speed,
    dash_loft_alt_ref,
    dash_ramp_distance,
)

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
    # Row/header SEAM: pin the shape against the PRODUCER's own header, not a
    # magic number, so a new column can never silently mis-align the CSV.
    assert len(rows) > 50
    assert all(r.count(",") == _CSV_HEADER.count(",") for r in rows)
    assert _CSV_HEADER.strip().split(",") == list(_CSV_FIELDS)


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


# ==================================================== review2 silent-failure fixes
# Each test below pins a path that produced a confident, plausible, WRONG result
# with nothing raised. All are offline: fake vehicle, fake drone, injected clock.


# ---- OFFBOARD: a latched constant is not a measurement ------------------------


def test_is_offboard_mode_reads_a_flight_mode_sample_not_a_latch():
    """The producer half. `offboard_active` used to be written once, right after
    `offboard.start()` succeeded, and never again -- so `arm_gate`'s OFFBOARD
    check could not fail in flight by construction. Name-compare so it works
    against MAVSDK's enum, a bare string, or a repr."""
    class _FM:
        def __init__(self, n): self.name = n
        def __str__(self): return f"FlightMode.{self.name}"
    assert is_offboard_mode(_FM("OFFBOARD")) is True
    assert is_offboard_mode(_FM("POSCTL")) is False
    assert is_offboard_mode(_FM("HOLD")) is False
    assert is_offboard_mode("OFFBOARD") is True
    assert is_offboard_mode("FlightMode.OFFBOARD") is True
    assert is_offboard_mode("FlightMode.LAND") is False
    # No sample yet MUST be None, not False and certainly not True: the gate
    # tests `is not True`, so None fails CLOSED.
    assert is_offboard_mode(None) is None


def test_the_mode_subscriber_drives_the_state_dict_and_the_gate_follows_it():
    """THE SEAM (the thing that had no test): drive the real subscriber with a
    fake drone that reports OFFBOARD then POSCTL, and assert the VehicleObs the
    state machine sees flips -- and that a GO is then refused.

    scripts/check_real_flight_sitl.sh structurally CANNOT catch this: SITL never
    leaves OFFBOARD. It has to be an offline seam test."""
    import asyncio

    class FakeTelemetry:
        def __init__(self, modes): self._modes = modes
        async def flight_mode(self):
            for m in self._modes:
                yield m

    class FakeDrone:
        def __init__(self, modes): self.telemetry = FakeTelemetry(modes)

    state = {"offboard": None, "mode": None}
    # Before the first sample the gate must FAIL (fail-closed), not pass.
    sm = RealFlightSM(cfg())
    assert sm.arm_gate(obs(0.0, offboard_active=state["offboard"]))[0] is False

    asyncio.run(make_mode_subscriber(FakeDrone(["OFFBOARD"]), state)())
    assert state["offboard"] is True and state["mode"] == "OFFBOARD"
    assert sm.arm_gate(obs(0.0, offboard_active=state["offboard"]))[0] is True

    asyncio.run(make_mode_subscriber(FakeDrone(["OFFBOARD", "POSCTL"]), state)())
    assert state["offboard"] is False, "PX4 leaving OFFBOARD must be visible"
    ok, fails = sm.arm_gate(obs(0.0, offboard_active=state["offboard"]))
    assert not ok and any("not_offboard" in f for f in fails)

    # ...and a GO on that observation ABORTS instead of dashing.
    sm2 = RealFlightSM(cfg())
    sm2.step(obs(0.0, trigger=NO, offboard_active=False))
    sm2.step(obs(DT, trigger=GO, offboard_active=False))
    assert sm2.state == State.SAFE and sm2.dash_heading.is_set is False


# ---- RC trigger clock base ----------------------------------------------------


def test_the_rc_trigger_link_failsafe_survives_the_drivers_own_clock():
    """THE BLOCKER. The live driver stamped `ingest()` with absolute
    time.monotonic() but polled with `time.monotonic() - t0`, so `age` was ~-t0
    (a large negative), `link_ok` was pinned True for the whole flight, and the
    per-tick CSV asserted `trig_link=1, trig_age_s=-24580.70`. FAILSAFE 1
    (link-denied -> SAFE) could never fire.

    Drive ingest+poll through the DRIVER's own expressions, with a realistic
    ~1e4 s uptime offset. The existing unit test at :189 is clock-base-consistent
    and structurally cannot catch this."""
    boot = 24_580.0                     # "seconds since boot", i.e. real uptime
    t0 = boot + 12.0                    # mission start, as the driver sets it
    trg = RcChannelTrigger(channel=7, link_timeout_s=1.0)

    def driver_poll(t_abs):
        """EXACTLY what run_mavsdk_mission does, via the shared seam helper."""
        return trg.poll(trigger_poll_time(trg, t_mission=t_abs - t0, t_abs=t_abs))

    # A live link: frames arriving, GO honoured.
    trg.ingest({"chan7_raw": 1900}, boot + 20.0)          # absolute, as the driver
    st = driver_poll(boot + 20.2)
    assert st.link_ok is True and st.go is True and st.clock_fault is False
    assert st.age_s == pytest.approx(0.2)

    # The TX dies. Within link_timeout_s the link MUST read denied.
    dead = driver_poll(boot + 21.5)
    assert dead.link_ok is False, "the link-denied failsafe is dead again"
    assert dead.go is False
    assert dead.age_s > trg.link_timeout_s

    # ...and the state machine actually transitions on it.
    sm = RealFlightSM(cfg())
    sm.step(obs(0.0, trigger=driver_poll(boot + 20.2)))
    sm.step(obs(DT, trigger=dead))
    assert sm.state == State.SAFE and sm.safe_reason.startswith("link_denied")


def test_a_negative_rc_age_is_a_hard_link_failure_not_a_healthy_link():
    """Belt-and-braces for the whole defect CLASS: if two call sites ever
    disagree about the clock again, a physically impossible negative staleness
    must read DENIED and say so, not flatter the log with trig_link=1."""
    trg = RcChannelTrigger(channel=7, link_timeout_s=1.0)
    trg.ingest({"chan7_raw": 1900}, t=24_580.0)      # absolute
    st = trg.poll(12.0)                              # elapsed -- the old mismatch
    assert st.age_s < 0
    assert st.link_ok is False and st.go is False and st.clock_fault is True
    assert trg.n_clock_faults == 1


def test_the_clock_base_seam_routes_each_trigger_to_its_own_clock():
    """The offline triggers keep MISSION time (go_at_s is defined against mission
    start); only the MAVLink-fed one wants absolute monotonic."""
    assert trigger_poll_time(RcChannelTrigger(), t_mission=5.0, t_abs=9000.0) == 9000.0
    assert trigger_poll_time(ScriptedTrigger(go_at_s=1.0), 5.0, 9000.0) == 5.0
    assert trigger_poll_time(GateReadyTrigger(), 5.0, 9000.0) == 5.0
    # ScriptedTrigger semantics are unchanged by the routing.
    trg = ScriptedTrigger(go_at_s=1.0, link_dies_at_s=2.0)
    assert trg.poll(trigger_poll_time(trg, 1.5, 9000.0)).go is True
    assert trg.poll(trigger_poll_time(trg, 2.5, 9000.0)).link_ok is False


# ---- the honesty audit --------------------------------------------------------


def test_the_honesty_audit_covers_the_whole_real_build_import_closure():
    """It used to audit `__file__` only, so seeker_loop.py -- the module that
    actually produces det_range_m -- was covered by NO AST honesty check at all,
    while the PASS line claimed "inputs by construction"."""
    covered = {os.path.relpath(p, _REPO_ROOT) for p in audited_module_paths()}
    for must in ("flight/deploy/real_flight.py", "flight/deploy/seeker_loop.py",
                 "flight/geometry.py", "flight/guidance.py", "flight/camera.py",
                 "flight/estimator.py"):
        assert must in covered, f"{must} is not in the audited closure"
    assert all(os.path.exists(p) for p in audited_module_paths())
    assert honesty_audit(verbose=False) == 0


@pytest.mark.parametrize("injection,why", [
    ("        gt_range = obs.det_range_m\n", "the gt_ prefix (already caught)"),
    ("        _r = ground_truth_world_points()[0]\n",
     "the repo's REAL accessor name -- passed the one-prefix guard"),
    ("        _r = self._tracker.ground_truth_rel_optical()\n",
     "the attribute form of the real accessor"),
    ("        _r = obs.ground_truth_range\n", "a ground-truth attribute read"),
    ("from m3_static_intercept import ground_truth_world_points\n",
     "a sim-module import that would drag truth in under a clean name"),
    ("from m4_intercept import compute_v_close\n",
     "a sim-module import at all"),
])
def test_injected_ground_truth_reads_fail_the_audit(tmp_path, injection, why):
    """MUTATION CALIBRATION -- the test the guard never had. Without it the fix
    is unverified in exactly the way the audit was: `_FORBIDDEN_ID_PREFIXES =
    ("gt_",)` did not include `ground_truth`, which is the name of the repo's
    real accessors (m3_static_intercept.ground_truth_world_points,
    PoseTracker.ground_truth_rel_optical), so a planted read of the REAL thing
    printed "[PASS] no ground-truth identifier is read anywhere (found 0)"."""
    src = open(os.path.join(_REPO_ROOT, "flight", "deploy", "real_flight.py")).read()
    assert honesty_audit(path=os.path.join(_REPO_ROOT, "flight", "deploy",
                                           "real_flight.py"), verbose=False) == 0
    anchor = "    def link_status(self, obs: VehicleObs) -> Tuple[bool, str]:\n"
    assert anchor in src
    if injection.startswith("from "):
        mutated = injection + src
    else:
        mutated = src.replace(anchor, anchor + '        """x"""\n' + injection, 1)
    p = tmp_path / "mutant.py"
    p.write_text(mutated)
    assert honesty_audit(path=str(p), verbose=False) == 1, (
        f"the audit did NOT catch {why}")


def test_a_second_latch_write_still_fails_the_audit(tmp_path):
    """The latch arm must survive the multi-file refactor."""
    src = open(os.path.join(_REPO_ROOT, "flight", "deploy", "real_flight.py")).read()
    anchor = "            self.t_dash_start = obs.t\n"
    assert anchor in src
    p = tmp_path / "mutant.py"
    p.write_text(src.replace(
        anchor, "            self.dash_heading.set(1.0, t=obs.t)\n" + anchor, 1))
    assert honesty_audit(path=str(p), verbose=False) == 1


# ---- arm gate: the attitude quaternion ---------------------------------------


def test_the_arm_gate_refuses_to_dash_without_an_attitude_quaternion():
    """The terminal's whole LOS chain hangs on the quat; without it seeker_loop
    reverts to the pre-fix yaw-only formula on an assumed-level vehicle. The gate
    already refused on `no_yaw`; quat is the same own-state EKF."""
    sm = RealFlightSM(cfg())
    ok, fails = sm.arm_gate(obs(0.0, quat=None))
    assert not ok and "no_attitude_quat" in fails
    sm.step(obs(0.0, trigger=NO, quat=None))
    sm.step(obs(DT, trigger=GO, quat=None))
    assert sm.state == State.SAFE
    assert "no_attitude_quat" in sm.transitions[-1].reason
    assert sm.dash_heading.is_set is False


# ---- FAILSAFE 8: a diverged range channel aborts, it does not coast ----------


def test_a_persistently_broken_range_track_breaks_off_instead_of_coasting():
    """Pre-fix, r_hat <= 0 / a non-physical rdot silently selected the
    terminal-coast branch and flew a frozen vector to the end of the engagement.
    Now the terminal FLAGS it and the state machine breaks off, loudly."""
    class BrokenGuidance:
        """Stand-in terminal that reports a diverged range channel."""
        def __init__(self): self.n = 0
        def step(self, box, own, t):
            self.n += 1
            tel = StepTelemetry(t=t, detected=box is not None)
            tel.track_broken = True
            tel.health.append("range_track_broken")
            return Setpoint(1.0, 0.0, 0.0, 0.0), tel

    c = cfg(engage_track_broken_ticks=3)
    sm = RealFlightSM(c, guidance=BrokenGuidance())
    t = dash_now(sm)
    for i in range(5):
        sm.step(obs(t + DT * (i + 1), det_new=True, det_range_m=9.0))
    assert sm.state == State.ENGAGE, f"setup: {sm.transitions}"
    for i in range(3):
        sm.step(obs(t + DT * (6 + i), det_new=True, det_range_m=9.0))
    assert sm.state == State.BREAKOFF
    assert "range_track_broken" in sm.transitions[-1].reason
    # One isolated broken tick must NOT end an engagement.
    sm2 = RealFlightSM(cfg(), guidance=None)
    t2 = dash_now(sm2)
    for i in range(5):
        sm2.step(obs(t2 + DT * (i + 1), det_new=True, det_range_m=9.0))
    assert sm2.state == State.ENGAGE


def test_failsafe_8_does_not_abort_a_legitimate_terminal_coast():
    """Inside the armed freeze the range estimate is SUPPOSED to run through
    zero -- that is the coast flying through CPA (ADR-0023, and un-freezing it
    is graveyarded). FAILSAFE 8 must not delete the validated mechanism."""
    class CoastingGuidance:
        def step(self, box, own, t):
            tel = StepTelemetry(t=t, detected=box is not None)
            tel.terminal_coast = True
            tel.r_hat_m = -1.5           # past CPA, as a real coast reads
            tel.track_broken = True
            tel.health.append("range_track_broken")
            return Setpoint(9.0, 0.0, 0.0, 0.0), tel

    sm = RealFlightSM(cfg(engage_track_broken_ticks=3, engage_max_s=30.0,
                          engage_lost_target_s=30.0),
                      guidance=CoastingGuidance())
    t = dash_now(sm)
    for i in range(5):
        sm.step(obs(t + DT * (i + 1), det_new=True, det_range_m=9.0))
    assert sm.state == State.ENGAGE
    for i in range(20):
        sm.step(obs(t + DT * (6 + i), det_new=True, det_range_m=9.0))
    assert sm.state == State.ENGAGE, (
        f"FAILSAFE 8 aborted an armed terminal coast: {sm.transitions}")


# ---- FAILSAFE 4: a modelled metre is not a flown metre ------------------------


def test_the_distance_bound_prefers_the_measured_ground_speed():
    """`dash_accel_ms2 = 10.0` is a MISS-MAE AIM fit, not a kinematic
    acceleration: ADR-0083 measured it predicting 22.2 m of travel where the
    vehicle covered 8.2 m. Dead-reckoning the distance failsafe off it aborts the
    dash at roughly a third of the real distance AND writes a fabricated number
    into the log. Integrate the EKF ground speed when the vehicle reports it."""
    # Same 20 m bound, same 2 s of flight, two very different truths.
    fast = RealFlightSM(cfg(dash_max_s=60.0, dash_max_dist_m=20.0))
    t = dash_now(fast, ground_speed_ms=0.0)
    for i in range(1, 200):
        fast.step(obs(t + DT * i, ground_speed_ms=16.0))
        if fast.state == State.SAFE:
            break
    assert fast.safe_reason == "dash_distance_bound"
    assert "EKF" in fast.transitions[-1].reason
    # 20 m at 16 m/s is ~1.25 s, so it must NOT trip before ~1.2 s of dash.
    assert (fast.transitions[-1].t - t) == pytest.approx(20.0 / 16.0, abs=0.15)

    # A vehicle that is barely moving must NOT trip the bound at all...
    slow = RealFlightSM(cfg(dash_max_s=60.0, dash_max_dist_m=20.0))
    t = dash_now(slow, ground_speed_ms=0.0)
    for i in range(1, 60):                      # 3 s at 1 m/s = 3 m flown
        slow.step(obs(t + DT * i, ground_speed_ms=1.0))
    assert slow.state == State.CODED_DASH
    # ...whereas the MODELLED bound would have "flown" >> 20 m by then.
    assert dash_ramp_distance(10.0, 10.0, 3.0) > 20.0

    # And when no ground speed is available the reason SAYS it is a model.
    mdl = RealFlightSM(cfg(dash_max_s=60.0, dash_max_dist_m=20.0))
    t = dash_now(mdl)
    for i in range(1, 200):
        mdl.step(obs(t + DT * i))
        if mdl.state == State.SAFE:
            break
    assert mdl.safe_reason == "dash_distance_bound"
    assert "MODELLED" in mdl.transitions[-1].reason


# ---- the mission log ----------------------------------------------------------


def test_the_mission_log_carries_the_guidance_state_not_just_the_command():
    """A flight that latched the coast freeze, spun on a stale bearing or ran on
    a missing attitude used to produce a CSV of smooth, plausible setpoints with
    NO way to tell any of them apart from correct guidance."""
    from flight.camera import CameraModel
    from flight.deploy.seeker_loop import GuidanceConfig, SeekerGuidance
    g = SeekerGuidance(GuidanceConfig(), CameraModel(539.936, 539.936, 640.0, 480.0),
                       span_m=1.0)
    sm, rows = run_offline(cfg(engage_max_s=4.0, breakoff_s=0.5, safe_hold_s=0.5),
                           GateReadyTrigger(), guidance=g, fps=20.0, max_s=60.0,
                           verbose=False)
    assert State.ENGAGE in sm.visited
    cols = list(_CSV_FIELDS)
    for needed in ("sp_source", "r_hat_m", "rdot_hat_m_s", "lambda_dot_deg_s",
                   "terminal_coast", "own_state_ok", "meas_age_s", "yaw_hold",
                   "track_broken", "health", "mode", "trig_clock_fault"):
        assert needed in cols
    eng = [r.strip().split(",") for r in rows if f",{State.ENGAGE}," in r]
    assert eng, "setup: the run must reach ENGAGE"
    i_src = cols.index("sp_source")
    i_r = cols.index("r_hat_m")
    assert any(r[i_src] == "guided" for r in eng), (
        "a run whose terminal never steered must be visible in the log")
    assert any(r[i_r] for r in eng), "r_hat is still being thrown away"
    assert all(len(r) == len(cols) for r in eng)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
