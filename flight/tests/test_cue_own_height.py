"""The cue places the target against the GPS RECEIVER's height, never the
Pixhawk's reported height (the barometer's on the real board: 97 m low on
2026-11-26, which latched a target on the ground 101 m overhead)."""
import asyncio

from flight.deploy import real_flight as rf
from flight.tests.fake_mavsdk import FakeDrone


def _state(**kw):
    s = rf._new_link_state()
    s.update(kw)
    return s


def test_height_is_the_receivers_not_the_pixhawks():
    s = _state(lla=(40.0, -105.0, 1543.9), gps_amsl=1641.0, t_gps_amsl=100.0)
    assert rf.cue_own_lla(s, now=100.5) == (40.0, -105.0, 1641.0)


def test_no_receiver_height_means_no_answer_not_the_pixhawks_height():
    assert rf.cue_own_lla(_state(lla=(40.0, -105.0, 1543.9)), now=1.0) is None
    stale = _state(lla=(40.0, -105.0, 1543.9), gps_amsl=1641.0, t_gps_amsl=100.0)
    assert rf.cue_own_lla(stale, now=100.0 + rf.CUE_OWN_GPS_HEIGHT_MAX_AGE_S + 0.1) is None
    assert rf.cue_own_lla(_state(gps_amsl=1641.0, t_gps_amsl=1.0), now=1.0) is None


def test_cue_trigger_asks_for_the_receivers_height_stream():
    trig = rf.CueTrigger.__new__(rf.CueTrigger)
    s = _state()
    assert rf._wire_cue_trigger(trig, s) is True
    assert rf._wire_cue_trigger(object(), s) is False
    s.update(lla=(40.0, -105.0, 1543.9), gps_amsl=1641.0, t_gps_amsl=rf.time.monotonic())
    assert trig.own_lla()[2] == 1641.0


def test_telemetry_carries_the_receivers_height_through_a_barometer_offset():
    """Producer to consumer: the fake vehicle's streams, read by the driver's
    own telemetry readers, with the Pixhawk height 97 m low."""
    async def go():
        drone = FakeDrone()
        drone.lla = (40.0, -105.0, 1641.0)
        drone.pixhawk_height_error_m = -97.1
        s = rf._new_link_state()
        tasks = rf._start_telemetry(drone, s, rc_sinks=[], want_battery=False,
                                    want_gps_height=True)
        try:
            for _ in range(200):
                await asyncio.sleep(0.01)
                if s["lla"] is not None and s["gps_amsl"] is not None:
                    break
        finally:
            for t in tasks:
                t.cancel()
        return s
    s = asyncio.run(go())
    assert abs(s["lla"][2] - 1543.9) < 0.01          # what the Pixhawk reports
    assert abs(rf.cue_own_lla(s)[2] - 1641.0) < 0.01  # what the cue uses
