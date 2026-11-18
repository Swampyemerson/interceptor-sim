"""flight.cue_link: the live cue is latched once, at GO, advanced to GO.

The headline test is the one the interface audit asked for (G3): a target on a
straight track, a long standby, and the belief at GO must be the truth at GO,
not the truth at program start."""
import math
import socket
import time

import numpy as np
import pytest

from flight.cue_link import (CueLinkError, CueReceiver, encode_record,
                             DEFAULT_MAX_STALENESS_AT_GO_S)
from flight.cue_relay import CueQualityError, CueRecord, lla_to_ned

OWN = (40.0, -105.0, 1650.0)
_R = 6378137.0


def _lla(n, e, d=-10.0):
    """Tangent-plane inverse of flight.cue_relay.lla_to_ned about OWN."""
    lat = OWN[0] + math.degrees(n / _R)
    lon = OWN[1] + math.degrees(e / (_R * math.cos(math.radians(OWN[0]))))
    return lat, lon, OWN[2] - d


def _truth(t, p0=(200.0, -300.0), v=(0.0, 12.0)):
    return (p0[0] + v[0] * t, p0[1] + v[1] * t)


def _feed_track(rx, t0, t1, rate=5.0, noise_m=0.0, latency=(0.05, 0.05), seed=0,
                with_t_src=True, v=(0.0, 12.0)):
    """Send truth samples taken at t in [t0, t1) to the receiver, each arriving
    after a latency drawn from `latency` (min, max)."""
    rng = np.random.default_rng(seed)
    t = t0
    while t < t1:
        n, e = _truth(t, v=v)
        n += rng.normal(0.0, noise_m)
        e += rng.normal(0.0, noise_m)
        lat, lon, alt = _lla(n, e)
        rec = CueRecord(t=0.0, lat=lat, lon=lon, alt_m_msl=alt,
                        vn=v[0], ve=v[1], vd=0.0)
        lag = rng.uniform(*latency)
        rx.feed(encode_record(rec, t_src=(t if with_t_src else None)), t_rx=t + lag)
        t += 1.0 / rate


def test_belief_at_go_is_truth_at_go_after_a_long_standby():
    rx = CueReceiver(clock=lambda: 0.0)
    _feed_track(rx, 0.0, 40.0)                     # 40 s of standby
    go = 40.0
    sol = rx.latch(*OWN, now_t=go)
    n, e = _truth(go - 0.05)                       # belief lags by the link's latency
    assert abs(sol.belief_r0_ned[0] - n) < 0.05
    assert abs(sol.belief_r0_ned[1] - e) < 0.05
    # and it is nowhere near the stale answer (truth at program start: 480 m back)
    assert abs(sol.belief_r0_ned[1] - _truth(0.0)[1]) > 400.0
    assert sol.latch_t == go


def test_min_delay_anchor_removes_jitter_and_leaves_the_minimum_latency():
    rx = CueReceiver(clock=lambda: 0.0)
    _feed_track(rx, 0.0, 20.0, latency=(0.04, 0.30), seed=3)   # jittery link
    sol = rx.latch(*OWN, now_t=20.0)
    # the belief should lag truth by about the MINIMUM latency (0.04 s -> 0.48 m),
    # not the mean (0.17 s -> 2 m)
    lag_m = _truth(20.0)[1] - sol.belief_r0_ned[1]
    assert 0.3 < lag_m < 0.9, lag_m


def test_without_t_src_receive_time_is_used():
    rx = CueReceiver(clock=lambda: 0.0)
    _feed_track(rx, 0.0, 10.0, with_t_src=False)
    sol = rx.latch(*OWN, now_t=10.0)
    assert abs(sol.belief_r0_ned[1] - _truth(10.0 - 0.05)[1]) < 0.1


def test_latch_is_once_and_later_records_are_dropped():
    rx = CueReceiver(clock=lambda: 0.0)
    _feed_track(rx, 0.0, 10.0)
    sol = rx.latch(*OWN, now_t=10.0)
    before = rx.latched
    n_rx_at_latch = rx.stats(now_t=10.0).n_rx
    _feed_track(rx, 10.0, 12.0)                    # the link is dead by design
    assert rx.latched is before is sol
    st = rx.stats(now_t=12.0)
    assert st.n_after_latch >= 10 and st.n_rx == n_rx_at_latch
    with pytest.raises(CueLinkError):
        rx.latch(*OWN, now_t=12.0)


def test_stale_stream_refuses_go():
    rx = CueReceiver(clock=lambda: 0.0)
    _feed_track(rx, 0.0, 10.0)
    with pytest.raises(CueQualityError):
        rx.latch(*OWN, now_t=10.0 + DEFAULT_MAX_STALENESS_AT_GO_S + 1.0)
    assert rx.latched is None


def test_no_records_refuses_go():
    rx = CueReceiver(clock=lambda: 0.0)
    with pytest.raises(CueLinkError):
        rx.latch(*OWN, now_t=1.0)


def test_garbage_is_counted_not_fatal():
    rx = CueReceiver(clock=lambda: 0.0)
    assert rx.feed(b"not json\n{\"lat\": 1}\n", t_rx=0.0) == 0
    assert rx.feed(b'{"lat": 95.0, "lon": 0, "alt_m_msl": 0}\n', t_rx=0.0) == 0
    assert rx.stats(now_t=0.0).n_bad == 3


def test_only_the_freshest_window_is_fitted_so_an_old_turn_does_not_matter():
    rx = CueReceiver(clock=lambda: 0.0)
    _feed_track(rx, 0.0, 6.0, v=(12.0, 0.0))       # flying north, then...
    # ...a new leg east, sampled on the same clock from where the first ended
    rng_t = 6.0
    while rng_t < 12.0:
        n, e = 200.0 + 12.0 * 6.0, -300.0 + 12.0 * (rng_t - 6.0)
        lat, lon, alt = _lla(n, e)
        rx.feed(encode_record(CueRecord(0.0, lat, lon, alt, 0.0, 12.0, 0.0),
                              t_src=rng_t), t_rx=rng_t + 0.05)
        rng_t += 0.2
    sol = rx.latch(*OWN, now_t=12.0)
    assert abs(sol.belief_vel0_ned[0]) < 0.2 and abs(sol.belief_vel0_ned[1] - 12.0) < 0.2


def test_udp_loopback_end_to_end():
    rx = CueReceiver(bind="127.0.0.1", port=0)
    rx.start()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        t0 = time.monotonic()
        for i in range(30):
            ts = time.monotonic()
            n, e = _truth(ts - t0)
            lat, lon, alt = _lla(n, e)
            s.sendto(encode_record(CueRecord(0.0, lat, lon, alt, 0.0, 12.0, 0.0),
                                   t_src=ts, seq=i), ("127.0.0.1", rx.port))
            time.sleep(0.05)
        time.sleep(0.1)
        st = rx.stats()
        assert st.n_rx == 30 and st.n_bad == 0
        now = time.monotonic()
        sol = rx.latch(*OWN, now_t=now)
        assert abs(sol.belief_r0_ned[1] - _truth(now - t0)[1]) < 0.5
    finally:
        rx.close()
