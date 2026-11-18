#!/usr/bin/env python3
"""READ-ONLY: listen on a MAVLink link for N seconds and report per-message rates
and the worst gap. Sends only a GCS heartbeat. No commands, no params."""
import sys, time, collections
from pymavlink import mavutil
url, baud, secs = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
m = mavutil.mavlink_connection(url, baud=baud, source_system=254, source_component=191)
cnt = collections.Counter(); last = {}; gap = collections.defaultdict(float)
t0 = time.time(); hb = 0.0
while time.time() - t0 < secs:
    now = time.time()
    if now - hb > 1.0:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0); hb = now
    msg = m.recv_match(blocking=True, timeout=0.2)
    if msg is None or msg.get_srcSystem() != 1: continue
    n = msg.get_type(); now = time.time()
    if n in last: gap[n] = max(gap[n], now - last[n])
    last[n] = now; cnt[n] += 1
el = time.time() - t0
print(f"listened {el:.1f} s on {url}@{baud}; {sum(cnt.values())} messages, {len(cnt)} types")
for n, c in sorted(cnt.items(), key=lambda kv: -kv[1]):
    print(f"{n:32s} {c/el:7.1f} Hz   worst gap {gap[n]*1000:6.0f} ms")
