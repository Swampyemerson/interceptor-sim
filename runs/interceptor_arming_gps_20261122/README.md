# Interceptor would not arm outdoors, 2026-11-22: GPS reception, not wiring

Read-only diagnosis from the logs the Pi saves and from the Pi's ground-station bridge.
Nothing was written to the Pixhawk.

**What the Pixhawk said** (log `fc_20261122T221947Z`, first 77 s after power-up):
`Arming denied: Resolve system health failures first`, with the standing failures
`Preflight Fail: Global position estimate required` and `Missing FMU SD Card` (the second
is a warning only, `COM_ARM_SDCARD` 1). Mode: Position. `COM_ARM_WO_GPS` is 0 on this
board, set on purpose (`configs/px4_6cmini/bench.params`): no arming in any mode without a
good GPS solution. Radio link, sticks, kill switch (off), battery (24.0 V) all normal.

**The GPS unit is connected and talking**: its noise and jamming readings change every
second and both compasses deliver data. It was reporting 0 satellites used and no fix.
Live through the bridge, about 5 minutes after power-up: 0 satellites until 15:24:51
local (laptop clock), then a 3D fix on 4 satellites with HDOP 2.6. PX4 wants 6 by default
before it trusts the position.

**GPS readings across logs** (`gps_cmp.py`; jamming indicator is the receiver's own 0 to
255 figure, higher is worse):

| Log | Where the GPS sat | Satellites used | Jamming indicator (median, max) | Noise (median) |
|---|---|---|---|---|
| 2026-11-14 | bench rig, before the pod | 7 to 10, 3D fix | 30, 33 | 90 |
| 2026-11-15 | bench rig, before the pod | 8, 3D fix | 47, 82 | 84 |
| 2026-11-19 | not recorded | 0 | 53, 63 | 110 |
| 2026-11-22 13:54, indoors | on the pod lid, over the Pi | 0 | 45, 69 | 96 |
| 2026-11-22 14:20, outdoors | on the pod lid, over the Pi | 0 in the first 76 s | 71, 139 | 116 |

**Reading, not yet proven:** the unit now sits a few centimetres above the Pi 5 and its
AI board, and the receiver's own interference figure is two to four times what it was
when it had 8 satellites. Slow, weak acquisition fits radio noise from the Pi. Two other
causes are not ruled out: each power cycle restarts the search (the aircraft was
power-cycled three times in ten minutes), and the sky view at that spot is unknown.

**The test that separates them:** leave it powered and still for three minutes under open
sky and note the satellite count; then lift the GPS unit 20 cm or more away from the Pi
without unplugging it and watch the count for a minute. A jump means the Pi is the cause
and the unit needs distance or a ground plane under it.

Also seen in the same log: `navigator low on stack! (268 bytes left)`, twice. Not
followed up.
