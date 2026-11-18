# Interceptor flight controller, read-only bench snapshot (2026-11-18)

Bench state: Pixhawk 6C Mini on USB power from the Pi (no battery, no GPS fix,
transmitter off), Pi 5 at 192.168.0.61, flight-recorder service running on the
USB link. Nothing was written to the board and nothing was commanded.

| File | What | How |
|---|---|---|
| `interceptor_params.txt` | 75 named settings plus heartbeat (disarmed) | `scripts/bench/px4_named_params_readonly.py udpout:192.168.0.61:14550` from the laptop, through the Pi's bridge |
| `pi_telem2_link.txt` | MAVSDK link check and message rates on the Pi's serial cable to TELEM2 | on the Pi: `scripts/field/fc_link_check.py --url serial:///dev/ttyAMA10:921600` and `scripts/bench/mavlink_rate_probe_readonly.py /dev/ttyAMA10 921600 15` |

What it shows for automatic flight:

- The Pi reaches PX4 1.16.0 over TELEM2 at 921600 with MAVSDK while the recorder
  holds the USB link. The two links work at the same time.
- On TELEM2 the Pi gets attitude at 100 Hz, position and velocity at 30 Hz, GPS
  at 10 Hz, PX4's clock (`SYSTEM_TIME`) at 1 Hz. `RC_CHANNELS` was absent because
  the transmitter was off, so the rate of the channel the GO switch is read from
  is NOT measured here.
- Flight envelope on the board: 12 m/s, 5 m/s², 45° lean (stock).
- Offboard loss: 1.0 s timeout, then Position mode (`COM_OBL_RC_ACT` 0).
- Radio loss: 0.5 s timeout, then Return (`NAV_RCL_ACT` 2), with NO exception for
  offboard flight (`COM_RCL_EXCEPT` 0). Switching the transmitter off after GO
  will pull the aircraft out of the intercept.
- Geofence: 457 m horizontal, 30.5 m vertical, action Hold (`GF_ACTION` 2).
- Power module scale 36.37 A/V, so the board cannot read above 120 A.
- No flight mode slot or switch is mapped to Offboard; the Pi's program has to
  request it.

## Cue link over Wi-Fi (added the same day)

`cue_link_wifi_tx.json` / `cue_link_wifi_rx.json`: the laptop sent a made-up
target (12 m/s, 0.3 m position noise, 5 Hz) with `scripts/field/cue_send.py`;
the Pi received it with `scripts/bench/cue_link_bench.py` and latched after 30 s.

- Received at 5.00 Hz, largest gap 0.21 s, nothing malformed.
- Latched belief against the truth at the matching instant: 0.12 m north,
  0.29 m east. The stale answer (where the target was when the stream began)
  would have been 336 m behind.
- Round trip laptop to Pi and back: 4.3 ms at best, 6.9 ms median. Some echoes
  took 1.5 to 2.5 s while the Pi's receive gaps stayed at 0.21 s, so the stalls
  were on the return path.
- `cue_link_wifi_rx_stale_refusal.json`: with the sender stopped 4 s before the
  latch, the Pi refused ("last record is 3.95 s old, limit 1.00 s").

Not covered: a real target's GPS (the laptop made the track up), the field
network (this was home Wi-Fi), and the flight program, which does not use this
link yet.

A wrong turn worth recording: my first runs delivered nothing, and I spent a
while suspecting the Pi's Wi-Fi. The fault was mine. Starting the listener with
`cd DIR && nohup ... &` over SSH keeps the SSH command open until the listener
exits, so the sender started after it had gone. A sniffer on the Pi's interface
showed the packets arriving. I had switched the Pi's Wi-Fi power saving off and
on while testing; it is back on, as found.
