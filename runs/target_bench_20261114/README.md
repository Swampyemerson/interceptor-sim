# Target drone bench session, 2026-11-14

Kakute H7, ArduCopter 4.7.0, on USB, props off, never flown. Write-up: ADR-0138.

| File | What |
|---|---|
| `accelcal_result.json` | Six-position accelerometer calibration run with `scripts/bench/ap_accel_cal.py` after QGroundControl's own page would not advance past the first position. Offsets 0.067 / -0.016 / 0.031 m/s/s, scale factors within 0.2% of 1, identical before and after the reboot. The board's "3D Accel calibration needed" complaint is gone. |

Setting dumps from the same session: `configs/target_kakute/dumps/20261114_before.tsv`, `_after.tsv`
(taken before the calibration).
| `gps_wire_poll.json` | Outdoors, clear sky, `scripts/bench/ap_gps_wire_check.py`. On the board's GPS port: nothing heard in 3 s of listening, a 30-byte nonsense test string came back byte for byte at both 230400 and 115200 baud, and the module answered none of four questions (the only replies were the questions themselves). The board's GPS transmit and receive lines are connected to each other and the module's output does not reach the board. |
| `gps_wire_poll_after_fix.json` | Same check after the builder removed a stray wire strand that was crossing the two GPS signal lines: no echo, 2,062 bytes heard in 3 s, the module identifies itself as a u-blox SAM-M10Q, 3D fix with 14 satellites. The raw bytes are removed from this copy because they contain a position. After a reboot the board's own driver reported a 3D fix with 15 to 16 satellites. |
