# Target drone bench session, 2026-11-14

Kakute H7, ArduCopter 4.7.0, on USB, props off, never flown. Write-up: ADR-0138.

| File | What |
|---|---|
| `accelcal_result.json` | Six-position accelerometer calibration run with `scripts/bench/ap_accel_cal.py` after QGroundControl's own page would not advance past the first position. Offsets 0.067 / -0.016 / 0.031 m/s/s, scale factors within 0.2% of 1, identical before and after the reboot. The board's "3D Accel calibration needed" complaint is gone. |

Setting dumps from the same session: `configs/target_kakute/dumps/20261114_before.tsv`, `_after.tsv`
(taken before the calibration).
| `gps_wire_poll.json` | Outdoors, clear sky, `scripts/bench/ap_gps_wire_check.py`. On the board's GPS port: nothing heard in 3 s of listening, a 30-byte nonsense test string came back byte for byte at both 230400 and 115200 baud, and the module answered none of four questions (the only replies were the questions themselves). The board's GPS transmit and receive lines are connected to each other and the module's output does not reach the board. |
| `gps_wire_poll_after_fix.json` | Same check after the builder removed a stray wire strand that was crossing the two GPS signal lines: no echo, 2,062 bytes heard in 3 s, the module identifies itself as a u-blox SAM-M10Q, 3D fix with 14 satellites. The raw bytes are removed from this copy because they contain a position. After a reboot the board's own driver reported a 3D fix with 15 to 16 satellites. |
| `crash_flight_timeline.txt` | The crash flight (board log 7 of 7), four rows a second from arming: throttle stick, the height controller's throttle output (`ThO`), desired and actual climb rate, the four motor outputs, vibration, attitude, battery. Made with `scripts/bench/ap_flight_summary.py`. The `.BIN` logs themselves stay off the repo because they contain positions (kept on the dev PC under `C:\Temp\bench\target_crash_20261114\`). |

## What the crash log shows (2026-11-14, read the same day)

Loiter mode, 3.8 s in the air, 23 m at the top. From lift-off to the motor cut:

| Quantity | Value | Meaning |
|---|---|---|
| Throttle stick | at minimum (987) from 1.3 s after lift-off | the pilot was asking for down |
| Height controller's throttle output | 0.00 throughout; wanted -2.5 m/s | it was asking for down too |
| Actual climb rate | rose to +7.6 m/s | |
| Roll / pitch rate error | 27 / 24 deg/s standard deviation, controller output flipping sign every sample at up to 0.38 | the roll and pitch rate loops were oscillating hard |
| Yaw rate error | 5 deg/s | yaw was calm |
| Motor mixer's average thrust (`ThrOut`) | 0.10, pinned at its cap `ThrAvMx` 0.10 | the mixer raised thrust to make room for the oscillation |
| Motor outputs | mean 1302 us; one motor at the 1150 floor in 30 of 30 samples | |
| Lift-off | at a mean output of roughly 1200 to 1250 us | hover thrust is about 0.06 in ArduPilot's units, not the 0.20 set |

The cap is half of `MOT_THST_HOVER` in Loiter and AltHold (`ATC_THR_MIX_MAX` 0.5). With the hover
guess at 0.20 the cap was 0.10, well above this aircraft's real hover thrust, so an oscillating
attitude controller held the aircraft in a climb the height controller could not cancel.
