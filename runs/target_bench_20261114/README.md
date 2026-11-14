# Target drone bench session, 2026-11-14

Kakute H7, ArduCopter 4.7.0, on USB, props off, never flown. Write-up: ADR-0138.

| File | What |
|---|---|
| `accelcal_result.json` | Six-position accelerometer calibration run with `scripts/bench/ap_accel_cal.py` after QGroundControl's own page would not advance past the first position. Offsets 0.067 / -0.016 / 0.031 m/s/s, scale factors within 0.2% of 1, identical before and after the reboot. The board's "3D Accel calibration needed" complaint is gone. |

Setting dumps from the same session: `configs/target_kakute/dumps/20261114_before.tsv`, `_after.tsv`
(taken before the calibration).
