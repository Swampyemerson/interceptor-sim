# Target logs, 26 November 2026

Five of the seven logs on the target's card, downloaded over USB after the field
session with `tgt_loglist.py` and `tgt_logget.py` (Windows Python; they send only log
list and log data requests). The `.bin` files stay on the laptop and are not in git:
they carry GPS positions.

| Log | Size | What it is |
|---|---|---|
| 1 | 1.0 MB | 52 s, Loiter, no automatic flight |
| 2 | 10.7 MB | automatic course for 205 s at about 0.9 m/s, 3.6 m up |
| 4 | 2.3 MB | Loiter on the ground; a radio failsafe disarm |
| 5 | 8.9 MB | automatic course for 98 s at about 1.0 m/s, 5.0 m up |
| 7 | 4.6 MB | Stabilize, after the session |

Not fetched: log 3 (0.1 MB) and log 6 (40 MB, powered and sitting during the
rehearsal). Numbers and their comparison: `runs/flight_log_analysis_20261126/README.md`.
