# Target drone, seventh flight: climb the pilot did not command, killed in the air (2026-11-15)

Status: **cause not established.** The flight log has not been read: after the flight the
board could not see its storage card. This file was written BEFORE the log was seen, so the
predictions below are pre-registered. Decision record: ADR-0138 addendum 17.

## Builder's report (his words, 2026-11-15)

"It started climbing uncontrolled again, I kill switched it but much lower"; "a semi hard
landing"; "I just unplugged the battery right after flight like that one time".

## What was read from the board afterwards (USB power only, battery out)

| Reading | Value | Source |
|---|---|---|
| Log list | 0 logs (logs 1 and 2 were listed that morning) | `loglist.py` |
| Opening any path on the card over MAVLink FTP | fails with system error 28 = FatFS `FR_NOT_ENABLED`, "the volume has no work area": no card mounted | `ftp_ls.py` probe; ArduPilot `AP_Filesystem_FATFS.cpp`, Copter-4.7.0 |
| Low-battery action | `BATT_FS_LOW_ACT` 2 (return home) at `BATT_LOW_VOLT` 21.6 V held for `BATT_LOW_TIMER` 10 s | `p_read.py` |
| Return-home height | `RTL_ALT_M` 15 m, `RTL_CONE_SLOPE` 3, climb at `WP_SPD_UP` 2.5 m/s | `p_read.py` |
| Flight counters | `STAT_FLTCNT` 7, `STAT_FLTTIME` 1126 s total | `p_read.py` |

The "low voltage failsafe" pre-arm message seen afterwards is not evidence: on USB alone the
board reads 0.006 V.

## Two candidate causes, and what the log must show for each

**A. Low-battery return home.** Flight six sat at 21.9 to 22.0 V under hover load after 5.3
minutes, and the same failsafe ended flight four. Away from home, return-home climbs toward
15 m at 2.5 m/s and ignores the throttle stick. Predicts: a `MODE` change to RTL with a
battery failsafe `ERR`, battery under 21.6 V for the 10 s before it, climb rate close to
2.5 m/s, throttle stick not followed.

**B. Stabilize with the stick at mid.** Flight six measured it: switching to Stabilize with
the stick where Loiter hovers raised throttle from 0.078 to 0.198 at once. Predicts: `MODE`
0 at the start of the climb, throttle output jumping within one sample, climb rate above
5 m/s within a second, no failsafe `ERR`.

**Neither.** If the log shows Loiter or Auto throughout, no failsafe, and a climb the height
controller did not ask for, that is a new mechanism and both of the above are wrong.

The Pi's ground video (`sessions/video_20261115T182243Z/`, 864 s, on the PC, not in the
repo) may show the climb rate, which separates A (steady) from B (violent).

## Not established

- Which cause. Nothing above is a measurement of this flight.
- Why the card is not mounted: knocked loose by the landing, or failed. The morning's
  refusal to start a log (addendum 15) makes a failing card more likely than it was.
- Whether this flight's log exists on the card.
