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

---

# RESULT (added after the card was read in the PC, same day)

The card was intact: logs 1, 2 and 3 were all on it, `LASTLOG.TXT` = 3. The board had simply
not mounted it after the landing. Everything was copied to
`C:\Temp\bench\target_card_copy_20261115\`; this flight is `00000003.BIN`, 105 MB, 557 s.

## Verdict: candidate A. Candidate B is refuted (the log has no Stabilize at all).

| Prediction for A | Log | Verdict |
|---|---|---|
| `MODE` to RTL with a battery failsafe `ERR` | 546.77 s: `MODE` 6 reason 4, `ERR` subsystem 6 code 1, "Battery 1 is low 21.52V", "Battery Failsafe" | held |
| Under 21.6 V for the 10 s before | 21.48 to 21.55 V from 543 s on | held |
| Climb rate close to 2.5 m/s | 0 to 1.86 m/s in 3.1 s and still rising when the motors were cut | consistent; cut before it reached the limit |
| Throttle stick not followed | stick to the bottom (987) by 547.5 s; height went 3.24 to 5.26 m | held |

## Sequence (log seconds; clock time from the log's GPS time)

| Log time | UTC | Event |
|---|---|---|
| 52.3 | 18:27:50.8 | Armed (yaw stick) |
| 143.9 to 240.2 | | Auto: 7 laps completed (jumps 1 to 7 of 10) |
| 376.4 to 461.6 | | Auto again: 4 more laps to the end of the mission (jumps 8 to 10, then the last pass), restart, 2 more |
| 461.6 to 546.8 | | Loiter, flown by hand, leaning up to 26 degrees |
| 546.77 | 18:36:05.3 | Battery failsafe, return home begins at 3.24 m |
| 549.87 | 18:36:08.4 | Motors cut with the arm switch at 5.26 m, rising 1.86 m/s |
| about 550.1 | | Peak 5.8 m |
| about 551.2 | | Ground. A fall of 5.8 m is about 10.7 m/s at impact; the log's last airborne climb rate is -9.5 m/s |

Flight from arming to the cut: 497.6 s (8.3 minutes). Battery 24.55 V at the start, 21.5 V
under hover load at the failsafe. The board counted 2758 mAh from a 1500 mAh pack, so the
current sensor over-reads by more than the factor of two estimated from flight six.

After the impact the board reported "Check mag field (xy diff:243>100)" and "DCM Roll/Pitch
inconsistent 17 deg". Both were logged within two seconds of hitting the ground and may be
only that; the compass mount is worth a look.

## Ground video

`sessions/video_20261115T182243Z/` started at 18:22:44 UTC, so arming is at 306 s into the
video and the motor cut at 804 s of its 864 s. `scripts/bench/ap_log_clock_times.py`.

## What this changes

- The climb was the aircraft doing what it was configured to do, not a control fault.
- Leaving return-home from Loiter needs the mode switch to move, and the next position is
  Stabilize: with the stick at the bottom that is motors at the floor, with it at mid 1.6
  times hover thrust (flight six). The switch layout makes the recovery itself hazardous.
