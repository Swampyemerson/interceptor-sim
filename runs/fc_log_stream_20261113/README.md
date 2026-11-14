# Flight log streamed off the Pixhawk, bench session 2026-11-13

Pixhawk 6C Mini, PX4 v1.16.0, disarmed, on the bench. Its microSD slot is dead; these
runs test having PX4 stream its flight log to another computer instead. Write-up:
`docs/fc_logging_without_sd.md`. The `.ulg` files are git-ignored (4.5 MB); every capture's
`.stats.json` is here. One trimmed capture is the test fixture
`tests/fixtures/px4_v1160_log_stream_capture.ulg`.

| Folder / file | What |
|---|---|
| `px4_shell_status.txt` | The Pixhawk's own console: logger running and idle, no SD mounted, link rate caps (46,080 B/s serial, 100,000 B/s USB). |
| `fc_20261114T052256Z` | Serial cable, 20 s: 966 packets lost in 163 gaps, attitude stream 50 Hz. |
| `fc_20261114T052317Z` | Serial, session ended WITHOUT a stop command. The next bare start was refused (result 1; it wrote no file). |
| `fc_20261114T052327Z` | Serial, stop-then-start: accepted. |
| `usb/fc_20261114T052426Z` | USB from the PC, 60 s, slow acknowledgements: header took 22 s, 74 packets lost at the hand-over. |
| `usb/fc_20261114T052551Z` | USB, 60 s, after the fix: 4 packets lost at the hand-over, none after, 40 KB/s. |
| `usb/fc_20261114T053729Z` | USB, 45 s, verdict CLEAN, taken while the Pi ran the offboard bench gate. |
| `gate/` | That bench gate's log: PASS, 300 setpoints, largest gap 0.05 s, oldest own-state 0.02 s. |
| `arm_trigger/` | Recorder on the Pi with a pretend "armed" from 8 s to 20 s: video meta (918 frames, none dropped) and the two log stats files. |
| `gcs_bridge/` | 2026-11-14, ground-station bridge. `probe_usb_loopback.json` + the stats file: a stand-in ground station downloaded all 1,122 settings in 0.73 s through the recorder on the PC's USB port while a 60 s log recorded CLEAN. `probe_wifi_autodiscover.json`: the same stand-in on the PC, listening on UDP 14550, was found by the Pi's service over Wi-Fi in 0.25 s and downloaded all settings in 5.25 s (Pi on the serial cable). "1123 received" counts PX4's extra checksum entry. `fc_20261114T165725Z_ground.stats.json`: the Pi's first file over USB, with the real QGroundControl connecting 231 s in: one start-up gap, then two holes at 242 s and 288 s (7,332 packets), none in the remaining five minutes. |

The three earliest stats files predate the `gaps_at_startup` field; their verdict `GAPS`
was computed by the first version of the rule (any gap).
