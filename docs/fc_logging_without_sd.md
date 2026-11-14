# Flight logs without the Pixhawk's SD card

The interceptor's Pixhawk 6C Mini cannot mount a microSD card, so it writes no flight log
(ULog, PX4's flight-recorder file). This page is the substitute: PX4 sends the same log
over a cable to the Raspberry Pi, and the Pi writes it to its own card.

## Predictions, written before the first measurement (2026-11-13)

Set-up: Pixhawk on the bench, disarmed, PX4 v1.16.0, talking to the Pi over the 3-pin serial
cable at 921600 baud. PX4's shell reports the logger running with 166 topics, "not logging",
no SD mounted, and the serial link capped at 46,080 bytes/s with 24,042 already used by
ordinary telemetry.

1. **Why streaming was "accepted once, then refused" on 2026-10-25.** PX4's logger refuses a
   start request while it believes a stream is already running, and only a stop request
   clears that. A client that dies without sending stop leaves it stuck until reboot.
   Prediction: a start sent after a session that was killed without a stop is refused
   (result code 1, "temporarily rejected"); the same start sent after a stop is accepted.
   Three sessions in a row start cleanly when each begins with stop-then-start.
   *If this is wrong* (start refused even after a stop), the refusal has another cause and
   the serial path needs a reboot between sessions, which makes it unusable in the field.
2. **The full log does not fit down the serial cable at today's settings.** PX4 gives the
   log stream at most 70% of the link cap (about 32 KB/s), and scales ordinary telemetry
   down to make room. Prediction: gaps in the log's packet numbering, and the attitude
   stream the guidance code reads falls below its normal 100 Hz while the log is streaming.
   *If this is wrong* (no gaps, attitude stays at 100 Hz), the existing cable is enough and
   nothing else needs changing.

A path counts as a usable flight recorder when a 60 s capture opens in `pyulog` (the
standard ULog reader), has no packet gaps, and leaves the attitude stream at 100 Hz.
