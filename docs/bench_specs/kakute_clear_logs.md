# Task: clear the junk dataflash logs off the target drone's Kakute H7 (builder asked for this)

The Kakute H7 (ArduPilot) is plugged into the Windows PC by USB. WSL cannot see COM ports, so
drive it with WINDOWS Python from WSL:
  cd /mnt/c/Users/Emers
  /mnt/c/Users/Emers/AppData/Local/Programs/Python/Python313/python.exe Downloads/CubeProgSetup/<script>.py
Existing scripts in /mnt/c/Users/Emers/Downloads/CubeProgSetup/ show how the port is found and
the MAVLink link opened (read `pull_newest.py` and `fc_gps_check.py` first; pymavlink is
installed for that Python). A Pixhawk may ALSO be plugged in: identify the Kakute by its
autopilot/board (ArduPilot heartbeat, not PX4) and do not touch the other board.

Write `erase_logs.py` next to them: connect; LOG_REQUEST_LIST and print how many logs exist
and the newest one's size/date; send LOG_ERASE; wait; list again and confirm 0 (or 1 tiny new
one); read back LOG_DISARMED and make sure it is 0 (set it to 0 with a read-back if not).
Change nothing else. Timeouts on every wait; print progress lines.

Report: counts before/after, LOG_DISARMED value, and whether the builder must unplug/replug
USB (say so plainly if the port would not open or the board did not answer). Do not run git.
