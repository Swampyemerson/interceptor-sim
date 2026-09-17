# Task: measure how well the Pi timestamps camera frames

Why: simulation says the interceptor's estimator needs to know WHEN each frame was exposed
to within 10-20 ms. The Pi 5 (ssh admin@192.168.0.42, key auth works, picamera2 + OpenCV
installed, OV9281 on CSI, repo at ~/interceptor-sim) gives every frame a hardware
`SensorTimestamp` in its metadata. Measure, do not assume.

Write `scripts/bench/frame_timing_probe.py` (runs ON THE PI; copy with scp, run with ssh):
picamera2, 1280x800, fixed exposure 1000 us, as fast as the mode allows, 20 s. Per frame
record: SensorTimestamp (ns), time.monotonic_ns() and time.clock_gettime_ns(CLOCK_BOOTTIME)
when the frame reaches Python, and the same again after running the AprilTag detector the
repo already uses on the frame (find it: grep -ri apriltag flight/ scripts/ | head).
Report to a CSV + printed summary: frame interval mean/std/max (from SensorTimestamp);
capture->Python latency mean/std/p99; capture->after-detect latency mean/std/p99; which
clock SensorTimestamp is on (compare with BOOTTIME vs MONOTONIC); dropped frames. Pull the
CSV back to `runs/frame_timing/` in the WSL repo. State plainly what this does NOT measure:
the Pi <-> flight-controller clock offset (needs the FC wired to the Pi; later).
Verdict line: if guidance uses SensorTimestamp, the residual timestamp error is ___ ms.
Do not run git. Do not install packages without saying so first (pip --user is acceptable
for a pure-python package; report it).
