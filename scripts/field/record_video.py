#!/usr/bin/env python3
"""Record video from the Pi camera for training and validation footage (run ON the Pi).

    .venv-pi/bin/python scripts/field/record_video.py --seconds 300 --exposure-us 1000
    .venv-pi/bin/python scripts/field/record_video.py            # until Ctrl-C

Writes sessions/video_<UTC>/ with video.h264 (software H.264; the Pi 5 has no hardware
encoder), frames.pts (one sensor timestamp per frame, ms) and meta.json (what was asked, what
the camera applied, frames written, gaps, size, temperature). Exits 1 if more than 1% of
frame gaps exceed 1.5 frame periods, so a recording that dropped frames cannot pass as clean.
The video is synced to the card every second, so a power cut costs about the last second.
The flight recorder service starts this at arming (scripts/pi_setup/flight-recorder.service).
Measured 2026-11-13 on the Pi 5 + AR0234: 1920x1200 at 59.5 fps and 1280x720 at 119 fps with
no dropped frames (runs/hailo_day1_20261113/README.md).
"""
import argparse
import json
import os
import signal
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seconds", type=float, default=0, help="0 = until Ctrl-C")
    ap.add_argument("--size", default="1920x1200"); ap.add_argument("--fps", type=float, default=60)
    ap.add_argument("--exposure-us", type=int, default=0, help="0 = auto exposure; outdoors use 1000 or less to freeze motion")
    ap.add_argument("--gain", type=float, default=0, help="analogue gain with a fixed exposure; 0 = leave to the camera")
    ap.add_argument("--bitrate", type=int, default=30_000_000); ap.add_argument("--flip", action="store_true", help="rotate 180 deg (camera mounted upside down)")
    ap.add_argument("--out-dir", default="sessions"); ap.add_argument("--note", default="")
    a = ap.parse_args()
    import numpy as np
    from libcamera import Transform
    from picamera2 import Picamera2
    from picamera2.encoders import H264Encoder
    from picamera2.outputs import FileOutput
    w, h = (int(v) for v in a.size.lower().split("x"))
    d = os.path.join(a.out_dir, time.strftime("video_%Y%m%dT%H%M%SZ", time.gmtime())); os.makedirs(d)
    ctrl = {"FrameRate": a.fps}
    if a.exposure_us:
        ctrl.update({"AeEnable": False, "ExposureTime": a.exposure_us})
        if a.gain: ctrl["AnalogueGain"] = a.gain
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": (w, h), "format": "YUV420"}, controls=ctrl, buffer_count=8,
                                                 transform=Transform(hflip=a.flip, vflip=a.flip)))
    stop = {"now": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True)); signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    video, pts = os.path.join(d, "video.h264"), os.path.join(d, "frames.pts")
    out = FileOutput(video, pts=pts)
    cam.start_recording(H264Encoder(bitrate=a.bitrate), out); t0 = time.time()
    time.sleep(1.0); applied = cam.capture_metadata()
    print(f"recording to {d}  ({w}x{h} at {a.fps:g} fps; Ctrl-C to stop)", flush=True)
    # Push the video to the card every second: a flight usually ends with the battery being
    # unplugged, and Linux otherwise holds up to ~30 s of written data in memory.
    fds = [os.open(p, os.O_RDONLY) for p in (video, pts)]; last_sync = time.time()
    while not stop["now"] and (a.seconds <= 0 or time.time() - t0 < a.seconds):
        time.sleep(0.2)
        if time.time() - last_sync >= 1.0:
            for fd in fds: os.fsync(fd)
            last_sync = time.time()
    for fd in fds: os.close(fd)
    cam.stop_recording(); cam.close(); wall = time.time() - t0
    ts = np.array([float(x) for x in open(pts).read().split()[1:] if x.strip()]); dt = np.diff(ts) if len(ts) > 2 else np.array([0.0])
    bad = int((dt > 1500.0 / a.fps).sum())
    meta = {"created_utc": os.path.basename(d)[6:], "size": [w, h], "asked_fps": a.fps, "flip_180": a.flip, "note": a.note,
            "recording_started_unix_approx": round(t0, 3), "fc_log_stamp": os.environ.get("FC_LOG_STAMP"),
            "requested_exposure_us": a.exposure_us or "auto", "applied_exposure_us": applied.get("ExposureTime"), "applied_gain": applied.get("AnalogueGain"),
            "frames": int(len(ts)), "seconds_of_video": round(float(ts[-1] - ts[0]) / 1000, 2) if len(ts) > 1 else 0, "wall_seconds": round(wall, 1),
            "measured_fps": round(1000.0 / float(np.median(dt)), 2) if len(ts) > 2 else None, "gaps_over_1.5_frames": bad, "worst_gap_ms": round(float(dt.max()), 1),
            "video_MB": round(os.path.getsize(video) / 1e6, 1), "temp": os.popen("vcgencmd measure_temp").read().strip(), "throttled": os.popen("vcgencmd get_throttled").read().strip()}
    json.dump(meta, open(os.path.join(d, "meta.json"), "w"), indent=1); print(json.dumps(meta, indent=1))
    return 1 if len(ts) < 3 or bad > 0.01 * len(dt) else 0


if __name__ == "__main__":
    sys.exit(main())
