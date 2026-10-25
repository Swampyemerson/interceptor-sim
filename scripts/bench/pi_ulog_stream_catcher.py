"""Onboard ULog stream-catcher: PX4 streams its flight log over TELEM2 (MAVLink),
the Pi writes it to its own SD. No FC SD card and no WiFi needed during flight.

Run (foreground test):   python stream_logger.py --seconds 30
Run (flight, daemon):    setsid nohup python stream_logger.py > /tmp/stream_logger.log 2>&1 &
Stop a flight recording: touch /tmp/stream_logger.stop   (clean stop + footer)
"""
import argparse
import asyncio
import base64
import os
import time
from datetime import datetime, timezone

from mavsdk import System

OUT_DIR = os.path.expanduser("~/interceptor-sim/logs/streamed_ulg")
STOP_FLAG = "/tmp/stream_logger.stop"


async def main(seconds: float | None) -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    if os.path.exists(STOP_FLAG):
        os.remove(STOP_FLAG)
    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    path = os.path.join(OUT_DIR, f"stream_{ts}.ulg")

    drone = System()
    await drone.connect(system_address="serial:///dev/ttyAMA0:921600")
    print("[stream_logger] connecting ...", flush=True)
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    print("[stream_logger] connected; starting log streaming", flush=True)
    await drone.log_streaming.start_log_streaming()

    n_bytes = 0
    t0 = time.time()
    last_print = 0.0
    with open(path, "wb") as f:
        async for raw in drone.log_streaming.log_streaming_raw():
            data = base64.b64decode(raw.data_base64)
            f.write(data)
            n_bytes += len(data)
            now = time.time()
            if now - last_print >= 5:
                last_print = now
                rate = n_bytes / max(0.1, now - t0) / 1000
                print(f"[stream_logger] {n_bytes/1e6:.2f} MB ({rate:.0f} KB/s avg)",
                      flush=True)
            if seconds is not None and now - t0 >= seconds:
                break
            if os.path.exists(STOP_FLAG):
                print("[stream_logger] stop flag seen", flush=True)
                break
    try:
        await drone.log_streaming.stop_log_streaming()
    except Exception as e:  # noqa: BLE001 -- teardown is best-effort
        print(f"[stream_logger] stop_log_streaming: {e}", flush=True)
    print(f"[stream_logger] DONE: {path} ({n_bytes/1e6:.2f} MB in "
          f"{time.time()-t0:.0f}s)", flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=None,
                    help="stop after this long (default: run until stop flag)")
    args = ap.parse_args()
    if args.seconds is not None:
        # HARD bound independent of data arrival -- a stream that never sends
        # a byte must still end the test (the first version hung forever).
        try:
            asyncio.run(asyncio.wait_for(main(args.seconds),
                                         timeout=args.seconds + 45))
        except (asyncio.TimeoutError, TimeoutError):
            print("[stream_logger] HARD TIMEOUT: no stream established "
                  "(connect or first byte never happened)", flush=True)
            raise SystemExit(3)
    else:
        asyncio.run(main(None))
