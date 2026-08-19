#!/usr/bin/env python3
"""M0 gate script: arm, take off to 2 m, land — over MAVSDK-Python.

What: connects to a running PX4 SITL instance, waits for GPS/home health,
sets a 2 m takeoff altitude, arms, takes off, waits for altitude, then lands.
Why: this is the smallest possible proof that the offboard-control path
(MAVSDK -> PX4 -> Gazebo) works end to end before we build camera/guidance
logic on top of it. See docs/goals.md milestone M0.

Every stage is logged to logs/m0_takeoff_<UTC timestamp>.csv as
(time_s, relative_altitude_m, flight_mode) rows. Exits 0 only if every
stage (connect, health, arm, takeoff, altitude reached, land, landed/
disarmed) succeeded; exits 1 with a clear reason otherwise.

Run manually (with PX4 SITL already running and listening for MAVSDK on
udpin://0.0.0.0:14540):

    .venv/bin/python scripts/m0_takeoff.py

Normally this is invoked by scripts/check_m0.sh, which also launches and
tears down the simulator.
"""

import asyncio
import csv
import os
import sys
import time
from datetime import datetime, timezone

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.telemetry import LandedState

SYSTEM_ADDRESS = "udpin://0.0.0.0:14540"
CONNECT_TIMEOUT_S = 60
HEALTH_TIMEOUT_S = 90
TAKEOFF_ALTITUDE_M = 2.0
# PX4 position-control loops settle asymptotically, so requiring 100% of the
# target altitude is flaky. 80% is the commonly used "close enough, still
# climbing correctly" threshold for this kind of scripted check.
ALTITUDE_SUCCESS_FRACTION = 0.8
ALTITUDE_TIMEOUT_S = 60
LAND_TIMEOUT_S = 90
LOG_POLL_HZ = 5.0

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS_DIR = os.path.join(REPO_ROOT, "logs")


class TelemetryState:
    """Latest-known values from MAVSDK's telemetry streams.

    MAVSDK telemetry calls (position(), flight_mode(), ...) are async
    generators ("subscribe and stream forever"), not one-shot getters. We
    run one background task per stream that just remembers the latest
    value here, so the main logic can poll a plain attribute.
    """

    def __init__(self):
        self.relative_altitude_m = None
        self.flight_mode = "UNKNOWN"
        self.armed = None
        self.landed_state = None


async def track_position(drone, state):
    async for position in drone.telemetry.position():
        state.relative_altitude_m = position.relative_altitude_m


async def track_flight_mode(drone, state):
    async for mode in drone.telemetry.flight_mode():
        state.flight_mode = str(mode)


async def track_armed(drone, state):
    async for armed in drone.telemetry.armed():
        state.armed = armed


async def track_landed_state(drone, state):
    async for landed in drone.telemetry.landed_state():
        state.landed_state = landed


async def wait_for_connection(drone, timeout_s):
    async def _wait():
        async for conn_state in drone.core.connection_state():
            if conn_state.is_connected:
                return

    await asyncio.wait_for(_wait(), timeout=timeout_s)


async def wait_for_health(drone, timeout_s):
    async def _wait():
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                return

    await asyncio.wait_for(_wait(), timeout=timeout_s)


async def main():
    started = time.monotonic()
    os.makedirs(LOGS_DIR, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log_path = os.path.join(LOGS_DIR, f"m0_takeoff_{timestamp}.csv")

    print(f"[m0] Logging telemetry to {log_path}")
    log_file = open(log_path, "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow(["time_s", "relative_altitude_m", "flight_mode"])
    log_file.flush()

    def log_row():
        alt = state.relative_altitude_m
        alt_str = f"{alt:.3f}" if alt is not None else ""
        log_writer.writerow(
            [f"{time.monotonic() - started:.3f}", alt_str, state.flight_mode]
        )
        log_file.flush()

    drone = System()
    state = TelemetryState()
    tracker_tasks = []

    try:
        print(
            f"[m0] Connecting to {SYSTEM_ADDRESS} (timeout {CONNECT_TIMEOUT_S}s)..."
        )
        await drone.connect(system_address=SYSTEM_ADDRESS)
        await wait_for_connection(drone, CONNECT_TIMEOUT_S)
        print("[m0] Connected.")

        tracker_tasks = [
            asyncio.create_task(track_position(drone, state)),
            asyncio.create_task(track_flight_mode(drone, state)),
            asyncio.create_task(track_armed(drone, state)),
            asyncio.create_task(track_landed_state(drone, state)),
        ]

        print(
            "[m0] Waiting for health (is_global_position_ok, is_home_position_ok), "
            f"timeout {HEALTH_TIMEOUT_S}s..."
        )
        await wait_for_health(drone, HEALTH_TIMEOUT_S)
        print("[m0] Health OK.")

        print(f"[m0] Setting takeoff altitude to {TAKEOFF_ALTITUDE_M} m...")
        await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE_M)

        print("[m0] Arming...")
        await drone.action.arm()
        log_row()

        print("[m0] Commanding takeoff...")
        await drone.action.takeoff()

        success_altitude = TAKEOFF_ALTITUDE_M * ALTITUDE_SUCCESS_FRACTION
        print(
            f"[m0] Waiting for relative_altitude_m >= {success_altitude:.2f} m "
            f"({ALTITUDE_SUCCESS_FRACTION * 100:.0f}% of {TAKEOFF_ALTITUDE_M} m "
            f"target), timeout {ALTITUDE_TIMEOUT_S}s..."
        )
        altitude_reached = False
        deadline = time.monotonic() + ALTITUDE_TIMEOUT_S
        while time.monotonic() < deadline:
            log_row()
            if (
                state.relative_altitude_m is not None
                and state.relative_altitude_m >= success_altitude
            ):
                altitude_reached = True
                break
            await asyncio.sleep(1.0 / LOG_POLL_HZ)

        if not altitude_reached:
            log_row()
            raise RuntimeError(
                f"altitude target not reached within {ALTITUDE_TIMEOUT_S}s "
                f"(last relative_altitude_m={state.relative_altitude_m})"
            )

        print(f"[m0] Altitude reached: {state.relative_altitude_m:.2f} m")

        print("[m0] Commanding land...")
        await drone.action.land()

        print(f"[m0] Waiting for landed/disarmed, timeout {LAND_TIMEOUT_S}s...")
        landed = False
        deadline = time.monotonic() + LAND_TIMEOUT_S
        while time.monotonic() < deadline:
            log_row()
            if state.landed_state == LandedState.ON_GROUND or state.armed is False:
                landed = True
                break
            await asyncio.sleep(1.0 / LOG_POLL_HZ)

        if not landed:
            log_row()
            raise RuntimeError(
                f"vehicle did not report landed/disarmed within {LAND_TIMEOUT_S}s "
                f"(last landed_state={state.landed_state}, armed={state.armed})"
            )

        log_row()
        print("[m0] Landed / disarmed. M0 takeoff-and-land check PASSED.")
        return 0

    except asyncio.TimeoutError as exc:
        print(f"[m0] FAILED: timed out waiting for a stage: {exc}")
        return 1
    except ActionError as exc:
        print(f"[m0] FAILED: MAVSDK action error: {exc}")
        return 1
    except RuntimeError as exc:
        print(f"[m0] FAILED: {exc}")
        return 1
    except Exception as exc:  # noqa: BLE001 - top-level gate script, log and exit
        print(f"[m0] FAILED: unexpected error: {exc!r}")
        return 1
    finally:
        for task in tracker_tasks:
            task.cancel()
        for task in tracker_tasks:
            try:
                await task
            except asyncio.CancelledError:
                pass
            except Exception:
                pass
        log_file.close()
        print(f"[m0] Log written to {log_path}")


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
