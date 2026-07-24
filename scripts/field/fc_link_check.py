#!/usr/bin/env python3
"""fc_link_check.py -- READ-ONLY MAVSDK link check against a real flight controller.

Used by scripts/field/03_fc_bench_check.sh. It connects to the flight
controller, reads identity + telemetry + health, prints an arming-readiness
READOUT, and disconnects. It is the props-off "can my laptop talk to the FC"
proof that comes BEFORE the OFFBOARD bench gate (scripts/check_deploy_bench.sh,
build_tab step brn-05).

IT CANNOT ARM. THAT IS ENFORCED, NOT PROMISED.
---------------------------------------------
Arming, takeoff, offboard, manual control and parameter writes all live behind
MAVSDK plugins this file never touches (`drone.action.*`, `drone.offboard.*`,
`drone.manual_control.*`, `drone.param.*`, `drone.actuator_*`). At startup the
script parses its OWN source with `ast` and refuses to run if any call into
those plugins exists (`audit_no_actuation`). A comment or a docstring cannot
fool it -- the audit walks the syntax tree, not the text. `--self-test` runs
that audit on its own and exits 0/1 with no hardware. This mirrors the repo's
existing no-gt AST audits (CLAUDE.md honesty boundary; audit finding DEEP-N1).

The only MAVSDK surfaces used are `core` (connection state), `info` (identity)
and `telemetry` (read-only subscriptions).

WHY 57600 OVER USB: scripts/check_deploy_bench.sh documents the real link as
Pi<->TELEM2 at 921600, with "USB fallback: /dev/ttyACM0 @ 57600". A USB CDC-ACM
port ignores the baud rate anyway, but MAVSDK's serial:// URL requires one.

Usage:
  fc_link_check.py --url serial:///dev/ttyACM0:57600
  fc_link_check.py --url udpin://0.0.0.0:14540      # e.g. against SITL
  fc_link_check.py --self-test                      # no hardware, exits 0/1

Exit: 0 PASS (connected + telemetry read) | 1 FAIL (no link / no telemetry)
      | 2 usage or SAFETY AUDIT failure.
"""
from __future__ import annotations

import argparse
import ast
import asyncio
import json
import os
import sys

# MAVSDK plugin namespaces that can MOVE the aircraft or change its config.
# Any call whose dotted name touches one of these is forbidden in this file.
FORBIDDEN_PLUGINS = ("action", "offboard", "manual_control", "param",
                     "actuator_control", "actuator_test", "mission",
                     "mission_raw", "gimbal", "shell")


def _dotted(node: ast.AST) -> str:
    """Best-effort dotted name for an ast expression (a.b.c(...) -> 'a.b.c')."""
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if isinstance(node, ast.Name):
        parts.append(node.id)
    return ".".join(reversed(parts))


def audit_no_actuation(path: str | None = None):
    """Return a list of forbidden calls found in `path` (default: this file).

    Walks the AST, so text inside comments/docstrings is invisible to it. This
    is the mechanical proof that this tool cannot arm or command the vehicle.
    """
    path = path or os.path.abspath(__file__)
    with open(path) as fh:
        tree = ast.parse(fh.read(), filename=path)
    bad = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = _dotted(node.func)
        # Rule 1: no direct call into a forbidden plugin namespace.
        if name and any(p in name.split(".") for p in FORBIDDEN_PLUGINS):
            bad.append(f"line {node.lineno}: {name}()")
        # Rule 2: no DYNAMIC dispatch on the vehicle handle -- getattr(drone...)
        # would let a call slip past rule 1, so it is banned outright.
        if name == "getattr" and node.args:
            target = _dotted(node.args[0])
            if target.split(".")[:1] == ["drone"]:
                bad.append(f"line {node.lineno}: getattr({target}, ...) "
                           f"-- dynamic dispatch on the vehicle handle")
    return bad


async def _first(stream, timeout):
    """First item from a MAVSDK async subscription, or None on timeout.

    MAVSDK telemetry is a stream; a bench check wants one sample. We close the
    generator explicitly so no subscription is left running at exit.
    """
    agen = stream.__aiter__()
    try:
        return await asyncio.wait_for(agen.__anext__(), timeout)
    except (asyncio.TimeoutError, StopAsyncIteration):
        return None
    finally:
        close = getattr(agen, "aclose", None)
        if close is not None:
            try:
                await close()
            except Exception:  # pragma: no cover - generator already finished
                pass


def _stop_backend(drone) -> None:
    """Kill the mavsdk_server child this System spawned.

    MAVSDK-Python launches a `mavsdk_server` subprocess per System(); if we bail
    out on a timeout it would otherwise linger. `_server_process` is MAVSDK's
    own handle (mavsdk/system.py) -- accessed defensively.
    """
    try:
        proc = drone._server_process
    except AttributeError:
        return
    if proc is not None:
        try:
            proc.kill()
        except Exception:      # pragma: no cover - already dead
            pass


async def run(url: str, connect_timeout: float, sample_timeout: float) -> tuple[int, dict]:
    from mavsdk import System

    report: dict = {"url": url, "connected": False}
    drone = System()
    print(f"[fc-link] connecting to {url} ...")

    # THE TRAP THIS AVOIDS (measured on this machine, 2026-07-24): `mavsdk_server`
    # does not open its gRPC port until it has DISCOVERED a vehicle -- with
    # nothing on the other end of the link it just logs "Waiting to discover
    # system on <url>...". MAVSDK-Python's `connect()` awaits `channel_ready()`,
    # so on a dead link connect() itself BLOCKS FOREVER, before any of the usual
    # connection_state() timeouts can fire. So the timeout has to wrap connect().
    try:
        await asyncio.wait_for(drone.connect(system_address=url), timeout=connect_timeout)
    except asyncio.TimeoutError:
        print(f"[fc-link] FAIL: no MAVLink heartbeat within {connect_timeout:.0f} s "
              f"(nothing is talking on {url})")
        _stop_backend(drone)
        return 1, report

    async def _wait_connected():
        async for st in drone.core.connection_state():
            if st.is_connected:
                return True
        return False

    try:
        await asyncio.wait_for(_wait_connected(), timeout=sample_timeout)
    except asyncio.TimeoutError:
        print("[fc-link] FAIL: backend came up but reports no connected system")
        _stop_backend(drone)
        return 1, report
    report["connected"] = True
    print("[fc-link] connected -- heartbeat is alive")

    # ---- identity ---------------------------------------------------------
    # Written out explicitly (no getattr dispatch) so the AST safety audit can
    # see every single call this file makes on the vehicle handle.
    async def _try(label, awaitable):
        try:
            val = await asyncio.wait_for(awaitable, sample_timeout)
            print(f"[fc-link] {label:14s}: {val}")
            report[label] = str(val)
        except Exception as exc:                      # older/other autopilots
            print(f"[fc-link] {label:14s}: unavailable ({type(exc).__name__})")

    await _try("version", drone.info.get_version())
    await _try("product", drone.info.get_product())
    await _try("identification", drone.info.get_identification())

    # ---- state ------------------------------------------------------------
    armed = await _first(drone.telemetry.armed(), sample_timeout)
    mode = await _first(drone.telemetry.flight_mode(), sample_timeout)
    att = await _first(drone.telemetry.attitude_euler(), sample_timeout)
    gps = await _first(drone.telemetry.gps_info(), sample_timeout)
    bat = await _first(drone.telemetry.battery(), sample_timeout)
    health = await _first(drone.telemetry.health(), sample_timeout)

    print(f"[fc-link] armed         : {armed}   <- this tool never changes it")
    print(f"[fc-link] flight mode   : {mode}")
    if att is not None:
        print(f"[fc-link] attitude      : roll {att.roll_deg:+.1f} deg  "
              f"pitch {att.pitch_deg:+.1f} deg  yaw {att.yaw_deg:+.1f} deg")
        print("[fc-link]   (tilt the board -- these numbers must move sensibly)")
    if gps is not None:
        print(f"[fc-link] gps           : {gps.num_satellites} sats, fix {gps.fix_type}")
    if bat is not None:
        print(f"[fc-link] battery       : {bat.voltage_v:.2f} V "
              f"({bat.remaining_percent}) [USB-powered on the bench: expect ~0/low]")

    report.update({"armed": armed, "flight_mode": str(mode),
                   "gps_num_satellites": getattr(gps, "num_satellites", None),
                   "gps_fix_type": str(getattr(gps, "fix_type", None)),
                   "battery_voltage_v": getattr(bat, "voltage_v", None)})

    # ---- health readout (NOT an arming command) ---------------------------
    rc = 0
    if health is None:
        print("[fc-link] FAIL: no telemetry health sample -- link is up but the "
              "autopilot is not publishing")
        rc = 1
    else:
        flags = {
            "gyrometer calibration": health.is_gyrometer_calibration_ok,
            "accelerometer calibration": health.is_accelerometer_calibration_ok,
            "magnetometer calibration": health.is_magnetometer_calibration_ok,
            "local position": health.is_local_position_ok,
            "global position": health.is_global_position_ok,
            "home position": health.is_home_position_ok,
            "armable (autopilot's own flag)": health.is_armable,
        }
        print("[fc-link] health readout (read-only):")
        for k, v in flags.items():
            print(f"[fc-link]   {'OK  ' if v else '--  '} {k}")
        report["health"] = {k: bool(v) for k, v in flags.items()}
        if not flags["gyrometer calibration"] or not flags["accelerometer calibration"]:
            print("[fc-link]   -> run Sensor Setup in QGroundControl "
                  "(gyro/accel calibration) before any flight")
        if not flags["global position"]:
            print("[fc-link]   -> no GPS fix indoors. Expected on a desk; the "
                  "OFFBOARD bench gate DOES need a fix (run it by a window/outdoors, "
                  "configs/px4_6cmini/README.md)")

    _stop_backend(drone)      # leave no mavsdk_server child behind
    return rc, report


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", help="MAVSDK url, e.g. serial:///dev/ttyACM0:57600")
    ap.add_argument("--connect-timeout", type=float, default=30.0)
    ap.add_argument("--sample-timeout", type=float, default=8.0)
    ap.add_argument("--json-out", default=None)
    ap.add_argument("--self-test", action="store_true",
                    help="run the safety audit only; no hardware; exit 0/1")
    args = ap.parse_args()

    # --- safety audit: this file must contain no actuation calls ------------
    violations = audit_no_actuation()
    if violations:
        print("[fc-link] SAFETY AUDIT FAILED -- this tool must never command the "
              "vehicle, but it calls:", file=sys.stderr)
        for v in violations:
            print(f"[fc-link]   {v}", file=sys.stderr)
        return 2
    print(f"[fc-link] safety audit OK: no calls into {', '.join(FORBIDDEN_PLUGINS)} "
          f"(AST-verified) -- this tool CANNOT arm the vehicle")

    if args.self_test:
        print("[fc-link] self-test PASS (audit only, no hardware touched)")
        return 0
    if not args.url:
        print("[fc-link] usage: --url serial:///dev/ttyACM0:57600 (or --self-test)",
              file=sys.stderr)
        return 2

    try:
        rc, report = asyncio.run(run(args.url, args.connect_timeout, args.sample_timeout))
    except KeyboardInterrupt:
        return 1
    if args.json_out:
        os.makedirs(os.path.dirname(os.path.abspath(args.json_out)), exist_ok=True)
        with open(args.json_out, "w") as fh:
            json.dump(report, fh, indent=2, default=str)
    return rc


if __name__ == "__main__":
    sys.exit(main())
