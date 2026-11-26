#!/usr/bin/env python3
"""flight/deploy/board_overrides.py -- the board settings a program-flown
intercept changes for the length of the flight, and puts back afterwards.

WHY (builder ruling 2026-11-26: "ensure low battery triggers won't activate
once intercept go is activated, and other triggers like gps falling some
(unless it literally zero)"): the flight program has no battery or GPS-quality
trigger of its own. The PIXHAWK does. When one of its failsafes acts it leaves
OFFBOARD, the program sees that (FAILSAFE 7) and the intercept is over. PX4 has
no "only before GO" switch for these, so the program changes the setting while
the aircraft sits DISARMED on the ground, and restores it after landing.

    setting            board   flight   what the flight value means
    COM_LOW_BAT_ACT      3       0      low battery: a warning, no return/land.
                                        The voltage sag of a full-power dash
                                        (77 A against 8.8 A in hover) reads as
                                        an empty pack to a voltage-only gauge.
    COM_FLTT_LOW_ACT     3       0      "not enough battery to fly home": no
                                        action. Inert today (no capacity set),
                                        live the day BAT1_CAPACITY returns.
    EKF2_GPS_CHECK     1023      0      GPS quality checks off; only losing
                                        the 3D fix stops GPS use. With them on,
                                        fewer than 6 satellites or a speed
                                        accuracy past 1.0 m/s for 5 s stops
                                        GPS use, velocity goes invalid and PX4
                                        leaves OFFBOARD
                                        (PX4 v1.16 ekf2 gps_control.cpp,
                                        gps_checks.cpp; offboardCheck.cpp).

WHAT REPLACES THEM, so nothing is simply switched off:
  * battery BEFORE GO and AFTER the intercept: the flight program's own rule
    (real_flight.MissionConfig.battery_low_frac): no arming on a low pack, a
    low pack in the standby hover lands, a low pack in the after-miss hover
    lands. Between GO and the end of the engagement it is ignored, as ruled.
  * GPS quality BEFORE takeoff: the program applies the board's own thresholds
    itself (real_flight.gps_prearm_refusals) before it changes the setting.

SAFETY OF THE WRITES THEMSELVES:
  * written only while the vehicle reports DISARMED; never in flight;
  * the originals are saved to a file on this computer BEFORE the first write;
  * every write is read back; with a backup link the read-back uses the OTHER
    link, so it cannot be answered from the writing server's memory;
  * restored after landing, and again at the start of the next run if a run
    died first (`pending()`); `scripts/field/recorder_check.py` says NO-GO
    while the file exists.
  * ONLY the names in OVERRIDES can be written by this module (a test pins the
    list). It sends no command and no setpoint.

    status:   python -m flight.deploy.board_overrides --status
    restore:  python -m flight.deploy.board_overrides --restore --url serial:///dev/ttyAMA10:921600
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import time
from typing import Callable, Dict, List, Optional, Tuple

# (name, flight value, plain meaning). All INT32 on PX4 v1.16.
OVERRIDES: Tuple[Tuple[str, int, str], ...] = (
    ("COM_LOW_BAT_ACT", 0, "low battery: warning only"),
    ("COM_FLTT_LOW_ACT", 0, "not enough battery to fly home: no action"),
    ("EKF2_GPS_CHECK", 0, "GPS quality checks off; loss of the 3D fix still "
                          "stops GPS use"),
)
DEFAULT_STATE_PATH = os.path.expanduser("~/.interceptor/board_overrides.json")
_PARAM_TIMEOUT_S = 5.0


def pending(state_path: str = DEFAULT_STATE_PATH) -> Optional[dict]:
    """The saved originals of a flight configuration that was never restored,
    or None. A file that cannot be read counts as pending (fail closed)."""
    if not os.path.exists(state_path):
        return None
    try:
        with open(state_path) as fh:
            d = json.load(fh)
        if isinstance(d, dict) and isinstance(d.get("originals"), dict):
            return d
    except (OSError, ValueError):
        pass
    return {"originals": {}, "unreadable": True}


def _save(state_path: str, originals: Dict[str, int]) -> None:
    os.makedirs(os.path.dirname(state_path) or ".", exist_ok=True)
    tmp = state_path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump({"written_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                                time.gmtime()),
                   "originals": originals,
                   "flight_values": {n: v for n, v, _ in OVERRIDES}}, fh)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, state_path)


async def _get(drone, name: str) -> int:
    return int(await asyncio.wait_for(drone.param.get_param_int(name),
                                      timeout=_PARAM_TIMEOUT_S))


async def _set(drone, name: str, value: int) -> None:
    await asyncio.wait_for(drone.param.set_param_int(name, int(value)),
                           timeout=_PARAM_TIMEOUT_S)


async def _write_and_verify(drone, verify_drone, wanted: Dict[str, int],
                            say: Callable[[str], None]) -> List[str]:
    """Write each value and read it back. Returns the names that did NOT end
    at the wanted value (empty = all good)."""
    bad: List[str] = []
    for name, value in wanted.items():
        try:
            await _set(drone, name, value)
            got = await _get(verify_drone, name)
        except Exception as e:  # noqa: BLE001 -- any failure is "not confirmed"
            say(f"[board] {name}: write or read-back failed "
                f"({type(e).__name__}: {e})")
            bad.append(name)
            continue
        if got != int(value):
            say(f"[board] {name}: wrote {value}, read back {got}")
            bad.append(name)
        else:
            say(f"[board] {name} = {got} (read back)")
    return bad


async def restore(drone, armed: Optional[bool], *, verify_drone=None,
                  state_path: str = DEFAULT_STATE_PATH,
                  say: Callable[[str], None] = print) -> bool:
    """Put the saved originals back. True when nothing is pending afterwards.
    Refuses while the vehicle is not known to be disarmed: restoring the
    battery action in the air could turn the aircraft for home under a pilot
    who has just taken it."""
    saved = pending(state_path)
    if saved is None:
        return True
    if saved.get("unreadable"):
        say(f"[board] {state_path} exists but cannot be read. The board may "
            f"still hold the flight values of: "
            f"{', '.join(n for n, _, _ in OVERRIDES)}. Check them by hand "
            f"(QGroundControl), then delete the file.")
        return False
    if armed is not False:
        say(f"[board] NOT restoring while armed={armed}: the flight values "
            f"stay until the aircraft is disarmed. Run "
            f"`python -m flight.deploy.board_overrides --restore` after "
            f"landing.")
        return False
    originals = {n: int(v) for n, v in saved["originals"].items()
                 if n in {o[0] for o in OVERRIDES}}
    bad = await _write_and_verify(drone, verify_drone or drone, originals, say)
    if bad:
        say(f"[board] RESTORE INCOMPLETE: {', '.join(bad)} not confirmed. "
            f"The originals are still saved in {state_path}.")
        return False
    try:
        os.remove(state_path)
    except OSError as e:
        say(f"[board] restored, but {state_path} could not be removed: {e}")
        return False
    say("[board] board settings restored to their saved values")
    return True


async def apply(drone, armed: Optional[bool], *, verify_drone=None,
                state_path: str = DEFAULT_STATE_PATH,
                say: Callable[[str], None] = print) -> bool:
    """Save the originals, write the flight values, read them back. True only
    when every flight value is confirmed. On any failure it tries to restore
    and returns False (the caller then does not arm)."""
    if armed is not False:
        say(f"[board] REFUSED: vehicle reports armed={armed}; the flight "
            f"values are only ever written on a disarmed aircraft")
        return False
    if pending(state_path) is not None:
        say("[board] an earlier run left the flight values on the board; "
            "restoring them first")
        if not await restore(drone, armed, verify_drone=verify_drone,
                             state_path=state_path, say=say):
            return False
    originals: Dict[str, int] = {}
    for name, _value, _what in OVERRIDES:
        try:
            originals[name] = await _get(drone, name)
        except Exception as e:  # noqa: BLE001 -- fail closed: never guess an original
            say(f"[board] REFUSED: could not read {name} "
                f"({type(e).__name__}: {e}); nothing was changed")
            return False
    try:
        _save(state_path, originals)
    except OSError as e:
        say(f"[board] REFUSED: could not save the originals to {state_path} "
            f"({e}); nothing was changed")
        return False
    wanted = {n: v for n, v, _ in OVERRIDES}
    for name, value, what in OVERRIDES:
        say(f"[board] {name}: {originals[name]} -> {value} for this flight "
            f"({what})")
    bad = await _write_and_verify(drone, verify_drone or drone, wanted, say)
    if bad:
        say(f"[board] REFUSED: {', '.join(bad)} not confirmed; putting the "
            f"originals back")
        await restore(drone, armed, verify_drone=verify_drone,
                      state_path=state_path, say=say)
        return False
    return True


def summary_lines() -> List[str]:
    """What the pre-arm summary says about this module."""
    out = ["[fly] for this flight the program changes three board settings "
           "(disarmed, read back) and restores them after landing:"]
    for name, value, what in OVERRIDES:
        out.append(f"[fly]   {name} -> {value}: {what}")
    return out


# ---------------------------------------------------------------- CLI

async def _cli(args) -> int:
    if args.status:
        saved = pending(args.state_path)
        if saved is None:
            print("[board] nothing pending: the board holds its own values")
            return 0
        print(f"[board] PENDING since {saved.get('written_utc', '?')}: the "
              f"board may hold flight values. Saved originals: "
              f"{saved.get('originals')}")
        return 1
    from mavsdk import System
    drone = System()
    print(f"[board] connecting {args.url} ...")
    await asyncio.wait_for(drone.connect(system_address=args.url), timeout=30.0)
    armed = None

    async def _armed():
        async for a in drone.telemetry.armed():
            return a

    try:
        armed = await asyncio.wait_for(_armed(), timeout=10.0)
    except asyncio.TimeoutError:
        print("[board] no armed/disarmed report within 10 s")
    try:
        if args.restore:
            ok = await restore(drone, armed, state_path=args.state_path)
        else:
            ok = await apply(drone, armed, state_path=args.state_path)
        return 0 if ok else 1
    finally:
        proc = getattr(drone, "_server_process", None)
        if proc is not None and proc.poll() is None:
            proc.kill()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--status", action="store_true",
                   help="say whether flight values are pending (no link needed)")
    g.add_argument("--restore", action="store_true",
                   help="put the saved originals back (disarmed only)")
    g.add_argument("--apply", action="store_true",
                   help="bench rehearsal: write the flight values (disarmed "
                        "only); follow with --restore")
    ap.add_argument("--url", default="serial:///dev/ttyAMA10:921600")
    ap.add_argument("--state-path", default=DEFAULT_STATE_PATH)
    args = ap.parse_args(argv)
    return asyncio.run(_cli(args))


if __name__ == "__main__":
    sys.exit(main())
