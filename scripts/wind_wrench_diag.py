#!/usr/bin/env python3
"""WRENCH DIAGNOSTIC -- WHICH publish recipe actually moves the airframe?

WHY THIS EXISTS
    The Gate-0 probe (scripts/wind_gate0_probe.py) returned NULL on
    2026-08-29: scripts/wind_driver.py published 598 persistent wrenches at
    1.909 N, gz-sim's ApplyLinkWrench system was loaded, no error appeared in
    the server log -- and the aircraft tilted 0.167 deg against a predicted
    5.262 deg. Essentially zero force arrived.

    "Gazebo does not apply the wrench" is a conclusion with real consequences
    (ADR-0096's whole wind capability rests on it), so it must not be drawn
    from a single recipe. There are several independent candidate causes and
    they are cheap to separate. This script separates them.

THE CANDIDATES
    A  nothing            -- baseline, so every arm has its own control
    B  persistent, scoped name  ("x500_mono_cam_0::base_link"), published ONCE
    C  persistent, bare link name ("base_link"), published ONCE
    D  persistent, scoped, published ONCE -- entity addressed by ID if resolvable
    E  one-shot topic /world/<w>/wrench at 20 Hz, scoped name
    F  clear+publish at 20 Hz -- THE CURRENT DRIVER RECIPE, reproduced exactly

    B/C/D vs F separates ENTITY RESOLUTION from the clear-race. If B tilts and
    F does not, the per-tick clear is destroying the force (the two messages go
    out on DIFFERENT topics, and gz-transport gives no ordering guarantee
    across topics -- so "clear then publish" can arrive as "publish then
    clear", leaving nothing applied). If NOTHING tilts, the persistent-wrench
    route is dead here and ADR-0096's rejected alternatives come back off the
    shelf.

    E is the control on the whole ApplyLinkWrench system: the one-shot topic
    is a different code path into the same plugin. If E moves the aircraft and
    the persistent topic never does, the fault is specifically in the
    persistent path, not in wrench application at all.

    Each arm holds for --arm-sim-s and is followed by a clear + settle, so the
    arms do not contaminate each other. The aircraft hovers in position hold
    throughout: with no external force that requires a level attitude, so any
    sustained tilt is force that arrived.

READ THE RESULT, NOT THE INTENT
    Every arm reports the ground-truth tilt the AIRCRAFT reached. Nothing here
    trusts a publish() return value -- that was exactly the thing that looked
    healthy while nothing happened.

USAGE
    scripts/check_wind_gate0.sh boots the sim; this is normally run the same
    way via scripts/check_wrench_diag.sh, or by hand against a live sim:
        .venv/bin/python scripts/wind_wrench_diag.py --out-dir logs/wrenchdiag
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import sys
import time
from datetime import datetime, timezone

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from wind_gate0_probe import (  # noqa: E402
    GroundTruthPose,
    Telem,
    _track_armed,
    _track_landed,
    _track_position,
    _wait_connected,
    _wait_health,
    record_phase,
    summarise,
)
from wind_model import DragParams  # noqa: E402

WORLD_NAME = os.environ.get("INTERCEPTOR_WORLD_NAME", "apriltag")
INTERCEPTOR_MODEL = os.environ.get("INTERCEPTOR_WIND_MODEL", "x500_mono_cam_0")
INTERCEPTOR_LINK = os.environ.get("INTERCEPTOR_WIND_LINK", "base_link")
G = 9.80665

# The force the real driver would apply for a steady 5 m/s wind from the west:
# it pushes the aircraft EAST, i.e. +world_x in Gazebo's ENU.
DRAG = DragParams.px4_x500_mono_cam_sitl()


class WrenchPublisher:
    """Raw access to both ApplyLinkWrench topics, with no policy of its own."""

    def __init__(self):
        from gz.transport13 import Node                       # noqa: PLC0415
        from gz.msgs10.entity_wrench_pb2 import EntityWrench  # noqa: PLC0415
        from gz.msgs10.entity_pb2 import Entity               # noqa: PLC0415

        self._EntityWrench = EntityWrench
        self._Entity = Entity
        self.node = Node()
        self.persistent = self.node.advertise(
            f"/world/{WORLD_NAME}/wrench/persistent", EntityWrench)
        self.oneshot = self.node.advertise(
            f"/world/{WORLD_NAME}/wrench", EntityWrench)
        self.clear = self.node.advertise(
            f"/world/{WORLD_NAME}/wrench/clear", Entity)

    def _msg(self, name, fx, fy, fz):
        m = self._EntityWrench()
        m.entity.name = name
        m.entity.type = self._Entity.LINK
        m.wrench.force.x = fx
        m.wrench.force.y = fy
        m.wrench.force.z = fz
        return m

    def publish_persistent(self, name, fx, fy, fz):
        return bool(self.persistent.publish(self._msg(name, fx, fy, fz)))

    def publish_oneshot(self, name, fx, fy, fz):
        return bool(self.oneshot.publish(self._msg(name, fx, fy, fz)))

    def clear_entity(self, name):
        e = self._Entity()
        e.name = name
        e.type = self._Entity.LINK
        return bool(self.clear.publish(e))


async def hold_arm(gt, writer, pub, arm, force_n, args):
    """Run one arm for --arm-sim-s of SIM time, return its settled summary."""
    scoped = f"{INTERCEPTOR_MODEL}::{INTERCEPTOR_LINK}"
    fx = force_n          # push EAST (+world_x)
    published = 0
    note = ""

    if arm == "A_baseline":
        note = "no wrench published at all"
    elif arm == "B_persistent_scoped_once":
        published = int(pub.publish_persistent(scoped, fx, 0.0, 0.0))
        note = f"one persistent publish, entity='{scoped}'"
    elif arm == "C_persistent_bare_once":
        published = int(pub.publish_persistent(INTERCEPTOR_LINK, fx, 0.0, 0.0))
        note = f"one persistent publish, entity='{INTERCEPTOR_LINK}'"
    elif arm == "D_persistent_scoped_repeat_noclear":
        note = "persistent republished at 20 Hz, NO clear (accumulation test)"
    elif arm == "E_oneshot_20hz":
        note = "one-shot /wrench topic at 20 Hz"
    elif arm == "F_clear_then_publish_20hz":
        note = "clear+publish at 20 Hz -- the CURRENT wind_driver recipe"

    async def pump():
        """Arms that need a repeating publish drive it here, on wall time at
        20 Hz -- matching the driver's own tick rate. (The driver ticks on sim
        time; this diagnostic only needs a steady stream, and the measurement
        window is on sim time regardless.)"""
        nonlocal published
        while True:
            if arm == "D_persistent_scoped_repeat_noclear":
                published += int(pub.publish_persistent(scoped, fx, 0.0, 0.0))
            elif arm == "E_oneshot_20hz":
                published += int(pub.publish_oneshot(scoped, fx, 0.0, 0.0))
            elif arm == "F_clear_then_publish_20hz":
                pub.clear_entity(scoped)
                published += int(pub.publish_persistent(scoped, fx, 0.0, 0.0))
            else:
                return
            await asyncio.sleep(0.05)

    pump_task = asyncio.create_task(pump())
    try:
        rows = await record_phase(gt, writer, arm, args.arm_sim_s,
                                  args.sample_hz,
                                  wall_budget_s=args.arm_sim_s * 6 + 60)
    finally:
        pump_task.cancel()
        try:
            await pump_task
        except asyncio.CancelledError:
            pass

    # Clear whatever this arm left behind, then let the aircraft re-level so
    # the next arm starts from the same place.
    pub.clear_entity(scoped)
    pub.clear_entity(INTERCEPTOR_LINK)
    await record_phase(gt, writer, f"{arm}__settle", args.settle_between_sim_s,
                       args.sample_hz,
                       wall_budget_s=args.settle_between_sim_s * 6 + 60)

    s = summarise(rows, args.tail_sim_s)
    s["arm"] = arm
    s["note"] = note
    s["publishes"] = published
    return s


async def run(args):
    from mavsdk import System  # noqa: PLC0415
    import csv                 # noqa: PLC0415

    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, "attitude.csv")
    out_path = os.path.join(args.out_dir, "diag.json")

    a_drag = DRAG.drag_accel_m_s2(args.wind_mps)
    force_n = a_drag * DRAG.mass_kg
    predicted_tilt = math.degrees(math.atan(a_drag / G))
    print(f"[diag] force {force_n:.4f} N -> predicted tilt "
          f"{predicted_tilt:.3f} deg", flush=True)

    gt = GroundTruthPose()
    print(f"[diag] pose topics: {gt.start()}", flush=True)
    pub = WrenchPublisher()

    fh = open(csv_path, "w", newline="")
    writer = csv.DictWriter(fh, fieldnames=[
        "phase", "t_sim", "t_phase", "pose_t_sim", "east_m", "north_m", "up_m",
        "tilt_deg", "lean_bearing_deg"])
    writer.writeheader()

    drone = System()
    st = Telem()
    trackers = []
    results = []
    try:
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        await _wait_connected(drone, 60)
        trackers = [asyncio.create_task(_track_position(drone, st)),
                    asyncio.create_task(_track_armed(drone, st)),
                    asyncio.create_task(_track_landed(drone, st))]
        await _wait_health(drone, 120)
        print(f"[diag] taking off to {args.hover_alt_m} m", flush=True)
        await drone.action.set_takeoff_altitude(args.hover_alt_m)
        await drone.action.arm()
        await drone.action.takeoff()
        deadline = time.monotonic() + 120
        while time.monotonic() < deadline:
            if (st.relative_altitude_m is not None
                    and st.relative_altitude_m >= args.hover_alt_m * 0.9):
                break
            await asyncio.sleep(0.2)
        print(f"[diag] at {st.relative_altitude_m:.2f} m, settling", flush=True)
        await record_phase(gt, writer, "settle", args.settle_sim_s,
                           args.sample_hz,
                           wall_budget_s=args.settle_sim_s * 6 + 60)

        for arm in args.arms:
            print(f"[diag] --- {arm} ---", flush=True)
            s = await hold_arm(gt, writer, pub, arm, force_n, args)
            results.append(s)
            print(f"[diag]   tilt {s['mean_tilt_deg']:.3f} deg "
                  f"(max {s['max_tilt_deg']:.3f}), lean "
                  f"{s['mean_lean_bearing_deg']:.1f}, "
                  f"publishes={s['publishes']}, drift "
                  f"{s['drift_within_window_m']:.2f} m", flush=True)

        try:
            await drone.action.land()
        except Exception:                                     # noqa: BLE001
            pass
    finally:
        for t in trackers:
            t.cancel()
        for t in trackers:
            try:
                await t
            except (asyncio.CancelledError, Exception):
                pass
        fh.close()

    baseline = next((r for r in results if r["arm"] == "A_baseline"), None)
    base_tilt = baseline["mean_tilt_deg"] if baseline else 0.0
    # An arm "moved the aircraft" if it reached at least a quarter of the
    # predicted tilt above baseline. Deliberately loose: this is a
    # does-anything-happen screen, not a calibration.
    threshold = base_tilt + 0.25 * predicted_tilt
    for r in results:
        r["moved"] = r["mean_tilt_deg"] > threshold

    out = {
        "utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "force_n": force_n,
        "predicted_tilt_deg": predicted_tilt,
        "baseline_tilt_deg": base_tilt,
        "moved_threshold_deg": threshold,
        "pose_source": gt.read()[2],
        "arms": results,
    }
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2, default=str)

    print("\n" + "=" * 72, flush=True)
    print(f"WRENCH DIAGNOSTIC -- predicted tilt {predicted_tilt:.3f} deg, "
          f"'moved' threshold {threshold:.3f} deg", flush=True)
    print("=" * 72, flush=True)
    for r in results:
        print(f"  {'MOVED  ' if r['moved'] else 'nothing'}  "
              f"{r['arm']:<34} tilt {r['mean_tilt_deg']:6.3f} deg  "
              f"pub={r['publishes']:<5} {r['note']}", flush=True)
    movers = [r["arm"] for r in results if r["moved"]]
    print(f"\n  arms that moved the aircraft: {movers or 'NONE'}", flush=True)
    if not movers:
        print("  -> no recipe applied force. The persistent-wrench route is "
              "not viable here; ADR-0096's rejected alternatives come back "
              "off the shelf.", flush=True)
    print(f"  written to {out_path}", flush=True)
    return 0


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--wind-mps", type=float, default=5.0)
    p.add_argument("--hover-alt-m", type=float, default=6.0)
    p.add_argument("--settle-sim-s", type=float, default=12.0)
    p.add_argument("--arm-sim-s", type=float, default=15.0)
    p.add_argument("--settle-between-sim-s", type=float, default=8.0)
    p.add_argument("--tail-sim-s", type=float, default=8.0)
    p.add_argument("--sample-hz", type=float, default=10.0)
    p.add_argument("--arms", nargs="+", default=[
        "A_baseline",
        "B_persistent_scoped_once",
        "C_persistent_bare_once",
        "D_persistent_scoped_repeat_noclear",
        "E_oneshot_20hz",
        "F_clear_then_publish_20hz",
    ])
    p.add_argument("--out-dir", default=None)
    a = p.parse_args(argv)
    if a.out_dir is None:
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        a.out_dir = os.path.join(REPO_ROOT, "logs", f"wrench_diag_{ts}")
    return a


if __name__ == "__main__":
    sys.exit(asyncio.run(run(parse_args())))
