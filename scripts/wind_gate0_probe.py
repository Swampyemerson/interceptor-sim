#!/usr/bin/env python3
"""WIND GATE-0 (W3) PHYSICS PROBE -- does Gazebo actually apply the wrench?

Pre-registration: docs/wind_gate0_prereg.md (written BEFORE this ran; config,
prediction, criterion and the meaning of a null are fixed there).
Parent decision: ADR-0096, whose closing line is the reason this file exists --
"NOT YET DONE, and nothing may be quoted until it is: the W3 physics probe."

THE QUESTION
    scripts/wind_driver.py publishes a drag force as a persistent EntityWrench.
    Its offline self-test proves the FORCE VECTOR and the PUBLISH PATH. It
    cannot prove that Gazebo puts that force on the airframe -- and the
    driver's own CSV logs what it PUT ON THE WIRE, so an inert wrench would
    produce a perfect-looking log over a windless flight. That is ADR-0096's
    generalisation of the instrument-defect class: a component that logs what
    it commanded rather than what was applied can diverge silently. This probe
    measures what the AIRCRAFT DID.

THE MEASUREMENT
    Hover in position hold. With no external force, holding position requires a
    LEVEL attitude -- there is no other way to sit still. Apply a steady
    horizontal force and the only way to keep holding position is to tilt the
    thrust vector into it. So a sustained tilt at zero groundspeed IS the
    evidence that a force arrived, and its magnitude gives the force:

        theta = atan(a_drag / g),  a_drag = MCOEF*V + rho*V^2/(2*BCOEF)

    Three phases in ONE flight so the control is paired within the run:
      A  driver OFF          -> tilt must be ~0        (baseline)
      B  driver ON, 5 m/s    -> tilt must be ~5.26 deg (the measurement)
      C  driver exited       -> tilt must be ~0 again  (the wrench is REMOVABLE)

    Phase C is not decoration. ApplyLinkWrench persists a wrench until replaced
    or cleared, and the accumulating-wrench defect ADR-0096 fixed lived in this
    exact machinery. A wrench that cannot be removed contaminates every later
    flight in the same sim boot.

ATTITUDE COMES FROM GAZEBO GROUND TRUTH, NOT THE ESTIMATOR
    We read the airframe's true orientation off the world pose topics rather
    than PX4's attitude estimate. The question is whether PHYSICS moved, and
    routing the answer through an estimator inserts an instrument between the
    question and the answer. This is gt_* usage in the sanctioned category --
    scoring/audit, exactly like m4's scorer and the target mover. Nothing here
    feeds guidance; the flight is a plain MAVSDK takeoff/hold/land.

    MATCH THE MODEL ENTITY, NEVER THE LINK. A nested link's pose is reported
    RELATIVE TO ITS ENCLOSING MODEL, so `base_link` reads a constant (0,0,0.24)
    forever. The first run of this probe learned that the expensive way and it
    is ADR-0006's root cause recurring; `pose_motion()` now fails the run closed
    if the pose never moves, so a frozen reader can never again be mistaken for
    a finding about Gazebo.

SIM TIME, NEVER WALL TIME
    Every phase boundary and every window is measured on /clock (ADR-0009). RTF
    sags under load and wall-clock phases would silently shift the windows.

USAGE (normally invoked by scripts/check_wind_gate0.sh, which boots the sim)
    .venv/bin/python scripts/wind_gate0_probe.py --out-dir logs/wind_gate0_<UTC>
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from wind_model import DragParams  # noqa: E402

SYSTEM_ADDRESS = "udpin://0.0.0.0:14540"
WORLD_NAME = os.environ.get("INTERCEPTOR_WORLD_NAME", "apriltag")
INTERCEPTOR_MODEL = os.environ.get("INTERCEPTOR_WIND_MODEL", "x500_mono_cam_0")
INTERCEPTOR_LINK = os.environ.get("INTERCEPTOR_WIND_LINK", "base_link")

G = 9.80665

# --- criteria, from the pre-registration (docs/wind_gate0_prereg.md sec 4) ---
TILT_TOLERANCE_FRAC = 0.30      # +/-30% band on the derived tilt
LEVEL_MAX_DEG = 1.0             # phases A and C must be below this
BEARING_TOLERANCE_DEG = 30.0    # lean must point into the wind within this
MIN_SAMPLES_PER_PHASE = 50      # fewer than this -> VOID, not PASS
MAX_DRIFT_M = 3.0               # position hold lost -> derivation does not apply
FORCE_MATCH_FRAC = 0.10         # commanded force must match the assumed field


# --------------------------------------------------------------- ground truth
class GroundTruthPose:
    """Gazebo's true pose of the interceptor, on sim time.

    Subscribes BOTH pose topics and stays on whichever delivers first
    (interleaving two topics with different stamps would jitter the time base),
    but matches ONLY the top-level MODEL entity -- see make_on_pose. Records
    which topic delivered, and the caller cross-checks that the pose actually
    moved before believing any tilt it reports.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.sim_t = None
        self.sample = None      # (t, east, north, up, tilt_deg, lean_bearing_deg)
        self.source = None
        self.n_updates = 0
        self._node = None

    def start(self):
        from gz.transport13 import Node          # noqa: PLC0415
        from gz.msgs10.clock_pb2 import Clock    # noqa: PLC0415
        from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: PLC0415

        self._node = Node()
        scoped = f"{INTERCEPTOR_MODEL}::{INTERCEPTOR_LINK}"

        def on_clock(msg):
            with self.lock:
                self.sim_t = msg.sim.sec + msg.sim.nsec * 1e-9

        def make_on_pose(source):
            def on_pose(msg):
                # THE MODEL POSE IS THE WORLD POSE. THE LINK POSE IS NOT.
                # A nested link's pose in gz's pose topics is relative to its
                # enclosing MODEL, so base_link reads a constant (0,0,0.24) --
                # x500_base's declared offset -- no matter how the aircraft
                # flies. Measured the hard way on the first run of this probe
                # (logs/wind_gate0_20260829T154442Z): 4789 callbacks, z=0.2400
                # every time, while the aircraft sat at 5.45 m. Same root cause
                # as ADR-0006. Match the model, and only the model.
                best = None
                for p in msg.pose:
                    if p.name == INTERCEPTOR_MODEL:
                        best = p
                        break
                if best is None:
                    return
                with self.lock:
                    if self.source not in (None, source):
                        return
                    t = None
                    if msg.header.stamp.sec or msg.header.stamp.nsec:
                        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
                    if t is None:
                        t = self.sim_t
                    if t is None:
                        return
                    q = best.orientation
                    tilt, bearing = tilt_from_quaternion(q.w, q.x, q.y, q.z)
                    # Gazebo world is ENU: x = east, y = north, z = up.
                    self.sample = (t, best.position.x, best.position.y,
                                   best.position.z, tilt, bearing)
                    self.source = source
                    self.n_updates += 1
            return on_pose

        if not self._node.subscribe(Clock, "/clock", on_clock):
            raise RuntimeError("could not subscribe /clock")
        subscribed = []
        for topic in (f"/world/{WORLD_NAME}/pose/info",
                      f"/world/{WORLD_NAME}/dynamic_pose/info"):
            if self._node.subscribe(Pose_V, topic, make_on_pose(topic)):
                subscribed.append(topic)
        if not subscribed:
            raise RuntimeError(
                f"could not subscribe either pose topic for world '{WORLD_NAME}'")
        return subscribed

    def read(self):
        with self.lock:
            return self.sim_t, self.sample, self.source, self.n_updates


def tilt_from_quaternion(w, x, y, z):
    """(tilt_deg, lean_bearing_deg) of the thrust axis, from a Gazebo ENU quat.

    The multirotor's thrust acts along its body +z. Rotating the body z-axis
    into the world gives the direction the thrust points:

        u = R(q) * [0, 0, 1]  ->  (u_east, u_north, u_up)

    tilt is the angle off vertical; the lean bearing is the compass bearing of
    the horizontal component -- i.e. the direction the aircraft leans TOWARD.
    To hold position against a wind from the west, the thrust must lean west,
    so the expected lean bearing EQUALS the meteorological --dir-from-deg.
    """
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n == 0.0:
        return float("nan"), float("nan")
    w, x, y, z = w / n, x / n, y / n, z / n
    # Third column of the rotation matrix = body z-axis expressed in world ENU.
    u_e = 2.0 * (x * z + w * y)
    u_n = 2.0 * (y * z - w * x)
    u_u = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, u_u))))
    bearing = wrap360(math.degrees(math.atan2(u_e, u_n)))
    return tilt, bearing


def wrap360(deg):
    """Wrap to [0, 360). Guards the float artefact where atan2 returns a tiny
    NEGATIVE (e.g. -5e-15) for a due-north bearing: plain `% 360.0` then
    rounds up to exactly 360.0, which reads as a full turn rather than zero."""
    if math.isnan(deg):
        return float("nan")
    deg = deg % 360.0
    return 0.0 if deg >= 360.0 else deg


def angle_diff_deg(a, b):
    """Smallest signed difference a-b, wrapped to [-180, 180]."""
    return (a - b + 180.0) % 360.0 - 180.0


def circular_mean_deg(values):
    if not values:
        return float("nan")
    s = sum(math.sin(math.radians(v)) for v in values)
    c = sum(math.cos(math.radians(v)) for v in values)
    if s == 0.0 and c == 0.0:
        return float("nan")
    return wrap360(math.degrees(math.atan2(s, c)))


# ------------------------------------------------------------------- MAVSDK
class Telem:
    def __init__(self):
        self.relative_altitude_m = None
        self.armed = None
        self.landed_state = None


async def _track_position(drone, st):
    async for p in drone.telemetry.position():
        st.relative_altitude_m = p.relative_altitude_m


async def _track_armed(drone, st):
    async for a in drone.telemetry.armed():
        st.armed = a


async def _track_landed(drone, st):
    async for s in drone.telemetry.landed_state():
        st.landed_state = s


async def _wait_connected(drone, timeout_s):
    async def _w():
        async for cs in drone.core.connection_state():
            if cs.is_connected:
                return
    await asyncio.wait_for(_w(), timeout=timeout_s)


async def _wait_health(drone, timeout_s):
    async def _w():
        async for h in drone.telemetry.health():
            if h.is_global_position_ok and h.is_home_position_ok:
                return
    await asyncio.wait_for(_w(), timeout=timeout_s)


# ------------------------------------------------------------------ recorder
async def record_phase(gt, writer, phase, sim_seconds, sample_hz, wall_budget_s):
    """Sample ground-truth pose for `sim_seconds` of SIM time.

    Returns the list of rows. Fails closed on a wall-clock budget so a stalled
    or absent /clock cannot hang the probe forever -- a phase that never
    advanced in sim time is a VOID run, not a silent short window.
    """
    rows = []
    t0 = None
    wall_deadline = time.monotonic() + wall_budget_s
    period = 1.0 / sample_hz
    while True:
        sim_t, sample, _src, _n = gt.read()
        if sample is not None:
            t, e, n, u, tilt, bearing = sample
            now = sim_t if sim_t is not None else t
            if t0 is None:
                t0 = now
            elapsed = now - t0
            row = {
                "phase": phase,
                "t_sim": f"{now:.4f}",
                "t_phase": f"{elapsed:.4f}",
                "pose_t_sim": f"{t:.4f}",
                "east_m": f"{e:.4f}",
                "north_m": f"{n:.4f}",
                "up_m": f"{u:.4f}",
                "tilt_deg": f"{tilt:.4f}",
                "lean_bearing_deg": f"{bearing:.3f}",
            }
            writer.writerow(row)
            rows.append({"t_sim": now, "t_phase": elapsed, "east_m": e,
                         "north_m": n, "up_m": u, "tilt_deg": tilt,
                         "lean_bearing_deg": bearing})
            if elapsed >= sim_seconds:
                return rows
        if time.monotonic() > wall_deadline:
            print(f"[gate0] WARNING: phase '{phase}' hit its wall budget "
                  f"({wall_budget_s:.0f} s) after {len(rows)} samples -- sim "
                  f"time did not advance as expected.", flush=True)
            return rows
        await asyncio.sleep(period)


def summarise(rows, tail_sim_s):
    """Mean tilt / lean / drift over the LAST `tail_sim_s` sim seconds."""
    if not rows:
        return None
    t_end = rows[-1]["t_phase"]
    window = [r for r in rows if r["t_phase"] >= t_end - tail_sim_s]
    if not window:
        return None
    tilts = [r["tilt_deg"] for r in window]
    # Lean bearing is meaningless when the aircraft is level; weight it out by
    # only averaging samples with a real tilt. A bearing computed from numerical
    # noise at 0.01 deg would be a fabricated direction.
    bearings = [r["lean_bearing_deg"] for r in window if r["tilt_deg"] > 0.2]
    e0 = window[0]["east_m"]
    n0 = window[0]["north_m"]
    drift = max(math.hypot(r["east_m"] - e0, r["north_m"] - n0) for r in window)
    return {
        "n_samples_phase": len(rows),
        "n_samples_window": len(window),
        "window_sim_s": round(t_end - window[0]["t_phase"], 3),
        "mean_tilt_deg": sum(tilts) / len(tilts),
        "max_tilt_deg": max(tilts),
        "min_tilt_deg": min(tilts),
        "mean_lean_bearing_deg": circular_mean_deg(bearings),
        "n_bearing_samples": len(bearings),
        "drift_within_window_m": drift,
        "mean_up_m": sum(r["up_m"] for r in window) / len(window),
    }


def read_driver_csv(path):
    """Mean commanded force magnitude and the publish tallies from the driver.

    The driver's CSV opens with '#' provenance banner lines before the real
    header. Feeding those straight to DictReader makes the FIRST BANNER the
    header row, every lookup then KeyErrors, and the reader reports "0 rows" for
    a file with 599 good ones -- which is exactly what happened on the first
    probe run, producing a VOID over a driver that had worked perfectly. A
    parser that mis-reads a healthy file is an instrument defect like any other.
    """
    if not os.path.exists(path):
        return None
    forces, published, failed = [], 0, 0
    with open(path, newline="") as fh:
        lines = [ln for ln in fh if not ln.lstrip().startswith("#")]
        for row in csv.DictReader(lines):
            try:
                fn = float(row["applied_f_n"])
                fe = float(row["applied_f_e"])
            except (KeyError, ValueError):
                continue
            forces.append(math.hypot(fn, fe))
            if row.get("published") in ("1", "True", "true"):
                published += 1
            if row.get("publish_error"):
                failed += 1
    if not forces:
        return {"rows": 0, "published": 0, "publish_failed": failed,
                "mean_force_n": float("nan")}
    return {
        "rows": len(forces),
        "published": published,
        "publish_failed": failed,
        "mean_force_n": sum(forces) / len(forces),
        "max_force_n": max(forces),
    }


# ------------------------------------------------------------------ verdict
def pose_motion(all_rows):
    """Total span of the ground-truth pose over the WHOLE run.

    A flying aircraft's pose moves. If the span is exactly zero the reader is
    latched onto something that does not move -- a model-relative link pose,
    a stale cache, the wrong entity -- and every tilt it reported is an
    artefact. This is the check that was MISSING on the first run: with the
    reader frozen at (0,0,0.24), phases A/B/C all read 0.000 deg, and the
    probe was one CSV-parser bug away from reporting a confident NULL and
    condemning a wind driver that had worked perfectly. A false null is more
    expensive than no result.
    """
    if not all_rows:
        return {"span_east_m": 0.0, "span_north_m": 0.0, "span_up_m": 0.0,
                "span_tilt_deg": 0.0, "n": 0}
    def span(key):
        vals = [r[key] for r in all_rows]
        return max(vals) - min(vals)
    return {"span_east_m": span("east_m"), "span_north_m": span("north_m"),
            "span_up_m": span("up_m"), "span_tilt_deg": span("tilt_deg"),
            "n": len(all_rows)}


def build_verdict(cfg, phases, driver, driver_result_line, motion=None):
    """Apply the PRE-REGISTERED criteria. VOID beats PASS beats FAIL."""
    drag = DragParams.px4_x500_mono_cam_sitl()
    a_drag = drag.drag_accel_m_s2(cfg["wind_mps"])
    predicted_tilt = math.degrees(math.atan(a_drag / G))
    predicted_force = a_drag * drag.mass_kg
    lo = predicted_tilt * (1.0 - TILT_TOLERANCE_FRAC)
    hi = predicted_tilt * (1.0 + TILT_TOLERANCE_FRAC)

    voids, checks = [], []

    # --- VOID conditions (no vacuous verdicts) ---------------------------
    for name in ("A", "B", "C"):
        s = phases.get(name)
        if s is None:
            voids.append(f"phase {name} produced no samples at all")
        elif s["n_samples_phase"] < MIN_SAMPLES_PER_PHASE:
            voids.append(f"phase {name} had {s['n_samples_phase']} samples "
                         f"(< {MIN_SAMPLES_PER_PHASE} required)")
    if driver is None:
        voids.append("the wind driver wrote no applied-wrench CSV")
    else:
        if driver["rows"] == 0:
            voids.append("the wind driver CSV has zero rows")
        if driver["published"] == 0:
            voids.append("the wind driver published ZERO wrenches "
                         "(published=0 -- nothing was ever put on the wire)")
        if driver["publish_failed"] > 0:
            voids.append(f"the wind driver recorded {driver['publish_failed']} "
                         f"publish failures")
        if driver["rows"] and not math.isnan(driver["mean_force_n"]):
            rel = abs(driver["mean_force_n"] - predicted_force) / predicted_force
            if rel > FORCE_MATCH_FRAC:
                voids.append(
                    f"commanded force {driver['mean_force_n']:.3f} N is "
                    f"{rel * 100:.1f}% from the assumed {predicted_force:.3f} N "
                    f"-- the driver was not commanding the field this "
                    f"prediction assumes")
    if motion is not None:
        if motion["n"] == 0:
            voids.append("the ground-truth pose reader produced no samples")
        elif (motion["span_up_m"] == 0.0 and motion["span_east_m"] == 0.0
                and motion["span_north_m"] == 0.0):
            voids.append(
                f"the ground-truth pose NEVER CHANGED across {motion['n']} "
                f"samples (span 0.0 m in east/north/up) -- the reader is "
                f"latched onto something that does not move, so every tilt it "
                f"reported is an artefact. A frozen instrument is VOID, never "
                f"a NULL finding about Gazebo.")
    if driver_result_line is None:
        voids.append("the driver never printed its WIND_DRIVER_RESULT summary "
                     "(it did not exit cleanly)")
    if "pose_updates=0" in (driver_result_line or ""):
        voids.append("the driver reported pose_updates=0 -- it never saw the "
                     "airframe, so it applied nothing")
    for name in ("A", "B", "C"):
        s = phases.get(name)
        if s and s["drift_within_window_m"] > MAX_DRIFT_M:
            voids.append(
                f"phase {name} drifted {s['drift_within_window_m']:.2f} m "
                f"(> {MAX_DRIFT_M} m): position hold was lost, so the "
                f"force-balance derivation does not apply")

    if voids:
        return {"verdict": "VOID", "reasons": voids, "checks": [],
                "predicted_tilt_deg": predicted_tilt,
                "predicted_force_n": predicted_force,
                "band_deg": [lo, hi]}

    a, b, c = phases["A"], phases["B"], phases["C"]

    checks.append({
        "id": "A-level",
        "what": f"phase A (no driver) mean tilt < {LEVEL_MAX_DEG} deg",
        "measured": round(a["mean_tilt_deg"], 4),
        "pass": a["mean_tilt_deg"] < LEVEL_MAX_DEG,
    })
    checks.append({
        "id": "B-magnitude",
        "what": f"phase B settled tilt within +/-{TILT_TOLERANCE_FRAC:.0%} of "
                f"{predicted_tilt:.3f} deg  ({lo:.3f}..{hi:.3f})",
        "measured": round(b["mean_tilt_deg"], 4),
        "pass": lo <= b["mean_tilt_deg"] <= hi,
    })
    bearing_err = angle_diff_deg(b["mean_lean_bearing_deg"], cfg["dir_from_deg"])
    checks.append({
        "id": "B-direction",
        "what": f"phase B lean points INTO the wind: bearing within "
                f"+/-{BEARING_TOLERANCE_DEG} deg of {cfg['dir_from_deg']:.0f}",
        "measured": (round(b["mean_lean_bearing_deg"], 3),
                     f"err {bearing_err:+.2f} deg"),
        "pass": (not math.isnan(bearing_err)
                 and abs(bearing_err) <= BEARING_TOLERANCE_DEG),
    })
    checks.append({
        "id": "C-removable",
        "what": f"phase C (driver exited) mean tilt back < {LEVEL_MAX_DEG} deg",
        "measured": round(c["mean_tilt_deg"], 4),
        "pass": c["mean_tilt_deg"] < LEVEL_MAX_DEG,
    })

    all_pass = all(ch["pass"] for ch in checks)
    if all_pass:
        verdict = "PASS"
    elif (checks[1]["pass"] is False and checks[2]["pass"]
          and b["mean_tilt_deg"] > LEVEL_MAX_DEG):
        # Force arrived, direction right, magnitude off band -> the specific
        # PARTIAL outcome the pre-registration names. Still not a pass.
        verdict = "PARTIAL"
    elif b["mean_tilt_deg"] <= LEVEL_MAX_DEG:
        verdict = "NULL"     # no tilt appeared: the wrench did not drive it
    else:
        verdict = "FAIL"
    return {
        "verdict": verdict,
        "reasons": [],
        "checks": checks,
        "predicted_tilt_deg": predicted_tilt,
        "predicted_force_n": predicted_force,
        "band_deg": [lo, hi],
    }


# --------------------------------------------------------------------- main
async def run(args):
    from mavsdk import System  # noqa: PLC0415

    os.makedirs(args.out_dir, exist_ok=True)
    attitude_csv = os.path.join(args.out_dir, "attitude.csv")
    driver_csv = os.path.join(args.out_dir, "wind_applied.csv")
    driver_log = os.path.join(args.out_dir, "wind_driver.log")
    verdict_path = os.path.join(args.out_dir, "verdict.json")

    cfg = {
        "wind_mps": args.wind_mps,
        "dir_from_deg": args.dir_from_deg,
        "height_m": args.height_m,
        "seed": args.seed,
        "hover_alt_m": args.hover_alt_m,
        "phase_a_sim_s": args.phase_a_sim_s,
        "phase_b_sim_s": args.phase_b_sim_s,
        "phase_c_sim_s": args.phase_c_sim_s,
        "tail_sim_s": args.tail_sim_s,
        "gust_factor": args.gust_factor,
        "prereg": "docs/wind_gate0_prereg.md",
        "parent_decision": "ADR-0096",
        "utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }

    print("[gate0] Subscribing to Gazebo ground truth...", flush=True)
    gt = GroundTruthPose()
    topics = gt.start()
    print(f"[gate0] Subscribed: {topics}", flush=True)

    drone = System()
    st = Telem()
    trackers = []
    driver_proc = None
    driver_result_line = None
    phases = {}
    all_rows = []

    fh = open(attitude_csv, "w", newline="")
    writer = csv.DictWriter(fh, fieldnames=[
        "phase", "t_sim", "t_phase", "pose_t_sim", "east_m", "north_m", "up_m",
        "tilt_deg", "lean_bearing_deg"])
    writer.writeheader()

    try:
        print(f"[gate0] Connecting to {SYSTEM_ADDRESS}...", flush=True)
        await drone.connect(system_address=SYSTEM_ADDRESS)
        await _wait_connected(drone, 60)
        trackers = [asyncio.create_task(_track_position(drone, st)),
                    asyncio.create_task(_track_armed(drone, st)),
                    asyncio.create_task(_track_landed(drone, st))]
        print("[gate0] Connected. Waiting for health...", flush=True)
        await _wait_health(drone, 120)

        print(f"[gate0] Taking off to {args.hover_alt_m} m...", flush=True)
        await drone.action.set_takeoff_altitude(args.hover_alt_m)
        await drone.action.arm()
        await drone.action.takeoff()

        target = args.hover_alt_m * 0.9
        deadline = time.monotonic() + 120
        while time.monotonic() < deadline:
            if (st.relative_altitude_m is not None
                    and st.relative_altitude_m >= target):
                break
            await asyncio.sleep(0.2)
        else:
            raise RuntimeError(
                f"never reached {target:.1f} m (last "
                f"{st.relative_altitude_m}) -- cannot probe a hover that "
                f"did not happen")
        print(f"[gate0] At {st.relative_altitude_m:.2f} m. Settling "
              f"{args.settle_sim_s:.0f} s (sim)...", flush=True)

        # Discard the climb transient: sample but do not score. The rows still
        # count toward the frozen-pose check -- an instrument that cannot see a
        # 6 m climb cannot see a 5 degree tilt either.
        all_rows += await record_phase(gt, writer, "settle", args.settle_sim_s,
                                       args.sample_hz,
                                       wall_budget_s=args.settle_sim_s * 6 + 60)

        # ---- PHASE A: baseline, no driver -------------------------------
        print(f"[gate0] PHASE A ({args.phase_a_sim_s:.0f} sim s): driver OFF, "
              f"expect level.", flush=True)
        rows_a = await record_phase(gt, writer, "A", args.phase_a_sim_s,
                                    args.sample_hz,
                                    wall_budget_s=args.phase_a_sim_s * 6 + 60)
        all_rows += rows_a
        phases["A"] = summarise(rows_a, args.tail_sim_s)
        print(f"[gate0]   A: mean tilt {phases['A']['mean_tilt_deg']:.3f} deg "
              f"over {phases['A']['n_samples_window']} samples", flush=True)

        # ---- PHASE B: wind on -------------------------------------------
        cmd = [sys.executable, os.path.join(SCRIPTS_DIR, "wind_driver.py"),
               "--mean-mps", str(args.wind_mps),
               "--dir-from-deg", str(args.dir_from_deg),
               "--height-m", str(args.height_m),
               "--seed", str(args.seed),
               "--gust-factor", str(args.gust_factor),
               "--duration", str(args.phase_b_sim_s),
               "--out", driver_csv,
               "--label", "gate0",
               "--provenance",
               "Gate-0 physics probe: synthetic steady wind, NOT a field "
               "reading (docs/wind_gate0_prereg.md)"]
        print(f"[gate0] PHASE B ({args.phase_b_sim_s:.0f} sim s): starting "
              f"driver -> {' '.join(cmd[1:])}", flush=True)
        dlog = open(driver_log, "w")
        driver_proc = subprocess.Popen(cmd, cwd=REPO_ROOT, stdout=dlog,
                                       stderr=subprocess.STDOUT, start_new_session=True)

        rows_b = await record_phase(gt, writer, "B", args.phase_b_sim_s,
                                    args.sample_hz,
                                    wall_budget_s=args.phase_b_sim_s * 6 + 90)
        all_rows += rows_b
        phases["B"] = summarise(rows_b, args.tail_sim_s)
        print(f"[gate0]   B: mean tilt {phases['B']['mean_tilt_deg']:.3f} deg, "
              f"lean {phases['B']['mean_lean_bearing_deg']:.1f} deg, "
              f"drift {phases['B']['drift_within_window_m']:.2f} m", flush=True)

        # ---- driver must exit and CLEAR the wrench -----------------------
        print("[gate0] Waiting for the driver to exit and clear the wrench...",
              flush=True)
        try:
            driver_proc.wait(timeout=90)
        except subprocess.TimeoutExpired:
            print("[gate0] driver overran --duration; sending SIGINT so it "
                  "runs its clear-on-exit path.", flush=True)
            driver_proc.send_signal(signal.SIGINT)
            try:
                driver_proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                driver_proc.kill()
                driver_proc.wait(timeout=10)
        dlog.close()
        print(f"[gate0] driver exited rc={driver_proc.returncode}", flush=True)
        with open(driver_log) as lf:
            for line in lf:
                if "WIND_DRIVER_RESULT" in line:
                    driver_result_line = line.strip()
        if driver_result_line:
            print(f"[gate0] {driver_result_line}", flush=True)

        # ---- PHASE C: wrench cleared ------------------------------------
        print(f"[gate0] PHASE C ({args.phase_c_sim_s:.0f} sim s): driver gone, "
              f"expect level again.", flush=True)
        rows_c = await record_phase(gt, writer, "C", args.phase_c_sim_s,
                                    args.sample_hz,
                                    wall_budget_s=args.phase_c_sim_s * 6 + 60)
        all_rows += rows_c
        phases["C"] = summarise(rows_c, args.tail_sim_s)
        print(f"[gate0]   C: mean tilt {phases['C']['mean_tilt_deg']:.3f} deg",
              flush=True)

        print("[gate0] Landing...", flush=True)
        try:
            await drone.action.land()
            deadline = time.monotonic() + 90
            while time.monotonic() < deadline:
                if st.armed is False:
                    break
                await asyncio.sleep(0.5)
        except Exception as exc:                              # noqa: BLE001
            print(f"[gate0] land failed ({exc!r}) -- the measurement is "
                  f"already taken, continuing to the verdict.", flush=True)

    finally:
        if driver_proc is not None and driver_proc.poll() is None:
            driver_proc.send_signal(signal.SIGINT)
            try:
                driver_proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                driver_proc.kill()
        for t in trackers:
            t.cancel()
        for t in trackers:
            try:
                await t
            except (asyncio.CancelledError, Exception):
                pass
        fh.close()

    _sim_t, _s, source, n_updates = gt.read()
    driver = read_driver_csv(driver_csv)
    motion = pose_motion(all_rows)
    result = build_verdict(cfg, phases, driver, driver_result_line, motion)
    out = {
        "config": cfg,
        "pose_source": source,
        "pose_updates": n_updates,
        "pose_entity": INTERCEPTOR_MODEL,
        "pose_motion": motion,
        "phases": phases,
        "driver": driver,
        "driver_result_line": driver_result_line,
        **result,
    }
    with open(verdict_path, "w") as vf:
        json.dump(out, vf, indent=2, default=str)

    print("\n" + "=" * 72, flush=True)
    print(f"WIND GATE-0 (W3) PHYSICS PROBE -- {result['verdict']}", flush=True)
    print("=" * 72, flush=True)
    print(f"  predicted tilt : {result['predicted_tilt_deg']:.3f} deg "
          f"(band {result['band_deg'][0]:.3f}..{result['band_deg'][1]:.3f})",
          flush=True)
    print(f"  predicted force: {result['predicted_force_n']:.3f} N", flush=True)
    print(f"  pose source    : {source}  ({n_updates} updates)", flush=True)
    for name in ("A", "B", "C"):
        s = phases.get(name)
        if s:
            print(f"  phase {name}: tilt mean {s['mean_tilt_deg']:.3f} deg "
                  f"(min {s['min_tilt_deg']:.3f} max {s['max_tilt_deg']:.3f}), "
                  f"lean {s['mean_lean_bearing_deg']:.1f} deg, "
                  f"drift {s['drift_within_window_m']:.2f} m, "
                  f"n={s['n_samples_window']}/{s['n_samples_phase']}", flush=True)
        else:
            print(f"  phase {name}: NO SAMPLES", flush=True)
    if driver:
        print(f"  driver: rows={driver['rows']} published={driver['published']} "
              f"failed={driver['publish_failed']} "
              f"mean_force={driver.get('mean_force_n', float('nan')):.3f} N",
              flush=True)
    for ch in result["checks"]:
        print(f"  [{'PASS' if ch['pass'] else 'FAIL'}] {ch['id']}: {ch['what']} "
              f"-> {ch['measured']}", flush=True)
    for r in result["reasons"]:
        print(f"  VOID: {r}", flush=True)
    print(f"\n  verdict written to {verdict_path}", flush=True)
    if result["verdict"] == "PASS":
        print("  ADR-0096's stop sign lifts: Gazebo APPLIES the wrench, at the "
              "commanded magnitude and direction, and releases it.\n"
              "  The drag PARAMETERS remain PX4 defaults, not airframe "
              "measurements -- that assumption is untouched by this probe.",
              flush=True)
    else:
        print("  Per docs/wind_gate0_prereg.md sec 5, this outcome must be "
              "recorded on the always-visible surfaces the same turn.",
              flush=True)
    return 0 if result["verdict"] == "PASS" else 1


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--wind-mps", type=float, default=5.0)
    p.add_argument("--dir-from-deg", type=float, default=270.0,
                   help="METEOROLOGICAL bearing the wind blows FROM")
    p.add_argument("--height-m", type=float, default=6.0)
    p.add_argument("--hover-alt-m", type=float, default=6.0)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--gust-factor", type=float, default=1.0,
                   help="1.0 = steady, no turbulence (a settled tilt is only "
                        "readable in a steady field)")
    p.add_argument("--settle-sim-s", type=float, default=12.0)
    p.add_argument("--phase-a-sim-s", type=float, default=10.0)
    p.add_argument("--phase-b-sim-s", type=float, default=30.0)
    p.add_argument("--phase-c-sim-s", type=float, default=15.0)
    p.add_argument("--tail-sim-s", type=float, default=10.0,
                   help="length of the settled window scored at the END of "
                        "each phase")
    p.add_argument("--sample-hz", type=float, default=10.0)
    p.add_argument("--out-dir", default=None)
    a = p.parse_args(argv)
    if a.out_dir is None:
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        a.out_dir = os.path.join(REPO_ROOT, "logs", f"wind_gate0_{ts}")
    return a


if __name__ == "__main__":
    sys.exit(asyncio.run(run(parse_args())))
