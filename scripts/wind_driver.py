#!/usr/bin/env python3
"""WIND DRIVER -- the sim-side process that turns scripts/wind_model.py into an
actual force on the airframe (wind arm W1).

STATUS 2026-08-10: NEW, DEFAULT-OFF, NOT YET FLOWN. Nothing spawns this unless
`--wind-mps` / `--wind-tier` / `--wind-still-air` is passed to
scripts/m4_intercept.py. With none of those, m4 never builds a wind command and
this file is dead code for every existing arm -- so every measured result in the
repo is byte-identical to before it existed.

WHAT IT IS
    A standalone gz-transport process in the exact mould of
    scripts/m4_target_mover.py: a WORLD-SIDE DISTURBANCE. It reads Gazebo state
    in order to APPLY PHYSICS, and it publishes nothing the flight code can
    read. It:
      1. subscribes `/clock` (sim time -- ADR-0009, never wall time) and
         `/world/<w>/dynamic_pose/info`;
      2. finite-differences the interceptor's pose ON SIM TIME to get its world
         velocity;
      3. asks scripts/wind_model.py for the wind vector at that sim instant and
         for the drag force from the RELATIVE air velocity;
      4. publishes that force as a gz.msgs.EntityWrench on
         `/world/<w>/wrench/persistent`;
      5. writes every APPLIED wrench to logs/wind_<run>.csv.

HONESTY BOUNDARY (CLAUDE.md)
    This process is on the gt_* side of the line, exactly like the target mover
    and the ground-truth scorer. It reads world pose to compute physics; the
    guidance code in flight/ and the tick loop in m4_intercept.py never import
    it, never read its CSV, and never receive its wind vector. The ONE
    sanctioned path from wind toward the vehicle is the operator's noisy
    pre-flight anemometer reading (scripts/wind_anemometer_sim.py ->
    scripts/wind_trim.py), which is a separate file boundary with its own
    machine-checked audit (tests/test_wind_trim_honesty.py).

WHY PERSISTENT WRENCH, NOT ONE-SHOT
    gz-sim 8's ApplyLinkWrench system (already in PX4's server.config line 10 --
    no plugin change, no world-SDF edit, no server.config swap) listens on three
    topics: `/world/<w>/wrench` applies a wrench for ONE physics iteration,
    `/world/<w>/wrench/persistent` applies it every iteration until replaced or
    cleared, and `/world/<w>/wrench/clear` removes it. PX4 SITL steps physics at
    250 Hz while this driver ticks at 20 Hz, so a one-shot wrench would act on
    ~1 step in 12 -- about 8% of the intended impulse, silently. We therefore
    publish PERSISTENT (each publish replaces the previous value for that
    entity) and CLEAR on exit.

    THIS IS THE ONE UNVERIFIED LINK IN THE CHAIN and it is the reason the
    approved design has a Gate-0 physics probe: hover in a steady 5 m/s wind and
    check the ULog shows the derived tilt-into-wind. Until that probe runs, a
    green test suite here proves the FORCE VECTOR is right, NOT that Gazebo
    applies it. `--offline-selftest` says so in its own output.

WHAT GETS LOGGED IS WHAT WAS APPLIED, NOT WHAT WAS INTENDED
    Same rule as the camera frame-rate pinning in flight/deploy/seeker_loop.py
    (ADR-0090): that code prints the frame duration the sensor ACTUALLY took,
    because an earlier version printed the requested value and would have
    reported "60 fps" on a camera running at 30. Here:
      - every CSV row carries `published` (1/0) and `publish_error`, so a run
        where the publisher silently failed is distinguishable from a calm day;
      - a run that never sees the target entity's pose EXITS NON-ZERO instead of
        applying zero force forever (no-vacuous-verdict: "the driver ran and
        nothing happened" is a FAIL, never a finding);
      - the final `WIND_DRIVER_RESULT` line reports ticks, publishes, and the
        mean/max force actually put on the wire. m4_intercept.py echoes that
        line into the flight log at teardown.

USAGE
    # what m4_intercept.py spawns (it fills these in from its own flags):
    scripts/wind_driver.py --tier EXPECTED --dir-from-deg 270 --seed 7 \
        --height-m 6.0 --duration 25 --out logs/wind_run.csv

    # the Gate-1 DRAG-ONLY CONTROL (driver running, wind identically zero):
    scripts/wind_driver.py --still-air --seed 7 --height-m 6.0 --duration 25

    # offline: no Gazebo, no sim, proves the pipeline computes and "publishes"
    scripts/wind_driver.py --offline-selftest

FRAMES
    wind_model.py speaks NORTH/EAST (met/aero convention). Gazebo's world is
    ENU: east = world_x, north = world_y (ADR-0013, the same convention
    m4_intercept.py's dash azimuth uses). The conversion happens in exactly one
    place, `force_to_world_xyz`, and is pinned by a test.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from wind_model import (  # noqa: E402
    DragParams,
    MissingMeasurementError,
    WindConditions,
    WindField,
    WindForce,
    WindSample,
    tier as wind_tier,
)

# --- world / entity naming ------------------------------------------------
# Same INTERCEPTOR_WORLD_NAME inherited-env pattern as m4_target_mover.py and
# m2_detect.py: m4_intercept.py spawns this as a subprocess, so setting the var
# once before launching m4 retargets this driver too.
WORLD_NAME = os.environ.get("INTERCEPTOR_WORLD_NAME", "apriltag")
# PX4 spawns gz_x500_mono_cam as model "x500_mono_cam_0" (see
# scripts/adaptive_tilt_arm.sh's header, which documents the same name).
INTERCEPTOR_MODEL = os.environ.get("INTERCEPTOR_WIND_MODEL", "x500_mono_cam_0")
INTERCEPTOR_LINK = os.environ.get("INTERCEPTOR_WIND_LINK", "base_link")

WRENCH_PERSISTENT_TOPIC = f"/world/{WORLD_NAME}/wrench/persistent"
WRENCH_CLEAR_TOPIC = f"/world/{WORLD_NAME}/wrench/clear"
POSE_TOPIC = f"/world/{WORLD_NAME}/dynamic_pose/info"
CLOCK_TOPIC = "/clock"

DEFAULT_RATE_HZ = 20.0
# Wall seconds to wait for the first /clock sample and the first pose of the
# interceptor. Both FAIL LOUD on timeout -- see the module docstring.
CLOCK_TIMEOUT_S = 15.0
ENTITY_TIMEOUT_S = 30.0
CSV_FLUSH_EVERY = 20

CSV_HEADER = [
    "t_sim", "grid_index",
    "wind_n", "wind_e", "wind_d",
    "steady_n", "steady_e", "gust_u", "gust_v",
    "v_veh_n", "v_veh_e",
    "v_rel_n", "v_rel_e", "v_rel_mps",
    "a_drag_m_s2",
    "applied_f_n", "applied_f_e", "applied_f_d",
    "applied_f_world_x", "applied_f_world_y", "applied_f_world_z",
    "published", "publish_error",
]


class WindDriverError(RuntimeError):
    """Fail-closed driver fault. Always exits non-zero; never degrades to a
    quiet zero-wind run (that would manufacture a fake 'wind had no effect')."""


# ------------------------------------------------------------------ frames
def force_to_world_xyz(force: WindForce) -> Tuple[float, float, float]:
    """(north, east, down) newtons -> Gazebo ENU world (x, y, z) newtons.

    ADR-0013: east = world_x, north = world_y. Down is +z downward, so a
    downward force is -z in ENU. wind_model declares f_d == 0.0 (no vertical
    wind component is modelled), but the term is carried through rather than
    dropped so that the day a vertical component is added, the sign is already
    right and already tested.
    """
    return (force.f_e, force.f_n, -force.f_d)


# ------------------------------------------------- sim-time velocity source
class PoseVelocityEstimator:
    """Finite-difference the interceptor's world velocity ON SIM TIME.

    The vehicle NEVER sees this number -- it is the world's own knowledge of how
    fast the airframe is moving through the air, used to form the relative air
    velocity. It is the same category of read as the ground-truth scorer's.

    `min_dt_s` guards a divide-by-noise when two poses arrive inside one physics
    step; `lowpass_tau_s` optionally smooths the differenced velocity (the
    approved design flagged 20 Hz differencing noise as a risk to check in
    Gate 0 -- the knob exists so the check has something to turn, and defaults
    to 0.0 = OFF so the raw behaviour is what flies unless someone changes it
    deliberately and says so in the log).
    """

    def __init__(self, min_dt_s: float = 1e-3, lowpass_tau_s: float = 0.0):
        if min_dt_s <= 0.0:
            raise WindDriverError("min_dt_s must be > 0")
        if lowpass_tau_s < 0.0:
            raise WindDriverError("lowpass_tau_s must be >= 0")
        self.min_dt_s = float(min_dt_s)
        self.lowpass_tau_s = float(lowpass_tau_s)
        self._last: Optional[Tuple[float, float, float]] = None   # (t, n, e)
        self.v_n = 0.0
        self.v_e = 0.0
        self.n_updates = 0
        self.has_velocity = False

    def update(self, t_sim: float, north_m: float, east_m: float) -> bool:
        """Feed one pose. Returns True if the velocity estimate advanced."""
        t = float(t_sim)
        if self._last is None:
            self._last = (t, float(north_m), float(east_m))
            return False
        t0, n0, e0 = self._last
        dt = t - t0
        if dt < 0.0:
            # Sim time going backwards means a wrong clock basis, not a gust.
            raise WindDriverError(
                f"sim time went BACKWARDS in the pose stream ({t0:.3f} -> "
                f"{t:.3f} s). Refusing to differentiate a broken clock "
                "(ADR-0009: sim time, never wall time).")
        if dt < self.min_dt_s:
            return False
        raw_n = (float(north_m) - n0) / dt
        raw_e = (float(east_m) - e0) / dt
        if self.lowpass_tau_s > 0.0 and self.has_velocity:
            a = math.exp(-dt / self.lowpass_tau_s)
            self.v_n = a * self.v_n + (1.0 - a) * raw_n
            self.v_e = a * self.v_e + (1.0 - a) * raw_e
        else:
            self.v_n, self.v_e = raw_n, raw_e
        self._last = (t, float(north_m), float(east_m))
        self.n_updates += 1
        self.has_velocity = True
        return True


# ------------------------------------------------------------- driver core
@dataclass
class AppliedRow:
    """One tick, recording WHAT WAS APPLIED (see the module docstring)."""
    sample: WindSample
    force: WindForce
    v_veh_n: float
    v_veh_e: float
    world_xyz: Tuple[float, float, float]
    published: bool
    publish_error: str = ""

    def as_csv_row(self) -> list:
        s, f = self.sample, self.force
        wx, wy, wz = self.world_xyz
        return [
            f"{s.t_sim_s:.4f}", s.grid_index,
            f"{s.wind_n:.5f}", f"{s.wind_e:.5f}", f"{s.wind_d:.5f}",
            f"{s.steady_n:.5f}", f"{s.steady_e:.5f}",
            f"{s.gust_u:.5f}", f"{s.gust_v:.5f}",
            f"{self.v_veh_n:.5f}", f"{self.v_veh_e:.5f}",
            f"{f.v_rel_n:.5f}", f"{f.v_rel_e:.5f}", f"{f.v_rel_mps:.5f}",
            f"{f.a_drag_m_s2:.5f}",
            f"{f.f_n:.5f}", f"{f.f_e:.5f}", f"{f.f_d:.5f}",
            f"{wx:.5f}", f"{wy:.5f}", f"{wz:.5f}",
            1 if self.published else 0, self.publish_error,
        ]


class WindDriverCore:
    """The whole computation, with gz-transport injected as a callable.

    `publish` takes (fx, fy, fz) in Gazebo ENU world newtons and returns True on
    success. Injecting it is what makes `--offline-selftest` a real exercise of
    the real path rather than a parallel reimplementation.
    """

    def __init__(self, field: WindField, drag: DragParams,
                 publish: Callable[[float, float, float], bool],
                 estimator: Optional[PoseVelocityEstimator] = None):
        self.field = field
        self.drag = drag
        self.publish = publish
        self.est = estimator if estimator is not None else PoseVelocityEstimator()
        self.rows: List[AppliedRow] = []
        self.n_ticks = 0
        self.n_published = 0
        self.n_publish_failed = 0
        self.sum_force_n = 0.0
        self.max_force_n = 0.0

    def on_pose(self, t_sim: float, north_m: float, east_m: float) -> bool:
        return self.est.update(t_sim, north_m, east_m)

    def tick(self, t_sim: float) -> Optional[AppliedRow]:
        """Sample the wind at `t_sim`, form the force, publish it, record it.

        Returns None (and publishes NOTHING) until the pose stream has produced
        a velocity: applying wind force against an assumed-zero velocity would
        be a made-up number, and a default substituted for a measured quantity
        is the fail-closed rule's exact prohibition.
        """
        if not self.est.has_velocity:
            return None
        sample = self.field.sample(t_sim)
        force = self.field.force(sample, self.est.v_n, self.est.v_e, self.drag)
        wx, wy, wz = force_to_world_xyz(force)
        err = ""
        try:
            ok = bool(self.publish(wx, wy, wz))
        except Exception as exc:                      # noqa: BLE001 -- recorded
            ok, err = False, f"{type(exc).__name__}: {exc}"
        row = AppliedRow(sample=sample, force=force,
                         v_veh_n=self.est.v_n, v_veh_e=self.est.v_e,
                         world_xyz=(wx, wy, wz), published=ok, publish_error=err)
        self.rows.append(row)
        self.n_ticks += 1
        if ok:
            self.n_published += 1
            mag = force.magnitude_n
            self.sum_force_n += mag
            self.max_force_n = max(self.max_force_n, mag)
        else:
            self.n_publish_failed += 1
        return row

    # -- the APPLIED summary, echoed by m4_intercept.py at teardown ---------
    def result_line(self) -> str:
        mean = (self.sum_force_n / self.n_published) if self.n_published else float("nan")
        return (
            f"WIND_DRIVER_RESULT label={self.field.conditions.label} "
            f"mean_mps={self.field.conditions.mean_mps:.2f} "
            f"dir_from_deg={self.field.conditions.dir_from_deg:.1f} "
            f"sigma_mps={self.field.conditions.sigma_mps:.3f} "
            f"tau_s={self.field.conditions.tau_s:.3f} "
            f"seed={self.field.seed} "
            f"ticks={self.n_ticks} published={self.n_published} "
            f"publish_failed={self.n_publish_failed} "
            f"pose_updates={self.est.n_updates} "
            f"mean_force_n={mean:.4f} max_force_n={self.max_force_n:.4f}")


# ------------------------------------------------------------- CSV writing
def write_rows_csv(path: str, rows: Sequence[AppliedRow], header_lines: Sequence[str]) -> None:
    d = os.path.dirname(os.path.abspath(path))
    if d:
        os.makedirs(d, exist_ok=True)
    with open(path, "w", newline="") as fh:
        for line in header_lines:
            fh.write(f"# {line}\n")
        w = csv.writer(fh)
        w.writerow(CSV_HEADER)
        for r in rows:
            w.writerow(r.as_csv_row())


# ------------------------------------------------------------ config build
def build_conditions(args) -> WindConditions:
    """Turn CLI values into a WindConditions, fail-closed.

    Exactly one of --still-air / --tier / --mean-mps must be given: an implicit
    default wind speed is a measured quantity substituted by a constant, which
    is the failure mode `fail closed on measured quantities` exists to stop.
    """
    chosen = [bool(args.still_air), args.tier is not None, args.mean_mps is not None]
    if sum(1 for c in chosen if c) != 1:
        raise WindDriverError(
            "give exactly ONE of --still-air / --tier / --mean-mps. There is no "
            "default wind speed: it is a site measurement (handheld anemometer, "
            "1-minute mean at launch height), not a constant.")
    if args.height_m is None:
        raise WindDriverError(
            "--height-m is REQUIRED and has no default. The Dryden gust "
            "parameters are strong functions of height (at 4.5 m/s the gust "
            "correlation time is 9.59 s at h=6 m and 0.88 s at h=0.5 m -- a "
            "qualitative difference over a ~2 s engagement), and the sim's own "
            "ALT_REF_M is 0.5 m, BELOW the MIL-F-8785C low-altitude form's "
            "10 ft (3.048 m) floor. Someone has to choose the flight height of "
            "record and have it appear in the run log.")
    if args.still_air:
        return WindConditions.still_air(
            height_m=args.height_m,
            provenance=("Gate-1 DRAG-ONLY CONTROL: driver running, wind "
                        "identically zero. Turning wind on also turns drag on, "
                        "so every wind arm compares against THIS, never against "
                        "a no-driver baseline."))
    if args.tier is not None:
        if args.dir_from_deg is None:
            raise WindDriverError(
                "--dir-from-deg is required with --tier (deg from true N the "
                "wind blows FROM -- wind sock + compass, +/-15 deg). The tier "
                "fixes the SPEED; direction is a property of the session.")
        return wind_tier(args.tier, height_m=args.height_m,
                         dir_from_deg=args.dir_from_deg,
                         allow_below_10ft=args.allow_below_10ft)
    if args.dir_from_deg is None:
        raise WindDriverError(
            "--dir-from-deg is required with --mean-mps (deg from true N the "
            "wind blows FROM -- wind sock + compass, +/-15 deg).")
    return WindConditions.from_anemometer(
        mean_mps=args.mean_mps, dir_from_deg=args.dir_from_deg,
        height_m=args.height_m,
        provenance=(args.provenance or
                    "explicit --mean-mps/--dir-from-deg (state the source in "
                    "--provenance when this is a field reading)"),
        label=args.label or "custom",
        gust_factor=args.gust_factor,
        allow_below_10ft=args.allow_below_10ft)


def header_lines(cond: WindConditions, drag: DragParams, args) -> List[str]:
    return [
        "scripts/wind_driver.py -- APPLIED wind wrench log (world-side "
        "disturbance; gt side of the honesty boundary, scoring/audit only)",
        f"world={WORLD_NAME} entity={INTERCEPTOR_MODEL}::{INTERCEPTOR_LINK} "
        f"topic={WRENCH_PERSISTENT_TOPIC}",
        f"label={cond.label} mean_mps={cond.mean_mps} "
        f"dir_from_deg={cond.dir_from_deg} height_m={cond.height_m} "
        f"sigma_mps={cond.sigma_mps} tau_s={cond.tau_s} seed={args.seed} "
        f"rate_hz={args.rate_hz}",
        f"conditions_provenance={cond.provenance}",
        f"drag: mass_kg={drag.mass_kg} mcoef_per_s={drag.mcoef_per_s} "
        f"bcoef={drag.bcoef_kg_m2} rho={drag.rho_kg_m3}",
        f"drag_provenance={drag.provenance}",
        "columns applied_f_* are what was PUT ON THE WIRE; `published` is "
        "whether gz-transport accepted it (ADR-0090 pattern: log what was "
        "applied, not what was intended)",
    ]


# --------------------------------------------------------- offline selftest
def _offline_selftest(argv_height: float = 6.0) -> int:
    """Exercise the REAL core against a synthetic pose stream, no Gazebo.

    This proves the computation and the publish path; it does NOT prove Gazebo
    applies the wrench (that is the Gate-0 probe -- see the module docstring).
    """
    checks: List[Tuple[str, bool, str]] = []

    def check(name, cond, detail=""):
        checks.append((name, bool(cond), detail))

    published: List[Tuple[float, float, float]] = []

    def fake_publish(fx, fy, fz):
        published.append((fx, fy, fz))
        return True

    step = 0.05
    drag = DragParams.px4_x500_mono_cam_sitl()

    # --- arm 1: still air, vehicle at rest -> zero force -------------------
    cond0 = WindConditions.still_air(
        height_m=argv_height, provenance="offline self-test still-air control")
    core0 = WindDriverCore(WindField(cond0, seed=1, sim_step_s=step), drag, fake_publish)
    for i in range(40):
        t = i * step
        core0.on_pose(t, 0.0, 0.0)          # parked
        core0.tick(t)
    check("still air + parked vehicle -> zero force",
          core0.n_ticks > 0 and core0.max_force_n == 0.0,
          f"ticks={core0.n_ticks} max_force={core0.max_force_n:.6f} N")

    # --- arm 2: still air, vehicle DASHING -> pure drag, opposing motion ---
    published.clear()
    core1 = WindDriverCore(WindField(cond0, seed=1, sim_step_s=step), drag, fake_publish)
    v_dash = 16.0                            # m/s north, the coded-dash speed
    for i in range(40):
        t = i * step
        core1.on_pose(t, v_dash * t, 0.0)
        core1.tick(t)
    last = core1.rows[-1]
    check("still air + moving vehicle -> nonzero DRAG (this is the Gate-1 arm)",
          core1.max_force_n > 0.0, f"max_force={core1.max_force_n:.3f} N")
    check("drag opposes motion (force points south for a northward dash)",
          last.force.f_n < 0.0 and abs(last.force.f_e) < 1e-9,
          f"f_n={last.force.f_n:.3f} f_e={last.force.f_e:.3f} N")
    # derivation: a = MCOEF*v + rho*v^2/(2*BCOEF); F = m*a
    a_expect = drag.drag_accel_m_s2(v_dash)
    f_expect = drag.mass_kg * a_expect
    check("drag magnitude matches the derivation m*(MCOEF*v + rho*v^2/(2*BCOEF))",
          abs(last.force.magnitude_n - f_expect) < 1e-6,
          f"applied={last.force.magnitude_n:.4f} N derived={f_expect:.4f} N")
    check("ENU conversion: north force -> world +y, east force -> world +x",
          abs(published[-1][1] - last.force.f_n) < 1e-12
          and abs(published[-1][0] - last.force.f_e) < 1e-12,
          f"world_xyz={published[-1]}")

    # --- arm 3: EXPECTED-tier wind on a parked vehicle ---------------------
    cond2 = wind_tier("EXPECTED", height_m=argv_height, dir_from_deg=0.0)
    core2 = WindDriverCore(WindField(cond2, seed=7, sim_step_s=step), drag, fake_publish)
    for i in range(200):
        t = i * step
        core2.on_pose(t, 0.0, 0.0)
        core2.tick(t)
    r2 = core2.rows[-1]
    check("wind FROM north pushes the parked vehicle SOUTH (met convention)",
          r2.force.f_n < 0.0, f"f_n={r2.force.f_n:.3f} N")
    check("wind arm force is strictly larger than the still-air control",
          core2.max_force_n > 0.0 and core2.n_published == core2.n_ticks,
          f"max_force={core2.max_force_n:.3f} N published={core2.n_published}")
    gusty = len({round(r.sample.gust_u, 6) for r in core2.rows}) > 5
    check("gusts actually vary (a constant 'gust' would be a dead model)", gusty,
          f"{len({round(r.sample.gust_u, 6) for r in core2.rows})} distinct gust_u values")

    # --- arm 4: pairing -- same seed, same realisation ---------------------
    core3 = WindDriverCore(WindField(cond2, seed=7, sim_step_s=step), drag, fake_publish)
    for i in range(200):
        t = i * step
        core3.on_pose(t, 0.0, 0.0)
        core3.tick(t)
    same = all(abs(a.force.f_n - b.force.f_n) < 1e-12
               for a, b in zip(core2.rows, core3.rows))
    check("same seed -> identical applied force sequence (paired arms)", same)

    # --- arm 5: fail-closed -- no pose, no publish -------------------------
    core4 = WindDriverCore(WindField(cond2, seed=7, sim_step_s=step), drag, fake_publish)
    core4.tick(0.0)
    core4.on_pose(0.0, 0.0, 0.0)     # first pose only sets the reference
    core4.tick(0.05)
    check("no velocity yet -> NOTHING published (never assume v=0)",
          core4.n_ticks == 0 and core4.n_published == 0,
          f"ticks={core4.n_ticks}")

    # --- arm 6: a failing publisher is RECORDED, not swallowed -------------
    def broken_publish(fx, fy, fz):
        raise RuntimeError("gz-transport down")

    core5 = WindDriverCore(WindField(cond2, seed=7, sim_step_s=step), drag, broken_publish)
    core5.on_pose(0.0, 0.0, 0.0)
    core5.on_pose(0.05, 0.5, 0.0)
    core5.tick(0.05)
    check("publish failure is recorded in the row, not swallowed",
          core5.n_publish_failed == 1 and core5.rows[-1].published is False
          and "gz-transport down" in core5.rows[-1].publish_error,
          core5.rows[-1].publish_error)
    check("the APPLIED summary reports the failure",
          "publish_failed=1" in core5.result_line(), core5.result_line())

    # --- arm 7: backwards sim time fails loud ------------------------------
    est = PoseVelocityEstimator()
    est.update(1.0, 0.0, 0.0)
    try:
        est.update(0.5, 1.0, 0.0)
        check("backwards sim time raises", False, "no exception")
    except WindDriverError:
        check("backwards sim time raises", True)

    for name, ok, detail in checks:
        print(f"{'PASS' if ok else 'FAIL'} {name}" + (f"  [{detail}]" if detail else ""))
    n_ok = sum(1 for _, ok, _ in checks if ok)
    print(f"\n{n_ok}/{len(checks)} wind_driver offline self-test checks passed")
    print("NOTE: this proves the FORCE VECTOR and the publish path. It does NOT "
          "prove Gazebo applies the wrench -- that is the Gate-0 hover probe.")
    return 0 if n_ok == len(checks) else 1


# ------------------------------------------------------------------- gz run
def _run_gz(args, cond: WindConditions, drag: DragParams) -> int:
    """The real driver loop. Imported lazily so --offline-selftest, --dry-run
    and the test suite never need gz-transport installed."""
    from gz.transport13 import Node                      # noqa: PLC0415
    from gz.msgs10.clock_pb2 import Clock                # noqa: PLC0415
    from gz.msgs10.pose_v_pb2 import Pose_V              # noqa: PLC0415
    from gz.msgs10.entity_wrench_pb2 import EntityWrench  # noqa: PLC0415
    from gz.msgs10.entity_pb2 import Entity              # noqa: PLC0415

    node = Node()
    clock = {"t": None}
    pose = {"t": None, "n": None, "e": None, "seen": False}
    scoped = f"{INTERCEPTOR_MODEL}::{INTERCEPTOR_LINK}"

    def on_clock(msg: Clock) -> None:
        clock["t"] = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_pose(msg: Pose_V) -> None:
        # dynamic_pose/info carries the model pose under the MODEL name and the
        # link poses under the link name; accept either, preferring the link.
        best = None
        for p in msg.pose:
            if p.name in (scoped, INTERCEPTOR_LINK, INTERCEPTOR_MODEL):
                best = p
                if p.name in (scoped, INTERCEPTOR_LINK):
                    break
        if best is None:
            return
        t = None
        if msg.header.stamp.sec or msg.header.stamp.nsec:
            t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        if t is None:
            t = clock["t"]
        if t is None:
            return
        # Gazebo ENU: x = east, y = north.
        pose.update({"t": t, "n": best.position.y, "e": best.position.x,
                     "seen": True})

    if not node.subscribe(Clock, CLOCK_TOPIC, on_clock):
        raise WindDriverError(f"could not subscribe {CLOCK_TOPIC}")
    if not node.subscribe(Pose_V, POSE_TOPIC, on_pose):
        raise WindDriverError(f"could not subscribe {POSE_TOPIC}")

    pub = node.advertise(WRENCH_PERSISTENT_TOPIC, EntityWrench)
    clear_pub = node.advertise(WRENCH_CLEAR_TOPIC, Entity)

    msg = EntityWrench()
    msg.entity.name = scoped
    msg.entity.type = Entity.LINK

    def publish(fx: float, fy: float, fz: float) -> bool:
        msg.wrench.force.x = fx
        msg.wrench.force.y = fy
        msg.wrench.force.z = fz
        return bool(pub.publish(msg))

    core = WindDriverCore(
        WindField(cond, seed=args.seed, sim_step_s=1.0 / args.rate_hz),
        drag, publish,
        estimator=PoseVelocityEstimator(lowpass_tau_s=args.velocity_lowpass_tau_s))

    print(f"[wind] driver up: {WRENCH_PERSISTENT_TOPIC} -> {scoped}; "
          f"{cond.label} {cond.mean_mps:.2f} m/s FROM {cond.dir_from_deg:.1f} deg, "
          f"sigma {cond.sigma_mps:.3f} m/s, tau {cond.tau_s:.2f} s, seed "
          f"{args.seed}, {args.rate_hz:.0f} Hz on SIM time", flush=True)

    # -- wait for the sim clock (fail loud; never fall back to wall time) ---
    deadline = time.monotonic() + CLOCK_TIMEOUT_S
    while clock["t"] is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if clock["t"] is None:
        raise WindDriverError(
            f"no /clock sample in {CLOCK_TIMEOUT_S:.0f} wall-seconds. The gust "
            "process advances on SIM time only (ADR-0009); there is no "
            "wall-clock fallback, so this is a hard failure, not a warning.")

    # -- wait for the interceptor's pose -----------------------------------
    deadline = time.monotonic() + ENTITY_TIMEOUT_S
    while not pose["seen"] and time.monotonic() < deadline:
        time.sleep(0.05)
    if not pose["seen"]:
        raise WindDriverError(
            f"never saw entity {scoped!r} on {POSE_TOPIC} within "
            f"{ENTITY_TIMEOUT_S:.0f} wall-seconds. Exiting NON-ZERO rather than "
            "applying zero force for the whole flight -- 'the wind driver ran "
            "and nothing happened' is a FAIL, never a finding "
            "(no-vacuous-verdicts). Set INTERCEPTOR_WIND_MODEL if the airframe "
            "model name differs.")

    stop = {"now": False}

    def _sigterm(_sig, _frm):
        stop["now"] = True

    signal.signal(signal.SIGTERM, _sigterm)
    signal.signal(signal.SIGINT, _sigterm)

    t0_sim = clock["t"]
    period = 1.0 / args.rate_hz
    next_sim_t = t0_sim
    rc = 0
    try:
        while not stop["now"]:
            now_sim = clock["t"]
            if now_sim is None:
                time.sleep(0.01)
                continue
            if args.duration is not None and (now_sim - t0_sim) >= args.duration:
                break
            if now_sim + 1e-9 < next_sim_t:
                time.sleep(0.002)
                continue
            if pose["t"] is not None:
                core.on_pose(pose["t"], pose["n"], pose["e"])
            core.tick(now_sim)
            # Advance the tick schedule on SIM time, catching up if RTF sagged.
            next_sim_t += period
            if next_sim_t < now_sim:
                next_sim_t = now_sim + period
    except MissingMeasurementError as exc:
        print(f"[wind] FAIL: {exc}", flush=True)
        rc = 1
    finally:
        # Clear the persistent wrench so a crashed/killed driver cannot leave a
        # force applied to the next flight in the same sim.
        try:
            ent = Entity()
            ent.name = scoped
            ent.type = Entity.LINK
            clear_pub.publish(ent)
        except Exception as exc:                          # noqa: BLE001
            print(f"[wind] WARNING: could not clear persistent wrench: {exc}",
                  flush=True)
        if args.out:
            write_rows_csv(args.out, core.rows, header_lines(cond, drag, args))
            print(f"[wind] wrote {len(core.rows)} applied rows -> {args.out}",
                  flush=True)
        print(f"[wind] {core.result_line()}", flush=True)
        if core.n_published == 0:
            print("[wind] FAIL: the driver published ZERO wrenches. A wind arm "
                  "flown like this is a vacuous verdict -- discard the flight.",
                  flush=True)
            rc = 1
    return rc


# ---------------------------------------------------------------------- CLI
def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_argument_group(
        "wind (exactly one; there is NO default wind speed)")
    src.add_argument("--still-air", action="store_true",
                     help="Gate-1 DRAG-ONLY CONTROL: driver running, wind zero")
    src.add_argument("--tier", choices=["BEST", "EXPECTED", "WORST"],
                     help="sourced Beaufort/WMO tier (see wind_model.TIERS)")
    src.add_argument("--mean-mps", type=float,
                     help="measured 1-minute mean wind speed [m/s]")
    ap.add_argument("--dir-from-deg", type=float,
                    help="METEOROLOGICAL direction [deg from true N] the wind "
                         "blows FROM (wind sock + compass, +/-15 deg)")
    ap.add_argument("--height-m", type=float,
                    help="REQUIRED. Height AGL the wind is quoted at [m]")
    ap.add_argument("--allow-below-10ft", action="store_true",
                    help="acknowledge Dryden extrapolation below 3.048 m AGL")
    ap.add_argument("--gust-factor", type=float,
                    help="MEASURED anemometer peak/mean; omit to derive sigma "
                         "from the MIL-F-8785C Dryden form")
    ap.add_argument("--seed", type=int,
                    help="REQUIRED (except --dry-run): gust realisation seed; "
                         "paired across A/B arms")
    ap.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ,
                    help=f"driver tick rate on SIM time (default {DEFAULT_RATE_HZ})")
    ap.add_argument("--duration", type=float, default=None,
                    help="stop after this many SIM seconds (default: until killed)")
    ap.add_argument("--velocity-lowpass-tau-s", type=float, default=0.0,
                    help="optional low-pass on the differenced vehicle velocity "
                         "[s]; default 0.0 = OFF (raw)")
    ap.add_argument("--out", default=None, help="applied-wrench CSV path")
    ap.add_argument("--label", default=None, help="arm label for logs")
    ap.add_argument("--provenance", default=None,
                    help="where --mean-mps came from (required in spirit for a "
                         "field reading)")
    ap.add_argument("--dry-run", action="store_true",
                    help="resolve + print the config and exit 0; touches no sim")
    ap.add_argument("--offline-selftest", action="store_true",
                    help="exercise the real core against a synthetic pose "
                         "stream, no Gazebo; exits 0/1")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.offline_selftest:
        return _offline_selftest()
    try:
        cond = build_conditions(args)
    except (WindDriverError, MissingMeasurementError) as exc:
        print(f"[wind] REFUSING TO RUN: {exc}", file=sys.stderr)
        return 2
    drag = DragParams.px4_x500_mono_cam_sitl()
    if args.seed is None and not args.dry_run:
        print("[wind] REFUSING TO RUN: --seed is required and has no default "
              "(paired-seed A/B arms are this project's evidence standard).",
              file=sys.stderr)
        return 2
    if args.dry_run:
        for line in header_lines(cond, drag, args):
            print(f"[wind] {line}")
        print(f"[wind] peak(2 sigma) = {cond.peak_mps():.2f} m/s; "
              f"hover tilt at the mean = "
              f"{drag.hover_tilt_deg(cond.mean_mps):.2f} deg (DERIVED -- the "
              f"Gate-0 probe has not confirmed it)")
        print("[wind] --dry-run: no sim touched, nothing published.")
        return 0
    try:
        return _run_gz(args, cond, drag)
    except WindDriverError as exc:
        print(f"[wind] FAIL: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
