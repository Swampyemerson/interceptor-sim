#!/usr/bin/env python3
r"""T19a -- ground_station.py: replays a T16/T18 pre-rendered stereo-rig
capture through the REAL detect -> triangulate -> (optional) track pipeline,
LIVE during a flight, emitting the unchanged :47800 UDP JSON cue
(`scripts/ground_station.triangulate.build_cue_payload`'s exact schema:
`{"seq","t_sim","x","y","z"[,"vx","vy","vz"]}`) -- the "stereo" half of
`m4_intercept.py`'s future `--cue-source {mock,stereo}` (ADR-0048 decision
2; the T19b wiring itself is OUT OF SCOPE here, see the module-level "T19b
OWES" note at the bottom).

READ FIRST: docs/decisions.md ADR-0048 (THE spec for this file -- SS3 is the
dual-gate this module exists to implement correctly, SS5 is the interpreter
finding reproduced below, SS4 is the reproducibility scope that keeps
`scripts/s2_cue_mock.py` and `m4_intercept.py`'s `CueReader` byte-identical
and untouched by this file's existence) and ADR-0046 (why the rig ships as
offline-rendered, trajectory-matched replay instead of a live second camera
-- decision item 2/3 is this file's whole reason to exist).

=======================================================================
THE DUAL GATE (ADR-0048 SS3 -- the core deliverable of this file)
=======================================================================
`s2_cue_mock.py`'s pending-queue buffer-and-release (its own lines ~700-744)
schedules `deliver_t = t_sample + latency` and drains the FIFO front once
`sim_clock.t >= deliver_t`. That single gate is CORRECT there because the
mock's "measurement" (reading a pose topic + adding noise) is effectively
instantaneous -- there is nothing to wait on besides the clock.

Here there IS something else to wait on: turning a captured frame PAIR into
a triangulated position means running a real single-class detector
(`scripts/ground_station/detect.py`'s `GroundDetector`, ADR-0047/ADR-0049's
`ground_v1.onnx`) TWICE (left + right) plus a triangulation call -- genuine
wall-clock compute, sharing one CPU (this checkout's onnxruntime has no CUDA
execution provider -- see the interpreter note below) with whatever else is
running. If ground_station.py released a cue the instant sim-time crossed
`deliver_t` WITHOUT checking whether that frame's detection had actually
finished, a slow tick would ship a STALE or incomplete result -- a
correctness bug wearing a "just wall jitter" disguise.

So the release condition here is a CONJUNCTION, exactly as ADR-0048 SS3
mandates: `sim_time >= deliver_t_sim AND result_ready`. `CueScheduler`
below is that gate, fully decoupled from I/O (no socket, no gz) so it is
directly unit-testable (`tests/test_ground_station.py`) with a synthetic
clock and a synthetic readiness signal. Any tick where the deadline has
passed but the result is NOT ready is logged as a distinct `floor_violated`
event (`CueScheduler.floor_violation_log` / the emitted audit CSV's
`floor_violated` column) -- this is what makes the mechanism REAL rather
than cosmetic: a batch/gate can later assert delivered latency never reads
BELOW the floor (ADR-0048 SS3's own pre-registered T19 gate condition).

t_sim PROVENANCE (ADR-0048 SS3 (1), also GOALS.md's sim-time-not-wall-time
standing rule, ADR-0009) -- UPDATED by ADR-0052's epoch rebase: every
emitted payload's `"t_sim"` is `t0 + index.csv`'s baked `t_sim_nominal`
column, where `t0` is the LIVE `/clock` sim time sampled once, at the
moment this run started streaming (0.0 in `--replay-clock`/offline mode,
so that mode's `t_sim` is still the raw baked value, byte-identical to
pre-ADR-0052 behavior). It is STILL never this flight's live wall clock,
and STILL never the live sim clock's value AT RELEASE time (that continues
to only set WHEN a datagram may be released, never what timestamp it
carries) -- only the ONE-TIME `t0` sample at stream start is new. Why:
ADR-0052 found that the raw baked 0-2.7s clock has no relationship to a
real flight's live clock (station.py spawns tens of seconds into a real
flight, at CUE_WAIT entry) -- gating against the unrebased value either
bursts the whole cache in one microsecond (sim_t already far past every
`deliver_t_sim`) or works only by the accident of starting the process at
sim_t~0. Rebasing to `t0` streams the cache over `[t0, t0+cache_span]` of
THIS flight's live time, and makes `t_sim` the live sim time the target was
actually AT that position -- which is exactly what `CueReader`'s staleness
math (`ext_age_s = sim_clock.t - t_sim_cue`, `m4_intercept.py`) needs to
read as "latency", not as "~23s stale". Still immune to THIS run's RTF
inside the `[t0, t0+cache_span]` window, exactly like the mock's own
`t_sim_sample`.

=======================================================================
INTERPRETER FINDING (ADR-0048 SS5 -- resolved, no T19b blocker)
=======================================================================
ADR-0048 SS5 asked for a venv with BOTH gz-transport13 (`/clock`) and a
working detector. Checked on this machine (2026-07-08/09):

    venv               gz.transport13   ultralytics   onnxruntime   cv2
    .venv (main)             YES            no             no        --
    .venv-seeker              YES            no            YES      YES
    .venv-seeker-train        no            YES            YES       --

LITERALLY no venv has gz + ultralytics together. But this codebase's own
established pattern (`scripts/seeker/finetuned_seeker.py`, `nn_seeker.py`)
is to run flight-time inference via onnxruntime directly on an exported
`.onnx`, NEVER via ultralytics/torch at inference time (ultralytics is a
TRAINING/EXPORT-time tool only, per those modules' own docstrings) --
`scripts/seeker/weights/ground_v1.onnx` already exists (T17 exported it
alongside `ground_v1.pt`). `scripts/ground_station/detect.py`'s
`GroundDetector` therefore runs `ground_v1.onnx` via onnxruntime, matching
that established pattern -- so `.venv-seeker` (gz + onnxruntime + cv2 + numpy)
IS the one venv with everything this module's LIVE path needs. Verified: its
box centers on `logs/rig_captures/full_sweep_20260709T015530Z/` frames agree
with the T18 harness's ultralytics-derived centroids to sub-0.02 px (see
`detect.py`'s module docstring for the exact comparison). NOT a T19b
blocker; NO package installed to reach this conclusion. T19b's job: define
`GROUND_STATION_VENV_PYTHON` in `m4_intercept.py` pointing at
`.venv-seeker/bin/python` (ADR-0048 SS5's own naming), mirroring how
`CUE_MOCK_SCRIPT` is a computed path -- NOT done here (would touch a file
this task is explicitly told not to modify).

Because `.venv-seeker` has NO pytest (`.venv` main does), and this task's
offline tests must run without gz/onnxruntime being importable at all (per
the task brief), every import of `gz.transport13` / `onnxruntime` / `cv2` in
this module and in `detect.py` is DEFERRED (inside a function/method body,
never at module top level) -- so `import ground_station.station` succeeds
under bare `.venv`, and the offline dual-gate/schema/honesty tests never
need either dependency. `--replay-clock` (below) makes the LIVE-clock path
itself skippable too, for a no-gz smoke demonstration.

=======================================================================
HONESTY (ADR-0048 SS4's scope + GOALS.md's honesty boundary)
=======================================================================
This module holds exactly ONE gz subscription in its LIVE path: `/clock`.
It NEVER subscribes to a pose/gt topic of any kind -- unlike
`s2_cue_mock.py`, which legitimately reads the world pose-info topic to
SYNTHESIZE its degraded ground-sensor cue, this module's cue comes from
REAL pixels (pre-rendered frames on disk) run through the REAL detector +
triangulator. `index.csv`'s `gt_target_{x,y,z}` columns ARE read here, but
ONLY into this process's own audit CSV (`logs/ground_station_*.csv`) --
mirroring `s2_cue_mock.py`'s own "true position written to CSV log for
audit ONLY, never sent over the wire" pattern -- never into the emitted
:47800 payload and never into the triangulation math (which uses the RIG's
own known/assumed pose, a sensor-siting fact, not the target's gt).
`tests/test_ground_station.py::test_no_gt_or_pose_subscription` greps this
file's source for that boundary.

=======================================================================
CENTROID-CACHE REPLAY MODE (ADR-0051, T19b part-1 -- built here)
=======================================================================
ADR-0051 refines the "runs the real detect code LIVE during the flight"
language above: because the capture frames are pre-rendered and
DETERMINISTIC (ADR-0046), running `GroundDetector` on a given frame ahead
of time (`scripts/ground_station/build_centroid_cache.py`) yields the
IDENTICAL detection a live call during the flight would produce -- so
`--centroid-cache PATH` lets this module skip the live onnxruntime/cv2 call
entirely and instead look up each frame's already-computed centroid from a
`centroids.csv` built offline. `CachedDetector` below is a drop-in
replacement for `GroundDetector` (same one-method `.detect_path(path) ->
DetectionResult` contract `process_frame()` calls), so `process_frame()`,
`CueScheduler`, the dual gate, the track filter, and the :47800 emit path
are ALL completely unmodified by this mode -- only WHICH object plays the
`detector` role changes. Because a cache lookup is a dict access (not
~0.85-1.25 s of CPU inference, ADR-0051's T19a evidence), `result_ready`
becomes true almost immediately after `schedule()`, so the dual gate's
release timing in this mode is governed by the MODELED latency floor
(`--latency-s` / `DEFAULT_LATENCY_S`) rather than by unmodeled live compute
-- this is ADR-0051's whole point, and is exactly what
`tests/test_ground_station.py`'s cache-mode floor-violation test checks.

The LIVE path (no `--centroid-cache`) is UNCHANGED and still the default --
ADR-0051 keeps it for the hardware model (a real Pi/Jetson genuinely runs
detection concurrently with flight) and for reversibility (one flag).

=======================================================================
CONFIDENCE GATE (ADR-0052 addendum -- `--min-conf`, closes the T19 flight
finding, both detection modes)
=======================================================================
The FIRST live flight of this module (ADR-0052) found that a real
detection can technically "hit" on both cameras and still be USELESS: the
T19 stereo-cue flight's own `centroids.csv` seq=2 is the target's first
frame straddling the right camera's FOV edge (`u_r~4.9px`, `conf_r=0.32`
against a normal ~0.94-0.96) -- triangulating that low-confidence,
near-degenerate-disparity box anyway produced a ~43 m position error, a
single outlier that dominated the flight's cue-vs-target alignment check
even after the epoch-rebase fix above made every OTHER frame track to
~1 m (matching ADR-0050's expected real-detector sigma_R). `process_frame()`'s
`min_conf` parameter (`--min-conf`, default 0.5) rejects EITHER camera's
confidence below threshold as a `"rejected_low_conf"` status -- handled by
the EXACT SAME downstream code path as a real miss (`scheduler.cancel()`,
logged to the audit CSV, never blocks the FIFO) -- in BOTH detection modes,
because both `CachedDetector` and `GroundDetector` funnel through this one
`process_frame()` call; there is no separate gate to duplicate per mode.
This is a real-rig-fidelity fix, not a threshold tuned to erase one number:
a real ground station has no business triangulating a detection it isn't
confident in, cached replay or live.

=======================================================================
T19b OWES (explicitly out of scope for this task; NOT built here)
=======================================================================
  1. `m4_intercept.py --cue-source {mock,stereo}` wiring (spawn this file
     under `--cue-source stereo`, `GROUND_STATION_VENV_PYTHON` constant).
  2. A `check_t19.sh` flight gate asserting delivered sim-time latency never
     reads below the floor across a batch (ADR-0048 SS3's own condition).
  3. LIVE-clock validation (a real PX4/Gazebo flight actually subscribing
     `/clock` and driving this module's dual gate under real RTF/contention
     -- this task deliberately does not boot a simulator; `--replay-clock`
     is an OFFLINE stand-in for demonstration only, disclosed as such, never
     used by a gate). Applies to BOTH the LIVE and CACHED detection modes --
     `--centroid-cache` only changes where the centroid comes from, not the
     clock/gate mechanics.
  4. A real offline latency-floor profile "measured WITH Gazebo concurrently
     rendering" (ADR-0048 SS3) to replace `DEFAULT_LATENCY_S` below, which
     currently just MIRRORS `s2_cue_mock.DEFAULT_LATENCY_S` (0.12 s) for
     comparability, NOT because it has been profiled for this module's own
     (measured) ~0.25-0.35 s/frame-pair CPU onnxruntime compute cost -- see
     the smoke-run notes in the task report for why this default routinely
     trips `floor_violated` today in LIVE mode (the dual gate doing its job
     honestly on an unprofiled floor, not a bug) and why CACHED mode does
     not.
  5. Real `--spoof` corruption modes (`apply_spoof()` below is a documented
     T19a no-op stub).

Run (OFFLINE smoke demo, no PX4/Gazebo, LIVE detection, `.venv-seeker` for
onnxruntime):
    .venv-seeker/bin/python scripts/ground_station/station.py \\
        --capture-dir logs/rig_captures/full_sweep_20260709T015530Z \\
        --dash-direction r2l --replay-clock --seq-limit 8

Run (OFFLINE smoke demo, CACHED detection -- ADR-0051, no onnxruntime/cv2
needed at all once the cache exists; build it first with
build_centroid_cache.py, see that module's own docstring):
    .venv-seeker/bin/python scripts/ground_station/station.py \\
        --capture-dir logs/rig_captures/full_sweep_20260709T015530Z \\
        --centroid-cache logs/rig_captures/full_sweep_20260709T015530Z/centroids.csv \\
        --dash-direction r2l --replay-clock --seq-limit 8

Run (future T19b LIVE usage, PX4/Gazebo already up, /clock publishing):
    .venv-seeker/bin/python scripts/ground_station/station.py \\
        --capture-dir logs/rig_captures/full_sweep_20260709T015530Z \\
        --dash-direction r2l --port 47800
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import socket
import sys
import threading
import time
from typing import NamedTuple, Optional

HERE = os.path.dirname(os.path.abspath(__file__))          # scripts/ground_station
SCRIPTS_DIR = os.path.dirname(HERE)                         # scripts
REPO_ROOT = os.path.dirname(SCRIPTS_DIR)                    # repo root
sys.path.insert(0, SCRIPTS_DIR)

import stereo_model as sm  # noqa: E402  (reuse constants, not re-derive -- same as triangulate.py)
from ground_station.triangulate import (  # noqa: E402
    Pose4,
    RigConfig,
    apply_datum_bias,
    build_cue_payload,
    parse_pose4,
    triangulate,
)
from ground_station.track_filter import GroundTrackFilter  # noqa: E402
from ground_station.detect import (  # noqa: E402
    DEFAULT_CONF,
    DEFAULT_IMGSZ,
    DEFAULT_WEIGHTS,
    DetectionResult,
    GroundDetector,
    box_center,
)

LOGS_DIR = os.path.join(REPO_ROOT, "logs")
DEFAULT_CAPTURE_DIR = os.path.join(REPO_ROOT, "logs", "rig_captures", "full_sweep_20260709T015530Z")

CLOCK_TOPIC = "/clock"
DEFAULT_PORT = 47800                # matches s2_cue_mock.DEFAULT_PORT / m4_intercept.py's s2params["CUE_PORT"]
DEFAULT_LATENCY_S = 0.12            # MIRRORS s2_cue_mock.DEFAULT_LATENCY_S -- see "T19b OWES" #4 above:
                                     # this is NOT yet a profiled floor for THIS module's own compute cost.
DEFAULT_POLL_INTERVAL_S = 0.01      # tight poll, same rationale as s2_cue_mock.WALL_POLL_S
DEFAULT_SETUP_TIMEOUT_S = 15.0      # wall seconds to wait for the first live /clock sample
CSV_FLUSH_EVERY = 10
# ADR-0052 confidence gate: a detection with EITHER camera's confidence
# below this is treated as a MISS (rejected, not triangulated) -- see
# process_frame()'s own docstring for the full rationale/citation. 0.5
# rejects the T19 flight-diagnosed seq=2 conf=0.32 edge-of-FOV degenerate
# while keeping every ~0.94-0.96 normal frame in
# logs/rig_captures/full_sweep_20260709T015530Z/centroids.csv.
DEFAULT_MIN_CONF = 0.5

# Secondary feature (--emit-velocity, off by default): GroundTrackFilter's
# Kalata mode needs a measurement-noise scale at construction time (fixed,
# not per-tick -- track_filter.py's own contract, not re-derived here).
# DEFAULT_KALATA_NOMINAL_RANGE_M is a disclosed, NOT-per-capture-tuned
# placeholder (unlike e.g. gt_target_* audit fields, this is a DESIGN-TIME
# constant, never read from any per-flight ground truth) -- a real
# deployment would pass --kalata-nominal-range-m for its own actual handoff
# range, or --kalata-sigma-meas-m directly.
DEFAULT_KALATA_SIGMA_PROCESS_MPS2 = 1.0   # "gentle" tier, matches t18_sigma_validation.py's DASH_ACCEL_CLASS_GENTLE_MPS2
DEFAULT_KALATA_NOMINAL_RANGE_M = 100.0


# ============================================================================
# Sim clock -- LIVE (gz /clock) or --replay-clock (offline, wall-clock-driven)
# ============================================================================
class LatestSimClock:
    """Latest sim time from /clock. Gz-import-FREE class body on purpose
    (see module docstring's interpreter note): the gz message type is only
    needed by whoever calls `node.subscribe(Clock, CLOCK_TOPIC, ...)`
    (`make_live_node_and_clock()` below), never by this class itself --
    `on_clock` just reads `.sim.sec`/`.sim.nsec` off whatever object gz's
    callback machinery hands it. Same atomic-attribute pattern as
    `s2_cue_mock.LatestSimClock` (one attribute, replaced atomically under
    the GIL -- lock-free by construction, not by omission)."""

    def __init__(self):
        self.t: Optional[float] = None

    def on_clock(self, msg) -> None:
        self.t = msg.sim.sec + msg.sim.nsec * 1e-9


class ReplayClock:
    """OFFLINE stand-in for `LatestSimClock`, used ONLY by `--replay-clock`
    (no PX4/Gazebo process to publish a real /clock). `t` advances with WALL
    time (optionally scaled by `--replay-rtf`) -- explicitly NOT a real sim
    clock, disclosed here and at startup, and never wired into any gated
    check_*.sh (see module docstring's "T19b OWES" #3). Good enough to
    demonstrate the dual gate end-to-end against REAL detector compute time
    without booting a simulator."""

    def __init__(self, rtf: float = 1.0):
        self.rtf = rtf
        self._t0 = time.monotonic()

    @property
    def t(self) -> float:
        return (time.monotonic() - self._t0) * self.rtf


def make_live_node_and_clock(timeout_s: float = DEFAULT_SETUP_TIMEOUT_S):
    """LIVE mode: subscribe ONLY to `/clock` (deferred gz import -- module
    docstring). Blocks (wall-clock, `timeout_s`) until the first sample
    arrives, mirroring `s2_cue_mock.py`'s own setup wait. Returns
    `(node, clock)`; caller is responsible for `node.unsubscribe(CLOCK_TOPIC)`
    on shutdown."""
    from gz.transport13 import Node
    from gz.msgs10.clock_pb2 import Clock

    node = Node()
    clock = LatestSimClock()
    if not node.subscribe(Clock, CLOCK_TOPIC, clock.on_clock):
        raise RuntimeError(f"[ground-station] FAILED: could not subscribe to {CLOCK_TOPIC}")
    deadline = time.monotonic() + timeout_s
    while clock.t is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if clock.t is None:
        node.unsubscribe(CLOCK_TOPIC)
        raise RuntimeError(
            f"[ground-station] FAILED: no /clock sample within {timeout_s}s -- "
            "is PX4/Gazebo actually running?"
        )
    return node, clock


# ============================================================================
# Capture-directory loading (T16 layout: capture_meta.json + index.csv +
# left/right PNGs, matches ground_station.triangulate.RigConfig /
# t18_sigma_validation.load_capture exactly)
# ============================================================================
def load_capture(capture_dir: str):
    meta_path = os.path.join(capture_dir, "capture_meta.json")
    index_path = os.path.join(capture_dir, "index.csv")
    with open(meta_path) as f:
        meta = json.load(f)
    with open(index_path, newline="") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r["seq"] = int(r["seq"])
        r["t_sim_nominal"] = float(r["t_sim_nominal"])
    rows.sort(key=lambda r: r["seq"])
    return meta, rows


def select_dash_direction(rows: list, dash_direction: Optional[str] = None):
    """`index.csv`'s `t_sim_nominal` RESTARTS AT 0 for each `dash_direction`
    (two independent corridor passes baked into one T16 capture directory --
    see `capture_meta.json`'s `signs_captured: ["+", "-"]`). `CueScheduler`'s
    FIFO invariant needs a single monotonically-non-decreasing `frame_t_sim`
    stream, so a replay must pick exactly ONE direction. `dash_direction=
    None` (default) picks whichever direction the first row (by seq) belongs
    to -- deterministic and sufficient for an offline smoke run; T19b's job
    is to pass the direction matching that flight's actual mover/seed.
    Returns `(selected_rows, chosen_direction, all_directions_present)`."""
    all_directions = sorted(set(r["dash_direction"] for r in rows))
    if dash_direction is None:
        dash_direction = rows[0]["dash_direction"] if rows else None
    selected = [r for r in rows if r["dash_direction"] == dash_direction]
    return selected, dash_direction, all_directions


def frame_paths(capture_dir: str, seq: int):
    return (
        os.path.join(capture_dir, "left", f"{seq:05d}.png"),
        os.path.join(capture_dir, "right", f"{seq:05d}.png"),
    )


# ============================================================================
# Centroid cache (ADR-0051, T19b part-1) -- CachedDetector replay mode.
# `scripts/ground_station/build_centroid_cache.py` writes `centroids.csv` in
# this exact column shape (single source of truth: it imports
# CENTROID_CACHE_FIELDNAMES from here). See module docstring's
# "CENTROID-CACHE REPLAY MODE" section for the rationale.
# ============================================================================
CENTROID_CACHE_FIELDNAMES = [
    "seq", "t_sim", "det_l", "u_l", "v_l", "conf_l", "det_r", "u_r", "v_r", "conf_r",
]


def load_centroid_cache(path: str) -> dict:
    """Loads a `centroids.csv` (built by `build_centroid_cache.py`) into
    `{seq: {"l": DetectionResult, "r": DetectionResult, "t_sim": float}}`. A
    cache MISS on a side (`det_l`/`det_r` == 0) is stored as
    `DetectionResult(False, None, None)` -- the exact same shape a live
    `GroundDetector` miss produces, so `process_frame()`'s existing
    `not dl.hit or not dr.hit -> status="miss"` branch (and the scheduler's
    `cancel()` path it feeds) handles a cache miss with ZERO new code."""
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"centroid cache not found: {path} -- build it first with "
            "scripts/ground_station/build_centroid_cache.py (ADR-0051)"
        )
    cache: dict = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            seq = int(row["seq"])

            def _side(prefix, row=row):
                hit = str(row[f"det_{prefix}"]).strip() in ("1", "True", "true")
                if not hit:
                    return DetectionResult(False, None, None)
                u = float(row[f"u_{prefix}"])
                v = float(row[f"v_{prefix}"])
                conf = float(row[f"conf_{prefix}"])
                # Zero-size box so box_center() returns exactly (u, v) --
                # same convention tests/test_ground_station.py's
                # _synthetic_stub_detector() already uses.
                return DetectionResult(True, conf, (u, v, u, v))

            cache[seq] = {"l": _side("l"), "r": _side("r"), "t_sim": float(row["t_sim"])}
    if not cache:
        raise ValueError(f"centroid cache at {path} is empty (0 rows)")
    return cache


def _parse_cached_frame_path(path: str):
    """Inverse of `frame_paths()`: given a path of the form
    `.../{left,right}/NNNNN.png`, returns `(seq, side)` with
    `side in {"l", "r"}`. Raises ValueError loudly on anything that doesn't
    match that convention -- a cache-mode misuse (e.g. a stub path from a
    test not following this layout) should fail fast, not silently mis-key."""
    parent = os.path.basename(os.path.dirname(path))
    stem = os.path.splitext(os.path.basename(path))[0]
    if parent == "left":
        side = "l"
    elif parent == "right":
        side = "r"
    else:
        raise ValueError(
            f"cannot infer camera side from path {path!r} -- expected "
            ".../left/NNNNN.png or .../right/NNNNN.png (frame_paths()'s own convention)"
        )
    try:
        seq = int(stem)
    except ValueError:
        raise ValueError(f"cannot parse frame seq from path {path!r} (expected an integer filename stem)")
    return seq, side


class CachedDetector:
    """Drop-in replacement for `GroundDetector` (same one-method
    `.detect_path(path) -> DetectionResult` contract `process_frame()`
    calls), backed by a `load_centroid_cache()` dict. Zero onnxruntime/cv2
    import -- constructing and using this never touches either dependency,
    which is why `--centroid-cache` mode can run under a venv without them
    (see `test_station_module_imports_without_onnxruntime_or_cv2`'s
    counterpart for cache mode)."""

    def __init__(self, cache: dict):
        self.cache = cache

    def detect_path(self, path: str) -> DetectionResult:
        seq, side = _parse_cached_frame_path(path)
        entry = self.cache.get(seq)
        if entry is None:
            return DetectionResult(False, None, None)
        return entry[side]


# ============================================================================
# Per-frame detect -> triangulate (the "producer" work; blocking, real
# wall-clock compute in LIVE mode, an immediate dict lookup in CACHED mode --
# this is exactly what CueScheduler's readiness gate is timing).
# ============================================================================
def process_frame(row: dict, capture_dir: str, rig: RigConfig, assumed_pose: Pose4, detector,
                   min_conf: Optional[float] = None) -> dict:
    """Runs detection on BOTH camera frames for one `index.csv` row, then
    triangulates if both hit. `detector` needs only `.detect_path(path) ->
    DetectionResult` -- a real `GroundDetector` in production, any object
    with that one method in a test (no onnxruntime/cv2 import required to
    call this function with a stub).

    HONESTY: `row`'s `gt_target_{x,y,z}` are read here ONLY to pass through
    into the returned dict's audit fields (never used in the triangulation
    call itself, which uses `assumed_pose` -- the RIG's own known/assumed
    position, not the target's). Callers must treat `gt_target_*` as
    log-only, exactly like `s2_cue_mock.py`'s own true-position CSV columns.

    CONFIDENCE GATE (ADR-0052 addendum, closes the T19 seq=2 outlier):
    `min_conf` (`None` = disabled, the pre-gate default -- every EXISTING
    caller that doesn't pass it is byte-identical) rejects a detection as a
    MISS -- same `"status"`, same downstream `cancel()`/audit handling as a
    real det=0 miss, just a distinct `"rejected_low_conf"` status string so
    it is separately countable -- when EITHER camera's confidence is below
    `min_conf`, even though both cameras technically hit. Rationale: a real
    ground rig should not triangulate off a low-confidence, edge-of-FOV,
    tiny-disparity detection any more than it should off a miss -- doing so
    is exactly how T19's flight-diagnosed seq=2 outlier happened (ADR-0052:
    `centroids.csv` seq=2 is the target's first frame straddling the right
    camera's FOV edge, `u_r~4.9px`, `conf_r=0.32` against a normal
    ~0.94-0.96, and triangulating it anyway produced a ~43 m position error
    -- ADR-0050's own single-range sigma_R checkpoint already flagged real-
    detector edge cases as the open risk this closes). `station.py --min-
    conf` defaults to 0.5, which rejects that one conf=0.32 frame while
    keeping every ~0.94+ normal frame.

    Returns a dict with `"status"` in
    `{"hit", "miss", "rejected_low_conf", "degenerate"}`.
    """
    seq = row["seq"]
    left_path, right_path = frame_paths(capture_dir, seq)
    dl: DetectionResult = detector.detect_path(left_path)
    dr: DetectionResult = detector.detect_path(right_path)

    def _gt(key):
        v = row.get(key)
        return float(v) if v not in (None, "") else None

    base = {
        "seq": seq, "frame_t_sim": row["t_sim_nominal"],
        "det_conf_left": dl.conf, "det_conf_right": dr.conf,
        "gt_target_x": _gt("gt_target_x"), "gt_target_y": _gt("gt_target_y"),
        "gt_target_z": _gt("gt_target_z"),
    }
    if not dl.hit or not dr.hit:
        base["status"] = "miss"
        return base

    if min_conf is not None and (
        (dl.conf is not None and dl.conf < min_conf)
        or (dr.conf is not None and dr.conf < min_conf)
    ):
        base["status"] = "rejected_low_conf"
        return base

    u_l, v_l = box_center(dl.box_xyxy)
    u_r, v_r = box_center(dr.box_xyxy)
    tri = triangulate(u_l, v_l, u_r, v_r, assumed_pose, rig.baseline_m, rig.f_px, rig.cx, rig.cy)
    if tri is None:
        base.update({"status": "degenerate", "u_l": u_l, "v_l": v_l, "u_r": u_r, "v_r": v_r})
        return base

    base.update({
        "status": "hit", "u_l": u_l, "v_l": v_l, "u_r": u_r, "v_r": v_r,
        "x": tri.x, "y": tri.y, "z": tri.z,
        "range_m": tri.range_m, "disparity_px": tri.disparity_px,
    })
    return base


# ============================================================================
# --spoof (ADR-0048 SS5.6 #2's "emit-path track-message spoof injector"):
# documented T19a NO-OP STUB, real corruption modes are T19b work.
# ============================================================================
def apply_spoof(payload: dict, spoof_mode: Optional[str]) -> dict:
    """`None`/`"none"` (default) is a byte-identical no-op: `payload`
    returned unchanged. Any OTHER value currently raises -- refusing to
    silently pretend a requested spoof mode did something it didn't."""
    if spoof_mode is None or spoof_mode == "none":
        return payload
    raise NotImplementedError(
        f"--spoof {spoof_mode!r} is a documented T19a no-op stub (see "
        "apply_spoof()'s docstring) -- real corruption modes are T19b work "
        "(ADR-0048 SS5.6 #2's 'emit-path track-message spoof injector'). "
        "Pass --spoof none (or omit --spoof) for T19a's no-op behavior."
    )


# ============================================================================
# CueScheduler -- THE DUAL GATE (ADR-0048 SS3). See module docstring's
# "THE DUAL GATE" section for the full rationale; this is the core
# deliverable of T19a.
# ============================================================================
class ReleasedItem(NamedTuple):
    seq: int
    payload: dict
    meta: dict
    floor_violated: bool
    frame_t_sim: float
    deliver_t_sim: float


class CueScheduler:
    """Ported IN SPIRIT from `s2_cue_mock.py`'s pending-list buffer-and-
    release loop (its own lines ~700-744: append a `deliver_t`-tagged item,
    drain the FIFO front while `pending[0]["deliver_t"] <= now`) -- but the
    release condition here is the ADR-0048 SS3 CONJUNCTION
    `sim_t >= deliver_t_sim AND result_ready`, not sim-time alone. Fully
    decoupled from I/O (no socket, no gz) so it is directly unit-testable.

    FIFO INVARIANT: `schedule()` calls must arrive in non-decreasing
    `deliver_t_sim` order (true for `station.py`'s real usage: frames
    process in `t_sim_nominal` order and the latency floor is a constant
    added to each). `poll()` only ever inspects the FRONT of the queue --
    a not-yet-ready front item BLOCKS every later item from releasing even
    if THEIR results are ready. This is intentional (a real UDP cue stream
    must stay seq-ordered): it is exactly the scenario `floor_violated`
    exists to make visible, not silently paper over with an out-of-order
    release.

    THREAD SAFETY: `schedule()`/`complete()`/`cancel()`/`poll()` all take an
    internal lock -- `station.py`'s real usage has one producer thread
    (running real detector compute) and the main thread (polling the sim
    clock and releasing) touching this concurrently; every offline test
    below drives it single-threaded, where the lock is simply inert.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.pending: list = []              # FIFO, oldest deliver_t first
        self.emitted: list = []              # [(seq, payload), ...] in release order
        self.floor_violation_log: list = []  # one dict per NEWLY-observed violation
        self.cancelled: list = []            # [(seq, reason), ...]

    def schedule(self, seq, frame_t_sim: float, deliver_t_sim: float, meta: Optional[dict] = None) -> None:
        """Enqueue a not-yet-ready item. Call this BEFORE starting the
        (possibly slow) detect+triangulate work for `seq`, so a `poll()`
        during that work can actually observe `result_ready=False`."""
        with self._lock:
            self.pending.append({
                "seq": seq, "frame_t_sim": frame_t_sim, "deliver_t_sim": deliver_t_sim,
                "result_ready": False, "payload": None,
                "floor_violated_logged": False, "meta": dict(meta or {}),
            })

    def complete(self, seq, payload: dict, meta: Optional[dict] = None) -> None:
        """Mark `seq`'s result ready, attaching the (already fully-built)
        cue payload. `meta` (if given) is merged into the item's audit meta
        (e.g. detection confidences computed only once compute finished)."""
        with self._lock:
            item = self._find(seq)
            item["result_ready"] = True
            item["payload"] = payload
            if meta:
                item["meta"].update(meta)

    def cancel(self, seq, reason: Optional[str] = None) -> None:
        """A frame that will NEVER produce a cue (detection miss on either
        camera, or a degenerate/behind-camera triangulation) -- removed from
        the FIFO so it can't block later items forever. Deliberately NOT a
        `floor_violated` event: nothing was "not ready yet", there is simply
        no result coming."""
        with self._lock:
            item = self._find(seq)
            self.pending.remove(item)
            self.cancelled.append((seq, reason))

    def _find(self, seq):
        for item in self.pending:
            if item["seq"] == seq:
                return item
        raise KeyError(f"seq={seq!r} not pending (never scheduled, or already completed/cancelled)")

    def poll(self, sim_t: float):
        """Drain the FIFO front while BOTH dual-gate conditions hold at
        `sim_t`. Returns a list of `ReleasedItem`s released THIS call, in
        release order. `.floor_violated` is True iff THIS item's deadline
        was observed to have passed on some poll() call (this one or an
        earlier one) while it was still not-ready -- i.e. it reflects the
        item's own history, not just the instant of release."""
        released = []
        with self._lock:
            while self.pending:
                item = self.pending[0]
                deadline_passed = sim_t >= item["deliver_t_sim"]
                if not deadline_passed:
                    break
                if not item["result_ready"]:
                    if not item["floor_violated_logged"]:
                        item["floor_violated_logged"] = True
                        self.floor_violation_log.append({
                            "seq": item["seq"], "sim_t": sim_t,
                            "deliver_t_sim": item["deliver_t_sim"],
                            "frame_t_sim": item["frame_t_sim"],
                        })
                    break  # FIFO: cannot skip ahead to a later, maybe-ready item
                popped = self.pending.pop(0)
                released.append(ReleasedItem(
                    popped["seq"], popped["payload"], popped["meta"],
                    popped["floor_violated_logged"], popped["frame_t_sim"], popped["deliver_t_sim"],
                ))
                self.emitted.append((popped["seq"], popped["payload"]))
        return released

    @property
    def n_pending(self) -> int:
        return len(self.pending)


# ============================================================================
# Audit CSV (mirrors s2_cue_mock.py's "log everything, send only the
# degraded/derived cue" pattern -- gt_target_* columns here are LOG-ONLY,
# never in the UDP payload).
# ============================================================================
AUDIT_FIELDNAMES = [
    "event", "seq", "frame_t_sim", "deliver_t_sim", "t_sim_emit", "latency_actual_s",
    "floor_violated", "x", "y", "z", "vx", "vy", "vz",
    "det_conf_left", "det_conf_right", "disparity_px", "range_m",
    "gt_target_x", "gt_target_y", "gt_target_z",
]


class AuditLog:
    def __init__(self, path: str):
        self.path = path
        self._lock = threading.Lock()
        self._fh = open(path, "w", newline="")
        self._writer = csv.DictWriter(self._fh, fieldnames=AUDIT_FIELDNAMES)
        self._writer.writeheader()
        self._fh.flush()
        self._n = 0

    def write(self, row: dict) -> None:
        with self._lock:
            self._writer.writerow({k: row.get(k, "") for k in AUDIT_FIELDNAMES})
            self._n += 1
            if self._n % CSV_FLUSH_EVERY == 0:
                self._fh.flush()

    def close(self) -> None:
        with self._lock:
            self._fh.flush()
            self._fh.close()


# ============================================================================
# Producer / consumer loops (the real, threaded orchestration -- offline
# tests exercise CueScheduler/process_frame directly instead of these).
# ============================================================================
def _producer_loop(frames, capture_dir, rig, assumed_pose, detector, scheduler: CueScheduler,
                    latency_s, emit_velocity, kalata_sigma_process, kalata_sigma_meas,
                    spoof_mode, audit: AuditLog, stop_event: threading.Event,
                    finished_event: threading.Event, error_box: list, seq_limit=None,
                    t0: float = 0.0, min_conf: Optional[float] = None):
    """`error_box` is a plain list used as an out-parameter (a thread's
    return value / raised exception is otherwise invisible to the joining
    caller): on ANY unexpected exception mid-frame, this appends it,
    `cancel()`s the in-flight scheduled item (if still pending) so it can
    never wedge the consumer's FIFO drain forever, and stops -- found the
    hard way in manual smoke-testing (`--spoof <unimplemented mode>` used to
    raise INSIDE the loop, after `schedule()` but before `complete()`/
    `cancel()`, permanently stranding that seq as pending and hanging
    `_consumer_loop` forever since `finished_event` alone was never
    sufficient -- `n_pending` never reached 0). `--spoof` itself is now ALSO
    validated once at startup in `main()`, before any thread starts, so this
    is defense-in-depth for any OTHER future per-frame exception, not the
    primary fix for that specific case.

    `t0` (ADR-0052 epoch rebase): the LIVE sim time at which this run's
    streaming started (0.0 in `--replay-clock`/offline mode and in every
    existing caller that doesn't pass it -- byte-identical to pre-ADR-0052
    behavior, since `0.0 + x == x` exactly for any finite float). Every
    per-frame `frame_t_sim` used below is `t0 + row["t_sim_nominal"]`, NOT
    the raw baked capture-time value -- see module docstring's "t_sim
    PROVENANCE" section for why: the cache's baked 0-2.7s clock has no
    relationship to THIS flight's live clock (ADR-0052's root cause), so
    gating/emitting against the raw baked value either bursts the whole
    cache the instant `sim_t` first exceeds ~2.7s (LIVE mode, spawned tens
    of seconds into a real flight) or is fine by accident (the old
    sim-time-0 smoke test). Rebasing to `t0` streams the cache over
    `[t0, t0 + cache_span]` of live flight time instead, and `deliver_t_sim`
    is `frame_t_sim + latency_s` in that SAME rebased frame -- so it still
    correctly bounds the modeled latency floor above the true release time.

    `min_conf` (ADR-0052 confidence gate): forwarded straight to
    `process_frame()` -- see that function's own docstring. `None` (default,
    every caller that doesn't pass it) disables the gate."""
    track_filter = GroundTrackFilter(
        kalata_sigma_process=kalata_sigma_process, kalata_sigma_meas=kalata_sigma_meas,
    ) if emit_velocity else None
    last_track_t = None
    n = 0
    seq = None
    try:
        for row in frames:
            if stop_event.is_set():
                break
            if seq_limit is not None and n >= seq_limit:
                break
            n += 1
            # ADR-0052 epoch rebase: frame_t_sim is t0 + the frame's BAKED
            # capture-time offset, not the raw baked value -- see this
            # function's own docstring and module docstring's "t_sim
            # PROVENANCE" section. t0=0.0 (every caller that doesn't pass it,
            # including --replay-clock) makes this an exact no-op.
            seq = row["seq"]
            frame_t_sim = t0 + row["t_sim_nominal"]
            deliver_t_sim = frame_t_sim + latency_s
            scheduler.schedule(seq, frame_t_sim, deliver_t_sim)

            res = process_frame(row, capture_dir, rig, assumed_pose, detector, min_conf=min_conf)
            if res["status"] != "hit":
                scheduler.cancel(seq, reason=res["status"])
                audit.write({
                    "event": res["status"], "seq": seq, "frame_t_sim": frame_t_sim,
                    "deliver_t_sim": deliver_t_sim,
                    "det_conf_left": res.get("det_conf_left"), "det_conf_right": res.get("det_conf_right"),
                    "gt_target_x": res.get("gt_target_x"), "gt_target_y": res.get("gt_target_y"),
                    "gt_target_z": res.get("gt_target_z"),
                })
                continue

            vel = None
            if track_filter is not None:
                dt = (frame_t_sim - last_track_t) if (track_filter.initialized and last_track_t is not None) else None
                state = track_filter.step(res["x"], res["y"], res["z"], frame_t_sim, dt=dt)
                last_track_t = frame_t_sim
                vel = (state.vx, state.vy, state.vz)

            payload = (build_cue_payload(seq, frame_t_sim, res["x"], res["y"], res["z"], *vel)
                       if vel is not None else
                       build_cue_payload(seq, frame_t_sim, res["x"], res["y"], res["z"]))
            payload = apply_spoof(payload, spoof_mode)
            meta = {
                "det_conf_left": res.get("det_conf_left"), "det_conf_right": res.get("det_conf_right"),
                "disparity_px": res.get("disparity_px"), "range_m": res.get("range_m"),
                "gt_target_x": res.get("gt_target_x"), "gt_target_y": res.get("gt_target_y"),
                "gt_target_z": res.get("gt_target_z"),
            }
            if vel is not None:
                meta["vx"], meta["vy"], meta["vz"] = vel
            scheduler.complete(seq, payload, meta=meta)
    except Exception as exc:  # noqa: BLE001 -- deliberately broad: see docstring
        error_box.append(exc)
        if seq is not None:
            try:
                scheduler.cancel(seq, reason="producer_error")
            except KeyError:
                pass  # already completed/cancelled before the exception hit
        stop_event.set()
    finally:
        finished_event.set()


def _consumer_loop(clock, scheduler: CueScheduler, sock, dest, stop_event: threading.Event,
                    finished_event: threading.Event, audit: AuditLog, poll_interval_s: float) -> int:
    n_emitted = 0
    while True:
        if stop_event.is_set():
            break
        t = clock.t
        if t is not None:
            for item in scheduler.poll(t):
                sock.sendto(json.dumps(item.payload).encode("utf-8"), dest)
                n_emitted += 1
                audit.write({
                    "event": "emit", "seq": item.seq,
                    "frame_t_sim": item.frame_t_sim, "deliver_t_sim": item.deliver_t_sim,
                    "t_sim_emit": t, "latency_actual_s": t - item.frame_t_sim,
                    "floor_violated": int(bool(item.floor_violated)),
                    "x": item.payload.get("x"), "y": item.payload.get("y"), "z": item.payload.get("z"),
                    "vx": item.payload.get("vx"), "vy": item.payload.get("vy"), "vz": item.payload.get("vz"),
                    "det_conf_left": item.meta.get("det_conf_left"),
                    "det_conf_right": item.meta.get("det_conf_right"),
                    "disparity_px": item.meta.get("disparity_px"), "range_m": item.meta.get("range_m"),
                    "gt_target_x": item.meta.get("gt_target_x"), "gt_target_y": item.meta.get("gt_target_y"),
                    "gt_target_z": item.meta.get("gt_target_z"),
                })
        if finished_event.is_set() and scheduler.n_pending == 0:
            break
        time.sleep(poll_interval_s)
    return n_emitted


# ============================================================================
# CLI
# ============================================================================
def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture-dir", default=DEFAULT_CAPTURE_DIR,
                     help="T16/T18 capture directory (capture_meta.json + index.csv + left/right PNGs)")
    ap.add_argument("--dash-direction", choices=("r2l", "l2r"), default=None,
                     help="which of the capture's two corridor passes to replay (default: whichever "
                          "the first row belongs to -- see select_dash_direction()'s docstring for why "
                          "exactly one direction must be chosen)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP port on 127.0.0.1 to emit cues to")
    ap.add_argument("--latency-s", type=float, default=DEFAULT_LATENCY_S,
                     help="constant modeled latency floor added to each frame's baked t_sim "
                          "(default mirrors s2_cue_mock's DEFAULT_LATENCY_S; see module docstring "
                          "'T19b OWES' #4 for why this is a placeholder, not a profiled number)")
    ap.add_argument("--weights", default=DEFAULT_WEIGHTS, help="ground_v1 .onnx weights")
    ap.add_argument("--conf", type=float, default=DEFAULT_CONF, help="detector confidence threshold")
    ap.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ, help="detector inference size")
    ap.add_argument("--min-conf", type=float, default=DEFAULT_MIN_CONF,
                     help="ADR-0052 confidence gate: reject a detection (in BOTH cached and live "
                          "modes) as a MISS -- no triangulation, no cue, doesn't block the FIFO -- "
                          "when EITHER camera's confidence is below this, even if both cameras "
                          "technically hit. A real ground rig should not triangulate off a "
                          "low-confidence/edge-of-FOV/tiny-disparity detection any more than off "
                          "a miss (see process_frame()'s docstring for the T19 seq=2 incident this "
                          "closes: conf_r=0.32 -> ~43 m triangulation error). Default 0.5.")
    ap.add_argument("--centroid-cache", type=str, default=None,
                     help="ADR-0051 T19b part-1: path to a centroids.csv built by "
                          "scripts/ground_station/build_centroid_cache.py. When given, "
                          "replays those PRECOMPUTED centroids through the (still LIVE) "
                          "triangulate+track+emit pipeline instead of calling GroundDetector "
                          "live -- no onnxruntime/cv2 import at all in this mode. "
                          "--weights/--conf/--imgsz are ignored when this is set. "
                          "Default: None (LIVE detection, unchanged T19a behavior).")
    ap.add_argument("--datum-bias-m", type=float, default=None,
                     help="ADR-0046 SS5.6 #2 / ADR-0048 SS2: shift the ASSUMED rig pose by this many "
                          "metres along the rig's own boresight (mutually exclusive with "
                          "--assumed-rig-pose); see ground_station.triangulate.apply_datum_bias")
    ap.add_argument("--assumed-rig-pose", type=parse_pose4, default=None,
                     help="'x,y,z,yaw' -- assumed rig pose used for triangulation "
                          "(default: the TRUE pose from capture_meta.json)")
    ap.add_argument("--emit-velocity", action="store_true",
                     help="also run a GroundTrackFilter over the triangulated positions and append "
                          "vx/vy/vz to the emitted payload (default off -- 5-key payload)")
    ap.add_argument("--kalata-sigma-process", type=float, default=DEFAULT_KALATA_SIGMA_PROCESS_MPS2)
    ap.add_argument("--kalata-nominal-range-m", type=float, default=DEFAULT_KALATA_NOMINAL_RANGE_M,
                     help="design-time nominal range used to size the track filter's Kalata "
                          "measurement-noise constant (NOT read from any per-flight ground truth)")
    ap.add_argument("--kalata-sigma-meas-m", type=float, default=None,
                     help="override: skip the nominal-range derivation and use this sigma_R directly")
    ap.add_argument("--spoof", type=str, default=None,
                     help="ADR-0048 SS5.6 #2's emit-path track-message spoof injector -- "
                          "T19a NO-OP STUB, see apply_spoof()'s docstring")
    ap.add_argument("--replay-clock", action="store_true",
                     help="OFFLINE demo mode: a synthetic wall-clock-driven clock, no PX4/Gazebo "
                          "/clock subscription. NOT a real sim clock -- disclosed, never gated.")
    ap.add_argument("--replay-rtf", type=float, default=1.0,
                     help="--replay-clock's wall-to-sim-time multiplier (default 1.0)")
    ap.add_argument("--seq-limit", type=int, default=None,
                     help="process at most this many frames (smoke-test convenience)")
    ap.add_argument("--poll-interval-s", type=float, default=DEFAULT_POLL_INTERVAL_S)
    ap.add_argument("--setup-timeout-s", type=float, default=DEFAULT_SETUP_TIMEOUT_S)
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    if args.assumed_rig_pose is not None and args.datum_bias_m is not None:
        print("[ground-station] FAILED: --assumed-rig-pose and --datum-bias-m are mutually exclusive")
        return 2

    # Validate --spoof ONCE here, before any thread starts (fail fast on a
    # bad CLI arg) -- NOT deferred to the first frame that happens to hit,
    # which is what used to hang the whole process (see _producer_loop's
    # docstring for the incident this fixes).
    try:
        apply_spoof({}, args.spoof)
    except NotImplementedError as exc:
        print(f"[ground-station] FAILED: {exc}")
        return 2

    os.makedirs(LOGS_DIR, exist_ok=True)
    from datetime import datetime, timezone
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    audit = AuditLog(os.path.join(LOGS_DIR, f"ground_station_{timestamp}.csv"))
    print(f"[ground-station] audit log: {audit.path}")

    stop_event = threading.Event()
    finished_event = threading.Event()

    def _handle_signal(signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    meta, rows = load_capture(args.capture_dir)
    frames, chosen_direction, all_directions = select_dash_direction(rows, args.dash_direction)
    print(f"[ground-station] capture_dir={args.capture_dir} directions_present={all_directions} "
          f"chosen={chosen_direction} n_frames={len(frames)}")

    rig = RigConfig.from_capture_meta(os.path.join(args.capture_dir, "capture_meta.json"))
    if args.assumed_rig_pose is not None:
        assumed_pose = args.assumed_rig_pose
    elif args.datum_bias_m is not None:
        assumed_pose = apply_datum_bias(rig.true_pose, args.datum_bias_m)
    else:
        assumed_pose = rig.true_pose
    print(f"[ground-station] rig true_pose={tuple(rig.true_pose)} assumed_pose={tuple(assumed_pose)}")
    print(f"[ground-station] ADR-0052 confidence gate: min_conf={args.min_conf} "
          "(either camera below this -> rejected_low_conf, treated as a miss)")

    if args.centroid_cache:
        try:
            cache = load_centroid_cache(args.centroid_cache)
        except (FileNotFoundError, ValueError) as exc:
            print(f"[ground-station] FAILED: {exc}")
            audit.close()
            return 2
        detector = CachedDetector(cache)
        print(f"[ground-station] detection mode: CACHED (ADR-0051 T19b part-1) -- "
              f"centroid_cache={args.centroid_cache} n_seq={len(cache)} "
              "(no onnxruntime/cv2 import, no live detector compute)")
    else:
        try:
            detector = GroundDetector(weights=args.weights, conf=args.conf, imgsz=args.imgsz)
        except ImportError as exc:
            print(f"[ground-station] FAILED: detector deps missing ({exc}). Run under a venv with "
                  "onnxruntime + opencv (this checkout: .venv-seeker) -- see module docstring's "
                  "interpreter note.")
            audit.close()
            return 2
        print(f"[ground-station] detection mode: LIVE -- weights={args.weights} conf={args.conf} "
              f"imgsz={args.imgsz} (onnxruntime CPU compute per frame)")

    if args.replay_clock:
        clock = ReplayClock(rtf=args.replay_rtf)
        node = None
        # ADR-0052: t0=0.0 in offline mode -- no epoch rebase, byte-identical
        # to pre-ADR-0052 behavior (this mode's t_sim stays the raw baked
        # value; there is no real flight clock to rebase against).
        t0 = 0.0
        print(f"[ground-station] --replay-clock ON (rtf={args.replay_rtf}) -- NOT a real sim clock, "
              "offline demo only (module docstring 'T19b OWES' #3); t0=0.0 (no epoch rebase)")
    else:
        try:
            node, clock = make_live_node_and_clock(timeout_s=args.setup_timeout_s)
        except (ImportError, RuntimeError) as exc:
            print(f"[ground-station] FAILED: {exc}")
            audit.close()
            return 2
        # ADR-0052 epoch rebase: t0 = the live /clock sim time RIGHT NOW, at
        # the moment streaming starts -- every cache frame's frame_t_sim/
        # deliver_t_sim/emitted t_sim is t0 + its baked offset from here on
        # (see _producer_loop's docstring). This is the fix for the T19
        # clock-epoch bug (ADR-0052): station.py spawns tens of seconds into
        # a real flight, so gating against the raw 0-2.7s baked clock bursts
        # the whole cache in one microsecond.
        t0 = clock.t
        print(f"[ground-station] subscribed to {CLOCK_TOPIC}; ADR-0052 epoch rebase t0={t0:.4f}s "
              "(cache will stream over [t0, t0+cache_span] of this flight's live sim time)")

    kalata_sigma_meas = (
        args.kalata_sigma_meas_m if args.kalata_sigma_meas_m is not None
        else float(sm.sigma_R(args.kalata_nominal_range_m, rig.baseline_m, rig.f_px, "expected"))
    )

    scheduler = CueScheduler()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("127.0.0.1", args.port)

    print("[ground-station] streaming started")

    error_box: list = []  # producer thread's out-of-band exception channel (see _producer_loop docstring)
    worker = threading.Thread(
        target=_producer_loop,
        args=(frames, args.capture_dir, rig, assumed_pose, detector, scheduler,
              args.latency_s, args.emit_velocity, args.kalata_sigma_process, kalata_sigma_meas,
              args.spoof, audit, stop_event, finished_event, error_box, args.seq_limit),
        kwargs={"t0": t0, "min_conf": args.min_conf},  # ADR-0052 epoch rebase + confidence gate
        daemon=False,
    )
    worker.start()

    n_emitted = _consumer_loop(clock, scheduler, sock, dest, stop_event, finished_event,
                                audit, args.poll_interval_s)
    worker.join(timeout=5.0)

    sock.close()
    if node is not None:
        node.unsubscribe(CLOCK_TOPIC)
    audit.close()

    n_floor_violations = len(scheduler.floor_violation_log)
    n_missed = sum(1 for _, r in scheduler.cancelled if r == "miss")
    n_rejected_low_conf = sum(1 for _, r in scheduler.cancelled if r == "rejected_low_conf")
    n_degenerate = sum(1 for _, r in scheduler.cancelled if r == "degenerate")
    print(
        f"[ground-station] done: n_emitted={n_emitted} n_missed={n_missed} "
        f"n_rejected_low_conf={n_rejected_low_conf} "
        f"n_degenerate={n_degenerate} n_floor_violations={n_floor_violations} log={audit.path}"
    )
    if error_box:
        print(f"[ground-station] FAILED: producer thread raised: {error_box[0]!r}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
