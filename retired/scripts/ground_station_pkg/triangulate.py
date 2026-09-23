#!/usr/bin/env python3
r"""T18 -- real stereo triangulation: pixel centroids (left, right) + rig
config -> disparity -> range -> 3D WORLD position.

Companion to `docs/stereo_design.md` (the physics) and `scripts/
stereo_model.py` (the calibrated constants -- REUSED here, not re-derived:
`f_px`, `sigma_match_px` tiers, `CHOSEN_BASELINE_M`). Feeds `scripts/
t18_sigma_validation.py` (the perfect-centroid floor mode) and, later, a real
`ground_station.py` (T19) that swaps ground-truth-derived centroids for T17's
NN detections without touching any of the math below.

WHY THIS FILE HOLDS NO GZ/MAVSDK IMPORTS: it must run offline, against
already-rendered/captured frames or synthetic centroids, with no simulator
and no live process (docs/decisions.md ADR-0046 decision item 2: triangulate
captured pairs under perturbed extrinsics, CPU-only, no re-render).

=======================================================================
RIG GEOMETRY (matches `models/ground_stereo_rig/model.sdf` EXACTLY)
=======================================================================
The rig is one static link at world pose (rx, ry, rz, yaw) -- no pitch/roll,
per `logs/rig_captures/*/capture_meta.json`'s `rig_pose_xyz_yaw`. Two camera
sensors sit at link-LOCAL offsets `(0, +1.0, 0)` (`rig_left`) and
`(0, -1.0, 0)` (`rig_right`), i.e. +/- half the 2.0 m baseline along the
link's local Y axis, both boresights along local +X (fronto-parallel, no
toe-in -- matches `scripts/rig_geometry_analysis.py`'s wedge model).

Camera axes, OpenCV convention (docs/goals.md "Coordinate frames": z forward, x
right, y down), derived from the rig's world yaw (up = world +Z, since the
rig has no pitch/roll):

    forward = ( cos(yaw),  sin(yaw), 0)      -- camera z
    right   = ( sin(yaw), -cos(yaw), 0)      -- camera x  (= forward x up)
    down    = ( 0, 0, -1)                    -- camera y

Cross-checked against the SDF at yaw=0: `right = (0, -1, 0)`, so
`rig_right`'s local `-Y` offset is `+1.0 * right` (physically the camera's
own right side) and `rig_left`'s local `+Y` offset is `-1.0 * right` (the
camera's own left side) -- the SDF sensor names match their physical side.

Standard fronto-parallel stereo then gives (b = baseline, f_px = focal
length in pixels, d = u_left - u_right = disparity):

    Z (depth along `forward`, measured from the RIG CENTER -- identical for
       both cameras because their offset from the rig center is purely
       LATERAL, i.e. perpendicular to `forward`) = b * f_px / d
    X (lateral offset from rig center, along `right`)  = (u_avg - cx) * Z / f_px
    Y (vertical offset from rig center, along `down`)  = (v_avg - cy) * Z / f_px
    P_world = rig_center + Z*forward + X*right + Y*down

where `u_avg = (u_left + u_right)/2`, `v_avg = (v_left + v_right)/2` (the
usual noise-averaging central-point formula; v_left == v_right exactly for a
true, noise-free observation because the rig has no vertical baseline).

=======================================================================
DATUM BIAS / ASSUMED-POSE INJECTION (ADR-0046 decision item 3; binds
docs/phase2_sim_to_real_plan.md Sec 5.6 finding #2)
=======================================================================
In sim, the rig and the target share one world frame, so a real
triangulation pipeline has ZERO natural extrinsics error -- unlike a fielded
rig, whose GPS/survey-derived position is never exactly its true position
(docs/stereo_design.md: 0.3/0.5/2.5 m BEST/EXPECTED/WORST datum floor). To
make that error INJECTABLE and its effect on the output PHYSICALLY FAITHFUL
(not a hand-waved offset added after the fact), `triangulate()` takes an
`assumed_rig_pose` SEPARATE from the true pose used to interpret the pixels:
the pixels themselves are exactly what the (truly-positioned) cameras would
have produced; the software's belief about WHERE the rig is is what's wrong.

Algebraically (see the module's docstring derivation in the task report):
for a pure position bias with the ASSUMED YAW EQUAL to the true yaw, the
reconstructed point is

    P_hat = P_true + (assumed_rig_center - true_rig_center)

i.e. a CONSTANT VECTOR SHIFT of the true position, independent of range or
bearing -- exactly the physical signature of a GPS/survey datum offset
shifting the whole reported map by a fixed vector. `--datum-bias-m B`
therefore injects a bias of magnitude B along the rig's own BORESIGHT
(`forward`) axis -- disclosed here, not tuned for effect: the range/depth
axis is this rig's weakest metrology axis (`docs/stereo_design.md`:
"both sensors are angle-strong, range-weak"), so it is the natural,
worst-case-flavored default direction; any OTHER direction would produce
the identical B-metre output shift per the algebra above, since only a
YAW (pointing) error, not a pure position error, makes the error grow with
range. `--assumed-rig-pose x,y,z,yaw` is the general escape hatch (also
lets a caller inject a POINTING error, which the position-only
`--datum-bias-m` shorthand deliberately does not).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import NamedTuple, Optional

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, SCRIPTS_DIR)
import stereo_model as sm  # noqa: E402  (reuse constants, not re-derive -- task instruction)


# ============================================================================
# Rig geometry / camera model
# ============================================================================
class Pose4(NamedTuple):
    """A rig pose: world (x, y, z) + yaw (rad). No pitch/roll -- matches
    `models/ground_stereo_rig/model.sdf` (static link, yaw-only placement)."""
    x: float
    y: float
    z: float
    yaw: float


def f_px_from_hfov(width_px: float, hfov_rad: float) -> float:
    """f_px = (W/2) / tan(HFOV/2) -- pixels-per-radian at a fixed lens FOV.
    Matches `scripts/rig_geometry_analysis.py`'s `f_px_at_fixed_hfov` (same
    formula, this module's own copy so `ground_station/` has zero import
    dependency on the analysis script)."""
    return (width_px / 2.0) / math.tan(hfov_rad / 2.0)


def camera_axes(yaw_rad: float):
    """(forward, right, down) unit vectors in world XYZ, OpenCV convention,
    for a rig with only yaw (no pitch/roll). See module docstring."""
    forward = (math.cos(yaw_rad), math.sin(yaw_rad), 0.0)
    right = (math.sin(yaw_rad), -math.cos(yaw_rad), 0.0)
    down = (0.0, 0.0, -1.0)
    return forward, right, down


def camera_positions(rig_pose: Pose4, baseline_m: float):
    """World positions of (rig_left, rig_right), matching model.sdf's
    link-local sensor offsets (0, +1, 0) / (0, -1, 0) rotated by yaw."""
    _, right, _ = camera_axes(rig_pose.yaw)
    half = baseline_m / 2.0
    left = (
        rig_pose.x - right[0] * half,
        rig_pose.y - right[1] * half,
        rig_pose.z - right[2] * half,
    )
    right_cam = (
        rig_pose.x + right[0] * half,
        rig_pose.y + right[1] * half,
        rig_pose.z + right[2] * half,
    )
    return left, right_cam


def project_point(p_world, cam_pos, yaw_rad: float, f_px: float, cx: float, cy: float):
    """Pinhole-project a world point into ONE camera. Returns (u, v, Z) or
    None if the point is behind the camera (Z <= 0 -- not physically
    imageable)."""
    forward, right, down = camera_axes(yaw_rad)
    dx = p_world[0] - cam_pos[0]
    dy = p_world[1] - cam_pos[1]
    dz = p_world[2] - cam_pos[2]
    Z = dx * forward[0] + dy * forward[1] + dz * forward[2]
    if Z <= 1e-6:
        return None
    X = dx * right[0] + dy * right[1] + dz * right[2]
    Y = dx * down[0] + dy * down[1] + dz * down[2]
    u = cx + f_px * X / Z
    v = cy + f_px * Y / Z
    return u, v, Z


def project_world_to_stereo_pixels(p_world, rig_pose: Pose4, baseline_m: float,
                                    f_px: float, cx: float, cy: float):
    """The exact inverse of `triangulate()`: world point -> (u_l, v_l, u_r,
    v_r). Used by `t18_sigma_validation.py`'s perfect-centroid floor mode to
    synthesize centroids from ground truth (no NN, no rendered image)."""
    left_pos, right_pos = camera_positions(rig_pose, baseline_m)
    pl = project_point(p_world, left_pos, rig_pose.yaw, f_px, cx, cy)
    pr = project_point(p_world, right_pos, rig_pose.yaw, f_px, cx, cy)
    if pl is None or pr is None:
        return None
    u_l, v_l, _ = pl
    u_r, v_r, _ = pr
    return u_l, v_l, u_r, v_r


class TriangulationResult(NamedTuple):
    x: float
    y: float
    z: float
    range_m: float          # rig-center-to-point range along the boresight-ish 3D distance
    disparity_px: float
    Z_forward_m: float      # depth along the assumed rig's boresight


def triangulate(u_l: float, v_l: float, u_r: float, v_r: float,
                 assumed_rig_pose: Pose4, baseline_m: float,
                 f_px: float, cx: float, cy: float) -> Optional[TriangulationResult]:
    """Pixel centroids (both cameras) -> disparity -> range -> 3D WORLD
    position, inverting `project_world_to_stereo_pixels()` under the
    ASSUMED rig pose (see module docstring's datum-bias section). Returns
    None for a degenerate/behind-camera observation (disparity <= 0)."""
    disparity = u_l - u_r
    if disparity <= 1e-6:
        return None
    Z = baseline_m * f_px / disparity
    u_avg = 0.5 * (u_l + u_r)
    v_avg = 0.5 * (v_l + v_r)
    X = (u_avg - cx) * Z / f_px
    Y = (v_avg - cy) * Z / f_px
    forward, right, down = camera_axes(assumed_rig_pose.yaw)
    wx = assumed_rig_pose.x + Z * forward[0] + X * right[0] + Y * down[0]
    wy = assumed_rig_pose.y + Z * forward[1] + X * right[1] + Y * down[1]
    wz = assumed_rig_pose.z + Z * forward[2] + X * right[2] + Y * down[2]
    range_m = math.sqrt(
        (wx - assumed_rig_pose.x) ** 2
        + (wy - assumed_rig_pose.y) ** 2
        + (wz - assumed_rig_pose.z) ** 2
    )
    return TriangulationResult(wx, wy, wz, range_m, disparity, Z)


def apply_datum_bias(true_rig_pose: Pose4, bias_m: float) -> Pose4:
    """`--datum-bias-m B` shorthand: shift the ASSUMED rig position by B
    metres along the rig's own boresight (`forward`), pushing the assumed
    rig center FARTHER from the target than it truly is. Yaw is left
    UNCHANGED (a pure position/datum error, not a pointing error -- see
    module docstring for why direction doesn't change the output shift
    MAGNITUDE, only which axis narrates the story; range is this rig's
    disclosed weakest axis, so it is the chosen default)."""
    forward, _, _ = camera_axes(true_rig_pose.yaw)
    return Pose4(
        true_rig_pose.x - forward[0] * bias_m,
        true_rig_pose.y - forward[1] * bias_m,
        true_rig_pose.z - forward[2] * bias_m,
        true_rig_pose.yaw,
    )


# ============================================================================
# Rig config (loaded from a T16 capture_meta.json, or CLI overrides)
# ============================================================================
class RigConfig(NamedTuple):
    true_pose: Pose4
    baseline_m: float
    hfov_rad: float
    width_px: int
    height_px: int

    @property
    def f_px(self) -> float:
        return f_px_from_hfov(self.width_px, self.hfov_rad)

    @property
    def cx(self) -> float:
        return self.width_px / 2.0

    @property
    def cy(self) -> float:
        return self.height_px / 2.0

    @classmethod
    def from_capture_meta(cls, meta_path: str) -> "RigConfig":
        with open(meta_path) as f:
            meta = json.load(f)
        x, y, z, yaw = meta["rig_pose_xyz_yaw"]
        w, h = meta["resolution"]
        return cls(
            true_pose=Pose4(x, y, z, yaw),
            baseline_m=meta["rig_baseline_m"],
            hfov_rad=meta["rig_hfov_rad"],
            width_px=w,
            height_px=h,
        )


def parse_pose4(s: str) -> Pose4:
    parts = [float(p) for p in s.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            f"expected 'x,y,z,yaw' (4 comma-separated floats), got {s!r}"
        )
    return Pose4(*parts)


# ============================================================================
# Cue-payload formatting -- matches scripts/s2_cue_mock.py's emit code
# EXACTLY (same key set / key order / optional-velocity-keys convention),
# so a future ground_station.py (T19) can reuse this unchanged.
# ============================================================================
def build_cue_payload(seq: int, t_sim: float, x: float, y: float, z: float,
                       vx: Optional[float] = None, vy: Optional[float] = None,
                       vz: Optional[float] = None) -> dict:
    """Mirrors scripts/s2_cue_mock.py:720-729 exactly: `{"seq", "t_sim", "x",
    "y", "z"}` always; `"vx"/"vy"/"vz"` appended ONLY when velocity is
    available (so an old consumer that only knows the 5-key schema still
    parses a payload built here)."""
    payload = {"seq": seq, "t_sim": t_sim, "x": x, "y": y, "z": z}
    if vx is not None and vy is not None and vz is not None:
        payload["vx"] = vx
        payload["vy"] = vy
        payload["vz"] = vz
    return payload


# ============================================================================
# CLI -- manual/offline triangulation of one centroid pair (also used by
# scripts to sanity-check the geometry without spinning up a batch harness).
# ============================================================================
def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--rig-meta", required=True,
                     help="path to a T16 capture_meta.json (rig baseline/HFOV/"
                          "resolution/TRUE pose)")
    ap.add_argument("--centroids", required=True,
                     help="'u_l,v_l,u_r,v_r' pixel centroids")
    ap.add_argument("--assumed-rig-pose", type=parse_pose4, default=None,
                     help="'x,y,z,yaw' -- assumed rig pose used for "
                          "triangulation (default: the TRUE pose from --rig-meta, "
                          "i.e. zero extrinsics error)")
    ap.add_argument("--datum-bias-m", type=float, default=None,
                     help="shorthand: shift the assumed pose by this many "
                          "metres along the rig's boresight (mutually "
                          "exclusive with --assumed-rig-pose)")
    ap.add_argument("--seq", type=int, default=0)
    ap.add_argument("--t-sim", type=float, default=0.0)
    args = ap.parse_args()

    if args.assumed_rig_pose is not None and args.datum_bias_m is not None:
        ap.error("--assumed-rig-pose and --datum-bias-m are mutually exclusive")

    rig = RigConfig.from_capture_meta(args.rig_meta)
    if args.assumed_rig_pose is not None:
        assumed_pose = args.assumed_rig_pose
    elif args.datum_bias_m is not None:
        assumed_pose = apply_datum_bias(rig.true_pose, args.datum_bias_m)
    else:
        assumed_pose = rig.true_pose

    u_l, v_l, u_r, v_r = (float(p) for p in args.centroids.split(","))
    result = triangulate(u_l, v_l, u_r, v_r, assumed_pose, rig.baseline_m,
                          rig.f_px, rig.cx, rig.cy)
    if result is None:
        print(json.dumps({"error": "degenerate observation (disparity <= 0)"}))
        return 1

    payload = build_cue_payload(args.seq, args.t_sim, result.x, result.y, result.z)
    payload["_debug"] = {
        "disparity_px": result.disparity_px,
        "range_m": result.range_m,
        "assumed_rig_pose": list(assumed_pose),
        "true_rig_pose": list(rig.true_pose),
        "f_px": rig.f_px,
    }
    print(json.dumps(payload))
    return 0


if __name__ == "__main__":
    sys.exit(main())
