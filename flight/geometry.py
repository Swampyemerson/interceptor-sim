"""flight.geometry -- pure frame math for the camera-only terminal seeker.

Camera(optical) -> body(FRD) -> NED line-of-sight derotation, using the vehicle's
OWN-state EKF attitude (no ground truth). Extracted verbatim from
scripts/m4_intercept.py (audit-3 H1 fix + #40 mount compose); the underscore
aliases keep the exact names m4 imports so wiring is a drop-in. Pinned by
flight/tests/test_geometry.py (mirrors bench_derotation_selfcheck).

Honesty boundary (ADR-0008/0010): `meas_xyz`/`bearing_rad` are pure CAMERA
measurements; `quat`/`psi_rad` are the vehicle's own attitude EKF -- the same
own-state basis as position. Nothing here reads gt_*.
"""

import collections
import math
import xml.etree.ElementTree as ET


def wrap_pi(angle):
    """Wrap an angle (radians) into [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quat_rotate(quat, vec):
    """Rotate 3-vector `vec` by unit quaternion `quat` = (w, x, y, z), i.e.
    apply the rotation q (v' = q * v * q_conj). Pure math, no numpy dependency.
    MAVSDK's attitude_quaternion() is the body(FRD)->NED rotation, so feeding a
    body-frame vector returns it in NED."""
    w, x, y, z = quat
    vx, vy, vz = vec
    # t = 2 * (q_xyz x v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    # v' = v + w*t + (q_xyz x t)
    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return rx, ry, rz


def euler_to_quat_body_to_ned(roll, pitch, yaw):
    """(roll, pitch, yaw) radians -> body(FRD)->NED unit quaternion (w,x,y,z),
    standard aerospace ZYX (yaw about down, then pitch about right, then roll
    about forward). Builds known-attitude fixtures for the derotation self-check."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,   # w
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
    )


def derotate_bearing_lambda(meas_xyz, bearing_rad, quat, psi_rad,
                            mount_up_rad=0.0):
    """Inertial LOS azimuth lambda from a camera measurement, derotating the
    FULL vehicle attitude (roll + pitch + yaw), not just yaw (audit-3 H1 fix),
    and composing a FIXED camera-mount up-tilt when one is fitted (#40).

    THE BUG THIS FIXES: the pre-fix `lambda = psi + beta` compensates YAW ONLY.
    beta (bearing_rad) is the target's HORIZONTAL azimuth in the camera OPTICAL
    frame; adding vehicle yaw psi gives the inertial azimuth ONLY when the camera
    boresight is level. At a 27-36 deg dash pitch that inflates the azimuth
    ~1/cos(pitch) and, under roll, mixes the target's VERTICAL off-boresight
    component into the azimuth -- corrupting lambda/lambda_dot, the pro-nav input.

    THE FIX: rotate the camera ray optical->body(FRD)->NED with the full attitude
    quaternion, then lambda = atan2(east, north). meas_xyz (optical-frame target
    vector: x right, y down, z forward) carries the elevation the bearing throws
    away, so the roll cross-coupling is captured. Only a UNIT direction matters.

    IDENTITY AT ZERO ROLL/PITCH: with the quaternion a pure-yaw rotation, the
    chain reduces to lambda = psi + beta EXACTLY -- the validated level results
    are preserved to the bit; only tilt introduces a (correct) change.

    FALLBACK: if the quaternion / bearing is not yet available, return the pre-fix
    yaw-only lambda = psi + beta so nothing regresses during stream warmup. (The
    mount is NOT composed in the fallback: with no attitude there is no frame
    chain to compose into.)"""
    if quat is None or bearing_rad is None:
        return psi_rad + (bearing_rad if bearing_rad is not None else 0.0)
    # Camera ray in the OPTICAL frame (x right, y down, z forward). Prefer the
    # full 3D vector (carries elevation -> the roll cross-coupling term); fall
    # back to the bearing at zero elevation if it's missing.
    if meas_xyz is not None:
        rx, ry, rz = float(meas_xyz[0]), float(meas_xyz[1]), float(meas_xyz[2])
    else:
        rx, ry, rz = math.sin(bearing_rad), 0.0, math.cos(bearing_rad)
    # optical -> body FRD: forward(+x_b)=+z_opt, right(+y_b)=+x_opt, down(+z_b)=+y_opt
    body = (rz, rx, ry)
    # MOUNT COMPOSE (#40, ADR-0067): a camera mounted tilted UP by mount_up_rad
    # rotates the boresight about body +y (right): R_y(t)@(1,0,0) = (cos t,0,-sin t).
    # Guarded so the 0.0 default keeps the EXACT pre-compose float path.
    if mount_up_rad:
        ct, st = math.cos(mount_up_rad), math.sin(mount_up_rad)
        bx, by, bz = body
        body = (ct * bx + st * bz, by, -st * bx + ct * bz)
    n_e_d = quat_rotate(quat, body)
    return math.atan2(n_e_d[1], n_e_d[0])


def camera_to_cg_los(range_m, lambda_rad, quat, cam_offset_body):
    """Re-anchor a camera range/LOS from the CAMERA to the vehicle CG/IMU
    (base_link) -- the lever-arm a forward/elevated camera mount injects.

    WHY (ADR-0076 add #18k / Fable gap-check 2026-07-17): pro-nav commands the
    vehicle about its CG, so it wants the CG->target LOS, but the camera measures
    from the CAMERA. Anchoring `own_ned + range*LOS` at base_link while the ray
    started 0.1-0.4 m ahead biases the terminal azimuth by several degrees at a
    few-metre range -- largest exactly when it matters (contact). The m4 SIM
    harness compensated inline (m4_intercept.py ~2493); this is the SAME math in
    the PORTABLE core so the real Pi/Pixhawk terminal is honest too. The forward
    mount (prop-clearance) makes this load-bearing, not optional.

    cam_offset_body = (fwd, left, up) metres: the camera position relative to the
    CG in the vehicle BODY frame. Rotated to NED by the FULL attitude quaternion
    (the SAME quat + quat_rotate the LOS derotation uses -> correct through the
    dash PITCH and any mount up-tilt; a yaw-only version ignored both and scored
    0/16, add #16). Only the horizontal (N,E) offset moves the azimuth.

    Returns (range_cg, lambda_cg). Zero offset (or missing range/quat) -> the
    inputs unchanged, so a CG-mounted camera is byte-identical. Own attitude (EKF)
    + static mount constants, no gt_*.
    """
    fwd, left, up = cam_offset_body
    if (not (fwd or left or up)) or range_m is None or quat is None:
        return range_m, lambda_rad
    off_n, off_e, _off_d = quat_rotate(quat, (fwd, -left, -up))
    tn = range_m * math.cos(lambda_rad) + off_n
    te = range_m * math.sin(lambda_rad) + off_e
    return math.hypot(tn, te), math.atan2(te, tn)


# --- P0.4 CAMERA LEVER-ARM GUARD: tie --cam-fwd-offset-m to the active airframe --
#     the moved-camera model (props-clearance, cam_fwd 0.12 -> 0.40) and the
#     guidance compensation (camera_to_cg_los / --cam-fwd-offset-m) must agree, or
#     a run is a SILENT uncompensated parallax (several deg of terminal LOS error) --
#     exactly the mirage class ADR-0076 exists to stop. PX4 spawns the airframe from
#     a HARDCODED file://.../x500_mono_cam/model.sdf (bypasses GZ_SIM_RESOURCE_PATH,
#     see scripts/adaptive_tilt_arm.sh), so whatever variant a swap script copied
#     into that file IS what flies -- reading its CameraJoint forward offset is the
#     ground truth of the active geometry, with no marker file to drift. These are
#     PURE (no filesystem): parse SDF text, then compare -- unit-tested in
#     flight/tests/test_geometry.py. ---

STOCK_CAM_FWD_M = 0.12          # stock x500_mono_cam camera forward-x (base_link +x)
CAM_OFFSET_MATCH_TOL_M = 0.05   # |applied - active| within this = compensated

CamOffsetCheck = collections.namedtuple(
    "CamOffsetCheck", ["ok", "mismatch", "variant", "detail"])


def parse_cam_fwd_from_sdf(sdf):
    """Forward (base_link +x) offset of the seeker camera, in metres, parsed from
    an x500_mono_cam `model.sdf`. Returns None if it can't be found/parsed.

    Prefers the authoritative `CameraJoint` `<pose relative_to="base_link">`
    (which places camera_link); falls back to the `mono_cam` `<include>` pose
    (same forward x in every variant here). Pure text/bytes in, float out -- no
    filesystem, so it is unit-testable. `sdf` may be str or bytes; a str carrying
    an `encoding=` XML declaration is encoded to bytes first (ElementTree rejects
    a str with an encoding declaration)."""
    try:
        data = sdf.encode("utf-8") if isinstance(sdf, str) else sdf
        root = ET.fromstring(data)
    except (ET.ParseError, ValueError, AttributeError):
        return None

    def _first_float(pose_elem):
        if pose_elem is not None and pose_elem.text:
            try:
                return float(pose_elem.text.split()[0])
            except (ValueError, IndexError):
                return None
        return None

    for joint in root.iter("joint"):
        if joint.get("name") == "CameraJoint":
            val = _first_float(joint.find("pose"))
            if val is not None:
                return val
    for inc in root.iter("include"):
        uri = inc.find("uri")
        if uri is not None and uri.text and "mono_cam" in uri.text:
            val = _first_float(inc.find("pose"))
            if val is not None:
                return val
    return None


def check_cam_offset_consistency(sdf_fwd_m, applied_fwd_m, allow_mismatch=False,
                                 stock_fwd_m=STOCK_CAM_FWD_M,
                                 match_tol_m=CAM_OFFSET_MATCH_TOL_M):
    """Is the applied `--cam-fwd-offset-m` consistent with the ACTIVE camera
    forward offset read from the airframe SDF? Returns CamOffsetCheck(ok,
    mismatch, variant, detail).

    Consistent (mismatch=False) when EITHER:
      * the applied offset matches the active camera forward-x within
        `match_tol_m` (properly compensated -- e.g. moved 0.40 + flag 0.40), OR
      * the active camera is the stock mount (~`stock_fwd_m`) AND the applied
        offset is ~0 (the sanctioned negligible-parallax default -- stock 0.12
        + flag 0.0, which is byte-identical to every prior run).

    The two DANGEROUS classes are caught: a moved camera flown with offset 0
    (uncompensated parallax) and a stock camera flown with a large offset
    (over-compensation). `allow_mismatch=True` forces ok=True (deliberate A/B)
    but still reports mismatch=True so the override is visible. Forward axis only
    -- the axis the model swap changes; left/up don't move with the swap."""
    is_stock = abs(sdf_fwd_m - stock_fwd_m) <= match_tol_m
    variant = "stock" if is_stock else ("moved(cam_fwd=%.3fm)" % sdf_fwd_m)
    compensated = abs(applied_fwd_m - sdf_fwd_m) <= match_tol_m
    stock_default = is_stock and abs(applied_fwd_m) <= match_tol_m
    mismatch = not (compensated or stock_default)
    detail = ""
    if mismatch:
        need = 0.0 if is_stock else sdf_fwd_m
        detail = (
            "active camera cam_fwd=%.3f m (%s) but --cam-fwd-offset-m=%.3f m -- "
            "uncompensated parallax. This geometry needs --cam-fwd-offset-m "
            "~%.3f m (within %.2f m); fly the matching model, fix the flag, or "
            "pass --allow-cam-offset-mismatch for a deliberate A/B."
            % (sdf_fwd_m, variant, applied_fwd_m, need, match_tol_m))
    return CamOffsetCheck(
        ok=(not mismatch) or allow_mismatch, mismatch=mismatch,
        variant=variant, detail=detail)


# --- Legacy aliases: the exact private names scripts/m4_intercept.py used, so it
#     can `from flight.geometry import _quat_rotate, _euler_to_quat_body_to_ned`
#     as a drop-in replacement for its local copies. ---
_quat_rotate = quat_rotate
_euler_to_quat_body_to_ned = euler_to_quat_body_to_ned
