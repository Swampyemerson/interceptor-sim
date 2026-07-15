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

import math


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


# --- Legacy aliases: the exact private names scripts/m4_intercept.py used, so it
#     can `from flight.geometry import _quat_rotate, _euler_to_quat_body_to_ned`
#     as a drop-in replacement for its local copies. ---
_quat_rotate = quat_rotate
_euler_to_quat_body_to_ned = euler_to_quat_body_to_ned
