"""AprilTag seeker: a geometric + statistical model of "did this frame decode?".

There is no rendering here. Each camera frame is reduced to four numbers --
apparent tag side in pixels, incidence angle, motion blur in pixels, and whether
all four corners are inside the image -- and those drive a decode probability.
A decoded frame yields a noisy pixel measurement, which is then converted to
bearing / elevation / range exactly the way the Pi's tag pose solver would.

The seeker is a SENSOR MODEL, so it is allowed to read true states. Guidance is
not: it only ever sees the Detection this module emits, one latency late.

Sources for the numbers, per the honesty rule:
  MEASURED (bench, Raspberry Pi 5 + OV9281, 2026-09): sensor 1280x800,
    capture+decode pipeline 38.2 fps, outdoor exposure 994 us.
  UNMEASURED (see the UNMEASURED tuple): the lens is not calibrated, the
    pipeline latency is not instrumented, and the whole decode surface is a
    literature-plausible shape, not a fit to bench data.

Frames: world NED, body FRD, camera OpenCV (z forward, x right, y down).
Quaternions are (w, x, y, z), body -> NED.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from isim.types import Detection, FrameReport, TargetState, VehicleState

# Every field below that is NOT traceable to a bench measurement. The project
# grades inputs `measured` / `given-noisy` / `given-perfect` / `unmeasured`;
# these are all `unmeasured` and any number computed on them is a model output,
# not a claim about the real camera.
UNMEASURED = (
    "fx", "fy", "cx", "cy",                       # lens never calibrated
    "mount_tilt_up_deg", "mount_xyz_body",        # mount geometry not built
    "latency_s", "latency_jitter_s",              # pipeline not instrumented
    "side_m", "normal_ned", "faces_velocity", "faces_camera",
    "p_max", "size50_px", "size_k_px", "min_side_px",
    "inc50_deg", "inc_k_deg",
    "cells_across", "blur50_cells", "blur_n",
    "pixel_noise_px", "side_noise_factor",
    # tag_realism_v1 (isim/specs/tag_realism_v1.md) -- all `estimate`:
    "body_normal_frd",                            # tag mount on the target body
    "tgt_shake_rms_deg", "tgt_shake_bw_hz",       # target attitude wobble (B1)
    "vib_rate_rms_dps",                           # own-camera vibration (B2)
)
# tag_realism_v1 C: GlareParams is a separate dataclass, so its (all
# `estimate`) fields get a parallel tuple rather than widening UNMEASURED's
# CameraParams/TagParams/DecodeParams contract.
GLARE_UNMEASURED = (
    "sun_azimuth_deg", "sun_elevation_deg",
    "specular_strength", "specular_width_deg",
    "backlight_kill_deg", "backlight_strength",
)
MEASURED = ("width", "height", "fps", "exposure_s")

_EPS = 1e-12
# Sim-clock slop for the "is this frame due?" comparison. A step time built as
# i*dt lands a few ULP below an exactly-equal t_available, which would defer the
# release by a whole step; 1 ns of sim time is far below anything we model.
_T_EPS = 1e-9
# v_body = _B @ v_cam : camera z(fwd)->body x, camera x(right)->body y,
# camera y(down)->body z.
_B = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


@dataclass
class CameraParams:
    """OV9281 global shutter on a Pi 5. width/height/fps/exposure are measured;
    the intrinsics and the mount are placeholders (see UNMEASURED)."""
    width: int = 1280
    height: int = 800
    fx: float = 540.0
    fy: float = 540.0
    cx: float = 640.0
    cy: float = 400.0
    mount_tilt_up_deg: float = 0.0          # camera pitched UP from body x
    mount_xyz_body: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # FRD, metres
    fps: float = 38.2                       # measured: capture + decode
    exposure_s: float = 994e-6              # measured: outdoors
    latency_s: float = 0.045                # capture -> available to guidance
    latency_jitter_s: float = 0.005         # 1-sigma, guess
    # tag_realism_v1 B2: prop-induced angular vibration of the interceptor,
    # 1-sigma body rate in deg/s. Global shutter + ~1 ms exposure, so it is a
    # small extra smear (fx * rate * exposure), not jello. One |Gaussian| draw
    # per frame when > 0; 0.0 (default) = off, no draw. estimate (bench-
    # measurable later: motors-on static frame-to-frame corner jitter).
    vib_rate_rms_dps: float = 0.0

    def rot_body_from_cam(self) -> np.ndarray:
        t = math.radians(self.mount_tilt_up_deg)
        c, s = math.cos(t), math.sin(t)
        # Positive rotation about body y (right) raises the nose in FRD/NED,
        # so Ry(+tilt) is "camera pitched up".
        ry = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
        return ry @ _B


@dataclass
class TagParams:
    """The tag36h11 marker carried by the target. `side_m` is the printed side
    (0.30 m is the planned print, not yet cut). Orientation is one of three
    modes; incidence is the angle between the tag normal and the tag->camera
    line, so incidence > 90 deg means we are looking at the BACK of the tag."""
    side_m: float = 0.30
    normal_ned: Tuple[float, float, float] = (-1.0, 0.0, 0.0)  # faces south
    faces_velocity: bool = False    # tag on the nose, normal = unit(velocity)
    faces_camera: bool = False      # best case: incidence pinned to 0
    # tag_realism_v1 A3: tag normal in the TARGET's body FRD. Used only when
    # the TargetState carries an attitude (quat_wxyz); then the normal AND
    # the in-plane corner basis are body-fixed (a banked target visibly
    # rotates the square). No quat -> the legacy modes above, unchanged.
    # `faces_camera` still wins (the idealized best case ignores attitude).
    body_normal_frd: Optional[Tuple[float, float, float]] = None

    def body_mounted(self, tgt: TargetState) -> bool:
        """True when this frame's tag orientation comes from the target body."""
        return (not self.faces_camera and self.body_normal_frd is not None
                and getattr(tgt, "quat_wxyz", None) is not None)

    def frame(self, tgt: TargetState, cam_pos_ned: np.ndarray
              ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """(unit normal, in-plane e1 or None). e1 is returned only for the
        body-mounted mode; None means "use the legacy world-level basis"
        (`tag_corners_ned` builds it), so the legacy path is untouched."""
        if not self.body_mounted(tgt):
            return self.normal(tgt, cam_pos_ned), None
        r_tn = quat_to_rot(tgt.quat_wxyz)
        nb = np.asarray(self.body_normal_frd, float)
        nb = nb / max(float(np.linalg.norm(nb)), _EPS)
        # e1 chosen in the BODY frame exactly as tag_corners_ned chooses it in
        # the world frame (horizontal-at-level), then rotated with the body.
        e1b = _level_e1(nb)
        n = r_tn @ nb
        e1 = r_tn @ e1b
        return (n / max(float(np.linalg.norm(n)), _EPS),
                e1 / max(float(np.linalg.norm(e1)), _EPS))

    def normal(self, tgt: TargetState, cam_pos_ned: np.ndarray) -> np.ndarray:
        if self.body_mounted(tgt):
            return self.frame(tgt, cam_pos_ned)[0]
        if self.faces_camera:
            v = np.asarray(cam_pos_ned, float) - np.asarray(tgt.pos_ned, float)
        elif self.faces_velocity:
            v = np.asarray(tgt.vel_ned, float)
            if float(np.linalg.norm(v)) < 1e-3:
                v = np.asarray(self.normal_ned, float)
        else:
            v = np.asarray(self.normal_ned, float)
        n = float(np.linalg.norm(v))
        return np.asarray(self.normal_ned, float) if n < _EPS else v / n


@dataclass
class DecodeParams:
    """p = p_max * p_size(side*cos i) * p_incidence(i) * p_blur(blur).

    p_size: logistic in apparent side. 50% at 22 px, ~95% by 35 px -- tag36h11
      is 8 cells across and the detector needs roughly 3-4 px per cell
      (literature: AprilTag 2/3 papers; NOT bench-fit).
    p_incidence: extra angular term on top of the cos() foreshortening, flat to
      ~45 deg and ~dead by 75-80 deg (the quad's corner geometry degenerates and
      the printed border self-occludes). Guess.
    p_blur: Hill function in blur measured in CELL widths -- 50% at one cell of
      smear, which is the standard "blur below one sample" rule of thumb.
    """
    p_max: float = 0.98
    size50_px: float = 22.0
    size_k_px: float = 4.4          # logistic scale -> ~95% at 35 px
    min_side_px: float = 8.0        # < 1 px per cell: decode impossible
    inc50_deg: float = 60.0
    inc_k_deg: float = 5.0
    cells_across: float = 8.0       # tag36h11: 6 data + 1 border each side
    blur50_cells: float = 1.0
    blur_n: float = 3.0
    pixel_noise_px: float = 0.3     # 1-sigma on the decoded centre
    side_noise_factor: float = 1.414  # side = difference of two noisy corners
    # DETECTION-CADENCE THINNING (2026-09-23, tick-trace S3 -- isim/specs/
    # xcheck_tick_trace_2026-09-23.md): a real driver may decode far fewer
    # frames than the camera produces (the Gazebo cross-check driver consumed
    # 5.2 det/s in ENGAGE against a 30 fps camera). When > 0, a frame whose
    # capture time is within this interval of the last DECODED frame is not
    # attempted at all (p forced to 0 -- the pipeline never polled it), so
    # the delivered detection rate is capped at ~1/interval. 0.0 (default) =
    # no gate, byte-identical to before this field existed (guarded so no
    # rng draw or state changes on the default path).
    min_decode_interval_s: float = 0.0
    # TARGET ATTITUDE WOBBLE (tag_realism_v1 B1): a hovering/translating quad
    # wobbles 1-3 deg at a few Hz (gusts + control dither). TRUE tag motion,
    # modelled here (not in the pure target) as a 2-axis Ornstein-Uhlenbeck
    # process sampled at frame times; the angles tilt the tag about its two
    # in-plane axes before incidence/corners, and the process increment/dt
    # adds to the rotational blur. 0.0 (default) = off, no rng draw.
    # `faces_camera` tags ignore it (the idealized best case). estimate.
    tgt_shake_rms_deg: float = 0.0     # 1-sigma wobble ANGLE per axis
    tgt_shake_bw_hz: float = 3.0       # bandwidth: OU tau = 1/(2*pi*bw)


@dataclass
class GlareParams:
    """Sun geometry (tag_realism_v1 C): glare that switches on exactly when
    the approach lines up with the sun. Two decode-probability multipliers,
    both deterministic (no rng):
      specular:  reflect the sunlight about the tag plane; theta = angle
                 between that ray and the tag->camera line;
                 p *= 1 - specular_strength * exp(-(theta/specular_width_deg)^2).
                 No glare when the sun is behind the tag plane.
      backlight: phi = angle(camera boresight, sun); inside backlight_kill_deg
                 p *= 1 - backlight_strength * (1 - phi/backlight_kill_deg).
    specular_strength = 0 and backlight_kill_deg = 0 (defaults) = off. All
    `estimate` (matte print ~0.4, glossy/laminated ~0.9 -- not bench-fit)."""
    sun_azimuth_deg: float = 180.0      # NED azimuth of the sun, 0 = north, 90 = east
    sun_elevation_deg: float = 35.0     # above the horizon
    specular_strength: float = 0.0      # 0 = off
    specular_width_deg: float = 12.0    # half-width of the glare lobe
    backlight_kill_deg: float = 0.0     # 0 = off; cone half-angle around the sun
    backlight_strength: float = 0.9     # multiplier depth on the sun axis

    def active(self) -> bool:
        return self.specular_strength > 0.0 or self.backlight_kill_deg > 0.0

    def sun_ned(self) -> np.ndarray:
        """Unit vector FROM the scene TOWARD the sun (NED: up is -z)."""
        az, el = math.radians(self.sun_azimuth_deg), math.radians(self.sun_elevation_deg)
        return np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az),
                         -math.sin(el)])


# ---------------------------------------------------------------- pure helpers

def quat_to_rot(q: Tuple[float, float, float, float]) -> np.ndarray:
    """(w,x,y,z) body->NED to a 3x3 rotation matrix (columns = body axes)."""
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < _EPS:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def camera_origin_ned(own_pos: np.ndarray, r_bn: np.ndarray,
                      cam: CameraParams) -> np.ndarray:
    """Lens centre in NED = vehicle origin + body-frame mount offset."""
    return np.asarray(own_pos, float) + r_bn @ np.asarray(cam.mount_xyz_body, float)


def world_to_camera(p_ned, own_pos, q_body_to_ned, cam: CameraParams) -> np.ndarray:
    """A NED point expressed in the camera frame (OpenCV: z fwd, x right, y down)."""
    r_bn = quat_to_rot(q_body_to_ned)
    d = np.asarray(p_ned, float) - camera_origin_ned(own_pos, r_bn, cam)
    return (r_bn @ cam.rot_body_from_cam()).T @ d


def project(p_cam: np.ndarray, cam: CameraParams) -> Tuple[float, float, bool]:
    """Pinhole projection. Returns (u, v, in_front). No distortion model: the
    lens is uncalibrated, so pretending to know k1/k2 would be fiction."""
    z = float(p_cam[2])
    if z <= _EPS:
        return float("nan"), float("nan"), False
    return cam.cx + cam.fx * float(p_cam[0]) / z, cam.cy + cam.fy * float(p_cam[1]) / z, True


def _level_e1(n: np.ndarray) -> np.ndarray:
    """In-plane axis e1 of a tag with unit normal `n`: horizontal (perpendicular
    to `n` and to "down" of whatever frame `n` is in), north/forward fallback
    when the tag lies flat. Shared by the world-level (legacy) and the
    body-fixed (tag_realism_v1 A3) bases."""
    up = np.array([0.0, 0.0, -1.0])
    e1 = np.cross(up, n)
    if float(np.linalg.norm(e1)) < 1e-6:          # tag lies flat: use north
        e1 = np.cross(np.array([1.0, 0.0, 0.0]), n)
    return e1 / max(float(np.linalg.norm(e1)), _EPS)


def tag_corners_ned(tgt_pos, normal, side: float, e1=None) -> np.ndarray:
    """The 4 corners of a planar square tag centred on the target, in NED.
    In-plane axes: e1 horizontal (perpendicular to the normal and to world
    down), e2 = normal x e1 completes the right-handed set. An explicit `e1`
    (tag_realism_v1: body-fixed or wobbled basis, already perpendicular to
    the normal) overrides the world-level choice."""
    c = np.asarray(tgt_pos, float)
    n = np.asarray(normal, float)
    n = n / max(float(np.linalg.norm(n)), _EPS)
    if e1 is None:
        e1 = _level_e1(n)
    else:
        e1 = np.asarray(e1, float)
        e1 = e1 / max(float(np.linalg.norm(e1)), _EPS)
    e2 = np.cross(n, e1)
    h = 0.5 * side
    return np.array([c + h * e1 + h * e2, c + h * e1 - h * e2,
                     c - h * e1 - h * e2, c - h * e1 + h * e2])


def p_decode(side_px: float, incidence_deg: float, blur_px: float,
             dp: DecodeParams) -> float:
    """Decode probability for one frame. Monotone: increasing in side_px,
    decreasing in incidence and in blur. `side_px` is the FRONTAL apparent side;
    the cos(incidence) foreshortening is applied here."""
    if incidence_deg >= 90.0:
        return 0.0                     # looking at the back of the tag
    eff = max(side_px, 0.0) * math.cos(math.radians(incidence_deg))
    if eff < dp.min_side_px:
        return 0.0
    p_size = 1.0 / (1.0 + math.exp(-(eff - dp.size50_px) / dp.size_k_px))
    p_inc = 1.0 / (1.0 + math.exp((incidence_deg - dp.inc50_deg) / dp.inc_k_deg))
    cells = max(eff / dp.cells_across, _EPS)
    ratio = max(blur_px, 0.0) / (cells * dp.blur50_cells)
    p_blur = 1.0 / (1.0 + ratio ** dp.blur_n)
    return dp.p_max * p_size * p_inc * p_blur


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    c = float(np.dot(a, b)) / max(na * nb, _EPS)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def glare_multipliers(gp: GlareParams, normal: np.ndarray, to_cam: np.ndarray,
                      boresight_ned: np.ndarray) -> Tuple[float, float]:
    """(specular, backlight) decode-probability multipliers for one frame --
    see GlareParams. `normal` is the tag's unit normal, `to_cam` the tag ->
    camera vector, `boresight_ned` the camera +z axis in NED. Pure; each
    multiplier is exactly 1.0 when its effect is off."""
    s_ned = gp.sun_ned()
    spec = 1.0
    if gp.specular_strength > 0.0:
        n = np.asarray(normal, float)
        if float(np.dot(s_ned, n)) > 0.0:          # sun in front of the tag plane
            d = -s_ned                              # sunlight travel direction
            r = d - 2.0 * float(np.dot(d, n)) * n   # mirror reflection
            theta = _angle_deg(r, np.asarray(to_cam, float))
            w = max(gp.specular_width_deg, _EPS)
            spec = 1.0 - gp.specular_strength * math.exp(-(theta / w) ** 2)
    back = 1.0
    if gp.backlight_kill_deg > 0.0:
        phi = _angle_deg(np.asarray(boresight_ned, float), s_ned)
        if phi < gp.backlight_kill_deg:
            back = 1.0 - gp.backlight_strength * (1.0 - phi / gp.backlight_kill_deg)
    return max(0.0, spec), max(0.0, back)


def _rotate(v: np.ndarray, rotvec: np.ndarray) -> np.ndarray:
    """Rodrigues: rotate `v` by the rotation vector `rotvec` (axis*angle)."""
    ang = float(np.linalg.norm(rotvec))
    if ang < _EPS:
        return np.asarray(v, float)
    k = rotvec / ang
    c, s_ = math.cos(ang), math.sin(ang)
    v = np.asarray(v, float)
    return v * c + np.cross(k, v) * s_ + k * float(np.dot(k, v)) * (1.0 - c)


# ------------------------------------------------------------------- the model

class AprilTagSeeker:
    """SeekerModel implementation. Exposes a frame when the sim clock crosses
    k/fps (integer frame index -- never a float accumulator), scores it, and
    queues a Detection that becomes visible one pipeline latency later."""

    def __init__(self, cam: Optional[CameraParams] = None,
                 tag: Optional[TagParams] = None,
                 dec: Optional[DecodeParams] = None,
                 glare: Optional[GlareParams] = None) -> None:
        self.cam = cam or CameraParams()
        self.tag = tag or TagParams()
        self.dec = dec or DecodeParams()
        self.glare = glare or GlareParams()
        self.rng = np.random.default_rng(0)
        self.reset(self.rng)

    def reset(self, rng: np.random.Generator) -> None:
        self.rng = rng
        self._last_frame = -1
        self._queue: List[Detection] = []
        self._prev_t: Optional[float] = None
        self._prev_q: Optional[Tuple[float, float, float, float]] = None
        self._last_decode_capture_t = -math.inf   # DecodeParams.min_decode_interval_s
        # tag_realism_v1 B1: target-wobble OU state (radians, about the tag's
        # two in-plane axes) and the capture time it was last advanced to.
        self._shake = np.zeros(2)
        self._shake_t: Optional[float] = None
        self.frames = 0
        self.decodes = 0

    # ---- per-step entry point -------------------------------------------
    def observe(self, t: float, own: VehicleState, tgt: TargetState
                ) -> Tuple[Optional[Detection], Optional[FrameReport]]:
        k = int(math.floor(t * self.cam.fps + 1e-9))
        rep: Optional[FrameReport] = None
        if k > self._last_frame:
            self._last_frame = k
            rep = self._expose(t, own, tgt)
        self._prev_t, self._prev_q = t, own.quat_wxyz
        return self._release(t), rep

    def _release(self, t: float) -> Optional[Detection]:
        """At most one Detection per step. If several are due (latency jitter
        can reorder them), the freshest wins and the stale ones are dropped --
        a real pipeline never hands guidance an out-of-date frame."""
        if not self._queue:
            return None
        due = [d for d in self._queue if d.t_available <= t + _T_EPS]
        if not due:
            return None
        self._queue = [d for d in self._queue if d.t_available > t + _T_EPS]
        return max(due, key=lambda d: d.t_capture)

    # ---- one exposure ----------------------------------------------------
    def _expose(self, t: float, own: VehicleState, tgt: TargetState
                ) -> FrameReport:
        self.frames += 1
        cam, tag, dp = self.cam, self.tag, self.dec
        r_bn = quat_to_rot(own.quat_wxyz)
        r_cn = r_bn @ cam.rot_body_from_cam()          # camera -> NED
        cam_pos = camera_origin_ned(own.pos_ned, r_bn, cam)
        d_ned = np.asarray(tgt.pos_ned, float) - cam_pos
        p_cam = r_cn.T @ d_ned
        z = float(p_cam[2])
        u, v, in_front = project(p_cam, cam)
        in_fov = bool(in_front and 0.0 <= u < cam.width and 0.0 <= v < cam.height)

        normal, e1 = tag.frame(tgt, cam_pos)
        body = tag.body_mounted(tgt)

        # tag_realism_v1 B1: target attitude wobble. Tilts the tag about its
        # own in-plane axes BEFORE incidence/corners; the OU increment/dt is
        # the wobble's angular rate (feeds the rotational blur below). Guarded:
        # no draw and no state change when the knob is off.
        w_rot_ned = np.zeros(3)
        shake_deg = 0.0
        if dp.tgt_shake_rms_deg > 0.0 and not tag.faces_camera:
            ang, rate = self._shake_step(t)
            if e1 is None:
                e1 = _level_e1(normal)
            e2 = np.cross(normal, e1)
            rv = ang[0] * e1 + ang[1] * e2
            normal, e1 = _rotate(normal, rv), _rotate(e1, rv)
            w_rot_ned = w_rot_ned + rate[0] * e1 + rate[1] * e2
            shake_deg = math.degrees(float(np.linalg.norm(ang)))
        # A4: a body-mounted tag rotates with the target.
        if body and tgt.ang_vel_body is not None:
            w_rot_ned = w_rot_ned + quat_to_rot(tgt.quat_wxyz) @ np.asarray(
                tgt.ang_vel_body, float)

        # incidence: tag normal vs the tag->camera line.
        to_cam = -d_ned
        n_to_cam = float(np.linalg.norm(to_cam))
        cos_i = float(np.dot(normal, to_cam)) / max(n_to_cam, _EPS)
        incidence = math.degrees(math.acos(max(-1.0, min(1.0, cos_i))))

        side_px = cam.fx * tag.side_m / z if in_front and z > _EPS else 0.0
        blur_px = self._blur_px(t, own, tgt, p_cam, r_bn, r_cn) if in_front else 0.0

        # A4/B1 rotational smear (worst corner, small angle: corner speed
        # |w| * side/2 at depth z) and B2 own-camera vibration smear, combined
        # with the translational smear by root-sum-square (independent
        # directions). Both exactly 0 -- and blur_px untouched -- when off.
        blur_rot = 0.0
        w_rot = float(np.linalg.norm(w_rot_ned))
        if in_front and z > _EPS and w_rot > 0.0:
            blur_rot = cam.fx * w_rot * (0.5 * tag.side_m) / z * cam.exposure_s
        blur_vib = 0.0
        if cam.vib_rate_rms_dps > 0.0:
            vib = abs(float(self.rng.normal(0.0, cam.vib_rate_rms_dps)))
            if in_front:
                blur_vib = cam.fx * math.radians(vib) * cam.exposure_s
        if blur_rot > 0.0 or blur_vib > 0.0:
            blur_px = math.sqrt(blur_px * blur_px + blur_rot * blur_rot
                                + blur_vib * blur_vib)

        full = in_fov and self._corners_inside(tgt, normal, cam_pos, r_cn, e1)
        p = p_decode(side_px, incidence, blur_px, dp) if full else 0.0
        # C: sun-geometry glare, deterministic multipliers.
        g_spec = g_back = 1.0
        if self.glare.active():
            g_spec, g_back = glare_multipliers(self.glare, normal, to_cam, r_cn[:, 2])
            p = p * g_spec * g_back
        # Detection-cadence thinning (DecodeParams.min_decode_interval_s):
        # the pipeline never attempted this frame, so p -> 0 BEFORE the rng
        # draw (same no-draw shape as full=False). Inert at the 0.0 default.
        if dp.min_decode_interval_s > 0.0 and \
                (t - self._last_decode_capture_t) < dp.min_decode_interval_s - _T_EPS:
            p = 0.0
        decoded = bool(p > 0.0 and self.rng.random() < p)
        if decoded:
            self.decodes += 1
            self._last_decode_capture_t = t
            self._queue.append(self._measure(t, u, v, side_px, p_cam))
        return FrameReport(t_capture=t, in_fov=in_fov, side_px=side_px,
                           incidence_deg=incidence, blur_px=blur_px,
                           p_decode=p, decoded=decoded,
                           blur_rot_px=blur_rot, blur_vib_px=blur_vib,
                           tgt_shake_deg=shake_deg,
                           glare_specular_mult=g_spec, glare_backlight_mult=g_back)

    def _shake_step(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """Advance the 2-axis wobble OU process to capture time `t`. Exact
        discretisation: x' = phi*x + sigma*sqrt(1-phi^2)*N, phi = exp(-dt/tau),
        tau = 1/(2*pi*bw); the first frame draws from the stationary
        distribution. Returns (angles_rad, rate_rad_s) with rate = the actual
        increment / dt (0 on the first frame)."""
        dp = self.dec
        sigma = math.radians(dp.tgt_shake_rms_deg)
        if self._shake_t is None:
            self._shake = sigma * self.rng.standard_normal(2)
            self._shake_t = t
            return self._shake.copy(), np.zeros(2)
        dt = t - self._shake_t
        if dt <= 0.0:
            return self._shake.copy(), np.zeros(2)
        if dp.tgt_shake_bw_hz > 0.0:
            phi = math.exp(-dt * 2.0 * math.pi * dp.tgt_shake_bw_hz)
        else:
            phi = 1.0                              # zero bandwidth: frozen tilt
        new = phi * self._shake + sigma * math.sqrt(max(0.0, 1.0 - phi * phi)) \
            * self.rng.standard_normal(2)
        rate = (new - self._shake) / dt
        self._shake, self._shake_t = new, t
        return new.copy(), rate

    def _corners_inside(self, tgt, normal, cam_pos, r_cn, e1=None) -> bool:
        """Partial visibility: a tag with any corner off the sensor cannot be
        decoded (the detector needs the whole closed quad). `e1` = explicit
        in-plane basis (body-fixed / wobbled tag); None = legacy level basis."""
        cam = self.cam
        for c in tag_corners_ned(tgt.pos_ned, normal, self.tag.side_m, e1):
            cu, cv, ok = project(r_cn.T @ (c - cam_pos), cam)
            if not ok or not (0.0 <= cu < cam.width and 0.0 <= cv < cam.height):
                return False
        return True

    def _blur_px(self, t, own, tgt, p_cam, r_bn, r_cn) -> float:
        """Image-plane smear during the exposure = image-plane speed of the tag
        centre x exposure time. The apparent velocity includes the vehicle's own
        body rate, which is estimated by differencing the quaternion seen on the
        previous observe() call (first frame: assumed zero)."""
        w_b = np.zeros(3)
        if self._prev_q is not None and self._prev_t is not None:
            dt = t - self._prev_t
            if dt > 1e-9:
                w_b = _omega_body(self._prev_q, own.quat_wxyz, dt)
        d_body = r_bn.T @ (np.asarray(tgt.pos_ned, float) - np.asarray(own.pos_ned, float))
        v_body = r_bn.T @ (np.asarray(tgt.vel_ned, float) - np.asarray(own.vel_ned, float))
        # d/dt of a NED offset seen in a rotating body frame.
        dp_cam = self.cam.rot_body_from_cam().T @ (v_body - np.cross(w_b, d_body))
        z = float(p_cam[2])
        if z <= _EPS:
            return 0.0
        du = self.cam.fx * (float(dp_cam[0]) * z - float(p_cam[0]) * float(dp_cam[2])) / (z * z)
        dv = self.cam.fy * (float(dp_cam[1]) * z - float(p_cam[1]) * float(dp_cam[2])) / (z * z)
        return math.hypot(du, dv) * self.cam.exposure_s

    def _measure(self, t: float, u: float, v: float, side_px: float,
                 p_cam: np.ndarray) -> Detection:
        """Noisy pixel measurement -> the quantities the Pi would publish.

        Range comes from the tag pose, i.e. from apparent size:
        z = fx*side/side_px, so d(z)/d(side_px) = -z/side_px and
        sigma_z = z^2 * sigma_side / (fx*side) -- the range error grows as
        range squared, which is the standard monocular-fiducial behaviour. No
        separate sigma_range knob: it falls out of the pixel noise.
        """
        cam, dp = self.cam, self.dec
        un = u + self.rng.normal(0.0, dp.pixel_noise_px)
        vn = v + self.rng.normal(0.0, dp.pixel_noise_px)
        sn = side_px + self.rng.normal(0.0, dp.pixel_noise_px * dp.side_noise_factor)
        sn = max(sn, 1e-3)
        xn = (un - cam.cx) / cam.fx
        yn = (vn - cam.cy) / cam.fy
        z_est = cam.fx * self.tag.side_m / sn
        lat = cam.latency_s
        if cam.latency_jitter_s > 0.0:
            lat = max(0.0, lat + self.rng.normal(0.0, cam.latency_jitter_s))
        return Detection(
            t_capture=t, t_available=t + lat, u_px=un, v_px=vn, side_px=sn,
            range_m=z_est * math.sqrt(1.0 + xn * xn + yn * yn),
            bearing_deg=math.degrees(math.atan2(xn, 1.0)),
            elevation_deg=math.degrees(math.atan2(-yn, 1.0)),
        )


def _omega_body(q_prev, q_now, dt: float) -> np.ndarray:
    """Body angular rate from two body->NED quaternions, small-angle."""
    w0, x0, y0, z0 = (float(c) for c in q_prev)
    w1, x1, y1, z1 = (float(c) for c in q_now)
    # dq = conj(q_prev) * q_now  (the increment expressed in the body frame)
    dw = w0 * w1 + x0 * x1 + y0 * y1 + z0 * z1
    dx = w0 * x1 - x0 * w1 - y0 * z1 + z0 * y1
    dy = w0 * y1 + x0 * z1 - y0 * w1 - z0 * x1
    dz = w0 * z1 - x0 * y1 + y0 * x1 - z0 * w1
    s = -1.0 if dw < 0.0 else 1.0        # shortest-arc branch
    return np.array([2.0 * s * dx / dt, 2.0 * s * dy / dt, 2.0 * s * dz / dt])
