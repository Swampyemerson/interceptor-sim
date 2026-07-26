"""flight.camera -- pinhole + Brown-Conrady lens model for the real seeker's
bearing computation. Honesty-critical for the hardware build: the real
innomaker OV9281 (1280x800 mono global shutter, ~118 deg HFoV -- docs/
project_state.json hardware node, scripts/seeker/pi_capture.py DEF_WIDTH/HEIGHT)
behind a wide M12 lens has heavy barrel distortion the sim's pinhole camera does
NOT, so a bearing taken from a RAW distorted pixel is wrong toward the frame
edges -> corrupts the LOS the pro-nav terminal steers on. The seeker must
UNDISTORT the target pixel first.

This module is the parameterized machinery (the coefficients come from
`scripts/calibrate_camera.py`'s checkerboard calibration). Pure Python + math,
no cv2 dep -> runs anywhere and is unit-testable now; a zero-distortion model is
the pinhole special case the Gazebo sim uses, so wiring it into the sim seeker is
byte-identical there. The optical ray it returns feeds
flight.geometry.derotate_bearing_lambda unchanged (x right, y down, z forward).

Convention (OpenCV, matches GOALS.md camera frame): image (u,v) with +u right,
+v down; normalized coords xn=(u-cx)/fx, yn=(v-cy)/fy; optical ray (xn, yn, 1).
Brown-Conrady: xd = xn*(1+k1 r^2+k2 r^4+k3 r^6) + 2 p1 xn yn + p2 (r^2+2 xn^2),
r^2 = xn^2+yn^2 (the standard OpenCV distortion model).
"""

import math


class CameraModel:
    """Pinhole intrinsics + radial/tangential distortion. `dist` = (k1,k2,p1,p2,k3);
    all-zero (default) is an ideal pinhole (the Gazebo sim case)."""

    def __init__(self, fx, fy, cx, cy, dist=(0.0, 0.0, 0.0, 0.0, 0.0),
                 width=None, height=None, source=None):
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        k = list(dist) + [0.0] * (5 - len(dist))
        self.k1, self.k2, self.p1, self.p2, self.k3 = (float(v) for v in k[:5])
        # THE RESOLUTION IS PART OF THE CALIBRATION, NOT DECORATION (2026-07-26).
        # fx/fy/cx/cy only describe the pixel grid they were fitted on. from_dict
        # used to DROP calibrate_camera.py's `resolution` key, so nothing could
        # notice that seeker_loop streams the Pi camera at a different size than
        # the lens was calibrated at -- every bearing and every
        # range = fx*span/box_width_px silently rescales/shifts. None = the
        # calibration file did not say (older sidecars); callers on a live camera
        # must treat that as UNKNOWN, not as "matches".
        self.width = None if width is None else int(width)
        self.height = None if height is None else int(height)
        # Provenance stamp written by scripts/calibrate_camera.py ("checkerboard").
        # Absent => not a measured calibration (e.g. the Gazebo camera_info dump).
        self.source = source

    @property
    def has_distortion(self):
        return any((self.k1, self.k2, self.p1, self.p2, self.k3))

    def distort_normalized(self, xn, yn):
        """Forward model: ideal normalized (xn,yn) -> distorted normalized
        (what the real lens actually images). Used to build test fixtures and to
        verify the undistort inverse."""
        r2 = xn * xn + yn * yn
        radial = 1.0 + self.k1 * r2 + self.k2 * r2 * r2 + self.k3 * r2 * r2 * r2
        x_tan = 2.0 * self.p1 * xn * yn + self.p2 * (r2 + 2.0 * xn * xn)
        y_tan = self.p1 * (r2 + 2.0 * yn * yn) + 2.0 * self.p2 * xn * yn
        return xn * radial + x_tan, yn * radial + y_tan

    def undistort_pixel(self, u, v, iters=20):
        """Distorted pixel (u,v) -> IDEAL normalized (xn,yn) (removes lens
        distortion). Fixed-point inverse of distort_normalized (the same
        iteration OpenCV's undistortPoints uses); converges fast for lenses up
        to strong wide-angle. Returns the pinhole-equivalent normalized ray."""
        xd = (u - self.cx) / self.fx
        yd = (v - self.cy) / self.fy
        xn, yn = xd, yd  # initial guess = the distorted point itself
        for _ in range(iters):
            r2 = xn * xn + yn * yn
            radial = 1.0 + self.k1 * r2 + self.k2 * r2 * r2 + self.k3 * r2 * r2 * r2
            x_tan = 2.0 * self.p1 * xn * yn + self.p2 * (r2 + 2.0 * xn * xn)
            y_tan = self.p1 * (r2 + 2.0 * yn * yn) + 2.0 * self.p2 * xn * yn
            xn = (xd - x_tan) / radial
            yn = (yd - y_tan) / radial
        return xn, yn

    def pixel_to_ray(self, u, v):
        """Distorted pixel -> UNIT optical-frame ray (x right, y down, z forward),
        distortion removed. Feeds flight.geometry.derotate_bearing_lambda as
        meas_xyz."""
        xn, yn = self.undistort_pixel(u, v)
        n = math.sqrt(xn * xn + yn * yn + 1.0)
        return xn / n, yn / n, 1.0 / n

    def pixel_to_bearing(self, u, v):
        """Distorted pixel -> HORIZONTAL bearing (rad) in the camera optical
        frame: atan2(x_right, z_forward), distortion removed."""
        xn, yn = self.undistort_pixel(u, v)
        return math.atan2(xn, 1.0)

    @classmethod
    def from_dict(cls, d):
        """Build from a calibration dict. Accepts BOTH the clean
        {fx,fy,cx,cy, dist:[k1,k2,p1,p2,k3]} form AND scripts/calibrate_camera.py's
        output {fx,fy,cx,cy, dist_coeffs:[...], resolution:{...}} (the format that
        matches camera_intrinsics.json) -> the calibration tool's JSON loads
        directly. Only the first 5 distortion terms (Brown-Conrady k1,k2,p1,p2,k3)
        are used; a higher-order (rational) calibration is truncated to those.

        `resolution` (if present) and `source` are RETAINED -- see __init__: they
        are what lets a live-camera caller refuse a calibration that does not
        describe the frames it is actually getting."""
        dist = d.get("dist", d.get("dist_coeffs", (0.0, 0.0, 0.0, 0.0, 0.0)))
        res = d.get("resolution") or {}
        return cls(d["fx"], d["fy"], d["cx"], d["cy"], tuple(dist),
                   width=res.get("width"), height=res.get("height"),
                   source=d.get("source"))

    @classmethod
    def from_json(cls, path):
        """Load a calibration JSON file (scripts/calibrate_camera.py output)."""
        import json
        with open(path) as f:
            return cls.from_dict(json.load(f))
