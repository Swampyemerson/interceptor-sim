"""Pin flight.camera: pinhole exactness (the Gazebo-sim special case) + a
distort/undistort roundtrip for a wide-angle barrel lens (the real M12).
Run standalone (exit 0/1) or under pytest.
"""

import math

from flight.camera import CameraModel

# A 1280x720-ish pinhole (no distortion) and the same body with barrel distortion.
PINHOLE = CameraModel(fx=500.0, fy=500.0, cx=640.0, cy=360.0)
WIDE = CameraModel(fx=500.0, fy=500.0, cx=640.0, cy=360.0,
                   dist=(-0.30, 0.10, 0.001, -0.001, 0.0))  # barrel + small tangential


def test_pinhole_center_is_boresight():
    r = PINHOLE.pixel_to_ray(640.0, 360.0)
    assert abs(r[0]) < 1e-12 and abs(r[1]) < 1e-12 and abs(r[2] - 1.0) < 1e-12
    assert abs(PINHOLE.pixel_to_bearing(640.0, 360.0)) < 1e-12


def test_pinhole_45_degrees():
    """A pixel one focal length right of center -> xn=1 -> bearing 45 deg."""
    b = PINHOLE.pixel_to_bearing(640.0 + 500.0, 360.0)
    assert abs(b - math.radians(45.0)) < 1e-9
    r = PINHOLE.pixel_to_ray(640.0 + 500.0, 360.0)
    assert abs(math.hypot(r[0], math.hypot(r[1], r[2])) - 1.0) < 1e-12  # unit ray


def test_pinhole_no_distortion_flag():
    assert not PINHOLE.has_distortion
    assert WIDE.has_distortion


def test_distort_undistort_roundtrip():
    """distort_normalized then undistort_pixel recovers the ideal ray to sub-
    pixel accuracy across the frame (the inverse converges)."""
    for xn in (-0.5, -0.2, 0.0, 0.3, 0.6):
        for yn in (-0.4, 0.0, 0.25, 0.5):
            xd, yd = WIDE.distort_normalized(xn, yn)
            u = WIDE.cx + xd * WIDE.fx
            v = WIDE.cy + yd * WIDE.fy
            xr, yr = WIDE.undistort_pixel(u, v)
            assert abs(xr - xn) < 1e-6 and abs(yr - yn) < 1e-6, (xn, yn, xr, yr)


def test_distortion_actually_bends_edge():
    """At the frame edge, the barrel lens gives a DIFFERENT bearing than the
    pinhole would from the same raw pixel -- the error the undistort removes."""
    u_edge = 640.0 + 480.0   # near the right edge
    b_wide = WIDE.pixel_to_bearing(u_edge, 360.0)
    b_pin = PINHOLE.pixel_to_bearing(u_edge, 360.0)
    assert abs(b_wide - b_pin) > math.radians(1.0)  # >1 deg correction at the edge


def test_from_dict():
    m = CameraModel.from_dict({"fx": 500, "fy": 500, "cx": 640, "cy": 360,
                               "dist": [-0.3, 0.1, 0.0, 0.0, 0.0]})
    assert m.k1 == -0.3 and m.has_distortion


ALL = [test_pinhole_center_is_boresight, test_pinhole_45_degrees,
       test_pinhole_no_distortion_flag, test_distort_undistort_roundtrip,
       test_distortion_actually_bends_edge, test_from_dict]

if __name__ == "__main__":
    import sys
    failed = 0
    for t in ALL:
        try:
            t()
            print(f"PASS {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {t.__name__}: {e}")
    print(f"{len(ALL) - failed}/{len(ALL)} camera tests passed")
    sys.exit(1 if failed else 0)
