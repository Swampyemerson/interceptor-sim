"""Pin flight.fov_guidance (Phase B: vertical FOV-hold -- look-angle-
constrained PN augmentation + IBVS pixel-centering). Pure-math behavioral
assertions on synthetic geometry -- no sim. Run standalone (exit 0/1) or
under pytest.
"""

import math

from flight.fov_guidance import (
    FovHoldConfig,
    augment_pronav,
    fov_edge_barrier_accel,
    fov_hold_vertical_accel,
    half_fov_from_intrinsics,
    horizontal_look_angle,
    ibvs_centering_accel,
    pitch_headroom_accel_cap,
    pixel_v_to_look_angle,
    vertical_look_angle,
)


def test_look_angle_signs():
    """Optical frame is x right, y down, z forward: a ray with y<0 (target
    ABOVE boresight -- the dash failure mode) gives eps_v<0; y>0 gives >0;
    on-boresight gives 0. Horizontal mirrors the bearing convention."""
    assert vertical_look_angle((0.0, -0.5, 1.0)) < 0.0
    assert vertical_look_angle((0.0, 0.5, 1.0)) > 0.0
    assert vertical_look_angle((0.3, 0.0, 1.0)) == 0.0
    assert vertical_look_angle(None) is None
    assert abs(horizontal_look_angle((0.5, 0.0, 1.0)) - math.atan2(0.5, 1.0)) < 1e-12
    assert abs(vertical_look_angle((0.0, -1.0, 1.0)) + math.pi / 4) < 1e-12


def test_pixel_form_matches_ray_form():
    """pixel_v_to_look_angle is the pixel-space alias of the ray form: a
    pixel offset (v-cy) maps through fy to the same angle as the normalized
    ray (0, (v-cy)/fy, 1)."""
    fy, cy = 500.0, 360.0
    for v in (0.0, 200.0, 360.0, 719.0):
        yn = (v - cy) / fy
        a_px = pixel_v_to_look_angle(v, cy, fy)
        a_ray = vertical_look_angle((0.0, yn, 1.0))
        assert abs(a_px - a_ray) < 1e-12, (v, a_px, a_ray)


def test_half_fov_from_intrinsics():
    """Nominal sim camera: ~100 deg HFOV at 1280 px -> fx = 640/tan(50 deg);
    the helper must recover the 50 deg half-angle (numbers trace to a
    derivation, not a typed constant)."""
    fx = 640.0 / math.tan(math.radians(50.0))
    assert abs(math.degrees(half_fov_from_intrinsics(fx, 1280)) - 50.0) < 1e-9


def test_centering_sign():
    """IBVS centering opposes the offset: target at frame TOP (eps_v<0) ->
    CLIMB (a_up>0); bottom -> descend; centered/no-ray -> zero. Scales with
    Vc (the PN-style normalization)."""
    cfg = FovHoldConfig()
    up = ibvs_centering_accel(-0.2, 6.0, cfg)
    dn = ibvs_centering_accel(0.2, 6.0, cfg)
    assert up > 0.0 and dn < 0.0 and abs(up + dn) < 1e-12
    assert ibvs_centering_accel(0.0, 6.0, cfg) == 0.0
    assert ibvs_centering_accel(None, 6.0, cfg) == 0.0
    assert abs(ibvs_centering_accel(-0.2, 12.0, cfg) - 2.0 * up) < 1e-12


def test_barrier_deadband_and_monotone_growth():
    """The barrier is OFF inside engage_frac of the usable half-FOV (the
    wedge + base law own the frame center), C1-smooth at engagement, and
    grows monotonically toward the edge, always pushing back to center."""
    cfg = FovHoldConfig()  # limit 0.54, engage 0.324
    assert fov_edge_barrier_accel(0.0, 6.0, cfg) == 0.0
    assert fov_edge_barrier_accel(-0.30, 6.0, cfg) == 0.0     # inside deadband
    a1 = fov_edge_barrier_accel(-0.40, 6.0, cfg)
    a2 = fov_edge_barrier_accel(-0.50, 6.0, cfg)
    assert a1 > 0.0 and a2 > a1                                # push = climb, growing
    assert fov_edge_barrier_accel(0.40, 6.0, cfg) == -a1       # mirror
    # C1 at engagement: just past the threshold the push is ~0 (no step).
    eps = -(cfg.eps_engage_rad + 1e-4)
    assert 0.0 < fov_edge_barrier_accel(eps, 6.0, cfg) < 1e-3


def test_total_command_capped():
    """The combined vertical command saturates at a_up_max (bounded
    authority -- the term cannot strand the base law)."""
    cfg = FovHoldConfig(a_up_max_m_s2=4.0, k_barrier=50.0)
    a = fov_hold_vertical_accel(-0.53, 20.0, cfg)
    assert abs(a) <= 4.0 + 1e-12 and a > 0.0


def test_pitch_headroom_cap():
    """Look-angle constraint: centered target + level flight allows
    g*tan(eps_limit) of forward accel; a target already at the usable top
    edge allows NONE; more headroom the lower the target sits in frame."""
    cfg = FovHoldConfig()
    g = 9.81
    cap_center = pitch_headroom_accel_cap(0.0, 0.0, cfg, g)
    assert abs(cap_center - g * math.tan(cfg.eps_limit_rad)) < 1e-9
    assert pitch_headroom_accel_cap(-cfg.eps_limit_rad, 0.0, cfg, g) == 0.0
    cap_low = pitch_headroom_accel_cap(0.2, 0.0, cfg, g)
    cap_high = pitch_headroom_accel_cap(-0.2, 0.0, cfg, g)
    assert cap_low > cap_center > cap_high > 0.0
    # A vehicle already nosed down 0.3 rad with the target STILL centered
    # has that pitch's worth of extra absolute-accel headroom.
    cap_pitched = pitch_headroom_accel_cap(0.0, -0.3, cfg, g)
    assert abs(cap_pitched - g * math.tan(cfg.eps_limit_rad + 0.3)) < 1e-9
    assert pitch_headroom_accel_cap(0.0, None, cfg, g) is None


def test_augment_passthrough_and_inert():
    """augment_pronav never touches the horizontal base command, and with no
    ray it is fully inert (byte-identical coasting behavior)."""
    cfg = FovHoldConfig()
    out = augment_pronav(3.7, (0.0, -0.4, 1.0), 6.0, cfg, pitch_rad=-0.45)
    assert out.a_lat_m_s2 == 3.7
    assert out.a_up_m_s2 > 0.0                 # target high -> climb
    assert out.barrier_active                  # 0.38 rad > engage 0.324
    assert out.a_fwd_cap_m_s2 is not None and out.a_fwd_cap_m_s2 > 0.0
    inert = augment_pronav(3.7, None, 6.0, cfg, pitch_rad=-0.45)
    assert inert.a_lat_m_s2 == 3.7 and inert.a_up_m_s2 == 0.0
    assert inert.a_fwd_cap_m_s2 is None and inert.eps_v_rad is None
    assert not inert.barrier_active


def test_closed_loop_reduces_offset_and_holds_fov():
    """THE PHASE-B PROPERTY on a synthetic vertical plant: camera boresight
    pitched 0.45 rad below a co-altitude target (the dash geometry), closing
    12 m -> 2.5 m at 6 m/s. Integrating only the FOV-hold vertical command
    (double-integrator plant, LOS elevation = atan2(-z, R)) must (a) shrink
    the look-angle offset, (b) drive it inside the barrier deadband at some
    point, and (c) never let the target leave the usable FOV."""
    cfg = FovHoldConfig()
    theta_c = -0.45          # camera boresight elevation (rad, nose-down dash)
    z, v_up = 0.0, 0.0       # interceptor vertical state (up positive)
    r, vc = 12.0, 6.0
    dt = 0.02
    eps0 = None
    min_abs_eps = float("inf")
    while r > 2.5:
        el = math.atan2(-z, r)                    # LOS elevation seen from vehicle
        off = el - theta_c                        # target angle above boresight
        ray = (0.0, -math.sin(off), math.cos(off))
        cmd = augment_pronav(0.0, ray, vc, cfg, pitch_rad=theta_c)
        eps = cmd.eps_v_rad
        if eps0 is None:
            eps0 = eps
        min_abs_eps = min(min_abs_eps, abs(eps))
        assert abs(eps) < cfg.eps_limit_rad, (r, eps)   # (c) stays in usable FOV
        v_up += cmd.a_up_m_s2 * dt
        z += v_up * dt
        r -= vc * dt
    assert abs(eps0) > cfg.eps_engage_rad             # started past the deadband
    assert abs(eps) < abs(eps0)                       # (a) offset reduced
    assert min_abs_eps < cfg.eps_engage_rad           # (b) re-centered past engage


ALL = [test_look_angle_signs, test_pixel_form_matches_ray_form,
       test_half_fov_from_intrinsics, test_centering_sign,
       test_barrier_deadband_and_monotone_growth, test_total_command_capped,
       test_pitch_headroom_cap, test_augment_passthrough_and_inert,
       test_closed_loop_reduces_offset_and_holds_fov]

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
    print(f"{len(ALL) - failed}/{len(ALL)} fov_guidance tests passed")
    sys.exit(1 if failed else 0)
