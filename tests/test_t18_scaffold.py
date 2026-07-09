#!/usr/bin/env python3
"""T18 offline scaffold tests -- stereo triangulation (`scripts/ground_station/
triangulate.py`), the track filter (`scripts/ground_station/track_filter.py`),
and the sigma-R/sigma-v validation harness (`scripts/t18_sigma_validation.py`).
PURE MATH, no Gazebo/PX4/GPU -- runs in a few seconds against the T16 capture
already on disk (`logs/rig_captures/full_sweep_20260709T015530Z`).

VENV: main `.venv` (numpy 2.5.1 + pytest 9.1.1 -- confirmed present; `.venv-seeker`
has numpy but NO pytest). Matches how `scripts/stereo_model.py` / `scripts/
rig_geometry_analysis.py` already run (`.venv/bin/python scripts/...`), and
the project's existing pytest convention (`scripts/check_seeker_v2.sh`'s
`.venv/bin/python -m pytest tests/test_honesty_static.py`).

Run:  .venv/bin/python -m pytest tests/test_t18_scaffold.py -v

Four checks, mapping to the task's requirements:
  (a) test_round_trip_zero_jitter -- project world->pixels->triangulate is
      the identity map at zero jitter, to mm precision.
  (b) test_datum_bias_shifts_output_by_bias_magnitude -- injecting a 0.5 m
      --datum-bias-m shifts the triangulated output by ~0.5 m.
  (c) test_sigma_R_scaling_reproduces_analytic_row -- the harness's own
      resolution-scaled sigma_R measurement reproduces rig_geometry_analysis
      .md's EXPECTED-tier table row (c ~= 3.67e-05 @ 1920x1200) within 10%.
  (d) test_track_filter_converges_on_constant_velocity -- GroundTrackFilter's
      velocity estimate converges close to the true constant velocity on a
      noise-free synthetic track.
"""
import math
import os
import sys

import pytest

SCRIPTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts")
sys.path.insert(0, SCRIPTS)

from ground_station.triangulate import (  # noqa: E402
    RigConfig,
    apply_datum_bias,
    project_world_to_stereo_pixels,
    triangulate,
)
from ground_station.track_filter import GroundTrackFilter  # noqa: E402
import t18_sigma_validation as t18  # noqa: E402

REPO_ROOT = os.path.dirname(SCRIPTS)
DEFAULT_SWEEP_DIR = os.path.join(REPO_ROOT, "logs", "rig_captures", "full_sweep_20260709T015530Z")


@pytest.fixture(scope="module")
def rig():
    meta_path = os.path.join(DEFAULT_SWEEP_DIR, "capture_meta.json")
    if not os.path.exists(meta_path):
        pytest.skip(f"no T16 capture at {DEFAULT_SWEEP_DIR}")
    return RigConfig.from_capture_meta(meta_path)


# ============================================================================
# (a) round-trip project -> triangulate == identity at zero jitter
# ============================================================================
def test_round_trip_zero_jitter(rig):
    test_points = [
        (6.5, 0.0, 0.5),      # CPA, on the dash line
        (6.5, 29.3, 0.5),     # dash start (r2l)
        (6.5, -29.3, 0.5),    # dash start (l2r)
        (6.5, 16.0, 0.5),     # mid-dash
        (0.0, 0.0, 1.8),      # near the rig's own altitude/lateral origin
    ]
    for p in test_points:
        uv = project_world_to_stereo_pixels(p, rig.true_pose, rig.baseline_m,
                                             rig.f_px, rig.cx, rig.cy)
        assert uv is not None, f"point {p} projected as None (behind camera?)"
        u_l, v_l, u_r, v_r = uv
        result = triangulate(u_l, v_l, u_r, v_r, rig.true_pose, rig.baseline_m,
                              rig.f_px, rig.cx, rig.cy)
        assert result is not None
        err_m = math.dist(p, (result.x, result.y, result.z))
        assert err_m < 1e-6, f"round-trip error {err_m*1000:.6f} mm at {p} (expected mm-level/exact)"


# ============================================================================
# (b) datum-bias injection shifts the output by ~the bias magnitude
# ============================================================================
def test_datum_bias_shifts_output_by_bias_magnitude(rig):
    p = (6.5, 10.0, 0.5)
    uv = project_world_to_stereo_pixels(p, rig.true_pose, rig.baseline_m,
                                         rig.f_px, rig.cx, rig.cy)
    u_l, v_l, u_r, v_r = uv
    unbiased = triangulate(u_l, v_l, u_r, v_r, rig.true_pose, rig.baseline_m,
                            rig.f_px, rig.cx, rig.cy)

    bias_m = 0.5
    biased_pose = apply_datum_bias(rig.true_pose, bias_m)
    biased = triangulate(u_l, v_l, u_r, v_r, biased_pose, rig.baseline_m,
                          rig.f_px, rig.cx, rig.cy)

    shift = math.dist((unbiased.x, unbiased.y, unbiased.z), (biased.x, biased.y, biased.z))
    assert shift == pytest.approx(bias_m, abs=1e-6), (
        f"datum-bias-m={bias_m} produced an output shift of {shift:.6f} m "
        "(expected an exact 1:1 shift for a pure position bias, see "
        "triangulate.py's module docstring derivation)"
    )

    # --assumed-rig-pose is the general escape hatch: an explicit pose with
    # only x shifted by -0.5 (matching apply_datum_bias's own convention)
    # must reproduce apply_datum_bias's result exactly for this yaw=0 rig.
    explicit_pose = rig.true_pose._replace(x=rig.true_pose.x - bias_m)
    explicit = triangulate(u_l, v_l, u_r, v_r, explicit_pose, rig.baseline_m,
                            rig.f_px, rig.cx, rig.cy)
    assert explicit.x == pytest.approx(biased.x, abs=1e-9)
    assert explicit.y == pytest.approx(biased.y, abs=1e-9)
    assert explicit.z == pytest.approx(biased.z, abs=1e-9)


# ============================================================================
# (c) sigma_R scaling reproduces the analytic table row (~10%)
# ============================================================================
def test_sigma_R_scaling_reproduces_analytic_row(rig):
    """Reproduces docs/rig_geometry_analysis.md's EXPECTED-tier row for
    1920x1200 (c ~= 3.67e-05) at every A1_RANGES_M point, via the same
    synthetic range-sweep the harness's report uses (smaller n_trials here
    for test speed -- the report uses 3000; the tolerance is unchanged)."""
    sigma_d = 0.4  # EXPECTED matching-noise tier
    c_model = t18.model_c_matching(sigma_d, rig.baseline_m, rig.f_px)
    assert c_model == pytest.approx(3.6727715e-05, rel=0.01), (
        "this rig's own f_px should reproduce the doc's c_EXPECTED ~= 3.67e-05 "
        f"row (docs/rig_geometry_analysis.md Sec 1); got {c_model:.4e}"
    )

    for R in t18.A1_RANGES_M:
        p = t18.synthetic_boresight_point(rig, R)
        measured = t18.measure_sigma_R_at_point(rig, p, sigma_d, n_trials=1500, seed=42)
        assert measured is not None
        measured_sigma, _mean_resid, _n_ok, true_R = measured
        model_sigma = c_model * true_R * true_R
        rel_err = abs(measured_sigma - model_sigma) / model_sigma
        assert rel_err <= 0.10, (
            f"R={R} m: measured sigma_R={measured_sigma:.4f} vs model={model_sigma:.4f} "
            f"(rel_err={rel_err:.2%}, tolerance 10%)"
        )


# ============================================================================
# (d) filter converges on a synthetic constant-velocity track
# ============================================================================
def test_track_filter_converges_on_constant_velocity():
    """No triangulation here (pure filter-mechanics check): a target moving
    at a known constant velocity, measured NOISE-FREE (deterministic, so
    this test is non-flaky -- see t18_sigma_validation.py's own
    docstring/report for why a NOISY Kalata convergence check is highly
    gain-dependent and easy to under/over-tolerance), tracked by
    GroundTrackFilter -- after enough ticks the velocity estimate should
    converge to the true velocity on every axis (confirmed against a
    zero-noise deterministic trace during this task: ~150-200 ticks at
    these gains, see the task report)."""
    true_vel = (2.0, -3.0, 0.5)
    pos = [0.0, 10.0, 5.0]
    T = 0.1
    gtf = GroundTrackFilter(update_rate_hz=1.0 / T, kalata_sigma_process=1.0,
                             kalata_sigma_meas=0.5)
    t = 0.0
    state = None
    for i in range(400):
        dt = T if gtf.initialized else None
        state = gtf.step(pos[0], pos[1], pos[2], t, dt=dt)
        t += T
        pos[0] += true_vel[0] * T
        pos[1] += true_vel[1] * T
        pos[2] += true_vel[2] * T

    assert state.vx == pytest.approx(true_vel[0], abs=1e-3)
    assert state.vy == pytest.approx(true_vel[1], abs=1e-3)
    assert state.vz == pytest.approx(true_vel[2], abs=1e-3)
