"""isim/ownstate.py tests (isim/specs/pursuit_hardening_v5.md #1-4). Loose
bounds only -- pins BEHAVIOUR (the wrapper perturbs what it's supposed to,
leaves everything else alone, and every error source is individually
zeroable to exactly a no-op), never a specific numeric miss outcome."""
from __future__ import annotations

import math

import numpy as np
import pytest

from isim.ownstate import OwnStateNoise, OwnStateNoiseConfig
from isim.stubs import yaw_to_quat_wxyz
from isim.types import Detection, VehicleState, VelCmd


class _RecordingInner:
    """A trivial Guidance that records every `own`/`det` it is handed and
    returns a fixed command -- lets a test see EXACTLY what OwnStateNoise
    forwarded, without a real guidance law's own dynamics in the way."""

    def __init__(self) -> None:
        self.seen = []
        self.debug = "inner-debug-sentinel"   # for the __getattr__ passthrough test

    def reset(self) -> None:
        self.seen = []

    def step(self, t, own, det):
        self.seen.append((t, own, det))
        return VelCmd(0.0, 0.0, 0.0, 0.0)


def _own(t, pos=(0.0, 0.0, -5.0), vel=(0.0, 0.0, 0.0), yaw=0.0):
    return VehicleState(t=t, pos_ned=np.asarray(pos, dtype=np.float64),
                       vel_ned=np.asarray(vel, dtype=np.float64),
                       quat_wxyz=yaw_to_quat_wxyz(yaw), yaw_rad=yaw)


def _zero_cfg(**overrides) -> OwnStateNoiseConfig:
    """Every sigma at 0.0 -- the "disabled" config; individual overrides
    turn ONE error source back on, per the spec's "each individually
    switchable to zero so its cost can be isolated"."""
    zeros = dict(
        attitude_bias_rp_sigma_deg=0.0, attitude_bias_yaw_sigma_deg=0.0,
        attitude_white_sigma_deg=0.0, attitude_bias_tau_s=20.0,
        vel_bias_sigma_ms=0.0, vel_white_sigma_ms=0.0,
        pos_randomwalk_sigma_ms_sqrt_s=0.0,
        frame_ts_bias_max_s=0.0, frame_ts_jitter_sigma_s=0.0,
    )
    zeros.update(overrides)
    return OwnStateNoiseConfig(**zeros)


# --------------------------------------------------------------- zero = no-op

def test_all_zero_config_is_an_exact_noop():
    inner = _RecordingInner()
    wrapper = OwnStateNoise(inner, _zero_cfg(), np.random.default_rng(0))
    own = _own(0.0)
    det = Detection(t_capture=0.0, t_available=0.02, u_px=1, v_px=1, side_px=20.0,
                   range_m=5.0, bearing_deg=1.0, elevation_deg=1.0)
    wrapper.step(0.0, own, det)
    seen_t, seen_own, seen_det = inner.seen[0]
    np.testing.assert_array_equal(seen_own.pos_ned, own.pos_ned)
    np.testing.assert_array_equal(seen_own.vel_ned, own.vel_ned)
    np.testing.assert_allclose(seen_own.quat_wxyz, own.quat_wxyz, atol=1e-12)
    assert seen_det.t_capture == pytest.approx(det.t_capture, abs=1e-12)


def test_getattr_passthrough_to_inner():
    inner = _RecordingInner()
    wrapper = OwnStateNoise(inner, _zero_cfg(), np.random.default_rng(0))
    assert wrapper.debug == "inner-debug-sentinel"


# ------------------------------------------------------------- each source alone

def test_attitude_noise_perturbs_quat_but_not_pos_or_vel():
    inner = _RecordingInner()
    cfg = _zero_cfg(attitude_white_sigma_deg=5.0)   # big, so it's unmistakable
    wrapper = OwnStateNoise(inner, cfg, np.random.default_rng(1))
    own = _own(0.0)
    wrapper.step(0.0, own, None)
    _, seen_own, _ = inner.seen[0]
    assert not np.allclose(seen_own.quat_wxyz, own.quat_wxyz, atol=1e-6)
    np.testing.assert_array_equal(seen_own.pos_ned, own.pos_ned)
    np.testing.assert_array_equal(seen_own.vel_ned, own.vel_ned)


def test_velocity_noise_perturbs_vel_but_not_pos_or_quat():
    inner = _RecordingInner()
    cfg = _zero_cfg(vel_white_sigma_ms=5.0)
    wrapper = OwnStateNoise(inner, cfg, np.random.default_rng(2))
    own = _own(0.0)
    wrapper.step(0.0, own, None)
    _, seen_own, _ = inner.seen[0]
    assert not np.allclose(seen_own.vel_ned, own.vel_ned, atol=1e-6)
    np.testing.assert_array_equal(seen_own.pos_ned, own.pos_ned)
    np.testing.assert_allclose(seen_own.quat_wxyz, own.quat_wxyz, atol=1e-12)


def test_velocity_bias_is_constant_across_the_run():
    """A per-run bias, not white noise: with white noise at 0, EVERY tick's
    velocity offset from truth must be IDENTICAL."""
    inner = _RecordingInner()
    cfg = _zero_cfg(vel_bias_sigma_ms=1.0)
    wrapper = OwnStateNoise(inner, cfg, np.random.default_rng(3))
    for k in range(5):
        wrapper.step(float(k) * 0.02, _own(float(k) * 0.02), None)
    offsets = [seen_own.vel_ned - own_vel for (_, seen_own, _), own_vel in
              zip(inner.seen, [np.zeros(3)] * 5)]
    for off in offsets[1:]:
        np.testing.assert_allclose(off, offsets[0], atol=1e-12)
    assert float(np.linalg.norm(offsets[0])) > 0.0   # and it's not trivially zero


def test_position_randomwalk_grows_with_time_not_constant():
    """A random walk's variance grows with time (a bias would not) --
    checked across many independent seeds (a single trajectory can wander
    back toward zero by chance; the ENSEMBLE spread cannot shrink)."""
    early_offsets, late_offsets = [], []
    for seed in range(60):
        inner = _RecordingInner()
        cfg = _zero_cfg(pos_randomwalk_sigma_ms_sqrt_s=1.0)
        wrapper = OwnStateNoise(inner, cfg, np.random.default_rng(seed))
        t = 0.0
        for _ in range(500):   # 10 s at dt=0.02
            wrapper.step(t, _own(t), None)
            t += 0.02
        early_offsets.append(float(np.linalg.norm(inner.seen[10][1].pos_ned)))
        late_offsets.append(float(np.linalg.norm(inner.seen[-1][1].pos_ned)))
    early_rms = float(np.sqrt(np.mean(np.square(early_offsets))))
    late_rms = float(np.sqrt(np.mean(np.square(late_offsets))))
    print(f"pos randomwalk RMS |offset|: at t=0.2s={early_rms:.3f}, at t=10s={late_rms:.3f}")
    assert late_rms > early_rms   # a walk's spread grows; a bias's would not


def test_frame_timestamp_bias_is_constant_and_jitter_varies():
    inner = _RecordingInner()
    cfg = _zero_cfg(frame_ts_bias_max_s=0.020)   # jitter still 0 -- pure bias test
    wrapper = OwnStateNoise(inner, cfg, np.random.default_rng(5))
    dets = []
    for k in range(3):
        det = Detection(t_capture=float(k), t_available=float(k) + 0.02, u_px=1, v_px=1,
                       side_px=20.0, range_m=5.0, bearing_deg=0.0, elevation_deg=0.0)
        wrapper.step(float(k) + 0.02, _own(float(k) + 0.02), det)
        dets.append(det)
    biases = [seen_det.t_capture - det.t_capture for (_, _, seen_det), det in
             zip(inner.seen, dets)]
    for b in biases[1:]:
        assert b == pytest.approx(biases[0], abs=1e-12)
    assert abs(biases[0]) <= 0.020 + 1e-9
    assert biases[0] != 0.0   # (astronomically unlikely to land exactly on 0)


def test_frame_timestamp_jitter_differs_per_frame():
    inner = _RecordingInner()
    cfg = _zero_cfg(frame_ts_jitter_sigma_s=0.010)
    wrapper = OwnStateNoise(inner, cfg, np.random.default_rng(6))
    for k in range(5):
        det = Detection(t_capture=float(k), t_available=float(k) + 0.02, u_px=1, v_px=1,
                       side_px=20.0, range_m=5.0, bearing_deg=0.0, elevation_deg=0.0)
        wrapper.step(float(k) + 0.02, _own(float(k) + 0.02), det)
    ts_seen = [seen_det.t_capture - float(k) for k, (_, _, seen_det) in enumerate(inner.seen)]
    assert len(set(ts_seen)) > 1   # not all identical -- real per-frame jitter


# --------------------------------------------------------------------- honesty

def test_true_det_object_is_not_mutated():
    """The wrapper must hand `inner` a NEW Detection (dataclasses.replace),
    never mutate the caller's own `det` -- the engine/scoring's copy of
    `det` (if it kept a reference) must stay exactly the true value."""
    inner = _RecordingInner()
    cfg = _zero_cfg(frame_ts_bias_max_s=0.020, frame_ts_jitter_sigma_s=0.010)
    wrapper = OwnStateNoise(inner, cfg, np.random.default_rng(7))
    det = Detection(t_capture=1.0, t_available=1.02, u_px=1, v_px=1, side_px=20.0,
                   range_m=5.0, bearing_deg=0.0, elevation_deg=0.0)
    wrapper.step(1.02, _own(1.02), det)
    assert det.t_capture == 1.0   # untouched
    _, _, seen_det = inner.seen[0]
    assert seen_det is not det
    assert seen_det.t_capture != 1.0
