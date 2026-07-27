"""The flying camera's frame rate must stay PINNED, not inherited.

WHY THIS TEST EXISTS
--------------------
`PicameraSource` set no `FrameDurationLimits`, so the flying seeker ran at
whatever picamera2 defaulted to. Measured on the real Pi 5
(`scripts/seeker/probe_flight_frame_rate.py`): **30.048 fps as flown vs 114.9
with the cap lifted** — 3.82x — while the sensor does 143 and AprilTag decode
throughput is ~112. A default nobody chose was setting a load-bearing number:
`R_streak_burn = (E[T]/fps) x V_closing` (ADR-0079), so it was a 3.8x error in the
range consumed forming the handoff lock, and an input to the ~$740 gate.

The audit of 2026-07-25 recorded exactly this failure mode one layer over, on the
deployed weights: "No test asserts the default, so the fix can silently revert."
This is that test for the frame rate — the revert would be invisible off-hardware
and would only show up as a seeker that mysteriously burns 3.8x the range.

Runs off-hardware by faking picamera2, so CI catches a revert without a Pi.
"""
import sys
import types

import pytest

from flight.deploy import seeker_loop


def test_default_is_a_real_positive_rate():
    fps = seeker_loop.CAMERA_FPS_DEFAULT
    assert isinstance(fps, (int, float)) and fps > 0, (
        "CAMERA_FPS_DEFAULT must be a positive rate; unsetting it silently "
        "restores picamera2's inherited default (measured 30.048 fps)")
    # Sanity-bracket it against what the hardware measured: above the inherited
    # 30 it replaces, at or below the 96.6 fps sustained at quad_decimate=2.0.
    assert 30.0 < fps <= 96.6, (
        f"CAMERA_FPS_DEFAULT={fps} is outside the measured envelope "
        "(inherited 30.048 fps .. 96.6 fps sustained, ADR-0090)")


def test_cli_default_tracks_the_constant():
    """--camera-fps must not drift away from the constant it documents."""
    ap = seeker_loop.build_argparser() if hasattr(seeker_loop, "build_argparser") \
        else None
    if ap is None:
        pytest.skip("no separable argparser to introspect")
    ns = ap.parse_args([])
    assert ns.camera_fps == seeker_loop.CAMERA_FPS_DEFAULT


class _FakeCam:
    """Minimal picamera2 stand-in: records what it was configured with."""

    def __init__(self):
        self.camera_controls = {}
        self.configured = None
        self.started = False

    def create_video_configuration(self, main=None, controls=None):
        return {"main": main, "controls": controls}

    def configure(self, cfg):
        self.configured = cfg

    def start(self):
        self.started = True

    def set_controls(self, c):
        pass


def _install_fake_picamera2(monkeypatch, holder):
    mod = types.ModuleType("picamera2")

    def _ctor():
        cam = _FakeCam()
        holder.append(cam)
        return cam

    mod.Picamera2 = _ctor
    monkeypatch.setitem(sys.modules, "picamera2", mod)


def test_source_pins_frame_duration(monkeypatch):
    """The control must actually reach the camera configuration."""
    cams = []
    _install_fake_picamera2(monkeypatch, cams)

    src = seeker_loop.PicameraSource()
    controls = cams[0].configured["controls"]
    assert "FrameDurationLimits" in controls, (
        "FrameDurationLimits absent -- the flying camera would inherit "
        "picamera2's default again (30.048 fps measured)")

    lo, hi = controls["FrameDurationLimits"]
    assert lo == hi, "frame duration must be pinned, not a range"
    assert abs(1e6 / lo - seeker_loop.CAMERA_FPS_DEFAULT) < 0.5, (
        f"pinned duration {lo} us != {seeker_loop.CAMERA_FPS_DEFAULT} fps")
    assert src.target_fps == seeker_loop.CAMERA_FPS_DEFAULT


def test_explicit_zero_leaves_it_unpinned(monkeypatch):
    """Opting out stays possible -- it just must be deliberate."""
    cams = []
    _install_fake_picamera2(monkeypatch, cams)
    seeker_loop.PicameraSource(target_fps=0)
    assert "FrameDurationLimits" not in cams[0].configured["controls"]


def test_frame_duration_shorter_than_exposure_is_refused(monkeypatch):
    """Exposure is a hard floor on frame duration; asking for the impossible
    must fail loudly rather than silently deliver some other rate."""
    cams = []
    _install_fake_picamera2(monkeypatch, cams)
    with pytest.raises(RuntimeError, match="frame duration"):
        # 2000 fps => 500 us frame, shorter than the 1000 us exposure spec
        seeker_loop.PicameraSource(target_fps=2000.0,
                                   exposure_us=seeker_loop.EXPOSURE_SPEC_US)
