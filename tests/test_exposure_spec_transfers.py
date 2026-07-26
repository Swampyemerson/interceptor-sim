#!/usr/bin/env python3
"""THE INSTRUMENT AND THE FLYING SYSTEM MUST AGREE ON THE EXPOSURE.

THE DEFECT (2026-07-26). `scripts/seeker/pi_capture.py` -- the tripod instrument
the ~$740 decode-envelope verdict comes from -- captures with AE OFF at the <=1 ms
spec and VERIFIES the applied value. `flight/deploy/seeker_loop.PicameraSource`,
the loop that actually flies against that verdict, set NO camera controls at all,
so the flight camera ran picamera2's auto-exposure. Overcast light picks 8-20 ms;
motion blur at 9 m/s closing is then 8-20x the measured condition and detection
dies at ranges the tripod curve certified.

This is ARM-ASYMMETRIC in the worst way: the measuring instrument and the
operational system disagree on the one knob the sensor was chosen for, and
NEITHER the offline tests nor the tripod data itself can see it -- the tripod
session is captured by the instrument, so it always looks correct.

Pinned here:
  1. the two files carry the SAME spec number (duplicated because flight/ cannot
     import scripts/, so the equality needs a test rather than an import);
  2. the flight path REQUESTS a manual exposure at that spec;
  3. gain is left AUTO -- pinning it as well would be a new failure mode (a
     flight in different light than the bench gets a black or blown frame);
  4. an unreported applied exposure reads UNVERIFIED, never OK.

Run: .venv/bin/python -m pytest tests/test_exposure_spec_transfers.py -v
"""
import importlib.util
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from flight.deploy import seeker_loop as SL                       # noqa: E402


def _load_pi_capture():
    """flight/ cannot `import scripts.seeker.pi_capture` (no package), so load it
    by path -- which is exactly why the constant needs pinning by a test."""
    path = os.path.join(REPO_ROOT, "scripts", "seeker", "pi_capture.py")
    spec = importlib.util.spec_from_file_location("_pi_capture_for_test", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_flight_loop_and_the_instrument_share_one_exposure_spec():
    pc = _load_pi_capture()
    assert SL.EXPOSURE_SPEC_US == pc.EXPOSURE_SPEC_US == 1000, (
        "the tripod curve is measured at pi_capture.EXPOSURE_SPEC_US and flown "
        "against at seeker_loop.EXPOSURE_SPEC_US -- if they drift, the flight "
        "runs an optical condition the money-gate curve never covered")


def test_the_flight_camera_requests_a_manual_exposure_at_the_spec():
    """PRE-FIX this dict did not exist: no controls were sent at all."""
    # newer libcamera stack: split mode controls
    c = SL.picam_exposure_controls({"ExposureTimeMode": 1, "AnalogueGainMode": 1},
                                   SL.EXPOSURE_SPEC_US)
    assert c["ExposureTime"] == SL.EXPOSURE_SPEC_US
    assert c["ExposureTimeMode"] == 1, "manual exposure"
    # older stack without the split controls: AeEnable fallback
    c_old = SL.picam_exposure_controls({}, SL.EXPOSURE_SPEC_US)
    assert c_old["AeEnable"] is False
    assert c_old["ExposureTime"] == SL.EXPOSURE_SPEC_US
    c_none = SL.picam_exposure_controls(None, SL.EXPOSURE_SPEC_US)
    assert c_none == c_old, "an unknown control map must still pin exposure"


def test_gain_is_left_automatic_on_the_modern_control_path():
    """Deliberate: AeEnable=False pins gain too, and a flight in different light
    than the bench session would then get a black or blown frame with no
    adaptation -- a failure the auto-everything code did not have."""
    c = SL.picam_exposure_controls({"ExposureTimeMode": 1, "AnalogueGainMode": 1},
                                   SL.EXPOSURE_SPEC_US)
    assert c["AnalogueGainMode"] == 0, "gain AUTO"
    assert "AnalogueGain" not in c, "gain must not be pinned to a fixed value"


@pytest.mark.parametrize("applied,expect", [
    (900.0, "OK"),
    (5000.0, "WARNING"),
    (None, "UNVERIFIED"),
])
def test_the_applied_exposure_is_graded_fail_closed(applied, expect):
    assert expect in SL.picam_exposure_verdict(applied)
