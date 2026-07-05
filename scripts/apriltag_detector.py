#!/usr/bin/env python3
"""Portable AprilTag Detector import: works on dev x86 AND the drone's ARM64.

The perception stack uses one class, `Detector`, from the pupil-apriltags
family. The catch (ADR-0012, confirmed): `pupil-apriltags` ships NO Linux
aarch64 wheel (PyPI is x86_64 + macOS arm64 only; piwheels only 32-bit), and
the deployment companion (Raspberry Pi 5, aarch64) needs one. `pyapriltags`
is an API-compatible fork that DOES ship aarch64 wheels -- ADR-0003 already
pre-registered it as the drop-in fallback.

So: import `Detector` from here, not from either package directly. This tries
pupil-apriltags first (what the dev machine + all the sim gates use), then
falls back to pyapriltags (the drone). Both expose the same
`Detector(families=..., nthreads=..., quad_decimate=...)` constructor and the
same `detect(gray, estimate_tag_pose=, camera_params=, tag_size=)` returning
detections with `.pose_t`, `.pose_err`, `.decision_margin`, `.tag_id`,
`.corners` -- so nothing downstream changes.

`DETECTOR_BACKEND` records which one loaded (logged at flight start so a run's
log says exactly which detector produced its numbers -- the two are
API-compatible but not bit-identical, worth recording).
"""

try:
    from pupil_apriltags import Detector  # noqa: F401
    DETECTOR_BACKEND = "pupil_apriltags"
except ImportError:  # pragma: no cover - exercised only on aarch64/hardware
    try:
        from pyapriltags import Detector  # noqa: F401
        DETECTOR_BACKEND = "pyapriltags"
    except ImportError as exc:  # pragma: no cover
        raise ImportError(
            "no AprilTag detector available: install 'pupil-apriltags' "
            "(x86_64/dev) or 'pyapriltags' (aarch64/Raspberry Pi). See "
            "ADR-0012 / requirements.txt."
        ) from exc
