"""flight.deploy -- the REAL-HARDWARE terminal seeker loop.

This package holds the code the physical interceptor's Pi 5 will run: the same
camera-only pro-nav terminal the Gazebo sim validates, but wired to REAL I/O
(Picamera2 -> ONNX detector -> flight/ guidance core -> MAVSDK/PX4 offboard).

It is a THIN driver around the portable `flight/` core (geometry, camera,
estimator, guidance) -- all the honesty-audited, test-pinned math lives there and
runs bit-for-bit identically in the sim and on the vehicle. `seeker_loop.py`
only adds the parts that are inherently hardware-specific: grabbing a frame from
a real camera, and pushing a velocity/attitude setpoint over MAVLink.

Honesty boundary (CLAUDE.md / ADR-0010): the loop reads ONLY camera pixels and
the flight controller's OWN-state EKF (attitude quaternion + position). It never
reads any ground truth -- there is no ground truth on real hardware to read.

WHAT PROVES IT, and exactly what that proves: `python -m flight.deploy.real_flight
--audit` walks the SYNTAX TREE of the whole real-build import closure
(real_flight.py, seeker_loop.py and flight/{camera,geometry,estimator,guidance,
range_fusion,terminal_coast,fov_guidance}.py -- `real_flight._AUDITED_MODULES`)
and fails on any `gt_`/`ground_truth` identifier or simulator import, plus the
one-write dash-heading latch. It is a NAMING/IMPORT check, not information-flow
proof: it cannot see ground truth handed in by a caller under a clean name. Its
calibration -- that it actually FIRES on a planted read -- is pinned by
flight/tests/test_real_flight.py. Before 2026-07-25 the audit covered
real_flight.py ALONE and forbade only the `gt_` prefix, so this paragraph claimed
more than the check delivered; that is the gap it closed. See the no-cheat note
at the top of seeker_loop.py.
"""
