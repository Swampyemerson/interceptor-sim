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
reads any ground truth -- there is no gt_* on real hardware to read. See the
no-cheat note at the top of seeker_loop.py.
"""
