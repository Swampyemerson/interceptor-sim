"""flight/ — the portable guidance+vision CORE of the interceptor.

This package holds the pieces that run IDENTICALLY in Gazebo SITL and on the
real Pixhawk-6C/PX4 + Pi5 interceptor (docs/hardware_order_list.md): the frame
math, the camera-LOS derotation, the coded-dash aim, and the pro-nav terminal
law. It has NO Gazebo / gz-transport / ground-truth / sim-cue imports -- only
`math` and own-state (EKF attitude + position) inputs the flight controller
supplies over MAVLink. That boundary is the whole point of the real-build pivot
(memory [[real-build-pivot]], ADR-0076 add #18): the sim was validating a
cue-guided architecture the hardware won't have; this core is the architecture
it WILL have -- open-loop coded dash -> camera-only pro-nav terminal.

Extracted VERBATIM from scripts/m4_intercept.py (the honesty-audited, test-pinned
originals) so the numbers transfer bit-for-bit; m4_intercept.py will import from
here rather than keep private copies. See flight/tests/ for the pinning tests.
"""
