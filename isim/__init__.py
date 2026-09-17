"""isim -- fast, deterministic, headless engagement simulator (ADR-0102).

No PX4, no Gazebo, no rendering. Plan of record: docs/new_sim_plan.md.
Frames: world NED (north, east, down), body FRD, camera OpenCV (z fwd, x right,
y down). SI units; angles in radians unless a name ends in _deg.
"""
