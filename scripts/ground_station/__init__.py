"""T18 (Phase 2, docs/phase2_sim_to_real_plan.md Track A) -- the ground-side
pipeline package: real stereo triangulation (`triangulate.py`) + a track
filter (`track_filter.py`) that will eventually sit behind a real
`ground_station.py` process (T19). Nothing in this package holds a gz
subscription or imports gz/MAVSDK -- it is pure Python (numpy only) so it
runs offline against captured frames/labels, per ADR-0046 decision item 2
("§5.6 #2: `--datum-bias-m`/`--assumed-rig-pose` re-triangulates captured
pairs under perturbed extrinsics (CPU-only, no re-render)").
"""
