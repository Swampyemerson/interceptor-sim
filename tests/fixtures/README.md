# tests/fixtures — the small TRACKED fixtures the offline suite needs on a clean clone

Everything under `logs/`, `scripts/seeker/data/` and `scripts/seeker/weights/` is
gitignored (evidence + 18.5 GB of media/weights, see `docs/license_notice_weights.md`).
Tests that reached into those paths therefore passed on this machine and **errored on
every clean clone** — which is what kept GitHub Actions red on 73/73 runs from the
first push (2026-07-21) until 2026-07-25. This directory holds the minimum tracked
data that makes the offline suite reproducible from `git clone` alone.

## `rig_capture_min/` — 200 KB (fallback for `tests/test_ground_station.py`)

A trimmed copy of `logs/rig_captures/full_sweep_20260709T015530Z/` (the T16 ground
stereo-rig sweep, 3.1 MB / 56 stereo pairs):

| file | why it is here |
|---|---|
| `capture_meta.json` | `RigConfig.from_capture_meta` — baseline, intrinsics, true pose |
| `index.csv` | `load_capture` — per-frame `seq` / `t_sim_nominal` / gt target rows |
| `centroids.csv`, `centroid_cache_meta.json` | ADR-0051 centroid-cache replay cases |
| `left/0000{5,6,7}.png`, `right/0000{5,6,7}.png` | the only real frames a non-onnx case **stats** (`test_datum_bias_shifts_triangulated_position_through_process_frame`; pixels are never opened — centroids are synthesized) |

`tests/test_ground_station.py` prefers the real on-disk capture when present and falls
back here otherwise, so a local run still exercises the full 56-frame sweep. The two
`onnxruntime`-gated cases need frames this subset does not carry (seq 10-15); they
`importorskip` first, and if they are ever reached without the full capture they
**fail loudly** rather than skip.

## `apriltag_sim_frames/` — 772 KB / 32 frames (no-hardware fixture for the field pack)

`scripts/field/01_camera_live_check.sh` defaults its NO-HARDWARE replay to
`scripts/seeker/data/autolabel_sim_20260721T032324Z/images`, and its comment claimed
those frames were "shipped with the repo". They are not — `scripts/seeker/data/` is
gitignored — so on a clean clone the script exited 2 (USAGE: replay dir not found),
`02_apriltag_desk_check.sh` fell through to the same missing path, and
`scripts/field/selftest.sh` reported 2 failures. That is the **third** independent
cause of the 73/73 red CI streak, and it lives in CI stage 4.

This is a 32-frame subset of that same sim capture, sampled evenly across it
(ranges 1.9 m → 29.9 m, 1280×960, each really containing a `tag36h11` at the sim
tag's 0.5 m edge). Both scripts prefer the full 252-frame set when it is on disk and
fall back here otherwise. 32 clears `01`'s default `--frames 30`.

## `field_selftest_frame.png` — 28 KB (fixture for `scripts/field/selftest.sh`)

`scripts/field/selftest.sh` check 5c replays a one-image directory through
`01_camera_live_check.sh --replay --frames 30` to prove a short session FAILS. It
looked for any `*.png` under `scripts/seeker/data/` — a gitignored dataset dir — so on
a clean clone the check reported `no fixture PNG ... cannot test the short-session
branch` and the pack exited 1. This is the SECOND cause that kept CI red: it is
venv-independent and sits in CI stage 4, which was added 2026-07-25 to close the
review-2 "green != ran" finding. One tracked frame (the seq-5 left image from the
same rig capture) makes the check run everywhere.
