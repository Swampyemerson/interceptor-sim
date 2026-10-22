# tests/fixtures — the small TRACKED fixtures the offline suite needs on a clean clone

Everything under `logs/`, `scripts/seeker/data/` and `scripts/seeker/weights/` is
gitignored (evidence + 18.5 GB of media/weights, see `docs/license_notice_weights.md`).
Tests that reached into those paths therefore passed on this machine and **errored on
every clean clone** — which is what kept GitHub Actions red on 73/73 runs from the
first push (2026-07-21) until 2026-07-25. This directory holds the minimum tracked
data that makes the offline suite reproducible from `git clone` alone.

## `rig_capture_min/` — REMOVED 2026-10-22

Was the fallback for `tests/test_ground_station.py`, which commit 3880153 retired to
`retired/tests/`. The fixture had no remaining live consumer and was deleted in the
review-3 sweep (recoverable from git history if the ground-station test is ever revived).

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
