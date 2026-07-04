# Progress — milestone roll-up

| Milestone | What it proves | Gate | Status |
|---|---|---|---|
| M0 Toolchain | PX4 SITL + Gazebo boot headless; MAVSDK arms/takes off/lands | `scripts/check_m0.sh` exits 0 | ✅ 2026-07-04 — exit 0; peak alt 1.656 m (bar: ≥1.6 m), landed at 0.014 m; verifier-confirmed (`logs/m0_takeoff_20260704T225304Z.csv`) |
| M1 Camera | Frames from `gz_x500_mono_cam` into Python via gz-transport | N frames at expected resolution | ⬜ pending |
| M2 AprilTag | Tag detected live; bearing/rel-position vs ground truth | detection rate + pose error logged | ⬜ pending |
| M3 Static intercept | Close on static tag, hold 2 m standoff | final standoff error < 0.5 m | ⬜ pending |
| M4 Moving target | Pursuit vs pro-nav on moving tag | < 1 m closest approach @ ≥ 2 m/s | ⬜ pending |
| M5 Polish | Monte-Carlo, plots, README, demo GIF | plots + README complete | ⬜ pending |

Details per milestone live in `NEXT.md` (current work) and `docs/decisions.md` (choices).
