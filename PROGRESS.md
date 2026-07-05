# Progress — milestone roll-up

| Milestone | What it proves | Gate | Status |
|---|---|---|---|
| M0 Toolchain | PX4 SITL + Gazebo boot headless; MAVSDK arms/takes off/lands | `scripts/check_m0.sh` exits 0 | ✅ 2026-07-04 — exit 0; peak alt 1.656 m (bar: ≥1.6 m), landed at 0.014 m; verifier-confirmed (`logs/m0_takeoff_20260704T225304Z.csv`) |
| M1 Camera | Frames from `gz_x500_mono_cam` into Python via gz-transport | N frames at expected resolution | ✅ 2026-07-04 — exit 0; 10/10 frames @ 1280×960 RGB, real content (stddev 13.2), verifier-confirmed (`logs/m1_frames_20260704T230509Z/`) |
| M2 AprilTag | Tag detected live; bearing/rel-position vs ground truth | detection rate + pose error logged | ✅ 2026-07-04 — exit 0; detection_rate 1.000, mean err_norm 0.0861 m (bar: ≤0.25 m), max 0.0866 m, mean range 4.888 m (`logs/m2_detect_20260704T233941Z.csv`) |
| M3 Static intercept | Close on static tag, hold 2 m standoff | final standoff error < 0.5 m | ✅ 2026-07-05 — exit 0; final standoff err 0.018 m / 0.035 m across two verifier runs (bar: <0.5 m), detection coverage 1.000, settle ~10 s, no overshoot; verifier-confirmed (`logs/m3_intercept_20260705T000619Z.csv`, `...000818Z.csv`) |
| M4 Moving target | Pursuit vs pro-nav on moving tag | < 1 m closest approach @ ≥ 2 m/s | ⬜ pending |
| M5 Polish | Monte-Carlo, plots, README, demo GIF | plots + README complete | ⬜ pending |

Details per milestone live in `NEXT.md` (current work) and `docs/decisions.md` (choices).
