# Decisions log (ADR-lite)

Format: context / options / decision / why / date. Councils noted where convened.

---

## ADR-0001 — PX4-Autopilot lives outside the project repo
- **Context:** PX4 is a ~2 GB clone with its own git history and build tree.
- **Options:** (a) clone inside `interceptor-sim/` and gitignore it; (b) clone to `~/PX4-Autopilot`.
- **Decision:** (b) `~/PX4-Autopilot`.
- **Why:** keeps this portfolio repo small and clean; no risk of accidentally committing build artifacts; standard PX4 dev layout.
- **Date:** 2026-07-04. (No council — reversible.)

## ADR-0004 — M0 gate waits on PX4's boot-complete line, not "Ready for takeoff!"
- **Context:** M0 gate deadlocked twice: gz_x500 airframes set `NAV_DLL_ACT=2` (datalink-loss failsafe), making "No connection to the GCS" a blocking preflight failure — PX4 only prints "Ready for takeoff!" after a GCS/MAVSDK link exists, but our gate started MAVSDK only after seeing that line. Root-caused by the verifier in PX4 source (Commander.cpp / rcAndDataLinkCheck.cpp / airframe 4001).
- **Options:** (a) set `NAV_DLL_ACT 0` in SITL; (b) gate on "Startup script returned successfully" and let the MAVSDK connection clear the preflight check.
- **Decision:** (b). Flight-safety parameters stay at airframe defaults; tests adapt to the system, not the reverse.
- **Why:** keeping the failsafe honest preserves the "no cheating" credibility of the sim (GOALS.md); the boot-complete line is unconditional and appeared in every log. Verified: after the fix, "Ready for takeoff!" appears immediately after MAVSDK's connection lands — the causal story confirmed end-to-end.
- **Date:** 2026-07-04.

## ADR-0003 — AprilTag library: pupil-apriltags (COUNCIL, unanimous 3-0)
- **Context:** The tag detector feeds every milestone from M2 on (detection → pose → guidance). Swapping later means re-validating the whole perception chain — a one-way door per CLAUDE.md, so a 3-member Sonnet council was convened with an identical brief.
- **Options:** (A) pupil-apriltags, (B) dt-apriltags, (C) cv2.aruco + solvePnP with the 36h11 dictionary.
- **Decision:** (A) **pupil-apriltags**, version pinned in requirements.
- **Why (council synthesis, all three voted A at high confidence):**
  - Actively maintained; 1.0.4.post11 (2025-04) ships cp312 manylinux wheels — one member verified the wheel downloads cleanly on this exact machine.
  - Wraps the genuine AprilTag 3 reference algorithm (not ArUco's reimplementation) and exposes `estimate_tag_pose` returning `pose_R`/`pose_t`/`pose_err` directly — no hand-rolled solvePnP, one less coordinate-convention surface to get wrong, and `pose_err` gives us a loggable per-detection accuracy signal.
  - (B) is dead: last release 2021, no Python 3.12 wheels. (C) works but adds pose-ambiguity pitfalls (`SOLVEPNP_IPPE_SQUARE` needed) and more of our own geometry code.
- **Risks & fallbacks (logged from council dissent/caveats):** small maintainer team — pin the version; if wheels ever break, the `pyapriltags` fork is the drop-in fallback. If M2 shows unacceptable pose jitter at oblique angles (inherent monocular planar-tag ambiguity), revisit (C) cv2.aruco + `SOLVEPNP_IPPE_SQUARE` as the swap — the detector interface will be isolated behind one module to keep that cheap.
- **Date:** 2026-07-04.

## ADR-0002 — Environment baseline: WSL2 + RTX 4070, headless by default
- **Context:** KICKOFF-PROMPT.md described a "fresh VM, no GPU"; CLAUDE.md (newer) and `nvidia-smi` confirm WSL2 with an RTX 4070.
- **Decision:** Trust the live environment: GPU available for camera rendering (M1+) and the demo; all automated/batch runs stay `HEADLESS=1` for speed and reproducibility.
- **Why:** headless runs are reproducible and faster; the GPU only matters where pixels matter.
- **Date:** 2026-07-04. (No council — observation, not a fork.)
