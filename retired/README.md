# retired/ — superseded work, preserved

Nothing in this project is deleted; superseded work is RELOCATED here so the
live tree reads current. Everything below was green and load-bearing in its
day; each cluster's decision record stays in `docs/decisions_archive.md` /
`docs/project_state.json` (graveyard + superseded stages).

Moved 2026-09-23 (inventory: a full reference-topology sweep; tests that
exercised only retired code moved with it, so `scripts/run_tests.sh` no
longer runs them — they were green at retirement):

- **S2 ground-stereo cue architecture** (`scripts/check_s1,s2,t16,t17,t19*`,
  `stereo_model.py`, `s2_cue_mock.py`, the multirange/T18 sigma tooling,
  `ground_station_pkg/`, their tests, `docs/stereo_design.md` and friends).
  The contract marks this stage superseded by the no-datalink coded dash;
  the CURRENT cue work is `flight/cue_relay.py` (GPS cue latched at trigger),
  which is unrelated to this rig.
- **2026-07 demo/T25 video pipeline** (`build_demo.py`, `render_hud.py`,
  `scripts/video/` T25 tooling, storyboard docs). Superseded by the
  2026-09-22 LinkedIn media pack (`scripts/linkedin_media_figures.py`,
  `scripts/video/linkedin_hero_cut.py`). `scripts/demo_capture_frames.py`
  stays live (still the frame recorder for new videos).
- **July one-shot forensics** (`deep_dive.py`, `q6_experiment.py`,
  `quantify.py`, `terminal_forensics.py`) — superseded by the forensics
  wired into `scripts/run_tests.sh` stage 4.
- **Detect-then-track / jam-recovery arms** (`analyze_track_ab.py`,
  `check_jam_mc.py`, `mc_jam_arm.sh`, `mc_t21_trackgate_jink16.sh`,
  `stage2_tilt_recovery_arm.sh`) — closed NULLs / retired concepts
  (ADR-0076 add #2/#8, ADR-0101).
- **Fixed up-tilt A/B harness** (`uptilt_*`, mount variants up00/25/35).
  `scripts/experiments/uptilt_mounts/up10` stays live (still referenced by
  the current flight-plan arms).
- **Sim-phase-closed planning docs** (m5 batch plan, sim-to-real pivot
  plans, audit punch-lists, old preregs). Live status is
  `docs/project_state.json`; decisions live in the ADR logs.
- `scripts/experiments/attic/` — the previous generation of this same
  folder, absorbed wholesale.

Do not resurrect anything here without new evidence — the graveyard and
contradiction ledger rules apply (`CLAUDE.md`).
