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

Moved 2026-10-14 (follow-up scan: stragglers of the 2026-09-23 clusters —
companion docs and figures whose tooling had already moved; every file was
reference-checked repo-wide and had no live consumer):

- **S2 ground-stereo stragglers**: `docs/ground_rig_sweep_plan.md` (the
  Option C sweep plan, written against the retired `s2_cue_mock.py`),
  `docs/ground_rig_division_of_labor.md`, `docs/t18_nn_validation_notes.md`
  (companion to the retired `t18_nn_sigma_validation.py`), and the six
  stereo/rig-geometry figures from `plots/` → `retired/plots/` (referenced
  only by retired docs/scripts; pointers inside those updated).
- **Cue-era tilt prereg**: `docs/adaptive_tilt_prereg.md` (2026-07-10 arms,
  outcome in the ADR log; the live mount lever is the fixed up-tilt A/B).
- **Sim-phase analysis note**: `docs/pk_vs_radius_note.md` (G15 chosen-radius
  sensitivity; superseded by the real build's measured 0.35 m contact
  envelope; zero inbound references).

Known dormant coupling, NOT moved (flagged for a future deletion decision):
`scripts/m4_intercept.py` CUE_WAIT/S2 mode still resolves
`scripts/s2_cue_mock.py` (line ~483) and `scripts/mc_batch.sh` still draws
`--cue-seed` for it — dead-but-wired legacy paths that would fail if an
S2-era mode were re-run; left in place because ripping sim-driver plumbing
is beyond a housekeeping move.
