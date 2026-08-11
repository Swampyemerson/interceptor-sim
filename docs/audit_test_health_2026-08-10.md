# Test-suite health audit — 2026-08-10

Independent read-only audit of the offline test suite, run overnight. Recorded here
because the findings would otherwise have existed only in an agent transcript — which
is the same single-copy failure the skr-07 evidence rescue fixed the same day.

**Scope:** offline suite only. No Gazebo, no PX4, no Monte-Carlo — the sim is serialized
and the builder was asleep.

---

## Headline

The suite is **substantially healthy and unusually well designed**: 739 passing tests,
deterministic across repeated runs, ~35 s end to end, **zero** `assert True`, zero
vacuously-true `all()`/`any()` assertions, a real producer-written-fixture contract-test
culture, working `--self-test` gating with a coverage-drift guard, and a model-blocking
hook that does exactly what its docs claim.

**The problem was never in the assertions. It was in the reporting layer around them.**

---

## What was found, and what was done

| # | Finding | Severity | Status |
|---|---|---|---|
| F1 | `run_tests.sh` RED at HEAD for 12 days / 9 commits | **CRITICAL** | ✅ FIXED |
| F2 | Under-powered range calibration consumed as calibrated | **HIGH** | ✅ FIXED |
| F3 | Flight-CSV completeness guard can verify zero units and pass | HIGH | ⬜ OPEN |
| F4 | Corrupt calibration reported identically to a missing one | MEDIUM | ✅ FIXED |
| F5 | `test_camera_fps_pinned` could never run (no `build_argparser`) | MEDIUM | ✅ FIXED |
| F6 | `test_decode_fps_bound_contract` pointed at a never-existent path | MEDIUM | ✅ FIXED |
| F7 | Stage 1 had no skip enforcement (the mechanism that hid F5/F6/F8) | MEDIUM | ✅ FIXED |
| F8 | Three tests pass locally, silently skip in CI (untracked fixture) | LOW/MED | ⬜ OPEN |

### F1 — the gate was not gating (fixed, `dd0f6d4`)

`flight/tests/test_seeker_loop_coast.py:373` asserted the DEFAULT config accepts a 1.50 s
stale own-state. ADR-0093 (`4df285f`, 2026-07-29) turned that gate ON at 0.25 s.

The instructive part is *how it survived*. ADR-0093's commit message certifies
**"Suite 519 passed."** Measured: 519 is *exactly* `pytest tests/`. The 173-test
`flight/tests/` half — which `run_tests.sh` stage 1 also runs, and where the
contradicting test lives — **was never executed**. The commit certified a suite it had
not run, and nine commits landed on top.

Two tests asserted opposite things about one constant:

- `tests/test_own_state_age_gate.py:46` — default `is not None`
- `flight/tests/test_seeker_loop_coast.py:373` — default accepts a stale sample

The **code was right**; the test was the stale artefact, and its docstring
("the AGE gate is off by default on purpose — no measured own-state stall latency exists
yet") was a claim brn-05 had already falsified on hardware.

### F2 — an unvetted span could steer the aircraft (fixed, `dd0f6d4`)

Seam: `scripts/seeker/calibrate_range.py` (producer) → `flight/deploy/seeker_loop.py:resolve_span`
(consumer). **No contract test existed on this seam at all.**

The producer grades its own fit — `return 0 if n_kept >= 20 else 2` — but writes the
sidecar *before* that return, and the consumer never read `n_kept`. So a 3-sample fit the
producer itself graded a FAILURE arrived indistinguishable from a 57-sample fit and was
consumed as `calibrated=True`.

This is not cosmetic. `span_m_eff` sets `range = fx*span/box_width_px`, and range
multiplies `Vc` into the pro-nav command `a_cmd = N·Vc·λ̇`, as well as keying every
range threshold (closing-speed throttle, terminal freeze, breakoff arm, hard floor).

Fixed on both sides, plus `tests/test_range_calib_contract.py` (8 tests, mutant-verified —
restoring the defect kills 4).

### F3 — the one true vacuous verdict (OPEN)

`tests/test_flight_csv_contract.py:118` calls itself a *"mechanical completeness guard"*.
It regex-sweeps three consumers for row reads and asserts `not unknown` — but **never
asserts the sweep found anything**. Mutant-proven: renaming the row variable `r` → `rec`
(a plausible refactor the regex does not match) drops the swept count from 8 to **0** and
the assertion still passes, having checked nothing.

The repo already knows this shape — the sibling `tests/test_ci_gz_deselect_list.py`
contains a test literally named `test_the_measurement_is_not_vacuous`. Fix is one line per
file: `assert len(reads) >= <floor>`.

### F7 — skip-blindness (fixed, `dd0f6d4`)

Stage 2 failed on any skip; stage 1 — 692 of 721 tests — had no `-rs` and no check. That
asymmetry is precisely what let F5, F6 and F8 sit invisible. Stage 1 now requires every
skip to be declared in `ALLOWED_SKIPS` with a human-written reason. Mutant-verified.

### F8 — CI-only skips (OPEN)

`tests/test_t18_scaffold.py`'s `rig` fixture needs
`logs/rig_captures/full_sweep_20260709T015530Z/capture_meta.json`, which is **untracked**.
Three of four tests take that fixture, including
`test_sigma_R_scaling_reproduces_analytic_row`. Locally: 4 passed. On a clean clone or in
CI: 3 skip silently. Now *visible* thanks to F7's enforcement, but the fixture still needs
either committing or an explicit allowlist entry.

*(Contrast, and it is the right pattern: `test_antimirage_pairing.py`'s published-headline
check depends on `logs/mc_loftdive_armA_line9_s123.csv`, which **is** tracked — so it
really runs in CI.)*

---

## Negative results, stated deliberately

A manufactured finding is worse than none, so these are recorded as clean:

- `assert True` / `assert 1` / `assert not False`: **zero** occurrences.
- AST scan for `assert all(<filtered comprehension>)` — vacuous on an empty filter: **0 hits**.
- AST scan for asserts inside loops over possibly-empty iterables: 8 hits, **all cleared**
  (6 iterate module-level constants measured non-empty at runtime; 2 already carry count floors).
- **No flaky tests.** Slowest single test 0.96 s; nothing else above 1 s. No `sleep`, no
  wall-clock reads, no network. Both RNG uses explicitly seeded. Deterministic across
  three full runs.
- **The Opus-4.8 block works as documented.** `tests/test_block_opus48_hook.py`: 40 passed,
  and exercised live — `opus` DENY, `claude-opus-4-8` DENY, `claude-opus-5` ALLOW, no-model
  ALLOW. `.claude/settings.json` wires it on `PreToolUse` matching `Agent|Workflow`.

---

## Uncovered producer→consumer seams (for later)

| Seam | Note |
|---|---|
| `calibrate_range.py` → `resolve_span` | **Was F2. Now covered.** |
| `multirange_capture` → `multirange_nn_detection` → `multirange_sigma_fit` | 3-stage chain producing σ_R (the range noise model). Zero test references. |
| `mc_analyze.py` | An uncovered *consumer* of a covered producer — it is absent from `test_flight_csv_contract.py`'s enumerated consumer list. |
| `tag_decode_cost.py`, `phantom_competition_replay.py`, `pi_fps_soak.py` | No tests; `pi_fps_soak`'s number reaches `tripod_score` via a CLI flag (no file boundary), so lower priority. |

`tests/test_parse_flight_log_contract.py` is the model to copy: it walks the real state
machine with the producer's own writer and mutation-tests added/dropped columns on both sides.

---

## One config observation — for the builder, not a defect

`~/.claude/settings.json` has `"fastMode": true` persisted globally while `"model"` is
`claude-fable-5`. CLAUDE.md §5 sanctions fast mode for *in-person bench sessions on Opus 5*
and says the head should return to Fable when the bench session ends. A globally-persisted
`fastMode` is the "left a bench-tuned session pinned for a design decision" drift that
bullet warns about.

**Not changed** — it is outside the project directory and is the builder's call.
