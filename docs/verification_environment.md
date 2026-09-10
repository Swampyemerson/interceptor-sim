# What each environment can verify — and what it cannot

> **Why this exists.** This repo is worked on from three different places: the dev
> machine (WSL2 + GPU, PX4 + Gazebo, all the gitignored data), GitHub Actions, and
> ephemeral cloud sessions with a fresh clone and no sim. Each verifies a *different
> subset*, and a claim of "green" means nothing until you know which subset produced
> it. This project has already been bitten twice by exactly that: a commit certified
> "Suite 519 passed" while never running the 173-test portable-core half (12 days
> red, 9 commits), and CI failed 73 of its first 73 runs while a project surface said
> it was green.
>
> **Read this before trusting a green run, and before asking why something was not
> tested.** Written 2026-09-10 from a cloud session that hit every limit below.

---

## The three environments

| | Dev machine | GitHub Actions | Cloud session / fresh clone |
|---|---|---|---|
| PX4 SITL + Gazebo | ✅ | ❌ | ❌ |
| gz-transport / MAVSDK bindings | ✅ (apt) | ⚠️ best-effort apt, fallback deselects | ❌ not pip-installable |
| GPU render | ✅ | ❌ | ❌ |
| Real hardware (Pi 5, Pixhawk, camera) | ⚠️ when connected | ❌ | ❌ |
| Gitignored per-tick flight archive (168 flights) | ✅ | ❌ | ❌ |
| Gitignored deployed weights + T16 rig capture | ✅ | ❌ | ❌ |
| Committed per-tick CSVs (`git ls-files 'logs/*.csv'`) | ✅ | ✅ | ✅ |
| Offline pytest suite | ✅ full | ✅ minus the gz files | ✅ minus the gz files |
| Drift checks (`render_dashboard/mbse --check`) | ✅ | ✅ | ✅ (stdlib only) |
| AST honesty audit (`real_flight --audit`) | ✅ | ✅ | ✅ |
| Milestone gates (`check_m*.sh`, `check_t*.sh`) | ✅ | ❌ needs the sim | ❌ |
| `scripts/field/selftest.sh` (needs an `ssh` client) | ✅ | ✅ | ❌ 6 failures, correctly |
| Monte-Carlo arms (`mc_batch.sh`) | ✅ idle-load only | ❌ | ❌ |

---

## Setting up a fresh clone to run what it can

`requirements.txt` is complete and correct — the earlier dependency-rot finding is
closed. On a bare cloud container the offline suite needs, at minimum:

```
pip install pytest numpy matplotlib opencv-python-headless \
            pyulog==1.2.3 pymavlink==2.4.49 pupil-apriltags==1.0.4.post11
```

Then run the suite with the CI fallback deselect list applied, because the ten
gz-importing files cannot be collected without the bindings (the list is
machine-checked by `tests/test_ci_gz_deselect_list.py`; copy it out of
`.github/workflows/ci.yml`, do not retype it).

**`scripts/field/selftest.sh` will still report 6 failures on a bare container, and
that is the pack working correctly.** It shells out to `05_pi_link_check.sh`, which
needs an `ssh` client to reach the Pi and exits **2 (USAGE)** with the remedy printed
when there is none. The selftest expects 0 or 3 there, so a machine without `ssh`
reads as a failure. Verified pre-existing 2026-09-10: identical 6 failures with the
field pack untouched and with the original `field_score.py` restored. Installing
`ssh` is an apt change outside the project directory, which CLAUDE.md says to ask
about first — so on a cloud session, **read past this pack** rather than treating it
as a regression. It is the one stage-4 item a fresh container cannot clear.

`scripts/run_tests.sh` itself **can** now be run on a clean clone: its stage-1 skip
gate declares the seven clean-clone skips as of 2026-09-10 (test-health audit F8).
Stages 1, 3 and 4 will run; stage 2 will fail loudly because `.venv-seeker` does not
exist, and that is by design — a missing ONNX-parity environment is a real gap, not a
skip.

---

## What a cloud session genuinely CANNOT do, and what to do instead

Every item here is a real limit, not a missing effort. Each names the substitute that
*is* available, because "cannot verify" is only useful with a next step attached.

### 1. Fly anything

No sim, no autopilot, no hardware. **No arm, gate or Monte-Carlo batch can be run**,
so no guidance change made from a cloud session is validated — in sim or otherwise.

*Substitute:* ship guidance changes **default-OFF and byte-identical**, prove the
byte-identity with a test that compares the whole setpoint stream tick by tick, and
**pre-register** the arm before it flies. That is what `flight/fov_guidance.py`,
`range_fusion.py`, `terminal_coast.py` and (2026-09-10) the dash altitude trim all do.
Their module docstrings say `NOT SIM-VALIDATED` for this reason. Do not quietly
promote one of them to a default.

### 2. Reproduce any number from the 168-flight per-tick archive

That archive is gitignored and dev-machine-only. Anything scored from it —
ADR-0095's −0.374 m vertical, the 3/16 and 5/16 counts inside the ram radius, the
rescore acceptance gate — cannot be recomputed from a fresh clone.

*Substitute:* the **committed** per-tick CSVs (`git ls-files 'logs/*.csv'`, 144 files,
of which 24 carry a full `phase` + `gt_range` + `gt_cam_z` + `alt_m` schema). They are
a different, cue-era fleet, so they test **mechanisms** and not **magnitudes**. Every
tool in `scripts/forensics/` that reads them says so in its docstring and prints the
scope on its verdict line. Two of them take `--phase CODED_DASH` specifically so the
same measurement can be re-run on the right fleet on the dev machine.

### 3. Verify the deployed model, or anything needing the weights

`weights/DEPLOYED.sha256` pins them but the files are never committed
(`docs/license_notice_weights.md`). `test_deployed_weights` therefore skips, and says
"integrity NOT verified on this machine".

*Substitute:* none from a cloud session. Run it where the weights live.

### 4. Measure a frame rate, a latency or a temperature

Emulation proves a stack *runs*, never how fast — the point already settled for the
Pi 5 (`pi5-emulation-gap`, resolved by buying the board). The same holds here.

*Substitute:* nothing. Any fps, latency or thermal number must come from the real
board, and `scripts/seeker/tripod_score.py` correctly refuses to invent one.

### 5. Check the pitch-to-altitude-drift link

The committed CSVs carry **no attitude columns** — deep-audit DEEP-R2, still open. So
reasoning that runs through body pitch (for example "the dash pitches nose-down, so
vertical thrust falls, so altitude drifts") is *inferred*, not measured, from this
data.

*Substitute:* add the quaternion/roll/pitch columns to the flight CSV writer on the
dev machine (DEEP-R2's own recommendation) and re-run
`scripts/forensics/vertical_miss_anatomy.py`. Until then, treat the mechanism as a
hypothesis with a measured *correlate*, which is what that tool prints.

---

## Standing rule this all serves

**Green means RAN.** A skipped test protects nothing, an unrun self-test is not
evidence, and a verdict computed on zero units is UNCERTAIN and never a PASS
(`docs/error_handling_policy.md`). When you report a green run, say **which
environment** produced it and **which of the rows above were therefore not
exercised** — otherwise the next reader inherits a number with no idea what stood
behind it.

The commit that certified 519 passing tests was not dishonest. It just did not say
which suite.
