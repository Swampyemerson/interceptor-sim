# Ground-rig sweep plan — does a longer-carried cue buy back the markerless
# acquisition-range regression? (Option C deliverable, ADR-0039)

*Precise, runnable experimental plan. Nothing here has been run — this is the
design the main session executes next, one arm at a time, sims ONE at a time
at idle load. Every flag named below was verified against the current
`scripts/m4_intercept.py` / `scripts/mc_batch.sh` / `scripts/s2_cue_mock.py`
source, not assumed.*

Companions: `docs/ground_rig_division_of_labor.md` (the design this
operationalizes — read that first for the "why" of the two-sensor split),
`docs/seeker_prototype_results.md` (the measured markerless acquisition
numbers this plan tries to buy back), `.claude/skills/mc-batch/SKILL.md` (the
launch/gate/parse loop these arms reuse verbatim), `docs/bird_discrimination_design.md`
(§5), ADR-0023/0025/0030/0034/0036 in `docs/decisions.md`.

---

## 0. One-paragraph summary

The markerless seeker only acquires the target body at **~1.6–3.0 m** (vs the
AprilTag's ~9–12 m — `docs/seeker_prototype_results.md` §2–4), which is the
disclosed cost of killing the tag. `HANDOFF_RANGE_M` and the handoff detection
streak (`--early-handoff`) are the two knobs in `m4_intercept.py` that decide
**when the ground-cue-driven DASH phase concedes control to the camera-only
terminal**. This plan sweeps those two knobs, crossed with `--seeker
{apriltag, markerless}`, to test whether letting the ground rig's cue keep
flying the interceptor for longer — i.e. making the handoff trigger later
and/or under different eligibility windows — recovers the **handoff-reach
rate** and **clean-rate** that the ADR-0030 dash-abort failure mode predicts
will regress under markerless, **without moving the kinematically-capped
terminal miss** (ADR-0023). This is a single-source (α-β tracker, no fusion)
dry run of the exact lever ADR-0034's fusion capstone will later attack with a
smarter mechanism (covariance-weighted handoff quality) — a cheap, honest
first look before that bigger build.

---

## 1. Hypothesis

**H1 (primary).** Under `--seeker markerless`, the *default* handoff
configuration (whatever is currently the adopted M5 profile — see §2) causes
more dash-aborts and lower handoff-reach than the AprilTag baseline, because
the seeker's genuine ~1.6–3.0 m acquisition floor gives very few frames to
assemble a detection streak before the interceptor either loses the tag or
flies past CPA while still in DASH — the exact `DASH: failed to reach handoff
range within 20 s` / `link_lost_no_acq` failure modes ADR-0030/0031 already
documented for a *degraded cue*, now hypothesized for a *degraded seeker*.

**H2 (the recovery claim).** Letting the ground rig's cue **carry the track
longer** — i.e. delaying the point at which HANDOFF is permitted, either by
requiring the *stricter* (not `--early-handoff`) detection streak, or by
changing the `HANDOFF_RANGE_M` eligibility ceiling — recovers clean-rate and
handoff-reach for the markerless arm, because it keeps the interceptor on the
well-tuned, well-aimed cue-driven DASH lead-pursuit law longer, arriving
closer (where the seeker's detection confidence and target-in-frame size are
both higher) before the terminal state machine is allowed to commit.

**H3 (the honest ceiling — pre-registered, not to be walked back post-hoc).**
Whatever H2 finds, it will **not** move mean/median terminal miss materially,
because the miss is 96%+ set by the delivered ZEM at handoff (r²=0.99,
ADR-0023) and that geometry is a *guidance/tracker* quantity this sweep does
not touch (same α-β tracker, same DASH lead-pursuit law, same pro-nav
terminal — only the trigger threshold for *when* control transfers changes).
The payoff, if any, is **handoff-reach and clean-rate** (mid-course
robustness), matching ADR-0034's own pre-registered framing for the fusion
capstone this sweep is a cheap dry run of.

**Mechanism honesty note (why one of the literally-suggested levers may be a
null by construction):** the task framing names "larger handoff range" as a
way to let the cue carry longer. Reading the code (`m4_intercept.py`, the
HANDOFF transition test: `meas.range_m <= HANDOFF_RANGE_M` gates which
detections count toward the streak), `HANDOFF_RANGE_M` is a **ceiling**, not a
floor — raising it *above* the seeker's natural acquisition envelope (which
is already `1.6–3.0 m << 10 m` default) should be **mechanically non-binding**
for markerless: detections still can't occur beyond ~3 m regardless of how
high the ceiling is set. Arm C below tests this directly (and doubles as a
sanity check: if raising the ceiling does nothing, that confirms the ceiling
isn't the bottleneck, strengthening confidence in the streak-count and
DASH-duration explanations instead). The more mechanically-motivated version
of "cue carries longer" is a **lowered** ceiling (Arm D — new, not in the
original task wording, added because the code reading suggested it): forcing
the interceptor to stay under cue-driven DASH control until it is genuinely
close in, before any detection is even eligible to count.

---

## 2. What is held constant — the M5 adopted deployment profile

Every arm reuses the exact adopted profile from `.claude/skills/mc-batch/SKILL.md`
(the same config ADR-0036's M5 final batch flew), so results compare directly
against the existing AprilTag numbers with nothing else changing:

```bash
export S2_CUE_MOCK_EXTRA="--sigma-range --datum-bias-m 0.5 --latency-jitter-s 0.05 --dropout-markov --emit-velocity --vel-sigma 0.5"
```

- Running-start geometry: `--y0-mag 29.3 --x0 6.5`
- Dash: `--dash-speed 16 --dash-unclamp` (removes the V_TOTAL_MAX clamp that
  silently capped the 16 m/s dash to 13, ADR-0028 addendum)
- Cue velocity emission: `--cue-velocity` (paired with the mock's
  `--emit-velocity` above — ADR-0030's "the fix")
- Path/geometry: `--path line --geometry standard` (the straight crosser —
  keep the maneuver axis out of scope; this sweep is about the seeker/handoff
  axis only, not path robustness, which ADR-0036 already characterized)
- Speed: **9 m/s** only (the reference speed ADR-0030/0031's dash-abort study
  and ADR-0037's EKF A/B both used — keeps this sweep directly comparable to
  existing precedent rather than adding a third free variable)
- Law: **pronav only** (ADR-0036 found pursuit and pro-nav TIE at FPV speed —
  running both would double the budget to re-confirm something already
  settled; if a follow-up wants pursuit too, it is a cheap re-run of the same
  arms with `--laws pursuit`)
- Directions: `both` (mc_batch alternates L→R/R→L across the n flights)
- Tracker: `--tracker alphabeta` (default — **not** `--tracker ekf`). Keeping
  the tracker fixed isolates the seeker/handoff-timing axis; mixing in the
  EKF is explicitly a *separate* rung of ADR-0034's ladder ("don't run all 8
  cells at once")
- Fusion: **off** (`--fuse-midcourse` NOT set). This sweep is a single-source
  (cue-then-camera, hard handoff) dry run — no mid-course blending. That is
  deliberate: it isolates "does changing WHEN the hard handoff fires help"
  from "does blending sources help," which is the separate ADR-0034 question.
- Master seed: `--master-seed 42` (same convention as every prior batch — the
  cue-noise draw sequence is then identical across arms at matched run_idx,
  giving a genuinely paired comparison)
- `--n 8 --laws pronav` → **8 flights per arm** (4 L→R + 4 R→L)

---

## 3. Arms — exactly which flags vary

| Arm | seeker | `--early-handoff` | `--handoff-range` | New flights? | What it tests |
|---|---|---|---|---|---|
| **BASE** (control, reused) | `apriltag` (default) | ON (adopted default) | default (10.0 m) | **No — reuse `logs/mc_final_line9.csv` (ADR-0036), pronav rows only (8 of 16)** | Tag-baseline handoff-reach/clean-rate/miss under the identical adopted profile — the number the markerless arms are measured against |
| **A** | `markerless` | ON (adopted default — i.e. literally just swap `--seeker`, nothing else) | default (10.0 m) | Yes, 8 | H1: the as-adopted regression case |
| **B** | `markerless` | **OFF** (drop the flag → stricter 3-detection streak, not 2) | default (10.0 m) | Yes, 8 | H2, "later handoff" via a stricter commit condition — literally the task's "later `--early-handoff`" |
| **C** | `markerless` | ON | **20.0 m** (raised ceiling) | Yes, 8 | H2, "larger handoff range" as literally suggested; pre-registered EXPECTED null per §1's mechanism note — a raised ceiling should not matter when real detections never occur past ~3 m |
| **D** | `markerless` | ON | **2.5 m** (new — lowered ceiling, below/at the seeker's own acquisition floor) | Yes, 8 | H2, the mechanistically-motivated "cue carries longer" — forces DASH to keep flying the cue-driven lead-pursuit law until genuinely close in before any detection is even eligible to count toward the streak |
| **E** (optional, mechanism-isolation stretch) | `apriltag` | ON | **2.5 m** (same tight ceiling as D) | Optional, 8 | Isolates whether the *lever itself* (a tight ceiling) helps/hurts independent of seeker — apriltag already acquires reliably by 9–12 m, so forcing a tight late commit should, per ADR-0023's early-handoff-hurts-ZEM logic, plausibly *hurt* the tag case even as it *helps* (H2) the markerless case. A clean opposite-sign result on the identical flag is the most defensible "context matters" finding in this whole sweep. |

**Required budget:** Arms A–D = 4 arms × 8 flights = **32 new flights**.
**With optional Arm E:** 40 new flights. Both fit comfortably inside the
skill's proven-safe "~48 boots/arm-cluster" envelope — runnable in one idle-
load session, sims strictly sequential.

**Exact commands** (one arm shown; repeat with the arm's own flags/`--out`
path; launch/wait/parse/GPU-health/cooldown loop per
`.claude/skills/mc-batch/SKILL.md`, unchanged):

```bash
cd /home/emerson/interceptor-sim
export S2_CUE_MOCK_EXTRA="--sigma-range --datum-bias-m 0.5 --latency-jitter-s 0.05 --dropout-markov --emit-velocity --vel-sigma 0.5"

# Arm A -- markerless, as-adopted handoff config
scripts/mc_batch.sh --laws pronav --speeds 9 --directions both \
  --path line --geometry standard --n 8 --y0-mag 29.3 --x0 6.5 --master-seed 42 \
  --extra-args "--dash-speed 16 --early-handoff --cue-velocity --dash-unclamp --seeker markerless" \
  --out logs/mc_grsweep_armA_markerless_default.csv \
  > logs/mc_grsweep_armA_stdout.log 2>&1

# Arm B -- markerless, stricter streak (drop --early-handoff)
scripts/mc_batch.sh --laws pronav --speeds 9 --directions both \
  --path line --geometry standard --n 8 --y0-mag 29.3 --x0 6.5 --master-seed 42 \
  --extra-args "--dash-speed 16 --cue-velocity --dash-unclamp --seeker markerless" \
  --out logs/mc_grsweep_armB_markerless_lateHandoff.csv \
  > logs/mc_grsweep_armB_stdout.log 2>&1

# Arm C -- markerless, raised handoff-range ceiling (pre-registered EXPECTED null)
scripts/mc_batch.sh --laws pronav --speeds 9 --directions both \
  --path line --geometry standard --n 8 --y0-mag 29.3 --x0 6.5 --master-seed 42 \
  --extra-args "--dash-speed 16 --early-handoff --cue-velocity --dash-unclamp --seeker markerless --handoff-range 20.0" \
  --out logs/mc_grsweep_armC_markerless_wideCeiling.csv \
  > logs/mc_grsweep_armC_stdout.log 2>&1

# Arm D -- markerless, lowered (tight) handoff-range ceiling
scripts/mc_batch.sh --laws pronav --speeds 9 --directions both \
  --path line --geometry standard --n 8 --y0-mag 29.3 --x0 6.5 --master-seed 42 \
  --extra-args "--dash-speed 16 --early-handoff --cue-velocity --dash-unclamp --seeker markerless --handoff-range 2.5" \
  --out logs/mc_grsweep_armD_markerless_tightCeiling.csv \
  > logs/mc_grsweep_armD_stdout.log 2>&1

# Arm E (optional) -- apriltag control, same tight ceiling as D
scripts/mc_batch.sh --laws pronav --speeds 9 --directions both \
  --path line --geometry standard --n 8 --y0-mag 29.3 --x0 6.5 --master-seed 42 \
  --extra-args "--dash-speed 16 --early-handoff --cue-velocity --dash-unclamp --handoff-range 2.5" \
  --out logs/mc_grsweep_armE_apriltag_tightCeiling.csv \
  > logs/mc_grsweep_armE_stdout.log 2>&1
```

Always `--dry-run` each arm first to confirm the flight count (8) before
committing sim time, per the mc-batch skill. Gate loop per arm: launch as a
tracked (non-`&`) background command, poll `logs/mc_grsweep_armX_stdout.log`
for the `Batch complete` sentinel (never grep a sim-process pattern from a
waiting shell — self-kill footgun), parse the arm CSV, check `dmesg | grep
-icE dxgk` against the pre-arm baseline, `sleep 120` cooldown, next arm. ONE
sim at a time, idle machine load, never two arms concurrently.

---

## 3a. BLOCKING PREREQUISITE — found while writing this plan, not yet fixed

**`scripts/mc_batch.sh` hardcodes `VENV_PYTHON="$REPO_ROOT/.venv/bin/python"`
(line 131), and the main `.venv` has no `onnxruntime`** (verified:
`.venv/bin/python -c "import onnxruntime"` → `ModuleNotFoundError`). The
markerless seeker's own module docstrings already flag this exact gap
(`scripts/seeker/two_stage_seeker.py` lines 29–37, `scripts/seeker/
markerless_loop.py` lines 29–37): a live `--seeker markerless` run needs
**one** venv carrying both gz-transport/MAVSDK (main `.venv`) and
onnxruntime + opencv-contrib (currently isolated in `.venv-seeker`).

**Arms A, B, C, D (and E) cannot run through `mc_batch.sh` as it stands
today** — the moment `m4_intercept.py --seeker markerless` tries `from
markerless_loop import markerless_detection_loop`, the `onnxruntime` import
inside `nn_seeker.py` will fail under `.venv/bin/python`.

**Two documented fix options (both named in the seeker docstrings; picking
one is a real, if small, build decision — flagged for the main session, not
decided here):**
1. `.venv/bin/pip install onnxruntime opencv-contrib-python` (merge the
   seeker deps into the main venv `mc_batch.sh` already uses) — simplest,
   but grows the main venv's dependency surface for every future run, even
   ones that never touch `--seeker markerless`.
2. Add `gz-transport13`/`mavsdk` to `.venv-seeker` and give `mc_batch.sh` a
   `--venv-python PATH` override (or an env-var override of `VENV_PYTHON`)
   so a markerless arm can point at `.venv-seeker` instead — keeps the venvs
   cleanly separated but needs a one-line `mc_batch.sh` change.

This plan does not resolve it (out of this task's scope — no file besides
this plan doc was to be touched, no sim launched). **This is the first thing
the executing session must do before Arm A can boot.**

---

## 4. Metrics — all read from existing CSV columns, nothing new to instrument

Per-flight, from the `mc_batch.sh` aggregate CSV (`miss_m, clean, handoff,
handoff_range_m, handoff_t_s, breakoff_reason, ...` — header already defined
at `scripts/mc_batch.sh` line 398) plus the per-flight `m4_intercept.py` log
pointed to by `flight_csv_path` for the honesty audit:

- **Handoff range achieved** — `handoff_range_m` (blank/NaN when `handoff=0`,
  i.e. cue-driven DASH never conceded control at all). Report mean/median +
  spread per arm; this is the direct answer to "how close did the ground
  cue's custody have to extend."
- **Handoff-reach rate** — fraction of the arm's 8 flights with `handoff=1`.
  This is the ADR-0030/0031 headline metric under degraded perception ("handoff-
  rate, not mean miss, is the honest metric").
- **Clean-rate** — fraction with `clean=1`. Per ADR-0037, clean-rate can
  regress even when `handoff` and `miss_m` both look fine (that A/B's EKF arm
  reached handoff 100% of the time but was clean only 2/8) — track it
  separately, never infer it from the other two.
- **Dash-abort count** — flights with `handoff=0` (never reached the terminal
  at all; `breakoff_reason` will read something like `"DASH: failed to reach
  handoff range within 20.0s"` or the link-lost variant).
- **Terminal-abort count** — flights with `handoff=1` AND `clean=0`
  (reached the camera-only terminal but then lost the tag — `breakoff_reason`
  ~`"lost tag for more than 5.0s (far from target)"`). Distinguishing this
  from dash-abort matters: a fix that trades dash-aborts for terminal-aborts
  is not obviously a win.
- **Miss mean/median** — `miss_m` (present even for a `handoff=0` flyby —
  ADR-0031's warning applies here too: a small miss with `handoff=0` is a
  **blind dead-reckoned near-pass**, not a validated camera-only intercept;
  report handoff-reach and miss together, never miss alone).
- **Pk-vs-radius** — via `scripts/mc_analyze.py`'s `compute_arm_pk()`, ram
  0.5 m / net 1.5 m reference lines (ADR-0025). **Per-arm, NEVER pooled**
  across arms or against the ADR-0036 tag numbers in one curve — five (or
  six) small-multiple subplots, one per arm, exactly like ADR-0025/0036's own
  discipline. A single pooled markerless-vs-tag curve would hide exactly the
  handoff-reach story this sweep exists to surface.

**Honesty audit (re-earned per `two_stage_seeker.py`'s own docstring — "a NEW
guidance path RE-EARNS the numeric no-cheat audit at the live Gazebo A/B"):**
1. **Static check:** grep `scripts/seeker/*.py` for `gt_` reads — expect
   zero hits (the seeker reads only camera pixels + fixed
   `camera_intrinsics.json`, per its own honesty-boundary docstring).
2. **Per-tick numeric check** (mirrors the ADR-0009/0010 verifier pattern):
   recompute what a ground-truth-derived command would have been at a sample
   of ticks and confirm the *actually sent* command tracks the seeker's
   `meas_*` output, diverging from the gt-derived command at exactly the
   ticks where they differ — never matching gt_* where meas_* disagrees with
   it.
3. **`ext_*` columns blank post-HANDOFF** in every flight log — the existing
   CSV-level evidence that the cue channel is closed one-way (ADR-0010 #5:
   illegal-state-unrepresentable, not merely unused-by-convention).
4. **`cue_reads_post_handoff=0`** in every per-run result line — note this
   field is currently a hardcoded structural assertion (the cue socket is
   closed + the Python reference set to `None` at latch), not a live
   per-tick measurement; treat it as confirming the *design* guarantee, and
   let checks 2–3 above supply the *measured* evidence.
5. **Tracker/fusion scope check:** confirm every arm ran with `--tracker
   alphabeta` (default) and without `--fuse-midcourse` — this sweep's
   honesty boundary is the same as every prior gate (camera-only terminal);
   it does **not** yet need ADR-0034's extended "no cue-tainted EKF
   state/covariance survives handoff" audit, because no EKF or fusion is in
   play here. That audit is owed when the fusion capstone itself is built.

---

## 5. Expected-result framing — be honest about what this can and cannot show

**What a positive H2 result would mean:** Arms B and/or D showing higher
handoff-reach and clean-rate than Arm A, at statistically-defensible n=8
paired deltas (CLAUDE.md: n≥8 paired seeds + mechanism, "not significant at
this n" language where the delta doesn't clear the ~1 m/binary-outcome noise
floor) — evidence that **when** the hard cue→camera handoff is allowed to
fire matters more, for a low-recall markerless seeker, than any property of
the terminal guidance law itself. That is a **mid-course robustness** result
(handoff-reach, fewer dash-aborts), not a terminal-accuracy result — expect
`miss_m` (among the flights that DO reach handoff and go clean) to stay flat
across arms A–D, per H3. If it does, that is itself confirmatory: it says the
recovery mechanism is exactly "more flights complete the intercept," not
"the intercepts that complete are more accurate" — consistent with the
kinematic-ceiling diagnosis (ADR-0023) that a smarter/later handoff cannot
out-run a delivered ZEM problem.

**What a negative/null H2 result would mean:** if none of B/C/D move
handoff-reach relative to Arm A, the honest reading is that the markerless
seeker's regression is **not** a handoff-timing problem — it is a raw
detection-probability problem (the seeker's confidence/recall curve itself is
too thin in the available window, independent of when the state machine is
willing to accept a streak). That would redirect the "buy back the
regression" effort toward the seeker's own detection threshold/fine-tune
(`docs/seeker_prototype_results.md` §6's recommended next steps — a
license-clean fine-tuned single-class nano) rather than the handoff state
machine, and would sharpen (not weaken) the case for ADR-0034's fusion
capstone, since a covariance-weighted EKF buys a *continuously* fused,
always-improving track rather than a single threshold-triggered handoff — a
structurally different lever than anything this sweep tests.

**Tie-in to the fusion capstone (ADR-0034):** this sweep is a **single-
source dry run of the same underlying question ADR-0034 asks with fusion** —
"does a warmer/later/better-aimed handoff recover mid-course robustness for
the markerless seeker?" Here the mechanism is a fixed threshold change (when
the hard handoff is allowed to fire); ADR-0034's mechanism is a covariance-
weighted EKF that continuously blends cue and camera through the same
window, letting the "when" answer itself out of live uncertainty rather than
a hand-set flag. A positive result here (H2 holds) is direct motivating
evidence to build the fusion capstone next — it will show the *lever*
(handoff quality/timing) is real and worth a smarter mechanism. A null result
here does not kill the fusion case (fusion buys track *quality* through the
window, not just the trigger point) but does mean the fusion capstone should
not be sold on "fixes markerless handoff-reach" alone.

---

## 6. How the ground stereo rig also helps bird rejection (cross-ref)

Extending the ground rig's custody of the track — exactly what Arms B/D test
mechanically — is not only a handoff-reach lever; it is also, for free, more
time inside the window where **bird rejection is actually possible**. Per
`docs/ground_rig_division_of_labor.md` §3 and `docs/bird_discrimination_design.md`,
the onboard camera's 0.41 s median terminal window (ADR-0023) is too short
for a time-series discriminant and mono cannot recover absolute size at all —
so positive identification (PID) has to be **manufactured on the ground**,
where three things the onboard seeker structurally lacks are available: **(1)
absolute true-size** (stereo range × angular size → metres, cleanly
separating a 0.3 m quad from a 1.0–1.5 m raptor — impossible from a single
mono frame, which can only guess size from an assumed width); **(2) a
kinematic time-series** (sustained powered flight — station-keeping, powered
acceleration against wind, non-ballistic curvature — that a coasting or
flapping bird does not show, accumulated over the mid-course *seconds* the
ground rig has and the onboard terminal does not); and **(3)** the compute
and observation time to run a proper detect-then-track classifier instead of
a single confidence score. A longer-carried cue (this sweep's Arms B/D) is
literally more seconds of ground-channel custody — the same seconds
`bird_discrimination_design.md` §1 requires to build an affirmative
`P(hostile)` posterior before the interceptor is allowed to commit. The two
payoffs (handoff-reach recovery and bird-rejection evidence-accumulation
time) are complementary, not competing, uses of the same mid-course window.
**This sweep does not build or gate bird discrimination** — that remains
gated behind the eight red-team fixes and the Stage-0 bench, defaults to
VETO-ALL (ADR-0035) — this section only notes the mechanism overlap so a
later fusion/PID capstone can reuse the same "extend ground custody" lever
this sweep validates (or doesn't) for handoff-reach.

---

## 7. Analysis + close-out (after the arms run)

1. Merge Arms A–D (+E if flown) with the reused BASE pronav rows into one
   comparison table (NOT one Pk curve — per-arm small multiples, §4).
2. `scripts/mc_analyze.py` per arm for Pk-vs-radius, miss histogram/CDF.
3. Hand-tabulate handoff-reach / clean-rate / dash-abort / terminal-abort
   counts per arm (small n — a spreadsheet-sized table, no new tooling
   needed).
4. Write the result up as an ADR addendum to ADR-0039 (or a new ADR if the
   finding is substantial) with the same "not significant at this n" honesty
   discipline as ADR-0030/0037, and feed the H2 verdict into the ADR-0034
   fusion-capstone build decision per §5 above.
5. Re-run the numeric no-cheat audit checklist (§4) and log the result
   explicitly — do not just assert it passed.

---

### Sources

`docs/ground_rig_division_of_labor.md`; `docs/seeker_prototype_results.md`;
`docs/bird_discrimination_design.md`; `.claude/skills/mc-batch/SKILL.md`;
`docs/decisions.md` ADR-0010/0015/0023/0025/0028/0030/0031/0033/0034/0035/
0036/0037; `scripts/m4_intercept.py` (`--seeker`, `--early-handoff`,
`--handoff-range`, `HANDOFF_RANGE_M`, `DASH_TIMEOUT_S`, `LOST_TAG_ABORT_S`,
CSV_HEADER, the S2 phase-machine docstring); `scripts/mc_batch.sh` (flag
list, aggregate CSV header, `VENV_PYTHON`); `scripts/seeker/two_stage_seeker.py`,
`scripts/seeker/markerless_loop.py`, `scripts/seeker/nn_seeker.py`,
`scripts/seeker/weights/LICENSES.md` (the venv-merge prerequisite + the
AGPL/MIT weight provenance).
