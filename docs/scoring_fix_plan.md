# Scoring-integrity fix plan — issues #8, #9, #3 (planned 2026-08-10)

**Scope:** the three open measurement-layer defects — the wrong-reference scorer (#8), the
coded-dash zero-command bug (#9), and the informationless past-CPA breakoff (#3). All three
either contaminate camera-arm results or misgrade every result; two of them are arm-asymmetric
in the direction that penalises the camera. This plan exists to fix them **in an order that
flies the sim exactly once**, and to sweep every published number the fixes flip.

**Sources:** GitHub issues #8/#9/#3 · `docs/flight_plan_candidates.md` §"THE RULER WAS WRONG"
(lines ~1263–1312) and §"TWO HEAD ERRORS CORRECTED" (ERROR 3, lines ~1113–1123) ·
`scripts/m4_intercept.py` · `scripts/m3_static_intercept.py:311` · ADR-0062, ADR-0084,
commits `0c80454`, `1b51fb4`.

**Hard constraints honoured throughout:** the ram radius stays **0.35 m** (ADR-0084 — no
compensating inflation); dead-band re-tries are graveyarded (measured-out, do not revisit);
every re-stated number keeps the **best-case-upper-bound framing** (perfect launch cue —
ledger `launch-aim-derived-from-ground-truth` — co-altitude by construction, no wind).

---

## 1. THE ORDER

**#8 → #9 → #3 (all three landed and tested), THEN one combined camera-arm re-fly. Zero
flights before that point.**

1. **#8 first — fix the ruler before anything flies.** It is the only fix that changes the
   *meaning* of every flight, and it needs **zero sim time**: every historical arm re-scores
   offline from the per-tick `gt_*` columns already in `logs/`. If anything flew before #8
   landed, its verdict would be computed against the lens-referenced, logged-tick ruler and
   would need re-adjudication afterwards. Fixing the ruler first means no flight in this
   project is ever scored twice.
2. **#9 second — a one-line control-flow fix plus tests, still zero flights.** The fix is
   verifiable by unit test + auditor upgrade + a single canonical-geometry demonstration
   flight; the *fleet-level* verification rides the combined re-fly.
3. **#3 third — validated offline before it flies.** The first-candidate discriminator
   (`--breakoff-max-range-m`, see §5) is already in the codebase and its separation power is
   already measured on existing logs (27 breakoff events, five arms). Adoption is decided by
   the same combined re-fly.
4. **One combined camera-arm fleet, on a frozen HEAD containing all three fixes**, at idle
   load, serial, no workflows running. This single fleet triple-serves as: the #9
   fleet-verification (auditor count = 0), the #3 adopt/reject measurement (premature-breakoff
   rate against ground truth), and the refreshed camera-vs-dash verdicts on the correct ruler.

**Why this order minimises re-flying:** #9 and #3 both live in the ENGAGE terminal, so both
contaminate *only camera arms* — the same ~40 flights. Fix them serially with a fleet between
each and you fly those 40 flights two or three times (the project already flew this exact
fleet once, `1b51fb4`, and #9/#3 were both live during it). Fix everything first and the
fleet flies once. #8 forces no flights at all, ever — it must simply land before the fleet so
the fleet's pre-registered criteria are stated against the correct ruler on day one.

**Why no fix lands on a contaminated baseline:** the fleet's dash-only comparison twins are
**re-scored, never re-flown** (they never ENGAGE — structurally immune to #9 and #3; #8 is
scoring-only). Pairing survives because the twins are reused with the same master seeds
(the review-2 rule). The camera arms' *new* numbers are compared against the *re-scored*
twins, so both sides of every A/B sit on the fixed ruler.

**Sequencing around the live wind/error-correction Workflow** (its wiring agent is editing
`scripts/m4_intercept.py` now): see §6, risks 5–6. Short version: land #9 and #8's small m4
touches as isolated, coordinated commits (most of #8 lives in a new module + standalone
re-scorer precisely to shrink the merge surface); the fleet flies only **after** the workflow
lands, with wind proven default-off/byte-identical — and never while its agents are running
(the 2026-07-26 load-confound VOID).

---

## 2. THE FIXES

### #8 — score centre-to-centre, at the true closest approach

**Concrete change**
- `scripts/m3_static_intercept.py:311` `ground_truth_world_points()` already holds the
  interceptor **model origin** (`tracker.latest[DRONE_MODEL]` → `p_model`) before composing
  the camera. Return it too. Target end unchanged: `fpv_quad_enemy`'s origin IS its airframe
  centre (verified, frame visual offset 0.025 m).
- **Additive CSV columns**, not a semantic change: `gt_intc_x/y/z` (base_link world) and
  `gt_range_c2c`. `gt_cam_*`/`gt_range` keep their current meaning — 16 downstream files read
  them (`render_hud.py`, `eval_seeker_v3.py`, `ekf_ab_analyze.py`, `audit_per_tick.py`, …);
  silently changing their semantics is exactly the schema-break class that bit this project
  twice. Also add the attitude columns (quaternion or roll/pitch) ADR-0062 deferred — they
  close the historical-reconstruction problem permanently (risk 4).
- **Scoring switches to c2c + interpolated CPA** at all three scoring sites:
  `min_gt_range_running` (`m4_intercept.py:4051–4053`), `recompute_min_gt_range_from_csv`
  (`:1691`), and the RESULT cross-check (~`:4859`). Interpolation: linearly interpolate BOTH
  world positions across each tick pair; |Δp(t)|² is quadratic per segment with a closed-form
  minimum. Report three numbers per flight: **c2c interpolated (primary)**, c2c logged-tick
  (conservative check), lens logged-tick (legacy continuity, labelled as such).
- **Standalone re-scorer** `scripts/rescore_cpa.py` (new module — keeps the m4 diff small):
  new-format logs use the exact `gt_intc_*` columns; historical logs reconstruct base_link
  from `gt_cam_*` minus the mount lever arm (vertical 0.208 m — measured, median over 1,748
  ticks; forward 0.12 m rotated by logged `psi_deg`; pitch/roll approximated — see risk 4 for
  the calibration bound). AprilTag/billboard-era logs get the lens correction only, with the
  target-reference caveat stated (centre==origin verified only for `fpv_quad_enemy`).
- **The radius does not move.** 0.35 m, ADR-0084, restated here so nobody "balances" the
  ruler fix by inflating the criterion.

**Proof it is not inert**
- **Producer→consumer contract test**: the fixture CSV is generated by `m4_intercept.py`'s
  OWN `write_row`/`CSV_HEADER` on synthetic ticks — never hand-typed (hand-typed fixtures are
  what hid both prior schema breaks).
- **Mutant test (required by the issue)**: on a fixture with a non-zero lever arm and a
  between-ticks crossing, assert scorer output equals the centre-based interpolated value AND
  differs from both the lens-based and the logged-tick values. Re-pointing the scorer at
  `gt_cam_*` (the restored defect) makes this test FAIL.
- **No-vacuous-verdict**: empty/NaN gt columns → exit ≠ 0, never a number.
- **End-to-end demonstration**: `rescore_cpa.py` over `logs/mc_fp_armAE5dash_line9_s{123,777}.csv`
  must reproduce the head's ad-hoc table — 0.445 m / 3/16 → 0.293 m / 12/16 → ~0.237 m /
  16/16 — from the shipped tool. If the shipped tool does not move those numbers, the fix
  did not ship.

**Evidence to regenerate:** a full old-vs-new re-score table for every arm in `logs/`
(`mc_fp_arm*`, `mc_speed10_*`, loft-dive A/B/Adash/Bdash, and the M5-era batches with the
era caveat), committed as a doc + ADR — old numbers reported alongside, never silently
relabelled. Then the §4 surface sweep, same turn.

### #9 — the coded-dash `last_cmd` hole

**Concrete change**
- `scripts/m4_intercept.py:3370–3371`: the `--coded-dash` branch builds `cmd` and never
  writes `last_cmd`; add `last_cmd = cmd`, mirroring the S2 dash branch at `:3577` (which
  already does it). Consequence to state in the commit: a terminal dropout in the first
  ENGAGE ticks now **holds the dash command** (keeps closing) instead of re-issuing the
  `(0,0,0,0)` initialiser from `:2730` — i.e., the dropout handler at `:4009–4012` finally
  does what its comment says.
- Check the other `last_cmd` readers see sane values on this path: `:3539`, `:3897`,
  `:4039` (`hold_yaw`).
- **Hardware parity, same turn** (the frozen_vworld lesson — one code path): verify
  `flight/deploy/seeker_loop.py` and `flight/deploy/real_flight.py` do not carry the twin
  defect in their dropout/hold paths; if they do, fix there too before any bench work
  inherits it.

**Proof it is not inert**
- Unit test driving the tick logic dash → ENGAGE → dropout-on-first-tick, asserting the held
  command equals the dash velocity and is not the zero initialiser.
- **Auditor upgrade — the existing instrument provably missed this**: `audit_per_tick.py`
  check (e) fails only at >50% zero-command ticks; this bug produced 24 ticks across 5/16
  flights and sailed under it. Add check (f): ANY pre-CPA ENGAGE dropout tick whose command
  is exactly the `(0,0,0,0)` initialiser → FAIL. Run (f) over the OLD logs and show it
  fires (the instrument can see the defect); run it over a post-fix flight and show 0.
- Mutant: revert the line; the unit test and check (f) must both fail.
- **One canonical demonstration flight** (not a fleet): reproduce the worst measured case —
  the flight that held 5 consecutive zero ticks while closing 7.3 → 4.6 m at 13 m/s — using
  the canonical master-seed geometry extracted from the baseline log (the
  reproduce-canonical-gate-geometry rule; arm definitions at
  `scripts/experiments/flight_plans/run_arm.sh:88–104`). Show the dropout now holds the dash
  vector.

**Evidence to regenerate:** audit (f) = 0 across the combined fleet (§3); the camera-arm
verdict refresh itself rides that fleet.

### #3 — the past-CPA breakoff discriminator

**Concrete change** (per the §5 pre-registration, which is written before any flying)
- **First candidate is NOT a new estimator** — see §6 risk 1. Turn on
  `--breakoff-max-range-m 5.0` (flag exists at `m4_intercept.py:2169`, wired at
  `:3989–3992`, shipped inert by ADR-0062 FIX-B): gate breakoff on the ABSOLUTE measured
  range at fire time — a different signal from the measured-out per-step rise magnitude.
  Adopt it into the canonical camera-arm config (`run_arm.sh`) and the `--fpv` defaults.
- Offline validator script that replays historical per-tick logs and counts
  premature-vs-legitimate breakoffs under the gate — it must reproduce the already-measured
  separation (blocks 6/7 premature including every catastrophic r2l flight; suppresses 2/24
  legitimate, both at `closed_after = 0.00` where CPA was already recorded).
- Port the decision to `flight/deploy/real_flight.py` (its breakoff arms at 4.0 m —
  harmonise the constants deliberately, don't let sim and hardware drift).
- The dead-band/min-rise family stays graveyarded. Nothing here re-tries it.

**Proof it is not inert**
- Unit test of the `range_ok` clause (fires blocked above threshold, unaffected below).
- The offline validator reproducing 6/7 vs 2/24 on the historical logs — the instrument
  agrees with the hand measurement before anything flies.
- The combined fleet (§3) measured against ground truth: premature-breakoff rate collapses
  from ~29% (10 mph) / 10–16 mentions per 8-flight arm (9 m/s) to the §5 criterion.

**Evidence to regenerate:** the fleet's premature/legitimate breakoff counts per arm; the
speed-ladder UNBLOCK note (the ladder stays blocked until §5's criterion passes); an ADR
recording adoption or the NULL.

---

## 3. THE RE-FLY BUDGET

**Total: 40 flights (48 with the optional arm), flown once, in one idle session, after all
three fixes land on a frozen HEAD.** ~70–90 s/flight plus boots ≈ 1.5–2 h serial.

| arm | flights | action | why |
|---|---|---|---|
| AE5dash s123+s777 (adopted config), AAL, G20dash, AE15dash, AE2dash, AE7dash, EAL, speed10-dash | 0 | **re-score offline only — never re-flown** | dash-only arms never ENGAGE: structurally immune to #9 and #3; #8 is scoring-only |
| G20 s123, G20 s777 | 16 | **re-fly** (fleet) | the "camera ≈ parity at correct aim" claim — the most load-bearing camera verdict |
| AE5 s123 | 8 | **re-fly** (fleet) | aim-error null adjacent to the adopted config |
| AE15 s123 | 8 | **re-fly** (fleet) | the wider-aim-error null; feeds the cue-error sweep next |
| speed10 camera s123 | 8 | **re-fly** (fleet) | 29% premature-breakoff rate — the #3 validation rung, and the ladder's gate |
| GL s777 (190 ms LOS-lag comp) | (8) | **recommend DROP** | the lever is already closed-negative and both GL and G20 are camera arms (defect hits both sides); direction robust. Re-fly only if the builder wants "closed" re-earned. Saves 8 flights. |

**Sharing — the point of the whole plan:** this ONE fleet is simultaneously (a) #9's
fleet-verification (audit (f) = 0 across all arms), (b) #3's adopt/reject measurement
(§5 criteria, scored against per-tick ground truth), and (c) the refreshed camera-vs-dash
verdicts on the correct ruler, paired per `run_idx` against the **re-scored** dash twins.
No fix gets its own fleet.

**What mis-ordering would cost, concretely:** fleet after #9 but before #3 → breakoff
contamination persists (it fires on ~every camera flight) → a second 40-flight fleet. Fleet
before #8 → pre-registered pass bars adjudicated against the lens ruler → re-adjudication
and, if any verdict is marginal, a credibility re-fly. The project has already paid for one
full fleet (`1b51fb4`) that #9/#3 were live under; this plan makes the next one the last.

**Opportunistic, not required:** the VOID'd 10 mph **aim-trim sweep** (dash-only, 4 arms × 8
= 32 flights, pre-registered in `flight_plan_candidates.md`) depends only on #8 (its decisive
criterion is Pk@0.35). It may fly in any idle window once #8 lands — before #9/#3 if
convenient — because dash-only arms cannot express either bug. Keep it a separate session
from the fleet; never overlap batches.

---

## 4. WHAT MUST BE RE-STATED PUBLICLY (same turn the re-score lands)

Numbers that flip when #8 lands, and every surface that carries them:

| currently published | becomes | where it appears |
|---|---|---|
| "Nothing yet lands inside the 0.35 m ram radius (0/16)" | AE5dash: **12/16** inside 0.35 m at logged ticks, **16/16** interpolated, median 0.293/0.237 m — as a perfect-cue UPPER BOUND | `NEXT.md` §"Where the project is" (also stale vs ERROR 1's 3/16 **today**) |
| kill-stage caption "So far 0 of 16 sim flights get that close" | same correction | `docs/project_state.json` stage `kill` caption + note |
| AE5dash "median 0.445 m, 3/16" | 0.293 / 12/16 (logged), 0.237 / 16/16 (interp) | `docs/flight_plan_candidates.md` (correction already drafted there — mark SHIPPED), ADR-0083 quotes |
| dash floors "0.71 m / 0.43 m" (ADR-0080/0083) | re-scored values from `rescore_cpa.py` | `project_state.json` stage `coded_dash` note, `docs/real_build_coded_dash.md` |
| "the terminal must earn ~1 m" | re-derived gap at the centre datum (near zero for the adopted config **under a perfect cue**) — this changes the airframe-size discussion the kill note already flags | `docs/hardware_order_list.md` §0b/§0c, stage `kill` note |
| "the miss is vertical, −0.37 m" | ~−0.16 m real vertical after removing the 0.208 m lens lever (direction stands, magnitude halves) | anywhere the vertical-offset finding propagated (check the wind/error-correction workflow — §6 risk 6) |
| camera-vs-dash margins & "camera makes it worse" attributions | re-stated from the fleet; until then, margins remain "not quantitative" (already softened) | `project_state.json` stage `terminal`, `README.md` results rows, `NEXT.md` |
| M5-era "median 0.93 m, n=96" | keep, footnoted "lens-referenced, logged-tick, billboard era" (or re-scored with era caveat) | `project_state.json` `key_numbers` |

Mechanics per the CLAUDE.md ritual: edit `project_state.json` → `render_dashboard.py` →
**republish the Artifact to the same URL** → commit; sweep `NEXT.md`, `README.md`, dashboard
§0 headline/hero the SAME turn. Ledger: add-and-resolve `scorer-measured-camera-lens`.
Assumptions register: scoring reference moves to `measured`. Graveyard: dead-band entry
stays. New ADRs: one for #8+#9 (instrument fix + re-score), one for #3 (adoption or NULL).

**Framing rule for every re-stated number:** perfect launch cue, co-altitude by
construction, no wind, sim optics — so "the adopted config meets the contact criterion in
sim **under a zero-error cue**" is the maximal honest claim. Not "we reach contact in the
field." The 16/16 must never argue for skipping the cue-error sweep — that constraint is
exactly what the sweep exists to remove.

---

## 5. PRE-REGISTRATION — #3's replacement discriminator (written before any flying)

**Candidate signal:** the ABSOLUTE measured range at which the breakoff fires
(`--breakoff-max-range-m 5.0` — in-code, documented deployment value under `--fpv`). This is
a different signal from the measured-out per-step rise magnitude: legitimate breakoffs fire
at measured 1.60–2.40 m (20/22), premature ones above 3.1 m (6/7), across 27 recorded events
on five arms. Registered fallback if it nulls: an own-state kinematic passage test (own-EKF
displacement since ENGAGE ≥ initial `r_hat` minus margin — you cannot have passed a target
you have not yet flown far enough to reach), as its own design change with its own pre-reg.

**Config:** canonical camera arms (`run_arm.sh`), gate ON at 5.0 m, everything else frozen at
the post-#8/#9 HEAD. Decided on the §3 fleet — no dedicated arms.

**Predictions:**
1. Premature breakoffs (defined against ground truth: breakoff fires, then true range closes
   > 0.5 m further) collapse from 2/7 (10 mph) and 10–16 mentions/arm (9 m/s) to ≤ 1 across
   the 16 flights of the 10 mph + AE15 rungs, and ≤ 1/40 fleet-wide.
2. Legitimate past-CPA breakoffs still terminate at least as many flights as pre-fix, within
   1.5 s of true CPA; at most one flight converts to an ENGAGE-timeout abort instead.
3. No phantom-chase regression: flights that would previously have broken off early do not
   keep steering > 2 s past true CPA (the honest caveat from ERROR 3 — suppressing an early
   breakoff does not automatically help; this measures it). ≤ 1/40.

**Adopt iff all three hold.**

**What a NULL means (stated now):** if premature breakoffs persist ≥ 2 among the 16
slow/wide-error flights with the gate ON, then the *threshold family* is exhausted — both the
per-step rise AND the absolute range have been measured and failed — and the discriminator
must move to the own-state kinematic fallback as a design change. The speed ladder **stays
blocked**. The cue-error sweep may proceed only with the breakoff disabled and
`ENGAGE_TIMEOUT_S` as the sole backstop, disclosed as a sim-only concession — or it waits. A
null here does NOT license revisiting dead-bands, and it does not impugn the #8/#9 results
(they are independent of which discriminator fires).

**Threshold-portability rule (binding):** 5.0 m is validated only in the 4.5–10 m/s regimes
these logs cover. Every new speed rung re-earns the threshold OFFLINE from that rung's first
camera logs (fire-range distributions, legit vs premature) before that rung's verdict counts
— per the "a threshold validated at one operating point is not validated at another" rule.

---

## 6. RISKS — and where the issue descriptions are wrong

1. **Issue #3 overlooks a measured, in-codebase fix.** Its "a fix needs a different signal:
   own-state kinematics, filtered closing speed, or the LOS sweeping past boresight" reads as
   if a new estimator is required. The red-team already found otherwise (ERROR 3,
   `flight_plan_candidates.md:1113–1123`): the dead-band measurement ruled out ONE
   discriminator (per-step rise magnitude); the absolute-range gate is a different signal,
   already implemented, with a documented value and measured 6/7-vs-2/24 separation. The
   first candidate is a validated flag flip; the issue's signals are the registered
   fallbacks. Taking the issue at face value would have cost a design-and-A/B cycle.
2. **Issue #8's 16/16 is an upper bound, not a result.** Perfect cue, co-altitude, no wind —
   all `given-perfect` inputs. The re-statement must carry that framing everywhere (§4), and
   the conservative logged-tick 12/16 rides alongside the interpolated number permanently.
3. **Interpolation is mildly anticonservative** if 20 Hz sampling aliases real curvature near
   CPA. At ≤ 16 m/s relative speed segments are ≤ 0.8 m and linear-motion error near CPA is
   negligible, but state it, and keep the logged-tick number as the check.
4. **Historical re-score accuracy.** Old CSVs lack attitude, so base_link is reconstructed
   from `gt_cam_*` via measured offsets + `psi_deg` (pitch/roll ignored — worst case during
   a pitched dash ≈ lever × sin(pitch), a few cm). Bound it empirically: the first new-format
   flights carry BOTH exact `gt_intc_*` and the reconstruction; publish the measured error
   bound; if > 0.05 m near CPA, qualify pitched-regime re-scores. Adding attitude columns now
   (ADR-0062's deferred follow-up) closes this class permanently.
5. **The wind Workflow is editing `m4_intercept.py` right now.** #9's fix and #8's m4 touch
   collide with it. Sequence: coordinate with the head session; land #9 as an isolated
   commit either before the workflow's merge or immediately after — never interleaved with
   its wiring agent mid-file. #8's new logic deliberately lives in a new module +
   `rescore_cpa.py` so the m4 diff is a few lines. The fleet flies only after the workflow
   lands AND a byte-identity check proves wind is default-off (else comparability with the
   re-scored dash twins breaks and the fleet repeats — the exact waste this plan prevents).
   And the fleet NEVER flies while workflow agents run: machine load 6.5–7.6 is what VOIDed
   the aim-trim sweep on 2026-07-26.
6. **The Workflow itself is at risk from these defects — tell it.** (a) Any camera arm it
   flies to validate "error correction" before #9/#3 land has a terminal that brakes on
   dropouts and quits early on noise — its A/B could manufacture or erase the effect it is
   measuring. (b) If its vertical error-correction was sized off the published −0.37 m
   offset, half of that number is the camera lever arm (#8): the real vertical gap is
   ~−0.16 m. It may be correcting a number that is 56% measurement artefact.
7. **Verdict re-adjudication discipline.** The old pre-registered bars (camera earns its
   handoff at ≥ 6/8 vs twin; adoption at ≥ 13/16) were chosen before these fixes; they stay
   as-is for the fleet. Re-scoring may flip individual old verdicts — each flip goes through
   the §4 sweep, and no bar is re-chosen after seeing new numbers.
8. **Scope note:** `NEXT.md` currently lists #3 as work-item 1. This plan supersedes that
   ordering (#8 → #9 → #3 → fleet); update `NEXT.md` when the plan is adopted so the two
   surfaces don't contradict.
