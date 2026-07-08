#!/usr/bin/env python3
"""Bird-vs-hostile-UAS discrimination Monte-Carlo GATE -- OFFLINE SCAFFOLD.

Defines and exercises the "drones-and-birds" Monte-Carlo gate from
docs/bird_discrimination_design.md S6 and ADR-0035 (docs/decisions.md):
the safety requirement that the interceptor's positive-ID interlock
classifies birds as NON-hostile and NEVER engages one --
false_engage_on_birds MUST == 0, fail-safe by construction (uncertain ->
do not engage, S1/S5.3).

THIS SCRIPT IS OFFLINE AND BOOTS NOTHING. It never launches PX4 or
Gazebo, spawns no models, and calls no `gz service`. Per this task's
scope, it only (a) documents the gate, (b) builds a deterministic,
seeded plan of synthetic scenarios mirroring the four situations in
S6.3 (single drone / single bird / mixed / flock), (c) runs each planned
track through the safety INTERLOCK LOGIC (interlock_decision(), below --
this part is real, testable today), and (d) attempts to run each track
through the PERCEPTION hook (classify(), below -- this part is a STUB,
because there is no seeker classifier in this repo yet that emits the
dedicated `P(hostile)` posterior ADR-0035 requires). Logs one row per
(gate, track) to a CSV under logs/ and prints a scripted PASS/FAIL gate
summary, in the spirit of scripts/check_m5.sh / scripts/mc_batch.sh.

WHY THIS EXITS NON-ZERO TODAY, ON PURPOSE: classify() is unwired, so
EVERY track's classification is unavailable. Per ADR-0035's own fail-safe
rule (S5.2c, red-team fix #7): "absent, stale, or timed-out
classification => P(hostile)=0 => break-off" -- an unwired classifier is
treated identically to a BIRD vote. That makes false_engage_on_birds read
0/N trivially, which is NOT evidence of working discrimination -- it is
the documented VETO-ALL default correctly firing. The tell is the OTHER
half of the gate (S6.4): correct_engage_on_drones ALSO reads 0%, because
with no live classifier even a real hostile drone is held. A gate that
"passes" on false-engage only because it also fails every real engage is
not a pass, so this harness reports it honestly as NOT YET SATISFIABLE
and exits 1 -- see print_gate_summary() for the exact wiring this hook
needs before the gate can produce a real PASS/FAIL.

WHAT REMAINS REAL AND VERIFIABLE TODAY, even with classify() unwired:
  * The interlock's fail-safe AND-gate logic (interlock_decision()) --
    any bird vote, any uncertain vote, or a contested (>1-candidate)
    association gate forces "hold". This is pure decision logic per
    S5.1/S5.3 and does not depend on a real classifier existing.
  * That EVERY flock/contested-association scenario holds for EVERY
    member, regardless of what classify() would have said (S2c: >1
    candidate in the gate is itself disqualifying).
  * The deterministic seeded plan generation + CSV logging plumbing this
    harness will keep once classify() exists for real.

GATING (be honest about sequencing, per S6.1): this sits behind (a) the
M5 finish (done, ADR-0036) and (b) the markerless-seeker classifier
EXISTING with a dedicated `P(hostile)` output -- neither scripts/seeker/
nor any other module in this repo has one today, hence the stub -- and
(c) the Stage-0 bench producing the P_correct(R) curve that sets
PID_HIGH / PID_ONBOARD_MIN / N-of-M (still UNSET). Until (c), ADR-0035
says the interlock DEFAULTS TO VETO-ALL; this harness's own numbers must
NEVER be used to back-fit those thresholds (S6.4 red-team fix #6).

DECOY MODEL: models/fpv_bird_decoy/model.sdf is the PLUMBING-tier bird
decoy this gate is written against (static, no flap, no motion blur --
see that model's header for the two-tier framing). A green run of a
FUTURE live version of this gate (once classify() is wired and actually
spawns models/fpv_bird_decoy/ via a mover, mirroring scripts/
m4_target_mover.py) would be interlock-WIRING validated, never
bird-rejection validated (S6.4) -- that claim needs the Stage-0 bench on
real hardware against a WORST-tier flapping decoy (S6.5).

SIM-TIME NOTE (ADR-0009, for the future live version, not this offline
scaffold): this script performs no scheduling of its own -- it is a pure
Python loop over a static plan, with no sim clock involved. If/when this
gate grows a LIVE arm (booting Gazebo and spawning models/fpv_bird_decoy/
+ a bird-path mover, analogous to scripts/mc_batch.sh), every duration,
hold, or timeout in that live arm MUST be scheduled/measured in SIM time,
never wall time (RTF sags to ~0.3-0.5 under load) -- the same rule
scripts/m4_target_mover.py and scripts/mc_batch.sh already follow. It
would also inherit the mc-batch skill's rules: ONE sim at a time, idle
machine load only, sequential arms, never `pkill` from a shell whose argv
contains the batch script's own name.

DETERMINISM: the whole scenario plan is drawn from a single
`random.Random(master_seed)` instance in a fixed iteration order (mirrors
scripts/mc_batch.sh's flight-plan generator) -- no unseeded `random.*`
module-level calls, no OS entropy. The same --master-seed always produces
the same plan and (once classify() is real) the same classification
inputs.

HONESTY BOUNDARY: gt_type / gt_subtype below are SCORING-ONLY ground
truth -- which row of the confusion matrix a track belongs to. They are
NEVER passed to classify() (see the Track dataclass docstring); a real
classify() must derive its answer from camera-observable features only,
exactly like every other guidance/perception path in this repo
(CLAUDE.md honesty boundary).

Usage:
    .venv-seeker/bin/python scripts/bird_mc_harness.py --help
    .venv-seeker/bin/python scripts/bird_mc_harness.py --dry-run
    .venv-seeker/bin/python scripts/bird_mc_harness.py --n-per-scenario 8

Exit codes: 0 only if every gate criterion below is satisfied for real
(not possible today -- see WHY above); 1 otherwise, including the current
"classifier not wired yet" state. --dry-run always exits 0 (it only
prints the plan and writes nothing).
"""

from __future__ import annotations

import argparse
import csv
import random
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parent.parent
LOGS_DIR = REPO_ROOT / "logs"

# ---------------------------------------------------------------------------
# The three classification outcomes the interlock reads (ADR-0035 S2c/S5.1).
# `P(hostile)` in the full design is a continuous posterior; this scaffold
# collapses it to the three decision-relevant buckets the interlock actually
# branches on (>= PID_HIGH+affirmative-signature -> "hostile", anything below
# that or ambiguous -> "uncertain", a confident bird-like read -> "bird").
# ---------------------------------------------------------------------------
CLASS_HOSTILE = "hostile"
CLASS_BIRD = "bird"
CLASS_UNCERTAIN = "uncertain"
VALID_CLASSES = (CLASS_HOSTILE, CLASS_BIRD, CLASS_UNCERTAIN)

DECISION_ENGAGE = "engage"
DECISION_HOLD = "hold"

SCENARIOS = ("single_drone", "single_bird", "mixed", "flock")


# ---------------------------------------------------------------------------
# The perception hook -- UNWIRED STUB. This is the ONLY function in this
# file that a real integration needs to replace.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Track:
    """Placeholder shape for what a real classify() would receive: a stand-in
    for camera/track-observable features (bearing history, apparent size,
    motion smoothness, appearance crop -- the real feature list is
    docs/bird_discrimination_design.md S2a onboard + S2b ground). Deliberately
    carries NO scenario label and NO gt_type: a real classify() must derive
    its answer from what the sensor can see, never from this harness's own
    test bookkeeping. Feeding gt_type into classify() would violate the same
    gt_*-is-scoring-only honesty boundary CLAUDE.md enforces everywhere else
    in this repo (the cue/EKF honesty audits, guidance never reading gt_*)."""

    track_id: int
    seed: int


def classify(track: Track) -> str:
    """STUB -- NOT WIRED. Must return one of CLASS_HOSTILE / CLASS_BIRD /
    CLASS_UNCERTAIN once implemented for real.

    THIS FUNCTION DOES NOT EXIST YET as a real classifier, and this stub
    must never be treated as a pass. Per ADR-0035 / docs/
    bird_discrimination_design.md S2c/S5.1, wiring it for real requires:

      1. A seeker classifier module (scripts/seeker/ has none today) that
         emits a DEDICATED hostile-class posterior `P(hostile)` -- a NEW
         field, NEVER a reuse of the existing `Measurement.decision_margin`
         slot that scripts/seeker/classical_seeker.py, nn_seeker.py, and
         two_stage_seeker.py already carry. `decision_margin` is
         detection-PRESENCE / decode confidence and is HIGH for a
         clearly-seen bird; gating on it would engage the bird (red-team
         fix #2, S5.1/S2c). The two confidences are orthogonal and must
         never share a slot.
      2. `P(hostile)` must combine onboard appearance + (when available)
         ground kinematic / stereo-true-size / radar likelihoods via the
         decision-level fusion rule in S2c (Bayesian LLR-sum default),
         riding the SAME track + chi-square association gate as the
         ADR-0034 EKF -- so it degrades to camera-only automatically when
         the ground link drops, instead of a hand-coded fallback.
      3. The hostile streak must be bound to a tracked OBJECT IDENTITY
         (S2c), resetting on any re-association / ID switch / detection
         gap; inside the terminal a re-association is a MANDATORY
         immediate break-off, not merely a streak reset (S5.2b).
      4. The ARM gate needs an AFFIRMATIVE powered-drone signature (hover /
         vertical-climb / rotor micro-Doppler / sustained powered cruise),
         never "failed to look bird-like" (S0 item 5, S2b-1).
      5. Operating-point thresholds (PID_HIGH, PID_ONBOARD_MIN, N-of-M) are
         UNSET pending the Stage-0 bench's measured P_correct(R) curve
         (S6.1/S6.4/S6.5, red-team fix #6) -- they must NOT be back-fit
         from this harness's own numbers. Until the bench sets them, ANY
         real wiring of this hook must still default to CLASS_UNCERTAIN
         for every track (VETO-ALL) -- the same fail-safe default this
         stub effectively produces by raising below.

    Args:
        track: camera/track-observable features ONLY (see the Track
            dataclass docstring) -- never a gt_type ground-truth label.

    Raises:
        NotImplementedError: always, until the wiring above exists. The
        interlock's OWN fail-safe default (see interlock_decision() and
        S5.2c) treats an absent/unwired/stale classification identically
        to a "bird" vote, so a caller that catches this and substitutes
        CLASS_BIRD gets the SAME safe (hold) outcome this raise implies --
        by construction, not by the caller's discretion. run_gate() below
        does exactly that substitution, which is why the false-engage
        check can still be exercised honestly even with this stub in
        place.
    """
    raise NotImplementedError(
        "classify() is an intentionally unwired STUB (ADR-0035, "
        "docs/bird_discrimination_design.md S2c/S5.1) -- there is no "
        "seeker classifier module in this repo yet that emits a dedicated "
        "P(hostile) posterior. See this function's docstring for the "
        "exact wiring required before the bird-reject gate can produce a "
        "real PASS/FAIL."
    )


# ---------------------------------------------------------------------------
# The safety interlock -- REAL logic, testable today independent of
# classify() being wired (ADR-0035 S5.1/S5.3).
# ---------------------------------------------------------------------------


def interlock_decision(classifications: Sequence[str], contested: bool) -> str:
    """Fail-safe interlock AND-gate (ADR-0035 S5.1/S5.3), simplified to a
    SINGLE-FRAME decision. (The real onboard interlock is a temporal state
    machine -- slow N-of-M to commit, fast asymmetric positive-ID-to-hold to
    veto, S5.2 -- which is out of scope for this offline scaffold; this
    function captures only the one-frame AND-gate that every temporal state
    ultimately reduces to, which is enough to prove the fail-safe branches
    in S5.3 route to "hold".)

    Args:
        classifications: the classify() result (or its fail-safe
            substitute, see run_gate()) for every candidate currently in
            this track's association gate.
        contested: True if more than one candidate falls in the
            association gate, OR a recent merge/split event was flagged
            upstream (S2c) -- passed explicitly so a caller can flag the
            "recent split, one candidate left" case the bare candidate
            count would miss.

    Returns:
        DECISION_ENGAGE iff exactly one candidate is present, it
        classified CLASS_HOSTILE, and the gate is not contested. Every
        other combination -- any CLASS_BIRD, any CLASS_UNCERTAIN, more
        than one candidate, or a contested flag -- returns DECISION_HOLD.
        Fail-safe by construction: ambiguity always breaks off (S5.3
        table; S1.5.1 invariant #2, which ROE can never move).
    """
    classifications = list(classifications)
    for c in classifications:
        if c not in VALID_CLASSES:
            raise ValueError(f"interlock_decision: invalid classification {c!r}")
    if contested or len(classifications) != 1:
        return DECISION_HOLD
    return DECISION_ENGAGE if classifications[0] == CLASS_HOSTILE else DECISION_HOLD


# ---------------------------------------------------------------------------
# Deterministic scenario plan -- mirrors S6.3's four situations.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PlannedGate:
    """One association gate: one or more tracks the interlock must decide
    on together. `tracks` is (track_id, gt_type, gt_subtype) tuples; gt_* is
    scoring-only (see the module docstring's honesty-boundary note)."""

    run_idx: int
    scenario: str
    seed: int
    contested: bool
    tracks: Tuple[Tuple[int, str, str], ...]


def build_plan(n_per_scenario: int, scenarios: Sequence[str], master_seed: int) -> List[PlannedGate]:
    """Builds the whole scenario plan from ONE `random.Random(master_seed)`
    instance in a fixed (scenario outer, index inner) iteration order --
    mirrors scripts/mc_batch.sh's flight-plan generator. Deterministic: the
    same --master-seed always yields the same plan.

    Scenario semantics (S6.3):
      single_drone -- one hostile track alone; correct-engage baseline.
      single_bird  -- one bird track alone (random glide/flap-analog
                      subtype); false-engage baseline -- there is no valid
                      target in the gate.
      mixed        -- one hostile + one bird track, each its OWN uncontested
                      gate (a working tracker keeps them separate) --
                      exercises DISCRIMINATION: engage the drone, veto the
                      bird.
      flock        -- 2-4 bird tracks sharing ONE contested association gate
                      -- exercises S2c's "ambiguity -> break off" rule; every
                      member must hold regardless of individual
                      classification.
    """
    if n_per_scenario < 1:
        raise ValueError("--n-per-scenario must be >= 1")
    for s in scenarios:
        if s not in SCENARIOS:
            raise ValueError(f"unknown scenario {s!r} (expected one of {SCENARIOS})")

    rng = random.Random(master_seed)
    plan: List[PlannedGate] = []
    run_idx = 0
    for scenario in scenarios:
        for _ in range(n_per_scenario):
            seed = rng.randint(1, 999_999)
            if scenario == "single_drone":
                tracks = ((0, CLASS_HOSTILE, "powered_cruise"),)
                contested = False
            elif scenario == "single_bird":
                subtype = "adversarial_straight_glide" if rng.random() < 0.5 else "flap_analog"
                tracks = ((0, CLASS_BIRD, subtype),)
                contested = False
            elif scenario == "mixed":
                subtype = "adversarial_straight_glide" if rng.random() < 0.5 else "flap_analog"
                tracks = (
                    (0, CLASS_HOSTILE, "powered_cruise"),
                    (1, CLASS_BIRD, subtype),
                )
                contested = False
            elif scenario == "flock":
                n_birds = rng.randint(2, 4)
                tracks = tuple((i, CLASS_BIRD, "flock_member") for i in range(n_birds))
                contested = True
            else:  # pragma: no cover -- guarded above
                raise ValueError(f"unknown scenario {scenario!r}")
            plan.append(PlannedGate(run_idx, scenario, seed, contested, tracks))
            run_idx += 1
    return plan


# ---------------------------------------------------------------------------
# Running the plan through classify() + interlock_decision(), and logging.
# ---------------------------------------------------------------------------


@dataclass
class GateRow:
    run_idx: int
    scenario: str
    seed: int
    track_id: int
    gt_type: str
    gt_subtype: str
    n_candidates_in_gate: int
    contested: bool
    classify_result: str  # "" if classify() raised
    classify_error: str  # "" if classify() succeeded
    effective_vote: str  # fail-safe-substituted vote actually fed to the interlock
    interlock_decision: str


CSV_HEADER = [
    "run_idx", "scenario", "seed", "track_id", "gt_type", "gt_subtype",
    "n_candidates_in_gate", "contested", "classify_result", "classify_error",
    "effective_vote", "interlock_decision", "is_false_engage",
    "is_correct_engage_opportunity", "is_correct_engage",
]


def run_gate(plan: Sequence[PlannedGate]) -> List[GateRow]:
    """Runs every planned gate through classify() (real if wired, else the
    stub's NotImplementedError) and interlock_decision(). Per S5.2c
    (red-team fix #7), an unavailable classification is substituted with
    CLASS_BIRD before it reaches the interlock -- "absent classifier ==
    P(hostile)=0 == a bird vote" is the documented default, not a
    workaround this harness invented. That substitution is exactly what
    lets false_engage_on_birds be computed honestly even while classify()
    is a stub: it will read 0 because EVERYTHING is held, and the printed
    gate summary explains why that is not, by itself, a pass."""
    rows: List[GateRow] = []
    for gate in plan:
        votes: List[str] = []
        per_track = []
        for track_id, gt_type, gt_subtype in gate.tracks:
            track = Track(track_id=track_id, seed=gate.seed)
            try:
                result = classify(track)
                if result not in VALID_CLASSES:
                    raise ValueError(f"classify() returned invalid class {result!r}")
                error = ""
            except NotImplementedError as exc:
                result = ""
                error = str(exc)
            effective_vote = result if result in VALID_CLASSES else CLASS_BIRD
            votes.append(effective_vote)
            per_track.append((track_id, gt_type, gt_subtype, result, error, effective_vote))

        decision = interlock_decision(votes, gate.contested)

        for track_id, gt_type, gt_subtype, result, error, effective_vote in per_track:
            rows.append(
                GateRow(
                    run_idx=gate.run_idx,
                    scenario=gate.scenario,
                    seed=gate.seed,
                    track_id=track_id,
                    gt_type=gt_type,
                    gt_subtype=gt_subtype,
                    n_candidates_in_gate=len(gate.tracks),
                    contested=gate.contested,
                    classify_result=result,
                    classify_error=error,
                    effective_vote=effective_vote,
                    interlock_decision=decision,
                )
            )
    return rows


def write_csv(rows: Sequence[GateRow], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        for r in rows:
            is_false_engage = int(r.gt_type == CLASS_BIRD and r.interlock_decision == DECISION_ENGAGE)
            is_correct_opportunity = int(r.gt_type == CLASS_HOSTILE)
            is_correct_engage = int(r.gt_type == CLASS_HOSTILE and r.interlock_decision == DECISION_ENGAGE)
            w.writerow(
                [
                    r.run_idx, r.scenario, r.seed, r.track_id, r.gt_type, r.gt_subtype,
                    r.n_candidates_in_gate, int(r.contested), r.classify_result,
                    r.classify_error, r.effective_vote, r.interlock_decision,
                    is_false_engage, is_correct_opportunity, is_correct_engage,
                ]
            )


# ---------------------------------------------------------------------------
# Gate scoring + summary printing.
# ---------------------------------------------------------------------------


@dataclass
class GateSummary:
    n_gates: int
    n_tracks: int
    n_bird_tracks: int
    n_hostile_tracks: int
    n_contested_gates: int
    false_engages: int
    contested_engages: int
    correct_engages: int
    n_unwired: int

    @property
    def false_engage_ok(self) -> bool:
        return self.false_engages == 0

    @property
    def contested_hold_ok(self) -> bool:
        return self.contested_engages == 0

    @property
    def correct_engage_frac(self) -> float:
        return self.correct_engages / self.n_hostile_tracks if self.n_hostile_tracks else float("nan")

    @property
    def classifier_wired(self) -> bool:
        return self.n_unwired == 0

    @property
    def gate_passes(self) -> bool:
        # A real pass needs ALL FOUR: no false engages, every contested gate
        # held, a nonzero (bench-disclosed-floor) correct-engage rate on real
        # hostiles, and a live classifier. Today (2) is trivially true and
        # (3)/(4) cannot be true -- see print_gate_summary().
        return (
            self.false_engage_ok
            and self.contested_hold_ok
            and self.classifier_wired
            and self.correct_engages > 0
        )


def summarize(rows: Sequence[GateRow]) -> GateSummary:
    gate_ids = {r.run_idx for r in rows}
    bird_rows = [r for r in rows if r.gt_type == CLASS_BIRD]
    hostile_rows = [r for r in rows if r.gt_type == CLASS_HOSTILE]
    contested_rows = [r for r in rows if r.contested]
    return GateSummary(
        n_gates=len(gate_ids),
        n_tracks=len(rows),
        n_bird_tracks=len(bird_rows),
        n_hostile_tracks=len(hostile_rows),
        n_contested_gates=len({r.run_idx for r in contested_rows}),
        false_engages=sum(1 for r in bird_rows if r.interlock_decision == DECISION_ENGAGE),
        contested_engages=sum(1 for r in contested_rows if r.interlock_decision == DECISION_ENGAGE),
        correct_engages=sum(1 for r in hostile_rows if r.interlock_decision == DECISION_ENGAGE),
        n_unwired=sum(1 for r in rows if r.classify_error),
    )


def _mark(ok: bool) -> str:
    return "PASS" if ok else "FAIL"


def print_gate_summary(summary: GateSummary, out_path: Path) -> None:
    print("[bird_mc_gate] === Gate summary (ADR-0035 drones-and-birds interlock, S6.3/S6.4) ===")
    print(
        f"[bird_mc_gate] n_gates={summary.n_gates} n_tracks={summary.n_tracks} "
        f"bird_tracks={summary.n_bird_tracks} hostile_tracks={summary.n_hostile_tracks} "
        f"contested_gates={summary.n_contested_gates}"
    )
    print(
        f"[bird_mc_gate]   [{_mark(summary.false_engage_ok)}] false_engage_on_birds == 0 "
        f"({summary.false_engages}/{summary.n_bird_tracks} birds engaged)"
    )
    print(
        f"[bird_mc_gate]   [{_mark(summary.contested_hold_ok)}] every contested/flock gate held "
        f"({summary.contested_engages} engages across {summary.n_contested_gates} contested gates)"
    )
    ce_frac = summary.correct_engage_frac
    ce_frac_str = "nan" if ce_frac != ce_frac else f"{ce_frac * 100.0:.1f}%"
    print(
        f"[bird_mc_gate]   [{_mark(summary.correct_engages > 0)}] correct_engage_on_drones > 0 "
        f"({summary.correct_engages}/{summary.n_hostile_tracks} = {ce_frac_str} -- floor is UNSET "
        "pending the Stage-0 bench, ADR-0035 S6.1; 0 here is expected while classify() is unwired)"
    )
    print(
        f"[bird_mc_gate]   [{_mark(summary.classifier_wired)}] classify() wired "
        f"({summary.n_tracks - summary.n_unwired}/{summary.n_tracks} tracks actually classified; "
        f"{summary.n_unwired} raised NotImplementedError)"
    )
    print(f"[bird_mc_gate] CSV: {out_path}")
    print("[bird_mc_gate]")

    if summary.gate_passes:
        print("[bird_mc_gate] GATE PASSES.")
        return

    print("[bird_mc_gate] GATE NOT YET SATISFIABLE -- exiting 1 (a real failure, not a placeholder pass).")
    print("[bird_mc_gate]")
    print("[bird_mc_gate] WHY: classify() (this file) is an intentional stub -- there is no seeker")
    print("[bird_mc_gate] classifier anywhere in this repo yet that emits a dedicated P(hostile)")
    print("[bird_mc_gate] posterior. Per ADR-0035's own fail-safe rule (S5.2c), an absent/unwired")
    print("[bird_mc_gate] classification is treated as a BIRD vote for every track, which is why")
    print("[bird_mc_gate] false_engage_on_birds trivially reads 0 above -- that is NOT evidence of")
    print("[bird_mc_gate] working discrimination, it is the documented VETO-ALL default correctly")
    print("[bird_mc_gate] firing. The proof: correct_engage_on_drones ALSO reads 0 -- with no live")
    print("[bird_mc_gate] classifier, real hostile drones are held too. A 0/0 gate is not a pass.")
    print("[bird_mc_gate]")
    print("[bird_mc_gate] WIRING NEEDED before this gate can produce a real PASS/FAIL:")
    print("[bird_mc_gate]   1. A seeker classifier module (scripts/seeker/ has none today) emitting a")
    print("[bird_mc_gate]      DEDICATED P(hostile) posterior -- never the existing Measurement.")
    print("[bird_mc_gate]      decision_margin slot (detection confidence; orthogonal; ADR-0035")
    print("[bird_mc_gate]      S2c/S5.1 red-team fix #2).")
    print("[bird_mc_gate]   2. classify() implemented against that posterior + the S5.1 AND-gate:")
    print("[bird_mc_gate]      fused P(hostile) >= PID_HIGH AND onboard-alone P(hostile) >=")
    print("[bird_mc_gate]      PID_ONBOARD_MIN AND an affirmative powered-drone signature AND")
    print("[bird_mc_gate]      association uncontested.")
    print("[bird_mc_gate]   3. PID_HIGH / PID_ONBOARD_MIN / N-of-M thresholds set from the Stage-0")
    print("[bird_mc_gate]      bench's measured P_correct(R) curve (ADR-0035 S6.1/S6.4/S6.5) -- NOT")
    print("[bird_mc_gate]      back-fit from this harness's own numbers (red-team fix #6).")
    print("[bird_mc_gate]   4. Even once wired, a green result here is INTERLOCK-WIRING validated,")
    print("[bird_mc_gate]      never BIRD-REJECTION validated (S6.4) -- models/fpv_bird_decoy/ is the")
    print("[bird_mc_gate]      PLUMBING-tier decoy, not the WORST-tier flapping/matched-footprint")
    print("[bird_mc_gate]      decoy a real bird-rejection claim requires.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="bird_mc_harness.py",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--n-per-scenario", type=int, default=8,
        help="planned gates per scenario (default 8, matches the repo's paired-seed n>=8 "
             "discipline for A/B claims, CLAUDE.md 'Statistics before verdicts')",
    )
    p.add_argument(
        "--scenarios", default=",".join(SCENARIOS),
        help=f"comma-separated scenario list, subset of {{{','.join(SCENARIOS)}}} "
             f"(default: all four, S6.3)",
    )
    p.add_argument(
        "--master-seed", type=int, default=42,
        help="seeds the whole deterministic plan (default 42, matches scripts/mc_batch.sh's "
             "convention); the same seed always yields the same plan",
    )
    p.add_argument(
        "--out", default=None,
        help="CSV output path (default: logs/bird_mc_gate_<UTC timestamp>.csv)",
    )
    p.add_argument(
        "--dry-run", action="store_true",
        help="print the planned gate/track matrix and exit 0 -- no classify() calls, "
             "no CSV written, no gate scored",
    )
    return p


def cmd_dry_run(plan: Sequence[PlannedGate]) -> None:
    print(f"[bird_mc_gate] DRY RUN -- planned {len(plan)} gates, "
          f"{sum(len(g.tracks) for g in plan)} tracks total. No classify() calls, no files written.")
    print(f"{'run_idx':>8} {'scenario':<14} {'seed':>8} {'contested':<10} tracks(track_id:gt_type:gt_subtype)")
    for gate in plan:
        tracks_str = " ".join(f"{tid}:{gt}:{sub}" for tid, gt, sub in gate.tracks)
        print(f"{gate.run_idx:>8} {gate.scenario:<14} {gate.seed:>8} {str(gate.contested):<10} {tracks_str}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)

    scenarios = [s.strip() for s in args.scenarios.split(",") if s.strip()]
    try:
        plan = build_plan(args.n_per_scenario, scenarios, args.master_seed)
    except ValueError as exc:
        print(f"[bird_mc_gate] FAIL: {exc}", file=sys.stderr)
        return 1

    if args.dry_run:
        cmd_dry_run(plan)
        return 0

    rows = run_gate(plan)

    if args.out:
        out_path = Path(args.out)
    else:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        out_path = LOGS_DIR / f"bird_mc_gate_{stamp}.csv"
    write_csv(rows, out_path)

    summary = summarize(rows)
    print_gate_summary(summary, out_path)

    return 0 if summary.gate_passes else 1


if __name__ == "__main__":
    raise SystemExit(main())
