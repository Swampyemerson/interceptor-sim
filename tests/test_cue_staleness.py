#!/usr/bin/env python3
"""ADR-0059 cue-staleness age-out: unit + behavioral + honesty tests for the
mid-dash cue-jam fix.

THE HOLE (ADR-0059). The adopted deployment config `--track --handoff-cue-gate 8`
FAILS CLOSED under a mid-dash cue jam. When the cue link is jammed,
CueReader.read() keeps returning the last datagram forever, so `last_cue_pos`
FREEZES at the pre-jam target position. Two walls then reject the REAL target as
it moves away from that frozen point: (1) the handoff cue-gate (m4), and (2) the
detect-then-track seeker SEED gate (_seed_ok's pre-handoff cue-consistency
branch). The interceptor never hands off -> a regression against the ADR-0015
WORST-tier link-cutoff coast-search was built to survive.

THE FIX. Age the frozen cue out on a SIM-time staleness clock (m4.cue_is_stale).
Once stale, treat the cue as ABSENT at BOTH read-sites: skip the handoff cue-gate
(camera-only N-in-range streak) and feed cue_pos=None to the seeker seed, routing
_seed_ok into its camera-track-continuity branch. That branch -- the guidance's
own velocity-aware (dead-reckoned) camera track -- is the ANTI-PHANTOM lever under
jam: a phantom that reads in-range (~1.8 m) while the real target is ~20 m away is
rejected because its camera-implied position is far from the dead-reckoned track.

WHAT THESE TESTS PROVE.
  * INERTNESS: cue_is_stale() is False for every full-duration-cue case (fresh
    cue, no cue ever, no sim clock) -> the fix path is never entered, so all
    ADR-0058 validation + pinned tests stay byte-identical.
  * ANTI-FAIL-CLOSED: under the fix the real target is ADMITTED (the hole shut).
  * ANTI-PHANTOM: under the fix a phantom is REJECTED by the camera-track gate.
  * REGRESSION WITNESS: WITHOUT the fix a frozen cue rejects the real target.
  * HONESTY (ADR-0010/0059): once stale, NO cue-derived value influences the seed
    decision -- a frozen cue that would have ADMITTED a phantom has zero effect.
  * WIRING (AST): both m4 read-sites consult the flag; the clock is sim-time; and
    the predicate is guarded `(not handoff_done) and ...` so it never false-fires
    post-handoff (Fable-review defect fix -- the cue channel is closed at the latch).

Run: `.venv/bin/python -m pytest tests/test_cue_staleness.py -v`
"""
import ast
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
M4_PATH = os.path.join(SCRIPTS, "m4_intercept.py")
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, os.path.join(SCRIPTS, "seeker"))

from m4_intercept import COAST_STALE_S, cue_is_stale         # noqa: E402
from detect_track import DetectThenTracker, SeekerSeedContext  # noqa: E402
from nn_seeker import SeekerDetection                        # noqa: E402

# The default --cue-stale-horizon in m4 (= COAST_STALE_S), repeated as a literal
# so the cases below read as arithmetic rather than as an import.
#
# COMMENT CORRECTED 2026-07-25 (audit, reader 5). It previously claimed a silent
# change to the default would "trip test_m4_exposes_cue_stale_horizon_flag" --
# FALSE, and the worse half of the defect: that test is a pure AST check that the
# argparse default is the *Name* `COAST_STALE_S`, and it never reads the value,
# so m4's 1.0 could drift to 10.0 with the whole suite green. An overclaiming
# comment inside an instrument is the project's named defect class -- it is what
# stops the next reader from adding the pin. The real pin is
# test_coast_stale_s_value_is_pinned below (DEEP-T1's "remaining" item).
HORIZON_S = 1.0


def test_coast_stale_s_value_is_pinned():
    """THE VALUE PIN (docs/audit_findings_tracker.md DEEP-T1).

    PROVENANCE of 1.0 s: `scripts/m4_intercept.py:544` -- "cue silent longer than
    this (sim) => stale/link-loss". It is the ADR-0059 dead-reckon-coast horizon
    and, as the argparse default for --cue-stale-horizon, the value every
    coded-dash/handoff flight ran at unless an arm overrode it (scripts/mc_jam_arm.sh
    documents the same default). Changing it re-scopes every logged coast latch,
    so it is a deliberate decision + ADR, never a silent edit.

    Mirror in the design-time surrogate: scripts/guidance_lab.py:1833 carries
    `COAST_STALE_S=1.0  # m4 COAST_STALE_S`. Divergence there breaks "lab ranks,
    Gazebo decides", so it is pinned to the same number here."""
    assert COAST_STALE_S == 1.0
    assert HORIZON_S == COAST_STALE_S, (
        "this file's local literal drifted from m4's constant")

    # guidance_lab's copy of the same constant must not drift from m4's.
    lab_src = open(os.path.join(SCRIPTS, "guidance_lab.py")).read()
    m = re.search(r"COAST_STALE_S\s*=\s*([0-9.]+)", lab_src)
    assert m, "guidance_lab.py no longer defines COAST_STALE_S"
    assert float(m.group(1)) == COAST_STALE_S, (
        f"guidance_lab COAST_STALE_S={m.group(1)} != m4's {COAST_STALE_S}")


# ---------------------------------------------------------------- helpers
def _det(range_m, bearing_rad=0.0):
    """A minimal SeekerDetection carrying just what _seed_ok reads (range +
    bearing). meas_xyz/box_xywh are unused on the _seed_ok path."""
    return SeekerDetection(
        t_mono=0.0, range_m=range_m, bearing_rad=bearing_rad,
        bearing_vert_rad=None, meas_xyz=None, decision_margin=0.5,
        n_detections=1)


def _tracker(seed_ctx):
    """A DetectThenTracker with HERMETIC gate constants (independent of ambient
    MARKERLESS_TRACK_* env). seeker=None is safe: _seed_ok never touches
    self.seeker -- both its branches go through _cam_implied_ne (pure geometry).
    frames_since_loss=None (pre-handoff, no loss yet) -> re-acq radius == base."""
    trk = DetectThenTracker(seeker=None, fx=600.0, fy=600.0, cx=320.0, cy=240.0,
                            seed_ctx=seed_ctx)
    trk.seed_cue_gate_m = 8.0
    trk.reacq_base_m = 8.0
    trk.reacq_rate_m_s = 5.0
    trk.reacq_cap_m = 12.0
    trk.frames_since_loss = None
    return trk


# ---------------------------------------------------------------- pure helper
# The INERTNESS proof at the unit level: the fix path is entered iff this returns
# True, and it is False for every full-duration-cue case.
def test_cue_is_stale_false_when_fresh():
    # 0.1 s since the last datagram, 1.0 s horizon -> NOT stale (the normal
    # full-duration-cue case; the mock streams at ~10 Hz => age ~0.1 s).
    assert cue_is_stale(last_cue_recv_sim_t=10.0, sim_now=10.1, horizon=HORIZON_S) is False


def test_cue_is_stale_false_at_boundary():
    # Staleness is STRICTLY greater-than: exactly at the horizon is not stale.
    assert cue_is_stale(5.0, 6.0, 1.0) is False


def test_cue_is_stale_true_after_horizon():
    # 1.5 s of silence with a 1.0 s horizon -> the jam has aged the cue out.
    assert cue_is_stale(5.0, 6.5, 1.0) is True


def test_cue_is_stale_false_when_no_cue_ever():
    # Cueless run (or jammed-before-first-fix): last_cue_recv_sim_t stays None ->
    # never stale. The existing last_cue_pos=None fallbacks own that case
    # (ADR-0059 NUANCE) -- we must not spuriously fire here.
    assert cue_is_stale(None, 100.0, 1.0) is False


def test_cue_is_stale_false_when_no_sim_clock():
    # No /clock sample this tick -> not stale (defensive; SIM time only).
    assert cue_is_stale(5.0, None, 1.0) is False


def test_cue_is_stale_uses_sim_time_only_no_wall_clock():
    # ADR-0009 pin: the staleness predicate must never read wall time. AST-scan
    # the cue_is_stale body for any time.monotonic / time.time reference.
    tree = ast.parse(_m4_src(), filename=M4_PATH)
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == "cue_is_stale")
    names = {a.attr for a in ast.walk(fn) if isinstance(a, ast.Attribute)}
    names |= {n.id for n in ast.walk(fn) if isinstance(n, ast.Name)}
    assert "monotonic" not in names and "time" not in names, (
        "cue_is_stale must be SIM-time only (ADR-0009); found a wall-clock "
        f"reference among {sorted(names)}")


# ---------------------------------------------------------------- seed-feed decision
def test_seed_feed_nulls_cue_when_stale():
    # Replicates m4's `seed_cue_pos = None if cue_is_stale_now else last_cue_pos`,
    # fed into the REAL SeekerSeedContext. Fresh -> the cue flows; stale -> None.
    last_cue_pos = (8.0, 0.0)
    ctx = SeekerSeedContext()

    stale = cue_is_stale(10.0, 10.1, HORIZON_S)          # fresh
    ctx.update(0.0, 0.0, 0.0, None if stale else last_cue_pos, False, cam_track=(20.0, 0.0))
    assert ctx.legal_cue_pos() == (8.0, 0.0)

    stale = cue_is_stale(10.0, 12.0, HORIZON_S)          # jammed
    ctx.update(0.0, 0.0, 0.0, None if stale else last_cue_pos, False, cam_track=(20.0, 0.0))
    assert ctx.legal_cue_pos() is None


# ---------------------------------------------------------------- anti-fail-closed
def test_stale_fallback_admits_real_target_via_cam_track():
    # Cue nulled (stale). The dead-reckoned camera track sits on the real target
    # 20 m north. Real detections near it must SEED -> the fail-closed hole shut.
    ctx = SeekerSeedContext()
    ctx.update(0.0, 0.0, 0.0, None, False, cam_track=(20.0, 0.0))
    trk = _tracker(ctx)
    assert trk._seed_ok(_det(range_m=20.0)) is True   # implied (20,0), 0 m off cam_track
    assert trk._seed_ok(_det(range_m=18.0)) is True   # implied (18,0), 2 m off -> within 8
    assert trk.n_reacq_rejected == 0


# ---------------------------------------------------------------- anti-phantom
def test_stale_fallback_rejects_phantom_via_cam_track():
    # Same stale fallback: a phantom reads in-range (~1.8 m) while the real target
    # is 20 m away. Its camera-implied position (1.8, 0) is 18.2 m off the
    # dead-reckoned camera track (20, 0) > the 8 m radius -> REJECTED. The
    # tracker-continuity gate is the anti-phantom lever under jam.
    ctx = SeekerSeedContext()
    ctx.update(0.0, 0.0, 0.0, None, False, cam_track=(20.0, 0.0))
    trk = _tracker(ctx)
    assert trk._seed_ok(_det(range_m=1.8)) is False
    assert trk.n_reacq_rejected == 1


# ---------------------------------------------------------------- regression witness
def test_frozen_cue_fails_closed_regression_witness():
    # WITHOUT the fix: cue jams, last_cue_pos FROZEN at the pre-jam target (8 m
    # north), NOT nulled. The real target has moved to 20 m. The cue-consistency
    # seed gate rejects it (cue_gap 12 m > 8 m) -> no seed -> FAIL CLOSED.
    ctx = SeekerSeedContext()
    ctx.update(0.0, 0.0, 0.0, (8.0, 0.0), False, cam_track=(20.0, 0.0))  # cue NOT nulled
    trk = _tracker(ctx)
    assert trk._seed_ok(_det(range_m=20.0)) is False   # the hole: real target rejected

    # The fix (null the cue) admits the very same detection via cam_track.
    ctx.update(0.0, 0.0, 0.0, None, False, cam_track=(20.0, 0.0))
    assert trk._seed_ok(_det(range_m=20.0)) is True


# ---------------------------------------------------------------- honesty tripwire
def test_stale_fallback_uses_no_cue_value_honesty():
    # The dangerous case the honesty boundary must cover: a frozen cue sitting
    # RIGHT ON a phantom would ADMIT that phantom if it were used. Under the fix
    # the cue is nulled, so the phantom is judged ONLY against the camera track
    # (real target 20 m north) and REJECTED -- proving no cue-derived value
    # reaches the seed decision once the cue is stale.
    phantom = _det(range_m=1.8)      # camera-implied (1.8, 0)

    # (a) frozen cue ON the phantom, NOT nulled -> the cue-gated path ADMITS it.
    ctx_bug = SeekerSeedContext()
    ctx_bug.update(0.0, 0.0, 0.0, (1.8, 0.0), False, cam_track=(20.0, 0.0))
    assert _tracker(ctx_bug)._seed_ok(phantom) is True

    # (b) the fix: cue nulled -> decision is camera-track-only -> phantom REJECTED,
    #     and NO cue is observable to the seed (legal_cue_pos() is None).
    ctx_fix = SeekerSeedContext()
    ctx_fix.update(0.0, 0.0, 0.0, None, False, cam_track=(20.0, 0.0))
    assert ctx_fix.legal_cue_pos() is None
    assert _tracker(ctx_fix)._seed_ok(phantom) is False


# ---------------------------------------------------------------- m4 wiring pins
# AST, not substring (the project idiom -- tests/test_honesty_static.py): pins
# STRUCTURE and survives reformatting/reindent.
def _m4_src():
    with open(M4_PATH, encoding="utf-8") as f:
        return f.read()


def _m4_tree():
    return ast.parse(_m4_src(), filename=M4_PATH)


def _assign_values(tree, target_name):
    """RHS value nodes of every `<target_name> = ...` assignment in the tree."""
    out = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == target_name:
                    out.append(node.value)
    return out


def test_m4_exposes_cue_stale_horizon_flag():
    # add_argument("--cue-stale-horizon", ..., default=COAST_STALE_S, ...)
    calls = [
        n for n in ast.walk(_m4_tree())
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute) and n.func.attr == "add_argument"
        and n.args and isinstance(n.args[0], ast.Constant)
        and n.args[0].value == "--cue-stale-horizon"
    ]
    assert calls, "the --cue-stale-horizon flag is missing from m4's argparse"
    default_kw = next((k for k in calls[0].keywords if k.arg == "default"), None)
    assert (default_kw is not None and isinstance(default_kw.value, ast.Name)
            and default_kw.value.id == "COAST_STALE_S"), (
        "--cue-stale-horizon must default to the COAST_STALE_S name (one concept, "
        "one number) so the fix is DEFAULT-ON but inert under a live cue")


def test_m4_predicate_guarded_post_handoff():
    # Fable-review fix: cue_is_stale_now = (not handoff_done) and cue_is_stale(...)
    # -> forced False post-handoff (no false 'cue STALE' marker/notice mid-ENGAGE,
    # post-handoff guidance + CSV byte-identity restored). Staleness is only
    # meaningful pre-handoff (the cue channel is closed one-way at the latch).
    vals = _assign_values(_m4_tree(), "cue_is_stale_now")
    assert vals, "no assignment to cue_is_stale_now found"
    v = vals[-1]
    assert isinstance(v, ast.BoolOp) and isinstance(v.op, ast.And), (
        "cue_is_stale_now must be `(not handoff_done) and cue_is_stale(...)` -- a "
        "BoolOp(And); the guard prevents the post-handoff false-STALE defect")
    first = v.values[0]
    assert (isinstance(first, ast.UnaryOp) and isinstance(first.op, ast.Not)
            and isinstance(first.operand, ast.Name)
            and first.operand.id == "handoff_done"), (
        "the FIRST operand of cue_is_stale_now must be `not handoff_done` (the "
        "post-handoff guard)")
    assert any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
               and n.func.id == "cue_is_stale" for n in ast.walk(v)), (
        "cue_is_stale_now must still call cue_is_stale(...) as its stale test")


def test_m4_seed_feed_nulls_cue_under_staleness():
    # seed_cue_pos = None if cue_is_stale_now else last_cue_pos ; fed to update()
    tree = _m4_tree()
    vals = _assign_values(tree, "seed_cue_pos")
    assert vals, "no assignment to seed_cue_pos found (the staleness-nulled seed)"
    v = vals[-1]
    assert (isinstance(v, ast.IfExp)
            and isinstance(v.test, ast.Name) and v.test.id == "cue_is_stale_now"
            and isinstance(v.body, ast.Constant) and v.body.value is None
            and isinstance(v.orelse, ast.Name) and v.orelse.id == "last_cue_pos"), (
        "seed_cue_pos must be `None if cue_is_stale_now else last_cue_pos` (ADR-0059)")
    update_calls = [
        n for n in ast.walk(tree)
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
        and n.func.attr == "update"
        and isinstance(n.func.value, ast.Name) and n.func.value.id == "seed_ctx"
    ]
    assert update_calls, "seed_ctx.update(...) call not found"
    assert any(
        any(isinstance(a, ast.Name) and a.id == "seed_cue_pos" for a in c.args)
        for c in update_calls), (
        "seed_ctx.update must be fed seed_cue_pos (the staleness-nulled cue), not "
        "the raw frozen last_cue_pos")


def test_m4_handoff_gate_skipped_under_staleness():
    # The handoff cue-gate condition must carry `and not cue_is_stale_now`: some
    # BoolOp(And) has `not cue_is_stale_now` among its operands. (The predicate's
    # own `(not handoff_done) and ...` BoolOp does NOT match -- it negates
    # handoff_done, not cue_is_stale_now.)
    found = any(
        isinstance(node, ast.BoolOp) and isinstance(node.op, ast.And)
        and any(isinstance(val, ast.UnaryOp) and isinstance(val.op, ast.Not)
                and isinstance(val.operand, ast.Name)
                and val.operand.id == "cue_is_stale_now"
                for val in node.values)
        for node in ast.walk(_m4_tree())
    )
    assert found, (
        "the handoff cue-gate must be skipped when the cue is stale: no "
        "`and not cue_is_stale_now` operand found in any BoolOp (ADR-0059)")
