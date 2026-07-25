#!/usr/bin/env python3
"""REGRESSION: R_decode90 must be MEASURED, not inferred across missing data.

THE DEFECT (review 2, 2026-07-25 -- docs/review2_silent_failure_findings.md
"R_decode90 walks across UNSAMPLED range bins and can be set by a single decoded
frame"). `tripod_score.curve_a` walked the >=90% decode band with

    for lo in sorted(bins):        # <-- only the bins that EXIST
        if rate >= 0.9: r90 = lo + bin_m
        else: break

A range bin holding zero binnable frames is simply ABSENT from `bins`, so the
walk JUMPED coverage holes and kept extending R_decode90 into ranges the session
never sampled. There was no per-bin sample floor either, so ONE lucky far decode
read rate 1.00 and pushed the envelope out by a whole bin on n=1 -- and
`decode_rate_near` then handed the ~$740 money gate p=1.00 measured on that
single frame. The only guard was a session-wide floor of 5 DECODED frames.

Both failure shapes are ordinary field events: a GPS/telemetry dropout whose
frames all carry range_quality=gap, an occluded mid-band, or two passes shot at
different stations. The output was a clean bin table and GATE PASS.

THE FIX pinned here: a STEPPED walk (`lo += bin_m`) that stops on a hole
(`bin_absent`), on an underpopulated bin (`bin_underpopulated`, --min-bin-n,
default 5), or at the end of the data (`end_of_data`) -- and a verdict policy in
which only `rate_below_90` (a MEASURED cutoff) may yield PASS/FAIL. The three
data-bounded reasons mean R_decode90 is a LOWER BOUND set by what was never
shot, so the gate returns UNCERTAIN.

Every test below FAILS against the pre-fix code and PASSES after. Pure
arithmetic -- no session, no camera, no sim.

Run: .venv/bin/python -m pytest tests/test_r90_walk_and_gate.py -v
"""
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts", "seeker"))

import tripod_score as TS                                    # noqa: E402


def _bins(*rows):
    """rows: (lo, n_decoded, n_total) -> curve_a's bins dict shape."""
    return {lo: [dec, tot, []] for lo, dec, tot in rows}


# --------------------------------------------------------------- (i) the hole
def test_mid_band_hole_stops_the_walk_and_forces_UNCERTAIN():
    """A session sampled at 4-8 m and again at 12-16 m, decoding 100% in both.

    PRE-FIX: `for lo in sorted(bins)` walks 4,6,12,14 with every rate 1.0 and
    reports R_decode90 = 16 m -- an envelope that claims a >=90% decode rate
    across 8-12 m, where NOT ONE FRAME was ever binned. At 9 m/s / p=0.9 / k=5 /
    30 fps that is t_go = (16 - 2.08)/9 = 1.55 s: a comfortable GATE PASS built
    on ranges nobody shot.

    POST-FIX: the walk stops AT the 8 m hole (r90 = 8 m) and the verdict is
    UNCERTAIN naming the station to re-shoot.
    """
    bins = _bins((4.0, 10, 10), (6.0, 10, 10), (12.0, 10, 10), (14.0, 10, 10))
    r90, why, stop_lo, n_at, gaps = TS.r90_walk(bins, 2.0)

    assert r90 == 8.0, "the walk must stop at the hole, not jump it to 16 m"
    assert why == TS.R90_STOP_ABSENT
    assert stop_lo == 8.0
    assert gaps == [8.0, 10.0], "every unsampled bin must be reported"

    gate = TS.gate_verdict(r90, 16.0, 1.0, 30.0, 5, 9.0, 0.5, n_decoded=40,
                           r90_stop_reason=why, r90_stop_lo=stop_lo,
                           n_at_r90_bin=n_at)
    assert gate["verdict"] == "UNCERTAIN", (
        "an envelope bounded by MISSING DATA may never PASS the $740 gate")
    assert gate["r90_data_bounded"] is True
    assert "COVERAGE HOLE" in gate["reason"] and "8.00 m" in gate["reason"]
    assert "RE-SHOOT" in gate["reason"]


def test_the_hole_verdict_is_fps_independent():
    """No frame rate can rescue an envelope whose outer edge was never sampled,
    so the data-bounded leg is decided BEFORE the fps check (and still returns
    the same UNCERTAIN when fps is missing too)."""
    bins = _bins((4.0, 10, 10), (6.0, 10, 10), (12.0, 10, 10))
    r90, why, stop_lo, n_at, _ = TS.r90_walk(bins, 2.0)
    for fps in (None, 14.0, 20.0, 30.0, 200.0):
        g = TS.gate_verdict(r90, 12.0, 1.0, fps, 5, 9.0, 0.5, n_decoded=40,
                            r90_stop_reason=why, r90_stop_lo=stop_lo,
                            n_at_r90_bin=n_at)
        assert g["verdict"] == "UNCERTAIN", f"fps={fps} must not decide this"


# ------------------------------------------------------- (ii) the n=1 far bin
def test_a_single_far_decode_cannot_extend_the_envelope_or_set_the_gate_p():
    """ONE decoded frame in the 8-10 m bin reads rate 1.00.

    PRE-FIX: r90 = 10 m and `decode_rate_near` selected that same bin, handing
    the gate p = 1.00 measured on n = 1 -- the most optimistic possible input to
    a purchase gate, from a single frame.

    POST-FIX: the walk refuses to step onto a bin below --min-bin-n, so r90 = 8 m
    (set by the last bin with real support, n = 10), the gate's p comes from
    THAT bin, and the verdict is UNCERTAIN.
    """
    bins = _bins((4.0, 10, 10), (6.0, 10, 10), (8.0, 1, 1))
    r90, why, stop_lo, n_at, _gaps = TS.r90_walk(bins, 2.0)

    assert r90 == 8.0, "an n=1 bin must not extend the >=90% band"
    assert why == TS.R90_STOP_UNDERPOPULATED
    assert stop_lo == 8.0
    assert n_at == 10, "the reported support is the last SUSTAINING bin's n"

    p = TS.decode_rate_near(bins, r90, 2.0)
    assert p == 1.0
    # ...and it is the 6-8 m bin's rate (n=10), not the n=1 bin's.
    assert TS.r90_walk(bins, 2.0)[3] == 10

    gate = TS.gate_verdict(r90, 9.5, p, 30.0, 5, 9.0, 0.5, n_decoded=40,
                           r90_stop_reason=why, r90_stop_lo=stop_lo,
                           n_at_r90_bin=n_at)
    assert gate["verdict"] == "UNCERTAIN"
    assert gate["n_at_r90_bin"] == 10


def test_min_bin_n_is_configurable_and_5_by_default():
    bins = _bins((4.0, 10, 10), (6.0, 4, 4))
    assert TS.DEFAULT_MIN_BIN_N == 5
    assert TS.r90_walk(bins, 2.0)[0] == 6.0                    # n=4 refused
    assert TS.r90_walk(bins, 2.0, min_bin_n=4)[0] == 8.0       # n=4 accepted
    assert TS.r90_walk(bins, 2.0, min_bin_n=4)[1] == TS.R90_STOP_END


# ------------------------------------------------- (iii) the measured cutoff
def test_a_measured_cutoff_still_decides_PASS_or_FAIL():
    """The fix must not turn every session into UNCERTAIN: when the tag was
    actually TRIED at the next station and failed, that is a measurement and the
    gate still decides. This is the leg the whole tool exists for."""
    bins = _bins((4.0, 20, 20), (6.0, 20, 20), (8.0, 2, 20))
    r90, why, stop_lo, n_at, gaps = TS.r90_walk(bins, 2.0)
    assert (r90, why, gaps) == (8.0, TS.R90_STOP_RATE, [])

    p = TS.decode_rate_near(bins, r90, 2.0)
    g_pass = TS.gate_verdict(r90, 9.0, p, 30.0, 5, 9.0, 0.5, n_decoded=40,
                             r90_stop_reason=why, r90_stop_lo=stop_lo,
                             n_at_r90_bin=n_at)
    assert g_pass["verdict"] == "PASS"
    # the same optical data at the aggressive closing speed still FAILs
    g_fail = TS.gate_verdict(r90, 9.0, p, 30.0, 5, 20.0, 0.5, n_decoded=40,
                             r90_stop_reason=why, r90_stop_lo=stop_lo,
                             n_at_r90_bin=n_at)
    assert g_fail["verdict"] == "FAIL"


def test_walking_off_the_end_of_the_data_is_not_a_measured_ceiling():
    """Every sampled bin sustained >=90%: the tag's real ceiling is somewhere
    BEYOND the farthest station shot, so R_decode90 is a lower bound."""
    bins = _bins((4.0, 10, 10), (6.0, 10, 10), (8.0, 10, 10))
    r90, why, stop_lo, n_at, gaps = TS.r90_walk(bins, 2.0)
    assert (r90, why, gaps) == (10.0, TS.R90_STOP_END, [])
    g = TS.gate_verdict(r90, 10.0, 1.0, 30.0, 5, 9.0, 0.5, n_decoded=40,
                        r90_stop_reason=why, r90_stop_lo=stop_lo,
                        n_at_r90_bin=n_at)
    assert g["verdict"] == "UNCERTAIN"
    assert "END OF THE DATA" in g["reason"]


def test_the_nearest_bin_failing_on_RATE_is_still_a_FAIL_not_UNCERTAIN():
    """The fps-INDEPENDENT FAIL leg must survive the fix: sparse decodes with no
    >=90% band anywhere is a measured failure of the placard, not missing data."""
    bins = _bins((4.0, 2, 20), (6.0, 1, 20))
    r90, why, stop_lo, n_at, _ = TS.r90_walk(bins, 2.0)
    assert r90 == 0.0 and why == TS.R90_STOP_RATE
    g = TS.gate_verdict(r90, 6.5, 0.1, None, 5, 9.0, 0.5, n_decoded=40,
                        r90_stop_reason=why, r90_stop_lo=stop_lo,
                        n_at_r90_bin=n_at)
    assert g["verdict"] == "FAIL"


def test_an_underpopulated_NEAREST_bin_is_UNCERTAIN_not_FAIL():
    """r90 == 0 has two very different causes. Measured failure -> FAIL (above).
    Too few frames to measure anything -> UNCERTAIN, because 'we did not shoot
    enough' must never read as 'the placard is too small'."""
    bins = _bins((4.0, 0, 2),)
    r90, why, stop_lo, n_at, _ = TS.r90_walk(bins, 2.0)
    assert r90 == 0.0 and why == TS.R90_STOP_UNDERPOPULATED
    g = TS.gate_verdict(r90, 0.0, 0.0, 30.0, 5, 9.0, 0.5, n_decoded=40,
                        r90_stop_reason=why, r90_stop_lo=stop_lo,
                        n_at_r90_bin=n_at)
    assert g["verdict"] == "UNCERTAIN"


# --------------------------------------------------- (iv) the default is safe
def test_gate_verdict_default_stop_reason_preserves_pure_arithmetic_callers():
    """gate_verdict is also a pure unit under test elsewhere; a caller that
    supplies a hand-built r90 with no walk metadata must behave exactly as
    before the fix."""
    g = TS.gate_verdict(8.0, 8.0, 0.9, 30.0, 5, 9.0, 0.5, n_decoded=40)
    assert g["verdict"] == "PASS"
    assert g["r90_stop_reason"] == TS.R90_STOP_RATE
    assert g["r90_data_bounded"] is False


def test_curve_a_and_r90_walk_agree():
    """curve_a's r90 IS the stepped walk (they must never drift apart)."""
    frames = [(f"/x/{i:06d}.png", f"{i:06d}") for i in range(24)]
    # 12 frames at 5 m (all decode), 12 at 15 m (none) -> a 6-14 m hole.
    index_map, decode = {}, {}
    for i, (_p, b) in enumerate(frames):
        rng = 5.0 if i < 12 else 15.0
        index_map[b] = {"true_range_m": f"{rng}"}
        decode[b] = (rng < 10.0, rng if rng < 10.0 else None, 640.0, 400.0, 1)
    bins, r_any, r90, _acc, n_bin, _n_unbin = TS.curve_a(
        frames, decode, index_map, 2.0, 40.0)
    assert n_bin == 24
    assert r90 == TS.r90_walk(bins, 2.0)[0] == 6.0
    assert TS.r90_walk(bins, 2.0)[1] == TS.R90_STOP_ABSENT


# ------------------------------------------- (v) §8.2 must not be mis-answered
def test_section_8_2_is_unanswerable_when_no_cell_reaches_the_10_25m_band():
    """Curve (b) truths ONLY frames where the tag decoded (ceiling ~7.10 m
    predicted for the adopted 0.35 m placard), while protocol §8.2's threshold is
    '>=50% recall across the 10-25 m operational band'. Handing `overall_recall`
    over as the §8.2 answer quotes a number measured on a DIFFERENT range band.
    """
    near_only = {(4.0, "middle"): [3, 4], (6.0, "top"): [1, 4]}
    cov = TS.curve_b_band_coverage(near_only, 2.0)
    assert cov["answerable"] is False
    assert cov["n_frames_in_band"] == 0 and cov["n_frames_scored"] == 8
    assert cov["scored_lo_m"] == 4.0 and cov["scored_hi_m"] == 8.0

    with_far = dict(near_only)
    with_far[(10.0, "middle")] = [2, 5]
    cov2 = TS.curve_b_band_coverage(with_far, 2.0)
    assert cov2["answerable"] is True and cov2["n_frames_in_band"] == 5

    # A cell that merely OVERLAPS the band counts (8-10 m does not; 24-26 does).
    assert TS.curve_b_band_coverage({(8.0, "m"): [1, 3]}, 2.0)["answerable"] is False
    assert TS.curve_b_band_coverage({(24.0, "m"): [1, 3]}, 2.0)["answerable"] is True
    assert TS.curve_b_band_coverage({}, 2.0)["answerable"] is False


# --------------------------------------------- (vi) the tag-size single source
def test_default_tag_size_is_the_adopted_placard():
    """docs/placard_mount.md §2: tag black-square edge 0.350 m on a 0.450 m
    sheet. The scorer defaulted to 0.30, so any session whose meta.json lacked
    tag_size_m read every tag-recovered range ~14% short -- silently, in the
    accuracy column, the curve (b) truth boxes and range_truth_join's bias test.
    """
    assert TS.DEFAULT_TAG_SIZE_M == 0.35


def test_partial_tags_csv_is_refused_not_silently_scored_as_failures():
    """A tags.csv truncated by an interrupted capture or a partial copy used to
    turn its missing (typically NEAR, high-rate) frames into decode FAILURES,
    deflating curve (a) and collapsing R_decode90 with no error at all."""
    frames = [(f"/x/{i:06d}.png", f"{i:06d}") for i in range(10)]
    tags = {f"{i:06d}": {"n_tags": "1", "range_m": "5.0",
                         "center_u": "640", "center_v": "400"}
            for i in range(7)}                     # last 3 frames missing
    calib = (384.6, 384.6, 640.0, 400.0, 1280, 800, [0.0] * 5)

    with pytest.raises(SystemExit) as e:
        TS.decode_frames(frames, calib, 0.35, tags_map=tags, quiet=True)
    assert "tags.csv covers 7 of 10" in str(e.value)
    assert "--allow-partial-tags" in str(e.value)

    # The escape hatch still works and is explicit.
    dec = TS.decode_frames(frames, calib, 0.35, tags_map=tags, quiet=True,
                           allow_partial_tags=True)
    assert sum(1 for v in dec.values() if v[0]) == 7
