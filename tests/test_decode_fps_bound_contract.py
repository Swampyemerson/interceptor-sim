"""Producer->consumer contract: pi_capture's decode_loop_fps -> tripod_score's gate.

THE BREAK THIS EXISTS TO CATCH (2026-07-26)
-------------------------------------------
`tripod_score.resolve_stream_fps` PREFERS `meta.json decode_loop_fps` as the money
gate's 1/fps divisor. That field was an UNBOUNDED decode throughput -- an upper
bound on a cadence, not a cadence -- until pi_capture started clamping it by the
delivered frame cadence (commit 1b89340).

It bit for real: the 2026-07-26 skr-05 bench session ran on a Pi checked out at
15b88b0, one commit before the clamp, and published decode_loop_fps=89.5 on a rig
whose camera was delivering 30.14 fps. Feeding that session to the gate would have
understated R_streak_burn by ~3x, in the flattering direction for a ~$740 purchase.
meta.json carried the git_rev that would have exposed it and nothing read it.

Both scripts passed their own self-tests throughout. Nothing tested the PAIR --
which is this project's documented recurring failure mode, so the fixture below
comes from pi_capture's OWN source-string builder rather than a hand-typed copy.
If the producer's wording drifts away from what the consumer matches on, this
fails instead of the gate silently accepting an unbounded number again.
"""
import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SEEKER = os.path.join(os.path.dirname(HERE), "scripts", "seeker")
if SEEKER not in sys.path:
    sys.path.insert(0, SEEKER)

pi_capture = pytest.importorskip("pi_capture")
tripod_score = pytest.importorskip("tripod_score")


class _Args:
    """resolve_stream_fps reads .stream_fps off its args object."""
    stream_fps = None


def _meta(**over):
    """A meta.json that passes the capture-source whitelist, so the ONLY thing
    under test is the decode-rate bound."""
    m = {
        "source": "picamera2",
        "tag_decode": True,
        "git_rev": "deadbee",
        "stream_fps": 30.14,
        "stream_fps_source": "measured: median inter-frame dt (t_mono_s)",
        "decode_loop_fps": 89.532,
    }
    m.update(over)
    return m


def test_bounded_source_is_accepted():
    """The CURRENT producer's own string must satisfy the consumer."""
    src = pi_capture.bounded_decode_source(117.4, 30.01)
    fps, why = tripod_score.resolve_stream_fps(
        _Args(), _meta(decode_loop_fps=30.01, decode_loop_fps_source=src))
    assert fps == pytest.approx(30.01), why
    assert "REFUSED" not in why


def test_producer_string_carries_the_mark_the_consumer_matches():
    """The contract itself: drift in either literal breaks the pair."""
    src = pi_capture.bounded_decode_source(117.4, 30.01)
    assert tripod_score.BOUNDED_MARK in src.lower()


def test_preclamp_source_is_refused():
    """The exact shape of the 15b88b0 session: a real Pi, a real picamera2
    capture, tag decode on -- and an UNBOUNDED throughput."""
    preclamp = ("measured: median AprilTag decode wall-time per frame, PNG "
                "WRITE EXCLUDED (the modelled handoff cadence)")
    fps, why = tripod_score.resolve_stream_fps(
        _Args(), _meta(decode_loop_fps_source=preclamp, git_rev="15b88b0"))
    assert fps is None, f"an unbounded 89.5 fps reached the gate: {why}"
    assert "REFUSED" in why
    assert "15b88b0" in why          # the refusal names the culprit rev


def test_absent_source_is_refused():
    """No source string at all is not a licence to trust the number."""
    fps, why = tripod_score.resolve_stream_fps(_Args(), _meta())
    assert fps is None, why
    assert "REFUSED" in why


def test_explicit_bench_fps_still_wins():
    """--stream-fps from the §7.3 soak outranks anything in meta.json, even a
    refusable one -- the bench number is the intended path."""
    a = _Args()
    a.stream_fps = 111.8
    fps, why = tripod_score.resolve_stream_fps(a, _meta())
    assert fps == pytest.approx(111.8)
    assert "explicit" in why.lower()


def test_real_session_on_disk_would_have_been_refused():
    """Regression on the ACTUAL skr-05 artefact: its meta must not be able to
    feed the ~$740 gate.

    PATH CORRECTED 2026-08-10. This looked for `<repo>/scripts/seeker/sessions/
    smoke/meta.json`, a directory that has never existed in any commit on any
    branch -- so the test skipped on every run since it was written and the
    regression never once executed against the real file. Meanwhile the real
    artefact was rescued off the Pi's SD card the same day (commit 29ec8bc) to
    `runs/skr05_smoke_session/meta.json`, carrying exactly the pathology this
    test is about: decode_loop_fps 89.532 published with no delivered-cadence
    clamp, from a session recorded at git_rev 15b88b0 (pre-1b89340).

    The lesson is the one the project already writes down: a guard that skips is
    not a guard. It is now pointed at the committed artefact and FAILS CLOSED if
    that artefact goes missing, rather than skipping quietly.
    """
    cand = os.path.join(REPO, "runs", "skr05_smoke_session", "meta.json")
    assert os.path.exists(cand), (
        f"the skr-05 session meta is missing from the repo at {cand}. It is "
        f"COMMITTED evidence (29ec8bc) and this regression depends on it; if it "
        f"was deleted, restore it rather than letting this test skip.")
    with open(cand) as fh:
        meta = json.load(fh)
    assert meta.get("decode_loop_fps"), (
        "the skr-05 meta lost its decode_loop_fps field -- this test cannot "
        "verify the refusal without it, and a vacuous pass is not acceptable")
    fps, why = tripod_score.resolve_stream_fps(_Args(), meta)
    assert fps is None, f"pre-clamp session still feeds the gate: {why}"
    # The refusal must NAME the culprit, not merely decline: an operator has to
    # learn which capture rev produced the bad number.
    assert "15b88b0" in why or "clamp" in why.lower(), (
        f"the refusal did not explain itself: {why}")
