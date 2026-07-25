#!/usr/bin/env python3
"""REGRESSION: an interrupted capture must still be SCORABLE.

THE DEFECT (review 2, 2026-07-25 -- docs/review2_silent_failure_findings.md
"Ctrl-C at end of a pass silently destroys meta.json"). `record_session` wrote
meta.json only AFTER the frame loop completed; its `finally` closed the CSVs and
nothing caught KeyboardInterrupt. Yet Ctrl-C is the DOCUMENTED way to end a live
capture (pi_capture's own --n-frames help), and ending a pass over ssh with
Ctrl-C is the natural field workflow.

What a Ctrl-C'd pass destroyed, for frames that were already on disk:
  * `stream_fps`  -- the money gate's DIVISOR. Without it tripod_score returns
    UNCERTAIN (the gate refuses to invent a rate, and rightly).
  * `tag_size_m`  -- every tag-recovered range scales LINEARLY in it, so the
    scorer's fallback silently shortens the whole curve.
  * the exposure verification and n_frames; camera_session_summary exits 2
    ("that directory is not a pi_capture.py session").
One-shot field data, unrecoverable after the window closes.

THE FIX pinned here: meta.json is written from a `finally` on EVERY exit path, a
SIGTERM handler makes `systemctl stop` unwind the same way, the session
self-labels `terminated_early` / `capture_truncated`, and a short capture returns
non-zero instead of reporting success.

Also pinned: `decode_loop_fps`, the WRITE-EXCLUSIVE grab+decode cadence. The
gate models the flight decode rate but was being fed `stream_fps`, which is the
disk-bound recorder loop (it pays a per-frame PNG write the flight loop never
pays, and with decode off pays no decode cost at all).

Offline: a synthetic Frame generator, no camera, no cv2 backend.

Run: .venv/bin/python -m pytest tests/test_pi_capture_crashsafe.py -v
"""
import csv
import json
import os
import sys

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("cv2")

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEEKER = os.path.join(REPO_ROOT, "scripts", "seeker")
sys.path.insert(0, SEEKER)

import pi_capture as PC                                       # noqa: E402
import tripod_score as TS                                     # noqa: E402

FRAME_DT = 0.05          # 20 Hz


def _frames(n, interrupt_after=None, exc=KeyboardInterrupt):
    t = 0.0
    for i in range(n):
        if interrupt_after is not None and i == interrupt_after:
            raise exc("simulated stop")
        yield PC.Frame(np.zeros((16, 16), dtype=np.uint8), t,
                       1_752_700_000.0 + t, 900.0, 1.0)
        t += FRAME_DT


def _record(out_dir, gen, requested=None, source="picamera2"):
    """`source` defaults to picamera2 (2026-07-25): tripod_score's fps resolver
    now WHITELISTS the capture sources that may feed the money gate's 1/fps
    divisor, and only the real Pi 5 path qualifies. The fixture must therefore
    say which capture it is standing in for -- the interrupt/truncation
    behaviour under test is backend-independent, while the fps assertions are
    specifically about a session whose rate the gate is ALLOWED to consume."""
    return PC.record_session(gen, str(out_dir), source, {}, 16, 16,
                             PC.DEF_EXPOSURE_US, 1.0, False, None, 0.35,
                             "pytest", requested_n_frames=requested)


def test_ctrl_c_mid_capture_still_writes_meta_json(tmp_path):
    """THE headline. PRE-FIX: frames/ and index.csv exist, meta.json does NOT."""
    meta = _record(tmp_path, _frames(50, interrupt_after=7), requested=300)
    meta_path = tmp_path / "meta.json"
    assert meta_path.exists(), "an interrupted pass must still be scorable"

    on_disk = json.loads(meta_path.read_text())
    assert on_disk["n_frames"] == 7
    assert on_disk["terminated_early"] is True
    assert on_disk["capture_truncated"] is True
    assert on_disk["requested_n_frames"] == 300
    assert "KeyboardInterrupt" in str(on_disk["terminated_early_reason"])
    assert meta == on_disk or meta["n_frames"] == on_disk["n_frames"]


def test_the_money_gates_inputs_survive_the_interrupt(tmp_path):
    """The point of the fix is not the file, it is these three numbers: the rate
    the gate divides by, the tag edge every range scales by, and the exposure
    verification. All measured from the frames that DID land."""
    _record(tmp_path, _frames(50, interrupt_after=12), requested=300)
    meta = json.loads((tmp_path / "meta.json").read_text())

    assert meta["stream_fps"] == pytest.approx(1.0 / FRAME_DT, abs=1e-6)
    assert "measured" in meta["stream_fps_source"]
    assert meta["stream_fps_p10_slow_tail"] is not None
    assert meta["tag_size_m"] == 0.35
    assert meta["applied_exposure_us"] == 900.0
    assert meta["exposure_meets_spec"] is True

    # ...and the scorer accepts that rate, i.e. the gate is decidable again.
    fps, src = TS.resolve_stream_fps(type("A", (), {"stream_fps": None})(), meta)
    assert fps == pytest.approx(20.0)
    assert "meta.json" in src


def test_index_csv_covers_every_captured_frame_after_an_interrupt(tmp_path):
    """Rows are flushed periodically and the CSVs are closed in the same
    `finally`, so a truncated pass loses at most the frames it never grabbed --
    not a buffer's worth of rows for frames that ARE on disk."""
    _record(tmp_path, _frames(200, interrupt_after=95), requested=200)
    with open(tmp_path / "index.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    assert list(rows[0].keys()) == PC.INDEX_HEADER
    assert len(rows) == 95
    for r in rows:
        assert (tmp_path / r["frame_path"]).exists()


def test_a_backend_exception_also_finalizes_then_propagates(tmp_path):
    """SIGTERM/backend death is the SILENT variant (no traceback on the operator's
    terminal). meta.json must exist and label the cause, and the exception must
    still reach the caller rather than being swallowed."""
    with pytest.raises(RuntimeError):
        _record(tmp_path, _frames(50, interrupt_after=5, exc=RuntimeError),
                requested=50)
    meta = json.loads((tmp_path / "meta.json").read_text())
    assert meta["n_frames"] == 5
    assert meta["terminated_early"] is True
    assert "RuntimeError" in str(meta["terminated_early_reason"])


def test_a_complete_capture_is_not_flagged_truncated(tmp_path):
    meta = _record(tmp_path, _frames(30), requested=30)
    assert meta["n_frames"] == 30
    assert meta["terminated_early"] is False
    assert meta["capture_truncated"] is False


def test_a_short_capture_is_flagged_even_without_an_exception(tmp_path):
    """The v4l2 grab-death path: the backend just stops. 12 frames into a
    300-frame pass used to print 'wrote 12 frames' and exit 0."""
    meta = _record(tmp_path, _frames(12), requested=300)
    assert meta["n_frames"] == 12
    assert meta["terminated_early"] is False      # no exception, just short
    assert meta["capture_truncated"] is True


def test_sigterm_handler_is_installable():
    """`systemctl stop` must unwind through the finally, not hard-kill."""
    import signal
    prev = signal.getsignal(signal.SIGTERM)
    try:
        PC._install_sigterm_handler()
        h = signal.getsignal(signal.SIGTERM)
        assert callable(h) and h is not signal.SIG_DFL
        with pytest.raises(KeyboardInterrupt):
            h(signal.SIGTERM, None)
    finally:
        signal.signal(signal.SIGTERM, prev)


# ------------------------------------------------- two rates, not one
def test_decode_loop_fps_is_recorded_separately_from_stream_fps(tmp_path):
    """`stream_fps` is the DISK-BOUND recorder loop; the gate models the onboard
    DECODE cadence. Recording one number and consuming it as the other is the
    'measured number of the wrong quantity' defect."""
    pytest.importorskip("pupil_apriltags")
    meta = PC.record_session(_frames(20), str(tmp_path), "picamera2", {}, 16, 16,
                             PC.DEF_EXPOSURE_US, 1.0, True, None, 0.35,
                             "pytest", requested_n_frames=20)
    assert meta["decode_loop_fps"] is not None and meta["decode_loop_fps"] > 0
    assert "PNG WRITE EXCLUDED" in meta["decode_loop_fps_source"]
    assert "INCLUDES the per-frame PNG write" in meta["stream_fps_source"]
    # the scorer PREFERS the decode-inclusive rate when it exists
    fps, src = TS.resolve_stream_fps(type("A", (), {"stream_fps": None})(), meta)
    assert fps == meta["decode_loop_fps"]
    assert "decode_loop_fps" in src


def test_a_decode_free_session_is_flagged_as_the_wrong_quantity(tmp_path):
    """--no-decode-tags records a rate with ZERO decode cost. The scorer must
    say so in the provenance string rather than presenting it as the modelled
    handoff cadence."""
    meta = _record(tmp_path, _frames(20), requested=20)
    assert meta["tag_decode"] is False
    assert meta["decode_loop_fps"] is None
    _fps, src = TS.resolve_stream_fps(type("A", (), {"stream_fps": None})(), meta)
    assert "WRONG QUANTITY" in src
    assert "tag decode OFF" in src


def test_a_desk_rehearsal_may_not_supply_the_money_gates_fps(tmp_path):
    """BUILDER RULING 2026-07-25 (review 2's open question, answered NO): a
    `--local`/v4l2 DESK REHEARSAL -- a laptop webcam on a desk -- must never
    supply the ~$740 gate's 1/fps divisor.

    THE DEFECT: tripod_score's source check was a BLACKLIST (it rejected only
    `stream_fps_source` strings containing "replay") and never looked at
    `meta["source"]`, which pi_capture already writes. So a v4l2 desk session
    with tag decode ON produced a `decode_loop_fps` the gate consumed as the
    modelled quantity -- a rate measured on a different CPU, driver and sensor
    from the Pi 5 the burn model is about. It is now a WHITELIST."""
    meta = _record(tmp_path, _frames(20), requested=20, source="v4l2")
    assert meta["source"] == "v4l2" and meta["stream_fps"] is not None, (
        "the recorder still MEASURES the rate -- the refusal is the scorer's job")

    fps, src = TS.resolve_stream_fps(type("A", (), {"stream_fps": None})(), meta)
    assert fps is None, "a desk-rehearsal rate must not reach the gate"
    assert "REFUSED" in src and "v4l2" in src

    # ...and the refusal reaches the VERDICT: UNCERTAIN, never a PASS/FAIL.
    g = TS.gate_verdict(7.10, 7.10, 0.9, fps, 5, 9.0, 0.5, n_decoded=40)
    assert g["verdict"] == "UNCERTAIN" and "stream_fps" in g["reason"]

    # The sanctioned override is the protocol §7.3 bench, passed explicitly.
    fps2, src2 = TS.resolve_stream_fps(type("A", (), {"stream_fps": 20.0})(), meta)
    assert fps2 == 20.0 and "--stream-fps" in src2


def test_the_fps_source_check_is_a_whitelist_not_a_blacklist():
    """Pins the SHAPE of the guard, not just one rejected value: an unknown or
    absent `source` must fail closed. A blacklist is wrong the moment a new
    capture backend appears; this enumerates the one sanctioned case."""
    A = type("A", (), {"stream_fps": None})
    assert TS.ALLOWED_FPS_CAPTURE_SOURCES == ("picamera2",)
    ok, _ = TS.capture_source_allowed({"source": "picamera2"})
    assert ok
    for bad in ({"source": "v4l2"}, {"source": "dir"}, {"source": "libcamera-vid"},
                {"source": ""}, {}):
        ok, detail = TS.capture_source_allowed(bad)
        assert not ok, f"{bad} must not be a sanctioned gate-fps source"
        assert detail
    # a rate attached to a non-whitelisted source is refused, with the number
    # still visible in the message so the operator can see what was rejected
    fps, src = TS.resolve_stream_fps(A(), {"source": "libcamera-vid",
                                           "decode_loop_fps": 41.0})
    assert fps is None and "REFUSED" in src


def test_replay_sessions_still_publish_no_stream_fps(tmp_path):
    """Unchanged guarantee: the dir-replay backend's timestamps are synthetic."""
    meta = PC.record_session(_frames(20), str(tmp_path), "dir", {}, 16, 16,
                             PC.DEF_EXPOSURE_US, 1.0, False, None, 0.35,
                             "pytest", requested_n_frames=20)
    assert meta["stream_fps"] is None
    assert "replay" in meta["stream_fps_source"].lower()
    fps, src = TS.resolve_stream_fps(type("A", (), {"stream_fps": None})(), meta)
    assert fps is None and "NOT SUPPLIED" in src
