"""Producer->consumer contract for the range-calibration sidecar.

THE SEAM: `scripts/seeker/calibrate_range.py` WRITES `<weights>.calib.json`;
`flight/deploy/seeker_loop.py:resolve_span` READS it. Nothing tested the pair
until 2026-08-10, and the pair is where the defect lived.

THE DEFECT THIS FILE EXISTS TO KEEP FIXED. The producer grades its own fit --
`return 0 if n_kept >= MIN_KEPT_FOR_TRUSTWORTHY_FIT else 2` -- but it writes the
sidecar BEFORE that return, and the consumer never read `n_kept`. So a 3-sample
fit the producer itself called a FAILURE was consumed as `calibrated=True`,
indistinguishable from a 57-sample fit. That is not cosmetic: `span_m_eff` sets

    range = fx * span_m_eff / box_width_px

and range multiplies Vc into the pro-nav command `a_cmd = N * Vc * lambda_dot`,
as well as keying every range threshold (closing-speed throttle, terminal
freeze, breakoff arm, hard floor). An unvetted span STEERS THE AIRCRAFT.

Fixture policy (docs/error_handling_policy.md): sidecars here are built by
calling the producer's OWN result-dict construction where practical and by
mirroring its exact key set otherwise -- never a hand-invented schema, which is
what hid two prior schema breaks in this repo.
"""
import json
import os
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO, os.path.join(REPO, "scripts", "seeker")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from flight.deploy.seeker_loop import (  # noqa: E402
    CALIB_MIN_KEPT, GuidanceConfig, resolve_span,
)


def _producer_floor():
    """The producer's floor, imported from the producer itself.

    calibrate_range imports cv2/numpy/torch-adjacent modules that are not in
    every venv, so this skips rather than fails when the import is unavailable
    -- but it NEVER silently passes: the drift assertion below is the point of
    the test, and a skip is visible in `pytest -rs` (run_tests.sh stage 1 now
    passes -rs for exactly this reason).
    """
    try:
        from calibrate_range import MIN_KEPT_FOR_TRUSTWORTHY_FIT
    except Exception as exc:  # pragma: no cover - env-dependent
        pytest.skip(f"calibrate_range not importable in this venv: {exc}")
    return MIN_KEPT_FOR_TRUSTWORTHY_FIT


def _write_sidecar(tmp_path, **overrides):
    """A sidecar with the producer's exact key set. Keys mirror the `result`
    dict in calibrate_range.main()."""
    weights = tmp_path / "fake.onnx"
    weights.write_text("not a real model")
    blob = {
        "span_m_eff": 0.42,
        "n_kept": 57,
        "iqr": [0.40, 0.44],
        "fit": "median(range_gt*bw_px/FX)",
        "images": "irrelevant",
        "weights": "fake.onnx",
        "min_kept_required": CALIB_MIN_KEPT,
        "trustworthy": True,
    }
    blob.update(overrides)
    for k in [k for k, v in blob.items() if v is _OMIT]:
        del blob[k]
    (tmp_path / "fake.onnx.calib.json").write_text(json.dumps(blob))
    return str(weights)


class _Omit:
    pass


_OMIT = _Omit()


def test_the_two_floors_have_not_drifted():
    """The deployed constant and the producer's must stay equal.

    They are DUPLICATED on purpose -- flight/ must not import from scripts/,
    which is dev tooling absent from the aircraft -- so this test is the thing
    that keeps them honest."""
    assert CALIB_MIN_KEPT == _producer_floor(), (
        "flight/deploy/seeker_loop.CALIB_MIN_KEPT and "
        "scripts/seeker/calibrate_range.MIN_KEPT_FOR_TRUSTWORTHY_FIT disagree. "
        "They are two copies of one contract; change both or neither.")


def test_a_good_fit_is_accepted_and_reports_its_sample_count(tmp_path):
    w = _write_sidecar(tmp_path)
    span, source, calibrated = resolve_span(w, GuidanceConfig())
    assert calibrated is True
    assert span == pytest.approx(0.42)
    # The count travels into the source string so a run log records the
    # evidence, not just the conclusion.
    assert "n=57" in source


def test_an_under_powered_fit_is_REFUSED(tmp_path):
    """THE REGRESSION. A fit the producer graded a failure must not be
    consumed as calibrated."""
    w = _write_sidecar(tmp_path, n_kept=3, trustworthy=False)
    span, source, calibrated = resolve_span(w, GuidanceConfig())
    assert calibrated is False, (
        "an under-powered calibration was accepted as calibrated -- this is the "
        "2026-08-10 defect returning; span_m_eff multiplies into a_cmd")
    assert span == GuidanceConfig().target_span_m
    assert "UNDER-POWERED" in source and "3" in source


def test_the_refusal_uses_the_sidecars_own_required_floor(tmp_path):
    """A sidecar written by a future producer with a STRICTER floor is honoured
    at that floor, not at the consumer's older copy."""
    w = _write_sidecar(tmp_path, n_kept=30, min_kept_required=50,
                       trustworthy=False)
    _span, source, calibrated = resolve_span(w, GuidanceConfig())
    assert calibrated is False and "30<50" in source


def test_an_ungraded_legacy_sidecar_fails_CLOSED(tmp_path):
    """Sidecars written before this contract carry no n_kept. An unmeasurable
    quality must be treated as untrustworthy, never assumed good."""
    w = _write_sidecar(tmp_path, n_kept=_OMIT, min_kept_required=_OMIT,
                       trustworthy=_OMIT)
    _span, source, calibrated = resolve_span(w, GuidanceConfig())
    assert calibrated is False and "UNGRADED" in source


def test_a_corrupt_sidecar_is_distinguishable_from_a_missing_one(tmp_path):
    """'You never calibrated' and 'your calibration is broken' must not print
    the same string -- they call for different actions."""
    weights = tmp_path / "fake.onnx"
    weights.write_text("x")
    (tmp_path / "fake.onnx.calib.json").write_text('{"span_m_eff": 0.4')  # truncated
    _s, corrupt_source, corrupt_cal = resolve_span(str(weights), GuidanceConfig())

    clean = tmp_path / "sub"
    clean.mkdir()
    w2 = clean / "fake.onnx"
    w2.write_text("x")
    _s2, missing_source, missing_cal = resolve_span(str(w2), GuidanceConfig())

    assert corrupt_cal is False and missing_cal is False
    assert corrupt_source != missing_source, (
        "a corrupt sidecar and an absent one produced the same report")
    assert "UNREADABLE" in corrupt_source
    assert "NOT CALIBRATED" in missing_source


def test_explicit_and_env_still_win_over_the_sidecar(tmp_path, monkeypatch):
    """The precedence order documented in resolve_span must not be disturbed by
    the new checks."""
    w = _write_sidecar(tmp_path, n_kept=3, trustworthy=False)
    span, source, calibrated = resolve_span(w, GuidanceConfig(), explicit_m=0.9)
    assert (span, calibrated) == (0.9, True) and "--target-span-m" in source

    monkeypatch.setenv("MARKERLESS_SPAN_M", "0.77")
    span, source, calibrated = resolve_span(w, GuidanceConfig())
    assert (span, calibrated) == (0.77, True) and "MARKERLESS_SPAN_M" in source


def test_the_producer_records_its_grade_in_the_file():
    """The producer must WRITE the grade, not just return it as an exit code --
    nothing downstream can see an exit code.

    Asserted against the producer's source rather than by running it (it needs
    images, cv2 and a model), which is why the runtime behaviour above is
    covered by the sidecar cases."""
    src = open(os.path.join(REPO, "scripts", "seeker",
                            "calibrate_range.py")).read()
    assert '"trustworthy"' in src or "result[\"trustworthy\"]" in src, (
        "calibrate_range no longer records `trustworthy` in the sidecar")
    assert "min_kept_required" in src, (
        "calibrate_range no longer records the floor it graded against")
