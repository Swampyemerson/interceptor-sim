#!/usr/bin/env python3
"""REGRESSION: the KILL/MISS scorer must not let the LOG RATE decide the verdict.

THE DEFECT (review 2, 2026-07-25 -- docs/review2_silent_failure_findings.md
"field_score.py reports the CPA as the minimum over a coarse resample grid").
`score_engagement` took CPA = min(range) over a uniform grid whose step was the
finer log's median sample interval clamped to <=0.2 s. A grid argmin carries a
quantization error of ~v_rel*dt/2 -- 1-2 m at a 20 m/s closing speed, i.e. 2-4x
the 0.5 m lethal radius. PX4's default logger writes vehicle_global_position
(the topic this scorer PREFERS) at 200 ms = 5 Hz, and no caller passes --dt, so
this WAS the production path: a true 0.30 m ram scored a confident MISS.

Compounding it, every case in field_score's own `--self-test` pinned dt=0.01, so
the auto-dt path the field actually runs was never exercised, and the ULog
reader -- the interceptor's own log, half the binary-kill verdict -- had no test
anywhere in the repo.

THE FIX pinned here:
  * the CPA is now the EXACT per-segment closed form on the UNION of both logs'
    sample times (both tracks are linear between their own samples, so this is
    exact for the interpolated tracks -- the quantization TERM is removed, not
    shrunk);
  * a minimum that sits at the edge of the logged overlap is not a closest
    approach at all -> verdict INCONCLUSIVE_LOG_TRUNCATED;
  * the ULog reader is driven end-to-end through a stub pyulog.

Everything is offline: synthetic straight-line tracks with an analytic ground
truth, plus a fake pyulog module. No sim, no logs, no network.

Run: .venv/bin/python -m pytest tests/test_field_score.py -v
"""
import importlib.util
import os
import subprocess
import sys

import numpy as np
import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SPEC = importlib.util.spec_from_file_location(
    "field_score", os.path.join(REPO_ROOT, "scripts", "field_score.py"))
FS = importlib.util.module_from_spec(_SPEC)
# register BEFORE exec: @dataclass resolves annotations via sys.modules
sys.modules["field_score"] = FS
_SPEC.loader.exec_module(FS)

LAT0, LON0, ALT0 = 34.05, -118.25, 100.0
BASE_UTC = 1_752_700_000.0


def _track(label, pos0, vel, duration, hz, t0_offset=0.0):
    """A constant-velocity synthetic track sampled at `hz`, its own start phase
    offset by `t0_offset` so its samples do NOT land on the other track's."""
    n = max(int(round(duration * hz)) + 1, 2)
    t_rel = np.linspace(0.0, duration, n) + t0_offset
    pos = np.asarray(pos0, float)[None, :] + t_rel[:, None] * np.asarray(vel, float)[None, :]
    lat, lon, alt = FS.enu_to_latlon(pos[:, 0], pos[:, 1], pos[:, 2], LAT0, LON0, ALT0)
    return FS.Track(label=label, t_utc_s=BASE_UTC + t_rel,
                    latlon=np.column_stack([lat, lon, alt]),
                    utc_synced=True, source="synthetic")


def _grid_argmin_cpa(track_a, track_b, dt=None):
    """What the PRE-FIX code reported: the minimum over the resample grid."""
    pa, pb = FS.project_to_common_enu(track_a, track_b)
    _t, _a, _b, rng, _dt = FS.compute_range_track(
        track_a.t_utc_s, pa, track_b.t_utc_s, pb, dt=dt)
    return float(np.min(rng))


# ------------------------------------------------------ THE verdict-flip case
@pytest.mark.parametrize("hz_a,hz_b", [(5.0, 5.0), (5.0, 10.0), (5.0, 25.0)])
def test_true_ram_is_a_KILL_at_every_px4_log_rate(hz_a, hz_b):
    """A true 0.300 m CPA at 12 m/s closing must score KILL at the ram radius
    regardless of how fast either flight controller logged.

    PRE-FIX the reported CPA was the grid argmin, which grows with the log
    period: this same geometry returned >0.5 m (a confident MISS) at PX4's
    default 5 Hz. The verdict must be a function of GEOMETRY, not of dt.
    """
    a = _track("interceptor", (-60.6, 0.30, 0.0), (12.0, 0.0, 0.0), 10.0, hz_a)
    b = _track("target", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 10.0, hz_b,
               t0_offset=0.15)          # adversarial phase: CPA falls off-grid
    res = FS.score_engagement(a, b, lethal_radius_m=FS.DEFAULT_LETHAL_RADIUS_M)

    assert res.cpa_m == pytest.approx(0.300, abs=0.005), (
        f"analytic CPA {res.cpa_m:.4f} m must not depend on the log rate")
    assert res.verdict == "KILL"
    assert res.cpa_t_utc_s == pytest.approx(BASE_UTC + 60.6 / 12.0, abs=0.02)


def test_the_grid_argmin_really_would_have_called_it_a_MISS():
    """Mechanism evidence for the test above: at PX4's default 5 Hz on BOTH
    sides, the pre-fix grid argmin on this exact data reads well past the ram
    radius. Without this the fix could be dismissed as a rounding tidy-up."""
    a = _track("interceptor", (-60.6, 0.30, 0.0), (12.0, 0.0, 0.0), 10.0, 5.0)
    b = _track("target", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 10.0, 5.0, t0_offset=0.15)
    grid = _grid_argmin_cpa(a, b, dt=None)
    res = FS.score_engagement(a, b, lethal_radius_m=FS.DEFAULT_LETHAL_RADIUS_M)

    assert grid > FS.DEFAULT_LETHAL_RADIUS_M, (
        f"expected the coarse-grid CPA ({grid:.3f} m) to exceed the "
        f"{FS.DEFAULT_LETHAL_RADIUS_M} m radius -- that is the defect")
    assert grid == pytest.approx(res.cpa_grid_m, abs=1e-9)
    assert res.cpa_m < FS.DEFAULT_LETHAL_RADIUS_M
    # the removed error is reported, not hidden
    assert res.cpa_grid_quantization_m > 0.5


def test_threshold_case_is_scored_against_the_module_constant():
    """A 1.050 m CPA sits STRICTLY between the ADR-0025 ram radius (0.5 m) and
    the net radius (1.5 m). Scored against DEFAULT_LETHAL_RADIUS_M it is a MISS
    -- and it flips to KILL if that constant ever regresses to the net number,
    which is what makes this a radius-regression guard and not just a CPA test.
    """
    a = _track("interceptor", (-60.6, 1.05, 0.0), (12.0, 0.0, 0.0), 10.0, 5.0)
    b = _track("target", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 10.0, 10.0, t0_offset=0.15)
    res = FS.score_engagement(a, b, lethal_radius_m=FS.DEFAULT_LETHAL_RADIUS_M)
    assert 0.5 < res.cpa_m < 1.5
    assert res.cpa_m == pytest.approx(1.050, abs=0.005)
    assert res.verdict == "MISS"
    assert FS.DEFAULT_LETHAL_RADIUS_M == 0.5, (
        "the ram radius is the default (the net was PARKED by builder ruling, "
        "contract stage `kill`); scoring a ram attempt at 1.5 m flatters it 3x")
    assert "net radius" not in FS._lethal_radius_help().lower(), (
        "the help text must not relabel the ram radius as the net radius")


def test_analytic_cpa_matches_the_independent_closed_form_on_a_crossing():
    """Cross-check against `_analytic_straight_line_cpa`, which is derived
    independently of the resampling path."""
    pA, vA = np.array([0.0, 0.0, 50.0]), np.array([12.0, 3.0, -1.0])
    pB, vB = np.array([100.0, -20.0, 45.0]), np.array([-4.0, 9.0, -0.3])
    t_star, cpa_true = FS._analytic_straight_line_cpa(pA - pB, vA - vB, 0.0, 10.0)
    a = _track("interceptor", pA, vA, 10.0, 5.0)
    b = _track("target", pB, vB, 10.0, 5.0, t0_offset=0.07)
    res = FS.score_engagement(a, b, lethal_radius_m=3.0)
    assert res.cpa_m == pytest.approx(cpa_true, abs=0.01)
    assert res.cpa_t_utc_s == pytest.approx(BASE_UTC + t_star, abs=0.02)


# ------------------------------------------------------- truncated-log honesty
def test_a_still_closing_log_is_INCONCLUSIVE_not_a_MISS():
    """The overlap window ends while the two aircraft are still converging, so
    the smallest range IN the window is a boundary value, not a CPA. Reporting
    it as a MISS would be a confident, plausible, wrong verdict on the one-shot
    kill day."""
    a = _track("interceptor", (-60.0, 0.30, 0.0), (12.0, 0.0, 0.0), 4.0, 5.0)
    b = _track("target", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 4.0, 10.0)
    res = FS.score_engagement(a, b, lethal_radius_m=FS.DEFAULT_LETHAL_RADIUS_M)
    assert res.verdict == FS.VERDICT_TRUNCATED
    assert res.cpa_at_window_edge is True and res.cpa_edge_side == "end"


def test_an_interior_minimum_is_never_flagged_truncated():
    a = _track("interceptor", (-60.6, 0.30, 0.0), (12.0, 0.0, 0.0), 10.0, 5.0)
    b = _track("target", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 10.0, 5.0, t0_offset=0.15)
    res = FS.score_engagement(a, b, lethal_radius_m=FS.DEFAULT_LETHAL_RADIUS_M)
    assert res.cpa_at_window_edge is False and res.verdict == "KILL"


def test_no_overlap_still_raises_rather_than_scoring():
    a = _track("a", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 5.0, 10.0)
    b = FS.Track(label="b", t_utc_s=a.t_utc_s + 100.0, latlon=a.latlon,
                 utc_synced=True, source="synthetic")
    with pytest.raises(ValueError):
        FS.score_engagement(a, b, lethal_radius_m=1.0)


# ---------------------------------------------------------- THE ULog reader
def _ulog_track(datasets, monkeypatch):
    import types
    mod = types.ModuleType("pyulog")
    mod.ULog = lambda _p: FS._StubULog(datasets)
    monkeypatch.setitem(sys.modules, "pyulog", mod)
    from pathlib import Path
    return FS.load_track_from_ulog(Path("stub.ulg"), "interceptor")


def _px4_datasets(n=50, hz=5.0, with_utc=True, low_amsl=3.0):
    t_boot_us = np.arange(n, dtype=float) * (1e6 / hz)
    utc_us = (t_boot_us + BASE_UTC * 1e6) if with_utc else np.zeros(n)
    east = np.linspace(0.0, 40.0, n)
    north = np.full(n, 5.0)
    up = np.full(n, low_amsl)
    lat, lon, alt = FS.enu_to_latlon(east, north, up, LAT0, LON0, ALT0)
    return t_boot_us, utc_us, lat, lon, alt, east, north, up


def test_ulog_prefers_vehicle_global_position_and_recovers_utc(monkeypatch):
    """PX4's EKF-fused topic is preferred over raw GPS, and the GPS UTC offset
    is applied so the two aircraft share a time axis."""
    t_boot, utc, lat, lon, alt, east, _n, up = _px4_datasets()
    tr = _ulog_track({
        "vehicle_gps_position": {"timestamp": t_boot, "time_utc_usec": utc,
                                 "lat": lat * 1e7, "lon": lon * 1e7, "alt": alt * 1e3},
        "vehicle_global_position": {"timestamp": t_boot, "lat": lat, "lon": lon,
                                    "alt": alt},
    }, monkeypatch)
    assert tr.source == "vehicle_global_position"
    assert tr.utc_synced is True
    assert tr.t_utc_s[0] == pytest.approx(BASE_UTC, abs=1e-3)
    e, _nn, u = FS.latlon_to_enu(*tr.latlon.T, LAT0, LON0, ALT0)
    assert np.max(np.abs(e - east)) < 0.05
    assert np.max(np.abs(u - up)) < 0.05


def test_ulog_raw_gps_fallback_scales_both_field_generations(monkeypatch):
    """The raw-GPS fallback stores lat/lon as int 1e-7 deg and alt as int mm
    (old fields) or plain degrees/metres (PX4 v1.17 SensorGps). Getting the alt
    scale wrong at a LOW-AMSL site reads 3 m as 3000 m and turns a genuine kill
    into a kilometres-wide MISS with a clean exit 0."""
    t_boot, utc, lat, lon, alt, _e, _n, up = _px4_datasets(low_amsl=3.0)
    old = _ulog_track({"vehicle_gps_position": {
        "timestamp": t_boot, "time_utc_usec": utc,
        "lat": np.round(lat * 1e7), "lon": np.round(lon * 1e7),
        "alt": np.round(alt * 1e3)}}, monkeypatch)
    assert old.source == "vehicle_gps_position"
    assert any("raw vehicle_gps_position" in w for w in old.warnings)
    _e1, _n1, u1 = FS.latlon_to_enu(*old.latlon.T, LAT0, LON0, ALT0)
    assert np.max(np.abs(u1 - up)) < 0.05

    new = _ulog_track({"vehicle_gps_position": {
        "timestamp": t_boot, "time_utc_usec": utc,
        "latitude_deg": lat, "longitude_deg": lon,
        "altitude_msl_m": alt}}, monkeypatch)
    _e2, _n2, u2 = FS.latlon_to_enu(*new.latlon.T, LAT0, LON0, ALT0)
    assert np.max(np.abs(u2 - up)) < 0.05


def test_ulog_without_a_utc_fix_falls_back_loudly(monkeypatch):
    t_boot, utc, lat, lon, alt, _e, _n, _u = _px4_datasets(with_utc=False)
    tr = _ulog_track({
        "vehicle_gps_position": {"timestamp": t_boot, "time_utc_usec": utc,
                                 "lat": lat * 1e7, "lon": lon * 1e7, "alt": alt * 1e3},
        "vehicle_global_position": {"timestamp": t_boot, "lat": lat, "lon": lon,
                                    "alt": alt}}, monkeypatch)
    assert tr.utc_synced is False
    assert any("boot-relative" in w for w in tr.warnings)


def test_ulog_with_neither_position_topic_raises(monkeypatch):
    with pytest.raises(ValueError, match="cannot recover"):
        _ulog_track({}, monkeypatch)


def test_ulog_track_scores_end_to_end_against_a_csv_track(monkeypatch):
    """The seam the repo never covered: a PX4 ULog interceptor vs the other
    side, all the way to a verdict."""
    n, hz = 60, 5.0
    t_boot_us = np.arange(n, dtype=float) * (1e6 / hz)
    utc_us = t_boot_us + BASE_UTC * 1e6
    east = -60.6 + 12.0 * (t_boot_us / 1e6)
    north = np.full(n, 0.30)
    up = np.full(n, 3.0)
    lat, lon, alt = FS.enu_to_latlon(east, north, up, LAT0, LON0, ALT0)
    tr_a = _ulog_track({"vehicle_global_position": {
        "timestamp": t_boot_us, "lat": lat, "lon": lon, "alt": alt},
        "vehicle_gps_position": {"timestamp": t_boot_us, "time_utc_usec": utc_us,
                                 "lat": lat * 1e7, "lon": lon * 1e7,
                                 "alt": alt * 1e3}}, monkeypatch)
    tr_b = _track("target", (0.0, 0.0, 3.0), (0.0, 0.0, 0.0), 11.8, 5.0,
                  t0_offset=0.15)
    # both tracks are ENU-projected from lat/lon against tr_a's own first sample
    res = FS.score_engagement(tr_a, tr_b, lethal_radius_m=FS.DEFAULT_LETHAL_RADIUS_M)
    assert res.cpa_m == pytest.approx(0.300, abs=0.02)
    assert res.verdict == "KILL"


# ----------------------------------------------------------------- the gate
def test_field_score_self_test_passes():
    """The tool's own offline gate, which now exercises the production dt path,
    the truncation leg and the ULog reader."""
    proc = subprocess.run([sys.executable,
                           os.path.join(REPO_ROOT, "scripts", "field_score.py"),
                           "--self-test"],
                          capture_output=True, text=True, cwd=REPO_ROOT)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "[self-test] PASS" in proc.stdout
    for leg in ("case2b_autodt_offgrid_KILL", "case2c_autodt_offgrid_MISS",
                "case2e_truncated_log", "case5a_ulog_globalpos_preferred"):
        assert leg in proc.stdout, f"{leg} did not run"
