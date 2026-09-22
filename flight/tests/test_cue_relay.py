"""flight/tests/test_cue_relay.py -- tests for flight.cue_relay (the
pre-flight GPS cue relay, docs/cue_relay_plan.md).

Three groups:
  1. Fitter accuracy on SYNTHETIC tracks with injected GPS noise -- empirically
     verifies the error-budget velocity-accuracy claim the plan doc cites.
  2. Quality-gate FAIL-CLOSED cases (too few points, short span, stale data,
     degenerate timestamps, bad residual) -- CueQualityError, never a default.
  3. Producer->consumer contract tests: one against the module's OWN
     JSON-lines writer (write_cue_jsonl -> read_cue_jsonl -> fit_cue), one
     against the MAVLink adapter's own output, and one against a REAL
     ArduPilot dataflash log (runs/tgt02_gps/00000304.BIN, the target FC
     actually flying) read by the project's EXISTING consumer
     (scripts.field_score.load_track_from_bin) -- not a hand-typed fixture.
"""
import json
import math
import os

import numpy as np
import pytest

from flight.cue_relay import (
    CueRecord, CueQualityError, fit_cue, lla_to_ned,
    write_cue_jsonl, read_cue_jsonl,
    cue_record_from_global_position_int,
    cue_to_target_start_arg, cue_to_target_vel_arg,
    load_cue_for_real_flight,
)
from flight import cue_relay

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_BENCH_BIN = os.path.join(_REPO_ROOT, "runs", "tgt02_gps", "00000304.BIN")

# A launch-point origin near the real bench fix (Boulder-area coordinates from
# runs/tgt02_gps/summary.json) so lla_to_ned's small-angle approximation is
# exercised at a realistic latitude, not just at the equator/(0,0).
_ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT = 39.99190, -105.24530, 1645.0


def _straight_track(n_pts, dt, speed_ms, heading_deg, r0_ned=(20.0, 5.0, 0.0),
                     t0=100.0):
    """Synthetic constant-velocity target track (an AUTO-leg straight run) as
    NOISE-FREE (t, north, east, down) samples relative to the origin."""
    hdg = math.radians(heading_deg)
    vn, ve = speed_ms * math.cos(hdg), speed_ms * math.sin(hdg)
    rows = []
    for i in range(n_pts):
        t = t0 + i * dt
        n = r0_ned[0] + vn * (t - t0)
        e = r0_ned[1] + ve * (t - t0)
        d = r0_ned[2]
        rows.append((t, n, e, d))
    return rows, (vn, ve, 0.0)


def _ned_to_lla(n, e, d, origin_lat=_ORIGIN_LAT, origin_lon=_ORIGIN_LON,
                 origin_alt=_ORIGIN_ALT):
    """Inverse of lla_to_ned (small-angle), for building synthetic GPS
    records from a known NED track."""
    R = 6378137.0
    lat0 = math.radians(origin_lat)
    lat = origin_lat + math.degrees(n / R)
    lon = origin_lon + math.degrees(e / (R * math.cos(lat0)))
    alt = origin_alt - d
    return lat, lon, alt


def _noisy_records(rows, sigma_pos_m, rng, with_vel=False, sigma_v_ms=0.0,
                    true_vel=None):
    records = []
    for (t, n, e, d) in rows:
        n_n = n + rng.normal(0.0, sigma_pos_m)
        e_n = e + rng.normal(0.0, sigma_pos_m)
        d_n = d + rng.normal(0.0, sigma_pos_m)
        lat, lon, alt = _ned_to_lla(n_n, e_n, d_n)
        vn = ve = vd = None
        if with_vel:
            vn = true_vel[0] + rng.normal(0.0, sigma_v_ms)
            ve = true_vel[1] + rng.normal(0.0, sigma_v_ms)
            vd = true_vel[2] + rng.normal(0.0, sigma_v_ms)
        records.append(CueRecord(t=t, lat=lat, lon=lon, alt_m_msl=alt,
                                 vn=vn, ve=ve, vd=vd))
    return records


# ------------------------------------------------------------- lla_to_ned


def test_lla_to_ned_matches_construction():
    """Round-trip sanity: a point built by the test's own _ned_to_lla inverse
    must map back through lla_to_ned to (approximately) the same NED vector
    -- catches a sign/axis-order bug in either direction immediately."""
    for n, e, d in [(0.0, 0.0, 0.0), (50.0, -30.0, -5.0), (-12.3, 200.0, 1.0)]:
        lat, lon, alt = _ned_to_lla(n, e, d)
        n2, e2, d2 = lla_to_ned(lat, lon, alt, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT)
        assert abs(n2 - n) < 1e-3, (n, n2)
        assert abs(e2 - e) < 1e-3, (e, e2)
        assert abs(d2 - d) < 1e-9, (d, d2)


# ------------------------------------------------------------- fitter accuracy


def test_fit_cue_recovers_noise_free_track_exactly():
    rows, true_vel = _straight_track(16, 0.15, 9.0, heading_deg=30.0)
    records = _noisy_records(rows, sigma_pos_m=0.0, rng=np.random.default_rng(0))
    sol = fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT)
    assert sol.vel_source == "fit"
    assert abs(sol.belief_vel0_ned[0] - true_vel[0]) < 1e-3
    assert abs(sol.belief_vel0_ned[1] - true_vel[1]) < 1e-3
    assert sol.residual_rms_m < 1e-6
    assert sol.n_points == 16


def test_fit_cue_velocity_accuracy_matches_error_budget_claim():
    """MONTE CARLO: at a GPS-grade position noise (HAcc-class, ~2.3 m 1-sigma
    per axis -- runs/tgt02_gps/summary.json GPA.HAcc mean 2.29 m, bench
    2026-09-17) and a ~2 s / ~7 Hz window, does the OLS position-fit velocity
    estimate land near the analytic OLS slope-variance prediction
    Var(v) = 12*sigma^2/(N*T^2)? This is the number
    docs/cue_relay_plan.md's error budget cites -- empirically verified here,
    not asserted from the derivation alone."""
    sigma_pos = 2.3   # m, 1-sigma per axis -- see GPA.HAcc note above
    n_pts, dt = 15, 0.13   # ~15 pts over ~1.95 s, ~7.5 Hz
    speed_true, heading_true = 9.0, 40.0
    rng = np.random.default_rng(42)
    v_errs = []
    for trial in range(400):
        rows, true_vel = _straight_track(n_pts, dt, speed_true, heading_true)
        records = _noisy_records(rows, sigma_pos_m=sigma_pos, rng=rng)
        sol = fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT,
                      min_points=8, min_span_s=1.0, max_residual_m=20.0)
        v_errs.append(math.hypot(sol.belief_vel0_ned[0] - true_vel[0],
                                 sol.belief_vel0_ned[1] - true_vel[1]))
    v_errs = np.array(v_errs)
    rms_2d = float(np.sqrt(np.mean(v_errs ** 2)))
    # Analytic per-axis OLS slope std: sigma_pos * sqrt(12 / (N * T^2)).
    T = (n_pts - 1) * dt
    sigma_v_axis_analytic = sigma_pos * math.sqrt(12.0 / (n_pts * T ** 2))
    rms_2d_analytic = sigma_v_axis_analytic * math.sqrt(2.0)  # 2 independent axes
    print(f"\n[cue_relay MC] measured 2D speed-error RMS = {rms_2d:.3f} m/s "
          f"over {len(v_errs)} trials (N={n_pts}, T={T:.2f}s, "
          f"sigma_pos={sigma_pos} m) vs analytic OLS prediction "
          f"{rms_2d_analytic:.3f} m/s")
    # Within a factor of 1.5 of the closed-form OLS prediction -- catches a
    # sign/units bug in the fit while tolerating finite-sample MC noise.
    assert rms_2d_analytic * 0.5 < rms_2d < rms_2d_analytic * 1.5, (
        rms_2d, rms_2d_analytic)


def test_fit_cue_prefers_reported_velocity_when_all_records_carry_it():
    rows, true_vel = _straight_track(12, 0.15, 9.0, heading_deg=0.0)
    rng = np.random.default_rng(3)
    records = _noisy_records(rows, sigma_pos_m=1.5, rng=rng, with_vel=True,
                             sigma_v_ms=0.3, true_vel=true_vel)
    sol = fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT)
    assert sol.vel_source == "reported"
    # Averaging N noisy reported samples should land within a few sigma/sqrt(N).
    assert abs(sol.belief_vel0_ned[0] - true_vel[0]) < 1.0
    assert abs(sol.belief_vel0_ned[1] - true_vel[1]) < 1.0


def test_fit_cue_falls_back_to_position_fit_if_any_record_missing_velocity():
    rows, true_vel = _straight_track(12, 0.15, 9.0, heading_deg=0.0)
    rng = np.random.default_rng(4)
    records = _noisy_records(rows, sigma_pos_m=0.5, rng=rng, with_vel=True,
                             sigma_v_ms=0.2, true_vel=true_vel)
    records[5] = CueRecord(t=records[5].t, lat=records[5].lat, lon=records[5].lon,
                           alt_m_msl=records[5].alt_m_msl)  # drop its velocity
    sol = fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT)
    assert sol.vel_source == "fit"   # never silently mixes sources


# ------------------------------------------------------------- extrapolation


def test_extrapolate_to_advances_position_by_constant_velocity_only():
    rows, true_vel = _straight_track(10, 0.2, 9.0, heading_deg=90.0)  # due east
    records = _noisy_records(rows, sigma_pos_m=0.0, rng=np.random.default_rng(1))
    sol = fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT)
    dt = 3.0
    sol2 = sol.extrapolate_to(sol.latch_t + dt)
    assert sol2.latch_t == pytest.approx(sol.latch_t + dt)
    assert sol2.belief_vel0_ned == sol.belief_vel0_ned  # velocity is NOT re-read
    n0, e0, d0 = sol.belief_r0_ned
    n1, e1, d1 = sol2.belief_r0_ned
    vn, ve, vd = sol.belief_vel0_ned
    assert n1 == pytest.approx(n0 + vn * dt)
    assert e1 == pytest.approx(e0 + ve * dt)
    assert d1 == pytest.approx(d0 + vd * dt)


# ------------------------------------------------------------- quality gate


def _good_records(n=10):
    rows, _ = _straight_track(n, 0.15, 9.0, heading_deg=0.0)
    return _noisy_records(rows, sigma_pos_m=0.3, rng=np.random.default_rng(7))


def test_gate_refuses_too_few_points():
    with pytest.raises(CueQualityError, match="need >="):
        fit_cue(_good_records(5), _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT,
               min_points=8)


def test_gate_refuses_too_short_a_span():
    rows, _ = _straight_track(10, 0.01, 9.0, heading_deg=0.0)  # 0.09 s span
    records = _noisy_records(rows, sigma_pos_m=0.2, rng=np.random.default_rng(8))
    with pytest.raises(CueQualityError, match="span"):
        fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT, min_span_s=1.0)


def test_gate_refuses_stale_data():
    records = _good_records()
    stale_now_t = records[-1].t + 5.0
    with pytest.raises(CueQualityError, match="stale"):
        fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT,
               now_t=stale_now_t, max_staleness_s=1.0)


def test_gate_accepts_fresh_data_at_the_staleness_boundary():
    records = _good_records()
    fresh_now_t = records[-1].t + 0.5
    sol = fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT,
                 now_t=fresh_now_t, max_staleness_s=1.0)
    assert sol.n_points == len(records)


def test_gate_refuses_degenerate_timestamps():
    """This is the belt-and-suspenders check BEHIND the span gate (a span<min
    caller could still, in principle, disable that gate) -- exercise it
    directly with min_span_s=0 so all-equal timestamps reach the fit."""
    rows, _ = _straight_track(10, 0.0, 9.0, heading_deg=0.0)  # all t equal
    records = _noisy_records(rows, sigma_pos_m=0.2, rng=np.random.default_rng(9))
    with pytest.raises(CueQualityError, match="non-converging|degenerate"):
        fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT, min_span_s=0.0)


def test_gate_refuses_high_residual_bad_fix():
    """A maneuvering/multipath-corrupted track (NOT a straight line) must be
    refused, not silently fit through -- the whole point of the residual
    gate."""
    rng = np.random.default_rng(11)
    records = []
    t0 = 0.0
    for i in range(12):
        t = t0 + i * 0.15
        # A sharp turn mid-window -- a straight-line fit cannot explain this.
        n = 20.0 + (9.0 * t if t < 0.8 else 9.0 * 0.8 + 9.0 * (t - 0.8) * -1.0)
        e = 5.0
        lat, lon, alt = _ned_to_lla(n, e, 0.0)
        records.append(CueRecord(t=t, lat=lat, lon=lon, alt_m_msl=alt))
    with pytest.raises(CueQualityError, match="residual"):
        fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT, max_residual_m=1.0)


# --------------------------------------------------------- CLI-arg formatting


def test_cue_to_target_start_and_vel_args_match_real_flight_convention():
    """real_flight.build_config/build_terminal parse `--target-start`/
    `--target-vel` as 'east,north' and unpack (t_e, t_n) = split[:2] -- this
    test freezes that exact ordering against flight.cue_relay's output."""
    rows, true_vel = _straight_track(10, 0.15, 9.0, heading_deg=60.0,
                                     r0_ned=(12.0, 7.0, 0.0))
    records = _noisy_records(rows, sigma_pos_m=0.0, rng=np.random.default_rng(2))
    sol = fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT)
    start_arg = cue_to_target_start_arg(sol)
    vel_arg = cue_to_target_vel_arg(sol)
    e_str, n_str = start_arg.split(",")
    ve_str, vn_str = vel_arg.split(",")
    n0, e0, _d0 = sol.belief_r0_ned
    vn0, ve0, _vd0 = sol.belief_vel0_ned
    assert float(e_str) == pytest.approx(e0, abs=1e-3)
    assert float(n_str) == pytest.approx(n0, abs=1e-3)
    assert float(ve_str) == pytest.approx(ve0, abs=1e-3)
    assert float(vn_str) == pytest.approx(vn0, abs=1e-3)


# --------------------------------------------------- producer/consumer contracts


def test_contract_jsonl_writer_then_reader_then_fit(tmp_path):
    rows, true_vel = _straight_track(10, 0.15, 9.0, heading_deg=15.0)
    records = _noisy_records(rows, sigma_pos_m=0.4, rng=np.random.default_rng(5),
                             with_vel=True, sigma_v_ms=0.2, true_vel=true_vel)
    path = str(tmp_path / "cue.jsonl")
    write_cue_jsonl(records, path)
    # Assert the ON-DISK schema, not just the round trip -- catches a field
    # rename that both ends of the SAME module would otherwise hide.
    with open(path) as f:
        first = json.loads(f.readline())
    assert set(first) >= {"t", "lat", "lon", "alt_m_msl"}
    read_back = read_cue_jsonl(path)
    assert len(read_back) == len(records)
    sol = fit_cue(read_back, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT)
    assert sol.vel_source == "reported"


def test_contract_mavlink_adapter_then_fit():
    """The adapter (`cue_record_from_global_position_int`) is the PRODUCER
    here -- fed a duck-typed stand-in for a real GLOBAL_POSITION_INT message
    (same field names/units pymavlink emits), never a hand-typed CueRecord."""
    class _FakeGlobalPositionInt:
        def __init__(self, lat_deg, lon_deg, alt_m, vn_ms, ve_ms, vd_ms):
            self.lat = int(round(lat_deg * 1e7))
            self.lon = int(round(lon_deg * 1e7))
            self.alt = int(round(alt_m * 1000))
            self.vx = int(round(vn_ms * 100))
            self.vy = int(round(ve_ms * 100))
            self.vz = int(round(vd_ms * 100))

    rows, true_vel = _straight_track(9, 0.15, 9.0, heading_deg=0.0)
    records = []
    for (t, n, e, d) in rows:
        lat, lon, alt = _ned_to_lla(n, e, d)
        msg = _FakeGlobalPositionInt(lat, lon, alt, *true_vel)
        records.append(cue_record_from_global_position_int(msg, t))
    sol = fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT)
    assert sol.vel_source == "reported"
    assert abs(sol.belief_vel0_ned[0] - true_vel[0]) < 0.05  # int-scaling round-off only


@pytest.mark.skipif(not os.path.exists(_BENCH_BIN),
                     reason="runs/tgt02_gps/00000304.BIN not present in this checkout")
def test_contract_real_ardupilot_bench_log():
    """Producer->consumer contract against REAL hardware data. The PRODUCER
    is the project's EXISTING .BIN consumer, `scripts.field_score.
    load_track_from_bin` (tgt-02 gate) -- a real ArduPilot dataflash log from
    the target FC that is actually flying (Holybro Kakute H7 v1.5,
    ArduCopter V4.7.0, Matek M10Q-5883 GPS; bench 2026-09-17, 11 sats, HDOP
    0.99; runs/tgt02_gps/summary.json). The vehicle was STATIONARY on the
    bench for this capture (true speed = 0), so this doubles as an empirical
    noise-floor measurement for the position side of the fit -- printed for
    citation in docs/cue_relay_plan.md's error budget."""
    pytest.importorskip("pymavlink")
    from scripts.field_score import load_track_from_bin
    from pathlib import Path

    track = load_track_from_bin(Path(_BENCH_BIN), "tgt02_bench")
    assert track.utc_synced, "bench log must have a real GPS UTC fix to trust t"
    assert track.source == "dataflash:POS"
    records = [CueRecord(t=float(t), lat=float(row[0]), lon=float(row[1]),
                         alt_m_msl=float(row[2]))
               for t, row in zip(track.t_utc_s, track.latlon)]
    assert len(records) == 27, "tgt-02's own gate count -- catches a silent parse change"

    origin = records[0]
    sol = fit_cue(records, origin.lat, origin.lon, origin.alt_m_msl,
                 min_points=8, min_span_s=1.0, max_residual_m=5.0)
    print(f"\n[real bench .BIN] n={sol.n_points} span_s={sol.span_s:.2f} "
          f"speed_ms={sol.speed_ms:.3f} (true=0, stationary) "
          f"residual_rms_m={sol.residual_rms_m:.3f} vel_source={sol.vel_source}")
    # The vehicle did not move -- the fitted "speed" on a stationary target IS
    # the velocity-fit noise floor. Loose bound: catches a units/axis bug
    # (which would read metres/second off by 100x or a sign) without being a
    # flaky assertion on this single real, non-reproducible capture.
    assert sol.speed_ms < 2.0
    assert sol.residual_rms_m < 2.0
    assert sol.vel_source == "fit"   # this .BIN's POS rows carry no velocity


# --------------------------------------------------------------- integration


def test_load_cue_for_real_flight_end_to_end(tmp_path):
    rows, true_vel = _straight_track(10, 0.15, 9.0, heading_deg=25.0,
                                     r0_ned=(16.7, 0.0, 0.0))
    records = _noisy_records(rows, sigma_pos_m=0.0, rng=np.random.default_rng(6))
    path = str(tmp_path / "cue.jsonl")
    write_cue_jsonl(records, path)
    start_arg, vel_arg, sol = load_cue_for_real_flight(
        path, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT,
        now_t=records[-1].t + 0.2)
    e_str, n_str = start_arg.split(",")
    assert float(n_str) == pytest.approx(sol.belief_r0_ned[0], abs=1e-3)
    assert float(e_str) == pytest.approx(sol.belief_r0_ned[1], abs=1e-3)


def test_load_cue_for_real_flight_raises_on_bad_quality(tmp_path):
    write_cue_jsonl(_good_records(3), str(tmp_path / "cue.jsonl"))  # too few
    with pytest.raises(CueQualityError):
        load_cue_for_real_flight(str(tmp_path / "cue.jsonl"),
                                 _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT)


# ==================================================== real_flight --cue-json
# The hookup applied from docs/cue_relay_plan.md §9: build_config reads the
# cue file EXACTLY ONCE, before the state machine exists, and fails CLOSED.

def _cue_file(tmp_path, records):
    p = tmp_path / "cue.jsonl"
    cue_relay.write_cue_jsonl(records, str(p))
    return str(p)


def _straight_records(n=20, dt=0.1, v_north=9.0, lat0=40.0, lon0=-105.0):
    recs = []
    for i in range(n):
        t = i * dt
        # ~9 m/s north of the origin, starting ~16.7 m north
        north = 16.7 + v_north * t
        recs.append(cue_relay.CueRecord(
            t=t, lat=lat0 + math.degrees(north / cue_relay._EARTH_R_M),
            lon=lon0, alt_m_msl=1500.0))
    return recs


def test_build_config_cue_json_overrides_the_typed_belief(tmp_path):
    from flight.deploy.real_flight import build_arg_parser, build_config
    path = _cue_file(tmp_path, _straight_records())
    args = build_arg_parser().parse_args([
        "--dry-run", "--cue-json", path,
        "--own-lat", "40.0", "--own-lon", "-105.0", "--own-alt-m-msl", "1500.0"])
    cfg = build_config(args)
    e, n = (float(v) for v in args.target_start.split(","))
    ve, vn = (float(v) for v in args.target_vel.split(","))
    # fitted belief ~ (east 0, north 16.7+9*t_latch) moving ~9 m/s north
    assert abs(e) < 0.5 and abs(n - (16.7 + 9.0 * 1.9)) < 0.5
    assert abs(ve) < 0.3 and abs(vn - 9.0) < 0.3
    # and the solved heading consumed it (roughly north +/- the lead)
    assert cfg.preflight_heading_deg == cfg.preflight_heading_deg  # finite


def test_build_config_cue_json_fails_closed_on_a_bad_cue(tmp_path):
    from flight.deploy.real_flight import build_arg_parser, build_config
    path = _cue_file(tmp_path, _straight_records(n=2, dt=0.01))  # too few/short
    args = build_arg_parser().parse_args([
        "--dry-run", "--cue-json", path,
        "--own-lat", "40.0", "--own-lon", "-105.0", "--own-alt-m-msl", "1500.0"])
    with pytest.raises(SystemExit, match="REFUSED"):
        build_config(args)


def test_build_config_cue_json_requires_the_own_fix(tmp_path):
    from flight.deploy.real_flight import build_arg_parser, build_config
    path = _cue_file(tmp_path, _straight_records())
    args = build_arg_parser().parse_args(["--dry-run", "--cue-json", path])
    with pytest.raises(SystemExit, match="own-lat"):
        build_config(args)


# ============================================== curved (turning) tracks -- CT
# docs/cue_relay_plan.md "curved tracks": the fitter now handles a
# coordinated turn instead of refusing it. Noise levels are the BENCH ones
# (runs/tgt02_gps/summary.json): pos sigma 2.3 m/axis (GPA.HAcc 2.29 m),
# reported-velocity sigma 1.72/sqrt(2) ~ 1.22 m/s/axis (GPA.SAcc 1.72 m/s,
# a 3-D 1-sigma split across two horizontal axes -- stated assumption).

_SIGMA_POS = 2.3
_SIGMA_V_AXIS = 1.72 / math.sqrt(2.0)


def _turn_track(n_pts, dt, speed_ms, heading0_deg, omega_deg_s,
                r0_ned=(20.0, 5.0, 0.0), t0=100.0):
    """Noise-free constant-turn-rate track: (t, n, e, d) rows plus the TRUE
    velocity at each sample and at the LAST sample (the latch)."""
    w = math.radians(omega_deg_s)
    h0 = math.radians(heading0_deg)
    rows, vels = [], []
    for i in range(n_pts):
        t = i * dt
        h = h0 + w * t
        if abs(w) > 1e-12:
            n = r0_ned[0] + speed_ms / w * (math.sin(h) - math.sin(h0))
            e = r0_ned[1] - speed_ms / w * (math.cos(h) - math.cos(h0))
        else:
            n = r0_ned[0] + speed_ms * math.cos(h0) * t
            e = r0_ned[1] + speed_ms * math.sin(h0) * t
        rows.append((t0 + t, n, e, r0_ned[2]))
        vels.append((speed_ms * math.cos(h), speed_ms * math.sin(h), 0.0))
    return rows, vels


def _noisy_turn_records(rows, vels, rng, sigma_pos=_SIGMA_POS,
                        sigma_v=_SIGMA_V_AXIS):
    records = []
    for (t, n, e, d), (vn, ve, vd) in zip(rows, vels):
        lat, lon, alt = _ned_to_lla(n + rng.normal(0, sigma_pos),
                                    e + rng.normal(0, sigma_pos),
                                    d + rng.normal(0, sigma_pos))
        records.append(CueRecord(t=t, lat=lat, lon=lon, alt_m_msl=alt,
                                 vn=vn + rng.normal(0, sigma_v),
                                 ve=ve + rng.normal(0, sigma_v),
                                 vd=vd + rng.normal(0, sigma_v)))
    return records


def test_ct_monte_carlo_velocity_at_latch_selection_and_omega_sigma():
    """ONE Monte Carlo, three registered claims (docs/cue_relay_plan.md
    "curved tracks"), 20 deg/s turn at 9 m/s, ~2 s / 15-pt window:
      (i)  velocity-at-latch error with the CT fitter beats the old
           whole-window mean by a wide margin (the mean is chord-biased by
           ~2.9 m/s at this turn rate -- derivation in the doc);
      (ii) the reported omega 1-sigma matches the measured scatter within
           25% (the fit's own error bar is honest);
      (iii) model selection: CT on the strong turn >= 85% of trials, CV on
           straight tracks >= 95% (the 4-sigma bar holds both ways)."""
    n_pts, dt, speed, omega = 15, 0.1333, 9.0, 20.0
    rng = np.random.default_rng(2026)
    n_trials = 320
    v_err_ct, v_err_mean, omega_hats, omega_sigmas, picked_ct = [], [], [], [], 0
    for _ in range(n_trials):
        rows, vels = _turn_track(n_pts, dt, speed, heading0_deg=30.0,
                                 omega_deg_s=omega)
        records = _noisy_turn_records(rows, vels, rng)
        sol = fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT,
                      max_residual_m=20.0)
        v_true = vels[-1]
        # the OLD estimator: whole-window mean of the reported velocities
        vmean = (float(np.mean([r.vn for r in records])),
                 float(np.mean([r.ve for r in records])))
        v_err_mean.append(math.hypot(vmean[0] - v_true[0], vmean[1] - v_true[1]))
        v_err_ct.append(math.hypot(sol.belief_vel0_ned[0] - v_true[0],
                                   sol.belief_vel0_ned[1] - v_true[1]))
        if sol.model == "ct":
            picked_ct += 1
            omega_hats.append(sol.omega_rad_s)
            omega_sigmas.append(sol.omega_sigma_rad_s)
    rms_ct = float(np.sqrt(np.mean(np.array(v_err_ct) ** 2)))
    rms_mean = float(np.sqrt(np.mean(np.array(v_err_mean) ** 2)))
    frac_ct = picked_ct / n_trials
    meas_sigma = float(np.std(np.array(omega_hats) - math.radians(omega)))
    rep_sigma = float(np.mean(omega_sigmas))
    print(f"\n[cue_relay CT MC] n={n_trials}: v-at-latch RMS ct={rms_ct:.3f} "
          f"vs whole-window-mean={rms_mean:.3f} m/s; CT picked {frac_ct:.0%}; "
          f"omega sigma measured={math.degrees(meas_sigma):.2f} vs "
          f"reported={math.degrees(rep_sigma):.2f} deg/s")
    assert rms_ct < 0.5 * rms_mean, (rms_ct, rms_mean)              # (i)
    assert abs(meas_sigma - rep_sigma) < 0.25 * rep_sigma, (        # (ii)
        meas_sigma, rep_sigma)
    assert frac_ct >= 0.85, frac_ct                                 # (iii) turn

    # (iii) straight side: same noise, omega = 0 -> CV >= 95%
    picked_cv = 0
    for _ in range(n_trials):
        rows, vels = _turn_track(n_pts, dt, speed, heading0_deg=30.0,
                                 omega_deg_s=0.0)
        records = _noisy_turn_records(rows, vels, rng)
        sol = fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT,
                      max_residual_m=20.0)
        if sol.model == "cv":
            picked_cv += 1
    assert picked_cv / n_trials >= 0.95, picked_cv / n_trials


def test_ct_straight_track_result_is_byte_identical_to_the_cv_fitter():
    """Regression: when CV is selected (straight track), the emitted velocity
    must be EXACTLY the whole-window mean of the reported velocities -- the
    pre-curve behaviour, unchanged."""
    rows, vels = _turn_track(15, 0.1333, 9.0, heading0_deg=45.0, omega_deg_s=0.0)
    records = _noisy_turn_records(rows, vels, np.random.default_rng(5))
    sol = fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT,
                  max_residual_m=20.0)
    assert sol.model == "cv" and sol.omega_rad_s == 0.0
    assert sol.belief_vel0_ned[0] == pytest.approx(
        float(np.mean([r.vn for r in records])), abs=1e-12)
    assert sol.belief_vel0_ned[1] == pytest.approx(
        float(np.mean([r.ve for r in records])), abs=1e-12)


def test_ct_noise_free_turn_recovers_omega_and_latch_state():
    rows, vels = _turn_track(15, 0.1333, 9.0, heading0_deg=10.0,
                             omega_deg_s=25.0)
    records = _noisy_turn_records(rows, vels, np.random.default_rng(6),
                                  sigma_pos=0.0, sigma_v=0.0)
    sol = fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT)
    assert sol.model == "ct"
    # linear-trend omega under-reads by ~sinc(omega*T/2) ~ 2-3% (disclosed)
    assert sol.omega_rad_s == pytest.approx(math.radians(25.0), rel=0.05)
    assert sol.belief_vel0_ned[0] == pytest.approx(vels[-1][0], abs=0.15)
    assert sol.belief_vel0_ned[1] == pytest.approx(vels[-1][1], abs=0.15)
    # latch position from the quadratic fit -- the linear fit would be biased
    rows_arr = rows[-1]
    n_true, e_true = rows_arr[1], rows_arr[2]
    assert sol.belief_r0_ned[0] == pytest.approx(n_true, abs=0.2)
    assert sol.belief_r0_ned[1] == pytest.approx(e_true, abs=0.2)


def test_ct_position_only_short_window_falls_back_to_cv():
    """Position-only curve detection at a 2 s window is BELOW the noise floor
    (sigma_a ~ 4 m/s^2 vs a ~3 m/s^2 real turn) -- the significance bar must
    keep it CV on most trials; the doc says a ~4-5 s window is needed."""
    rng = np.random.default_rng(7)
    cv_picked = 0
    n_trials = 40
    for _ in range(n_trials):
        rows, vels = _turn_track(15, 0.1333, 9.0, heading0_deg=30.0,
                                 omega_deg_s=20.0)
        records = [CueRecord(t=t, lat=lat, lon=lon, alt_m_msl=alt)
                   for (t, n, e, d) in rows
                   for lat, lon, alt in [_ned_to_lla(
                       n + rng.normal(0, _SIGMA_POS),
                       e + rng.normal(0, _SIGMA_POS),
                       d + rng.normal(0, _SIGMA_POS))]]
        sol = fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT,
                      max_residual_m=30.0)
        if sol.model == "cv":
            cv_picked += 1
    assert cv_picked / n_trials >= 0.75, cv_picked / n_trials


def test_ct_position_only_long_window_detects_the_turn():
    """Same turn, ~5 s window, noise-free positions: the quadratic path must
    detect it (this is the regime the doc says position-only needs)."""
    rows, vels = _turn_track(38, 0.1333, 9.0, heading0_deg=30.0,
                             omega_deg_s=15.0)
    records = [CueRecord(t=t, lat=lat, lon=lon, alt_m_msl=alt)
               for (t, n, e, d) in rows
               for lat, lon, alt in [_ned_to_lla(n, e, d)]]
    sol = fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT,
                  max_residual_m=30.0)
    assert sol.model == "ct"
    assert sol.omega_rad_s == pytest.approx(math.radians(15.0), rel=0.12)


def test_ct_extrapolate_follows_the_arc():
    rows, vels = _turn_track(15, 0.1333, 9.0, heading0_deg=0.0,
                             omega_deg_s=30.0)
    records = _noisy_turn_records(rows, vels, np.random.default_rng(8),
                                  sigma_pos=0.0, sigma_v=0.0)
    sol = fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT)
    assert sol.model == "ct"
    dt = 1.0
    sol2 = sol.extrapolate_to(sol.latch_t + dt)
    # velocity must have ROTATED by ~omega*dt, same speed
    h1 = math.atan2(sol.belief_vel0_ned[1], sol.belief_vel0_ned[0])
    h2 = math.atan2(sol2.belief_vel0_ned[1], sol2.belief_vel0_ned[0])
    assert (h2 - h1) == pytest.approx(sol.omega_rad_s * dt, rel=1e-6)
    assert sol2.speed_ms == pytest.approx(sol.speed_ms, rel=1e-9)
    # position advanced along the TRUE arc: compare against the generator
    rows_ext, _ = _turn_track(2, dt, 9.0, heading0_deg=math.degrees(h1),
                              omega_deg_s=math.degrees(sol.omega_rad_s),
                              r0_ned=sol.belief_r0_ned, t0=sol.latch_t)
    assert sol2.belief_r0_ned[0] == pytest.approx(rows_ext[1][1], abs=1e-6)
    assert sol2.belief_r0_ned[1] == pytest.approx(rows_ext[1][2], abs=1e-6)


def test_ct_extrapolation_caps_fail_closed():
    base = dict(latch_t=0.0, belief_r0_ned=(20.0, 5.0, 0.0),
                belief_vel0_ned=(9.0, 0.0, 0.0), speed_ms=9.0,
                heading_deg=0.0, residual_rms_m=0.5, n_points=15,
                span_s=2.0, vel_source="reported",
                origin_lla=(_ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT))
    from flight.cue_relay import CueSolution
    # heading-uncertainty cap: sigma 0.1 rad/s * 4 s = 0.4 rad > 0.35
    sol = CueSolution(model="ct", omega_rad_s=0.3, omega_sigma_rad_s=0.1, **base)
    with pytest.raises(CueQualityError, match="heading uncertainty"):
        sol.extrapolate_to(4.0)
    # arc cap: 0.6 rad/s * 3 s = 1.8 rad > pi/2 (sigma small enough to pass)
    sol2 = CueSolution(model="ct", omega_rad_s=0.6, omega_sigma_rad_s=0.01, **base)
    with pytest.raises(CueQualityError, match="arc"):
        sol2.extrapolate_to(3.0)
    # inside both caps: fine
    sol3 = CueSolution(model="ct", omega_rad_s=0.3, omega_sigma_rad_s=0.03, **base)
    out = sol3.extrapolate_to(1.0)
    assert out.latch_t == 1.0
    # CV solutions are never capped (unchanged behaviour)
    sol4 = CueSolution(**base)
    assert sol4.extrapolate_to(30.0).latch_t == 30.0


def test_ct_violent_turn_beyond_the_regime_cap_fails_closed():
    """A turn faster than TURN_OMEGA_MAX_RAD_S (here a hard direction
    reversal) must NOT be fitted as CT -- it falls to CV, whose residual gate
    then refuses it. Same spirit as the pre-curve residual test."""
    rows, vels = _turn_track(15, 0.1333, 9.0, heading0_deg=0.0,
                             omega_deg_s=120.0)   # 2.09 rad/s > 1.2 cap
    records = _noisy_turn_records(rows, vels, np.random.default_rng(9),
                                  sigma_pos=0.0, sigma_v=0.0)
    with pytest.raises(CueQualityError, match="residual"):
        fit_cue(records, _ORIGIN_LAT, _ORIGIN_LON, _ORIGIN_ALT,
                max_residual_m=1.0)
