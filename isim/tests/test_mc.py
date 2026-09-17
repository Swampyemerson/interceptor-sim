"""isim/mc.py tests (ADR-0102 isim). Determinism / worker-count independence
and the no-vacuous-verdicts table behaviour. Full engagements are slow (real
flight-code state machine + vehicle model), so these keep scenario counts
small; `sweep`'s own CLI is exercised for real in the subagent's manual
verification run, not repeated here."""
from __future__ import annotations

import csv
import dataclasses

import pytest

from isim.mc import (
    _ATTRIBUTION_CATEGORIES,
    _attribute_miss,
    _print_attribution,
    _print_table,
    _summarize,
    _write_csv,
    attribution_shares,
    build_parser,
    run_many,
    sweep,
)
from isim.scenario import Scatter, Scenario


def _fast_scenario(seed: int) -> Scenario:
    """A scenario tuned to reach a verdict quickly: short lead distance and a
    generous cross range keep n_steps/engagement small everywhere this file
    runs it."""
    return Scenario(target_speed_ms=9.0, cross_range_m=6.5, lead_dist_m=10.0, seed=seed)


def _fast_scattered_scenario(seed: int) -> Scenario:
    return Scenario(target_speed_ms=9.0, cross_range_m=6.5, lead_dist_m=10.0, seed=seed,
                    scatter=Scatter())


def _fast_pursuit_scenario(seed: int) -> Scenario:
    """A pursuit scenario cheap enough for these tests: same fast geometry,
    plus a short `pursuit_window_s` (the default 25 s would be needlessly
    slow for a contract test)."""
    return Scenario(concept="pursuit", target_speed_ms=9.0, cross_range_m=6.5,
                    lead_dist_m=10.0, cam_fx_px=933.0, tag_facing="camera",
                    cam_tilt_up_deg=12.0, pursuit_window_s=6.0, seed=seed)


def test_run_many_is_deterministic_per_seed():
    scns = [_fast_scenario(seed=3)]
    r1 = run_many(scns, workers=1)
    r2 = run_many(scns, workers=1)
    assert r1[0]["miss_m"] == r2[0]["miss_m"]
    assert r1[0]["transitions"] == r2[0]["transitions"]
    assert r1[0]["n_decoded"] == r2[0]["n_decoded"]


def test_run_many_result_independent_of_worker_count():
    scns = [_fast_scenario(seed=s) for s in range(3)]
    seq = run_many(scns, workers=1)
    par = run_many(scns, workers=2)
    seq_by_seed = {r["seed"]: r for r in seq}
    par_by_seed = {r["seed"]: r for r in par}
    assert set(seq_by_seed) == set(par_by_seed)
    for seed, r_seq in seq_by_seed.items():
        r_par = par_by_seed[seed]
        assert r_seq["miss_m"] == r_par["miss_m"]
        assert r_seq["transitions"] == r_par["transitions"]


def test_run_many_empty_list_returns_empty():
    assert run_many([], workers=1) == []


def test_result_dict_carries_scenario_fields_and_metrics():
    row = run_many([_fast_scenario(seed=0)], workers=1)[0]
    for f in ("target_speed_ms", "cross_range_m", "lead_dist_m", "direction", "seed"):
        assert f in row
    for f in ("miss_m", "miss_horiz_m", "miss_vert_m", "t_cpa", "closing_speed",
             "n_frames", "n_in_fov", "n_decoded", "frac_saturated", "final_state",
             "reached_engage", "transitions"):
        assert f in row


# ------------------------------------------------------------- no-vacuous-verdicts

def test_summarize_zero_rows_is_uncertain_not_a_pass():
    s = _summarize([])
    assert s["n"] == 0
    assert s["verdict"] == "UNCERTAIN"
    assert "med_miss_m" not in s   # no metric computed on zero units


def test_summarize_nonzero_rows_computes_stats():
    rows = run_many([_fast_scenario(seed=s) for s in range(3)], workers=1)
    s = _summarize(rows)
    assert s["n"] == 3
    assert s["verdict"] == "ok"
    assert s["med_miss_m"] >= 0.0
    assert 0.0 <= s["pct_hit"] <= 100.0
    assert 0.0 <= s["pct_engage"] <= 100.0


def test_write_csv_on_zero_rows_does_not_fabricate_a_header(tmp_path):
    path = str(tmp_path / "empty.csv")
    _write_csv(path, [])
    with open(path) as f:
        content = f.read()
    assert "UNCERTAIN" in content


def test_write_csv_round_trips_scenario_and_metric_fields(tmp_path):
    rows = run_many([_fast_scenario(seed=0)], workers=1)
    path = str(tmp_path / "one.csv")
    _write_csv(path, rows)
    with open(path, newline="") as f:
        read_rows = list(csv.DictReader(f))
    assert len(read_rows) == 1
    assert "miss_m" in read_rows[0]
    assert "target_speed_ms" in read_rows[0]


def test_print_table_runs_on_nonzero_and_zero_rows(capsys):
    """Regression: `_print_table`'s header formatting (a literal '%' next to
    an f-string field) was never exercised by `_summarize`/`_write_csv`
    directly and threw `TypeError` on every real invocation until caught by
    a manual `sweep` run -- this pins it so a return would fail loudly here,
    not only on a live CLI call."""
    rows = sweep("target_speed_ms", [9.0], n_seeds=1,
                base=_fast_scenario(seed=0), workers=1)
    _print_table("target_speed_ms", rows, [9.0])
    _print_table("target_speed_ms", [], [123.0])   # zero-row value -> UNCERTAIN row
    out = capsys.readouterr().out
    assert "UNCERTAIN" in out


def test_sweep_tags_each_row_with_its_axis_value():
    rows = sweep("target_speed_ms", [2.0, 9.0], n_seeds=1,
                base=_fast_scenario(seed=0), workers=1)
    assert len(rows) == 2
    values_seen = {r["_axis_value"] for r in rows}
    assert values_seen == {2.0, 9.0}
    for r in rows:
        assert r["target_speed_ms"] == r["_axis_value"]


# --------------------------------------------------------------------- scatter

def test_run_many_result_independent_of_worker_count_with_scatter_on():
    scns = [_fast_scattered_scenario(seed=s) for s in range(3)]
    seq = run_many(scns, workers=1)
    par = run_many(scns, workers=2)
    seq_by_seed = {r["seed"]: r for r in seq}
    par_by_seed = {r["seed"]: r for r in par}
    assert set(seq_by_seed) == set(par_by_seed)
    for seed, r_seq in seq_by_seed.items():
        r_par = par_by_seed[seed]
        assert r_seq["miss_m"] == r_par["miss_m"]
        assert r_seq["transitions"] == r_par["transitions"]


def test_run_many_deterministic_per_seed_with_scatter_on():
    scns = [_fast_scattered_scenario(seed=4)]
    r1 = run_many(scns, workers=1)
    r2 = run_many(scns, workers=1)
    assert r1[0]["miss_m"] == r2[0]["miss_m"]
    assert r1[0]["transitions"] == r2[0]["transitions"]


def test_result_row_carries_n_fault_lines():
    row = run_many([_fast_scenario(seed=0)], workers=1)[0]
    assert "n_fault_lines" in row
    assert row["n_fault_lines"] >= 0


def test_summarize_has_p10_and_pct_hit_loose():
    rows = run_many([_fast_scenario(seed=s) for s in range(3)], workers=1)
    s = _summarize(rows)
    assert "p10_miss_m" in s
    assert "p90_miss_m" in s
    assert s["p10_miss_m"] <= s["p90_miss_m"]
    assert "pct_hit_loose" in s
    assert s["pct_hit"] <= s["pct_hit_loose"] + 1e-9   # 0.35 m hit implies 1.0 m hit


def test_cli_accepts_scatter_terminal_camtilt_flags():
    p = build_parser()
    args = p.parse_args(["sweep", "--axis", "target_speed_ms", "--values", "9.0",
                         "--n", "1", "--scatter", "--terminal", "stock",
                         "--cam-tilt", "3.0"])
    assert args.scatter is True
    assert args.terminal == "stock"
    assert args.cam_tilt == 3.0

    args2 = p.parse_args(["requirement", "--out", "/tmp/does_not_matter.csv"])
    assert args2.scatter is False
    assert args2.terminal == "stock"
    assert args2.cam_tilt == 0.0


def test_sweep_cli_scatter_flag_builds_scattered_scenarios(tmp_path):
    rows = sweep("target_speed_ms", [9.0], n_seeds=2,
                base=Scenario(cross_range_m=6.5, lead_dist_m=10.0, scatter=Scatter()),
                workers=1)
    assert len(rows) == 2
    for r in rows:
        assert "n_fault_lines" in r


def test_cli_accepts_concept_and_tag_facing_flags():
    p = build_parser()
    args = p.parse_args(["sweep", "--axis", "target_speed_ms", "--values", "9.0",
                         "--n", "1", "--concept", "pursuit", "--tag-facing", "rear"])
    assert args.concept == "pursuit"
    assert args.tag_facing == "rear"


# ------------------------------------------------------- v2: miss attribution

def test_flyby_rows_carry_no_attribution():
    """`concept="flyby"` rows must carry the diagnostic columns (so a mixed
    CSV has a uniform header) but every value is None -- the diagnostics are
    pursuit-only (isim/specs/pursuit_concept_v2.md #5)."""
    row = run_many([_fast_scenario(seed=0)], workers=1)[0]
    assert row["attribution"] is None
    assert row["estimator_pos_err_m"] is None
    assert row["phase_at_cpa"] is None


def test_pursuit_rows_carry_a_valid_attribution():
    row = run_many([_fast_pursuit_scenario(seed=0)], workers=1)[0]
    assert row["attribution"] in _ATTRIBUTION_CATEGORIES
    assert row["estimator_pos_err_m"] is not None
    assert row["control_err_m"] is not None
    # The "_m1s" (one second before CPA) twin columns exist alongside the
    # at-CPA ones.
    assert "estimator_pos_err_m_m1s" in row
    assert "phase_at_cpa_m1s" in row


def test_attribution_shares_sums_to_100_and_is_uncertain_on_no_rows():
    rows = run_many([_fast_pursuit_scenario(seed=s) for s in range(4)], workers=1)
    shares = attribution_shares(rows)
    assert set(shares) == set(_ATTRIBUTION_CATEGORIES)
    assert sum(shares.values()) == pytest.approx(100.0, abs=1e-6)
    assert attribution_shares([]) == {}   # no-vacuous-verdicts: empty, not a fabricated 0%


def test_attribute_miss_never_engaged_beats_everything_else():
    diag = {"phase": "A", "decodes_last_1s": 0, "accel_exceed_frac_last_1s": 1.0,
           "estimator_pos_err_m": 50.0, "control_err_m": 50.0}
    assert _attribute_miss(diag, miss_m=50.0) == "never_engaged"


def test_attribute_miss_no_tag_only_when_zero_decodes():
    diag = {"phase": "B", "decodes_last_1s": 0, "accel_exceed_frac_last_1s": 0.0,
           "estimator_pos_err_m": 0.01, "control_err_m": 0.01}
    assert _attribute_miss(diag, miss_m=5.0) == "no_tag_last_second"


def test_attribute_miss_picks_the_larger_of_estimate_and_control():
    base = {"phase": "B", "decodes_last_1s": 3, "accel_exceed_frac_last_1s": 0.0}
    assert _attribute_miss({**base, "estimator_pos_err_m": 5.0, "control_err_m": 1.0},
                           miss_m=1.0) == "estimate"
    assert _attribute_miss({**base, "estimator_pos_err_m": 1.0, "control_err_m": 5.0},
                           miss_m=1.0) == "control"


def test_attribute_miss_saturated_needs_the_real_test_and_the_larger_magnitude():
    base = {"phase": "B", "decodes_last_1s": 3,
           "estimator_pos_err_m": 0.1, "control_err_m": 0.1}
    # Real test fires (>=0.5) AND its score (miss_m) beats estimate/control.
    assert _attribute_miss({**base, "accel_exceed_frac_last_1s": 0.8}, miss_m=3.0) == "saturated"
    # Real test does NOT fire (<0.5): saturated scores 0, never wins.
    assert _attribute_miss({**base, "accel_exceed_frac_last_1s": 0.2}, miss_m=3.0) != "saturated"


def test_print_attribution_is_a_noop_for_flyby_rows(capsys):
    rows = run_many([_fast_scenario(seed=0)], workers=1)
    for r in rows:
        r["_axis_value"] = r["target_speed_ms"]
    _print_attribution("target_speed_ms", rows, [9.0])
    assert capsys.readouterr().out == ""


def test_print_attribution_prints_for_pursuit_rows(capsys):
    rows = run_many([_fast_pursuit_scenario(seed=s) for s in range(3)], workers=1)
    for r in rows:
        r["_axis_value"] = r["target_speed_ms"]
    _print_attribution("target_speed_ms", rows, [9.0])
    out = capsys.readouterr().out
    assert "attribution shares" in out
    for cat in _ATTRIBUTION_CATEGORIES:
        assert cat in out


# --------------------------------------------------- v4: "at last decode"

def test_pursuit_rows_carry_estimator_error_at_last_decode():
    """v4 #4: a second query point (the last decode before CPA, not CPA
    itself) resolving the v3 oddity -- see the task's final report."""
    rows = run_many([_fast_pursuit_scenario(seed=s) for s in range(5)], workers=1)
    assert any(r["estimator_pos_err_m_at_last_decode"] is not None for r in rows)
    for r in rows:
        assert "estimator_pos_err_m_at_last_decode" in r
        assert "estimator_vel_err_m_at_last_decode" in r


def test_flyby_rows_carry_none_for_at_last_decode_too():
    row = run_many([_fast_scenario(seed=0)], workers=1)[0]
    assert row["estimator_pos_err_m_at_last_decode"] is None


# --------------------------------------------------- v4: pursuit_overrides

def test_pursuit_overrides_reach_a_spawned_worker():
    """The gotcha from the last two rounds: a monkeypatched default does NOT
    reach a `spawn` worker process, but a plain `Scenario` field value does
    -- this is exactly that field, exercised with workers=2 to prove it."""
    base_scn = _fast_pursuit_scenario(seed=0)
    scn_tight = dataclasses.replace(base_scn, pursuit_overrides={"kf_q_accel_ms2": 50.0})
    rows_default = run_many([base_scn], workers=2)
    rows_tight = run_many([scn_tight], workers=2)
    # Wildly different process noise must change SOMETHING measurable
    # (estimator velocity error at CPA-1s, a real number either way).
    v_default = rows_default[0]["estimator_vel_err_m_m1s"]
    v_tight = rows_tight[0]["estimator_vel_err_m_m1s"]
    assert v_default is not None and v_tight is not None
    assert v_default != v_tight
