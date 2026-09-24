"""flight/tests/test_terminal_cli.py -- directed tests for the `--terminal
{stock,tag,pursuit}` CLI wiring in flight.deploy.real_flight (ADR-0103 "chase
only", docs/pursuit_port_2026-09-17.md work item 3).

Parses the REAL parser (`build_arg_parser`) and constructs through the REAL
`build_config`/`build_terminal` -- no mission is flown except the one
subprocess dry-run at the bottom, which is the end-to-end claim.
"""
import os
import subprocess
import sys

import pytest

from flight.deploy.real_flight import (
    RealFlightSM,
    ScriptedTrigger,
    State,
    build_arg_parser,
    build_config,
    build_terminal,
    run_offline,
)
from flight.deploy.seeker_loop import GuidanceConfig, SeekerGuidance
from flight.camera import CameraModel
from flight.pursuit_terminal import PursuitTerminalGuidance
from flight.tag_terminal import TagInterceptGuidance

_REAL_FLIGHT = os.path.join(os.path.dirname(__file__), "..", "deploy",
                            "real_flight.py")


def _parse(*extra):
    return build_arg_parser().parse_args(["--dry-run", *extra])


def _build(*extra):
    args = _parse(*extra)
    cfg = build_config(args)
    gcfg = GuidanceConfig()
    cam = CameraModel(539.936, 539.936, 640.0, 480.0)
    return args, cfg, build_terminal(args, cfg, gcfg, cam)


def test_default_terminal_is_stock_seeker_guidance():
    _args, cfg, g = _build()
    assert isinstance(g, SeekerGuidance)
    assert cfg.pursuit_mode is False


def test_terminal_tag_builds_tag_intercept_guidance():
    _args, cfg, g = _build("--terminal", "tag")
    assert isinstance(g, TagInterceptGuidance)
    assert cfg.pursuit_mode is False


def test_terminal_pursuit_builds_pursuit_and_sets_pursuit_mode():
    _args, cfg, g = _build("--terminal", "pursuit")
    assert isinstance(g, PursuitTerminalGuidance)
    assert cfg.pursuit_mode is True
    # go_at_s=0.0: the SM gates ENGAGE entry; the class's own pre-GO hold
    # must never re-trigger once ENGAGE has begun.
    assert g.go_at_s == 0.0
    # yaw seed = the latched pre-flight heading.
    assert g.initial_yaw_deg == pytest.approx(cfg.preflight_heading_deg)


def test_pursuit_belief_seed_follows_target_start_and_vel():
    """--target-start is (east, north) about the launch point; the belief is
    NED [north, east, down] with down = +dash_loft_m."""
    _a, _c, g = _build("--terminal", "pursuit",
                       "--target-start", "3,20", "--target-vel", "9,-1")
    assert g._r_track[0] == pytest.approx(20.0)   # north
    assert g._r_track[1] == pytest.approx(3.0)    # east
    assert g._r_track[2] == pytest.approx(0.0)    # no loft -> level belief
    assert g._v_track[0] == pytest.approx(-1.0)   # v_north
    assert g._v_track[1] == pytest.approx(9.0)    # v_east

    _a, _c, g2 = _build("--terminal", "pursuit",
                        "--target-start", "3,20", "--dash-loft-m", "1.5")
    assert g2._r_track[2] == pytest.approx(1.5)   # believed target sits loft below


def test_pursuit_go_edge_enters_engage_directly():
    """pursuit_mode: the GO edge transitions STANDBY -> ENGAGE with no coded
    dash and no acquire streak -- Phase A needs no detection."""
    args = _parse("--terminal", "pursuit")
    cfg = build_config(args)
    gcfg = GuidanceConfig()
    cam = CameraModel(539.936, 539.936, 640.0, 480.0)
    guidance = build_terminal(args, cfg, gcfg, cam)
    sm, _rows = run_offline(cfg, ScriptedTrigger(go_at_s=3.0),
                            guidance=guidance, max_s=20.0, verbose=False)
    assert State.ENGAGE in sm.visited
    assert State.CODED_DASH not in sm.visited
    reasons = [tr.reason for tr in sm.transitions]
    assert any("go_edge_pursuit" in r for r in reasons), reasons


def test_stock_go_edge_still_dashes():
    """Regression guard: without --terminal pursuit the GO edge is
    byte-for-byte the historical CODED_DASH path."""
    args = _parse()
    cfg = build_config(args)
    sm, _rows = run_offline(cfg, ScriptedTrigger(go_at_s=3.0),
                            guidance=None, max_s=20.0, verbose=False)
    assert State.CODED_DASH in sm.visited


def test_dry_run_terminal_pursuit_exits_zero():
    """End-to-end through the actual CLI entrypoint, as a subprocess (a child
    interpreter, same as the field invocation)."""
    r = subprocess.run(
        [sys.executable, _REAL_FLIGHT, "--dry-run", "--terminal", "pursuit"],
        capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "go_edge_pursuit" in r.stdout
    assert "receding-range breakoff" in r.stdout.lower() or \
        "SUPPRESSED" in r.stdout


def test_pursuit_brake_flag_sets_the_adopted_package_and_defaults_off():
    """--pursuit-brake (builder ruling 2026-09-24, ADR-0115): sets exactly
    the registered package (a=3 m/s^2, lead 0.45 s default, horizontal-only
    cap default, vertical arrival-sync); absent -> every brake field at its
    legacy-off default."""
    _a, _c, g_off = _build("--terminal", "pursuit")
    assert g_off.cfg.brake_shaping is False
    assert g_off.cfg.brake_vert_sync is False

    _a, _c, g_on = _build("--terminal", "pursuit", "--pursuit-brake")
    assert g_on.cfg.brake_shaping is True
    assert g_on.cfg.brake_accel_ms2 == 3.0
    assert g_on.cfg.brake_lead_s == 0.45
    assert g_on.cfg.brake_horizontal_only is True
    assert g_on.cfg.brake_vert_sync is True


def test_pursuit_rehearsal_flag_sets_only_the_switch_and_defaults_off():
    """--pursuit-rehearsal (builder directive 2026-09-24,
    isim/specs/rehearsal_breakoff_prereg_2026-09-24.md): one switch -- sets
    rehearsal_breakoff=True and leaves the range/gate/evade at the config
    defaults; absent -> off. Composes with --pursuit-brake."""
    from flight.pursuit_terminal import PursuitTerminalConfig
    d = PursuitTerminalConfig()
    _a, _c, g_off = _build("--terminal", "pursuit")
    assert g_off.cfg.rehearsal_breakoff is False
    assert g_off.cfg == d

    _a, _c, g_on = _build("--terminal", "pursuit", "--pursuit-rehearsal")
    assert g_on.cfg.rehearsal_breakoff is True
    assert g_on.cfg.rehearsal_range_m == d.rehearsal_range_m
    assert g_on.cfg.rehearsal_min_updates == d.rehearsal_min_updates
    assert g_on.cfg.rehearsal_fresh_s == d.rehearsal_fresh_s
    assert g_on.cfg.rehearsal_evade_s == d.rehearsal_evade_s
    assert g_on.cfg.brake_shaping is False

    _a, _c, g_both = _build("--terminal", "pursuit", "--pursuit-brake",
                            "--pursuit-rehearsal")
    assert g_both.cfg.rehearsal_breakoff is True
    assert g_both.cfg.brake_shaping is True and g_both.cfg.brake_vert_sync is True
