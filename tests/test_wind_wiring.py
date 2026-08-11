"""THE WIND ARM IS ACTUALLY WIRED -- proved by making the wiring fail.

A flag declared but never read is this project's single most-repeated defect
(CLAUDE.md: "a fix is not done until its EFFECT is observed end-to-end"; the
--dash-accel-aware-lead INERT note, the --law pip / --terminal-bearing-bias
refusal, and ADR-0090's frame-rate line all exist because of it). So this file
does not assert that the flags PARSE. It:

  1. drives the REAL scripts/m4_intercept.py pre-flight block -- the actual
     `run_acquire_and_engage` coroutine, stepped once, which is far enough to
     execute the real dash-heading resolution -- with the trim off and on, and
     asserts the aim the vehicle would fly CHANGES; then
  2. MUTATES a copy of m4_intercept.py to remove the wiring, re-runs the same
     probe, and asserts the two arms now agree. If step 2 ever stops failing,
     step 1 was decoration.

It also pins the default-OFF contract in both directions: no --wind-* flag ->
no wind driver process, no wind trim object, and a lead solve that is a single
untouched call; and a half-configured wind run (a --wind-seed with no source,
a trim with no measured sag table) REFUSES TO RUN rather than flying inert.

Offline only. Nothing here boots Gazebo. What it CANNOT prove -- and what the
Gate-0 hover probe exists for -- is that Gazebo applies the wrench the driver
publishes.
"""

from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import os
import subprocess
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO, "scripts")
for _p in (SCRIPTS, REPO):
    if _p not in sys.path:
        sys.path.insert(0, _p)

M4_PATH = os.path.join(SCRIPTS, "m4_intercept.py")
MC_BATCH = os.path.join(SCRIPTS, "mc_batch.sh")

m4 = pytest.importorskip("m4_intercept", reason="needs the gz/mavsdk sim venv")
wind_driver = pytest.importorskip("wind_driver")
import wind_model  # noqa: E402
import wind_trim  # noqa: E402


# A table shaped like the one Gate 2 will produce, LOUDLY not measured. Tests
# may use it because they only assert that a correction MOVES; no number here
# is ever quoted as a result.
SYNTHETIC_TABLE = {
    "points": [[-4.0, -0.9], [0.0, 0.0], [4.0, 0.9], [8.0, 1.8]],
    "provenance": ("SYNTHETIC fixture for tests/test_wind_wiring.py -- NOT "
                   "MEASURED. No Gate-2 wind arm has flown."),
}

# A realistic coded-dash geometry: 16 m/s dash, 9 m/s crossing target, the
# speeds the real-build arms use. Deliberately NOT a degenerate geometry where
# a small speed change makes the target uncatchable.
BASE_ARGV = [
    "m4_intercept.py", "--law", "pronav", "--fpv", "--seeker", "markerless",
    "--coded-dash", "--dash-accel-aware-lead", "--dash-speed", "16",
    "--target-start", "20,14,0.5", "--target-vel", "0,9.0",
]


# --------------------------------------------------------------- the probe
def _parse(module, extra):
    old = sys.argv
    sys.argv = list(BASE_ARGV) + list(extra)
    try:
        return module.parse_args()
    finally:
        sys.argv = old


def preflight_stdout(module, extra):
    """Run the REAL pre-flight block and return everything it printed.

    `run_acquire_and_engage` is a coroutine; `send(None)` executes it from the
    top until the first suspension. There is NO await between its first line
    and the dash-heading resolution (asserted by
    test_no_await_precedes_the_wind_wiring, so this probe cannot silently stop
    covering the wiring), so one step reaches the aim solve. It then dies on the
    first real sim access, which is expected and swallowed.
    """
    args = _parse(module, extra)
    buf = io.StringIO()
    coro = module.run_acquire_and_engage(
        None, None, None, None, None, None, 0.0, args)
    with contextlib.redirect_stdout(buf):
        try:
            coro.send(None)
        except BaseException as exc:                  # noqa: BLE001
            print(f"__STOPPED__ {type(exc).__name__}")
        finally:
            coro.close()
    return buf.getvalue()


def heading_from(out):
    for line in out.splitlines():
        if "Accel-aware collision lead ON" in line and "-> heading" in line:
            return float(line.split("-> heading")[1].split("deg")[0].strip())
    raise AssertionError(f"no aim line in probe output:\n{out}")


@pytest.fixture()
def sag_table(tmp_path):
    p = tmp_path / "sag_synthetic.json"
    p.write_text(json.dumps(SYNTHETIC_TABLE))
    return str(p)


# ======================================================================== 1
# THE DEFAULT IS A TRUE NO-OP
# ======================================================================== 1
def test_default_run_builds_no_wind_of_any_kind():
    args = _parse(m4, [])
    assert args.wind_enabled is False
    assert m4.wind_driver_command(args, "/tmp/never.csv") is None
    assert m4.build_wind_trim(args).enabled is False
    assert m4.spawn_wind_driver(args, "/tmp/never.csv") == (None, None)


def test_default_preflight_says_trim_off_and_solves_once():
    out = preflight_stdout(m4, [])
    assert "[wind-trim] OFF" in out
    assert "byte-identical to a no-wind run" in out
    assert "WIND ARM ON" not in out


def test_explicit_zero_trim_is_identical_to_the_default():
    a = heading_from(preflight_stdout(m4, []))
    b = heading_from(preflight_stdout(m4, ["--wind-trim-mps", "0.0"]))
    assert a == b, "0.0 is the disabled sentinel and must be an exact identity"


def test_disabled_trim_calls_the_lead_solver_exactly_once():
    """The fixed point must not spin when there is nothing to correct."""
    calls = []

    def solver(pos, vel, spd):
        calls.append(spd)
        return 12.5, 1.0

    h, t, res = wind_trim.solve_wind_trimmed_lead(
        solver, (20.0, 14.0), (0.0, 9.0), 16.0, wind_trim.WindTrim.disabled())
    assert calls == [16.0] and h == 12.5 and res.enabled is False


# ======================================================================== 2
# THE WIRING IS NOT INERT -- the aim the vehicle would fly actually changes
# ======================================================================== 2
def test_trim_on_changes_the_real_dash_heading(sag_table):
    off = preflight_stdout(m4, [])
    on = preflight_stdout(m4, ["--wind-trim-mps", "6.0",
                               "--wind-trim-dir-deg", "20",
                               "--wind-sag-table", sag_table])
    h_off, h_on = heading_from(off), heading_from(on)
    assert "[wind-trim] PRE-FLIGHT anemometer trim ON" in on
    assert abs(h_on - h_off) > 0.1, (
        f"the trim did not move the aim ({h_off} -> {h_on}); it is INERT")


def test_trim_direction_matters_not_just_its_magnitude(sag_table):
    """A crosswind and a headwind of the same SPEED must not give the same aim,
    or the trim would be reading only the number the operator typed."""
    head = heading_from(preflight_stdout(m4, [
        "--wind-trim-mps", "6.0", "--wind-trim-dir-deg", "20",
        "--wind-sag-table", sag_table]))
    cross = heading_from(preflight_stdout(m4, [
        "--wind-trim-mps", "6.0", "--wind-trim-dir-deg", "110",
        "--wind-sag-table", sag_table]))
    assert abs(head - cross) > 0.1


def test_the_applied_log_line_states_what_was_applied(sag_table):
    """ADR-0090: log what was APPLIED, not what was intended. The line must
    carry the resolved components and the before->after speed, not just echo
    the flag."""
    out = preflight_stdout(m4, ["--wind-trim-mps", "6.0",
                                "--wind-trim-dir-deg", "20",
                                "--wind-sag-table", sag_table])
    line = [ln for ln in out.splitlines() if "[wind-trim]" in ln][0]
    for token in ("operator read", "headwind", "crosswind", "measured sag",
                  "lead-solve dash speed", "no in-flight wind sensing",
                  "table:"):
        assert token in line, f"applied line is missing {token!r}: {line}"
    # the before->after speeds must both appear and differ
    seg = line.split("lead-solve dash speed")[1]
    before, after = seg.split("m/s")[0].split("->")
    assert float(before) != float(after)


# ======================================================================== 3
# THE MUTANT: delete the wiring and the tests above must go red
# ======================================================================== 3
WIRING_LINES = {
    "trim_built": "_wind_trim = build_wind_trim(args)",
    "trim_accel_solve": "lambda pos, vel, spd: collision_lead_heading_accel(",
    "trim_const_solve": "coded_dash_heading_deg, _, _wind_trim_result = solve_wind_trimmed_lead(",
    "trim_logged": "print(f\"[m4] {_wind_trim_result.log_line}\", flush=True)",
    "driver_spawned": "wind_proc, wind_log_path = spawn_wind_driver(",
    "applied_summary": "summarize_applied_wind(wind_log_path, wind_proc.returncode)",
}


def test_every_wiring_line_is_actually_present_in_m4():
    src = open(M4_PATH, encoding="utf-8").read()
    missing = [k for k, v in WIRING_LINES.items() if v not in src]
    assert not missing, f"wiring lines vanished from m4_intercept.py: {missing}"


def _load_mutant(tmp_path, old, new, name):
    src = open(M4_PATH, encoding="utf-8").read()
    assert old in src, f"mutation anchor not found: {old!r}"
    mutant_dir = tmp_path / name
    mutant_dir.mkdir()
    path = mutant_dir / "m4_mutant.py"
    path.write_text(src.replace(old, new, 1))
    spec = importlib.util.spec_from_file_location(f"m4_mutant_{name}", str(path))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_mutant_removing_the_accel_branch_wiring_makes_the_aim_stop_moving(
        tmp_path, sag_table):
    """THE LOAD-BEARING TEST. Put the pre-wind call back, and the trim-on aim
    collapses onto the trim-off aim -- i.e. the assertion in
    test_trim_on_changes_the_real_dash_heading is caused by the wiring and
    nothing else."""
    mutant = _load_mutant(
        tmp_path,
        """            coded_dash_heading_deg, _t_lead, _wind_trim_result = solve_wind_trimmed_lead(
                lambda pos, vel, spd: collision_lead_heading_accel(
                    pos, vel, spd, _lead_accel),
                _lead_pos, (_tvx, _tvy), _vi, _wind_trim)""",
        """            coded_dash_heading_deg, _t_lead = collision_lead_heading_accel(
                _lead_pos, (_tvx, _tvy), _vi, _lead_accel)
            _wind_trim_result = wind_trim_module_apply(_vi, coded_dash_heading_deg, _wind_trim)""",
        "unwired")
    # keep the log line working so the mutant still runs to completion
    mutant.wind_trim_module_apply = wind_trim.apply_wind_trim
    off = heading_from(preflight_stdout(mutant, []))
    on = heading_from(preflight_stdout(mutant, [
        "--wind-trim-mps", "6.0", "--wind-trim-dir-deg", "20",
        "--wind-sag-table", sag_table]))
    assert off == on, (
        "the UNWIRED mutant still moved the aim -- something other than the "
        "wiring under test is changing it, so the real test proves nothing")


def test_mutant_dropping_the_trim_object_makes_the_aim_stop_moving(
        tmp_path, sag_table):
    """The second half of the wiring: if build_wind_trim's result is replaced
    by a permanently-disabled trim, the flag becomes a classic inert knob."""
    mutant = _load_mutant(
        tmp_path,
        "    _wind_trim = build_wind_trim(args)",
        "    _wind_trim = WindTrim.disabled()  # MUTANT: flag read, never used",
        "disabled")
    off = heading_from(preflight_stdout(mutant, []))
    on = heading_from(preflight_stdout(mutant, [
        "--wind-trim-mps", "6.0", "--wind-trim-dir-deg", "20",
        "--wind-sag-table", sag_table]))
    assert off == on


def test_no_await_precedes_the_wind_wiring():
    """The probe above only reaches the wiring because nothing suspends first.
    If an `await` is ever inserted before it, this test fails LOUDLY instead of
    the probe silently stopping short and passing on an empty comparison (no
    vacuous verdicts)."""
    src = open(M4_PATH, encoding="utf-8").read().splitlines()
    start = next(i for i, l in enumerate(src)
                 if l.startswith("async def run_acquire_and_engage"))
    end = next(i for i, l in enumerate(src)
               if WIRING_LINES["trim_built"] in l)
    offending = [(i + 1, l.strip()) for i, l in enumerate(src[start:end], start)
                 if "await " in l]
    assert not offending, (
        "an await was added before the wind wiring; tests/test_wind_wiring.py's "
        f"one-step probe no longer reaches it: {offending}")


# ======================================================================== 4
# FAIL CLOSED: a half-configured wind run refuses, it does not fly inert
# ======================================================================== 4
@pytest.mark.parametrize("extra,needle", [
    (["--wind-seed", "7"], "NO wind source"),
    (["--wind-height-m", "6.0"], "NO wind source"),
    (["--wind-dir-from-deg", "270"], "NO wind source"),
    (["--wind-allow-below-10ft"], "silently inert"),
    (["--wind-tier", "EXPECTED", "--wind-dir-from-deg", "270",
      "--wind-height-m", "6.0"], "--wind-seed"),
    (["--wind-tier", "EXPECTED", "--wind-dir-from-deg", "270",
      "--wind-seed", "7"], "--wind-height-m"),
    (["--wind-tier", "EXPECTED", "--wind-seed", "7", "--wind-height-m", "6.0"],
     "--wind-dir-from-deg"),
    (["--wind-tier", "EXPECTED", "--wind-still-air", "--wind-seed", "7",
      "--wind-height-m", "6.0", "--wind-dir-from-deg", "0"], "mutually exclusive"),
    (["--wind-trim-mps", "6.0"], "--wind-sag-table"),
])
def test_half_configured_wind_refuses_to_run(extra, needle, capsys):
    with pytest.raises(SystemExit):
        _parse(m4, extra)
    assert needle in capsys.readouterr().err


def test_sag_table_without_trim_refuses(sag_table, capsys):
    with pytest.raises(SystemExit):
        _parse(m4, ["--wind-sag-table", sag_table])
    assert "silently unused" in capsys.readouterr().err


def test_trim_with_an_explicit_heading_refuses(sag_table, capsys):
    with pytest.raises(SystemExit):
        _parse(m4, ["--wind-trim-mps", "6.0", "--wind-sag-table", sag_table,
                    "--dash-heading-deg", "10"])
    assert "SILENTLY INERT" in capsys.readouterr().err


def test_trim_without_a_measured_table_raises_even_if_argparse_is_bypassed():
    """Belt and braces: a caller that builds `args` by hand still cannot enable
    a trim with no measured sag."""
    class A:
        wind_trim_mps = 6.0
        wind_trim_dir_deg = 20.0
        wind_sag_table = None
    with pytest.raises(wind_trim.WindTrimError):
        m4.build_wind_trim(A())


# ======================================================================== 5
# THE DRIVER SPAWN
# ======================================================================== 5
def test_driver_command_off_and_on(tmp_path):
    assert m4.wind_driver_command(_parse(m4, []), "x.csv") is None
    args = _parse(m4, ["--wind-tier", "WORST", "--wind-dir-from-deg", "270",
                       "--wind-seed", "7", "--wind-height-m", "6.0"])
    cmd = m4.wind_driver_command(args, str(tmp_path / "w.csv"))
    assert cmd is not None
    for token in ("--tier", "WORST", "--dir-from-deg", "270.0",
                  "--seed", "7", "--height-m", "6.0", "--out"):
        assert token in cmd, f"{token!r} missing from {cmd}"
    assert "--still-air" not in cmd


def test_driver_command_still_air_is_the_control_arm(tmp_path):
    args = _parse(m4, ["--wind-still-air", "--wind-seed", "7",
                       "--wind-height-m", "6.0"])
    cmd = m4.wind_driver_command(args, str(tmp_path / "w.csv"))
    assert "--still-air" in cmd and "--tier" not in cmd and "--mean-mps" not in cmd


def test_the_spawned_command_is_one_the_driver_actually_accepts(tmp_path):
    """PRODUCER -> CONSUMER contract, fixture built by the producer's own
    writer: m4 emits the argv, wind_driver.py parses it. A hand-typed fixture is
    exactly what has hidden schema breaks in this repo before."""
    args = _parse(m4, ["--wind-tier", "EXPECTED", "--wind-dir-from-deg", "270",
                       "--wind-seed", "7", "--wind-height-m", "6.0"])
    cmd = m4.wind_driver_command(args, str(tmp_path / "w.csv"))
    parsed = wind_driver.parse_args(cmd[2:])          # drop python + script
    cond = wind_driver.build_conditions(parsed)
    assert cond.label == "EXPECTED"
    assert cond.mean_mps == wind_model.TIERS["EXPECTED"][0]
    assert cond.dir_from_deg == 270.0
    assert parsed.seed == 7


def test_spawn_fails_loud_when_the_driver_dies_immediately(tmp_path):
    stub = tmp_path / "dead_driver.py"
    stub.write_text("import sys; sys.exit(2)\n")
    args = _parse(m4, ["--wind-still-air", "--wind-seed", "7",
                       "--wind-height-m", "6.0"])
    with pytest.raises(RuntimeError, match="exited immediately"):
        m4.spawn_wind_driver(args, str(tmp_path / "w.csv"),
                             script=str(stub), settle_s=0.4)


def test_spawn_returns_a_live_process_and_the_applied_log_path(tmp_path):
    stub = tmp_path / "slow_driver.py"
    stub.write_text("import time; time.sleep(30)\n")
    out = tmp_path / "w.csv"
    args = _parse(m4, ["--wind-still-air", "--wind-seed", "7",
                       "--wind-height-m", "6.0"])
    proc, path = m4.spawn_wind_driver(args, str(out), script=str(stub),
                                      settle_s=0.3)
    try:
        assert proc.poll() is None
        assert path == str(out)
    finally:
        proc.kill()
        proc.wait(timeout=5)


# ======================================================================== 6
# THE APPLIED SUMMARY NEVER PASSES ON NOTHING
# ======================================================================== 6
def test_applied_summary_on_a_missing_or_empty_log_is_never_a_pass(tmp_path):
    assert "UNKNOWN" in m4.summarize_applied_wind(None)
    assert "NONE" in m4.summarize_applied_wind(str(tmp_path / "absent.csv"), 1)
    empty = tmp_path / "empty.csv"
    wind_driver.write_rows_csv(str(empty), [], ["header only"])
    s = m4.summarize_applied_wind(str(empty), 0)
    assert "NONE" in s and "ZERO rows" in s and "discard" in s


def test_applied_summary_reads_the_drivers_own_writer(tmp_path):
    """Second producer->consumer contract: the fixture is written by
    wind_driver.write_rows_csv, not typed here, so a column rename breaks this
    test instead of silently breaking the flight log."""
    drag = wind_model.DragParams.px4_x500_mono_cam_sitl()
    cond = wind_model.tier("EXPECTED", height_m=6.0, dir_from_deg=270.0)
    core = wind_driver.WindDriverCore(
        wind_model.WindField(cond, seed=7, sim_step_s=0.05), drag,
        lambda fx, fy, fz: True)
    for i in range(60):
        t = i * 0.05
        core.on_pose(t, 0.0, 0.0)
        core.tick(t)
    path = tmp_path / "wind.csv"
    wind_driver.write_rows_csv(str(path), core.rows,
                               wind_driver.header_lines(cond, drag, _FakeArgs()))
    s = m4.summarize_applied_wind(str(path), 0)
    assert "WIND APPLIED:" in s and "/59 wrenches published" in s
    assert "mean |F|=" in s and "max wind speed=" in s
    assert core.n_published == 59


class _FakeArgs:
    seed = 7
    rate_hz = 20.0


def test_applied_summary_calls_out_a_driver_that_published_nothing(tmp_path):
    drag = wind_model.DragParams.px4_x500_mono_cam_sitl()
    cond = wind_model.tier("WORST", height_m=6.0, dir_from_deg=0.0)
    core = wind_driver.WindDriverCore(
        wind_model.WindField(cond, seed=1, sim_step_s=0.05), drag,
        lambda fx, fy, fz: False)          # every publish silently "fails"
    for i in range(10):
        core.on_pose(i * 0.05, 0.0, 0.0)
        core.tick(i * 0.05)
    path = tmp_path / "failed.csv"
    wind_driver.write_rows_csv(str(path), core.rows, ["x"])
    s = m4.summarize_applied_wind(str(path), 0)
    assert "NONE" in s and "0 published" in s and "discard" in s


# ======================================================================== 7
# FRAMES AND HYGIENE
# ======================================================================== 7
def test_force_north_east_maps_to_gazebo_enu_y_x():
    """ADR-0013: east = world_x, north = world_y. A sign/axis swap here would
    blow the airframe 90 deg away from the configured wind and no test of the
    wind MODEL could see it."""
    f = wind_model.WindForce(f_n=3.0, f_e=-4.0, f_d=0.0)
    x, y, z = wind_driver.force_to_world_xyz(f)
    assert (x, y, z) == (-4.0, 3.0, 0.0)


def _shell_body(src, func):
    """The EXECUTABLE lines of a shell function, comments stripped.

    Comments are stripped because the first version of the test below searched
    the raw body for "wind_driver.py" -- and the explanatory COMMENT above the
    pkill line contains that string, so deleting the actual command left the
    test green. A mutation run caught it. That is the instruments-are-evidence
    rule biting the instrument: a check that passes on prose checks nothing.
    """
    body = src.split(f"{func}() {{")[1].split("\n}")[0]
    return "\n".join(ln for ln in body.splitlines()
                     if not ln.strip().startswith("#"))


@pytest.mark.parametrize("func", ["kill_stale_sim", "teardown_sim"])
def test_mc_batch_reaps_a_stranded_wind_driver(func):
    """A killed driver's PERSISTENT wrench keeps blowing on the NEXT flight --
    contamination only the wind arms could suffer, which a paired control
    structurally cannot see."""
    body = _shell_body(open(MC_BATCH, encoding="utf-8").read(), func)
    assert 'pkill -f "wind_driver.py"' in body, (
        f"{func} does not reap wind_driver.py; a stranded driver would apply "
        "its persistent wrench to subsequent flights")


def test_wind_driver_offline_selftest_passes():
    r = subprocess.run([sys.executable, os.path.join(SCRIPTS, "wind_driver.py"),
                        "--offline-selftest"], capture_output=True, text=True,
                       timeout=120)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "offline self-test checks passed" in r.stdout
    # The honest caveat must survive in the output, not just in a docstring.
    assert "does NOT" in r.stdout and "Gate-0" in r.stdout


def test_m4_never_imports_the_wind_injection_side():
    """HONESTY BOUNDARY. m4_intercept.py may import the FLIGHT-side trim; it
    must never import wind_model or wind_driver, because those hold the true
    wind vector. The injection lives in a separate PROCESS for exactly this
    reason."""
    src = open(M4_PATH, encoding="utf-8").read()
    for banned in ("import wind_model", "from wind_model",
                   "import wind_driver", "from wind_driver",
                   "import wind_anemometer_sim", "from wind_anemometer_sim"):
        assert banned not in src, (
            f"m4_intercept.py imports {banned!r} -- that is a path from the "
            "sim's TRUE wind into the flight harness")


def test_the_trim_is_reachable_only_through_operator_typed_numbers():
    """The only wind value m4 can consume is what the operator types. Assert the
    flag help says so and that no --wind-trim-* value is derived from a
    --wind-* injection flag anywhere in the file."""
    src = open(M4_PATH, encoding="utf-8").read()
    assert "args.wind_trim_mps" in src
    # the injection values must never appear inside a trim expression
    for line in src.splitlines():
        if "wind_trim" in line and "wind_mps" in line and "wind_trim_mps" not in line:
            raise AssertionError(f"injection value leaks into the trim: {line}")


if __name__ == "__main__":                              # pragma: no cover
    sys.exit(pytest.main([__file__, "-q"]))
