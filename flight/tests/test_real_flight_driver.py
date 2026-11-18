"""Drive the LIVE driver in flight/deploy/real_flight.py end to end on the desk,
against a fake `mavsdk` (flight/tests/fake_mavsdk.py) on a virtual clock.

WHY THIS FILE EXISTS: the state machine was unit-tested but the driver around it
(`run_mavsdk_mission`) was `# pragma: no cover -- needs a vehicle`. Everything
that decides whether props spin lives in that driver: who arms, what teardown
sends, what happens when the Pi program stalls. The 2026-11-18 interface audit
found eight gaps there by reading; these tests pin the fixes by running.

Each fix has a test that FAILS without it (the assertion names the old
behaviour). No PX4, no simulator, no hardware.

Run: .venv/bin/python -m pytest flight/tests/test_real_flight_driver.py -q
"""
import asyncio
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_REPO_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fake_mavsdk as fm  # noqa: E402
import flight.deploy.real_flight as rf  # noqa: E402

# The SITL gate's own termination arguments (scripts/check_real_flight_sitl.sh):
# the synthetic engagement always ends in a miss, and landing after it is what
# ends the smoke.
SMOKE = ["--sitl-smoke", "--mavsdk-url", "udpin://fake", "--trigger", "gate-ready",
         "--miss-safe-behavior", "land", "--engage-max-s", "8"]


class Run:
    """One driver run: the exit code, the fake vehicle, and everything printed."""

    def __init__(self, rc, drone, out):
        self.rc, self.drone, self.out = rc, drone, out

    @property
    def actions(self):
        """Actuation calls in order, without the setpoint stream and param reads."""
        return [n for n in self.drone.names()
                if n != "offboard.set_velocity_ned" and not n.startswith("param.")
                and n != "connect"]


def drive(monkeypatch, capsys, tmp_path, argv, drone=None, detector=None):
    """Run `real_flight.main(argv)` with the fake vehicle on a virtual clock."""
    drone = drone or fm.FakeDrone()
    fm.install(monkeypatch, drone)
    monkeypatch.setattr(
        asyncio, "run",
        lambda coro: fm.run_virtual(lambda: coro, rf, monkeypatch))
    if detector is not None:
        monkeypatch.setattr(rf, "build_source_detector",
                            lambda args, gcfg, cam: (detector, None))
    argv = list(argv) + ["--log-csv", str(tmp_path / "ticks.csv")]
    try:
        rc = rf.main(argv)
    except SystemExit as e:      # argparse refusals exit through here
        rc = e.code
    out = capsys.readouterr()
    return Run(rc, drone, out.out + out.err)


# ---------------------------------------------------------------- baseline

@pytest.mark.parametrize("terminal,states", [
    ("pursuit", "STANDBY -> ENGAGE -> BREAKOFF -> SAFE"),
    ("stock", "STANDBY -> CODED_DASH -> ENGAGE -> BREAKOFF -> SAFE"),
])
def test_sitl_smoke_path_is_unchanged_over_udp(monkeypatch, capsys, tmp_path,
                                               terminal, states):
    """The regression harness for every change below: the SITL smoke path, with
    the gate's own arguments, still arms, takes off, flies the whole state
    sequence, lands and passes its own contract."""
    r = drive(monkeypatch, capsys, tmp_path, SMOKE + ["--terminal", terminal])
    assert r.rc == 0, r.out[-2000:]
    assert f"[mission] states visited: {states}" in r.out
    assert "[sitl-smoke] PASS" in r.out
    assert r.actions == ["action.set_takeoff_altitude", "action.arm",
                         "action.takeoff", "offboard.start", "offboard.stop",
                         "action.land"]
