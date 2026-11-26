"""A FAKE `mavsdk` package + a virtual-time asyncio loop, so the LIVE driver in
flight/deploy/real_flight.py (`run_mavsdk_mission`, `run_bench_props_off`) can be
driven end to end on the desk: no PX4, no simulator, no hardware, no real seconds.

WHY: the live driver was `# pragma: no cover -- needs a vehicle`, so every
property of it (who arms, what teardown sends, what happens when a stream
freezes) was readable but untestable. The fake records every actuation call the
driver makes, which is exactly what the safety tests need to assert on.

It is a TEST DOUBLE, not a flight model: velocity is the commanded velocity,
yaw is the commanded yaw, takeoff is a ramp. It answers "what did the driver
ask the vehicle to do, and when" -- nothing about how an aircraft flies.

Not collected by pytest (no `test_` prefix); imported by test_real_flight_driver.py.
"""
from __future__ import annotations

import asyncio
import json
import math
import sys
import types
from typing import Callable, Dict, List, Optional, Tuple


class VirtualTimeLoop(asyncio.SelectorEventLoop):
    """An event loop whose clock JUMPS instead of waiting: whenever the loop
    would block until its next timer, virtual time advances by that amount and
    the loop carries on. `asyncio.sleep`, `wait_for` and `call_later` all run
    on it, so a 60 s mission takes milliseconds and is deterministic under any
    machine load."""

    def __init__(self):
        super().__init__()
        self._vt = 1000.0
        real_select = self._selector.select

        def select(timeout=None):
            if timeout is not None and timeout > 0:
                self._vt += timeout
            return real_select(0)

        self._selector.select = select

    def time(self):
        return self._vt


class ClockShim:
    """Stands in for the `time` module inside real_flight (the same seam the
    Gazebo cross-check uses): monotonic() = the virtual loop's clock."""

    def __init__(self, loop, real_time_module):
        self._loop = loop
        self._real = real_time_module

    def monotonic(self):
        return self._loop.time()

    def __getattr__(self, name):
        return getattr(self._real, name)


class _Mode:
    def __init__(self, name):
        self.name = name

    def __str__(self):
        return self.name


class LandedState:
    ON_GROUND = "ON_GROUND"
    IN_AIR = "IN_AIR"


class OffboardError(Exception):
    pass


class ParamError(Exception):
    pass


class VelocityNedYaw:
    def __init__(self, north_m_s, east_m_s, down_m_s, yaw_deg):
        self.north_m_s, self.east_m_s = north_m_s, east_m_s
        self.down_m_s, self.yaw_deg = down_m_s, yaw_deg


DEFAULT_PARAMS = {
    # the interceptor's board as read 2026-11-18 (runs/fc_readonly_20261118/)
    "MPC_XY_VEL_MAX": 12.0, "MPC_TILTMAX_AIR": 45.0, "MPC_ACC_HOR_MAX": 5.0,
    "GF_ACTION": 2, "GF_MAX_HOR_DIST": 457.2, "GF_MAX_VER_DIST": 30.48,
    "COM_RCL_EXCEPT": 0, "COM_OF_LOSS_T": 1.0, "COM_OBL_RC_ACT": 0,
    # read 2026-11-26 (runs/interceptor_redundancy_20261126/)
    "BAT_LOW_THR": 0.15, "BAT_CRIT_THR": 0.07,
    "EKF2_REQ_NSATS": 6, "EKF2_REQ_EPH": 3.0, "EKF2_REQ_SACC": 1.0,
    "COM_LOW_BAT_ACT": 3, "COM_FLTT_LOW_ACT": 3, "EKF2_GPS_CHECK": 1023,
}


class FakeDrone:
    """One fake vehicle. Everything a test wants to script or read is a plain
    attribute; `calls` is the ordered record of what the driver asked for."""

    def __init__(self):
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.calls: List[Tuple[float, str, tuple]] = []
        self.armed = False
        self.mode = "HOLD"
        self.alt = 0.0
        self.yaw_deg = 0.0
        self.vel = (0.0, 0.0, 0.0)
        self.landed = LandedState.ON_GROUND
        self.battery_v: Optional[float] = 0.0
        self.battery_pct: Optional[float] = 80.0    # MAVSDK's 0..100 scale
        # GPS quality as MAVSDK reports it (gps_info + raw_gps)
        self.gps_sats = 14
        self.gps_fix = 3
        self.gps_hacc = 0.8
        self.gps_sacc = 0.2
        self.param_set_fails: set = set()           # names whose write raises
        self.param_set_ignored: set = set()         # names whose write is lost
        self.params: Dict[str, float] = dict(DEFAULT_PARAMS)
        self.health_ok = True
        self.takeoff_alt = 2.5
        self._target_alt: Optional[float] = None
        self._sp = (0.0, 0.0, 0.0, 0.0)
        # scripts
        self.rc_script: Optional[Callable[[float], Optional[dict]]] = None
        self.rc_period_s = 0.2                      # PX4 onboard-mode RC_CHANNELS
        self.mode_script: Optional[Callable[[float, str], Optional[str]]] = None
        self.freeze_after: Dict[str, float] = {}    # stream name -> mission t
        self.arm_script: Optional[Callable[[float], Optional[bool]]] = None
        # --trigger cue support: own GPS position (lat, lon, launch height AMSL;
        # None = the vehicle publishes no global position), the UTC the
        # vehicle reports in SYSTEM_TIME (seconds at connect; None = no GPS
        # time, the message carries zero), and a hook the physics loop calls
        # every 10 ms of fake time so a test can feed the cue link on the
        # same clock the driver runs on.
        self.lla: Optional[Tuple[float, float, float]] = (40.0, -105.0, 1650.0)
        self.utc0: Optional[float] = 1.8e9
        self.systime_period_s = 1.0
        self.tick_hook: Optional[Callable[[float], None]] = None
        self.pos_ne = [0.0, 0.0]                    # dead-reckoned, for lla
        self.t0: Optional[float] = None
        self._server_process = None                 # tests may attach a Popen
        self.connect_url: Optional[str] = None

        self.core = types.SimpleNamespace(connection_state=self._connection_state)
        self.telemetry = types.SimpleNamespace(
            attitude_quaternion=lambda: self._stream("quat", 0.02, self._quat),
            attitude_euler=lambda: self._stream("euler", 0.02, self._euler),
            position=lambda: self._stream("pos", 0.02, self._pos),
            velocity_ned=lambda: self._stream("vel", 0.033, self._vel),
            armed=lambda: self._stream("armed", 0.1, lambda: self.armed),
            landed_state=lambda: self._stream("landed", 0.2, lambda: self.landed),
            flight_mode=lambda: self._stream("mode", 0.1, lambda: _Mode(self.mode)),
            battery=lambda: self._stream("battery", 0.5, self._battery),
            gps_info=lambda: self._stream("gps_info", 0.2, lambda: types.SimpleNamespace(
                num_satellites=self.gps_sats, fix_type=self.gps_fix)),
            raw_gps=lambda: self._stream("raw_gps", 0.2, lambda: types.SimpleNamespace(
                horizontal_uncertainty_m=self.gps_hacc,
                velocity_uncertainty_m_s=self.gps_sacc)),
            health=self._health)
        self.action = types.SimpleNamespace(
            set_takeoff_altitude=self._set_takeoff_altitude, arm=self._arm,
            takeoff=self._takeoff, land=self._land,
            return_to_launch=self._rtl, disarm=self._disarm)
        self.offboard = types.SimpleNamespace(
            set_velocity_ned=self._set_velocity_ned, start=self._offb_start,
            stop=self._offb_stop)
        self.param = types.SimpleNamespace(
            get_param_float=self._get_param, get_param_int=self._get_param,
            set_param_int=self._set_param)
        self.mavlink_direct = types.SimpleNamespace(message=self._mav_message)

    # ---- helpers ---------------------------------------------------------
    def now(self) -> float:
        return asyncio.get_event_loop().time()

    def t(self) -> float:
        return 0.0 if self.t0 is None else self.now() - self.t0

    def _rec(self, name, *args):
        self.calls.append((self.t(), name, args))

    def names(self) -> List[str]:
        return [c[1] for c in self.calls]

    def count(self, name) -> int:
        return sum(1 for c in self.calls if c[1] == name)

    async def connect(self, system_address=None):
        self.connect_url = system_address
        self.t0 = self.now()
        asyncio.ensure_future(self._physics())
        self._rec("connect", system_address)

    async def _physics(self):
        dt = 0.01
        while True:
            await asyncio.sleep(dt)
            t = self.t()
            if self.mode_script is not None:
                new = self.mode_script(t, self.mode)
                if new is not None and new != self.mode:
                    self.mode = new
            if self.arm_script is not None:
                a = self.arm_script(t)
                if a is not None:
                    self.armed = a
            if self.tick_hook is not None:
                self.tick_hook(t)
            if self.mode == "OFFBOARD" and self.armed:
                vn, ve, vd, yaw = self._sp
                self.vel = (vn, ve, vd)
                self.alt += -vd * dt
                self.pos_ne[0] += vn * dt
                self.pos_ne[1] += ve * dt
                self.yaw_deg = yaw
            elif self.mode == "OFFBOARD":
                self.yaw_deg = self.yaw_deg     # disarmed: nothing moves
                self.vel = (0.0, 0.0, 0.0)
            elif self.mode == "TAKEOFF" and self._target_alt is not None:
                self.alt = min(self._target_alt, self.alt + 3.0 * dt)
                if self.alt >= self._target_alt - 1e-6:
                    self.mode = "HOLD"
            elif self.mode in ("LAND", "RETURN_TO_LAUNCH"):
                self.vel = (0.0, 0.0, 0.0)
                self.alt = max(0.0, self.alt - 3.0 * dt)
                if self.alt <= 0.0:
                    self.landed = LandedState.ON_GROUND
                    self.armed = False
            else:
                self.vel = (0.0, 0.0, 0.0)
            if self.alt > 0.05:
                self.landed = LandedState.IN_AIR

    async def _stream(self, name, period, getter):
        while True:
            lim = self.freeze_after.get(name)
            if lim is not None and self.t() >= lim:
                await asyncio.sleep(3600.0)     # frozen: never yields again
                continue
            v = getter()
            if v is not None:
                yield v
            await asyncio.sleep(period)

    def _quat(self):
        h = math.radians(self.yaw_deg) / 2.0
        return types.SimpleNamespace(w=math.cos(h), x=0.0, y=0.0, z=math.sin(h))

    def _euler(self):
        return types.SimpleNamespace(yaw_deg=self.yaw_deg)

    def _pos(self):
        if self.lla is None:
            return types.SimpleNamespace(relative_altitude_m=self.alt)
        r = 6378137.0
        lat = self.lla[0] + math.degrees(self.pos_ne[0] / r)
        lon = self.lla[1] + math.degrees(
            self.pos_ne[1] / (r * math.cos(math.radians(self.lla[0]))))
        return types.SimpleNamespace(relative_altitude_m=self.alt,
                                     latitude_deg=lat, longitude_deg=lon,
                                     absolute_altitude_m=self.lla[2] + self.alt)

    def _vel(self):
        return types.SimpleNamespace(north_m_s=self.vel[0], east_m_s=self.vel[1],
                                     down_m_s=self.vel[2])

    def _battery(self):
        if self.battery_v is None:
            return None
        return types.SimpleNamespace(
            voltage_v=self.battery_v,
            remaining_percent=(-1.0 if self.battery_pct is None
                               else self.battery_pct))

    async def _health(self):
        while True:
            yield types.SimpleNamespace(is_global_position_ok=self.health_ok,
                                        is_home_position_ok=self.health_ok)
            await asyncio.sleep(0.5)

    async def _connection_state(self):
        while True:
            yield types.SimpleNamespace(is_connected=True)
            await asyncio.sleep(1.0)

    async def _mav_message(self, name):
        assert name in ("RC_CHANNELS", "SYSTEM_TIME"), name
        while name == "SYSTEM_TIME":
            await asyncio.sleep(self.systime_period_s)
            unix = 0.0 if self.utc0 is None else self.utc0 + self.t()
            yield types.SimpleNamespace(fields_json=json.dumps(
                {"time_unix_usec": int(unix * 1e6),
                 "time_boot_ms": int(self.t() * 1e3)}))
        while True:
            await asyncio.sleep(self.rc_period_s)
            if self.rc_script is None:
                continue
            fields = self.rc_script(self.t())
            if fields is not None:
                yield types.SimpleNamespace(fields_json=json.dumps(fields))

    # ---- actuation (recorded) ---------------------------------------------
    async def _set_takeoff_altitude(self, alt):
        self._rec("action.set_takeoff_altitude", alt)
        self.takeoff_alt = alt

    async def _arm(self):
        self._rec("action.arm")
        self.armed = True

    async def _takeoff(self):
        self._rec("action.takeoff")
        self._target_alt = self.takeoff_alt
        self.mode = "TAKEOFF"

    async def _land(self):
        self._rec("action.land")
        self.mode = "LAND"

    async def _rtl(self):
        self._rec("action.return_to_launch")
        self.mode = "RETURN_TO_LAUNCH"

    async def _disarm(self):
        self._rec("action.disarm")
        self.armed = False

    async def _set_velocity_ned(self, sp):
        self._sp = (sp.north_m_s, sp.east_m_s, sp.down_m_s, sp.yaw_deg)
        self._rec("offboard.set_velocity_ned", *self._sp)

    async def _offb_start(self):
        self._rec("offboard.start")
        self.mode = "OFFBOARD"

    async def _offb_stop(self):
        self._rec("offboard.stop")
        self.mode = "HOLD"          # MAVSDK's Offboard.stop() commands HOLD

    async def _get_param(self, name):
        self._rec("param.get", name)
        if name not in self.params:
            raise ParamError(f"no such parameter {name}")
        return self.params[name]


    async def _set_param_placeholder(self):  # pragma: no cover
        raise NotImplementedError


def _attach_set_param():
    async def _set_param(self, name, value):
        self._rec("param.set", name, value)
        if name in self.param_set_fails:
            raise ParamError(f"write of {name} refused")
        if name not in self.param_set_ignored:
            self.params[name] = value
    FakeDrone._set_param = _set_param


_attach_set_param()


def install(monkeypatch, drone: FakeDrone):
    """Put the fake `mavsdk` package in sys.modules so the driver's guarded
    `from mavsdk import System` picks it up. `System()` returns `drone`."""
    pkg = types.ModuleType("mavsdk")
    pkg.System = lambda *a, **k: drone
    offb = types.ModuleType("mavsdk.offboard")
    offb.OffboardError, offb.VelocityNedYaw = OffboardError, VelocityNedYaw
    tel = types.ModuleType("mavsdk.telemetry")
    tel.LandedState = LandedState
    par = types.ModuleType("mavsdk.param")
    par.ParamError = ParamError
    pkg.offboard, pkg.telemetry, pkg.param = offb, tel, par
    monkeypatch.setitem(sys.modules, "mavsdk", pkg)
    monkeypatch.setitem(sys.modules, "mavsdk.offboard", offb)
    monkeypatch.setitem(sys.modules, "mavsdk.telemetry", tel)
    monkeypatch.setitem(sys.modules, "mavsdk.param", par)


def run_virtual(coro_factory, rf_module, monkeypatch, timeout_s=600.0):
    """Run one driver coroutine to completion on a fresh virtual-time loop with
    real_flight's `time` swapped for the loop's clock. Returns its result."""
    import time as real_time
    loop = VirtualTimeLoop()
    asyncio.set_event_loop(loop)
    monkeypatch.setattr(rf_module, "time", ClockShim(loop, real_time))
    try:
        return loop.run_until_complete(
            asyncio.wait_for(coro_factory(), timeout=timeout_s))
    finally:
        for task in asyncio.all_tasks(loop):
            task.cancel()
        try:
            loop.run_until_complete(asyncio.sleep(0))
        except Exception:  # noqa: BLE001 -- best-effort drain of cancelled tasks
            pass
        asyncio.set_event_loop(None)
        loop.close()
