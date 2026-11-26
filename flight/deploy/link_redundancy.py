#!/usr/bin/env python3
"""flight/deploy/link_redundancy.py -- two MAVLink links to ONE flight controller,
used as one.

THE FAULT IT EXISTS FOR (builder request 2026-11-26): the flight program talks
to the Pixhawk over ONE cable, the Pi's serial port to TELEM2. If that cable or
its connector fails in flight, the Pixhawk stops receiving velocity commands,
leaves OFFBOARD after COM_OF_LOSS_T (1.0 s on this board) and the intercept is
over. A second cable is already plugged in: the Pixhawk's USB port, held by the
flight recorder, which passes MAVLink to and from UDP port 14550 on the Pi.

WHAT THIS DOES: it wraps two MAVSDK `System` objects (one per link, each with
its own mavsdk_server) behind the handful of calls the flight driver makes.

  * VELOCITY COMMANDS go out on BOTH links every tick. PX4 accepts offboard
    setpoints on any MAVLink port, and both carry the same values, so there is
    no switch-over and no gap when one link dies -- including a break in the
    Pi-to-Pixhawk wire alone, which no telemetry check on the Pi could see.
  * TELEMETRY is read from the PRIMARY link. A sample from the BACKUP link is
    used only when the primary has been silent on that same stream for longer
    than its quiet time (`prefer_primary`). So a healthy flight is flown on
    exactly the numbers it was flown on before, and the backup's slightly later
    copies can never overwrite newer primary ones.
  * COMMANDS (arm, takeoff, land, return, parameter reads and writes) go to the
    primary; if that call fails they are tried once on the backup.
  * A link whose server does not take a setpoint (it hangs, or errors several
    ticks running) is ENDED: its server is killed. A server left alive would
    keep re-sending its last setpoint on its own timer, against the fresh
    setpoints still flowing on the good link.

WHAT IT DOES NOT COVER (say it plainly): the Pixhawk itself, the Pi itself, the
Pi's power, and this program. The backup link runs through the flight
recorder's bridge, so it is only as alive as the recorder. Both links dead is
the same as one link dead was before: PX4's offboard-loss failsafe takes the
aircraft.

It sends nothing of its own accord and imports no MAVSDK: it only forwards the
calls it is given, so every path runs offline against a fake.
"""
from __future__ import annotations

import asyncio
import time
from typing import Callable, Dict, List, Optional

# Quiet time (s) before a backup sample of a stream is used, by stream class.
# FAST: the own-state streams the guidance steers on (30 to 100 Hz on TELEM2,
# runs/fc_readonly_20261118/). 0.12 s is 4 missed samples of the slowest, and
# with the backup's own period on top stays under the terminal's 0.25 s
# own-state bound (GuidanceConfig.own_state_max_age_s).
FAST_QUIET_S = 0.12
# STATUS: streams that arrive about once a second (flight mode, armed, landed,
# battery, health). 1.5 s is one missed sample plus margin; with the backup's
# 1 s period on top the flight-mode sample stays younger than FAILSAFE 7's
# 3.0 s staleness bound (MissionConfig.offboard_stale_s).
STATUS_QUIET_S = 1.5
# RC_CHANNELS: the GO and arm-permit switches. Under MissionConfig.link_timeout_s
# (1.0 s) so a dead primary does not read as a dead radio.
RC_QUIET_S = 0.5
FAST_STREAMS = frozenset({"attitude_quaternion", "attitude_euler", "position",
                          "velocity_ned", "position_velocity_ned"})
DIRECT_QUIET_S = {"RC_CHANNELS": RC_QUIET_S}

# A setpoint call that takes longer than this has hung; its link is ended at
# once. Well under one control tick's worth of the watchdog bound.
SEND_TIMEOUT_S = 0.25
# Consecutive setpoint calls that RAISE before the link is ended.
SEND_FAILS_TO_END = 3


class LinkDown(RuntimeError):
    """No link accepted the call. The driver treats it like any other failure
    of the control path."""


class StreamStats:
    """What one merged stream did. Read by the end-of-flight link report."""

    def __init__(self, name: str):
        self.name = name
        self.n_primary = 0
        self.n_backup = 0          # backup samples USED (primary was silent)
        self.n_backup_dropped = 0  # backup samples dropped (primary was live)
        self.switches = 0          # times the stream fell back to the backup
        self.on_backup = False
        self.longest_primary_gap_s = 0.0


async def prefer_primary(primary_factory: Callable, backup_factory: Callable,
                         quiet_s: float, stats: Optional[StreamStats] = None,
                         clock: Callable[[], float] = time.monotonic,
                         say: Optional[Callable[[str], None]] = None):
    """Merge two async streams of the SAME quantity into one.

    Every primary item is passed on. A backup item is passed on only when no
    primary item has arrived for `quiet_s` (or none ever has). The merged
    stream ends only when BOTH sources have ended; if both ended on an
    exception the primary's is raised, so a caller that reports dead streams
    still hears about it."""
    stats = stats if stats is not None else StreamStats("stream")
    q: asyncio.Queue = asyncio.Queue()
    _END = object()

    async def pump(tag: str, factory: Callable):
        try:
            async for item in factory():
                await q.put((tag, item))
            await q.put((tag, _END, None))
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001 -- reported, never swallowed
            await q.put((tag, _END, e))

    tasks = [asyncio.ensure_future(pump("primary", primary_factory)),
             asyncio.ensure_future(pump("backup", backup_factory))]
    ended: Dict[str, Optional[BaseException]] = {}
    last_primary: Optional[float] = None
    try:
        while len(ended) < 2:
            got = await q.get()
            tag, item = got[0], got[1]
            if item is _END:
                ended[tag] = got[2]
                if say is not None and got[2] is not None:
                    say(f"[link] {stats.name}: the {tag} link's stream died "
                        f"({type(got[2]).__name__}: {got[2]})")
                continue
            now = clock()
            if tag == "primary":
                if last_primary is not None:
                    stats.longest_primary_gap_s = max(
                        stats.longest_primary_gap_s, now - last_primary)
                last_primary = now
                stats.n_primary += 1
                if stats.on_backup:
                    stats.on_backup = False
                    if say is not None:
                        say(f"[link] {stats.name}: the primary link is back")
                yield item
                continue
            if last_primary is not None and (now - last_primary) <= quiet_s \
                    and "primary" not in ended:
                stats.n_backup_dropped += 1
                continue
            stats.n_backup += 1
            if not stats.on_backup:
                stats.on_backup = True
                stats.switches += 1
                if say is not None:
                    gap = ("never spoke" if last_primary is None
                           else f"silent {now - last_primary:.2f} s")
                    say(f"[link] {stats.name}: primary {gap} -- using the "
                        f"BACKUP link")
            yield item
        err = ended.get("primary") or ended.get("backup")
        if err is not None:
            raise err
    finally:
        for tk in tasks:
            tk.cancel()


class _Link:
    def __init__(self, name: str, system):
        self.name = name
        self.system = system
        self.alive = True
        self.send_fails = 0        # consecutive
        self.n_sent = 0
        self.n_failed = 0
        self.ended_why: Optional[str] = None

    def server_process(self):
        return getattr(self.system, "_server_process", None)

    def end(self, why: str) -> None:
        """Kill this link's server so it cannot keep re-sending a setpoint."""
        if not self.alive:
            return
        self.alive = False
        self.ended_why = why
        proc = self.server_process()
        try:
            if proc is not None and proc.poll() is None:
                proc.kill()
        except Exception:  # noqa: BLE001 -- best effort; the link is dead to us
            pass


class _Telemetry:
    def __init__(self, owner: "RedundantSystem"):
        self._o = owner

    def __getattr__(self, name: str):
        o = self._o

        def stream(*a, **k):
            quiet = FAST_QUIET_S if name in FAST_STREAMS else STATUS_QUIET_S
            return o._merged(f"telemetry.{name}", quiet,
                             lambda s: getattr(s.telemetry, name)(*a, **k))
        return stream


class _Direct:
    def __init__(self, owner: "RedundantSystem"):
        self._o = owner

    def message(self, msg_name: str, *a, **k):
        quiet = DIRECT_QUIET_S.get(msg_name, STATUS_QUIET_S)
        return self._o._merged(
            f"mavlink.{msg_name}", quiet,
            lambda s: s.mavlink_direct.message(msg_name, *a, **k))


class _Offboard:
    def __init__(self, owner: "RedundantSystem"):
        self._o = owner

    async def set_velocity_ned(self, sp):
        """Send one setpoint on EVERY live link. Raises LinkDown only when no
        link took it."""
        o = self._o
        live = [ln for ln in o.links if ln.alive]
        if not live:
            raise LinkDown("no MAVLink link is left")

        async def one(ln: _Link):
            await asyncio.wait_for(ln.system.offboard.set_velocity_ned(sp),
                                   timeout=o.send_timeout_s)

        results = await asyncio.gather(*(one(ln) for ln in live),
                                       return_exceptions=True)
        n_ok = 0
        for ln, res in zip(live, results):
            if isinstance(res, asyncio.CancelledError):
                raise res
            if not isinstance(res, BaseException):
                ln.send_fails = 0
                ln.n_sent += 1
                n_ok += 1
                continue
            ln.send_fails += 1
            ln.n_failed += 1
            hung = isinstance(res, asyncio.TimeoutError)
            if hung or ln.send_fails >= o.send_fails_to_end:
                why = (f"setpoint call hung > {o.send_timeout_s:.2f} s" if hung
                       else f"{ln.send_fails} setpoint calls failed running "
                            f"({type(res).__name__}: {res})")
                ln.end(why)
                others = [x.name for x in o.links if x.alive]
                o.say(f"[link] FAULT: the {ln.name} link is ENDED ({why}); "
                      f"its server is killed so it cannot re-send an old "
                      f"setpoint. Flying on: "
                      f"{', '.join(others) if others else 'NOTHING'}")
        if n_ok == 0:
            raise LinkDown("no MAVLink link accepted the setpoint")

    async def start(self):
        return await self._o._first("offboard.start",
                                    lambda s: s.offboard.start())

    async def stop(self):
        """Stop on every live link: each server is streaming setpoints."""
        o = self._o
        errs = []
        for ln in [x for x in o.links if x.alive]:
            try:
                await ln.system.offboard.stop()
            except Exception as e:  # noqa: BLE001
                errs.append(e)
        if errs and len(errs) == len([x for x in o.links if x.alive]):
            raise errs[0]

    async def is_active(self):
        return await self._o._first("offboard.is_active",
                                    lambda s: s.offboard.is_active())


class _FirstThatWorks:
    """`action` and `param`: the primary link first, the backup if it fails."""

    def __init__(self, owner: "RedundantSystem", plugin: str):
        self._o, self._plugin = owner, plugin

    def __getattr__(self, name: str):
        o, plugin = self._o, self._plugin

        async def call(*a, **k):
            return await o._first(
                f"{plugin}.{name}",
                lambda s: getattr(getattr(s, plugin), name)(*a, **k))
        return call


class RedundantSystem:
    """Two MAVSDK `System`s to the same flight controller behind the calls the
    flight driver makes: `.telemetry`, `.mavlink_direct`, `.offboard`,
    `.action`, `.param`. See the module header for the rules."""

    def __init__(self, primary, backup, *, send_timeout_s: float = SEND_TIMEOUT_S,
                 send_fails_to_end: int = SEND_FAILS_TO_END,
                 clock: Callable[[], float] = time.monotonic,
                 say: Callable[[str], None] = print):
        self.links: List[_Link] = [_Link("primary", primary),
                                   _Link("backup", backup)]
        self.send_timeout_s = float(send_timeout_s)
        self.send_fails_to_end = int(send_fails_to_end)
        self.clock = clock
        self.say = say
        self.stats: Dict[str, StreamStats] = {}
        self.telemetry = _Telemetry(self)
        self.mavlink_direct = _Direct(self)
        self.offboard = _Offboard(self)
        self.action = _FirstThatWorks(self, "action")
        self.param = _FirstThatWorks(self, "param")

    # -- what the driver's process handling reads --------------------------------
    @property
    def _server_process(self):
        """The first live server (the watchdog and teardown use
        `_server_processes`; this keeps single-server callers working)."""
        procs = self._server_processes
        return procs[0] if procs else None

    @property
    def _server_processes(self) -> list:
        return [p for p in (ln.server_process() for ln in self.links)
                if p is not None]

    # -- internals ----------------------------------------------------------------
    def _merged(self, name: str, quiet_s: float, make: Callable):
        primary, backup = self.links[0].system, self.links[1].system
        st = self.stats.setdefault(name, StreamStats(name))
        return prefer_primary(lambda: make(primary), lambda: make(backup),
                              quiet_s, st, self.clock, self.say)

    async def _first(self, what: str, call: Callable):
        last: Optional[BaseException] = None
        for ln in self.links:
            if not ln.alive:
                continue
            try:
                return await call(ln.system)
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001 -- try the other link
                last = e
                self.say(f"[link] {what} failed on the {ln.name} link "
                         f"({type(e).__name__}: {e})")
        if last is None:
            raise LinkDown(f"{what}: no MAVLink link is left")
        raise last

    # -- reporting ----------------------------------------------------------------
    def report_lines(self) -> List[str]:
        """Plain end-of-flight account of what each link did."""
        out = []
        for ln in self.links:
            state = "alive" if ln.alive else f"ENDED ({ln.ended_why})"
            out.append(f"[link] {ln.name}: {state}; setpoints sent {ln.n_sent}, "
                       f"failed {ln.n_failed}")
        for st in sorted(self.stats.values(), key=lambda s: s.name):
            if st.n_backup or st.switches:
                out.append(f"[link] {st.name}: fell back to the backup "
                           f"{st.switches} time(s), {st.n_backup} backup "
                           f"samples used; longest primary gap "
                           f"{st.longest_primary_gap_s:.2f} s")
        if not any(st.n_backup for st in self.stats.values()):
            out.append("[link] telemetry: the backup link was never needed")
        return out

    def backup_alive(self) -> bool:
        return self.links[1].alive

    def primary_alive(self) -> bool:
        return self.links[0].alive
