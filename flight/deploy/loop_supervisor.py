#!/usr/bin/env python3
"""flight/deploy/loop_supervisor.py -- the setpoint stream's OUT-OF-PROCESS watchdog.

THE FAULT IT EXISTS FOR (interface audit 2026-11-18, gap G2): MAVSDK-Python
talks to PX4 through a child process, `mavsdk_server`, and that server re-sends
the LAST velocity setpoint on its own timer. So when the Pi's flight program
hangs, is stopped, or is killed, PX4 still sees a healthy OFFBOARD stream and
keeps flying the last command -- in an approach dash, a straight line at dash
speed. PX4's own offboard-loss failsafe (COM_OF_LOSS_T) never fires, because
from where it sits nothing was lost.

WHAT THIS DOES: it is a separate, tiny, stdlib-only process. The flight program
writes one byte to its stdin on every control tick. If the bytes stop for
longer than `--bound-s`, or the pipe closes without a clean goodbye (the flight
program died: SIGKILL, out-of-memory kill, crash, dropped SSH session), it
kills `mavsdk_server` -- every one it was given (`--server-pid` once per
server: a flight with a backup link runs two, and one survivor would keep
re-sending the last command). The setpoint stream then ends for real and PX4's own
failsafe takes the aircraft (on this board: Position mode after 1.0 s,
runs/fc_readonly_20261118/).

WHY A PROCESS AND NOT A THREAD: a thread inside the flight program stops when
the program is stopped (SIGSTOP), dies when it is killed, and waits on the GIL
when a C extension holds it. A watchdog that shares its fate with the thing it
watches is not one.

PROTOCOL on stdin, one byte each:
    W   watch    -- start timing the gap between heartbeats
    .   heartbeat
    U   unwatch  -- a deliberate pause or a clean exit follows; stop timing,
                    and do NOT kill on EOF
  EOF without a preceding U means the flight program is gone: kill the server,
  watching or not.

EXIT CODES: 0 clean (U then EOF) | 2 usage | 3 heartbeat timeout, server killed
            | 4 flight program died, server killed

WHAT IT DOES NOT COVER (say it plainly): if THIS process is killed, the stream
is unwatched (the flight program checks it is alive every tick and stops flying
if not); a hang inside mavsdk_server itself; a Pi that loses power (then the
stream stops anyway); and a flight program that keeps ticking while computing
nonsense -- this watches liveness, not correctness.

It reads no sensor and sends nothing to the vehicle. Stdlib only.
"""
from __future__ import annotations

import argparse
import os
import select
import signal
import sys
import time


def _cmdline(pid: int) -> str:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as fh:
            return fh.read().replace(b"\0", b" ").decode(errors="replace")
    except OSError:
        return ""


def kill_server(pid: int, match: str) -> str:
    """SIGKILL `pid` if (and only if) its command line still contains `match`.
    The check is what stops a recycled pid from taking down a stranger.
    Returns what happened, for the log."""
    cmd = _cmdline(pid)
    if not cmd:
        return f"pid {pid} already gone"
    if match and match not in cmd:
        return f"pid {pid} is now '{cmd.strip()[:60]}' (no '{match}') -- NOT killed"
    try:
        os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        return f"pid {pid} already gone"
    except PermissionError as e:
        return f"pid {pid} could not be killed: {e}"
    return f"pid {pid} killed"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--server-pid", type=int, required=True, action="append",
                    help="a setpoint-sending server to kill; give it once per "
                         "server (a flight with a backup link runs two, and "
                         "BOTH must end or the survivor keeps re-sending the "
                         "last command)")
    ap.add_argument("--bound-s", type=float, required=True,
                    help="largest allowed gap between heartbeats while armed")
    ap.add_argument("--match", default="mavsdk_server",
                    help="text the server's command line must contain to be killed")
    ap.add_argument("--log", default=None, help="append one line per event here")
    args = ap.parse_args(argv)
    if args.bound_s <= 0:
        ap.error("--bound-s must be > 0")

    # A terminal Ctrl-C or a dropped SSH session must not take the watchdog
    # down before it has done its job.
    for sig in (signal.SIGINT, signal.SIGHUP, signal.SIGTERM):
        signal.signal(sig, signal.SIG_IGN)

    def log(msg: str) -> None:
        line = (f"{time.strftime('%Y-%m-%dT%H:%M:%S')} [loop-supervisor] {msg}")
        print(line, file=sys.stderr, flush=True)
        if args.log:
            try:
                with open(args.log, "a") as fh:
                    fh.write(line + "\n")
            except OSError:
                pass

    fd = sys.stdin.fileno()
    armed = False
    clean = False
    last = time.monotonic()
    n_beats = 0
    worst_gap = 0.0
    pids = list(dict.fromkeys(args.server_pid))

    def kill_all() -> str:
        return "; ".join(kill_server(p, args.match) for p in pids)

    log(f"watching server pid(s) {', '.join(map(str, pids))}, bound "
        f"{args.bound_s:.2f} s")
    while True:
        timeout = None
        if armed:
            timeout = max(0.0, args.bound_s - (time.monotonic() - last))
        ready, _, _ = select.select([fd], [], [], timeout)
        now = time.monotonic()
        if not ready:
            gap = now - last
            what = kill_all()
            log(f"TRIPPED: no control tick for {gap:.2f} s (bound "
                f"{args.bound_s:.2f} s, {n_beats} beats seen) -- {what}; the "
                f"setpoint stream is now ended and PX4's offboard-loss failsafe acts")
            return 3
        data = os.read(fd, 4096)
        if not data:
            if clean:
                log(f"clean exit ({n_beats} beats, worst gap {worst_gap:.3f} s)")
                return 0
            what = kill_all()
            log(f"TRIPPED: the flight program is gone without a goodbye "
                f"({n_beats} beats seen) -- {what}")
            return 4
        for b in data:
            ch = chr(b)
            if ch == "W":
                armed, clean, last = True, False, now
            elif ch == ".":
                if armed:
                    worst_gap = max(worst_gap, now - last)
                last = now
                n_beats += 1
            elif ch == "U":
                armed, clean = False, True


if __name__ == "__main__":
    raise SystemExit(main())
