"""End-to-end check of the watchdog against REAL PX4 (SIH, no Gazebo) and a REAL
mavsdk_server: fly the SITL smoke, freeze or kill the flight program mid-ENGAGE,
and watch what PX4's own flight mode does (independent observer on the GCS link).

usage: sih_hang.py stop|kill|pilot <watchdog_bound_s>
  stop  = SIGSTOP the flight program   kill = SIGKILL it
  pilot = switch PX4 to Position mode from a second client mid-ENGAGE
"""
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

from pymavlink import mavutil

WT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SP = os.path.join(tempfile.gettempdir(), "real_flight_sih")
os.makedirs(SP, exist_ok=True)
MAIN = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO", 6: "OFFBOARD",
        7: "STABILIZED"}
SUB = {2: "TAKEOFF", 3: "LOITER", 4: "MISSION", 5: "RTL", 6: "LAND"}


def mode_name(custom):
    main = (custom >> 16) & 0xFF
    sub = (custom >> 24) & 0xFF
    name = MAIN.get(main, f"main{main}")
    return f"AUTO.{SUB.get(sub, sub)}" if main == 4 else name


class Observer(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.conn = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
        self.log = []          # (t, mode, armed)
        self.cur = None
        self.stop = False

    def run(self):
        while not self.stop:
            m = self.conn.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if m is None or m.get_srcComponent() != 1:
                continue
            cur = (mode_name(m.custom_mode),
                   bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
            if cur != self.cur:
                self.cur = cur
                self.log.append((time.time(), cur[0], cur[1]))


def children(pid):
    out = subprocess.run(["ps", "-o", "pid=,comm=", "--ppid", str(pid)],
                         capture_output=True, text=True).stdout
    return [(int(l.split()[0]), l.split()[1]) for l in out.splitlines() if l.strip()]


def alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    try:
        with open(f"/proc/{pid}/stat") as fh:
            return fh.read().split(")")[-1].split()[0] != "Z"
    except OSError:
        return False


def main():
    how, bound = sys.argv[1], sys.argv[2]
    obs = Observer()
    obs.start()
    log_path = f"{SP}/sih_{how}_{bound}.log"
    cmd = [sys.executable, "-u", "-m", "flight.deploy.real_flight",
           "--sitl-smoke", "--mavsdk-url", "udpin://0.0.0.0:14541",
           "--trigger", "gate-ready", "--miss-safe-behavior", "land",
           "--dash-heading-deg", "25", "--standby-alt-m", "7.0", "--dash-speed", "6.0",
           "--engage-max-s", "30", "--yaw-tol-deg", "3.0", "--smoke-duration", "90",
           "--loop-watchdog-s", bound, "--log-csv", f"{SP}/sih_{how}_{bound}.csv"]
    with open(log_path, "w") as lf:
        prog = subprocess.Popen(cmd, cwd=WT, stdout=lf, stderr=subprocess.STDOUT)
    t_engage = None
    deadline = time.time() + 120
    while time.time() < deadline and t_engage is None:
        time.sleep(0.2)
        if "STANDBY -> ENGAGE" in open(log_path).read():
            t_engage = time.time()
    if t_engage is None:
        prog.kill()
        print("UNCERTAIN: never reached ENGAGE; see", log_path)
        return 2
    time.sleep(2.0)
    server = [p for p, c in children(prog.pid) if "mavsdk" in c]
    print(f"flight program pid {prog.pid}, mavsdk_server pids {server}, "
          f"PX4 mode before: {obs.cur}")
    t_event = time.time()
    if how == "stop":
        prog.send_signal(signal.SIGSTOP)
    elif how == "kill":
        prog.send_signal(signal.SIGKILL)
    elif how == "pilot":
        # A second MAVLink client takes the aircraft out of OFFBOARD. PX4 SITL
        # has no sticks, so it refuses Position mode; Return (AUTO.RTL) is the
        # mode it accepts, and it is equally "not this program's".
        obs.conn.mav.command_long_send(
            obs.conn.target_system or 2, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 5, 0, 0, 0, 0)
    watch_s = 8.0 if how != "pilot" else 14.0
    t_server_dead = None
    while time.time() - t_event < watch_s:
        if t_server_dead is None and server and not any(alive(p) for p in server):
            t_server_dead = time.time() - t_event
        time.sleep(0.02)
    print(f"event '{how}' at t=0 (watchdog bound {bound} s)")
    print(f"mavsdk_server dead after: "
          f"{'still ALIVE at +%.0f s' % watch_s if t_server_dead is None else '%.2f s' % t_server_dead}")
    print("PX4 flight mode as seen by an independent observer:")
    for t, mode, armed in obs.log:
        if t >= t_engage - 1.0:
            print(f"   {t - t_event:+7.2f} s  {mode:<14} armed={armed}")
    print(f"PX4 mode at +{watch_s:.0f} s: {obs.cur}")
    if how == "pilot":
        prog.wait(timeout=60)
        tail = open(log_path).read()
        print("flight program exit", prog.returncode)
        for ln in tail.splitlines():
            if "teardown" in ln or "no offboard stop" in ln or "landing" in ln \
                    or "offboard stopped" in ln or "FAILSAFE 7" in ln[:60]:
                print("   prog:", ln[:200])
        time.sleep(3.0)
        print(f"PX4 mode 3 s after the program exited: {obs.cur}")
    # cleanup: make sure the frozen program and its server are gone
    for p in [prog.pid] + server:
        try:
            os.kill(p, signal.SIGKILL)
        except ProcessLookupError:
            pass
    obs.stop = True
    return 0


if __name__ == "__main__":
    sys.exit(main())
