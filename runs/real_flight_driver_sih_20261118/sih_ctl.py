"""Start / stop the scratch PX4 SIH instance 1 (no Gazebo). usage: sih_ctl.py restart|stop"""
import os
import signal
import subprocess
import sys
import tempfile
import time

SP = os.path.join(tempfile.gettempdir(), "real_flight_sih")
BUILD = os.path.expanduser(os.environ.get(
    "PX4_BUILD", "~/PX4-Autopilot/build/px4_sitl_default"))
WORK = SP + "/sih1"


def px4_pids():
    out = subprocess.run(["ps", "-eo", "pid=,comm=,args="], capture_output=True, text=True).stdout
    pids = []
    for line in out.splitlines():
        parts = line.split(None, 2)
        if len(parts) == 3 and parts[1] == "px4" and WORK in parts[2]:
            pids.append(int(parts[0]))
    return pids


def stop():
    for pid in px4_pids():
        os.kill(pid, signal.SIGKILL)
    time.sleep(1.0)
    # stray servers / frozen flight programs from the scratch runs
    out = subprocess.run(["ps", "-eo", "pid=,comm=,args="], capture_output=True, text=True).stdout
    for line in out.splitlines():
        parts = line.split(None, 2)
        if len(parts) == 3 and parts[1] == "mavsdk_server" and "14541" in parts[2]:
            os.kill(int(parts[0]), signal.SIGKILL)


def start():
    os.makedirs(WORK, exist_ok=True)
    env = dict(os.environ, PX4_SIM_MODEL="sihsim_quadx")
    with open(WORK + "/px4.log", "w") as lf:
        subprocess.Popen([BUILD + "/bin/px4", "-i", "1", "-d", BUILD + "/etc", "-w", WORK],
                         stdout=lf, stderr=subprocess.STDOUT, env=env,
                         start_new_session=True)
    for _ in range(100):
        time.sleep(0.2)
        if "Startup script returned successfully" in open(WORK + "/px4.log").read():
            print("px4 SIH instance 1 up, pids", px4_pids())
            return 0
    print("px4 did not come up")
    return 1


if __name__ == "__main__":
    what = sys.argv[1]
    stop()
    if what == "restart":
        sys.exit(start())
    print("stopped; remaining", px4_pids())
