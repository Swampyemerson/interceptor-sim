"""--fly end to end against REAL PX4 (SIH, no Gazebo) and a REAL mavsdk_server.
A second MAVLink client plays the transmitter: RC_CHANNELS_OVERRIDE with the GO
switch on ch7 and the arm-permit switch on ch8. The camera is the synthetic box
generator handed in as the source (there is no camera on this laptop)."""
import os
import sys
import tempfile
import threading
import time

from pymavlink import mavutil

WT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SP = os.path.join(tempfile.gettempdir(), "real_flight_sih")
os.makedirs(SP, exist_ok=True)
sys.path.insert(0, WT)
import flight.deploy.real_flight as rf  # noqa: E402

PERMIT_AT, GO_AT = float(sys.argv[1]), float(sys.argv[2])
EXTRA = sys.argv[3:]
T0 = time.time()
events = []


def transmitter():
    tx = mavutil.mavlink_connection("udpout:127.0.0.1:18571", source_system=255)
    said = set()
    while True:
        t = time.time() - T0
        permit = 2000 if (PERMIT_AT >= 0 and t >= PERMIT_AT) else 1000
        go = 2000 if (GO_AT >= 0 and t >= GO_AT) else 1000
        for name, on in (("permit", permit == 2000), ("go", go == 2000)):
            if on and name not in said:
                said.add(name)
                events.append((t, f"transmitter: {name} switch HIGH"))
        tx.mav.rc_channels_override_send(2, 1, 1500, 1500, 1500, 1500, 1000, 1000,
                                         go, permit)
        tx.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                              mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.1)


class Tee:
    def __init__(self, real):
        self.real, self.buf = real, ""

    def write(self, s):
        self.real.write(s)
        self.buf += s
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            for key in ("[arm-permit]", "arm; takeoff", "GO edge", "TERMINATED",
                        "[limits] REFUSED", "[limits] OVERRIDDEN", "NOT ARMING",
                        "landing;", "[watchdog] loop"):
                if key in line:
                    events.append((time.time() - T0, "program: " + line.strip()[:150]))

    def flush(self):
        self.real.flush()

    def isatty(self):
        return False


def main():
    threading.Thread(target=transmitter, daemon=True).start()
    from flight.deploy.seeker_loop import SmokeSeeker
    rf.build_source_detector = lambda args, gcfg, cam: (
        SmokeSeeker(cam.fx, 0.30, cx=cam.cx, cy=cam.cy), None)
    sys.stdout = Tee(sys.__stdout__)
    argv = ["--fly", "--mavsdk-url", "udpin://0.0.0.0:14541", "--trigger", "rc",
            "--rc-channel", "7", "--arm-permit-channel", "8", "--source", "picamera",
            "--target-span-m", "0.30", "--target-start", "0,16.7", "--target-vel", "9,0",
            "--manual-belief", "--miss-safe-behavior", "land", "--engage-max-s", "8",
            "--dash-heading-deg", "25", "--yaw-tol-deg", "3.0",
            "--log-csv", SP + "/sih_fly.csv"] + EXTRA
    try:
        rc = rf.main(argv)
    except SystemExit as e:
        rc = e.code
    sys.stdout = sys.__stdout__
    print("\n==== timeline (wall seconds) ====")
    for t, what in sorted(events):
        print(f"  {t:6.1f}  {what}")
    print("EXIT", rc)


if __name__ == "__main__":
    main()
