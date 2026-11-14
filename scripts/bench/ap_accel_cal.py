#!/usr/bin/env python3
"""Six-position accelerometer calibration for an ARDUPILOT board, in a console
window, for when a ground station's own calibration page gets stuck.

It asks the board to start its calibration, then for each position the board
asks for it tells you how to hold the drone and waits for Enter. It answers the
board with MAV_CMD_ACCELCAL_VEHICLE_POS, the method Mission Planner uses and
the one ArduPilot's source recommends (AP_AccelCal.cpp, gcs_vehicle_position).
When the board reports success it reboots the board (ArduPilot wants a reboot
after this calibration), reads the new offsets back and writes them to a
result file.

    python.exe ap_accel_cal.py COM4 [result.json]

PROPS OFF. The board must be disarmed; this refuses to start otherwise. It
never arms and never runs a motor. ArduPilot only (the target's Kakute), not
the interceptor's PX4 Pixhawk.
"""
import json
import subprocess
import sys
import time

CMD_PREFLIGHT_CALIBRATION = 241
CMD_REBOOT = 246
CMD_ACCELCAL_VEHICLE_POS = 42429
POS_SUCCESS, POS_FAILED = 16777215, 16777216
POSITIONS = {
    1: "LEVEL, the way it sits on the ground",
    2: "on its LEFT side (left side down, right side pointing at the ceiling)",
    3: "on its RIGHT side (right side down)",
    4: "NOSE DOWN (the real nose pointing at the floor)",
    5: "NOSE UP (the real nose pointing at the ceiling)",
    6: "on its BACK (upside down)",
}
OFFSET_PARAMS = ["INS_ACCOFFS_X", "INS_ACCOFFS_Y", "INS_ACCOFFS_Z",
                 "INS_ACCSCAL_X", "INS_ACCSCAL_Y", "INS_ACCSCAL_Z"]


def say(text: str) -> None:
    """Print, and also speak through the PC's speakers when that is available."""
    print("\n>>> " + text, flush=True)
    try:
        subprocess.Popen(["powershell", "-NoProfile", "-Command",
                          "Add-Type -AssemblyName System.Speech; "
                          "(New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak('%s')"
                          % text.replace("'", "")],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError:
        pass


def connect(port: str, wait_s: float):
    """Open the port, waiting while another program (QGroundControl) holds it."""
    from pymavlink import mavutil
    t0, told = time.time(), False
    while True:
        try:
            m = mavutil.mavlink_connection(port, baud=115200)
            if m.wait_heartbeat(timeout=10) is not None:
                return m
            m.close()
        except Exception as e:                       # port busy or absent
            if not told:
                say("Close QGroundControl so this window can use the drone.")
                print(f"    (waiting for {port}: {e})", flush=True)
                told = True
        if time.time() - t0 > wait_s:
            raise SystemExit(f"FAIL: could not open {port} in {wait_s:.0f} s")
        time.sleep(2)


def read_params(m, names: list) -> dict:
    out = {}
    for _ in range(3):
        for n in names:
            if n not in out:
                m.mav.param_request_read_send(m.target_system, m.target_component, n.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < 3 and len(out) < len(names):
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg is not None and msg.param_id in names:
                out[msg.param_id] = round(msg.param_value, 5)
        if len(out) == len(names):
            break
    return out


def calibrated(p: dict) -> bool:
    """ArduPilot's own test: a board that has never been calibrated has all
    three offsets at exactly zero."""
    return len(p) == len(OFFSET_PARAMS) and any(p[k] != 0.0 for k in OFFSET_PARAMS[:3])


def run(port: str, result_path: str) -> int:
    result = {"port": port, "started": time.strftime("%Y-%m-%d %H:%M:%S"), "outcome": "not finished",
              "positions_done": [], "messages": []}

    def save():
        with open(result_path, "w") as fh:
            json.dump(result, fh, indent=1)

    save()
    m = connect(port, 900)
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    if hb is None or hb.autopilot != 3:
        result["outcome"] = "not an ArduPilot board"
        save()
        raise SystemExit("FAIL: this is not an ArduPilot board")
    if hb.base_mode & 128:
        result["outcome"] = "refused: armed"
        save()
        raise SystemExit("FAIL: the drone is ARMED. Disarm it first.")
    result["before"] = read_params(m, OFFSET_PARAMS)
    print("offsets before:", result["before"], flush=True)
    say("Accelerometer calibration. Props off. Six positions. Hold the drone still each time.")
    m.mav.command_long_send(m.target_system, m.target_component, CMD_PREFLIGHT_CALIBRATION, 0,
                            0, 0, 0, 0, 1, 0, 0)
    done, answered_at, t_last = set(), {}, time.time()
    while True:
        msg = m.recv_match(type=["COMMAND_LONG", "STATUSTEXT", "COMMAND_ACK"], blocking=True, timeout=1)
        if msg is None:
            if time.time() - t_last > 60:
                result["outcome"] = "board went quiet for 60 s"
                save()
                say("The board stopped answering. Calibration not finished.")
                return 2
            continue
        t = msg.get_type()
        if t == "STATUSTEXT":
            t_last = time.time()
            result["messages"].append(msg.text)
            print("    board says:", msg.text, flush=True)
            if "alibration" in msg.text and ("FAILED" in msg.text.upper()):
                result["outcome"] = "board reported failure: " + msg.text
                save()
                say("The board says the calibration failed. Run it again and hold stiller.")
                return 1
            continue
        if t == "COMMAND_ACK":
            if msg.command == CMD_PREFLIGHT_CALIBRATION and msg.result not in (0, 5):
                result["outcome"] = f"board refused to start (result {msg.result})"
                save()
                say("The board refused to start the calibration.")
                return 1
            continue
        if msg.command != CMD_ACCELCAL_VEHICLE_POS:
            continue
        t_last = time.time()
        pos = int(msg.param1)
        if pos == POS_SUCCESS:
            break
        if pos == POS_FAILED:
            result["outcome"] = "board reported failure"
            save()
            say("The board says the calibration failed. Run it again and hold stiller.")
            return 1
        if pos not in POSITIONS:
            continue
        # The board repeats its request about once a second until it has its
        # sample. A repeat soon after our answer is that; a repeat long after
        # means the answer did not take, so ask again.
        if pos in answered_at and time.time() - answered_at[pos] < 8.0:
            continue
        if pos in answered_at:
            say(f"The board did not take position {pos}. Same position again.")
        say(f"Position {pos} of 6. Put the drone {POSITIONS[pos]}. Hold it still, then press Enter.")
        input("    press Enter when it is still... ")
        # The board keeps repeating its request while we wait; drop the backlog.
        while m.recv_match(blocking=False) is not None:
            pass
        m.mav.command_long_send(m.target_system, m.target_component, CMD_ACCELCAL_VEHICLE_POS, 0,
                                pos, 0, 0, 0, 0, 0, 0)
        done.add(pos)
        answered_at[pos] = time.time()
        result["positions_done"] = sorted(done)
        save()
        print("    measuring, keep it still for 3 seconds...", flush=True)
        time.sleep(3.0)
        t_last = time.time()

    say("The board accepted the calibration. Put the drone down. Restarting the board.")
    result["after_before_reboot"] = read_params(m, OFFSET_PARAMS)
    m.mav.command_long_send(m.target_system, m.target_component, CMD_REBOOT, 0, 1, 0, 0, 0, 0, 0, 0)
    m.close()
    time.sleep(12)
    m = connect(port, 60)
    result["after_reboot"] = read_params(m, OFFSET_PARAMS)
    ok = calibrated(result["after_reboot"]) and result["after_reboot"] == result["after_before_reboot"]
    result["outcome"] = "CALIBRATED" if ok else "board said success but the offsets did not persist"
    save()
    print("offsets after reboot:", result["after_reboot"], flush=True)
    say("Accelerometer calibration done and saved." if ok else
        "Something is wrong: the calibration did not stay saved.")
    return 0 if ok else 1


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    rc = 1
    try:
        rc = run(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "accelcal_result.json")
    finally:
        print("\nThis window can be closed.", flush=True)
        time.sleep(600 if rc == 0 else 1800)
    sys.exit(rc)
