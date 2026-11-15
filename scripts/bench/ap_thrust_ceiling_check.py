#!/usr/bin/env python3
"""PROPS-OFF bench check for an ARDUPILOT copter: with the throttle stick at
the bottom, how high can the board still hold the motors?

Why: on 2026-11-14 the target drone climbed to 23 m at minimum throttle. The
board is allowed to raise average thrust to keep roll/pitch control, up to a
ceiling of ATC_THR_MIX_MAX x MOT_THST_HOVER, and that ceiling sat above the
drone's real hover point (ADR-0138 addenda 6 and 7). This reads the motor
signals the board sends while you provoke exactly that case on the bench.

    python.exe ap_thrust_ceiling_check.py COM4 [hover_us] [result.json]

THIS SCRIPT ONLY LISTENS. It never arms, never sends a motor command and
never changes a setting. YOU arm with the transmitter, PROPS OFF:
  1. Loiter (mode switch up; needs a GPS fix, so do it outdoors). Arm.
     (Stabilize has a much lower ceiling, so it does not test this.)
  2. Raise the throttle stick above the middle for 2 seconds (the board now
     thinks it has taken off).
  3. Pick the drone up and keep tilting and twisting it (the board fights you).
  4. Drop the stick to the bottom and KEEP tilting for 3 seconds.
  5. Repeat 2 to 4 twice more, then disarm.

Verdict (written before the first run):
  PASS          every low-stick reading taken while the board was fighting has
                a four-motor average below hover_us (default 1216, measured in
                the 2026-11-14 evening flight).
  FAIL          any such reading at or above hover_us.
  NOT EXERCISED no low-stick reading where the board was fighting (the motors
                differed by less than 30 us, or the stick was never raised
                first). That is NOT a pass: repeat the steps.
"""
import json
import sys
import time

SPREAD_MIN_US = 30        # motors must differ by this much to count as "fighting"
STICK_LOW_US = 1050
STICK_UP_US = 1500
AFTER_DROP_S = 3.0        # only readings this soon after the stick came down count


def verdict(samples: list, hover_us: float) -> dict:
    """samples: (seconds since the stick dropped, mean us, spread us) for
    armed, stick-low readings taken after a raise."""
    used = [s for s in samples if s[0] <= AFTER_DROP_S and s[2] >= SPREAD_MIN_US]
    if not used:
        return {"verdict": "NOT EXERCISED", "readings_used": 0}
    worst = max(s[1] for s in used)
    return {"verdict": "PASS" if worst < hover_us else "FAIL", "readings_used": len(used),
            "highest_mean_us": round(worst, 1), "hover_us": hover_us,
            "margin_us": round(hover_us - worst, 1)}


def run(port: str, hover_us: float, result_path: str, listen_s: float = 300.0) -> int:
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(port, baud=115200)
    if m.wait_heartbeat(timeout=15) is None:
        raise SystemExit(f"FAIL: no heartbeat on {port}")
    for msg_id in (36, 65):                       # SERVO_OUTPUT_RAW, RC_CHANNELS at 20 Hz
        m.mav.command_long_send(m.target_system, m.target_component, 511, 0, msg_id, 50000, 0, 0, 0, 0, 0)
    print(__doc__.split("THIS SCRIPT ONLY LISTENS.")[1].split("Verdict")[0], flush=True)
    print(f"Listening for {listen_s:.0f} s. Hover point used: {hover_us:.0f} us.", flush=True)
    armed = was_armed = False
    stick, raised_since, raised_ok, dropped_at = 0, None, False, None
    samples, t0 = [], time.time()
    while time.time() - t0 < listen_s:
        msg = m.recv_match(type=["HEARTBEAT", "RC_CHANNELS", "SERVO_OUTPUT_RAW"], blocking=True, timeout=1)
        if msg is None:
            continue
        now, kind = time.time(), msg.get_type()
        if kind == "HEARTBEAT" and msg.get_srcComponent() == 1:
            armed = bool(msg.base_mode & 128)
            if was_armed and not armed and samples:
                break
            was_armed = was_armed or armed
        elif kind == "RC_CHANNELS":
            stick = msg.chan3_raw
            if stick > STICK_UP_US:
                raised_since = raised_since or now
                raised_ok = raised_ok or now - raised_since >= 1.0
                dropped_at = None
            else:
                raised_since = None
                if stick < STICK_LOW_US and raised_ok and dropped_at is None:
                    dropped_at = now
                    raised_ok = False
        elif kind == "SERVO_OUTPUT_RAW" and armed and dropped_at and stick < STICK_LOW_US:
            outs = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
            mean, spread = sum(outs) / 4.0, max(outs) - min(outs)
            samples.append((now - dropped_at, mean, spread))
            if spread >= SPREAD_MIN_US and now - dropped_at <= AFTER_DROP_S:
                print(f"  stick low, board fighting: motors average {mean:.0f} us (spread {spread})", flush=True)
    result = verdict(samples, hover_us)
    result.update({"port": port, "when": time.strftime("%Y-%m-%d %H:%M:%S"), "low_stick_readings": len(samples)})
    with open(result_path, "w") as fh:
        json.dump(result, fh, indent=1)
    print("\nRESULT:", json.dumps(result), flush=True)
    return {"PASS": 0, "FAIL": 1}.get(result["verdict"], 3)


def self_test() -> int:
    assert verdict([], 1235)["verdict"] == "NOT EXERCISED"
    assert verdict([(0.5, 1300, 5)], 1235)["verdict"] == "NOT EXERCISED"      # not fighting
    assert verdict([(5.0, 1300, 80)], 1235)["verdict"] == "NOT EXERCISED"     # too late after the drop
    assert verdict([(0.5, 1195, 80), (1.0, 1201, 60)], 1235)["verdict"] == "PASS"
    crash = verdict([(0.5, 1302, 150)], 1235)                                 # the 2026-11-14 flight
    assert crash["verdict"] == "FAIL" and crash["margin_us"] < 0
    print("self-test passed: 5 cases")
    return 0


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--self-test":
        sys.exit(self_test())
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    code = 3
    try:
        code = run(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 1216.0,
                   sys.argv[3] if len(sys.argv) > 3 else "thrust_ceiling_result.json")
    finally:
        print("\nThis window can be closed.", flush=True)
        time.sleep(600)
    sys.exit(code)
