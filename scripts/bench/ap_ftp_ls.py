"""Read-only: list directories on an ArduPilot board's card over MAVLink FTP. Writes nothing."""
import sys, time
from pymavlink import mavutil, mavftp
m = mavutil.mavlink_connection(sys.argv[1], baud=115200); assert m.wait_heartbeat(timeout=20)
f = mavftp.MAVFTP(m, target_system=m.target_system, target_component=m.target_component)
for d in sys.argv[2:]:
    print("==", d, flush=True)
    try:
        r = f.cmd_list([d]); 
        res = f.process_ftp_reply("ListDirectory", timeout=15)
        print("  result:", getattr(res, "error_code", res), getattr(res, "system_error", ""), flush=True)
        for e in getattr(f, "list_result", []) or []:
            print("  ", "DIR " if e.is_dir else "file", e.name, "" if e.is_dir else e.size_b)
    except Exception as ex:
        print("  exception:", repr(ex)[:200])
