"""Download the newest dataflash log over MAVLink, then ALWAYS restore LOG_DISARMED=0
(read back). Progress goes to a file so nothing is lost if the caller is killed."""
import sys, time
from pymavlink import mavutil
port, out, prog = sys.argv[1], sys.argv[2], sys.argv[3]
def say(s):
    with open(prog, "a", encoding="utf-8") as f: f.write(f"{time.strftime('%H:%M:%S')} {s}\n")
m = mavutil.mavlink_connection(port, baud=115200)
if m.wait_heartbeat(timeout=25) is None: say("no heartbeat"); sys.exit(2)
def setp(name, val):
    for _ in range(3):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(val), 9)
        t = time.time() + 4
        while time.time() < t:
            x = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if x and x.param_id.strip("\x00").strip() == name:
                say(f"{name} read back = {x.param_value}"); return x.param_value
    say(f"!! {name} NOT confirmed"); return None
rc = 1
try:
    setp("LOG_DISARMED", 0)          # stop writing first: a growing log never finishes downloading
    time.sleep(3)
    m.mav.log_request_list_send(m.target_system, m.target_component, 0, 0xffff)
    logs = {}; end = time.time() + 25
    while time.time() < end:
        e = m.recv_match(type="LOG_ENTRY", blocking=True, timeout=2)
        if e is None: continue
        if e.size: logs[e.id] = e.size
        if e.num_logs and len(logs) >= e.num_logs: break
    if not logs: say("no logs listed"); sys.exit(1)
    lid = max(logs); size = logs[lid]; say(f"{len(logs)} logs; newest {lid} = {size} bytes")
    data = bytearray(size); got = 0; ofs = 0; stall = 0
    while ofs < size and stall < 60:
        m.mav.log_request_data_send(m.target_system, m.target_component, lid, ofs, 90 * 40)
        t = time.time() + 3; adv = False
        while time.time() < t:
            d = m.recv_match(type="LOG_DATA", blocking=True, timeout=1)
            if d is None or d.id != lid: continue
            if d.ofs == ofs and d.count:
                data[ofs:ofs + d.count] = bytes(d.data[:d.count]); ofs += d.count; adv = True
                if ofs >= size: break
        stall = 0 if adv else stall + 1
        if adv and (ofs // 90) % 500 == 0: say(f"{ofs}/{size}")
    open(out, "wb").write(bytes(data[:ofs]))
    say(f"wrote {out}: {ofs}/{size} bytes {'COMPLETE' if ofs >= size else 'PARTIAL'}")
    rc = 0 if ofs >= size else 1
finally:
    setp("LOG_DISARMED", 0)
    m.mav.log_request_end_send(m.target_system, m.target_component)
sys.exit(rc)
