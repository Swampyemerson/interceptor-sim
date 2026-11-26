"""Read-only: download onboard logs from the target autopilot. Sends only heartbeats, LOG_REQUEST_DATA and LOG_REQUEST_END."""
import sys, time, os
from pymavlink import mavutil
port, outdir, ids = sys.argv[1], sys.argv[2], [int(x) for x in sys.argv[3:]]
os.makedirs(outdir, exist_ok=True)
m = mavutil.mavlink_connection(port, baud=115200)
assert m.wait_heartbeat(timeout=12), "no heartbeat"
assert not m.motors_armed(), "armed: not downloading"
m.mav.log_request_list_send(m.target_system, m.target_component, 0, 0xFFFF)
sizes = {}; t0 = time.time()
while time.time() - t0 < 6:
    x = m.recv_match(type="LOG_ENTRY", blocking=True, timeout=1)
    if x: sizes[x.id] = x.size
for i in ids:
    size = sizes[i]; buf = bytearray(size); got = bytearray(size // 90 + 1)
    t0 = time.time(); last_print = t0
    def missing():
        out = []; start = None
        for k in range(len(got)):
            if k * 90 >= size: break
            if not got[k] and start is None: start = k
            if got[k] and start is not None: out.append((start, k)); start = None
        if start is not None: out.append((start, (size + 89) // 90))
        return out
    m.mav.log_request_data_send(m.target_system, m.target_component, i, 0, size)
    last_rx = time.time(); rounds = 0
    while True:
        x = m.recv_match(type="LOG_DATA", blocking=True, timeout=0.5)
        now = time.time()
        if x and x.id == i and x.count > 0:
            buf[x.ofs:x.ofs + x.count] = bytes(x.data[:x.count]); got[x.ofs // 90] = 1; last_rx = now
        if now - last_print > 20:
            done = sum(got) * 90
            print(f"log {i}: {min(done, size) / 1e6:.2f} of {size / 1e6:.2f} MB, {done / 1e3 / (now - t0):.0f} kB/s", flush=True); last_print = now
        if now - last_rx > 1.0:
            miss = missing()
            if not miss: break
            rounds += 1
            if rounds > 400: print("giving up with holes", len(miss)); break
            a, b = miss[0]
            m.mav.log_request_data_send(m.target_system, m.target_component, i, a * 90, min((b - a) * 90, size - a * 90))
            last_rx = now
    m.mav.log_request_end_send(m.target_system, m.target_component)
    holes = len(missing())
    name = os.path.join(outdir, f"target_log{i:02d}.bin")
    open(name, "wb").write(buf)
    print(f"log {i}: saved {name} {size} bytes in {time.time() - t0:.0f} s, holes {holes}", flush=True)
