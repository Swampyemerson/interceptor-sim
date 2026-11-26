"""Print the timeline of one run (truth-side scoring): range, height difference,
where the tag sits in the picture, decodes, the terminal's phase, the command.
Usage: .venv/bin/python runs/handover_profile_isim_20261126/trace_one.py TARGET V_T TAG CAP SEED [DH]"""
import os, sys, math, contextlib, io
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, "scripts"))
import numpy as np
import handover_profile_isim as h
from isim.seeker import quat_to_rot

kind, v_t, tag, cap, seed = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), int(sys.argv[5])
job = dict(cell="trace", seed=seed, target=kind, v_t=v_t, tag_m=tag, cap=cap)
if len(sys.argv) > 6:
    job["dh"] = float(sys.argv[6])
log = []
orig = h.HandoverGuidance.step
def step(self, t, own, det):
    cmd = orig(self, t, own, det)
    d = self.sm
    tel = getattr(self, "_last_tel", None)
    log.append((t, d.state, det is not None, cmd.v_north, cmd.v_east, cmd.v_down,
                getattr(d.guidance, "_phase", "?"), getattr(d.guidance, "rehearsal_trigger_t", None)))
    return cmd
h.HandoverGuidance.step = step
_orig_run = h.run_engagement
keep = {}
def cap_run(*a, **k):
    keep["res"] = _orig_run(*a, **k); return keep["res"]
h.run_engagement = cap_run
row = h.run_one(job)
tr = keep["res"].trace
r_bc = h.CameraParams(width=h.W, height=h.H, fx=h.FX, fy=h.FX, cx=h.W/2, cy=h.H/2, mount_tilt_up_deg=h.TILT_TOLD).rot_body_from_cam()
print("d0 %.1f dh %+.1f cue_err %.1f cue_h_err %+.1f" % (row["d0"], row["dh"], row["cue_err_m"], row["cue_h_err_m"]))
print("reasons:", row["reasons"])
print(f"{'t':>6} {'state':>8} {'ph':>2} {'range':>6} {'own-tgt h':>9} {'v_px':>6} {'side_px':>7} {'dec/s':>5} {'cmd_h':>5} {'cmd_vd':>6} {'pitch':>6}")
T = tr["t"]
for t0 in np.arange(h.GO_AT, min(T[-1], row["t_min_sep"] + 6.0), 0.5):
    j = int(np.searchsorted(T, t0))
    rel = tr["tgt_pos"][j] - tr["own_pos"][j]
    vc = r_bc.T @ (quat_to_rot(tuple(tr["quat"][j])).T @ rel)
    v = h.FX * vc[1] / vc[2] + h.H / 2 if vc[2] > 0.05 else float("nan")
    q = tr["quat"][j]; pitch = math.degrees(math.asin(max(-1, min(1, 2 * (q[0] * q[2] - q[3] * q[1])))))
    ll = [x for x in log if t0 - 0.5 < x[0] <= t0]
    nd = sum(x[2] for x in ll) * 2
    last = ll[-1] if ll else (0, "?", 0, 0, 0, 0, "?", None)
    print(f"{t0:6.1f} {last[1]:>8} {last[6]:>2} {np.linalg.norm(rel):6.2f} {-(tr['own_pos'][j][2] - tr['tgt_pos'][j][2]):9.2f} "
          f"{v:6.0f} {h.FX * tag / max(np.linalg.norm(rel), 1e-6):7.0f} {nd:5d} {math.hypot(last[3], last[4]):5.2f} {last[5]:6.2f} {pitch:6.1f}")
