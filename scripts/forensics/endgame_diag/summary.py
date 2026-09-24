#!/usr/bin/env python3
import json, math, os
import numpy as np

OD = "/home/emerson/interceptor-sim/logs/endgame_diag_20260924"
gz = json.load(open(os.path.join(OD, "gz_raw.json")))
im = json.load(open(os.path.join(OD, "isim_raw.json")))
BINS = [(">8", 8, 1e9), ("4-8", 4, 8), ("2-4", 2, 4), ("<2", 0, 2)]
lines = []


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    lines.append(s)


def fin(a):
    a = np.asarray([x for x in a if x is not None and not (isinstance(x, float) and math.isnan(x))], float)
    return a


def mstat(a):
    a = fin(a)
    if a.size == 0:
        return "UNCERTAIN(n=0)"
    return f"med {np.median(a):+.3f} [IQR {np.percentile(a,25):+.2f},{np.percentile(a,75):+.2f}] n={a.size}"


def med(a):
    a = fin(a)
    return float(np.median(a)) if a.size else math.nan


def spread(a):
    a = fin(a)
    return float((np.percentile(a, 84) - np.percentile(a, 16)) / 2) if a.size >= 3 else math.nan


groups = {
    "GZ brake": ("gz", lambda r: r["arm"] == "brake"),
    "GZ nobrake": ("gz", lambda r: r["arm"] == "nobrake"),
    "GZ pooled": ("gz", lambda r: True),
    "isim brake": ("im", lambda r: r["arm"] == "brake"),
    "isim nobrake": ("im", lambda r: r["arm"] == "nobrake"),
    "isim pooled": ("im", lambda r: r["arm"] in ("brake", "nobrake")),
    "isim brake NOVEL(cf)": ("im", lambda r: r["arm"] == "brake_novel"),
    "isim nobrake NOVEL(cf)": ("im", lambda r: r["arm"] == "nobrake_novel"),
}


def sel(src, f, kind):
    d = gz if src == "gz" else im
    return [r for r in d[kind] if f(r)]


P("=== T0 headline: KF range error e = r_hat - r_true (per-flight medians; stat = median across flights) ===")
for g, (src, f) in groups.items():
    fl = sel(src, f, "flights")
    P(f"{g:24s} flights={len(fl):2d} | last2 {mstat([r.get('e_last2_med') for r in fl])}"
      f" | pre-CPA2 {mstat([r.get('e_pre2_med') for r in fl])}"
      f" | CPA {mstat([r.get('cpa') for r in fl])}")
P("")
P("window geometry (median across flights): CPA-before-phaseB-end [s], dets in last2, dets in pre2, last det before CPA [s]")
for g, (src, f) in groups.items():
    fl = sel(src, f, "flights")
    P(f"{g:24s} {med([r.get('cpa_before_end_s') for r in fl]):+.2f}  "
      f"{med([r.get('n_det_last2') for r in fl]):.1f}  {med([r.get('n_det_pre2') for r in fl]):.1f}  "
      f"{med([r.get('last_det_before_cpa_s') for r in fl]):.2f}  "
      f"(flights with 0 dets in last2: {sum(1 for r in fl if r.get('n_det_last2', 0) == 0)}/{len(fl)})")

P("")
P("=== Q1a: post-update residual e at each consumed update, by TRUE range bin: median / half(p84-p16) / n ===")
P("           also (d) pose-range error vs truth at capture (camera->tag) and (b) latency error (age-0.045)*|rdot| as range error")
for g, (src, f) in groups.items():
    ds = sel(src, f, "dets")
    for lab, lo, hi in BINS:
        s = [d for d in ds if lo <= d["r_true"] < hi]
        if not s:
            P(f"{g:24s} {lab:>4s}: UNCERTAIN (n=0)")
            continue
        ep = [d["e_post"] for d in s]
        pe = [d["pose_err_cam"] for d in s]
        le = [d["lat_err_m"] for d in s]
        ag = [d["age"] for d in s]
        P(f"{g:24s} {lab:>4s}: e_post {med(ep):+.3f}/{spread(ep):.3f}  pose_err {med(pe):+.3f}/{spread(pe):.3f}"
          f"  lat_err {med(le):+.3f}/{spread(le):.3f}  age {med(ag):.3f}s  n={len(s)}")

P("")
P("=== Q1b: coast error growth, slope of (e - e_at_update) vs time since update [m/s], per coast interval ===")
for g, (src, f) in groups.items():
    iv = sel(src, f, "intervals")
    for lab, filt in (("all", lambda x: True), ("pre-CPA2", lambda x: x["in_pre2"]),
                      ("last2", lambda x: x["in_last2"]), ("dur>=0.3s", lambda x: x["dur"] >= 0.3)):
        s = [x for x in iv if filt(x)]
        extra = ""
        if src == "gz" and s:
            extra = (f" | own-input part {mstat([x['slope_own'] for x in s])}"
                     f" | remainder {mstat([x['slope_rest'] for x in s])}")
        if src == "im" and s:
            extra = f" | vector |E| slope {mstat([x['slopeE'] for x in s])}"
        P(f"{g:24s} {lab:9s}: scalar {mstat([x['slope'] for x in s])}{extra}")
P("final coast (last consumed det -> phase-B end), slope of VECTOR error |r_hat_vec - r_true_vec| [m/s]:")
for g, (src, f) in groups.items():
    fl = sel(src, f, "flights")
    if src == "gz":
        for rep in ("R0_cmd", "R1_ekf", "R2_truevel", "R3_truevel_perfmeas", "R4_cmd_perfmeas"):
            P(f"{g:24s} replica {rep:22s} {mstat([r.get(rep + '_final_coast_vecE_slope') for r in fl])}"
              f"  dur {med([r.get(rep + '_final_coast_dur') for r in fl]):.2f}s")
    else:
        P(f"{g:24s} logged                  {mstat([r.get('final_coast_vecE_slope') for r in fl])}"
          f"  dur {med([r.get('final_coast_dur') for r in fl]):.2f}s")

P("")
P("=== Q1c: implied relative-velocity error near CPA (pre-CPA 2 s window; per-flight medians) ===")
for g, (src, f) in groups.items():
    fl = sel(src, f, "flights")
    if src == "gz":
        P(f"{g:24s} implied d|r_hat|/dt - d|r|/dt (non-update ticks) {mstat([r.get('rdot_err_med_pre2') for r in fl])}")
        P(f"{'':24s}   own-input LOS part -u.(v_cmd_prev - v_true) {mstat([r.get('own_rdot_med_pre2') for r in fl])}")
        P(f"{'':24s}   replica R0 vector |v_rel_hat - v_rel| {mstat([r.get('R0_vrel_err_pre2_med') for r in fl])}"
          f"  (v_t part {mstat([r.get('R0_vt_part_pre2_med') for r in fl])}; own-input part {mstat([r.get('R0_own_part_pre2_med') for r in fl])})")
        P(f"{'':24s}   last2: implied rdot err {mstat([r.get('rdot_err_med_last2') for r in fl])}; own-input LOS part {mstat([r.get('own_rdot_med_last2') for r in fl])}")
    else:
        P(f"{g:24s} implied rdot err {mstat([r.get('rdot_err_med_pre2') for r in fl])}"
          f" | vector v_rel err {mstat([r.get('vrel_err_pre2_med') for r in fl])}"
          f" | v_t err {mstat([r.get('vt_err_pre2_med') for r in fl])} | last2 rdot err {mstat([r.get('rdot_err_med_last2') for r in fl])}")

P("")
P("=== (c) own velocity: registered EKF-vs-gz, and the input the KF ACTUALLY used (last commanded velocity) ===")
for g, (src, f) in groups.items():
    fl = sel(src, f, "flights")
    if src == "gz":
        P(f"{g:24s} PX4 EKF |v_h err| ENGAGE {mstat([r.get('ekf_err_h_med_eng') for r in fl])}; p90 {mstat([r.get('ekf_err_h_p90_eng') for r in fl])}; pre2 {mstat([r.get('ekf_err_h_med_pre2') for r in fl])}")
        P(f"{'':24s} KF own-vel INPUT (cmd_prev) |v_h err| ENGAGE {mstat([r.get('vin_err_h_med_eng') for r in fl])}; pre2 {mstat([r.get('vin_err_h_med_pre2') for r in fl])}; LOS pre2 {mstat([r.get('vin_err_los_med_pre2') for r in fl])}")
    else:
        P(f"{g:24s} KF own-vel INPUT |v err| phaseB {mstat([r.get('vin_err_eng_med') for r in fl])}")

P("")
P("=== Replica attribution (Gazebo, KF re-run offline on the logged tick stream; open-loop on the flown trajectory) ===")
P("fidelity: replica R0 (as flown: own_vel = last cmd, logged pose ranges, true capture times) vs logged r_hat, median |diff|")
for g in ("GZ brake", "GZ nobrake"):
    src, f = groups[g]
    fl = sel(src, f, "flights")
    P(f"{g:12s} all-phaseB {mstat([r.get('R0_cmd_fid_med_abs_all') for r in fl])}  last2 {mstat([r.get('R0_cmd_fid_med_abs_last2') for r in fl])}")
REPS = [("logged", "e_last2"), ("R0_cmd", "R0_cmd_e_last2"), ("R4_cmd_perfmeas", "R4_cmd_perfmeas_e_last2"),
        ("R1_ekf", "R1_ekf_e_last2"), ("R2_truevel", "R2_truevel_e_last2"),
        ("R5_truevel_poseonly", "R5_truevel_poseonly_e_last2"), ("R6_truevel_latonly", "R6_truevel_latonly_e_last2"),
        ("R3_truevel_perfmeas", "R3_truevel_perfmeas_e_last2")]
for g in ("GZ brake", "GZ nobrake", "GZ pooled"):
    src, f = groups[g]
    fl = sel(src, f, "flights")
    P(f"-- {g} (n={len(fl)} flights): last2 e median-of-medians | mean-of-means | pre2 median | pre2 vector |E|")
    for lab, key in REPS:
        pre = key.replace("e_last2", "e_pre2")
        vec = key.replace("e_last2", "vec_err_pre2") if lab != "logged" else None
        P(f"   {lab:22s} {med([r.get(key + '_med') for r in fl]):+.3f} | {np.nanmean(fin([r.get(key + '_mean') for r in fl])):+.3f}"
          f" | {med([r.get(pre + '_med') for r in fl]):+.3f} | "
          + (f"{med([r.get(vec + '_med') for r in fl]):.3f}" if vec else "n/a"))
    # shares on mean-of-means of last2 e (additive), order: own-input -> EKF-vs-true -> meas(b,d) -> residual(a)
    mm = {lab: float(np.nanmean(fin([r.get(key + "_mean") for r in fl]))) for lab, key in REPS}
    tot = mm["R0_cmd"]
    P(f"   SHARES of R0 last2 mean error {tot:+.3f} m:")
    P(f"     (c') own-vel input = last COMMAND vs EKF velocity (R0 -> R1): {mm['R0_cmd']-mm['R1_ekf']:+.3f} m = {100*(mm['R0_cmd']-mm['R1_ekf'])/tot:.0f}%")
    P(f"     (c)  EKF velocity error vs gz truth        (R1 -> R2): {mm['R1_ekf']-mm['R2_truevel']:+.3f} m = {100*(mm['R1_ekf']-mm['R2_truevel'])/tot:.0f}%")
    P(f"     (b)+(d) measurement (pose err + true age)   (R2 -> R3): {mm['R2_truevel']-mm['R3_truevel_perfmeas']:+.3f} m = {100*(mm['R2_truevel']-mm['R3_truevel_perfmeas'])/tot:.0f}%"
      f"   [d alone R5-R3: {mm['R5_truevel_poseonly']-mm['R3_truevel_perfmeas']:+.3f}; b alone R6-R3: {mm['R6_truevel_latonly']-mm['R3_truevel_perfmeas']:+.3f}]")
    P(f"     (a) residual with perfect inputs: v_t-estimate x coast (R3): {mm['R3_truevel_perfmeas']:+.3f} m = {100*mm['R3_truevel_perfmeas']/tot:.0f}%")
    P(f"     alt order: meas first (R0 -> R4) {mm['R0_cmd']-mm['R4_cmd_perfmeas']:+.3f}; then own-input (R4 -> R3) {mm['R4_cmd_perfmeas']-mm['R3_truevel_perfmeas']:+.3f}")
    P(f"     replica-vs-logged gap (logged - R0): {mm['logged']-mm['R0_cmd']:+.3f} m (bearing/attitude errors the replica omits)")

P("")
P("=== Coast LATCH (r_hat < hold_range 1.0 m freezes the command) ===")
for g in ("GZ brake", "GZ nobrake"):
    src, f = groups[g]
    fl = sel(src, f, "flights")
    P(f"{g:12s} logged latch: t-CPA {mstat([r.get('latch_t_minus_cpa') for r in fl])}; TRUE range at latch {mstat([r.get('latch_rtrue') for r in fl])}; true r_perp {mstat([r.get('latch_rperp_true') for r in fl])}")
    for rep in ("R0_cmd", "R1_ekf", "R2_truevel"):
        vals = [r.get(f"latch_{rep}_rtrue_at") for r in fl]
        nl = sum(1 for v in vals if v is not None and not math.isnan(v))
        P(f"{'':12s} replica {rep:10s} would latch in {nl}/{len(fl)}; true range then {mstat(vals)}; t-CPA {mstat([r.get(f'latch_{rep}_t_minus_cpa') for r in fl])}")
for g in ("isim brake", "isim nobrake", "isim brake NOVEL(cf)", "isim nobrake NOVEL(cf)"):
    src, f = groups[g]
    fl = sel(src, f, "flights")
    P(f"{g:24s} latch t-CPA {mstat([r.get('latch_t_minus_cpa') for r in fl])}; TRUE range at latch {mstat([r.get('latch_rtrue') for r in fl])}")

P("")
P("=== Q2 fidelity ratios Gazebo / isim (matched arm) ===")


def ratio(gs, is_, key, kind="flights", fn=med, absval=True):
    gv = fn([r.get(key) for r in gs])
    iv = fn([r.get(key) for r in is_])
    if absval:
        gv, iv = abs(gv), abs(iv)
    return gv, iv, (gv / iv if iv not in (0, 0.0) and not math.isnan(iv) else math.nan)


for arm in ("brake", "nobrake"):
    gs = sel("gz", lambda r: r["arm"] == arm, "flights")
    is_ = sel("im", lambda r: r["arm"] == arm, "flights")
    P(f"-- {arm}: Gazebo n={len(gs)} flights, isim n={len(is_)} seeds")
    for key, lab in (("e_last2_med", "|last2 e| median"), ("e_pre2_med", "|pre-CPA2 e| median"),
                     ("cpa", "CPA"), ("last_det_before_cpa_s", "last det before CPA [s]"),
                     ("cpa_before_end_s", "CPA before phaseB end [s]")):
        g, i, rt = ratio(gs, is_, key)
        P(f"   {lab:28s} GZ {g:.3f}  isim {i:.3f}  ratio {rt:.1f}")
    gd = sel("gz", lambda r: r["arm"] == arm, "dets")
    idd = sel("im", lambda r: r["arm"] == arm, "dets")
    for lab, lo, hi in BINS:
        a = [d["e_post"] for d in gd if lo <= d["r_true"] < hi]
        b = [d["e_post"] for d in idd if lo <= d["r_true"] < hi]
        c = [d["pose_err_cam"] for d in gd if lo <= d["r_true"] < hi]
        dd = [d["pose_err_cam"] for d in idd if lo <= d["r_true"] < hi]
        if not a or not b:
            P(f"   update resid {lab:>4s}: UNCERTAIN (GZ n={len(a)}, isim n={len(b)})")
            continue
        P(f"   update resid {lab:>4s}: bias GZ {med(a):+.3f} isim {med(b):+.3f} | spread GZ {spread(a):.3f} isim {spread(b):.3f} ratio {spread(a)/spread(b) if spread(b) else math.nan:.1f}"
          f" | pose-err spread GZ {spread(c):.3f} isim {spread(dd):.3f} ratio {spread(c)/spread(dd) if spread(dd) else math.nan:.1f}  (n GZ {len(a)}, isim {len(b)})")
    gi = sel("gz", lambda r: r["arm"] == arm, "intervals")
    ii = sel("im", lambda r: r["arm"] == arm, "intervals")
    g, i = med([x["slope"] for x in gi]), med([x["slope"] for x in ii])
    P(f"   coast slope (scalar, all intervals) GZ {g:+.3f} (n={len(gi)}) isim {i:+.3f} (n={len(ii)}) |ratio| {abs(g/i) if i else math.nan:.1f}")
    g = med([r.get("R0_cmd_final_coast_vecE_slope") for r in gs])
    i = med([r.get("final_coast_vecE_slope") for r in is_])
    P(f"   final-coast vector-error slope GZ(R0 replica) {g:+.3f} isim {i:+.3f} ratio {g/i if i else math.nan:.1f}")
    g = med([r.get("R0_vrel_err_pre2_med") for r in gs])
    i = med([r.get("vrel_err_pre2_med") for r in is_])
    P(f"   |v_rel_hat - v_rel| pre-CPA2 GZ(R0) {g:.3f} isim {i:.3f} ratio {g/i if i else math.nan:.1f}")

open(os.path.join(OD, "summary.txt"), "w").write("\n".join(lines) + "\n")
