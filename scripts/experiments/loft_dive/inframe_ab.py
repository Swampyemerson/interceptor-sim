#!/usr/bin/env python3
"""Loft-then-dive vs flat run-in: the DECISIVE analytical in-frame A/B.

WHAT THIS IS (and is not)
-------------------------
The binding wall of the camera-only coded-dash intercept is POINTING, not the
sensor (docs/intercept_accuracy_levers.md; ADR-0076 add #18k). To dash forward a
quad pitches ~35-40 deg NOSE-DOWN, tipping the body-fixed camera down so a
CO-ALTITUDE target sits at/above the top of the frame -- the detector is ~100%
reliable STATIC at 8-22 m but ~0.8% in flight, and ~75% of ticks in the 8-12 m
band have the target OUT OF FRAME.

This script models the dash as pure kinematics + the body-pitch relation
theta = arctan(a_horizontal / (g + a_vertical)) and asks the ONE question that
gates everything downstream:

    across the 8-12 m slant-range band, what FRACTION of control ticks have the
    target inside the camera FOV -- FLAT run-in vs LOFT-then-DIVE -- swept over
    loft height dh, camera up-tilt wedge, and forward-accel cap?

It is a pure-geometry UPPER BOUND: it has NO detector, NO motion blur, NO
appearance model. A target "in frame" here would still have to be DETECTED. So
these numbers bound how much pointing alone can recover; only a Gazebo run
(see gazebo_run_recipe.md) turns this ranking into a result ("lab ranks, Gazebo
decides"). It reads NO ground truth from any flight -- it is a forward model of
the KNOWN launch kinematics (the same honest, pre-flight inputs the coded dash
aims from), so nothing here touches the honesty boundary.

MODEL
-----
World ENU: x=east, y=north, z=up (matches scripts/mc_batch.sh + the mover).
Camera intrinsics from configs/camera_intrinsics.json: fx=fy=539.936, 1280x960 =>
  half-HFOV = atan(640/539.936) = 49.85 deg, half-VFOV = atan(480/539.936)
  = 41.64 deg. Rectangular FOV test (matches scripts/experiments/dash_pitch_probe.py).

Interceptor: launches at world origin, flies a FIXED heading (the collision-lead
azimuth from flight.guidance.collision_lead_heading -- the open-loop coded dash
aim, a pre-flight constant). Forward speed ramps 0 -> dash_speed at the forward
accel cap a_cap, then cruises. Body nose-down pitch theta = arctan(a_h/(g+a_v)):
while accelerating theta = arctan(a_cap/g) (reduced/increased by any vertical
accel during a loft); at cruise a_h=0 so theta=0. Camera boresight elevation
(from horizontal, + up) = -theta + wedge (a fixed up-tilt mount, #40/ADR-0067).

  * FLAT arm: interceptor holds base altitude (the sim's current ALT_REF 0.5 m).
  * LOFT-DIVE arm: the interceptor CLIMBS to +dh BEFORE committing the dash (during
    the takeoff/positioning phase, which already takes seconds), so the DASH itself
    is a monotonic gentle DIVE from base+dh down to co-altitude at intercept:
    z(t) = base + dh * 0.5 (1 + cos(pi t/T)) (a raised-cosine descent, T =
    engagement duration). Diving keeps the interceptor ABOVE the co-altitude target
    so the LOS points DOWN -- toward where the nose-down boresight already looks.
    Modelling the loft as pre-dash + a descent (NOT a ballistic lob within the
    dash) keeps the required vertical rate/accel realistic; the descent's vertical
    accel still feeds back into theta = arctan(a_h/(g+a_v)) honestly (it is small
    away from the endpoints), and the peak descent rate/accel are REPORTED as the
    loft's kinematic cost. WHY pre-dash climb: at 16 m/s the whole engagement is
    only ~1.7-2.5 s, so lofting AND diving 3-4 m *within* the dash would demand
    near-g vertical accel -- infeasible and self-defeating (it pitches the quad).

Target: straight-line crossing at target_vel from target_start (the canonical
9 m/s line geometry, extracted below).

OVERSHOOT: if dh is too large the target drops BELOW the bottom FOV edge (the
dive out-aims the nose-down pitch). The table reports frac_out_bottom so this is
visible, per the task's warning.

USAGE
  .venv/bin/python scripts/experiments/loft_dive/inframe_ab.py
    [--dash-speed 16] [--a-cap-deg 40 30 20 15] [--dh 0 2 3 4]
    [--wedge 0 10 20 25 30] [--csv logs/loft_dive_inframe_ab.csv]
"""
import argparse
import csv
import math
import os
import sys

# Portable coded-dash aim (no gz/gt) -- the same pre-flight lead the dash flies.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))
from flight.guidance import collision_lead_heading  # noqa: E402

# --- camera intrinsics (configs/camera_intrinsics.json) ---
FX = FY = 539.9363327026367
IMG_W, IMG_H = 1280, 960
HALF_HFOV_DEG = math.degrees(math.atan((IMG_W / 2.0) / FX))   # 49.85
HALF_VFOV_DEG = math.degrees(math.atan((IMG_H / 2.0) / FY))   # 41.64

G = 9.80665
DT = 0.05                 # 20 Hz control tick (CONTROL_RATE_HZ in m4_intercept.py)
BAND_LO, BAND_HI = 8.0, 12.0   # the acquisition range band that matters (add #18k)
NEAR_LO, NEAR_HI = 2.0, 5.0    # terminal band -- the quad BRAKES (nose-up) here
CENTERED_FRAC = 0.6       # "detection-comfortable": within +/- CENTERED_FRAC*half-VFOV
                          # of boresight (away from the frame-edge corners where the
                          # center-biased detector fails even when geometrically in-frame)
TERMINAL_BRAKE_RANGE_M = 5.0   # inside this the closing speed throttles -> decel -> nose-UP
TERMINAL_DECEL_MS2 = 4.0       # representative terminal braking decel (V_CLOSE 9->5.5 over
                               # the throttle band); nose-up pitch = arctan(decel/g)

# Interceptor camera sits ~0.25 m above base_link (ADR-0006) and ~0.12 m forward.
CAM_UP_M = 0.25
CAM_FWD_M = 0.12
BASE_ALT_M = 0.5          # ALT_REF_M in m4_intercept.py (current sim dash altitude)
TARGET_ALT_M = 0.5        # tag z (mover holds it constant)

# CANONICAL straight-line 9 m/s crossing geometry. Extracted from the committed
# coded-dash line-9 mc_batch arm (logs/mc_coded_dash_qv2_line9_s123.csv /
# logs/mc_batch_run_0_20260715T201327Z.log): target pre-placed at
# start=(6.500, -15.343, 0.5), vel=(0.000, 9.000) [E,N], dash_speed 16 m/s,
# heading auto = collision-lead. y0 magnitude ~15.3 is the mc_batch --y0-mag
# default for the line geometry; x0=6.5. Interceptor launches at world origin.
TGT_START = (6.5, -15.343)     # (east, north) m
TGT_VEL = (0.0, 9.0)           # (east, north) m/s -- r2l/l2r sign is symmetric for VFOV
DASH_SPEED = 16.0


def simulate(dash_speed, a_cap_deg, dh, wedge_deg, heading_deg, t_lead_est,
             tgt_start, tgt_vel, dt=DT, t_max=6.0):
    """One dash. Returns per-tick rows over the 8-12 m band with in-frame flags.

    a_cap_deg: body nose-down pitch at full forward accel; a_cap = g*tan(a_cap_deg)
      is the forward-accel cap (the accel-capped constant-pitch lever). Larger =>
      harder accel => more nose-down. dh: loft height (0 = flat). wedge_deg:
      fixed camera up-tilt. heading_deg: the fixed open-loop dash azimuth.
    """
    a_cap = G * math.tan(math.radians(a_cap_deg))
    psi = math.radians(heading_deg)           # compass: 0=N, 90=E
    fh = (math.sin(psi), math.cos(psi), 0.0)  # forward horizontal unit (E,N,U)
    rgt = (math.cos(psi), -math.sin(psi), 0.0)  # camera right unit (horizontal)
    T = max(t_lead_est, 0.4)                  # dive-profile engagement duration

    s = 0.0            # horizontal distance travelled along heading
    v = 0.0            # forward speed
    band_rows, near_rows = [], []
    peak_vz = 0.0
    peak_av = 0.0
    t = 0.0
    prev_z = BASE_ALT_M + dh    # LOFT-DIVE starts the dash already lofted at +dh
    prev_vz = 0.0
    n = int(t_max / dt)
    for _ in range(n):
        # --- forward speed ramp, cruise, then terminal BRAKE (nose-up) ---
        tx = tgt_start[0] + tgt_vel[0] * t
        ty = tgt_start[1] + tgt_vel[1] * t
        tz = TARGET_ALT_M
        # provisional range for the brake test (from last position is fine at 20 Hz)
        rng_prev = math.sqrt((tx - fh[0] * s) ** 2 + (ty - fh[1] * s) ** 2
                             + (tz - (prev_z + CAM_UP_M)) ** 2)
        braking = rng_prev <= TERMINAL_BRAKE_RANGE_M and v > 5.5
        if braking:
            a_h = -TERMINAL_DECEL_MS2                 # decel -> nose-UP
            v = max(5.5, v + a_h * dt)
        else:
            a_h = a_cap if v < dash_speed else 0.0    # accel -> nose-DOWN
            v = min(dash_speed, v + a_h * dt)
        s += v * dt

        # --- monotonic DIVE from base+dh to base over the engagement (loft was
        #     done pre-dash); flat arm holds base. Raised cosine = smooth ends. ---
        if dh > 0.0:
            frac = min(max(t / T, 0.0), 1.0)
            z_base = BASE_ALT_M + dh * 0.5 * (1.0 + math.cos(math.pi * frac))
        else:
            z_base = BASE_ALT_M
        vz = (z_base - prev_z) / dt
        a_v = (vz - prev_vz) / dt
        prev_z, prev_vz = z_base, vz
        peak_vz = min(peak_vz, vz)            # most-negative (fastest descent)
        peak_av = max(peak_av, abs(a_v))

        # --- body pitch from the accel relation (nose-down while a_h>0,
        #     nose-UP while braking a_h<0), with the honest vertical coupling ---
        theta = math.atan2(a_h, G + a_v)      # rad; + = nose-down, - = nose-up
        bore_el = -theta + math.radians(wedge_deg)   # boresight elevation, +up

        # --- interceptor camera position (base + mount, in world) ---
        ix = fh[0] * (s + CAM_FWD_M * math.cos(theta))
        iy = fh[1] * (s + CAM_FWD_M * math.cos(theta))
        iz = z_base + CAM_UP_M * math.cos(theta) - CAM_FWD_M * math.sin(theta)

        los = (tx - ix, ty - iy, tz - iz)
        rng = math.sqrt(los[0] ** 2 + los[1] ** 2 + los[2] ** 2)

        # --- camera-frame basis (boresight pitched, wedge added) ---
        cb, sb = math.cos(bore_el), math.sin(bore_el)
        f = (fh[0] * cb, fh[1] * cb, sb)                     # forward/boresight
        u = (-fh[0] * sb, -fh[1] * sb, cb)                   # camera up
        fwd = los[0] * f[0] + los[1] * f[1] + los[2] * f[2]
        rt = los[0] * rgt[0] + los[1] * rgt[1] + los[2] * rgt[2]
        up = los[0] * u[0] + los[1] * u[1] + los[2] * u[2]
        vert = math.degrees(math.atan2(up, fwd))    # + = target ABOVE boresight
        horiz = math.degrees(math.atan2(rt, fwd))   # + = target RIGHT of boresight

        in_vfov = abs(vert) <= HALF_VFOV_DEG
        in_hfov = abs(horiz) <= HALF_HFOV_DEG
        in_frame = in_vfov and in_hfov and fwd > 0.0
        centered = (abs(vert) <= CENTERED_FRAC * HALF_VFOV_DEG
                    and abs(horiz) <= CENTERED_FRAC * HALF_HFOV_DEG and fwd > 0.0)
        rec = {
            "t": t, "rng": rng, "theta_deg": math.degrees(theta),
            "bore_el_deg": math.degrees(bore_el), "vert": vert, "horiz": horiz,
            "in_vfov": in_vfov, "in_hfov": in_hfov, "in_frame": in_frame,
            "centered": centered,
            "out_top": vert > HALF_VFOV_DEG, "out_bottom": vert < -HALF_VFOV_DEG,
        }
        if BAND_LO <= rng <= BAND_HI and fwd > 0.0:
            band_rows.append(rec)
        if NEAR_LO <= rng <= NEAR_HI and fwd > 0.0:
            near_rows.append(rec)
        t += dt
        if rng < 1.5 and s > 1.0:
            break
    return band_rows, near_rows, {"peak_descent_ms": -peak_vz, "peak_av_ms2": peak_av}


def summarize(rows):
    n = len(rows)
    if n == 0:
        return None
    f = lambda key: sum(1 for r in rows if r[key]) / n  # noqa: E731
    verts = [r["vert"] for r in rows]
    return {
        "n_band": n,
        "frac_in_frame": f("in_frame"),
        "frac_centered": f("centered"),
        "frac_in_vfov": f("in_vfov"),
        "frac_out_top": f("out_top"),
        "frac_out_bottom": f("out_bottom"),
        "vert_med": sorted(verts)[n // 2],
        "vert_min": min(verts),
        "vert_max": max(verts),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dash-speed", type=float, default=DASH_SPEED)
    ap.add_argument("--a-cap-deg", type=float, nargs="+", default=[40, 30, 20, 15],
                    help="body nose-down pitch (deg) at full forward accel; "
                         "a_cap = g*tan(this). 40 ~ today's uncapped dash.")
    ap.add_argument("--dh", type=float, nargs="+", default=[0, 2, 3, 4],
                    help="loft height (m); 0 = flat run-in (the baseline arm).")
    ap.add_argument("--wedge", type=float, nargs="+", default=[0, 10, 20, 25, 30],
                    help="fixed camera up-tilt (deg).")
    ap.add_argument("--csv", default="scripts/experiments/loft_dive/inframe_ab_results.csv",
                    help="output CSV (default committed alongside the script; logs/ is "
                         "gitignored so results there would not persist).")
    args = ap.parse_args()

    heading_deg, t_lead = collision_lead_heading(TGT_START, TGT_VEL, args.dash_speed)
    # The constant-speed lead solve underestimates duration (the real dash ramps
    # from 0); scale the loft-profile duration to the ramp. Rough closed form:
    # time to cover the lead distance d under a mid accel. Use a simple estimate
    # from the a_cap=30deg mid case so the loft shape is stable across the sweep.
    a_mid = G * math.tan(math.radians(30))
    d_lead = math.hypot(TGT_START[0] + TGT_VEL[0] * t_lead,
                        TGT_START[1] + TGT_VEL[1] * t_lead)
    t_ramp = math.sqrt(2 * d_lead / a_mid)   # if still accelerating over the run
    t_lead_est = max(t_lead, min(t_ramp, d_lead / max(args.dash_speed, 1e-6) + 1.0))

    print(__doc__.split("USAGE")[0].rstrip())
    print("=" * 78)
    print("CANONICAL GEOMETRY (line 9 m/s, logs/mc_coded_dash_qv2_line9_s123.csv):")
    print(f"  target start (E,N) = {TGT_START} m, vel = {TGT_VEL} m/s, "
          f"tag z = {TARGET_ALT_M} m")
    print(f"  interceptor: origin, base alt {BASE_ALT_M} m, cam +{CAM_UP_M} up "
          f"/ +{CAM_FWD_M} fwd, dash {args.dash_speed:.0f} m/s")
    print(f"  collision-lead heading = {heading_deg:.2f} deg, "
          f"const-speed t_lead = {t_lead:.2f} s, loft-profile T = {t_lead_est:.2f} s")
    print(f"  camera half-FOV: VFOV +/-{HALF_VFOV_DEG:.2f} deg, "
          f"HFOV +/-{HALF_HFOV_DEG:.2f} deg | band {BAND_LO:.0f}-{BAND_HI:.0f} m")
    print("=" * 78)

    rows_out = []
    for a_cap_deg in args.a_cap_deg:
        print(f"\n--- forward-accel cap: theta_max = {a_cap_deg:.0f} deg "
              f"(a_cap = {G*math.tan(math.radians(a_cap_deg)):.2f} m/s^2) ---")
        print(f"  {'arm':>5} {'dh':>4} {'wedge':>6} {'n':>4} | 8-12 m BAND: "
              f"{'IN-FRAME':>9} {'CENTERED':>9} {'out_top':>8} {'out_bot':>8} "
              f"{'vert_med':>9} | <5m: {'IN-FRAME':>9} {'out_bot':>8} | loft cost")
        for dh in args.dh:
            for wedge in args.wedge:
                band, near, cost = simulate(
                    args.dash_speed, a_cap_deg, dh, wedge,
                    heading_deg, t_lead_est, TGT_START, TGT_VEL)
                s = summarize(band)
                sn = summarize(near)
                arm = "flat" if dh == 0 else "loft"
                if s is None:
                    print(f"  {arm:>5} {dh:>4.0f} {wedge:>6.0f}  (no ticks in band)")
                    continue
                near_if = f"{sn['frac_in_frame']:>8.0%}" if sn else "     n/a"
                near_ob = f"{sn['frac_out_bottom']:>7.0%}" if sn else "    n/a"
                cost_str = (f"vz {cost['peak_descent_ms']:.1f} a {cost['peak_av_ms2']:.1f}"
                            if dh > 0 else "-")
                print(f"  {arm:>5} {dh:>4.0f} {wedge:>6.0f} {s['n_band']:>4} | "
                      f"{'':13}{s['frac_in_frame']:>8.0%} {s['frac_centered']:>8.0%} "
                      f"{s['frac_out_top']:>8.0%} {s['frac_out_bottom']:>8.0%} "
                      f"{s['vert_med']:>+8.1f} | {'':5} {near_if} {near_ob} | {cost_str}")
                row = {
                    "arm": arm, "theta_max_deg": a_cap_deg, "dh_m": dh,
                    "wedge_deg": wedge, "n_band": s["n_band"],
                    "band_in_frame": round(s["frac_in_frame"], 4),
                    "band_centered": round(s["frac_centered"], 4),
                    "band_out_top": round(s["frac_out_top"], 4),
                    "band_out_bottom": round(s["frac_out_bottom"], 4),
                    "band_vert_med_deg": round(s["vert_med"], 2),
                    "band_vert_min_deg": round(s["vert_min"], 2),
                    "band_vert_max_deg": round(s["vert_max"], 2),
                    "near_n": sn["n_band"] if sn else 0,
                    "near_in_frame": round(sn["frac_in_frame"], 4) if sn else None,
                    "near_out_bottom": round(sn["frac_out_bottom"], 4) if sn else None,
                    "loft_peak_descent_ms": round(cost["peak_descent_ms"], 2),
                    "loft_peak_av_ms2": round(cost["peak_av_ms2"], 2),
                    "heading_deg": round(heading_deg, 2),
                    "dash_speed": args.dash_speed,
                }
                rows_out.append(row)

    # --- headline. IN-FRAME alone is a knife-edge: flat theta~40 pins the target
    #     at the +39 deg frame-TOP edge and STILL scores "in frame". The honest bar
    #     is CENTERED in the acquisition band AND held in-frame through the <5 m
    #     terminal (where the quad brakes nose-UP and a fat wedge dumps it out the
    #     bottom). Report the baseline, the accel-cap-alone effect, and the
    #     fixed-wedge terminal trap, rather than crowning one arbitrary combo. ---
    def get(arm, th, dh, wg):
        return next((r for r in rows_out if r["arm"] == arm and r["theta_max_deg"] == th
                     and r["dh_m"] == dh and r["wedge_deg"] == wg), None)
    th_hi = max(args.a_cap_deg)
    base = get("flat", th_hi, 0, 0)
    print("\n" + "=" * 92)
    print("HEADLINE -- 8-12 m acquisition band (+ <5 m terminal), fraction of ticks:")
    if base:
        print(f"  BASELINE today (flat, no wedge, theta~{th_hi:.0f}): band in-frame "
              f"{base['band_in_frame']:.0%} but CENTERED {base['band_centered']:.0%} -- target "
              f"pinned at vert_med {base['band_vert_med_deg']:+.0f} deg, the frame-TOP edge "
              f"(+/-{HALF_VFOV_DEG:.0f}). THIS is the ~0.8%-in-flight wall.")
    # accel-cap alone (flat, no wedge), swept
    print("  ACCEL-CAP alone (flat, no wedge) -- the cleanest single lever:")
    for th in sorted(set(args.a_cap_deg), reverse=True):
        r = get("flat", th, 0, 0)
        if r:
            print(f"     theta_max {th:>2.0f} deg: band centered {r['band_centered']:>4.0%} "
                  f"(vert_med {r['band_vert_med_deg']:+5.0f}), <5m in-frame "
                  f"{r['near_in_frame']:>4.0%} out-bottom {r['near_out_bottom']:>4.0%}")
    # the fixed-wedge terminal trap (flat, big wedge, high theta)
    trap = get("flat", th_hi, 0, 30)
    if trap and base:
        print(f"  FIXED-WEDGE TRAP: flat theta~{th_hi:.0f} + wedge 30 centers the BAND "
              f"({trap['band_centered']:.0%}, vert_med {trap['band_vert_med_deg']:+.0f}) but "
              f"DUMPS the <5m terminal out the bottom (<5m in-frame {trap['near_in_frame']:.0%}, "
              f"out-bottom {trap['near_out_bottom']:.0%}) -- 'a fixed tilt only relocates the "
              f"window' (ADR-0076 #18j-fix). So the wedge must stay MODEST.")
    # loft-dive as the complement
    ld = get("loft", 30, 2, 10)
    if ld:
        print(f"  LOFT-DIVE complement: theta~30 + dh 2 + wedge 10 -> band centered "
              f"{ld['band_centered']:.0%} (vert_med {ld['band_vert_med_deg']:+.0f}), <5m in-frame "
              f"{ld['near_in_frame']:.0%}; loft costs peak descent "
              f"{ld['loft_peak_descent_ms']:.1f} m/s, vert-accel {ld['loft_peak_av_ms2']:.1f} m/s^2.")
    print("  --> Read CENTERED, not IN-FRAME. Primary lever = ACCEL-CAP (centers the")
    print("      band, no maneuver, no terminal dump); wedge MODEST; loft-dive = complement.")
    print("      Cost of the cap: slower closing (theta20 a_cap 3.6 vs 8.2 m/s^2 -> the")
    print("      dash never reaches 16 m/s in ~2 s; longer t_go, staler open-loop lead).")
    print("  Pure-geometry UPPER BOUND (no detector, no blur, perfect aim). Gazebo decides.")
    print("=" * 92)

    csv_path = args.csv
    if not os.path.isabs(csv_path):
        csv_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.abspath(__file__))))), csv_path)
    with open(csv_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows_out[0].keys()))
        w.writeheader()
        w.writerows(rows_out)
    print(f"\nwrote {len(rows_out)} rows -> {csv_path}")


if __name__ == "__main__":
    main()
