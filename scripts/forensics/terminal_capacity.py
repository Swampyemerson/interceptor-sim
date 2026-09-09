"""Correction-capacity arithmetic from repo constants only. No new runs."""
K = 5                      # consecutive detections for handoff (real_flight acquire_streak)
A_LAT = 8.7                # m/s^2, median achieved lateral accel in ENGAGE (terminal_diagnosis.md:53)
A_CAP = 12.0               # m/s^2, MPC_ACC_HOR_MAX (FPV bundle)
A_ACHIEVED = 6.7           # m/s^2, actually achieved (ADR-0028 addendum) - guidance-ceiling limited

def burn_frames(p, k=K):
    """Run-length expectation for k CONSECUTIVE successes (ADR-0079)."""
    pk = p ** k
    return (1 - pk) / (pk * (1 - p))

def row(label, r90, rate_hz, p, v_burn, v_close, a=A_LAT):
    f = burn_frames(p)
    t_burn = f / rate_hz
    burn_m = t_burn * v_burn
    t_go = (r90 - burn_m) / v_close
    cap = 0.5 * a * t_go ** 2 if t_go > 0 else 0.0
    print(f"{label:<44} burn {burn_m:5.2f} m   t_go {t_go:6.3f} s   capacity {cap:6.2f} m")
    return t_go, cap

print("Burn frames at p=0.9, k=5:", round(burn_frames(0.9), 3), "(repo asserts 6.9351)\n")

print("--- AS THE GATE COMPUTES IT (burn and t_go both at 9 m/s) ---")
row("tag qd=2.0, R90 7.10 m, 20 Hz loop, p=0.9", 7.10, 20, 0.9, 9.0, 9.0)
row("tag qd=1.0, R90 8.97 m, 20 Hz loop, p=0.9", 8.97, 20, 0.9, 9.0, 9.0)
row("tag qd=2.0 pessimistic, R90 5.98 m, 14 Hz", 5.98, 14, 0.9, 9.0, 9.0)

print("\n--- WITH THE BURN AT DASH SPEED (streak forms in CODED_DASH at 16 m/s) ---")
row("tag qd=2.0, R90 7.10 m, burn@16, t_go@9", 7.10, 20, 0.9, 16.0, 9.0)
row("tag qd=1.0, R90 8.97 m, burn@16, t_go@9", 8.97, 20, 0.9, 16.0, 9.0)
row("tag qd=1.0, R90 8.97 m, burn@13.2, t_go@9", 8.97, 20, 0.9, 13.2, 9.0)

print("\n--- MARKERLESS ON AN ACCELERATOR (static ceiling ~24 m; take 20 m) ---")
row("NN R_acq 20 m, 20 Hz, p=0.9, burn@16", 20.0, 20, 0.9, 16.0, 9.0)

print("\n--- SENSITIVITY ON LATERAL AUTHORITY, at t_go = 0.65 s ---")
for a in (A_ACHIEVED, A_LAT, A_CAP, 20.0):
    print(f"  a = {a:5.1f} m/s^2  ->  capacity {0.5*a*0.65**2:5.2f} m")

print("\n--- WHAT NEEDS CORRECTING (ADR-0095, adopted config n=16) ---")
v, h = 0.374, 0.174
mag = (v*v + h*h) ** 0.5
print(f"  vertical bias {v} m, horizontal {h} m, magnitude {mag:.3f} m")
print(f"  vertical share of squared error: {100*v*v/(v*v+h*h):.0f}%")
print(f"  kill radius 0.35 m -> correction needed on the median: {mag-0.35:.3f} m")
print(f"  if the vertical bias alone were nulled: miss = {h:.3f} m  (inside 0.35 m)")
