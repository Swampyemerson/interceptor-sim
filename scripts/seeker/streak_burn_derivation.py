"""Derivations backing docs/detection_tracking_methods.md sections A.4, B.3, E.2.
Pure arithmetic; no sim, no data. Run: python3 scripts/seeker/streak_burn_derivation.py

Also backs the TODO in scripts/seeker/tripod_score.py:gate_verdict (the money gate's
streak-burn model). Two models are compared there and here:
  mean-rate  E[frames] ~ k/p           <- what gate_verdict uses today (decode_Hz = p*fps)
  run-length E[frames] = (1-p^k)/(p^k(1-p))  <- correct for k CONSECUTIVE detections
Both agree at p=1; mean-rate is OPTIMISTIC below it.
"""
from math import comb

def E_run(p, k):
    """Expected Bernoulli trials to first run of k consecutive successes."""
    return (1 - p**k) / (p**k * (1 - p))

K, V, FPS = 5, 9.0, 30.0
print("=== A.4  frames-to-5-streak vs per-frame recall (k=5, V=9 m/s, 30 fps) ===")
print("  run-length = (1-p^k)/(p^k(1-p))   mean-rate = k/p (gate_verdict today)")
print(f"{'p':>6} {'E[frames]':>10} {'burn_m':>8} {'R_acq(tgo>=.5s)':>16} "
      f"{'mean-rate_frm':>14} {'optimism':>9}")
for p in (0.9, 0.8, 0.7, 0.6, 0.5, 0.4417):
    e = E_run(p, K); burn = e / FPS * V; mr = K / p
    print(f"{p:6.4f} {e:10.1f} {burn:8.2f} {burn + 0.5*V:16.2f} {mr:14.1f} {e/mr:8.2f}x")
print(f"{'1.00':>6} {K:10.1f} {K/FPS*V:8.2f} {K/FPS*V + 0.5*V:16.2f} "
      f"{float(K):14.1f} {1.0:8.2f}x  (models agree at p=1)")

print("\n=== A.4  M-of-N alternative at p=0.4417 (5 of last 8) ===")
p = 0.4417
P = sum(comb(8, i) * p**i * (1-p)**(8-i) for i in range(5, 9))
print(f"P(>=5 hits in 8 frames) = {P:.4f}; expected windows to pass = {1/P:.2f}")
print(f"~frames = {8/1*(1/P):.0f} (independent-window bound) | sliding-window ~{8 + (1/P-1)*1:.0f}")
print(f"closure at 30 fps, 9 m/s over 32 frames = {32/30*9:.1f} m  vs strict-streak {E_run(p,5)/30*9:.1f} m")

print("\n=== B.3  streak burn vs detector fps (p=1.0 floor) ===")
for fps in (5, 8, 10, 30, 60):
    for v in (9.0, 20.0):
        burn = K/fps*v
        print(f"  fps={fps:3d} V={v:4.1f} burn={burn:5.2f} m  R_acq needed={burn+0.5*v:6.2f} m")

print("\n=== E.2  motion blur smear (OV9281 10.85 px/deg H = 1280/118) ===")
pxdeg = 1280/118
for rate in (485.0, 1870.0):
    for exp_ms in (5.0, 1.0):
        deg = rate * exp_ms/1000.0
        print(f"  LOS {rate:6.0f} deg/s x {exp_ms:.0f} ms = {deg:5.2f} deg = {deg*pxdeg:6.1f} px")
print(f"  (px/deg = {pxdeg:.2f}; target subtends 3.4-18 px @640 input over 6-20 m)")

print("\n=== D.6  naive-independence false 5-run illustration (p_ff=0.049) ===")
pff = 0.049
print(f"  p^5 = {pff**5:.3e}; expected 5-runs per 1000 frames (independent) ~ {1000*pff**5:.2e}")
print("  NOTE: clutter false fires are temporally CORRELATED -> this model is wrong;")
print("  the acceptance statistic is the OBSERVED max consecutive false-fire run length.")
