#!/usr/bin/env python3
"""ADR-0069 KILL GATE — can a clean-enough TARGET-ACCELERATION estimate even be
extracted from the fused cue to make an accel-augmented dash lead (`--dash-accel-
lead`) worth flying? This is the CHEAP unit-test decision gate the adversarial
review demanded BEFORE any sim spend.

THE PHYSICS UNDER TEST. The one honesty-legal accel signal is a derivative of the
cue-emitted NED velocity (s2_cue_mock: DEFAULT_VEL_SIGMA_M_S=0.5, DEFAULT_RATE_HZ
=10). Differentiating a sigma=0.5 m/s, 10 Hz stream gives raw accel noise
sigma_a_raw = sigma_v*sqrt(2)/dt = 0.5*1.414/0.1 ~ 7.07 m/s^2 — SNR<1 against the
4.71 m/s^2 weave peak. Smoothing cuts the noise but adds phase LAG; against a 4 s-
period weave a lag near 90 deg makes the accel term point the WRONG way at each
reversal (Zarchan). This probe sweeps the estimator smoothing and measures the
noise/lag Pareto frontier to see if ANY tuning meets BOTH pre-registered bars:
  sigma_a (RMS estimate error) <= 1.5 m/s^2   AND   phase lag <= 1/8 weave period.

THE WEAVE (verified vs scripts/m4_target_mover.py + mc_jam_arm.sh):
  lat displacement A=1.91 m, period T=4.0 s -> omega=2pi/T=1.571 rad/s.
  v_y = A*omega*cos(wt)  (amp 3.00 m/s = lat-speed)  ;
  a_y = -A*omega^2*sin(wt) (amp 4.71 m/s^2 = peak accel).

ESTIMATORS (fair test — the two cheap standard options the design named):
  (1) naive: finite-difference of cue velocity + EMA smoothing (alpha sweep).
  (2) alpha-beta on the velocity channel (=> smoothed velocity + acceleration),
      the standard constant-accel estimator, critically-damped beta=alpha^2/(2-alpha).

No sim, no gt_*, deterministic (fixed noise seed grid). Verdict KILL/PASS printed.
"""
import math

import numpy as np

A_DISP = 1.91
T_WEAVE = 4.0
OMEGA = 2 * math.pi / T_WEAVE
A_PEAK = A_DISP * OMEGA ** 2          # 4.71 m/s^2
V_AMP = A_DISP * OMEGA               # 3.00 m/s
CUE_HZ = 10.0
DT = 1.0 / CUE_HZ
SIGMA_V = 0.5
SIGMA_A_BAR = 1.5                    # kill-gate: RMS accel-estimate error
LAG_BAR = T_WEAVE / 8.0             # kill-gate: 0.5 s (45 deg)
DUR = 40.0                           # long run to average out noise stats
N_SEEDS = 24


def truth(t):
    return -A_PEAK * np.sin(OMEGA * t)   # a_y(t)


def cue_velocity(t, rng):
    return V_AMP * np.cos(OMEGA * t) + rng.normal(0, SIGMA_V, size=t.shape)


def est_naive(vmeas, alpha):
    """finite-difference + EMA. Returns a_hat aligned to vmeas samples."""
    a = np.zeros_like(vmeas)
    a_hat = 0.0
    for k in range(1, len(vmeas)):
        a_raw = (vmeas[k] - vmeas[k - 1]) / DT
        a_hat += alpha * (a_raw - a_hat)
        a[k] = a_hat
    return a


def est_alpha_beta(vmeas, alpha):
    """alpha-beta on the velocity channel (critically damped) -> vel + accel."""
    beta = alpha * alpha / (2 - alpha)
    v = vmeas[0]
    a = 0.0
    out = np.zeros_like(vmeas)
    for k in range(len(vmeas)):
        v_p = v + a * DT
        a_p = a
        r = vmeas[k] - v_p
        v = v_p + alpha * r
        a = a_p + (beta / DT) * r
        out[k] = a
    return out


def phase_lag_s(a_true, a_est, t):
    """lag (s) = shift maximizing cross-correlation of est vs true over +-1 period."""
    best, best_c = 0.0, -1e9
    for shift in range(0, int(T_WEAVE / DT)):
        if shift >= len(t):
            break
        c = np.corrcoef(a_true[:len(t) - shift], a_est[shift:])[0, 1]
        if c > best_c:
            best_c, best = c, shift * DT
    return best, best_c


def evaluate(est_fn, alphas):
    t = np.arange(0, DUR, DT)
    a_true = truth(t)
    warm = int(len(t) * 0.25)   # drop the filter settle
    rows = []
    for alpha in alphas:
        sig_list, lag_list = [], []
        for s in range(N_SEEDS):
            rng = np.random.default_rng(1000 + s)
            v = cue_velocity(t, rng)
            a_est = est_fn(v, alpha)
            # align by the estimator's own lag, then measure residual RMS about truth
            lag, corr = phase_lag_s(a_true[warm:], a_est[warm:], t[warm:])
            sig = float(np.sqrt(np.mean((a_est[warm:] - a_true[warm:]) ** 2)))
            sig_list.append(sig)
            lag_list.append(lag)
        rows.append((alpha, float(np.mean(sig_list)), float(np.mean(lag_list))))
    return rows


def main():
    alphas = [0.02, 0.05, 0.1, 0.15, 0.25, 0.4, 0.6, 0.8, 1.0]
    print(f"WEAVE: A_peak={A_PEAK:.2f} m/s^2, V_amp={V_AMP:.2f} m/s, T={T_WEAVE}s, "
          f"cue {CUE_HZ:.0f} Hz sigma_v={SIGMA_V} m/s")
    print(f"raw diff noise sigma_a_raw = {SIGMA_V*math.sqrt(2)/DT:.2f} m/s^2 "
          f"(SNR vs peak = {A_PEAK/(SIGMA_V*math.sqrt(2)/DT):.2f})")
    print(f"KILL-GATE BARS: sigma_a <= {SIGMA_A_BAR} m/s^2 AND lag <= {LAG_BAR:.2f}s "
          f"({math.degrees(OMEGA*LAG_BAR):.0f} deg)\n")
    any_pass = False
    for name, fn in [("naive diff+EMA", est_naive), ("alpha-beta (CA)", est_alpha_beta)]:
        print(f"--- estimator: {name} ---")
        print(f"  {'alpha':>6} {'sigma_a':>9} {'lag_s':>7} {'lag_deg':>8}  verdict")
        for alpha, sig, lag in evaluate(fn, alphas):
            ok = sig <= SIGMA_A_BAR and lag <= LAG_BAR
            any_pass = any_pass or ok
            print(f"  {alpha:>6.2f} {sig:>9.2f} {lag:>7.2f} "
                  f"{math.degrees(OMEGA*lag):>7.0f}  {'PASS' if ok else 'fail'}")
        print()
    print("=" * 60)
    if any_pass:
        print("VERDICT: PASS — a tuning meets both bars. Proceed to flight integration + A/B.")
    else:
        print("VERDICT: KILL — NO tuning meets sigma_a<=1.5 AND lag<=1/8 period simultaneously.")
        print("  The cue-velocity derivative cannot yield a clean, in-phase accel estimate")
        print("  for a 4 s-period weave. Accel-augmented dash lead is a pre-known negative")
        print("  (ADR-0069) — do NOT spend sim runs. The lever is acquisition range (#46).")


if __name__ == "__main__":
    main()
