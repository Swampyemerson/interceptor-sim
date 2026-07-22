"""Pin flight.terminal_coast (Phase D: tau-armed freeze-the-LOS-rate endgame
coast). Pure-math behavioral assertions on synthetic tracks -- no sim. Run
standalone (exit 0/1) or under pytest.
"""

import math

from flight.range_fusion import SizeRangeEstimator
from flight.terminal_coast import CoastCommand, TerminalCoast

N = 4.0  # matches m4's N_PRONAV


def _tracked(tc, t, tau):
    """One live-track tick with a canonical state."""
    tc.update_track(lambda_rad=0.3, lambda_dot_rad_s=2.0, vc_m_s=5.0,
                    t_sim=t, r_hat_m=1.0, tau_s=tau)


def test_no_snapshot_no_coast():
    """Track loss before any live track: nothing to freeze -> no coast."""
    tc = TerminalCoast(n_gain=N)
    assert not tc.on_track_loss(1.0)
    assert tc.command(1.0) is None


def test_tau_arming_gates_engagement():
    """The tau clock is the arm switch: loss with tau ABOVE the threshold
    (mid-course dropout) does NOT coast -- that stays the estimator's
    predict() job; loss with tau below it does."""
    tc = TerminalCoast(n_gain=N, arm_tau_s=0.35)
    _tracked(tc, 10.0, tau=1.2)                      # far out
    assert not tc.armed
    assert not tc.on_track_loss(10.05)
    assert not tc.coasting
    _tracked(tc, 10.1, tau=0.2)                      # endgame
    assert tc.armed
    assert tc.on_track_loss(10.15)
    assert tc.coasting


def test_unknown_tau_never_arms():
    """No tau feed (e.g. --range-fusion off, no bbox stream) -> never armed,
    module inert -- the safe default composition."""
    tc = TerminalCoast(n_gain=N)
    _tracked(tc, 10.0, tau=None)
    assert not tc.armed and not tc.on_track_loss(10.05)


def test_noisy_tau_dip_keeps_arm():
    """STICKY TAU (review finding 4): a single noisy wdot dip that makes
    tau_s None on the FINAL live tick must NOT disarm the coast -- the last
    valid tau keeps it armed within the staleness window, so the immediate
    track loss still coasts."""
    tc = TerminalCoast(n_gain=N, arm_tau_s=0.35, tau_stale_s=0.30)
    tc.update_track(0.3, 2.0, 5.0, 10.00, r_hat_m=1.0, tau_s=0.2)   # armed
    assert tc.armed
    tc.update_track(0.3, 2.0, 5.0, 10.03, r_hat_m=0.9, tau_s=None)  # noisy dip
    assert tc.armed                                                 # still armed
    assert tc.on_track_loss(10.05)                                  # -> coasts
    assert tc.coasting


def test_stale_tau_eventually_disarms():
    """The sticky arm is not permanent: past tau_stale_s of sim time with no
    fresh valid tau, `armed` goes False (an old opening geometry can't keep
    the coast armed forever)."""
    tc = TerminalCoast(n_gain=N, arm_tau_s=0.35, tau_stale_s=0.30)
    tc.update_track(0.3, 2.0, 5.0, 10.0, r_hat_m=1.0, tau_s=0.2)
    assert tc.armed
    tc.update_track(0.3, 2.0, 5.0, 10.5, r_hat_m=8.0, tau_s=None)   # 0.5 s later
    assert not tc.armed
    assert not tc.on_track_loss(10.55)


def test_freeze_propagation_matches_constant_rate_los():
    """THE PHASE-D PROPERTY: during the coast, lambda advances at the FROZEN
    rate from the last-good snapshot (a constant-LOS-rate propagation), the
    range dead-reckons down at the frozen Vc, and the lateral accel is the
    frozen PN command N*Vc*lambda_dot -- constant, preserving the commanded
    ZEM correction through the blind window."""
    tc = TerminalCoast(n_gain=N, arm_tau_s=0.35, max_coast_s=0.25)
    _tracked(tc, 10.0, tau=0.2)                      # snapshot at t=10.0
    assert tc.on_track_loss(10.05)                   # loss detected 50 ms later
    cmd = tc.command(10.15)
    assert isinstance(cmd, CoastCommand)
    # lambda propagates from the SNAPSHOT time (covers the loss-detection gap):
    assert abs(cmd.lambda_rad - (0.3 + 2.0 * 0.15)) < 1e-12
    assert cmd.lambda_dot_rad_s == 2.0 and cmd.vc_m_s == 5.0
    assert abs(cmd.a_lat_m_s2 - N * 5.0 * 2.0) < 1e-12
    assert abs(cmd.r_m - (1.0 - 5.0 * 0.15)) < 1e-12
    assert abs(cmd.coast_elapsed_s - 0.10) < 1e-12
    assert not cmd.expired
    # Constant-velocity LOS across the window: two samples differ by exactly
    # lambda_dot * dt, and the accel command is IDENTICAL (frozen ZEM).
    cmd2 = tc.command(10.25)
    assert abs((cmd2.lambda_rad - cmd.lambda_rad) - 2.0 * 0.10) < 1e-12
    assert cmd2.a_lat_m_s2 == cmd.a_lat_m_s2


def test_expiry_is_sim_time_bounded():
    """The coast is only trusted for max_coast_s of SIM time (measured blind
    window 0.1-0.2 s); past that the command flags expired and the caller's
    policy takes over."""
    tc = TerminalCoast(n_gain=N, arm_tau_s=0.35, max_coast_s=0.25)
    _tracked(tc, 10.0, tau=0.2)
    tc.on_track_loss(10.02)
    assert not tc.command(10.22).expired             # 0.20 s of coast
    assert tc.command(10.30).expired                 # 0.28 s > 0.25 s
    # Expired still reports the frozen command (caller decides what to do).
    assert tc.command(10.30).a_lat_m_s2 == N * 5.0 * 2.0


def test_reacquire_cancels_coast():
    """A fresh track tick cancels the coast (closed loop resumes) and
    re-snapshots -- a later loss coasts from the NEW state."""
    tc = TerminalCoast(n_gain=N, arm_tau_s=0.35)
    _tracked(tc, 10.0, tau=0.2)
    tc.on_track_loss(10.05)
    assert tc.coasting
    tc.update_track(0.5, -1.0, 6.0, 10.20, r_hat_m=0.8, tau_s=0.15)
    assert not tc.coasting and tc.command(10.21) is None
    assert tc.on_track_loss(10.25)
    cmd = tc.command(10.30)
    assert abs(cmd.a_lat_m_s2 - N * 6.0 * -1.0) < 1e-12
    assert abs(cmd.lambda_rad - (0.5 + -1.0 * 0.10)) < 1e-12


def test_partial_track_update_ignored():
    """A tick with a missing filter state (e.g. rate not yet initialized)
    must not corrupt the snapshot."""
    tc = TerminalCoast(n_gain=N, arm_tau_s=0.35)
    _tracked(tc, 10.0, tau=0.2)
    tc.update_track(0.9, None, 5.0, 10.1, tau_s=0.1)   # no rate -> ignored
    tc.on_track_loss(10.15)
    cmd = tc.command(10.15)
    assert abs(cmd.lambda_rad - (0.3 + 2.0 * 0.15)) < 1e-12   # old snapshot


def test_reset_clears_everything():
    tc = TerminalCoast(n_gain=N, arm_tau_s=0.35)
    _tracked(tc, 10.0, tau=0.2)
    tc.on_track_loss(10.05)
    tc.reset()
    assert not tc.coasting and not tc.armed
    assert not tc.on_track_loss(10.1)


def test_arms_from_bbox_growth_tau():
    """Integration with Phase C: the tau that arms the coast comes from bbox
    growth (SizeRangeEstimator.tau_s). A constant-Vc closing geometry crosses
    the arm threshold as true time-to-contact R/Vc shrinks."""
    fx, w_span = 600.0, 0.30
    est = SizeRangeEstimator(fx, w_span)
    tc = TerminalCoast(n_gain=N, arm_tau_s=0.35)
    r0, vc, hz = 6.0, 4.0, 30.0
    dt = 1.0 / hz
    t, armed_at_true_tau = 0.0, None
    while r0 - vc * t > 0.4:
        est.predict(dt)
        r = r0 - vc * t
        est.correct(fx * w_span / r, t)
        tc.update_track(0.3, 2.0, vc, t, r_hat_m=r, tau_s=est.tau_s)
        if tc.armed and armed_at_true_tau is None:
            armed_at_true_tau = r / vc
        t += dt
    assert armed_at_true_tau is not None               # it DID arm
    assert armed_at_true_tau < 0.8                     # ...in the endgame,
    assert tc.on_track_loss(t)                         # and the loss coasts.


ALL = [test_no_snapshot_no_coast, test_tau_arming_gates_engagement,
       test_unknown_tau_never_arms, test_noisy_tau_dip_keeps_arm,
       test_stale_tau_eventually_disarms,
       test_freeze_propagation_matches_constant_rate_los,
       test_expiry_is_sim_time_bounded, test_reacquire_cancels_coast,
       test_partial_track_update_ignored, test_reset_clears_everything,
       test_arms_from_bbox_growth_tau]

if __name__ == "__main__":
    import sys
    failed = 0
    for t in ALL:
        try:
            t()
            print(f"PASS {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {t.__name__}: {e}")
    print(f"{len(ALL) - failed}/{len(ALL)} terminal_coast tests passed")
    sys.exit(1 if failed else 0)
