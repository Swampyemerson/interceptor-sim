"""flight.terminal_coast -- Phase D of the intercept-accuracy program
(docs/intercept_accuracy_levers.md): ENDGAME. When the camera track dies
inside the sub-1 m blind window (measured: camera loss inside ~1 m on 20/20
flights, dropout ~0.1-0.2 s), FREEZE the last good LOS-rate and propagate
the endgame open-loop on the own-state EKF, armed by a TAU (time-to-contact)
clock -- never wall-clock.

STATUS: PRE-CODED, OPT-IN, NOT SIM-VALIDATED. Default-off behind
--terminal-coast (wiring: docs/phase_bcd_wiring.md). NAME COLLISIONS,
deliberately distinct mechanisms -- do not conflate:
  * m4's existing --terminal-coast-gate = a PHANTOM-REJECTION range gate
    (ADR-0057), not an endgame coast.
  * m4's existing inline "terminal coast" (frozen_vworld at
    TERMINAL_FREEZE_RANGE_M) freezes the VELOCITY VECTOR -- it holds the
    last command and DROPS any further correction.
This module is the levers-doc alternative to that hold-last-command block:
freeze the LOS-RATE and keep INTEGRATING the PN command through the window.
The decisive Phase D A/B is exactly freeze-and-propagate (this) vs
hold-last-command (the existing block), same flights, compare CPA.

THE PHYSICS (why freeze the rate, not the command state):
  * Terminal LOS rates are measured at 485-1870 deg/s. A STALE LOS ANGLE
    (or a velocity vector re-projected onto a stale angle) diverges within
    a frame or two -- pointing the correction along yesterday's geometry.
  * The PN command a = N*Vc*lambda_dot IS the zero-effort-miss correction:
    a_PN ~ N*ZEM/t_go^2. Freezing lambda_dot (and Vc) at the last good
    estimate and continuing to integrate that CONSTANT lateral acceleration
    holds the commanded ZEM correction through impact -- open-loop
    completion of the correction the tracker had already committed to,
    which is the best available policy once the camera is blind.
  * lambda itself is propagated at the frozen rate (lambda(t) = lambda_0 +
    lambda_dot_0 * (t - t_0)) so the caller keeps applying the lateral
    accel PERPENDICULAR to a coherently-advancing LOS, and range at
    R(t) = R_0 - Vc_0*(t - t_0) (>= 0) for logging/breakoff bookkeeping.
    Both propagate on the own-state basis (sim/EKF clock + frozen filter
    states); nothing new is sensed.
  * TAU ARMING: the coast must only engage in the true endgame, and "1 m"
    is exactly what the (noisy, monocular) range channel is worst at. Tau
    from bbox growth (tau = size/sizedot, flight.range_fusion.tau_s) is the
    camera's own optical time-to-contact -- scale-free, no metric range
    needed. Track loss with tau <= arm threshold => endgame blind window =>
    coast. Track loss when NOT armed is an ordinary mid-course dropout and
    stays the estimator's predict() job -- this module does nothing there.
  * EXPIRY: the measured blind window is ~0.1-0.2 s; the coast is only
    trusted for max_coast_s (default 0.25 s) of SIM time. Past that the
    command is flagged expired=True -- the caller decides policy (hold /
    breakoff); this module only reports.

NO WALL-CLOCK (ADR-0009, standing rule): every `t` in this module is SIM
seconds from the caller's sim clock. Arming is by tau (measurement
geometry), duration by sim-time elapsed -- an RTF sag cannot stretch or
truncate the coast.

HONESTY BOUNDARY (ADR-0008/0010): inputs are the guidance loop's own filter
states (lambda, lambda_dot, Vc from camera-only filters), a camera-derived
tau, and the sim/EKF clock. No gt_*, no datalink. The coast introduces no
new sensing path, but the tau feed rides the bbox pixel path (Phase C), so
the combined wiring RE-EARNS the numeric no-cheat audit.
"""

from dataclasses import dataclass
from typing import Optional

# Defaults -- STARTING POINTS for the Phase D A/B, not validated numbers.
ARM_TAU_S_DEFAULT = 0.35      # arm inside ~1 m at ~3-6 m/s closing (R/Vc)
MAX_COAST_S_DEFAULT = 0.25    # measured blind window 0.1-0.2 s + margin
TAU_STALE_S_DEFAULT = 0.30    # how long a valid tau keeps the arm alive (below)


@dataclass
class CoastCommand:
    """One coasting tick's frozen-and-propagated guidance state.

    a_lat_m_s2: N * Vc_0 * lambda_dot_0 -- CONSTANT through the coast (the
        frozen commanded-ZEM correction).
    lambda_rad: LOS azimuth propagated at the frozen rate (apply a_lat
        perpendicular to THIS, not to the stale last-seen lambda).
    lambda_dot_rad_s / vc_m_s: the frozen states (logging/diagnostics).
    r_m: dead-reckoned range R_0 - Vc_0*dt_coast, clamped >= 0 (None if the
        caller never supplied a range).
    coast_elapsed_s: sim seconds since the coast engaged.
    expired: True once coast_elapsed_s > max_coast_s -- the command is no
        longer trusted; caller policy takes over.
    """

    a_lat_m_s2: float
    lambda_rad: float
    lambda_dot_rad_s: float
    vc_m_s: float
    r_m: Optional[float]
    coast_elapsed_s: float
    expired: bool


class TerminalCoast:
    """Tau-armed freeze-and-propagate endgame coast.

    Protocol (all times SIM seconds):
      * every tick WITH a live track:  update_track(lam, lamdot, vc, t,
        r_hat=..., tau_s=...) -- snapshots last-good state, re-arms/disarms
        on tau, and CANCELS any active coast (reacquire).
      * on a tick with track loss:     on_track_loss(t) -> bool (True =
        coast engaged; False = not armed / nothing snapshotted -> caller
        falls through to its existing dropout handling).
      * while coasting:                command(t) -> CoastCommand.
      * reset() between engagements/flights.
    """

    def __init__(self, n_gain: float, arm_tau_s: float = ARM_TAU_S_DEFAULT,
                 max_coast_s: float = MAX_COAST_S_DEFAULT,
                 tau_stale_s: float = TAU_STALE_S_DEFAULT):
        self.n_gain = float(n_gain)
        self.arm_tau_s = float(arm_tau_s)
        self.max_coast_s = float(max_coast_s)
        self.tau_stale_s = float(tau_stale_s)
        self.reset()

    def reset(self) -> None:
        self._lam = None          # last-good LOS azimuth (rad)
        self._lamdot = None       # last-good LOS rate (rad/s)
        self._vc = None           # last-good closing speed (m/s)
        self._r = None            # last-good range (m) or None
        self._tau = None          # last VALID tau reading (s) or None
        self._tau_t = None        # sim time of the last valid tau
        self._t_snapshot = None   # sim time of the last-good snapshot
        self._t_coast_start = None  # sim time the coast engaged (None = not coasting)

    # -- state queries -------------------------------------------------------
    @property
    def armed(self) -> bool:
        """Tau clock says endgame: the last VALID tau was <= arm threshold AND
        is not stale. STICKY (review finding 4): a single noisy wdot dip that
        makes tau_s None on the final live tick must NOT disarm at the exact
        moment the coast is needed -- update_track keeps the last valid tau,
        and it stays arming for tau_stale_s of sim time (long enough to cover
        the loss-detection gap, short enough that an old opening geometry
        can't keep it armed forever)."""
        if self._tau is None or self._tau > self.arm_tau_s:
            return False
        if self._tau_t is not None and self._t_snapshot is not None:
            return (self._t_snapshot - self._tau_t) <= self.tau_stale_s
        return True

    @property
    def coasting(self) -> bool:
        return self._t_coast_start is not None

    # -- protocol ------------------------------------------------------------
    def update_track(self, lambda_rad: float, lambda_dot_rad_s: float,
                     vc_m_s: float, t_sim: float, r_hat_m=None,
                     tau_s=None) -> None:
        """Live-track tick: snapshot the states the coast would freeze, feed
        the tau clock, and cancel any active coast (the camera is back --
        closed-loop guidance resumes; the caller's normal law takes over)."""
        if lambda_rad is None or lambda_dot_rad_s is None or vc_m_s is None:
            return
        self._lam = float(lambda_rad)
        self._lamdot = float(lambda_dot_rad_s)
        self._vc = float(vc_m_s)
        self._r = None if r_hat_m is None else float(r_hat_m)
        # KEEP the last VALID tau (finding 4): a None reading (noisy wdot dip)
        # does not clobber a recent good one; staleness is handled in `armed`.
        if tau_s is not None:
            self._tau = float(tau_s)
            self._tau_t = float(t_sim)
        self._t_snapshot = float(t_sim)
        self._t_coast_start = None  # reacquire cancels the coast

    def on_track_loss(self, t_sim: float) -> bool:
        """Track-loss tick: engage the coast iff the tau clock is armed and a
        snapshot exists. Returns True when (now) coasting. A loss while NOT
        armed is a mid-course dropout -- not this module's job; returns
        False and the caller's existing dropout/predict path runs."""
        if self.coasting:
            return True
        if not self.armed or self._lam is None:
            return False
        self._t_coast_start = float(t_sim)
        return True

    def command(self, t_sim: float) -> Optional[CoastCommand]:
        """The frozen-and-propagated command at sim time t_sim (None if not
        coasting). Propagation runs from the LAST-GOOD SNAPSHOT time, so the
        gap between the final good frame and loss detection is covered too."""
        if not self.coasting:
            return None
        dt = max(0.0, t_sim - self._t_snapshot)
        dt_coast = max(0.0, t_sim - self._t_coast_start)
        lam = self._lam + self._lamdot * dt
        r = None if self._r is None else max(0.0, self._r - self._vc * dt)
        return CoastCommand(
            a_lat_m_s2=self.n_gain * self._vc * self._lamdot,
            lambda_rad=lam,
            lambda_dot_rad_s=self._lamdot,
            vc_m_s=self._vc,
            r_m=r,
            coast_elapsed_s=dt_coast,
            expired=dt_coast > self.max_coast_s,
        )
