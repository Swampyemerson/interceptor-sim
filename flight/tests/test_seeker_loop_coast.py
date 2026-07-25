"""flight/tests/test_seeker_loop_coast.py -- the deployed terminal's SILENT-FAILURE
guards (review2 code-first audit, 2026-07-25).

WHY THIS FILE EXISTS. `flight/deploy/seeker_loop.SeekerGuidance` is the terminal
that flies the real interceptor (real_flight.py composes it), and until today NO
test in the repo imported it: its only checks were its own `--self-test` and the
SITL smoke, both of which assert finiteness / non-zero speed / states-visited --
never a DIRECTION and never a health condition. Three defects lived behind that:

  1. ONE oversized detector box + a short dropout latched the terminal-coast
     velocity freeze PERMANENTLY. The camera was disconnected for the rest of the
     engagement; every logged value stayed finite and plausible.
  2. A detection dropout made the yaw command chase the vehicle it was steering
     (measured 90 deg of uncommanded yaw in 2.0 s at a 45 deg/s yaw loop).
  3. With the own-state EKF absent, guidance silently steered on the DESK
     stand-in (level attitude, yaw 0 = "the nose points north").

Every test below FAILS on the pre-fix code (verified by running it against
`git show HEAD~:flight/deploy/seeker_loop.py`) and PASSES on the fix, and the
first test pins the NOMINAL path byte-identical so the guards cost nothing when
nothing is wrong.

No sim, no MAVSDK, no weights, no camera: SeekerGuidance is a pure step function.
"""
import math
import os
import sys

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from flight.camera import CameraModel  # noqa: E402
from flight.deploy.seeker_loop import (  # noqa: E402
    GuidanceConfig,
    OwnState,
    SeekerGuidance,
    own_state_status,
)

FX = FY = 539.936
CX, CY = 640.0, 480.0
DT = 0.05                      # 20 Hz control loop, as flown
IDENT = (1.0, 0.0, 0.0, 0.0)   # level attitude


def guidance(**cfgkw):
    cfg = GuidanceConfig(**cfgkw)
    return SeekerGuidance(cfg, CameraModel(FX, FY, CX, CY), span_m=1.0), cfg


def box(u=CX, w=45.0):
    """A square detector box centred at pixel `u`. range = fx*span/w, so
    w=45 px -> 12.00 m, w=135 px -> 4.00 m, w=154 px -> 3.49 m."""
    return (u - w / 2.0, CY - w / 2.0, w, w)


def w_for_range(r):
    return FX / float(r)


def level(alt=5.0):
    return OwnState.level(alt)


def drive(g, boxes, own=None, dt=DT):
    """Step a guidance object over a box sequence; returns the telemetry list."""
    own = own or level(g.cfg.alt_ref_m)
    return [g.step(b, own, dt * k)[1] for k, b in enumerate(boxes)]


# ============================================================ nominal identity


def test_the_guards_are_a_no_op_on_a_clean_closing_run():
    """DEFAULT-SAFE: with a detection on every tick and a sane range channel the
    fixed code must produce the SAME setpoints as the pre-fix law. Pinned as an
    absolute expectation (not a diff against the old file) so it survives.

    The values are the pro-nav law itself: v_close from compute_v_close, the LOS
    basis (cos/sin lambda), v_perp integrated from a_cmd = N*Vc*lambda_dot."""
    g, cfg = guidance()
    boxes = [box(CX + 6.0 * k, 45.0) for k in range(30)]     # 12 m, walking right
    own = level(cfg.alt_ref_m)
    sps = [g.step(b, own, DT * k)[0] for k, b in enumerate(boxes)]
    assert all(s is not None for s in sps[1:])
    last = sps[-1]
    # No guard may have fired on a clean run.
    tel = g.step(boxes[-1], own, DT * len(boxes))[1]
    assert tel.health == [], f"a clean tick raised {tel.health}"
    assert tel.terminal_coast is False and tel.stale is False
    assert tel.own_state_ok is True and tel.track_broken is False
    # Direction: a target right of boresight steers EAST and yaws positive.
    assert last.v_east > 0 and last.yaw_deg > 0
    assert math.hypot(last.v_north, last.v_east) <= cfg.v_total_max + 1e-9


def test_a_mirrored_target_mirrors_the_command():
    """Sign anchor: the whole silent-failure class includes a frame/sign flip
    that yields a perfectly plausible setpoint pointing the WRONG way."""
    gr, _ = guidance()
    gl, _ = guidance()
    own = level()
    sr = gr.step(box(900.0), own, 0.0)[0]
    sl = gl.step(box(2 * CX - 900.0), own, 0.0)[0]
    assert sr.v_east > 0 > sl.v_east
    assert sr.yaw_deg > 0 > sl.yaw_deg
    assert sr.v_east == pytest.approx(-sl.v_east, abs=1e-9)
    assert sr.v_north == pytest.approx(sl.v_north, abs=1e-9)


# ============================================================ (1) the coast latch


def test_a_genuine_close_still_latches_the_terminal_freeze():
    """The latch is the VALIDATED mechanism (ADR-0023) -- the fix must not cost
    it. A monotone close through 3.5 m still arms, within one detector tick of
    the pre-fix behaviour, and then HOLDS (no un-freezing: split-freeze measured
    worse in Gazebo, +0.084 m)."""
    g, cfg = guidance()
    boxes = [box(CX + 2.0 * k, 30.0 + 8.0 * k) for k in range(46)]   # 18 m -> 1.3 m
    tels = drive(g, boxes)
    armed = [i for i, t in enumerate(tels) if t.terminal_coast]
    assert armed, "a genuine monotone close must still arm the freeze"
    i0 = armed[0]
    assert tels[i0].r_hat_m < cfg.terminal_freeze_range_m
    assert tels[i0].r_hat_m > cfg.terminal_freeze_range_m - 0.5, (
        f"the freeze armed at {tels[i0].r_hat_m:.2f} m -- the corroboration "
        f"requirement must cost at most ~one detector tick of range")
    # ...and once armed it HOLDS to the end of the run.
    assert all(t.terminal_coast for t in tels[i0:])
    assert g.n_coast_releases == 0
    # The frozen NED vector is literally constant.
    v = {(round(t.setpoint.v_north, 9), round(t.setpoint.v_east, 9))
         for t in tels[i0:]}
    assert len(v) == 1


def test_one_oversized_box_plus_a_dropout_cannot_latch_the_freeze():
    """THE BLOCKER. A 3x-oversized/merged box at 12 m reads 4.0 m and injects
    rdot_hat ~ -73 m/s into the (deliberately rate-cap-free) range channel; the
    next few dark ticks let predict() coast r_hat straight through the 3.5 m
    freeze range. Pre-fix that armed the latch FOREVER -- 42 of the following 50
    ticks flew a frozen vector while the target sat at a true 12 m and the
    detector had already recovered.

    Post-fix the latch may only arm off FRESHLY-CORRECTED ticks (m4 parity) with
    corroboration, so the dropout ticks cannot arm it and the recovered
    detections (r_hat ~ 12 m) never satisfy it."""
    g, cfg = guidance()
    boxes = ([box(CX, 45.0)] * 6              # clean, 12.0 m
             + [box(CX, 135.0)]               # ONE 3x box -> reads 4.0 m
             + [None] * 3                     # dropout: predict() coasts r_hat
             + [box(CX, 45.0)] * 40)          # detector recovers, target at 12 m
    tels = drive(g, boxes)
    assert not any(t.terminal_coast for t in tels), (
        "one oversized box + a dropout latched the terminal-coast freeze")
    assert tels[-1].r_hat_m == pytest.approx(12.10, abs=0.5)
    # ...and the event is not silent.
    assert any("range_track_broken" in t.health for t in tels)
    assert g.n_track_broken > 0


def test_the_range_channel_diverging_raises_a_health_flag_not_a_coast():
    """r_hat <= 0 / a non-physical |rdot_hat| is proof the range estimate is not
    the target any more. It must raise, never silently select the coast branch
    (real_flight FAILSAFE 8 breaks off on a persistent one)."""
    g, cfg = guidance()
    tels = drive(g, [box(CX, 45.0)] * 4 + [box(CX, 400.0)] + [None] * 6)
    assert any(t.track_broken for t in tels)
    broken = [t for t in tels if t.track_broken][0]
    assert "range_track_broken" in broken.health
    bound = 2.0 * cfg.v_total_max
    assert abs(broken.rdot_hat_m_s) > bound or broken.r_hat_m <= 0.0


def test_a_spurious_latch_releases_on_corroborated_far_detections():
    """RELEASE ARM 1: while frozen, N consecutive FRESH detections back out at
    > 2x the freeze range are proof the arming was spurious -- a real 3.5 m
    coast reaches CPA in well under a second and can never see them."""
    g, cfg = guidance(coast_arm_ticks=1)   # force an easy (spurious) arm
    own = level(cfg.alt_ref_m)
    # Arm it: two frames that read ~2 m.
    seq = [box(CX, w_for_range(2.0))] * 2
    tels = [g.step(b, own, DT * k)[1] for k, b in enumerate(seq)]
    assert tels[-1].terminal_coast, "setup: the latch must be armed"
    # Now the truth: the target is at 20 m and the detector says so, repeatedly.
    far = [box(CX, w_for_range(20.0))] * 12
    t2 = [g.step(b, own, DT * (len(seq) + k))[1] for k, b in enumerate(far)]
    assert not t2[-1].terminal_coast, "the spurious latch never released"
    assert g.n_coast_releases == 1
    assert any("coast_released_range" in t.health for t in t2)


def test_a_latch_that_outlives_the_physics_releases_on_timeout():
    """RELEASE ARM 2 (the backstop that needs no detections at all): a freeze
    armed at 3.5 m closing at >= 5.5 m/s reaches CPA in ~0.6 s. Still coasting
    after `coast_max_s` = 3.0 s is proof, whatever the detector is doing."""
    g, cfg = guidance(coast_arm_ticks=1)
    own = level(cfg.alt_ref_m)
    seq = [box(CX, w_for_range(2.0))] * 2
    for k, b in enumerate(seq):
        g.step(b, own, DT * k)
    assert g._frozen_vworld is not None
    tels = [g.step(None, own, DT * (len(seq) + k))[1] for k in range(80)]  # 4.0 s
    assert any("coast_released_timeout" in t.health for t in tels)
    assert g.n_coast_releases == 1
    fired = [t for t in tels if "coast_released_timeout" in t.health][0]
    assert fired.coast_age_s > cfg.coast_max_s


def test_the_release_can_be_disabled_and_then_the_latch_is_forever():
    """The release is a CONFIG, not a hidden behaviour: with both arms off the
    pre-fix latch-forever semantics are exactly recoverable (for a deliberate
    A/B against the ADR-0023 baseline)."""
    g, cfg = guidance(coast_arm_ticks=1, coast_release_range_m=None,
                      coast_max_s=None)
    own = level(cfg.alt_ref_m)
    for k, b in enumerate([box(CX, w_for_range(2.0))] * 2):
        g.step(b, own, DT * k)
    tels = [g.step(box(CX, w_for_range(20.0)), own, DT * (2 + k))[1]
            for k in range(60)]
    assert all(t.terminal_coast for t in tels)
    assert g.n_coast_releases == 0


# ============================================================ (2) the yaw chase


def _closed_loop_yaw(g, n_det, n_dark, yaw_rate_deg_s=45.0, u0=615.0, du=0.0):
    """Run the terminal against a vehicle that SLEWS toward the commanded yaw --
    the closed loop is what turns the pre-fix expression into a runaway."""
    psi = 0.0
    out = []
    for k in range(n_det + n_dark):
        b = box(u0 + du * k) if k < n_det else None
        own = OwnState(quat=IDENT, psi_rad=math.radians(psi), alt_m=5.0,
                       age_s=0.0)
        sp, tel = g.step(b, own, DT * k)
        if sp is None:
            out.append((None, tel))
            continue
        err = sp.yaw_deg - psi
        psi += max(-yaw_rate_deg_s * DT, min(yaw_rate_deg_s * DT, err))
        out.append((sp.yaw_deg, tel))
    return out


def test_a_dropout_cannot_make_the_yaw_command_chase_itself():
    """THE BLOCKER. Pre-fix, every dark tick recomputed yaw = psi(NOW) + <stale
    bearing>; as the vehicle slewed toward the command, psi grew and the command
    grew with it -- a positive-feedback integrator that spun the aircraft at the
    yaw-loop rate for the whole dropout (measured 90 deg in 2.0 s here, 108 deg
    in the reviewer's geometry), rotating the body-fixed camera off the target so
    the loss became self-sustaining.

    Post-fix the absolute command is LATCHED and coasted on the ESTIMATED LOS
    rate, so its excursion is bounded by |lambda_dot| * dt_dark -- the
    constant-LOS-rate doctrine, and zero when the LOS is not moving."""
    g, _ = guidance()
    trace = _closed_loop_yaw(g, n_det=10, n_dark=40)      # 2.0 s dark
    yaw_last_det = trace[9][0]
    lam_dot = abs(trace[9][1].lambda_dot_deg_s or 0.0)
    for k in range(10, len(trace)):
        yaw, tel = trace[k]
        dark_s = (k - 9) * DT
        bound = lam_dot * dark_s + 1e-6
        assert abs(yaw - yaw_last_det) <= bound, (
            f"tick {k}: yaw command ran {abs(yaw - yaw_last_det):.2f} deg in "
            f"{dark_s:.2f} s of dark, bound {bound:.2f} deg")
        assert tel.yaw_hold is True
    # A static LOS => the command simply holds.
    assert trace[-1][0] == pytest.approx(yaw_last_det, abs=1e-9)


def test_a_crossing_los_coasts_the_yaw_at_the_estimated_rate_not_the_loop_rate():
    """The bound must be the ESTIMATED LOS rate, not the un-slewed pointing
    error: on a 37 deg/s crossing the pre-fix command swung 90 deg (the yaw
    loop's own 45 deg/s x 2 s), the fixed one swings exactly |lambda_dot| * T."""
    g, _ = guidance()
    trace = _closed_loop_yaw(g, n_det=12, n_dark=40, du=20.0)
    i = 11
    lam_dot = trace[i][1].lambda_dot_deg_s
    assert abs(lam_dot) > 10.0, "setup: the LOS must actually be crossing"
    T = (len(trace) - 1 - i) * DT
    swing = trace[-1][0] - trace[i][0]
    assert swing == pytest.approx(lam_dot * T, abs=1e-6)
    assert abs(swing) < 45.0 * T, "the fix must beat the yaw-loop-rate runaway"


def test_the_yaw_command_is_unchanged_on_every_detected_tick():
    """Byte-identity where it matters: with a detection on the tick the command
    is still exactly psi + bearing."""
    g, _ = guidance()
    for k in range(12):
        psi = math.radians(3.0 * k)
        own = OwnState(quat=IDENT, psi_rad=psi, alt_m=5.0, age_s=0.0)
        sp, tel = g.step(box(700.0), own, DT * k)
        want = math.degrees(psi) + math.degrees(g._last_bearing_rad)
        assert sp.yaw_deg == pytest.approx(want, abs=1e-12)
        assert tel.yaw_hold is False


def test_v_perp_stops_integrating_once_the_los_rate_is_stale():
    """A stale rate is an extrapolation. Pre-fix the pro-nav acceleration kept
    integrating into v_perp through a dropout and walked it to the +-8 m/s
    clamp; post-fix v_perp HOLDS and the tick is flagged."""
    g, cfg = guidance()
    own = level(cfg.alt_ref_m)
    # A GENTLE crossing on purpose: v_perp must end strictly INSIDE the +-8 m/s
    # clamp, or "it did not move" would be true of the buggy code too (a
    # saturated v_perp is a test that cannot fail).
    det = [g.step(box(615.0 + 4.0 * k), own, DT * k)[1] for k in range(12)]
    v_at_loss = det[-1].v_perp
    assert 0.1 < abs(v_at_loss) < cfg.v_perp_max - 1.0, (
        f"setup: v_perp {v_at_loss:.2f} must be non-zero and unsaturated")
    dark = [g.step(None, own, DT * (12 + k))[1] for k in range(40)]
    # Ticks inside v_perp_stale_s still integrate ON PURPOSE (a 0.05-0.20 s gap
    # is normal cadence at a ~14 Hz detector on a 20 Hz loop). From the FIRST
    # stale tick onward v_perp must be frozen, exactly.
    first_stale = next(i for i, t in enumerate(dark) if t.stale)
    assert dark[first_stale].meas_age_s > cfg.v_perp_stale_s
    frozen = dark[first_stale].v_perp
    assert all(t.v_perp == pytest.approx(frozen, abs=1e-12)
               for t in dark[first_stale:]), "v_perp kept integrating on a stale rate"
    assert all("v_perp_hold_stale" in t.health for t in dark[first_stale:])
    # ...and it never reaches the clamp, which is where unguarded integration
    # of a 2 s extrapolation puts it (measured 1.71 -> 8.00 m/s pre-fix).
    assert abs(frozen) < cfg.v_perp_max - 1.0
    # The guard is silent while the detector is keeping up.
    assert not any(t.stale for t in det)


# ============================================================ (3) own-state


def test_guidance_refuses_to_steer_on_the_level_yaw0_desk_stand_in():
    """THE HIGH. With quat/psi absent, derotate_bearing_lambda falls back to the
    pre-fix yaw-only formula on an assumed-LEVEL, assumed-NORTH vehicle -- the
    exact bug flight/geometry.py calls 'THE BUG THIS FIXES' -- and the resulting
    setpoint is finite, plausible, and unmarked. Refuse it instead."""
    g, cfg = guidance()
    for k in range(6):
        sp, tel = g.step(box(900.0), OwnState(quat=None, psi_rad=None, alt_m=5.0),
                         DT * k)
        assert sp is None, "guidance steered on the desk stand-in"
        assert tel.own_state_ok is False
        assert "no_own_attitude_quat" in tel.health
    # CRITICAL: the tracker must not have INITIALISED off a stand-in lambda,
    # or the first real measurement arrives as a huge fake LOS rate.
    assert g.lam.x_hat is None and g.rng.x_hat is None
    # A missing YAW alone is equally disqualifying (psi None == "nose is north").
    sp, tel = g.step(box(900.0), OwnState(quat=(1.0, 0.0, 0.0, 0.0),
                                          psi_rad=None, alt_m=5.0), 1.0)
    assert sp is None and "no_own_yaw" in tel.health


def test_a_stale_own_state_is_refused_once_a_threshold_is_configured():
    """The AGE gate is off by default on purpose -- no measured own-state stall
    latency exists yet (bench brn-05 produces it from the logged own_age_s
    column). Once set, a stale attitude is refused exactly like a missing one."""
    g, cfg = guidance(own_state_max_age_s=0.25)
    fresh = OwnState(quat=IDENT, psi_rad=0.0, alt_m=5.0, age_s=0.05)
    stale = OwnState(quat=IDENT, psi_rad=0.0, alt_m=5.0, age_s=1.50)
    assert g.step(box(900.0), fresh, 0.0)[0] is not None
    sp, tel = g.step(box(900.0), stale, DT)
    assert sp is None and "own_state_stale" in tel.health
    # Default config: the same stale sample is accepted (and still REPORTED).
    g2, _ = guidance()
    sp2, tel2 = g2.step(box(900.0), stale, 0.0)
    assert sp2 is not None and tel2.own_age_s == 1.50


def test_own_state_status_is_pure_and_opt_outable():
    ok, why = own_state_status(OwnState.level(5.0), GuidanceConfig())
    assert ok and why == "ok"
    bad = OwnState(quat=None, psi_rad=None, alt_m=None)
    assert own_state_status(bad, GuidanceConfig()) == (False, "no_own_attitude_quat")
    # The permissive pre-fix behaviour stays reachable for a deliberate bench run.
    assert own_state_status(bad, GuidanceConfig(require_own_attitude=False))[0]


def test_an_uncalibrated_range_span_is_reported_as_uncalibrated():
    """review2 HIGH: resolve_span used to return only (span, source_string), so
    the DEFAULT hardware weights -- which have no `<weights>.calib.json` sidecar,
    unlike the retired sim models -- silently produced range = fx*1.0/box_width,
    a constant fitted to the SIM quad, printed as a reassuring
    'span=1.000 m (config default)'. span is not a display value: it feeds every
    range-keyed threshold AND multiplies Vc into a_cmd = N*Vc*lambda_dot."""
    from flight.deploy.seeker_loop import resolve_span, warn_uncalibrated_span
    cfg = GuidanceConfig()
    nn = os.path.join(_REPO_ROOT, "scripts", "seeker", "weights", "nn_tier",
                      "n-mono.onnx")
    span, src, ok = resolve_span(nn, cfg)
    assert (span, ok) == (cfg.target_span_m, False)
    assert "NOT CALIBRATED" in src
    assert not os.path.exists(nn + ".calib.json"), (
        "a sidecar now exists -- flip --require-span-calib on by default and "
        "delete this reminder")
    # Explicit override and a real sidecar both count as calibrated.
    assert resolve_span(nn, cfg, 0.35) == (0.35, "--target-span-m 0.35", True)
    sim = os.path.join(_REPO_ROOT, "scripts", "seeker", "weights",
                       "drone_finetuned_v2.onnx")
    if os.path.exists(sim + ".calib.json"):
        s2, src2, ok2 = resolve_span(sim, cfg)
        assert ok2 and s2 != cfg.target_span_m and "calib" in src2
    # LIVE + uncalibrated is loud; offline is silent; --require makes it refuse.
    assert warn_uncalibrated_span(span, src, nn, cfg, live=False) == 0
    assert warn_uncalibrated_span(span, src, nn, cfg, live=True) == 0
    assert warn_uncalibrated_span(span, src, nn, cfg, live=True, require=True) == 1
    assert warn_uncalibrated_span(0.35, "--target-span-m 0.35", nn, cfg,
                                  live=True, require=True) == 0


def test_a_non_level_attitude_changes_lambda_so_the_derotation_is_anchored():
    """Every other fixture is identity-quat / psi 0, where the full-attitude
    derotation and the yaw-only fallback are EQUAL by construction -- so a
    regression that reverted the derotation would hide. Pin a real attitude."""
    from flight.geometry import euler_to_quat_body_to_ned
    q = euler_to_quat_body_to_ned(math.radians(15.0), math.radians(-30.0),
                                  math.radians(90.0))
    g_full, _ = guidance()
    g_flat, _ = guidance(require_own_attitude=False)
    _, t_full = g_full.step(box(900.0),
                            OwnState(quat=q, psi_rad=math.radians(90.0),
                                     alt_m=5.0, age_s=0.0), 0.0)
    _, t_flat = g_flat.step(box(900.0),
                            OwnState(quat=None, psi_rad=math.radians(90.0),
                                     alt_m=5.0), 0.0)
    assert abs(t_full.lambda_deg - t_flat.lambda_deg) > 0.2, (
        "full-attitude derotation collapsed to the yaw-only fallback")
