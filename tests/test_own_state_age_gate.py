"""The own-state age gate must actually FIRE, and must not fire on healthy jitter.

WHY THIS FILE EXISTS. `GuidanceConfig.own_state_max_age_s` sat at `None` (OFF) for
months with a fully-implemented enforcement path behind it, waiting on bench brn-05
to measure the own-state latency that sizes it. brn-05 ran on 2026-07-29 (max
own_age 0.02 s over 300 setpoints) and the constant was set to 0.25 s (ADR-0093).

A declared-but-inert guard is exactly the defect class CLAUDE.md calls out -- "a fix
is not done until its EFFECT is observed end-to-end" -- and nothing in the suite
referenced this constant before today. So these tests pin the OBSERVABLE behaviour:
that a stale own-state is refused, that the refusal carries the right reason string,
and that the bound sits far enough above the measured bench maximum that ordinary
scheduling jitter cannot trip it.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from flight.deploy.seeker_loop import (  # noqa: E402
    GuidanceConfig,
    OwnState,
    own_state_status,
)

# The measured brn-05 maximum. Traceable to
# runs/brn05_bench/deploy_seeker_bench_20260729T230827Z.log ("max own_age 0.02s").
BRN05_MAX_OWN_AGE_S = 0.02


def _healthy(age_s):
    """An own-state with a real attitude solution, aged by `age_s`."""
    own = OwnState.level(alt_m=10.0)
    assert own.quat is not None and own.psi_rad is not None, \
        "OwnState.level must supply a full attitude, else this tests the wrong guard"
    own.age_s = age_s
    return own


def test_gate_is_armed_by_default():
    """The whole point of brn-05 was to turn this ON. A regression to None would
    silently disable the guard while every other test still passed."""
    assert GuidanceConfig().own_state_max_age_s is not None, \
        "own_state_max_age_s is back to None -- the age gate is disabled"


def test_stale_own_state_is_refused_with_its_reason():
    cfg = GuidanceConfig()
    ok, reason = own_state_status(_healthy(cfg.own_state_max_age_s + 0.05), cfg)
    assert ok is False
    assert reason == "own_state_stale"


def test_fresh_own_state_passes():
    cfg = GuidanceConfig()
    ok, reason = own_state_status(_healthy(BRN05_MAX_OWN_AGE_S), cfg)
    assert (ok, reason) == (True, "ok")


def test_the_measured_bench_maximum_is_nowhere_near_the_bound():
    """Guards the SIZING, not just the mechanism. If someone later tightens this
    toward the idle-bench number, guidance starts refusing to steer on healthy
    flight jitter -- and the failure mode is silent (it looks like a perception
    dropout). Requires >=5x headroom over the measured max."""
    cfg = GuidanceConfig()
    assert cfg.own_state_max_age_s >= 5 * BRN05_MAX_OWN_AGE_S, (
        f"own_state_max_age_s={cfg.own_state_max_age_s}s leaves under 5x headroom "
        f"over the measured bench max {BRN05_MAX_OWN_AGE_S}s"
    )


def test_bound_is_under_px4s_offboard_loss_timeout():
    """COM_OF_LOSS_T defaults to 1.0 s; PX4 drops OFFBOARD past it. A staleness
    bound above that would only ever be reached after PX4 had already given up."""
    cfg = GuidanceConfig()
    assert cfg.own_state_max_age_s < 1.0


@pytest.mark.parametrize("age_s", [0.0, 0.01, 0.02, 0.05, 0.1, 0.24])
def test_every_plausible_healthy_age_passes(age_s):
    cfg = GuidanceConfig()
    assert own_state_status(_healthy(age_s), cfg)[0] is True, \
        f"age {age_s}s should be healthy but was refused"


def test_exactly_at_the_bound_is_allowed_boundary_is_strict_greater():
    """Pins the comparison as `>` not `>=`, so the documented value is the last
    ACCEPTED age rather than the first rejected one."""
    cfg = GuidanceConfig()
    assert own_state_status(_healthy(cfg.own_state_max_age_s), cfg)[0] is True


def test_unknown_age_does_not_trip_the_gate():
    """age_s None means the age is UNKNOWN, not stale. Refusing here would ground
    guidance on any telemetry path that omits the timestamp; the missing-attitude
    guards above are what cover the genuinely unusable cases."""
    cfg = GuidanceConfig()
    own = _healthy(0.0)
    own.age_s = None
    assert own_state_status(own, cfg) == (True, "ok")


def test_gate_can_still_be_disabled_deliberately():
    """None must remain a supported value -- it is how a bench experiment opts out."""
    cfg = GuidanceConfig(own_state_max_age_s=None)
    assert own_state_status(_healthy(99.0), cfg) == (True, "ok")


def test_missing_attitude_outranks_staleness():
    """A fresh own-state with no quaternion is still refused, and the reason must
    name the attitude -- not the age -- so a log reader diagnoses the right fault."""
    cfg = GuidanceConfig()
    own = _healthy(0.0)
    own.quat = None
    ok, reason = own_state_status(own, cfg)
    assert ok is False
    assert reason == "no_own_attitude_quat"
