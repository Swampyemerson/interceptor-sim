#!/usr/bin/env python3
r"""T18 -- ground-side track filter: alpha-beta (g-h) position+velocity
filter over a stream of TRIANGULATED (x, y, z) positions (`triangulate.py`'s
output), emitting a filtered position + velocity estimate.

Reuses `scripts/guidance_lab.py`'s `AlphaBetaFilter` / `kalata_alpha_beta`
BYTE-FOR-BYTE (imported, not re-implemented) -- the same single-source
Kalata machinery `scripts/rig_geometry_analysis.py` already reuses for the
sigma_v-vs-rate analysis this module's validation harness checks against.
One `AlphaBetaFilter` instance per Cartesian axis (x, y, z); `m4_intercept.py`
/`guidance_lab.TargetTracker` only track (x, y) because M4's engagements are
fixed-altitude, but the ground rig's target can carry real altitude
variation, so this tracks all three axes.

KALATA MODE (the mode this task's validation harness uses): pass
`kalata_sigma_process` (target-maneuver-accel std, m/s^2) and
`kalata_sigma_meas` (position-measurement-noise std, m -- typically the
rig's sigma_R at the current range) to have EVERY axis recompute its
(alpha, beta) gains at each `correct()` from the ACTUAL elapsed time since
the last correction (`kalata_alpha_beta()`; see that function's docstring
in guidance_lab.py for why this is the principled response to irregular
update intervals). Pass fixed `alpha`/`beta` instead for the plain,
non-adaptive mode.

`update_rate_hz` is a DOCUMENTATION/DEFAULT-SPACING parameter (not used to
override actual per-tick dt, which is always computed from the real
timestamps passed to `correct()` -- exactly like `guidance_lab.AlphaBetaFilter`
itself): callers that don't have a real per-sample timestamp can use
`1.0 / update_rate_hz` as a synthetic, evenly-spaced `t`.
"""
from __future__ import annotations

import os
import sys
from typing import NamedTuple, Optional

SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, SCRIPTS_DIR)
from guidance_lab import AlphaBetaFilter  # noqa: E402  (single source of the Kalata/alpha-beta code)


class TrackState(NamedTuple):
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float


class GroundTrackFilter:
    """3-axis (x, y, z) alpha-beta position+velocity filter over the ground
    station's triangulated target positions. `predict()` every tick (or
    skip it and just `correct()` on each new measurement -- both are valid
    g-h filter usage, matching `guidance_lab.AlphaBetaFilter`'s own
    contract); `correct()` only on ticks with a genuinely fresh
    triangulation.
    """

    def __init__(self, alpha: float = 0.6, beta: float = 0.2,
                 update_rate_hz: float = 10.0,
                 kalata_sigma_process: Optional[float] = None,
                 kalata_sigma_meas: Optional[float] = None):
        self.update_rate_hz = update_rate_hz
        self.fx = AlphaBetaFilter(alpha, beta, kalata_sigma_process=kalata_sigma_process,
                                   kalata_sigma_meas=kalata_sigma_meas)
        self.fy = AlphaBetaFilter(alpha, beta, kalata_sigma_process=kalata_sigma_process,
                                   kalata_sigma_meas=kalata_sigma_meas)
        self.fz = AlphaBetaFilter(alpha, beta, kalata_sigma_process=kalata_sigma_process,
                                   kalata_sigma_meas=kalata_sigma_meas)

    @property
    def initialized(self) -> bool:
        return self.fx.initialized

    def predict(self, dt: float) -> None:
        self.fx.predict(dt)
        self.fy.predict(dt)
        self.fz.predict(dt)

    def correct(self, x: float, y: float, z: float, t: float) -> None:
        self.fx.correct(x, t)
        self.fy.correct(y, t)
        self.fz.correct(z, t)

    @property
    def state(self) -> TrackState:
        return TrackState(
            x=self.fx.x_hat if self.fx.x_hat is not None else float("nan"),
            y=self.fy.x_hat if self.fy.x_hat is not None else float("nan"),
            z=self.fz.x_hat if self.fz.x_hat is not None else float("nan"),
            vx=self.fx.xdot_hat, vy=self.fy.xdot_hat, vz=self.fz.xdot_hat,
        )

    def step(self, x: float, y: float, z: float, t: float, dt: Optional[float] = None) -> TrackState:
        """Convenience: predict forward by `dt` (default `1/update_rate_hz`)
        then correct with the new measurement, returning the resulting
        state. Matches the "predict every tick, correct on fresh
        measurement" usage pattern `m4_intercept.py` follows."""
        if dt is None:
            dt = 1.0 / self.update_rate_hz
        if self.initialized:
            self.predict(dt)
        self.correct(x, y, z, t)
        return self.state
