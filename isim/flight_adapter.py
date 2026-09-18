"""isim Guidance adapter over the UNMODIFIED real flight code (ADR-0102 isim).

`RealFlightGuidance` drives `flight.deploy.real_flight.RealFlightSM` -- the
exact state machine that will fly on the vehicle -- from inside the isim
engagement loop. No line of `flight/` is copied or re-implemented here: this
module only translates isim's `VehicleState`/`Detection` into the
`VehicleObs` the state machine reads, and its `Setpoint` back into a
`VelCmd`. Honesty: `step()` receives only an own-state `VehicleState` and an
optional `Detection` from the engine -- never a `TargetState` or a
`FrameReport` (isim's `Guidance` protocol structurally forbids it).
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

from flight.camera import CameraModel
from flight.deploy.real_flight import MissionConfig, RealFlightSM, ScriptedTrigger, VehicleObs
from flight.deploy.seeker_loop import GuidanceConfig, SeekerGuidance
from flight.pursuit_terminal import PursuitTerminalConfig, PursuitTerminalGuidance
from flight.tag_terminal import TagInterceptGuidance, TagTerminalConfig

from isim.seeker import CameraParams
from isim.types import Detection, VehicleState, VelCmd


def standby_init_state(cfg: MissionConfig) -> VehicleState:
    """A vehicle hovering at (0, 0, -standby_alt_m) NED, yaw = cfg.aim_yaw_deg,
    zero velocity -- satisfies the arm gate's altitude/yaw tolerance so a GO at
    or after `cfg.standby_settle_s` is accepted."""
    yaw_rad = math.radians(cfg.aim_yaw_deg)
    half = 0.5 * yaw_rad
    quat = (math.cos(half), 0.0, 0.0, math.sin(half))
    pos = np.array([0.0, 0.0, -cfg.standby_alt_m], dtype=np.float64)
    vel = np.zeros(3, dtype=np.float64)
    return VehicleState(t=0.0, pos_ned=pos, vel_ned=vel, quat_wxyz=quat, yaw_rad=yaw_rad)


def _assert_cam_match(guidance: SeekerGuidance, cam: CameraParams) -> None:
    """Refuse a caller-supplied SeekerGuidance whose camera disagrees with the
    isim CameraParams it is nominally paired with -- silently trusting a
    mismatched pair would make the sim's seeker geometry diverge from the
    guidance math's without any signal."""
    cm = guidance.cam
    bad: List[str] = []
    for attr, want in (("fx", cam.fx), ("fy", cam.fy), ("cx", cam.cx), ("cy", cam.cy)):
        got = getattr(cm, attr)
        if abs(got - want) > 1e-6:
            bad.append(f"{attr}: guidance={got} isim={want}")
    if cm.width is not None and cam.width is not None and cm.width != cam.width:
        bad.append(f"width: guidance={cm.width} isim={cam.width}")
    if cm.height is not None and cam.height is not None and cm.height != cam.height:
        bad.append(f"height: guidance={cm.height} isim={cam.height}")
    want_tilt = math.radians(cam.mount_tilt_up_deg)
    if abs(guidance.cfg.mount_up_rad - want_tilt) > 1e-6:
        bad.append(f"mount_tilt_rad: guidance={guidance.cfg.mount_up_rad} isim={want_tilt}")
    if bad:
        raise ValueError(
            "RealFlightGuidance: the supplied SeekerGuidance camera does not "
            "match the isim.seeker.CameraParams passed alongside it: " + "; ".join(bad))


class RealFlightGuidance:
    """isim `Guidance` wrapping a fresh `RealFlightSM` per engagement.

    `guidance`/`cam_params`/`span_m` describe the ENGAGE-phase camera terminal.
    If `guidance` is given, its `(cfg, cam, span_m)` are reused to rebuild a
    fresh `SeekerGuidance` on every `reset()` (it carries per-engagement
    alpha-beta filter state with no `reset()` of its own, so reusing the same
    instance across engagements would leak state -- see `reset()`). If
    `cam_params` is also given, it must describe the SAME camera (checked, not
    silently overridden). If `guidance` is None, a default one is built the
    same way `real_flight`'s CLI does, from `cam_params` (default
    `CameraParams()`): fx/fy/cx/cy/width/height feed `CameraModel`, and
    `mount_tilt_up_deg` feeds `GuidanceConfig.mount_up_rad`.

    `terminal="pursuit"` builds `flight.pursuit_terminal.PursuitTerminalGuidance`
    (ADR-0103's "chase only" concept) instead, and REQUIRES `belief_r0_ned`/
    `belief_vel0_ned` -- the target's pre-flight-believed position-relative-to-
    own and absolute velocity at `go_at_s`. This adapter does not compute them
    (it would need the dash's own endpoint kinematics, not just the target's);
    the caller supplies them, the same way it already supplies `tag_cfg`.

    NOT representable in `(CameraModel, GuidanceConfig)`, so not carried over:
    `CameraParams.fps`/`exposure_s`/`latency_s`/`latency_jitter_s` (pipeline
    timing -- the seeker model's concern; by the time a `Detection` reaches
    `step()` the latency has already elapsed) and lens distortion (isim's
    seeker is an ideal pinhole, so `CameraModel`'s `dist` stays the zero
    default -- nothing to carry).
    """

    def __init__(self, cfg: MissionConfig, guidance: Optional[SeekerGuidance] = None,
                 cam_params: Optional[CameraParams] = None, span_m: float = 1.0,
                 go_at_s: float = 0.0, home_alt_m: float = 0.0,
                 terminal: str = "stock",
                 tag_cfg: Optional[TagTerminalConfig] = None,
                 pursuit_cfg: Optional[PursuitTerminalConfig] = None,
                 belief_r0_ned: Optional[Tuple[float, float, float]] = None,
                 belief_vel0_ned: Optional[Tuple[float, float, float]] = None) -> None:
        # `terminal`: "stock" (default) builds the SAME SeekerGuidance (LOS-rate
        # pro-nav) this class always built; "tag" builds flight.tag_terminal.
        # TagInterceptGuidance (3-D predicted-intercept-point law); "pursuit"
        # builds flight.pursuit_terminal.PursuitTerminalGuidance (ADR-0103's
        # "chase only" concept, docs/pursuit_port_2026-09-17.md). Only ever
        # consulted in reset(), so an explicit `guidance=` (SeekerGuidance
        # override) is unaffected unless terminal="tag"/"pursuit" is requested.
        if terminal not in ("stock", "tag", "pursuit"):
            raise ValueError(f"RealFlightGuidance: terminal={terminal!r}, want "
                             f"'stock', 'tag' or 'pursuit'")
        if terminal == "pursuit" and (belief_r0_ned is None or belief_vel0_ned is None):
            raise ValueError("RealFlightGuidance: terminal='pursuit' requires "
                             "belief_r0_ned and belief_vel0_ned (the target's "
                             "position-relative-to-own and absolute velocity at "
                             "go_at_s -- a PRE-FLIGHT belief, the caller's to "
                             "supply; see flight.pursuit_terminal's module "
                             "docstring for why this can't be computed here).")
        self.terminal = terminal
        self.tag_cfg = tag_cfg
        self.pursuit_cfg = pursuit_cfg
        self.belief_r0_ned = belief_r0_ned
        self.belief_vel0_ned = belief_vel0_ned
        self.cfg = cfg
        self.go_at_s = go_at_s
        self.home_alt_m = home_alt_m
        if guidance is not None:
            if cam_params is not None:
                _assert_cam_match(guidance, cam_params)
            self._gcfg, self._cam_model, self._span_m = (
                guidance.cfg, guidance.cam, guidance.span_m)
        else:
            cp = cam_params or CameraParams()
            self._cam_model = CameraModel(cp.fx, cp.fy, cp.cx, cp.cy,
                                          width=cp.width, height=cp.height)
            fwd, right, down = cp.mount_xyz_body
            self._gcfg = GuidanceConfig(mount_up_rad=math.radians(cp.mount_tilt_up_deg),
                                        mount_fwd_m=fwd, mount_left_m=-right,
                                        mount_up_m=-down, target_span_m=span_m)
            self._span_m = span_m
        self.state_log: List[Tuple[float, str]] = []
        self.events: List[str] = []
        self.last_decision = None
        self._sm: Optional[RealFlightSM] = None
        self._trigger: Optional[ScriptedTrigger] = None
        self.reset()

    def reset(self) -> None:
        """Fresh `RealFlightSM`, fresh `SeekerGuidance`, fresh trigger -- no
        state (latch, streak, alpha-beta filters, RC edge memory) may survive
        into the next engagement."""
        if self.terminal == "tag":
            fresh_guidance = TagInterceptGuidance(
                self.tag_cfg or TagTerminalConfig(), self._cam_model, self._span_m,
                self._gcfg)
        elif self.terminal == "pursuit":
            fresh_guidance = PursuitTerminalGuidance(
                self.pursuit_cfg or PursuitTerminalConfig(), self._cam_model, self._span_m,
                self._gcfg, belief_r0_ned=self.belief_r0_ned,
                belief_vel0_ned=self.belief_vel0_ned, go_at_s=self.go_at_s)
        else:
            fresh_guidance = SeekerGuidance(self._gcfg, self._cam_model, self._span_m)
        self._sm = RealFlightSM(self.cfg, guidance=fresh_guidance)
        self._trigger = ScriptedTrigger(go_at_s=self.go_at_s)
        self.state_log = []
        self.events = []
        self.last_decision = None

    def step(self, t: float, own: VehicleState, det: Optional[Detection]) -> VelCmd:
        assert self._sm is not None and self._trigger is not None  # reset() ran in __init__
        trig = self._trigger.poll(t)

        box = None
        det_range_m = None
        det_bearing_deg = None
        if det is not None:
            half = 0.5 * det.side_px
            box = (det.u_px - half, det.v_px - half, det.side_px, det.side_px)
            det_range_m = det.range_m
            det_bearing_deg = det.bearing_deg

        obs = VehicleObs(
            t=t, armed=True, offboard_active=True, offboard_sample_age_s=0.0,
            mode="OFFBOARD", alt_m=(-float(own.pos_ned[2]) + self.home_alt_m),
            yaw_deg=math.degrees(own.yaw_rad), quat=tuple(float(c) for c in own.quat_wxyz),
            ground_speed_ms=math.hypot(float(own.vel_ned[0]), float(own.vel_ned[1])),
            vel_ned=tuple(float(c) for c in own.vel_ned),
            trigger=trig, det_new=(det is not None), det_box_xywh=box,
            det_range_m=det_range_m, det_bearing_deg=det_bearing_deg)

        dec = self._sm.step(obs)
        self.last_decision = dec
        self.events.extend(dec.events)
        if dec.transition is not None:
            self.state_log.append((t, dec.state))

        sp = dec.setpoint
        return VelCmd(v_north=sp.v_north, v_east=sp.v_east, v_down=sp.v_down,
                      yaw_deg=sp.yaw_deg)
