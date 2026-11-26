#!/usr/bin/env python3
"""The pilot hand-over profile in the fast sim (isim, flight code in the loop).
PRE-REGISTERED: isim/specs/handover_profile_prereg_2026-11-26.md -- read it
before interpreting output.

The flight program's own objects fly: arguments parsed by its parser, its
mission config, its pursuit terminal (flying preset + rehearsal profile), its
state machine, and its `HandoverLimits` on every setpoint. This file only
builds the world, draws the errors, and scores.

  --arm dev   seeds 1000+, a handful of runs (not registered)
  --arm S     slow grid: target hover/straight/loop x tag x cap
  --arm H     height hold as built vs vertical let through from GO
  --arm G     start height and start distance pinned
  --arm W     worst tier on a named cell (--tag-m, --cap, --dh-lo/--dh-hi, ...)
  --arm F     fast rung: target 5 and 9 m/s, cap 16
  --report    print every table from the saved rows
Rows land in runs/handover_profile_isim_20261126/<arm>.json.
"""
import argparse
import contextlib
import io
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, replace
from multiprocessing import get_context

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import numpy as np                                                    # noqa: E402

from flight.camera import CameraModel                                 # noqa: E402
from flight.deploy import real_flight as rf                           # noqa: E402
from flight.deploy.seeker_loop import GuidanceConfig                  # noqa: E402
from isim.engine import EngagementConfig, run_engagement              # noqa: E402
from isim.replay_a0 import load_params                                # noqa: E402
from isim.seeker import (AprilTagSeeker, CameraParams, DecodeParams,  # noqa: E402
                         TagParams, quat_to_rot)
from isim.target_attitude import AttitudeTarget                       # noqa: E402
from isim.targets import ConstantVelocityTarget, HoverTarget          # noqa: E402
from isim.types import TargetState, VehicleState, VelCmd              # noqa: E402
from isim.vehicle import QuadVelocityModel                            # noqa: E402

OUT = os.path.join(ROOT, "runs/handover_profile_isim_20261126")
FIT = os.path.join(ROOT, "isim/fits/vehicle_gazebo_x500_engage.json")
ENVELOPE = {"tilt_sat_vertical_priority": True, "thrust_max": 39.2,
            "max_setpoint_speed_horiz": 33.0, "max_setpoint_accel_horiz": 5.0,
            "max_tilt_deg": 45.0}                 # the 2026-11-18 rerun's "now"
FX, W, H = 800.0, 1280, 800                       # AR0234 + 3.6 mm at 2/3 scale (arithmetic)
TILT_TOLD = 20.0
TGT_ALT = 6.0
GO_AT = 3.0
CONTACT_M = 0.35
SALT = 20261126
SEEDS = list(range(24))


@dataclass
class CircleTarget:
    """Constant-speed circle, level. Starts at `phase0` (rad), turning left
    (counter-clockwise seen from above when `left` else clockwise)."""
    centre_ned: tuple
    radius_m: float
    speed_ms: float
    phase0: float
    left: bool = True

    def state(self, t: float) -> TargetState:
        w = self.speed_ms / self.radius_m * (1.0 if self.left else -1.0)
        a = self.phase0 + w * t
        c = np.asarray(self.centre_ned, float)
        pos = c + self.radius_m * np.array([math.cos(a), math.sin(a), 0.0])
        vel = self.radius_m * w * np.array([-math.sin(a), math.cos(a), 0.0])
        return TargetState(t=t, pos_ned=pos, vel_ned=vel)


class VerticalFromGo(rf.HandoverLimits):
    """COUNTERFACTUAL for arm H only (not flight code): the same limiter, with
    the capped vertical command let through from GO instead of from the first
    camera-guided tick."""
    def limit(self, sp, state_name, yaw_now_deg, camera_guided=False):
        return super().limit(sp, state_name, yaw_now_deg, camera_guided=True)


class HandoverGuidance:
    """isim Guidance: the flight state machine + terminal + HandoverLimits,
    stepped the way `real_flight._control_loop(handover=...)` steps them."""

    def __init__(self, args, cue_belief, limiter_cls=rf.HandoverLimits):
        self.args, self.cue_belief, self.limiter_cls = args, cue_belief, limiter_cls
        self.reset()

    def reset(self):
        a = self.args
        with contextlib.redirect_stdout(io.StringIO()):
            cfg = rf.handover_config(rf.build_config(a))
            gcfg = GuidanceConfig(
                n_pronav=a.n_pronav, mount_fwd_m=a.mount_fwd_m,
                mount_left_m=a.mount_left_m, mount_up_m=a.mount_up_m,
                mount_up_rad=math.radians(a.mount_tilt_deg),
                alt_ref_m=rf.apply_alt_ref_trim(cfg.dash_base_alt_m,
                                                cfg.dash_alt_trim_m, None))
            gcfg.target_span_m = float(a.target_span_m)
            cam = CameraModel(FX, FX, W / 2.0, H / 2.0, width=W, height=H)
            term = rf.build_terminal(a, cfg, gcfg, cam)
        self.cfg = cfg
        self.sm = rf.RealFlightSM(cfg, guidance=term)
        self.trigger = rf.ScriptedTrigger(go_at_s=GO_AT)
        self.limits = self.limiter_cls(float(a.handover_max_speed_ms),
                                       vertical="camera",
                                       max_vert_ms=float(a.handover_max_vert_ms))
        self._go_seen = False
        self.t_phase_b = None
        self.t_first_vert = None
        self.max_h = 0.0
        self.max_v = 0.0

    def step(self, t, own: VehicleState, det):
        trig = self.trigger.poll(t)
        belief = None
        if trig.go and not self._go_seen:
            self._go_seen = True
            belief = self.cue_belief(own)          # relative to own position NOW
        box = rng_m = None
        if det is not None:
            half = 0.5 * det.side_px
            box = (det.u_px - half, det.v_px - half, det.side_px, det.side_px)
            rng_m = det.range_m
        obs = rf.VehicleObs(
            t=t, armed=True, offboard_active=True, offboard_sample_age_s=0.0,
            mode="OFFBOARD", alt_m=-float(own.pos_ned[2]),
            yaw_deg=math.degrees(own.yaw_rad),
            quat=tuple(float(c) for c in own.quat_wxyz),
            ground_speed_ms=math.hypot(float(own.vel_ned[0]), float(own.vel_ned[1])),
            vel_ned=tuple(float(c) for c in own.vel_ned), trigger=trig,
            det_new=(det is not None), det_box_xywh=box, det_range_m=rng_m,
            det_range_pose_m=(det.range_m if det is not None else None),
            own_state_age_s=0.0, camera_age_s=0.0, cue_belief=belief)
        dec = self.sm.step(obs)
        # --- exactly the control loop's hand-over lines -----------------------
        cam_guided = (getattr(dec.telemetry, "phase", None) == "B"
                      or getattr(self.sm.guidance, "rehearsal_trigger_t", None) is not None)
        sp = self.limits.limit(dec.setpoint, dec.state, math.degrees(own.yaw_rad),
                               camera_guided=cam_guided)
        if self.t_phase_b is None and getattr(dec.telemetry, "phase", None) == "B":
            self.t_phase_b = t
        if self.t_first_vert is None and sp.v_down != 0.0:
            self.t_first_vert = t
        self.max_h = max(self.max_h, math.hypot(sp.v_north, sp.v_east))
        self.max_v = max(self.max_v, abs(sp.v_down))
        return VelCmd(sp.v_north, sp.v_east, sp.v_down, sp.yaw_deg)


def flight_args(job):
    argv = ["--pilot-handover", "--mavsdk-url", "serial:///dev/ttyAMA10:921600",
            "--trigger", "cue", "--terminal", "pursuit", "--rc-channel", "8",
            "--source", "picamera", "--detector", "tag",
            "--target-span-m", str(job["tag_m"]), "--camera-mount", "upside-down",
            "--min-agl-m", "2.0", "--mount-tilt-deg", str(job.get("tilt_told", TILT_TOLD)),
            "--handover-max-speed-ms", str(job["cap"]), "--fps", "20",
            "--standby-max-s", "600"]
    if job["cap"] > 3.0:
        argv.append("--handover-allow-fast")
    if job.get("contact"):
        argv.append("--handover-allow-contact")
    else:
        argv.append("--pursuit-rehearsal")
    argv += job.get("extra_argv", [])
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        args = rf.build_arg_parser().parse_args(argv)
    why = rf.validate_handover_args(args)
    if why:
        raise SystemExit("flight program would refuse these arguments: " + "; ".join(why))
    return args


def _unit(a):
    return np.array([math.cos(a), math.sin(a), 0.0])


def run_one(job):
    rng = np.random.default_rng([int(job["seed"]), SALT])
    d0 = job.get("dist") if job.get("dist") is not None else float(rng.uniform(8.0, 20.0))
    off = math.radians(float(rng.uniform(-20.0, 20.0)))
    dh = (job["dh"] if job.get("dh") is not None
          else float(rng.uniform(job.get("dh_lo", -2.0), job.get("dh_hi", 2.0))))
    nose = math.radians(float(rng.uniform(-30.0, 30.0)))
    e_lo, e_hi = job.get("cue_h_err", (2.0, 3.0))
    e_mag, e_dir = float(rng.uniform(e_lo, e_hi)), float(rng.uniform(0, 2 * math.pi))
    he = float(job.get("cue_height_err", 5.0))
    h_err = float(rng.uniform(-he, he))
    v_mag, v_dir = float(rng.uniform(0.05, 0.13)), float(rng.uniform(0, 2 * math.pi))
    phase0 = float(rng.uniform(0, 2 * math.pi))
    left = bool(rng.integers(0, 2))

    kind, v_t = job["target"], float(job["v_t"])
    if kind == "hover":
        inner = HoverTarget(np.array([0.0, 0.0, -TGT_ALT]))
    elif kind == "straight":
        inner = ConstantVelocityTarget(np.array([-v_t * GO_AT, 0.0, -TGT_ALT]),
                                       np.array([v_t, 0.0, 0.0]))
    elif kind == "loop":
        inner = CircleTarget((0.0, 0.0, -TGT_ALT), 2.75, v_t, phase0, left)
    else:
        raise ValueError(kind)
    target = AttitudeTarget(inner)
    s_go = target.state(GO_AT)
    vh = np.array([s_go.vel_ned[0], s_go.vel_ned[1]])
    course = math.atan2(vh[1], vh[0]) if np.linalg.norm(vh) > 0.5 else 0.0
    # interceptor: d0 behind the target (where the rear tag faces), `off` off astern
    back = course + math.pi + off
    own_pos = np.asarray(s_go.pos_ned, float) + d0 * _unit(back)
    own_pos[2] = -(TGT_ALT + dh)
    yaw = math.atan2(-math.sin(back), -math.cos(back)) + nose
    init = VehicleState(t=0.0, pos_ned=own_pos, vel_ned=np.zeros(3),
                        quat_wxyz=(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)),
                        yaw_rad=yaw)

    def cue_belief(own: VehicleState):
        s = target.state(GO_AT)
        p = np.asarray(s.pos_ned, float) + e_mag * _unit(e_dir) + np.array([0, 0, -h_err])
        v = np.asarray(s.vel_ned, float) + v_mag * _unit(v_dir)
        r0 = p - np.asarray(own.pos_ned, float)
        return (tuple(float(c) for c in r0), (float(v[0]), float(v[1]), 0.0))

    worst = bool(job.get("worst"))
    vp = replace(load_params(FIT), **ENVELOPE)
    if worst:
        vp = replace(vp, gust_ou_std=1.5, gust_ou_std_vert=0.5)
    vehicle = QuadVelocityModel(vp)
    cam_true = CameraParams(width=W, height=H, fx=FX, fy=FX, cx=W / 2.0, cy=H / 2.0,
                            mount_tilt_up_deg=float(job.get("tilt_true", TILT_TOLD)))
    dec = DecodeParams(min_decode_interval_s=0.05)
    if worst:
        dec = replace(dec, p_max=0.80, tgt_shake_rms_deg=3.0)
    if job.get("decode_strict"):
        # AMENDMENT 1 bracket: no decode under 20 px, half at 30 px (a detector
        # run at quad_decimate 2, as the flight code runs it, sees half the pixels)
        dec = replace(dec, min_side_px=20.0, size50_px=30.0)
    tag = TagParams(side_m=float(job["tag_m"]), faces_camera=False, faces_velocity=False,
                    normal_ned=(-1.0, 0.0, 0.0), body_normal_frd=(-1.0, 0.0, 0.0))
    seeker = AprilTagSeeker(cam=cam_true, tag=tag, dec=dec)
    args = flight_args(job)
    lim = VerticalFromGo if job.get("limiter") == "vertfree" else rf.HandoverLimits
    guidance = HandoverGuidance(args, cue_belief, lim)
    max_t = float(job.get("max_t", 75.0))
    ecfg = EngagementConfig(guidance_dt=0.05, max_t=max_t, stop_after_cpa_s=max_t,
                            stop_not_before_s=max_t, seed=int(job["seed"]))
    with contextlib.redirect_stdout(io.StringIO()):
        res = run_engagement(ecfg, vehicle, target, seeker, guidance, init,
                             record_trace=True)
    tr = res.trace
    t, rngm = tr["t"], tr["range"]
    post = t >= GO_AT
    sm, term = guidance.sm, guidance.sm.guidance
    reps = res.frame_reports
    t_rep = np.array([r.t_capture for r in reps])
    # acquisition is counted from GO: the terminal does not run before it
    decd = np.array([r.decoded and r.t_capture >= GO_AT for r in reps])
    idx = np.clip(np.searchsorted(t, t_rep), 0, len(t) - 1)
    row = dict(tag=job["cell"], seed=int(job["seed"]), d0=d0, dh=dh, off_deg=math.degrees(off),
               nose_deg=math.degrees(nose), cue_err_m=e_mag, cue_h_err_m=h_err,
               n_frames=len(reps), n_dec=int(decd.sum()),
               min_sep=float(rngm[post].min()), t_min_sep=float(t[post][np.argmin(rngm[post])]),
               contact=bool(rngm[post].min() < CONTACT_M),
               max_cmd_h=float(guidance.max_h), max_cmd_v=float(guidance.max_v),
               t_phase_b=guidance.t_phase_b, t_first_vert=guidance.t_first_vert,
               travel_m=float(np.linalg.norm(tr["own_pos"][-1, :2] - tr["own_pos"][0, :2])),
               alt_min=float((-tr["own_pos"][:, 2]).min()), alt_max=float((-tr["own_pos"][:, 2]).max()),
               final_sep=float(rngm[-1]),
               states="|".join(sm.visited),
               reasons=" || ".join(f"{x.to}@{x.t:.1f}:{x.reason}" for x in sm.transitions)[:400])
    if decd.any():
        i = int(np.argmax(decd))
        row.update(first_dec_t=float(t_rep[i]), first_dec_range=float(rngm[idx[i]]))
    else:
        row.update(first_dec_t=None, first_dec_range=None)
    tt = getattr(term, "rehearsal_trigger_t", None)
    row["trig_t"] = None if tt is None else float(tt)
    row["trig_range_est"] = (None if tt is None
                             else float(getattr(term, "rehearsal_trigger_range_m", float("nan"))))
    row["trig_tgo"] = None if tt is None else getattr(term, "rehearsal_t_go_s", None)
    row["too_late"] = bool(getattr(term, "rehearsal_too_late", False))
    row["rehearsal_complete"] = bool(getattr(term, "rehearsal_complete", False))
    if tt is not None:
        m = t >= tt
        k = int(np.argmax(m))
        row["trig_range_true"] = float(rngm[k])
        row["min_sep_after_trig"] = float(rngm[m].min())
        vrel = (tr["tgt_vel"][k] - tr["own_vel"][k])
        los = (tr["tgt_pos"][k] - tr["own_pos"][k]) / max(rngm[k], 1e-6)
        row["closing_at_trig"] = float(-vrel @ los)
    else:
        row["trig_range_true"] = row["min_sep_after_trig"] = row["closing_at_trig"] = None
    # WHERE WAS THE TAG when it was big enough to decode but not in the picture?
    # (truth; own attitude from the trace; the TRUE camera tilt)
    r_bc = cam_true.rot_body_from_cam()
    cnt = dict(top=0, bottom=0, side=0, behind=0, in_view=0)
    t_end = row["first_dec_t"] if row["first_dec_t"] is not None else t[-1]
    for j, rp in zip(idx, reps):
        if rp.t_capture < GO_AT or rp.t_capture > t_end:
            continue
        rel = tr["tgt_pos"][j] - tr["own_pos"][j]
        if FX * job["tag_m"] / max(np.linalg.norm(rel), 1e-6) < 22.0:
            continue                                   # too small to decode anyway
        vc = r_bc.T @ (quat_to_rot(tuple(tr["quat"][j])).T @ rel)
        if vc[2] <= 0.05:
            cnt["behind"] += 1
            continue
        u, v = FX * vc[0] / vc[2] + W / 2.0, FX * vc[1] / vc[2] + H / 2.0
        if v < 0:
            cnt["top"] += 1
        elif v > H:
            cnt["bottom"] += 1
        elif u < 0 or u > W:
            cnt["side"] += 1
        else:
            cnt["in_view"] += 1
    row.update({f"big_{k}": v for k, v in cnt.items()})
    return row


def run_jobs(jobs, workers):
    with ProcessPoolExecutor(max_workers=workers, mp_context=get_context("spawn")) as ex:
        return list(ex.map(run_one, jobs, chunksize=1))


def cell(name, seeds, **kw):
    return [dict(cell=name, seed=s, **kw) for s in seeds]


TARGETS = {"hover": ("hover", 0.0), "straight": ("straight", 0.9), "loop": ("loop", 1.0)}


def jobs_for(arm, a):
    J = []
    if arm == "dev":
        for nm, (k, v) in TARGETS.items():
            J += cell(f"dev {nm} tag0.35 cap1.5", [1000, 1001], target=k, v_t=v, tag_m=0.35, cap=1.5)
        J += cell("dev fast9 tag0.35 cap16", [1000], target="straight", v_t=9.0, tag_m=0.35,
                  cap=16.0, max_t=60.0)
    elif arm == "S":
        for nm, (k, v) in TARGETS.items():
            for tag in (0.10, 0.16, 0.20, 0.35):
                for cap in (0.9, 1.5, 2.5):
                    J += cell(f"S {nm} tag{tag:.2f} cap{cap}", SEEDS, target=k, v_t=v,
                              tag_m=tag, cap=cap)
    elif arm == "H":
        for nm in ("hover", "straight"):
            k, v = TARGETS[nm]
            for tag in (0.20, 0.35):
                for he in (5.0, 1.0):
                    for lim in ("asbuilt", "vertfree"):
                        J += cell(f"H {nm} tag{tag:.2f} herr{he:.0f} {lim}", SEEDS, target=k,
                                  v_t=v, tag_m=tag, cap=1.5, cue_height_err=he, limiter=lim)
    elif arm == "G":
        for tag in (0.20, 0.35):
            for dh in (-2.0, -1.0, 0.0, 1.0, 2.0):
                J += cell(f"G tag{tag:.2f} dh{dh:+.0f}", SEEDS, target="straight", v_t=0.9,
                          tag_m=tag, cap=1.5, dh=dh)
            for d in (6.0, 10.0, 15.0, 20.0):
                J += cell(f"G tag{tag:.2f} dist{d:.0f}", SEEDS, target="straight", v_t=0.9,
                          tag_m=tag, cap=1.5, dist=d)
    elif arm == "W":
        kw = dict(target="straight", v_t=0.9, tag_m=a.tag_m, cap=a.cap,
                  dh_lo=a.dh_lo, dh_hi=a.dh_hi)
        if a.dist is not None:
            kw["dist"] = a.dist
        base = f"tag{a.tag_m:.2f} cap{a.cap} dh[{a.dh_lo:+.0f},{a.dh_hi:+.0f}]" + (
            "" if a.dist is None else f" dist{a.dist:.0f}")
        J += cell(f"W expected {base}", SEEDS, **kw)
        J += cell(f"W worst {base}", SEEDS, worst=True, tilt_true=17.0,
                  cue_h_err=(3.0, 5.0), **kw)
    elif arm == "A":
        # AMENDMENT 1 (post hoc, after arms S/H/G/F were read; see the prereg)
        for nm in ("hover", "straight"):
            k, v = TARGETS[nm]
            for tag in (0.10, 0.16, 0.20, 0.35):
                J += cell(f"A strict {nm} tag{tag:.2f} cap1.5", SEEDS, target=k, v_t=v,
                          tag_m=tag, cap=1.5, decode_strict=True)
        for tag in (0.16, 0.20, 0.35):
            for cap in (0.9, 1.5):
                for dh in (-2.0, -1.0, 0.0, 1.0, 2.0):
                    J += cell(f"A hover tag{tag:.2f} cap{cap} dh{dh:+.0f}", SEEDS, target="hover",
                              v_t=0.0, tag_m=tag, cap=cap, dh=dh)
        for told in (0.0, 17.0, 23.0):
            J += cell(f"A straight tag0.20 cap1.5 told{told:.0f} true20", SEEDS, target="straight",
                      v_t=0.9, tag_m=0.20, cap=1.5, tilt_told=told, tilt_true=20.0)
    elif arm == "F":
        for v in (5.0, 9.0):
            for tag in (0.20, 0.35):
                J += cell(f"F v{v:.0f} tag{tag:.2f} cap16", SEEDS, target="straight", v_t=v,
                          tag_m=tag, cap=16.0, max_t=60.0)
    return J


def _med(xs):
    xs = [x for x in xs if x is not None]
    return float(np.median(xs)) if xs else float("nan")


def table(rows, expect_n=24):
    cells = {}
    for r in rows:
        cells.setdefault(r["tag"], []).append(r)
    print(f"{'cell':>44} {'n':>3} {'acq':>4} {'t_acq':>6} {'r_acq':>6} {'trig':>4} {'r_trig':>6} "
          f"{'late':>4} {'hit':>3} {'minsep':>6} {'min@trig':>8} {'worst@trig':>10} "
          f"{'top':>5} {'bot':>5} {'side':>5}")
    bad = 0
    for name, rs in cells.items():
        if len(rs) != expect_n:
            print(f"{name:>44} UNCERTAIN -- {len(rs)} rows, expected {expect_n}")
            bad += 1
            continue
        acq = [r for r in rs if r["first_dec_t"] is not None]
        trg = [r for r in rs if r["trig_t"] is not None]
        post = [r["min_sep_after_trig"] for r in trg]
        nb = sum(r["big_top"] + r["big_bottom"] + r["big_side"] + r["big_behind"] + r["big_in_view"]
                 for r in rs) or 1
        print(f"{name:>44} {len(rs):3d} {len(acq):4d} {_med([r['first_dec_t'] for r in acq]) - GO_AT:6.1f} "
              f"{_med([r['first_dec_range'] for r in acq]):6.1f} {len(trg):4d} "
              f"{_med([r['trig_range_true'] for r in trg]):6.2f} "
              f"{sum(r['too_late'] for r in rs):4d} {sum(r['contact'] for r in rs):3d} "
              f"{_med([r['min_sep'] for r in rs]):6.2f} {_med(post):8.2f} "
              f"{(min(post) if post else float('nan')):10.2f} "
              f"{sum(r['big_top'] for r in rs) / nb:5.2f} {sum(r['big_bottom'] for r in rs) / nb:5.2f} "
              f"{sum(r['big_side'] + r['big_behind'] for r in rs) / nb:5.2f}")
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", choices=["dev", "S", "H", "G", "W", "F", "A"])
    ap.add_argument("--report", action="store_true")
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--tag-m", type=float, default=0.35)
    ap.add_argument("--cap", type=float, default=1.5)
    ap.add_argument("--dh-lo", type=float, default=-2.0)
    ap.add_argument("--dh-hi", type=float, default=2.0)
    ap.add_argument("--dist", type=float, default=None)
    ap.add_argument("--suffix", default="")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    if a.report:
        for f in sorted(os.listdir(OUT)):
            if f.endswith(".json"):
                print(f"\n== {f}")
                table(json.load(open(os.path.join(OUT, f))),
                      expect_n=(2 if f.startswith("dev") else 24))
        return 0
    jobs = jobs_for(a.arm, a)
    rows = run_jobs(jobs, a.workers)
    path = os.path.join(OUT, f"{a.arm}{a.suffix}.json")
    json.dump(rows, open(path, "w"), indent=0)
    print(f"{len(rows)} rows -> {path}")
    return table(rows, expect_n=(2 if a.arm == 'dev' else 24))


if __name__ == "__main__":
    sys.exit(main())
