#!/usr/bin/env python3
"""Fuse one cinema take's logs into a single `<stem>_run.json` for the
renderer (blender_scene.py) and the HUD compositor (hud_compose.py).

Inputs (written by cinema_run.py):   <stem>.csv, <stem>_rf.csv,
<stem>_pose6.csv, <stem>_meta.json, <stem>_frames/ + <stem>_frames_index.csv
Output: <stem>_run.json
  timeline  : uniform SIM-time grid (default 60 Hz) from go-pre_roll to
              t_cpa+post_roll; vehicle/target pos (ENU m) + quat (wxyz),
              both linearly interpolated (quats nlerp'd) from the 6-DoF log;
              per-sample own speed / target speed / range / closing speed
              (finite-difference of the logged truth, smoothed).
  events    : go_t, first_det_t, last_det_t, det ticks (t, range), SM state
              transitions, cpa (m, t), lock segments (gaps > lock_gap_s split).
  frames    : index of saved seeker JPEGs (t -> file) + offline AprilTag
              re-detection boxes (same detector family/size as the run) so
              the PiP overlay shows the box the seeker really decoded.
All of it is logging/visualization; nothing feeds back into flight code.
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _REPO)
sys.path.insert(0, os.path.join(_REPO, "scripts"))


def load_pose6(path):
    t, vp, vq, gp, gq = [], [], [], [], []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            t.append(float(r["sim_t"]))
            vp.append([float(r["veh_x"]), float(r["veh_y"]), float(r["veh_z"])])
            vq.append([float(r["veh_qw"]), float(r["veh_qx"]),
                       float(r["veh_qy"]), float(r["veh_qz"])])
            gp.append([float(r["tgt_x"]), float(r["tgt_y"]), float(r["tgt_z"])])
            gq.append([float(r["tgt_qw"]), float(r["tgt_qx"]),
                       float(r["tgt_qy"]), float(r["tgt_qz"])])
    return (np.array(t), np.array(vp), np.array(vq), np.array(gp), np.array(gq))


def interp_vec(tq, t, v):
    return np.stack([np.interp(tq, t, v[:, i]) for i in range(v.shape[1])], axis=1)


def nlerp_quat(tq, t, q):
    # fix hemisphere flips, then componentwise lerp + renormalize (fine for
    # the small per-sample rotations of a 50 Hz log)
    q = q.copy()
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]
    out = interp_vec(tq, t, q)
    return out / np.linalg.norm(out, axis=1, keepdims=True)


def smooth(x, n=9):
    if len(x) < n:
        return x
    k = np.ones(n) / n
    return np.convolve(np.pad(x, (n // 2, n // 2), mode="edge"), k, mode="valid")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("stem", help="take stem, e.g. logs/cinema/take2 (no .csv)")
    ap.add_argument("--hz", type=float, default=60.0)
    ap.add_argument("--pre-roll", type=float, default=6.0)
    ap.add_argument("--post-roll", type=float, default=2.5)
    ap.add_argument("--lock-gap-s", type=float, default=1.0)
    ap.add_argument("--no-redetect", action="store_true")
    args = ap.parse_args()
    stem = args.stem[:-4] if args.stem.endswith(".csv") else args.stem

    meta = json.load(open(stem + "_meta.json"))
    t6, vp, vq, gp, gq = load_pose6(stem + "_pose6.csv")
    go, t_cpa = meta["go_sim_t"], meta["t_cpa"]
    t0 = max(float(t6[0]), go - args.pre_roll)
    t1 = min(float(t6[-1]), (t_cpa or go) + args.post_roll)
    n = int(round((t1 - t0) * args.hz)) + 1
    tq = t0 + np.arange(n) / args.hz

    VP, GP = interp_vec(tq, t6, vp), interp_vec(tq, t6, gp)
    VQ, GQ = nlerp_quat(tq, t6, vq), nlerp_quat(tq, t6, gq)
    dt = 1.0 / args.hz
    vv = np.gradient(VP, dt, axis=0)
    gv = np.gradient(GP, dt, axis=0)
    rel = GP - VP
    rng = np.linalg.norm(rel, axis=1)
    closing = -np.gradient(rng, dt)
    own_spd = smooth(np.linalg.norm(vv, axis=1))
    tgt_spd = smooth(np.linalg.norm(gv, axis=1))
    closing = smooth(closing)

    # events from the per-tick CSVs
    det_ticks, states = [], []
    with open(stem + ".csv") as fh:
        prev = None
        for r in csv.DictReader(fh):
            if r["det_range_m"]:
                det_ticks.append([float(r["sim_t"]), float(r["det_range_m"])])
            if r["sim_t"] and r["sm_state_pre_step"] != prev:
                states.append([float(r["sim_t"]), r["sm_state_pre_step"]])
                prev = r["sm_state_pre_step"]
    locks, seg = [], None
    for t, _ in det_ticks:
        if seg is None:
            seg = [t, t]
        elif t - seg[1] > args.lock_gap_s:
            locks.append(seg)
            seg = [t, t]
        else:
            seg[1] = t
    if seg:
        locks.append(seg)

    frames = []
    idx_path = stem + "_frames_index.csv"
    if os.path.exists(idx_path):
        with open(idx_path) as fh:
            frames = [[int(r["frame"]), float(r["sim_t"])]
                      for r in csv.DictReader(fh)]
    boxes = {}
    if frames and not args.no_redetect:
        import cv2
        from apriltag_detector import Detector
        det = Detector(families="tag36h11")
        fdir = stem + "_frames"
        for fnum, ft in frames:
            if ft < go - 1.0 or ft > t1 + 0.5:
                continue
            img = cv2.imread(os.path.join(fdir, f"{fnum:05d}.jpg"),
                             cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            hits = [d for d in det.detect(img) if d.tag_id == 0]
            if hits:
                c = hits[0].corners
                boxes[fnum] = [[float(x), float(y)] for x, y in c]
        print(f"[export] re-detected boxes on {len(boxes)} frames")

    run = {
        "meta": meta, "hz": args.hz, "t": tq.round(4).tolist(),
        "veh_pos_enu": VP.round(4).tolist(), "veh_quat_wxyz": VQ.round(5).tolist(),
        "tgt_pos_enu": GP.round(4).tolist(), "tgt_quat_wxyz": GQ.round(5).tolist(),
        "own_speed": own_spd.round(3).tolist(), "tgt_speed": tgt_spd.round(3).tolist(),
        "range_m": rng.round(3).tolist(), "closing_ms": closing.round(3).tolist(),
        "events": {"go_t": go, "t_cpa": t_cpa, "cpa_m": meta["cpa_m"],
                   "det_ticks": det_ticks, "lock_segments": locks,
                   "sm_states": states,
                   "first_det_t": det_ticks[0][0] if det_ticks else None,
                   "last_det_t": det_ticks[-1][0] if det_ticks else None},
        "frames_index": frames, "det_boxes": boxes,
    }
    out = stem + "_run.json"
    json.dump(run, open(out, "w"))
    print(f"[export] {out}: {n} samples {t0:.2f}..{t1:.2f} s, "
          f"{len(det_ticks)} det ticks, {len(locks)} lock segment(s), "
          f"cpa {meta['cpa_m']:.3f} m @ {t_cpa}")


if __name__ == "__main__":
    main()
