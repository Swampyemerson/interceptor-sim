#!/usr/bin/env python3
"""Auto-label a flying target in a STATIC ground camera's video, by motion, for detector training.

The camera does not move, so anything that differs from a background built from the target-free
start of the video is a candidate. Candidates are linked into tracks; frames where exactly one
qualified track is present get a box. No detector is involved in making the labels (a detector's
own boxes would teach the next detector its own mistakes); an optional teacher model is run
afterwards ONLY to report how often it agrees.

    python scripts/nn/ground_video_labels.py --video sessions/video_X/video.h264 \
        --pts sessions/video_X/frames.pts --flight-start 306 --flight-end 804 \
        --out data/own_target/video_X [--teacher scripts/seeker/weights/drone_yolo11x.pt]
    python scripts/nn/ground_video_labels.py --self-test

Rules it enforces (docs/real_data_pipeline.md, docs/error_handling_policy.md):
- Negatives come ONLY from before --flight-start (target-free footage). A flight frame with no
  track is DROPPED, never labelled background: the target may be there, hidden against the trees.
- Every drop is counted in summary.json. Zero positives is exit 1, never a pass.
- One video is one session: the whole output is one split group (train OR held-out, never both).

Output: <out>/images/*.jpg, <out>/labels/*.txt (YOLO, class 0), index.csv, summary.json,
<out>/qa/*.jpg (overlay sheets to look at before trusting any of it).
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import cv2
import numpy as np

K9 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))


def change(bg16: np.ndarray, frame: np.ndarray) -> np.ndarray:
    """Per-pixel change from the background (uint8): the largest colour-channel difference, either sign, with
    slow drift removed. Colour matters: a sunlit brown airframe on blue sky can match the sky in grey and
    still differ in red or blue. Drift (the sun moving over minutes, glare) is the difference averaged over
    a few hundred pixels; it is estimated at 1/8 scale and subtracted before taking the magnitude."""
    d = cv2.subtract(bg16, frame, dtype=cv2.CV_16S)
    small = cv2.blur(cv2.resize(d, None, fx=0.125, fy=0.125, interpolation=cv2.INTER_AREA), (25, 25))
    d = cv2.subtract(d, cv2.resize(small, (d.shape[1], d.shape[0]), interpolation=cv2.INTER_LINEAR))
    a = cv2.convertScaleAbs(d)
    return cv2.max(cv2.max(a[..., 0], a[..., 1]), a[..., 2])


def read_frames(path: str):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit(f"FAIL: cannot open {path}")
    i = 0
    while True:
        ok, f = cap.read()
        if not ok:
            break
        yield i, f
        i += 1
    cap.release()


def build_background(frames_fn, t_of, t_end: float, every_s: float, thr: float, clutter_stride: int, clutter_segments: int, clutter_grow: int):
    """Median colour background and a clutter mask from the target-free start.
    Pass 1 takes one frame every `every_s` for the median (these are also the negatives).
    Pass 2 looks at every `clutter_stride`-th target-free frame and counts, per pixel, how often it differs
    from that background: whatever moves when no target is flying (streamers, lanterns, leaves, swaying
    wires) is clutter, and its whole swept area is masked. The clutter test uses the SAME threshold as
    detection: at a lower one, the faint contour bands an 8-bit sky gradient has drift across the frame and
    end up masking half the sky (measured 2026-11-15: 55 percent masked at 0.7 x 18, the sky itself quiet)."""
    stack, keep, last = [], [], -1e9
    for i, f in frames_fn():
        t = t_of(i)
        if t >= t_end:
            break
        if t - last >= every_s:
            stack.append(f)
            keep.append((i, t, f))
            last = t
    if len(stack) < 10:
        raise SystemExit(f"FAIL: only {len(stack)} target-free frames before the flight; need 10 or more")
    arr = np.stack(stack)
    bg = np.stack([np.median(arr[..., c], axis=0) for c in range(3)], axis=2).astype(np.int16)
    del arr
    nseg = 10                           # clutter RECURS: it must show up in several separate stretches of the lead-in.
    seg_hits = np.zeros(bg.shape[:2], np.uint8)     # a person walking past the lens once is a transient, not clutter
    cur = np.zeros(bg.shape[:2], bool)
    n, seg = 0, 0
    for i, f in frames_fn():
        t = t_of(i)
        if t >= t_end:
            break
        if int(t / t_end * nseg) != seg:
            seg_hits += cur
            cur[:] = False
            seg = int(t / t_end * nseg)
        if i % clutter_stride:
            continue
        cur |= change(bg, f) > thr
        n += 1
    seg_hits += cur
    k = 2 * clutter_grow + 1
    raw = seg_hits >= clutter_segments
    clutter = cv2.dilate(raw.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))) > 0
    return bg, clutter, len(stack), keep, dict(clutter_frames=n, segments=nseg, segments_needed=clutter_segments, raw_clutter_fraction=round(float(raw.mean()), 4))


def candidates(frame, bg, clutter, thr, min_area, max_side, merge_px):
    d = change(bg, frame)
    m = (d > thr).astype(np.uint8)
    m[clutter] = 0
    k = 2 * merge_px + 1
    n, lab, st, _ = cv2.connectedComponentsWithStats(cv2.dilate(m, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))), 8)
    out = []
    for j in range(1, n):
        x, y, w, h, _a = st[j]
        sub = (m[y:y + h, x:x + w] > 0) & (lab[y:y + h, x:x + w] == j)
        ys, xs = np.nonzero(sub)
        if len(xs) < min_area:
            continue
        x0, x1, y0, y1 = x + xs.min(), x + xs.max() + 1, y + ys.min(), y + ys.max() + 1
        if max(x1 - x0, y1 - y0) > max_side:
            continue
        g = 4                                           # how much of the box and its rim is clutter: a box cut by the mask is not a box
        near = clutter[max(0, y0 - g):y1 + g, max(0, x0 - g):x1 + g]
        out.append((int(x0), int(y0), int(x1), int(y1), int(len(xs)), float(d[y:y + h, x:x + w][sub].mean()), float(near.mean())))
    return out


class Track:
    __slots__ = ("id", "obs", "vx", "vy", "miss")

    def __init__(self, tid, i, c):
        self.id, self.obs, self.vx, self.vy, self.miss = tid, [(i, c)], 0.0, 0.0, 0

    def centre(self):
        x0, y0, x1, y1 = self.obs[-1][1][:4]
        return (x0 + x1) / 2, (y0 + y1) / 2

    def size(self):
        x0, y0, x1, y1 = self.obs[-1][1][:4]
        return max(x1 - x0, y1 - y0)


def link(per_frame, coast: int):
    """Greedy nearest-neighbour linking with a constant-velocity prediction."""
    live, done, nid, prev = [], [], 0, None
    for i, cands in per_frame:
        df = 1 if prev is None else max(1, i - prev)
        prev = i
        free = list(cands)
        for tr in sorted(live, key=lambda t: -len(t.obs)):
            cx, cy = tr.centre()
            px, py = cx + tr.vx * (i - tr.obs[-1][0]), cy + tr.vy * (i - tr.obs[-1][0])
            gate = 30 + 1.5 * tr.size() + 4 * tr.miss
            best, bd = None, gate
            for c in free:
                dd = np.hypot((c[0] + c[2]) / 2 - px, (c[1] + c[3]) / 2 - py)
                if dd < bd:
                    best, bd = c, dd
            if best is None:
                tr.miss += df
                continue
            free.remove(best)
            nx, ny = (best[0] + best[2]) / 2, (best[1] + best[3]) / 2
            dt = i - tr.obs[-1][0]
            tr.vx, tr.vy = 0.6 * tr.vx + 0.4 * (nx - cx) / dt, 0.6 * tr.vy + 0.4 * (ny - cy) / dt
            tr.obs.append((i, best))
            tr.miss = 0
        for tr in [t for t in live if t.miss > coast]:
            live.remove(tr)
            done.append(tr)
        for c in free:
            live.append(Track(nid, i, c))
            nid += 1
    return done + live


def qualify(tracks, min_obs: int, min_travel: float, max_clutter: float):
    """Keep tracks that last, that go somewhere, and that do not live on the edge of the clutter mask."""
    good, why = [], dict(too_short=0, went_nowhere=0, on_clutter=0)
    for tr in tracks:
        if len(tr.obs) < min_obs:
            why["too_short"] += 1
            continue
        cs = np.array([((c[0] + c[2]) / 2, (c[1] + c[3]) / 2) for _, c in tr.obs])
        if float(np.hypot(np.ptp(cs[:, 0]), np.ptp(cs[:, 1]))) < min_travel:
            why["went_nowhere"] += 1
            continue
        if float(np.median([c[6] for _, c in tr.obs])) > max_clutter:
            why["on_clutter"] += 1
            continue
        good.append(tr)
    return good, why


def pad_box(c, W, H, frac=0.15, min_pad=3):
    x0, y0, x1, y1 = c[:4]
    px, py = max(min_pad, frac * (x1 - x0)), max(min_pad, frac * (y1 - y0))
    return max(0, x0 - px), max(0, y0 - py), min(W, x1 + px), min(H, y1 + py)


def run(frames_fn, t_of, a, out):
    thr = a.thr
    bg, clutter, nbg, pre, cinfo = build_background(frames_fn, t_of, a.flight_start - a.guard_s, a.neg_every_s, thr,
                                                    a.clutter_stride, a.clutter_segments, a.clutter_grow)
    H, W = bg.shape[:2]
    os.makedirs(f"{out}/qa", exist_ok=True)
    vis = np.clip(bg, 0, 255).astype(np.uint8)
    vis[clutter] = (0.45 * vis[clutter] + 0.55 * np.array([0, 0, 255])).astype(np.uint8)
    cv2.imwrite(f"{out}/qa/clutter_mask.jpg", cv2.resize(vis, None, fx=0.5, fy=0.5))
    per_frame, n_flight, last_i = [], 0, -1
    for i, f in frames_fn():
        last_i = i
        t = t_of(i)
        if t > a.flight_end:
            break
        if t < a.flight_start or i % a.stride:
            continue
        n_flight += 1
        per_frame.append((i, candidates(f, bg, clutter, thr, a.min_area, a.max_side, a.merge_px)))
    tracks = link(per_frame, a.coast)
    good, why = qualify(tracks, a.min_obs, a.min_travel, a.max_clutter)
    by_frame = {}
    for tr in good:
        for i, c in tr.obs:
            by_frame.setdefault(i, []).append((tr.id, c))
    single = {i: v[0] for i, v in by_frame.items() if len(v) == 1 and v[0][1][6] <= a.max_clutter}
    ambiguous = sum(1 for v in by_frame.values() if len(v) > 1)
    cut_by_clutter = sum(1 for v in by_frame.values() if len(v) == 1 and v[0][1][6] > a.max_clutter)
    chosen, last = [], -1e9
    for i in sorted(single):
        if t_of(i) - last >= 1.0 / a.label_fps:
            chosen.append(i)
            last = t_of(i)
    want = set(chosen)
    os.makedirs(f"{out}/images", exist_ok=True); os.makedirs(f"{out}/labels", exist_ok=True); os.makedirs(f"{out}/qa", exist_ok=True)
    rows, crops = [], []
    if want:
        for i, f in frames_fn():
            if i not in want:
                continue
            tid, c = single[i]
            x0, y0, x1, y1 = pad_box(c, W, H)
            name = f"f{i:06d}"
            cv2.imwrite(f"{out}/images/{name}.jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 95])
            with open(f"{out}/labels/{name}.txt", "w") as fh:
                fh.write(f"0 {(x0 + x1) / 2 / W:.6f} {(y0 + y1) / 2 / H:.6f} {(x1 - x0) / W:.6f} {(y1 - y0) / H:.6f}\n")
            rows.append(dict(name=name, frame=i, video_s=round(t_of(i), 3), kind="positive", track=tid, x0=round(x0, 1), y0=round(y0, 1),
                             x1=round(x1, 1), y1=round(y1, 1), long_side_px=round(max(x1 - x0, y1 - y0), 1), blob_px=c[4], contrast=round(c[5], 1), clutter_near=round(c[6], 3)))
            if len(crops) < 4000:
                crops.append((i, f, (x0, y0, x1, y1)))
            if i >= max(want):
                break
    n_pos = len(rows)
    n_neg_cap = int(round(a.neg_frac / (1 - a.neg_frac) * n_pos))
    step = max(1, len(pre) // max(1, n_neg_cap)) if n_neg_cap else len(pre) + 1
    n_neg = 0
    for i, t, f in pre[::step][:n_neg_cap]:
        name = f"n{i:06d}"
        cv2.imwrite(f"{out}/images/{name}.jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 95])
        open(f"{out}/labels/{name}.txt", "w").close()
        rows.append(dict(name=name, frame=i, video_s=round(t, 3), kind="negative"))
        n_neg += 1
    keys = ["name", "frame", "video_s", "kind", "track", "x0", "y0", "x1", "y1", "long_side_px", "blob_px", "contrast", "clutter_near"]
    with open(f"{out}/index.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, keys); w.writeheader(); w.writerows(rows)
    sides = [r["long_side_px"] for r in rows if r["kind"] == "positive"]
    bins = {"under_16": 0, "16_to_32": 0, "32_to_64": 0, "64_and_up": 0}
    for s in sides:
        bins["under_16" if s < 16 else "16_to_32" if s < 32 else "32_to_64" if s < 64 else "64_and_up"] += 1
    summary = dict(
        frames_decoded=last_i + 1, video_s_decoded=round(t_of(last_i), 1), frame_size=[W, H],
        background_frames=nbg, clutter_mask_fraction=round(float(clutter.mean()), 4),
        flight_frames_examined=n_flight, flight_frames_with_any_candidate=sum(1 for _, c in per_frame if c),
        clutter=cinfo, tracks_total=len(tracks), tracks_qualified=len(good), tracks_rejected=why,
        flight_frames_with_one_track=len(single), flight_frames_ambiguous_dropped=ambiguous,
        flight_frames_box_cut_by_clutter_dropped=cut_by_clutter,
        flight_frames_no_track_dropped=n_flight - len(by_frame),
        positives_written=n_pos, negatives_written=n_neg, label_fps=a.label_fps,
        positive_long_side_px_bins=bins, labels_from="motion against a static background; no detector",
        settings=dict(thr=thr, min_area=a.min_area, max_side=a.max_side, stride=a.stride, coast=a.coast, min_obs=a.min_obs,
                      min_travel=a.min_travel, flight_start=a.flight_start, flight_end=a.flight_end, merge_px=a.merge_px,
                      max_clutter=a.max_clutter, clutter_stride=a.clutter_stride, clutter_segments=a.clutter_segments, clutter_grow=a.clutter_grow),
    )
    qa_sheets(crops, out)
    return summary, rows


def qa_sheets(crops, out, per=48, cols=8, tile=200):
    if not crops:
        return
    pick = [crops[int(k)] for k in np.linspace(0, len(crops) - 1, min(len(crops), per * 4))]
    for s in range(0, len(pick), per):
        tiles = []
        for i, f, (x0, y0, x1, y1) in pick[s:s + per]:
            cx, cy, r = (x0 + x1) / 2, (y0 + y1) / 2, max(48, 1.6 * max(x1 - x0, y1 - y0))
            ax, ay = int(max(0, cx - r)), int(max(0, cy - r))
            crop = f[ay:int(cy + r), ax:int(cx + r)].copy()
            cv2.rectangle(crop, (int(x0) - ax, int(y0) - ay), (int(x1) - ax, int(y1) - ay), (0, 255, 0), 1)
            crop = cv2.resize(crop, (tile, tile), interpolation=cv2.INTER_NEAREST)
            cv2.putText(crop, f"{i} {int(max(x1 - x0, y1 - y0))}px", (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
            tiles.append(crop)
        while len(tiles) % cols:
            tiles.append(np.zeros((tile, tile, 3), np.uint8))
        cv2.imwrite(f"{out}/qa/labels_{s // per:02d}.jpg", np.vstack([np.hstack(tiles[r:r + cols]) for r in range(0, len(tiles), cols)]))


def teacher_agreement(out, rows, weights, conf=0.10):
    """How often an independent detector fires on the labelled target. A report, never a filter."""
    from ultralytics import YOLO
    model = YOLO(weights)
    pos = [r for r in rows if r["kind"] == "positive"]
    hit = 0
    by_bin = {}
    for r in pos:
        im = cv2.imread(f"{out}/images/{r['name']}.jpg")
        H, W = im.shape[:2]
        cx, cy = (r["x0"] + r["x1"]) / 2, (r["y0"] + r["y1"]) / 2
        ax, ay = int(min(max(0, cx - 320), W - 640)), int(min(max(0, cy - 320), H - 640))
        res = model.predict(im[ay:ay + 640, ax:ax + 640], imgsz=640, conf=conf, verbose=False)[0]
        ok = False
        for b in res.boxes.xyxy.cpu().numpy():
            bx0, by0, bx1, by1 = b[0] + ax, b[1] + ay, b[2] + ax, b[3] + ay
            iw, ih = min(bx1, r["x1"]) - max(bx0, r["x0"]), min(by1, r["y1"]) - max(by0, r["y0"])
            if iw > 0 and ih > 0:
                inter = iw * ih
                if inter / ((bx1 - bx0) * (by1 - by0) + (r["x1"] - r["x0"]) * (r["y1"] - r["y0"]) - inter) >= 0.3:
                    ok = True
        s = r["long_side_px"]
        k = "under_16" if s < 16 else "16_to_32" if s < 32 else "32_to_64" if s < 64 else "64_and_up"
        by_bin.setdefault(k, [0, 0])
        by_bin[k][0] += ok; by_bin[k][1] += 1
        hit += ok
    return dict(weights=os.path.basename(weights), conf=conf, iou=0.3, labelled=len(pos), teacher_hits=hit,
                agreement=round(hit / len(pos), 3) if pos else None, by_long_side_px={k: f"{v[0]}/{v[1]}" for k, v in by_bin.items()})


def self_test() -> int:
    rng = np.random.default_rng(0)
    base = cv2.blur(rng.integers(120, 200, (300, 400), dtype=np.uint8), (9, 9))
    truth = {}

    def gen():
        for i in range(400):
            g = np.clip(base.astype(np.int16) + rng.integers(-3, 4, base.shape), 0, 255).astype(np.uint8)
            if i >= 150:
                x, y = 40 + (i - 150) * 1.1, 60 + 20 * np.sin((i - 150) / 25)
                g[int(y):int(y) + 8, int(x):int(x) + 14] = 30
                truth[i] = (int(x), int(y), int(x) + 14, int(y) + 8)
            yield i, cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)

    a = argparse.Namespace(thr=30.0, flight_start=2.5, flight_end=1e9, guard_s=0.2, neg_every_s=0.1, stride=1, min_area=12, max_side=200,
                           coast=15, min_obs=20, min_travel=40.0, label_fps=10.0, neg_frac=0.2,
                           merge_px=4, max_clutter=0.08, clutter_stride=2, clutter_segments=3, clutter_grow=3)
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        s, rows = run(gen, lambda i: i / 60.0, a, d)
        pos = [r for r in rows if r["kind"] == "positive"]
        inside, ratio = [], []
        for r in pos:                   # the label is the blob plus padding: it must CONTAIN the truth and not balloon
            t = truth[r["frame"]]
            inside.append(r["x0"] <= t[0] + 1 and r["y0"] <= t[1] + 1 and r["x1"] >= t[2] - 1 and r["y1"] >= t[3] - 1)
            ratio.append((r["x1"] - r["x0"]) * (r["y1"] - r["y0"]) / (14 * 8))
        ok = (len(pos) >= 20 and all(inside) and max(ratio) <= 3.0 and s["negatives_written"] > 0
              and all(r["frame"] < 150 for r in rows if r["kind"] == "negative"))
        print(f"self-test: positives {len(pos)}, truth inside the box in {sum(inside)}/{len(pos)}, box area at most {max(ratio) if ratio else 0:.1f}x truth, "
              f"negatives {s['negatives_written']} (all before the target appears) -> {'PASS' if ok else 'FAIL'}")
        return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--video"); ap.add_argument("--pts", help="frames.pts from record_video.py (ms per frame)")
    ap.add_argument("--fps", type=float, default=60.0, help="used only without --pts")
    ap.add_argument("--flight-start", type=float, help="video seconds: the target is NOT in the air before this")
    ap.add_argument("--flight-end", type=float, default=1e9)
    ap.add_argument("--guard-s", type=float, default=10.0, help="stop taking background/negatives this long before --flight-start")
    ap.add_argument("--out")
    ap.add_argument("--thr", type=float, default=30.0, help="levels of change from the background, largest colour channel")
    ap.add_argument("--min-area", type=int, default=12); ap.add_argument("--max-side", type=int, default=400)
    ap.add_argument("--stride", type=int, default=2); ap.add_argument("--coast", type=int, default=30, help="frames a track may go unseen")
    ap.add_argument("--min-obs", type=int, default=20); ap.add_argument("--min-travel", type=float, default=120.0, help="px a track must cover")
    ap.add_argument("--merge-px", type=int, default=10, help="join blobs closer than about twice this (arms, props and body are one aircraft)")
    ap.add_argument("--max-clutter", type=float, default=0.08, help="largest share of a box and its rim that may be clutter mask")
    ap.add_argument("--clutter-stride", type=int, default=12); ap.add_argument("--clutter-segments", type=int, default=3, help="of 10 stretches of the lead-in a pixel must change in to count as clutter")
    ap.add_argument("--clutter-grow", type=int, default=8)
    ap.add_argument("--label-fps", type=float, default=5.0); ap.add_argument("--neg-every-s", type=float, default=1.0)
    ap.add_argument("--neg-frac", type=float, default=0.18, help="negatives as a fraction of the output (15 to 20 percent rule)")
    ap.add_argument("--teacher", help="detector weights, used ONLY to report agreement")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        return self_test()
    if not (a.video and a.out and a.flight_start is not None):
        ap.error("--video, --out and --flight-start are required (or --self-test)")
    pts = [float(x) / 1000.0 for x in open(a.pts).read().split()] if a.pts else None
    t_of = (lambda i: pts[i] if i < len(pts) else pts[-1] + (i - len(pts) + 1) / a.fps) if pts else (lambda i: i / a.fps)
    summary, rows = run(lambda: read_frames(a.video), t_of, a, a.out)
    if a.teacher and summary["positives_written"]:
        summary["teacher_report"] = teacher_agreement(a.out, rows, a.teacher)
    summary["video"] = a.video
    json.dump(summary, open(f"{a.out}/summary.json", "w"), indent=1)
    print(json.dumps(summary, indent=1))
    if summary["positives_written"] == 0:
        print("FAIL: no positives written", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
