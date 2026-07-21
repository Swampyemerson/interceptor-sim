#!/usr/bin/env python3
"""P0.6 -- the $0 free-footage NN kill-test for the outdoor-drone TRANSFER BET.

WHAT / WHY (read this before trusting any number below)
-------------------------------------------------------
build_plan P0, constraint `transfer-bet`: this project's whole terminal seeker
rests on a bet that a drone-detection NN trained in *simulation*
(`drone_finetuned_quad_v2.onnx`, ADR-0058, deployed) will still put a box on a
REAL drone against a REAL outdoor sky. That bet is currently "hope past
evidence" -- every frame the deployed seeker has ever seen is a Gazebo render.
This script runs the CHEAPEST possible read on that bet BEFORE the tripod day:
fetch permissively-licensed public OUTDOOR footage (drone-as-subject clips +
drone-free sky/bird/ground clips), then run BOTH the deployed sim-native seeker
AND the shortlisted candidate outdoor NNs over the frames and count how often
each fires.

HONESTY -- this is a SMOKE READ, not a curve. THREE hard limits, stated up front
so no reader over-reads it:
  1. The footage is UNLABELED. We therefore report *detection density*
     (fraction of frames with >=1 fire), NEVER "recall". On a drone-present
     segment higher density = the bet looks more alive; on a drone-free segment
     that same number IS the false-fire rate. We cannot compute precision/recall
     without per-frame ground-truth boxes we do not have.
  2. The positive VIDEO segments are ground-observer clips where the drone's
     per-frame visibility varies (it drifts, goes distant, leaves frame). So a
     low density there is a LOOSE lower bound, not a miss rate. The contact
     sheet (docs/images/) is the human check -- eyeball the boxes.
  3. mAP / frame-density does NOT predict in-flight recall in this repo
     (candidate_nn_shortlist.md Sec 5, ADR-0061). The tripod day + the
     detect-then-track streak gate (markerless_loop.py) measure the real thing.
     Nothing here upgrades to "the seeker works".

HONESTY BOUNDARY (CLAUDE.md gt_*): this script reads camera pixels only. There
are NO gt_* labels anywhere in the loop -- the footage has no ground truth to
leak. Detection is pixels-in, boxes-out, exactly like the live seeker.

SIM-FREE: no PX4/Gazebo/MAVSDK. Inference is onnxruntime+cv2 in .venv-seeker.
Footage fetch is Wikimedia Commons (no key, clear per-file CC/PD licensing,
recorded in a provenance manifest) + ffmpeg frame extraction.

USAGE
-----
  # 1. fetch footage (needs network; ~<150 MB; writes a provenance manifest):
  .venv-seeker/bin/python scripts/seeker/transfer_bet_test.py --fetch

  # 2. run all seekers over the footage -> CSVs + contact sheet:
  .venv-seeker/bin/python scripts/seeker/transfer_bet_test.py --run

  # do both:
  .venv-seeker/bin/python scripts/seeker/transfer_bet_test.py --fetch --run

  # self-test (NO network, NO real footage -- synthetic frames end-to-end):
  .venv-seeker/bin/python scripts/seeker/transfer_bet_test.py --self-test

Operating point: conf 0.25 (finetuned_seeker.py deployed default,
v3_onnx_infer.DEPLOY_CONF), self-mask OFF. The sim self-mask is a fixed-camera
geometry gate (30 deg bearing on 1280x960 sim intrinsics); it is meaningless on
arbitrary-resolution real footage where the drone can sit anywhere in frame, so
we measure the RAW detector, and say so.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import urllib.parse
import urllib.request

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v3_onnx_infer as v3  # reuse the byte-identical deployed letterbox/decode/session load

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
WEIGHTS = os.path.join(HERE, "weights")
FOOTAGE = os.path.join(HERE, "data", "transfer_bet_footage")  # gitignored (data/)
LOGS = os.path.join(REPO, "logs")
IMAGES = os.path.join(REPO, "docs", "images")
MANIFEST = os.path.join(FOOTAGE, "manifest.json")

CONF = v3.DEPLOY_CONF  # 0.25 -- deployed operating point

COMMONS_API = "https://commons.wikimedia.org/w/api.php"
UA = "interceptor-sim-transfer-test/1.0 (portfolio counter-UAS research; emersonium2@gmail.com)"

# ---------------------------------------------------------------------------
# Model registry -- the deployed sim-native seeker + the shortlisted candidates
# ---------------------------------------------------------------------------
# color is BGR (cv2). 'drone_idx' is the Drone class index for multiclass nets.
MODELS = [
    {"key": "v2_deployed", "file": "drone_finetuned_quad_v2.onnx", "kind": "single",
     "color": (0, 255, 0), "name": "deployed v2 (sim-native, ADR-0058)"},
    {"key": "flyingobj", "file": "flying_objects_yolov8m.onnx", "kind": "multi", "drone_idx": 0,
     "color": (0, 165, 255), "name": "cand flying_objects_yolov8m (MIT, real photos)"},
    {"key": "yolo11x", "file": "drone_yolo11x_1280.onnx", "kind": "single",
     "color": (255, 0, 255), "name": "cand drone_yolo11x (MIT, ground-observer real)"},
]

# ---------------------------------------------------------------------------
# Footage plan. Each segment is a labeled group of frames fetched from Commons
# by search query, license-filtered, provenance recorded. label 'drone' = the
# target is present in the clip; label 'none' = drone-free (false-fire test).
# The FPV aerial-POV clips (shot BY a drone, drone NOT in frame) are deliberately
# excluded -- see candidate_nn_shortlist.md Sec 3 disambiguation caveat.
# ---------------------------------------------------------------------------
SEGMENTS = [
    # POSITIVES (drone visible in frame) --------------------------------------
    {"id": "pos_photo_dji", "label": "drone", "media": "image",
     "query": "DJI Phantom quadcopter in flight", "n": 4,
     "note": "close-range drone-as-subject product/flight photos"},
    {"id": "pos_photo_sky", "label": "drone", "media": "image",
     "query": "quadcopter drone flying sky", "n": 4,
     "note": "small-quad-in-sky, ground-observer aspect (closest to deployment)"},
    {"id": "pos_video_blm", "label": "drone", "media": "video",
     "query": "BLM drone training", "n": 2, "fps": 0.5, "max_frames_per_video": 18,
     "note": "PD ground-observer training clips; per-frame drone visibility VARIES"},
    # NEGATIVES (drone-free -- false-fire test) --------------------------------
    {"id": "neg_bird", "label": "none", "media": "image",
     "query": "bird flying blue sky", "n": 6, "note": "the classic drone/bird confuser"},
    {"id": "neg_seagull", "label": "none", "media": "image",
     "query": "seagull in flight", "n": 4, "note": "more birds-in-sky"},
    {"id": "neg_sky", "label": "none", "media": "image",
     "query": "blue sky white clouds", "n": 4, "note": "empty sky / clouds"},
    {"id": "neg_ground", "label": "none", "media": "image",
     "query": "green meadow field landscape sky", "n": 3, "note": "ground / horizon clutter"},
]

PER_FILE_MAX_BYTES = 45 * 1024 * 1024   # skip any single source file over 45 MB
TOTAL_BUDGET_BYTES = 450 * 1024 * 1024  # hard stop well under the task's 500 MB


# ===========================================================================
# License filter
# ===========================================================================
def license_ok(lic: str) -> bool:
    """Accept only permissive licenses; reject NC/ND/non-free/unknown.
    PD, CC0, CC BY, CC BY-SA, and Free Art License are accepted."""
    if not lic:
        return False
    s = lic.strip().lower()
    if any(bad in s for bad in ("-nc", "-nd", "non-free", "fair use", "noncommercial")):
        return False
    return (
        "public domain" in s or "cc0" in s or s.startswith("fal") or "free art" in s
        or s.startswith("cc by")
    )


# ===========================================================================
# FETCH
# ===========================================================================
def _api_search(query: str, limit: int):
    """Return a list of {title,url,size,mime,license,artist,descurl} for a
    Commons file search, license-filtered, smallest-first-ish (search order)."""
    params = {
        "action": "query", "format": "json", "generator": "search",
        "gsrsearch": query, "gsrnamespace": "6", "gsrlimit": str(limit * 3 + 6),
        "prop": "imageinfo",
        "iiprop": "url|size|mime|extmetadata",
    }
    url = COMMONS_API + "?" + urllib.parse.urlencode(params)
    req = urllib.request.Request(url, headers={"User-Agent": UA})
    with urllib.request.urlopen(req, timeout=45) as resp:
        data = json.load(resp)
    out = []
    for p in data.get("query", {}).get("pages", {}).values():
        ii = (p.get("imageinfo") or [{}])[0]
        em = ii.get("extmetadata", {})
        lic = em.get("LicenseShortName", {}).get("value", "")
        artist = em.get("Artist", {}).get("value", "")
        # strip any HTML tags out of the artist field
        artist = _strip_html(artist)
        out.append({
            "title": p.get("title", ""), "url": ii.get("url", ""),
            "size": int(ii.get("size", 0) or 0), "mime": ii.get("mime", ""),
            "license": lic, "artist": artist,
            "descurl": ii.get("descriptionurl", ""),
            "index": p.get("index", 0),
        })
    out.sort(key=lambda d: d.get("index", 0))
    return out


def _strip_html(s: str) -> str:
    import re
    return re.sub(r"<[^>]+>", "", s or "").strip()


def _download(url: str, dest: str) -> int:
    req = urllib.request.Request(url, headers={"User-Agent": UA})
    with urllib.request.urlopen(req, timeout=120) as resp, open(dest, "wb") as fh:
        while True:
            chunk = resp.read(1 << 16)
            if not chunk:
                break
            fh.write(chunk)
    return os.path.getsize(dest)


def _extract_video_frames(video_path: str, out_dir: str, fps: float, max_frames: int, stem: str):
    """ffmpeg -> PNG frames at `fps`, capped at max_frames. Returns list of paths."""
    os.makedirs(out_dir, exist_ok=True)
    patt = os.path.join(out_dir, f"{stem}_%03d.png")
    cmd = ["ffmpeg", "-nostdin", "-loglevel", "error", "-y", "-i", video_path,
           "-vf", f"fps={fps}", "-frames:v", str(max_frames), patt]
    subprocess.run(cmd, check=True)
    return sorted(
        os.path.join(out_dir, f) for f in os.listdir(out_dir)
        if f.startswith(stem + "_") and f.endswith(".png")
    )


def fetch(verbose: bool = True) -> dict:
    os.makedirs(FOOTAGE, exist_ok=True)
    manifest = {"fetched_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "conf": CONF, "segments": {}, "sources": []}
    total = 0
    for seg in SEGMENTS:
        seg_dir = os.path.join(FOOTAGE, seg["id"])
        os.makedirs(seg_dir, exist_ok=True)
        seg_frames = []
        want = seg["n"]
        got = 0
        if verbose:
            print(f"\n[fetch] segment {seg['id']} ({seg['label']}, {seg['media']}) "
                  f"query={seg['query']!r} want={want}")
        try:
            candidates = _api_search(seg["query"], want)
        except Exception as e:  # noqa: BLE001
            print(f"[fetch]   search FAILED: {e}")
            continue
        want_mime = "video" if seg["media"] == "video" else "image"
        for c in candidates:
            if got >= want:
                break
            if not c["mime"].startswith(want_mime):
                continue
            if not license_ok(c["license"]):
                if verbose:
                    print(f"[fetch]   skip (license {c['license']!r}): {c['title']}")
                continue
            if c["size"] <= 0 or c["size"] > PER_FILE_MAX_BYTES:
                if verbose:
                    print(f"[fetch]   skip (size {c['size']}): {c['title']}")
                continue
            if total + c["size"] > TOTAL_BUDGET_BYTES:
                print("[fetch]   TOTAL BUDGET reached; stopping.")
                break
            ext = os.path.splitext(c["url"])[1].lower() or (".webm" if want_mime == "video" else ".jpg")
            safe = f"{seg['id']}_{got:02d}{ext}"
            raw_path = os.path.join(seg_dir, "_raw_" + safe)
            try:
                nbytes = _download(c["url"], raw_path)
            except Exception as e:  # noqa: BLE001
                print(f"[fetch]   download FAILED {c['title']}: {e}")
                continue
            total += nbytes
            sha = hashlib.sha256(open(raw_path, "rb").read()).hexdigest()[:12]
            src = {"segment": seg["id"], "label": seg["label"], "title": c["title"],
                   "license": c["license"], "artist": c["artist"], "url": c["url"],
                   "descurl": c["descurl"], "bytes": nbytes, "sha256_12": sha}
            manifest["sources"].append(src)
            if verbose:
                print(f"[fetch]   OK {c['title']}  [{c['license']}]  {nbytes//1024} KB")
            if want_mime == "video":
                try:
                    vframes = _extract_video_frames(
                        raw_path, seg_dir, seg.get("fps", 0.5),
                        seg.get("max_frames_per_video", 18), f"{seg['id']}_{got:02d}")
                    seg_frames.extend(vframes)
                except Exception as e:  # noqa: BLE001
                    print(f"[fetch]   ffmpeg FAILED {c['title']}: {e}")
                    continue
            else:
                # normalize the image to a .png frame the runner globs uniformly
                img = cv2.imread(raw_path)
                if img is None:
                    print(f"[fetch]   cv2 could not read {raw_path}; skipping")
                    continue
                frame_path = os.path.join(seg_dir, f"{seg['id']}_{got:02d}.png")
                cv2.imwrite(frame_path, img)
                seg_frames.append(frame_path)
            got += 1
        manifest["segments"][seg["id"]] = {
            "label": seg["label"], "media": seg["media"], "note": seg["note"],
            "query": seg["query"], "n_sources": got, "n_frames": len(seg_frames),
        }
        if verbose:
            print(f"[fetch]   -> {got} source(s), {len(seg_frames)} frame(s)")
    manifest["total_bytes"] = total
    with open(MANIFEST, "w") as fh:
        json.dump(manifest, fh, indent=2)
    print(f"\n[fetch] wrote {MANIFEST}  (total {total/1024/1024:.1f} MB, "
          f"{len(manifest['sources'])} source files)")
    return manifest


# ===========================================================================
# INFERENCE
# ===========================================================================
def load_models():
    """Load every registry model whose ONNX is present. Returns list of dicts
    with a live session; skips (with a note) any missing weight."""
    loaded = []
    for m in MODELS:
        path = os.path.join(WEIGHTS, m["file"])
        if not os.path.exists(path):
            print(f"[models] MISSING {m['file']} -- skipping {m['key']}")
            continue
        sess, iname, imgsz = v3.load_session(path)
        classes = []
        cf = os.path.splitext(path)[0] + ".classes.txt"
        if os.path.exists(cf):
            classes = [ln.strip() for ln in open(cf) if ln.strip()]
        mm = dict(m)
        mm.update({"sess": sess, "iname": iname, "imgsz": imgsz, "classes": classes})
        loaded.append(mm)
        print(f"[models] loaded {m['key']:12s} imgsz={imgsz} kind={m['kind']} "
              f"classes={classes if classes else '[drone]'}")
    if not loaded:
        raise RuntimeError("no ONNX weights found under scripts/seeker/weights/")
    return loaded


def _infer_multi(m, frame_bgr, conf=CONF):
    """Multiclass YOLOv8 decode (letterbox-undone, original-frame px). Returns
    (drone_dets, any_dets) each a list of {cx,cy,w,h,score,cls}."""
    img, r, (pad_l, pad_t) = v3._letterbox(frame_bgr, m["imgsz"])
    blob = img[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    out = m["sess"].run(None, {m["iname"]: np.ascontiguousarray(blob)})[0][0]
    if out.shape[0] < out.shape[1]:
        out = out.T  # (N, C)
    cls = out[:, 4:]
    scores = cls.max(1)
    args = cls.argmax(1)
    keep = np.where(scores >= conf)[0]
    drone, anyd = [], []
    for i in keep:
        cx_, cy_, w_, h_ = out[i, :4]
        d = {"cx": float((cx_ - pad_l) / r), "cy": float((cy_ - pad_t) / r),
             "w": float(w_ / r), "h": float(h_ / r),
             "score": float(scores[i]), "cls": int(args[i])}
        anyd.append(d)
        if int(args[i]) == m.get("drone_idx", 0):
            drone.append(d)
    return drone, anyd


def eval_frame(frame_bgr, models, conf=CONF):
    """Run every model on one frame. Returns {key: {...}} with drone+any dets."""
    res = {}
    for m in models:
        if m["kind"] == "single":
            dets = v3.infer_raw(m["sess"], m["iname"], m["imgsz"], frame_bgr, conf=conf)
            for d in dets:
                d["cls"] = 0
            drone, anyd = dets, dets
        else:
            drone, anyd = _infer_multi(m, frame_bgr, conf=conf)
        top_d = max(drone, key=lambda d: d["score"]) if drone else None
        top_a = max(anyd, key=lambda d: d["score"]) if anyd else None
        res[m["key"]] = {"drone": drone, "any": anyd, "top_drone": top_d, "top_any": top_a}
    return res


def _cls_name(m, idx):
    if m["kind"] == "single":
        return "drone"
    if 0 <= idx < len(m.get("classes", [])):
        return m["classes"][idx]
    return str(idx)


# ===========================================================================
# RUN over the fetched footage
# ===========================================================================
def _list_segment_frames():
    """From the manifest (or a bare footage dir), yield (segment, label, media,
    [frame_paths]) for every segment that has frames on disk."""
    labels = {s["id"]: (s["label"], s["media"]) for s in SEGMENTS}
    if os.path.exists(MANIFEST):
        man = json.load(open(MANIFEST))
        for sid, info in man.get("segments", {}).items():
            labels[sid] = (info["label"], info["media"])
    result = []
    if not os.path.isdir(FOOTAGE):
        return result
    for sid in sorted(os.listdir(FOOTAGE)):
        seg_dir = os.path.join(FOOTAGE, sid)
        if not os.path.isdir(seg_dir):
            continue
        frames = sorted(
            os.path.join(seg_dir, f) for f in os.listdir(seg_dir)
            if f.endswith(".png") and not f.startswith("_raw_"))
        if not frames:
            continue
        label, media = labels.get(sid, ("unknown", "image"))
        result.append((sid, label, media, frames))
    return result


def run(conf=CONF):
    os.makedirs(LOGS, exist_ok=True)
    os.makedirs(IMAGES, exist_ok=True)
    models = load_models()
    segments = _list_segment_frames()
    if not segments:
        raise RuntimeError(
            f"no footage frames under {FOOTAGE} -- run --fetch first (needs network).")

    ts = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    perframe_csv = os.path.join(LOGS, f"transfer_bet_perframe_{ts}.csv")
    summary_csv = os.path.join(LOGS, f"transfer_bet_summary_{ts}.csv")

    per_rows = []
    # keep per-frame results for contact-sheet sampling
    frame_cache = []  # (segment, label, media, frame_path, results)
    for sid, label, media, frames in segments:
        print(f"\n[run] {sid} ({label}, {media}): {len(frames)} frames")
        for fp in frames:
            img = cv2.imread(fp)
            if img is None:
                continue
            results = eval_frame(img, models, conf=conf)
            frame_cache.append((sid, label, media, fp, results))
            for m in models:
                r = results[m["key"]]
                td, ta = r["top_drone"], r["top_any"]
                per_rows.append({
                    "segment": sid, "label": label, "media": media,
                    "frame": os.path.basename(fp), "model": m["key"],
                    "n_det_drone": len(r["drone"]),
                    "top_drone_score": round(td["score"], 4) if td else 0.0,
                    "n_det_anyclass": len(r["any"]),
                    "top_any_score": round(ta["score"], 4) if ta else 0.0,
                    "top_any_class": _cls_name(m, ta["cls"]) if ta else "",
                    "top_cx": round(td["cx"], 1) if td else "",
                    "top_cy": round(td["cy"], 1) if td else "",
                    "top_w": round(td["w"], 1) if td else "",
                    "top_h": round(td["h"], 1) if td else "",
                })

    _write_csv(perframe_csv, per_rows, [
        "segment", "label", "media", "frame", "model", "n_det_drone",
        "top_drone_score", "n_det_anyclass", "top_any_score", "top_any_class",
        "top_cx", "top_cy", "top_w", "top_h"])

    # per-segment x model summary
    sum_rows = _summarize(per_rows)
    _write_csv(summary_csv, sum_rows, [
        "segment", "label", "model", "n_frames", "n_frames_with_drone_det",
        "drone_det_density", "mean_top_drone_score", "median_top_drone_score",
        "n_frames_with_anyclass_det", "anyclass_fire_density"])

    sheet_path = os.path.join(IMAGES, "transfer_bet_contact_sheet.png")
    build_contact_sheet(frame_cache, models, sheet_path, n_cells=20)

    print(f"\n[run] wrote:\n  {perframe_csv}\n  {summary_csv}\n  {sheet_path}")
    _print_verdict_table(sum_rows)
    return perframe_csv, summary_csv, sheet_path, sum_rows


def _summarize(per_rows):
    from collections import defaultdict
    groups = defaultdict(list)
    for r in per_rows:
        groups[(r["segment"], r["label"], r["model"])].append(r)
    out = []
    for (seg, label, model), rows in sorted(groups.items()):
        n = len(rows)
        with_d = sum(1 for r in rows if r["n_det_drone"] > 0)
        with_any = sum(1 for r in rows if r["n_det_anyclass"] > 0)
        scores = [r["top_drone_score"] for r in rows if r["n_det_drone"] > 0]
        out.append({
            "segment": seg, "label": label, "model": model, "n_frames": n,
            "n_frames_with_drone_det": with_d,
            "drone_det_density": round(with_d / n, 3) if n else 0.0,
            "mean_top_drone_score": round(float(np.mean(scores)), 3) if scores else 0.0,
            "median_top_drone_score": round(float(np.median(scores)), 3) if scores else 0.0,
            "n_frames_with_anyclass_det": with_any,
            "anyclass_fire_density": round(with_any / n, 3) if n else 0.0,
        })
    return out


def _write_csv(path, rows, cols):
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in cols})


def _print_verdict_table(sum_rows):
    print("\n=== detection density (fraction of frames with a Drone-class fire) ===")
    print(f"{'segment':18s} {'label':6s} {'model':12s} {'frames':>6s} {'density':>8s} {'medscore':>8s}")
    for r in sum_rows:
        print(f"{r['segment']:18s} {r['label']:6s} {r['model']:12s} "
              f"{r['n_frames']:6d} {r['drone_det_density']:8.2f} {r['median_top_drone_score']:8.2f}")


# ===========================================================================
# CONTACT SHEET
# ===========================================================================
def _draw_dets(cell, dets, color, sx, sy):
    for d in dets:
        x0 = int((d["cx"] - d["w"] / 2) * sx); y0 = int((d["cy"] - d["h"] / 2) * sy)
        x1 = int((d["cx"] + d["w"] / 2) * sx); y1 = int((d["cy"] + d["h"] / 2) * sy)
        cv2.rectangle(cell, (x0, y0), (x1, y1), color, 2)


def _sample_frames(frame_cache, n_cells):
    """Deterministic, representative round-robin across segments (not cherry-
    picked): evenly spaced frames per segment until n_cells is filled."""
    from collections import OrderedDict, defaultdict
    by_seg = OrderedDict()
    for item in frame_cache:
        by_seg.setdefault(item[0], []).append(item)
    per = max(1, math.ceil(n_cells / max(1, len(by_seg))))
    picked = []
    for seg, items in by_seg.items():
        if len(items) <= per:
            sel = items
        else:
            step = len(items) / per
            sel = [items[min(len(items) - 1, int(i * step))] for i in range(per)]
        picked.extend(sel)
    # trim to n_cells but keep segment balance (already interleaved-ish by order)
    return picked[:n_cells]


def build_contact_sheet(frame_cache, models, out_path, n_cells=20, cols=5):
    if not frame_cache:
        raise RuntimeError("no frames to build a contact sheet from")
    picked = _sample_frames(frame_cache, n_cells)
    cw, ch = 380, 300  # image area per cell
    hdr = 34           # per-cell header strip
    rows = math.ceil(len(picked) / cols)
    title_h = 70
    grid_w = cols * cw
    grid_h = title_h + rows * (ch + hdr)
    canvas = np.full((grid_h, grid_w, 3), 24, np.uint8)

    # title + legend
    cv2.putText(canvas, "TRANSFER-BET SMOKE READ  (unlabeled real outdoor footage; NOT recall)",
                (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (235, 235, 235), 2, cv2.LINE_AA)
    lx = 12
    for m in models:
        cv2.rectangle(canvas, (lx, 44), (lx + 22, 60), m["color"], -1)
        cv2.putText(canvas, m["key"], (lx + 28, 58), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (235, 235, 235), 1, cv2.LINE_AA)
        lx += 28 + 12 * len(m["key"]) + 24

    for idx, (sid, label, media, fp, results) in enumerate(picked):
        r, c = divmod(idx, cols)
        x0 = c * cw
        y0 = title_h + r * (ch + hdr)
        img = cv2.imread(fp)
        if img is None:
            continue
        H, W = img.shape[:2]
        sx, sy = cw / W, ch / H
        cell = cv2.resize(img, (cw, ch))
        # draw each model's Drone-class boxes in its color
        counts = []
        for m in models:
            dd = results[m["key"]]["drone"]
            _draw_dets(cell, dd, m["color"], sx, sy)
            counts.append(f"{m['key'].split('_')[0]}:{len(dd)}")
        # header strip: label color-coded (pos=green tint, neg=red tint)
        strip_col = (40, 90, 40) if label == "drone" else (40, 40, 90)
        hdr_img = np.full((hdr, cw, 3), strip_col, np.uint8)
        tag = "DRONE" if label == "drone" else "NO-DRONE"
        cv2.putText(hdr_img, f"{tag} {sid}", (4, 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.38, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(hdr_img, "  ".join(counts), (4, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.38, (200, 255, 200), 1, cv2.LINE_AA)
        canvas[y0:y0 + hdr, x0:x0 + cw] = hdr_img
        canvas[y0 + hdr:y0 + hdr + ch, x0:x0 + cw] = cell
        cv2.rectangle(canvas, (x0, y0), (x0 + cw - 1, y0 + hdr + ch - 1), (70, 70, 70), 1)

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    cv2.imwrite(out_path, canvas)
    return out_path


# ===========================================================================
# SELF-TEST -- synthetic frames, NO network, NO real footage
# ===========================================================================
def _synth_frames(tmp):
    """A handful of synthetic frames: plain sky (negatives) + sky-with-blob
    (crude positives). Pixels only; nothing is a real drone -- this exercises
    the PIPELINE (decode -> CSV -> contact sheet), not detection quality."""
    rng = np.random.default_rng(7)
    segs = {"selftest_sky": ("none", 3), "selftest_blob": ("drone", 3)}
    made = []
    for sid, (label, n) in segs.items():
        d = os.path.join(tmp, sid)
        os.makedirs(d, exist_ok=True)
        for i in range(n):
            img = np.zeros((480, 640, 3), np.uint8)
            img[:, :] = (200, 160, 120)  # BGR blueish sky
            img = cv2.add(img, rng.integers(0, 18, img.shape, dtype=np.uint8))
            if label == "drone":
                cx, cy = 320 + i * 20, 240
                cv2.circle(img, (cx, cy), 14, (40, 40, 40), -1)
                cv2.line(img, (cx - 22, cy), (cx + 22, cy), (30, 30, 30), 3)
            p = os.path.join(d, f"{sid}_{i:02d}.png")
            cv2.imwrite(p, img)
            made.append((sid, label, "image", p))
    return segs, made


def self_test() -> int:
    print("[self-test] synthetic-frame end-to-end (no network, no real footage)")
    tmp = tempfile.mkdtemp(prefix="transfer_bet_selftest_")
    ok = True
    try:
        models = load_models()
        segs, made = _synth_frames(tmp)
        # inference + CSV
        per_rows, frame_cache = [], []
        for sid, label, media, fp in made:
            img = cv2.imread(fp)
            assert img is not None, f"could not read synthetic frame {fp}"
            results = eval_frame(img, models)
            frame_cache.append((sid, label, media, fp, results))
            for m in models:
                r = results[m["key"]]
                per_rows.append({
                    "segment": sid, "label": label, "media": media,
                    "frame": os.path.basename(fp), "model": m["key"],
                    "n_det_drone": len(r["drone"]), "n_det_anyclass": len(r["any"]),
                    "top_drone_score": r["top_drone"]["score"] if r["top_drone"] else 0.0,
                })
        exp_rows = len(made) * len(models)
        assert len(per_rows) == exp_rows, f"expected {exp_rows} rows, got {len(per_rows)}"
        csv_path = os.path.join(tmp, "selftest_perframe.csv")
        _write_csv(csv_path, per_rows,
                   ["segment", "label", "media", "frame", "model",
                    "n_det_drone", "n_det_anyclass", "top_drone_score"])
        assert os.path.getsize(csv_path) > 0, "empty CSV"
        # summary
        sum_rows = _summarize([dict(r, top_any_score=0, top_any_class="") for r in per_rows])
        assert sum_rows, "empty summary"
        # contact sheet
        sheet = os.path.join(tmp, "selftest_sheet.png")
        build_contact_sheet(frame_cache, models, sheet, n_cells=6, cols=3)
        assert os.path.exists(sheet) and os.path.getsize(sheet) > 0, "no contact sheet"
        sh = cv2.imread(sheet)
        assert sh is not None and sh.shape[0] > 50 and sh.shape[1] > 50, "bad sheet"
        # every model produced a decodable result on every frame (no crash/shape bug)
        for _, _, _, _, results in frame_cache:
            for m in models:
                assert m["key"] in results, f"model {m['key']} produced no result"
        print(f"[self-test] {len(made)} frames x {len(models)} models = {exp_rows} rows; "
              f"CSV + summary + contact sheet all built.")
    except Exception as e:  # noqa: BLE001
        import traceback
        traceback.print_exc()
        ok = False
    finally:
        import shutil
        shutil.rmtree(tmp, ignore_errors=True)
    print("[self-test] PASS" if ok else "[self-test] FAIL")
    return 0 if ok else 1


# ===========================================================================
# main
# ===========================================================================
def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fetch", action="store_true", help="download footage from Commons (network)")
    ap.add_argument("--run", action="store_true", help="run all seekers over footage -> CSV + sheet")
    ap.add_argument("--self-test", action="store_true", help="synthetic end-to-end check (no network)")
    ap.add_argument("--conf", type=float, default=CONF, help=f"detection conf (default {CONF})")
    args = ap.parse_args()

    if args.self_test:
        return self_test()
    if not (args.fetch or args.run):
        ap.error("nothing to do: pass --fetch and/or --run (or --self-test)")
    if args.fetch:
        fetch()
    if args.run:
        run(conf=args.conf)
    return 0


if __name__ == "__main__":
    sys.exit(main())
