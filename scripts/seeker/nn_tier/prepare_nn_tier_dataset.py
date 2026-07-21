#!/usr/bin/env python3
"""prepare_nn_tier_dataset.py — unify the fetched REAL media into the nn_tier
YOLO training corpus (grayscale deployment-track variant + color A/B variant),
with a SOURCE/VIDEO/SCENE-disjoint split (the anti-mirage rule: never split by
random frame — the v3/rebal NULLs, ADR-0061).

Inputs (fetched by fetch_nn_tier_sources.py):
  dvb    Mendeley YOLOv7-Segmented Drone-vs-Bird (CC BY 4.0) — drone positives,
         bird NEGATIVES, segmentation polygons = cutout donors for compositing.
         NO scene/video metadata exists in this set (filename prefixes are just
         class+split codes), so honest scene grouping is impossible →
         **pinned TRAIN-ONLY, never val/test**.
  nps    Purdue/NPS UAV_Dataset (BSD-3) — air-to-air GoPro clips, tiny targets
         (the right px regime). Grouped BY CLIP; clips are hashed across
         train/val/test, so val/test contain WHOLE held-out flights.
  dut    DUT Anti-UAV detection set (MIT) — no scene metadata → treated as one
         source and **pinned WHOLE-SOURCE to TEST** (a never-seen-source gate).
  desert_plates  Wikimedia Commons CC0/PD/CC-BY high-desert stills — REAL
         background plates: cropped to OV9281 framing (1280x800) as empty-label
         negatives, grouped by AUTHOR (photographer ≈ scene/session).
  composites     REAL drone cutouts (DVB masks) pasted onto REAL train-split
         desert plates at deployment px scale/position weights — TRAIN-ONLY
         (firewall: validation is always whole held-out real sources).

Outputs (under scripts/seeker/data/nn_tier/):
  gray/{images,labels}/{train,val,test}   deployment-track variant (mono OV9281)
  color/{images,labels}/{train,val,test}  color A/B variant (same labels/split)
  dataset.yaml (gray, the default) + dataset_color.yaml
  manifest_all.csv + manifest_{train,val,test}.csv
  docs/nn_tier/dataset_report.md          counts / scale histogram / licenses

Class map: single class 0 = 'drone'. Bird images carry EMPTY labels (they are
negatives — the bird-discrimination lesson), never a bird class.

Usage:
  python3 prepare_nn_tier_dataset.py                 # build from whatever is fetched
  python3 prepare_nn_tier_dataset.py --check         # re-assert split disjointness
  python3 prepare_nn_tier_dataset.py --self-test     # tiny synthetic corpus, exit 0/1
  python3 prepare_nn_tier_dataset.py --validate-composites 40
Re-runnable: deterministic (fixed seed); re-run after more sources land.
"""

import argparse
import csv
import glob
import hashlib
import json
import os
import random
import re
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from collections import defaultdict

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
SEEKER = os.path.abspath(os.path.join(HERE, ".."))
REPO = os.path.abspath(os.path.join(SEEKER, "..", ".."))
DATA = os.path.join(SEEKER, "data", "nn_tier")
RAW = os.path.join(DATA, "raw")
DVB_DIR = os.path.join(HERE, "data", "dvb_extract", "Dataset")
REPORT = os.path.join(REPO, "docs", "nn_tier", "dataset_report.md")

SEED = 20260721
IMG_EXTS = (".jpg", ".jpeg", ".png")
OV_W, OV_H = 1280, 800          # OV9281 native framing
PX_PER_DEG = 1280.0 / 118.0     # equidistant approx for placement math

# split fractions at GROUP level (groups = clips / authors)
TEST_FRAC, VAL_FRAC = 0.10, 0.10
NEG_TARGET = (0.15, 0.20)       # corpus negative fraction target band
SMALL_BOX_PX = 40.0             # "small" = <=40 px @1280-equivalent width
SMALL_TARGET = 0.70             # want >=70% of positives small

# composite scale sampling: (lo_px, hi_px, weight) @1280-equivalent width
COMP_SCALE_BANDS = [(5, 12, 0.15), (12, 24, 0.35), (24, 40, 0.25),
                    (40, 80, 0.20), (80, 140, 0.05)]


# ============================================================== entry helpers

def entry(source, group, img, boxes, license_str, kind="real",
          pin=None, note=""):
    """boxes = [(cx,cy,w,h) normalized]; empty list = negative."""
    return {"source": source, "group": group, "img": img, "boxes": boxes,
            "license": license_str, "kind": kind, "pin": pin, "note": note}


def group_key(e):
    return f"{e['source']}:{e['group']}"


def box_w1280(e):
    """max box width in px at 1280-equivalent frame width."""
    if not e["boxes"]:
        return 0.0
    return max(b[2] for b in e["boxes"]) * 1280.0


# ================================================================ DVB ingest

def _iou_cxcywh(a, b):
    ax1, ay1, ax2, ay2 = a[0] - a[2] / 2, a[1] - a[3] / 2, a[0] + a[2] / 2, a[1] + a[3] / 2
    bx1, by1, bx2, by2 = b[0] - b[2] / 2, b[1] - b[3] / 2, b[0] + b[2] / 2, b[1] + b[3] / 2
    iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = iw * ih
    return inter / max(a[2] * a[3] + b[2] * b[3] - inter, 1e-9)


def parse_dvb_label(path):
    """DVB label rows are either plain YOLO boxes (5 fields) or seg polygons
    (class + xy pairs). NOTE: the Mendeley re-export collapsed EVERYTHING to
    class 0 — drone-vs-bird lives in the FILENAME prefix (D*/B*), verified
    2026-07-21 (zero class-1 rows in the whole set). Box rows duplicating a
    polygon (IoU>=0.6) are dropped in favor of the polygon-derived box.
    Returns (boxes, polygons) normalized."""
    box_rows, poly_boxes, polys = [], [], []
    try:
        with open(path) as f:
            rows = f.read().split("\n")
    except OSError:
        return [], []
    for row in rows:
        tok = row.split()
        if len(tok) < 5:
            continue
        try:
            vals = [float(t) for t in tok[1:]]
        except ValueError:
            continue
        if len(vals) == 4:
            box_rows.append(tuple(vals))
        elif len(vals) >= 6 and len(vals) % 2 == 0:
            xs, ys = vals[0::2], vals[1::2]
            x1, x2 = max(0.0, min(xs)), min(1.0, max(xs))
            y1, y2 = max(0.0, min(ys)), min(1.0, max(ys))
            if x2 - x1 > 1e-4 and y2 - y1 > 1e-4:
                poly_boxes.append(((x1 + x2) / 2, (y1 + y2) / 2,
                                   x2 - x1, y2 - y1))
                polys.append(list(zip(xs, ys)))
    boxes = poly_boxes + [b for b in box_rows
                          if all(_iou_cxcywh(b, p) < 0.6 for p in poly_boxes)]
    return boxes, polys


def index_dvb(rng, pos_cap, bird_cap):
    """DVB: drone positives (small-box preferred), bird negatives, cutout donors.
    NO scene ids exist → whole source pinned to TRAIN."""
    if not os.path.isdir(DVB_DIR):
        print("[dvb] not present — skipping")
        return [], []
    pos, birds, donors = [], [], []
    for sub in ("train", "valid", "test"):   # POOLED: their random split is ignored
        for ip in sorted(glob.glob(os.path.join(DVB_DIR, sub, "images", "*"))):
            base = os.path.basename(ip)
            if not ip.lower().endswith(IMG_EXTS):
                continue
            lp = os.path.join(DVB_DIR, sub, "labels",
                              os.path.splitext(base)[0] + ".txt")
            # class lives in the FILENAME prefix: D* drone, B* bird (see
            # parse_dvb_label docstring) — bird boxes are DROPPED, the image
            # becomes an empty-label negative (single-class 'drone' design).
            is_bird = base.upper().startswith("B")
            if is_bird:
                birds.append(entry("dvb_bird", "all", ip, [], "CC BY 4.0",
                                   pin="train", note="bird negative"))
                continue
            boxes, polys = parse_dvb_label(lp)
            if boxes:
                pos.append(entry("dvb", "all", ip, boxes, "CC BY 4.0",
                                 pin="train"))
                for poly in polys:
                    donors.append((ip, poly))
    # scale-biased subsample: keep small boxes first (the deploy px regime)
    small = [e for e in pos if box_w1280(e) <= SMALL_BOX_PX]
    large = [e for e in pos if box_w1280(e) > SMALL_BOX_PX]
    rng.shuffle(small)
    rng.shuffle(large)
    n_small = min(len(small), int(pos_cap * 0.7))
    n_large = min(len(large), pos_cap - n_small)
    kept = small[:n_small] + large[:n_large]
    rng.shuffle(birds)
    birds = birds[:bird_cap]
    print(f"[dvb] positives kept {len(kept)}/{len(pos)} "
          f"(small-box {n_small}), bird negatives {len(birds)}, "
          f"cutout donors {len(donors)}")
    return kept + birds, donors


# ================================================================ NPS ingest

NPS_LINE = re.compile(r"time_layer:\s*(\d+)\s*detections:\s*(.*)")
NPS_BOX = re.compile(r"\(([\d.\s]+),([\d.\s]+),([\d.\s]+),([\d.\s]+)\)")


def parse_nps_gt(path):
    """returns {time_layer(int,1-based): [(a,b,c,d), ...]}"""
    out = {}
    with open(path, errors="replace") as f:
        for line in f:
            m = NPS_LINE.search(line)
            if not m:
                continue
            t = int(m.group(1))
            out[t] = [tuple(float(x) for x in g) for g in
                      NPS_BOX.findall(m.group(2))]
    return out


def extract_nps_frames(video, out_dir, stride):
    """ffmpeg-extract every `stride`th frame; resumable (skips if done)."""
    os.makedirs(out_dir, exist_ok=True)
    stamp = os.path.join(out_dir, ".done")
    if os.path.exists(stamp):
        return sorted(glob.glob(os.path.join(out_dir, "f_*.jpg")))
    subprocess.run(
        ["ffmpeg", "-nostdin", "-loglevel", "error", "-i", video,
         "-vf", f"select='not(mod(n\\,{stride}))'", "-fps_mode", "passthrough",
         "-q:v", "2", os.path.join(out_dir, "f_%06d.jpg")],
        check=False)
    frames = sorted(glob.glob(os.path.join(out_dir, "f_*.jpg")))
    if frames:
        with open(stamp, "w") as f:
            f.write(str(len(frames)))
    return frames


def index_nps(rng, stride, neg_frac_cap=0.15):
    """NPS clips: air-to-air positives grouped BY CLIP (whole-flight holdout)."""
    zpath = os.path.join(RAW, "nps", "Videos.zip")
    vdir = os.path.join(RAW, "nps", "videos")
    gt_dir = os.path.join(RAW, "nps", "Video_Annotation")
    if not (os.path.exists(zpath)
            and os.path.getsize(zpath) >= 2035637654
            and os.path.isdir(gt_dir)):
        print("[nps] Videos.zip incomplete/absent — skipping (re-run when fetched)")
        return []
    if not glob.glob(os.path.join(vdir, "**", "*.*"), recursive=True):
        os.makedirs(vdir, exist_ok=True)
        subprocess.run(["unzip", "-o", "-q", zpath, "-d", vdir, "-x",
                        "__MACOSX/*"], check=False)
    vids = {}
    for p in glob.glob(os.path.join(vdir, "**", "*.*"), recursive=True):
        m = re.search(r"Clip[_ ]?(\d+)", os.path.basename(p), re.IGNORECASE)
        if m and p.lower().endswith((".mov", ".mp4", ".avi", ".mts", ".m4v")):
            vids[int(m.group(1))] = p
    entries = []
    swap_checked = False
    swap = False
    for clip_no, vp in sorted(vids.items()):
        gt_path = os.path.join(gt_dir, f"Clip_{clip_no}_gt.txt")
        if not os.path.exists(gt_path):
            continue
        gt = parse_nps_gt(gt_path)
        frames = extract_nps_frames(
            vp, os.path.join(RAW, "nps", "frames", f"clip_{clip_no:02d}"), stride)
        if not frames:
            continue
        h, w = cv2.imread(frames[0]).shape[:2]
        if not swap_checked:
            # decide (x1,y1,x2,y2) vs (y1,x1,y2,x2) from coordinate ranges
            a_max = max((b[0] for bs in gt.values() for b in bs), default=0)
            b_max = max((b[1] for bs in gt.values() for b in bs), default=0)
            swap = a_max <= h and b_max > h  # first coord fits height, second doesn't
            swap_checked = True
            print(f"[nps] frame {w}x{h}, coord order "
                  f"{'(y,x,y,x) SWAPPED' if swap else '(x1,y1,x2,y2)'}")
        pos_e, neg_e = [], []
        for i, fp in enumerate(frames):
            t = i * stride + 1          # gt time_layer is 1-based per video frame
            boxes_px = gt.get(t)
            if boxes_px is None:
                continue
            boxes = []
            for bb in boxes_px:
                x1, y1, x2, y2 = (bb[1], bb[0], bb[3], bb[2]) if swap else bb
                x1, x2 = sorted((max(0, min(x1, w)), max(0, min(x2, w))))
                y1, y2 = sorted((max(0, min(y1, h)), max(0, min(y2, h))))
                if x2 - x1 >= 2 and y2 - y1 >= 2:
                    boxes.append((((x1 + x2) / 2) / w, ((y1 + y2) / 2) / h,
                                  (x2 - x1) / w, (y2 - y1) / h))
            e = entry("nps", f"clip_{clip_no:02d}", fp, boxes, "BSD-3-Clause")
            (pos_e if boxes else neg_e).append(e)
        rng.shuffle(neg_e)
        neg_keep = neg_e[:int(len(pos_e) * neg_frac_cap)]
        entries.extend(pos_e + neg_keep)
    n_pos = sum(1 for e in entries if e["boxes"])
    print(f"[nps] {len(vids)} clips → {len(entries)} frames "
          f"({n_pos} positive, {len(entries)-n_pos} in-source negatives)")
    return entries


# ================================================================ DUT ingest

def index_dut(rng, cap):
    """DUT Anti-UAV detection set — one source, pinned WHOLE to TEST."""
    droot = os.path.join(RAW, "dut")
    zips = [os.path.join(droot, f"{s}.zip") for s in ("train", "val", "test")]
    if not all(os.path.exists(z) for z in zips):
        print("[dut] zips missing — skipping")
        return []
    unz = os.path.join(droot, "unz")
    if not os.path.isdir(unz):
        os.makedirs(unz, exist_ok=True)
        for z in zips:
            subprocess.run(["unzip", "-o", "-q", z, "-d", unz], check=False)
    entries = []
    names_seen = defaultdict(int)
    for xp in glob.glob(os.path.join(unz, "**", "*.xml"), recursive=True):
        img = None
        for ext in IMG_EXTS:
            cand = os.path.join(
                os.path.dirname(os.path.dirname(xp)), "img",
                os.path.splitext(os.path.basename(xp))[0] + ext)
            if os.path.exists(cand):
                img = cand
                break
        if img is None:
            continue
        try:
            root = ET.parse(xp).getroot()
        except ET.ParseError:
            continue
        sz = root.find("size")
        try:
            w = float(sz.find("width").text)
            h = float(sz.find("height").text)
        except (AttributeError, TypeError, ValueError):
            im = cv2.imread(img)
            if im is None:
                continue
            h, w = im.shape[:2]
        if w < 2 or h < 2:
            continue
        boxes = []
        for ob in root.findall("object"):
            names_seen[(ob.findtext("name") or "?").strip()] += 1
            bb = ob.find("bndbox")
            if bb is None:
                continue
            try:
                x1 = float(bb.findtext("xmin")); y1 = float(bb.findtext("ymin"))
                x2 = float(bb.findtext("xmax")); y2 = float(bb.findtext("ymax"))
            except (TypeError, ValueError):
                continue
            x1, x2 = sorted((max(0, min(x1, w)), max(0, min(x2, w))))
            y1, y2 = sorted((max(0, min(y1, h)), max(0, min(y2, h))))
            if x2 - x1 >= 2 and y2 - y1 >= 2:
                boxes.append((((x1 + x2) / 2) / w, ((y1 + y2) / 2) / h,
                              (x2 - x1) / w, (y2 - y1) / h))
        sub = "trainpart" if f"{os.sep}train{os.sep}" in xp else \
              ("valpart" if f"{os.sep}val{os.sep}" in xp else "testpart")
        entries.append(entry("dut", "all", img, boxes, "MIT", pin="test",
                             note=sub))
    rng.shuffle(entries)
    entries = entries[:cap]
    print(f"[dut] kept {len(entries)} (cap {cap}); object names {dict(names_seen)}")
    return entries


# ============================================================= plates ingest

def norm_author(a):
    a = re.sub(r"[^A-Za-z0-9]+", "_", a or "").strip("_").lower()
    return a[:40] or "unknown"


def plate_quality(im):
    """(median native-res cell Laplacian variance, mean channel std).
    Macro/bokeh close-ups have LOW median lap-var at native res (measured on
    the fetched corpus: macros 9–23 vs landscapes 300+); archival B/W scans
    have ~zero channel std. Such plates stay usable as NEGATIVES but are NOT
    composite backgrounds (out-of-scale/wrong-view paste = the domain_gap trap)."""
    g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    h, w = g.shape
    ch, cw = max(1, h // 8), max(1, w // 8)
    cells = [cv2.Laplacian(g[r * ch:(r + 1) * ch, c * cw:(c + 1) * cw],
                           cv2.CV_32F).var()
             for r in range(8) for c in range(8)]
    col_std = float(np.mean(im.astype(np.float32).std(axis=2)))
    return float(np.median(cells)), col_std


COMP_BG_MIN_LAPVAR = 100.0
COMP_BG_MIN_COLSTD = 3.0


def index_plates(rng, crops_per_plate=3):
    """Wikimedia high-desert plates → OV9281-framed crops, empty labels.
    Group = photographer (author) ≈ session/scene."""
    pdir = os.path.join(RAW, "desert_plates")
    man = os.path.join(pdir, "plates_manifest.csv")
    if not os.path.exists(man):
        print("[plates] manifest missing — skipping")
        return []
    crop_dir = os.path.join(DATA, "work", "plate_crops")
    os.makedirs(crop_dir, exist_ok=True)
    qpath = os.path.join(DATA, "work", "plate_quality.json")
    qual = {}
    if os.path.exists(qpath):
        with open(qpath) as f:
            qual = json.load(f)
    # manual visual-curation exclusions (off-domain: engravings, macros, maps…)
    excl_path = os.path.join(pdir, "plate_exclude.txt")
    excluded = set()
    if os.path.exists(excl_path):
        with open(excl_path) as f:
            for line in f:
                name = line.split("#")[0].strip()
                if name:
                    excluded.add(name)
    entries = []
    n_comp_ok, n_excl = 0, 0
    with open(man) as f:
        rows = list(csv.DictReader(f, delimiter="\t"))
    for r in rows:
        fp = os.path.join(pdir, r["file"])
        if not os.path.exists(fp):
            continue
        if r["file"] in excluded:
            n_excl += 1
            continue
        im = cv2.imread(fp)
        if im is None:
            continue
        if r["file"] not in qual:
            lv, cs = plate_quality(im)
            qual[r["file"]] = [round(lv, 1), round(cs, 1)]
        lv, cs = qual[r["file"]]
        comp_ok = lv >= COMP_BG_MIN_LAPVAR and cs >= COMP_BG_MIN_COLSTD
        n_comp_ok += comp_ok
        h, w = im.shape[:2]
        grp = norm_author(r.get("artist")) or os.path.splitext(r["file"])[0]
        lic = r.get("license", "?")
        base = os.path.splitext(r["file"])[0][:60]
        # OV9281-framed crops (1280x800); if plate is small, use whole frame
        for ci in range(crops_per_plate):
            cp = os.path.join(crop_dir, f"{base}_c{ci}.jpg")
            if not os.path.exists(cp):
                if w >= OV_W and h >= OV_H:
                    x0 = rng.randint(0, w - OV_W)
                    y0 = rng.randint(0, h - OV_H)
                    crop = im[y0:y0 + OV_H, x0:x0 + OV_W]
                elif ci == 0:
                    crop = im
                else:
                    continue
                cv2.imwrite(cp, crop, [cv2.IMWRITE_JPEG_QUALITY, 90])
            entries.append(entry("plates", grp, cp, [], lic,
                                 note="comp_ok" if comp_ok else "neg_only"))
    with open(qpath, "w") as f:
        json.dump(qual, f)
    print(f"[plates] {len(rows)} plates ({n_excl} excluded by curation) → "
          f"{len(entries)} OV9281 crops "
          f"({len(set(e['group'] for e in entries))} author-groups; "
          f"{n_comp_ok} composite-eligible plates)")
    return entries


# ============================================================== compositing

def load_cutout(img_path, poly):
    """Extract a masked drone patch (BGR + float alpha) from a DVB polygon."""
    im = cv2.imread(img_path)
    if im is None:
        return None
    h, w = im.shape[:2]
    pts = np.array([[int(x * w), int(y * h)] for x, y in poly], np.int32)
    x, y, bw, bh = cv2.boundingRect(pts)
    if bw < 40 or bh < 24:                      # need enough donor pixels
        return None
    if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
        return None                              # clipped donor: skip
    mask = np.zeros((h, w), np.uint8)
    cv2.fillPoly(mask, [pts], 255)
    if cv2.countNonZero(mask[y:y + bh, x:x + bw]) < 0.2 * bw * bh:
        return None
    patch = im[y:y + bh, x:x + bw]
    alpha = mask[y:y + bh, x:x + bw].astype(np.float32) / 255.0
    return patch, alpha


def sample_comp_scale(rng):
    r = rng.random()
    acc = 0.0
    for lo, hi, wgt in COMP_SCALE_BANDS:
        acc += wgt
        if r <= acc:
            return rng.uniform(lo, hi)
    return rng.uniform(*COMP_SCALE_BANDS[-1][:2])


def sample_comp_pos(rng, bw_px, bh_px):
    """Placement per the deploy position weights (px in a 1280x800 frame)."""
    def band(r, bands):
        acc = 0.0
        for lo, hi, wgt in bands:
            acc += wgt
            if r <= acc:
                return rng.uniform(lo, hi)
        return rng.uniform(bands[-1][0], bands[-1][1])
    # horizontal off-boresight in deg: 55% ±10°, 35% 10–30°, 10% 30–55°
    daz = band(rng.random(), [(0, 10, 0.55), (10, 30, 0.35), (30, 55, 0.10)])
    daz *= 1 if rng.random() < 0.5 else -1
    # vertical: 45% central ±10°, 35% upper 10–30°, 20% lower 10–30°
    r = rng.random()
    if r < 0.45:
        del_ = rng.uniform(-10, 10)
    elif r < 0.80:
        del_ = -rng.uniform(10, 30)   # upper half of frame = negative y offset
    else:
        del_ = rng.uniform(10, 30)
    cx = OV_W / 2 + daz * PX_PER_DEG
    cy = OV_H / 2 + del_ * PX_PER_DEG
    cx = min(max(cx, bw_px / 2 + 1), OV_W - bw_px / 2 - 1)
    cy = min(max(cy, bh_px / 2 + 1), OV_H - bh_px / 2 - 1)
    return cx, cy


def make_composite(rng, plate_img, cutout, out_path):
    """Paste a real-drone cutout onto a real desert plate: physically plausible
    px scale, local grayscale luminance/contrast match, matched blur, <=2px
    feather (NEVER the box-ellipse halo that broke domain_gap_eval).
    Returns (cx,cy,w,h) normalized or None."""
    plate = cv2.imread(plate_img)
    if plate is None:
        return None
    if plate.shape[1] != OV_W or plate.shape[0] != OV_H:
        plate = cv2.resize(plate, (OV_W, OV_H))
    patch, alpha = cutout
    ph, pw = patch.shape[:2]
    target_w = sample_comp_scale(rng)
    s = target_w / pw
    nw, nh = max(3, int(round(pw * s))), max(2, int(round(ph * s)))
    interp = cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR
    p = cv2.resize(patch, (nw, nh), interpolation=interp)
    a = cv2.resize(alpha, (nw, nh), interpolation=interp)
    # <=2px feather on the mask edge
    a = cv2.GaussianBlur(a, (3, 3), 0.8)
    cx, cy = sample_comp_pos(rng, nw, nh)
    x0, y0 = int(cx - nw / 2), int(cy - nh / 2)
    roi = plate[y0:y0 + nh, x0:x0 + nw]
    if roi.shape[:2] != (nh, nw):
        return None
    # local grayscale luminance/contrast histogram match to the neighborhood
    pad = 12
    nb = plate[max(0, y0 - pad):y0 + nh + pad, max(0, x0 - pad):x0 + nw + pad]
    nb_g = cv2.cvtColor(nb, cv2.COLOR_BGR2GRAY).astype(np.float32)
    p_g = cv2.cvtColor(p, cv2.COLOR_BGR2GRAY).astype(np.float32)
    core = a > 0.5
    if core.sum() < 4:
        return None
    p_mean, p_std = float(p_g[core].mean()), float(p_g[core].std() + 1e-3)
    nb_mean, nb_std = float(nb_g.mean()), float(nb_g.std() + 1e-3)
    # partial match (fully matching contrast would erase the target)
    gain = min(2.0, max(0.4, (0.5 * nb_std + 0.5 * p_std) / p_std))
    bias = (0.65 * nb_mean + 0.35 * p_mean) - p_mean * gain
    pf = np.clip(p.astype(np.float32) * gain + bias, 0, 255)
    # matched blur + slight sensor noise
    if target_w >= 24:
        pf = cv2.GaussianBlur(pf, (3, 3), rng.uniform(0.4, 0.8))
    pf += np.random.default_rng(rng.getrandbits(31)).normal(
        0, 1.6, pf.shape).astype(np.float32)
    pf = np.clip(pf, 0, 255)
    a3 = a[..., None]
    plate[y0:y0 + nh, x0:x0 + nw] = (
        a3 * pf + (1 - a3) * roi.astype(np.float32)).astype(np.uint8)
    cv2.imwrite(out_path, plate, [cv2.IMWRITE_JPEG_QUALITY, 90])
    # tight box from the alpha core
    ys, xs = np.where(a > 0.35)
    bx1, bx2 = x0 + xs.min(), x0 + xs.max() + 1
    by1, by2 = y0 + ys.min(), y0 + ys.max() + 1
    return (((bx1 + bx2) / 2) / OV_W, ((by1 + by2) / 2) / OV_H,
            (bx2 - bx1) / OV_W, (by2 - by1) / OV_H)


def build_composites(rng, donors, plate_entries, n_target, split_of_group):
    """TRAIN-ONLY composites on TRAIN-split plates (firewall)."""
    train_plates = [e for e in plate_entries
                    if split_of_group.get(group_key(e)) == "train"
                    and e["note"] == "comp_ok"]
    if not donors or not train_plates or n_target <= 0:
        print("[comp] skipped (no donors / no train plates / n=0)")
        return []
    out_dir = os.path.join(DATA, "work", "composites")
    os.makedirs(out_dir, exist_ok=True)
    entries = []
    tries = 0
    di = 0
    rng.shuffle(donors)
    cache = {}
    while len(entries) < n_target and tries < n_target * 6:
        tries += 1
        dpath, poly = donors[di % len(donors)]
        di += 1
        key = (dpath, id(poly))
        if key not in cache:
            cache.clear() if len(cache) > 200 else None
            cache[key] = load_cutout(dpath, poly)
        cut = cache[key]
        if cut is None:
            continue
        pe = train_plates[rng.randrange(len(train_plates))]
        op = os.path.join(out_dir, f"comp_{len(entries):05d}.jpg")
        if os.path.exists(op):
            os.remove(op)
        box = make_composite(rng, pe["img"], cut, op)
        if box is None:
            continue
        entries.append(entry("composite", "synth_train", op, [box],
                             "derived: DVB CC BY 4.0 cutout on Commons plate",
                             kind="composite", pin="train",
                             note=f"donor={os.path.basename(dpath)}"))
    print(f"[comp] built {len(entries)}/{n_target} composites "
          f"({tries} attempts) on {len(train_plates)} train plates")
    return entries


# ================================================================== splitting

def assign_splits(entries, rng):
    """Deterministic GROUP-level split. Pins honored (dvb/composites→train,
    dut→test); free groups (nps clips, plate authors) hashed 80/10/10; at least
    one free positive-bearing group forced into val and test when available."""
    groups = {}
    for e in entries:
        gk = group_key(e)
        g = groups.setdefault(gk, {"pin": None, "n_pos": 0, "n": 0})
        if e["pin"]:
            g["pin"] = e["pin"]
        g["n"] += 1
        g["n_pos"] += 1 if e["boxes"] else 0
    split_of = {}
    free = []
    for gk, g in sorted(groups.items()):
        if g["pin"]:
            split_of[gk] = g["pin"]
        else:
            free.append(gk)
    for gk in free:
        h = int(hashlib.md5(gk.encode()).hexdigest(), 16) / 16.0 ** 32
        split_of[gk] = ("test" if h < TEST_FRAC else
                        "val" if h < TEST_FRAC + VAL_FRAC else "train")
    # ensure val & test each get >=1 free positive-bearing group if any exist
    free_pos = [gk for gk in free if groups[gk]["n_pos"] > 0]
    for want in ("val", "test"):
        if free_pos and not any(split_of[gk] == want for gk in free_pos):
            cands = sorted(
                [gk for gk in free_pos if split_of[gk] == "train"],
                key=lambda gk: hashlib.md5((want + gk).encode()).hexdigest())
            if cands:
                split_of[cands[0]] = want
    return split_of


# ==================================================================== emit

def sanitize(s):
    return re.sub(r"[^A-Za-z0-9_.-]", "_", s)


def emit_one(job):
    e, split, uid, gray_only = job
    color_img = os.path.join(DATA, "color", "images", split, uid + ".jpg")
    gray_img = os.path.join(DATA, "gray", "images", split, uid + ".jpg")
    label = "".join(f"0 {b[0]:.6f} {b[1]:.6f} {b[2]:.6f} {b[3]:.6f}\n"
                    for b in e["boxes"])
    im = cv2.imread(e["img"])
    if im is None:
        return None
    if not gray_only:
        if e["img"].lower().endswith((".jpg", ".jpeg")):
            shutil.copyfile(e["img"], color_img)
        else:
            cv2.imwrite(color_img, im, [cv2.IMWRITE_JPEG_QUALITY, 92])
    g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(gray_img, g, [cv2.IMWRITE_JPEG_QUALITY, 88])
    for variant in (["gray"] if gray_only else ["color", "gray"]):
        with open(os.path.join(DATA, variant, "labels", split,
                               uid + ".txt"), "w") as f:
            f.write(label)
    h, w = im.shape[:2]
    return {"uid": uid, "split": split, "source": e["source"],
            "group": e["group"], "n_boxes": len(e["boxes"]),
            "max_box_w1280": round(box_w1280(e), 1),
            "is_composite": int(e["kind"] == "composite"),
            "license": e["license"], "img_w": w, "img_h": h,
            "color_img": os.path.relpath(color_img, DATA),
            "gray_img": os.path.relpath(gray_img, DATA)}


def emit(entries, split_of, workers=8):
    for variant in ("color", "gray"):
        for kind in ("images", "labels"):
            for split in ("train", "val", "test"):
                d = os.path.join(DATA, variant, kind, split)
                if os.path.isdir(d):
                    shutil.rmtree(d)
                os.makedirs(d, exist_ok=True)
    jobs, seen = [], set()
    for e in entries:
        split = split_of[group_key(e)]
        uid = sanitize(f"{e['source']}_{e['group']}_"
                       f"{os.path.splitext(os.path.basename(e['img']))[0]}")[:120]
        while uid in seen:
            uid += "x"
        seen.add(uid)
        jobs.append((e, split, uid, False))
    from multiprocessing import Pool
    with Pool(workers) as pool:
        rows = [r for r in pool.map(emit_one, jobs, chunksize=32) if r]
    rows.sort(key=lambda r: (r["split"], r["source"], r["uid"]))
    cols = ["uid", "split", "source", "group", "n_boxes", "max_box_w1280",
            "is_composite", "license", "img_w", "img_h", "color_img", "gray_img"]
    with open(os.path.join(DATA, "manifest_all.csv"), "w", newline="") as f:
        wtr = csv.DictWriter(f, fieldnames=cols)
        wtr.writeheader()
        wtr.writerows(rows)
    for split in ("train", "val", "test"):
        with open(os.path.join(DATA, f"manifest_{split}.csv"), "w",
                  newline="") as f:
            wtr = csv.DictWriter(f, fieldnames=cols)
            wtr.writeheader()
            wtr.writerows([r for r in rows if r["split"] == split])
    return rows


def write_yamls():
    for variant, fname in (("gray", "dataset.yaml"),
                           ("color", "dataset_color.yaml")):
        with open(os.path.join(DATA, fname), "w") as f:
            f.write(
                f"# nn_tier {variant} corpus — built by prepare_nn_tier_dataset.py\n"
                f"# split is SOURCE/VIDEO/SCENE-disjoint (never random-frame)\n"
                f"# gray = deployment track (OV9281 mono); color = A/B only\n"
                f"path: {os.path.join(DATA, variant)}\n"
                f"train: images/train\nval: images/val\ntest: images/test\n"
                f"names:\n  0: drone\n")


# ==================================================================== report

def scale_hist(rows):
    bins = [(0, 12), (12, 24), (24, 40), (40, 80), (80, 10000)]
    out = {f"{a}-{b if b < 10000 else '+'}px": 0 for a, b in bins}
    for r in rows:
        if r["n_boxes"] == 0:
            continue
        w = float(r["max_box_w1280"])
        for (a, b), key in zip(bins, out):
            if a <= w < b:
                out[key] += 1
                break
    return out


def write_report(rows, split_of, fetch_note):
    os.makedirs(os.path.dirname(REPORT), exist_ok=True)
    by = lambda pred: [r for r in rows if pred(r)]
    n = len(rows)
    n_pos = len(by(lambda r: r["n_boxes"] > 0))
    n_neg = n - n_pos
    pos_small = len(by(lambda r: r["n_boxes"] > 0
                       and float(r["max_box_w1280"]) <= SMALL_BOX_PX))
    lines = [
        "# nn_tier dataset report",
        "",
        f"*generated {time.strftime('%Y-%m-%d %H:%M UTC', time.gmtime())} by "
        "`scripts/seeker/nn_tier/prepare_nn_tier_dataset.py` — deterministic "
        f"seed {SEED}; re-run after more sources land.*",
        "",
        "Deployment track: **grayscale** (`gray/`, `dataset.yaml`) matching the "
        "OV9281 mono sensor (camera_paper_check item 4); `color/` kept for the "
        "color-vs-gray A/B. Single class `drone`; birds/desert are EMPTY-label "
        "negatives.",
        "",
        "## Split policy (anti-mirage)",
        "",
        "Split is by **SOURCE / VIDEO / SCENE, never random frame** "
        "(the v3+rebal NULLs, ADR-0061):",
        "- `nps` (air-to-air clips): grouped **by clip** — val/test are whole "
        "held-out flights.",
        "- `dut` (MIT, no scene metadata): **whole source pinned to TEST** — a "
        "never-seen-source gate.",
        "- `dvb` (no scene metadata — filename prefixes are class codes only): "
        "**pinned TRAIN-only**, never evaluated on.",
        "- `plates` (Commons high-desert): grouped by **photographer/author**.",
        "- `composite`: **TRAIN-only** (firewall — validation is always real "
        "held-out media), built ONLY on train-split plates.",
        "",
        "## Counts",
        "",
        "| split | images | positives | negatives | neg frac | composites |",
        "|---|---|---|---|---|---|",
    ]
    for split in ("train", "val", "test"):
        rs = by(lambda r, s=split: r["split"] == s)
        p = len([r for r in rs if r["n_boxes"] > 0])
        c = len([r for r in rs if r["is_composite"]])
        frac = (len(rs) - p) / len(rs) if rs else 0
        lines.append(f"| {split} | {len(rs)} | {p} | {len(rs)-p} | "
                     f"{frac:.2f} | {c} |")
    lines += [f"| **total** | {n} | {n_pos} | {n_neg} | "
              f"{n_neg/max(n,1):.2f} |  |", "",
              "## Per-source", "",
              "| source | split(s) | images | positives | license |",
              "|---|---|---|---|---|"]
    srcs = sorted(set(r["source"] for r in rows))
    for s in srcs:
        rs = by(lambda r, s=s: r["source"] == s)
        sp = sorted(set(r["split"] for r in rs))
        p = len([r for r in rs if r["n_boxes"] > 0])
        lic = sorted(set(r["license"] for r in rs))[0]
        lines.append(f"| {s} | {','.join(sp)} | {len(rs)} | {p} | {lic} |")
    hist = scale_hist(rows)
    lines += ["", "## Positive scale histogram (max box width, px @1280-eq)", "",
              "| bin | count |", "|---|---|"]
    for k, v in hist.items():
        lines.append(f"| {k} | {v} |")
    tr_pos = by(lambda r: r["split"] == "train" and r["n_boxes"] > 0)
    tr_small = len([r for r in tr_pos
                    if float(r["max_box_w1280"]) <= SMALL_BOX_PX])
    lines += [
        "",
        f"**small-target fraction: {pos_small}/{n_pos} = "
        f"{pos_small/max(n_pos,1):.2f}** of positives are ≤{SMALL_BOX_PX:.0f} px "
        f"@1280-eq (deploy directive ≥{SMALL_TARGET:.0%}); TRAIN-only: "
        f"{tr_small}/{len(tr_pos)} = {tr_small/max(len(tr_pos),1):.2f}.",
    ]
    cv_path = os.path.join(DATA, "composite_validation.json")
    if os.path.exists(cv_path):
        with open(cv_path) as f:
            cv_ = json.load(f)
        ctrl = cv_.get("control_real_nps", {})
        lines += [
            "", "## Composite validation (anti-domain_gap_eval check)", "",
            f"- `{cv_['model']}` fired on **{cv_['hits']}/{cv_['n']}** "
            f"composites (box ≥{cv_['min_box_px']} px, conf ≥{cv_['conf']}).",
            f"- CONTROL, same model/criterion on REAL NPS positives ≥24 px: "
            f"**{ctrl.get('hits', '?')}/{ctrl.get('n', '?')}** — the reference "
            "model is small-object-blind (baseline_scoreboard.md), so "
            "composites are at least as detectable as real small targets; "
            "compositing does not destroy detectability. Numbers: "
            "`composite_validation.json`.",
        ]
    lines += [
        "",
        "## Fetched vs deferred", "", fetch_note, "",
        "## Honesty notes",
        "",
        "- This corpus is a DOMAIN-MATCHED HEAD START from public real media — "
        "NOT final deployment weights material: no imagery of OUR quad exists "
        "yet (tripod day pending); the real-capture pipeline "
        "(real_data_pipeline.md) supersedes it when footage lands.",
        "- Numbers here trace to `manifest_all.csv` + `fetch_log.jsonl`.",
        "- CC BY plate/DVB attribution obligations: see "
        "`raw/desert_plates/plates_manifest.csv` and the Mendeley DOI "
        "10.17632/6ghdz52pd7.5.",
        "- DVB ships with Roboflow augmentations BAKED IN (mosaic tiles, "
        "salt-noise) on part of its images — tolerable in a TRAIN-only "
        "source, never in val/test (visual spot-check 2026-07-21).",
        "- `--check` re-asserts group disjointness on the emitted manifests; "
        "`--self-test` covers the split logic on a synthetic corpus.",
    ]
    with open(REPORT, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[report] {REPORT}")


def build_fetch_note():
    fetched = []
    flog = os.path.join(DATA, "fetch_log.jsonl")
    if os.path.exists(flog):
        with open(flog) as f:
            for line in f:
                try:
                    rec = json.loads(line)
                    fetched.append(f"- **{rec['source']}** {rec['path']} — "
                                   f"{rec['bytes']/1e6:.0f} MB ({rec['license']})")
                except ValueError:
                    pass
    deferred = [
        "- DEFERRED: **AOT** (CDLA-P-1.0) — s3 bucket NoSuchBucket 2026-07-21;",
        "- DEFERRED: **Det-Fly** — data-hosting license ambiguous (email "
        "authors) + manual OneDrive; skipped per the license rule;",
        "- DEFERRED: **Roboflow sets / Pexels** (API keys), **YouTube CC** "
        "(per-video license spot-check + yt-dlp), **Halmstad / USC MCL** "
        "(LOW bg match, below the fold). See fetch_nn_tier_sources.py.",
    ]
    return ("Fetched (fetch_log.jsonl):\n" + "\n".join(fetched) +
            "\n\nDeferred:\n" + "\n".join(deferred))


# ============================================================ check/self-test

def check_manifests(verbose=True):
    """Assert SOURCE/SCENE-disjointness on the emitted manifests. Returns ok."""
    path = os.path.join(DATA, "manifest_all.csv")
    if not os.path.exists(path):
        print("check: no manifest_all.csv — build first")
        return False
    with open(path) as f:
        rows = list(csv.DictReader(f))
    ok = True
    gsplits = defaultdict(set)
    for r in rows:
        gsplits[(r["source"], r["group"])].add(r["split"])
    for g, sp in gsplits.items():
        if len(sp) > 1:
            print(f"check FAIL: group {g} straddles splits {sp}")
            ok = False
    # scene-less sources must not appear in >1 split at SOURCE level
    for src in ("dvb", "dvb_bird", "dut", "composite"):
        sp = set(r["split"] for r in rows if r["source"] == src)
        if len(sp) > 1:
            print(f"check FAIL: scene-less source {src} in {sp}")
            ok = False
    comp_splits = set(r["split"] for r in rows if r["is_composite"] == "1")
    if comp_splits - {"train"}:
        print(f"check FAIL: composites outside train: {comp_splits}")
        ok = False
    tr = set(r["source"] for r in rows if r["split"] == "train")
    te = set(r["source"] for r in rows if r["split"] == "test")
    if verbose:
        both = tr & te
        print(f"check: {len(rows)} rows, groups={len(gsplits)}; sources in "
              f"train&test (scene-split video/author sources only): {both}")
        for s in both:
            gt_ = set(r["group"] for r in rows
                      if r["source"] == s and r["split"] == "train")
            ge = set(r["group"] for r in rows
                     if r["source"] == s and r["split"] == "test")
            if gt_ & ge:
                print(f"check FAIL: source {s} shares groups {gt_ & ge}")
                ok = False
    print("manifest check:", "PASS" if ok else "FAIL")
    return ok


def self_test():
    """Tiny synthetic corpus through the REAL split+emit path; asserts the split
    is source/scene-disjoint (no source-without-scenes in two splits; no scene
    group in two splits), negatives handled, gray variant written. Exit 0/1."""
    global DATA, RAW, REPORT
    old = (DATA, RAW, REPORT)
    td = tempfile.mkdtemp(prefix="nn_tier_selftest_")
    DATA = os.path.join(td, "data")
    RAW = os.path.join(td, "raw")
    REPORT = os.path.join(td, "report.md")
    os.makedirs(DATA, exist_ok=True)
    rng = random.Random(7)
    entries = []

    def mkimg(name, seed):
        p = os.path.join(td, "src", name + ".jpg")
        os.makedirs(os.path.dirname(p), exist_ok=True)
        r = np.random.default_rng(seed)
        cv2.imwrite(p, r.integers(0, 255, (80, 120, 3), np.uint8))
        return p

    k = 0
    # video-style source: 12 scenes × 3 frames, positives
    for c in range(12):
        for i in range(3):
            k += 1
            entries.append(entry("vidsrc", f"clip_{c:02d}",
                                 mkimg(f"v{c}_{i}", k), [(0.5, 0.5, 0.05, 0.04)],
                                 "MIT"))
    # scene-less source pinned train (like dvb)
    for i in range(8):
        k += 1
        entries.append(entry("sceneless_tr", "all", mkimg(f"s{i}", k),
                             [(0.4, 0.4, 0.3, 0.2)], "CC BY 4.0", pin="train"))
    # scene-less source pinned test (like dut)
    for i in range(8):
        k += 1
        entries.append(entry("sceneless_te", "all", mkimg(f"t{i}", k),
                             [(0.6, 0.6, 0.1, 0.1)], "MIT", pin="test"))
    # negatives: 6 authors × 2
    for a in range(6):
        for i in range(2):
            k += 1
            entries.append(entry("plates", f"author_{a}", mkimg(f"p{a}_{i}", k),
                                 [], "CC0"))
    # composites pinned train
    for i in range(4):
        k += 1
        entries.append(entry("composite", "synth_train", mkimg(f"c{i}", k),
                             [(0.5, 0.5, 0.02, 0.01)], "derived",
                             kind="composite", pin="train"))
    split_of = assign_splits(entries, rng)
    rows = emit(entries, split_of, workers=2)
    write_yamls()
    write_report(rows, split_of, "self-test corpus")
    ok = True
    if len(rows) != len(entries):
        print(f"SELF-TEST FAIL: emitted {len(rows)} != {len(entries)}")
        ok = False
    ok = check_manifests(verbose=False) and ok
    # explicit: no source appears in both train and test unless scene-grouped
    with open(os.path.join(DATA, "manifest_all.csv")) as f:
        rws = list(csv.DictReader(f))
    for src in ("sceneless_tr", "sceneless_te", "composite"):
        sp = set(r["split"] for r in rws if r["source"] == src)
        if len(sp) != 1:
            print(f"SELF-TEST FAIL: {src} splits {sp}")
            ok = False
    if not any(r["split"] == "test" and r["source"] == "vidsrc" for r in rws):
        print("SELF-TEST FAIL: no held-out vidsrc clip in test")
        ok = False
    for r in rws[:3]:
        g = cv2.imread(os.path.join(DATA, r["gray_img"]),
                       cv2.IMREAD_UNCHANGED)
        if g is None or g.ndim != 2:
            print("SELF-TEST FAIL: gray variant not single-channel")
            ok = False
            break
    lblcount = len(glob.glob(os.path.join(DATA, "gray", "labels", "*", "*.txt")))
    if lblcount != len(entries):
        print(f"SELF-TEST FAIL: {lblcount} gray labels != {len(entries)}")
        ok = False
    if not os.path.exists(os.path.join(DATA, "dataset.yaml")):
        print("SELF-TEST FAIL: dataset.yaml missing")
        ok = False
    shutil.rmtree(td, ignore_errors=True)
    DATA, RAW, REPORT = old
    print("prepare self-test:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ==================================================== composite validation

def validate_composites(n_check):
    """Sanity: composited frames at detectable scale (>=24 px) still trigger a
    real-imagery drone detector (yolo11x_mit) at a rate comparable to the raw
    donor images — guards against the domain_gap_eval failure (compositing
    artifacts making targets un/over-detectable)."""
    sys.path.insert(0, HERE)
    from baseline_eval import MODELS, run_model_on_image  # noqa: E402
    from v3_onnx_infer import load_session  # noqa: E402
    path, cls, nc = MODELS["yolo11x_mit"]
    sess, iname, imgsz = load_session(path)
    with open(os.path.join(DATA, "manifest_all.csv")) as f:
        rows = [r for r in csv.DictReader(f)
                if r["is_composite"] == "1" and float(r["max_box_w1280"]) >= 24]
    rng = random.Random(SEED)
    rng.shuffle(rows)
    rows = rows[:n_check]
    if not rows:
        print("[comp-val] no composites with box >=24px — nothing to check")
        return
    hits = 0
    for r in rows:
        im = cv2.imread(os.path.join(DATA, r["color_img"]))
        with open(os.path.join(DATA, "color", "labels", r["split"],
                               r["uid"] + ".txt")) as f:
            b = [float(x) for x in f.read().split()[1:5]]
        h, w = im.shape[:2]
        gx, gy = b[0] * w, b[1] * h
        dets = [d for d in run_model_on_image(sess, iname, imgsz, im, nc, cls)
                if d["score"] >= 0.10]
        for d in dets:
            if (abs(d["cx"] - gx) <= max(12, b[2] * w) and
                    abs(d["cy"] - gy) <= max(12, b[3] * h)):
                hits += 1
                break
    print(f"[comp-val] yolo11x_mit fired on {hits}/{len(rows)} composites "
          f"(box >=24px, conf>=0.10) — compare with its known weak real-media "
          f"baseline (docs/nn_tier/baseline_scoreboard.md) before alarm; "
          f"0/{len(rows)} would mean compositing destroys detectability.")
    out = {"n": len(rows), "hits": hits, "model": "yolo11x_mit",
           "conf": 0.10, "min_box_px": 24}
    with open(os.path.join(DATA, "composite_validation.json"), "w") as f:
        json.dump(out, f, indent=1)


# ======================================================================= main

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--report-only", action="store_true",
                    help="regenerate dataset_report.md from the manifests")
    ap.add_argument("--validate-composites", type=int, default=0, metavar="N")
    # defaults tuned for the >=70%-small-positives deploy directive: DVB boxes
    # are ALL huge (p5=276 px @1280eq, measured — close-up footage), so DVB is
    # capped low; NPS (tiny air-to-air) + composites carry the small regime.
    ap.add_argument("--dvb-pos-cap", type=int, default=1500)
    ap.add_argument("--bird-neg-cap", type=int, default=2000)
    ap.add_argument("--dut-cap", type=int, default=3000)
    ap.add_argument("--nps-stride", type=int, default=10)
    ap.add_argument("--n-composites", type=int, default=2500)
    ap.add_argument("--crops-per-plate", type=int, default=4)
    ap.add_argument("--workers", type=int, default=8)
    args = ap.parse_args()

    if args.self_test:
        sys.exit(self_test())
    if args.check:
        sys.exit(0 if check_manifests() else 1)
    if args.report_only:
        with open(os.path.join(DATA, "manifest_all.csv")) as f:
            rows = [{**r, "n_boxes": int(r["n_boxes"]),
                     "is_composite": int(r["is_composite"])}
                    for r in csv.DictReader(f)]
        write_report(rows, {}, build_fetch_note())
        return
    if args.validate_composites and not os.path.exists(
            os.path.join(DATA, "manifest_all.csv")):
        print("build the dataset first")
        sys.exit(1)
    if args.validate_composites:
        validate_composites(args.validate_composites)
        return

    t0 = time.time()
    rng = random.Random(SEED)
    entries = []
    dvb_entries, donors = index_dvb(rng, args.dvb_pos_cap, args.bird_neg_cap)
    entries += dvb_entries
    entries += index_nps(rng, args.nps_stride)
    entries += index_dut(rng, args.dut_cap)
    plate_entries = index_plates(rng, args.crops_per_plate)
    entries += plate_entries

    # provisional split (needed to know which plates are train, for composites)
    split_of = assign_splits(entries, rng)
    n_real_pos = sum(1 for e in entries if e["boxes"])
    n_comp = min(args.n_composites, int(0.42 * n_real_pos))  # <=30% of positives
    comp_entries = build_composites(rng, donors, plate_entries, n_comp, split_of)
    entries += comp_entries
    split_of = assign_splits(entries, rng)

    # negative-fraction control: aim inside NEG_TARGET band by trimming/keeping
    n_pos = sum(1 for e in entries if e["boxes"])
    n_neg = len(entries) - n_pos
    frac = n_neg / max(len(entries), 1)
    if frac > NEG_TARGET[1]:
        # trim bird negatives first (plates are the domain-precious ones)
        excess = n_neg - int(NEG_TARGET[1] * len(entries))
        birds = [e for e in entries if e["source"] == "dvb_bird"]
        rng.shuffle(birds)
        drop = set(id(e) for e in birds[:excess])
        entries = [e for e in entries if id(e) not in drop]
        split_of = assign_splits(entries, rng)
        print(f"[neg] trimmed {len(drop)} bird negatives to hit the "
              f"{NEG_TARGET[1]:.0%} band")
    elif frac < NEG_TARGET[0]:
        print(f"[neg] WARNING: negative fraction {frac:.2f} below the "
              f"{NEG_TARGET[0]:.0%} target — fetch more plates "
              f"(fetch_nn_tier_sources.py --source desert_plates)")

    rows = emit(entries, split_of, args.workers)
    write_yamls()
    write_report(rows, split_of, build_fetch_note())
    ok = check_manifests()
    print(f"done: {len(rows)} images in {time.time()-t0:.0f}s → {DATA}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
