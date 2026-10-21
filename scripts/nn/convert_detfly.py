#!/usr/bin/env python3
"""Det-Fly (Pascal-VOC XML) -> YOLO single-class layout + grayscale copy.

Det-Fly download is manual (OneDrive links in github.com/Jake-WU/Det-Fly,
MIT). Once the two zips are extracted:

  .venv/bin/python scripts/nn/convert_detfly.py \
      --images data/detfly/JPEGImages --ann data/detfly/Annotations \
      --out data/detfly_yolo [--gray]

Writes out/images/ + out/labels/ (class 0 = drone) + a stats line with the
box-size histogram (the number that matters for our 5-40 px regime). Fails
loud on zero conversions. data/ is gitignored; this script is the artifact.
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET
from collections import Counter


def convert_one(xml_path, out_labels):
    root = ET.parse(xml_path).getroot()
    size = root.find("size")
    W, H = float(size.find("width").text), float(size.find("height").text)
    if W <= 0 or H <= 0:
        return None, "bad-size"
    lines, sides = [], []
    for obj in root.iter("object"):
        bb = obj.find("bndbox")
        x0, y0 = float(bb.find("xmin").text), float(bb.find("ymin").text)
        x1, y1 = float(bb.find("xmax").text), float(bb.find("ymax").text)
        w, h = x1 - x0, y1 - y0
        if w <= 1 or h <= 1:
            continue
        lines.append(f"0 {(x0 + w / 2) / W:.6f} {(y0 + h / 2) / H:.6f} "
                     f"{w / W:.6f} {h / H:.6f}")
        sides.append(max(w, h))
    if not lines:
        return None, "no-box"
    stem = os.path.splitext(os.path.basename(xml_path))[0]
    with open(os.path.join(out_labels, stem + ".txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    return sides, None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--images", required=True)
    ap.add_argument("--ann", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--gray", action="store_true",
                    help="write grayscale copies (else hardlink originals)")
    args = ap.parse_args()
    out_img = os.path.join(args.out, "images")
    out_lab = os.path.join(args.out, "labels")
    os.makedirs(out_img, exist_ok=True)
    os.makedirs(out_lab, exist_ok=True)

    rejects = Counter()
    hist = Counter()
    n_ok = 0
    for name in sorted(os.listdir(args.ann)):
        if not name.endswith(".xml"):
            continue
        sides, err = convert_one(os.path.join(args.ann, name), out_lab)
        if err:
            rejects[err] += 1
            continue
        stem = os.path.splitext(name)[0]
        src = None
        for ext in (".jpg", ".JPG", ".jpeg", ".png"):
            cand = os.path.join(args.images, stem + ext)
            if os.path.exists(cand):
                src = cand
                break
        if src is None:
            rejects["no-image"] += 1
            os.remove(os.path.join(out_lab, stem + ".txt"))
            continue
        dst = os.path.join(out_img, os.path.basename(src))
        if args.gray:
            import cv2
            cv2.imwrite(dst, cv2.imread(src, cv2.IMREAD_GRAYSCALE),
                        [cv2.IMWRITE_JPEG_QUALITY, 95])
        elif not os.path.exists(dst):
            os.link(src, dst)
        n_ok += 1
        for s in sides:
            for lo, hi in ((0, 10), (10, 20), (20, 40), (40, 80), (80, 1e9)):
                if lo <= s < hi:
                    hist[f"{lo}-{int(hi) if hi < 1e9 else '+'}px"] += 1
                    break

    print(f"converted {n_ok} frames; rejects {dict(rejects)}; "
          f"box-side histogram {dict(sorted(hist.items()))}")
    if n_ok == 0:
        print("ZERO conversions -- wrong paths or unexpected XML layout; failing loud")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
