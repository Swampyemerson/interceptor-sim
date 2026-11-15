#!/usr/bin/env python3
"""Score existing detector weights on an own-target frame set made by ground_video_labels.py.

    python scripts/nn/eval_own_target.py --data data/own_target/video_X --out runs/.../eval.json \
        --model v8n-color-640.pt:640 --model n-mono.pt:640:gray

Recall = labelled frames where some box overlaps the label at IoU >= --iou. False fire = empty
frames with any box. Extra boxes on labelled frames are counted too. Zero labelled frames is exit 1.
"""
import argparse, csv, json, os, sys
import cv2


def iou(a, b):
    iw, ih = min(a[2], b[2]) - max(a[0], b[0]), min(a[3], b[3]) - max(a[1], b[1])
    if iw <= 0 or ih <= 0: return 0.0
    i = iw * ih
    return i / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--data", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--weights-dir", default="scripts/seeker/weights/nn_tier")
    ap.add_argument("--model", action="append", required=True, help="file:imgsz[:gray]")
    ap.add_argument("--conf", type=float, default=0.25); ap.add_argument("--iou", type=float, default=0.3)
    a = ap.parse_args()
    from ultralytics import YOLO
    rows = list(csv.DictReader(open(f"{a.data}/index.csv")))
    pos = [r for r in rows if r["kind"] == "positive"]; neg = [r for r in rows if r["kind"] == "negative"]
    if not pos:
        print("FAIL: no labelled frames", file=sys.stderr); return 1
    out = dict(data=a.data, conf=a.conf, iou=a.iou, positives=len(pos), negatives=len(neg), models=[])
    for spec in a.model:
        parts = spec.split(":"); name, size, gray = parts[0], int(parts[1]), len(parts) > 2 and parts[2] == "gray"
        model = YOLO(os.path.join(a.weights_dir, name))
        def boxes(r):
            im = cv2.imread(f"{a.data}/images/{r['name']}.jpg")
            if gray: im = cv2.cvtColor(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
            res = model.predict(im, imgsz=size, conf=a.conf, verbose=False)[0]
            return [tuple(float(v) for v in b) for b in res.boxes.xyxy.cpu().numpy()]
        hit = extra = 0; bins = {}
        for r in pos:
            gt = tuple(float(r[k]) for k in ("x0", "y0", "x1", "y1")); bs = boxes(r)
            ok = any(iou(b, gt) >= a.iou for b in bs)
            hit += ok; extra += sum(1 for b in bs if iou(b, gt) < a.iou)
            s = float(r["long_side_px"]); k = "32_to_64" if s < 64 else "64_to_128" if s < 128 else "128_and_up"
            bins.setdefault(k, [0, 0]); bins[k][0] += ok; bins[k][1] += 1
        ff = sum(1 for r in neg if boxes(r))
        m = dict(model=name, imgsz=size, gray=gray, recall=round(hit / len(pos), 3), hits=hit,
                 recall_by_long_side_px={k: f"{v[0]}/{v[1]}" for k, v in sorted(bins.items())},
                 extra_boxes_on_labelled_frames=extra, false_fire_frames=ff,
                 false_fire_rate=round(ff / len(neg), 3) if neg else None)
        print(json.dumps(m), flush=True); out["models"].append(m)
    json.dump(out, open(a.out, "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
