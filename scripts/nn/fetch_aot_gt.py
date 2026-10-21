#!/usr/bin/env python3
"""Stage 1 of the AOT slice (docs/nn_seeker_program.md, training step 1):
pull the per-part ground-truth JSON from the public Amazon Airborne Object
Tracking bucket (CDLA-Permissive-1.0; unsigned S3), then write a manifest
of small-target frames for the later, builder-gated bulk image pull.

Layout (probed 2026-10-21): part{1,2,3}/ImageSets/groundtruth.json
(~455 MB each, ~1.4 GB all three -- under the 2 GB ask-first rule) and
part{N}/Images/<flight_id>/<img_name>.

Usage:
  .venv/bin/python scripts/nn/fetch_aot_gt.py --parts 1        # one part
  .venv/bin/python scripts/nn/fetch_aot_gt.py --parts 1 2 3    # all metadata

The BULK image pull (50-100 GB) is a separate, builder-gated step: this
script only sizes it. Output under data/aot/ (gitignored); the manifest
stats print for the log.
"""
import argparse
import json
import os
import sys

BUCKET = "airborne-obj-detection-challenge-training"
REGION = "us-east-1"
OUT_DIR = "data/aot"
MAX_BOX_PX = 40.0          # "small target": box side at or under this
MIN_BOX_PX = 3.0           # below this nothing can learn


def s3_unsigned():
    import boto3
    from botocore import UNSIGNED
    from botocore.config import Config
    return boto3.client("s3", region_name=REGION,
                        config=Config(signature_version=UNSIGNED))


def iter_entities(gt):
    """Yield entity dicts from either the nested samples layout or a flat one."""
    if "samples" in gt:
        for flight_id, fl in gt["samples"].items():
            for e in fl.get("entities", []):
                e.setdefault("flight_id", flight_id)
                yield e
    else:
        yield from gt.get("entities", [])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--parts", type=int, nargs="+", default=[1])
    args = ap.parse_args()
    os.makedirs(OUT_DIR, exist_ok=True)
    s3 = s3_unsigned()

    manifest, n_ent = [], 0
    for part in args.parts:
        key = f"part{part}/ImageSets/groundtruth.json"
        dst = os.path.join(OUT_DIR, f"groundtruth_part{part}.json")
        if not os.path.exists(dst):
            print(f"downloading {key} ...", flush=True)
            s3.download_file(BUCKET, key, dst)
        print(f"parsing {dst} ...", flush=True)
        with open(dst) as f:
            gt = json.load(f)
        for e in iter_entities(gt):
            n_ent += 1
            bb = e.get("bb")
            if not bb or len(bb) < 4:
                continue
            side = max(bb[2], bb[3])
            if MIN_BOX_PX <= side <= MAX_BOX_PX:
                manifest.append({
                    "part": part,
                    "flight": e.get("flight_id"),
                    "img": e.get("img_name"),
                    "bb": [round(x, 1) for x in bb[:4]],
                    "range_m": (e.get("blob") or {}).get("range_distance_m"),
                })
        del gt

    mpath = os.path.join(OUT_DIR, "small_target_manifest.json")
    with open(mpath, "w") as f:
        json.dump(manifest, f)
    uniq_imgs = len({(m["part"], m["flight"], m["img"]) for m in manifest})
    est_gb = uniq_imgs * 450_000 / 1e9     # ~450 KB/frame, 2448x2048 png
    print(f"entities seen: {n_ent}; small-target boxes: {len(manifest)}; "
          f"unique frames: {uniq_imgs}; bulk pull ~{est_gb:.1f} GB "
          f"-> BUILDER GO-AHEAD required (>2 GB rule). Manifest: {mpath}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
