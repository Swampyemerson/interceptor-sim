#!/usr/bin/env python3
"""Stage 3 of the AOT slice: pick frames BY CLASS from the ground truth and download them.

Why this replaces the 66,170-frame pull list (make_aot_pull_list.py): that list was
sampled by box size only, and AOT is 96% helicopters and airplanes. Of 2.9 M labelled boxes
only 3,662 are class "Drone" (counted 2026-11-13). For a one-class DRONE detector the useful
slice is every Drone frame as positives, plus a capped sample of Bird / Airplane /
Helicopter frames kept as NEGATIVES (no drone label), not 30 GB of mostly helicopters.

  .venv/bin/python scripts/nn/fetch_aot_frames.py --plan      # write data/aot/pull_list_v2.json, no download
  setsid nohup .venv/bin/python scripts/nn/fetch_aot_frames.py > logs/nn_tier/fetch_aot_frames.log 2>&1 &

Resumable (skips files already on disk). Unsigned public S3, CDLA-Permissive-1.0.
Prints AOT_FETCH_DONE with counts; exits non-zero if more than 2% of frames fail.
"""
import argparse
import collections
import json
import os
import random
import re
import sys
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fetch_aot_gt import BUCKET, OUT_DIR, iter_entities, s3_unsigned  # noqa: E402

PLAN = os.path.join(OUT_DIR, "pull_list_v2.json")
NEG_CAPS = {"Bird": 4000, "Airplane": 2000, "Helicopter": 2000}
MAX_NEG_PER_FLIGHT = 40


def build_plan(seed: int = 0) -> dict:
    rng = random.Random(seed)
    frames = {}                                   # (part, flight, img) -> list of {cls, bb}
    for part in (1, 2, 3):
        with open(os.path.join(OUT_DIR, f"groundtruth_part{part}.json")) as f:
            gt = json.load(f)
        for e in iter_entities(gt):
            bb = e.get("bb")
            if not bb or len(bb) < 4:
                continue
            cls = re.sub(r"\d+$", "", str(e.get("id", "?")))
            cls = "Bird" if cls == "BIrd" else cls
            frames.setdefault((part, e["flight_id"], e["img_name"]), []).append({"cls": cls, "bb": [float(v) for v in bb[:4]]})
        del gt
    drone = [k for k, v in frames.items() if any(o["cls"] == "Drone" for o in v)]
    plan = [{"part": k[0], "flight": k[1], "img": k[2], "role": "drone", "objects": frames[k]} for k in sorted(drone)]
    taken = set(drone)
    for cls, cap in NEG_CAPS.items():
        pool = [k for k, v in frames.items() if k not in taken and all(o["cls"] == cls for o in v)]
        rng.shuffle(pool)
        per_flight = collections.Counter(); n = 0
        for k in pool:
            if n >= cap:
                break
            if per_flight[k[1]] >= MAX_NEG_PER_FLIGHT:
                continue
            per_flight[k[1]] += 1; n += 1; taken.add(k)
            plan.append({"part": k[0], "flight": k[1], "img": k[2], "role": f"negative_{cls.lower()}", "objects": frames[k]})
    json.dump(plan, open(PLAN, "w"))
    c = collections.Counter(p["role"] for p in plan)
    print(f"plan: {len(plan)} frames {dict(c)}; drone boxes {sum(1 for p in plan for o in p['objects'] if o['cls'] == 'Drone')}", flush=True)
    return plan


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", action="store_true", help="write the plan only, download nothing")
    ap.add_argument("--threads", type=int, default=16)
    a = ap.parse_args()
    plan = json.load(open(PLAN)) if os.path.exists(PLAN) else build_plan()
    if a.plan:
        return 0
    s3 = s3_unsigned()
    done = collections.Counter()

    def fetch(p):
        dst = os.path.join(OUT_DIR, "images", f"part{p['part']}", p["flight"], p["img"])
        if os.path.exists(dst) and os.path.getsize(dst) > 0:
            return "have"
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        try:
            s3.download_file(BUCKET, f"part{p['part']}/Images/{p['flight']}/{p['img']}", dst + ".part")
            os.replace(dst + ".part", dst)
            return "ok"
        except Exception as e:                      # counted, reported, never silently absorbed
            return f"fail:{type(e).__name__}"

    first = fetch(plan[0])
    if first.startswith("fail"):
        print(f"FIRST FETCH FAILED ({first}) -- key layout wrong or bucket unreachable; stopping", flush=True)
        return 2
    done[first] += 1
    with ThreadPoolExecutor(a.threads) as ex:
        for i, r in enumerate(ex.map(fetch, plan[1:]), 2):
            done[r.split(":")[0]] += 1
            if i % 500 == 0:
                print(f"{i}/{len(plan)} {dict(done)}", flush=True)
    size = sum(os.path.getsize(os.path.join(dp, f)) for dp, _, fs in os.walk(os.path.join(OUT_DIR, "images")) for f in fs)
    print(f"AOT_FETCH_DONE {dict(done)} of {len(plan)}; {size / 1e9:.2f} GB on disk", flush=True)
    return 1 if done["fail"] > 0.02 * len(plan) else 0


if __name__ == "__main__":
    sys.exit(main())
