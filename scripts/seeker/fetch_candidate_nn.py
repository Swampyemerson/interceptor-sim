#!/usr/bin/env python3
"""Fetch + ONNX-convert the candidate transfer-bet NN (docs/candidate_nn_shortlist.md).

WHAT / WHY
----------
The shortlist's top DIRECTLY-FETCHABLE candidate is `Javvanny/yolov8m_flying_objects_detection`
(Hugging Face, MIT-licensed weights, ~52 MB, YOLOv8m, 5 classes: Drone/Airplane/
Helicopter/Bird/Background). It is the only shortlisted candidate that is BOTH
small (<2 GB) AND has a ready checkpoint an HTTP GET can reach directly (Det-Fly
and TransVisDrone need either training or a multi-frame pipeline change -- see
the shortlist doc). It also happens to ship a labeled Bird class, which none of
this project's own sim-rendered training data has ever had (ADR note in
docs/nn_transfer_plan.md: "most drone-detection datasets have no labeled bird
class") -- worth a look purely for that.

NOT claimed to transfer. Like `drone_yolo11x` before it (drone_detector_eval.py,
did not transfer -- real-photo, mostly ground-observer-up domain vs this
project's onboard nose-on sim renders), this is a SCOPING fetch: get the weights
+ an ONNX export in-repo so a future eval pass (mirroring drone_detector_eval.py)
can measure acquisition range on the existing onboard frames, not a claim the
transfer will work.

USAGE (sim-free; does not boot PX4/Gazebo)
-------------------------------------------
  # download only (needs network, ~52 MB):
  .venv-seeker/bin/python scripts/seeker/fetch_candidate_nn.py

  # download + export to ONNX (needs ultralytics+torch -> .venv-seeker-train):
  .venv-seeker-train/bin/python scripts/seeker/fetch_candidate_nn.py --onnx

  # just print what would happen, no network:
  .venv-seeker/bin/python scripts/seeker/fetch_candidate_nn.py --dry-run

LICENSE NOTE (mirrors weights/LICENSES.md's existing drone_yolo11x entry)
---------------------------------------------------------------------------
The uploader (Javvanny) states MIT for these WEIGHTS. The weights were produced
by fine-tuning with Ultralytics' YOLOv8 codebase, which is itself AGPL-3.0 --
the same "MIT weights, AGPL export tooling" split already recorded for
drone_yolo11x. If publishing, the MIT claim governs the weight *artifact*; note
the AGPL tool provenance alongside it, same as the existing entry. NOT
independently re-verified beyond the HF model card's `license: mit` metadata
(no separate LICENSE file in the HF repo as of this fetch -- flagged, not
resolved, in the shortlist doc).

HONESTY: this script only downloads/converts a public pretrained checkpoint.
No sim, no gt_*, no training. Running the resulting model against seeker frames
(a follow-on step, not done by this script) would read camera pixels only, same
as every other seeker in this repo.
"""
from __future__ import annotations

import argparse
import hashlib
import os
import sys
import urllib.request

HF_REPO = "Javvanny/yolov8m_flying_objects_detection"
HF_FILE = "yolov8m/weights/best.pt"
HF_URL = f"https://huggingface.co/{HF_REPO}/resolve/main/{HF_FILE}"
EXPECTED_SIZE_BYTES = 52_033_814  # from a HEAD request on 2026-07-17; sanity check only
MAX_ALLOWED_BYTES = 2 * 1024 * 1024 * 1024  # 2 GB ask-first ceiling (CLAUDE.md)

WEIGHTS_DIR = os.path.join(os.path.dirname(__file__), "weights")
OUT_PT = os.path.join(WEIGHTS_DIR, "flying_objects_yolov8m.pt")
OUT_ONNX = os.path.join(WEIGHTS_DIR, "flying_objects_yolov8m.onnx")
OUT_CLASSES = os.path.join(WEIGHTS_DIR, "flying_objects_yolov8m.classes.txt")

# Class order as documented on the HF model card (Drone/Airplane/Helicopter/
# Bird/Background) -- written out here so a downstream eval script does not
# need network access just to know the label map.
CLASS_NAMES = ["Drone", "Airplane", "Helicopter", "Bird", "Background"]


def _human(nbytes: int) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if nbytes < 1024:
            return f"{nbytes:.1f} {unit}"
        nbytes /= 1024
    return f"{nbytes:.1f} TB"


def fetch(dry_run: bool = False) -> str:
    os.makedirs(WEIGHTS_DIR, exist_ok=True)

    if os.path.exists(OUT_PT):
        sz = os.path.getsize(OUT_PT)
        print(f"[fetch_candidate_nn] already present: {OUT_PT} ({_human(sz)}) -- skipping download")
        return OUT_PT

    print(f"[fetch_candidate_nn] HEAD {HF_URL}")
    req = urllib.request.Request(HF_URL, method="HEAD")
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            # HF redirects to a CDN with the real content-length; urlopen follows redirects.
            size = int(resp.headers.get("Content-Length", "0"))
    except Exception as e:  # noqa: BLE001 - report and bail cleanly, no partial state
        print(f"[fetch_candidate_nn] HEAD failed ({e}); aborting, no file written.")
        raise

    print(f"[fetch_candidate_nn] remote size: {_human(size)}")
    if size <= 0:
        raise RuntimeError("could not determine remote file size; refusing to download blind")
    if size > MAX_ALLOWED_BYTES:
        raise RuntimeError(
            f"remote file is {_human(size)}, over the 2 GB ask-first ceiling -- "
            "do not download without asking the builder first (CLAUDE.md)."
        )

    if dry_run:
        print(f"[fetch_candidate_nn] --dry-run: would download {_human(size)} to {OUT_PT}")
        return OUT_PT

    tmp = OUT_PT + ".part"
    print(f"[fetch_candidate_nn] GET {HF_URL} -> {tmp}")
    urllib.request.urlretrieve(HF_URL, tmp)  # nosec - trusted HF resolve URL, HEAD-checked above
    got = os.path.getsize(tmp)
    if got != size:
        os.remove(tmp)
        raise RuntimeError(f"size mismatch after download: expected {size}, got {got}")
    os.rename(tmp, OUT_PT)
    sha = hashlib.sha256(open(OUT_PT, "rb").read()).hexdigest()[:16]
    print(f"[fetch_candidate_nn] wrote {OUT_PT} ({_human(got)}, sha256[:16]={sha})")

    with open(OUT_CLASSES, "w") as f:
        f.write("\n".join(CLASS_NAMES) + "\n")
    print(f"[fetch_candidate_nn] wrote {OUT_CLASSES}")
    return OUT_PT


def export_onnx(pt_path: str, imgsz: int = 1280, dry_run: bool = False) -> str:
    """Export the .pt to ONNX at imgsz (default 1280, matching drone_yolo11x_1280's
    precedent -- the target is a few-px body at range; the default 640 letterbox
    would destroy it, per drone_detector_eval.py's docstring)."""
    if dry_run:
        print(f"[fetch_candidate_nn] --dry-run: would export {pt_path} -> ONNX @ imgsz={imgsz}")
        return OUT_ONNX
    try:
        from ultralytics import YOLO
    except ImportError:
        print(
            "[fetch_candidate_nn] ultralytics not importable in this venv -- "
            "re-run with .venv-seeker-train/bin/python (has ultralytics+torch). "
            f"The .pt is already at {pt_path}; export is a separate, optional step."
        )
        return ""
    model = YOLO(pt_path)
    model.export(format="onnx", imgsz=imgsz, opset=12, simplify=True)
    # ultralytics writes <stem>.onnx next to the .pt; move it to our canonical name
    produced = os.path.splitext(pt_path)[0] + ".onnx"
    if os.path.exists(produced) and produced != OUT_ONNX:
        os.replace(produced, OUT_ONNX)
    print(f"[fetch_candidate_nn] wrote {OUT_ONNX}")
    return OUT_ONNX


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--onnx", action="store_true", help="also export to ONNX (needs ultralytics+torch)")
    ap.add_argument("--imgsz", type=int, default=1280, help="ONNX export square input size (default 1280)")
    ap.add_argument("--dry-run", action="store_true", help="print what would happen; no network, no files written")
    args = ap.parse_args()

    pt_path = fetch(dry_run=args.dry_run)
    if args.onnx:
        export_onnx(pt_path, imgsz=args.imgsz, dry_run=args.dry_run)

    print(
        "\nNext step (not run by this script): eval acquisition range on "
        "demo_out/onboard_frames/ the same way drone_detector_eval.py measured "
        "drone_yolo11x -- point that script (or a copy of it) at "
        f"{OUT_ONNX if args.onnx else pt_path} and compare. See "
        "docs/candidate_nn_shortlist.md for the integration path and honesty caveats."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
