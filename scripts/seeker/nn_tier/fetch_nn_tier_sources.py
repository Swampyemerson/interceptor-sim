#!/usr/bin/env python3
"""fetch_nn_tier_sources.py — resumable fetcher for the nn_tier real-media corpus.

Pulls the permissively-licensed REAL drone/bird/desert sources ranked by the recon
pass (docs/nn_tier/domain_and_sources.md) into scripts/seeker/data/nn_tier/raw/,
logging the byte size of every completed download to fetch_log.jsonl.

Sources implemented (license — status):
  dvb            Mendeley YOLOv7-Segmented Drone-vs-Bird, CC BY 4.0 — recon already
                 fetched it to scripts/seeker/nn_tier/data/; verified + logged here.
  nps            Purdue/NPS "UAV_Dataset" (NPS-Drones), BSD-3-Clause — direct HTTP,
                 resumable (Videos.zip 2.0 GB + Video_Annotation.zip 0.76 MB).
  dut            DUT Anti-UAV detection set, MIT — Google Drive direct download
                 (train/val/test zips, 1.39 GB total).
  desert_plates  Wikimedia Commons high-desert imagery (CC0 / Public domain / CC BY
                 only — license filtered per file via extmetadata), the REAL desert
                 background-plate + negatives pool.
  aot            Amazon Airborne Object Tracking, CDLA-Permissive-1.0 — DEFERRED:
                 s3://airborne-obj-tracking returns NoSuchBucket as of 2026-07-21
                 (verified via HTTPS listing). The stub re-checks and explains.

Deliberately NOT fetched (deferred with reasons, see --status):
  det_fly    MIT repo but data hosting license ambiguous (candidate_nn_shortlist §4:
             email authors) + OneDrive links are manual-only. SKIPPED per the
             "skip anything license-ambiguous" rule.
  roboflow   CC BY sets need a Roboflow API key (none configured on this box).
  halmstad   CC0 visible-video subset — GitHub-linked hosting needs a per-file
             manual pass; LOW bg match, deferred below the fold.
  usc_mcl    MIT-style, HF hub — ground-up urban, LOW bg match, deferred.
  youtube_cc per-video CC BY needs manual license spot-checks + yt-dlp (absent).
  pexels     platform API key needed; Wikimedia Commons covers the desert plates.

Usage:
  python3 fetch_nn_tier_sources.py --source all            # fetch everything viable
  python3 fetch_nn_tier_sources.py --source nps --daemon   # setsid-detached fetch
  python3 fetch_nn_tier_sources.py --source desert_plates --max-plates 320
  python3 fetch_nn_tier_sources.py --status                # progress + byte totals
  python3 fetch_nn_tier_sources.py --self-test             # offline logic check

Reaper-safety: long fetches loop `curl -C -` (resume-from-byte) so a killed fetch
continues where it stopped on the next invocation; --daemon setsid-detaches.
"""

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import time
import urllib.parse
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
SEEKER = os.path.abspath(os.path.join(HERE, ".."))
DATA = os.path.join(SEEKER, "data", "nn_tier")
RAW = os.path.join(DATA, "raw")
FETCH_LOG = os.path.join(DATA, "fetch_log.jsonl")
RECON_DVB = os.path.join(HERE, "data")  # recon agent's cache dir

UA = "interceptor-sim-dataset-fetch/1.0 (research; contact: emersonium2@gmail.com)"

# --------------------------------------------------------------------------
# static download table: (source, relpath, url, expected_bytes, license)
DOWNLOADS = [
    ("nps", "nps/Videos.zip",
     "https://engineering.purdue.edu/~bouman/UAV_Dataset/Videos.zip",
     2035637654, "BSD-3-Clause"),
    ("nps", "nps/Video_Annotation.zip",
     "https://engineering.purdue.edu/~bouman/UAV_Dataset/Video_Annotation.zip",
     759315, "BSD-3-Clause"),
    ("dut", "dut/train.zip",
     "https://drive.usercontent.google.com/download?id=1RVsSGPUKTdmoyoPTBTWwroyulLek1eTj&export=download&confirm=t",
     744616230, "MIT"),
    ("dut", "dut/val.zip",
     "https://drive.usercontent.google.com/download?id=1333uEQfGuqTKslRkkeLSCxylh6AQ0X6n&export=download&confirm=t",
     372283691, "MIT"),
    ("dut", "dut/test.zip",
     "https://drive.usercontent.google.com/download?id=1L1zeW1EMDLlXHClSDcCjl3rs_A6sVai0&export=download&confirm=t",
     271153425, "MIT"),
]

# Wikimedia Commons search terms tuned to the Central-Oregon high-desert domain
# classes (docs/nn_tier/domain_and_sources.md §1).
PLATE_TERMS = [
    "Oregon Badlands Wilderness",
    "high desert Oregon sagebrush",
    "sagebrush steppe",
    "western juniper woodland",
    "basalt rimrock",
    "Fort Rock Oregon",
    "Smith Rock Oregon",
    "central Oregon desert",
    "great basin sagebrush",
    "lava beds Oregon",
    "juniper sagebrush desert",
    "desert dry grassland shrub steppe",
    "Steens Mountain",
    "Alvord Desert",
    "Hart Mountain Oregon",
    "Crooked River canyon Oregon",
    "Deschutes canyon",
    "Christmas Valley Oregon",
    "Owyhee canyonlands",
    "Prineville district BLM",
    "Newberry volcanic lava",
    "rabbitbrush steppe",
]
ALLOWED_PLATE_LICENSES = re.compile(
    r"^(cc0|public domain|pd|no restrictions|cc[- ]by(?![- ]?(sa|nc|nd))[- ]?\d?\.?\d?)",
    re.IGNORECASE)
# off-domain content blocklist (title + Commons categories): archival engravings,
# maps, artwork, postcards etc. that slip through the search terms
PLATE_BLOCK = re.compile(
    r"engrav|lithograph|illustrat|newspaper|drawing|sketch|painting|artwork|"
    r"woodcut|etching|map[_ ]|_map|postcard|stereo[_ ]?card|poster|book[_ ]|"
    r"leslie|harper|magazine|cover|diagram|logo|chart", re.IGNORECASE)


def log_fetch(source, path, nbytes, url, license_str, note=""):
    os.makedirs(DATA, exist_ok=True)
    rec = {"ts": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
           "source": source, "path": os.path.relpath(path, DATA),
           "bytes": int(nbytes), "url": url, "license": license_str}
    if note:
        rec["note"] = note
    # dedupe: skip if an identical (path,bytes) record already exists
    if os.path.exists(FETCH_LOG):
        with open(FETCH_LOG) as f:
            for line in f:
                try:
                    old = json.loads(line)
                except ValueError:
                    continue
                if old.get("path") == rec["path"] and old.get("bytes") == rec["bytes"]:
                    return
    with open(FETCH_LOG, "a") as f:
        f.write(json.dumps(rec) + "\n")


def curl_resumable(url, out_path, expected_bytes, max_rounds=100):
    """Loop curl -C - until out_path reaches expected size. Returns True on done."""
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    for _ in range(max_rounds):
        sz = os.path.getsize(out_path) if os.path.exists(out_path) else 0
        if sz >= expected_bytes:
            return True
        subprocess.run(
            ["curl", "-s", "-L", "-C", "-", "--retry", "5", "--retry-delay", "10",
             "--max-time", "3300", "-A", UA, "-o", out_path, url],
            check=False)
        time.sleep(2)
    return os.path.exists(out_path) and os.path.getsize(out_path) >= expected_bytes


def fetch_static(source):
    ok = True
    for src, rel, url, exp, lic in DOWNLOADS:
        if src != source:
            continue
        out = os.path.join(RAW, rel)
        if os.path.exists(out) and os.path.getsize(out) >= exp:
            print(f"[{src}] {rel} already complete ({exp/1e6:.1f} MB)")
        else:
            print(f"[{src}] fetching {rel} ({exp/1e6:.1f} MB) ...")
            if not curl_resumable(url, out, exp):
                print(f"[{src}] INCOMPLETE {rel} "
                      f"({os.path.getsize(out) if os.path.exists(out) else 0} / {exp})")
                ok = False
                continue
        log_fetch(src, out, os.path.getsize(out), url, lic)
    return ok


def fetch_dvb():
    """Verify the recon-fetched Mendeley Drone-vs-Bird set and log it."""
    z = os.path.join(RECON_DVB, "dvb_mendeley.zip")
    d = os.path.join(RECON_DVB, "dvb_extract", "Dataset")
    if os.path.isdir(d) and os.path.exists(z):
        log_fetch("dvb", z, os.path.getsize(z),
                  "https://data.mendeley.com/datasets/6ghdz52pd7/5",
                  "CC BY 4.0", note="fetched by the recon pass; verified here")
        print(f"[dvb] present at {d} ({os.path.getsize(z)/1e9:.2f} GB zip) — OK")
        return True
    print("[dvb] NOT found in the recon cache; fetch it via the Mendeley API:")
    print("      https://data.mendeley.com/api/datasets/6ghdz52pd7/versions/5 "
          "(follow files[].download_url), unzip to "
          f"{os.path.join(RECON_DVB, 'dvb_extract')}")
    return False


def commons_query(params):
    url = ("https://commons.wikimedia.org/w/api.php?" +
           urllib.parse.urlencode(params))
    req = urllib.request.Request(url, headers={"User-Agent": UA})
    with urllib.request.urlopen(req, timeout=45) as r:
        return json.loads(r.read().decode())


def fetch_desert_plates(max_plates=320, per_term=40, width=1600):
    """License-filtered (CC0/PD/CC BY only) high-desert stills from Wikimedia
    Commons — the REAL background-plate + negatives pool. Manifest records the
    license + author of every file (CC BY attribution obligation)."""
    out_dir = os.path.join(RAW, "desert_plates")
    os.makedirs(out_dir, exist_ok=True)
    man_path = os.path.join(out_dir, "plates_manifest.csv")
    have = {}
    if os.path.exists(man_path):
        with open(man_path) as f:
            for line in f.read().splitlines()[1:]:
                parts = line.split("\t")
                if parts and parts[0]:
                    have[parts[0]] = True
    rows = []
    n_new, n_total = 0, len(have)
    for term in PLATE_TERMS:
        if n_total >= max_plates:
            break
        try:
            res = commons_query({
                "action": "query", "format": "json",
                "generator": "search", "gsrsearch": term,
                "gsrnamespace": 6, "gsrlimit": per_term,
                "prop": "imageinfo",
                "iiprop": "url|size|extmetadata|mime",
                "iiurlwidth": width})
        except Exception as e:  # network hiccup: skip term, keep going
            print(f"[plates] term '{term}' failed: {e}")
            continue
        pages = (res.get("query") or {}).get("pages") or {}
        for pid, page in sorted(pages.items()):
            if n_total >= max_plates:
                break
            info = (page.get("imageinfo") or [{}])[0]
            meta = info.get("extmetadata") or {}
            lic = (meta.get("LicenseShortName") or {}).get("value", "")
            artist = re.sub(r"<[^>]+>", "",
                            (meta.get("Artist") or {}).get("value", ""))[:120]
            mime = info.get("mime", "")
            if not mime.startswith("image/") or mime == "image/gif":
                continue
            if not ALLOWED_PLATE_LICENSES.match(lic.strip()):
                continue
            if info.get("width", 0) < 800:
                continue
            cats = (meta.get("Categories") or {}).get("value", "")
            desc = (meta.get("ImageDescription") or {}).get("value", "")[:300]
            if PLATE_BLOCK.search(page.get("title", "") + " " + cats + " " + desc):
                continue
            url = info.get("thumburl") or info.get("url")
            if not url:
                continue
            slug = re.sub(r"[^A-Za-z0-9_.-]", "_",
                          page.get("title", "F").replace("File:", ""))[:80]
            fname = f"{pid}_{slug}"
            if not fname.lower().endswith((".jpg", ".jpeg", ".png")):
                fname += ".jpg"
            if fname in have:
                continue
            fpath = os.path.join(out_dir, fname)
            got = False
            for attempt in (1, 2):
                try:
                    req = urllib.request.Request(url, headers={"User-Agent": UA})
                    with urllib.request.urlopen(req, timeout=60) as r, \
                            open(fpath, "wb") as f:
                        f.write(r.read())
                    got = True
                    break
                except urllib.error.HTTPError as e:
                    if e.code == 429 and attempt == 1:
                        print("[plates] 429 rate-limited — backing off 45 s")
                        time.sleep(45)
                        continue
                    print(f"[plates] download failed {fname}: {e}")
                    break
                except Exception as e:
                    print(f"[plates] download failed {fname}: {e}")
                    break
            if not got:
                continue
            have[fname] = True
            rows.append((fname, term, lic.strip(), artist,
                         info.get("descriptionurl", "")))
            n_new += 1
            n_total += 1
            time.sleep(1.2)  # polite (Wikimedia 429s at faster rates)
    if rows:
        newfile = not os.path.exists(man_path)
        with open(man_path, "a") as f:
            if newfile:
                f.write("file\tsearch_term\tlicense\tartist\tsource_url\n")
            for r_ in rows:
                f.write("\t".join(str(x).replace("\t", " ") for x in r_) + "\n")
    total_bytes = sum(os.path.getsize(os.path.join(out_dir, f))
                      for f in os.listdir(out_dir)
                      if f.lower().endswith((".jpg", ".jpeg", ".png")))
    log_fetch("desert_plates", out_dir, total_bytes,
              "https://commons.wikimedia.org (API, license-filtered)",
              "CC0/PD/CC BY per-file (plates_manifest.csv)",
              note=f"{n_total} plates ({n_new} new this run)")
    print(f"[plates] {n_total} plates on disk ({total_bytes/1e6:.0f} MB), "
          f"{n_new} new; manifest: {man_path}")
    return n_total > 0


def fetch_aot():
    """AOT (CDLA-Permissive-1.0) — targeted-subset fetch, currently DEFERRED."""
    try:
        req = urllib.request.Request(
            "https://airborne-obj-tracking.s3.amazonaws.com/?list-type=2&max-keys=1",
            headers={"User-Agent": UA})
        with urllib.request.urlopen(req, timeout=30) as r:
            body = r.read().decode()
        alive = "NoSuchBucket" not in body
    except Exception as e:
        body = str(e)
        alive = False
    if not alive:
        print("[aot] DEFERRED: s3://airborne-obj-tracking is gone "
              "(NoSuchBucket via HTTPS, verified 2026-07-21). Re-check "
              "https://registry.opendata.aws/airborne-object-tracking/ for a "
              "new location before spending more effort here.")
        return False
    print("[aot] bucket is alive again — implement the targeted per-sequence "
          "subset fetch (ImageSets/groundtruth.json + a few Images/ prefixes) "
          "before bulk use; full corpus is multi-TB, DO NOT bulk-fetch.")
    return False


def status():
    print("=== nn_tier fetch status ===")
    for src, rel, url, exp, lic in DOWNLOADS:
        out = os.path.join(RAW, rel)
        sz = os.path.getsize(out) if os.path.exists(out) else 0
        state = "DONE" if sz >= exp else (f"{100.0*sz/exp:5.1f}%" if sz else "  —  ")
        print(f"  [{src:>4}] {rel:<28} {state}  {sz/1e6:8.1f} / {exp/1e6:.1f} MB  ({lic})")
    d = os.path.join(RECON_DVB, "dvb_extract", "Dataset")
    print(f"  [ dvb] recon cache {'PRESENT' if os.path.isdir(d) else 'MISSING'}: {d}")
    pd_ = os.path.join(RAW, "desert_plates")
    n = len([f for f in os.listdir(pd_)
             if f.lower().endswith((".jpg", ".jpeg", ".png"))]) if os.path.isdir(pd_) else 0
    print(f"  [plat] desert_plates: {n} images")
    print("  [ aot] DEFERRED (bucket NoSuchBucket 2026-07-21)")
    print("  [defer] det_fly (license-ambiguous data hosting), roboflow (API key),"
          " halmstad, usc_mcl, youtube_cc, pexels — see module docstring")
    if os.path.exists(FETCH_LOG):
        tot = 0
        with open(FETCH_LOG) as f:
            for line in f:
                try:
                    tot += json.loads(line).get("bytes", 0)
                except ValueError:
                    pass
        print(f"  fetch_log.jsonl total logged bytes: {tot/1e9:.2f} GB")


def self_test():
    """Offline logic check: log dedupe + plate license filter. Exit 0/1."""
    import tempfile
    global FETCH_LOG
    old = FETCH_LOG
    ok = True
    with tempfile.TemporaryDirectory() as td:
        FETCH_LOG = os.path.join(td, "log.jsonl")
        log_fetch("t", os.path.join(DATA, "x.zip"), 10, "u", "MIT")
        log_fetch("t", os.path.join(DATA, "x.zip"), 10, "u", "MIT")  # dupe
        log_fetch("t", os.path.join(DATA, "x.zip"), 20, "u", "MIT")  # grown
        with open(FETCH_LOG) as f:
            n = len(f.read().splitlines())
        if n != 2:
            print(f"SELF-TEST FAIL: fetch-log dedupe wrote {n} records, want 2")
            ok = False
    FETCH_LOG = old
    good = ["CC0", "Public domain", "CC BY 4.0", "CC BY 3.0", "cc-by-2.0",
            "No restrictions"]
    bad = ["CC BY-SA 4.0", "CC BY-NC 2.0", "CC BY-ND 4.0", "GFDL", "Fair use",
           "CC BY-SA-3.0"]
    for lic in good:
        if not ALLOWED_PLATE_LICENSES.match(lic):
            print(f"SELF-TEST FAIL: license '{lic}' should be allowed")
            ok = False
    for lic in bad:
        if ALLOWED_PLATE_LICENSES.match(lic):
            print(f"SELF-TEST FAIL: license '{lic}' should be REJECTED")
            ok = False
    print("fetch self-test:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--source", choices=["all", "dvb", "nps", "dut",
                                         "desert_plates", "aot"], default=None)
    ap.add_argument("--status", action="store_true")
    ap.add_argument("--daemon", action="store_true",
                    help="setsid-detach this fetch (reaper-safe)")
    ap.add_argument("--max-plates", type=int, default=320)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        sys.exit(self_test())
    if args.status or not args.source:
        status()
        return
    if args.daemon:
        cmd = [sys.executable, os.path.abspath(__file__), "--source", args.source,
               "--max-plates", str(args.max_plates)]
        logf = os.path.join(DATA, f"fetch_{args.source}_daemon.log")
        with open(logf, "a") as f:
            subprocess.Popen(cmd, stdout=f, stderr=f,
                             start_new_session=True)  # setsid-equivalent
        print(f"daemonized: {' '.join(cmd)} → {logf}")
        return

    os.makedirs(RAW, exist_ok=True)
    todo = (["dvb", "nps", "dut", "desert_plates", "aot"]
            if args.source == "all" else [args.source])
    results = {}
    for s in todo:
        if s == "dvb":
            results[s] = fetch_dvb()
        elif s in ("nps", "dut"):
            results[s] = fetch_static(s)
        elif s == "desert_plates":
            results[s] = fetch_desert_plates(args.max_plates)
        elif s == "aot":
            results[s] = fetch_aot()
    print("fetch results:", results)


if __name__ == "__main__":
    main()
