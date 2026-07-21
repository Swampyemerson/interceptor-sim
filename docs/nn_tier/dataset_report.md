# nn_tier dataset report

*generated 2026-07-21 05:20 UTC by `scripts/seeker/nn_tier/prepare_nn_tier_dataset.py` — deterministic seed 20260721; re-run after more sources land.*

Deployment track: **grayscale** (`gray/`, `dataset.yaml`) matching the OV9281 mono sensor (camera_paper_check item 4); `color/` kept for the color-vs-gray A/B. Single class `drone`; birds/desert are EMPTY-label negatives.

## Split policy (anti-mirage)

Split is by **SOURCE / VIDEO / SCENE, never random frame** (the v3+rebal NULLs, ADR-0061):
- `nps` (air-to-air clips): grouped **by clip** — val/test are whole held-out flights.
- `dut` (MIT, no scene metadata): **whole source pinned to TEST** — a never-seen-source gate.
- `dvb` (no scene metadata — filename prefixes are class codes only): **pinned TRAIN-only**, never evaluated on.
- `plates` (Commons high-desert): grouped by **photographer/author**.
- `composite`: **TRAIN-only** (firewall — validation is always real held-out media), built ONLY on train-split plates.

## Counts

| split | images | positives | negatives | neg frac | composites |
|---|---|---|---|---|---|
| train | 10357 | 8193 | 2164 | 0.21 | 2500 |
| val | 859 | 339 | 520 | 0.61 | 0 |
| test | 4175 | 3706 | 469 | 0.11 | 0 |
| **total** | 15391 | 12238 | 3153 | 0.20 |  |

## Per-source

| source | split(s) | images | positives | license |
|---|---|---|---|---|
| composite | train | 2500 | 2500 | derived: DVB CC BY 4.0 cutout on Commons plate |
| dut | test | 3000 | 2999 | MIT |
| dvb | train | 1500 | 1500 | CC BY 4.0 |
| dvb_bird | train | 1623 | 0 | CC BY 4.0 |
| nps | test,train,val | 5652 | 5239 | BSD-3-Clause |
| plates | test,train,val | 1116 | 0 | CC BY 2.0 |

## Positive scale histogram (max box width, px @1280-eq)

| bin | count |
|---|---|
| 0-12px | 1162 |
| 12-24px | 4749 |
| 24-40px | 2603 |
| 40-80px | 1477 |
| 80-+px | 2247 |

**small-target fraction: 8626/12238 = 0.70** of positives are ≤40 px @1280-eq (deploy directive ≥70%); TRAIN-only: 5698/8193 = 0.70.

## Composite validation (anti-domain_gap_eval check)

- `yolo11x_mit` fired on **4/40** composites (box ≥24 px, conf ≥0.1).
- CONTROL, same model/criterion on REAL NPS positives ≥24 px: **1/40** — the reference model is small-object-blind (baseline_scoreboard.md), so composites are at least as detectable as real small targets; compositing does not destroy detectability. Numbers: `composite_validation.json`.

## Fetched vs deferred

Fetched (fetch_log.jsonl):
- **dvb** ../../nn_tier/data/dvb_mendeley.zip — 1129 MB (CC BY 4.0)
- **desert_plates** raw/desert_plates — 82 MB (CC0/PD/CC BY per-file (plates_manifest.csv))
- **desert_plates** raw/desert_plates — 121 MB (CC0/PD/CC BY per-file (plates_manifest.csv))
- **nps** raw/nps/Videos.zip — 2036 MB (BSD-3-Clause)
- **nps** raw/nps/Video_Annotation.zip — 1 MB (BSD-3-Clause)
- **desert_plates** raw/desert_plates — 329 MB (CC0/PD/CC BY per-file (plates_manifest.csv))
- **dut** raw/dut/train.zip — 745 MB (MIT)
- **dut** raw/dut/val.zip — 372 MB (MIT)
- **dut** raw/dut/test.zip — 271 MB (MIT)

Deferred:
- DEFERRED: **AOT** (CDLA-P-1.0) — s3 bucket NoSuchBucket 2026-07-21;
- DEFERRED: **Det-Fly** — data-hosting license ambiguous (email authors) + manual OneDrive; skipped per the license rule;
- DEFERRED: **Roboflow sets / Pexels** (API keys), **YouTube CC** (per-video license spot-check + yt-dlp), **Halmstad / USC MCL** (LOW bg match, below the fold). See fetch_nn_tier_sources.py.

## Honesty notes

- This corpus is a DOMAIN-MATCHED HEAD START from public real media — NOT final deployment weights material: no imagery of OUR quad exists yet (tripod day pending); the real-capture pipeline (real_data_pipeline.md) supersedes it when footage lands.
- Numbers here trace to `manifest_all.csv` + `fetch_log.jsonl`.
- CC BY plate/DVB attribution obligations: see `raw/desert_plates/plates_manifest.csv` and the Mendeley DOI 10.17632/6ghdz52pd7.5.
- DVB ships with Roboflow augmentations BAKED IN (mosaic tiles, salt-noise) on part of its images — tolerable in a TRAIN-only source, never in val/test (visual spot-check 2026-07-21).
- `--check` re-asserts group disjointness on the emitted manifests; `--self-test` covers the split logic on a synthetic corpus.
