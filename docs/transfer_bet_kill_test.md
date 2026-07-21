# Transfer-bet kill-test — the $0 free-footage smoke read (P0.6)

**Date:** 2026-07-20 · **Runner:** `scripts/seeker/transfer_bet_test.py`
**Footage manifest:** `scripts/seeker/data/transfer_bet_footage/manifest.json` (25 source files, 191 MB, all permissively licensed)
**Raw numbers:** `logs/transfer_bet_perframe_20260721T032450Z.csv`, `logs/transfer_bet_summary_20260721T032450Z.csv`
**Contact sheet (eyeball this):** `docs/images/transfer_bet_contact_sheet.png`

---

## Verdict — UNRESOLVED, leaning "detector generalizes, but is NOT discriminative"

The transfer bet (build_plan P0, constraint `transfer-bet`) is: *a drone-detection
NN trained entirely in Gazebo — `drone_finetuned_quad_v2.onnx`, ADR-0058, deployed —
will still box a REAL drone against a REAL outdoor sky.* This is the cheapest possible
read on that bet before the tripod day.

- **NOT DEAD.** The deployed sim-native seeker put a box on a real DJI Phantom in
  **2 of the 3** clean in-flight photos it saw — a Gazebo-only net firing on a real
  drone at all is the core generalization, and it clears that bar. The real-photo
  candidate `yolo11x` did **3 of 3**. So real drones *are* detectable.
- **NOT ALIVE-AND-WELL.** At the deployed raw operating point (conf 0.25, self-mask
  off, no streak gate) the per-frame **separation** between drone-present and
  drone-free real footage is only **+0.07** for the deployed net (fires on 0.71 of
  drone frames vs **0.65 of drone-FREE frames**). That is essentially no
  discrimination. The contact sheet shows why: on the ground-observer clips the
  deployed net's boxes land on the **human operators, tree lines, field furrows and
  frame corners — not the drone**. The optimistic-looking 0.76 density on the video
  segment is inflated clutter, not drone hits.
- **NO candidate is a drop-in win.** `yolo11x` detects the clean drone best but
  false-fires just as hard on clutter (0.65 on drone-free) and is 228 MB (22× the
  deployed net). `flying_objects_yolov8m` barely calls a real drone "Drone" (1/3 —
  it prefers "Airplane" for a DJI); not a useful drone detector on this footage.
- **The bet rides on a gate this test cannot see.** The deployed seeker does not act
  on raw per-frame fires — it is NN-only on every frame (detect-then-track/CSRT was
  TRIED and DROPPED for the maneuvering quad, ADR-0076 add #2; the ADR-0058 tracker
  config this doc originally cited is historical), and what stands between a raw
  false fire and a false HANDOFF is the **5-consecutive-fresh-detections streak**
  (handoff stage, `docs/project_state.json`) plus the alpha-beta filter's frame-to-
  frame consistency during terminal. A single cloud/field false-fire never forms a
  streak. So the alarming raw false-fire density
  below **overstates** what the streak-gated system would do outdoors — but whether
  that gate holds on real clutter is exactly the unanswered question. **A single-frame
  smoke read cannot answer it. The tripod day can.**

> **This is a SMOKE READ, not a curve.** The footage is UNLABELED, so nothing here is
> "recall". mAP/frame-density has already been shown in this repo *not* to predict
> in-flight recall (`docs/candidate_nn_shortlist.md` §5, ADR-0061). The tripod day —
> the real drone, the real seeker rig, the streak gate running end-to-end — measures
> the thing this file only sniffs.

---

## 1. What was measured, and the three honesty limits

We fetched permissively-licensed public **outdoor** footage from Wikimedia Commons
(drone-as-subject clips/photos = positives; drone-free sky / bird / ground = negatives),
extracted frames, and ran **three** seekers over every frame at the deployed operating
point (conf 0.25 — `finetuned_seeker.py` / `v3_onnx_infer.DEPLOY_CONF`), reporting how
often each fires. We report **detection density** (fraction of frames with ≥1 fire),
never recall, because:

1. **The footage is unlabeled.** No per-frame ground-truth boxes exist, so
   precision/recall are uncomputable. On a drone-present segment a higher density
   *suggests* the bet is more alive; on a drone-free segment the same number **is** the
   false-fire rate.
2. **The positive segments are loose.** The ground-observer video/photo positives are
   frames where the drone is often tiny, distant, or briefly out of frame while the
   **operators dominate the shot** — so their density is a noisy lower bound, and (per
   the eyeball check) the fires that do happen are mostly *not* on the drone. Only the
   three `pos_photo_dji` in-flight photos are clean drone-in-frame positives.
3. **Frame-density ≠ in-flight recall** (see the verdict box). No number here upgrades
   to "the seeker works."

**Honesty boundary (CLAUDE.md `gt_*`):** the loop reads camera pixels only. The footage
carries no ground truth to leak; detection is pixels-in, boxes-out, exactly like the live
seeker. **Self-mask is deliberately OFF:** the deployed self-mask is a *sim-camera-geometry*
gate (30° bearing on the 1280×960 sim intrinsics, `v3_onnx_infer.apply_self_mask`) — it is
meaningless on arbitrary-resolution real footage where the drone can sit anywhere in frame,
so we measure the **raw** detector and say so.

## 2. Footage (provenance + licensing)

All 25 source files are permissively licensed (Public Domain / CC0 / CC BY / CC BY-SA /
Free Art License — NC/ND/non-free are filtered out by `license_ok()` in the runner). The
FPV "drone" clips that are aerial-POV footage shot *by* a drone (drone NOT in frame) were
deliberately excluded — see `docs/candidate_nn_shortlist.md` §3 disambiguation caveat.
Per-file title, license, uploader, URL and sha256 are in the manifest JSON.

| Segment | Label | Media | Frames | What it is |
|---|---|---|---|---|
| `pos_photo_dji` | drone | image | 4 | close-range DJI Phantom photos (incl. 1 battery hard-negative) |
| `pos_photo_sky` | drone | image | 2 | small-quad-in-sky, ground-observer aspect |
| `pos_video_blm` | drone | video | 29 | PD "BLM drone training" clips; per-frame drone visibility **varies** |
| `neg_bird` | none | image | 6 | the classic drone/bird confuser |
| `neg_seagull` | none | image | 4 | more birds-in-sky |
| `neg_sky` | none | image | 4 | empty sky / clouds / sunset water |
| `neg_ground` | none | image | 3 | meadow / field / horizon clutter |

Licenses seen: CC BY-SA 4.0 ×11, Public domain ×7, FAL ×2, CC BY 4.0 ×2, CC0 ×1,
CC BY-SA 3.0 ×1, CC BY 2.0 ×1.

## 3. Models

| Key | Weights | Trained on | Input | Size |
|---|---|---|---|---|
| `v2_deployed` | `drone_finetuned_quad_v2.onnx` | **Gazebo sim renders** (ADR-0058, the deployed seeker) | 640² | 10.6 MB |
| `flyingobj` | `flying_objects_yolov8m.onnx` | real photos, 5-class Drone/Airplane/Helicopter/Bird/Background (MIT) | 1280² | 104 MB |
| `yolo11x` | `drone_yolo11x_1280.onnx` | real photos, ground-observer-up (MIT) | 1280² | 228 MB |

The candidates are the shortlist's fetchable outdoor NNs (`docs/candidate_nn_shortlist.md`
rows 1–2). For the 5-class `flyingobj`, the drone metric filters to **class 0 (Drone)**;
its other classes double as a free multi-class hard-negative channel.

## 4. Results — no model separates drone from background at the raw operating point

**Aggregate drone-class fire density, POS (all drone segments) vs NEG (all drone-free):**

| Model | POS density | NEG density | **Separation** |
|---|---|---|---|
| `v2_deployed` (sim-native) | 0.71 (25/35) | 0.65 (11/17) | **+0.07** |
| `flyingobj` | 0.20 (7/35) | 0.12 (2/17) | +0.08 |
| `yolo11x` | 0.80 (28/35) | 0.65 (11/17) | +0.15 |

No model achieves meaningful per-frame separation. `yolo11x` is the best (still fires on
**⅔ of drone-free frames**). The deployed net's +0.07 is, at this n, nothing.

**Per-segment drone-class density (from the summary CSV):**

| Segment (label) | `v2_deployed` | `flyingobj` | `yolo11x` |
|---|---|---|---|
| pos_photo_dji (drone) | 0.50 | 0.25 | **0.75** |
| pos_photo_sky (drone) | 0.50 | 0.00 | 0.50 |
| pos_video_blm (drone) | 0.76 | 0.21 | **0.83** |
| neg_bird (none) | 0.50 | 0.17 | 0.50 |
| neg_seagull (none) | 0.50 | 0.25 | 0.25 |
| neg_sky (none) | 0.75 | 0.00 | **1.00** |
| neg_ground (none) | **1.00** | 0.00 | **1.00** |

The negative rows are the alarming part: `v2_deployed` fires on a "drone" in **every**
meadow/field frame (1.00) and 3-of-4 empty-sky frames (0.75); `yolo11x` fires on every
empty-sky and every ground frame. `flyingobj`'s Drone channel is quiet on negatives (it
routes clutter to Airplane/Bird/Background) — but it is equally quiet on the *real drones*,
so that is conservatism, not skill.

## 5. The eyeball check — this is the decisive evidence

**See `docs/images/transfer_bet_contact_sheet.png`** (20 frames, boxes drawn per model:
`v2_deployed`=green, `flyingobj`=orange, `yolo11x`=magenta; header tints green for
drone-present, red for drone-free). What a human sees on it:

- **Clean drone-in-flight photos work.** On the two clear DJI-Phantom-in-flight cells,
  `yolo11x` draws a tight box on the actual drone and `v2_deployed`/`flyingobj` sometimes
  do too. **This is the "not dead" evidence.**
- **The DJI battery cell correctly gets zero drone boxes** from all three — a real
  hard-negative handled right (`flyingobj` calls it "Airplane").
- **On the ground-observer frames the boxes are on the wrong things.** The green
  (`v2_deployed`) and magenta (`yolo11x`) boxes pile onto the operators' heads, the tree
  line, the strawberry-field furrows (28 boxes on one field!), the beach buildings, and
  the frame corners — **not the drone**. This is why the pos_video_blm 0.76 density is
  misleading: most of those "detections" are clutter false-fires that happen to land in a
  drone-present frame.
- **Birds are a coin-flip.** One seagull-on-clouds cell is clean (all three ignore it);
  another bird-flock cell draws 13 `flyingobj` Drone boxes on the distant birds.

The contact sheet **overturns the optimistic reading** of the positive-segment densities:
strip out the clutter and the only unambiguous true positives in the whole set are the
clean DJI in-flight photos.

**The cleanest positives — `pos_photo_dji` per frame** (`n_det_drone` @ conf 0.25):

| Frame | Content | `v2_deployed` | `flyingobj` (Drone) | `yolo11x` |
|---|---|---|---|---|
| `_00` | DJI **battery** (hard-neg) | 0 ✓reject | 0 (calls Airplane) | 0 ✓reject |
| `_01` | Phantom in flight | 2 (0.40) | 2 (0.28) | 10 (0.58) |
| `_02` | Phantom in flight | **0 miss** | 0 (calls Airplane) | 8 (0.72) |
| `_03` | Phantom in flight | 4 (0.53) | 0 (calls Airplane) | 12 (0.65) |

Deployed sim-native net on clean drones: **2/3**. Real-photo `yolo11x`: **3/3**.
`flyingobj` as a drone detector: **1/3** (it labels the DJI "Airplane").

## 6. What this test cannot see — and why it matters

The deployed seeker's real defense against exactly the false-fires above is **not** in
this test. In flight it runs detect-then-track with a streak/consistency gate
(`scripts/seeker/markerless_loop.py`, ADR-0058) that reached "zero phantoms" in sim by
demanding a detection persist and translate consistently over several frames. A one-frame
fire on a cloud, a furrow, or an operator's head does not form a streak and is dropped.
So the raw per-frame false-fire density here is a **worst-case upper bound** on the fielded
system, not its operating number.

But the streak gate was only ever validated against **sim** clutter. Real outdoor clutter
(wind-moving tree lines, birds tracking across sky, people) can produce *consistent,
moving* false tracks that a streak gate will happily pass. Whether it holds is the open
question this smoke read raises and cannot close.

## 7. So — is the bet alive, dead, or unresolved?

**UNRESOLVED, with a specific red flag.** Detection of real drones by a sim-only-trained
net is demonstrably possible (2/3 clean shots), so the bet is **not dead**. But raw
per-frame discrimination on real footage is **≈ 0** for the deployed net, and the true
risk this exposes is **false-fires on outdoor clutter (fields, tree lines, birds, people),
which is a bigger problem than detection itself.** No off-the-shelf candidate NN fixes it.

**What the tripod day must therefore measure (not just "does it see the drone"):**

1. **Streak-gated end-to-end**, not raw per-frame — run `markerless_loop.py`'s detect-then-track
   on real footage; the raw detector alone is unusable outdoors.
2. **False-fire rate on drone-FREE real clutter** — point the rig at an empty sky, a tree
   line, a field, and a bird, *record how often a track forms with no drone present.* This
   test says that is the dominant failure mode.
3. **The ground-observer / nose-on aspect specifically** — the clean wins here were
   side-on product photos; the deployment aspect (small quad against sky/terrain) is where
   the boxes went to clutter.
4. A cheap lever if detection (not discrimination) turns out weak: `yolo11x` detected the
   clean drone best (3/3) — but it is 228 MB and false-fires just as much, so it is only
   worth its cost if paired with a much stronger gate. `flyingobj` is not worth pursuing as
   a drone detector (1/3, mislabels DJI as Airplane).

## 8. Reproduce

```bash
# fetch footage (network; ~191 MB; writes provenance manifest):
.venv-seeker/bin/python scripts/seeker/transfer_bet_test.py --fetch
# run all three seekers -> CSVs + contact sheet:
.venv-seeker/bin/python scripts/seeker/transfer_bet_test.py --run
# self-test (NO network, synthetic frames, exits 0/1):
.venv-seeker/bin/python scripts/seeker/transfer_bet_test.py --self-test
```

Operating point conf 0.25 = `v3_onnx_infer.DEPLOY_CONF` (`finetuned_seeker.py` deployed
default). Deployed net = ADR-0058. mAP-doesn't-predict-recall precedent =
`docs/candidate_nn_shortlist.md` §5 / ADR-0061. Candidate provenance =
`docs/candidate_nn_shortlist.md` + `scripts/seeker/fetch_candidate_nn.py`.
