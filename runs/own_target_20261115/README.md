# First frames of our own target through our own camera (2026-11-15)

Source: the Pi's AR0234 camera sitting on the ground in the yard, filming the target drone's
seventh flight. Video `sessions/video_20261115T182243Z/` (864 s, 1920x1200, 60 fps, fixed
0.15 ms exposure; on the PC and the Pi, not in the repo: it shows the site). Flight window
306 to 803 s of video, from the flight log's GPS clock (`runs/target_flight7_20261115/`).

Tool: `scripts/nn/ground_video_labels.py` (self-test and `tests/test_ground_video_labels.py`).
Output: `data/own_target/video_20261115T182243Z/` (gitignored), 590 images, 293 MB.
`summary.json` and `index.csv` here are copies.

## What was collected

| | Count |
|---|---|
| Flight frames examined (every second frame) | 14,785 |
| With one accepted track, box clear of clutter | 2,702 |
| Dropped: box cut by the clutter mask | 1,472 |
| Dropped: no accepted track (out of view, against the trees, or missed) | 10,611 |
| **Positives written (5 per second of labelled time)** | **484** |
| **Negatives written (only from before take-off)** | **106** (18%) |
| Target long side 32 to 64 px / 64 px and up | 212 / 272 |
| Target long side under 32 px | 0 |

Tracks: 960 formed, 25 accepted; rejected 779 too short, 95 that went nowhere, 61 sitting on
clutter. Clutter mask 26% of the frame before growing, 38% after (trees, the string of
lights, streamers, swaying wires).

## Label quality, looked at

144 labelled crops viewed (48 from a two-minute trial, 96 from the full run, spread evenly
through the flight): 144 of 144 are the drone; none is a streamer, bulb or bird. About 6 of
the 96 have a box visibly loose on one side (a wire or the frame edge joined the blob). No
box was seen cutting the drone. Not checked: the other 388.

Independent detector (`drone_yolo11x.pt`, conf 0.10, IoU 0.3, on a 640 crop): fires on 392
of 484 (81%); 249/272 at 64 px and up, 143/212 at 32 to 64 px. It drew no labels.

## What went wrong first (kept because the fixes are the design)

1. Grey-only, "darker than the sky" detection boxed single arms and props: the sunlit
   airframe matches the sky in grey. Fix: largest colour-channel difference, either sign.
2. Streamers were labelled as the drone. Fix: a clutter mask from the target-free lead-in.
3. That mask covered 55 to 73% of the frame. Two causes, both measured: faint contour bands
   in the 8-bit sky drift about and trip a low threshold (fix: same threshold as detection,
   30 levels), and the builder walking past the lens once counted as clutter (fix: clutter
   must recur in 3 of 10 stretches of the lead-in).

## Limits

- One session = one split group. All of it trains or all of it tests.
- Close range only: nothing under 32 px, and the seeker's hard case is under 30 px.
- Biased easy: only frames where the drone stood clear against sky could be labelled. Any
  recall measured on these frames is an upper bound for this range.
- A static ground camera, not the interceptor in flight: no own-motion blur, no props in view.

## Pre-registered before running: how do the EXISTING detectors do on these frames?

None of the existing weights has seen this target or this camera, so the 590 frames are a
held-out own-target set for them. Models: `v8n-color-640.pt`, `v8n-color-1280.pt`,
`n-color.pt`, `n-mono.pt` (grey input), full frame, conf 0.25, hit = IoU 0.3 or better.

- **Prediction:** recall between 0.5 and 0.75 for the v8n colour model at 640 input. Reason:
  at 640 the target is 11 to 45 px, and the screen bench gave 67 to 82% at 26 to 49 px and 35
  to 40% at 11 to 20 px. The 1280 model should be higher.
- **If recall is 0.6 or more:** the public-data model already transfers to our target at
  close range; own footage is then worth most at longer range, not here.
- **If recall is under 0.3:** own-target footage is load-bearing and a fine-tune comes first.
- **Between:** not decisive either way.
- **False fires** on the 106 empty frames (string lights, lantern, streamers, trees) are
  reported per model; the July model's bar on public desert frames was 4.9%.
- Whatever the number, it is an upper bound (see Limits) and one session.
