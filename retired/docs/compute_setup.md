# Compute setup — the real interceptor's brains, end-to-end

*Companion to ADR-0015 (perception architecture) and ADR-0012 (hardware stack).
This file makes the compute concrete: who computes what, at what rate, over what
link, and — the load-bearing part — a **sourced, three-tier end-to-end latency
budget** you can later replace 1:1 with bench measurements. Covers docs/next.md items
**P-3** (compute split), **P-5** (onboard ML chip), **P-7** (latency budget).*

> **⛔ SUPERSEDED for the ONBOARD interceptor by the 2026-07-15 coded-dash pivot
> (ADR-0076 add #18, `docs/real_build_coded_dash.md`; live state: `docs/project_state.json`).**
> This doc describes the **parent-project ground-sensing half** — a ground Jetson
> track node + a cue radio ("~90-byte track message") + RTK ($400/pair) feeding a
> mid-course cue. The interceptor being built has **NO datalink and NO ground cue**:
> it flies a coded open-loop dash and the camera-only terminal acquires from a
> roughly-aimed dash; **RTK is CUT** (§0c binary-kill re-scope); the immediate air
> compute is **Pi 5 + AprilTag (CPU-real-time)**, with the Hailo HAT + markerless
> deferred. Read the latency-budget METHOD below as reusable; read the ground-rig
> ARCHITECTURE as parent-project context, not the current onboard build.

**Read this first if you're new to the terms** (one line each):
- **NPU** — "neural processing unit," a chip that runs neural-net math fast and
  cheap on power. The Hailo hat is an NPU.
- **TOPS** — "trillion operations per second," a rough size rating for an NPU. More
  TOPS = can run a bigger model or more streams.
- **TensorRT** — NVIDIA's tool that compiles a neural net to run as fast as possible
  on a Jetson. Always used on the ground node here.
- **Global shutter** — a camera that captures the whole frame at one instant (no
  skew when things move fast). Mandatory here (ADR-0012).
- **RTK / PPS** — RTK = a GPS trick that gets two receivers to agree to ~centimeters
  instead of meters; PPS = "pulse per second," a wire that makes both computers'
  clocks tick in lockstep with GPS time. Together they fix the "wrong place, wrong
  time" error that dominates cross-machine tracking (perception_design.md).
- **OOSM comp** — "out-of-sequence-measurement compensation." A cue that arrives
  late is stale; you slide it forward along its own velocity by its known age. The
  project already built this as `--cue-latency-comp`.
- **Latency vs. rate** — *rate* is how often a number arrives (Hz); *latency* is how
  old that number is when you act on it (ms). Both matter; they are not the same.

**The methodology (non-negotiable, from the builder):** every rate and latency below
gets three tiers — **BEST** (a datasheet or clean benchmark), **EXPECTED** (a
realistic integrated system), and **WORST-CREDIBLE** (thermal throttling, two
streams fighting for the chip, radio retries). **The sim is driven by EXPECTED, with
WORST as a stress case. A BEST number never sets a design margin.** Where sources
disagree, the pessimistic one is used and said so.

---

## 1. Pipeline map — who computes what, and how fast

Two computers. The **ground node** (a Jetson, power and weight are free) does the
heavy detection and builds the target *track*. The **air node** (Pi 5 + Hailo on the
drone) runs its own terminal seeker and the guidance. The link between them carries a
tiny **track message**, never video (that is the whole point of the split — Section 2).

### Ground node — Jetson Orin NX

```
2x global-shutter EO cameras (~2 m baseline)
      |  (each stream)
      v
[1] Small-object detector  (YOLOv8n/8s-class, TensorRT, run TWICE - once per stream)
      |
      v
[2] Stereo association + triangulation   (match the same drone in both views -> 3-D range)
      |
      v
[3] Track filter (Kalman)   ->  outputs filtered global-frame POSITION *and* VELOCITY
      |
      v
[4] Track message  ->  radio TX  at >=10 Hz
```

**The detector, honestly, run twice.** Peer-reviewed benchmark on Jetson Orin NX with
TensorRT: **YOLOv8n ~= 52 fps at 640x640 (FP16); YOLOv8s slower**, and
TensorRT cuts runtime 52-63% vs. plain PyTorch
([MDPI, *Computers* 2025, 15(2):74](https://www.mdpi.com/2073-431X/15/2/74)).
[Audit 2026-07-06: the paper tests FP16/FP32 only and names INT8 as *future work* —
the earlier "65 fps with INT8" figure is NOT supported by this source and is struck.
The load-bearing EXPECTED tier below uses the derated FP16 numbers, not INT8.] A
weaker Orin *Nano* gives YOLOv8n 16 ms / YOLOv8s 33 ms / YOLOv8m 50 ms per frame with
TensorRT ([arXiv 2409.16808](https://arxiv.org/html/2409.16808v1)) — a useful
pessimistic anchor. Newer YOLO26n on Orin NX 16GB is 4.13 ms FP16 / 3.49 ms INT8
*inference only* ([Ultralytics Jetson guide](https://docs.ultralytics.com/guides/nvidia-jetson/)).

Two honest deratings the datasheet fps hides:
- **Small objects need more pixels.** A drone at 100 m is a few pixels in a 640 crop.
  Real detection runs a larger input (~960-1280), which costs roughly 3-4x the
  compute — so a clean 52 fps single-stream drops toward ~12-15 fps.
- **Two streams share one GPU.** Running the detector on both cameras roughly halves
  what one stream gets (or you batch the two frames together — same total throughput).

Detector fps, three tiers, **per stream**:

| Tier | Detector fps (ground, per stream) | Basis |
|---|---|---|
| BEST | ~50-65 fps (YOLOv8n @640, INT8, one stream) | MDPI Orin NX |
| EXPECTED | **~15-20 fps** (YOLOv8s-class @~960-1280, small-object, two streams, detect-then-track front end) | MDPI derated for res + 2 streams |
| WORST-CREDIBLE | ~8-10 fps (thermal throttle at 25->15 W, contention) | derating; bench to confirm |

**This is fine, because of [3].** The Kalman filter *predicts* the target between
detections, so even a ~10 fps detector emits a smooth **>=10 Hz track** — and detection
mid-course is against a slow-changing, far target, the easy case. The mandatory output
is a **filtered velocity**, not just position: the project's own lab and Gazebo A/B
show emitting velocity vs. the drone differentiating a noisy position swings mean miss
**3.16 -> 1.01 m in the lab**; the paired Gazebo A/B confirms the direction with a
smaller effect (**EMIT 1.394 m vs. DIFF 1.808 m mean, Pk@2 75% vs. 50%**, n=8/arm,
worse-for-differentiate on 6/8 paired seeds — ADR-0015 2nd addendum). Velocity is a
few extra bytes; it is the #1 design lever in the whole perception half.

### Air node — Raspberry Pi 5 + Hailo-8/8L

```
Global-shutter mono camera (onboard)
      |
      v
[5] Detect (Hailo NPU)   - real ML, no ROS 2 (HailoRT Python API)
      |
      v
[6] Correlation tracker + alpha-beta/Kalman   (holds the lock through detection blinks)
      |
      v
[7] EKF fusion:  ground cue track  +  own camera bearing   ->  one target state
      |
      v
[8] Guidance (pro-nav / PIP) at 20 Hz  ->  MAVSDK  ->  TELEM2 UART  ->  PX4
```

**Verifying the "~35 fps / 29 ms" terminal claim (ADR-0015 / perception_design.md).**
The *raw* Hailo number is much faster: YOLOv11n on Pi 5 + Hailo-8 measures **104.9 fps
/ 7.75 ms hardware latency** over the Pi's single-lane PCIe Gen3 x1
([Hailo community official benchmark](https://community.hailo.ai/t/official-fps-benchmark-on-hailo-8-using-raspberry-pi-5/18873)),
and YOLOv8s on Pi 5 + Hailo-8L runs ~80-120 fps batched / ~107 fps ~= 8.4 ms
single-stream ([Seeed YOLOv8s on RPi5 + AI Kit](https://wiki.seeedstudio.com/benchmark_on_rpi5_and_cm4_running_yolov8s_with_rpi_ai_kit/)).
[Audit 2026-07-06: the previously-linked Seeed *multistream* page benchmarks YOLOv8**m**
(1-ch 77 fps), not YOLOv8s — corrected to the YOLOv8s page; if the exact 107 fps figure
can't be reconfirmed on the corrected page, treat it as a BEST-tier upper bound, not a
design number. The load-bearing end-to-end figure is the ~35 fps below, which stands.]
Two caveats keep us honest: the Pi 5 is **PCIe x1**, roughly *half* the host bandwidth
of Hailo's official x2 Model-Zoo numbers (same source); and those figures are the NPU
kernel alone. The **~29 ms / ~35 fps is the realistic *end-to-end*** figure once the
Pi CPU does capture + pre-processing + NMS post-processing + the detect-then-track
overhead. So:

| Tier | Terminal detect (air) | Basis |
|---|---|---|
| BEST | ~105 fps / ~8 ms (single model, NPU kernel) | Hailo community |
| EXPECTED | **~35 fps / ~29 ms end-to-end** (capture+pre+NMS+tracker on Pi CPU) | ADR-0015 / Hailo |
| WORST-CREDIBLE | ~15-20 fps (two streams — main + wide terminal cam — plus thermal) | derating |

**Hailo-8 vs. Hailo-8L:** the **8L = 13 TOPS** ($70 hat), the **8 = 26 TOPS** ($110
hat) ([Raspberry Pi AI HAT+](https://www.raspberrypi.com/news/raspberry-pi-ai-hat/)).
For one terminal detector the 8L clears >=30 Hz with ~2x margin. Recommend the
**Hailo-8** anyway: ADR-0015's acquisition-vs-terminal-FOV fix may add a *second*
wide-FOV terminal camera, and the 8 has the headroom to run two streams; the 8L does
not comfortably. Comfortably above the PX4 offboard **>=2 Hz "proof-of-life" minimum**
and the 20 Hz guidance loop ([PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard)).

---

## 2. Track message spec — and why the link carries it but never video

The ground node sends one small message per update. Global-frame position so the drone
can place it in its own map; **filtered velocity** because that is the #1 lever; a
covariance diagonal so the EKF knows how much to trust it; a **source-GPS timestamp**
so OOSM comp can age-correct it; a track id and a quality flag.

| Field | Type | Bytes |
|---|---|---|
| Source-GPS timestamp (us) | int64 | 8 |
| Track id | uint16 | 2 |
| Global position (lat, lon x1e7; alt mm) | 3x int32 | 12 |
| Filtered velocity (vN, vE, vD) | 3x float32 | 12 |
| Covariance diagonal (sigma^2: 3 pos + 3 vel) | 6x float32 | 24 |
| Quality flag + source id + seq | 3x uint8 | 3 |
| CRC | uint16 | 2 |
| **Payload total** | | **~63 bytes** |
| + framing (MAVLink-style header/overhead) | | ~12 |
| **On-the-wire total** | | **~75-90 bytes** |

**Bandwidth math.** ~90 bytes = **720 bits/message**.
- At **10 Hz -> 7.2 kbps**. At **20 Hz -> 14.4 kbps**.

Now compare to what the two candidate links carry:
- **SiK telemetry radio (cheap):** serial 57 600 baud; over-air *air rate* 64 kbps,
  and enabling error-correction **halves** it to ~32 kbps net
  ([ArduPilot SiK config](https://ardupilot.org/copter/docs/common-3dr-radio-advanced-configuration-and-technical-information.html);
  [SiK V3](https://holybro.com/products/sik-telemetry-radio-v3)). Track at 10 Hz uses
  **~22% of a 32 kbps SiK link**; at 20 Hz ~45%. Fits, with room to share the link
  with normal MAVLink telemetry at 10 Hz.
- **MANET radio (jam-resistant, expensive):** Doodle Labs Mesh Rider is rated up to
  **80-100 Mbps** ([Doodle Labs](https://doodlelabs.com/blog/mesh-rider-technology-introduction/)).
  A 7.2 kbps track is **~0.01%** of it.

**Video, for contrast:** even a compressed 720p30 H.265 stream is ~2-4 **Mbps**. That
is **60-125x a SiK link's entire capacity — impossible** — and even on MANET it hogs
the channel, is easy to detect, and dies when jammed. **Sending a 90-byte track
instead of pixels is the architecture**: the cheap link is *trivially* enough for a
track and *categorically* can't do video. The heavy vision stays on the machine that
saw the photons.

---

## 3. End-to-end latency budget (P-7) — three tiers, sourced

How stale is the target information by the time it moves an actuator? Split into two
segments: the **cue-delivery chain** (ground camera -> the drone has fused the track) —
this is what the sim's cue-latency knob models — and the **onboard control chain**
(guidance -> PX4), which the sim models *separately* via its 20 Hz loop and PX4's
first-order response.

Link row uses **SiK-class** for EXPECTED (the cheaper, more likely, pessimistic
choice); a low-latency MANET link would roughly halve that one row.

| # | Stage | BEST (ms) | EXPECTED (ms) | WORST (ms) | Source / basis |
|---|---|---:|---:|---:|---|
| 1 | Exposure (global shutter, short to freeze motion) | 0.5 | 2 | 5 | [Arducam OV9281](https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/Global-Shutter/1MP-OV9281-OV9282/) — programmable exposure |
| 2 | Readout + capture to host | 3 | 8 | 17 | OV9281 60 fps@1280x800 -> <=1 frame = 16.7 ms |
| 3 | Detect, ground (incl. pre/post) | 10 | 30 | 70 | MDPI Orin NX v8n ~19 ms; arXiv Nano v8s 33 ms |
| 4 | Stereo association + triangulation | 1 | 5 | 15 | linear triangulation on Jetson (derived) |
| 5 | Track filter (Kalman update) | 0.2 | 1 | 3 | derived (few-state filter) |
| 6 | Serialize + radio TX (ground->air) | 5 | **40** | 90 | SiK: 90 B x8 / 32 kbps ~= 22 ms airtime + TDM half-duplex slot ([SiK config](https://ardupilot.org/copter/docs/common-3dr-radio-advanced-configuration-and-technical-information.html)); MANET ~= 5-20 ms |
| 7 | Radio RX + parse (air) | 0.5 | 2 | 5 | derived (small packet) |
| 8 | OOSM latency compensation | 0.1 | 0.5 | 2 | compute only; it *removes* staleness, not adds |
| 9 | EKF fusion (cue + bearing) | 0.2 | 1 | 3 | derived |
| | **Cue-delivery subtotal (compare to sim 0.12 s)** | **~20** | **~90** | **~210** | sum of 1-9 |
| 10 | Guidance cycle (20 Hz quantization) | 5 | 25 | 50 | half-to-full 50 ms period |
| 11 | MAVSDK setpoint (serialize + TELEM2 921600 + resend) | 2 | 8 | 20 | MAVSDK resends @20 Hz ([MAVSDK Offboard](https://mavsdk.mavlink.io/main/en/cpp/guide/offboard.html)) |
| 12 | PX4 ingest -> controller accepts setpoint | 5 | 15 | 30 | [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard) (500 ms failsafe timeout) |
| | **Onboard control subtotal** | **~12** | **~48** | **~100** | sum of 10-12 |
| | **Grand total (photon->PX4 acts)** | **~32** | **~138** | **~310** | excludes vehicle dynamics |

*Note: the vehicle's physical acceleration response — motors spinning up, attitude
changing — is a further ~0.3 s first-order lag (tau ~= 0.3 s, the value `guidance_lab.py`
already uses). That is dynamics, not information latency, so it is kept out of this
table and modeled by PX4 itself.*

**Jitter.** The mean is only half the story. The dominant jitter sources are the radio
(row 6: MAC contention, retries, TDM slot phase) and the detect-frame phase (rows
2-3). EXPECTED jitter ~= **+/-30 ms**; WORST bursts to +/-80 ms **plus full dropouts** (the
bursty Markov outages ADR-0015 already added). Jitter, not mean, is what breaks a
velocity estimate — which is exactly why the cue *emits* filtered velocity.

### Does the sim's 0.12 s +/- 0.05 s model survive?

Compare against the **cue-delivery subtotal**, not the grand total (the sim's cue knob
is only the ground->drone channel; the onboard rows are modeled by the sim's own 20 Hz
loop + PX4 lag).

- **EXPECTED cue-delivery ~= 90 ms** (SiK link) — or ~70 ms with a MANET link.
- The sim uses **120 ms mean**. That sits *between* EXPECTED (90 ms) and WORST
  (210 ms): the sim is **slightly pessimistic** vs. a realistic SiK build, and clearly
  pessimistic vs. MANET. Good — pessimistic is honest; it is not "cheating" the result.
- The sim's **50 ms jitter** is a shade *generous* vs. the smooth part (~+/-30 ms) but is
  the right order once radio retries and frame-phase are included; the real tail is
  captured better by the **bursty Markov dropout** (already implemented) than by
  widening the Gaussian jitter.

**Verdict: the 0.12 s +/- 0.05 s model SURVIVES — it is realistic-to-mildly-pessimistic,
so any Pk it produces is not flattered by an optimistic latency.** Corrections to adopt:

1. **Keep 0.12 s as the EXPECTED mean.** It straddles SiK-realistic and MANET, leaning
   pessimistic — the safe choice.
2. **Add a 0.20 s WORST-CREDIBLE stress tier** (`--latency-s 0.20`) and report Pk under
   it. Right now the sim never tests the throttled/contended tail; it should.
3. **Keep bursty Markov dropout ON and keep `--latency-jitter-s 0.05`.** The realism
   comes from the *shape* of the tail (outages), not a bigger symmetric jitter.
4. **Keep OOSM comp (`--cue-latency-comp`) available** — with a real 40-90 ms link it
   earns more than at the sim's current noise floor (ADR-0013 found it inconclusive at
   n=1); M5's Monte-Carlo has the power to settle it.

---

## 4. Compute-split rationale (P-3) — why hybrid wins, with numbers

Four ways to divide the work. Latency, cost, jam-exposure, weight for each:

| Split | Cue-delivery latency | Drone cost/weight | Jam behavior | Verdict |
|---|---|---|---|---|
| **A. All-onboard** (drone does everything, no ground rig) | Lowest — no link (~29 ms terminal only) | Highest onboard load | **Immune** (no link to jam) | But **no mid-course cue** -> hover-start kinematic cap ~3 m/s (ADR-0011). Can only catch slow/close targets. |
| **B. Central ground** (Jetson does all; drone is a thin actuator) | Highest — full ground pipeline **+ link both ways every cycle** | Cheapest/lightest drone | **Catastrophic** — link dies -> drone blind | Rejected: contradicts the comms-denied headline outright. |
| **C. Per-camera edge** (each ground cam has its own compute) | Removes 2-stream contention (row 3) but adds an intra-ground hop | Unchanged drone | Same as B on the air side | Overkill for 2 cameras; extra box + sync for little gain. Fold into ground node instead. |
| **D. Hybrid (CHOSEN)** — ground detects/stereos/tracks -> compact track; drone does terminal detect+track+fusion+guidance, finishes comms-denied | Mid-course uses ground cue (~90 ms — fine, slow phase); **terminal uses onboard (~29 ms, NO link)** | Drone carries only Pi+Hailo+cam (~50-100 g), not a Jetson | **Bounded** — jamming costs only the mid-course aid; the decisive terminal is jam-immune by design | **Wins.** |

**Failure-mode row (link jammed) — the deciding column, tied to ADR-0015's data:**

| Split | What happens when the link is jammed |
|---|---|
| A. All-onboard | No effect (no link) — but was never able to engage fast/far targets. |
| B. Central ground | Total loss: the drone has no autonomy. A hard cut mid-terminal = crash/miss. |
| C. Per-camera edge | Same as B for the drone. |
| **D. Hybrid** | **Graceful.** Drone dead-reckons to the predicted basket, opens a bounded seeker search, acquires in the last ~10-15 m or breaks off. The lab quantifies the cost: a jammer link-cut at 11.5 m is **-26% Pk@2 and +0.78 m mean miss** — the single worst *individual* degradation (ADR-0015 addendum, constraint c5), and still recoverable because the terminal is onboard. Design rule: `R_acquire >= R_cutoff + coast_margin`. |

**Why hybrid wins in one line:** it puts the power-hungry, heavy compute where power and
weight are free (the ground), keeps the drone light and cheap (+$70-110 of Hailo), and
**isolates the one phase that must survive jamming (the terminal) onto the one machine
that can't be jammed off it (the drone itself).** Everything ADR-0015 and docs/goals.md
argue for.

---

## 5. Parts + prices (2026, researched) — with stale-BOM flags

### Ground node

| Part | Choice | Price (2026) | Source |
|---|---|---|---|
| Compute | reComputer J4012 — Orin NX **16GB** + carrier + 128 GB NVMe (integrated) | **~$899** | [Seeed J4012](https://www.seeedstudio.com/reComputer-J4012-p-5586.html); [JetsonHacks](https://jetsonhacks.com/2023/01/23/seeed-studio-recomputer-j4012-jetson-orin-nx/) |
| — cheaper option | reComputer **Super J4011** — Orin NX **8GB** (117 TOPS Super) | ~$599 (8GB class) | [Seeed Super J4011](https://www.seeedstudio.com/reComputer-Super-J4011-p-6445.html) |
| — bare module ref | Orin NX 16GB **module only** (no carrier) | ~$989 | [ThinkRobotics](https://thinkrobotics.com/products/nvidia-jetson-orin-nx-module) |
| EO cameras x2 | global-shutter, ~2 m baseline (higher-res than the onboard cam for 100 m range) | ~$150-250 each -> **~$400** | (OV9281 1 MP ~$60 is too low-res at 100 m; budget a 2-4 MP GS cam + lens) |
| RTK base + PPS | ArduSimple simpleRTK2B (ZED-F9P, has TIMEPULSE/PPS) | **~$197** + antenna ~$50 | [ArduSimple](https://www.ardusimple.com/product/simplertk2b/); [ZED-F9P](https://www.u-blox.com/en/product/zed-f9p-module) |
| — or | Holybro H-RTK F9P | ~$189-325 | [Holybro H-RTK](https://holybro.com/collections/h-rtk-gps) |
| Rig / mount | ~2 m baseline bar + tripods | ~$150-300 | (COTS) |
| Link (ground end) | SiK V3 telemetry (day-proof, not jam-resistant) | **~$50** | [SiK V3](https://holybro.com/products/sik-telemetry-radio-v3) |
| — jam-resistant upgrade | Doodle Labs Mesh Rider MANET | **~$1,500-3,000+** | [Doodle Labs](https://doodlelabs.com/products/) |

**Ground-node total (EO-only, day-proof, SiK link): ~$1,600** (16GB J4012) — or
**~$1,300** with the 8GB Super J4011 + budget cameras. Add **~$1,500-3,000** for a FLIR
Boson 640 thermal (night + bird-rejection), and **~$1,500-3,000** if you swap SiK for a
MANET link.

### Air node (delta over the existing ADR-0012 ~$800 drone)

| Part | Choice | Price | Source |
|---|---|---|---|
| NPU hat | Raspberry Pi AI HAT+ **26 TOPS (Hailo-8)** | **$110** | [RPi AI HAT+](https://www.raspberrypi.com/products/ai-hat/) |
| — minimum | AI HAT+ **13 TOPS (Hailo-8L)** | $70 | same |
| RTK rover + PPS | ZED-F9P (shares the ground base; time-syncs to GPS) | ~$189-200 | [ArduSimple](https://www.ardusimple.com/product/simplertk2b/) / [Holybro](https://holybro.com/collections/h-rtk-gps) |
| Link (air end) | SiK V3 (pairs with ground) | ~$50 | [SiK V3](https://holybro.com/products/sik-telemetry-radio-v3) |

### Stale-BOM flags against ADR-0015

- **[!] "Jetson Orin NX ($400-600)" is STALE for the 16GB part.** After the 2024-25
  "Super" refresh, the 16GB module is ~$989 bare / ~$899 integrated (reComputer J4012).
  The **$400-600 band now only covers the 8GB** part (Super J4011). Use **~$899 (16GB)**
  or **~$599 (8GB)** as the current figure.
- **[!] The link price is understated.** ADR-0015 lists "SiK/MANET ~$20-100" and
  perception_design.md "~$20-300" — that is fine for **SiK (~$50)** but **MANET is
  ~$1,500-3,000+** (Doodle Labs). The two are not interchangeable: SiK = cheap +
  day-proof; MANET = jam-resistant + expensive. Budget them as separate lines.
- **[ok] Hailo +$70-110 is accurate** (AI HAT+ 13 TOPS $70 / 26 TOPS $110) — no change.
- **[ok] RTK/PPS enablers are real and cheap** (~$200/end, ~$400 for the shared pair);
  PPS is built into the ZED-F9P (TIMEPULSE), no extra hardware.

---

## 6. Bench-validation plan — one measurement per latency row

Ties to ADR-0015 build-plan step 4 (the five-number bench test). Each row of the
Section 3 budget has one concrete thing you'd measure on a bench so a real number
replaces the estimate 1:1:

| # | Row | One-line bench measurement |
|---|---|---|
| 1 | Exposure | Log the sensor's exposure register; strobe an LED, read the captured pulse width. |
| 2 | Readout/capture | Timestamp a hardware trigger vs. frame-arrived-in-userspace on the Jetson. |
| 3 | Detect (ground) | Wrap the TensorRT `infer()` in a timer over 1000 frames at the real input res, **two streams live**; report p50/p95. |
| 4 | Stereo/triangulate | Timer around the association+triangulation call on real dual-camera frames. |
| 5 | Track filter | Timer around one Kalman predict+update. |
| 6 | Radio TX | Timestamp-at-source vs. timestamp-at-receive over the real link, clocks PPS-synced; histogram the one-way delay. |
| 7 | Radio RX + parse | Timer from bytes-available to struct-parsed on the Pi. |
| 8 | OOSM comp | Timer around the age-advance; separately, measure residual position error vs. truth with comp on/off. |
| 9 | EKF fusion | Timer around one EKF update on the Pi. |
| 10 | Guidance cycle | Log guidance-loop period (already logged as sim-time in CSVs); on hardware, wall-clock it. |
| 11 | MAVSDK setpoint | Timestamp `set_velocity_ned()` call vs. the MAVLink packet on TELEM2 (scope or logic analyzer). |
| 12 | PX4 response | PX4 ulog: setpoint-received vs. setpoint-acted timestamps. |

Plus the two ADR-0015 bench extras: a **two-clock PPS time-sync** check (are both GPS
clocks within ~1 us?) and a **static stereo-vs-mono range-accuracy** measurement at
50-150 m (does sigma_R actually grow like R^2?).

---

## 7. Proposed ADR-lite block (for docs/decisions.md)

> ## ADR-0016 — Compute setup: hybrid split, track-message link, sourced latency budget (P-3/P-5/P-7)
> - **Context:** Make ADR-0015's split concrete — pipeline, rates, message spec, and a
>   sourced end-to-end latency budget the sim can adopt and a bench can later replace 1:1.
> - **Options:** all-onboard / central-ground / per-camera-edge / **hybrid**.
> - **Decision:** **Hybrid.** Ground Jetson Orin NX runs a two-stream small-object
>   detector (YOLOv8n/8s-class, TensorRT ~15-20 fps/stream realistic) -> stereo
>   triangulation -> Kalman track emitting **filtered position AND velocity** at >=10 Hz.
>   Air Pi 5 + **Hailo-8 (26 TOPS, $110)** runs terminal detect (~35 fps/29 ms
>   end-to-end) + tracker + EKF fusion + 20 Hz guidance -> MAVSDK/TELEM2 -> PX4. Link
>   carries a **~90-byte track message** (7.2 kbps @10 Hz), never video.
> - **Why:** heavy compute where power/weight are free; a cheap link trivially carries a
>   track and categorically can't carry video; the jam-critical terminal is onboard =
>   jam-immune. Failure-mode: only hybrid degrades gracefully when jammed (dead-reckon +
>   seeker search), vs. total loss for central-ground.
> - **Latency finding:** EXPECTED cue-delivery ~= **90 ms** (SiK) / ~70 ms (MANET), WORST
>   ~210 ms. The sim's **0.12 s +/- 0.05 s cue model SURVIVES** — realistic-to-mildly-
>   pessimistic. Add a **0.20 s WORST stress tier**; keep bursty dropout + OOSM comp.
> - **BOM flags:** Orin NX 16GB is **~$899** now (ADR-0015's $400-600 is stale, was the
>   8GB/pre-Super price); **MANET ~= $1,500-3,000+**, far above the "$20-100" link line.
>   Hailo +$70-110 and RTK/PPS ~$400/pair confirmed. **Ground node ~$1,600 EO-only.**
> - **Date:** 2026-07-05. (No council — synthesis of ADR-0012/0015 + 2026 sourcing;
>   reversible; every number bench-measurable per Section 6. [Date corrected from
>   2026-07-06 to match ADR-0016 in decisions.md; committed 2026-07-05 evening.])
