# NN-tier compute: GPU training is UP on this box (2026-07-20)

**Verdict: CUDA training works.** A separate GPU venv now exists; use it for all
NN-tier fine-tunes. The working CPU venv is untouched and remains the fallback.

## The environment to use

- **Train (GPU): `/home/emerson/interceptor-sim/.venv-seeker-train-gpu/bin/python`**
  - torch **2.12.1+cu126** + torchvision 0.27.1+cu126 (exact version match to the CPU
    venv's 2.12.1, so behavior parity), ultralytics **8.4.90** (same pin as CPU venv).
  - Verified: `torch.cuda.is_available()=True`, matmul runs on
    `NVIDIA GeForce RTX 4070 Laptop GPU`, CUDA runtime 12.6 under driver 573.09
    (driver CUDA 12.8 → backward-compatible).
- Train (CPU fallback): `/home/emerson/interceptor-sim/.venv-seeker-train/bin/python`
  (torch 2.12.1+cpu — unchanged).
- Inference/eval stays in `.venv-seeker` (onnxruntime, cv2) as before.
- Download cost: **~3.6 GB** of wheels (torch cu126 wheel alone 843.5 MB + NVIDIA
  runtime libs); venv is **6.7 GB on disk**. Builder pre-authorized ("use any compute").

Why cu126 and not cu124/cu128: the cu126 index is the only one carrying
**torch 2.12.1** exactly (cu124 tops out at 2.6.0, cu128 at 2.11.0). Matching the CPU
venv's torch version eliminates a version-skew confound between GPU and CPU runs.

## Benchmark (yolo11n, imgsz=640, COCO-init, 300 train / 60 val frames of quad_dataset_v2)

Method: 2 epochs per config, per-epoch wall time from ultralytics epoch callbacks;
epoch 1 includes warmup/JIT so **epoch 2 is the steady-state number**. Peak VRAM from
`torch.cuda.max_memory_reserved`. Run 2026-07-20, idle box (only the Windows desktop
holding ~1.1 GB VRAM). JSONs: scratchpad `bench_gpu_b16/b32/b64.json`, `bench_cpu_b16.json`.

| Config | steady s/epoch | peak VRAM reserved |
|---|---|---|
| GPU batch 16 | **2.77 s** | 2.43 GB |
| GPU batch 32 | **2.58 s** | 4.21 GB |
| GPU batch 64 | 10.24 s (!) | **8.54 GB — oversubscribed** |
| CPU batch 16 | **~65 s** (61.8 / 68.3) | n/a |

- **GPU speedup ≈ 23x** (65 → 2.77 s/epoch, paired on the identical dataset).
  A 100-epoch fine-tune on a few-thousand-image set drops from overnight-CPU to
  ~15-30 min GPU. (The remembered "~44 s/epoch" CPU figure was a different-sized set;
  this table is the like-for-like pair.)
- Epoch time scales ~linearly with image count; numbers above are per-300-images.

## Safe batch / imgsz for this 8 GB card — and the WDDM spill trap

- **Recommended: `batch=32, imgsz=640`** → 4.21 GB reserved, ~2.6 GB headroom under
  the ~6.9 GB actually free (8.00 GB total minus ~1.1 GB Windows desktop).
- **Drop to `batch=16`** (2.43 GB) whenever anything else uses the GPU — notably the
  Gazebo GPU render path (ADR-0075 d3d12) if a sim is flying concurrently.
- **Do NOT use batch 64 @640.** On WSL2/WDDM there is **no hard CUDA OOM**: the
  allocator silently spills into shared system memory. Batch 64 "succeeded" at 8.54 GB
  reserved but ran **4x slower** than batch 32. Failure mode = no error, just a
  mysteriously slow epoch — if s/epoch jumps way above the table, suspect VRAM spill
  and cut the batch.
- imgsz above 640 costs ~quadratically in VRAM; if a 1280-class experiment is ever
  wanted, probe batch 4-8 first and watch reserved VRAM vs the spill trap.

## Standing rules that still apply

- ONE training at a time on this box (single GPU; concurrent runs thrash/spill).
- Long trainings stay reaper-safe: setsid-detached daemon + checkpoints + resume
  (~51 min background reap), poll by log staleness — GPU speed makes most fine-tunes
  fit inside one window, but keep the pattern for the big real-media runs.
- The GPU offloads only training/render; CPU dataloader workers can bottleneck very
  large datasets — raise `workers` before blaming the GPU.
