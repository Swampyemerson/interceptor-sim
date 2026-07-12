# shellcheck shell=bash
# GPU RENDER for Gazebo camera sensors — source this before booting a headless sim.
#
# Renders the gz camera sensors on the RTX 4070 via the WSL2 D3D12 path instead of the
# llvmpipe SOFTWARE rasterizer (the WSL default, which left the GPU idle). MEASURED win
# (ADR-0075): with a consumed onboard camera, RTF 0.336 (llvmpipe) -> 0.951 (d3d12), the
# camera hitting its full 30 Hz, GPU util 12-31%. Unblocks faster batches/captures/demo
# and higher-res (incl. 4K) cameras (ADR-0074).
#
# HOW: exports GALLIUM_DRIVER=d3d12 (+ the NVIDIA adapter) into the shell, so the
# inherited `env PX4_GZ_WORLD=... HEADLESS=1 make px4_sitl gz_x500_mono_cam` boot picks
# it up (env(1) adds to, never resets, the inherited environment).
#
# TOGGLE: SIM_GPU_RENDER=0 forces the software (llvmpipe) render (e.g. if d3d12 flakes on
# WSL, the known caveat). Auto-falls-back to software if the d3d12 Mesa driver is absent.
#
# HONESTY/REPRO NOTE (ADR-0075): GPU vs software render can differ at the pixel level, so
# any headline Pk/detection number re-measured under GPU render must be validated against
# (or re-baselined vs) the software-render baseline — treat a render-backend switch like a
# sensor change (ADR-0010 discipline). Default-ON for SPEED; the equivalence check folds
# into the next Pk batch.

if [ "${SIM_GPU_RENDER:-1}" = "1" ] && [ -e /usr/lib/x86_64-linux-gnu/dri/d3d12_dri.so ]; then
    export GALLIUM_DRIVER=d3d12
    export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
    export __SIM_GPU_RENDER_ACTIVE=1
else
    export __SIM_GPU_RENDER_ACTIVE=0
fi
