#!/usr/bin/env bash
# probe_stereo_rtf.sh — T16-pre feasibility probe (Phase 2 plan §5.5).
#
# QUESTION: can the sim carry a two-camera ground stereo rig LIVE alongside the
# onboard camera, and at what resolution/rate — or does RTF collapse the way the
# single extra 1280x720 demo camera did (RTF ~0.05, worlds/apriltag_demo.sdf:300)?
# Also settles whether headless rendering runs on llvmpipe (CPU) or the 4070,
# and whether GALLIUM_DRIVER=d3d12 buys the GPU back (px4-gazebo skill note:
# "rendering warnings = llvmpipe").
#
# METHOD: for each config, generate a stereo_probe world (markerless.sdf +
# inline static rig model, 2.0 m baseline, ADR-0017 20-deg HFOV), boot the full
# PX4+gz stack headless, measure steady-state RTF from /world/stereo_probe/stats
# twice — ISOLATED (no image subscribers) and LOADED (gz topic echo draining
# every camera topic, approximating client transport cost) — while sampling
# nvidia-smi. ONE sim at a time, sequential configs, idle machine only
# (ADR-0015 load confound). Results: logs/stereo_rtf_probe/results.csv.
#
# RTF floors for reading the CSV: ~1.0 = healthy idle; ADR-0009 documents
# ~0.3-0.5 as the "under load" floor where sim-time scheduling still works;
# the 0.05 demo collapse broke MAVSDK OFFBOARD entry outright.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="$HOME/PX4-Autopilot"
WORLD_NAME="stereo_probe"
WORLDS_LINK_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
OUT_DIR="$REPO/logs/stereo_rtf_probe"
GEN_DIR="$OUT_DIR/gen"
CSV="$OUT_DIR/results.csv"
BOOT_TIMEOUT_S=240
SETTLE_S=8
SAMPLE_S=25
COOLDOWN_S=8

# label:width:height:rate:extra_env  (width 0 = control, no rig)
DEFAULT_CONFIGS=(
  "control:0:0:0:"
  "r1920x1200_5hz:1920:1200:5:"
  "r1920x1200_10hz:1920:1200:10:"
  "r1920x1200_30hz:1920:1200:30:"
  "r1280x960_10hz:1280:960:10:"
  "r960x540_10hz:960:540:10:"
  "r960x540_30hz:960:540:30:"
  "r640x480_10hz:640:480:10:"
  "r1920x1200_10hz_d3d12:1920:1200:10:GALLIUM_DRIVER=d3d12"
  "control_d3d12:0:0:0:GALLIUM_DRIVER=d3d12"
)
CONFIGS=("${@:-}")
if [ -z "${1:-}" ]; then CONFIGS=("${DEFAULT_CONFIGS[@]}"); fi

mkdir -p "$OUT_DIR" "$GEN_DIR"
SUB_PIDS=()
GPU_PID=""

cleanup() {
  for p in "${SUB_PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  [ -n "$GPU_PID" ] && kill "$GPU_PID" 2>/dev/null
  bash "$REPO/scripts/sim_kill.sh"
}
trap cleanup EXIT

gen_world() { # $1=width $2=height $3=rate ; 0-width = no rig
  local w="$1" h="$2" rate="$3"
  local rig=""
  if [ "$w" != "0" ]; then
    rig=$(cat <<EOF

    <!-- GENERATED probe rig: 2 cameras, 2.0 m baseline (ADR-0017), 20-deg HFOV
         (stereo_design.md 16 mm design point), broadside of the dash corridor. -->
    <model name="ground_stereo_rig_probe">
      <static>true</static>
      <pose>20 -14 1.5 0 0 3.14159</pose>
      <link name="link">
        <inertial><mass>1</mass>
          <inertia><ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz></inertia>
        </inertial>
        <sensor name="rig_left" type="camera">
          <pose>0 1.0 0 0 0 0</pose>
          <camera>
            <horizontal_fov>0.349</horizontal_fov>
            <image><width>$w</width><height>$h</height></image>
            <clip><near>0.1</near><far>3000</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>$rate</update_rate>
        </sensor>
        <sensor name="rig_right" type="camera">
          <pose>0 -1.0 0 0 0 0</pose>
          <camera>
            <horizontal_fov>0.349</horizontal_fov>
            <image><width>$w</width><height>$h</height></image>
            <clip><near>0.1</near><far>3000</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>$rate</update_rate>
        </sensor>
      </link>
    </model>
EOF
)
  fi
  # markerless.sdf, renamed, rig injected before </world>
  awk -v rig="$rig" '
    { gsub(/<world name="markerless">/, "<world name=\"'"$WORLD_NAME"'\">") }
    /<\/world>/ { print rig }
    { print }
  ' "$REPO/worlds/markerless.sdf" > "$GEN_DIR/$WORLD_NAME.sdf"
  ln -sf "$GEN_DIR/$WORLD_NAME.sdf" "$WORLDS_LINK_DIR/$WORLD_NAME.sdf"
}

rtf_stats() { # $1=seconds ; echoes "mean min n"
  timeout "$1" gz topic -e -t "/world/$WORLD_NAME/stats" 2>/dev/null \
    | awk '/real_time_factor:/ {v=$2; s+=v; n++; if(min==""||v<min)min=v}
           END { if(n>0) printf "%.4f %.4f %d\n", s/n, min, n; else print "nan nan 0" }'
}

echo "label,width,height,rate,extra_env,boot_ok,rtf_isolated_mean,rtf_isolated_min,rtf_isolated_n,rtf_loaded_mean,rtf_loaded_min,rtf_loaded_n,gpu_util_max_pct,gpu_mem_max_mib,renderer_hint" > "$CSV"

for cfg in "${CONFIGS[@]}"; do
  IFS=':' read -r LABEL W H RATE EXTRA_ENV <<< "$cfg"
  echo "=== [$LABEL] rig=${W}x${H}@${RATE} env='${EXTRA_ENV}' ==="
  bash "$REPO/scripts/sim_kill.sh"; sleep "$COOLDOWN_S"

  gen_world "$W" "$H" "$RATE"
  BOOT_LOG="$OUT_DIR/boot_$LABEL.log"

  ( cd "$PX4_DIR" && env $EXTRA_ENV PX4_GZ_WORLD="$WORLD_NAME" \
      GZ_SIM_RESOURCE_PATH="$REPO/models" HEADLESS=1 \
      make px4_sitl gz_x500_mono_cam > "$BOOT_LOG" 2>&1 ) &
  MAKE_PID=$!

  BOOT_OK=0
  for _ in $(seq 1 "$BOOT_TIMEOUT_S"); do
    grep -q "Startup script returned successfully" "$BOOT_LOG" 2>/dev/null && { BOOT_OK=1; break; }
    kill -0 "$MAKE_PID" 2>/dev/null || break
    sleep 1
  done

  ISO="nan nan 0"; LOAD="nan nan 0"; GPU_MAX="0"; MEM_MAX="0"; RHINT="none"
  if [ "$BOOT_OK" = "1" ]; then
    sleep "$SETTLE_S"
    GPU_CSV="$OUT_DIR/gpu_$LABEL.csv"; : > "$GPU_CSV"
    ( while true; do nvidia-smi --query-gpu=utilization.gpu,memory.used \
        --format=csv,noheader,nounits >> "$GPU_CSV" 2>/dev/null; sleep 2; done ) &
    GPU_PID=$!

    ISO=$(rtf_stats "$SAMPLE_S")

    # loaded phase: drain every camera image topic like a client would
    SUB_PIDS=()
    while IFS= read -r t; do
      gz topic -e -t "$t" > /dev/null 2>&1 &
      SUB_PIDS+=($!)
    done < <(gz topic -l 2>/dev/null | grep -E "/world/$WORLD_NAME/.*(imager|rig_left|rig_right)/image$")
    sleep 2
    LOAD=$(rtf_stats "$SAMPLE_S")
    for p in "${SUB_PIDS[@]:-}"; do kill "$p" 2>/dev/null; done; SUB_PIDS=()

    kill "$GPU_PID" 2>/dev/null; GPU_PID=""
    GPU_MAX=$(awk -F', *' '{if($1>m)m=$1} END{print m+0}' "$GPU_CSV")
    MEM_MAX=$(awk -F', *' '{if($2>m)m=$2} END{print m+0}' "$GPU_CSV")
    RHINT=$(grep -io -m1 "llvmpipe[^\"]*\|d3d12[^\"]*\|nvidia [a-z0-9 ]*\|OGRE2* is using [^\"]*" "$BOOT_LOG" | head -c 60 | tr ',' ';')
    [ -z "$RHINT" ] && RHINT="none"
  else
    echo "  BOOT FAILED (see $BOOT_LOG tail):"; tail -3 "$BOOT_LOG" | sed 's/^/    /'
  fi

  read -r IM IN INN <<< "$ISO"; read -r LM LN LNN <<< "$LOAD"
  echo "$LABEL,$W,$H,$RATE,$EXTRA_ENV,$BOOT_OK,$IM,$IN,$INN,$LM,$LN,$LNN,$GPU_MAX,$MEM_MAX,$RHINT" >> "$CSV"
  echo "  isolated RTF mean=$IM min=$IN (n=$INN) | loaded mean=$LM min=$LN (n=$LNN) | GPU max ${GPU_MAX}% ${MEM_MAX}MiB | $RHINT"

  bash "$REPO/scripts/sim_kill.sh"
done

echo; echo "=== RESULTS ($CSV) ==="; column -s, -t "$CSV"
