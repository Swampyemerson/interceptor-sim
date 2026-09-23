#!/usr/bin/env bash
# compose_demo.sh — turn a logged intercept flight into the portfolio DEMO VIDEO.
#
# Two products from one flight CSV:
#   (1) a data-driven STATUS HUD panel (scripts/render_hud.py) — phase, the
#       external-cue -> camera-only handoff flip, closing speed / range / LOS-rate,
#       and a top-down map with the lethal-radius closest-approach marker; and
#   (2) an MP4 + a README-friendly GIF, either HUD-only (works today) or the HUD
#       composited BESIDE a Gazebo GUI render of the same flight (the full demo).
#
# WHY THIS IS A SCRIPT: it is meant to be ONE command and reproducible (project
# convention). It is safe to run repeatedly; outputs go to a timestamp-free dir
# so re-runs overwrite.
#
# PREREQUISITE (the one thing a human must do): ffmpeg. It is NOT installed in
# this VM by default. Install it once:
#     sudo apt install -y ffmpeg
# (from the interactive prompt: `! sudo apt install -y ffmpeg`). Everything else
# — the HUD frames, the flight log — is already produced by the sim pipeline.
#
# USAGE:
#   scripts/compose_demo.sh [FLIGHT_CSV] [GAZEBO_FRAMES_DIR]
#     FLIGHT_CSV         a per-tick m4_intercept CSV with DASH+HANDOFF+ENGAGE
#                        (default: the dash-track-fix hero flight, 0.58 m @ 9 m/s).
#     GAZEBO_FRAMES_DIR  optional dir of numbered PNGs from a GUI screen-capture /
#                        the gz camera topic (see scripts/sim_gui.sh). If given,
#                        the HUD is composited to its RIGHT; if omitted, a HUD-only
#                        video is produced (still a real deliverable).
set -uo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_PY="$REPO_ROOT/.venv/bin/python"
FLIGHT="${1:-$REPO_ROOT/logs/m4_intercept_pronav_20260707T000735Z.csv}"
GZ_FRAMES="${2:-}"
FPS=30
RADIUS=1.5               # net lethal radius (ADR-0025) drawn on the map
OUT_DIR="$REPO_ROOT/demo_out"
HUD_DIR="$OUT_DIR/hud_frames"

# --- preflight ---
command -v ffmpeg >/dev/null 2>&1 || {
    echo "[compose_demo] ffmpeg is NOT installed — this is the only blocker."
    echo "[compose_demo] Install it once, then re-run this script:"
    echo "[compose_demo]     sudo apt install -y ffmpeg      (or:  ! sudo apt install -y ffmpeg )"
    exit 3
}
[ -f "$FLIGHT" ] || { echo "[compose_demo] flight CSV not found: $FLIGHT"; exit 1; }
[ -x "$VENV_PY" ] || { echo "[compose_demo] venv python missing: $VENV_PY"; exit 1; }

mkdir -p "$OUT_DIR"; rm -rf "$HUD_DIR"; mkdir -p "$HUD_DIR"

# --- (1) HUD frames from the CSV ---
echo "[compose_demo] rendering HUD frames from $(basename "$FLIGHT") ..."
"$VENV_PY" "$REPO_ROOT/scripts/render_hud.py" "$FLIGHT" --out "$HUD_DIR" --fps "$FPS" --radius "$RADIUS" || {
    echo "[compose_demo] render_hud.py failed"; exit 1; }

MP4="$OUT_DIR/interceptor_demo.mp4"
GIF="$OUT_DIR/interceptor_demo.gif"

# --- (2) encode ---
if [ -n "$GZ_FRAMES" ] && [ -d "$GZ_FRAMES" ]; then
    echo "[compose_demo] compositing Gazebo render (left) + HUD (right) ..."
    # scale both to a common height, hstack; assumes GZ_FRAMES holds frame_%06d.png
    ffmpeg -y -framerate "$FPS" -i "$GZ_FRAMES/frame_%06d.png" \
           -framerate "$FPS" -i "$HUD_DIR/frame_%06d.png" \
           -filter_complex "[0:v]scale=-1:1080[a];[1:v]scale=-1:1080[b];[a][b]hstack=inputs=2,format=yuv420p" \
           -c:v libx264 -crf 20 "$MP4" || { echo "[compose_demo] ffmpeg composite failed"; exit 1; }
else
    echo "[compose_demo] no Gazebo frames given -> HUD-only video (still shippable)."
    ffmpeg -y -framerate "$FPS" -i "$HUD_DIR/frame_%06d.png" \
           -vf "format=yuv420p" -c:v libx264 -crf 20 "$MP4" || { echo "[compose_demo] ffmpeg HUD encode failed"; exit 1; }
fi

# README-friendly GIF (palette for quality, 12 fps, 720 px tall)
echo "[compose_demo] making GIF ..."
ffmpeg -y -i "$MP4" -vf "fps=12,scale=-1:720:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" "$GIF" \
    || echo "[compose_demo] GIF step failed (MP4 is still fine)"

echo "[compose_demo] DONE:"
echo "  video: $MP4"
echo "  gif:   $GIF"
echo "  (add demo_out/ to .gitignore or commit the GIF into docs/images/ for the README)"
