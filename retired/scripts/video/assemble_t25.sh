#!/usr/bin/env bash
# =============================================================================
# assemble_t25.sh -- ffmpeg stitcher for the T25 demo video
# (docs/t25_storyboard.md, 9 shots, target ~60-75 s).
#
# OFFLINE: no sim, no GPU render -- it only stitches shot clips/cards that
# already exist. Designed so shots can be dropped in AS THEY ARE RENDERED:
# a missing shot prints a loud SKIP and the cut is assembled from whatever is
# present (use --require-all for the final publish pass, which fails instead).
#
# SHOT LIST FILE (default scripts/video/t25_shots.conf), one line per shot:
#     id|type|source|opts
#   type=card  source is a PNG; opts: dur=SECONDS (default 4)
#   type=clip  source is an .mp4 OR a directory of frame_%06d.png
#              opts: speed=F  (setpts factor: 1=as-is, 2=half speed...)
#              NOTE: slow-mo is normally baked upstream by hud_overlay.py
#              frame pacing WITH its on-screen "SLOW MOTION (retimed)"
#              watermark. If you retime HERE instead (speed != 1), this
#              script auto-burns the same watermark -- honesty constraint #4
#              (disclose retiming on screen) is enforced by construction:
#              there is no code path that slows a clip without the label.
#
# USAGE:
#   scripts/video/assemble_t25.sh [--shots FILE] [--out FILE] [--fps 30]
#                                 [--size 1920x1080] [--require-all]
# Output default: demo_out/t25/t25_demo.mp4  (+ per-shot normalized clips in
# demo_out/t25/build/ for re-edits).
# =============================================================================
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SHOTS_FILE="$REPO_ROOT/scripts/video/t25_shots.conf"
OUT="$REPO_ROOT/demo_out/t25/t25_demo.mp4"
FPS=30
SIZE="1920x1080"
REQUIRE_ALL=0
FONT="/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"
SLOWMO_TEXT="SLOW MOTION (retimed)"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --shots) SHOTS_FILE="$2"; shift 2;;
        --out) OUT="$2"; shift 2;;
        --fps) FPS="$2"; shift 2;;
        --size) SIZE="$2"; shift 2;;
        --require-all) REQUIRE_ALL=1; shift;;
        *) echo "[assemble_t25] unknown arg '$1'" >&2; exit 2;;
    esac
done

command -v ffmpeg >/dev/null 2>&1 || { echo "[assemble_t25] ffmpeg not installed"; exit 3; }
[[ -f "$SHOTS_FILE" ]] || { echo "[assemble_t25] shots file not found: $SHOTS_FILE"; exit 2; }
[[ -f "$FONT" ]] || FONT="/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"

VW="${SIZE%x*}"; VH="${SIZE#*x}"
BUILD_DIR="$(dirname "$OUT")/build"
mkdir -p "$BUILD_DIR"
LIST_FILE="$BUILD_DIR/concat_list.txt"
: > "$LIST_FILE"

# scale to fit, pad to exact frame, uniform fps/pixfmt -> concat-safe clips
NORM_VF="scale=${VW}:${VH}:force_original_aspect_ratio=decrease,pad=${VW}:${VH}:(ow-iw)/2:(oh-ih)/2:color=black,fps=${FPS},format=yuv420p"

n_ok=0; n_skip=0; missing=""

while IFS='|' read -r id type source opts; do
    # strip comments/blank lines
    [[ -z "${id// }" || "${id:0:1}" == "#" ]] && continue
    id="$(echo "$id" | tr -d ' ')"
    type="$(echo "$type" | tr -d ' ')"
    source="$(echo "$source" | sed 's/^ *//; s/ *$//')"
    opts="${opts:-}"
    # relative sources resolve against the repo root
    [[ "$source" != /* ]] && source="$REPO_ROOT/$source"

    if [[ ! -e "$source" ]]; then
        echo "[assemble_t25] SHOT $id MISSING -> SKIPPED: $source"
        n_skip=$((n_skip + 1)); missing="$missing $id"
        continue
    fi

    dur=4; speed=1
    for kv in $opts; do
        case "$kv" in
            dur=*) dur="${kv#dur=}";;
            speed=*) speed="${kv#speed=}";;
        esac
    done

    clip="$BUILD_DIR/shot${id}.mp4"
    echo "[assemble_t25] shot $id ($type): $(basename "$source") -> $(basename "$clip")"

    case "$type" in
        card)
            ffmpeg -nostdin -y -loop 1 -t "$dur" -i "$source" \
                -vf "$NORM_VF" -an -c:v libx264 -crf 19 -preset medium \
                "$clip" -loglevel error || { echo "[assemble_t25] FFMPEG FAILED on shot $id"; exit 1; }
            ;;
        clip)
            # directory => frame_%06d.png sequence; file => any ffmpeg-readable video
            if [[ -d "$source" ]]; then
                input=(-framerate "$FPS" -i "$source/frame_%06d.png")
            else
                input=(-i "$source")
            fi
            vf="$NORM_VF"
            if [[ "$speed" != "1" && "$speed" != "1.0" ]]; then
                # retiming here => the disclosure watermark is burned in, always
                # (honesty constraint #4). Prefer baking slow-mo upstream in
                # hud_overlay.py, which stamps the same label per-frame.
                # top-centre = the same spot hud_overlay.py's own pill uses
                # (that spot is empty when retiming happens here, so the two
                # disclosure paths can never stack into a double label)
                vf="setpts=${speed}*PTS,${NORM_VF},drawtext=fontfile=${FONT}:text='${SLOWMO_TEXT}':fontcolor=white@0.9:fontsize=$((VH/28)):box=1:boxcolor=black@0.45:boxborderw=10:x=(w-text_w)/2:y=h/9"
                echo "[assemble_t25]   retimed x${speed} -- '${SLOWMO_TEXT}' watermark burned in"
            fi
            ffmpeg -nostdin -y "${input[@]}" -vf "$vf" -an -c:v libx264 -crf 19 -preset medium \
                "$clip" -loglevel error || { echo "[assemble_t25] FFMPEG FAILED on shot $id"; exit 1; }
            ;;
        *)
            echo "[assemble_t25] SHOT $id: unknown type '$type' (want card|clip)"; exit 2;;
    esac

    echo "file '$clip'" >> "$LIST_FILE"
    n_ok=$((n_ok + 1))
done < "$SHOTS_FILE"

if [[ $n_ok -eq 0 ]]; then
    echo "[assemble_t25] FAIL: no shots available."; exit 1
fi
if [[ $REQUIRE_ALL -eq 1 && $n_skip -gt 0 ]]; then
    echo "[assemble_t25] FAIL (--require-all): missing shot(s):$missing"; exit 1
fi

# uniform clips (same codec/size/fps/pixfmt) => stream-copy concat is safe
ffmpeg -nostdin -y -f concat -safe 0 -i "$LIST_FILE" -c copy "$OUT" -loglevel error \
    || { echo "[assemble_t25] concat failed"; exit 1; }

dur_s="$(ffprobe -v error -show_entries format=duration -of default=nw=1:nk=1 "$OUT" 2>/dev/null)"
echo "[assemble_t25] ================================================"
echo "[assemble_t25] wrote $OUT  (${dur_s%.*}s, $n_ok shots stitched, $n_skip skipped${missing:+ --$missing})"
[[ $n_skip -gt 0 ]] && echo "[assemble_t25] re-run after rendering the missing shots to produce the full cut."
echo "[assemble_t25] storyboard target: ~60-75 s total (docs/t25_storyboard.md)"
exit 0
